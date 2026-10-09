// Binding of the dryer: one instance on the heater and the airflow bindings' instances, its log a
// working copy of the config record's; the inputs gathered in the tick from the world heat's tick
// read, the chamber and the link; the report, the events and the log store from a task (sendf and
// the flash never run in timer context); the commands, each core call bracketed with interrupts
// masked (the tick runs the dryer in the timer interrupt).  A refused command is answered with its
// reason, never shut down: a range, a fault, a missing mains are conditions of the unit.  A log
// index outside the dictionary's range is a host bug and takes Klipper's way out.
#include "dryer/dryer_cmds.h"
#include "heat/airflow_cmds.h" // ace2k_airflow_binding
#include "core/config_cmds.h"  // ace2k_config_binding, ace2k_config_binding_store
#include "env/env_cmds.h"      // ace2k_env_binding
#include "core/health.h"       // ACE2K_RESET_POWER_ON
#include "heat/heat_cmds.h"    // ace2k_heat_binding, ace2k_heat_binding_world, the after hook
#include "core/report.h"
#include "lane/sensors.h"        // ACE2K_SENSORS_STATE_INSERT_SHIFT
#include "core/tx.h"             // ACE2K_SENDF
#include "ace2k_board/serial.h"  // ace2k_link_ok
#include "ace2k_board/sysinfo.h" // ace2k_sysinfo_reset_cause
#include "ace2k_board/tick.h"    // ace2k_tick_now_ms
#include "board/irq.h"           // irq_save, irq_restore
#include "board/misc.h"          // timer_from_us
#include "command.h"             // DECL_COMMAND, DECL_CONSTANT, sendf, shutdown
#include "sched.h"               // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task

// The phase (report.h): sensors 0, airflow 10, env 30, lane 50, rfid 60, mains 70, feed 80,
// dryer 90.
#define ACE2K_DRYER_REPORT_PHASE_MS        90U
#define ACE2K_DRYER_EVENT_FMT              "ace2k_dryer_event kind=%c arg=%c seq=%c"
#define ACE2K_DRYER_LOG_FMT                "ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u"
#define ACE2K_DRYER_LOG_FIELDS             6U
#define ACE2K_DRYER_LOG_ENTRY_SHIFT_REASON 8U
#define ACE2K_DRYER_LOG_ENTRY_SHIFT_CYCLE  16U
#define ACE2K_DRYER_INSERT_MASK            0xFU
#define ACE2K_DRYER_AGE_NEVER_MS           UINT32_MAX
// After a store that failed on the flash (not a refusal for a lease): no retry before this long
// has passed, doubling at each failure up to the cap — a page that cannot be written is not
// erased a hundred times a second, and the same log is tried again (a cutout marked once must
// still reach the page: rule 10).
#define ACE2K_DRYER_STORE_BACKOFF_MIN_MS 1000U
#define ACE2K_DRYER_STORE_BACKOFF_MAX_MS 60000U

// The host checks the arguments against these before it sends (ace2k_dryer.py).
DECL_CONSTANT("ACE2K_DRYER_TARGET_MIN_C", ACE2K_DRYER_TARGET_MIN_C);
DECL_CONSTANT("ACE2K_DRYER_TARGET_MAX_C", ACE2K_DRYER_TARGET_MAX_C);
DECL_CONSTANT("ACE2K_DRYER_MINUTES_MAX", ACE2K_DRYER_MINUTES_MAX);

static struct ace2k_dryer ace2k_dryer_instance;
static struct ace2k_dryer_log ace2k_dryer_log_work; // the tick's; copied to the record to store
static struct ace2k_config *ace2k_dryer_cfg;
static const struct ace2k_env *ace2k_dryer_env;
static const struct ace2k_heat_world *ace2k_dryer_world;
static struct task_wake ace2k_dryer_wake;
static struct ace2k_report ace2k_dryer_report;
static volatile bool ace2k_dryer_report_due;
static volatile bool ace2k_dryer_subscribed; // the host queried once: the events may leave
// The events' resend: the task writes last_send_ms, the tick reads it; the tick (5 s since the
// last send) and the query set resend_due, the task clears it.  Single aligned words.
static volatile bool ace2k_dryer_resend_due;
static volatile uint32_t ace2k_dryer_last_send_ms;
static bool ace2k_dryer_ready;
// The store's back-off (task writes, tick reads; single aligned words).
static volatile bool ace2k_dryer_store_failed;       // the last store failed on the flash
static volatile uint32_t ace2k_dryer_store_retry_ms; // no retry before this
static uint32_t ace2k_dryer_store_backoff_ms = ACE2K_DRYER_STORE_BACKOFF_MIN_MS;

enum dryer_op { OP_START = 0, OP_STOP = 1, OP_CLEAR = 2 };

// The chamber's RH is a reading with the chamber's: valid, and within the sensor's full scale.
static bool rh_valid(const struct ace2k_env *env, bool chamber_valid)
{
    if (!chamber_valid) {
        return false;
    }
    return env->chamber_rh_pct10 <= ACE2K_DRYER_RH_FULL_PCT10;
}

// The world heat's tick read (the NTCs held at their last valid value while invalid, the mains,
// the switches), the chamber and the link.  The chamber's fields are written by the env task: a
// value read torn costs one tick's reading, and its age catches a silent sensor.  valid means a
// plausible reading exists — a failed transaction leaves the last one in place and only ages it,
// so staleness is the age alone.
static void gather(struct ace2k_dryer_inputs *in, uint32_t now_ms)
{
    const struct ace2k_heat_world *w = ace2k_dryer_world;
    const struct ace2k_env *env = ace2k_dryer_env;
    in->ntc_left_mc = w->ntc_left_mc;
    in->ntc_right_mc = w->ntc_right_mc;
    in->left_valid = w->left_valid;
    in->right_valid = w->right_valid;
    in->chamber_mc = env->chamber_mc;
    in->chamber_valid = ace2k_env_chamber_plausible(env);
    in->chamber_age_ms =
        env->chamber_ever ? ace2k_time_since(now_ms, env->chamber_at_ms) : ACE2K_DRYER_AGE_NEVER_MS;
    in->chamber_rh_pct10 = env->chamber_rh_pct10;
    in->rh_valid = rh_valid(env, in->chamber_valid);
    in->mains_present = w->mains_present;
    in->mains_plausible = w->mains_plausible;
    in->cutout = w->cutout;
    in->insert_mask = (uint8_t)(((uint32_t)w->switches >> ACE2K_SENSORS_STATE_INSERT_SHIFT) &
                                ACE2K_DRYER_INSERT_MASK);
    in->link_ok = ace2k_link_ok();
}

// A store may be tried: the log dirty, the gate idle, and after a flash failure the back-off
// elapsed.
static bool store_allowed(uint32_t now_ms)
{
    if (!ace2k_dryer_log_dirty(&ace2k_dryer_instance) ||
        ace2k_heat_busy(ace2k_dryer_instance.heat)) {
        return false;
    }
    if (!ace2k_dryer_store_failed) {
        return true;
    }
    return ace2k_time_after(now_ms, ace2k_dryer_store_retry_ms);
}

// Something for the task: a due report, an event to send or resend once the host listens, a log
// it may store
// (while leased or backing off the store waits, so no wake every tick for it).
static bool task_wanted(uint32_t now_ms)
{
    struct ace2k_dryer_event ev;
    if (ace2k_dryer_report_due) {
        return true;
    }
    if (ace2k_dryer_subscribed &&
        (ace2k_dryer_resend_due || ace2k_dryer_event_next(&ace2k_dryer_instance, &ev))) {
        return true;
    }
    return store_allowed(now_ms);
}

// Timer interrupt, after airflow and heat (heat_cmds.c's ordered tick).
static void tick(uint32_t now_ms)
{
    struct ace2k_dryer_inputs in;
    gather(&in, now_ms);
    ace2k_dryer_tick(&ace2k_dryer_instance, &in, now_ms);
    if (ace2k_report_due(&ace2k_dryer_report, now_ms)) {
        ace2k_dryer_report_due = true;
    }
    if (ace2k_dryer_event_resend_due(&ace2k_dryer_instance, ace2k_dryer_subscribed,
                                     ace2k_dryer_last_send_ms, now_ms)) {
        ace2k_dryer_resend_due = true;
    }
    if (task_wanted(now_ms)) {
        sched_wake_task(&ace2k_dryer_wake);
    }
}

static bool report(void)
{
    struct ace2k_dryer_status st;
    irqstatus_t flag = irq_save();
    ace2k_dryer_status(&ace2k_dryer_instance, &st, ace2k_tick_now_ms());
    irq_restore(flag);
    uint8_t notices = st.notices;
    if (ace2k_dryer_store_failed) {
        notices |= (uint8_t)ACE2K_DRYER_NOTICE_LOG_UNSTORED;
    }
    return ACE2K_SENDF("ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c "
                       "remaining_min=%hu fans=%c flaps=%c fault=%c notices=%c lost=%hu "
                       "oldest=%c",
                       st.state, st.target_c, st.drive_dc, st.duty_pct, st.remaining_min,
                       st.fans_read, st.flaps, st.fault, notices, st.events_lost, st.oldest);
}

// The log reaches the page only with the gate idle (rule 13; the config store veto enforces it
// too).  With the tick masked: the generation read, then the working log copied into the record
// — the snapshot the store encodes.  The store outside the bracket; the generation handed back
// only after a successful one, so a change the tick logs meanwhile or a refusal (a lease began)
// leaves the log dirty for the next wake; a flash failure backs off (store_failed) and raises
// ACE2K_DRYER_NOTICE_LOG_UNSTORED in the report until a store succeeds.
static void store_failed(void)
{
    uint32_t backoff = ace2k_dryer_store_backoff_ms;
    ace2k_dryer_store_retry_ms = ace2k_tick_now_ms() + backoff;
    ace2k_dryer_store_failed = true;
    backoff *= 2U;
    ace2k_dryer_store_backoff_ms =
        backoff > ACE2K_DRYER_STORE_BACKOFF_MAX_MS ? ACE2K_DRYER_STORE_BACKOFF_MAX_MS : backoff;
}

static void store_log(void)
{
    irqstatus_t flag = irq_save();
    bool due = store_allowed(ace2k_tick_now_ms());
    uint32_t gen = ace2k_dryer_log_generation(&ace2k_dryer_instance);
    if (due) {
        ace2k_dryer_cfg->record.dryer_log = ace2k_dryer_log_work;
    }
    irq_restore(flag);
    if (!due) {
        return;
    }
    int rc = ace2k_config_binding_store();
    if (rc == -ACE2K_EBUSY) {
        return; // a lease began: tried again when it ends
    }
    if (rc != 0) {
        store_failed();
        return;
    }
    ace2k_dryer_store_failed = false;
    ace2k_dryer_store_backoff_ms = ACE2K_DRYER_STORE_BACKOFF_MIN_MS;
    flag = irq_save();
    ace2k_dryer_log_stored(&ace2k_dryer_instance, gen);
    irq_restore(flag);
}

// The events once the host listens, oldest first, each kept until the host acknowledges it
// (ace2k_dryer_event_ack): a resend asked by the query or by the 5 s timer starts again from the
// oldest unacknowledged.  Seven in the ring; one dropped on a full ring is counted in the report's
// lost.  False when a frame did not fit (tx.h): the rest waits for the next tick.
static bool send_events(void)
{
    if (!ace2k_dryer_subscribed) {
        return true;
    }
    if (ace2k_dryer_resend_due) {
        ace2k_dryer_resend_due = false;
        ace2k_dryer_event_resend(&ace2k_dryer_instance);
    }
    struct ace2k_dryer_event ev;
    while (ace2k_dryer_event_next(&ace2k_dryer_instance, &ev)) {
        if (!ACE2K_SENDF(ACE2K_DRYER_EVENT_FMT, ev.kind, ev.arg, ev.seq)) {
            return false;
        }
        ace2k_dryer_event_sent(&ace2k_dryer_instance);
        ace2k_dryer_last_send_ms = ace2k_tick_now_ms();
    }
    return true;
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.  The
// due report first, then the events (report.h's order), then the log.
void ace2k_dryer_binding_task(void)
{
    if (!sched_check_wake(&ace2k_dryer_wake)) {
        return;
    }
    if (ace2k_dryer_report_due && !ace2k_report_try(&ace2k_dryer_report_due, report)) {
        return; // the buffer is full: the events wait behind the report, the next tick retries
    }
    if (!send_events()) {
        return;
    }
    store_log();
}
DECL_TASK(ace2k_dryer_binding_task);

// Every input binding fetched before the hook is installed: the tick only reads them, never
// initialises one from the timer interrupt.  The reset cause is read once, here.
static void ensure_ready(void)
{
    if (ace2k_dryer_ready) {
        return;
    }
    struct ace2k_heat *heat = ace2k_heat_binding();
    struct ace2k_airflow *air = ace2k_airflow_binding();
    ace2k_dryer_env = ace2k_env_binding();
    ace2k_dryer_world = ace2k_heat_binding_world();
    ace2k_dryer_cfg = ace2k_config_binding();
    ace2k_dryer_log_work = ace2k_dryer_cfg->record.dryer_log;
    bool power_on = ace2k_sysinfo_reset_cause() == (uint8_t)ACE2K_RESET_POWER_ON;
    ace2k_dryer_init(&ace2k_dryer_instance, heat, air, &ace2k_dryer_log_work, power_on,
                     ace2k_tick_now_ms());
    ace2k_heat_binding_set_after_hook(tick); // last: the tick runs the dryer from here on
    ace2k_dryer_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_dryer_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_dryer_binding_init);

const struct ace2k_dryer *ace2k_dryer_binding(void)
{
    ensure_ready();
    return &ace2k_dryer_instance;
}

// Runs with interrupts disabled after Klipper's shutdown functions begin: the lease released, the
// gate dropped by heat; the fans stay with rule 7 (airflow, on the surviving tick).
void ace2k_dryer_binding_shutdown(void)
{
    if (ace2k_dryer_ready) {
        ace2k_dryer_shutdown(&ace2k_dryer_instance);
    }
}
DECL_SHUTDOWN(ace2k_dryer_binding_shutdown);

static void respond(enum dryer_op op, enum ace2k_dryer_refusal r)
{
    sendf("ace2k_dryer_response op=%c accepted=%c reason=%c", (uint8_t)op,
          r == ACE2K_DRYER_OK ? 1U : 0U, (uint8_t)r);
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).  On the raw
// arguments: a target or a duration past its field is out of range, never narrowed into it.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_start(uint32_t *args)
{
    ensure_ready();
    uint32_t target_c = args[0];
    uint32_t minutes = args[1];
    enum ace2k_dryer_refusal r = ACE2K_DRYER_RANGE;
    if (target_c <= UINT8_MAX && minutes <= UINT16_MAX) {
        irqstatus_t flag = irq_save();
        r = ace2k_dryer_start(&ace2k_dryer_instance, (uint8_t)target_c, (uint16_t)minutes,
                              ace2k_tick_now_ms());
        irq_restore(flag);
    }
    respond(OP_START, r);
}
DECL_COMMAND(ace2k_dryer_cmd_start, "ace2k_dryer_start target_c=%c minutes=%hu");

void ace2k_dryer_cmd_stop(uint32_t *args)
{
    (void)args;
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_dryer_refusal r = ace2k_dryer_stop(&ace2k_dryer_instance, ace2k_tick_now_ms());
    irq_restore(flag);
    respond(OP_STOP, r);
}
DECL_COMMAND(ace2k_dryer_cmd_stop, "ace2k_dryer_stop");

void ace2k_dryer_cmd_clear(uint32_t *args)
{
    (void)args;
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_dryer_refusal r = ace2k_dryer_clear(&ace2k_dryer_instance, ace2k_tick_now_ms());
    irq_restore(flag);
    respond(OP_CLEAR, r);
}
DECL_COMMAND(ace2k_dryer_cmd_clear, "ace2k_dryer_clear");

// The host's query also says it listens: from the first one on, the events leave (the boot's
// among them), every unacknowledged one again — at a connect, and when the host saw a gap.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = args[0] / timer_from_us(1000U);
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_dryer_report, rest_ms, ace2k_tick_now_ms(),
                     ACE2K_DRYER_REPORT_PHASE_MS);
    ace2k_dryer_report_due = false; // a report held from before the query: the new grid's
    ace2k_dryer_subscribed = true;
    ace2k_dryer_resend_due = true; // a new host, or a gap: everything unacknowledged again
    irq_restore(flag);
}
DECL_COMMAND(ace2k_dryer_cmd_query, "ace2k_dryer_query rest_ticks=%u");

// The host took every event up to seq: ace2k_dryer.py acknowledges the last one it handled in
// order, and one past the newest event ever sent is ignored (ace2k_dryer_event_ack).
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_event_ack(uint32_t *args)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    ace2k_dryer_event_ack(&ace2k_dryer_instance, (uint8_t)args[0]);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_dryer_cmd_event_ack, "ace2k_dryer_event_ack seq=%c");

// One entry packed into the frame's six words: kind | reason << 8 | cycle << 16, the heating
// seconds, the three temperatures in 0.1 °C (signed values sent as their two's complement — the
// host reads them back as int32), and a spare 0.
static void entry_fields(const struct ace2k_dryer_log_entry *e, uint32_t *v)
{
    v[0] = (uint32_t)e->kind | ((uint32_t)e->reason << ACE2K_DRYER_LOG_ENTRY_SHIFT_REASON) |
           ((uint32_t)e->cycle << ACE2K_DRYER_LOG_ENTRY_SHIFT_CYCLE);
    v[1] = e->heat_s;
    v[2] = (uint32_t)(int32_t)e->ntc_left_dc;
    v[3] = (uint32_t)(int32_t)e->ntc_right_dc;
    v[4] = (uint32_t)(int32_t)e->chamber_dc;
    v[5] = 0U;
}

// Index 0: the counters; 1–8: the entries, newest first.  Read with the tick masked, so an entry
// the tick logs meanwhile never reaches the frame half-written.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_log_query(uint32_t *args)
{
    ensure_ready();
    uint32_t index = args[0];
    if (index > ACE2K_DRYER_LOG_ENTRIES) {
        shutdown("ace2k: dryer log index out of range");
    }
    const struct ace2k_dryer_log *log = &ace2k_dryer_log_work;
    uint32_t v[ACE2K_DRYER_LOG_FIELDS];
    irqstatus_t flag = irq_save();
    if (index == 0U) {
        v[0] = log->cycles_started;
        v[1] = log->cycles_completed;
        v[2] = log->heat_s;
        v[3] = log->full_power_s;
        v[4] = log->faults;
        v[5] = log->flags;
    } else {
        entry_fields(&log->entry[index - 1U], v);
    }
    irq_restore(flag);
    sendf(ACE2K_DRYER_LOG_FMT, (uint8_t)index, v[0], v[1], v[2], v[3], v[4], v[5]);
}
DECL_COMMAND(ace2k_dryer_cmd_log_query, "ace2k_dryer_log_query index=%c");
