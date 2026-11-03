// Binding of the dryer. It runs one dryer instance on the instances of the heater and airflow
// bindings. Its log is a working copy of the log in the config record. The automatic modes
// (dryer_auto.h) live beside it: their tick runs right after the dryer's, and the host's start and
// stop go through them. Their commands, report and store are in dryer_auto_cmds.c.
// - The tick gathers the inputs: the world that heat's tick read, the chamber and the link.
// - A task sends the report and the events, and stores the log and the modes' block, in one write
//   of the page. sendf and the flash never run in timer context.
// - The commands bracket each core call with interrupts masked, because the tick runs the dryer
//   in the timer interrupt.
// A refused command is answered with its reason, never with a shutdown. A range, a fault or a
// missing mains are conditions of the unit. A log index outside the dictionary's range is a
// host bug, and it takes the Klipper way out (a shutdown).
#include "dryer/dryer_cmds.h"
#include "dryer/dryer_auto.h"
#include "heat/airflow_cmds.h"   // ace2k_airflow_binding
#include "core/backoff.h"        // ace2k_backoff, the store's back-off
#include "core/config_cmds.h"    // ace2k_config_binding, ace2k_config_binding_store
#include "env/env_cmds.h"        // ace2k_env_binding
#include "core/health.h"         // ACE2K_RESET_POWER_ON
#include "heat/heat_cmds.h"      // ace2k_heat_binding, ace2k_heat_binding_world, the after hook
#include "core/report_cmds.h"    // ace2k_report_query
#include "lane/sensors.h"        // ACE2K_SENSORS_STATE_INSERT_SHIFT
#include "ace2k_board/args.h"    // ACE2K_ARG_U8, ACE2K_ARG_U8_MAX
#include "ace2k_board/tx.h"      // ACE2K_SENDF
#include "ace2k_board/serial.h"  // ace2k_link_ok
#include "ace2k_board/sysinfo.h" // ace2k_sysinfo_reset_cause
#include "ace2k_board/tick.h"    // ace2k_tick_now_ms
#include "board/irq.h"           // irq_save, irq_restore
#include "board/misc.h"          // timer_from_us
#include "command.h"             // DECL_COMMAND, DECL_CONSTANT, sendf, shutdown
#include "sched.h"               // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task

#define ACE2K_DRYER_EVENT_FMT              "ace2k_dryer_event kind=%c arg=%c seq=%c"
#define ACE2K_DRYER_LOG_FMT                "ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u"
#define ACE2K_DRYER_LOG_FIELDS             6U
#define ACE2K_DRYER_LOG_ENTRY_SHIFT_REASON 8U
#define ACE2K_DRYER_LOG_ENTRY_SHIFT_CYCLE  16U
#define ACE2K_DRYER_INSERT_MASK            0xFU
#define ACE2K_DRYER_AGE_NEVER_MS           UINT32_MAX
// The last word of a log frame.  It carries the flags in the counters' frame and a spare 0 in an
// entry's frame.
#define ACE2K_DRYER_LOG_LAST (ACE2K_DRYER_LOG_FIELDS - 1U)

// Klipper's DECL_CONSTANT encodes its value with signed literals inside the macro. So the
// lint's signed-bitwise check is off for these declarations only.
// NOLINTBEGIN(bugprone-signed-bitwise)
// The host checks the arguments against these before it sends them (ace2k_dryer.py).
DECL_CONSTANT("ACE2K_DRYER_TARGET_MIN_C", ACE2K_DRYER_TARGET_MIN_C);
DECL_CONSTANT("ACE2K_DRYER_TARGET_MAX_C", ACE2K_DRYER_TARGET_MAX_C);
DECL_CONSTANT("ACE2K_DRYER_MINUTES_MAX", ACE2K_DRYER_MINUTES_MAX);
// NOLINTEND(bugprone-signed-bitwise)

static struct ace2k_dryer ace2k_dryer_instance;
static struct ace2k_dryer_log ace2k_dryer_log_work; // the tick writes it; copied to the record
static struct ace2k_dryer_auto ace2k_dryer_auto_instance;
static struct ace2k_dryer_auto_store ace2k_dryer_auto_work; // copied to the record when stored
static struct ace2k_config *ace2k_dryer_cfg;
static const struct ace2k_env *ace2k_dryer_env;
static const struct ace2k_heat_world *ace2k_dryer_world;
static struct task_wake ace2k_dryer_wake;
static struct ace2k_report ace2k_dryer_report;
static volatile bool ace2k_dryer_report_due;
static volatile bool ace2k_dryer_subscribed; // the host queried once, so the events may leave
// The resend of the events. The task writes last_send_ms, and the tick reads it. The tick sets
// resend_due once 5 s have passed since the last send. The query sets it at once. The task clears
// it. These are single aligned words.
static volatile bool ace2k_dryer_resend_due;
static volatile uint32_t ace2k_dryer_last_send_ms;
static bool ace2k_dryer_ready;
// The store's back-off (core/backoff.h). After a flash failure the same data is tried again,
// because a cutout marked once must still reach the page: only a power cycle clears it. The task
// changes it with the tick masked; the tick reads it.
static struct ace2k_backoff ace2k_dryer_store_backoff = ACE2K_BACKOFF_INIT;

// The chamber's RH is valid with the chamber reading, and within the full scale of the
// sensor.
static bool rh_valid(const struct ace2k_env *env, bool chamber_valid)
{
    if (!chamber_valid) {
        return false;
    }
    return env->chamber_rh_pct10 <= ACE2K_DRYER_RH_FULL_PCT10;
}

// Gathers the inputs. They are the world that heat's tick read, the chamber and the link. The
// world holds the NTCs, kept at their last valid value while invalid, the mains and the switches.
// The env task writes the chamber fields. A value read torn costs the reading of one tick, and its
// age catches a silent sensor. valid means that a plausible reading exists. A failed transaction
// leaves the last reading in place and only ages it. So staleness is the age alone.
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

// True when a store may be tried: the log or the modes' block is dirty, the gate is idle, and
// after a flash failure the back-off has elapsed.
static bool store_allowed(uint32_t now_ms)
{
    if ((!ace2k_dryer_log_dirty(&ace2k_dryer_instance) &&
         !ace2k_dryer_auto_dirty(&ace2k_dryer_auto_instance)) ||
        ace2k_heat_busy(ace2k_dryer_instance.heat)) {
        return false;
    }
    return ace2k_backoff_allowed(&ace2k_dryer_store_backoff, now_ms);
}

// True when the task has work: a due report (the dryer's or its modes'), an event to send or
// resend once the host listens, or a log it may store. While leased or backing off, the store
// waits, so it causes no wake every tick.
static bool task_wanted(uint32_t now_ms)
{
    struct ace2k_dryer_event ev;
    if (ace2k_dryer_report_due || ace2k_dryer_auto_binding_due()) {
        return true;
    }
    if (ace2k_dryer_subscribed &&
        (ace2k_dryer_resend_due || ace2k_dryer_event_next(&ace2k_dryer_instance, &ev))) {
        return true;
    }
    return store_allowed(now_ms);
}

// Runs in the timer interrupt, after airflow and heat, in heat_cmds.c's ordered tick.
static void tick(uint32_t now_ms)
{
    struct ace2k_dryer_inputs in;
    gather(&in, now_ms);
    ace2k_dryer_tick(&ace2k_dryer_instance, &in, now_ms);
    ace2k_dryer_auto_tick(&ace2k_dryer_auto_instance, &in, now_ms);
    ace2k_dryer_auto_binding_tick(now_ms);
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
    if (ace2k_dryer_store_backoff.failed) {
        notices |= (uint8_t)ACE2K_DRYER_NOTICE_LOG_UNSTORED;
    }
    return ACE2K_SENDF("ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c "
                       "remaining_min=%hu fans=%c flaps=%c fault=%c notices=%c lost=%hu "
                       "oldest=%c",
                       st.state, st.target_c, st.drive_dc, st.duty_pct, st.remaining_min,
                       st.fans_read, st.flaps, st.fault, notices, st.events_lost, st.oldest);
}

// The log and the modes' block reach the page only with the gate idle. The veto of the config
// store enforces it too. Every store writes the whole record, so one write carries both. With the
// tick masked, both generations are read, then both working copies are copied into the record.
// That copy is the snapshot the store encodes. The store itself runs outside the bracket. The
// generations are handed back only after a successful store. So a block stays dirty for the next
// wake when the tick changes it meanwhile, or when the store is refused because a lease began. A
// flash failure backs off. It raises ACE2K_DRYER_NOTICE_LOG_UNSTORED in the report until a store
// succeeds.
static void store_page(void)
{
    irqstatus_t flag = irq_save();
    bool due = store_allowed(ace2k_tick_now_ms());
    uint32_t log_gen = ace2k_dryer_log_generation(&ace2k_dryer_instance);
    uint32_t auto_gen = ace2k_dryer_auto_generation(&ace2k_dryer_auto_instance);
    if (due) {
        ace2k_dryer_cfg->record.dryer_log = ace2k_dryer_log_work;
        ace2k_dryer_cfg->record.dryer_auto = ace2k_dryer_auto_work;
    }
    irq_restore(flag);
    if (!due) {
        return;
    }
    int rc = ace2k_config_binding_store();
    if (rc == -ACE2K_EBUSY) {
        return; // a lease began: the store is tried again when it ends
    }
    flag = irq_save();
    if (rc != 0) {
        ace2k_backoff_failed(&ace2k_dryer_store_backoff, ace2k_tick_now_ms());
    } else {
        ace2k_backoff_succeeded(&ace2k_dryer_store_backoff);
        ace2k_dryer_log_stored(&ace2k_dryer_instance, log_gen);
        ace2k_dryer_auto_stored(&ace2k_dryer_auto_instance, auto_gen);
    }
    irq_restore(flag);
}

// Sends the events once the host listens, oldest first. Each event is kept until the host
// acknowledges it (ace2k_dryer_event_ack). A resend, asked by the query or by the 5 s timer,
// starts again from the oldest unacknowledged event. The ring holds seven. An event dropped on a
// full ring is counted in the lost field of the report. Returns false when a frame did not fit
// (tx.h); the rest waits for the next tick.
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

// Klipper calls every DECL_TASK target by name from generated code, so it is not static. It sends
// the due reports first, the dryer's and then its modes', then the events, as report.h orders
// them, then stores the page.
void ace2k_dryer_binding_task(void)
{
    if (!sched_check_wake(&ace2k_dryer_wake)) {
        return;
    }
    if (ace2k_dryer_report_due && !ace2k_report_try(&ace2k_dryer_report_due, report)) {
        return; // the buffer is full: the events wait behind the report; the next tick retries
    }
    if (!ace2k_dryer_auto_binding_send()) {
        return; // the same, for the modes' report
    }
    if (!send_events()) {
        return;
    }
    store_page();
}
DECL_TASK(ace2k_dryer_binding_task);

// Every input binding is fetched before the hook is installed. So the tick only reads them, and
// never initialises one from the timer interrupt. The reset cause is read once, here.
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
    ace2k_dryer_auto_work = ace2k_dryer_cfg->record.dryer_auto;
    ace2k_dryer_auto_init(&ace2k_dryer_auto_instance, &ace2k_dryer_instance,
                          &ace2k_dryer_auto_work);
    ace2k_heat_binding_set_after_hook(tick); // last: from here on the tick runs the dryer
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

struct ace2k_dryer_auto *ace2k_dryer_auto_binding(void)
{
    ensure_ready();
    return &ace2k_dryer_auto_instance;
}

// Runs with interrupts disabled, after Klipper's shutdown functions begin. The lease is
// released, and heat drops the gate. The fans stay with airflow's fan rule (a fan never off
// above 45 °C), on the tick that survives the shutdown.
void ace2k_dryer_binding_shutdown(void)
{
    if (ace2k_dryer_ready) {
        ace2k_dryer_shutdown(&ace2k_dryer_instance);
    }
}
DECL_SHUTDOWN(ace2k_dryer_binding_shutdown);

void ace2k_dryer_binding_respond(enum ace2k_dryer_cmd_op op, enum ace2k_dryer_refusal r)
{
    sendf("ace2k_dryer_response op=%c accepted=%c reason=%c", (uint8_t)op,
          r == ACE2K_DRYER_OK ? 1U : 0U, (uint8_t)r);
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *). The checks run
// on the raw arguments. A target or a duration past its field is out of range, and it is never
// narrowed into the field.
// The host's start goes through the automatic modes: over an automatic cycle it retargets that
// cycle (dryer_auto.h).
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_start(uint32_t *args)
{
    ensure_ready();
    uint32_t target_c = args[0];
    uint32_t minutes = args[1];
    enum ace2k_dryer_refusal r = ACE2K_DRYER_RANGE;
    if (target_c <= UINT8_MAX && minutes <= UINT16_MAX) {
        irqstatus_t flag = irq_save();
        r = ace2k_dryer_auto_manual_start(&ace2k_dryer_auto_instance, (uint8_t)target_c,
                                          (uint16_t)minutes, ace2k_tick_now_ms());
        irq_restore(flag);
    }
    ace2k_dryer_binding_respond(ACE2K_DRYER_OP_START, r);
}
DECL_COMMAND(ace2k_dryer_cmd_start, "ace2k_dryer_start target_c=%c minutes=%hu");

// The host's stop goes through the automatic modes too: the stop of an own cycle turns the
// humidity mode off, and it is never scored as a completed cycle (dryer_auto.h).
void ace2k_dryer_cmd_stop(uint32_t *args)
{
    (void)args;
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_dryer_refusal r =
        ace2k_dryer_auto_manual_stop(&ace2k_dryer_auto_instance, ace2k_tick_now_ms());
    irq_restore(flag);
    ace2k_dryer_binding_respond(ACE2K_DRYER_OP_STOP, r);
}
DECL_COMMAND(ace2k_dryer_cmd_stop, "ace2k_dryer_stop");

void ace2k_dryer_cmd_clear(uint32_t *args)
{
    (void)args;
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_dryer_refusal r = ace2k_dryer_clear(&ace2k_dryer_instance, ace2k_tick_now_ms());
    irq_restore(flag);
    ace2k_dryer_binding_respond(ACE2K_DRYER_OP_CLEAR, r);
}
DECL_COMMAND(ace2k_dryer_cmd_clear, "ace2k_dryer_clear");

// The host's query also says that it listens. From the first query on, the events leave,
// the boot events among them. Each query sends every unacknowledged event again. That happens at
// a connect, and when the host saw a gap.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_query(uint32_t *args)
{
    ensure_ready();
    // Not ace2k_report_query, because the subscription is set under the same mask as the deadline.
    uint32_t rest_ms = ace2k_report_rest_ms(args[0], timer_from_us(ACE2K_REPORT_US_PER_MS));
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_dryer_report, rest_ms, ace2k_tick_now_ms(),
                     ACE2K_DRYER_REPORT_PHASE_MS);
    ace2k_dryer_report_due = false; // a held report is dropped; the next one is on the new grid
    ace2k_dryer_subscribed = true;
    ace2k_dryer_resend_due = true; // a new host, or a gap: send everything unacknowledged again
    irq_restore(flag);
}
DECL_COMMAND(ace2k_dryer_cmd_query, "ace2k_dryer_query rest_ticks=%u");

// The host took every event up to seq. ace2k_dryer.py acknowledges the last event it handled in
// order. An ack past the newest event ever sent is ignored (ace2k_dryer_event_ack). A seq above
// 255 is a host bug: the sequence is 8 bits wide, and it must not wrap into a valid one.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_event_ack(uint32_t *args)
{
    ensure_ready();
    uint8_t seq = ACE2K_ARG_U8(args[0], "ace2k_dryer_event_ack: seq out of range");
    irqstatus_t flag = irq_save();
    ace2k_dryer_event_ack(&ace2k_dryer_instance, seq);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_dryer_cmd_event_ack, "ace2k_dryer_event_ack seq=%c");

// Packs one entry into the frame's six words: kind | reason << 8 | cycle << 16, the heating
// seconds, the three temperatures in 0.1 °C, and a spare 0. The temperatures are signed values,
// sent as their two's complement, and the host reads them back as int32.
static void entry_fields(const struct ace2k_dryer_log_entry *e, uint32_t *v)
{
    v[0] = (uint32_t)e->kind | ((uint32_t)e->reason << ACE2K_DRYER_LOG_ENTRY_SHIFT_REASON) |
           ((uint32_t)e->cycle << ACE2K_DRYER_LOG_ENTRY_SHIFT_CYCLE);
    v[1] = e->heat_s;
    v[2] = (uint32_t)(int32_t)e->ntc_left_dc;
    v[3] = (uint32_t)(int32_t)e->ntc_right_dc;
    v[4] = (uint32_t)(int32_t)e->chamber_dc;
    v[ACE2K_DRYER_LOG_LAST] = 0U;
}

// Index 0 is the counters, and 1–8 are the entries, newest first. The log is read with the tick
// masked. So an entry that the tick logs meanwhile never reaches the frame half-written.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_log_query(uint32_t *args)
{
    ensure_ready();
    uint8_t index =
        ACE2K_ARG_U8_MAX(args[0], ACE2K_DRYER_LOG_ENTRIES, "ace2k: dryer log index out of range");
    const struct ace2k_dryer_log *log = &ace2k_dryer_log_work;
    uint32_t v[ACE2K_DRYER_LOG_FIELDS];
    irqstatus_t flag = irq_save();
    if (index == 0U) {
        v[0] = log->cycles_started;
        v[1] = log->cycles_completed;
        v[2] = log->heat_s;
        v[3] = log->full_power_s;
        v[4] = log->faults;
        v[ACE2K_DRYER_LOG_LAST] = log->flags;
    } else {
        entry_fields(&log->entry[index - 1U], v);
    }
    irq_restore(flag);
    sendf(ACE2K_DRYER_LOG_FMT, index, v[0], v[1], v[2], v[3], v[4], v[5]);
}
DECL_COMMAND(ace2k_dryer_cmd_log_query, "ace2k_dryer_log_query index=%c");
