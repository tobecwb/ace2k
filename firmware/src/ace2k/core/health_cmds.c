// Binding of the self-test. It holds:
// - the boot window, 2 s after init: the counters must be quiet, and the zero-cross window must
//   have filled;
// - the full evaluation from a task, which probes the readers over SPI and checks the image CRC
//   once;
// - a periodic re-evaluation every second;
// - the run, clear and query commands, and the device identifier.
// The full evaluation is three task runs, not one. A reader probe busy-waits up to 150 ms
// (rfid/reader.h). Klipper's independent watchdog (273–546 ms, fed from a task of its own) is
// not fed while a task runs. Two probes back to back plus the evaluation would come too close to
// it. So one task run probes reader A, the next probes reader B, and the third gathers and
// evaluates. Each step wakes the task again for the next.
#include "core/health.h"
#include "core/config_cmds.h"
#include "env/env_cmds.h"
#include "heat/mains_cmds.h"
#include "core/report.h"
#include "lane/sensors_cmds.h"
#include "rfid/reader_cmds.h"
#include "ace2k_board/encoder.h"
#include "ace2k_board/fg.h"
#include "ace2k_board/sysinfo.h"
#include "ace2k_board/tick.h"
#include "autoconf.h"  // CONFIG_ACE2K_RFID_READ
#include "board/irq.h" // irq_save, irq_restore
#include "command.h"   // DECL_COMMAND, sendf, shutdown
#include "sched.h"     // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_RFID_READ
#include "rfid/watch_cmds.h" // a field fault fails its bit; a run revives a dead reader
#endif

// The steps of a full evaluation, one per task run (see the top of this file).
enum ace2k_health_step {
    ACE2K_HEALTH_STEP_IDLE = 0,
    ACE2K_HEALTH_PROBE_A,
    ACE2K_HEALTH_PROBE_B,
    ACE2K_HEALTH_EVALUATE,
};

static struct ace2k_health ace2k_health_instance;
static struct task_wake ace2k_health_wake;
static uint32_t ace2k_health_window_end_ms;
static struct ace2k_report ace2k_health_periodic; // rest ACE2K_HEALTH_PERIOD_MS, no phase
static uint16_t ace2k_health_snap_encoder[ACE2K_LANE_COUNT];
static uint32_t ace2k_health_snap_fg[ACE2K_LANE_COUNT];
static volatile bool ace2k_health_full_due;
static volatile bool ace2k_health_periodic_due;
static volatile bool ace2k_health_armed;
static enum ace2k_health_step ace2k_health_step;
static bool ace2k_health_image_checked;
static bool ace2k_health_image_ok;
static bool ace2k_health_ready;

// The board's raw counters, not those of the lane core. ACEK_COUNTERS_RESET rebases the
// core's values, and one inside the boot window would read as motion (bits 17–24) on a still
// unit. The encoder is the timer's free-running 16-bit count, so its difference is a signed
// 16-bit quantity. The tach count is a plain uint32_t.
static void snapshot_counters(void)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_health_snap_encoder[i] = ace2k_encoder_read(i);
        ace2k_health_snap_fg[i] = ace2k_fg_read(i);
    }
}

// The tick reads the pair in the timer interrupt. Both stores happen in one step, so the tick
// never sees the armed flag against a stale deadline.
static void arm(uint32_t now_ms)
{
    snapshot_counters();
    irqstatus_t flag = irq_save();
    ace2k_health_window_end_ms = now_ms + ACE2K_HEALTH_BOOT_WINDOW_MS;
    ace2k_health_armed = true;
    irq_restore(flag);
}

static bool field_fault(uint8_t reader)
{
#if CONFIG_ACE2K_RFID_READ
    return ace2k_rfid_binding_field_fault(reader);
#else
    (void)reader;
    return false;
#endif
}

_Static_assert(ACE2K_HEALTH_READERS == ACE2K_RFID_READER_COUNT, "a health bit per reader");

static void gather(struct ace2k_health_inputs *in)
{
    const struct ace2k_sensors *s = ace2k_sensors_binding();
    const struct ace2k_env *e = ace2k_env_binding();
    const struct ace2k_mains *m = ace2k_mains_binding();
    const struct ace2k_rfid_reader *r = ace2k_rfid_reader_binding();
    in->ntc_left_valid = ace2k_env_valid_bit(e, ACE2K_ENV_VALID_NTC_LEFT);
    in->ntc_right_valid = ace2k_env_valid_bit(e, ACE2K_ENV_VALID_NTC_RIGHT);
    in->chamber_valid = ace2k_env_valid_bit(e, ACE2K_ENV_VALID_CHAMBER);
    in->chamber_plausible = ace2k_env_chamber_plausible(e);
    in->reader_ok[0] = (bool)(r->version_ok[0] && !field_fault(ACE2K_RFID_READER_A));
    in->reader_ok[1] = (bool)(r->version_ok[1] && !field_fault(ACE2K_RFID_READER_B));
    in->zerocross_present = m->present;
    in->mains_plausible = ace2k_mains_plausible(m);
    in->cutout_idle = (bool)!s->cutout;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        in->insert_connected[i] = ace2k_sensors_insert_connected(s, i);
        in->buffer_consistent[i] = ace2k_sensors_buffer_consistent(s, i);
        in->encoder_delta[i] =
            (int16_t)(uint16_t)(ace2k_encoder_read(i) - ace2k_health_snap_encoder[i]);
        in->fg_delta[i] = ace2k_fg_read(i) - ace2k_health_snap_fg[i];
    }
    in->sensors_fresh = ace2k_sensors_fresh(s);
    in->vdda_ok = ace2k_env_valid_bit(e, ACE2K_ENV_VALID_VDDA);
    in->clock_ok = ace2k_sysinfo_clock_pll_hse();
    in->watchdog_ok = ace2k_sysinfo_watchdog_armed();
    in->config_page_ok = ace2k_config_binding()->source != ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT;
    in->image_crc_ok = ace2k_health_image_ok;
}

// Task context. A full pass checks the image CRC once. It seeds the periodic deadline together
// with post_done, in one step, because the tick reads the pair in the timer interrupt. So the tick
// never sees post_done against a stale deadline. A full pass also replaces a periodic pass that is
// due.
static void evaluate(bool full)
{
    if (full && !ace2k_health_image_checked) {
        ace2k_health_image_ok = ace2k_sysinfo_image_crc_ok();
        ace2k_health_image_checked = true;
    }
    struct ace2k_health_inputs in;
    gather(&in);
    uint32_t mask = ace2k_health_evaluate(&in);
    uint32_t now = ace2k_tick_now_ms();
    irqstatus_t flag = irq_save();
    if (full) {
        ace2k_report_set(&ace2k_health_periodic, ACE2K_HEALTH_PERIOD_MS, now, 0);
        ace2k_health_periodic_due = false;
    }
    ace2k_health_apply(&ace2k_health_instance, mask, now, full);
    irq_restore(flag);
}

// The periodic deadline is a struct ace2k_report. After a stall it catches up on its grid
// (report.h), which is the one policy of the tree. A catch-up tick opens a settle window first.
// The deadline is not consulted, so not advanced, until the window closes. So a pass due inside
// the window is raised at the first tick after it, on the grid, and not skipped. The settle gates
// the full pass too. The flash write of the link proof lands inside the first connect, exactly
// when the boot window can expire. The window's deadline keeps its time; the pass just starts
// after the settle.
static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    bool settled = ace2k_health_tick_settled(&ace2k_health_instance, now_ms, ACE2K_TICK_PERIOD_MS);
    if (ace2k_health_armed && ace2k_time_after(now_ms, ace2k_health_window_end_ms) && settled) {
        ace2k_health_armed = false;
        ace2k_health_full_due = true;
        sched_wake_task(&ace2k_health_wake);
    }
    if (ace2k_health_instance.post_done && settled &&
        ace2k_report_due(&ace2k_health_periodic, now_ms)) {
        ace2k_health_periodic_due = true;
        sched_wake_task(&ace2k_health_wake);
    }
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static. A full
// evaluation that is due restarts the three-step sequence. While one is in progress, a periodic
// pass waits, because the full pass replaces it.
void ace2k_health_binding_task(void)
{
    if (!sched_check_wake(&ace2k_health_wake)) {
        return;
    }
    if (ace2k_health_full_due) {
        ace2k_health_full_due = false;
        ace2k_health_step = ACE2K_HEALTH_PROBE_A;
    }
    switch (ace2k_health_step) {
    case ACE2K_HEALTH_PROBE_A:
        // A reader that the watch uses is not probed (rfid/reader_cmds.h). Its last verdict
        // stands until the next full pass finds it idle.
        (void)ace2k_rfid_reader_binding_probe(ACE2K_RFID_READER_A);
        ace2k_health_step = ACE2K_HEALTH_PROBE_B;
        sched_wake_task(&ace2k_health_wake);
        return;
    case ACE2K_HEALTH_PROBE_B:
        (void)ace2k_rfid_reader_binding_probe(ACE2K_RFID_READER_B);
        ace2k_health_step = ACE2K_HEALTH_EVALUATE;
        sched_wake_task(&ace2k_health_wake);
        return;
    case ACE2K_HEALTH_EVALUATE:
        ace2k_health_step = ACE2K_HEALTH_STEP_IDLE;
        evaluate(true);
        return;
    case ACE2K_HEALTH_STEP_IDLE:
    default:
        break;
    }
    if (ace2k_health_periodic_due) {
        ace2k_health_periodic_due = false;
        evaluate(false);
    }
}
DECL_TASK(ace2k_health_binding_task);

static void ensure_ready(void)
{
    if (ace2k_health_ready) {
        return;
    }
    uint32_t now = ace2k_tick_now_ms();
    ace2k_health_init(&ace2k_health_instance, ace2k_sysinfo_reset_cause(), now);
    arm(now);
    // A full registry is a programming error. From init context this ends in a fault and a
    // watchdog reset. Either way it is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_health_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_health_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_health_binding_init);

void ace2k_health_cmd_query(uint32_t *args)
{
    (void)args;
    ensure_ready();
    sendf("ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c",
          ace2k_health_instance.now, ace2k_health_instance.latched, ace2k_health_instance.post_ms,
          ace2k_health_instance.reset_cause);
}
DECL_COMMAND(ace2k_health_cmd_query, "ace2k_health_query");

// Arms the boot window again. The counters are snapshotted now, and the full evaluation runs when
// the window expires. A run that arrives while a three-step sequence is in flight snapshots the
// counters again under it, so that sequence measures a shorter window. The full pass 2 s later is
// the one that counts.
void ace2k_health_cmd_run(uint32_t *args)
{
    (void)args;
    ensure_ready();
#if CONFIG_ACE2K_RFID_READ
    ace2k_rfid_binding_clear_dead(); // a reader the watch gave up on is tried again
#endif
    arm(ace2k_tick_now_ms());
}
DECL_COMMAND(ace2k_health_cmd_run, "ace2k_health_run");

void ace2k_health_cmd_clear(uint32_t *args)
{
    (void)args;
    ensure_ready();
    ace2k_health_clear(&ace2k_health_instance);
}
DECL_COMMAND(ace2k_health_cmd_clear, "ace2k_health_clear");

// Read-only, and answered after a Klipper shutdown too, as the version and the link proof are.
// The host reads it at every connect, and a unit keeps a shutdown across a host restart.
void ace2k_health_cmd_uid(uint32_t *args)
{
    (void)args;
    uint8_t uid[ACE2K_SYSINFO_UID_LEN];
    ace2k_sysinfo_uid(uid);
    sendf("ace2k_uid_response uid=%*s", (uint8_t)sizeof uid, uid);
}
DECL_COMMAND_FLAGS(ace2k_health_cmd_uid, HF_IN_SHUTDOWN, "ace2k_uid_query");
