// Binding of the airflow module: the board's fan and coil pins as the core's ops, env's two
// outlet NTCs into the thermal fan rule every tick, the manual commands, the periodic report from
// a task (sendf never runs in timer context).  The commands call the core from task context with
// the tick masked (irq_save), as mains_cmds.c does for its report deadline, and with the tick's
// clock (ace2k_tick_now_ms), so a pulse's length is measured in tick time; the report reads a few
// bytes the tick writes — a torn read costs one stale value in one report, never a fault.
#include "heat/airflow_cmds.h"
#include "ace2k_board/airflow_io.h"
#include "ace2k_board/tick.h"
#include "env/env_cmds.h"    // ace2k_env_binding
#include "core/guard_cmds.h" // ace2k_guard_binding
#include "core/report.h"
#include "core/tx.h"    // ACE2K_SENDF
#include "board/irq.h"  // irq_save, irq_restore
#include "board/misc.h" // timer_from_us
#include "command.h"    // DECL_COMMAND, sendf
#include "sched.h"      // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task, sched_check_wake

#define ACE2K_AIRFLOW_REPORT_PHASE_MS 10U /* the phase: report.h */

enum airflow_op { AIRFLOW_OP_FAN = 0, AIRFLOW_OP_FLAP = 1 };
enum airflow_reason { AIRFLOW_REASON_OK = 0, AIRFLOW_REASON_BUSY = 1, AIRFLOW_REASON_RANGE = 2 };

static struct ace2k_airflow ace2k_airflow_instance;
static const struct ace2k_env *ace2k_airflow_env;
static struct task_wake ace2k_airflow_wake;
static struct ace2k_report ace2k_airflow_report;
static volatile bool ace2k_airflow_report_due;
static bool ace2k_airflow_ready;

static void fan_write(void *ctx, bool on)
{
    (void)ctx;
    ace2k_airflow_io_fans(on);
}

static uint8_t fan_read(void *ctx)
{
    (void)ctx;
    return ace2k_airflow_io_fans_read();
}

static void flap_write(void *ctx, enum ace2k_flap flap, bool open_coil, bool close_coil)
{
    (void)ctx;
    ace2k_airflow_io_flap((uint8_t)flap, open_coil, close_coil);
}

static const struct ace2k_airflow_ops ace2k_airflow_board_ops = {
    .fan_write = fan_write,
    .fan_read = fan_read,
    .flap_write = flap_write,
};

void ace2k_airflow_binding_tick(uint32_t now_ms)
{
    const struct ace2k_env *e = ace2k_airflow_env;
    bool left_valid = (e->valid & ACE2K_ENV_VALID_NTC_LEFT) != 0U;
    bool right_valid = (e->valid & ACE2K_ENV_VALID_NTC_RIGHT) != 0U;
    ace2k_airflow_tick(&ace2k_airflow_instance, e->ptc_left_mc, e->ptc_right_mc, left_valid,
                       right_valid, now_ms);
    if (ace2k_report_due(&ace2k_airflow_report, now_ms)) {
        ace2k_airflow_report_due = true;
    }
    // a report the last pass held (its frame did not fit) leaves on this one
    if (ace2k_airflow_report_due) {
        sched_wake_task(&ace2k_airflow_wake);
    }
}

// True when the report was queued; false when it did not fit and its flag holds it for the next
// tick (tx.h; the order: report.h).
static bool report(void)
{
    const struct ace2k_airflow *a = &ace2k_airflow_instance;
    return ACE2K_SENDF("ace2k_airflow_state fans_cmd=%c fans_read=%c owner=%c flaps=%c",
                       ace2k_airflow_fans_commanded(a) ? 1 : 0, ace2k_airflow_fans_read(a),
                       (uint8_t)a->owner, ace2k_airflow_flaps_report(a));
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
void ace2k_airflow_binding_task(void)
{
    if (!sched_check_wake(&ace2k_airflow_wake)) {
        return;
    }
    if (!ace2k_airflow_report_due) {
        return;
    }
    (void)ace2k_report_try(&ace2k_airflow_report_due, report);
}
DECL_TASK(ace2k_airflow_binding_task);

// The bootloader drives no fans: neither ace2k_bootloader_enter nor the link proof's reset into
// recovery (both ask the guard) while the air path must keep running (rule 7) — until the
// cool-down, bounded by the dryer's 10 min hand-over and rule 7's release below 42 °C.
static bool fans_required_veto(void *ctx)
{
    (void)ctx;
    return ace2k_airflow_fans_required(&ace2k_airflow_instance);
}

static void ensure_ready(void)
{
    if (ace2k_airflow_ready) {
        return;
    }
    ace2k_airflow_io_init();
    ace2k_airflow_env = ace2k_env_binding();
    ace2k_airflow_init(&ace2k_airflow_instance, &ace2k_airflow_board_ops, NULL);
    // No tick of its own: heat_cmds.c's one ordered tick calls ace2k_airflow_binding_tick()
    // first, before heat and the dryer.
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
    if (ace2k_guard_register_veto(ace2k_guard_binding(), fans_required_veto, NULL) != 0) {
        shutdown("ace2k: guard veto slots exhausted");
    }
    ace2k_airflow_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_airflow_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_airflow_binding_init);

struct ace2k_airflow *ace2k_airflow_binding(void)
{
    ensure_ready();
    return &ace2k_airflow_instance;
}

// Interrupts are disabled here (Klipper's run_shutdown).  Every owner released and every coil
// let go; the fans are not written off — rule 7 keeps them while an NTC is hot, and the tick,
// which survives the shutdown (tick.c), keeps applying it with the ADC scan's NTCs (adc_scan.c
// restarts its timer on a shutdown too).
void ace2k_airflow_binding_shutdown(void)
{
    if (!ace2k_airflow_ready) {
        return;
    }
    ace2k_airflow_shutdown(&ace2k_airflow_instance);
}
DECL_SHUTDOWN(ace2k_airflow_binding_shutdown);

static uint8_t reason_of(int rc)
{
    if (rc == 0) {
        return AIRFLOW_REASON_OK;
    }
    if (rc == -ACE2K_EBUSY) {
        return AIRFLOW_REASON_BUSY;
    }
    return AIRFLOW_REASON_RANGE;
}

static void respond(uint8_t op, int rc)
{
    sendf("ace2k_airflow_response op=%c accepted=%c reason=%c", op, rc == 0 ? 1 : 0, reason_of(rc));
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_airflow_cmd_fan_set(uint32_t *args)
{
    ensure_ready();
    bool on = args[0] != 0U;
    uint32_t seconds = args[1];
    irqstatus_t flag = irq_save();
    int rc = ace2k_airflow_fans(&ace2k_airflow_instance, ACE2K_AIRFLOW_OWNER_MANUAL, on, seconds,
                                ace2k_tick_now_ms());
    irq_restore(flag);
    respond(AIRFLOW_OP_FAN, rc);
}
DECL_COMMAND(ace2k_airflow_cmd_fan_set, "ace2k_fan_set on=%c seconds=%hu");

// The pulse with the tick masked, for the compiled ACE2K_FLAP_PULSE_MS: refused while the dryer
// holds the flaps or another pulse runs or waits (a manual pulse is never queued).
// cppcheck-suppress constParameterPointer
void ace2k_airflow_cmd_flap_pulse(uint32_t *args)
{
    ensure_ready();
    uint32_t flap = args[0];
    bool open = args[1] != 0U;
    int rc = -ACE2K_EINVAL;
    if (flap < (uint32_t)ACE2K_FLAP_COUNT) {
        irqstatus_t flag = irq_save();
        rc = ace2k_airflow_flap_pulse(&ace2k_airflow_instance, ACE2K_AIRFLOW_OWNER_MANUAL,
                                      (enum ace2k_flap)flap, open, ace2k_tick_now_ms());
        irq_restore(flag);
    }
    respond(AIRFLOW_OP_FLAP, rc);
}
DECL_COMMAND(ace2k_airflow_cmd_flap_pulse, "ace2k_flap_pulse flap=%c open=%c");

// cppcheck-suppress constParameterPointer
void ace2k_airflow_cmd_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = args[0] / timer_from_us(1000U);
    // The tick reads the pair in the timer interrupt; both stores in one step.
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_airflow_report, rest_ms, ace2k_tick_now_ms(),
                     ACE2K_AIRFLOW_REPORT_PHASE_MS);
    ace2k_airflow_report_due = false; // a report held from before the query: the new grid's
    irq_restore(flag);
}
DECL_COMMAND(ace2k_airflow_cmd_query, "ace2k_airflow_query rest_ticks=%u");
