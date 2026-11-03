// Binding of the mains module. It feeds the board's zero-cross edge count into the core
// every tick. A task sends the periodic report, because sendf never runs in timer context. The
// tick and the task share the instance: the report reads three fields that the tick writes
// (hz10, present and rejects). A torn read costs one stale value in one report, never a fault.
#include "heat/mains_cmds.h"
#include "ace2k_board/tick.h"
#include "ace2k_board/tx.h" // ACE2K_SENDF
#include "ace2k_board/zerocross.h"
#include "core/report_cmds.h" // ace2k_report_query
#include "board/irq.h"        // irq_save, irq_restore
#include "board/misc.h"       // timer_from_us
#include "command.h"          // DECL_COMMAND, sendf, shutdown
#include "sched.h"            // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake

#define ACE2K_MAINS_REPORT_PHASE_MS 70U /* the phase: report.h */

static struct ace2k_mains ace2k_mains_instance;
static struct task_wake ace2k_mains_wake;
static struct ace2k_report ace2k_mains_report;
static volatile bool ace2k_mains_report_due;
static bool ace2k_mains_ready;

static uint32_t zerocross_count(void *ctx)
{
    (void)ctx;
    return ace2k_zerocross_count();
}

static uint32_t zerocross_rejects(void *ctx)
{
    (void)ctx;
    return ace2k_zerocross_rejects();
}

static void span(void *ctx, struct ace2k_mains_span *out)
{
    (void)ctx;
    ace2k_zerocross_span(out);
}

static const struct ace2k_mains_ops board_ops = {
    .zerocross_count = zerocross_count,
    .zerocross_rejects = zerocross_rejects,
    .span = span,
};

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    ace2k_mains_tick(&ace2k_mains_instance, now_ms);
    if (ace2k_report_due(&ace2k_mains_report, now_ms)) {
        ace2k_mains_report_due = true;
    }
    // A report that the last pass held, because its frame did not fit, leaves on this one.
    if (ace2k_mains_report_due) {
        sched_wake_task(&ace2k_mains_wake);
    }
}

// True when the report was queued. False when it did not fit, and its flag holds it for the next
// tick (tx.h; report.h gives the order).
static bool report(void)
{
    const struct ace2k_mains *m = &ace2k_mains_instance;
    return ACE2K_SENDF("ace2k_mains_state hz10=%hu present=%c rejects=%u", m->hz10,
                       m->present ? 1 : 0, m->rejects);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static. The only
// wake is the report. It is tried (tx.h), and held with its flag when it did not fit.
void ace2k_mains_binding_task(void)
{
    if (!sched_check_wake(&ace2k_mains_wake)) {
        return;
    }
    if (!ace2k_mains_report_due) {
        return; // nothing due: a wake raised and served in one pass. No report.
    }
    (void)ace2k_report_try(&ace2k_mains_report_due, report);
}
DECL_TASK(ace2k_mains_binding_task);

static void ensure_ready(void)
{
    if (ace2k_mains_ready) {
        return;
    }
    // The counter first, because the core's init reads it once.
    ace2k_zerocross_init();
    ace2k_mains_init(&ace2k_mains_instance, &board_ops, NULL, ace2k_tick_now_ms(),
                     timer_from_us(1000U)); // the span's clock: timer ticks
    // A full registry is a programming error. From init context this ends in a fault and a
    // watchdog reset. Either way it is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_mains_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_mains_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_mains_binding_init);

const struct ace2k_mains *ace2k_mains_binding(void)
{
    ensure_ready();
    return &ace2k_mains_instance;
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_mains_cmd_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_mains_report, &ace2k_mains_report_due, args[0],
                       ACE2K_MAINS_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_mains_cmd_query, "ace2k_mains_query rest_ticks=%u");
