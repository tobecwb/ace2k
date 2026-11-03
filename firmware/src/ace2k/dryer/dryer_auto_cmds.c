// Binding of the dryer's automatic modes. The instance lives in dryer_cmds.c, beside the dryer,
// because its tick must run right after the dryer's. This file holds the commands and the 1 Hz
// state report.
// - The commands bracket each core call with interrupts masked, because the tick runs in the
//   timer interrupt. Each fetches the instance before the bracket: the first fetch sets it up.
// - The dryer's task sends the report, right after the dryer's (dryer_cmds.c). sendf never runs
//   in timer context.
// - The modes' block reaches the page with the dryer's log, in the same write, from the dryer's
//   task (dryer_cmds.c): only with the gate idle, and with the log's back-off.
// A refused command is answered with its reason, never with a shutdown.
#include "dryer/dryer_auto.h"
#include "dryer/dryer_cmds.h" // ace2k_dryer_auto_binding, ace2k_dryer_binding_respond
#include "core/report.h"      // ace2k_report_due, ace2k_report_try
#include "core/report_cmds.h" // ace2k_report_query
#include "core/util.h"        // ACE2K_EINVAL
#include "ace2k_board/tx.h"   // ACE2K_SENDF
#include "ace2k_board/tick.h" // ace2k_tick_now_ms
#include "board/irq.h"        // irq_save, irq_restore
#include "command.h"          // DECL_COMMAND, sendf

enum ace2k_dryer_auto_op {
    ACE2K_DRYER_AUTO_OP_MODE = 0,
    ACE2K_DRYER_AUTO_OP_LIMITS = 1,
    ACE2K_DRYER_AUTO_OP_PROFILE = 2,
    ACE2K_DRYER_AUTO_OP_TIMING = 3,
};
#define ACE2K_DRYER_AUTO_REASON_RANGE 1U

static struct ace2k_report ace2k_dryer_auto_report;
static volatile bool ace2k_dryer_auto_report_due;

static void respond(enum ace2k_dryer_auto_op op, int rc)
{
    sendf("ace2k_dryer_auto_response op=%c accepted=%c reason=%c", (uint8_t)op, rc == 0 ? 1U : 0U,
          rc == 0 ? 0U : ACE2K_DRYER_AUTO_REASON_RANGE);
}

// Runs in the timer interrupt, right after the modes' tick. It touches only this file's report,
// which stays off until the host's first query.
void ace2k_dryer_auto_binding_tick(uint32_t now_ms)
{
    if (ace2k_report_due(&ace2k_dryer_auto_report, now_ms)) {
        ace2k_dryer_auto_report_due = true;
    }
}

// Also true for a report that the last pass held, because its frame did not fit.
bool ace2k_dryer_auto_binding_due(void)
{
    return ace2k_dryer_auto_report_due;
}

static bool report(void)
{
    const struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    struct ace2k_dryer_auto_status st;
    irqstatus_t flag = irq_save();
    ace2k_dryer_auto_status(a, &st, ace2k_tick_now_ms());
    irq_restore(flag);
    return ACE2K_SENDF("ace2k_dryer_auto_state mode=%c limit=%c max=%c failures=%c flags=%c "
                       "profile_c=%c profile_min=%hu cold=%c own=%c bound=%c rh=%hu above_s=%u",
                       st.mode, st.limit_pct, st.max_failures, st.failures, st.flags, st.profile_c,
                       st.profile_min, st.cold, st.own, st.bound, st.rh_pct10, st.above_s);
}

// Runs in the dryer's task. False: the frame did not fit, and the report stays due.
bool ace2k_dryer_auto_binding_send(void)
{
    if (!ace2k_dryer_auto_report_due) {
        return true;
    }
    return ace2k_report_try(&ace2k_dryer_auto_report_due, report);
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *). The checks run
// on the raw arguments. A value past its field is out of range, and it is never narrowed into
// the field.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_auto_mode(uint32_t *args)
{
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    int rc = -ACE2K_EINVAL;
    if (args[0] <= UINT8_MAX) {
        irqstatus_t flag = irq_save();
        rc = ace2k_dryer_auto_mode(a, (uint8_t)args[0]);
        irq_restore(flag);
    }
    respond(ACE2K_DRYER_AUTO_OP_MODE, rc);
}
DECL_COMMAND(ace2k_dryer_cmd_auto_mode, "ace2k_dryer_auto_mode mode=%c");

// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_auto_limits(uint32_t *args)
{
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    uint32_t limit_pct = args[0];
    uint32_t max_failures = args[1];
    int rc = -ACE2K_EINVAL;
    if (limit_pct <= UINT8_MAX && max_failures <= UINT8_MAX) {
        irqstatus_t flag = irq_save();
        rc = ace2k_dryer_auto_limits(a, (uint8_t)limit_pct, (uint8_t)max_failures);
        irq_restore(flag);
    }
    respond(ACE2K_DRYER_AUTO_OP_LIMITS, rc);
}
DECL_COMMAND(ace2k_dryer_cmd_auto_limits, "ace2k_dryer_auto_limits limit_pct=%c max_failures=%c");

// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_auto_profile(uint32_t *args)
{
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    uint32_t target_c = args[0];
    uint32_t minutes = args[1];
    int rc = -ACE2K_EINVAL;
    if (target_c <= UINT8_MAX && minutes <= UINT16_MAX) {
        irqstatus_t flag = irq_save();
        rc = ace2k_dryer_auto_profile(a, (uint8_t)target_c, (uint16_t)minutes);
        irq_restore(flag);
    }
    respond(ACE2K_DRYER_AUTO_OP_PROFILE, rc);
}
DECL_COMMAND(ace2k_dryer_cmd_auto_profile, "ace2k_dryer_auto_profile target_c=%c minutes=%hu");

// The bench's overrides, in seconds; 0 keeps the default. RAM only, so a reset brings the
// defaults back.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_auto_timing(uint32_t *args)
{
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    uint32_t hold_s = args[0];
    uint32_t window_s = args[1];
    uint32_t link_s = args[2];
    irqstatus_t flag = irq_save();
    int rc = ace2k_dryer_auto_timing_s(a, hold_s, window_s, link_s);
    irq_restore(flag);
    respond(ACE2K_DRYER_AUTO_OP_TIMING, rc);
}
DECL_COMMAND(ace2k_dryer_cmd_auto_timing,
             "ace2k_dryer_auto_timing hold_s=%u cold_window_s=%u link_s=%u");

// A start bound to the host (the print mode). It ends when the link stays down for the link
// time. It answers on the dryer's own response, with op 3.
// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_start_bound(uint32_t *args)
{
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    uint32_t target_c = args[0];
    uint32_t minutes = args[1];
    enum ace2k_dryer_refusal r = ACE2K_DRYER_RANGE;
    if (target_c <= UINT8_MAX && minutes <= UINT16_MAX) {
        irqstatus_t flag = irq_save();
        r = ace2k_dryer_auto_start_bound(a, (uint8_t)target_c, (uint16_t)minutes,
                                         ace2k_tick_now_ms());
        irq_restore(flag);
    }
    ace2k_dryer_binding_respond(ACE2K_DRYER_OP_START_BOUND, r);
}
DECL_COMMAND(ace2k_dryer_cmd_start_bound, "ace2k_dryer_start_bound target_c=%c minutes=%hu");

// The print mode's stop. It stops the running cycle only when it is bound to the host; over any
// other cycle it answers OK and does nothing. It answers on the dryer's own response, with op 4.
void ace2k_dryer_cmd_stop_bound(uint32_t *args)
{
    (void)args;
    struct ace2k_dryer_auto *a = ace2k_dryer_auto_binding();
    irqstatus_t flag = irq_save();
    enum ace2k_dryer_refusal r = ace2k_dryer_auto_stop_bound(a, ace2k_tick_now_ms());
    irq_restore(flag);
    ace2k_dryer_binding_respond(ACE2K_DRYER_OP_STOP_BOUND, r);
}
DECL_COMMAND(ace2k_dryer_cmd_stop_bound, "ace2k_dryer_stop_bound");

// cppcheck-suppress constParameterPointer
void ace2k_dryer_cmd_auto_query(uint32_t *args)
{
    ace2k_report_query(&ace2k_dryer_auto_report, &ace2k_dryer_auto_report_due, args[0],
                       ACE2K_DRYER_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_dryer_cmd_auto_query, "ace2k_dryer_auto_query rest_ticks=%u");
