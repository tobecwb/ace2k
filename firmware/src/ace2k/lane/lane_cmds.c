// Binding of the lane module: the board's encoder timers and FG counters into the core every
// tick, with link_ok from the serial driver (ace2k_board/serial.h: a host heard from within
// ACE2K_LINK_LOST_MS — its clock queries arrive about once a second); with CONFIG_ACE2K_MOTOR
// the motor lines through ace2k_board/motor.c, the shutdown handler that stops every motor first
// through the board and then in the core (the feed's rule 6), the veto on
// ace2k_bootloader_enter while a lane moves (the feed's rule 10), the dictionary's bounds (the reserved pins
// are the board driver's, ace2k_board/motor.c); the report of the four lanes from a task (sendf
// never runs in timer context); the reset and scale commands.  Every core call from command
// context is bracketed with interrupts masked: the tick ends moves in the timer interrupt.
#include "lane/lane_cmds.h"
#include "ace2k_board/encoder.h"
#include "ace2k_board/fg.h"
#include "ace2k_board/serial.h" // ace2k_link_ok
#include "ace2k_board/tick.h"
#include "core/tx.h" // ACE2K_SENDF
#include "core/report.h"
#include "core/util.h"  // ace2k_put_i32_le, ace2k_put_u32_le
#include "autoconf.h"   // CONFIG_ACE2K_MOTOR
#include "board/irq.h"  // irq_save, irq_restore
#include "board/misc.h" // timer_from_us
#include "command.h"    // DECL_COMMAND, DECL_CONSTANT, sendf, shutdown
#include "sched.h"      // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_MOTOR
#include "ace2k_board/motor.h"
#include "core/guard_cmds.h" // ace2k_guard_binding, ace2k_guard_register_veto
#endif

#define ACE2K_LANE_REPORT_PHASE_MS 50U /* the phase: report.h */

static struct ace2k_lane ace2k_lane_instance;
static struct task_wake ace2k_lane_wake;
static struct ace2k_report ace2k_lane_report;
static volatile bool ace2k_lane_report_due;
static bool ace2k_lane_ready;
// The link_ok of the last lane tick: written there, read by the feed tick of the same interrupt
// (its slot comes after this one), so both cores act on one sample per tick.
static bool ace2k_lane_link_ok_sample;
// The one reset veto (the feed binding's): set at init, read under the mask of a reset.
static ace2k_lane_reset_veto_fn ace2k_lane_reset_veto;
static void *ace2k_lane_reset_veto_ctx;

static uint16_t encoder_read(void *ctx, uint8_t lane)
{
    (void)ctx;
    return ace2k_encoder_read(lane);
}

static uint32_t fg_read(void *ctx, uint8_t lane)
{
    (void)ctx;
    return ace2k_fg_read(lane);
}

#if CONFIG_ACE2K_MOTOR
DECL_CONSTANT("ACE2K_LANE_SPEED_MIN_UM_S", ACE2K_LANE_SPEED_MIN_UM_S);
DECL_CONSTANT("ACE2K_LANE_SPEED_MAX_UM_S", ACE2K_LANE_SPEED_MAX_UM_S);
DECL_CONSTANT("ACE2K_LANE_MOVE_MAX_UM", ACE2K_LANE_MOVE_MAX_UM);
DECL_CONSTANT("ACE2K_LANE_DUTY_MIN_PCT", ACE2K_LANE_DUTY_MIN_PCT);
// The host budgets a waited move's timeout the way the core budgets its deadline: the landing
// at the floor speed, the rest at the commanded one.
DECL_CONSTANT("ACE2K_LANE_LANDING_UM", ACE2K_LANE_LANDING_UM);

static void pwm_set(void *ctx, uint8_t lane, uint8_t duty_pct)
{
    (void)ctx;
    ace2k_motor_pwm(lane, duty_pct);
}

static void run_set(void *ctx, uint8_t lane, bool on)
{
    (void)ctx;
    ace2k_motor_run(lane, on);
}

static void dir_set(void *ctx, uint8_t lane, bool reverse)
{
    (void)ctx;
    ace2k_motor_dir(lane, reverse);
}

static bool moving_veto(void *ctx)
{
    (void)ctx;
    return ace2k_lane_any_moving(&ace2k_lane_instance);
}
#else
// Without the motor flag the ops that would drive a line do nothing: the core never calls
// them (no move can start — no command reaches ace2k_lane_move), and the image configures no
// motor pin (the feed's rule 11).
static void pwm_set(void *ctx, uint8_t lane, uint8_t duty_pct)
{
    (void)ctx;
    (void)lane;
    (void)duty_pct;
}

static void run_set(void *ctx, uint8_t lane, bool on)
{
    (void)ctx;
    (void)lane;
    (void)on;
}

static void dir_set(void *ctx, uint8_t lane, bool reverse)
{
    (void)ctx;
    (void)lane;
    (void)reverse;
}
#endif

static const struct ace2k_lane_ops ace2k_lane_board_ops = {
    .encoder_read = encoder_read,
    .fg_read = fg_read,
    .pwm_set = pwm_set,
    .run_set = run_set,
    .dir_set = dir_set,
};

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
#if CONFIG_ACE2K_MOTOR
    // The duty write's own check covers a move in progress; this one covers the idle unit, so a
    // TIM2 remap lost while idle is restored before the next move starts.
    ace2k_motor_remap_check();
#endif
    ace2k_lane_link_ok_sample = ace2k_link_ok();
    ace2k_lane_tick(&ace2k_lane_instance, now_ms, ace2k_lane_link_ok_sample);
    if (ace2k_report_due(&ace2k_lane_report, now_ms)) {
        ace2k_lane_report_due = true;
    }
    // a report the last pass held (its frame did not fit) leaves on this one
    if (ace2k_lane_report_due) {
        sched_wake_task(&ace2k_lane_wake);
    }
}

// True when the report was queued; false when it did not fit and its flag holds it for the next
// tick.  The arrays are packed before the frame is tried: a try needs the encoded frame.
static bool report_counters(void)
{
    // One frame for the four lanes, not one per lane (report.h).  encoder_um: four int32
    // little-endian; fg: four uint32 little-endian.
    uint8_t encoder_um[4 * ACE2K_LANE_COUNT];
    uint8_t fg[4 * ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        ace2k_put_i32_le(encoder_um + 4 * lane, ace2k_lane_encoder_um(&ace2k_lane_instance, lane));
        ace2k_put_u32_le(fg + 4 * lane, ace2k_lane_fg(&ace2k_lane_instance, lane));
    }
    return ACE2K_SENDF("ace2k_lane_counters encoder_um=%*s fg=%*s", (uint8_t)sizeof encoder_um,
                       encoder_um, (uint8_t)sizeof fg, fg);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.  The
// report is tried, never dropped (tx.h, report.h): one that does not fit is held by its flag for
// the next tick.
void ace2k_lane_binding_task(void)
{
    if (!sched_check_wake(&ace2k_lane_wake)) {
        return;
    }
    if (ace2k_lane_report_due) {
        (void)ace2k_report_try(&ace2k_lane_report_due, report_counters);
    }
}
DECL_TASK(ace2k_lane_binding_task);

static void ensure_ready(void)
{
    if (ace2k_lane_ready) {
        return;
    }
    ace2k_encoder_init();
    ace2k_fg_init();
#if CONFIG_ACE2K_MOTOR
    ace2k_motor_init();
#endif
    ace2k_lane_init(&ace2k_lane_instance, &ace2k_lane_board_ops, NULL);
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
#if CONFIG_ACE2K_MOTOR
    if (ace2k_guard_register_veto(ace2k_guard_binding(), moving_veto, NULL) != 0) {
        shutdown("ace2k: veto slots exhausted");
    }
#endif
    ace2k_lane_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_lane_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_lane_binding_init);

struct ace2k_lane *ace2k_lane_binding(void)
{
    ensure_ready();
    return &ace2k_lane_instance;
}

bool ace2k_lane_binding_link_ok(void)
{
    return ace2k_lane_link_ok_sample;
}

#if CONFIG_ACE2K_MOTOR
// Interrupts are disabled here (Klipper's run_shutdown).  The board first — certain, whatever
// the core's state — then the core, so every move reports ACE2K_LANE_SHUTDOWN.
void ace2k_lane_binding_shutdown(void)
{
    ace2k_motor_stop_all();
    ace2k_lane_abort_all(&ace2k_lane_instance, ACE2K_LANE_SHUTDOWN, ace2k_tick_now_ms());
}
DECL_SHUTDOWN(ace2k_lane_binding_shutdown);
#endif

int ace2k_lane_binding_register_reset_veto(ace2k_lane_reset_veto_fn fn, void *ctx)
{
    if (fn == NULL) {
        return -ACE2K_EINVAL;
    }
    if (ace2k_lane_reset_veto != NULL) {
        return -ACE2K_EFULL;
    }
    ace2k_lane_reset_veto = fn;
    ace2k_lane_reset_veto_ctx = ctx;
    return 0;
}

static bool reset_vetoed(uint8_t lane)
{
    if (ace2k_lane_reset_veto == NULL) {
        return false;
    }
    return ace2k_lane_reset_veto(ace2k_lane_reset_veto_ctx, lane);
}

int ace2k_lane_binding_reset(uint8_t lane)
{
    ensure_ready();
    // The range first, so the veto only ever sees a lane the core would accept: 0..3 or
    // ACE2K_LANE_ALL (the callback's contract, lane_cmds.h).
    if (lane >= ACE2K_LANE_COUNT && lane != ACE2K_LANE_ALL) {
        return -ACE2K_EINVAL;
    }
    // One masked section for the veto and the core's pending mask: the tick may change what the
    // veto reads (the feed's automatic load starts from it) and consumes the mask in the timer
    // interrupt.
    irqstatus_t flag = irq_save();
    int rc = reset_vetoed(lane) ? -ACE2K_EREFUSED : ace2k_lane_reset(&ace2k_lane_instance, lane);
    irq_restore(flag);
    return rc;
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_lane_cmd_counters_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = args[0] / timer_from_us(1000U);
    // The tick reads the pair in the timer interrupt; both stores in one step, so it never
    // sees the new period against the old deadline.
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_lane_report, rest_ms, ace2k_tick_now_ms(), ACE2K_LANE_REPORT_PHASE_MS);
    ace2k_lane_report_due = false; // a report held from before the query: the new grid's
    irq_restore(flag);
}
DECL_COMMAND(ace2k_lane_cmd_counters_query, "ace2k_lane_counters_query rest_ticks=%u");

// A lane outside 0..3 and 255 is a host bug, not a lane condition: Klipper's idiom for a bad
// command is a shutdown with a message.  A reset while the lane moves, or the veto objects, is
// refused and answered as such: the host refuses ACE_COUNTERS_RESET on its own picture first,
// and the unit's refusal is the backstop for the blind window of the host's 1 Hz report.
// cppcheck-suppress constParameterPointer
void ace2k_lane_cmd_counters_reset(uint32_t *args)
{
    uint8_t lane = (uint8_t)args[0];
    int rc = ace2k_lane_binding_reset(lane);
    if (rc == -ACE2K_EINVAL) {
        shutdown("ace2k_lane_counters_reset: lane out of range");
    }
    sendf("ace2k_lane_counters_reset_response lane=%c accepted=%c", lane, rc == 0 ? 1 : 0);
}
DECL_COMMAND(ace2k_lane_cmd_counters_reset, "ace2k_lane_counters_reset lane=%c");

// The encoder scale from printer.cfg (a config command at connect).  A value outside ±20 % of
// the default, or a lane out of range, is a host bug.
// cppcheck-suppress constParameterPointer
void ace2k_lane_cmd_scale_set(uint32_t *args)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    int rc = ace2k_lane_scale_set(&ace2k_lane_instance, (uint8_t)args[0], args[1]);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_lane_scale_set: lane out of range or scale outside ±20 %");
    }
}
DECL_COMMAND(ace2k_lane_cmd_scale_set, "ace2k_lane_scale_set lane=%c um_per_count_x10=%u");
