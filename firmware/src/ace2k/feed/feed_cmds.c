// Binding of the feed module.  It holds these parts.
// - Every tick, it feeds the debounced states of the sensors, the lane instance and the link into
//   the core.
// - A task sends the events and the periodic state report, because sendf never runs in timer
//   context.
// - The commands.  Each core call runs with interrupts masked, because the tick ends moves and pops
//   results in the timer interrupt.
// - The lane LEDs, which follow the mode.
// A refusal is answered (ace2k_feed_start_response, ace2k_feed_speed_set_response,
// ace2k_feed_stop_when_response), never shut down.  A busy lane, an error, missing filament, a lost
// link, or a lane with no move to change are conditions of the unit.  A lane or a mode outside the
// range of the dictionary is a bug in the host, and takes the Klipper way out.
#include "feed/feed_cmds.h"
#include "lane/lane_cmds.h"    // ace2k_lane_binding, ace2k_lane_binding_link_ok, the reset veto
#include "lane/sensors_cmds.h" // ace2k_sensors_binding
#include "ace2k_board/args.h"  // ACE2K_ARG_U8, ACE2K_ARG_U8_MAX, ACE2K_ARG_FLAG
#include "lane/lane_args.h"    // ACE2K_ARG_LANE, ACE2K_ARG_LANE_OR_ALL
#include "ace2k_board/tick.h"
#include "core/report_cmds.h" // ace2k_report_query
#include "ace2k_board/tx.h"   // ACE2K_SENDF
#include "core/util.h"        // ace2k_put_u16_le, ace2k_put_u32_le
#include "autoconf.h"         // CONFIG_ACE2K_LED, CONFIG_ACE2K_RFID_READ
#include "board/irq.h"        // irq_save, irq_restore
#include "command.h"          // DECL_COMMAND, DECL_CONSTANT, sendf, shutdown
#include "sched.h" // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_LED
#include "lane/led_cmds.h" // ace2k_led_binding_set_lane
#endif
#if CONFIG_ACE2K_RFID_READ
#include "rfid/watch_cmds.h" // ace2k_rfid_binding_feed_bits
#endif

// The report phase (report.h).  The grid is sensors 0 (10 Hz, so every tenth tick), env 30, lane
// 50, mains 70, feed 80.  A report that this task holds for one tick, because its frame did not
// fit, lands on 90, the dryer's phase.  The two then try on the same tick, the feed first, and the
// transmit rule holds whichever does not fit (tx.h).  So a frame is held, never lost.
#define ACE2K_FEED_REPORT_PHASE_MS 80U
// The counters of the feed-forward (ace2k_feed_ff_state) have their own phase, 40, which is free on
// the grid (report.h).  So this task's two frames never fall due on the same tick.
#define ACE2K_FEED_FF_REPORT_PHASE_MS 40U
// Every frame of this task is tried, never dropped (tx.h, report.h).  Klipper refuses a frame that
// does not fit its transmit buffer, and this task holds the frame for the next tick.  The report is
// held by its flag, and an event at the ring's tail.  The wake of the next tick brings the task
// back, because the tick is the only waker of the task.  The due report goes first, then the
// events, oldest first, while they fit.  An event leaves on the first tick where its frame fits.
// Normally that is the next one, within 10 ms.
#define ACE2K_FEED_EVENT_FMT                                                                       \
    "ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c"

// Klipper's DECL_CONSTANT encodes its value with signed literals inside the macro.  So the lint's
// signed-bitwise check is off for these declarations only.
// NOLINTBEGIN(bugprone-signed-bitwise)
// The host bounds the comparator's windows by this value (the thresholds command, below), without a
// copy of its own.
DECL_CONSTANT("ACE2K_FEED_ASSIST_BURST_UM", ACE2K_FEED_ASSIST_BURST_UM);
// The supervisors' own windows.  The host sends them back at every connect when printer.cfg carries
// no threshold key.  So a removed key reaches the unit with no MCU restart.
DECL_CONSTANT("ACE2K_FEED_SLIP_CHECK_UM", ACE2K_FEED_SLIP_CHECK_UM);
DECL_CONSTANT("ACE2K_FEED_SLIP_ALLOW_UM", ACE2K_FEED_SLIP_ALLOW_UM);
DECL_CONSTANT("ACE2K_FEED_STALL_CHECK_UM", ACE2K_FEED_STALL_CHECK_UM);
// The unload's tail, its travel past the bay entrance.  The host's unload wait counts it.
DECL_CONSTANT("ACE2K_FEED_UNLOAD_TAIL_UM", ACE2K_FEED_UNLOAD_TAIL_UM);
// The clear of an unload: its default and bounds.  The host checks its key against them before it
// sends ace2k_feed_unload_set.
DECL_CONSTANT("ACE2K_FEED_UNLOAD_CLEAR_UM", ACE2K_FEED_UNLOAD_CLEAR_UM);
DECL_CONSTANT("ACE2K_FEED_UNLOAD_CLEAR_UM_MIN", ACE2K_FEED_UNLOAD_CLEAR_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_UNLOAD_CLEAR_UM_MAX", ACE2K_FEED_UNLOAD_CLEAR_UM_MAX);
// The defaults and bounds of the tip snag, named as those of the follow.  The host reads them.  So
// ACEK_SNAG_SET can change one value and send the others as they are.  The host also checks the
// values against the bounds before it sends.  The duty has no _MIN, because its least value is 0.
DECL_CONSTANT("ACE2K_FEED_SNAG_FWD_UM", ACE2K_FEED_SNAG_FWD_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_FWD_UM_MIN", ACE2K_FEED_SNAG_FWD_MIN_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_FWD_UM_MAX", ACE2K_FEED_SNAG_FWD_MAX_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_LAG_UM", ACE2K_FEED_SNAG_LAG_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_LAG_UM_MIN", ACE2K_FEED_SNAG_LAG_MIN_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_LAG_UM_MAX", ACE2K_FEED_SNAG_LAG_MAX_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_DUTY_PCT", ACE2K_FEED_SNAG_DUTY_PCT);
DECL_CONSTANT("ACE2K_FEED_SNAG_DUTY_PCT_MAX", ACE2K_FEED_SNAG_DUTY_MAX_PCT);
DECL_CONSTANT("ACE2K_FEED_SNAG_BACK_UM", ACE2K_FEED_SNAG_BACK_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_BACK_UM_MIN", ACE2K_FEED_SNAG_BACK_MIN_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_BACK_UM_MAX", ACE2K_FEED_SNAG_BACK_MAX_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_REST_MS", ACE2K_FEED_SNAG_REST_MS);
DECL_CONSTANT("ACE2K_FEED_SNAG_REST_MS_MIN", ACE2K_FEED_SNAG_REST_MIN_MS);
DECL_CONSTANT("ACE2K_FEED_SNAG_REST_MS_MAX", ACE2K_FEED_SNAG_REST_MAX_MS);
DECL_CONSTANT("ACE2K_FEED_SNAG_FREE_UM", ACE2K_FEED_SNAG_FREE_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_FREE_UM_MIN", ACE2K_FEED_SNAG_FREE_MIN_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_FREE_UM_MAX", ACE2K_FEED_SNAG_FREE_MAX_UM);
// The follow's defaults and bounds.  The host checks ACEK_FOLLOW_SET against them before it sends,
// and sends one changed value with the others as they are.
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS", ACE2K_FEED_FOLLOW_FLIP_MS);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS_MIN", ACE2K_FEED_FOLLOW_FLIP_MS_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS_MAX", ACE2K_FEED_FOLLOW_FLIP_MS_MAX);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM", ACE2K_FEED_FOLLOW_TAKE_UM);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM_MIN", ACE2K_FEED_FOLLOW_TAKE_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM_MAX", ACE2K_FEED_FOLLOW_TAKE_UM_MAX);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM", ACE2K_FEED_FOLLOW_TAIL_UM);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM_MIN", ACE2K_FEED_FOLLOW_TAIL_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM_MAX", ACE2K_FEED_FOLLOW_TAIL_UM_MAX);
// The feed-forward's defaults and bounds.  The host checks ACEK_FF_SET against them before it
// sends, and sends one changed value with the other as it is.
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM", ACE2K_FEED_FF_CHUNK_UM);
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM_MIN", ACE2K_FEED_FF_CHUNK_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM_MAX", ACE2K_FEED_FF_CHUNK_UM_MAX);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S", ACE2K_FEED_FF_PULSE_UM_S);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S_MIN", ACE2K_FEED_FF_PULSE_UM_S_MIN);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S_MAX", ACE2K_FEED_FF_PULSE_UM_S_MAX);
// The held stop's defaults and bounds.  The host checks its keys against them before it sends
// ace2k_feed_held_set.
DECL_CONSTANT("ACE2K_FEED_HELD_UM", ACE2K_FEED_HELD_UM);
DECL_CONSTANT("ACE2K_FEED_HELD_UM_MIN", ACE2K_FEED_HELD_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_HELD_UM_MAX", ACE2K_FEED_HELD_UM_MAX);
DECL_CONSTANT("ACE2K_FEED_HELD_MS", ACE2K_FEED_HELD_MS);
DECL_CONSTANT("ACE2K_FEED_HELD_MS_MIN", ACE2K_FEED_HELD_MS_MIN);
DECL_CONSTANT("ACE2K_FEED_HELD_MS_MAX", ACE2K_FEED_HELD_MS_MAX);
// The feed-forward takes back a long retraction, which is a negative base rate.  The host sends
// such a rate only to an image that carries this constant.
DECL_CONSTANT("ACE2K_FEED_FF_REVERSE", 1);
// A rollback started with free_tip forgives every taut trip whose filament came back free_um past
// the last one.  By this constant, the host knows that the image reads the flag on a rollback.
DECL_CONSTANT("ACE2K_FEED_FREE_ROLLBACK", 1);
// A speed set on a following lane is refused with this reason.  By it, the host knows that the
// image judges the follow itself.  For an image without it, the host refuses by itself.
DECL_CONSTANT("ACE2K_FEED_REFUSED_FOLLOWING", ACE2K_FEED_REFUSED_FOLLOWING);
// The grip's default and bounds.  The host checks its grip against them before it sends.
DECL_CONSTANT("ACE2K_FEED_GRIP_UM", ACE2K_FEED_GRIP_UM);
DECL_CONSTANT("ACE2K_FEED_GRIP_UM_MIN", ACE2K_FEED_GRIP_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_GRIP_UM_MAX", ACE2K_FEED_GRIP_UM_MAX);
// NOLINTEND(bugprone-signed-bitwise)

static struct ace2k_feed ace2k_feed_instance;
static const struct ace2k_sensors *ace2k_feed_sensors;
static struct task_wake ace2k_feed_wake;
static struct ace2k_report ace2k_feed_report;
static volatile bool ace2k_feed_report_due;
static struct ace2k_report ace2k_feed_ff_report;
static volatile bool ace2k_feed_ff_report_due;
static bool ace2k_feed_ready;

#if CONFIG_ACE2K_LED
static void leds_follow(void)
{
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        ace2k_led_binding_set_lane(
            lane, ace2k_feed_led_pattern(ace2k_feed_mode(&ace2k_feed_instance, lane),
                                         ace2k_feed_sensors->lane[lane].insert));
    }
}
#endif

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    struct ace2k_feed_inputs in = { 0 };
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        const struct ace2k_sensors_lane *s = &ace2k_feed_sensors->lane[lane];
        if (s->insert) {
            in.insert |= (uint8_t)(1U << lane);
        }
        if (s->rest) {
            in.rest |= (uint8_t)(1U << lane);
        }
        if (s->pushed) {
            in.pushed |= (uint8_t)(1U << lane);
        }
    }
    in.pulled_any = ace2k_feed_sensors->pulled;
    in.insert_unknown = (bool)!ace2k_sensors_insert_known(ace2k_feed_sensors); // the scan stopped
    in.link_ok = ace2k_lane_binding_link_ok(); // the lane tick's sample, one per tick
#if CONFIG_ACE2K_RFID_READ
    ace2k_rfid_binding_feed_bits(&in.tag_hold, &in.tag_read, &in.tag_lost, &in.tag_confirm,
                                 &in.tag_confirm_rev, &in.tag_confirm_epoch);
    in.in_use_valid = ace2k_rfid_binding_in_use(&in.in_use); // the mask 0 while not valid
    in.tag_by_hand = ace2k_rfid_binding_by_hand();
#endif
    ace2k_feed_tick(&ace2k_feed_instance, now_ms, &in);
#if CONFIG_ACE2K_LED
    leds_follow();
#endif
    if (ace2k_report_due(&ace2k_feed_report, now_ms)) {
        ace2k_feed_report_due = true;
    }
    if (ace2k_report_due(&ace2k_feed_ff_report, now_ms)) {
        ace2k_feed_ff_report_due = true;
    }
    // The rest of the ring, and a report that the last pass held because its frame did not fit,
    // leave now.  The tick is the only waker of the task.
    if (ace2k_feed_has_events(&ace2k_feed_instance) || ace2k_feed_report_due ||
        ace2k_feed_ff_report_due) {
        sched_wake_task(&ace2k_feed_wake);
    }
}

// Returns true when the report was queued.  It returns false when the report did not fit, and then
// its flag holds it for the next tick.  The arrays are packed before the frame is tried, because a
// try needs the encoded frame.
static bool report(void)
{
    const struct ace2k_feed *f = &ace2k_feed_instance;
    uint8_t mode[ACE2K_LANE_COUNT];
    uint8_t error[ACE2K_LANE_COUNT];
    uint8_t speed[4 * ACE2K_LANE_COUNT];
    uint8_t duty[ACE2K_LANE_COUNT];
    uint8_t bursts[2 * ACE2K_LANE_COUNT];
    uint8_t seq[ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        mode[lane] = ace2k_feed_mode(f, lane);
        error[lane] = ace2k_feed_error(f, lane);
        ace2k_put_u32_le(speed + (4 * lane), ace2k_lane_speed_um_s(f->lane, lane));
        duty[lane] = ace2k_lane_duty_pct(f->lane, lane);
        ace2k_put_u16_le(bursts + (2 * lane), ace2k_feed_bursts(f, lane));
        seq[lane] = ace2k_feed_seq(f, lane);
    }
    // seq has one byte per lane: the sequence of the last start that the unit accepted from the
    // host on the lane, 0 until one.  So a host that reconnects continues after it.  It is kept
    // through idle and error.  A lane that runs the move of a host start reports the seq of that
    // start.  A mode of the unit itself leaves it as the seq of the last accepted host start.  The
    // host seeds its counter from it before its first start of a session.
    return ACE2K_SENDF(
        "ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s",
        (uint8_t)sizeof mode, mode, (uint8_t)sizeof error, error, (uint8_t)sizeof speed, speed,
        (uint8_t)sizeof duty, duty, (uint8_t)sizeof bursts, bursts, (uint8_t)sizeof seq, seq);
}

// The counters of the feed-forward since the last entry of each lane.  There are four arrays of
// uint16 × 4, little-endian, lane 0 first: doses, reverse doses, taut and full.  Then comes the
// epoch of each lane (uint8 × 4), which steps at every entry, so the host restarts its tally
// exactly when the epoch moves.  It returns true when the report was queued.  It returns false when
// the report did not fit, and then its flag holds it for the next tick.
static bool report_ff(void)
{
    const struct ace2k_feed *f = &ace2k_feed_instance;
    uint8_t doses[2 * ACE2K_LANE_COUNT];
    uint8_t back[2 * ACE2K_LANE_COUNT];
    uint8_t taut[2 * ACE2K_LANE_COUNT];
    uint8_t full[2 * ACE2K_LANE_COUNT];
    uint8_t epoch[ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        struct ace2k_feed_ff_counts c = ace2k_feed_ff_counts(f, lane);
        ace2k_put_u16_le(doses + (2 * lane), c.doses);
        ace2k_put_u16_le(back + (2 * lane), c.back_doses);
        ace2k_put_u16_le(taut + (2 * lane), c.taut_fixes);
        ace2k_put_u16_le(full + (2 * lane), c.full_fixes);
        epoch[lane] = c.epoch;
    }
    return ACE2K_SENDF("ace2k_feed_ff_state doses=%*s back=%*s taut=%*s full=%*s epoch=%*s",
                       (uint8_t)sizeof doses, doses, (uint8_t)sizeof back, back,
                       (uint8_t)sizeof taut, taut, (uint8_t)sizeof full, full,
                       (uint8_t)sizeof epoch, epoch);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
void ace2k_feed_task(void)
{
    if (!sched_check_wake(&ace2k_feed_wake)) {
        return;
    }
    // The events go first, oldest first.  Each is tried from the ring's tail, and dropped from it
    // only once queued.  So an event that does not fit stays for the next tick, with everything
    // behind it.  The due reports go once the ring is empty: a report shows the modes after every
    // event in the ring, so it must not reach the host before them (a lane reported idle before
    // the final event of its move).  The ring drains at every pass that has room, so the report
    // waits at most for the events of a tick.  The task never wakes itself.  The tick brings it
    // back while the ring holds anything or a report is pending.
    struct ace2k_feed_event ev;
    while (ace2k_feed_peek_event(&ace2k_feed_instance, &ev)) {
        if (!ACE2K_SENDF(ACE2K_FEED_EVENT_FMT, ev.lane, ev.kind, ev.mode, ev.motor_um,
                         ev.filament_um, ev.seq)) {
            break;
        }
        ace2k_feed_drop_event(&ace2k_feed_instance);
    }
    if (ace2k_feed_has_events(&ace2k_feed_instance)) {
        return;
    }
    if (ace2k_feed_report_due) {
        (void)ace2k_report_try(&ace2k_feed_report_due, report);
    }
    if (ace2k_feed_ff_report_due) {
        (void)ace2k_report_try(&ace2k_feed_ff_report_due, report_ff);
    }
}
DECL_TASK(ace2k_feed_task);

// The counters-reset veto of the lane binding.  The odometer bases of the feed span modes in which
// the lane core is idle between bursts, such as an assist waiting on the buffer, or a load settling
// after the insert edge.  So a reset must wait for idle or error here too.  It is called under the
// mask of the lane binding, after init, and is a plain read of the instance.
static bool reset_veto(void *ctx, uint8_t lane)
{
    (void)ctx;
    return ace2k_feed_lane_busy(&ace2k_feed_instance, lane);
}

static void ensure_ready(void)
{
    if (ace2k_feed_ready) {
        return;
    }
    // The lane binding goes first.  Its tick slot must come before this one, so that a move ends in
    // the lane's tick before the feed's tick of the same period reads the result.
    struct ace2k_lane *lane = ace2k_lane_binding();
    ace2k_feed_sensors = ace2k_sensors_binding();
    ace2k_feed_init(&ace2k_feed_instance, lane);
    // A full registry is a programming error.  From init context this ends in a fault and a
    // watchdog reset.  Either way, the failure is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    if (ace2k_lane_binding_register_reset_veto(reset_veto, NULL) != 0) {
        shutdown("ace2k: the lane's reset veto slot is taken");
    }
    ace2k_feed_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_feed_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_feed_binding_init);

// A Klipper shutdown ends every mode and suspends every start.  The tick outlives a shutdown
// (ace2k_board/tick.c), and the clock queries of the host keep link_ok up.  Otherwise an assist
// waiting between bursts, a load in its settle, or the automatic load would move the motor.
// Interrupts are disabled here (Klipper's run_shutdown), so there is no sendf.  The
// stopped_shutdown events leave from the task as usual, or not at all once the transport is gone.
// The order against the lane binding's handler does not matter, because each lane gets one event.
// Klipper calls every DECL_SHUTDOWN target by name from generated code, so it is not static.
void ace2k_feed_binding_shutdown(void)
{
    if (ace2k_feed_ready) {
        ace2k_feed_shutdown(&ace2k_feed_instance, ace2k_tick_now_ms());
    }
}
DECL_SHUTDOWN(ace2k_feed_binding_shutdown);

// The arguments of ace2k_feed_start, in the order of its format.
enum ace2k_feed_start_arg {
    ACE2K_FEED_START_ARG_LANE,
    ACE2K_FEED_START_ARG_MODE,
    ACE2K_FEED_START_ARG_LENGTH_UM,
    ACE2K_FEED_START_ARG_SPEED_UM_S,
    ACE2K_FEED_START_ARG_SEQ,
    ACE2K_FEED_START_ARG_FREE_TIP,
};

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_start(uint32_t *args)
{
    ensure_ready();
    uint8_t lane =
        ACE2K_ARG_LANE(args[ACE2K_FEED_START_ARG_LANE], "ace2k_feed_start: lane out of range");
    uint8_t mode = ACE2K_ARG_U8_MAX(args[ACE2K_FEED_START_ARG_MODE], ACE2K_FEED_CMD_ASSIST_BOTH,
                                    "ace2k_feed_start: mode out of range");
    // The host chooses the seq.  Any value 0..255 is accepted, and the events of the mode echo it.
    // The host never sends 0, which marks a motion that the firmware started on its own.  A seq
    // above 255 is a bug in the host, as for the other arguments.
    uint8_t seq =
        ACE2K_ARG_U8(args[ACE2K_FEED_START_ARG_SEQ], "ace2k_feed_start: seq out of range");
    bool free_tip = ACE2K_ARG_FLAG(args[ACE2K_FEED_START_ARG_FREE_TIP],
                                   "ace2k_feed_start: free_tip out of range");
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal verdict = ace2k_feed_start_tip(
        &ace2k_feed_instance, lane, (enum ace2k_feed_cmd)mode, args[ACE2K_FEED_START_ARG_LENGTH_UM],
        args[ACE2K_FEED_START_ARG_SPEED_UM_S], seq, free_tip, ace2k_tick_now_ms());
    irq_restore(flag);
    // reason is an enum ace2k_feed_refusal.  no_filament answers a feed, an assist, a load or an
    // unload on a lane whose insert sensor reads absent.  A rollback is accepted there.
    sendf("ace2k_feed_start_response lane=%c accepted=%c reason=%c", lane,
          verdict == ACE2K_FEED_ACCEPTED ? 1 : 0, (uint8_t)verdict);
}
DECL_COMMAND(ace2k_feed_cmd_start,
             "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c free_tip=%c");

// Arms the stop for the next ace2k_feed_start on the lane: held, at rest, both, or neither to
// disarm.  It carries the move's own held window, held_ms, where 0 means the unit's setting.  The
// start keeps its format, so an older host never sends this, and its starts run as before.
// A lane that is not idle is a condition of the unit.  It is answered accepted=0, with nothing
// armed and the start's reason: 1 busy (in a mode) or 2 in_error.  A held_ms other than 0 outside
// its bounds is answered accepted=0, with nothing armed and reason 5 bounds, whatever the lane's
// mode.  The host picks the window for each move from its speed, so a bad one is refused, never a
// shutdown.  A lane above 3, or a flag above 1, is a bug in the host.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_stop_when(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_stop_when: lane out of range");
    bool held = ACE2K_ARG_FLAG(args[1], "ace2k_feed_stop_when: held out of range");
    bool at_rest = ACE2K_ARG_FLAG(args[2], "ace2k_feed_stop_when: at_rest out of range");
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_stop_when(&ace2k_feed_instance, lane, held, at_rest, args[3]);
    uint8_t reason = ACE2K_FEED_ACCEPTED;
    if (rc == -ACE2K_EINVAL) {
        reason = ACE2K_FEED_REFUSED_BOUNDS; /* the lane is in range: the window is not */
    } else if (rc != 0) {
        reason = ace2k_feed_mode(&ace2k_feed_instance, lane) == ACE2K_FEED_ERROR
                     ? ACE2K_FEED_REFUSED_IN_ERROR
                     : ACE2K_FEED_REFUSED_BUSY;
    }
    irq_restore(flag);
    sendf("ace2k_feed_stop_when_response lane=%c accepted=%c reason=%c", lane, rc == 0 ? 1 : 0,
          reason);
}
DECL_COMMAND(ace2k_feed_cmd_stop_when,
             "ace2k_feed_stop_when lane=%c held=%c at_rest=%c held_ms=%hu");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_stop(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE_OR_ALL(args[0], "ace2k_feed_stop: lane out of range");
    irqstatus_t flag = irq_save();
    ace2k_feed_stop(&ace2k_feed_instance, lane, ace2k_tick_now_ms());
    irq_restore(flag);
    // The stopped events leave with the next tick, the only waker of the task.  Each leaves on the
    // first tick where its frame fits.  Four of them normally fit in one empty buffer, with nothing
    // else in it.
}
DECL_COMMAND(ace2k_feed_cmd_stop, "ace2k_feed_stop lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_clear(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_clear: lane out of range");
    irqstatus_t flag = irq_save();
    (void)ace2k_feed_clear(&ace2k_feed_instance, lane); // a no-op outside error
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_clear, "ace2k_feed_clear lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_speed_set(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_speed_set: lane out of range");
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal verdict =
        ace2k_feed_set_speed(&ace2k_feed_instance, lane, args[1], ace2k_tick_now_ms());
    irq_restore(flag);
    // reason is an enum ace2k_feed_refusal: no_move on an idle lane, in_error on a lane in error,
    // following on a following lane, and bounds for a speed out of bounds.  In each case, nothing
    // is kept for a later move.
    sendf("ace2k_feed_speed_set_response lane=%c accepted=%c reason=%c", lane,
          verdict == ACE2K_FEED_ACCEPTED ? 1 : 0, (uint8_t)verdict);
}
DECL_COMMAND(ace2k_feed_cmd_speed_set, "ace2k_feed_speed_set lane=%c speed_um_s=%u");

// The thresholds.  They are the keys from printer.cfg, or the defaults above.  They are sent as an
// init command at every connect.  A host older than v0.12.0 sends them as a config command, and
// only with a key.  The core refuses a set that breaks one of its rules
// (ace2k_feed_thresholds_check).  Such a set is a bug in the host, because the host refuses the
// same set before it sends.  So this guards against a host from another commit.  The shutdown names
// the rule.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_thresholds_set(uint32_t *args)
{
    ensure_ready();
    struct ace2k_feed_thresholds th = {
        .slip_check_um = args[0],
        .slip_allow_um = args[1],
        .stall_check_um = args[2],
    };
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_thresholds_set(&ace2k_feed_instance, &th);
    irq_restore(flag);
    if (rc == 0) {
        return;
    }
    enum ace2k_feed_thresholds_fault fault = ace2k_feed_thresholds_check(&th);
    if (fault == ACE2K_FEED_THRESHOLDS_ZERO) {
        shutdown("ace2k_feed_thresholds_set: a threshold of zero");
    }
    if (fault == ACE2K_FEED_THRESHOLDS_ALLOW_NOT_UNDER_CHECK) {
        shutdown("ace2k_feed_thresholds_set: slip_allow_um must be under slip_check_um");
    }
    // This is the burst rule.  check returns OK only when set accepts, so OK never gets here.
    shutdown("ace2k_feed_thresholds_set: stall_check_um and slip_check_um must be under an"
             " assist's burst (ACE2K_FEED_ASSIST_BURST_UM)");
}
DECL_COMMAND(ace2k_feed_cmd_thresholds_set,
             "ace2k_feed_thresholds_set slip_check_um=%u slip_allow_um=%u stall_check_um=%u");

// The arguments of ace2k_feed_snag_set, in the order of its format.
enum ace2k_feed_snag_arg {
    ACE2K_FEED_SNAG_ARG_FWD_UM,
    ACE2K_FEED_SNAG_ARG_LAG_UM,
    ACE2K_FEED_SNAG_ARG_DUTY_PCT,
    ACE2K_FEED_SNAG_ARG_BACK_UM,
    ACE2K_FEED_SNAG_ARG_REST_MS,
    ACE2K_FEED_SNAG_ARG_FREE_UM,
};

// The tip snag's values.  They are sent as a config command at connect when printer.cfg carries a
// snag key, and by ACEK_SNAG_SET on the bench.  A value outside its bounds, or a lag at or above
// the tolerance, is a bug in the host.  The host checks the same bounds before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_snag_set(uint32_t *args)
{
    ensure_ready();
    uint8_t duty_pct =
        ACE2K_ARG_U8_MAX(args[ACE2K_FEED_SNAG_ARG_DUTY_PCT], ACE2K_FEED_SNAG_DUTY_MAX_PCT,
                         "ace2k_feed_snag_set: a value out of bounds");
    struct ace2k_feed_snag_cfg cfg = {
        .fwd_um = args[ACE2K_FEED_SNAG_ARG_FWD_UM],
        .lag_um = args[ACE2K_FEED_SNAG_ARG_LAG_UM],
        .duty_pct = duty_pct,
        .back_um = args[ACE2K_FEED_SNAG_ARG_BACK_UM],
        .rest_ms = args[ACE2K_FEED_SNAG_ARG_REST_MS],
        .free_um = args[ACE2K_FEED_SNAG_ARG_FREE_UM],
    };
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_snag_set(&ace2k_feed_instance, &cfg);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_snag_set: a value out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_snag_set, "ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c"
                                      " back_um=%u rest_ms=%u free_um=%u");

// The follow's values.  They are sent as an init command at every connect, every value, keys or
// not, and again on every ACEK_FOLLOW_SET.  A value outside its bounds is a bug in the host.  The
// host checks the same bounds before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_follow_set(uint32_t *args)
{
    ensure_ready();
    struct ace2k_feed_follow_cfg cfg = {
        .flip_ms = args[0],
        .take_um = args[1],
        .tail_um = args[2],
    };
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_follow_set(&ace2k_feed_instance, &cfg);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_follow_set: a value out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_follow_set, "ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u");

// The feed-forward's values.  They are sent as an init command at every connect, and again on every
// ACEK_FF_SET.  A value outside its bounds is a bug in the host.  The host checks the same bounds
// before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_ff_set(uint32_t *args)
{
    ensure_ready();
    struct ace2k_feed_ff_cfg cfg = {
        .chunk_um = args[0],
        .pulse_um_s = args[1],
    };
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_ff_set(&ace2k_feed_instance, &cfg);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_ff_set: a value out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_ff_set, "ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u");

// The held stop's values.  A value outside its bounds is a bug in the host.  The host checks the
// same bounds before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_held_set(uint32_t *args)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_held_set(&ace2k_feed_instance, args[0], args[1]);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_held_set: a value out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_held_set, "ace2k_feed_held_set held_um=%u held_ms=%hu");

// The clear of an unload.  A value outside its bounds is a bug in the host.  The host checks the
// same bounds before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_unload_set(uint32_t *args)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_unload_set(&ace2k_feed_instance, args[0]);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_unload_set: a value out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_unload_set, "ace2k_feed_unload_set clear_um=%u");

// The lane's base rate for the feed-forward, signed: + feeds toward the head, and - takes a long
// retraction back.  The host sends it while the lane follows.  clear=1 also drops what the lane
// owes.  The rate is taken only in the follow.  Anywhere else it is ignored silently, because a
// base sent as the follow ends is a race, not a bug.  A lane out of range, a clear other than
// 0 or 1, or a rate that the core refuses (beyond the lane's top speed either way) is a bug in the
// host.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_base(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_base: lane out of range");
    bool clear = ACE2K_ARG_FLAG(args[2], "ace2k_feed_base: clear out of bounds");
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_base_set(&ace2k_feed_instance, lane, (int32_t)args[1], clear);
    irq_restore(flag);
    // -ACE2K_EREFUSED: a lane not in the follow, ignored by design (above)
    if (rc == -ACE2K_EINVAL) {
        shutdown("ace2k_feed_base: rate out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_base, "ace2k_feed_base lane=%c rate_um_s=%i clear=%c");

// The lane's grip.  Its next automatic load pulls grip_um alone, and ends loaded.  0 clears the
// grip.  A lane out of range, or a grip outside its bounds, is a bug in the host.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_lane_grip(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_lane_grip: lane out of range");
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_grip_set(&ace2k_feed_instance, lane, args[1]);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_lane_grip: grip out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_lane_grip, "ace2k_feed_lane_grip lane=%c grip_um=%u");

// The lane's park.  Its next automatic load pulls to the parking point with no tag search, and ends
// loaded.  ace2k_feed_lane_grip with grip_um=0 clears it.  A lane out of range is a bug in the
// host.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_lane_park(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_lane_park: lane out of range");
    irqstatus_t flag = irq_save();
    (void)ace2k_feed_park_set(&ace2k_feed_instance, lane);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_lane_park, "ace2k_feed_lane_park lane=%c");

// The lane's wait.  An insert edge on the lane then starts no automatic load, until
// ace2k_feed_lane_grip with grip_um=0 or an accepted host start on the lane clears it.  A lane out
// of range is a bug in the host.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_lane_wait(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(args[0], "ace2k_feed_lane_wait: lane out of range");
    irqstatus_t flag = irq_save();
    (void)ace2k_feed_wait_set(&ace2k_feed_instance, lane);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_lane_wait, "ace2k_feed_lane_wait lane=%c");

// The settings of the automatic load, sent at connect and on every ACEK_LOAD_SET.  The bounds are
// those of the core: a parking distance that the lane cannot move, or a speed outside its range.  A
// set that the core refuses is a bug in the host, and so is an auto_load other than 0 or 1.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_load_set(uint32_t *args)
{
    ensure_ready();
    bool auto_load = ACE2K_ARG_FLAG(args[2], "ace2k_feed_load_set: auto_load out of range");
    struct ace2k_feed_load_cfg cfg = {
        .park_um = args[0],
        .speed_um_s = args[1],
        .auto_load = auto_load,
    };
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_load_set(&ace2k_feed_instance, &cfg);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_load_set: park or speed out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_load_set, "ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_feed_report, &ace2k_feed_report_due, args[0],
                       ACE2K_FEED_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_feed_cmd_query, "ace2k_feed_query rest_ticks=%u");

// The counters of the feed-forward, at the rate of the feed report, on their own phase.  The host
// sends the same period.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_ff_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_feed_ff_report, &ace2k_feed_ff_report_due, args[0],
                       ACE2K_FEED_FF_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_feed_cmd_ff_query, "ace2k_feed_ff_query rest_ticks=%u");

void ace2k_feed_binding_rfid_bits(uint8_t *loading, uint8_t *exhausted, uint8_t *confirm_out,
                                  uint8_t *confirm_out_epoch)
{
    ensure_ready();
    *loading = 0;
    *exhausted = 0;
    *confirm_out = 0;
    *confirm_out_epoch = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (ace2k_feed_instance.l[lane].confirm_out) {
            *confirm_out |= (uint8_t)(1U << lane);
        }
        if (ace2k_feed_instance.l[lane].confirm_epoch) {
            *confirm_out_epoch |= (uint8_t)(1U << lane);
        }
        if (ace2k_feed_mode(&ace2k_feed_instance, lane) == ACE2K_FEED_LOADING) {
            *loading |= (uint8_t)(1U << lane);
        }
        if (ace2k_feed_search_exhausted(&ace2k_feed_instance, lane)) {
            *exhausted |= (uint8_t)(1U << lane);
        }
    }
}

enum ace2k_feed_refusal ace2k_feed_binding_search_start(uint8_t lane, uint8_t seq)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal r =
        ace2k_feed_search_start(&ace2k_feed_instance, lane, seq, ace2k_tick_now_ms());
    irq_restore(flag);
    return r;
}

enum ace2k_feed_refusal ace2k_feed_binding_tiebreak_start(uint8_t lane)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal r =
        ace2k_feed_tiebreak_start(&ace2k_feed_instance, lane, ace2k_tick_now_ms());
    irq_restore(flag);
    return r;
}

uint8_t ace2k_feed_binding_idle_bits(void)
{
    ensure_ready();
    uint8_t idle = 0;
    irqstatus_t flag = irq_save();
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (ace2k_feed_mode(&ace2k_feed_instance, lane) == ACE2K_FEED_IDLE) {
            idle |= (uint8_t)(1U << lane);
        }
    }
    irq_restore(flag);
    return idle;
}

bool ace2k_feed_binding_search_retry(uint8_t lane, uint8_t seq)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    bool retry = ace2k_feed_search_retry(&ace2k_feed_instance, lane, seq, ace2k_tick_now_ms());
    irq_restore(flag);
    return retry;
}

void ace2k_feed_binding_confirm_cfg_set(uint32_t leg_max_um, uint32_t speed_um_s)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    ace2k_feed_confirm_cfg_set(&ace2k_feed_instance, leg_max_um, speed_um_s);
    irq_restore(flag);
}

int ace2k_feed_binding_search_set(uint32_t search_um)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_search_set(&ace2k_feed_instance, search_um);
    irq_restore(flag);
    return rc;
}
