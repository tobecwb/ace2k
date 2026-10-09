// Binding of the feed module: the sensors' debounced states, the lane instance and the link
// into the core every tick; the events and the periodic state report from a task (sendf never
// runs in timer context); the commands, each core call bracketed with interrupts masked (the
// tick ends moves and pops results in the timer interrupt); the lane LEDs following the mode.
// A refusal is answered (ace2k_feed_start_response), never shut down: a busy lane, an error, a
// missing strand or a lost link are conditions of the unit.  A lane or mode outside the
// dictionary's range is a host bug and takes Klipper's way out.
#include "feed/feed_cmds.h"
#include "lane/lane_cmds.h"    // ace2k_lane_binding, ace2k_lane_binding_link_ok, the reset veto
#include "lane/sensors_cmds.h" // ace2k_sensors_binding
#include "ace2k_board/tick.h"
#include "core/report.h"
#include "core/tx.h"    // ACE2K_SENDF
#include "core/util.h"  // ace2k_put_u16_le, ace2k_put_u32_le
#include "autoconf.h"   // CONFIG_ACE2K_LED, CONFIG_ACE2K_RFID_READ
#include "board/irq.h"  // irq_save, irq_restore
#include "board/misc.h" // timer_from_us
#include "command.h"    // DECL_COMMAND, DECL_CONSTANT, sendf, shutdown
#include "sched.h"      // DECL_INIT, DECL_TASK, DECL_SHUTDOWN, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_LED
#include "lane/led_cmds.h" // ace2k_led_binding_set_lane
#endif
#if CONFIG_ACE2K_RFID_READ
#include "rfid/watch_cmds.h" // ace2k_rfid_binding_feed_bits
#endif

// The phase (report.h): the grid is sensors 0 (10 Hz, so every tenth tick), env 30, lane 50,
// mains 70, feed 80 — a report this task holds for one tick (its frame did not fit) lands on 90,
// the dryer's phase: the two then try on the same tick, the feed's first, and the transmit rule
// holds whichever does not fit (tx.h) — a held frame, never a lost one.
#define ACE2K_FEED_REPORT_PHASE_MS 80U
// The feed-forward's counters (ace2k_feed_ff_state) on their own phase, 40 — free on the grid
// (report.h), so the two frames of this task never fall due on the same tick.
#define ACE2K_FEED_FF_REPORT_PHASE_MS 40U
// Every frame of this task is tried, never dropped (tx.h, report.h): Klipper refuses a frame
// that does not fit its transmit buffer, and this task holds it — the report by its flag, an
// event at the ring's tail — for the next tick, whose wake brings the task back (the tick is the
// task's only waker).  The due report goes first, then the events, oldest first, while they fit.
// An event leaves on the first tick its frame fits: normally the next one, within 10 ms.
#define ACE2K_FEED_EVENT_FMT                                                                       \
    "ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c"

// The host bounds the comparator's windows by it (the thresholds command, below) without a copy.
DECL_CONSTANT("ACE2K_FEED_ASSIST_BURST_UM", ACE2K_FEED_ASSIST_BURST_UM);
// The unload's tail past the slot mouth: the host's unload wait counts it.
DECL_CONSTANT("ACE2K_FEED_UNLOAD_TAIL_UM", ACE2K_FEED_UNLOAD_TAIL_UM);
// The tip snag's defaults: the host reads them, so ACE_SNAG_SET can change one value and send the
// others as they are.
DECL_CONSTANT("ACE2K_FEED_SNAG_FWD_UM", ACE2K_FEED_SNAG_FWD_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_LAG_UM", ACE2K_FEED_SNAG_LAG_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_DUTY_PCT", ACE2K_FEED_SNAG_DUTY_PCT);
DECL_CONSTANT("ACE2K_FEED_SNAG_BACK_UM", ACE2K_FEED_SNAG_BACK_UM);
DECL_CONSTANT("ACE2K_FEED_SNAG_REST_MS", ACE2K_FEED_SNAG_REST_MS);
DECL_CONSTANT("ACE2K_FEED_SNAG_FREE_UM", ACE2K_FEED_SNAG_FREE_UM);
// The follow's defaults and bounds: the host checks ACE_FOLLOW_SET against them before it sends,
// and sends one value changed with the others as they are.
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS", ACE2K_FEED_FOLLOW_FLIP_MS);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS_MIN", ACE2K_FEED_FOLLOW_FLIP_MS_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_FLIP_MS_MAX", ACE2K_FEED_FOLLOW_FLIP_MS_MAX);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM", ACE2K_FEED_FOLLOW_TAKE_UM);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM_MIN", ACE2K_FEED_FOLLOW_TAKE_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAKE_UM_MAX", ACE2K_FEED_FOLLOW_TAKE_UM_MAX);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM", ACE2K_FEED_FOLLOW_TAIL_UM);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM_MIN", ACE2K_FEED_FOLLOW_TAIL_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FOLLOW_TAIL_UM_MAX", ACE2K_FEED_FOLLOW_TAIL_UM_MAX);
// The feed-forward's defaults and bounds: the host checks ACE_FF_SET against them before it sends,
// and sends one value changed with the other as it is.
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM", ACE2K_FEED_FF_CHUNK_UM);
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM_MIN", ACE2K_FEED_FF_CHUNK_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_FF_CHUNK_UM_MAX", ACE2K_FEED_FF_CHUNK_UM_MAX);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S", ACE2K_FEED_FF_PULSE_UM_S);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S_MIN", ACE2K_FEED_FF_PULSE_UM_S_MIN);
DECL_CONSTANT("ACE2K_FEED_FF_PULSE_UM_S_MAX", ACE2K_FEED_FF_PULSE_UM_S_MAX);
// The grip's default and bounds: the host checks its grip against them before it sends.
DECL_CONSTANT("ACE2K_FEED_GRIP_UM", ACE2K_FEED_GRIP_UM);
DECL_CONSTANT("ACE2K_FEED_GRIP_UM_MIN", ACE2K_FEED_GRIP_UM_MIN);
DECL_CONSTANT("ACE2K_FEED_GRIP_UM_MAX", ACE2K_FEED_GRIP_UM_MAX);

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
        enum ace2k_led_lane_pattern p;
        switch (ace2k_feed_mode(&ace2k_feed_instance, lane)) {
        case ACE2K_FEED_ERROR:
            p = ACE2K_LED_LANE_ERROR;
            break;
        case ACE2K_FEED_ASSISTING:
        case ACE2K_FEED_ASSISTING_BACK:
        case ACE2K_FEED_FOLLOWING:
            p = ACE2K_LED_LANE_LOADED; /* the steady state of a lane in use: nothing to look at */
            break;
        case ACE2K_FEED_FEEDING:
        case ACE2K_FEED_ROLLING_BACK:
        case ACE2K_FEED_UNLOADING:
        case ACE2K_FEED_LOADING:
            p = ACE2K_LED_LANE_MOVING;
            break;
        case ACE2K_FEED_IDLE:
        default:
            p = ace2k_feed_sensors->lane[lane].insert ? ACE2K_LED_LANE_LOADED : ACE2K_LED_LANE_OFF;
            break;
        }
        ace2k_led_binding_set_lane(lane, p);
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
    in.link_ok = ace2k_lane_binding_link_ok(); // the lane tick's sample, one per tick
#if CONFIG_ACE2K_RFID_READ
    ace2k_rfid_binding_feed_bits(&in.tag_hold, &in.tag_read, &in.tag_lost);
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
    // the ring's remainder and a report the last pass held (its frame did not fit) leave on
    // this one: the tick is the task's only waker
    if (ace2k_feed_has_events(&ace2k_feed_instance) || ace2k_feed_report_due ||
        ace2k_feed_ff_report_due) {
        sched_wake_task(&ace2k_feed_wake);
    }
}

// True when the report was queued; false when it did not fit and its flag holds it for the next
// tick.  The arrays are packed before the frame is tried: a try needs the encoded frame.
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
        ace2k_put_u32_le(speed + 4 * lane, ace2k_lane_speed_um_s(f->lane, lane));
        duty[lane] = ace2k_lane_duty_pct(f->lane, lane);
        ace2k_put_u16_le(bursts + 2 * lane, ace2k_feed_bursts(f, lane));
        seq[lane] = ace2k_feed_seq(f, lane);
    }
    // seq: one byte per lane, the sequence of the last start the unit accepted from the host on
    // the lane (0 until one), so a host that reconnects continues after it — kept through idle
    // and error; a lane running a host start's move reports that start's seq, a mode of the
    // unit's own leaves it as the last accepted host start's; the host seeds its counter from it
    // before its first start of a session
    return ACE2K_SENDF(
        "ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s",
        (uint8_t)sizeof mode, mode, (uint8_t)sizeof error, error, (uint8_t)sizeof speed, speed,
        (uint8_t)sizeof duty, duty, (uint8_t)sizeof bursts, bursts, (uint8_t)sizeof seq, seq);
}

// The feed-forward's counters since each lane's last entry: three
// arrays of uint16 × 4, little-endian, lane 0 first, and each lane's epoch (uint8 × 4), stepped at
// every entry — the host restarts its tally exactly when it moves.  True when queued; false when
// it did not fit and its flag holds it for the next tick.
static bool report_ff(void)
{
    const struct ace2k_feed *f = &ace2k_feed_instance;
    uint8_t doses[2 * ACE2K_LANE_COUNT];
    uint8_t taut[2 * ACE2K_LANE_COUNT];
    uint8_t full[2 * ACE2K_LANE_COUNT];
    uint8_t epoch[ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        struct ace2k_feed_ff_counts c = ace2k_feed_ff_counts(f, lane);
        ace2k_put_u16_le(doses + 2 * lane, c.doses);
        ace2k_put_u16_le(taut + 2 * lane, c.taut_fixes);
        ace2k_put_u16_le(full + 2 * lane, c.full_fixes);
        epoch[lane] = c.epoch;
    }
    return ACE2K_SENDF("ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s",
                       (uint8_t)sizeof doses, doses, (uint8_t)sizeof taut, taut,
                       (uint8_t)sizeof full, full, (uint8_t)sizeof epoch, epoch);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
void ace2k_feed_task(void)
{
    if (!sched_check_wake(&ace2k_feed_wake)) {
        return;
    }
    // The due report first (a tick of events must not starve it), then the events, oldest first:
    // each is tried from the ring's tail and dropped from it only once queued, so one that does
    // not fit stays, with everything behind it, for the next tick.  Never a self-wake: the tick
    // brings the task back while the ring holds anything or the report is pending.
    if (ace2k_feed_report_due) {
        (void)ace2k_report_try(&ace2k_feed_report_due, report);
    }
    if (ace2k_feed_ff_report_due) {
        (void)ace2k_report_try(&ace2k_feed_ff_report_due, report_ff);
    }
    struct ace2k_feed_event ev;
    while (ace2k_feed_peek_event(&ace2k_feed_instance, &ev)) {
        if (!ACE2K_SENDF(ACE2K_FEED_EVENT_FMT, ev.lane, ev.kind, ev.mode, ev.motor_um,
                         ev.filament_um, ev.seq)) {
            break;
        }
        ace2k_feed_drop_event(&ace2k_feed_instance);
    }
}
DECL_TASK(ace2k_feed_task);

// The lane binding's counters-reset veto: the feed's odometer bases span modes in which the lane
// core is idle between bursts (an assist waiting on the buffer, a load settling after the insert
// edge), so a reset must wait for idle or error here too.  Called
// under the lane binding's mask, after init: a plain read of the instance.
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
    // The lane binding first: its tick slot must precede this one, so a move ends in the lane's
    // tick before the feed's tick of the same period reads the result.
    struct ace2k_lane *lane = ace2k_lane_binding();
    ace2k_feed_sensors = ace2k_sensors_binding();
    ace2k_feed_init(&ace2k_feed_instance, lane);
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
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

// A Klipper shutdown ends every mode and suspends every start:
// the tick outlives a shutdown (ace2k_board/tick.c) and the host's clock queries keep link_ok
// up, so an assist waiting between bursts, a load in its settle or the automatic load would
// otherwise move the motor.  Interrupts are disabled here (Klipper's run_shutdown); no sendf —
// the stopped_shutdown events leave from the task as usual, or not at all once the transport is
// gone.  The order against the lane binding's handler does not matter: one event per lane.
// Klipper calls every DECL_SHUTDOWN target by name from generated code, so it is not static.
void ace2k_feed_binding_shutdown(void)
{
    if (ace2k_feed_ready) {
        ace2k_feed_shutdown(&ace2k_feed_instance, ace2k_tick_now_ms());
    }
}
DECL_SHUTDOWN(ace2k_feed_binding_shutdown);

// True for a lane the command may name: 0..3, or ACE2K_LANE_ALL where the command allows it.
// A predicate, not a shutdown helper: Klipper's shutdown() takes a string literal (the message
// goes into the dictionary at compile time), so each command shuts down with its own.
static bool lane_in_range(uint32_t lane, bool all_allowed)
{
    return lane < ACE2K_LANE_COUNT || (all_allowed && lane == ACE2K_LANE_ALL);
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_start(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], false)) {
        shutdown("ace2k_feed_start: lane out of range");
    }
    if (args[1] > ACE2K_FEED_CMD_ASSIST_BOTH) {
        shutdown("ace2k_feed_start: mode out of range");
    }
    uint8_t lane = (uint8_t)args[0];
    // The seq is the host's to choose: any value 0..255 is accepted (the host never sends 0, which
    // marks a motion the firmware started on its own); the events of the mode echo it.
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal verdict =
        ace2k_feed_start(&ace2k_feed_instance, lane, (enum ace2k_feed_cmd)args[1], args[2], args[3],
                         (uint8_t)args[4], ace2k_tick_now_ms());
    irq_restore(flag);
    // reason: enum ace2k_feed_refusal — no_filament is the answer of a feed, an assist, a load or
    // an unload on a lane whose insert sensor reads absent; a rollback is accepted there
    sendf("ace2k_feed_start_response lane=%c accepted=%c reason=%c", lane,
          verdict == ACE2K_FEED_ACCEPTED ? 1 : 0, (uint8_t)verdict);
}
DECL_COMMAND(ace2k_feed_cmd_start,
             "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_stop(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], true)) {
        shutdown("ace2k_feed_stop: lane out of range");
    }
    irqstatus_t flag = irq_save();
    ace2k_feed_stop(&ace2k_feed_instance, (uint8_t)args[0], ace2k_tick_now_ms());
    irq_restore(flag);
    // the stopped events leave with the next tick, the task's only waker: each on the first
    // tick its frame fits — four of them normally fit one empty buffer beside nothing else
}
DECL_COMMAND(ace2k_feed_cmd_stop, "ace2k_feed_stop lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_clear(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], false)) {
        shutdown("ace2k_feed_clear: lane out of range");
    }
    irqstatus_t flag = irq_save();
    (void)ace2k_feed_clear(&ace2k_feed_instance, (uint8_t)args[0]); // a no-op outside error
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_clear, "ace2k_feed_clear lane=%c");

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_speed_set(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], false)) {
        shutdown("ace2k_feed_speed_set: lane out of range");
    }
    irqstatus_t flag = irq_save();
    int rc =
        ace2k_feed_set_speed(&ace2k_feed_instance, (uint8_t)args[0], args[1], ace2k_tick_now_ms());
    irq_restore(flag);
    if (rc == -ACE2K_EINVAL) {
        shutdown("ace2k_feed_speed_set: speed out of bounds");
    }
    // -ACE2K_EREFUSED (idle or in error) is silent: nothing to change
}
DECL_COMMAND(ace2k_feed_cmd_speed_set, "ace2k_feed_speed_set lane=%c speed_um_s=%u");

// The thresholds from printer.cfg (a config command at connect).  Zero is a host bug: a
// window of nothing would trip on every tick.  So is an allowance at or above the partial's
// window: the strand could never be short by more than it, and the partial would never trip
// (the host refuses the pair before it sends; this is the guard of a host from another commit).
// And so is a standstill or a partial window at or above an assist's burst: every burst takes
// the comparator's bases afresh and chains into the next while the strand is taut, so a strand
// held during a forward assist would never trip either trigger and the motor would grind on,
// burst after burst (the host refuses these too, against ACE2K_FEED_ASSIST_BURST_UM).
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_thresholds_set(uint32_t *args)
{
    ensure_ready();
    struct ace2k_feed_thresholds th = {
        .slip_check_um = args[0],
        .slip_allow_um = args[1],
        .stall_check_um = args[2],
    };
    if (th.slip_check_um == 0 || th.slip_allow_um == 0 || th.stall_check_um == 0) {
        shutdown("ace2k_feed_thresholds_set: a threshold of zero");
    }
    if (th.slip_allow_um >= th.slip_check_um) {
        shutdown("ace2k_feed_thresholds_set: slip_allow_um must be under slip_check_um");
    }
    if (th.stall_check_um >= ACE2K_FEED_ASSIST_BURST_UM ||
        th.slip_check_um >= ACE2K_FEED_ASSIST_BURST_UM) {
        shutdown("ace2k_feed_thresholds_set: stall_check_um and slip_check_um must be under an"
                 " assist's burst (ACE2K_FEED_ASSIST_BURST_UM)");
    }
    irqstatus_t flag = irq_save();
    ace2k_feed_thresholds_set(&ace2k_feed_instance, &th);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_thresholds_set,
             "ace2k_feed_thresholds_set slip_check_um=%u slip_allow_um=%u stall_check_um=%u");

// The tip snag's values: a config command at connect when printer.cfg carries a snag key, and
// ACE_SNAG_SET on the bench.  A value outside its bounds, or a lag at or above the tolerance, is
// a host bug — the host checks the same bounds before it sends.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_snag_set(uint32_t *args)
{
    ensure_ready();
    if (args[2] > ACE2K_FEED_SNAG_DUTY_MAX_PCT) {
        shutdown("ace2k_feed_snag_set: a value out of bounds"); /* before the narrowing cast */
    }
    struct ace2k_feed_snag_cfg cfg = {
        .fwd_um = args[0],
        .lag_um = args[1],
        .duty_pct = (uint8_t)args[2],
        .back_um = args[3],
        .rest_ms = args[4],
        .free_um = args[5],
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

// The follow's values: an init command at every connect (every value, keys or not) and again on
// every ACE_FOLLOW_SET.  A value outside its bounds is a host bug — the host checks the same bounds
// before it sends.
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

// The feed-forward's values: an init command at every connect and again on every ACE_FF_SET.  A
// value outside its bounds is a host bug — the host checks the same bounds before it sends.
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

// The lane's base rate for the feed-forward, sent while it
// follows; clear=1 also drops what the lane owes.  Taken only in the follow — anywhere else it is
// ignored, silently: a base sent as the follow ends is a race, not a bug.  A lane out of range, a
// rate above the lane's ceiling or a clear other than 0 or 1 is a host bug.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_base(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], false)) {
        shutdown("ace2k_feed_base: lane out of range");
    }
    if (args[1] > ACE2K_LANE_SPEED_MAX_UM_S) {
        shutdown("ace2k_feed_base: rate out of bounds");
    }
    if (args[2] > 1U) {
        shutdown("ace2k_feed_base: clear out of bounds");
    }
    irqstatus_t flag = irq_save();
    (void)ace2k_feed_base_set(&ace2k_feed_instance, (uint8_t)args[0], args[1], args[2] != 0U);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_base, "ace2k_feed_base lane=%c rate_um_s=%u clear=%c");

// The lane's grip: its next automatic load pulls grip_um alone and ends
// loaded; 0 clears it.  A lane out of range or a grip outside its bounds is a host bug.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_lane_grip(uint32_t *args)
{
    ensure_ready();
    if (!lane_in_range(args[0], false)) {
        shutdown("ace2k_feed_lane_grip: lane out of range");
    }
    irqstatus_t flag = irq_save();
    int rc = ace2k_feed_grip_set(&ace2k_feed_instance, (uint8_t)args[0], args[1]);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_feed_lane_grip: grip out of bounds");
    }
}
DECL_COMMAND(ace2k_feed_cmd_lane_grip, "ace2k_feed_lane_grip lane=%c grip_um=%u");

// The automatic load's settings: at connect and on every ACE_LOAD_SET.  The bounds are the
// lane's (a parking distance the lane cannot move, or a speed outside its range, is a host bug).
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_load_set(uint32_t *args)
{
    ensure_ready();
    struct ace2k_feed_load_cfg cfg = {
        .park_um = args[0],
        .speed_um_s = args[1],
        .auto_load = args[2] != 0,
    };
    if (cfg.park_um == 0 || cfg.park_um > ACE2K_LANE_MOVE_MAX_UM ||
        !ace2k_lane_speed_in_bounds(cfg.speed_um_s)) {
        shutdown("ace2k_feed_load_set: park or speed out of bounds");
    }
    irqstatus_t flag = irq_save();
    ace2k_feed_load_set(&ace2k_feed_instance, &cfg);
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_load_set, "ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c");

// A query's period in milliseconds: to the nearest millisecond, without the sum that could
// overflow near 2^32 — the host snaps the period to the 100 ms grid the phases need, and a
// truncation on either side would put it a millisecond off, drifting through every phase.  The
// other bindings keep the plain division — their periods, 1 s and 100 ms, are exact multiples of
// the millisecond at 120 MHz.
static uint32_t rest_ms_of(uint32_t rest_ticks)
{
    uint32_t t = timer_from_us(1000U);
    return rest_ticks / t + (rest_ticks % t >= t / 2 ? 1U : 0U);
}

// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = rest_ms_of(args[0]);
    // A report held from before the query would leave on the new grid's terms: cleared under
    // the same mask as the deadline.
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_feed_report, rest_ms, ace2k_tick_now_ms(), ACE2K_FEED_REPORT_PHASE_MS);
    ace2k_feed_report_due = false;
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_query, "ace2k_feed_query rest_ticks=%u");

// The feed-forward's counters at the feed report's rate (the host sends the same period), on
// their own phase.
// cppcheck-suppress constParameterPointer
void ace2k_feed_cmd_ff_query(uint32_t *args)
{
    ensure_ready();
    uint32_t rest_ms = rest_ms_of(args[0]);
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_feed_ff_report, rest_ms, ace2k_tick_now_ms(),
                     ACE2K_FEED_FF_REPORT_PHASE_MS);
    ace2k_feed_ff_report_due = false;
    irq_restore(flag);
}
DECL_COMMAND(ace2k_feed_cmd_ff_query, "ace2k_feed_ff_query rest_ticks=%u");

void ace2k_feed_binding_rfid_bits(uint8_t *loading, uint8_t *exhausted)
{
    ensure_ready();
    *loading = 0;
    *exhausted = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (ace2k_feed_mode(&ace2k_feed_instance, lane) == ACE2K_FEED_LOADING) {
            *loading |= (uint8_t)(1U << lane);
        }
        if (ace2k_feed_search_end(&ace2k_feed_instance, lane) == ACE2K_FEED_SEARCH_LENGTH) {
            *exhausted |= (uint8_t)(1U << lane);
        }
    }
}

enum ace2k_feed_refusal ace2k_feed_binding_search_start(uint8_t lane)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    enum ace2k_feed_refusal r =
        ace2k_feed_search_start(&ace2k_feed_instance, lane, ace2k_tick_now_ms());
    irq_restore(flag);
    return r;
}

void ace2k_feed_binding_search_set(uint32_t search_um)
{
    ensure_ready();
    irqstatus_t flag = irq_save();
    ace2k_feed_search_set(&ace2k_feed_instance, search_um);
    irq_restore(flag);
}
