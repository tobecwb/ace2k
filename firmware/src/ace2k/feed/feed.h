/* What: the motion modes of each lane, with their checks and their events.  An error is a state,
 * never a shutdown.
 * How: the binding owns a struct ace2k_feed, ticks it every 10 ms, and masks interrupts around
 * commands.
 * Depends on: <stdbool.h>, <stdint.h>, lane.h, led.h (the lane patterns), util.h. */
#ifndef ACE2K_FEED_H
#define ACE2K_FEED_H
#include <stdbool.h>
#include <stdint.h>
#include "lane/lane.h"
#include "lane/led.h"
#include "core/util.h"

#define ACE2K_FEED_EVENT_RING 16U
/* the unload budget when the command gives none */
#define ACE2K_FEED_UNLOAD_MAX_UM ACE2K_LANE_MOVE_MAX_UM
/* the window of motor travel of the partial trigger; initial value */
#define ACE2K_FEED_SLIP_CHECK_UM 50000U
/* The filament short by more than this in a window is a slip.  Initial value.  It must stay under
 * ACE2K_FEED_SLIP_CHECK_UM, or the partial never trips, so ace2k_feed_thresholds_set refuses that
 * pair. */
#define ACE2K_FEED_SLIP_ALLOW_UM 15000U
/* Initial value.  Measured on the unit, the trip came after about 20 mm of motor. */
#define ACE2K_FEED_STALL_CHECK_UM 20000U /* the standstill trigger: this much motor since … */
/* … the filament last moved.  "Moved" means this much encoder travel along the command.  That is
 * more than one count and less than two at every accepted scale.  A count is 987..1481 µm across
 * the ±20 % window around 1234.2 µm, and two counts are 1975..2962 µm.  So a filament that stays
 * within one count is still, whatever the lane's wheel. */
#define ACE2K_FEED_STANDSTILL_UM 1500U
/* An assist burst never goes past this length (rule 2).  When a burst reaches it with the filament
 * still taut, the next burst follows at once.  The forward assist keeps the buffer at rest.  A
 * burst starts on taut (pushed), and stops at rest, or on the full reading as a safety (chosen from
 * tests on the unit).  Nothing else starts a burst, neither rest nor a timer.  The length is an
 * initial value.  The cycle was chosen from tests on the unit. */
#define ACE2K_FEED_ASSIST_BURST_UM 100000U
/* The reverse assist's take-up.  Take-ups follow one another while this lane's buffer reads full.
 * A take-up stops at rest or on taut, so the buffer at rest is its target too.  This was chosen
 * from tests on the unit.  Observed on the unit: a creep at rest ran on with nothing holding the
 * filament, and would have unloaded the lane.  The length is an initial value, and it ran in the
 * tests on the unit. */
#define ACE2K_FEED_ASSIST_BACK_FULL_UM 15000U
/* the wait after the insert edge, while the hand is still pushing; initial value; bench */
#define ACE2K_FEED_LOAD_SETTLE_MS 500U
/* A start that repeats the last host start on the lane (see ace2k_feed_start_tip), within this time
 * after that mode ended, is a retry by Klipper.  Klipper resends a query whose response was lost.
 * It is not a new command, so it is accepted and nothing starts.  Initial value, not measured
 * against the retransmit timing of Klipper. */
#define ACE2K_FEED_RETRY_WINDOW_MS 1000U
/* the comparator waits for this much motor into a load; initial value; bench */
#define ACE2K_FEED_LOAD_GRACE_UM 50000U
/* The grip.  While the grip of a lane is set, its next automatic load pulls only this much, without
 * the tag search, and ends loaded.  The drive gear takes a new filament, and the filament waits
 * there while the head still holds the previous piece.  The bounds keep the whole grip strictly
 * inside the load's grace.  The grace ends below ACE2K_FEED_LOAD_GRACE_UM, and the maximum grip
 * stops 5 mm short of it.  So a grip never meets the comparator.  The host's initial value;
 * bench. */
#define ACE2K_FEED_GRIP_UM     40000U
#define ACE2K_FEED_GRIP_UM_MIN 10000U
#define ACE2K_FEED_GRIP_UM_MAX 45000U
_Static_assert(ACE2K_FEED_GRIP_UM_MAX < ACE2K_FEED_LOAD_GRACE_UM,
               "the grip must end inside the grace");
/* The park.  The lane's grip slot holds this value instead of a length.  Then the next automatic
 * load pulls to the parking point and ends loaded.  It reads the tag on the way, as any load does,
 * and runs no tag search past the parking point.  It is for a bay whose search could reach past the
 * outlet, into a tube that the filament of another lane shares.  It is a whole load, so the
 * comparator judges it as any load.  No grip length comes near this value. */
#define ACE2K_FEED_GRIP_PARK UINT32_MAX
/* The wait.  The lane's grip slot holds this value instead of a length.  Then an insert edge on
 * the lane starts no automatic load: the filament stays where the hand left it.  It is for a head
 * that still holds a piece the lane does not own, whose end may still sit at the gear.  It is not
 * used up by an insertion.  It ends as a grip does: an accepted host start, or a grip of 0. */
#define ACE2K_FEED_GRIP_WAIT (UINT32_MAX - 1U)
_Static_assert(ACE2K_FEED_GRIP_WAIT > ACE2K_FEED_GRIP_UM_MAX, "the wait is never a grip length");
/* The path minus a margin.  On the development unit, the drive gear is 323 mm from the outlet,
 * and the filament parks 23 mm inside. */
#define ACE2K_FEED_LOAD_PARK_UM    300000U
#define ACE2K_FEED_LOAD_SPEED_UM_S 30000U /* initial value */
/* A reverse move whose filament stands still, with the lane's plunger at rest, has nothing left to
 * drive.  The end of the filament is past the drive gear, and only the spool can move it.  This was
 * measured on lane 2 of the development unit.  535 mm rewound freely, then the filament stopped
 * with the sensor covered, and the standstill tripped.  The move goes on for this much motor past
 * the standstill, so that the spool winds the end out.  Then it ends unload_incomplete.  Initial
 * value. */
#define ACE2K_FEED_UNLOAD_TAIL_UM 100000U
/* The clear of an unload.  The insert's fall is the goal of an unload, yet the end of the filament
 * could stop right on the sensor of the bay, which then could read it again.  This was seen on the
 * unit in one eject of seven.  So past the fall the unload turns this much more motor, and then
 * ends unloaded (feed.c, the clear).  Nothing in that stretch is a fault, because the goal is
 * reached.  An end already past the drive gear is not moved by it.  0 ends the unload at the fall.
 * ace2k_feed_unload_set() changes it within the bounds compiled beside it.  Initial value; bench.
 */
#define ACE2K_FEED_UNLOAD_CLEAR_UM     20000U
#define ACE2K_FEED_UNLOAD_CLEAR_UM_MIN 0U
/* as long as a tail: the host's wait for a reverse move counts the tail and the clear */
#define ACE2K_FEED_UNLOAD_CLEAR_UM_MAX ACE2K_FEED_UNLOAD_TAIL_UM
/* The load's search for the tag of a spool.  These are its limit (the tag search's rule 3), the
 * default that the reader's binding sets, the reacquire of a tag that a stop left behind, and the
 * least return worth a move.  The default and the reacquire are values measured on the unit. */
#define ACE2K_FEED_SEARCH_MAX_UM 1000000U
/* One revolution of the largest spool that fits (π × a 200 mm flange ≈ 628 mm), plus the longest
 * tag window measured (116 mm), rounded up to 50 mm. */
#define ACE2K_FEED_SEARCH_DEFAULT_UM 750000U
_Static_assert(ACE2K_FEED_SEARCH_DEFAULT_UM <= ACE2K_FEED_SEARCH_MAX_UM,
               "the default search must be one ace2k_feed_search_set() takes");
/* The overshoot of a stop past the tag (1.2 mm, one encoder count), plus the longest tag window
 * (116 mm), rounded up to 10 mm. */
#define ACE2K_FEED_REACQUIRE_UM  120000U
#define ACE2K_FEED_RETURN_MIN_UM 2000U /* under two encoder counts: already there */
/* The tip snag.  The tip of a filament can be thicker than the filament.  It catches the entry of
 * the tube that the plunger carries, and drags the plunger to an end while nothing holds the
 * filament ahead.  This was measured on the unit: stuck after 41.2 mm, with the filament following
 * the motor 1:1.
 * Forward, in a feed started without a free tip, this lane's full is watched instead of failed at
 * once.  The motor may turn fwd_um more while the filament follows within lag_um and the duty stays
 * within its guard.  Then a touch of back_um against the move frees the plunger.  The plunger must
 * read rest within rest_ms, and the move resumes.  The same holds for the search of a read with
 * motion, whose filament may sit in an extruder.  Its stuck verdicts become the obstacle return.
 * With a free tip, fwd_um and lag_um are a window instead.  A free tip is a feed that the host
 * started with free_tip, or a load from the pull of the bay or from its tag search.  The filament
 * lag_um behind the motor within fwd_um of it is the obstacle.  In the search, it is the obstacle
 * return.  There is no touch, no limit on fulls, and one notice per mode.
 * Toward the spool, in a rollback or an unload, a comparator trip with the filament taut is
 * forgiven once the filament has come back free_um in the mode.  A rollback that the host started
 * with free_tip forgives every such trip whose filament came back free_um past the last forgiven
 * one.
 * For the watch and the reverse case, there is one tolerance per mode, except for that rollback.
 * These are the defaults, and ace2k_feed_snag_set() changes them within the bounds. */
/* Chosen from tests on the unit.
 * FWD_UM: the comparator's standstill.  No test contradicted it, and every real catch released on
 * its own within it.
 * LAG_UM: about four encoder counts.  No real catch on a sound tube lagged that much, and a jam is
 * cut sooner.  12 mm was tried, and is a host option. */
#define ACE2K_FEED_SNAG_FWD_UM     20000U
#define ACE2K_FEED_SNAG_FWD_MIN_UM 5000U
#define ACE2K_FEED_SNAG_FWD_MAX_UM 50000U
#define ACE2K_FEED_SNAG_LAG_UM     5000U
#define ACE2K_FEED_SNAG_LAG_MIN_UM 1500U
#define ACE2K_FEED_SNAG_LAG_MAX_UM 20000U
/* One encoder count at the widest scale that the lane takes (+20 % of the default): 1481 µm. */
#define ACE2K_FEED_COUNT_MAX_UM (ACE2K_LANE_SCALE_MAX_X10 / ACE2K_LANE_SCALE_TENTHS_PER_UM)
/* The lag is measured from a base taken at any point of a count.  The filament reads 0 until its
 * next count, while the motor runs on.  So a lag of one count or less trips on a filament that
 * follows the motor exactly.  The least lag is the 1500 µm of the standstill, for the same
 * reason. */
_Static_assert(ACE2K_FEED_SNAG_LAG_MIN_UM > ACE2K_FEED_COUNT_MAX_UM,
               "the least lag must be more than one encoder count at every scale");
/* The duty guard, off.  It was built as a second guard, for a hold that the filament slips through
 * while the encoder still follows.  It did not prove effective in tests on the unit.  The lag
 * decided every jam first.  A fixed excess over the feed-forward has no margin: 9 points or less
 * when healthy, 6 or less on the catch of a tip, and 10–14 for a jam at its stop.
 * It is kept, and settable, for a possible future case.  If it is ever needed, its better form is a
 * peak reading, the rise since the full, which cancels the drag of a spool.  That form is not
 * built. */
#define ACE2K_FEED_SNAG_DUTY_PCT     0U
#define ACE2K_FEED_SNAG_DUTY_MAX_PCT 100U
/* BACK_UM, REST_MS: no test on the unit contradicted them.  The touch was never needed on a sound
 * tube, so they rest on the host tests. */
#define ACE2K_FEED_SNAG_BACK_UM     5000U
#define ACE2K_FEED_SNAG_BACK_MIN_UM 2000U
#define ACE2K_FEED_SNAG_BACK_MAX_UM 20000U
#define ACE2K_FEED_SNAG_REST_MS     500U
#define ACE2K_FEED_SNAG_REST_MIN_MS 100U
#define ACE2K_FEED_SNAG_REST_MAX_MS 2000U
/* Well above the buffer's travel, about 18 mm (docs/hardware.md, the buffer's readings).  It is the
 * most that a filament held from the mode's start can come back.  Measured on the unit: a hold from
 * the start came back 46.3 mm, under this value. */
#define ACE2K_FEED_SNAG_FREE_UM     50000U
#define ACE2K_FEED_SNAG_FREE_MIN_UM 20000U
#define ACE2K_FEED_SNAG_FREE_MAX_UM 200000U
/* The armed stop on a held tip (ace2k_feed_stop_when).  When the plunger is taut and the filament
 * moves less than HELD_UM for HELD_MS, the move ends held.  A taut plunger alone is no stop.  A
 * thick tip dragged through a narrow tube pulls the plunger taut while the path ahead is free, and
 * the filament still comes.  ace2k_feed_held_set() changes both values within the bounds compiled
 * beside them.
 * HELD_UM: a filament that moves this much is not held, and the window starts again.  It is at
 * least two encoder counts at the widest scale, so one count of jitter never restarts it.  Initial
 * value; re-tune on the bench. */
#define ACE2K_FEED_HELD_UM 3000U
/* more than one count at the widest scale, as ACE2K_FEED_STANDSTILL_UM for the standstill */
#define ACE2K_FEED_HELD_UM_MIN 1500U
/* the buffer's travel, about 18 mm, rounded up: a filament that came back that far was not held */
#define ACE2K_FEED_HELD_UM_MAX 20000U
/* HELD_MS: how long the filament must stand with the plunger taut.  A comparator trip that comes
 * sooner, with the plunger taut and the filament still, ends the move held too (feed_stop.c).  So a
 * held tip never ends stuck, at any speed.  Friction must restart the window before HELD_MS.  A
 * filament that comes at speed v moves HELD_UM within HELD_MS when v is above ceil(HELD_UM / count)
 * x count / HELD_MS.  That is the friction floor.  The floor is compared with the speed of the
 * filament, which trails the motor under a spool.  So it keeps a 5 % margin under the least speed
 * of the lane.  At 550 ms it is 8.1 mm/s at the widest scale.  Initial value; re-tune on the
 * bench. */
#define ACE2K_FEED_HELD_MS     550U
#define ACE2K_FEED_HELD_MS_MIN 50U /* five ticks */
#define ACE2K_FEED_HELD_MS_MAX 2000U
_Static_assert(ACE2K_FEED_HELD_UM >= 2U * ACE2K_FEED_COUNT_MAX_UM,
               "the held window must outlast one count of jitter at every scale");
_Static_assert(ACE2K_FEED_HELD_UM_MIN > ACE2K_FEED_COUNT_MAX_UM,
               "the least held_um must be more than one encoder count at every scale");
_Static_assert(ACE2K_FEED_HELD_MS_MAX <= UINT16_MAX, "held_ms travels as a 16-bit value");
/* the upper limit of the friction floor, in per cent of the least speed of the lane, because the
 * filament trails the motor under a spool */
#define ACE2K_FEED_HELD_FLOOR_PCT 95U
_Static_assert((ACE2K_FEED_HELD_UM + ACE2K_FEED_COUNT_MAX_UM - 1U) / ACE2K_FEED_COUNT_MAX_UM *
                       ACE2K_FEED_COUNT_MAX_UM * ACE2K_MS_PER_S / ACE2K_FEED_HELD_MS <
                   ACE2K_LANE_SPEED_MIN_UM_S * ACE2K_FEED_HELD_FLOOR_PCT / ACE2K_PCT_FULL,
               "friction at the lane's least speed, less 5 %, must restart the held window");
/* The follow: the cycles of both assists in one arm.  Initial values; bench.
 * ace2k_feed_follow_set() changes them within the bounds compiled beside them.
 * FLIP_MS: the follow's rule 1, no hunting.  After a move of the follow ends, a move the other way
 * waits for rest, or for this long.
 * TAKE_UM: one take-up of the follow.  The reverse assist keeps its own
 * ACE2K_FEED_ASSIST_BACK_FULL_UM. */
#define ACE2K_FEED_FOLLOW_FLIP_MS     200U
#define ACE2K_FEED_FOLLOW_FLIP_MS_MIN 50U
#define ACE2K_FEED_FOLLOW_FLIP_MS_MAX 2000U
#define ACE2K_FEED_FOLLOW_TAKE_UM     15000U
#define ACE2K_FEED_FOLLOW_TAKE_UM_MIN 5000U
#define ACE2K_FEED_FOLLOW_TAKE_UM_MAX 30000U
/* TAIL_UM: the tail's bound, in motor travel since the insert fell in the follow.  Initial value;
 * bench. */
#define ACE2K_FEED_FOLLOW_TAIL_UM     2000000U
#define ACE2K_FEED_FOLLOW_TAIL_UM_MIN 200000U
#define ACE2K_FEED_FOLLOW_TAIL_UM_MAX 2000000U
/* TAIL_ENC_UM: the filament's signed net travel on the encoder since the follow's tail began.  A
 * move's deadline in the tail is the end leaving the encoder (tail_out), not a timeout, when both
 * of these hold.
 * - The travel is at least this.  The end falls past the insert sensor, then still runs the path
 *   from the sensor to the encoder: on the unit the end left after about 35-37 mm (the strand
 *   travel of +37.3 mm, measured on the unit).  20 mm is about 15 mm under that.
 * - Through the move whose deadline fired, the filament stood still (less than
 *   ACE2K_FEED_STANDSTILL_UM) while the motor turned at least that much.
 * A fault upstream of the end, a slip or a lost encoder, shows as a filament that stops before
 * 20 mm: the timeout.  The end leaving shows as a filament that stops after it.  A filament that
 * still counts through the move is not the end either: the timeout.  Not a setting, so not in the
 * dictionary. */
#define ACE2K_FEED_TAIL_ENC_UM 20000U
/* follow_dir with no direction to hold against: just armed, or rest read since the last move */
#define ACE2K_FEED_FOLLOW_DIR_NONE 0xFFU
/* The feed-forward.  Initial values; bench.  ace2k_feed_ff_set() changes them within the bounds
 * compiled beside them.
 * CHUNK_UM: one dose, and half the debt's cap.
 * PULSE_UM_S: the speed that a forward dose is asked for, and the least speed of a reverse dose.  A
 * forward dose runs at the floor all the same, because it is no longer than the lane's landing. */
#define ACE2K_FEED_FF_CHUNK_UM       3000U
#define ACE2K_FEED_FF_CHUNK_UM_MIN   1000U
#define ACE2K_FEED_FF_CHUNK_UM_MAX   10000U
#define ACE2K_FEED_FF_PULSE_UM_S     15000U
#define ACE2K_FEED_FF_PULSE_UM_S_MIN ACE2K_LANE_SPEED_MIN_UM_S
#define ACE2K_FEED_FF_PULSE_UM_S_MAX ACE2K_LANE_SPEED_MAX_UM_S
/* The debt of the feed-forward is capped at this many chunks either way.  A pull that the doses
 * cannot keep up with is for the burst of the follow to correct, not a debt to grow.  This is the
 * firmware's own value, not published.  The ACE2K_FEED_FF_ names are the ones the host mirrors. */
#define ACE2K_FEED_DEBT_CAP_CHUNKS 2U
/* The base rate accrues for the time since the last tick of the follow, at most this long.  The
 * board's tick runs once for a stretch that it caught up on, with its clock several periods on.  A
 * gap longer than this is a stalled MCU, not filament to owe.  The debt is capped at two chunks
 * anyway.  The limit also keeps |base| × gap within 32 bits: 80 mm/s × 1 s is 8·10⁷ nm. */
#define ACE2K_FEED_LEDGER_GAP_MAX_MS 1000U

enum ace2k_feed_mode {
    ACE2K_FEED_IDLE = 0,
    ACE2K_FEED_FEEDING = 1,
    ACE2K_FEED_ROLLING_BACK = 2,
    ACE2K_FEED_UNLOADING = 3,
    ACE2K_FEED_LOADING = 4,
    ACE2K_FEED_ASSISTING = 5,
    ACE2K_FEED_ASSISTING_BACK = 6,
    /* The lane stays here, with the motor stopped, until ace2k_feed_clear().  It also leaves when
     * the filament leaves the insert sensor.  The error was about the filament, and without a
     * filament there is no error.  So the lane goes idle with a runout.  This is the one exception
     * to rule 8, chosen from tests on the unit. */
    ACE2K_FEED_ERROR = 7,
    /* The follow: forward bursts on taut, and take-ups on this lane's full, each one ended at rest.
     * Once the insert has fallen, its tail. */
    ACE2K_FEED_FOLLOWING = 8,
};

/* The mode argument of ace2k_feed_start on the wire. */
enum ace2k_feed_cmd {
    ACE2K_FEED_CMD_FEED = 0,
    ACE2K_FEED_CMD_ROLLBACK = 1,
    ACE2K_FEED_CMD_ASSIST = 2,
    ACE2K_FEED_CMD_ASSIST_BACK = 3,
    ACE2K_FEED_CMD_UNLOAD = 4,
    ACE2K_FEED_CMD_LOAD = 5,
    ACE2K_FEED_CMD_ASSIST_BOTH = 6, /* the follow */
};

enum ace2k_feed_kind {
    ACE2K_FEED_DONE = 0,
    ACE2K_FEED_STOPPED = 1,
    ACE2K_FEED_STOPPED_LINK = 2,
    ACE2K_FEED_STOPPED_SHUTDOWN = 3,
    ACE2K_FEED_RUNOUT = 4,
    ACE2K_FEED_LOADED = 5,
    ACE2K_FEED_UNLOADED = 6,
    /* The error kinds are 7..13.
     * stuck: the filament does not pass ahead.  Forward, it is a comparator trip with the buffer
     * full.  This now happens only in the follow, the forward assist and the forward return of a
     * load.  Toward the spool, it is the trip with the filament taut, or a full of this lane that
     * the tip snag's tolerance does not forgive, in a rollback or an unload.  The same obstacle in
     * a feed, or in the pull of a load, ends blocked (18), not in an error.
     * tangled: the forward trip without the full, in the same modes. */
    ACE2K_FEED_STUCK = 7,
    ACE2K_FEED_TANGLED = 8,
    ACE2K_FEED_MOTOR_STALLED = 9,
    ACE2K_FEED_TIMEOUT = 10,
    /* Reserved, never produced.  In a forward assist, a filament taut and still under command is
     * the comparator's standstill, answered as tangled.  This was chosen from tests on the unit,
     * and the timed rule was removed since.  The value stays reserved on the wire, because the kind
     * table of the host names it.  So nothing shifts. */
    ACE2K_FEED_ASSIST_STALL = 11,
    /* Not produced any more.  A burst that reaches its bound while taut is followed at once by the
     * next burst.  The head is pulling, and that is no fault.  The value stays reserved on the
     * wire, because the kind table of the host names it.  So nothing shifts. */
    ACE2K_FEED_ASSIST_OVERRUN = 12,
    /* The reverse move could not go on.  The filament is still at the sensor and nothing is left to
     * drive.  Either the end is past the drive and the spool did not wind it out within
     * ACE2K_FEED_UNLOAD_TAIL_UM of motor, or the budget ran out.  It is an error like the others,
     * because a hand is needed. */
    ACE2K_FEED_UNLOAD_INCOMPLETE = 13,
    /* A notice, and the lane goes on.  A burst reached its bound with the filament still taut, so
     * the head uses filament faster than a burst delivers it.  Sent once per taut episode. */
    ACE2K_FEED_BEHIND = 14,
    /* A notice, and the lane goes on.  Either a tip caught the buffer's tube and the tolerance
     * freed it, or the resistance of a free tip was proved.  It is proved when a window passed with
     * the filament following, or when the plunger came back to rest.  Sent once per mode.  The
     * event carries the odometers of the mode where the tip caught. */
    ACE2K_FEED_SNAG = 15,
    /* A notice, and the lane goes on.  The insert fell in the follow.  The end of the filament is
     * on its way to the drive, and the follow feeds on without the insert. */
    ACE2K_FEED_TAIL = 16,
    /* The end of the tail.  The lane is idle, with no error.  It comes from one of these.
     * - A comparator trip in the tail, which means the end has left the drive.
     * - A move's deadline in the tail once the filament has counted ACE2K_FEED_TAIL_ENC_UM
     *   since the tail began and stood still through that move while the motor turned: the end
     *   has left the encoder, so the move cannot reach its length.
     * - The bound of motor of the tail. */
    ACE2K_FEED_TAIL_OUT = 17,
    /* A push toward the head met something that cannot move.  The lane is idle, with no error.  In
     * a feed or in the pull of a load, it comes from one of these.
     * - A comparator trip: the standstill or the partial.
     * - A full of this lane that the tip snag's tolerance does not forgive.
     * - With a free tip, the filament falling lag_um behind the motor in a window while full.
     * The filament waits where it stopped.  A load blocked before its parking point leaves the
     * filament parked there. */
    ACE2K_FEED_BLOCKED = 18,
    /* A move armed with the held stop (ace2k_feed_stop_when).  The plunger stayed taut with the
     * filament still for held_ms, so the tip is held ahead.  The lane is idle, with no error. */
    ACE2K_FEED_HELD = 19,
    /* A move armed with the at-rest stop.  The plunger read rest, from the move's second tick on.
     * The lane is idle, with no error. */
    ACE2K_FEED_AT_REST = 20,
    /* The insert sensors went unknown: the ADC scan stopped, so their reading is frozen.  A
     * running mode cannot see its filament run out, so it stops, as on a lost link.  This applies
     * alike to a running move, an assist waiting between bursts, the follow and its tail, and a
     * load in its settle or its search.  The lane is idle, with no error. */
    ACE2K_FEED_STOPPED_SENSORS = 21,
};

/* The answer to ace2k_feed_start() and ace2k_feed_set_speed().  0 means accepted.  Any other value
 * says why not. */
enum ace2k_feed_refusal {
    ACE2K_FEED_ACCEPTED = 0,
    ACE2K_FEED_REFUSED_BUSY = 1,
    ACE2K_FEED_REFUSED_IN_ERROR = 2,
    ACE2K_FEED_REFUSED_NO_FILAMENT = 3,
    ACE2K_FEED_REFUSED_NO_LINK = 4,
    ACE2K_FEED_REFUSED_BOUNDS = 5,
    /* the read with motion: another lane is busy, other than a follow at rest */
    ACE2K_FEED_REFUSED_OTHER_LANE = 6,
    ACE2K_FEED_REFUSED_NO_MOVE = 7, /* a speed set: the lane is idle, no move to change */
    /* a speed set: the lane is following, and the follow keeps the speed it was armed with */
    ACE2K_FEED_REFUSED_FOLLOWING = 8,
    /* every start and the search: the insert sensors are unknown (the ADC scan stopped) */
    ACE2K_FEED_REFUSED_SENSORS_STALE = 9,
};

enum ace2k_feed_sub {
    ACE2K_FEED_SUB_NONE = 0,
    ACE2K_FEED_SUB_SETTLE = 1, /* load: the 500 ms after the insert edge or the command */
    ACE2K_FEED_SUB_MOVING = 2, /* load: the pull; assist: a burst running */
    /* Assist: no move runs.  The buffer is at rest, or full (forward), or taut (reverse), or it is
     * between two moves.  The switches decide the next move.  Forward, taut and not full starts a
     * burst.  Reverse, this lane's full starts a take-up. */
    ACE2K_FEED_SUB_WAITING = 3,
    ACE2K_FEED_SUB_PAUSED = 4,    /* load: a read session holds the lane */
    ACE2K_FEED_SUB_SEARCH = 5,    /* load: past the parking point, looking for the tag */
    ACE2K_FEED_SUB_RETURN = 6,    /* load: back to the parking point (or the read's start) */
    ACE2K_FEED_SUB_REACQUIRE = 7, /* load: backwards at the floor speed for a lost tag */
};

/* How the search of the last load ended (ace2k_feed_search_end).  The binding of the reader turns a
 * search that ran its length, with no UID ever seen, into no_tag. */
enum ace2k_feed_search_end {
    ACE2K_FEED_SEARCH_NONE =
        0, /* no search: the tag read in the pull, another lane busy, none configured */
    ACE2K_FEED_SEARCH_FOUND = 1,
    ACE2K_FEED_SEARCH_OBSTACLE = 2, /* the lane's own buffer full past the outlet */
    ACE2K_FEED_SEARCH_LENGTH = 3,
    ACE2K_FEED_SEARCH_YIELD = 4, /* another lane was given a start, or its motor ran */
};

/* The tip snag's phase on a lane (feed_snag.c). */
enum ace2k_feed_snag_phase {
    ACE2K_FEED_SNAG_NONE = 0,
    ACE2K_FEED_SNAG_WATCH = 1, /* forward: the full read, the move still running, judged */
    ACE2K_FEED_SNAG_BACK = 2,  /* the touch against the mode's direction is running */
    ACE2K_FEED_SNAG_REST = 3,  /* the touch done, waiting for the plunger at rest */
    /* Forward, with a free tip: a feed that the host started with free_tip, or a load from the pull
     * of the bay or from its search.  The full is judged by the filament, in windows of fwd_um.
     * The move runs on, on its own. */
    ACE2K_FEED_SNAG_FREE_TIP = 4,
};

struct ace2k_feed_snag_cfg {
    uint32_t fwd_um;  /* forward: motor past the full before the touch; a free tip's window */
    uint32_t lag_um;  /* forward: filament behind the motor in the watch (or a window): a jam */
    uint32_t back_um; /* the touch's length */
    uint32_t rest_ms; /* the wait for rest after the touch */
    uint32_t free_um; /* reverse: filament travel back in the mode before a trip is forgiven */
    uint8_t duty_pct; /* forward: the duty above the feed-forward that is a jam; 0 off */
};

struct ace2k_feed_follow_cfg {
    uint32_t flip_ms; /* the follow's rule 1: the other direction after rest or this long */
    uint32_t take_um; /* one take-up */
    uint32_t tail_um; /* the tail's bound: motor since it began */
};

struct ace2k_feed_ff_cfg {
    uint32_t chunk_um;   /* one dose; the debt is capped at two */
    uint32_t pulse_um_s; /* a forward dose's speed, a reverse dose's least */
};

struct ace2k_feed_held_cfg {
    uint32_t held_um; /* a filament that moves this much starts the window again */
    uint32_t held_ms; /* the window: plunger taut and filament still this long is held */
};

/* What a move stops on, besides its own end (ace2k_feed_stop_when). */
struct ace2k_feed_stop_on {
    bool held;    /* the plunger taut and the filament still for the window */
    bool at_rest; /* the plunger at rest */
    /* This move's held window, in ms.  0 means the held.held_ms of the unit.  Each arming carries
     * its own value, so two moves on one unit never share it. */
    uint16_t held_ms;
};

/* The counters of the feed-forward on a lane, since the follow was armed. */
struct ace2k_feed_ff_counts {
    uint16_t doses;      /* doses started */
    uint16_t back_doses; /* reverse doses started: filament taken back on a long retraction */
    uint16_t taut_fixes; /* taut episodes whose burst started: filament short of the head */
    uint16_t full_fixes; /* this lane's full episodes whose take-up started: too much filament */
    uint8_t epoch;       /* steps (wrapping) at every reset of the three: a new count */
};

struct ace2k_feed_inputs {
    uint8_t insert;  /* bit per lane: filament present at the bay entrance */
    uint8_t rest;    /* bit per lane: the plunger at rest */
    uint8_t pushed;  /* bit per lane: the plunger at its pushed end, filament taut */
    bool pulled_any; /* a plunger at its pulled end: a buffer full (the shared input) */
    bool link_ok;
    /* The insert sensors are unknown: the ADC scan stopped, and `insert` is their last, frozen
     * reading.  Nothing is judged on it: ACE2K_FEED_STOPPED_SENSORS and
     * ACE2K_FEED_REFUSED_SENSORS_STALE. */
    bool insert_unknown;
    /* From the reader's binding, one bit per lane (0 in an image without it): a read session holds
     * the lane, its tag is read, its last session lost the tag. */
    uint8_t tag_hold;
    uint8_t tag_read;
    uint8_t tag_lost;
};

/* The comparator's thresholds.  ace2k_feed_thresholds_check has their rules. */
struct ace2k_feed_thresholds {
    uint32_t slip_check_um;  /* the partial's window of motor travel */
    uint32_t slip_allow_um;  /* filament short by more than this in a window: the partial */
    uint32_t stall_check_um; /* the standstill: this much motor since the filament last moved */
};

/* The settings of the automatic load, and the defaults of the load command. */
struct ace2k_feed_load_cfg {
    uint32_t park_um;    /* the pull to the parking point, as filament travel */
    uint32_t speed_um_s; /* the speed of the pull and of the tag search */
    bool auto_load;      /* filament put in at the bay entrance of an idle lane loads on its own */
};

/* One event of a lane.  The two odometers of a final event cover the whole mode, every move in it,
 * whatever ended it.  That includes the bursts of an assist, the follow's doses, the return of a
 * load, the tail of an unload, and a feed resumed after a touch.  The odometers of a notice cover
 * the mode so far.  For the snag notice, that is up to the catch. */
struct ace2k_feed_event {
    uint8_t lane;
    uint8_t kind;        /* enum ace2k_feed_kind */
    uint8_t mode;        /* enum ace2k_feed_mode: where it happened */
    uint8_t seq;         /* seq of the start that entered the mode; 0: the firmware's own */
    uint32_t motor_um;   /* travel of the tach since the mode started */
    int32_t filament_um; /* encoder travel since the mode started, + toward the printer */
};

struct ace2k_feed_lane {
    uint8_t mode;        /* enum ace2k_feed_mode */
    uint8_t error_kind;  /* the kind that put the lane in error */
    uint8_t failed_mode; /* in error: the mode it happened in; the runout that ends it names it */
    uint8_t sub;         /* enum ace2k_feed_sub */
    /* The start's sequence, echoed in every event of this mode: done, stopped, runout, the errors,
     * every kind.  It is 0 with the lane idle, or in a mode that the unit entered on its own (the
     * automatic load).  go_idle() clears it once the mode's events are out, because a finish emits
     * before the mode changes.  So an event of an idle lane never echoes a stale host seq, and the
     * tick's own entries pass 0.  A lane in error keeps the seq of the start that failed until
     * ace2k_feed_clear(). */
    uint8_t seq;
    /* The retry key (see ace2k_feed_start_tip).  It is the last start that the unit accepted from
     * the host on the lane, as the host asked for it: its seq (0 until one), command, length, speed
     * and effective free_tip.  It also holds when the mode that start entered ended.  The effective
     * tip is not the mode's tip, because a load from the bay runs with a free tip that it sets on
     * its own.  The key is kept through idle and error.  A mode that the unit enters on its own
     * leaves all of it as it is, its end included.  last_seq is what the state report publishes
     * (ace2k_feed_seq). */
    uint8_t last_seq;
    uint8_t last_cmd; /* enum ace2k_feed_cmd */
    uint32_t last_length_um;
    uint32_t last_speed_um_s;
    bool last_free_tip;
    uint32_t t_end_ms;
    /* The retry key of the read with motion (see ace2k_feed_search_retry).  search_seq is the
     * host's seq of the read that started the lane's last search, 0 for none.  search_on holds
     * while the mode that read entered runs, and search_end_ms is when it ended.  Any later entry
     * on the lane clears the key. */
    uint8_t search_seq;
    bool search_on;
    uint32_t search_end_ms;
    uint32_t speed_um_s; /* the speed of the command */
    uint32_t length_um;  /* the length or budget of the command */
    /* Nothing holds the front tip of the filament.  So the push judges a full by the filament, not
     * by the plunger (feed_snag.c, the free phase).  For a feed it comes from the host's word.  A
     * load from the bay always has it (ace2k_feed_begin_load).  The read with motion never has it,
     * and keeps the watch.  For a rollback it comes from the host's word too, and then every taut
     * trip past free_um is forgiven, not just one. */
    bool free_tip;
    /* the bases of the odometers: the raw tach and the encoder at the mode's start */
    uint32_t fg_start_pulses;
    int32_t enc_um_start;
    /* the window of the partial trigger: its bases, restarted at every close */
    uint32_t fg_window_pulses;
    int32_t enc_um_window;
    /* The standstill's base: the tach and the encoder when the filament last moved along the
     * command by ACE2K_FEED_STANDSTILL_UM or more.  It is its own base, not the window's base. */
    uint32_t fg_still_pulses;
    int32_t enc_um_still;
    uint32_t t_mode_ms; /* when the mode was entered */
    bool behind_sent;   /* assist: the behind notice for this taut episode is out */
    /* Set by the standstill of a reverse move, or by its deadline, with the plunger at rest.  The
     * end of the filament is past the drive, and the comparator has done its part.  The mode goes
     * on for tail_budget_um of motor from fg_tail_pulses, the raw tach at that first tick.  The
     * budget is ACE2K_FEED_UNLOAD_TAIL_UM for an unload.  For a rollback, it is what the length
     * still owed at that tick (feed_judge.c). */
    bool tail_out;
    uint32_t fg_tail_pulses;
    uint32_t tail_budget_um;
    /* Set by the insert's fall in an unload: the clear.  The unload ends unloaded once the motor
     * has turned unload_clear_um from fg_clear_pulses, the raw tach at the fall.  clear_moved says
     * that the clear's one fresh move has started (feed.c). */
    bool clearing;
    bool clear_moved;
    uint32_t fg_clear_pulses;
    uint16_t bursts;
    bool insert_last;
    /* The load.  resume_sub is the phase that a pause or a reacquire goes back to.  search_end is
     * how the search ended.  searched, yield and reacquired say whether the search began, was asked
     * to yield, and has reacquired once.  return_um is the travel since the load began that the
     * load returns to: the parking point, or 0 for a read with motion. */
    uint8_t resume_sub;
    uint8_t search_end;
    bool searched;
    bool yield;
    bool reacquired;
    bool gripped; /* the automatic load a grip or the park took: no tag search past its pull */
    /* A load from the bay: the comparator skips its first ACE2K_FEED_LOAD_GRACE_UM of motor.  A
     * read with motion never has it, because its filament is gripped from its first millimetre.  It
     * is cleared at every entry and end (reset_mode_state), and set by ace2k_feed_begin_load after
     * its entry. */
    bool grace;
    uint32_t return_um;
    /* The tip snag (feed_snag.c).
     * - snag: its phase.
     * - snag_used: the mode's one tolerance is spent.  In the free phase, its first full is
     *   recorded.  In a rollback with a free tip, a catch was forgiven, which does not stop the
     *   next.
     * - snag_sent: its notice is out.
     * - fg_snag_pulses, enc_um_snag: the counts at its start.  They are the watch's base, and the
     *   window of the free phase.  In that rollback, they are those of the last forgiven catch, the
     *   base that the next one must come back free_um past.
     * - t_snag_ms: when the wait for rest began.
     * - snag_motor_um, snag_filament_um: the mode's odometers at its start, which the notice
     *   carries. */
    uint8_t snag; /* enum ace2k_feed_snag_phase */
    bool snag_used;
    bool snag_sent; /* the snag notice of the mode is out: once a mode */
    uint32_t fg_snag_pulses;
    int32_t enc_um_snag;
    uint32_t t_snag_ms;
    uint32_t snag_motor_um;
    int32_t snag_filament_um;
    /* The follow.  follow_dir is the direction of the running move, or of the last one (enum
     * ace2k_lane_dir), or ACE2K_FEED_FOLLOW_DIR_NONE once rest was read.  follow_end_ms is when
     * that move ended, the clock of the follow's rule 1. */
    uint8_t follow_dir;
    uint32_t follow_end_ms;
    /* The feed-forward, in FOLLOWING only.  Every entry and every end clears all of it but the
     * counters, which the next entry clears, as with bursts. */
    int32_t base_um_s;    /* the rate of the host: + toward the head, − to the spool, 0 none */
    int32_t debt_um;      /* filament owed: + to the head, − back to the spool; capped at
                          * 2 × chunk either way */
    uint32_t debt_rem_nm; /* the sub-µm remainder of |base| × elapsed, carried */
    uint32_t ledger_ms;   /* the last accrual (or the entry): the base accrues from here */
    bool dose;            /* the running move is a dose, not a burst nor a take-up */
    int32_t dose_enc_um;  /* the encoder at the start of the dose */
    bool dose_against;    /* the dose was stopped once a chunk was owed the other way */
    bool taut_seen;       /* this taut episode already counted */
    bool full_seen;       /* this full episode already counted */
    uint16_t doses, back_doses, taut_fixes, full_fixes; /* since the entry */
    uint8_t ff_epoch; /* steps (wrapping) at every entry, which resets the three */
    /* The follow's tail.  The insert fell in FOLLOWING.  The follow feeds on without it, and takes
     * up nothing.  tail_fg_start_pulses is the raw tach when the tail began, the base of its bound.
     * This is not the tail_out of the reverse moves above.  It is cleared at every entry and every
     * end.  When the insert rises again it stays set, because the tail ignores the bay. */
    bool tail;
    uint32_t tail_fg_start_pulses;
    int32_t tail_enc_start_counts; /* the raw encoder when the tail began (a deadline's ruling) */
    /* The armed stop (feed_stop.c).
     * - stop_next: armed by the host for its next start on the lane.  It is kept through idle until
     *   that start takes it, whatever the start's answer.
     * - stop_on: the stop of the running mode, cleared at every entry and every end.
     * - stop_judged: the stop has judged one tick of the mode.  At rest is not judged on the first
     *   tick.
     * - The held window is open while the plunger reads taut, from held_t0_ms, with the encoder at
     *   held_enc0_um. */
    struct ace2k_feed_stop_on stop_next;
    struct ace2k_feed_stop_on stop_on;
    bool stop_judged;
    bool held_open;
    uint32_t held_t0_ms;
    int32_t held_enc0_um;
};

struct ace2k_feed {
    struct ace2k_lane *lane;
    struct ace2k_feed_lane l[ACE2K_LANE_COUNT];
    struct ace2k_feed_thresholds th;
    struct ace2k_feed_load_cfg load;
    struct ace2k_feed_snag_cfg snag;
    struct ace2k_feed_follow_cfg follow;
    struct ace2k_feed_ff_cfg ff;
    struct ace2k_feed_held_cfg held;
    uint32_t unload_clear_um; /* the clear of an unload past the insert's fall; 0: none */
    uint32_t search_um;       /* reach of the search as travel since the load began; 0: no search */
    /* per lane, the grip that the next automatic load takes and clears; 0: none.  RAM only */
    uint32_t grip_um[ACE2K_LANE_COUNT];
    /* The last tick's inputs, whole.  The refusals of ace2k_feed_start() read the link and the
     * insert.  The first move of the assists is the tick's own step, run on these inputs.  So
     * arming and the tick use one reading of the buffer. */
    struct ace2k_feed_inputs last;
    bool primed; /* the first tick has run: insert edges are judged from the next one */
    /* The last tick had the insert unknown.  The first tick with it known primes the insert again,
     * as the first tick does: an edge is judged only between two known readings. */
    bool insert_was_unknown;
    /* A Klipper shutdown was seen (ace2k_feed_shutdown).  The tick reads the link as down, and
     * every start is refused as if it were, until ace2k_feed_init, which is the reset of the
     * MCU. */
    bool shut_down;
    struct ace2k_feed_event ring[ACE2K_FEED_EVENT_RING];
    uint8_t head;     /* slot of the next event; producers write it (see ace2k_feed_peek_event) */
    uint8_t tail;     /* the oldest event; the consumer writes it */
    uint32_t dropped; /* events lost to a full ring */
};

/* Sets every lane idle, with the default thresholds and load settings.  link_ok reads false until
 * the first tick. */
void ace2k_feed_init(struct ace2k_feed *self, struct ace2k_lane *lane);

/* Runs one 10 ms tick, after the lane's tick.  Interrupt context. */
void ace2k_feed_tick(struct ace2k_feed *self, uint32_t now_ms, const struct ace2k_feed_inputs *in);

/* Starts a motion command.
 * - length_um: the length for a feed or a rollback.  The budget for an unload, where 0 means the
 *   default.  The parking distance for a load, where 0 means the configured one.  Ignored for an
 *   assist.
 * - speed_um_s: within the lane's bounds, for the assist too.  There is no configured assist speed,
 *   because the host always sends one.  For a load, 0 means the configured speed.
 * - seq: the host's sequence for this start, echoed in every event of the mode that it enters.  Any
 *   value is taken.  The host never sends 0, because 0 is reserved for a motion that the firmware
 *   starts on its own.
 * It returns ACE2K_FEED_ACCEPTED, or the refusal.  A feed or an unload with no filament at the bay
 * entrance is ACE2K_FEED_REFUSED_NO_FILAMENT.  A rollback on an empty lane is accepted, because it
 * brings a piece parked past the sensor back out.  A start equal to the last start that the unit
 * accepted from the host on the lane is a retry of the transport, not a new command.  See
 * ace2k_feed_start_tip for the rule.  The caller masks interrupts. */
enum ace2k_feed_refusal ace2k_feed_start(struct ace2k_feed *self, uint8_t lane,
                                         enum ace2k_feed_cmd cmd, uint32_t length_um,
                                         uint32_t speed_um_s, uint8_t seq, uint32_t now_ms);

/* ace2k_feed_start(), with the state of the tip.  free_tip is true when nothing holds the front tip
 * of the filament.  It is stored for a feed, whose full is then judged by the filament
 * (feed_snag.c).  It is stored for a rollback, which then forgives every taut trip past free_um
 * (feed_snag.c).  Every other command ignores it.  A load sets its own, because the filament from
 * the bay is always free.  For an unload, the taut trip stays its one tolerance.
 *
 * The retry rule, stated here once.  The effective tip is the flag for a feed or a rollback, and
 * false for every other command.  So another command resent with another flag is still the retry.
 * A start is the transport's retry when it equals the last start that the unit accepted from the
 * host on the lane.  It must be equal in seq (non-zero), command, length, speed and effective tip.
 * It must also come while the mode that the first start entered still runs, or within
 * ACE2K_FEED_RETRY_WINDOW_MS after it ended.  The retry is accepted and nothing starts.  The event
 * that went out, or goes out, serves the completion to the host.  A mode that the unit runs on its
 * own on the lane, running or ending, neither closes nor opens the window again.  The same seq with
 * another command, length, speed, or, for a feed or a rollback, free_tip, is a new command.  It
 * takes the normal path: busy, or a new move.
 *
 * ace2k_feed_start() is this call with free_tip false. */
enum ace2k_feed_refusal ace2k_feed_start_tip(struct ace2k_feed *self, uint8_t lane,
                                             enum ace2k_feed_cmd cmd, uint32_t length_um,
                                             uint32_t speed_um_s, uint8_t seq, bool free_tip,
                                             uint32_t now_ms);

/* Handles a Klipper shutdown (rule 6).  Every lane is stopped, and its pending lane result is
 * discarded.  The lane's shutdown handler may have left one.  Whichever handler runs first, a lane
 * gets one stopped_shutdown.  A lane in a mode goes idle with that event.  A lane in error stays in
 * error (rule 8).
 * Then the latch holds.  The tick reads link_ok as false for every decision, so there is no burst,
 * no load step, no automatic load and no tail move.  ace2k_feed_start() answers
 * ACE2K_FEED_REFUSED_NO_LINK.  This lasts until ace2k_feed_init().  The tick outlives a shutdown,
 * because it is the board's tick.  The host also keeps sending bytes after a shutdown.  So the link
 * alone would not keep an armed mode from moving.  The caller disables interrupts. */
void ace2k_feed_shutdown(struct ace2k_feed *self, uint32_t now_ms);

/* Stops the motor, whatever the mode (rule 5).  A moving or assisting lane goes idle with a
 * `stopped` event.  A load still in its settle is cancelled, with nothing ever moved (rule 9).  A
 * lane in error stays there.  ACE2K_LANE_ALL stops every lane. */
void ace2k_feed_stop(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* Moves the lane from error to idle.  It returns true when the lane was in error.  A clear ends no
 * mode, so the retry window stays the one of the failure.  The other way out of error is the
 * filament leaving the sensor, which is the tick's runout on a lane in error.  That way keeps the
 * failure's window too.  A clear is for an error whose filament stays. */
bool ace2k_feed_clear(struct ace2k_feed *self, uint8_t lane);

/* Sets a new speed for the lane.  It is judged in this order.
 * - A lane above 3, or a speed outside [ACE2K_LANE_SPEED_MIN_UM_S, ACE2K_LANE_SPEED_MAX_UM_S]:
 *   ACE2K_FEED_REFUSED_BOUNDS, whatever the lane's mode.
 * - A lane in error: ACE2K_FEED_REFUSED_IN_ERROR.
 * - An idle lane: ACE2K_FEED_REFUSED_NO_MOVE.
 * - A following lane, whether a burst, a take-up, a dose or the tail runs or not:
 *   ACE2K_FEED_REFUSED_FOLLOWING.  The follow keeps the speed it was armed with.
 * - Otherwise ACE2K_FEED_ACCEPTED, for a feed, a rollback, an unload, a load or a one-way assist.
 *   The command's running move takes the speed now, and its deadline follows from now_ms.  A move
 *   that the mode makes for itself, such as the reacquire of a load or the touch of a snag, keeps
 *   its own speed.  The mode keeps the new speed for its next command move.  Examples are an assist
 *   between bursts, a load settling, and a feed resumed after a touch.
 * A refused speed changes nothing, and is kept for no later move. */
enum ace2k_feed_refusal ace2k_feed_set_speed(struct ace2k_feed *self, uint8_t lane,
                                             uint32_t speed_um_s, uint32_t now_ms);

/* Why a set of thresholds is refused (ace2k_feed_thresholds_check).  It is the first rule broken,
 * in the order they are judged. */
enum ace2k_feed_thresholds_fault {
    ACE2K_FEED_THRESHOLDS_OK = 0,
    ACE2K_FEED_THRESHOLDS_ZERO = 1, /* a window of nothing would trip on every tick */
    /* slip_allow_um at or above slip_check_um.  The filament could never be short by more than the
     * allowance, so the partial would never trip. */
    ACE2K_FEED_THRESHOLDS_ALLOW_NOT_UNDER_CHECK = 2,
    /* stall_check_um or slip_check_um at or above ACE2K_FEED_ASSIST_BURST_UM.  Every burst takes
     * the bases of the comparator afresh.  So such a window never closes in an assist, and a
     * filament held during one would be ground on, burst after burst. */
    ACE2K_FEED_THRESHOLDS_NOT_UNDER_BURST = 3,
};

/* The rules that a set of thresholds of the comparator must keep: no zero, slip_allow_um under
 * slip_check_um, and both windows under ACE2K_FEED_ASSIST_BURST_UM.  It returns the first rule
 * broken, or ACE2K_FEED_THRESHOLDS_OK.  It keeps no state.  The binding names the rule in its
 * answer to a bug in the host. */
enum ace2k_feed_thresholds_fault
ace2k_feed_thresholds_check(const struct ace2k_feed_thresholds *th);

/* Sets the thresholds of the supervisors, in effect from the next tick.  It returns 0, or
 * -ACE2K_EINVAL for a set that ace2k_feed_thresholds_check() refuses, and then nothing is taken.
 * The host checks the same rules before it sends.  The caller masks interrupts. */
int ace2k_feed_thresholds_set(struct ace2k_feed *self, const struct ace2k_feed_thresholds *th);

/* Sets the automatic load: the parking distance, the pull's speed, and whether filament put in at
 * the bay entrance loads on its own.  It returns 0, or -ACE2K_EINVAL for a park of 0 or above
 * ACE2K_LANE_MOVE_MAX_UM, or a speed outside the lane's bounds.  Then nothing is taken.  The caller
 * masks interrupts. */
int ace2k_feed_load_set(struct ace2k_feed *self, const struct ace2k_feed_load_cfg *cfg);
/* Sets the tip snag's values, in effect from the next read.  Each value is read where it is used,
 * so a change under a running tolerance mixes two sets, each value within its bounds.  It returns
 * 0, or -ACE2K_EINVAL for any value outside its bounds (ACE2K_FEED_SNAG_*_MIN/_MAX) or lag_um ≥
 * fwd_um, and then nothing is taken.  A fwd_um lowered under a running watch may start the touch at
 * once.  That is harmless, because the tolerance it shortens is the one that the host asked for.
 * The caller masks interrupts. */
int ace2k_feed_snag_set(struct ace2k_feed *self, const struct ace2k_feed_snag_cfg *cfg);

/* Sets the follow's values, read where they are used: the next move, the next flip.  It returns 0,
 * or -ACE2K_EINVAL for a value outside its bounds (ACE2K_FEED_FOLLOW_*_MIN / _MAX), and then
 * nothing is taken.  The caller masks interrupts. */
int ace2k_feed_follow_set(struct ace2k_feed *self, const struct ace2k_feed_follow_cfg *cfg);

/* Sets the values of the feed-forward, read where they are used: the next dose, the cap of the next
 * accrual.  It returns 0, or -ACE2K_EINVAL for a value outside its bounds
 * (ACE2K_FEED_FF_*_MIN / _MAX), and then nothing is taken.  The caller masks interrupts. */
int ace2k_feed_ff_set(struct ace2k_feed *self, const struct ace2k_feed_ff_cfg *cfg);

/* Arms a stop for the next ace2k_feed_start() on the lane.
 * - held: while a move toward the spool runs, the plunger taut with the filament moving less than
 *   held.held_um for held_ms.  held_ms 0 means held.held_ms, the unit's setting.  The window
 *   belongs to this move, and is cleared with the arming.  A comparator trip with the plunger taut
 *   and the filament still also counts.  A reverse assist runs no move while the plunger reads
 *   taut, so held never ends it.  A start takes held only for a rollback and an unload.
 * - at_rest: in either direction, the plunger at rest, judged from the move's second tick on.  A
 *   move started at rest runs one tick, then ends.  A start takes it for any mode but the follow.
 * A start that does not fit the arming is refused ACE2K_FEED_REFUSED_BOUNDS, after the refusals
 * that come before every command and before the grip or the move, and drops the arming.
 * Either one ends the move with ACE2K_FEED_HELD or ACE2K_FEED_AT_REST, with the lane idle and no
 * error.  The stop is judged every tick, before the autonomous steps, the tip snag (the reverse
 * tolerance included), the tails and the comparator.  The next start takes the arming whatever its
 * answer: accepted, a retry or refused.  The mode that it enters keeps the arming until it ends.  A
 * mode that the unit enters on its own, the automatic load, neither takes it nor uses it.  A tick
 * with the link down clears an arming that no start took.  Both false disarms.
 * It returns 0.  It returns -ACE2K_EINVAL, with nothing armed, for a lane out of range, or for a
 * held_ms other than 0 outside ACE2K_FEED_HELD_MS_MIN.._MAX, whatever the lane's mode.  It returns
 * -ACE2K_EBUSY, with nothing armed, for a lane that is not idle, in a mode or in error.
 * The caller masks interrupts. */
int ace2k_feed_stop_when(struct ace2k_feed *self, uint8_t lane, bool held, bool at_rest,
                         uint32_t held_ms);

/* Sets the values of the held stop, read where they are used: the next tick of a window.  It
 * returns 0, or -ACE2K_EINVAL for a value outside its bounds (ACE2K_FEED_HELD_*_MIN / _MAX), and
 * then nothing is taken.  The caller masks interrupts. */
int ace2k_feed_held_set(struct ace2k_feed *self, uint32_t held_um, uint32_t held_ms);

/* Sets the clear of an unload, read at the next insert's fall.  It returns 0, or -ACE2K_EINVAL for
 * a value outside ACE2K_FEED_UNLOAD_CLEAR_UM_MIN.._MAX, and then nothing is taken.  The caller
 * masks interrupts. */
int ace2k_feed_unload_set(struct ace2k_feed *self, uint32_t clear_um);

/* Sets the lane's base rate: the filament that the head is about to use.  While the lane follows,
 * it is metered out in doses of ff.chunk_um at ff.pulse_um_s.  Each dose is no longer than the
 * lane's landing, so it runs at the floor speed.  A negative rate is a long retraction.  The
 * filament owed back is metered out in reverse doses of ff.chunk_um at |rate|, never under
 * ff.pulse_um_s, with no landing.  0 stops only the accrual.  What is owed either way is still paid
 * in doses.  With clear, what is owed is dropped too.  The host uses clear for a lane remapped to
 * another extruder, or for one of two lanes that follow one extruder.
 * It returns 0 when the rate is taken.  It returns -ACE2K_EINVAL for a lane out of range, or for a
 * rate whose magnitude is above ACE2K_LANE_SPEED_MAX_UM_S.  It returns -ACE2K_EREFUSED, with
 * nothing taken, when the lane is not in FOLLOWING.  The follow's arm, and every end, set the rate
 * to 0.  RAM only.  The caller masks interrupts. */
int ace2k_feed_base_set(struct ace2k_feed *self, uint8_t lane, int32_t rate_um_s, bool clear);

/* Sets the search's reach, as travel since the load began.  0 turns the search off.  It returns 0,
 * or -ACE2K_EINVAL above ACE2K_FEED_SEARCH_MAX_UM, and then nothing is taken.  Past that limit the
 * search would run the filament beyond the outlet.  The reader's binding sets it.  The caller masks
 * interrupts. */
int ace2k_feed_search_set(struct ace2k_feed *self, uint32_t search_um);

/* Sets the lane's grip.  Its next automatic load pulls grip_um alone, at the load's speed, with no
 * tag search, and ends loaded.  That load clears the grip.  So does any start that the host gives
 * the lane and that is accepted: ace2k_feed_start, its retry included, and ace2k_feed_search_start.
 * A refused start leaves it.  0 clears it.  It returns 0, or -ACE2K_EINVAL for a lane out of range
 * or a grip outside ACE2K_FEED_GRIP_UM_MIN..ACE2K_FEED_GRIP_UM_MAX, and then nothing is taken.  RAM
 * only.  The caller masks interrupts. */
int ace2k_feed_grip_set(struct ace2k_feed *self, uint8_t lane, uint32_t grip_um);

/* Sets the lane's park, in its grip slot (ACE2K_FEED_GRIP_PARK).  Its next automatic load pulls to
 * the parking point with no tag search, and ends loaded.  The park is used up and cleared as a grip
 * is, and a grip of 0 clears it too.  It returns 0, or -ACE2K_EINVAL for a lane out of range, and
 * then nothing is taken.  RAM only.  The caller masks interrupts. */
int ace2k_feed_park_set(struct ace2k_feed *self, uint8_t lane);

/* Sets the lane's wait, in its grip slot (ACE2K_FEED_GRIP_WAIT).  An insert edge on the lane then
 * starts no automatic load, however many times the filament goes in.  A grip of 0 clears it, and
 * so does any accepted host start on the lane.  It returns 0, or -ACE2K_EINVAL for a lane out of
 * range, and then nothing is taken.  RAM only.  The caller masks interrupts. */
int ace2k_feed_wait_set(struct ace2k_feed *self, uint8_t lane);

/* Starts the read on command with motion.  The lane enters its load at the search.  It moves
 * forward up to the search's reach from where the filament is, and back to there.  Its filament may
 * sit in an extruder.  So it has no free tip, and a full takes the tip snag's watch.  It returns
 * the refusals of a start: no_link, in_error, busy, no_filament.  Then it returns
 * ACE2K_FEED_REFUSED_OTHER_LANE while another lane is busy (the tag search's rule 4).  A lane in
 * the follow with its motor at rest does not count, and the search yields when its motor starts.
 * Then it returns ACE2K_FEED_REFUSED_BOUNDS with no search configured.  An accepted read records
 * seq as the lane's retry key (ace2k_feed_search_retry); 0 records none.  The mode's events still
 * carry seq 0, the unit's own load.  Interrupts masked. */
enum ace2k_feed_refusal ace2k_feed_search_start(struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                                                uint32_t now_ms);

/* Tells whether a read with motion on the lane with this seq is the transport's retry of the last
 * one the unit accepted.  It is when seq is not 0, is that read's, and the search that read started
 * still runs, or ended at most ACE2K_FEED_RETRY_WINDOW_MS ago with no mode entered on the lane
 * since.  The caller then answers accepted and starts nothing.  False for a lane out of range. */
bool ace2k_feed_search_retry(const struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                             uint32_t now_ms);

/* Returns how the last load search of the lane ended (enum ace2k_feed_search_end).  It is kept
 * until the next load.  It returns ACE2K_FEED_SEARCH_NONE for a lane out of range. */
uint8_t ace2k_feed_search_end(const struct ace2k_feed *self, uint8_t lane);

/* Copies the oldest event and leaves it in the ring.  The binding tries to send it, and pops it
 * only once the frame is queued.  So a frame that does not fit waits at the ring's tail for the
 * next tick.  The producers write the head.  They are the tick, and the calls that end a mode from
 * task context: ace2k_feed_stop, ace2k_feed_shutdown, and ace2k_feed_search_start with nothing left
 * to search.  Those calls run with interrupts masked, so no two producers interleave.  None of them
 * writes the slot at the tail, because one slot stays free.  So the peek and the pop of the
 * consumer need no mask.  It returns false when the ring is empty. */
bool ace2k_feed_peek_event(const struct ace2k_feed *self, struct ace2k_feed_event *out);
/* Releases the oldest event without a copy.  This is the binding's step once the peeked frame is
 * queued.  It does nothing when the ring is empty. */
void ace2k_feed_drop_event(struct ace2k_feed *self);
/* Copies and releases the oldest event, as a peek and a drop.  It returns false when the ring is
 * empty. */
bool ace2k_feed_pop_event(struct ace2k_feed *self, struct ace2k_feed_event *out);
/* Returns true while the ring holds an event.  The task of the binding wakes for it. */
bool ace2k_feed_has_events(const struct ace2k_feed *self);

/* Returns true in every mode but idle and error.  For ACE2K_LANE_ALL, it returns true when any lane
 * is busy.  The lane binding's counters-reset veto reads it, because the bases of the odometers
 * live across those modes.  The dryer's rotation lockout will read it too.  A lane out of range is
 * not busy. */
bool ace2k_feed_lane_busy(const struct ace2k_feed *self, uint8_t lane);
/* Returns the lane's mode (enum ace2k_feed_mode), or idle for a lane out of range. */
uint8_t ace2k_feed_mode(const struct ace2k_feed *self, uint8_t lane);
/* Returns the pattern that the LED of a lane shows for its mode (enum ace2k_feed_mode).  An error
 * shows as an error, and a motion that the firmware drives shows as moving.  An assist or a follow
 * is the steady state of a lane in use, so it shows as loaded.  An idle lane shows loaded with
 * filament inserted, and off otherwise.  Any other mode value is read as idle. */
enum ace2k_led_lane_pattern ace2k_feed_led_pattern(uint8_t mode, bool insert);
/* Returns the kind that put the lane in error (enum ace2k_feed_kind).  It returns 0 for a lane not
 * in error, or out of range. */
uint8_t ace2k_feed_error(const struct ace2k_feed *self, uint8_t lane);
/* Returns the bursts and take-ups that the lane's assists started since its last entry into a mode.
 * The count is kept after the mode ends, and the state report publishes it.  It returns 0 for a
 * lane out of range. */
uint16_t ace2k_feed_bursts(const struct ace2k_feed *self, uint8_t lane);
/* Returns the counters of the feed-forward since the last entry of the lane.  They are kept after
 * the mode ends, as bursts are.  It returns zeros for a lane out of range. */
struct ace2k_feed_ff_counts ace2k_feed_ff_counts(const struct ace2k_feed *self, uint8_t lane);
/* Returns the sequence of the last start that the unit accepted from the host on the lane, 0 until
 * one.  So a host that reconnects continues after it.  The state report publishes it, and it is
 * kept through idle and error.  A lane that runs the move of a host start reports the seq of that
 * start.  A mode of the unit itself leaves it as the seq of the last accepted host start.  It
 * returns 0 for a lane out of range. */
uint8_t ace2k_feed_seq(const struct ace2k_feed *self, uint8_t lane);

#endif
