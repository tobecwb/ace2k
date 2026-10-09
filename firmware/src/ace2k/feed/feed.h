/* What: the per-lane motion state machine of the unit — feed, rollback, unload, load, assist in
 * both directions, the error state — with the per-move odometers, the supervisors (motor ×
 * filament, the motor's own outcomes, the assist's behind notice, the runout, the link, a
 * Klipper shutdown), the events the host sees and the refusals it gets.
 * How: the binding owns one struct ace2k_feed over the lane instance (core to core: feed calls
 * ace2k_lane_move / _stop / _set_speed / _pop_result); ace2k_feed_tick() every 10 ms, after the
 * lane's tick, with the sensors' debounced states and link_ok; the commands from task context
 * with interrupts masked; the task peeks, tries the frame, and drops the event once it is
 * queued.  Every motor run is a bounded lane move; every error
 * is a state plus an event, never a shutdown (rule 7); ace2k_feed_clear() leaves the error state
 * (rule 8), and so does the strand leaving the sensor — the one exception, decided at the bench.  A Klipper shutdown ends every mode through ace2k_feed_shutdown() and
 * latches: from then on nothing starts until the MCU is reset (rule 6).  The ring is
 * single-producer / single-consumer as the sensors' ring.  feed.c keeps the modes, the tick and
 * the events; the start command lives in feed_start.c, the comparator in feed_compare.c, the
 * verdicts read from the buffer and the counters in feed_judge.c, the assists in feed_assist.c,
 * the load and its tag search in feed_load.c, the tip snag's tolerance in feed_snag.c;
 * feed_internal.h is the seven files' private seam.
 * The follow is the third assist, both cycles in one arm, in
 * feed_assist.c too, with the feed-forward: a host's base rate
 * metered out in bounded doses ahead of the pull while the lane follows.
 * Depends on: <stdbool.h>, <stdint.h>, lane.h, util.h. */
#ifndef ACE2K_FEED_H
#define ACE2K_FEED_H
#include <stdbool.h>
#include <stdint.h>
#include "lane/lane.h"
#include "core/util.h"

#define ACE2K_FEED_EVENT_RING 16U
/* the unload budget when the command gives none */
#define ACE2K_FEED_UNLOAD_MAX_UM ACE2K_LANE_MOVE_MAX_UM
/* the partial trigger's window of motor travel; initial value */
#define ACE2K_FEED_SLIP_CHECK_UM 50000U
/* the filament short by more than this in a window: slip; initial value — under
 * ACE2K_FEED_SLIP_CHECK_UM, or the partial never trips (the binding refuses the pair) */
#define ACE2K_FEED_SLIP_ALLOW_UM  15000U
#define ACE2K_FEED_STALL_CHECK_UM 20000U /* the standstill trigger: this much motor since … */
/* … the strand last moved.  "Moved" is this much encoder travel along the command: more than
 * one count and less than two at every accepted scale — a count is 987..1481 µm across the ±20 %
 * window around 1234.2 µm, two are 1975..2962 — so a strand still to within one count is still,
 * whatever the lane's wheel. */
#define ACE2K_FEED_STANDSTILL_UM 1500U
/* An assist burst never exceeds this (rule 2); a bound reached with the strand still taut chains
 * into the next burst — the forward assist keeps the buffer at rest: a burst on taut (pushed),
 * stopped at rest, or on the full reading as a safety (decided at the bench).  Nothing else starts one: not rest, not a timer. */
#define ACE2K_FEED_ASSIST_BURST_UM 100000U
/* The reverse assist's take-up, chained while this lane's buffer reads full and stopped at rest
 * or on taut — the buffer at rest is its target too (decided at the bench: a creep
 * at rest ran on with nothing holding the strand and would have unloaded the lane). */
#define ACE2K_FEED_ASSIST_BACK_FULL_UM 15000U
#define ACE2K_FEED_LOAD_SETTLE_MS      500U /* after the insert edge: the hand is still pushing */
/* A start repeating the lane's last host start — its sequence, command, length and speed — this
 * long after that mode ended is Klipper's retry of a query whose response was lost, not a new
 * command: accepted, nothing started. */
#define ACE2K_FEED_RETRY_WINDOW_MS 1000U
#define ACE2K_FEED_LOAD_GRACE_UM   50000U /* the comparator waits this much motor into a load */
/* The grip: while a lane's grip is set, its next automatic load pulls
 * only this much, without the tag search, and ends loaded — a new strand taken by the drive gear
 * and left waiting while the head still holds the previous piece.  The bounds keep the whole grip
 * strictly inside the load's grace (the grace ends at < ACE2K_FEED_LOAD_GRACE_UM, the maximum
 * stops 5 mm short of it), so it never meets the comparator.  The host's initial value; bench. */
#define ACE2K_FEED_GRIP_UM     40000U
#define ACE2K_FEED_GRIP_UM_MIN 10000U
#define ACE2K_FEED_GRIP_UM_MAX 45000U
_Static_assert(ACE2K_FEED_GRIP_UM_MAX < ACE2K_FEED_LOAD_GRACE_UM,
               "the grip must end inside the grace");
#define ACE2K_FEED_LOAD_PARK_UM                                                                    \
    300000U /* the path minus a margin: 323 mm from the drive's grip to the outlet on the development unit, parked 23 mm inside */
#define ACE2K_FEED_LOAD_SPEED_UM_S 30000U /* initial value */
/* A reverse move whose strand stands still with the lane's plunger at rest has nothing left to
 * drive: the tail is past the drive gear and only the spool can move it (measured on lane 2 of the
 * development unit: 535 mm rewound freely, then the strand stopped with the
 * sensor covered and the standstill tripped).  The move goes on this much motor past the
 * standstill for the spool to wind the tail out, then ends unload_incomplete; initial value. */
#define ACE2K_FEED_UNLOAD_TAIL_UM 100000U
/* The load's search for a spool's tag: its ceiling (the tag search's rule 3), the default
 * the reader's binding sets, the reacquire of a tag a stop left behind, and the least return
 * worth a move.  The default and the reacquire are values measured on the unit. */
#define ACE2K_FEED_SEARCH_MAX_UM 1000000U
/* one revolution of the largest spool that fits (π × a 200 mm flange ≈ 628 mm) plus the longest
 * tag window measured (116 mm), rounded up to 50 mm */
#define ACE2K_FEED_SEARCH_DEFAULT_UM 750000U
/* a stop's overshoot past the tag (1.2 mm, one encoder count) plus the longest tag window
 * (116 mm), rounded up to 10 mm */
#define ACE2K_FEED_REACQUIRE_UM  120000U
#define ACE2K_FEED_RETURN_MIN_UM 2000U /* under two encoder counts: already there */
/* The tip snag: a strand tip thicker than the strand catches the entry
 * of the tube the plunger carries and drags the plunger to an end while nothing holds the strand
 * ahead (measured on the unit: stuck after 41.2 mm with the strand following the motor 1:1).
 * Forward — a feed, a load's pull or its tag search — this lane's full is watched instead of
 * failed at once (in the search, its stuck verdicts are the obstacle return instead): the
 * motor may turn fwd_um more while the strand follows within lag_um and the duty stays within its
 * guard; then a touch of back_um against the move frees the plunger, which must read rest within
 * rest_ms, and the move resumes.  Toward the spool — a rollback, an unload — a comparator trip
 * with the strand taut is forgiven once the strand has come back free_um in the mode.  One
 * tolerance per mode.  The defaults; ace2k_feed_snag_set() changes them within the bounds. */
/* Settled at the bench.
 * FWD_UM: the comparator's standstill; never contradicted — every real catch released on its own
 * within it.  LAG_UM: about four encoder counts; no real catch on a sound tube lagged, and a jam
 * is cut sooner (12 mm was tried and is a host option). */
#define ACE2K_FEED_SNAG_FWD_UM     20000U
#define ACE2K_FEED_SNAG_FWD_MIN_UM 5000U
#define ACE2K_FEED_SNAG_FWD_MAX_UM 50000U
#define ACE2K_FEED_SNAG_LAG_UM     5000U
#define ACE2K_FEED_SNAG_LAG_MIN_UM 1000U
#define ACE2K_FEED_SNAG_LAG_MAX_UM 20000U
/* The duty guard, off: built as a second guard for a hold the strand slips through (the encoder
 * still following), it did not prove effective at the bench — the lag decided every jam first,
 * and a fixed excess over the feed-forward has no margin (≤ 9 points healthy, ≤ 6 a tip's catch,
 * 10–14 a jam at its stop).
 * Kept, settable, for a possible future case; if ever needed, its better form is a peak reading —
 * the rise since the full, which cancels a spool's drag — not built. */
#define ACE2K_FEED_SNAG_DUTY_PCT     0U
#define ACE2K_FEED_SNAG_DUTY_MAX_PCT 100U
/* BACK_UM, REST_MS: never contradicted at the bench; the touch was never needed on a sound tube
 * — they stand on the host tests. */
#define ACE2K_FEED_SNAG_BACK_UM     5000U
#define ACE2K_FEED_SNAG_BACK_MIN_UM 2000U
#define ACE2K_FEED_SNAG_BACK_MAX_UM 20000U
#define ACE2K_FEED_SNAG_REST_MS     500U
#define ACE2K_FEED_SNAG_REST_MIN_MS 100U
#define ACE2K_FEED_SNAG_REST_MAX_MS 2000U
/* well above the buffer's travel, ≈ 18 mm (docs/hardware.md, the buffer's readings): the most a
 * strand held from the mode's start can come back; settled at the bench — a hold from the start
 * came back 46.3 mm and stayed under it */
#define ACE2K_FEED_SNAG_FREE_UM     50000U
#define ACE2K_FEED_SNAG_FREE_MIN_UM 20000U
#define ACE2K_FEED_SNAG_FREE_MAX_UM 200000U
/* The follow: both assists' cycles in one arm.  Initial values; bench —
 * ace2k_feed_follow_set() changes them within the bounds compiled beside them.
 * FLIP_MS: the follow's rule 1, no hunting — after a move of the follow ends, a move the other way waits for
 * rest or this long.  TAKE_UM: one take-up of the follow (the reverse assist keeps its own
 * ACE2K_FEED_ASSIST_BACK_FULL_UM). */
#define ACE2K_FEED_FOLLOW_FLIP_MS     200U
#define ACE2K_FEED_FOLLOW_FLIP_MS_MIN 50U
#define ACE2K_FEED_FOLLOW_FLIP_MS_MAX 2000U
#define ACE2K_FEED_FOLLOW_TAKE_UM     15000U
#define ACE2K_FEED_FOLLOW_TAKE_UM_MIN 5000U
#define ACE2K_FEED_FOLLOW_TAKE_UM_MAX 30000U
/* TAIL_UM: the tail's bound — motor since the insert fell in the
 * follow; initial value; bench. */
#define ACE2K_FEED_FOLLOW_TAIL_UM     2000000U
#define ACE2K_FEED_FOLLOW_TAIL_UM_MIN 200000U
#define ACE2K_FEED_FOLLOW_TAIL_UM_MAX 2000000U
/* follow_dir with no direction to hold against: armed, or rest read since the last move */
#define ACE2K_FOLLOW_DIR_NONE 0xFFU
/* The feed-forward.  Initial values; bench —
 * ace2k_feed_ff_set() changes them within the bounds compiled beside them.
 * CHUNK_UM: one dose, and half the debt's cap.  PULSE_UM_S: the dose's speed. */
#define ACE2K_FEED_FF_CHUNK_UM       3000U
#define ACE2K_FEED_FF_CHUNK_UM_MIN   1000U
#define ACE2K_FEED_FF_CHUNK_UM_MAX   10000U
#define ACE2K_FEED_FF_PULSE_UM_S     15000U
#define ACE2K_FEED_FF_PULSE_UM_S_MIN ACE2K_LANE_SPEED_MIN_UM_S
#define ACE2K_FEED_FF_PULSE_UM_S_MAX ACE2K_LANE_SPEED_MAX_UM_S
/* the period of ace2k_feed_tick(): the base rate accrues once per tick */
#define ACE2K_FEED_TICK_MS 10U

enum ace2k_feed_mode {
    ACE2K_FEED_IDLE = 0,
    ACE2K_FEED_FEEDING = 1,
    ACE2K_FEED_ROLLING_BACK = 2,
    ACE2K_FEED_UNLOADING = 3,
    ACE2K_FEED_LOADING = 4,
    ACE2K_FEED_ASSISTING = 5,
    ACE2K_FEED_ASSISTING_BACK = 6,
    /* the lane stays here, the motor stopped, until ace2k_feed_clear() — or until the strand
     * leaves the insert sensor: the error was about the strand, and without one there is none,
     * so the lane goes idle with a runout (the one exception to rule 8, decided at the bench) */
    ACE2K_FEED_ERROR = 7,
    /* the follow: forward bursts on taut, take-ups on this lane's full, each ended at rest;
     * with the insert fallen, its tail */
    ACE2K_FEED_FOLLOWING = 8,
};

/* The wire's mode argument of ace2k_feed_start. */
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
    /* the error kinds: 7..13.  stuck: the strand does not pass ahead — a forward comparator's
     * trip with the buffer full, now only in the follow, the forward assist and a load's forward
     * return; toward the spool, the trip with the strand taut, or this lane's full the tip snag's
     * tolerance does not forgive in a rollback or an unload.  The same obstacle in a feed or a
     * load's pull ends blocked (18), not an error.  tangled: the forward trip without the full,
     * in the same modes */
    ACE2K_FEED_STUCK = 7,
    ACE2K_FEED_TANGLED = 8,
    ACE2K_FEED_MOTOR_STALLED = 9,
    ACE2K_FEED_TIMEOUT = 10,
    /* reserved — never produced: a forward assist's strand taut and still under command is the
     * comparator's standstill, answered as tangled (decided at the bench, and
     * the timed rule removed since).  The value stays reserved on the wire — the host's kind
     * table names it — so nothing shifts */
    ACE2K_FEED_ASSIST_STALL = 11,
    /* not produced any more: a burst's bound reached while taut chains into the next
     * burst (the head is pulling, no fault).  The value stays reserved on the wire — the host's
     * kind table names it — so nothing shifts */
    ACE2K_FEED_ASSIST_OVERRUN = 12,
    /* the reverse move could not go on: the strand is still at the sensor and nothing is left to
     * drive — the tail is past the drive and the spool did not wind it out within
     * ACE2K_FEED_UNLOAD_TAIL_UM of motor, or the budget ran out.  An error like the others: a
     * hand is needed */
    ACE2K_FEED_UNLOAD_INCOMPLETE = 13,
    /* a notice, the lane goes on: a burst reached its bound with the strand still taut — the
     * head consumes faster than a burst delivers; once per taut episode */
    ACE2K_FEED_BEHIND = 14,
    /* a notice, the lane goes on: a tip caught the buffer's tube and the tolerance freed it —
     * the event carries the mode's odometers where it caught */
    ACE2K_FEED_SNAG = 15,
    /* a notice, the lane goes on: the insert fell in the follow — the strand's tail is on its way
     * to the drive, and the follow feeds on without the insert */
    ACE2K_FEED_TAIL = 16,
    /* the tail's end, the lane idle, no error: a comparator trip in the tail — the tail has left
     * the drive — or the tail's bound of motor reached */
    ACE2K_FEED_TAIL_OUT = 17,
    /* a push toward the head met what cannot move, the lane idle, no error: in a feed or a load's pull, a comparator trip — the
     * standstill or the partial — or this lane's full the tip snag's tolerance does not forgive.  The strand waits where it
     * stopped; a load blocked before its parking point is a strand parked there */
    ACE2K_FEED_BLOCKED = 18,
};

/* The answer to ace2k_feed_start(): 0 accepted, otherwise why not. */
enum ace2k_feed_refusal {
    ACE2K_FEED_ACCEPTED = 0,
    ACE2K_FEED_REFUSED_BUSY = 1,
    ACE2K_FEED_REFUSED_IN_ERROR = 2,
    ACE2K_FEED_REFUSED_NO_FILAMENT = 3,
    ACE2K_FEED_REFUSED_NO_LINK = 4,
    ACE2K_FEED_REFUSED_BOUNDS = 5,
    ACE2K_FEED_REFUSED_OTHER_LANE = 6, /* the read with motion: another lane is busy */
};

enum ace2k_feed_sub {
    ACE2K_FEED_SUB_NONE = 0,
    ACE2K_FEED_SUB_SETTLE = 1, /* load: the 500 ms after the insert edge or the command */
    ACE2K_FEED_SUB_MOVING = 2, /* load: the pull; assist: a burst running */
    /* assist: no move running — the buffer at rest, or full (forward) or taut (reverse), or
     * between two moves; the switches decide the next: forward, taut and not full starts a
     * burst; reverse, this lane's full starts a take-up */
    ACE2K_FEED_SUB_WAITING = 3,
    ACE2K_FEED_SUB_PAUSED = 4,    /* load: a read session holds the lane */
    ACE2K_FEED_SUB_SEARCH = 5,    /* load: past the parking point, looking for the tag */
    ACE2K_FEED_SUB_RETURN = 6,    /* load: back to the parking point (or the read's start) */
    ACE2K_FEED_SUB_REACQUIRE = 7, /* load: backwards at the floor speed for a lost tag */
};

/* How the last load's search ended (ace2k_feed_search_end): the reader's binding turns a search
 * that ran its length, with no UID ever seen, into no_tag. */
enum ace2k_feed_search_end {
    ACE2K_FEED_SEARCH_NONE =
        0, /* no search: the tag read in the pull, another lane busy, none configured */
    ACE2K_FEED_SEARCH_FOUND = 1,
    ACE2K_FEED_SEARCH_OBSTACLE = 2, /* the lane's own buffer full past the outlet */
    ACE2K_FEED_SEARCH_LENGTH = 3,
    ACE2K_FEED_SEARCH_YIELD = 4, /* another lane was given a start */
};

/* The tip snag's phase on a lane (feed_snag.c). */
enum ace2k_feed_snag_phase {
    ACE2K_FEED_SNAG_NONE = 0,
    ACE2K_FEED_SNAG_WATCH = 1, /* forward: the full read, the move still running, judged */
    ACE2K_FEED_SNAG_BACK = 2,  /* the touch against the mode's direction is running */
    ACE2K_FEED_SNAG_REST = 3,  /* the touch done, waiting for the plunger at rest */
};

struct ace2k_feed_snag_cfg {
    uint32_t fwd_um;  /* forward: motor past the full before the touch */
    uint32_t lag_um;  /* forward: the strand behind the motor in the watch that is a jam */
    uint32_t back_um; /* the touch's length */
    uint32_t rest_ms; /* the wait for rest after the touch */
    uint32_t free_um; /* reverse: the strand's travel back in the mode before a trip is forgiven */
    uint8_t duty_pct; /* forward: the duty above the feed-forward that is a jam; 0 off */
};

struct ace2k_feed_follow_cfg {
    uint32_t flip_ms; /* the follow's rule 1: the other direction after rest or this long */
    uint32_t take_um; /* one take-up */
    uint32_t tail_um; /* the tail's bound: motor since it began */
};

struct ace2k_feed_ff_cfg {
    uint32_t chunk_um;   /* one dose; the debt is capped at two */
    uint32_t pulse_um_s; /* the dose's speed */
};

/* The feed-forward's counters on a lane since the follow was armed. */
struct ace2k_feed_ff_counts {
    uint16_t doses;      /* doses started */
    uint16_t taut_fixes; /* taut episodes whose burst started: the strand short of the head */
    uint16_t full_fixes; /* this lane's full episodes whose take-up started: too much strand */
    uint8_t epoch;       /* steps (wrapping) at every reset of the three: a new count */
};

struct ace2k_feed_inputs {
    uint8_t insert;  /* bit per lane: filament present at the slot mouth */
    uint8_t rest;    /* bit per lane: the plunger at rest */
    uint8_t pushed;  /* bit per lane: the plunger at its pushed end — the strand taut */
    bool pulled_any; /* some plunger at its pulled end — a buffer full (the shared input) */
    bool link_ok;
    /* the reader's binding, a bit per lane (0 in an image without it): a read session holds the
     * lane; its tag is read; its last session lost the tag */
    uint8_t tag_hold;
    uint8_t tag_read;
    uint8_t tag_lost;
};

struct ace2k_feed_thresholds {
    uint32_t slip_check_um;
    uint32_t slip_allow_um;
    uint32_t stall_check_um;
};

struct ace2k_feed_load_cfg {
    uint32_t park_um;
    uint32_t speed_um_s;
    bool auto_load;
};

struct ace2k_feed_event {
    uint8_t lane;
    uint8_t kind;        /* enum ace2k_feed_kind */
    uint8_t mode;        /* enum ace2k_feed_mode: where it happened */
    uint8_t seq;         /* the seq of the start that entered the mode; 0: the firmware's own */
    uint32_t motor_um;   /* the tach's travel since the mode started */
    int32_t filament_um; /* the encoder's travel since the mode started, + toward the printer */
};

struct ace2k_feed_lane {
    uint8_t mode;        /* enum ace2k_feed_mode */
    uint8_t error_kind;  /* the kind that put the lane in error */
    uint8_t failed_mode; /* in error: the mode it happened in — the runout that ends it names it */
    uint8_t sub;         /* enum ace2k_feed_sub */
    /* The start's sequence, echoed in every event of this mode — done, stopped, runout, the
     * errors, every kind.  0 with the lane idle or in a mode the unit entered on its own (the
     * automatic load): go_idle() clears it once the mode's events are out (a finish emits before
     * the mode changes), so an event of an idle lane never echoes a stale host seq, and the
     * tick's own entries pass 0.  A lane in error keeps the seq of the start that failed until
     * ace2k_feed_clear(). */
    uint8_t seq;
    /* The last start the unit accepted from the host on the lane, as it asked — its seq (0 until
     * one), command, length and speed — and when the mode it entered ended: a start repeating
     * all four while that mode runs, or within ACE2K_FEED_RETRY_WINDOW_MS after it ended, is the
     * transport's retry of that start, and answers accepted without starting again.  Kept
     * through idle and error; a mode the unit enters on its own leaves all of them as they are,
     * its end included.  last_seq is what the state report publishes (ace2k_feed_seq). */
    uint8_t last_seq;
    uint8_t last_cmd; /* enum ace2k_feed_cmd */
    uint32_t last_length_um;
    uint32_t last_speed_um_s;
    uint32_t t_end_ms;
    uint32_t speed_um_s; /* the command's speed */
    uint32_t length_um;  /* the command's length or budget */
    uint32_t fg_start; /* the odometers' bases: the raw tach and the encoder at the mode's start */
    int32_t enc_um_start;
    uint32_t fg_window; /* the partial trigger's window: its bases, restarted at every close */
    int32_t enc_um_window;
    /* the standstill's base: the tach and the encoder when the strand last moved along the
     * command by ACE2K_FEED_STANDSTILL_UM or more — its own, not the window's */
    uint32_t fg_still;
    int32_t enc_um_still;
    uint32_t t_mode_ms; /* when the mode was entered */
    bool behind_sent;   /* assist: the behind notice for this taut episode is out */
    /* a reverse move's standstill, or its deadline, with the plunger at rest: the tail is past
     * the drive, the comparator has said its piece, and the mode goes on for
     * ACE2K_FEED_UNLOAD_TAIL_UM of motor from fg_tail (the raw tach at that first tick) */
    bool tail_out;
    uint32_t fg_tail;
    uint16_t bursts;
    bool insert_last;
    /* the load: the phase a pause or a reacquire goes back to; how the
     * search ended; whether the search began, was asked to yield, and has reacquired once; the
     * travel since the load began that it returns to — the parking point, or 0 for a read with
     * motion */
    uint8_t resume_sub;
    uint8_t search_end;
    bool searched;
    bool yield;
    bool reacquired;
    bool gripped; /* the automatic load a grip turned into a pull of grip_um alone */
    uint32_t return_um;
    /* the tip snag (feed_snag.c): its phase, whether the mode's one tolerance is spent, the
     * counts at its start (the watch's base), when the wait for rest began, and the mode's
     * odometers at its start — what the notice carries */
    uint8_t snag; /* enum ace2k_feed_snag_phase */
    bool snag_used;
    uint32_t fg_snag;
    int32_t enc_um_snag;
    uint32_t t_snag_ms;
    uint32_t snag_motor_um;
    int32_t snag_filament_um;
    /* the follow: the running or the last move's direction (enum ace2k_lane_dir), or
     * ACE2K_FOLLOW_DIR_NONE once rest was read — and when that move ended: the follow's rule 1 clock */
    uint8_t follow_dir;
    uint32_t follow_end_ms;
    /* the feed-forward, FOLLOWING only — every entry and every
     * end clear all of it but the counters, which the next entry clears (as bursts) */
    uint32_t base_um_s;  /* the host's rate for the lane, 0 none */
    uint32_t debt_um;    /* strand owed to the head, capped at 2 × chunk */
    uint32_t debt_rem;   /* the sub-µm remainder of base × tick, in thousandths, carried */
    bool dose;           /* the running move is a dose (not a burst nor a take-up) */
    int32_t dose_enc_um; /* the encoder at the dose's start */
    bool taut_seen;      /* this taut episode already counted */
    bool full_seen;      /* this full episode already counted */
    uint16_t doses, taut_fixes, full_fixes; /* since the entry */
    uint8_t ff_epoch; /* steps (wrapping) at every entry, which resets the three */
    /* the follow's tail: the insert fell in FOLLOWING, the follow feeds
     * on without it and takes up nothing; tail_fg_start is the raw tach when it began, the base of
     * its bound.  Not the reverse moves' tail_out above.  Cleared at every entry and every end;
     * the insert rising again leaves it set — the tail ignores the bay */
    bool tail;
    uint32_t tail_fg_start;
};

struct ace2k_feed {
    struct ace2k_lane *lane;
    struct ace2k_feed_lane l[ACE2K_LANE_COUNT];
    struct ace2k_feed_thresholds th;
    struct ace2k_feed_load_cfg load;
    struct ace2k_feed_snag_cfg snag;
    struct ace2k_feed_follow_cfg follow;
    struct ace2k_feed_ff_cfg ff;
    uint32_t search_um; /* the search's reach as travel since the load began; 0: no search */
    /* per lane, the grip the next automatic load takes (and clears); 0: none.  RAM only */
    uint32_t grip_um[ACE2K_LANE_COUNT];
    /* the last tick's inputs, whole: the refusals of ace2k_feed_start() read the link and the
     * insert, and the assists' first move is the tick's own step run on them — one reading of the
     * buffer for arming and for the tick */
    struct ace2k_feed_inputs last;
    bool primed;
    /* a Klipper shutdown was seen (ace2k_feed_shutdown): the tick reads the link as down and
     * every start is refused as if it were, until ace2k_feed_init — the MCU's reset */
    bool shut_down;
    struct ace2k_feed_event ring[ACE2K_FEED_EVENT_RING];
    uint8_t head, tail;
    uint32_t dropped;
};

/* Every lane idle, the default thresholds and load settings, link_ok false until the first tick. */
void ace2k_feed_init(struct ace2k_feed *self, struct ace2k_lane *lane);

/* One 10 ms tick, after the lane's.  Interrupt context. */
void ace2k_feed_tick(struct ace2k_feed *self, uint32_t now_ms, const struct ace2k_feed_inputs *in);

/* A motion command.  length_um: feed / rollback the length, unload the budget (0: the default),
 * load the parking distance (0: the configured one), assist ignored; speed_um_s within the lane's
 * bounds — the assist's too, there is no configured assist speed (the host always sends one);
 * the load's 0 is the configured one; seq the host's sequence for this start, echoed in
 * every event of the mode it enters (any value; the host never sends 0 — 0 is reserved for a
 * motion the firmware starts on its own).  ACE2K_FEED_ACCEPTED, or the refusal — a feed or an
 * unload with no strand at the mouth is ACE2K_FEED_REFUSED_NO_FILAMENT; a rollback on an empty
 * lane is accepted (it brings a piece parked past the sensor back out).  A start equal to the
 * last start the unit accepted from the host on the lane in seq (non-zero), command, length and
 * speed, while the mode that start entered still runs or within ACE2K_FEED_RETRY_WINDOW_MS after
 * it ended, is the transport's retry of that start: accepted, nothing started, the host's
 * completion served by the event that went or goes out (a mode of the unit's own running or
 * ending on the lane neither closes nor re-opens the window).  The same seq with another
 * command, length or speed is a new command and takes the normal path (busy, or a new move).
 * Interrupts masked by the caller. */
enum ace2k_feed_refusal ace2k_feed_start(struct ace2k_feed *self, uint8_t lane,
                                         enum ace2k_feed_cmd cmd, uint32_t length_um,
                                         uint32_t speed_um_s, uint8_t seq, uint32_t now_ms);

/* A Klipper shutdown (rule 6): every lane stopped with its
 * pending lane result discarded — the lane's own shutdown handler may have left one, and whichever
 * handler runs first, a lane gets one stopped_shutdown —; a lane in a mode goes idle with that
 * event, a lane in error stays in error (rule 8).  Then the latch: the tick reads link_ok as false
 * for every decision — no burst, no load step, no automatic load, no tail move — and
 * ace2k_feed_start() answers ACE2K_FEED_REFUSED_NO_LINK, until ace2k_feed_init().  The tick
 * outlives a shutdown (the board's tick) and the host keeps sending bytes after one, so the link
 * alone would not keep an armed mode from moving.  Interrupts disabled by the caller. */
void ace2k_feed_shutdown(struct ace2k_feed *self, uint32_t now_ms);

/* Stops the motor whatever the mode (rule 5): a moving or assisting lane goes idle with a
 * `stopped` event — a load still in its settle is cancelled with nothing ever moved (rule 9); a
 * lane in error stays there.  ACE2K_LANE_ALL for every lane. */
void ace2k_feed_stop(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms);

/* error → idle; true when the lane was in error.  A clear ends no mode: the retry window stays
 * the failure's.  The other way out of error is the strand leaving the sensor (the tick's runout
 * on an errored lane); a clear is for an error whose strand stays. */
bool ace2k_feed_clear(struct ace2k_feed *self, uint8_t lane);

/* A new speed for the lane's mode: the running move takes it (its deadline follows from now_ms),
 * and so does the next burst of an assist or the pull of a load still settling — with no move
 * running the speed is still armed.  0, -ACE2K_EINVAL out of bounds, -ACE2K_EREFUSED when the
 * lane is idle or in error. */
int ace2k_feed_set_speed(struct ace2k_feed *self, uint8_t lane, uint32_t speed_um_s,
                         uint32_t now_ms);

/* The supervisors' thresholds, in effect from the next tick.  Asserts nothing: the binding
 * refuses a zero, slip_allow_um ≥ slip_check_um (the partial would never trip) and a
 * stall_check_um or slip_check_um at or above ACE2K_FEED_ASSIST_BURST_UM (every burst rebases
 * the comparator, so such a window never closes in an assist) as a host bug, and the host checks
 * the same before it sends. */
void ace2k_feed_thresholds_set(struct ace2k_feed *self, const struct ace2k_feed_thresholds *th);
void ace2k_feed_load_set(struct ace2k_feed *self, const struct ace2k_feed_load_cfg *cfg);
/* The tip snag's values, in effect from the next read (each is read where it is used, so a change
 * under a running tolerance mixes two sets, every value within its bounds).  0, or -ACE2K_EINVAL
 * for any value outside its bounds (ACE2K_FEED_SNAG_*_MIN/_MAX) or lag_um ≥ fwd_um — nothing
 * taken.  A fwd_um lowered under a running watch may start the touch at once (harmless: the
 * tolerance it shortens is the one the host asked for).  Interrupts masked by the caller. */
int ace2k_feed_snag_set(struct ace2k_feed *self, const struct ace2k_feed_snag_cfg *cfg);

/* The follow's values, read where used (the next move, the next flip).  0, or -ACE2K_EINVAL for
 * a value outside its bounds (ACE2K_FEED_FOLLOW_*_MIN / _MAX) — nothing taken.  Interrupts masked
 * by the caller. */
int ace2k_feed_follow_set(struct ace2k_feed *self, const struct ace2k_feed_follow_cfg *cfg);

/* The feed-forward's values, read where used (the next dose, the next accrual's cap).  0, or
 * -ACE2K_EINVAL for a value outside its bounds (ACE2K_FEED_FF_*_MIN / _MAX) — nothing taken.
 * Interrupts masked by the caller. */
int ace2k_feed_ff_set(struct ace2k_feed *self, const struct ace2k_feed_ff_cfg *cfg);

/* The lane's base rate: the strand the head is about to use,
 * metered out in doses of ff.chunk_um at ff.pulse_um_s while the lane follows.  0 stops the
 * accrual only: what is owed is strand the head has already pulled, still paid in doses.  With
 * clear, what is owed is dropped too — the host's, for a lane remapped to another extruder or
 * one of two lanes following one extruder.  0 taken; -ACE2K_EINVAL for a lane out of range or a rate
 * above ACE2K_LANE_SPEED_MAX_UM_S; -ACE2K_EREFUSED, nothing taken, when the lane is not in
 * FOLLOWING (the follow's arm, and every end, set it to 0).  RAM only.  Interrupts masked by the
 * caller. */
int ace2k_feed_base_set(struct ace2k_feed *self, uint8_t lane, uint32_t rate_um_s, bool clear);

/* The search's reach (0 turns it off; the reader's binding sets it, up to
 * ACE2K_FEED_SEARCH_MAX_UM, which the binding checks). */
void ace2k_feed_search_set(struct ace2k_feed *self, uint32_t search_um);

/* The lane's grip: its next automatic load pulls grip_um alone at the
 * load's speed, with no tag search, and ends loaded; that load clears it, and so does any start
 * the host gives the lane (ace2k_feed_start, ace2k_feed_search_start).  0 clears it.  0, or
 * -ACE2K_EINVAL — nothing taken — for a lane out of range or a grip outside
 * ACE2K_FEED_GRIP_UM_MIN..ACE2K_FEED_GRIP_UM_MAX.  RAM only.  Interrupts masked by the caller. */
int ace2k_feed_grip_set(struct ace2k_feed *self, uint8_t lane, uint32_t grip_um);

/* The read on command with motion: the lane enters its load at the search, forward up to the
 * search's reach from where the strand is and back to there.  The refusals of a start —
 * no_link, in_error, busy, no_filament — then ACE2K_FEED_REFUSED_OTHER_LANE while another lane is
 * busy (the tag search's rule 4), ACE2K_FEED_REFUSED_BOUNDS with no search configured.  Interrupts masked. */
enum ace2k_feed_refusal ace2k_feed_search_start(struct ace2k_feed *self, uint8_t lane,
                                                uint32_t now_ms);

uint8_t ace2k_feed_search_end(const struct ace2k_feed *self, uint8_t lane);

/* The oldest event, left in the ring: the binding tries to send it and pops it only once the
 * frame is queued, so a frame that does not fit waits at the ring's tail for the next tick.
 * The tick writes the head alone and never the slot at the tail (one slot stays free), so the
 * peek and the pop need no mask.  False when the ring is empty. */
bool ace2k_feed_peek_event(const struct ace2k_feed *self, struct ace2k_feed_event *out);
/* The oldest event released without a copy — the binding's step once the peeked frame is queued.
 * Nothing when the ring is empty. */
void ace2k_feed_drop_event(struct ace2k_feed *self);
/* The oldest event, copied and released (a peek and a drop); false when the ring is empty. */
bool ace2k_feed_pop_event(struct ace2k_feed *self, struct ace2k_feed_event *out);
bool ace2k_feed_has_events(const struct ace2k_feed *self);

/* True in every mode but idle and error; ACE2K_LANE_ALL: true when any lane is.  The lane
 * binding's counters-reset veto reads it (the odometers' bases live across those modes), and the
 * dryer's rotation lockout will.  A lane out of range is not busy. */
bool ace2k_feed_lane_busy(const struct ace2k_feed *self, uint8_t lane);
uint8_t ace2k_feed_mode(const struct ace2k_feed *self, uint8_t lane);
uint8_t ace2k_feed_error(const struct ace2k_feed *self, uint8_t lane);
uint16_t ace2k_feed_bursts(const struct ace2k_feed *self, uint8_t lane);
/* The feed-forward's counters since the lane's last entry, kept after the mode ends (as bursts);
 * zeros for a lane out of range. */
struct ace2k_feed_ff_counts ace2k_feed_ff_counts(const struct ace2k_feed *self, uint8_t lane);
/* The sequence of the last start the unit accepted from the host on the lane (0 until one), so
 * a host that reconnects continues after it: the state report publishes it, kept through idle
 * and error; a lane running a host start's move reports that start's seq, a mode of the unit's
 * own leaves it as the last accepted host start's.  0 for a lane out of range. */
uint8_t ace2k_feed_seq(const struct ace2k_feed *self, uint8_t lane);

#endif
