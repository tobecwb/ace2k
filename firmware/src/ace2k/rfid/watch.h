/* What: the watch of the unit over the tags of its four lanes.  It covers these parts.
 * - The tag state of each lane.
 * - The scheduler that turns the field of a reader on only while one of its lanes needs it.
 * - The inventories, and the attribution of a tag to a lane by the motion of that lane.
 * - The bounded read session with the host.
 * - A read on command, without motion.
 * - The events that the binding sends.
 * A read on command is answered within ACE2K_RFID_PROBE_MAX_MS.  The inventory that completes the
 * baseline answers it, or the next one once the baseline is done, and the baseline's tags count.
 * When no inventory decided it in time, or when the reader is dead, the lane's unchanged state
 * answers it.
 * How: call ace2k_rfid_watch_step() once per task run, with the inputs of the lanes.  The inputs
 * are the insert sensors, the motors that run, the feed's load and whether its search ran its
 * length, the direction and travel of each lane, whether a confirmation leg ran out, and the link.
 * For each reader, a step does at most one thing.  It steps the iso14443a job of the reader by one
 * exchange, or configures the reader after a soft reset, or switches the field, or begins a
 * job.  Only a configure waits, because its soft reset is polled for up to 150 ms
 * (rfid/reader.h).  The host's commands are calls from the same main loop.  They are the step of a
 * session, its end, a read on command, and a forget.
 * The field.  The field of a reader is on in these cases.
 * - One of its lanes is pending or searching, and moving or in its load.
 * - A session runs on the reader.
 * - A read on command looks.
 * Otherwise the field is switched off and read back off.  When a field stays on, the fault of the
 * reader latches.  A field stays on when either antenna driver reads back enabled.  Then the field
 * is switched off again at every step until it reads back off.  After ACE2K_RFID_FIELD_MAX_MS with
 * no lane of the reader moving and no session, the field is switched off.  It stays off until a
 * lane of the reader moves or a read is asked (rule 2).
 * Failures.  When the configuration of a reader fails, it is retried after
 * ACE2K_RFID_CONFIGURE_RETRY_MS.  After ACE2K_RFID_CONFIGURE_TRIES failures in a row the reader is
 * dead, until ace2k_rfid_watch_clear_dead() (the health run) or a restart.  A reader found reset
 * while its field is on gets no job.  Its field is switched off, and the next switch-on configures
 * it again.  A job that a reset lands in is not trusted.  The result of an inventory is dropped,
 * and a host step ends as failed, with no data.
 * Attribution: protocol.md, "Watching and attribution" (a defined lane, confirmation by following,
 * passive watching, the neighbour's UID).  The first ACE2K_RFID_BASELINE_POLLS inventories after
 * the field comes on are the baseline: the tags in any of them were there at rest.  In the feed's
 * load the watch asks the feed to reverse the lane (the confirm bits, watch_confirm.c), and each
 * step of a confirmation leaves a trace for the bench (ace2k_rfid_watch_peek_trace).  A read on
 * command attributes no new tag: the lane's own UID reopens its session, any other UID that is not
 * the neighbour's answers ambiguous, and none answers the lane's unchanged state.  A lane that saw
 * a UID is never no_tag (rule 9).  After a forget on a pending lane, that lane counts as one that
 * saw none.  A searching lane ignores a forget.
 * Keys and sessions.  The watch keeps the key of a step only until its job has begun (rule 7).  The
 * job of a step carries its lane and its session.  So a result that outlives its session goes
 * nowhere.  No session opens on a reader while a job runs on it.
 * Depends on: <stdbool.h>, <stdint.h>, rfid/iso14443a.h, util.h. */
#ifndef ACE2K_RFID_WATCH_H
#define ACE2K_RFID_WATCH_H
#include <stdbool.h>
#include <stdint.h>
#include "rfid/iso14443a.h"
#include "core/util.h"

/* The time between two inventories.  Measured on the unit, it missed no passage. */
#define ACE2K_RFID_POLL_MS 50U
/* A UID never seen in this field enters at its first inventory after the baseline.  One that was
 * seen re-enters only after this many inventories in a row without it.  3 (150 ms) rides
 * out a tag at the field's edge missing one or two inventories; a turning spool keeps a tag away
 * much longer.  Chosen, not measured. */
#define ACE2K_RFID_DEBOUNCE_POLLS 3U
/* The first this many inventories after the field comes on are the baseline: a tag seen in any of
 * them is at rest, and a read on command is decided only once they are done.  Failed inventories
 * count too.  A resting tag that all of them miss can still be taken for
 * one that enters.  Chosen, not measured. */
#define ACE2K_RFID_BASELINE_POLLS 3U
/* The UIDs a reader keeps track of: those in view, and those away for fewer than DEBOUNCE_POLLS. */
#define ACE2K_RFID_TRACKED_MAX (2U * ACE2K_ISO_INVENTORY_MAX)
/* A session ends when the host sends no step for this long.  Initial value; re-tune on the
 * bench. */
#define ACE2K_RFID_SESSION_IDLE_MS    500U
#define ACE2K_RFID_FIELD_MAX_MS       120000U /* rule 2: no lane of the reader moved this long */
#define ACE2K_RFID_EVENT_RING         16U
#define ACE2K_RFID_POWER_UP_MS        10U   /* from field on to the first inventory: one run */
#define ACE2K_RFID_CONFIGURE_RETRY_MS 1000U /* from a failed configure (soft reset) to the next */
#define ACE2K_RFID_CONFIGURE_TRIES    3U    /* failed configures in a row make the reader dead */
#define ACE2K_RFID_PROBE_MAX_MS       2000U /* a read on command is answered by then */
#define ACE2K_RFID_OP_NTAG            0U
#define ACE2K_RFID_OP_MIFARE_A        1U
#define ACE2K_RFID_OP_MIFARE_B        2U
#define ACE2K_RFID_KIND_AMBIGUOUS 6U /* kind of a state event: a read on command could not decide */
#define ACE2K_RFID_LANES_PER_READER 2U

/* The constants of the confirmation of a tag on a shared antenna.  The feed moves a lane to see
 * whether a tag follows it.  Initial values; the bench sets the final ones, and the host can change
 * them at run time. */
/* The longest single leg of a confirmation: the Anycubic tag window of 116 mm, and a margin. */
#define ACE2K_RFID_CONFIRM_LEG_MAX_UM 150000
/* The speed of every leg of a confirmation: the floor speed of the reacquire. */
#define ACE2K_RFID_CONFIRM_SPEED_UM_S 9000
/* The reversals that the candidate tag must follow before it is the lane's. */
#define ACE2K_RFID_CONFIRM_FOLLOWS 1
/* How far apart, in the lane's travel, two changes of the candidate's presence may be and still be
 * one reversal followed.  A tag on the lane's own spool changes at the same lane position both
 * ways.  A tag that is lifted and hung back, or that turns with the neighbour's spool, changes
 * somewhere else. */
#define ACE2K_RFID_CONFIRM_WINDOW_UM 30000
/* The confirmations one load of a lane may run.  A tag that never follows (one at the edge of the
 * field) would make the lane go back and forth for the whole load; past this many the lane is left
 * to the tie-break.  Chosen, not measured. */
#define ACE2K_RFID_CONFIRM_PER_LOAD_MAX 3U
/* The least travel of a lane between the leave of a UID and its return, in passive watching.  The
 * shortest revolution of a spool, 432 mm, less the longest tag window, 116 mm, rounded down. */
#define ACE2K_RFID_PASSIVE_MIN_UM 300000
/* The most travel of a lane between the leave of a UID and its return, in passive watching: the
 * least, and about one revolution of the largest spool.  A return later than that is not one turn
 * of the lane's spool. */
#define ACE2K_RFID_PASSIVE_MAX_UM 950000
/* A mask of the lanes in use, older than this, is no mask.  The host sends it every second. */
#define ACE2K_RFID_IN_USE_STALE_MS 3000U
/* The lanes of a reader that are not in use must have been still this long before a tie-break
 * starts. */
#define ACE2K_RFID_TIEBREAK_QUIET_MS 2000U

enum ace2k_rfid_tag_state {
    ACE2K_RFID_UNKNOWN = 0,   /* no filament at the insert sensor */
    ACE2K_RFID_PENDING = 1,   /* filament, its tag not read: watched on every move */
    ACE2K_RFID_SEARCHING = 2, /* the load of the feed (pull, search) runs on the lane */
    ACE2K_RFID_READING = 3,   /* a session with the host is open */
    ACE2K_RFID_READ = 4,      /* a tag attributed, and the host confirmed its read */
    ACE2K_RFID_NO_TAG = 5,    /* the load's search ran its length and no UID was ever seen */
};

enum ace2k_rfid_ev_type {
    ACE2K_RFID_EV_TAG = 0,   /* a session opened: lane, session, the tag */
    ACE2K_RFID_EV_DATA = 1,  /* 16 bytes of a step, or a failed step: kind is the status */
    ACE2K_RFID_EV_STATE = 2, /* the new state of the lane, or ACE2K_RFID_KIND_AMBIGUOUS */
};

/* The iso14443a results, as the wire carries them. */
enum ace2k_rfid_data_status {
    ACE2K_RFID_DATA_OK = 0,
    ACE2K_RFID_DATA_AUTH_FAILED = 1,
    ACE2K_RFID_DATA_TAG_GONE = 2,
    ACE2K_RFID_DATA_ERROR = 3,
};

/* The answer to a read on command, as the wire carries it. */
enum ace2k_rfid_read_refusal {
    ACE2K_RFID_READ_OK = 0,
    ACE2K_RFID_READ_BUSY = 1,
    ACE2K_RFID_READ_NO_FILAMENT = 2,
    ACE2K_RFID_READ_NO_LINK = 3,
    /* from the binding, for a read with motion: another lane busy, other than a follow at rest */
    ACE2K_RFID_READ_OTHER_LANE_MOVING = 4,
    ACE2K_RFID_READ_UNSUPPORTED = 5, /* from the binding: a read with motion, no feed */
};

struct ace2k_rfid_event {
    uint8_t type;    /* enum ace2k_rfid_ev_type */
    uint8_t lane;    /* 0..3 */
    uint8_t session; /* 0 for a state event */
    uint8_t kind;    /* the new state, or a data status */
    uint8_t block;   /* for data, the block or the first of four pages */
    struct ace2k_iso14443a_tag tag;
    uint8_t data[ACE2K_ISO_BLOCK_BYTES];
};

/* The control at reader level that the watch drives, besides the tag protocol.  In the binding it
 * is rfid/transceive, and in the tests a fake. */
struct ace2k_rfid_watch_ops {
    int (*configure)(void *ctx, uint8_t reader);
    bool (*needs_configure)(void *ctx, uint8_t reader);
    void (*field_set)(void *ctx, uint8_t reader, bool on);
    bool (*field_driven)(void *ctx, uint8_t reader); /* either antenna driver enabled */
};

struct ace2k_rfid_watch_inputs {
    uint8_t insert;  /* bit per lane: filament at the bay entrance */
    uint8_t moving;  /* bit per lane: the motor of the lane runs */
    uint8_t loading; /* bit per lane: the feed's load runs (settle, pull, pause, search, return) */
    uint8_t exhausted; /* bit per lane: the lane's last load search ran its whole length */
    uint8_t reverse;   /* bit per lane: its move runs toward the spool */
    int32_t travel_um[ACE2K_LANE_COUNT]; /* the encoder in um, signed, + toward the printer */
    /* Bit per lane: the confirmation leg of the feed ran its longest, or had no room, and the
     * reading did not change. */
    uint8_t confirm_out;
    /* Bit per lane: the epoch of the confirmation that the feed's legs serve (confirm_epoch).  A
     * confirm_out counts only for the confirmation of that epoch. */
    uint8_t confirm_out_epoch;
    bool link_ok;
    /* The insert sensors are unknown (the ADC scan stopped), and `insert` is frozen.  No lane
     * changes state on it, no tag is attributed on it, a read on command is busy, and one already
     * waiting is not decided until the readings are back. */
    bool insert_unknown;
};

struct ace2k_rfid_watch_lane {
    uint8_t state;                  /* enum ace2k_rfid_tag_state */
    struct ace2k_iso14443a_tag tag; /* the attributed tag; uid_len 0 when none */
    bool has_tag;                   /* a UID attributed in this insertion, not forgotten (rule 9) */
    bool lost;                      /* the last session ended with its tag gone */
    bool seen; /* a UID not the neighbour's was in view during this search (rule 9) */
    bool loading_last;
    bool by_hand; /* the host set the lane by hand: no tag is attributed until its strand leaves */
    bool probe;   /* a read on command waits for an inventory, the baseline done */
    bool probe_other;    /* a UID not the lane's nor the neighbour's was seen since the probe */
    uint32_t t_probe_ms; /* arrival of the read on command; answered by PROBE_MAX_MS */
    bool want;           /* a UID attributed while the session was busy; opened next */
    struct ace2k_iso14443a_tag wanted; /* the UID of want: its session opens with it */
    uint8_t confirms; /* the confirmations started in the lane's load, up to the cap */
    /* Turned over at each confirmation started on the lane.  The feed reads it to know a new
     * confirmation, and echoes the epoch of the one its legs serve with confirm_out. */
    bool confirm_epoch;
};

/* What a trace of a confirmation tells, for the bench (ace2k_rfid_confirm_trace, protocol.md). */
enum ace2k_rfid_trace_what {
    ACE2K_RFID_TRACE_START = 0,     /* a confirmation starts */
    ACE2K_RFID_TRACE_FLIP = 1,      /* a change of the candidate's presence, counted */
    ACE2K_RFID_TRACE_WRONG_WAY = 2, /* a change while the lane stood or moved the other way: ends */
    ACE2K_RFID_TRACE_OUTSIDE = 3,   /* a change outside the window: ends */
    ACE2K_RFID_TRACE_LEG_OUT = 4,   /* the leg ran out with no change: ends */
    ACE2K_RFID_TRACE_TAKEN = 5,     /* the candidate followed: it is the lane's */
    ACE2K_RFID_TRACE_ENDED = 6,     /* ended for another reason (the load, the link, ...) */
};

/* One trace of a confirmation: the lane, what happened, the lane's travel and whether the lane
 * moved toward the spool then.  lost counts the traces dropped before it, up to 255. */
struct ace2k_rfid_confirm_trace {
    uint8_t lane;
    uint8_t what; /* enum ace2k_rfid_trace_what */
    bool reverse;
    uint8_t lost;
    int32_t travel_um;
};

/* The traces wait here for the binding; a trace that finds it full is dropped and counted. */
#define ACE2K_RFID_TRACE_RING 8U

/* The confirmation of a candidate tag by the reversals of its lane, or one that waits for it. */
struct ace2k_rfid_confirm {
    bool on;
    uint8_t lane;
    bool reverse;  /* the direction asked of the lane: toward the spool */
    bool present;  /* the candidate in view, as the debounce of the inventories has it */
    uint8_t flips; /* the changes of presence since the start, each one in the asked direction */
    int32_t flip_at_um; /* the lane's travel at the last change counted */
    struct ace2k_iso14443a_tag uid;
};

struct ace2k_rfid_watch_reader {
    bool field;
    /* Switched off, but a driver read back on.  The switch-off is tried again at every step until
     * it reads back off.  Until then, the field is reported on. */
    bool off_pending;
    bool fault;      /* latched: the field read back on after a switch-off */
    bool suppressed; /* off by the time limit, until a lane of the reader moves */
    bool dead;       /* CONFIGURE_TRIES failed configures in a row; until clear_dead */
    uint8_t configure_failures;
    uint32_t t_configure_failed_ms;
    uint32_t t_last_moving_ms;
    uint32_t t_last_inventory_ms;
    uint8_t baseline_polls; /* inventories since field on, up to BASELINE_POLLS: the baseline */
    struct ace2k_iso14443a_tag tracked[ACE2K_RFID_TRACKED_MAX];
    uint8_t absent[ACE2K_RFID_TRACKED_MAX]; /* inventories in a row without tracked[i] */
    uint8_t tracked_count;
    /* The UIDs that left the field while a lane of the reader moved outside a load, for passive
     * watching.  One record per UID and lane: the lane's travel at the leave.  The oldest is
     * dropped when the table is full. */
    struct ace2k_iso14443a_tag gone[ACE2K_RFID_TRACKED_MAX];
    int32_t gone_at_um[ACE2K_RFID_TRACKED_MAX];
    uint8_t gone_lane[ACE2K_RFID_TRACKED_MAX];
    uint8_t gone_count;
    /* At most one confirmation runs on the reader.  A candidate of the other lane waits for it. */
    struct ace2k_rfid_confirm confirm;
    struct ace2k_rfid_confirm waiting;
    bool session_open;
    uint8_t session_lane;
    uint8_t session_id;
    uint32_t t_session_ms; /* the opening of the session, or its last step */
    bool step_pending;     /* a host step waits for the engine */
    bool step_running;
    uint8_t step_lane; /* session of the running step; its result goes there only */
    uint8_t step_session;
    uint8_t step_first; /* block or first page of the step, for a failed step's event */
    struct ace2k_iso14443a_job step_job;
};

/* The settings of the confirmation of a tag on a shared antenna, and of passive watching.  They
 * start at the ACE2K_RFID_CONFIRM_* and ACE2K_RFID_PASSIVE_MIN_UM values. */
struct ace2k_rfid_confirm_cfg {
    int32_t leg_max_um;     /* the longest single leg of a confirmation */
    int32_t speed_um_s;     /* the speed of every leg */
    int32_t passive_min_um; /* the least travel between a UID's leave and its return */
    uint8_t follows;        /* the reversals the candidate must follow */
    int32_t window_um;      /* the most travel between two changes of one reversal followed */
    int32_t passive_max_um; /* the most travel between a UID's leave and its return */
};

/* The bounds that the host's settings of the confirmation must keep.  The speed keeps the lane's
 * own bounds (ace2k_lane_speed_in_bounds). */
#define ACE2K_RFID_CFG_LEG_UM_MIN     10000U   /* a leg shorter than this misses a tag's window */
#define ACE2K_RFID_CFG_LEG_UM_MAX     600000U  /* longer than a revolution of the largest spool */
#define ACE2K_RFID_CFG_PASSIVE_UM_MIN 50000U   /* above an edge flicker, with a margin */
#define ACE2K_RFID_CFG_PASSIVE_UM_MAX 1000000U /* above two revolutions of the largest spool */
#define ACE2K_RFID_CFG_FOLLOWS_MIN    1U       /* a candidate must follow at least one reversal */
#define ACE2K_RFID_CFG_FOLLOWS_MAX    3U       /* more would make every leg pass a spool's turn */
/* A window narrower than this misses the lag of the inventories at a leg's speed, and a wider
 * one lets a neighbour's turning tag through. */
#define ACE2K_RFID_CFG_WINDOW_UM_MIN 5000U
#define ACE2K_RFID_CFG_WINDOW_UM_MAX 100000U
/* The passive maximum is above the minimum, and at most three revolutions of the largest spool. */
#define ACE2K_RFID_CFG_PASSIVE_MAX_UM_MAX 2000000U

/* The settings of the confirmation as the host sends them, in this order. */
enum ace2k_rfid_cfg_index {
    ACE2K_RFID_CFG_LEG,         /* um */
    ACE2K_RFID_CFG_SPEED,       /* um/s */
    ACE2K_RFID_CFG_PASSIVE_MIN, /* um */
    ACE2K_RFID_CFG_FOLLOWS,     /* reversals */
    ACE2K_RFID_CFG_WINDOW,      /* um */
    ACE2K_RFID_CFG_PASSIVE_MAX, /* um, above the passive minimum */
    ACE2K_RFID_CFG_COUNT,
};

struct ace2k_rfid_watch {
    struct ace2k_iso14443a *iso;
    const struct ace2k_rfid_watch_ops *ops;
    void *ctx;
    uint8_t next_session;
    uint32_t now_ms;                   /* the clock of the last step: "now" for host commands */
    struct ace2k_rfid_watch_inputs in; /* those of the last step */
    uint8_t in_use;                    /* the host's mask of the lanes in use */
    bool in_use_set;                   /* a mask was set, and the link has not dropped since */
    uint32_t t_in_use_ms;              /* the clock when the mask was set */
    struct ace2k_rfid_confirm_cfg cfg;
    struct ace2k_rfid_watch_lane l[ACE2K_LANE_COUNT];
    struct ace2k_rfid_watch_reader r[ACE2K_RFID_READER_COUNT];
    struct ace2k_rfid_event ring[ACE2K_RFID_EVENT_RING];
    uint8_t head, tail;
    uint32_t dropped;
    struct ace2k_rfid_confirm_trace trace[ACE2K_RFID_TRACE_RING];
    uint8_t trace_head, trace_tail;
    uint8_t trace_lost; /* the traces dropped since a trace was last sent, up to 255 */
};

/* Reader B serves lanes 1–2 (index 0, 1), and reader A serves lanes 3–4 (docs/hardware.md "RFID
 * readers"). */
static inline uint8_t ace2k_rfid_reader_of(uint8_t lane)
{
    return lane < ACE2K_RFID_LANES_PER_READER ? (uint8_t)ACE2K_RFID_READER_B
                                              : (uint8_t)ACE2K_RFID_READER_A;
}

/* The other lane on the same antenna. */
static inline uint8_t ace2k_rfid_neighbour(uint8_t lane)
{
    return (uint8_t)(lane ^ 1U);
}

void ace2k_rfid_watch_init(struct ace2k_rfid_watch *self, struct ace2k_iso14443a *iso,
                           const struct ace2k_rfid_watch_ops *ops, void *ctx);

/* Runs one task step. */
void ace2k_rfid_watch_step(struct ace2k_rfid_watch *self, uint32_t now_ms,
                           const struct ace2k_rfid_watch_inputs *in);

/* Queues a host step of the open session of the lane.  op is one of these.
 * - ACE2K_RFID_OP_NTAG: arg is the first page, count is 1..3 groups of four pages, and key is
 *   ignored.
 * - ACE2K_RFID_OP_MIFARE_A or _MIFARE_B: arg is the sector, count is the mask of its data blocks
 *   0..2, and key is six bytes.
 * It returns -ACE2K_EINVAL out of bounds, which is a bug in the host.  It returns -ACE2K_EREFUSED
 * when the lane has no open session with that id, or when a step already waits or runs.  That comes
 * from a stale host, and is ignored.  The idle timer of the session restarts. */
int ace2k_rfid_watch_step_request(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session,
                                  uint8_t op, uint8_t arg, uint8_t count, const uint8_t *key,
                                  uint32_t now_ms);

/* The host ends the session.  With read, the lane becomes read.  When the host gives up, the lane
 * goes back to pending, or to searching during its load.  It returns -ACE2K_EINVAL for a lane out
 * of range, and -ACE2K_EREFUSED for a session that is not open.  "Now" is the clock of the last
 * ace2k_rfid_watch_step() (self->now_ms).  A session that this call opens for a waiting lane starts
 * its idle timer there. */
int ace2k_rfid_watch_done(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session, bool read);

/* Asks for a read on command, without motion.  The next inventory of the reader of the lane
 * decides. */
enum ace2k_rfid_read_refusal ace2k_rfid_watch_probe(struct ace2k_rfid_watch *self, uint8_t lane);

/* Sets a read or no_tag lane back to pending, and forgets its UID.  It is for a tag swapped on the
 * spool.  Its state event (pending) answers a read on command that waits on the lane.  On a pending
 * lane, it drops the UID kept from a session that ended without a read, and a UID that waits for
 * the reader's session.  The lane stays pending, and no event goes out.  A read on command that
 * waits stays, and the next inventory answers it.  The lane's next move reads the tag again, and
 * the neighbour may take it by the rules.  A searching or reading lane is left alone. */
void ace2k_rfid_watch_forget(struct ace2k_rfid_watch *self, uint8_t lane);

/* The host's forget (ace2k_rfid_forget): ace2k_rfid_watch_forget, and the lane's by-hand mark goes
 * too, whatever its state, so the lane can be given a tag again.  The read with motion forgets
 * without it: a lane set by hand stays defined through it. */
void ace2k_rfid_watch_host_forget(struct ace2k_rfid_watch *self, uint8_t lane);

/* Returns true while a session is open on the lane, or a read on command is pending on it.  A read
 * with motion is busy until it ends.  Otherwise its search would start under a session that the
 * load cannot see begin. */
bool ace2k_rfid_watch_lane_busy(const struct ace2k_rfid_watch *self, uint8_t lane);

/* Starts the search of a read with motion on the lane.  It returns ACE2K_RFID_READ_OK when the
 * search started, and the refusal otherwise. */
typedef enum ace2k_rfid_read_refusal (*ace2k_rfid_search_start_fn)(void *ctx, uint8_t lane);

/* A read with motion.  The search moves the filament forward up to its reach, and back.  A busy
 * lane answers ACE2K_RFID_READ_BUSY before the search is asked, so a refusal changes nothing.  Once
 * the search started, a read or no_tag lane forgets its tag.  Otherwise the search would run its
 * whole reach past a tag that the watch no longer looks for.  It returns what start answered. */
enum ace2k_rfid_read_refusal ace2k_rfid_watch_read_with_motion(struct ace2k_rfid_watch *self,
                                                               uint8_t lane,
                                                               ace2k_rfid_search_start_fn start,
                                                               void *ctx);

/* The host sets a lane by hand: it took the spool or the strand itself.  Until the insert sensor
 * of the lane falls, no tag is attributed to it.  A read lane keeps its UID, so the neighbour still
 * cannot take it.  The mark is taken whenever the lane's insert sensor read filament at the last
 * step, whatever the lane's state (it may not be pending yet right after an insertion).  A lane
 * with no filament, or out of range, takes no mark.  A link drop keeps the mark: only the strand
 * leaving clears it, and a reset of the unit. */
void ace2k_rfid_watch_set_by_hand(struct ace2k_rfid_watch *self, uint8_t lane);

/* The lanes set by hand, one bit per lane. */
uint8_t ace2k_rfid_watch_by_hand_bits(const struct ace2k_rfid_watch *self);

/* The host sends the mask of the lanes in use (a bit per lane).  The watch keeps it with the clock
 * of the last step. */
void ace2k_rfid_watch_set_in_use(struct ace2k_rfid_watch *self, uint8_t mask);

/* Returns the mask of the lanes in use.  It returns false, with the mask 0, when none was set, the
 * link dropped since, or the mask is older than ACE2K_RFID_IN_USE_STALE_MS. */
bool ace2k_rfid_watch_in_use(const struct ace2k_rfid_watch *self, uint8_t *mask);

/* The host sets the confirmation, one value per enum ace2k_rfid_cfg_index: the longest leg, the
 * speed of every leg, the least travel of passive watching, the reversals to follow, the window
 * of a reversal followed, and the most travel of passive watching.  It returns 0, or
 * -ACE2K_EINVAL when one value is outside its bounds (ACE2K_RFID_CFG_*, the lane's bounds for the
 * speed, and above the passive minimum for the passive maximum), and then nothing is taken.  The
 * settings apply from the next leg, and from the next return in passive watching. */
int ace2k_rfid_watch_confirm_set(struct ace2k_rfid_watch *self,
                                 const uint32_t v[ACE2K_RFID_CFG_COUNT]);

uint8_t ace2k_rfid_watch_state(const struct ace2k_rfid_watch *self, uint8_t lane);
/* Returns the tag attributed to the lane, or NULL for a lane out of range. */
const struct ace2k_iso14443a_tag *ace2k_rfid_watch_tag(const struct ace2k_rfid_watch *self,
                                                       uint8_t lane);
/* The three sets of bits that the feed's tick reads, one bit per lane.  The binding publishes them
 * in one word.
 * - hold: a session holds the lane.
 * - read: the lane's tag is read.
 * - lost: the last session of the lane lost its tag. */
uint8_t ace2k_rfid_watch_hold_bits(const struct ace2k_rfid_watch *self);
uint8_t ace2k_rfid_watch_read_bits(const struct ace2k_rfid_watch *self);
uint8_t ace2k_rfid_watch_lost_bits(const struct ace2k_rfid_watch *self);
/* The confirmation of a candidate tag, one bit per lane, for the feed.
 * - confirm: a confirmation runs on the lane.  The feed moves it at cfg.speed_um_s, in the asked
 *   direction, until the reading changes or the leg reaches cfg.leg_max_um (then confirm_out).
 * - confirm_reverse: of those, the lanes asked to go toward the spool.
 * - confirm_epoch: the epoch of each lane's last confirmation. */
uint8_t ace2k_rfid_watch_confirm_bits(const struct ace2k_rfid_watch *self);
uint8_t ace2k_rfid_watch_confirm_reverse_bits(const struct ace2k_rfid_watch *self);
/* The epoch of each lane's confirmation (confirm_epoch), one bit per lane.  It turns over at each
 * confirmation started, so the feed starts a fresh leg for a new one even in the same direction. */
uint8_t ace2k_rfid_watch_confirm_epoch_bits(const struct ace2k_rfid_watch *self);
/* The reader's field.  It reads on from its switch-on until a switch-off reads back off. */
bool ace2k_rfid_watch_field(const struct ace2k_rfid_watch *self, uint8_t reader);
bool ace2k_rfid_watch_fault(const struct ace2k_rfid_watch *self, uint8_t reader);
/* True when the reader gave up after ACE2K_RFID_CONFIGURE_TRIES failed configures. */
bool ace2k_rfid_watch_dead(const struct ace2k_rfid_watch *self, uint8_t reader);
/* The health run calls this.  A dead reader is tried again from the next task run. */
void ace2k_rfid_watch_clear_dead(struct ace2k_rfid_watch *self, uint8_t reader);
/* Returns true while the field is wanted on, or a job runs.  Then a probe leaves the reader alone.
 * A switch-off that has not read back off does not count.  No read runs then, and the soft reset of
 * a probe is a switch-off too. */
bool ace2k_rfid_watch_reader_busy(const struct ace2k_rfid_watch *self, uint8_t reader);

bool ace2k_rfid_watch_peek_event(const struct ace2k_rfid_watch *self, struct ace2k_rfid_event *out);
void ace2k_rfid_watch_drop_event(struct ace2k_rfid_watch *self);

/* The oldest trace of a confirmation not sent yet, and its drop once sent. */
bool ace2k_rfid_watch_peek_trace(const struct ace2k_rfid_watch *self,
                                 struct ace2k_rfid_confirm_trace *out);
void ace2k_rfid_watch_drop_trace(struct ace2k_rfid_watch *self);

#endif
