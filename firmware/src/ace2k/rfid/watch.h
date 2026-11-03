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
 * length, and the link.  For each reader, a step does at most one thing.  It steps the iso14443a
 * job of the reader by one exchange, or configures the reader after a soft reset, or switches the
 * field, or begins a job.  Only a configure waits, because its soft reset is polled for up to
 * 150 ms (rfid/reader.h).  The host's commands are calls from the same main loop.  They are the
 * step of a session, its end, a read on command, and a forget.
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
 * Attribution (rule 8).  The first ACE2K_RFID_BASELINE_POLLS inventories after the field comes on
 * are the baseline.  The tags in any of them were there at rest.  Take lane m of a reader whose
 * other lane is n.
 * - A UID attributed to n in its insertion, kept or read, is never the UID of m.  This holds
 *   whatever the order of the inventory, and even when the UID enters while m moves.
 * - The UID of m itself, once known, belongs to m.
 * - With n empty, any UID belongs to m.
 * - A UID that enters while m moves and n does not belongs to m.
 * - With both moving, a UID belongs to m only when the tag of n is read.
 * A kept UID is held until the filament of n leaves its insert sensor, or a forget on pending n.
 * After that forget, the tag of n counts as an unknown tag, and the rules above apply to it as to
 * any other.  A read on command leaves out the UID of n, when n is read, reading or kept.  It
 * attributes the one UID left when n is empty or read, and only while the link is up (link_ok).
 * With the link down, it answers ambiguous instead.  It answers the unchanged state when no UID is
 * left, and ambiguous otherwise.  A lane that saw a UID is never no_tag (rule 9).  After a forget
 * on a pending lane, that lane counts as one that saw none.  A searching lane ignores a forget.
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
    bool probe;          /* a read on command waits for an inventory, the baseline done */
    bool probe_other;    /* a UID not the lane's nor the neighbour's was seen since the probe */
    uint32_t t_probe_ms; /* arrival of the read on command; answered by PROBE_MAX_MS */
    bool want;           /* a UID attributed while the session was busy; opened next */
    struct ace2k_iso14443a_tag wanted; /* the UID of want: its session opens with it */
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

struct ace2k_rfid_watch {
    struct ace2k_iso14443a *iso;
    const struct ace2k_rfid_watch_ops *ops;
    void *ctx;
    uint8_t next_session;
    uint32_t now_ms;                   /* the clock of the last step: "now" for host commands */
    struct ace2k_rfid_watch_inputs in; /* those of the last step */
    struct ace2k_rfid_watch_lane l[ACE2K_LANE_COUNT];
    struct ace2k_rfid_watch_reader r[ACE2K_RFID_READER_COUNT];
    struct ace2k_rfid_event ring[ACE2K_RFID_EVENT_RING];
    uint8_t head, tail;
    uint32_t dropped;
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

#endif
