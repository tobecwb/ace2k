/* What: the unit's watch over the tags of its four lanes — each lane's tag state, the scheduler
 * that turns a reader's field on only while one of its lanes needs it, the inventories and the
 * attribution of a tag to a lane by that lane's motion, the bounded read session with the host,
 * a read on command without motion, and the events the binding sends.  A read on command is
 * answered within ACE2K_RFID_PROBE_MAX_MS: by its inventory, or by the lane's unchanged state
 * when none decided it in time or the reader is dead.
 * How: ace2k_rfid_watch_step() once per task run with the lanes' inputs (the insert sensors, the
 * motors running, the feed's load and whether its search ran its length, the link); per reader
 * it does one thing at most — step the reader's iso14443a job (one exchange), configure the
 * reader after a soft reset, switch the field, or begin a job — so a step never waits.
 * The host's commands — a session's step, its end, a read on
 * command, a forget — are calls from the same main loop.  A reader's field is on while one of
 * its lanes is pending or searching and moving or in its load, while a session runs on it, or
 * while a read on command looks; otherwise it is switched off and read back off, and a field
 * that stays on (either antenna driver read back enabled) latches the reader's fault.  A
 * reader whose configuration fails is retried after ACE2K_RFID_CONFIGURE_RETRY_MS, and after
 * ACE2K_RFID_CONFIGURE_TRIES failures in a row it is dead until ace2k_rfid_watch_clear_dead()
 * (the health run) or a restart.  After ACE2K_RFID_FIELD_MAX_MS with no lane of the reader
 * moving and no session it is switched off and stays off until a lane of the reader moves or a
 * read is asked (rule 2).  The first inventory after the field comes on is the
 * baseline: tags in it were there at rest.  Attribution (rule 8), for lane m of a reader whose
 * other lane is n: a UID attributed to n is never m's; m's own UID, once known, is m's; with n
 * empty any UID is m's; a UID entering while m moves and n does not is m's; with both moving,
 * only when n's tag is read.  A read on command attributes the one UID in the field when n is
 * empty or read, else answers ambiguous.  A lane that saw a UID is never no_tag (rule 9).  The
 * watch keeps a step's key only until its job has begun (rule 7).  A step's job carries its
 * lane and session: a result that outlives its session goes nowhere, and no session opens on a
 * reader while a job runs on it.
 * Depends on: <stdbool.h>, <stdint.h>, rfid/iso14443a.h, util.h. */
#ifndef ACE2K_RFID_WATCH_H
#define ACE2K_RFID_WATCH_H
#include <stdbool.h>
#include <stdint.h>
#include "rfid/iso14443a.h"
#include "core/util.h"

#define ACE2K_RFID_POLL_MS                                                                         \
    50U /* between two inventories; measured on the unit: no passage missed */
#define ACE2K_RFID_SESSION_IDLE_MS    500U    /* a session with no host step this long ends */
#define ACE2K_RFID_FIELD_MAX_MS       120000U /* rule 2: no lane of the reader moving this long */
#define ACE2K_RFID_EVENT_RING         16U
#define ACE2K_RFID_POWER_UP_MS        10U   /* field on to the first inventory: one task run */
#define ACE2K_RFID_CONFIGURE_RETRY_MS 1000U /* a failed configure (a soft reset) to the next */
#define ACE2K_RFID_CONFIGURE_TRIES    3U    /* failed configures in a row: the reader is dead */
#define ACE2K_RFID_PROBE_MAX_MS       2000U /* a read on command answered at the latest */
#define ACE2K_RFID_OP_NTAG            0U
#define ACE2K_RFID_OP_MIFARE_A        1U
#define ACE2K_RFID_OP_MIFARE_B        2U
#define ACE2K_RFID_KIND_AMBIGUOUS 6U /* a state event's kind: a read on command could not decide */

enum ace2k_rfid_tag_state {
    ACE2K_RFID_UNKNOWN = 0,   /* no strand at the insert sensor */
    ACE2K_RFID_PENDING = 1,   /* a strand, its tag not read: watched on every move */
    ACE2K_RFID_SEARCHING = 2, /* the feed's load (its pull, its search) runs on the lane */
    ACE2K_RFID_READING = 3,   /* a session with the host is open */
    ACE2K_RFID_READ = 4,      /* a tag attributed and its read confirmed by the host */
    ACE2K_RFID_NO_TAG = 5,    /* the load's search ran its length and no UID was ever seen */
};

enum ace2k_rfid_ev_type {
    ACE2K_RFID_EV_TAG = 0,   /* a session opened: lane, session, the tag */
    ACE2K_RFID_EV_DATA = 1,  /* 16 bytes of a step, or a failed step: kind is the status */
    ACE2K_RFID_EV_STATE = 2, /* the lane's new state, or ACE2K_RFID_KIND_AMBIGUOUS */
};

enum ace2k_rfid_data_status { /* the iso14443a results, as the wire carries them */
                              ACE2K_RFID_DATA_OK = 0,
                              ACE2K_RFID_DATA_AUTH_FAILED = 1,
                              ACE2K_RFID_DATA_TAG_GONE = 2,
                              ACE2K_RFID_DATA_ERROR = 3,
};

enum ace2k_rfid_read_refusal { /* the answer to a read on command, as the wire carries it */
                               ACE2K_RFID_READ_OK = 0,
                               ACE2K_RFID_READ_BUSY = 1,
                               ACE2K_RFID_READ_NO_FILAMENT = 2,
                               ACE2K_RFID_READ_NO_LINK = 3,
                               ACE2K_RFID_READ_OTHER_LANE_MOVING =
                                   4, /* the binding's, for a read with motion */
                               ACE2K_RFID_READ_UNSUPPORTED =
                                   5, /* the binding's: a read with motion without the feed */
};

struct ace2k_rfid_event {
    uint8_t type;    /* enum ace2k_rfid_ev_type */
    uint8_t lane;    /* 0..3 */
    uint8_t session; /* 0 for a state event */
    uint8_t kind;    /* the new state; a data status */
    uint8_t block;   /* data: the block, or the first of four pages */
    struct ace2k_iso14443a_tag tag;
    uint8_t data[ACE2K_ISO_BLOCK_BYTES];
};

/* The reader-level control the watch drives besides the tag protocol: rfid/transceive in the
 * binding, a fake in the tests. */
struct ace2k_rfid_watch_ops {
    int (*configure)(void *ctx, uint8_t reader);
    bool (*needs_configure)(void *ctx, uint8_t reader);
    void (*field_set)(void *ctx, uint8_t reader, bool on);
    bool (*field_driven)(void *ctx, uint8_t reader); /* either antenna driver enabled */
};

struct ace2k_rfid_watch_inputs {
    uint8_t insert;  /* bit per lane: a strand at the slot mouth */
    uint8_t moving;  /* bit per lane: the lane's motor runs */
    uint8_t loading; /* bit per lane: the feed's load runs (settle, pull, pause, search, return) */
    uint8_t exhausted; /* bit per lane: the lane's last load search ran its whole length */
    bool link_ok;
};

struct ace2k_rfid_watch_lane {
    uint8_t state;                  /* enum ace2k_rfid_tag_state */
    struct ace2k_iso14443a_tag tag; /* the attributed tag; uid_len 0 when none */
    bool has_tag;                   /* a UID was attributed during this insertion (rule 9) */
    bool lost;                      /* the last session ended with its tag gone */
    bool loading_last;
    bool probe;          /* a read on command waits for the next inventory of the reader */
    uint32_t t_probe_ms; /* the read on command's arrival: answered by PROBE_MAX_MS */
    bool want;           /* a UID attributed while the reader's session was busy: opened next */
    struct ace2k_iso14443a_tag wanted;
};

struct ace2k_rfid_watch_reader {
    bool field;
    bool fault;      /* the field read back on after it was switched off: latched */
    bool suppressed; /* switched off by the ceiling: until a lane of the reader moves */
    bool dead;       /* CONFIGURE_TRIES failed configures in a row: until clear_dead */
    uint8_t configure_failures;
    uint32_t t_configure_failed_ms;
    uint32_t t_last_moving_ms;
    uint32_t t_last_inventory_ms;
    bool baseline_taken;
    struct ace2k_iso14443a_tag baseline[ACE2K_ISO_INVENTORY_MAX];
    uint8_t baseline_count;
    struct ace2k_iso14443a_tag present[ACE2K_ISO_INVENTORY_MAX];
    uint8_t present_count;
    bool session_open;
    uint8_t session_lane;
    uint8_t session_id;
    uint32_t t_session_ms; /* the session's opening, or its last step */
    bool step_pending;     /* a host step waits for the engine */
    bool step_running;
    uint8_t step_lane; /* the running step's session: its result goes to that one only */
    uint8_t step_session;
    uint8_t step_first; /* the step's block, or first page: a failed step's data event */
    struct ace2k_iso14443a_job step_job;
};

struct ace2k_rfid_watch {
    struct ace2k_iso14443a *iso;
    const struct ace2k_rfid_watch_ops *ops;
    void *ctx;
    bool primed;
    uint8_t next_session;
    uint32_t now_ms;                   /* the last step's clock: "now" for the host's commands */
    struct ace2k_rfid_watch_inputs in; /* the last step's */
    struct ace2k_rfid_watch_lane l[ACE2K_LANE_COUNT];
    struct ace2k_rfid_watch_reader r[ACE2K_RFID_READER_COUNT];
    struct ace2k_rfid_event ring[ACE2K_RFID_EVENT_RING];
    uint8_t head, tail;
    uint32_t dropped;
};

/* Reader B serves lanes 1–2 (0, 1), reader A lanes 3–4 (docs/hardware.md "RFID readers"). */
static inline uint8_t ace2k_rfid_reader_of(uint8_t lane)
{
    return lane < 2U ? (uint8_t)ACE2K_RFID_READER_B : (uint8_t)ACE2K_RFID_READER_A;
}

/* The other lane on the same antenna. */
static inline uint8_t ace2k_rfid_neighbour(uint8_t lane)
{
    return (uint8_t)(lane ^ 1U);
}

void ace2k_rfid_watch_init(struct ace2k_rfid_watch *self, struct ace2k_iso14443a *iso,
                           const struct ace2k_rfid_watch_ops *ops, void *ctx);

/* One task run. */
void ace2k_rfid_watch_step(struct ace2k_rfid_watch *self, uint32_t now_ms,
                           const struct ace2k_rfid_watch_inputs *in);

/* A host step of the lane's open session: op ACE2K_RFID_OP_NTAG (arg the first page, count
 * 1..3 groups of four pages, key ignored) or _MIFARE_A / _B (arg the sector, count the mask of
 * its data blocks 0..2, key six bytes).  -ACE2K_EINVAL out of bounds (a host bug);
 * -ACE2K_EREFUSED when the lane has no open session with that id, or a step already waits or
 * runs (a stale host: ignored).  The session's idle timer restarts. */
int ace2k_rfid_watch_step_request(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session,
                                  uint8_t op, uint8_t arg, uint8_t count, const uint8_t *key,
                                  uint32_t now_ms);

/* The host ends the session: read → the lane read; give up → pending (or searching, in its
 * load).  -ACE2K_EINVAL for a lane out of range; -ACE2K_EREFUSED for a session not open.  "Now"
 * is the last ace2k_rfid_watch_step()'s clock (self->now_ms): a session this opens for a waiting
 * lane starts its idle timer there. */
int ace2k_rfid_watch_done(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session, bool read);

/* A read on command without motion: the next inventory of the lane's reader decides. */
enum ace2k_rfid_read_refusal ace2k_rfid_watch_probe(struct ace2k_rfid_watch *self, uint8_t lane);

/* A read or no_tag lane back to pending, its UID forgotten (a tag swapped on the spool). */
void ace2k_rfid_watch_forget(struct ace2k_rfid_watch *self, uint8_t lane);

/* A session open on the lane, or a read on command pending on it: a read with motion is busy
 * until it ends (its search would start under a session the load cannot see begin). */
bool ace2k_rfid_watch_lane_busy(const struct ace2k_rfid_watch *self, uint8_t lane);

uint8_t ace2k_rfid_watch_state(const struct ace2k_rfid_watch *self, uint8_t lane);
const struct ace2k_iso14443a_tag *ace2k_rfid_watch_tag(const struct ace2k_rfid_watch *self,
                                                       uint8_t lane);
/* The three sets of bits the feed's tick reads (the binding publishes them in one word), a bit
 * per lane: a session holds the lane; the lane's
 * tag is read; the lane's last session lost its tag. */
uint8_t ace2k_rfid_watch_hold_bits(const struct ace2k_rfid_watch *self);
uint8_t ace2k_rfid_watch_read_bits(const struct ace2k_rfid_watch *self);
uint8_t ace2k_rfid_watch_lost_bits(const struct ace2k_rfid_watch *self);
bool ace2k_rfid_watch_field(const struct ace2k_rfid_watch *self, uint8_t reader);
bool ace2k_rfid_watch_fault(const struct ace2k_rfid_watch *self, uint8_t reader);
/* The reader gave up after ACE2K_RFID_CONFIGURE_TRIES failed configures. */
bool ace2k_rfid_watch_dead(const struct ace2k_rfid_watch *self, uint8_t reader);
/* The health run: a dead reader is tried again from the next task run. */
void ace2k_rfid_watch_clear_dead(struct ace2k_rfid_watch *self, uint8_t reader);
/* The field on or a job running: the health probe leaves the reader alone. */
bool ace2k_rfid_watch_reader_busy(const struct ace2k_rfid_watch *self, uint8_t reader);

bool ace2k_rfid_watch_peek_event(const struct ace2k_rfid_watch *self, struct ace2k_rfid_event *out);
void ace2k_rfid_watch_drop_event(struct ace2k_rfid_watch *self);

#endif
