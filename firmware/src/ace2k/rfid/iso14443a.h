/* What: the ISO/IEC 14443-A tag protocol on the unit's two readers — CRC_A, the read-only gate,
 * the inventory of a field (every tag in it, by bit-level anticollision over up to three cascade
 * levels, each tag halted once found) and the two reads of one target UID: up to three groups of
 * four NTAG pages, or up to three data blocks of one MIFARE Classic sector after an
 * authentication with a key the caller hands in.
 * How: a job per reader, begun with ace2k_iso14443a_begin() and advanced by one
 * ace2k_iso14443a_step() per task run — each step polls the exchange running and, once it ended,
 * acts on its answer and starts the next: at most one start and one poll per step, never a wait.
 * An exchange with no answer after ACE2K_ISO_POLL_MAX polls
 * ends the job in error.  A read job that read all its blocks ends with HLTA — for MIFARE sent
 * while Crypto1 is still on, which the reader encrypts itself — so its tag is halted and the next
 * job's WUPA wakes it (a tag left ACTIVE or AUTHENTICATED ignores a WUPA).  A read job's WUPA
 * that meets silence is sent once more before the job ends TAG_GONE: a job that ended in error
 * sent no HLTA, and its tag, still ACTIVE, drops silently to IDLE or HALT on the first WUPA.  Every frame to a tag
 * leaves through ace2k_iso14443a_send(), whose gate
 * lets through only the nine first bytes of the protocol's activation and read commands — no
 * write, value or password command can reach a tag (rule 1).  The key of a MIFARE job is wiped
 * the moment its authentication is started, and the whole job at its end, through ace2k_wipe()
 * (rule 7).  The reader
 * behind the ops is rfid/transceive; the tests put a field of fake tags there.  Sources:
 * ISO/IEC 14443-3 (activation, anticollision, CRC_A in Annex B); NXP MIFARE Classic and NTAG213
 * datasheets (READ, HLTA, authentication).
 * Depends on: <stdbool.h>, <stdint.h>, rfid/transceive.h (the poll status, the received frame,
 * the reader command codes), util.h. */
#ifndef ACE2K_RFID_ISO14443A_H
#define ACE2K_RFID_ISO14443A_H
#include <stdbool.h>
#include <stdint.h>
#include "rfid/transceive.h"
#include "core/util.h"

#define ACE2K_ISO_REQA                 0x26U
#define ACE2K_ISO_WUPA                 0x52U
#define ACE2K_ISO_SEL_CL1              0x93U
#define ACE2K_ISO_SEL_CL2              0x95U
#define ACE2K_ISO_SEL_CL3              0x97U
#define ACE2K_ISO_HLTA                 0x50U
#define ACE2K_ISO_READ                 0x30U
#define ACE2K_ISO_AUTH_A               0x60U
#define ACE2K_ISO_AUTH_B               0x61U
#define ACE2K_ISO_NVB_SELECT           0x70U
#define ACE2K_ISO_NVB_BYTES_SHIFT      4U
#define ACE2K_ISO_CT                   0x88U /* the cascade tag: more UID bytes at the next level */
#define ACE2K_ISO_SAK_CASCADE          0x04U
#define ACE2K_ISO_SHORT_FRAME_BITS     7U
#define ACE2K_ISO_CRC_A_INIT           0x6363U
#define ACE2K_ISO_UID_SINGLE           4U
#define ACE2K_ISO_UID_DOUBLE           7U
#define ACE2K_ISO_UID_MAX              10U /* a triple-size UID */
#define ACE2K_ISO_CL_BYTES             5U  /* a cascade level: four bytes and the BCC */
#define ACE2K_ISO_CL_UID_BYTES         4U
#define ACE2K_ISO_CL_BITS              40U
#define ACE2K_ISO_COLL_BITS_MAX        32U /* a collision can only fall in the four UID bytes */
#define ACE2K_ISO_LEVELS_MAX           3U
#define ACE2K_ISO_INVENTORY_MAX        4U
#define ACE2K_ISO_BLOCK_BYTES          16U
#define ACE2K_ISO_READ_FRAME           18U /* 16 bytes and the CRC_A */
#define ACE2K_ISO_READS_MAX            3U
#define ACE2K_ISO_KEY_BYTES            6U
#define ACE2K_ISO_AUTH_UID_BYTES       4U /* the last four UID bytes enter the authentication */
#define ACE2K_ISO_AUTH_FRAME           12U
#define ACE2K_ISO_MIFARE_SECTORS       16U
#define ACE2K_ISO_BLOCKS_PER_SECTOR    4U
#define ACE2K_ISO_DATA_BLOCKS_MASK     0x07U /* blocks 0..2 of a sector: never its trailer */
#define ACE2K_ISO_NTAG_PAGES_PER_READ  4U
#define ACE2K_ISO_NTAG_LAST_FIRST_PAGE 244U /* the last group of three reads ends on page 255 */
#define ACE2K_ISO_SELECT_FRAME         9U
#define ACE2K_ISO_SAK_FRAME            3U
#define ACE2K_ISO_READ_CMD_FRAME       4U
#define ACE2K_ISO_POLL_MAX     10U /* 100 ms of task runs: four times the reader's own timer */
#define ACE2K_ISO_ANTICOLL_MAX 32U

enum ace2k_iso14443a_job_kind {
    ACE2K_ISO_JOB_NONE = 0,
    ACE2K_ISO_JOB_INVENTORY = 1,
    ACE2K_ISO_JOB_NTAG_READ = 2,
    ACE2K_ISO_JOB_MIFARE_READ = 3,
};

enum ace2k_iso14443a_result {
    ACE2K_ISO_OK = 0,
    ACE2K_ISO_AUTH_FAILED = 1,
    ACE2K_ISO_TAG_GONE = 2, /* the target did not answer two wake-ups or its selection */
    ACE2K_ISO_ERROR = 3,    /* a malformed answer, a CRC, a NAK, a reader that never answered */
};

struct ace2k_iso14443a_tag {
    uint8_t uid[ACE2K_ISO_UID_MAX];
    uint8_t uid_len; /* 4, 7 or 10 */
    uint16_t atqa;   /* the answer to the wake-up, little-endian as sent; 0 after a collision */
    uint8_t sak;
};

struct ace2k_iso14443a_job {
    uint8_t kind;                      /* enum ace2k_iso14443a_job_kind */
    struct ace2k_iso14443a_tag target; /* the reads: the tag to select */
    uint8_t arg;                       /* NTAG: the first page; MIFARE: the sector, 0..15 */
    uint8_t count; /* NTAG: groups of four pages, 1..3; MIFARE: the data blocks' mask, bits 0..2 */
    uint8_t key_type; /* MIFARE: 0 key A, 1 key B */
    uint8_t key[ACE2K_ISO_KEY_BYTES];
};

struct ace2k_iso14443a_out {
    uint8_t result; /* enum ace2k_iso14443a_result */
    struct ace2k_iso14443a_tag tags[ACE2K_ISO_INVENTORY_MAX];
    uint8_t tag_count;
    uint8_t data[ACE2K_ISO_READS_MAX][ACE2K_ISO_BLOCK_BYTES];
    uint8_t data_first[ACE2K_ISO_READS_MAX]; /* the block, or the first page, of each */
    uint8_t data_count;
};

struct ace2k_iso14443a_ops {
    int (*start)(void *ctx, uint8_t reader, uint8_t cmd, const uint8_t *tx, uint8_t len,
                 uint8_t last_bits);
    enum ace2k_rfid_poll (*poll)(void *ctx, uint8_t reader, struct ace2k_rfid_rx *rx);
    void (*crypto_off)(void *ctx, uint8_t reader);
};

struct ace2k_iso14443a_engine {
    struct ace2k_iso14443a_job job;
    struct ace2k_iso14443a_out out;
    uint8_t phase;
    bool waiting;
    bool woken;   /* the inventory's first wake was the WUPA; the next are REQAs */
    bool rewoken; /* a read's WUPA met silence once and was sent again */
    uint8_t polls;
    uint8_t level;
    uint8_t cl[ACE2K_ISO_CL_BYTES];
    uint8_t known_bits;
    uint8_t iterations;
    struct ace2k_iso14443a_tag cur;
    uint8_t reads_total;
    uint8_t reads_done;
    uint8_t read_addr[ACE2K_ISO_READS_MAX];
};

struct ace2k_iso14443a {
    const struct ace2k_iso14443a_ops *ops;
    void *ctx;
    struct ace2k_iso14443a_engine r[ACE2K_RFID_READER_COUNT];
};

void ace2k_iso14443a_init(struct ace2k_iso14443a *self, const struct ace2k_iso14443a_ops *ops,
                          void *ctx);

/* CRC_A of len bytes, low byte first into out[0..1] (ISO/IEC 14443-3 Annex B). */
void ace2k_iso14443a_crc_a(const uint8_t *data, uint8_t len, uint8_t out[2]);

/* The first bytes the gate lets through: REQA, WUPA, the three SEL codes, HLTA, READ and the
 * two authentication codes — nothing that writes, changes a value or presents a password. */
bool ace2k_iso14443a_frame_allowed(uint8_t first);

/* The one path of a frame to a tag: -ACE2K_EREFUSED, the reader untouched, when the first byte
 * is not allowed; otherwise ops->start's answer. */
int ace2k_iso14443a_send(struct ace2k_iso14443a *self, uint8_t reader, uint8_t cmd,
                         const uint8_t *tx, uint8_t len, uint8_t last_bits);

/* Starts a job: -ACE2K_EREFUSED while one runs on the reader, -ACE2K_EINVAL for a job out of
 * bounds (a reader ≥ 2, an unknown kind, a target UID not 4, 7 or 10 bytes, NTAG groups outside
 * 1..3 or a first page past ACE2K_ISO_NTAG_LAST_FIRST_PAGE, a sector ≥ 16, a block mask 0 or
 * with bit 3, a key type above 1). */
int ace2k_iso14443a_begin(struct ace2k_iso14443a *self, uint8_t reader,
                          const struct ace2k_iso14443a_job *job);

/* The bounds begin() checks on a job, for a caller that validates a job before it queues it. */
bool ace2k_iso14443a_job_valid(const struct ace2k_iso14443a_job *job);

bool ace2k_iso14443a_busy(const struct ace2k_iso14443a *self, uint8_t reader);

/* One step of the reader's job; true on the step it ended (its result readable until the next
 * begin). */
bool ace2k_iso14443a_step(struct ace2k_iso14443a *self, uint8_t reader);

const struct ace2k_iso14443a_out *ace2k_iso14443a_result(const struct ace2k_iso14443a *self,
                                                         uint8_t reader);

bool ace2k_iso14443a_same_uid(const struct ace2k_iso14443a_tag *a,
                              const struct ace2k_iso14443a_tag *b);

#endif
