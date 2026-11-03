/* What: the ISO/IEC 14443-A tag protocol on the unit's two readers.  It covers these parts.
 * - CRC_A.
 * - The read-only gate.
 * - The inventory of a field.  It finds every tag in the field by bit-level anticollision, over up
 *   to three cascade levels, and halts each tag once found.
 * - The two reads of one target UID.  An NTAG read reads up to three groups of four pages.  A
 *   MIFARE Classic read reads up to three data blocks of one sector, after an authentication with a
 *   key that the caller hands in.
 * How: each reader runs one job.  ace2k_iso14443a_begin() starts the job, and
 * ace2k_iso14443a_step() advances it once per task run.  Each step polls the running exchange.
 * Once the exchange ended, the step acts on its answer and starts the next one.  A step makes at
 * most one start and one poll, and never waits.  An exchange with no answer after
 * ACE2K_ISO_POLL_MAX polls ends the job in error.
 * A read job that read all its blocks ends with HLTA, so its tag is halted, and the WUPA of the
 * next job wakes it.  A tag left ACTIVE or AUTHENTICATED ignores a WUPA.  For MIFARE, the HLTA is
 * sent while Crypto1 is still on, and the reader encrypts it itself.  When the WUPA of a read job
 * meets silence, it is sent once more before the job ends with TAG_GONE.  The reason is that a job
 * that ended in error sent no HLTA.  Its tag is still ACTIVE, and the first WUPA only drops it to
 * IDLE or HALT, without an answer.
 * Every frame to a tag leaves through ace2k_iso14443a_send().  Its gate lets through only nine
 * first bytes, those of the activation and read commands of the protocol.  So no write, value or
 * password command can reach a tag (rule 1).  The key of a MIFARE job is wiped as soon as its
 * authentication is started, and the whole job is wiped at its end, through ace2k_wipe() (rule 7).
 * The reader behind the ops is rfid/transceive.  The tests put a field of fake tags there.
 * Sources: ISO/IEC 14443-3 for the activation, the anticollision and CRC_A (Annex B).  The NXP
 * MIFARE Classic and NTAG213 datasheets for READ, HLTA and the authentication.
 * Depends on: <stdbool.h>, <stdint.h>, util.h, and rfid/transceive.h for the poll status, the
 * received frame and the reader command codes. */
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
#define ACE2K_ISO_DATA_BLOCKS_MASK     0x07U /* blocks 0..2 of a sector, never its trailer */
#define ACE2K_ISO_NTAG_PAGES_PER_READ  4U
#define ACE2K_ISO_NTAG_LAST_FIRST_PAGE 244U /* the last group of three reads ends on page 255 */
#define ACE2K_ISO_SELECT_FRAME         9U
#define ACE2K_ISO_SAK_FRAME            3U
#define ACE2K_ISO_READ_CMD_FRAME       4U
#define ACE2K_ISO_POLL_MAX     10U /* 100 ms of task runs, four times the timer of the reader */
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
    ACE2K_ISO_ERROR = 3,    /* a bad answer or CRC, a NAK, or a reader that never answered */
};

struct ace2k_iso14443a_tag {
    uint8_t uid[ACE2K_ISO_UID_MAX];
    uint8_t uid_len; /* 4, 7 or 10 */
    uint16_t atqa;   /* the answer to the wake-up, little-endian as sent.  It is 0 after a
                    * collision, or when the answer was not two bytes long.  The tag is still
                    * selected then. */
    uint8_t sak;
};

struct ace2k_iso14443a_job {
    uint8_t kind;                      /* enum ace2k_iso14443a_job_kind */
    struct ace2k_iso14443a_tag target; /* for the reads, the tag to select */
    uint8_t arg;                       /* NTAG: the first page.  MIFARE: the sector, 0..15 */
    uint8_t count; /* NTAG: groups of four pages, 1..3; MIFARE: the data blocks' mask, bits 0..2 */
    uint8_t key_type; /* MIFARE: 0 for key A, 1 for key B */
    uint8_t key[ACE2K_ISO_KEY_BYTES];
};

struct ace2k_iso14443a_out {
    uint8_t result; /* enum ace2k_iso14443a_result */
    struct ace2k_iso14443a_tag tags[ACE2K_ISO_INVENTORY_MAX];
    uint8_t tag_count;
    uint8_t data[ACE2K_ISO_READS_MAX][ACE2K_ISO_BLOCK_BYTES];
    uint8_t data_first[ACE2K_ISO_READS_MAX]; /* the block, or the first page, of each read */
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
    bool woken;   /* the first wake of the inventory was the WUPA; the next ones are REQAs */
    bool rewoken; /* the WUPA of a read met silence once and was sent again */
    uint8_t polls;
    uint8_t level;
    uint8_t cl[ACE2K_ISO_CL_BYTES];
    uint8_t known_bits;
    uint8_t iterations;
    struct ace2k_iso14443a_tag cur;
    uint8_t selected; /* the selections of an inventory; a tag selected twice counts twice */
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

/* Computes the CRC_A of len bytes into out[0..1], low byte first (ISO/IEC 14443-3 Annex B). */
void ace2k_iso14443a_crc_a(const uint8_t *data, uint8_t len, uint8_t out[2]);

/* Returns true for a first byte that the gate lets through.  These are REQA, WUPA, the three SEL
 * codes, HLTA, READ and the two authentication codes.  Nothing that writes, changes a value or
 * presents a password gets through. */
bool ace2k_iso14443a_frame_allowed(uint8_t first);

/* Sends a frame to a tag.  This is the only path for such a frame.  When the first byte is not
 * allowed, it returns -ACE2K_EREFUSED and the reader is not touched.  Otherwise it returns the
 * answer of ops->start. */
int ace2k_iso14443a_send(struct ace2k_iso14443a *self, uint8_t reader, uint8_t cmd,
                         const uint8_t *tx, uint8_t len, uint8_t last_bits);

/* Starts a job.  It returns -ACE2K_EREFUSED while another job runs on the reader.  It returns
 * -ACE2K_EINVAL for a job out of bounds.  Out of bounds means any of these.
 * - A reader ≥ 2.
 * - An unknown kind.
 * - A target UID that is not 4, 7 or 10 bytes long.
 * - NTAG groups outside 1..3, or a first page past ACE2K_ISO_NTAG_LAST_FIRST_PAGE.
 * - A sector ≥ 16, a block mask of 0 or with bit 3 set, or a key type above 1. */
int ace2k_iso14443a_begin(struct ace2k_iso14443a *self, uint8_t reader,
                          const struct ace2k_iso14443a_job *job);

/* Checks the bounds that begin() checks on a job.  It is for a caller that validates a job before
 * it queues it. */
bool ace2k_iso14443a_job_valid(const struct ace2k_iso14443a_job *job);

bool ace2k_iso14443a_busy(const struct ace2k_iso14443a *self, uint8_t reader);

/* Runs one step of the job of the reader.  It returns true on the step where the job ended.  The
 * result can be read until the next begin. */
bool ace2k_iso14443a_step(struct ace2k_iso14443a *self, uint8_t reader);

/* Returns the last result of the reader, or NULL for a reader out of range. */
const struct ace2k_iso14443a_out *ace2k_iso14443a_result(const struct ace2k_iso14443a *self,
                                                         uint8_t reader);

bool ace2k_iso14443a_same_uid(const struct ace2k_iso14443a_tag *a,
                              const struct ace2k_iso14443a_tag *b);

#endif
