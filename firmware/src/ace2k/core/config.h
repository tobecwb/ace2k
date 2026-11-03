/* What: the record that ace2k keeps on its config page: the link proof, the dryer's log
 * (counters and the last eight faults), and the automatic modes' settings. Also the read-only
 * view of the factory calibration pages of the unit: the four insert-sensor bands.
 * How: the binding owns one struct ace2k_config on the board's flash ops.
 * - ace2k_config_init() loads it. A blank or corrupt page yields the defaults, and says so in
 *   `source`. A layout-1 page is migrated: its head is kept, and the log is empty.
 * - A module changes `record` and calls ace2k_config_store().
 * - encode and decode are exposed for the tests, and for anyone reading a page image.
 * - ace2k_config_factory_read() goes through the same flash_read op: the primary page, then the
 *   backup, then the built-in defaults. It never writes.
 * Stack budget: the 240-byte page image lives in `scratch` of struct ace2k_config, never on the
 * stack, because Klipper's tasks share one 512-byte stack. ace2k_config_init() and
 * ace2k_config_store() keep a few words of locals. store reads the page back one 4-byte word at a
 * time against `scratch`. Neither holds a page-sized buffer.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, calibration.h (the insert band), dryer_log.h
 * (struct ace2k_dryer_log), dryer_auto_store.h (the automatic modes' store), util.h
 * (ACE2K_LANE_COUNT, the CRCs). */
#ifndef ACE2K_CONFIG_H
#define ACE2K_CONFIG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "core/calibration.h"
#include "dryer/dryer_auto_store.h"
#include "dryer/dryer_log.h"
#include "core/util.h"

#define ACE2K_CONFIG_MAGIC 0x32454341U /* "ACE2" read as a little-endian word */
/* 2: with the dryer log and the automatic modes' marked block; 1 (v0.4.0) is migrated */
#define ACE2K_CONFIG_LAYOUT       2U
#define ACE2K_CONFIG_LAYOUT_1     1U
#define ACE2K_CONFIG_VERSION_LEN  32  /* NUL-terminated: 31 characters of `git describe` */
#define ACE2K_CONFIG_RECORD_LEN   240 /* 60 words: the unit programs 32-bit words */
#define ACE2K_CONFIG_LAYOUT_1_LEN 48  /* the v0.4.0 record, CRC at offset 46 */

#define ACE2K_CONFIG_FLAG_LINK_PROVEN 0x1U

struct ace2k_config_record {
    char proven_version[ACE2K_CONFIG_VERSION_LEN]; /* the image that proved the link */
    uint32_t flags;                                /* ACE2K_CONFIG_FLAG_* */
    struct ace2k_dryer_log dryer_log;              /* the dryer's counters and entries */
    struct ace2k_dryer_auto_store dryer_auto;      /* the automatic modes (a marked block) */
};

enum ace2k_config_source {
    ACE2K_CONFIG_FROM_FLASH,            /* the page carried a valid record */
    ACE2K_CONFIG_FROM_DEFAULTS_BLANK,   /* the page is erased (first boot after a flash) */
    ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT, /* magic, layout or CRC wrong: defaults, reported */
};

/* The flash that the record lives on. read never fails. erase and program return 0 or
 * -ACE2K_E<NAME>. The board implementation refuses anything outside the config page. */
struct ace2k_config_ops {
    void (*flash_read)(void *ctx, uint32_t addr, uint8_t *out, uint32_t len);
    int (*flash_erase_page)(void *ctx, uint32_t addr);
    int (*flash_program_word)(void *ctx, uint32_t addr, uint32_t word);
};

struct ace2k_config {
    const struct ace2k_config_ops *ops;
    void *ctx;
    struct ace2k_config_record record;
    enum ace2k_config_source source;
    uint8_t scratch[ACE2K_CONFIG_RECORD_LEN]; /* page image of init and store, off the stack */
};

/* Sets the defaults: no version, no flags, an empty log. */
void ace2k_config_defaults(struct ace2k_config_record *rec);

/* Serialises rec into the 240-byte layout-2 page, CRC included. */
void ace2k_config_encode(const struct ace2k_config_record *rec,
                         uint8_t out[ACE2K_CONFIG_RECORD_LEN]);

/* Parses 240 bytes: layout 2, or a layout-1 head with an empty log. On anything but
 * ACE2K_CONFIG_FROM_FLASH, out holds the defaults. */
enum ace2k_config_source ace2k_config_decode(const uint8_t in[ACE2K_CONFIG_RECORD_LEN],
                                             struct ace2k_config_record *out);

/* Binds the ops and loads the record from the page. */
void ace2k_config_init(struct ace2k_config *self, const struct ace2k_config_ops *ops, void *ctx);

/* Erases the page, programs the record, and reads it back word by word. Returns 0, or the error
 * of the ops, or -ACE2K_EFLASH when a word read back differs.
 * - On success, `source` becomes FROM_FLASH.
 * - On a failure after the erase, it becomes FROM_DEFAULTS_CORRUPT, because the page then holds no
 *   valid record.
 * - A refused erase (-ACE2K_EREFUSED) leaves `source` as it was.
 * - Any other failed erase reads the page again and sets `source` from what it now holds, because
 *   the erase may have run in part. */
int ace2k_config_store(struct ace2k_config *self);

/* The factory calibration (docs/hardware.md "Factory calibration page"). Each 2 KB page starts
 * with a 1064-byte record. Its CRC-16/MCRF4XX is stored little-endian at page offset 0x7FC. The
 * insert pair of lane s is two uint16 millivolt values, at record offsets 2 + 8s and 4 + 8s. A
 * pair whose values differ by 99 mV or less is degenerate, and the defaults apply. That is the
 * factory firmware's own rule. */
#define ACE2K_CONFIG_FACTORY_RECORD_LEN     1064U
#define ACE2K_CONFIG_FACTORY_CRC_OFF        0x7FCU
#define ACE2K_CONFIG_FACTORY_PAIR_STRIDE    8U
#define ACE2K_CONFIG_FACTORY_PAIR_OFF_A     2U
#define ACE2K_CONFIG_FACTORY_PAIR_OFF_B     4U
#define ACE2K_CONFIG_FACTORY_DEGENERATE_MV  99U
#define ACE2K_CONFIG_FACTORY_CHUNK          64U
#define ACE2K_CONFIG_INSERT_DEFAULT_LOW_MV  600U
#define ACE2K_CONFIG_INSERT_DEFAULT_HIGH_MV 1100U

struct ace2k_factory_calibration {
    struct ace2k_insert_band insert[ACE2K_LANE_COUNT];
    enum ace2k_calibration_source source; /* FACTORY when a page was valid, else DEFAULT */
};

/* The defaults for one band. */
void ace2k_insert_band_defaults(struct ace2k_insert_band *band);

/* Reads the primary page, then the backup, then falls back to the defaults. Returns 1 (primary
 * used), 2 (backup) or 0 (defaults). */
int ace2k_config_factory_read(const struct ace2k_config_ops *ops, void *ctx,
                              struct ace2k_factory_calibration *out);

#endif
