/* What: the record ace2k persists on its config page — the link proof and the dryer's log
 * (counters and the last eight faults) — and the read-only view of the unit's factory
 * calibration pages (the four insert-sensor bands).
 * How: the binding owns one struct ace2k_config on the board's flash ops; ace2k_config_init()
 * loads it (a blank or corrupt page yields the defaults and says so in `source`; a layout-1 page
 * is migrated: its head kept, the log empty); a module changes `record` and calls
 * ace2k_config_store().  encode/decode are exposed for the tests and for anyone reading a page
 * image.  ace2k_config_factory_read() goes through the same flash_read op — primary page, then
 * backup, then the built-in defaults — and never writes.
 * Stack budget: the 240-byte page image lives in struct ace2k_config's `scratch`, never on the
 * stack — Klipper's tasks share one 512-byte stack.  ace2k_config_init() and
 * ace2k_config_store() keep a few words of locals (store reads the page back one 4-byte word at a
 * time against `scratch`); neither holds a page-sized buffer.
 * Depends on: <stdbool.h>, <stddef.h>, <stdint.h>, guard.h (the page addresses), util.h
 * (ACE2K_LANE_COUNT, the CRCs), dryer.h (struct ace2k_dryer_log — the type only). */
#ifndef ACE2K_CONFIG_H
#define ACE2K_CONFIG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dryer/dryer.h" /* struct ace2k_dryer_log — the type only */
#include "core/util.h"

#define ACE2K_CONFIG_MAGIC        0x32454341U /* "ACE2" read as a little-endian word */
#define ACE2K_CONFIG_LAYOUT       2U          /* 2: the dryer's log; 1 (v0.4.0) is migrated */
#define ACE2K_CONFIG_LAYOUT_1     1U
#define ACE2K_CONFIG_VERSION_LEN  32  /* NUL-terminated: 31 characters of `git describe` */
#define ACE2K_CONFIG_RECORD_LEN   240 /* 60 words: the unit programs 32-bit words */
#define ACE2K_CONFIG_LAYOUT_1_LEN 48  /* the v0.4.0 record, CRC at offset 46 */

#define ACE2K_CONFIG_FLAG_LINK_PROVEN 0x1U

struct ace2k_config_record {
    char proven_version[ACE2K_CONFIG_VERSION_LEN]; /* the image that proved the link */
    uint32_t flags;                                /* ACE2K_CONFIG_FLAG_* */
    struct ace2k_dryer_log dryer_log;              /* the dryer's counters and entries */
};

enum ace2k_config_source {
    ACE2K_CONFIG_FROM_FLASH,            /* the page carried a valid record */
    ACE2K_CONFIG_FROM_DEFAULTS_BLANK,   /* the page is erased (first boot after a flash) */
    ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT, /* magic, layout or CRC wrong: defaults, reported */
};

/* The flash the record lives on.  read never fails; erase and program return 0 or
 * -ACE2K_E<NAME> — the board implementation refuses anything outside the config page. */
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
    uint8_t scratch[ACE2K_CONFIG_RECORD_LEN]; /* the page image of init and store: off the stack */
};

/* Sets the defaults: no version, no flags, an empty log. */
void ace2k_config_defaults(struct ace2k_config_record *rec);

/* Serialises rec into the 240-byte layout-2 page, CRC included. */
void ace2k_config_encode(const struct ace2k_config_record *rec,
                         uint8_t out[ACE2K_CONFIG_RECORD_LEN]);

/* Parses 240 bytes (layout 2, or a layout-1 head with an empty log); on anything but
 * ACE2K_CONFIG_FROM_FLASH, out holds the defaults. */
enum ace2k_config_source ace2k_config_decode(const uint8_t in[ACE2K_CONFIG_RECORD_LEN],
                                             struct ace2k_config_record *out);

/* Binds the ops and loads the record from the page. */
void ace2k_config_init(struct ace2k_config *self, const struct ace2k_config_ops *ops, void *ctx);

/* Erases the page, programs the record, reads it back word by word.  0, or the ops' error, or
 * -ACE2K_EFLASH when a word read back differs; `source` becomes FROM_FLASH only on success. */
int ace2k_config_store(struct ace2k_config *self);

/* The factory calibration — docs/hardware.md "Factory calibration page": a 1064-byte record at
 * the start of each 2 KB page, its CRC-16/MCRF4XX little-endian at page offset 0x7FC; lane s's
 * insert pair is two uint16 millivolt values at record offsets 2 + 8s and 4 + 8s.  A pair whose
 * values differ by 99 mV or less is degenerate and the defaults apply — the factory firmware's
 * own rule. */
#define ACE2K_CONFIG_FACTORY_RECORD_LEN     1064U
#define ACE2K_CONFIG_FACTORY_CRC_OFF        0x7FCU
#define ACE2K_CONFIG_FACTORY_PAIR_STRIDE    8U
#define ACE2K_CONFIG_FACTORY_PAIR_OFF_A     2U
#define ACE2K_CONFIG_FACTORY_PAIR_OFF_B     4U
#define ACE2K_CONFIG_FACTORY_DEGENERATE_MV  99U
#define ACE2K_CONFIG_FACTORY_CHUNK          64U
#define ACE2K_CONFIG_INSERT_DEFAULT_LOW_MV  600U
#define ACE2K_CONFIG_INSERT_DEFAULT_HIGH_MV 1100U

enum ace2k_calibration_source {
    ACE2K_CAL_DEFAULT = 0, /* built-in 600 / 1100 mV */
    ACE2K_CAL_FACTORY = 1, /* the unit's factory page */
    ACE2K_CAL_CONFIG = 2,  /* printer.cfg, through ace2k_sensors_thresholds_set */
};

struct ace2k_insert_band {
    uint16_t low_mv;
    uint16_t high_mv;
    enum ace2k_calibration_source source;
};

struct ace2k_factory_calibration {
    struct ace2k_insert_band insert[ACE2K_LANE_COUNT];
    enum ace2k_calibration_source source; /* FACTORY when a page was valid, else DEFAULT */
};

/* The defaults for one band. */
void ace2k_insert_band_defaults(struct ace2k_insert_band *band);

/* True when the page at page_addr carries a record whose CRC matches; reads in 64-byte chunks. */
bool ace2k_config_factory_page_valid(const struct ace2k_config_ops *ops, void *ctx,
                                     uint32_t page_addr);

/* The four bands of a page assumed valid: low = min, high = max, the degenerate rule. */
void ace2k_config_factory_bands(const struct ace2k_config_ops *ops, void *ctx, uint32_t page_addr,
                                struct ace2k_insert_band out[ACE2K_LANE_COUNT]);

/* Primary, then backup, then defaults.  Returns 1 (primary used), 2 (backup) or 0 (defaults). */
int ace2k_config_factory_read(const struct ace2k_config_ops *ops, void *ctx,
                              struct ace2k_factory_calibration *out);

#endif
