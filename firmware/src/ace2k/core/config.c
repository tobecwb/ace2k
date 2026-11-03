#include "core/config.h"
#include "core/guard.h"
#include "core/util.h"

#define OFF_MAGIC    0
#define OFF_LAYOUT   4
#define OFF_VERSION  8
#define OFF_FLAGS    40
#define OFF_LOG      44 /* layout 2: the dryer log */
#define OFF_CRC_L1   46 /* layout 1: the CRC right after the two spare bytes of the flags */
#define OFF_CRC      (ACE2K_CONFIG_RECORD_LEN - 2)
#define LOG_COUNTERS 6U /* cycles_started, cycles_completed, heat_s, full_power_s, faults, flags */
#define ENTRY_LEN    16U
#define OFF_ENTRIES  (OFF_LOG + (LOG_COUNTERS * 4U))
#define C_STARTED    0U /* the order of the counters on the page */
#define C_COMPLETED  1U
#define C_HEAT_S     2U
#define C_FULL_POWER 3U
#define C_FAULTS     4U
#define C_FLAGS      5U
#define E_KIND       0U
#define E_REASON     1U
#define E_CYCLE      2U
#define E_HEAT_S     4U
#define E_LEFT       8U
#define E_RIGHT      10U
#define E_CHAMBER    12U
#define BLANK_BYTE   0xFFU
#define WORD_BYTES   4U

#define OFF_AUTO      (OFF_ENTRIES + (ACE2K_DRYER_LOG_ENTRIES * ENTRY_LEN)) /* 196 */
#define AUTO_MARK     0xA7U /* the block is present; an older image writes 0 here */
#define AUTO_MODE     1U
#define AUTO_LIMIT    2U
#define AUTO_MAX      3U
#define AUTO_FAILURES 4U
#define AUTO_PROFILE  5U
#define AUTO_FLAGS    6U
#define AUTO_MINUTES  7U /* u16 LE */
#define AUTO_LEN      9U

_Static_assert(OFF_AUTO + AUTO_LEN <= OFF_CRC, "the automatic modes fit before the CRC");
_Static_assert(OFF_ENTRIES + (ACE2K_DRYER_LOG_ENTRIES * ENTRY_LEN) <= OFF_CRC,
               "the dryer's log fits before the CRC");

static bool all_blank(const uint8_t *p, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (p[i] != BLANK_BYTE) {
            return false;
        }
    }
    return true;
}

void ace2k_config_defaults(struct ace2k_config_record *rec)
{
    rec->proven_version[0] = '\0';
    rec->flags = 0;
    rec->dryer_log = (struct ace2k_dryer_log){ 0 };
    ace2k_dryer_auto_store_defaults(&rec->dryer_auto);
}

static void put_i16(uint8_t *p, int16_t v)
{
    ace2k_put_u16_le(p, (uint16_t)v);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)ace2k_get_u16_le(p);
}

static void encode_log(const struct ace2k_dryer_log *log, uint8_t *out)
{
    const uint32_t counters[LOG_COUNTERS] = {
        log->cycles_started, log->cycles_completed, log->heat_s,
        log->full_power_s,   log->faults,           log->flags,
    };
    for (uint32_t i = 0; i < LOG_COUNTERS; i++) {
        ace2k_put_u32_le(out + OFF_LOG + ((size_t)i * WORD_BYTES), counters[i]);
    }
    for (uint32_t i = 0; i < ACE2K_DRYER_LOG_ENTRIES; i++) {
        const struct ace2k_dryer_log_entry *e = &log->entry[i];
        uint8_t *p = out + OFF_ENTRIES + ((size_t)i * ENTRY_LEN);
        p[E_KIND] = e->kind;
        p[E_REASON] = e->reason;
        ace2k_put_u16_le(p + E_CYCLE, e->cycle);
        ace2k_put_u32_le(p + E_HEAT_S, e->heat_s);
        put_i16(p + E_LEFT, e->ntc_left_dc);
        put_i16(p + E_RIGHT, e->ntc_right_dc);
        put_i16(p + E_CHAMBER, e->chamber_dc);
    }
}

static void decode_log(const uint8_t *in, struct ace2k_dryer_log *log)
{
    uint32_t c[LOG_COUNTERS];
    for (uint32_t i = 0; i < LOG_COUNTERS; i++) {
        c[i] = ace2k_get_u32_le(in + OFF_LOG + ((size_t)i * WORD_BYTES));
    }
    log->cycles_started = c[C_STARTED];
    log->cycles_completed = c[C_COMPLETED];
    log->heat_s = c[C_HEAT_S];
    log->full_power_s = c[C_FULL_POWER];
    log->faults = c[C_FAULTS];
    log->flags = c[C_FLAGS];
    for (uint32_t i = 0; i < ACE2K_DRYER_LOG_ENTRIES; i++) {
        struct ace2k_dryer_log_entry *e = &log->entry[i];
        const uint8_t *p = in + OFF_ENTRIES + ((size_t)i * ENTRY_LEN);
        e->kind = p[E_KIND];
        e->reason = p[E_REASON];
        e->cycle = ace2k_get_u16_le(p + E_CYCLE);
        e->heat_s = ace2k_get_u32_le(p + E_HEAT_S);
        e->ntc_left_dc = get_i16(p + E_LEFT);
        e->ntc_right_dc = get_i16(p + E_RIGHT);
        e->chamber_dc = get_i16(p + E_CHAMBER);
        e->reserved = 0;
    }
}

static void encode_auto(const struct ace2k_dryer_auto_store *s, uint8_t *out)
{
    uint8_t *p = out + OFF_AUTO;
    p[0] = AUTO_MARK;
    p[AUTO_MODE] = s->mode;
    p[AUTO_LIMIT] = s->limit_pct;
    p[AUTO_MAX] = s->max_failures;
    p[AUTO_FAILURES] = s->failures;
    p[AUTO_PROFILE] = s->profile_c;
    p[AUTO_FLAGS] = s->flags;
    ace2k_put_u16_le(p + AUTO_MINUTES, s->profile_min);
}

static void decode_auto(const uint8_t *in, struct ace2k_dryer_auto_store *s)
{
    const uint8_t *p = in + OFF_AUTO;
    if (p[0] != AUTO_MARK) {
        return; /* an older image's page: the defaults stand */
    }
    s->mode = p[AUTO_MODE];
    s->limit_pct = p[AUTO_LIMIT];
    s->max_failures = p[AUTO_MAX];
    s->failures = p[AUTO_FAILURES];
    s->profile_c = p[AUTO_PROFILE];
    s->flags = p[AUTO_FLAGS];
    s->profile_min = ace2k_get_u16_le(p + AUTO_MINUTES);
}

void ace2k_config_encode(const struct ace2k_config_record *rec,
                         uint8_t out[ACE2K_CONFIG_RECORD_LEN])
{
    for (size_t i = 0; i < ACE2K_CONFIG_RECORD_LEN; i++) {
        out[i] = 0;
    }
    ace2k_put_u32_le(out + OFF_MAGIC, ACE2K_CONFIG_MAGIC);
    ace2k_put_u16_le(out + OFF_LAYOUT, (uint16_t)ACE2K_CONFIG_LAYOUT);
    ace2k_str_copy((char *)(out + OFF_VERSION), ACE2K_CONFIG_VERSION_LEN, rec->proven_version);
    ace2k_put_u32_le(out + OFF_FLAGS, rec->flags);
    encode_log(&rec->dryer_log, out);
    encode_auto(&rec->dryer_auto, out);
    ace2k_put_u16_le(out + OFF_CRC, ace2k_crc16_ccitt(out, OFF_CRC));
}

static void decode_head(const uint8_t *in, struct ace2k_config_record *out)
{
    /* The field may lack a NUL on a page written by a future layout. So the copy reads at most 31
     * bytes of it, and terminates the result itself. */
    ace2k_str_copy(out->proven_version, ACE2K_CONFIG_VERSION_LEN, (const char *)(in + OFF_VERSION));
    out->flags = ace2k_get_u32_le(in + OFF_FLAGS);
}

/* Layout 1 (v0.4.0): the head, and its CRC at 46. The log starts empty: that is the migration. */
static bool layout1_valid(const uint8_t *in)
{
    if (ace2k_get_u16_le(in + OFF_LAYOUT) != ACE2K_CONFIG_LAYOUT_1) {
        return false;
    }
    return ace2k_get_u16_le(in + OFF_CRC_L1) == ace2k_crc16_ccitt(in, OFF_CRC_L1);
}

static bool layout2_valid(const uint8_t *in)
{
    if (ace2k_get_u16_le(in + OFF_LAYOUT) != ACE2K_CONFIG_LAYOUT) {
        return false;
    }
    return ace2k_get_u16_le(in + OFF_CRC) == ace2k_crc16_ccitt(in, OFF_CRC);
}

/* What a page image holds, without decoding it. It needs no record buffer, so a store can ask it
 * of the page with the stack it has. */
static enum ace2k_config_source page_source(const uint8_t in[ACE2K_CONFIG_RECORD_LEN])
{
    if (all_blank(in, ACE2K_CONFIG_RECORD_LEN)) {
        return ACE2K_CONFIG_FROM_DEFAULTS_BLANK;
    }
    if (ace2k_get_u32_le(in + OFF_MAGIC) != ACE2K_CONFIG_MAGIC) {
        return ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT;
    }
    if (layout2_valid(in) || layout1_valid(in)) {
        return ACE2K_CONFIG_FROM_FLASH;
    }
    return ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT;
}

enum ace2k_config_source ace2k_config_decode(const uint8_t in[ACE2K_CONFIG_RECORD_LEN],
                                             struct ace2k_config_record *out)
{
    ace2k_config_defaults(out);
    enum ace2k_config_source source = page_source(in);
    if (source != ACE2K_CONFIG_FROM_FLASH) {
        return source;
    }
    decode_head(in, out);
    if (layout2_valid(in)) {
        decode_log(in, &out->dryer_log);
        decode_auto(in, &out->dryer_auto);
    }
    return ACE2K_CONFIG_FROM_FLASH;
}

void ace2k_config_init(struct ace2k_config *self, const struct ace2k_config_ops *ops, void *ctx)
{
    self->ops = ops;
    self->ctx = ctx;
    ops->flash_read(ctx, ACE2K_GUARD_CONFIG_PAGE_ADDR, self->scratch, ACE2K_CONFIG_RECORD_LEN);
    self->source = ace2k_config_decode(self->scratch, &self->record);
}

/* Reads the programmed page back one word at a time, against the image in scratch. */
static int readback_matches(struct ace2k_config *self)
{
    for (uint32_t off = 0; off < ACE2K_CONFIG_RECORD_LEN; off += WORD_BYTES) {
        uint8_t word[WORD_BYTES];
        self->ops->flash_read(self->ctx, ACE2K_GUARD_CONFIG_PAGE_ADDR + off, word, WORD_BYTES);
        if (ace2k_get_u32_le(word) != ace2k_get_u32_le(self->scratch + off)) {
            return -ACE2K_EFLASH;
        }
    }
    return 0;
}

/* Programs the record into the erased page, and reads it back. */
static int program_and_check(struct ace2k_config *self)
{
    for (uint32_t off = 0; off < ACE2K_CONFIG_RECORD_LEN; off += WORD_BYTES) {
        int rc = self->ops->flash_program_word(self->ctx, ACE2K_GUARD_CONFIG_PAGE_ADDR + off,
                                               ace2k_get_u32_le(self->scratch + off));
        if (rc != 0) {
            return rc;
        }
    }
    return readback_matches(self);
}

int ace2k_config_store(struct ace2k_config *self)
{
    ace2k_config_encode(&self->record, self->scratch);
    int rc = self->ops->flash_erase_page(self->ctx, ACE2K_GUARD_CONFIG_PAGE_ADDR);
    if (rc == -ACE2K_EREFUSED) {
        return rc; /* refused before the controller was touched, so the page is as it was */
    }
    if (rc != 0) {
        /* The controller failed, or the erase ran and a word did not read back erased. The page
         * may be untouched, erased or partly erased. So it is read again and reported as it is. */
        self->ops->flash_read(self->ctx, ACE2K_GUARD_CONFIG_PAGE_ADDR, self->scratch,
                              ACE2K_CONFIG_RECORD_LEN);
        self->source = page_source(self->scratch);
        return rc;
    }
    rc = program_and_check(self);
    if (rc != 0) {
        /* The page is erased or half written, and holds no valid record. Say so now, not only at
         * the next boot. */
        self->source = ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT;
        return rc;
    }
    self->source = ACE2K_CONFIG_FROM_FLASH;
    return 0;
}

void ace2k_insert_band_defaults(struct ace2k_insert_band *band)
{
    band->low_mv = ACE2K_CONFIG_INSERT_DEFAULT_LOW_MV;
    band->high_mv = ACE2K_CONFIG_INSERT_DEFAULT_HIGH_MV;
    band->source = ACE2K_CAL_DEFAULT;
}

/* True when the page at page_addr carries a record whose CRC matches. Reads in 64-byte chunks. */
static bool factory_page_valid(const struct ace2k_config_ops *ops, void *ctx, uint32_t page_addr)
{
    uint8_t chunk[ACE2K_CONFIG_FACTORY_CHUNK];
    uint16_t crc = ACE2K_CRC16_MCRF4XX_INIT;
    for (uint32_t off = 0; off < ACE2K_CONFIG_FACTORY_RECORD_LEN;
         off += ACE2K_CONFIG_FACTORY_CHUNK) {
        uint32_t len = ACE2K_CONFIG_FACTORY_RECORD_LEN - off;
        if (len > ACE2K_CONFIG_FACTORY_CHUNK) {
            len = ACE2K_CONFIG_FACTORY_CHUNK;
        }
        ops->flash_read(ctx, page_addr + off, chunk, len);
        crc = ace2k_crc16_mcrf4xx_update(crc, chunk, len);
    }
    ops->flash_read(ctx, page_addr + ACE2K_CONFIG_FACTORY_CRC_OFF, chunk, 2U);
    return ace2k_get_u16_le(chunk) == crc;
}

/* The four bands of a page assumed valid: low = min, high = max, and the degenerate rule. */
static void factory_bands(const struct ace2k_config_ops *ops, void *ctx, uint32_t page_addr,
                          struct ace2k_insert_band out[ACE2K_LANE_COUNT])
{
    uint8_t raw[ACE2K_CONFIG_FACTORY_PAIR_STRIDE * ACE2K_LANE_COUNT];
    ops->flash_read(ctx, page_addr, raw, sizeof raw);
    for (uint32_t s = 0; s < ACE2K_LANE_COUNT; s++) {
        const uint8_t *rec = raw + ((size_t)s * ACE2K_CONFIG_FACTORY_PAIR_STRIDE);
        uint16_t a = ace2k_get_u16_le(rec + ACE2K_CONFIG_FACTORY_PAIR_OFF_A);
        uint16_t b = ace2k_get_u16_le(rec + ACE2K_CONFIG_FACTORY_PAIR_OFF_B);
        uint16_t low = a < b ? a : b;
        uint16_t high = a < b ? b : a;
        if ((uint32_t)high - low <= ACE2K_CONFIG_FACTORY_DEGENERATE_MV) {
            ace2k_insert_band_defaults(&out[s]);
            continue;
        }
        out[s].low_mv = low;
        out[s].high_mv = high;
        out[s].source = ACE2K_CAL_FACTORY;
    }
}

int ace2k_config_factory_read(const struct ace2k_config_ops *ops, void *ctx,
                              struct ace2k_factory_calibration *out)
{
    const uint32_t pages[] = {
        ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR,
        ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR,
    };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(pages); i++) {
        if (factory_page_valid(ops, ctx, pages[i])) {
            factory_bands(ops, ctx, pages[i], out->insert);
            out->source = ACE2K_CAL_FACTORY;
            return (int)i + 1;
        }
    }
    for (uint32_t s = 0; s < ACE2K_LANE_COUNT; s++) {
        ace2k_insert_band_defaults(&out->insert[s]);
    }
    out->source = ACE2K_CAL_DEFAULT;
    return 0;
}
