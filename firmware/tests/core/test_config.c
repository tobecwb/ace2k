#include "test.h"
#include "core/config.h"
#include "core/guard.h"
#include "core/util.h"

/* A RAM page that behaves like the flash controller: erase sets 0xFF, a word programs only
 * onto an erased word, every call outside the config page counts as a violation. */
struct ace2k_fake_flash {
    uint8_t page[ACE2K_GUARD_CONFIG_PAGE_SIZE];
    int erases;
    int programs;
    int violations;
    int fail_erase;
    int fail_program;
    int lie_on_readback;
    int lie_at; /* a page offset whose byte reads back flipped, or -1 */
};

static void fake_reset(struct ace2k_fake_flash *f)
{
    for (size_t i = 0; i < sizeof f->page; i++) {
        f->page[i] = 0xFF;
    }
    f->erases = 0;
    f->programs = 0;
    f->violations = 0;
    f->fail_erase = 0;
    f->fail_program = 0;
    f->lie_on_readback = 0;
    f->lie_at = -1;
}

static void fake_read(void *ctx, uint32_t addr, uint8_t *out, uint32_t len)
{
    struct ace2k_fake_flash *f = ctx;
    if (!ace2k_guard_flash_allowed(addr, len)) {
        f->violations++;
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        out[i] = f->page[addr - ACE2K_GUARD_CONFIG_PAGE_ADDR + i];
        if (f->lie_on_readback) {
            out[i] ^= 0x01U;
        }
        if ((int)(addr - ACE2K_GUARD_CONFIG_PAGE_ADDR + i) == f->lie_at) {
            out[i] ^= 0x80U;
        }
    }
}

static int fake_erase(void *ctx, uint32_t addr)
{
    struct ace2k_fake_flash *f = ctx;
    if (!ace2k_guard_flash_allowed(addr, ACE2K_GUARD_CONFIG_PAGE_SIZE)) {
        f->violations++;
        return -ACE2K_EREFUSED;
    }
    f->erases++;
    if (f->fail_erase) {
        return -ACE2K_EFLASH;
    }
    for (size_t i = 0; i < sizeof f->page; i++) {
        f->page[i] = 0xFF;
    }
    return 0;
}

static int fake_program(void *ctx, uint32_t addr, uint32_t word)
{
    struct ace2k_fake_flash *f = ctx;
    if (!ace2k_guard_flash_allowed(addr, 4U) || (addr & 3U) != 0) {
        f->violations++;
        return -ACE2K_EREFUSED;
    }
    f->programs++;
    if (f->fail_program) {
        return -ACE2K_EFLASH;
    }
    uint8_t *p = f->page + (addr - ACE2K_GUARD_CONFIG_PAGE_ADDR);
    for (uint32_t i = 0; i < 4U; i++) {
        if (p[i] != 0xFF) {
            return -ACE2K_EFLASH; /* real flash refuses to program a written word */
        }
        p[i] = (uint8_t)(word >> (8U * i));
    }
    return 0;
}

static const struct ace2k_config_ops ace2k_fake_ops = {
    .flash_read = fake_read,
    .flash_erase_page = fake_erase,
    .flash_program_word = fake_program,
};

static struct ace2k_fake_flash ace2k_flash;

TEST(a_blank_page_loads_the_defaults)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_DEFAULTS_BLANK);
    ASSERT_STR_EQ(c.record.proven_version, "");
    ASSERT_EQ(c.record.flags, 0);
    ASSERT_EQ(ace2k_flash.violations, 0);
}

TEST(encode_then_decode_round_trips)
{
    struct ace2k_config_record in = {
        .proven_version = "0.1.0-3-gabcdef1",
        .flags = ACE2K_CONFIG_FLAG_LINK_PROVEN,
    };
    struct ace2k_config_record out;
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    ace2k_config_encode(&in, raw);
    ASSERT_EQ(raw[0], 'A');
    ASSERT_EQ(raw[1], 'C');
    ASSERT_EQ(raw[2], 'E');
    ASSERT_EQ(raw[3], '2');
    ASSERT_EQ(raw[4], 2);
    ASSERT_EQ(raw[5], 0);
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_FLASH);
    ASSERT_STR_EQ(out.proven_version, "0.1.0-3-gabcdef1");
    ASSERT_EQ(out.flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
}

TEST(store_then_load_returns_the_record)
{
    struct ace2k_config c;
    struct ace2k_config again;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_str_copy(c.record.proven_version, ACE2K_CONFIG_VERSION_LEN, "0.1.0");
    c.record.flags = ACE2K_CONFIG_FLAG_LINK_PROVEN;
    ASSERT_EQ(ace2k_config_store(&c), 0);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_FLASH);
    ASSERT_EQ(ace2k_flash.erases, 1);
    ASSERT_EQ(ace2k_flash.programs, ACE2K_CONFIG_RECORD_LEN / 4);
    ASSERT_EQ(ace2k_flash.violations, 0);
    ace2k_config_init(&again, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(again.source, ACE2K_CONFIG_FROM_FLASH);
    ASSERT_STR_EQ(again.record.proven_version, "0.1.0");
    ASSERT_EQ(again.record.flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
}

TEST(a_corrupt_page_loads_the_defaults)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_str_copy(c.record.proven_version, ACE2K_CONFIG_VERSION_LEN, "0.1.0");
    c.record.flags = ACE2K_CONFIG_FLAG_LINK_PROVEN;
    ASSERT_EQ(ace2k_config_store(&c), 0);
    ace2k_flash.page[12] ^= 0x40U; /* one bit of the version */
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
    ASSERT_STR_EQ(c.record.proven_version, "");
    ASSERT_EQ(c.record.flags, 0);
}

TEST(a_wrong_magic_or_layout_is_corrupt)
{
    struct ace2k_config_record out;
    struct ace2k_config_record in = { .proven_version = "x", .flags = 0 };
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    ace2k_config_encode(&in, raw);
    raw[0] = 'B';
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
    ace2k_config_encode(&in, raw);
    raw[4] = 3;
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
}

TEST(a_long_version_is_truncated_to_31_characters)
{
    /* A 40-character version overflows the 32-byte field: it ends up full, with no NUL.  Built
     * by hand because the host compiler refuses an initializer that long under -Werror. */
    const char *forty = "0123456789012345678901234567890123456789";
    struct ace2k_config_record in = { .proven_version = "", .flags = 0 };
    struct ace2k_config_record out;
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    for (size_t i = 0; i < ACE2K_CONFIG_VERSION_LEN; i++) {
        in.proven_version[i] = forty[i];
    }
    ace2k_config_encode(&in, raw);
    ASSERT_EQ(raw[8 + 31], 0);
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_FLASH);
    ASSERT_STR_EQ(out.proven_version, "0123456789012345678901234567890");
}

TEST(a_failing_erase_is_reported_and_nothing_is_programmed)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_flash.fail_erase = 1;
    ASSERT_EQ(ace2k_config_store(&c), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_flash.programs, 0);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_DEFAULTS_BLANK);
}

TEST(a_failing_program_is_reported)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_flash.fail_program = 1;
    ASSERT_EQ(ace2k_config_store(&c), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_flash.programs, 1);
}

TEST(a_readback_mismatch_is_reported)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_flash.lie_on_readback = 1;
    ASSERT_EQ(ace2k_config_store(&c), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_flash.programs, ACE2K_CONFIG_RECORD_LEN / 4);
}

/* The readback compares word by word: one flipped byte in the last word is still caught. */
TEST(a_single_mismatching_word_on_readback_is_reported)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ace2k_flash.lie_at = ACE2K_CONFIG_RECORD_LEN - 1;
    ASSERT_EQ(ace2k_config_store(&c), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_flash.programs, ACE2K_CONFIG_RECORD_LEN / 4);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_DEFAULTS_BLANK);
    ace2k_flash.lie_at = 100;
    ASSERT_EQ(ace2k_config_store(&c), -ACE2K_EFLASH);
    ace2k_flash.lie_at = -1;
    ASSERT_EQ(ace2k_config_store(&c), 0);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_FLASH);
}

static struct ace2k_config_record sample_record(void)
{
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_str_copy(r.proven_version, ACE2K_CONFIG_VERSION_LEN, "0.5.0-7-gabcdef1");
    r.flags = ACE2K_CONFIG_FLAG_LINK_PROVEN;
    r.dryer_log.cycles_started = 12;
    r.dryer_log.cycles_completed = 10;
    r.dryer_log.heat_s = 86400U;
    r.dryer_log.full_power_s = 40000U;
    r.dryer_log.faults = 2;
    r.dryer_log.flags = ACE2K_DRYER_LOG_CUTOUT;
    for (uint32_t i = 0; i < ACE2K_DRYER_LOG_ENTRIES; i++) {
        struct ace2k_dryer_log_entry *e = &r.dryer_log.entry[i];
        e->kind = ACE2K_DRYER_EV_FAULT;
        e->reason = (uint8_t)(i + 1U);
        e->cycle = (uint16_t)(100U + i);
        e->heat_s = 1000U * (i + 1U);
        e->ntc_left_dc = (int16_t)(550 + (int)i);
        e->ntc_right_dc = (int16_t)(-40 - (int)i);
        e->chamber_dc = INT16_MIN;
        e->reserved = 0;
    }
    return r;
}

static void assert_records_equal(const struct ace2k_config_record *a,
                                 const struct ace2k_config_record *b)
{
    ASSERT_STR_EQ(a->proven_version, b->proven_version);
    ASSERT_EQ(a->flags, b->flags);
    ASSERT_EQ(a->dryer_log.cycles_started, b->dryer_log.cycles_started);
    ASSERT_EQ(a->dryer_log.cycles_completed, b->dryer_log.cycles_completed);
    ASSERT_EQ(a->dryer_log.heat_s, b->dryer_log.heat_s);
    ASSERT_EQ(a->dryer_log.full_power_s, b->dryer_log.full_power_s);
    ASSERT_EQ(a->dryer_log.faults, b->dryer_log.faults);
    ASSERT_EQ(a->dryer_log.flags, b->dryer_log.flags);
    for (uint32_t i = 0; i < ACE2K_DRYER_LOG_ENTRIES; i++) {
        const struct ace2k_dryer_log_entry *x = &a->dryer_log.entry[i];
        const struct ace2k_dryer_log_entry *y = &b->dryer_log.entry[i];
        ASSERT_EQ(x->kind, y->kind);
        ASSERT_EQ(x->reason, y->reason);
        ASSERT_EQ(x->cycle, y->cycle);
        ASSERT_EQ(x->heat_s, y->heat_s);
        ASSERT_EQ(x->ntc_left_dc, y->ntc_left_dc);
        ASSERT_EQ(x->ntc_right_dc, y->ntc_right_dc);
        ASSERT_EQ(x->chamber_dc, y->chamber_dc);
    }
}

TEST(layout_2_is_240_bytes_and_round_trips_the_log)
{
    struct ace2k_config_record in = sample_record();
    struct ace2k_config_record out;
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    ASSERT_EQ(ACE2K_CONFIG_RECORD_LEN, 240);
    ASSERT_EQ(ACE2K_CONFIG_LAYOUT, 2);
    ace2k_config_encode(&in, raw);
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_FLASH);
    assert_records_equal(&in, &out);
}

TEST(store_then_load_returns_the_log_in_60_words)
{
    struct ace2k_config c;
    struct ace2k_config again;
    fake_reset(&ace2k_flash);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    c.record = sample_record();
    ASSERT_EQ(ace2k_config_store(&c), 0);
    ASSERT_EQ(ace2k_flash.programs, 60);
    ace2k_config_init(&again, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(again.source, ACE2K_CONFIG_FROM_FLASH);
    assert_records_equal(&c.record, &again.record);
}

/* A page the v0.4.0 image wrote: 48 bytes, layout 1, CRC-16/CCITT over bytes 0–45 at 46. */
static void write_layout1_page(const char *version, uint32_t flags)
{
    uint8_t *p = ace2k_flash.page;
    for (uint32_t i = 0; i < 48U; i++) {
        p[i] = 0;
    }
    ace2k_put_u32_le(p + 0, ACE2K_CONFIG_MAGIC);
    ace2k_put_u16_le(p + 4, 1U);
    ace2k_str_copy((char *)(p + 8), ACE2K_CONFIG_VERSION_LEN, version);
    ace2k_put_u32_le(p + 40, flags);
    ace2k_put_u16_le(p + 46, ace2k_crc16_ccitt(p, 46U));
}

TEST(a_layout_1_page_is_migrated_with_its_version_and_flags_and_an_empty_log)
{
    struct ace2k_config c;
    struct ace2k_config again;
    fake_reset(&ace2k_flash);
    write_layout1_page("0.4.0", ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_FLASH);
    ASSERT_STR_EQ(c.record.proven_version, "0.4.0");
    ASSERT_EQ(c.record.flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ASSERT_EQ(c.record.dryer_log.cycles_started, 0);
    ASSERT_EQ(c.record.dryer_log.flags, 0);
    ASSERT_EQ(c.record.dryer_log.entry[0].kind, 0);
    ASSERT_EQ(ace2k_config_store(&c), 0);
    ASSERT_EQ(ace2k_get_u16_le(ace2k_flash.page + 4), 2);
    ace2k_config_init(&again, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(again.source, ACE2K_CONFIG_FROM_FLASH);
    ASSERT_STR_EQ(again.record.proven_version, "0.4.0");
}

TEST(a_layout_1_page_with_a_bad_crc_is_corrupt)
{
    struct ace2k_config c;
    fake_reset(&ace2k_flash);
    write_layout1_page("0.4.0", ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ace2k_flash.page[10] ^= 0x01U;
    ace2k_config_init(&c, &ace2k_fake_ops, &ace2k_flash);
    ASSERT_EQ(c.source, ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
    ASSERT_STR_EQ(c.record.proven_version, "");
}

TEST(any_flipped_byte_of_a_layout_2_page_is_corrupt)
{
    struct ace2k_config_record in = sample_record();
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    for (uint32_t i = 0; i < ACE2K_CONFIG_RECORD_LEN; i++) {
        struct ace2k_config_record out;
        ace2k_config_encode(&in, raw);
        raw[i] ^= 0x10U;
        ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
        ASSERT_EQ(out.dryer_log.cycles_started, 0);
    }
}

TEST(an_unknown_layout_is_corrupt)
{
    struct ace2k_config_record in = sample_record();
    struct ace2k_config_record out;
    uint8_t raw[ACE2K_CONFIG_RECORD_LEN];
    ace2k_config_encode(&in, raw);
    ace2k_put_u16_le(raw + 4, 3U);
    ace2k_put_u16_le(raw + ACE2K_CONFIG_RECORD_LEN - 2U,
                     ace2k_crc16_ccitt(raw, ACE2K_CONFIG_RECORD_LEN - 2U));
    ASSERT_EQ(ace2k_config_decode(raw, &out), ACE2K_CONFIG_FROM_DEFAULTS_CORRUPT);
}

/* The two factory pages as RAM, read-only: a read outside them is a violation, any erase or
 * program on them is a violation (the ops must never be pointed at them for a write). */
struct ace2k_fake_factory {
    uint8_t primary[ACE2K_GUARD_FACTORY_PAGE_SIZE];
    uint8_t backup[ACE2K_GUARD_FACTORY_PAGE_SIZE];
    int violations;
    int writes;
};

static uint8_t *factory_page_of(struct ace2k_fake_factory *f, uint32_t addr, uint32_t len)
{
    if (addr >= ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR &&
        addr + len <= ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR + ACE2K_GUARD_FACTORY_PAGE_SIZE) {
        return f->primary + (addr - ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR);
    }
    if (addr >= ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR &&
        addr + len <= ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR + ACE2K_GUARD_FACTORY_PAGE_SIZE) {
        return f->backup + (addr - ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR);
    }
    return 0;
}

static void factory_read(void *ctx, uint32_t addr, uint8_t *out, uint32_t len)
{
    struct ace2k_fake_factory *f = ctx;
    const uint8_t *p = factory_page_of(f, addr, len);
    if (!p) {
        f->violations++;
        return;
    }
    for (uint32_t i = 0; i < len; i++) {
        out[i] = p[i];
    }
}

static int factory_write_refused(void *ctx, uint32_t addr)
{
    struct ace2k_fake_factory *f = ctx;
    (void)addr;
    f->writes++;
    return -ACE2K_EREFUSED;
}

static int factory_program_refused(void *ctx, uint32_t addr, uint32_t word)
{
    (void)word;
    return factory_write_refused(ctx, addr);
}

static const struct ace2k_config_ops ace2k_factory_ops = {
    .flash_read = factory_read,
    .flash_erase_page = factory_write_refused,
    .flash_program_word = factory_program_refused,
};

/* Builds a page: zeros, the four pairs at their offsets, the CRC over the 1064-byte record at
 * 0x7FC.  Synthetic values — never a real unit's. */
static void factory_page_build(uint8_t page[ACE2K_GUARD_FACTORY_PAGE_SIZE],
                               const uint16_t pairs[ACE2K_LANE_COUNT][2])
{
    for (size_t i = 0; i < ACE2K_GUARD_FACTORY_PAGE_SIZE; i++) {
        page[i] = i < ACE2K_CONFIG_FACTORY_RECORD_LEN ? 0 : 0xFF;
    }
    for (size_t s = 0; s < ACE2K_LANE_COUNT; s++) {
        uint8_t *rec = page + (s * ACE2K_CONFIG_FACTORY_PAIR_STRIDE);
        rec[ACE2K_CONFIG_FACTORY_PAIR_OFF_A] = (uint8_t)pairs[s][0];
        rec[ACE2K_CONFIG_FACTORY_PAIR_OFF_A + 1] = (uint8_t)(pairs[s][0] >> 8U);
        rec[ACE2K_CONFIG_FACTORY_PAIR_OFF_B] = (uint8_t)pairs[s][1];
        rec[ACE2K_CONFIG_FACTORY_PAIR_OFF_B + 1] = (uint8_t)(pairs[s][1] >> 8U);
    }
    uint16_t crc = ace2k_crc16_mcrf4xx(page, ACE2K_CONFIG_FACTORY_RECORD_LEN);
    page[ACE2K_CONFIG_FACTORY_CRC_OFF] = (uint8_t)crc;
    page[ACE2K_CONFIG_FACTORY_CRC_OFF + 1] = (uint8_t)(crc >> 8U);
}

static const uint16_t ace2k_pairs_primary[ACE2K_LANE_COUNT][2] = {
    { 610, 1190 },
    { 1205, 650 },
    { 590, 1210 },
    { 700, 1300 },
};
static const uint16_t ace2k_pairs_backup[ACE2K_LANE_COUNT][2] = {
    { 500, 1000 },
    { 510, 1010 },
    { 520, 1020 },
    { 530, 1030 },
};

TEST(factory_primary_page_yields_the_bands_low_first)
{
    static struct ace2k_fake_factory f;
    struct ace2k_factory_calibration cal;
    f.violations = 0;
    f.writes = 0;
    factory_page_build(f.primary, ace2k_pairs_primary);
    factory_page_build(f.backup, ace2k_pairs_backup);
    ASSERT_EQ(ace2k_config_factory_read(&ace2k_factory_ops, &f, &cal), 1);
    ASSERT_EQ(cal.source, ACE2K_CAL_FACTORY);
    ASSERT_EQ(cal.insert[0].low_mv, 610);
    ASSERT_EQ(cal.insert[0].high_mv, 1190);
    ASSERT_EQ(cal.insert[1].low_mv, 650); /* stored high-first */
    ASSERT_EQ(cal.insert[1].high_mv, 1205);
    ASSERT_EQ(cal.insert[3].high_mv, 1300);
    ASSERT_EQ(cal.insert[2].source, ACE2K_CAL_FACTORY);
    ASSERT_EQ(f.violations, 0);
    ASSERT_EQ(f.writes, 0);
}

TEST(factory_backup_page_is_used_when_the_primary_is_corrupt)
{
    static struct ace2k_fake_factory f;
    struct ace2k_factory_calibration cal;
    f.violations = 0;
    factory_page_build(f.primary, ace2k_pairs_primary);
    factory_page_build(f.backup, ace2k_pairs_backup);
    f.primary[100] ^= 0x01U;
    ASSERT_EQ(ace2k_config_factory_read(&ace2k_factory_ops, &f, &cal), 2);
    ASSERT_EQ(cal.insert[0].low_mv, 500);
    ASSERT_EQ(cal.insert[3].high_mv, 1030);
    ASSERT_EQ(f.violations, 0);
}

TEST(factory_defaults_when_both_pages_fail_and_a_blank_page_is_corrupt)
{
    static struct ace2k_fake_factory f;
    struct ace2k_factory_calibration cal;
    for (size_t i = 0; i < ACE2K_GUARD_FACTORY_PAGE_SIZE; i++) {
        f.primary[i] = 0xFF;
    }
    factory_page_build(f.backup, ace2k_pairs_backup);
    f.backup[ACE2K_CONFIG_FACTORY_CRC_OFF] ^= 0xFFU;
    ASSERT_EQ(ace2k_config_factory_read(&ace2k_factory_ops, &f, &cal), 0);
    ASSERT_EQ(cal.source, ACE2K_CAL_DEFAULT);
    for (size_t s = 0; s < ACE2K_LANE_COUNT; s++) {
        ASSERT_EQ(cal.insert[s].low_mv, ACE2K_CONFIG_INSERT_DEFAULT_LOW_MV);
        ASSERT_EQ(cal.insert[s].high_mv, ACE2K_CONFIG_INSERT_DEFAULT_HIGH_MV);
        ASSERT_EQ(cal.insert[s].source, ACE2K_CAL_DEFAULT);
    }
}

TEST(factory_degenerate_pair_falls_back_to_the_defaults_for_that_lane_only)
{
    static struct ace2k_fake_factory f;
    struct ace2k_factory_calibration cal;
    const uint16_t pairs[ACE2K_LANE_COUNT][2] = {
        { 610, 1190 },
        { 700, 799 },
        { 700, 800 },
        { 0, 0 },
    };
    factory_page_build(f.primary, pairs);
    factory_page_build(f.backup, ace2k_pairs_backup);
    ASSERT_EQ(ace2k_config_factory_read(&ace2k_factory_ops, &f, &cal), 1);
    ASSERT_EQ(cal.insert[0].source, ACE2K_CAL_FACTORY);
    ASSERT_EQ(cal.insert[1].source, ACE2K_CAL_DEFAULT); /* 99 mV apart: degenerate */
    ASSERT_EQ(cal.insert[1].low_mv, 600);
    ASSERT_EQ(cal.insert[2].source, ACE2K_CAL_FACTORY); /* 100 mV apart: kept */
    ASSERT_EQ(cal.insert[2].low_mv, 700);
    ASSERT_EQ(cal.insert[3].source, ACE2K_CAL_DEFAULT);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
