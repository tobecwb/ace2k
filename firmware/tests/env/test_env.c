#include "test.h"
#include "env/env.h"
#include "env/env_table.h"
#include "core/util.h"

/* Recorded on the unit 2026-09-12 (docs/hardware.md "Chamber sensor"): 23.5 °C, 67.1 % RH. */
static const uint8_t ace2k_frame_ok[7] = { 0x18, 0xAB, 0xA6, 0xF5, 0xE0, 0xE1, 0x9F };

/* A scripted I²C bus: what the sensor answers to each read, whether writes are acked. */
struct ace2k_fake_i2c {
    uint8_t status; /* first byte of every read */
    const uint8_t *frame;
    int nack_reads; /* fail the next N reads */
    int writes, reads;
    uint8_t last_write[3];
};

static int fake_write(void *ctx, uint8_t addr, const uint8_t *data, uint8_t len)
{
    struct ace2k_fake_i2c *f = ctx;
    f->writes++;
    if (addr != ACE2K_ENV_AHT_ADDR) {
        return -1;
    }
    for (uint8_t i = 0; i < len && i < 3; i++) {
        f->last_write[i] = data[i];
    }
    if (data[0] == 0xBE) {
        f->status |= ACE2K_ENV_AHT_STATUS_CAL;
    }
    return 0;
}

static int fake_read(void *ctx, uint8_t addr, uint8_t *data, uint8_t len)
{
    struct ace2k_fake_i2c *f = ctx;
    f->reads++;
    if (addr != ACE2K_ENV_AHT_ADDR || f->nack_reads > 0) {
        f->nack_reads--;
        return -1;
    }
    data[0] = f->status;
    for (uint8_t i = 1; i < len; i++) {
        data[i] = f->frame[i];
    }
    return 0;
}

static const struct ace2k_env_ops ace2k_fake_ops = {
    .i2c_write = fake_write,
    .i2c_read = fake_read,
};

/* An enum constant is a signed int in C, and the lint refuses one under a bitwise operator. */
static bool has_valid(const struct ace2k_env *e, unsigned bit)
{
    return (e->valid & bit) != 0;
}

TEST(ntc_table_known_points_and_window)
{
    bool valid = false;
    ASSERT_EQ(ace2k_env_ntc_mc(1650, &valid), 25000);
    ASSERT_TRUE(valid);
    ASSERT_EQ(ace2k_env_ntc_mc(1000, &valid), 45002);
    ASSERT_EQ(ace2k_env_ntc_mc(100, &valid), 130627);
    ASSERT_EQ(ace2k_env_ntc_mc(3200, &valid), -36823);
    ASSERT_TRUE(valid);
    ASSERT_EQ(ace2k_env_ntc_mc(99, &valid), 0);
    ASSERT_TRUE(!valid);
    ASSERT_EQ(ace2k_env_ntc_mc(3201, &valid), 0);
    ASSERT_TRUE(!valid);
    /* between two entries: linear, and the curve only falls */
    int32_t a = ace2k_env_ntc_mc(1650, &valid);
    int32_t m = ace2k_env_ntc_mc(1662, &valid);
    int32_t b = ace2k_env_ntc_mc(1675, &valid);
    ASSERT_TRUE(a > m && m > b);
    for (size_t i = 1; i < ACE2K_ENV_NTC_TABLE_LEN; i++) {
        ASSERT_TRUE(ace2k_env_ntc_table_mc[i] < ace2k_env_ntc_table_mc[i - 1]);
    }
}

TEST(aht_frame_decodes_and_refuses_bad_crc_and_busy)
{
    int32_t mc = 0;
    uint16_t rh = 0;
    ASSERT_EQ(ace2k_env_crc8(ace2k_frame_ok, 6), 0x9F);
    ASSERT_TRUE(ace2k_env_aht_decode(ace2k_frame_ok, &mc, &rh));
    ASSERT_EQ(mc, 23480);
    ASSERT_EQ(rh, 670);
    uint8_t bad[7];
    for (int i = 0; i < 7; i++) {
        bad[i] = ace2k_frame_ok[i];
    }
    bad[6] ^= 0x01U;
    ASSERT_TRUE(!ace2k_env_aht_decode(bad, &mc, &rh));
    bad[6] = ace2k_frame_ok[6];
    bad[0] |= ACE2K_ENV_AHT_STATUS_BUSY;
    ASSERT_TRUE(!ace2k_env_aht_decode(bad, &mc, &rh));
}

TEST(aht_state_machine_initialises_triggers_and_reads_once_a_second)
{
    struct ace2k_env e;
    struct ace2k_fake_i2c f = {
        .status = 0x18U & (uint8_t)~ACE2K_ENV_AHT_STATUS_CAL,
        .frame = ace2k_frame_ok,
    };
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 1000);
    ace2k_env_task(&e, 1000); /* status uncalibrated → BE 08 00 */
    ASSERT_EQ(f.last_write[0], 0xBE);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_INIT_SENT);
    ace2k_env_task(&e, 1005);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_INIT_SENT); /* not yet 10 ms */
    ace2k_env_task(&e, 1010);
    ASSERT_EQ(f.last_write[0], 0xAC);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_TRIGGERED);
    ace2k_env_task(&e, 1050);
    ASSERT_TRUE(!e.chamber_ever); /* not yet 80 ms */
    ace2k_env_task(&e, 1090);
    ASSERT_TRUE(e.chamber_ever);
    ASSERT_EQ(e.chamber_mc, 23480);
    ASSERT_EQ(e.chamber_rh_pct10, 670);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_IDLE);
    int reads_before = f.reads;
    ace2k_env_task(&e, 1500);
    ASSERT_EQ(f.reads, reads_before); /* the next cycle starts ~1 s after the trigger */
    ace2k_env_task(&e, 2010);
    ASSERT_EQ(f.reads, reads_before + 1);            /* status read of the next cycle */
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_TRIGGERED); /* calibrated now: straight to trigger */
    ace2k_env_tick(&e, 2010, 1650, 1650, 1489);
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_CHAMBER));
    ASSERT_TRUE(ace2k_env_chamber_plausible(&e));
}

TEST(aht_failure_counts_keeps_the_value_and_goes_stale_after_two_seconds)
{
    struct ace2k_env e;
    struct ace2k_fake_i2c f = { .status = 0x18, .frame = ace2k_frame_ok };
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 0);
    ace2k_env_task(&e, 0);
    ace2k_env_task(&e, 80);
    ASSERT_TRUE(e.chamber_ever);
    ace2k_env_tick(&e, 80, 1650, 1650, 1489);
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_CHAMBER));
    f.nack_reads = 10;
    ace2k_env_task(&e, 1000); /* status read NACKs */
    ASSERT_EQ(e.aht_failures, 1);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_IDLE);
    ASSERT_EQ(e.chamber_mc, 23480);
    ace2k_env_tick(&e, 2080, 1650, 1650, 1489);
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_CHAMBER));
    ace2k_env_tick(&e, 2081, 1650, 1650, 1489);
    ASSERT_TRUE(!has_valid(&e, ACE2K_ENV_VALID_CHAMBER));
    ace2k_env_task(&e, 2000);
    ASSERT_EQ(e.aht_failures, 2);
}

TEST(aht_busy_is_retried_three_times_then_fails)
{
    struct ace2k_env e;
    struct ace2k_fake_i2c f = {
        .status = 0x18U | ACE2K_ENV_AHT_STATUS_BUSY,
        .frame = ace2k_frame_ok,
    };
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 0);
    ace2k_env_task(&e, 0); /* status read; the busy bit does not block the trigger */
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_TRIGGERED);
    ace2k_env_task(&e, 80);
    ace2k_env_task(&e, 90);
    ace2k_env_task(&e, 100);
    ASSERT_EQ(e.aht_failures, 0);
    ASSERT_EQ(e.aht_phase, ACE2K_ENV_AHT_TRIGGERED);
    ace2k_env_task(&e, 110);
    ASSERT_EQ(e.aht_failures, 1);
    ASSERT_TRUE(!e.chamber_ever);
}

TEST(ntc_valid_bits_and_vdda)
{
    struct ace2k_env e;
    struct ace2k_fake_i2c f = { .status = 0x18, .frame = ace2k_frame_ok };
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 0);
    ace2k_env_tick(&e, 10, 1650, 50, 1489); /* right NTC shorted */
    ASSERT_EQ(e.ptc_left_mc, 25000);
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_NTC_LEFT));
    ASSERT_TRUE(!has_valid(&e, ACE2K_ENV_VALID_NTC_RIGHT));
    ASSERT_EQ(e.vdda_mv, 3300);
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_VDDA));
    ace2k_env_tick(&e, 20, 1650, 3250, 1400); /* VDDA 3510: outside 3.3 V ± 5 % */
    ASSERT_EQ(e.vdda_mv, 3510);
    ASSERT_TRUE(!has_valid(&e, ACE2K_ENV_VALID_VDDA));
    /* right NTC open: the α = 1/8 filter settles above the window within 60 ticks */
    uint32_t now = 20;
    for (int i = 0; i < 60; i++) {
        now += 10;
        ace2k_env_tick(&e, now, 1650, 3250, 1489);
    }
    ASSERT_TRUE(!has_valid(&e, ACE2K_ENV_VALID_NTC_RIGHT));
    ASSERT_TRUE(has_valid(&e, ACE2K_ENV_VALID_VDDA));
    /* the filter: a step from 1650 to 1000 mV settles within ~40 ticks */
    for (int i = 0; i < 40; i++) {
        now += 10;
        ace2k_env_tick(&e, now, 1000, 1650, 1489);
    }
    ASSERT_TRUE(e.ptc_left_mc > 44500 && e.ptc_left_mc < 45500);
    now += 10;
    ace2k_env_tick(&e, now, 1650, 1650, 0); /* a zero reference must not divide by zero */
    ASSERT_EQ(e.vdda_mv, 0);
    now += 10;
    ace2k_env_tick(&e, now, 1650, 1650, 1); /* raw 1: 4.9 MV, saturated to the uint16_t ceiling */
    ASSERT_EQ(e.vdda_mv, 65535);
    ASSERT_TRUE(!has_valid(&e, ACE2K_ENV_VALID_VDDA));
}

TEST(the_task_is_due_only_at_the_chamber_state_machines_deadlines)
{
    /* What the binding's tick wakes the task on: false until each step's deadline, true from
     * it — never on every tick. */
    struct ace2k_env e;
    struct ace2k_fake_i2c f = {
        .status = 0x18U & (uint8_t)~ACE2K_ENV_AHT_STATUS_CAL,
        .frame = ace2k_frame_ok,
    };
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 1000);
    ASSERT_TRUE(!ace2k_env_task_due(&e, 999));
    ASSERT_TRUE(ace2k_env_task_due(&e, 1000)); /* the first cycle starts at init */
    ace2k_env_task(&e, 1000);                  /* BE 08 00: trigger after 10 ms */
    ASSERT_TRUE(!ace2k_env_task_due(&e, 1005));
    ASSERT_TRUE(!ace2k_env_task_due(&e, 1009));
    ASSERT_TRUE(ace2k_env_task_due(&e, 1010));
    int writes = f.writes;
    ace2k_env_task(&e, 1005); /* not due: nothing happens */
    ASSERT_EQ(f.writes, writes);
    ace2k_env_task(&e, 1010); /* AC 33 00: read after 80 ms */
    ASSERT_TRUE(!ace2k_env_task_due(&e, 1089));
    ASSERT_TRUE(ace2k_env_task_due(&e, 1090));
    ace2k_env_task(&e, 1090); /* the frame: the next cycle ~1 s after the trigger */
    ASSERT_TRUE(e.chamber_ever);
    ASSERT_TRUE(!ace2k_env_task_due(&e, 2009));
    ASSERT_TRUE(ace2k_env_task_due(&e, 2010));
    /* a bus failure: the next attempt one period later */
    f.nack_reads = 1;
    ace2k_env_task(&e, 2010);
    ASSERT_EQ(e.aht_failures, 1);
    ASSERT_TRUE(!ace2k_env_task_due(&e, 3009));
    ASSERT_TRUE(ace2k_env_task_due(&e, 3010));
    /* wrap-safe, as every deadline in the firmware */
    ace2k_env_init(&e, &ace2k_fake_ops, &f, 0xFFFFFFF0U);
    ace2k_env_task(&e, 0xFFFFFFF0U); /* calibrated now: AC 33 00, read 80 ms later */
    ASSERT_TRUE(!ace2k_env_task_due(&e, 0xFFFFFFFFU));
    ASSERT_TRUE(!ace2k_env_task_due(&e, 63U));
    ASSERT_TRUE(ace2k_env_task_due(&e, 64U));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
