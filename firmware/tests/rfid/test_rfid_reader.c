#include "test.h"
#include "rfid/reader.h"

/* Two scripted readers: a register file each; a transfer decodes the address byte.  NSS must be
 * selected for exactly the transfer; RST low is counted (rule 2: must stay zero). */
struct ace2k_fake_bus {
    uint8_t regs[ACE2K_RFID_READER_COUNT][64];
    bool selected[ACE2K_RFID_READER_COUNT];
    int rst_low_calls;
    int xfers_unselected;
    int polls_until_reset_done; /* reads of CommandReg during a reset before the bit clears */
    bool mute;                  /* the bus answers 0xFF */
    uint8_t last_out[2];
    uint32_t delayed_us;
};

static void fake_nss(void *ctx, uint8_t reader, bool selected)
{
    ((struct ace2k_fake_bus *)ctx)->selected[reader] = selected;
}

static void fake_rst(void *ctx, uint8_t reader, bool running)
{
    (void)reader;
    if (!running) {
        ((struct ace2k_fake_bus *)ctx)->rst_low_calls++;
    }
}

/* The selected reader, or -1 when neither is. */
static int selected_reader(const struct ace2k_fake_bus *f)
{
    if (f->selected[0]) {
        return 0;
    }
    if (f->selected[1]) {
        return 1;
    }
    return -1;
}

static void fake_xfer(void *ctx, uint8_t *buf, uint8_t len)
{
    struct ace2k_fake_bus *f = ctx;
    int reader = selected_reader(f);
    f->last_out[0] = buf[0];
    f->last_out[1] = buf[1];
    if (reader < 0) {
        f->xfers_unselected++;
        return;
    }
    uint8_t reg = (uint8_t)((buf[0] & ACE2K_RFID_ADDR_MASK) >> 1U);
    if (buf[0] & ACE2K_RFID_ADDR_READ) {
        if (f->mute) {
            buf[1] = 0xFF;
            return;
        }
        /* the power-down bit clears on the scripted poll; the countdown runs only while a
         * reset is in progress */
        if (reg == ACE2K_RFID_REG_COMMAND && (f->regs[reader][reg] & ACE2K_RFID_CMD_POWER_DOWN) &&
            f->polls_until_reset_done-- <= 1) {
            f->regs[reader][reg] &= (uint8_t)~ACE2K_RFID_CMD_POWER_DOWN;
        }
        buf[1] = f->regs[reader][reg];
        return;
    }
    (void)len;
    f->regs[reader][reg] = buf[1];
    if (reg == ACE2K_RFID_REG_COMMAND && buf[1] == ACE2K_RFID_CMD_SOFT_RESET) {
        f->regs[reader][reg] = ACE2K_RFID_CMD_POWER_DOWN; /* the reset runs */
    }
}

static void fake_delay(void *ctx, uint32_t us)
{
    ((struct ace2k_fake_bus *)ctx)->delayed_us += us;
}

static const struct ace2k_rfid_reader_ops ace2k_fake_ops = {
    .nss = fake_nss,
    .rst = fake_rst,
    .spi_xfer = fake_xfer,
    .delay_us = fake_delay,
};

static void fake_init(struct ace2k_fake_bus *f, uint8_t version_a, uint8_t version_b)
{
    *f = (struct ace2k_fake_bus){ 0 };
    f->regs[0][ACE2K_RFID_REG_VERSION] = version_a;
    f->regs[1][ACE2K_RFID_REG_VERSION] = version_b;
    f->polls_until_reset_done = 2;
}

TEST(address_framing_and_chip_select)
{
    struct ace2k_fake_bus f;
    struct ace2k_rfid_reader r;
    fake_init(&f, 0x18, 0x18);
    ace2k_rfid_reader_init(&r, &ace2k_fake_ops, &f);
    ASSERT_TRUE(!f.selected[0] && !f.selected[1]);
    ASSERT_EQ(ace2k_rfid_reader_read_reg(&r, ACE2K_RFID_READER_A, ACE2K_RFID_REG_VERSION), 0x18);
    ASSERT_EQ(f.last_out[0], 0xEE);
    ASSERT_EQ(f.last_out[1], 0x00);
    ASSERT_TRUE(!f.selected[0]);
    ace2k_rfid_reader_write_reg(&r, ACE2K_RFID_READER_B, ACE2K_RFID_REG_COMMAND, 0x0F);
    ASSERT_EQ(f.last_out[0], 0x02);
    ASSERT_EQ(f.last_out[1], 0x0F);
    ASSERT_EQ(f.xfers_unselected, 0);
    ASSERT_EQ(f.rst_low_calls, 0);
}

TEST(probe_resets_polls_and_reads_the_version)
{
    struct ace2k_fake_bus f;
    struct ace2k_rfid_reader r;
    fake_init(&f, 0x18, 0x92);
    ace2k_rfid_reader_init(&r, &ace2k_fake_ops, &f);
    ASSERT_EQ(ace2k_rfid_reader_probe(&r, ACE2K_RFID_READER_A), 0);
    ASSERT_EQ(r.version[0], 0x18);
    ASSERT_TRUE(r.version_ok[0]);
    ASSERT_EQ(f.delayed_us, 2 * 1000); /* two 1 ms polls, not two 50 ms ones */
    ASSERT_EQ(ace2k_rfid_reader_probe(&r, ACE2K_RFID_READER_B), -ACE2K_EIO); /* a genuine part */
    ASSERT_EQ(r.version[1], 0x92);
    ASSERT_TRUE(!r.version_ok[1]);
    ASSERT_EQ(ace2k_rfid_reader_probe(&r, 2), -ACE2K_EINVAL);
    ASSERT_EQ(f.rst_low_calls, 0);
}

TEST(a_mute_bus_and_a_reset_that_never_completes_fail)
{
    struct ace2k_fake_bus f;
    struct ace2k_rfid_reader r;
    fake_init(&f, 0x18, 0x18);
    f.mute = true;
    ace2k_rfid_reader_init(&r, &ace2k_fake_ops, &f);
    ASSERT_EQ(ace2k_rfid_reader_probe(&r, ACE2K_RFID_READER_A), -ACE2K_EIO);
    ASSERT_EQ(f.delayed_us, 150 * 1000); /* the 150 ms cap: 150 polls of 1 ms */
    ASSERT_TRUE(!r.version_ok[0]);
    fake_init(&f, 0x18, 0x18);
    f.polls_until_reset_done = (int)ACE2K_RFID_RESET_POLLS + 1; /* one poll past the cap */
    ace2k_rfid_reader_init(&r, &ace2k_fake_ops, &f);
    ASSERT_EQ(ace2k_rfid_reader_soft_reset(&r, ACE2K_RFID_READER_B), -ACE2K_EIO);
    ASSERT_EQ(f.rst_low_calls, 0);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
