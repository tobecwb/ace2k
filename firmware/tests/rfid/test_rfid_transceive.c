#include "test.h"
#include "rfid/transceive.h"

/* One RC522 register file per reader, with the FIFO as two queues: what the firmware wrote into
 * FIFODataReg (fifo_in) and what the scripted tag answered (fifo_out, read back through
 * FIFODataReg and counted by FIFOLevelReg).  Every register write is logged in order. */
#define LOG_MAX 64
struct ace2k_fake_rc522 {
    uint8_t regs[ACE2K_RFID_READER_COUNT][64];
    bool selected[ACE2K_RFID_READER_COUNT];
    uint8_t fifo_in[64];
    uint8_t fifo_in_len;
    uint8_t fifo_out[64];
    uint8_t fifo_out_len, fifo_out_pos;
    uint8_t log_reg[LOG_MAX], log_val[LOG_MAX];
    int log_len;
};

static void fake_nss(void *ctx, uint8_t reader, bool selected)
{
    ((struct ace2k_fake_rc522 *)ctx)->selected[reader] = selected;
}

static void fake_rst(void *ctx, uint8_t reader, bool running)
{
    (void)ctx;
    (void)reader;
    (void)running;
}

static int sel(const struct ace2k_fake_rc522 *f)
{
    if (f->selected[0]) {
        return 0;
    }
    return f->selected[1] ? 1 : -1;
}

static void fake_write(struct ace2k_fake_rc522 *f, int r, uint8_t reg, uint8_t v)
{
    if (f->log_len < LOG_MAX) {
        f->log_reg[f->log_len] = reg;
        f->log_val[f->log_len] = v;
        f->log_len++;
    }
    if (reg == ACE2K_RFID_REG_FIFO_DATA) {
        f->fifo_in[f->fifo_in_len++] = v;
        return;
    }
    if (reg == ACE2K_RFID_REG_FIFO_LEVEL && (v & ACE2K_RFID_FIFO_FLUSH)) {
        f->fifo_in_len = 0;
        return;
    }
    if (reg == ACE2K_RFID_REG_COM_IRQ && v == ACE2K_RFID_COM_IRQ_CLEAR) {
        f->regs[r][reg] = 0;
        return;
    }
    f->regs[r][reg] = v;
    if (reg == ACE2K_RFID_REG_COMMAND && v == ACE2K_RFID_CMD_SOFT_RESET) {
        f->regs[r][reg] = 0;                          /* the reset completes at once in the fake */
        f->regs[r][ACE2K_RFID_REG_TX_CONTROL] = 0x80; /* the datasheet's reset value */
    }
}

static uint8_t fake_read(struct ace2k_fake_rc522 *f, int r, uint8_t reg)
{
    if (reg == ACE2K_RFID_REG_FIFO_DATA) {
        return f->fifo_out_pos < f->fifo_out_len ? f->fifo_out[f->fifo_out_pos++] : 0;
    }
    if (reg == ACE2K_RFID_REG_FIFO_LEVEL) {
        return (uint8_t)(f->fifo_out_len - f->fifo_out_pos);
    }
    return f->regs[r][reg];
}

static void fake_xfer(void *ctx, uint8_t *buf, uint8_t len)
{
    struct ace2k_fake_rc522 *f = ctx;
    int r = sel(f);
    (void)len;
    if (r < 0) {
        return;
    }
    uint8_t reg = (uint8_t)((buf[0] & ACE2K_RFID_ADDR_MASK) >> 1U);
    if (buf[0] & ACE2K_RFID_ADDR_READ) {
        buf[1] = fake_read(f, r, reg);
    } else {
        fake_write(f, r, reg, buf[1]);
    }
}

static void fake_delay(void *ctx, uint32_t us)
{
    (void)ctx;
    (void)us;
}

static const struct ace2k_rfid_reader_ops ace2k_fake_ops = {
    .nss = fake_nss,
    .rst = fake_rst,
    .spi_xfer = fake_xfer,
    .delay_us = fake_delay,
};

// NOLINTNEXTLINE(readability-identifier-naming)
struct rig {
    struct ace2k_fake_rc522 f;
    struct ace2k_rfid_reader r;
    struct ace2k_rfid_transceive t;
};

static void rig_init(struct rig *g)
{
    *g = (struct rig){ 0 };
    g->f.regs[0][ACE2K_RFID_REG_VERSION] = 0x18;
    g->f.regs[1][ACE2K_RFID_REG_VERSION] = 0x18;
    ace2k_rfid_reader_init(&g->r, &ace2k_fake_ops, &g->f);
    ace2k_rfid_transceive_init(&g->t, &g->r);
}

/* The index in the write log of the first write of value v to reg at or after `from`; -1. */
static int logged(const struct ace2k_fake_rc522 *f, int from, uint8_t reg, uint8_t v)
{
    for (int i = from; i < f->log_len; i++) {
        if (f->log_reg[i] == reg && f->log_val[i] == v) {
            return i;
        }
    }
    return -1;
}

TEST(configure_resets_and_writes_the_timer_modulation_and_crc_preset_field_off)
{
    struct rig g;
    rig_init(&g);
    ASSERT_TRUE(ace2k_rfid_transceive_needs_configure(&g.t, ACE2K_RFID_READER_B));
    ASSERT_EQ(ace2k_rfid_transceive_configure(&g.t, ACE2K_RFID_READER_B), 0);
    ASSERT_TRUE(logged(&g.f, 0, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_SOFT_RESET) >= 0);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_T_MODE], 0x80);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_T_PRESCALER], 0xA9);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_T_RELOAD_H], 0x03);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_T_RELOAD_L], 0xE8);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_TX_ASK], 0x40);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_MODE], 0x3D);
    ASSERT_TRUE(!ace2k_rfid_transceive_field_is_on(&g.t, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_transceive_needs_configure(&g.t, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ace2k_rfid_transceive_needs_configure(&g.t, ACE2K_RFID_READER_A)); /* the other */
    /* the health probe's soft reset clears the registers: configure again */
    ASSERT_EQ(ace2k_rfid_reader_probe(&g.r, ACE2K_RFID_READER_B), 0);
    ASSERT_TRUE(ace2k_rfid_transceive_needs_configure(&g.t, ACE2K_RFID_READER_B));
    ASSERT_EQ(ace2k_rfid_transceive_configure(&g.t, 2), -ACE2K_EINVAL);
}

TEST(the_field_is_bits_zero_and_one_of_tx_control_read_modify_write_and_read_back)
{
    struct rig g;
    rig_init(&g);
    ASSERT_EQ(ace2k_rfid_transceive_configure(&g.t, ACE2K_RFID_READER_A), 0);
    ace2k_rfid_transceive_field_set(&g.t, ACE2K_RFID_READER_A, true);
    ASSERT_EQ(g.f.regs[0][ACE2K_RFID_REG_TX_CONTROL], 0x83); /* the reset value's bit 7 kept */
    ASSERT_TRUE(ace2k_rfid_transceive_field_is_on(&g.t, ACE2K_RFID_READER_A));
    ASSERT_TRUE(!ace2k_rfid_transceive_field_is_on(&g.t, ACE2K_RFID_READER_B));
    ace2k_rfid_transceive_field_set(&g.t, ACE2K_RFID_READER_A, false);
    ASSERT_EQ(g.f.regs[0][ACE2K_RFID_REG_TX_CONTROL], 0x80);
    ASSERT_TRUE(!ace2k_rfid_transceive_field_is_on(&g.t, ACE2K_RFID_READER_A));
    ASSERT_TRUE(!ace2k_rfid_transceive_field_driven(&g.t, ACE2K_RFID_READER_A));
    /* one bit of the two is not a field on, but after a switch-off a single stuck driver is
     * still a field: the switch-off's read-back wants both clear */
    g.f.regs[0][ACE2K_RFID_REG_TX_CONTROL] = 0x81;
    ASSERT_TRUE(!ace2k_rfid_transceive_field_is_on(&g.t, ACE2K_RFID_READER_A));
    ASSERT_TRUE(ace2k_rfid_transceive_field_driven(&g.t, ACE2K_RFID_READER_A));
    g.f.regs[0][ACE2K_RFID_REG_TX_CONTROL] = 0x82;
    ASSERT_TRUE(ace2k_rfid_transceive_field_driven(&g.t, ACE2K_RFID_READER_A));
}

TEST(start_writes_idle_clear_flush_fifo_framing_command_then_start_send)
{
    struct rig g;
    rig_init(&g);
    const uint8_t wupa[1] = { 0x52 };
    g.f.log_len = 0;
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, ACE2K_RFID_READER_A, ACE2K_RFID_CMD_TRANSCEIVE,
                                          wupa, 1, 7),
              0);
    int i0 = logged(&g.f, 0, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_IDLE);
    int i1 = logged(&g.f, i0, ACE2K_RFID_REG_COM_IRQ, ACE2K_RFID_COM_IRQ_CLEAR);
    int i2 = logged(&g.f, i1, ACE2K_RFID_REG_FIFO_LEVEL, ACE2K_RFID_FIFO_FLUSH);
    int i3 = logged(&g.f, i2, ACE2K_RFID_REG_FIFO_DATA, 0x52);
    int i4 = logged(&g.f, i3, ACE2K_RFID_REG_BIT_FRAMING, 0x77);
    int i5 = logged(&g.f, i4, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_TRANSCEIVE);
    int i6 = logged(&g.f, i5, ACE2K_RFID_REG_BIT_FRAMING, 0xF7);
    ASSERT_TRUE(i0 >= 0 && i1 > i0 && i2 > i1 && i3 > i2 && i4 > i3 && i5 > i4 && i6 > i5);
    ASSERT_EQ(g.f.fifo_in_len, 1);
    /* MFAuthent: no start-send bit */
    const uint8_t auth[12] = { 0x60, 4, 1, 2, 3, 4, 5, 6, 0xA, 0xB, 0xC, 0xD };
    g.f.log_len = 0;
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, ACE2K_RFID_READER_A, ACE2K_RFID_CMD_MF_AUTHENT,
                                          auth, 12, 0),
              0);
    ASSERT_TRUE(logged(&g.f, 0, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_MF_AUTHENT) >= 0);
    ASSERT_EQ(logged(&g.f, 0, ACE2K_RFID_REG_BIT_FRAMING, 0x80), -1);
    ASSERT_EQ(g.f.fifo_in_len, 12);
}

TEST(start_refuses_any_other_command_an_empty_or_long_frame_and_bad_framing_writing_nothing)
{
    struct rig g;
    rig_init(&g);
    const uint8_t frame[19] = { 0x30, 4 };
    g.f.log_len = 0;
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_SOFT_RESET, frame, 2, 0),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, 0x03 /* CalcCRC */, frame, 2, 0), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, 0x01 /* Mem */, frame, 2, 0), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, frame, 0, 0),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, frame, 19, 0),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, frame, 2, 8),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 2, ACE2K_RFID_CMD_TRANSCEIVE, frame, 2, 0),
              -ACE2K_EINVAL);
    ASSERT_EQ(g.f.log_len, 0);
}

TEST(poll_busy_timeout_done_collision_and_errors)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_rfid_rx rx;
    const uint8_t reqa[1] = { 0x26 };
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_ERROR); /* nothing runs */
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_BUSY);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_TIMER;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_TIMEOUT);
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_ERROR); /* it ended */
    /* an ATQA: two bytes, all bits valid */
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_IDLE;
    g.f.fifo_out[0] = 0x44;
    g.f.fifo_out[1] = 0x00;
    g.f.fifo_out_len = 2;
    g.f.fifo_out_pos = 0;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_DONE);
    ASSERT_EQ(rx.len, 2);
    ASSERT_EQ(rx.data[0], 0x44);
    ASSERT_EQ(rx.last_bits, 0);
    /* a collision at bit 5, and CollPos 0 read as bit 32 */
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_ERR;
    g.f.regs[0][ACE2K_RFID_REG_ERROR] = ACE2K_RFID_ERR_COLL;
    g.f.regs[0][ACE2K_RFID_REG_COLL] = 0x05;
    g.f.fifo_out_len = 1;
    g.f.fifo_out_pos = 0;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_COLLISION);
    ASSERT_EQ(rx.coll_pos, 5);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_ERR;
    g.f.regs[0][ACE2K_RFID_REG_COLL] = 0x00;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_COLLISION);
    ASSERT_EQ(rx.coll_pos, 32);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_ERR;
    g.f.regs[0][ACE2K_RFID_REG_COLL] = ACE2K_RFID_COLL_POS_NOT_VALID;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_ERROR);
    /* a parity error, then a FIFO holding more than a READ's 18 bytes */
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_ERR;
    g.f.regs[0][ACE2K_RFID_REG_ERROR] = ACE2K_RFID_ERR_PARITY;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_ERROR);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 0, ACE2K_RFID_CMD_TRANSCEIVE, reqa, 1, 7), 0);
    g.f.regs[0][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_IDLE;
    g.f.regs[0][ACE2K_RFID_REG_ERROR] = 0;
    g.f.fifo_out_len = 19;
    g.f.fifo_out_pos = 0;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 0, &rx), ACE2K_RFID_POLL_ERROR);
}

TEST(authentication_is_done_only_with_crypto_on_and_crypto_off_clears_it)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_rfid_rx rx;
    const uint8_t auth[12] = { 0x60, 4, 1, 2, 3, 4, 5, 6, 0xA, 0xB, 0xC, 0xD };
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 1, ACE2K_RFID_CMD_MF_AUTHENT, auth, 12, 0), 0);
    g.f.regs[1][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_IDLE;
    g.f.regs[1][ACE2K_RFID_REG_STATUS2] = 0x00;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 1, &rx), ACE2K_RFID_POLL_AUTH_FAILED);
    ASSERT_EQ(ace2k_rfid_transceive_start(&g.t, 1, ACE2K_RFID_CMD_MF_AUTHENT, auth, 12, 0), 0);
    g.f.regs[1][ACE2K_RFID_REG_COM_IRQ] = ACE2K_RFID_IRQ_IDLE;
    g.f.regs[1][ACE2K_RFID_REG_STATUS2] = 0x01U | ACE2K_RFID_STATUS2_CRYPTO1_ON;
    ASSERT_EQ(ace2k_rfid_transceive_poll(&g.t, 1, &rx), ACE2K_RFID_POLL_DONE);
    ace2k_rfid_transceive_crypto_off(&g.t, 1);
    ASSERT_EQ(g.f.regs[1][ACE2K_RFID_REG_STATUS2], 0x01); /* the other bits kept */
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
