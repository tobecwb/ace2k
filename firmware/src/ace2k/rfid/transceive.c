#include "rfid/transceive.h"

void ace2k_rfid_transceive_init(struct ace2k_rfid_transceive *self,
                                struct ace2k_rfid_reader *reader)
{
    *self = (struct ace2k_rfid_transceive){ .reader = reader };
}

static void wr(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t reg, uint8_t v)
{
    ace2k_rfid_reader_write_reg(self->reader, reader, reg, v);
}

static uint8_t rd(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t reg)
{
    return ace2k_rfid_reader_read_reg(self->reader, reader, reg);
}

int ace2k_rfid_transceive_configure(struct ace2k_rfid_transceive *self, uint8_t reader)
{
    if (reader >= ACE2K_RFID_READER_COUNT) {
        return -ACE2K_EINVAL;
    }
    int rc = ace2k_rfid_reader_probe(self->reader, reader); /* the soft reset and the version */
    if (rc != 0) {
        self->configured[reader] = false;
        return rc;
    }
    wr(self, reader, ACE2K_RFID_REG_T_MODE, ACE2K_RFID_T_MODE_AUTO);
    wr(self, reader, ACE2K_RFID_REG_T_PRESCALER, ACE2K_RFID_T_PRESCALER);
    wr(self, reader, ACE2K_RFID_REG_T_RELOAD_H, ACE2K_RFID_T_RELOAD_H);
    wr(self, reader, ACE2K_RFID_REG_T_RELOAD_L, ACE2K_RFID_T_RELOAD_L);
    wr(self, reader, ACE2K_RFID_REG_TX_ASK, ACE2K_RFID_TX_ASK_100);
    wr(self, reader, ACE2K_RFID_REG_MODE, ACE2K_RFID_MODE_CRC_6363);
    ace2k_rfid_transceive_field_set(self, reader, false);
    self->cmd[reader] = ACE2K_RFID_CMD_IDLE;
    self->configured[reader] = true;
    self->configured_at[reader] = self->reader->reset_count[reader];
    return 0;
}

bool ace2k_rfid_transceive_needs_configure(const struct ace2k_rfid_transceive *self, uint8_t reader)
{
    if (reader >= ACE2K_RFID_READER_COUNT || !self->configured[reader]) {
        return true;
    }
    return self->configured_at[reader] != self->reader->reset_count[reader];
}

void ace2k_rfid_transceive_field_set(struct ace2k_rfid_transceive *self, uint8_t reader, bool on)
{
    uint32_t v = rd(self, reader, ACE2K_RFID_REG_TX_CONTROL);
    v = on ? (v | ACE2K_RFID_ANTENNA) : (v & ~(uint32_t)ACE2K_RFID_ANTENNA);
    wr(self, reader, ACE2K_RFID_REG_TX_CONTROL, (uint8_t)v);
}

bool ace2k_rfid_transceive_field_is_on(struct ace2k_rfid_transceive *self, uint8_t reader)
{
    uint32_t v = rd(self, reader, ACE2K_RFID_REG_TX_CONTROL);
    return (v & ACE2K_RFID_ANTENNA) == ACE2K_RFID_ANTENNA;
}

bool ace2k_rfid_transceive_field_driven(struct ace2k_rfid_transceive *self, uint8_t reader)
{
    uint32_t v = rd(self, reader, ACE2K_RFID_REG_TX_CONTROL);
    return (v & ACE2K_RFID_ANTENNA) != 0;
}

static bool start_args_ok(uint8_t reader, uint8_t cmd, uint8_t len, uint8_t last_bits)
{
    if (reader >= ACE2K_RFID_READER_COUNT || len == 0 || len > ACE2K_RFID_TX_MAX ||
        last_bits > ACE2K_RFID_LAST_BITS) {
        return false;
    }
    if (cmd == ACE2K_RFID_CMD_TRANSCEIVE || cmd == ACE2K_RFID_CMD_MF_AUTHENT) {
        return true;
    }
    return false;
}

int ace2k_rfid_transceive_start(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t cmd,
                                const uint8_t *tx, uint8_t len, uint8_t last_bits)
{
    if (!start_args_ok(reader, cmd, len, last_bits)) {
        return -ACE2K_EINVAL;
    }
    wr(self, reader, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_IDLE);
    wr(self, reader, ACE2K_RFID_REG_COM_IRQ, ACE2K_RFID_COM_IRQ_CLEAR);
    wr(self, reader, ACE2K_RFID_REG_FIFO_LEVEL, ACE2K_RFID_FIFO_FLUSH);
    for (uint8_t i = 0; i < len; i++) {
        wr(self, reader, ACE2K_RFID_REG_FIFO_DATA, tx[i]);
    }
    uint8_t framing = (uint8_t)(((uint32_t)last_bits << ACE2K_RFID_RX_ALIGN_SHIFT) | last_bits);
    wr(self, reader, ACE2K_RFID_REG_BIT_FRAMING, framing);
    wr(self, reader, ACE2K_RFID_REG_COMMAND, cmd);
    if (cmd == ACE2K_RFID_CMD_TRANSCEIVE) {
        wr(self, reader, ACE2K_RFID_REG_BIT_FRAMING,
           (uint8_t)((uint32_t)framing | ACE2K_RFID_START_SEND));
    }
    self->cmd[reader] = cmd;
    return 0;
}

/* The bits of ComIrqReg that end each command: a transceive ends on a reception (or idle), an
 * authentication on idle. */
static uint32_t done_mask(uint8_t cmd)
{
    if (cmd == ACE2K_RFID_CMD_MF_AUTHENT) {
        return ACE2K_RFID_IRQ_IDLE;
    }
    return (uint32_t)ACE2K_RFID_IRQ_RX | ACE2K_RFID_IRQ_IDLE;
}

static enum ace2k_rfid_poll read_fifo(struct ace2k_rfid_transceive *self, uint8_t reader,
                                      struct ace2k_rfid_rx *rx)
{
    uint8_t n =
        (uint8_t)((uint32_t)rd(self, reader, ACE2K_RFID_REG_FIFO_LEVEL) & ACE2K_RFID_FIFO_LEVEL);
    if (n > ACE2K_RFID_RX_MAX) {
        return ACE2K_RFID_POLL_ERROR;
    }
    for (uint8_t i = 0; i < n; i++) {
        rx->data[i] = rd(self, reader, ACE2K_RFID_REG_FIFO_DATA);
    }
    rx->len = n;
    rx->last_bits =
        (uint8_t)((uint32_t)rd(self, reader, ACE2K_RFID_REG_CONTROL) & ACE2K_RFID_LAST_BITS);
    return ACE2K_RFID_POLL_DONE;
}

static enum ace2k_rfid_poll collision(struct ace2k_rfid_transceive *self, uint8_t reader,
                                      struct ace2k_rfid_rx *rx)
{
    (void)read_fifo(self, reader, rx); /* the bits before the collision are the tag's */
    uint32_t coll = rd(self, reader, ACE2K_RFID_REG_COLL);
    if (coll & ACE2K_RFID_COLL_POS_NOT_VALID) {
        return ACE2K_RFID_POLL_ERROR;
    }
    uint8_t pos = (uint8_t)(coll & ACE2K_RFID_COLL_POS);
    rx->coll_pos = pos == 0 ? (uint8_t)ACE2K_RFID_COLL_POS_ZERO : pos;
    return ACE2K_RFID_POLL_COLLISION;
}

static enum ace2k_rfid_poll finish(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t cmd,
                                   struct ace2k_rfid_rx *rx)
{
    uint32_t err = rd(self, reader, ACE2K_RFID_REG_ERROR);
    if (err & ACE2K_RFID_ERR_COLL) {
        return collision(self, reader, rx);
    }
    if (err &
        ((uint32_t)ACE2K_RFID_ERR_PROTOCOL | ACE2K_RFID_ERR_PARITY | ACE2K_RFID_ERR_OVERFLOW)) {
        return ACE2K_RFID_POLL_ERROR;
    }
    if (cmd == ACE2K_RFID_CMD_MF_AUTHENT) {
        uint32_t s2 = rd(self, reader, ACE2K_RFID_REG_STATUS2);
        return (s2 & ACE2K_RFID_STATUS2_CRYPTO1_ON) ? ACE2K_RFID_POLL_DONE
                                                    : ACE2K_RFID_POLL_AUTH_FAILED;
    }
    return read_fifo(self, reader, rx);
}

enum ace2k_rfid_poll ace2k_rfid_transceive_poll(struct ace2k_rfid_transceive *self, uint8_t reader,
                                                struct ace2k_rfid_rx *rx)
{
    if (reader >= ACE2K_RFID_READER_COUNT || self->cmd[reader] == ACE2K_RFID_CMD_IDLE) {
        return ACE2K_RFID_POLL_ERROR;
    }
    uint8_t cmd = self->cmd[reader];
    uint32_t irq = rd(self, reader, ACE2K_RFID_REG_COM_IRQ);
    *rx = (struct ace2k_rfid_rx){ 0 };
    if (irq & (done_mask(cmd) | ACE2K_RFID_IRQ_ERR)) {
        self->cmd[reader] = ACE2K_RFID_CMD_IDLE;
        return finish(self, reader, cmd, rx);
    }
    if (irq & ACE2K_RFID_IRQ_TIMER) {
        self->cmd[reader] = ACE2K_RFID_CMD_IDLE;
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    return ACE2K_RFID_POLL_BUSY;
}

void ace2k_rfid_transceive_crypto_off(struct ace2k_rfid_transceive *self, uint8_t reader)
{
    uint32_t v = rd(self, reader, ACE2K_RFID_REG_STATUS2);
    wr(self, reader, ACE2K_RFID_REG_STATUS2,
       (uint8_t)(v & ~(uint32_t)ACE2K_RFID_STATUS2_CRYPTO1_ON));
}
