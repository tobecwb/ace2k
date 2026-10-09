#include "rfid/reader.h"

static uint8_t addr_byte(uint8_t reg, bool read)
{
    uint8_t a = (uint8_t)(((uint32_t)reg << 1U) & ACE2K_RFID_ADDR_MASK);
    return read ? (uint8_t)(a | ACE2K_RFID_ADDR_READ) : a;
}

void ace2k_rfid_reader_init(struct ace2k_rfid_reader *self, const struct ace2k_rfid_reader_ops *ops,
                            void *ctx)
{
    *self = (struct ace2k_rfid_reader){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        ops->nss(ctx, r, false);
        ops->rst(ctx, r, true);
    }
}

uint8_t ace2k_rfid_reader_read_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg)
{
    uint8_t buf[2] = { addr_byte(reg, true), 0x00 };
    self->ops->nss(self->ctx, reader, true);
    self->ops->spi_xfer(self->ctx, buf, 2U);
    self->ops->nss(self->ctx, reader, false);
    return buf[1];
}

void ace2k_rfid_reader_write_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg,
                                 uint8_t value)
{
    uint8_t buf[2] = { addr_byte(reg, false), value };
    self->ops->nss(self->ctx, reader, true);
    self->ops->spi_xfer(self->ctx, buf, 2U);
    self->ops->nss(self->ctx, reader, false);
}

int ace2k_rfid_reader_soft_reset(struct ace2k_rfid_reader *self, uint8_t reader)
{
    self->reset_count[reader]++;
    ace2k_rfid_reader_write_reg(self, reader, ACE2K_RFID_REG_COMMAND, ACE2K_RFID_CMD_SOFT_RESET);
    for (uint8_t poll = 0; poll < ACE2K_RFID_RESET_POLLS; poll++) {
        self->ops->delay_us(self->ctx, ACE2K_RFID_RESET_POLL_US);
        if (!(ace2k_rfid_reader_read_reg(self, reader, ACE2K_RFID_REG_COMMAND) &
              ACE2K_RFID_CMD_POWER_DOWN)) {
            return 0;
        }
    }
    return -ACE2K_EIO;
}

int ace2k_rfid_reader_probe(struct ace2k_rfid_reader *self, uint8_t reader)
{
    if (reader >= ACE2K_RFID_READER_COUNT) {
        return -ACE2K_EINVAL;
    }
    self->version_ok[reader] = false;
    if (ace2k_rfid_reader_soft_reset(self, reader) != 0) {
        self->version[reader] = 0;
        return -ACE2K_EIO;
    }
    uint8_t v = ace2k_rfid_reader_read_reg(self, reader, ACE2K_RFID_REG_VERSION);
    self->version[reader] = v;
    self->version_ok[reader] = v == ACE2K_RFID_VERSION_EXPECTED;
    return self->version_ok[reader] ? 0 : -ACE2K_EIO;
}
