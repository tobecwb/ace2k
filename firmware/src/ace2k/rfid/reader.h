/* What: the RC522-class readers at register level.  This covers the address framing, a soft reset
 * and the version register.  Reader A serves lanes 3–4 and reader B serves lanes 1–2
 * (docs/hardware.md "RFID readers").  The register map comes from the public NXP MFRC522 datasheet.
 * The parts on the board are clones that report version 0x18.
 * How: the binding owns one struct ace2k_rfid_reader over SPI, NSS, RST and a delay op.  Every
 * function runs in task context, because the SPI transfers and the waits of the reset block.  The
 * soft reset is polled every millisecond.  The part clears PowerDown well inside one millisecond,
 * so a healthy probe costs one or two polls.  The reset is given up after 150 ms.  That bound is
 * the budget of the public MFRC522 library, 3 × 50 ms.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_RFID_READER_H
#define ACE2K_RFID_READER_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_RFID_READER_COUNT 2U
#define ACE2K_RFID_READER_A     0U /* NSS PB12, RST PD13 — lanes 3–4 */
#define ACE2K_RFID_READER_B     1U /* NSS PD10, RST PD12 — lanes 1–2 */

#define ACE2K_RFID_REG_COMMAND      0x01U
#define ACE2K_RFID_REG_VERSION      0x37U
#define ACE2K_RFID_CMD_SOFT_RESET   0x0FU
#define ACE2K_RFID_CMD_POWER_DOWN   0x10U /* CommandReg bit 4, set while the reset runs */
#define ACE2K_RFID_VERSION_EXPECTED 0x18U /* measured on both readers (docs/hardware.md) */
#define ACE2K_RFID_RESET_POLL_US    1000U /* short, because PowerDown clears well inside 1 ms */
#define ACE2K_RFID_RESET_POLLS      150U  /* × 1 ms, the 3 × 50 ms budget of the library */
#define ACE2K_RFID_ADDR_READ        0x80U
#define ACE2K_RFID_ADDR_MASK        0x7EU

struct ace2k_rfid_reader_ops {
    void (*nss)(void *ctx, uint8_t reader, bool selected);
    void (*rst)(void *ctx, uint8_t reader, bool running); /* only ever passed true */
    void (*spi_xfer)(void *ctx, uint8_t *buf, uint8_t len);
    void (*delay_us)(void *ctx, uint32_t us);
};

struct ace2k_rfid_reader {
    const struct ace2k_rfid_reader_ops *ops;
    void *ctx;
    uint8_t version[ACE2K_RFID_READER_COUNT];
    bool version_ok[ACE2K_RFID_READER_COUNT];
    /* The soft resets of each reader since init.  A reset clears the registers that a
     * transceive needs.  So rfid/transceive applies them again when this count moved.  The health
     * probe resets a reader, for example. */
    uint16_t reset_count[ACE2K_RFID_READER_COUNT];
};

/* Releases both readers from reset (RST high) and deselects them. */
void ace2k_rfid_reader_init(struct ace2k_rfid_reader *self, const struct ace2k_rfid_reader_ops *ops,
                            void *ctx);

/* A reader out of range touches no line.  read_reg returns 0, and write_reg writes nothing. */
uint8_t ace2k_rfid_reader_read_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg);
void ace2k_rfid_reader_write_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg,
                                 uint8_t value);

/* Soft-resets one reader.  It returns 0 once the power-down bit of CommandReg clears.  It returns
 * -ACE2K_EIO after ACE2K_RFID_RESET_POLLS polls, and -ACE2K_EINVAL for a reader out of range. */
int ace2k_rfid_reader_soft_reset(struct ace2k_rfid_reader *self, uint8_t reader);

/* Soft-resets one reader, then reads its version register into version[reader].  It returns 0 when
 * the version is the expected value, and -ACE2K_EIO otherwise.  0x00 or 0xFF means a mute bus.  It
 * returns -ACE2K_EINVAL for reader ≥ 2. */
int ace2k_rfid_reader_probe(struct ace2k_rfid_reader *self, uint8_t reader);

#endif
