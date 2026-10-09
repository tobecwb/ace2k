/* What: the RC522-class readers at register level — address framing, a soft reset
 * and the version register.  Reader A serves lanes 3–4, reader B lanes 1–2 (docs/hardware.md
 * "RFID readers").  Register map: NXP MFRC522 datasheet (public); the parts on the board are
 * clones that report version 0x18.
 * How: the binding owns one struct ace2k_rfid_reader over SPI, NSS, RST and a delay op; every
 * function runs in task context (SPI transfers and the reset's waits block).  The soft reset
 * is polled every millisecond — the part clears PowerDown well inside one, so a healthy probe
 * costs one or two polls — and given up after 150 ms, the public MFRC522 library's 3 × 50 ms
 * budget kept as the bound.
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
#define ACE2K_RFID_VERSION_EXPECTED 0x18U /* measured on both readers, 2026-09-13 */
#define ACE2K_RFID_RESET_POLL_US    1000U /* fine-grained: PowerDown clears well inside 1 ms */
#define ACE2K_RFID_RESET_POLLS      150U  /* × 1 ms = the library's 3 × 50 ms budget, as a bound */
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
    /* Soft resets since init, per reader: a reset clears the registers a transceive needs, so
     * rfid/transceive re-applies them when this moved (the health probe resets a reader). */
    uint16_t reset_count[ACE2K_RFID_READER_COUNT];
};

/* Both readers released from reset (RST high) and deselected. */
void ace2k_rfid_reader_init(struct ace2k_rfid_reader *self, const struct ace2k_rfid_reader_ops *ops,
                            void *ctx);

uint8_t ace2k_rfid_reader_read_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg);
void ace2k_rfid_reader_write_reg(struct ace2k_rfid_reader *self, uint8_t reader, uint8_t reg,
                                 uint8_t value);

/* 0 once CommandReg's power-down bit clears; -ACE2K_EIO after ACE2K_RFID_RESET_POLLS. */
int ace2k_rfid_reader_soft_reset(struct ace2k_rfid_reader *self, uint8_t reader);

/* Soft reset, then the version register into version[reader]; 0 when it is the expected
 * value, -ACE2K_EIO otherwise (0x00 / 0xFF = a mute bus).  -ACE2K_EINVAL for reader ≥ 2. */
int ace2k_rfid_reader_probe(struct ace2k_rfid_reader *self, uint8_t reader);

#endif
