/* What: the field of the RC522 and its exchanges with a tag.  This is the half of a tag read that
 * happens at the reader.  It covers these parts.
 * - The configuration that a soft reset clears: the timer that bounds the answer of a tag, the
 *   100 % ASK modulation, and the CRC preset.
 * - The field on and off, with its read-back.
 * - One exchange, Transceive or MFAuthent, started and then polled.
 * - The MIFARE crypto unit, cleared.
 * How: it uses the register access of rfid/reader.h.  Every call runs in task context, and no call
 * waits for the tag.  ace2k_rfid_transceive_start() loads the FIFO and starts the command.  Later,
 * the task calls ace2k_rfid_transceive_poll(), which reads the interrupt register.  The exchange
 * ends when the tag answered, when the timer of the reader expired, or when the reader flagged an
 * error.  The timer is 25 ms, the value that the public MFRC522 library uses.  This file writes
 * only the reader commands Idle, Transceive and MFAuthent.  SoftReset is written through
 * rfid/reader.  rfid/iso14443a computes the CRC_A of the tag in software, so CalcCRC is never used.
 * The register map and the bits come from the public NXP MFRC522 datasheet.
 * Depends on: <stdbool.h>, <stdint.h>, rfid/reader.h, util.h. */
#ifndef ACE2K_RFID_TRANSCEIVE_H
#define ACE2K_RFID_TRANSCEIVE_H
#include <stdbool.h>
#include <stdint.h>
#include "rfid/reader.h"
#include "core/util.h"

#define ACE2K_RFID_REG_COM_IRQ     0x04U
#define ACE2K_RFID_REG_ERROR       0x06U
#define ACE2K_RFID_REG_STATUS2     0x08U
#define ACE2K_RFID_REG_FIFO_DATA   0x09U
#define ACE2K_RFID_REG_FIFO_LEVEL  0x0AU
#define ACE2K_RFID_REG_CONTROL     0x0CU
#define ACE2K_RFID_REG_BIT_FRAMING 0x0DU
#define ACE2K_RFID_REG_COLL        0x0EU
#define ACE2K_RFID_REG_MODE        0x11U
#define ACE2K_RFID_REG_TX_CONTROL  0x14U
#define ACE2K_RFID_REG_TX_ASK      0x15U
#define ACE2K_RFID_REG_T_MODE      0x2AU
#define ACE2K_RFID_REG_T_PRESCALER 0x2BU
#define ACE2K_RFID_REG_T_RELOAD_H  0x2CU
#define ACE2K_RFID_REG_T_RELOAD_L  0x2DU

#define ACE2K_RFID_CMD_IDLE       0x00U
#define ACE2K_RFID_CMD_TRANSCEIVE 0x0CU
#define ACE2K_RFID_CMD_MF_AUTHENT 0x0EU

#define ACE2K_RFID_COM_IRQ_CLEAR      0x7FU /* Set1 = 0: clear every request bit */
#define ACE2K_RFID_IRQ_TIMER          0x01U
#define ACE2K_RFID_IRQ_ERR            0x02U
#define ACE2K_RFID_IRQ_IDLE           0x10U
#define ACE2K_RFID_IRQ_RX             0x20U
#define ACE2K_RFID_FIFO_FLUSH         0x80U
#define ACE2K_RFID_FIFO_LEVEL         0x7FU
#define ACE2K_RFID_START_SEND         0x80U
#define ACE2K_RFID_RX_ALIGN_SHIFT     4U
#define ACE2K_RFID_LAST_BITS          0x07U
#define ACE2K_RFID_ERR_PROTOCOL       0x01U
#define ACE2K_RFID_ERR_PARITY         0x02U
#define ACE2K_RFID_ERR_COLL           0x08U
#define ACE2K_RFID_ERR_OVERFLOW       0x10U
#define ACE2K_RFID_STATUS2_CRYPTO1_ON 0x08U
#define ACE2K_RFID_COLL_POS           0x1FU
#define ACE2K_RFID_COLL_POS_NOT_VALID 0x20U /* a collision outside the bits CollPos can name */
#define ACE2K_RFID_COLL_POS_ZERO      32U   /* CollPos 0 is the 32nd bit */
#define ACE2K_RFID_ANTENNA            0x03U /* TxControlReg Tx1RFEn | Tx2RFEn */
/* The configuration, as the public MFRC522 library applies it after a reset.  The timer starts at
 * the end of a transmission (TAuto).  It runs at 13.56 MHz / (2 × TPrescaler + 1) = 13.56 MHz / 339
 * ≈ 40 kHz, which is 25 µs per count.  So a reload of 1000 gives a tag 25 ms to answer.  The
 * modulation is 100 % ASK.  The CRC preset is 0x6363, as ISO 14443-A sets it. */
#define ACE2K_RFID_T_MODE_AUTO   0x80U
#define ACE2K_RFID_T_PRESCALER   0xA9U
#define ACE2K_RFID_T_RELOAD_H    0x03U
#define ACE2K_RFID_T_RELOAD_L    0xE8U
#define ACE2K_RFID_TX_ASK_100    0x40U
#define ACE2K_RFID_MODE_CRC_6363 0x3DU
/* The answer to a READ, 16 data bytes and the CRC_A, is the longest frame that a tag sends here.
 * An authentication request, 12 bytes, is the longest frame that the firmware sends, apart from a
 * SELECT of 9 bytes. */
#define ACE2K_RFID_RX_MAX 18U
#define ACE2K_RFID_TX_MAX 18U

enum ace2k_rfid_poll {
    ACE2K_RFID_POLL_BUSY = 0,
    ACE2K_RFID_POLL_DONE = 1,
    ACE2K_RFID_POLL_TIMEOUT = 2, /* no answer within the reader's timer */
    ACE2K_RFID_POLL_ERROR = 3,   /* protocol, parity or buffer error; the exchange is over */
    ACE2K_RFID_POLL_COLLISION = 4,
    ACE2K_RFID_POLL_AUTH_FAILED = 5, /* MFAuthent ended without MFCrypto1On */
};

struct ace2k_rfid_rx {
    uint8_t data[ACE2K_RFID_RX_MAX];
    uint8_t len;       /* bytes in data, counting a last partial byte */
    uint8_t last_bits; /* the valid bits of the last byte; 0 means all eight */
    uint8_t coll_pos;  /* on ACE2K_RFID_POLL_COLLISION, the first collided bit, 1..32 */
};

struct ace2k_rfid_transceive {
    struct ace2k_rfid_reader *reader;
    uint8_t cmd[ACE2K_RFID_READER_COUNT]; /* the exchange that runs; Idle when none */
    bool configured[ACE2K_RFID_READER_COUNT];
    uint16_t configured_at[ACE2K_RFID_READER_COUNT]; /* reader->reset_count when configured */
};

void ace2k_rfid_transceive_init(struct ace2k_rfid_transceive *self,
                                struct ace2k_rfid_reader *reader);

/* Soft-resets one reader, which takes up to 150 ms (rfid/reader.h), then writes the configuration,
 * with the field off.  It returns 0, or the -ACE2K_EIO of the probe.  It returns -ACE2K_EINVAL for
 * a reader out of range. */
int ace2k_rfid_transceive_configure(struct ace2k_rfid_transceive *self, uint8_t reader);

/* Returns true until configure() ran.  It returns true again after any later soft reset, such as
 * the one of the health probe. */
bool ace2k_rfid_transceive_needs_configure(const struct ace2k_rfid_transceive *self,
                                           uint8_t reader);

/* The field calls and crypto_off go through the register access of rfid/reader.  For a reader out
 * of range they write nothing, and the read-backs say off. */
void ace2k_rfid_transceive_field_set(struct ace2k_rfid_transceive *self, uint8_t reader, bool on);
/* Reads the register back.  True when both antenna drivers are enabled.  For the tests. */
bool ace2k_rfid_transceive_field_is_on(struct ace2k_rfid_transceive *self, uint8_t reader);
/* Reads the register back.  True when either antenna driver is enabled.  After a switch-off, one
 * driver still on is a field, although TxControlReg & 3 reads 0. */
bool ace2k_rfid_transceive_field_driven(struct ace2k_rfid_transceive *self, uint8_t reader);

/* Starts one exchange.  cmd is ACE2K_RFID_CMD_TRANSCEIVE or ACE2K_RFID_CMD_MF_AUTHENT.  tx[len] is
 * the frame, with len 1..ACE2K_RFID_TX_MAX.  last_bits is 0..7, the valid bits of the last byte
 * sent, where 0 means eight.  It is also the position of the first received bit (RxAlign), as ISO
 * 14443-A anticollision wants.  A running exchange is abandoned first, with Idle.  Any other input
 * returns -ACE2K_EINVAL, and nothing is written. */
int ace2k_rfid_transceive_start(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t cmd,
                                const uint8_t *tx, uint8_t len, uint8_t last_bits);

/* Returns the state of the running exchange.  Anything but BUSY ends the exchange. */
enum ace2k_rfid_poll ace2k_rfid_transceive_poll(struct ace2k_rfid_transceive *self, uint8_t reader,
                                                struct ace2k_rfid_rx *rx);

/* Clears MFCrypto1On.  This ends every authenticated session. */
void ace2k_rfid_transceive_crypto_off(struct ace2k_rfid_transceive *self, uint8_t reader);

#endif
