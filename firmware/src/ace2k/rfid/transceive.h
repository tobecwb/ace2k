/* What: the RC522's field and its exchanges with a tag — the reader-level half of a tag read: the
 * configuration a soft reset clears (the timer that bounds a tag's answer, the 100 % ASK
 * modulation, the CRC preset), the field on and off with its read-back, one exchange (Transceive
 * or MFAuthent) started and then polled, and the MIFARE crypto unit cleared.
 * How: over the register access of rfid/reader.h, every call in task context and none waiting on
 * the tag: ace2k_rfid_transceive_start() loads the FIFO and starts the command; the task's later
 * calls of ace2k_rfid_transceive_poll() read the interrupt register until the tag answered, the
 * reader's own timer expired (25 ms, the value the public MFRC522 library uses) or an error was
 * flagged.  Only the reader commands Idle, Transceive and MFAuthent are written here, and
 * SoftReset through rfid/reader: the tag's CRC_A is computed in
 * software by rfid/iso14443a, so CalcCRC is never used.  Register map and bits: NXP MFRC522
 * datasheet (public).
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
/* The configuration, as the public MFRC522 library applies it after a reset: the timer starts
 * at the end of a transmission (TAuto) and runs at 13.56 MHz / (2 × TPrescaler + 1) =
 * 13.56 MHz / 339 ≈ 40 kHz, 25 µs per count, so a reload of 1000 is 25 ms for a tag's answer;
 * 100 % ASK; the CRC preset 0x6363 of ISO 14443-A. */
#define ACE2K_RFID_T_MODE_AUTO   0x80U
#define ACE2K_RFID_T_PRESCALER   0xA9U
#define ACE2K_RFID_T_RELOAD_H    0x03U
#define ACE2K_RFID_T_RELOAD_L    0xE8U
#define ACE2K_RFID_TX_ASK_100    0x40U
#define ACE2K_RFID_MODE_CRC_6363 0x3DU
/* A READ's answer, 16 data bytes and the CRC_A, is the longest frame a tag sends here; an
 * authentication request, 12 bytes, the longest the firmware sends but a SELECT's 9. */
#define ACE2K_RFID_RX_MAX 18U
#define ACE2K_RFID_TX_MAX 18U

enum ace2k_rfid_poll {
    ACE2K_RFID_POLL_BUSY = 0,
    ACE2K_RFID_POLL_DONE = 1,
    ACE2K_RFID_POLL_TIMEOUT = 2, /* no answer within the reader's timer */
    ACE2K_RFID_POLL_ERROR = 3,   /* protocol, parity or buffer error; no exchange running */
    ACE2K_RFID_POLL_COLLISION = 4,
    ACE2K_RFID_POLL_AUTH_FAILED = 5, /* MFAuthent ended without MFCrypto1On */
};

struct ace2k_rfid_rx {
    uint8_t data[ACE2K_RFID_RX_MAX];
    uint8_t len;       /* bytes in data, a last partial byte counted */
    uint8_t last_bits; /* the last byte's valid bits; 0: all eight */
    uint8_t coll_pos;  /* ACE2K_RFID_POLL_COLLISION: the first collided bit, 1..32 */
};

struct ace2k_rfid_transceive {
    struct ace2k_rfid_reader *reader;
    uint8_t cmd[ACE2K_RFID_READER_COUNT]; /* the exchange running; Idle when none */
    bool configured[ACE2K_RFID_READER_COUNT];
    uint16_t configured_at[ACE2K_RFID_READER_COUNT]; /* reader->reset_count at configure */
};

void ace2k_rfid_transceive_init(struct ace2k_rfid_transceive *self,
                                struct ace2k_rfid_reader *reader);

/* A soft reset (up to 150 ms, rfid/reader.h), then the configuration; the field off.  0, or the
 * probe's -ACE2K_EIO; -ACE2K_EINVAL for a reader out of range. */
int ace2k_rfid_transceive_configure(struct ace2k_rfid_transceive *self, uint8_t reader);

/* True until configure() ran, and again after any soft reset since (the health probe's). */
bool ace2k_rfid_transceive_needs_configure(const struct ace2k_rfid_transceive *self,
                                           uint8_t reader);

void ace2k_rfid_transceive_field_set(struct ace2k_rfid_transceive *self, uint8_t reader, bool on);
/* The register read back: both antenna drivers enabled. */
bool ace2k_rfid_transceive_field_is_on(struct ace2k_rfid_transceive *self, uint8_t reader);
/* The register read back: either antenna driver enabled — after a switch-off, one driver still
 * on is a field (TxControlReg & 3 reads 0). */
bool ace2k_rfid_transceive_field_driven(struct ace2k_rfid_transceive *self, uint8_t reader);

/* Starts one exchange: cmd ACE2K_RFID_CMD_TRANSCEIVE or ACE2K_RFID_CMD_MF_AUTHENT, tx[len] with
 * len 1..ACE2K_RFID_TX_MAX, last_bits 0..7 — the valid bits of the last byte sent (0: eight), the
 * same value as the first received bit's position (RxAlign), as ISO 14443-A anticollision
 * wants.  A running exchange is abandoned (Idle first).  -ACE2K_EINVAL, nothing written,
 * otherwise. */
int ace2k_rfid_transceive_start(struct ace2k_rfid_transceive *self, uint8_t reader, uint8_t cmd,
                                const uint8_t *tx, uint8_t len, uint8_t last_bits);

/* The running exchange's state; anything but BUSY ends it. */
enum ace2k_rfid_poll ace2k_rfid_transceive_poll(struct ace2k_rfid_transceive *self, uint8_t reader,
                                                struct ace2k_rfid_rx *rx);

/* MFCrypto1On cleared: the end of every authenticated session. */
void ace2k_rfid_transceive_crypto_off(struct ace2k_rfid_transceive *self, uint8_t reader);

#endif
