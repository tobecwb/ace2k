/* What: the one reader instance of the image.  It drives both RC522-class readers on SPI2, with NSS
 * and RST as plain outputs.  RST is held high, because a reader is reset through CommandReg, never
 * through its RST pin.
 * How: ace2k_rfid_reader_binding() hands the instance to another binding.  The health self-test and
 * the query command call ace2k_rfid_reader_binding_probe(), which leaves alone a reader that the
 * watch uses.  Every function transfers on SPI, so all of them run in task context only.  This file
 * is compiled only with CONFIG_ACE2K_RFID, and callers guard their calls with the same flag.
 * Depends on: rfid/reader.h, ace2k_board/pins.h, rfid/watch_cmds.h (with CONFIG_ACE2K_RFID_READ),
 * Klipper's gpio, spi, command and udelay. */
#ifndef ACE2K_RFID_READER_CMDS_H
#define ACE2K_RFID_READER_CMDS_H
#include "rfid/reader.h"

/* Returns the one reader instance of the image, read-only.  It is set up on first use. */
const struct ace2k_rfid_reader *ace2k_rfid_reader_binding(void);

/* Returns the same instance, writable, for the binding that drives the readers beyond the probe.
 * That is the watch's transceive, with CONFIG_ACE2K_RFID_READ.  Its soft resets count in
 * reset_count. */
struct ace2k_rfid_reader *ace2k_rfid_reader_binding_mut(void);

/* Soft-resets one reader and reads its version again.  It returns the result of
 * ace2k_rfid_reader_probe().  With CONFIG_ACE2K_RFID_READ, a reader that the watch uses is not
 * reset.  The watch uses a reader while it wants the field on, or while a job runs on it.  Then the
 * call returns -ACE2K_EBUSY, and version[reader] keeps the value of the last soft reset. */
int ace2k_rfid_reader_binding_probe(uint8_t reader);

#endif
