/* What: the one reader instance of the image — both RC522-class readers on SPI2, NSS and RST as
 * plain outputs, RST held high (a reader is reset through
 * CommandReg, never through its RST pin).
 * How: ace2k_rfid_reader_binding() hands the instance to another binding; the health self-test
 * calls ace2k_rfid_reader_binding_probe() from its task.  Task context only: every function
 * transfers on SPI.  Compiled only with CONFIG_ACE2K_RFID; callers guard their calls with the
 * same flag.
 * Depends on: rfid/reader.h, ace2k_board/pins.h, Klipper's gpio, spi, command and udelay. */
#ifndef ACE2K_RFID_READER_CMDS_H
#define ACE2K_RFID_READER_CMDS_H
#include "rfid/reader.h"

const struct ace2k_rfid_reader *ace2k_rfid_reader_binding(void);

/* The same instance for the binding that drives the readers beyond the probe (the watch's
 * transceive, CONFIG_ACE2K_RFID_READ): its soft resets count in reset_count. */
struct ace2k_rfid_reader *ace2k_rfid_reader_binding_mut(void);

/* A fresh soft reset and version read of one reader; ace2k_rfid_reader_probe()'s result. */
int ace2k_rfid_reader_binding_probe(uint8_t reader);

#endif
