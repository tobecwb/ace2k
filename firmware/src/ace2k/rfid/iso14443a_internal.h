/* What: the seam between iso14443a.c and iso14443a_anticoll.c, and the phases of the engine.
 * How: plain functions on struct ace2k_iso14443a.  None of them is part of the public interface.
 * Depends on: rfid/iso14443a.h. */
#ifndef ACE2K_RFID_ISO14443A_INTERNAL_H
#define ACE2K_RFID_ISO14443A_INTERNAL_H
#include "rfid/iso14443a.h"

enum ace2k_iso_phase {
    ACE2K_ISO_PH_IDLE = 0,
    ACE2K_ISO_PH_WAKE = 1,
    ACE2K_ISO_PH_ANTICOLL = 2,
    ACE2K_ISO_PH_SELECT = 3,
    ACE2K_ISO_PH_HALT = 4,
    ACE2K_ISO_PH_AUTH = 5,
    ACE2K_ISO_PH_READ = 6,
};

/* iso14443a.c */
/* Ends the job with this result.  After a MIFARE job the crypto unit is cleared.  The job is
 * wiped. */
void ace2k_iso14443a_end(struct ace2k_iso14443a *self, uint8_t reader, uint8_t result);
/* Handles no answer.  For an inventory, it means every tag there is has been found.  For a read, it
 * means the target has left. */
void ace2k_iso14443a_on_silence(struct ace2k_iso14443a *self, uint8_t reader);
/* Checks that the last two bytes of the answer are the CRC_A of the others.  The caller has checked
 * len >= 2. */
bool ace2k_iso14443a_crc_ok(const struct ace2k_rfid_rx *rx);

/* iso14443a_anticoll.c */
int ace2k_iso14443a_send_anticoll(struct ace2k_iso14443a *self, uint8_t reader);
int ace2k_iso14443a_send_select(struct ace2k_iso14443a *self, uint8_t reader);
/* Starts a cascade level from scratch, for the anticollision of an inventory or for the SELECT of a
 * read. */
void ace2k_iso14443a_begin_level(struct ace2k_iso14443a_engine *e);
void ace2k_iso14443a_on_anticoll(struct ace2k_iso14443a *self, uint8_t reader,
                                 enum ace2k_rfid_poll p, const struct ace2k_rfid_rx *rx);
void ace2k_iso14443a_on_select(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                               const struct ace2k_rfid_rx *rx);

#endif
