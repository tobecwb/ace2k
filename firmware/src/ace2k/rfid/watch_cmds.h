/* What: the one watch instance of the image.  It holds the transceive of the two readers, the
 * engine of the tag protocol, and the watch over the tags of the four lanes.
 * How: every 10 ms tick wakes a task.  The task runs one watch step, and sends the events and the
 * periodic state.  Every SPI transfer happens in that task, never in the tick.  The commands of a
 * session, a read on command and a forget are handled in the same main loop.
 * The feed binding reads the bits that the watch publishes for each lane: the session, the read tag
 * and the lost tag.  The task packs them in one word and writes it whole, so the tick never sees a
 * torn set.  The tick reads them with ace2k_rfid_binding_feed_bits.  The health binding asks
 * whether a reader is in use before it probes it.  It also asks whether the field of a reader
 * faulted, or the reader is dead.  A health run gives a dead reader another try.
 * This file is compiled only with CONFIG_ACE2K_RFID_READ, and callers guard their calls with the
 * same flag.
 * Depends on: rfid/watch.h, rfid/transceive.h, rfid/iso14443a.h, rfid/reader_cmds.h,
 * sensors_cmds.h, lane_cmds.h, feed_cmds.h (with CONFIG_ACE2K_FEED), ace2k_board/serial.h,
 * ace2k_board/tick.h, report.h, ace2k_board/tx.h, Klipper's sched and command. */
#ifndef ACE2K_RFID_WATCH_CMDS_H
#define ACE2K_RFID_WATCH_CMDS_H
#include <stdbool.h>
#include <stdint.h>

/* Returns the last published bits, one bit per lane.  hold: a session holds the lane.  read: its
 * tag is read.  lost: its last session lost its tag.  It may be called from any context.  It makes
 * one load of the word that the task publishes whole, and unpacks it into the three. */
void ace2k_rfid_binding_feed_bits(uint8_t *hold, uint8_t *read, uint8_t *lost);

/* Returns true when the watch has the reader's field on, or a job on it.  A probe would then reset
 * the reader in the middle of its use. */
bool ace2k_rfid_binding_reader_busy(uint8_t reader);

/* Returns true when the reader's field read back on after it was switched off, which latches.  It
 * also returns true when the reader is dead after ACE2K_RFID_CONFIGURE_TRIES failed configures. */
bool ace2k_rfid_binding_field_fault(uint8_t reader);

/* The health run calls this.  A dead reader is tried again.  The latched field fault stays. */
void ace2k_rfid_binding_clear_dead(void);

#endif
