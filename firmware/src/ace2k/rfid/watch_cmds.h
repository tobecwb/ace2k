/* What: the one watch instance of the image.  It holds the transceive of the two readers, the
 * engine of the tag protocol, and the watch over the tags of the four lanes.
 * How: every 10 ms tick wakes a task.  The task runs one watch step, then the tie-break's scheduler
 * (rfid/watch_tiebreak.h, with CONFIG_ACE2K_FEED), and sends the events and the periodic state.
 * Every SPI transfer happens in that task, never in the tick.  The commands of a session, a read on
 * command, a forget, the host's mask of the lanes in use, a lane set by hand and the query of the
 * marks, and the settings of the confirmation are handled in the same main loop.  The mask of the
 * lanes set by hand is sent after each by-hand command and whenever it changes.
 * The feed binding reads the bits that the watch publishes for each lane: the session, the read
 * tag, the lost tag, and the confirmation asked of the lane's load with its direction.  The task
 * packs them in one word and writes it whole, so the tick never sees a torn set.  The tick reads
 * them with ace2k_rfid_binding_feed_bits.  The health binding asks whether a reader is in use
 * before it probes it.  It also asks whether the field of a reader faulted, or the reader is dead.
 * A health run gives a dead reader another try.
 * This file is compiled only with CONFIG_ACE2K_RFID_READ, and callers guard their calls with the
 * same flag.
 * Depends on: rfid/watch.h, rfid/watch_tiebreak.h, rfid/transceive.h, rfid/iso14443a.h,
 * rfid/reader_cmds.h, sensors_cmds.h, lane_cmds.h, feed_cmds.h (with CONFIG_ACE2K_FEED),
 * ace2k_board/serial.h, ace2k_board/tick.h, report.h, ace2k_board/tx.h, Klipper's sched and
 * command. */
#ifndef ACE2K_RFID_WATCH_CMDS_H
#define ACE2K_RFID_WATCH_CMDS_H
#include <stdbool.h>
#include <stdint.h>

/* Returns the last published bits, one bit per lane.  hold: a session holds the lane.  read: its
 * tag is read.  lost: its last session lost its tag.  confirm: the watch asks the lane's load to
 * confirm a candidate tag.  confirm_rev: the leg asked goes toward the spool.  confirm_epoch: the
 * epoch of the lane's last confirmation, which turns over at each new one.  It may be called from
 * any context.  It makes one load of the word that the task publishes whole, and unpacks it into
 * the six. */
void ace2k_rfid_binding_feed_bits(uint8_t *hold, uint8_t *read, uint8_t *lost, uint8_t *confirm,
                                  uint8_t *confirm_rev, uint8_t *confirm_epoch);

/* Puts the host's mask of the lanes in use, one bit per lane, as the task last published it, and
 * returns true when that mask is valid.  It is not valid, and the mask is 0, with none set since
 * the link came up, or one older than ACE2K_RFID_IN_USE_STALE_MS.  It may be called from any
 * context: one load of the published word. */
bool ace2k_rfid_binding_in_use(uint8_t *mask);

/* Returns the lanes the unit holds set by hand, one bit per lane, as the task last published them.
 * It may be called from any context: one load of the published byte. */
uint8_t ace2k_rfid_binding_by_hand(void);

/* Returns true when the watch has the reader's field on, or a job on it.  A probe would then reset
 * the reader in the middle of its use. */
bool ace2k_rfid_binding_reader_busy(uint8_t reader);

/* Returns true when the reader's field read back on after it was switched off, which latches.  It
 * also returns true when the reader is dead after ACE2K_RFID_CONFIGURE_TRIES failed configures. */
bool ace2k_rfid_binding_field_fault(uint8_t reader);

/* The health run calls this.  A dead reader is tried again.  The latched field fault stays. */
void ace2k_rfid_binding_clear_dead(void);

#endif
