/* What: the one watch instance of the image — the two readers' transceive, the tag protocol
 * engine and the watch of the four lanes' tags.
 * How: a task woken by every 10 ms tick runs one watch step (every SPI transfer happens there,
 * never in the tick) and sends the events and the periodic state; the commands of a session, a
 * read on command and a forget are handled in the same main loop.  The feed binding reads the
 * bits a lane's session, read tag and lost tag publish — packed in one word the task writes
 * whole, so the tick never sees a torn set (ace2k_rfid_binding_feed_bits, from the tick); the
 * health binding asks whether a reader is in use before it probes it and
 * whether its field faulted or it is dead, and a health run gives a dead reader another try.
 * Compiled only with CONFIG_ACE2K_RFID_READ; callers guard their calls with the same flag.
 * Depends on: rfid/watch.h, rfid/transceive.h, rfid/iso14443a.h, rfid/reader_cmds.h,
 * sensors_cmds.h, lane_cmds.h, feed_cmds.h (with CONFIG_ACE2K_FEED), ace2k_board/serial.h,
 * ace2k_board/tick.h, report.h, tx.h, Klipper's sched and command. */
#ifndef ACE2K_RFID_WATCH_CMDS_H
#define ACE2K_RFID_WATCH_CMDS_H
#include <stdbool.h>
#include <stdint.h>

/* The last published bits, a bit per lane: a session holds the lane, its tag is read, its last
 * session lost its tag.  From any context: one load of the word the task publishes whole,
 * unpacked into the three. */
void ace2k_rfid_binding_feed_bits(uint8_t *hold, uint8_t *read, uint8_t *lost);

/* The watch has the reader's field on or a job on it: a probe would reset it mid-use. */
bool ace2k_rfid_binding_reader_busy(uint8_t reader);

/* The reader's field read back on after it was switched off (latched), or the reader dead
 * after ACE2K_RFID_CONFIGURE_TRIES failed configures. */
bool ace2k_rfid_binding_field_fault(uint8_t reader);

/* The health run: a dead reader is tried again (the latched field fault stays). */
void ace2k_rfid_binding_clear_dead(void);

#endif
