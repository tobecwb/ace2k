/* What: the one feed instance of the image — the four lanes' motion state machines over the
 * lane instance, the sensors' states and the link.
 * How: the tick feeds the core every 10 ms after the lane's tick; a task sends the events and
 * the periodic state report.  The feed's busy verdict reaches the lane binding's counters reset
 * through the veto callback this binding registers at init — nothing is exported for it; the
 * dryer's rotation lockout will get its own accessor when it arrives.
 * Compiled only with CONFIG_ACE2K_FEED; callers guard their calls with the same flag.
 * The reader's binding (CONFIG_ACE2K_RFID_READ) gives the tick its three sets of lane bits, read
 * in one load — a session holds a lane, its tag read, its tag lost — and reads back which lanes load and whose search ran its
 * length; it starts the read with motion and sets the search's reach through the calls below.
 * Depends on: <stdint.h>, feed.h, lane_cmds.h (the instance, the tick's link_ok, the reset veto),
 * sensors_cmds.h, led_cmds.h, rfid/watch_cmds.h (with CONFIG_ACE2K_RFID_READ), ace2k_board/tick.h, Klipper's sched and command. */
#ifndef ACE2K_FEED_CMDS_H
#define ACE2K_FEED_CMDS_H
#include <stdint.h>
#include "feed/feed.h"

/* For the reader's binding (CONFIG_ACE2K_RFID_READ): which lanes are in their load, and whose
 * last load search ran its length.  Under the caller's mask. */
void ace2k_feed_binding_rfid_bits(uint8_t *loading, uint8_t *exhausted);
/* The read on command with motion, under the binding's own mask. */
enum ace2k_feed_refusal ace2k_feed_binding_search_start(uint8_t lane);
/* The search's reach, under the binding's own mask. */
void ace2k_feed_binding_search_set(uint32_t search_um);

#endif
