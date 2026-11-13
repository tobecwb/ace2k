/* What: the one feed instance of the image.  It runs the motion state machines of the four lanes
 * over the lane instance, the states of the sensors and the link.
 * How: every 10 ms, the tick feeds the core after the lane's tick.  A task sends the events and the
 * periodic state report.  The busy verdict of the feed reaches the lane binding's counters reset
 * through the veto callback that this binding registers at init.  Nothing is exported for it.  The
 * dryer's rotation lockout will get its own accessor when it arrives.  This file is compiled only
 * with CONFIG_ACE2K_FEED, and callers guard their calls with the same flag.
 * The reader's binding (CONFIG_ACE2K_RFID_READ) gives the tick its lane bits, read in one load (a
 * session holds a lane, its tag is read or lost, a confirmation is asked, its direction and epoch,
 * the lanes in use and whether that mask is valid), and the lanes set by hand.  It reads back
 * which lanes load, which ones had their search run its length, which ones ran a confirmation leg
 * out, and which ones are idle.  It starts the read with motion and the tie-break, and sets the
 * search's reach and the legs of the confirmation, through the calls below.
 * Depends on: <stdint.h>, feed.h, lane_cmds.h (the instance, the tick's link_ok, the reset veto),
 * sensors_cmds.h, led_cmds.h, rfid/watch_cmds.h (with CONFIG_ACE2K_RFID_READ), ace2k_board/tick.h,
 * Klipper's sched and command. */
#ifndef ACE2K_FEED_CMDS_H
#define ACE2K_FEED_CMDS_H
#include <stdint.h>
#include "feed/feed.h"

/* For the binding of the reader (CONFIG_ACE2K_RFID_READ): which lanes are in their load, which
 * ones had their last load search run its length, which ones ran a confirmation leg out, or had
 * no room for it, and the epoch of the confirmation each lane's legs serve.  Under the caller's
 * mask. */
void ace2k_feed_binding_rfid_bits(uint8_t *loading, uint8_t *exhausted, uint8_t *confirm_out,
                                  uint8_t *confirm_out_epoch);
/* Sets the legs of the confirmation (ace2k_feed_confirm_cfg_set), under the binding's own mask. */
void ace2k_feed_binding_confirm_cfg_set(uint32_t leg_max_um, uint32_t speed_um_s);
/* The read on command with motion, under the binding's own mask.  seq is the read's retry key. */
enum ace2k_feed_refusal ace2k_feed_binding_search_start(uint8_t lane, uint8_t seq);
/* The unit's own read with motion, the tie-break (ace2k_feed_tiebreak_start), under the binding's
 * own mask. */
enum ace2k_feed_refusal ace2k_feed_binding_tiebreak_start(uint8_t lane);
/* The lanes, one bit per lane, that the feed has idle.  Under the binding's own mask. */
uint8_t ace2k_feed_binding_idle_bits(void);
/* Whether a read with motion with this seq is the transport's retry of the last one, under the
 * binding's own mask (ace2k_feed_search_retry). */
bool ace2k_feed_binding_search_retry(uint8_t lane, uint8_t seq);
/* Sets the search's reach, under the binding's own mask.  It returns the answer of
 * ace2k_feed_search_set(): 0, or -ACE2K_EINVAL above ACE2K_FEED_SEARCH_MAX_UM, with nothing
 * taken. */
int ace2k_feed_binding_search_set(uint32_t search_um);

#endif
