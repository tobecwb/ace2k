/* What: the scheduler of the tie-break.  The unit starts a read with motion on its own on a lane
 * whose tag is still pending, so that the watch can see which tag follows that lane.
 * How: the binding calls ace2k_rfid_tiebreak_note() after every watch step, with the inputs of that
 * step, then asks ace2k_rfid_tiebreak_pick() for a lane.  When it gets one, it starts the feed's
 * tie-break (ace2k_feed_tiebreak_start), and marks the lane tried once the start is accepted.
 * A lane is picked only when all of these hold.
 * - The link is up, and the host's mask of the lanes in use is valid.
 * - The lane is pending, its strand is at the insert sensor and the sensors are known, the host did
 *   not set it by hand, and the host does not have it in use.
 * - It was not tried since the last insertion on its reader.
 * - Its reader is not dead, and no session or read on command runs on the reader.
 * - The feed has the lane idle and its motor stopped.  The neighbour is idle too, or in use.
 * - The lanes of the reader that are not in use have been still for ACE2K_RFID_TIEBREAK_QUIET_MS.
 *   A strand going in counts as motion: the hand that put it in may still be there.
 * The lower lane goes first.  A tie-break that started spends the lane's try, however it ends.
 * Only a strand going in at either lane of the reader (both lanes) or the host's own read with
 * motion on the lane (ace2k_rfid_tiebreak_restart) makes a lane tried no more.
 * Depends on: <stdint.h>, rfid/watch.h. */
#ifndef ACE2K_RFID_WATCH_TIEBREAK_H
#define ACE2K_RFID_WATCH_TIEBREAK_H
#include <stdint.h>
#include "rfid/watch.h"

struct ace2k_rfid_tiebreak {
    uint8_t tried;       /* bit per lane: its tie-break started since the last insertion */
    uint8_t insert_last; /* the insert sensors of the last step, for their rising edges */
    /* For each reader, the clock when one of its lanes not in use last moved, or a strand went
     * in. */
    uint32_t t_still_ms[ACE2K_RFID_READER_COUNT];
};

/* Takes the inputs of the watch step that just ran.  A strand going in clears tried for both lanes
 * of its reader, and restarts the reader's quiet time.  A lane that
 * moves and is not in use restarts it too.  While the mask of the lanes in use is not valid, every
 * lane counts. */
void ace2k_rfid_tiebreak_note(struct ace2k_rfid_tiebreak *tb, const struct ace2k_rfid_watch *w,
                              const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms);

/* Returns the lane whose tie-break may start now, or -1.  idle_mask has a bit per lane that the
 * feed has idle with its motor stopped.  The rest comes from the watch's last step. */
int8_t ace2k_rfid_tiebreak_pick(const struct ace2k_rfid_tiebreak *tb,
                                const struct ace2k_rfid_watch *w, uint8_t idle_mask,
                                uint32_t now_ms);

/* The tie-break of the lane started.  It is not picked again until a strand goes in on its
 * reader.  A lane out of range is ignored. */
void ace2k_rfid_tiebreak_tried(struct ace2k_rfid_tiebreak *tb, uint8_t lane);

/* The host's own read with motion started on the lane: its request to start over.  The lane is
 * tried no more.  Out of range is ignored. */
void ace2k_rfid_tiebreak_restart(struct ace2k_rfid_tiebreak *tb, uint8_t lane);

#endif
