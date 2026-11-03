/* What: the image's one mains instance: the frequency and the presence of the mains, from
 * the zero-cross edges that the board counts on PC0.
 * How: the tick feeds the core every 10 ms, and a task sends the periodic report.
 * ace2k_mains_binding() hands the instance to another binding; the health self-test reads it.
 * Compiled only with CONFIG_ACE2K_MAINS. Callers guard their calls with the same flag.
 * Depends on: mains.h, ace2k_board/zerocross.h, ace2k_board/tick.h, Klipper's sched and
 * command. */
#ifndef ACE2K_MAINS_CMDS_H
#define ACE2K_MAINS_CMDS_H
#include "heat/mains.h"

/* The image's one mains instance, read-only, set up on first use. */
const struct ace2k_mains *ace2k_mains_binding(void);

#endif
