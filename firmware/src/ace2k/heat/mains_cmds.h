/* What: the one mains instance of the image — the frequency and presence of the mains, from the
 * zero-cross edges the board counts on PC0.
 * How: the tick feeds the core every 10 ms; a task sends the periodic report;
 * ace2k_mains_binding() hands the instance to another binding (the health self-test reads it).
 * Compiled only with CONFIG_ACE2K_MAINS; callers guard their calls with the same flag.
 * Depends on: mains.h, ace2k_board/zerocross.h, ace2k_board/tick.h, Klipper's sched and
 * command. */
#ifndef ACE2K_MAINS_CMDS_H
#define ACE2K_MAINS_CMDS_H
#include "heat/mains.h"

const struct ace2k_mains *ace2k_mains_binding(void);

#endif
