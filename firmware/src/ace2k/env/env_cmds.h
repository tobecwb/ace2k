/* What: the one env instance of the image: the two outlet NTCs, the chamber sensor on the
 * software I²C bus of the board, and VDDA from the internal reference.
 * How: every 10 ms, the tick feeds the NTC millivolts and the raw reference from the ADC scan. A
 * task runs the transaction of the chamber sensor, and sends the periodic report.
 * ace2k_env_binding() hands the instance to another binding; the health self-test reads it.
 * Compiled only with CONFIG_ACE2K_ENV. Callers guard their calls with the same flag.
 * Depends on: env.h, ace2k_board/adc_scan.h, ace2k_board/pins.h, ace2k_board/tick.h,
 * Klipper's software I²C, sched and command. */
#ifndef ACE2K_ENV_CMDS_H
#define ACE2K_ENV_CMDS_H
#include "env/env.h"

/* The image's one environment instance, read-only, set up on first use. */
const struct ace2k_env *ace2k_env_binding(void);

#endif
