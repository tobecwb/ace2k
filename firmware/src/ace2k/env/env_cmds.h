/* What: the one env instance of the image — the two outlet NTCs, the chamber sensor over the
 * board's software I²C bus, VDDA from the internal reference.
 * How: the tick feeds the NTC millivolts and the raw reference from the ADC scan every 10 ms;
 * a task runs the chamber sensor's transaction and sends the periodic report;
 * ace2k_env_binding() hands the instance to another binding (the health self-test reads it).
 * Compiled only with CONFIG_ACE2K_ENV; callers guard their calls with the same flag.
 * Depends on: env.h, ace2k_board/adc_scan.h, ace2k_board/pins.h, ace2k_board/tick.h,
 * Klipper's software I²C, sched and command. */
#ifndef ACE2K_ENV_CMDS_H
#define ACE2K_ENV_CMDS_H
#include "env/env.h"

const struct ace2k_env *ace2k_env_binding(void);

#endif
