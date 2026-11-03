/* What: the one sensors instance of the image.  It holds the four insert bands, the *empty*
 * millivolts, the buffer switches and the cutout input.  The ADC scan and the GPIO inputs of the
 * board feed it.
 * How: the tick feeds the core every 10 ms.  A task drains the events and sends the periodic
 * report.  ace2k_sensors_binding() hands the instance to another binding, for example the health
 * self-test, which reads it.  This file is compiled only with CONFIG_ACE2K_SENSORS, and callers
 * guard their calls with the same flag.
 * Depends on: sensors.h, config_cmds.h (the factory bands), ace2k_board/adc_scan.h,
 * ace2k_board/pins.h, ace2k_board/tick.h, Klipper's gpio, sched and command. */
#ifndef ACE2K_SENSORS_CMDS_H
#define ACE2K_SENSORS_CMDS_H
#include "lane/sensors.h"

/* Returns the one sensors instance of the image, read-only.  It is set up on first use. */
const struct ace2k_sensors *ace2k_sensors_binding(void);

#endif
