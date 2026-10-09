/* What: the one sensors instance of the image — the four insert bands, the *empty* millivolts,
 * the buffer switches and the cutout input, fed by the board's ADC scan and GPIO inputs.
 * How: the tick feeds the core every 10 ms; a task drains the events and sends the periodic
 * report; ace2k_sensors_binding() hands the instance to another binding (the health self-test
 * reads it).  Compiled only with CONFIG_ACE2K_SENSORS; callers guard their calls with the
 * same flag.
 * Depends on: sensors.h, config_cmds.h (the factory bands), ace2k_board/adc_scan.h,
 * ace2k_board/pins.h, ace2k_board/tick.h, Klipper's gpio, sched and command. */
#ifndef ACE2K_SENSORS_CMDS_H
#define ACE2K_SENSORS_CMDS_H
#include "lane/sensors.h"

const struct ace2k_sensors *ace2k_sensors_binding(void);

#endif
