/* What: the band of an insert sensor, in millivolts, and where it came from.
 * How: types only. core/config fills them from the factory page, and lane/sensors uses them.
 * Depends on: <stdint.h>. */
#ifndef ACE2K_CALIBRATION_H
#define ACE2K_CALIBRATION_H
#include <stdint.h>

enum ace2k_calibration_source {
    ACE2K_CAL_DEFAULT = 0, /* built-in 600 / 1100 mV */
    ACE2K_CAL_FACTORY = 1, /* the unit's factory page */
    ACE2K_CAL_CONFIG = 2,  /* printer.cfg, through ace2k_sensors_thresholds_set */
};

struct ace2k_insert_band {
    uint16_t low_mv;
    uint16_t high_mv;
    enum ace2k_calibration_source source;
};

#endif
