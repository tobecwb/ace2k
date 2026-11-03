/* What: the dryer's log, as the config page keeps it: counters, flags and the last eight
 * entries.
 * How: types only. dryer.h declares the functions that change it (dryer_log.c).
 * Depends on: <stdint.h>. */
#ifndef ACE2K_DRYER_LOG_H
#define ACE2K_DRYER_LOG_H
#include <stdint.h>

struct ace2k_dryer_log_entry { /* 16 bytes on the page */
    uint8_t kind;              /* ACE2K_DRYER_EV_FAULT or ACE2K_DRYER_EV_INTERRUPTED */
    uint8_t reason;            /* enum ace2k_dryer_fault, 0 for interrupted */
    uint16_t cycle;            /* the cycle open at the time (cycles_started), 0 if none was */
    uint32_t heat_s;           /* heat_s at the time */
    int16_t ntc_left_dc, ntc_right_dc, chamber_dc; /* 0.1 °C */
    uint16_t reserved;
};

#define ACE2K_DRYER_LOG_ENTRIES 8U
struct ace2k_dryer_log {
    uint32_t cycles_started, cycles_completed, heat_s, full_power_s, faults;
    uint32_t flags; /* ACE2K_DRYER_LOG_CYCLE_OPEN, ACE2K_DRYER_LOG_CUTOUT */
    /* the sub-second remainders of the two time counters; not persisted */
    uint32_t full_power_ms_frac;
    uint32_t heat_ms_frac;
    struct ace2k_dryer_log_entry entry[ACE2K_DRYER_LOG_ENTRIES]; /* [0] newest */
};
#define ACE2K_DRYER_LOG_CYCLE_OPEN 0x1U
#define ACE2K_DRYER_LOG_CUTOUT     0x2U

#define ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC INT16_MIN /* a reading that was invalid or absent */

#endif
