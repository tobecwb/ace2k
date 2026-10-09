/* What: the one heat instance of the image, its ordered tick, the zero-cross hook, the
 * vetoes, the shutdown.  The dryer is the only lease holder.
 * How: one tick callback runs airflow's tick, then reads the world once (env, mains, sensors:
 * struct ace2k_heat_world), then heat's tick (its inputs from that world and airflow), then the
 * after hook the dryer binding installs (which reads the same world); the zero-cross interrupt
 * calls ace2k_heat_zerocross() through ace2k_zerocross_set_hook().
 * Guard veto (the bootloader): ace2k_heat_active(); config-store veto: ace2k_heat_busy().
 * Depends on: heat.h, airflow_cmds.h, env_cmds.h, mains_cmds.h, sensors_cmds.h, guard_cmds.h,
 * config_cmds.h, ace2k_board/gate.h, ace2k_board/zerocross.h, ace2k_board/tick.h, Klipper's
 * sched and command. */
#ifndef ACE2K_HEAT_CMDS_H
#define ACE2K_HEAT_CMDS_H
#include <stdbool.h>
#include <stdint.h>
#include "heat/heat.h"

typedef void (*ace2k_heat_after_fn)(uint32_t now_ms);

/* What the ordered tick read this tick, before heat's step, for heat and the dryer alike.  An
 * NTC whose reading is invalid keeps its last valid value (0 before the first) with its valid
 * flag false — never a sentinel.  Written and read in the timer interrupt only. */
struct ace2k_heat_world {
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    bool mains_present, mains_plausible;
    bool mains_measured; /* ace2k_mains_measured(): a window measured since the mains came back */
    uint16_t mains_hz10; /* the last published frequency, 0.1 Hz (mains.h; its markers are
                          * off-band): heat's half-period */
    bool cutout;
    uint16_t switches; /* ace2k_sensors_switches(): the insert sensors among them */
};

struct ace2k_heat *ace2k_heat_binding(void);

/* This tick's world (the pointer is fixed; the dryer's after hook reads through it). */
const struct ace2k_heat_world *ace2k_heat_binding_world(void);

/* The dryer binding's tick, run right after heat's in the same callback.  Set once, from init
 * context. */
void ace2k_heat_binding_set_after_hook(ace2k_heat_after_fn fn);

#endif
