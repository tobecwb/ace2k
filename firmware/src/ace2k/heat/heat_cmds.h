/* What: the image's one heat instance, its ordered tick, the zero-cross hook, the vetoes and
 * the shutdown. The dryer is the only lease holder.
 * How: one tick callback runs, in this order: airflow's tick; one read of the world (env,
 * mains, sensors: struct ace2k_heat_world); heat's tick, with its inputs from that world and
 * airflow; then the after hook that the dryer binding installs, which reads the same world. The
 * zero-cross interrupt calls ace2k_heat_zerocross() through ace2k_zerocross_set_hook().
 * The veto of the guard (the bootloader) is ace2k_heat_active(). The veto of the config store is
 * ace2k_heat_busy().
 * Depends on: heat.h, airflow_cmds.h, env_cmds.h, mains_cmds.h, sensors_cmds.h, guard_cmds.h,
 * config_cmds.h, ace2k_board/gate.h, ace2k_board/zerocross.h, ace2k_board/tick.h, Klipper's
 * sched and command. */
#ifndef ACE2K_HEAT_CMDS_H
#define ACE2K_HEAT_CMDS_H
#include <stdbool.h>
#include <stdint.h>
#include "heat/heat.h"

typedef void (*ace2k_heat_after_fn)(uint32_t now_ms);

/* What the ordered tick read in this tick, before heat's step, for heat and the dryer alike.
 * An NTC whose reading is invalid keeps its last valid value, or 0 before the first, with its
 * valid flag false. It is never a sentinel. Written and read in the timer interrupt only. */
struct ace2k_heat_world {
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    bool mains_present, mains_plausible;
    bool mains_measured; /* ace2k_mains_measured(): a window measured since the mains came back */
    uint16_t mains_hz10; /* the last published frequency, in 0.1 Hz; its markers are off-band
                          * (mains.h). heat takes its half-period from it. */
    bool cutout;
    uint16_t switches; /* ace2k_sensors_switches(), the insert sensors among them */
};

/* The image's one heat instance, set up on first use. */
struct ace2k_heat *ace2k_heat_binding(void);

/* The world of this tick. The pointer is fixed; the dryer's after hook reads through it. */
const struct ace2k_heat_world *ace2k_heat_binding_world(void);

/* The tick of the dryer binding, run right after heat's tick in the same callback. Set it
 * once, from init context. */
void ace2k_heat_binding_set_after_hook(ace2k_heat_after_fn fn);

#endif
