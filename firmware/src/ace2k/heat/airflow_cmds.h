/* What: the image's one airflow instance: the fans and the flaps on the board, the thermal
 * fan rule fed with the outlet NTCs, the manual commands and the periodic report.
 * How: ace2k_airflow_binding() hands the instance to the heat and dryer bindings, in task or tick
 * context, after init. ace2k_airflow_binding_tick() runs the core with env's NTCs. The tick
 * calls it every 10 ms, before heat and dryer. Compiled only with CONFIG_ACE2K_HEAT.
 * Depends on: airflow.h, env_cmds.h, ace2k_board/airflow_io.h, ace2k_board/tick.h, Klipper's
 * sched and command. */
#ifndef ACE2K_AIRFLOW_CMDS_H
#define ACE2K_AIRFLOW_CMDS_H
#include <stdint.h>
#include "heat/airflow.h"

/* The image's one airflow instance, set up on first use. */
struct ace2k_airflow *ace2k_airflow_binding(void);

/* Interrupt context, in the 10 ms tick. Runs the core's tick with the NTCs that env holds,
 * and wakes the report. */
void ace2k_airflow_binding_tick(uint32_t now_ms);

#endif
