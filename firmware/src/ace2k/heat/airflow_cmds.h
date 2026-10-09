/* What: the one airflow instance of the image — the fans and the flaps on the board, the thermal
 * fan rule fed with the outlet NTCs, the manual commands and the periodic report.
 * How: ace2k_airflow_binding() hands the instance to the heat and dryer bindings (task or tick
 * context, after init); ace2k_airflow_binding_tick() runs the core with env's NTCs — called
 * every 10 ms from the tick, before heat and dryer.  Compiled only with CONFIG_ACE2K_HEAT.
 * Depends on: airflow.h, env_cmds.h, ace2k_board/airflow_io.h, ace2k_board/tick.h, Klipper's
 * sched and command. */
#ifndef ACE2K_AIRFLOW_CMDS_H
#define ACE2K_AIRFLOW_CMDS_H
#include <stdint.h>
#include "heat/airflow.h"

struct ace2k_airflow *ace2k_airflow_binding(void);

/* Interrupt context (the 10 ms tick): the core's tick with the NTCs env holds, and the report's
 * wake. */
void ace2k_airflow_binding_tick(uint32_t now_ms);

#endif
