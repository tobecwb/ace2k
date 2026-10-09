/* What: the one dryer instance of the image — the drying cycle over the heater, the fans and the
 * flaps, fed every tick with the outlet NTCs, the chamber, the mains, the insert sensors, the
 * cutout and the host link; its commands, periodic report, events and log.
 * How: heat_cmds.c's tick calls this binding's tick after airflow and heat (the one ordered
 * tick of the three, which survives a Klipper shutdown), with the world that tick read
 * (ace2k_heat_binding_world()) plus the chamber and the link; a task sends the report and the
 * events and stores the log on the config page, only with the gate idle (a page erase stalls the core);
 * the commands bracket each core call with interrupts masked.  The log the tick writes is the
 * binding's own copy, loaded from the config record at init and copied back into it, with the
 * tick masked, just before each store.  The events wait for the host's first query, so a boot
 * event (interrupted, the cutout still tripped) reaches a host that listens.  Compiled only with
 * CONFIG_ACE2K_DRYER.
 * Depends on: dryer.h, heat_cmds.h, airflow_cmds.h, config_cmds.h, env_cmds.h, health.h,
 * sensors.h, report.h, tx.h, ace2k_board/serial.h, ace2k_board/sysinfo.h, ace2k_board/tick.h,
 * Klipper's sched and command. */
#ifndef ACE2K_DRYER_CMDS_H
#define ACE2K_DRYER_CMDS_H
#include "dryer/dryer.h"

const struct ace2k_dryer *ace2k_dryer_binding(void);

#endif
