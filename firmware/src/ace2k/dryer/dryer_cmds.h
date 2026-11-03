/* What: the image's one dryer instance. It runs the drying cycle with the heater, the fans
 * and the flaps. Every tick it is fed with the outlet NTCs, the chamber, the mains, the insert
 * sensors, the cutout and the host link. It has its commands, a periodic report, events and a
 * log.
 * How: heat_cmds.c's tick calls the tick of this binding after airflow and heat. That is the
 * one ordered tick of the three, and it survives a Klipper shutdown. It passes the world that tick
 * read (ace2k_heat_binding_world()), plus the chamber and the link. A task sends the report and
 * the events. The task also stores the log on the config page, only with the gate idle, because a
 * page erase stalls the core. The commands bracket each core call with interrupts masked.
 * The tick writes to a copy of the log that the binding owns. The copy is loaded from the config
 * record at init. Just before each store, it is copied back into the record with the tick masked.
 * The events wait for the host's first query. So a boot event, such as interrupted or the
 * cutout still tripped, reaches a host that listens. Compiled only with CONFIG_ACE2K_DRYER.
 * The automatic modes' instance lives here too, because its tick runs right after the dryer's.
 * The host's start and stop go through it. Its block reaches the page with the log, in the same
 * write. dryer_auto_cmds.c holds the modes' commands and report, through the calls below.
 * Depends on: dryer.h, dryer_auto.h, heat_cmds.h, airflow_cmds.h, config_cmds.h, env_cmds.h,
 * health.h, sensors.h, report.h, backoff.h, ace2k_board/tx.h, ace2k_board/serial.h,
 * ace2k_board/sysinfo.h, ace2k_board/tick.h, Klipper's sched and command. */
#ifndef ACE2K_DRYER_CMDS_H
#define ACE2K_DRYER_CMDS_H
#include "dryer/dryer.h"
#include "dryer/dryer_auto.h"

/* The image's one dryer instance, read-only. It is set up on first use. */
const struct ace2k_dryer *ace2k_dryer_binding(void);
/* The automatic modes' instance. It is set up on first use, so never fetch it first from the
 * timer interrupt. */
struct ace2k_dryer_auto *ace2k_dryer_auto_binding(void);
/* The op of ace2k_dryer_response: the command it answers. */
enum ace2k_dryer_cmd_op {
    ACE2K_DRYER_OP_START = 0,
    ACE2K_DRYER_OP_STOP = 1,
    ACE2K_DRYER_OP_CLEAR = 2,
    ACE2K_DRYER_OP_START_BOUND = 3, /* ace2k_dryer_start_bound, in dryer_auto_cmds.c */
    ACE2K_DRYER_OP_STOP_BOUND = 4,  /* ace2k_dryer_stop_bound, in dryer_auto_cmds.c */
};
/* Sends ace2k_dryer_response for op with the refusal r (0 ok). */
void ace2k_dryer_binding_respond(enum ace2k_dryer_cmd_op op, enum ace2k_dryer_refusal r);
/* The phase of the dryer's report (report.h). Its modes' report takes the same one, because the
 * dryer's task sends both. */
#define ACE2K_DRYER_REPORT_PHASE_MS 90U
/* Implemented in dryer_auto_cmds.c. The modes' report, which the dryer's task sends right after
 * the dryer's own. The tick runs in the timer interrupt, right after the modes' tick: it marks the
 * report due. The due flag reads true while it is due, also for a report that did not fit. The
 * send runs in the task: it tries a due report and returns false when its frame did not fit (the
 * report stays due for the next tick). */
void ace2k_dryer_auto_binding_tick(uint32_t now_ms);
bool ace2k_dryer_auto_binding_due(void);
bool ace2k_dryer_auto_binding_send(void);

#endif
