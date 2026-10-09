/* What: the one guard instance, ace2k_bootloader_enter, and the halt that stops feeding the
 * watchdog so the chip resets into the bootloader's recovery.
 * How: subsystems register vetoes on ace2k_guard_binding(); the link proof asks for the halt
 * with ace2k_guard_binding_request_halt_ms(0) from its tick, only on a tick where
 * ace2k_guard_bootloader_allowed() (it asks again every tick until then); the halt itself runs
 * in a task.
 * Depends on: guard.h, Klipper's sched, timer and irq. */
#ifndef ACE2K_GUARD_CMDS_H
#define ACE2K_GUARD_CMDS_H
#include <stdint.h>
#include "core/guard.h"

struct ace2k_guard *ace2k_guard_binding(void);

/* Stops the scheduler delay_ms from now (0 = at the next task run).  Safe from an interrupt.
 * Irreversible: the watchdog resets the chip within half a second. */
void ace2k_guard_binding_request_halt_ms(uint32_t delay_ms);

#endif
