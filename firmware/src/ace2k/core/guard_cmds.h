/* What: the one guard instance, ace2k_bootloader_enter, and the halt. The halt stops feeding the
 * watchdog, so the chip resets into the bootloader's recovery.
 * How: subsystems register vetoes on ace2k_guard_binding(). The link proof asks for the halt with
 * ace2k_guard_binding_request_halt_ms(0) from its tick, only on a tick where
 * ace2k_guard_bootloader_allowed() is true. It asks again every tick until then. The halt itself
 * runs in a task.
 * Depends on: guard.h, Klipper's sched, timer and irq. */
#ifndef ACE2K_GUARD_CMDS_H
#define ACE2K_GUARD_CMDS_H
#include <stdint.h>
#include "core/guard.h"

/* The image's one guard instance, initialised on first use. */
struct ace2k_guard *ace2k_guard_binding(void);

/* Requests the halt delay_ms from now; 0 means at the next task run. Only delay 0 is safe from
 * an interrupt, because it only sets a flag. A delay above 0 adds a timer, so call it from
 * command context only. Right before the halt, the halt task asks the vetoes again. A veto that
 * objects then withdraws the request. Once the halt runs, it cannot be undone: the watchdog
 * resets the chip within 273–546 ms. */
void ace2k_guard_binding_request_halt_ms(uint32_t delay_ms);

#endif
