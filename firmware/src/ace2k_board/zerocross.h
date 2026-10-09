/* What: the mains zero-cross edges — EXTI line 0 on PC0, rising edge — filtered by a 7 ms lockout
 * after each accepted edge (docs/hardware.md "Heater": a half-cycle is 8.3 ms at 60 Hz and 10 ms
 * at 50 Hz; a slow edge that crosses the input's threshold twice lands well inside the lockout),
 * counted, timed into a span for the mains measurement, and handed to one hook.  A flash operation
 * stalls the core with interrupts off: the edges in it are lost but one, which the pending bit
 * holds and the interrupt takes late, right at its end; the flash layer records whether the bit
 * was set then.  That late edge is a resync edge — it ends the span, every consumer restarts what
 * it builds on the edges' timing there — and the reject its lockout causes is not counted.
 * How: ace2k_zerocross_init() once; ace2k_zerocross_count() and ace2k_zerocross_rejects() return
 * the running counts and ace2k_zerocross_span() the span, read whole, its bridge handed
 * over (the mains binding reads all three, once a tick); ace2k_zerocross_set_hook() installs the
 * function the interrupt calls on every accepted edge with Klipper's timer ticks at the edge and
 * whether it is a resync edge (heat_cmds.c installs the heater's).  The decision itself is
 * ace2k_mains_edge_filter() (ace2k/mains.h), host-tested.  The triac gate (PC8) is driven by ace2k_board/gate.c alone; the
 * latch reset (PC9) by nothing.
 * Depends on: EXTI / AFIO through CMSIS, Klipper's timer and armcm interrupt registration (board
 * code), ace2k/mains.h, ace2k_board/flash.h (the flash operations' record). */
#ifndef ACE2K_BOARD_ZEROCROSS_H
#define ACE2K_BOARD_ZEROCROSS_H
#include <stdbool.h>
#include <stdint.h>
#include "ace2k/heat/mains.h" // struct ace2k_mains_span

#define ACE2K_ZEROCROSS_LOCKOUT_US 7000U

/* Interrupt context; now_ticks is timer_read_time() at the edge; resync: the edge was held pending
 * through a flash operation and taken late (ace2k/mains.h ace2k_mains_edge_filter). */
typedef void (*ace2k_zerocross_hook_fn)(uint32_t now_ticks, bool resync);

void ace2k_zerocross_init(void);
uint32_t ace2k_zerocross_count(void);
uint32_t ace2k_zerocross_rejects(void);
/* The interrupt's edge span, copied with interrupts off, its bridge handed over (the tick
 * reads it, once a tick). */
void ace2k_zerocross_span(struct ace2k_mains_span *out);
/* NULL removes the hook. */
void ace2k_zerocross_set_hook(ace2k_zerocross_hook_fn fn);

#endif
