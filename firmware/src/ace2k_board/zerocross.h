/* What: the mains zero-cross edges, on EXTI line 0 on PC0, rising edge. They are filtered by a
 * 7 ms lockout after each accepted edge. docs/hardware.md "Heater": a half-cycle is 8.3 ms at
 * 60 Hz and 10 ms at 50 Hz, and a slow edge that crosses the input's threshold twice lands
 * well inside the lockout. The edges are counted, timed into a span for the mains measurement,
 * and handed to one hook.
 * A flash operation stalls the core with interrupts off. The edges inside it are lost, except
 * one: the pending bit holds it, and the interrupt takes it late, right at the end. The flash
 * layer records whether the bit was set then. That late edge is a resync edge. It ends the span,
 * and every consumer restarts there whatever it builds on the timing of the edges. The reject that
 * its lockout causes is not counted.
 * How:
 * - ace2k_zerocross_init() once.
 * - ace2k_zerocross_count() and ace2k_zerocross_rejects() return the running counts.
 *   ace2k_zerocross_span() returns the span, read whole, with its bridge handed over. The mains
 *   binding reads all three, once a tick.
 * - ace2k_zerocross_set_hook() installs the function that the interrupt calls on every accepted
 *   edge, with the Klipper timer ticks at the edge, and whether it is a resync edge. heat_cmds.c
 *   installs the one of the heater.
 * The decision itself is ace2k_mains_edge_filter() (heat/mains.h), host-tested. Only
 * ace2k_board/gate.c drives the triac gate (PC8). Nothing drives the latch reset (PC9).
 * Depends on: EXTI / AFIO through CMSIS, Klipper's timer and armcm interrupt registration
 * (board code), heat/mains.h, ace2k_board/flash.h (the record of the flash operations). */
#ifndef ACE2K_BOARD_ZEROCROSS_H
#define ACE2K_BOARD_ZEROCROSS_H
#include <stdbool.h>
#include <stdint.h>
#include "ace2k/heat/mains.h" // struct ace2k_mains_span

#define ACE2K_ZEROCROSS_LOCKOUT_US 7000U

/* Interrupt context. now_ticks is timer_read_time() at the edge. resync means the edge was held
 * pending through a flash operation, and taken late (ace2k_mains_edge_filter in heat/mains.h). */
typedef void (*ace2k_zerocross_hook_fn)(uint32_t now_ticks, bool resync);

/* Once, at init: makes PC0 an input on EXTI line 0, rising edge, and enables its interrupt. */
void ace2k_zerocross_init(void);
/* The edges accepted since boot; wraps at 2^32. */
uint32_t ace2k_zerocross_count(void);
/* The edges that the lockout rejected since boot. The one after a resync edge is not counted. */
uint32_t ace2k_zerocross_rejects(void);
/* The interrupt's edge span, copied with interrupts off, with its bridge handed over. The
 * tick reads it, once a tick. */
void ace2k_zerocross_span(struct ace2k_mains_span *out);
/* NULL removes the hook. */
void ace2k_zerocross_set_hook(ace2k_zerocross_hook_fn fn);

#endif
