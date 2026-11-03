/* What: the board's millisecond clock, and the 10 ms tick that the bindings hang their core
 * ticks on.
 * How: one Klipper timer. ace2k_tick_register() adds a callback, called in interrupt context every
 * ACE2K_TICK_PERIOD_MS with now_ms. ace2k_tick_now_ms() is readable from any context.
 * Depends on: Klipper's sched and timer (board code, not host-compiled). */
#ifndef ACE2K_BOARD_TICK_H
#define ACE2K_BOARD_TICK_H
#include <stdint.h>

#define ACE2K_TICK_PERIOD_MS 10U
/* led, linkproof, sensors, env, lane, mains, health, feed, rfid, heat: 10 in the fullest image,
 * two to spare. Heat's one callback runs airflow, heat and the dryer in order. */
#define ACE2K_TICK_SLOTS 12

typedef void (*ace2k_tick_fn)(void *ctx, uint32_t now_ms);

/* Returns 0, or -ACE2K_EFULL when every slot is taken. Safe from task context at any time,
 * because the registration is made in one step against the running tick. */
int ace2k_tick_register(ace2k_tick_fn fn, void *ctx);

/* Milliseconds since boot, in steps of ACE2K_TICK_PERIOD_MS. Wraps after 49 days. */
uint32_t ace2k_tick_now_ms(void);

#endif
