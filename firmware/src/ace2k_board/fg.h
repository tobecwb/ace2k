/* What: the four motor tach (FG) outputs, as pulse counters. TIM4 CH1–4 run in input capture,
 * with one interrupt per falling edge. No capture value is used.
 * How: call ace2k_fg_init() once. ace2k_fg_read(lane) returns the running count, which wraps at
 * 2^32.
 * Depends on: TIM4 through CMSIS, Klipper's armcm interrupt registration (board code). */
#ifndef ACE2K_BOARD_FG_H
#define ACE2K_BOARD_FG_H
#include <stdint.h>

/* Once, at init: sets up TIM4's four capture channels and their interrupt. */
void ace2k_fg_init(void);
/* The lane's tach pulses since boot, which wrap at 2^32; 0 for a lane out of range. */
uint32_t ace2k_fg_read(uint8_t lane);

#endif
