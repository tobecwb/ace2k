/* What: the four motor tach (FG) outputs as pulse counters — TIM4 CH1–4 input capture, one
 * interrupt per falling edge, no capture value used.
 * How: ace2k_fg_init() once; ace2k_fg_read(lane) returns the running count (wraps at 2^32).
 * Depends on: TIM4 through CMSIS, Klipper's armcm interrupt registration (board code). */
#ifndef ACE2K_BOARD_FG_H
#define ACE2K_BOARD_FG_H
#include <stdint.h>

void ace2k_fg_init(void);
uint32_t ace2k_fg_read(uint8_t lane);

#endif
