/* What: the four filament encoders as free-running 16-bit quadrature counters (TIM1, TIM8,
 * TIM3, TIM5 in encoder mode, both edges of both channels: x4).
 * How: ace2k_encoder_init() configures the pins and timers once; ace2k_encoder_read(lane)
 * returns the timer's CNT; the lane core extends it.  Pins: docs/hardware.md "Counters".
 * Depends on: the timer registers through CMSIS (board code, not host-compiled). */
#ifndef ACE2K_BOARD_ENCODER_H
#define ACE2K_BOARD_ENCODER_H
#include <stdint.h>

void ace2k_encoder_init(void);
uint16_t ace2k_encoder_read(uint8_t lane);

#endif
