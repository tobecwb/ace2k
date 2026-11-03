/* What: the four filament encoders, as free-running 16-bit quadrature counters. TIM1, TIM8, TIM3
 * and TIM5 run in encoder mode, on both edges of both channels: x4.
 * How: ace2k_encoder_init() configures the pins and timers once. ace2k_encoder_read(lane) returns
 * the timer's CNT, and the lane core extends it. Pins: docs/hardware.md "Counters".
 * Depends on: the timer registers through CMSIS (board code, not host-compiled). */
#ifndef ACE2K_BOARD_ENCODER_H
#define ACE2K_BOARD_ENCODER_H
#include <stdint.h>

/* Once, at init: sets up the input pins, and the four timers in encoder mode. */
void ace2k_encoder_init(void);
/* The lane's raw count, which wraps at 16 bits; 0 for a lane out of range. */
uint16_t ace2k_encoder_read(uint8_t lane);

#endif
