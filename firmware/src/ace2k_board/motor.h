/* What: the lines of the four lane motors:
 * - PWM on TIM2 CH1–4 (PA15 / PB3 / PB10 / PB11, the full remap), 30 kHz, active low: the pin
 *   high means stop;
 * - the run pins and the direction pins;
 * - the guard of the remap: a check every tick and on every duty write, and the reservation of the
 *   pins whose configuration would rewrite it.
 * How:
 * - ace2k_motor_init() once. Every line is at its stop level before a PWM pin is driven.
 * - ace2k_motor_pwm(lane, duty_pct), with 0 = stop; ace2k_motor_run(lane, on);
 *   ace2k_motor_dir(lane, reverse).
 * - ace2k_motor_stop_all() from the shutdown handler.
 * - ace2k_motor_remap_check() from the tick of the lane binding.
 * Pins and levels: docs/hardware.md "Motors". This is not Klipper's hard_pwm. Its F1 remap
 * emulation writes a partial TIM2 remap per pin, and the second write undoes the first.
 * Depends on: TIM2 and AFIO through CMSIS, Klipper's gpio, internal and command (board code). */
#ifndef ACE2K_BOARD_MOTOR_H
#define ACE2K_BOARD_MOTOR_H
#include <stdbool.h>
#include <stdint.h>

/* Once, at init: puts every line at its stop level, then starts TIM2's PWM at 30 kHz. */
void ace2k_motor_init(void);
/* Restores the TIM2 full remap if the register no longer holds it. The duty write checks it for a
 * move in progress. The tick of the lane binding calls this every 10 ms for the idle unit. So a
 * remap lost while idle is back before the next move starts. */
void ace2k_motor_remap_check(void);
/* Sets the lane's duty in percent: 0 stops, and above 100 reads as 100. A lane out of range
 * is ignored. */
void ace2k_motor_pwm(uint8_t lane, uint8_t duty_pct);
/* Sets the lane's run pin: high to run. A lane out of range is ignored. */
void ace2k_motor_run(uint8_t lane, bool on);
/* Sets the lane's direction pin: high for reverse. A lane out of range is ignored. */
void ace2k_motor_dir(uint8_t lane, bool reverse);
/* Drives every run pin low and every compare to stop, in that order. Safe with interrupts
 * disabled. */
void ace2k_motor_stop_all(void);

#endif
