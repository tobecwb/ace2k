/* What: the four lane motors' lines — PWM on TIM2 CH1–4 (PA15 / PB3 / PB10 / PB11, the full
 * remap, 30 kHz, active low: the pin high is stop), the run pins and the direction pins; the
 * guard of the remap (a re-check every tick and on every duty write, and the reservation of the
 * pins whose configuration would rewrite it).
 * How: ace2k_motor_init() once — every line at its stop level before a PWM pin is driven;
 * ace2k_motor_pwm(lane, duty_pct) with 0 = stop; ace2k_motor_run(lane, on);
 * ace2k_motor_dir(lane, reverse); ace2k_motor_stop_all() from the shutdown handler;
 * ace2k_motor_remap_check() from the lane binding's tick.  Pins and levels: docs/hardware.md
 * "Motors".  Not Klipper's hard_pwm: its F1 remap emulation writes a partial TIM2 remap per pin
 * and the second write undoes the first.
 * Depends on: TIM2 and AFIO through CMSIS, Klipper's gpio, internal and command (board code). */
#ifndef ACE2K_BOARD_MOTOR_H
#define ACE2K_BOARD_MOTOR_H
#include <stdbool.h>
#include <stdint.h>

void ace2k_motor_init(void);
/* Restores the TIM2 full remap if the register no longer holds it.  The duty write checks it for
 * a move in progress; the lane binding's tick calls this every 10 ms for the idle unit, so a
 * remap lost while idle is back before the next move starts. */
void ace2k_motor_remap_check(void);
void ace2k_motor_pwm(uint8_t lane, uint8_t duty_pct); /* 0 = stop; above 100 reads as 100 */
void ace2k_motor_run(uint8_t lane, bool on);
void ace2k_motor_dir(uint8_t lane, bool reverse);
/* Every run pin low and every compare at stop, in that order; safe with interrupts disabled. */
void ace2k_motor_stop_all(void);

#endif
