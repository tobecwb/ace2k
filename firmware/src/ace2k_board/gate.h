/* What: the triac gate PC8 and its pulse timer TIM7 — the only code that configures or writes
 * PC8.  The cutout latch-reset pin PC9 is not touched here or
 * anywhere in any image.
 * How: ace2k_gate_init() once: PC8 an output driving low (the level first, then the mode), TIM7
 * in one-pulse mode at 1 µs per count, its update interrupt enabled.  ace2k_gate_fire() raises
 * the gate and restarts the timer; the timer's interrupt drops the gate ACE2K_HEAT_GATE_PULSE_US
 * later, whatever else runs.  ace2k_gate_off() stops the timer first, then drops the gate.
 * ace2k_gate_float() (shutdown only, interrupts disabled) returns PC8 to a floating input — the
 * gate's external pull-down holds it off (docs/hardware.md "Heater").  ace2k_gate_now_us() is a
 * free-running microsecond clock that wraps at 2^32 µs, for heat's rule-5 ages; it must be
 * called at least once per 2^32 core cycles (35.8 s at 120 MHz) — the 10 ms tick does.
 * ace2k_gate_us_at() gives that clock at a cycle count read less than 2^31 cycles (17.9 s)
 * before the call, rounded down to the microsecond — the zero-cross handler's entry, so heat times
 * each edge's interrupt entry, not the handler's run after it.
 * Depends on: Klipper's gpio, clock lines, timer and armcm interrupt registration (board code). */
#ifndef ACE2K_BOARD_GATE_H
#define ACE2K_BOARD_GATE_H
#include <stdbool.h>
#include <stdint.h>

void ace2k_gate_init(void);
void ace2k_gate_fire(void);
void ace2k_gate_off(void);
bool ace2k_gate_read(void);
void ace2k_gate_float(void);
uint32_t ace2k_gate_now_us(void);
uint32_t ace2k_gate_us_at(uint32_t cycles);

#endif
