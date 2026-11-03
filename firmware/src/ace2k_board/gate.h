/* What: the triac gate PC8 and its pulse timer TIM7. This is the only code that configures or
 * writes PC8. The latch-reset pin of the cutout, PC9, is not touched here, nor anywhere in any
 * image.
 * How:
 * - ace2k_gate_init() runs once. It makes PC8 an output driving low: the level first, then the
 *   mode. It sets TIM7 in one-pulse mode at 1 µs per count, with its update interrupt enabled.
 * - ace2k_gate_fire() raises the gate and restarts the timer. The timer interrupt drops the gate
 *   ACE2K_HEAT_GATE_PULSE_US later, whatever else runs.
 * - ace2k_gate_off() stops the timer first, then drops the gate.
 * - ace2k_gate_float() is for the shutdown only, with interrupts disabled. It returns PC8 to a
 *   floating input, and the gate's external pull-down holds it off (docs/hardware.md
 *   "Heater").
 * - ace2k_gate_now_us() is a free-running microsecond clock that wraps at 2^32 µs. heat uses it
 *   for the ages of its gate check: the gate is never left high. It must be called at least once
 *   per 2^32 core cycles (35.8 s at 120 MHz), and the 10 ms tick does.
 * - ace2k_gate_us_at() gives that clock at a cycle count read less than 2^31 cycles (17.9 s)
 *   before the call, rounded down to the microsecond. That count is read at the entry of the
 *   zero-cross handler. So heat times the interrupt entry of each edge, not the handler's run
 *   after it.
 * Depends on: Klipper's gpio, clock lines, timer and armcm interrupt registration (board code). */
#ifndef ACE2K_BOARD_GATE_H
#define ACE2K_BOARD_GATE_H
#include <stdbool.h>
#include <stdint.h>

/* Once, at init: drives PC8 low, and readies TIM7 in one-pulse mode. */
void ace2k_gate_init(void);
/* Interrupt context: raises the gate for one pulse of ACE2K_HEAT_GATE_PULSE_US. */
void ace2k_gate_fire(void);
/* Stops the pulse timer, then drops the gate. */
void ace2k_gate_off(void);
/* True while PC8 reads high, that is while the triac may conduct. */
bool ace2k_gate_read(void);
/* Shutdown only, with interrupts disabled: turns the gate off, then makes PC8 a floating input. */
void ace2k_gate_float(void);
/* The microsecond clock now; it wraps at 2^32 µs. */
uint32_t ace2k_gate_now_us(void);
/* The microsecond clock at the moment the core cycle counter read the value in cycles. That read
 * must be less than 2^31 cycles old (17.9 s at 120 MHz). */
uint32_t ace2k_gate_us_at(uint32_t cycles);

#endif
