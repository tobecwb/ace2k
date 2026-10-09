/* What: every ADC channel of adc_map.h sampled in turn, one pass every 10 ms, published with
 * the tick clock's time of completion.
 * How: one Klipper timer; gpio_adc_sample()/gpio_adc_read() one conversion at a time, the
 * channels ACE2K_ADC_SCAN_SLOT_US apart so that a pass spans its period and no two conversions
 * of ours run back-to-back (Klipper's own ADC users then wait one conversion at most);
 * ace2k_adc_scan_latest() copies the last complete pass (timer context only — the callers are
 * the 10 ms tick callbacks, which run in the same context, so no lock is needed).  The age is
 * the stamp's; the first channel of a pass is up to ACE2K_ADC_COUNT − 1 slots older than it.
 * Depends on: Klipper's gpio, sched and timer (board code, not host-compiled); adc_map.h. */
#ifndef ACE2K_BOARD_ADC_SCAN_H
#define ACE2K_BOARD_ADC_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#include "ace2k/lane/adc_map.h"

#define ACE2K_ADC_SCAN_PERIOD_MS 10U
/* one channel per slot: 10 ms / 12 channels = 833 µs; the last channel starts at 9 163 µs */
#define ACE2K_ADC_SCAN_SLOT_US (ACE2K_ADC_SCAN_PERIOD_MS * 1000U / ACE2K_ADC_COUNT)

/* Copies the last complete pass (raw 12-bit) and its age; false (nothing copied) before the
 * first pass completes. */
bool ace2k_adc_scan_latest(uint16_t raw[ACE2K_ADC_COUNT], uint32_t *age_ms);

#endif
