/* What: the order of the channels in every ADC pass the board delivers, and the raw → mV rule.
 * How: the board scan fills a uint16_t[ACE2K_ADC_COUNT] in this order; sensors and env index
 * it with these names.  Twelve channels: the ten external inputs of docs/hardware.md "Analogue
 * inputs", the auxiliary, and the internal reference.  ADC1, 12-bit, 3.3 V reference.  The MCU's
 * own temperature sensor is left to Klipper's temperature_mcu, which reads it through Klipper's
 * analog_in, so the two users of the ADC never share a channel (measured 2026-09-17: sharing it
 * with a back-to-back scan stalled the host link).
 * Depends on: <stdint.h>. */
#ifndef ACE2K_ADC_MAP_H
#define ACE2K_ADC_MAP_H
#include <stdint.h>

enum ace2k_adc_index {
    ACE2K_ADC_NTC_LEFT,  /* PA5, ch 5 */
    ACE2K_ADC_NTC_RIGHT, /* PA4, ch 4 */
    ACE2K_ADC_INSERT1,   /* PC5, ch 15 — lanes 1–4 contiguous */
    ACE2K_ADC_INSERT2,
    ACE2K_ADC_INSERT3,
    ACE2K_ADC_INSERT4,
    ACE2K_ADC_EMPTY1, /* PA2, ch 2 — lanes 1–4 contiguous */
    ACE2K_ADC_EMPTY2,
    ACE2K_ADC_EMPTY3,
    ACE2K_ADC_EMPTY4,
    ACE2K_ADC_AUX,     /* PC1, ch 11 */
    ACE2K_ADC_VREFINT, /* internal 1.2 V reference, ch 17 */
    ACE2K_ADC_COUNT,
};

#define ACE2K_ADC_FULL_SCALE 4095U
#define ACE2K_ADC_REF_MV     3300U

static inline uint16_t ace2k_adc_raw_to_mv(uint16_t raw)
{
    return (uint16_t)(((uint32_t)raw * ACE2K_ADC_REF_MV) / ACE2K_ADC_FULL_SCALE);
}

#endif
