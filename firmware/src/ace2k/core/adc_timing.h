/* What: the period of an ADC pass and the age past which a pass says nothing.
 * How: env (the NTCs and VDDA) and lane (adc_map.h, the inserts) both read these, so the two never
 * disagree on a stopped scan.
 * Depends on: nothing. */
#ifndef ACE2K_ADC_TIMING_H
#define ACE2K_ADC_TIMING_H

/* The period of a pass, ACE2K_ADC_SCAN_PERIOD_MS in ace2k_board/adc_scan.h. The core cannot
 * include a board header. So the binding checks that the two agree, and that the tick has the
 * same period. */
#define ACE2K_ADC_PASS_PERIOD_MS 10U
/* A pass older than this many periods says nothing: env makes both NTC readings and VDDA invalid,
 * and sensors leaves the inserts unknown. A flash erase holds the scan for tens of ms, and the
 * next pass ends up to two periods after it. Ten periods let an erase go by without the readings
 * turning invalid. The age comes in steps of the tick (10 ms), so the first age past the limit is
 * 110 ms. Heat stops firing on the first invalid tick. So it never fires on a reading more than
 * about 120 ms old. At the fastest rise measured (0.81 °C/s at 50 % duty, docs/hardware.md
 * "Heater"), that is about 0.1 °C. Heat allows up to 90 % duty, so the rise can be faster. Even
 * twice as fast is 0.2 °C. That is small against the 12 °C between the 73 °C drive ceiling and
 * ACE2K_HEAT_NTC_MAX_MC. The age wraps after 2^32 ms (49.7 days) of a frozen scan. Every reader
 * has seen the readings invalid long before then. docs/protocol.md names the value. Initial
 * value; re-tune on the bench. */
#define ACE2K_ADC_STALE_PASSES 10U
#define ACE2K_ADC_STALE_MS     (ACE2K_ADC_PASS_PERIOD_MS * ACE2K_ADC_STALE_PASSES)

#endif
