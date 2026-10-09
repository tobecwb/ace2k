/* What: the four lane LEDs of the board behind the led module.
 * How: ace2k_led_binding_set_pattern() from any binding, ace2k_led_binding_set_lane() from any
 * binding's tick; the tick drives the patterns.
 * Compiled only with CONFIG_ACE2K_LED; callers guard their calls with the same flag.
 * Depends on: led.h, ace2k_board/pins.h, ace2k_board/tick.h, Klipper's gpio. */
#ifndef ACE2K_LED_CMDS_H
#define ACE2K_LED_CMDS_H
#include "lane/led.h"

void ace2k_led_binding_set_pattern(enum ace2k_led_pattern pattern);

/* From any binding's tick; a no-op before init. */
void ace2k_led_binding_set_lane(uint8_t lane, enum ace2k_led_lane_pattern pattern);

#endif
