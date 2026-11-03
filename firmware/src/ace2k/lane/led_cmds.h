/* What: the four lane LEDs of the board, behind the led module.
 * How: any binding may call ace2k_led_binding_set_pattern().  Any binding may call
 * ace2k_led_binding_set_lane() from its tick.  The tick drives the patterns.  This file is compiled
 * only with CONFIG_ACE2K_LED, and callers guard their calls with the same flag.
 * Depends on: led.h, ace2k_board/pins.h, ace2k_board/tick.h, Klipper's gpio. */
#ifndef ACE2K_LED_CMDS_H
#define ACE2K_LED_CMDS_H
#include "lane/led.h"

/* Sets the pattern of the whole unit.  It shows at once and restarts from its beginning.  INTRO
 * again while the pattern is INTRO changes nothing. */
void ace2k_led_binding_set_pattern(enum ace2k_led_pattern pattern);

/* Sets the pattern of a lane, from the tick of any binding.  It does nothing before init. */
void ace2k_led_binding_set_lane(uint8_t lane, enum ace2k_led_lane_pattern pattern);

/* Blinks the unit for ace2k_identify, in command context. */
void ace2k_led_binding_identify(void);

#endif
