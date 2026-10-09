/* What: the four lane LEDs as named patterns — the unit's "waiting for the host" and its intro,
 * and under them each lane's own: off, loaded, moving, error.
 * How: the binding owns one struct ace2k_led with an op that writes a 4-bit mask (bit n = lane
 * n + 1 lit); it calls ace2k_led_tick() every tick, ace2k_led_set_pattern() on unit events and
 * ace2k_led_set_lane() on lane events.  Steady means a state, blinking asks for a look; every
 * lane blinks on one shared clock, so lanes in the same state blink together.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_LED_H
#define ACE2K_LED_H
#include <stdbool.h>
#include <stdint.h>

#define ACE2K_LED_COUNT         4
#define ACE2K_LED_LANE1_MASK    0x01U
#define ACE2K_LED_CHASE_STEP_MS 250U /* one lane at a time, a full round per second */
#define ACE2K_LED_INTRO_HOLD_MS 300U /* dark first: the insert sensors settle in ~30 ms */
#define ACE2K_LED_INTRO_STEP_MS 150U /* 1→4→1 twice: 13 steps, about 2 s */

/* The lane rates are initial values.  The error period divides the moving one, so on the shared
 * clock a moving lane and an erroring one turn on together. */
#define ACE2K_LED_MOVING_PERIOD_MS 1000U /* feeding, rolling back, unloading, loading: 1 Hz */
#define ACE2K_LED_ERROR_PERIOD_MS  200U  /* error: 5 Hz — far from the bootloader's ~17 Hz strobe */
_Static_assert(ACE2K_LED_MOVING_PERIOD_MS % ACE2K_LED_ERROR_PERIOD_MS == 0U,
               "the error period must divide the moving one");

enum ace2k_led_pattern {
    ACE2K_LED_OFF,
    ACE2K_LED_WAITING_HOST, /* unproven image: chase, the bench's sign of life */
    ACE2K_LED_INTRO,        /* proven image: dark, then one sweep while every lane is empty */
};

enum ace2k_led_lane_pattern {
    ACE2K_LED_LANE_OFF,    /* no filament, idle */
    ACE2K_LED_LANE_LOADED, /* filament present: idle, assisting or following — steady */
    ACE2K_LED_LANE_MOVING,
    ACE2K_LED_LANE_ERROR,
};

struct ace2k_led_ops {
    void (*set_mask)(void *ctx, uint8_t on_mask);
};

struct ace2k_led {
    const struct ace2k_led_ops *ops;
    void *ctx;
    enum ace2k_led_pattern pattern;
    uint32_t epoch_ms; /* when the current pattern started */
    bool intro_over;   /* a lane was set to something other than OFF since the intro began */
    uint8_t lane_pattern[ACE2K_LED_COUNT]; /* per lane, under the unit pattern */
    uint8_t last_mask;
    bool has_last;
};

/* Pattern OFF, every lane OFF; writes the mask once. */
void ace2k_led_init(struct ace2k_led *self, const struct ace2k_led_ops *ops, void *ctx,
                    uint32_t now_ms);

/* Restarts the pattern's clock at now_ms and writes the new mask at once; an INTRO sweeps again,
 * never while a lane shows something. */
void ace2k_led_set_pattern(struct ace2k_led *self, enum ace2k_led_pattern pattern, uint32_t now_ms);

/* The lane's own pattern; the unit's WAITING_HOST wins over every lane, and any lane other than
 * OFF ends the intro for good.  A lane out of range or a pattern already set is ignored.
 * Writes the new mask at once. */
void ace2k_led_set_lane(struct ace2k_led *self, uint8_t lane, enum ace2k_led_lane_pattern pattern,
                        uint32_t now_ms);

/* Writes the mask through the op only when it changed since the last write. */
void ace2k_led_tick(struct ace2k_led *self, uint32_t now_ms);

/* The mask the patterns show at now_ms.  Pure. */
uint8_t ace2k_led_mask(const struct ace2k_led *self, uint32_t now_ms);

#endif
