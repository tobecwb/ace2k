/* What: the four lane LEDs as named patterns.  The unit has two patterns, "waiting for the host"
 * and its intro, and a blink that identifies it.  Under them, each lane has its own: off, loaded,
 * moving, error.
 * How: the binding owns one struct ace2k_led, with an op that writes a 4-bit mask.  Bit n lights
 * lane n + 1.  The binding calls ace2k_led_tick() every tick, ace2k_led_set_pattern() on unit
 * events, and ace2k_led_set_lane() on lane events.  A steady light shows a state, and a blinking
 * light asks for a look.  Every lane blinks on one shared clock, so lanes in the same state blink
 * together.  While every lane is off, a short and slow flash of all four shows that the unit is on.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_LED_H
#define ACE2K_LED_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_LED_LANE1_MASK    0x01U
#define ACE2K_LED_ALL_MASK      0x0FU /* the four lanes: the powered-on sign */
#define ACE2K_LED_CHASE_STEP_MS 250U  /* one lane at a time, a full round per second */
#define ACE2K_LED_INTRO_HOLD_MS 300U  /* dark first: the insert sensors settle in about 30 ms */
#define ACE2K_LED_INTRO_STEP_MS 150U  /* 1→4→1 twice: 13 steps, about 2 s */
#define ACE2K_LED_INTRO_STEPS   13U
#define ACE2K_LED_INTRO_MS                                                                         \
    (ACE2K_LED_INTRO_HOLD_MS + (ACE2K_LED_INTRO_STEPS * ACE2K_LED_INTRO_STEP_MS))

/* The lane rates are initial values.  The error period divides the moving one.  So on the shared
 * clock, a moving lane and a lane in error turn on together. */
#define ACE2K_LED_MOVING_PERIOD_MS 1000U /* feeding, rolling back, unloading, loading: 1 Hz */
#define ACE2K_LED_ERROR_PERIOD_MS  200U  /* error: 5 Hz, far from the bootloader's ~17 Hz strobe */
_Static_assert(ACE2K_LED_MOVING_PERIOD_MS % ACE2K_LED_ERROR_PERIOD_MS == 0U,
               "the error period must divide the moving one");

/* The powered-on sign, with initial values.  While every lane is off, all four flash together once
 * a period.  The first flash comes a whole period after the lanes went dark, or after the intro. */
#define ACE2K_LED_IDLE_PERIOD_MS 10000U /* slow: a sign of life, nothing to look at */
#define ACE2K_LED_IDLE_ON_MS     100U   /* short: never steady, never a lane's blink */
/* The sign must never pass for another pattern.  The flash is short enough never to read as steady,
 * and long enough to catch at a glance.  The period is far slower than the 1 Hz of a moving
 * lane. */
#define ACE2K_LED_IDLE_ON_MIN_MS     60U
#define ACE2K_LED_IDLE_ON_MAX_MS     150U
#define ACE2K_LED_IDLE_PERIOD_MIN_MS 5000U
_Static_assert(ACE2K_LED_IDLE_ON_MS >= ACE2K_LED_IDLE_ON_MIN_MS &&
                   ACE2K_LED_IDLE_ON_MS <= ACE2K_LED_IDLE_ON_MAX_MS,
               "the flash must be short: never steady, never missed");
_Static_assert(ACE2K_LED_IDLE_PERIOD_MS >= ACE2K_LED_IDLE_PERIOD_MIN_MS,
               "the sign must be slow: nothing to look at");

/* ace2k_identify: all four together, on and off at 2 Hz, for 10 s, over every other pattern.  It
 * must never pass for another one: the chase lights one lane at a time, the error blinks at 5 Hz,
 * a moving lane at 1 Hz, the bootloader strobes at about 17 Hz, and the sign is a 100 ms flash. */
#define ACE2K_LED_IDENTIFY_MS        10000U
#define ACE2K_LED_IDENTIFY_PERIOD_MS 500U
_Static_assert(ACE2K_LED_IDENTIFY_PERIOD_MS != ACE2K_LED_MOVING_PERIOD_MS &&
                   ACE2K_LED_IDENTIFY_PERIOD_MS != ACE2K_LED_ERROR_PERIOD_MS,
               "the identify blink must differ from a lane's");

enum ace2k_led_pattern {
    ACE2K_LED_OFF,
    ACE2K_LED_WAITING_HOST, /* unproven image: a chase, the sign of life */
    ACE2K_LED_INTRO,        /* proven image: dark, then one sweep while every lane is empty */
};

enum ace2k_led_lane_pattern {
    ACE2K_LED_LANE_OFF,    /* no filament, idle */
    ACE2K_LED_LANE_LOADED, /* filament present: idle, assisting or following; steady */
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
    uint32_t epoch_ms;     /* when the current pattern started */
    bool intro_over;       /* the intro played to its end, or a lane was lit since it began */
    uint32_t lanes_off_ms; /* when the last lane went off; the powered-on sign counts from here */
    uint8_t lane_pattern[ACE2K_LANE_COUNT]; /* per lane, under the unit pattern */
    uint8_t last_mask;
    bool has_last;
    bool identifying;     /* ace2k_identify: the four blink, from identify_ms */
    uint32_t identify_ms; /* when the last ace2k_identify came */
};

/* Sets the pattern OFF and every lane OFF, and writes the mask once. */
void ace2k_led_init(struct ace2k_led *self, const struct ace2k_led_ops *ops, void *ctx,
                    uint32_t now_ms);

/* Restarts the clock of the pattern at now_ms, and writes the new mask at once, even for the
 * pattern already set.  An INTRO already set is ignored, though.  So the intro plays once however
 * often it is asked for, and never while a lane shows something.  It never runs between the steps
 * of the tick or of set_lane, because the binding calls it with interrupts masked. */
void ace2k_led_set_pattern(struct ace2k_led *self, enum ace2k_led_pattern pattern, uint32_t now_ms);

/* Sets a lane's own pattern.  The unit's WAITING_HOST wins over every lane.  Any lane other than
 * OFF ends the intro for good.  The last lane back to OFF starts the period of the powered-on sign.
 * A lane out of range, or a pattern already set, is ignored.  It writes the new mask at once. */
void ace2k_led_set_lane(struct ace2k_led *self, uint8_t lane, enum ace2k_led_lane_pattern pattern,
                        uint32_t now_ms);

/* Blinks all four lanes together for ACE2K_LED_IDENTIFY_MS, over every pattern, and writes the
 * mask at once.  A second call starts the time again.  Then the patterns show as before: their
 * clocks ran on underneath. */
void ace2k_led_identify(struct ace2k_led *self, uint32_t now_ms);

/* Writes the mask through the op only when it changed since the last write. */
void ace2k_led_tick(struct ace2k_led *self, uint32_t now_ms);

/* Returns the mask that the patterns show at now_ms.  Pure.  For the tests. */
uint8_t ace2k_led_mask(const struct ace2k_led *self, uint32_t now_ms);

#endif
