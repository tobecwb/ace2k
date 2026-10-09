#include "lane/led.h"
#include "core/util.h"

/* Lane positions of the intro, one per step: there and back, twice, ending on lane 1. */
static const uint8_t intro_sweep[] = { 0, 1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 0 };
#define INTRO_STEPS (sizeof(intro_sweep) / sizeof(intro_sweep[0]))

static bool blink_on(uint32_t now_ms, uint32_t period_ms)
{
    /* now_ms itself is the shared clock: a wrap of the 32-bit ms counter (every 49.7 days)
     * shifts the phase once, harmless for an LED */
    return (now_ms % period_ms) < period_ms / 2U;
}

static bool lane_lit(const struct ace2k_led *self, uint8_t lane, uint32_t now_ms)
{
    switch (self->lane_pattern[lane]) {
    case ACE2K_LED_LANE_LOADED:
        return true;
    case ACE2K_LED_LANE_MOVING:
        return blink_on(now_ms, ACE2K_LED_MOVING_PERIOD_MS);
    case ACE2K_LED_LANE_ERROR:
        return blink_on(now_ms, ACE2K_LED_ERROR_PERIOD_MS);
    case ACE2K_LED_LANE_OFF:
    default:
        return false;
    }
}

static bool any_lane_pattern(const struct ace2k_led *self)
{
    for (uint8_t i = 0; i < ACE2K_LED_COUNT; i++) {
        if (self->lane_pattern[i] != ACE2K_LED_LANE_OFF) {
            return true;
        }
    }
    return false;
}

uint8_t ace2k_led_mask(const struct ace2k_led *self, uint32_t now_ms)
{
    uint32_t t = ace2k_time_since(now_ms, self->epoch_ms);
    if (self->pattern == ACE2K_LED_WAITING_HOST) {
        return (uint8_t)(ACE2K_LED_LANE1_MASK << ((t / ACE2K_LED_CHASE_STEP_MS) % ACE2K_LED_COUNT));
    }
    if (self->pattern == ACE2K_LED_INTRO && !self->intro_over && !any_lane_pattern(self)) {
        if (t < ACE2K_LED_INTRO_HOLD_MS) {
            return 0;
        }
        uint32_t step = (t - ACE2K_LED_INTRO_HOLD_MS) / ACE2K_LED_INTRO_STEP_MS;
        if (step < INTRO_STEPS) {
            return (uint8_t)(ACE2K_LED_LANE1_MASK << intro_sweep[step]);
        }
    }
    uint8_t mask = 0;
    for (uint8_t i = 0; i < ACE2K_LED_COUNT; i++) {
        if (lane_lit(self, i, now_ms)) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

void ace2k_led_tick(struct ace2k_led *self, uint32_t now_ms)
{
    uint8_t mask = ace2k_led_mask(self, now_ms);
    if (self->has_last && mask == self->last_mask) {
        return;
    }
    self->ops->set_mask(self->ctx, mask);
    self->last_mask = mask;
    self->has_last = true;
}

void ace2k_led_init(struct ace2k_led *self, const struct ace2k_led_ops *ops, void *ctx,
                    uint32_t now_ms)
{
    *self = (struct ace2k_led){ 0 }; /* every lane OFF */
    self->ops = ops;
    self->ctx = ctx;
    self->pattern = ACE2K_LED_OFF;
    self->epoch_ms = now_ms;
    self->has_last = false;
    self->last_mask = 0;
    ace2k_led_tick(self, now_ms);
}

void ace2k_led_set_pattern(struct ace2k_led *self, enum ace2k_led_pattern pattern, uint32_t now_ms)
{
    self->pattern = pattern;
    self->epoch_ms = now_ms;
    self->intro_over = false;
    ace2k_led_tick(self, now_ms);
}

void ace2k_led_set_lane(struct ace2k_led *self, uint8_t lane, enum ace2k_led_lane_pattern pattern,
                        uint32_t now_ms)
{
    if (lane >= ACE2K_LED_COUNT || self->lane_pattern[lane] == (uint8_t)pattern) {
        return;
    }
    self->lane_pattern[lane] = (uint8_t)pattern;
    if (pattern != ACE2K_LED_LANE_OFF) {
        self->intro_over = true;
    }
    ace2k_led_tick(self, now_ms);
}
