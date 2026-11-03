#include "lane/led.h"
#include "core/util.h"

/* The lane positions of the intro, one per step: there and back, twice, ending on lane 1. */
static const uint8_t intro_sweep[] = { 0, 1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 0 };
_Static_assert(sizeof(intro_sweep) / sizeof(intro_sweep[0]) == ACE2K_LED_INTRO_STEPS,
               "the sweep table and its length must agree");

static bool blink_on(uint32_t now_ms, uint32_t period_ms)
{
    /* now_ms itself is the shared clock.  A wrap of the 32-bit ms counter, every 49.7 days, shifts
     * the phase once.  That is harmless for an LED. */
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
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (self->lane_pattern[i] != ACE2K_LED_LANE_OFF) {
            return true;
        }
    }
    return false;
}

static bool intro_pending(const struct ace2k_led *self)
{
    return (bool)(self->pattern == ACE2K_LED_INTRO && !self->intro_over);
}

/* Where the sign counts its periods from.  It is the last lane-off.  While the intro is pending, it
 * is the later of that and the end of the intro.  An intro resumed by an emptying plays first. */
static uint32_t sign_from(const struct ace2k_led *self)
{
    if (intro_pending(self)) {
        uint32_t intro_end = self->epoch_ms + ACE2K_LED_INTRO_MS;
        if (!ace2k_time_after(self->lanes_off_ms, intro_end)) {
            return intro_end;
        }
    }
    return self->lanes_off_ms;
}

static bool identify_on(const struct ace2k_led *self, uint32_t now_ms)
{
    return (bool)(self->identifying &&
                  ace2k_time_since(now_ms, self->identify_ms) < ACE2K_LED_IDENTIFY_MS);
}

/* The mask at now_ms, given whether any lane shows a pattern of its own. */
static uint8_t mask_at(const struct ace2k_led *self, uint32_t now_ms, bool lit)
{
    uint32_t t = ace2k_time_since(now_ms, self->epoch_ms);
    if (self->pattern == ACE2K_LED_WAITING_HOST) {
        return (uint8_t)(ACE2K_LED_LANE1_MASK
                         << ((t / ACE2K_LED_CHASE_STEP_MS) % ACE2K_LANE_COUNT));
    }
    if (intro_pending(self) && !lit) {
        if (t < ACE2K_LED_INTRO_HOLD_MS) {
            return 0;
        }
        uint32_t step = (t - ACE2K_LED_INTRO_HOLD_MS) / ACE2K_LED_INTRO_STEP_MS;
        if (step < ACE2K_LED_INTRO_STEPS) {
            return (uint8_t)(ACE2K_LED_LANE1_MASK << intro_sweep[step]);
        }
    }
    if (!lit) {
        /* The powered-on sign runs on its own phase.  The first flash comes a whole period after
         * the lanes went dark, never at once, because it would read as a reaction to that. */
        uint32_t idle = ace2k_time_since(now_ms, sign_from(self));
        if (idle < ACE2K_LED_IDLE_PERIOD_MS ||
            (idle % ACE2K_LED_IDLE_PERIOD_MS) >= ACE2K_LED_IDLE_ON_MS) {
            return 0;
        }
        return ACE2K_LED_ALL_MASK;
    }
    uint8_t mask = 0;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (lane_lit(self, i, now_ms)) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

/* Keeps the clocks clear of the wrap of the counter.  An intro whose time is up is marked over, so
 * it can never replay from a wrap.  Its end is folded into the start of the sign.  The start of the
 * sign moves on by whole periods, so its count stays under two periods and its phase holds. */
static void settle(struct ace2k_led *self, uint32_t now_ms, bool lit)
{
    if (self->identifying && !identify_on(self, now_ms)) {
        self->identifying = false; /* over: it can never come back from a wrap */
    }
    if (intro_pending(self) && ace2k_time_since(now_ms, self->epoch_ms) >= ACE2K_LED_INTRO_MS) {
        if (!lit) {
            self->lanes_off_ms = sign_from(self);
        }
        self->intro_over = true;
    }
    if (lit || intro_pending(self)) {
        return;
    }
    uint32_t idle = ace2k_time_since(now_ms, self->lanes_off_ms);
    if (idle >= 2U * ACE2K_LED_IDLE_PERIOD_MS) {
        self->lanes_off_ms += ((idle / ACE2K_LED_IDLE_PERIOD_MS) - 1U) * ACE2K_LED_IDLE_PERIOD_MS;
    }
}

/* The mask now: the identify blink over everything, else the patterns. */
static uint8_t shown_at(const struct ace2k_led *self, uint32_t now_ms, bool lit)
{
    if (identify_on(self, now_ms)) {
        bool on =
            blink_on(ace2k_time_since(now_ms, self->identify_ms), ACE2K_LED_IDENTIFY_PERIOD_MS);
        return on ? (uint8_t)ACE2K_LED_ALL_MASK : 0U;
    }
    return mask_at(self, now_ms, lit);
}

uint8_t ace2k_led_mask(const struct ace2k_led *self, uint32_t now_ms)
{
    return shown_at(self, now_ms, any_lane_pattern(self));
}

void ace2k_led_tick(struct ace2k_led *self, uint32_t now_ms)
{
    bool lit = any_lane_pattern(self); /* settle() changes no lane */
    settle(self, now_ms, lit);
    uint8_t mask = shown_at(self, now_ms, lit);
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
    self->lanes_off_ms = now_ms;
    self->has_last = false;
    self->last_mask = 0;
    ace2k_led_tick(self, now_ms);
}

void ace2k_led_set_pattern(struct ace2k_led *self, enum ace2k_led_pattern pattern, uint32_t now_ms)
{
    if (pattern == ACE2K_LED_INTRO && self->pattern == ACE2K_LED_INTRO) {
        return; /* a proven image gets INTRO at every version query; the intro plays once */
    }
    self->pattern = pattern;
    self->epoch_ms = now_ms;
    self->intro_over = false;
    ace2k_led_tick(self, now_ms);
}

void ace2k_led_set_lane(struct ace2k_led *self, uint8_t lane, enum ace2k_led_lane_pattern pattern,
                        uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || self->lane_pattern[lane] == (uint8_t)pattern) {
        return;
    }
    self->lane_pattern[lane] = (uint8_t)pattern;
    if (pattern != ACE2K_LED_LANE_OFF) {
        self->intro_over = true;
    } else if (!any_lane_pattern(self)) {
        self->lanes_off_ms = now_ms; /* the last lane went dark */
    }
    ace2k_led_tick(self, now_ms);
}

void ace2k_led_identify(struct ace2k_led *self, uint32_t now_ms)
{
    self->identifying = true;
    self->identify_ms = now_ms;
    ace2k_led_tick(self, now_ms);
}
