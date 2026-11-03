/* The dryer's automatic modes (dryer_auto.h). Each tick runs the rules in order: the insert rule,
 * the link rule of a bound cycle, the end of a cycle, the cold box, the outcome, the hold above
 * the limit, the failures, and the start. While the readings are invalid, the outcome, the hold
 * and the start wait. Events go out through the dryer's ring, from the tick only. */
#include "dryer/dryer_auto.h"
#include "dryer/dryer_internal.h" /* ace2k_dryer_emit and the state, chamber, insert tests */
#include "core/util.h"            /* ace2k_time_after, ace2k_time_since, ACE2K_EINVAL */

_Static_assert(ACE2K_DRYER_AUTO_PROFILE_C_DEFAULT == ACE2K_DRYER_SPOOL_SAFE_C,
               "the default profile is the spool rule's safe temperature");

#define SAMPLE_INTERVALS (ACE2K_DRYER_AUTO_SAMPLES - 1U)
#define MS_PER_S         1000U

/* A profile is a cycle the dryer would accept. */
static bool profile_valid(uint8_t c, uint16_t min)
{
    return (bool)(ace2k_dryer_request_range_refusal(c, min) == ACE2K_DRYER_OK);
}

static bool limits_valid(uint8_t limit, uint8_t max)
{
    return (bool)(limit >= ACE2K_DRYER_AUTO_LIMIT_MIN_PCT &&
                  limit <= ACE2K_DRYER_AUTO_LIMIT_MAX_PCT && max >= ACE2K_DRYER_AUTO_FAILURES_MIN &&
                  max <= ACE2K_DRYER_AUTO_FAILURES_MAX);
}

static void mark(struct ace2k_dryer_auto *self)
{
    self->gen++;
}

static void sanitise(struct ace2k_dryer_auto_store *s)
{
    struct ace2k_dryer_auto_store d;
    ace2k_dryer_auto_store_defaults(&d);
    if (s->mode >= ACE2K_DRYER_AUTO_MODES) {
        s->mode = d.mode;
    }
    if (s->limit_pct < ACE2K_DRYER_AUTO_LIMIT_MIN_PCT ||
        s->limit_pct > ACE2K_DRYER_AUTO_LIMIT_MAX_PCT) {
        s->limit_pct = d.limit_pct;
    }
    if (s->max_failures < ACE2K_DRYER_AUTO_FAILURES_MIN ||
        s->max_failures > ACE2K_DRYER_AUTO_FAILURES_MAX) {
        s->max_failures = d.max_failures;
    }
    if (!profile_valid(s->profile_c, s->profile_min)) {
        s->profile_c = d.profile_c;
        s->profile_min = d.profile_min;
    }
    s->flags &= ACE2K_DRYER_AUTO_STOPPED;
}

static void cold_reset(struct ace2k_dryer_auto *self)
{
    self->ring_n = 0U;
    self->ring_idx = 0U;
    self->cold = false;
    self->above = false;
    self->invalid = false;
}

void ace2k_dryer_auto_init(struct ace2k_dryer_auto *self, struct ace2k_dryer *dryer,
                           struct ace2k_dryer_auto_store *store)
{
    *self = (struct ace2k_dryer_auto){ 0 };
    self->dryer = dryer;
    self->store = store;
    sanitise(store);
    self->hold_ms = ACE2K_DRYER_AUTO_HOLD_MS;
    self->window_ms = ACE2K_DRYER_AUTO_COLD_WINDOW_MS;
    self->link_ms = ACE2K_DRYER_AUTO_LINK_MS;
    cold_reset(self);
}

int ace2k_dryer_auto_mode(struct ace2k_dryer_auto *self, uint8_t mode)
{
    struct ace2k_dryer_auto_store *s = self->store;
    if (mode >= ACE2K_DRYER_AUTO_MODES) {
        return -ACE2K_EINVAL;
    }
    bool humidity = (bool)(mode == ACE2K_DRYER_AUTO_HUMIDITY);
    if (humidity || mode != s->mode) {
        self->outcome_due = false; /* the user's choice: a pending outcome counts neither way */
    }
    bool re_enable =
        (bool)(humidity && ((s->flags & ACE2K_DRYER_AUTO_STOPPED) != 0U || s->failures != 0U));
    if (mode == s->mode && !re_enable) {
        return 0; /* the same mode again: nothing to store, and the hold runs on */
    }
    if (mode != s->mode && (self->own || self->bound)) {
        self->stop_pending = true;
    }
    s->mode = mode;
    if (humidity) {
        s->flags &= (uint8_t)~ACE2K_DRYER_AUTO_STOPPED;
        s->failures = 0U;
    }
    self->above = false; /* a new hold from now */
    mark(self);
    return 0;
}

int ace2k_dryer_auto_limits(struct ace2k_dryer_auto *self, uint8_t limit_pct, uint8_t max_failures)
{
    if (!limits_valid(limit_pct, max_failures)) {
        return -ACE2K_EINVAL;
    }
    if (limit_pct != self->store->limit_pct || max_failures != self->store->max_failures) {
        self->store->limit_pct = limit_pct;
        self->store->max_failures = max_failures;
        mark(self);
    }
    return 0;
}

int ace2k_dryer_auto_profile(struct ace2k_dryer_auto *self, uint8_t target_c, uint16_t minutes)
{
    if (!profile_valid(target_c, minutes)) {
        return -ACE2K_EINVAL;
    }
    /* A more fragile strand lowers a running automatic cycle at once. A manual cycle is the
     * user's, and a higher profile waits for the next cycle. */
    if (self->own || self->bound) {
        ace2k_dryer_lower_target(self->dryer, target_c);
    }
    if (target_c != self->store->profile_c || minutes != self->store->profile_min) {
        self->store->profile_c = target_c;
        self->store->profile_min = minutes;
        mark(self);
    }
    return 0;
}

static uint32_t timing_or(uint32_t v, uint32_t dflt)
{
    if (v == 0U) {
        return dflt;
    }
    return v < ACE2K_DRYER_AUTO_MIN_TIMING_MS ? ACE2K_DRYER_AUTO_MIN_TIMING_MS : v;
}

void ace2k_dryer_auto_timing(struct ace2k_dryer_auto *self, uint32_t hold_ms, uint32_t window_ms,
                             uint32_t link_ms)
{
    uint32_t hold = timing_or(hold_ms, ACE2K_DRYER_AUTO_HOLD_MS);
    uint32_t window = timing_or(window_ms, ACE2K_DRYER_AUTO_COLD_WINDOW_MS);
    /* The host sends the timing at every connect. Only a change restarts what it times. */
    if (window != self->window_ms) {
        self->window_ms = window;
        cold_reset(self);
    }
    if (hold != self->hold_ms) {
        self->hold_ms = hold;
        self->above = false;
    }
    self->link_ms = timing_or(link_ms, ACE2K_DRYER_AUTO_LINK_MS);
}

int ace2k_dryer_auto_timing_s(struct ace2k_dryer_auto *self, uint32_t hold_s, uint32_t window_s,
                              uint32_t link_s)
{
    const uint32_t max_s = UINT32_MAX / MS_PER_S;
    if (hold_s > max_s || window_s > max_s || link_s > max_s) {
        return -ACE2K_EINVAL;
    }
    ace2k_dryer_auto_timing(self, hold_s * MS_PER_S, window_s * MS_PER_S, link_s * MS_PER_S);
    return 0;
}

enum ace2k_dryer_refusal ace2k_dryer_auto_manual_start(struct ace2k_dryer_auto *self,
                                                       uint8_t target_c, uint16_t minutes,
                                                       uint32_t now_ms)
{
    enum ace2k_dryer_refusal r;
    if ((self->own || self->bound) && ace2k_dryer_running_state(self->dryer->state)) {
        r = ace2k_dryer_retarget(self->dryer, target_c, minutes, now_ms);
        if (r == ACE2K_DRYER_OK) {
            self->own = false;
            self->bound = false;
            self->stop_pending = false; /* the cycle is the user's now: a mode change spares it */
        }
    } else {
        r = ace2k_dryer_start(self->dryer, target_c, minutes, now_ms);
    }
    if (r == ACE2K_DRYER_OK) {
        self->outcome_due = false; /* a manual cycle: a pending outcome counts neither way */
    }
    return r;
}

enum ace2k_dryer_refusal ace2k_dryer_auto_manual_stop(struct ace2k_dryer_auto *self,
                                                      uint32_t now_ms)
{
    if (self->own && ace2k_dryer_running_state(self->dryer->state) &&
        self->store->mode == ACE2K_DRYER_AUTO_HUMIDITY) {
        self->store->mode = ACE2K_DRYER_AUTO_MANUAL;
        self->off_pending = true;
        mark(self);
    }
    self->own = false;
    self->bound = false;
    return ace2k_dryer_stop(self->dryer, now_ms);
}

enum ace2k_dryer_refusal ace2k_dryer_auto_start_bound(struct ace2k_dryer_auto *self,
                                                      uint8_t target_c, uint16_t minutes,
                                                      uint32_t now_ms)
{
    if (self->store->mode != ACE2K_DRYER_AUTO_PRINT) {
        return ACE2K_DRYER_BUSY; /* a bound cycle belongs to the print mode */
    }
    enum ace2k_dryer_refusal r = ace2k_dryer_start(self->dryer, target_c, minutes, now_ms);
    if (r == ACE2K_DRYER_OK) {
        self->bound = true;
        self->own = false;
        self->link_down = false;
    }
    return r;
}

enum ace2k_dryer_refusal ace2k_dryer_auto_stop_bound(struct ace2k_dryer_auto *self, uint32_t now_ms)
{
    if (!self->bound || !ace2k_dryer_running_state(self->dryer->state)) {
        return ACE2K_DRYER_OK; /* not the print mode's cycle: the user's, or nothing */
    }
    self->bound = false;
    return ace2k_dryer_stop(self->dryer, now_ms);
}

/* With no host, a strand at an empty lane's insert sensor lowers the stored profile, as the
 * dryer's spool rule lowers a running cycle. A removal changes nothing. */
static void insert_rule(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in)
{
    if (!self->have_prev) {
        self->insert_prev = in->insert_mask;
    }
    uint8_t fresh = ace2k_dryer_insert_fresh(&self->insert_prev, in);
    if (fresh == 0U || self->store->profile_c <= ACE2K_DRYER_SPOOL_SAFE_C) {
        return;
    }
    self->store->profile_c = ACE2K_DRYER_SPOOL_SAFE_C;
    self->store->profile_min = ACE2K_DRYER_AUTO_PROFILE_MIN_DEFAULT;
    mark(self);
}

/* A bound cycle ends once the link has been down for link_ms. */
static void link_rule(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in,
                      uint32_t now_ms)
{
    if (!self->bound || !ace2k_dryer_running_state(self->dryer->state) || in->link_ok) {
        self->link_down = false;
        return;
    }
    if (!self->link_down) {
        self->link_down = true;
        self->link_down_since_ms = now_ms;
        return;
    }
    if (ace2k_time_since(now_ms, self->link_down_since_ms) >= self->link_ms) {
        self->bound = false;
        self->link_down = false;
        (void)ace2k_dryer_stop(self->dryer, now_ms);
        ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_STOPPED_LINK, 0U);
    }
}

/* A failed cycle: counted, announced, and at the maximum the mode stops. */
static void record_failure(struct ace2k_dryer_auto *self)
{
    if (self->store->failures < UINT8_MAX) {
        self->store->failures++;
    }
    ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_FAILED, self->store->failures);
    if (self->store->failures >= self->store->max_failures &&
        (self->store->flags & ACE2K_DRYER_AUTO_STOPPED) == 0U) {
        self->store->flags |= ACE2K_DRYER_AUTO_STOPPED;
        ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_STOPPED, self->store->failures);
    }
}

/* The end of an own cycle: completed (COOLDOWN, nobody stopped it) waits for the outcome; a
 * fault counts neither way, except the lid fault, which is a failed cycle at once. A bound cycle
 * that ended clears its mark. */
static void end_rule(struct ace2k_dryer_auto *self)
{
    uint8_t st = self->dryer->state;
    /* prev_state starts IDLE, so the first tick after a reset sees no cycle end */
    if (ace2k_dryer_running_state(self->prev_state) && !ace2k_dryer_running_state(st)) {
        if (self->own && st == ACE2K_DRYER_COOLDOWN) {
            self->outcome_due = true;
        }
        if (self->own && st == ACE2K_DRYER_FAULT && self->dryer->fault == ACE2K_DRYER_FAULT_LID) {
            record_failure(self); /* the fault is the outcome: no cold reading to wait for */
            mark(self);
        }
        self->own = false;
        self->bound = false;
    }
    self->prev_state = st;
}

static bool readings_valid(const struct ace2k_dryer_inputs *in)
{
    return (bool)(ace2k_dryer_chamber_fresh(in) && in->rh_valid);
}

/* One sample every window / 30 while the dryer is idle and the readings count. The box is cold
 * when the window is full and the chamber is steady across it: it neither fell nor rose by
 * COLD_BAND or more. A warming box is not cold. Invalid readings take no sample. Once they stay
 * invalid for ACE2K_DRYER_AUTO_INVALID_MS, the window empties and the hold ends. */
static void cold_rule(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in,
                      bool valid, uint32_t now_ms)
{
    if (self->dryer->state != ACE2K_DRYER_IDLE) {
        cold_reset(self);
        self->sample_next_ms = now_ms;
        return;
    }
    if (!valid) {
        if (!self->invalid) {
            self->invalid = true;
            self->invalid_since_ms = now_ms;
        } else if (ace2k_time_since(now_ms, self->invalid_since_ms) >=
                   ACE2K_DRYER_AUTO_INVALID_MS) {
            cold_reset(self);
            self->sample_next_ms = now_ms;
        }
        return;
    }
    self->invalid = false;
    self->rh_pct10 = in->chamber_rh_pct10;
    if (!ace2k_time_after(now_ms, self->sample_next_ms) && self->ring_n > 0U) {
        return;
    }
    self->sample_next_ms = now_ms + (self->window_ms / SAMPLE_INTERVALS);
    self->ring_mc[self->ring_idx] = in->chamber_mc;
    self->ring_idx = (uint8_t)((self->ring_idx + 1U) % ACE2K_DRYER_AUTO_SAMPLES);
    if (self->ring_n < ACE2K_DRYER_AUTO_SAMPLES) {
        self->ring_n++;
    }
    if (self->ring_n < ACE2K_DRYER_AUTO_SAMPLES) {
        self->cold = false;
        return;
    }
    int32_t oldest = self->ring_mc[self->ring_idx]; /* the next slot to write is the oldest */
    int32_t change = oldest - in->chamber_mc;
    self->cold =
        (bool)(change < ACE2K_DRYER_AUTO_COLD_BAND_MC && change > -ACE2K_DRYER_AUTO_COLD_BAND_MC);
}

static uint32_t limit_pct10(const struct ace2k_dryer_auto *self)
{
    return (uint32_t)self->store->limit_pct * ACE2K_DRYER_AUTO_PCT10_PER_PCT;
}

static void outcome_rule(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in)
{
    if (!self->outcome_due || !self->cold) {
        return;
    }
    self->outcome_due = false; /* only the humidity mode holds one: a mode change drops it */
    uint8_t pct = (uint8_t)(in->chamber_rh_pct10 / ACE2K_DRYER_AUTO_PCT10_PER_PCT);
    if (in->chamber_rh_pct10 < limit_pct10(self)) {
        self->store->failures = 0U;
        ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_DRY, pct);
    } else {
        record_failure(self);
    }
    mark(self);
}

static void above_rule(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in,
                       uint32_t now_ms)
{
    /* at the limit too: an outcome there is a failure, so a start is its complement */
    bool over = (bool)(self->cold && in->chamber_rh_pct10 >= limit_pct10(self));
    if (!over) {
        self->above = false;
        return;
    }
    if (!self->above) {
        self->above = true;
        self->above_since_ms = now_ms;
    }
}

/* The host lowered the maximum to the failures already counted: the mode stops, as after a
 * failed outcome. */
static void failures_rule(struct ace2k_dryer_auto *self)
{
    struct ace2k_dryer_auto_store *s = self->store;
    if (s->mode != ACE2K_DRYER_AUTO_HUMIDITY || (s->flags & ACE2K_DRYER_AUTO_STOPPED) != 0U ||
        s->failures < s->max_failures) {
        return;
    }
    s->flags |= ACE2K_DRYER_AUTO_STOPPED;
    ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_STOPPED, s->failures);
    mark(self);
}

static void start_rule(struct ace2k_dryer_auto *self, uint32_t now_ms)
{
    const struct ace2k_dryer_auto_store *s = self->store;
    /* failures_rule has already set STOPPED when the count reached the maximum. The outcome_due
     * term is defensive: above needs a cold box, and outcome_rule takes the outcome on that
     * tick. */
    if (s->mode != ACE2K_DRYER_AUTO_HUMIDITY || (s->flags & ACE2K_DRYER_AUTO_STOPPED) != 0U ||
        self->outcome_due || !self->above || self->dryer->state != ACE2K_DRYER_IDLE ||
        ace2k_time_since(now_ms, self->above_since_ms) < self->hold_ms) {
        return;
    }
    if (ace2k_dryer_start(self->dryer, s->profile_c, s->profile_min, now_ms) != ACE2K_DRYER_OK) {
        self->above_since_ms = now_ms; /* refused: another full hold before the next try */
        return;
    }
    self->own = true;
    self->bound = false;
    self->above = false;
    ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_STARTED, s->profile_c);
}

void ace2k_dryer_auto_tick(struct ace2k_dryer_auto *self, const struct ace2k_dryer_inputs *in,
                           uint32_t now_ms)
{
    if (self->off_pending) {
        self->off_pending = false;
        ace2k_dryer_emit(self->dryer, ACE2K_DRYER_EV_AUTO_OFF, 0U);
    }
    if (self->stop_pending) {
        self->stop_pending = false;
        self->own = false;
        self->bound = false;
        (void)ace2k_dryer_stop(self->dryer, now_ms);
    }
    insert_rule(self, in);
    self->have_prev = true;
    link_rule(self, in, now_ms);
    end_rule(self);
    bool valid = readings_valid(in);
    cold_rule(self, in, valid, now_ms);
    if (valid) { /* a bad reading decides nothing and keeps the hold */
        outcome_rule(self, in);
        above_rule(self, in, now_ms);
    }
    failures_rule(self);
    if (valid) {
        start_rule(self, now_ms);
    }
}

void ace2k_dryer_auto_status(const struct ace2k_dryer_auto *self,
                             struct ace2k_dryer_auto_status *out, uint32_t now_ms)
{
    const struct ace2k_dryer_auto_store *s = self->store;
    /* Invalid readings show no cold box and no time above, so the host starts nothing on them.
     * Inside, both run on through a short glitch. */
    bool cold = (bool)(self->cold && !self->invalid);
    bool above = (bool)(self->above && !self->invalid);
    *out = (struct ace2k_dryer_auto_status){
        .mode = s->mode,
        .limit_pct = s->limit_pct,
        .max_failures = s->max_failures,
        .failures = s->failures,
        .flags = s->flags,
        .profile_c = s->profile_c,
        .profile_min = s->profile_min,
        .cold = cold,
        .own = self->own,
        .bound = self->bound,
        .rh_pct10 = self->rh_pct10,
        .above_s = above ? ace2k_time_since(now_ms, self->above_since_ms) / MS_PER_S : 0U,
    };
}

bool ace2k_dryer_auto_dirty(const struct ace2k_dryer_auto *self)
{
    return self->gen != self->stored_gen;
}

uint32_t ace2k_dryer_auto_generation(const struct ace2k_dryer_auto *self)
{
    return self->gen;
}

void ace2k_dryer_auto_stored(struct ace2k_dryer_auto *self, uint32_t generation)
{
    self->stored_gen = generation;
}
