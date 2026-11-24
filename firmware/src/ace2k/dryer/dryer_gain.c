/* What: the heater's rate, learned per unit. The response rule (dryer_protect.c) judges the rise
 * of each window against this rate times the duty that drove it. A unit with a stored rate judges
 * from the first full window. A unit without one only measures over the first GAIN_LEARN of
 * HEATING, then judges with the largest ratio it saw, with the margin, bounded by the floor and
 * the maximum of dryer_log.h. A clean cycle stores that rate in the log, once: HEATING reached the
 * drive target, ran GAIN_CLEAN more with no fault, and from that first reach on the sides never
 * drifted further apart than GAIN_SIDES. The gap of the warm-up before the reach does not count:
 * it is the heater ramping, and a blocked duct shows after the reach. A fault drops a candidate
 * not yet on the page, never a stored rate. Only the forget (or a reflash) clears a stored rate.
 * How: plain functions on struct ace2k_dryer, called from dryer.c (init, the entry into HEATING,
 * every HEATING tick, a fault), from dryer_protect.c (the windows), from dryer_log.c (a store of
 * the page) and from the command layer (the forget, the query). Integers only.
 * Depends on: dryer.h, dryer_internal.h. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

#define GAIN_PCT 100

void ace2k_dryer_gain_init(struct ace2k_dryer *self)
{
    struct ace2k_dryer_gain *g = &self->gain;
    *g = (struct ace2k_dryer_gain){ 0 };
    g->rate = self->log->gain_mc_s_per_pct;
    g->source = g->rate != 0U ? ACE2K_DRYER_GAIN_STORED : ACE2K_DRYER_GAIN_NONE;
}

void ace2k_dryer_gain_begin(struct ace2k_dryer *self)
{
    struct ace2k_dryer_gain *g = &self->gain;
    bool pending = g->pending;
    uint32_t pending_gen = g->pending_gen;
    *g = (struct ace2k_dryer_gain){ 0 };
    g->pending = pending;
    g->pending_gen = pending_gen;
    if (self->log->gain_mc_s_per_pct != 0U) {
        g->rate = self->log->gain_mc_s_per_pct;
        g->source = ACE2K_DRYER_GAIN_STORED;
        return;
    }
    g->source = ACE2K_DRYER_GAIN_LEARNING;
    g->sides_ok = true;
}

uint8_t ace2k_dryer_gain_rate(const struct ace2k_dryer *self)
{
    return self->gain.rate;
}

void ace2k_dryer_gain_learn(struct ace2k_dryer *self, int32_t rise_mc, int32_t duty_pct_s)
{
    struct ace2k_dryer_gain *g = &self->gain;
    if (duty_pct_s < ACE2K_DRYER_GAIN_MIN_DUTY_PCT_S || rise_mc <= 0) {
        return; /* too little heat went in for the ratio to mean anything */
    }
    g->counted = true;
    int64_t num = (int64_t)rise_mc * ACE2K_DRYER_GAIN_MARGIN_PCT;
    int64_t den = (int64_t)duty_pct_s * GAIN_PCT;
    int64_t ratio = (num + den - 1) / den; /* rounded up */
    if (ratio > UINT16_MAX) {
        ratio = UINT16_MAX;
    }
    if (ratio > g->best) {
        g->best = (uint16_t)ratio;
    }
}

void ace2k_dryer_gain_finish(struct ace2k_dryer *self)
{
    struct ace2k_dryer_gain *g = &self->gain;
    g->source = ACE2K_DRYER_GAIN_LEARNED;
    if (!g->counted) {
        g->rate = ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT; /* nothing measured: the maximum, unstored */
        g->capped = true;
        return;
    }
    uint16_t rate = g->best;
    if (rate < ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT) {
        rate = ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT;
    }
    if (rate > ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT) {
        rate = ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT;
    }
    g->rate = (uint8_t)rate;
    g->capped = g->best >= ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT;
}

/* The first reach is followed from the entry into HEATING, learning included, and the sides from
 * that reach on. The store waits for the learned rate. */
void ace2k_dryer_gain_track(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                            uint32_t now_ms)
{
    struct ace2k_dryer_gain *g = &self->gain;
    bool learns =
        (bool)(g->source == ACE2K_DRYER_GAIN_LEARNING || g->source == ACE2K_DRYER_GAIN_LEARNED);
    /* A rate in the log, stored or still pending, ends the tracking: the store happens once. */
    if (!learns || g->capped || self->log->gain_mc_s_per_pct != 0U) {
        return;
    }
    if (!g->reached && ace2k_dryer_hotter_mc(in) >= ace2k_dryer_drive_mc(self)) {
        g->reached = true;
        g->reached_ms = now_ms;
    }
    if (!g->reached) {
        return; /* the warm-up: its side gap does not count */
    }
    if (ace2k_abs_i32(in->ntc_left_mc - in->ntc_right_mc) > (uint32_t)ACE2K_DRYER_GAIN_SIDES_MC) {
        g->sides_ok = false; /* for good, this cycle */
    }
    if (g->source != ACE2K_DRYER_GAIN_LEARNED || !g->sides_ok ||
        ace2k_time_since(now_ms, g->reached_ms) < ACE2K_DRYER_GAIN_CLEAN_MS) {
        return;
    }
    self->log->gain_mc_s_per_pct = g->rate;
    ace2k_dryer_log_mark(self);
    g->pending = true;
    g->pending_gen = self->log_gen;
}

void ace2k_dryer_gain_fault(struct ace2k_dryer *self)
{
    struct ace2k_dryer_gain *g = &self->gain;
    if (!g->pending) {
        return;
    }
    self->log->gain_mc_s_per_pct = 0U;
    g->pending = false;
    if (g->source == ACE2K_DRYER_GAIN_STORED) {
        g->source = ACE2K_DRYER_GAIN_LEARNED; /* a new cycle took the unstored candidate */
    }
    ace2k_dryer_log_mark(self);
}

enum ace2k_dryer_refusal ace2k_dryer_gain_forget(struct ace2k_dryer *self)
{
    if (ace2k_dryer_running_state(self->state)) {
        return ACE2K_DRYER_BUSY;
    }
    struct ace2k_dryer_gain *g = &self->gain;
    self->log->gain_mc_s_per_pct = 0U;
    g->pending = false;
    g->rate = 0U;
    g->source = ACE2K_DRYER_GAIN_NONE;
    ace2k_dryer_log_mark(self);
    return ACE2K_DRYER_OK;
}

void ace2k_dryer_gain_get(const struct ace2k_dryer *self, uint16_t *gain, uint8_t *source)
{
    *gain = self->gain.rate;
    *source = self->gain.source;
}
