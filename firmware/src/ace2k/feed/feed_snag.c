/* The tip snag of the feed: a strand tip thicker than the strand
 * catches the entry of the tube the buffer's plunger carries and drags the plunger to an end while
 * the path ahead is free.  Forward — a feed, a load's pull or its tag search — this lane's full is
 * watched: the motor may turn fwd_um more with the strand following and the duty within its guard;
 * then a short touch against the move frees the plunger, which must come back to rest, and the
 * move resumes for what is left.  One tolerance per mode, each announced with the snag notice; a
 * second full, a strand that lags, a plunger that does not come back is the obstacle — blocked in a
 * feed or a load's pull, in the tag search the obstacle return to the
 * parking point.  Toward the spool — a rollback, an unload — a comparator trip with the strand
 * taut is the same catch from the other side once the strand has come back freely: a short
 * forward touch, rest, the move resumes.  Split from feed.c; the seam is feed_internal.h.  Runs
 * in the feed's tick, interrupt context. */
#include "feed/feed_internal.h"

/* The strand's travel along a direction since a base, never negative. */
static uint32_t along_um(const struct ace2k_feed *self, uint8_t lane, int32_t base_um, bool reverse)
{
    int32_t travel = ace2k_lane_encoder_um(self->lane, lane) - base_um;
    int32_t along = reverse ? -travel : travel;
    return along > 0 ? (uint32_t)along : 0;
}

static uint32_t motor_since_um(const struct ace2k_feed *self, uint8_t lane, uint32_t fg_base)
{
    return ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - fg_base);
}

bool ace2k_feed_snag_active(const struct ace2k_feed *self, uint8_t lane)
{
    return self->l[lane].snag != ACE2K_FEED_SNAG_NONE;
}

/* The tolerance begins: spent for the mode, its base, the mode's odometers for the notice. */
static void begin(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->snag_used = true;
    l->fg_snag = self->lane->lane[lane].fg_raw;
    l->enc_um_snag = ace2k_lane_encoder_um(self->lane, lane);
    ace2k_feed_odometers(self, lane, &l->snag_motor_um, &l->snag_filament_um);
}

static bool in_search(const struct ace2k_feed_lane *l)
{
    if (l->mode != ACE2K_FEED_LOADING) {
        return false;
    }
    return l->sub == ACE2K_FEED_SUB_SEARCH;
}

/* The tolerance ends in a load's tag search: the return to the parking point — on the yield when
 * one is pending, else on the obstacle.  The phase ends first, or the load's tick would stay
 * gated; the tolerance stays spent. */
static void end_in_search(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    self->l[lane].snag = ACE2K_FEED_SNAG_NONE;
    ace2k_feed_load_search_return(self, lane, now_ms);
}

/* A verdict of the tolerance that is stuck: in a load's tag search, the return instead — the
 * search never ends in an error; in a feed or a load's pull,
 * blocked — the lane idle, no error; a rollback's or an unload's,
 * stuck. */
static void stuck(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (in_search(&self->l[lane])) {
        end_in_search(self, lane, now_ms);
        return;
    }
    if (ace2k_feed_block(self, lane, now_ms)) {
        return;
    }
    ace2k_feed_fail(self, lane, ACE2K_FEED_STUCK, now_ms);
}

/* The touch: the lane stopped, a short move against the mode's direction at its speed — the
 * hand's touch that put the plunger back, as measured on the unit.  A lane that refuses it fails the mode. */
static void touch(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    ace2k_feed_stop_discard(self, lane, now_ms);
    if (ace2k_lane_move(self->lane, lane, dir, self->snag.back_um, l->speed_um_s, now_ms) != 0) {
        stuck(self, lane, now_ms);
        return;
    }
    l->snag = ACE2K_FEED_SNAG_BACK;
}

/* The notice, the comparator's bases afresh, the phase over. */
static void announce(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->snag = ACE2K_FEED_SNAG_NONE;
    ace2k_feed_notice_at(self, lane, ACE2K_FEED_SNAG, l->snag_motor_um, l->snag_filament_um);
    ace2k_feed_compare_restart(self, lane);
}

static bool mode_is_reverse(uint8_t mode)
{
    if (mode == ACE2K_FEED_ROLLING_BACK) {
        return true;
    }
    return mode == ACE2K_FEED_UNLOADING;
}

/* What the mode asked, along its direction: a feed's or a rollback's length, an unload's budget. */
static uint32_t budget_um(const struct ace2k_feed_lane *l)
{
    if (l->mode == ACE2K_FEED_UNLOADING && l->length_um == 0) {
        return ACE2K_FEED_UNLOAD_MAX_UM;
    }
    return l->length_um;
}

/* Less than a move's worth left: a feed or a rollback has delivered its length, an unload has
 * spent its budget with the strand still at the mouth. */
static void finish_short(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (self->l[lane].mode == ACE2K_FEED_UNLOADING) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
        return;
    }
    ace2k_feed_finish(self, lane, ACE2K_FEED_DONE, now_ms);
}

/* The plunger is back: the notice, then the mode goes on for what is left of it on the encoder
 * from its start — the touch's own travel included, so the strand still ends where the command
 * asked. */
static void resume(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                   uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    announce(self, lane);
    if (l->mode == ACE2K_FEED_LOADING) {
        ace2k_feed_load_resume_after_snag(self, lane, in, now_ms);
        return;
    }
    bool reverse = mode_is_reverse(l->mode);
    uint32_t budget = budget_um(l);
    uint32_t done_um = along_um(self, lane, l->enc_um_start, reverse);
    if ((uint64_t)done_um + ACE2K_FEED_RETURN_MIN_UM > budget) {
        finish_short(self, lane, now_ms);
        return;
    }
    enum ace2k_lane_dir dir = reverse ? ACE2K_LANE_REVERSE : ACE2K_LANE_FORWARD;
    if (ace2k_lane_move(self->lane, lane, dir, budget - done_um, l->speed_um_s, now_ms) != 0) {
        stuck(self, lane, now_ms);
    }
}

/* The duty guard: the motor working more than duty_pct points above what its setpoint asks on a
 * free lane (the feed-forward table) is pushing against something; 0 turns it off.  The duty
 * also rises with drag from behind — a badly wound spool, docs/hardware.md "Motors" — which is
 * why it guards and does not judge, and why its value is measured.  Off by default: it did not prove effective at the bench — the lag ended every jam
 * first, and a fixed excess has no margin (≤ 9 points healthy, 10–14 a jam at its stop).  Kept for a possible future case; the better form, if ever
 * needed, is the rise since the full, which cancels a spool's drag — not built. */
static bool duty_over(const struct ace2k_feed *self, uint8_t lane)
{
    if (self->snag.duty_pct == 0) {
        return false;
    }
    uint32_t ff = ace2k_lane_feed_forward_pct(ace2k_lane_setpoint_um_s(self->lane, lane));
    return (uint32_t)ace2k_lane_duty_pct(self->lane, lane) > ff + self->snag.duty_pct;
}

/* The watch, every tick, in this order: the strand behind the motor, the duty above its guard,
 * the plunger back at rest on its own (the tip went through and nothing holds the plunger:
 * the notice, the move untouched), the tolerance spent (the touch). */
static void watch(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                  uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t motor_um = motor_since_um(self, lane, l->fg_snag);
    uint32_t strand_um = along_um(self, lane, l->enc_um_snag, false);
    if ((uint64_t)motor_um > (uint64_t)strand_um + self->snag.lag_um || duty_over(self, lane)) {
        stuck(self, lane, now_ms);
        return;
    }
    if (ace2k_feed_bit(in->rest, lane)) {
        announce(self, lane);
        if (in_search(l) && ace2k_feed_bit(in->tag_read, lane)) {
            /* read meanwhile (the tolerance held the load's tick): the search is over — back */
            ace2k_feed_load_search_return(self, lane, now_ms);
        }
        return;
    }
    if (motor_um >= self->snag.fwd_um) {
        touch(self, lane, ACE2K_LANE_REVERSE, now_ms);
    }
}

/* After the touch: the plunger at rest resumes the mode; not by rest_ms, it is held — stuck. */
static void wait_rest(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    if (ace2k_feed_bit(in->rest, lane)) {
        resume(self, lane, in, now_ms);
        return;
    }
    if (ace2k_time_since(now_ms, self->l[lane].t_snag_ms) >= self->snag.rest_ms) {
        stuck(self, lane, now_ms);
    }
}

bool ace2k_feed_snag_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag != ACE2K_FEED_SNAG_NONE && l->yield && in_search(l)) {
        /* another lane was given a start: the search yields at once, whatever the phase — never
         * forward past the parking point while another lane moves */
        end_in_search(self, lane, now_ms);
        return true;
    }
    switch (l->snag) {
    case ACE2K_FEED_SNAG_WATCH:
        watch(self, lane, in, now_ms);
        return true;
    case ACE2K_FEED_SNAG_BACK:
        return true; /* the touch runs; its result comes through on_result */
    case ACE2K_FEED_SNAG_REST:
        wait_rest(self, lane, in, now_ms);
        return true;
    default:
        break;
    }
    if (!ace2k_feed_full_ahead(self, lane, in)) {
        return false;
    }
    if (self->l[lane].snag_used) {
        stuck(self, lane, now_ms); /* the mode's one tolerance spent */
        return true;
    }
    begin(self, lane);
    self->l[lane].snag = ACE2K_FEED_SNAG_WATCH;
    return true;
}

/* The search's own move stalled or past its deadline in the watch — the strand held with the
 * motor pushing, where the lag cannot trip with no motor travel: in the search, a verdict of the
 * tolerance; elsewhere the mode's, as before. */
static bool search_move_fault(const struct ace2k_feed_lane *l, const struct ace2k_lane_result *r)
{
    if (!in_search(l)) {
        return false;
    }
    if (r->outcome == ACE2K_LANE_STALLED) {
        return true;
    }
    return r->outcome == ACE2K_LANE_TIMEOUT;
}

bool ace2k_feed_snag_on_result(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_lane_result *r, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag == ACE2K_FEED_SNAG_WATCH) {
        if (search_move_fault(l, r)) {
            stuck(self, lane, now_ms); /* the search never ends in an error: the return */
            return true;
        }
        if (r->outcome != ACE2K_LANE_DONE) {
            return false; /* a stall or a deadline in the watch is the mode's, as before */
        }
        touch(self, lane, ACE2K_LANE_REVERSE, now_ms); /* the length reached: still free it */
        return true;
    }
    if (l->snag != ACE2K_FEED_SNAG_BACK) {
        return false;
    }
    if (r->outcome == ACE2K_LANE_DONE) {
        l->snag = ACE2K_FEED_SNAG_REST;
        l->t_snag_ms = now_ms;
        return true;
    }
    if (r->outcome != ACE2K_LANE_STOPPED) {
        stuck(self, lane, now_ms); /* the touch stalled or timed out */
    }
    return true;
}

/* The reverse forgiveness: a strand that came back freely for free_um and then stood taut caught
 * the tube's entry on its way (a tip held in the head is held from
 * the move's start and comes back at most the buffer's travel). */
static bool reverse_forgivable(const struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_feed_inputs *in)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag_used || !mode_is_reverse(l->mode)) {
        return false;
    }
    if (!ace2k_feed_bit(in->pushed, lane) || !ace2k_feed_bit(in->insert, lane)) {
        return false;
    }
    return along_um(self, lane, l->enc_um_start, true) >= self->snag.free_um;
}

bool ace2k_feed_snag_reverse(struct ace2k_feed *self, uint8_t lane,
                             const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (!reverse_forgivable(self, lane, in)) {
        return false;
    }
    begin(self, lane);
    touch(self, lane, ACE2K_LANE_FORWARD, now_ms);
    return true;
}

static bool within(uint32_t value, uint32_t low, uint32_t high)
{
    if (value < low) {
        return false;
    }
    return value <= high;
}

static bool cfg_valid(const struct ace2k_feed_snag_cfg *cfg)
{
    if (!within(cfg->fwd_um, ACE2K_FEED_SNAG_FWD_MIN_UM, ACE2K_FEED_SNAG_FWD_MAX_UM) ||
        !within(cfg->lag_um, ACE2K_FEED_SNAG_LAG_MIN_UM, ACE2K_FEED_SNAG_LAG_MAX_UM)) {
        return false;
    }
    if (cfg->lag_um >= cfg->fwd_um || cfg->duty_pct > ACE2K_FEED_SNAG_DUTY_MAX_PCT) {
        return false; /* a lag the watch can never reach would make it a plain tolerance */
    }
    if (!within(cfg->back_um, ACE2K_FEED_SNAG_BACK_MIN_UM, ACE2K_FEED_SNAG_BACK_MAX_UM) ||
        !within(cfg->rest_ms, ACE2K_FEED_SNAG_REST_MIN_MS, ACE2K_FEED_SNAG_REST_MAX_MS)) {
        return false;
    }
    return within(cfg->free_um, ACE2K_FEED_SNAG_FREE_MIN_UM, ACE2K_FEED_SNAG_FREE_MAX_UM);
}

int ace2k_feed_snag_set(struct ace2k_feed *self, const struct ace2k_feed_snag_cfg *cfg)
{
    if (!cfg_valid(cfg)) {
        return -ACE2K_EINVAL;
    }
    self->snag = *cfg;
    return 0;
}
