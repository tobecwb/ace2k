/* The feed's tip snag.  The tip of a filament can be thicker than the filament.  It catches the
 * entry of the tube that the buffer's plunger carries, and drags the plunger to an end while the
 * path ahead is free.
 * Forward without a free tip, this lane's full is watched.  That covers a feed that the host
 * started without a free tip, and the read with motion, whose filament may sit in an extruder.  The
 * motor may turn fwd_um more, with the filament following and the duty within its guard.  Then a
 * short touch against the move frees the plunger.  The plunger must come back to rest, and the move
 * resumes for what is left.  There is one tolerance per mode, announced with the snag notice.  A
 * second full, a filament that lags, or a plunger that does not come back is the obstacle.  It ends
 * blocked in a feed, and in the obstacle return in the search of the read.
 * With a free tip, the full is judged by the filament alone, with no touch, no wait and no limit on
 * fulls.  That covers a feed that the host started with free_tip, and a load from the pull of the
 * bay or from its search.  The filament falling lag_um behind the motor within a window of fwd_um
 * is the obstacle.  It ends blocked in a feed or in the pull of a load, and in the tag search it is
 * the obstacle return to the parking point.  A window passed, or the plunger back at rest, proves
 * the resistance, announced once.
 * Toward the spool, in a rollback or an unload, a comparator trip with the filament taut is the
 * same catch from the other side, once the filament has come back freely.  A short forward touch
 * follows, then rest, and the move resumes.  This happens once a mode.  In a rollback with a free
 * tip, it happens at every catch that is free_um past the last one.
 * This was split from feed.c.  The seam is feed_internal.h.  It runs in the feed's tick, in
 * interrupt context. */
#include "feed/feed_internal.h"

bool ace2k_feed_snag_active(const struct ace2k_feed *self, uint8_t lane)
{
    uint8_t phase = self->l[lane].snag;
    return (bool)(phase != ACE2K_FEED_SNAG_NONE && phase != ACE2K_FEED_SNAG_FREE_TIP);
}

/* The snag notice, once a mode, with the odometers of the catch that begin() last recorded.  Those
 * are the odometers of the watch and of the touch, of the first window of the free phase, or of the
 * first catch of a rollback with a free tip. */
static void notice_once(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag_sent) {
        return;
    }
    l->snag_sent = true;
    ace2k_feed_notice_at(self, lane, ACE2K_FEED_SNAG, l->snag_motor_um, l->snag_filament_um);
}

/* A tolerance begins.  The mode's one tolerance is spent.  In a rollback with a free tip, a catch
 * is forgiven instead, and the next must come back free_um past the base of this one.  The base is
 * taken, and so are the mode's odometers for the notice. */
static void begin(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->snag_used = true;
    l->fg_snag_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->enc_um_snag = ace2k_lane_encoder_um(self->lane, lane);
    ace2k_feed_odometers(self, lane, &l->snag_motor_um, &l->snag_filament_um);
}

static bool in_search(const struct ace2k_feed_lane *l)
{
    return (bool)(l->mode == ACE2K_FEED_LOADING && l->sub == ACE2K_FEED_SUB_SEARCH);
}

/* The watch or the free phase ends in the tag search of a load.  The load returns to the parking
 * point, or to the start of the read.  It does so on the yield when one is pending, else on the
 * obstacle.  The phase ends first, or the tick of the load would stay held behind the watch.  The
 * first full of the mode stays recorded. */
static void end_in_search(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    self->l[lane].snag = ACE2K_FEED_SNAG_NONE;
    ace2k_feed_load_search_return(self, lane, now_ms);
}

/* A verdict of the tolerance that is stuck.
 * - In the tag search of a load, it is the return instead, because the search never ends in an
 *   error.
 * - In a feed or in the pull of a load, it is blocked, with the lane idle and no error.
 * - In a rollback or an unload, it is stuck. */
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

/* The touch.  The lane stops, then makes a short move against the mode's direction.  It is the
 * touch of the hand that put the plunger back, as measured on the unit.  It is asked at the mode's
 * speed.  A touch no longer than the lane's landing (lane.h), the default 5 mm among them, runs
 * whole at the floor speed.  A lane that refuses it fails the mode. */
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

/* The notice, once a mode, although a rollback with a free tip may forgive several catches.  The
 * comparator's bases are taken afresh, and the phase is over. */
static void announce(struct ace2k_feed *self, uint8_t lane)
{
    self->l[lane].snag = ACE2K_FEED_SNAG_NONE;
    notice_once(self, lane);
    ace2k_feed_compare_restart(self, lane);
}

/* What the mode asked for, along its direction: the length of a feed or a rollback, or the budget
 * of an unload. */
static uint32_t budget_um(const struct ace2k_feed_lane *l)
{
    if (l->mode == ACE2K_FEED_UNLOADING && l->length_um == 0) {
        return ACE2K_FEED_UNLOAD_MAX_UM;
    }
    return l->length_um;
}

/* Less than one move is left.  A feed or a rollback has delivered its length, or an unload has
 * spent its budget with the filament still at the bay entrance. */
static void finish_short(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (self->l[lane].mode == ACE2K_FEED_UNLOADING) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
        return;
    }
    ace2k_feed_finish(self, lane, ACE2K_FEED_DONE, now_ms);
}

/* The plunger is back.  The notice goes out, then the mode goes on for what is left of it on the
 * encoder from its start.  The touch's own travel is included, so the filament still ends where the
 * command asked.  In a load, only the read with motion makes the touch, because a load from the bay
 * has a free tip.  There the search goes on for what is left of its reach (feed_load.c). */
static void resume(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                   uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    announce(self, lane);
    if (l->mode == ACE2K_FEED_LOADING) {
        ace2k_feed_load_resume_after_snag(self, lane, in, now_ms);
        return;
    }
    bool reverse = ace2k_feed_mode_rewinds(l->mode);
    uint32_t budget = budget_um(l);
    uint32_t done_um = ace2k_feed_strand_along_um(self, lane, l->enc_um_start, reverse);
    if ((uint64_t)done_um + ACE2K_FEED_RETURN_MIN_UM > budget) {
        finish_short(self, lane, now_ms);
        return;
    }
    enum ace2k_lane_dir dir = reverse ? ACE2K_LANE_REVERSE : ACE2K_LANE_FORWARD;
    if (ace2k_lane_move(self->lane, lane, dir, budget - done_um, l->speed_um_s, now_ms) != 0) {
        stuck(self, lane, now_ms);
    }
}

/* The duty guard.  A motor that works more than duty_pct points above what its setpoint asks on a
 * free lane, by the feed-forward table, is pushing against something.  0 turns the guard off.  The
 * duty also rises with drag from behind, such as a badly wound spool (docs/hardware.md "Motors").
 * That is why it guards and does not judge, and why its value is measured.  It is off by default,
 * because it did not prove effective in tests on the unit.  The lag ended every jam first, and a
 * fixed excess has no margin: 9 points or less when healthy, 10–14 for a jam at its stop.  It is
 * kept for a possible future case.  If it is ever needed, the better form is the rise since the
 * full, which cancels the drag of a spool.  That form is not built. */
static bool duty_over(const struct ace2k_feed *self, uint8_t lane)
{
    if (self->snag.duty_pct == 0) {
        return false;
    }
    uint32_t ff = ace2k_lane_feed_forward_pct(ace2k_lane_setpoint_um_s(self->lane, lane));
    return (uint32_t)ace2k_lane_duty_pct(self->lane, lane) > ff + self->snag.duty_pct;
}

/* The watch, every tick, in this order.
 * 1. The filament behind the motor.
 * 2. The duty above its guard.
 * 3. The plunger back at rest on its own.  The tip went through and nothing holds the plunger, so
 *    the notice goes out and the move is left untouched.
 * 4. The tolerance spent, which starts the touch. */
static void watch(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                  uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t motor_um = ace2k_feed_motor_since_um(self, lane, l->fg_snag_pulses);
    uint32_t strand_um = ace2k_feed_strand_along_um(self, lane, l->enc_um_snag, false);
    if ((uint64_t)motor_um > (uint64_t)strand_um + self->snag.lag_um || duty_over(self, lane)) {
        stuck(self, lane, now_ms);
        return;
    }
    if (ace2k_feed_bit(in->rest, lane)) {
        announce(self, lane);
        if (in_search(l) && ace2k_feed_bit(in->tag_read, lane)) {
            /* The tag was read meanwhile, while the watch held the load's tick.  The search is
             * over, so go back. */
            ace2k_feed_load_search_return(self, lane, now_ms);
        }
        return;
    }
    if (motor_um >= self->snag.fwd_um) {
        touch(self, lane, ACE2K_LANE_REVERSE, now_ms);
    }
}

/* After the touch, the plunger at rest resumes the mode.  If it is not at rest by rest_ms, the tip
 * is held, and that is stuck. */
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

/* The free phase's window starts afresh at the counts of this tick. */
static void free_base(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->fg_snag_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->enc_um_snag = ace2k_lane_encoder_um(self->lane, lane);
}

/* A full with a free tip.  The first full of the mode records where the plunger caught, for the
 * odometers of the notice (begin).  Every full takes a fresh base. */
static void free_begin(struct ace2k_feed *self, uint8_t lane)
{
    if (!self->l[lane].snag_used) {
        begin(self, lane);
    } else {
        free_base(self, lane);
    }
    self->l[lane].snag = ACE2K_FEED_SNAG_FREE_TIP;
}

/* The free phase, every tick.
 * - The push no longer running forward ends it.
 * - The plunger back at rest ends it, with the notice.  In the search, a tag read meanwhile turns
 *   the load back.
 * - The filament lag_um behind the motor since the base is the obstacle.
 * - fwd_um of motor with the filament following gives the notice and a new base.
 * It returns true when the lag ended the push, or when a tag read in the search turned the load
 * back.  The comparator judges every other tick. */
static bool free_watch(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                       uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_pushing_ahead(self, lane)) {
        l->snag = ACE2K_FEED_SNAG_NONE;
        return false;
    }
    if (ace2k_feed_bit(in->rest, lane)) {
        notice_once(self, lane); /* the plunger back at rest: the resistance proved */
        l->snag = ACE2K_FEED_SNAG_NONE;
        if (in_search(l) && ace2k_feed_bit(in->tag_read, lane)) {
            ace2k_feed_load_search_return(self, lane, now_ms);
            return true;
        }
        return false;
    }
    uint32_t motor_um = ace2k_feed_motor_since_um(self, lane, l->fg_snag_pulses);
    uint32_t strand_um = ace2k_feed_strand_along_um(self, lane, l->enc_um_snag, false);
    if ((uint64_t)motor_um > (uint64_t)strand_um + self->snag.lag_um) {
        stuck(self, lane, now_ms);
        return true;
    }
    if (motor_um >= self->snag.fwd_um) {
        notice_once(self, lane); /* a window passed with the filament following */
        free_base(self, lane);
    }
    return false;
}

bool ace2k_feed_snag_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag != ACE2K_FEED_SNAG_NONE && l->yield && in_search(l)) {
        /* Another lane was given a start, so the search yields at once.  It never goes forward past
         * the parking point while another lane moves.  In the watch, the touch or the wait of the
         * read with motion, the load's tick is held, and this is the yield.  In the free phase, the
         * tick of the load yields first, earlier in the tick.  The phase then ends because the push
         * no longer runs forward.  This code is kept in case that order changes. */
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
    case ACE2K_FEED_SNAG_FREE_TIP:
        return free_watch(self, lane, in, now_ms);
    default:
        break;
    }
    if (!ace2k_feed_full_ahead(self, lane, in)) {
        return false;
    }
    /* free_tip: a feed that the host started with one, or a load from the bay.  Never the read with
     * motion, whose filament may sit in an extruder (feed_load.c). */
    if (self->l[lane].free_tip) {
        free_begin(self, lane);
        return false; /* the comparator judges this tick too */
    }
    if (self->l[lane].snag_used) {
        stuck(self, lane, now_ms); /* the mode's one tolerance spent */
        return true;
    }
    begin(self, lane);
    self->l[lane].snag = ACE2K_FEED_SNAG_WATCH;
    return true;
}

/* The own move of the search stalled, or passed its deadline, in the watch or the free phase.  The
 * filament is held with the motor pushing, and the lag cannot trip with no motor travel.  In the
 * search, this is a verdict of the tolerance.  Elsewhere it belongs to the mode, as before. */
static bool search_move_fault(const struct ace2k_feed_lane *l, const struct ace2k_lane_result *r)
{
    return (bool)(in_search(l) &&
                  (r->outcome == ACE2K_LANE_STALLED || r->outcome == ACE2K_LANE_TIMEOUT));
}

bool ace2k_feed_snag_on_result(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_lane_result *r, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->snag == ACE2K_FEED_SNAG_FREE_TIP) {
        if (search_move_fault(l, r)) {
            stuck(self, lane, now_ms); /* the search never ends in an error: the return */
            return true;
        }
        l->snag = ACE2K_FEED_SNAG_NONE;
        return false; /* the mode's own result, as outside the phase */
    }
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

/* The reverse forgiveness.  A filament that came back freely for free_um and then stood taut caught
 * the entry of the tube on its way.  A tip held in the head is held from the move's start, and
 * comes back at most the buffer's travel.  This happens once a mode.  In a rollback with a free
 * tip, it happens again at every catch whose filament came back free_um past the last forgiven one.
 * So each forgiven catch is free_um of progress, and a tip held from the start is never
 * forgiven. */
static bool reverse_forgivable(const struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_feed_inputs *in)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_mode_rewinds(l->mode) || (l->snag_used && !l->free_tip)) {
        return false;
    }
    if (!ace2k_feed_bit(in->pushed, lane) || !ace2k_feed_bit(in->insert, lane)) {
        return false;
    }
    int32_t base_um = l->snag_used ? l->enc_um_snag : l->enc_um_start;
    return ace2k_feed_strand_along_um(self, lane, base_um, true) >= self->snag.free_um;
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

static bool cfg_valid(const struct ace2k_feed_snag_cfg *cfg)
{
    if (!ace2k_within_u32(cfg->fwd_um, ACE2K_FEED_SNAG_FWD_MIN_UM, ACE2K_FEED_SNAG_FWD_MAX_UM) ||
        !ace2k_within_u32(cfg->lag_um, ACE2K_FEED_SNAG_LAG_MIN_UM, ACE2K_FEED_SNAG_LAG_MAX_UM)) {
        return false;
    }
    if (cfg->lag_um >= cfg->fwd_um || cfg->duty_pct > ACE2K_FEED_SNAG_DUTY_MAX_PCT) {
        return false; /* a lag the watch can never reach would make it a plain tolerance */
    }
    if (!ace2k_within_u32(cfg->back_um, ACE2K_FEED_SNAG_BACK_MIN_UM, ACE2K_FEED_SNAG_BACK_MAX_UM) ||
        !ace2k_within_u32(cfg->rest_ms, ACE2K_FEED_SNAG_REST_MIN_MS, ACE2K_FEED_SNAG_REST_MAX_MS)) {
        return false;
    }
    return ace2k_within_u32(cfg->free_um, ACE2K_FEED_SNAG_FREE_MIN_UM, ACE2K_FEED_SNAG_FREE_MAX_UM);
}

int ace2k_feed_snag_set(struct ace2k_feed *self, const struct ace2k_feed_snag_cfg *cfg)
{
    if (!cfg_valid(cfg)) {
        return -ACE2K_EINVAL;
    }
    self->snag = *cfg;
    return 0;
}
