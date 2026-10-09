#include "feed/feed_internal.h"

/* A switch instead of `a || b || …`: in C that result is an int, and the lint refuses one as a
 * bool. */
static bool is_moving_mode(uint8_t mode)
{
    switch (mode) {
    case ACE2K_FEED_FEEDING:
    case ACE2K_FEED_ROLLING_BACK:
    case ACE2K_FEED_UNLOADING:
    case ACE2K_FEED_LOADING:
        return true;
    default:
        return false;
    }
}

static bool is_assist_mode(uint8_t mode)
{
    switch (mode) {
    case ACE2K_FEED_ASSISTING:
    case ACE2K_FEED_ASSISTING_BACK:
    case ACE2K_FEED_FOLLOWING:
        return true;
    default:
        return false;
    }
}

bool ace2k_feed_mode_runs(uint8_t mode)
{
    if (is_moving_mode(mode)) {
        return true;
    }
    return is_assist_mode(mode);
}

/* The insert line's falling edge, once the first tick has primed insert_last.  An early return
 * instead of `a && b`, for the same reason. */
static bool insert_fell(bool primed, bool last, bool now)
{
    if (!primed || now) {
        return false;
    }
    return last;
}

/* The insert line's rising edge: a strand put in at the mouth — the automatic load's trigger. */
static bool insert_rose(bool primed, bool last, bool now)
{
    if (!primed || !now || last) {
        return false;
    }
    return true;
}

/* mode: the lane's, but for the runout that ends an error, which names the mode that failed. */
static void push_event(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint8_t mode,
                       uint32_t motor_um, int32_t filament_um)
{
    uint8_t next = (uint8_t)((self->head + 1U) % ACE2K_FEED_EVENT_RING);
    if (next == self->tail) {
        self->dropped++;
        return;
    }
    struct ace2k_feed_event *e = &self->ring[self->head];
    e->lane = lane;
    e->kind = kind;
    e->mode = mode;
    e->seq = self->l[lane].seq;
    e->motor_um = motor_um;
    e->filament_um = filament_um;
    ACE2K_COMPILER_BARRIER(); /* payload before index */
    self->head = next;
}

/* The mode's odometers: the tach and the encoder since the mode started. */
void ace2k_feed_odometers(const struct ace2k_feed *self, uint8_t lane, uint32_t *motor_um,
                          int32_t *filament_um)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    *motor_um = ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - l->fg_start);
    *filament_um = ace2k_lane_encoder_um(self->lane, lane) - l->enc_um_start;
}

void ace2k_feed_notice(struct ace2k_feed *self, uint8_t lane, uint8_t kind)
{
    uint32_t motor_um;
    int32_t filament_um;
    ace2k_feed_odometers(self, lane, &motor_um, &filament_um);
    push_event(self, lane, kind, self->l[lane].mode, motor_um, filament_um);
}

void ace2k_feed_notice_at(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t motor_um,
                          int32_t filament_um)
{
    push_event(self, lane, kind, self->l[lane].mode, motor_um, filament_um);
}

/* What a mode carries and no other may read: the sub state, the tail's flag and base, the taut
 * episode's notice, the tip snag's phase and its spent flag, the follow's direction and clock,
 * the feed-forward's base rate, debt, running dose and episodes, the follow's tail.
 * Cleared at every entry and at every end — idle or error — so a lane between modes holds
 * nothing the tick could act on (a tail flag left set once finished a lane again on every idle
 * tick), and a base rate never outlives the follow it was sent to.  bursts and the feed-forward's counters are counts, not flags: the state reports
 * publish them after the assist has ended, so only the next entry clears them. */
static void reset_mode_state(struct ace2k_feed_lane *l)
{
    l->sub = ACE2K_FEED_SUB_NONE;
    l->tail_out = false;
    l->fg_tail = 0;
    l->behind_sent = false;
    l->snag = ACE2K_FEED_SNAG_NONE;
    l->snag_used = false;
    l->follow_dir = ACE2K_FOLLOW_DIR_NONE;
    l->follow_end_ms = 0;
    l->base_um_s = 0;
    l->debt_um = 0;
    l->debt_rem = 0;
    l->dose = false;
    l->dose_enc_um = 0;
    l->taut_seen = false;
    l->full_seen = false;
    l->tail = false;
    l->tail_fg_start = 0;
}

void ace2k_feed_enter(struct ace2k_feed *self, uint8_t lane, uint8_t mode, uint8_t seq,
                      uint32_t now_ms)
{
    ace2k_lane_land_reset(self->lane, lane); /* a reset asked in this tick lands before the bases */
    struct ace2k_feed_lane *l = &self->l[lane];
    l->mode = mode;
    l->seq = seq;
    l->fg_start = self->lane->lane[lane].fg_raw;
    l->enc_um_start = ace2k_lane_encoder_um(self->lane, lane);
    ace2k_feed_compare_restart(self, lane);
    l->t_mode_ms = now_ms;
    reset_mode_state(l);
    l->bursts = 0;
    l->doses = 0;
    l->taut_fixes = 0;
    l->full_fixes = 0;
    l->ff_epoch++; /* every entry, a host start's or the unit's own: the host's tally restarts */
}

/* The lane's mode ends at end_ms: the retry window counts from it when a host start entered the
 * mode (seq non-zero); a mode of the unit's own ending leaves the last host start's window as it
 * was.  Called before the seq is cleared. */
static void end_mode(struct ace2k_feed *self, uint8_t lane, uint32_t end_ms)
{
    if (self->l[lane].seq != 0) {
        self->l[lane].t_end_ms = end_ms;
    }
}

/* Every caller has emitted the mode's events before this (finish() and on_result() push, then
 * come here), so the seq goes back to 0: a lane result surfacing while the lane is idle is the
 * firmware's own, and must not echo a stale host seq.  end_ms is when the mode ended. */
static void go_idle(struct ace2k_feed *self, uint8_t lane, uint32_t end_ms)
{
    end_mode(self, lane, end_ms);
    self->l[lane].mode = ACE2K_FEED_IDLE;
    reset_mode_state(&self->l[lane]);
    self->l[lane].seq = 0;
    self->l[lane].failed_mode = ACE2K_FEED_IDLE;
}

void ace2k_feed_stop_discard(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_lane_result discarded;
    ace2k_lane_stop(self->lane, lane, now_ms);
    (void)ace2k_lane_pop_result(self->lane, lane, &discarded);
}

/* The lane stops, the event goes out with the mode it happened in, then the mode changes. */
void ace2k_feed_finish(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms)
{
    ace2k_feed_stop_discard(self, lane, now_ms);
    ace2k_feed_notice(self, lane, kind);
    go_idle(self, lane, now_ms);
}

void ace2k_feed_init(struct ace2k_feed *self, struct ace2k_lane *lane)
{
    *self = (struct ace2k_feed){ 0 };
    self->lane = lane;
    self->th.slip_check_um = ACE2K_FEED_SLIP_CHECK_UM;
    self->th.slip_allow_um = ACE2K_FEED_SLIP_ALLOW_UM;
    self->th.stall_check_um = ACE2K_FEED_STALL_CHECK_UM;
    self->load.park_um = ACE2K_FEED_LOAD_PARK_UM;
    self->load.speed_um_s = ACE2K_FEED_LOAD_SPEED_UM_S;
    self->load.auto_load = true;
    self->snag = (struct ace2k_feed_snag_cfg){
        .fwd_um = ACE2K_FEED_SNAG_FWD_UM,
        .lag_um = ACE2K_FEED_SNAG_LAG_UM,
        .back_um = ACE2K_FEED_SNAG_BACK_UM,
        .rest_ms = ACE2K_FEED_SNAG_REST_MS,
        .free_um = ACE2K_FEED_SNAG_FREE_UM,
        .duty_pct = ACE2K_FEED_SNAG_DUTY_PCT,
    };
    self->follow = (struct ace2k_feed_follow_cfg){
        .flip_ms = ACE2K_FEED_FOLLOW_FLIP_MS,
        .take_um = ACE2K_FEED_FOLLOW_TAKE_UM,
        .tail_um = ACE2K_FEED_FOLLOW_TAIL_UM,
    };
    self->ff = (struct ace2k_feed_ff_cfg){
        .chunk_um = ACE2K_FEED_FF_CHUNK_UM,
        .pulse_um_s = ACE2K_FEED_FF_PULSE_UM_S,
    };
}

/* An error the lane core has already stopped the motor for (its result is what brought us
 * here): the event goes out with the mode it happened in and the result's odometers, then the
 * lane stays in error until clear (rule 8). */
static void enter_error(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t motor_um,
                        int32_t filament_um, uint32_t now_ms)
{
    push_event(self, lane, kind, self->l[lane].mode, motor_um, filament_um);
    self->l[lane].failed_mode = self->l[lane].mode;
    self->l[lane].mode = ACE2K_FEED_ERROR;
    reset_mode_state(&self->l[lane]);
    self->l[lane].error_kind = kind;
    end_mode(self, lane, now_ms);
}

void ace2k_feed_fail(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms)
{
    uint32_t motor_um;
    int32_t filament_um;
    ace2k_feed_stop_discard(self, lane, now_ms);
    ace2k_feed_odometers(self, lane, &motor_um, &filament_um);
    enter_error(self, lane, kind, motor_um, filament_um, now_ms);
}

/* A move that ran its length ends a feed, a rollback or an unload: true when it did.  Once a
 * tolerance ran, the last move is not the mode: the event carries the mode's odometers. */
static bool move_done_ends_mode(struct ace2k_feed *self, uint8_t lane,
                                const struct ace2k_lane_result *r, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode == ACE2K_FEED_UNLOADING) {
        /* the budget spent with the strand still at the mouth: a hand is needed */
        if (l->snag_used) {
            ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
            return true;
        }
        enter_error(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, r->motor_um, r->filament_um, now_ms);
        return true;
    }
    if (l->mode == ACE2K_FEED_FEEDING || l->mode == ACE2K_FEED_ROLLING_BACK) {
        if (l->snag_used) {
            ace2k_feed_finish(self, lane, ACE2K_FEED_DONE, now_ms); /* the whole mode's odometers */
            return true;
        }
        push_event(self, lane, ACE2K_FEED_DONE, l->mode, r->motor_um, r->filament_um);
        go_idle(self, lane, now_ms);
        return true;
    }
    return false;
}

/* An assist's move reached its bound.  In the follow, the follow's rule 1 clock starts here, and the next
 * move in the same direction may start on this tick — the chaining of either assist below,
 * by the direction the move ran.  A dose of the feed-forward reaching its chunk is no burst: no
 * behind notice — the tick's step settles its debt and reads the switches (feed_assist.c). */
static void assist_bound(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                         uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool forward_assist = l->mode == ACE2K_FEED_ASSISTING;
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        l->follow_end_ms = now_ms;
        forward_assist = l->follow_dir == ACE2K_LANE_FORWARD;
    }
    if (forward_assist) {
        /* the burst reached its bound: the head is still pulling and the plunger has not come
         * back to rest — the next burst on this tick (decided at the bench:
         * a steady pull near the burst's speed keeps the plunger between rest and full, and a
         * bound reached is no fault).  The tick's assist step reads the switches again: taut
         * and not full starts it; rest or the full reading leaves the lane waiting.  The bound
         * reached with the strand still taut is the head outrunning a whole burst: the behind
         * notice, once per taut episode (taut is the normal start of a burst, not a notice:
         * observed at the bench, fourteen notices in a minute of ordinary pulling) */
        l->sub = ACE2K_FEED_SUB_WAITING;
        if (ace2k_feed_bit(in->pushed, lane) && !l->behind_sent && !l->dose) {
            ace2k_feed_notice(self, lane, ACE2K_FEED_BEHIND);
            l->behind_sent = true;
        }
        return;
    }
    if (l->mode == ACE2K_FEED_ASSISTING_BACK || l->mode == ACE2K_FEED_FOLLOWING) {
        /* the take-up's bound: this tick reads the switches again — still full, the next
         * take-up; rest or taut, the lane waits */
        l->sub = ACE2K_FEED_SUB_WAITING;
    }
}

/* A lane result the lane core produced on its own (the stop paths pop theirs at once). */
static void on_result(struct ace2k_feed *self, uint8_t lane, const struct ace2k_lane_result *r,
                      const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (r->outcome != ACE2K_LANE_LINK_LOST && r->outcome != ACE2K_LANE_SHUTDOWN &&
        ace2k_feed_snag_on_result(self, lane, r, now_ms)) {
        return; /* the touch's end, or a length reached in the watch: the tolerance's */
    }
    switch (r->outcome) {
    case ACE2K_LANE_LINK_LOST:
        push_event(self, lane, ACE2K_FEED_STOPPED_LINK, self->l[lane].mode, r->motor_um,
                   r->filament_um);
        go_idle(self, lane, now_ms);
        return;
    case ACE2K_LANE_SHUTDOWN:
        push_event(self, lane, ACE2K_FEED_STOPPED_SHUTDOWN, self->l[lane].mode, r->motor_um,
                   r->filament_um);
        go_idle(self, lane, now_ms);
        return;
    case ACE2K_LANE_STALLED:
        enter_error(self, lane, ACE2K_FEED_MOTOR_STALLED, r->motor_um, r->filament_um, now_ms);
        return;
    case ACE2K_LANE_TIMEOUT:
        if (!ace2k_feed_tail_after_deadline(self, lane, in, now_ms)) {
            enter_error(self, lane, ACE2K_FEED_TIMEOUT, r->motor_um, r->filament_um, now_ms);
        }
        return;
    case ACE2K_LANE_STOPPED:
        return; /* our own stop: the event is already out */
    case ACE2K_LANE_DONE:
    default:
        break;
    }
    if (move_done_ends_mode(self, lane, r, now_ms)) {
        return;
    }
    if (l->mode == ACE2K_FEED_LOADING) {
        if (!ace2k_feed_load_move_done(self, lane, in, now_ms)) {
            ace2k_feed_finish(self, lane, ACE2K_FEED_LOADED, now_ms); /* the mode's odometers */
        }
        return;
    }
    assist_bound(self, lane, in, now_ms);
}

/* The insert's falling edge, first in a lane's tick: true when the edge ended the mode or the
 * error.  In the follow the strand ran out but its tail is still between the insert and the
 * drive: the follow feeds on as its tail, with a notice.  The tail
 * ignores the bay from then on: the old end holds the drive gear until the tail is out,
 * so a strand put in at the mouth meanwhile can never be gripped — neither its rising edge nor
 * its withdrawal changes the tail, and at the tail-out the lane goes idle with no load (the
 * insert already set, no fresh edge: the operator puts it in again).  In unloading the fall is
 * the goal, elsewhere the strand ran out — in a load, the operator withdrew it. */
static bool on_insert_fall(struct ace2k_feed *self, uint8_t lane, bool fell, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!fell) {
        return false;
    }
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        if (!l->tail) {
            ace2k_feed_follow_tail_begin(self, lane, now_ms);
        }
        return false;
    }
    if (ace2k_feed_mode_runs(l->mode)) {
        uint8_t kind = l->mode == ACE2K_FEED_UNLOADING ? ACE2K_FEED_UNLOADED : ACE2K_FEED_RUNOUT;
        ace2k_feed_finish(self, lane, kind, now_ms);
        return true;
    }
    /* the strand leaving a lane in error ends the error (decided at the bench: a strand withdrawn by the lever during a load's pull tripped tangled — true at that
     * instant — and the lane stayed in error with no strand in it).  The error was about the
     * strand; without one there is none: runout, the odometers since the failed mode began, the
     * lane idle with no clear.  The event names the mode that failed — a runout of the load, of
     * the rollback — not the error state it left, and carries the failed start's seq, which the
     * lane kept.  The one way out of error but clear (feed.h). */
    if (l->mode == ACE2K_FEED_ERROR) {
        uint32_t motor_um;
        int32_t filament_um;
        ace2k_feed_stop_discard(self, lane, now_ms);
        ace2k_feed_odometers(self, lane, &motor_um, &filament_um);
        push_event(self, lane, ACE2K_FEED_RUNOUT, l->failed_mode, motor_um, filament_um);
        go_idle(self, lane, now_ms);
        l->error_kind = 0;
        return true;
    }
    return false;
}

/* One lane's tick, in this order: the insert's fall, the lane's own result, the link, the
 * autonomous behaviours (the automatic load, the assists' and the load's steps), the follow's
 * tail bound, this lane's buffer full against a forward push — the tip snag's tolerance
 * (feed_snag.c), which never runs in the follow's tail —, then the comparator or the tail budget
 * (the verdicts: feed_judge.c).  Every step that ends the mode
 * returns: the rest judges a mode that no longer runs. */
static void tick_lane(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool insert = ace2k_feed_bit(in->insert, lane);
    bool fell = insert_fell(self->primed, l->insert_last, insert);
    bool rose = insert_rose(self->primed, l->insert_last, insert);
    /* an edge during a mode is that mode's business — a rollback that brings a piece back to the
     * mouth on the tick it ends must not load it again — so the automatic load reads the mode as
     * it was before this tick's result could end it */
    bool load_edge = false;
    if (rose && l->mode == ACE2K_FEED_IDLE) {
        load_edge = true;
    }
    l->insert_last = insert;
    if (on_insert_fall(self, lane, fell, now_ms)) {
        return;
    }
    struct ace2k_lane_result r;
    if (ace2k_lane_pop_result(self->lane, lane, &r)) {
        on_result(self, lane, &r, in, now_ms);
    }
    /* nothing keeps moving, or stays armed to move, without a host (rule 6): the lane core has
     * ended every running move with link_lost by now (above); an assist waiting on the buffer or
     * a load in its settle has no move for it to end, so the feed ends those itself */
    if (!in->link_ok && ace2k_feed_mode_runs(l->mode)) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED_LINK, now_ms);
        return;
    }
    ace2k_feed_auto_tick(self, lane, in, load_edge, now_ms);
    if (!ace2k_feed_mode_runs(l->mode)) {
        return; /* idle, or the assist's own supervisor failed the lane on this tick */
    }
    if (l->tail && ace2k_feed_follow_tail_step(self, lane, now_ms)) {
        return; /* the tail's bound reached: tail_out */
    }
    /* !l->tail is defensive: a tail exists only while FOLLOWING, which pushes toward the head and
     * so never reaches the snag today; the guard keeps the tail out of the snag should that
     * change */
    if (!l->tail && ace2k_feed_snag_step(self, lane, in, now_ms)) {
        return; /* this lane's full, or a tolerance running: feed_snag.c judges the tick */
    }
    if (l->tail_out) {
        ace2k_feed_tail_step(self, lane, now_ms); /* the comparator has spoken; the budget judges */
        return;
    }
    enum ace2k_feed_trip trip = ace2k_feed_compare(self, lane); /* none once the lane stops */
    if (trip != ACE2K_FEED_TRIP_NONE) {
        ace2k_feed_on_trip(self, lane, trip, in, now_ms);
    }
}

void ace2k_feed_tick(struct ace2k_feed *self, uint32_t now_ms, const struct ace2k_feed_inputs *in)
{
    /* after a Klipper shutdown the link reads as down whatever the transport says: the host's
     * clock queries keep it up, and nothing may start (ace2k_feed_shutdown) */
    struct ace2k_feed_inputs seen = *in;
    if (self->shut_down) {
        seen.link_ok = false;
    }
    self->last = seen;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        tick_lane(self, i, &seen, now_ms);
    }
    self->primed = true;
}

void ace2k_feed_shutdown(struct ace2k_feed *self, uint32_t now_ms)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        /* a SHUTDOWN result the lane's handler left, or any other not yet read: this is the
         * lane's one event, so on_result never sees it */
        struct ace2k_lane_result discarded;
        (void)ace2k_lane_pop_result(self->lane, i, &discarded);
        ace2k_feed_stop_discard(self, i, now_ms);
        if (ace2k_feed_mode_runs(self->l[i].mode)) {
            ace2k_feed_notice(self, i, ACE2K_FEED_STOPPED_SHUTDOWN);
            go_idle(self, i, now_ms);
        }
    }
    self->last.link_ok = false;
    self->shut_down = true;
}

static void stop_one(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (ace2k_feed_mode_runs(self->l[lane].mode)) {
        /* a load still settling ends here with nothing ever moved (rule 9) */
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED, now_ms);
        return;
    }
    /* idle or error: the lines at stop whatever the state (rule 5), no event, no mode change */
    ace2k_feed_stop_discard(self, lane, now_ms);
}

void ace2k_feed_stop(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (lane == ACE2K_LANE_ALL) {
        for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
            stop_one(self, i, now_ms);
        }
        return;
    }
    if (lane < ACE2K_LANE_COUNT) {
        stop_one(self, lane, now_ms);
    }
}

bool ace2k_feed_clear(struct ace2k_feed *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || self->l[lane].mode != ACE2K_FEED_ERROR) {
        return false;
    }
    /* a clear ends no mode: the retry window stays the failure's, so a start repeating the
     * failed one long after it is a new command, not its retry */
    go_idle(self, lane, self->l[lane].t_end_ms);
    self->l[lane].error_kind = 0;
    return true;
}

int ace2k_feed_set_speed(struct ace2k_feed *self, uint8_t lane, uint32_t speed_um_s,
                         uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_mode_runs(l->mode)) {
        return -ACE2K_EREFUSED;
    }
    /* the lane checks the bounds first: -ACE2K_EINVAL is out of bounds, -ACE2K_EREFUSED no move
     * running — an assist between two bursts, a load in its settle — where the speed is armed
     * for the next move all the same */
    int rc = ace2k_lane_set_speed(self->lane, lane, speed_um_s, now_ms);
    if (rc == -ACE2K_EINVAL) {
        return rc;
    }
    l->speed_um_s = speed_um_s;
    return 0;
}

void ace2k_feed_thresholds_set(struct ace2k_feed *self, const struct ace2k_feed_thresholds *th)
{
    self->th = *th;
}

void ace2k_feed_load_set(struct ace2k_feed *self, const struct ace2k_feed_load_cfg *cfg)
{
    self->load = *cfg;
}

bool ace2k_feed_peek_event(const struct ace2k_feed *self, struct ace2k_feed_event *out)
{
    if (self->tail == self->head) {
        return false;
    }
    *out = self->ring[self->tail];
    return true;
}

void ace2k_feed_drop_event(struct ace2k_feed *self)
{
    if (self->tail == self->head) {
        return;
    }
    ACE2K_COMPILER_BARRIER(); /* the copy, if any, before the release */
    self->tail = (uint8_t)((self->tail + 1U) % ACE2K_FEED_EVENT_RING);
}

bool ace2k_feed_pop_event(struct ace2k_feed *self, struct ace2k_feed_event *out)
{
    if (!ace2k_feed_peek_event(self, out)) {
        return false;
    }
    ace2k_feed_drop_event(self);
    return true;
}

bool ace2k_feed_has_events(const struct ace2k_feed *self)
{
    return self->tail != self->head;
}

static bool one_lane_busy(const struct ace2k_feed *self, uint8_t lane)
{
    uint8_t mode = self->l[lane].mode;
    if (mode == ACE2K_FEED_IDLE) {
        return false;
    }
    return mode != ACE2K_FEED_ERROR;
}

bool ace2k_feed_lane_busy(const struct ace2k_feed *self, uint8_t lane)
{
    if (lane == ACE2K_LANE_ALL) {
        for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
            if (one_lane_busy(self, i)) {
                return true;
            }
        }
        return false;
    }
    if (lane >= ACE2K_LANE_COUNT) {
        return false;
    }
    return one_lane_busy(self, lane);
}

uint8_t ace2k_feed_mode(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].mode : 0;
}

uint8_t ace2k_feed_error(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].error_kind : 0;
}

uint16_t ace2k_feed_bursts(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].bursts : 0;
}

struct ace2k_feed_ff_counts ace2k_feed_ff_counts(const struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_ff_counts c = { 0 };
    if (lane < ACE2K_LANE_COUNT) {
        c.doses = self->l[lane].doses;
        c.taut_fixes = self->l[lane].taut_fixes;
        c.full_fixes = self->l[lane].full_fixes;
        c.epoch = self->l[lane].ff_epoch;
    }
    return c;
}

uint8_t ace2k_feed_seq(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].last_seq : 0;
}
