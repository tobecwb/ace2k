#include "feed/feed_internal.h"

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

/* In C, the result of a logical operator is an int.  The (bool) casts below say that the conversion
 * is meant, because the lint refuses an implicit one. */
bool ace2k_feed_mode_runs(uint8_t mode)
{
    return (bool)(is_moving_mode(mode) || is_assist_mode(mode));
}

/* The falling edge of the insert line, once the first tick has primed insert_last. */
static bool insert_fell(bool primed, bool last, bool now)
{
    return (bool)(primed && last && !now);
}

/* The rising edge of the insert line: filament put in at the bay entrance.  This is the trigger of
 * the automatic load. */
static bool insert_rose(bool primed, bool last, bool now)
{
    return (bool)(primed && !last && now);
}

/* What a mode carries, and no other mode may read.  That is the sub state, the flag and base of the
 * tail, the notice of the taut episode, the tip snag's phase with its spent and notice flags, and
 * the free tip.  It is also the direction and clock of the follow, the base rate, debt, running
 * dose and episodes of the feed-forward, the follow's tail, and the armed stop with its held
 * window.  The stop armed for the next start (stop_next) does not belong to the mode, because only
 * a host start takes it.
 * All of it is cleared at every entry and at every end, idle or error.  So a lane between modes
 * holds nothing that the tick could act on.  A tail flag once left set finished a lane again on
 * every idle tick.  A base rate never outlives the follow it was sent to.  bursts and the counters
 * of the feed-forward are counts, not flags.  The state reports publish them after the assist has
 * ended, so only the next entry clears them. */
static void reset_mode_state(struct ace2k_feed_lane *l)
{
    l->sub = ACE2K_FEED_SUB_NONE;
    l->tail_out = false;
    l->fg_tail_pulses = 0;
    l->tail_budget_um = 0;
    l->clearing = false;
    l->clear_moved = false;
    l->fg_clear_pulses = 0;
    l->grace = false; /* a load from the bay sets it after its entry (ace2k_feed_begin_load) */
    l->behind_sent = false;
    l->snag = ACE2K_FEED_SNAG_NONE;
    l->snag_used = false;
    l->free_tip = false;
    l->snag_sent = false;
    l->follow_dir = ACE2K_FEED_FOLLOW_DIR_NONE;
    l->follow_end_ms = 0;
    l->base_um_s = 0;
    l->debt_um = 0;
    l->debt_rem_nm = 0;
    l->dose = false;
    l->dose_enc_um = 0;
    l->dose_against = false;
    l->taut_seen = false;
    l->full_seen = false;
    l->tail = false;
    l->tail_fg_start_pulses = 0;
    l->tail_enc_start_counts = 0;
    l->stop_on = (struct ace2k_feed_stop_on){ .held = false, .at_rest = false };
    l->stop_judged = false;
    l->held_open = false;
    l->held_t0_ms = 0;
    l->held_enc0_um = 0;
}

void ace2k_feed_enter(struct ace2k_feed *self, uint8_t lane, uint8_t mode, uint8_t seq,
                      uint32_t now_ms)
{
    ace2k_lane_land_reset(self->lane, lane); /* a reset asked in this tick lands before the bases */
    struct ace2k_feed_lane *l = &self->l[lane];
    l->mode = mode;
    l->seq = seq;
    l->search_seq = 0; /* a read with motion sets its own key after its entry */
    l->search_on = false;
    l->fg_start_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->enc_um_start = ace2k_lane_encoder_um(self->lane, lane);
    ace2k_feed_compare_restart(self, lane);
    l->t_mode_ms = now_ms;
    reset_mode_state(l);
    l->ledger_ms = now_ms;
    l->bursts = 0;
    l->doses = 0;
    l->back_doses = 0;
    l->taut_fixes = 0;
    l->full_fixes = 0;
    l->ff_epoch++; /* every entry, by a host start or by the unit: the host's tally restarts */
}

/* The lane's mode ends at end_ms.  When a host start entered the mode (seq non-zero), the retry
 * window counts from this time.  When a mode of the unit itself ends, the window of the last host
 * start stays as it was.  Called before the seq is cleared. */
static void end_mode(struct ace2k_feed *self, uint8_t lane, uint32_t end_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->seq != 0) {
        l->t_end_ms = end_ms;
    }
    if (l->search_on) { /* the read with motion's window counts from here */
        l->search_on = false;
        l->search_end_ms = end_ms;
    }
}

/* Every caller has emitted the mode's events before this.  finish() and on_result() push, then come
 * here.  So the seq goes back to 0.  A lane result that surfaces while the lane is idle is the
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
    self->unload_clear_um = ACE2K_FEED_UNLOAD_CLEAR_UM;
    self->held = (struct ace2k_feed_held_cfg){
        .held_um = ACE2K_FEED_HELD_UM,
        .held_ms = ACE2K_FEED_HELD_MS,
    };
}

/* An error with the motor already stopped, by the lane core, whose result brought us here, or by
 * ace2k_feed_fail.  The event goes out with the mode it happened in and the odometers of the mode.
 * Then the lane stays in error until clear (rule 8). */
static void enter_error(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms)
{
    ace2k_feed_notice(self, lane, kind);
    self->l[lane].failed_mode = self->l[lane].mode;
    self->l[lane].mode = ACE2K_FEED_ERROR;
    reset_mode_state(&self->l[lane]);
    self->l[lane].error_kind = kind;
    end_mode(self, lane, now_ms);
}

void ace2k_feed_fail(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t now_ms)
{
    ace2k_feed_stop_discard(self, lane, now_ms);
    enter_error(self, lane, kind, now_ms);
}

/* A move that ran its length ends a feed, a rollback or an unload.  It returns true when it did.
 * The event carries the mode's odometers, not those of the move.  A mode can run several moves,
 * such as a tail or a resume after a touch, and the event reports them all. */
static bool move_done_ends_mode(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode == ACE2K_FEED_UNLOADING) {
        /* The budget is spent with the filament still at the bay entrance, so a hand is needed. */
        enter_error(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
        return true;
    }
    if (l->mode == ACE2K_FEED_FEEDING || l->mode == ACE2K_FEED_ROLLING_BACK) {
        ace2k_feed_notice(self, lane, ACE2K_FEED_DONE);
        go_idle(self, lane, now_ms);
        return true;
    }
    return false;
}

/* A move of an assist reached its bound.  In the follow, the clock of the follow's rule 1 starts
 * here.  The next move in the same direction may start on this tick.  That is the chaining of
 * either assist below, by the direction the move ran.  A dose of the feed-forward that reaches its
 * chunk is no burst, so there is no behind notice.  The step of the tick settles its debt and reads
 * the switches (feed_assist.c). */
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
        /* The burst reached its bound.  The head is still pulling, and the plunger has not come
         * back to rest.  So the next burst starts on this tick.  This was chosen from tests on the
         * unit.  A steady pull near the burst's speed keeps the plunger between rest and full, and
         * a bound reached is no fault.  The tick's assist step reads the switches again.  Taut and
         * not full starts the burst.  Rest, or the full reading, leaves the lane waiting.  When the
         * bound is reached with the filament still taut, the head is outrunning a whole burst.
         * That gives the behind notice, once per taut episode.  Taut alone is the normal start of a
         * burst, not a notice.  Observed on the unit: a notice on taut alone fired fourteen times
         * in a minute of ordinary pulling. */
        l->sub = ACE2K_FEED_SUB_WAITING;
        if (ace2k_feed_bit(in->pushed, lane) && !l->behind_sent && !l->dose) {
            ace2k_feed_notice(self, lane, ACE2K_FEED_BEHIND);
            l->behind_sent = true;
        }
        return;
    }
    if (l->mode == ACE2K_FEED_ASSISTING_BACK || l->mode == ACE2K_FEED_FOLLOWING) {
        /* The take-up reached its bound.  This tick reads the switches again.  Still full starts
         * the next take-up.  Rest or taut leaves the lane waiting. */
        l->sub = ACE2K_FEED_SUB_WAITING;
    }
}

/* A lane result that the lane core produced on its own.  The stop paths pop their own results at
 * once.  Only a mode that runs owns a result.  The feed is the only mover of the lane core, and
 * every way into idle or error stops the lane and drops its result first.  So a result on a lane
 * that is idle or in error is a move that the feed never started.  It is not an outcome of any
 * mode, and it is dropped.  Every event raised here carries the mode's odometers, never those of
 * the result, because the result covers only its own move. */
static void on_result(struct ace2k_feed *self, uint8_t lane, const struct ace2k_lane_result *r,
                      const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_mode_runs(l->mode)) {
        return;
    }
    if (r->outcome != ACE2K_LANE_LINK_LOST && r->outcome != ACE2K_LANE_SHUTDOWN &&
        ace2k_feed_snag_on_result(self, lane, r, now_ms)) {
        /* The end of the touch, a length reached in the watch, or a stall or a deadline in the
         * watch or free phase of the search (the obstacle return).  These belong to the
         * tolerance. */
        return;
    }
    switch (r->outcome) {
    case ACE2K_LANE_LINK_LOST:
        ace2k_feed_notice(self, lane, ACE2K_FEED_STOPPED_LINK);
        go_idle(self, lane, now_ms);
        return;
    case ACE2K_LANE_SHUTDOWN:
        ace2k_feed_notice(self, lane, ACE2K_FEED_STOPPED_SHUTDOWN);
        go_idle(self, lane, now_ms);
        return;
    case ACE2K_LANE_STALLED:
        enter_error(self, lane, ACE2K_FEED_MOTOR_STALLED, now_ms);
        return;
    case ACE2K_LANE_TIMEOUT:
        if (!ace2k_feed_tail_after_deadline(self, lane, r, in, now_ms)) {
            enter_error(self, lane, ACE2K_FEED_TIMEOUT, now_ms);
        }
        return;
    case ACE2K_LANE_STOPPED:
        return; /* our own stop: the event is already out */
    case ACE2K_LANE_DONE:
    default:
        break;
    }
    if (move_done_ends_mode(self, lane, now_ms)) {
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

/* The clear's one fresh move: the rest of the clear, back toward the spool, at the mode's speed.
 * It returns false when the clear cannot go on: its move was spent, or the lane refused it. */
static bool clear_move(struct ace2k_feed *self, uint8_t lane, uint32_t owed_um, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->clear_moved) {
        return false;
    }
    l->clear_moved = true;
    ace2k_feed_stop_discard(self, lane, now_ms);
    return ace2k_lane_move(self->lane, lane, ACE2K_LANE_REVERSE, owed_um, l->speed_um_s, now_ms) ==
           0;
}

/* The clear of an unload begins at the insert's fall (ACE2K_FEED_UNLOAD_CLEAR_UM).  It returns true
 * when it began, so the fall does not end the unload.  With the motor turning back, the running
 * move goes on.  With the motor turning forward, in the touch of the tip snag, or stopped, as in
 * its wait for rest, the clear's fresh move turns it back.  So does a move that ran its length on
 * this tick.  Any other end of the move on this tick ends the unload at the fall, as without a
 * clear.  So do a clear of 0 and a fresh move the lane refuses.  The base of the clear's travel is
 * taken after the stop of a forward touch, although the tach, which has no direction, still counts
 * its coast.  A second fall in the clear changes nothing: its base stays at the first. */
static bool clear_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->clearing) {
        return true;
    }
    if (self->unload_clear_um == 0) {
        return false;
    }
    bool moving = ace2k_lane_is_moving(self->lane, lane);
    if (!moving || !ace2k_lane_is_reverse(self->lane, lane)) {
        /* The result of a move that ended on this tick is consumed here on purpose, in both
         * branches: a move that ran its length gives way to the fresh move, and any other end
         * ends the unload at the fall, which needs no result. */
        struct ace2k_lane_result r;
        bool ended = false;
        if (!moving) {
            ended = ace2k_lane_pop_result(self->lane, lane, &r);
        }
        if (ended && r.outcome != ACE2K_LANE_DONE) {
            return false;
        }
        if (!clear_move(self, lane, self->unload_clear_um, now_ms)) {
            return false;
        }
    }
    l->fg_clear_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->clearing = true;
    return true;
}

/* The insert's falling edge, first in the tick of a lane.  It returns true when the edge ended the
 * mode or the error.
 * In the follow, the filament ran out, but its tail is still between the insert and the drive.  The
 * follow feeds on as its tail, with a notice.  From then on the tail ignores the bay.  The old end
 * holds the drive gear until the tail is out.  So filament put in at the bay entrance meanwhile can
 * never be gripped.  Neither its rising edge nor its withdrawal changes the tail.  At the tail-out
 * the lane goes idle with no load, because the insert is already set and there is no fresh edge.
 * The operator puts the filament in again.
 * In unloading, the fall is the goal, and the clear begins (clear_begin).  Elsewhere, the filament
 * ran out.  In a load, the operator withdrew it. */
static bool on_insert_fall(struct ace2k_feed *self, uint8_t lane, bool fell, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!fell) {
        return false;
    }
    if (l->mode == ACE2K_FEED_UNLOADING && clear_begin(self, lane, now_ms)) {
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
    /* Filament that leaves a lane in error ends the error.  This was chosen from tests on the unit.
     * Observed on the unit: filament withdrawn by the lever during the pull of a load tripped
     * tangled, which was true at that instant.  The lane then stayed in error with no filament in
     * it.  The error was about the filament, and without filament there is no error.  So the lane
     * reports a runout, with the odometers since the failed mode began, and goes idle with no
     * clear.  The event names the mode that failed, such as a runout of the load or of the
     * rollback, not the error state that it left.  It carries the seq of the failed start, which
     * the lane kept.  This is the one way out of error other than clear (feed.h). */
    if (l->mode == ACE2K_FEED_ERROR) {
        uint32_t motor_um;
        int32_t filament_um;
        ace2k_feed_stop_discard(self, lane, now_ms);
        ace2k_feed_odometers(self, lane, &motor_um, &filament_um);
        ace2k_feed_push_event(self, lane, ACE2K_FEED_RUNOUT, l->failed_mode, motor_um, filament_um);
        /* like a clear, this ends no mode: the retry window stays the failure's */
        go_idle(self, lane, l->t_end_ms);
        l->error_kind = 0;
        return true;
    }
    return false;
}

/* Nothing keeps moving, or stays armed to move, without a host (rule 6), nor on insert sensors that
 * say nothing.  It returns true when it ended the lane's mode.
 * - The link: by now the lane core has ended every running move with link_lost.  An assist waiting
 *   on the buffer, or a load in its settle, has no move for the core to end.  So the feed ends
 *   those itself.
 * - The insert sensors unknown: a running mode would never see its filament run out, or leave the
 *   sensor.  It stops rather than act on a frozen reading.  Nothing starts again on its own,
 *   because the starts are refused until the readings are back (feed_start.c). */
static bool inputs_stop(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    if (!ace2k_feed_mode_runs(self->l[lane].mode)) {
        return false;
    }
    if (!in->link_ok) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED_LINK, now_ms);
        return true;
    }
    if (in->insert_unknown) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED_SENSORS, now_ms);
        return true;
    }
    return false;
}

/* The tick of a lane in the clear, in place of the steps after the insert's fall.  The goal of the
 * unload is reached, so nothing ends it as a fault.  A link lost or a shutdown ends it as ever, and
 * so do the link and the insert sensors (inputs_stop).  The clear's own travel ends it unloaded.
 * A move that runs its length before that is followed by the clear's one fresh move for what the
 * clear still owes.  A stall or a deadline ends it unloaded at once: the motor is not driven again
 * into what stopped it.  When the fresh move ends too, or the lane refuses it, the unload ends
 * unloaded.  The armed stop, the tip snag, the tail and the comparator do not judge it. */
static void clear_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                       uint32_t now_ms)
{
    struct ace2k_lane_result r;
    bool ended = ace2k_lane_pop_result(self->lane, lane, &r);
    if (ended && (r.outcome == ACE2K_LANE_LINK_LOST || r.outcome == ACE2K_LANE_SHUTDOWN)) {
        on_result(self, lane, &r, in, now_ms);
        return;
    }
    if (inputs_stop(self, lane, in, now_ms)) {
        return;
    }
    uint32_t spent_um = ace2k_feed_motor_since_um(self, lane, self->l[lane].fg_clear_pulses);
    if (spent_um >= self->unload_clear_um) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_UNLOADED, now_ms);
        return;
    }
    if (ended && r.outcome != ACE2K_LANE_DONE) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_UNLOADED, now_ms);
        return;
    }
    if ((ended || !ace2k_lane_is_moving(self->lane, lane)) &&
        !clear_move(self, lane, self->unload_clear_um - spent_um, now_ms)) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_UNLOADED, now_ms);
    }
}

/* The insert's fall (on_insert_fall), then the clear of an unload, which is then the whole tick
 * (clear_step).  It returns true when the rest of the tick must not run. */
static bool fall_or_clear(struct ace2k_feed *self, uint8_t lane, bool fell,
                          const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (on_insert_fall(self, lane, fell, now_ms)) {
        return true;
    }
    if (self->l[lane].clearing) {
        clear_step(self, lane, in, now_ms);
        return true;
    }
    return false;
}

/* The tick of one lane, in this order.
 * 1. The insert's fall.  Its edges are judged only between two known readings.  In the clear of an
 *    unload, the clear's own step is the whole tick (clear_step).
 * 2. The lane's own result.
 * 3. The link, then the insert sensors: unknown, they stop a running mode.
 * 4. The stop that the host armed for this move, held or at rest (feed_stop.c).
 * 5. The autonomous behaviours: the automatic load, and the steps of the assists and of the load.
 * 6. The bound of the follow's tail.
 * 7. A full buffer on this lane during a forward push.  This is the tip snag's tolerance
 *    (feed_snag.c), and it never runs in the follow's tail.
 * 8. The comparator or the tail budget, which give the verdicts (feed_judge.c).
 * The armed stop goes after the link and before the autonomous steps on purpose, so a move that it
 * ends starts no burst on that tick.  Every step that ends the mode returns, because the rest would
 * judge a mode that no longer runs. */
static void tick_lane(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool insert = ace2k_feed_bit(in->insert, lane);
    /* A frozen reading says nothing, and the first known one after it only primes the bit: it may
     * differ from the frozen one without any filament having moved (the boot's first pass too). */
    bool judged = (bool)(self->primed && !self->insert_was_unknown && !in->insert_unknown);
    bool fell = insert_fell(judged, l->insert_last, insert);
    bool rose = insert_rose(judged, l->insert_last, insert);
    /* An edge during a mode belongs to that mode.  A rollback that brings a piece back to the bay
     * entrance on the tick it ends must not load it again.  So the automatic load reads the mode as
     * it was before the result of this tick could end it. */
    bool load_edge = false;
    if (rose && l->mode == ACE2K_FEED_IDLE) {
        load_edge = true;
    }
    l->insert_last = insert;
    if (fall_or_clear(self, lane, fell, in, now_ms)) {
        return;
    }
    struct ace2k_lane_result r;
    if (ace2k_lane_pop_result(self->lane, lane, &r)) {
        on_result(self, lane, &r, in, now_ms);
    }
    if (inputs_stop(self, lane, in, now_ms)) {
        return;
    }
    /* An armed stop goes first: held after held_ms, or at rest.  The comparator trip on a held tip
     * is handled below. */
    if (ace2k_feed_stop_step(self, lane, in, now_ms)) {
        return;
    }
    ace2k_feed_auto_tick(self, lane, in, load_edge, now_ms);
    if (!ace2k_feed_mode_runs(l->mode)) {
        return; /* idle, or the assist's own supervisor failed the lane on this tick */
    }
    if (l->tail && ace2k_feed_follow_tail_step(self, lane, now_ms)) {
        return; /* the bound of the tail is reached: tail_out */
    }
    /* !l->tail is defensive.  A tail exists only while FOLLOWING, which moves either way.  Its
     * bursts and forward doses go toward the head, and its take-ups and reverse doses go back.  Yet
     * today a tail never reaches the snag.  The forward tolerance arms only in a mode that pushes
     * toward the head (pushes_toward_head): a feed, the pull of a load or its search, never
     * FOLLOWING.  The reverse case arms only in a rollback or an unload (feed_snag.c).  The guard
     * keeps the tail out of the snag if either of these changes. */
    if (!l->tail && ace2k_feed_snag_step(self, lane, in, now_ms)) {
        /* This lane's full, or a running tolerance: feed_snag.c judges the tick.  The free phase
         * judges, returns false, and lets the comparator run. */
        return;
    }
    if (l->tail_out) {
        ace2k_feed_tail_step(self, lane, now_ms); /* the comparator has spoken; the budget judges */
        return;
    }
    enum ace2k_feed_trip trip = ace2k_feed_compare(self, lane); /* none once the lane stops */
    if (trip != ACE2K_FEED_TRIP_NONE && !ace2k_feed_stop_on_trip(self, lane, trip, in, now_ms)) {
        /* Not a held tip on a move armed for it, so the verdicts apply.  A held tip is held and
         * never stuck, or its partial trip is put off to the next tick. */
        ace2k_feed_on_trip(self, lane, trip, in, now_ms);
    }
}

void ace2k_feed_tick(struct ace2k_feed *self, uint32_t now_ms, const struct ace2k_feed_inputs *in)
{
    /* After a Klipper shutdown the link reads as down, whatever the transport says.  The clock
     * queries of the host keep the transport up, and nothing may start (ace2k_feed_shutdown). */
    struct ace2k_feed_inputs seen = *in;
    if (self->shut_down) {
        seen.link_ok = false;
    }
    self->last = seen;
    /* A tag search gives way to any other lane whose motor runs: a follow's burst, take-up, dose
     * or tail included, which no start announces (the tag search's rule 4).  The search yields on
     * this tick, or on the next one for a move that a later lane started on the last tick.  The
     * other lane's move is never held. */
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (ace2k_lane_is_moving(self->lane, i)) {
            ace2k_feed_yield_others(self, i);
        }
    }
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (!seen.link_ok) {
            /* An arming that no start took does not outlive the host that sent it (rule 6). */
            self->l[i].stop_next = (struct ace2k_feed_stop_on){ .held = false, .at_rest = false };
        }
        tick_lane(self, i, &seen, now_ms);
    }
    self->primed = true;
    self->insert_was_unknown = seen.insert_unknown;
}

void ace2k_feed_shutdown(struct ace2k_feed *self, uint32_t now_ms)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        /* A SHUTDOWN result that the lane's handler left, or any other result not yet read.  This
         * is the lane's one event, so on_result never sees it. */
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
    /* Idle or error: the lines go to stop, whatever the state (rule 5), with no event and no mode
     * change. */
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
    /* A clear ends no mode.  The retry window stays the one of the failure.  So a start that
     * repeats the failed one long after it is a new command, not its retry. */
    go_idle(self, lane, self->l[lane].t_end_ms);
    self->l[lane].error_kind = 0;
    return true;
}

enum ace2k_feed_refusal ace2k_feed_set_speed(struct ace2k_feed *self, uint8_t lane,
                                             uint32_t speed_um_s, uint32_t now_ms)
{
    /* Check the bounds first.  A speed out of them is bounds, whatever the lane's mode. */
    if (lane >= ACE2K_LANE_COUNT || !ace2k_lane_speed_in_bounds(speed_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode == ACE2K_FEED_ERROR) {
        return ACE2K_FEED_REFUSED_IN_ERROR;
    }
    if (!ace2k_feed_mode_runs(l->mode)) {
        return ACE2K_FEED_REFUSED_NO_MOVE;
    }
    /* The follow keeps the speed it was armed with.  Its speed changes only when it is armed
     * again, so the host that armed it always knows the speed it runs at. */
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        return ACE2K_FEED_REFUSED_FOLLOWING;
    }
    /* Only the command's own move takes the speed at once.  A move that the mode makes for itself
     * keeps the speed it was started at.  Examples are the reacquire of a load, backwards at the
     * floor speed, the touch of a snag, and a dose of the follow.  The dose cannot reach here,
     * because the follow was refused above.  The speed is kept for the next command move of the
     * mode all the same.  The bounds are checked above.  So the lane answers 0 when a motor move
     * took the speed, or -ACE2K_EREFUSED when no motor move runs at this instant.  That is an
     * assist between two bursts, or a load in its settle. */
    if (l->sub != ACE2K_FEED_SUB_REACQUIRE && l->snag != ACE2K_FEED_SNAG_BACK && !l->dose) {
        (void)ace2k_lane_set_speed(self->lane, lane, speed_um_s, now_ms);
    }
    l->speed_um_s = speed_um_s;
    return ACE2K_FEED_ACCEPTED;
}

int ace2k_feed_unload_set(struct ace2k_feed *self, uint32_t clear_um)
{
    if (!ace2k_within_u32(clear_um, ACE2K_FEED_UNLOAD_CLEAR_UM_MIN,
                          ACE2K_FEED_UNLOAD_CLEAR_UM_MAX)) {
        return -ACE2K_EINVAL;
    }
    self->unload_clear_um = clear_um;
    return 0;
}
