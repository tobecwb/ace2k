/* The feed's verdicts, read from the buffer and the counters, with the rules chosen from tests on
 * the unit.
 * - This lane's full, out of the shared pulled reading.
 * - The jam ahead, seen at once.
 * - The kind of a comparator trip.
 * - The tail past the drive: a reverse move whose filament stands still, or misses its deadline,
 *   with the plunger at rest.
 * This was split from feed.c, which keeps the modes, the tick and the events.  The seam is
 * feed_internal.h.  It runs in the feed's tick, in interrupt context. */
#include "feed/feed_internal.h"

/* The modes whose moves run toward the spool.  This is what "along the command" means once the move
 * has ended and the lane no longer knows its direction. */
static bool is_reverse_mode(uint8_t mode)
{
    if (mode == ACE2K_FEED_ASSISTING_BACK) {
        return true;
    }
    return ace2k_feed_mode_rewinds(mode);
}

/* The direction that the motor of the lane turns, also once the move has ended.  It comes from the
 * mode, or in the follow from its running or last move. */
static bool lane_is_reverse(const struct ace2k_feed_lane *l)
{
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        return l->follow_dir == ACE2K_LANE_REVERSE;
    }
    return is_reverse_mode(l->mode);
}

/* A take-up of an assist, either of the reverse assist or of the follow toward the spool, waits for
 * the switches where an unload or a rollback runs its tail.  The move has ended, and the lane
 * waits.  It is reached with the plunger read at rest.  So in the follow, that is the pass through
 * rest of the follow's rule 1, and its clock starts. */
static bool take_up_waits(struct ace2k_feed_lane *l, uint32_t now_ms)
{
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        l->follow_end_ms = now_ms;
        l->follow_dir = ACE2K_FEED_FOLLOW_DIR_NONE;
    } else if (l->mode != ACE2K_FEED_ASSISTING_BACK) {
        return false;
    }
    l->sub = ACE2K_FEED_SUB_WAITING;
    return true;
}

/* The modes that push the filament toward the head.  The buffer full while their motor runs is a
 * jam ahead (the tick, below). */
static bool pushes_toward_head(uint8_t mode, uint8_t sub)
{
    if (mode == ACE2K_FEED_FEEDING) {
        return true;
    }
    /* The load's pull and its tag search push toward the head.  The search takes the tip snag too:
     * the free phase of a load from the bay, or the watch of the read with motion.  Its stuck
     * verdicts become the obstacle return (feed_snag.c).  The reacquire moves away from the head.
     * The return moves away from it too.  After a reacquire that reversed below the return point,
     * the return moves forward to that point instead.  That is at most the length of the reacquire,
     * over filament that the pull already fed (feed_load.c, begin_return).  A pause and the settle
     * move nothing. */
    if (mode != ACE2K_FEED_LOADING) {
        return false;
    }
    if (sub == ACE2K_FEED_SUB_MOVING) {
        return true;
    }
    return sub == ACE2K_FEED_SUB_SEARCH;
}

/* This lane's buffer is full.  That is the shared pulled reading, with this lane's own plunger off
 * its rest and not at its pushed end.  pulled_any is one input for the four lanes.  A plunger still
 * at rest, or taut, is not the one at its pulled end.  Then the full belongs to another lane, and
 * is no verdict on this one. */
bool ace2k_feed_lane_full(const struct ace2k_feed_inputs *in, uint8_t lane)
{
    return (bool)(in->pulled_any && !ace2k_feed_bit(in->rest, lane) &&
                  !ace2k_feed_bit(in->pushed, lane));
}

/* A reverse move whose filament stands still, with the plunger at rest.  Nothing pulls on the
 * buffer, so nothing is held ahead.  The end of the filament is past the drive gear, the counters
 * have said what they can, and only the spool can move the filament now (measured on lane 2).
 * - The reverse assist, and the take-up of the follow, just wait for the switches.
 * - An unload goes on for ACE2K_FEED_UNLOAD_TAIL_UM of motor from the first such tick.
 * - A rollback goes on for what its length still owes (tail_mark).
 * They go on until the insert clears, or that budget is spent (tail_step).  This is reached from
 * the comparator's standstill while the move runs, and from the lane's own deadline once the move
 * has ended (below). */
static void tail_mark(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->tail_out) {
        return;
    }
    l->tail_out = true;
    l->fg_tail_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->tail_budget_um = ACE2K_FEED_UNLOAD_TAIL_UM;
    if (l->mode == ACE2K_FEED_ROLLING_BACK) {
        /* The length less the travel done.  That is the travel of the filament on the encoder from
         * the mode's start, plus the motor's travel since the filament last moved.  The second part
         * is travel that the encoder did not see: a filament past the drive that the spool winds,
         * or a lost encoder.  So a rollback never pulls more than its length, apart from the
         * standstill's own motor.  Up to v0.12.0 a rollback had the 100 mm of the unload too. */
        uint32_t along_um = ace2k_feed_strand_along_um(self, lane, l->enc_um_start, true);
        uint32_t unseen_um = ace2k_feed_motor_since_um(self, lane, l->fg_still_pulses);
        uint64_t done_um = (uint64_t)along_um + unseen_um;
        uint32_t owed_um = l->length_um > done_um ? (uint32_t)(l->length_um - done_um) : 0U;
        if (owed_um < l->tail_budget_um) {
            l->tail_budget_um = owed_um;
        }
    }
}

/* The motor turned since the tail was marked. */
static uint32_t tail_spent_um(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_feed_motor_since_um(self, lane, self->l[lane].fg_tail_pulses);
}

/* The comparator's standstill with the plunger at rest.  The running move goes on as the tail.  A
 * take-up of an assist ends instead, and the lane waits. */
static void tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (take_up_waits(l, now_ms)) {
        ace2k_feed_stop_discard(self, lane, now_ms);
        return;
    }
    tail_mark(self, lane);
}

/* The end of the filament left the encoder in the follow's tail (ACE2K_FEED_TAIL_ENC_UM).  The
 * filament's signed net travel since the tail began, retractions of the head included, reached the
 * path from the insert sensor to the encoder.  And through the move whose deadline fired, the
 * filament stood still while the motor turned. */
static bool tail_end_left_encoder(const struct ace2k_feed *self, uint8_t lane,
                                  const struct ace2k_lane_result *r)
{
    int32_t net_um =
        ace2k_lane_counts_since_um(self->lane, lane, self->l[lane].tail_enc_start_counts);
    int32_t still_um = (int32_t)ACE2K_FEED_STANDSTILL_UM;
    bool move_still = (bool)(r->filament_um > -still_um && r->filament_um < still_um);
    return (bool)(net_um >= (int32_t)ACE2K_FEED_TAIL_ENC_UM && move_still &&
                  r->motor_um >= ACE2K_FEED_STANDSTILL_UM);
}

/* The deadline of the lane passed.
 * - In the follow's tail, when the end has left the encoder (tail_end_left_encoder), a move that
 *   cannot reach its length is the end leaving the drive, whatever the move and the buffer.  That
 *   is tail_out, as a comparator trip in the tail is.  A forward dose of the feed-forward is the
 *   usual case: its few millimetres never count once the end has left the encoder (observed on the
 *   unit).  Otherwise the deadline is a slip or a lost encoder: the timeout, as before the tail.
 * - Outside the tail, nothing changes.  With the plunger at rest, in a reverse mode, the filament
 *   did not travel the length because nothing is left to drive.  That is the same tail, not a
 *   timeout.  The move has ended, so a take-up of an assist is already waiting.  An unload or a
 *   rollback continues its tail on a fresh bounded move, for what the budget still allows.  The own
 *   deadline of that move covers turning that much.  Rule 2 holds, because every run is a lane
 *   move.
 * It returns false when the deadline is a timeout after all: in the tail without the end leaving;
 * out of the tail, a forward mode, the filament taut, or the plunger between its switches. */
bool ace2k_feed_tail_after_deadline(struct ace2k_feed *self, uint8_t lane,
                                    const struct ace2k_lane_result *r,
                                    const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->tail) {
        if (!tail_end_left_encoder(self, lane, r)) {
            return false;
        }
        ace2k_feed_finish(self, lane, ACE2K_FEED_TAIL_OUT, now_ms);
        return true;
    }
    if (!lane_is_reverse(l) || !ace2k_feed_bit(in->rest, lane)) {
        return false;
    }
    if (take_up_waits(l, now_ms)) {
        return true;
    }
    tail_mark(self, lane);
    uint32_t spent_um = tail_spent_um(self, lane);
    uint32_t budget_um = l->tail_budget_um;
    if (spent_um >= budget_um ||
        ace2k_lane_move(self->lane, lane, ACE2K_LANE_REVERSE, budget_um - spent_um, l->speed_um_s,
                        now_ms) != 0) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
    }
    return true;
}

/* The tail budget is spent: the spool did not wind the end out.  The reverse move could not go on.
 * The filament is still at the sensor, and nothing is left to drive.  That is unload_incomplete,
 * whichever way the budget ran out.  It is an error, because a hand is needed. */
void ace2k_feed_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (tail_spent_um(self, lane) >= self->l[lane].tail_budget_um) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
    }
}

/* The bound of the tail of the follow.  follow.tail_um of motor since the tail began ends it, as
 * its standstill does: tail_out, the lane idle, no error. */
bool ace2k_feed_follow_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    uint32_t spent_um = ace2k_feed_motor_since_um(self, lane, self->l[lane].tail_fg_start_pulses);
    if (spent_um < self->follow.tail_um) {
        return false;
    }
    ace2k_feed_finish(self, lane, ACE2K_FEED_TAIL_OUT, now_ms);
    return true;
}

/* A feed, or the pull of a load to its parking point.  These are the pushes whose obstacle is
 * blocked, not stuck.  The filament waits where it stopped, and the lane is idle.  The tag search
 * past the parking point has its own end, the obstacle return (feed_load.c).  A rollback, an
 * unload, the assists and the follow keep their verdicts. */
bool ace2k_feed_block(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode == ACE2K_FEED_LOADING) {
        if (l->sub != ACE2K_FEED_SUB_MOVING) {
            return false;
        }
    } else if (l->mode != ACE2K_FEED_FEEDING) {
        return false;
    }
    ace2k_feed_finish(self, lane, ACE2K_FEED_BLOCKED, now_ms); /* the mode's odometers */
    return true;
}

/* The comparator tripped.
 * - In the follow's tail: the tail-out.
 * - In a feed, or in the pull of a load: blocked.
 * - In the tag search: the obstacle return.
 * Elsewhere the kind comes from the buffer.  A forward move with the buffer of this lane full
 * cannot push into the head (stuck).  Without that reading, the filament does not come from the
 * spool, because of a tangle or an end tied to the spool (tangled).
 * Toward the spool, the filament taut (pushed) is a tip held ahead, in a hot end or at the exit of
 * a tube, that the drive grinds against.  That is stuck, at once (measured on the unit).  A
 * standstill with the plunger at rest is the tail past the drive (above).  Any other trip is stuck,
 * unless the filament came back freely first and now stands taut.  That is the tip snag's reverse
 * case (feed_snag.c). */
void ace2k_feed_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                        const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (self->l[lane].tail) {
        /* The follow's tail: every trip, the partial as the standstill, whatever the buffer reads.
         * The filament no longer keeps up with the motor because the tail is leaving the drive.
         * That is the end that the tail runs for, not an error. */
        ace2k_feed_finish(self, lane, ACE2K_FEED_TAIL_OUT, now_ms);
        return;
    }
    if (self->l[lane].mode == ACE2K_FEED_LOADING && ace2k_lane_is_reverse(self->lane, lane)) {
        /* The return or the reacquire of the load: a filament that will not come back is held.  It
         * is never the tail of the unload, which is for a filament past the drive gear. */
        ace2k_feed_fail(self, lane, ACE2K_FEED_STUCK, now_ms);
        return;
    }
    if (!ace2k_lane_is_reverse(self->lane, lane)) {
        if (ace2k_feed_block(self, lane, now_ms)) {
            return; /* the obstacle of a feed or of a load's pull: blocked */
        }
        if (self->l[lane].mode == ACE2K_FEED_LOADING &&
            self->l[lane].sub == ACE2K_FEED_SUB_SEARCH) {
            /* The tag search: the obstacle return to the parking point, never an error. */
            ace2k_feed_load_search_return(self, lane, now_ms);
            return;
        }
        ace2k_feed_fail(self, lane,
                        ace2k_feed_lane_full(in, lane) ? ACE2K_FEED_STUCK : ACE2K_FEED_TANGLED,
                        now_ms);
        return;
    }
    if (trip == ACE2K_FEED_TRIP_STANDSTILL && ace2k_feed_bit(in->rest, lane)) {
        tail_begin(self, lane, now_ms);
        return;
    }
    if (ace2k_feed_snag_reverse(self, lane, in, now_ms)) {
        return; /* the tip caught the buffer's tube on its way back: one touch forward */
    }
    ace2k_feed_fail(self, lane, ACE2K_FEED_STUCK, now_ms);
}

/* The buffer of this lane full, while the filament is pushed toward the head with the motor
 * running, is the fast signal of a jam ahead.  The ≈ 18 mm of travel of the plunger, at the force
 * of the spring, is the early warning.  The 20 mm of motor of the standstill would come after the
 * filament has buckled in the head.  Observed on the unit: with the outlet blocked, a 300 mm feed
 * ran to done at full force.  This applies to a feed, the pull of a load and its tag search.  A
 * rollback or an unload moves away from the head, the assist stops on this reading by design, and
 * the settle of a load moves nothing. */
bool ace2k_feed_full_ahead(const struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in)
{
    if (!pushes_toward_head(self->l[lane].mode, self->l[lane].sub) ||
        !ace2k_feed_lane_full(in, lane)) {
        return false;
    }
    return ace2k_lane_is_moving(self->lane, lane);
}

bool ace2k_feed_pushing_ahead(const struct ace2k_feed *self, uint8_t lane)
{
    if (!pushes_toward_head(self->l[lane].mode, self->l[lane].sub) ||
        !ace2k_lane_is_moving(self->lane, lane)) {
        return false;
    }
    return (bool)!ace2k_lane_is_reverse(self->lane, lane);
}
