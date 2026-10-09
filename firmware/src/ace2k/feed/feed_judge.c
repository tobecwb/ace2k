/* The verdicts of the feed read from the buffer and the counters (the rules decided at the bench): this lane's full out of the
 * shared pulled reading, the jam ahead seen at once, the kind of a comparator's trip, and the
 * tail past the drive — a reverse move whose strand stands still, or misses its deadline, with the
 * plunger at rest.  Split from feed.c, which keeps the modes, the tick and the events; the seam
 * is feed_internal.h.  Runs in the feed's tick, interrupt context. */
#include "feed/feed_internal.h"

/* The modes whose moves run toward the spool — what "along the command" means once the move has
 * ended and the lane no longer knows its direction. */
static bool is_reverse_mode(uint8_t mode)
{
    switch (mode) {
    case ACE2K_FEED_ROLLING_BACK:
    case ACE2K_FEED_UNLOADING:
    case ACE2K_FEED_ASSISTING_BACK:
        return true;
    default:
        return false;
    }
}

/* The direction the lane's motor turns, once the move has ended too: from the mode, or in the
 * follow from its running — or last — move. */
static bool lane_is_reverse(const struct ace2k_feed_lane *l)
{
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        return l->follow_dir == ACE2K_LANE_REVERSE;
    }
    return is_reverse_mode(l->mode);
}

/* A take-up of an assist — the reverse assist's, or the follow's toward the spool — waits for
 * the switches where an unload or a rollback runs its tail: the move has ended, the lane waits.
 * Reached with the plunger read at rest, so in the follow that is the follow's rule 1 pass through rest,
 * and its clock starts. */
static bool take_up_waits(struct ace2k_feed_lane *l, uint32_t now_ms)
{
    if (l->mode == ACE2K_FEED_FOLLOWING) {
        l->follow_end_ms = now_ms;
        l->follow_dir = ACE2K_FOLLOW_DIR_NONE;
    } else if (l->mode != ACE2K_FEED_ASSISTING_BACK) {
        return false;
    }
    l->sub = ACE2K_FEED_SUB_WAITING;
    return true;
}

/* The modes that push the strand toward the head: the buffer full while their motor runs is a
 * jam ahead (the tick, below). */
static bool pushes_toward_head(uint8_t mode, uint8_t sub)
{
    if (mode == ACE2K_FEED_FEEDING) {
        return true;
    }
    /* the load's pull and its tag search: the search takes the tip snag's tolerance too, its
     * stuck verdicts the obstacle return (feed_snag.c); the
     * reacquire moves away from the head; the return moves away from it too, or — after a
     * reacquire that reversed below the return point — forward to that point, at most the
     * reacquire's length over strand the pull already fed (feed_load.c, begin_return); a pause
     * and the settle move nothing */
    if (mode != ACE2K_FEED_LOADING) {
        return false;
    }
    if (sub == ACE2K_FEED_SUB_MOVING) {
        return true;
    }
    return sub == ACE2K_FEED_SUB_SEARCH;
}

/* This lane's buffer full: the shared pulled reading with this lane's own plunger off its rest
 * and not at its pushed end.  pulled_any is one input for the four lanes: a plunger still at rest, or taut, is not the one at its pulled end, so
 * the full is another lane's and no verdict on this one. */
bool ace2k_feed_lane_full(const struct ace2k_feed_inputs *in, uint8_t lane)
{
    if (!in->pulled_any || ace2k_feed_bit(in->rest, lane) || ace2k_feed_bit(in->pushed, lane)) {
        return false;
    }
    return true;
}

/* A reverse move whose strand stands still with the plunger at rest: nothing pulls on the
 * buffer, so nothing is held ahead — the tail is past the drive gear, the counters have said what
 * they can, and only the spool can move the strand now (measured
 * on lane 2).  The reverse assist, and the follow's take-up, just wait for the
 * switches; an unload or a rollback goes on for ACE2K_FEED_UNLOAD_TAIL_UM of motor from the first
 * such tick, until the insert clears or that budget is spent (tail_step).  Reached from the
 * comparator's standstill while the move runs, and from the lane's own deadline once the move has
 * ended (below). */
static void tail_mark(struct ace2k_feed_lane *l, uint32_t fg_raw)
{
    if (!l->tail_out) {
        l->tail_out = true;
        l->fg_tail = fg_raw;
    }
}

/* The motor turned since the tail was marked. */
static uint32_t tail_spent_um(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - self->l[lane].fg_tail);
}

/* The comparator's standstill with the plunger at rest: the running move goes on as the tail —
 * an assist's take-up ends instead, the lane waiting. */
static void tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (take_up_waits(l, now_ms)) {
        ace2k_feed_stop_discard(self, lane, now_ms);
        return;
    }
    tail_mark(l, self->lane->lane[lane].fg_raw);
}

/* The lane's deadline passed with the plunger at rest, in a reverse mode: the strand did not
 * travel the length because nothing is left to drive — the same tail, not a timeout.  The move
 * has ended, so an assist's take-up is already waiting; an unload or a rollback continues its
 * tail on a fresh bounded move for what the budget still allows, whose own deadline covers
 * turning that much (rule 2 holds: every run is a lane move).  False
 * when the deadline is a timeout after all: a forward mode, the strand taut, the plunger between
 * its switches. */
bool ace2k_feed_tail_after_deadline(struct ace2k_feed *self, uint8_t lane,
                                    const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!lane_is_reverse(l) || !ace2k_feed_bit(in->rest, lane)) {
        return false;
    }
    if (take_up_waits(l, now_ms)) {
        return true;
    }
    tail_mark(l, self->lane->lane[lane].fg_raw);
    uint32_t spent_um = tail_spent_um(self, lane);
    if (spent_um >= ACE2K_FEED_UNLOAD_TAIL_UM ||
        ace2k_lane_move(self->lane, lane, ACE2K_LANE_REVERSE, ACE2K_FEED_UNLOAD_TAIL_UM - spent_um,
                        l->speed_um_s, now_ms) != 0) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
    }
    return true;
}

/* The tail budget spent: the spool did not wind the tail out.  The reverse move could not go on
 * — the strand is still at the sensor and nothing is left to drive — which is unload_incomplete
 * whichever way the budget ran out: an error, a hand is needed. */
void ace2k_feed_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    if (tail_spent_um(self, lane) >= ACE2K_FEED_UNLOAD_TAIL_UM) {
        ace2k_feed_fail(self, lane, ACE2K_FEED_UNLOAD_INCOMPLETE, now_ms);
    }
}

/* The follow's tail bound: follow.tail_um of motor since the tail
 * began ends it as its standstill does — tail_out, the lane idle, no error. */
bool ace2k_feed_follow_tail_step(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    uint32_t spent_um =
        ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - self->l[lane].tail_fg_start);
    if (spent_um < self->follow.tail_um) {
        return false;
    }
    ace2k_feed_finish(self, lane, ACE2K_FEED_TAIL_OUT, now_ms);
    return true;
}

/* A feed, or a load's pull to its parking point: the pushes whose obstacle is blocked, not stuck
 * — the strand waits where it stopped, the lane idle.  The tag search
 * past the parking point has its own end, the obstacle return (feed_load.c); a rollback, an
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

/* The comparator tripped: in the follow's tail, the tail-out; in a feed or a load's pull, blocked;
 * in the tag search, the obstacle return.  Elsewhere the kind comes from the buffer.
 * A forward move with this lane's buffer full cannot push into the head (stuck); without that
 * reading the strand does not come from the spool — a tangle, a tail tied to it (tangled).  Toward the
 * spool: the strand taut (pushed) is a tip held ahead — a hot end, a tube's exit — that the drive
 * grinds against (stuck, at once: measured on the unit); a standstill with the
 * plunger at rest is the tail past the drive (above), any other trip stuck — unless the strand
 * came back freely first and now stands taut, the tip snag's reverse case (feed_snag.c). */
void ace2k_feed_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                        const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (self->l[lane].tail) {
        /* the follow's tail: every trip, the partial as the standstill, whatever the buffer reads
         * — the strand no longer keeps up with the motor because the tail is leaving the drive,
         * the end the tail runs for, not an error */
        ace2k_feed_finish(self, lane, ACE2K_FEED_TAIL_OUT, now_ms);
        return;
    }
    if (self->l[lane].mode == ACE2K_FEED_LOADING && ace2k_lane_is_reverse(self->lane, lane)) {
        /* the load's return or reacquire: a strand that will not come back is held — never the
         * unload's tail, which is for a strand past the drive gear */
        ace2k_feed_fail(self, lane, ACE2K_FEED_STUCK, now_ms);
        return;
    }
    if (!ace2k_lane_is_reverse(self->lane, lane)) {
        if (ace2k_feed_block(self, lane, now_ms)) {
            return; /* a feed's or a load's pull's obstacle: blocked */
        }
        if (self->l[lane].mode == ACE2K_FEED_LOADING &&
            self->l[lane].sub == ACE2K_FEED_SUB_SEARCH) {
            /* the tag search: the obstacle return to the parking point, never an error */
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

/* This lane's buffer full while the strand is pushed toward the head with the motor running is
 * the fast signal of a jam ahead: the plunger's ≈ 18 mm of travel at the spring's force is the
 * early warning, where the standstill's 20 mm of motor would come after the strand has buckled in
 * the head (observed on the bench: a blocked outlet let a 300 mm feed run to done at full force).
 * A feed, a load's pull and its tag search — a rollback or an unload moves away from the head, the
 * assist stops on this reading by design, and a load's settle moves nothing. */
bool ace2k_feed_full_ahead(const struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in)
{
    if (!pushes_toward_head(self->l[lane].mode, self->l[lane].sub) ||
        !ace2k_feed_lane_full(in, lane)) {
        return false;
    }
    return ace2k_lane_is_moving(self->lane, lane);
}
