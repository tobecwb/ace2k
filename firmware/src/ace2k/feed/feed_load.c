/* The feed's load.  It holds these phases.
 * - The settle.
 * - The pull to the parking point.
 * - With the reader's binding, the pause while a read session holds the lane.
 * - The search past the parking point, for a tag not seen yet.
 * - The return to the parking point.
 * - The reacquire of a tag that a stop left behind.
 * - The read on command with motion.
 * - The grip: an automatic load cut to a short pull, with no search.
 * - With the reader's binding, the confirmation of a candidate tag: the lane moves back and forth,
 *   one bounded leg at a time, while the watch looks whether the tag follows it.
 * Every phase is one bounded lane move (the feed's rule 2).  A STOP, the link, a shutdown, a runout
 * and the supervisors end it, as they end any mode.  The seam is feed_internal.h.  The tick runs
 * in interrupt context, and the starts run from task context with interrupts masked. */
#include "feed/feed_internal.h"
#include "rfid/watch.h" /* ace2k_rfid_neighbour, ace2k_rfid_reader_of: the lanes on a reader */

/* The travel of the filament since the load began, on the encoder. */
static int32_t travel_um(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_feed_strand_since_um(self, lane, self->l[lane].enc_um_start);
}

/* The tag search's rule 4: the search moves only while every other lane is quiet.  Idle, error
 * and a lane the host has in use (a print's) are quiet.  With follow_at_rest_quiet, so is a follow
 * with its motor at rest: the read with motion and a resumed search take it so, a load's search
 * after its pull does not.  A follow that moves later makes the search yield (ace2k_feed_tick).
 * Without it (a load's search after its pull), so is a tie-break on the other reader. */
static bool others_quiet(const struct ace2k_feed *self, uint8_t lane, bool follow_at_rest_quiet)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (i == lane || !ace2k_feed_lane_busy(self, i) || ace2k_feed_bit(self->last.in_use, i)) {
            continue;
        }
        if (!follow_at_rest_quiet && ace2k_feed_tiebreak_on(&self->l[i]) &&
            ace2k_rfid_reader_of(i) != ace2k_rfid_reader_of(lane)) {
            continue;
        }
        if (!follow_at_rest_quiet || self->l[i].mode != ACE2K_FEED_FOLLOWING ||
            ace2k_lane_is_moving(self->lane, i)) {
            return false;
        }
    }
    return true;
}

/* A move of the load itself, with the bases of the comparator taken afresh.  It returns false when
 * the lane refused it, and then the caller ends the load.  A forward move takes up the free phase
 * that a reverse leg of a confirmation set aside. */
static bool load_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir, uint32_t um,
                      uint32_t speed_um_s, uint8_t sub, uint32_t now_ms)
{
    if (um > ACE2K_LANE_MOVE_MAX_UM) {
        um = ACE2K_LANE_MOVE_MAX_UM;
    }
    if (ace2k_lane_move(self->lane, lane, dir, um, speed_um_s, now_ms) != 0) {
        return false;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    l->sub = sub;
    ace2k_feed_compare_restart(self, lane);
    if (dir == ACE2K_LANE_FORWARD && l->free_held) {
        /* the push goes on after a reverse leg of a confirmation: its free phase with it */
        l->free_held = false;
        if (!ace2k_feed_bit(self->last.rest, lane)) {
            ace2k_feed_snag_free_resume(self, lane);
        }
    }
    return true;
}

/* The load ends. */
static void loaded(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    ace2k_feed_finish(self, lane, ACE2K_FEED_LOADED, now_ms);
}

/* Go back to the return point: the parking point, or the start of a read.  This works from either
 * side (rule 5 of the tag search: a search always ends there, a yield included).  A filament past
 * the point comes back.  A filament short of it, after a reacquire that reversed below it, goes
 * forward to it, never past it.  The tag search's rule 4 allows that while another lane moves.
 * Within ACE2K_FEED_RETURN_MIN_UM, the filament is already there. */
static void begin_return(struct ace2k_feed *self, uint8_t lane, uint8_t end, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    ace2k_feed_stop_discard(self, lane, now_ms);
    if (l->sub == ACE2K_FEED_SUB_CONFIRM) {
        /* The load leaves a confirmation that the watch still asks for: its leg is out, so the
         * watch gives the confirmation up now rather than at the end of the load. */
        l->confirm_out = true;
    }
    l->search_end = end;
    l->yield = false;
    int32_t over = travel_um(self, lane) - (int32_t)l->return_um;
    enum ace2k_lane_dir dir = over > 0 ? ACE2K_LANE_REVERSE : ACE2K_LANE_FORWARD;
    uint32_t um = over > 0 ? (uint32_t)over : (uint32_t)-over;
    if (um < ACE2K_FEED_RETURN_MIN_UM ||
        !load_move(self, lane, dir, um, l->speed_um_s, ACE2K_FEED_SUB_RETURN, now_ms)) {
        loaded(self, lane, now_ms);
    }
}

/* The reach of the lane's search, from where its load or read began.  It is the host's search_um,
 * and the confirmation's longest leg for the unit's own tie-break: the tie-break only has to move
 * the lane enough for a tag to follow it, and stays far short of a shared head's splitter. */
static uint32_t search_reach_um(const struct ace2k_feed *self, uint8_t lane)
{
    return self->l[lane].autonomous ? self->confirm_leg_um : self->search_um;
}

static void begin_search(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->searched = true;
    /* The search's reach counts from where the load, or the read with motion, began. */
    int32_t left = (int32_t)search_reach_um(self, lane) - travel_um(self, lane);
    if (left < (int32_t)ACE2K_FEED_RETURN_MIN_UM) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_LENGTH, now_ms);
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_FORWARD, (uint32_t)left, l->speed_um_s,
                   ACE2K_FEED_SUB_SEARCH, now_ms)) {
        /* Not reached: the lane has stopped, the reach is clamped to a move, and the speed was
         * checked at the start.  Still, loaded is what happened.  The pull is over, the filament
         * waits where it stopped, and search_end stays none, because no search ran. */
        loaded(self, lane, now_ms);
    }
}

/* The load has a search to run past its pull: not a grip or a park, a reach past the parking
 * point, its tag not read yet, and the lane not set by hand (it is never given a tag). */
static bool search_wanted(const struct ace2k_feed *self, uint8_t lane,
                          const struct ace2k_feed_inputs *in)
{
    return (bool)!(self->l[lane].gripped || self->search_um <= self->l[lane].return_um ||
                   ace2k_feed_bit(in->tag_read, lane) || ace2k_feed_bit(in->tag_by_hand, lane));
}

static bool should_search(const struct ace2k_feed *self, uint8_t lane,
                          const struct ace2k_feed_inputs *in)
{
    /* a load's search: every other lane idle */
    return (bool)(search_wanted(self, lane, in) && others_quiet(self, lane, false));
}

/* After the pull: the search, or the end.  A tie-break on the same reader is not quiet, so the load
 * skips its search; the lane stays pending for the watch. */
static void after_pull(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                       uint32_t now_ms)
{
    if (should_search(self, lane, in)) {
        begin_search(self, lane, now_ms);
        return;
    }
    self->l[lane].search_end = ACE2K_FEED_SEARCH_NONE;
    loaded(self, lane, now_ms);
}

/* The search goes on for what is left of it.  There are two exceptions, and both go straight back.
 * One is a tag read meanwhile, during the watch of the read with motion, which holds the load's
 * tick.  Then the search ends as found.  The other is a start given to another lane, or another
 * lane's motor that ran, while the search was paused or reacquiring (the tag search's rule 4).
 * The search never goes forward past the parking point while another lane moves. */
static void resume_search(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    if (ace2k_feed_bit(in->tag_read, lane)) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_FOUND, now_ms);
        return;
    }
    if (self->l[lane].yield || !others_quiet(self, lane, true)) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
        return;
    }
    begin_search(self, lane, now_ms);
}

/* The pull, or the search, goes on for what is left of it.  The lane is stopped. */
static void resume(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                   uint8_t sub, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (sub == ACE2K_FEED_SUB_SEARCH) {
        resume_search(self, lane, in, now_ms);
        return;
    }
    int32_t left = (int32_t)l->length_um - travel_um(self, lane);
    if (left < (int32_t)ACE2K_FEED_RETURN_MIN_UM) {
        after_pull(self, lane, in, now_ms);
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_FORWARD, (uint32_t)left, l->speed_um_s,
                   ACE2K_FEED_SUB_MOVING, now_ms)) {
        loaded(self, lane, now_ms);
    }
}

/* Why the search ends, in the order of resume_search.  First its tag read on this tick's inputs,
 * during the watch of the read with motion, which holds the load's tick.  Then a pending yield (the
 * tag search's rule 4).  Otherwise the obstacle, not stuck. */
static uint8_t search_end_now(const struct ace2k_feed *self, uint8_t lane)
{
    if (ace2k_feed_bit(self->last.tag_read, lane)) {
        return ACE2K_FEED_SEARCH_FOUND;
    }
    return self->l[lane].yield ? ACE2K_FEED_SEARCH_YIELD : ACE2K_FEED_SEARCH_OBSTACLE;
}

void ace2k_feed_load_search_return(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    begin_return(self, lane, search_end_now(self, lane), now_ms);
}

/* The pause has no bound of its own.  It lasts while the reader's binding holds the lane for its
 * read session, and the session is bounded there.  The session ends after
 * ACE2K_RFID_SESSION_IDLE_MS without a host step, and at once when the link is lost (rfid/watch.c).
 * A stop, the link, a shutdown and a runout end the paused load here, as they end any mode.  The
 * motor is off throughout. */
static void pause_load(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    ace2k_feed_stop_discard(self, lane, now_ms); /* a pause is not an outcome */
    /* A reacquire or a confirmation already holds the phase it interrupted. */
    if (l->sub != ACE2K_FEED_SUB_REACQUIRE && l->sub != ACE2K_FEED_SUB_CONFIRM) {
        l->resume_sub = l->sub;
    }
    l->confirm_out = false;
    l->sub = ACE2K_FEED_SUB_PAUSED;
}

/* How far a reacquire may reverse: ACE2K_FEED_REACQUIRE_UM, and never behind the phase's own start.
 * That start is the start of the load during the pull, and the return point during a search.  The
 * return point is the parking point, or the start of a read.  So a reacquire cannot draw the tip
 * back past the insert sensor or out of the drive.  It cannot leave the filament of a search short
 * of where it must come back to. */
static uint32_t room_behind_um(const struct ace2k_feed *self, uint8_t lane)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    int32_t floor_um = l->resume_sub == ACE2K_FEED_SUB_SEARCH ? (int32_t)l->return_um : 0;
    int32_t room = travel_um(self, lane) - floor_um;
    return room > 0 ? (uint32_t)room : 0;
}

static uint32_t reacquire_um(const struct ace2k_feed *self, uint8_t lane)
{
    uint32_t room = room_behind_um(self, lane);
    return room < ACE2K_FEED_REACQUIRE_UM ? room : ACE2K_FEED_REACQUIRE_UM;
}

/* Once backwards at the floor speed, for a tag that a stop left behind.  With no room behind the
 * start of the phase, under ACE2K_FEED_RETURN_MIN_UM, there is no reverse.  The phase goes on as if
 * the tag had not been lost. */
static void reacquire(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->reacquired = true;
    uint32_t um = reacquire_um(self, lane);
    if (um < ACE2K_FEED_RETURN_MIN_UM) {
        resume(self, lane, in, l->resume_sub, now_ms);
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_REVERSE, um, ACE2K_LANE_SPEED_MIN_UM_S,
                   ACE2K_FEED_SUB_REACQUIRE, now_ms)) {
        loaded(self, lane, now_ms);
    }
}

/* The session released the lane.
 * - The tag was read in the search: go back.
 * - The tag was lost: go backwards once.
 * - Otherwise, read in the pull or given up: the phase goes on.
 * A search asked to yield returns. */
static void on_released(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    bool read = ace2k_feed_bit(in->tag_read, lane);
    bool in_search = l->resume_sub == ACE2K_FEED_SUB_SEARCH;
    if (in_search && (read || l->yield)) {
        begin_return(self, lane, read ? ACE2K_FEED_SEARCH_FOUND : ACE2K_FEED_SEARCH_YIELD, now_ms);
        return;
    }
    if (!read && ace2k_feed_bit(in->tag_lost, lane) && !l->reacquired) {
        reacquire(self, lane, in, now_ms);
        return;
    }
    resume(self, lane, in, l->resume_sub, now_ms);
}

/* How far ahead a forward leg may go: never past the end of the phase it interrupted.  That is the
 * parking point (or the grip) in the pull, and the search's reach in the search, which keeps the
 * filament short of the outlet. */
static uint32_t room_ahead_um(const struct ace2k_feed *self, uint8_t lane)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t end_um =
        l->resume_sub == ACE2K_FEED_SUB_SEARCH ? search_reach_um(self, lane) : l->length_um;
    int32_t room = (int32_t)end_um - travel_um(self, lane);
    return room > 0 ? (uint32_t)room : 0;
}

/* One leg of the confirmation, at its speed, in the direction the watch asks, for the confirmation
 * of the epoch it shows.  A leg is the confirmation's longest, cut to the room the phase leaves.
 * Backwards, that is the room above the phase's own start, as for the reacquire.  With under
 * ACE2K_FEED_RETURN_MIN_UM of room, nothing moves and the leg is out at once.  The lane holds still
 * until the watch lets it go. */
static void confirm_leg(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool reverse = ace2k_feed_bit(in->tag_confirm_rev, lane);
    ace2k_feed_stop_discard(self, lane, now_ms); /* the turn is not an outcome */
    l->sub = ACE2K_FEED_SUB_CONFIRM;
    l->confirm_rev = reverse;
    l->confirm_epoch = ace2k_feed_bit(in->tag_confirm_epoch, lane);
    l->confirm_out = false;
    if (reverse && l->snag == ACE2K_FEED_SNAG_FREE_TIP) {
        /* the free phase judges a push toward the head only: set aside until the next one */
        l->snag = ACE2K_FEED_SNAG_NONE;
        l->free_held = true;
    }
    uint32_t room = reverse ? room_behind_um(self, lane) : room_ahead_um(self, lane);
    uint32_t um = room < self->confirm_leg_um ? room : self->confirm_leg_um;
    if (um < ACE2K_FEED_RETURN_MIN_UM) {
        l->confirm_out = true;
        return;
    }
    if (!load_move(self, lane, reverse ? ACE2K_LANE_REVERSE : ACE2K_LANE_FORWARD, um,
                   self->confirm_speed_um_s, ACE2K_FEED_SUB_CONFIRM, now_ms)) {
        loaded(self, lane, now_ms); /* as any other move of the load the lane refuses */
    }
}

/* The watch let the confirmation go: the phase it interrupted goes on for what is left of it. */
static void confirm_end(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    ace2k_feed_stop_discard(self, lane, now_ms);
    resume(self, lane, in, self->l[lane].resume_sub, now_ms);
}

/* The tick of a confirmation, in this order.
 * - A read session holds the lane: the load pauses, and the pause goes back to the phase that the
 *   confirmation interrupted.
 * - The watch no longer asks: that phase goes on.
 * - A search asked to yield goes back, as from the search itself.
 * - The direction asked turned round, or a new confirmation began (its epoch): the next leg starts
 *   at once, so the lane is still for at most this one tick.
 * Otherwise the leg runs, or the lane holds still after a leg that ran out. */
static void confirm_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                         uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_feed_bit(in->tag_hold, lane)) {
        pause_load(self, lane, now_ms);
        return;
    }
    if (!ace2k_feed_bit(in->tag_confirm, lane)) {
        confirm_end(self, lane, in, now_ms);
        return;
    }
    if (l->resume_sub == ACE2K_FEED_SUB_SEARCH && l->yield) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
        return;
    }
    /* a new confirmation (its epoch) gets a fresh leg, even in the direction of the last one */
    if (ace2k_feed_bit(in->tag_confirm_rev, lane) != l->confirm_rev ||
        ace2k_feed_bit(in->tag_confirm_epoch, lane) != l->confirm_epoch) {
        confirm_leg(self, lane, in, now_ms);
    }
}

static void on_moving(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_feed_bit(in->tag_hold, lane)) {
        pause_load(self, lane, now_ms);
        return;
    }
    /* This lane's full belongs to the tip snag, in feed_snag.c, after this step on the tick.  For a
     * load from the bay, that is the free phase.  A lag gives the obstacle return, and a yield or a
     * pause runs here through the phase.  For the read with motion, it is the watch, which holds
     * this tick until it ends. */
    if (l->sub == ACE2K_FEED_SUB_SEARCH && l->yield) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
        return;
    }
    if (ace2k_feed_bit(in->tag_confirm, lane)) {
        l->resume_sub = l->sub; /* the pull or the search, which the confirmation interrupts */
        confirm_leg(self, lane, in, now_ms);
    }
}

/* The touch of the read with motion's tip snag freed the plunger during a forward leg.  The
 * confirmation goes on with a fresh leg in the direction asked now.  When the watch let it go
 * meanwhile, the phase it interrupted goes on. */
static void confirm_resume_after_snag(struct ace2k_feed *self, uint8_t lane,
                                      const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (!ace2k_feed_bit(in->tag_confirm, lane)) {
        confirm_end(self, lane, in, now_ms);
        return;
    }
    confirm_leg(self, lane, in, now_ms);
}

void ace2k_feed_load_resume_after_snag(struct ace2k_feed *self, uint8_t lane,
                                       const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    if (self->l[lane].sub == ACE2K_FEED_SUB_CONFIRM) {
        confirm_resume_after_snag(self, lane, in, now_ms);
        return;
    }
    resume(self, lane, in, self->l[lane].sub, now_ms); /* the snag never changes sub */
}

/* The settle, while the hand is still pushing, then the pull to the parking point. */
static void load_settle(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_time_since(now_ms, l->t_mode_ms) < ACE2K_FEED_LOAD_SETTLE_MS) {
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_FORWARD, l->length_um, l->speed_um_s,
                   ACE2K_FEED_SUB_MOVING, now_ms)) {
        /* Not reached: the lane was idle through the settle, and the bounds were checked at the
         * start.  Still, a load that cannot pull ends, and does not hang.  It ends the way a stop
         * during the settle ends it: stopped, with nothing moved. */
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED, now_ms);
    }
}

void ace2k_feed_load_preempt(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->sub == ACE2K_FEED_SUB_RETURN) {
        return;
    }
    l->snag = ACE2K_FEED_SNAG_NONE; /* as the snag's own end in the search: the tick is not held */
    begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
}

void ace2k_feed_load_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    if (!ace2k_feed_bit(in->tag_confirm, lane)) {
        self->l[lane].confirm_out = false; /* the watch no longer asks: nothing is out */
    }
    if (self->l[lane].autonomous &&
        (!in->in_use_valid || ace2k_feed_bit(in->in_use, lane) || self->l[lane].yield)) {
        /* The host took the lane for a print, or its mask is no longer valid, so nothing may move
         * on its own.  Or a start or a strand going in asked the tie-break to yield.  It goes back
         * now, the tip snag's phase included, without waiting for a read session, and never moves
         * the lane forward again. */
        ace2k_feed_load_preempt(self, lane, now_ms);
        return;
    }
    if (ace2k_feed_snag_active(self, lane)) {
        /* The watch, touch or wait of the read with motion owns the moves of the lane, so a read
         * session waits for it.  The free phase of a load from the bay only judges, and never holds
         * this tick.  A pause for a read session and a yield run through it, and end it
         * (feed_snag.c). */
        return;
    }
    switch (self->l[lane].sub) {
    case ACE2K_FEED_SUB_SETTLE:
        load_settle(self, lane, now_ms);
        break;
    case ACE2K_FEED_SUB_MOVING:
    case ACE2K_FEED_SUB_SEARCH:
        on_moving(self, lane, in, now_ms);
        break;
    case ACE2K_FEED_SUB_PAUSED:
        if (!ace2k_feed_bit(in->tag_hold, lane)) {
            on_released(self, lane, in, now_ms);
        }
        break;
    case ACE2K_FEED_SUB_REACQUIRE:
        if (ace2k_feed_bit(in->tag_hold, lane)) {
            pause_load(self, lane, now_ms); /* found back: the session; resume_sub kept */
        }
        break;
    case ACE2K_FEED_SUB_CONFIRM:
        confirm_tick(self, lane, in, now_ms);
        break;
    default:
        break; /* the return runs to the parking point */
    }
}

bool ace2k_feed_load_move_done(struct ace2k_feed *self, uint8_t lane,
                               const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    switch (l->sub) {
    case ACE2K_FEED_SUB_MOVING:
        after_pull(self, lane, in, now_ms);
        return true;
    case ACE2K_FEED_SUB_SEARCH:
        begin_return(self, lane, ACE2K_FEED_SEARCH_LENGTH, now_ms);
        return true;
    case ACE2K_FEED_SUB_REACQUIRE:
        resume(self, lane, in, l->resume_sub, now_ms); /* not found back */
        return true;
    case ACE2K_FEED_SUB_CONFIRM:
        l->confirm_out = true; /* the leg ran its length: held still until the watch lets go */
        return true;
    default:
        return false; /* the return reached the parking point, or where the read began */
    }
}

void ace2k_feed_yield_others(struct ace2k_feed *self, uint8_t lane)
{
    if (ace2k_feed_bit(self->last.in_use, lane)) {
        return; /* a print's move: the search beside it goes on */
    }
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (i != lane && self->l[i].mode == ACE2K_FEED_LOADING && self->l[i].searched) {
            self->l[i].yield = true;
        }
    }
}

void ace2k_feed_yield_neighbour_tiebreak(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *n = &self->l[ace2k_rfid_neighbour(lane)];
    if (ace2k_feed_tiebreak_on(n)) {
        n->yield = true;
    }
}

static void reset_load(struct ace2k_feed_lane *l, uint32_t return_um)
{
    l->return_um = return_um;
    l->search_end = ACE2K_FEED_SEARCH_NONE;
    l->searched = false;
    l->yield = false;
    l->reacquired = false;
    l->gripped = false;
    l->resume_sub = ACE2K_FEED_SUB_NONE;
    l->confirm_rev = false;
    l->confirm_out = false;
}

/* The grip slot holds a pull length, not one of its reserved values (feed.h). */
static bool grip_is_length(uint32_t grip_um)
{
    return (bool)(grip_um != 0 && grip_um != ACE2K_FEED_GRIP_PARK &&
                  grip_um != ACE2K_FEED_GRIP_WAIT);
}

/* A grip that is set takes this load.  The pull is grip_um, and nothing follows it.  The park keeps
 * the parking distance and drops only the search.  A load that the host starts never finds either
 * one, because its start cleared it first (feed_start.c). */
void ace2k_feed_begin_load(struct ace2k_feed *self, uint8_t lane, uint32_t park_um,
                           uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t grip_um = self->grip_um[lane];
    self->grip_um[lane] = 0;
    if (grip_is_length(grip_um)) {
        park_um = grip_um;
    }
    ace2k_feed_enter(self, lane, ACE2K_FEED_LOADING, seq, now_ms);
    l->sub = ACE2K_FEED_SUB_SETTLE;
    l->length_um = park_um;
    l->speed_um_s = speed_um_s;
    reset_load(l, park_um);
    l->gripped = (bool)(grip_is_length(grip_um) || grip_um == ACE2K_FEED_GRIP_PARK);
    l->grace = true; /* the hand and the pinch catching the filament are not a slip */
    /* The filament comes from the bay with nothing ahead of it.  So its pull and its search take
     * the free rule (feed_snag.c).  The read with motion never comes here.  Its filament may sit in
     * an extruder, and it keeps the watch. */
    l->free_tip = true;
}

/* Every tie-break of the unit goes back now, whatever it does: going, held by a read session, or in
 * a confirmation.  One already on its way back goes on.  It returns true when one ran. */
static bool preempt_tiebreaks(struct ace2k_feed *self, uint32_t now_ms)
{
    bool ran = false;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (ace2k_feed_tiebreak_on(&self->l[i])) {
            ace2k_feed_load_preempt(self, i, now_ms);
            ran = true;
        }
    }
    return ran;
}

/* The read with motion's start, the host's (seq, the grip cleared, the load's speed and the
 * search's reach) or the unit's own tie-break (autonomous: the grip kept, the confirmation's speed
 * and longest leg).  A host's read while a tie-break runs, on any lane, sends every tie-break back
 * and is refused busy: the host tries it again later. */
static enum ace2k_feed_refusal start_search(struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                                            bool autonomous, uint32_t now_ms)
{
    if (self->shut_down) {
        return lane >= ACE2K_LANE_COUNT ? ACE2K_FEED_REFUSED_BOUNDS : ACE2K_FEED_REFUSED_NO_LINK;
    }
    if (!autonomous && lane < ACE2K_LANE_COUNT && preempt_tiebreaks(self, now_ms)) {
        return ACE2K_FEED_REFUSED_BUSY;
    }
    /* The refusals of every start, a load's among them: the search needs filament at the bay. */
    const enum ace2k_feed_refusal verdict =
        ace2k_feed_start_refusal(self, lane, ACE2K_FEED_CMD_LOAD);
    if (verdict != ACE2K_FEED_ACCEPTED) {
        return verdict;
    }
    if (!others_quiet(self, lane, true)) {
        return ACE2K_FEED_REFUSED_OTHER_LANE;
    }
    if ((autonomous ? self->confirm_leg_um : self->search_um) == 0) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    /* free_tip stays false, as the entry left it.  The filament may sit in an extruder, where a
     * filament that follows the motor proves nothing.  So the search keeps the tip snag's watch. */
    ace2k_feed_enter(self, lane, ACE2K_FEED_LOADING, 0, now_ms);
    struct ace2k_feed_lane *l = &self->l[lane];
    l->autonomous = autonomous;
    l->length_um = 0;
    l->speed_um_s = autonomous ? self->confirm_speed_um_s : self->load.speed_um_s;
    reset_load(l, 0);
    if (!autonomous) {
        self->grip_um[lane] = 0; /* as at a host start */
    }
    begin_search(self, lane, now_ms);
    /* The key after the search began: a search with nothing left to run ended it already. */
    if (!autonomous && l->mode == ACE2K_FEED_LOADING) {
        l->search_seq = seq;
        l->search_on = seq != 0;
    }
    return ACE2K_FEED_ACCEPTED;
}

enum ace2k_feed_refusal ace2k_feed_search_start(struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                                                uint32_t now_ms)
{
    return start_search(self, lane, seq, false, now_ms);
}

enum ace2k_feed_refusal ace2k_feed_tiebreak_start(struct ace2k_feed *self, uint8_t lane,
                                                  uint32_t now_ms)
{
    return start_search(self, lane, 0, true, now_ms);
}
