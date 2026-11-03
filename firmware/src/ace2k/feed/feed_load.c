/* The feed's load.  It holds these phases.
 * - The settle.
 * - The pull to the parking point.
 * - With the reader's binding, the pause while a read session holds the lane.
 * - The search past the parking point, for a tag not seen yet.
 * - The return to the parking point.
 * - The reacquire of a tag that a stop left behind.
 * - The read on command with motion.
 * - The grip: an automatic load cut to a short pull, with no search.
 * Every phase is one bounded lane move (the feed's rule 2).  A STOP, the link, a shutdown, a runout
 * and the supervisors end it, as they end any mode.  This was split from feed_assist.c.  The seam
 * is feed_internal.h.  The tick runs in interrupt context, and the starts run from task context
 * with interrupts masked. */
#include "feed/feed_internal.h"

/* The travel of the filament since the load began, on the encoder. */
static int32_t travel_um(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_feed_strand_since_um(self, lane, self->l[lane].enc_um_start);
}

/* The tag search's rule 4, in one place: the search moves only while every other lane is quiet.
 * Idle and error are quiet.  So is a follow with its motor at rest, when follow_at_rest_quiet:
 * what the search needs is its own motor and the reader, and a follow at rest holds neither.  A
 * follow that moves later makes the search yield (ace2k_feed_tick).  The read with motion and a
 * resumed search take a follow at rest as quiet.  A load's search, after its pull, does not: it
 * starts only with every other lane idle. */
static bool others_quiet(const struct ace2k_feed *self, uint8_t lane, bool follow_at_rest_quiet)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (i == lane || !ace2k_feed_lane_busy(self, i)) {
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
 * the lane refused it, and then the caller ends the load. */
static bool load_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir, uint32_t um,
                      uint32_t speed_um_s, uint8_t sub, uint32_t now_ms)
{
    if (um > ACE2K_LANE_MOVE_MAX_UM) {
        um = ACE2K_LANE_MOVE_MAX_UM;
    }
    if (ace2k_lane_move(self->lane, lane, dir, um, speed_um_s, now_ms) != 0) {
        return false;
    }
    self->l[lane].sub = sub;
    ace2k_feed_compare_restart(self, lane);
    return true;
}

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

static void begin_search(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    l->searched = true;
    /* The search's reach counts from where the load, or the read with motion, began. */
    int32_t left = (int32_t)self->search_um - travel_um(self, lane);
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

static bool should_search(const struct ace2k_feed *self, uint8_t lane,
                          const struct ace2k_feed_inputs *in)
{
    if (self->l[lane].gripped || self->search_um <= self->l[lane].return_um ||
        ace2k_feed_bit(in->tag_read, lane)) {
        return false;
    }
    return others_quiet(self, lane, false); /* a load's search: every other lane idle */
}

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

/* The pull, or the search, goes on for what is left of it. */
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

void ace2k_feed_load_resume_after_snag(struct ace2k_feed *self, uint8_t lane,
                                       const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    resume(self, lane, in, self->l[lane].sub, now_ms); /* the snag never changes sub */
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
    if (l->sub != ACE2K_FEED_SUB_REACQUIRE) {
        l->resume_sub = l->sub;
    }
    l->sub = ACE2K_FEED_SUB_PAUSED;
}

/* How far a reacquire may reverse: ACE2K_FEED_REACQUIRE_UM, and never behind the phase's own start.
 * That start is the start of the load during the pull, and the return point during a search.  The
 * return point is the parking point, or the start of a read.  So a reacquire cannot draw the tip
 * back past the insert sensor or out of the drive.  It cannot leave the filament of a search short
 * of where it must come back to. */
static uint32_t reacquire_um(const struct ace2k_feed *self, uint8_t lane)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    int32_t floor_um = l->resume_sub == ACE2K_FEED_SUB_SEARCH ? (int32_t)l->return_um : 0;
    int32_t room = travel_um(self, lane) - floor_um;
    if (room <= 0) {
        return 0;
    }
    return (uint32_t)room < ACE2K_FEED_REACQUIRE_UM ? (uint32_t)room : ACE2K_FEED_REACQUIRE_UM;
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

static void on_moving(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                      uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_feed_bit(in->tag_hold, lane)) {
        pause_load(self, lane, now_ms);
        return;
    }
    if (l->sub != ACE2K_FEED_SUB_SEARCH) {
        return;
    }
    /* This lane's full belongs to the tip snag, in feed_snag.c, after this step on the tick.  For a
     * load from the bay, that is the free phase.  A lag gives the obstacle return, and a yield or a
     * pause runs here through the phase.  For the read with motion, it is the watch, which holds
     * this tick until it ends. */
    if (l->yield) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
    }
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

void ace2k_feed_load_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
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
        if (!should_search(self, lane, in)) {
            l->search_end = ACE2K_FEED_SEARCH_NONE;
            return false;
        }
        begin_search(self, lane, now_ms);
        return true;
    case ACE2K_FEED_SUB_SEARCH:
        begin_return(self, lane, ACE2K_FEED_SEARCH_LENGTH, now_ms);
        return true;
    case ACE2K_FEED_SUB_REACQUIRE:
        resume(self, lane, in, l->resume_sub, now_ms); /* not found back */
        return true;
    default:
        return false; /* the return reached the parking point */
    }
}

void ace2k_feed_yield_others(struct ace2k_feed *self, uint8_t lane)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (i != lane && self->l[i].mode == ACE2K_FEED_LOADING && self->l[i].searched) {
            self->l[i].yield = true;
        }
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

int ace2k_feed_grip_set(struct ace2k_feed *self, uint8_t lane, uint32_t grip_um)
{
    if (lane >= ACE2K_LANE_COUNT ||
        (grip_um != 0 &&
         !ace2k_within_u32(grip_um, ACE2K_FEED_GRIP_UM_MIN, ACE2K_FEED_GRIP_UM_MAX))) {
        return -ACE2K_EINVAL;
    }
    self->grip_um[lane] = grip_um;
    return 0;
}

/* Puts one of the grip slot's reserved values on a lane: the park or the wait. */
static int slot_set(struct ace2k_feed *self, uint8_t lane, uint32_t value)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return -ACE2K_EINVAL;
    }
    self->grip_um[lane] = value;
    return 0;
}

int ace2k_feed_park_set(struct ace2k_feed *self, uint8_t lane)
{
    return slot_set(self, lane, ACE2K_FEED_GRIP_PARK);
}

int ace2k_feed_wait_set(struct ace2k_feed *self, uint8_t lane)
{
    return slot_set(self, lane, ACE2K_FEED_GRIP_WAIT);
}

/* The bounds of a load, for the command and for the settings alike: a parking distance that the
 * lane can move in one move, and a pull speed within the lane's bounds. */
static bool load_in_bounds(uint32_t park_um, uint32_t speed_um_s)
{
    return (bool)(park_um != 0 && park_um <= ACE2K_LANE_MOVE_MAX_UM &&
                  ace2k_lane_speed_in_bounds(speed_um_s));
}

enum ace2k_feed_refusal ace2k_feed_load_start(struct ace2k_feed *self, uint8_t lane,
                                              uint32_t length_um, uint32_t speed_um_s, uint8_t seq,
                                              uint32_t now_ms)
{
    uint32_t park_um = length_um != 0 ? length_um : self->load.park_um;
    uint32_t pull_um_s = speed_um_s != 0 ? speed_um_s : self->load.speed_um_s;
    if (!load_in_bounds(park_um, pull_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    ace2k_feed_begin_load(self, lane, park_um, pull_um_s, seq, now_ms);
    return ACE2K_FEED_ACCEPTED;
}

int ace2k_feed_load_set(struct ace2k_feed *self, const struct ace2k_feed_load_cfg *cfg)
{
    if (!load_in_bounds(cfg->park_um, cfg->speed_um_s)) {
        return -ACE2K_EINVAL;
    }
    self->load = *cfg;
    return 0;
}

int ace2k_feed_search_set(struct ace2k_feed *self, uint32_t search_um)
{
    if (search_um > ACE2K_FEED_SEARCH_MAX_UM) {
        return -ACE2K_EINVAL;
    }
    self->search_um = search_um;
    return 0;
}

bool ace2k_feed_search_retry(const struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                             uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || seq == 0 || seq != self->l[lane].search_seq) {
        return false;
    }
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (l->search_on) {
        return true; /* the search that read started still runs */
    }
    return ace2k_time_since(now_ms, l->search_end_ms) <= ACE2K_FEED_RETRY_WINDOW_MS;
}

enum ace2k_feed_refusal ace2k_feed_search_start(struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                                                uint32_t now_ms)
{
    if (self->shut_down) {
        return lane >= ACE2K_LANE_COUNT ? ACE2K_FEED_REFUSED_BOUNDS : ACE2K_FEED_REFUSED_NO_LINK;
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
    if (self->search_um == 0) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    self->grip_um[lane] = 0; /* a host start */
    /* free_tip stays false, as the entry left it.  The filament may sit in an extruder, where a
     * filament that follows the motor proves nothing.  So the search keeps the tip snag's watch. */
    ace2k_feed_enter(self, lane, ACE2K_FEED_LOADING, 0, now_ms);
    self->l[lane].length_um = 0;
    self->l[lane].speed_um_s = self->load.speed_um_s;
    reset_load(&self->l[lane], 0);
    begin_search(self, lane, now_ms);
    /* The key after the search began: a search with nothing left to run ended it already. */
    if (self->l[lane].mode == ACE2K_FEED_LOADING) {
        self->l[lane].search_seq = seq;
        self->l[lane].search_on = seq != 0;
    }
    return ACE2K_FEED_ACCEPTED;
}

uint8_t ace2k_feed_search_end(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].search_end : (uint8_t)ACE2K_FEED_SEARCH_NONE;
}
