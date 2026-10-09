/* The load of the feed: the settle, the pull to the parking point and — with the reader's binding — the pause while a
 * read session holds the lane, the search past the parking point for a tag not seen yet, the
 * return to the parking point, the reacquire of a tag a stop left behind, the read on command
 * with motion, and the grip — an automatic load cut to a short pull with no search.  Every phase
 * is one bounded lane move (the feed's rule 2); a STOP, the link, a shutdown, a runout and
 * the supervisors end it as they end any mode.  Split from
 * feed_assist.c; the seam is feed_internal.h.  The tick runs in interrupt context, the starts
 * from task context with interrupts masked. */
#include "feed/feed_internal.h"

/* The strand's travel since the load began, on the encoder. */
static int32_t travel_um(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_lane_encoder_um(self->lane, lane) - self->l[lane].enc_um_start;
}

static bool others_idle(const struct ace2k_feed *self, uint8_t lane)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (i != lane && ace2k_feed_lane_busy(self, i)) {
            return false;
        }
    }
    return true;
}

/* A move of the load's own, the comparator's bases taken afresh; false when the lane refused
 * it — the caller ends the load. */
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

/* Back to the return point — the parking point, or a read's start — from either side (rule 5
 * of the tag search: a search always ends there, a yield included): a strand past it comes
 * back; one short of it (a reacquire that reversed below it) goes forward to it, never past it,
 * which the tag search's rule 4 allows while another lane moves; within ACE2K_FEED_RETURN_MIN_UM it is there. */
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
    /* the search's reach counts from where the load (or the read with motion) began */
    int32_t left = (int32_t)self->search_um - travel_um(self, lane);
    if (left < (int32_t)ACE2K_FEED_RETURN_MIN_UM) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_LENGTH, now_ms);
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_FORWARD, (uint32_t)left, l->speed_um_s,
                   ACE2K_FEED_SUB_SEARCH, now_ms)) {
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
    return others_idle(self, lane); /* the tag search's rule 4 */
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

/* The search goes on for what is left of it — unless its tag was read meanwhile (during a tip
 * snag's tolerance, which holds the load's tick: back, found), or another lane was given a start
 * while it was paused or reacquiring (the tag search's rule 4): then straight back, never forward past the parking
 * point while another lane moves. */
static void resume_search(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    if (ace2k_feed_bit(in->tag_read, lane)) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_FOUND, now_ms);
        return;
    }
    if (self->l[lane].yield || !others_idle(self, lane)) {
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

/* Why the search ends, in resume_search's order: its tag read on this tick's inputs (during a
 * tolerance, which holds the load's tick); a yield pending (the tag search's rule 4); otherwise the obstacle, not
 * stuck. */
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

static void pause_load(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    ace2k_feed_stop_discard(self, lane, now_ms); /* a pause is not an outcome */
    if (l->sub != ACE2K_FEED_SUB_REACQUIRE) {
        l->resume_sub = l->sub;
    }
    l->sub = ACE2K_FEED_SUB_PAUSED;
}

/* How far a reacquire may reverse: ACE2K_FEED_REACQUIRE_UM, never behind the phase's own start
 * — the load's start during the pull, the return point (the parking point, a read's start)
 * during a search — so it cannot draw the tip back past the insert sensor or out of the drive,
 * nor leave a search's strand short of where it must come back to. */
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

/* Once backwards at the floor speed for a tag a stop left behind; with no room behind the
 * phase's start (under ACE2K_FEED_RETURN_MIN_UM), no reverse: the phase goes on as if the tag
 * had not been lost. */
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

/* The session released the lane: the tag read in the search → back; lost → once backwards;
 * otherwise (read in the pull, given up) the phase goes on; a search asked to yield returns. */
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
    /* this lane's full is the tip snag's (feed_snag.c, after this step on the tick): its
     * tolerance, or the obstacle return where it would be stuck */
    if (l->yield) {
        begin_return(self, lane, ACE2K_FEED_SEARCH_YIELD, now_ms);
    }
}

/* The settle — the hand is still pushing — then the pull to the parking point. */
static void load_settle(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_time_since(now_ms, l->t_mode_ms) < ACE2K_FEED_LOAD_SETTLE_MS) {
        return;
    }
    if (!load_move(self, lane, ACE2K_LANE_FORWARD, l->length_um, l->speed_um_s,
                   ACE2K_FEED_SUB_MOVING, now_ms)) {
        /* not reached: the lane was idle through the settle and the bounds were checked at the
         * start; still, a load that cannot pull ends, it does not hang */
        ace2k_feed_finish(self, lane, ACE2K_FEED_STOPPED, now_ms);
    }
}

void ace2k_feed_load_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    if (ace2k_feed_snag_active(self, lane)) {
        return; /* the tolerance owns the lane's moves; a read session waits for it */
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

/* A grip set takes this load: the pull is grip_um and nothing follows
 * it.  A host's load never finds one — its start cleared it first (feed_start.c). */
void ace2k_feed_begin_load(struct ace2k_feed *self, uint8_t lane, uint32_t park_um,
                           uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t grip_um = self->grip_um[lane];
    self->grip_um[lane] = 0;
    if (grip_um != 0) {
        park_um = grip_um;
    }
    ace2k_feed_enter(self, lane, ACE2K_FEED_LOADING, seq, now_ms);
    l->sub = ACE2K_FEED_SUB_SETTLE;
    l->length_um = park_um;
    l->speed_um_s = speed_um_s;
    reset_load(l, park_um);
    l->gripped = grip_um != 0;
}

int ace2k_feed_grip_set(struct ace2k_feed *self, uint8_t lane, uint32_t grip_um)
{
    if (lane >= ACE2K_LANE_COUNT ||
        (grip_um != 0 && (grip_um < ACE2K_FEED_GRIP_UM_MIN || grip_um > ACE2K_FEED_GRIP_UM_MAX))) {
        return -ACE2K_EINVAL;
    }
    self->grip_um[lane] = grip_um;
    return 0;
}

enum ace2k_feed_refusal ace2k_feed_load_start(struct ace2k_feed *self, uint8_t lane,
                                              uint32_t length_um, uint32_t speed_um_s, uint8_t seq,
                                              uint32_t now_ms)
{
    uint32_t park_um = length_um != 0 ? length_um : self->load.park_um;
    uint32_t pull_um_s = speed_um_s != 0 ? speed_um_s : self->load.speed_um_s;
    if (park_um == 0 || park_um > ACE2K_LANE_MOVE_MAX_UM ||
        !ace2k_lane_speed_in_bounds(pull_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    ace2k_feed_begin_load(self, lane, park_um, pull_um_s, seq, now_ms);
    return ACE2K_FEED_ACCEPTED;
}

void ace2k_feed_search_set(struct ace2k_feed *self, uint32_t search_um)
{
    self->search_um = search_um;
}

enum ace2k_feed_refusal ace2k_feed_search_start(struct ace2k_feed *self, uint8_t lane,
                                                uint32_t now_ms)
{
    if (self->shut_down || lane >= ACE2K_LANE_COUNT || !self->last.link_ok) {
        return lane >= ACE2K_LANE_COUNT ? ACE2K_FEED_REFUSED_BOUNDS : ACE2K_FEED_REFUSED_NO_LINK;
    }
    if (self->l[lane].mode == ACE2K_FEED_ERROR) {
        return ACE2K_FEED_REFUSED_IN_ERROR;
    }
    if (self->l[lane].mode != ACE2K_FEED_IDLE || ace2k_lane_is_moving(self->lane, lane)) {
        return ACE2K_FEED_REFUSED_BUSY;
    }
    if (!ace2k_feed_bit(self->last.insert, lane)) {
        return ACE2K_FEED_REFUSED_NO_FILAMENT;
    }
    if (!others_idle(self, lane)) {
        return ACE2K_FEED_REFUSED_OTHER_LANE;
    }
    if (self->search_um == 0) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    self->grip_um[lane] = 0; /* a host start */
    ace2k_feed_enter(self, lane, ACE2K_FEED_LOADING, 0, now_ms);
    self->l[lane].length_um = 0;
    self->l[lane].speed_um_s = self->load.speed_um_s;
    reset_load(&self->l[lane], 0);
    begin_search(self, lane, now_ms);
    return ACE2K_FEED_ACCEPTED;
}

uint8_t ace2k_feed_search_end(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].search_end : (uint8_t)ACE2K_FEED_SEARCH_NONE;
}
