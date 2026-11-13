/* The confirmation of a candidate tag by the reversals of its lane.  In the feed's load the
 * firmware drives the lane, so it can test whether a tag sits on the lane's spool: it asks the feed
 * to reverse the lane, and the tag must follow.  A tag on the neighbour's spool, at rest or turning
 * slowly one way, does not.  The seam is watch_internal.h.
 * The legs.  The first leg goes toward the spool.  Each time the candidate's presence changes while
 * the lane moves in the asked direction, the asked direction turns round.  From the second change
 * on, each change brings back the presence that the candidate had when the leg before the last one
 * started: that is one reversal followed.  Once cfg.follows reversals are followed and the
 * candidate is in view, it is the lane's.  When it is out of view then, one more leg brings it
 * back.
 * The place.  A tag on the lane's own spool changes at the same lane position both ways, so each
 * change after the first must come within cfg.window_um of the lane's travel at the change before
 * it.  A neighbour's spool lifted off and hung back, or turning with its own lane, changes at a
 * time that has nothing to do with the lane, so mostly somewhere else.  A change outside the window
 * ends the confirmation. */
#include "rfid/watch_internal.h"
#include "lane/lane.h" // ace2k_lane_speed_in_bounds

/* A trace for the bench, dropped and counted when the ring is full. */
static void trace(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t what)
{
    uint8_t next = (uint8_t)((self->trace_head + 1U) % ACE2K_RFID_TRACE_RING);
    if (next == self->trace_tail) {
        if (self->trace_lost < UINT8_MAX) {
            self->trace_lost++;
        }
        return;
    }
    self->trace[self->trace_head] = (struct ace2k_rfid_confirm_trace){
        .lane = lane,
        .what = what,
        .reverse = ace2k_rfid_watch_bit(self->in.reverse, lane),
        .travel_um = self->in.travel_um[lane],
    };
    self->trace_head = next;
}

bool ace2k_rfid_watch_peek_trace(const struct ace2k_rfid_watch *self,
                                 struct ace2k_rfid_confirm_trace *out)
{
    if (self->trace_tail == self->trace_head) {
        return false;
    }
    *out = self->trace[self->trace_tail];
    out->lost = self->trace_lost;
    return true;
}

void ace2k_rfid_watch_drop_trace(struct ace2k_rfid_watch *self)
{
    if (self->trace_tail != self->trace_head) {
        self->trace_tail = (uint8_t)((self->trace_tail + 1U) % ACE2K_RFID_TRACE_RING);
        self->trace_lost = 0; /* told with the trace just sent */
    }
}

/* The lane may run a confirmation for the UID: it can be attributed a tag, its load runs, the host
 * does not have it in use (a confirmation moves the lane back), and the UID is not the
 * neighbour's. */
static bool lane_may_confirm(const struct ace2k_rfid_watch *self, uint8_t lane,
                             const struct ace2k_iso14443a_tag *t)
{
    const struct ace2k_rfid_watch_lane *n = &self->l[ace2k_rfid_neighbour(lane)];
    if (!ace2k_rfid_watch_eligible(self, lane) || !ace2k_rfid_watch_bit(self->in.loading, lane)) {
        return false;
    }
    uint8_t in_use = 0;
    if (ace2k_rfid_watch_in_use(self, &in_use) && ace2k_rfid_watch_bit(in_use, lane)) {
        return false;
    }
    return (bool)!ace2k_rfid_watch_holds(n, t);
}

/* A new confirmation may start on the lane: it may confirm, and its load has not run its cap. */
static bool lane_may_start(const struct ace2k_rfid_watch *self, uint8_t lane,
                           const struct ace2k_iso14443a_tag *t)
{
    return (bool)(self->l[lane].confirms < ACE2K_RFID_CONFIRM_PER_LOAD_MAX &&
                  lane_may_confirm(self, lane, t));
}

/* The reader can watch a confirmation: its field is on, so its tracked UIDs are current, and no
 * session holds it. */
static bool reader_free(const struct ace2k_rfid_watch_reader *rd)
{
    return (bool)(rd->field && !rd->session_open);
}

static void start(struct ace2k_rfid_watch *self, struct ace2k_rfid_watch_reader *rd, uint8_t lane,
                  const struct ace2k_iso14443a_tag *t)
{
    self->l[lane].confirms++;
    self->l[lane].confirm_epoch = (bool)!self->l[lane].confirm_epoch; /* a new confirmation */
    rd->confirm = (struct ace2k_rfid_confirm){
        .on = true,
        .lane = lane,
        .reverse = true,
        .present = ace2k_rfid_watch_in_list(rd->tracked, rd->tracked_count, t),
        .uid = *t,
    };
    trace(self, lane, ACE2K_RFID_TRACE_START);
}

bool ace2k_rfid_watch_confirm_offer(struct ace2k_rfid_watch *self, uint8_t lane,
                                    const struct ace2k_iso14443a_tag *t)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (!lane_may_start(self, lane, t) || !reader_free(rd)) {
        return false;
    }
    if (!rd->confirm.on) {
        start(self, rd, lane, t);
        return true;
    }
    if (rd->confirm.lane != lane && !rd->waiting.on) {
        rd->waiting = (struct ace2k_rfid_confirm){ .on = true, .lane = lane, .uid = *t };
        return true;
    }
    return false;
}

void ace2k_rfid_watch_confirm_inventory(struct ace2k_rfid_watch *self, uint8_t reader,
                                        uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    struct ace2k_rfid_confirm *c = &rd->confirm;
    if (!c->on) {
        return;
    }
    bool present = ace2k_rfid_watch_in_list(rd->tracked, rd->tracked_count, &c->uid);
    if (present == c->present) {
        return;
    }
    uint8_t lane = c->lane;
    int32_t at_um = self->in.travel_um[lane];
    bool asked_way = (bool)(ace2k_rfid_watch_bit(self->in.moving, lane) &&
                            ace2k_rfid_watch_bit(self->in.reverse, lane) == c->reverse);
    if (!asked_way) {
        /* a change that the lane did not make */
        trace(self, lane, ACE2K_RFID_TRACE_WRONG_WAY);
        *c = (struct ace2k_rfid_confirm){ .on = false };
        return;
    }
    if (c->flips > 0 && !ace2k_rfid_watch_apart(at_um, c->flip_at_um, 0, self->cfg.window_um)) {
        /* not where the change before it was */
        trace(self, lane, ACE2K_RFID_TRACE_OUTSIDE);
        *c = (struct ace2k_rfid_confirm){ .on = false };
        return;
    }
    trace(self, lane, ACE2K_RFID_TRACE_FLIP);
    c->flip_at_um = at_um;
    c->present = present;
    c->reverse = (bool)!c->reverse;
    if (c->flips < UINT8_MAX) {
        c->flips++;
    }
    /* The first change ends the first leg, and each later one is a reversal followed. */
    if (present && c->flips > self->cfg.follows) {
        struct ace2k_iso14443a_tag uid = c->uid;
        *c = (struct ace2k_rfid_confirm){ .on = false };
        trace(self, lane, ACE2K_RFID_TRACE_TAKEN);
        ace2k_rfid_watch_take(self, lane, &uid, now_ms);
    }
}

/* The feed ran out a leg of the lane's running confirmation.  An out that the feed shows for the
 * confirmation before (another epoch) is not this one's: the feed has not seen this one yet. */
static bool leg_out(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (!ace2k_rfid_watch_bit(self->in.confirm_out, lane)) {
        return false;
    }
    return ace2k_rfid_watch_bit(self->in.confirm_out_epoch, lane) == self->l[lane].confirm_epoch;
}

void ace2k_rfid_watch_confirm_step(struct ace2k_rfid_watch *self, uint8_t reader)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    struct ace2k_rfid_confirm *c = &rd->confirm;
    if (c->on) {
        if (leg_out(self, c->lane)) {
            trace(self, c->lane, ACE2K_RFID_TRACE_LEG_OUT);
            *c = (struct ace2k_rfid_confirm){ .on = false };
        } else if (!lane_may_confirm(self, c->lane, &c->uid) || !reader_free(rd)) {
            trace(self, c->lane, ACE2K_RFID_TRACE_ENDED);
            *c = (struct ace2k_rfid_confirm){ .on = false };
        }
    }
    struct ace2k_rfid_confirm *w = &rd->waiting;
    if (w->on && !lane_may_start(self, w->lane, &w->uid)) {
        *w = (struct ace2k_rfid_confirm){ .on = false };
    }
    if (w->on && !c->on && reader_free(rd)) {
        start(self, rd, w->lane, &w->uid);
        *w = (struct ace2k_rfid_confirm){ .on = false };
    }
}

uint8_t ace2k_rfid_watch_confirm_bits(const struct ace2k_rfid_watch *self)
{
    uint8_t mask = 0;
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        if (self->r[r].confirm.on) {
            mask = (uint8_t)(mask | (1U << self->r[r].confirm.lane));
        }
    }
    return mask;
}

uint8_t ace2k_rfid_watch_confirm_reverse_bits(const struct ace2k_rfid_watch *self)
{
    uint8_t mask = 0;
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        if (self->r[r].confirm.on && self->r[r].confirm.reverse) {
            mask = (uint8_t)(mask | (1U << self->r[r].confirm.lane));
        }
    }
    return mask;
}

uint8_t ace2k_rfid_watch_confirm_epoch_bits(const struct ace2k_rfid_watch *self)
{
    uint8_t mask = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (self->l[lane].confirm_epoch) {
            mask = (uint8_t)(mask | (1U << lane));
        }
    }
    return mask;
}

int ace2k_rfid_watch_confirm_set(struct ace2k_rfid_watch *self,
                                 const uint32_t v[ACE2K_RFID_CFG_COUNT])
{
    if (!ace2k_within_u32(v[ACE2K_RFID_CFG_LEG], ACE2K_RFID_CFG_LEG_UM_MIN,
                          ACE2K_RFID_CFG_LEG_UM_MAX) ||
        !ace2k_lane_speed_in_bounds(v[ACE2K_RFID_CFG_SPEED]) ||
        !ace2k_within_u32(v[ACE2K_RFID_CFG_PASSIVE_MIN], ACE2K_RFID_CFG_PASSIVE_UM_MIN,
                          ACE2K_RFID_CFG_PASSIVE_UM_MAX) ||
        !ace2k_within_u32(v[ACE2K_RFID_CFG_FOLLOWS], ACE2K_RFID_CFG_FOLLOWS_MIN,
                          ACE2K_RFID_CFG_FOLLOWS_MAX) ||
        !ace2k_within_u32(v[ACE2K_RFID_CFG_WINDOW], ACE2K_RFID_CFG_WINDOW_UM_MIN,
                          ACE2K_RFID_CFG_WINDOW_UM_MAX) ||
        /* above the minimum, so some return can count */
        !ace2k_within_u32(v[ACE2K_RFID_CFG_PASSIVE_MAX], v[ACE2K_RFID_CFG_PASSIVE_MIN] + 1U,
                          ACE2K_RFID_CFG_PASSIVE_MAX_UM_MAX)) {
        return -ACE2K_EINVAL;
    }
    self->cfg = (struct ace2k_rfid_confirm_cfg){
        .leg_max_um = (int32_t)v[ACE2K_RFID_CFG_LEG],
        .speed_um_s = (int32_t)v[ACE2K_RFID_CFG_SPEED],
        .passive_min_um = (int32_t)v[ACE2K_RFID_CFG_PASSIVE_MIN],
        .follows = (uint8_t)v[ACE2K_RFID_CFG_FOLLOWS],
        .window_um = (int32_t)v[ACE2K_RFID_CFG_WINDOW],
        .passive_max_um = (int32_t)v[ACE2K_RFID_CFG_PASSIVE_MAX],
    };
    return 0;
}
