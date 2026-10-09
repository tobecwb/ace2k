/* The autonomous behaviours of the feed (both assists' cycles as decided
 * at the bench): the forward assist — bounded bursts at the armed speed while the
 * strand is taut, stopped when the buffer is back at rest or reads full, with its taut episode's
 * notice —, the reverse assist's take-ups while the buffer reads full from the switches, the
 * follow — both cycles in one arm, with the feed-forward's doses
 * metered out of a host's base rate —, and the load's automatic trigger
 * on the insert's rising edge — the load itself in feed_load.c.  Split from feed.c, which keeps
 * the modes, the tick and the events; the seam is feed_internal.h.  The tick runs in interrupt
 * context, the starts from task context with interrupts masked. */
#include "feed/feed_internal.h"

/* A move of the assist's own: bounded (rule 2), and the comparator's bases taken afresh, so a
 * strand the head pulled ahead during the last one is not held against this one.  False when the
 * lane refused it — still moving: not reached from a waiting lane; the next tick asks. */
static bool start_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                       uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    if (ace2k_lane_move(self->lane, lane, dir, length_um, speed_um_s, now_ms) != 0) {
        return false;
    }
    self->l[lane].sub = ACE2K_FEED_SUB_MOVING;
    ace2k_feed_compare_restart(self, lane);
    return true;
}

/* A burst or a take-up: a move of the assist's own, counted in bursts (a dose of the
 * feed-forward is counted apart, in doses). */
static void start_assist_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                              uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    if (start_move(self, lane, dir, length_um, speed_um_s, now_ms)) {
        self->l[lane].bursts++;
    }
}

/* One forward burst: at most ACE2K_FEED_ASSIST_BURST_UM at the armed speed. */
static void start_burst(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    start_assist_move(self, lane, ACE2K_LANE_FORWARD, ACE2K_FEED_ASSIST_BURST_UM,
                      self->l[lane].speed_um_s, now_ms);
}

/* The forward assist keeps the buffer at rest (decided at the bench: a
 * cycle that ran until the buffer read full kept the strand compressed in the tube, and every
 * burst the head did not fill ran to its bound).  The plunger's three readings: pushed — the
 * strand taut, the head has pulled — starts a burst; rest — the spring centred — ends it; the
 * shared full reading ends it too, as a safety (one input for the four lanes: another lane's
 * plunger at its pulled end pauses this one, and this lane's taut restarts it only once that
 * reading is gone).  Nothing else starts a burst: not rest, not a timer.  A burst that reaches
 * its bound still taut chains into the next on that tick (on_result, in feed.c). */
static void assist_forward(struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool taut = ace2k_feed_bit(in->pushed, lane);
    if (l->sub == ACE2K_FEED_SUB_MOVING) {
        if (in->pulled_any || ace2k_feed_bit(in->rest, lane)) {
            ace2k_feed_stop_discard(self, lane, now_ms); /* a burst's end is not an event */
            l->sub = ACE2K_FEED_SUB_WAITING;
        }
        return;
    }
    if (taut && !in->pulled_any) {
        start_burst(self, lane, now_ms);
    }
}

/* The taut episode of a forward assist: the behind notice is a burst's bound reached with the
 * strand still taut (on_result, in feed.c), once per episode; the episode's end — the plunger off
 * its pushed end — re-arms it.  A strand taut and still under command is no concern of the
 * episode: the comparator's standstill ends it as tangled, stall_check_um of motor after the
 * strand stopped (decided at the bench). */
static void assist_taut(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in)
{
    if (!ace2k_feed_bit(in->pushed, lane)) {
        self->l[lane].behind_sent = false;
    }
}

/* The reverse assist keeps the buffer at rest too (decided at the bench: a creep at
 * rest, meant to keep the strand taut against the head, ran on with nothing holding the strand
 * — 56 creeps, 267 mm taken up — and would have unloaded the lane).  This lane's full — the head
 * pushed strand back — starts a take-up of ACE2K_FEED_ASSIST_BACK_FULL_UM at the armed speed,
 * chained while the plunger still reads full when a move ends (on_result, in feed.c); rest ends
 * it; taut ends it at once too, as before — taut toward the spool is the head holding the strand
 * (the normal end of an unload from the nozzle), and a drive that goes on grinds it.  At rest,
 * nothing; at taut, nothing.  Runs until the host stops it. */
static void assist_back(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->sub == ACE2K_FEED_SUB_MOVING) {
        if (ace2k_feed_bit(in->pushed, lane) || ace2k_feed_bit(in->rest, lane)) {
            ace2k_feed_stop_discard(self, lane, now_ms); /* a take-up's end is not an event */
            l->sub = ACE2K_FEED_SUB_WAITING;
        }
        return;
    }
    if (ace2k_feed_lane_full(in, lane)) {
        start_assist_move(self, lane, ACE2K_LANE_REVERSE, ACE2K_FEED_ASSIST_BACK_FULL_UM,
                          l->speed_um_s, now_ms);
    }
}

/* The follow's first rule, no hunting: a move the other way than the last only once
 * the lane has read rest since (follow_dir none), or flip_ms after that move ended; the same
 * direction at once. */
static bool follow_may(const struct ace2k_feed *self, const struct ace2k_feed_lane *l,
                       enum ace2k_lane_dir dir, uint32_t now_ms)
{
    if (l->follow_dir == ACE2K_FOLLOW_DIR_NONE || l->follow_dir == (uint8_t)dir) {
        return true;
    }
    return ace2k_time_since(now_ms, l->follow_end_ms) >= self->follow.flip_ms;
}

/* A move of the follow, allowed by the follow's rule 1: a forward burst or a take-up of follow.take_um, its
 * direction kept once the lane took it.  The feed-forward counts a correction when it is
 * applied: the first burst of a taut episode, the first take-up of a full one (a taut held
 * under the shared full counts once the burst starts). */
static void follow_start(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                         uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!follow_may(self, l, dir, now_ms)) {
        return;
    }
    if (dir == ACE2K_LANE_FORWARD) {
        start_burst(self, lane, now_ms);
    } else {
        start_assist_move(self, lane, dir, self->follow.take_um, l->speed_um_s, now_ms);
    }
    if (l->sub != ACE2K_FEED_SUB_MOVING) {
        return;
    }
    l->follow_dir = (uint8_t)dir;
    if (dir == ACE2K_LANE_FORWARD) {
        if (!l->taut_seen) {
            l->taut_fixes++;
            l->taut_seen = true;
        }
    } else if (!l->full_seen) {
        l->full_fixes++;
        l->full_seen = true;
    }
}

/* The running move's end, read from the direction it runs (follow_dir, still the move's here):
 * forward, rest or the shared full — the safety stop (the follow's rule 2); back, rest or taut.  A dose ends
 * on the shared full or on taut (or its chunk), never on rest: it is metered ahead of the pull,
 * and rest is where it runs; taut hands over to the burst on
 * the same tick (assist_follow).  Not an event; the follow's rule 1 clock starts. */
static void follow_moving(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool ends = ace2k_feed_bit(in->pushed, lane);
    if (l->follow_dir == ACE2K_LANE_FORWARD) {
        ends = in->pulled_any;
    }
    bool at_rest = ace2k_feed_bit(in->rest, lane);
    if (l->dose) {
        at_rest = false;
        if (ace2k_feed_bit(in->pushed, lane)) {
            ends = true;
        }
    }
    if (ends || at_rest) {
        ace2k_feed_stop_discard(self, lane, now_ms);
        l->sub = ACE2K_FEED_SUB_WAITING;
        l->follow_end_ms = now_ms;
    }
}

/* A dose has ended — at its chunk (on_result, in feed.c), or stopped by the follow's step: what
 * the strand travelled on the encoder pays the debt, floored at 0 (a strand the head pulled back
 * owes nothing more). */
static void ff_dose_end(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    int32_t travel_um = ace2k_lane_encoder_um(self->lane, lane) - l->dose_enc_um;
    if (travel_um > 0) {
        uint32_t paid_um = (uint32_t)travel_um;
        l->debt_um = paid_um >= l->debt_um ? 0U : l->debt_um - paid_um;
    }
    l->dose = false;
}

/* The feed-forward's ledger on every tick of the follow: the
 * base rate accrues — the sub-µm remainder carried, so a slow rate is not lost to the tick —
 * capped at two chunks; taut and this lane's full each zero it, the follow's own
 * move correcting the buffer.  The episodes end here; their fixes are counted where the
 * correction starts (follow_start). */
static void ff_ledger(struct ace2k_feed *self, struct ace2k_feed_lane *l, bool taut, bool full)
{
    if (l->base_um_s > 0) {
        uint32_t add = (l->base_um_s * ACE2K_FEED_TICK_MS) + l->debt_rem; /* µm × 1000 */
        l->debt_um += add / 1000U;
        l->debt_rem = add % 1000U;
        uint32_t cap = 2U * self->ff.chunk_um;
        if (l->debt_um > cap) {
            l->debt_um = cap;
        }
    }
    if (taut) {
        l->debt_um = 0;
    } else {
        l->taut_seen = false;
    }
    if (full) {
        l->debt_um = 0;
    } else {
        l->full_seen = false;
    }
}

/* A dose: one chunk owed — at any base, 0 included — no move running (the caller's), the strand
 * neither taut nor at any full — this lane's or another's, which pauses the doses — and the follow's rule 1
 * allowing forward: a move of ff.chunk_um at ff.pulse_um_s, judged by the comparator as a burst. */
static void ff_dose(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                    uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->debt_um < self->ff.chunk_um || in->pulled_any || ace2k_feed_bit(in->pushed, lane) ||
        !follow_may(self, l, ACE2K_LANE_FORWARD, now_ms)) {
        return;
    }
    if (!start_move(self, lane, ACE2K_LANE_FORWARD, self->ff.chunk_um, self->ff.pulse_um_s,
                    now_ms)) {
        return;
    }
    l->follow_dir = (uint8_t)ACE2K_LANE_FORWARD;
    l->dose = true;
    l->dose_enc_um = ace2k_lane_encoder_um(self->lane, lane);
    l->doses++;
}

/* A dose that is no longer running is settled (ff_dose_end): ended by the follow's step on
 * this tick, or at its chunk on this tick's result.  True when the step ended it on taut — the
 * burst follows on the same tick. */
static bool ff_dose_settle(struct ace2k_feed *self, uint8_t lane, bool was_moving, bool taut)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!l->dose || l->sub == ACE2K_FEED_SUB_MOVING) {
        return false;
    }
    ff_dose_end(self, lane);
    if (!was_moving) {
        return false;
    }
    return taut;
}

/* The follow's corrections, off rest with no move running: taut, the burst — paused under the
 * shared full; this lane's full, the take-up — none in the tail, which only feeds
 * (a take-up would pull the loose tail back out of the drive's
 * reach).  True when either reading holds the tick. */
static bool follow_correct(struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in, bool taut, bool full,
                           uint32_t now_ms)
{
    if (taut) {
        if (!in->pulled_any) {
            follow_start(self, lane, ACE2K_LANE_FORWARD, now_ms);
        }
        return true; /* taut under the shared full: paused */
    }
    if (full) {
        if (!self->l[lane].tail) {
            follow_start(self, lane, ACE2K_LANE_REVERSE, now_ms);
        }
        return true;
    }
    return false;
}

/* The follow keeps the buffer at rest whichever way the head moves the strand:
 * the forward assist's burst on taut — paused by the shared full
 * reading, which ends a running one as the safety stop (the follow's rule 2) — and the reverse assist's
 * take-up of follow.take_um on this lane's full, each ended at rest, the take-up on taut too.
 * The move's direction is kept in follow_dir: the comparator, the tail and the chaining at a
 * bound (on_result, in feed.c) read it, and the follow's rule 1 holds a move the other way.  Rest clears
 * it — the pass through rest that allows the flip at once.
 * With a base rate the debt is metered out in doses ahead of
 * the pull: at rest too, where the follow itself starts nothing; taut and full are the follow's
 * corrections, and zero the debt.  A base of 0 stops the accrual, and what is owed is still
 * delivered; with none ever set, the ledger stays empty and this is the follow. */
static void assist_follow(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool rest = ace2k_feed_bit(in->rest, lane);
    bool taut = ace2k_feed_bit(in->pushed, lane);
    bool full = ace2k_feed_lane_full(in, lane);
    bool was_moving = l->sub == ACE2K_FEED_SUB_MOVING;
    if (was_moving) {
        follow_moving(self, lane, in, now_ms); /* reads the running move's direction */
    }
    bool dose_cut = ff_dose_settle(self, lane, was_moving, taut);
    if (rest && l->sub != ACE2K_FEED_SUB_MOVING) {
        l->follow_dir = ACE2K_FOLLOW_DIR_NONE; /* read now: either way is open on the next tick */
    }
    ff_ledger(self, l, taut, full); /* after the settle: the dose's travel booked, then zeroed */
    if (was_moving && !dose_cut) {
        return; /* a move running, or ended on this tick: none starts now */
    }
    /* at rest the follow's target: it starts nothing there */
    if (!rest && follow_correct(self, lane, in, taut, full, now_ms)) {
        return;
    }
    ff_dose(self, lane, in, now_ms);
}

/* The tail: the strand ran out at the insert in the follow, and what is
 * left of it still lies between the insert and the drive.  The follow goes on — its bursts on
 * taut and the doses, none of which reads the insert — until the tail leaves the drive (any
 * comparator trip: feed_judge.c) or follow.tail_um of motor from here; a take-up running
 * now ends at once, and none starts while the tail lasts (follow_correct). */
void ace2k_feed_follow_tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->sub == ACE2K_FEED_SUB_MOVING && l->follow_dir == ACE2K_LANE_REVERSE) {
        ace2k_feed_stop_discard(self, lane, now_ms); /* a take-up's end is not an event */
        l->sub = ACE2K_FEED_SUB_WAITING;
        l->follow_end_ms = now_ms;
    }
    l->tail = true;
    l->tail_fg_start = self->lane->lane[lane].fg_raw;
    ace2k_feed_notice(self, lane, ACE2K_FEED_TAIL);
}

enum ace2k_feed_refusal ace2k_feed_assist_start(struct ace2k_feed *self, uint8_t lane,
                                                enum ace2k_feed_cmd cmd, uint32_t speed_um_s,
                                                uint8_t seq, uint32_t now_ms)
{
    if (!ace2k_lane_speed_in_bounds(speed_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    uint8_t mode = ACE2K_FEED_ASSISTING;
    if (cmd == ACE2K_FEED_CMD_ASSIST_BACK) {
        mode = ACE2K_FEED_ASSISTING_BACK;
    } else if (cmd == ACE2K_FEED_CMD_ASSIST_BOTH) {
        mode = ACE2K_FEED_FOLLOWING;
    }
    ace2k_feed_enter(self, lane, mode, seq, now_ms); /* the follow's direction none, and rest */
    l->follow_end_ms = now_ms;
    l->speed_um_s = speed_um_s;
    l->length_um = 0;
    l->sub = ACE2K_FEED_SUB_WAITING;
    /* the first move at once when the last tick's readings ask for one: the tick's own step, run
     * on that snapshot, so arming and the tick read the buffer the same way — forward, the strand
     * taut and the buffer not full; reverse, this lane's buffer full; the follow, either;
     * otherwise the lane waits for the head's first pull or push */
    if (mode == ACE2K_FEED_ASSISTING_BACK) {
        assist_back(self, lane, &self->last, now_ms);
    } else if (mode == ACE2K_FEED_FOLLOWING) {
        assist_follow(self, lane, &self->last, now_ms);
    } else {
        assist_forward(self, lane, &self->last, now_ms);
    }
    return ACE2K_FEED_ACCEPTED;
}

int ace2k_feed_base_set(struct ace2k_feed *self, uint8_t lane, uint32_t rate_um_s, bool clear)
{
    if (lane >= ACE2K_LANE_COUNT || rate_um_s > ACE2K_LANE_SPEED_MAX_UM_S) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode != ACE2K_FEED_FOLLOWING) {
        return -ACE2K_EREFUSED; /* ignored: the feed-forward acts on a lane in follow only */
    }
    /* 0 stops the accrual only: the debt is strand the head has already pulled, still paid in
     * doses — a travel or a retract between two extrusions sends 0.  The clear drops it: the
     * strand was owed to an extruder this lane no longer feeds alone. */
    l->base_um_s = rate_um_s;
    if (clear) {
        l->debt_um = 0;
        l->debt_rem = 0;
    }
    return 0;
}

void ace2k_feed_auto_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          bool rose, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!in->link_ok) {
        return; /* nothing starts without a host (rule 6); the tick has ended what was armed */
    }
    /* the automatic load: a strand put in at the mouth of a lane idle through this tick, seq 0 —
     * a mode of the unit's own, which leaves the last host start's retry key as it was */
    if (rose && l->mode == ACE2K_FEED_IDLE && self->load.auto_load) {
        ace2k_feed_begin_load(self, lane, self->load.park_um, self->load.speed_um_s, 0, now_ms);
        ace2k_feed_yield_others(self, lane); /* another lane moves: the tag search yields */
    }
    switch (l->mode) {
    case ACE2K_FEED_LOADING:
        ace2k_feed_load_tick(self, lane, in, now_ms);
        break;
    case ACE2K_FEED_ASSISTING:
        assist_forward(self, lane, in, now_ms);
        assist_taut(self, lane, in);
        break;
    case ACE2K_FEED_ASSISTING_BACK:
        assist_back(self, lane, in, now_ms);
        break;
    case ACE2K_FEED_FOLLOWING:
        assist_follow(self, lane, in, now_ms);
        assist_taut(self, lane, in);
        break;
    default:
        break;
    }
}

int ace2k_feed_follow_set(struct ace2k_feed *self, const struct ace2k_feed_follow_cfg *cfg)
{
    if (cfg->flip_ms < ACE2K_FEED_FOLLOW_FLIP_MS_MIN ||
        cfg->flip_ms > ACE2K_FEED_FOLLOW_FLIP_MS_MAX ||
        cfg->take_um < ACE2K_FEED_FOLLOW_TAKE_UM_MIN ||
        cfg->take_um > ACE2K_FEED_FOLLOW_TAKE_UM_MAX ||
        cfg->tail_um < ACE2K_FEED_FOLLOW_TAIL_UM_MIN ||
        cfg->tail_um > ACE2K_FEED_FOLLOW_TAIL_UM_MAX) {
        return -ACE2K_EINVAL;
    }
    self->follow = *cfg;
    return 0;
}

int ace2k_feed_ff_set(struct ace2k_feed *self, const struct ace2k_feed_ff_cfg *cfg)
{
    if (cfg->chunk_um < ACE2K_FEED_FF_CHUNK_UM_MIN || cfg->chunk_um > ACE2K_FEED_FF_CHUNK_UM_MAX ||
        cfg->pulse_um_s < ACE2K_FEED_FF_PULSE_UM_S_MIN ||
        cfg->pulse_um_s > ACE2K_FEED_FF_PULSE_UM_S_MAX) {
        return -ACE2K_EINVAL;
    }
    self->ff = *cfg;
    return 0;
}
