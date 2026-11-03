/* The feed's follow: the cycles of both assists in one arm.
 * - The forward assist's bursts on taut, and the take-ups of the reverse assist on this lane's
 *   full, each ended at rest, with the follow's rule 1 against hunting.
 * - The doses of the feed-forward, metered out of a base rate of the host, ahead of the pull.
 * - The follow's tail, once the insert fell.
 * This was split from feed_assist.c, which arms the follow and runs its step on the tick.  The seam
 * is feed_internal.h.  The tick runs in interrupt context, and the setters run from task context
 * with interrupts masked. */
#include "feed/feed_internal.h"

/* The follow's first rule, no hunting.  A move in the other direction than the last one starts only
 * once the lane has read rest since (follow_dir none), or flip_ms after that move ended.  A move in
 * the same direction starts at once. */
static bool follow_may(const struct ace2k_feed *self, const struct ace2k_feed_lane *l,
                       enum ace2k_lane_dir dir, uint32_t now_ms)
{
    if (l->follow_dir == ACE2K_FEED_FOLLOW_DIR_NONE || l->follow_dir == (uint8_t)dir) {
        return true;
    }
    return ace2k_time_since(now_ms, l->follow_end_ms) >= self->follow.flip_ms;
}

/* A move of the follow, allowed by the follow's rule 1: a forward burst, or a take-up of
 * follow.take_um.  Its direction is kept once the lane took it.  The feed-forward counts a
 * correction when it is applied: the first burst of a taut episode, or the first take-up of a full
 * one.  A taut held under the shared full counts once the burst starts. */
static void follow_start(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                         uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!follow_may(self, l, dir, now_ms)) {
        return;
    }
    if (dir == ACE2K_LANE_FORWARD) {
        ace2k_feed_assist_burst(self, lane, now_ms);
    } else {
        ace2k_feed_assist_counted_move(self, lane, dir, self->follow.take_um, l->speed_um_s,
                                       now_ms);
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

/* The end of the running move, read from the direction it runs.  Here follow_dir is still the
 * move's direction.
 * - Forward: rest, or the shared full, which is the safety stop (the follow's rule 2).
 * - Back: rest or taut.
 * - A dose ends on the shared full, on taut, or at its chunk, never on rest.  It is metered ahead
 *   of the pull, and rest is where it runs.  Taut hands over to the burst on the same tick
 *   (ace2k_feed_follow_step).  A dose also ends once a chunk is owed the other way, and its travel
 *   is then booked as it is (ff_dose_end).
 * This is not an event.  The clock of the follow's rule 1 starts. */
static void follow_moving(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool ends = ace2k_feed_bit(in->pushed, lane);
    if (l->follow_dir == ACE2K_LANE_FORWARD) {
        ends = in->pulled_any;
    }
    bool at_rest = ace2k_feed_bit(in->rest, lane);
    bool against = false;
    if (l->dose) {
        at_rest = false;
        int32_t chunk = (int32_t)self->ff.chunk_um;
        /* A chunk is owed the other way.  The dose runs against the debt, for example a forward
         * dose started in an extrusion with a long retraction since.  It would hold up the dose
         * that pays the debt back. */
        against = l->debt_um >= chunk;
        if (l->follow_dir == ACE2K_LANE_FORWARD) {
            against = l->debt_um <= -chunk;
        }
        if (ace2k_feed_bit(in->pushed, lane) || against) {
            ends = true;
        }
    }
    if (ends || at_rest) {
        l->dose_against = against; /* its travel is booked as it is (ff_dose_end) */
        ace2k_feed_stop_discard(self, lane, now_ms);
        l->sub = ACE2K_FEED_SUB_WAITING;
        l->follow_end_ms = now_ms;
    }
}

/* The cap of the debt either way, in micrometres. */
static int32_t ff_cap_um(const struct ace2k_feed *self)
{
    return (int32_t)(ACE2K_FEED_DEBT_CAP_CHUNKS * self->ff.chunk_um);
}

/* A dose has ended, at its chunk (on_result, in feed.c), or stopped by the follow's step.  The
 * travel of the filament on the encoder pays a debt of the same sign, floored at 0.  Filament that
 * the head moved the other way owes nothing more.  Only a dose cut once a chunk was owed the other
 * way (follow_moving) books its travel against the debt's sign as it is, capped at two chunks.
 * Then the lane owes that much more the other way.  The travel of any other dose against the debt
 * is left unbooked. */
static void ff_dose_end(struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    int32_t travel_um = ace2k_feed_strand_since_um(self, lane, l->dose_enc_um);
    bool same = (int64_t)l->debt_um * travel_um > 0;
    if (same || (l->dose_against && l->debt_um != 0 && travel_um != 0)) {
        int32_t left = l->debt_um - travel_um;
        if (same && (int64_t)l->debt_um * left <= 0) {
            left = 0; /* the same direction past the debt: floored */
        }
        int32_t cap = ff_cap_um(self);
        l->debt_um = ace2k_clamp_i32(left, -cap, cap);
    }
    l->dose = false;
    l->dose_against = false;
}

/* |base_um_s|, INT32_MIN included (ace2k_abs_i32). */
static uint32_t ff_base_mag(const struct ace2k_feed_lane *l)
{
    return ace2k_abs_i32(l->base_um_s);
}

/* The ledger of the feed-forward, on every tick of the follow.  The base rate accrues either way
 * for the time since the last tick.  That time is at most ACE2K_FEED_LEDGER_GAP_MAX_MS, because a
 * tick that caught up on a stall comes once for several periods.  The sub-µm remainder is carried,
 * so a slow rate is not lost to the tick.  The debt is capped at two chunks either way.  Taut and
 * this lane's full each zero it, either sign, because the follow's own move corrects the buffer.
 * The episodes end here.  Their fixes are counted where the correction starts (follow_start). */
static void ff_ledger(const struct ace2k_feed *self, struct ace2k_feed_lane *l, bool taut,
                      bool full, uint32_t now_ms)
{
    uint32_t elapsed_ms = ace2k_time_since(now_ms, l->ledger_ms);
    if (elapsed_ms > ACE2K_FEED_LEDGER_GAP_MAX_MS) {
        elapsed_ms = ACE2K_FEED_LEDGER_GAP_MAX_MS;
    }
    l->ledger_ms = now_ms; /* at a rate of 0 too: that time is never booked later */
    if (l->base_um_s != 0) {
        bool back = l->base_um_s < 0;
        /* µm/s × ms is nanometres.  Dividing by ACE2K_MS_PER_S (1000) gives whole micrometres
         * for the debt.  The nanometres left over are carried to the next tick. */
        uint32_t add_nm = (ff_base_mag(l) * elapsed_ms) + l->debt_rem_nm;
        int32_t um = (int32_t)(add_nm / ACE2K_MS_PER_S);
        l->debt_rem_nm = add_nm % ACE2K_MS_PER_S;
        int32_t cap = ff_cap_um(self);
        l->debt_um = ace2k_clamp_i32(l->debt_um + (back ? -um : um), -cap, cap);
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

/* The speed of a reverse dose: the long retraction's own speed.  It is never under the pulse, and
 * never over the top speed of the lane.  At base 0, what is still owed back goes at the pulse. */
static uint32_t ff_back_speed(const struct ace2k_feed *self, const struct ace2k_feed_lane *l)
{
    uint32_t speed = l->base_um_s < 0 ? ff_base_mag(l) : 0U;
    if (speed < self->ff.pulse_um_s) {
        speed = self->ff.pulse_um_s;
    }
    return speed > ACE2K_LANE_SPEED_MAX_UM_S ? ACE2K_LANE_SPEED_MAX_UM_S : speed;
}

/* A dose.  It needs one chunk owed either way, at any base, 0 included.  It needs no move running,
 * which the caller checks, and the follow's rule 1 must allow that direction.
 * - Forward: the filament is neither taut nor at any full, of this lane or of another, which pauses
 *   the doses.  It is a move of ff.chunk_um at ff.pulse_um_s.  A forward dose lands as any short
 *   move.
 * - Back: the filament is not taut, because then the head holds it, and not in the tail, because
 *   its end would leave the drive's reach.  It is a move of ff.chunk_um at ff_back_speed(), with no
 *   landing.  At the floor speed a dose could not keep up with the retraction that it takes back.
 *   Its stop is no target, because ff_dose_end books the real travel of the filament.
 * The comparator judges both, as it judges a burst. */
static void ff_dose(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                    uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    int32_t chunk = (int32_t)self->ff.chunk_um;
    enum ace2k_lane_dir dir;
    uint32_t speed;
    if (l->debt_um >= chunk) {
        if (in->pulled_any || ace2k_feed_bit(in->pushed, lane)) {
            return;
        }
        dir = ACE2K_LANE_FORWARD;
        speed = self->ff.pulse_um_s;
    } else if (l->debt_um <= -chunk) {
        if (ace2k_feed_bit(in->pushed, lane) || l->tail) {
            return;
        }
        dir = ACE2K_LANE_REVERSE;
        speed = ff_back_speed(self, l);
    } else {
        return;
    }
    if (!follow_may(self, l, dir, now_ms) ||
        !ace2k_feed_assist_move(self, lane, dir, self->ff.chunk_um, speed, now_ms,
                                dir == ACE2K_LANE_FORWARD)) {
        return;
    }
    l->follow_dir = (uint8_t)dir;
    l->dose = true;
    l->dose_enc_um = ace2k_lane_encoder_um(self->lane, lane);
    if (dir == ACE2K_LANE_FORWARD) {
        l->doses++;
    } else {
        l->back_doses++;
    }
}

/* A dose that is no longer running is settled (ff_dose_end).  The follow's step ended it on this
 * tick, or it reached its chunk on this tick's result.  It returns true when the step ended it on
 * taut, and then the burst follows on the same tick. */
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

/* The corrections of the follow, off rest, with no move running.
 * - Taut starts the burst, which is paused under the shared full.
 * - This lane's full starts the take-up.  There is none in the tail, which only feeds, because a
 *   take-up would pull the loose tail back out of the reach of the drive.
 * It returns true when either reading holds the tick. */
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

/* The follow keeps the buffer at rest, whichever way the head moves the filament.
 * - The forward assist's burst on taut.  It is paused by the shared full reading, which ends a
 *   running burst as the safety stop (the follow's rule 2).
 * - The take-up of the reverse assist, of follow.take_um, on this lane's full.
 * Each move ends at rest, and the take-up on taut too.  The move's direction is kept in follow_dir.
 * The comparator, the tail and the chaining at a bound (on_result, in feed.c) read it, and the
 * follow's rule 1 holds a move the other way.  Rest clears it, and that pass through rest allows
 * the flip at once.
 * With a base rate, the debt is metered out in doses ahead of the pull.  This happens at rest too,
 * where the follow itself starts nothing.  Taut and full are the follow's corrections, and they
 * zero the debt.  A base of 0 stops the accrual, and what is owed is still delivered.  With no base
 * ever set, the ledger stays empty, and this is the plain follow. */
void ace2k_feed_follow_step(struct ace2k_feed *self, uint8_t lane,
                            const struct ace2k_feed_inputs *in, uint32_t now_ms)
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
        l->follow_dir =
            ACE2K_FEED_FOLLOW_DIR_NONE; /* read now: either way is open on the next tick */
    }
    ff_ledger(self, l, taut, full,
              now_ms); /* after the settle: the dose's travel booked, then zeroed */
    if (was_moving && !dose_cut) {
        return; /* a move runs, or ended on this tick: none starts now */
    }
    /* at rest, the follow's target: it starts nothing there */
    if (!rest && follow_correct(self, lane, in, taut, full, now_ms)) {
        return;
    }
    ff_dose(self, lane, in, now_ms);
}

/* The tail.  The filament ran out at the insert in the follow, and what is left of it still lies
 * between the insert and the drive.  The follow goes on with its bursts on taut and its doses, none
 * of which reads the insert.  It goes on until the tail leaves the drive, or until follow.tail_um
 * of motor from here.  Leaving the drive is any comparator trip, or a move's deadline once the
 * filament has counted ACE2K_FEED_TAIL_ENC_UM from here and stood still through that move
 * (feed_judge.c).  A take-up that runs now ends at once, and none starts while the tail lasts
 * (follow_correct). */
void ace2k_feed_follow_tail_begin(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->sub == ACE2K_FEED_SUB_MOVING && l->follow_dir == ACE2K_LANE_REVERSE) {
        ace2k_feed_stop_discard(self, lane, now_ms); /* the end of a take-up is not an event */
        l->sub = ACE2K_FEED_SUB_WAITING;
        l->follow_end_ms = now_ms;
    }
    l->tail = true;
    l->tail_fg_start_pulses = ace2k_lane_fg_raw(self->lane, lane);
    l->tail_enc_start_counts = ace2k_lane_encoder_count(self->lane, lane);
    ace2k_feed_notice(self, lane, ACE2K_FEED_TAIL);
}

int ace2k_feed_base_set(struct ace2k_feed *self, uint8_t lane, int32_t rate_um_s, bool clear)
{
    int32_t top = (int32_t)ACE2K_LANE_SPEED_MAX_UM_S;
    if (lane >= ACE2K_LANE_COUNT || rate_um_s > top || rate_um_s < -top) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->mode != ACE2K_FEED_FOLLOWING) {
        return -ACE2K_EREFUSED; /* ignored: the feed-forward acts on a lane in follow only */
    }
    /* 0 stops only the accrual.  The debt is filament that the head has already moved, either way,
     * and it is still paid in doses.  A travel or a retract between two extrusions sends 0.  The
     * clear drops the debt, because the filament was owed to an extruder that this lane no longer
     * feeds alone. */
    if ((rate_um_s < 0) != (l->base_um_s < 0)) {
        l->debt_rem_nm = 0; /* the remainder carried belongs to the other way */
    }
    l->base_um_s = rate_um_s;
    if (clear) {
        l->debt_um = 0;
        l->debt_rem_nm = 0;
    }
    return 0;
}

int ace2k_feed_follow_set(struct ace2k_feed *self, const struct ace2k_feed_follow_cfg *cfg)
{
    if (!ace2k_within_u32(cfg->flip_ms, ACE2K_FEED_FOLLOW_FLIP_MS_MIN,
                          ACE2K_FEED_FOLLOW_FLIP_MS_MAX) ||
        !ace2k_within_u32(cfg->take_um, ACE2K_FEED_FOLLOW_TAKE_UM_MIN,
                          ACE2K_FEED_FOLLOW_TAKE_UM_MAX) ||
        !ace2k_within_u32(cfg->tail_um, ACE2K_FEED_FOLLOW_TAIL_UM_MIN,
                          ACE2K_FEED_FOLLOW_TAIL_UM_MAX)) {
        return -ACE2K_EINVAL;
    }
    self->follow = *cfg;
    return 0;
}

int ace2k_feed_ff_set(struct ace2k_feed *self, const struct ace2k_feed_ff_cfg *cfg)
{
    if (!ace2k_within_u32(cfg->chunk_um, ACE2K_FEED_FF_CHUNK_UM_MIN, ACE2K_FEED_FF_CHUNK_UM_MAX) ||
        !ace2k_within_u32(cfg->pulse_um_s, ACE2K_FEED_FF_PULSE_UM_S_MIN,
                          ACE2K_FEED_FF_PULSE_UM_S_MAX)) {
        return -ACE2K_EINVAL;
    }
    self->ff = *cfg;
    return 0;
}
