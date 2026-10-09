#include "heat/heat.h"
#include "heat/mains.h"

/* The limits that latch at once.  Cutout first: of every limit it is the one that says the
 * hardware already overheated.  An over-temperature is read on a valid NTC only. */
static uint8_t hard_limits_reason(const struct ace2k_heat_inputs *in)
{
    if (in->cutout) {
        return ACE2K_HEAT_CUTOUT;
    }
    if ((in->left_valid && in->ntc_left_mc >= ACE2K_HEAT_NTC_MAX_MC) ||
        (in->right_valid && in->ntc_right_mc >= ACE2K_HEAT_NTC_MAX_MC)) {
        return ACE2K_HEAT_NTC_OVER;
    }
    if (!in->mains_present) {
        return ACE2K_HEAT_MAINS_ABSENT;
    }
    return ACE2K_HEAT_OK;
}

/* Either outlet NTC reads invalid. */
static bool ntc_invalid(const struct ace2k_heat_inputs *in)
{
    if (!in->left_valid) {
        return true;
    }
    if (!in->right_valid) {
        return true;
    }
    return false;
}

/* Every limit: the immediate ones first, then the two the tick debounces — an invalid NTC, the
 * mains implausible. */
static uint8_t limits_reason(const struct ace2k_heat_inputs *in)
{
    uint8_t hard = hard_limits_reason(in);
    if (hard != ACE2K_HEAT_OK) {
        return hard;
    }
    if (ntc_invalid(in)) {
        return ACE2K_HEAT_NTC_INVALID; /* ahead of the mains: a dip never masks a sensor */
    }
    if (!ace2k_mains_hz10_plausible(in->mains_hz10)) {
        return ACE2K_HEAT_MAINS_IMPLAUSIBLE;
    }
    return ACE2K_HEAT_OK;
}

/* The limits a lease rides out for a while (the tick's buckets latch them). */
static bool reason_debounced(uint8_t reason)
{
    if (reason == ACE2K_HEAT_NTC_INVALID) {
        return true;
    }
    return reason == ACE2K_HEAT_MAINS_IMPLAUSIBLE;
}

/* The immediate limits, the fans, the duty — then a debounced limit, so a fan or a duty fault is
 * never held back by a debounce. */
static uint8_t verdict_reason(const struct ace2k_heat_inputs *in, uint8_t limits, uint8_t duty)
{
    if (limits != ACE2K_HEAT_OK && !reason_debounced(limits)) {
        return limits;
    }
    if (!in->fans_commanded || (in->fans_read & ACE2K_HEAT_FANS_BOTH) != ACE2K_HEAT_FANS_BOTH) {
        return ACE2K_HEAT_FANS;
    }
    if (duty > ACE2K_HEAT_DUTY_MAX_PCT) {
        return ACE2K_HEAT_DUTY;
    }
    return limits;
}

void ace2k_heat_init(struct ace2k_heat *self, const struct ace2k_heat_ops *ops, void *ctx)
{
    *self = (struct ace2k_heat){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    self->state = ACE2K_HEAT_IDLE;
    self->reason = ACE2K_HEAT_OK;
    /* no inputs seen yet: a lease is refused until the first tick */
    self->limits = ACE2K_HEAT_NTC_INVALID;
    self->verdict = ACE2K_HEAT_NTC_INVALID;
    self->ntc_invalid = true;
    ops->gate_off(ctx);
}

/* No half-cycle due: the next edge starts a fresh cycle.  The interrupt's fields; written by
 * the tick only while fire_mode keeps the interrupt off them (NONE). */
static void cycle_reset(struct ace2k_heat *self)
{
    self->fire_this_cycle = false;
    self->edge_parity = 0U;
}

/* The silencing store: from here on no edge fires anything. */
static void silence(struct ace2k_heat *self)
{
    self->fire_mode = ACE2K_HEAT_FIRE_NONE;
    ACE2K_COMPILER_BARRIER();
}

/* Silenced first, the gate low last. */
static void latch(struct ace2k_heat *self, uint8_t reason)
{
    silence(self);
    if (self->state != ACE2K_HEAT_LATCHED) {
        self->reason = reason;
        ACE2K_COMPILER_BARRIER();
        self->state = ACE2K_HEAT_LATCHED;
    }
    self->ops->gate_off(self->ctx);
}

/* How a stop treats a cycle whose even half has fired:
 * KEEP_HALF — the explicit release: its odd half still fires, on an ok verdict;
 * LEAVE     — the lease's end: the cycle guard left no cycle begun; a pending half stays;
 * DROP      — a dryer fault, a shutdown: no half-cycle after the call. */
enum ace2k_heat_stop {
    ACE2K_HEAT_STOP_KEEP_HALF,
    ACE2K_HEAT_STOP_LEAVE,
    ACE2K_HEAT_STOP_DROP,
};

/* The even half of the cycle in progress fired and its odd edge has not come. */
static bool cycle_half_fired(const struct ace2k_heat *self)
{
    if (self->edge_parity == 0U) {
        return false;
    }
    return self->fire_this_cycle;
}

/* The mode a stop leaves, in one store, before anything else.  Only the tick (or a command with
 * interrupts masked) writes LEASED, and the interrupt only ever turns PENDING into NONE, so a
 * LEASED read here cannot change under the store that follows it. */
static void stop_mode(struct ace2k_heat *self, enum ace2k_heat_stop half)
{
    if (half == ACE2K_HEAT_STOP_DROP) {
        silence(self);
        return;
    }
    uint32_t mode = self->fire_mode;
    if (mode != ACE2K_HEAT_FIRE_LEASED) {
        return; /* NONE stays NONE; a PENDING half stays the interrupt's */
    }
    self->fire_mode =
        half == ACE2K_HEAT_STOP_KEEP_HALF ? ACE2K_HEAT_FIRE_PENDING : ACE2K_HEAT_FIRE_NONE;
    ACE2K_COMPILER_BARRIER();
}

/* Back to IDLE.  The first store is the mode: NONE (DROP, LEAVE) fires nothing from then on;
 * PENDING (KEEP_HALF) lets only the odd half of a cycle whose even half already fired complete
 * — the pending path never starts a half.  Once the mode is stored no edge can begin a cycle, so
 * the cycle state read after it is final: with no even half fired the release drops to NONE.
 * Only DROP cuts the gate: a pulse the interrupt started a moment ago — an odd half, or an even
 * half whose odd half is now pending — is a whole half-cycle only if TIM7 ends it, so a release
 * and a lease's end leave it to TIM7 (6 ms). */
static void stop(struct ace2k_heat *self, enum ace2k_heat_stop half)
{
    stop_mode(self, half);
    if (self->state == ACE2K_HEAT_LEASED) {
        self->state = ACE2K_HEAT_IDLE;
    }
    if (half == ACE2K_HEAT_STOP_KEEP_HALF && !cycle_half_fired(self)) {
        self->fire_mode = ACE2K_HEAT_FIRE_NONE; /* the interrupt writes only NONE too */
    }
    self->duty_pct = 0U;
    if (half == ACE2K_HEAT_STOP_DROP) {
        self->ops->gate_off(self->ctx);
    }
}

void ace2k_heat_release(struct ace2k_heat *self)
{
    stop(self, ACE2K_HEAT_STOP_KEEP_HALF);
}

void ace2k_heat_abort(struct ace2k_heat *self)
{
    stop(self, ACE2K_HEAT_STOP_DROP);
}

void ace2k_heat_shutdown(struct ace2k_heat *self)
{
    if (self->shut) {
        return; /* once: the binding's tick calls it on every tick after a shutdown */
    }
    stop(self, ACE2K_HEAT_STOP_DROP); /* the silencing store first */
    self->shut = true;
    cycle_reset(self);
}

void ace2k_heat_set_inhibit(struct ace2k_heat *self, bool on)
{
    self->inhibit = on;
}

/* The refusals that do not depend on the inputs. */
static int lease_refusal(const struct ace2k_heat *self)
{
    if (self->shut) {
        return -ACE2K_EREFUSED;
    }
    if (self->state == ACE2K_HEAT_LATCHED) {
        return -ACE2K_ELATCHED;
    }
    if (self->cutout_seen) {
        return -ACE2K_EREFUSED;
    }
    return 0;
}

/* An ok verdict — or a live lease riding out an NTC's invalid excursion or an implausible
 * mains, which the tick latches if it lasts. */
static bool verdict_admits(const struct ace2k_heat *self)
{
    if (self->verdict == ACE2K_HEAT_OK) {
        return true;
    }
    if (self->state != ACE2K_HEAT_LEASED) {
        return false;
    }
    return reason_debounced(self->verdict);
}

/* A verdict that does not admit the lease: transient (ask again) for the mains implausible with
 * its bucket not full and for an invalid NTC with no live lease — the dryer's own debounce judges
 * that one (dryer_protect.c); final for anything else. */
static int verdict_refusal(const struct ace2k_heat *self)
{
    if (self->verdict == ACE2K_HEAT_MAINS_IMPLAUSIBLE && !ace2k_heat_mains_implausible_held(self)) {
        return -ACE2K_EAGAIN;
    }
    if (self->verdict == ACE2K_HEAT_NTC_INVALID) {
        return -ACE2K_EAGAIN;
    }
    return -ACE2K_EBUSY;
}

/* A new lease (none live) waits for a mains window measured since the return; a live lease is
 * never refused for it.  Firing needs a measured period anyway: defence in depth.  Early returns:
 * in C `!b` is an int, and the lint refuses one returned as a bool. */
static bool mains_unmeasured_blocks(const struct ace2k_heat *self)
{
    if (self->state == ACE2K_HEAT_LEASED) {
        return false;
    }
    if (self->mains_measured) {
        return false;
    }
    return true;
}

uint8_t ace2k_heat_refusal_reason(const struct ace2k_heat *self)
{
    if (self->state == ACE2K_HEAT_LATCHED) {
        return self->reason;
    }
    if (self->verdict != ACE2K_HEAT_OK) {
        return self->verdict;
    }
    if (!ace2k_heat_mains_recovered(self) || mains_unmeasured_blocks(self)) {
        return ACE2K_HEAT_MAINS_IMPLAUSIBLE;
    }
    return ACE2K_HEAT_OK;
}

int ace2k_heat_lease(struct ace2k_heat *self, uint8_t duty_pct, uint32_t ms, uint32_t now_ms)
{
    if (duty_pct == 0U || ms == 0U) {
        ace2k_heat_release(self);
        return 0;
    }
    if (duty_pct > ACE2K_HEAT_DUTY_MAX_PCT || ms > ACE2K_HEAT_LEASE_MAX_MS) {
        return -ACE2K_EINVAL;
    }
    int rc = lease_refusal(self);
    if (rc != 0) {
        return rc;
    }
    if (!verdict_admits(self)) {
        rc = verdict_refusal(self);
    }
    /* a final verdict first: the flash inhibit is transient and would hide it behind a retry */
    if (rc == -ACE2K_EBUSY) {
        return rc;
    }
    if (self->inhibit) {
        return -ACE2K_EAGAIN;
    }
    if (rc != 0) {
        return rc;
    }
    if (self->state != ACE2K_HEAT_LEASED && !ace2k_heat_mains_recovered(self)) {
        return -ACE2K_EAGAIN; /* a new lease waits for the restart level: it rides out a dip */
    }
    if (mains_unmeasured_blocks(self)) {
        return -ACE2K_EAGAIN; /* the mains is back, its first window not measured yet */
    }
    if (self->state == ACE2K_HEAT_IDLE && self->fire_mode == ACE2K_HEAT_FIRE_NONE) {
        self->acc = 0U;
        self->edge_parity = 0U;
        self->fire_this_cycle = false;
    }
    self->duty_pct = duty_pct;
    self->lease_end_ms = now_ms + ms;
    ACE2K_COMPILER_BARRIER();
    self->state = ACE2K_HEAT_LEASED; /* published last: fire_mode LEASED follows at the next tick */
    return 0;
}

/* The even edge decides for the whole cycle, only if the lease covers it and the odd half is
 * still within ACE2K_HEAT_EDGES_PER_TICK_MAX edges of the last tick. */
static void decide_cycle(struct ace2k_heat *self, uint32_t edges)
{
    uint32_t guard_end = self->now_ms + ACE2K_HEAT_CYCLE_GUARD_MS;
    if (edges >= ACE2K_HEAT_EDGES_PER_TICK_MAX || ace2k_time_after(guard_end, self->lease_end_ms)) {
        self->fire_this_cycle = false;
        return;
    }
    self->acc += self->duty_pct;
    self->fire_this_cycle = self->acc >= ACE2K_HEAT_PERCENT;
    if (self->fire_this_cycle) {
        self->acc -= ACE2K_HEAT_PERCENT;
    }
}

static void fire(struct ace2k_heat *self, uint32_t now_us)
{
    self->ops->gate_fire(self->ctx);
    self->fired++;
    self->last_fire_us = now_us;
    self->fired_pending_check = true;
}

/* One more edge since the last tick, saturating: a dead tick never wraps back under the limit. */
static uint32_t count_edge(struct ace2k_heat *self)
{
    uint32_t edges = self->edges_since_tick;
    if (edges != UINT32_MAX) {
        edges++;
        self->edges_since_tick = edges;
    }
    return edges;
}

/* The odd edge: the cycle ends here whatever happens, so the next edge starts a fresh one.  It
 * fires only in phase — one measured period after the edge before the even one, so a zero-cross
 * missed between the halves (the gap a period and a half) abandons the cycle. */
static void odd_edge(struct ace2k_heat *self, uint32_t now_us, bool in_phase)
{
    bool fire_it = false;
    if (self->fire_this_cycle) {
        fire_it = in_phase;
    }
    cycle_reset(self);
    if (fire_it) {
        fire(self, now_us);
    }
}

/* Released between the two edges of a fired cycle: its odd half — never an even one — then
 * NONE.  A verdict not ok or a latch already turned the mode to NONE. */
static void pending_edge(struct ace2k_heat *self, uint32_t now_us, bool in_phase)
{
    uint32_t edges = count_edge(self);
    if (edges > ACE2K_HEAT_EDGES_PER_TICK_MAX) {
        cycle_reset(self);
    } else {
        odd_edge(self, now_us, in_phase);
    }
    self->fire_mode = ACE2K_HEAT_FIRE_NONE;
}

/* A leased edge: the odd one of a cycle, fired only in phase, or an even one that starts a cycle
 * only when start_ok (ace2k_heat_zerocross). */
static void leased_edge(struct ace2k_heat *self, uint32_t now_us, bool in_phase, bool start_ok)
{
    uint32_t edges = count_edge(self);
    if (edges > ACE2K_HEAT_EDGES_PER_TICK_MAX) {
        /* The tick lags: the cycle in progress is abandoned, so the next edge processed starts
         * a fresh one and never completes the old cycle with a half of the same polarity. */
        cycle_reset(self);
        return;
    }
    if (self->duty_pct > ACE2K_HEAT_DUTY_MAX_PCT) {
        return;
    }
    if (self->edge_parity != 0U) {
        odd_edge(self, now_us, in_phase);
        return;
    }
    if (!start_ok) {
        return; /* at an unknown phase: no cycle starts here, the next edge is tried as even */
    }
    decide_cycle(self, edges);
    self->even_us = now_us;
    self->edge_parity = 1U;
    if (self->fire_this_cycle) {
        fire(self, now_us);
    }
}

/* One measured period after the edge two before it — the same polarity, so an asymmetry between
 * the input's two half-cycles cancels — within ACE2K_HEAT_EDGE_TOL_US.  Never with no plausible
 * frequency (period_us 0), fewer than two edges before it, or on a resync edge
 * (ace2k_heat_resync).  An edge judged out of phase against a measured period is counted
 * (phase_rejects). */
static bool edge_in_phase(struct ace2k_heat *self, uint32_t now_us)
{
    if (self->skip_edge) {
        self->skip_edge = false; /* the resync edge: maybe late, kept out of the history */
        return false;
    }
    uint8_t seen = self->edges_seen;
    uint32_t two_before = self->prev2_edge_us;
    self->prev2_edge_us = self->prev_edge_us;
    self->prev_edge_us = now_us;
    if (seen < 2U) {
        self->edges_seen = (uint8_t)(seen + 1U); /* saturates at 2 */
        return false;
    }
    uint32_t period = self->period_us;
    if (period == 0U) {
        return false;
    }
    uint32_t gap = now_us - two_before;
    uint32_t off = gap > period ? gap - period : period - gap;
    if (off <= ACE2K_HEAT_EDGE_TOL_US) {
        return true;
    }
    self->phase_rejects++;
    return false;
}

void ace2k_heat_resync(struct ace2k_heat *self)
{
    self->edges_seen = 0U; /* the history restarts as at boot: two edges, then judged */
    self->prev_in_phase = false;
    self->skip_edge = true;
}

void ace2k_heat_zerocross(struct ace2k_heat *self, uint32_t now_us)
{
    uint32_t mode = self->fire_mode; /* read once: the one word that decides this edge */
    bool prev_in_phase = self->prev_in_phase;
    bool in_phase = edge_in_phase(self, now_us); /* every edge, in every mode */
    self->prev_in_phase = in_phase;
    /* A cycle starts only on an edge in phase whose previous edge was too: that edge is the one
     * the odd edge will be judged against, so an edge out of phase just before it (a noise edge,
     * a missed one) would leave the odd half unjudgeable — the even half alone. */
    bool start_ok = false;
    if (in_phase) {
        start_ok = prev_in_phase;
    }
    if (mode == ACE2K_HEAT_FIRE_PENDING) {
        pending_edge(self, now_us, in_phase);
        return;
    }
    if (mode != ACE2K_HEAT_FIRE_LEASED) {
        return;
    }
    leased_edge(self, now_us, in_phase, start_ok);
}

/* Rule 5.  The gate is read before last_fire_us: a pulse that raised it is already recorded.
 * A fire landing between the two reads makes last_fire_us newer than now_us — a negative age,
 * never stuck. */
static void check_gate(struct ace2k_heat *self, uint32_t now_us)
{
    bool high = self->ops->gate_read(self->ctx);
    bool valid = self->fired_pending_check;
    int32_t age = (int32_t)(now_us - self->last_fire_us);
    if (high) {
        if (!valid || age > (int32_t)ACE2K_HEAT_GATE_STUCK_US) {
            latch(self, ACE2K_HEAT_GATE_STUCK);
        }
        return;
    }
    if (valid && age > (int32_t)ACE2K_HEAT_GATE_STUCK_US) {
        self->fired_pending_check = false; /* long over: no stale age after the µs wrap */
    }
}

/* A pending odd half whose edge never came (the mains gone) is dropped once past the window:
 * NONE first, then the cycle fields, which the interrupt no longer touches. */
static void pending_expire(struct ace2k_heat *self, uint32_t now_us)
{
    if (self->fire_mode != ACE2K_HEAT_FIRE_PENDING) {
        return;
    }
    if (now_us - self->even_us > ACE2K_HEAT_PENDING_MAX_US) {
        silence(self);
        cycle_reset(self);
    }
}

/* The tick's last store: LEASED while leased on an ok verdict; otherwise a LEASED mode left by
 * a lease that ended goes to NONE, and a PENDING one stays the interrupt's (its verdict was
 * judged at the tick's start). */
static void publish_mode(struct ace2k_heat *self)
{
    if (self->state == ACE2K_HEAT_LEASED && self->verdict == ACE2K_HEAT_OK) {
        self->fire_mode = ACE2K_HEAT_FIRE_LEASED;
        return;
    }
    if (self->fire_mode == ACE2K_HEAT_FIRE_LEASED) {
        self->fire_mode = ACE2K_HEAT_FIRE_NONE;
    }
}

/* The invalid NTC's leaky bucket: an invalid tick adds ACE2K_HEAT_NTC_INVALID_WEIGHT, a valid
 * one drains one.  True once the level reaches ACE2K_HEAT_NTC_INVALID_LEVEL. */
static bool ntc_bucket_full(struct ace2k_heat *self, bool invalid)
{
    if (!invalid) {
        if (self->ntc_invalid_level != 0U) {
            self->ntc_invalid_level--;
        }
        return false;
    }
    self->ntc_invalid_level += ACE2K_HEAT_NTC_INVALID_WEIGHT;
    return self->ntc_invalid_level >= ACE2K_HEAT_NTC_INVALID_LEVEL;
}

/* The implausible mains' leaky bucket, every tick in every state: an implausible tick adds
 * ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT, a plausible one drains one; saturated at the level, so the
 * first plausible tick takes it below. */
static void mains_bucket_step(struct ace2k_heat *self, bool plausible)
{
    if (plausible) {
        if (self->mains_level != 0U) {
            self->mains_level--;
        }
        return;
    }
    uint32_t level = (uint32_t)self->mains_level + ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT;
    if (level > ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL) {
        level = ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL;
    }
    self->mains_level = (uint16_t)level;
}

/* The period the edges must keep, from the measured frequency: 0 while it is implausible. */
static uint32_t expected_period_us(const struct ace2k_heat_inputs *in)
{
    if (!ace2k_mains_hz10_plausible(in->mains_hz10)) {
        return 0U;
    }
    return ACE2K_HEAT_HZ10_PERIOD_US / in->mains_hz10;
}

/* LEASED: an invalid NTC or an implausible mains stops the firing (fire_mode NONE) and latches
 * once its bucket is full; any other violation latches at once; then the lease's end. */
static void tick_leased(struct ace2k_heat *self, const struct ace2k_heat_inputs *in,
                        uint32_t now_ms)
{
    if (self->verdict != ACE2K_HEAT_OK && !reason_debounced(self->verdict)) {
        latch(self, self->verdict);
        return;
    }
    if (ntc_bucket_full(self, ntc_invalid(in))) {
        latch(self, ACE2K_HEAT_NTC_INVALID);
        return;
    }
    if (ace2k_heat_mains_implausible_held(self)) {
        latch(self, ACE2K_HEAT_MAINS_IMPLAUSIBLE);
        return;
    }
    if (ace2k_time_after(now_ms, self->lease_end_ms)) {
        stop(self, ACE2K_HEAT_STOP_LEAVE); /* the cycle guard left no cycle half done */
    }
}

void ace2k_heat_tick(struct ace2k_heat *self, const struct ace2k_heat_inputs *in, uint32_t now_ms,
                     uint32_t now_us)
{
    self->limits = limits_reason(in);
    self->verdict = verdict_reason(in, self->limits, self->duty_pct);
    self->mains_measured = in->mains_measured;
    self->ntc_invalid = ntc_invalid(in);
    if (self->verdict != ACE2K_HEAT_OK) {
        silence(self); /* a verdict not ok: no half-cycle from the next edge on, pending or not */
        cycle_reset(self); /* NONE: the fields are the tick's; a resumed lease starts afresh */
    }
    if (in->mains_hz10 != 0U) { /* no window measured yet (boot): not implausible, the bucket
                                  * holds — nothing fires anyway without a measured period */
        mains_bucket_step(self, ace2k_mains_hz10_plausible(in->mains_hz10));
    }
    self->period_us = expected_period_us(in);
    check_gate(self, now_us);
    if (in->cutout) {
        self->cutout_seen = true;
        latch(self, ACE2K_HEAT_CUTOUT);
    }
    if (self->state == ACE2K_HEAT_LEASED) {
        tick_leased(self, in, now_ms);
    } else {
        self->ntc_invalid_level = 0U;
    }
    pending_expire(self, now_us);
    self->now_ms = now_ms;
    self->edges_since_tick = 0U;
    ACE2K_COMPILER_BARRIER();
    publish_mode(self);
}

uint8_t ace2k_heat_clear_blocker(const struct ace2k_heat *self)
{
    if (self->ops->gate_read(self->ctx)) {
        return ACE2K_HEAT_GATE_STUCK; /* first: a gate reading high is heat on (ace2k_heat_active) */
    }
    if (self->reason == ACE2K_HEAT_CUTOUT || self->cutout_seen) {
        return ACE2K_HEAT_CUTOUT;
    }
    if (self->limits != ACE2K_HEAT_OK && self->limits != ACE2K_HEAT_MAINS_IMPLAUSIBLE) {
        return self->limits; /* the hard limits, an invalid NTC */
    }
    if (self->reason == ACE2K_HEAT_MAINS_IMPLAUSIBLE && !ace2k_heat_mains_recovered(self)) {
        return ACE2K_HEAT_MAINS_IMPLAUSIBLE; /* judged on the bucket, not the last reading */
    }
    return ACE2K_HEAT_OK;
}

int ace2k_heat_clear(struct ace2k_heat *self)
{
    if (self->state != ACE2K_HEAT_LATCHED) {
        return 0;
    }
    if (self->reason == ACE2K_HEAT_CUTOUT || self->cutout_seen) {
        return -ACE2K_EREFUSED; /* never by command, whatever else holds it */
    }
    uint8_t blocker = ace2k_heat_clear_blocker(self);
    if (blocker != ACE2K_HEAT_OK) {
        return -ACE2K_EBUSY;
    }
    self->reason = ACE2K_HEAT_OK;
    self->duty_pct = 0U;
    self->state = ACE2K_HEAT_IDLE;
    return 0;
}

bool ace2k_heat_leased(const struct ace2k_heat *self)
{
    return self->state == ACE2K_HEAT_LEASED;
}

bool ace2k_heat_mains_measured(const struct ace2k_heat *self)
{
    return self->mains_measured;
}

bool ace2k_heat_mains_implausible_held(const struct ace2k_heat *self)
{
    return self->mains_level >= ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL;
}

bool ace2k_heat_mains_recovered(const struct ace2k_heat *self)
{
    return self->mains_level <= ACE2K_HEAT_MAINS_RESTART_LEVEL;
}

bool ace2k_heat_firing(const struct ace2k_heat *self)
{
    if (self->state != ACE2K_HEAT_LEASED) {
        return false;
    }
    return self->fire_mode == ACE2K_HEAT_FIRE_LEASED;
}

bool ace2k_heat_busy(const struct ace2k_heat *self)
{
    if (self->state == ACE2K_HEAT_LEASED || self->fire_mode == ACE2K_HEAT_FIRE_PENDING) {
        return true;
    }
    /* Latched or shut down: the gate was commanded off and nothing raises it again, so a pulse
     * check still open — a gate stuck high — does not hold the flash (the latch's log entry must
     * reach it).  ace2k_heat_active() still sees the gate. */
    if (self->state == ACE2K_HEAT_LATCHED || self->shut) {
        return false;
    }
    return self->fired_pending_check;
}

bool ace2k_heat_active(const struct ace2k_heat *self)
{
    if (ace2k_heat_busy(self)) {
        return true;
    }
    return self->ops->gate_read(self->ctx); /* a gate stuck high: the heater may conduct */
}
