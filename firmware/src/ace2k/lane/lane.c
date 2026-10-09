#include "lane/lane.h"

/* Duty → speed, unloaded (no filament, no spool), at 20, 30 … 100 % duty: the tach rate measured
 * at each duty on the bench — 190, 307, 423, 538, 649, 762, 880, 992, 1126 pulses/s, a straight
 * line within ±2 %.  The loaded floor is the first entry's duty. */
static const uint32_t ace2k_lane_ff_um_s[ACE2K_LANE_FF_POINTS] = {
    15384, 24858, 34251, 43562, 52550, 61700, 71255, 80323, 91173,
};
/* The anti-windup's lower bound at the floor setpoint is (DUTY_MIN − ff) × KI_DIV = 0 only while
 * the table's first entry is the floor duty; a probe image lowers both together (lane.h). */
_Static_assert(ACE2K_LANE_FF_FIRST_PCT == ACE2K_LANE_DUTY_MIN_PCT,
               "the table's first entry is the floor duty: the anti-windup's lower bound is 0 at "
               "the floor setpoint");

/* The lower bound rounds up: 80 % of 12342 is 9873.6, and 9873 is outside the tolerance. */
#define SCALE_MIN_X10                                                                              \
    (((ACE2K_LANE_UM_PER_COUNT_X10 * (ACE2K_LANE_PCT_MAX - ACE2K_LANE_SCALE_TOLERANCE_PCT)) +      \
      ACE2K_LANE_PCT_MAX - 1U) /                                                                   \
     ACE2K_LANE_PCT_MAX)
#define SCALE_MAX_X10                                                                              \
    (ACE2K_LANE_UM_PER_COUNT_X10 * (ACE2K_LANE_PCT_MAX + ACE2K_LANE_SCALE_TOLERANCE_PCT) /         \
     ACE2K_LANE_PCT_MAX)

static uint8_t clamp_duty(int32_t duty_pct)
{
    return (uint8_t)ace2k_clamp_i32(duty_pct, (int32_t)ACE2K_LANE_DUTY_MIN_PCT,
                                    (int32_t)ACE2K_LANE_DUTY_MAX_PCT);
}

uint8_t ace2k_lane_feed_forward_pct(uint32_t speed_um_s)
{
    if (speed_um_s <= ace2k_lane_ff_um_s[0]) {
        return (uint8_t)ACE2K_LANE_FF_FIRST_PCT;
    }
    for (uint32_t i = 0; i + 1 < ACE2K_LANE_FF_POINTS; i++) {
        if (speed_um_s <= ace2k_lane_ff_um_s[i + 1]) {
            uint32_t span = ace2k_lane_ff_um_s[i + 1] - ace2k_lane_ff_um_s[i];
            uint32_t frac =
                span ? (speed_um_s - ace2k_lane_ff_um_s[i]) * ACE2K_LANE_FF_STEP_PCT / span : 0;
            return clamp_duty(
                (int32_t)(ACE2K_LANE_FF_FIRST_PCT + (i * ACE2K_LANE_FF_STEP_PCT) + frac));
        }
    }
    return (uint8_t)ACE2K_LANE_DUTY_MAX_PCT;
}

void ace2k_lane_init(struct ace2k_lane *self, const struct ace2k_lane_ops *ops, void *ctx)
{
    *self = (struct ace2k_lane){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        self->lane[i].um_per_count_x10 = ACE2K_LANE_UM_PER_COUNT_X10;
    }
}

/* util.h clamps 32-bit values; the micrometre product needs 64 bits before it is narrowed. */
static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    return (int32_t)value;
}

static int32_t counts_to_um(int32_t counts, uint32_t um_per_count_x10)
{
    int64_t um10 = (int64_t)counts * (int64_t)um_per_count_x10;
    /* saturate instead of wrapping: a count past the range reads as the rail, never as a
     * plausible value of the opposite sign */
    return clamp_i64_to_i32(um10 / 10);
}

/* Rounded to nearest; in 64 bits so that any um fits (a move's is at most 2 000 000). */
static uint32_t um_to_counts(uint32_t um, uint32_t um_per_count_x10)
{
    return (uint32_t)((((uint64_t)um * 10U) + (um_per_count_x10 / 2U)) / um_per_count_x10);
}

/* The strand's travel along the command since the move began, in counts: the encoder relative
 * to the start, negated for a reverse move — negative when the strand went the other way. */
static int32_t travel_counts(const struct ace2k_lane_counters *l)
{
    int32_t delta = l->encoder_count - l->move.enc_start;
    return l->move.reverse ? -delta : delta;
}

/* Counts still to travel along the command: 0 once the target is reached; a strand behind its
 * start owes the way back as well.  In 64 bits because travel_counts() is any int32: a strand far
 * behind its start (travel near INT32_MIN) puts target − travel above INT32_MAX, which a 32-bit
 * signed difference would wrap.  The saturation is a guard only: a target is at most
 * ACE2K_LANE_MOVE_MAX_UM in counts (about 2 000), so the difference stays below UINT32_MAX. */
static uint32_t left_counts(const struct ace2k_lane_counters *l)
{
    int64_t remaining = (int64_t)l->move.target_counts - (int64_t)travel_counts(l);
    if (remaining <= 0) {
        return 0;
    }
    return remaining > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

/* The travel left in micrometres at the lane's scale, saturated: what the landing and the
 * deadline of a speed change are decided on. */
static uint32_t left_um(const struct ace2k_lane_counters *l)
{
    uint64_t um = ((uint64_t)left_counts(l) * l->um_per_count_x10) / 10U;
    return um > UINT32_MAX ? UINT32_MAX : (uint32_t)um;
}

static void drive_stop(struct ace2k_lane *self, uint8_t lane)
{
    self->ops->run_set(self->ctx, lane, false);
    self->ops->pwm_set(self->ctx, lane, 0);
}

/* The move is over: the lines at stop, the result recorded.  A pending one is overwritten — the
 * consumer pops every tick — and counted as dropped. */
static void end_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_outcome outcome,
                     uint32_t now_ms)
{
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    drive_stop(self, lane);
    if (l->result_pending) {
        l->results_dropped++;
    }
    l->result.outcome = (uint8_t)outcome;
    l->result.motor_um = ace2k_lane_fg_to_um(l->fg_raw - m->fg_start);
    l->result.filament_um = counts_to_um(l->encoder_count - m->enc_start, l->um_per_count_x10);
    l->result.duration_ms = ace2k_time_since(now_ms, m->t_start_ms);
    ACE2K_COMPILER_BARRIER(); /* payload before the flag */
    l->result_pending = true;
    m->active = false;
    m->measured_um_s = 0;
    m->duty_pct = 0;
}

/* True when the duty as applied — ff + P + the integral as it stands, before the clamp — is past
 * a bound and the error pushes it further that way: below the floor with the motor too fast,
 * above the ceiling with it too slow.  A duty exactly at a bound is not past it: the integral
 * still moves, so it can cross the bound in either direction. */
static bool winds_up(int32_t applied_pct, int32_t error_um_s)
{
    if (applied_pct < (int32_t)ACE2K_LANE_DUTY_MIN_PCT) {
        return error_um_s < 0;
    }
    if (applied_pct > (int32_t)ACE2K_LANE_DUTY_MAX_PCT) {
        return error_um_s > 0;
    }
    return false;
}

/* The setpoint in effect: the floor in the landing, the commanded speed before it. */
static uint32_t setpoint_of(const struct ace2k_lane_move *m)
{
    return m->landing ? ACE2K_LANE_SPEED_MIN_UM_S : m->speed_um_s;
}

/* Every ACE2K_LANE_LOOP_TICKS ticks: the tach rate over the interval, a PI around the
 * feed-forward duty, the duty clamped to the bounds.  Two anti-windup mechanisms, one after the
 * other.  First the conditional integration (measured on the unit): this loop's error stays
 * out of the integral while the duty applied with the integral as it stands is already past a
 * bound in the error's direction; the ±INTEG_MAX clamp stays.  Then the integral is clamped
 * into the feed-forward window [(DUTY_MIN − ff) × KI_DIV, (DUTY_MAX − ff) × KI_DIV]: it never
 * holds more than the table can absorb.  The duty is ff + P + integral / KI_DIV, clamped.
 * Three properties: (a) the P term never enters the integral, so a landing from a cruise holds
 * the floor for as long as the motor is above the floor setpoint — nothing accumulates while P
 * keeps the duty there, and a negative cruise integral is dropped by the window at the landing
 * (it could not have gone under the floor anyway); (b) a load arriving after an over-speed
 * episode is answered on the first loop — the integral was frozen while P held the floor, not
 * wound down, so nothing has to unwind (the window alone let it wind down to its lower bound and
 * answered ~7 loops late); (c) a setpoint increase (30 → 70 mm/s: ff 35 → 78 %) drops a large
 * integral to what the new entry can absorb, (100 − 78) × 6000 = 132 000, at once instead of
 * unwinding it over many loops.
 * Known residue, accepted: the integral may sit up to one loop's error past a bound (≤ ~3 %)
 * and unwinds one loop late — landing it exactly on the bound was tried in two forms, and each
 * regressed a behaviour measured on the unit. */
static void speed_loop(struct ace2k_lane *self, uint8_t lane)
{
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    uint32_t pulses = l->fg_raw - m->fg_loop;
    m->fg_loop = l->fg_raw;
    m->measured_um_s = ace2k_lane_fg_to_um(pulses * ACE2K_LANE_LOOPS_PER_S);
    uint32_t setpoint = setpoint_of(m);
    m->loop_setpoint_um_s = setpoint;
    int32_t error = (int32_t)setpoint - (int32_t)m->measured_um_s;
    int32_t ff = (int32_t)ace2k_lane_feed_forward_pct(setpoint);
    int32_t p = error / ACE2K_LANE_KP_DIV;
    int32_t integ_max = ACE2K_LANE_INTEG_MAX_PCT * ACE2K_LANE_KI_DIV;
    if (!winds_up(ff + p + (m->integ_um_s / ACE2K_LANE_KI_DIV), error)) {
        m->integ_um_s = ace2k_clamp_i32(m->integ_um_s + error, -integ_max, integ_max);
    }
    /* ff lies within the duty bounds, so low ≤ 0 ≤ high and the window is never empty */
    int32_t low = ace2k_clamp_i32(((int32_t)ACE2K_LANE_DUTY_MIN_PCT - ff) * ACE2K_LANE_KI_DIV,
                                  -integ_max, integ_max);
    int32_t high = ace2k_clamp_i32(((int32_t)ACE2K_LANE_DUTY_MAX_PCT - ff) * ACE2K_LANE_KI_DIV,
                                   -integ_max, integ_max);
    m->integ_um_s = ace2k_clamp_i32(m->integ_um_s, low, high);
    m->duty_pct = clamp_duty(ff + p + (m->integ_um_s / ACE2K_LANE_KI_DIV));
    self->ops->pwm_set(self->ctx, lane, m->duty_pct);
}

static void run_move(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms, bool link_ok)
{
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    if (!link_ok) {
        end_move(self, lane, ACE2K_LANE_LINK_LOST, now_ms);
        return;
    }
    if (l->fg_raw != m->fg_seen) {
        m->fg_seen = l->fg_raw;
        m->t_fg_seen_ms = now_ms;
    } else if (ace2k_time_since(now_ms, m->t_fg_seen_ms) >= ACE2K_LANE_STALL_MS) {
        end_move(self, lane, ACE2K_LANE_STALLED, now_ms);
        return;
    }
    /* the position stop closes on the strand: the motor's travel is the result's odometer, not
     * the target — under a spool the drive gear slips a few per cent, and the host asked for filament */
    if (left_counts(l) == 0) {
        end_move(self, lane, ACE2K_LANE_DONE, now_ms);
        return;
    }
    if (ace2k_time_after(now_ms, m->deadline_ms)) {
        /* a move that gets here has a strand short of the length — none, jammed, or slipping
         * past the margin */
        end_move(self, lane, ACE2K_LANE_TIMEOUT, now_ms);
        return;
    }
    if (left_um(l) <= ACE2K_LANE_LANDING_UM) {
        m->landing = true;
    }
    m->loop_ticks++;
    if (m->loop_ticks >= ACE2K_LANE_LOOP_TICKS) {
        m->loop_ticks = 0;
        speed_loop(self, lane);
    }
}

void ace2k_lane_tick(struct ace2k_lane *self, uint32_t now_ms, bool link_ok)
{
    uint8_t resets = self->reset_pending;
    self->reset_pending = 0;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        struct ace2k_lane_counters *l = &self->lane[i];
        uint16_t raw = self->ops->encoder_read(self->ctx, i);
        uint32_t fg = self->ops->fg_read(self->ctx, i);
        if (!self->primed) {
            l->encoder_last_raw = raw;
            l->fg_base = fg;
        }
        /* the difference of two uint16 readings, taken as int16: ±32767 counts per tick */
        int16_t delta = (int16_t)(uint16_t)(raw - l->encoder_last_raw);
        l->encoder_count += delta;
        l->encoder_last_raw = raw;
        l->fg_raw = fg;
        if (resets & (1U << i)) {
            l->encoder_count = 0;
            l->fg_base = fg;
        }
        l->fg = l->fg_raw - l->fg_base; /* after the reset: one store the task reads whole */
        if (l->move.active) {
            run_move(self, i, now_ms, link_ok);
        }
    }
    self->primed = true;
}

int32_t ace2k_lane_encoder_count(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].encoder_count : 0;
}

int32_t ace2k_lane_encoder_um(const struct ace2k_lane *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return 0;
    }
    return counts_to_um(self->lane[lane].encoder_count, self->lane[lane].um_per_count_x10);
}

uint32_t ace2k_lane_fg(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].fg : 0;
}

/* Refused while the lane moves: the tick would zero the count and move the base under a move
 * whose start values are the old ones, and every result after that would be off by them. */
int ace2k_lane_reset(struct ace2k_lane *self, uint8_t lane)
{
    if (lane == ACE2K_LANE_ALL) {
        if (ace2k_lane_any_moving(self)) {
            return -ACE2K_EREFUSED;
        }
        /* The lane count is a signed literal, and the lint refuses one as a shift count. */
        self->reset_pending = (uint8_t)((1U << (unsigned)ACE2K_LANE_COUNT) - 1U);
        return 0;
    }
    if (lane >= ACE2K_LANE_COUNT) {
        return -ACE2K_EINVAL;
    }
    if (self->lane[lane].move.active) {
        return -ACE2K_EREFUSED;
    }
    self->reset_pending |= (uint8_t)(1U << lane);
    return 0;
}

/* A reset still pending for the lane, landed now: the count zeroed and the tach's base moved, as
 * the tick would at its next pass.  Every base taken after this call is the reset's (a move's in
 * begin_move(), a feed mode's in ace2k_feed_enter()).  Under the caller's mask. */
void ace2k_lane_land_reset(struct ace2k_lane *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || (self->reset_pending & (1U << lane)) == 0U) {
        return;
    }
    struct ace2k_lane_counters *l = &self->lane[lane];
    self->reset_pending = (uint8_t)(self->reset_pending & ~(1U << lane));
    l->encoder_count = 0;
    l->fg_base = l->fg_raw;
    l->fg = 0;
}

/* When a move over remaining_um at speed_um_s must be over: 3/2 of the expected time plus the
 * margin, from now.  The expected time budgets the landing — the last ACE2K_LANE_LANDING_UM, or
 * the whole of a shorter move — at the floor speed, the rest at the commanded speed: with the
 * floor at 9 mm/s the landing alone takes over a second, and a short move at the ceiling budgeted
 * at the ceiling would end within a few hundred milliseconds of its deadline under a heavy spool.
 * The products in 64 bits: 2 000 000 µm × 1000 ms overflows 32. */
static uint32_t deadline_for(uint32_t remaining_um, uint32_t speed_um_s, uint32_t now_ms)
{
    uint32_t landing_um =
        remaining_um < ACE2K_LANE_LANDING_UM ? remaining_um : ACE2K_LANE_LANDING_UM;
    uint32_t cruise_um = remaining_um - landing_um;
    uint32_t expected_ms = (uint32_t)(((uint64_t)cruise_um * 1000U) / speed_um_s) +
                           (uint32_t)(((uint64_t)landing_um * 1000U) / ACE2K_LANE_SPEED_MIN_UM_S);
    return now_ms + (expected_ms * ACE2K_LANE_DEADLINE_NUM / ACE2K_LANE_DEADLINE_DEN) +
           ACE2K_LANE_DEADLINE_MARGIN_MS;
}

/* What every kind of start shares, once the caller has filled the move's own fields (reverse,
 * speed_um_s, target_counts, deadline_ms, duty_pct): a reset still pending for the lane
 * lands first, then the bases, the tach watch, the ops, then active.  The reset lands here, under
 * the caller's mask, because the tick would otherwise zero the count and move the base under a
 * move whose bases are the old ones, and its every result would be off by the old count. */
static void begin_move(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms)
{
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    ace2k_lane_land_reset(self, lane);
    m->fg_start = l->fg_raw;
    m->enc_start = l->encoder_count;
    m->t_start_ms = now_ms;
    m->fg_seen = l->fg_raw;
    m->t_fg_seen_ms = now_ms;
    m->fg_loop = l->fg_raw;
    /* direction first, never while run is high (the feed's rule 4); then run; then the drive */
    self->ops->dir_set(self->ctx, lane, m->reverse);
    self->ops->run_set(self->ctx, lane, true);
    self->ops->pwm_set(self->ctx, lane, m->duty_pct);
    m->active = true;
}

int ace2k_lane_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || length_um == 0 || length_um > ACE2K_LANE_MOVE_MAX_UM ||
        !ace2k_lane_speed_in_bounds(speed_um_s)) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_lane_move *m = &self->lane[lane].move;
    if (m->active) {
        return -ACE2K_EREFUSED;
    }
    *m = (struct ace2k_lane_move){ .active = false };
    m->reverse = (dir == ACE2K_LANE_REVERSE);
    m->speed_um_s = speed_um_s;
    m->target_counts = um_to_counts(length_um, self->lane[lane].um_per_count_x10);
    if (m->target_counts == 0) {
        m->target_counts = 1; /* a length under one count still ends on a count */
    }
    m->deadline_ms = deadline_for(length_um, speed_um_s, now_ms);
    m->duty_pct = ace2k_lane_feed_forward_pct(speed_um_s);
    m->loop_setpoint_um_s = speed_um_s;
    begin_move(self, lane, now_ms);
    return 0;
}

static void stop_one(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms)
{
    if (self->lane[lane].move.active) {
        end_move(self, lane, ACE2K_LANE_STOPPED, now_ms);
    } else {
        drive_stop(self, lane); /* the feed's rule 5: the lines at stop whatever the state */
    }
}

void ace2k_lane_stop(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms)
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

void ace2k_lane_abort_all(struct ace2k_lane *self, enum ace2k_lane_outcome outcome, uint32_t now_ms)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (self->lane[i].move.active) {
            end_move(self, i, outcome, now_ms);
        }
    }
}

int ace2k_lane_set_speed(struct ace2k_lane *self, uint8_t lane, uint32_t speed_um_s,
                         uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || !ace2k_lane_speed_in_bounds(speed_um_s)) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    if (!m->active) {
        return -ACE2K_EREFUSED;
    }
    m->speed_um_s = speed_um_s; /* in effect at the next loop */
    /* the deadline follows the setpoint: what remains on the encoder, at the new speed, from
     * now — a move slowed to the floor would otherwise time out on the deadline of its first
     * speed */
    m->deadline_ms = deadline_for(left_um(l), speed_um_s, now_ms);
    return 0;
}

bool ace2k_lane_is_moving(const struct ace2k_lane *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return false;
    }
    return self->lane[lane].move.active;
}

bool ace2k_lane_is_reverse(const struct ace2k_lane *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || !self->lane[lane].move.active) {
        return false;
    }
    return self->lane[lane].move.reverse;
}

bool ace2k_lane_any_moving(const struct ace2k_lane *self)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        if (self->lane[i].move.active) {
            return true;
        }
    }
    return false;
}

bool ace2k_lane_pop_result(struct ace2k_lane *self, uint8_t lane, struct ace2k_lane_result *out)
{
    if (lane >= ACE2K_LANE_COUNT || !self->lane[lane].result_pending) {
        return false;
    }
    struct ace2k_lane_counters *l = &self->lane[lane];
    *out = l->result;
    ACE2K_COMPILER_BARRIER(); /* payload before the flag */
    l->result_pending = false;
    return true;
}

uint32_t ace2k_lane_speed_um_s(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].move.measured_um_s : 0;
}

uint8_t ace2k_lane_duty_pct(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].move.duty_pct : 0;
}

uint32_t ace2k_lane_setpoint_um_s(const struct ace2k_lane *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || !self->lane[lane].move.active) {
        return 0;
    }
    return self->lane[lane].move.loop_setpoint_um_s;
}

int ace2k_lane_scale_set(struct ace2k_lane *self, uint8_t lane, uint32_t um_per_count_x10)
{
    if (lane >= ACE2K_LANE_COUNT || um_per_count_x10 < SCALE_MIN_X10 ||
        um_per_count_x10 > SCALE_MAX_X10) {
        return -ACE2K_EINVAL;
    }
    self->lane[lane].um_per_count_x10 = um_per_count_x10;
    return 0;
}

uint32_t ace2k_lane_scale(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].um_per_count_x10 : 0;
}

uint32_t ace2k_lane_um_to_counts(const struct ace2k_lane *self, uint8_t lane, uint32_t um)
{
    return lane < ACE2K_LANE_COUNT ? um_to_counts(um, self->lane[lane].um_per_count_x10) : 0;
}
