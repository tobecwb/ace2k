#include "lane/lane.h"

/* Duty to speed, unloaded (no filament, no spool), at 20, 30 … 100 % duty.  Each entry is the tach
 * rate measured on the unit at that duty: 190, 307, 423, 538, 649, 762, 880, 992 and
 * 1126 pulses/s.  They lie on a straight line within ±2 %.  The duty of the first entry is the
 * loaded floor. */
static const uint32_t ff_um_s[ACE2K_LANE_FF_POINTS] = {
    15384, 24858, 34251, 43562, 52550, 61700, 71255, 80323, 91173,
};
/* At the floor setpoint, the lower bound of the anti-windup is (DUTY_MIN − ff) × KI_DIV.  It is 0
 * only while the first entry of the table is the floor duty.  A probe image lowers both together
 * (lane.h). */
_Static_assert(ACE2K_LANE_FF_FIRST_PCT == ACE2K_LANE_DUTY_MIN_PCT,
               "the table's first entry is the floor duty: the anti-windup's lower bound is 0 at "
               "the floor setpoint");

/* The lower bound rounds up.  80 % of 12342 is 9873.6, and 9873 is outside the tolerance. */
#define SCALE_MIN_X10                                                                              \
    (((ACE2K_LANE_UM_PER_COUNT_X10 * (ACE2K_PCT_FULL - ACE2K_LANE_SCALE_TOLERANCE_PCT)) +          \
      ACE2K_PCT_FULL - 1U) /                                                                       \
     ACE2K_PCT_FULL)

static uint8_t clamp_duty(int32_t duty_pct)
{
    return (uint8_t)ace2k_clamp_i32(duty_pct, (int32_t)ACE2K_LANE_DUTY_MIN_PCT,
                                    (int32_t)ACE2K_LANE_DUTY_MAX_PCT);
}

uint8_t ace2k_lane_feed_forward_pct(uint32_t speed_um_s)
{
    if (speed_um_s <= ff_um_s[0]) {
        return (uint8_t)ACE2K_LANE_FF_FIRST_PCT;
    }
    for (uint32_t i = 0; i + 1 < ACE2K_LANE_FF_POINTS; i++) {
        if (speed_um_s <= ff_um_s[i + 1]) {
            uint32_t span = ff_um_s[i + 1] - ff_um_s[i];
            uint32_t frac = span ? (speed_um_s - ff_um_s[i]) * ACE2K_LANE_FF_STEP_PCT / span : 0;
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

/* util.h clamps 32-bit values, but the product in micrometres needs 64 bits before it is
 * narrowed. */
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
    /* Saturate instead of wrapping.  A count past the range reads as the limit, never as a
     * plausible value of the opposite sign. */
    return clamp_i64_to_i32(um10 / 10);
}

/* Rounds to nearest.  It works in 64 bits so that any um fits.  The length of a move is at most
 * 2 000 000. */
static uint32_t um_to_counts(uint32_t um, uint32_t um_per_count_x10)
{
    return (uint32_t)((((uint64_t)um * 10U) + (um_per_count_x10 / 2U)) / um_per_count_x10);
}

/* The travel of the filament along the command since the move began, in counts.  It is the encoder
 * relative to the start, negated for a reverse move.  It is negative when the filament went the
 * other way. */
static int32_t travel_counts(const struct ace2k_lane_counters *l)
{
    int32_t delta = l->encoder_count - l->move.enc_start;
    return l->move.reverse ? -delta : delta;
}

/* The counts still to travel along the command.  It is 0 once the target is reached.  A filament
 * behind its start owes the way back as well.  It works in 64 bits because travel_counts() can be
 * any int32.  A filament far behind its start, with a travel near INT32_MIN, puts target − travel
 * above INT32_MAX, which a 32-bit signed difference would wrap.  The saturation is only a guard.  A
 * target is at most ACE2K_LANE_MOVE_MAX_UM in counts, about 2 000, so the difference stays below
 * UINT32_MAX. */
static uint32_t left_counts(const struct ace2k_lane_counters *l)
{
    int64_t remaining = (int64_t)l->move.target_counts - (int64_t)travel_counts(l);
    if (remaining <= 0) {
        return 0;
    }
    return remaining > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

/* Converts counts to micrometres at a scale, saturated. */
static uint32_t counts_span_um(uint32_t counts, uint32_t um_per_count_x10)
{
    uint64_t um = ((uint64_t)counts * um_per_count_x10) / ACE2K_LANE_SCALE_TENTHS_PER_UM;
    return um > UINT32_MAX ? UINT32_MAX : (uint32_t)um;
}

/* The travel left in micrometres at the lane's scale, saturated.  The landing and the deadline of a
 * speed change are decided on it. */
static uint32_t left_um(const struct ace2k_lane_counters *l)
{
    return counts_span_um(left_counts(l), l->um_per_count_x10);
}

/* The landing's one test, at the start of a move and on every tick.  A move is in its landing when
 * ACE2K_LANE_LANDING_UM or less is left on the encoder. */
static bool in_landing(uint32_t remaining_um)
{
    return remaining_um <= ACE2K_LANE_LANDING_UM;
}

static void drive_stop(struct ace2k_lane *self, uint8_t lane)
{
    self->ops->run_set(self->ctx, lane, false);
    self->ops->pwm_set(self->ctx, lane, 0);
}

/* Ends the move.  The lines go to stop, and the result is recorded.  A pending result is
 * overwritten and counted as dropped.  The consumer pops the result every tick. */
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

/* Returns true when the duty as applied is past a bound and the error pushes it further that way.
 * The duty as applied is ff + P + the integral as it stands, before the clamp.  Past a bound means
 * below the floor with the motor too fast, or above the ceiling with the motor too slow.  A duty
 * exactly at a bound is not past it.  The integral still moves there, so it can cross the bound in
 * either direction. */
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

/* The setpoint in effect: the floor during the landing, and the commanded speed before it. */
static uint32_t setpoint_of(const struct ace2k_lane_move *m)
{
    return m->landing ? ACE2K_LANE_SPEED_MIN_UM_S : m->speed_um_s;
}

/* Clamps the integral into the feed-forward window of a duty.  The window is
 * (DUTY_MIN − ff) × KI_DIV up to (DUTY_MAX − ff) × KI_DIV, inside ±INTEG_MAX.  So the integral
 * never holds more than the table can absorb.  ff lies within the duty bounds, so
 * low ≤ 0 ≤ high, and the window is never empty. */
static int32_t in_window(int32_t integ_um_s, int32_t ff_pct)
{
    int32_t integ_max = ACE2K_LANE_INTEG_MAX_PCT * ACE2K_LANE_KI_DIV;
    int32_t low = ace2k_clamp_i32(((int32_t)ACE2K_LANE_DUTY_MIN_PCT - ff_pct) * ACE2K_LANE_KI_DIV,
                                  -integ_max, integ_max);
    int32_t high = ace2k_clamp_i32(((int32_t)ACE2K_LANE_DUTY_MAX_PCT - ff_pct) * ACE2K_LANE_KI_DIV,
                                   -integ_max, integ_max);
    return ace2k_clamp_i32(integ_um_s, low, high);
}

/* The setpoint less the speed that the last loop measured.  It is 0 before the first loop of the
 * move.  No speed is measured yet, and a motor that is only starting is not a slow one. */
static int32_t error_of(const struct ace2k_lane_move *m, uint32_t setpoint_um_s)
{
    if (!m->measured) {
        return 0;
    }
    return (int32_t)setpoint_um_s - (int32_t)m->measured_um_s;
}

/* Writes the duty for the setpoint in effect.  The duty is its feed-forward, plus P on error_of(),
 * plus the integral clamped into the setpoint's window, all clamped to the duty bounds.  It leaves
 * the integral as it is.  Three writes go through here: the start of a move, the tick where a
 * landing begins, and every loop.  At the start nothing is measured and the integral is 0, so the
 * duty is the feed-forward alone.  When a landing begins between two loops, the duty is what the
 * next loop would write if the motor kept its last measured speed, without the integration of that
 * loop.  So a motor faster than the floor drops to the floor duty at once.  A motor that the load
 * held back keeps the duty that the integral learned. */
static void apply_setpoint(struct ace2k_lane *self, uint8_t lane)
{
    struct ace2k_lane_move *m = &self->lane[lane].move;
    uint32_t setpoint = setpoint_of(m);
    int32_t ff = (int32_t)ace2k_lane_feed_forward_pct(setpoint);
    int32_t p = error_of(m, setpoint) / ACE2K_LANE_KP_DIV;
    m->loop_setpoint_um_s = setpoint;
    m->duty_pct = clamp_duty(ff + p + (in_window(m->integ_um_s, ff) / ACE2K_LANE_KI_DIV));
    self->ops->pwm_set(self->ctx, lane, m->duty_pct);
}

/* Runs every ACE2K_LANE_LOOP_TICKS ticks.  It measures the tach rate over the interval, runs a PI
 * around the feed-forward duty, and clamps the duty to the bounds.  The duty is
 * ff + P + integral / KI_DIV, clamped.
 * Two anti-windup mechanisms run, one after the other.
 * - First, the conditional integration (measured on the unit).  The error of this loop stays out of
 *   the integral while the duty, applied with the integral as it stands, is already past a bound in
 *   the direction of the error.  The ±INTEG_MAX clamp stays.
 * - Then the integral is clamped into the feed-forward window
 *   [(DUTY_MIN − ff) × KI_DIV, (DUTY_MAX − ff) × KI_DIV].  So it never holds more than the table
 *   can absorb.
 * This gives three properties.
 * - (a) The P term never enters the integral.  So a landing from a cruise holds the floor for as
 *   long as the motor is above the floor setpoint.  Nothing accumulates while P keeps the duty
 *   there.  The window drops a negative cruise integral at the landing, and that integral could not
 *   have gone under the floor anyway.
 * - (b) A load that arrives after a period of over-speed is answered on the first loop.  The
 *   integral was frozen while P held the floor, not wound down, so nothing has to unwind.  With the
 *   window alone, the integral wound down to its lower bound, and the loop answered about 7 loops
 *   late.
 * - (c) A setpoint increase, for example 30 → 80 mm/s with ff 35 → 89 %, drops a large integral at
 *   once to what the new entry can absorb, (100 − 89) × 6000 = 66 000.  It does not unwind it over
 *   many loops.
 * A known residue is accepted.  The integral may sit up to the error of one loop past a bound
 * (about 3 % or less), and unwinds one loop late.  Landing it exactly on the bound was tried in two
 * forms, and each made a behaviour measured on the unit worse. */
static void speed_loop(struct ace2k_lane *self, uint8_t lane)
{
    struct ace2k_lane_counters *l = &self->lane[lane];
    struct ace2k_lane_move *m = &l->move;
    uint32_t pulses = l->fg_raw - m->fg_loop;
    m->fg_loop = l->fg_raw;
    m->measured_um_s = ace2k_lane_fg_to_um(pulses * ACE2K_LANE_LOOPS_PER_S);
    m->measured = true;
    uint32_t setpoint = setpoint_of(m);
    int32_t error = error_of(m, setpoint);
    int32_t ff = (int32_t)ace2k_lane_feed_forward_pct(setpoint);
    int32_t p = error / ACE2K_LANE_KP_DIV;
    if (!winds_up(ff + p + (m->integ_um_s / ACE2K_LANE_KI_DIV), error)) {
        int32_t integ_max = ACE2K_LANE_INTEG_MAX_PCT * ACE2K_LANE_KI_DIV;
        m->integ_um_s = ace2k_clamp_i32(m->integ_um_s + error, -integ_max, integ_max);
    }
    m->integ_um_s = in_window(m->integ_um_s, ff);
    apply_setpoint(self, lane);
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
    /* The position stop closes on the filament.  The motor's travel is the result's odometer, not
     * the target.  Under a spool the drive gear slips a few per cent, and the host asked for
     * filament. */
    if (left_counts(l) == 0) {
        end_move(self, lane, ACE2K_LANE_DONE, now_ms);
        return;
    }
    if (ace2k_time_after(now_ms, m->deadline_ms)) {
        /* A move that gets here has a filament short of the length.  There is none, or it is
         * jammed, or it slips past the margin. */
        end_move(self, lane, ACE2K_LANE_TIMEOUT, now_ms);
        return;
    }
    bool lands = (bool)(!m->landing && !m->no_landing && in_landing(left_um(l)));
    if (lands) {
        m->landing = true;
    }
    m->loop_ticks++;
    if (m->loop_ticks >= ACE2K_LANE_LOOP_TICKS) {
        m->loop_ticks = 0;
        speed_loop(self, lane); /* the loop applies a landing that begins on this tick */
        return;
    }
    if (lands) {
        /* now, not at the next loop; at the ceiling, a wait of up to 90 ms is 6 mm of the 10 */
        apply_setpoint(self, lane);
    }
}

/* Counts the tach pulses since the last split in the direction last driven. */
static void split_tach(struct ace2k_lane_counters *l, uint32_t fg)
{
    uint32_t step = fg - l->fg_split; /* across the 2^32 wrap too */
    if (l->last_reverse) {
        l->fg_back += step;
    } else {
        l->fg_fwd += step;
    }
    l->fg_split = fg;
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
        if (self->primed) {
            split_tach(l, fg);
        } else {
            l->fg_split = fg;
        }
        l->fg_raw = fg;
        if (resets & (1U << i)) {
            l->encoder_count = 0;
            l->fg_base = fg;
        }
        l->fg = l->fg_raw - l->fg_base; /* after the reset; one store that the task reads whole */
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

int32_t ace2k_lane_counts_since_um(const struct ace2k_lane *self, uint8_t lane, int32_t base_counts)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return 0;
    }
    int32_t delta = (int32_t)((uint32_t)self->lane[lane].encoder_count - (uint32_t)base_counts);
    return counts_to_um(delta, self->lane[lane].um_per_count_x10);
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

uint32_t ace2k_lane_fg_raw(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->lane[lane].fg_raw : 0;
}

uint32_t ace2k_lane_motor_fwd_um(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? ace2k_lane_fg_to_um(self->lane[lane].fg_fwd) : 0;
}

uint32_t ace2k_lane_motor_back_um(const struct ace2k_lane *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? ace2k_lane_fg_to_um(self->lane[lane].fg_back) : 0;
}

/* Refused while the lane moves.  The tick would zero the count and move the base under a move whose
 * start values are the old ones.  Every result after that would be off by them. */
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

/* Lands now a reset still pending for the lane.  The count is zeroed and the base of the tach
 * moves, as the tick would do at its next pass.  Every base taken after this call belongs to the
 * reset: the base of a move in begin_move(), and of a feed mode in ace2k_feed_enter().  It runs
 * under the caller's mask. */
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

/* Returns when a move over remaining_um at speed_um_s must be over.  That is 3/2 of the expected
 * time, plus the margin, from now.  The expected time budgets the landing at the floor speed, and
 * the rest at the commanded speed.  The landing is the last ACE2K_LANE_LANDING_UM, or the whole of
 * a shorter move.  With the floor at 9 mm/s, the landing alone takes over a second.  A short move
 * at the ceiling, budgeted at the ceiling, would end within a few hundred milliseconds of its
 * deadline under a heavy spool.  The products are in 64 bits, because 2 000 000 µm × 1000 ms
 * overflows 32 bits. */
static uint32_t deadline_for(uint32_t remaining_um, uint32_t speed_um_s, uint32_t now_ms)
{
    uint32_t landing_um =
        remaining_um < ACE2K_LANE_LANDING_UM ? remaining_um : ACE2K_LANE_LANDING_UM;
    uint32_t cruise_um = remaining_um - landing_um;
    uint32_t expected_ms =
        (uint32_t)(((uint64_t)cruise_um * ACE2K_MS_PER_S) / speed_um_s) +
        (uint32_t)(((uint64_t)landing_um * ACE2K_MS_PER_S) / ACE2K_LANE_SPEED_MIN_UM_S);
    return now_ms + (expected_ms * ACE2K_LANE_DEADLINE_NUM / ACE2K_LANE_DEADLINE_DEN) +
           ACE2K_LANE_DEADLINE_MARGIN_MS;
}

/* What every kind of start shares, once the caller has filled the move's own fields (reverse,
 * speed_um_s, target_counts, deadline_ms, landing, no_landing).  A reset still pending for the lane
 * lands first.  Then come the bases, the tach watch, the ops, and then active.  The reset lands
 * here, under the caller's mask.  Otherwise the tick would zero the count and move the base under a
 * move whose bases are the old ones, and every result of that move would be off by the old
 * count. */
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
    /* The pulses since the last tick belong to the move before this one: a start within 10 ms of
     * a stop must not count its coast in the new direction.  Then the tach counts in this one. */
    if (self->primed) {
        split_tach(l, self->ops->fg_read(self->ctx, lane));
    }
    l->last_reverse = m->reverse;
    /* direction first, never while run is high (the feed's rule 4); then run; then PWM */
    self->ops->dir_set(self->ctx, lane, m->reverse);
    self->ops->run_set(self->ctx, lane, true);
    apply_setpoint(self, lane); /* nothing measured yet: the feed-forward of the setpoint */
    m->active = true;
}

/* The start shared by ace2k_lane_move() and ace2k_lane_move_unlanded().  lands says whether the
 * move has a landing at all. */
static int start_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                      uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms, bool lands)
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
    m->no_landing = (bool)!lands;
    /* Decided as the tick decides it, on the target in counts.  A move no longer than the landing
     * runs at the floor from its start. */
    m->landing = (bool)(lands && in_landing(counts_span_um(m->target_counts,
                                                           self->lane[lane].um_per_count_x10)));
    begin_move(self, lane, now_ms);
    return 0;
}

int ace2k_lane_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    return start_move(self, lane, dir, length_um, speed_um_s, now_ms, true);
}

int ace2k_lane_move_unlanded(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                             uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    return start_move(self, lane, dir, length_um, speed_um_s, now_ms, false);
}

static void stop_one(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms)
{
    if (self->lane[lane].move.active) {
        end_move(self, lane, ACE2K_LANE_STOPPED, now_ms);
    } else {
        drive_stop(self, lane); /* the feed's rule 5: the lines go to stop, whatever the state */
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
    /* The deadline follows the setpoint.  It is computed for what remains on the encoder, at the
     * new speed, from now.  Otherwise a move slowed to the floor would time out on the deadline of
     * its first speed. */
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
        um_per_count_x10 > ACE2K_LANE_SCALE_MAX_X10) {
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
