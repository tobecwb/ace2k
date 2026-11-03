/* What: the counters of each lane, and its motor.  The counters are the filament encoder and the
 * motor tach (FG) pulses.  The encoder is signed, in counts and in micrometres at the lane's scale.
 * The motor runs a bounded move at a held speed, stops at once, and reports the outcome of every
 * move.
 * How: the binding owns one struct ace2k_lane over ops.  The ops read the 16-bit encoder timers and
 * the FG counters of the board, and drive the PWM, run and direction lines.  Call ace2k_lane_tick()
 * every 10 ms, with link_ok.  The tick extends the encoder to 32 bits.  It runs the position stop
 * on the encoder, because the length of a move is the travel of the filament, not of the motor.  It
 * runs the stall and deadline checks.  Every ACE2K_LANE_LOOP_TICKS ticks of a move, it runs the
 * speed loop, which is a feed-forward table plus a PI on the tach rate.  The landing is the last
 * ACE2K_LANE_LANDING_UM of a move, at the floor speed.  It applies on the tick where it begins.
 * - ace2k_lane_move() starts a move, writing the ops in the order direction, run, PWM.
 * - ace2k_lane_move_unlanded() starts a move with no landing.
 * - ace2k_lane_set_speed() changes the setpoint and the deadline of a move.
 * - ace2k_lane_stop() ends a move on the spot.
 * - ace2k_lane_pop_result() hands the outcome over.
 * A counter reset waits for the lane to be idle.  The tick runs in interrupt context.  The binding
 * masks interrupts around every other call.
 * Constants: docs/hardware.md "Motors" and "Counters".  The table, the speed bounds and the gains
 * are measurements on the unit.  The values marked initial or provisional are still to be measured.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_LANE_H
#define ACE2K_LANE_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_LANE_UM_PER_COUNT_X10    12342U /* 1234.2 µm per count: the default scale */
#define ACE2K_LANE_SCALE_TOLERANCE_PCT 20U /* a scale outside ±20 % of the default is a host bug */
/* A scale is in tenths of a µm per count: divide by this to get µm. */
#define ACE2K_LANE_SCALE_TENTHS_PER_UM 10U
/* The widest scale the lane takes, +20 % of the default: 14810, so 1481.0 µm per count. */
#define ACE2K_LANE_SCALE_MAX_X10                                                                   \
    (ACE2K_LANE_UM_PER_COUNT_X10 * (ACE2K_PCT_FULL + ACE2K_LANE_SCALE_TOLERANCE_PCT) /             \
     ACE2K_PCT_FULL)
#define ACE2K_LANE_ALL                 255U
#define ACE2K_LANE_FG_PULSES_PER_100MM 1235U /* ≈ 12.35 tach pulses per mm */
#define ACE2K_LANE_UM_PER_100MM        100000U
/* The speed at the floor duty under a 1 kg spool.  The unit measured 8 502 µm/s, rounded up here to
 * the next 1 000.  Unloaded, the floor duty runs at about 15 mm/s. */
#define ACE2K_LANE_SPEED_MIN_UM_S 9000U
/* 90 % of the loaded maximum under a well-wound spool of about 950 g, rounded down to a multiple of
 * 5 000 (measured on the unit).  That spool ran 79 433 µm/s at 100 % duty, and 90 % of it is
 * 71 490.  A badly wound 1 kg spool ran 76 923 at 100 %, which leaves 9 % of headroom at this
 * limit.  A higher limit was preferred, as long as the overshoot stayed small.  Unloaded, 100 %
 * runs 91 173 (the table). */
#define ACE2K_LANE_SPEED_MAX_UM_S 70000U
#define ACE2K_LANE_MOVE_MAX_UM    2000000U /* the feed's rule 2: 2 000 mm per move */
/* Measured on the unit (docs/hardware.md).  At 10 %, the motor does not start, even unloaded.  At
 * 15 %, it carries a 1 kg spool at about 4.7 mm/s, but not steadily at the tach's resolution.  One
 * pulse per 100 ms loop is about 0.8 mm/s.  At 20 %, it carries the spool steadily, at
 * 9.7 mm/s, so 20 % is kept.  The feed-forward table starts at this duty (ACE2K_LANE_FF_FIRST_PCT,
 * asserted equal in lane.c).  A lower floor would lower both together, and the first entry of the
 * table would have to be measured there. */
#define ACE2K_LANE_DUTY_MIN_PCT 20U
#define ACE2K_LANE_DUTY_MAX_PCT 100U /* the duty ceiling, a limit that could be set below 100 % */
#define ACE2K_LANE_LOOP_TICKS   10U  /* the speed loop runs every 10 ticks: 100 ms */
#define ACE2K_LANE_LOOPS_PER_S  10U
/* The landing: the last 10 mm of a move, or the whole of a shorter one, runs at the floor speed.  A
 * move started by ace2k_lane_move_unlanded() has no landing.  With the landing, the overshoot past
 * the target measured on the unit stayed within 9 tach pulses, about 0.7 mm. */
#define ACE2K_LANE_LANDING_UM 10000U
/* The motor is stalled when run is high and the tach stays silent this long.  At the floor duty the
 * tach gives about 190 pulses a second (the table in lane.c), so a silent second is never just a
 * slow motor. */
#define ACE2K_LANE_STALL_MS 1000U
/* The deadline is 3/2 of the expected time, plus one second.  The expected time is the length less
 * the landing at the commanded speed, plus the landing at the floor speed. */
#define ACE2K_LANE_DEADLINE_NUM       3U
#define ACE2K_LANE_DEADLINE_DEN       2U
#define ACE2K_LANE_DEADLINE_MARGIN_MS 1000U
/* The gains.  The P term in duty % is error / KP_DIV, and the I term is sum / KI_DIV, both in µm/s.
 * Measured on the unit, with the initial straight line as feed-forward and the initial gains
 * (2000, 20000), the loaded loop settled 6–11 % under the setpoint after 2–4 s.  The measured
 * table removes most of the feed-forward error.  These gains answer the 15–20 % speed drop of the
 * load within about a second.  The gains 1500/6000 were then checked on the unit: the loop settled
 * 2–4 % under the setpoint. */
#define ACE2K_LANE_KP_DIV        1500
#define ACE2K_LANE_KI_DIV        6000
#define ACE2K_LANE_INTEG_MAX_PCT 50  /* bound of the integral term, ±50 % of duty; initial value */
#define ACE2K_LANE_FF_FIRST_PCT  20U /* the feed-forward table starts at the floor … */
#define ACE2K_LANE_FF_STEP_PCT   10U /* … in steps of 10 % … */
#define ACE2K_LANE_FF_POINTS     9U  /* … up to 100 % */

enum ace2k_lane_dir {
    ACE2K_LANE_FORWARD = 0, /* toward the printer */
    ACE2K_LANE_REVERSE = 1, /* toward the spool */
};

enum ace2k_lane_outcome {
    ACE2K_LANE_DONE = 0,    /* the filament travelled the length */
    ACE2K_LANE_STOPPED = 1, /* ace2k_lane_stop() */
    ACE2K_LANE_STALLED = 2, /* the tach silent for ACE2K_LANE_STALL_MS while running */
    /* The deadline passed with the filament short of the length.  The cause is no filament, a jam,
     * or a slip past the margin.  The motor may have run the whole length and more. */
    ACE2K_LANE_TIMEOUT = 3,
    ACE2K_LANE_LINK_LOST = 4, /* the tick saw link_ok false */
    ACE2K_LANE_SHUTDOWN = 5,  /* ace2k_lane_abort_all() from the binding's shutdown handler */
};

struct ace2k_lane_ops {
    uint16_t (*encoder_read)(void *ctx, uint8_t lane);
    uint32_t (*fg_read)(void *ctx, uint8_t lane);
    void (*pwm_set)(void *ctx, uint8_t lane, uint8_t duty_pct); /* 0 = stop; the board inverts */
    void (*run_set)(void *ctx, uint8_t lane, bool on);
    void (*dir_set)(void *ctx, uint8_t lane, bool reverse); /* false = toward the printer */
};

/* The outcome of one move, popped once. */
struct ace2k_lane_result {
    uint8_t outcome;     /* enum ace2k_lane_outcome */
    uint32_t motor_um;   /* travel of the tach since the start, unsigned */
    int32_t filament_um; /* encoder travel since the start, signed (+ toward the printer) */
    uint32_t duration_ms;
};

struct ace2k_lane_move {
    bool active;
    bool reverse;
    /* True when the landing's travel, or less, remains on the encoder, or did at the start.  The
     * floor setpoint is written on the tick where the landing begins. */
    bool landing;
    bool no_landing; /* started by ace2k_lane_move_unlanded(): never lands */
    bool measured;   /* a speed loop has measured the motor: the P term has an error to act on */
    uint32_t speed_um_s;    /* the setpoint */
    uint32_t target_counts; /* encoder counts to travel along the command */
    uint32_t fg_start;      /* the raw tach at the start, for motor_um and the speed loop */
    int32_t enc_start;      /* the encoder count at the start, for the position stop */
    uint32_t t_start_ms;
    uint32_t deadline_ms;
    uint32_t fg_seen; /* the raw tach when it last changed */
    uint32_t t_fg_seen_ms;
    uint32_t fg_loop;   /* the raw tach at the last speed loop */
    uint8_t loop_ticks; /* ticks since the last speed loop */
    int32_t integ_um_s; /* the summed error of the PI */
    uint32_t measured_um_s;
    uint32_t loop_setpoint_um_s; /* the setpoint duty_pct was computed for */
    uint8_t duty_pct;
};

struct ace2k_lane_counters {
    int32_t encoder_count;
    uint16_t encoder_last_raw;
    uint32_t fg_raw;
    uint32_t fg_base;
    volatile uint32_t fg; /* fg_raw − fg_base, published by the tick as one value */
    uint32_t um_per_count_x10;
    /* The tach pulses since boot, split by the direction last driven: a move's pulses, and those
     * that coast after its stop, go to its direction.  Pulses before any move go forward.  The
     * tick writes them, and so does the start of a move, which first counts the pulses since the
     * last tick in the direction before it.  A reset leaves them. */
    volatile uint32_t fg_fwd;
    volatile uint32_t fg_back;
    uint32_t fg_split; /* the tach reading up to which fg_fwd and fg_back are counted */
    bool last_reverse; /* the direction of the last move started */
    struct ace2k_lane_move move;
    struct ace2k_lane_result result;
    volatile bool result_pending; /* the tick sets it, the task clears it */
    uint32_t results_dropped;     /* a result overwritten before it was popped */
};

struct ace2k_lane {
    const struct ace2k_lane_ops *ops;
    void *ctx;
    struct ace2k_lane_counters lane[ACE2K_LANE_COUNT];
    volatile uint8_t reset_pending; /* bit per lane: set by ace2k_lane_reset, cleared by the tick */
    bool primed;
};

/* Clears every state and sets the default scale on every lane.  The ops are not written, because
 * the board leaves the lines at their stop levels before any init. */
void ace2k_lane_init(struct ace2k_lane *self, const struct ace2k_lane_ops *ops, void *ctx);

/* Runs in interrupt context.  It reads every lane, extends the encoder, applies pending resets and
 * runs the moves.  With link_ok false, it ends every active move with ACE2K_LANE_LINK_LOST (the
 * feed's rule 6). */
void ace2k_lane_tick(struct ace2k_lane *self, uint32_t now_ms, bool link_ok);

/* Returns the encoder count, or 0 for a lane out of range.  A base for
 * ace2k_lane_counts_since_um(). */
int32_t ace2k_lane_encoder_count(const struct ace2k_lane *self, uint8_t lane);

/* Returns the encoder's signed travel since a base count taken from ace2k_lane_encoder_count(), in
 * micrometres at the lane's scale as it is now, saturated to INT32_MIN..INT32_MAX.  A scale set
 * after the base still applies to the whole travel.  It returns 0 for a lane out of range. */
int32_t ace2k_lane_counts_since_um(const struct ace2k_lane *self, uint8_t lane,
                                   int32_t base_counts);

/* Returns the encoder count in micrometres at the lane's scale, saturated to
 * INT32_MIN..INT32_MAX. */
int32_t ace2k_lane_encoder_um(const struct ace2k_lane *self, uint8_t lane);

/* Returns the FG pulses since init or the last reset, in one load. */
uint32_t ace2k_lane_fg(const struct ace2k_lane *self, uint8_t lane);

/* Returns the FG pulse counter as the last tick read it, never re-based.  A reset moves the base of
 * ace2k_lane_fg(), not this counter.  So the difference of two readings is the pulses between them,
 * across a reset and across the 2^32 wrap.  It is for a client that keeps its own bases, as the
 * odometers of the feed do.  It returns 0 for a lane out of range. */
uint32_t ace2k_lane_fg_raw(const struct ace2k_lane *self, uint8_t lane);

/* Returns the motor travel since boot in micrometres, forward (toward the printer) or back, as the
 * tach counted it under the direction last driven.  The tach itself has no direction.  Pulses
 * that coast after a stop count in the direction of the move that stopped, and pulses before the
 * first move count forward.  Each value wraps at 2^32 µm (about 4.3 km); the difference of two
 * readings, modulo 2^32, is the travel between them, but for the one that spans the wrap of the
 * pulse count itself (2^32 pulses, about 348 km).  A counters reset leaves both.  Each returns
 * 0 for a lane out of range. */
uint32_t ace2k_lane_motor_fwd_um(const struct ace2k_lane *self, uint8_t lane);
uint32_t ace2k_lane_motor_back_um(const struct ace2k_lane *self, uint8_t lane);

/* Converts tach pulses to micrometres: pulses × 100000 / 1235, in 64 bits. */
static inline uint32_t ace2k_lane_fg_to_um(uint32_t pulses)
{
    return (uint32_t)(((uint64_t)pulses * ACE2K_LANE_UM_PER_100MM) /
                      ACE2K_LANE_FG_PULSES_PER_100MM);
}

/* Returns true for a speed that a move may be asked for, within [ACE2K_LANE_SPEED_MIN_UM_S,
 * ACE2K_LANE_SPEED_MAX_UM_S].  This is the one place where the bounds are read.  The moves, and the
 * bindings that check a speed before a move, ask here. */
static inline bool ace2k_lane_speed_in_bounds(uint32_t speed_um_s)
{
    if (speed_um_s < ACE2K_LANE_SPEED_MIN_UM_S) {
        return false;
    }
    return speed_um_s <= ACE2K_LANE_SPEED_MAX_UM_S;
}

/* Converts micrometres to encoder counts at the lane's scale, rounded to nearest.  100 mm is
 * 81 counts at the default scale.  It returns 0 for a lane out of range.  This is what the length
 * of a move becomes.  For the tests. */
uint32_t ace2k_lane_um_to_counts(const struct ace2k_lane *self, uint8_t lane, uint32_t um);

/* Zeroes the lane's encoder count and re-bases its FG on the next tick.  It does it at once when a
 * move starts or a feed mode is entered on the lane before that tick.  ACE2K_LANE_ALL resets all
 * four.  It returns -ACE2K_EINVAL for any other lane ≥ 4.  It returns -ACE2K_EREFUSED, with nothing
 * changed, while the lane moves, or for ACE2K_LANE_ALL while any lane moves.  The counters are the
 * move's odometers.  The caller masks interrupts. */
int ace2k_lane_reset(struct ace2k_lane *self, uint8_t lane);

/* Lands now a reset still pending for the lane.  The count is zeroed and the base of the tach
 * moves, as the tick would do at its next pass.  Every base taken after this call belongs to the
 * reset: the base of a move when it starts, and of a feed mode when it is entered.  It does nothing
 * for a lane out of range or with no reset pending.  The caller masks interrupts, or calls it from
 * the tick. */
void ace2k_lane_land_reset(struct ace2k_lane *self, uint8_t lane);

/* Starts a bounded move.  length_um is in 1..ACE2K_LANE_MOVE_MAX_UM, and speed_um_s is within
 * [ACE2K_LANE_SPEED_MIN_UM_S, ACE2K_LANE_SPEED_MAX_UM_S].  Otherwise, or for a lane out of range,
 * it returns -ACE2K_EINVAL.  It returns -ACE2K_EREFUSED while the lane moves.
 * The length is the travel of the filament as the encoder measures it, along the command's
 * direction.  So a host that asks for 100 mm gets 100 mm of filament, however much the drive gear
 * slips.  It slips a few per cent under a spool, as measured on the unit.  The target is the length
 * in counts at the lane's scale, rounded to nearest, and at least one.
 * The speed loop and the stall check stay on the tach.  The 1.2 mm per count of the encoder is too
 * coarse for a 100 ms loop, and the setpoint commands the motor.
 * The deadline is budgeted from the length and the speed.  The slip makes a move a few per cent
 * longer, inside the 3/2 margin.  A filament that does not move, because there is none or it is
 * jammed, never reaches the target.  It ends ACE2K_LANE_TIMEOUT at the deadline, unless a
 * supervisor above stops the lane first.
 * It writes the ops in the order direction, run, PWM.  The first duty comes from the feed-forward
 * table.  A move no longer than the landing takes the duty of the floor, and runs whole at the
 * floor setpoint.  The caller masks interrupts. */
int ace2k_lane_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms);

/* Starts the same move with no landing.  It runs at speed_um_s from its first write to its end,
 * however little remains.  The bounds, the results and the deadline are those of ace2k_lane_move().
 * The deadline still budgets the last ACE2K_LANE_LANDING_UM at the floor speed, so it is only more
 * generous.  It is for the reverse doses of the feed-forward.  The stop of a dose is not a target,
 * and the real travel on the encoder is booked against the debt.  At the floor, a dose could not
 * keep up with the retraction that it takes back.  The caller masks interrupts. */
int ace2k_lane_move_unlanded(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                             uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms);

/* Stops the lane on the spot, with run low and PWM at stop, whatever its state.  A move that was
 * active leaves an ACE2K_LANE_STOPPED result.  ACE2K_LANE_ALL stops every lane.  The caller masks
 * interrupts. */
void ace2k_lane_stop(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms);

/* Ends every active move with the given outcome (LINK_LOST, SHUTDOWN), with the ops stopped. */
void ace2k_lane_abort_all(struct ace2k_lane *self, enum ace2k_lane_outcome outcome,
                          uint32_t now_ms);

/* Sets a new setpoint for a running move, with the same bounds as move().  The new deadline is the
 * one for what remains on the encoder at the new speed, from now_ms.  It returns -ACE2K_EREFUSED
 * while idle.  The caller masks interrupts. */
int ace2k_lane_set_speed(struct ace2k_lane *self, uint8_t lane, uint32_t speed_um_s,
                         uint32_t now_ms);

bool ace2k_lane_is_moving(const struct ace2k_lane *self, uint8_t lane);
bool ace2k_lane_any_moving(const struct ace2k_lane *self);
/* Returns the running move's direction: true toward the spool.  It returns false while idle, or for
 * a lane out of range.  This is what "along the command" means to a supervisor that reads the
 * encoder. */
bool ace2k_lane_is_reverse(const struct ace2k_lane *self, uint8_t lane);

/* Hands over the last move's result, once.  It returns false when none is pending. */
bool ace2k_lane_pop_result(struct ace2k_lane *self, uint8_t lane, struct ace2k_lane_result *out);
/* Returns the measured speed of the last loop (0 while idle), and the duty in effect. */
uint32_t ace2k_lane_speed_um_s(const struct ace2k_lane *self, uint8_t lane);
uint8_t ace2k_lane_duty_pct(const struct ace2k_lane *self, uint8_t lane);

/* Returns the setpoint that the duty in effect was computed for, as the last write of the duty used
 * it.  That is the floor from the tick where the landing begins, or from the start for a move no
 * longer than the landing.  Before that, it is the commanded speed.  It returns 0 while idle, or
 * for a lane out of range.  The feed-forward duty is read for this setpoint. */
uint32_t ace2k_lane_setpoint_um_s(const struct ace2k_lane *self, uint8_t lane);

/* Sets the lane's encoder scale, in tenths of a micrometre per count.  It accepts a value within
 * ±20 % of the default (9874..14810).  Otherwise, or for a lane out of range, it returns
 * -ACE2K_EINVAL. */
int ace2k_lane_scale_set(struct ace2k_lane *self, uint8_t lane, uint32_t um_per_count_x10);
uint32_t ace2k_lane_scale(const struct ace2k_lane *self, uint8_t lane);

/* Returns the feed-forward duty for a setpoint, from the table, interpolated and clamped to the
 * duty bounds.  Exposed for the tests. */
uint8_t ace2k_lane_feed_forward_pct(uint32_t speed_um_s);

#endif
