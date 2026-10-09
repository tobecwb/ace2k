/* What: the per-lane counters — the filament encoder (signed, counts and micrometres at the
 * lane's scale) and the motor tach (FG) pulses — and the motor: a bounded move at a held speed,
 * an immediate stop, the outcome of every move.
 * How: the binding owns one struct ace2k_lane over ops that read the board's 16-bit encoder
 * timers and FG counters and drive the PWM, run and direction lines.  ace2k_lane_tick() every
 * 10 ms, with link_ok, extends the encoder to 32 bits, runs the position stop — on the encoder:
 * a move's length is the strand's travel, not the motor's — the stall and deadline checks and,
 * every ACE2K_LANE_LOOP_TICKS ticks of a move, the speed loop (a feed-forward table plus a PI on
 * the tach rate).  ace2k_lane_move() starts a move (the ops in the order direction, run, PWM),
 * ace2k_lane_set_speed() changes its setpoint and its deadline, ace2k_lane_stop() ends one on
 * the spot, ace2k_lane_pop_result() hands the outcome over; a counter reset waits for the lane
 * to be idle.  The tick runs in interrupt context; the binding brackets every other call with
 * interrupts masked.  Constants: docs/hardware.md "Motors" and "Counters"; the table, the speed
 * bounds and the gains are measurements on the unit; the values marked
 * initial or provisional are still to be measured.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_LANE_H
#define ACE2K_LANE_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_LANE_UM_PER_COUNT_X10    12342U /* 1234.2 µm per count: the default scale */
#define ACE2K_LANE_SCALE_TOLERANCE_PCT 20U /* a scale outside ±20 % of the default is a host bug */
#define ACE2K_LANE_ALL                 255U
#define ACE2K_LANE_FG_PULSES_PER_100MM 1235U /* ≈ 12.35 tach pulses per mm */
#define ACE2K_LANE_UM_PER_100MM        100000U
/* The speed at the floor duty under a 1 kg spool — 8 502 µm/s measured on the bench, rounded up
 * to the next 1 000 (measured on the unit); unloaded, the floor duty runs at about
 * 15 mm/s. */
#define ACE2K_LANE_SPEED_MIN_UM_S 9000U
/* 90 % of the loaded maximum under a well-wound ~950 g spool — 79 433 µm/s at 100 % duty, so
 * 71 490 — rounded down to a multiple of 5 000 (measured on the unit).  A badly wound 1 kg spool ran
 * 76 923 at 100 %, which leaves 9 % of headroom at this ceiling: higher was preferred, without
 * much overshoot.  Unloaded, 100 % runs 91 173 (the table). */
#define ACE2K_LANE_SPEED_MAX_UM_S 70000U
#define ACE2K_LANE_MOVE_MAX_UM    2000000U /* the feed's rule 2: 2 000 mm per move */
/* Measured 2026-09-27: 10 % does not start the motor even unloaded;
 * 15 % carries a 1 kg spool at about 4.7 mm/s, but not steadily at the tach's resolution (one
 * pulse per 100 ms loop is about 0.8 mm/s); 20 % carries it steadily (9.7 mm/s), so 20 % is kept.
 * The feed-forward table starts at this duty (ACE2K_LANE_FF_FIRST_PCT, asserted equal in lane.c):
 * a lower floor would lower both together, with the table's first entry measured there. */
#define ACE2K_LANE_DUTY_MIN_PCT 20U
#define ACE2K_LANE_DUTY_MAX_PCT 100U
#define ACE2K_LANE_LOOP_TICKS   10U /* the speed loop every 10 ticks: 100 ms */
#define ACE2K_LANE_LOOPS_PER_S  10U
#define ACE2K_LANE_LANDING_UM   10000U /* the last 10 mm at the floor speed */
#define ACE2K_LANE_STALL_MS     1000U  /* run high and the tach silent this long: stalled */
/* The deadline: 3/2 of the expected time — the length less the landing at the commanded speed,
 * the landing at the floor speed — plus one second. */
#define ACE2K_LANE_DEADLINE_NUM       3U
#define ACE2K_LANE_DEADLINE_DEN       2U
#define ACE2K_LANE_DEADLINE_MARGIN_MS 1000U
/* The gains: duty % per µm/s of error = error / KP_DIV, per summed µm/s = sum / KI_DIV.  On the bench,
 * with the initial line as feed-forward and the initial gains (2000, 20000) the loaded
 * loop settled 6–11 % under the setpoint after 2–4 s; the measured table removes most of the
 * feed-forward error and these gains answer the load's 15–20 % speed drop within about a
 * second.  Image A′ re-checks them. */
#define ACE2K_LANE_KP_DIV        1500
#define ACE2K_LANE_KI_DIV        6000
#define ACE2K_LANE_INTEG_MAX_PCT 50  /* the integral term saturates at ±50 % of duty */
#define ACE2K_LANE_FF_FIRST_PCT  20U /* the feed-forward table starts at the floor … */
#define ACE2K_LANE_FF_STEP_PCT   10U /* … in steps of 10 % … */
#define ACE2K_LANE_FF_POINTS     9U  /* … up to 100 % */
#define ACE2K_LANE_PCT_MAX       100U

enum ace2k_lane_dir {
    ACE2K_LANE_FORWARD = 0, /* toward the printer */
    ACE2K_LANE_REVERSE = 1, /* toward the spool */
};

enum ace2k_lane_outcome {
    ACE2K_LANE_DONE = 0,    /* the strand travelled the length */
    ACE2K_LANE_STOPPED = 1, /* ace2k_lane_stop() */
    ACE2K_LANE_STALLED = 2, /* the tach silent for ACE2K_LANE_STALL_MS while running */
    /* the deadline passed with the strand short of the length: no filament, a jam, or a slip
     * past the margin — the motor may have run the whole length and more */
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
    uint32_t motor_um;   /* the tach's travel since the start, unsigned */
    int32_t filament_um; /* the encoder's travel since the start, signed (+ toward the printer) */
    uint32_t duration_ms;
};

struct ace2k_lane_move {
    bool active;
    bool reverse;
    bool landing; /* the landing's travel or less remains on the encoder: the floor setpoint */
    uint32_t speed_um_s;    /* the setpoint */
    uint32_t target_counts; /* encoder counts to travel along the command */
    uint32_t fg_start;      /* the raw tach at the start: motor_um and the speed loop */
    int32_t enc_start;      /* the encoder count at the start: the position stop */
    uint32_t t_start_ms;
    uint32_t deadline_ms;
    uint32_t fg_seen; /* the raw tach when it last changed */
    uint32_t t_fg_seen_ms;
    uint32_t fg_loop;   /* the raw tach at the last speed loop */
    uint8_t loop_ticks; /* ticks since the last speed loop */
    int32_t integ_um_s; /* the PI's summed error */
    uint32_t measured_um_s;
    uint32_t loop_setpoint_um_s; /* the setpoint duty_pct was computed for */
    uint8_t duty_pct;
};

struct ace2k_lane_counters {
    int32_t encoder_count;
    uint16_t encoder_last_raw;
    uint32_t fg_raw;
    uint32_t fg_base;
    uint32_t fg; /* fg_raw − fg_base, published by the tick as one value */
    uint32_t um_per_count_x10;
    struct ace2k_lane_move move;
    struct ace2k_lane_result result;
    bool result_pending;
    uint32_t results_dropped; /* a result overwritten before it was popped */
};

struct ace2k_lane {
    const struct ace2k_lane_ops *ops;
    void *ctx;
    struct ace2k_lane_counters lane[ACE2K_LANE_COUNT];
    uint8_t reset_pending; /* bit per lane; written by ace2k_lane_reset, consumed by the tick */
    bool primed;
};

/* Every state cleared, the default scale on every lane; the ops are not written (the board
 * leaves the lines at their stop levels before any init). */
void ace2k_lane_init(struct ace2k_lane *self, const struct ace2k_lane_ops *ops, void *ctx);

/* Interrupt context: reads every lane, extends the encoder, applies pending resets, runs the
 * moves.  link_ok false ends every active move with ACE2K_LANE_LINK_LOST (the feed's rule 6). */
void ace2k_lane_tick(struct ace2k_lane *self, uint32_t now_ms, bool link_ok);

int32_t ace2k_lane_encoder_count(const struct ace2k_lane *self, uint8_t lane);

/* The encoder count in micrometres at the lane's scale, saturated to INT32_MIN..INT32_MAX. */
int32_t ace2k_lane_encoder_um(const struct ace2k_lane *self, uint8_t lane);

/* FG pulses since init or the last reset: one load. */
uint32_t ace2k_lane_fg(const struct ace2k_lane *self, uint8_t lane);

/* Tach pulses to micrometres: pulses × 100000 / 1235, in 64 bits. */
static inline uint32_t ace2k_lane_fg_to_um(uint32_t pulses)
{
    return (uint32_t)(((uint64_t)pulses * ACE2K_LANE_UM_PER_100MM) /
                      ACE2K_LANE_FG_PULSES_PER_100MM);
}

/* A speed a move may be asked for: within [ACE2K_LANE_SPEED_MIN_UM_S, ACE2K_LANE_SPEED_MAX_UM_S].
 * The one place the bounds are read — the moves and the bindings that check a speed before a
 * move ask here. */
static inline bool ace2k_lane_speed_in_bounds(uint32_t speed_um_s)
{
    if (speed_um_s < ACE2K_LANE_SPEED_MIN_UM_S) {
        return false;
    }
    return speed_um_s <= ACE2K_LANE_SPEED_MAX_UM_S;
}

/* Micrometres to encoder counts at the lane's scale, rounded to nearest (100 mm is 81 counts at
 * the default scale); 0 for a lane out of range.  What a move's length becomes. */
uint32_t ace2k_lane_um_to_counts(const struct ace2k_lane *self, uint8_t lane, uint32_t um);

/* Zeroes the lane's encoder count and re-bases its FG on the next tick, or at once when a move
 * starts or a feed mode is entered on the lane before that tick; ACE2K_LANE_ALL for all four.
 * -ACE2K_EINVAL for any other lane ≥ 4; -ACE2K_EREFUSED, nothing changed, while the lane moves
 * (for ACE2K_LANE_ALL: while any lane moves) — the counters are the move's odometers.  Interrupts
 * masked by the caller. */
int ace2k_lane_reset(struct ace2k_lane *self, uint8_t lane);

/* A reset still pending for the lane, landed now: the count zeroed and the tach's base moved, as
 * the tick would at its next pass.  Every base taken after this call is the reset's (a move's
 * when it starts, a feed mode's when it is entered).  Nothing for a lane out of range or with no
 * reset pending.  Interrupts masked by the caller, or from the tick. */
void ace2k_lane_land_reset(struct ace2k_lane *self, uint8_t lane);

/* Starts a bounded move: length_um in 1..ACE2K_LANE_MOVE_MAX_UM, speed_um_s within
 * [ACE2K_LANE_SPEED_MIN_UM_S, ACE2K_LANE_SPEED_MAX_UM_S] (-ACE2K_EINVAL otherwise, or for a lane
 * out of range); -ACE2K_EREFUSED while the lane moves.  The length is the strand's travel as the
 * encoder measures it, along the command's direction — a host that asks for 100 mm gets 100 mm
 * of filament, whatever the drive gear slips (a few per cent under a spool, measured on the unit); the target is the length in counts at the lane's scale, rounded to
 * nearest and at least one.  The speed loop and the stall check stay on the tach: the encoder's
 * 1.2 mm per count is too coarse for a 100 ms loop, and the motor is what the setpoint commands.
 * The deadline is budgeted from the length and the speed (the slip lengthens a move by a few per
 * cent, inside the 3/2 margin): a strand that does not move — no filament, a jam — never reaches
 * the target and ends ACE2K_LANE_TIMEOUT at the deadline, unless a supervisor above stops the
 * lane first.  Writes the ops in the order direction, run, PWM (the first duty from the
 * feed-forward table).  Interrupts masked by the caller. */
int ace2k_lane_move(struct ace2k_lane *self, uint8_t lane, enum ace2k_lane_dir dir,
                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms);

/* Stops the lane on the spot — run low, PWM at stop — whatever its state; a move that was
 * active leaves an ACE2K_LANE_STOPPED result.  ACE2K_LANE_ALL for every lane.  Interrupts masked
 * by the caller. */
void ace2k_lane_stop(struct ace2k_lane *self, uint8_t lane, uint32_t now_ms);

/* Ends every active move with the given outcome (LINK_LOST, SHUTDOWN), the ops stopped. */
void ace2k_lane_abort_all(struct ace2k_lane *self, enum ace2k_lane_outcome outcome,
                          uint32_t now_ms);

/* A new setpoint for a running move, same bounds as move(); the deadline becomes the one of
 * what remains on the encoder at the new speed, from now_ms.  -ACE2K_EREFUSED while idle.
 * Interrupts masked by the caller. */
int ace2k_lane_set_speed(struct ace2k_lane *self, uint8_t lane, uint32_t speed_um_s,
                         uint32_t now_ms);

bool ace2k_lane_is_moving(const struct ace2k_lane *self, uint8_t lane);
bool ace2k_lane_any_moving(const struct ace2k_lane *self);
/* The running move's direction — true toward the spool; false while idle or for a lane out of
 * range.  What "along the command" means to a supervisor reading the encoder. */
bool ace2k_lane_is_reverse(const struct ace2k_lane *self, uint8_t lane);

/* The last move's result, once; false when none is pending. */
bool ace2k_lane_pop_result(struct ace2k_lane *self, uint8_t lane, struct ace2k_lane_result *out);
/* The measured speed of the last loop (0 while idle) and the duty in effect. */
uint32_t ace2k_lane_speed_um_s(const struct ace2k_lane *self, uint8_t lane);
uint8_t ace2k_lane_duty_pct(const struct ace2k_lane *self, uint8_t lane);

/* The setpoint the duty in effect was computed for: the one the last speed loop used (the floor
 * once a loop ran in the landing, the commanded speed before), the move's speed until the first
 * loop; 0 while idle or for a lane out of range — what the feed-forward duty is read for. */
uint32_t ace2k_lane_setpoint_um_s(const struct ace2k_lane *self, uint8_t lane);

/* The lane's encoder scale in tenths of a micrometre per count; accepted within ±20 % of the
 * default (9874..14810), -ACE2K_EINVAL otherwise or for a lane out of range. */
int ace2k_lane_scale_set(struct ace2k_lane *self, uint8_t lane, uint32_t um_per_count_x10);
uint32_t ace2k_lane_scale(const struct ace2k_lane *self, uint8_t lane);

/* The feed-forward duty for a setpoint, from the table (interpolated, clamped to the duty
 * bounds).  Exposed for the tests. */
uint8_t ace2k_lane_feed_forward_pct(uint32_t speed_um_s);

#endif
