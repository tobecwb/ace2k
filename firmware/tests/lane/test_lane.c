#include "test.h"
#include "lane/lane.h"

struct ace2k_fake_counters {
    uint16_t enc[ACE2K_LANE_COUNT];
    uint32_t fg[ACE2K_LANE_COUNT];
    uint8_t duty[ACE2K_LANE_COUNT];
    bool run[ACE2K_LANE_COUNT];
    bool reverse[ACE2K_LANE_COUNT];
    char order[32]; /* 'd' dir, 'r' run, 'p' pwm, in call order; NUL-terminated */
    int calls;
};

static uint16_t fake_enc(void *ctx, uint8_t lane)
{
    return ((struct ace2k_fake_counters *)ctx)->enc[lane];
}

static uint32_t fake_fg(void *ctx, uint8_t lane)
{
    return ((struct ace2k_fake_counters *)ctx)->fg[lane];
}

static void note(struct ace2k_fake_counters *f, char what)
{
    if (f->calls < (int)sizeof f->order - 1) {
        f->order[f->calls++] = what;
        f->order[f->calls] = '\0';
    }
}

static void fake_pwm(void *ctx, uint8_t lane, uint8_t duty_pct)
{
    struct ace2k_fake_counters *f = ctx;
    f->duty[lane] = duty_pct;
    note(f, 'p');
}

static void fake_run(void *ctx, uint8_t lane, bool on)
{
    struct ace2k_fake_counters *f = ctx;
    f->run[lane] = on;
    note(f, 'r');
}

static void fake_dir(void *ctx, uint8_t lane, bool reverse)
{
    struct ace2k_fake_counters *f = ctx;
    f->reverse[lane] = reverse;
    note(f, 'd');
}

static const struct ace2k_lane_ops ace2k_fake_ops = {
    .encoder_read = fake_enc,
    .fg_read = fake_fg,
    .pwm_set = fake_pwm,
    .run_set = fake_run,
    .dir_set = fake_dir,
};

/* A primed instance at t = 10 ms, every counter at 0. */
static void primed(struct ace2k_lane *l, struct ace2k_fake_counters *f)
{
    *f = (struct ace2k_fake_counters){ 0 };
    ace2k_lane_init(l, &ace2k_fake_ops, f);
    ace2k_lane_tick(l, 10, true);
}

/* Advance the tach by pulses_per_tick for n ticks of 10 ms from *now, encoder along at
 * counts_per_tick (may be 0). */
static void run_ticks(struct ace2k_lane *l, struct ace2k_fake_counters *f, uint32_t *now, int n,
                      uint32_t pulses_per_tick, int counts_per_tick)
{
    for (int i = 0; i < n; i++) {
        *now += 10;
        f->fg[0] += pulses_per_tick;
        f->enc[0] = (uint16_t)(f->enc[0] + counts_per_tick);
        ace2k_lane_tick(l, *now, true);
    }
}

/* The same for n ticks with the encoder advancing count_step (+1 along a forward move, −1 along
 * a reverse one) once every `every` ticks — on the every-th, 2·every-th … tick of this call, so
 * n / every counts land in all.  A move ends on the encoder, so this is what finishes one: the
 * motor at 4 pulses a tick is ≈ 32 mm/s, a count every four ticks ≈ 31 mm/s of strand — the
 * few per cent the drive gear slips under a spool (measured on the unit). */
static void run_paced(struct ace2k_lane *l, struct ace2k_fake_counters *f, uint32_t *now, int n,
                      uint32_t pulses_per_tick, int every, int count_step)
{
    for (int i = 0; i < n; i++) {
        run_ticks(l, f, now, 1, pulses_per_tick, ((i + 1) % every == 0) ? count_step : 0);
    }
}

TEST(encoder_extends_across_the_sixteen_bit_wrap_in_both_directions)
{
    struct ace2k_fake_counters f = { .enc = { 65530, 5, 100, 0 }, .fg = { 0 } };
    struct ace2k_lane l;
    ace2k_lane_init(&l, &ace2k_fake_ops, &f);
    ace2k_lane_tick(&l, 10, true); /* primes: no delta */
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 0), 0);
    f.enc[0] = 5;
    f.enc[1] = 65530;
    f.enc[2] = 181;
    ace2k_lane_tick(&l, 20, true);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 0), 11);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), -11);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 2), 81);
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 2), 99970);
    f.enc[2] = 19; /* back 162 counts: −81 net */
    ace2k_lane_tick(&l, 30, true);
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 2), -99970);
}

TEST(fg_counts_from_the_base_taken_at_init_or_reset)
{
    struct ace2k_fake_counters f = { .enc = { 0 }, .fg = { 1000, 0, 0, 0 } };
    struct ace2k_lane l;
    ace2k_lane_init(&l, &ace2k_fake_ops, &f);
    ace2k_lane_tick(&l, 10, true);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 0);
    f.fg[0] = 1250;
    ace2k_lane_tick(&l, 20, true);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 250);
    ASSERT_EQ(ace2k_lane_reset(&l, 0), 0);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 250); /* not before the tick */
    ace2k_lane_tick(&l, 30, true);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 0);
    f.fg[0] = 1260;
    ace2k_lane_tick(&l, 40, true);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 10);
}

TEST(encoder_um_saturates_instead_of_wrapping)
{
    struct ace2k_fake_counters f = { .enc = { 0 }, .fg = { 0 } };
    struct ace2k_lane l;
    ace2k_lane_init(&l, &ace2k_fake_ops, &f);
    ace2k_lane_tick(&l, 10, true);
    l.lane[0].encoder_count = 1800000; /* × 1234.2 µm = 2.22e9 > INT32_MAX */
    l.lane[1].encoder_count = -1800000;
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 0), INT32_MAX);
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 1), INT32_MIN);
    l.lane[0].encoder_count = 1000;
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 0), 1234200);
}

TEST(fg_is_one_published_value_equal_to_raw_minus_base_after_a_reset_tick)
{
    struct ace2k_fake_counters f = { .enc = { 0 }, .fg = { 500, 0, 0, 0 } };
    struct ace2k_lane l;
    ace2k_lane_init(&l, &ace2k_fake_ops, &f);
    ace2k_lane_tick(&l, 10, true);
    f.fg[0] = 900;
    ace2k_lane_tick(&l, 20, true);
    ASSERT_EQ(l.lane[0].fg, 400);
    ASSERT_EQ(ace2k_lane_reset(&l, 0), 0);
    f.fg[0] = 950;
    ace2k_lane_tick(&l, 30, true); /* the reset tick: the base moves to 950 and fg follows it */
    ASSERT_EQ(l.lane[0].fg_base, 950);
    ASSERT_EQ(l.lane[0].fg, l.lane[0].fg_raw - l.lane[0].fg_base);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 0);
    f.fg[0] = 975;
    ace2k_lane_tick(&l, 40, true);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 25);
}

TEST(reset_all_and_refusals_and_once_only)
{
    struct ace2k_fake_counters f = { .enc = { 10, 20, 30, 40 }, .fg = { 0 } };
    struct ace2k_lane l;
    ace2k_lane_init(&l, &ace2k_fake_ops, &f);
    ace2k_lane_tick(&l, 10, true);
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        f.enc[i] += 7;
    }
    ace2k_lane_tick(&l, 20, true);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 3), 7);
    ASSERT_EQ(ace2k_lane_reset(&l, 4), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_reset(&l, ACE2K_LANE_ALL), 0);
    ASSERT_EQ(ace2k_lane_reset(&l, 1), 0); /* twice before a tick: applied once */
    ace2k_lane_tick(&l, 30, true);
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ASSERT_EQ(ace2k_lane_encoder_count(&l, i), 0);
    }
    f.enc[1] += 3;
    ace2k_lane_tick(&l, 40, true);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), 3); /* the pending mask was consumed */
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 4), 0); /* out of range reads 0 */
}

TEST(move_refuses_out_of_bounds_and_a_busy_lane)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    ASSERT_EQ(ace2k_lane_move(&l, 4, ACE2K_LANE_FORWARD, 100000, 30000, 10), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 0, 30000, 10), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, ACE2K_LANE_MOVE_MAX_UM + 1, 30000, 10),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MIN_UM_S - 1, 10),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MAX_UM_S + 1, 10),
              -ACE2K_EINVAL);
    ASSERT_EQ(f.calls, 0); /* a refused move writes no op */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, 30000, 10), 0);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_REVERSE, 100000, 30000, 10), -ACE2K_EREFUSED);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(ace2k_lane_any_moving(&l));
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 1));
}

TEST(the_running_moves_direction_is_the_lanes_to_tell_and_idle_is_not_reverse)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_TRUE(!ace2k_lane_is_reverse(&l, 0)); /* idle */
    ASSERT_TRUE(!ace2k_lane_is_reverse(&l, 4)); /* out of range */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_REVERSE, 100000, 30000, now), 0);
    ASSERT_TRUE(ace2k_lane_is_reverse(&l, 0));
    ASSERT_TRUE(!ace2k_lane_is_reverse(&l, 1)); /* another lane, idle */
    ASSERT_EQ(ace2k_lane_move(&l, 1, ACE2K_LANE_FORWARD, 100000, 30000, now), 0);
    ASSERT_TRUE(!ace2k_lane_is_reverse(&l, 1));
    ace2k_lane_stop(&l, 0, now);
    ASSERT_TRUE(!ace2k_lane_is_reverse(&l, 0)); /* the move over, the answer is idle's */
    ace2k_lane_stop(&l, ACE2K_LANE_ALL, now);
}

TEST(move_writes_direction_then_run_then_pwm_inside_the_duty_bounds)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    ASSERT_EQ(ace2k_lane_move(&l, 2, ACE2K_LANE_REVERSE, 100000, 30000, 10), 0);
    ASSERT_STR_EQ(f.order, "drp");
    ASSERT_TRUE(f.reverse[2]);
    ASSERT_TRUE(f.run[2]);
    ASSERT_TRUE(f.duty[2] >= ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_TRUE(f.duty[2] <= ACE2K_LANE_DUTY_MAX_PCT);
    ASSERT_EQ(f.duty[2], ace2k_lane_feed_forward_pct(30000));
}

TEST(feed_forward_interpolates_the_table_and_clamps)
{
    /* the table measured on the bench: 15 384, 24 858, 34 251, 43 562, 52 550, 61 700, 71 255,
     * 80 323, 91 173 µm/s at 20, 30 … 100 % */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(15384), 20); /* the first entry: the floor duty */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(1000), 20);
    ASSERT_EQ(ace2k_lane_feed_forward_pct(91173), 100);  /* the last entry */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(100000), 100); /* past the table: the ceiling */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(200000), 100);
    ASSERT_EQ(ace2k_lane_feed_forward_pct(52550), 60); /* the fifth entry exactly */
    /* 57 500 lies between 52 550 (60 %) and 61 700 (70 %): 60 + 4950 × 10 / 9150 = 65 */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(57500), 65);
    /* 20 000 between 15 384 (20 %) and 24 858 (30 %): 20 + 4616 × 10 / 9474 = 24 */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(20000), 24);
    /* 70 000, the ceiling, between 61 700 (70 %) and 71 255 (80 %): 70 + 8300 × 10 / 9555 = 78 */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(ACE2K_LANE_SPEED_MAX_UM_S), 78);
    /* the floor setpoint, 9 000, lies under the first entry (15 384), so the table gives the
     * floor duty — the anti-windup's lower bound is 0 at the floor setpoint (lane.c) only while
     * it does; a probe image that lowers the floor duty moves SPEED_MIN together with the table's
     * first entry, measured at the new floor */
    ASSERT_EQ(ace2k_lane_feed_forward_pct(ACE2K_LANE_SPEED_MIN_UM_S), ACE2K_LANE_DUTY_MIN_PCT);
}

TEST(a_move_ends_done_at_the_target_with_the_lines_at_stop_and_a_result)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 100 mm at 30 mm/s is 81 counts of 1234.2 µm (81.02, rounded); the motor at 4 pulses a tick
     * (≈ 32 mm/s, 370 pulses/s is 30), the strand a count every four ticks (≈ 31 mm/s: the slip
     * of a spool) — done on the 81st count, the 324th tick */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, 30000, now), 0);
    ASSERT_EQ(l.lane[0].move.target_counts, 81);
    run_paced(&l, &f, &now, 320, 4, 4, 1); /* 80 counts: one remains — moving, landing */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(l.lane[0].move.landing);
    ASSERT_TRUE(f.run[0]);
    run_ticks(&l, &f, &now, 3, 4, 0); /* 1292 pulses, 104.6 mm of motor: the tach ends nothing */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 4, 1); /* the 81st count */
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(!f.run[0]);
    ASSERT_EQ(f.duty[0], 0);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(1296)); /* 104.9 mm of motor for 100 of strand */
    ASSERT_EQ(r.filament_um, 99970);                  /* 81 × 1234.2 */
    ASSERT_EQ(r.duration_ms, 3240);
    ASSERT_TRUE(!ace2k_lane_pop_result(&l, 0, &r)); /* popped once */
}

TEST(the_landing_drops_the_setpoint_to_the_floor_for_the_last_ten_millimetres)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* the ceiling, 70 mm/s, is 865 pulses/s; the fake runs 8 per tick (≈ 65 mm/s) with a count
     * every two ticks (≈ 62 mm/s: the strand 5 % behind).  100 mm is 81 counts, and the landing
     * is decided on the encoder: it begins once 8 counts (9 874 µm ≤ 10 mm) remain — on the
     * 73rd count, tick 146 */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MAX_UM_S, now),
              0);
    ASSERT_TRUE(!l.lane[0].move.landing);
    run_paced(&l, &f, &now, 145, 8, 2, 1); /* 72 counts: 9 remain, 11 107 µm > 10 mm */
    ASSERT_TRUE(!l.lane[0].move.landing);
    run_ticks(&l, &f, &now, 1, 8, 1); /* the 73rd: 8 remain */
    ASSERT_TRUE(l.lane[0].move.landing);
    /* tick 150: the loop runs with the floor setpoint against 64 777 µm/s measured — the
     * error's P term alone (−37 %) puts the duty at the floor */
    run_paced(&l, &f, &now, 4, 8, 2, 1);
    ASSERT_TRUE(f.duty[0] < ace2k_lane_feed_forward_pct(ACE2K_LANE_SPEED_MAX_UM_S));
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
}

TEST(the_result_carries_the_filament_travel_at_the_lane_scale_and_its_sign)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_scale_set(&l, 0, 12000), 0);
    /* 10 mm at 1200.0 µm a count is 8 counts (8.33, rounded): 9.6 mm of strand */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_REVERSE, 10000, 30000, now), 0);
    ASSERT_EQ(l.lane[0].move.target_counts, 8);
    run_paced(&l, &f, &now, 32, 4, 4, -1); /* the encoder counts down: −8 on the 32nd tick, done */
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.filament_um, -9600); /* −8 × 1200.0 µm */
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(128));
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 0), -9600);
}

TEST(stop_always_stops_and_reports_stopped_only_when_a_move_was_active)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ace2k_lane_stop(&l, 1, now); /* idle: the lines written at stop, no result */
    ASSERT_TRUE(!f.run[1]);
    ASSERT_EQ(f.duty[1], 0);
    struct ace2k_lane_result r;
    ASSERT_TRUE(!ace2k_lane_pop_result(&l, 1, &r));
    ASSERT_EQ(ace2k_lane_move(&l, 1, ACE2K_LANE_FORWARD, 500000, 50000, now), 0);
    ASSERT_TRUE(f.run[1]);
    ace2k_lane_stop(&l, 1, now + 250);
    ASSERT_TRUE(!f.run[1]);
    ASSERT_EQ(f.duty[1], 0);
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 1));
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 1, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_STOPPED);
    ASSERT_EQ(r.duration_ms, 250);
    ace2k_lane_stop(&l, 1, now + 260); /* a second stop: nothing new */
    ASSERT_TRUE(!ace2k_lane_pop_result(&l, 1, &r));
    /* every lane at once */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 50000, now), 0);
    ASSERT_EQ(ace2k_lane_move(&l, 3, ACE2K_LANE_FORWARD, 500000, 50000, now), 0);
    ace2k_lane_stop(&l, ACE2K_LANE_ALL, now + 300);
    ASSERT_TRUE(!ace2k_lane_any_moving(&l));
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 3, &r));
}

TEST(a_silent_tach_for_one_second_is_a_stall)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 30000, now), 0);
    run_ticks(&l, &f, &now, 5, 4, 0);  /* moving */
    run_ticks(&l, &f, &now, 99, 0, 0); /* 990 ms silent: not yet */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 0, 0); /* 1000 ms */
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(!f.run[0]);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_STALLED);
}

TEST(a_move_that_outlives_its_deadline_times_out)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 100 mm at the ceiling, 70 mm/s: 90 mm at the ceiling, 90 000 × 1000 / 70 000 = 1285 ms,
     * and the 10 mm landing at the floor speed, 10 000 × 1000 / 9000 = 1111 ms — expected 2396,
     * the deadline 2396 × 3 / 2 + 1000 = 4594 ms after the start, i.e. the tick at 4600 ms (the
     * 460th); the tach creeps (1 pulse per tick keeps the stall check quiet) and the strand
     * never moves: 0 of 81 counts */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MAX_UM_S, now),
              0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 4594);
    run_ticks(&l, &f, &now, 459, 1, 0);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 1, 0);
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 0));
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_TIMEOUT);
}

TEST(link_loss_and_shutdown_end_every_active_move_with_the_lines_at_stop)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 30000, now), 0);
    ASSERT_EQ(ace2k_lane_move(&l, 2, ACE2K_LANE_REVERSE, 500000, 30000, now), 0);
    ace2k_lane_tick(&l, now + 10, false);
    ASSERT_TRUE(!ace2k_lane_any_moving(&l));
    ASSERT_TRUE(!f.run[0] && !f.run[2]);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_LINK_LOST);
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 2, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_LINK_LOST);
    ASSERT_TRUE(!ace2k_lane_pop_result(&l, 1, &r)); /* an idle lane has no result */
    ASSERT_EQ(ace2k_lane_move(&l, 1, ACE2K_LANE_FORWARD, 500000, 30000, now + 20), 0);
    ace2k_lane_abort_all(&l, ACE2K_LANE_SHUTDOWN, now + 30);
    ASSERT_TRUE(!f.run[1]);
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 1, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_SHUTDOWN);
}

TEST(the_speed_loop_raises_the_duty_below_the_setpoint_and_lowers_it_above_within_bounds)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 2000000, 50000, now), 0);
    uint8_t first = f.duty[0];
    ASSERT_EQ(first, 57);              /* the table: 50 + (50 000 − 43 562) × 10 / 8988 */
    run_ticks(&l, &f, &now, 10, 2, 0); /* 200 pulses/s ≈ 16 mm/s, far below 50 */
    ASSERT_TRUE(f.duty[0] > first);
    ASSERT_EQ(ace2k_lane_speed_um_s(&l, 0), ace2k_lane_fg_to_um(200));
    for (int i = 0; i < 30; i++) {
        run_ticks(&l, &f, &now, 10, 2, 0); /* still slow: the integral reaches the ceiling */
    }
    /* Each loop's error is 50 000 − 16 194 = 33 806 µm/s, P = 22 (33 806 / 1500) on the table's
     * 57, and the duty 79 + integral / 6000: 84, 90, 95 after one to three loops.  The fourth
     * integrates to 135 224 and applies 101, clamped to the ceiling; from the fifth the applied
     * duty, 79 + 22 = 101, is past the ceiling with the error positive, so the error stays out:
     * the integral is frozen at 4 × 33 806 = 135 224 — one loop's error past the value that
     * lands on the ceiling, well inside the window's (100 − 57) × 6000 = 258 000. */
    ASSERT_EQ(l.lane[0].move.integ_um_s, 4 * 33806);
    ASSERT_TRUE(l.lane[0].move.integ_um_s <= (100 - 57) * ACE2K_LANE_KI_DIV);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MAX_PCT);
    /* 1500 pulses/s is 121 457 µm/s: the error −71 457, P −47.  The first fast loop applies
     * 57 − 47 + 22 = 32 and integrates (63 767, the duty 20); the second applies 10 + 10 = 20 —
     * at the floor, not past it — and integrates again (−7 690, the duty 20); the third applies
     * 10 − 1 = 9, past the floor with the error negative, and the integral freezes there,
     * inside the window's (20 − 57) × 6000 = −222 000: unwound across the floor, the duty
     * sitting on it. */
    for (int i = 0; i < 40; i++) {
        run_ticks(&l, &f, &now, 10, 15, 0); /* 1500 pulses/s ≈ 121 mm/s, above 50 */
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, 135224 - (2 * 71457));
    ASSERT_TRUE(l.lane[0].move.integ_um_s >= (20 - 57) * ACE2K_LANE_KI_DIV);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_EQ(ace2k_lane_duty_pct(&l, 0), f.duty[0]);
    /* the error flips (slow again): the applied duty 57 + 22 − 1 = 78 is inside the bounds, so
     * the integral moves on this first loop — −7 690 + 33 806 = 26 116, worth 4 — and the duty
     * is 57 + 22 + 4 = 83: the load answered at once */
    run_ticks(&l, &f, &now, 10, 2, 0);
    ASSERT_EQ(l.lane[0].move.integ_um_s, -7690 + 33806);
    ASSERT_EQ(f.duty[0], 83);
}

TEST(a_load_arriving_after_an_over_speed_episode_is_answered_on_the_first_loop)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 50 mm/s asked (the table: 57 %) with the roller driven at 15 pulses a tick, 121 457 µm/s:
     * the error −71 457, P −47, so the applied duty 57 − 47 + 0 = 10 is past the floor with the
     * error negative from the first loop — the integral never moves from 0 (a window clamp alone
     * would have let it wind down to (20 − 57) × 6000 = −222 000) and the duty sits on the floor */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 2000000, 50000, now), 0);
    ASSERT_EQ(f.duty[0], 57);
    for (int i = 0; i < 6; i++) {
        run_ticks(&l, &f, &now, 10, 15, 0);
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, 0);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    /* the load arrives: 2 pulses a tick is 16 194 µm/s, the error +33 806, P +22.  Nothing to
     * unwind: this very loop applies 57 + 22 + 33 806 / 6000 = 84 (the window-only form would
     * have given 57 + 22 − 37 = 42 and reached 79 about seven loops on) */
    run_ticks(&l, &f, &now, 10, 2, 0);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 33806);
    ASSERT_EQ(f.duty[0], 84);
    ASSERT_TRUE(f.duty[0] >= 78);
}

TEST(a_setpoint_increase_drops_the_integral_to_what_the_new_table_entry_can_absorb)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 30 mm/s asked (the table: 35 %) with the tach at 1 pulse a tick, 8 097 µm/s: the error
     * 21 903, P 14, the applied duty 49 + integral / 6000 — never past the ceiling (99 at most),
     * so the integral climbs 21 903 a loop to its ±INTEG_MAX clamp, 300 000, on the 14th loop
     * (14 × 21 903 = 306 642 clamped); the window at 35 %, (100 − 35) × 6000 = 390 000, is wider */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 2000000, 30000, now), 0);
    ASSERT_EQ(f.duty[0], 35);
    for (int i = 0; i < 15; i++) {
        run_ticks(&l, &f, &now, 10, 1, 0);
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, ACE2K_LANE_INTEG_MAX_PCT * ACE2K_LANE_KI_DIV);
    ASSERT_EQ(f.duty[0], 99);
    /* the ceiling asked: the table's 78 can absorb (100 − 78) × 6000 = 132 000 of integral, no
     * more.  The next loop's error is 61 903 (P 41), the applied duty 78 + 41 + 50 = 169 is past
     * the ceiling so the error stays out, and the window drops the integral from 300 000 to
     * 132 000 at once — instead of unwinding 168 000 over many loops */
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, ACE2K_LANE_SPEED_MAX_UM_S, now), 0);
    run_ticks(&l, &f, &now, 10, 1, 0);
    ASSERT_EQ(l.lane[0].move.integ_um_s, (100 - 78) * ACE2K_LANE_KI_DIV);
    ASSERT_TRUE(l.lane[0].move.integ_um_s <=
                (100 - ace2k_lane_feed_forward_pct(ACE2K_LANE_SPEED_MAX_UM_S)) * ACE2K_LANE_KI_DIV);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MAX_PCT);
}

TEST(set_speed_changes_a_running_move_and_is_refused_idle_or_out_of_bounds)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, 30000, 10), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 30000, 10), 0);
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, ACE2K_LANE_SPEED_MAX_UM_S + 1, 10), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, 60000, 10), 0);
    ASSERT_EQ(l.lane[0].move.speed_um_s, 60000);
}

TEST(set_speed_moves_the_deadline_to_what_remains_at_the_new_speed)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 500 mm at the ceiling, 70 mm/s: 490 mm at the ceiling, 7000 ms, and the 10 mm landing at
     * the floor speed, 1111 ms — expected 8111, the deadline 8111 × 3 / 2 + 1000 = 13 166 ms
     * after the start; 500 mm is 405 counts */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, ACE2K_LANE_SPEED_MAX_UM_S, now),
              0);
    uint32_t old_deadline = l.lane[0].move.deadline_ms;
    ASSERT_EQ(old_deadline, now + 13166);
    run_paced(&l, &f, &now, 100, 8, 2, 1); /* one second: 50 of 405 counts (≈ 62 mm), 800 pulses */
    /* slowed to the floor, 9 mm/s: 355 counts = 438 141 µm remain on the encoder — 428 141 at
     * 9 mm/s is 47 571 ms and the 10 mm landing, at the same speed, 1111 — expected 48 682, so
     * the deadline is now + 48682 × 3 / 2 + 1000 = now + 74 023 */
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, ACE2K_LANE_SPEED_MIN_UM_S, now), 0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 74023);
    /* ≈ 16 mm/s of motor (2 pulses a tick), the strand a count every eight ticks (≈ 15 mm/s):
     * the move runs on past the old deadline and ends done, not timed out */
    run_paced(&l, &f, &now, 2832, 2, 8, 1); /* 354 counts: 404 of 405, one short */
    ASSERT_TRUE(ace2k_time_after(now, old_deadline));
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 2, 1); /* the 405th */
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
}

TEST(set_speed_inside_the_landing_keeps_the_floor_setpoint_and_rebudgets_the_remainder)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 100 mm at 32 388 µm/s — exactly 4 pulses per tick, so the cruise's 29 loops see no error
     * and leave the integral at 0; 81 counts in all, a count every four ticks, and the landing
     * begins on the 73rd (8 remain: 9 874 µm ≤ 10 mm), tick 292 */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, 32388, now), 0);
    run_paced(&l, &f, &now, 291, 4, 4, 1); /* 72 counts: 9 remain, 11 107 µm > 10 mm */
    ASSERT_TRUE(!l.lane[0].move.landing);
    run_ticks(&l, &f, &now, 1, 4, 1); /* the 73rd: 8 remain */
    ASSERT_TRUE(l.lane[0].move.landing);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 0);
    /* the ceiling asked inside the landing: stored, the landing kept, and the deadline is the
     * remainder's — 8 counts = 9 873 µm, all of it landing, so all of it at the floor speed:
     * 9 873 × 1000 / 9000 = 1097 → 1097 × 3 / 2 = 1645 + 1000 = 2645 from now; the new speed
     * does not enter */
    ASSERT_EQ(ace2k_lane_set_speed(&l, 0, ACE2K_LANE_SPEED_MAX_UM_S, now), 0);
    ASSERT_EQ(l.lane[0].move.speed_um_s, ACE2K_LANE_SPEED_MAX_UM_S);
    ASSERT_TRUE(l.lane[0].move.landing);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 2645);
    /* the 30th loop, eight ticks on, still measures 32 388 against the floor setpoint of 9 000:
     * the error −23 388, P −15 on the table's 20: the applied duty 5 is past the floor, the error
     * stays out, the integral is still 0 and the duty on the floor.  Against the new speed as
     * setpoint the error would be +37 612, P +25 on 78, and the duty at the ceiling. */
    run_paced(&l, &f, &now, 8, 4, 4, 1); /* two more counts: 75 of 81 */
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 0);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
}

TEST(a_reset_pending_when_a_move_starts_lands_before_the_move_takes_its_bases)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    f.enc[1] = 77;                        /* lane 1 has a count of its own, taken in below */
    run_ticks(&l, &f, &now, 10, 50, 100); /* 500 pulses, 1000 counts on lane 0 before any move */
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 0), 1000);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), 77);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 500);
    ASSERT_EQ(ace2k_lane_reset(&l, ACE2K_LANE_ALL), 0);
    /* the move starts before the tick that would apply the reset: it lands at the start */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 10000, 30000, now), 0); /* 8 counts */
    ASSERT_EQ(l.reset_pending, 0x0E); /* lane 0's bit consumed, the other three wait for the tick */
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 0), 0);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), 77); /* the idle lane's count waits for the tick */
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 0);
    ASSERT_EQ(l.lane[0].move.enc_start, 0);
    run_ticks(&l, &f, &now, 1, 4, 0); /* the tick applies the other three */
    ASSERT_EQ(l.reset_pending, 0);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), 0);
    run_paced(&l, &f, &now, 32, 4, 4, 1); /* the 8th count on the 32nd tick, 132 pulses: done */
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.filament_um, 8 * 12342 / 10); /* the travel, not the travel minus 1000 counts */
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(132));
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 0), 8 * 12342 / 10);
    ASSERT_EQ(ace2k_lane_fg(&l, 0), 132); /* published from the move's start, not from 500 */
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 1), 0); /* and the idle lane stayed reset */
}

TEST(a_reset_is_refused_while_the_lane_moves_and_accepted_once_it_is_idle)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 1, ACE2K_LANE_FORWARD, 10000, 30000, now), 0); /* 8 counts */
    ASSERT_EQ(ace2k_lane_reset(&l, 1), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_lane_reset(&l, ACE2K_LANE_ALL), -ACE2K_EREFUSED);
    ASSERT_EQ(l.reset_pending, 0);         /* nothing changed */
    ASSERT_EQ(ace2k_lane_reset(&l, 0), 0); /* an idle lane, by itself, is fine */
    ASSERT_EQ(l.reset_pending, 0x01);
    ASSERT_EQ(ace2k_lane_reset(&l, 4), -ACE2K_EINVAL);
    for (int i = 0; i < 32; i++) { /* a count every four ticks: the 8th ends the move */
        now += 10;
        f.fg[1] += 4;
        f.enc[1] = (uint16_t)(f.enc[1] + ((i % 4 == 3) ? 1U : 0U));
        ace2k_lane_tick(&l, now, true);
    }
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 1));
    ASSERT_EQ(ace2k_lane_reset(&l, 1), 0);
    ASSERT_EQ(ace2k_lane_reset(&l, ACE2K_LANE_ALL), 0);
    ASSERT_EQ(l.reset_pending, 0x0F);
}

TEST(a_short_move_at_the_ceiling_whose_landing_crawls_ends_before_its_deadline)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    uint32_t start = now;
    /* 15 mm at 70 mm/s: 5 mm at the ceiling (5000 × 1000 / 70 000 = 71 ms) and the 10 mm landing
     * at the floor speed (10 000 × 1000 / 9000 = 1111 ms) — expected 1182, the deadline
     * 1182 × 3 / 2 + 1000 = 2773 ms; the whole move budgeted at the ceiling would have given
     * 15 000 × 1000 / 70 000 = 214 → 214 × 3 / 2 + 1000 = 1321 ms.  15 mm is 12 counts. */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 15000, ACE2K_LANE_SPEED_MAX_UM_S, now), 0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, start + 2773);
    run_paced(&l, &f, &now, 7, 8, 2, 1); /* 3 counts: 9 remain, 11 107 µm — no landing yet */
    ASSERT_TRUE(!l.lane[0].move.landing);
    run_ticks(&l, &f, &now, 1, 8, 1); /* the 4th: 8 remain, 9 874 µm — the landing begins */
    ASSERT_TRUE(l.lane[0].move.landing);
    /* a heavy spool: the landing crawls at 3 pulses per 4 ticks, ≈ 6 mm/s, under the floor
     * speed, the strand a count every 20 ticks (≈ 6.2 mm/s); the 8 counts take 160 ticks —
     * 1.68 s in all, past the old budget, well within the new one */
    for (int i = 0; i < 160; i++) {
        now += 10;
        f.fg[0] += (i % 4 == 3) ? 0U : 1U;
        f.enc[0] = (uint16_t)(f.enc[0] + ((i % 20 == 19) ? 1U : 0U));
        ace2k_lane_tick(&l, now, true);
    }
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 0));
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.duration_ms, 1680);
    ASSERT_TRUE(ace2k_time_after(start + r.duration_ms, start + 1321));
    ASSERT_TRUE(!ace2k_time_after(start + r.duration_ms, start + 2773));
}

TEST(a_move_no_longer_than_the_landing_is_budgeted_whole_at_the_floor_speed)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 5 mm at the ceiling: shorter than the 10 mm landing, so nothing is budgeted at the
     * commanded speed — 5 000 × 1000 / 9000 = 555 ms at the floor speed, then 555 × 3 / 2 = 832
     * (integer) + 1000 = 1832 ms; whole at the ceiling it would have been 71 → 1106 */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 5000, ACE2K_LANE_SPEED_MAX_UM_S, now), 0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 1832);
    ace2k_lane_stop(&l, 0, now);
    /* exactly the landing's length, at 30 mm/s: 10 000 × 1000 / 9000 = 1111 → 1666 + 1000 */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, ACE2K_LANE_LANDING_UM, 30000, now), 0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 2666);
}

/* Feed the tach `pulses` and the encoder `counts` over one speed-loop window of ten ticks, both
 * spread evenly (the loop measures the window's total, so this sets what it sees: 37 → 29 959
 * µm/s, 25 → 20 242, 15 → 12 145, 10 → 8 097, 5 → 4 048; three counts land on the 4th, 7th and
 * 10th tick, two on the 5th and 10th, one on the 10th). */
static void run_loop_with(struct ace2k_lane *l, struct ace2k_fake_counters *f, uint32_t *now,
                          uint32_t pulses, uint32_t counts)
{
    for (uint32_t i = 0; i < ACE2K_LANE_LOOP_TICKS; i++) {
        uint32_t step =
            (pulses * (i + 1) / ACE2K_LANE_LOOP_TICKS) - (pulses * i / ACE2K_LANE_LOOP_TICKS);
        uint32_t count =
            (counts * (i + 1) / ACE2K_LANE_LOOP_TICKS) - (counts * i / ACE2K_LANE_LOOP_TICKS);
        run_ticks(l, f, now, 1, step, (int)count);
    }
}

TEST(a_landing_from_a_cruise_holds_the_floor_while_the_motor_is_still_above_the_floor_setpoint)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 100 mm at 30 mm/s (the table: 30 + (30 000 − 24 858) × 10 / 9393 = 35 %), 81 counts; the
     * tach at 37 pulses a loop is 29 959 µm/s — an error of 41, P 0, the integral 41 a loop and
     * still worth 0 % after the 29 cruise loops (1 189 / 6000), so the duty sits on the table.
     * The strand at two and three counts a loop in turn (≈ 3.1 mm a loop against 3.0 of motor):
     * 72 counts after 29 loops, 9 remain (11 107 µm > 10 mm) — no landing yet */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, 30000, now), 0);
    ASSERT_EQ(f.duty[0], 35);
    for (int i = 0; i < 29; i++) {
        run_loop_with(&l, &f, &now, 37, (i % 2) ? 3U : 2U);
    }
    ASSERT_TRUE(!l.lane[0].move.landing);
    ASSERT_EQ(ace2k_lane_encoder_count(&l, 0), 72);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 29 * 41);
    ASSERT_EQ(f.duty[0], 35);
    /* the 73rd count lands on the fourth tick of the 30th window (8 remain: the landing), so the
     * 30th loop is the first with the floor setpoint, 9 000, whose table entry is the floor duty.
     * The motor still at 29 959: the error −20 959, P −13, the applied duty 20 − 13 + 0 = 7 is
     * past the floor with the error negative, so the error stays out and the integral keeps the
     * cruise's 1 189 (inside the floor's window, 0..300 000); the duty is the floor.  Then
     * 20 242 (P −7, applied 13) and 12 145 (P −2, applied 18): frozen, on the floor.  The P term
     * alone keeps the duty there for as long as the motor is above the setpoint, and nothing
     * accumulates to lift it early. */
    run_loop_with(&l, &f, &now, 37, 3);
    ASSERT_TRUE(l.lane[0].move.landing);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 29 * 41);
    run_loop_with(&l, &f, &now, 25, 2);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    run_loop_with(&l, &f, &now, 15, 1);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_EQ(l.lane[0].move.integ_um_s, 29 * 41);
    /* below the setpoint at 8 097 the error is 903: the applied duty 20 + 0 + 0 is at the floor,
     * not past it, so the error integrates (2 092) — worth 0 % as P is: the floor holds one more
     * loop; at 4 048 (error 4 952, P 3) the integral is 7 044, worth 1, and the duty rises to
     * 20 + 3 + 1 = 24 */
    run_loop_with(&l, &f, &now, 10, 1);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_EQ(l.lane[0].move.integ_um_s, (29 * 41) + 903);
    run_loop_with(&l, &f, &now, 5, 0);
    ASSERT_EQ(f.duty[0], 24);
    ASSERT_EQ(l.lane[0].move.integ_um_s, (29 * 41) + 903 + 4952);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0)); /* 79 of 81 counts */
}

TEST(the_integral_stops_at_its_clamp_while_the_duty_stays_under_the_ceiling)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 20 mm/s asked (the table: 24 %) with the tach at 1 pulse per tick, 8 097 µm/s: the error is
     * 11 903 a loop and P = 7, so the duty is 31 + integral / 6000 and never passes 81 — the
     * ±INTEG_MAX clamp is the tighter bound here ((100 − 24) × 6000 = 456 000 lies above it):
     * the integral climbs unhindered and meets it, 50 × 6000 = 300 000, on the 26th loop
     * (26 × 11 903 = 309 478 clamped); one pulse a tick keeps the stall check quiet */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 2000000, 20000, now), 0);
    ASSERT_EQ(f.duty[0], 24);
    for (int i = 0; i < 25; i++) {
        run_ticks(&l, &f, &now, 10, 1, 0);
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, 25 * 11903); /* 297 575: one loop short of the clamp */
    ASSERT_EQ(f.duty[0], 80);
    for (int i = 0; i < 5; i++) {
        run_ticks(&l, &f, &now, 10, 1, 0);
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, ACE2K_LANE_INTEG_MAX_PCT * ACE2K_LANE_KI_DIV);
    ASSERT_EQ(f.duty[0], 81);
}

TEST(the_integral_does_not_wind_down_at_the_floor_and_answers_a_load_at_once)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 2000000, ACE2K_LANE_SPEED_MIN_UM_S, now),
              0);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    /* unloaded, the floor duty runs the motor faster than the floor setpoint: 2 pulses per tick
     * is 16 194 µm/s against 9 000 asked, an error of −7194 every loop.  The applied duty
     * 20 − 4 + 0 = 16 is past the floor with the error negative, so the error stays out — and the
     * table's entry for the floor setpoint is the floor duty, so the window has nothing below 0
     * either way: 30 loops on the integral is still 0, not 30 × −7194, and the P term alone holds
     * the duty at the floor. */
    for (int i = 0; i < 30; i++) {
        run_ticks(&l, &f, &now, 10, 2, 0);
    }
    ASSERT_EQ(l.lane[0].move.integ_um_s, 0);
    ASSERT_EQ(f.duty[0], ACE2K_LANE_DUTY_MIN_PCT);
    /* a load arrives: one pulse every other tick is 4 048 µm/s, under half the setpoint.  With
     * nothing to unwind, the first loop already lifts the duty (20 + 4952 / 1500 = 23). */
    for (int i = 0; i < 30; i++) {
        now += 10;
        f.fg[0] += (i % 2 == 0) ? 1U : 0U;
        ace2k_lane_tick(&l, now, true);
    }
    ASSERT_TRUE(f.duty[0] > ACE2K_LANE_DUTY_MIN_PCT);
    ASSERT_TRUE(l.lane[0].move.integ_um_s > 0);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
}

TEST(the_scale_is_accepted_within_twenty_percent_and_refused_outside)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    ASSERT_EQ(ace2k_lane_scale(&l, 0), ACE2K_LANE_UM_PER_COUNT_X10);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 0, 9873), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 0, 14811), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 4, 12342), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 0, 9874), 0);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 0, 14810), 0);
    ASSERT_EQ(ace2k_lane_scale_set(&l, 3, 13000), 0);
    ASSERT_EQ(ace2k_lane_scale(&l, 3), 13000);
    ASSERT_EQ(ace2k_lane_scale(&l, 0), 14810);
    f.enc[3] = 10;
    ace2k_lane_tick(&l, 20, true);
    ASSERT_EQ(ace2k_lane_encoder_um(&l, 3), 13000); /* 10 counts × 1300.0 µm */
}

TEST(a_move_whose_strand_never_moves_times_out_at_the_deadline_however_far_the_motor_ran)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* 100 mm at 30 mm/s: 90 mm at 30 mm/s is 3000 ms and the 10 mm landing at the floor speed
     * 1111 — expected 4111, the deadline 4111 × 3 / 2 + 1000 = 7166 ms after the start, i.e. the
     * tick at 7180 ms (the 717th).  The motor runs the whole time at 4 pulses a tick — it passes
     * the length's worth of tach, 1235 pulses, on the 309th tick — and the encoder never moves:
     * no strand in the lane, or one held fast.  The stall check stays quiet (the tach turns). */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, 30000, now), 0);
    ASSERT_EQ(l.lane[0].move.deadline_ms, now + 7166);
    run_ticks(&l, &f, &now, 309, 4, 0); /* 1236 pulses of motor, 0 of 81 counts: not done */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(!l.lane[0].move.landing); /* the whole length remains: no landing either */
    run_ticks(&l, &f, &now, 407, 4, 0);   /* the 716th tick, 7170 ms: still moving */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 4, 0); /* 7180 ms: past the deadline */
    ASSERT_TRUE(!ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(!f.run[0]);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_TIMEOUT);
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(2868)); /* 232 mm of motor … */
    ASSERT_EQ(r.filament_um, 0);                      /* … for no strand */
}

TEST(a_move_ends_on_the_strands_travel_whether_it_lags_or_leads_the_motor)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    struct ace2k_lane_result r;
    /* 100 mm, 81 counts.  A heavy spool: the motor at 9 pulses a tick (≈ 73 mm/s), the strand a
     * count every two ticks (≈ 62 mm/s, 15 % behind) — done on the 81st count, tick 162, with
     * 1458 pulses (118 mm) of motor: more than the length */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MAX_UM_S, now),
              0);
    run_paced(&l, &f, &now, 161, 9, 2, 1); /* 80 counts */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 9, 1);
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.filament_um, 99970);
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(1458));
    ASSERT_TRUE(r.motor_um > 100000);
    ASSERT_EQ(r.duration_ms, 1620);
    /* the head pulling the strand ahead: the motor at 7 pulses a tick (≈ 57 mm/s), the strand
     * still a count every two ticks (9 % ahead) — done on the same 81st count, tick 162, with
     * 1134 pulses (92 mm) of motor: less than the length */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 100000, ACE2K_LANE_SPEED_MAX_UM_S, now),
              0);
    run_paced(&l, &f, &now, 161, 7, 2, 1);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    run_ticks(&l, &f, &now, 1, 7, 1);
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.filament_um, 99970);
    ASSERT_EQ(r.motor_um, ace2k_lane_fg_to_um(1134));
    ASSERT_TRUE(r.motor_um < 100000);
    ASSERT_EQ(r.duration_ms, 1620);
    /* in reverse the travel is counted toward the spool: a rollback whose encoder runs forward —
     * the head pulling while the lane rewinds — never closes, whatever the motor does */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_REVERSE, 10000, 30000, now), 0); /* 8 counts */
    run_ticks(&l, &f, &now, 40, 4, 1); /* +40 counts: 48 remain along the command */
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    ace2k_lane_stop(&l, 0, now);
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_STOPPED);
    ASSERT_EQ(r.filament_um, 40 * 12342 / 10); /* signed toward the printer, as always */
}

TEST(the_target_is_the_length_in_counts_at_the_lanes_scale_rounded_and_at_least_one)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    /* at the default 1234.2 µm a count: 100 mm is 81.02 → 81, 200 mm 162.05 → 162, 10 mm
     * 8.10 → 8, the 2 000 mm bound 1620.48 → 1620; the rounding turns at half a count: 617 µm
     * is 0.4999 → 0, 618 µm 0.5007 → 1 */
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 100000), 81);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 200000), 162);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 10000), 8);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, ACE2K_LANE_MOVE_MAX_UM), 1620);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 617), 0);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 618), 1);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 4, 100000), 0); /* out of range */
    /* the lane's own scale: at 1200.0 µm a count 100 mm is 83.33 → 83 */
    ASSERT_EQ(ace2k_lane_scale_set(&l, 1, 12000), 0);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 1, 100000), 83);
    ASSERT_EQ(ace2k_lane_um_to_counts(&l, 0, 100000), 81); /* the other lanes keep theirs */
    /* a length under half a count is still one count, and the move ends on the first count;
     * with one count to go it is inside the landing from its first tick */
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500, 30000, now), 0);
    ASSERT_EQ(l.lane[0].move.target_counts, 1);
    run_ticks(&l, &f, &now, 3, 4, 0);
    ASSERT_TRUE(ace2k_lane_is_moving(&l, 0));
    ASSERT_TRUE(l.lane[0].move.landing);
    run_ticks(&l, &f, &now, 1, 4, 1);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_EQ(r.outcome, ACE2K_LANE_DONE);
    ASSERT_EQ(r.filament_um, 1234); /* one count: more than the 500 µm asked */
    /* a move takes the target at its own lane's scale: 10 mm on lane 1 is 8 counts of 1200 µm */
    ASSERT_EQ(ace2k_lane_move(&l, 1, ACE2K_LANE_FORWARD, 10000, 30000, now), 0);
    ASSERT_EQ(l.lane[1].move.target_counts, 8);
    ace2k_lane_stop(&l, 1, now);
}

TEST(an_unpopped_result_is_overwritten_and_counted)
{
    struct ace2k_lane l;
    struct ace2k_fake_counters f;
    primed(&l, &f);
    uint32_t now = 10;
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 30000, now), 0);
    ace2k_lane_stop(&l, 0, now);
    ASSERT_EQ(ace2k_lane_move(&l, 0, ACE2K_LANE_FORWARD, 500000, 30000, now), 0);
    ace2k_lane_stop(&l, 0, now);
    ASSERT_EQ(l.lane[0].results_dropped, 1);
    struct ace2k_lane_result r;
    ASSERT_TRUE(ace2k_lane_pop_result(&l, 0, &r));
    ASSERT_TRUE(!ace2k_lane_pop_result(&l, 0, &r));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
