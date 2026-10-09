#include "test.h"
#include "feed/feed.h"

/* The world: the lane's fake counters and lines, the feed's inputs, one clock. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct world {
    uint16_t enc[ACE2K_LANE_COUNT];
    uint32_t fg[ACE2K_LANE_COUNT];
    uint8_t duty[ACE2K_LANE_COUNT];
    bool run[ACE2K_LANE_COUNT];
    bool reverse[ACE2K_LANE_COUNT];
    /* every forward start of the motor, even one stopped again within the same tick */
    uint32_t forward_starts[ACE2K_LANE_COUNT];
    struct ace2k_feed_inputs in;
    uint32_t now;
    struct ace2k_lane lane;
    struct ace2k_feed feed;
};

static uint16_t w_enc(void *ctx, uint8_t lane)
{
    return ((struct world *)ctx)->enc[lane];
}

static uint32_t w_fg(void *ctx, uint8_t lane)
{
    return ((struct world *)ctx)->fg[lane];
}

static void w_pwm(void *ctx, uint8_t lane, uint8_t duty_pct)
{
    ((struct world *)ctx)->duty[lane] = duty_pct;
}

static void w_run(void *ctx, uint8_t lane, bool on)
{
    struct world *w = ctx;
    w->run[lane] = on;
    if (on && !w->reverse[lane]) {
        w->forward_starts[lane]++;
    }
}

static void w_dir(void *ctx, uint8_t lane, bool reverse)
{
    ((struct world *)ctx)->reverse[lane] = reverse;
}

static const struct ace2k_lane_ops ace2k_w_ops = {
    .encoder_read = w_enc,
    .fg_read = w_fg,
    .pwm_set = w_pwm,
    .run_set = w_run,
    .dir_set = w_dir,
};

/* Both cores primed at t = 10 ms, the link up, filament in lane 1 (bit 0), every plunger at
 * rest. */
static void world_init(struct world *w)
{
    *w = (struct world){ 0 };
    w->now = 10;
    w->in.link_ok = true;
    w->in.insert = 0x01;
    w->in.rest = 0x0F;
    ace2k_lane_init(&w->lane, &ace2k_w_ops, w);
    ace2k_feed_init(&w->feed, &w->lane);
    w->feed.load.auto_load = false; /* the automatic load's own test arms it */
    ace2k_lane_tick(&w->lane, w->now, true);
    ace2k_feed_tick(&w->feed, w->now, &w->in);
}

/* One 10 ms tick: lane 0's tach and encoder advance by the given amounts, both cores tick. */
static void step(struct world *w, uint32_t pulses, int counts)
{
    w->now += 10;
    w->fg[0] += pulses;
    w->enc[0] = (uint16_t)(w->enc[0] + counts);
    ace2k_lane_tick(&w->lane, w->now, w->in.link_ok);
    ace2k_feed_tick(&w->feed, w->now, &w->in);
}

static void steps(struct world *w, int n, uint32_t pulses, int counts)
{
    for (int i = 0; i < n; i++) {
        step(w, pulses, counts);
    }
}

/* n ticks with the tach at `pulses` a tick and the encoder advancing count_step (+1 forward, −1
 * in reverse) once every `every` ticks — on the every-th, 2·every-th … tick of the call: n / every
 * counts in all.  A move ends on the encoder (the length is the strand's travel), so this is what
 * finishes one: 4 pulses a tick is ≈ 32 mm/s of motor, a count every four ticks ≈ 31 mm/s of
 * strand — the few per cent the drive gear slips under a spool.  At 1234.2 µm a count 10 mm is 8 counts, 20 mm 16, 50 mm 41, 100 mm 81. */
static void paced(struct world *w, int n, uint32_t pulses, int every, int count_step)
{
    for (int i = 0; i < n; i++) {
        step(w, pulses, ((i + 1) % every == 0) ? count_step : 0);
    }
}

/* n ticks with a tach pulse every `every` ticks and the strand still: a motor turning too slowly
 * for the standstill's 20 mm to come before a short move's deadline, yet never silent for the
 * lane's own second. */
static void crawl(struct world *w, int n, int every)
{
    for (int i = 0; i < n; i++) {
        step(w, ((i + 1) % every == 0) ? 1U : 0U, 0);
    }
}

static bool pop(struct world *w, struct ace2k_feed_event *e)
{
    return ace2k_feed_pop_event(&w->feed, e);
}

TEST(start_refusals_no_link_in_error_busy_bounds)
{
    struct world w;
    world_init(&w);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 11, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 4, ACE2K_FEED_CMD_FEED, 100000, 30000, 12, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 0, 30000, 13, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 1000, 14, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 15, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 100000, 30000, 16, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, 0));
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, 1));
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, ACE2K_LANE_ALL)); /* one lane is enough */
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, 4));             /* out of range: not busy */
    /* a stall puts the lane in error; every start is refused until clear */
    steps(&w, 101, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_MOTOR_STALLED);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 17, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, 0));
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, ACE2K_LANE_ALL)); /* error is not busy */
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 18, w.now),
              ACE2K_FEED_ACCEPTED);
}

TEST(a_feed_runs_to_done_with_the_odometers_in_the_event)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 21, w.now),
              ACE2K_FEED_ACCEPTED);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
    paced(&w, 324, 4, 4, 1); /* the 81st count on the 324th tick: done; 1296 pulses of motor */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(1296)); /* 104.9 mm of motor … */
    ASSERT_EQ(e.filament_um, 81 * 12342 / 10);        /* … for the 100 mm of strand asked */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_counter_reset_pending_at_the_start_does_not_reach_the_odometers)
{
    struct world w;
    world_init(&w);
    steps(&w, 10, 50, 100); /* 1000 counts on lane 0 before the feed */
    ASSERT_EQ(ace2k_lane_reset(&w.lane, 0), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 31, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, 1); /* the 8th count on the 32nd tick: done; 128 pulses */
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.filament_um, 8 * 12342 / 10);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(128));
}

TEST(a_rollback_moves_in_reverse_and_reports_a_negative_filament_travel)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 50000, 30000, 41, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.reverse[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    paced(&w, 164, 4, 4, -1); /* 50 mm is 41 counts, back: done on the 164th tick */
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(e.filament_um < 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(the_motor_outcomes_become_errors_or_stops)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* timeout: 100 mm at the ceiling, 70 mm/s — 90 mm at the ceiling is 1285 ms and the 10 mm
     * landing at the floor speed 1111, expected 2396, the deadline 2396 × 3 / 2 + 1000 = 4594 ms
     * after the start, the 460th tick; the tach creeping (1 pulse a tick, ≈ 8 mm/s, keeps the
     * stall check quiet) with the strand keeping pace — a count every 12 ticks is ≈ 10 mm/s, so
     * the comparator has nothing to say — and 38 of the 81 counts by the deadline */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               51, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 459, 1, 12, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    steps(&w, 1, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TIMEOUT);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TIMEOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 51);
    ASSERT_EQ(w.feed.l[0].seq, 51); /* in error the lane keeps the failed start's seq … */
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 51); /* … and the state report shows it … */
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    ASSERT_EQ(w.feed.l[0].seq, 0);             /* … until the clear; */
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 51); /* the report keeps the last host start's */
    /* link lost: idle with stopped_link, not an error */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 52, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 4, 3);
    w.in.link_ok = false;
    step(&w, 4, 3);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.seq, 52);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 53, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    /* shutdown: the binding aborts the lane core; the feed sees it on its tick */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 54, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_lane_abort_all(&w.lane, ACE2K_LANE_SHUTDOWN, w.now);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SHUTDOWN);
    ASSERT_EQ(e.seq, 54); /* not the refused start's 53: a refusal enters no mode */
}

/* After a Klipper shutdown: n ticks with the link up (the host's clock queries keep it so) and
 * whatever the switches read — lane 0 never moves, stays idle, and no event. */
static void nothing_moves_after_the_shutdown(struct world *w, int n)
{
    struct ace2k_feed_event e;
    for (int i = 0; i < n; i++) {
        step(w, 0, 0);
        ASSERT_TRUE(!w->run[0]);
        ASSERT_TRUE(!w->lane.lane[0].move.active);
        ASSERT_EQ(ace2k_feed_mode(&w->feed, 0), ACE2K_FEED_IDLE);
    }
    ASSERT_TRUE(!pop(w, &e));
}

/* The one event a Klipper shutdown leaves for lane 0, and nothing after it. */
static void one_stopped_shutdown(struct world *w, uint8_t mode, uint8_t seq)
{
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(w, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SHUTDOWN);
    ASSERT_EQ(e.mode, mode);
    ASSERT_EQ(e.seq, seq);
    ASSERT_TRUE(!pop(w, &e));
}

TEST(a_shutdown_disarms_a_forward_assist_waiting_between_bursts)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 30000, 91, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* taut: a burst */
    steps(&w, 5, 4, 1);
    ASSERT_TRUE(w.run[0]);
    w.in.pushed = 0x00;
    w.in.rest = 0x0F; /* back at rest: the burst ends, the lane waits for the next pull */
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ace2k_lane_abort_all(&w.lane, ACE2K_LANE_SHUTDOWN, w.now); /* no move: nothing to end */
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_stopped_shutdown(&w, ACE2K_FEED_ASSISTING, 91);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* the head pulls taut again, the link up: no burst */
    nothing_moves_after_the_shutdown(&w, 100);
}

TEST(a_shutdown_disarms_a_reverse_assist_waiting_between_take_ups)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 30000, 92, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_stopped_shutdown(&w, ACE2K_FEED_ASSISTING_BACK, 92);
    w.in.rest = 0x0E;
    w.in.pulled_any = true; /* this lane's buffer full: no take-up */
    nothing_moves_after_the_shutdown(&w, 100);
}

TEST(a_shutdown_cancels_a_load_in_its_settle_and_the_pull_never_starts)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 93, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 20, 0, 0); /* 200 ms of the 500 ms settle */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SETTLE);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_stopped_shutdown(&w, ACE2K_FEED_LOADING, 93);
    nothing_moves_after_the_shutdown(&w, 100); /* well past the settle's end: no pull */
}

TEST(after_a_shutdown_an_insert_edge_starts_no_automatic_load)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_TRUE(!pop(&w, &e)); /* every lane idle: no event */
    w.in.insert = 0x01;        /* the edge, the link up */
    nothing_moves_after_the_shutdown(&w, 100);
}

TEST(a_moving_lane_gets_one_stopped_shutdown_whichever_handler_runs_first)
{
    struct world w;
    /* the lane's handler first, then the feed's */
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 94, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 4, 1);
    ASSERT_TRUE(w.run[0]);
    ace2k_lane_abort_all(&w.lane, ACE2K_LANE_SHUTDOWN, w.now);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_stopped_shutdown(&w, ACE2K_FEED_FEEDING, 94);
    nothing_moves_after_the_shutdown(&w, 10);
    /* the feed's first, then the lane's: the lane has no move left to end */
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 95, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 4, 1);
    ace2k_feed_shutdown(&w.feed, w.now);
    ace2k_lane_abort_all(&w.lane, ACE2K_LANE_SHUTDOWN, w.now);
    ASSERT_TRUE(!w.run[0]);
    one_stopped_shutdown(&w, ACE2K_FEED_FEEDING, 95);
    nothing_moves_after_the_shutdown(&w, 10);
    /* the lane's, a feed tick that reads its result, then the feed's: still one */
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 96, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 4, 1);
    ace2k_lane_abort_all(&w.lane, ACE2K_LANE_SHUTDOWN, w.now);
    step(&w, 0, 0);
    ace2k_feed_shutdown(&w.feed, w.now);
    one_stopped_shutdown(&w, ACE2K_FEED_FEEDING, 96);
    nothing_moves_after_the_shutdown(&w, 10);
}

TEST(after_a_shutdown_every_start_is_refused_as_without_a_link)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 97, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 98, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    steps(&w, 5, 0, 0); /* the link up on every tick */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 99, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 100, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    /* the stopped start once more, inside its retry window: refused too */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 97, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    ASSERT_TRUE(!w.run[0]);
    /* only the MCU's reset — init — lifts it */
    ace2k_feed_init(&w.feed, &w.lane);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 101, w.now),
              ACE2K_FEED_ACCEPTED);
}

TEST(a_lane_in_error_stays_in_error_across_a_shutdown)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 102, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 101, 0, 0); /* the tach silent 1 s: motor_stalled */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_MOTOR_STALLED);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_MOTOR_STALLED);
    ASSERT_TRUE(!pop(&w, &e)); /* no event for a lane already stopped in error */
    steps(&w, 50, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_insert_falling_is_a_runout_in_feed_and_rollback_and_the_goal_of_an_unload)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 61, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 20, 4, 3);
    w.in.insert = 0x00;
    step(&w, 4, 3);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(84));
    ASSERT_TRUE(!pop(&w, &e)); /* the lane's own stopped result was discarded */
    /* the lane now empty: a feed is refused — nothing to push, and with the stop on the encoder
     * it could only end tangled — while a rollback is accepted: it is what brings a piece parked
     * past the sensor back out (measured on the unit) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 62, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 62, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, -1); /* the piece's 8 counts back: done */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    /* unload: reverse until the sensor clears */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 63, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_UNLOAD_MAX_UM)); /* 1620 */
    steps(&w, 50, 4, -3);
    w.in.insert = 0x00;
    step(&w, 4, -3);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 63);
    ASSERT_TRUE(e.filament_um < 0);
}

TEST(a_feed_or_an_unload_needs_a_strand_at_the_mouth_but_a_rollback_does_not)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    /* an unload would rewind the whole budget into unload_incomplete; a feed has nothing to push
     * and, ending on the encoder, would only be a tangle after stall_check_um of motor — the
     * wheels of lanes 2–4 are not dragged (docs/hardware.md "Motors") */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 71, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 72, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 0); /* a refusal records no start */
    /* the same empty lane rolls back: a piece parked past the sensor comes out this way
     * (measured on the unit) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 73, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    /* the strand back at the mouth: the unload and the feed are accepted */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 74, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 75, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
}

TEST(an_unload_whose_budget_runs_out_with_the_strand_still_present_is_an_error)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 10000, 30000, 81, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, -1); /* 8 counts back: the budget spent, the insert still set */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_UNLOAD_INCOMPLETE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_EQ(e.seq, 81);
    ASSERT_TRUE(!w.run[0]);
    /* the hand finishes the unload — the tail pulled out past the sensor: runout, the lane idle,
     * the odometers of the unload (128 pulses, 8 counts back) */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING); /* the mode that failed, not the error it left */
    ASSERT_EQ(e.seq, 81);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(128));
    ASSERT_EQ(e.filament_um, -(8 * 12342 / 10));
    ASSERT_TRUE(!pop(&w, &e));
    /* the falling edge on an idle lane, as ever: nothing */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(stop_always_stops_and_only_a_moving_lane_gets_the_event)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_stop(&w.feed, 0, w.now); /* idle: the lines at stop, no event */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 91, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 10, 4, 3);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(40));
    step(&w, 0, 0);
    ASSERT_TRUE(!pop(&w, &e)); /* the lane's stopped result never surfaces twice */
    /* in error: stop keeps the error and sends nothing */
    w.in.rest = 0x0E; /* a tip held ahead: the rollback pulls the strand taut */
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 100000,
                               ACE2K_LANE_SPEED_MAX_UM_S, 92, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 247, 1, 0); /* 20 mm of motor with the strand still: stuck, an error */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    ASSERT_TRUE(pop(&w, &e));
    w.run[0] = true; /* pretend a line was left high: stop must clear it */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* every lane */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 93, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.insert = 0x05;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 2, ACE2K_FEED_CMD_FEED, 500000, 30000, 94, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_stop(&w.feed, ACE2K_LANE_ALL, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 2), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 0); /* the lanes in order, each event with its own start's seq */
    ASSERT_EQ(e.seq, 93);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 2);
    ASSERT_EQ(e.seq, 94);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(set_speed_reaches_a_running_move_and_is_refused_idle_or_in_error)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 30000, w.now), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 101, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 60000, w.now), 0);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 60000);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 1000, w.now), -ACE2K_EINVAL);
}

TEST(the_done_event_echoes_the_seq_of_the_start)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 7, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.l[0].seq, 7);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 7); /* what the state report publishes */
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 1), 0);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 4), 0); /* out of range: 0 */
    paced(&w, 32, 4, 4, 1);                   /* the 8th count: done */
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.seq, 7);
    ASSERT_EQ(w.feed.l[0].seq, 0); /* idle again: the seq is the firmware's own, 0 */
    /* the report keeps the last host start's seq through idle: a host that reconnects continues
     * after it */
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 7);
    /* the next start's seq replaces it; a rollback's done carries the rollback's */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 8, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 8);
    paced(&w, 32, 4, 4, -1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 8);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 8);
}

TEST(stopped_and_runout_echo_the_seq_of_the_start_of_their_lane)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x05; /* filament in lanes 1 and 3 */
    step(&w, 0, 0);
    /* two lanes with two sequences: a stop on one lane echoes that lane's */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 200, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 2, ACE2K_FEED_CMD_FEED, 500000, 30000, 33, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 10, 4, 3);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.seq, 200);
    ASSERT_EQ(w.feed.l[0].seq, 0);  /* cleared by the stop's way to idle */
    ASSERT_EQ(w.feed.l[2].seq, 33); /* the other lane still holds its own */
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 2, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 2);
    ASSERT_EQ(e.seq, 33);
    /* the insert falling mid-move: the runout carries the start's seq, 255 included */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 255, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 20, 4, 3);
    w.in.insert = 0x04;
    step(&w, 4, 3);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.seq, 255);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_event_ring_drops_and_counts_beyond_sixteen)
{
    struct world w;
    world_init(&w);
    for (int i = 0; i < 20; i++) {
        /* a seq per start: the same seq again would be the transport's retry, not a new move */
        ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000,
                                   (uint8_t)(100 + i), w.now),
                  ACE2K_FEED_ACCEPTED);
        ace2k_feed_stop(&w.feed, 0, w.now);
    }
    ASSERT_EQ(w.feed.dropped, 5); /* the ring holds 15 with one slot spare */
    ASSERT_TRUE(ace2k_feed_has_events(&w.feed));
    struct ace2k_feed_event e;
    int n = 0;
    while (pop(&w, &e)) {
        n++;
    }
    ASSERT_EQ(n, 15);
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
}

TEST(a_start_repeating_the_last_seq_is_the_transports_retry_and_starts_nothing)
{
    /* Klipper re-sends a query whose response it did not get: the second start must not read
     * as a new command — busy during the move, or a second move after it */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    uint32_t t_mode = w.feed.l[0].t_mode_ms;
    paced(&w, 5, 4, 4, 1); /* one count of the 8 */
    /* during the move: accepted, and the move goes on as it was (no re-entry) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].t_mode_ms, t_mode);
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
    /* another seq during the move is a second command: busy, as before */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 10, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    /* the same seq with another length, speed or command is a new command too (the host's
     * counter wrapped, or a host bug): busy during the move */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 9, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 40000, 9, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 9, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].t_mode_ms, t_mode);
    paced(&w, 28, 4, 4, 1); /* seven more counts, the 8th: done */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.seq, 9);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    /* 500 ms after the done: still the retry — accepted, nothing started, no event */
    steps(&w, 50, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
    ASSERT_TRUE(!w.run[0]);
    /* within the window, the same seq with another length: a new command — the lane is idle, so
     * a move starts; its done carries that seq, and the new key replaces the old one */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(w.feed.l[0].last_length_um, 20000);
    paced(&w, 64, 4, 4, 1); /* 20 mm is 16 counts: done on the 64th tick */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.seq, 9);
    /* the retry of that one, within the window: accepted, nothing started */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    /* a set_speed during a move changes the running speed, not the key: the retry of the start as
     * it was asked is still the retry */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 11, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 5, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 40000, w.now), 0);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 40000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 11, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 40000); /* nothing re-entered */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 40000, 11, w.now),
              ACE2K_FEED_REFUSED_BUSY); /* the running speed is not what the start asked */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.seq, 11);
    /* back on seq 9's key for the window's closing: a start of it now is a new command */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, 1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.seq, 9);
    /* 1 500 ms after the done: the window is closed — a new move with that seq */
    steps(&w, 150, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    paced(&w, 32, 4, 4, 1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    /* seq 0 is the firmware's own and never a retry: a second start with 0 right after the
     * first's done is a new move */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 0, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, 1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.seq, 0);
    steps(&w, 10, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 0, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* a lane in error keeps the window too: the retry of the failed start is accepted, another
     * seq is in_error, and past the window the same seq is in_error as well */
    w.in.insert = 0x03; /* a strand in lane 2 (bit 1): a feed needs one */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               51, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 105, 0, 0); /* lane 1's tach never moves: a stall after one second, an error */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_ERROR);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.seq, 51);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               51, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               52, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    /* the same seq at another speed is not the retry: in_error like any new command */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, 30000, 51, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    steps(&w, 101, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               51, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    /* a clear ends no mode: the window stays the failure's, so the failed start repeated after
     * the clear, past the window, is a new command — a move */
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 1));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 1), 51); /* the report keeps it through the clear */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, ACE2K_LANE_SPEED_MAX_UM_S,
                               51, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[1]);
}

TEST(a_mode_the_unit_enters_on_its_own_leaves_the_last_host_starts_retry_window_as_it_was)
{
    /* the automatic load of a later feature enters a mode with seq 0 from the tick; the retry key
     * and its window are the host start's and must survive it — neither closed by the own mode's
     * start nor re-opened by its end — reached through the struct here, as no path of this build
     * enters a mode on its own */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, 1); /* done */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.seq, 9);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    /* the unit's own mode, 200 ms later */
    steps(&w, 20, 0, 0);
    w.feed.l[0].mode = ACE2K_FEED_LOADING;
    w.feed.l[0].sub = ACE2K_FEED_SUB_MOVING;
    w.feed.l[0].seq = 0;
    w.feed.l[0].t_mode_ms = w.now;
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 9); /* the report still names the last host start */
    /* the retry of 9 during it, within the window of 9's end: accepted, the mode untouched */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
    /* a new seq during it: busy, as for any running mode */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 10, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    /* the own mode runs on past 9's window: the same start is no retry — busy, like any new
     * command against a running mode */
    steps(&w, 80, 0, 0); /* 1 210 ms after 9's end */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    /* the own mode ends (the insert falls: a runout of the unit's own, seq 0) — its end does not
     * re-open the host start's window */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 9);
    /* right after that end, with the strand back at the mouth, the same start is a new command:
     * a move, not "accepted, nothing started" */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
}

TEST(a_peek_leaves_the_oldest_event_in_the_ring_for_the_pop)
{
    /* the binding peeks, tries to send, and pops only once the frame is queued: a frame that
     * did not fit waits at the tail for the next tick */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x03; /* strands in lanes 1 and 2 */
    step(&w, 0, 0);
    ASSERT_TRUE(!ace2k_feed_peek_event(&w.feed, &e)); /* empty */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 21, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 500000, 30000, 22, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_stop(&w.feed, 1, w.now);
    ASSERT_TRUE(ace2k_feed_peek_event(&w.feed, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.seq, 21);
    ASSERT_TRUE(ace2k_feed_peek_event(&w.feed, &e)); /* still there, still the oldest */
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(ace2k_feed_has_events(&w.feed));
    ace2k_feed_drop_event(&w.feed); /* the binding's step once the peeked frame is queued */
    ASSERT_TRUE(ace2k_feed_peek_event(&w.feed, &e)); /* the next one */
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(e.seq, 22);
    ASSERT_TRUE(pop(&w, &e)); /* a pop is the peek and the drop */
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(!ace2k_feed_peek_event(&w.feed, &e));
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
    ace2k_feed_drop_event(&w.feed); /* nothing to drop: nothing happens */
    ASSERT_TRUE(!ace2k_feed_has_events(&w.feed));
    ASSERT_EQ(w.feed.tail, w.feed.head);
}

TEST(the_standstill_in_a_feed_with_the_buffer_not_full_is_blocked_not_an_error)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ACE2K_FEED_BLOCKED, 18);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 121, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 61, 4, 0); /* 244 pulses ≈ 19.8 mm of motor, no filament: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    steps(&w, 1, 4, 0); /* 248 pulses ≈ 20.1 mm ≥ 20 mm with < 2 mm of filament */
    /* a push toward the head that met what cannot move: blocked —
     * the lane idle, no error, the strand waiting where it stopped */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 121);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248));
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!pop(&w, &e));                  /* the lane's own stopped result was discarded */
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* nothing more on its own, and the next start is accepted at once */
    steps(&w, 50, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 122, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
}

TEST(the_same_standstill_with_the_buffer_full_is_blocked_and_any_reverse_mode_taut_stuck)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.pulled_any = true;
    w.in.rest = 0x0E; /* this lane's plunger at its pulled end, not another's */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 131, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0); /* no filament, this lane's buffer full: nothing passes ahead */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 131);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    w.in.pulled_any = false;
    /* toward the spool the kind is read from the plunger: a tip held in the hot end pulls the
     * strand taut (pushed) while the drive grinds — stuck at once (measured on the unit); at rest the same standstill is the tail past the drive, tested further down */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 132, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0); /* a tip held in the hot end: under a second at 30 mm/s */
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 132);
    /* the strand pulled out by hand: the stuck ends as a runout, the lane idle — then the strand
     * back for the next case */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 132);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0));
    w.in.insert = 0x01;
    step(&w, 0, 0);
    /* filament moving the wrong way counts as none along the command */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 133, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 3); /* +3 counts per tick while rewinding */
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_TRUE(e.filament_um > 0); /* the odometer keeps the sign: toward the printer */
    /* an unload is a reverse mode too */
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 134, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
}

TEST(the_partial_trigger_trips_at_fifty_millimetres_when_the_filament_is_short_by_more_than_fifteen)
{
    struct world w;
    world_init(&w);
    /* 4 pulses ≈ 324 µm of motor per tick; the window closes at 620 pulses (≥ 50 mm), on the
     * 155th tick.  A count every 8 ticks: 20 counts ≈ 24.7 mm — short by ≈ 25 mm: a trip. */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 141, w.now),
              ACE2K_FEED_ACCEPTED);
    for (int i = 0; i < 154; i++) {
        step(&w, 4, (i % 8 == 0) ? 1 : 0);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING); /* 616 pulses, 49.9 mm: not yet */
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.seq, 141);
    ASSERT_TRUE(e.motor_um > 50000 && e.filament_um > 20000);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(620));
    ASSERT_EQ(e.filament_um, 20 * 12342 / 10);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* a count every 5 ticks: 31 counts ≈ 38.3 mm — short by ≈ 12 mm: the window restarts, the
     * move goes on, window after window (two windows in 310 ticks, 62 of the 405 counts) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 142, w.now),
              ACE2K_FEED_ACCEPTED);
    for (int i = 0; i < 310; i++) {
        step(&w, 4, (i % 5 == 0) ? 1 : 0);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(w.feed.l[0].fg_window, w.lane.lane[0].fg_raw); /* the second window closed here */
}

TEST(a_healthy_move_never_trips_and_the_thresholds_command_takes_effect)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* healthy: 200 mm is 162 counts; a count every 4 ticks ≈ 308 µm per tick against 324 µm of
     * motor, 5 % short — every 50 mm window closes with the strand 2–3 mm behind, restarts, and
     * the move ends on the 162nd count, the 645th tick, with 2580 pulses (209 mm) of motor */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 200000, 30000, 151, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 644, 4, 4, 1); /* 161 counts */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 1); /* the 162nd: done */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.seq, 151);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(2580));
    ASSERT_EQ(e.filament_um, 162 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* the same rollback, healthy: the encoder counts down */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 200000, 30000, 152, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 644, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, -1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 152);
    ASSERT_EQ(e.filament_um, -(162 * 12342 / 10));
    /* tighter thresholds: the same 5 % shortfall trips once 50 mm allow only 1 mm — at the
     * first window's close, the 155th tick: 620 pulses (50.2 mm) against 38 counts (46.9 mm) */
    struct ace2k_feed_thresholds th = {
        .slip_check_um = 50000,
        .slip_allow_um = 1000,
        .stall_check_um = 20000,
    };
    ace2k_feed_thresholds_set(&w.feed, &th);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 200000, 30000, 153, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 154, 4, 4, 1); /* 616 pulses: the window still open */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.seq, 153);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(620));
    ASSERT_EQ(e.filament_um, 38 * 12342 / 10);
}

TEST(a_strand_the_head_pulls_ahead_of_the_motor_never_trips_and_the_move_ends_on_its_travel)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* 200 mm, 162 counts; the motor at 4 pulses a tick (≈ 32 mm/s), the strand a count every
     * three ticks (≈ 41 mm/s: the head pulling).  At the window's close, the 155th tick, 620
     * pulses (50.2 mm) stand against 51 counts (62.9 mm): the strand leads, is not short, and
     * a difference taken the wrong way round would have wrapped into a trip.  Done on the 162nd
     * count, the 486th tick, with 1944 pulses (157 mm) of motor: less than the length. */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 200000, 30000, 161, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 155, 4, 3, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].fg_window, w.lane.lane[0].fg_raw); /* the window restarted */
    paced(&w, 330, 4, 3, 1);                                 /* 161 counts in all */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.seq, 161);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(1944));
    ASSERT_EQ(e.filament_um, 162 * 12342 / 10);
    ASSERT_TRUE(e.motor_um < 200000);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_jam_after_the_strand_followed_is_a_standstill_twenty_millimetres_of_motor_later)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a rollback whose strand follows for 5 counts (6.2 mm) and then is held: the standstill's
     * base is the last tick the strand moved by two counts or more — the 4th count, tick 16,
     * 64 pulses — so the trip comes 248 pulses (20.1 mm ≥ 20) later, 312 pulses, the 78th tick:
     * 232 pulses (18.8 mm) of motor past the jam.  Judged inside the partial's window instead,
     * the trip would have waited for the window's close at 50 mm of motor, the 155th tick. */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 171, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 20, 4, 4, -1); /* the strand follows: 5 counts back, 80 pulses */
    ASSERT_EQ(w.feed.l[0].fg_still, 64);
    w.in.rest = 0x0E; /* held ahead: the drive pulls the strand taut against the hold */
    w.in.pushed = 0x01;
    steps(&w, 57, 4, 0); /* held: 308 pulses, 244 since the base — 19.8 mm, not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(e.seq, 171);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(312));
    ASSERT_EQ(e.filament_um, -(5 * 12342 / 10));
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    /* a feed whose strand follows for 42 counts (51.8 mm), through the first window's close at
     * the 155th tick (620 pulses against 38 counts, 46.9 mm: within the allowance, the window
     * restarts), and is then held: the base is the 42nd count, tick 168, 672 pulses; the trip
     * 248 pulses later, 920 pulses, the 230th tick — 20.1 mm of motor past the jam, where the
     * second window would have closed at 100 mm of motor, the 310th tick */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 172, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 168, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw); /* moved on this very tick */
    ASSERT_EQ(w.feed.l[0].fg_window, w.lane.lane[0].fg_raw - 672 + 620); /* the window's base */
    steps(&w, 61, 4, 0); /* 916 pulses: 244 since the base — not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.seq, 172);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(920));
    ASSERT_EQ(e.filament_um, 42 * 12342 / 10);
}

TEST(one_count_is_still_and_two_are_a_move_at_every_accepted_scale)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* one count on the first tick (1234 µm < 1500) does not move the standstill's base: the trip
     * comes at 248 pulses of motor from the start, the 62nd tick, with that one count in the
     * odometer */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 181, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 1);
    ASSERT_EQ(w.feed.l[0].fg_still, 0);
    steps(&w, 60, 4, 0); /* 244 pulses: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248));
    ASSERT_EQ(e.filament_um, 1234);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* two counts on the first two ticks (2468 µm ≥ 1500) move it to the second tick's 8 pulses:
     * the trip at 256 pulses, the 64th tick */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 182, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 2, 4, 1);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw);
    steps(&w, 61, 4, 0); /* 252 pulses, 244 since the base: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(256));
    ASSERT_EQ(e.filament_um, 2468);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* the widest accepted count, 1481.0 µm (+20 %): one is still */
    ASSERT_EQ(ace2k_lane_scale_set(&w.lane, 0, 14810), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 183, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 1);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw - 4);
    steps(&w, 61, 4, 0); /* the 62nd tick, 248 pulses */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.filament_um, 1481);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* the narrowest, 987.4 µm (−20 %): two, 1974 µm, are a move */
    ASSERT_EQ(ace2k_lane_scale_set(&w.lane, 0, 9874), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 500000, 30000, 184, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 2, 4, 1);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw);
    steps(&w, 61, 4, 0); /* 244 since the base: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    /* the odometer is the difference of two truncated readings: 4 counts stood at the start
     * (3949 µm at this scale), 6 at the trip (5924) — 1975, not the 1974 of two counts alone */
    ASSERT_EQ(e.filament_um, 1975);
}

TEST(the_comparator_judges_only_a_mode_the_feed_entered)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a lane the lane core moves for someone else — the bench, a test — while the feed is idle:
     * 400 pulses (32 mm) with no filament is nobody's tangle; the feed's bases were never taken */
    ASSERT_EQ(ace2k_lane_move(&w.lane, 0, ACE2K_LANE_FORWARD, 500000, 30000, w.now), 0);
    steps(&w, 100, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_lane_stop(&w.lane, 0, w.now);
    step(&w, 0, 0); /* the lane's stopped result: not an event either */
    ASSERT_TRUE(!pop(&w, &e));
    /* the same with the lane in error (a rollback held ahead, stuck): the error stays as it
     * was, no second event */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 191, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(ace2k_lane_move(&w.lane, 0, ACE2K_LANE_FORWARD, 500000, 30000, w.now), 0);
    steps(&w, 100, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_lane_stop(&w.lane, 0, w.now);
}

TEST(the_buffer_full_during_a_feed_is_watched_and_a_strand_that_stops_is_blocked)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a healthy feed: 30 ticks, 120 pulses, 7 counts.  The shared pulled reading with this lane's
     * plunger still at rest is another lane's full and no verdict here: the feed goes on.  This
     * lane's plunger leaves rest and reaches its pulled end on the 31st tick — the watch begins on
     * that tick (base 124 pulses, 8 counts; the count of the tick lands before the feed's tick).
     * The watch, a strand that does not follow: stuck once it lags 5 mm — 64 pulses past the full,
     * the 17th tick of the watch: error, stuck (feeding), the odometers so far, the motor still */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 201, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.pulled_any = true;
    paced(&w, 30, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    w.in.rest = 0x0E;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    steps(&w, 15, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 201);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(124 + 64));
    ASSERT_EQ(e.filament_um, 8 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));                  /* the lane's own stopped result was discarded */
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* a rollback moves away from the head: the full reading is not its business — the move runs
     * to done with the plunger held at its pulled end throughout (50 mm is 41 counts) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 50000, 30000, 202, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 164, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(!pop(&w, &e));
    /* a feed started with this lane's buffer already full: accepted, and watched from its first
     * tick (base 4 pulses) — the watch, a strand that does not follow: stuck once it lags 5 mm,
     * 64 pulses past the base: the head is not taking filament, whatever asked for more */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 203, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    steps(&w, 15, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.seq, 203);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(68));
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* the plunger taut is not full either: the strand pulled by the head while the lane feeds is
     * the feed keeping up, not a jam — the move goes on */
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 204, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 20, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
}

TEST(the_load_grace_skips_the_first_fifty_millimetres)
{
    struct world w;
    world_init(&w);
    /* the load mode by hand until the automatic load exists: the lane moves, the feed judges
     * from the mode; seq 0, a mode of the unit's own */
    ASSERT_EQ(ace2k_lane_move(&w.lane, 0, ACE2K_LANE_FORWARD, 500000, 30000, w.now), 0);
    w.feed.l[0].mode = ACE2K_FEED_LOADING;
    w.feed.l[0].sub = ACE2K_FEED_SUB_MOVING; /* the pull */
    w.feed.l[0].fg_start = w.lane.lane[0].fg_raw;
    w.feed.l[0].enc_um_start = ace2k_lane_encoder_um(&w.lane, 0);
    w.feed.l[0].fg_window = w.feed.l[0].fg_start; /* the bases enter() would have taken */
    w.feed.l[0].enc_um_window = w.feed.l[0].enc_um_start;
    w.feed.l[0].fg_still = w.feed.l[0].fg_start;
    w.feed.l[0].enc_um_still = w.feed.l[0].enc_um_start;
    steps(&w, 150, 4, 0); /* 600 pulses ≈ 48.6 mm of motor, no filament: inside the grace */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw); /* both bases restart meanwhile */
    ASSERT_EQ(w.feed.l[0].fg_window, w.lane.lane[0].fg_raw);
    /* the grace ends on the 155th tick (620 pulses ≥ 50 mm), the standstill's base the 154th's
     * 616; it trips 248 pulses later — 864 pulses, the 216th tick */
    steps(&w, 65, 4, 0); /* 860 pulses: 244 in the window, 19.8 mm — not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    step(&w, 4, 0);
    /* the pull met what cannot move: blocked, the lane idle, no
     * error, the strand where it stopped */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(864)); /* the mode's odometer, grace included */
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_forward_assist_bursts_on_taut_and_stops_at_rest_or_full)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 211, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 1000, 212, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    /* armed with the plunger at rest: nothing moves — the head has not pulled yet, and no timer
     * starts a burst (decided at the bench) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 213, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, 0));
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    steps(&w, 300, 0, 0); /* 3 s at rest: still nothing */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    /* the head pulls the strand taut: a burst at the armed speed, its bound the strand's travel
     * (100 mm is 81 counts) */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_ASSIST_BURST_UM));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 50000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    /* feeding taut, the strand moving: 20 ticks, 80 pulses, 20 counts — taut is the normal start
     * of a burst, not a notice (observed at the bench: fourteen notices in a minute
     * of ordinary pulling); a burst that ends at rest before its bound says nothing */
    steps(&w, 20, 4, 1);
    ASSERT_TRUE(!pop(&w, &e));
    /* the plunger back at rest: the burst stops on that tick — no event, the lane waits */
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 300, 0, 0); /* rest is not a start: nothing for 3 s */
    ASSERT_TRUE(!w.run[0]);
    /* taut again: burst 2, the comparator's bases taken afresh */
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw);
    ASSERT_EQ(w.feed.l[0].fg_window, w.lane.lane[0].fg_raw);
    /* the full reading during a burst stops it, as a safety — this lane's plunger at its pulled
     * end, or any lane's: one input for the four */
    w.in.pushed = 0x00;
    w.in.pulled_any = true;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!pop(&w, &e));
    /* this lane taut while another lane's plunger holds the full reading: waiting; the reading
     * gone, taut starts burst 3 at once */
    w.in.pushed = 0x01;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    w.in.pulled_any = false;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    /* a stop disarms with the event, the mode's odometers: 22 counts fed over the three bursts */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 213);
    ASSERT_EQ(e.filament_um, 22 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_burst_reaching_its_bound_while_taut_chains_into_the_next)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* armed taut: the first burst at once.  The head keeps pulling near the burst's speed — the
     * plunger never comes back to rest — so the burst runs its 81 counts to its bound, the 81st
     * on the 324th tick (a count every four ticks, 1296 pulses of motor, inside its deadline:
     * 100 mm at 50 mm/s is 2.9 s expected, 5.4 s allowed), and the next burst starts on that
     * tick: no event, no error — a bound reached is not a fault (decided at the bench) */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 221, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    paced(&w, 323, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_TRUE(!pop(&w, &e)); /* taut and moving is the normal burst: nothing yet */
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_ASSIST_BURST_UM));
    ASSERT_EQ(w.lane.lane[0].move.enc_start, ace2k_lane_encoder_count(&w.lane, 0));
    ASSERT_EQ(w.feed.l[0].fg_still, w.lane.lane[0].fg_raw); /* the bases afresh */
    /* the bound reached with the strand still taut is the head outrunning a whole burst: the
     * behind notice, on the tick of the chain, with the mode's odometers (1296 pulses, 81 counts) */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BEHIND);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 221);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(1296));
    ASSERT_EQ(e.filament_um, 81 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* and again: the same episode, no second notice, burst 3 on the 648th tick */
    paced(&w, 323, 4, 4, 1);
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* the plunger back at rest: the burst stops; the mode's odometer spans the chain */
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* a second taut episode: its own notice, again only at a bound reached taut — burst 4 runs
     * its 81 counts, burst 5 chains on the 324th tick of the episode with the notice */
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 4);
    paced(&w, 323, 4, 4, 1);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 5);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BEHIND);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 0, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.seq, 221);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(2592 + 1296));
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10);
}

TEST(taut_and_still_under_command_in_an_assist_is_tangled_at_the_standstill_never_assist_stall)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 231, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.run[0]); /* armed at rest: waiting for the head's pull */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* the head pulls the strand taut: the burst, and the episode */
    /* a count a tick (1234 µm): taut and moving under command is the ordinary burst — no
     * event; the plunger back at rest ends the episode and the burst */
    steps(&w, 25, 4, 1);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    /* a new episode: burst 2, five counts of its 81 */
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, 5, 4, 1);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_TRUE(!pop(&w, &e));
    /* the strand held: taut, the motor commanded, nothing moves — at a crawl, one pulse a tick
     * (≈ 8 mm/s), where a timed rule would have come first.  The burst started on the episode's
     * 1st tick and the comparator's standstill base moved with every second count after it, the
     * 5th tick the last.  20 mm of motor is 247 pulses (1235 per 100 mm), so the 247th still tick
     * trips: 2.46 s still — nothing, no assist_stall past 2 s; 2.47 s — tangled in assisting */
    steps(&w, 246, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TANGLED);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 231);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(at_an_assists_real_speed_a_taut_still_strand_is_tangled_after_twenty_millimetres_of_motor)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 232, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* the head pulls taut: a burst at 50 mm/s */
    /* 6 pulses a tick is 48.6 mm/s of motor; the head pulling the strand a count a tick.  The
     * burst starts on the 1st tick; the standstill's base moves with every second count after
     * it — the 3rd tick the last — and the 4th adds 6 pulses */
    steps(&w, 4, 6, 1);
    ASSERT_TRUE(w.run[0]);
    /* then the strand held: 247 pulses is 20 mm, 6 + 41 × 6 = 252 (6 + 40 × 6 = 246, 19.9 mm)
     * — tangled 410 ms after the strand stopped, a fifth of the 2 s the removed timed rule
     * waited */
    steps(&w, 40, 6, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 6, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TANGLED);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(24 + 246));
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(runout_and_set_speed_during_an_assist)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 241, w.now),
              ACE2K_FEED_ACCEPTED);
    /* set_speed in the wait — armed at rest, no move running — arms the first burst */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 30000, w.now), 0);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 30000);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 1000, w.now), -ACE2K_EINVAL);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 30000);
    /* set_speed reaches the running burst */
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 40000, w.now), 0);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 40000);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 40000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    /* the insert falling: a runout, idle, not an error */
    steps(&w, 10, 4, 1);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 241);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_reverse_assist_takes_up_on_full_and_stops_at_rest_or_taut)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 251, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    /* armed with the plunger at rest: nothing — and nothing while the strand is taut either; the
     * buffer at rest is the target (decided at the bench) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 252, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING_BACK);
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, 0));
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    steps(&w, 300, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    /* the head pushes strand back — this lane's buffer full: a take-up of 15 mm (12 counts) at
     * the armed speed */
    w.in.pushed = 0x00;
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_ASSIST_BACK_FULL_UM));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 50000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    /* a count back a tick; after 5 the plunger is back at rest: the take-up stops on that tick —
     * no event — and rest starts nothing for 3 s */
    steps(&w, 5, 1, -1);
    ASSERT_TRUE(w.run[0]);
    w.in.rest = 0x0F;
    w.in.pulled_any = false;
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 300, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* full again: take-up 2; its 12th count lands with the plunger still full — the next take-up
     * on that tick, a fresh move from the current count, no event */
    w.in.rest = 0x0E;
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    steps(&w, 11, 1, -1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    step(&w, 1, -1);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w.lane.lane[0].move.enc_start, ace2k_lane_encoder_count(&w.lane, 0));
    ASSERT_TRUE(!pop(&w, &e));
    /* the strand goes taut mid-move — the head holds it: the take-up stops on that tick, and
     * nothing runs while taut, the full reading with it or not */
    w.in.pulled_any = false;
    w.in.pushed = 0x01;
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    w.in.pulled_any = true;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* the plunger between its switches: nothing */
    w.in.pulled_any = false;
    w.in.pushed = 0x00;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    /* a stop disarms with the event: 19 counts taken back over the three take-ups */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING_BACK);
    ASSERT_EQ(e.seq, 252);
    ASSERT_EQ(e.filament_um, -(19 * 12342 / 10));
    ASSERT_TRUE(!pop(&w, &e));
    /* armed with the buffer already full: the take-up at once */
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 30000, 253, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 30000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
}

TEST(the_load_settles_then_pulls_to_the_parking_point_and_a_stop_in_the_settle_cancels)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 161, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x01;
    step(&w, 0, 0); /* auto_load is off in the fixture: the edge starts nothing */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 1000, 162, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, ACE2K_LANE_MOVE_MAX_UM + 1, 0, 163,
                               w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 164, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SETTLE);
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, 0));
    steps(&w, 49, 0, 0); /* 490 ms: the settle, no motion */
    ASSERT_TRUE(!w.run[0]);
    step(&w, 0, 0); /* 500 ms: the pull, the configured 300 mm (243 counts) at 30 mm/s */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_LOAD_PARK_UM));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, ACE2K_FEED_LOAD_SPEED_UM_S);
    /* a count every four ticks, 5 % short of the motor as a healthy lane is: the 243rd count
     * (300 000 / 1 234.2 = 243.07, rounded) on the 972nd tick — 9.7 s, inside the deadline of
     * 300 mm at 30 mm/s (290 mm cruising + the 10 mm landing at 9 mm/s = 10.8 s expected, 17.2 s
     * allowed) — with 3 888 pulses (315 mm) of motor */
    paced(&w, 971, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 164); /* the host's start: its seq */
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(3888));
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* a stop inside the settle: cancelled, nothing ever moved (rule 9) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 100000, 40000, 165, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 10, 0, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 165);
    ASSERT_EQ(e.motor_um, 0);
    steps(&w, 60, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* the command's own distance and speed, a set_speed in the settle, the strand withdrawn
     * during the pull */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 100000, 40000, 166, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.l[0].length_um, 100000);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 40000);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 60000, w.now), 0); /* no move yet: armed */
    ASSERT_TRUE(!w.run[0]);
    steps(&w, 30, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w.lane, 0, 100000));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 60000);
    steps(&w, 20, 4, 1); /* 20 counts of the 81 */
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 166);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(84));
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_buffer_full_during_a_loads_pull_is_watched_and_in_its_settle_nothing)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 171, w.now),
              ACE2K_FEED_ACCEPTED);
    /* the settle: this lane's plunger at its pulled end moves nothing here and is no jam */
    w.in.pulled_any = true;
    w.in.rest = 0x0E;
    steps(&w, 49, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SETTLE);
    ASSERT_TRUE(!pop(&w, &e));
    /* the pull starts on the 50th tick with the buffer still full: as a feed started with the
     * buffer full, the watch begins on that tick (base 0) — the watch, a strand that does not
     * follow: stuck once it lags 5 mm, 64 pulses in — the head is not taking filament */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    steps(&w, 15, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 171);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(64));
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    w.in.pulled_any = false;
    w.in.rest = 0x0F;
    /* a healthy pull, then the buffer full 31 mm in — inside the comparator's grace, which does
     * not cover this reading: the watch, a strand that does not follow: stuck once it lags 5 mm,
     * 64 pulses past the full, with the odometers so far */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 172, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(w.run[0]);
    paced(&w, 100, 4, 4, 1); /* 25 counts (30.9 mm), 400 pulses */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    w.in.pulled_any = true;
    w.in.rest = 0x0E;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    steps(&w, 15, 4, 0);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 172);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(404 + 64));
    ASSERT_EQ(e.filament_um, 25 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    /* the pulled reading is one input for the four lanes: lane 2 loading while lane 1's plunger
     * — assisting, say — holds it, lane 2's own plunger at rest, is no jam on lane 2: its pull
     * runs to loaded (a strand in lane 2: insert bit 1; the fake's counters are lane 1's, so the
     * pull is ended by hand with the lane's stop — the point is that the feed did not end it) */
    w.in.insert = 0x03;
    w.in.rest = 0x0E; /* lane 1's plunger off rest and pulled; lane 2's at rest */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_LOAD, 0, 0, 173, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(w.run[1]);
    steps(&w, 30, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_LOADING);
    ASSERT_TRUE(w.run[1]);
    ASSERT_TRUE(!pop(&w, &e));
    /* lane 2's own plunger leaves rest with the reading still up: its full — the watch begins
     * on that tick.  The watch, a strand that does not follow: stuck once it lags 5 mm — lane 2's
     * tach driven by hand (the fake's step() moves lane 1's counters), 4 pulses a tick with its
     * strand still: 64 pulses past the full, the 16th tick */
    w.in.rest = 0x0C;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_LOADING);
    for (int i = 0; i < 15; i++) {
        w.fg[1] += 4;
        step(&w, 0, 0);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_LOADING);
    w.fg[1] += 4;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 1), 0);
    ASSERT_TRUE(!w.run[1]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.seq, 173);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_automatic_load_starts_on_the_insert_edge_only_when_idle_armed_and_linked)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.link_ok = false;
    step(&w, 0, 0);
    w.in.insert = 0x01; /* no link: nothing */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    w.in.link_ok = true;
    steps(&w, 5, 0, 0); /* the strand still there: no edge, no load */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01; /* the edge: a load, seq 0 — the unit's own */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].seq, 0);
    ASSERT_EQ(w.feed.l[0].length_um, ACE2K_FEED_LOAD_PARK_UM);
    ASSERT_EQ(w.feed.l[0].speed_um_s, ACE2K_FEED_LOAD_SPEED_UM_S);
    ASSERT_TRUE(!w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 0); /* no host start yet */
    /* a lane in another mode does not load, nor does a lane in error */
    struct ace2k_feed_load_cfg cfg = { .park_um = 250000, .speed_um_s = 20000, .auto_load = true };
    ace2k_feed_load_set(&w.feed, &cfg);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    /* a rollback needs no strand: the piece parked past the sensor comes back to the mouth */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 100000, 30000, 181, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.insert = 0x01;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    w.in.rest = 0x0E; /* a tip held ahead: the rollback pulls the strand taut */
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 100000, 30000, 182, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0); /* the strand still, 248 pulses: stuck — an error */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248));
    ASSERT_EQ(e.filament_um, 0);
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    steps(&w, 20, 0, 0); /* the strand in place: the error holds, no load */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(!pop(&w, &e));
    /* the strand pulled out ends the error as a runout (decided at the bench), the lane idle; put back in, the edge loads with load_set's settings — 250 mm is 203
     * counts — and the loaded event is the unit's own (seq 0) while the report keeps the last
     * host start's seq */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 182);
    /* the odometers since the failed mode began: the error's own, 248 pulses, no strand */
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248));
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].length_um, 250000);
    ASSERT_EQ(w.feed.l[0].speed_um_s, 20000);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 182);
    steps(&w, 50, 0, 0); /* the settle */
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w.lane, 0, 250000));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 20000);
    paced(&w, 812, 4, 4, 1); /* the 203rd count on the 812th tick: done */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(e.filament_um, 203 * 12342 / 10);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 182);
    ASSERT_TRUE(!pop(&w, &e));
    /* auto_load off: the edge starts nothing */
    w.feed.load.auto_load = false;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
}

/* The grip. */
TEST(the_grip_takes_its_bounds_and_zero_clears_it)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ACE2K_FEED_GRIP_UM_MAX, 45000);
    ASSERT_TRUE(ACE2K_FEED_GRIP_UM_MAX < ACE2K_FEED_LOAD_GRACE_UM); /* in_load_grace is strict */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, ACE2K_FEED_GRIP_UM), 0);
    ASSERT_EQ(w.feed.grip_um[0], ACE2K_FEED_GRIP_UM);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 1, ACE2K_FEED_GRIP_UM_MIN), 0);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 2, ACE2K_FEED_GRIP_UM_MAX), 0);
    ASSERT_EQ(w.feed.grip_um[1], ACE2K_FEED_GRIP_UM_MIN);
    ASSERT_EQ(w.feed.grip_um[2], ACE2K_FEED_GRIP_UM_MAX);
    /* out of bounds: refused, nothing taken */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, ACE2K_FEED_GRIP_UM_MIN - 1), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, ACE2K_FEED_GRIP_UM_MAX + 1), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, ACE2K_LANE_COUNT, ACE2K_FEED_GRIP_UM), -ACE2K_EINVAL);
    ASSERT_EQ(w.feed.grip_um[0], ACE2K_FEED_GRIP_UM);
    /* 0 clears: the next automatic load is the plain one */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 0), 0);
    ASSERT_EQ(w.feed.grip_um[0], 0);
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].length_um, ACE2K_FEED_LOAD_PARK_UM);
    ASSERT_TRUE(!w.feed.l[0].gripped);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(a_gripped_automatic_load_pulls_the_grip_alone_ends_loaded_and_clears_it)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000); /* a search configured: the grip skips it */
    w.feed.load.auto_load = true;
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].seq, 0);
    ASSERT_EQ(w.feed.l[0].length_um, 40000);
    ASSERT_EQ(w.feed.l[0].speed_um_s, ACE2K_FEED_LOAD_SPEED_UM_S);
    ASSERT_TRUE(w.feed.l[0].gripped);
    ASSERT_EQ(w.feed.grip_um[0], 0); /* consumed by this load */
    steps(&w, 50, 0, 0);             /* the settle */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w.lane, 0, 40000));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, ACE2K_FEED_LOAD_SPEED_UM_S);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_LOADING && n < 400) {
        step(&w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    /* no search past it: idle at the grip, the motor stopped */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 0);
    ASSERT_TRUE(e.filament_um >= 40000 - 1235 && e.filament_um < 40000 + 1235);
    ASSERT_TRUE(!pop(&w, &e));
    /* the next insert edge is a plain load: the park point and the search */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].length_um, ACE2K_FEED_LOAD_PARK_UM);
    ASSERT_TRUE(!w.feed.l[0].gripped);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* An automatic load with a 40 mm grip, past its settle and 10 counts (12.3 mm) into the pull. */
static void grip_pull_started(struct world *w)
{
    ace2k_feed_search_set(&w->feed, 600000);
    w->feed.load.auto_load = true;
    ASSERT_EQ(ace2k_feed_grip_set(&w->feed, 0, 40000), 0);
    w->in.insert = 0x00;
    step(w, 0, 0);
    w->in.insert = 0x01;
    step(w, 0, 0);
    steps(w, 50, 0, 0);
    paced(w, 40, 4, 4, 1);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w->feed.l[0].gripped);
}

TEST(a_read_session_during_a_gripped_pull_resumes_to_the_grip_and_never_searches)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    grip_pull_started(&w);
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
    ASSERT_TRUE(!w.run[0]);
    steps(&w, 30, 0, 0);
    /* released, no tag read: what is left of the grip, nothing past it */
    w.in.tag_hold = 0;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, 40000 - (10 * 12342 / 10)));
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_LOADING && n < 400) {
        ASSERT_TRUE(w.feed.l[0].sub != ACE2K_FEED_SUB_SEARCH);
        step(&w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_TRUE(e.filament_um >= 40000 - 1235 && e.filament_um < 40000 + 1235);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(an_obstacle_inside_the_grip_ends_the_gripped_load_blocked)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    grip_pull_started(&w);
    /* this lane's buffer full and the strand still: the watch, blocked once it lags 5 mm */
    w.in.pulled_any = true;
    w.in.rest = 0x0E;
    step(&w, 4, 0);
    steps(&w, 15, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(e.filament_um, 10 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0));
    /* nothing more: no search, no return */
    w.in.pulled_any = false;
    w.in.rest = 0x0F;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_host_start_clears_the_grip)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a feed */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 191, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.grip_um[0], 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* a host load: the park point, never the grip */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 192, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.grip_um[0], 0);
    ASSERT_EQ(w.feed.l[0].length_um, ACE2K_FEED_LOAD_PARK_UM);
    ASSERT_TRUE(!w.feed.l[0].gripped);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* a read with motion */
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.grip_um[0], 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    while (pop(&w, &e)) {
    }
    /* a refused start leaves it; another lane's start leaves it */
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 193, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    ASSERT_EQ(w.feed.grip_um[0], 40000);
    w.in.link_ok = true;
    w.in.insert = 0x03;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 100000, 30000, 194, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.grip_um[0], 40000);
    ace2k_feed_stop(&w.feed, 1, w.now);
}

TEST(a_retried_host_start_clears_the_grip_too)
{
    /* the transport's retry of a start is still a host start: the grip the host set since then
     * ends with it, as the host believes */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, 1); /* 10 mm is 8 counts: done */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 40000), 0);
    steps(&w, 50, 0, 0); /* 500 ms after the done: inside the retry window */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 9, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE); /* the retry: nothing started */
    ASSERT_EQ(w.feed.grip_um[0], 0);
}

TEST(the_link_down_ends_a_waiting_assist_and_a_settling_load_and_starts_no_burst)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a burst running (armed taut): the lane core ends it, the feed maps it — stopped_link, idle */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 191, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    steps(&w, 5, 4, 1);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.link_ok = false;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 191);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 192, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    /* a waiting assist (armed at rest) — no move for the lane core to end: the feed ends it
     * itself, and taut with the link down starts no burst */
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 193, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    w.in.link_ok = false;
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* taut: a burst — with a host */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_EQ(e.seq, 193);
    steps(&w, 250, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.pushed = 0x00;
    w.in.link_ok = true;
    step(&w, 0, 0);
    /* a load in its settle: stopped_link, nothing ever moved */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 194, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 10, 0, 0);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 194);
    ASSERT_EQ(e.motor_um, 0);
    steps(&w, 60, 0, 0);
    ASSERT_TRUE(!w.run[0]); /* the pull never came */
    ASSERT_TRUE(!pop(&w, &e));
    w.in.link_ok = true;
    step(&w, 0, 0);
    /* the reverse assist waiting on taut: the same */
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 195, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    w.in.link_ok = false;
    w.in.pushed = 0x00; /* rest: a creep — with a host */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING_BACK);
    ASSERT_EQ(e.seq, 195);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_reverse_standstill_at_rest_is_the_tail_past_the_drive_and_the_move_goes_on)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* an unload whose strand follows for 41 counts (50.6 mm) and then stands still with the
     * plunger at rest: the tail has left the drive (measured on the unit, on lane 2).  The standstill's base is the 40th count, tick 160, 640 pulses (the 41st
     * is one count past it, under the 1500 µm floor); it trips 248 pulses later, 888, the 222nd
     * tick — no error: the motor keeps turning for the spool to wind the tail out */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 201, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 164, 4, 4, -1);
    ASSERT_EQ(w.feed.l[0].fg_still, 640);
    steps(&w, 57, 4, 0); /* 884 pulses: 244 since the base, not yet */
    ASSERT_TRUE(!w.feed.l[0].tail_out);
    step(&w, 4, 0);
    ASSERT_TRUE(w.feed.l[0].tail_out);
    ASSERT_EQ(w.feed.l[0].fg_tail, 888);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* 60 mm of motor later the insert clears: unloaded, as ever */
    steps(&w, 185, 4, 0); /* 740 pulses, 59.9 mm: still turning */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0]);
    w.in.insert = 0x00;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 201);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(1632));
    ASSERT_EQ(e.filament_um, -(41 * 12342 / 10));
    ASSERT_TRUE(!pop(&w, &e));
    /* the strand never moves and the insert stays: the standstill at 248 pulses, then the tail
     * budget — 100 mm is 1235 pulses, 1236 on the 309th tick after the trip — ends the move
     * unload_incomplete, an error like the budget spent: the strand is still at the sensor and
     * nothing is left to drive, a hand is needed */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 202, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_TRUE(w.feed.l[0].tail_out);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    steps(&w, 308, 4, 0); /* 1232 pulses since the trip, 99.8 mm: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0]);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.duty[0], 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 202);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248 + 1236));
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(!w.feed.l[0].tail_out); /* the mode's state went with the mode */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 203, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* nothing of the tail outlives the mode: 20 idle ticks with the motor's counter still turning
     * (the fake's) send no event, and a move the lane core runs for someone else on this lane is
     * left alone — it is not the feed's to end */
    steps(&w, 20, 4, 0);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_lane_move(&w.lane, 0, ACE2K_LANE_REVERSE, 500000, 30000, w.now), 0);
    steps(&w, 100, 4, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ace2k_lane_stop(&w.lane, 0, w.now);
    step(&w, 0, 0);
    ASSERT_TRUE(!pop(&w, &e));
    /* a rollback, the same two ways: the insert falling is its runout; the budget spent, the
     * same unload_incomplete */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 111, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_TRUE(w.feed.l[0].tail_out);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    steps(&w, 185, 4, 0);
    ASSERT_TRUE(w.run[0]);
    w.in.insert = 0x00;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 111);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 112, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    steps(&w, 308, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 112);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* between its switches the plunger says nothing: the reading stays the conservative one,
     * stuck — as it does taut (the tests above) */
    w.in.rest = 0x0E;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 113, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(e.seq, 113);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the partial trigger is not re-read from the plunger: toward the spool, short is stuck.  A
     * count back every 8 ticks — 19 counts (23.4 mm) by the window's close at 620 pulses
     * (50.2 mm), the 155th tick: short by 27 mm; the standstill's base moves with every two
     * counts (16 ticks, 64 pulses, 5.2 mm), so it never trips first.  The odometer is the
     * difference of two truncated readings, as the encoder did not start this move at zero */
    w.in.rest = 0x0F;
    int32_t enc_um_before = ace2k_lane_encoder_um(&w.lane, 0);
    int32_t enc_before = ace2k_lane_encoder_count(&w.lane, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 500000, 30000, 114, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 154, 4, 8, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(!w.feed.l[0].tail_out);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(620));
    ASSERT_EQ(e.filament_um, ace2k_lane_encoder_um(&w.lane, 0) - enc_um_before);
    ASSERT_EQ(ace2k_lane_encoder_count(&w.lane, 0) - enc_before, -19);
}

TEST(a_take_up_the_strand_does_not_follow_is_stuck_or_a_timeout_unless_the_plunger_is_back_at_rest)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* armed full: the take-up at once.  The strand does not follow and the plunger stays full:
     * 248 pulses ≥ 20 mm of motor with the strand still — the standstill toward the spool with the
     * plunger off rest is stuck, as in every reverse mode */
    w.in.rest = 0x0E;
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 121, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    steps(&w, 61, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING_BACK);
    ASSERT_EQ(e.seq, 121);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the take-up's own deadline — 15 mm at 50 mm/s: 5 mm at speed (100 ms) and the 10 mm landing
     * at the floor (1111 ms), 1211 ms expected, 1211 × 3 / 2 + 1000 = 2816 ms allowed, the 282nd
     * tick — with the motor turning a pulse every two ticks (140 pulses, 11.3 mm: under the
     * standstill's 20 mm, never a silent second) and the plunger still full: a timeout, an error */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 122, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    crawl(&w, 281, 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING_BACK);
    ASSERT_TRUE(w.run[0]);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TIMEOUT);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TIMEOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING_BACK);
    ASSERT_EQ(e.seq, 122);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the same deadline on the tick the plunger comes back to rest: the tail rule — the move has
     * ended, the assist waits, no error, no event */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 123, w.now),
              ACE2K_FEED_ACCEPTED);
    crawl(&w, 281, 2);
    w.in.rest = 0x0F;
    w.in.pulled_any = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING_BACK);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
}

TEST(a_reverse_deadline_with_the_plunger_at_rest_is_the_tail_too_and_the_mode_goes_on)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* an unload with a 30 mm budget at 30 mm/s: 20 mm at speed and the 10 mm landing at the floor
     * are 1777 ms expected, 3665 ms allowed — the 367th tick.  A pulse every two ticks is 183
     * pulses (14.8 mm) by then: under the standstill's 20 mm, and never a silent second for the
     * lane's own stall check.  The deadline with the plunger at rest is the tail past the drive:
     * no error, a fresh bounded move for the tail budget, and the insert clearing 60 mm of motor
     * later ends it unloaded — the mode's odometer spans both moves */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 30000, 30000, 61, w.now),
              ACE2K_FEED_ACCEPTED);
    crawl(&w, 366, 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(!w.feed.l[0].tail_out);
    ASSERT_TRUE(w.run[0]);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.feed.l[0].tail_out);
    ASSERT_TRUE(w.run[0] && w.reverse[0]); /* the tail's own move */
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_UNLOAD_TAIL_UM));
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 30000);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 185, 4, 0); /* 740 pulses, 59.9 mm of the tail budget */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0]);
    w.in.insert = 0x00;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 61);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(183 + 740 + 4));
    ASSERT_TRUE(!pop(&w, &e));
    /* the tail begun by the standstill (248 pulses, the 62nd tick) and the deadline falling inside
     * it (the 367th): the mode continues on a move for what the budget still allows — 1236 − 1220
     * pulses — and the budget ends it unload_incomplete four ticks later */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 30000, 30000, 62, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 62, 4, 0);
    ASSERT_TRUE(w.feed.l[0].tail_out);
    steps(&w, 304, 4, 0); /* the 366th tick: 1464 pulses, 1216 since the trip */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w.lane, 0, 30000));
    step(&w, 4, 0); /* the deadline: 1220 since the trip, 98.8 mm — the tail's move for 1.2 mm */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, 1);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 3, 4, 0); /* 1232 since the trip: not yet */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    step(&w, 4, 0); /* 1236 */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOAD_INCOMPLETE);
    ASSERT_EQ(e.seq, 62);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(248 + 1236));
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the same deadline with the strand taut is a timeout, as ever */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 30000, 30000, 63, w.now),
              ACE2K_FEED_ACCEPTED);
    crawl(&w, 367, 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TIMEOUT);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TIMEOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
}

TEST(a_long_taut_wait_with_the_motor_off_is_no_fault_and_the_next_burst_is_judged_afresh)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* armed taut: the first burst at once */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 71, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    /* another lane's full stops the burst (the shared reading); this lane's strand stays taut
     * for 5 s with the motor off — the head is pulling what the buffer holds */
    w.in.pulled_any = true;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    steps(&w, 500, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* the full gone: taut restarts a burst, whose comparator bases are its own start's — not
     * 5 s of taut wait.  Still and taut under command, one pulse a tick: the standstill's 20 mm
     * (247 pulses) — nothing on the 246th tick, tangled on the 247th, never assist_stall */
    w.in.pulled_any = false;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    steps(&w, 246, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TANGLED);
    ASSERT_EQ(e.seq, 71);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_bursts_bound_reached_at_rest_or_full_leaves_the_lane_waiting)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 81, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 323, 4, 4, 1); /* 80 counts of the burst's 81 */
    ASSERT_TRUE(!pop(&w, &e));
    /* the 81st count lands as the plunger comes back to rest: the bound ends the burst and rest
     * starts no other — waiting, no event, no notice: the head did not outrun the burst */
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_TRUE(!pop(&w, &e));
    /* taut: burst 2; its bound lands with the plunger at its pulled end (full) — waiting */
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    paced(&w, 323, 4, 4, 1);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.pushed = 0x00;
    w.in.pulled_any = true;
    step(&w, 4, 1); /* the bound with the plunger at its pulled end: no notice either */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_TRUE(!pop(&w, &e));
    /* the full gone and taut: burst 3 */
    w.in.pulled_any = false;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
}

TEST(an_insert_edge_on_the_tick_a_mode_ends_does_not_start_the_automatic_load)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    /* a rollback bringing a piece parked past the sensor back to the mouth: 10 mm is 8 counts; the
     * 8th count and the insert rising land on the same tick — the rollback's done, and no load:
     * the edge was that mode's, not an idle lane's */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 10000, 30000, 91, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 28, 4, 4, -1); /* 7 counts */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    w.in.insert = 0x01;
    step(&w, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    steps(&w, 60, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* an edge on a lane idle through the tick still loads */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].seq, 0);
}

TEST(a_move_the_lane_core_runs_for_someone_else_makes_every_start_busy)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_lane_move(&w.lane, 0, ACE2K_LANE_FORWARD, 500000, 30000, w.now), 0);
    steps(&w, 5, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 101, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 102, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 103, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 104, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(w.run[0]); /* the foreign move untouched */
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 0); /* a refusal records no start */
    ace2k_lane_stop(&w.lane, 0, w.now);
    step(&w, 0, 0);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 101, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING); /* waiting at rest */
}

TEST(arming_an_assist_reads_the_buffer_as_the_tick_does)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* reverse: the shared pulled reading with this lane's plunger at rest is another lane's full —
     * armed, nothing moves; 50 ticks the same */
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 131, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING_BACK);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* this lane's plunger off rest and not taut with the reading up: its full — the take-up at
     * once, 12 counts at the armed speed */
    w.in.rest = 0x0E;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 132, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_ASSIST_BACK_FULL_UM));
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* taut with the reading up is not full either: nothing */
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 133, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* forward: taut with the shared reading up waits, as the tick's own start does — any lane's
     * full holds a burst (the shared input); the reading gone, the burst
     * at once */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 134, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    w.in.pulled_any = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 135, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_TRUE(!pop(&w, &e));
}

/* The follow: lane 0's three plunger readings for its tests. */
static void follow_rest(struct world *w)
{
    w->in.rest = 0x0F;
    w->in.pushed = 0x00;
    w->in.pulled_any = false;
}

static void follow_taut(struct world *w)
{
    w->in.rest = 0x0E;
    w->in.pushed = 0x01;
    w->in.pulled_any = false;
}

/* this lane's plunger at its pulled end: off rest, not taut, the shared reading up */
static void follow_full(struct world *w)
{
    w->in.rest = 0x0E;
    w->in.pushed = 0x00;
    w->in.pulled_any = true;
}

static void assert_burst_running(struct world *w)
{
    ASSERT_TRUE(w->run[0] && !w->reverse[0]);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w->lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w->lane, 0, ACE2K_FEED_ASSIST_BURST_UM));
}

static void assert_take_up_running(struct world *w, uint32_t take_um)
{
    ASSERT_TRUE(w->run[0] && w->reverse[0]);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w->lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w->lane, 0, take_um));
}

TEST(the_follow_bursts_on_taut_and_takes_up_on_full_each_ended_at_rest)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 41, w.now),
              ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 1000, 42, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    /* armed at rest: the follow, waiting — rest is the target, and nothing starts on it */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 43, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(ACE2K_FEED_FOLLOWING, 8);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 44, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    steps(&w, 300, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    /* taut: a forward burst at the armed speed, bounded at 100 mm */
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 50000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    steps(&w, 5, 4, 1);
    /* rest ends it on that tick: no event, the lane waits */
    follow_rest(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!pop(&w, &e));
    /* this lane's full — the head pushed strand back: a take-up of 15 mm at once (rest was read
     * since the burst), toward the spool */
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 50000);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    steps(&w, 5, 1, -1);
    /* rest ends it: no event */
    follow_rest(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 100, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* full again, then taut mid-move — the head holds the strand: the take-up ends on that tick */
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    steps(&w, 3, 1, -1);
    follow_taut(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    /* a stop disarms with the event, mode following: 6 counts forward, 9 back */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 43);
    ASSERT_EQ(e.filament_um, (6 - 10) * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_follow_does_not_flip_without_rest_or_the_flip_time)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 51, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    assert_burst_running(&w);
    /* the shared full with this lane's plunger at its pulled end ends the burst — the safety —
     * and the plunger never passes rest: no take-up at +10 … +190 ms, one at +200 ms */
    follow_full(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    /* the mirror: the take-up ended by taut, no burst before 200 ms, one at 200 ms */
    steps(&w, 3, 1, -1);
    follow_taut(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    /* a pass through rest allows the flip at once: the burst ended by the full, one tick at rest,
     * then this lane's full — the take-up on that tick */
    steps(&w, 3, 4, 1);
    follow_full(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 4);
    /* and the move that ends at rest is itself the pass: taut at once after it */
    steps(&w, 3, 1, -1);
    follow_rest(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 5);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_follow_chains_in_the_same_direction_at_once)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* armed taut: the burst at once; its 81st count on the 324th tick with the strand still taut
     * — the next burst on that tick, and the behind notice once for the episode */
    follow_taut(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 61, w.now),
              ACE2K_FEED_ACCEPTED);
    assert_burst_running(&w);
    paced(&w, 323, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 4, 1);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_EQ(w.lane.lane[0].move.enc_start, ace2k_lane_encoder_count(&w.lane, 0));
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BEHIND);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 61);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(1296));
    ASSERT_TRUE(!pop(&w, &e));
    paced(&w, 323, 4, 4, 1);
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 3);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e)); /* the same episode: no second notice */
    /* rest; then this lane's full: a take-up whose 12th count lands still full — the next take-up
     * on that tick, from the current count, no event */
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 4);
    steps(&w, 11, 1, -1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 4);
    step(&w, 1, -1);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 5);
    ASSERT_EQ(w.lane.lane[0].move.enc_start, ace2k_lane_encoder_count(&w.lane, 0));
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
}

TEST(another_lanes_full_starts_no_take_up_and_pauses_the_follows_bursts)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 71, w.now),
              ACE2K_FEED_ACCEPTED);
    /* the shared reading up with this lane's plunger at rest: another lane's full — nothing */
    w.in.pulled_any = true;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* this lane taut with it: no burst while it lasts, and no take-up either; gone, the burst */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0);
    w.in.pulled_any = false;
    step(&w, 0, 0);
    assert_burst_running(&w);
    /* another lane's full during the burst pauses it — the safety — and holds the next */
    steps(&w, 3, 4, 1);
    w.in.pulled_any = true;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    w.in.pulled_any = false;
    step(&w, 0, 0);
    assert_burst_running(&w); /* the same direction: at once */
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

/* The shared pulled-end reading is one wired line for the four lanes: it cannot name the lane at
 * its pulled end.  With this lane's plunger mid-travel (neither at rest nor taut) another lane's
 * full reads exactly as this lane's own, so it may start one take-up here — bounded, and ended
 * when this lane reads taut.  A lane at rest or taut is immune (the test above). */
TEST(another_lanes_full_may_start_one_take_up_on_a_lane_mid_travel)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 75, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    assert_burst_running(&w);
    /* this lane's plunger leaves taut for mid-travel as another lane's full comes up: the burst
     * ends on the shared reading, and the follow's rule 1 holds the take-up until the flip time */
    w.in.rest = 0x0E;
    w.in.pushed = 0x00;
    w.in.pulled_any = true;
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    /* this lane reads taut, the other lane still full: the take-up ends, and no move follows
     * while both last — exactly one take-up, no event */
    steps(&w, 3, 1, -1);
    w.in.pushed = 0x01;
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_follows_comparator_reads_the_running_moves_direction)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* Two of the comparator's verdicts cannot be reached through the tick, here as in the two
     * assists: a forward trip with this lane's full (stuck) — the follow's step ends the burst on
     * the shared full before the comparator reads that tick — and a take-up's standstill at rest
     * — the step ends the take-up on rest first.  The reachable equivalent of the latter is the
     * take-up's deadline on the tick the plunger is back at rest, tested below.
     * forward: the strand held taut under a burst — the standstill without this lane's full is
     * tangled, as in the forward assist (the burst on the 1st tick, the base's last move on the
     * 5th; 247 pulses of motor still) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 81, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    assert_burst_running(&w);
    steps(&w, 246, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 1, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TANGLED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 81);
    /* in error: the follow is refused as every start is, until clear */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 85, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* toward the spool: a take-up whose strand does not follow, the plunger still full — the
     * standstill off rest is stuck, as in every reverse mode */
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 82, w.now),
              ACE2K_FEED_ACCEPTED);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    steps(&w, 61, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 82);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the take-up's deadline with the plunger still full: a timeout (15 mm at 50 mm/s, the 282nd
     * tick, the motor crawling under the standstill) */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 83, w.now),
              ACE2K_FEED_ACCEPTED);
    crawl(&w, 281, 2);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TIMEOUT);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TIMEOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the same deadline on the tick the plunger is back at rest: the tail at rest — the take-up
     * ends, the lane waits, no error, no event; the next reading starts the next move */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 84, w.now),
              ACE2K_FEED_ACCEPTED);
    crawl(&w, 281, 2);
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_tail_stop_link_and_shutdown_in_the_follow)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* the insert falling during a burst: no runout — the tail, a notice, the follow goes on;
     * the stop ends it */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 91, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 91);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert = 0x01;
    follow_rest(&w);
    step(&w, 0, 0);
    /* the link down during a take-up: the lane core ends it, stopped_link */
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 92, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    steps(&w, 3, 1, -1);
    w.in.link_ok = false;
    step(&w, 1, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 92);
    ASSERT_TRUE(!pop(&w, &e));
    /* the link down while waiting: the feed ends it itself, and no move starts */
    w.in.link_ok = true;
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 93, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.link_ok = false;
    follow_taut(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 93);
    ASSERT_TRUE(!pop(&w, &e));
    /* a stop while a take-up runs */
    w.in.link_ok = true;
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 94, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 94);
    ASSERT_TRUE(!pop(&w, &e));
    /* a shutdown while waiting disarms it; nothing moves after, taut or full */
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 95, w.now),
              ACE2K_FEED_ACCEPTED);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_stopped_shutdown(&w, ACE2K_FEED_FOLLOWING, 95);
    follow_taut(&w);
    nothing_moves_after_the_shutdown(&w, 50);
    follow_full(&w);
    nothing_moves_after_the_shutdown(&w, 50);
}

TEST(arming_the_follow_reads_the_buffer_as_the_tick_does)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* armed taut: the burst at once */
    follow_taut(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 101, w.now),
              ACE2K_FEED_ACCEPTED);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* armed full: the take-up at once */
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 102, w.now),
              ACE2K_FEED_ACCEPTED);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* armed at rest: waits */
    follow_rest(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 103, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* armed taut with another lane's full: waits, as the tick's own step does */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    w.in.pulled_any = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 104, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_follow_is_an_assist_for_busy_and_the_leds)
{
    struct world w;
    world_init(&w);
    w.in.insert = 0x03;
    ace2k_feed_search_set(&w.feed, ACE2K_FEED_SEARCH_DEFAULT_UM);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 111, w.now),
              ACE2K_FEED_ACCEPTED);
    /* waiting with no move running, the lane is still busy: every start, the read with motion on
     * it or (the tag search's rule 4) on another lane, and the counters reset's veto read it */
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, 0));
    ASSERT_TRUE(ace2k_feed_lane_busy(&w.feed, ACE2K_LANE_ALL));
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, 1));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 112, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 1, w.now), ACE2K_FEED_REFUSED_OTHER_LANE);
    /* a speed armed in the wait reaches the next move */
    ASSERT_EQ(ace2k_feed_set_speed(&w.feed, 0, 30000, w.now), 0);
    follow_taut(&w);
    step(&w, 0, 0);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 30000);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(!ace2k_feed_lane_busy(&w.feed, 0));
    /* the LEDs' pattern is the binding's (feed_cmds.c), not reachable from a host test: it maps
     * FOLLOWING to ASSIST beside the two assists */
}

TEST(the_follows_values_are_set_at_run_time_within_bounds)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.follow.flip_ms, ACE2K_FEED_FOLLOW_FLIP_MS);
    ASSERT_EQ(w.feed.follow.take_um, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ACE2K_FEED_FOLLOW_FLIP_MS, 200);
    ASSERT_EQ(ACE2K_FEED_FOLLOW_TAKE_UM, 15000);
    const struct ace2k_feed_follow_cfg low = {
        .flip_ms = ACE2K_FEED_FOLLOW_FLIP_MS_MIN,
        .take_um = ACE2K_FEED_FOLLOW_TAKE_UM_MIN,
        .tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MIN,
    };
    const struct ace2k_feed_follow_cfg high = {
        .flip_ms = ACE2K_FEED_FOLLOW_FLIP_MS_MAX,
        .take_um = ACE2K_FEED_FOLLOW_TAKE_UM_MAX,
        .tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MAX,
    };
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &low), 0);
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &high), 0);
    struct ace2k_feed_follow_cfg bad;
    bad = high;
    bad.flip_ms = ACE2K_FEED_FOLLOW_FLIP_MS_MAX + 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.flip_ms = ACE2K_FEED_FOLLOW_FLIP_MS_MIN - 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.take_um = ACE2K_FEED_FOLLOW_TAKE_UM_MAX + 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.take_um = ACE2K_FEED_FOLLOW_TAKE_UM_MIN - 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &bad), -ACE2K_EINVAL);
    /* the last good set stands, whole */
    ASSERT_EQ(w.feed.follow.flip_ms, high.flip_ms);
    ASSERT_EQ(w.feed.follow.take_um, high.take_um);
    /* 500 ms and 20 mm, set while the follow is armed: read by the next move and the next flip */
    const struct ace2k_feed_follow_cfg mid = {
        .flip_ms = 500,
        .take_um = 20000,
        .tail_um = ACE2K_FEED_FOLLOW_TAIL_UM,
    };
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &mid), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 121, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    assert_burst_running(&w);
    follow_full(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 49; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    step(&w, 0, 0);
    assert_take_up_running(&w, 20000);
    /* the reverse assist keeps its own 15 mm */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 122, w.now),
              ACE2K_FEED_ACCEPTED);
    assert_take_up_running(&w, ACE2K_FEED_ASSIST_BACK_FULL_UM);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* The feed-forward: the follow armed on lane 0 at 50 mm/s, a
 * base rate metered out in doses of 3 mm at 15 mm/s — 3 mm is two counts at 1234.2 µm a count. */
static void ff_arm(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
}

/* lane 0 at rest under another lane's full: the doses paused, the debt free to grow */
static void ff_paused_at_rest(struct world *w)
{
    w->in.rest = 0x0F;
    w->in.pushed = 0x00;
    w->in.pulled_any = true;
}

static void assert_dose_running(struct world *w, uint32_t chunk_um, uint32_t pulse_um_s)
{
    ASSERT_TRUE(w->run[0] && !w->reverse[0]);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w->feed.l[0].dose);
    ASSERT_EQ(w->lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w->lane, 0, chunk_um));
    ASSERT_EQ(w->lane.lane[0].move.speed_um_s, pulse_um_s);
}

/* ticks until the motor runs, at most `bound`: the number taken */
static int until_running(struct world *w, int bound)
{
    int n = 0;
    while (!w->run[0] && n < bound) {
        step(w, 0, 0);
        n++;
    }
    return n;
}

TEST(the_base_rate_accrues_a_capped_debt_with_its_remainder)
{
    struct world w;
    world_init(&w);
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 141);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    /* 0.3 mm/s: 3 µm a tick, 3 mm owed after 10 s — and no dose under the shared full */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    steps(&w, 999, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 2997);
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    /* 0 stops the accrual: what is owed is kept */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    steps(&w, 10, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    /* 1.234 mm/s is 12.34 µm a tick: one second owes 1234 µm, not the 1200 of a truncated tick */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 1234, false), 0);
    steps(&w, 100, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000U + 1234U);
    /* 50 mm/s: 500 µm a tick, capped at two chunks */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 100, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 2U * ACE2K_FEED_FF_CHUNK_UM);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    ASSERT_TRUE(!w.run[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_debt_of_one_chunk_starts_a_dose_at_the_pulse_speed_even_at_rest)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_arm(&w, 142);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    steps(&w, 999, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* the 1000th tick owes 3 mm: a dose at rest — metering runs ahead of the pull */
    step(&w, 0, 0);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 0); /* a dose is no burst */
    int32_t enc0 = ace2k_lane_encoder_um(&w.lane, 0);
    /* rest does not end a dose: the strand follows its two counts */
    step(&w, 4, 0);
    step(&w, 4, 1);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    uint32_t owed = w.feed.l[0].debt_um;
    ASSERT_EQ(owed, 3006);
    step(&w, 4, 1);
    /* the chunk reached: the debt drops by the strand's travel, then this tick's 3 µm */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(!w.feed.l[0].dose);
    int32_t travel = ace2k_lane_encoder_um(&w.lane, 0) - enc0;
    ASSERT_EQ(travel, 2 * 12342 / 10);
    ASSERT_EQ(w.feed.l[0].debt_um, owed - (uint32_t)travel + 3U);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!pop(&w, &e));
    /* a strand pulled ahead during a dose never raises the debt: the drop floors at 0 */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 10), 5); /* 541 µm owed, 500 a tick */
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    step(&w, 4, 10);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, 500);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 2);
    ASSERT_TRUE(!pop(&w, &e));
}

/* A rate of 0 stops the accrual, not the metering: the strand
 * the head has already pulled is still owed — a travel or a retract between two extrusions sends
 * 0, and dropping the debt there lost up to a chunk at every gap. */
TEST(a_rate_of_0_keeps_the_debt_and_a_dose_still_pays_it)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 160);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    steps(&w, 1000, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    /* 0: the debt kept, nothing more accrues, and no dose while the shared full lasts */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    steps(&w, 50, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    /* the shared full gone: a dose at base 0, one chunk at the pulse speed */
    w.in.pulled_any = false;
    step(&w, 0, 0);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    step(&w, 4, 0);
    step(&w, 4, 1);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(w.feed.l[0].debt_um, 3000); /* no accrual at 0 */
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].dose);
    /* the residual under a chunk stays owed: no partial dose, however long the 0 lasts */
    uint32_t residual = 3000U - (uint32_t)(2 * 12342 / 10);
    ASSERT_EQ(w.feed.l[0].debt_um, residual);
    ASSERT_EQ(residual, 532);
    steps(&w, 500, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, residual);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    /* a 0 sent again changes nothing; the next accrual adds to the residual, paid with the next
     * dose: 532 + 3 × 823 µm reaches the chunk on the 823rd tick */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, residual);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    ASSERT_EQ(until_running(&w, 1000), 823);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(w.feed.l[0].debt_um, residual + (3U * 823U));
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 2);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* A debt kept across a 0 is still cleared by every end of the follow, and by taut and this
 * lane's full — the follow's own move correcting the buffer. */
TEST(a_debt_kept_at_rate_0_is_cleared_by_disarm_taut_and_full)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    /* the stop */
    ff_arm(&w, 161);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 1234, false), 0);
    steps(&w, 150, 0, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 1851);
    ASSERT_EQ(w.feed.l[0].debt_rem, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(w.feed.l[0].debt_rem, 0);
    /* the link lost */
    ff_arm(&w, 162);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 1234, false), 0);
    steps(&w, 101, 0, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 1246);
    ASSERT_EQ(w.feed.l[0].debt_rem, 340);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(w.feed.l[0].debt_rem, 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    /* taut, at base 0: the burst, the debt to 0 */
    ff_arm(&w, 163);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    w.in.pulled_any = false;
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* this lane's full, at base 0: no dose, the debt to 0 */
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 164);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_TRUE(!w.feed.l[0].dose);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_TRUE(!pop(&w, &e));
}

/* The clear (the host's, for a lane remapped to another extruder or one of two lanes following
 * one extruder): the base set and what is owed dropped, the sub-µm remainder with it; without
 * it the debt is kept.  Refused outside the follow, as the base: nothing taken. */
TEST(the_clear_drops_the_debt_and_its_remainder_without_it_both_are_kept)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 165);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 1234, false), 0);
    steps(&w, 101, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 1246);
    ASSERT_EQ(w.feed.l[0].debt_rem, 340);
    /* clear 0 at 0, and at a rate: both kept */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 1246);
    ASSERT_EQ(w.feed.l[0].debt_rem, 340);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 1234, false), 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 1246);
    ASSERT_EQ(w.feed.l[0].debt_rem, 340);
    /* clear 1 at 0: the base 0, nothing owed, nothing accrues */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, true), 0);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(w.feed.l[0].debt_rem, 0);
    steps(&w, 50, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    /* clear 1 with a rate: the rate taken, accruing from nothing */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, true), 0);
    ASSERT_EQ(w.feed.l[0].base_um_s, 300);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 3);
    /* a cleared debt is never paid: the shared full gone at 0, no dose */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, true), 0);
    w.in.pulled_any = false;
    steps(&w, 20, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    /* outside the follow: refused, nothing taken */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 0, true), -ACE2K_EREFUSED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(no_dose_while_taut_full_shared_full_or_a_move_runs)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* the shared full: the debt at its cap, no dose */
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 143);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    ASSERT_TRUE(!w.run[0]);
    /* taut: the follow's burst, no dose */
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_TRUE(!w.feed.l[0].dose);
    /* the plunger mid-travel under the burst: the debt grows back to its cap, no dose while the
     * burst runs */
    w.in.rest = 0x0E;
    w.in.pushed = 0x00;
    steps(&w, 12, 4, 1);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    assert_burst_running(&w);
    /* this lane's full: the burst ends on it, the take-up after the flip time, no dose */
    follow_full(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    /* rest ends the take-up; owed again, the dose — but not on the tick a move ended */
    follow_rest(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, 500);
    steps(&w, 4, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    step(&w, 0, 0);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(taut_zeroes_the_debt_and_bursts_counting_one_fix_per_episode)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 144);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    /* taut: the burst, the debt to 0, one fix — however long the episode lasts */
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    steps(&w, 10, 4, 1);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    /* rest ends the burst and the episode; taut again is the second */
    follow_rest(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 2);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(a_taut_mid_dose_ends_it_and_the_burst_follows_on_that_tick)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_arm(&w, 154);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 10), 6);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    int32_t enc0 = ace2k_lane_encoder_um(&w.lane, 0);
    step(&w, 4, 1); /* one count of the dose's two */
    ASSERT_EQ(w.feed.l[0].debt_um, 3500);
    /* taut: the dose ends at once — its travel booked, then the debt to 0 — and the burst starts
     * on the same tick at the armed speed, the comparator rebased on it; one fix, one dose, no
     * behind notice */
    follow_taut(&w);
    step(&w, 4, 0);
    assert_burst_running(&w);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, 50000);
    ASSERT_EQ(w.lane.lane[0].move.enc_start, ace2k_lane_encoder_count(&w.lane, 0));
    ASSERT_TRUE(!w.feed.l[0].dose);
    ASSERT_EQ(ace2k_lane_encoder_um(&w.lane, 0) - enc0, 12342 / 10);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    steps(&w, 10, 4, 1);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* The counters' epoch steps at every entry — the follow's arm, the unit's own automatic load
 * (seq 0) and the arm again — and nowhere else: the host restarts its tally exactly then. */
TEST(the_counters_epoch_steps_at_every_entry_and_only_there)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    uint8_t e0 = ace2k_feed_ff_counts(&w.feed, 0).epoch;
    ff_arm(&w, 171);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).epoch, (uint8_t)(e0 + 1));
    steps(&w, 20, 0, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).epoch, (uint8_t)(e0 + 1));
    /* disarmed: the stop resets nothing */
    ace2k_feed_stop(&w.feed, 0, w.now);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).epoch, (uint8_t)(e0 + 1));
    while (pop(&w, &e)) {
    }
    /* the filament out and in again: the unit's own load, seq 0 — a new count */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(ace2k_feed_seq(&w.feed, 0), 171);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).epoch, (uint8_t)(e0 + 2));
    ace2k_feed_stop(&w.feed, 0, w.now);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    /* armed again; the epoch wraps as a uint8 */
    w.feed.l[0].ff_epoch = 255;
    ff_arm(&w, 172);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).epoch, 0);
}

TEST(a_fix_is_counted_when_its_correction_starts)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_arm(&w, 155);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    /* taut under the shared full: no burst, so no fix — the debt zeroed all the same */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    w.in.pulled_any = true;
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 0);
    /* the full clears: the burst, and the episode's one fix */
    w.in.pulled_any = false;
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    steps(&w, 10, 4, 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 1);
    /* the mirror: this lane's full after the burst — held by the follow's rule 1 for the flip time, no fix
     * until the take-up starts */
    follow_full(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
        ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 0);
    }
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    steps(&w, 5, 1, -1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(this_lanes_full_zeroes_the_debt_and_takes_up_counting_one_fix_per_episode)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_paused_at_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 145);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    steps(&w, 20, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    /* this lane's full: no dose, the debt to 0, the take-up, one fix per episode */
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    steps(&w, 5, 1, -1);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    follow_rest(&w);
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 2);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).taut_fixes, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 0);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(a_dose_is_judged_and_ended_as_a_burst)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* the strand held at rest under a dose: the standstill without this lane's full is tangled.
     * The forward trip with this lane's full (stuck) is unreachable through the tick, as for a
     * burst: the shared full ends the dose before the comparator reads that tick. */
    ff_arm(&w, 146);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 10), 6);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 100) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n < 100);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TANGLED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 146);
    ASSERT_TRUE(!pop(&w, &e));
    /* the error ends the metering */
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    /* the shared full ends a dose: no event, the debt lowered by what the strand travelled */
    ff_arm(&w, 147);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 10), 6);
    int32_t enc0 = ace2k_lane_encoder_um(&w.lane, 0);
    step(&w, 4, 1);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    uint32_t owed = w.feed.l[0].debt_um;
    ASSERT_EQ(owed, 3500);
    w.in.pulled_any = true;
    step(&w, 4, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    int32_t travel = ace2k_lane_encoder_um(&w.lane, 0) - enc0;
    ASSERT_EQ(w.feed.l[0].debt_um, owed - (uint32_t)travel + 500U);
    ASSERT_TRUE(!pop(&w, &e));
    /* paused while it lasts: no dose, no counter */
    steps(&w, 20, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, 6000);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 0);
    /* gone: the next dose at once; this lane's full ends it — the debt to 0, and the
     * take-up — its fix — only after the flip time (the follow's rule 1: the dose ran forward) */
    w.in.pulled_any = false;
    step(&w, 0, 0);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 2);
    follow_full(&w);
    step(&w, 4, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 0); /* counted at the take-up */
    for (int i = 0; i < 19; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(the_base_is_ignored_outside_the_follow_and_cleared_by_arm_and_disarm)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* idle, and the forward assist: ignored */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), -ACE2K_EREFUSED);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 148, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), -ACE2K_EREFUSED);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    steps(&w, 10, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    /* the follow: within bounds taken, above the lane's ceiling or a lane out of range not */
    ff_arm(&w, 149);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, ACE2K_LANE_SPEED_MAX_UM_S + 1U, false),
              -ACE2K_EINVAL);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 4, 300, false), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, ACE2K_LANE_SPEED_MAX_UM_S, false), 0);
    ASSERT_EQ(w.feed.l[0].base_um_s, ACE2K_LANE_SPEED_MAX_UM_S);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    ASSERT_EQ(w.feed.l[0].base_um_s, 300);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 15);
    /* the stop disarms: the base and the debt to 0 */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), -ACE2K_EREFUSED);
    /* every arm starts at 0 */
    ff_arm(&w, 150);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    /* the link lost disarms too */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 300, false), 0);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(w.feed.l[0].base_um_s, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(ff_set_tunes_chunk_and_pulse_within_bounds)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.ff.chunk_um, ACE2K_FEED_FF_CHUNK_UM);
    ASSERT_EQ(w.feed.ff.pulse_um_s, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ACE2K_FEED_FF_CHUNK_UM, 3000);
    ASSERT_EQ(ACE2K_FEED_FF_CHUNK_UM_MIN, 1000);
    ASSERT_EQ(ACE2K_FEED_FF_CHUNK_UM_MAX, 10000);
    ASSERT_EQ(ACE2K_FEED_FF_PULSE_UM_S, 15000);
    ASSERT_EQ(ACE2K_FEED_FF_PULSE_UM_S_MIN, ACE2K_LANE_SPEED_MIN_UM_S);
    ASSERT_EQ(ACE2K_FEED_FF_PULSE_UM_S_MAX, ACE2K_LANE_SPEED_MAX_UM_S);
    const struct ace2k_feed_ff_cfg low = {
        .chunk_um = ACE2K_FEED_FF_CHUNK_UM_MIN,
        .pulse_um_s = ACE2K_FEED_FF_PULSE_UM_S_MIN,
    };
    const struct ace2k_feed_ff_cfg high = {
        .chunk_um = ACE2K_FEED_FF_CHUNK_UM_MAX,
        .pulse_um_s = ACE2K_FEED_FF_PULSE_UM_S_MAX,
    };
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &low), 0);
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &high), 0);
    struct ace2k_feed_ff_cfg bad;
    bad = high;
    bad.chunk_um = ACE2K_FEED_FF_CHUNK_UM_MAX + 1U;
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.chunk_um = ACE2K_FEED_FF_CHUNK_UM_MIN - 1U;
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.pulse_um_s = ACE2K_FEED_FF_PULSE_UM_S_MAX + 1U;
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.pulse_um_s = ACE2K_FEED_FF_PULSE_UM_S_MIN - 1U;
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &bad), -ACE2K_EINVAL);
    /* the last good set stands, whole */
    ASSERT_EQ(w.feed.ff.chunk_um, high.chunk_um);
    ASSERT_EQ(w.feed.ff.pulse_um_s, high.pulse_um_s);
    /* 5 mm at 20 mm/s: the next dose takes them, and the cap is two of the new chunk */
    const struct ace2k_feed_ff_cfg mid = { .chunk_um = 5000, .pulse_um_s = 20000 };
    ASSERT_EQ(ace2k_feed_ff_set(&w.feed, &mid), 0);
    ff_arm(&w, 151);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 20), 10);
    assert_dose_running(&w, 5000, 20000);
    steps(&w, 20, 1, 0);
    ASSERT_EQ(w.feed.l[0].debt_um, 10000);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(the_ff_state_frame_carries_the_counters_and_they_reset_at_arm)
{
    struct world w;
    world_init(&w);
    /* the frame is the binding's (feed_cmds.c, ace2k_feed_ff_state): uint16 × 4 per counter from
     * ace2k_feed_ff_counts() — what a host test reaches is the core's counters */
    ff_arm(&w, 152);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_EQ(until_running(&w, 10), 6);
    step(&w, 4, 1);
    step(&w, 4, 1); /* the dose's two counts */
    ASSERT_TRUE(!w.run[0]);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    follow_rest(&w);
    step(&w, 4, 1);
    follow_full(&w);
    step(&w, 0, 0);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    struct ace2k_feed_ff_counts c = ace2k_feed_ff_counts(&w.feed, 0);
    ASSERT_EQ(c.doses, 1);
    ASSERT_EQ(c.taut_fixes, 1);
    ASSERT_EQ(c.full_fixes, 1);
    c = ace2k_feed_ff_counts(&w.feed, 1);
    ASSERT_EQ(c.doses + c.taut_fixes + c.full_fixes, 0);
    c = ace2k_feed_ff_counts(&w.feed, 4);
    ASSERT_EQ(c.doses + c.taut_fixes + c.full_fixes, 0);
    /* kept after the disarm, for the report; the next arm clears them */
    ace2k_feed_stop(&w.feed, 0, w.now);
    c = ace2k_feed_ff_counts(&w.feed, 0);
    ASSERT_EQ(c.doses, 1);
    ASSERT_EQ(c.taut_fixes, 1);
    ASSERT_EQ(c.full_fixes, 1);
    follow_rest(&w);
    step(&w, 0, 0);
    ff_arm(&w, 153);
    c = ace2k_feed_ff_counts(&w.feed, 0);
    ASSERT_EQ(c.doses + c.taut_fixes + c.full_fixes, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* The load's phases.  At 1234.2 µm a count: 300 mm is 243 counts, 600 mm 486, 120 mm 97. */
static void load_to_park(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    steps(w, 50, 0, 0);     /* the settle, then the pull */
    paced(w, 972, 4, 4, 1); /* 243 counts: the parking point */
}

TEST(a_load_with_no_search_configured_is_the_feed_briefs)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.search_um, 0);
    load_to_park(&w, 170);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
}

TEST(a_hold_pauses_the_pull_and_its_release_resumes_what_is_left)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 171, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    paced(&w, 400, 4, 4, 1); /* 100 counts of the 243 */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    steps(&w, 30, 0, 0); /* a session takes its time: no comparator, no deadline */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
    w.in.tag_hold = 0;
    w.in.tag_read = 0x01; /* read: the pull finishes, no search */
    step(&w, 0, 0);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, 300000 - (100 * 12342 / 10)));
    paced(&w, 572, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
}

TEST(no_tag_by_the_parking_point_searches_and_a_read_returns_to_it)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 172);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    paced(&w, 200, 4, 4, 1); /* 50 counts past the parking point */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
    w.in.tag_hold = 0;
    w.in.tag_read = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
    paced(&w, 200, 4, 4, -1); /* 50 counts back */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 172);
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10); /* back at the parking point */
}

TEST(the_search_ends_on_the_lanes_own_full_returns_and_is_no_error)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 173);
    paced(&w, 80, 4, 4, 1);
    w.in.pulled_any = true; /* lane 1's plunger at its pulled end: the strand met something */
    w.in.rest = 0x0E;
    /* the tip snag's watch: with the strand
     * still, the lag's 5 mm of motor (64 pulses) past the base ends it — the obstacle, not
     * stuck */
    step(&w, 4, 0); /* the watch begins: the search keeps running */
    steps(&w, 15, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    w.in.pulled_any = false;
    w.in.rest = 0x0F;
    paced(&w, 80, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED); /* no snag notice, no error */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_search_ends_on_its_length_and_returns)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 174);
    paced(&w, 972, 4, 4, 1); /* 243 counts more: 600 mm since the load began */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    paced(&w, 972, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(with_another_lane_busy_the_load_stops_at_the_parking_point)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 175, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 176, w.now),
              ACE2K_FEED_ACCEPTED); /* lane 2 printing: armed, waiting on its buffer */
    paced(&w, 972, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
}

TEST(another_lanes_start_during_the_search_makes_it_yield_and_return)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    load_to_park(&w, 177);
    paced(&w, 40, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 178, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
}

TEST(a_stop_in_the_search_stops_there_and_does_not_return)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 179);
    paced(&w, 40, 4, 4, 1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    steps(&w, 20, 0, 0);
    ASSERT_TRUE(!w.run[0]);
}

TEST(a_lost_tag_is_reacquired_once_backwards_at_the_floor_then_the_pull_resumes)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 180, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    paced(&w, 400, 4, 4, 1);
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, ACE2K_LANE_SPEED_MIN_UM_S);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_REACQUIRE_UM));
    uint32_t back = ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_REACQUIRE_UM);
    paced(&w, (int)(back * 4), 4, 4, -1); /* not found back: the pull resumes */
    w.in.tag_lost = 0x01;                 /* still lost: no second reacquire */
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
}

TEST(a_return_that_grinds_is_stuck_not_the_unloads_tail)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 181);
    paced(&w, 200, 4, 4, 1);
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_read = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    steps(&w, 80, 4, 0); /* the motor turns, the strand does not come back */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
}

TEST(the_read_with_motion_searches_from_where_the_strand_is_and_comes_back)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 182, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_OTHER_LANE);
    ace2k_feed_stop(&w.feed, 1, w.now);
    w.in.insert = 0x02;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_BUSY);
    paced(&w, 400, 4, 4, 1);
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_read = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    paced(&w, 400, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    while (pop(&w, &e) && e.lane != 0) {
    }
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 0);         /* the unit's own motion */
    ASSERT_EQ(e.filament_um, 0); /* back where it started */
}

TEST(a_start_elsewhere_during_the_reacquire_returns_instead_of_searching_on)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    load_to_park(&w, 183);
    paced(&w, 600, 4, 4, 1); /* 150 counts past the parking point: more than a reacquire's 97 */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 184, w.now),
              ACE2K_FEED_ACCEPTED); /* lane 2 printing, during the reacquire */
    uint32_t forward_before = w.forward_starts[0];
    uint32_t back = ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_FEED_REACQUIRE_UM);
    for (uint32_t i = 0; i < back * 4; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? -1 : 0);
        ASSERT_TRUE(!w.run[0] || w.reverse[0]); /* never forward while lane 2 is armed */
    }
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    uint32_t over = 150 + 243 - back - ace2k_lane_um_to_counts(&w.lane, 0, 300000);
    for (uint32_t i = 0; i < over * 4; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? -1 : 0);
        ASSERT_TRUE(!w.run[0] || w.reverse[0]);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(w.forward_starts[0], forward_before); /* not even a tick's forward move */
    struct ace2k_feed_event e;
    while (pop(&w, &e) && e.lane != 0) {
    }
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
}

/* The lane's encoder in counts from the world's, signed: where the strand is. */
static int32_t enc_counts(const struct world *w)
{
    return (int16_t)w->enc[0];
}

TEST(a_tag_lost_early_in_the_pull_reverses_no_further_than_the_loads_start)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 185, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    paced(&w, 64, 4, 4, 1); /* 16 counts, ≈ 20 mm into the pull */
    int32_t start = enc_counts(&w) - 16;
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    /* 20 mm back, not ACE2K_FEED_REACQUIRE_UM: never behind where the load began */
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, 16 * 12342 / 10));
    paced(&w, 64, 4, 4, -1);
    ASSERT_EQ(enc_counts(&w), start);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING); /* not found back: the pull resumes */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    w.in.tag_lost = 0;
    w.in.tag_read = 0x01; /* a later session read it: no search */
    paced(&w, 972, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10); /* at the parking point */
}

TEST(a_tag_lost_at_the_loads_very_start_reverses_nothing)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 186, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    paced(&w, 4, 4, 4, 1); /* one count: under ACE2K_FEED_RETURN_MIN_UM of room behind */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING); /* straight on */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_TRUE(w.feed.l[0].reacquired);
}

TEST(a_search_that_yields_after_a_reacquire_ends_at_the_parking_point_not_below)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    load_to_park(&w, 187);
    int32_t park = enc_counts(&w);
    paced(&w, 96, 4, 4, 1); /* 24 counts, ≈ 30 mm past the parking point */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 188, w.now),
              ACE2K_FEED_ACCEPTED);
    for (int i = 0; i < 400 && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_LOADING; i++) {
        step(&w, 4, (w.run[0] && w.reverse[0] && (i + 1) % 4 == 0) ? -1 : 0);
        ASSERT_TRUE(enc_counts(&w) >= park); /* never below the parking point */
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_EQ(enc_counts(&w), park);
    struct ace2k_feed_event e;
    while (pop(&w, &e) && e.lane != 0) {
    }
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
}

TEST(a_read_with_motion_that_yields_after_a_reacquire_ends_where_it_started)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    step(&w, 0, 0);
    int32_t start = enc_counts(&w);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, w.now), ACE2K_FEED_ACCEPTED);
    paced(&w, 80, 4, 4, 1); /* 20 counts out */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    w.in.tag_hold = 0;
    w.in.tag_lost = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_EQ(
        w.lane.lane[0].move.target_counts,
        ace2k_lane_um_to_counts(&w.lane, 0, 20 * 12342 / 10)); /* back to the start, no further */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 189, w.now),
              ACE2K_FEED_ACCEPTED);
    for (int i = 0; i < 400 && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_LOADING; i++) {
        step(&w, 4, (w.run[0] && w.reverse[0] && (i + 1) % 4 == 0) ? -1 : 0);
        ASSERT_TRUE(enc_counts(&w) >= start);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(enc_counts(&w), start);
    struct ace2k_feed_event e;
    while (pop(&w, &e) && e.lane != 0) {
    }
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.filament_um, 0);
}

TEST(a_strand_short_of_its_return_point_goes_forward_to_it)
{
    struct world w;
    world_init(&w);
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 190);
    int32_t park = enc_counts(&w);
    paced(&w, 40, 4, 4, 1); /* 10 counts past the parking point */
    w.in.tag_hold = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
    steps(&w, 20, 0, -1); /* while paused, the strand drawn 20 counts back: 10 below */
    w.in.tag_hold = 0;
    w.in.tag_read = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* forward, up to the parking point */
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
    paced(&w, 40, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(enc_counts(&w), park);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
}

/* v0.3.0's limitation: the reset was pending until the next lane tick, and an assist armed in the
 * same tick took its base from the old count — every odometer of the mode off by it. */
TEST(a_reset_then_an_assist_armed_in_the_same_tick_counts_from_the_reset)
{
    struct world w;
    world_init(&w);
    steps(&w, 10, 50, 100); /* 1000 counts on lane 0 */
    ASSERT_EQ(ace2k_lane_reset(&w.lane, 0), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 41, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01; /* taut: a burst */
    step(&w, 0, 0);
    steps(&w, 20, 4, 1); /* 20 counts */
    ace2k_feed_stop(&w.feed, 0, w.now);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.filament_um, 20 * 12342 / 10);
}

/* The same in a load: the mode's base taken in the settle, before any move lands the reset — the
 * pull's odometers count from the reset, not from the 1000 counts before it.  Shown through the
 * LOADED event's odometers alone: a base off by the 1000 counts would read 1000 counts more. */
TEST(a_reset_then_a_load_entered_in_the_same_tick_counts_from_the_reset)
{
    struct world w;
    world_init(&w);
    steps(&w, 10, 50, 100); /* 1000 counts and 500 pulses on lane 0 */
    ASSERT_EQ(ace2k_lane_reset(&w.lane, 0), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 42, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_lane_encoder_count(&w.lane, 0), 0); /* landed at the mode's entry */
    steps(&w, 50, 0, 0);                                /* the settle */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    paced(&w, 972, 4, 4, 1); /* the 243rd count: the parking point */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(3888));
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10);
}

/* --- the tip snag ------------------------------------------------ */

/* Lane 1's plunger: at its pulled end (the shared input, its rest off), at rest. */
static void lane_full(struct world *w)
{
    w->in.pulled_any = true;
    w->in.rest = 0x0E;
    w->in.pushed = 0x00;
}

static void lane_rest(struct world *w)
{
    w->in.pulled_any = false;
    w->in.rest = 0x0F;
    w->in.pushed = 0x00;
}

static void lane_taut(struct world *w)
{
    w->in.pulled_any = false;
    w->in.rest = 0x0E;
    w->in.pushed = 0x01;
}

/* A 300 mm feed (or the load's pull, already running) whose tip catches at 37 mm: 30 counts and
 * 480 pulses healthy; the full on the next tick begins the watch (base 484 pulses, 30 counts);
 * the strand follows — 15 counts in 63 ticks — until 248 pulses (20.08 mm) past the base, the
 * tolerance: the touch starts in reverse on the 63rd tick. */
static void catch_and_follow(struct world *w)
{
    paced(w, 120, 4, 4, 1);
    lane_full(w);
    paced(w, 63, 4, 4, 1);
}

/* The touch's 5 mm (4 counts) back, the plunger set at rest half way: on the 16th tick the touch
 * ends and the same tick resumes forward. */
static void touch_and_rest(struct world *w)
{
    paced(w, 8, 4, 4, -1);
    lane_rest(w);
    paced(w, 8, 4, 4, -1);
}

TEST(snag_a_feed_whose_tip_drags_the_buffer_full_goes_on_after_a_touch)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 211, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    paced(&w, 62, 4, 4, 1);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* still watching: 244 pulses past the base */
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 4, 0);
    ASSERT_TRUE(w.run[0] && w.reverse[0]); /* the touch */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(!pop(&w, &e));
    touch_and_rest(&w);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* resumed */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 211);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(484)); /* where it caught */
    ASSERT_EQ(e.filament_um, 30 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* 41 counts net (30 + 15 − 4): 202 more to 300 mm */
    paced(&w, 808, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10); /* the whole mode's strand */
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(484 + 248 + 64 + 3232));
}

TEST(snag_a_strand_that_stops_while_the_buffer_reads_full_is_blocked_within_the_lag)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 212, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    step(&w, 4, 0); /* the watch begins: base 484 */
    steps(&w, 15, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING); /* 60 pulses: 4.86 mm */
    step(&w, 4, 0);                                             /* 64: 5.18 mm behind */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(484 + 64));
    ASSERT_EQ(e.filament_um, 30 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e)); /* no snag notice */
}

TEST(snag_a_second_full_in_the_same_feed_is_blocked_at_once)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 213, w.now),
              ACE2K_FEED_ACCEPTED);
    catch_and_follow(&w);
    touch_and_rest(&w);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    paced(&w, 40, 4, 4, 1);
    lane_full(&w);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
}

TEST(snag_a_plunger_that_does_not_come_back_after_the_touch_is_blocked_after_the_wait)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 214, w.now),
              ACE2K_FEED_ACCEPTED);
    catch_and_follow(&w);
    paced(&w, 16, 4, 4, -1); /* the touch ends; the plunger still at its pulled end */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    steps(&w, 49, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    step(&w, 0, 0); /* 500 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_plunger_back_at_rest_on_its_own_is_a_notice_and_no_touch)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 215, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    paced(&w, 20, 4, 4, 1);
    lane_rest(&w);
    uint32_t reverse_seen = 0;
    for (int i = 0; i < 4; i++) {
        step(&w, 4, 0);
        reverse_seen += w.reverse[0] ? 1U : 0U;
    }
    ASSERT_EQ(reverse_seen, 0);
    ASSERT_TRUE(w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(484));
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
}

TEST(snag_a_loads_pull_goes_on_after_a_touch_and_ends_loaded)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 216, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0); /* the settle; the pull starts on the 50th tick */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    catch_and_follow(&w);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    touch_and_rest(&w);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    paced(&w, 808, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.filament_um, 243 * 12342 / 10);
}

/* The load's tag search: the search
 * paced forward until it turns into the return, at most `bound` ticks; the ticks taken. */
static int search_until_return(struct world *w, int bound)
{
    int n = 0;
    while (n < bound && w->feed.l[0].sub != ACE2K_FEED_SUB_RETURN) {
        step(w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    return n;
}

/* The return paced back until the load ends, at most `bound` ticks; the ticks taken. */
static int return_until_idle(struct world *w, int bound)
{
    int n = 0;
    while (n < bound && ace2k_feed_mode(&w->feed, 0) == ACE2K_FEED_LOADING) {
        step(w, 4, ((n + 1) % 4 == 0) ? -1 : 0);
        n++;
    }
    return n;
}

TEST(snag_the_search_takes_the_tolerance_resumes_and_ends_on_its_length)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 231);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    catch_and_follow(&w); /* the tip catches 37 mm into the search: the watch, then the touch */
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(!pop(&w, &e));
    touch_and_rest(&w);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the search goes on */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 231);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(3888 + 484)); /* the pull's, then the search's */
    ASSERT_EQ(e.filament_um, (243 + 30) * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* 284 counts net of the 486 the search reaches: ≈ 202 more */
    ASSERT_TRUE(search_until_return(&w, 1000) < 1000);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(return_until_idle(&w, 1200) < 1200);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 231);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_plunger_not_back_after_the_touch_in_the_search_is_the_obstacle_return)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 232);
    catch_and_follow(&w);
    paced(&w, 16, 4, 4, -1); /* the touch ends; the plunger still at its pulled end */
    ASSERT_TRUE(!w.run[0]);
    steps(&w, 49, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    step(&w, 0, 0); /* 500 ms: in a feed, stuck; in the search, the obstacle */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    lane_rest(&w);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED); /* no error, no snag notice */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_tolerance_spent_in_the_pull_makes_a_full_in_the_search_the_obstacle_at_once)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 233, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);
    catch_and_follow(&w);
    touch_and_rest(&w);
    paced(&w, 808, 4, 4, 1); /* the pull reaches the parking point: the search starts */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    paced(&w, 40, 4, 4, 1);
    lane_full(&w);
    step(&w, 4, 0); /* the load's one tolerance is spent: the obstacle at once */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    lane_rest(&w);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_tag_read_during_the_searchs_tolerance_returns_found_after_the_rest)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 234);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    paced(&w, 20, 4, 4, 1); /* the watch */
    w.in.tag_read = 0x01;   /* the reader's session completes meanwhile */
    paced(&w, 43, 4, 4, 1); /* the 63rd tick of the watch: the touch */
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    touch_and_rest(&w);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN); /* back, not forward */
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_tag_read_in_the_searchs_watch_then_a_lag_returns_found_not_obstacle)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 237);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    step(&w, 4, 0);       /* the watch begins */
    w.in.tag_read = 0x01; /* the reader's session completes meanwhile */
    steps(&w, 15, 4, 0);  /* the strand still */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    step(&w, 4, 0); /* 5.18 mm behind: the lag ends the tolerance — the tag is read, so found */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
    lane_rest(&w);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED); /* no snag notice, no error */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_tag_read_in_the_searchs_watch_then_rest_on_its_own_returns_found)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 238);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    paced(&w, 20, 4, 4, 1); /* the watch */
    w.in.tag_read = 0x01;   /* the reader's session completes meanwhile */
    paced(&w, 4, 4, 4, 1);
    lane_rest(&w);
    step(&w, 4, 0); /* rest on its own: the notice, and the search is over — back, found */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_stall_in_the_searchs_watch_is_the_obstacle_return_not_an_error)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 239);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    step(&w, 4, 0); /* the watch begins */
    /* the strand held and the tach silent: no motor travel, so the lag cannot trip; the lane
     * core's stall, ACE2K_LANE_STALL_MS later, ends the watch */
    int n = 0;
    while (n < 150 && w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH) {
        step(&w, 0, 0);
        n++;
    }
    ASSERT_TRUE(n >= (int)(ACE2K_LANE_STALL_MS / 10) - 1 && n < 150);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    lane_rest(&w);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED); /* no error, no snag notice */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_standstill_in_a_loads_pull_is_blocked_and_the_strand_stays_where_it_stopped)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 176, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 50, 0, 0);     /* the settle, then the pull */
    paced(&w, 200, 4, 4, 1); /* 50 counts, 800 pulses: past the grace */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    /* the strand stops short of the parking point: 248 pulses past the last count's tick */
    steps(&w, 61, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_BLOCKED);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.seq, 176);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(800 + 248));
    ASSERT_EQ(e.filament_um, 50 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* no return, no search: the strand waits where it stopped */
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0));
}

TEST(a_standstill_in_the_search_is_the_obstacle_return_and_ends_loaded)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    load_to_park(&w, 177);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    paced(&w, 80, 4, 4, 1); /* 20 counts past the parking point */
    /* the strand stops, the plunger at rest: the comparator's standstill — in the search, the
     * obstacle return to the parking point, never an error */
    int n = 0;
    while (w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH && n < 100) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_EQ(n, 62);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 177);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
}

TEST(snag_another_lanes_start_during_the_searchs_watch_yields_at_once)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ace2k_feed_search_set(&w.feed, 600000);
    w.in.insert = 0x03;
    load_to_park(&w, 235);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    paced(&w, 20, 4, 4, 1); /* the watch */
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_WATCH);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 236, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 0); /* the yield on this tick: no forward travel after it */
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    lane_rest(&w);
    ASSERT_TRUE(return_until_idle(&w, 400) < 400);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED); /* no snag notice, no error */
    ASSERT_EQ(e.seq, 235);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_link_loss_in_the_touch_and_a_stop_in_the_wait_end_the_mode_as_any)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 217, w.now),
              ACE2K_FEED_ACCEPTED);
    catch_and_follow(&w);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    w.in.link_ok = false;
    step(&w, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.link_ok = true;
    lane_rest(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 218, w.now),
              ACE2K_FEED_ACCEPTED);
    catch_and_follow(&w);
    paced(&w, 16, 4, 4, -1); /* the touch ends; waiting for rest */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    steps(&w, 60, 0, 0); /* past the wait: nothing more */
    ASSERT_TRUE(!pop(&w, &e));
}

/* A move that reaches its length during the watch still gets the touch, and the resume, with
 * less than ACE2K_FEED_RETURN_MIN_UM left, finishes the feed: snag, then done with the mode's
 * odometers.  The touch at its 2 mm minimum (2 counts): with the default 5 mm (4 counts) the
 * touch takes back more than a move's rounding leaves past the length, and the resume moves
 * again.  39 mm is a target of 32 counts (39.49 mm); 30 counts after the touch are 37.03 mm,
 * 1.97 mm short — under the 2 mm a move is worth. */
TEST(snag_a_length_reached_in_the_watch_gets_the_touch_and_then_the_feed_is_done)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    struct ace2k_feed_snag_cfg cfg = w.feed.snag;
    cfg.back_um = ACE2K_FEED_SNAG_BACK_MIN_UM;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &cfg), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 39000, 30000, 219, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1); /* 30 counts, 480 pulses: the move still running */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    lane_full(&w);
    /* the watch begins on the first tick (base 484 pulses, 30 counts); the 32nd count on the
     * 8th ends the move done in the watch: the touch, on the same tick */
    paced(&w, 7, 4, 4, 1);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    step(&w, 4, 1);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_TRUE(!pop(&w, &e));
    /* the touch's 2 counts back, the plunger at rest half way: the 8th tick ends the touch, and
     * the same tick's resume finds 1.97 mm left — the feed is done */
    paced(&w, 4, 4, 4, -1);
    lane_rest(&w);
    paced(&w, 4, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 219);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(484)); /* where it caught */
    ASSERT_EQ(e.filament_um, 30 * 12342 / 10);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_EQ(e.seq, 219);
    /* the mode's odometers, not the touch's: 32 counts out, 2 back; 480 + 32 + 32 pulses */
    ASSERT_EQ(e.filament_um, 30 * 12342 / 10);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(480 + 32 + 32));
    ASSERT_TRUE(39000 - e.filament_um < (int32_t)ACE2K_FEED_RETURN_MIN_UM);
    ASSERT_TRUE(!pop(&w, &e));
}

/* Steps until the lane's motor runs forward (the reverse touch) or the lane leaves its mode,
 * at most `bound` ticks with the strand still; the number of ticks taken. */
static int until_forward_or_error(struct world *w, int bound)
{
    int n = 0;
    while (n < bound && w->reverse[0] && ace2k_feed_mode(&w->feed, 0) != ACE2K_FEED_ERROR) {
        step(w, 4, 0);
        n++;
    }
    return n;
}

TEST(snag_a_rollback_whose_tip_catches_after_coming_back_freely_is_forgiven_once)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 300000, 30000, 221, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1); /* 45 counts back: 55.5 mm */
    lane_taut(&w);
    int n = until_forward_or_error(&w, 100);
    ASSERT_TRUE(n < 100); /* a trigger tripped within 20 mm of motor */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch, forward */
    ASSERT_TRUE(!pop(&w, &e));
    paced(&w, 8, 4, 4, 1);
    lane_rest(&w);
    paced(&w, 8, 4, 4, 1); /* 4 counts forward: the touch ends, the rollback resumes */
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 221);
    ASSERT_EQ(e.filament_um, -(45 * 12342 / 10));
    paced(&w, 808, 4, 4, -1); /* 41 counts net back: 202 more */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(e.filament_um, -(243 * 12342 / 10));
}

TEST(snag_a_rollback_taut_before_the_strand_came_back_freely_is_stuck)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 300000, 30000, 222, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 100, 4, 4, -1); /* 25 counts back: 30.9 mm, under 50 */
    lane_taut(&w);
    (void)until_forward_or_error(&w, 100);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_an_unload_whose_tip_catches_is_forgiven_and_ends_unloaded)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 223, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1);
    lane_taut(&w);
    ASSERT_TRUE(until_forward_or_error(&w, 100) < 100);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    paced(&w, 8, 4, 4, 1);
    lane_rest(&w);
    paced(&w, 8, 4, 4, 1);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    paced(&w, 40, 4, 4, -1);
    w.in.insert = 0x00;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
}

TEST(snag_a_second_taut_trip_in_the_same_rollback_is_stuck)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 300000, 30000, 224, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1);
    lane_taut(&w);
    ASSERT_TRUE(until_forward_or_error(&w, 100) < 100);
    paced(&w, 8, 4, 4, 1);
    lane_rest(&w);
    paced(&w, 8, 4, 4, 1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    paced(&w, 40, 4, 4, -1);
    lane_taut(&w);
    (void)until_forward_or_error(&w, 100);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
}

/* A rollback whose strand comes back `counts` counts, the last on the final paced tick, then
 * stands taut: the strand is still from there to the trip (until_forward_or_error moves no
 * count), so the travel at the trip is exactly `counts` · 1234.2 µm. */
static void rollback_taut_after(struct world *w, int counts, uint32_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_ROLLBACK, 300000, 30000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    paced(w, counts * 4, 4, 4, -1);
    lane_taut(w);
    (void)until_forward_or_error(w, 100);
    ASSERT_EQ(ace2k_lane_encoder_um(&w->lane, 0), -(counts * 12342 / 10));
}

TEST(snag_a_reverse_trip_just_under_free_um_is_stuck)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    rollback_taut_after(&w, 40, 225); /* 49 368 µm back: under 50 000 */
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_reverse_trip_just_over_free_um_is_forgiven)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    rollback_taut_after(&w, 41, 226); /* 50 602 µm back: over 50 000 */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch, forward */
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_reverse_trip_without_the_insert_is_stuck)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.in.insert = 0x00; /* no strand at the mouth; the edge falls while idle, before the start */
    step(&w, 0, 0);
    ASSERT_TRUE(!pop(&w, &e));
    rollback_taut_after(&w, 45, 227);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_a_reverse_touch_whose_plunger_never_rests_is_stuck_after_rest_ms)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    rollback_taut_after(&w, 45, 228);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch, forward */
    paced(&w, 16, 4, 4, 1);                 /* 4 counts forward: the touch ends, still taut */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    steps(&w, (int)(ACE2K_FEED_SNAG_REST_MS / 10) - 2, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK); /* still waiting */
    steps(&w, 2, 0, 0);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_STUCK);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(snag_the_lane_setpoint_is_the_speed_then_the_floor_in_the_landing)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 0), 0);
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 4), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 231, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 0), 30000);
    paced(&w, 32, 4, 4, 1); /* 8 of 16 counts: 9.87 mm left, the landing begins on tick 32 */
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 0), 30000); /* the duty still the cruise's */
    paced(&w, 8, 4, 4, 1); /* 10 counts: the loop of tick 40 is the first in the landing */
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 0), ACE2K_LANE_SPEED_MIN_UM_S);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_lane_setpoint_um_s(&w.lane, 0), 0);
}

/* A motor at half the setpoint: 2 pulses a tick (≈ 16 mm/s) against 30 mm/s — the loop raises
 * the duty well over the feed-forward; the strand follows a count every 8 ticks. */
static void slowed_feed_to_full(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    paced(w, 64, 2, 8, 1);
    ASSERT_TRUE(ace2k_lane_duty_pct(&w->lane, 0) > ace2k_lane_feed_forward_pct(30000) + 5U);
    lane_full(w);
}

TEST(snag_the_duty_guard_ends_the_watch_and_off_it_does_not)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_snag_cfg cfg = w.feed.snag;
    cfg.duty_pct = 5;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &cfg), 0);
    slowed_feed_to_full(&w, 232);
    paced(&w, 2, 2, 8, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!ace2k_feed_clear(&w.feed, 0)); /* nothing to clear */
    lane_rest(&w);
    cfg.duty_pct = 0;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &cfg), 0);
    slowed_feed_to_full(&w, 233);
    paced(&w, 20, 2, 8, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(snag_the_settings_default_and_refuse_a_value_out_of_bounds_whole)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.snag.fwd_um, ACE2K_FEED_SNAG_FWD_UM);
    ASSERT_EQ(w.feed.snag.lag_um, ACE2K_FEED_SNAG_LAG_UM);
    ASSERT_EQ(w.feed.snag.duty_pct, ACE2K_FEED_SNAG_DUTY_PCT);
    ASSERT_EQ(w.feed.snag.back_um, ACE2K_FEED_SNAG_BACK_UM);
    ASSERT_EQ(w.feed.snag.rest_ms, ACE2K_FEED_SNAG_REST_MS);
    ASSERT_EQ(w.feed.snag.free_um, ACE2K_FEED_SNAG_FREE_UM);
    const struct ace2k_feed_snag_cfg low = {
        .fwd_um = ACE2K_FEED_SNAG_FWD_MIN_UM,
        .lag_um = ACE2K_FEED_SNAG_LAG_MIN_UM,
        .duty_pct = 0,
        .back_um = ACE2K_FEED_SNAG_BACK_MIN_UM,
        .rest_ms = ACE2K_FEED_SNAG_REST_MIN_MS,
        .free_um = ACE2K_FEED_SNAG_FREE_MIN_UM,
    };
    const struct ace2k_feed_snag_cfg high = {
        .fwd_um = ACE2K_FEED_SNAG_FWD_MAX_UM,
        .lag_um = ACE2K_FEED_SNAG_LAG_MAX_UM,
        .duty_pct = ACE2K_FEED_SNAG_DUTY_MAX_PCT,
        .back_um = ACE2K_FEED_SNAG_BACK_MAX_UM,
        .rest_ms = ACE2K_FEED_SNAG_REST_MAX_MS,
        .free_um = ACE2K_FEED_SNAG_FREE_MAX_UM,
    };
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &low), 0);
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &high), 0);
    struct ace2k_feed_snag_cfg bad;
    bad = high;
    bad.fwd_um = ACE2K_FEED_SNAG_FWD_MAX_UM + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.fwd_um = ACE2K_FEED_SNAG_FWD_MIN_UM - 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.lag_um = ACE2K_FEED_SNAG_LAG_MAX_UM + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.lag_um = ACE2K_FEED_SNAG_LAG_MIN_UM - 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.lag_um = low.fwd_um; /* lag at the tolerance */
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.duty_pct = ACE2K_FEED_SNAG_DUTY_MAX_PCT + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.back_um = ACE2K_FEED_SNAG_BACK_MAX_UM + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.back_um = ACE2K_FEED_SNAG_BACK_MIN_UM - 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.rest_ms = ACE2K_FEED_SNAG_REST_MAX_MS + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.rest_ms = ACE2K_FEED_SNAG_REST_MIN_MS - 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = high;
    bad.free_um = ACE2K_FEED_SNAG_FREE_MAX_UM + 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    bad = low;
    bad.free_um = ACE2K_FEED_SNAG_FREE_MIN_UM - 1U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &bad), -ACE2K_EINVAL);
    /* the last good set stands, whole */
    ASSERT_EQ(w.feed.snag.fwd_um, high.fwd_um);
    ASSERT_EQ(w.feed.snag.lag_um, high.lag_um);
    ASSERT_EQ(w.feed.snag.duty_pct, high.duty_pct);
    ASSERT_EQ(w.feed.snag.back_um, high.back_um);
    ASSERT_EQ(w.feed.snag.rest_ms, high.rest_ms);
    ASSERT_EQ(w.feed.snag.free_um, high.free_um);
}

TEST(snag_a_lowered_tolerance_touches_sooner)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_snag_cfg cfg = w.feed.snag;
    cfg.fwd_um = ACE2K_FEED_SNAG_FWD_MIN_UM;
    cfg.lag_um = ACE2K_FEED_SNAG_LAG_MIN_UM * 4U;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &cfg), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 300000, 30000, 234, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1);
    lane_full(&w);
    int n = 0;
    while (n < 100 && !w.reverse[0]) {
        step(&w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    ASSERT_TRUE(n <= 17); /* 64 pulses: 5.2 mm past the full */
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
}

/* The guard compares the duty with the setpoint it was computed for: a healthy feed that crosses
 * into its landing during the watch is not stuck.  20 mm is 16 counts; the full after 5, the
 * landing on the 8th count (tick 32, 9.87 mm left) — two ticks before the loop of tick 40 lowers
 * the duty to the floor's. */
TEST(snag_the_duty_guard_holds_across_the_landing)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_snag_cfg cfg = w.feed.snag;
    cfg.duty_pct = 5;
    ASSERT_EQ(ace2k_feed_snag_set(&w.feed, &cfg), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 235, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 20, 4, 4, 1);
    lane_full(&w);
    paced(&w, 20, 4, 4, 1); /* the landing on tick 32, the loop on tick 40 */
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* The tail: a runout in the follow feeds on until the strand's tail
 * leaves the drive.  Lane 0 armed in the follow at rest at 50 mm/s, then its insert falls: the
 * tail notice, checked and popped. */
static void tail_enter(struct world *w, uint8_t seq)
{
    struct ace2k_feed_event e;
    ff_arm(w, seq);
    w->in.insert = 0x00;
    step(w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w->feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(w->feed.l[0].tail);
    ASSERT_TRUE(pop(w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL);
    ASSERT_EQ(e.seq, seq);
    ASSERT_TRUE(!pop(w, &e));
}

/* The strand back at the mouth and lane 0 at rest, the next arm possible. */
static void tail_reset(struct world *w)
{
    w->in.link_ok = true;
    w->in.insert = 0x01;
    follow_rest(w);
    step(w, 0, 0);
}

TEST(an_insert_falling_in_the_follow_enters_the_tail_with_a_notice)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ACE2K_FEED_TAIL, 16);
    ASSERT_EQ(ACE2K_FEED_TAIL_OUT, 17);
    ff_arm(&w, 201);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    assert_burst_running(&w);
    /* the insert falls during the burst: the mode stays, the burst runs on, one notice with the
     * mode's seq and odometers (24 pulses, 6 counts) — no runout */
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(w.feed.l[0].tail);
    assert_burst_running(&w);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 201);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(24));
    ASSERT_EQ(e.filament_um, 6 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* the tail goes on, no further notice */
    steps(&w, 10, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    assert_burst_running(&w);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    /* waiting at rest the same: the tail and its notice */
    tail_reset(&w);
    tail_enter(&w, 202);
    ASSERT_TRUE(!w.run[0]);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_tail_feeds_bursts_and_doses_without_the_insert)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    tail_enter(&w, 203);
    /* taut: a burst, the insert empty */
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    steps(&w, 5, 4, 1);
    follow_rest(&w);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.run[0]);
    /* a base rate: a debt of one chunk starts a dose, the insert empty */
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_TRUE(until_running(&w, 10) <= 7);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).doses, 1);
    step(&w, 4, 1);
    step(&w, 4, 1);
    ASSERT_TRUE(!w.feed.l[0].dose);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(w.feed.l[0].tail);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(the_tail_takes_up_nothing)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* armed full: the take-up at once; the insert falls under it — the take-up ends at once */
    follow_full(&w);
    step(&w, 0, 0);
    ff_arm(&w, 204);
    assert_take_up_running(&w, ACE2K_FEED_FOLLOW_TAKE_UM);
    steps(&w, 3, 1, -1);
    w.in.insert = 0x00;
    step(&w, 1, -1);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_WAITING);
    ASSERT_TRUE(w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL);
    ASSERT_TRUE(!pop(&w, &e));
    /* this lane's full held: nothing, past the flip time too */
    for (int i = 0; i < 100; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    /* through rest and full again: nothing */
    follow_rest(&w);
    step(&w, 0, 0);
    follow_full(&w);
    for (int i = 0; i < 100; i++) {
        step(&w, 0, 0);
        ASSERT_TRUE(!w.run[0]);
    }
    ASSERT_EQ(ace2k_feed_bursts(&w.feed, 0), 1);
    ASSERT_EQ(ace2k_feed_ff_counts(&w.feed, 0).full_fixes, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

TEST(the_standstill_in_the_tail_is_tail_out_not_an_error)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ff_arm(&w, 205);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL);
    /* the tail leaves the drive: the motor turns, the strand stands still — the standstill's
     * 20 mm of motor, ≈ 62 ticks at 4 pulses */
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 200) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n > 55 && n < 70);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL_OUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 205);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(w.fg[0]));
    ASSERT_EQ(e.filament_um, 6 * 12342 / 10);
    ASSERT_TRUE(!pop(&w, &e));
    /* the same standstill with the strand present is the follow's tangled, as before */
    tail_reset(&w);
    ff_arm(&w, 206);
    follow_taut(&w);
    step(&w, 0, 0);
    n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 200) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
}

TEST(tail_max_ends_the_tail_as_tail_out)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    struct ace2k_feed_follow_cfg cfg = w.feed.follow;
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MIN;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), 0);
    tail_enter(&w, 207);
    uint32_t fg0 = w.fg[0];
    /* the head keeps pulling, the strand following the motor: bursts chain, never a standstill —
     * 200 mm of motor since the tail began end it */
    follow_taut(&w);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 1000) {
        step(&w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    uint32_t since = w.fg[0] - fg0;
    ASSERT_TRUE(ace2k_lane_fg_to_um(since) >= ACE2K_FEED_FOLLOW_TAIL_UM_MIN);
    ASSERT_TRUE(ace2k_lane_fg_to_um(since - 4U) < ACE2K_FEED_FOLLOW_TAIL_UM_MIN);
    ASSERT_TRUE(ace2k_feed_bursts(&w.feed, 0) >= 2);
    /* the behind notices of the chained bursts, then tail_out — no error, no runout */
    uint8_t last = 0xFF;
    while (pop(&w, &e)) {
        ASSERT_TRUE(e.kind == ACE2K_FEED_BEHIND || e.kind == ACE2K_FEED_TAIL_OUT);
        last = e.kind;
        if (e.kind == ACE2K_FEED_TAIL_OUT) {
            ASSERT_EQ(e.seq, 207);
            ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
            ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(w.fg[0]));
        }
    }
    ASSERT_EQ(last, ACE2K_FEED_TAIL_OUT);
}

TEST(the_tail_ignores_a_strand_put_in_at_the_bay_and_loads_nothing_at_its_end)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    tail_enter(&w, 208);
    uint32_t base = w.feed.l[0].tail_fg_start;
    follow_full(&w);
    steps(&w, 20, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    /* a new strand pushed in at the mouth: the old end holds the
     * drive gear, so nothing changes — the tail goes on, no take-up on this lane's full, no
     * event */
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(w.feed.l[0].tail);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    steps(&w, 100, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* withdrawn and pushed in again during the tail: no second notice, the tail's base kept */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_TRUE(w.feed.l[0].tail);
    ASSERT_EQ(w.feed.l[0].tail_fg_start, base);
    ASSERT_TRUE(!pop(&w, &e));
    /* the head pulls: a burst; the old end leaves the drive and the new strand behind it is not
     * gripped — the standstill is the tail-out, never tangled, with the insert set */
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 200) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n > 55 && n < 70);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL_OUT);
    ASSERT_EQ(e.seq, 208);
    ASSERT_TRUE(!pop(&w, &e));
    /* the insert set at the tail-out: no load — the strand is not at the gear, and no fresh
     * edge; nothing moves, whatever the buffer reads */
    follow_rest(&w);
    steps(&w, 100, 0, 0);
    follow_full(&w);
    steps(&w, 100, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    /* the operator pulls it out and pushes it in again: the fresh edge, the automatic load */
    follow_rest(&w);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].seq, 0);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
}

TEST(tail_max_with_a_strand_put_in_at_the_bay_ends_idle_and_loads_nothing)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    struct ace2k_feed_follow_cfg cfg = w.feed.follow;
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MIN;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), 0);
    w.feed.load.auto_load = true;
    tail_enter(&w, 209);
    w.in.insert = 0x01; /* put in during the tail */
    follow_taut(&w);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 1000) {
        step(&w, 4, ((n + 1) % 4 == 0) ? 1 : 0);
        n++;
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    uint8_t last = 0xFF;
    while (pop(&w, &e)) {
        ASSERT_TRUE(e.kind == ACE2K_FEED_BEHIND || e.kind == ACE2K_FEED_TAIL_OUT);
        last = e.kind;
    }
    ASSERT_EQ(last, ACE2K_FEED_TAIL_OUT);
    follow_rest(&w);
    steps(&w, 100, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(stop_link_and_shutdown_end_the_tail)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* a stop during a burst of the tail */
    tail_enter(&w, 211);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 211);
    ASSERT_TRUE(!pop(&w, &e));
    /* the link down during a burst of the tail: the lane core ends it */
    tail_reset(&w);
    tail_enter(&w, 212);
    follow_taut(&w);
    steps(&w, 3, 4, 1);
    w.in.link_ok = false;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.seq, 212);
    ASSERT_TRUE(!pop(&w, &e));
    /* the link down while the tail waits: the feed ends it */
    tail_reset(&w);
    tail_enter(&w, 213);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(e.seq, 213);
    ASSERT_TRUE(!pop(&w, &e));
    /* a shutdown during a burst of the tail; nothing moves after */
    tail_reset(&w);
    tail_enter(&w, 214);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.feed.l[0].tail);
    one_stopped_shutdown(&w, ACE2K_FEED_FOLLOWING, 214);
    nothing_moves_after_the_shutdown(&w, 50);
}

TEST(the_assists_still_end_on_runout)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    /* the forward assist during a burst */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 50000, 221, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING);
    ASSERT_TRUE(!pop(&w, &e));
    /* the reverse assist during a take-up */
    tail_reset(&w);
    follow_full(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BACK, 0, 50000, 222, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    steps(&w, 3, 1, -1);
    w.in.insert = 0x00;
    step(&w, 1, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].tail);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_RUNOUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_ASSISTING_BACK);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(follow_set_tunes_tail_um_within_bounds)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ACE2K_FEED_FOLLOW_TAIL_UM, 2000000);
    ASSERT_EQ(ACE2K_FEED_FOLLOW_TAIL_UM_MIN, 200000);
    ASSERT_EQ(ACE2K_FEED_FOLLOW_TAIL_UM_MAX, 2000000);
    ASSERT_EQ(w.feed.follow.tail_um, ACE2K_FEED_FOLLOW_TAIL_UM);
    struct ace2k_feed_follow_cfg cfg = w.feed.follow;
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MIN;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), 0);
    ASSERT_EQ(w.feed.follow.tail_um, ACE2K_FEED_FOLLOW_TAIL_UM_MIN);
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MAX;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), 0);
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MIN - 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), -ACE2K_EINVAL);
    cfg.tail_um = ACE2K_FEED_FOLLOW_TAIL_UM_MAX + 1U;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), -ACE2K_EINVAL);
    cfg.tail_um = 0;
    ASSERT_EQ(ace2k_feed_follow_set(&w.feed, &cfg), -ACE2K_EINVAL);
    /* nothing taken from a refused set */
    ASSERT_EQ(w.feed.follow.tail_um, ACE2K_FEED_FOLLOW_TAIL_UM_MAX);
}

/* Only the second half exercises the tail's own gate: the first half holds anyway, since the
 * tolerance never arms in FOLLOWING (pushes_toward_head is false there), tail or not. The preset
 * REST phase is what shows the tail's tick skipping a tolerance phase it finds. */
TEST(the_snag_tolerance_does_not_run_in_the_tail)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    tail_enter(&w, 231);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    steps(&w, 5, 4, 1);
    /* this lane's buffer goes full against the forward push: the burst ends on the full, and no
     * touch back, no watch, no snag verdict — not now, not while the full holds */
    follow_full(&w);
    step(&w, 4, 1);
    for (int i = 0; i < 100; i++) {
        ASSERT_TRUE(!w.run[0]);
        ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
        step(&w, 0, 0);
    }
    ASSERT_TRUE(!w.feed.l[0].snag_used);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!pop(&w, &e));
    /* a tolerance phase found on a lane in the tail is not stepped: the tail's tick skips it */
    w.feed.l[0].snag = ACE2K_FEED_SNAG_REST;
    w.feed.l[0].t_snag_ms = w.now;
    steps(&w, 100, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!pop(&w, &e));
    ace2k_feed_stop(&w.feed, 0, w.now);
}

/* A burst whose strand comes a count every seven ticks of 4 pulses — ≈ 54 % of the motor: never
 * a standstill (two counts in 4.5 mm of motor), short by ≈ 23 mm when the 50 mm window closes. */
static int partial_until_end(struct world *w)
{
    int n = 0;
    while (ace2k_feed_mode(&w->feed, 0) == ACE2K_FEED_FOLLOWING && n < 400) {
        step(w, 4, ((n + 1) % 7 == 0) ? 1 : 0);
        n++;
    }
    return n;
}

TEST(a_partial_trip_in_the_tail_is_tail_out_not_an_error)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    tail_enter(&w, 232);
    follow_taut(&w);
    step(&w, 0, 0);
    assert_burst_running(&w);
    int32_t enc0 = ace2k_lane_encoder_um(&w.lane, 0);
    int n = partial_until_end(&w);
    /* the window closes at 50 mm of motor: ≈ 155 ticks */
    ASSERT_TRUE(n > 150 && n < 165);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL_OUT);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 232);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(w.fg[0]));
    ASSERT_EQ(e.filament_um, ace2k_lane_encoder_um(&w.lane, 0) - enc0);
    ASSERT_TRUE(!pop(&w, &e));
    /* the same strand with the insert present: the follow's partial, tangled — as before */
    tail_reset(&w);
    ff_arm(&w, 233);
    follow_taut(&w);
    step(&w, 0, 0);
    (void)partial_until_end(&w);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_TANGLED);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
}

TEST(a_standstill_during_a_dose_in_the_tail_is_tail_out)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    tail_enter(&w, 234);
    ASSERT_EQ(ace2k_feed_base_set(&w.feed, 0, 50000, false), 0);
    ASSERT_TRUE(until_running(&w, 10) <= 7);
    assert_dose_running(&w, ACE2K_FEED_FF_CHUNK_UM, ACE2K_FEED_FF_PULSE_UM_S);
    /* the dose turns, the strand stands still: the standstill's 20 mm of motor */
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_FOLLOWING && n < 200) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n > 55 && n < 70);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].dose);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_TAIL_OUT);
    ASSERT_EQ(e.seq, 234);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(w.fg[0]));
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!pop(&w, &e));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
