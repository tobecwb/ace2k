/* The feed with its insert sensors unknown: the ADC scan stopped, so the insert reading is frozen
 * and says nothing.  A start is refused, a running mode stops, and no edge is judged. */
#include "test.h"
#include "fake_feed_world.h"
#include "lane/sensors.h"
#include "core/config.h" // ace2k_insert_band_defaults

static void follow_taut(struct world *w)
{
    w->in.rest = 0x0E;
    w->in.pushed = 0x01;
    w->in.pulled_any = false;
}

/* The insert sensors as the binding feeds them to the feed: lane 1's reading with filament, every
 * switch pin high, the pass age_ms old.  The sensors' clock is the world's. */
struct ace2k_sensed {
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
};

static void sensed_init(struct ace2k_sensed *x, const struct world *w)
{
    struct ace2k_insert_band bands[ACE2K_LANE_COUNT];
    for (int i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_insert_band_defaults(&bands[i]);
    }
    ace2k_sensors_init(&x->s, bands);
    x->in = (struct ace2k_sensors_inputs){ .adc_valid = true, .now_ms = w->now };
    for (int i = 0; i < ACE2K_ADC_COUNT; i++) {
        x->in.mv[i] = 1200;
    }
    x->in.mv[ACE2K_ADC_INSERT1] = 400; /* lane 1 present */
    x->in.levels = 0x1FFU;             /* every switch released, the cutout idle */
}

/* One tick of the sensors, then the world's, with what the binding passes on (feed_cmds.c).
 * A world clock ahead of the last tick is a catch-up, as the board's tick makes one. */
static void sensed_step(struct world *w, struct ace2k_sensed *x, uint32_t pulses, int counts)
{
    x->in.now_ms = w->now + 10U;
    ace2k_sensors_tick(&x->s, &x->in);
    w->in.insert = x->s.lane[0].insert ? 0x01U : 0x00U;
    w->in.insert_unknown = (bool)!ace2k_sensors_insert_known(&x->s);
    step(w, pulses, counts);
}

TEST(sensors_unknown_refuses_every_start_and_the_search)
{
    struct world w;
    world_init(&w);
    w.feed.search_um = 100000;
    w.in.insert_unknown = true;
    step(&w, 0, 0);
    static const enum ace2k_feed_cmd cmds[] = {
        ACE2K_FEED_CMD_FEED,        ACE2K_FEED_CMD_ROLLBACK, ACE2K_FEED_CMD_ASSIST,
        ACE2K_FEED_CMD_ASSIST_BACK, ACE2K_FEED_CMD_UNLOAD,   ACE2K_FEED_CMD_LOAD,
        ACE2K_FEED_CMD_ASSIST_BOTH,
    };
    uint8_t seq = 31;
    for (unsigned i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        ASSERT_EQ(ace2k_feed_start(&w.feed, 0, cmds[i], 100000, 30000, seq++, w.now),
                  ACE2K_FEED_REFUSED_SENSORS_STALE);
    }
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 41, w.now), ACE2K_FEED_REFUSED_SENSORS_STALE);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    /* the link is judged first, as for every start */
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 42, w.now),
              ACE2K_FEED_REFUSED_NO_LINK);
    /* the readings back: the start goes */
    w.in.link_ok = true;
    w.in.insert_unknown = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 43, w.now),
              ACE2K_FEED_ACCEPTED);
}

TEST(sensors_unknown_stops_a_running_follow_with_its_own_kind)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 51, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    steps(&w, 5, 4, 1);
    ASSERT_TRUE(w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert_unknown = true;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SENSORS);
    ASSERT_EQ(e.mode, ACE2K_FEED_FOLLOWING);
    ASSERT_EQ(e.seq, 51);
    ASSERT_TRUE(!pop(&w, &e));
    /* nothing starts again on its own while the readings are gone */
    steps(&w, 50, 0, 0);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(sensors_unknown_stops_a_feed_and_a_load_in_its_settle_and_judges_no_fall)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 61, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 20, 4, 4, 1);
    /* the frozen bit drops on the same tick: it is no runout, the readings are gone */
    w.in.insert_unknown = true;
    w.in.insert = 0x00;
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SENSORS);
    ASSERT_EQ(e.mode, ACE2K_FEED_FEEDING);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert = 0x01;
    w.in.insert_unknown = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 62, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 0, 0);
    w.in.insert_unknown = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SENSORS);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_EQ(e.motor_um, 0);
}

TEST(sensors_unknown_starts_no_automatic_load_and_the_first_reading_back_is_no_edge)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert_unknown = true;
    step(&w, 0, 0);
    w.in.insert = 0x01; /* a rise while unknown: not judged */
    steps(&w, 3, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    /* the first tick back primes the bit again, as the boot does: no edge from the stale one */
    w.in.insert_unknown = false;
    steps(&w, 3, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!pop(&w, &e));
    /* a real edge after it loads as always */
    w.in.insert = 0x00;
    step(&w, 0, 0);
    w.in.insert = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
}

TEST(sensors_unknown_keeps_a_lane_in_error_whatever_the_frozen_bit_does)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 71, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 101, 0, 0); /* the tach silent: motor_stalled */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    while (pop(&w, &e)) {
    }
    w.in.insert_unknown = true;
    step(&w, 0, 0);
    w.in.insert = 0x00; /* not read: the error is not ended by a runout */
    steps(&w, 3, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 100000, 30000, 72, w.now),
              ACE2K_FEED_REFUSED_IN_ERROR);
}

TEST(a_flash_stall_does_not_stop_a_follow_and_a_scan_that_stays_stopped_does)
{
    /* A flash write holds the core for 300 ms.  The pass from before it is then past the bound,
     * and the first pass after it comes a little later.  The scan never stopped, so the follow
     * goes on.  Then the scan stops for good: the follow ends stopped_sensors. */
    struct world w;
    struct ace2k_sensed x;
    struct ace2k_feed_event e;
    world_init(&w);
    sensed_init(&x, &w);
    for (int i = 0; i < 5; i++) {
        x.in.adc_age_ms = 5U;
        sensed_step(&w, &x, 0, 0);
    }
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, 81, w.now),
              ACE2K_FEED_ACCEPTED);
    follow_taut(&w);
    for (int i = 0; i < 5; i++) {
        sensed_step(&w, &x, 4, 1);
    }
    ASSERT_TRUE(w.run[0]);
    w.now += 300U; /* the stall: the tick catches up on the next step */
    x.in.adc_age_ms += 300U;
    for (int i = 0; i < 3; i++) {
        x.in.adc_age_ms += 10U;
        sensed_step(&w, &x, 4, 1);
    }
    x.in.adc_age_ms = 2U; /* the scan is back */
    for (int i = 0; i < 20; i++) {
        sensed_step(&w, &x, 4, 1);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FOLLOWING);
    while (pop(&w, &e)) {
        ASSERT_TRUE(e.kind != ACE2K_FEED_STOPPED_SENSORS);
    }
    for (int i = 0; i < 20; i++) { /* the scan stops: its age grows with the clock */
        x.in.adc_age_ms += 10U;
        sensed_step(&w, &x, 4, 1);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    bool stopped = false;
    while (pop(&w, &e)) {
        stopped = (bool)(stopped || e.kind == ACE2K_FEED_STOPPED_SENSORS);
    }
    ASSERT_TRUE(stopped);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
