/* The clear of an unload: past the insert's fall, the motor turns unload_clear_um more, so the end
 * of the filament never stops on the sensor of the bay.  Nothing in that stretch is a fault, since
 * the goal of the unload is reached.  0 ends the unload at the fall. */
#include "test.h"
#include "fake_feed_world.h"

/* The pulses of the tach for this much motor travel, rounded up. */
static uint32_t pulses_for(uint32_t um)
{
    uint32_t n = 0;
    while (ace2k_lane_fg_to_um(n) < um) {
        n++;
    }
    return n;
}

/* An unload of lane 0 that rewinds 50 ticks, then the insert clears on the next one.  It returns
 * the tach at the fall. */
static uint32_t unload_to_the_fall(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    paced(w, 50, 4, 4, -1);
    w->in.insert = 0x00;
    step(w, 4, -1);
    return w->fg[0];
}

TEST(the_default_clear_is_twenty_mm_within_its_bounds)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.unload_clear_um, ACE2K_FEED_UNLOAD_CLEAR_UM);
    ASSERT_EQ(ACE2K_FEED_UNLOAD_CLEAR_UM, 20000);
    ASSERT_EQ(ace2k_feed_unload_set(&w.feed, ACE2K_FEED_UNLOAD_CLEAR_UM_MAX + 1U), -ACE2K_EINVAL);
    ASSERT_EQ(w.feed.unload_clear_um, ACE2K_FEED_UNLOAD_CLEAR_UM); /* nothing taken */
    ASSERT_EQ(ace2k_feed_unload_set(&w.feed, ACE2K_FEED_UNLOAD_CLEAR_UM_MAX), 0);
    ASSERT_EQ(ace2k_feed_unload_set(&w.feed, 0), 0);
    ASSERT_EQ(w.feed.unload_clear_um, 0);
}

TEST(an_unload_turns_its_clear_past_the_fall_then_ends_unloaded)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    uint32_t fall = unload_to_the_fall(&w, 41);
    /* the fall is no end yet: the motor turns on in reverse, with no event */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(!pop(&w, &e));
    uint32_t need = pulses_for(ACE2K_FEED_UNLOAD_CLEAR_UM);
    while (w.fg[0] + 4U < fall + need) { /* the strand comes too: it is still in the drive */
        step(&w, 4, -1);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    }
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 41);
    ASSERT_EQ(e.motor_um, ace2k_lane_fg_to_um(w.fg[0])); /* the clear is in the odometers */
    ASSERT_TRUE(e.filament_um < 0);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_strand_that_stands_still_in_the_clear_is_no_fault)
{
    /* The end has left the drive: the motor turns with nothing to drive, the standstill's
     * distance and more.  No tail, no error: the clear ends unloaded on the motor's travel. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    uint32_t fall = unload_to_the_fall(&w, 42);
    uint32_t need = pulses_for(ACE2K_FEED_UNLOAD_CLEAR_UM);
    while (w.fg[0] + 4U < fall + need) {
        step(&w, 4, 0);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
        ASSERT_TRUE(!w.feed.l[0].tail_out);
    }
    step(&w, 4, 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(a_taut_plunger_in_the_clear_is_no_fault)
{
    /* A taut plunger with the strand still would trip the comparator in an unload: in the clear
     * the goal is reached, and the clear runs on to unloaded. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    uint32_t fall = unload_to_the_fall(&w, 43);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    uint32_t need = pulses_for(ACE2K_FEED_UNLOAD_CLEAR_UM);
    while (w.fg[0] + 4U < fall + need) {
        step(&w, 4, 0);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
        ASSERT_TRUE(w.run[0] && w.reverse[0]);
    }
    step(&w, 4, 0);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
}

/* Steps lane 0 rewinding with the strand at the pace of paced(), until the motor has turned the
 * clear from fall, then one tick more: the clear must have ended by then.  It returns false if
 * the unload ended earlier. */
static bool clear_runs_from(struct world *w, uint32_t fall)
{
    uint32_t need = pulses_for(ACE2K_FEED_UNLOAD_CLEAR_UM);
    for (int i = 1; w->fg[0] + 4U < fall + need; i++) {
        step(w, 4, i % 4 == 0 ? -1 : 0);
        if (ace2k_feed_mode(&w->feed, 0) != ACE2K_FEED_UNLOADING || !w->run[0] || !w->reverse[0]) {
            return false;
        }
    }
    step(w, 4, 0);
    return true;
}

TEST(a_move_that_runs_its_length_in_the_clear_is_followed_by_one_move_for_the_rest)
{
    /* The budget's last millimetres: the move ends on its length inside the clear.  One fresh
     * move turns what the clear still owes, and the unload ends unloaded, not unload_incomplete. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 12000, 30000, 44, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 24, 4, 4, -1); /* 6 counts, 7.4 mm */
    w.in.insert = 0x00;
    step(&w, 4, -1); /* the 7th count at the fall */
    uint32_t fall = w.fg[0];
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(clear_runs_from(&w, fall)); /* the 10th count ended the move: a fresh one ran */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.seq, 44);
    ASSERT_TRUE(e.motor_um >= ace2k_lane_fg_to_um(fall) + ACE2K_FEED_UNLOAD_CLEAR_UM);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
}

TEST(the_fresh_move_of_the_clear_comes_once)
{
    /* The fresh move ends short too, on its own length (a strand that runs ahead of the motor):
     * the unload ends unloaded there, with no second fresh move. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 12000, 30000, 50, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 24, 4, 4, -1);
    w.in.insert = 0x00;
    step(&w, 4, -1);
    paced(&w, 12, 4, 4, -1); /* the move done at the 10th count: the fresh move starts */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    for (int i = 0; i < 40 && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_UNLOADING; i++) {
        step(&w, 1, -1); /* a count a tick, the motor barely turning: the strand's length first */
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_TRUE(!w.run[0]);
}

TEST(a_stop_in_the_clear_is_a_stop)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    (void)unload_to_the_fall(&w, 45);
    steps(&w, 5, 4, -1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_link_lost_in_the_clear_stops_as_ever)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    (void)unload_to_the_fall(&w, 46);
    w.in.link_ok = false;
    step(&w, 4, -1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_LINK);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
}

TEST(the_insert_set_again_in_the_clear_changes_nothing)
{
    /* The end passing back under the sensor's edge: no new edge for the unload, the clear runs
     * on from the first fall. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    uint32_t fall = unload_to_the_fall(&w, 47);
    w.in.insert = 0x01;
    step(&w, 4, -1);
    w.in.insert = 0x00;
    step(&w, 4, -1);
    uint32_t need = pulses_for(ACE2K_FEED_UNLOAD_CLEAR_UM);
    while (w.fg[0] + 4U < fall + need) {
        step(&w, 4, -1);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    }
    step(&w, 4, -1);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_clear_of_zero_ends_the_unload_at_the_fall)
{
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_unload_set(&w.feed, 0), 0);
    (void)unload_to_the_fall(&w, 48);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
}

TEST(a_fall_while_the_motor_does_not_rewind_turns_the_clear_back)
{
    /* The tip snag's touch runs forward against an unload when the insert falls.  The clear does
     * not drive forward: the touch stops, and one fresh move turns the clear back. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 49, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    for (int i = 0; i < 100 && w.reverse[0] && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_UNLOADING;
         i++) {
        step(&w, 4, 0);
    }
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch */
    while (pop(&w, &e)) {
    }
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]); /* the fresh move of the clear */
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(clear_runs_from(&w, w.fg[0]));
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(a_stall_in_the_clear_ends_unloaded_with_no_fresh_move)
{
    /* The end jams just past the sensor: the motor stops turning.  The goal is reached, so the
     * unload ends unloaded, and the motor is not driven into the jam again. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    (void)unload_to_the_fall(&w, 51);
    int i = 0;
    while (i < 300 && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_UNLOADING) {
        step(&w, 0, 0);
        i++;
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(i < 150); /* the lane's one silent second, not a second fresh move's too */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_TRUE(!pop(&w, &e));
    steps(&w, 5, 0, 0);
    ASSERT_TRUE(!w.run[0]); /* nothing started after it */
}

/* An unload of lane 0 with a 12 mm budget, paced: the move ends on its length at the 10th count. It
 * returns the ticks until that end, the fall left alone. */
static int ticks_to_the_end_of(struct world *w)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_UNLOAD, 12000, 30000, 52, w->now),
              ACE2K_FEED_ACCEPTED);
    int n = 0;
    while (n < 200 && ace2k_feed_mode(&w->feed, 0) == ACE2K_FEED_UNLOADING) {
        n++;
        step(w, 4, n % 4 == 0 ? -1 : 0);
    }
    return n;
}

TEST(a_fall_on_the_tick_the_move_ran_its_length_turns_the_clear_with_a_fresh_move)
{
    struct world probe;
    world_init(&probe);
    int n = ticks_to_the_end_of(&probe); /* without a fall: unload_incomplete there */
    ASSERT_EQ(ace2k_feed_error(&probe.feed, 0), ACE2K_FEED_UNLOAD_INCOMPLETE);
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 12000, 30000, 52, w.now),
              ACE2K_FEED_ACCEPTED);
    for (int i = 1; i < n; i++) {
        step(&w, 4, i % 4 == 0 ? -1 : 0);
    }
    w.in.insert = 0x00;
    step(&w, 4, n % 4 == 0 ? -1 : 0); /* the move's end and the fall on one tick */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_TRUE(clear_runs_from(&w, w.fg[0]));
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
}

TEST(a_fall_on_the_tick_the_move_stalled_ends_the_unload_as_before)
{
    /* The stall and the fall come on one tick: the unload ends unloaded at the fall, as it did
     * before the clear, and no fresh move starts. */
    struct world probe;
    world_init(&probe);
    ASSERT_EQ(ace2k_feed_start(&probe.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 53, probe.now),
              ACE2K_FEED_ACCEPTED);
    int n = 0;
    while (n < 300 && ace2k_feed_mode(&probe.feed, 0) == ACE2K_FEED_UNLOADING) {
        n++;
        step(&probe, 0, 0);
    }
    ASSERT_EQ(ace2k_feed_error(&probe.feed, 0), ACE2K_FEED_MOTOR_STALLED);
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 53, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, n - 1, 0, 0);
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
}

TEST(insert_sensors_unknown_in_the_clear_stop_it_with_their_own_kind)
{
    /* The clear is past the goal, but a reading that says nothing still stops it: the unload ends
     * stopped_sensors, not unloaded. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    (void)unload_to_the_fall(&w, 54);
    steps(&w, 5, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    ASSERT_TRUE(w.feed.l[0].clearing);
    ASSERT_TRUE(!pop(&w, &e));
    w.in.insert_unknown = true;
    step(&w, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_STOPPED_SENSORS);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(e.seq, 54);
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_fresh_move_of_the_clear_the_lane_refuses_ends_the_unload_at_the_fall)
{
    /* The fall comes in the tip snag's forward touch, so the clear needs its fresh move.  A move
     * the lane refuses (here a speed out of its bounds, which no command can set) ends the unload
     * unloaded at the fall, as without a clear. */
    struct world w;
    world_init(&w);
    struct ace2k_feed_event e;
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 55, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1);
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    for (int i = 0; i < 100 && w.reverse[0] && ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_UNLOADING;
         i++) {
        step(&w, 4, 0);
    }
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch */
    while (pop(&w, &e)) {
    }
    w.feed.l[0].speed_um_s = ACE2K_LANE_SPEED_MAX_UM_S + 1U;
    w.in.rest = 0x0F;
    w.in.pushed = 0x00;
    w.in.insert = 0x00;
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.feed.l[0].clearing);
    ASSERT_TRUE(!w.run[0]); /* no move after the refusal */
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_UNLOADED);
    ASSERT_EQ(e.seq, 55);
    ASSERT_TRUE(!pop(&w, &e));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
