/* The armed stop: ace2k_feed_stop_when() arms the next start on a lane to end on a held tip (the
 * plunger taut and the strand still for held_ms), on the plunger at rest, or on either.  The
 * tests drive the real lane and feed cores through the fake world (fake_feed_world.h). */
#include <stdio.h>

#include "test.h"
#include "fake_feed_world.h"

/* The fake world ticks every 10 ms; the default window in those ticks. */
#define TICK_MS    10U
#define HELD_TICKS ((int)(ACE2K_FEED_HELD_MS / TICK_MS))

/* One rollback on lane 0 at 30 mm/s, armed first as asked. */
static void armed_rollback(struct world *w, bool held, bool at_rest, uint32_t length_um,
                           uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_stop_when(&w->feed, 0, held, at_rest, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_ROLLBACK, length_um, 30000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
}

/* The one event the ring holds. */
static struct ace2k_feed_event only_event(struct world *w)
{
    struct ace2k_feed_event e = { 0 };
    ASSERT_TRUE(pop(w, &e));
    struct ace2k_feed_event more;
    ASSERT_TRUE(!pop(w, &more));
    return e;
}

TEST(a_rollback_armed_for_held_ends_held_when_taut_and_the_strand_stands)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 51);
    paced(&w, 40, 4, 4, -1); /* rolls back freely: 10 counts */
    w.in.pushed = 0x01;      /* the head holds the tip: taut … */
    w.in.rest = 0x0E;
    steps(&w, HELD_TICKS, 4, 0); /* … the strand stands: not yet held_ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(w.run[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 4, 0); /* held_ms after the window opened */
    e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 51);
    ASSERT_EQ(e.filament_um, -(10 * 12342 / 10));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
}

TEST(a_taut_plunger_with_the_strand_still_coming_is_friction_not_held)
{
    struct world w;
    world_init(&w);
    w.in.pushed = 0x01; /* taut from the start: a thick tip dragging in the tube */
    w.in.rest = 0x0E;
    armed_rollback(&w, true, false, 50000, 52);
    paced(&w, 163, 4, 4, -1); /* the strand keeps coming: a window never lasts held_ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    paced(&w, 1, 4, 1, -1); /* the 41st count: 50 mm */
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_DONE);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(a_strand_that_moves_held_um_restarts_the_window)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 53);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 2, 0); /* the window opens */
    steps(&w, HELD_TICKS - 2, 2, 0);
    /* three counts (3.7 mm, at least held_um) just before held_ms: a fresh window */
    step(&w, 2, -3);
    steps(&w, HELD_TICKS - 1, 2, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 2, 0); /* held_ms after the fresh window */
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(e.filament_um, -(3 * 12342 / 10));
}

TEST(a_strand_that_moves_less_than_held_um_keeps_the_window)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 54);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 2, 0);                  /* the window opens */
    step(&w, 2, -2);                 /* two counts, 2.5 mm: under held_um */
    steps(&w, HELD_TICKS - 2, 2, 0); /* held_ms - 10 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

TEST(a_taut_plunger_released_before_held_ms_restarts_the_window)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 55);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, HELD_TICKS - 5, 2, 0); /* taut and still, not for held_ms */
    w.in.pushed = 0x00;              /* released for one tick */
    step(&w, 2, 0);
    w.in.pushed = 0x01;
    steps(&w, HELD_TICKS, 2, 0); /* the new window: held_ms - 10 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(a_rollback_armed_for_at_rest_ends_at_rest_when_the_plunger_comes_to_rest)
{
    struct world w;
    world_init(&w);
    w.in.pushed = 0x01; /* taut at the start: the lane eases the plunger back to rest */
    w.in.rest = 0x0E;
    step(&w, 0, 0);
    armed_rollback(&w, false, true, 80000, 56);
    paced(&w, 8, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 4, 0);
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_AT_REST);
    ASSERT_EQ(e.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(e.seq, 56);
    ASSERT_EQ(e.filament_um, -(2 * 12342 / 10));
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
}

/* The plunger already at rest when the move starts: the first tick is not judged, so the move
 * runs one tick and ends at rest on the second. */
TEST(a_move_started_at_rest_does_not_end_on_its_first_tick)
{
    struct world w;
    world_init(&w); /* every plunger at rest */
    armed_rollback(&w, false, true, 80000, 57);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    ASSERT_TRUE(w.run[0]);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

TEST(a_stop_armed_for_both_ends_on_whichever_comes_first)
{
    struct world w;
    world_init(&w);
    w.in.rest = 0x0E;
    armed_rollback(&w, true, true, 80000, 58);
    w.in.pushed = 0x01;
    steps(&w, HELD_TICKS + 1, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);

    world_init(&w);
    w.in.rest = 0x0E;
    armed_rollback(&w, true, true, 80000, 59);
    w.in.pushed = 0x01;
    steps(&w, 5, 2, 0);
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
}

/* Not armed: the plunger taut and the strand still is today's verdict, the comparator's stuck
 * after the standstill's 20 mm of motor, and rest ends nothing. */
TEST(an_unarmed_rollback_held_at_the_head_is_still_stuck)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 60, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 40, 4, 4, -1);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, 61, 4, 0); /* 244 pulses: 19.8 mm, under the standstill */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0); /* 20.1 mm */
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
}

TEST(an_unarmed_rollback_runs_through_rest)
{
    struct world w;
    world_init(&w); /* every plunger at rest */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 61, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 63, 4, 4, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    paced(&w, 1, 4, 1, -1); /* the 17th count: 20 mm */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

TEST(a_stop_armed_then_refused_by_its_start_is_gone)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, true, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 1000, 62, w.now),
              ACE2K_FEED_REFUSED_BOUNDS); /* a speed under the lane's floor */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 63, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, HELD_TICKS + 5, 2, 0);
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    steps(&w, 5, 2, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK); /* neither stop */
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(a_stop_is_spent_by_its_move_and_the_next_move_runs_unarmed)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, false, true, 20000, 64);
    step(&w, 4, 0);
    step(&w, 4, -1);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST); /* the first move's stop */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 65, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1); /* at rest all along: the second runs its length */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

TEST(a_stop_armed_with_neither_disarms_the_lane)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, false, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 66, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

TEST(a_stop_is_refused_busy_on_a_lane_not_idle)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 4, true, false, 0U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 67, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, true, 0U), -ACE2K_EBUSY);
    step(&w, 4, 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, true, 0U), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 1, true, true, 0U), 0); /* another lane, idle */
    /* the refused arming never reached the running move: at rest goes on */
    steps(&w, 3, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    /* in error: busy too */
    steps(&w, 101, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0U), -ACE2K_EBUSY);
    ASSERT_TRUE(ace2k_feed_clear(&w.feed, 0));
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0U), 0);
}

TEST(a_mode_the_unit_enters_on_its_own_leaves_the_arming_for_the_next_start)
{
    struct world w;
    world_init(&w);
    w.feed.load.auto_load = true;
    w.in.insert = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    w.in.insert = 0x01; /* a strand put in: the automatic load, not a host start */
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    steps(&w, 5, 0, 0); /* its settle, the plunger at rest: no stop */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STOPPED);
    /* the host's next start takes the arming */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 68, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 2, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
}

TEST(held_set_takes_values_within_their_bounds_only)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.held.held_um, ACE2K_FEED_HELD_UM);
    ASSERT_EQ(w.feed.held.held_ms, ACE2K_FEED_HELD_MS);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM_MIN - 1U, ACE2K_FEED_HELD_MS),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM_MAX + 1U, ACE2K_FEED_HELD_MS),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM, ACE2K_FEED_HELD_MS_MIN - 1U),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM, ACE2K_FEED_HELD_MS_MAX + 1U),
              -ACE2K_EINVAL);
    ASSERT_EQ(w.feed.held.held_um, ACE2K_FEED_HELD_UM); /* nothing taken */
    ASSERT_EQ(w.feed.held.held_ms, ACE2K_FEED_HELD_MS);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM_MIN, ACE2K_FEED_HELD_MS_MIN), 0);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM_MAX, ACE2K_FEED_HELD_MS_MAX), 0);
    ASSERT_EQ(w.feed.held.held_um, ACE2K_FEED_HELD_UM_MAX);
    ASSERT_EQ(w.feed.held.held_ms, ACE2K_FEED_HELD_MS_MAX);
}

TEST(a_held_window_set_shorter_ends_the_move_sooner)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM, 100U), 0);
    armed_rollback(&w, true, false, 80000, 70);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, 10, 4, 0); /* the window opens on the first: 90 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

TEST(a_held_um_set_wider_holds_a_strand_that_moves_less)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, 5000U, ACE2K_FEED_HELD_MS), 0);
    armed_rollback(&w, true, false, 80000, 71);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    step(&w, 2, 0);  /* the window opens */
    step(&w, 2, -3); /* 3.7 mm: under 5 mm, the window stays */
    steps(&w, HELD_TICKS - 2, 2, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

/* One 10 ms tick with lanes 0 and 1 both moving: the tach and the encoder of each advance. */
static void step_two(struct world *w, uint32_t pulses, int counts0, int counts1)
{
    w->fg[1] += pulses;
    w->enc[1] = (uint16_t)(w->enc[1] + counts1);
    step(w, pulses, counts0);
}

/* A held_ms carried by the arming is this move's window, whatever the unit's setting. */
TEST(a_window_armed_with_the_stop_ends_held_after_its_own_held_ms)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 120U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 80, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, 12, 4, 0); /* the window opens on the first: 110 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
    ASSERT_EQ(w.feed.held.held_ms, ACE2K_FEED_HELD_MS); /* the setting is untouched */

    /* the window went with that start: the next arming with 0 takes the unit's setting */
    w.in.pushed = 0x00;
    w.in.rest = 0x0F;
    step(&w, 0, 0);
    armed_rollback(&w, true, false, 80000, 81);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, HELD_TICKS, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

/* A re-arming replaces the earlier one whole: a window armed before and replaced by 0 (the host's
 * disarm, or a new arming with the unit's setting) never reaches the next move. */
TEST(a_rearming_with_0_replaces_an_earlier_window)
{
    for (int disarm = 0; disarm < 2; disarm++) {
        struct world w;
        world_init(&w);
        ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 120U), 0);
        if (disarm) {
            ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, false, 0U), 0);
        }
        armed_rollback(&w, true, false, 80000, 87); /* held_ms 0: the unit's 550 ms */
        w.in.pushed = 0x01;
        w.in.rest = 0x0E;
        steps(&w, 12, 4, 0); /* past the old 120 ms window: still running */
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
        steps(&w, HELD_TICKS - 12, 4, 0);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
        step(&w, 4, 0); /* the tick after the unit's held_ms */
        ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
    }
}

/* 0 in the arming is the unit's held_ms, as ace2k_feed_held_set left it. */
TEST(a_window_of_0_uses_the_units_setting)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM, 200U), 0);
    armed_rollback(&w, true, false, 80000, 82);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, 20, 4, 0); /* the window opens on the first: 190 ms */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

/* Two lanes armed with different windows: each move keeps its own. */
TEST(two_lanes_armed_with_different_windows_each_use_their_own)
{
    struct world w;
    world_init(&w);
    w.in.insert = 0x03;
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 600U), 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 1, true, false, 120U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 10000, 83, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ROLLBACK, 80000, 50000, 84, w.now),
              ACE2K_FEED_ACCEPTED);
    w.in.pushed = 0x03; /* both tips held at once */
    w.in.rest = 0x0C;
    for (int i = 0; i < 12; i++) {
        step_two(&w, 2, 0, 0);
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_ROLLING_BACK);
    step_two(&w, 2, 0, 0); /* lane 1's 120 ms */
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    for (int i = 0; i < 47; i++) {
        step_two(&w, 2, 0, 0); /* lane 0: up to 590 ms after its window opened */
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step_two(&w, 2, 0, 0); /* lane 0's 600 ms */
    e = only_event(&w);
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
}

/* A window outside the bounds is refused, whatever the lane's mode, and nothing is armed. */
TEST(a_window_out_of_bounds_is_refused_and_arms_nothing)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, ACE2K_FEED_HELD_MS_MIN - 1U),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, ACE2K_FEED_HELD_MS_MAX + 1U),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 70000U), -ACE2K_EINVAL);
    ASSERT_TRUE(!w.feed.l[0].stop_next.held && !w.feed.l[0].stop_next.at_rest);
    ASSERT_EQ(w.feed.l[0].stop_next.held_ms, 0U);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, ACE2K_FEED_HELD_MS_MIN), 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, ACE2K_FEED_HELD_MS_MAX), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 85, w.now),
              ACE2K_FEED_ACCEPTED);
    /* out of bounds on a running lane: still the bounds, never busy */
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, ACE2K_FEED_HELD_MS_MAX + 1U),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 120U), -ACE2K_EBUSY);
}

/* A 120 ms window at 50 mm/s: a strand that still comes at about 85 % of the motor, a thick tip
 * dragging in the tube, moves held_um inside every window, so the move runs to its end. */
TEST(friction_at_50_mm_s_with_a_120_ms_window_runs_to_done)
{
    struct world w;
    world_init(&w);
    w.in.pushed = 0x01; /* taut from the start */
    w.in.rest = 0x0E;
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 120U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 50000, 50000, 86, w.now),
              ACE2K_FEED_ACCEPTED);
    /* 6 pulses a tick: 48.6 mm/s of motor; a count every 3 ticks: 41.1 mm/s of strand */
    paced(&w, 120, 6, 3, -1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    paced(&w, 3, 6, 3, -1); /* the 41st count: 50 mm */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

/* The window keeps its time across the millisecond clock's wrap. */
TEST(a_held_window_across_the_clock_wrap_ends_on_time)
{
    struct world w;
    world_init_at(&w, UINT32_MAX - 100U);
    armed_rollback(&w, true, false, 80000, 72);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    steps(&w, HELD_TICKS, 2, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

/* The stop runs ahead of the comparator.  The unarmed move finds the tick on which the
 * standstill trips (stuck); the armed one sets its window to end on that same tick, and ends
 * held: a held tip never reaches stuck. */
TEST(an_armed_stop_is_judged_before_the_comparator)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 73, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 40, 4, 4, -1);
    w.in.pushed = 0x01;
    w.in.rest = 0x0E;
    uint32_t n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 200U) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(n, 62U);

    world_init(&w);
    ASSERT_EQ(ace2k_feed_held_set(&w.feed, ACE2K_FEED_HELD_UM, (n - 1U) * 10U), 0);
    armed_rollback(&w, true, false, 80000, 74);
    paced(&w, 40, 4, 4, -1);
    w.in.pushed = 0x01; /* the window opens on the next tick */
    w.in.rest = 0x0E;
    steps(&w, (int)n - 1, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0); /* the standstill's tick */
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(e.filament_um, -(10 * 12342 / 10));
}

/* A link lost ends an armed move as any other, and a start after it runs unarmed. */
TEST(an_armed_move_ended_by_the_link_leaves_nothing_armed)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, false, true, 80000, 76);
    w.in.link_ok = false;
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STOPPED_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 77, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

/* The plunger taut: the head holds the tip. */
static void taut(struct world *w)
{
    w->in.pulled_any = false;
    w->in.rest = 0x0E;
    w->in.pushed = 0x01;
}

/* The plunger at rest. */
static void at_rest(struct world *w)
{
    w->in.pulled_any = false;
    w->in.rest = 0x0F;
    w->in.pushed = 0x00;
}

/* A 200 mm rollback, armed for held or not, whose strand comes back freely for `free_ticks` (the
 * tach at `pulses` a tick, a count every second tick) and then stands with the plunger taut.  The
 * ticks the move ran on after that, until it left its mode; its one event in *e. */
static int roll_then_hold(struct world *w, bool armed, uint32_t pulses, uint32_t speed_um_s,
                          int free_ticks, uint8_t seq, struct ace2k_feed_event *e)
{
    world_init(w);
    ASSERT_EQ(ace2k_feed_stop_when(&w->feed, 0, armed, false, 0U), 0);
    ASSERT_EQ(
        ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_ROLLBACK, 200000, speed_um_s, seq, w->now),
        ACE2K_FEED_ACCEPTED);
    paced(w, free_ticks, pulses, 2, -1);
    taut(w);
    int n = 0;
    while (ace2k_feed_mode(&w->feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 200) {
        step(w, pulses, 0);
        n++;
    }
    *e = only_event(w);
    return n;
}

/* Fast, the comparator speaks before held_ms: armed for held, its trip with the plunger taut and
 * the strand still ends held, on the tick the unarmed move ends stuck.  `free_ticks` sets which
 * trigger trips first; `still_ticks` is the tick it trips on. */
static void a_trip_while_held_ends_held(uint32_t pulses, uint32_t speed_um_s, int free_ticks,
                                        int still_ticks)
{
    struct world w;
    struct ace2k_feed_event e;
    int n = roll_then_hold(&w, false, pulses, speed_um_s, free_ticks, 80, &e);
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(n, still_ticks);
    ASSERT_TRUE(n < HELD_TICKS); /* the comparator first */
    struct ace2k_feed_event armed;
    ASSERT_EQ(roll_then_hold(&w, true, pulses, speed_um_s, free_ticks, 81, &armed), still_ticks);
    ASSERT_EQ(armed.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(armed.mode, ACE2K_FEED_ROLLING_BACK);
    ASSERT_EQ(armed.filament_um, e.filament_um);
    ASSERT_EQ(armed.motor_um, e.motor_um);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
    ASSERT_TRUE(!w.run[0]);
}

/* 8 pulses a tick: 64.8 mm/s.  The strand stops 25 ticks (16.2 mm of motor) before the partial's
 * window closes 26 counts short: the partial trips first, under the standstill's 20 mm. */
TEST(at_64_8_mm_s_a_partial_trip_while_held_ends_held)
{
    a_trip_while_held_ends_held(8, 64800, 53, 25);
}

/* The strand stops just after the window restarted: the standstill trips first, 20 mm of motor
 * after its base, a count before the stop (28 ticks after the stop). */
TEST(at_64_8_mm_s_a_standstill_while_held_ends_held)
{
    a_trip_while_held_ends_held(8, 64800, 79, 28);
}

/* 9 pulses a tick: 72.9 mm/s of motor.  The partial first, 22 ticks (16 mm) after the stop. */
TEST(at_72_9_mm_s_a_partial_trip_while_held_ends_held)
{
    a_trip_while_held_ends_held(9, 70000, 47, 22);
}

/* The standstill first, 25 ticks after the stop. */
TEST(at_72_9_mm_s_a_standstill_while_held_ends_held)
{
    a_trip_while_held_ends_held(9, 70000, 71, 25);
}

/* The strand still comes, at 60 % of the motor, with the plunger taut: a slip, not a held tip.
 * The window restarts every three counts, and the partial's trip is the comparator's: stuck. */
TEST(a_slipping_strand_with_the_plunger_taut_on_an_armed_rollback_is_still_stuck)
{
    struct world w;
    world_init(&w);
    taut(&w);
    armed_rollback(&w, true, false, 200000, 82);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 400) {
        paced(&w, 1, 4, 1, (n % 6 == 5) ? -1 : 0); /* a count every six ticks: 0.6 of the motor */
        n++;
    }
    ASSERT_TRUE(n < 400);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
}

/* Friction, slow: a thick tip dragging the plunger taut while the strand comes at the lane's
 * floor speed restarts the window before held_ms, so the move runs to done. */
static void friction_at(uint32_t speed_um_s, int every, uint8_t seq)
{
    struct world w;
    world_init(&w);
    taut(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, speed_um_s, seq, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 15 * every, 1, every, -1); /* 15 counts: 18.5 mm, held_ms passed many times */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    paced(&w, 2 * every, 1, every, -1); /* the lane lands the last counts: 20 mm */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

TEST(friction_at_the_lane_floor_speed_runs_to_done)
{
    /* a count every 14 ticks: 8.8 mm/s, under the 9 mm/s floor the lane holds */
    friction_at(ACE2K_LANE_SPEED_MIN_UM_S, 14, 83);
}

TEST(friction_at_10_mm_s_runs_to_done)
{
    friction_at(10000, 12, 84); /* a count every 12 ticks: 10.3 mm/s */
}

/* The reverse tip-snag tolerance forgives a tip that caught after the strand came back freely.
 * An armed stop goes first: held ends the move where the unarmed one starts its touch, and the
 * event's filament_um (55.5 mm back) is what tells a catch after free travel from a tip held at
 * the head. */
TEST(an_armed_held_stop_goes_before_the_reverse_snag_tolerance)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 300000, 30000, 85, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 180, 4, 4, -1); /* 45 counts back: 55.5 mm, past free_um */
    taut(&w);
    int n = 0;
    while (w.reverse[0] && n < 100) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n < 100);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK); /* unarmed: the touch */
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);

    world_init(&w);
    armed_rollback(&w, true, false, 300000, 86);
    paced(&w, 180, 4, 4, -1);
    taut(&w);
    steps(&w, n, 4, 0);
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(e.filament_um, -(45 * 12342 / 10));
    ASSERT_TRUE(!w.run[0]);
}

/* At rest goes first too: the tolerance's touch brings the plunger back to rest, where the
 * unarmed move resumes, and the armed one ends at rest. */
TEST(an_armed_at_rest_stop_goes_before_the_reverse_snag_tolerance)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, false, true, 300000, 87);
    w.in.rest = 0x0E;
    paced(&w, 180, 4, 4, -1);
    taut(&w);
    int n = 0;
    while (w.reverse[0] && n < 100) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n < 100);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the touch, forward */
    paced(&w, 8, 4, 4, 1);
    at_rest(&w);
    step(&w, 4, 0);
    /* At rest is judged before the touch resumes, so no snag notice comes first. */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
}

/* An arming the host did not follow with a start: a tick with the link down clears it. */
TEST(a_link_down_clears_an_arming_no_start_took)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    w.in.link_ok = false;
    step(&w, 0, 0);
    w.in.link_ok = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 88, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1); /* at rest all along: no stop */
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

/* The transport's retry of the last start takes a fresh arming too, and starts nothing: the next
 * new start runs unarmed. */
TEST(a_retried_start_drops_a_fresh_arming)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 89, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 89, w.now),
              ACE2K_FEED_ACCEPTED); /* within the second: the retry */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 90, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 64, 4, 4, -1);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_DONE);
}

/* Two armed moves in a row: the second's stop starts afresh — at rest not judged on its first
 * tick, and its held window opened on its own first taut tick. */
TEST(the_next_armed_move_judges_its_stop_afresh)
{
    struct world w;
    world_init(&w);
    taut(&w);
    armed_rollback(&w, true, false, 80000, 91);
    steps(&w, HELD_TICKS + 1, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD); /* ended with its window open */
    armed_rollback(&w, true, false, 80000, 92);
    steps(&w, HELD_TICKS, 2, 0); /* a stale window would have ended it at once */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
    at_rest(&w);
    step(&w, 0, 0);
    armed_rollback(&w, false, true, 80000, 93);
    step(&w, 4, 0); /* its first tick, at rest: not judged */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    step(&w, 4, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
}

/* A start refused busy under a running armed move leaves that move's stop in force. */
TEST(a_start_refused_busy_leaves_the_running_moves_stop)
{
    struct world w;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 94);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, 95, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    taut(&w);
    steps(&w, HELD_TICKS + 1, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_HELD);
}

/* Both armed, and on held's due tick the plunger also reads rest: at rest is judged first. */
TEST(at_rest_and_held_due_on_one_tick_end_at_rest)
{
    struct world w;
    world_init(&w);
    taut(&w);
    armed_rollback(&w, true, true, 80000, 96);
    steps(&w, HELD_TICKS, 2, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ROLLING_BACK);
    w.in.rest = 0x0F; /* rest and taut on one tick */
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
}

/* The link lost on held's due tick: the link's end goes first. */
TEST(the_link_lost_on_helds_due_tick_ends_stopped_link)
{
    struct world w;
    world_init(&w);
    taut(&w);
    armed_rollback(&w, true, false, 80000, 97);
    steps(&w, HELD_TICKS, 2, 0);
    w.in.link_ok = false;
    step(&w, 2, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STOPPED_LINK);
}

/* The same with no lane move for the lane core to end: an assist waiting on a full buffer, armed
 * for at rest, is ended by the feed's own link check, which goes before the stop. */
TEST(the_link_lost_as_rest_comes_ends_a_waiting_assist_stopped_link)
{
    struct world w;
    world_init(&w);
    w.in.pulled_any = true; /* a buffer full somewhere: the assist waits */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 30000, 99, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    w.in.link_ok = false;
    w.in.rest = 0x0F; /* rest on the tick the link goes */
    w.in.pushed = 0x00;
    step(&w, 0, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_STOPPED_LINK);
}

/* Armed for held, a comparator trip with the plunger not taut is the comparator's: a strand that
 * stands with the plunger between its ends is stuck, as unarmed. */
TEST(a_trip_with_the_plunger_not_taut_is_not_held)
{
    struct world w;
    struct ace2k_feed_event e;
    world_init(&w);
    armed_rollback(&w, true, false, 80000, 100);
    paced(&w, 40, 4, 4, -1);
    w.in.rest = 0x0E; /* off rest, not taut */
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 200) {
        step(&w, 4, 0);
        n++;
    }
    ASSERT_TRUE(n < HELD_TICKS + 20);
    e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
}

/* The stop goes before the autonomous steps: an assist armed for at rest, waiting on a full
 * buffer, reads rest on the tick the buffer clears.  It ends at rest, and no burst starts on that
 * tick (the plunger also reads taut there, which would start one). */
TEST(an_armed_stop_goes_before_the_assists_step)
{
    struct world w;
    world_init(&w);
    w.in.pulled_any = true; /* a buffer full somewhere: the assist waits */
    w.in.rest = 0x0E;
    w.in.pushed = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, false, true, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ASSIST, 0, 30000, 98, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 5, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ASSISTING);
    uint32_t starts = w.forward_starts[0];
    w.in.pulled_any = false; /* nothing full, taut: a burst would start on this tick … */
    w.in.rest = 0x0F;        /* … and rest */
    step(&w, 0, 0);
    ASSERT_EQ(only_event(&w).kind, ACE2K_FEED_AT_REST);
    ASSERT_EQ(w.forward_starts[0], starts);
    ASSERT_TRUE(!w.run[0]);
}

/* A start that breaks the arming's contract: held on a mode other than a rollback or an unload, or
 * at rest on the follow.  It is refused bounds, before anything moves.  The arming is spent as by
 * any refused start, the lane stays idle with no event, and the grip stays (a refused start leaves
 * it).  The same start unarmed is then accepted, so the refusal is the arming's. */
static void contract_refused(enum ace2k_feed_cmd cmd, bool held, bool at_rest, uint32_t length_um,
                             uint32_t speed_um_s)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, ACE2K_FEED_GRIP_UM_MIN), 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, held, at_rest, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, cmd, length_um, speed_um_s, 110, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].stop_next.held && !w.feed.l[0].stop_next.at_rest);
    ASSERT_EQ(w.feed.grip_um[0], ACE2K_FEED_GRIP_UM_MIN);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, cmd, length_um, speed_um_s, 111, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.feed.l[0].stop_on.held && !w.feed.l[0].stop_on.at_rest);
}

/* A load judges held on its return and its reacquire, toward the spool in the middle of its
 * search.  Held there would end the load idle with the search half done. */
TEST(held_on_a_load_is_refused_bounds)
{
    contract_refused(ACE2K_FEED_CMD_LOAD, true, false, 0, 0);
    contract_refused(ACE2K_FEED_CMD_LOAD, true, true, 0, 0);
}

TEST(held_on_a_feed_is_refused_bounds)
{
    contract_refused(ACE2K_FEED_CMD_FEED, true, false, 50000, 30000);
    contract_refused(ACE2K_FEED_CMD_FEED, true, true, 50000, 30000);
}

TEST(held_on_an_assist_of_any_direction_is_refused_bounds)
{
    contract_refused(ACE2K_FEED_CMD_ASSIST, true, false, 0, 30000);
    contract_refused(ACE2K_FEED_CMD_ASSIST_BACK, true, false, 0, 30000);
    contract_refused(ACE2K_FEED_CMD_ASSIST_BOTH, true, false, 0, 50000);
}

/* The follow keeps the plunger at rest, so at rest would end it on its second tick. */
TEST(at_rest_on_the_follow_is_refused_bounds)
{
    contract_refused(ACE2K_FEED_CMD_ASSIST_BOTH, false, true, 0, 50000);
}

/* The contract's own modes: held on a rollback and an unload, at rest on every mode but the
 * follow.  Each is accepted and its mode keeps the arming. */
static void contract_kept(enum ace2k_feed_cmd cmd, bool held, bool at_rest, uint32_t length_um,
                          uint32_t speed_um_s)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, held, at_rest, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, cmd, length_um, speed_um_s, 112, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.l[0].stop_on.held, held);
    ASSERT_EQ(w.feed.l[0].stop_on.at_rest, at_rest);
}

TEST(held_on_a_rollback_and_an_unload_is_accepted)
{
    contract_kept(ACE2K_FEED_CMD_ROLLBACK, true, false, 80000, 30000);
    contract_kept(ACE2K_FEED_CMD_ROLLBACK, true, true, 80000, 30000);
    contract_kept(ACE2K_FEED_CMD_UNLOAD, true, false, 0, 30000);
    contract_kept(ACE2K_FEED_CMD_UNLOAD, true, true, 0, 30000);
}

TEST(at_rest_on_every_mode_but_the_follow_is_accepted)
{
    contract_kept(ACE2K_FEED_CMD_FEED, false, true, 50000, 30000);
    contract_kept(ACE2K_FEED_CMD_ROLLBACK, false, true, 80000, 30000);
    contract_kept(ACE2K_FEED_CMD_UNLOAD, false, true, 0, 30000);
    contract_kept(ACE2K_FEED_CMD_ASSIST, false, true, 0, 30000);
    contract_kept(ACE2K_FEED_CMD_ASSIST_BACK, false, true, 0, 30000);
    contract_kept(ACE2K_FEED_CMD_LOAD, false, true, 0, 0);
}

/* Toward the spool held is judged in any mode: an armed unload whose tip the head holds ends
 * held, idle, no error. */
TEST(an_armed_unload_held_at_the_head_ends_held)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 0, 30000, 102, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 20, 4, 4, -1);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    taut(&w);
    steps(&w, HELD_TICKS, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_UNLOADING);
    step(&w, 4, 0);
    struct ace2k_feed_event e = only_event(&w);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
    ASSERT_EQ(e.mode, ACE2K_FEED_UNLOADING);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), 0);
}

/* A host's thresholds with the standstill under the partial's allowance: the standstill comes at
 * 10 mm of motor, under slip_allow_um, and still ends held, on the tick the unarmed move ends
 * stuck. */
static int standstill_at_10_mm(struct world *w, bool armed, uint8_t seq, struct ace2k_feed_event *e)
{
    world_init(w);
    const struct ace2k_feed_thresholds th = {
        .slip_check_um = 50000U,
        .slip_allow_um = 15000U,
        .stall_check_um = 10000U,
    };
    ASSERT_EQ(ace2k_feed_thresholds_set(&w->feed, &th), 0);
    ASSERT_EQ(ace2k_feed_stop_when(&w->feed, 0, armed, false, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_ROLLBACK, 80000, 30000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    paced(w, 40, 4, 4, -1);
    taut(w);
    int n = 0;
    while (ace2k_feed_mode(&w->feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 200) {
        step(w, 4, 0);
        n++;
    }
    *e = only_event(w);
    return n;
}

TEST(a_standstill_under_slip_allow_ends_held)
{
    struct world w;
    struct ace2k_feed_event e;
    int n = standstill_at_10_mm(&w, false, 103, &e);
    ASSERT_EQ(e.kind, ACE2K_FEED_STUCK);
    ASSERT_EQ(n, 31); /* 124 pulses: 10 mm since the base */
    ASSERT_EQ(standstill_at_10_mm(&w, true, 104, &e), n);
    ASSERT_EQ(e.kind, ACE2K_FEED_HELD);
}

/* A rollback whose strand comes back at `follow_pct` of the motor for `free_ticks`, then stands
 * with the plunger taut (or, with `slip_pct` > 0, keeps coming at that share): its final kind. */
static uint8_t trailing_then_held(uint32_t pulses, int follow_pct, int free_ticks, int slip_pct)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0U), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 400000, 70000, 104, w.now),
              ACE2K_FEED_ACCEPTED);
    uint32_t acc = 0; /* the strand's travel owed, in tenths of a µm */
    for (int i = 0; i < free_ticks; i++) {
        acc += ace2k_lane_fg_to_um(pulses) * 10U * (uint32_t)follow_pct / 100U;
        int counts = 0;
        while (acc >= 12342U) {
            acc -= 12342U;
            counts--;
        }
        step(&w, pulses, counts);
    }
    taut(&w);
    int n = 0;
    while (ace2k_feed_mode(&w.feed, 0) == ACE2K_FEED_ROLLING_BACK && n < 400) {
        acc += ace2k_lane_fg_to_um(pulses) * 10U * (uint32_t)slip_pct / 100U;
        int counts = 0;
        while (acc >= 12342U) {
            acc -= 12342U;
            counts--;
        }
        step(&w, pulses, counts);
        n++;
    }
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    return e.kind;
}

/* The strand trailed the motor by a few per cent before it stopped: the partial's window carries
 * that shortfall and can trip before the strand is still by its base.  In every phase of the
 * window, at 32.4 to 72.9 mm/s, the move still ends held. */
TEST(a_held_tip_ends_held_in_every_phase_with_the_strand_trailing)
{
    static const uint32_t pulses[] = { 4U, 6U, 8U, 9U };
    static const int follow[] = { 97, 90 };
    for (size_t p = 0; p < sizeof(pulses) / sizeof(pulses[0]); p++) {
        for (size_t f = 0; f < sizeof(follow) / sizeof(follow[0]); f++) {
            for (int phase = 20; phase < 140; phase++) {
                uint8_t kind = trailing_then_held(pulses[p], follow[f], phase, 0);
                if (kind != ACE2K_FEED_HELD) {
                    (void)printf("    %u pulses, %d %%, phase %d: kind %u\n", pulses[p], follow[f],
                                 phase, kind);
                }
                ASSERT_EQ(kind, ACE2K_FEED_HELD);
            }
        }
    }
}

/* The two phases that tripped the partial too early, at 8 pulses a tick (64.8 mm/s). */
TEST(phases_55_and_56_at_64_8_mm_s_end_held)
{
    ASSERT_EQ(trailing_then_held(8U, 97, 55, 0), ACE2K_FEED_HELD);
    ASSERT_EQ(trailing_then_held(8U, 97, 56, 0), ACE2K_FEED_HELD);
    ASSERT_EQ(trailing_then_held(8U, 90, 55, 0), ACE2K_FEED_HELD);
    ASSERT_EQ(trailing_then_held(8U, 90, 56, 0), ACE2K_FEED_HELD);
}

/* A strand that keeps coming at 60 % with the plunger taut is a slip in every phase: stuck. */
TEST(a_slip_with_the_plunger_taut_ends_stuck_in_every_phase)
{
    for (int phase = 20; phase < 140; phase++) {
        ASSERT_EQ(trailing_then_held(8U, 97, phase, 60), ACE2K_FEED_STUCK);
    }
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
