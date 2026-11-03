/* What: the load's less common paths, each test named for the rule it pins.
 * How: the shared feed world of fake_feed_world.h, one 10 ms tick at a time.
 * Depends on: test.h, fake_feed_world.h. */
#include "test.h"
#include "fake_feed_world.h"

/* At 1234.2 µm a count, the default park of 300 mm is 243 counts and 600 mm is 486. */
#define PARK_UM          300000U
#define SEARCH_UM        600000U
#define UM_PER_COUNT_X10 12342U

/* The strand's travel over n counts, as the feed computes it. */
static uint32_t counts_um(uint32_t n)
{
    return n * UM_PER_COUNT_X10 / 10U;
}

/* A load from lane 1 past its settle, with the pull just started. */
static void load_pulling(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_start(&w->feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    steps(w, 50, 0, 0);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
}

/* The pull of a load, then the search's first counts past the parking point. */
static void load_searching(struct world *w, uint8_t seq, int counts_past)
{
    load_pulling(w, seq);
    paced(w, 972, 4, 4, 1);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    paced(w, counts_past * 4, 4, 4, 1);
}

/* A read session takes the lane on the next tick: the load pauses with the motor stopped. */
static void pause_for_a_session(struct world *w)
{
    w->in.tag_hold = 0x01;
    step(w, 0, 0);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
    ASSERT_TRUE(!w->run[0]);
}

/* The session lets the lane go on the next tick, with these readings of the tag. */
static void release_session(struct world *w, uint8_t read, uint8_t lost)
{
    w->in.tag_hold = 0;
    w->in.tag_read = read;
    w->in.tag_lost = lost;
    step(w, 0, 0);
}

/* The one event the load left, and nothing after it. */
static void one_event(struct world *w, uint8_t kind)
{
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(w, &e));
    ASSERT_EQ(e.kind, kind);
    ASSERT_EQ(e.mode, ACE2K_FEED_LOADING);
    ASSERT_TRUE(!pop(w, &e));
}

/* A move the lane core runs for someone else: the next move the load asks for is refused. */
static void foreign_move(struct world *w)
{
    ASSERT_EQ(ace2k_lane_move(&w->lane, 0, ACE2K_LANE_FORWARD, 500000, 30000, w->now), 0);
}

/* Ticks one count back every four ticks while the load is in `sub`, at most `bound` ticks. */
static void back_while(struct world *w, uint8_t sub, int bound)
{
    for (int i = 0; i < bound && w->feed.l[0].sub == sub; i++) {
        step(w, 4, ((i + 1) % 4 == 0) ? -1 : 0);
    }
}

TEST(the_read_with_motion_is_refused_out_of_range_without_a_link_and_after_a_shutdown)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, ACE2K_LANE_COUNT, 0, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    w.in.link_ok = false;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_REFUSED_NO_LINK);
    w.in.link_ok = true;
    step(&w, 0, 0);
    ace2k_feed_shutdown(&w.feed, w.now);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_REFUSED_NO_LINK);
    /* after a shutdown a lane out of range is still a bounds refusal, not a link one */
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, ACE2K_LANE_COUNT, 0, w.now),
              ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

TEST(the_read_with_motion_is_refused_on_a_lane_in_error)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_UNLOAD, 10000, 30000, 1, w.now),
              ACE2K_FEED_ACCEPTED);
    paced(&w, 32, 4, 4, -1); /* the unload's budget spent with the strand still present */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_REFUSED_IN_ERROR);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_TRUE(!w.run[0]);
}

TEST(the_read_with_motion_is_refused_busy_while_the_lane_core_moves_for_someone_else)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    foreign_move(&w);
    step(&w, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]); /* the other move goes on untouched */
}

TEST(the_read_with_motion_with_no_search_set_is_refused_bounds_and_moves_nothing)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(w.feed.search_um, 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_REFUSED_BOUNDS);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
}

TEST(a_lane_out_of_range_has_no_search_end)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, ACE2K_LANE_COUNT), ACE2K_FEED_SEARCH_NONE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0xFF), ACE2K_FEED_SEARCH_NONE);
}

/* A search that would reach less than 2 mm past the parking point does not move. The load ends
 * there, and the search counts as having run its length. */
TEST(a_search_reaching_under_two_millimetres_past_the_park_ends_the_load_at_the_park)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, PARK_UM + 1000U), 0);
    load_pulling(&w, 10);
    uint32_t starts = w.forward_starts[0];
    paced(&w, 972, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    ASSERT_EQ(w.forward_starts[0], starts); /* no move after the pull */
    ASSERT_TRUE(!w.run[0]);
    one_event(&w, ACE2K_FEED_LOADED);
}

/* A pause that ends with under 2 mm of the pull left does not resume the pull. With no search
 * set, the load ends there. */
TEST(a_pull_resumed_within_two_millimetres_of_the_park_ends_loaded_without_a_search)
{
    struct world w;
    world_init(&w);
    load_pulling(&w, 11);
    paced(&w, 968, 4, 4, 1); /* 242 counts: 1.3 mm short of the parking point */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    pause_for_a_session(&w);
    uint32_t starts = w.forward_starts[0];
    release_session(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    ASSERT_EQ(w.forward_starts[0], starts);
    one_event(&w, ACE2K_FEED_LOADED);
}

/* The same pause with a search set goes straight on to the search, for the rest of its reach. */
TEST(a_pull_resumed_within_two_millimetres_of_the_park_goes_on_to_the_search)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_pulling(&w, 12);
    paced(&w, 968, 4, 4, 1);
    pause_for_a_session(&w);
    release_session(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, SEARCH_UM - counts_um(242)));
}

/* A tag lost at the very start of the pull has no room behind the load's start. Here the hand
 * even drew the strand 3 counts back during the pause. The load never reverses past its start,
 * so the tip cannot leave the drive. The pull goes on as if the tag had not been lost. */
TEST(a_tag_lost_at_the_start_of_the_pull_is_not_reacquired_behind_the_start)
{
    struct world w;
    world_init(&w);
    load_pulling(&w, 13);
    pause_for_a_session(&w);
    steps(&w, 3, 0, -1);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, PARK_UM + counts_um(3)));
}

/* A session that finds the tag again during the reacquire pauses it. If the tag is lost once
 * more, the pull resumes forward: a load reacquires only once. */
TEST(a_session_during_the_reacquire_pauses_it_and_a_second_loss_resumes_the_pull)
{
    struct world w;
    world_init(&w);
    load_pulling(&w, 14);
    paced(&w, 400, 4, 4, 1); /* 100 counts of the 243 */
    pause_for_a_session(&w);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    paced(&w, 20, 4, 4, -1); /* 5 counts back */
    pause_for_a_session(&w);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, PARK_UM - counts_um(95)));
}

/* The search reacquires a lost tag and does not find it on the way back. If the tag is read on
 * the tick that reverse ends, the search returns to the parking point as found. */
TEST(a_tag_read_as_the_searchs_reacquire_ends_returns_found)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 15, 150);
    pause_for_a_session(&w);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    w.in.tag_lost = 0;
    w.in.tag_read = 0x01;
    back_while(&w, ACE2K_FEED_SUB_REACQUIRE, 600);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
}

/* A session can also hold the search's reacquire. The pause keeps the phase the reacquire came
 * from, so a read in that session still counts as the search's find. */
TEST(a_tag_read_in_a_session_during_the_searchs_reacquire_returns_found)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 19, 150);
    pause_for_a_session(&w);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    paced(&w, 20, 4, 4, -1); /* 5 counts back */
    pause_for_a_session(&w);
    release_session(&w, 0x01, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_FOUND);
}

/* Another lane starts while a session holds the search. When the session lets go without a
 * read, the search does not go on: it returns to the parking point as a yield. */
TEST(another_lanes_start_during_a_searchs_pause_makes_it_return_as_a_yield)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    w.in.insert = 0x03;
    load_searching(&w, 16, 10);
    pause_for_a_session(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 17, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED); /* the session still holds the lane */
    release_session(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
}

/* The same yield, but the session also lost the tag. A search asked to yield does not try to
 * find the tag again. It goes straight back to the parking point, with no reacquire first. */
TEST(a_yield_with_the_tag_lost_returns_at_once_without_a_reacquire)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    w.in.insert = 0x03;
    load_searching(&w, 20, 10);
    pause_for_a_session(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 21, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 0, 0);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    /* the move goes the 10 counts back to the parking point */
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, counts_um(253) - PARK_UM));
}

/* Each phase of a load is one move of at most ACE2K_LANE_MOVE_MAX_UM. A strand the encoder saw
 * run 2 500 mm past the start comes back by one move of that maximum, not more. */
TEST(a_return_longer_than_one_move_is_cut_to_the_lanes_move_maximum)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 18, 0);
    step(&w, 4, 2000); /* the encoder jumps far past the search's reach in one tick */
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, ACE2K_LANE_MOVE_MAX_UM));
}

/* The next tests start a move of someone else's on the lane while the load waits. The lane then
 * refuses the load's next move. The load ends rather than hangs, and leaves one event. */
TEST(a_pull_the_lane_refuses_after_the_settle_ends_the_load_stopped)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 20, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 49, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SETTLE);
    foreign_move(&w);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_event(&w, ACE2K_FEED_STOPPED);
}

TEST(a_resumed_pull_the_lane_refuses_ends_the_load_loaded)
{
    struct world w;
    world_init(&w);
    load_pulling(&w, 21);
    paced(&w, 400, 4, 4, 1);
    pause_for_a_session(&w);
    foreign_move(&w);
    release_session(&w, 0x01, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_event(&w, ACE2K_FEED_LOADED);
}

TEST(a_reacquire_the_lane_refuses_ends_the_load_loaded)
{
    struct world w;
    world_init(&w);
    load_pulling(&w, 22);
    paced(&w, 400, 4, 4, 1);
    pause_for_a_session(&w);
    foreign_move(&w);
    release_session(&w, 0, 0x01);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_event(&w, ACE2K_FEED_LOADED);
}

TEST(a_search_the_lane_refuses_after_a_resumed_pull_ends_loaded_with_no_search_run)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_pulling(&w, 23);
    paced(&w, 968, 4, 4, 1);
    pause_for_a_session(&w);
    foreign_move(&w);
    release_session(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    one_event(&w, ACE2K_FEED_LOADED);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}

/* The read with motion's retry key.  Klipper resends a query whose answer it did not get.  The same
 * seq on the lane is that retry while the search it started runs, and up to the retry window after
 * that search ended.  Seq 0 is never a key, and any later mode on the lane closes the window. */
TEST(a_read_with_motion_sent_again_with_its_seq_is_the_retry_while_its_search_runs)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 7, w.now), ACE2K_FEED_ACCEPTED);
    steps(&w, 20, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_TRUE(ace2k_feed_search_retry(&w.feed, 0, 7, w.now));
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 0, 8, w.now)); /* another seq: a new read */
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 1, 7, w.now)); /* another lane */
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, ACE2K_LANE_COUNT, 7, w.now));
    /* the window counts from the end of the search, here its stop */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    steps(&w, 100, 0, 0); /* 1000 ms after the stop: still the retry */
    ASSERT_TRUE(ace2k_feed_search_retry(&w.feed, 0, 7, w.now));
    step(&w, 0, 0); /* past the window: a new read */
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 0, 7, w.now));
}

TEST(seq_0_and_a_later_mode_on_the_lane_are_never_the_reads_retry)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 0, 0, w.now)); /* no key: an older host */
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 9, w.now), ACE2K_FEED_ACCEPTED);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_TRUE(ace2k_feed_search_retry(&w.feed, 0, 9, w.now));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 10000, 30000, 3, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 0, 9, w.now)); /* the feed closed it */
}

/* A read with motion whose search has nothing left to run ends at once, loaded, and records no
 * key: the same seq sent again is a new read, not that one's retry. */
TEST(a_read_with_motion_with_nothing_left_to_search_records_no_retry_key)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, ACE2K_FEED_RETURN_MIN_UM - 1U), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 7, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    ASSERT_TRUE(!w.run[0]);
    one_event(&w, ACE2K_FEED_LOADED);
    ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, 0, 7, w.now));
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 7, w.now), ACE2K_FEED_ACCEPTED);
    one_event(&w, ACE2K_FEED_LOADED); /* a second read ran: its own event */
}
