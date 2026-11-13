/* What: the load's less common paths, each test named for the rule it pins.
 * How: the shared feed world of fake_feed_world.h, one 10 ms tick at a time.
 * Depends on: test.h, fake_feed_world.h. */
#include "test.h"
#include "fake_feed_world.h"
#include "feed/feed_internal.h"

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

/* A lane the unit holds set by hand never gets a tag: its load ends after the pull with no search.
 * The same load on a lane not set by hand goes on to the search. */
TEST(a_load_on_a_lane_set_by_hand_ends_after_its_pull_with_no_search)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    w.in.tag_by_hand = 0x01;
    load_pulling(&w, 13);
    uint32_t starts = w.forward_starts[0];
    paced(&w, 972, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);
    ASSERT_EQ(w.forward_starts[0], starts); /* no move after the pull */
    ASSERT_TRUE(!w.run[0]);
    one_event(&w, ACE2K_FEED_LOADED);

    struct world v;
    world_init(&v);
    ASSERT_EQ(ace2k_feed_search_set(&v.feed, SEARCH_UM), 0);
    v.in.tag_by_hand = 0x02; /* another lane set by hand */
    load_pulling(&v, 14);
    paced(&v, 972, 4, 4, 1);
    ASSERT_EQ(v.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(v.run[0] && !v.reverse[0]);
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

/* The confirmation of a tag: the reader's watch asks the loading lane to move back and forth, one
 * bounded leg at a time, until the tag follows the lane. */
#define LEG_UM        150000U
#define LEG_UM_S      12000U
#define CONFIRM_COUNT 200U /* counts of the pull before the request: 246.8 mm, room for a leg */

/* The watch asks lane 1 to confirm, toward the spool or toward the head, from the next tick. */
static void ask_confirm(struct world *w, bool reverse)
{
    w->in.tag_confirm = 0x01;
    w->in.tag_confirm_rev = reverse ? 0x01 : 0x00;
    step(w, 0, 0);
}

/* The watch lets the confirmation go on the next tick. */
static void clear_confirm(struct world *w)
{
    w->in.tag_confirm = 0;
    w->in.tag_confirm_rev = 0;
    step(w, 0, 0);
}

/* A pull with CONFIRM_COUNT counts behind it, the confirmation set up. */
static void pull_for_a_confirm(struct world *w, uint8_t seq)
{
    ace2k_feed_confirm_cfg_set(&w->feed, LEG_UM, LEG_UM_S);
    load_pulling(w, seq);
    paced(w, (int)CONFIRM_COUNT * 4, 4, 4, 1);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
}

/* Ticks one count every four ticks along `count_step` until the motor stops, at most `bound`. */
static void run_out(struct world *w, int count_step, int bound)
{
    for (int i = 0; i < bound && w->run[0]; i++) {
        step(w, 4, ((i + 1) % 4 == 0) ? count_step : 0);
    }
}

TEST(a_confirm_request_reverses_the_load_and_its_clearing_resumes_the_pull)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 30);
    ask_confirm(&w, true);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, LEG_UM_S);
    ASSERT_EQ(w.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&w.lane, 0, LEG_UM));
    ASSERT_TRUE(!w.feed.l[0].confirm_out);
    paced(&w, 80, 4, 4, -1); /* 20 counts back */
    /* the reading changed: the watch turns the lane round, and the new leg starts on that tick */
    w.in.tag_confirm_rev = 0;
    uint32_t starts = w.forward_starts[0];
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.forward_starts[0], starts + 1U);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, LEG_UM_S);
    /* forward, never past the end of the pull */
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, PARK_UM - counts_um(CONFIRM_COUNT - 20U)));
    paced(&w, 40, 4, 4, 1); /* 10 counts on */
    clear_confirm(&w);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, ACE2K_FEED_LOAD_SPEED_UM_S);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, PARK_UM - counts_um(CONFIRM_COUNT - 10U)));
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e)); /* nothing ended */
}

TEST(a_confirm_reverse_never_goes_behind_the_phase_start)
{
    /* In the pull: 20 counts in, the leg goes back those 20 counts, no more. */
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    load_pulling(&w, 31);
    paced(&w, 80, 4, 4, 1);
    ask_confirm(&w, true);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, counts_um(20)));
    ASSERT_TRUE(!w.feed.l[0].confirm_out);

    /* At the very start of the pull there is no room: no move, and the leg is out at once. */
    struct world v;
    world_init(&v);
    ace2k_feed_confirm_cfg_set(&v.feed, LEG_UM, LEG_UM_S);
    load_pulling(&v, 32);
    paced(&v, 4, 4, 4, 1); /* 1 count: 1.2 mm */
    ask_confirm(&v, true);
    ASSERT_EQ(v.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(!v.run[0]);
    ASSERT_TRUE(v.feed.l[0].confirm_out);
    steps(&v, 10, 0, 0); /* it holds still while the watch still asks */
    ASSERT_TRUE(!v.run[0]);
    ASSERT_EQ(v.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);

    /* In the search the floor is the parking point: 10 counts past it, the leg goes back to it. */
    struct world s;
    world_init(&s);
    ace2k_feed_confirm_cfg_set(&s.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&s.feed, SEARCH_UM), 0);
    load_searching(&s, 33, 10);
    ask_confirm(&s, true);
    ASSERT_EQ(s.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(s.run[0] && s.reverse[0]);
    ASSERT_EQ(s.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&s.lane, 0, counts_um(253) - PARK_UM));
    /* forward in the search: never past the search's reach */
    s.in.tag_confirm_rev = 0;
    step(&s, 0, 0);
    ASSERT_TRUE(s.run[0] && !s.reverse[0]);
    ASSERT_EQ(s.lane.lane[0].move.target_counts, ace2k_lane_um_to_counts(&s.lane, 0, LEG_UM));
}

TEST(a_confirm_leg_that_runs_out_raises_confirm_out)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 34);
    ask_confirm(&w, true);
    run_out(&w, -1, 1000);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    uint32_t starts = w.forward_starts[0];
    steps(&w, 20, 0, 0); /* held still until the watch lets go */
    ASSERT_TRUE(!w.run[0]);
    ASSERT_EQ(w.forward_starts[0], starts);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    clear_confirm(&w);
    ASSERT_TRUE(!w.feed.l[0].confirm_out);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e));
}

/* A leg ran out and the lane holds still.  The watch starts a new confirmation on
 * the lane, asking the same direction: its new epoch starts a fresh leg, and the old out goes. */
TEST(a_new_confirmation_epoch_starts_a_fresh_leg_in_the_same_direction)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 36);
    ask_confirm(&w, true);
    run_out(&w, -1, 1000);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    ASSERT_TRUE(!w.feed.l[0].confirm_epoch);
    w.in.tag_confirm_epoch = 0x01; /* a new confirmation, toward the spool again */
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_TRUE(!w.feed.l[0].confirm_out);
    ASSERT_TRUE(w.feed.l[0].confirm_epoch);
    steps(&w, 5, 0, 0);
    ASSERT_TRUE(w.run[0]); /* the same epoch: the leg runs on */
}

TEST(a_stop_or_a_link_drop_during_a_confirm_ends_the_load_as_today)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 35);
    ask_confirm(&w, true);
    paced(&w, 8, 4, 4, -1);
    ace2k_feed_stop(&w.feed, 0, w.now);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(!w.feed.l[0].confirm_out);
    one_event(&w, ACE2K_FEED_STOPPED);
    step(&w, 0, 0); /* the request still set on an idle lane moves nothing */
    ASSERT_TRUE(!w.run[0]);

    struct world v;
    world_init(&v);
    pull_for_a_confirm(&v, 36);
    ask_confirm(&v, true);
    paced(&v, 8, 4, 4, -1);
    v.in.link_ok = false;
    step(&v, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&v.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!v.run[0]);
    one_event(&v, ACE2K_FEED_STOPPED_LINK);

    /* a leg already out, held still with no move: the link drop ends it all the same */
    struct world o;
    world_init(&o);
    ace2k_feed_confirm_cfg_set(&o.feed, LEG_UM, LEG_UM_S);
    load_pulling(&o, 37);
    ask_confirm(&o, true);
    ASSERT_TRUE(o.feed.l[0].confirm_out && !o.run[0]);
    o.in.link_ok = false;
    step(&o, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&o.feed, 0), ACE2K_FEED_IDLE);
    one_event(&o, ACE2K_FEED_STOPPED_LINK);
    ASSERT_TRUE(!o.feed.l[0].confirm_out);

    /* the strand withdrawn from the bay during a leg: the runout, as in the pull */
    struct world r;
    world_init(&r);
    pull_for_a_confirm(&r, 38);
    ask_confirm(&r, true);
    r.in.insert = 0;
    step(&r, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&r.feed, 0), ACE2K_FEED_IDLE);
    one_event(&r, ACE2K_FEED_RUNOUT);
}

TEST(a_session_hold_during_a_confirm_pauses_and_resumes_the_original_phase)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 39, 40);
    ask_confirm(&w, true);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    paced(&w, 20, 4, 4, -1); /* 5 counts back */
    /* the session opens on the reader: the watch drops the confirmation, the session holds */
    w.in.tag_confirm = 0;
    w.in.tag_confirm_rev = 0;
    pause_for_a_session(&w);
    ASSERT_EQ(w.feed.l[0].resume_sub, ACE2K_FEED_SUB_SEARCH);
    release_session(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, SEARCH_UM - counts_um(243U + 40U - 5U)));

    /* The same hold with the request still up: the hold comes first, and a read in the session
     * ends the search as found. */
    struct world v;
    world_init(&v);
    ace2k_feed_confirm_cfg_set(&v.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&v.feed, SEARCH_UM), 0);
    load_searching(&v, 40, 40);
    ask_confirm(&v, true);
    pause_for_a_session(&v);
    ASSERT_EQ(v.feed.l[0].resume_sub, ACE2K_FEED_SUB_SEARCH);
    v.in.tag_confirm = 0;
    release_session(&v, 0x01, 0);
    ASSERT_EQ(v.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&v.feed, 0), ACE2K_FEED_SEARCH_FOUND);
}

/* Each leg runs under the supervisors of its phase.  A forward leg in the pull whose filament does
 * not follow is the pull's obstacle: blocked.  A reverse leg whose filament will not come back is
 * held: stuck, as the reacquire. */
TEST(a_confirm_leg_is_judged_by_the_phases_supervisors)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 41);
    ask_confirm(&w, false);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    steps(&w, 200, 4, 0); /* the motor turns, the filament stays */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    one_event(&w, ACE2K_FEED_BLOCKED);

    struct world v;
    world_init(&v);
    pull_for_a_confirm(&v, 42);
    ask_confirm(&v, true);
    ASSERT_TRUE(v.run[0] && v.reverse[0]);
    steps(&v, 200, 4, 0);
    ASSERT_EQ(ace2k_feed_mode(&v.feed, 0), ACE2K_FEED_ERROR);
    one_event(&v, ACE2K_FEED_STUCK);
}

/* The read with motion keeps the tip snag's watch on a forward leg, as in its search.  The touch
 * frees the plunger, and the confirmation goes on with a fresh leg in the direction asked then. */
TEST(a_forward_confirm_leg_of_the_read_with_motion_keeps_the_tip_snags_touch)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 50, w.now), ACE2K_FEED_ACCEPTED);
    paced(&w, 120, 4, 4, 1); /* 30 counts */
    ask_confirm(&w, false);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    w.in.pulled_any = true; /* the tip drags the plunger to its pulled end */
    w.in.rest = 0x0E;
    for (int i = 0; i < 200 && w.feed.l[0].snag != ACE2K_FEED_SNAG_BACK; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? 1 : 0); /* the strand follows until the tolerance */
    }
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_BACK);
    ASSERT_TRUE(w.run[0] && w.reverse[0]); /* the touch */
    run_out(&w, -1, 200);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_REST);
    w.in.pulled_any = false; /* the plunger back at rest */
    w.in.rest = 0x0F;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, LEG_UM_S);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_SNAG);
    ASSERT_TRUE(!pop(&w, &e));
}

/* A forward leg in the search stops at the search's reach when that is nearer than a leg. */
TEST(a_forward_confirm_leg_in_the_search_stops_at_the_searchs_reach)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, PARK_UM + 50000U), 0);
    load_searching(&w, 43, 10);
    ask_confirm(&w, false);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.lane.lane[0].move.target_counts,
              ace2k_lane_um_to_counts(&w.lane, 0, PARK_UM + 50000U - counts_um(253)));
}

/* A reverse leg drops the free phase of the push toward the head, so a stall on it is the motor's,
 * as on the reacquire: an error, not the search's obstacle return. */
TEST(a_reverse_confirm_leg_that_stalls_ends_as_other_reverse_moves_do)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 44, 20);
    w.in.pulled_any = true; /* the plunger at its pulled end: the free phase */
    w.in.rest = 0x0E;
    paced(&w, 4, 4, 4, 1);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_FREE_TIP);
    ask_confirm(&w, true);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    steps(&w, 150, 0, 0); /* the tach silent */
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_ERROR);
    ASSERT_EQ(ace2k_feed_error(&w.feed, 0), ACE2K_FEED_MOTOR_STALLED);
    one_event(&w, ACE2K_FEED_MOTOR_STALLED);
}

/* The pull of a load from the bay is in the free phase when a confirmation turns
 * it back.  The reverse leg sets the phase aside; the forward leg after it takes it up again, with
 * a fresh window, though the buffer no longer reads full (the plunger is not back at rest).  So
 * does the pull when the confirmation lets it go after a reverse leg. */
TEST(a_reverse_confirm_leg_sets_the_free_phase_aside_until_the_next_push)
{
    for (int resume = 0; resume < 2; resume++) {
        struct world w;
        world_init(&w);
        pull_for_a_confirm(&w, (uint8_t)(50 + resume));
        w.in.pulled_any = true; /* the plunger at its pulled end: the free phase */
        w.in.rest = 0x0E;
        paced(&w, 4, 4, 4, 1);
        ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_FREE_TIP);
        ask_confirm(&w, true);
        ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
        w.in.pulled_any = false; /* not full any more, and not at rest */
        paced(&w, 40, 4, 4, -1);
        ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
        if (resume != 0) {
            clear_confirm(&w);
            ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_MOVING);
        } else {
            ask_confirm(&w, false);
            ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
        }
        ASSERT_TRUE(w.run[0] && !w.reverse[0]);
        ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_FREE_TIP);
        ASSERT_EQ(w.feed.l[0].enc_um_snag, ace2k_lane_encoder_um(&w.lane, 0)); /* a fresh window */
    }
}

/* The free phase set aside by a reverse leg is not taken up when the plunger is back at rest: the
 * phase is over, and no notice goes out for it. */
TEST(the_free_phase_set_aside_is_dropped_once_the_plunger_is_at_rest)
{
    struct world w;
    world_init(&w);
    pull_for_a_confirm(&w, 52);
    w.in.pulled_any = true;
    w.in.rest = 0x0E;
    paced(&w, 4, 4, 4, 1);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_FREE_TIP);
    ask_confirm(&w, true);
    w.in.pulled_any = false;
    w.in.rest = 0x0F; /* back at rest as the lane went back */
    paced(&w, 40, 4, 4, -1);
    ask_confirm(&w, false);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(!pop(&w, &e)); /* no snag notice */
}

/* A forward leg in the search whose filament does not follow takes the search's obstacle return.
 * The load left the confirmation while the watch still asks, so its leg is out at once; the watch
 * letting go clears it. */
TEST(a_forward_confirm_leg_in_the_search_that_trips_takes_the_obstacle_return)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    load_searching(&w, 45, 20);
    ask_confirm(&w, false);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    for (int i = 0; i < 300 && w.feed.l[0].sub == ACE2K_FEED_SUB_CONFIRM; i++) {
        step(&w, 4, 0); /* the motor turns, the filament stays */
    }
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_LOADING);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_OBSTACLE);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    clear_confirm(&w);
    ASSERT_TRUE(!w.feed.l[0].confirm_out);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
}

/* Another lane's start while a leg runs in the search: the search yields, as from the search
 * itself, and the leg is out for the watch. */
TEST(a_yield_during_a_confirm_in_the_search_returns_and_raises_confirm_out)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    w.in.insert = 0x03;
    load_searching(&w, 46, 20);
    ask_confirm(&w, true);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 47, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
}

/* A request in the settle, the return or the reacquire starts no leg: the phase runs on. */
TEST(a_confirm_request_in_the_settle_the_return_or_the_reacquire_is_ignored)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_LOAD, 0, 0, 48, w.now),
              ACE2K_FEED_ACCEPTED);
    steps(&w, 10, 0, 0);
    ask_confirm(&w, true);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SETTLE);
    ASSERT_TRUE(!w.run[0]);

    struct world r;
    world_init(&r);
    ace2k_feed_confirm_cfg_set(&r.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_search_set(&r.feed, SEARCH_UM), 0);
    load_searching(&r, 49, 10);
    pause_for_a_session(&r);
    release_session(&r, 0x01, 0); /* read: the return */
    ASSERT_EQ(r.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    uint32_t target = r.lane.lane[0].move.target_counts;
    ask_confirm(&r, false);
    ASSERT_EQ(r.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(r.run[0] && r.reverse[0]);
    ASSERT_EQ(r.lane.lane[0].move.target_counts, target);
    ASSERT_TRUE(!r.feed.l[0].confirm_out);

    struct world q;
    world_init(&q);
    ace2k_feed_confirm_cfg_set(&q.feed, LEG_UM, LEG_UM_S);
    load_pulling(&q, 50);
    paced(&q, 400, 4, 4, 1);
    pause_for_a_session(&q);
    release_session(&q, 0, 0x01); /* lost: the reacquire */
    ASSERT_EQ(q.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    target = q.lane.lane[0].move.target_counts;
    ask_confirm(&q, false);
    ASSERT_EQ(q.feed.l[0].sub, ACE2K_FEED_SUB_REACQUIRE);
    ASSERT_TRUE(q.run[0] && q.reverse[0]);
    ASSERT_EQ(q.lane.lane[0].move.target_counts, target);
}

/* Lane 2's plunger: at rest, or taut (its follow's burst runs). */
static void lane2_rest(struct world *w)
{
    w->in.rest = 0x0F;
    w->in.pushed = 0x00;
}

static void lane2_taut(struct world *w)
{
    w->in.rest = 0x0D;
    w->in.pushed = 0x02;
}

/* Lane 2 printing: its follow armed, at rest, filament in both lanes. */
static void lane2_following(struct world *w, uint8_t seq)
{
    ASSERT_EQ(ace2k_feed_search_set(&w->feed, SEARCH_UM), 0);
    w->in.insert = 0x03;
    step(w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w->feed, 1, ACE2K_FEED_CMD_ASSIST_BOTH, 0, 50000, seq, w->now),
              ACE2K_FEED_ACCEPTED);
    step(w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w->feed, 1), ACE2K_FEED_FOLLOWING);
    ASSERT_TRUE(!w->run[1]);
}

/* The events of lane 1 in order, skipping lane 2's: the next one, or false. */
static bool pop_lane0(struct world *w, struct ace2k_feed_event *e)
{
    while (pop(w, e)) {
        if (e->lane == 0) {
            return true;
        }
    }
    return false;
}

/* A neighbour in use (a print's follow) neither keeps the load's search from starting nor makes it
 * yield when its motor runs: the search runs its whole reach and comes back. */
TEST(a_search_runs_and_goes_on_while_an_in_use_neighbour_follows)
{
    struct world w;
    world_init(&w);
    w.in.in_use = 0x02;
    w.in.in_use_valid = true;
    lane2_following(&w, 60);
    load_searching(&w, 61, 10); /* the search starts beside the follow */
    lane2_taut(&w);
    step(&w, 4, 0);
    ASSERT_TRUE(w.run[1]); /* the follow's burst runs */
    for (int i = 0; i < 20; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? 1 : 0);
        ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
        ASSERT_TRUE(w.run[0] && !w.reverse[0]);
    }
    ASSERT_TRUE(w.run[1]);
    ASSERT_TRUE(!w.feed.l[0].yield);
    lane2_rest(&w);
    for (int i = 0; i < 1200 && w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? 1 : 0); /* the rest of the reach */
    }
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    back_while(&w, ACE2K_FEED_SUB_RETURN, 1200);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop_lane0(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 61);
    ace2k_feed_stop(&w.feed, ACE2K_LANE_ALL, w.now);
}

/* With no valid mask (in_use 0: none, stale, or the link dropped), today's rule holds: the load's
 * search waits for every other lane idle, and a read with motion yields to a moving follow. */
TEST(with_no_valid_mask_a_moving_neighbour_still_makes_the_search_yield)
{
    struct world w;
    world_init(&w);
    lane2_following(&w, 62);
    load_pulling(&w, 63);
    paced(&w, 972, 4, 4, 1);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE); /* no search beside the follow */
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_NONE);

    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_ACCEPTED);
    paced(&w, 40, 4, 4, 1);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    lane2_taut(&w);
    step(&w, 4, 0);
    ASSERT_TRUE(w.run[1]);
    if (w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH) {
        step(&w, 4, 0);
    }
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ace2k_feed_stop(&w.feed, ACE2K_LANE_ALL, w.now);
}

/* The tie-break on lane 1, 10 counts into its search. */
static void tiebreak_searching(struct world *w)
{
    ASSERT_EQ(ace2k_feed_search_set(&w->feed, SEARCH_UM), 0);
    ace2k_feed_confirm_cfg_set(&w->feed, LEG_UM, LEG_UM_S); /* the reader's binding sets it */
    w->in.in_use_valid = true;                              /* the host's mask, nothing in use */
    step(w, 0, 0);
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w->feed, 0, w->now), ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(w->feed.l[0].autonomous);
    paced(w, 40, 4, 4, 1);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w->run[0] && !w->reverse[0]);
}

/* A host start on the lane while its tie-break runs: refused busy, as a host's read is, and the
 * tie-break goes back to where it began and ends with seq 0.  Nothing waits: once the lane is idle,
 * the host's resend starts. */
TEST(a_host_start_during_the_tiebreak_is_refused_busy_and_sends_it_back)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 70, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN); /* back at once */
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 70, w.now),
              ACE2K_FEED_REFUSED_BUSY); /* still on its way back */
    back_while(&w, ACE2K_FEED_SUB_RETURN, 200);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(e.filament_um, 0); /* exactly what it pushed, back */
    ASSERT_TRUE(!pop(&w, &e));
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 70, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_FEEDING);
    ASSERT_EQ(w.feed.l[0].seq, 70);
}

/* A stop armed on the lane while its tie-break runs is refused busy and sends the tie-break back,
 * as a start does.  Once the lane is idle, the host's resend arms it. */
TEST(a_stop_armed_during_the_tiebreak_is_refused_busy_and_sends_it_back)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0), -ACE2K_EBUSY);
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    back_while(&w, ACE2K_FEED_SUB_RETURN, 200);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(ace2k_feed_stop_when(&w.feed, 0, true, false, 0), 0);
}

/* The print takes the lane: the tie-break goes back at once, and never forward again. */
TEST(the_lane_marked_in_use_during_its_tiebreak_returns_at_once)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    w.in.in_use = 0x01;
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    uint32_t starts = w.forward_starts[0];
    back_while(&w, ACE2K_FEED_SUB_RETURN, 200);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_EQ(w.forward_starts[0], starts);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 0);
    ASSERT_EQ(e.filament_um, 0);
}

/* The tie-break takes the read with motion's refusals, records no retry key, keeps the grip, and
 * runs with no free tip; a search with nothing to run leaves it unmarked. */
TEST(the_tiebreak_is_the_read_with_motion_with_seq_0_and_its_refusals)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    /* its reach is the confirmation's longest leg: with none set, nothing to run */
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_BOUNDS);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, LEG_UM_S);
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w.feed, 1, w.now), ACE2K_FEED_REFUSED_NO_FILAMENT);
    w.in.insert = 0x03;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_ASSIST, 0, 30000, 72, w.now),
              ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_OTHER_LANE);
    w.in.in_use = 0x02; /* the same assist, now a print's */
    w.in.in_use_valid = true;
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_grip_set(&w.feed, 0, 30000), 0);
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w.feed, 0, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_EQ(w.feed.grip_um[0], 30000);
    ASSERT_EQ(w.feed.l[0].seq, 0);
    ASSERT_EQ(w.feed.l[0].search_seq, 0);
    ASSERT_TRUE(!w.feed.l[0].free_tip);
    ASSERT_TRUE(w.feed.l[0].autonomous);
    ASSERT_EQ(ace2k_feed_tiebreak_start(&w.feed, 0, w.now), ACE2K_FEED_REFUSED_BUSY);
    ace2k_feed_stop(&w.feed, ACE2K_LANE_ALL, w.now);
    ASSERT_TRUE(!ace2k_feed_tiebreak_on(&w.feed.l[0]));
}

/* The read with motion of the host is not the tie-break: a start on its lane is busy as ever. */
TEST(a_host_start_during_the_hosts_own_read_with_motion_is_still_busy)
{
    struct world w;
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 73, w.now), ACE2K_FEED_ACCEPTED);
    ASSERT_TRUE(!w.feed.l[0].autonomous);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 74, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
}

/* The host's mask goes stale (or the link restarts it): nothing may move on its own, so the
 * tie-break goes back on that tick.  A read with motion the host started is not affected. */
TEST(a_tiebreak_goes_back_when_the_mask_becomes_invalid_and_a_hosts_read_does_not)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    w.in.in_use_valid = false;
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    back_while(&w, ACE2K_FEED_SUB_RETURN, 200);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.filament_um, 0);

    world_init(&w); /* no mask ever: the host's own read runs as before */
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, SEARCH_UM), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 90, w.now), ACE2K_FEED_ACCEPTED);
    paced(&w, 80, 4, 4, 1);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    ASSERT_TRUE(w.run[0] && !w.reverse[0]);
}

/* The tie-break 90 counts in, its plunger then full: the tip snag's watch holds the load. */
static void tiebreak_in_the_snag_watch(struct world *w)
{
    tiebreak_searching(w);
    paced(w, 320, 4, 4, 1); /* 90 counts, 111 mm: short of the tie-break's 150 mm */
    w->in.pulled_any = true;
    w->in.rest = 0x0E;
    paced(w, 20, 4, 4, 1);
    ASSERT_EQ(w->feed.l[0].snag, ACE2K_FEED_SNAG_WATCH);
}

/* Back to rest and the return run home: the tie-break's own end, seq 0. */
static void tiebreak_home(struct world *w)
{
    w->in.pulled_any = false;
    w->in.rest = 0x0F;
    back_while(w, ACE2K_FEED_SUB_RETURN, 2000);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.seq, 0);
}

/* The lane marked in use, or a host start, during the tip snag's watch: back on that tick, the
 * snag's phase ended. */
TEST(the_in_use_preempt_and_a_host_start_end_the_tip_snags_watch_at_once)
{
    struct world w;
    world_init(&w);
    tiebreak_in_the_snag_watch(&w);
    uint32_t starts = w.forward_starts[0];
    w.in.in_use = 0x01;
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    tiebreak_home(&w);
    ASSERT_EQ(w.forward_starts[0], starts);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);

    world_init(&w);
    tiebreak_in_the_snag_watch(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_ROLLBACK, 20000, 30000, 102, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(w.feed.l[0].snag, ACE2K_FEED_SNAG_NONE);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    tiebreak_home(&w);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

/* The tie-break in a forward confirmation leg of its search. */
static void tiebreak_in_a_confirm_leg(struct world *w)
{
    ace2k_feed_confirm_cfg_set(&w->feed, LEG_UM, LEG_UM_S);
    tiebreak_searching(w);
    paced(w, 80, 4, 4, 1);
    ask_confirm(w, true);
    ASSERT_EQ(w->feed.l[0].sub, ACE2K_FEED_SUB_CONFIRM);
    ASSERT_TRUE(w->run[0] && w->reverse[0]);
}

/* The lane marked in use, or a host start, during a confirmation leg: back on that tick, with the
 * leg out for the watch. */
TEST(the_in_use_preempt_and_a_host_start_end_a_confirm_leg_at_once)
{
    struct world w;
    world_init(&w);
    tiebreak_in_a_confirm_leg(&w);
    w.in.in_use = 0x01;
    step(&w, 0, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    tiebreak_home(&w);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);

    world_init(&w);
    tiebreak_in_a_confirm_leg(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 0, ACE2K_FEED_CMD_FEED, 20000, 30000, 103, w.now),
              ACE2K_FEED_REFUSED_BUSY);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_TRUE(w.feed.l[0].confirm_out);
    w.in.tag_confirm = 0;
    tiebreak_home(&w);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

/* The tie-break goes forward at the confirmation's speed, and no farther than
 * the confirmation's longest leg from where it began; then back exactly what it pushed, and the
 * lane is idle.  Its search ran its whole reach, yet it is no no-tag: a tie-break is too short for
 * a spool's turn, so the lane stays pending for the watch. */
TEST(the_tiebreak_goes_forward_at_most_a_confirmation_leg_at_its_speed)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    ASSERT_EQ(w.lane.lane[0].move.speed_um_s, LEG_UM_S);
    uint32_t reach_counts = ace2k_lane_um_to_counts(&w.lane, 0, LEG_UM);
    for (int i = 0; i < 4000 && w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? 1 : 0);
    }
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    ASSERT_TRUE(w.enc[0] <= reach_counts + 1U); /* never past the reach */
    back_while(&w, ACE2K_FEED_SUB_RETURN, 4000);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    struct ace2k_feed_event e;
    ASSERT_TRUE(pop(&w, &e));
    ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
    ASSERT_EQ(e.filament_um, 0);
    ASSERT_TRUE(!ace2k_feed_search_exhausted(&w.feed, 0));

    /* the host's own read with motion that runs its reach is exhausted, as before */
    world_init(&w);
    ASSERT_EQ(ace2k_feed_search_set(&w.feed, 40000), 0);
    ASSERT_EQ(ace2k_feed_search_start(&w.feed, 0, 0, w.now), ACE2K_FEED_ACCEPTED);
    for (int i = 0; i < 4000 && w.feed.l[0].sub == ACE2K_FEED_SUB_SEARCH; i++) {
        step(&w, 4, ((i + 1) % 4 == 0) ? 1 : 0);
    }
    back_while(&w, ACE2K_FEED_SUB_RETURN, 4000);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_LENGTH);
    ASSERT_TRUE(ace2k_feed_search_exhausted(&w.feed, 0));
}

/* A host start on the other lane of the reader ends the tie-break at once, even on a lane the host
 * has in use; the tie-break goes back.  A start on the other reader's in-use lane leaves it
 * alone. */
TEST(a_host_start_on_the_other_lane_of_the_reader_ends_the_tiebreak_even_in_use)
{
    struct world w;
    world_init(&w);
    w.in.insert = 0x07;
    w.in.in_use = 0x06; /* lanes 2 and 3 print */
    tiebreak_searching(&w);
    ASSERT_EQ(ace2k_feed_start(&w.feed, 2, ACE2K_FEED_CMD_FEED, 20000, 30000, 110, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH); /* the other reader: goes on */
    ASSERT_EQ(ace2k_feed_start(&w.feed, 1, ACE2K_FEED_CMD_FEED, 20000, 30000, 111, w.now),
              ACE2K_FEED_ACCEPTED);
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    ASSERT_TRUE(w.run[0] && w.reverse[0]);
    back_while(&w, ACE2K_FEED_SUB_RETURN, 400);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

/* A strand going in at the other lane of the reader ends the tie-break at once,
 * with the automatic load off; one going in at the other reader does not. */
TEST(a_strand_going_in_beside_the_tiebreak_ends_it)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    w.in.insert = 0x05; /* lane 3: the other reader */
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_SEARCH);
    w.in.insert = 0x07; /* lane 2, beside it */
    step(&w, 4, 0);
    step(&w, 4, 0);
    ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
    ASSERT_EQ(ace2k_feed_search_end(&w.feed, 0), ACE2K_FEED_SEARCH_YIELD);
    back_while(&w, ACE2K_FEED_SUB_RETURN, 400);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
}

/* A stop during the tie-break leaves the lane where it stopped (STOP means stop). */
TEST(a_stop_during_the_tiebreak_leaves_the_lane_where_it_is)
{
    struct world w;
    world_init(&w);
    tiebreak_searching(&w);
    ace2k_feed_stop(&w.feed, 0, w.now);
    step(&w, 0, 0);
    ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
    ASSERT_TRUE(!w.run[0]);
    ASSERT_TRUE(w.enc[0] > 0U);
}

/* The tie-break on lane 1 with lane 2 holding filament too. */
static void tiebreak_beside_a_strand(struct world *w)
{
    w->in.insert = 0x03;
    tiebreak_searching(w);
    struct ace2k_feed_event e;
    while (pop(w, &e)) {
    }
}

/* A print's load ends its pull while lane 1's tie-break still runs.  On lane 2, the same reader
 * (its start sent the tie-break back, which has not reached home): the load does not wait, it ends
 * with no search.  On lane 3, the other reader (in use, so its start asks nothing of the tie-break,
 * which a read session holds still): the load searches, and the tie-break goes on. */
TEST(a_loads_pull_ending_during_a_tiebreak_skips_its_search_on_the_same_reader_only)
{
    for (uint8_t lane = 1; lane < 3; lane++) {
        struct world w;
        world_init(&w);
        w.in.insert = (uint8_t)(0x01U | (1U << lane));
        tiebreak_searching(&w);
        w.in.tag_hold = 0x01;
        w.in.in_use = (uint8_t)(1U << lane);
        step(&w, 0, 0);
        ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED);
        ASSERT_EQ(ace2k_feed_start(&w.feed, lane, ACE2K_FEED_CMD_LOAD, 10000, 0, 60, w.now),
                  ACE2K_FEED_ACCEPTED);
        for (int i = 0; i < 400 && ace2k_feed_mode(&w.feed, lane) == ACE2K_FEED_LOADING &&
                        w.feed.l[lane].sub != ACE2K_FEED_SUB_SEARCH;
             i++) {
            bool pull = (bool)(w.run[lane] && !w.reverse[lane]);
            w.fg[lane] += pull ? 4U : 0U;
            w.enc[lane] = (uint16_t)(w.enc[lane] + ((pull && (i % 4 == 3)) ? 1 : 0));
            step(&w, 0, 0);
        }
        if (lane == 1) {
            ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN); /* not home: no pulses on lane 1 */
            ASSERT_EQ(ace2k_feed_mode(&w.feed, lane), ACE2K_FEED_IDLE);
            ASSERT_EQ(ace2k_feed_search_end(&w.feed, lane), ACE2K_FEED_SEARCH_NONE);
        } else {
            ASSERT_EQ(w.feed.l[lane].sub, ACE2K_FEED_SUB_SEARCH);
            ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_PAUSED); /* the tie-break goes on */
        }
    }
}

/* A host's read with motion while the tie-break runs, on another lane or on the tie-break's own:
 * refused busy, nothing waits, and the tie-break goes back at once.  Once it has ended, the same
 * read is accepted and runs with its own key. */
TEST(a_hosts_read_during_the_tiebreak_is_refused_busy_and_sends_it_back)
{
    for (uint8_t lane = 0; lane < 2; lane++) {
        struct world w;
        world_init(&w);
        tiebreak_beside_a_strand(&w);
        ASSERT_EQ(ace2k_feed_search_start(&w.feed, lane, 9, w.now), ACE2K_FEED_REFUSED_BUSY);
        ASSERT_TRUE(!ace2k_feed_search_retry(&w.feed, lane, 9, w.now));
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 1), ACE2K_FEED_IDLE);
        ASSERT_EQ(w.feed.l[0].sub, ACE2K_FEED_SUB_RETURN);
        ASSERT_EQ(ace2k_feed_search_start(&w.feed, lane, 9, w.now), ACE2K_FEED_REFUSED_BUSY);
        back_while(&w, ACE2K_FEED_SUB_RETURN, 200);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, 0), ACE2K_FEED_IDLE);
        struct ace2k_feed_event e;
        ASSERT_TRUE(pop(&w, &e)); /* the tie-break's own end, and nothing else */
        ASSERT_EQ(e.lane, 0);
        ASSERT_EQ(e.kind, ACE2K_FEED_LOADED);
        ASSERT_EQ(e.seq, 0);
        ASSERT_TRUE(!pop(&w, &e));
        step(&w, 0, 0);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, lane), ACE2K_FEED_IDLE); /* nothing ran on its own */
        ASSERT_EQ(ace2k_feed_search_start(&w.feed, lane, 9, w.now), ACE2K_FEED_ACCEPTED);
        ASSERT_EQ(ace2k_feed_mode(&w.feed, lane), ACE2K_FEED_LOADING);
        ASSERT_TRUE(!w.feed.l[lane].autonomous);
        ASSERT_EQ(w.feed.l[lane].search_seq, 9);
    }
}

/* The confirmation's settings that the reader's binding forwards are taken within the lane's
 * bounds: the leg at most one move, the speed between the lane's floor and its top.  A lane whose
 * last search did not run its reach is not exhausted. */
TEST(the_confirmation_settings_are_held_within_the_lanes_bounds)
{
    struct world w;
    world_init(&w);
    ace2k_feed_confirm_cfg_set(&w.feed, ACE2K_LANE_MOVE_MAX_UM + 1U,
                               ACE2K_LANE_SPEED_MIN_UM_S - 1U);
    ASSERT_EQ(w.feed.confirm_leg_um, ACE2K_LANE_MOVE_MAX_UM);
    ASSERT_EQ(w.feed.confirm_speed_um_s, ACE2K_LANE_SPEED_MIN_UM_S);
    ace2k_feed_confirm_cfg_set(&w.feed, LEG_UM, ACE2K_LANE_SPEED_MAX_UM_S + 1U);
    ASSERT_EQ(w.feed.confirm_leg_um, LEG_UM);
    ASSERT_EQ(w.feed.confirm_speed_um_s, ACE2K_LANE_SPEED_MAX_UM_S);
    ASSERT_TRUE(!ace2k_feed_search_exhausted(&w.feed, 0)); /* no search ran */
}
