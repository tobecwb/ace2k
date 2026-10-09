#include "test.h"
#include "link/link_turnaround.h"

#define IDLE 100U
#define LOST 3000U /* the silence that is a lost link, in the test's ticks */

TEST(no_byte_ever_seen_starts_at_once)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 5000, &at), ACE2K_LINK_START);
    ASSERT_EQ(t.deferred, 0);
    ASSERT_TRUE(!t.tx_pending);
}

TEST(quiet_line_starts_and_busy_line_waits_until_last_byte_plus_idle)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 1100, &at), ACE2K_LINK_START); /* exactly idle */
    ace2k_link_turnaround_rx(&t, 2000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 2099, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 2100);
    ASSERT_TRUE(t.tx_pending);
    ASSERT_EQ(t.deferred, 1);
}

TEST(timer_starts_when_quiet_and_rearms_while_bytes_keep_arriving)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 2000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 2010, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 2100);
    ace2k_link_turnaround_rx(&t, 2050); /* the burst goes on */
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 2100, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 2150);
    ASSERT_EQ(t.rearmed, 1);
    ASSERT_TRUE(t.tx_pending);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 2150, &at), ACE2K_LINK_START);
    ASSERT_TRUE(!t.tx_pending);
    ASSERT_EQ(t.deferred, 1);
    ASSERT_EQ(t.rearmed, 1);
}

TEST(timer_with_nothing_pending_does_nothing)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 10);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 500, &at), ACE2K_LINK_NOTHING);
    ASSERT_EQ(t.deferred, 0);
    ASSERT_EQ(t.rearmed, 0);
}

TEST(a_request_that_finds_the_line_quiet_ends_a_pending_wait)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 3000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 3001, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 3200, &at), ACE2K_LINK_START); /* late timer */
    ASSERT_TRUE(!t.tx_pending);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 3201, &at), ACE2K_LINK_NOTHING);
    ASSERT_EQ(t.deferred, 1);
}

TEST(second_request_during_a_wait_is_the_same_wait_and_keeps_the_same_wake)
{
    /* deferred counts waits, not calls: the console queues bytes in several calls during one
     * wait (twice when it compacts its buffer, once per frame it adds). */
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    uint32_t at2 = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 4000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 4001, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 4002, &at2), ACE2K_LINK_WAIT);
    ASSERT_EQ(at2, at);
    ASSERT_TRUE(t.tx_pending);
    ASSERT_EQ(t.deferred, 1);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 4100, &at), ACE2K_LINK_START);
    ace2k_link_turnaround_rx(&t, 5000); /* the next wait counts again */
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 5001, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(t.deferred, 2);
}

TEST(the_tick_counter_may_wrap_inside_the_window)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 0xFFFFFFF0U);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 0xFFFFFFF8U, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 0xFFFFFFF0U + IDLE); /* wraps to 0x54 */
    ace2k_link_turnaround_rx(&t, 0x00000010U);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 0x00000054U, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 0x00000010U + IDLE);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 0x00000074U, &at), ACE2K_LINK_START);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 0x00000080U, &at), ACE2K_LINK_START);
}

TEST(a_byte_recorded_after_the_callers_clock_reads_as_busy)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 998, &at), ACE2K_LINK_WAIT); /* now < last_rx */
    ASSERT_EQ(at, 1100);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 999, &at), ACE2K_LINK_WAIT);
    ASSERT_EQ(at, 1100);
    ASSERT_EQ(t.rearmed, 1);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 1100, &at), ACE2K_LINK_START);
}

TEST(a_host_silent_for_longer_than_half_the_counter_range_is_quiet)
{
    /* The signed difference alone reads "busy" again once the host has been silent for 2^31
     * ticks (17.9 s at 120 MHz); the window is bounded on both sides, so a long silence is quiet
     * and nothing is left pending for a timer to find. */
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 1000 + 0x80000000U + 5000, &at),
              ACE2K_LINK_START);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 1000 + 0x80000000U + 1000000, &at),
              ACE2K_LINK_START);
    ASSERT_TRUE(!t.tx_pending);
    ASSERT_EQ(ace2k_link_turnaround_timer(&t, 1000 + 0x80000000U + 1000001, &at),
              ACE2K_LINK_NOTHING);
    ASSERT_EQ(t.deferred, 0);
    ASSERT_EQ(t.rearmed, 0);
}

TEST(a_silence_just_below_the_signed_horizon_is_quiet)
{
    struct ace2k_link_turnaround t;
    uint32_t at = 0;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_EQ(ace2k_link_turnaround_tx_request(&t, 1000 + 0x7FFFFFFFU, &at), ACE2K_LINK_START);
    ASSERT_EQ(t.deferred, 0);
}

TEST(link_ok_is_false_before_the_first_byte_and_true_while_bytes_keep_coming)
{
    struct ace2k_link_turnaround t;
    ace2k_link_turnaround_init(&t, IDLE);
    ASSERT_TRUE(!ace2k_link_turnaround_link_ok(&t, 5000, LOST));
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 1000, LOST));
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 1000 + LOST - 1, LOST));
    ace2k_link_turnaround_rx(&t, 1000 + LOST - 1); /* the host's next query, just in time */
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 1000 + (2 * LOST) - 2, LOST));
    ASSERT_TRUE(!t.lost);
}

TEST(a_silence_of_lost_ticks_latches_the_loss_through_a_counter_wrap_until_the_next_byte)
{
    struct ace2k_link_turnaround t;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 1000 + LOST - 1, LOST));
    ASSERT_TRUE(!ace2k_link_turnaround_link_ok(&t, 1000 + LOST, LOST)); /* exactly lost_ticks */
    ASSERT_TRUE(t.lost);
    ASSERT_TRUE(!ace2k_link_turnaround_link_ok(&t, 1000 + LOST + 1, LOST));
    /* one wrap later the unsigned age reads as ten ticks: the latch holds */
    uint64_t wrapped = 1000ULL + 10ULL + (1ULL << 32U);
    ASSERT_EQ((uint32_t)wrapped - 1000U, 10U);
    ASSERT_TRUE(!ace2k_link_turnaround_link_ok(&t, (uint32_t)wrapped, LOST));
    ASSERT_TRUE(t.lost);
    ace2k_link_turnaround_rx(&t, 9000); /* the host is back */
    ASSERT_TRUE(!t.lost);
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 9010, LOST));
}

TEST(a_loss_that_was_never_evaluated_is_not_latched_by_a_later_byte)
{
    /* the latch is set by a call, not by time: the bindings call every 10 ms, and a byte before
     * the next call means the host was heard from */
    struct ace2k_link_turnaround t;
    ace2k_link_turnaround_init(&t, IDLE);
    ace2k_link_turnaround_rx(&t, 1000);
    ace2k_link_turnaround_rx(&t, 1000 + (2 * LOST));
    ASSERT_TRUE(ace2k_link_turnaround_link_ok(&t, 1000 + (2 * LOST) + 5, LOST));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
