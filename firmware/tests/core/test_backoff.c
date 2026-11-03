/* The back-off of a store that failed on the flash: no retry before the wait, a wait that doubles
 * from 1 s up to 60 s, and the first wait again after a success. */
#include "test.h"
#include "core/backoff.h"

TEST(a_fresh_backoff_allows_at_once)
{
    struct ace2k_backoff b = ACE2K_BACKOFF_INIT;
    ASSERT_TRUE(!b.failed);
    ASSERT_TRUE(ace2k_backoff_allowed(&b, 0U));
    ASSERT_TRUE(ace2k_backoff_allowed(&b, 123456U));
}

TEST(a_failure_allows_again_only_after_the_wait)
{
    struct ace2k_backoff b = ACE2K_BACKOFF_INIT;
    ace2k_backoff_failed(&b, 5000U);
    ASSERT_TRUE(b.failed);
    ASSERT_TRUE(!ace2k_backoff_allowed(&b, 5000U));
    ASSERT_TRUE(!ace2k_backoff_allowed(&b, 5000U + ACE2K_BACKOFF_MIN_MS - 1U));
    ASSERT_TRUE(ace2k_backoff_allowed(&b, 5000U + ACE2K_BACKOFF_MIN_MS));
}

TEST(the_wait_doubles_from_1_s_to_the_60_s_cap)
{
    struct ace2k_backoff b = ACE2K_BACKOFF_INIT;
    const uint32_t waits[] = { 1000U, 2000U, 4000U, 8000U, 16000U, 32000U, 60000U, 60000U };
    uint32_t now = 0U;
    for (unsigned i = 0; i < sizeof waits / sizeof waits[0]; i++) {
        ace2k_backoff_failed(&b, now);
        ASSERT_EQ(b.retry_ms - now, waits[i]);
        now = b.retry_ms;
    }
}

TEST(a_success_clears_the_failure_and_resets_the_wait)
{
    struct ace2k_backoff b = ACE2K_BACKOFF_INIT;
    ace2k_backoff_failed(&b, 0U);
    ace2k_backoff_failed(&b, 1000U);
    ace2k_backoff_failed(&b, 3000U);
    ace2k_backoff_succeeded(&b);
    ASSERT_TRUE(!b.failed);
    ASSERT_TRUE(ace2k_backoff_allowed(&b, 3001U));
    ace2k_backoff_failed(&b, 10000U);
    ASSERT_EQ(b.retry_ms, 10000U + ACE2K_BACKOFF_MIN_MS);
}

/* The retry time is compared wrap-safe, as every deadline of the tick's clock. */
TEST(the_retry_survives_the_clock_wrap)
{
    struct ace2k_backoff b = ACE2K_BACKOFF_INIT;
    ace2k_backoff_failed(&b, UINT32_MAX - 100U);
    ASSERT_TRUE(!ace2k_backoff_allowed(&b, UINT32_MAX));
    ASSERT_TRUE(ace2k_backoff_allowed(&b, ACE2K_BACKOFF_MIN_MS));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
