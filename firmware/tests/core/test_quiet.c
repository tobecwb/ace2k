#include "test.h"
#include "core/quiet.h"

TEST(a_running_unit_sends_everything)
{
    bool quiet = false;
    ASSERT_TRUE(!ace2k_quiet_drop(false, 5000U, 0U, &quiet));
}

TEST(a_stopped_unit_sends_its_shutdowns_own_frames_for_the_grace_then_drops_every_one)
{
    bool quiet = false;
    uint32_t shut = 1000U;
    ASSERT_TRUE(!ace2k_quiet_drop(true, shut, shut, &quiet));
    ASSERT_TRUE(!ace2k_quiet_drop(true, shut + ACE2K_QUIET_GRACE_MS - 1U, shut, &quiet));
    ASSERT_TRUE(ace2k_quiet_drop(true, shut + ACE2K_QUIET_GRACE_MS, shut, &quiet));
    ASSERT_TRUE(quiet);
}

/* Once quiet, it stays quiet for as long as the stop lasts, past the clock's half range too. */
TEST(quiet_holds_past_the_wrap_of_the_comparison)
{
    bool quiet = false;
    ASSERT_TRUE(ace2k_quiet_drop(true, 2000U, 0U, &quiet));
    ASSERT_TRUE(ace2k_quiet_drop(true, 0x90000000U, 0U, &quiet));
}

/* Klipper's clear_shutdown, with no reset: the unit reports again. */
TEST(a_cleared_stop_sends_again)
{
    bool quiet = false;
    ASSERT_TRUE(ace2k_quiet_drop(true, 5000U, 0U, &quiet));
    ASSERT_TRUE(!ace2k_quiet_drop(false, 6000U, 0U, &quiet));
    ASSERT_TRUE(!quiet);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
