#include "test.h"
#include "lane/led.h"

struct ace2k_fake_leds {
    int writes;
    uint8_t mask;
};

static void fake_set_mask(void *ctx, uint8_t on_mask)
{
    struct ace2k_fake_leds *f = ctx;
    f->writes++;
    f->mask = on_mask;
}

static const struct ace2k_led_ops fake_ops = { .set_mask = fake_set_mask };

TEST(init_writes_everything_off_once)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0xFF };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ASSERT_EQ(f.writes, 1);
    ASSERT_EQ(f.mask, 0);
    ace2k_led_tick(&led, 500U);
    ASSERT_EQ(f.writes, 1);
}

TEST(waiting_host_chases_one_lane_at_a_time)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 1000U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 1000U);
    ASSERT_EQ(f.mask, 0x01);
    ASSERT_EQ(ace2k_led_mask(&led, 1249U), 0x01);
    ASSERT_EQ(ace2k_led_mask(&led, 1250U), 0x02);
    ASSERT_EQ(ace2k_led_mask(&led, 1500U), 0x04);
    ASSERT_EQ(ace2k_led_mask(&led, 1750U), 0x08);
    ASSERT_EQ(ace2k_led_mask(&led, 2000U), 0x01);
}

TEST(intro_sweeps_there_and_back_twice_then_goes_dark)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 1000U);
    ASSERT_EQ(ace2k_led_mask(&led, 1000U), 0x00); /* dark while the sensors settle */
    ASSERT_EQ(ace2k_led_mask(&led, 1000U + ACE2K_LED_INTRO_HOLD_MS - 1U), 0x00);
    static const uint8_t sweep[] = {
        0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01,
    };
    for (uint32_t i = 0; i < sizeof(sweep); i++) {
        uint32_t t = 1000U + ACE2K_LED_INTRO_HOLD_MS + (i * ACE2K_LED_INTRO_STEP_MS);
        ASSERT_EQ(ace2k_led_mask(&led, t), sweep[i]);
        ASSERT_EQ(ace2k_led_mask(&led, t + ACE2K_LED_INTRO_STEP_MS - 1U), sweep[i]);
    }
    uint32_t end = 1000U + ACE2K_LED_INTRO_HOLD_MS + (sizeof(sweep) * ACE2K_LED_INTRO_STEP_MS);
    ASSERT_EQ(ace2k_led_mask(&led, end), 0x00); /* every lane empty: dark until the sign */
    ASSERT_EQ(ace2k_led_mask(&led, end + ACE2K_LED_IDLE_PERIOD_MS - 1U), 0x00);
}

TEST(a_loaded_lane_ends_the_intro_for_good)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 0U);
    ASSERT_EQ(ace2k_led_mask(&led, ACE2K_LED_INTRO_HOLD_MS), 0x01);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_LOADED, ACE2K_LED_INTRO_HOLD_MS + 300U);
    ASSERT_EQ(f.mask, 0x04); /* written at once: the lane, not the sweep */
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_OFF, 500U);
    ASSERT_EQ(ace2k_led_mask(&led, 900U), 0x00);         /* the sweep does not come back */
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 5000U); /* a reconnect: the intro is set */
    ASSERT_EQ(ace2k_led_mask(&led, 5000U + ACE2K_LED_INTRO_HOLD_MS), 0x00); /* no new sweep */
}

TEST(no_intro_over_a_lane_already_loaded)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 0U);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 100U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 2000U); /* proven only now */
    ASSERT_EQ(f.mask, 0x02);
    ASSERT_EQ(ace2k_led_mask(&led, 2000U + ACE2K_LED_INTRO_HOLD_MS), 0x02);
}

TEST(a_lane_loaded_during_the_hold_means_no_sweep_at_all)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 0U); /* at init: no lane state yet */
    ace2k_led_set_lane(&led, 3, ACE2K_LED_LANE_LOADED, 30U);
    for (uint32_t t = 30U; t < 3000U; t += 10U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x08);
    }
}

TEST(tick_writes_only_on_change)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 0U);
    ASSERT_EQ(f.writes, 2);
    for (uint32_t t = 0; t < 250U; t += 10U) {
        ace2k_led_tick(&led, t);
    }
    ASSERT_EQ(f.writes, 2);
    ace2k_led_tick(&led, 250U);
    ASSERT_EQ(f.writes, 3);
    ASSERT_EQ(f.mask, 0x02);
    ace2k_led_set_pattern(&led, ACE2K_LED_OFF, 260U);
    ASSERT_EQ(f.writes, 4);
    ASSERT_EQ(f.mask, 0);
}

TEST(set_pattern_restarts_the_clock)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 700U);
    ASSERT_EQ(ace2k_led_mask(&led, 700U), 0x01);
    ASSERT_EQ(ace2k_led_mask(&led, 950U), 0x02);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 950U); /* again: the chase restarts */
    ASSERT_EQ(ace2k_led_mask(&led, 950U), 0x01);
}

TEST(loaded_is_steady_and_the_chase_wins_over_every_lane)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 1000U);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_LOADED, 1000U);
    for (uint32_t t = 1000U; t < 9000U; t += 50U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x04);
    }
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 9000U);
    ASSERT_EQ(ace2k_led_mask(&led, 9000U), 0x01);
    ASSERT_EQ(ace2k_led_mask(&led, 9250U), 0x02);
    ace2k_led_set_pattern(&led, ACE2K_LED_OFF, 9500U);
    ASSERT_EQ(ace2k_led_mask(&led, 9500U), 0x04);
    ace2k_led_set_lane(&led, 4, ACE2K_LED_LANE_ERROR, 9500U); /* out of range: ignored */
    ASSERT_EQ(ace2k_led_mask(&led, 9500U), 0x04);
}

TEST(lanes_in_the_same_state_blink_together)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_MOVING, 0U);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_MOVING, 300U);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_MOVING, 777U);
    for (uint32_t t = 800U; t < 5000U; t += 10U) {
        uint8_t m = ace2k_led_mask(&led, t);
        ASSERT_TRUE(m == 0x00 || m == 0x07);
    }
}

TEST(moving_and_error_blink_at_their_rates)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_MOVING, 0U);
    ace2k_led_set_lane(&led, 3, ACE2K_LED_LANE_ERROR, 0U);
    ASSERT_EQ(ace2k_led_mask(&led, 0U), 0x09);
    ASSERT_EQ(ace2k_led_mask(&led, 100U), 0x01); /* error off at 100 ms (5 Hz) */
    ASSERT_EQ(ace2k_led_mask(&led, 200U), 0x09);
    ASSERT_EQ(ace2k_led_mask(&led, 600U), 0x08); /* moving off from 500 ms (1 Hz) */
    ASSERT_EQ(ace2k_led_mask(&led, 1000U), 0x09);
    ace2k_led_tick(&led, 100U);
    ASSERT_EQ(f.mask, 0x01);
}

TEST(the_same_lane_pattern_again_writes_nothing)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 0U);
    int writes = f.writes;
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 400U);
    ASSERT_EQ(f.writes, writes);
}

/* The powered-on sign: every lane off, all four flash together for IDLE_ON_MS every IDLE_PERIOD_MS,
 * the first one a whole period after the lanes went dark. */
static void assert_sign_cycle(const struct ace2k_led *led, uint32_t flash_at)
{
    ASSERT_EQ(ace2k_led_mask(led, flash_at - 1U), 0x00);
    ASSERT_EQ(ace2k_led_mask(led, flash_at), 0x0F);
    ASSERT_EQ(ace2k_led_mask(led, flash_at + ACE2K_LED_IDLE_ON_MS - 1U), 0x0F);
    ASSERT_EQ(ace2k_led_mask(led, flash_at + ACE2K_LED_IDLE_ON_MS), 0x00);
}

TEST(every_lane_off_flashes_all_four_once_a_period)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 1000U);
    for (uint32_t t = 1000U; t < 1000U + ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x00); /* nothing in the first period */
    }
    for (uint32_t n = 1U; n <= 6U; n++) {
        assert_sign_cycle(&led, 1000U + (n * ACE2K_LED_IDLE_PERIOD_MS));
    }
    ace2k_led_tick(&led, 1000U + ACE2K_LED_IDLE_PERIOD_MS);
    ASSERT_EQ(f.mask, 0x0F);
    ace2k_led_tick(&led, 1000U + ACE2K_LED_IDLE_PERIOD_MS + ACE2K_LED_IDLE_ON_MS);
    ASSERT_EQ(f.mask, 0x00);
}

TEST(the_sign_follows_the_intro_by_a_period)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 500U);
    uint32_t end = 500U + ACE2K_LED_INTRO_MS;
    for (uint32_t t = end; t < end + ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x00);
    }
    assert_sign_cycle(&led, end + ACE2K_LED_IDLE_PERIOD_MS);
    assert_sign_cycle(&led, end + (2U * ACE2K_LED_IDLE_PERIOD_MS));
}

TEST(an_intro_over_a_loaded_lane_leaves_the_sign_to_its_emptying)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_LOADED, 10U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 100U);     /* no sweep: lane 1 is lit */
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_OFF, 1000U); /* emptied: the sweep resumes */
    uint32_t end = 100U + ACE2K_LED_INTRO_MS;
    ASSERT_EQ(ace2k_led_mask(&led, end - 1U), 0x01); /* the sweep's last step */
    for (uint32_t t = end; t < end + ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x00);
    }
    assert_sign_cycle(&led, end + ACE2K_LED_IDLE_PERIOD_MS);
}

TEST(an_emptying_after_the_intro_counts_from_itself)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_LOADED, 10U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 100U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_OFF, 5000U); /* after the intro's end */
    ASSERT_EQ(ace2k_led_mask(&led, 5000U + ACE2K_LED_IDLE_PERIOD_MS - 1U), 0x00);
    assert_sign_cycle(&led, 5000U + ACE2K_LED_IDLE_PERIOD_MS);
}

TEST(a_lane_lit_and_emptied_during_the_intro_counts_from_its_emptying)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 0U);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_MOVING, 500U); /* ends the intro for good */
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_OFF, 900U);
    ASSERT_EQ(ace2k_led_mask(&led, 900U + ACE2K_LED_IDLE_PERIOD_MS - 1U), 0x00);
    assert_sign_cycle(&led, 900U + ACE2K_LED_IDLE_PERIOD_MS);
}

/* A proven image answers every ace2k_version (each Klipper connect) by setting INTRO again:
 * the sweep plays once, and the sign is never pushed back. */
TEST(version_queries_on_a_proven_image_keep_the_sign)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 0U);
    int flashes = 0;
    uint8_t last = 0;
    for (uint32_t t = 0U; t < 120000U; t += 10U) {
        if (t % 5000U == 0U) {
            ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, t); /* a version query */
        }
        ace2k_led_tick(&led, t);
        if (t >= ACE2K_LED_INTRO_MS) {
            ASSERT_TRUE(f.mask == 0x00 || f.mask == 0x0F); /* no sweep again */
        }
        if (f.mask == 0x0F && last != 0x0F) {
            ASSERT_EQ((t - ACE2K_LED_INTRO_MS) % ACE2K_LED_IDLE_PERIOD_MS, 0U);
            flashes++;
        }
        last = f.mask;
    }
    ASSERT_EQ(flashes, (int)((120000U - ACE2K_LED_INTRO_MS) / ACE2K_LED_IDLE_PERIOD_MS));
}

/* The ms counter wraps every 2^32 ms (49.7 days): run the unit from just before one wrap to past
 * the next, ticking as the binding does (coarse in between), and hold the sign to its schedule on
 * a 64-bit clock. */
static uint8_t expected_sign(uint64_t since_intro_end)
{
    if (since_intro_end < ACE2K_LED_IDLE_PERIOD_MS) {
        return 0x00;
    }
    return (since_intro_end % ACE2K_LED_IDLE_PERIOD_MS) < ACE2K_LED_IDLE_ON_MS ? 0x0F : 0x00;
}

TEST(the_intro_and_the_sign_survive_the_clock_wrap)
{
    const uint64_t start = 0xFFFFFFFFULL - 4999ULL; /* the counter wraps 5 s in */
    const uint64_t wrap = 0x100000000ULL;
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, (uint32_t)start);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, (uint32_t)start);
    uint64_t intro_end = start + ACE2K_LED_INTRO_MS;
    int mismatches = 0;
    for (uint64_t t = start; t < start + wrap + 120000ULL;) {
        ace2k_led_tick(&led, (uint32_t)t);
        if (t >= intro_end && f.mask != expected_sign(t - intro_end)) {
            mismatches++;
        }
        if (t < start + 120000ULL || t >= start + wrap - 30000ULL) {
            t += 10ULL;
        } else {
            t += 5000ULL;
        }
    }
    ASSERT_EQ(mismatches, 0); /* no sweep replayed, no flash late or missing */
}

/* The boot path as the binding runs it: the instance in zero-initialised storage, init, the
 * link proof's first INTRO, then the 10 ms tick — the sweep plays once, the sign a period later,
 * and the host's later INTRO at its connect replays nothing. */
static struct ace2k_led ace2k_boot_led; /* static: zero-initialised, as in the firmware */

TEST(the_boot_path_plays_the_sweep_from_a_zeroed_module)
{
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&ace2k_boot_led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&ace2k_boot_led, ACE2K_LED_INTRO, 0U);
    static const uint8_t sweep[] = {
        0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01,
    };
    for (uint32_t t = 10U; t < ACE2K_LED_INTRO_MS + (2U * ACE2K_LED_IDLE_PERIOD_MS); t += 10U) {
        if (t == 5000U) {
            ace2k_led_set_pattern(&ace2k_boot_led, ACE2K_LED_INTRO, t); /* the host connects */
        }
        ace2k_led_tick(&ace2k_boot_led, t);
        uint8_t want = 0x00;
        if (t >= ACE2K_LED_INTRO_HOLD_MS && t < ACE2K_LED_INTRO_MS) {
            want = sweep[(t - ACE2K_LED_INTRO_HOLD_MS) / ACE2K_LED_INTRO_STEP_MS];
        } else if (t >= ACE2K_LED_INTRO_MS + ACE2K_LED_IDLE_PERIOD_MS &&
                   ((t - ACE2K_LED_INTRO_MS) % ACE2K_LED_IDLE_PERIOD_MS) < ACE2K_LED_IDLE_ON_MS) {
            want = 0x0F;
        }
        ASSERT_EQ(f.mask, want);
    }
}

TEST(no_sign_under_the_chase)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 0U);
    for (uint32_t t = 0U; t < 5U * ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
        uint8_t m = ace2k_led_mask(&led, t);
        ASSERT_TRUE(m == 0x01 || m == 0x02 || m == 0x04 || m == 0x08); /* one lane at a time */
    }
}

TEST(no_sign_while_a_lane_shows_anything)
{
    static const enum ace2k_led_lane_pattern shown[] = {
        ACE2K_LED_LANE_LOADED,
        ACE2K_LED_LANE_MOVING,
        ACE2K_LED_LANE_ERROR,
    };
    for (uint32_t i = 0; i < sizeof(shown) / sizeof(shown[0]); i++) {
        struct ace2k_led led;
        struct ace2k_fake_leds f = { 0, 0 };
        ace2k_led_init(&led, &fake_ops, &f, 0U);
        ace2k_led_set_lane(&led, 2, shown[i], 0U);
        for (uint32_t t = 0U; t < 5U * ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
            uint8_t m = ace2k_led_mask(&led, t);
            ASSERT_TRUE(m == 0x00 || m == 0x04); /* only the lane's own pattern */
        }
    }
}

TEST(a_lane_set_mid_flash_ends_the_flash_at_once)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    uint32_t flash = ACE2K_LED_IDLE_PERIOD_MS;
    ace2k_led_tick(&led, flash);
    ASSERT_EQ(f.mask, 0x0F);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, flash + 30U);
    ASSERT_EQ(f.mask, 0x02); /* written in the same call */
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_OFF, flash + 30U);
    ace2k_led_set_lane(&led, 3, ACE2K_LED_LANE_ERROR, flash + 30U);
    ASSERT_EQ(f.mask, 0x08); /* the error blink, alone */
}

TEST(the_last_lane_off_restarts_the_period)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_LOADED, 100U);
    ace2k_led_set_lane(&led, 3, ACE2K_LED_LANE_ERROR, 200U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_OFF, 7000U);
    for (uint32_t t = 7000U; t < 23456U; t += 10U) {
        /* lane 4 still in error: its own 5 Hz blink, and no sign */
        uint8_t error_blink =
            (t % ACE2K_LED_ERROR_PERIOD_MS) < (ACE2K_LED_ERROR_PERIOD_MS / 2U) ? 0x08 : 0x00;
        ASSERT_EQ(ace2k_led_mask(&led, t), error_blink);
    }
    ace2k_led_set_lane(&led, 3, ACE2K_LED_LANE_OFF, 23456U); /* the error cleared */
    ASSERT_EQ(f.mask, 0x00);
    for (uint32_t t = 23456U; t < 23456U + ACE2K_LED_IDLE_PERIOD_MS; t += 10U) {
        ASSERT_EQ(ace2k_led_mask(&led, t), 0x00); /* never at once */
    }
    assert_sign_cycle(&led, 23456U + ACE2K_LED_IDLE_PERIOD_MS);
}

TEST(identify_blinks_all_four_at_2hz_at_once)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_identify(&led, 1000U);
    ASSERT_EQ(f.mask, ACE2K_LED_ALL_MASK); /* written at once */
    ASSERT_EQ(ace2k_led_mask(&led, 1249U), 0x0F);
    ASSERT_EQ(ace2k_led_mask(&led, 1250U), 0x00);
    ASSERT_EQ(ace2k_led_mask(&led, 1499U), 0x00);
    ASSERT_EQ(ace2k_led_mask(&led, 1500U), 0x0F);
    ASSERT_EQ(ace2k_led_mask(&led, 1000U + ACE2K_LED_IDENTIFY_MS - 1U), 0x00);
}

TEST(identify_wins_over_the_chase_and_the_lanes_then_gives_them_back)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 0U);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_LOADED, 0U);
    ace2k_led_identify(&led, 2000U);
    ASSERT_EQ(ace2k_led_mask(&led, 2000U), 0x0F);
    ASSERT_EQ(ace2k_led_mask(&led, 2300U), 0x00);
    uint32_t end = 2000U + ACE2K_LED_IDENTIFY_MS;
    /* the chase again, on its own clock from 0: one lane per 250 ms */
    ASSERT_EQ(ace2k_led_mask(&led, end),
              (uint8_t)(ACE2K_LED_LANE1_MASK << ((end / ACE2K_LED_CHASE_STEP_MS) % 4U)));
}

TEST(identify_again_restarts_the_ten_seconds)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 0, ACE2K_LED_LANE_LOADED, 0U);
    ace2k_led_identify(&led, 1000U);
    ace2k_led_identify(&led, 9000U);
    ASSERT_EQ(ace2k_led_mask(&led, 11000U), 0x0F); /* past the first 10 s, within the second */
    ASSERT_EQ(ace2k_led_mask(&led, 9000U + ACE2K_LED_IDENTIFY_MS), 0x01); /* lane 1 loaded */
}

TEST(the_tick_ends_identify_and_writes_the_normal_mask)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 0U);
    ace2k_led_identify(&led, 100U);
    ace2k_led_tick(&led, 100U + ACE2K_LED_IDENTIFY_MS);
    ASSERT_EQ(f.mask, 0x02);
    ASSERT_TRUE(!led.identifying);
}

TEST(identify_survives_the_clock_wrap)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    uint32_t start = 0xFFFFFF00U;
    ace2k_led_init(&led, &fake_ops, &f, start);
    ace2k_led_identify(&led, start);
    ASSERT_EQ(ace2k_led_mask(&led, start + 600U), 0x0F); /* wrapped: 600 ms in, on again */
    /* 10 s in, identify is over: the mask is what it would be without it.  Every lane is off and
     * 10 s have passed since init, so that is the start of the powered-on sign. */
    ASSERT_EQ(ace2k_led_mask(&led, start + ACE2K_LED_IDENTIFY_MS), ACE2K_LED_ALL_MASK);
}

TEST(identify_over_the_chase_with_no_lane_lit_gives_the_chase_back)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 0U);
    ace2k_led_identify(&led, 3000U);
    ASSERT_EQ(f.mask, ACE2K_LED_ALL_MASK); /* all four, not the chase's one lane */
    ASSERT_EQ(ace2k_led_mask(&led, 3250U), 0x00);
    ASSERT_EQ(ace2k_led_mask(&led, 3500U), ACE2K_LED_ALL_MASK);
    uint32_t end = 3000U + ACE2K_LED_IDENTIFY_MS;
    ace2k_led_tick(&led, end);
    ASSERT_TRUE(!led.identifying);
    ASSERT_EQ(f.mask, (uint8_t)(ACE2K_LED_LANE1_MASK << ((end / ACE2K_LED_CHASE_STEP_MS) % 4U)));
}

TEST(the_tick_ends_identify_across_the_clock_wrap)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    uint32_t start = 0xFFFFFF00U;
    ace2k_led_init(&led, &fake_ops, &f, start);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, start);
    ace2k_led_identify(&led, start);
    for (uint32_t dt = 0U; dt < ACE2K_LED_IDENTIFY_MS; dt += 10U) {
        ace2k_led_tick(&led, start + dt); /* wraps after 256 ms */
        ASSERT_TRUE(led.identifying);
        ASSERT_EQ(f.mask, (dt % ACE2K_LED_IDENTIFY_PERIOD_MS) < (ACE2K_LED_IDENTIFY_PERIOD_MS / 2U)
                              ? ACE2K_LED_ALL_MASK
                              : 0x00);
    }
    ace2k_led_tick(&led, start + ACE2K_LED_IDENTIFY_MS);
    ASSERT_TRUE(!led.identifying);
    ASSERT_EQ(f.mask, 0x02); /* lane 2 loaded, as before identify */
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
