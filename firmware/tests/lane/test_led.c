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

static const struct ace2k_led_ops ace2k_fake_ops = { .set_mask = fake_set_mask };

TEST(init_writes_everything_off_once)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0xFF };
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
    ASSERT_EQ(f.writes, 1);
    ASSERT_EQ(f.mask, 0);
    ace2k_led_tick(&led, 500U);
    ASSERT_EQ(f.writes, 1);
}

TEST(waiting_host_chases_one_lane_at_a_time)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 1000U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ASSERT_EQ(ace2k_led_mask(&led, end), 0x00); /* every lane empty: dark */
    ASSERT_EQ(ace2k_led_mask(&led, end + 60000U), 0x00);
}

TEST(a_loaded_lane_ends_the_intro_for_good)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 0U);
    ASSERT_EQ(ace2k_led_mask(&led, ACE2K_LED_INTRO_HOLD_MS), 0x01);
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_LOADED, ACE2K_LED_INTRO_HOLD_MS + 300U);
    ASSERT_EQ(f.mask, 0x04); /* written at once: the lane, not the sweep */
    ace2k_led_set_lane(&led, 2, ACE2K_LED_LANE_OFF, 500U);
    ASSERT_EQ(ace2k_led_mask(&led, 900U), 0x00); /* the sweep does not come back */
    ace2k_led_set_pattern(&led, ACE2K_LED_INTRO, 5000U);
    ASSERT_EQ(ace2k_led_mask(&led, 5000U + ACE2K_LED_INTRO_HOLD_MS), 0x01); /* a new intro */
}

TEST(no_intro_over_a_lane_already_loaded)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 700U);
    ASSERT_EQ(ace2k_led_mask(&led, 700U), 0x01);
    ASSERT_EQ(ace2k_led_mask(&led, 950U), 0x02);
    ace2k_led_set_pattern(&led, ACE2K_LED_WAITING_HOST, 950U);
    ASSERT_EQ(ace2k_led_mask(&led, 950U), 0x01);
}

TEST(loaded_is_steady_and_the_chase_wins_over_every_lane)
{
    struct ace2k_led led;
    struct ace2k_fake_leds f = { 0, 0 };
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 1000U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
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
    ace2k_led_init(&led, &ace2k_fake_ops, &f, 0U);
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 0U);
    int writes = f.writes;
    ace2k_led_set_lane(&led, 1, ACE2K_LED_LANE_LOADED, 400U);
    ASSERT_EQ(f.writes, writes);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
