#include "test.h"
#include "core/health.h"

static void all_good(struct ace2k_health_inputs *in)
{
    *in = (struct ace2k_health_inputs){ false }; /* the first member is a bool */
    in->ntc_left_valid = in->ntc_right_valid = true;
    in->chamber_valid = in->chamber_plausible = true;
    in->reader_ok[0] = in->reader_ok[1] = true;
    in->zerocross_present = in->mains_plausible = true;
    in->cutout_idle = true;
    for (int i = 0; i < ACE2K_LANE_COUNT; i++) {
        in->insert_connected[i] = true;
        in->buffer_consistent[i] = true;
    }
    in->buffer_check_enabled = true;
    in->sensors_fresh = in->vdda_ok = true;
    in->clock_ok = in->watchdog_ok = in->config_page_ok = in->image_crc_ok = true;
}

TEST(all_good_is_zero_and_each_input_maps_to_its_bit)
{
    struct ace2k_health_inputs in;
    all_good(&in);
    ASSERT_EQ(ace2k_health_evaluate(&in), 0);
    /* every scalar check: break it, expect its bit alone */
    struct {
        bool *flag;
        unsigned bit;
    } scalars[] = {
        { &in.ntc_left_valid, ACE2K_HEALTH_NTC_LEFT },
        { &in.ntc_right_valid, ACE2K_HEALTH_NTC_RIGHT },
        { &in.chamber_valid, ACE2K_HEALTH_CHAMBER },
        { &in.chamber_plausible, ACE2K_HEALTH_CHAMBER_PLAUSIBLE },
        { &in.reader_ok[0], ACE2K_HEALTH_READER_A },
        { &in.reader_ok[1], ACE2K_HEALTH_READER_B },
        { &in.zerocross_present, ACE2K_HEALTH_ZEROCROSS },
        { &in.mains_plausible, ACE2K_HEALTH_MAINS_HZ },
        { &in.cutout_idle, ACE2K_HEALTH_CUTOUT },
        { &in.insert_connected[0], ACE2K_HEALTH_INSERT1 },
        { &in.insert_connected[3], ACE2K_HEALTH_INSERT1 + 3 },
        { &in.buffer_consistent[1], ACE2K_HEALTH_BUFFER1 + 1 },
        { &in.sensors_fresh, ACE2K_HEALTH_FRESH },
        { &in.vdda_ok, ACE2K_HEALTH_VDDA },
        { &in.clock_ok, ACE2K_HEALTH_CLOCK },
        { &in.watchdog_ok, ACE2K_HEALTH_WATCHDOG },
        { &in.config_page_ok, ACE2K_HEALTH_CONFIG_PAGE },
        { &in.image_crc_ok, ACE2K_HEALTH_IMAGE_CRC },
    };
    for (size_t i = 0; i < sizeof scalars / sizeof scalars[0]; i++) {
        *scalars[i].flag = false;
        ASSERT_EQ(ace2k_health_evaluate(&in), ACE2K_HEALTH_BIT(scalars[i].bit));
        *scalars[i].flag = true;
    }
    in.encoder_delta[2] = 3;
    ASSERT_EQ(ace2k_health_evaluate(&in), ACE2K_HEALTH_BIT(ACE2K_HEALTH_ENCODER1 + 2));
    in.encoder_delta[2] = -2; /* within the quiet band */
    ASSERT_EQ(ace2k_health_evaluate(&in), 0);
    in.fg_delta[0] = 1;
    ASSERT_EQ(ace2k_health_evaluate(&in), ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1));
    in.fg_delta[0] = 0;
    in.buffer_consistent[3] = false;
    in.buffer_check_enabled = false;
    ASSERT_EQ(ace2k_health_evaluate(&in), 0); /* not enabled: never a bit */
}

TEST(apply_keeps_boot_only_bits_on_a_periodic_pass_and_latches)
{
    struct ace2k_health h;
    ace2k_health_init(&h, ACE2K_RESET_POWER_ON);
    ASSERT_EQ(h.reset_cause, ACE2K_RESET_POWER_ON);
    ASSERT_TRUE(!h.post_done);
    ace2k_health_apply(
        &h, ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1 + 1) | ACE2K_HEALTH_BIT(ACE2K_HEALTH_CHAMBER), 2000,
        true);
    ASSERT_TRUE(h.post_done);
    ASSERT_EQ(h.post_ms, 2000);
    ASSERT_EQ(h.now,
              ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1 + 1) | ACE2K_HEALTH_BIT(ACE2K_HEALTH_CHAMBER));
    /* periodic: the chamber recovers, the FG bit (boot-only) stays */
    ace2k_health_apply(&h, 0, 3000, false);
    ASSERT_EQ(h.now, ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1 + 1));
    ASSERT_EQ(h.latched,
              ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1 + 1) | ACE2K_HEALTH_BIT(ACE2K_HEALTH_CHAMBER));
    /* a periodic mask carrying a boot-only bit does not set it */
    ace2k_health_apply(&h, ACE2K_HEALTH_BIT(ACE2K_HEALTH_IMAGE_CRC), 4000, false);
    ASSERT_EQ(h.now, ACE2K_HEALTH_BIT(ACE2K_HEALTH_FG1 + 1));
    ace2k_health_clear(&h);
    ASSERT_EQ(h.latched, h.now);
    /* a full run replaces everything */
    ace2k_health_apply(&h, 0, 5000, true);
    ASSERT_EQ(h.now, 0);
    ASSERT_EQ(h.post_ms, 5000);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
