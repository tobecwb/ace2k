#include "core/health.h"

static uint32_t bit_if(bool failing, unsigned bit)
{
    return failing ? ACE2K_HEALTH_BIT(bit) : 0;
}

/* The bit when the check does NOT hold.  A separate reader instead of bit_if(!ok, …): in C a
 * `!` result is an int, and the lint refuses one handed to a bool parameter. */
static uint32_t bit_unless(bool ok, unsigned bit)
{
    return ok ? 0 : ACE2K_HEALTH_BIT(bit);
}

/* |delta| > the quiet band, as an early return for the same reason. */
static bool encoder_moved(int32_t delta)
{
    if (delta > ACE2K_HEALTH_ENCODER_QUIET_COUNTS) {
        return true;
    }
    return delta < -ACE2K_HEALTH_ENCODER_QUIET_COUNTS;
}

uint32_t ace2k_health_evaluate(const struct ace2k_health_inputs *in)
{
    uint32_t f = 0;
    f |= bit_unless(in->ntc_left_valid, ACE2K_HEALTH_NTC_LEFT);
    f |= bit_unless(in->ntc_right_valid, ACE2K_HEALTH_NTC_RIGHT);
    f |= bit_unless(in->chamber_valid, ACE2K_HEALTH_CHAMBER);
    f |= bit_unless(in->chamber_plausible, ACE2K_HEALTH_CHAMBER_PLAUSIBLE);
    f |= bit_unless(in->reader_ok[0], ACE2K_HEALTH_READER_A);
    f |= bit_unless(in->reader_ok[1], ACE2K_HEALTH_READER_B);
    f |= bit_unless(in->zerocross_present, ACE2K_HEALTH_ZEROCROSS);
    f |= bit_unless(in->mains_plausible, ACE2K_HEALTH_MAINS_HZ);
    f |= bit_unless(in->cutout_idle, ACE2K_HEALTH_CUTOUT);
    for (unsigned i = 0; i < ACE2K_LANE_COUNT; i++) {
        f |= bit_unless(in->insert_connected[i], ACE2K_HEALTH_INSERT1 + i);
        if (in->buffer_check_enabled) {
            f |= bit_unless(in->buffer_consistent[i], ACE2K_HEALTH_BUFFER1 + i);
        }
        f |= bit_if(encoder_moved(in->encoder_delta[i]), ACE2K_HEALTH_ENCODER1 + i);
        f |= bit_if(in->fg_delta[i] != 0, ACE2K_HEALTH_FG1 + i);
    }
    f |= bit_unless(in->sensors_fresh, ACE2K_HEALTH_FRESH);
    f |= bit_unless(in->vdda_ok, ACE2K_HEALTH_VDDA);
    f |= bit_unless(in->clock_ok, ACE2K_HEALTH_CLOCK);
    f |= bit_unless(in->watchdog_ok, ACE2K_HEALTH_WATCHDOG);
    f |= bit_unless(in->config_page_ok, ACE2K_HEALTH_CONFIG_PAGE);
    f |= bit_unless(in->image_crc_ok, ACE2K_HEALTH_IMAGE_CRC);
    return f;
}

void ace2k_health_init(struct ace2k_health *self, uint8_t reset_cause)
{
    *self = (struct ace2k_health){ 0 };
    self->reset_cause = reset_cause;
}

void ace2k_health_apply(struct ace2k_health *self, uint32_t mask, uint32_t now_ms, bool full)
{
    if (full) {
        self->now = mask;
        self->post_ms = now_ms;
        self->post_done = true;
    } else {
        self->now =
            (self->now & ACE2K_HEALTH_BOOT_ONLY_MASK) | (mask & ~ACE2K_HEALTH_BOOT_ONLY_MASK);
    }
    self->latched |= self->now;
}

void ace2k_health_clear(struct ace2k_health *self)
{
    self->latched = self->now;
}
