#include "test.h"
#include "lane/sensors.h"

static struct ace2k_insert_band ace2k_defaults[ACE2K_LANE_COUNT];

/* An enum constant is a signed int in C, and the lint refuses one as a shift count. */
static uint32_t bit_of(unsigned n)
{
    return 1U << n;
}

static void make_defaults(void)
{
    for (int i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_insert_band_defaults(&ace2k_defaults[i]);
    }
}

/* Every switch pin high (nothing asserted; the cutout idle), every ADC channel at 1200 mV: an
 * insert with no filament reads at the high end of its band. */
static void inputs_idle(struct ace2k_sensors_inputs *in)
{
    for (int i = 0; i < ACE2K_ADC_COUNT; i++) {
        in->mv[i] = 1200;
    }
    in->adc_age_ms = 0;
    in->adc_valid = true;
    in->levels = (uint16_t)(bit_of(ACE2K_SENSORS_IN_COUNT) - 1U);
    in->levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_CUTOUT); /* idle low */
}

static int drain(struct ace2k_sensors *s, struct ace2k_sensors_event *last)
{
    int n = 0;
    struct ace2k_sensors_event ev;
    while (ace2k_sensors_pop_event(s, &ev)) {
        n++;
        if (last) {
            *last = ev;
        }
    }
    return n;
}

static void ticks(struct ace2k_sensors *s, struct ace2k_sensors_inputs *in, int n, uint32_t *now)
{
    for (int i = 0; i < n; i++) {
        *now += 10;
        ace2k_sensors_tick(s, *now, in);
    }
}

TEST(first_tick_primes_silently_even_with_switches_asserted)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_REST1); /* lane 1 at rest */
    in.mv[ACE2K_ADC_INSERT1] = 400;                         /* present: below 750 */
    uint32_t now = 0;
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(s.lane[0].rest);
    ASSERT_TRUE(s.lane[0].insert); /* primed straight from the hysteresis of the seeded filter */
}

TEST(insert_asserts_after_the_filter_and_the_debounce_with_one_event)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    struct ace2k_sensors_event ev;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ticks(&s, &in, 5, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    in.mv[ACE2K_ADC_INSERT1] = 500; /* a strand: the reading falls */
    ticks(&s, &in, 4, &now);
    ASSERT_EQ(drain(&s, 0), 0); /* α = 0.2: 500 + 700 × 0.8^4 = 786 > 750 */
    /* tick 5: 729 ≤ 750, raw present; tick 6: the debounce's second sample, the event */
    ticks(&s, &in, 4, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.lane, 0);
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_INSERT);
    ASSERT_EQ(ev.level, 1);
    ASSERT_TRUE(ace2k_sensors_switches(&s) & 1U);
    ticks(&s, &in, 20, &now);
    ASSERT_EQ(drain(&s, 0), 0);
}

TEST(insert_hysteresis_at_thirty_and_seventy_percent_of_the_band)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    struct ace2k_sensors_event ev;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    in.mv[ACE2K_ADC_INSERT2] = 800; /* above 750: never asserts */
    ticks(&s, &in, 50, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(!s.lane[1].insert);
    in.mv[ACE2K_ADC_INSERT2] = 500; /* tick 1: 740 ≤ 750, raw present; the event on tick 2 */
    ticks(&s, &in, 30, &now);
    ASSERT_EQ(drain(&s, 0), 1);
    in.mv[ACE2K_ADC_INSERT2] = 900; /* below 950: stays (the filter settles at 899) */
    ticks(&s, &in, 50, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(s.lane[1].insert);
    in.mv[ACE2K_ADC_INSERT2] = 1000; /* tick 4: 958 ≥ 950, raw absent; the event on tick 5 */
    ticks(&s, &in, 30, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.lane, 1);
    ASSERT_EQ(ev.level, 0);
}

TEST(switch_needs_two_identical_samples_and_ignores_a_glitch)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    struct ace2k_sensors_event ev;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ticks(&s, &in, 3, &now);
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_PUSHED1 + 2); /* lane 3 pushed */
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(!s.lane[2].pushed);
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.lane, 2);
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_PUSHED);
    ASSERT_EQ(ev.level, 1);
    ASSERT_TRUE(((uint32_t)ace2k_sensors_switches(&s) >> (ACE2K_SENSORS_STATE_PUSHED_SHIFT + 2)) &
                1U);
    /* a one-tick release is a glitch */
    in.levels |= (uint16_t)bit_of(ACE2K_SENSORS_IN_PUSHED1 + 2);
    ticks(&s, &in, 1, &now);
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_PUSHED1 + 2);
    ticks(&s, &in, 3, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(s.lane[2].pushed);
}

TEST(pulled_and_cutout_are_shared_events_with_no_lane)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    struct ace2k_sensors_event ev;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ticks(&s, &in, 2, &now);
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_PULLED);
    ticks(&s, &in, 2, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.lane, ACE2K_SENSORS_LANE_NONE);
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_PULLED);
    ASSERT_EQ(ev.level, 1);
    ASSERT_TRUE(((uint32_t)ace2k_sensors_switches(&s) >> ACE2K_SENSORS_STATE_PULLED_BIT) & 1U);
    in.levels |= (uint16_t)bit_of(ACE2K_SENSORS_IN_CUTOUT); /* asserted high */
    ticks(&s, &in, 2, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_CUTOUT);
    ASSERT_TRUE(((uint32_t)ace2k_sensors_switches(&s) >> ACE2K_SENSORS_STATE_CUTOUT_BIT) & 1U);
}

TEST(band_set_and_get_with_refusals)
{
    struct ace2k_sensors s;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    ASSERT_EQ(ace2k_sensors_set_band(&s, 4, 600, 1100, ACE2K_CAL_CONFIG), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_sensors_set_band(&s, 1, 1100, 1100, ACE2K_CAL_CONFIG), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_sensors_set_band(&s, 1, 700, 1300, ACE2K_CAL_CONFIG), 0);
    ASSERT_EQ(ace2k_sensors_band(&s, 1)->low_mv, 700);
    ASSERT_EQ(ace2k_sensors_band(&s, 1)->source, ACE2K_CAL_CONFIG);
    ASSERT_EQ(ace2k_sensors_band(&s, 0)->source, ACE2K_CAL_DEFAULT);
    ASSERT_TRUE(ace2k_sensors_band(&s, 4) == NULL);
}

TEST(fresh_connected_and_consistent)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ASSERT_TRUE(!ace2k_sensors_fresh(&s));
    in.adc_age_ms = 50;
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(ace2k_sensors_fresh(&s));
    in.adc_age_ms = 51;
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(!ace2k_sensors_fresh(&s));
    in.mv[ACE2K_ADC_INSERT1] = 10;
    in.mv[ACE2K_ADC_INSERT2] = 3260;
    in.mv[ACE2K_ADC_INSERT3] = 800;
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 0));
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 1));
    ASSERT_TRUE(ace2k_sensors_insert_connected(&s, 2));
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 4)); /* no such lane */
    /* the rails themselves: at either one is disconnected, one millivolt inside is connected */
    in.mv[ACE2K_ADC_INSERT1] = ACE2K_SENSORS_RAIL_LOW_MV;
    in.mv[ACE2K_ADC_INSERT2] = ACE2K_SENSORS_RAIL_HIGH_MV;
    in.mv[ACE2K_ADC_INSERT3] = ACE2K_SENSORS_RAIL_LOW_MV + 1;
    in.mv[ACE2K_ADC_INSERT4] = ACE2K_SENSORS_RAIL_HIGH_MV - 1;
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 0));
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 1));
    ASSERT_TRUE(ace2k_sensors_insert_connected(&s, 2));
    ASSERT_TRUE(ace2k_sensors_insert_connected(&s, 3));
    ASSERT_TRUE(ace2k_sensors_buffer_consistent(&s, 3));
    in.levels &=
        (uint16_t)~(bit_of(ACE2K_SENSORS_IN_REST1 + 3) | bit_of(ACE2K_SENSORS_IN_PUSHED1 + 3));
    ticks(&s, &in, 2, &now);
    ASSERT_TRUE(!ace2k_sensors_buffer_consistent(&s, 3));
}

TEST(a_sample_at_a_rail_is_never_filament)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    in.mv[ACE2K_ADC_INSERT1] = 10;   /* a disconnected lever board: below every band */
    in.mv[ACE2K_ADC_INSERT2] = 3260; /* a shorted one */
    ticks(&s, &in, 50, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(!s.lane[0].insert);
    ASSERT_TRUE(!s.lane[1].insert);
    ASSERT_TRUE(!(ace2k_sensors_switches(&s) & 3U));
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 0));
    ASSERT_TRUE(!ace2k_sensors_insert_connected(&s, 1));
    /* a strand on a lane with filament already, then the board comes off: absent, one event */
    in.mv[ACE2K_ADC_INSERT3] = 500;
    ticks(&s, &in, 30, &now);
    ASSERT_EQ(drain(&s, 0), 1);
    ASSERT_TRUE(s.lane[2].insert);
    in.mv[ACE2K_ADC_INSERT3] = 0;
    ticks(&s, &in, 2, &now);
    ASSERT_EQ(drain(&s, 0), 1);
    ASSERT_TRUE(!s.lane[2].insert);
    /* back between the rails at a "present" reading: present again after the debounce */
    in.mv[ACE2K_ADC_INSERT3] = 500;
    ticks(&s, &in, 30, &now);
    ASSERT_EQ(drain(&s, 0), 1);
    ASSERT_TRUE(s.lane[2].insert);
}

TEST(a_return_from_the_low_rail_restarts_the_filter_from_the_real_value)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    struct ace2k_sensors_event ev;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    /* the lever board off for 500 ms, then back with no filament: a filter left at 0 mV would
     * climb 240, 432, 585, 708 … through the "present" band and report a strand for 70 ms */
    in.mv[ACE2K_ADC_INSERT1] = 0;
    ticks(&s, &in, 50, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    ASSERT_TRUE(!s.lane[0].insert);
    in.mv[ACE2K_ADC_INSERT1] = 1200;
    for (int t = 0; t < 30; t++) {
        ticks(&s, &in, 1, &now);
        ASSERT_EQ(drain(&s, 0), 0);
        ASSERT_TRUE(!s.lane[0].insert);
    }
    /* off again, then back with a strand present: one event, present */
    in.mv[ACE2K_ADC_INSERT1] = 0;
    ticks(&s, &in, 50, &now);
    ASSERT_EQ(drain(&s, 0), 0);
    in.mv[ACE2K_ADC_INSERT1] = 500;
    ticks(&s, &in, 30, &now);
    ASSERT_EQ(drain(&s, &ev), 1);
    ASSERT_EQ(ev.lane, 0);
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_INSERT);
    ASSERT_EQ(ev.level, 1);
    ASSERT_TRUE(s.lane[0].insert);
}

TEST(a_full_ring_drops_and_counts)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ticks(&s, &in, 1, &now);
    /* toggle lane 1's rest switch: every two ticks one event; 40 toggles overflow 15 slots */
    for (int t = 0; t < 40; t++) {
        in.levels ^= (uint16_t)bit_of(ACE2K_SENSORS_IN_REST1);
        ticks(&s, &in, 2, &now);
    }
    ASSERT_EQ(s.dropped, 40 - (ACE2K_SENSORS_EVENT_RING - 1));
    ASSERT_TRUE(ace2k_sensors_has_events(&s));
    ASSERT_EQ(drain(&s, 0), ACE2K_SENSORS_EVENT_RING - 1);
    ASSERT_TRUE(!ace2k_sensors_has_events(&s));
}

TEST(a_peek_leaves_the_oldest_event_in_the_ring_for_the_pop)
{
    /* the binding peeks, tries to send, and pops only once the frame is queued: a frame that
     * did not fit waits at the tail for the next tick */
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    ticks(&s, &in, 1, &now);
    struct ace2k_sensors_event ev;
    ASSERT_TRUE(!ace2k_sensors_peek_event(&s, &ev)); /* empty */
    in.levels ^= (uint16_t)bit_of(ACE2K_SENSORS_IN_REST1);
    ticks(&s, &in, 2, &now);
    in.levels ^= (uint16_t)bit_of(ACE2K_SENSORS_IN_PUSHED1 + 1);
    ticks(&s, &in, 2, &now);
    ASSERT_TRUE(ace2k_sensors_peek_event(&s, &ev));
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_REST);
    ASSERT_EQ(ev.lane, 0);
    ASSERT_TRUE(ace2k_sensors_peek_event(&s, &ev)); /* still there, still the oldest */
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_REST);
    ace2k_sensors_drop_event(&s); /* the binding's step once the peeked frame is queued */
    ASSERT_TRUE(ace2k_sensors_peek_event(&s, &ev)); /* the next one */
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_PUSHED);
    ASSERT_EQ(ev.lane, 1);
    ASSERT_TRUE(ace2k_sensors_pop_event(&s, &ev)); /* a pop is the peek and the drop */
    ASSERT_EQ(ev.kind, ACE2K_SENSORS_KIND_PUSHED);
    ASSERT_TRUE(!ace2k_sensors_peek_event(&s, &ev));
    ASSERT_TRUE(!ace2k_sensors_has_events(&s));
    ace2k_sensors_drop_event(&s); /* nothing to drop: nothing happens */
    ASSERT_EQ(s.tail, s.head);
}

TEST(insert_filter_seeds_on_the_first_pass_when_the_first_tick_had_none)
{
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    in.adc_valid = false;
    uint32_t now = 0;
    ticks(&s, &in, 3, &now);
    in.adc_valid = true;
    in.mv[ACE2K_ADC_INSERT1] = 400; /* present: below 750 */
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(drain(&s, 0), 0); /* seeded present, silently */
    ASSERT_TRUE(s.lane[0].insert);
}

TEST(a_lane_moving_from_rest_to_pushed_in_one_pass_never_reads_as_both)
{
    /* The health self-test reads the pair from task context while the tick can flip both
     * switches in one pass; it reads one byte the tick writes after both debounces, so a lane
     * in transit is consistent and only a lane with both switches held is not. */
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_REST1 + 1); /* lane 2 at rest */
    ticks(&s, &in, 2, &now);
    ASSERT_TRUE(s.lane[1].rest);
    ASSERT_TRUE(!s.lane[1].rest_and_pushed);
    ASSERT_TRUE(ace2k_sensors_buffer_consistent(&s, 1));
    /* rest released and pushed asserted on the same sample: both debounce on the same tick */
    in.levels |= (uint16_t)bit_of(ACE2K_SENSORS_IN_REST1 + 1);
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_PUSHED1 + 1);
    for (int t = 0; t < 4; t++) {
        ticks(&s, &in, 1, &now);
        ASSERT_TRUE(!s.lane[1].rest_and_pushed);
        ASSERT_TRUE(ace2k_sensors_buffer_consistent(&s, 1));
    }
    ASSERT_TRUE(!s.lane[1].rest);
    ASSERT_TRUE(s.lane[1].pushed);
    /* a genuinely broken lane: both held for two ticks */
    in.levels &= (uint16_t)~bit_of(ACE2K_SENSORS_IN_REST1 + 1);
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(!s.lane[1].rest_and_pushed); /* the first sample: not yet debounced */
    ticks(&s, &in, 1, &now);
    ASSERT_TRUE(s.lane[1].rest_and_pushed);
    ASSERT_TRUE(!ace2k_sensors_buffer_consistent(&s, 1));
    ASSERT_TRUE(ace2k_sensors_buffer_consistent(&s, 0));
    ASSERT_TRUE(!ace2k_sensors_buffer_consistent(&s, 4)); /* no such lane */
    /* released: consistent again after the debounce */
    in.levels |= (uint16_t)bit_of(ACE2K_SENSORS_IN_PUSHED1 + 1);
    ticks(&s, &in, 2, &now);
    ASSERT_TRUE(!s.lane[1].rest_and_pushed);
    ASSERT_TRUE(ace2k_sensors_buffer_consistent(&s, 1));
}

TEST(the_age_is_zero_while_no_adc_pass_exists)
{
    /* The binding's age is whatever the board left in it when no pass exists yet: the core
     * does not copy it — 0 until the first pass, then the pass's age, 0 again without one. */
    struct ace2k_sensors s;
    struct ace2k_sensors_inputs in;
    make_defaults();
    ace2k_sensors_init(&s, ace2k_defaults);
    inputs_idle(&in);
    uint32_t now = 0;
    in.adc_valid = false;
    in.adc_age_ms = 0xDEADBEEFU; /* indeterminate on the board */
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(s.adc_age_ms, 0);
    ASSERT_TRUE(!ace2k_sensors_fresh(&s));
    in.adc_valid = true;
    in.adc_age_ms = 20;
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(s.adc_age_ms, 20);
    ASSERT_TRUE(ace2k_sensors_fresh(&s));
    in.adc_valid = false;
    in.adc_age_ms = 0xDEADBEEFU;
    ticks(&s, &in, 1, &now);
    ASSERT_EQ(s.adc_age_ms, 0);
    ASSERT_TRUE(!ace2k_sensors_fresh(&s));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
