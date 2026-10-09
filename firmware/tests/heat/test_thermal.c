/* The thermal simulator reproduces the heater runs measured on the unit (the heater-2026-09-13 data set): each run replayed from its first sample with its own duty column, the
 * model's outlet NTCs and chamber compared at every sample.  The tolerances are the fit's
 * achieved error (tools/thermal_fit.py prints it) with a margin; a re-fit that makes them fail
 * has made the model worse. */
#include "fake_thermal.h"
#include "test.h"
#include "thermal_runs.h"

#define TOL_NTC_RMS_MC     800  /* the fit reaches ≤ 0.58 degC */
#define TOL_CHAMBER_RMS_MC 500  /* ≤ 0.33 degC */
#define TOL_PEAK_MC        1500 /* ≤ 1.04 degC */

/* sqrt by Newton's method: the host build links no libm. */
static double root(double v)
{
    double r = v > 1.0 ? v : 1.0;
    for (int i = 0; i < 64; i++) {
        r = 0.5 * (r + (v / r));
    }
    return r;
}

// NOLINTNEXTLINE(readability-identifier-naming)
struct replay_error {
    double rms_mc[3]; /* left, right, chamber */
    int32_t peak_mc;  /* the worst NTC error at any sample */
};

static struct replay_error replay(const struct thermal_run *run)
{
    struct fake_thermal m;
    struct replay_error e = { { 0.0, 0.0, 0.0 }, 0 };
    double sum[3] = { 0.0, 0.0, 0.0 };
    fake_thermal_init(&m, (int32_t)(THERMAL_BENCH_AMBIENT_C * 1000.0));
    fake_thermal_set(&m, run->s[0].left_mc, run->s[0].right_mc, run->s[0].chamber_mc);
    for (unsigned i = 1; i < run->n; i++) {
        const struct thermal_sample *prev = &run->s[i - 1];
        const struct thermal_sample *cur = &run->s[i];
        fake_thermal_step(&m, prev->duty_pct, FAKE_THERMAL_FANS_BOTH, cur->t_ms - prev->t_ms);
        int32_t d[3] = {
            fake_thermal_ntc_mc(&m, 0) - cur->left_mc,
            fake_thermal_ntc_mc(&m, 1) - cur->right_mc,
            fake_thermal_chamber_mc(&m) - cur->chamber_mc,
        };
        for (unsigned j = 0; j < 3U; j++) {
            sum[j] += (double)d[j] * (double)d[j];
        }
        for (unsigned j = 0; j < 2U; j++) {
            int32_t a = d[j] < 0 ? -d[j] : d[j];
            if (a > e.peak_mc) {
                e.peak_mc = a;
            }
        }
    }
    for (unsigned j = 0; j < 3U; j++) {
        e.rms_mc[j] = root(sum[j] / (double)run->n);
    }
    return e;
}

TEST(each_measured_run_is_reproduced_within_the_fits_tolerance)
{
    for (unsigned r = 0; r < THERMAL_RUN_COUNT; r++) {
        struct replay_error e = replay(&thermal_runs[r]);
        printf("  %s: rms left %.0f right %.0f chamber %.0f mdegC, peak %d mdegC\n",
               thermal_runs[r].name, e.rms_mc[0], e.rms_mc[1], e.rms_mc[2], (int)e.peak_mc);
        ASSERT_TRUE(e.rms_mc[0] <= TOL_NTC_RMS_MC);
        ASSERT_TRUE(e.rms_mc[1] <= TOL_NTC_RMS_MC);
        ASSERT_TRUE(e.rms_mc[2] <= TOL_CHAMBER_RMS_MC);
        ASSERT_TRUE(e.peak_mc <= TOL_PEAK_MC);
    }
}

TEST(a_unit_at_rest_stays_at_its_ambient)
{
    struct fake_thermal m;
    fake_thermal_init(&m, 25000);
    fake_thermal_step(&m, 0, FAKE_THERMAL_FANS_BOTH, 600000U);
    ASSERT_EQ(fake_thermal_ntc_mc(&m, 0), 25000);
    ASSERT_EQ(fake_thermal_ntc_mc(&m, 1), 25000);
    ASSERT_EQ(fake_thermal_chamber_mc(&m), 25000);
}

TEST(the_ntc_lags_the_heater_and_overshoots_after_the_gate_stops)
{
    struct fake_thermal m;
    fake_thermal_init(&m, 28000);
    fake_thermal_step(&m, 50, FAKE_THERMAL_FANS_BOTH, 40000U);
    ASSERT_TRUE(fake_thermal_heater_mc(&m, 1) > fake_thermal_ntc_mc(&m, 1));
    int32_t at_stop = fake_thermal_ntc_mc(&m, 1);
    int32_t peak = at_stop;
    for (unsigned t = 0; t < 30000U; t += FAKE_THERMAL_STEP_MS) {
        fake_thermal_step(&m, 0, FAKE_THERMAL_FANS_BOTH, FAKE_THERMAL_STEP_MS);
        if (fake_thermal_ntc_mc(&m, 1) > peak) {
            peak = fake_thermal_ntc_mc(&m, 1);
        }
    }
    printf("  50 %% for 40 s: right NTC %d at the stop, peak %d mdegC\n", (int)at_stop, (int)peak);
    ASSERT_TRUE(peak - at_stop >= 1000); /* the bench: +3.8 degC at 50 % */
}

TEST(a_stopped_fan_in_the_synthetic_variant_heats_its_side_faster)
{
    struct fake_thermal both;
    struct fake_thermal one;
    fake_thermal_init(&both, 25000);
    fake_thermal_init(&one, 25000);
    fake_thermal_step(&both, 30, FAKE_THERMAL_FANS_BOTH, 60000U);
    fake_thermal_step(&one, 30, 2U /* left fan off */, 60000U);
    ASSERT_TRUE(fake_thermal_ntc_mc(&one, 0) > fake_thermal_ntc_mc(&both, 0));
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
