/* The dryer's learned heater rate (dryer_gain.c): what the response rule judges against, the
 * clean cycle that stores it, the fault that drops an unstored candidate, and the forget. The
 * simulator's heater gain stands for a stronger heater (230 V), the blocked ducts for a fault. */
#include <stdio.h>
#include <stdlib.h>

#include "fake_dryer_rig.h"
#include "test.h"

/* Today's response fault on the blocked 65 °C start from 15 °C, into HEATING: measured on the
 * code before the rate was learned. A stored floor must give exactly this. */
#define PINNED_BLOCKED_65_15_MS 55010U
#define CYCLE_MIN               20U
/* The blocked first cycle's stored rate must still fault a blocked cold start within this. */
#define LIMITATION_FAULT_MS 130000U

// NOLINTNEXTLINE(readability-identifier-naming)
static struct rig r; /* kept off the stack */

/* The time into HEATING at which the dryer faulted, or UINT32_MAX. */
static uint32_t fault_after(uint32_t max_ms)
{
    uint32_t since = r.dryer.heating_since_ms;
    for (uint32_t t = 0; t < max_ms; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (r.dryer.state == ACE2K_DRYER_FAULT) {
            return r.now_ms - since;
        }
    }
    return UINT32_MAX;
}

/* Runs a started cycle to its end, or to a fault; true when it faulted RESPONSE. */
static bool run_cycle(uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (r.dryer.state != ACE2K_DRYER_HEATING) {
            break;
        }
    }
    return (bool)(r.dryer.state == ACE2K_DRYER_FAULT &&
                  r.dryer.fault == ACE2K_DRYER_FAULT_RESPONSE);
}

/* A cold unit at ambient_mc that keeps the stored rate it had. */
static void cold_with(int32_t ambient_mc, uint8_t stored, double heater_gain)
{
    rig_init(&r, ambient_mc);
    r.log.gain_mc_s_per_pct = stored;
    r.plant.heater_gain = heater_gain;
}

/* Runs tick by tick until the clean cycle's candidate is written into the log. */
static bool run_until_due(uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (r.log.gain_mc_s_per_pct != 0U || r.dryer.state != ACE2K_DRYER_HEATING) {
            break;
        }
    }
    return r.log.gain_mc_s_per_pct != 0U;
}

static uint8_t source(void)
{
    uint16_t g;
    uint8_t s;
    ace2k_dryer_gain_get(&r.dryer, &g, &s);
    return s;
}

static uint16_t rate(void)
{
    uint16_t g;
    uint8_t s;
    ace2k_dryer_gain_get(&r.dryer, &g, &s);
    return g;
}

/* The open-duct starts that store, of nine: all at 1× and 2×, eight at 3×. The one at 3× that does
 * not has its sides up to 9.4 °C apart after the first reach (measured on the simulator). */
#define STORES_AT_1X 9U
#define STORES_AT_2X 9U
#define STORES_AT_3X 8U

/* Nine starts at heater_gain, each twice: a first 20-min cycle with nothing stored, then a cold
 * one with the rate the first cycle stored. A first cycle that stored nothing hands its learned
 * rate to the second as if stored, so every start is also judged from a stored rate. Returns the
 * first cycles that stored. */
static unsigned nine_starts_never_fault(double heater_gain)
{
    static const int32_t ambients[] = { 15000, 25000, 35000 };
    static const uint8_t targets[] = { 45U, 55U, 65U };
    unsigned stores = 0U;
    for (unsigned a = 0; a < 3U; a++) {
        for (unsigned t = 0; t < 3U; t++) {
            cold_with(ambients[a], 0U, heater_gain);
            ASSERT_EQ(rig_start(&r, targets[t], CYCLE_MIN), ACE2K_DRYER_OK);
            ASSERT_TRUE(!run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS));
            uint8_t stored = r.log.gain_mc_s_per_pct;
            uint8_t learned = r.dryer.gain.rate;
            ASSERT_TRUE(stored == 0U || stored == learned);
            ASSERT_TRUE(learned <= ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
            if (stored != 0U) {
                stores++;
            }
            cold_with(ambients[a], stored != 0U ? stored : learned, heater_gain);
            ASSERT_EQ(rig_start(&r, targets[t], CYCLE_MIN), ACE2K_DRYER_OK);
            ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
            ASSERT_TRUE(!run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS));
            printf("  gain %.1f ambient %d target %u: learned %u, stored %u, then %u\n",
                   heater_gain, ambients[a] / 1000, targets[t], learned, stored, r.dryer.fault);
        }
    }
    printf("  gain %.1f: %u of 9 stored\n", heater_gain, stores);
    return stores;
}

TEST(one_two_and_three_times_the_heater_never_fault_the_response_and_store)
{
    ASSERT_EQ(nine_starts_never_fault(1.0), STORES_AT_1X);
    ASSERT_EQ(nine_starts_never_fault(2.0), STORES_AT_2X);
    ASSERT_EQ(nine_starts_never_fault(3.0), STORES_AT_3X);
}

/* The sides of a cold 65 °C start drift apart while the heater ramps; that gap comes before the
 * first reach and does not count. */
TEST(a_side_gap_in_the_warm_up_does_not_block_the_store)
{
    cold_with(15000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 65U, CYCLE_MIN), ACE2K_DRYER_OK);
    uint32_t warm_gap = 0U;
    while (!r.dryer.gain.reached && r.dryer.state == ACE2K_DRYER_HEATING) {
        rig_tick(&r);
        uint32_t gap = ace2k_abs_i32(r.last_in.ntc_left_mc - r.last_in.ntc_right_mc);
        if (!r.dryer.gain.reached && gap > warm_gap) {
            warm_gap = gap;
        }
    }
    printf("  65 C from 15 C: warm-up gap %u mC\n", warm_gap);
    ASSERT_TRUE(warm_gap > (uint32_t)ACE2K_DRYER_GAIN_SIDES_MC);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_EQ(r.log.gain_mc_s_per_pct, ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT);
}

/* After the first reach, the sides apart by more than the bound for a moment: no store. */
TEST(a_side_gap_after_the_reach_blocks_the_store)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    while (!r.dryer.gain.reached && r.dryer.state == ACE2K_DRYER_HEATING) {
        rig_tick(&r);
    }
    ASSERT_TRUE(r.dryer.gain.reached);
    ASSERT_TRUE(r.dryer.gain.sides_ok);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    for (uint32_t t = 0; t < 2000U; t += RIG_TICK_MS) {
        r.ovr_left_mc = fake_thermal_ntc_mc(&r.plant, 0U);
        r.ovr_right_mc = r.ovr_left_mc - ACE2K_DRYER_GAIN_SIDES_MC - 500;
        rig_tick(&r);
    }
    r.ovr_ntc = false;
    ASSERT_TRUE(!r.dryer.gain.sides_ok);
    ASSERT_TRUE(!run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
}

TEST(a_stored_floor_faults_a_blocked_start_as_today)
{
    cold_with(15000, ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT, 1.0);
    r.plant_blocked = FAKE_THERMAL_FANS_BOTH;
    ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
    uint32_t at = fault_after(5U * RIG_MIN_MS);
    printf("  blocked 65/15, stored 35: fault %u at %u ms\n", r.dryer.fault, at);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_RESPONSE);
    ASSERT_EQ(at, PINNED_BLOCKED_65_15_MS);
}

TEST(a_clean_cycle_at_127_v_stores_the_floor)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNING);
    ASSERT_EQ(rate(), 0U);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending); /* in the log, not yet on the page */
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNED);
    ASSERT_TRUE(!run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    printf("  127 V, 45 C from 25 C: stored %u, best %u\n", r.log.gain_mc_s_per_pct,
           r.dryer.gain.best);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT);
    ASSERT_TRUE(!r.dryer.gain.pending); /* the rig stored the page once the gate went idle */
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
    ASSERT_EQ(rate(), ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT);
}

/* The known limitation (the brief's §2): a first clean cycle with both ducts blocked may store a
 * rate above the floor. It stays bounded, and another protection still catches the blocked cold
 * start that the floor's response rule catches. */
TEST(a_blocked_first_cycle_stores_a_bounded_rate_that_sides_still_catches)
{
    cold_with(25000, 0U, 1.0);
    r.plant_blocked = FAKE_THERMAL_FANS_BOTH;
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    (void)run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS);
    uint8_t stored = r.log.gain_mc_s_per_pct;
    uint8_t learned = r.dryer.gain.rate;
    printf("  blocked 45/25 first cycle: fault %u, learned %u, sides_ok %d, stored %u\n",
           r.dryer.fault, learned, (int)r.dryer.gain.sides_ok, stored);
    ASSERT_TRUE(stored <= ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
    ASSERT_TRUE(learned <= ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
    /* Here the cycle was not clean, so nothing was stored; the learned rate stands in for what a
     * clean blocked cycle would have stored. */
    uint8_t carried = stored != 0U ? stored : learned;
    /* And the worst any store can leave: the maximum. */
    const uint8_t rates[] = { carried, ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT };
    for (unsigned i = 0; i < 2U; i++) {
        cold_with(15000, rates[i], 1.0);
        r.plant_blocked = FAKE_THERMAL_FANS_BOTH;
        ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
        uint32_t at = fault_after(5U * RIG_MIN_MS);
        printf("  blocked 65/15 with stored %u: fault %u at %u ms\n", rates[i], r.dryer.fault, at);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
        ASSERT_TRUE(at <= LIMITATION_FAULT_MS);
    }
}

/* What makes counting the sides from the first reach safe: one duct blocked opens a gap the sides
 * rule faults on before any store, whichever duct, at 1× and 2×. The run stops at the fault. */
TEST(one_duct_blocked_faults_sides_and_stores_nothing)
{
    static const uint8_t ducts[] = { 1U, 2U }; /* left, then right */
    static const double gains[] = { 1.0, 2.0 };
    for (unsigned d = 0; d < 2U; d++) {
        for (unsigned g = 0; g < 2U; g++) {
            cold_with(25000, 0U, gains[g]);
            r.plant_blocked = ducts[d];
            ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
            uint32_t at = fault_after(CYCLE_MIN * RIG_MIN_MS);
            printf("  duct %u blocked, gain %.1f: fault %u at %u ms, stored %u\n", ducts[d],
                   gains[g], r.dryer.fault, at, r.log.gain_mc_s_per_pct);
            ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_SIDES);
            ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
            ASSERT_TRUE(!r.dryer.gain.pending);
        }
    }
}

TEST(a_capped_rate_is_used_and_never_stored)
{
    cold_with(25000, 0U, 5.0);
    ASSERT_EQ(rig_start(&r, 55U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_EQ(fault_after(ACE2K_DRYER_GAIN_LEARN_MS + 2000U), UINT32_MAX);
    printf("  gain 5.0, 55 C from 25 C: best %u, rate %u, capped %d\n", r.dryer.gain.best, rate(),
           (int)r.dryer.gain.capped);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNED);
    ASSERT_TRUE(r.dryer.gain.counted);
    ASSERT_TRUE(r.dryer.gain.best >= ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
    ASSERT_EQ(rate(), ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
    ASSERT_TRUE(r.dryer.gain.capped);
    (void)run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
}

TEST(a_window_without_duty_does_not_count)
{
    cold_with(25000, 0U, 1.0);
    ace2k_dryer_gain_begin(&r.dryer);
    ace2k_dryer_gain_learn(&r.dryer, 50000, ACE2K_DRYER_GAIN_MIN_DUTY_PCT_S - 1);
    ace2k_dryer_gain_learn(&r.dryer, 0, 3000);
    ace2k_dryer_gain_finish(&r.dryer);
    ASSERT_EQ(rate(), ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT); /* no window counted: the maximum */
    ASSERT_TRUE(r.dryer.gain.capped);
    ace2k_dryer_gain_begin(&r.dryer);
    ace2k_dryer_gain_learn(&r.dryer, 50000, ACE2K_DRYER_GAIN_MIN_DUTY_PCT_S - 1);
    ace2k_dryer_gain_learn(&r.dryer, 10001, ACE2K_DRYER_GAIN_MIN_DUTY_PCT_S);
    ace2k_dryer_gain_finish(&r.dryer);
    ASSERT_EQ(rate(), 51U); /* 10001 × 150 / 30000, rounded up */
    ASSERT_TRUE(!r.dryer.gain.capped);
    /* On the rig: outlets held above the drive target, so the controller fires nothing. */
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 60000;
    r.ovr_right_mc = 60000;
    ASSERT_EQ(fault_after(ACE2K_DRYER_GAIN_LEARN_MS + 2000U), UINT32_MAX);
    ASSERT_TRUE(!r.dryer.gain.counted);
    ASSERT_EQ(rate(), ACE2K_DRYER_GAIN_MAX_MC_S_PER_PCT);
}

/* Nothing stored: a rise that no duty drove is not judged while the rate is learned, and faults
 * on the first window after it, the maximum's ceiling being the floor alone at no duty. */
TEST(a_rise_with_no_duty_faults_when_learning_ends)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    uint32_t at = UINT32_MAX;
    for (uint32_t s = 1; s <= 70U && at == UINT32_MAX; s++) {
        r.ovr_left_mc = 62000 + ((int32_t)s * 300);
        r.ovr_right_mc = r.ovr_left_mc;
        at = fault_after(1000U);
    }
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_RESPONSE);
    ASSERT_TRUE(at >= ACE2K_DRYER_GAIN_LEARN_MS);
    ASSERT_TRUE(at <= ACE2K_DRYER_GAIN_LEARN_MS + 1100U);
    ASSERT_TRUE(!r.dryer.gain.counted);
}

TEST(a_fault_keeps_the_stored_rate)
{
    cold_with(25000, 50U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5000U);
    r.mains_present = false;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 50U);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
}

TEST(a_fault_before_the_store_drops_the_candidate)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending); /* heat was busy: the page not written yet */
    r.mains_present = false;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
    ASSERT_TRUE(!r.dryer.gain.pending);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNED); /* learned, never stored */
}

/* A new cycle that begins while the candidate is still off the page takes it as stored; a fault
 * that drops it leaves the rate learned, not stored. */
TEST(a_dropped_candidate_taken_by_a_new_cycle_does_not_read_stored)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending);
    ace2k_dryer_gain_begin(&r.dryer); /* the entry into HEATING of the next cycle */
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
    ASSERT_TRUE(r.dryer.gain.pending);
    ace2k_dryer_gain_fault(&r.dryer);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
    ASSERT_TRUE(!r.dryer.gain.pending);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNED);
    ASSERT_EQ(rate(), ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT);
}

/* A latch in COOLDOWN, after a stop and before the page store, drops the candidate too. */
TEST(a_cutout_in_the_cooldown_drops_the_candidate)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    r.cutout = true;
    rig_tick(&r); /* the dryer's tick runs before the rig stores the page */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CUTOUT);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
    ASSERT_TRUE(!r.dryer.gain.pending);
}

/* Nothing stored: only the response rule waits while the rate is learned. The other protections
 * run from the entry into HEATING through the same ace2k_dryer_protect_check, ahead of the
 * response rule: the chamber ceiling faults at 30 s, the sides at 60 s, as with a rate stored.
 * not_heating judges at 300 s, after learning, by its own definition. */
TEST(the_other_protections_run_while_the_rate_is_learned)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 70000; /* over 55 + 10 °C */
    uint32_t at = fault_after(ACE2K_DRYER_GAIN_LEARN_MS);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
    ASSERT_TRUE(at <= ACE2K_DRYER_CHAMBER_OVER_MS + 1000U);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNING);
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 61000;
    r.ovr_right_mc = 45000;
    at = fault_after(ACE2K_DRYER_GAIN_LEARN_MS + 1000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_SIDES);
    ASSERT_TRUE(at <= ACE2K_DRYER_SIDES_MS + 1000U);
}

TEST(the_forget_is_refused_while_heating)
{
    cold_with(25000, 50U, 1.0);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 45U, CYCLE_MIN, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_STARTING);
    ASSERT_EQ(ace2k_dryer_gain_forget(&r.dryer), ACE2K_DRYER_BUSY);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS + 100U));
    uint32_t gen = ace2k_dryer_log_generation(&r.dryer);
    ASSERT_EQ(ace2k_dryer_gain_forget(&r.dryer), ACE2K_DRYER_BUSY);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 50U);
    ASSERT_EQ(rate(), 50U);
    ASSERT_EQ(ace2k_dryer_log_generation(&r.dryer), gen);
}

TEST(the_forget_clears_the_rate_and_the_candidate)
{
    /* IDLE. */
    cold_with(25000, 50U, 1.0);
    ace2k_dryer_gain_init(&r.dryer);
    uint32_t gen = ace2k_dryer_log_generation(&r.dryer);
    ASSERT_EQ(ace2k_dryer_gain_forget(&r.dryer), ACE2K_DRYER_OK);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
    ASSERT_EQ(rate(), 0U);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_NONE);
    ASSERT_TRUE(ace2k_dryer_log_generation(&r.dryer) != gen);
    /* COOLDOWN, with a candidate due and not yet on the page. */
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(ace2k_dryer_gain_forget(&r.dryer), ACE2K_DRYER_OK);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
    ASSERT_TRUE(!r.dryer.gain.pending);
    /* FAULT. */
    cold_with(25000, 50U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    r.mains_present = false;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(ace2k_dryer_gain_forget(&r.dryer), ACE2K_DRYER_OK);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
}

TEST(a_stop_while_learning_stores_nothing)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    rig_run_ms(&r, ACE2K_DRYER_GAIN_LEARN_MS / 2U);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_LEARNING);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    rig_run_ms(&r, 15U * RIG_MIN_MS);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
}

TEST(a_cycle_shorter_than_ten_minutes_after_the_reach_stores_nothing)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, 9U), ACE2K_DRYER_OK);
    ASSERT_TRUE(!run_cycle(10U * RIG_MIN_MS));
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_TRUE(r.dryer.gain.reached);
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 0U);
}

TEST(a_retarget_after_the_first_reach_still_stores)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    for (uint32_t t = 0; t < 10U * RIG_MIN_MS && !r.dryer.gain.reached; t += RIG_TICK_MS) {
        rig_tick(&r);
    }
    ASSERT_TRUE(r.dryer.gain.reached);
    uint32_t reached_ms = r.dryer.gain.reached_ms;
    ASSERT_EQ(ace2k_dryer_retarget(&r.dryer, 55U, CYCLE_MIN, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_EQ(r.dryer.gain.reached_ms, reached_ms); /* the first reach counts */
    ASSERT_EQ(r.log.gain_mc_s_per_pct, ACE2K_DRYER_GAIN_FLOOR_MC_S_PER_PCT);
}

TEST(a_stored_rate_is_never_overwritten)
{
    cold_with(25000, 50U, 3.0);
    ASSERT_EQ(rig_start(&r, 55U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(!run_cycle((CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_EQ(r.log.gain_mc_s_per_pct, 50U);
    ASSERT_EQ(source(), ACE2K_DRYER_GAIN_STORED);
    ASSERT_EQ(rate(), 50U);
    ASSERT_TRUE(!r.dryer.gain.counted); /* it never learned */
}

/* The page's store settles the candidate: an older generation leaves it waiting. */
TEST(a_store_of_an_older_generation_keeps_the_candidate)
{
    cold_with(25000, 0U, 1.0);
    ASSERT_EQ(rig_start(&r, 45U, CYCLE_MIN), ACE2K_DRYER_OK);
    ASSERT_TRUE(run_until_due(CYCLE_MIN * RIG_MIN_MS));
    ASSERT_TRUE(r.dryer.gain.pending);
    ace2k_dryer_log_stored(&r.dryer, r.dryer.gain.pending_gen - 1U);
    ASSERT_TRUE(r.dryer.gain.pending);
    ace2k_dryer_log_stored(&r.dryer, ace2k_dryer_log_generation(&r.dryer));
    ASSERT_TRUE(!r.dryer.gain.pending);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
