/* The dryer's protections and the spool rule, each
 * fired in its scenario on the rig; the normal cycles that must never fire them are
 * test_dryer.c's. */
#include "fake_dryer_rig.h"
#include "test.h"

/* The firmware never includes a test header, so the fit's ceiling is copied into dryer.h by
 * hand; a re-fit that moves it fails this build until the copy follows. */
_Static_assert(ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT == THERMAL_RISE_MAX_MC_S_PER_PCT,
               "dryer.h's rise ceiling is tools/thermal_fit.py's (tests/thermal_params.h)");

// NOLINTNEXTLINE(readability-identifier-naming)
static struct rig r; /* kept off the stack */

/* A 55 °C cycle (drive 62 °C) in HEATING with the NTCs overridden. */
static void heating_with_ntc(int32_t left_mc, int32_t right_mc)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = left_mc;
    r.ovr_right_mc = right_mc;
}

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

TEST(a_dead_heater_faults_not_heating_at_300_s)
{
    rig_init(&r, 25000);
    r.heater_dead = true;
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    uint32_t at = fault_after(310000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NOT_HEATING);
    ASSERT_TRUE(at >= 299000U);
    ASSERT_TRUE(at <= 301000U);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
}

TEST(ntcs_already_near_the_drive_target_are_not_a_dead_heater)
{
    heating_with_ntc(60500, 60500);
    ASSERT_EQ(fault_after(310000U), UINT32_MAX);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
}

TEST(ntcs_rising_with_the_duty_at_zero_fault_the_response)
{
    heating_with_ntc(62000, 62000);
    uint32_t at = UINT32_MAX;
    for (uint32_t s = 1; s <= 40U && at == UINT32_MAX; s++) {
        r.ovr_left_mc = 62000 + ((int32_t)s * 500);
        r.ovr_right_mc = r.ovr_left_mc;
        at = fault_after(1000U);
    }
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_RESPONSE);
    ASSERT_TRUE(at <= 32000U);
    ASSERT_TRUE(r.max_ntc_mc < ACE2K_HEAT_NTC_MAX_MC);
}

TEST(a_blocked_duct_faults_before_any_ntc_reaches_85_c)
{
    rig_init(&r, 25000);
    r.plant_blocked = FAKE_THERMAL_FANS_BOTH;
    ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
    (void)fault_after(20U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    uint8_t f = r.dryer.fault;
    ASSERT_TRUE(f == ACE2K_DRYER_FAULT_RESPONSE || f == ACE2K_DRYER_FAULT_SIDES ||
                f == ACE2K_DRYER_FAULT_NOT_HEATING || f == ACE2K_DRYER_FAULT_NTC);
    ASSERT_TRUE(r.max_ntc_mc < ACE2K_HEAT_NTC_MAX_MC);
}

TEST(sixteen_degrees_apart_for_a_minute_faults_the_sides)
{
    heating_with_ntc(61000, 45000);
    uint32_t at = fault_after(70000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_SIDES);
    ASSERT_TRUE(at >= 59000U);
    ASSERT_TRUE(at <= 61000U);
}

TEST(fourteen_degrees_apart_is_not_a_fault)
{
    heating_with_ntc(61000, 47000);
    ASSERT_EQ(fault_after(120000U), UINT32_MAX);
}

TEST(the_chamber_above_the_ceiling_for_30_s_faults)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 65500;
    uint32_t start = r.now_ms;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_FAULT, 40000U));
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
    ASSERT_TRUE(r.now_ms - start >= 29000U);
    ASSERT_TRUE(r.now_ms - start <= 31000U);
}

TEST(the_chamber_just_below_the_ceiling_is_not_a_fault)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 64500;
    rig_run_ms(&r, 5U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
}

TEST(the_chamber_above_80_c_faults_on_the_next_tick)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 80500;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
}

TEST(a_stale_chamber_faults_while_heating_and_not_while_idle)
{
    rig_init(&r, 25000);
    r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_run_ms(&r, 10000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    r.chamber_age_ms = 0U;
    rig_tick(&r);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_STALE);
}

/* While leased, an invalid NTC stops the firing on the tick that sees it (heat's verdict) and
 * faults the cycle when heat latches it, ACE2K_HEAT_NTC_INVALID_TICKS ticks later (author ruling:
 * a one-tick glitch no longer ends the cycle); the lease is gone on the fault's tick. */
TEST(an_invalid_ntc_stops_the_firing_at_once_and_faults_after_five_ticks)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat)); /* 50 °C against a 62 °C drive: heating */
    r.ovr_left_valid = false;
    rig_tick(&r); /* the tick that sees it */
    uint32_t fired = r.heat.fired;
    uint32_t ticks = 1U;
    while (r.dryer.state != ACE2K_DRYER_FAULT && ticks < 50U) {
        rig_tick(&r);
        ticks++;
    }
    ASSERT_EQ(ticks, ACE2K_HEAT_NTC_INVALID_TICKS);
    ASSERT_EQ(r.heat.fired, fired); /* not a half-cycle after the tick that saw it */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_TRUE(!r.gate);
}

/* A glitch of 1–4 ticks while heating: no fault, the lease kept (renewals ride it out), and
 * the firing resumes. */
TEST(an_ntc_glitch_under_five_ticks_leaves_the_cycle_heating)
{
    for (uint32_t k = 1U; k < ACE2K_HEAT_NTC_INVALID_TICKS; k++) {
        heating_with_ntc(50000, 50000);
        rig_run_ms(&r, 1000U);
        for (uint32_t n = 0; n < 3U; n++) { /* three glitches, one across a renewal */
            r.ovr_right_valid = false;
            rig_run_ms(&r, k * RIG_TICK_MS);
            r.ovr_right_valid = true;
            rig_run_ms(&r, 300U);
        }
        uint32_t fired = r.heat.fired;
        rig_run_ms(&r, 1000U);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
        ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED);
        ASSERT_TRUE(r.heat.fired > fired);
    }
}

/* Not leased (the duty at 0, or between leases), an invalid NTC is the dryer's own debounce:
 * a lease refused for it waits, a short glitch passes, a lasting one faults NTC. */
TEST(an_invalid_ntc_with_no_lease_is_judged_by_the_dryers_debounce)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ace2k_heat_release(&r.heat);
    r.ovr_left_valid = false;
    rig_tick(&r);
    r.dryer.ctl.renew_next_ms = r.now_ms - 1U; /* a renewal falls inside the glitch */
    rig_run_ms(&r, 2U * RIG_TICK_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    r.ovr_left_valid = true;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    ace2k_heat_release(&r.heat);
    r.ovr_left_valid = false;
    uint32_t at = fault_after(2000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC);
    ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED); /* the dryer's fault, not heat's */
    ASSERT_TRUE(at != UINT32_MAX);
}

/* Rule 13: a lease refused because a store is in progress is no fault — asked again next tick. */
TEST(a_lease_refused_for_a_store_in_progress_waits)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ace2k_heat_release(&r.heat);
    ace2k_heat_set_inhibit(&r.heat, true);
    rig_run_ms(&r, 200U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ace2k_heat_set_inhibit(&r.heat, false);
    rig_run_ms(&r, 2U * RIG_TICK_MS);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
}

TEST(absent_mains_fault_the_cycle)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, RIG_MIN_MS);
    r.mains_present = false;
    rig_tick(&r); /* at once: no debounce for a mains gone */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
}

/* The bench's case: the mains read off-band for one tick, 50 ticks or one whole 1 s
 * window from the tick the dryer takes its first lease.  The lease waits (refused as transient),
 * nothing fires on the off-band mains, and the cycle heats once it reads plausible again. */
TEST(a_mains_dip_at_the_first_lease_is_no_fault_and_the_cycle_heats)
{
    static const uint32_t dip_ticks[] = { 1U, 50U, 101U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(dip_ticks); i++) {
        rig_init(&r, 25000);
        ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_TRUE(!ace2k_heat_leased(&r.heat)); /* the first lease is the next tick's */
        r.mains_plausible = false;
        uint32_t fired = r.heat.fired;
        rig_run_ms(&r, dip_ticks[i] * RIG_TICK_MS);
        ASSERT_EQ(r.heat.fired, fired); /* not a half-cycle on the off-band mains */
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED);
        r.mains_plausible = true;
        /* the first lease waits for heat's bucket at its restart level (a whole window: 1.1 s) */
        rig_run_ms(&r, 2000U);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
        ASSERT_TRUE(ace2k_heat_leased(&r.heat));
        ASSERT_TRUE(r.heat.fired > fired);
        ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 0U);
    }
}

/* A dip that begins in STARTING and outlasts the entry into HEATING: no fault either. */
TEST(a_mains_dip_across_the_start_is_no_fault)
{
    rig_init(&r, 25000);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    r.mains_plausible = false;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS));
    rig_run_ms(&r, 5U * RIG_TICK_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(!ace2k_heat_mains_implausible_held(&r.heat)); /* a dip, not a lasting fault */
    r.mains_plausible = true;
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
}

/* The same dip while heating with the lease held: no half-cycle during it, no fault, the lease
 * kept (its renewals ride it out) and the firing resumes. */
TEST(a_mains_dip_while_heating_stops_the_firing_without_a_fault)
{
    static const uint32_t dip_ticks[] = { 1U, 50U, 101U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(dip_ticks); i++) {
        heating_with_ntc(50000, 50000);
        rig_run_ms(&r, 1000U);
        ASSERT_TRUE(ace2k_heat_leased(&r.heat));
        r.mains_plausible = false;
        rig_tick(&r); /* the tick that sees it */
        uint32_t fired = r.heat.fired;
        rig_run_ms(&r, (dip_ticks[i] - 1U) * RIG_TICK_MS);
        ASSERT_EQ(r.heat.fired, fired);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_TRUE(ace2k_heat_leased(&r.heat));
        r.mains_plausible = true;
        rig_run_ms(&r, 1000U);
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
        ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
        ASSERT_TRUE(r.heat.fired > fired);
    }
}

/* The mains off-band for heat's threshold while leased: heat latches MAINS_IMPLAUSIBLE and the
 * dryer faults MAINS on that tick, with no half-cycle from the first tick that saw it. */
TEST(a_mains_off_band_past_the_threshold_while_heating_faults_mains)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.mains_plausible = false;
    rig_tick(&r);
    uint32_t fired = r.heat.fired;
    uint32_t ticks = 1U;
    while (r.dryer.state != ACE2K_DRYER_FAULT && ticks < 1000U) {
        rig_tick(&r);
        ticks++;
    }
    ASSERT_EQ(ticks, ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    ASSERT_EQ(r.heat.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(r.heat.reason, ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    ASSERT_EQ(r.heat.fired, fired);
}

/* With no lease held (at the first lease, or a duty of 0) the same persistence — heat's bucket,
 * one counter for both layers — faults MAINS, well inside the start timeout. */
TEST(a_mains_off_band_past_the_threshold_with_no_lease_faults_mains)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.mains_plausible = false;
    uint32_t since = r.now_ms;
    uint32_t fired = r.heat.fired;
    while (r.dryer.state != ACE2K_DRYER_FAULT && r.now_ms - since < 5000U) {
        rig_tick(&r);
    }
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    ASSERT_EQ(r.now_ms - since, ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * RIG_TICK_MS);
    ASSERT_TRUE(r.now_ms - since < ACE2K_DRYER_START_TIMEOUT_MS);
    ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED); /* never leased: the dryer's fault */
    ASSERT_EQ(r.heat.fired, fired);
}

TEST(the_first_reason_stays)
{
    heating_with_ntc(61000, 45000);
    (void)fault_after(70000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_SIDES);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 81000;
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_SIDES);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 1U);
}

TEST(a_strand_inserted_with_no_host_lowers_to_45_c_without_a_chamber_fault)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 65U, 120U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 60U * RIG_MIN_MS);
    r.link_ok = false;
    r.insert_mask |= 0x4U; /* lane 3 */
    rig_tick(&r);
    ASSERT_EQ(r.dryer.target_c, ACE2K_DRYER_SPOOL_SAFE_C);
    ASSERT_EQ(r.dryer.highest_target_c, 65U);
    ASSERT_TRUE((r.dryer.notices & ACE2K_DRYER_NOTICE_LOWERED) != 0U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_LOWERED), 1U);
    ASSERT_EQ(r.ev[r.ev_n - 1U].arg, 3U);
    rig_run_ms(&r, 15U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
}

TEST(the_spool_rule_leaves_the_target_alone_with_the_host_low_targets_and_old_strands)
{
    rig_init(&r, 25000);
    r.insert_mask = 0x1U; /* lane 1's strand in before the start */
    rig_tick(&r);
    ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
    r.insert_mask |= 0x2U; /* link up */
    rig_tick(&r);
    ASSERT_EQ(r.dryer.target_c, 65U);
    r.link_ok = false;
    rig_tick(&r); /* no new strand: lane 1 was in already */
    ASSERT_EQ(r.dryer.target_c, 65U);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 11U * RIG_MIN_MS));
    r.link_ok = true;
    rig_tick(&r);
    ASSERT_EQ(rig_start(&r, 40U, 60U), ACE2K_DRYER_OK);
    r.link_ok = false;
    r.insert_mask |= 0x8U;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.target_c, 40U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_LOWERED), 0U);
}

/* A strand that glitches (present → absent → present) with no host: the rule may lower the
 * target, never raise it; once at 45 °C a further glitch changes nothing. */
TEST(a_strand_glitch_with_no_host_only_ever_lowers_the_target)
{
    rig_init(&r, 25000);
    r.insert_mask = 0x2U; /* lane 2's strand in before the start */
    rig_tick(&r);
    ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
    r.link_ok = false;
    uint8_t highest_seen = r.dryer.target_c;
    for (uint32_t i = 0; i < 4U; i++) {
        r.insert_mask = (i % 2U == 0U) ? 0x0U : 0x2U; /* absent, present, absent, present */
        rig_tick(&r);
        ASSERT_TRUE(r.dryer.target_c <= highest_seen);
        highest_seen = r.dryer.target_c;
    }
    ASSERT_EQ(r.dryer.target_c, ACE2K_DRYER_SPOOL_SAFE_C);
    ASSERT_EQ(r.dryer.highest_target_c, 65U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_LOWERED), 1U);
    rig_run_ms(&r, RIG_MIN_MS);
    ASSERT_EQ(r.dryer.target_c, ACE2K_DRYER_SPOOL_SAFE_C);
}

/* Rule 10: a cutout that trips after another fault is still a cutout.  The first reason stays,
 * but once the cutout recloses and everything cools, clear answers CUTOUT and start refuses. */
TEST(a_cutout_after_an_ntc_fault_is_never_cleared_by_command)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.ovr_left_valid = false;
    rig_run_ms(&r, 500U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC);
    r.cutout = true;
    rig_run_ms(&r, 1000U);
    r.cutout = false;
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC); /* the first reason stays */
    ASSERT_TRUE(r.dryer.cutout_seen);
    ASSERT_TRUE(r.heat.cutout_seen);
    r.ovr_ntc = false; /* the NTCs back, valid and reading the cooling plant */
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_CUTOUT);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_TRUE(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms) != ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_heat_clear(&r.heat), -ACE2K_EREFUSED);
}

/* Rule 10 across a software reset: a cutout that trips while the dryer is already in FAULT for
 * another reason still reaches the page, and a re-init without a power-on reset holds FAULT. */
TEST(a_cutout_after_an_ntc_fault_is_persisted_across_a_software_reset)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.ovr_left_valid = false;
    rig_run_ms(&r, 500U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC);
    ASSERT_EQ(r.log.flags & ACE2K_DRYER_LOG_CUTOUT, 0U);
    uint32_t gen = ace2k_dryer_log_generation(&r.dryer);
    r.cutout = true;
    rig_tick(&r);
    ASSERT_TRUE((r.log.flags & ACE2K_DRYER_LOG_CUTOUT) != 0U);
    ASSERT_TRUE(ace2k_dryer_log_generation(&r.dryer) != gen);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC); /* the first reason stays */
    r.cutout = false;
    /* the software reset: every core re-initialised, the log (the flash page) kept */
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, false, r.now_ms);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CUTOUT);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_CUTOUT);
}

/* The sticky cutout alone refuses a start as CUTOUT (the other refusals out of the way). */
TEST(a_cutout_seen_refuses_a_start_as_cutout)
{
    rig_init(&r, 25000);
    r.dryer.cutout_seen = true;
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_CUTOUT);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
}

/* The flap pulses counted from a mark: each flap's opening and closing pulses. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct flap_mark {
    uint32_t opens[ACE2K_FLAP_COUNT], closes[ACE2K_FLAP_COUNT];
};

static struct flap_mark flap_mark_now(void)
{
    struct flap_mark m;
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        m.opens[f] = r.flap_opens[f];
        m.closes[f] = r.flap_closes[f];
    }
    return m;
}

/* Since the mark, each flap opened once and closed once, the opening pulses first (the fault's
 * cool-down opens both at its entry and closes both at its end), never both coils, both closed
 * at the end.  The simulator's plant has no flap term; the open flaps' gain is for the bench to measure. */
static void assert_flaps_opened_then_closed(const struct flap_mark *m, bool opened_first)
{
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_opens[f] - m->opens[f], 1U);
        ASSERT_EQ(r.flap_closes[f] - m->closes[f], 1U);
    }
    ASSERT_TRUE(opened_first);
    ASSERT_TRUE(!r.coil_both);
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
}

/* Right after a fault's entry: both flaps pulsed open, neither closed yet. */
static bool opened_at_entry(const struct flap_mark *m)
{
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        if (r.flap_opens[f] - m->opens[f] != 1U || r.flap_closes[f] != m->closes[f]) {
            return false;
        }
    }
    return true;
}

TEST(a_sensor_fault_clears_only_once_cool_and_with_the_cause_gone)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    struct flap_mark mark = flap_mark_now();
    r.ovr_left_valid = false;
    rig_run_ms(&r, 500U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NTC);
    bool opened_first = opened_at_entry(&mark);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NOT_COOL);
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);   /* an invalid NTC is hot: 10 min */
    assert_flaps_opened_then_closed(&mark, opened_first); /* an NTC fault's cool-down too */
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_SENSORS);
    r.ovr_ntc = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    rig_tick(&r);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_CLEARED), 1U);
}

/* True when either NTC of the last tick reads above limit_mc. */
static bool an_ntc_above(int32_t limit_mc)
{
    if (r.last_in.ntc_left_mc > limit_mc) {
        return true;
    }
    return r.last_in.ntc_right_mc > limit_mc;
}

/* A protection's fault on a real, hot cycle: no half-cycle after it, the fans held while an NTC
 * is above 45 °C, the fault's cool-down opens both flaps at its entry and ends in them pulsed
 * closed and the fans handed to rule 7 (off only once both NTCs read 42 °C or less) with the
 * state still FAULT; the clear is refused while the chamber is still over its ceiling and
 * accepted once it is not. */
TEST(a_fault_cools_down_opens_then_closes_the_flaps_hands_the_fans_to_rule_7_and_clears)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 15U * RIG_MIN_MS);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 80500;
    uint32_t pulses = r.flap_pulses;
    struct flap_mark mark = flap_mark_now();
    rig_tick(&r); /* the fault, and the first flap's opening on this tick */
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_TRUE(r.fans_on);
    uint32_t fired = r.heat.fired;
    rig_run_ms(&r, RIG_FLAP_PAIR_MS);
    bool opened_first = opened_at_entry(&mark);
    bool fans_off_hot = false;
    bool fans_off_warm = false;
    bool fans_stopped = false;
    for (uint32_t t = 0; t < 60U * RIG_MIN_MS; t += RIG_TICK_MS) {
        rig_tick(&r);
        bool hot = an_ntc_above(ACE2K_AIRFLOW_FAN_ON_MC);
        bool warm = an_ntc_above(ACE2K_AIRFLOW_FAN_OFF_MC);
        if (hot && !r.fans_on) {
            fans_off_hot = true;
        }
        if (r.dryer.cool == ACE2K_DRYER_COOL_RELEASED && !r.fans_on) {
            fans_stopped = true;
            if (warm) {
                fans_off_warm = true;
            }
        }
    }
    ASSERT_EQ(r.heat.fired, fired);
    ASSERT_TRUE(!fans_off_hot);
    ASSERT_TRUE(!fans_off_warm);
    ASSERT_TRUE(fans_stopped);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    ASSERT_EQ(r.flap_pulses - pulses, 4U);
    assert_flaps_opened_then_closed(&mark, opened_first);
    ASSERT_TRUE(r.airflow.owner != ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 1U);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_FAULTED);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NOT_COOL); /* still over */
    r.ovr_chamber = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    rig_tick(&r);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_CLEARED), 1U);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
}

TEST(a_cutout_fault_is_never_cleared_by_command)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5U * RIG_MIN_MS);
    struct flap_mark mark = flap_mark_now();
    r.cutout = true;
    rig_tick(&r); /* the latch is seen before the state's work: the first flap on this tick */
    r.cutout = false;
    rig_run_ms(&r, RIG_FLAP_PAIR_MS);
    bool opened_first = opened_at_entry(&mark);
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    assert_flaps_opened_then_closed(&mark, opened_first); /* a cutout's cool-down opens them too */
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_CUTOUT);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
}

TEST(the_protections_stay_quiet_through_a_normal_warm_up)
{
    rig_init(&r, 15000);
    ASSERT_EQ(rig_start(&r, 65U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 20U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 0U);
}

/* The protections alone on the plant, the duty imposed: `duty_pct` for on_s seconds from a unit
 * at 25 °C, then 0 for off_s, the fans on.  Returns the first fault (or NONE); *overshoot_mc the
 * hotter NTC's rise after the cut. */
static uint8_t imposed_duty_run(uint8_t duty_pct, uint32_t on_s, uint32_t off_s,
                                int32_t *overshoot_mc)
{
    rig_init(&r, 25000);
    r.dryer.highest_target_c = 65U; /* the chamber's ceiling out of the way */
    r.dryer.target_c = 65U;
    struct ace2k_dryer_inputs in;
    rig_readings(&r, &in);
    ace2k_dryer_protect_begin(&r.dryer, &in, r.now_ms);
    int32_t at_cut = 0;
    int32_t peak = INT32_MIN;
    uint32_t end_ms = (on_s + off_s) * 1000U;
    for (uint32_t t = 0; t < end_ms; t += RIG_TICK_MS) {
        bool on = t < on_s * 1000U;
        uint8_t duty = on ? duty_pct : 0U;
        if (t % RIG_PLANT_MS == 0U) {
            fake_thermal_step(&r.plant, duty, FAKE_THERMAL_FANS_BOTH, RIG_PLANT_MS);
        }
        r.now_ms += RIG_TICK_MS;
        r.dryer.ctl.duty_cpct = (uint16_t)(duty * ACE2K_DRYER_CPCT_PER_PCT);
        rig_readings(&r, &in);
        int32_t hot = ace2k_dryer_hotter_mc(&in);
        if (on) {
            at_cut = hot;
        } else if (hot > peak) {
            peak = hot;
        }
        uint8_t f = ace2k_dryer_protect_check(&r.dryer, &in, r.now_ms);
        if (f != ACE2K_DRYER_FAULT_NONE) {
            printf("  %u %% for %u s: fault %u at %u ms\n", (unsigned)duty_pct, (unsigned)on_s,
                   (unsigned)f, (unsigned)t);
            *overshoot_mc = peak - at_cut;
            return f;
        }
    }
    *overshoot_mc = peak - at_cut;
    return ACE2K_DRYER_FAULT_NONE;
}

/* The outlet NTC keeps rising 10–15 s after the gate stops (docs/hardware.md "Heater": up to
 * +3.8 °C at 50 %; the model gives +3.3 °C): heat already delivered, never an impossible
 * response — the ceiling counts the duty of the lag before the window too. */
TEST(the_overshoot_after_a_cut_to_zero_is_not_an_impossible_response)
{
    static const uint8_t duties[] = { 35U, 50U, 90U };
    int32_t worst = 0;
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(duties); i++) {
        for (uint32_t on_s = 10U; on_s <= 60U; on_s += 5U) {
            int32_t over = 0;
            ASSERT_EQ(imposed_duty_run(duties[i], on_s, 90U, &over), ACE2K_DRYER_FAULT_NONE);
            if (over > worst) {
                worst = over;
            }
        }
    }
    printf("  the largest overshoot after a cut: %d mdegC\n", (int)worst);
    ASSERT_TRUE(worst > ACE2K_DRYER_RISE_FLOOR_MC); /* the floor alone would not have held it */
    int32_t at50 = 0;
    ASSERT_EQ(imposed_duty_run(50U, 40U, 90U, &at50), ACE2K_DRYER_FAULT_NONE);
    ASSERT_TRUE(at50 >= 3000); /* the plant reproduces the ~3.3 °C of the 50 % run */
}

/* The bench's own numbers, not the model's: 50 % for 40 s rising 0.8 °C/s, then 0 and the NTC
 * climbing +3.8 °C over the next 13 s (docs/hardware.md "Heater"), then flat. */
TEST(the_measured_overshoot_after_50_percent_is_not_an_impossible_response)
{
    rig_init(&r, 25000);
    r.dryer.highest_target_c = 65U;
    r.dryer.target_c = 65U;
    struct ace2k_dryer_inputs in;
    rig_readings(&r, &in);
    in.ntc_left_mc = in.ntc_right_mc = 18000;
    ace2k_dryer_protect_begin(&r.dryer, &in, r.now_ms);
    uint8_t f = ACE2K_DRYER_FAULT_NONE;
    for (uint32_t t = 0; t < 120000U && f == ACE2K_DRYER_FAULT_NONE; t += RIG_TICK_MS) {
        int32_t ntc = 18000 + (int32_t)(t * 8U / 10U); /* 0.8 °C/s */
        uint16_t duty = 50U;
        if (t >= 40000U) {
            uint32_t after = t - 40000U < 13000U ? t - 40000U : 13000U;
            ntc = 50000 + (int32_t)(after * 3800U / 13000U);
            duty = 0U;
        }
        r.now_ms += RIG_TICK_MS;
        r.dryer.ctl.duty_cpct = (uint16_t)(duty * ACE2K_DRYER_CPCT_PER_PCT);
        rig_readings(&r, &in);
        in.ntc_left_mc = in.ntc_right_mc = ntc;
        f = ace2k_dryer_protect_check(&r.dryer, &in, r.now_ms);
    }
    ASSERT_EQ(f, ACE2K_DRYER_FAULT_NONE);
}

/* A clear that heat refuses names heat's cause: a latch for the fans, then the mains gone or an
 * NTC invalid by the time of the clear — NO_MAINS or SENSORS, never "faulted, clear again". */
TEST(a_clear_refused_by_heat_names_the_limit_still_out_of_bounds)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.fan_fail = true;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
    ASSERT_EQ(r.heat.state, ACE2K_HEAT_LATCHED);
    r.fan_fail = false;
    r.ovr_left_mc = 30000;
    r.ovr_right_mc = 30000;
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    r.mains_present = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NO_MAINS);
    r.mains_present = true;
    r.ovr_left_valid = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_SENSORS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    r.ovr_left_valid = true;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(r.heat.state, ACE2K_HEAT_IDLE);
}

/* The log's heating time, in ms. */
static uint32_t log_heat_ms(void)
{
    return (r.log.heat_s * 1000U) + r.log.heat_ms_frac;
}

/* Round 1, item 3: a lease silenced by a mains dip heats nothing, and the log counts nothing for
 * it; the heating time runs again once the firing resumes. */
TEST(a_mains_dip_adds_no_heating_time_to_the_log)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    r.mains_plausible = false;
    rig_tick(&r);
    uint32_t before = log_heat_ms();
    rig_run_ms(&r, 100U * RIG_TICK_MS);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    ASSERT_EQ(log_heat_ms(), before);
    r.mains_plausible = true;
    rig_run_ms(&r, 1000U);
    ASSERT_TRUE(log_heat_ms() > before);
}

/* Round 1, item 1: every dip ridden out in a cycle is counted and raises the notice; a cycle with
 * none leaves it clear, and a new start clears both. */
TEST(each_mains_dip_ridden_out_is_counted_and_noticed)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, 0U);
    for (uint32_t n = 0; n < 2U; n++) {
        r.mains_plausible = false;
        rig_run_ms(&r, 101U * RIG_TICK_MS);
        ASSERT_EQ(r.dryer.mains_dips, n); /* counted on its end, not on its start */
        r.mains_plausible = true;
        rig_run_ms(&r, 3000U); /* the bucket drained */
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(r.dryer.mains_dips, 2U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, ACE2K_DRYER_NOTICE_MAINS_DIP);
    struct ace2k_dryer_status st;
    ace2k_dryer_status(&r.dryer, &st, r.now_ms);
    ASSERT_EQ(st.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, ACE2K_DRYER_NOTICE_MAINS_DIP);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 1200000U));
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.mains_dips, 0U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, 0U);
}

/* Round 1, item 5: a start asked during a dip is accepted (the dip is ridden out in STARTING and
 * HEATING); one asked once heat holds the mains implausible is refused NO_MAINS; absent at once. */
TEST(a_start_during_a_dip_is_accepted_and_refused_once_the_dip_lasts)
{
    rig_init(&r, 25000);
    r.mains_plausible = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    r.mains_plausible = true;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS + 100U));
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(r.dryer.mains_dips, 1U); /* in progress at the start, ridden out: counted */
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, ACE2K_DRYER_NOTICE_MAINS_DIP);

    rig_init(&r, 25000);
    r.mains_plausible = false;
    rig_run_ms(&r, ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * RIG_TICK_MS);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_NO_MAINS);
    r.mains_plausible = true;
    r.mains_present = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_NO_MAINS);
}

/* Round 1, item 5: a mains fault clears through a later dip shorter than heat's bucket, not
 * through one it holds. */
TEST(a_mains_fault_clears_through_a_dip_but_not_a_lasting_one)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.mains_plausible = false; /* no lease yet: heat is not latched, the dryer faults */
    (void)fault_after(5000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 30000;
    r.ovr_right_mc = 30000;
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U); /* the mains still off-band throughout */
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NO_MAINS);
    r.mains_plausible = true;
    rig_run_ms(&r, 3000U);
    r.mains_plausible = false; /* a fresh dip at the clear */
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
}

/* Round 1, item 6: heat latched on the fans; at the clear an NTC reads invalid and the mains
 * off-band — the refusal names the sensor, not the dip. */
TEST(a_clear_refused_with_an_invalid_ntc_and_a_dip_names_the_sensor)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.fan_fail = true;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
    r.fan_fail = false;
    r.ovr_left_mc = 30000;
    r.ovr_right_mc = 30000;
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    r.ovr_left_valid = false;
    r.mains_plausible = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_SENSORS);
}

/* Round 2, items 3–4: a dip that ends in the mains fault was not ridden out and is not counted;
 * the mains absent is never a dip. */
TEST(a_dip_ending_in_a_fault_and_an_absent_mains_are_not_counted)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.mains_plausible = false;
    (void)fault_after(5000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    r.mains_plausible = true;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.mains_dips, 0U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, 0U);

    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.mains_present = false;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    r.mains_present = true;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.mains_dips, 0U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, 0U);
}

/* Round 3, item 2: heat latched on a gate stuck high, the gate still reading high at the clear
 * and the mains one tick off-band: the refusal names the heat, not the mains. */
TEST(a_clear_with_the_gate_high_and_a_dip_names_the_heat_not_the_mains)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    r.gate_stuck = true;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_FAULT, 1000U));
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_HEAT);
    rig_run_ms(&r, 60U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    r.mains_plausible = false;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_FAULTED);
    r.mains_plausible = true;
    r.gate_stuck = false;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
}

/* Round 3, item 3: a mains fault does not clear while heat's bucket is above its restart level,
 * the mains plausible again or not; it clears once drained to it. */
TEST(a_mains_fault_clears_only_once_the_bucket_is_at_its_restart_level)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.mains_plausible = false;
    (void)fault_after(5000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 30000;
    r.ovr_right_mc = 30000;
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    r.mains_plausible = true;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NO_MAINS);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_FAULTED);
    while (!ace2k_heat_mains_recovered(&r.heat)) {
        rig_tick(&r);
    }
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
}

/* Round 3, item 4: a dip still open when the cycle ends without a fault — the time up, or a
 * stop — was ridden out: counted, the notice raised. */
TEST(a_dip_open_at_the_time_up_or_a_stop_is_counted)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 1U), ACE2K_DRYER_OK);
    while ((int32_t)(r.dryer.end_ms - r.now_ms) > 500) {
        rig_tick(&r);
    }
    r.mains_plausible = false;
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(r.dryer.mains_dips, 1U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, ACE2K_DRYER_NOTICE_MAINS_DIP);

    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    r.mains_plausible = false;
    rig_run_ms(&r, 200U);
    ASSERT_EQ(r.dryer.mains_dips, 0U);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&r);
    ASSERT_EQ(r.dryer.mains_dips, 1U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_MAINS_DIP, ACE2K_DRYER_NOTICE_MAINS_DIP);
}

/* Round 3, item 1: heat's edges judged out of phase this cycle raise the notice from
 * ACE2K_DRYER_PHASE_REJECTS_NOTICE on; those before the start do not count. */
TEST(edges_off_phase_raise_the_notice_from_the_threshold)
{
    rig_init(&r, 25000);
    r.heat.phase_rejects += 50U; /* before the cycle: not this cycle's */
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.heat.phase_rejects += ACE2K_DRYER_PHASE_REJECTS_NOTICE - 1U;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE, 0U);
    r.heat.phase_rejects += 1U;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE,
              ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING); /* a notice, not a fault */
}

/* Round 4, item 4: a second with no window measured (a boot) leaves heat's bucket empty: a start
 * 1.2 s in, on a clean line, is accepted and heats. */
TEST(a_start_just_after_boot_on_a_clean_line_is_accepted)
{
    rig_init(&r, 25000);
    r.mains_unmeasured = true;
    r.mains_plausible = false; /* the binding reads no frequency as not plausible */
    rig_run_ms(&r, 1000U);
    r.mains_unmeasured = false;
    r.mains_plausible = true;
    rig_run_ms(&r, 200U);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS + 100U));
    rig_run_ms(&r, 500U);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
}

/* Residual 1: a mains off-band often enough to hold heat's bucket above its restart level, but
 * never full — a lease from IDLE refused as transient forever — faults MAINS once the dryer has
 * wanted heat with no lease for ACE2K_DRYER_NO_LEASE_MAX_MS. */
TEST(a_bucket_hovering_above_the_restart_level_faults_mains_at_the_bound)
{
    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    r.mains_plausible = false;
    rig_run_ms(&r, 50U * RIG_TICK_MS); /* leased: rides it out, the bucket at 100 */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    r.mains_plausible = true;
    ace2k_heat_release(&r.heat); /* the lease gone: the next one comes from IDLE */
    uint32_t since = r.now_ms;
    uint32_t fired = r.heat.fired;
    uint32_t t = 0U;
    while (r.dryer.state == ACE2K_DRYER_HEATING && t < 30000U) {
        r.mains_plausible = (t % 1500U) >= 500U; /* 0.5 s off-band in 1.5 s: 100..200 */
        rig_tick(&r);
        t += RIG_TICK_MS;
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED); /* never leased: the dryer's bound */
    ASSERT_EQ(r.heat.fired, fired);
    uint32_t took = r.now_ms - since;
    ASSERT_TRUE(took >= ACE2K_DRYER_NO_LEASE_MAX_MS);
    ASSERT_TRUE(took <= ACE2K_DRYER_NO_LEASE_MAX_MS + 100U);
}

/* A duty of 0 (the NTCs at the drive target: no lease wanted) for longer than the bound is no
 * wait and no fault; neither is a dip ridden out with the lease waiting for the restart level. */
TEST(a_duty_of_zero_and_a_dip_ridden_out_never_hit_the_no_lease_bound)
{
    heating_with_ntc(70000, 70000); /* above the 62 °C drive: duty 0 */
    rig_run_ms(&r, ACE2K_DRYER_NO_LEASE_MAX_MS + 5000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));

    heating_with_ntc(50000, 50000);
    rig_run_ms(&r, 1000U);
    ace2k_heat_release(&r.heat);
    r.mains_plausible = false;
    rig_run_ms(&r, 101U * RIG_TICK_MS);
    r.mains_plausible = true;
    rig_run_ms(&r, ACE2K_DRYER_NO_LEASE_MAX_MS + 1000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
}

/* A 30 °C room and a 15 °C target: the old ceiling (15 + 10) latched chamber_over with nothing
 * heating.  Now the cycle runs, says so, and fires nothing. */
TEST(a_target_below_the_room_runs_with_the_notice_and_fires_nothing)
{
    rig_init(&r, 30000);
    uint32_t fired = r.heat.fired;
    ASSERT_EQ(rig_start(&r, 15U, 60U), ACE2K_DRYER_OK);
    ASSERT_TRUE((r.dryer.notices & ACE2K_DRYER_NOTICE_AMBIENT_ABOVE) != 0U);
    rig_run_ms(&r, 20U * RIG_MIN_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    ASSERT_EQ(r.heat.fired, fired);
}

TEST(the_ceiling_counts_from_the_start_chamber_when_it_is_above_the_target)
{
    rig_init(&r, 30000);
    ASSERT_EQ(rig_start(&r, 15U, 60U), ACE2K_DRYER_OK);
    int32_t start = r.dryer.start_chamber_mc;
    r.ovr_chamber = true;
    r.ovr_chamber_mc = start + 9500;
    rig_run_ms(&r, 40000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    r.ovr_chamber_mc = start + 10500;
    rig_run_ms(&r, 31000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
}

/* A warm restart: the chamber still 44 °C from the last cycle at the start, a 35 °C target.  The
 * ceiling follows the chamber's lowest reading since the start, so once it fell to 36 °C a chamber
 * pushed to 47 °C for 30 s is over (36 + 10), not under the start's 54 °C. */
TEST(a_warm_restart_ceiling_follows_the_chamber_down)
{
    rig_init(&r, 25000);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 44000;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(rig_start(&r, 35U, 60U), ACE2K_DRYER_OK);
    ASSERT_TRUE((r.dryer.notices & ACE2K_DRYER_NOTICE_AMBIENT_ABOVE) != 0U);
    for (int32_t mc = 44000; mc >= 36000; mc -= 500) {
        r.ovr_chamber_mc = mc;
        rig_run_ms(&r, 10000U);
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    r.ovr_chamber_mc = 47000;
    rig_run_ms(&r, 31000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
}

/* The same warm restart, the fault's clear: it reads the same running-minimum ceiling (36 + 10),
 * not the start's — refused at the ceiling, accepted just below it. */
TEST(a_warm_restart_clear_reads_the_running_minimum_ceiling)
{
    rig_init(&r, 25000);
    r.ovr_chamber = true;
    r.ovr_chamber_mc = 44000;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(rig_start(&r, 35U, 60U), ACE2K_DRYER_OK);
    for (int32_t mc = 44000; mc >= 36000; mc -= 500) {
        r.ovr_chamber_mc = mc;
        rig_run_ms(&r, 10000U);
    }
    r.ovr_chamber_mc = 47000;
    rig_run_ms(&r, 31000U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_OVER);
    ASSERT_EQ(ace2k_dryer_chamber_ceiling_mc(&r.dryer), 46000);
    for (uint32_t t = 0; t < 60U * RIG_MIN_MS && r.dryer.cool != ACE2K_DRYER_COOL_RELEASED;
         t += RIG_TICK_MS) {
        rig_tick(&r);
    }
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    r.ovr_chamber_mc = 46000; /* at the ceiling: still over */
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_NOT_COOL);
    r.ovr_chamber_mc = 45999; /* just below the ceiling, still far above the start's 44 + 10 */
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
}

TEST(a_chamber_below_its_target_raises_no_ambient_notice)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.notices & ACE2K_DRYER_NOTICE_AMBIENT_ABOVE, 0U);
}

TEST(a_start_after_an_outage_waits_for_a_measured_window)
{
    rig_init(&r, 25000);
    r.mains_present = false;
    rig_run_ms(&r, 300U);
    r.mains_present = true;
    r.mains_unmeasured = true; /* back, the first window not yet published */
    r.mains_plausible = false;
    rig_run_ms(&r, 200U);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_MEASURING);
    r.mains_unmeasured = false;
    r.mains_plausible = true;
    rig_run_ms(&r, 200U);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
}

TEST(a_mains_fault_clears_only_once_the_mains_is_measured_again)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.mains_present = false;
    rig_run_ms(&r, 100U);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_MAINS);
    r.mains_present = true;
    r.mains_unmeasured = true;
    r.mains_plausible = false;
    rig_run_ms(&r, 11U * RIG_MIN_MS); /* cooled, the flaps closed */
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_MEASURING);
    r.mains_unmeasured = false;
    r.mains_plausible = true;
    rig_run_ms(&r, 5000U); /* heat's bucket drains below its restart level */
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
