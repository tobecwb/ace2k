/* The dryer's states, controller, vent and cool-down on the thermal simulator.
 * The rig runs the real airflow and heat cores; every test starts
 * from a powered, cold unit. */
#include <limits.h>

#include "dryer/dryer_internal.h"
#include "fake_dryer_rig.h"
#include "test.h"

// NOLINTNEXTLINE(readability-identifier-naming)
static struct rig r; /* ~10 KB: kept off the stack */

/* The simulator's chamber loss to the room is assumed, not fitted (fake_thermal.h), so these
 * cycles assert what the model can prove: the invariants of every cycle, and the chamber's
 * accuracy only where the model makes it feasible.  The chamber's real accuracy and the NTC's real
 * overshoot are settled on the bench. */
#define CYCLE_NTC_BAND_MC      2000 /* the measured NTC's overshoot above the drive cap */
#define CYCLE_OVERSHOOT_MC     2500 /* the measured NTC above the cycle's own highest drive */
#define CYCLE_SETTLED_MC       500  /* the chamber's range (hi − lo) over the last 10 min */
#define CYCLE_CHAMBER_BAND_MC  2000 /* ± the target, where feasible */
#define CYCLE_HOT_AMBIENT_OVER 5000 /* a hot room: the chamber at most this far above the target */

// NOLINTNEXTLINE(readability-identifier-naming)
struct cycle_seen {
    int32_t lo, hi; /* the chamber over the last 10 min */
};

/* A 90-min cycle; the invariants every cycle keeps: no fault, the drive target never above the
 * absolute 73 °C, the measured NTC within the overshoot band (of the cap and of the cycle's own
 * highest drive) and below the heat layer's limit,
 * the duty never above 90 %, the chamber settled over the last 10 min. */
static void run_cycle(uint8_t target_c, int32_t ambient_mc, struct cycle_seen *seen)
{
    rig_init(&r, ambient_mc);
    ASSERT_EQ(rig_start(&r, target_c, 120U), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    rig_run_ms(&r, 80U * RIG_MIN_MS);
    seen->lo = INT32_MAX;
    seen->hi = INT32_MIN;
    for (uint32_t t = 0; t < 10U * RIG_MIN_MS; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (r.last_in.chamber_mc < seen->lo) {
            seen->lo = r.last_in.chamber_mc;
        }
        if (r.last_in.chamber_mc > seen->hi) {
            seen->hi = r.last_in.chamber_mc;
        }
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 0U);
    ASSERT_TRUE(r.max_drive_mc <= ACE2K_DRYER_DRIVE_ABS_MAX_MC);
    ASSERT_TRUE(r.max_ntc_mc <= ACE2K_DRYER_DRIVE_ABS_MAX_MC + CYCLE_NTC_BAND_MC);
    ASSERT_TRUE(r.max_ntc_mc <= r.max_drive_mc + CYCLE_OVERSHOOT_MC);
    ASSERT_TRUE(r.max_ntc_mc < ACE2K_HEAT_NTC_MAX_MC);
    ASSERT_TRUE(r.max_duty_pct <= ACE2K_HEAT_DUTY_MAX_PCT);
    ASSERT_TRUE(seen->hi - seen->lo < CYCLE_SETTLED_MC);
}

static void assert_chamber_on_target(const struct cycle_seen *seen, uint8_t target_c)
{
    int32_t target_mc = (int32_t)target_c * 1000;
    ASSERT_TRUE(seen->lo >= target_mc - CYCLE_CHAMBER_BAND_MC);
    ASSERT_TRUE(seen->hi <= target_mc + CYCLE_CHAMBER_BAND_MC);
}

TEST(a_45_c_cycle_at_15_c_holds_the_chamber_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(45U, 15000, &seen);
    assert_chamber_on_target(&seen, 45U);
}

TEST(a_55_c_cycle_at_15_c_holds_the_chamber_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(55U, 15000, &seen);
    assert_chamber_on_target(&seen, 55U);
}

/* The drive is capped at 73 °C (rule 8), and on the model's assumed chamber loss that cap holds
 * the chamber short of 65 °C: it settles at ≈ 61.7 °C.  The bench decides the real value. */
TEST(a_65_c_cycle_at_15_c_saturates_the_drive_at_the_cap_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(65U, 15000, &seen);
    ASSERT_EQ(ace2k_dryer_drive_mc(&r.dryer), ACE2K_DRYER_DRIVE_ABS_MAX_MC);
    ASSERT_EQ(r.max_drive_mc, ACE2K_DRYER_DRIVE_ABS_MAX_MC);
}

/* A 42 °C room carries the chamber above a 45 °C target (≈ 48.2 °C on the model) with the trim
 * pinned at its floor: accepted, no adaptive offset. */
TEST(a_45_c_cycle_at_42_c_stays_within_5_c_above_the_target_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(45U, 42000, &seen);
    ASSERT_TRUE(seen.hi <= 45000 + CYCLE_HOT_AMBIENT_OVER);
    ASSERT_TRUE(r.dryer.ctl.trim_mc >= -ACE2K_DRYER_TRIM_MAX_MC);
}

TEST(a_55_c_cycle_at_42_c_settles_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(55U, 42000, &seen);
}

TEST(a_65_c_cycle_at_42_c_settles_with_no_fault)
{
    struct cycle_seen seen;
    run_cycle(65U, 42000, &seen);
}

TEST(start_refuses_what_it_must_and_accepts_24_hours)
{
    rig_init(&r, 25000);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 14U, 60U, r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 66U, 60U, r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 0U, r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 1441U, r.now_ms), ACE2K_DRYER_RANGE);
    r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_SENSORS);
    r.chamber_age_ms = 0U;
    r.mains_plausible = false; /* lasting: heat's bucket holds it (a dip alone is accepted) */
    rig_run_ms(&r, ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * RIG_TICK_MS);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_NO_MAINS);
    r.mains_plausible = true;
    rig_tick(&r);
    /* plausible again, the bucket still above its restart level: refused until it drains */
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_NO_MAINS);
    while (!ace2k_heat_mains_recovered(&r.heat)) {
        rig_tick(&r);
    }
    r.log.flags |= ACE2K_DRYER_LOG_CUTOUT;
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_CUTOUT);
    r.log.flags &= ~ACE2K_DRYER_LOG_CUTOUT;
    uint32_t now = r.now_ms;
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 1440U, now), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.end_ms - now, 1440U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_BUSY);
}

TEST(a_start_from_fault_is_refused_as_faulted)
{
    rig_init(&r, 25000);
    r.cutout = true;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_FAULTED);
}

TEST(starting_closes_both_flaps_takes_the_fans_and_heats_only_with_both_reading_high)
{
    rig_init(&r, 25000);
    uint32_t pulses = r.flap_pulses;
    uint32_t closes_bottom = r.flap_closes[ACE2K_FLAP_BOTTOM];
    uint32_t closes_rear = r.flap_closes[ACE2K_FLAP_REAR];
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_STARTING);
    rig_tick(&r);
    ASSERT_EQ(r.flap_pulses - pulses, 1U); /* the bottom first; the rear after its pulse */
    ASSERT_TRUE(r.airflow.flap_wait[ACE2K_FLAP_REAR].set);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN);
    /* no lease, still STARTING, on every tick until the rear's pulse is over */
    uint32_t waited = 0U;
    while (r.flap_pulses - pulses < 2U || r.flap_coil_on[ACE2K_FLAP_REAR]) {
        ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_STARTING);
        ASSERT_TRUE(waited < RIG_FLAP_PAIR_MS);
        if (waited >= RIG_FLAP_PAIR_MS) {
            break;
        }
        rig_tick(&r);
        waited += RIG_TICK_MS;
    }
    ASSERT_TRUE(waited >= (2U * ACE2K_FLAP_PULSE_MS) - RIG_TICK_MS); /* both pulses, in turn */
    ASSERT_EQ(r.flap_closes[ACE2K_FLAP_BOTTOM] - closes_bottom, 1U);
    ASSERT_EQ(r.flap_closes[ACE2K_FLAP_REAR] - closes_rear, 1U);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS));
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_TRUE(r.fans_on);
    ASSERT_EQ(r.log.cycles_started, 1U);
    ASSERT_TRUE((r.log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN) != 0U);
    ASSERT_TRUE(r.stores >= 1U); /* the open mark reached "flash" before the first lease */
    ASSERT_TRUE(!r.coil_both);
}

TEST(fans_that_never_read_high_fault_the_start_without_a_half_cycle)
{
    rig_init(&r, 25000);
    r.fan_fail = true;
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    rig_run_ms(&r, ACE2K_DRYER_START_TIMEOUT_MS - RIG_TICK_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_STARTING); /* 2 990 ms: still waiting for the fans */
    rig_run_ms(&r, 100U + RIG_TICK_MS);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
    ASSERT_EQ(r.heat.fired, 0U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_FAULT), 1U);
}

TEST(the_lease_never_runs_more_than_two_seconds_ahead_and_a_frozen_dryer_stops_the_gate)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    for (uint32_t t = 0; t < 5U * RIG_MIN_MS; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (ace2k_heat_leased(&r.heat)) {
            ASSERT_TRUE(r.heat.lease_end_ms - r.now_ms <= ACE2K_HEAT_LEASE_MAX_MS);
        }
    }
    r.dryer_frozen = true;
    rig_run_ms(&r, ACE2K_HEAT_LEASE_MAX_MS + 20U);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    uint32_t fired = r.heat.fired;
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.heat.fired, fired);
}

/* What a cool-down showed: its path and whether an invariant broke. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct cool_seen {
    bool idle;        /* IDLE reached */
    bool handed_over; /* by the 10-min hand-over to rule 7 (the hot_ambient notice) */
    uint32_t idle_ms; /* from the entry into COOLDOWN to IDLE */
    /* a half-cycle fired after the entry into COOLDOWN, beyond the one the release allows: the
     * odd half of a cycle already begun, on the first edge (heat.h) */
    bool gate_fired;
    bool fans_off_hot;  /* a fan off while an NTC read above 45 °C */
    bool fans_off_warm; /* after IDLE, a fan off while an NTC read above 42 °C */
    uint32_t pulses;    /* flap pulses from the entry into COOLDOWN */
    /* each flap's opening and closing pulses from the entry, the counts at the entry kept */
    uint32_t opens[ACE2K_FLAP_COUNT], closes[ACE2K_FLAP_COUNT];
    uint32_t open_base[ACE2K_FLAP_COUNT], close_base[ACE2K_FLAP_COUNT];
    bool opened;        /* both flaps' opening pulses started */
    uint32_t opened_ms; /* from the entry into COOLDOWN to the second one */
    bool close_first;   /* a closing pulse started before both opening ones */
    bool pending_fired; /* the release's pending odd half fired on the first tick */
    /* at the entry: the even half of a cycle had fired, its odd half was due, on an ok verdict */
    bool half_due;
    uint32_t even_us; /* that even half's edge */
};

/* The even half of a cycle has fired and its odd edge has not come yet. */
static bool even_half_fired(void)
{
    if (r.heat.edge_parity == 0U) {
        return false;
    }
    return r.heat.fire_this_cycle;
}

/* The one fire the release allows: on the first tick, the odd half of the cycle whose even half
 * had fired at the entry, on the next half-cycle, the verdict ok (heat.h). */
static bool pending_half_legal(const struct cool_seen *seen, bool first)
{
    if (!first || !seen->half_due) {
        return false;
    }
    uint32_t gap = r.gate_on_us - seen->even_us;
    if (gap < RIG_HALF_CYCLE_US - ACE2K_HEAT_EDGE_TOL_US) {
        return false;
    }
    return gap <= RIG_HALF_CYCLE_US + ACE2K_HEAT_EDGE_TOL_US;
}

/* True when either NTC of the last tick reads above limit_mc. */
static bool an_ntc_above(int32_t limit_mc)
{
    if (r.last_in.ntc_left_mc > limit_mc) {
        return true;
    }
    return r.last_in.ntc_right_mc > limit_mc;
}

/* The flaps through a watched cool-down: both opened at its entry, closed only after. */
static void watch_flaps(struct cool_seen *seen, uint32_t entry)
{
    bool both_open = true;
    bool a_close = false;
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        seen->opens[f] = r.flap_opens[f] - seen->open_base[f];
        seen->closes[f] = r.flap_closes[f] - seen->close_base[f];
        if (seen->opens[f] == 0U) {
            both_open = false;
        }
        if (seen->closes[f] != 0U) {
            a_close = true;
        }
    }
    if (a_close && !both_open) {
        seen->close_first = true;
    }
    if (both_open && !seen->opened) {
        seen->opened = true;
        seen->opened_ms = r.now_ms - entry;
    }
}

/* One tick of a watched cool-down: the invariants, and the moment IDLE is reached. */
static void watch_tick(struct cool_seen *seen, uint32_t fired, uint32_t pulses, uint32_t entry)
{
    bool first = r.now_ms == entry;
    rig_tick(&r);
    uint32_t n = r.heat.fired - fired;
    if (n > 1U || (n == 1U && !seen->pending_fired && !pending_half_legal(seen, first))) {
        seen->gate_fired = true;
    }
    if (first && n == 1U) {
        seen->pending_fired = true;
    }
    if (an_ntc_above(ACE2K_AIRFLOW_FAN_ON_MC) && !r.fans_on) {
        seen->fans_off_hot = true;
    }
    if (!seen->idle) {
        watch_flaps(seen, entry);
    }
    if (!seen->idle && r.dryer.state == ACE2K_DRYER_IDLE) {
        seen->idle = true;
        seen->idle_ms = r.now_ms - entry;
        seen->handed_over = ((unsigned)r.dryer.notices & ACE2K_DRYER_NOTICE_HOT_AMBIENT) != 0U;
        seen->pulses = r.flap_pulses - pulses;
    }
    if (seen->idle && an_ntc_above(ACE2K_AIRFLOW_FAN_OFF_MC) && !r.fans_on) {
        seen->fans_off_warm = true;
    }
}

/* Called on the tick the dryer entered COOLDOWN: runs until IDLE (at most max_ms), then after_ms
 * more, watching the invariants on every tick.  The path is recorded, never required: both the
 * cool rule and the 10-min hand-over to rule 7 are legal ends. */
static void watch_cooldown(uint32_t max_ms, uint32_t after_ms, struct cool_seen *seen)
{
    *seen = (struct cool_seen){ .idle = false };
    /* a release keeps its request only when the even half fired (heat.h) */
    ASSERT_TRUE((r.heat.fire_mode != ACE2K_HEAT_FIRE_PENDING) || even_half_fired());
    /* PENDING holds only on an ok verdict: a verdict not ok turns it to NONE at once */
    seen->half_due = r.heat.fire_mode == ACE2K_HEAT_FIRE_PENDING;
    seen->even_us = r.heat.even_us;
    /* entered by a command, or by the last tick (whose flap pulses belong to the entry) */
    if (!r.entered_by_tick) {
        rig_mark_entry(&r);
    }
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        seen->open_base[f] = r.entry_opens[f];
        seen->close_base[f] = r.entry_closes[f];
    }
    uint32_t fired = r.heat.fired;
    uint32_t pulses = r.entry_pulses;
    uint32_t entry = r.now_ms;
    for (uint32_t t = 0; t < max_ms && !seen->idle; t += RIG_TICK_MS) {
        watch_tick(seen, fired, pulses, entry);
    }
    for (uint32_t t = 0; seen->idle && t < after_ms; t += RIG_TICK_MS) {
        watch_tick(seen, fired, pulses, entry);
    }
}

/* Every cool-down's invariants and end state: IDLE by either legal path, never a half-cycle,
 * the fans on while hot and, once handed to rule 7, until both NTCs read 42 °C or less, the
 * flaps pulsed open once each at the entry and closed once each at the end (never both coils),
 * the notice and its event only on the hand-over.  The simulator's plant has no flap term: the
 * open flaps change nothing in these runs; their thermal gain is measured on the bench. */
static void assert_cooldown(const struct cool_seen *seen)
{
    ASSERT_TRUE(seen->idle);
    ASSERT_TRUE(seen->idle_ms <= ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_TRUE(!seen->gate_fired);
    ASSERT_TRUE(!seen->fans_off_hot);
    ASSERT_TRUE(!seen->fans_off_warm);
    ASSERT_EQ(seen->pulses, 4U);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(seen->opens[f], 1U);
        ASSERT_EQ(seen->closes[f], 1U);
    }
    ASSERT_TRUE(seen->opened);
    ASSERT_TRUE(seen->opened_ms <= RIG_FLAP_PAIR_MS); /* the rear after the bottom's pulse */
    ASSERT_TRUE(!seen->close_first);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_TRUE(r.airflow.owner != ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_HOT_AMBIENT), seen->handed_over ? 1U : 0U);
    ASSERT_TRUE(!r.coil_both);
    ASSERT_TRUE(!r.flap_overlap); /* the two flaps never pulsed together, from the boot on */
    ASSERT_TRUE(!r.flap_claimed); /* no position published while a flap travelled */
}

/* On the model's assumed chamber loss this cool-down ends by the 10-min hand-over (an NTC still
 * ≈ 45.4 °C); rule 7 then keeps the fans until both read 42 °C or less, and they stop within
 * the 30 min watched after IDLE. */
TEST(the_duration_ends_in_a_cool_down_that_opens_then_closes_the_flaps_and_hands_the_fans_back)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 30U), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_COOLDOWN, 31U * RIG_MIN_MS));
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_DONE), 1U);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_TRUE(r.fans_on);
    struct cool_seen seen;
    watch_cooldown(11U * RIG_MIN_MS, 30U * RIG_MIN_MS, &seen);
    assert_cooldown(&seen);
    ASSERT_EQ(r.log.cycles_completed, 1U);
    ASSERT_TRUE((r.log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN) == 0U);
    ASSERT_TRUE(!r.fans_on);
}

/* Stopped after 10 min, the model's NTCs reach 45 °C in ≈ 8 min: this one ends by the cool rule. */
TEST(a_stop_releases_the_lease_at_once_and_cools_down_to_idle)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_STOPPED), 0U); /* not drained yet */
    struct cool_seen seen;
    watch_cooldown(11U * RIG_MIN_MS, 0U, &seen);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_STOPPED), 1U);
    assert_cooldown(&seen);
    ASSERT_EQ(r.log.cycles_completed, 0U);
}

/* On the model's assumed chamber loss the chamber is still ≈ 47.5 °C at 10 min, so this
 * cool-down ends by the 10-min hand-over to rule 7 (with the notice); on a leakier chamber it
 * ends by the cool rule (the synthetic test below).  Both are legal; the bench decides. */
TEST(a_42_c_ambient_cool_down_reaches_idle_within_ten_minutes)
{
    rig_init(&r, 42000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 20U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    struct cool_seen seen;
    watch_cooldown(ACE2K_DRYER_HOT_AMBIENT_MS + 1000U, 0U, &seen);
    assert_cooldown(&seen);
}

/* A 47 °C room: no NTC can fall to 45 °C, so the hand-over is the only way out. */
TEST(a_47_c_ambient_cool_down_ends_at_ten_minutes_with_the_notice_and_the_fans_left_on)
{
    rig_init(&r, 47000);
    ASSERT_EQ(rig_start(&r, 50U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    struct cool_seen seen;
    watch_cooldown(ACE2K_DRYER_HOT_AMBIENT_MS + 1000U, 0U, &seen);
    assert_cooldown(&seen);
    ASSERT_TRUE(seen.handed_over);
    ASSERT_TRUE(((unsigned)r.dryer.notices & ACE2K_DRYER_NOTICE_HOT_AMBIENT) != 0U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_HOT_AMBIENT), 1U);
    ASSERT_TRUE(r.fans_on); /* rule 7: an NTC above 45 °C */
}

/* SYNTHETIC plant: the chamber's loss to the room multiplied by 5 — not a measured unit, only a
 * chamber that sheds heat fast enough for the NTCs to reach 45 °C inside 10 min.  It proves the
 * normal exit: the cool rule ends the cool-down, with no notice. */
#define SYNTHETIC_CHAMBER_M_FACTOR 5.0
TEST(a_cool_down_on_a_synthetic_leaky_chamber_ends_by_the_cool_rule)
{
    rig_init(&r, 25000);
    r.plant.chamber_m *= SYNTHETIC_CHAMBER_M_FACTOR;
    ASSERT_EQ(rig_start(&r, 55U, 30U), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_COOLDOWN, 31U * RIG_MIN_MS));
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_DONE), 1U);
    struct cool_seen seen;
    watch_cooldown(11U * RIG_MIN_MS, 0U, &seen);
    assert_cooldown(&seen);
    ASSERT_TRUE(!seen.handed_over);
    ASSERT_TRUE(seen.idle_ms < ACE2K_DRYER_HOT_AMBIENT_MS);
    ASSERT_EQ((unsigned)r.dryer.notices & ACE2K_DRYER_NOTICE_HOT_AMBIENT, 0U);
    ASSERT_TRUE(r.last_in.ntc_left_mc <= ACE2K_DRYER_COOL_MC);
    ASSERT_TRUE(r.last_in.ntc_right_mc <= ACE2K_DRYER_COOL_MC);
    ASSERT_EQ(r.log.cycles_completed, 1U);
}

/* The vent opens once the chamber is near its target or stopped rising, after 5 min of HEATING,
 * and the flaps stay open. */
TEST(the_vent_fires_once_after_five_minutes_and_leaves_both_flaps_open)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 180U), ACE2K_DRYER_OK);
    uint32_t heating_at = r.now_ms;
    uint32_t vented_at = 0U;
    for (uint32_t t = 0; t < 60U * RIG_MIN_MS && vented_at == 0U; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (rig_events(&r, ACE2K_DRYER_EV_VENTED) != 0U) {
            vented_at = r.now_ms;
        }
    }
    ASSERT_TRUE(vented_at != 0U);
    ASSERT_TRUE(vented_at - heating_at >= ACE2K_DRYER_VENT_WINDOW_MS);
    rig_run_ms(&r, 90U * RIG_MIN_MS);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_VENTED), 1U);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_NONE);
    ASSERT_TRUE(!r.coil_both);
    ASSERT_TRUE(!r.flap_overlap);
}

/* Ticks until the bottom flap's count of pulses one way moves, at most max_ms: the tick it moved
 * at, or 0. */
static uint32_t run_until_pulse(const uint32_t *count, uint32_t max_ms)
{
    uint32_t before = *count;
    for (uint32_t t = 0; t < max_ms; t += RIG_TICK_MS) {
        rig_tick(&r);
        if (*count != before) {
            return r.now_ms;
        }
    }
    return 0U;
}

/* The room at 25 °C / 30 % (6.90 g/m³).  Vented, the chamber (~55 °C) with its RH dropped to
 * 0.5 % reads ~0.52 g/m³, drier than the room: both flaps close, without an event; humid again
 * (30 %, ~31 g/m³), they reopen, no sooner than 10 min after the close; dry again at once, they
 * close again no sooner than 10 min after the reopen. */
TEST(the_humidity_guard_closes_and_reopens_the_flaps_without_an_event)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 180U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 60U * RIG_MIN_MS);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_VENTED), 1U);
    r.rh_pct10 = 5U;
    uint32_t closed = run_until_pulse(&r.flap_closes[ACE2K_FLAP_BOTTOM], 2U * RIG_MIN_MS);
    ASSERT_TRUE(closed != 0U);
    rig_run_ms(&r, RIG_MIN_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    r.rh_pct10 = 300U;
    uint32_t reopened = run_until_pulse(&r.flap_opens[ACE2K_FLAP_BOTTOM], 15U * RIG_MIN_MS);
    ASSERT_TRUE(reopened != 0U);
    ASSERT_TRUE(reopened - closed >= ACE2K_DRYER_VENT_GUARD_MIN_MS);
    r.rh_pct10 = 5U; /* drier at once: the guard wants them closed again */
    uint32_t closes = r.flap_closes[ACE2K_FLAP_BOTTOM];
    rig_run_ms(&r, ACE2K_DRYER_VENT_GUARD_MIN_MS - (2U * RIG_TICK_MS));
    ASSERT_EQ(r.flap_closes[ACE2K_FLAP_BOTTOM], closes); /* no move within 10 min */
    uint32_t again = run_until_pulse(&r.flap_closes[ACE2K_FLAP_BOTTOM], 2U * RIG_MIN_MS);
    ASSERT_TRUE(again != 0U);
    ASSERT_TRUE(again - reopened >= ACE2K_DRYER_VENT_GUARD_MIN_MS);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_VENTED), 1U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
}

/* The flaps hold their position unpowered, so a cool-down cut short by a reset (a Klipper
 * shutdown, FIRMWARE_RESTART, the watchdog, a power loss) can leave them open: every boot
 * closes both, once, the rear after the bottom's pulse, with no owner taken — and so does the
 * boot that logs an interrupted cycle. */
TEST(the_boot_closes_both_flaps_once_one_after_the_other)
{
    rig_init(&r, 25000); /* one second of ticks behind it */
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_closes[f], 1U);
        ASSERT_EQ(r.flap_opens[f], 0U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.coil_both);
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    rig_run_ms(&r, 10000U);
    ASSERT_EQ(r.flap_pulses, 2U); /* once */
    /* a reset in the middle of a cycle: the flaps left open, the cycle logged interrupted */
    ASSERT_EQ(rig_start(&r, 55U, 180U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 150U * RIG_MIN_MS); /* past the vent: both open, the cycle still heating */
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);
    uint32_t closes_bottom = r.flap_closes[ACE2K_FLAP_BOTTOM];
    uint32_t closes_rear = r.flap_closes[ACE2K_FLAP_REAR];
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, false, r.now_ms);
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_INTERRUPTED), 1U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(r.flap_closes[ACE2K_FLAP_BOTTOM] - closes_bottom, 1U);
    ASSERT_EQ(r.flap_closes[ACE2K_FLAP_REAR] - closes_rear, 1U);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_TRUE(!r.flap_overlap);
}

/* A boot with a persisted cutout (FAULT held until a power-on reset): the boot closes both flaps,
 * and the fault's own close still runs at its end — it never leans on the boot's. */
TEST(a_boot_with_a_persisted_cutout_closes_at_boot_and_again_at_the_faults_end)
{
    rig_init(&r, 25000);
    r.log.flags |= ACE2K_DRYER_LOG_CUTOUT;
    uint32_t closes[ACE2K_FLAP_COUNT] = { r.flap_closes[0], r.flap_closes[1] };
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, false, r.now_ms);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    rig_run_ms(&r, 1000U);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_closes[f] - closes[f], 1U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, (enum ace2k_flap)f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_closes[f] - closes[f], 2U);
        ASSERT_EQ(r.flap_opens[f], 0U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, (enum ace2k_flap)f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
}

/* A fault on the first tick, the boot's close still in flight: the fault's close still runs, after
 * the boot's has ended — and the two never overlap. */
TEST(a_fault_during_the_boot_close_still_closes_the_flaps_at_its_end)
{
    rig_init(&r, 25000);
    uint32_t closes[ACE2K_FLAP_COUNT] = { r.flap_closes[0], r.flap_closes[1] };
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, true, r.now_ms);
    r.cutout = true;
    rig_tick(&r); /* the fault, and the boot close's first pulse */
    r.cutout = false;
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_TRUE(r.flap_coil_on[ACE2K_FLAP_BOTTOM]);
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_closes[f] - closes[f], 2U); /* the boot's, then the fault's */
        ASSERT_EQ(r.flap_opens[f], 0U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, (enum ace2k_flap)f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
}

/* A manual pulse during the boot's close: one asked before the dryer runs first, and the dryer's
 * waits behind it; one asked while the dryer's close runs is refused busy — never queued, so it
 * neither runs later nor holds the dryer, and the sequence never times out. */
TEST(a_manual_pulse_during_the_boot_close_neither_overlaps_nor_holds_the_dryer)
{
    rig_init(&r, 25000);
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, true, r.now_ms);
    uint32_t closes[ACE2K_FLAP_COUNT] = { r.flap_closes[0], r.flap_closes[1] };
    ASSERT_EQ(ace2k_airflow_flap_pulse(&r.airflow, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM,
                                       true, r.now_ms),
              0); /* running before the dryer asks */
    bool asked = false;
    for (uint32_t t = 0; t < ACE2K_DRYER_FLAPS_TIMEOUT_MS; t += RIG_TICK_MS) {
        rig_tick(&r);
        bool rear_pending =
            ace2k_airflow_flap_pending(&r.airflow, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER);
        if (!asked && !rear_pending &&
            ace2k_airflow_flap_pending(&r.airflow, ACE2K_FLAP_BOTTOM, ACE2K_AIRFLOW_OWNER_DRYER)) {
            /* the dryer's bottom close runs: a manual rear close is refused, not queued */
            ASSERT_EQ(ace2k_airflow_flap_pulse(&r.airflow, ACE2K_AIRFLOW_OWNER_MANUAL,
                                               ACE2K_FLAP_REAR, false, r.now_ms),
                      -ACE2K_EBUSY);
            asked = true;
        }
        if (r.dryer.flaps.result != ACE2K_DRYER_FLAPS_RUNNING) {
            break;
        }
    }
    ASSERT_TRUE(asked);
    ASSERT_EQ(r.dryer.flaps.result, ACE2K_DRYER_FLAPS_OVER);
    /* nothing manual waits: the refused request was never queued */
    ASSERT_TRUE(
        !ace2k_airflow_flap_pending(&r.airflow, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_MANUAL));
    rig_run_ms(&r, RIG_FLAP_PAIR_MS);
    ASSERT_EQ(r.flap_opens[ACE2K_FLAP_BOTTOM], 1U); /* the manual pulse, before the dryer's */
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_closes[f] - closes[f], 1U); /* the dryer's close only */
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, (enum ace2k_flap)f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
}

/* STARTING's timeout covers the flaps' close too: flaps that never come to rest (airflow not
 * ticked, so no pulse ever ends) fault the start FLAPS — not FANS, which read high — within
 * ACE2K_DRYER_START_TIMEOUT_MS. */
TEST(a_start_whose_flaps_never_come_to_rest_faults_within_the_timeout)
{
    rig_init(&r, 25000);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_airflow_fans_read(&r.airflow), ACE2K_DRYER_FANS_BOTH); /* only the flaps */
    struct ace2k_dryer_inputs in;
    rig_readings(&r, &in);
    uint32_t start = r.now_ms;
    while (r.dryer.state != ACE2K_DRYER_FAULT &&
           r.now_ms - start < 2U * ACE2K_DRYER_START_TIMEOUT_MS) {
        r.now_ms += RIG_TICK_MS;
        ace2k_dryer_tick(&r.dryer, &in, r.now_ms);
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FLAPS);
    ASSERT_TRUE(r.now_ms - start <= ACE2K_DRYER_START_TIMEOUT_MS + RIG_TICK_MS);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
}

/* An owner that is neither none, manual nor the dryer forced onto the outputs — a test-only
 * stand-in for a refusal no image can produce: may_drive() refuses the dryer while it holds, and
 * it holds until released. */
/* value 2 stands in for an owner that no longer exists, to keep the dryer's refusal handling
 * (kept as defence) covered */
#define TEST_OWNER_OTHER ((enum ace2k_airflow_owner)2)
static void hold_the_outputs(void)
{
    r.airflow.owner = TEST_OWNER_OTHER;
    r.airflow.owner_on = true;
}

static void release_the_outputs(void)
{
    r.airflow.owner = ACE2K_AIRFLOW_OWNER_NONE;
    r.airflow.owner_on = false;
}

/* A boot whose close is refused: asked again every tick, then over by the bound — both flaps
 * never pulsed, still unknown; the dryer idles. */
TEST(a_refused_boot_close_times_out_and_claims_no_position)
{
    rig_init(&r, 25000);
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    hold_the_outputs();
    uint32_t pulses = r.flap_pulses;
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, true, r.now_ms);
    rig_run_ms(&r, ACE2K_DRYER_FLAPS_TIMEOUT_MS - RIG_TICK_MS);
    ASSERT_EQ(r.dryer.flaps.result, ACE2K_DRYER_FLAPS_RUNNING);
    ASSERT_EQ(r.dryer.flaps.next, (uint8_t)ACE2K_FLAP_BOTTOM); /* refused: never counted asked */
    rig_run_ms(&r, 2U * RIG_TICK_MS);
    ASSERT_EQ(r.dryer.flaps.result, ACE2K_DRYER_FLAPS_TIMED_OUT);
    release_the_outputs();
    rig_run_ms(&r, 20U * ACE2K_DRYER_FLAPS_TIMEOUT_MS); /* past the owner's hold too */
    ASSERT_EQ(r.flap_pulses, pulses);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, (enum ace2k_flap)f), ACE2K_FLAP_UNKNOWN);
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
}

/* A cool-down whose flaps are refused, opening and closing: each sequence ends by its bound, the
 * cool-down still reaches IDLE, and the flaps keep the position their last pulse reached (closed
 * by the start) — never one they were not seen to reach. */
TEST(a_cool_down_with_refused_flaps_still_ends_and_claims_no_new_position)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    hold_the_outputs();
    uint32_t pulses = r.flap_pulses;
    rig_run_ms(&r, ACE2K_DRYER_FLAPS_TIMEOUT_MS + RIG_TICK_MS);
    ASSERT_EQ(r.dryer.flaps.result, ACE2K_DRYER_FLAPS_TIMED_OUT);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 11U * RIG_MIN_MS));
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    ASSERT_EQ(r.flap_pulses, pulses);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
}

/* A pulse that never ends (airflow not ticked) in a cool-down's opening: the sequence ends by its
 * bound, the cool-down goes on, and both flaps read unknown — the one travelling and the one
 * waiting alike. */
TEST(a_stalled_pulse_ends_the_sequence_by_its_bound_with_the_flaps_unknown)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    struct ace2k_dryer_inputs in;
    rig_readings(&r, &in);
    uint32_t start = r.now_ms;
    while (r.dryer.flaps.result == ACE2K_DRYER_FLAPS_RUNNING &&
           r.now_ms - start < 2U * ACE2K_DRYER_FLAPS_TIMEOUT_MS) {
        r.now_ms += RIG_TICK_MS;
        ace2k_dryer_tick(&r.dryer, &in, r.now_ms);
    }
    ASSERT_EQ(r.dryer.flaps.result, ACE2K_DRYER_FLAPS_TIMED_OUT);
    ASSERT_TRUE(r.now_ms - start <= ACE2K_DRYER_FLAPS_TIMEOUT_MS + RIG_TICK_MS);
    r.now_ms += RIG_TICK_MS;
    ace2k_dryer_tick(&r.dryer, &in, r.now_ms);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    ASSERT_TRUE(r.flap_coil_on[ACE2K_FLAP_BOTTOM]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN);
}

/* A shutdown in the middle of a flap's pulse: that flap is unknown and nothing moves any more;
 * the next boot closes both. */
TEST(a_shutdown_mid_pulse_leaves_the_flap_unknown_until_the_boot_closes_it)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5U * RIG_TICK_MS); /* the bottom's opening pulse runs, the rear's waits */
    ASSERT_TRUE(r.flap_coil_on[ACE2K_FLAP_BOTTOM]);
    ace2k_airflow_shutdown(&r.airflow);
    ace2k_dryer_shutdown(&r.dryer);
    uint32_t pulses = r.flap_pulses;
    rig_run_ms(&r, 10000U);
    ASSERT_EQ(r.flap_pulses, pulses);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, false, r.now_ms);
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_TRUE(!r.flap_overlap);
    ASSERT_TRUE(!r.flap_claimed);
}

/* STARTING's timeout with the fans not both high and the flaps not at rest: FANS goes first. */
TEST(a_start_timeout_with_the_fans_down_faults_fans_even_with_the_flaps_moving)
{
    rig_init(&r, 25000);
    r.fan_fail = true;
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    struct ace2k_dryer_inputs in;
    rig_readings(&r, &in);
    uint32_t start = r.now_ms;
    while (r.dryer.state != ACE2K_DRYER_FAULT &&
           r.now_ms - start < 2U * ACE2K_DRYER_START_TIMEOUT_MS) {
        r.now_ms += RIG_TICK_MS; /* airflow not ticked: the flaps never come to rest */
        ace2k_dryer_tick(&r.dryer, &in, r.now_ms);
    }
    ASSERT_TRUE(
        ace2k_airflow_flap_pending(&r.airflow, ACE2K_FLAP_BOTTOM, ACE2K_AIRFLOW_OWNER_DRYER));
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
}

/* A cool-down with no heating in it opens nothing — a fault at IDLE (a cutout glitch on a cold
 * unit) and a stop in STARTING alike: the flaps are only (re)closed, as the boot's cutout does. */
TEST(a_cool_down_without_heating_does_not_open_the_flaps)
{
    rig_init(&r, 25000);
    uint32_t pulses = r.flap_pulses;
    r.cutout = true;
    rig_tick(&r);
    r.cutout = false;
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    rig_run_ms(&r, ACE2K_DRYER_HOT_AMBIENT_MS + 1000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_RELEASED);
    ASSERT_EQ(r.flap_pulses - pulses, 2U);
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_opens[f], 0U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
    rig_init(&r, 25000);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&r);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 11U * RIG_MIN_MS));
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        ASSERT_EQ(r.flap_opens[f], 0U);
        ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, f), ACE2K_FLAP_CLOSED);
    }
    ASSERT_TRUE(!r.flap_overlap);
}

/* A stop after the vent: the flaps already open are pulsed open again at the cool-down's entry
 * (no position cache), then closed at its end. */
TEST(a_stop_after_the_vent_pulses_the_open_flaps_open_again_then_closes_them)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 180U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 150U * RIG_MIN_MS);
    ASSERT_EQ(rig_events(&r, ACE2K_DRYER_EV_VENTED), 1U);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&r.airflow, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    struct cool_seen seen;
    watch_cooldown(11U * RIG_MIN_MS, 0U, &seen);
    assert_cooldown(&seen);
}

TEST(the_controller_starts_from_zero_on_every_cycle)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 20U * RIG_MIN_MS);
    ASSERT_TRUE(r.dryer.ctl.integ_m != 0);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 11U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.ctl.integ_m, 0);
    ASSERT_EQ(r.dryer.ctl.trim_mc, 0);
    ASSERT_TRUE(!r.dryer.ctl.primed);
    ASSERT_EQ(r.dryer.ctl.duty_cpct, 0U);
}

TEST(a_reset_mid_cycle_leaves_the_dryer_idle_and_fires_nothing)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 10U * RIG_MIN_MS);
    /* the reset: every core re-initialised, the log (the flash page) kept */
    ace2k_airflow_init(&r.airflow, &rig_airflow_ops, &r);
    ace2k_heat_init(&r.heat, &rig_heat_ops, &r);
    ace2k_dryer_init(&r.dryer, &r.heat, &r.airflow, &r.log, false, r.now_ms);
    r.fired_mark = 0U;
    rig_run_ms(&r, 10000U);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_EQ(r.heat.fired, 0U);
}

TEST(a_shutdown_releases_the_lease_fires_nothing_after_and_refuses_a_start)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5U * RIG_MIN_MS);
    ace2k_dryer_shutdown(&r.dryer);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    uint32_t fired = r.heat.fired; /* no grace: nothing fires after the shutdown returns */
    rig_run_ms(&r, 5000U);
    ASSERT_EQ(r.heat.fired, fired);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_TRUE(r.fans_on); /* rule 7 holds them: the NTCs are warm */
    /* a clear refusal, and heat refuses every lease until a reset (no "fire" into the floated
     * pin after a clear_shutdown) */
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 55U, 60U, r.now_ms), ACE2K_DRYER_SHUTDOWN);
    ASSERT_EQ(ace2k_heat_lease(&r.heat, 20U, 1000U, r.now_ms), -ACE2K_EREFUSED);
}

/* The shutdown gives the dryer's claim on the fans up although heat held them on at the last
 * tick (heat_holds): from then on the fans follow rule 7 alone — on while an NTC is hot, off
 * once both read 42 °C or less — and a manual request is no longer refused as the dryer's. */
TEST(a_shutdown_mid_cycle_leaves_the_fans_to_rule_7_alone)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5U * RIG_MIN_MS);
    ASSERT_TRUE(r.airflow.heat_holds); /* the lease was held at the last tick */
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_DRYER);
    ace2k_dryer_shutdown(&r.dryer);
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_NONE);
    rig_tick(&r);
    ASSERT_TRUE(!r.airflow.heat_holds); /* heat shut down, the gate low: no hold from the tick */
    ASSERT_TRUE(r.fans_on);             /* rule 7: the NTCs are hot */
    rig_run_ms(&r, 60U * RIG_MIN_MS);
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(!an_ntc_above(ACE2K_AIRFLOW_FAN_OFF_MC));
    ASSERT_TRUE(!r.fans_on);
}

TEST(the_cutout_mid_cycle_faults_stops_the_gate_and_keeps_the_fans)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 5U * RIG_MIN_MS);
    r.cutout = true;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CUTOUT);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    ASSERT_TRUE(!r.gate);
    ASSERT_TRUE(r.fans_on);
    ASSERT_EQ(r.log.faults, 1U);
}

/* A gate stuck high is heat on, latched or not: heat latches gate_stuck, the dryer faults and
 * cools down, the flaps close and the dryer lets the fans go — and they stay commanded on, a
 * manual off refused, though both NTCs have long fallen below 42 °C.  The fault's log entry still
 * reaches the flash (a latched gate does not hold the store). */
TEST(a_gate_stuck_high_keeps_the_fans_on_after_the_dryer_lets_them_go)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    r.gate_stuck = true;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_FAULT, 1000U));
    ASSERT_EQ(r.heat.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(r.heat.reason, ACE2K_HEAT_GATE_STUCK);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_HEAT);
    rig_run_ms(&r, 60U * RIG_MIN_MS);
    ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer)); /* the entry was stored */
    ASSERT_EQ(r.airflow.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(!an_ntc_above(ACE2K_AIRFLOW_FAN_OFF_MC));
    ASSERT_TRUE(r.fans_on);
    ASSERT_EQ(ace2k_airflow_fans(&r.airflow, ACE2K_AIRFLOW_OWNER_MANUAL, false, 0U, r.now_ms),
              -ACE2K_EBUSY);
    r.gate_stuck = false; /* the gate reads low again: the hold goes, rule 7 lets the fans off */
    rig_run_ms(&r, 100U);
    ASSERT_TRUE(!r.fans_on);
}

/* A dryer fault stops heat at once (ace2k_heat_abort): the odd half of a cycle whose even half
 * already fired does not fire.  Only a stop, a duty of 0 and the cool-down keep it (heat.h). */
TEST(a_dryer_fault_between_the_halves_of_a_cycle_fires_nothing_more)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, RIG_MIN_MS);
    r.dryer_frozen = true; /* the dryer's tick is run by hand, on the tick chosen below */
    bool half_due = false;
    for (uint32_t i = 0; i < 100U && !half_due; i++) {
        rig_tick(&r);
        half_due = even_half_fired();
    }
    ASSERT_TRUE(half_due); /* the even half fired, its odd edge not yet come */
    ASSERT_TRUE(ace2k_heat_leased(&r.heat));
    struct ace2k_dryer_inputs in = r.last_in;
    in.chamber_valid = false; /* the chamber reading lost: a dryer fault on this very tick */
    uint32_t fired = r.heat.fired;
    ace2k_dryer_tick(&r.dryer, &in, r.now_ms);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_CHAMBER_STALE);
    ASSERT_TRUE((r.heat.fire_mode != ACE2K_HEAT_FIRE_PENDING));
    r.dryer_frozen = false;
    rig_run_ms(&r, 1000U);
    ASSERT_EQ(r.heat.fired, fired);
}

/* The outer loop's period closing on a tick with no valid chamber reading takes no trim step,
 * however far the (meaningless) reading is from the target. */
TEST(the_outer_loop_takes_no_step_on_an_invalid_chamber_reading)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    struct ace2k_dryer_inputs in = r.last_in;
    r.dryer.ctl.ch_min_mc = 54900;
    r.dryer.ctl.ch_max_mc = 55100; /* a steady window */
    r.dryer.ctl.outer_next_ms = r.now_ms;
    in.chamber_valid = false;
    in.chamber_mc = INT32_MIN + 1;
    (void)ace2k_dryer_ctl_step(&r.dryer, &in, r.now_ms + RIG_TICK_MS);
    ASSERT_EQ(r.dryer.ctl.trim_mc, 0);
    ASSERT_EQ(r.dryer.ctl.ch_min_mc, INT32_MAX); /* the window restarts all the same */
    /* the same window closing on a valid reading does step */
    r.dryer.ctl.ch_min_mc = 54900;
    r.dryer.ctl.ch_max_mc = 55100;
    r.dryer.ctl.outer_next_ms = r.now_ms;
    in.chamber_valid = true;
    in.chamber_mc = 54950;
    (void)ace2k_dryer_ctl_step(&r.dryer, &in, r.now_ms + RIG_TICK_MS);
    ASSERT_EQ(r.dryer.ctl.trim_mc, 25); /* (55 000 − 54 950) / 2 */
}

/* The lease off (duty 0, the NTCs above the drive), the fans lost, then heat wanted again: heat
 * refuses the lease on its verdict without latching, and the dryer names the verdict's reason. */
TEST(a_lease_refused_on_the_fans_verdict_faults_as_fans)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 64000;
    r.ovr_right_mc = 64000;
    rig_run_ms(&r, 5000U);
    ASSERT_EQ(r.dryer.ctl.duty_cpct, 0U);
    ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    uint32_t fired = r.heat.fired;
    r.fan_fail = true;
    rig_tick(&r);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_HEATING);
    r.ovr_left_mc = 55000;
    r.ovr_right_mc = 55000;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_FAULT, 5000U));
    ASSERT_TRUE(r.heat.state != ACE2K_HEAT_LATCHED);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
    ASSERT_EQ(r.heat.fired, fired); /* not a half-cycle since the fans were lost */
}

TEST(the_status_reports_the_cycle)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 55U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, RIG_MIN_MS);
    struct ace2k_dryer_status st;
    ace2k_dryer_status(&r.dryer, &st, r.now_ms);
    ASSERT_EQ(st.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(st.target_c, 55U);
    ASSERT_EQ(st.drive_dc, 620U);
    ASSERT_TRUE(st.remaining_min == 58U || st.remaining_min == 59U);
    ASSERT_EQ(st.fans_read, ACE2K_DRYER_FANS_BOTH);
    ASSERT_EQ(st.flaps,
              (uint8_t)((unsigned)ACE2K_FLAP_CLOSED | ((unsigned)ACE2K_FLAP_CLOSED << 2U)));
    ASSERT_TRUE(st.duty_pct <= ACE2K_HEAT_DUTY_MAX_PCT);
}

static void emit_n(struct ace2k_dryer *d, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        ace2k_dryer_emit(d, ACE2K_DRYER_EV_VENTED, (uint8_t)i);
    }
}

static unsigned send_all(struct ace2k_dryer *d, uint8_t *first_seq)
{
    struct ace2k_dryer_event ev;
    unsigned n = 0;
    while (ace2k_dryer_event_next(d, &ev)) {
        if (n == 0U && first_seq) {
            *first_seq = ev.seq;
        }
        ace2k_dryer_event_sent(d);
        n++;
    }
    return n;
}

TEST(an_unacknowledged_event_is_sent_again_after_a_resend_and_an_acknowledged_one_never)
{
    rig_init(&r, 25000);
    rig_drain(&r); /* the boot's events acknowledged */
    emit_n(&r.dryer, 3U);
    uint8_t first;
    ASSERT_EQ(send_all(&r.dryer, &first), 3U);
    ASSERT_EQ(send_all(&r.dryer, NULL), 0U); /* sent: not again without a resend */
    ASSERT_TRUE(ace2k_dryer_event_unacked(&r.dryer));
    ace2k_dryer_event_resend(&r.dryer);
    uint8_t again;
    ASSERT_EQ(send_all(&r.dryer, &again), 3U);
    ASSERT_EQ(again, first);
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(first + 1U)); /* the first two */
    ace2k_dryer_event_resend(&r.dryer);
    struct ace2k_dryer_event ev;
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ASSERT_EQ(ev.seq, (uint8_t)(first + 2U));
    ace2k_dryer_event_sent(&r.dryer);
    ace2k_dryer_event_ack(&r.dryer, ev.seq);
    ASSERT_TRUE(!ace2k_dryer_event_unacked(&r.dryer));
    ace2k_dryer_event_resend(&r.dryer);
    ASSERT_TRUE(!ace2k_dryer_event_next(&r.dryer, &ev));
}

TEST(the_acknowledgement_follows_the_sequence_across_its_wrap)
{
    rig_init(&r, 25000);
    rig_drain(&r);
    r.dryer.ev_seq = 254U;
    emit_n(&r.dryer, 4U); /* seq 254, 255, 0, 1 */
    ASSERT_EQ(send_all(&r.dryer, NULL), 4U);
    ace2k_dryer_event_ack(&r.dryer, 0U);
    struct ace2k_dryer_event ev;
    ace2k_dryer_event_resend(&r.dryer);
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ASSERT_EQ(ev.seq, 1U);
    ASSERT_EQ(send_all(&r.dryer, NULL), 1U);
    ace2k_dryer_event_ack(&r.dryer, 253U); /* behind the oldest: nothing more leaves */
    ace2k_dryer_event_resend(&r.dryer);
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ASSERT_EQ(ev.seq, 1U);
}

/* An acknowledgement ahead of the newest event ever sent is ignored whole: the events after the
 * sent ones were never seen by the host, and taking them would drop them uncounted.  One for an
 * event sent before a resend, landing after it, is honoured. */
TEST(an_acknowledgement_beyond_the_sent_events_is_ignored)
{
    rig_init(&r, 25000);
    rig_drain(&r);
    emit_n(&r.dryer, 4U);
    struct ace2k_dryer_event ev;
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    uint8_t first = ev.seq;
    ace2k_dryer_event_sent(&r.dryer); /* only the first one sent */
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(first + 3U));
    ace2k_dryer_event_resend(&r.dryer);
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ASSERT_EQ(ev.seq, first); /* not even the sent one was taken */
    ASSERT_EQ(send_all(&r.dryer, NULL), 4U);
    ace2k_dryer_event_resend(&r.dryer);                     /* a resend, nothing sent again yet */
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(first + 1U)); /* sent before it: honoured */
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ASSERT_EQ(ev.seq, (uint8_t)(first + 2U)); /* the resend's cursor moved with the tail */
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(first + 4U)); /* never sent: ignored */
    ASSERT_EQ(send_all(&r.dryer, NULL), 2U);
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(first + 3U)); /* all four sent: all four taken */
    ASSERT_TRUE(!ace2k_dryer_event_unacked(&r.dryer));
    struct ace2k_dryer_status st;
    ace2k_dryer_status(&r.dryer, &st, r.now_ms);
    ASSERT_EQ(st.events_lost, 0U);
}

/* The rest of a short cycle after its start: stopped, cooled down with both outlets steady. */
static void cycle_to_idle(void)
{
    rig_run_ms(&r, RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 40000;
    r.ovr_right_mc = 40000;
    ASSERT_TRUE(rig_run_until(&r, ACE2K_DRYER_IDLE, 11U * RIG_MIN_MS));
    r.ovr_ntc = false;
}

/* The first start after init takes the reading; back to back, the second start — the chamber
 * much drier from the first cycle — keeps the first start's room reference. */
TEST(a_quick_restart_keeps_the_room_reference_of_the_cycle_before)
{
    rig_init(&r, 25000);
    r.rh_pct10 = 600U;
    rig_run_ms(&r, 1000U); /* the reading taken in */
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    uint32_t first = r.dryer.vent.room_cg;
    ASSERT_EQ(first, ace2k_dryer_vent_ah_cg(r.dryer.last.chamber_mc, 600U));
    ASSERT_TRUE(r.dryer.room.kept);
    cycle_to_idle();
    r.rh_pct10 = 100U;
    rig_run_ms(&r, RIG_MIN_MS);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    ASSERT_EQ(r.dryer.vent.room_cg, first);
    ASSERT_TRUE(first > ace2k_dryer_vent_ah_cg(r.dryer.last.chamber_mc, 100U));
}

/* The binding's timer resend: only once the host listens, with an event unacknowledged, and 5 s
 * after the last send — across the clock's wrap; the query's resend puts the cursor back on the
 * oldest unacknowledged event. */
TEST(the_timer_resend_is_due_five_seconds_after_the_last_send_with_an_event_unacknowledged)
{
    rig_init(&r, 25000);
    rig_drain(&r);
    const uint32_t sent = UINT32_MAX - 1000U;
    const uint32_t due = sent + ACE2K_DRYER_EVENT_RESEND_MS;               /* wraps */
    ASSERT_TRUE(!ace2k_dryer_event_resend_due(&r.dryer, true, sent, due)); /* none unacked */
    emit_n(&r.dryer, 2U);
    ASSERT_EQ(send_all(&r.dryer, NULL), 2U);
    ASSERT_TRUE(!ace2k_dryer_event_resend_due(&r.dryer, false, sent, due)); /* no host */
    ASSERT_TRUE(!ace2k_dryer_event_resend_due(&r.dryer, true, sent, due - 1U));
    ASSERT_TRUE(ace2k_dryer_event_resend_due(&r.dryer, true, sent, due));
    ace2k_dryer_event_resend(&r.dryer); /* the query's, or the timer's */
    ASSERT_EQ(send_all(&r.dryer, NULL), 2U);
    struct ace2k_dryer_event ev;
    ace2k_dryer_event_resend(&r.dryer);
    ASSERT_TRUE(ace2k_dryer_event_next(&r.dryer, &ev));
    ace2k_dryer_event_ack(&r.dryer, (uint8_t)(ev.seq + 1U));
    ASSERT_TRUE(!ace2k_dryer_event_resend_due(&r.dryer, true, sent, due)); /* all acknowledged */
}

static uint8_t status_oldest(void)
{
    struct ace2k_dryer_status st;
    ace2k_dryer_status(&r.dryer, &st, r.now_ms);
    return st.oldest;
}

/* The state report's oldest: the oldest unacknowledged event, or the next seq when none is held —
 * through acks and across the sequence's wrap. */
TEST(the_status_reports_the_oldest_unacknowledged_event_or_the_next_seq)
{
    rig_init(&r, 25000);
    rig_drain(&r);
    r.dryer.ev_seq = 253U;
    ASSERT_EQ(status_oldest(), 253U); /* empty: the next one's */
    emit_n(&r.dryer, 4U);             /* 253, 254, 255, 0 */
    ASSERT_EQ(status_oldest(), 253U);
    ASSERT_EQ(send_all(&r.dryer, NULL), 4U);
    ace2k_dryer_event_ack(&r.dryer, 254U);
    ASSERT_EQ(status_oldest(), 255U);
    ace2k_dryer_event_ack(&r.dryer, 0U); /* across the wrap: all taken */
    ASSERT_EQ(status_oldest(), 1U);
    emit_n(&r.dryer, 1U);
    ASSERT_EQ(status_oldest(), 1U);
}

TEST(an_eighth_unacknowledged_event_is_dropped_and_counted_in_the_status)
{
    rig_init(&r, 25000);
    rig_drain(&r);
    emit_n(&r.dryer, ACE2K_DRYER_EVENT_RING); /* eight into seven slots */
    struct ace2k_dryer_status st;
    ace2k_dryer_status(&r.dryer, &st, r.now_ms);
    ASSERT_EQ(st.events_lost, 1U);
    ASSERT_EQ(send_all(&r.dryer, NULL), ACE2K_DRYER_EVENT_RING - 1U);
}

/* Measured on the unit: the overshoot after the heater stops passed "fell less than 1 °C/min" and closed
 * the flaps early.  A window over which an outlet rose never ends the cool-down. */
TEST(a_cool_down_never_ends_on_a_rising_outlet)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 44000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS + 1000U); /* the first window closes */
    r.ovr_left_mc = 44400;
    r.ovr_right_mc = 44400;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the second: rising */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the third: steady */
    ASSERT_TRUE(r.dryer.cool != ACE2K_DRYER_COOL_WAITING);
}

/* A rise within ACE2K_DRYER_COOL_RISE_MC over the window is the NTC's noise, read as steady: the
 * cool-down ends on it.  One m°C more is a rise, and it does not. */
static void cool_window_after_rise(int32_t rise_mc)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 44000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS + 1000U); /* the first window closes */
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    r.ovr_left_mc = 44000 + rise_mc;
    r.ovr_right_mc = 44000 + rise_mc;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the second */
}

TEST(a_cool_down_ends_on_a_rise_within_the_noise_tolerance)
{
    cool_window_after_rise(ACE2K_DRYER_COOL_RISE_MC);
    ASSERT_TRUE(r.dryer.cool != ACE2K_DRYER_COOL_WAITING);
}

TEST(a_cool_down_does_not_end_on_a_rise_one_millidegree_over_the_tolerance)
{
    cool_window_after_rise(ACE2K_DRYER_COOL_RISE_MC + 1);
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
}

/* Both outlets steady at 44 °C through a first window, then a cool-down entered by a stop. */
static void cool_steady_first_window(void)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 44000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS + 1000U); /* the first window closes */
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
}

/* An NTC outage across a window's close rolls no reference: the readings taken while invalid
 * (here 44.5 °C) are not what the first window after the recovery is judged against.  The
 * outlets come back at 44.0 °C and rise to 44.3 °C — a rise over that window, so no end; judged
 * against the outage's 44.5 °C it would have read as a fall. */
TEST(a_cool_down_does_not_end_on_a_rising_outlet_after_an_ntc_outage)
{
    cool_steady_first_window();
    r.ovr_left_valid = false;
    r.ovr_right_valid = false;
    r.ovr_left_mc = 44500;
    r.ovr_right_mc = 44500;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS + 1000U); /* the second window closes in the outage */
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 44000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS / 2U);
    r.ovr_left_mc = 44300;
    r.ovr_right_mc = 44300;
    rig_run_ms(&r, (ACE2K_DRYER_COOL_MIN_MS / 2U) + 2000U); /* a window from the recovery closes */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the next one: steady at 44.3 °C */
    ASSERT_TRUE(r.dryer.cool != ACE2K_DRYER_COOL_WAITING);
}

/* The rule is judged at a window's two ends only: a spike in mid-window (here over 45 °C) that
 * is back at the reference by the close does not hold the cool-down. */
TEST(a_cool_down_judges_the_window_at_its_ends_only)
{
    cool_steady_first_window();
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS / 3U);
    r.ovr_left_mc = 46000;
    r.ovr_right_mc = 46000;
    rig_run_ms(&r, 5000U);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 44000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the window closes back at the reference */
    ASSERT_TRUE(r.dryer.cool != ACE2K_DRYER_COOL_WAITING);
}

/* One side rising is enough to refuse: left steady at 44.0 °C, right 43.0 → 43.4 °C (over the
 * noise tolerance) over the window. */
TEST(a_cool_down_does_not_end_on_one_rising_outlet)
{
    rig_init(&r, 25000);
    ASSERT_EQ(rig_start(&r, 45U, 60U), ACE2K_DRYER_OK);
    rig_run_ms(&r, 2U * RIG_MIN_MS);
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, r.now_ms), ACE2K_DRYER_OK);
    r.ovr_ntc = true;
    r.ovr_left_valid = true;
    r.ovr_right_valid = true;
    r.ovr_left_mc = 44000;
    r.ovr_right_mc = 43000;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS + 1000U); /* the first window closes */
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
    r.ovr_right_mc = 43400;
    rig_run_ms(&r, ACE2K_DRYER_COOL_MIN_MS); /* the second: the right one rose */
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(r.dryer.cool, ACE2K_DRYER_COOL_WAITING);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
