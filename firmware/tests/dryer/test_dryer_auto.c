/* The automatic modes against the real dryer on the rig: the humidity mode's start, outcome,
 * failures and stop; the manual calls; the no-host profile rule; the bound cycle; and the cold
 * rule replayed on three cool-downs recorded on the printer (tests/data/dryer-modes/). The CSVs
 * are read relative to firmware/, where make runs the tests. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dryer/dryer_auto.h"
#include "fake_dryer_rig.h"
#include "test.h"

#define CSV_DIR   "tests/data/dryer-modes/"
#define HOLD_MS   60000U /* the bench's hold: 1 min */
#define WINDOW_MS 60000U /* the bench's window: 1 min */
#define LINK_MS   60000U
#define CYCLE_MIN 2U
#define COOL_ROOM 25000 /* a fixed chamber: the box reads cold once the dryer is idle */

// NOLINTNEXTLINE(readability-identifier-naming)
struct auto_rig {
    struct rig r;
    struct ace2k_dryer_auto a;
    struct ace2k_dryer_auto_store s;
};

static void auto_init(struct auto_rig *t, uint8_t mode)
{
    rig_init(&t->r, COOL_ROOM);
    t->r.ovr_chamber = true;
    t->r.ovr_chamber_mc = COOL_ROOM;
    ace2k_dryer_auto_store_defaults(&t->s);
    ace2k_dryer_auto_init(&t->a, &t->r.dryer, &t->s);
    ace2k_dryer_auto_timing(&t->a, HOLD_MS, WINDOW_MS, LINK_MS);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t->a, 45U, CYCLE_MIN), 0);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t->a, mode), 0);
    t->r.auto_ = &t->a;
}

static void set_rh(struct auto_rig *t, uint32_t pct)
{
    t->r.rh_pct10 = (uint16_t)(pct * 10U);
}

/* Runs until the dryer leaves IDLE, at most max_ms; true when it did. */
static bool run_until_started(struct auto_rig *t, uint32_t max_ms)
{
    for (uint32_t ms = 0; ms < max_ms; ms += RIG_TICK_MS) {
        if (t->r.dryer.state != ACE2K_DRYER_IDLE) {
            return true;
        }
        rig_tick(&t->r);
    }
    return t->r.dryer.state != ACE2K_DRYER_IDLE;
}

/* The arg of the last event of `kind`, or -1 when there is none. */
static int last_arg(const struct rig *r, uint8_t kind)
{
    int arg = -1;
    for (uint32_t i = 0; i < r->ev_n; i++) {
        if (r->ev[i].kind == kind) {
            arg = r->ev[i].arg;
        }
    }
    return arg;
}

/* One own cycle from a box above the limit to the dryer idle again, the RH at `outcome_pct`. */
static void cycle_to_idle(struct auto_rig *t, uint32_t outcome_pct)
{
    set_rh(t, 60U);
    ASSERT_TRUE(run_until_started(t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(t->a.own);
    set_rh(t, outcome_pct);
    ASSERT_TRUE(rig_run_until(&t->r, ACE2K_DRYER_COOLDOWN, (CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t->r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
}

/* One own cycle from a box above the limit to its outcome read at `outcome_pct`. */
static void one_cycle(struct auto_rig *t, uint32_t outcome_pct)
{
    cycle_to_idle(t, outcome_pct);
    /* the window fills, the box reads cold, the outcome is taken */
    rig_run_ms(&t->r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_TRUE(!t->a.outcome_due);
}

TEST(a_box_above_the_limit_for_the_hold_starts_one_cycle_at_the_profile)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 41U);
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS - (RIG_MIN_MS / 2U));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE); /* not yet: cold, but the hold has not run */
    ASSERT_TRUE(run_until_started(&t, RIG_MIN_MS));
    ASSERT_EQ(t.r.dryer.target_c, 45U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STARTED), 1U);
    ASSERT_EQ(last_arg(&t.r, ACE2K_DRYER_EV_AUTO_STARTED), 45);
}

TEST(a_box_below_the_limit_never_starts)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    t.r.rh_pct10 = 399U;
    rig_run_ms(&t.r, WINDOW_MS + (3U * HOLD_MS));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
}

/* An outcome at the limit is a failure, so a box at the limit must start again: the start is the
 * complement of a success. */
TEST(a_box_at_the_limit_starts)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 40U);
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS - (RIG_MIN_MS / 2U));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(run_until_started(&t, RIG_MIN_MS));
    ASSERT_TRUE(t.a.own);
}

TEST(the_manual_and_print_modes_never_start_on_humidity)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    set_rh(&t, 80U);
    rig_run_ms(&t.r, WINDOW_MS + (3U * HOLD_MS));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    rig_run_ms(&t.r, 3U * HOLD_MS);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold); /* cold and above_s run in every mode, for the host's print mode */
    ASSERT_TRUE(st.above_s >= (2U * HOLD_MS) / 1000U);
}

TEST(the_operators_sequence_counts_1_2_0_0_1)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    const uint32_t outcome[] = { 45U, 41U, 39U, 35U, 41U };
    const uint8_t failures[] = { 1U, 2U, 0U, 0U, 1U };
    for (uint32_t i = 0; i < 5U; i++) {
        one_cycle(&t, outcome[i]);
        ASSERT_EQ(t.s.failures, failures[i]);
    }
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 3U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_DRY), 2U);
    ASSERT_EQ(t.s.flags & ACE2K_DRYER_AUTO_STOPPED, 0U);
}

TEST(an_outcome_at_the_limit_is_a_failure)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    one_cycle(&t, 40U);
    ASSERT_EQ(t.s.failures, 1U);
}

TEST(the_fourth_failure_stops_the_mode_until_it_is_enabled_again)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    for (uint32_t i = 0; i < 4U; i++) {
        one_cycle(&t, 50U);
    }
    ASSERT_EQ(t.s.failures, 4U);
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, 3U * HOLD_MS);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE); /* stopped: never again by itself */
    set_rh(&t, 30U);
    rig_run_ms(&t.r, 3U * HOLD_MS);
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U); /* a good reading does not clear */
    /* a reset: the store comes back from the page as it was */
    struct ace2k_dryer_auto_store kept = t.s;
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &kept);
    t.a.store = &t.s;
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    ASSERT_EQ(t.s.flags & ACE2K_DRYER_AUTO_STOPPED, 0U);
    ASSERT_EQ(t.s.failures, 0U);
}

TEST(a_manual_stop_of_an_own_cycle_turns_the_mode_off)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_manual_stop(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&t.r);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 1U);
    ASSERT_TRUE(!t.a.own && !t.a.outcome_due);
}

TEST(a_manual_start_over_an_own_cycle_makes_it_manual)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 60U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.target_c, 60U);
    ASSERT_TRUE(!t.a.own);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_HUMIDITY); /* the mode stays */
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    ASSERT_EQ(t.s.failures, 0U); /* a manual cycle counts neither way */
}

TEST(a_manual_start_over_a_manual_cycle_is_busy)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 50U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 60U, 30U, t.r.now_ms), ACE2K_DRYER_BUSY);
}

TEST(a_running_manual_cycle_blocks_the_humidity_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 30U);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 50U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, 10U * RIG_MIN_MS);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STARTED), 0U);
    ASSERT_TRUE(!t.a.own);
}

TEST(a_mode_change_stops_its_running_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_MANUAL), 0);
    rig_tick(&t.r);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 0U); /* the user's choice, no notice */
}

TEST(a_reset_mid_cycle_counts_neither_way_and_keeps_the_count)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    one_cycle(&t, 50U);
    ASSERT_EQ(t.s.failures, 1U);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    struct ace2k_dryer_auto_store kept = t.s; /* the page as the reset finds it */
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);   /* a fresh dryer: a reset never resumes a cycle */
    t.r.auto_ = NULL;
    t.s = kept;
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &t.s);
    t.r.auto_ = &t.a;
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(t.s.failures, 1U);
    ASSERT_TRUE(!t.a.own && !t.a.outcome_due);
}

TEST(a_fault_ends_an_own_cycle_with_no_outcome)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    t.r.cutout = true;
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_FAULT, 2000U));
    rig_run_ms(&t.r, 2U * RIG_MIN_MS);
    ASSERT_TRUE(!t.a.own && !t.a.outcome_due);
    ASSERT_EQ(t.s.failures, 0U);
}

/* The lid fault: the NTCs hot, the chamber flat. */
static void open_the_lid(struct auto_rig *t)
{
    t->r.ovr_ntc = true;
    t->r.ovr_left_valid = true;
    t->r.ovr_right_valid = true;
    t->r.ovr_left_mc = 45000;
    t->r.ovr_right_mc = 45000;
}

TEST(a_lid_fault_ends_an_own_cycle_as_a_failure_at_once)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    t.s.max_failures = 2U;
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 55U, 60U), 0); /* long enough for the lid check */
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    open_the_lid(&t);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_FAULT, 15U * RIG_MIN_MS));
    rig_tick(&t.r);
    ASSERT_EQ(t.r.dryer.fault, ACE2K_DRYER_FAULT_LID);
    ASSERT_EQ(t.s.failures, 1U);
    ASSERT_EQ(last_arg(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 1);
    ASSERT_TRUE(!t.a.own && !t.a.outcome_due);
    rig_run_ms(&t.r, 2U * RIG_MIN_MS);
    ASSERT_EQ(t.s.failures, 1U); /* counted once */
}

TEST(a_lid_fault_at_the_maximum_stops_the_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    t.s.max_failures = 1U;
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 55U, 60U), 0);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    open_the_lid(&t);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_FAULT, 15U * RIG_MIN_MS));
    rig_tick(&t.r);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 1U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U);
}

TEST(a_bound_cycle_ending_in_a_lid_fault_counts_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 55U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    open_the_lid(&t);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_FAULT, 15U * RIG_MIN_MS));
    rig_run_ms(&t.r, 2U * RIG_MIN_MS);
    ASSERT_EQ(t.r.dryer.fault, ACE2K_DRYER_FAULT_LID);
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
}

TEST(a_manual_cycle_ending_in_a_lid_fault_counts_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 55U, 60U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    open_the_lid(&t);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_FAULT, 15U * RIG_MIN_MS));
    rig_run_ms(&t.r, 2U * RIG_MIN_MS);
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
}

TEST(an_invalid_rh_never_starts_and_restarts_the_hold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS - (RIG_MIN_MS / 4U));
    t.r.rh_valid = false;
    rig_run_ms(&t.r, ACE2K_DRYER_AUTO_INVALID_MS + RIG_MIN_MS);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    t.r.rh_valid = true;
    /* readings invalid past the limit are not a cold box: the window fills again, then a whole
     * hold runs */
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS - (RIG_MIN_MS / 4U));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE); /* the hold started again */
    ASSERT_TRUE(run_until_started(&t, RIG_MIN_MS));
}

TEST(a_stale_chamber_is_not_a_cold_box)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    t.r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_run_ms(&t.r, WINDOW_MS + (3U * HOLD_MS));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(!t.a.cold);
}

TEST(a_refused_start_waits_another_hold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    t.r.mains_present = false; /* the dryer refuses NO_MAINS */
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS + (RIG_MIN_MS / 2U));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    t.r.mains_present = true;
    rig_run_ms(&t.r, HOLD_MS - RIG_MIN_MS); /* not before a full hold from the refusal */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(run_until_started(&t, 2U * RIG_MIN_MS));
}

TEST(an_insert_with_no_host_lowers_the_stored_profile)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 60U, 240U), 0);
    t.r.insert_mask = 0x1U; /* with the host: nothing */
    rig_run_ms(&t.r, 100U);
    ASSERT_EQ(t.s.profile_c, 60U);
    t.r.link_ok = false;
    t.r.insert_mask = 0x0U; /* a removal: nothing */
    rig_run_ms(&t.r, 100U);
    ASSERT_EQ(t.s.profile_c, 60U);
    t.r.insert_mask = 0x4U;
    rig_run_ms(&t.r, 100U);
    ASSERT_EQ(t.s.profile_c, 45U);
    ASSERT_EQ(t.s.profile_min, 360U);
    ASSERT_TRUE(ace2k_dryer_auto_dirty(&t.a));
}

TEST(an_insert_with_no_host_leaves_a_profile_already_at_45)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, 120U), 0);
    t.r.link_ok = false;
    t.r.insert_mask = 0x2U;
    rig_run_ms(&t.r, 100U);
    ASSERT_EQ(t.s.profile_min, 120U);
}

TEST(a_bound_cycle_ends_after_the_link_time_and_a_plain_one_does_not)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    t.r.link_ok = false;
    rig_run_ms(&t.r, LINK_MS - 1000U);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_HEATING);
    t.r.link_ok = true; /* a host restart: the link back in time */
    rig_run_ms(&t.r, 2000U);
    t.r.link_ok = false;
    rig_run_ms(&t.r, LINK_MS + 1000U);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_STOPPED_LINK), 1U);
    ASSERT_TRUE(!t.a.bound);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 45U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    rig_run_ms(&t.r, LINK_MS + 1000U); /* link still down: a manual cycle runs on */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_HEATING);
}

TEST(a_manual_stop_of_a_bound_cycle_leaves_the_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_dryer_auto_manual_stop(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&t.r);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 0U);
}

TEST(the_calls_refuse_out_of_range_and_change_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, 3U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 4U, 4U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 96U, 4U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, 0U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, 21U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 14U, 60U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, 0U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, 1441U), -ACE2K_EINVAL);
    ASSERT_EQ(t.s.limit_pct, 40U);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 35U, 6U), 0);
    ASSERT_EQ(t.s.limit_pct, 35U);
    ASSERT_EQ(t.s.max_failures, 6U);
}

TEST(a_store_out_of_range_is_reset_field_by_field)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    struct ace2k_dryer_auto_store bad = {
        .mode = 7U,
        .limit_pct = 99U,
        .max_failures = 0U,
        .failures = 2U,
        .profile_c = 90U,
        .flags = 0U,
        .profile_min = 0U,
    };
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &bad);
    ASSERT_EQ(bad.mode, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(bad.limit_pct, ACE2K_DRYER_AUTO_LIMIT_DEFAULT_PCT);
    ASSERT_EQ(bad.max_failures, ACE2K_DRYER_AUTO_FAILURES_DEFAULT);
    ASSERT_EQ(bad.failures, 2U);
    ASSERT_EQ(bad.profile_c, ACE2K_DRYER_AUTO_PROFILE_C_DEFAULT);
    ASSERT_EQ(bad.profile_min, ACE2K_DRYER_AUTO_PROFILE_MIN_DEFAULT);
}

/* An outcome waits through bad readings: none is decided until they count again. */
static void outcome_waits_for_valid_readings(bool stale_chamber)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_TRUE(t.a.outcome_due);
    if (stale_chamber) {
        t.r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    } else {
        t.r.rh_valid = false;
    }
    rig_run_ms(&t.r, 3U * WINDOW_MS);
    ASSERT_TRUE(t.a.outcome_due);
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_DRY), 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
    t.r.chamber_age_ms = 0U;
    t.r.rh_valid = true;
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_TRUE(!t.a.outcome_due);
    ASSERT_EQ(t.s.failures, 1U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 1U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_DRY), 0U);
}

TEST(an_invalid_rh_decides_no_outcome)
{
    outcome_waits_for_valid_readings(false);
}

TEST(a_stale_chamber_decides_no_outcome)
{
    outcome_waits_for_valid_readings(true);
}

TEST(a_manual_start_after_a_mode_change_keeps_its_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_MANUAL), 0);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 60U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    rig_run_ms(&t.r, 1000U);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_HEATING); /* the user's new cycle runs on */
    ASSERT_EQ(t.r.dryer.target_c, 60U);
}

TEST(a_maximum_lowered_below_the_count_starts_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, 3U), 0);
    for (int i = 0; i < 2; i++) {
        cycle_to_idle(&t, 40U); /* at the limit: a failure */
        for (uint32_t ms = 0; ms < WINDOW_MS + (2U * RIG_MIN_MS) && t.a.outcome_due;
             ms += RIG_TICK_MS) {
            rig_tick(&t.r);
        }
        ASSERT_TRUE(!t.a.outcome_due);
        t.r.rh_pct10 = 399U; /* below the limit at once: no new hold runs */
    }
    ASSERT_EQ(t.s.failures, 2U);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, 2U), 0);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + (3U * HOLD_MS));
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STARTED), 2U);
}

TEST(a_manual_start_over_a_bound_cycle_makes_it_manual)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 50U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(!t.a.bound);
    ASSERT_EQ(t.r.dryer.target_c, 50U);
    t.r.link_ok = false;
    rig_run_ms(&t.r, LINK_MS + 1000U); /* a manual cycle runs on with the link down */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_STOPPED_LINK), 0U);
}

/* The cold rule on the recorded cool-downs. The chamber and the RH are the nearest row's: a row
 * takes over halfway from the one before, as the data's own reduction read them, gaps included.
 * The dryer's state changes at its row. The state comes from the file, so the detector runs
 * alone, on a dryer whose state the test sets. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct replay {
    struct rig r;
    struct ace2k_dryer_auto a;
    struct ace2k_dryer_auto_store s;
    struct ace2k_dryer_inputs in;
    long t_s;
    uint32_t now_ms;
    bool ran;
    long idle_at;
    double found;
};

// NOLINTNEXTLINE(readability-identifier-naming)
struct replay_row {
    long t_s;
    int32_t chamber_mc;
    uint16_t rh_pct10;
    bool idle;
};

/* Parses "t_s,chamber_c,rh_pct,state". False for a line that is not a row. */
static bool replay_parse(char *line, struct replay_row *row)
{
    char *p = line;
    char *end = NULL;
    row->t_s = strtol(p, &end, 10);
    if (end == p || *end != ',') {
        return false;
    }
    p = end + 1;
    double ch = strtod(p, &end);
    if (end == p || *end != ',') {
        return false;
    }
    p = end + 1;
    double rh = strtod(p, &end);
    if (end == p || *end != ',') {
        return false;
    }
    row->chamber_mc = (int32_t)lround(ch * 1000.0);
    row->rh_pct10 = (uint16_t)lround(rh * 10.0);
    row->idle = strncmp(end + 1, "idle", 4) == 0;
    return true;
}

/* Ticks once a second up to the row, the row's readings from halfway; then the row's state. */
static void replay_step(struct replay *rp, const struct replay_row *row)
{
    long mid = (rp->t_s + row->t_s) / 2;
    for (; rp->t_s < row->t_s && rp->found < 0.0; rp->t_s++) {
        if (rp->t_s == mid) {
            rp->in.chamber_mc = row->chamber_mc;
            rp->in.chamber_rh_pct10 = row->rh_pct10;
        }
        rp->now_ms += 1000U;
        ace2k_dryer_auto_tick(&rp->a, &rp->in, rp->now_ms);
        if (rp->a.cold && rp->ran && rp->idle_at >= 0) {
            rp->found = (double)(rp->t_s - rp->idle_at) / 3600.0;
        }
    }
    rp->r.dryer.state = row->idle ? ACE2K_DRYER_IDLE : ACE2K_DRYER_HEATING;
    if (!row->idle) {
        rp->ran = true;
        rp->idle_at = -1;
    } else if (rp->ran && rp->idle_at < 0) {
        rp->idle_at = row->t_s;
    }
    rp->in.chamber_mc = row->chamber_mc;
    rp->in.chamber_rh_pct10 = row->rh_pct10;
}

/* Returns the hours from the first idle row after a running one to the first cold tick, or -1. */
static double replay_cold_h(const char *file)
{
    char path[128];
    (void)snprintf(path, sizeof path, CSV_DIR "%s", file);
    FILE *f = fopen(path, "r");
    ASSERT_TRUE(f != NULL);
    if (f == NULL) {
        return -1.0;
    }
    static struct replay rp;
    memset(&rp, 0, sizeof rp);
    ace2k_dryer_auto_store_defaults(&rp.s);
    ace2k_dryer_auto_init(&rp.a, &rp.r.dryer, &rp.s);
    rp.in.chamber_valid = true;
    rp.in.rh_valid = true;
    rp.in.link_ok = true;
    rp.idle_at = -1;
    rp.found = -1.0;
    char line[128];
    if (fgets(line, sizeof line, f) == NULL) { /* the header */
        (void)fclose(f);
        return -1.0;
    }
    struct replay_row row;
    while (fgets(line, sizeof line, f) != NULL && rp.found < 0.0) {
        if (replay_parse(line, &row)) {
            replay_step(&rp, &row);
        }
    }
    (void)fclose(f);
    return rp.found;
}

/* The host sends the bench's timing in seconds. A value whose milliseconds do not fit 32 bits is
 * refused, and nothing changes; 0 keeps the default, as in ace2k_dryer_auto_timing(). */
TEST(the_timing_in_seconds_refuses_an_overflow_and_changes_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, 120U, 0U, 7U), 0);
    ASSERT_EQ(t.a.hold_ms, 120000U);
    ASSERT_EQ(t.a.window_ms, ACE2K_DRYER_AUTO_COLD_WINDOW_MS);
    ASSERT_EQ(t.a.link_ms, 7000U);
    const uint32_t over = UINT32_MAX / 1000U + 1U;
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, 1U, 1U, over), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, over, 1U, 1U), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, 1U, over, 1U), -ACE2K_EINVAL);
    ASSERT_EQ(t.a.hold_ms, 120000U);
    ASSERT_EQ(t.a.window_ms, ACE2K_DRYER_AUTO_COLD_WINDOW_MS);
    ASSERT_EQ(t.a.link_ms, 7000U);
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, over - 1U, 0U, 0U), 0);
    ASSERT_EQ(t.a.hold_ms, (over - 1U) * 1000U);
}

TEST(the_cold_rule_on_the_recorded_cool_downs)
{
    const double tol_h = 2.0 / 60.0;
    double h = replay_cold_h("cool-24h-45c.csv");
    ASSERT_TRUE(h > 3.92 - tol_h && h < 3.92 + tol_h);
    h = replay_cold_h("cool-30min-55c.csv");
    ASSERT_TRUE(h > 2.95 - tol_h && h < 2.95 + tol_h);
    h = replay_cold_h("cool-10min-45c.csv");
    ASSERT_TRUE(h > 1.92 - tol_h && h < 1.92 + tol_h);
}

/* ---- the branches the scenarios above do not reach ---- */

TEST(a_store_with_one_bad_field_resets_only_that_field)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    struct ace2k_dryer_auto_store low = {
        .mode = ACE2K_DRYER_AUTO_PRINT,
        .limit_pct = ACE2K_DRYER_AUTO_LIMIT_MIN_PCT - 1U,
        .max_failures = 7U,
        .profile_c = 50U,
        .profile_min = 90U,
    };
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &low);
    ASSERT_EQ(low.limit_pct, ACE2K_DRYER_AUTO_LIMIT_DEFAULT_PCT);
    ASSERT_EQ(low.max_failures, 7U);
    ASSERT_EQ(low.mode, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(low.profile_c, 50U);
    struct ace2k_dryer_auto_store many = {
        .mode = ACE2K_DRYER_AUTO_HUMIDITY,
        .limit_pct = 30U,
        .max_failures = ACE2K_DRYER_AUTO_FAILURES_MAX + 1U,
        .profile_c = 50U,
        .profile_min = 90U,
    };
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &many);
    ASSERT_EQ(many.max_failures, ACE2K_DRYER_AUTO_FAILURES_DEFAULT);
    ASSERT_EQ(many.mode, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(many.limit_pct, 30U);
    ASSERT_EQ(many.profile_min, 90U);
}

TEST(a_mode_change_stops_a_bound_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_MANUAL), 0);
    rig_tick(&t.r);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_TRUE(!t.a.bound);
}

TEST(limits_and_profile_set_to_what_they_are_do_not_reach_the_page)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, t.s.limit_pct, t.s.max_failures), 0);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, t.s.profile_c, t.s.profile_min), 0);
    ASSERT_TRUE(!ace2k_dryer_auto_dirty(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, t.s.limit_pct, (uint8_t)(t.s.max_failures + 1U)), 0);
    ASSERT_TRUE(ace2k_dryer_auto_dirty(&t.a));
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, t.s.profile_c, (uint16_t)(t.s.profile_min + 1U)), 0);
    ASSERT_TRUE(ace2k_dryer_auto_dirty(&t.a));
}

TEST(a_timing_below_the_minimum_is_raised_to_it)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ace2k_dryer_auto_timing(&t.a, 10U, 20U, 30U);
    ASSERT_EQ(t.a.hold_ms, ACE2K_DRYER_AUTO_MIN_TIMING_MS);
    ASSERT_EQ(t.a.window_ms, ACE2K_DRYER_AUTO_MIN_TIMING_MS);
    ASSERT_EQ(t.a.link_ms, ACE2K_DRYER_AUTO_MIN_TIMING_MS);
}

TEST(a_manual_start_over_an_own_cycle_out_of_range_is_refused_and_the_cycle_stays_own)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 14U, 30U, t.r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(t.r.dryer.target_c, 45U);
}

TEST(a_manual_start_when_the_own_cycle_has_just_stopped_is_a_plain_start)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK); /* no tick since */
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 50U, 30U, t.r.now_ms), ACE2K_DRYER_BUSY);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
}

TEST(a_manual_stop_keeps_the_mode_when_the_own_cycle_is_no_longer_running)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK); /* no tick since */
    (void)ace2k_dryer_auto_manual_stop(&t.a, t.r.now_ms);
    rig_tick(&t.r);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 0U);
}

TEST(a_manual_stop_after_a_mode_change_gives_no_off_notice)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    ASSERT_EQ(ace2k_dryer_auto_manual_stop(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&t.r);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 0U);
}

TEST(a_bound_start_the_dryer_refuses_binds_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 5U, 60U, t.r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_TRUE(!t.a.bound);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
}

TEST(an_insert_with_no_host_at_the_safe_temperature_changes_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, ACE2K_DRYER_SPOOL_SAFE_C, 120U), 0);
    t.r.link_ok = false;
    rig_run_ms(&t.r, 100U); /* the first look at the sensors */
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    t.r.insert_mask = 0x2U; /* a strand enters */
    rig_run_ms(&t.r, 100U);
    ASSERT_EQ(t.s.profile_min, 120U);
    ASSERT_TRUE(!ace2k_dryer_auto_dirty(&t.a));
}

TEST(a_bound_cycle_that_stopped_is_not_ended_by_the_link_rule)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK);
    t.r.link_ok = false;
    rig_tick(&t.r); /* still marked bound at the link rule, but no longer running */
    ASSERT_TRUE(!t.a.bound);
    ASSERT_TRUE(!t.a.link_down);
    rig_run_ms(&t.r, LINK_MS + 1000U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_STOPPED_LINK), 0U);
}

TEST(the_first_valid_idle_tick_takes_a_sample_even_when_the_clock_reads_before_the_deadline)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &t.s); /* fresh: sample_next_ms is 0 */
    struct ace2k_dryer_inputs in = {
        .chamber_valid = true,
        .chamber_mc = COOL_ROOM,
        .rh_valid = true,
        .link_ok = true,
    };
    const uint32_t late = 0x90000000U; /* half the clock round away from 0: "not after" 0 */
    ace2k_dryer_auto_tick(&t.a, &in, late);
    ASSERT_EQ(t.a.ring_n, 1U);
    ace2k_dryer_auto_tick(&t.a, &in, late); /* the same instant: no second sample */
    ASSERT_EQ(t.a.ring_n, 1U);
}

TEST(a_mode_change_after_the_cycle_drops_its_outcome_without_counting)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_TRUE(t.a.outcome_due);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_MANUAL), 0);
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_TRUE(!t.a.outcome_due);
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_DRY), 0U);
}

TEST(the_failure_count_stops_at_255)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, ACE2K_DRYER_AUTO_FAILURES_MAX), 0);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    t.s.failures = UINT8_MAX; /* as a page that was written by hand would hold it */
    set_rh(&t, 50U);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_COOLDOWN, (CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_EQ(t.s.failures, UINT8_MAX);
    ASSERT_EQ(last_arg(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), UINT8_MAX);
}

/* The host sends the timing at every connect. The same timing again changes nothing: the box
 * stays cold and the hold keeps running. */
TEST(the_same_timing_sent_again_keeps_the_cold_box_and_the_hold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_TRUE(st.above_s >= 30U);
    uint32_t above_s = st.above_s;
    ace2k_dryer_auto_timing(&t.a, HOLD_MS, WINDOW_MS, LINK_MS);
    ASSERT_EQ(ace2k_dryer_auto_timing_s(&t.a, HOLD_MS / 1000U, WINDOW_MS / 1000U, LINK_MS / 1000U),
              0);
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_EQ(st.above_s, above_s);
    /* a new link time resets nothing either */
    ace2k_dryer_auto_timing(&t.a, HOLD_MS, WINDOW_MS, LINK_MS + 1000U);
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_EQ(st.above_s, above_s);
}

TEST(a_changed_window_resets_the_cold_box)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    ASSERT_TRUE(t.a.cold);
    ace2k_dryer_auto_timing(&t.a, HOLD_MS, 2U * WINDOW_MS, LINK_MS);
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(!st.cold);
    ASSERT_EQ(st.above_s, 0U);
    ASSERT_EQ(t.a.ring_n, 0U);
}

TEST(a_changed_hold_restarts_the_hold_and_keeps_the_cold_box)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    ace2k_dryer_auto_timing(&t.a, 2U * HOLD_MS, WINDOW_MS, LINK_MS);
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_EQ(st.above_s, 0U);
    rig_run_ms(&t.r, 2000U);
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.above_s >= 1U && st.above_s <= 2U); /* a new hold from the change */
}

/* The host lowers the maximum to the failures already counted: the mode stops on the next tick,
 * as it would after a failed outcome, once. */
TEST(a_maximum_lowered_to_the_failures_stops_the_humidity_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    one_cycle(&t, 50U);
    one_cycle(&t, 50U);
    ASSERT_EQ(t.s.failures, 2U);
    ASSERT_EQ(t.s.flags & ACE2K_DRYER_AUTO_STOPPED, 0U);
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, t.s.limit_pct, 2U), 0);
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    rig_tick(&t.r);
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
    ASSERT_EQ(last_arg(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 2);
    ASSERT_TRUE(ace2k_dryer_auto_dirty(&t.a));
    rig_run_ms(&t.r, RIG_MIN_MS);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
}

TEST(a_maximum_lowered_in_another_mode_does_not_stop_the_humidity_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    t.s.failures = 3U;
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, t.s.limit_pct, 2U), 0);
    rig_run_ms(&t.r, RIG_MIN_MS);
    ASSERT_EQ(t.s.flags & ACE2K_DRYER_AUTO_STOPPED, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 0U);
}

TEST(the_status_of_a_box_not_above_the_limit_reads_zero_seconds_above)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 30U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_EQ(st.above_s, 0U);
}

/* ---- round 1 of the review ---- */

/* A manual start drops a pending outcome: it counts neither way. */
TEST(a_manual_start_drops_a_pending_outcome)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_TRUE(t.a.outcome_due);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 50U, 1U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(!t.a.outcome_due);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_COOLDOWN, 2U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_DRY), 0U);
}

/* A refused manual start changes nothing: the outcome still waits. */
TEST(a_refused_manual_start_keeps_a_pending_outcome)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 14U, 30U, t.r.now_ms), ACE2K_DRYER_RANGE);
    ASSERT_TRUE(t.a.outcome_due);
}

TEST(a_mode_change_drops_a_pending_outcome_at_once)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    ASSERT_TRUE(!t.a.outcome_due);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
}

TEST(choosing_humidity_again_drops_a_pending_outcome)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 50U);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    ASSERT_TRUE(!t.a.outcome_due);
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 0U);
}

/* A glitch shorter than the limit keeps the window and the hold; nothing starts during it. */
TEST(a_short_glitch_keeps_the_cold_box_and_the_hold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + HOLD_MS - (RIG_MIN_MS / 4U));
    ASSERT_TRUE(t.a.cold && t.a.above);
    uint8_t ring_n = t.a.ring_n;
    t.r.rh_valid = false;
    rig_run_ms(&t.r, ACE2K_DRYER_AUTO_INVALID_MS / 2U); /* the hold runs out during it */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);       /* but nothing starts on a bad reading */
    ASSERT_TRUE(t.a.cold && t.a.above);
    ASSERT_EQ(t.a.ring_n, ring_n);
    t.r.rh_valid = true;
    ASSERT_TRUE(run_until_started(&t, 1000U));
}

/* A glitch of the limit or longer empties the window and ends the hold. */
TEST(a_long_glitch_resets_the_cold_box_and_the_hold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    ASSERT_TRUE(t.a.cold && t.a.above);
    t.r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_run_ms(&t.r, ACE2K_DRYER_AUTO_INVALID_MS - 1000U);
    ASSERT_TRUE(t.a.cold && t.a.above);
    rig_run_ms(&t.r, 2000U);
    ASSERT_TRUE(!t.a.cold && !t.a.above);
    ASSERT_EQ(t.a.ring_n, 0U);
    /* a second glitch times from its own start */
    t.r.chamber_age_ms = 0U;
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    ASSERT_TRUE(t.a.cold);
    t.r.chamber_age_ms = ACE2K_DRYER_CHAMBER_STALE_MS + 1U;
    rig_run_ms(&t.r, ACE2K_DRYER_AUTO_INVALID_MS / 2U);
    ASSERT_TRUE(t.a.cold);
}

/* A glitch that began with the dryer idle is forgotten once a cycle runs. */
TEST(a_cycle_forgets_a_glitch)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    set_rh(&t, 30U);
    rig_run_ms(&t.r, WINDOW_MS + RIG_MIN_MS);
    t.r.rh_valid = false;
    rig_run_ms(&t.r, ACE2K_DRYER_AUTO_INVALID_MS / 2U);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 45U, 1U, t.r.now_ms), ACE2K_DRYER_OK);
    rig_tick(&t.r);
    ASSERT_TRUE(!t.a.invalid);
}

/* With the host, a lower profile lowers a running automatic cycle, never a manual one, and a
 * higher profile never raises one. */
TEST(a_lower_profile_lowers_a_running_own_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 65U, CYCLE_MIN), 0);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(t.r.dryer.target_c, 65U);
    uint32_t end_ms = t.r.dryer.end_ms;
    uint32_t ev_n = t.r.ev_n;
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, CYCLE_MIN), 0);
    ASSERT_EQ(t.r.dryer.target_c, 45U);
    ASSERT_EQ(t.r.dryer.end_ms, end_ms); /* the end stays */
    ASSERT_TRUE(t.a.own);
    rig_tick(&t.r);
    ASSERT_EQ(t.r.ev_n, ev_n); /* no event: the host logs the profile */
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 60U, CYCLE_MIN), 0);
    ASSERT_EQ(t.r.dryer.target_c, 45U); /* never raised */
}

TEST(a_lower_profile_lowers_a_running_bound_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 65U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_STARTING);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 60U, CYCLE_MIN), 0);
    ASSERT_EQ(t.r.dryer.target_c, 60U); /* already while it starts */
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    /* the bound cycle runs above the stored profile: the same profile again still lowers it */
    t.s.profile_c = 45U;
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, CYCLE_MIN), 0);
    ASSERT_EQ(t.r.dryer.target_c, 45U);
    ASSERT_TRUE(t.a.bound);
}

TEST(a_lower_profile_leaves_a_manual_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 65U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, 120U), 0);
    ASSERT_EQ(t.r.dryer.target_c, 65U);
}

TEST(a_lower_profile_leaves_an_own_cycle_that_already_stopped)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 65U, CYCLE_MIN), 0);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK); /* no tick since */
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(ace2k_dryer_auto_profile(&t.a, 45U, CYCLE_MIN), 0);
    ASSERT_EQ(t.r.dryer.target_c, 65U); /* the cool-down's target is not touched */
}

/* The same mode again writes nothing and keeps the hold. */
TEST(the_same_mode_again_changes_nothing)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + (HOLD_MS / 2U));
    ASSERT_TRUE(t.a.above);
    uint32_t since = t.a.above_since_ms;
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    ASSERT_TRUE(!ace2k_dryer_auto_dirty(&t.a));
    ASSERT_TRUE(t.a.above);
    ASSERT_EQ(t.a.above_since_ms, since);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    rig_run_ms(&t.r, RIG_MIN_MS / 2U);
    ASSERT_TRUE(t.a.above);
    since = t.a.above_since_ms;
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    ASSERT_TRUE(!ace2k_dryer_auto_dirty(&t.a));
    ASSERT_EQ(t.a.above_since_ms, since);
}

/* Humidity again with failures counted is the user's re-enable: the count clears. */
TEST(humidity_again_with_failures_clears_them)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    one_cycle(&t, 50U);
    ASSERT_EQ(t.s.failures, 1U);
    ace2k_dryer_auto_stored(&t.a, ace2k_dryer_auto_generation(&t.a));
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    ASSERT_EQ(t.s.failures, 0U);
    ASSERT_TRUE(ace2k_dryer_auto_dirty(&t.a));
}

/* The maximum lowered during an own cycle stops the mode; the cycle's failed outcome then says
 * so no second time. */
TEST(a_failed_outcome_after_the_stop_does_not_say_stopped_again)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    one_cycle(&t, 50U);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_limits(&t.a, 40U, 1U), 0);
    rig_tick(&t.r);
    ASSERT_TRUE((t.s.flags & ACE2K_DRYER_AUTO_STOPPED) != 0U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
    set_rh(&t, 50U);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_COOLDOWN, (CYCLE_MIN + 1U) * RIG_MIN_MS));
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    rig_run_ms(&t.r, WINDOW_MS + (2U * RIG_MIN_MS));
    ASSERT_EQ(t.s.failures, 2U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED), 2U);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_STOPPED), 1U);
}

/* A chamber that warms (the sun, the room heating up) is not a cold box: the rule asks for a
 * steady chamber, neither falling nor rising by COLD_BAND over the window. */
TEST(a_warming_box_is_not_cold)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    set_rh(&t, 60U);
    uint32_t cold_ticks = 0U;
    for (uint32_t ms = 0; ms < 3U * WINDOW_MS; ms += RIG_TICK_MS) {
        t.r.ovr_chamber_mc = COOL_ROOM + (int32_t)(ms / 60U); /* +1 °C a minute */
        rig_tick(&t.r);
        cold_ticks += t.a.cold ? 1U : 0U;
    }
    ASSERT_EQ(cold_ticks, 0U);
    rig_run_ms(&t.r, WINDOW_MS + (RIG_MIN_MS / 2U)); /* steady again: cold */
    ASSERT_TRUE(t.a.cold);
}

/* The print mode's end stops only its own bound cycle. */
TEST(stop_bound_stops_a_bound_cycle_and_keeps_the_mode)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
    ASSERT_TRUE(!t.a.bound);
    rig_tick(&t.r);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(rig_events(&t.r, ACE2K_DRYER_EV_AUTO_OFF), 0U);
    /* in HEATING too */
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_IDLE, 20U * RIG_MIN_MS));
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
}

TEST(stop_bound_leaves_a_manual_cycle)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 5000U));
    /* the user retargets the bound cycle: it is manual now */
    ASSERT_EQ(ace2k_dryer_auto_manual_start(&t.a, 55U, 30U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_HEATING);
    ASSERT_EQ(t.r.dryer.target_c, 55U);
}

TEST(stop_bound_leaves_an_own_cycle_and_an_idle_dryer)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK); /* idle */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    set_rh(&t, 60U);
    ASSERT_TRUE(run_until_started(&t, 4U * RIG_MIN_MS));
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(t.r.dryer.state == ACE2K_DRYER_STARTING || t.r.dryer.state == ACE2K_DRYER_HEATING);
    ASSERT_TRUE(t.a.own);
    ASSERT_EQ(t.s.mode, ACE2K_DRYER_AUTO_HUMIDITY);
}

/* A bound cycle that already stopped, before the tick saw it: nothing to stop, OK. */
TEST(stop_bound_after_the_cycle_stopped_is_ok)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(ace2k_dryer_stop(&t.r.dryer, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(t.a.bound);
    ASSERT_EQ(ace2k_dryer_auto_stop_bound(&t.a, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_COOLDOWN);
}

/* The status the host reads: a glitch shows no cold box and no time above, so the print mode
 * never starts on invalid readings. Inside, the cold box and the hold run on. */
TEST(a_glitch_reads_not_cold_in_the_status_and_the_hold_runs_on)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_PRINT);
    set_rh(&t, 60U);
    rig_run_ms(&t.r, WINDOW_MS + (RIG_MIN_MS / 2U));
    struct ace2k_dryer_auto_status st;
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    uint32_t before_s = st.above_s;
    ASSERT_TRUE(before_s > 0U);
    t.r.rh_valid = false;
    rig_run_ms(&t.r, 10000U);
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(!st.cold);
    ASSERT_EQ(st.above_s, 0U);
    ASSERT_TRUE(t.a.cold && t.a.above); /* kept inside */
    t.r.rh_valid = true;
    rig_tick(&t.r);
    ace2k_dryer_auto_status(&t.a, &st, t.r.now_ms);
    ASSERT_TRUE(st.cold);
    ASSERT_TRUE(st.above_s >= before_s + 10U); /* from where it was, the glitch included */
}

/* A bound start belongs to the print mode: in any other mode the unit refuses it, busy. */
TEST(a_bound_start_outside_the_print_mode_is_refused_busy)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_BUSY);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_HUMIDITY), 0);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_BUSY);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(!t.a.bound);
    ASSERT_EQ(ace2k_dryer_auto_mode(&t.a, ACE2K_DRYER_AUTO_PRINT), 0);
    ASSERT_EQ(ace2k_dryer_auto_start_bound(&t.a, 45U, 1440U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(t.a.bound);
}

/* The hold needs a cold box, and the first cold tick takes a pending outcome. So an outcome is
 * always taken before a start, and the start waits a full hold from that tick. */
TEST(a_pending_outcome_is_taken_before_any_start)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_HUMIDITY);
    cycle_to_idle(&t, 60U);
    ASSERT_TRUE(t.a.outcome_due);
    uint32_t waited_ms = 0U;
    while (rig_events(&t.r, ACE2K_DRYER_EV_AUTO_FAILED) == 0U) {
        ASSERT_TRUE(!t.a.above); /* no hold runs while the outcome waits */
        ASSERT_TRUE(waited_ms < WINDOW_MS + (2U * RIG_MIN_MS));
        rig_tick(&t.r);
        waited_ms += RIG_TICK_MS;
    }
    ASSERT_TRUE(!t.a.outcome_due);
    ASSERT_TRUE(t.a.cold && t.a.above);
    ASSERT_EQ(t.a.above_since_ms, t.r.now_ms); /* the hold starts on the outcome's tick */
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    rig_run_ms(&t.r, HOLD_MS - RIG_TICK_MS);
    ASSERT_EQ(t.r.dryer.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(run_until_started(&t, 2U * RIG_TICK_MS));
}

/* A module bound over a dryer that is already in a cycle sees that cycle end from its first
 * tick on, and takes no outcome for it: the cycle was not its own. */
TEST(a_module_bound_over_a_running_cycle_takes_no_outcome_when_it_ends)
{
    struct auto_rig t;
    auto_init(&t, ACE2K_DRYER_AUTO_MANUAL);
    t.r.auto_ = NULL;
    ASSERT_EQ(ace2k_dryer_start(&t.r.dryer, 45U, 1U, t.r.now_ms), ACE2K_DRYER_OK);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_HEATING, 2U * RIG_MIN_MS));
    t.s.mode = ACE2K_DRYER_AUTO_HUMIDITY;
    ace2k_dryer_auto_init(&t.a, &t.r.dryer, &t.s);
    t.r.auto_ = &t.a;
    rig_tick(&t.r);
    ASSERT_EQ(t.a.prev_state, ACE2K_DRYER_HEATING);
    ASSERT_TRUE(rig_run_until(&t.r, ACE2K_DRYER_COOLDOWN, 2U * RIG_MIN_MS));
    rig_tick(&t.r);
    ASSERT_EQ(t.a.prev_state, ACE2K_DRYER_COOLDOWN);
    ASSERT_TRUE(!t.a.outcome_due && !t.a.own);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
