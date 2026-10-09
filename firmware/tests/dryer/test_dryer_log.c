/* The dryer's log: the counters, the
 * accounting, the ring, the boot's interrupted cycle and persisted cutout, and the dryer over it
 * on a real heat and airflow with do-nothing ops. */
#include "test.h"
#include "heat/airflow.h"
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"
#include "heat/heat.h"
#include "core/util.h"

/* ---- the log alone ---- */

static struct ace2k_dryer_inputs inputs_at(int32_t left_mc, int32_t right_mc, int32_t chamber_mc)
{
    struct ace2k_dryer_inputs in = { 0 };
    in.ntc_left_mc = left_mc;
    in.ntc_right_mc = right_mc;
    in.left_valid = true;
    in.right_valid = true;
    in.chamber_mc = chamber_mc;
    in.chamber_valid = true;
    in.mains_present = true;
    in.mains_plausible = true;
    in.link_ok = true;
    return in;
}

TEST(a_cycle_start_opens_and_counts_and_its_end_closes)
{
    struct ace2k_dryer_log log = { 0 };
    ace2k_dryer_log_cycle_start(&log);
    ASSERT_EQ(log.cycles_started, 1);
    ASSERT_TRUE((log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN) != 0U);
    ace2k_dryer_log_cycle_end(&log, true);
    ASSERT_EQ(log.cycles_completed, 1);
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN, 0);
    ace2k_dryer_log_cycle_start(&log);
    ace2k_dryer_log_cycle_end(&log, false);
    ASSERT_EQ(log.cycles_started, 2);
    ASSERT_EQ(log.cycles_completed, 1);
}

TEST(hundred_seconds_at_fifty_percent_is_hundred_heating_and_fifty_full_power_seconds)
{
    struct ace2k_dryer_log log = { 0 };
    for (uint32_t t = 0; t < 10000U; t++) { /* 10 000 ticks of 10 ms */
        ace2k_dryer_log_account(&log, 50U, 10U);
    }
    ASSERT_EQ(log.heat_s, 100);
    ASSERT_EQ(log.full_power_s, 50);
    ASSERT_EQ(log.heat_ms_frac, 0);
    ASSERT_EQ(log.full_power_ms_frac, 0);
}

TEST(odd_duties_accumulate_without_drift)
{
    struct ace2k_dryer_log log = { 0 };
    for (uint32_t t = 0; t < 30000U; t++) { /* 300 s at 33 % */
        ace2k_dryer_log_account(&log, 33U, 10U);
    }
    ASSERT_EQ(log.heat_s, 300);
    ASSERT_EQ(log.full_power_s, 99);
    ASSERT_EQ(log.full_power_ms_frac, 0);
}

TEST(a_fault_entry_carries_the_cycle_the_heat_and_the_temperatures_in_decidegrees)
{
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_inputs in = inputs_at(61234, 58999, 44950);
    ace2k_dryer_log_cycle_start(&log);
    log.heat_s = 777;
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_SIDES, &in);
    ASSERT_EQ(log.faults, 1);
    ASSERT_EQ(log.entry[0].kind, ACE2K_DRYER_EV_FAULT);
    ASSERT_EQ(log.entry[0].reason, ACE2K_DRYER_FAULT_SIDES);
    ASSERT_EQ(log.entry[0].cycle, 1);
    ASSERT_EQ(log.entry[0].heat_s, 777);
    ASSERT_EQ(log.entry[0].ntc_left_dc, 612);
    ASSERT_EQ(log.entry[0].ntc_right_dc, 589);
    ASSERT_EQ(log.entry[0].chamber_dc, 449);
}

/* An entry names the cycle open at the time; with none open it says 0, never the previous one's. */
TEST(an_entry_with_no_cycle_open_carries_cycle_zero)
{
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_inputs in = inputs_at(30000, 30000, 25000);
    ace2k_dryer_log_cycle_start(&log);
    ace2k_dryer_log_cycle_end(&log, true);
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_FANS, &in);
    ASSERT_EQ(log.entry[0].cycle, 0);
    ASSERT_EQ(log.faults, 1);
}

TEST(an_invalid_or_missing_reading_is_logged_as_unknown)
{
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_inputs in = inputs_at(50000, 50000, 40000);
    in.right_valid = false;
    in.chamber_valid = false;
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_NTC, &in);
    ASSERT_EQ(log.entry[0].ntc_left_dc, 500);
    ASSERT_EQ(log.entry[0].ntc_right_dc, ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC);
    ASSERT_EQ(log.entry[0].chamber_dc, ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC);
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_INTERRUPTED, 0, NULL);
    ASSERT_EQ(log.entry[0].ntc_left_dc, ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC);
    ASSERT_EQ(log.faults, 1); /* interrupted is not a fault */
}

TEST(the_ring_keeps_the_newest_eight_newest_first)
{
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_inputs in = inputs_at(30000, 30000, 25000);
    for (uint8_t reason = 1; reason <= 10U; reason++) {
        ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, reason, &in);
    }
    ASSERT_EQ(log.faults, 10);
    for (uint32_t i = 0; i < ACE2K_DRYER_LOG_ENTRIES; i++) {
        ASSERT_EQ(log.entry[i].reason, 10U - i);
    }
}

TEST(a_cutout_fault_sets_the_persistent_cutout_flag)
{
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_inputs in = inputs_at(30000, 30000, 25000);
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_NTC, &in);
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CUTOUT, 0);
    ace2k_dryer_log_record(&log, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_CUTOUT, &in);
    ASSERT_TRUE((log.flags & ACE2K_DRYER_LOG_CUTOUT) != 0U);
}

TEST(boot_over_an_open_cycle_logs_interrupted_once_and_closes_it)
{
    struct ace2k_dryer_log log = { 0 };
    ace2k_dryer_log_cycle_start(&log);
    ASSERT_EQ(ace2k_dryer_log_boot(&log, false), ACE2K_DRYER_LOG_BOOT_INTERRUPTED);
    ASSERT_EQ(log.entry[0].kind, ACE2K_DRYER_EV_INTERRUPTED);
    ASSERT_EQ(log.entry[0].cycle, 1);
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN, 0);
    ASSERT_EQ(ace2k_dryer_log_boot(&log, false), 0);
}

TEST(boot_keeps_the_cutout_flag_but_a_power_on_reset_clears_it)
{
    struct ace2k_dryer_log log = { 0 };
    log.flags = ACE2K_DRYER_LOG_CUTOUT;
    ASSERT_EQ(ace2k_dryer_log_boot(&log, false), ACE2K_DRYER_LOG_BOOT_CUTOUT);
    ASSERT_TRUE((log.flags & ACE2K_DRYER_LOG_CUTOUT) != 0U);
    ASSERT_EQ(ace2k_dryer_log_boot(&log, true), ACE2K_DRYER_LOG_BOOT_CUTOUT_CLEARED);
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CUTOUT, 0);
    ASSERT_EQ(ace2k_dryer_log_boot(&log, true), 0); /* nothing left to clear */
}

/* ---- the dryer over the log: a real heat and airflow on do-nothing ops ---- */

static void gate_nop(void *ctx)
{
    (void)ctx;
}

static bool gate_low(void *ctx)
{
    (void)ctx;
    return false;
}

static const struct ace2k_heat_ops ace2k_heat_nop_ops = {
    .gate_fire = gate_nop,
    .gate_off = gate_nop,
    .gate_read = gate_low,
};

static bool ace2k_fans_on;
static bool ace2k_fans_fail; /* the fans commanded on never read high */

static void fan_write(void *ctx, bool on)
{
    (void)ctx;
    ace2k_fans_on = on;
}

static uint8_t fan_read(void *ctx)
{
    (void)ctx;
    return (ace2k_fans_on && !ace2k_fans_fail) ? 3U : 0U;
}

static void flap_write(void *ctx, enum ace2k_flap f, bool o, bool c)
{
    (void)ctx;
    (void)f;
    (void)o;
    (void)c;
}

static const struct ace2k_airflow_ops ace2k_log_airflow_ops = {
    .fan_write = fan_write,
    .fan_read = fan_read,
    .flap_write = flap_write,
};

// NOLINTNEXTLINE(readability-identifier-naming)
struct log_rig {
    struct ace2k_heat heat;
    struct ace2k_airflow airflow;
    struct ace2k_dryer dryer;
};

static void rig_boot(struct log_rig *r, struct ace2k_dryer_log *log, bool power_on)
{
    ace2k_fans_on = false;
    ace2k_fans_fail = false;
    ace2k_heat_init(&r->heat, &ace2k_heat_nop_ops, NULL);
    ace2k_airflow_init(&r->airflow, &ace2k_log_airflow_ops, NULL);
    ace2k_dryer_init(&r->dryer, &r->heat, &r->airflow, log, power_on, 0);
}

/* One 10 ms tick in the binding's order: airflow, heat, dryer; cool NTCs, fans reading high. */
static void rig_step(struct log_rig *r, const struct ace2k_dryer_inputs *in, uint32_t t)
{
    const struct ace2k_heat_inputs hi = {
        .ntc_left_mc = in->ntc_left_mc,
        .ntc_right_mc = in->ntc_right_mc,
        .left_valid = true,
        .right_valid = true,
        .mains_present = true,
        .mains_hz10 = 600U,
        .mains_measured = true,
        .cutout = false,
        .fans_read = 3U,
        .fans_commanded = true,
    };
    ace2k_airflow_tick(&r->airflow, in->ntc_left_mc, in->ntc_right_mc, true, true, t);
    ace2k_heat_tick(&r->heat, &hi, t, t * 1000U);
    ace2k_dryer_tick(&r->dryer, in, t);
}

static bool next_event_is(struct ace2k_dryer *d, uint8_t kind)
{
    struct ace2k_dryer_event ev;
    if (!ace2k_dryer_event_next(d, &ev)) {
        return false;
    }
    ace2k_dryer_event_sent(d);
    ace2k_dryer_event_ack(d, ev.seq);
    return ev.kind == kind;
}

/* The condition the unit booted in comes first, ahead of the interrupted cycle the same boot
 * found. */
TEST(a_boot_with_a_cutout_and_an_open_cycle_queues_the_cutout_first)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    log.flags = ACE2K_DRYER_LOG_CUTOUT;
    ace2k_dryer_log_cycle_start(&log);
    rig_boot(&r, &log, false);
    ASSERT_TRUE(next_event_is(&r.dryer, ACE2K_DRYER_EV_FAULT));
    ASSERT_TRUE(next_event_is(&r.dryer, ACE2K_DRYER_EV_INTERRUPTED));
    struct ace2k_dryer_event ev;
    ASSERT_TRUE(!ace2k_dryer_event_next(&r.dryer, &ev));
}

TEST(a_reset_mid_cycle_never_resumes_and_is_logged_interrupted)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_status st;
    ace2k_dryer_log_cycle_start(&log); /* the mark a cycle stored before its first lease */
    rig_boot(&r, &log, false);
    ace2k_dryer_status(&r.dryer, &st, 0U);
    ASSERT_EQ(st.state, ACE2K_DRYER_IDLE);
    ASSERT_TRUE(next_event_is(&r.dryer, ACE2K_DRYER_EV_INTERRUPTED));
    ASSERT_EQ(log.entry[0].kind, ACE2K_DRYER_EV_INTERRUPTED);
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer));
    struct ace2k_dryer_inputs in = inputs_at(26000, 26000, 25000);
    for (uint32_t t = 10; t <= 5000U; t += 10U) {
        ace2k_dryer_tick(&r.dryer, &in, t);
        ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    }
}

TEST(a_persisted_cutout_holds_the_dryer_in_fault_until_a_power_on_reset)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_status st;
    log.flags = ACE2K_DRYER_LOG_CUTOUT;
    rig_boot(&r, &log, false); /* FIRMWARE_RESTART: a software reset */
    ace2k_dryer_status(&r.dryer, &st, 0U);
    ASSERT_EQ(st.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(st.fault, ACE2K_DRYER_FAULT_CUTOUT);
    ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer)); /* nothing changed: nothing to store */
    ASSERT_EQ(ace2k_dryer_clear(&r.dryer, 100), ACE2K_DRYER_CUTOUT);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 50, 60, 100), ACE2K_DRYER_FAULTED);
    rig_boot(&r, &log, true); /* the unit unplugged and plugged back */
    ace2k_dryer_status(&r.dryer, &st, 0U);
    ASSERT_EQ(st.state, ACE2K_DRYER_IDLE);
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CUTOUT, 0);
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer)); /* the cleared flag must reach the page */
}

/* A power-on reset over a log with nothing to clear marks nothing: no page erase per power-on. */
TEST(a_power_on_boot_over_a_clean_log_does_not_mark_it)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    log.cycles_started = 3;
    log.cycles_completed = 3;
    rig_boot(&r, &log, true);
    ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer));
    rig_boot(&r, &log, false);
    ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer));
}

TEST(the_cutout_asserted_while_idle_is_a_cutout_fault_and_persisted)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    struct ace2k_dryer_status st;
    rig_boot(&r, &log, true);
    struct ace2k_dryer_inputs in = inputs_at(26000, 26000, 25000);
    in.cutout = true;
    ace2k_dryer_tick(&r.dryer, &in, 10);
    ace2k_dryer_status(&r.dryer, &st, 0U);
    ASSERT_EQ(st.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(st.fault, ACE2K_DRYER_FAULT_CUTOUT);
    ASSERT_TRUE((log.flags & ACE2K_DRYER_LOG_CUTOUT) != 0U);
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer));
}

TEST(starting_waits_for_the_open_mark_to_be_stored_before_the_first_lease)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    rig_boot(&r, &log, true);
    struct ace2k_dryer_inputs in = inputs_at(26000, 26000, 25000);
    uint32_t t = 10;
    rig_step(&r, &in, t); /* the dryer's first view of the inputs: a start needs one */
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 50, 60, t), ACE2K_DRYER_OK);
    for (t += 10U; t <= 1000U; t += 10U) { /* fans on and read high, flaps pulsed: no lease */
        rig_step(&r, &in, t);
        ASSERT_TRUE(!ace2k_heat_leased(&r.heat));
    }
    ASSERT_TRUE((log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN) != 0U);
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer));
    uint32_t stale = ace2k_dryer_log_generation(&r.dryer) - 1U;
    ace2k_dryer_log_stored(&r.dryer, stale); /* an older store: not this mark */
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer));
    ace2k_dryer_log_stored(&r.dryer, ace2k_dryer_log_generation(&r.dryer));
    ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer));
    bool leased = false;
    for (; t <= 3000U && !leased; t += 10U) {
        rig_step(&r, &in, t);
        leased = ace2k_heat_leased(&r.heat);
    }
    ASSERT_TRUE(leased);
}

/* A stop in STARTING before the open mark leaves the log alone: nothing to close, no store (no
 * page erase for a cycle that never began).  After the mark, a stop closes it. */
TEST(a_stop_in_starting_before_the_cycle_opened_marks_nothing)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    rig_boot(&r, &log, true);
    struct ace2k_dryer_inputs in = inputs_at(26000, 26000, 25000);
    uint32_t t = 10;
    rig_step(&r, &in, t);
    for (uint32_t ticks = 0; ticks < 4U; ticks++) { /* at 0, 1, 2 and 3 ticks into STARTING */
        uint32_t gen = ace2k_dryer_log_generation(&r.dryer);
        ASSERT_EQ(ace2k_dryer_start(&r.dryer, 50, 60, t), ACE2K_DRYER_OK);
        for (uint32_t k = 0; k < ticks; k++) {
            t += 10U;
            rig_step(&r, &in, t);
        }
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_STARTING);
        ASSERT_TRUE(!r.dryer.start_logged);
        ASSERT_EQ(ace2k_dryer_stop(&r.dryer, t), ACE2K_DRYER_OK);
        ASSERT_EQ(ace2k_dryer_log_generation(&r.dryer), gen);
        ASSERT_EQ(log.cycles_started, 0);
        ASSERT_TRUE(!ace2k_dryer_log_dirty(&r.dryer));
        for (uint32_t k = 0; k < 6000U && r.dryer.state != ACE2K_DRYER_IDLE; k++) {
            t += 10U;
            rig_step(&r, &in, t); /* the cool-down: 30 s at 26 °C */
        }
        ASSERT_EQ(r.dryer.state, ACE2K_DRYER_IDLE);
    }
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 50, 60, t), ACE2K_DRYER_OK);
    for (uint32_t k = 0; k < 100U && !r.dryer.start_logged; k++) {
        t += 10U;
        rig_step(&r, &in, t);
    }
    ASSERT_EQ(log.cycles_started, 1);
    ace2k_dryer_log_stored(&r.dryer, ace2k_dryer_log_generation(&r.dryer));
    ASSERT_EQ(ace2k_dryer_stop(&r.dryer, t), ACE2K_DRYER_OK);
    ASSERT_TRUE(ace2k_dryer_log_dirty(&r.dryer));
    ASSERT_EQ(log.flags & ACE2K_DRYER_LOG_CYCLE_OPEN, 0);
}

/* The fans never read high in STARTING: the fault's entry is stamped cycle 0 (the cycle was never
 * opened), not the previous cycle's number, and no cycle is counted. */
TEST(a_starting_fans_fault_is_logged_with_no_cycle)
{
    static struct log_rig r;
    struct ace2k_dryer_log log = { 0 };
    log.cycles_started = 7;
    log.cycles_completed = 7;
    rig_boot(&r, &log, true);
    ace2k_fans_fail = true;
    struct ace2k_dryer_inputs in = inputs_at(26000, 26000, 25000);
    uint32_t t = 10;
    rig_step(&r, &in, t);
    ASSERT_EQ(ace2k_dryer_start(&r.dryer, 50, 60, t), ACE2K_DRYER_OK);
    for (uint32_t k = 0; k < 400U && r.dryer.state != ACE2K_DRYER_FAULT; k++) {
        t += 10U;
        rig_step(&r, &in, t); /* airflow ends the flaps' pulses: STARTING waits for them */
    }
    ASSERT_EQ(r.dryer.state, ACE2K_DRYER_FAULT);
    ASSERT_EQ(r.dryer.fault, ACE2K_DRYER_FAULT_FANS);
    ASSERT_EQ(log.entry[0].kind, ACE2K_DRYER_EV_FAULT);
    ASSERT_EQ(log.entry[0].reason, ACE2K_DRYER_FAULT_FANS);
    ASSERT_EQ(log.entry[0].cycle, 0);
    ASSERT_EQ(log.cycles_started, 7);
    ASSERT_EQ(log.faults, 1);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
