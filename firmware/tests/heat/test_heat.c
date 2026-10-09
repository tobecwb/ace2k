/* heat: every safety rule that heat enforces (the dryer's rules 2, 3, 4 and 5) tried and
 * refused.  The rig simulates 60 Hz mains (an edge every 8 333 µs) and the 10 ms tick on one
 * microsecond clock; the fake gate checks every fire against the inputs the last tick
 * published — the contract: the interrupt acts on the tick's verdict. */
#include "test.h"
#include "ace2k_board/zerocross.h" /* ACE2K_ZEROCROSS_LOCKOUT_US */
#include "heat/heat.h"
#include "heat/mains.h"

#define HALF_CYCLE_US_60 8333U
#define HALF_CYCLE_US_50 10000U
#define TICK_US          10000U
#define PULSE_US         6000U
#define EDGES_MAX        20000U

// NOLINTNEXTLINE(readability-identifier-naming)
struct rig {
    struct ace2k_heat h;
    struct ace2k_heat_inputs in;  /* the world, as the next tick will publish it */
    struct ace2k_heat_inputs pub; /* what the last tick published */
    uint32_t now_us, next_edge_us, next_tick_us, half_us;
    bool ticking; /* false: the tick is dead, edges keep coming */
    bool gate_high, stuck;
    bool pending_at_edge; /* heat held a pending odd half when this edge came */
    /* the mains as it is, dropped edges included: real_edges counts every zero-cross, the
     * detector's misses too; a fire is on the polarity of its real index's parity */
    uint32_t drop_edges; /* the next this many edges are missed by the detector */
    uint32_t real_edges, pos, neg, split_pairs;
    uint32_t last_fire_real;
    bool fired_once;
    uint32_t fire_us;
    uint32_t fires, offs, violations;
    uint32_t edges;
    bool fired_at[EDGES_MAX];
    uint32_t lease_every_ms, lease_ms, lease_next_ms; /* the renewal the test asks for */
    uint8_t lease_duty;
    uint16_t hz10;    /* the frequency the mains measures of the train */
    uint32_t asym_us; /* the input's half-cycles alternate half_us ± asym_us */
    bool resync_next; /* the next edge taken is the zero-cross filter's resync edge */
    /* the board's zero-cross filter, on this clock, and the flash operations it reads */
    struct ace2k_mains_edge_filter zc;
    struct ace2k_mains_stall stall;
};

// NOLINTNEXTLINE(readability-identifier-naming)
static struct rig R;

static bool inputs_bad(const struct ace2k_heat_inputs *in)
{
    if (in->cutout || !in->left_valid || !in->right_valid) {
        return true;
    }
    if (in->ntc_left_mc >= ACE2K_HEAT_NTC_MAX_MC || in->ntc_right_mc >= ACE2K_HEAT_NTC_MAX_MC) {
        return true;
    }
    if (!in->mains_present || !ace2k_mains_hz10_plausible(in->mains_hz10)) {
        return true;
    }
    if (!in->fans_commanded) {
        return true;
    }
    return (in->fans_read & 3U) != 3U;
}

/* A fire is legal on inputs the last tick published good, inside a live lease — or as the one
 * fire outside a lease: the odd half a release left pending (heat.h). */
static bool fire_legal(void)
{
    if (inputs_bad(&R.pub) || R.h.duty_pct > ACE2K_HEAT_DUTY_MAX_PCT) {
        return false;
    }
    if (R.pending_at_edge && R.h.state == ACE2K_HEAT_IDLE) {
        return true;
    }
    if (R.h.state != ACE2K_HEAT_LEASED) {
        return false;
    }
    if (ace2k_time_after(R.now_us / 1000U, R.h.lease_end_ms + 1U)) {
        return false;
    }
    return true;
}

static void fake_fire(void *ctx)
{
    (void)ctx;
    if (!fire_legal()) {
        R.violations++;
    }
    R.gate_high = true;
    R.fire_us = R.now_us;
    R.fires++;
}

static void fake_off(void *ctx)
{
    (void)ctx;
    R.gate_high = false;
    R.offs++;
}

static bool fake_read(void *ctx)
{
    (void)ctx;
    if (R.stuck) {
        return true;
    }
    if (!R.gate_high) {
        return false;
    }
    return (R.now_us - R.fire_us) < PULSE_US;
}

static const struct ace2k_heat_ops ace2k_fake_ops = {
    .gate_fire = fake_fire,
    .gate_off = fake_off,
    .gate_read = fake_read,
};

static void world_ok(struct ace2k_heat_inputs *in)
{
    *in = (struct ace2k_heat_inputs){
        .ntc_left_mc = 25000,
        .ntc_right_mc = 25000,
        .left_valid = true,
        .right_valid = true,
        .mains_present = true,
        .mains_hz10 = 600U,
        .mains_measured = true,
        .cutout = false,
        .fans_read = 3U,
        .fans_commanded = true,
    };
}

static void rig_reset(uint32_t half_us)
{
    R = (struct rig){ 0 };
    world_ok(&R.in);
    R.half_us = half_us;
    R.hz10 = (uint16_t)(ACE2K_HEAT_HZ10_PERIOD_US / (2U * half_us)); /* as the mains measures it */
    R.in.mains_hz10 = R.hz10;
    R.next_edge_us = half_us;
    R.next_tick_us = TICK_US;
    R.ticking = true;
    ace2k_mains_edge_filter_init(&R.zc, ACE2K_ZEROCROSS_LOCKOUT_US, 0U);
    ace2k_heat_init(&R.h, &ace2k_fake_ops, 0);
}

#define HZ10_OFF_BAND 585U /* the bench's dip: one window read 58.5 Hz */

/* The mains measured plausible (the train's own frequency) or off-band. */
static void set_plausible(bool ok)
{
    R.in.mains_hz10 = ok ? R.hz10 : HZ10_OFF_BAND;
}

static void do_tick(void)
{
    R.pub = R.in;
    uint32_t now_ms = R.now_us / 1000U;
    ace2k_heat_tick(&R.h, &R.in, now_ms, R.now_us);
    if (R.lease_every_ms != 0U && ace2k_time_after(now_ms, R.lease_next_ms)) {
        (void)ace2k_heat_lease(&R.h, R.lease_duty, R.lease_ms, now_ms);
        R.lease_next_ms = now_ms + R.lease_every_ms;
    }
}

/* A fire as the odd half of a cycle must land on the real half-cycle right after the previous
 * fire (the even half): anything else pairs two halves of one polarity (split_pairs). */
static void record_polarity(bool odd_half)
{
    if (odd_half && (!R.fired_once || R.real_edges != R.last_fire_real + 1U)) {
        R.split_pairs++;
    }
    if ((R.real_edges & 1U) == 0U) {
        R.pos++;
    } else {
        R.neg++;
    }
    R.last_fire_real = R.real_edges;
    R.fired_once = true;
}

static void do_edge(void)
{
    uint32_t before = R.fires;
    R.pending_at_edge = (R.h.fire_mode == ACE2K_HEAT_FIRE_PENDING);
    bool odd_half = (R.h.fire_mode == ACE2K_HEAT_FIRE_PENDING);
    if (R.h.edge_parity != 0U) {
        odd_half = true;
    }
    if (R.resync_next) {
        R.resync_next = false;
        ace2k_heat_resync(&R.h);
    }
    ace2k_heat_zerocross(&R.h, R.now_us);
    if (R.fires != before) {
        record_polarity(odd_half);
    }
    if (R.edges < EDGES_MAX) {
        R.fired_at[R.edges] = R.fires != before;
    }
    R.edges++;
}

/* A detected edge at R.now_us through the board's zero-cross filter, as the interrupt does it: an
 * accepted edge goes to heat, a resync edge marked as one; a refused one goes nowhere. */
static void filtered_edge(void)
{
    enum ace2k_mains_edge v = ace2k_mains_edge_filter(&R.zc, R.now_us, &R.stall);
    if (v == ACE2K_MAINS_EDGE_ACCEPT || v == ACE2K_MAINS_EDGE_RESYNC) {
        R.resync_next = v == ACE2K_MAINS_EDGE_RESYNC;
        do_edge();
    }
}

/* A zero-cross of the mains, at R.now_us: the next one scheduled, this one passed to the filter
 * unless the detector misses it. */
static void real_edge(void)
{
    R.next_edge_us += R.half_us;
    if (R.asym_us != 0U) { /* the input's half-cycles alternate long and short */
        R.next_edge_us += (R.real_edges & 1U) != 0U ? R.asym_us : 0U - R.asym_us;
    }
    if (R.drop_edges != 0U) {
        R.drop_edges--; /* the detector missed it; the mains did not */
    } else {
        filtered_edge();
    }
    R.real_edges++;
}

/* Runs the clock for us microseconds: edges and ticks in time order. */
static void run_us(uint32_t us)
{
    uint32_t end = R.now_us + us;
    for (;;) {
        uint32_t next = R.next_edge_us;
        bool tick = false;
        if (R.ticking && R.next_tick_us <= R.next_edge_us) {
            tick = true;
        }
        if (tick) {
            next = R.next_tick_us;
        }
        if (next > end) {
            break;
        }
        R.now_us = next;
        if (tick) {
            R.next_tick_us += TICK_US;
            do_tick();
        } else {
            real_edge();
        }
    }
    R.now_us = end;
}

/* One tick with the world as it is, then a lease; returns the lease's result. */
static int start_lease(uint8_t duty, uint32_t ms)
{
    run_us(TICK_US);
    return ace2k_heat_lease(&R.h, duty, ms, R.now_us / 1000U);
}

/* Renew a 2 000 ms lease every 1 000 ms, as the dryer does. */
static void keep_leased(uint8_t duty)
{
    R.lease_duty = duty;
    R.lease_ms = ACE2K_HEAT_LEASE_MAX_MS;
    R.lease_every_ms = 1000U;
    R.lease_next_ms = (R.now_us / 1000U) + 1000U;
}

/* Every fired half-cycle is one of a pair: scanning the edges, a fired edge is followed by a
 * fired edge, and the scan steps over the pair. */
static bool pairs_only(void)
{
    uint32_t n = R.edges < EDGES_MAX ? R.edges : EDGES_MAX;
    uint32_t i = 0;
    while (i < n) {
        if (!R.fired_at[i]) {
            i++;
            continue;
        }
        if (i + 1U >= n || !R.fired_at[i + 1U]) {
            return false;
        }
        i += 2U;
    }
    return true;
}

static uint32_t fires_after(uint32_t edge_index)
{
    uint32_t n = 0;
    uint32_t end = R.edges < EDGES_MAX ? R.edges : EDGES_MAX;
    for (uint32_t i = edge_index; i < end; i++) {
        n += R.fired_at[i] ? 1U : 0U;
    }
    return n;
}

TEST(init_is_idle_with_the_gate_off_and_a_lease_before_any_tick_is_refused)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.offs, 1);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, 0), -ACE2K_EAGAIN);
    run_us(100000U);
    ASSERT_EQ(R.fires, 0);
    ASSERT_EQ(R.violations, 0);
}

TEST(a_lease_out_of_range_is_refused_and_duty_or_ms_zero_releases)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(91, 1000), -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2001, R.now_us / 1000U), -ACE2K_EINVAL);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    uint32_t offs = R.offs;
    ASSERT_EQ(ace2k_heat_lease(&R.h, 0, 2000, R.now_us / 1000U), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.offs, offs); /* a release: any running pulse is TIM7's to end */
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 0, R.now_us / 1000U), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.violations, 0);
}

/* Rule 2: no pulse without both fans commanded on and both reading high. */
TEST(rule2_a_lease_is_refused_with_a_fan_not_reading_high)
{
    rig_reset(HALF_CYCLE_US_60);
    R.in.fans_read = 1U; /* the right fan's pin reads low */
    ASSERT_EQ(start_lease(50, 2000), -ACE2K_EBUSY);
    R.in.fans_read = 3U;
    R.in.fans_commanded = false; /* both read high, but nobody asked: a stray level */
    ASSERT_EQ(start_lease(50, 2000), -ACE2K_EBUSY);
    run_us(200000U);
    ASSERT_EQ(R.fires, 0);
    ASSERT_EQ(R.violations, 0);
}

TEST(rule2_a_fan_dropping_mid_lease_latches_and_nothing_fires_after)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(500000U);
    ASSERT_TRUE(R.fires > 0);
    R.in.fans_read = 2U; /* the left fan's pin reads low */
    uint32_t offs = R.offs;
    run_us(TICK_US); /* the tick that sees it */
    uint32_t edge_seen = R.edges;
    run_us(1000000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_FANS);
    ASSERT_TRUE(R.offs > offs);
    ASSERT_EQ(fires_after(edge_seen), 0);
    ASSERT_EQ(R.violations, 0);
}

/* Rule 3: the gate fires only inside a live lease. */
TEST(rule3_a_lease_not_renewed_ends_idle_not_latched_and_nothing_fires_past_it)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 200), 0);
    uint32_t end_ms = R.h.lease_end_ms;
    run_us(1000000U);
    ASSERT_TRUE(R.fires > 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
    ASSERT_TRUE(!ace2k_time_after(R.fire_us / 1000U, end_ms + 1U));
    ASSERT_EQ(R.violations, 0);
    ASSERT_TRUE(pairs_only());
    ASSERT_EQ(R.violations, 0);
}

TEST(rule3_a_dead_tick_stops_the_gate_within_three_half_cycles)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(300000U);
    ASSERT_TRUE(R.fires > 0); /* firing before the tick dies: the bound below is not trivial */
    R.ticking = false;        /* the timer dispatch is gone; the mains is not */
    uint32_t edge_dead = R.edges;
    run_us(1000000U);
    ASSERT_TRUE(fires_after(edge_dead) <= ACE2K_HEAT_EDGES_PER_TICK_MAX);
    ASSERT_EQ(R.violations, 0);
}

TEST(rule3_the_lease_is_bounded_at_two_seconds)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, ACE2K_HEAT_LEASE_MAX_MS), 0);
    run_us(2100000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    uint32_t fires = R.fires;
    run_us(1000000U);
    ASSERT_EQ(R.fires, fires);
    ASSERT_EQ(R.violations, 0);
}

/* Rule 4: each absolute limit, broken during a live lease. */
struct ace2k_breach {
    const char *name;
    void (*apply)(struct ace2k_heat_inputs *in);
    uint8_t reason;
};

static void ntc_left_over(struct ace2k_heat_inputs *in)
{
    in->ntc_left_mc = 85000;
}
static void ntc_right_over(struct ace2k_heat_inputs *in)
{
    in->ntc_right_mc = 90000;
}
static void ntc_left_invalid(struct ace2k_heat_inputs *in)
{
    in->left_valid = false;
}
static void ntc_right_invalid(struct ace2k_heat_inputs *in)
{
    in->right_valid = false;
}
static void mains_gone(struct ace2k_heat_inputs *in)
{
    in->mains_present = false;
}
static void mains_doubled(struct ace2k_heat_inputs *in)
{
    in->mains_hz10 = 1200U; /* a doubled frequency */
}
static void cutout_tripped(struct ace2k_heat_inputs *in)
{
    in->cutout = true;
}

static const struct ace2k_breach ace2k_breaches[] = {
    { "ntc left at 85 °C", ntc_left_over, ACE2K_HEAT_NTC_OVER },
    { "ntc right at 90 °C", ntc_right_over, ACE2K_HEAT_NTC_OVER },
    { "ntc left invalid", ntc_left_invalid, ACE2K_HEAT_NTC_INVALID },
    { "ntc right invalid", ntc_right_invalid, ACE2K_HEAT_NTC_INVALID },
    { "mains absent", mains_gone, ACE2K_HEAT_MAINS_ABSENT },
    { "mains implausible", mains_doubled, ACE2K_HEAT_MAINS_IMPLAUSIBLE },
    { "cutout asserted", cutout_tripped, ACE2K_HEAT_CUTOUT },
};

TEST(rule4_every_limit_broken_mid_lease_latches_its_reason_and_stops_the_gate)
{
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(ace2k_breaches); i++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(90, 2000), 0);
        keep_leased(90);
        run_us(400000U);
        ASSERT_TRUE(R.fires > 0);
        ace2k_breaches[i].apply(&R.in);
        uint32_t offs = R.offs;
        run_us(TICK_US);
        uint32_t edge_seen = R.edges;
        run_us(2000000U); /* past the mains' bucket (1.5 s); the others latch on their tick */
        if (R.h.state != ACE2K_HEAT_LATCHED || R.h.reason != ace2k_breaches[i].reason) {
            printf("  breach: %s\n", ace2k_breaches[i].name);
        }
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
        ASSERT_EQ(R.h.reason, ace2k_breaches[i].reason);
        ASSERT_TRUE(R.offs > offs);
        ASSERT_EQ(fires_after(edge_seen), 0);
        ASSERT_EQ(R.violations, 0);
    }
    ASSERT_EQ(R.violations, 0);
}

TEST(rule4_each_limit_refuses_a_lease_from_idle)
{
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(ace2k_breaches); i++) {
        rig_reset(HALF_CYCLE_US_60);
        ace2k_breaches[i].apply(&R.in);
        ASSERT_TRUE(start_lease(20, 1000) != 0);
        run_us(200000U);
        ASSERT_EQ(R.fires, 0);
        ASSERT_EQ(R.violations, 0);
    }
    ASSERT_EQ(R.violations, 0);
}

TEST(rule4_a_duty_above_the_cap_found_by_the_tick_latches)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    R.h.duty_pct = 95; /* a corrupted duty: the tick and the interrupt must both refuse it */
    run_us(TICK_US);
    uint32_t edge_seen = R.edges;
    run_us(300000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_DUTY);
    ASSERT_EQ(fires_after(edge_seen), 0);
    ASSERT_EQ(R.violations, 0);
}

TEST(rule4_the_cutout_latches_in_every_state)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(50000U);
    R.in.cutout = true;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_CUTOUT);
    ASSERT_EQ(R.violations, 0);
}

/* Rule 5: the gate is never left high. */
TEST(rule5_a_gate_still_high_after_its_pulse_latches_gate_stuck)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(200000U);
    R.stuck = true;
    /* At 90 % a fire lands on most edges, so a tick soon after the gate sticks still sees a pulse
     * younger than ACE2K_HEAT_GATE_STUCK_US; with renewal stopped, the first skipped cycle (one
     * in ten) leaves a tick facing a pulse older than that. */
    R.lease_every_ms = 0U;
    run_us(300000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_GATE_STUCK);
    ASSERT_EQ(R.violations, 0);
}

TEST(rule5_a_gate_high_with_no_pulse_ever_fired_latches_gate_stuck)
{
    rig_reset(HALF_CYCLE_US_60);
    R.stuck = true;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_GATE_STUCK);
    ASSERT_EQ(R.violations, 0);
}

TEST(rule5_normal_pulses_never_read_as_stuck)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(20000000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
    ASSERT_EQ(R.violations, 0);
}

TEST(the_latch_keeps_its_first_reason)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    R.in.mains_present = false;
    run_us(TICK_US);
    R.in.cutout = true;
    R.in.left_valid = false;
    run_us(5U * TICK_US);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_MAINS_ABSENT);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_ELATCHED);
    ASSERT_EQ(R.violations, 0);
}

TEST(clear_waits_for_the_limits_and_never_clears_the_cutout)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    R.in.ntc_left_mc = 86000;
    run_us(TICK_US);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY);
    R.in.ntc_left_mc = 40000;
    R.in.fans_commanded = false; /* the fans are no limit for a clear */
    R.in.fans_read = 0U;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_clear(&R.h), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
    ASSERT_EQ(R.violations, 0);

    rig_reset(HALF_CYCLE_US_60);
    R.in.cutout = true;
    run_us(TICK_US);
    R.in.cutout = false;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EREFUSED);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.violations, 0);
}

/* Rule 10: a cutout that trips after another latch is still a cutout — heat's own clear and any
 * new lease are refused once it recloses, though the latch keeps its first reason. */
TEST(a_cutout_after_another_latch_is_never_cleared_and_refuses_a_lease)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    R.in.ntc_left_mc = 86000;
    run_us(TICK_US);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER);
    R.in.cutout = true;
    run_us(TICK_US);
    R.in.cutout = false;
    R.in.ntc_left_mc = 40000;
    run_us(TICK_US);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER); /* the first reason stays */
    ASSERT_TRUE(R.h.cutout_seen);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EREFUSED);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_TRUE(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U) != 0);
    ASSERT_EQ(R.violations, 0);
}

/* A cutout seen while idle refuses a lease even with every input back in bounds. */
TEST(a_cutout_seen_refuses_a_lease_even_when_the_latch_is_gone)
{
    rig_reset(HALF_CYCLE_US_60);
    R.in.cutout = true;
    run_us(TICK_US);
    R.in.cutout = false;
    run_us(TICK_US);
    R.h.state = ACE2K_HEAT_IDLE; /* as if the latch were lost: the sticky flag still refuses */
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EREFUSED);
    ASSERT_TRUE(!ace2k_heat_leased(&R.h));
}

/* A release leaves a running pulse to TIM7 (a cut pulse may not make a half-cycle); a latch and
 * an abort drive the gate off at once. */
TEST(a_release_leaves_the_pulse_to_tim7_a_latch_or_an_abort_drives_the_gate_off)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    uint32_t offs = R.offs;
    ace2k_heat_release(&R.h);
    ASSERT_EQ(R.offs, offs);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(start_lease(50, 2000), 0);
    offs = R.offs;
    ace2k_heat_abort(&R.h);
    ASSERT_EQ(R.offs, offs + 1U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(start_lease(50, 2000), 0);
    offs = R.offs;
    R.in.mains_present = false;
    run_us(TICK_US);
    ASSERT_TRUE(R.offs > offs);
    ASSERT_EQ(R.violations, 0);
}

/* Polarity of every fired half-cycle: edge i has the polarity of i's parity. */
static void fired_polarity(uint32_t *pos, uint32_t *neg)
{
    uint32_t n = R.edges < EDGES_MAX ? R.edges : EDGES_MAX;
    *pos = 0U;
    *neg = 0U;
    for (uint32_t i = 0; i < n; i++) {
        if (!R.fired_at[i]) {
            continue;
        }
        if ((i & 1U) == 0U) {
            (*pos)++;
        } else {
            (*neg)++;
        }
    }
}

/* The tick misses `edges` mains edges, then resumes on its own 10 ms grid. */
static void tick_gap(uint32_t edges)
{
    R.ticking = false;
    run_us(edges * R.half_us);
    R.next_tick_us = ((R.now_us / TICK_US) + 1U) * TICK_US;
    R.ticking = true;
}

/* A lagging tick drops edges past ACE2K_HEAT_EDGES_PER_TICK_MAX; the cycle cut by the drop is
 * abandoned, never completed by a later edge of the same polarity, so the load sees no DC. */
TEST(a_lagging_tick_keeps_every_fired_cycle_whole_at_50_hz)
{
    for (uint32_t gap = 2U; gap <= 4U; gap++) {
        rig_reset(HALF_CYCLE_US_50);
        ASSERT_EQ(start_lease(90, 2000), 0);
        keep_leased(90);
        run_us(200000U);
        ASSERT_TRUE(R.fires > 0);
        for (uint32_t k = 0; k < 40U; k++) {
            tick_gap(gap);
            run_us(((k % 5U) + 1U) * R.half_us);
        }
        /* no window cut: renewal stops and the lease runs out, the cycle guard closes the last
         * pair */
        R.lease_every_ms = 0U;
        run_us((ACE2K_HEAT_LEASE_MAX_MS * 1000U) + 100000U);
        uint32_t pos = 0;
        uint32_t neg = 0;
        fired_polarity(&pos, &neg);
        if (!pairs_only() || pos != neg) {
            printf("  gap %u edges: pos=%u neg=%u\n", (unsigned)gap, (unsigned)pos, (unsigned)neg);
        }
        ASSERT_TRUE(pairs_only());
        ASSERT_EQ(pos, neg);
        ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
        ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
        ASSERT_EQ(R.violations, 0);
    }
}

static void duty_run(uint32_t half_us, uint8_t duty)
{
    rig_reset(half_us);
    ASSERT_EQ(start_lease(duty, 2000), 0);
    keep_leased(duty);
    uint32_t first = R.edges;
    /* 1 200 mains cycles, plus one half-cycle so the last cycle's odd half is inside the run */
    run_us((1200U * 2U * half_us) + half_us);
    uint32_t n = R.edges - first;
    uint32_t fired = fires_after(first);
    uint32_t want = n * duty / ACE2K_HEAT_PERCENT;
    uint32_t tol = n / 100U; /* 1 % of the half-cycles */
    ASSERT_TRUE(fired + tol >= want);
    ASSERT_TRUE(fired <= want + tol);
    /* the lease runs out: the cycle guard closes the last pair, wherever the cycles started */
    R.lease_every_ms = 0U;
    run_us((ACE2K_HEAT_LEASE_MAX_MS * 1000U) + 100000U);
    ASSERT_EQ(fires_after(first) % 2U, 0);
    ASSERT_TRUE(pairs_only());
    ASSERT_EQ(R.violations, 0);
}

TEST(full_cycles_only_and_the_duty_within_one_percent_at_60_hz)
{
    duty_run(HALF_CYCLE_US_60, 10);
    duty_run(HALF_CYCLE_US_60, 33);
    duty_run(HALF_CYCLE_US_60, 50);
    duty_run(HALF_CYCLE_US_60, 90);
    ASSERT_EQ(R.violations, 0);
}

TEST(full_cycles_only_and_the_duty_within_one_percent_at_50_hz)
{
    duty_run(HALF_CYCLE_US_50, 10);
    duty_run(HALF_CYCLE_US_50, 50);
    duty_run(HALF_CYCLE_US_50, 90);
    ASSERT_EQ(R.violations, 0);
}

/* Runs up to and including the next mains edge (a tick due before it runs too). */
static void next_edge(void)
{
    run_us(R.next_edge_us - R.now_us);
}

/* Runs edge by edge until the even half of a cycle has just fired; false if none within max. */
static bool until_even_fired(uint32_t max_edges)
{
    for (uint32_t i = 0; i < max_edges; i++) {
        next_edge();
        if (R.h.state == ACE2K_HEAT_LEASED && R.h.edge_parity != 0U && R.h.fire_this_cycle) {
            return true;
        }
    }
    return false;
}

/* A zero-cross missed between the even and the odd edge: the next edge the detector passes is
 * a whole period after the even one, on the same polarity — the odd half never fires on it.
 * Each miss leaves at most the even half, which had already conducted, alone. */
static void dropped_edge_run(uint32_t half_us, uint8_t duty)
{
    rig_reset(half_us);
    ASSERT_EQ(start_lease(duty, 2000), 0);
    keep_leased(duty);
    run_us(200000U);
    uint32_t drops = 0U;
    for (uint32_t k = 0; k < 20U; k++) {
        if (!until_even_fired(400U)) {
            break;
        }
        R.drop_edges = 1U;
        drops++;
        run_us(((k % 7U) + 3U) * half_us);
    }
    run_us(500000U);
    ASSERT_TRUE(drops >= 10U);
    if (R.split_pairs != 0U) {
        printf("  %u us, %u %%: %u split pairs\n", (unsigned)half_us, (unsigned)duty,
               (unsigned)R.split_pairs);
    }
    ASSERT_EQ(R.split_pairs, 0U);
    uint32_t lone = R.pos > R.neg ? R.pos - R.neg : R.neg - R.pos;
    ASSERT_TRUE(lone <= drops);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
    ASSERT_EQ(R.violations, 0);
}

TEST(a_missed_zero_cross_between_the_halves_never_pairs_one_polarity)
{
    static const uint8_t duties[] = { 10U, 33U, 50U, 90U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(duties); i++) {
        dropped_edge_run(HALF_CYCLE_US_60, duties[i]);
        dropped_edge_run(HALF_CYCLE_US_50, duties[i]);
    }
    ASSERT_EQ(R.violations, 0);
}

TEST(without_a_missed_edge_the_polarities_balance_exactly)
{
    rig_reset(HALF_CYCLE_US_50);
    ASSERT_EQ(start_lease(33, 2000), 0);
    keep_leased(33);
    run_us(3000000U);
    R.lease_every_ms = 0U;
    run_us(3000000U);
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.pos, R.neg);
    ASSERT_TRUE(R.pos > 0U);
}

/* Q6: an outlet NTC reading invalid for 1–4 ticks stops the firing at once, latches nothing, and
 * the firing resumes inside the live lease when the reading comes back. */
TEST(an_invalid_ntc_for_under_five_ticks_stops_firing_without_a_latch)
{
    for (uint32_t k = 1U; k < ACE2K_HEAT_NTC_INVALID_TICKS; k++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(90, 2000), 0);
        keep_leased(90);
        run_us(400000U);
        ASSERT_TRUE(R.fires > 0);
        R.in.left_valid = false;
        run_us(TICK_US); /* the tick that sees it */
        uint32_t edge_seen = R.edges;
        run_us((k - 1U) * TICK_US);
        R.in.left_valid = true;
        run_us(TICK_US); /* the tick that sees it back */
        uint32_t edge_back = R.edges;
        ASSERT_EQ(fires_after(edge_seen) - fires_after(edge_back), 0U);
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
        ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
        run_us(500000U);
        ASSERT_TRUE(fires_after(edge_back) > 0U);
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
        ASSERT_EQ(R.split_pairs, 0U);
        ASSERT_EQ(R.violations, 0);
    }
}

/* A tick whose verdict is not ok silences the interrupt and resets the cycle it was in: a lease
 * resumed after an NTC glitch starts a fresh cycle — it never inherits a half from before. */
TEST(a_glitch_between_the_halves_leaves_no_stale_half_for_the_resumed_lease)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    keep_leased(50);
    run_us(100000U);
    ASSERT_TRUE(until_even_fired(100U));
    R.in.left_valid = false;
    do_tick(); /* the tick that sees it, before the odd edge */
    ASSERT_EQ(R.h.fire_mode, ACE2K_HEAT_FIRE_NONE);
    ASSERT_EQ(R.h.edge_parity, 0U);
    ASSERT_TRUE(!R.h.fire_this_cycle);
    R.in.left_valid = true;
    run_us(500000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.violations, 0);
}

TEST(an_invalid_ntc_for_five_ticks_latches_ntc_invalid)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    R.in.right_valid = false;
    run_us(TICK_US);
    uint32_t edge_seen = R.edges;
    run_us((ACE2K_HEAT_NTC_INVALID_TICKS - 2U) * TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED); /* four ticks: still riding it out */
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_INVALID);
    R.in.right_valid = true;
    run_us(500000U);
    ASSERT_EQ(fires_after(edge_seen), 0U);
    ASSERT_EQ(R.violations, 0);
}

TEST(an_over_temperature_and_a_fan_latch_on_their_first_tick_even_with_an_ntc_invalid)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    R.in.ntc_left_mc = ACE2K_HEAT_NTC_MAX_MC;
    R.in.right_valid = false;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER);

    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    R.in.fans_read = 1U;
    R.in.left_valid = false;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_FANS);
    ASSERT_EQ(R.violations, 0);
}

/* A renewal during the excursion is taken (the dryer renews every 500 ms); from IDLE it is not. */
TEST(a_live_lease_is_renewed_through_an_ntc_excursion_but_not_started_in_one)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    R.in.left_valid = false;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ace2k_heat_release(&R.h);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    ASSERT_EQ(R.violations, 0);
}

/* Q7: a release between the two edges of a fired cycle lets its odd half fire, then nothing. */
TEST(a_release_between_the_halves_fires_the_odd_half_then_nothing)
{
    for (uint32_t way = 0U; way < 2U; way++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(50, 2000), 0);
        run_us(100000U);
        ASSERT_TRUE(until_even_fired(100U));
        if (way == 0U) {
            ace2k_heat_release(&R.h);
        } else {
            ASSERT_EQ(ace2k_heat_lease(&R.h, 0, 2000, R.now_us / 1000U), 0); /* duty 0 */
        }
        ASSERT_TRUE((R.h.fire_mode == ACE2K_HEAT_FIRE_PENDING));
        ASSERT_TRUE(ace2k_heat_busy(&R.h));
        uint32_t edge = R.edges;
        next_edge();
        ASSERT_TRUE(R.fired_at[edge]);
        run_us(500000U);
        ASSERT_EQ(fires_after(edge), 1U);
        ASSERT_TRUE((R.h.fire_mode != ACE2K_HEAT_FIRE_PENDING));
        ASSERT_TRUE(pairs_only());
        ASSERT_EQ(R.split_pairs, 0U);
        ASSERT_EQ(R.pos, R.neg);
        ASSERT_TRUE(!ace2k_heat_busy(&R.h));
        ASSERT_EQ(R.violations, 0);
    }
}

/* A release with no even half fired — between cycles, or on a cycle the accumulator skipped —
 * keeps no request: the release is immediate, nothing fires after it. */
TEST(a_release_with_no_even_half_fired_is_immediate)
{
    uint32_t tried = 0U;
    for (uint32_t k = 0; k < 40U; k++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(50, 2000), 0);
        run_us(100000U + (k * 1000U));
        if (R.h.edge_parity != 0U && R.h.fire_this_cycle) {
            continue; /* a half fired: the other test's case */
        }
        tried++;
        ace2k_heat_release(&R.h);
        ASSERT_TRUE((R.h.fire_mode != ACE2K_HEAT_FIRE_PENDING));
        uint32_t edge = R.edges;
        run_us(500000U);
        ASSERT_EQ(fires_after(edge), 0U);
        ASSERT_TRUE(pairs_only());
        ASSERT_EQ(R.violations, 0);
    }
    ASSERT_TRUE(tried >= 10U);
}

TEST(a_release_with_the_verdict_not_ok_leaves_nothing_pending)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    ASSERT_TRUE(until_even_fired(100U));
    ace2k_heat_release(&R.h);
    R.in.fans_read = 1U; /* a fan stops reading high: the tick publishes a verdict not ok */
    do_tick();
    uint32_t edge = R.edges;
    run_us(500000U);
    ASSERT_EQ(fires_after(edge), 0U);
    ASSERT_TRUE((R.h.fire_mode != ACE2K_HEAT_FIRE_PENDING));
    ASSERT_EQ(R.violations, 0);
}

TEST(a_latch_or_a_shutdown_between_the_halves_fires_nothing_more)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    ASSERT_TRUE(until_even_fired(100U));
    R.in.mains_present = false;
    do_tick(); /* the latch */
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    uint32_t edge = R.edges;
    run_us(500000U);
    ASSERT_EQ(fires_after(edge), 0U);
    ASSERT_EQ(R.violations, 0);

    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    keep_leased(50);
    run_us(100000U);
    ASSERT_TRUE(until_even_fired(100U));
    ace2k_heat_shutdown(&R.h);
    ASSERT_TRUE((R.h.fire_mode != ACE2K_HEAT_FIRE_PENDING));
    ASSERT_TRUE(!ace2k_heat_leased(&R.h));
    edge = R.edges;
    run_us(3000000U); /* keep_leased keeps asking: every lease refused */
    ASSERT_EQ(fires_after(edge), 0U);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EREFUSED);
    ASSERT_TRUE(!ace2k_heat_leased(&R.h));
    ASSERT_EQ(R.violations, 0);
}

/* Rule 13's interlock: while a store is in progress no lease starts. */
TEST(the_flash_inhibit_refuses_a_new_lease_until_it_is_lifted)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    ASSERT_TRUE(!ace2k_heat_busy(&R.h));
    ace2k_heat_set_inhibit(&R.h, true);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    run_us(100000U);
    ASSERT_EQ(R.fires, 0U);
    ace2k_heat_set_inhibit(&R.h, false);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ASSERT_TRUE(ace2k_heat_busy(&R.h));
    ASSERT_EQ(R.violations, 0);
}

/* A gate stuck high latches GATE_STUCK with the pulse check still open (the gate never reads
 * low).  The latch commanded the gate off for good, so heat is no longer busy and the GATE_STUCK
 * log entry may reach the flash (rule 13); but the gate still reads high — heat on — so heat
 * stays active (the bootloader refused, the fans held: rules 12, 2) and a clear is refused until
 * the gate reads low. */
TEST(a_latched_gate_stuck_high_frees_the_flash_but_holds_the_bootloader_and_refuses_a_clear)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(200000U);
    R.stuck = true;
    R.lease_every_ms = 0U;
    run_us(300000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_GATE_STUCK);
    ASSERT_TRUE(fake_read(0)); /* still high */
    ASSERT_TRUE(!ace2k_heat_busy(&R.h));
    ASSERT_TRUE(ace2k_heat_active(&R.h)); /* the bootloader and the fans still see it */
    run_us(1000000U);
    ASSERT_TRUE(!ace2k_heat_busy(&R.h));
    ASSERT_TRUE(ace2k_heat_active(&R.h));
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY); /* every limit in bounds, the gate high */
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    R.stuck = false;
    ASSERT_TRUE(!ace2k_heat_active(&R.h));
    ASSERT_EQ(ace2k_heat_clear(&R.h), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.violations, 0);

    /* the same after a shutdown with a pulse just fired and a gate that never falls */
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    ASSERT_TRUE(until_even_fired(100U));
    R.stuck = true;
    ace2k_heat_shutdown(&R.h);
    ASSERT_TRUE(!ace2k_heat_busy(&R.h));
    ASSERT_TRUE(ace2k_heat_active(&R.h));
    ASSERT_EQ(R.violations, 0);
}

/* One tick with the left NTC valid or not. */
static void ntc_tick(bool valid)
{
    R.in.left_valid = valid;
    run_us(TICK_US);
}

/* A flapping NTC — invalid n ticks, valid one, over and over — fills the bucket (+4 an invalid
 * tick, −1 a valid one, full at 20) and latches NTC_INVALID on a bounded tick: 4/1 on the 7th, 3/1
 * on the 7th, 2/1 on the 8th, 1/1 on the 13th.  It never rides the lease out firing on the valid
 * ticks. */
TEST(a_flapping_ntc_latches_ntc_invalid_within_a_bounded_time)
{
    static const uint32_t invalid_run[] = { 4U, 3U, 2U, 1U };
    static const uint32_t latch_tick[] = { 7U, 7U, 8U, 13U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(invalid_run); i++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(90, 2000), 0);
        keep_leased(90);
        run_us(400000U);
        uint32_t ticks = 0U;
        while (R.h.state == ACE2K_HEAT_LEASED && ticks < 1000U) {
            ntc_tick((ticks % (invalid_run[i] + 1U)) == invalid_run[i]);
            ticks++;
        }
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
        ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_INVALID);
        ASSERT_EQ(ticks, latch_tick[i]);
        ASSERT_EQ(R.violations, 0);
    }
}

/* A rare glitch — one invalid tick in ten — drains between glitches and never latches; the
 * firing resumes after each. */
TEST(a_rare_ntc_glitch_never_fills_the_bucket)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    uint32_t fires = R.fires;
    for (uint32_t t = 0; t < 1000U; t++) {
        ntc_tick((t % 10U) != 0U);
    }
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
    ASSERT_TRUE(R.fires > fires);
    ASSERT_EQ(R.violations, 0);
}

/* The mains read off-band for less than its bucket — one tick, 50 ticks, one whole
 * 1 s measurement window (101 ticks, the bench's case) — stops the firing on the tick that sees
 * it, latches nothing, and the firing resumes inside the live lease. */
TEST(a_mains_implausible_dip_under_its_bucket_stops_firing_without_a_latch)
{
    static const uint32_t dip_ticks[] = { 1U, 50U, 101U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(dip_ticks); i++) {
        rig_reset(HALF_CYCLE_US_60);
        ASSERT_EQ(start_lease(90, 2000), 0);
        keep_leased(90);
        run_us(400000U);
        ASSERT_TRUE(R.fires > 0);
        set_plausible(false);
        run_us(TICK_US); /* the tick that sees it */
        uint32_t edge_seen = R.edges;
        run_us((dip_ticks[i] - 1U) * TICK_US);
        set_plausible(true);
        run_us(TICK_US); /* the tick that sees it back */
        uint32_t edge_back = R.edges;
        ASSERT_EQ(fires_after(edge_seen) - fires_after(edge_back), 0U);
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
        ASSERT_EQ(R.h.reason, ACE2K_HEAT_OK);
        run_us(500000U);
        ASSERT_TRUE(fires_after(edge_back) > 0U);
        ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
        ASSERT_EQ(R.split_pairs, 0U);
        ASSERT_EQ(R.violations, 0);
    }
}

/* Off-band on every tick: the lease latches MAINS_IMPLAUSIBLE on the threshold's tick, not one
 * before, and nothing fires from the first tick that saw it. */
TEST(a_mains_implausible_for_the_threshold_latches_mains_implausible)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    set_plausible(false);
    run_us(TICK_US);
    uint32_t edge_seen = R.edges;
    run_us((ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS - 2U) * TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED); /* one tick short: still riding it out */
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    set_plausible(true);
    run_us(500000U);
    ASSERT_EQ(fires_after(edge_seen), 0U);
    ASSERT_EQ(R.violations, 0);
}

/* A mains off-band one window in two, sustained, never rides the lease out: the bucket latches
 * it in its second off-band window. */
TEST(a_mains_off_band_every_other_window_latches_in_the_second)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    uint32_t ticks = 0U;
    while (R.h.state == ACE2K_HEAT_LEASED && ticks < 1000U) {
        set_plausible(((ticks / 100U) % 2U) != 0U);
        run_us(TICK_US);
        ticks++;
    }
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    ASSERT_TRUE(ticks > 200U && ticks <= 300U); /* inside the second off-band window */
    ASSERT_EQ(R.violations, 0);
}

/* From idle: a lease asked during a dip is refused as transient (-ACE2K_EAGAIN), latches
 * nothing and is granted once the mains reads plausible; a mains off-band past the bucket is
 * refused as out of bounds (-ACE2K_EBUSY), still with no latch — heat latches only a lease. */
TEST(a_lease_from_idle_during_a_mains_dip_is_refused_as_transient)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(100000U);
    set_plausible(false);
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    run_us(49U * TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    set_plausible(true);
    run_us(TICK_US);
    /* plausible again, the bucket (100) still above the restart level: a new lease waits */
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    while (!ace2k_heat_mains_recovered(&R.h)) {
        run_us(TICK_US);
    }
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ace2k_heat_release(&R.h);
    run_us(500000U); /* the bucket drained */

    set_plausible(false);
    run_us(ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * TICK_US);
    ASSERT_TRUE(ace2k_heat_mains_implausible_held(&R.h));
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EBUSY);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    set_plausible(true);
    run_us(TICK_US);
    ASSERT_TRUE(!ace2k_heat_mains_implausible_held(&R.h));
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
    while (!ace2k_heat_mains_recovered(&R.h)) {
        run_us(TICK_US);
    }
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    ASSERT_EQ(R.violations, 0);
}

/* The mains absent stays immediate, and a fan or an over-temperature is never held back by the
 * mains' debounce. */
TEST(the_mains_absent_a_fan_and_an_over_temperature_latch_at_once_through_a_mains_dip)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    set_plausible(false);
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    R.in.mains_present = false;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_MAINS_ABSENT);

    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    set_plausible(false);
    R.in.fans_read = 1U;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_FANS);

    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    run_us(100000U);
    set_plausible(false);
    R.in.ntc_right_mc = ACE2K_HEAT_NTC_MAX_MC;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER);
    ASSERT_EQ(R.violations, 0);
}

/* A noise edge the board's lockout let through gap_us after the edge before it, in place of the
 * next real zero-cross — which falls inside the 7 ms lockout after it and is refused, as on the
 * board.  The noise edge takes the real edge's index for the polarity bookkeeping. */
static void noise_edge(uint32_t gap_us)
{
    uint32_t prev = R.next_edge_us - R.half_us;
    run_us(prev + gap_us - R.now_us);
    do_edge();
    R.drop_edges = 1U;
}

/* Round 2, items 1–2: at 50 and 60 Hz, a noise edge 7.2 ms — and just past the 7 ms lockout —
 * after the edge before it, at a cycle's even edge: it fires nothing and starts no cycle, and the
 * run keeps whole cycles only (no split pair, the polarities equal). */
static void noise_at_even_run(uint32_t half_us, uint32_t gap_us)
{
    rig_reset(half_us);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(200000U);
    uint32_t noise_fires = 0U;
    for (uint32_t k = 0; k < 20U; k++) {
        next_edge();
        while (R.h.edge_parity != 0U) {
            next_edge();
        }
        uint32_t fires = R.fires;
        noise_edge(gap_us);
        noise_fires += R.fires - fires;
        ASSERT_EQ(R.h.edge_parity, 0U); /* not counted as an even edge */
        run_us(5U * half_us);
    }
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    R.lease_every_ms = 0U; /* the lease runs out: the cycle guard ends it on a whole cycle */
    run_us(3000000U);
    ASSERT_EQ(noise_fires, 0U);
    ASSERT_TRUE(R.h.phase_rejects >= 20U); /* every noise edge counted */
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.pos, R.neg);
    ASSERT_TRUE(R.pos > 0U);
    ASSERT_EQ(R.violations, 0);
}

TEST(a_noise_edge_at_an_even_edge_fires_nothing_at_50_and_60_hz)
{
    static const uint32_t gaps[] = { 7200U, 7001U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(gaps); i++) {
        noise_at_even_run(HALF_CYCLE_US_60, gaps[i]);
        noise_at_even_run(HALF_CYCLE_US_50, gaps[i]);
    }
}

/* The same noise at a cycle's odd edge: the odd half does not fire on it — the real zero-cross is
 * lost to the lockout, so the even half that already fired stays alone, as with a missed edge (at
 * most one lone half per noise edge), and never pairs with a half of its own polarity. */
static void noise_at_odd_run(uint32_t half_us, uint32_t gap_us)
{
    rig_reset(half_us);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(200000U);
    uint32_t noise_fires = 0U;
    uint32_t events = 0U;
    for (uint32_t k = 0; k < 20U; k++) {
        if (!until_even_fired(400U)) {
            break;
        }
        uint32_t fires = R.fires;
        noise_edge(gap_us);
        noise_fires += R.fires - fires;
        events++;
        run_us(5U * half_us);
    }
    run_us(500000U);
    ASSERT_TRUE(events >= 10U);
    ASSERT_EQ(noise_fires, 0U);
    ASSERT_TRUE(R.h.phase_rejects >= 20U); /* every noise edge counted */
    ASSERT_EQ(R.split_pairs, 0U);
    uint32_t lone = R.pos > R.neg ? R.pos - R.neg : R.neg - R.pos;
    ASSERT_TRUE(lone <= events);
    ASSERT_EQ(R.violations, 0);
}

TEST(a_noise_edge_at_an_odd_edge_fires_nothing_at_50_and_60_hz)
{
    static const uint32_t gaps[] = { 7200U, 7001U };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(gaps); i++) {
        noise_at_odd_run(HALF_CYCLE_US_60, gaps[i]);
        noise_at_odd_run(HALF_CYCLE_US_50, gaps[i]);
    }
}

/* Edges off the measured period by more than the tolerance fire nothing: they are judged against
 * the measured frequency, not a fixed window. */
TEST(edges_off_the_measured_period_fire_nothing)
{
    rig_reset(HALF_CYCLE_US_50);
    R.in.mains_hz10 = 600U; /* 60 Hz measured, 50 Hz edges: 1 667 µs off */
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(500000U);
    ASSERT_EQ(R.fires, 0U);
    ASSERT_EQ(R.violations, 0);
}

/* The first edge heat ever sees has no edge before it: it starts no cycle. */
TEST(the_first_edge_ever_starts_no_cycle)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    ace2k_heat_zerocross(&R.h, R.now_us);
    ASSERT_EQ(R.fires, 0U);
    ASSERT_EQ(R.h.edge_parity, 0U);
}

/* Round 3, items 6–7 and 3: a clear judges the mains on its bucket, like a start: refused while
 * the bucket is above ACE2K_HEAT_MAINS_RESTART_LEVEL — even with the mains plausible again —
 * granted at or below it, a dip then included; the clear never resets the bucket, and the next
 * lease still has the restart level's tolerance: at least 75 implausible ticks before it latches. */
TEST(a_mains_latch_clears_only_at_the_restart_level_and_keeps_its_bucket)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    set_plausible(false);
    run_us(ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    R.lease_every_ms = 0U;
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY); /* full */
    set_plausible(true);
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY); /* plausible again, the bucket still high */
    ASSERT_EQ(ace2k_heat_clear_blocker(&R.h), ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    while (R.h.mains_level > ACE2K_HEAT_MAINS_RESTART_LEVEL + 1U) {
        run_us(TICK_US);
    }
    set_plausible(false); /* a dip at the clear: +2, over the restart level */
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY);
    set_plausible(true);
    run_us(5U * TICK_US);
    set_plausible(false); /* a dip once at or below it */
    run_us(TICK_US);
    ASSERT_TRUE(R.h.mains_level <= ACE2K_HEAT_MAINS_RESTART_LEVEL);
    uint16_t level = R.h.mains_level;
    ASSERT_EQ(ace2k_heat_clear(&R.h), 0);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    ASSERT_EQ(R.h.mains_level, level); /* not reset by the clear */
    set_plausible(true);
    ASSERT_EQ(start_lease(90, 2000), 0);
    set_plausible(false);
    uint32_t ticks = 0U;
    while (R.h.state == ACE2K_HEAT_LEASED && ticks < 200U) {
        run_us(TICK_US);
        ticks++;
    }
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_TRUE(ticks >= 75U); /* the restart level's tolerance, about 0.75 window */
    ASSERT_TRUE(ticks < ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS);
    ASSERT_EQ(R.violations, 0);
}

/* Round 3, item 1: an asymmetric input — the half-cycles alternating half ± 450 µs, 900 µs
 * apart, beyond any half-period window — still fires normally: the period between edges of one
 * polarity is exact, so no edge is judged out of phase. */
static void asymmetric_run(uint32_t half_us)
{
    rig_reset(half_us);
    R.asym_us = 450U;
    ASSERT_EQ(start_lease(50, 2000), 0);
    keep_leased(50);
    run_us(3000000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    R.lease_every_ms = 0U;
    run_us(3000000U);
    ASSERT_TRUE(R.fires > 100U);
    ASSERT_EQ(R.h.phase_rejects, 0U);
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.pos, R.neg);
    ASSERT_EQ(R.violations, 0);
}

TEST(an_asymmetric_input_fires_normally_at_50_and_60_hz)
{
    asymmetric_run(HALF_CYCLE_US_60);
    asymmetric_run(HALF_CYCLE_US_50);
}

/* Round 1, item 6: an invalid NTC is reported ahead of an implausible mains — the verdict, the
 * limits and a lease's refusal name the sensor, never the dip. */
TEST(an_invalid_ntc_is_reported_ahead_of_an_implausible_mains)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(100000U);
    set_plausible(false);
    R.in.left_valid = false;
    run_us(TICK_US);
    ASSERT_EQ(R.h.limits, ACE2K_HEAT_NTC_INVALID);
    ASSERT_EQ(R.h.verdict, ACE2K_HEAT_NTC_INVALID);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);

    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(400000U);
    set_plausible(false);
    R.in.left_valid = false;
    run_us(ACE2K_HEAT_NTC_INVALID_TICKS * TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_INVALID);
    ASSERT_EQ(R.violations, 0);
}

/* Round 4, items 1 and 3: a lease from IDLE waits for the restart level; a cycle started right at
 * it rides out one whole 101-tick off-band window without a latch. */
TEST(a_lease_at_the_restart_level_rides_out_one_whole_window)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(100000U);
    set_plausible(false);
    run_us(ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * TICK_US); /* full */
    set_plausible(true);
    run_us(TICK_US);
    while (R.h.mains_level > ACE2K_HEAT_MAINS_RESTART_LEVEL) {
        ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), -ACE2K_EAGAIN);
        run_us(TICK_US);
    }
    ASSERT_EQ(R.h.mains_level, ACE2K_HEAT_MAINS_RESTART_LEVEL);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
    keep_leased(50);
    set_plausible(false);
    run_us(ACE2K_HEAT_MAINS_WINDOW_TICKS * TICK_US);
    set_plausible(true);
    run_us(500000U);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.violations, 0);
}

/* Round 4, item 2: the gate reading high ranks first — the heater may be on — ahead of an
 * over-temperature; the cutout still refuses the clear outright. */
TEST(a_gate_reading_high_ranks_first_among_what_holds_a_latch)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    run_us(100000U);
    R.in.ntc_left_mc = ACE2K_HEAT_NTC_MAX_MC;
    run_us(TICK_US);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_OVER);
    R.stuck = true;
    ASSERT_EQ(ace2k_heat_clear_blocker(&R.h), ACE2K_HEAT_GATE_STUCK);
    ASSERT_EQ(ace2k_heat_clear(&R.h), -ACE2K_EBUSY);
    R.stuck = false;
    ASSERT_EQ(ace2k_heat_clear_blocker(&R.h), ACE2K_HEAT_NTC_OVER);
}

/* Round 4, item 4: before the first window is measured (hz10 0, the boot) the bucket does not
 * fill: a lease 1.2 s after boot on a clean line is granted. */
TEST(no_window_measured_yet_does_not_fill_the_bucket)
{
    rig_reset(HALF_CYCLE_US_60);
    R.in.mains_hz10 = 0U;
    run_us(1000000U);
    ASSERT_EQ(R.h.mains_level, 0U);
    set_plausible(true);
    run_us(200000U);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 50, 2000, R.now_us / 1000U), 0);
}

/* Round 4, item 5: a latch for another cause is never held by the mains bucket — an NTC-invalid
 * latch clears once the NTC reads again, the bucket at 200 notwithstanding. */
TEST(the_mains_bucket_holds_only_a_mains_latch)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    keep_leased(50);
    run_us(100000U);
    set_plausible(false);
    run_us(100U * TICK_US); /* the bucket at 200: a dip, no latch */
    R.in.left_valid = false;
    run_us(ACE2K_HEAT_NTC_INVALID_TICKS * TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    ASSERT_EQ(R.h.reason, ACE2K_HEAT_NTC_INVALID);
    R.lease_every_ms = 0U;
    R.in.left_valid = true;
    set_plausible(true);
    run_us(TICK_US);
    ASSERT_TRUE(R.h.mains_level > ACE2K_HEAT_MAINS_RESTART_LEVEL);
    ASSERT_EQ(ace2k_heat_clear_blocker(&R.h), ACE2K_HEAT_OK);
    ASSERT_EQ(ace2k_heat_clear(&R.h), 0);
    ASSERT_EQ(R.violations, 0);
}

/* A flash operation: interrupts off for `us`.  Every zero-cross in it is lost to the detector but
 * one — the pending bit holds it, and it is taken, late, when the stall ends, with the stall's end
 * as its time — and no tick runs (the next one is the catch-up).  The operation's record moves as
 * flash.c moves it (its pending bit held or not); the late edge and the ones after go through the
 * zero-cross filter, which tells the late one (a resync edge) and refuses a real edge inside its
 * lockout.  Returns the late edge's index in fired_at, or EDGES_MAX when no edge fell in the
 * stall. */
static uint32_t stall_us(uint32_t us)
{
    uint32_t start = R.now_us;
    uint32_t end = R.now_us + us;
    bool pended = false;
    while (R.next_edge_us <= end) {
        R.next_edge_us += R.half_us;
        R.real_edges++;
        pended = true;
    }
    while (R.next_tick_us <= end) {
        R.next_tick_us += TICK_US;
    }
    R.now_us = end;
    R.stall.count++;
    R.stall.start = start;
    R.stall.end = end;
    R.stall.held = pended;
    if (!pended) {
        return EDGES_MAX;
    }
    uint32_t late = R.edges;
    filtered_edge();
    return late;
}

/* The first edge fired at or after `from`, or EDGES_MAX. */
static uint32_t first_fire_from(uint32_t from)
{
    for (uint32_t i = from; i < R.edges && i < EDGES_MAX; i++) {
        if (R.fired_at[i]) {
            return i;
        }
    }
    return EDGES_MAX;
}

/* Defence in depth: a flash stall while leased.  No store can run then — every config
 * writer goes through the config binding's store, which heat's veto refuses while a lease is held
 * or a release pending (rule 13) — so this is not a path of today's firmware; the case pins down
 * what heat would do were that veto ever bypassed.  The edge taken late at the stall's end, and
 * the edges after it, are not judged against the edges before the stall: no phase reject.  The
 * phase history restarts as at boot — the late edge kept out of it, two edges to fill it, a third
 * judged, a fourth the first that may start a cycle — so the firing resumes no earlier than that,
 * and in pairs. */
TEST(edges_after_a_flash_stall_restart_the_phase_history_without_rejects)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(500000U);
    uint32_t before = R.fires;
    ASSERT_TRUE(before > 0U);
    ASSERT_EQ(R.h.phase_rejects, 0U);
    uint32_t late = stall_us(40000U);
    ASSERT_TRUE(late != EDGES_MAX);
    run_us(1000000U);
    ASSERT_EQ(R.h.phase_rejects, 0U);
    ASSERT_TRUE(first_fire_from(late) >= late + 4U);
    ASSERT_TRUE(first_fire_from(late) != EDGES_MAX); /* the firing resumes */
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LEASED);
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.violations, 0);
}

/* The dryer's order: the log stored at the cycle's open, the first lease right after. */
TEST(a_flash_stall_just_before_the_first_lease_costs_no_reject)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(500000U);
    uint32_t late = stall_us(45000U);
    ASSERT_TRUE(late != EDGES_MAX);
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(1000000U);
    ASSERT_EQ(R.h.phase_rejects, 0U);
    ASSERT_TRUE(first_fire_from(late) >= late + 4U);
    ASSERT_TRUE(first_fire_from(late) != EDGES_MAX);
    R.lease_every_ms = 0U; /* the lease runs out: every cycle begun completes */
    run_us(3000000U);
    ASSERT_EQ(R.split_pairs, 0U);
    ASSERT_EQ(R.pos, R.neg);
    ASSERT_EQ(R.violations, 0);
}

/* A stall with no edge in it (one word programmed, microseconds) ending well before the next
 * edge: that edge came on time, so it is an ordinary edge — the firing is exactly as without the
 * stall. */
static uint32_t first_fire_after_word_program(bool program)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(500000U);
    while (R.next_edge_us - R.now_us < 3000U) {
        run_us(100U);
    }
    if (program) {
        ASSERT_EQ(stall_us(50U), EDGES_MAX);
    } else {
        run_us(50U);
    }
    uint32_t next = R.edges;
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(1000000U);
    ASSERT_EQ(R.h.phase_rejects, 0U);
    ASSERT_EQ(R.violations, 0);
    ASSERT_EQ(R.zc.span.gen, 0U); /* no resync edge */
    return first_fire_from(next) - next;
}

TEST(a_stall_with_no_edge_in_it_changes_nothing)
{
    uint32_t without = first_fire_after_word_program(false);
    ASSERT_TRUE(without < EDGES_MAX);
    ASSERT_EQ(first_fire_after_word_program(true), without);
}

/* A word programmed just before an on-time edge (no edge pending at its end): that edge came on
 * time and is an ordinary one — no resync, the firing exactly as without the operation.  The late
 * edge is told by the pending bit, never by its closeness to an operation's end. */
static uint32_t first_fire_after_word_program_ending(uint32_t before_edge_us, bool program)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(500000U);
    while (R.next_edge_us - R.now_us > before_edge_us + 50U) {
        run_us(10U);
    }
    if (program) {
        ASSERT_EQ(stall_us(50U), EDGES_MAX);
    } else {
        run_us(50U);
    }
    uint32_t next = R.edges;
    ASSERT_EQ(start_lease(90, 2000), 0);
    keep_leased(90);
    run_us(1000000U);
    ASSERT_EQ(R.zc.span.gen, 0U); /* no resync edge */
    ASSERT_EQ(R.h.phase_rejects, 0U);
    ASSERT_EQ(R.violations, 0);
    return first_fire_from(next) - next;
}

TEST(an_on_time_edge_right_after_an_operation_is_not_taken_for_a_late_one)
{
    uint32_t without = first_fire_after_word_program_ending(20U, false);
    ASSERT_TRUE(without < EDGES_MAX);
    ASSERT_EQ(first_fire_after_word_program_ending(20U, true), without);
}

/* A refused lease says whether to ask again (-ACE2K_EAGAIN) or not (-ACE2K_EBUSY), and heat
 * names why (ace2k_heat_refusal_reason). */
TEST(a_lease_refused_for_a_store_in_progress_is_transient)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    ace2k_heat_set_inhibit(&R.h, true);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EAGAIN);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_OK);
}

/* A final verdict is not hidden behind the store's inhibit: fans not reading high refuse as
 * final at once, a store in progress or not. */
TEST(a_final_refusal_is_final_during_a_store_in_progress)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.in.fans_read = 1U;
    run_us(TICK_US);
    ace2k_heat_set_inhibit(&R.h, true);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_FANS);
    R.in.fans_read = ACE2K_HEAT_FANS_BOTH;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EAGAIN);
}

TEST(an_invalid_ntc_with_no_lease_is_transient_and_named)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.in.left_valid = false;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EAGAIN);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_NTC_INVALID);
}

TEST(fans_not_reading_high_are_a_final_refusal_and_named)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.in.fans_read = 1U;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_FANS);
}

/* Each cause of a refused lease answers transient (-ACE2K_EAGAIN: ask again) or final
 * (-ACE2K_EBUSY): an invalid NTC with no lease, a store in progress, a mains dip under its
 * bucket and a mains not yet measured since its return are transient; a fan not reading high is
 * final. */
TEST(each_cause_of_a_refused_lease_answers_transient_or_final)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.in.left_valid = false;
    run_us(TICK_US);
    int rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);
    ace2k_heat_set_inhibit(&R.h, true); /* an invalid NTC during a store */
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);

    R.in.left_valid = true; /* a store in progress alone: asked again */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);
    ace2k_heat_set_inhibit(&R.h, false);

    set_plausible(false); /* a mains dip under its bucket: asked again */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);
    set_plausible(true);
    run_us(500000U); /* the bucket drained */

    R.in.mains_measured = false; /* the mains back, not measured yet: asked again */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);
    R.in.mains_measured = true;

    R.in.fans_read = 1U; /* a final refusal, no latch */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EBUSY);
}

/* An invalid outlet NTC with no live lease alongside another cause: the lease answers as the
 * verdict ranks the causes — transient with a mains off-band short of its bucket, refused once
 * the bucket is full, final with the mains absent or a fan not reading high. */
TEST(an_invalid_ntc_with_another_cause_refuses_as_the_verdict_ranks)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.in.right_valid = false;
    set_plausible(false);
    run_us(TICK_US); /* both at once, the mains bucket short of full */
    int rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EAGAIN);
    run_us(ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * TICK_US); /* the mains bucket full */
    ASSERT_EQ(R.h.state, ACE2K_HEAT_IDLE);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_TRUE(rc != 0);
    set_plausible(true);
    run_us(2000000U); /* the bucket drained */

    R.in.mains_present = false; /* a hard limit ranked ahead of the NTC */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EBUSY);
    R.in.mains_present = true;

    R.in.fans_read = 1U; /* a fan fault ranked ahead of a debounced limit */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EBUSY);

    R.in.right_valid = true; /* the fan fault alone */
    run_us(TICK_US);
    rc = ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U);
    ASSERT_EQ(rc, -ACE2K_EBUSY);
}

/* A latch outranks a not-ok verdict: a real latch (an outlet over its limit while leased), then
 * the fans reading low at a later tick — the latch's reason is named, not the last tick's. */
TEST(the_refusal_reason_names_the_latch_over_a_not_ok_verdict)
{
    rig_reset(HALF_CYCLE_US_60);
    ASSERT_EQ(start_lease(50, 2000), 0);
    R.in.ntc_left_mc = 86000;
    run_us(TICK_US);
    ASSERT_EQ(R.h.state, ACE2K_HEAT_LATCHED);
    R.in.ntc_left_mc = 40000;
    R.in.fans_read = 1U;
    run_us(TICK_US);
    ASSERT_EQ(R.h.verdict, ACE2K_HEAT_FANS);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_NTC_OVER);
}

TEST(the_refusal_reason_names_a_latch_and_a_mains_bucket_not_recovered)
{
    rig_reset(HALF_CYCLE_US_60);
    run_us(TICK_US);
    R.h.mains_level = ACE2K_HEAT_MAINS_RESTART_LEVEL + 1U;
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    R.h.mains_level = 0U;
    R.h.state = ACE2K_HEAT_LATCHED;
    R.h.reason = ACE2K_HEAT_NTC_OVER;
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_NTC_OVER);
}

/* The mains back from an outage with no window measured yet: a new lease waits (transient,
 * named MAINS_IMPLAUSIBLE); a live lease renews through it — firing needs a period anyway. */
TEST(a_new_lease_waits_for_a_measured_mains_and_a_live_one_renews_through_it)
{
    rig_reset(HALF_CYCLE_US_60);
    R.in.mains_measured = false;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), -ACE2K_EAGAIN);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_MAINS_IMPLAUSIBLE);
    R.in.mains_measured = true;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_refusal_reason(&R.h), ACE2K_HEAT_OK);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), 0);
    R.in.mains_measured = false;
    run_us(TICK_US);
    ASSERT_EQ(ace2k_heat_lease(&R.h, 20, 1000, R.now_us / 1000U), 0);
    ASSERT_TRUE(ace2k_heat_leased(&R.h));
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
