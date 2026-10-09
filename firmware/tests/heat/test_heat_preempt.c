/* heat's stops against the zero-cross interrupt, preempted.  The interrupt (priority 1) may run
 * between any two stores of a stop called from the tick (priority 2); heat's stores are ordered by
 * its compiler barriers, so this file compiles heat.c into itself with the barrier replaced by a
 * hook that runs the next mains edge at a chosen barrier — every barrier of every stop in turn
 * (release, lease of duty 0, abort, latch, shutdown, a tick whose verdict is not ok, a new lease
 * over a pending half), at every phase of the mains cycle, with and without an odd half pending.
 * An immediate stop fires nothing from the call on — the injected edge included, whatever
 * barrier it lands on; a release never leaves a half-cycle without its pair (heat.h). */
#include <stdbool.h>
#include <stdint.h>

/* heat.c's names below are its own, redefined for this file */
// NOLINTBEGIN(readability-identifier-naming)
static void preempt_point(void);
#define ACE2K_COMPILER_BARRIER() preempt_point()
/* heat.c's interface under private names, so this copy links beside the core's own */
#define ace2k_heat_init                   preempt_heat_init
#define ace2k_heat_lease                  preempt_heat_lease
#define ace2k_heat_release                preempt_heat_release
#define ace2k_heat_abort                  preempt_heat_abort
#define ace2k_heat_shutdown               preempt_heat_shutdown
#define ace2k_heat_set_inhibit            preempt_heat_set_inhibit
#define ace2k_heat_zerocross              preempt_heat_zerocross
#define ace2k_heat_resync                 preempt_heat_resync
#define ace2k_heat_tick                   preempt_heat_tick
#define ace2k_heat_clear                  preempt_heat_clear
#define ace2k_heat_leased                 preempt_heat_leased
#define ace2k_heat_busy                   preempt_heat_busy
#define ace2k_heat_active                 preempt_heat_active
#define ace2k_heat_mains_implausible_held preempt_heat_mains_implausible_held
#define ace2k_heat_firing                 preempt_heat_firing
#define ace2k_heat_clear_blocker          preempt_heat_clear_blocker
#define ace2k_heat_mains_recovered        preempt_heat_mains_recovered
#define ace2k_heat_refusal_reason         preempt_heat_refusal_reason
#define ace2k_heat_mains_measured         preempt_heat_mains_measured
// NOLINTEND(readability-identifier-naming)
#include "heat/heat.c" // NOLINT(bugprone-suspicious-include): the unit under test, with the hook
#include "test.h"

#define HALF_US   8333U /* 60 Hz */
#define TICK_US   10000U
#define EDGES_MAX 512U
#define DUTY_HALF 50U
#define DUTY_MOST 90U
#define PHASES    24U /* release ticks tried: they walk the 10 ms tick across the 8.3 ms edges */
#define BARRIERS  4U  /* more than any stop has */
#define AFTER_US  100000U

// NOLINTBEGIN(readability-identifier-naming)
enum way {
    WAY_RELEASE,
    WAY_DUTY_ZERO,
    WAY_ABORT,
    WAY_LATCH,
    WAY_SHUTDOWN,
    WAY_BAD_TICK,
    WAY_RE_LEASE,
    WAY_COUNT,
};
// NOLINTEND(readability-identifier-naming)

// NOLINTNEXTLINE(readability-identifier-naming)
struct prig {
    struct ace2k_heat h;
    struct ace2k_heat_inputs in;
    uint32_t now_us, next_edge_us, next_tick_us;
    uint32_t edge; /* index of the next edge */
    bool fired_at[EDGES_MAX];
    bool armed;
    uint32_t barrier, inject_at;
    bool injected;
    /* the gate: high from a fire until TIM7's 6 ms end; a gate_off before that cuts a pulse */
    bool gate_high;
    uint32_t fire_us, cuts;
};

// NOLINTNEXTLINE(readability-identifier-naming)
static struct prig P;

static void p_fire(void *ctx)
{
    (void)ctx;
    if (P.edge < EDGES_MAX) {
        P.fired_at[P.edge] = true;
    }
    P.gate_high = true;
    P.fire_us = P.now_us;
}

/* TIM7 ends a pulse ACE2K_HEAT_GATE_PULSE_US after its fire. */
static void pulse_end(void)
{
    if (P.gate_high && P.now_us - P.fire_us >= ACE2K_HEAT_GATE_PULSE_US) {
        P.gate_high = false;
    }
}

static void p_off(void *ctx)
{
    (void)ctx;
    pulse_end();
    if (P.gate_high) {
        P.cuts++; /* a pulse cut short: it may not make its half-cycle */
    }
    P.gate_high = false;
}

static bool p_read(void *ctx)
{
    (void)ctx;
    return false;
}

// NOLINTNEXTLINE(readability-identifier-naming)
static const struct ace2k_heat_ops p_ops = {
    .gate_fire = p_fire,
    .gate_off = p_off,
    .gate_read = p_read,
};

static void run_edge(void)
{
    P.now_us = P.next_edge_us;
    pulse_end();
    ace2k_heat_zerocross(&P.h, P.now_us);
    P.edge++;
    P.next_edge_us += HALF_US;
}

/* The interrupt arrives at the inject_at-th barrier of the stop in progress. */
static void preempt_point(void)
{
    if (!P.armed) {
        return;
    }
    P.barrier++;
    if (P.barrier == P.inject_at) {
        P.armed = false;
        P.injected = true;
        run_edge();
    }
}

static void run_tick(void)
{
    P.now_us = P.next_tick_us;
    pulse_end();
    ace2k_heat_tick(&P.h, &P.in, P.now_us / 1000U, P.now_us);
    P.next_tick_us += TICK_US;
}

/* Edges and ticks in time order up to end_us. */
static void run_to(uint32_t end_us)
{
    for (;;) {
        bool tick = P.next_tick_us <= P.next_edge_us;
        uint32_t next = tick ? P.next_tick_us : P.next_edge_us;
        if (next > end_us) {
            return;
        }
        if (tick) {
            run_tick();
        } else {
            run_edge();
        }
    }
}

static void p_reset(void)
{
    P = (struct prig){ 0 };
    P.in = (struct ace2k_heat_inputs){
        .ntc_left_mc = 25000,
        .ntc_right_mc = 25000,
        .left_valid = true,
        .right_valid = true,
        .mains_present = true,
        .mains_hz10 = 600U, /* 60 Hz: HALF_US */
        .mains_measured = true,
        .fans_read = 3U,
        .fans_commanded = true,
    };
    P.next_edge_us = HALF_US;
    P.next_tick_us = TICK_US;
    ace2k_heat_init(&P.h, &p_ops, 0);
}

/* The heat operation under test, from the tick's context. */
static void stop_by(enum way way, uint8_t duty)
{
    switch (way) {
    case WAY_RELEASE:
        ace2k_heat_release(&P.h);
        break;
    case WAY_DUTY_ZERO:
        (void)ace2k_heat_lease(&P.h, 0U, ACE2K_HEAT_LEASE_MAX_MS, P.now_us / 1000U);
        break;
    case WAY_ABORT:
        ace2k_heat_abort(&P.h);
        break;
    case WAY_LATCH:
        latch(&P.h, ACE2K_HEAT_GATE_STUCK); /* the tick's latch, on a verdict still ok */
        break;
    case WAY_SHUTDOWN:
        ace2k_heat_shutdown(&P.h);
        break;
    case WAY_BAD_TICK:
        P.in.fans_read = 1U; /* a fan stops reading high: this tick's verdict is not ok */
        run_tick();
        break;
    default:
        (void)ace2k_heat_lease(&P.h, duty, ACE2K_HEAT_LEASE_MAX_MS, P.now_us / 1000U);
        break;
    }
}

static bool immediate(enum way way)
{
    switch (way) {
    case WAY_ABORT:
    case WAY_LATCH:
    case WAY_SHUTDOWN:
    case WAY_BAD_TICK:
        return true;
    default:
        return false;
    }
}

/* Every fired edge is one of a pair of consecutive fired edges. */
static bool pairs_only(void)
{
    uint32_t n = P.edge < EDGES_MAX ? P.edge : EDGES_MAX;
    uint32_t i = 0;
    while (i < n) {
        if (!P.fired_at[i]) {
            i++;
            continue;
        }
        if (i + 1U >= n || !P.fired_at[i + 1U]) {
            return false;
        }
        i += 2U;
    }
    return true;
}

/* Runs edge by edge until the even half of a cycle has just fired; false if none within max. */
static bool until_even_fired(uint32_t max_edges)
{
    for (uint32_t i = 0; i < max_edges; i++) {
        run_to(P.next_edge_us);
        if (P.h.state == ACE2K_HEAT_LEASED && P.h.edge_parity != 0U && P.h.fire_this_cycle) {
            return true;
        }
    }
    return false;
}

static bool heat_pending(void)
{
    return P.h.fire_mode == ACE2K_HEAT_FIRE_PENDING;
}

/* A lease and `phase` ticks of firing; with `pending`, the even half of a cycle fired and
 * released (its odd half pending), else the moment right after a tick.  False when there is
 * nothing to try: the case needs a pending half, or the operation has fewer barriers than k. */
static bool set_up(uint8_t duty, uint32_t phase, bool pending, enum way way)
{
    if (way == WAY_RE_LEASE && !pending) {
        return false;
    }
    p_reset();
    run_tick();
    ASSERT_EQ(ace2k_heat_lease(&P.h, duty, ACE2K_HEAT_LEASE_MAX_MS, P.now_us / 1000U), 0);
    run_to(P.now_us + (phase * TICK_US) + 200000U);
    if (!pending) {
        run_to(P.next_tick_us); /* the stop runs right after a tick, as the dryer's does */
        return true;
    }
    ASSERT_TRUE(until_even_fired(40U));
    ace2k_heat_release(&P.h); /* at the very edge that fired the even half */
    ASSERT_TRUE(heat_pending());
    ASSERT_EQ(P.cuts, 0U); /* its pulse left whole */
    return true;
}

static uint32_t fires_from(uint32_t edge)
{
    uint32_t n = 0U;
    for (uint32_t i = edge; i < P.edge && i < EDGES_MAX; i++) {
        n += P.fired_at[i] ? 1U : 0U;
    }
    return n;
}

/* The operation with an edge at its k-th barrier, then 100 ms of mains and ticks. */
static bool scenario(uint8_t duty, uint32_t phase, bool pending, enum way way, uint32_t k)
{
    if (!set_up(duty, phase, pending, way)) {
        return false;
    }
    if (way == WAY_BAD_TICK) {
        run_to(P.next_tick_us - 1U); /* the edges before that tick, in time order */
    }
    uint32_t stopped_at = P.edge; /* before the call: the injected edge counts */
    P.armed = true;
    P.inject_at = k;
    stop_by(way, duty);
    P.armed = false;
    if (!P.injected) {
        return false;
    }
    run_to(P.now_us + AFTER_US);
    uint32_t after = fires_from(stopped_at);
    if (immediate(way)) {
        if (after != 0U) {
            printf("  duty %u phase %u pending %u way %u barrier %u: a half fired after the call\n",
                   (unsigned)duty, (unsigned)phase, (unsigned)pending, (unsigned)way, (unsigned)k);
        }
        ASSERT_EQ(after, 0U); /* nothing after the silencing store — a cycle begun stays lone */
        ASSERT_TRUE(!ace2k_heat_leased(&P.h));
        return true;
    }
    if (!pairs_only()) {
        printf("  duty %u phase %u pending %u way %u barrier %u: a half-cycle with no pair\n",
               (unsigned)duty, (unsigned)phase, (unsigned)pending, (unsigned)way, (unsigned)k);
    }
    ASSERT_TRUE(pairs_only());
    if (P.cuts != 0U) {
        printf("  duty %u phase %u pending %u way %u barrier %u: a pulse cut short\n",
               (unsigned)duty, (unsigned)phase, (unsigned)pending, (unsigned)way, (unsigned)k);
    }
    ASSERT_EQ(P.cuts, 0U); /* a release never cuts a pulse the interrupt just started */
    if (way == WAY_RE_LEASE) {
        ASSERT_TRUE(ace2k_heat_leased(&P.h)); /* the new lease keeps the cycle, then pairs */
        return true;
    }
    ASSERT_TRUE(after <= 1U); /* at most the odd half of the fired cycle */
    ASSERT_TRUE(!ace2k_heat_leased(&P.h));
    return true;
}

/* Every way, every barrier, with and without a pending half, at one duty and phase. */
static void every_way(uint8_t duty, uint32_t phase, uint32_t *tried)
{
    for (uint32_t way = 0; way < (uint32_t)WAY_COUNT; way++) {
        for (uint32_t k = 1U; k <= BARRIERS; k++) {
            tried[way] += scenario(duty, phase, false, (enum way)way, k) ? 1U : 0U;
            tried[way] += scenario(duty, phase, true, (enum way)way, k) ? 1U : 0U;
        }
    }
}

TEST(an_edge_preempting_any_stop_at_any_barrier_fires_nothing_after_it_and_orphans_nothing)
{
    static const uint8_t duties[] = { DUTY_HALF, DUTY_MOST };
    uint32_t tried[WAY_COUNT] = { 0U };
    for (size_t d = 0; d < ACE2K_ARRAY_SIZE(duties); d++) {
        for (uint32_t phase = 0; phase < PHASES; phase++) {
            every_way(duties[d], phase, tried);
        }
    }
    for (uint32_t way = 0; way < (uint32_t)WAY_COUNT; way++) {
        ASSERT_TRUE(tried[way] >= ACE2K_ARRAY_SIZE(duties) * PHASES); /* every way exercised */
    }
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
