#include "test.h"
#include "ace2k_board/zerocross.h" /* ACE2K_ZEROCROSS_LOCKOUT_US */
#include "heat/heat.h"
#include "heat/mains.h"

/* The board, faked: the interrupt's filter (driven by the train below, or set by hand) and the
 * counts the interrupt keeps.  The clock is microseconds (any clock works — the board uses the
 * timer's ticks). */
// NOLINTNEXTLINE(readability-identifier-naming)
static struct ace2k_mains_edge_filter F;
static uint32_t ace2k_fake_edges;
static uint32_t ace2k_fake_rejects;

#define US_PER_MS 1000U

static uint32_t fake_count(void *ctx)
{
    (void)ctx;
    return ace2k_fake_edges;
}

static uint32_t fake_rejects(void *ctx)
{
    (void)ctx;
    return ace2k_fake_rejects;
}

static void fake_span(void *ctx, struct ace2k_mains_span *out)
{
    (void)ctx;
    ace2k_mains_edge_filter_take(&F, out);
}

static const struct ace2k_mains_ops ace2k_fake_ops = {
    .zerocross_count = fake_count,
    .zerocross_rejects = fake_rejects,
    .span = fake_span,
};

/* The mains as a train of edges in true time: one every half_us from next_us, none in
 * [off_us, on_us), each followed by a bounce bounce_us later when that is not 0.  clock0 is the
 * filter clock's value at true time 0 (to put the timer's wrap anywhere).  stall is the flash
 * layer's record of its last operation, on the filter's clock. */
struct ace2k_train {
    uint32_t next_us;
    uint32_t half_us;
    uint32_t off_us;
    uint32_t on_us;
    uint32_t bounce_us;
    uint32_t asym_us; /* the half-cycles alternate half_us + asym_us and half_us − asym_us */
    uint32_t clock0;
    /* recurring dips: from dip_from_us, the mains off dip_len_us out of every dip_every_us (none
     * when dip_every_us is 0) */
    uint32_t dip_from_us;
    uint32_t dip_every_us;
    uint32_t dip_len_us;
    struct ace2k_mains_stall stall;
    uint32_t n; /* edges of the train so far */
};

// NOLINTNEXTLINE(readability-identifier-naming)
static struct ace2k_train T;

/* One edge through the filter, as the interrupt does it: accepted → the count, a counted reject
 * → the rejects. */
static void feed(uint32_t at_us)
{
    enum ace2k_mains_edge v = ace2k_mains_edge_filter(&F, T.clock0 + at_us, &T.stall);
    if (v == ACE2K_MAINS_EDGE_ACCEPT || v == ACE2K_MAINS_EDGE_RESYNC) {
        ace2k_fake_edges++;
    } else if (v == ACE2K_MAINS_EDGE_REJECT) {
        ace2k_fake_rejects++;
    }
}

static bool mains_on(uint32_t at_us)
{
    if (T.dip_every_us != 0U && at_us >= T.dip_from_us &&
        (at_us - T.dip_from_us) % T.dip_every_us < T.dip_len_us) {
        return false;
    }
    if (at_us < T.off_us) {
        return true;
    }
    return at_us >= T.on_us;
}

/* The train's next edge. */
static void advance(void)
{
    T.next_us += T.half_us;
    if (T.asym_us != 0U) {
        T.next_us += (T.n & 1U) != 0U ? T.asym_us : 0U - T.asym_us;
    }
    T.n++;
}

/* Every edge of the train up to t_us, in order. */
static void edges_until(uint32_t t_us)
{
    while (T.next_us <= t_us) {
        if (mains_on(T.next_us)) {
            feed(T.next_us);
            if (T.bounce_us != 0U) {
                feed(T.next_us + T.bounce_us);
            }
        }
        advance();
    }
}

/* The train's next edge, fed: moves the missing edge's parity by one. */
static void advance_fed(void)
{
    edges_until(T.next_us);
}

static void train_start(struct ace2k_mains *m, uint32_t half_us, uint32_t clock0)
{
    T = (struct ace2k_train){
        .next_us = half_us,
        .half_us = half_us,
        .off_us = UINT32_MAX,
        .on_us = UINT32_MAX,
        .clock0 = clock0,
    };
    ace2k_fake_edges = 0;
    ace2k_fake_rejects = 0;
    ace2k_mains_edge_filter_init(&F, ACE2K_ZEROCROSS_LOCKOUT_US, 0U);
    ace2k_mains_init(m, &ace2k_fake_ops, 0, 0, US_PER_MS);
}

/* The regular ticks after from_ms up to to_ms, each after the edges up to its own time. */
static uint32_t ticks(struct ace2k_mains *m, uint32_t from_ms, uint32_t to_ms)
{
    for (uint32_t t = from_ms + 10U; t <= to_ms; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(m, t);
    }
    return to_ms;
}

/* A flash operation holds interrupts off for `us` from start_us (the ticks up to it already ran):
 * the edges in it are lost but one — the pending bit holds it, the flash layer records it held,
 * and the interrupt takes it at the stall's end, late — and no tick runs; then the board tick's
 * catch-up pass (tick.c), whose now_ms is the last 10 ms boundary the stall passed while the span
 * and the count it reads are as of the stall's end.  Returns the catch-up tick's now_ms. */
static uint32_t stall(struct ace2k_mains *m, uint32_t start_us, uint32_t us)
{
    uint32_t end_us = start_us + us;
    bool pending = false;
    while (T.next_us <= end_us) {
        if (mains_on(T.next_us)) {
            pending = true;
        }
        advance();
    }
    T.stall.count++;
    T.stall.start = T.clock0 + start_us;
    T.stall.end = T.clock0 + end_us;
    T.stall.held = pending;
    if (pending) {
        feed(end_us + 5U); /* the interrupt's entry, microseconds after */
    }
    uint32_t catch_up_ms = end_us / 10000U * 10U;
    ace2k_mains_tick(m, catch_up_ms);
    return catch_up_ms;
}

#define HALF_US_60   8333U  /* 60.002 Hz: hz10 600 */
#define HALF_US_59_9 8347U  /* 59.901 Hz: hz10 599 */
#define HALF_US_50   10000U /* hz10 500 */
#define HALF_US_49_9 10020U /* 49.900 Hz: hz10 499 */

TEST(steady_trains_read_exact)
{
    struct ace2k_mains m;
    const uint32_t halves[] = { HALF_US_60, HALF_US_59_9, HALF_US_50, HALF_US_49_9 };
    const uint16_t hz10[] = { 600, 599, 500, 499 };
    for (size_t i = 0; i < sizeof halves / sizeof halves[0]; i++) {
        train_start(&m, halves[i], 0);
        ASSERT_EQ(m.hz10, 0);
        uint32_t now = ticks(&m, 0, 1000);
        /* the first window: a span from the first tick's edge, measured — or, when a tick came
         * before the first edge (49.9 Hz: 10.02 ms), a window the mains read absent in, counted
         * a few edges short: none measured (0), never a plausible count */
        if (ace2k_mains_measured(&m)) {
            ASSERT_TRUE(ace2k_mains_plausible(&m));
        } else {
            ASSERT_EQ(m.hz10, 0);
        }
        ASSERT_TRUE(m.present);
        for (uint32_t k = 0; k < 5U; k++) {
            now = ticks(&m, now, now + 1000U);
            ASSERT_EQ(m.hz10, hz10[i]);
            ASSERT_EQ(m.published_ms, now);
        }
        ASSERT_EQ(ace2k_fake_rejects, 0U);
    }
}

/* The span's clock wraps (the board's 32-bit timer, every 35.8 s at 120 MHz): unsigned differences
 * across it still measure right — a span is about a second, far below one wrap. */
TEST(a_span_across_the_clock_wrap_reads_exact)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_59_9, 0xFFFFFFFFU - 1500000U);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 599);
    now = ticks(&m, now, 2000); /* this span holds the wrap */
    ASSERT_EQ(m.hz10, 599);
    ticks(&m, now, 3000);
    ASSERT_EQ(m.hz10, 599);
}

TEST(present_drops_101_ms_after_the_last_edge)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    ASSERT_TRUE(!m.present); /* never an edge */
    ace2k_fake_edges = 1;
    ace2k_mains_tick(&m, 10);
    ASSERT_TRUE(m.present);
    ace2k_mains_tick(&m, 110);
    ASSERT_TRUE(m.present);
    ace2k_mains_tick(&m, 111);
    ASSERT_TRUE(!m.present);
}

/* A chattering input past what the lockout allows (a span set by hand): an impossible frequency,
 * saturated, never wrapped to a plausible one. */
TEST(a_chattering_input_saturates_hz10_instead_of_wrapping)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    for (uint32_t t = 10; t <= 1000U; t += 10U) {
        ace2k_fake_edges++;
        F.span.edges = 1U + (100U * t);                /* 100 000 edges a second */
        F.span.last = 5000U + ((t - 10U) * US_PER_MS); /* the last, at the tick */
        ace2k_mains_tick(&m, t);
    }
    ASSERT_EQ(m.hz10, 65535);
    ASSERT_TRUE(!ace2k_mains_plausible(&m));
}

TEST(plausible_only_near_fifty_or_sixty)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    const uint16_t yes[] = { 490, 500, 510, 590, 600, 610 };
    const uint16_t no[] = { 0, 489, 511, 550, 589, 611, 1200 };
    for (size_t i = 0; i < sizeof yes / sizeof yes[0]; i++) {
        m.hz10 = yes[i];
        ASSERT_TRUE(ace2k_mains_plausible(&m));
    }
    for (size_t i = 0; i < sizeof no / sizeof no[0]; i++) {
        m.hz10 = no[i];
        ASSERT_TRUE(!ace2k_mains_plausible(&m));
    }
}

#define LOCKOUT_US ACE2K_ZEROCROSS_LOCKOUT_US

TEST(the_first_edge_is_always_accepted_and_later_ones_only_past_the_lockout)
{
    ASSERT_TRUE(ace2k_mains_edge_accept(false, 0, 5, LOCKOUT_US));
    ASSERT_TRUE(!ace2k_mains_edge_accept(true, 1000, 1000 + LOCKOUT_US - 1U, LOCKOUT_US));
    ASSERT_TRUE(ace2k_mains_edge_accept(true, 1000, 1000 + LOCKOUT_US, LOCKOUT_US));
    /* across the clock's wrap */
    ASSERT_TRUE(!ace2k_mains_edge_accept(true, 0xFFFFF000U, 0x00000100U, LOCKOUT_US));
    ASSERT_TRUE(ace2k_mains_edge_accept(true, 0xFFFFF000U, 0xFFFFF000U + LOCKOUT_US, LOCKOUT_US));
}

TEST(a_doubled_edge_train_at_sixty_hertz_reads_sixty_and_counts_the_doubles)
{
    struct ace2k_mains m;
    /* 60 Hz, each edge followed by a second one 200 µs later — the slow edge crossing the input's
     * threshold twice, the v0.3.0 limitation's hypothesis */
    train_start(&m, HALF_US_60, 0);
    T.bounce_us = 200U;
    ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 600);
    ASSERT_EQ(m.rejects, 120);
    ASSERT_TRUE(ace2k_mains_plausible(&m));
}

TEST(a_clean_fifty_hertz_train_loses_nothing_to_the_lockout)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_50, 0);
    ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 500);
    ASSERT_EQ(m.rejects, 0);
}

/* The heater's side: its tick fed the hz10 the mains holds. */
static void heat_sees(struct ace2k_heat *h, const struct ace2k_mains *m, uint32_t now_ms)
{
    struct ace2k_heat_inputs in = {
        .ntc_left_mc = 25000,
        .ntc_right_mc = 25000,
        .left_valid = true,
        .right_valid = true,
        .mains_present = m->present,
        .mains_hz10 = m->hz10,
        .mains_measured = ace2k_mains_measured(m),
        .fans_read = 3U,
        .fans_commanded = true,
    };
    ace2k_heat_tick(h, &in, now_ms, now_ms * US_PER_MS);
}

static void gate_nop(void *ctx)
{
    (void)ctx;
}

static bool gate_low(void *ctx)
{
    (void)ctx;
    return false;
}

static const struct ace2k_heat_ops ace2k_nop_gate = {
    .gate_fire = gate_nop,
    .gate_off = gate_nop,
    .gate_read = gate_low,
};

/* The bench's dip, replayed: a 50 ms store in mid-window loses 6 edges but the late one, whose
 * lockout swallows the next real edge — counted over the window, 58 Hz.  The late edge ends the
 * span and the next starts at the first edge accepted after it, so no span holds a lost edge:
 * hz10 keeps 599 while no span is long enough, the heater never reads the mains implausible, its
 * bucket stays empty, and the next published value is exact. */
TEST(a_stall_with_a_late_edge_leaves_hz10_exact)
{
    struct ace2k_mains m;
    struct ace2k_heat h;
    train_start(&m, HALF_US_59_9, 0);
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 599);
    now = ticks(&m, now, 1400);
    uint32_t edges = ace2k_fake_edges;
    now = stall(&m, now * US_PER_MS, 50000U);
    ASSERT_EQ(now, 1450U);
    ASSERT_EQ(ace2k_fake_edges, edges + 1U); /* the late one */
    ASSERT_EQ(F.span.gen, 1U);
    ASSERT_EQ(F.span.edges, 0U); /* the late edge is in no span */
    ASSERT_TRUE(m.present);
    for (uint32_t t = now + 10U; t <= 4000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
        ASSERT_EQ(m.hz10, 599);
        ASSERT_TRUE(h.limits != ACE2K_HEAT_MAINS_IMPLAUSIBLE);
        ASSERT_EQ(h.mains_level, 0U);
    }
    ASSERT_EQ(ace2k_fake_rejects, 0U); /* the swallowed real edge: not counted */
    ASSERT_EQ(m.published_ms, 4000U);
}

/* A stall with no edge in it — here over the publication's tick: nothing lost, nothing late, the
 * span goes on and the catch-up tick publishes it exact. */
TEST(an_edgeless_stall_across_a_publication_changes_nothing)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_49_9, 0);
    T.next_us = 5020U; /* an edge before the first tick */
    uint32_t now = ticks(&m, 0, 1990);
    ASSERT_EQ(m.hz10, 499);
    /* edges at 1 999 000 and 2 009 020 µs: the stall runs between them, over the 2 000 ms tick */
    ASSERT_EQ(T.next_us, 1999000U);
    edges_until(1999500U);
    now = stall(&m, 1999500U, 7000U);
    ASSERT_EQ(now, 2000U);
    ASSERT_EQ(m.published_ms, 2000U);
    ASSERT_EQ(m.hz10, 499);
    ticks(&m, now, 3000);
    ASSERT_EQ(F.span.gen, 0U); /* the next edge, 2.5 ms after the stall, was on time */
    ASSERT_EQ(m.hz10, 499);
    ASSERT_EQ(m.published_ms, 3000U);
}

/* Stalls that keep breaking the span (a failing store retried at its shortest back-off): hz10 is
 * kept at most ACE2K_MAINS_STALE_MS, then the longest span since is published however short. */
TEST(stalls_that_keep_breaking_the_span_publish_the_best_span_after_the_bound)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_50, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 500);
    ASSERT_EQ(m.published_ms, 1000U);
    T.half_us = HALF_US_49_9; /* the frequency moves: only a publication can show it */
    for (uint32_t k = 0; k < 8U; k++) {
        now = ticks(&m, now, 1000U + (400U * (k + 1U)) - 30U);
        uint32_t gen = F.span.gen;
        now = stall(&m, now * US_PER_MS, 15000U);
        ASSERT_EQ(F.span.gen, gen + 1U);
        if (now < 1000U + ACE2K_MAINS_STALE_MS) {
            ASSERT_EQ(m.published_ms, 1000U);
            ASSERT_EQ(m.hz10, 500);
        }
    }
    ASSERT_EQ(m.published_ms, 1000U + ACE2K_MAINS_STALE_MS);
    ASSERT_EQ(m.hz10, 499); /* a span of under 400 ms, measured exact */
}

/* A stall does not mask an outage: presence is the time since the last edge, stall or not. */
TEST(a_stall_then_a_real_outage_reads_absent)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_TRUE(m.present);
    T.off_us = now * US_PER_MS; /* the mains gone before the stall: no edge in it */
    now = stall(&m, now * US_PER_MS, 150000U);
    ASSERT_TRUE(!m.present);
    now = stall(&m, now * US_PER_MS, 30000U);
    ASSERT_TRUE(!m.present);
    ticks(&m, now, now + 10U);
    ASSERT_TRUE(!m.present);
}

/* The mains stops inside a stall: the edge the pending bit held is counted at the catch-up tick,
 * so presence runs 100 ms from there — the outage reads absent up to the stall's length later than
 * without the stall, never later than that. */
TEST(an_outage_inside_a_stall_reads_absent_100_ms_after_the_catch_up)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.off_us = (now * US_PER_MS) + 20000U; /* 20 ms into a 45 ms stall */
    now = stall(&m, now * US_PER_MS, 45000U);
    ASSERT_EQ(now, 1040U);
    ASSERT_TRUE(m.present);
    now = ticks(&m, now, 1140);
    ASSERT_TRUE(m.present);
    now = ticks(&m, now, 1150);
    ASSERT_TRUE(!m.present);
    /* no stall: the mains stops, absent 101 ms after the last edge's tick */
    struct ace2k_mains n;
    train_start(&n, HALF_US_60, 0);
    now = ticks(&n, 0, 1000);
    T.off_us = now * US_PER_MS;
    now = ticks(&n, now, 1100);
    ASSERT_TRUE(n.present);
    ticks(&n, now, 1110);
    ASSERT_TRUE(!n.present);
}

/* An outage: a window the mains read absent in is counted as a whole, as before the span
 * existed — low, the heater's bucket fills as it always did — and the frequency is plausible
 * again only from the first window after the return, never sooner. */
TEST(an_outage_is_counted_and_plausible_again_only_a_window_after_the_return)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 600);
    T.off_us = 1200000U;
    T.on_us = 1500000U;
    uint32_t absent_at = 0;
    uint32_t implausible_at = 0;
    uint32_t plausible_at = 0;
    for (uint32_t t = now + 10U; t <= 4000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        if (absent_at == 0U && !m.present) {
            absent_at = t;
        }
        if (implausible_at == 0U && !ace2k_mains_plausible(&m)) {
            implausible_at = t;
            ASSERT_EQ(m.hz10, 420); /* 84 edges in the window 1 000–2 000 */
        }
        if (implausible_at != 0U && plausible_at == 0U && ace2k_mains_plausible(&m)) {
            plausible_at = t;
        }
    }
    ASSERT_EQ(absent_at, 1310U);
    ASSERT_EQ(implausible_at, 2000U);
    ASSERT_EQ(plausible_at, 3000U); /* the span from the return, published at the next window */
    ASSERT_EQ(m.hz10, 600);
}

/* A long outage: its windows count no edge (0), the return's window is counted too (low), and the
 * staleness bound never publishes a short span early — every counted window is a publication. */
TEST(a_long_outage_reads_as_counted_windows_until_one_after_the_return)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.off_us = 1200000U;
    T.on_us = 6500000U;
    now = ticks(&m, now, 6990);
    ASSERT_EQ(m.hz10, 0);
    now = ticks(&m, now, 7000); /* 60 edges since the return */
    ASSERT_EQ(m.hz10, 300);
    ticks(&m, now, 8000);
    ASSERT_EQ(m.hz10, 600);
}

/* A dip shorter than the presence window: the span holds its gap, so it reads as a low frequency —
 * a dip is never measured away. */
TEST(a_short_dip_inside_a_span_reads_off_band)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.off_us = 1300000U;
    T.on_us = 1360000U; /* 7 edges missing */
    now = ticks(&m, now, 2000);
    ASSERT_TRUE(m.present);
    ASSERT_TRUE(!ace2k_mains_plausible(&m));
    ticks(&m, now, 3000);
    ASSERT_EQ(m.hz10, 600);
}

/* The flash layer's record of an operation. */
static struct ace2k_mains_stall op(uint32_t count, uint32_t start, uint32_t end, bool held)
{
    return (struct ace2k_mains_stall){ .count = count, .start = start, .end = end, .held = held };
}

/* The interrupt's decision with the flash stalls in it: the first edge after an operation that
 * held one pending is the late edge — by the record, whatever its time — the resync edge; the
 * reject right after it (the real edge the late one's lockout swallowed) is not counted; an edge
 * after an operation that held none is an ordinary one, however close to its end, and a bounce
 * after it is counted. */
TEST(the_filter_tells_the_late_edge_by_the_pending_bit_and_forgives_the_one_reject_it_causes)
{
    struct ace2k_mains_edge_filter f;
    ace2k_mains_edge_filter_init(&f, LOCKOUT_US, 5U);
    struct ace2k_mains_stall none = op(5U, 0U, 0U, false);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 1000U, &none), ACE2K_MAINS_EDGE_ACCEPT); /* the first */
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 1200U, &none), ACE2K_MAINS_EDGE_REJECT); /* a bounce */
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 9333U, &none), ACE2K_MAINS_EDGE_ACCEPT);
    /* a stall 10 000–40 000 that held an edge: its late edge, the real one inside its lockout,
     * then a bounce */
    struct ace2k_mains_stall held = op(6U, 10000U, 40000U, true);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 40005U, &held), ACE2K_MAINS_EDGE_RESYNC);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 41000U, &held), ACE2K_MAINS_EDGE_REJECT_UNCOUNTED);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 41500U, &held), ACE2K_MAINS_EDGE_REJECT);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 49333U, &held), ACE2K_MAINS_EDGE_ACCEPT);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 50000U, &held), ACE2K_MAINS_EDGE_REJECT);
    /* a stall whose held edge the lockout refuses (it ended within the lockout): counted, and the
     * edge after it is on time */
    struct ace2k_mains_stall short_held = op(7U, 54000U, 54990U, true);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 55000U, &short_held), ACE2K_MAINS_EDGE_REJECT);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 57666U, &short_held), ACE2K_MAINS_EDGE_ACCEPT);
    /* no false late edge: an operation that held none ends 5 µs before an on-time edge — an
     * ordinary edge, its bounce counted */
    struct ace2k_mains_stall close = op(8U, 65944U, 65994U, false);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 65999U, &close), ACE2K_MAINS_EDGE_ACCEPT);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 66199U, &close), ACE2K_MAINS_EDGE_REJECT);
    /* exact: an operation that held one — the next edge is the late one, even taken 2 ms on */
    struct ace2k_mains_stall held2 = op(9U, 72000U, 72332U, true);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 74332U, &held2), ACE2K_MAINS_EDGE_RESYNC);
    /* every pair of bridges a period at most: 1 000 → 9 333 → 10 000 before the first stall,
     * then the lockout with 40 000 → 41 000 (its end to the swallowed edge), 40 000 → 41 000 →
     * 49 333 (the bounce at 41 500 ends none), edge to edge after — the longest, 49 333 → 57 666 →
     * 65 999, one period — and 57 666 → 65 999 → 72 000 before the second (the bounce at 66 199
     * ends none) */
    ASSERT_EQ(f.span.bridge, 65999U - 49333U);
}

/* The span the filter keeps: a resync edge closes it; the next accepted edge opens the next. */
TEST(the_resync_edge_ends_the_span_and_the_next_edge_starts_one)
{
    struct ace2k_mains_edge_filter f;
    ace2k_mains_edge_filter_init(&f, LOCKOUT_US, 0U);
    struct ace2k_mains_stall none = op(0U, 0U, 0U, false);
    (void)ace2k_mains_edge_filter(&f, 8333U, &none);
    (void)ace2k_mains_edge_filter(&f, 16666U, &none);
    ASSERT_EQ(f.span.gen, 0U);
    ASSERT_EQ(f.span.edges, 2U);
    ASSERT_EQ(f.span.last, 16666U);
    (void)ace2k_mains_edge_filter(&f, 17000U, &none); /* refused: the span unchanged */
    ASSERT_EQ(f.span.edges, 2U);
    struct ace2k_mains_stall held = op(1U, 20000U, 59990U, true);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 60000U, &held), ACE2K_MAINS_EDGE_RESYNC);
    ASSERT_EQ(f.span.gen, 1U);
    ASSERT_EQ(f.span.edges, 0U);
    ASSERT_EQ(ace2k_mains_edge_filter(&f, 68333U, &held), ACE2K_MAINS_EDGE_ACCEPT);
    ASSERT_EQ(f.span.edges, 1U);
    ASSERT_EQ(f.span.last, 68333U);
    /* the bridges: 16 666 → 20 000 before the stall (the reject at 17 000 ends none), paired with
     * 8 333 → 16 666; 59 990 → 68 333 after, paired with the lockout — the stall hid how many
     * edges it held */
    struct ace2k_mains_span s;
    ace2k_mains_edge_filter_take(&f, &s);
    ASSERT_EQ(s.bridge, LOCKOUT_US + (68333U - 59990U));
    ASSERT_EQ(f.span.bridge, 0U); /* handed over */
    ASSERT_EQ(s.edges, 1U);
}

/* The same through a clean 60 Hz train: a 40 ms stall whose late edge lands under 2 ms before the
 * next real one.  One resync edge, the one reject it causes not counted, none counted. */
TEST(a_stall_in_a_clean_train_counts_no_reject)
{
    struct ace2k_mains_edge_filter f;
    ace2k_mains_edge_filter_init(&f, LOCKOUT_US, 0U);
    uint32_t counts[4] = { 0 };
    uint32_t stall_from = 500000U;
    uint32_t stall_to = 500000U + 40000U;
    struct ace2k_mains_stall st = op(0U, 0U, 0U, false);
    for (uint32_t at = 8333U; at < 1000000U; at += 8333U) {
        if (at > stall_from && at <= stall_to) {
            continue; /* lost */
        }
        if (at > stall_to && st.count == 0U) {
            /* the operation ends: the pending edge, taken at the stall's end */
            st = op(1U, stall_from, stall_to, true);
            counts[ace2k_mains_edge_filter(&f, stall_to, &st)]++;
        }
        counts[ace2k_mains_edge_filter(&f, at, &st)]++;
    }
    ASSERT_EQ(counts[ACE2K_MAINS_EDGE_RESYNC], 1U);
    ASSERT_EQ(counts[ACE2K_MAINS_EDGE_REJECT_UNCOUNTED], 1U);
    ASSERT_EQ(counts[ACE2K_MAINS_EDGE_REJECT], 0U);
    ASSERT_TRUE(f.span.bridge <= 2U * 8333U); /* a period: no edge missed outside the stall */
}

/* Round 3's scenarios.  A dip (7 edges missing, the mains still present) and then a store 40 ms
 * later: the late edge closes the span that holds the dip, and the next span is clean and longer —
 * the dip's gap is a bridge: the publication is MISSED, not the longest span. */
TEST(a_dip_then_a_store_publishes_off_band)
{
    struct ace2k_mains m;
    struct ace2k_heat h;
    train_start(&m, HALF_US_60, 0);
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 600);
    T.off_us = 1300000U;
    T.on_us = 1360000U;
    now = ticks(&m, now, 1400);
    ASSERT_TRUE(m.present);
    now = stall(&m, now * US_PER_MS, 30000U);
    ASSERT_EQ(F.span.gen, 1U);
    for (uint32_t t = now + 10U; t <= 2000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
        ASSERT_TRUE(m.present); /* a dip, not an outage */
    }
    ASSERT_EQ(m.published_ms, 2000U);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED); /* the dip's gap, in the closed span */
    ASSERT_TRUE(h.mains_level > 0U);
    ticks(&m, 2000, 3000);
    ASSERT_EQ(m.hz10, 600);
}

/* A dip that a stall ends: the mains goes at 1 400, the store runs 1 460–1 475, the mains is back
 * at 1 470 — inside the stall, so the late edge carries the return.  The span before closes clean
 * at the dip's start; the gap from its last edge to the stall's start is the missed edges, far past
 * the bridge bound: the publication is ACE2K_MAINS_HZ10_MISSED. */
TEST(a_dip_a_stall_ends_publishes_the_missed_marker)
{
    struct ace2k_mains m;
    struct ace2k_heat h;
    train_start(&m, HALF_US_60, 0);
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.off_us = 1400000U;
    T.on_us = 1470000U;
    now = ticks(&m, now, 1460);
    ASSERT_TRUE(m.present);
    now = stall(&m, now * US_PER_MS, 15000U);
    ASSERT_EQ(F.span.gen, 1U); /* the return, held */
    ASSERT_TRUE(m.present);
    for (uint32_t t = now + 10U; t <= 2000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
        ASSERT_TRUE(m.present); /* a dip, not an outage */
    }
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    ASSERT_TRUE(!ace2k_mains_plausible(&m));
    ASSERT_TRUE(h.mains_level > 0U);
    ticks(&m, 2000, 3000);
    ASSERT_EQ(m.hz10, 600);
}

/* A dip that shortens a short span into the other band: 7 edges missing from a span of about 40
 * half-periods read 49.x Hz — plausible on its own.  It disagrees with the clean span after it
 * and with the 60 Hz published before: ACE2K_MAINS_HZ10_MISSED, never a 50 Hz reading of 60 Hz
 * mains. */
TEST(a_span_a_dip_moves_into_the_other_band_publishes_the_missed_marker)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.off_us = 1150000U;
    T.on_us = 1210000U;
    now = ticks(&m, now, 1330);
    now = stall(&m, now * US_PER_MS, 20000U);
    uint16_t closed = m.low; /* the closed span, judged at the catch-up tick */
    ticks(&m, now, 2000);
    ASSERT_TRUE(ace2k_mains_hz10_plausible(closed)); /* the closed span alone: in the 50 Hz band */
    ASSERT_TRUE(closed < ACE2K_MAINS_60_LOW_HZ10);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
}

/* One edge missing right after the swallowed one: the bridge from the swallowed real edge to the
 * next accepted is two half-periods — MISSED.  The same stall on intact mains: exact. */
TEST(an_edge_missed_behind_the_late_one_publishes_the_missed_marker)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    now = ticks(&m, now, 1400);
    now = stall(&m, now * US_PER_MS, 40000U);
    /* the edges after 1 440 000: the first falls in the late edge's lockout; drop the second */
    uint32_t first = T.next_us;
    T.off_us = first + T.half_us;
    T.on_us = T.off_us + 1U;
    ticks(&m, now, 2000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    struct ace2k_mains n;
    train_start(&n, HALF_US_60, 0);
    now = ticks(&n, 0, 1400);
    now = stall(&n, now * US_PER_MS, 40000U);
    ticks(&n, now, 2000);
    ASSERT_EQ(n.hz10, 600);
}

/* One edge missing among the span's last few, after the last tick measured it and before the
 * stall: no measurement covers them, and the bridge to the stall is short — the gap between the
 * two edges around the missing one closes the span as a bridge: MISSED. */
TEST(an_edge_missed_in_a_span_tail_before_a_stall_publishes_the_missed_marker)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1400);
    T.off_us = T.next_us; /* the first edge after the 1 400 ms tick */
    T.on_us = T.next_us + 1U;
    edges_until(T.next_us + T.half_us); /* the one after it, 1 416 610: before the stall */
    now = stall(&m, 1418000U, 20000U);
    ASSERT_EQ(F.span.gen, 1U);
    ticks(&m, now, 2000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
}

/* A dip in the open span while a longer, clean span closed earlier in the window: the worst is
 * published, and the dip is never left between two publications. */
TEST(a_dip_in_the_open_span_is_published_even_when_a_longer_span_closed)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    now = ticks(&m, now, 1700);
    now = stall(&m, now * US_PER_MS, 20000U);
    T.off_us = 1850000U;
    T.on_us = 1900000U;
    ticks(&m, now, 2000);
    ASSERT_TRUE(!ace2k_mains_plausible(&m));
}

/* Stalls every 50 ms, each holding an edge (a store loop gone wrong): no span ever reaches
 * ACE2K_MAINS_SPAN_MIN_STALE_HALF, so after ACE2K_MAINS_STALE_MS the stale marker is published —
 * never 0, never a frozen plausible value — and the heater's mains bucket fills to its latch. */
TEST(stalls_that_never_let_a_span_grow_publish_the_stale_marker_and_fill_the_bucket)
{
    struct ace2k_mains m;
    struct ace2k_heat h;
    train_start(&m, HALF_US_60, 0);
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 600);
    uint32_t stale_at = 0U;
    while (now < 8000U) {
        for (uint32_t k = 0; k < 4U; k++) {
            now += 10U;
            edges_until(now * US_PER_MS);
            ace2k_mains_tick(&m, now);
            heat_sees(&h, &m, now);
        }
        now = stall(&m, now * US_PER_MS, 10000U);
        heat_sees(&h, &m, now);
        ASSERT_TRUE(m.present);
        ASSERT_TRUE(m.hz10 != 0U);
        if (stale_at == 0U && m.hz10 == ACE2K_MAINS_HZ10_STALE) {
            stale_at = now;
        }
    }
    ASSERT_TRUE(stale_at >= 1000U + ACE2K_MAINS_STALE_MS);
    ASSERT_TRUE(stale_at < 1000U + ACE2K_MAINS_STALE_MS + ACE2K_MAINS_WINDOW_MS);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_STALE);
    ASSERT_EQ(h.mains_level, ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL);
    ASSERT_TRUE(ace2k_heat_mains_implausible_held(&h));
}

/* An input whose half-cycles alternate long and short (±1 ms): every measurement holds an even
 * number of half-periods, so the asymmetry cancels and every window reads exact — also between
 * stalls, where the spans hold odd numbers of edges. */
TEST(a_measurement_holds_an_even_number_of_half_periods)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    T.asym_us = 1000U;
    uint32_t now = ticks(&m, 0, 1000);
    for (uint32_t k = 0; k < 5U; k++) {
        now = ticks(&m, now, now + 1000U);
        ASSERT_EQ(m.hz10, 600);
    }
    for (uint32_t k = 0; k < 40U; k++) { /* a stall every 130 ms */
        now = ticks(&m, now, now + 120U);
        now = stall(&m, (now * US_PER_MS) + 100U, 9000U);
        ASSERT_TRUE(m.hz10 == 600 || m.hz10 == ACE2K_MAINS_HZ10_STALE);
    }
    ASSERT_EQ(m.hz10, 600); /* published from spans of 10–14 half-periods at the bound */
}

/* On the stale path a measurement needs ACE2K_MAINS_SPAN_MIN_STALE_HALF too: spans of at most 8
 * half-periods (a stall every 70 ms) publish the stale marker, not a short measurement. */
TEST(the_stale_path_needs_the_minimum_too)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_59_9, 0);
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, 599);
    while (now < 6000U) {
        now = ticks(&m, now, now + 60U);
        now = stall(&m, (now * US_PER_MS) + 100U, 9000U);
    }
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_STALE);
}

/* Two spans off-band in one window, the deeper first: the publication is the one furthest from the
 * bands, not the last judged.  (Mains at 56 then 57 Hz, no edge missed: a source off its band.) */
TEST(the_worst_is_the_measurement_furthest_from_the_bands)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 1000);
    T.half_us = 8929U; /* 56.0 Hz */
    now = ticks(&m, now, 1500);
    now = stall(&m, now * US_PER_MS, 20000U);
    T.half_us = 8772U; /* 57.0 Hz */
    ticks(&m, now, 2000);
    ASSERT_EQ(m.hz10, 562); /* the first span, 60 Hz edges at its start: 56.2, not 57 */
}

/* Every publication from the tick after from_ms up to to_ms is plausible. */
static bool all_published_plausible(struct ace2k_mains *m, uint32_t from_ms, uint32_t to_ms)
{
    bool all = true;
    for (uint32_t t = from_ms + 10U; t <= to_ms; t += 10U) {
        uint32_t before = m->published_ms;
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(m, t);
        if (m->published_ms != before && !ace2k_mains_plausible(m)) {
            all = false;
        }
    }
    return all;
}

/* A span between two writes too short to be judged (under ACE2K_MAINS_SPAN_MIN_STALE_HALF): one
 * edge missing in it — the 3rd to the 6th after the first write — is a gap of two half-periods
 * between two accepted edges, MISSED however short the span.  Without the missing edge: exact. */
TEST(an_edge_missed_in_a_short_span_between_two_writes_publishes_the_missed_marker)
{
    for (uint32_t k = 3; k <= 6U; k++) {
        struct ace2k_mains m;
        train_start(&m, HALF_US_60, 0);
        uint32_t now = ticks(&m, 0, 1400);
        now = stall(&m, now * US_PER_MS, 20000U);
        T.off_us = T.next_us + ((k - 1U) * T.half_us);
        T.on_us = T.off_us + 1U;
        now = ticks(&m, now, 1480);
        now = stall(&m, (now * US_PER_MS) + 100U, 20000U);
        ticks(&m, now, 2000);
        ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    }
    struct ace2k_mains n;
    train_start(&n, HALF_US_60, 0);
    uint32_t now = ticks(&n, 0, 1400);
    now = stall(&n, now * US_PER_MS, 20000U);
    now = ticks(&n, now, 1480);
    now = stall(&n, (now * US_PER_MS) + 100U, 20000U);
    ticks(&n, now, 2000);
    ASSERT_EQ(n.hz10, 600);
}

/* An edge missing among a span's last few, which no tick measured before the publication, then a
 * write right after it closes the span: the gap is judged when its closing edge comes, so a
 * publication is off-band — never a missed edge between two exact publications. */
TEST(an_edge_missed_just_before_a_publication_and_a_write_is_published)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    T.next_us += 7000U; /* edges at 7 000 + k × 8 333 µs */
    uint32_t now = ticks(&m, 0, 1960);
    T.off_us = 1990254U;
    T.on_us = T.off_us + 1U;
    bool before = all_published_plausible(&m, now, 2000);
    now = stall(&m, 2000050U, 20000U);
    bool after = all_published_plausible(&m, now, 3000);
    ASSERT_TRUE(!before || !after);
}

/* A bounce 3 ms after an edge E, E + 1 missing, then a write 16.5 ms after E that holds E + 2: the
 * bounce is refused and counted, and it is no edge of the mains — the bridge runs from E to the
 * write's start and, with the one before it, spans 26.5 ms, 1.32 periods; from the bounce the pair
 * would be 3 + 13.5 ms: MISSED.  With E + 1: exact. */
TEST(a_bounce_does_not_shorten_a_bridge)
{
    for (uint32_t missing = 0; missing <= 1U; missing++) {
        struct ace2k_mains m;
        train_start(&m, HALF_US_50, 0);
        T.bounce_us = 3000U;
        uint32_t now = ticks(&m, 0, 1400);
        uint32_t e = T.next_us; /* E, kept, and its bounce */
        if (missing != 0U) {
            T.off_us = e + T.half_us; /* E + 1 */
            T.on_us = T.off_us + 1U;
        }
        edges_until(e + 16000U);             /* E, E + 1 unless missing, their bounces */
        now = stall(&m, e + 16500U, 12000U); /* holds E + 2 */
        ticks(&m, now, 2000);
        ASSERT_EQ(m.hz10, missing != 0U ? ACE2K_MAINS_HZ10_MISSED : 500U);
    }
}

/* The zero-cross detector's two half-cycles may differ (its threshold off the mains' zero): at
 * 50 Hz up to ±3 ms, as far as the lockout admits (the short half at least ACE2K_ZEROCROSS_LOCKOUT_US);
 * at 60 Hz ±1 ms (±2.5 would put the short half inside the lockout: refused whatever the check).
 * Clean mains at the bands' edges, with a write every ~310 ms at a moving phase: never MISSED, and
 * every publication exact — a same-polarity interval (two consecutive gaps) is a full period
 * whatever the asymmetry. */
TEST(clean_mains_with_an_asymmetric_detector_never_reads_missed)
{
    const uint32_t halves[] = { 10204U, HALF_US_50, 9803U, 8474U, HALF_US_60, 8196U };
    const uint32_t asyms[] = { 0U, 1000U, 2500U, 3000U };
    uint32_t runs = 0U;
    for (size_t i = 0; i < sizeof halves / sizeof halves[0]; i++) {
        for (size_t j = 0; j < sizeof asyms / sizeof asyms[0]; j++) {
            if (halves[i] - asyms[j] < LOCKOUT_US) {
                continue; /* the short half inside the lockout: not a clean input */
            }
            runs++;
            struct ace2k_mains m;
            train_start(&m, halves[i], 0);
            T.asym_us = asyms[j];
            uint16_t exact = (uint16_t)(5000000U / halves[i]);
            uint32_t now = ticks(&m, 0, 1000);
            for (uint32_t k = 0; k < 3U; k++) {
                now = ticks(&m, now, now + 1000U);
                ASSERT_EQ(m.hz10, exact);
            }
            for (uint32_t k = 0; k < 25U; k++) {
                now = ticks(&m, now, now + 300U);
                uint32_t start_us = (now * US_PER_MS) + ((k * 1730U) % 9000U);
                edges_until(start_us); /* the edges before the write came */
                now = stall(&m, start_us, 9000U);
                ASSERT_EQ(m.hz10, exact);
            }
            now = ticks(&m, now, now + 2000U);
            ASSERT_EQ(m.hz10, exact);
        }
    }
    ASSERT_EQ(runs, 17U);
}

/* One edge missing from an asymmetric train — after a short half and after a long one: the
 * same-polarity interval over it is a period and a half-cycle, at least a period and a lockout,
 * past 125 % of the period — MISSED; the next window is exact again. */
TEST(a_missed_edge_with_an_asymmetric_detector_reads_missed)
{
    const uint32_t halves[] = { HALF_US_50, 10204U, HALF_US_60 };
    const uint32_t asyms[] = { 3000U, 3000U, 1000U };
    for (size_t i = 0; i < sizeof halves / sizeof halves[0]; i++) {
        for (uint32_t parity = 0; parity <= 1U; parity++) {
            struct ace2k_mains m;
            train_start(&m, halves[i], 0);
            T.asym_us = asyms[i];
            uint16_t exact = (uint16_t)(5000000U / halves[i]);
            uint32_t now = ticks(&m, 0, 2000);
            ASSERT_EQ(m.hz10, exact);
            now = ticks(&m, now, 2400);
            if ((T.n & 1U) != parity) {
                advance_fed();
            }
            T.off_us = T.next_us;
            T.on_us = T.next_us + 1U;
            now = ticks(&m, now, 3000);
            ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
            ticks(&m, now, 4000);
            ASSERT_EQ(m.hz10, exact);
        }
    }
}

/* With no plausible value to judge by (none yet, or the last publication MISSED), 60 Hz mains
 * missing one edge: the check's reference must still catch it — a 49 Hz reference would admit
 * 60 Hz's period plus a half (25 ms) as one clean period (20.4 ms × 125 %). */
TEST(a_missed_edge_at_sixty_hertz_reads_missed_with_no_plausible_reference)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    T.off_us = 500000U;
    T.on_us = T.off_us + T.half_us - 1U; /* one edge */
    uint32_t now = ticks(&m, 0, 1000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED); /* the first publication */
    T.off_us = 1500000U;
    T.on_us = T.off_us + T.half_us - 1U;
    now = ticks(&m, now, 2000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED); /* after a MISSED */
    ticks(&m, now, 3000);
    ASSERT_EQ(m.hz10, 600);
}

/* A write between two long half-cycles (50 Hz, ±3 ms): it starts 12.9 ms after an edge A, holds
 * the next two edges (A + 13 and A + 20 ms), and ends 12.9 ms before the one after (A + 33).  The
 * gaps on its two sides are both long halves, 25.8 ms together — more than 125 % of a period; the
 * stall hid how many edges it held, so the gap after it is paired with the lockout, never with the
 * gap before it: exact, never MISSED. */
TEST(a_write_between_two_long_half_cycles_reads_exact)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_50, 0);
    T.asym_us = 3000U;
    uint32_t now = ticks(&m, 0, 1400);
    for (uint32_t k = 0; k < 5U; k++) {
        uint32_t a = 0U;
        do { /* A: an edge a long half-cycle follows */
            a = T.next_us;
            edges_until(a);
        } while ((T.n & 1U) != 0U);
        now = stall(&m, a + 12900U, 7200U);
        ASSERT_TRUE(all_published_plausible(&m, now, now + 300U));
        now += 300U;
    }
    ASSERT_TRUE(all_published_plausible(&m, now, now + 2000U));
    ASSERT_EQ(m.hz10, 500);
}

/* A start after an outage waits for a window measured after the return:
 * the flag drops at the absent tick and rises with the first
 * plausible publication. */
TEST(the_mains_is_measured_again_only_by_a_plausible_window_after_an_outage)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 2000);
    ASSERT_TRUE(ace2k_mains_measured(&m));
    T.off_us = 3000000U;
    T.on_us = 3500000U;
    now = ticks(&m, now, 3200);
    ASSERT_TRUE(!m.present);
    ASSERT_TRUE(!ace2k_mains_measured(&m));
    now = ticks(&m, now, 3600);
    ASSERT_TRUE(m.present);
    ASSERT_TRUE(!ace2k_mains_measured(&m));
    uint32_t measured_at = 0U;
    uint32_t published = m.published_ms;
    bool early = false; /* measured before the first publication after the return */
    for (uint32_t t = now + 10U; t <= 7000U && measured_at == 0U; t += 10U) {
        ticks(&m, t - 10U, t);
        if (ace2k_mains_measured(&m)) {
            measured_at = t;
            early = m.published_ms == published;
        }
    }
    ASSERT_TRUE(measured_at != 0U);
    ASSERT_TRUE(!early);
    ASSERT_EQ(m.published_ms, measured_at);
    ASSERT_TRUE(ace2k_mains_plausible(&m));
}

/* After init nothing is measured until the first publication: every tick before it reads
 * unmeasured. */
TEST(the_mains_reads_unmeasured_from_init_to_the_first_publication)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    ASSERT_TRUE(!ace2k_mains_measured(&m));
    bool early = false;
    for (uint32_t t = 10U; t < ACE2K_MAINS_WINDOW_MS; t += 10U) {
        ticks(&m, t - 10U, t);
        if (ace2k_mains_measured(&m)) {
            early = true;
        }
    }
    ASSERT_TRUE(!early);
    ticks(&m, ACE2K_MAINS_WINDOW_MS - 10U, ACE2K_MAINS_WINDOW_MS);
    ASSERT_EQ(m.published_ms, ACE2K_MAINS_WINDOW_MS);
    ASSERT_TRUE(ace2k_mains_measured(&m));
}

TEST(a_short_dip_with_the_mains_present_keeps_it_measured)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 2000);
    T.off_us = 2500000U;
    T.on_us = 2560000U;
    ticks(&m, now, 4000);
    ASSERT_TRUE(ace2k_mains_measured(&m));
}

static uint32_t ace2k_boot_runs, ace2k_boot_unmeasured_plausible, ace2k_boot_period_used;

/* The first publication after a boot, the train starting at phase_us (its first edge after the
 * first tick when phase_us is large): plausible only when measured, and heat judges no edge
 * against an unmeasured period. */
static void boot_window(uint32_t half, uint32_t phase_us)
{
    struct ace2k_mains m;
    struct ace2k_heat h;
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    train_start(&m, half, 0);
    T.next_us += phase_us;
    for (uint32_t t = 10U; t <= 1000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
    }
    ASSERT_EQ(m.published_ms, 1000U);
    ace2k_boot_runs++;
    if (ace2k_mains_measured(&m)) {
        return;
    }
    if (ace2k_mains_plausible(&m)) {
        ace2k_boot_unmeasured_plausible++;
    }
    if (h.period_us != 0U) {
        ace2k_boot_period_used++;
    }
}

/* The first window after a boot holds the moments before the first edge, when the mains read
 * absent: it is counted (its edges over its length), not measured, and a count a few edges short
 * reads plausible but up to 1 Hz off (495 for 50 Hz, 595 for 60 Hz) — the whole misread budget
 * of the phase check (heat.h).  It publishes no plausible value: 0, none measured yet, and heat
 * takes no period from it; the next window measures. */
TEST(the_first_window_after_a_boot_publishes_no_unmeasured_plausible_value)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    ace2k_boot_runs = 0U;
    ace2k_boot_unmeasured_plausible = 0U;
    ace2k_boot_period_used = 0U;
    for (unsigned k = 0; k < 2U; k++) {
        for (uint32_t phase = 0U; phase < 30000U; phase += 250U) {
            boot_window(halves[k], phase);
        }
    }
    ASSERT_TRUE(ace2k_boot_runs > 0U);
    ASSERT_EQ(ace2k_boot_unmeasured_plausible, 0U);
    ASSERT_EQ(ace2k_boot_period_used, 0U);
}

static uint32_t ace2k_ret_runs, ace2k_ret_wrong, ace2k_ret_period_wrong, ace2k_ret_leased;

/* A lease asked while the mains reads plausible but unmeasured: counted when it is accepted. */
static void try_lease_unmeasured(struct ace2k_heat *h, const struct ace2k_mains *m, uint32_t t)
{
    if (!ace2k_mains_plausible(m) || ace2k_mains_measured(m)) {
        return;
    }
    if (ace2k_heat_lease(h, 20U, 2000U, t) == 0) {
        ace2k_ret_leased++;
        ace2k_heat_release(h);
    }
}

/* A 2.5 s outage ending at end_us, the train at phase_us: after the outage, every plausible
 * publication is the mains' frequency, heat never takes a period from any other, and no lease is
 * accepted on a plausible value that is not measured. */
static void outage_return_run(uint32_t half, uint32_t end_us, uint32_t phase_us)
{
    uint16_t nominal = half == HALF_US_60 ? 600U : 500U;
    struct ace2k_mains m;
    struct ace2k_heat h;
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    train_start(&m, half, 0);
    T.next_us += phase_us;
    T.off_us = 2500000U;
    T.on_us = end_us;
    uint32_t published = 0U;
    for (uint32_t t = 10U; t <= 7000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
        if (t > 2600U) {
            try_lease_unmeasured(&h, &m, t);
        }
        if (m.published_ms == published) {
            continue;
        }
        published = m.published_ms;
        if (t > 2500U && ace2k_mains_plausible(&m) && m.hz10 != nominal) {
            ace2k_ret_wrong++;
        }
        if (h.period_us != 0U && h.period_us != ACE2K_HEAT_HZ10_PERIOD_US / nominal) {
            ace2k_ret_period_wrong++;
        }
    }
    ace2k_ret_runs++;
}

/* The mains back just around a window boundary, at every phase: a window that read absent only
 * at its opening tick can still hold up to a tick with no edge — one edge short at 60 Hz reads
 * 59.5 Hz.  Such a count publishes none measured, never a plausible value off the mains'. */
TEST(an_outage_ending_near_a_window_boundary_publishes_no_wrong_plausible_value)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    ace2k_ret_runs = 0U;
    ace2k_ret_wrong = 0U;
    ace2k_ret_period_wrong = 0U;
    ace2k_ret_leased = 0U;
    for (unsigned k = 0; k < 2U; k++) {
        for (uint32_t phase = 0U; phase < 10000U; phase += 500U) {
            for (uint32_t end = 2990000U; end <= 3012000U; end += 500U) {
                outage_return_run(halves[k], end, phase);
            }
        }
    }
    ASSERT_TRUE(ace2k_ret_runs > 0U);
    ASSERT_EQ(ace2k_ret_wrong, 0U);
    ASSERT_EQ(ace2k_ret_period_wrong, 0U);
    ASSERT_EQ(ace2k_ret_leased, 0U);
}

static uint32_t ace2k_dip_runs_latched, ace2k_dip_runs_not_one, ace2k_dip_runs_wrong_hz10;

/* One dip of len_ms ending at end_us against a live 20 % lease renewed from 2 s; the run's
 * off-band publications and whether the lease latched. */
static void dip_run(uint32_t half, uint32_t len_ms, uint32_t end_us)
{
    uint16_t nominal = half == HALF_US_60 ? 600U : 500U;
    struct ace2k_mains m;
    struct ace2k_heat h;
    ace2k_heat_init(&h, &ace2k_nop_gate, 0);
    train_start(&m, half, 0);
    T.off_us = end_us - (len_ms * US_PER_MS);
    T.on_us = end_us;
    uint32_t off_band = 0U;
    uint32_t published = 0U;
    for (uint32_t t = 10U; t <= 7000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        heat_sees(&h, &m, t);
        if (t >= 2000U && h.state != ACE2K_HEAT_LATCHED) {
            (void)ace2k_heat_lease(&h, 20U, 2000U, t);
        }
        if (m.published_ms != published) {
            published = m.published_ms;
            if (!ace2k_mains_plausible(&m)) {
                off_band++;
            } else if (m.hz10 != nominal) {
                ace2k_dip_runs_wrong_hz10++; /* plausible, but not the mains' frequency */
            }
        }
    }
    if (h.state == ACE2K_HEAT_LATCHED) {
        ace2k_dip_runs_latched++;
    }
    if (off_band != 1U) {
        ace2k_dip_runs_not_one++;
    }
}

/* The dip's gap is over the bound paired with the gap before it (noted at the edge that ends the
 * dip) and with the gap after it (the next edge): a publication between the two made two off-band
 * windows, and a measurement still running from an edge before the gap made the next window read
 * it again — either way two windows in a row filled heat's bucket and latched the lease.
 * One gap, one off-band window, at every phase. */
TEST(a_dip_across_a_publication_is_one_off_band_window_and_never_latches_a_lease)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    ace2k_dip_runs_latched = 0U;
    ace2k_dip_runs_not_one = 0U;
    ace2k_dip_runs_wrong_hz10 = 0U;
    for (unsigned k = 0; k < 2U; k++) {
        for (uint32_t len_ms = 10U; len_ms <= 95U; len_ms += 5U) {
            for (uint32_t end_us = 3800000U; end_us <= 4200000U; end_us += 1000U) {
                dip_run(halves[k], len_ms, end_us);
            }
        }
    }
    ASSERT_EQ(ace2k_dip_runs_latched, 0U);
    ASSERT_EQ(ace2k_dip_runs_not_one, 0U);
    ASSERT_EQ(ace2k_dip_runs_wrong_hz10, 0U);
}

/* The train's edges the mains lacked, from its start to 9 s (its phase and dips as set). */
#define MISSED_MAX 512U
static uint32_t ace2k_missed_us[MISSED_MAX];
static uint32_t ace2k_missed_n;

static void missed_edges(void)
{
    ace2k_missed_n = 0U;
    for (uint32_t e = T.next_us; e <= 9000000U && ace2k_missed_n < MISSED_MAX; e += T.half_us) {
        if (!mains_on(e)) {
            ace2k_missed_us[ace2k_missed_n++] = e;
        }
    }
}

/* An edge the mains lacked in the window from from_ms to to_ms, at least a period before its end
 * (the gap it leaves is closed by an edge inside the window). */
static bool window_missed_an_edge(uint32_t from_ms, uint32_t to_ms)
{
    for (uint32_t i = 0U; i < ace2k_missed_n; i++) {
        uint32_t e = ace2k_missed_us[i];
        if (e >= from_ms * US_PER_MS && e + (2U * T.half_us) < to_ms * US_PER_MS) {
            return true;
        }
    }
    return false;
}

static uint32_t ace2k_recurring_runs, ace2k_recurring_windows;

/* Dips of len_us every every_us from from_us, on a train at phase_us: the publications of windows
 * that missed an edge and still read plausible. */
static uint32_t recurring_dips_run(uint32_t half, uint32_t phase_us, uint32_t from_us,
                                   uint32_t every_us, uint32_t len_us)
{
    struct ace2k_mains m;
    train_start(&m, half, 0);
    T.next_us += phase_us;
    T.dip_from_us = from_us;
    T.dip_every_us = every_us;
    T.dip_len_us = len_us;
    missed_edges();
    uint32_t published = 0U;
    uint32_t plausible = 0U;
    for (uint32_t t = 10U; t <= 6000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        if (m.published_ms == published) {
            continue;
        }
        uint32_t from_ms = published;
        published = m.published_ms;
        if (!window_missed_an_edge(from_ms, published)) {
            continue;
        }
        ace2k_recurring_windows++;
        if (ace2k_mains_plausible(&m)) {
            plausible++;
        }
    }
    ace2k_recurring_runs++;
    return plausible;
}

/* Recurring short dips — 10 ms every 150 ms (at 60 Hz a 10 ms dip always holds an edge, at 50 Hz
 * exactly one), and 20 ms — at every phase of the train and of the dips against the windows: a
 * window that missed an edge never publishes a plausible hz10. */
TEST(recurring_short_dips_never_publish_a_plausible_window)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    const uint32_t lens[] = { 10000U, 20000U };
    ace2k_recurring_runs = 0U;
    ace2k_recurring_windows = 0U;
    uint32_t plausible = 0U;
    for (unsigned k = 0; k < 2U; k++) {
        for (unsigned l = 0; l < 2U; l++) {
            for (uint32_t phase = 0U; phase < halves[k]; phase += 500U) {
                for (uint32_t from = 2000000U; from < 2150000U; from += 5000U) {
                    plausible += recurring_dips_run(halves[k], phase, from, 150000U, lens[l]);
                }
            }
        }
    }
    ASSERT_TRUE(ace2k_recurring_windows > ace2k_recurring_runs);
    ASSERT_EQ(plausible, 0U);
}

/* A dip and a flash operation that coincide, the operation anywhere from 60 ms before the dip's
 * start to 100 ms after it, at every phase of the train.  The operation hides how many edges it
 * held, and the gap after it is paired with the lockout instead of the real gap before it — so
 * the pair's first edge is approximate (mains.c bridge_note).  Pinned: a missed edge clear of the
 * operation by more than a half-period is always published off-band, never twice in a row (a
 * second window would latch a lease), and every plausible publication after the first window
 * (counted at boot) is the mains' own frequency. */
static uint32_t ace2k_sd_runs, ace2k_sd_hidden, ace2k_sd_twice, ace2k_sd_wrong;

/* A missed edge clear of the operation [s0_us, s0_us + slen_us) by more than a half-period. */
static bool missed_clear_of(uint32_t s0_us, uint32_t slen_us)
{
    for (uint32_t i = 0U; i < ace2k_missed_n; i++) {
        uint32_t e = ace2k_missed_us[i];
        if (e + T.half_us < s0_us || e > s0_us + slen_us + T.half_us) {
            return true;
        }
    }
    return false;
}

// NOLINTNEXTLINE(readability-identifier-naming)
struct pub_watch {
    uint32_t published, off_band, run, run_max;
    uint16_t nominal;
};

/* A new publication, counted: off-band ones and their longest run, plausible ones that are not
 * the mains' frequency (after the boot's first window). */
static void pub_see(struct pub_watch *w, const struct ace2k_mains *m)
{
    if (m->published_ms == w->published) {
        return;
    }
    w->published = m->published_ms;
    if (m->hz10 == 0U) {
        /* none measured (a count after a return): heat's bucket neither fills nor drains on it, so
         * it neither extends nor breaks a run of off-band windows */
        return;
    }
    if (!ace2k_mains_plausible(m)) {
        w->off_band++;
        w->run++;
        w->run_max = w->run > w->run_max ? w->run : w->run_max;
        return;
    }
    w->run = 0U;
    if (w->published > 1000U && m->hz10 != w->nominal) {
        ace2k_sd_wrong++;
    }
}

static void stall_dip_run(uint32_t half, uint32_t phase, uint32_t len_us, uint32_t s0_us,
                          uint32_t slen_us)
{
    const uint32_t end_us = 4000000U;
    struct pub_watch w = { .nominal = half == HALF_US_60 ? 600U : 500U };
    struct ace2k_mains m;
    train_start(&m, half, 0);
    T.next_us += phase;
    T.off_us = end_us - len_us;
    T.on_us = end_us;
    missed_edges();
    bool stalled = false;
    for (uint32_t t = 10U; t <= 6000U; t += 10U) {
        if (!stalled && t * US_PER_MS > s0_us) {
            stalled = true;
            edges_until(s0_us - 1U);
            t = stall(&m, s0_us, slen_us);
        } else {
            edges_until(t * US_PER_MS);
            ace2k_mains_tick(&m, t);
        }
        pub_see(&w, &m);
    }
    ace2k_sd_runs++;
    if (missed_clear_of(s0_us, slen_us) && w.off_band == 0U) {
        ace2k_sd_hidden++;
    }
    if (w.run_max > 1U) {
        ace2k_sd_twice++;
    }
}

TEST(a_dip_and_a_flash_operation_that_coincide_are_published_once_and_never_misread)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    const uint32_t lens[] = { 10000U, 30000U, 60000U, 90000U };
    const uint32_t stalls[] = { 5000U, 25000U, 45000U };
    ace2k_sd_runs = 0U;
    ace2k_sd_hidden = 0U;
    ace2k_sd_twice = 0U;
    ace2k_sd_wrong = 0U;
    for (unsigned k = 0; k < 2U; k++) {
        for (uint32_t phase = 0U; phase < halves[k]; phase += 1000U) {
            for (unsigned l = 0; l < 4U; l++) {
                for (unsigned s = 0; s < 3U; s++) {
                    for (uint32_t rel = 0U; rel <= 160000U; rel += 4000U) {
                        uint32_t s0 = 4000000U - lens[l] - 60000U + rel;
                        stall_dip_run(halves[k], phase, lens[l], s0, stalls[s]);
                    }
                }
            }
        }
    }
    ASSERT_TRUE(ace2k_sd_runs > 0U);
    ASSERT_EQ(ace2k_sd_hidden, 0U);
    ASSERT_EQ(ace2k_sd_twice, 0U);
    ASSERT_EQ(ace2k_sd_wrong, 0U);
}

TEST(a_second_dip_in_the_next_window_is_published_off_band_too)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 3000);
    T.off_us = 3930000U; /* a dip ending just before the publication at 4 s */
    T.on_us = 3990000U;
    now = ticks(&m, now, 4000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    T.off_us = 4400000U; /* a second dip, inside the next window */
    T.on_us = 4460000U;
    now = ticks(&m, now, 5000);
    ASSERT_TRUE(!ace2k_mains_plausible(&m));
    ticks(&m, now, 6000);
    ASSERT_EQ(m.hz10, 600);
}

static uint32_t ace2k_second_runs, ace2k_second_hidden;

/* A train at phase_us, a dip ending at end_us just before the publication at 4 s, then one edge
 * missed: the k-th of the train after the dip's return, when that one comes after the
 * publication.  True when some later window published off-band. */
static bool second_dropout_seen(uint32_t half, uint32_t phase_us, uint32_t end_us, uint32_t k)
{
    struct ace2k_mains m;
    train_start(&m, half, 0);
    T.next_us += phase_us;
    uint32_t now = ticks(&m, 0, 3000);
    T.off_us = end_us - 60000U;
    T.on_us = end_us;
    now = ticks(&m, now, 4000);
    /* the train's first edge back */
    uint32_t ret_us = phase_us + ((end_us - phase_us + half - 1U) / half * half);
    uint32_t miss_us = ret_us + (k * half);
    if (miss_us <= 4000000U || m.hz10 != ACE2K_MAINS_HZ10_MISSED) {
        return true; /* in the dip's own window, or the dip not yet published: not this case */
    }
    ace2k_second_runs++;
    T.off_us = miss_us - (half / 2U);
    T.on_us = miss_us + (half / 2U);
    uint32_t published = m.published_ms;
    bool seen = false;
    for (uint32_t t = now + 10U; t <= 6000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        if (m.published_ms != published) {
            published = m.published_ms;
            if (!ace2k_mains_plausible(&m)) {
                seen = true;
            }
        }
    }
    return seen;
}

/* A dip read MISSED at a publication, then a second, single missed edge right after the dip's
 * return.  The pair holding the dip's gap and the next one is skipped (already published); the
 * next pair, which holds the second dropout's gap, may be noted in the same 10 ms read — the
 * span hands over the longest pair of a read only, so the skipped one must not hide it.  At every
 * phase of the train against the ticks, for the first few edges after the return. */
TEST(a_single_edge_missed_just_after_a_published_dip_is_published_too)
{
    const uint32_t halves[] = { HALF_US_60, HALF_US_50 };
    ace2k_second_runs = 0U;
    ace2k_second_hidden = 0U;
    for (unsigned h = 0; h < 2U; h++) {
        for (uint32_t phase = 0U; phase < halves[h]; phase += 250U) {
            for (uint32_t end_us = 3975000U; end_us < 4000000U; end_us += 2500U) {
                for (uint32_t k = 1U; k <= 3U; k++) {
                    if (!second_dropout_seen(halves[h], phase, end_us, k)) {
                        ace2k_second_hidden++;
                    }
                }
            }
        }
    }
    ASSERT_TRUE(ace2k_second_runs > 0U);
    ASSERT_EQ(ace2k_second_hidden, 0U);
}

static uint32_t ace2k_asym_runs, ace2k_asym_gap_last, ace2k_asym_hidden;

/* The train's last three edges at or before t_us, from where it stands; the train is left as it
 * was. */
static void last_edges_until(uint32_t t_us, uint32_t out[3])
{
    struct ace2k_train saved = T;
    out[0] = out[1] = out[2] = 0U;
    while (T.next_us <= t_us) {
        out[0] = out[1];
        out[1] = out[2];
        out[2] = T.next_us;
        advance();
    }
    T = saved;
}

/* An asymmetric train at phase_us: the edge two before the last of the window that ends at 4 s
 * missed — the gap it leaves and the half-cycle after it both close inside that window — then the
 * first edge after the publication missed too, so the half-cycle after that one is the second
 * dropout's.  True when some later window published off-band. */
static bool asym_second_dropout_seen(uint32_t half, uint32_t asym, uint32_t phase_us)
{
    struct ace2k_mains m;
    train_start(&m, half, 0);
    T.asym_us = asym;
    T.next_us += phase_us;
    uint32_t now = ticks(&m, 0, 3000);
    uint32_t e[3];
    last_edges_until(4000000U, e);
    T.off_us = e[0];
    T.on_us = e[0] + 1U;
    now = ticks(&m, now, 4000);
    if (m.hz10 != ACE2K_MAINS_HZ10_MISSED) {
        return false;
    }
    if (m.missed_to == e[2]) {
        ace2k_asym_gap_last++; /* the published pair is the gap and the half-cycle after it */
    }
    ace2k_asym_runs++;
    T.off_us = T.next_us;
    T.on_us = T.next_us + 1U;
    uint32_t published = m.published_ms;
    bool seen = false;
    for (uint32_t t = now + 10U; t <= 6000U; t += 10U) {
        edges_until(t * US_PER_MS);
        ace2k_mains_tick(&m, t);
        if (m.published_ms != published) {
            published = m.published_ms;
            if (!ace2k_mains_plausible(&m)) {
                seen = true;
            }
        }
    }
    return seen;
}

/* On a detector with unequal half-cycles the longest pair over a missed edge is often the gap and
 * the half-cycle after it, so the MISSED publication's last edge is that half-cycle's end.  A
 * second dropout in the half-cycle after it makes two long pairs: one starts before that edge (it
 * holds the published half-cycle: skipped) and one starts at that edge exactly — a new gap, which
 * must be published.  At every phase of the train against the publication, both bands. */
TEST(a_second_dropout_right_after_a_published_gap_on_an_asymmetric_detector_is_published)
{
    const uint32_t halves[] = { HALF_US_50, HALF_US_60 };
    const uint32_t asyms[] = { 3000U, 1000U };
    ace2k_asym_runs = 0U;
    ace2k_asym_gap_last = 0U;
    ace2k_asym_hidden = 0U;
    for (unsigned h = 0; h < 2U; h++) {
        for (uint32_t phase = 0U; phase < 2U * halves[h]; phase += 250U) {
            if (!asym_second_dropout_seen(halves[h], asyms[h], phase)) {
                ace2k_asym_hidden++;
            }
        }
    }
    ASSERT_TRUE(ace2k_asym_runs > 0U);
    ASSERT_TRUE(ace2k_asym_gap_last > 0U);
    ASSERT_EQ(ace2k_asym_hidden, 0U);
}

/* A MISSED publication marks its gap published for that window and the next one, counted in
 * windows: a next window that publishes nothing (stalls kept every span short, hz10 kept) still
 * ends the mark, so it never stretches over a third window. */
TEST(the_published_gap_mark_ends_with_the_next_window_even_when_it_keeps_hz10)
{
    struct ace2k_mains m;
    train_start(&m, HALF_US_60, 0);
    uint32_t now = ticks(&m, 0, 3000);
    T.off_us = 3930000U; /* a dip ending just before the publication at 4 s */
    T.on_us = 3990000U;
    now = ticks(&m, now, 4000);
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    ASSERT_EQ(m.missed_hold, 1U);
    for (uint32_t k = 0; k < 3U; k++) { /* stalls every ~300 ms: no span reaches the minimum */
        now = ticks(&m, now, 4000U + (300U * (k + 1U)) - 30U);
        now = stall(&m, now * US_PER_MS, 15000U);
    }
    now = ticks(&m, now, 5000);
    ASSERT_EQ(m.published_ms, 4000U); /* the window kept hz10 */
    ASSERT_EQ(m.hz10, ACE2K_MAINS_HZ10_MISSED);
    ASSERT_EQ(m.missed_hold, 0U);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
