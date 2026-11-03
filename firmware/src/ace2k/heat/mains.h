/* What: the frequency and the presence of the mains, from the zero-cross edges the board accepts.
 * How: the binding owns one struct ace2k_mains and ticks it every 10 ms. The interrupt filters
 * the edges.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_MAINS_H
#define ACE2K_MAINS_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

/* The contract.
 *
 * The binding owns one struct ace2k_mains, on top of ops that return the running edge count and a
 * snapshot of the interrupt's edge span. ace2k_mains_tick() runs every 10 ms. An edge within
 * the last 100 ms means that the mains is present.
 *
 * The frequency comes from the times of the edges themselves, not from the tick's clock. The
 * interrupt (ace2k_mains_edge_filter()) keeps a span: a run of accepted edges, their count and
 * the time of the last one. The tick measures the span in an even number of half-periods, over the
 * time they took. The input's two half-cycles may differ, and a pair cancels that. Every
 * ACE2K_MAINS_WINDOW_MS, the tick publishes hz10 from what it measured since the last publication.
 *
 * Flash stalls. A flash operation stalls the core with interrupts off, so the edges inside it are
 * lost, except one. At the operation's end, the flash layer samples the pending bit of EXTI
 * line 0. When it holds an edge, the interrupt's next edge is exactly that late edge. The
 * late edge ends the span. The next span starts at the first edge accepted after it. So a span
 * never holds a lost edge nor a late one, and a stall cannot shorten a measurement. A stall that
 * holds no edge changes nothing.
 *
 * Bridges. Every gap in the edges is a bridge: between two accepted edges, from the last edge of a
 * span to the stall's start, and from the late edge to the next accepted one. In the last
 * case the start may also be the real edge that the lockout of the late edge swallowed. A bridge
 * is judged together with the bridge before it. The two span an interval between edges of the same
 * polarity. On intact mains that interval is at most a full period, whatever the asymmetry of the
 * zero-cross detector. Its two half-cycles differ when its threshold sits off the zero of the
 * mains. This is unmeasured at 230 V, and a half-period bound would trip on clean 50 Hz mains.
 * After a stall, the bridge is paired with the lockout instead. The stall hides how many edges it
 * held, and any real half-cycle is at least a lockout long, because a shorter one is refused.
 * A pair longer than ACE2K_MAINS_BRIDGE_PCT of the period missed an edge, because a missed edge
 * makes a pair of at least a period and a lockout. The period is that of the last published
 * hz10, or that of ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10 when none is plausible. Then the next
 * publication is ACE2K_MAINS_HZ10_MISSED, however short the span it fell in.
 *
 * Otherwise a window publishes the worst it measured:
 * - Any span read off-band over at least ACE2K_MAINS_SPAN_MIN_STALE_HALF half-periods makes the
 *   publication the one furthest from the bands.
 * - Spans, together with the value published before, that disagree by more than
 *   ACE2K_MAINS_AGREE_HZ10 make it ACE2K_MAINS_HZ10_MISSED.
 * - Else the longest measurement is published, when it holds ACE2K_MAINS_SPAN_MIN_HALF.
 * - Short of that, hz10 is kept for at most ACE2K_MAINS_STALE_MS. Then the longest measurement of
 *   at least ACE2K_MAINS_SPAN_MIN_STALE_HALF is published, or ACE2K_MAINS_HZ10_STALE when there is
 *   none. So flash operations that never let a span grow fill the heater's mains bucket,
 *   rather than freeze a plausible value.
 * The gap of a short outage is a bridge, so a dip reads ACE2K_MAINS_HZ10_MISSED.
 * One gap is over the bound twice: paired with the gap before it, and with the gap after it. So a
 * pair whose first edge lies before the last edge of the pair that a MISSED publication reported
 * is that same gap. It is not counted again in the next window. After such a publication, the next
 * measurement starts at the span's latest edge, past the gap.
 *
 * Absent windows. A window in which the mains read absent (below) is counted as before the span
 * existed: its accepted edges over its length. So an outage reads as it always did: low, or 0 for
 * a window with no edge. It reads plausible again only from a measured window after the return.
 * Such a count holds moments of no edge: before the boot's first edge, or up to a tick of them
 * when the mains came back just after the window opened. So it can read a few edges short; one
 * edge short at 60 Hz reads 59.5 Hz. A count that still reads plausible publishes 0, none
 * measured. So heat takes no period from it, and the next window measures.
 *
 * Presence. Presence is an edge in the last 100 ms, and it is not filtered. So a stall over a real
 * outage still reads absent. The late edge is counted at the stall's catch-up tick, though. So
 * an outage that begins inside a stall reads absent up to the stall's length (tens of ms)
 * later than without it.
 *
 * The tick copies the interrupt's reject count into rejects. */

#define ACE2K_MAINS_WINDOW_MS    1000U
#define ACE2K_MAINS_PRESENT_MS   100U
#define ACE2K_MAINS_50_LOW_HZ10  490U
#define ACE2K_MAINS_50_HIGH_HZ10 510U
#define ACE2K_MAINS_60_LOW_HZ10  590U
#define ACE2K_MAINS_60_HIGH_HZ10 610U
/* The fewest half-periods a span publishes at the pace of the window: 918 ms of edges at 49 Hz,
 * 738 ms at 61 Hz. That is below any span between two publications a window apart with no stall
 * (about 1 s). It is long enough that the timing jitter of the edges (microseconds) is lost in
 * it. */
#define ACE2K_MAINS_SPAN_MIN_HALF 90U
/* hz10 is kept at most this long with no measurement of ACE2K_MAINS_SPAN_MIN_HALF, when flash
 * operations keep breaking the span. Then a shorter one is published, or ACE2K_MAINS_HZ10_STALE. */
#define ACE2K_MAINS_STALE_MS 3000U
/* The fewest half-periods any measurement is judged or published on. 10 half-periods are 83 ms at
 * 60 Hz. One edge missing among them reads 10 % low, far off-band. The timing jitter of the edges
 * (microseconds) is still lost in them. */
#define ACE2K_MAINS_SPAN_MIN_STALE_HALF 10U
/* A pair of bridges (above) longer than this share of the period missed an edge. On intact mains a
 * pair is at most one period. A gap before or after a stall ends at the stall, short of the real
 * edge. A missed edge makes a pair of at least a period and a lockout. The lockout (7 ms,
 * zerocross.h) is 35 % of a 50 Hz period and 42 % of a 60 Hz one. The 25 % covers the misread of
 * the period (hz10 truncated to 0.1 Hz: 0.2 %) and the interrupt's entry (µs). */
#define ACE2K_MAINS_BRIDGE_PCT 125U
/* The reference when no hz10 is plausible: none yet, or the last publication was a marker. It
 * works for either band. 125 % of the period of 57 Hz is 21.9 ms. That lies above the longest
 * clean pair (49 Hz: 20.4 ms), and below the shortest pair with an edge missed (the period of
 * 61 Hz plus a lockout: 23.4 ms). It is about 7 % from each. The value for 49 Hz (25.5 ms) would
 * pass 60 Hz mains missing an edge (at most 25 ms). */
#define ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10 570U
/* Published values that are not frequencies. Both are implausible and never 0. 0 means none
 * measured yet, which the heater does not count as off-band. MISSED is an edge missed at the
 * bridge of a stall. STALE is no measurement of ACE2K_MAINS_SPAN_MIN_STALE_HALF for
 * ACE2K_MAINS_STALE_MS. */
#define ACE2K_MAINS_HZ10_MISSED 1U
#define ACE2K_MAINS_HZ10_STALE  2U
/* Measurements of one publication that differ by more than this missed an edge. The measurements
 * are its spans, and the value published before when that is a frequency. The mains drifts by
 * hundredths of a hertz in seconds, and a measurement of ACE2K_MAINS_SPAN_MIN_STALE_HALF resolves
 * about 0.01 %. A short span that a dip shortened can read in the other band: a 60 Hz span 7 edges
 * short of 47 reads 50.2 Hz. This test tells it apart. */
#define ACE2K_MAINS_AGREE_HZ10 10U

/* A snapshot of the interrupt's edge span. A new span (gen moved) starts with no edge. */
struct ace2k_mains_span {
    uint32_t gen;    /* bumped by every resync edge: the span it ends is closed */
    uint32_t edges;  /* edges accepted in this span, running */
    uint32_t last;   /* the time of the last of them, in the clock's units */
    uint32_t bridge; /* longest bridge pair since the last read, in the clock's units (read: 0) */
    uint32_t bridge_from; /* the first edge of that pair */
    uint32_t bridge_to;   /* and its last */
    /* The longest pair noted after that one (read: 0). Pairs are noted in time order, so these
     * start after it. This is the pair to judge when the tick skips the longest as a gap already
     * published (fold_bridge in mains.c). So one read never hides a later pair behind it. */
    uint32_t after;
    uint32_t after_from;
    uint32_t after_to;
};

/* The last flash operation, as the flash layer records it with interrupts off. */
struct ace2k_mains_stall {
    uint32_t count; /* flash operations since boot, running */
    uint32_t start; /* the start of the last one, in the clock's units */
    uint32_t end;   /* and its end */
    bool held;      /* an edge was pending at its end: the interrupt's next edge is late */
};

struct ace2k_mains_ops {
    uint32_t (*zerocross_count)(void *ctx);   /* accepted edges, running */
    uint32_t (*zerocross_rejects)(void *ctx); /* edges refused by the lockout, running */
    /* the span, read whole; its bridge is handed over once (the copy in the interrupt cleared) */
    void (*span)(void *ctx, struct ace2k_mains_span *out);
};

struct ace2k_mains {
    const struct ace2k_mains_ops *ops;
    void *ctx;
    uint32_t ticks_per_ms; /* the span's clock */
    uint32_t last_count;
    uint32_t last_edge_ms;
    uint32_t window_start_ms;     /* the pace of the publications */
    uint32_t window_count;        /* the edge count at the window's start */
    uint32_t published_ms;        /* the last publication */
    bool absent_in_window;        /* the mains read absent at a tick of this window */
    struct ace2k_mains_span base; /* the span's edge that the next measurement runs from */
    bool base_valid;
    uint32_t cur_half;   /* the span's measurement from base: even half-periods */
    uint32_t cur_dt;     /* and the clock units they took */
    uint32_t best_half;  /* the longest measurement since the last publication: half-periods */
    uint32_t best_dt;    /* and the clock units they took */
    uint16_t worst;      /* off-band hz10 furthest from the bands since the publication, or 0 */
    uint16_t low, high;  /* the range of the in-band ones, and the last published (0: none) */
    uint32_t bridge;     /* the longest bridge pair since the last publication */
    uint32_t bridge_to;  /* its last edge */
    uint32_t missed_to;  /* the last edge of the pair the last MISSED publication reported */
    uint8_t missed_hold; /* windows that missed_to still applies to */
    bool skip_gap;       /* the publication reports a gap: the next measurement starts after it */
    uint32_t rejects;    /* the running reject count, as of the last tick */
    uint16_t hz10;
    bool present;
    bool ever;
    bool fresh; /* a plausible window published since the mains was last absent */
};

/* ticks_per_ms: the units of the span clock per millisecond. The board uses its timer, the tests
 * use µs. */
void ace2k_mains_init(struct ace2k_mains *self, const struct ace2k_mains_ops *ops, void *ctx,
                      uint32_t now_ms, uint32_t ticks_per_ms);

/* The 10 ms tick, in the timer interrupt. It counts the edges for presence, takes the span of the
 * interrupt, and publishes hz10 every ACE2K_MAINS_WINDOW_MS. */
void ace2k_mains_tick(struct ace2k_mains *self, uint32_t now_ms);

/* True for 50 or 60 Hz within ±1 Hz. */
bool ace2k_mains_plausible(const struct ace2k_mains *self);

/* True when a plausible frequency was published since boot, and since the mains last read absent.
 * It is false from the absent tick until the first plausible publication after the return. An
 * off-band window with the mains present, which is a dip, leaves it as it is. */
bool ace2k_mains_measured(const struct ace2k_mains *self);

/* The same test on a frequency alone, in 0.1 Hz. 0 means none measured, and is never plausible.
 * Pure: the heater judges with it the hz10 that its tick is handed. */
bool ace2k_mains_hz10_plausible(uint16_t hz10);

/* The interrupt's bounce filter, pure. True for the first edge ever (!ever). Otherwise true
 * when now_ticks is at least lockout_ticks past last_ticks. The test is unsigned, so a clock that
 * wraps between the two still compares right, as long as they are less than one wrap apart. Any
 * clock works: the board passes timer ticks, the tests microseconds. */
bool ace2k_mains_edge_accept(bool ever, uint32_t last_ticks, uint32_t now_ticks,
                             uint32_t lockout_ticks);

/* The whole decision of the zero-cross interrupt on an edge, pure: the lockout
 * (ace2k_mains_edge_accept) and the flash stalls. The interrupt hands the edge's time, and the
 * record the flash layer keeps of its last operation.
 * - The first edge after an operation that held one pending is that late edge. When accepted, it
 *   is a RESYNC edge: every consumer restarts there whatever it builds on the timing of the edges,
 *   and the span ends. When refused, it is a REJECT like any other: the operation ended within the
 *   lockout of the last accepted edge, so nothing was lost.
 * - An edge after an operation that held none came on time: ACCEPT.
 * - The real edge after a late one may fall inside the lockout that the late one started. That
 *   reject belongs to the stall, not to the input: REJECT_UNCOUNTED. Only the one edge right
 *   after a resync edge gets it.
 * - Every other refused edge is a REJECT, counted.
 * The bridges (above) are measured here, from the time of each edge. That time is an upper bound
 * of the time of the real edge, so a bridge is never read long on intact mains. A bridge runs
 * from each accepted edge to the next. At the late edge, one runs from the last edge to the start
 * of the operation. The next runs from the operation's end, through the uncounted reject, to
 * the next accepted edge; the real time of the late edge is no later than that end. Each bridge
 * is paired with the bridge before it, or with the lockout after the operation. A counted reject
 * is a bounce, not an edge of the mains, and it never ends a bridge. */
enum ace2k_mains_edge {
    ACE2K_MAINS_EDGE_ACCEPT,
    ACE2K_MAINS_EDGE_RESYNC,
    ACE2K_MAINS_EDGE_REJECT,
    ACE2K_MAINS_EDGE_REJECT_UNCOUNTED,
};

struct ace2k_mains_edge_filter {
    uint32_t lockout; /* in the clock's units */
    uint32_t last;    /* the time of the last accepted edge */
    uint32_t seen;    /* where the next bridge starts (above) */
    uint32_t prev;    /* the last bridge, or the lockout at the start and after a stall */
    uint32_t stalls;  /* the count of flash operations at the last edge */
    struct ace2k_mains_span span;
    bool ever;     /* an edge was ever accepted */
    bool resynced; /* the last edge was a resync edge */
};

/* Sets the filter with no edge seen yet. lockout is in the clock's units. stalls is the
 * operation count of the flash layer now, so a stall before the init is not taken for one that
 * the next edge waited out. Call it before the interrupt is enabled. From then on, the filter
 * belongs to the interrupt alone. */
void ace2k_mains_edge_filter_init(struct ace2k_mains_edge_filter *f, uint32_t lockout,
                                  uint32_t stalls);
enum ace2k_mains_edge ace2k_mains_edge_filter(struct ace2k_mains_edge_filter *f, uint32_t now,
                                              const struct ace2k_mains_stall *stall);
/* Hands the span to the tick, with its bridge: it is copied, then the copy in the filter is
 * cleared. */
void ace2k_mains_edge_filter_take(struct ace2k_mains_edge_filter *f, struct ace2k_mains_span *out);

#endif
