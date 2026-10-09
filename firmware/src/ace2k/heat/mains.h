/* What: the mains frequency and presence, from the zero-cross edges the board accepts.
 * How: the binding owns one struct ace2k_mains over ops that return the running edge count and
 * a snapshot of the interrupt's edge span; ace2k_mains_tick() every 10 ms: an edge within the last
 * 100 ms means present.  The frequency comes from the edges' own times, not from the tick's clock:
 * the interrupt (ace2k_mains_edge_filter()) keeps a span — a run of accepted edges, their count
 * and the last one's time — and the tick measures it in an even number of half-periods (the
 * input's two half-cycles may differ; a pair cancels that) over the time they took; every
 * ACE2K_MAINS_WINDOW_MS it publishes hz10 from what it measured since the last publication.
 * A flash operation stalls the core with interrupts off, so the edges in it are lost but one:
 * the flash layer samples EXTI line 0's pending bit at the operation's end, and when it holds an
 * edge, the interrupt's next edge is that late edge exactly.  The late edge ends the span, and the
 * next span starts at the first edge accepted after it — a span never holds a lost edge nor a
 * late one, so a stall cannot shorten a measurement; a stall that holds no edge changes nothing.
 * Every gap in the edges is a bridge: between two accepted edges, from a span's last edge to the
 * stall's start, and from the late edge (or the real edge its lockout swallowed) to the next
 * accepted one.  A bridge is judged with the one before it — the two span an interval between edges
 * of the same polarity, which on intact mains is at most a full period whatever the zero-cross
 * detector's asymmetry (its two half-cycles differ when its threshold sits off the mains' zero:
 * unmeasured at 230 V, and a half-period bound would trip on clean 50 Hz mains).  After a stall
 * the bridge is paired with the lockout instead: the stall hides how many edges it held, and any
 * real half-cycle is at least a lockout long (a shorter one is refused).  A pair longer than
 * ACE2K_MAINS_BRIDGE_PCT of the period (the last published hz10's, or
 * ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10's with none plausible) missed an edge — a missed edge makes one
 * of at least a period and a lockout — and the next publication is ACE2K_MAINS_HZ10_MISSED,
 * however short the span it fell in.  Otherwise a window publishes the worst it measured: any span
 * read off-band over at least ACE2K_MAINS_SPAN_MIN_STALE_HALF half-periods makes the publication
 * the one furthest from the bands, and spans (with the value published before) that disagree by
 * more than ACE2K_MAINS_AGREE_HZ10 make it ACE2K_MAINS_HZ10_MISSED; else the longest measurement
 * is published when it holds ACE2K_MAINS_SPAN_MIN_HALF.  Short of that hz10 is kept,
 * at most ACE2K_MAINS_STALE_MS, then the longest measurement of at least
 * ACE2K_MAINS_SPAN_MIN_STALE_HALF is published, or ACE2K_MAINS_HZ10_STALE when there is none — so
 * flash operations that never let a span grow fill the heater's mains bucket rather than freeze
 * a plausible value.  A short outage's gap is a bridge, so a dip reads ACE2K_MAINS_HZ10_MISSED.
 * One gap is over the bound twice — paired with the gap before it and with the gap after it — so
 * a pair whose first edge lies before the last edge of the pair a MISSED publication reported is
 * that same gap, not counted again in the next window; and after such a publication the next
 * measurement starts at the span's latest edge, past the gap.
 * A window the mains read absent in (below) is counted as before the span existed — its accepted
 * edges over its length — so an outage reads as it always did: low, or 0 for a window with no edge,
 * and plausible again only from a measured window after the return.  Such a count holds moments of
 * no edge — before the boot's first edge, or up to a tick of them when the mains came back just
 * after the window opened — and can read a few edges short (one at 60 Hz reads 59.5 Hz): a count
 * that still reads plausible publishes 0, none measured, so heat takes no period from it and the
 * next window measures.  Presence — an edge in the last 100 ms — is not filtered: a stall over a
 * real outage still reads absent.  The late edge is counted at the stall's catch-up tick, though,
 * so an outage that begins inside a stall reads absent up to the stall's length (tens of ms) later
 * than without it.  The tick copies the interrupt's reject count into rejects.
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_MAINS_H
#define ACE2K_MAINS_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_MAINS_WINDOW_MS    1000U
#define ACE2K_MAINS_PRESENT_MS   100U
#define ACE2K_MAINS_50_LOW_HZ10  490U
#define ACE2K_MAINS_50_HIGH_HZ10 510U
#define ACE2K_MAINS_60_LOW_HZ10  590U
#define ACE2K_MAINS_60_HIGH_HZ10 610U
/* The fewest half-periods a span publishes at the window's cadence: 918 ms of edges at 49 Hz,
 * 738 ms at 61 Hz — below any span between two publications a window apart with no stall (about
 * 1 s), and long enough that the edges' timing jitter (microseconds) is lost in it. */
#define ACE2K_MAINS_SPAN_MIN_HALF 90U
/* hz10 kept at most this long with no measurement of ACE2K_MAINS_SPAN_MIN_HALF (flash operations
 * that keep breaking the span); then a shorter one is published, or ACE2K_MAINS_HZ10_STALE. */
#define ACE2K_MAINS_STALE_MS 3000U
/* The fewest half-periods any measurement is judged or published on: 10 half-periods are 83 ms at
 * 60 Hz — one edge missing among them reads 10 % low, far off-band, and the edges' timing jitter
 * (microseconds) still lost in them. */
#define ACE2K_MAINS_SPAN_MIN_STALE_HALF 10U
/* A pair of bridges (above) longer than this share of the period missed an edge.  On intact mains
 * a pair is at most one period (a gap before or after a stall ends at the stall, short of the
 * real edge), and a missed edge makes one of at least a period and a lockout — the lockout
 * (7 ms, zerocross.h) is 35 % of a 50 Hz period and 42 % of a 60 Hz one.  The 25 % covers the
 * period's misread (hz10 truncated to 0.1 Hz: 0.2 %) and the interrupt's entry (µs). */
#define ACE2K_MAINS_BRIDGE_PCT 125U
/* The reference with no plausible hz10 (none yet, or the last publication a marker), for either
 * band: 125 % of 57 Hz's period, 21.9 ms, lies above the longest clean pair (49 Hz: 20.4 ms) and
 * below the shortest with an edge missed (61 Hz's period and a lockout: 23.4 ms), ~7 % from each.
 * 49 Hz's (25.5 ms) would pass 60 Hz mains missing an edge (at most 25 ms). */
#define ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10 570U
/* Published values that are not frequencies, both implausible and never 0 (0: none measured
 * yet, which the heater does not count as off-band): an edge missed at a stall's bridge, and no
 * measurement of ACE2K_MAINS_SPAN_MIN_STALE_HALF for ACE2K_MAINS_STALE_MS. */
#define ACE2K_MAINS_HZ10_MISSED 1U
#define ACE2K_MAINS_HZ10_STALE  2U
/* Measurements of one publication — its spans, and the value published before when it is a
 * frequency — that differ by more than this missed an edge: the mains drifts by hundredths of a
 * hertz in seconds, and a measurement of ACE2K_MAINS_SPAN_MIN_STALE_HALF resolves ~0.01 %.  A
 * short span a dip shortened can read in the other band (a 60 Hz span 7 edges short of 47 reads
 * 50.2 Hz): this is what tells it. */
#define ACE2K_MAINS_AGREE_HZ10 10U

/* The interrupt's edge span, as a snapshot: a new span (gen moved) starts with no edge. */
struct ace2k_mains_span {
    uint32_t gen;   /* bumped by every resync edge: the span it ends is closed */
    uint32_t edges; /* edges accepted in this span, running */
    uint32_t last;  /* the last of them's time, in the clock's units */
    uint32_t
        bridge; /* the longest bridge pair since the last read, in the clock's units (read: 0) */
    uint32_t bridge_from; /* that pair's first edge */
    uint32_t bridge_to;   /* and its last */
    /* The longest pair noted after that one (read: 0): pairs are noted in time order, so these
     * start after it — the one to judge when the tick skips the longest as a gap already
     * published (mains.c fold_bridge), so one read never hides a later pair behind it. */
    uint32_t after;
    uint32_t after_from;
    uint32_t after_to;
};

/* The last flash operation, as the flash layer records it with interrupts off. */
struct ace2k_mains_stall {
    uint32_t count; /* flash operations since boot, running */
    uint32_t start; /* the last one's start, in the clock's units */
    uint32_t end;   /* and its end */
    bool held;      /* an edge was pending at its end: the interrupt's next edge is late */
};

struct ace2k_mains_ops {
    uint32_t (*zerocross_count)(void *ctx);   /* accepted edges, running */
    uint32_t (*zerocross_rejects)(void *ctx); /* edges refused by the lockout, running */
    /* the span, read whole; its bridge handed over once (the interrupt's cleared) */
    void (*span)(void *ctx, struct ace2k_mains_span *out);
};

struct ace2k_mains {
    const struct ace2k_mains_ops *ops;
    void *ctx;
    uint32_t ticks_per_ms; /* the span's clock */
    uint32_t last_count;
    uint32_t last_edge_ms;
    uint32_t window_start_ms;     /* the publication cadence */
    uint32_t window_count;        /* the edge count at the window's start */
    uint32_t published_ms;        /* the last publication */
    bool absent_in_window;        /* the mains read absent at a tick of this window */
    struct ace2k_mains_span base; /* the span's edge the next measurement runs from */
    bool base_valid;
    uint32_t cur_half;   /* the span's measurement from base: even half-periods */
    uint32_t cur_dt;     /* and the clock's units they took */
    uint32_t best_half;  /* the longest measurement since the last publication: half-periods */
    uint32_t best_dt;    /* and the clock's units they took */
    uint16_t worst;      /* the off-band hz10 furthest from the bands since the publication, or 0 */
    uint16_t low, high;  /* the range of the in-band ones, and the last published (0: none) */
    uint32_t bridge;     /* the longest bridge pair since the last publication */
    uint32_t bridge_to;  /* its last edge */
    uint32_t missed_to;  /* the last edge of the pair the last MISSED publication reported */
    uint8_t missed_hold; /* windows missed_to still applies to */
    bool skip_gap;       /* the publication reports a gap: the next measurement starts after it */
    uint32_t rejects;    /* the running reject count, as of the last tick */
    uint16_t hz10;
    bool present;
    bool ever;
    bool fresh; /* a plausible window published since the mains was last absent */
};

/* ticks_per_ms: the span clock's units per millisecond (the board's timer, the tests' µs). */
void ace2k_mains_init(struct ace2k_mains *self, const struct ace2k_mains_ops *ops, void *ctx,
                      uint32_t now_ms, uint32_t ticks_per_ms);

/* Interrupt context. */
void ace2k_mains_tick(struct ace2k_mains *self, uint32_t now_ms);

/* 50 or 60 Hz within ±1 Hz. */
bool ace2k_mains_plausible(const struct ace2k_mains *self);

/* A plausible frequency was published since boot and since the mains last read absent: false
 * from the absent tick until the first plausible publication after the return.  An off-band
 * window with the mains present (a dip) leaves it as it is. */
bool ace2k_mains_measured(const struct ace2k_mains *self);

/* The same test on a frequency alone, in 0.1 Hz (0 = none measured: never plausible).  Pure: the
 * heater judges the hz10 its tick is handed with it. */
bool ace2k_mains_hz10_plausible(uint16_t hz10);

/* The interrupt's bounce filter, pure: true for the first edge ever (!ever), otherwise when
 * now_ticks is at least lockout_ticks past last_ticks — unsigned, so a clock that wraps between
 * the two still compares right as long as they are less than one wrap apart.  Any clock: the
 * board passes timer ticks, the tests microseconds. */
bool ace2k_mains_edge_accept(bool ever, uint32_t last_ticks, uint32_t now_ticks,
                             uint32_t lockout_ticks);

/* The zero-cross interrupt's whole decision on an edge, pure: the lockout (ace2k_mains_edge_accept)
 * and the flash stalls.  The interrupt hands the edge's time and the flash layer's record of its
 * last operation.  The first edge after an operation that held one pending is that late edge:
 * accepted, a RESYNC edge — every consumer restarts what it builds on the edges' timing there,
 * and the span ends; refused (the operation ended within the lockout of the last accepted edge, so
 * nothing was lost), a REJECT as any.  An edge after an operation that held none came on time:
 * ACCEPT.  The real edge after a late one may fall inside the lockout the late one started: that
 * reject is the stall's, not the input's — REJECT_UNCOUNTED, the one edge right after a resync
 * edge only.  Every other refused edge is a REJECT, counted.  The bridges (above) are measured
 * here, from each edge's time — an upper bound of the real edge's, so a bridge is never read long
 * on intact mains: from each accepted edge to the next; at the late edge, from the last edge to
 * the operation's start, then from the operation's end (the late edge's real time is no later),
 * through the uncounted reject, to the next accepted edge — each paired with the bridge before
 * it, or with the lockout after the operation.  A counted reject is a bounce, not an edge of the
 * mains: it never ends a bridge. */
enum ace2k_mains_edge {
    ACE2K_MAINS_EDGE_ACCEPT,
    ACE2K_MAINS_EDGE_RESYNC,
    ACE2K_MAINS_EDGE_REJECT,
    ACE2K_MAINS_EDGE_REJECT_UNCOUNTED,
};

struct ace2k_mains_edge_filter {
    uint32_t lockout; /* in the clock's units */
    uint32_t last;    /* the last accepted edge's time */
    uint32_t seen;    /* where the next bridge starts (above) */
    uint32_t prev;    /* the last bridge, or the lockout at the start and after a stall */
    uint32_t stalls;  /* the flash operations count at the last edge */
    struct ace2k_mains_span span;
    bool ever;     /* an edge was ever accepted */
    bool resynced; /* the last edge was a resync edge */
};

void ace2k_mains_edge_filter_init(struct ace2k_mains_edge_filter *f, uint32_t lockout,
                                  uint32_t stalls);
enum ace2k_mains_edge ace2k_mains_edge_filter(struct ace2k_mains_edge_filter *f, uint32_t now,
                                              const struct ace2k_mains_stall *stall);
/* The span for the tick, its bridge handed over: copied, then the filter's cleared. */
void ace2k_mains_edge_filter_take(struct ace2k_mains_edge_filter *f, struct ace2k_mains_span *out);

#endif
