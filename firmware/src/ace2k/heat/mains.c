#include "heat/mains.h"

/* hz10 = 10 × half-periods / 2 / seconds = HZ10_HALF_MS × half-periods / ms: 10 (tenths of a
 * hertz) × 1 000 (ms a second) / 2 (half-periods a cycle). */
#define HZ10_HALF_MS 5000U
#define PERCENT      100U

void ace2k_mains_init(struct ace2k_mains *self, const struct ace2k_mains_ops *ops, void *ctx,
                      uint32_t now_ms, uint32_t ticks_per_ms)
{
    *self = (struct ace2k_mains){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    self->ticks_per_ms = ticks_per_ms;
    self->last_count = ops->zerocross_count(ctx);
    self->window_start_ms = now_ms;
    self->window_count = self->last_count;
    self->published_ms = now_ms;
}

/* An edge within the presence window.  Written as an early return: in C a `&&` result is an
 * int, and the lint refuses one assigned to a bool. */
static bool edge_recent(const struct ace2k_mains *self, uint32_t now_ms)
{
    if (!self->ever) {
        return false;
    }
    return ace2k_time_since(now_ms, self->last_edge_ms) <= ACE2K_MAINS_PRESENT_MS;
}

/* hz10 of `half` half-periods over `dt` clock units (units_per_ms a millisecond), truncated as
 * the count always was (a value just under a band's floor stays under it).  64-bit, saturated
 * before the narrowing store: a chattering input must read as an impossible frequency, never wrap
 * to a plausible one.  half < 2^32 and HZ10_HALF_MS × units_per_ms ≤ 2^30 for any clock up to
 * 200 MHz: the product fits. */
static uint16_t hz10_from(uint32_t half, uint32_t dt, uint32_t units_per_ms)
{
    if (dt == 0U) {
        return UINT16_MAX;
    }
    uint64_t num = (uint64_t)half * HZ10_HALF_MS * units_per_ms;
    uint64_t hz10 = num / dt;
    return hz10 > UINT16_MAX ? UINT16_MAX : (uint16_t)hz10;
}

/* The base: the span's edge the next measurement runs from — the span's latest, when it has one. */
static void rebase(struct ace2k_mains *self, const struct ace2k_mains_span *s)
{
    self->base = *s;
    self->base_valid = s->edges != 0U;
    self->cur_half = 0U;
    self->cur_dt = 0U;
}

/* How far hz10 lies below low_hz10 or above high_hz10, in 0.1 Hz (0 between them). */
static uint32_t outside(uint16_t hz10, uint16_t low_hz10, uint16_t high_hz10)
{
    if (hz10 < low_hz10) {
        return (uint32_t)low_hz10 - hz10;
    }
    return hz10 > high_hz10 ? (uint32_t)hz10 - high_hz10 : 0U;
}

/* How far hz10 lies from the nearer band, in 0.1 Hz (0 inside one). */
static uint32_t off_band(uint16_t hz10)
{
    uint32_t fifty = outside(hz10, ACE2K_MAINS_50_LOW_HZ10, ACE2K_MAINS_50_HIGH_HZ10);
    uint32_t sixty = outside(hz10, ACE2K_MAINS_60_LOW_HZ10, ACE2K_MAINS_60_HIGH_HZ10);
    return fifty < sixty ? fifty : sixty;
}

/* The span's measurement so far, judged when it holds ACE2K_MAINS_SPAN_MIN_STALE_HALF (a missed
 * edge needs no measurement: every gap is a bridge, above): off-band, it is the worst when it lies
 * further from the bands than any before it (the deepest dip); in band, it widens the range the
 * window's measurements span; the longest is kept for the publication. */
static void judge(struct ace2k_mains *self)
{
    if (self->cur_half < ACE2K_MAINS_SPAN_MIN_STALE_HALF) {
        return;
    }
    uint16_t hz10 = hz10_from(self->cur_half, self->cur_dt, self->ticks_per_ms);
    if (!ace2k_mains_hz10_plausible(hz10)) {
        if (self->worst == 0U || off_band(hz10) > off_band(self->worst)) {
            self->worst = hz10;
        }
    } else {
        self->low = self->low == 0U || hz10 < self->low ? hz10 : self->low;
        self->high = hz10 > self->high ? hz10 : self->high;
    }
    if (self->cur_half > self->best_half) {
        self->best_half = self->cur_half;
        self->best_dt = self->cur_dt;
    }
}

/* The range starts at the value just published, when it is a frequency. */
static void since_publication_clear(struct ace2k_mains *self)
{
    self->best_half = 0U;
    self->best_dt = 0U;
    self->worst = 0U;
    self->bridge = 0U;
    bool band = ace2k_mains_plausible(self);
    self->low = band ? self->hz10 : 0U;
    self->high = band ? self->hz10 : 0U;
}

/* True once now_ticks has reached at_ticks on the span clock — ace2k_time_after() in the span
 * clock's units, wrap-safe while the two are less than 2^31 ticks apart. */
static bool ticks_after(uint32_t now_ticks, uint32_t at_ticks)
{
    return (int32_t)(now_ticks - at_ticks) >= 0;
}

/* A pair sharing its first gap with the pair the last MISSED publication reported (its first
 * edge strictly before that pair's last — ticks_after() is inclusive, so a pair that starts at
 * that edge is a new one): the same gap, already published. */
static bool bridge_published(const struct ace2k_mains *self, uint32_t from_ticks)
{
    if (self->missed_hold == 0U) {
        return false;
    }
    if (ticks_after(from_ticks, self->missed_to)) {
        return false;
    }
    return true;
}

static void fold_pair(struct ace2k_mains *self, uint32_t pair, uint32_t from, uint32_t to)
{
    if (pair <= self->bridge || bridge_published(self, from)) {
        return;
    }
    self->bridge = pair;
    self->bridge_to = to;
}

/* The bridges since the last tick, folded into the window's longest, those already published
 * left out.  The longest of the read may be one (the second pair of a published gap); the
 * longest noted after it is folded too, so a dropout noted in the same read is not hidden. */
static void fold_bridge(struct ace2k_mains *self, const struct ace2k_mains_span *s)
{
    fold_pair(self, s->bridge, s->bridge_from, s->bridge_to);
    fold_pair(self, s->after, s->after_from, s->after_to);
}

/* Every tick: the bridges the interrupt measured since the last tick — every gap between two
 * edges — are folded in whatever else the tick does, then the span is followed.  Absent, the
 * window is marked, and publish() counts it instead of measuring it — a measurement over the gap
 * never reaches hz10 — and rebases on the first window end the mains is present at: an edge after
 * the return (present means an edge in the last 100 ms, so the span's latest is one).  A new span
 * (a resync edge closed the old one): the old one's measurement is judged, and the new one is
 * followed from its first edge on.  The measurement moves only on an even number of
 * half-periods. */
static void track(struct ace2k_mains *self, const struct ace2k_mains_span *s)
{
    fold_bridge(self, s);
    if (!self->present) {
        self->absent_in_window = true;
        return;
    }
    if (!self->base_valid || s->gen != self->base.gen) {
        judge(self);
        rebase(self, s);
        return;
    }
    uint32_t half = s->edges - self->base.edges;
    if (half == 0U || (half & 1U) != 0U) {
        return;
    }
    self->cur_half = half;
    self->cur_dt = s->last - self->base.last; /* unsigned: right across the clock's wrap */
}

/* The longest same-polarity interval (above) a missed edge cannot hide in: ACE2K_MAINS_BRIDGE_PCT
 * of the full period of the last published hz10, or of ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10's with none
 * plausible.  64-bit: the product is ~2^37 at 120 MHz. */
static uint32_t bridge_bound(const struct ace2k_mains *self)
{
    uint32_t hz10 = ace2k_mains_plausible(self) ? self->hz10 : ACE2K_MAINS_BRIDGE_UNKNOWN_HZ10;
    uint64_t num = (uint64_t)HZ10_HALF_MS * 2U * self->ticks_per_ms * ACE2K_MAINS_BRIDGE_PCT;
    return (uint32_t)(num / ((uint64_t)hz10 * PERCENT));
}

/* What a window the mains was present in publishes, or 0 to keep hz10: an edge missed at a
 * bridge; else the worst span; else an edge missed in some span — the measurements disagree
 * (a short span a dip shortened can read in the other band); else the longest measurement once it
 * holds the minimum, or — hz10 kept ACE2K_MAINS_STALE_MS already — a shorter one, or the stale
 * marker. */
static uint16_t measured(struct ace2k_mains *self, uint32_t now_ms)
{
    if (self->bridge > bridge_bound(self)) {
        self->missed_to = self->bridge_to;
        self->missed_hold = 2U; /* this window and the next one */
        self->skip_gap = true;
        return ACE2K_MAINS_HZ10_MISSED;
    }
    judge(self); /* the open span, as far as it goes */
    if (self->worst != 0U) {
        return self->worst;
    }
    if ((uint32_t)self->high - self->low > ACE2K_MAINS_AGREE_HZ10) {
        return ACE2K_MAINS_HZ10_MISSED;
    }
    if (self->best_half >= ACE2K_MAINS_SPAN_MIN_HALF) {
        return hz10_from(self->best_half, self->best_dt, self->ticks_per_ms);
    }
    if (ace2k_time_since(now_ms, self->published_ms) < ACE2K_MAINS_STALE_MS) {
        return 0U;
    }
    if (self->best_half != 0U) {
        return hz10_from(self->best_half, self->best_dt, self->ticks_per_ms);
    }
    return ACE2K_MAINS_HZ10_STALE;
}

/* The next measurement runs from where the open span's was measured to, so consecutive
 * publications tile the edges: no half-period between two of them is left unmeasured. */
static void rebase_measured(struct ace2k_mains *self)
{
    self->base.edges += self->cur_half;
    self->base.last += self->cur_dt;
    self->cur_half = 0U;
    self->cur_dt = 0U;
}

/* A window the mains was absent in, counted: its accepted edges over its length.  Its length holds
 * moments of no edge — before the boot's first edge, or up to a tick of them when the mains came
 * back just after the window opened — so the count can read a few edges short: when it still
 * reads plausible it is published as none measured (0), never as a frequency.  A count is no
 * measurement; the next window measures. */
static uint16_t counted(const struct ace2k_mains *self, uint32_t count, uint32_t now_ms)
{
    uint32_t elapsed_ms = ace2k_time_since(now_ms, self->window_start_ms);
    uint16_t hz10 = hz10_from(count - self->window_count, elapsed_ms, 1U);
    if (ace2k_mains_hz10_plausible(hz10)) {
        return 0U;
    }
    return hz10;
}

/* A window the mains was absent in is counted as a whole — its accepted edges over its length, as
 * a 1 s count reads an outage — and the next measurement runs from the span's latest edge;
 * otherwise measured() decides, and a window it keeps hz10 for publishes nothing. */
static void publish_value(struct ace2k_mains *self, const struct ace2k_mains_span *s,
                          uint32_t count, uint32_t now_ms)
{
    if (self->absent_in_window) {
        self->hz10 = counted(self, count, now_ms);
        if (self->present) {
            rebase(self, s);
        }
    } else {
        uint16_t hz10 = measured(self, now_ms);
        if (hz10 == 0U) {
            return;
        }
        self->hz10 = hz10;
        if (self->present && ace2k_mains_hz10_plausible(hz10)) {
            self->fresh = true;
        }
        if (self->skip_gap) {
            self->skip_gap = false;
            rebase(self, s); /* the next measurement starts past the gap */
        } else {
            rebase_measured(self);
        }
    }
    self->published_ms = now_ms;
    since_publication_clear(self);
}

/* Every ACE2K_MAINS_WINDOW_MS: the window's publication, if any; and the mark of a published gap
 * counts windows, not publications — it holds for the MISSED publication's window and the next
 * one, whether that one publishes or keeps hz10. */
static void publish(struct ace2k_mains *self, const struct ace2k_mains_span *s, uint32_t count,
                    uint32_t now_ms)
{
    publish_value(self, s, count, now_ms);
    if (self->missed_hold != 0U) {
        self->missed_hold--;
    }
}

void ace2k_mains_tick(struct ace2k_mains *self, uint32_t now_ms)
{
    uint32_t count = self->ops->zerocross_count(self->ctx);
    self->rejects = self->ops->zerocross_rejects(self->ctx);
    if (count != self->last_count) {
        self->last_count = count;
        self->last_edge_ms = now_ms;
        self->ever = true;
    }
    self->present = edge_recent(self, now_ms);
    if (!self->present) {
        self->fresh = false;
    }
    struct ace2k_mains_span s;
    self->ops->span(self->ctx, &s);
    track(self, &s);
    if (ace2k_time_since(now_ms, self->window_start_ms) >= ACE2K_MAINS_WINDOW_MS) {
        publish(self, &s, count, now_ms);
        self->window_start_ms = now_ms;
        self->window_count = count;
        self->absent_in_window = false;
        if (!self->present) {
            self->absent_in_window = true; /* this tick opens the next window too */
        }
    }
}

/* low_hz10 <= hz10 <= high_hz10, as an early return for the same reason as edge_recent(). */
static bool within(uint16_t hz10, uint16_t low_hz10, uint16_t high_hz10)
{
    if (hz10 < low_hz10) {
        return false;
    }
    return hz10 <= high_hz10;
}

bool ace2k_mains_hz10_plausible(uint16_t hz10)
{
    if (within(hz10, ACE2K_MAINS_50_LOW_HZ10, ACE2K_MAINS_50_HIGH_HZ10)) {
        return true;
    }
    return within(hz10, ACE2K_MAINS_60_LOW_HZ10, ACE2K_MAINS_60_HIGH_HZ10);
}

bool ace2k_mains_plausible(const struct ace2k_mains *self)
{
    return ace2k_mains_hz10_plausible(self->hz10);
}

bool ace2k_mains_measured(const struct ace2k_mains *self)
{
    return self->fresh;
}

bool ace2k_mains_edge_accept(bool ever, uint32_t last_ticks, uint32_t now_ticks,
                             uint32_t lockout_ticks)
{
    if (!ever) {
        return true;
    }
    return now_ticks - last_ticks >= lockout_ticks;
}

void ace2k_mains_edge_filter_init(struct ace2k_mains_edge_filter *f, uint32_t lockout,
                                  uint32_t stalls)
{
    *f = (struct ace2k_mains_edge_filter){ .lockout = lockout, .prev = lockout, .stalls = stalls };
}

/* A gap in the edges, noted: from `from` to `to`, with the gap before it — a same-polarity
 * interval — the longest since the last take, and the longest noted after that one.  Saturated:
 * an outage's gap never wraps short. */
static void bridge_note(struct ace2k_mains_edge_filter *f, uint32_t from, uint32_t to)
{
    uint32_t gap = to - from; /* unsigned: right across the clock's wrap */
    uint32_t pair = gap > UINT32_MAX - f->prev ? UINT32_MAX : gap + f->prev;
    if (pair > f->span.bridge) {
        f->span.bridge = pair;
        f->span.bridge_from = from - f->prev;
        f->span.bridge_to = to;
        f->span.after = 0U; /* every pair after the new longest is still to come */
    } else if (pair > f->span.after) {
        f->span.after = pair;
        f->span.after_from = from - f->prev;
        f->span.after_to = to;
    }
    f->prev = gap;
}

/* Every gap between two edges of the mains is a bridge: from `seen` — the last accepted edge, the
 * operation's end after a late one, or the real edge a late one's lockout swallowed — to the next
 * accepted edge, or to the start of an operation whose late edge comes; each is judged with the
 * gap before it (bridge_note).  The operation hides how many edges it held, so the gap after it is
 * paired with the lockout, not with the gap before it.  A counted reject (a bounce) is no edge of
 * the mains and moves nothing. */
enum ace2k_mains_edge ace2k_mains_edge_filter(struct ace2k_mains_edge_filter *f, uint32_t now,
                                              const struct ace2k_mains_stall *stall)
{
    bool after_resync = f->resynced;
    f->resynced = false;
    bool late = false; /* the first edge after an operation that held one pending */
    if (stall->count != f->stalls) {
        late = stall->held;
        f->stalls = stall->count;
    }
    if (!ace2k_mains_edge_accept(f->ever, f->last, now, f->lockout)) {
        if (!after_resync) {
            return ACE2K_MAINS_EDGE_REJECT;
        }
        bridge_note(f, f->seen, now);
        f->seen = now;
        return ACE2K_MAINS_EDGE_REJECT_UNCOUNTED;
    }
    if (f->ever) {
        bridge_note(f, f->seen, late ? stall->start : now);
    }
    f->seen = late ? stall->end : now;
    f->ever = true;
    f->last = now;
    if (late) {
        f->prev = f->lockout;
        f->resynced = true;
        f->span.gen++; /* the span ends; the next accepted edge starts the next */
        f->span.edges = 0U;
        return ACE2K_MAINS_EDGE_RESYNC;
    }
    f->span.edges++;
    f->span.last = now;
    return ACE2K_MAINS_EDGE_ACCEPT;
}

void ace2k_mains_edge_filter_take(struct ace2k_mains_edge_filter *f, struct ace2k_mains_span *out)
{
    *out = f->span;
    f->span.bridge = 0U;
    f->span.after = 0U;
}
