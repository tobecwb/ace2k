/* What: the lease of the triac gate and its absolute limits. This is the one path to the heater.
 * How: the binding owns one struct ace2k_heat. It runs a 10 ms tick, handles each zero-cross edge,
 * and grants the lease.
 * Depends on: <stdbool.h>, <stdint.h>, util.h; heat.c also on mains.h (the plausible bands). */
#ifndef ACE2K_HEAT_H
#define ACE2K_HEAT_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

/* The contract.
 *
 * Nothing else in the image fires the gate. The binding owns one struct ace2k_heat, on top of the
 * board's gate ops.
 *
 * The tick. ace2k_heat_tick() runs every 10 ms. It publishes a verdict on the inputs: the cutout,
 * the two outlet NTCs, the mains, the fans and the duty. While leased, it latches on a violation.
 * It checks that the gate is low outside a pulse, and it ends an expired lease.
 *
 * The edges. ace2k_heat_zerocross() runs in the zero-cross interrupt. It decides each edge on ONE
 * word, fire_mode, read once:
 * - NONE fires nothing.
 * - LEASED fires inside a live lease, on an ok verdict, within ACE2K_HEAT_EDGES_PER_TICK_MAX edges
 *   of the last tick. It decides per full mains cycle. An accumulator adds the duty on the even
 *   edge, and fires both halves of the cycle or neither. So the load draws no DC.
 * - PENDING fires only the odd half of a cycle whose even half already fired, never an even half.
 *   Then it turns the word to NONE.
 *
 * The phase. In any mode, every edge is judged against the edge TWO before it. That edge has the
 * same polarity, so an asymmetry between the input's two half-cycles cancels. An edge is in
 * phase only when the gap is the period of the measured mains frequency, within
 * ACE2K_HEAT_EDGE_TOL_US. The period is period_us, which the tick computes from hz10.
 * - An even edge out of phase starts no cycle, and it is not counted as even. Such an edge is a
 *   noise spike that the lockout let through ahead of the real zero-cross, or an edge after a
 *   missed one.
 * - An edge in phase right after one out of phase starts no cycle either, because the odd edge
 *   would be judged against the edge out of phase.
 * - An odd edge out of phase abandons its cycle before it fires. A missed zero-cross puts the odd
 *   edge a period and a half after the edge before the even one.
 * Each edge judged out of phase against a measured period is counted in phase_rejects. The dryer
 * raises a notice on them. With no plausible frequency measured, period_us is 0 and nothing fires.
 *
 * The guarantee of the stop. These make fire_mode = NONE their FIRST store: a latch,
 * ace2k_heat_abort() (a dryer fault), ace2k_heat_shutdown(), and a tick whose verdict is not ok.
 * No edge after that store fires anything, not even a pending odd half. A cycle begun before the
 * store stays a lone half.
 * An explicit release is ace2k_heat_release(), or a lease of duty 0. It turns LEASED into PENDING
 * in one store. Then it drops the mode to NONE if no even half of the cycle in progress fired. No
 * edge after its first store starts a half. The odd half of a fired cycle still completes: both
 * halves or neither.
 *
 * The invalid NTC. An outlet NTC that reads invalid stops the firing at once (NONE). It latches
 * through a leaky bucket (ACE2K_HEAT_NTC_INVALID_*): after five invalid ticks in a row, or with an
 * NTC invalid more than one tick in five, sustained. A reading that comes back before that resumes
 * the firing inside the live lease.
 *
 * The implausible mains. The mains reads implausible when it is present but outside 50 or
 * 60 ± 1 Hz. This stops the firing at once in the same way. It also fills its own leaky bucket
 * (ACE2K_HEAT_MAINS_IMPLAUSIBLE_*), in every state.
 * - A lease latches once the bucket is full.
 * - A lease asked while the bucket is not full is refused as transient (-ACE2K_EAGAIN).
 * - A lease asked once the bucket is full is refused as out of bounds (-ACE2K_EBUSY).
 * The bucket tolerates a mains off-band about one window in three, sustained.
 * ACE2K_HEAT_MAINS_IMPLAUSIBLE_* gives the exact figure. The firing happens only on the ticks whose
 * verdict found the mains plausible. The dryer counts the dips it rode out
 * (ACE2K_DRYER_NOTICE_MAINS_DIP in dryer.h). When both happen, an invalid NTC is reported ahead
 * of an implausible mains.
 *
 * Every other limit latches at once.
 *
 * The lease. ace2k_heat_lease() is the only way in. It grants at most ACE2K_HEAT_DUTY_MAX_PCT for
 * at most ACE2K_HEAT_LEASE_MAX_MS. After ace2k_heat_shutdown(), nothing is ever leased again until
 * a reset. While the binding sets the flash inhibit, there is no new lease. The inhibit marks a
 * store in progress, because the flash is written only with the gate idle.
 *
 * The contexts.
 * - The tick runs in the timer interrupt at priority 2. The lease callers are the same tick, or a
 *   command with interrupts masked. These write state, duty_pct, lease_end_ms, now_ms and
 *   fire_mode, with any value.
 * - The zero-cross interrupt runs at priority 1. It reads fire_mode once per edge. Then it reads
 *   duty_pct, lease_end_ms and now_ms, which are single aligned words written whole. It writes
 *   fire_mode only from PENDING to NONE. It owns acc, edge_parity, even_us, prev_edge_us,
 *   prev2_edge_us, prev_in_phase, edges_seen, skip_edge, phase_rejects, fire_this_cycle, fired,
 *   last_fire_us and edges_since_tick.
 * - The tick writes period_us whole. It also sets edges_since_tick back to 0 at its end.
 *
 * The pulse check. The interrupt counts each fire in fired and stamps its time in last_fire_us.
 * It writes nothing else of the pulse check. Only the tick writes fired_seen_over. It stores there
 * a value of fired that it read before last_fire_us, once that fire is long over. A fire is open
 * while fired differs from fired_seen_over. An open fire keeps ace2k_heat_busy() true, except after
 * a latch or a shutdown. So a fire that lands during the check stays open.
 *
 * The cycle fields. The tick writes them (cycle_reset) only once fire_mode is NONE. A lease from
 * IDLE sets acc, edge_parity and fire_this_cycle back to zero under the same rule, fire_mode NONE.
 * A release reads edge_parity and fire_this_cycle only after its PENDING store, when no edge can
 * begin a cycle any more. */

#define ACE2K_HEAT_DUTY_MAX_PCT       90U   /* initial value; re-tune on the bench */
#define ACE2K_HEAT_LEASE_MAX_MS       2000U /* a design bound (docs/rules.md) */
#define ACE2K_HEAT_NTC_MAX_MC         85000 /* 85 °C: above the 73 °C drive ceiling */
#define ACE2K_HEAT_GATE_PULSE_US      6000U /* docs/hardware.md "Heater" */
#define ACE2K_HEAT_GATE_STUCK_US      8000U /* a 6 ms pulse still high 8 ms after its fire */
#define ACE2K_HEAT_EDGES_PER_TICK_MAX 3U    /* half-cycles fired without a tick, at most */
#define ACE2K_HEAT_CYCLE_GUARD_MS     30U   /* a cycle starts only this far inside the lease */
/* An edge is in phase when its gap to the edge two before it is the measured period, within
 * ACE2K_HEAT_EDGE_TOL_US. The period is 10 000 000 / hz10 µs: 16 667 at 60 Hz, 20 000 at 50 Hz.
 *
 * The budget of the tolerance:
 * - hz10 is measured from the times of the edges themselves, over an even number of half-periods
 *   (mains.h). On intact mains it is exact to its 0.1 Hz truncation: 28 µs of period at 60 Hz and
 *   40 µs at 50 Hz.
 * - A span that lost an edge reads low. It is published only in band, and only within
 *   ACE2K_MAINS_AGREE_HZ10 (1 Hz) of the window's other measurements and of the value before.
 *   So the misread stays within about 1 Hz: ±278 µs of period at 60 Hz, ±400 µs at 50 Hz. That
 *   is the same bound the 1 s edge count had: 118–122 edges on 60 Hz mains (docs/hardware.md
 *   "Heater").
 * - The interrupt entries of the two edges add a few µs of jitter. The irq_save sections of this
 *   image last microseconds, and the gap is timed between the entries (heat_cmds.c).
 * - An asymmetry between the input's two half-cycles cancels, because both edges have one
 *   polarity.
 * So 600 µs leaves 200 µs of margin at 50 Hz.
 *
 * A noise edge that the board's 7 ms lockout lets through, in place of the next zero-cross,
 * comes a half-period plus 7.0–7.2 ms after the edge two before it. That is 1 133–1 333 µs short
 * of a 60 Hz period, and 2 800–3 000 µs short of a 50 Hz one. It is rejected even when the
 * frequency is read off by its whole step. A noise edge closer to the true zero-cross still passes.
 * The phase error it can carry is the tolerance plus the frequency's misread: about 0.9 ms at
 * 60 Hz (600 + 278 µs) and 1.0 ms at 50 Hz (600 + 400 µs). Such an edge is early, with the right
 * polarity, and the real zero-cross it replaces is lost to the lockout. */
#define ACE2K_HEAT_EDGE_TOL_US    600U
#define ACE2K_HEAT_HZ10_PERIOD_US 10000000U /* µs × hz10: one period at hz10 / 10 Hz */
/* A pending odd half whose edge never came is dropped this long after its even edge. That is
 * about 1.225 × the longest half-period that the mains plausibility accepts (10 204 µs at
 * 49 Hz). */
#define ACE2K_HEAT_PENDING_MAX_US 12500U
/* While leased, an invalid outlet NTC fills a leaky bucket. Each invalid tick adds
 * ACE2K_HEAT_NTC_INVALID_WEIGHT, and each valid tick drains one. The latch comes at the level of
 * ACE2K_HEAT_NTC_INVALID_TICKS invalid ticks in a row (50 ms). An NTC invalid more than one tick
 * in WEIGHT + 1, sustained, fills it: 4 invalid and 1 valid latches on the 7th tick. A lone
 * excursion of 1–4 ticks never latches. */
#define ACE2K_HEAT_NTC_INVALID_TICKS  5U /* ticks of 10 ms */
#define ACE2K_HEAT_NTC_INVALID_WEIGHT 4U /* bucket units per invalid tick; a valid tick drains 1 */
#define ACE2K_HEAT_NTC_INVALID_LEVEL  (ACE2K_HEAT_NTC_INVALID_TICKS * ACE2K_HEAT_NTC_INVALID_WEIGHT)
/* The mains frequency is measured over 1 s windows (ACE2K_MAINS_WINDOW_MS in mains.h), and each
 * value is held until the next. So one window read off-band is implausible for one window, which
 * is 100–101 ticks. A burst of noise at the zero-cross input does this: it costs edges to the
 * lockout. This was observed on the unit.
 * The mains has its own bucket: +WEIGHT for an implausible tick, −1 for a plausible one. It is full
 * at the level of ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS implausible ticks in a row (1.5 windows).
 * - A lone off-band window fills it to at most 202 of 300, and never latches.
 * - Two off-band windows in a row latch 1.5 s in.
 * - A mains off-band one window in two, sustained, latches in its second off-band window.
 * The tolerance is about one window in three, sustained: +2 × 100 ticks against −1 × 200 balances
 * and never latches. It is slightly less when the windows run long. A window lasts 100–101 ticks.
 * An off-band window one tick longer than its two plausible ones gains 2 units a period, so it
 * latches after minutes. More than one window in three latches. The firing happens only in the
 * plausible windows. */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS  150U /* ticks of 10 ms: 1.5 mains windows */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT 2U   /* per implausible tick; a plausible drains 1 */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL                                                         \
    (ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT)
/* A start (dryer.c), a lease from IDLE and the clear of a mains latch wait until the bucket is at
 * or below the restart level. That level leaves room below the latch for one whole off-band
 * window, plus a tick of margin. A window lasts 100–101 ticks. So a new cycle always rides out one
 * dip. Nothing resets the bucket. */
#define ACE2K_HEAT_MAINS_WINDOW_TICKS 101U /* mains.h's 1 s window in 10 ms ticks, plus one */
#define ACE2K_HEAT_MAINS_RESTART_LEVEL                                                             \
    (ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL -                                                          \
     (ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT * (ACE2K_HEAT_MAINS_WINDOW_TICKS + 1U)))
_Static_assert(ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL >=
                   ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT * (ACE2K_HEAT_MAINS_WINDOW_TICKS + 1U),
               "the restart level is not negative");
_Static_assert(ACE2K_HEAT_MAINS_RESTART_LEVEL +
                       (ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT * ACE2K_HEAT_MAINS_WINDOW_TICKS) <
                   ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL,
               "one whole off-band window from the restart level stays below the latch");
#define ACE2K_HEAT_FANS_BOTH 0x3U /* fans_read: bit 0 left | bit 1 right */
_Static_assert(ACE2K_HEAT_NTC_INVALID_LEVEL + ACE2K_HEAT_NTC_INVALID_WEIGHT <= UINT8_MAX,
               "the bucket's level, one step past the latch, fits ntc_invalid_level (uint8_t)");
_Static_assert(ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL <= UINT16_MAX,
               "the mains bucket's level, saturated at the latch, fits mains_level (uint16_t)");

/* What the next edge may fire (fire_mode). */
enum ace2k_heat_fire {
    ACE2K_HEAT_FIRE_NONE = 0,    /* nothing */
    ACE2K_HEAT_FIRE_LEASED = 1,  /* leased on an ok verdict: whole cycles at the duty */
    ACE2K_HEAT_FIRE_PENDING = 2, /* released mid-cycle: only the odd half of it, then NONE */
};

enum ace2k_heat_state { ACE2K_HEAT_IDLE = 0, ACE2K_HEAT_LEASED = 1, ACE2K_HEAT_LATCHED = 2 };

enum ace2k_heat_reason {
    ACE2K_HEAT_OK = 0,
    ACE2K_HEAT_FANS = 1,
    ACE2K_HEAT_NTC_INVALID = 2,
    ACE2K_HEAT_NTC_OVER = 3,
    ACE2K_HEAT_MAINS_ABSENT = 4,
    ACE2K_HEAT_MAINS_IMPLAUSIBLE = 5,
    ACE2K_HEAT_CUTOUT = 6,
    ACE2K_HEAT_GATE_STUCK = 7,
    ACE2K_HEAT_DUTY = 8,
};

struct ace2k_heat_ops {
    void (*gate_fire)(void *ctx); /* gate high + the one-pulse timer of ACE2K_HEAT_GATE_PULSE_US */
    void (*gate_off)(void *ctx);  /* the timer stopped, gate low */
    bool (*gate_read)(void *ctx); /* the pin's input register */
};

/* What the tick publishes for the interrupt. The binding fills it every 10 ms. */
struct ace2k_heat_inputs {
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    bool mains_present;
    /* The frequency of the last 1 s window in 0.1 Hz; 0 means none measured. heat judges it
     * plausible with mains.h's bands (ace2k_mains_hz10_plausible). */
    uint16_t mains_hz10;
    bool mains_measured; /* ace2k_mains_measured(): a window measured since the mains came back */
    bool cutout;
    uint8_t fans_read; /* bit 0 left, bit 1 right: the pin reads high */
    bool fans_commanded;
};

struct ace2k_heat {
    const struct ace2k_heat_ops *ops;
    void *ctx;
    volatile uint8_t state;  /* enum ace2k_heat_state */
    volatile uint8_t reason; /* enum ace2k_heat_reason, the first latch */
    volatile uint8_t duty_pct;
    volatile uint32_t lease_end_ms; /* on the tick's ms clock */
    volatile uint32_t now_ms;       /* the tick's last ms, for the interrupt */
    volatile uint32_t fire_mode; /* enum ace2k_heat_fire: the one word the interrupt decides on */
    volatile uint32_t edges_since_tick;
    uint8_t limits;  /* the verdict of the last tick on cutout, NTCs, mains */
    uint8_t verdict; /* limits, then fans, then duty */
    volatile uint32_t acc;
    volatile uint32_t edge_parity;   /* 1 between the even and the odd edge of a cycle */
    volatile uint32_t even_us;       /* the time of the even edge */
    volatile uint32_t prev_edge_us;  /* the last edge the interrupt saw, in any mode */
    volatile uint32_t prev2_edge_us; /* the edge before it */
    volatile uint8_t edges_seen;     /* 0, 1, then 2: the two above are meaningful */
    volatile bool prev_in_phase;     /* the last edge was judged in phase */
    volatile bool skip_edge;         /* the next edge is a resync edge: kept out of the history */
    volatile uint32_t period_us;     /* the measured period, 0 = none plausible (the tick) */
    volatile uint32_t phase_rejects; /* edges judged out of phase since boot (the interrupt) */
    volatile bool fire_this_cycle;
    uint8_t ntc_invalid_level; /* the leaky bucket of the invalid NTC, while leased */
    uint16_t mains_level;      /* the leaky bucket of the implausible mains, in every state */
    bool mains_measured;       /* input of the last tick: a window measured since the return */
    bool ntc_invalid;          /* input of the last tick: an outlet NTC reads invalid */
    volatile bool shut;        /* a Klipper shutdown: nothing is leased again until a reset */
    volatile bool inhibit;     /* set by the binding: a flash store in progress */
    volatile uint32_t fired;   /* half-cycles fired since boot (the interrupt) */
    /* The time of the last fire, in µs (the interrupt). */
    volatile uint32_t last_fire_us;
    /* The value of fired when the tick last saw every pulse over (the tick). A fire is open, with
     * its pulse not yet seen over, while fired differs from it. */
    volatile uint32_t fired_seen_over;
    /* The cutout input was seen asserted at a tick, in any state, since init. It is sticky,
     * whatever latched first, because only a power-on reset clears a tripped cutout. */
    bool cutout_seen;
};

/* Sets IDLE, with the gate driven off. No inputs are seen yet, so a lease within bounds is
 * refused as transient (-ACE2K_EAGAIN) until the first tick. A lease out of bounds still gets
 * -ACE2K_EINVAL, and a duty or a time of 0 is a release. */
void ace2k_heat_init(struct ace2k_heat *self, const struct ace2k_heat_ops *ops, void *ctx);

/* Grants or renews a lease, from IDLE or LEASED. duty 0 or ms 0 is a release. Returns:
 * - 0 on success.
 * - -ACE2K_EINVAL: duty > 90 or ms > 2000.
 * - -ACE2K_EREFUSED: shut down, or a cutout seen since init.
 * - -ACE2K_ELATCHED: latched.
 * - -ACE2K_EAGAIN for every transient refusal. Ask again. The transient refusals are: the flash
 *   inhibit set; the mains implausible with its bucket not full; from IDLE, the bucket above
 *   ACE2K_HEAT_MAINS_RESTART_LEVEL; an outlet NTC invalid with no live lease, which includes no
 *   tick yet; and a new lease, with none live, while the mains has no window measured since it
 *   last came back (inputs.mains_measured). The flash inhibit is judged after a final verdict,
 *   and never hides it.
 * - -ACE2K_EBUSY for every final refusal: an input out of bounds at the last tick. That is the
 *   fan interlock (both fans commanded on and reading high) or an absolute limit.
 * One exception: a live lease is renewed through an invalid excursion of an outlet NTC, an
 * implausible mains or an unmeasured one. Those latch on their own, or never fire because there
 * is no period. ace2k_heat_refusal_reason() names a refusal. */
int ace2k_heat_lease(struct ace2k_heat *self, uint8_t duty_pct, uint32_t ms, uint32_t now_ms);

/* Why the last lease was refused, or would be. In this order:
 * - the latch's reason;
 * - else the verdict of the last tick when it is not ok, MAINS_ABSENT among them;
 * - else MAINS_IMPLAUSIBLE while the mains bucket is above its restart level, or, with no live
 *   lease, while the mains has no window measured since its return;
 * - else OK. That covers only the flash inhibit, a shutdown, a cutout seen since init and no
 *   longer asserted (the -ACE2K_EREFUSED cases), or nothing. */
uint8_t ace2k_heat_refusal_reason(const struct ace2k_heat *self);

/* Ends the lease at once (IDLE). No edge after the call starts a half-cycle. The gate is not
 * driven off: a pulse already started ends on its timer (TIM7, ACE2K_HEAT_GATE_PULSE_US). A cycle
 * whose even half fired keeps its odd half pending. That half fires at the next edge only on an
 * ok verdict. Used by the dryer's stop, its cool-down, and a duty of 0. */
void ace2k_heat_release(struct ace2k_heat *self);

/* Ends a lease at once (IDLE) and drives the gate off. No half-cycle fires after the call, not
 * even the odd half of a cycle begun. Used for the dryer's faults. A latch stays LATCHED. */
void ace2k_heat_abort(struct ace2k_heat *self);

/* The Klipper shutdown. The lease is gone with no pending half, and the gate is off. Every later
 * lease is refused (-ACE2K_EREFUSED) until a reset. */
void ace2k_heat_shutdown(struct ace2k_heat *self);

/* The flash inhibit. While it is set, there is no new lease. The binding sets it for a store in
 * progress. */
void ace2k_heat_set_inhibit(struct ace2k_heat *self, bool on);

/* Interrupt context, called on every accepted zero-cross edge. */
void ace2k_heat_zerocross(struct ace2k_heat *self, uint32_t now_us);

/* Interrupt context. Call it right before ace2k_heat_zerocross() on a resync edge. A resync edge
 * is an edge that the zero-cross interrupt took late, because it was held pending through a flash
 * operation (ace2k_mains_edge_filter()). A flash operation holds interrupts off for up to tens of
 * ms. The edges inside it are lost, except one. The interrupt takes that one late, when the stall
 * ends, with that time.
 * The edge history restarts as at boot. The resync edge is kept out of it: it is not judged, not
 * counted in phase_rejects, and never an edge to judge against. The next two edges fill the
 * history. The edge after them is the first judged, and the one after that is the first that may
 * start a cycle.
 * If a cycle were in progress, its next edge would be judged out of phase, and the cycle
 * abandoned. But no store runs while heat holds a lease or a release is pending: the veto of the
 * config binding writes the flash only with the gate idle. So this is defence in depth. It can
 * only withhold firing, never allow it. */
void ace2k_heat_resync(struct ace2k_heat *self);

/* Runs every 10 ms. It judges the verdict, checks that the gate is never left high, checks the
 * cutout in every state, latches, and ends the lease. */
void ace2k_heat_tick(struct ace2k_heat *self, const struct ace2k_heat_inputs *in, uint32_t now_ms,
                     uint32_t now_us);

/* Clears a latch. From LATCHED, returns 0 and goes to IDLE; or -ACE2K_EBUSY when
 * ace2k_heat_clear_blocker() names a cause other than the cutout; or -ACE2K_EREFUSED for the
 * cutout, which only a power-on reset clears. From any other state, returns 0 and nothing
 * changes. The clear leaves both buckets as they are. */
int ace2k_heat_clear(struct ace2k_heat *self);

/* What holds a latch, in this order:
 * - GATE_STUCK: the gate reads high, so the heater may be on, as ace2k_heat_active() says.
 * - CUTOUT: it is the reason, or a cutout was seen since init.
 * - A limit still out of bounds other than the implausible mains: the hard limits, an invalid
 *   NTC.
 * - MAINS_IMPLAUSIBLE: only for a latch on the mains (reason MAINS_IMPLAUSIBLE), while the bucket
 *   is above ACE2K_HEAT_MAINS_RESTART_LEVEL. The bucket never holds a latch for another cause.
 * - Else OK. */
uint8_t ace2k_heat_clear_blocker(const struct ace2k_heat *self);

/* True when the mains bucket is at or below ACE2K_HEAT_MAINS_RESTART_LEVEL: a new cycle has
 * tolerance left. */
bool ace2k_heat_mains_recovered(const struct ace2k_heat *self);

/* True when a lease is live (LEASED). It may still fire nothing for a while: see
 * ace2k_heat_firing(). */
bool ace2k_heat_leased(const struct ace2k_heat *self);

/* The mains_measured input of the last tick. A start waits for it. */
bool ace2k_heat_mains_measured(const struct ace2k_heat *self);

/* True when the mains has been implausible long enough to latch a lease: its bucket is full. It
 * is false again from the first plausible tick. */
bool ace2k_heat_mains_implausible_held(const struct ace2k_heat *self);

/* True when the gate may fire at the next edge: leased, and the verdict of the last tick ok
 * (fire_mode LEASED). False through a debounced excursion that silences a live lease, such as an
 * invalid NTC or an implausible mains. */
bool ace2k_heat_firing(const struct ace2k_heat *self);

/* True when the gate may still act: leased, an odd half pending, or a fired pulse not yet seen
 * over. The last one does not count after a latch or a shutdown, because the gate was then
 * commanded off for good. The flash is written only while this is false, since the flash is
 * written only with the gate idle. So the log entry of a latched heat still reaches the flash. */
bool ace2k_heat_busy(const struct ace2k_heat *self);

/* True when the heater may be conducting: busy, or the gate reads high in any state. A gate stuck
 * high is heat on, latched or not. While this is true, the fans are held on (heat_holds in
 * airflow), because the gate never fires without both fans. The bootloader is refused too,
 * because it drives no fans. */
bool ace2k_heat_active(const struct ace2k_heat *self);

#endif
