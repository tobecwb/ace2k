/* What: the triac gate's lease and the absolute limits — the one path to the heater.  Nothing
 * else in the image fires the gate.
 * How: the binding owns one struct ace2k_heat over the board's gate ops.  ace2k_heat_tick()
 * every 10 ms publishes a verdict on the inputs (cutout, the two outlet NTCs, the mains, the
 * fans, the duty), latches on a violation while leased, checks the gate is low outside a pulse,
 * ends an expired lease.  ace2k_heat_zerocross() runs in the zero-cross interrupt and decides
 * each edge on ONE word, fire_mode, read once: NONE fires nothing; LEASED fires inside a live
 * lease on an ok verdict, within ACE2K_HEAT_EDGES_PER_TICK_MAX edges of the last tick, deciding
 * per full mains cycle — an accumulator adds the duty on the even edge and fires both halves of
 * the cycle or neither, so the load draws no DC; PENDING fires only the odd half of a cycle whose
 * even half already fired, never an even one, then turns the word to NONE.  Every edge is
 * judged against the edge TWO before it — the same polarity, so an asymmetry between the input's
 * two half-cycles cancels — in any mode: it is in phase only when the gap is the period of the
 * measured mains frequency (period_us, from the tick's hz10) within ACE2K_HEAT_EDGE_TOL_US.  An
 * even edge out of phase — a noise spike the lockout let through ahead of the real zero-cross,
 * or an edge after a missed one — starts no cycle and is not counted as even, nor does an edge in
 * phase right after one out of phase (the odd edge would be judged against it); an odd edge out of
 * phase abandons its cycle before firing (a missed zero-cross puts it a period and a half after
 * the edge before the even one).  Each edge judged out of phase against a measured period is
 * counted (phase_rejects; the dryer raises a notice on them).  With no plausible frequency
 * measured, period_us is 0 and nothing fires.
 * The guarantee: a latch, ace2k_heat_abort() (a dryer fault), ace2k_heat_shutdown() and a tick
 * whose verdict is not ok make fire_mode = NONE their FIRST store — no edge after that store
 * fires anything, not even a pending odd half (a cycle begun before it stays a lone half).  An
 * explicit release (ace2k_heat_release(), or a lease of duty 0) turns LEASED into PENDING in one
 * store, then drops it to NONE if no even half of the cycle in progress fired: no edge after its
 * first store starts a half, and the odd half of a fired cycle still completes — both or
 * neither.  An outlet NTC reading invalid stops the firing at once (NONE) and latches through a
 * leaky bucket (ACE2K_HEAT_NTC_INVALID_*): five invalid ticks in a row, or an NTC invalid more
 * than one tick in five, sustained; a reading back before that resumes inside the live lease.
 * The mains read implausible (present, but outside 50 or 60 ± 1 Hz) stops the firing at once the
 * same way and fills its own leaky bucket (ACE2K_HEAT_MAINS_IMPLAUSIBLE_*), in every state: a
 * lease latches once it is full, a lease asked while it is not is refused as transient
 * (-ACE2K_EAGAIN), and one asked once it is full as out of bounds (-ACE2K_EBUSY).  The bucket
 * tolerates a mains off-band about one window in three, sustained (ACE2K_HEAT_MAINS_IMPLAUSIBLE_*
 * says exactly) — the firing happens only on the ticks whose verdict found the mains plausible;
 * the dryer counts the dips it rode out (dryer.h ACE2K_DRYER_NOTICE_MAINS_DIP).  An invalid NTC
 * is reported ahead of an implausible mains when both are.
 * Every other limit latches at once.
 * ace2k_heat_lease() is the only way in: at most ACE2K_HEAT_DUTY_MAX_PCT for at most
 * ACE2K_HEAT_LEASE_MAX_MS.  After ace2k_heat_shutdown() nothing is ever leased again until a
 * reset; while the binding sets the flash inhibit (a store in progress, rule 13) no new lease.
 * Contexts: the tick (timer interrupt, priority 2) and the lease callers (the same tick, or a
 * command with interrupts masked) write state, duty_pct, lease_end_ms, now_ms and fire_mode
 * (any value); the zero-cross interrupt (priority 1) reads fire_mode once per edge, then
 * duty_pct, lease_end_ms and now_ms — single aligned words written whole — writes fire_mode only
 * from PENDING to NONE, and owns acc, edge_parity, even_us, prev_edge_us, prev2_edge_us,
 * prev_in_phase, edges_seen, skip_edge, phase_rejects, fire_this_cycle, fired, last_fire_us,
 * fired_pending_check and
 * edges_since_tick; the tick writes period_us whole.  The tick writes the cycle fields
 * (cycle_reset) only once fire_mode is NONE, and a release reads edge_parity / fire_this_cycle
 * only after its PENDING store, when no edge can begin a cycle any more.
 * Depends on: <stdbool.h>, <stdint.h>, util.h; heat.c on mains.h (the plausible bands). */
#ifndef ACE2K_HEAT_H
#define ACE2K_HEAT_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

#define ACE2K_HEAT_DUTY_MAX_PCT       90U /* initial value; re-tune on the bench */
#define ACE2K_HEAT_LEASE_MAX_MS       2000U
#define ACE2K_HEAT_NTC_MAX_MC         85000 /* 85 °C: above the 73 °C drive ceiling */
#define ACE2K_HEAT_GATE_PULSE_US      6000U /* docs/hardware.md "Heater" */
#define ACE2K_HEAT_GATE_STUCK_US      8000U /* a 6 ms pulse still high 8 ms after its fire */
#define ACE2K_HEAT_EDGES_PER_TICK_MAX 3U    /* half-cycles fired without a tick, at most */
#define ACE2K_HEAT_CYCLE_GUARD_MS     30U   /* a cycle starts only this far inside the lease */
#define ACE2K_HEAT_PERCENT            100U
/* An edge is in phase when its gap to the edge two before it is the measured period within
 * ACE2K_HEAT_EDGE_TOL_US.  The period is 10 000 000 / hz10 µs: 16 667 at 60 Hz, 20 000 at
 * 50 Hz.  The budget: hz10 is measured from the edges' own times over an even number of
 * half-periods (mains.h) — exact to its 0.1 Hz truncation on intact mains, 28 µs of period at
 * 60 Hz and 40 µs at 50 Hz.  A span that lost an edge reads low; it is published only in band and
 * within ACE2K_MAINS_AGREE_HZ10 (1 Hz) of the window's other measurements and the value before,
 * so the misread stays within about 1 Hz: ±278 µs of period at 60 Hz, ±400 µs at 50 Hz (the
 * bound the 1 s edge count had, 118–122 edges on the bench's 60 Hz mains, docs/hardware.md
 * "Heater"); the two edges' interrupt entries add a few µs
 * of jitter (the irq_save sections of this image last microseconds; the gap is timed between the
 * entries, heat_cmds.c).  An asymmetry between the input's two half-cycles cancels: both edges
 * are of one polarity.  600 µs leaves 200 µs of margin at 50 Hz.  A noise edge the board's 7 ms
 * lockout lets through in place of the next zero-cross comes a half-period plus 7.0–7.2 ms after
 * the edge two before it: 1 133–1 333 µs short of a 60 Hz period, 2 800–3 000 µs of a 50 Hz one —
 * rejected even with the frequency read off by its whole step.  A noise edge closer to the true
 * zero-cross still passes: the phase error it can carry is the tolerance plus the frequency's
 * misread, about 0.9 ms at 60 Hz (600 + 278 µs) and 1.0 ms at 50 Hz (600 + 400 µs) — early, with
 * the polarity right (the real zero-cross it replaces is lost to the lockout). */
#define ACE2K_HEAT_EDGE_TOL_US    600U
#define ACE2K_HEAT_HZ10_PERIOD_US 10000000U /* µs × hz10: one period at hz10 / 10 Hz */
/* A pending odd half whose edge never came is dropped this long after its even edge: 1.25 × the
 * longest half-period the mains plausibility accepts (10 204 µs at 49 Hz). */
#define ACE2K_HEAT_PENDING_MAX_US 12500U
/* An invalid outlet NTC while leased fills a leaky bucket: each invalid tick adds
 * ACE2K_HEAT_NTC_INVALID_WEIGHT, each valid tick drains one, the latch comes at the level of
 * ACE2K_HEAT_NTC_INVALID_TICKS invalid ticks in a row (50 ms).  Sustained, an NTC invalid more
 * than one tick in WEIGHT + 1 fills it: 4 invalid / 1 valid latches on the 7th tick, a lone
 * excursion of 1–4 ticks never does. */
#define ACE2K_HEAT_NTC_INVALID_TICKS  5U /* ticks of 10 ms */
#define ACE2K_HEAT_NTC_INVALID_WEIGHT 4U /* bucket units per invalid tick; a valid tick drains 1 */
#define ACE2K_HEAT_NTC_INVALID_LEVEL  (ACE2K_HEAT_NTC_INVALID_TICKS * ACE2K_HEAT_NTC_INVALID_WEIGHT)
/* The mains frequency is measured over 1 s windows (mains.h ACE2K_MAINS_WINDOW_MS) and held
 * until the next, so one window read off-band — a burst of noise at the zero-cross input costs
 * edges to the lockout, observed on the bench at the first lease — is implausible for one window,
 * 100–101 ticks.  The mains' own bucket: +WEIGHT an implausible tick, −1 a plausible one, full at
 * the level of ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS implausible ticks in a row (1.5 windows).  A
 * lone off-band window fills it to at most 202 of 300 and never latches; two windows in a row
 * latch 1.5 s in; a mains off-band one window in two, sustained, latches in its second window.
 * The tolerance is about one window in three, sustained: +2 × 100 ticks against −1 × 200
 * balances and never latches; slightly less when the windows run long — a window lasts 100–101
 * ticks, and an off-band window one tick longer than its two plausible ones gains 2 units a period,
 * a latch after minutes.  More than one in three latches.  The firing happens only on the
 * plausible windows. */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS  150U /* ticks of 10 ms: 1.5 mains windows */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT 2U   /* per implausible tick; a plausible drains 1 */
#define ACE2K_HEAT_MAINS_IMPLAUSIBLE_LEVEL                                                         \
    (ACE2K_HEAT_MAINS_IMPLAUSIBLE_TICKS * ACE2K_HEAT_MAINS_IMPLAUSIBLE_WEIGHT)
/* A start (dryer.c), a lease from IDLE and the clear of a mains latch wait for the bucket at or
 * below the restart level: room left below the latch for one whole off-band window (a window lasts
 * 100–101 ticks) plus a tick of margin, so a new cycle always rides out one dip.  Nothing resets
 * the bucket. */
#define ACE2K_HEAT_MAINS_WINDOW_TICKS 101U /* mains.h's 1 s window at 10 ms ticks, a tick long */
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
    ACE2K_HEAT_FIRE_PENDING = 2, /* released mid-cycle: only that cycle's odd half, then NONE */
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

/* What the tick publishes for the interrupt; the binding fills it every 10 ms. */
struct ace2k_heat_inputs {
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    bool mains_present;
    /* the last 1 s window's frequency in 0.1 Hz, 0 = none measured; heat judges it plausible with
     * mains.h's bands (ace2k_mains_hz10_plausible) */
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
    volatile uint32_t lease_end_ms; /* the tick's ms clock */
    volatile uint32_t now_ms;       /* the tick's last ms, for the interrupt */
    volatile uint32_t fire_mode; /* enum ace2k_heat_fire: the one word the interrupt decides on */
    volatile uint32_t edges_since_tick;
    uint8_t limits;  /* the last tick's verdict on cutout, NTCs, mains */
    uint8_t verdict; /* limits, then fans, then duty */
    uint32_t acc;
    uint32_t edge_parity;            /* 1 between the even and the odd edge of a cycle */
    uint32_t even_us;                /* the even edge's time */
    uint32_t prev_edge_us;           /* the last edge the interrupt saw, in any mode */
    uint32_t prev2_edge_us;          /* the edge before it */
    uint8_t edges_seen;              /* 0, 1, then 2: the two above are meaningful */
    bool prev_in_phase;              /* the last edge was judged in phase */
    bool skip_edge;                  /* the next edge is a resync edge: kept out of the history */
    volatile uint32_t period_us;     /* the measured period, 0 = none plausible (the tick) */
    volatile uint32_t phase_rejects; /* edges judged out of phase since boot (the interrupt) */
    bool fire_this_cycle;
    uint8_t ntc_invalid_level; /* the invalid NTC's leaky bucket, while leased */
    uint16_t mains_level;      /* the implausible mains' leaky bucket, in every state */
    bool mains_measured;       /* the last tick's inputs: a window measured since the return */
    bool ntc_invalid;          /* the last tick's inputs: an outlet NTC reads invalid */
    volatile bool shut;        /* a Klipper shutdown: nothing is leased again until a reset */
    volatile bool inhibit;     /* the binding: a flash store in progress (rule 13) */
    volatile uint32_t fired;   /* half-cycles fired since boot */
    volatile uint32_t last_fire_us;
    volatile bool fired_pending_check; /* last_fire_us is meaningful */
    /* The cutout input was seen asserted at a tick, in any state, since init: sticky (rule 10 —
     * only a power-on reset clears a tripped cutout, whatever latched first). */
    bool cutout_seen;
};

void ace2k_heat_init(struct ace2k_heat *self, const struct ace2k_heat_ops *ops, void *ctx);

/* From IDLE or LEASED: 0, or -ACE2K_EINVAL (duty > 90, ms > 2000), -ACE2K_EREFUSED (shut down, or a
 * cutout seen since init), -ACE2K_ELATCHED (latched), -ACE2K_EAGAIN for every transient refusal —
 * the flash inhibit set (judged after a final verdict, which it never hides), the mains implausible
 * with its bucket not full, from IDLE the bucket above ACE2K_HEAT_MAINS_RESTART_LEVEL, an outlet
 * NTC invalid with no live lease (no tick yet among them): ask again — -ACE2K_EBUSY for every final
 * one (a rule-2/4 input out of bounds at the last tick), and a new lease (none live) while the
 * mains has no window measured since it last came back (inputs.mains_measured) is transient too —
 * except that a live lease is renewed through an outlet NTC's invalid excursion, an implausible
 * mains or an unmeasured one, which latch on their own or never fire (no period).
 * ace2k_heat_refusal_reason() names a refusal.  duty 0 or ms 0 = release. */
int ace2k_heat_lease(struct ace2k_heat *self, uint8_t duty_pct, uint32_t ms, uint32_t now_ms);

/* Why the last lease was or would be refused: the latch's reason; else the last tick's verdict
 * when not ok (MAINS_ABSENT among them); else MAINS_IMPLAUSIBLE while the mains bucket is above its
 * restart level, or with no live lease while the mains has no window measured since its return;
 * else OK — only the flash inhibit, a shutdown or a cutout seen since init and no longer asserted
 * (the -ACE2K_EREFUSED cases), or nothing. */
uint8_t ace2k_heat_refusal_reason(const struct ace2k_heat *self);

/* Back to IDLE at once, the gate off.  A cycle whose even half fired keeps its odd half
 * pending — fired at the next edge only on an ok verdict (the dryer's stop, cool-down, duty
 * 0). */
void ace2k_heat_release(struct ace2k_heat *self);

/* Back to IDLE at once, the gate off, and no half-cycle after the call — not even the odd half
 * of a cycle begun (the dryer's faults). */
void ace2k_heat_abort(struct ace2k_heat *self);

/* A Klipper shutdown: the lease gone with no pending half, the gate off, and every later lease
 * refused (-ACE2K_EREFUSED) until a reset. */
void ace2k_heat_shutdown(struct ace2k_heat *self);

/* The flash inhibit: while set, no new lease (the binding sets it for a store in progress). */
void ace2k_heat_set_inhibit(struct ace2k_heat *self, bool on);

/* Interrupt context, every accepted zero-cross edge. */
void ace2k_heat_zerocross(struct ace2k_heat *self, uint32_t now_us);

/* Interrupt context, right before ace2k_heat_zerocross() on a resync edge — an edge the zero-cross
 * interrupt took late, held pending through a flash operation (ace2k_mains_edge_filter()).  A flash
 * operation holds interrupts off for up to tens of ms: the edges in it are lost but one, which the
 * interrupt takes late, when the stall ends, with that time.  The edge history restarts as at
 * boot — the resync edge is kept out of it (not judged, not counted in phase_rejects, never an
 * edge to judge against), the next two fill it, the one after is the first judged and the one
 * after that the first that may start a cycle.  Were a cycle in progress, its next edge would be
 * judged out of phase and the cycle abandoned; no store runs while heat holds a lease or a
 * release is pending (the config binding's veto, rule 13), so that is defence in depth.  It can
 * only withhold firing, never allow it. */
void ace2k_heat_resync(struct ace2k_heat *self);

/* Every 10 ms: the verdict, rule 5, the cutout in every state, the latch, the lease's end. */
void ace2k_heat_tick(struct ace2k_heat *self, const struct ace2k_heat_inputs *in, uint32_t now_ms,
                     uint32_t now_us);

/* From LATCHED: 0 (IDLE), -ACE2K_EBUSY (ace2k_heat_clear_blocker() names a cause other than the
 * cutout), -ACE2K_EREFUSED (the cutout: only a power-on reset clears it).  From any other state: 0, nothing changes.  The clear leaves both buckets as they are.
 */
int ace2k_heat_clear(struct ace2k_heat *self);

/* What holds a latch, in this order: GATE_STUCK (the gate reading high: the heater may be on, as
 * ace2k_heat_active() says); CUTOUT (the reason, or a cutout seen since init); a limit still out
 * of bounds other than the mains implausible (the hard limits, an invalid NTC); MAINS_IMPLAUSIBLE
 * only for a latch on the mains (reason MAINS_IMPLAUSIBLE), while the bucket is above
 * ACE2K_HEAT_MAINS_RESTART_LEVEL — a latch for another cause is never held by the bucket; else OK.
 */
uint8_t ace2k_heat_clear_blocker(const struct ace2k_heat *self);

/* The mains bucket at or below ACE2K_HEAT_MAINS_RESTART_LEVEL: a new cycle has tolerance left. */
bool ace2k_heat_mains_recovered(const struct ace2k_heat *self);

bool ace2k_heat_leased(const struct ace2k_heat *self);

/* The last tick's mains_measured: a start waits for it. */
bool ace2k_heat_mains_measured(const struct ace2k_heat *self);

/* The mains implausible long enough to latch a lease (its bucket full): false again from the
 * first plausible tick. */
bool ace2k_heat_mains_implausible_held(const struct ace2k_heat *self);

/* The gate may fire at the next edge: leased and the last tick's verdict ok (fire_mode LEASED).
 * False through a debounced excursion — an invalid NTC, an implausible mains — that silences a
 * live lease. */
bool ace2k_heat_firing(const struct ace2k_heat *self);

/* The gate may still act: leased, an odd half pending, or — unless latched or shut down, where
 * the gate was commanded off for good — a fired pulse not yet seen over.  The flash is written
 * only while this is false (rule 13): a latched heat's log entry reaches it. */
bool ace2k_heat_busy(const struct ace2k_heat *self);

/* The heater may be conducting: busy, or the gate reading high in any state — a gate stuck high
 * is heat on, latched or not.  While true the fans are held on (rule 2, airflow's heat_holds)
 * and the bootloader is refused (rule 12). */
bool ace2k_heat_active(const struct ace2k_heat *self);

#endif
