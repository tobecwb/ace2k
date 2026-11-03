/* What: the period of the periodic report of a binding, and its next deadline. It is one
 * scheduler for the report bindings: sensors at 10 Hz; airflow, env, feed-forward, lane, rfid,
 * mains, feed, dryer and the dryer's modes at 1 Hz; the lane's motor report at 1 Hz while the
 * host asks for it.
 * Health also uses it, for the deadline of its periodic re-evaluation every
 * ACE2K_HEALTH_PERIOD_MS (1 s), on phase 0. That pass sends no report.
 * How: the query handler calls ace2k_report_set() with the period the host asked for, and the
 * module's phase. A period of 0 stops the reports. The tick calls ace2k_report_due(), and
 * wakes its task on true.
 * Stalls: a due report moves the deadline whole periods ahead, until it is in the future. So
 * after a stall longer than one period, the missed reports are skipped, never replayed one per
 * tick, and the deadlines keep their grid. The board's ADC scan applies the same rule to its
 * passes.
 * The transmit rule: Klipper drops a frame that does not fit its 96-byte transmit buffer, on the
 * frame's real encoded length. So the bindings try each report and event instead (tx.h). They
 * hold one that did not fit for the next tick: a report by its due flag, an event in its ring. The
 * due report goes before the events. ace2k_report_try() is the one try of a due report, with the
 * flag's order.
 * The phases keep the bindings' periodic reports on distinct ticks, on periods that are
 * multiples of 100 ms: sensors 0, airflow 10, lane motor 20, env 30, feed-forward 40, lane 50,
 * rfid 60, mains 70, feed 80, dryer 90 ms. Health has no phase of its own: it uses 0, the same
 * tick as sensors. Every phase is claimed, so the dryer's modes report shares the dryer's phase:
 * the dryer's task sends both, the dryer's first. A held report leaves on the next tick, which is
 * the next binding's phase; it never waits a whole period. A report held at 90 ms so lands on
 * 0 ms, beside the sensors' report and the health pass.
 * The deadlines are anchored to the absolute 100 ms grid (ace2k_report_set), not to the tick the
 * query arrived on. So two queries from different ticks of the same 100 ms still land on their
 * phases. A deadline reseeded from a late tick would put two phased reports back on the same tick
 * for ever. That is why the whole-period rule exists.
 * The tick reads the pair in the timer interrupt, and the handler writes it from a task. So the
 * caller brackets ace2k_report_set() with irq_save() / irq_restore().
 * Depends on: <stdbool.h>, <stdint.h>, util.h. */
#ifndef ACE2K_REPORT_H
#define ACE2K_REPORT_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

struct ace2k_report {
    uint32_t rest_ms; /* 0: no reports */
    uint32_t next_ms;
};

#define ACE2K_REPORT_GRID_MS 100U
/* The longest period. ace2k_report_due() moves the deadline whole periods ahead, and its loop
 * ends only for a period under 2^31 ms, the range of ace2k_time_after(). Half of that leaves room
 * for the phase and the grid. That is about 12 days. The bindings take the period from a 32-bit
 * tick count, which is at most about 36 s at 120 MHz. So only a caller that passes a wrong value
 * meets the cap. */
#define ACE2K_REPORT_REST_MAX_MS ((uint32_t)INT32_MAX / 2U)

/* The period of a query, rest_ticks of the timer, in milliseconds to the nearest one. Half rounds
 * up. The host snaps its periods to the 100 ms grid that the phases need. A tick count just under
 * a whole millisecond, truncated, would put the period a millisecond short. The report would
 * then drift through every phase. A period under 1 ms is 0, which keeps the reports off, as the
 * truncation did: rounded, one from half a millisecond up would become 1 ms, a report on every
 * tick. The remainder is compared against what is left of the millisecond, rather than doubled,
 * so nothing overflows near 2^32. ticks_per_ms is never 0, because the binding passes
 * timer_from_us(1000). */
static inline uint32_t ace2k_report_rest_ms(uint32_t rest_ticks, uint32_t ticks_per_ms)
{
    if (rest_ticks < ticks_per_ms) {
        return 0U;
    }
    uint32_t rem = rest_ticks % ticks_per_ms;
    return (rest_ticks / ticks_per_ms) + ((rem >= ticks_per_ms - rem) ? 1U : 0U);
}

/* Sets the period rest_ms, capped at ACE2K_REPORT_REST_MAX_MS. The first report comes one period
 * plus phase_ms after the start of the 100 ms grid cell that now_ms lies in. A period of at least
 * a grid cell keeps that in the future. A shorter one may land it in the past; the first due call
 * then advances it whole periods ahead. The anchor wraps with the 32-bit millisecond clock, whose
 * range is not a multiple of 100. So a query from before the 49-day wrap and one from after it sit
 * on grids 4 ms apart, until the next query. The caller masks interrupts. */
static inline void ace2k_report_set(struct ace2k_report *r, uint32_t rest_ms, uint32_t now_ms,
                                    uint32_t phase_ms)
{
    if (rest_ms > ACE2K_REPORT_REST_MAX_MS) {
        rest_ms = ACE2K_REPORT_REST_MAX_MS;
    }
    r->rest_ms = rest_ms;
    r->next_ms = (now_ms - (now_ms % ACE2K_REPORT_GRID_MS)) + rest_ms + phase_ms;
}

/* True once per period, at the deadline. The next deadline is the first grid point after now,
 * whole periods ahead, so a stall skips the periods it swallowed. False while the reports are
 * off. */
static inline bool ace2k_report_due(struct ace2k_report *r, uint32_t now_ms)
{
    if (r->rest_ms == 0 || !ace2k_time_after(now_ms, r->next_ms)) {
        return false;
    }
    do {
        r->next_ms += r->rest_ms;
    } while (ace2k_time_after(now_ms, r->next_ms));
    return true;
}

/* One try of a due report, from the task. The flag is cleared BEFORE the send, and set again
 * when the send refused because the frame did not fit. So the report is held for the next tick.
 * The order matters. The tick may raise the flag during the send, and a clear after the send would
 * wipe that deadline. Clearing first yields an extra report at worst, never a missing one. Returns
 * what send returned. The caller checks the flag before calling. */
static inline bool ace2k_report_try(volatile bool *due, bool (*send)(void))
{
    *due = false;
    if (send()) {
        return true;
    }
    *due = true;
    return false;
}

#endif
