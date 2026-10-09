/* What: the period of a binding's periodic report and its next deadline — one scheduler for
 * the report bindings (sensors at 10 Hz; airflow, heat, env, lane, rfid, mains, feed and dryer
 * at 1 Hz).
 * How: the query handler calls ace2k_report_set() with the period the host asked for (0 stops
 * the reports) and the module's phase; the tick calls ace2k_report_due() and wakes its task on
 * true.  A due report moves the deadline whole periods ahead until it is in the future: after
 * a stall longer than one period the missed reports are skipped, never replayed one per tick,
 * and the deadlines keep their grid — the same rule the board's ADC scan applies to its passes.
 * The transmit rule: Klipper drops a frame that does not fit its 96-byte transmit buffer, on
 * the frame's real encoded length; the bindings try each report and event instead (tx.h) and
 * hold one that did not fit — a report by its due flag, an event in its ring — for the next
 * tick, the due report before the events; ace2k_report_try() is the one try of a due report,
 * with the flag's order.  The phases — sensors 0, airflow 10, env 30, feed-forward 40, lane 50,
 * rfid 60, mains 70, feed 80, dryer 90 ms, on periods that are multiples of 100 ms — keep the
 * periodic reports on distinct ticks; nothing else is claimed.
 * The deadlines are anchored to the absolute 100 ms grid (ace2k_report_set), not to the tick
 * the query arrived on, so two queries from different ticks of the same 100 ms still land on
 * their phases; and a deadline reseeded from a late tick would put two phased reports back on
 * the same tick for ever, hence the whole-period rule.
 * The tick reads the pair in the timer interrupt and the handler writes it from a task, so the
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

/* Period rest_ms, the first report one period and phase_ms after the start of the 100 ms grid
 * cell now_ms lies in — a period of at least a grid cell keeps that in the future (a shorter
 * one may land it in the past: the first due call then advances it whole periods ahead).  The
 * anchor wraps with the 32-bit millisecond clock, whose range is not a multiple of 100: a query
 * from before the 49-day wrap and one from after it sit on grids 4 ms apart until the next
 * query.  Interrupts masked by the caller. */
static inline void ace2k_report_set(struct ace2k_report *r, uint32_t rest_ms, uint32_t now_ms,
                                    uint32_t phase_ms)
{
    r->rest_ms = rest_ms;
    r->next_ms = (now_ms - (now_ms % ACE2K_REPORT_GRID_MS)) + rest_ms + phase_ms;
}

/* True once per period, at the deadline; the next deadline is the first grid point after now
 * (whole periods ahead, so a stall skips the periods it swallowed).  False while the reports
 * are off. */
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

/* One try of a due report from the task: the flag cleared BEFORE the send and set again when
 * the send refused (its frame did not fit), so the report is held for the next tick.  The order
 * matters: the tick may raise the flag during the send, and a clear after the send would wipe
 * that deadline; clearing first yields an extra report at worst, never a missing one.  Returns
 * what send returned.  The caller checks the flag before calling. */
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
