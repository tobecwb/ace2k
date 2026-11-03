/* What: the half of a periodic report that the query handler runs, shared by every report
 * binding.
 * How: ace2k_report_query() takes the period the host sent, in timer ticks. It converts it to
 * milliseconds, to the nearest one (ace2k_report_rest_ms). With the tick masked, it sets the
 * period and the deadline of the report from the tick's clock. Under the same mask it clears
 * the due flag, because a report held from before the query would leave on the terms of the new
 * grid. The tick reads the pair in the timer interrupt. Both stores happen in one step, so the
 * tick never sees the new period against the old deadline. Task context (a command handler).
 * Depends on: report.h, ace2k_board/tick.h (binding: Klipper-side code). */
#ifndef ACE2K_REPORT_CMDS_H
#define ACE2K_REPORT_CMDS_H
#include <stdbool.h>
#include <stdint.h>
#include "core/report.h"

/* Microseconds in a millisecond. timer_from_us() of it, in the binding, is the ticks per ms. */
#define ACE2K_REPORT_US_PER_MS 1000U

/* Sets the period rest_ticks (0 stops the reports) and the module's phase. */
void ace2k_report_query(struct ace2k_report *r, volatile bool *due, uint32_t rest_ticks,
                        uint32_t phase_ms);

#endif
