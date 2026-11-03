// Binding of the query of the periodic reports: one prologue for every report binding
// (report_cmds.h).
#include "core/report_cmds.h"
#include "ace2k_board/tick.h" // ace2k_tick_now_ms
#include "board/irq.h"        // irq_save, irq_restore
#include "board/misc.h"       // timer_from_us

void ace2k_report_query(struct ace2k_report *r, volatile bool *due, uint32_t rest_ticks,
                        uint32_t phase_ms)
{
    uint32_t rest_ms = ace2k_report_rest_ms(rest_ticks, timer_from_us(ACE2K_REPORT_US_PER_MS));
    irqstatus_t flag = irq_save();
    ace2k_report_set(r, rest_ms, ace2k_tick_now_ms(), phase_ms);
    *due = false;
    irq_restore(flag);
}
