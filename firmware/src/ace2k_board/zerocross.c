// ace2k board zero-cross: PC0 as a floating input (the signal is driven; measured 2026-09-12,
// docs/hardware.md "Counters"), EXTI line 0 routed to port C, rising edge, an interrupt that
// clears the pending flag and hands the edge to the filter (ace2k_mains_edge_filter): every edge
// closer than ACE2K_ZEROCROSS_LOCKOUT_US to the last accepted one is dropped and counted as a
// reject — but the one a flash stall's late edge causes; the first edge after a flash operation
// whose end found this line's pending bit set is that late edge, accepted as a resync edge and
// handed to the hook as one; an accepted edge goes to the hook, if any, and into the filter's
// span.  This is the one place that reads the flash operations' record: the edges' consumers
// learn of a stall only through the resync edge.  Klipper's F1 start-up (stm32/stm32f1.c,
// armcm_main) enables the AFIO clock for its own remap; the EXTICR write here relies on that.
// Register names: GD32F30x User Manual, EXTI and AFIO chapters (STM32F1-compatible layout).
#include "ace2k_board/zerocross.h"
#include "ace2k/heat/mains.h" // ace2k_mains_edge_filter
#include "ace2k_board/armcm_irq.h"
#include "ace2k_board/flash.h" // ace2k_flash_last_stall
#include "ace2k_board/pins.h"
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h"        // irq_save, irq_restore
#include "board/internal.h"   // AFIO, EXTI, gpio_peripheral
#include "board/misc.h"       // timer_read_time, timer_from_us
#include "command.h"          // DECL_CONSTANT_STR

// The host reserves every RESERVE_PINS_* constant, so a [gcode_button] cannot claim the
// zero-cross input (pins.h names it; docs/hardware.md "Counters").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_mains", "PC0");

// Below the host link's interrupt (priority 0, ace2k_board/serial.c), above Klipper's timer
// dispatch (2): the handler reads the clock, filters, counts and calls the hook — the heater's
// full-cycle decision and a gate pulse at most — at most 120 times a second, plus the rejects.
#define ZEROCROSS_IRQ_PRIORITY 1

static volatile uint32_t ace2k_zerocross_edges;
static volatile uint32_t ace2k_zerocross_rejected;
static struct ace2k_mains_edge_filter ace2k_zerocross_filter; // the interrupt's own, after init
static ace2k_zerocross_hook_fn volatile ace2k_zerocross_hook;

// Klipper's generated vector table names the handler, so it is not static.  Interrupt context:
// reads the clock first (the edge's time), clears the pending bit (write 1 to clear), filters.
void ace2k_zerocross_irq(void)
{
    uint32_t now = timer_read_time();
    EXTI->PR = EXTI_PR_PR0;
    struct ace2k_mains_stall stall;
    ace2k_flash_last_stall(&stall);
    enum ace2k_mains_edge verdict = ace2k_mains_edge_filter(&ace2k_zerocross_filter, now, &stall);
    if (verdict == ACE2K_MAINS_EDGE_REJECT) {
        ace2k_zerocross_rejected++;
        return;
    }
    if (verdict == ACE2K_MAINS_EDGE_REJECT_UNCOUNTED) {
        return;
    }
    bool resync = verdict == ACE2K_MAINS_EDGE_RESYNC;
    ace2k_zerocross_edges++;
    ace2k_zerocross_hook_fn hook = ace2k_zerocross_hook;
    if (hook != NULL) {
        hook(now, resync);
    }
}

void ace2k_zerocross_init(void)
{
    // before the interrupt is enabled: from then on the filter is the interrupt's alone
    struct ace2k_mains_stall stall;
    ace2k_flash_last_stall(&stall);
    ace2k_mains_edge_filter_init(&ace2k_zerocross_filter, timer_from_us(ACE2K_ZEROCROSS_LOCKOUT_US),
                                 stall.count);
    gpio_peripheral(ACE2K_PIN_ZEROCROSS, GPIO_INPUT, 0);
    AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~AFIO_EXTICR1_EXTI0) | AFIO_EXTICR1_EXTI0_PC;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->PR = EXTI_PR_PR0;
    EXTI->IMR |= EXTI_IMR_MR0;
    ACE2K_ARMCM_ENABLE_IRQ(ace2k_zerocross_irq, EXTI0_IRQn, ZEROCROSS_IRQ_PRIORITY);
}

uint32_t ace2k_zerocross_count(void)
{
    return ace2k_zerocross_edges;
}

uint32_t ace2k_zerocross_rejects(void)
{
    return ace2k_zerocross_rejected;
}

void ace2k_zerocross_span(struct ace2k_mains_span *out)
{
    // words the interrupt writes: copied, and the bridge handed over, in one step
    irqstatus_t flag = irq_save();
    ace2k_mains_edge_filter_take(&ace2k_zerocross_filter, out);
    irq_restore(flag);
}

void ace2k_zerocross_set_hook(ace2k_zerocross_hook_fn fn)
{
    // one aligned pointer store; bracketed anyway so the interrupt never runs between the
    // caller's own setup and the hook becoming visible
    irqstatus_t flag = irq_save();
    ace2k_zerocross_hook = fn;
    irq_restore(flag);
}
