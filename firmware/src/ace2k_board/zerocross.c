// ace2k board zero-cross. PC0 is a floating input, because the signal is driven (measured on the
// unit, docs/hardware.md "Counters"). EXTI line 0 is routed to port C, rising edge. The interrupt
// clears the pending flag and hands the edge to the filter (ace2k_mains_edge_filter):
// - Every edge closer than ACE2K_ZEROCROSS_LOCKOUT_US to the last accepted one is dropped, and
//   counted as a reject. The one that the late edge of a flash stall causes is not counted.
// - The first edge after a flash operation whose end found the pending bit of this line set is
//   that late edge. It is accepted as a resync edge, and handed to the hook as one.
// - An accepted edge goes to the hook, if any, and into the filter's span.
// This is the one place that reads the record of the flash operations. The consumers of the edges
// learn of a stall only through the resync edge. Klipper's F1 start-up (armcm_main in
// stm32/stm32f1.c) enables the AFIO clock for its own remap, and the EXTICR write here relies on
// that. Register names: GD32F30x User Manual, EXTI and AFIO chapters (STM32F1-compatible layout).
#include "ace2k_board/zerocross.h"
#include "ace2k/heat/mains.h" // ace2k_mains_edge_filter
#include "ace2k_board/armcm_irq.h"
#include "ace2k_board/flash.h" // ace2k_flash_last_stall
#include "ace2k_board/irq_prio.h"
#include "ace2k_board/pins.h"
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h"        // irq_save, irq_restore
#include "board/internal.h"   // AFIO, EXTI, gpio_peripheral
#include "board/misc.h"       // timer_read_time, timer_from_us
#include "command.h"          // DECL_CONSTANT_STR

// The host reserves every RESERVE_PINS_* constant, so a [gcode_button] cannot claim the
// zero-cross input. pins.h names it (docs/hardware.md "Counters").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_mains", "PC0");

static volatile uint32_t ace2k_zerocross_edges;
static volatile uint32_t ace2k_zerocross_rejected;
static struct ace2k_mains_edge_filter ace2k_zerocross_filter; // owned by the interrupt after init
static ace2k_zerocross_hook_fn volatile ace2k_zerocross_hook;

// Klipper's generated vector table names the handler, so it is not static. Interrupt
// context: it reads the clock first (the edge's time), clears the pending bit (write 1 to
// clear), and filters.
void ace2k_zerocross_irq(void)
{
    uint32_t now_ticks = timer_read_time();
    EXTI->PR = EXTI_PR_PR0;
    struct ace2k_mains_stall stall;
    ace2k_flash_last_stall(&stall);
    enum ace2k_mains_edge verdict =
        ace2k_mains_edge_filter(&ace2k_zerocross_filter, now_ticks, &stall);
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
        hook(now_ticks, resync);
    }
}

void ace2k_zerocross_init(void)
{
    // Before the interrupt is enabled. From then on, the filter belongs to the interrupt alone.
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
    // Klipper's IRQ declaration encodes the number with signed literals inside the macro.
    // NOLINTNEXTLINE(bugprone-signed-bitwise)
    ACE2K_ARMCM_ENABLE_IRQ(ace2k_zerocross_irq, EXTI0_IRQn, ACE2K_IRQ_PRIO_ZEROCROSS);
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
    // Words that the interrupt writes: copied, and the bridge handed over, in one step.
    irqstatus_t flag = irq_save();
    ace2k_mains_edge_filter_take(&ace2k_zerocross_filter, out);
    irq_restore(flag);
}

void ace2k_zerocross_set_hook(ace2k_zerocross_hook_fn fn)
{
    // One aligned pointer store. It is bracketed anyway, so the interrupt never runs between the
    // setup of the caller and the moment the hook becomes visible.
    irqstatus_t flag = irq_save();
    ace2k_zerocross_hook = fn;
    irq_restore(flag);
}
