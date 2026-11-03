// ace2k board FG counters. TIM4 channels 1–4, on PB6–PB9, run in input capture on the falling
// edge (initial value; either edge counts). Each capture raises an interrupt that only clears its
// flag and increments a counter. The capture register is never read. The inputs are pulled up
// before the timer runs, because a floating input floods the interrupt. Pins: docs/hardware.md
// "Counters". Register names: GD32F30x User Manual, timer chapter (STM32F1-compatible layout).
#include "ace2k_board/fg.h"
#include "ace2k_board/tim.h" // ACE2K_TIM_ARR_FREE_RUNNING
#include "ace2k_board/armcm_irq.h"
#include "ace2k_board/irq_prio.h"
#include "ace2k_board/pins.h"
#include "ace2k/core/util.h"  // ACE2K_LANE_COUNT
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/internal.h"   // TIM4, enable_pclock, gpio_peripheral

#define ACE2K_FG_CC_TI (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0)
#define ACE2K_FG_CC_FALLING                                                                        \
    (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC3E |               \
     TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P)
#define ACE2K_FG_CC_FLAGS (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF)
#define ACE2K_FG_CC_IRQS  (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE)

static const uint8_t fg_pins[ACE2K_LANE_COUNT] = {
    ACE2K_PIN_FG1,
    ACE2K_PIN_FG2,
    ACE2K_PIN_FG3,
    ACE2K_PIN_FG4,
};
static volatile uint32_t ace2k_fg_count[ACE2K_LANE_COUNT];

// Klipper's generated vector table names the handler, so it is not static. Interrupt
// context: it clears the capture flags it saw, and counts. The status bits are write-0-to-clear.
void ace2k_fg_irq(void)
{
    uint32_t sr = TIM4->SR;
    TIM4->SR = ~(sr & ACE2K_FG_CC_FLAGS);
    if (sr & TIM_SR_CC1IF) {
        ace2k_fg_count[0]++;
    }
    if (sr & TIM_SR_CC2IF) {
        ace2k_fg_count[1]++;
    }
    if (sr & TIM_SR_CC3IF) {
        ace2k_fg_count[2]++;
    }
    if (sr & TIM_SR_CC4IF) {
        ace2k_fg_count[3]++;
    }
}

void ace2k_fg_init(void)
{
    // The four inputs are pulled up first, and the timer is enabled last.
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        gpio_peripheral(fg_pins[i], GPIO_INPUT, 1);
    }
    enable_pclock((uint32_t)TIM4);
    TIM4->CR1 = 0;
    TIM4->PSC = 0;
    TIM4->ARR = ACE2K_TIM_ARR_FREE_RUNNING;
    TIM4->CCMR1 = ACE2K_FG_CC_TI;
    TIM4->CCMR2 = ACE2K_FG_CC_TI;     /* the same bit positions: CC3S / CC4S */
    TIM4->CCER = ACE2K_FG_CC_FALLING; /* initial value; re-tune on the bench */
    TIM4->SR = 0;
    TIM4->DIER = ACE2K_FG_CC_IRQS;
    TIM4->CR1 = TIM_CR1_CEN;
    // Klipper's IRQ declaration encodes the number with signed literals inside the macro.
    // NOLINTNEXTLINE(bugprone-signed-bitwise)
    ACE2K_ARMCM_ENABLE_IRQ(ace2k_fg_irq, TIM4_IRQn, ACE2K_IRQ_PRIO_FG);
}

uint32_t ace2k_fg_read(uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? ace2k_fg_count[lane] : 0;
}
