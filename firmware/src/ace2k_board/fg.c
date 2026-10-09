// ace2k board FG counters: TIM4 channels 1–4 on PB6–PB9 in input capture on the falling edge
// (initial value; either edge counts), an interrupt per capture that only clears its flag
// and increments a counter — the capture register is never read.  Pulled up before the timer
// runs (a floating input floods the interrupt).  Pins: docs/hardware.md "Counters"; register
// names: GD32F30x User Manual, timer chapter (STM32F1-compatible layout).
#include "ace2k_board/fg.h"
#include "ace2k_board/armcm_irq.h"
#include "ace2k_board/pins.h"
#include "ace2k/core/util.h"  // ACE2K_LANE_COUNT
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/internal.h"   // TIM4, enable_pclock, gpio_peripheral

// Below the host link's interrupt (priority 0, ace2k_board/serial.c), above Klipper's timer
// dispatch (2): the handler is a few loads and stores per pulse.
#define FG_IRQ_PRIORITY 1
#define FG_CC_TI        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0)
#define FG_CC_FALLING                                                                              \
    (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC3E |               \
     TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P)
#define FG_CC_FLAGS (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF)
#define FG_CC_IRQS  (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE)

static const uint8_t ace2k_fg_pins[ACE2K_LANE_COUNT] = { ACE2K_PIN_FG1, ACE2K_PIN_FG2,
                                                         ACE2K_PIN_FG3, ACE2K_PIN_FG4 };
static volatile uint32_t ace2k_fg_count[ACE2K_LANE_COUNT];

// Klipper's generated vector table names the handler, so it is not static.  Interrupt context:
// clears the capture flags it saw (the status bits are write-0-to-clear) and counts.
void ace2k_fg_irq(void)
{
    uint32_t sr = TIM4->SR;
    TIM4->SR = ~(sr & FG_CC_FLAGS);
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
    // the four inputs pulled up first, the timer enabled last
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        gpio_peripheral(ace2k_fg_pins[i], GPIO_INPUT, 1);
    }
    enable_pclock((uint32_t)TIM4);
    TIM4->CR1 = 0;
    TIM4->PSC = 0;
    TIM4->ARR = 0xFFFFU;
    TIM4->CCMR1 = FG_CC_TI;
    TIM4->CCMR2 = FG_CC_TI;     /* the same bit positions: CC3S / CC4S */
    TIM4->CCER = FG_CC_FALLING; /* initial value; adjust on the bench */
    TIM4->SR = 0;
    TIM4->DIER = FG_CC_IRQS;
    TIM4->CR1 = TIM_CR1_CEN;
    ACE2K_ARMCM_ENABLE_IRQ(ace2k_fg_irq, TIM4_IRQn, FG_IRQ_PRIORITY);
}

uint32_t ace2k_fg_read(uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? ace2k_fg_count[lane] : 0;
}
