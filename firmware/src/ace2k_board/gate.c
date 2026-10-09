// ace2k board gate: PC8, high = the triac fires (docs/hardware.md "Heater", measured on the
// unit 2026-09-13), with an external pull-down; TIM7 in one-pulse mode ends each pulse.  Register
// names: GD32F30x User Manual, GPIO and basic-timer chapters (STM32F1-compatible layout).  The
// pin's reservation for the host is heat_cmds.c's (RESERVE_PINS_ace2k_heat).
#include "ace2k_board/gate.h"
#include "ace2k/heat/heat.h" // ACE2K_HEAT_GATE_PULSE_US
#include "ace2k_board/armcm_irq.h"
#include "ace2k_board/pins.h"
#include "autoconf.h"         // CONFIG_CLOCK_FREQ
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/gpio.h"       // gpio_out_setup
#include "board/internal.h"   // TIM7, GPIOC, enable_pclock, get_pclock_frequency, gpio_peripheral
#include "board/irq.h"        // irq_save, irq_restore
#include "board/misc.h"       // timer_read_time

// Same level as the zero-cross interrupt, which fires the gate: neither preempts the other.
#define GATE_TIMER_IRQ_PRIORITY 1
#define US_PER_S                1000000U
#define GATE_BIT                (1U << 8U) // PC8 in GPIOC's BSRR / BRR / IDR

static uint32_t ace2k_gate_cycles_per_us;
static uint32_t ace2k_gate_cycles_last;
static uint32_t ace2k_gate_us;

// Klipper's generated vector table names the handler, so it is not static.  The pulse ends here.
void ace2k_gate_timer_irq(void)
{
    TIM7->SR = 0; // the flag first: cleared well before the handler returns
    GPIOC->BRR = GATE_BIT;
}

void ace2k_gate_init(void)
{
    (void)gpio_out_setup(ACE2K_PIN_TRIAC_GATE, 0); // BSRR reset first, then the mode
    enable_pclock((uint32_t)TIM7);
    // an F1 timer on a prescaled bus runs at twice the bus clock (60 MHz at 120 MHz, 72 at 72)
    uint32_t pclk = get_pclock_frequency((uint32_t)TIM7);
    uint32_t tim_hz = pclk < CONFIG_CLOCK_FREQ ? pclk * 2U : pclk;
    TIM7->CR1 = 0;
    TIM7->PSC = (tim_hz / US_PER_S) - 1U;
    TIM7->ARR = ACE2K_HEAT_GATE_PULSE_US;
    TIM7->EGR = TIM_EGR_UG; // load PSC and ARR — which raises UIF ...
    TIM7->SR = 0;           // ... cleared before the interrupt is enabled
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->CR1 = TIM_CR1_OPM; // one-pulse mode, not running
    ace2k_gate_cycles_per_us = CONFIG_CLOCK_FREQ / US_PER_S;
    ace2k_gate_cycles_last = timer_read_time();
    ACE2K_ARMCM_ENABLE_IRQ(ace2k_gate_timer_irq, TIM7_IRQn, GATE_TIMER_IRQ_PRIORITY);
}

// Interrupt context (the zero-cross interrupt, through heat).
void ace2k_gate_fire(void)
{
    GPIOC->BSRR = GATE_BIT;
    TIM7->CNT = 0;
    TIM7->CR1 = TIM_CR1_OPM | TIM_CR1_CEN; // UIF after the pulse; CEN clears itself
}

// The timer stops first: a fire that slips in after this line restarts it, and its pulse still
// ends on time.
void ace2k_gate_off(void)
{
    TIM7->CR1 = TIM_CR1_OPM;
    GPIOC->BRR = GATE_BIT;
    TIM7->CNT = 0;
}

bool ace2k_gate_read(void)
{
    return (GPIOC->IDR & GATE_BIT) != 0U;
}

// Shutdown only (Klipper's run_shutdown: interrupts disabled).
void ace2k_gate_float(void)
{
    ace2k_gate_off();
    gpio_peripheral(ACE2K_PIN_TRIAC_GATE, GPIO_INPUT, 0);
}

// The clock brought up to the cycle counter, in whole microseconds; the caller masks interrupts.
static void gate_clock_advance(void)
{
    uint32_t now = timer_read_time();
    uint32_t whole = (now - ace2k_gate_cycles_last) / ace2k_gate_cycles_per_us;
    ace2k_gate_us += whole;
    ace2k_gate_cycles_last += whole * ace2k_gate_cycles_per_us;
}

// Both interrupt contexts and the tick call it; the update is one step against them.
uint32_t ace2k_gate_now_us(void)
{
    irqstatus_t flag = irq_save();
    gate_clock_advance();
    uint32_t us = ace2k_gate_us;
    irq_restore(flag);
    return us;
}

// The zero-cross interrupt: the clock at a cycle count read a moment ago — brought up to now,
// then stepped back by the cycles since that read, rounded up (none when the read falls inside
// the last, partial microsecond).  What this removes is the handler's own run before heat is
// called; the read itself happens at the handler's entry, so the delay from the edge to that
// entry (another interrupt of the same priority, an irq_save section) is still in the time.
uint32_t ace2k_gate_us_at(uint32_t cycles)
{
    irqstatus_t flag = irq_save();
    gate_clock_advance();
    uint32_t us = ace2k_gate_us;
    int32_t behind = (int32_t)(ace2k_gate_cycles_last - cycles);
    if (behind > 0) {
        us -= ((uint32_t)behind + ace2k_gate_cycles_per_us - 1U) / ace2k_gate_cycles_per_us;
    }
    irq_restore(flag);
    return us;
}
