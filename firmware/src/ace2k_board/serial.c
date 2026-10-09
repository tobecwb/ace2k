// ace2k serial: Klipper's console on UART4 (PC10 TX, PC11 RX) behind an RS-485 transceiver
// whose driver-enable is PA11 — high before the first bit, low once the last stop bit is out
// (docs/hardware.md "Host link").  Same shape as Klipper's stm32/serial.c plus the
// transmit-complete interrupt that releases the line.  The turnaround (`link_turnaround.h`):
// PA11 rises only once the line has been quiet for `ACE2K_LINK_IDLE_US` (serial.h) since the
// last received byte, so the unit never answers over the tail of a host burst.
// The receiver-enable shares PA11, so nothing the unit sends
// comes back on RX.
#include "ace2k_board/serial.h"    // ACE2K_LINK_IDLE_US, ace2k_serial_link_counters
#include "autoconf.h"              // CONFIG_SERIAL_BAUD
#include "ace2k_board/armcm_irq.h" // ACE2K_ARMCM_ENABLE_IRQ
#include "ace2k_board/pins.h"      // ACE2K_PIN_LINK_*
#include "board/armcm_boot.h"      // armcm_enable_irq
#include "board/gpio.h"            // gpio_out_setup
#include "board/internal.h"        // enable_pclock, gpio_peripheral
#include "board/irq.h"             // irq_save, irq_restore
#include "board/misc.h"            // timer_read_time, timer_from_us, timer_is_before
#include "board/serial_irq.h"      // serial_rx_byte
#include "command.h"               // DECL_CONSTANT_STR
#include "link/link_turnaround.h"  // ace2k_link_turnaround_*
#include "sched.h"                 // DECL_INIT, DECL_SHUTDOWN, struct timer, sched_add_timer, SF_*

DECL_CONSTANT_STR("RESERVE_PINS_serial", "PC11,PC10,PA11");

#define USART_AF_MODE 7
#define CR1_FLAGS     (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE)

static struct gpio_out ace2k_driver_enable;

// A timer must not be added at a wake time already past (sched_add_timer shuts the MCU down
// with "Timer too close"): the least lead a deferred start is scheduled with.
#define ACE2K_LINK_TIMER_LEAD_US 10U

static struct ace2k_link_turnaround ace2k_link;
static struct timer ace2k_link_timer;
static bool ace2k_link_timer_armed;

// PA11 high, then the transmit interrupt: the line is ours from here to the TC interrupt.
static void start_tx(void)
{
    gpio_out_write(ace2k_driver_enable, 1);
    UART4->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
}

// Timer interrupt context.
static uint_fast8_t link_timer_event(struct timer *t)
{
    uint32_t at = 0;
    enum ace2k_link_verdict v = ace2k_link_turnaround_timer(&ace2k_link, timer_read_time(), &at);
    if (v == ACE2K_LINK_WAIT) {
        t->waketime = at;
        return SF_RESCHEDULE;
    }
    ace2k_link_timer_armed = false;
    if (v == ACE2K_LINK_START)
        start_tx();
    return SF_DONE;
}

// TC is never cleared by software: the data write that starts the next frame clears it (the
// USART's read-SR-then-write-DR sequence, which every pass below performs; GD32F30x User Manual,
// USART status register), so a set TC always means "nothing in flight", and a transmit enable
// that finds the queue already drained releases the line in the same pass instead of waiting for
// a completion that never comes.
void UART4_IRQHandler(void)
{
    uint32_t sr = UART4->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // ORE clears on the SR read followed by the DR read
        ace2k_link_turnaround_rx(&ace2k_link, timer_read_time());
        serial_rx_byte(UART4->DR);
    }
    if (sr & USART_SR_TXE && UART4->CR1 & USART_CR1_TXEIE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret) {
            // Queue empty: the last byte is in the shift register; wait for TC
            UART4->CR1 = CR1_FLAGS | USART_CR1_TCIE;
        } else {
            gpio_out_write(ace2k_driver_enable, 1);
            UART4->DR = data;
        }
    }
    if (sr & USART_SR_TC && UART4->CR1 & USART_CR1_TCIE) {
        gpio_out_write(ace2k_driver_enable, 0);
        UART4->CR1 = CR1_FLAGS;
    }
}

// Task context (Klipper queues bytes from tasks only).  A transmission in progress owns the
// line; otherwise the turnaround decides, and a deferred start goes through one timer.
void serial_enable_tx_irq(void)
{
    irqstatus_t flag = irq_save();
    if (UART4->CR1 & (USART_CR1_TXEIE | USART_CR1_TCIE)) {
        UART4->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
    } else {
        uint32_t at = 0;
        enum ace2k_link_verdict v =
            ace2k_link_turnaround_tx_request(&ace2k_link, timer_read_time(), &at);
        if (v == ACE2K_LINK_START) {
            start_tx();
        } else if (!ace2k_link_timer_armed) {
            uint32_t soonest = timer_read_time() + timer_from_us(ACE2K_LINK_TIMER_LEAD_US);
            ace2k_link_timer.waketime = timer_is_before(at, soonest) ? soonest : at;
            ace2k_link_timer_armed = true;
            sched_add_timer(&ace2k_link_timer);
        }
    }
    irq_restore(flag);
}

void serial_init(void)
{
    ace2k_driver_enable = gpio_out_setup(ACE2K_PIN_LINK_DE, 0);

    ace2k_link_turnaround_init(&ace2k_link, timer_from_us(ACE2K_LINK_IDLE_US));
    ace2k_link_timer.func = link_timer_event;

    enable_pclock((uint32_t)UART4);
    // The divisor is exact at both core clocks: UART4 sits on APB1, which get_pclock_frequency()
    // (Klipper's stm32/stm32f1.c as patched) reports as the core clock / 4 above 72 MHz and / 2
    // at it — 30 MHz / 250000 = 120 (mantissa 7, fraction 8) at 120 MHz, 36 MHz / 250000 = 144
    // (mantissa 9, fraction 0) at 72 MHz.
    uint32_t pclk = get_pclock_frequency((uint32_t)UART4);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, CONFIG_SERIAL_BAUD);
    UART4->BRR =
        (((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    UART4->CR1 = CR1_FLAGS;
    ACE2K_ARMCM_ENABLE_IRQ(UART4_IRQHandler, UART4_IRQn, 0);

    gpio_peripheral(ACE2K_PIN_LINK_RX, GPIO_FUNCTION(USART_AF_MODE), 1);
    gpio_peripheral(ACE2K_PIN_LINK_TX, GPIO_FUNCTION(USART_AF_MODE), 0);
}
DECL_INIT(serial_init);

// Any context.  The clock is read with interrupts masked, after the receive interrupt can no
// longer move last_rx past it, so the core's unsigned age is never negative; the core latches a
// loss until the next byte (the age alone would read small again one counter wrap later).
bool ace2k_serial_link_ok(void)
{
    irqstatus_t flag = irq_save();
    bool ok = ace2k_link_turnaround_link_ok(&ace2k_link, timer_read_time(),
                                            timer_from_us(ACE2K_LINK_LOST_MS * 1000U));
    irq_restore(flag);
    return ok;
}

// Klipper drops every user timer on a shutdown (sched_timer_reset, before the shutdown functions
// run), this one included.  A deferred start left pending cannot wait for a later transmit
// enable to re-arm it: Klipper skips that call while its transmit buffer is full, and nothing
// drains the buffer until a start — so the pending start is re-armed here, at the least lead,
// and the callback then starts or waits until the line is quiet as usual.  With nothing pending
// the timer is simply gone.  Runs with interrupts disabled; sched_add_timer is safe there
// (tick.c re-adds its timer the same way).  Not static: Klipper calls it by name from generated
// code.
void ace2k_link_shutdown(void)
{
    if (ace2k_link.tx_pending) {
        ace2k_link_timer.waketime = timer_read_time() + timer_from_us(ACE2K_LINK_TIMER_LEAD_US);
        ace2k_link_timer_armed = true;
        sched_add_timer(&ace2k_link_timer);
    } else {
        ace2k_link_timer_armed = false;
    }
}
DECL_SHUTDOWN(ace2k_link_shutdown);

// Task context (the query's binding).  The receive and timer interrupts write the counters, so
// the pair is copied with interrupts masked.
void ace2k_serial_link_counters(uint32_t *deferred, uint32_t *rearmed)
{
    irqstatus_t flag = irq_save();
    *deferred = ace2k_link.deferred;
    *rearmed = ace2k_link.rearmed;
    irq_restore(flag);
}
