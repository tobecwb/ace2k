// ace2k board ADC scan: one scheduler timer walks the channels of adc_map.h, one conversion
// per call.  Klipper's stm32/adc.c runs one conversion at a time on the peripheral:
// gpio_adc_sample() starts it and answers with a 20 µs wait, answers the same wait while a
// conversion of any channel is started and unread (the peripheral is taken), and returns 0
// once this channel's result is ready; gpio_adc_read() takes the result and frees it.  A
// conversion is 54 ADC clock cycles (adc.c's 41.5-cycle sample time + 12.5) ≈ 14 µs at the
// port's 3.75 MHz ADC clock (APB2 30 MHz / 8, src/stm32/gd32f30x.c), and the peripheral
// stays taken until the poll that reads it, ≈ 20 µs.
//
// A pass starts every ACE2K_ADC_SCAN_PERIOD_MS, and its channels are spaced
// ACE2K_ADC_SCAN_SLOT_US apart across that period instead of running back-to-back, so that
// Klipper's own ADC users never wait more than one conversion of ours.  Measured on the unit
// 2026-09-17: with the pass run back-to-back (the conversions 5 µs apart, every 10 ms) and the
// MCU's own temperature sensor (channel 16) among its channels, a [temperature_sensor] of type
// temperature_mcu — Klipper's analog_in on that same channel — stalled the host link.  Klipper's
// analog_in (adccmds.c) re-polls a taken ADC with waketime += 20 µs, so it falls behind real
// time while the peripheral is held, and the timer dispatcher (generic/armcm_timer.c,
// timer_dispatch_many) catches an overdue timer up by running it again and again with
// interrupts masked — longer than the 40 µs a byte the one-byte UART4 receiver allows at
// 250 kbaud: bytes of the host's frames were lost and its long frames (the configuration
// burst) never arrived intact.  The scan no longer samples channel 16 (adc_map.h): the MCU's
// temperature is Klipper's temperature_mcu's to read, and the only contact left between the
// two users of the ADC is the ≈ 25 µs a conversion holds the peripheral.
//
// A completed pass is published with the tick clock's time.  Publishing and reading both
// happen in timer context, so the copy needs no lock.  Pins: docs/hardware.md "Analogue inputs".
#include "ace2k_board/adc_scan.h"
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "ace2k/core/util.h" // ace2k_time_since
#include "board/gpio.h"      // gpio_adc_setup, gpio_adc_sample, gpio_adc_read, _cancel_sample
#include "board/misc.h"      // timer_read_time, timer_from_us, timer_is_before
#include "command.h"         // DECL_CONSTANT_STR
#include "sched.h"           // sched_add_timer, DECL_INIT

// Klipper's adc.c pseudo-pin for the internal reference, ADC_VREFINT (channel 17), added by the
// patch.  adc.c keeps the define private; the value is its enumeration's.
#define ACE2K_ADC_PIN_VREFINT 0xFDU

// The host reserves pin names as strings, the pseudo-pin included: a host analog_in on
// ADC_VREFINT would share the channel this scan reads.
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_adc",
                  "PA5,PA4,PC5,PC4,PC3,PC2,PA2,PA3,PB0,PB1,PC1,ADC_VREFINT");

static const uint8_t ace2k_adc_scan_pins[ACE2K_ADC_COUNT] = {
    [ACE2K_ADC_NTC_LEFT] = ACE2K_PIN_NTC_LEFT, [ACE2K_ADC_NTC_RIGHT] = ACE2K_PIN_NTC_RIGHT,
    [ACE2K_ADC_INSERT1] = ACE2K_PIN_INSERT1,   [ACE2K_ADC_INSERT2] = ACE2K_PIN_INSERT2,
    [ACE2K_ADC_INSERT3] = ACE2K_PIN_INSERT3,   [ACE2K_ADC_INSERT4] = ACE2K_PIN_INSERT4,
    [ACE2K_ADC_EMPTY1] = ACE2K_PIN_EMPTY1,     [ACE2K_ADC_EMPTY2] = ACE2K_PIN_EMPTY2,
    [ACE2K_ADC_EMPTY3] = ACE2K_PIN_EMPTY3,     [ACE2K_ADC_EMPTY4] = ACE2K_PIN_EMPTY4,
    [ACE2K_ADC_AUX] = ACE2K_PIN_AUX,           [ACE2K_ADC_VREFINT] = ACE2K_ADC_PIN_VREFINT,
};

static struct gpio_adc ace2k_adc_scan_chan[ACE2K_ADC_COUNT];
static uint16_t ace2k_adc_scan_pass[ACE2K_ADC_COUNT];
static uint16_t ace2k_adc_scan_latest_raw[ACE2K_ADC_COUNT];
static uint32_t ace2k_adc_scan_latest_ms;
static bool ace2k_adc_scan_have;
static uint8_t ace2k_adc_scan_idx;
static uint32_t ace2k_adc_scan_pass_start; // when channel 0 of the current pass starts
static struct timer ace2k_adc_scan_timer;

// The next pass starts on the period grid; after a stall (a flash write) that left the grid
// behind, whole periods ahead until the start is in the future, so the channels keep their
// slots.  Channel 0 first.
static void schedule_next_pass(struct timer *t)
{
    uint32_t now = timer_read_time();
    ace2k_adc_scan_idx = 0;
    do {
        ace2k_adc_scan_pass_start += timer_from_us(ACE2K_ADC_SCAN_PERIOD_MS * 1000U);
    } while (timer_is_before(ace2k_adc_scan_pass_start, now));
    t->waketime = ace2k_adc_scan_pass_start;
}

static uint_fast8_t scan_event(struct timer *t)
{
    uint8_t idx = ace2k_adc_scan_idx;
    uint32_t wait = gpio_adc_sample(ace2k_adc_scan_chan[idx]);
    if (wait) {
        // our conversion running, or another user's: poll again from now; nothing accumulates
        t->waketime = timer_read_time() + wait;
        return SF_RESCHEDULE;
    }
    ace2k_adc_scan_pass[idx] = gpio_adc_read(ace2k_adc_scan_chan[idx]);
    idx++;
    if (idx < ACE2K_ADC_COUNT) {
        // channel idx starts idx slots into the pass
        uint32_t due =
            ace2k_adc_scan_pass_start + (uint32_t)idx * timer_from_us(ACE2K_ADC_SCAN_SLOT_US);
        if (!timer_is_before(due, timer_read_time())) {
            ace2k_adc_scan_idx = idx;
            t->waketime = due;
            return SF_RESCHEDULE;
        }
        // Its slot is already past (a stall — a flash write — held the timer): the rest of
        // the pass is abandoned, not run back-to-back to catch up, which keeps the invariant
        // that no two of our conversions run back-to-back.  The pass is not published; the
        // previous one stays and its age grows.
        schedule_next_pass(t);
        return SF_RESCHEDULE;
    }
    for (uint8_t i = 0; i < ACE2K_ADC_COUNT; i++) {
        ace2k_adc_scan_latest_raw[i] = ace2k_adc_scan_pass[i];
    }
    // stamped at completion: with the channels spread across the period, channel 0 of this
    // pass is ACE2K_ADC_COUNT − 1 slots (≈ 9 ms) older than the stamp
    ace2k_adc_scan_latest_ms = ace2k_tick_now_ms();
    ace2k_adc_scan_have = true;
    schedule_next_pass(t);
    return SF_RESCHEDULE;
}

bool ace2k_adc_scan_latest(uint16_t raw[ACE2K_ADC_COUNT], uint32_t *age_ms)
{
    if (!ace2k_adc_scan_have) {
        return false;
    }
    for (uint8_t i = 0; i < ACE2K_ADC_COUNT; i++) {
        raw[i] = ace2k_adc_scan_latest_raw[i];
    }
    *age_ms = ace2k_time_since(ace2k_tick_now_ms(), ace2k_adc_scan_latest_ms);
    return true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_adc_scan_init(void)
{
    for (uint8_t i = 0; i < ACE2K_ADC_COUNT; i++) {
        ace2k_adc_scan_chan[i] = gpio_adc_setup(ace2k_adc_scan_pins[i]);
    }
    ace2k_adc_scan_pass_start = timer_read_time() + timer_from_us(ACE2K_ADC_SCAN_PERIOD_MS * 1000U);
    ace2k_adc_scan_timer.func = scan_event;
    ace2k_adc_scan_timer.waketime = ace2k_adc_scan_pass_start;
    sched_add_timer(&ace2k_adc_scan_timer);
}
DECL_INIT(ace2k_adc_scan_init);

// Klipper drops every user timer on a shutdown; the readings must go on (the health mask and,
// later, the dryer read them).  Runs with interrupts disabled, as tick.c's does.  The
// conversion in flight is cancelled first, as Klipper's own analog_in shutdown does: adc.c
// answers "busy" for every other channel while a started conversion stays unread, and the
// restarted pass begins on channel 0, at its pass start, as the timer's own publish path does.
void ace2k_adc_scan_shutdown(void)
{
    gpio_adc_cancel_sample(ace2k_adc_scan_chan[ace2k_adc_scan_idx]);
    ace2k_adc_scan_idx = 0;
    ace2k_adc_scan_pass_start = timer_read_time() + timer_from_us(ACE2K_ADC_SCAN_PERIOD_MS * 1000U);
    ace2k_adc_scan_timer.waketime = ace2k_adc_scan_pass_start;
    sched_add_timer(&ace2k_adc_scan_timer);
}
DECL_SHUTDOWN(ace2k_adc_scan_shutdown);
