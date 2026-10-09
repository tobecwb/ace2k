// Binding of the led module to PE13/PE9/PC15/PE5, low = on (docs/hardware.md "Lane LEDs").
//
// The tick callback and set_pattern both touch the instance: set_pattern runs in command
// context (a task) — it writes pattern, epoch_ms and intro_over — while the tick and set_lane
// run in the timer interrupt.  A pattern change between the tick's ace2k_led_mask() and its
// write can produce one stale mask for at most one tick; a set_lane landing inside set_pattern
// can lose its intro_over, which only lets the sweep resume if that lane is OFF again before
// the sweep's 2.3 s end — harmless for an LED; no lock.
#include "lane/led_cmds.h"
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "board/gpio.h" // gpio_out_setup, gpio_out_write
#include "command.h"    // DECL_CONSTANT_STR, shutdown
#include "sched.h"      // sched_shutdown (behind shutdown)

// The host reserves every RESERVE_PINS_* constant, so a [output_pin] cannot claim a lane LED
// (pins.h names them; docs/hardware.md "Lane LEDs").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_led", "PE13,PE9,PC15,PE5");

static struct gpio_out ace2k_led_lane_gpio[ACE2K_LED_COUNT];
static struct ace2k_led ace2k_led_instance;
static bool ace2k_led_ready;

static void set_mask(void *ctx, uint8_t on_mask)
{
    (void)ctx;
    for (int i = 0; i < ACE2K_LED_COUNT; i++) {
        // active low: a lit lane is driven 0
        gpio_out_write(ace2k_led_lane_gpio[i], (on_mask >> i) & 1U ? 0 : 1);
    }
}

static const struct ace2k_led_ops ace2k_led_board_ops = { .set_mask = set_mask };

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    ace2k_led_tick(&ace2k_led_instance, now_ms);
}

static void ensure_ready(void)
{
    static const uint8_t pins[ACE2K_LED_COUNT] = { ACE2K_PIN_LED_LANE1, ACE2K_PIN_LED_LANE2,
                                                   ACE2K_PIN_LED_LANE3, ACE2K_PIN_LED_LANE4 };
    if (ace2k_led_ready) {
        return;
    }
    for (int i = 0; i < ACE2K_LED_COUNT; i++) {
        ace2k_led_lane_gpio[i] = gpio_out_setup(pins[i], 1);
    }
    ace2k_led_init(&ace2k_led_instance, &ace2k_led_board_ops, NULL, ace2k_tick_now_ms());
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_led_ready = true;
}

void ace2k_led_binding_set_pattern(enum ace2k_led_pattern pattern)
{
    ensure_ready();
    ace2k_led_set_pattern(&ace2k_led_instance, pattern, ace2k_tick_now_ms());
}

// From the feed binding's tick (interrupt context): the instance is ready by then — every
// DECL_INIT ran before the first tick — and a call before it is dropped rather than set up
// GPIOs from an interrupt.
void ace2k_led_binding_set_lane(uint8_t lane, enum ace2k_led_lane_pattern pattern)
{
    if (!ace2k_led_ready) {
        return;
    }
    ace2k_led_set_lane(&ace2k_led_instance, lane, pattern, ace2k_tick_now_ms());
}
