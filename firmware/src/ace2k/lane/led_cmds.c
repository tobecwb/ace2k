// Binding of the led module to PE13/PE9/PC15/PE5, low = on (docs/hardware.md "Lane LEDs").
//
// The tick callback and set_lane run in the timer interrupt.  set_pattern and identify run in
// command context, in a task, with interrupts masked.  So the interrupt never sees it half done.
// The instance's fields, lanes_off_ms and intro_over included, and the last mask written, are
// changed by one context at a time, and the LED pins with them.
#include "lane/led_cmds.h"
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "board/gpio.h" // gpio_out_setup, gpio_out_write
#include "board/irq.h"  // irq_save, irq_restore
#include "command.h"    // DECL_CONSTANT_STR, shutdown
#include "sched.h"      // sched_shutdown (behind shutdown)

// The host reserves every RESERVE_PINS_* constant, so an [output_pin] cannot claim a lane LED.
// pins.h names the pins (docs/hardware.md "Lane LEDs").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_led", "PE13,PE9,PC15,PE5");

static struct gpio_out ace2k_led_lane_gpio[ACE2K_LANE_COUNT];
static struct ace2k_led ace2k_led_instance;
static bool ace2k_led_ready;

static void set_mask(void *ctx, uint8_t on_mask)
{
    (void)ctx;
    for (uint32_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        // active low: a lit lane is driven 0
        gpio_out_write(ace2k_led_lane_gpio[i], ((uint32_t)on_mask >> i) & 1U ? 0 : 1);
    }
}

static const struct ace2k_led_ops board_ops = { .set_mask = set_mask };

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    ace2k_led_tick(&ace2k_led_instance, now_ms);
}

static void ensure_ready(void)
{
    static const uint8_t pins[ACE2K_LANE_COUNT] = {
        ACE2K_PIN_LED_LANE1,
        ACE2K_PIN_LED_LANE2,
        ACE2K_PIN_LED_LANE3,
        ACE2K_PIN_LED_LANE4,
    };
    if (ace2k_led_ready) {
        return;
    }
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_led_lane_gpio[i] = gpio_out_setup(pins[i], 1);
    }
    ace2k_led_init(&ace2k_led_instance, &board_ops, NULL, ace2k_tick_now_ms());
    // A full registry is a programming error.  From init context this ends in a fault and a
    // watchdog reset.  Either way, the failure is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_led_ready = true;
}

void ace2k_led_binding_set_pattern(enum ace2k_led_pattern pattern)
{
    ensure_ready();
    irqstatus_t flag = irq_save(); // the tick and set_lane write the same fields (see the top)
    ace2k_led_set_pattern(&ace2k_led_instance, pattern, ace2k_tick_now_ms());
    irq_restore(flag);
}

// Called from the feed binding's tick, in interrupt context.  The instance is ready by then,
// because every DECL_INIT ran before the first tick.  A call before that is dropped, rather than
// set up GPIOs from an interrupt.
void ace2k_led_binding_set_lane(uint8_t lane, enum ace2k_led_lane_pattern pattern)
{
    if (!ace2k_led_ready) {
        return;
    }
    ace2k_led_set_lane(&ace2k_led_instance, lane, pattern, ace2k_tick_now_ms());
}

void ace2k_led_binding_identify(void)
{
    ensure_ready();
    irqstatus_t flag = irq_save(); // the tick and set_lane write the same fields (see the top)
    ace2k_led_identify(&ace2k_led_instance, ace2k_tick_now_ms());
    irq_restore(flag);
}

// The host or the flasher asks the unit to show itself, so a user can tell two units apart.
// Accepted in shutdown too: a unit that its host left in shutdown must still blink.  It changes
// the LEDs only.
// cppcheck-suppress constParameterPointer
void ace2k_led_cmd_identify(uint32_t *args)
{
    (void)args;
    ace2k_led_binding_identify();
    sendf("ace2k_identify_response ms=%u", (uint32_t)ACE2K_LED_IDENTIFY_MS);
}
DECL_COMMAND_FLAGS(ace2k_led_cmd_identify, HF_IN_SHUTDOWN, "ace2k_identify");
