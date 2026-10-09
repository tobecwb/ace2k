// ace2k board air path: the fans and the flap coils as GPIO outputs (docs/hardware.md "Fans",
// "Exhaust flaps").  A fan's read-back is the pin's input register: on an F1-compatible port the
// input register follows the pin in output mode too (GD32F30x User Manual, GPIO chapter), so a
// pin held low by a fault reads low while the output register says high.
#include "ace2k_board/airflow_io.h"
#include "ace2k_board/pins.h"
#include "board/gpio.h"     // gpio_out_setup, gpio_out_write, struct gpio_out
#include "board/internal.h" // GPIO_TypeDef
#include "command.h"        // DECL_CONSTANT_STR

// The host reserves every RESERVE_PINS_* constant, so no [output_pin] or [fan] section can
// claim a fan or a flap coil (pins.h names them; docs/hardware.md "Fans", "Exhaust flaps").
// PD5/PD6 leave RESERVE_PINS_ace2k_remap in this image (ace2k_board/motor.c).
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_airflow", "PE12,PE8,PD3,PD4,PD5,PD6");

static struct gpio_out ace2k_airflow_io_fan_left;
static struct gpio_out ace2k_airflow_io_fan_right;
static struct gpio_out ace2k_airflow_io_open[ACE2K_AIRFLOW_IO_FLAPS];
static struct gpio_out ace2k_airflow_io_close[ACE2K_AIRFLOW_IO_FLAPS];

void ace2k_airflow_io_init(void)
{
    ace2k_airflow_io_fan_left = gpio_out_setup(ACE2K_PIN_FAN_LEFT, 0);
    ace2k_airflow_io_fan_right = gpio_out_setup(ACE2K_PIN_FAN_RIGHT, 0);
    ace2k_airflow_io_open[0] = gpio_out_setup(ACE2K_PIN_FLAP_BOTTOM_OPEN, 0);
    ace2k_airflow_io_close[0] = gpio_out_setup(ACE2K_PIN_FLAP_BOTTOM_CLOSE, 0);
    ace2k_airflow_io_open[1] = gpio_out_setup(ACE2K_PIN_FLAP_REAR_OPEN, 0);
    ace2k_airflow_io_close[1] = gpio_out_setup(ACE2K_PIN_FLAP_REAR_CLOSE, 0);
}

void ace2k_airflow_io_fans(bool on)
{
    gpio_out_write(ace2k_airflow_io_fan_left, on ? 1U : 0U);
    gpio_out_write(ace2k_airflow_io_fan_right, on ? 1U : 0U);
}

static bool pin_reads_high(struct gpio_out g)
{
    const GPIO_TypeDef *regs = g.regs;
    return (regs->IDR & g.bit) != 0U;
}

uint8_t ace2k_airflow_io_fans_read(void)
{
    uint8_t bits = 0;
    if (pin_reads_high(ace2k_airflow_io_fan_left)) {
        bits |= 1U;
    }
    if (pin_reads_high(ace2k_airflow_io_fan_right)) {
        bits |= 2U;
    }
    return bits;
}

void ace2k_airflow_io_flap(uint8_t flap, bool open_coil, bool close_coil)
{
    if (flap >= ACE2K_AIRFLOW_IO_FLAPS) {
        return;
    }
    // the coil going low first: the two are never high together, not even for one write
    if (!open_coil) {
        gpio_out_write(ace2k_airflow_io_open[flap], 0);
    }
    if (!close_coil) {
        gpio_out_write(ace2k_airflow_io_close[flap], 0);
    }
    if (open_coil && !close_coil) {
        gpio_out_write(ace2k_airflow_io_open[flap], 1);
    }
    if (close_coil && !open_coil) {
        gpio_out_write(ace2k_airflow_io_close[flap], 1);
    }
}
