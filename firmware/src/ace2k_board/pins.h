/* What: every pin the firmware drives or reads, named once, with the measurement behind it.
 * How: bindings and board code use the ACE2K_PIN_* names; ace2k_pins_table() is the allow-list
 * a test walks.  ACE2K_PIN() reproduces Klipper's GPIO() encoding (port letter × 16 + number)
 * without including Klipper, so this header compiles on the host.  Each subsystem extends
 * the table: the read-only ones added everything the board exposes as an input, the readers' bus
 * and their reset lines; the feed added the motor lines; the dryer added the fans, the
 * flaps and the triac gate.  The cutout's latch-reset pin (PC9) is never named here: no image
 * drives, configures or reserves it.
 * Depends on: <stddef.h>, <stdint.h>. */
#ifndef ACE2K_BOARD_PINS_H
#define ACE2K_BOARD_PINS_H
#include <stddef.h>
#include <stdint.h>

#define ACE2K_PIN(port, num) ((uint8_t)((((port) - 'A') * 16) + (num)))

/* Host link — docs/hardware.md "Host link", measured 2026-09-12. */
#define ACE2K_PIN_LINK_TX ACE2K_PIN('C', 10) /* UART4 TX */
#define ACE2K_PIN_LINK_RX ACE2K_PIN('C', 11) /* UART4 RX */
#define ACE2K_PIN_LINK_DE ACE2K_PIN('A', 11) /* transceiver driver-enable, high = transmit */

/* Lane LEDs — docs/hardware.md "Lane LEDs", measured 2026-09-12; low = on. */
#define ACE2K_PIN_LED_LANE1 ACE2K_PIN('E', 13)
#define ACE2K_PIN_LED_LANE2 ACE2K_PIN('E', 9)
#define ACE2K_PIN_LED_LANE3 ACE2K_PIN('C', 15)
#define ACE2K_PIN_LED_LANE4 ACE2K_PIN('E', 5)

/* Analogue inputs — docs/hardware.md "Analogue inputs", measured 2026-09-12 (the NTC sides
 * 2026-09-13).  ADC1 channels in the comments. */
#define ACE2K_PIN_NTC_LEFT  ACE2K_PIN('A', 5) /* ch 5, the left outlet NTC */
#define ACE2K_PIN_NTC_RIGHT ACE2K_PIN('A', 4) /* ch 4, the right outlet NTC */
#define ACE2K_PIN_INSERT1   ACE2K_PIN('C', 5) /* ch 15 */
#define ACE2K_PIN_INSERT2   ACE2K_PIN('C', 4) /* ch 14 */
#define ACE2K_PIN_INSERT3   ACE2K_PIN('C', 3) /* ch 13 */
#define ACE2K_PIN_INSERT4   ACE2K_PIN('C', 2) /* ch 12 */
#define ACE2K_PIN_EMPTY1    ACE2K_PIN('A', 2) /* ch 2 — never read by the factory firmware */
#define ACE2K_PIN_EMPTY2    ACE2K_PIN('A', 3) /* ch 3 */
#define ACE2K_PIN_EMPTY3    ACE2K_PIN('B', 0) /* ch 8 */
#define ACE2K_PIN_EMPTY4    ACE2K_PIN('B', 1) /* ch 9 */
#define ACE2K_PIN_AUX       ACE2K_PIN('C', 1) /* ch 11, identity unknown */

/* Digital inputs — docs/hardware.md "Digital inputs", measured 2026-09-12; the Hall switches
 * are active low. */
#define ACE2K_PIN_REST1   ACE2K_PIN('B', 4)
#define ACE2K_PIN_REST2   ACE2K_PIN('B', 5)
#define ACE2K_PIN_REST3   ACE2K_PIN('D', 7)
#define ACE2K_PIN_REST4   ACE2K_PIN('E', 0)
#define ACE2K_PIN_PUSHED1 ACE2K_PIN('C', 12)
#define ACE2K_PIN_PUSHED2 ACE2K_PIN('D', 0)
#define ACE2K_PIN_PUSHED3 ACE2K_PIN('D', 1)
#define ACE2K_PIN_PUSHED4 ACE2K_PIN('D', 2)
#define ACE2K_PIN_PULLED  ACE2K_PIN('E', 1)  /* wired-OR of the four pulled-end switches */
#define ACE2K_PIN_CUTOUT  ACE2K_PIN('D', 15) /* thermal-cutout fault input; idle reads 0 */

/* Chamber sensor — docs/hardware.md "Chamber sensor", measured 2026-09-12; software I²C. */
#define ACE2K_PIN_I2C_SCL ACE2K_PIN('E', 14)
#define ACE2K_PIN_I2C_SDA ACE2K_PIN('E', 15)

/* Filament encoders — docs/hardware.md "Counters", measured 2026-09-12; x4 quadrature. */
#define ACE2K_PIN_ENC1_A ACE2K_PIN('A', 8) /* TIM1 CH1 */
#define ACE2K_PIN_ENC1_B ACE2K_PIN('A', 9) /* TIM1 CH2 */
#define ACE2K_PIN_ENC2_A ACE2K_PIN('C', 6) /* TIM8 CH1 */
#define ACE2K_PIN_ENC2_B ACE2K_PIN('C', 7) /* TIM8 CH2 */
#define ACE2K_PIN_ENC3_A ACE2K_PIN('A', 6) /* TIM3 CH1 */
#define ACE2K_PIN_ENC3_B ACE2K_PIN('A', 7) /* TIM3 CH2 */
#define ACE2K_PIN_ENC4_A ACE2K_PIN('A', 0) /* TIM5 CH1 */
#define ACE2K_PIN_ENC4_B ACE2K_PIN('A', 1) /* TIM5 CH2 */

/* Motor tach (FG) outputs — docs/hardware.md "Counters", measured 2026-09-12; TIM4 CH1–4. */
#define ACE2K_PIN_FG1 ACE2K_PIN('B', 6)
#define ACE2K_PIN_FG2 ACE2K_PIN('B', 7)
#define ACE2K_PIN_FG3 ACE2K_PIN('B', 8)
#define ACE2K_PIN_FG4 ACE2K_PIN('B', 9)

/* Mains zero-cross — docs/hardware.md "Counters", measured 2026-09-12; EXTI line 0. */
#define ACE2K_PIN_ZEROCROSS ACE2K_PIN('C', 0)

/* RFID readers — docs/hardware.md "RFID readers", measured 2026-09-13.  Reader A serves lanes
 * 3–4, reader B lanes 1–2.  RST is driven high only (running); the IRQ lines are not used. */
#define ACE2K_PIN_SPI_SCK    ACE2K_PIN('B', 13)
#define ACE2K_PIN_SPI_MISO   ACE2K_PIN('B', 14)
#define ACE2K_PIN_SPI_MOSI   ACE2K_PIN('B', 15)
#define ACE2K_PIN_RFID_A_NSS ACE2K_PIN('B', 12)
#define ACE2K_PIN_RFID_A_RST ACE2K_PIN('D', 13)
#define ACE2K_PIN_RFID_B_NSS ACE2K_PIN('D', 10)
#define ACE2K_PIN_RFID_B_RST ACE2K_PIN('D', 12)

/* Lane motors — docs/hardware.md "Motors", measured 2026-09-12.  PWM on TIM2 CH1–4 (the full
 * remap), 30 kHz, active low: the pin high is stop, and the driver's pull-up holds a floating
 * pin high.  Run (A): 1 while a move runs.  Direction (B): 0 toward the printer, 1 toward the
 * spool. */
#define ACE2K_PIN_MOTOR_PWM1 ACE2K_PIN('A', 15) /* TIM2 CH1 */
#define ACE2K_PIN_MOTOR_PWM2 ACE2K_PIN('B', 3)  /* TIM2 CH2 */
#define ACE2K_PIN_MOTOR_PWM3 ACE2K_PIN('B', 10) /* TIM2 CH3 */
#define ACE2K_PIN_MOTOR_PWM4 ACE2K_PIN('B', 11) /* TIM2 CH4 */
#define ACE2K_PIN_MOTOR_RUN1 ACE2K_PIN('E', 10)
#define ACE2K_PIN_MOTOR_RUN2 ACE2K_PIN('B', 2)
#define ACE2K_PIN_MOTOR_RUN3 ACE2K_PIN('E', 6)
#define ACE2K_PIN_MOTOR_RUN4 ACE2K_PIN('E', 2)
#define ACE2K_PIN_MOTOR_DIR1 ACE2K_PIN('E', 11)
#define ACE2K_PIN_MOTOR_DIR2 ACE2K_PIN('E', 7)
#define ACE2K_PIN_MOTOR_DIR3 ACE2K_PIN('C', 13)
#define ACE2K_PIN_MOTOR_DIR4 ACE2K_PIN('E', 3)

/* Dryer fans — docs/hardware.md "Fans", measured 2026-09-12 and 2026-09-13; high = on, external
 * pull-downs (released = off).  No tachometer: the pin is read back, never the rotation. */
#define ACE2K_PIN_FAN_LEFT  ACE2K_PIN('E', 12)
#define ACE2K_PIN_FAN_RIGHT ACE2K_PIN('E', 8)

/* Exhaust flaps — docs/hardware.md "Exhaust flaps", measured 2026-09-12; each input drives one
 * side of the flap's H-bridge, high for a pulse, low otherwise (both low = coil released). */
#define ACE2K_PIN_FLAP_BOTTOM_OPEN  ACE2K_PIN('D', 4)
#define ACE2K_PIN_FLAP_BOTTOM_CLOSE ACE2K_PIN('D', 3)
#define ACE2K_PIN_FLAP_REAR_OPEN    ACE2K_PIN('D', 6)
#define ACE2K_PIN_FLAP_REAR_CLOSE   ACE2K_PIN('D', 5)

/* Heater triac gate — docs/hardware.md "Heater", measured 2026-09-12 (idle, pull-down) and
 * 2026-09-13 (driven): high = heating.  Configured by ace2k_board/gate.c alone. */
#define ACE2K_PIN_TRIAC_GATE ACE2K_PIN('C', 8)

enum ace2k_pin_class {
    ACE2K_PIN_CLASS_LINK,
    ACE2K_PIN_CLASS_LED,
    ACE2K_PIN_CLASS_ADC,
    ACE2K_PIN_CLASS_SWITCH,
    ACE2K_PIN_CLASS_I2C,
    ACE2K_PIN_CLASS_ENCODER,
    ACE2K_PIN_CLASS_FG,
    ACE2K_PIN_CLASS_ZEROCROSS,
    ACE2K_PIN_CLASS_SPI,
    ACE2K_PIN_CLASS_RFID_RST,
    ACE2K_PIN_CLASS_MOTOR,
    ACE2K_PIN_CLASS_FAN,
    ACE2K_PIN_CLASS_FLAP,
    ACE2K_PIN_CLASS_TRIAC,
    ACE2K_PIN_CLASS_COUNT,
};

struct ace2k_pin_entry {
    uint8_t pin;
    enum ace2k_pin_class cls;
    const char *name;
};

/* The allow-list: every pin above, in this order.  A pin not in this table is not configured by
 * any ace2k code; each subsystem extends it. */
static inline const struct ace2k_pin_entry *ace2k_pins_table(size_t *count)
{
    static const struct ace2k_pin_entry table[] = {
        { ACE2K_PIN_LINK_TX, ACE2K_PIN_CLASS_LINK, "link_tx" },
        { ACE2K_PIN_LINK_RX, ACE2K_PIN_CLASS_LINK, "link_rx" },
        { ACE2K_PIN_LINK_DE, ACE2K_PIN_CLASS_LINK, "link_de" },
        { ACE2K_PIN_LED_LANE1, ACE2K_PIN_CLASS_LED, "led_lane1" },
        { ACE2K_PIN_LED_LANE2, ACE2K_PIN_CLASS_LED, "led_lane2" },
        { ACE2K_PIN_LED_LANE3, ACE2K_PIN_CLASS_LED, "led_lane3" },
        { ACE2K_PIN_LED_LANE4, ACE2K_PIN_CLASS_LED, "led_lane4" },
        { ACE2K_PIN_NTC_LEFT, ACE2K_PIN_CLASS_ADC, "ntc_left" },
        { ACE2K_PIN_NTC_RIGHT, ACE2K_PIN_CLASS_ADC, "ntc_right" },
        { ACE2K_PIN_INSERT1, ACE2K_PIN_CLASS_ADC, "insert1" },
        { ACE2K_PIN_INSERT2, ACE2K_PIN_CLASS_ADC, "insert2" },
        { ACE2K_PIN_INSERT3, ACE2K_PIN_CLASS_ADC, "insert3" },
        { ACE2K_PIN_INSERT4, ACE2K_PIN_CLASS_ADC, "insert4" },
        { ACE2K_PIN_EMPTY1, ACE2K_PIN_CLASS_ADC, "empty1" },
        { ACE2K_PIN_EMPTY2, ACE2K_PIN_CLASS_ADC, "empty2" },
        { ACE2K_PIN_EMPTY3, ACE2K_PIN_CLASS_ADC, "empty3" },
        { ACE2K_PIN_EMPTY4, ACE2K_PIN_CLASS_ADC, "empty4" },
        { ACE2K_PIN_AUX, ACE2K_PIN_CLASS_ADC, "aux" },
        { ACE2K_PIN_REST1, ACE2K_PIN_CLASS_SWITCH, "rest1" },
        { ACE2K_PIN_REST2, ACE2K_PIN_CLASS_SWITCH, "rest2" },
        { ACE2K_PIN_REST3, ACE2K_PIN_CLASS_SWITCH, "rest3" },
        { ACE2K_PIN_REST4, ACE2K_PIN_CLASS_SWITCH, "rest4" },
        { ACE2K_PIN_PUSHED1, ACE2K_PIN_CLASS_SWITCH, "pushed1" },
        { ACE2K_PIN_PUSHED2, ACE2K_PIN_CLASS_SWITCH, "pushed2" },
        { ACE2K_PIN_PUSHED3, ACE2K_PIN_CLASS_SWITCH, "pushed3" },
        { ACE2K_PIN_PUSHED4, ACE2K_PIN_CLASS_SWITCH, "pushed4" },
        { ACE2K_PIN_PULLED, ACE2K_PIN_CLASS_SWITCH, "pulled" },
        { ACE2K_PIN_CUTOUT, ACE2K_PIN_CLASS_SWITCH, "cutout" },
        { ACE2K_PIN_I2C_SCL, ACE2K_PIN_CLASS_I2C, "i2c_scl" },
        { ACE2K_PIN_I2C_SDA, ACE2K_PIN_CLASS_I2C, "i2c_sda" },
        { ACE2K_PIN_ENC1_A, ACE2K_PIN_CLASS_ENCODER, "enc1_a" },
        { ACE2K_PIN_ENC1_B, ACE2K_PIN_CLASS_ENCODER, "enc1_b" },
        { ACE2K_PIN_ENC2_A, ACE2K_PIN_CLASS_ENCODER, "enc2_a" },
        { ACE2K_PIN_ENC2_B, ACE2K_PIN_CLASS_ENCODER, "enc2_b" },
        { ACE2K_PIN_ENC3_A, ACE2K_PIN_CLASS_ENCODER, "enc3_a" },
        { ACE2K_PIN_ENC3_B, ACE2K_PIN_CLASS_ENCODER, "enc3_b" },
        { ACE2K_PIN_ENC4_A, ACE2K_PIN_CLASS_ENCODER, "enc4_a" },
        { ACE2K_PIN_ENC4_B, ACE2K_PIN_CLASS_ENCODER, "enc4_b" },
        { ACE2K_PIN_FG1, ACE2K_PIN_CLASS_FG, "fg1" },
        { ACE2K_PIN_FG2, ACE2K_PIN_CLASS_FG, "fg2" },
        { ACE2K_PIN_FG3, ACE2K_PIN_CLASS_FG, "fg3" },
        { ACE2K_PIN_FG4, ACE2K_PIN_CLASS_FG, "fg4" },
        { ACE2K_PIN_ZEROCROSS, ACE2K_PIN_CLASS_ZEROCROSS, "zerocross" },
        { ACE2K_PIN_SPI_SCK, ACE2K_PIN_CLASS_SPI, "spi_sck" },
        { ACE2K_PIN_SPI_MISO, ACE2K_PIN_CLASS_SPI, "spi_miso" },
        { ACE2K_PIN_SPI_MOSI, ACE2K_PIN_CLASS_SPI, "spi_mosi" },
        { ACE2K_PIN_RFID_A_NSS, ACE2K_PIN_CLASS_SPI, "rfid_a_nss" },
        { ACE2K_PIN_RFID_B_NSS, ACE2K_PIN_CLASS_SPI, "rfid_b_nss" },
        { ACE2K_PIN_RFID_A_RST, ACE2K_PIN_CLASS_RFID_RST, "rfid_a_rst" },
        { ACE2K_PIN_RFID_B_RST, ACE2K_PIN_CLASS_RFID_RST, "rfid_b_rst" },
        { ACE2K_PIN_MOTOR_PWM1, ACE2K_PIN_CLASS_MOTOR, "motor_pwm1" },
        { ACE2K_PIN_MOTOR_PWM2, ACE2K_PIN_CLASS_MOTOR, "motor_pwm2" },
        { ACE2K_PIN_MOTOR_PWM3, ACE2K_PIN_CLASS_MOTOR, "motor_pwm3" },
        { ACE2K_PIN_MOTOR_PWM4, ACE2K_PIN_CLASS_MOTOR, "motor_pwm4" },
        { ACE2K_PIN_MOTOR_RUN1, ACE2K_PIN_CLASS_MOTOR, "motor_run1" },
        { ACE2K_PIN_MOTOR_RUN2, ACE2K_PIN_CLASS_MOTOR, "motor_run2" },
        { ACE2K_PIN_MOTOR_RUN3, ACE2K_PIN_CLASS_MOTOR, "motor_run3" },
        { ACE2K_PIN_MOTOR_RUN4, ACE2K_PIN_CLASS_MOTOR, "motor_run4" },
        { ACE2K_PIN_MOTOR_DIR1, ACE2K_PIN_CLASS_MOTOR, "motor_dir1" },
        { ACE2K_PIN_MOTOR_DIR2, ACE2K_PIN_CLASS_MOTOR, "motor_dir2" },
        { ACE2K_PIN_MOTOR_DIR3, ACE2K_PIN_CLASS_MOTOR, "motor_dir3" },
        { ACE2K_PIN_MOTOR_DIR4, ACE2K_PIN_CLASS_MOTOR, "motor_dir4" },
        { ACE2K_PIN_FAN_LEFT, ACE2K_PIN_CLASS_FAN, "fan_left" },
        { ACE2K_PIN_FAN_RIGHT, ACE2K_PIN_CLASS_FAN, "fan_right" },
        { ACE2K_PIN_FLAP_BOTTOM_OPEN, ACE2K_PIN_CLASS_FLAP, "flap_bottom_open" },
        { ACE2K_PIN_FLAP_BOTTOM_CLOSE, ACE2K_PIN_CLASS_FLAP, "flap_bottom_close" },
        { ACE2K_PIN_FLAP_REAR_OPEN, ACE2K_PIN_CLASS_FLAP, "flap_rear_open" },
        { ACE2K_PIN_FLAP_REAR_CLOSE, ACE2K_PIN_CLASS_FLAP, "flap_rear_close" },
        { ACE2K_PIN_TRIAC_GATE, ACE2K_PIN_CLASS_TRIAC, "triac_gate" },
    };
    *count = sizeof table / sizeof table[0];
    return table;
}

#endif
