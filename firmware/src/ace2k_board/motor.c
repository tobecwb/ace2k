// ace2k board motors: TIM2 owned by ace2k — the full remap (channels 1–4 on PA15 / PB3 / PB10 /
// PB11, docs/hardware.md "Motors"), PWM mode 1 with the compare preloaded, 30 kHz from the
// timer clock, active low: compare = period holds the pin high (stop), compare = 0 drives it
// low the whole cycle.  Order at init: the run and direction pins low as outputs; the timer
// configured with every compare at stop and started; the remap; only then the PWM pins to
// alternate function — the driver's pull-up held them high until here, and the timer output is
// high from the first cycle.  The remap register's SWJ bits are write-only (they read 0) and
// Klipper set them to "JTAG off, SWD on" at boot: every write re-asserts that value.  Register
// names: GD32F30x User Manual, timer and AFIO chapters (STM32F1-compatible layout).
#include "ace2k_board/motor.h"
#include "ace2k_board/pins.h"
#include "ace2k/core/util.h" // ACE2K_LANE_COUNT
#include "autoconf.h"        // CONFIG_CLOCK_FREQ, CONFIG_ACE2K_SENSORS, CONFIG_ACE2K_HEAT
#include "board/gpio.h"      // gpio_out_setup, gpio_out_write
#include "board/internal.h"  // TIM2, AFIO, enable_pclock, gpio_peripheral, get_pclock_frequency
#include "command.h"         // DECL_CONSTANT_STR

#define MOTOR_PWM_HZ  30000U
#define MOTOR_PCT_MAX 100U
// PWM mode 1 (OCxM = 110) with the compare preloaded, for the two channels of each CCMR
#define MOTOR_CCMR_PAIR                                                                            \
    (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | \
     TIM_CCMR1_OC2PE)
#define MOTOR_CCER_ALL (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)

// The host reserves every RESERVE_PINS_* constant, so no [output_pin] or [gcode_button] can
// claim a motor line — the four PWM pins, the four run pins and the four direction pins
// (docs/hardware.md "Motors") — nor any other channel pin of TIM2: a [output_pin] with
// hardware_pwm on one takes the whole timer (Klipper's hard_pwm.c reprograms its prescaler,
// period and mode) and the four motors' PWM with it.  Every TIM2 channel pin in Klipper's F1
// hard_pwm table, default and remapped, and who reserves it in an image with the motors:
//   TIM2  PA15, PB3, PB10, PB11 (remapped CH1–CH4, the PWM lines): here; PA0, PA1 (CH1, CH2):
//         the lane's, lane 4's encoder, which the motors depend on; PA2, PA3 (CH3, CH4): the ADC
//         scan with the sensors, here without them.
// Klipper refuses a pin reserved under two names, hence the clause.
#if CONFIG_ACE2K_SENSORS
#define MOTOR_RESERVE_SENSORS ""
#else
#define MOTOR_RESERVE_SENSORS ",PA2,PA3"
#endif
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_motor",
                  "PA15,PB3,PB10,PB11,PE10,PE11,PB2,PE7,PE6,PC13,PE2,PE3" MOTOR_RESERVE_SENSORS);

// Klipper's F1 pin setup (stm32/stm32f1.c, gpio_peripheral) rewrites the whole remap register
// from a static shadow of its own writes — a shadow that never saw the TIM2 full remap — when a
// pin is configured for a function whose remap it emulates.  A host section on such a pin would
// erase the remap mid-move and put channels 1–2 on PA0/PA1, lane 4's encoder.  The pin-named host
// paths that reach that code are [output_pin] with hardware_pwm (TIM2, TIM3, TIM4 channel pins —
// every one of them reserved as a timer channel: the motor's above, the lane's in encoder.c) and
// the debug pair PA13/PA14, which rewrite the register on any function: reserved here.  The
// USART pairs PD5/PD6 and PD8/PD9 stay in the list though a USART is bus-named — harmless.
// Bus-named uses (spi_bus, i2c_bus, a USART) do not pass through pin reservation: the tick's
// re-check (ace2k_motor_remap_check) is the guard against those.  None of the pins listed here
// is driven by this firmware; a module that comes to own one moves it to its own constant.
// The USART2 remap pair PD5/PD6 are the rear flap's coils (docs/hardware.md "Exhaust flaps"): an
// image with the dryer's outputs owns and reserves them in RESERVE_PINS_ace2k_airflow
// (ace2k_board/airflow_io.c); without them they stay here.
#if CONFIG_ACE2K_HEAT
#define MOTOR_RESERVE_REMAP_PD56 ""
#else
#define MOTOR_RESERVE_REMAP_PD56 ",PD5,PD6"
#endif
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_remap", "PA13,PA14,PD8,PD9" MOTOR_RESERVE_REMAP_PD56);

static const uint8_t ace2k_motor_pwm_pins[ACE2K_LANE_COUNT] = {
    ACE2K_PIN_MOTOR_PWM1, ACE2K_PIN_MOTOR_PWM2, ACE2K_PIN_MOTOR_PWM3, ACE2K_PIN_MOTOR_PWM4
};
static const uint8_t ace2k_motor_run_pins[ACE2K_LANE_COUNT] = {
    ACE2K_PIN_MOTOR_RUN1, ACE2K_PIN_MOTOR_RUN2, ACE2K_PIN_MOTOR_RUN3, ACE2K_PIN_MOTOR_RUN4
};
static const uint8_t ace2k_motor_dir_pins[ACE2K_LANE_COUNT] = {
    ACE2K_PIN_MOTOR_DIR1, ACE2K_PIN_MOTOR_DIR2, ACE2K_PIN_MOTOR_DIR3, ACE2K_PIN_MOTOR_DIR4
};
static struct gpio_out ace2k_motor_run_gpio[ACE2K_LANE_COUNT];
static struct gpio_out ace2k_motor_dir_gpio[ACE2K_LANE_COUNT];
static uint32_t ace2k_motor_period; // timer ticks per PWM cycle; the compare at stop

static volatile uint32_t *compare_reg(uint8_t lane)
{
    switch (lane) {
    case 0:
        return &TIM2->CCR1;
    case 1:
        return &TIM2->CCR2;
    case 2:
        return &TIM2->CCR3;
    default:
        return &TIM2->CCR4;
    }
}

static void remap_full(void)
{
    uint32_t mapr = AFIO->MAPR & ~(AFIO_MAPR_TIM2_REMAP_Msk | AFIO_MAPR_SWJ_CFG_Msk);
    AFIO->MAPR = mapr | AFIO_MAPR_TIM2_REMAP_FULLREMAP | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
}

void ace2k_motor_init(void)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_motor_run_gpio[i] = gpio_out_setup(ace2k_motor_run_pins[i], 0);
        ace2k_motor_dir_gpio[i] = gpio_out_setup(ace2k_motor_dir_pins[i], 0);
    }
    enable_pclock((uint32_t)TIM2);
    // an F1 timer on a prescaled bus runs at twice the bus clock (60 MHz at 120 MHz, 72 at 72)
    uint32_t pclk = get_pclock_frequency((uint32_t)TIM2);
    uint32_t tim_hz = pclk < CONFIG_CLOCK_FREQ ? pclk * 2U : pclk;
    ace2k_motor_period = tim_hz / MOTOR_PWM_HZ;
    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = ace2k_motor_period - 1U;
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        *compare_reg(i) = ace2k_motor_period; // stop: the pin high the whole cycle
    }
    TIM2->CCMR1 = MOTOR_CCMR_PAIR;
    TIM2->CCMR2 = MOTOR_CCMR_PAIR; // the same bit positions for channels 3 and 4
    TIM2->CCER = MOTOR_CCER_ALL;   // active high on every channel (CCxP = 0)
    TIM2->EGR = TIM_EGR_UG;        // load the prescaler, the period and the compares
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    remap_full();
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        // alternate-function push-pull without a remap write (GPIO_FUNCTION(0) carries no
        // peripheral number for Klipper's F1 emulation to act on)
        gpio_peripheral(ace2k_motor_pwm_pins[i], GPIO_FUNCTION(0), 0);
    }
}

void ace2k_motor_remap_check(void)
{
    // never expected after init: nothing in this firmware writes the register, the pin-named host
    // uses that would are reserved (RESERVE_PINS_ace2k_remap), and the bus-named ones are what
    // this check is for
    if ((AFIO->MAPR & AFIO_MAPR_TIM2_REMAP_Msk) != AFIO_MAPR_TIM2_REMAP_FULLREMAP) {
        remap_full();
    }
}

void ace2k_motor_pwm(uint8_t lane, uint8_t duty_pct)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return;
    }
    ace2k_motor_remap_check();
    uint32_t duty = duty_pct > MOTOR_PCT_MAX ? MOTOR_PCT_MAX : duty_pct;
    // active low: the compare is the high time; 0 % → the period (never low), 100 % → 0
    *compare_reg(lane) = ace2k_motor_period - ace2k_motor_period * duty / MOTOR_PCT_MAX;
}

void ace2k_motor_run(uint8_t lane, bool on)
{
    if (lane < ACE2K_LANE_COUNT) {
        gpio_out_write(ace2k_motor_run_gpio[lane], on ? 1 : 0);
    }
}

void ace2k_motor_dir(uint8_t lane, bool reverse)
{
    if (lane < ACE2K_LANE_COUNT) {
        gpio_out_write(ace2k_motor_dir_gpio[lane], reverse ? 1 : 0);
    }
}

void ace2k_motor_stop_all(void)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        gpio_out_write(ace2k_motor_run_gpio[i], 0);
        *compare_reg(i) = ace2k_motor_period;
    }
}
