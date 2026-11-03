// ace2k board encoders. TIM1 / TIM8 (APB2) and TIM3 / TIM5 (APB1) run in encoder mode 3
// (SMS = 011: count on both TI1 and TI2 edges). CC1/CC2 map to TI1/TI2. There is no filter
// (initial value; re-tune on the bench if a lane counts at rest). ARR = 0xFFFF, free-running. The
// inputs are pulled up before the timers run, because a floating quadrature input chatters.
// Register names and the encoder mode: GD32F30x User Manual, timer chapter (STM32F1-compatible
// layout).
#include "ace2k_board/encoder.h"
#include "ace2k_board/tim.h" // ACE2K_TIM_ARR_FREE_RUNNING
#include "ace2k_board/pins.h"
#include "ace2k/core/util.h" // ACE2K_LANE_COUNT
#include "autoconf.h"        // CONFIG_ACE2K_RFID, CONFIG_ACE2K_SENSORS
#include "board/internal.h"  // TIM1, TIM3, TIM5, TIM8, enable_pclock, gpio_peripheral
#include "command.h"         // DECL_CONSTANT_STR

// The host reserves every RESERVE_PINS_* constant. So no [gcode_button] or [output_pin] can claim
// an encoder or tach input. pins.h names them (docs/hardware.md "Counters"). Nor can it claim any
// other channel pin of the timers that the lane owns. An [output_pin] with hardware_pwm on one of
// them takes the whole timer, and the lane's count with it: Klipper's hard_pwm.c reprograms
// the prescaler, the period and the mode.
// Klipper's F1 hard_pwm table reaches TIM2, TIM3 and TIM4 only. Of the lane's timers,
// that is TIM3 (the encoder of lane 3) and TIM4 (the tach). Here is every channel pin in the
// default and the remapped position, and who reserves it in an image with the lane:
//   TIM3  PA6, PA7 (CH1, CH2, the encoder): here. PC6, PC7 (remapped CH1, CH2; the encoder of
//         lane 2, on TIM8): here. PC8 (remapped CH3): the heater when it is built, tick.c when
//         it is not. PC9 (remapped CH4): tick.c, in every image. They are the triac gate and the
//         cutout's latch reset (zerocross.h and docs/hardware.md), outputs that this file never
//         touches. PB0, PB1 (CH3, CH4): the ADC scan when the sensors are built,
//         here when they are not.
//   TIM4  PB6–PB9 (CH1–CH4, the tach): here. PD14 (remapped CH3): here. PD12, PD13 (remapped
//         CH1, CH2): the reader when it is built, here when it is not. PD15 (remapped CH4): the
//         sensors when they are built, here when they are not.
//   TIM1, TIM5, TIM8 (the encoders of lanes 1, 4 and 2) are not in that table. No host section
//         reaches them by pin name. So their other channels need nothing on their account: PA10,
//         PA11 (the serial link), PA2, PA3 (also TIM2 channels, so the motor's constant covers
//         them), PC8, PC9 (above).
// Klipper refuses a pin reserved under two names, hence the clauses. The triac gate PC8 is
// reserved by the heat binding when it is built (heat_cmds.c, which owns the pin), and by
// ace2k_board/tick.c when it is not. PC9, the latch reset of the cutout, is reserved in every
// image by ace2k_board/tick.c, so the host cannot drive it, and no code ever configures or
// drives it.
#if CONFIG_ACE2K_SENSORS
#define ACE2K_ENCODER_RESERVE_SENSORS ""
#else
#define ACE2K_ENCODER_RESERVE_SENSORS ",PB0,PB1,PD15"
#endif
#if CONFIG_ACE2K_RFID
#define ACE2K_ENCODER_RESERVE_RFID ""
#else
#define ACE2K_ENCODER_RESERVE_RFID ",PD12,PD13"
#endif
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_lane",
                  "PA8,PA9,PC6,PC7,PA6,PA7,PA0,PA1,PB6,PB7,PB8,PB9"
                  ",PD14" ACE2K_ENCODER_RESERVE_SENSORS ACE2K_ENCODER_RESERVE_RFID);

#define ACE2K_ENCODER_SMS_MODE3 (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)
#define ACE2K_ENCODER_CC_TI     (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0)
#define ACE2K_ENCODER_CC_EN     (TIM_CCER_CC1E | TIM_CCER_CC2E)

static TIM_TypeDef *const encoder_tim[ACE2K_LANE_COUNT] = { TIM1, TIM8, TIM3, TIM5 };
static const uint8_t encoder_pins[ACE2K_LANE_COUNT][2] = {
    { ACE2K_PIN_ENC1_A, ACE2K_PIN_ENC1_B },
    { ACE2K_PIN_ENC2_A, ACE2K_PIN_ENC2_B },
    { ACE2K_PIN_ENC3_A, ACE2K_PIN_ENC3_B },
    { ACE2K_PIN_ENC4_A, ACE2K_PIN_ENC4_B },
};

void ace2k_encoder_init(void)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        // The two inputs are pulled up first, and the timer is enabled last.
        gpio_peripheral(encoder_pins[i][0], GPIO_INPUT, 1);
        gpio_peripheral(encoder_pins[i][1], GPIO_INPUT, 1);
        TIM_TypeDef *tim = encoder_tim[i];
        enable_pclock((uint32_t)tim);
        tim->CR1 = 0;
        tim->SMCR = ACE2K_ENCODER_SMS_MODE3;
        tim->CCMR1 = ACE2K_ENCODER_CC_TI; /* IC1F = IC2F = 0: initial value; re-tune on the bench */
        tim->CCER = ACE2K_ENCODER_CC_EN;
        tim->ARR = ACE2K_TIM_ARR_FREE_RUNNING;
        tim->CNT = 0;
        tim->CR1 = TIM_CR1_CEN;
    }
}

uint16_t ace2k_encoder_read(uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? (uint16_t)encoder_tim[lane]->CNT : 0;
}
