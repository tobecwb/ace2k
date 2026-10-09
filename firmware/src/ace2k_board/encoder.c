// ace2k board encoders: TIM1 / TIM8 (APB2) and TIM3 / TIM5 (APB1) in encoder mode 3 (SMS =
// 011: count on both TI1 and TI2 edges), CC1/CC2 mapped to TI1/TI2, no filter (initial value;
// adjust on the bench if a lane counts at rest), ARR = 0xFFFF, free-running.  The inputs are
// pulled up before the timers run: a floating quadrature input chatters.  Register names and
// the encoder mode: GD32F30x User Manual, timer chapter (STM32F1-compatible layout).
#include "ace2k_board/encoder.h"
#include "ace2k_board/pins.h"
#include "ace2k/core/util.h" // ACE2K_LANE_COUNT
#include "autoconf.h"        // CONFIG_ACE2K_HEAT, CONFIG_ACE2K_RFID, CONFIG_ACE2K_SENSORS
#include "board/internal.h"  // TIM1, TIM3, TIM5, TIM8, enable_pclock, gpio_peripheral
#include "command.h"         // DECL_CONSTANT_STR

// The host reserves every RESERVE_PINS_* constant, so no [gcode_button] or [output_pin] can
// claim an encoder or tach input (pins.h names them; docs/hardware.md "Counters") — nor any
// other channel pin of the timers the lane owns: a [output_pin] with hardware_pwm on one of them
// takes the whole timer (Klipper's hard_pwm.c reprograms its prescaler, period and mode) and
// the lane's count with it.  Klipper's F1 hard_pwm table reaches TIM2, TIM3 and TIM4 only; of
// the lane's timers that is TIM3 (lane 3's encoder) and TIM4 (the tach), every channel pin in
// the default and the remapped position, and who reserves it in an image with the lane:
//   TIM3  PA6, PA7 (CH1, CH2, the encoder), PC6, PC7 (remapped CH1, CH2, lane 2's encoder on
//         TIM8): here; PC8, PC9 (remapped CH3, CH4 — the triac gate and the cutout latch reset,
//         zerocross.h and docs/hardware.md; outputs this image never touches): here; PB0, PB1
//         (CH3, CH4): the ADC scan with the sensors, here without them.
//   TIM4  PB6–PB9 (CH1–CH4, the tach): here; PD14 (remapped CH3): here; PD12, PD13 (remapped
//         CH1, CH2): the reader with it, here without it; PD15 (remapped CH4): the sensors
//         with them, here without them.
//   TIM1, TIM5, TIM8 (lanes 1, 4 and 2's encoders) are not in that table: no host section
//         reaches them by pin name, so their other channels (PA10, PA11 — the serial's; PA2, PA3
//         — TIM2's too, the motor's constant; PC8, PC9 — above) need nothing on their account.
// Klipper refuses a pin reserved under two names, hence the clauses: the triac gate PC8 is the
// heat binding's to reserve when it is built (heat_cmds.c, which owns the pin); PC9, the cutout's
// latch reset, stays here in every image with the lane — reserved so the host cannot drive it,
// never configured or driven by any code.
#if CONFIG_ACE2K_HEAT
#define ENCODER_RESERVE_GATE ""
#else
#define ENCODER_RESERVE_GATE ",PC8"
#endif
#if CONFIG_ACE2K_SENSORS
#define ENCODER_RESERVE_SENSORS ""
#else
#define ENCODER_RESERVE_SENSORS ",PB0,PB1,PD15"
#endif
#if CONFIG_ACE2K_RFID
#define ENCODER_RESERVE_RFID ""
#else
#define ENCODER_RESERVE_RFID ",PD12,PD13"
#endif
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_lane",
                  "PA8,PA9,PC6,PC7,PA6,PA7,PA0,PA1,PB6,PB7,PB8,PB9" ENCODER_RESERVE_GATE
                  ",PC9,PD14" ENCODER_RESERVE_SENSORS ENCODER_RESERVE_RFID);

#define ENCODER_SMS_MODE3 (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1)
#define ENCODER_CC_TI     (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0)
#define ENCODER_CC_EN     (TIM_CCER_CC1E | TIM_CCER_CC2E)

static TIM_TypeDef *const ace2k_encoder_tim[ACE2K_LANE_COUNT] = { TIM1, TIM8, TIM3, TIM5 };
static const uint8_t ace2k_encoder_pins[ACE2K_LANE_COUNT][2] = {
    { ACE2K_PIN_ENC1_A, ACE2K_PIN_ENC1_B },
    { ACE2K_PIN_ENC2_A, ACE2K_PIN_ENC2_B },
    { ACE2K_PIN_ENC3_A, ACE2K_PIN_ENC3_B },
    { ACE2K_PIN_ENC4_A, ACE2K_PIN_ENC4_B },
};

void ace2k_encoder_init(void)
{
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        // the two inputs pulled up first, the timer enabled last
        gpio_peripheral(ace2k_encoder_pins[i][0], GPIO_INPUT, 1);
        gpio_peripheral(ace2k_encoder_pins[i][1], GPIO_INPUT, 1);
        TIM_TypeDef *tim = ace2k_encoder_tim[i];
        enable_pclock((uint32_t)tim);
        tim->CR1 = 0;
        tim->SMCR = ENCODER_SMS_MODE3;
        tim->CCMR1 = ENCODER_CC_TI; /* IC1F = IC2F = 0: initial value; adjust on the bench */
        tim->CCER = ENCODER_CC_EN;
        tim->ARR = 0xFFFFU;
        tim->CNT = 0;
        tim->CR1 = TIM_CR1_CEN;
    }
}

uint16_t ace2k_encoder_read(uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? (uint16_t)ace2k_encoder_tim[lane]->CNT : 0;
}
