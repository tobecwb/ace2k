#include "test.h"
#include "ace2k_board/pins.h"
#include <stdbool.h>

TEST(pin_encoding_matches_the_port_letter_times_sixteen)
{
    ASSERT_EQ(ACE2K_PIN('A', 0), 0);
    ASSERT_EQ(ACE2K_PIN('A', 11), 11);
    ASSERT_EQ(ACE2K_PIN('C', 10), 42);
    ASSERT_EQ(ACE2K_PIN('E', 13), 77);
}

static bool table_has(uint8_t pin)
{
    size_t count = 0;
    const struct ace2k_pin_entry *t = ace2k_pins_table(&count);
    for (size_t i = 0; i < count; i++) {
        if (t[i].pin == pin) {
            return true;
        }
    }
    return false;
}

TEST(the_image_claims_the_sixty_nine_pins_of_the_briefs)
{
    size_t count = 0;
    const struct ace2k_pin_entry *t = ace2k_pins_table(&count);
    ASSERT_EQ(count, 69);
    ASSERT_EQ(t[0].pin, ACE2K_PIN('C', 10));
    ASSERT_EQ(t[0].cls, ACE2K_PIN_CLASS_LINK);
    ASSERT_EQ(t[6].pin, ACE2K_PIN('E', 5));
    ASSERT_EQ(t[6].cls, ACE2K_PIN_CLASS_LED);
    /* the seven pins the port image drives today: a typo in one must not pass */
    ASSERT_EQ(ACE2K_PIN_LINK_TX, ACE2K_PIN('C', 10));
    ASSERT_EQ(ACE2K_PIN_LINK_RX, ACE2K_PIN('C', 11));
    ASSERT_EQ(ACE2K_PIN_LINK_DE, ACE2K_PIN('A', 11));
    ASSERT_EQ(ACE2K_PIN_LED_LANE1, ACE2K_PIN('E', 13));
    ASSERT_EQ(ACE2K_PIN_LED_LANE2, ACE2K_PIN('E', 9));
    ASSERT_EQ(ACE2K_PIN_LED_LANE3, ACE2K_PIN('C', 15));
    ASSERT_EQ(ACE2K_PIN_LED_LANE4, ACE2K_PIN('E', 5));
    ASSERT_EQ(ACE2K_PIN_NTC_LEFT, ACE2K_PIN('A', 5));
    ASSERT_EQ(ACE2K_PIN_NTC_RIGHT, ACE2K_PIN('A', 4));
    ASSERT_EQ(ACE2K_PIN_INSERT1, ACE2K_PIN('C', 5));
    ASSERT_EQ(ACE2K_PIN_INSERT4, ACE2K_PIN('C', 2));
    ASSERT_EQ(ACE2K_PIN_EMPTY1, ACE2K_PIN('A', 2));
    ASSERT_EQ(ACE2K_PIN_EMPTY4, ACE2K_PIN('B', 1));
    ASSERT_EQ(ACE2K_PIN_AUX, ACE2K_PIN('C', 1));
    ASSERT_EQ(ACE2K_PIN_REST1, ACE2K_PIN('B', 4));
    ASSERT_EQ(ACE2K_PIN_REST4, ACE2K_PIN('E', 0));
    ASSERT_EQ(ACE2K_PIN_PUSHED1, ACE2K_PIN('C', 12));
    ASSERT_EQ(ACE2K_PIN_PUSHED4, ACE2K_PIN('D', 2));
    ASSERT_EQ(ACE2K_PIN_PULLED, ACE2K_PIN('E', 1));
    ASSERT_EQ(ACE2K_PIN_CUTOUT, ACE2K_PIN('D', 15));
    ASSERT_EQ(ACE2K_PIN_I2C_SCL, ACE2K_PIN('E', 14));
    ASSERT_EQ(ACE2K_PIN_I2C_SDA, ACE2K_PIN('E', 15));
    ASSERT_EQ(ACE2K_PIN_ENC1_A, ACE2K_PIN('A', 8));
    ASSERT_EQ(ACE2K_PIN_ENC2_A, ACE2K_PIN('C', 6));
    ASSERT_EQ(ACE2K_PIN_ENC3_A, ACE2K_PIN('A', 6));
    ASSERT_EQ(ACE2K_PIN_ENC4_B, ACE2K_PIN('A', 1));
    ASSERT_EQ(ACE2K_PIN_FG1, ACE2K_PIN('B', 6));
    ASSERT_EQ(ACE2K_PIN_FG4, ACE2K_PIN('B', 9));
    ASSERT_EQ(ACE2K_PIN_ZEROCROSS, ACE2K_PIN('C', 0));
    ASSERT_EQ(ACE2K_PIN_SPI_SCK, ACE2K_PIN('B', 13));
    ASSERT_EQ(ACE2K_PIN_RFID_A_NSS, ACE2K_PIN('B', 12));
    ASSERT_EQ(ACE2K_PIN_RFID_A_RST, ACE2K_PIN('D', 13));
    ASSERT_EQ(ACE2K_PIN_RFID_B_NSS, ACE2K_PIN('D', 10));
    ASSERT_EQ(ACE2K_PIN_RFID_B_RST, ACE2K_PIN('D', 12));
    ASSERT_EQ(ACE2K_PIN_MOTOR_PWM1, ACE2K_PIN('A', 15));
    ASSERT_EQ(ACE2K_PIN_MOTOR_PWM2, ACE2K_PIN('B', 3));
    ASSERT_EQ(ACE2K_PIN_MOTOR_PWM3, ACE2K_PIN('B', 10));
    ASSERT_EQ(ACE2K_PIN_MOTOR_PWM4, ACE2K_PIN('B', 11));
    ASSERT_EQ(ACE2K_PIN_MOTOR_RUN1, ACE2K_PIN('E', 10));
    ASSERT_EQ(ACE2K_PIN_MOTOR_RUN2, ACE2K_PIN('B', 2));
    ASSERT_EQ(ACE2K_PIN_MOTOR_RUN3, ACE2K_PIN('E', 6));
    ASSERT_EQ(ACE2K_PIN_MOTOR_RUN4, ACE2K_PIN('E', 2));
    ASSERT_EQ(ACE2K_PIN_MOTOR_DIR1, ACE2K_PIN('E', 11));
    ASSERT_EQ(ACE2K_PIN_MOTOR_DIR2, ACE2K_PIN('E', 7));
    ASSERT_EQ(ACE2K_PIN_MOTOR_DIR3, ACE2K_PIN('C', 13));
    ASSERT_EQ(ACE2K_PIN_MOTOR_DIR4, ACE2K_PIN('E', 3));
    size_t per_class[ACE2K_PIN_CLASS_COUNT] = { 0 };
    for (size_t i = 0; i < count; i++) {
        per_class[t[i].cls]++;
    }
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_ADC], 11);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_SWITCH], 10);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_ENCODER], 8);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_FG], 4);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_SPI], 5);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_RFID_RST], 2);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_MOTOR], 12);
    ASSERT_EQ(ACE2K_PIN_FAN_LEFT, ACE2K_PIN('E', 12));
    ASSERT_EQ(ACE2K_PIN_FAN_RIGHT, ACE2K_PIN('E', 8));
    ASSERT_EQ(ACE2K_PIN_FLAP_BOTTOM_OPEN, ACE2K_PIN('D', 4));
    ASSERT_EQ(ACE2K_PIN_FLAP_BOTTOM_CLOSE, ACE2K_PIN('D', 3));
    ASSERT_EQ(ACE2K_PIN_FLAP_REAR_OPEN, ACE2K_PIN('D', 6));
    ASSERT_EQ(ACE2K_PIN_FLAP_REAR_CLOSE, ACE2K_PIN('D', 5));
    ASSERT_EQ(ACE2K_PIN_TRIAC_GATE, ACE2K_PIN('C', 8));
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_FAN], 2);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_FLAP], 4);
    ASSERT_EQ(per_class[ACE2K_PIN_CLASS_TRIAC], 1);
}

/* The dryer's first rule: the cutout's latch-reset pin, PC9, is never driven, configured or
 * reserved by any image.  Every entry of the table is scanned for it, and every named output of
 * pins.h must be one of those entries — so no name can expand to PC9 without failing here. */
TEST(the_cutout_latch_reset_is_in_no_table_and_has_no_name)
{
    size_t count = 0;
    const struct ace2k_pin_entry *t = ace2k_pins_table(&count);
    for (size_t i = 0; i < count; i++) {
        ASSERT_TRUE(t[i].pin != ACE2K_PIN('C', 9));
    }
    const uint8_t named_outputs[] = {
        ACE2K_PIN_LINK_TX,           ACE2K_PIN_LINK_DE,        ACE2K_PIN_LED_LANE1,
        ACE2K_PIN_LED_LANE2,         ACE2K_PIN_LED_LANE3,      ACE2K_PIN_LED_LANE4,
        ACE2K_PIN_SPI_SCK,           ACE2K_PIN_SPI_MOSI,       ACE2K_PIN_RFID_A_NSS,
        ACE2K_PIN_RFID_B_NSS,        ACE2K_PIN_RFID_A_RST,     ACE2K_PIN_RFID_B_RST,
        ACE2K_PIN_MOTOR_PWM1,        ACE2K_PIN_MOTOR_PWM2,     ACE2K_PIN_MOTOR_PWM3,
        ACE2K_PIN_MOTOR_PWM4,        ACE2K_PIN_MOTOR_RUN1,     ACE2K_PIN_MOTOR_RUN2,
        ACE2K_PIN_MOTOR_RUN3,        ACE2K_PIN_MOTOR_RUN4,     ACE2K_PIN_MOTOR_DIR1,
        ACE2K_PIN_MOTOR_DIR2,        ACE2K_PIN_MOTOR_DIR3,     ACE2K_PIN_MOTOR_DIR4,
        ACE2K_PIN_FAN_LEFT,          ACE2K_PIN_FAN_RIGHT,      ACE2K_PIN_FLAP_BOTTOM_OPEN,
        ACE2K_PIN_FLAP_BOTTOM_CLOSE, ACE2K_PIN_FLAP_REAR_OPEN, ACE2K_PIN_FLAP_REAR_CLOSE,
        ACE2K_PIN_TRIAC_GATE,
    };
    for (size_t i = 0; i < sizeof named_outputs / sizeof named_outputs[0]; i++) {
        ASSERT_TRUE(table_has(named_outputs[i]));
    }
}

/* The gate is the table's only triac pin, and the classes of the dryer's outputs are exact: a pin
 * of another class is never taken for the gate. */
TEST(the_gate_is_the_only_triac_pin_and_the_outputs_have_their_classes)
{
    size_t count = 0;
    const struct ace2k_pin_entry *t = ace2k_pins_table(&count);
    for (size_t i = 0; i < count; i++) {
        if (t[i].cls == ACE2K_PIN_CLASS_TRIAC) {
            ASSERT_EQ(t[i].pin, ACE2K_PIN_TRIAC_GATE);
        }
        if (t[i].pin == ACE2K_PIN_TRIAC_GATE) {
            ASSERT_EQ(t[i].cls, ACE2K_PIN_CLASS_TRIAC);
        }
        if (t[i].pin == ACE2K_PIN_FAN_LEFT || t[i].pin == ACE2K_PIN_FAN_RIGHT) {
            ASSERT_EQ(t[i].cls, ACE2K_PIN_CLASS_FAN);
        }
        if (t[i].pin == ACE2K_PIN('D', 3) || t[i].pin == ACE2K_PIN('D', 4) ||
            t[i].pin == ACE2K_PIN('D', 5) || t[i].pin == ACE2K_PIN('D', 6)) {
            ASSERT_EQ(t[i].cls, ACE2K_PIN_CLASS_FLAP);
        }
    }
}

TEST(no_pin_is_claimed_twice)
{
    size_t count = 0;
    const struct ace2k_pin_entry *t = ace2k_pins_table(&count);
    for (size_t i = 0; i < count; i++) {
        for (size_t j = i + 1; j < count; j++) {
            ASSERT_TRUE(t[i].pin != t[j].pin);
        }
    }
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
