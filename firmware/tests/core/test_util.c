#include "test.h"
#include "core/util.h"

TEST(time_after_is_false_before_the_deadline)
{
    ASSERT_TRUE(!ace2k_time_after(999, 1000));
}

TEST(time_after_is_true_at_and_past_the_deadline)
{
    ASSERT_TRUE(ace2k_time_after(1000, 1000));
    ASSERT_TRUE(ace2k_time_after(1001, 1000));
}

TEST(time_after_survives_the_wrap)
{
    uint32_t deadline = 0xFFFFFFF0U + 16U; /* wraps to 0 */
    ASSERT_TRUE(!ace2k_time_after(0xFFFFFFF0U, deadline));
    ASSERT_TRUE(ace2k_time_after(0U, deadline));
    ASSERT_TRUE(ace2k_time_after(5U, deadline));
}

TEST(time_since_survives_the_wrap)
{
    ASSERT_EQ(ace2k_time_since(4U, 0xFFFFFFFEU), 6);
    ASSERT_EQ(ace2k_time_since(1000U, 400U), 600);
}

TEST(clamp_i32_bounds_both_sides)
{
    ASSERT_EQ(ace2k_clamp_i32(-5, 0, 10), 0);
    ASSERT_EQ(ace2k_clamp_i32(15, 0, 10), 10);
    ASSERT_EQ(ace2k_clamp_i32(7, 0, 10), 7);
}

TEST(array_size_counts_elements)
{
    static const uint16_t four[4] = { 1, 2, 3, 4 };
    ASSERT_EQ(ACE2K_ARRAY_SIZE(four), 4);
}

TEST(crc16_ccitt_matches_the_check_value)
{
    static const uint8_t digits[9] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    ASSERT_EQ(ace2k_crc16_ccitt(digits, sizeof digits), 0x29B1);
}

TEST(crc16_ccitt_of_nothing_is_the_initial_value)
{
    static const uint8_t none[1] = { 0 };
    ASSERT_EQ(ace2k_crc16_ccitt(none, 0), 0xFFFF);
}

TEST(str_equal_compares_whole_strings)
{
    ASSERT_TRUE(ace2k_str_equal("0.1.0", "0.1.0"));
    ASSERT_TRUE(ace2k_str_equal("", ""));
    ASSERT_TRUE(!ace2k_str_equal("0.1.0", "0.1.0-dirty"));
    ASSERT_TRUE(!ace2k_str_equal("0.1.0-dirty", "0.1.0"));
    ASSERT_TRUE(!ace2k_str_equal("a", "b"));
}

TEST(str_copy_truncates_and_terminates)
{
    char buf[4] = { 'x', 'x', 'x', 'x' };
    ASSERT_EQ(ace2k_str_copy(buf, sizeof buf, "abcdef"), 3);
    ASSERT_STR_EQ(buf, "abc");
    ASSERT_EQ(ace2k_str_copy(buf, sizeof buf, "q"), 1);
    ASSERT_STR_EQ(buf, "q");
}

TEST(str_copy_into_one_byte_leaves_the_empty_string)
{
    char buf[4] = { 'x', 'x', 'x', 'x' };
    ASSERT_EQ(ace2k_str_copy(buf, 1, "abc"), 0);
    ASSERT_STR_EQ(buf, "");
}

TEST(crc16_mcrf4xx_check_value_and_chaining)
{
    const uint8_t s[] = "123456789";
    ASSERT_EQ(ace2k_crc16_mcrf4xx(s, 9), 0x6F91);
    uint16_t half = ace2k_crc16_mcrf4xx_update(ACE2K_CRC16_MCRF4XX_INIT, s, 4);
    ASSERT_EQ(ace2k_crc16_mcrf4xx_update(half, s + 4, 5), 0x6F91);
    ASSERT_EQ(ace2k_crc16_mcrf4xx(s, 0), 0xFFFF);
}

TEST(crc32_check_value_and_chaining)
{
    const uint8_t s[] = "123456789";
    ASSERT_EQ(ace2k_crc32(s, 9), 0xCBF43926U);
    uint32_t half = ace2k_crc32_update(ACE2K_CRC32_INIT, s, 3);
    ASSERT_EQ(ace2k_crc32_final(ace2k_crc32_update(half, s + 3, 6)), 0xCBF43926U);
    ASSERT_EQ(ace2k_crc32(s, 0), 0);
}

TEST(u16_le_round_trips_low_byte_first)
{
    uint8_t buf[2] = { 0, 0 };
    ace2k_put_u16_le(buf, 0xBEEF);
    ASSERT_TRUE(buf[0] == 0xEF && buf[1] == 0xBE);
    ASSERT_EQ(ace2k_get_u16_le(buf), 0xBEEF);
}

TEST(u32_and_i32_le_put_low_byte_first)
{
    uint8_t buf[4] = { 0, 0, 0, 0 };
    ace2k_put_u32_le(buf, 0xDEADBEEFU);
    ASSERT_TRUE(buf[0] == 0xEF && buf[1] == 0xBE && buf[2] == 0xAD && buf[3] == 0xDE);
    ace2k_put_i32_le(buf, -2); /* 0xFFFFFFFE */
    ASSERT_TRUE(buf[0] == 0xFE && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF);
}

TEST(u32_le_reads_low_byte_first_and_round_trips)
{
    static const uint8_t stored[4] = { 0x78, 0x56, 0x34, 0x12 };
    ASSERT_EQ(ace2k_get_u32_le(stored), 0x12345678U);
    uint8_t buf[4] = { 0, 0, 0, 0 };
    ace2k_put_u32_le(buf, 0xDEADBEEFU);
    ASSERT_EQ(ace2k_get_u32_le(buf), 0xDEADBEEFU);
    static const uint8_t top_bit[4] = { 0x00, 0x00, 0x00, 0x80 }; /* no sign extension */
    ASSERT_EQ(ace2k_get_u32_le(top_bit), 0x80000000U);
    static const uint8_t blank[4] = { 0xFF, 0xFF, 0xFF, 0xFF }; /* erased flash */
    ASSERT_EQ(ace2k_get_u32_le(blank), 0xFFFFFFFFU);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
