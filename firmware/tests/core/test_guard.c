#include "test.h"
#include "core/guard.h"
#include "core/util.h"

TEST(flash_allowed_accepts_only_the_config_page)
{
    ASSERT_TRUE(ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR, 4U));
    ASSERT_TRUE(
        ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR, ACE2K_GUARD_CONFIG_PAGE_SIZE));
    ASSERT_TRUE(ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR + 2044U, 4U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR - 4U, 4U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR + 2046U, 4U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR + 2048U, 4U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(0x08008000U, 4U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(0x08024000U, 4U));
}

TEST(flash_allowed_rejects_bad_lengths)
{
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR, 0U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR, 4096U));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(ACE2K_GUARD_CONFIG_PAGE_ADDR, 0xFFFFFFFFU));
    ASSERT_TRUE(!ace2k_guard_flash_allowed(0xFFFFFFFCU, 8U));
}

static bool veto_yes(void *ctx)
{
    (*(int *)ctx)++;
    return true;
}

static bool veto_no(void *ctx)
{
    (*(int *)ctx)++;
    return false;
}

TEST(bootloader_allowed_with_no_veto)
{
    struct ace2k_guard g;
    ace2k_guard_init(&g);
    ASSERT_TRUE(ace2k_guard_bootloader_allowed(&g));
}

TEST(one_objecting_veto_refuses)
{
    struct ace2k_guard g;
    int calls_before = 0;
    int calls_yes = 0;
    int calls_after = 0;
    ace2k_guard_init(&g);
    ASSERT_EQ(ace2k_guard_register_veto(&g, veto_no, &calls_before), 0);
    ASSERT_TRUE(ace2k_guard_bootloader_allowed(&g));
    ASSERT_EQ(ace2k_guard_register_veto(&g, veto_yes, &calls_yes), 0);
    /* A veto registered after the objector must still be asked: no short-circuit. */
    ASSERT_EQ(ace2k_guard_register_veto(&g, veto_no, &calls_after), 0);
    ASSERT_TRUE(!ace2k_guard_bootloader_allowed(&g));
    ASSERT_EQ(calls_before, 2);
    ASSERT_EQ(calls_yes, 1);
    ASSERT_EQ(calls_after, 1);
}

TEST(register_refuses_null_and_overflow)
{
    struct ace2k_guard g;
    int ctx = 0;
    ace2k_guard_init(&g);
    ASSERT_EQ(ace2k_guard_register_veto(&g, NULL, &ctx), -ACE2K_EINVAL);
    for (int i = 0; i < ACE2K_GUARD_VETO_SLOTS; i++) {
        ASSERT_EQ(ace2k_guard_register_veto(&g, veto_no, &ctx), 0);
    }
    ASSERT_EQ(ace2k_guard_register_veto(&g, veto_no, &ctx), -ACE2K_EFULL);
    ASSERT_TRUE(ace2k_guard_bootloader_allowed(&g));
    ASSERT_EQ(ctx, ACE2K_GUARD_VETO_SLOTS);
}

TEST(init_clears_an_old_registry)
{
    struct ace2k_guard g;
    int ctx = 0;
    ace2k_guard_init(&g);
    ASSERT_EQ(ace2k_guard_register_veto(&g, veto_yes, &ctx), 0);
    ace2k_guard_init(&g);
    ASSERT_TRUE(ace2k_guard_bootloader_allowed(&g));
    ASSERT_EQ(ctx, 0);
}

TEST(the_factory_calibration_pages_are_never_writable)
{
    const uint32_t pages[] = {
        ACE2K_GUARD_FACTORY_PAGE_PRIMARY_ADDR,
        ACE2K_GUARD_FACTORY_PAGE_BACKUP_ADDR,
    };
    for (size_t i = 0; i < ACE2K_ARRAY_SIZE(pages); i++) {
        ASSERT_TRUE(!ace2k_guard_flash_allowed(pages[i], 4U));
        ASSERT_TRUE(!ace2k_guard_flash_allowed(pages[i], ACE2K_GUARD_FACTORY_PAGE_SIZE));
        ASSERT_TRUE(!ace2k_guard_flash_allowed(pages[i] + ACE2K_GUARD_FACTORY_PAGE_SIZE - 4U, 4U));
        ASSERT_TRUE(!ace2k_guard_flash_allowed(pages[i] + 0x7FCU, 2U));
    }
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
