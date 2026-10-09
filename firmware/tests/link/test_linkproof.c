#include "test.h"
#include "core/config.h"
#include "link/linkproof.h"
#include "core/util.h"

struct ace2k_fake_store {
    int calls;
    int fail;
    char version[ACE2K_CONFIG_VERSION_LEN];
};

static int fake_store_proof(void *ctx, const char *version)
{
    struct ace2k_fake_store *s = ctx;
    s->calls++;
    if (s->fail) {
        return -ACE2K_EFLASH;
    }
    ace2k_str_copy(s->version, sizeof s->version, version);
    return 0;
}

static const struct ace2k_linkproof_ops ace2k_fake_ops = { .store_proof = fake_store_proof };

/* A store during which the tick fires and finds the deadline passed: on the unit the store
 * stalls the core, and the tick that was pending through the stall runs before the store
 * returns to the caller. */
struct ace2k_fake_stalling_store {
    int calls;
    struct ace2k_linkproof *lp;
    uint32_t tick_now_ms;
};

static int fake_store_proof_stalling(void *ctx, const char *version)
{
    (void)version;
    struct ace2k_fake_stalling_store *s = ctx;
    s->calls++;
    ace2k_linkproof_tick(s->lp, s->tick_now_ms);
    return 0;
}

static const struct ace2k_linkproof_ops ace2k_fake_stalling_ops = {
    .store_proof = fake_store_proof_stalling,
};

static struct ace2k_config_record proven_by(const char *version)
{
    struct ace2k_config_record r;
    ace2k_str_copy(r.proven_version, sizeof r.proven_version, version);
    r.flags = ACE2K_CONFIG_FLAG_LINK_PROVEN;
    return r;
}

TEST(a_matching_record_is_proven_and_never_expires)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r = proven_by("0.1.0");
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 1000U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U + ACE2K_LINKPROOF_WINDOW_MS), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U + (10U * ACE2K_LINKPROOF_WINDOW_MS)),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0xFFFFFFFFU), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 1000U), 0);
    ASSERT_EQ(s.calls, 0);
}

TEST(another_image_s_proof_does_not_count)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r = proven_by("0.1.0");
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0-dirty", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
}

TEST(a_clear_flag_or_the_defaults_wait)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r = proven_by("0.1.0");
    r.flags = 0;
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
}

TEST(the_window_expires_into_recovery)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 5000U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS - 1U),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS - 1U), 1);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS), 0);
    /* Stays expired: every later tick asks again. */
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS + 10U),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

/* The binding defers the reset while a guard veto objects and asks again on every tick: that
 * holds only because the expired state never leaves ENTER_RECOVERY, however long the deferral
 * (an hour of 10 ms ticks here). */
TEST(an_expired_window_asks_for_recovery_on_every_tick_for_as_long_as_it_takes)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    uint32_t t = ACE2K_LINKPROOF_WINDOW_MS;
    ASSERT_EQ(ace2k_linkproof_tick(&lp, t), ACE2K_LINKPROOF_ENTER_RECOVERY);
    uint32_t asked = 0U;
    for (uint32_t i = 0; i < 360000U; i++) {
        t += 10U;
        asked += ace2k_linkproof_tick(&lp, t) == ACE2K_LINKPROOF_ENTER_RECOVERY ? 1U : 0U;
    }
    ASSERT_EQ(asked, 360000U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED); /* a late proof never cancels it */
}

TEST(the_window_survives_the_millisecond_wrap)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    uint32_t start = 0xFFFFFFFFU - 1000U;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", start);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0xFFFFFFFFU), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1002U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1001U),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

TEST(the_proof_is_stored_once)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(s.calls, 1);
    ASSERT_STR_EQ(s.version, "0.1.0");
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(s.calls, 1);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 2U * ACE2K_LINKPROOF_WINDOW_MS), ACE2K_LINKPROOF_RUN);
}

TEST(a_failing_store_leaves_the_image_waiting)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 1, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(s.calls, 1);
    s.fail = 0;
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
}

TEST(no_proof_after_expiry)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS), ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.calls, 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
}

TEST(a_proof_that_lands_after_the_window_expired_is_refused)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_stalling_store s = { 0, &lp, ACE2K_LINKPROOF_WINDOW_MS };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_stalling_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1U), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.calls, 1);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    /* The halt is already on its way: every later tick still asks for it. */
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 10U),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

TEST(remaining_counts_down_from_the_window)
{
    struct ace2k_linkproof lp;
    struct ace2k_fake_store s = { 0, 0, "" };
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &ace2k_fake_ops, &s, &r, "0.1.0", 100U);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 100U), ACE2K_LINKPROOF_WINDOW_MS);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 1100U), ACE2K_LINKPROOF_WINDOW_MS - 1000U);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
