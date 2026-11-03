#include "test.h"
#include "core/config.h"
#include "link/linkproof.h"
#include "core/util.h"
#include "ace2k_board/tick.h" // ACE2K_TICK_PERIOD_MS

/* Where the deadline tick lands around the final check and write, which run between hold_tick
 * and release_tick. */
enum ace2k_fake_tick_at {
    ACE2K_FAKE_TICK_NOWHERE,
    ACE2K_FAKE_TICK_BEFORE_HOLD, /* it runs just before the interrupts are masked */
    ACE2K_FAKE_TICK_IN_WINDOW,   /* it comes while they are masked and runs at the release */
};

/* The binding's config store and interrupt mask, faked.  The store sees the record as it would
 * write it.  busy refusals come first (-ACE2K_EBUSY, nothing written), then a flash failure on
 * every call while fail is set.  With tick_in_store, the tick runs inside the store at
 * tick_now_ms: on the unit the store stalls the core, and the tick that was pending through the
 * stall runs before the store returns.  tick_at places the tick around the final check. */
struct ace2k_fake_store {
    int calls;
    int busy;
    int fail;
    struct ace2k_config_record *record;
    char seen_version[ACE2K_CONFIG_VERSION_LEN]; /* the record's version at the last call */
    uint32_t seen_flags;                         /* the record's flags at the last call */
    char stored_version[ACE2K_CONFIG_VERSION_LEN];
    uint32_t stored_flags;
    int stores; /* stores that wrote */
    bool tick_in_store;
    struct ace2k_linkproof *lp;
    uint32_t tick_now_ms;
    enum ace2k_linkproof_action action; /* the tick's last answer */
    enum ace2k_fake_tick_at tick_at;
    bool link_lost;  /* the fake's ticks see the host silent */
    bool held;       /* between hold_tick and release_tick */
    bool tick_due;   /* a tick came while held */
    bool store_held; /* a store ran while held: flash work with the tick masked */
    int ticks;       /* ticks run by the fake */
    int holds;
    int releases;
};

static void fake_tick(struct ace2k_fake_store *s)
{
    bool link_ok = true;
    if (s->link_lost) {
        link_ok = false;
    }
    s->action = ace2k_linkproof_tick(s->lp, s->tick_now_ms, link_ok);
    s->ticks++;
}

static int fake_store(void *ctx)
{
    struct ace2k_fake_store *s = ctx;
    s->calls++;
    if (s->held) {
        s->store_held = true;
    }
    ace2k_str_copy(s->seen_version, sizeof s->seen_version, s->record->proven_version);
    s->seen_flags = s->record->flags;
    if (s->tick_in_store) {
        fake_tick(s);
    }
    if (s->busy > 0) {
        s->busy--;
        return -ACE2K_EBUSY;
    }
    if (s->fail) {
        return -ACE2K_EFLASH;
    }
    ace2k_str_copy(s->stored_version, sizeof s->stored_version, s->record->proven_version);
    s->stored_flags = s->record->flags;
    s->stores++;
    return 0;
}

static void fake_hold_tick(void *ctx)
{
    struct ace2k_fake_store *s = ctx;
    s->holds++;
    if (s->tick_at == ACE2K_FAKE_TICK_BEFORE_HOLD) {
        fake_tick(s);
    }
    s->held = true;
    if (s->tick_at == ACE2K_FAKE_TICK_IN_WINDOW) {
        s->tick_due = true;
    }
}

static void fake_release_tick(void *ctx)
{
    struct ace2k_fake_store *s = ctx;
    s->releases++;
    s->held = false;
    if (s->tick_due) {
        s->tick_due = false;
        fake_tick(s);
    }
}

/* The invariant of the final window: the tick never asked for the halt while the state is
 * PROVEN.  Such a halt, deferred by a veto, would never be asked again. */
static bool halt_with_proven(const struct ace2k_fake_store *s, const struct ace2k_linkproof *lp)
{
    if (s->ticks == 0 || s->action != ACE2K_LINKPROOF_ENTER_RECOVERY) {
        return false;
    }
    return ace2k_linkproof_state(lp) == ACE2K_LINKPROOF_PROVEN;
}

static const struct ace2k_linkproof_ops fake_ops = {
    .store = fake_store,
    .hold_tick = fake_hold_tick,
    .release_tick = fake_release_tick,
};

static struct ace2k_config_record proven_by(const char *version)
{
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    ace2k_str_copy(r.proven_version, sizeof r.proven_version, version);
    r.flags = ACE2K_CONFIG_FLAG_LINK_PROVEN;
    return r;
}

/* The binding's tick, then its task: the task retries the store only when the tick wants it.
 * link_ok is the binding's sample of the host link, as the tick gets it.
 * This models a tick on which the store's veto lets the task through.  The veto gate itself is
 * the binding's (linkproof_cmds.c, the tick), which the host tests do not link; lint and review
 * check it. */
static enum ace2k_linkproof_action binding_tick(struct ace2k_linkproof *lp, uint32_t now_ms,
                                                bool link_ok)
{
    enum ace2k_linkproof_action action = ace2k_linkproof_tick(lp, now_ms, link_ok);
    if (action == ACE2K_LINKPROOF_RUN && ace2k_linkproof_retry_wanted(lp)) {
        int rc = ace2k_linkproof_retry(lp);
        ASSERT_TRUE(rc == 0 || rc == -ACE2K_EBUSY);
    }
    return action;
}

/* The record holds exactly what it held before: every byte of the version, and the flags. */
static bool record_same(const struct ace2k_config_record *r,
                        const struct ace2k_config_record *before)
{
    if (memcmp(r->proven_version, before->proven_version, sizeof r->proven_version) != 0) {
        return false;
    }
    return r->flags == before->flags;
}

static void assert_record_is(const struct ace2k_config_record *r,
                             const struct ace2k_config_record *before)
{
    ASSERT_EQ(memcmp(r->proven_version, before->proven_version, sizeof r->proven_version), 0);
    ASSERT_EQ(r->flags, before->flags);
}

TEST(a_matching_record_is_proven_and_never_expires)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.1.0");
    struct ace2k_fake_store s = { .record = &r };
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 1000U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U + ACE2K_LINKPROOF_WINDOW_MS, true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 1000U + (10U * ACE2K_LINKPROOF_WINDOW_MS), true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0xFFFFFFFFU, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 1000U), 0);
    ASSERT_EQ(s.calls, 0);
}

TEST(another_image_s_proof_does_not_count)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.1.0");
    struct ace2k_fake_store s = { .record = &r };
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0-dirty", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
}

TEST(a_clear_flag_or_the_defaults_wait)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.1.0");
    struct ace2k_fake_store s = { .record = &r };
    r.flags = 0;
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
}

TEST(the_window_expires_into_recovery)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 5000U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS - 1U, true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS - 1U), 1);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS), 0);
    /* Stays expired: every later tick asks again. */
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 5000U + ACE2K_LINKPROOF_WINDOW_MS + 10U, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

/* The binding defers the reset while a guard veto objects and asks again on every tick: that
 * holds only because the expired state never leaves ENTER_RECOVERY, however long the deferral
 * (an hour of 10 ms ticks here). */
TEST(an_expired_window_asks_for_recovery_on_every_tick_for_as_long_as_it_takes)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    uint32_t t = ACE2K_LINKPROOF_WINDOW_MS;
    ASSERT_EQ(ace2k_linkproof_tick(&lp, t, true), ACE2K_LINKPROOF_ENTER_RECOVERY);
    uint32_t asked = 0U;
    for (uint32_t i = 0; i < 360000U; i++) {
        t += 10U;
        asked += ace2k_linkproof_tick(&lp, t, true) == ACE2K_LINKPROOF_ENTER_RECOVERY ? 1U : 0U;
    }
    ASSERT_EQ(asked, 360000U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED); /* a late proof never cancels it */
}

TEST(the_window_survives_the_millisecond_wrap)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    uint32_t start = 0xFFFFFFFFU - 1000U;
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", start);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0xFFFFFFFFU, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 0U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1002U, true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1001U, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

TEST(the_proof_is_stored_once)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(s.calls, 1);
    ASSERT_STR_EQ(s.stored_version, "0.1.0");
    ASSERT_EQ(s.stored_flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
    /* The record keeps what the page now holds. */
    ASSERT_STR_EQ(r.proven_version, "0.1.0");
    ASSERT_EQ(r.flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(s.calls, 1);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, 2U * ACE2K_LINKPROOF_WINDOW_MS, true), ACE2K_LINKPROOF_RUN);
}

/* The flag is set over the record's other flags, never in place of them. */
TEST(the_proof_keeps_the_records_other_flags)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.0.9");
    struct ace2k_fake_store s = { .record = &r };
    r.flags = 0x80U;
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(s.stored_flags, 0x80U | ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ASSERT_EQ(r.flags, 0x80U | ACE2K_CONFIG_FLAG_LINK_PROVEN);
}

TEST(a_failing_store_leaves_the_image_waiting_and_the_record_as_it_was)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.0.9");
    struct ace2k_fake_store s = { .record = &r, .fail = 1 };
    struct ace2k_config_record before = r;
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EFLASH);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(s.calls, 1);
    assert_record_is(&r, &before);
    /* A flash failure is not the unit's to retry: the host's next query tries again. */
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    s.fail = 0;
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
}

/* A store refused busy (the heater holds the flash) writes nothing, and the record must say so.
 * A proof left in it would reach the page with the next store of another module. */
TEST(a_store_refused_busy_leaves_the_record_as_it_was)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r = proven_by("0.0.9");
    struct ace2k_fake_store s = { .record = &r, .busy = 1 };
    r.flags = 0x80U;
    /* the bytes after the end of the version are kept too */
    r.proven_version[ACE2K_CONFIG_VERSION_LEN - 1] = 'x';
    struct ace2k_config_record before = r;
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    /* the store was asked to write the proof */
    ASSERT_EQ(s.calls, 1);
    ASSERT_STR_EQ(s.seen_version, "0.1.0");
    ASSERT_EQ(s.seen_flags, 0x80U | ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ASSERT_EQ(s.stores, 0);
    assert_record_is(&r, &before);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
}

/* The store is refused busy at every try but the one on the last tick before the deadline: the
 * unit retries by itself, the proof is stored, and the window never expires. */
TEST(a_store_refused_busy_is_retried_until_it_is_stored_before_the_deadline)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    struct ace2k_config_record before = r;
    uint32_t ticks = ACE2K_LINKPROOF_WINDOW_MS / ACE2K_TICK_PERIOD_MS;
    /* the host's try, then one try per tick up to the last tick before the deadline */
    struct ace2k_fake_store s = { .record = &r, .busy = (int)ticks - 1 };
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    uint32_t t = 0U;
    bool ran = true;
    bool kept = true; /* while refused, nothing in the record */
    for (uint32_t i = 1U; i < ticks; i++) {
        t = i * ACE2K_TICK_PERIOD_MS;
        if (binding_tick(&lp, t, true) != ACE2K_LINKPROOF_RUN) {
            ran = false;
        }
        if (ace2k_linkproof_state(&lp) == ACE2K_LINKPROOF_WAITING && !record_same(&r, &before)) {
            kept = false;
        }
    }
    ASSERT_TRUE(ran);
    ASSERT_TRUE(kept);
    ASSERT_EQ(t, ACE2K_LINKPROOF_WINDOW_MS - ACE2K_TICK_PERIOD_MS);
    ASSERT_EQ(s.calls, (int)ticks);
    ASSERT_EQ(s.stores, 1);
    ASSERT_STR_EQ(s.stored_version, "0.1.0");
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_STR_EQ(r.proven_version, "0.1.0");
    ASSERT_EQ(r.flags, ACE2K_CONFIG_FLAG_LINK_PROVEN);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    bool never_expired = true;
    for (uint32_t i = 0U; i < 100U; i++) {
        if (binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + (i * ACE2K_TICK_PERIOD_MS), true) !=
            ACE2K_LINKPROOF_RUN) {
            never_expired = false;
        }
    }
    ASSERT_TRUE(never_expired);
    ASSERT_EQ(s.calls, (int)ticks);
}

/* A store still refused at the deadline: the host did speak, so the window does not expire.  The
 * unit keeps retrying, and stores the proof once the flash is free, however late that comes. */
TEST(a_store_refused_busy_through_the_deadline_is_stored_when_the_flash_is_free)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    ace2k_config_defaults(&r);
    struct ace2k_config_record before = r;
    uint32_t ticks = (2U * ACE2K_LINKPROOF_WINDOW_MS) / ACE2K_TICK_PERIOD_MS;
    /* the host's try, then one try per tick for two windows */
    struct ace2k_fake_store s = { .record = &r, .busy = (int)ticks };
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    bool ran = true;
    bool kept = true;
    for (uint32_t i = 1U; i < ticks; i++) {
        if (binding_tick(&lp, i * ACE2K_TICK_PERIOD_MS, true) != ACE2K_LINKPROOF_RUN) {
            ran = false;
        }
        if (!record_same(&r, &before)) {
            kept = false;
        }
    }
    ASSERT_TRUE(ran);
    ASSERT_TRUE(kept);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
    /* past the deadline the window has nothing left to give */
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, ticks * ACE2K_TICK_PERIOD_MS), 0U);
    ASSERT_EQ(s.stores, 0);
    /* the flash is free on the next tick */
    ASSERT_EQ(binding_tick(&lp, ticks * ACE2K_TICK_PERIOD_MS, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(s.stores, 1);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_STR_EQ(r.proven_version, "0.1.0");
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(binding_tick(&lp, (ticks + 1U) * ACE2K_TICK_PERIOD_MS, true), ACE2K_LINKPROOF_RUN);
}

/* A retry past the deadline that fails on the flash ends the retries, and the window, already
 * past, expires on the next tick: the flash error path still ends in recovery. */
TEST(a_retry_past_the_deadline_that_fails_on_the_flash_expires)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1, .fail = 1 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 10U, true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_retry(&lp), -ACE2K_EFLASH);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 20U, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_EQ(r.flags, 0);
}

/* The hold past the deadline lasts only while the host keeps talking.  A refused proof waits past
 * the deadline with the link alive.  Then the host goes silent (disconnected, the link broken):
 * the window expires on that tick, and stays expired when the host comes back.  Before, the
 * window never expired, and an image the host had left stayed up with no recovery for as long as
 * the heater held the flash. */
TEST(a_refused_proof_past_the_deadline_expires_when_the_link_is_lost)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1000 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(binding_tick(&lp, 2U * ACE2K_LINKPROOF_WINDOW_MS, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(binding_tick(&lp, (2U * ACE2K_LINKPROOF_WINDOW_MS) + 10U, false),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    /* expired is final: the host back does not undo it, and its next query is refused */
    ASSERT_EQ(binding_tick(&lp, (2U * ACE2K_LINKPROOF_WINDOW_MS) + 20U, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    s.busy = 0;
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.stores, 0);
}

/* The link is judged at each tick past the deadline, not remembered.  A host silent before the
 * deadline and back after it holds the window as if it had never gone: the window had time left
 * while it was silent, and the host talks again when the hold matters. */
TEST(a_link_lost_before_the_deadline_and_back_still_holds_after_it)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1000 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    /* before the deadline a silent host changes nothing */
    ASSERT_EQ(binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 10U, false), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(binding_tick(&lp, 3U * ACE2K_LINKPROOF_WINDOW_MS, true), ACE2K_LINKPROOF_RUN);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
    s.busy = 0;
    ASSERT_EQ(binding_tick(&lp, (3U * ACE2K_LINKPROOF_WINDOW_MS) + 10U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    /* proven, a silent host no longer matters */
    ASSERT_EQ(binding_tick(&lp, (3U * ACE2K_LINKPROOF_WINDOW_MS) + 20U, false),
              ACE2K_LINKPROOF_RUN);
}

/* A host still silent when the deadline comes holds nothing: the window expires at the deadline,
 * as for an image nobody spoke to. */
TEST(a_refused_proof_with_the_link_lost_at_the_deadline_expires)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1000 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 10U, false), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(binding_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS, false), ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
}

/* The store of a late retry stalls the core.  A tick inside it that finds the host silent expires
 * the window, and the proof is refused: the halt and the state agree. */
TEST(a_late_retry_whose_store_sees_the_link_lost_is_refused)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .busy = 1,
        .record = &r,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS + 20U,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 10U, true),
              ACE2K_LINKPROOF_RUN);
    s.tick_in_store = true;
    s.link_lost = true;
    ASSERT_EQ(ace2k_linkproof_retry(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
}

/* A retry past the deadline that stores: a tick that runs just before the final check finds the
 * retry still pending, and leaves the window alone.  The proof stands. */
TEST(a_tick_before_the_final_check_of_a_late_retry_does_not_expire_it)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .busy = 1,
        .record = &r,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS + 20U,
        .tick_at = ACE2K_FAKE_TICK_BEFORE_HOLD,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 10U, true),
              ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_retry(&lp), 0);
    ASSERT_EQ(s.ticks, 1);
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_RUN);
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
}

/* The task only retries a proof the host asked for: an image nobody spoke to stays unproven. */
TEST(a_retry_with_no_refused_proof_stores_nothing)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_retry(&lp), -ACE2K_EINVAL);
    ASSERT_EQ(s.calls, 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

/* A retry that then fails on the flash ends the retries, as a first failure does. */
TEST(a_retry_that_fails_on_the_flash_stops_retrying)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1, .fail = 1 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_retry(&lp), -ACE2K_EFLASH);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_WAITING);
    ASSERT_EQ(r.flags, 0);
}

/* The host asks again while a refused proof waits: its query tries the store at once. */
TEST(the_hosts_next_query_also_stores_a_refused_proof)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r, .busy = 1 };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_TRUE(!ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_retry(&lp), -ACE2K_EINVAL);
    ASSERT_EQ(s.calls, 2);
}

TEST(no_proof_after_expiry)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.calls, 0);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
}

TEST(a_proof_that_lands_after_the_window_expired_is_refused)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .record = &r,
        .tick_in_store = true,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.calls, 1);
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    /* The store did write, and the record keeps what the page holds.  That is harmless: the host
     * did speak to this image. */
    ASSERT_STR_EQ(r.proven_version, "0.1.0");
    /* The halt is already on its way: every later tick still asks for it. */
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + 10U, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

/* The deadline tick comes between the final check and the write.  The interrupts are masked
 * there, so it runs after the write, finds PROVEN and asks for nothing.  Before, it ran inside
 * the window: it expired the window and asked for the halt, then the write made the state PROVEN.
 * A halt deferred by a veto was then never asked again, and the documented rule that a deadline
 * that passed wins did not hold. */
TEST(the_deadline_tick_inside_the_final_window_runs_after_the_proof)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .record = &r,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS,
        .tick_at = ACE2K_FAKE_TICK_IN_WINDOW,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), 0);
    ASSERT_EQ(s.ticks, 1); /* the tick did come */
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(s.holds, 1);
    ASSERT_EQ(s.releases, 1);
    ASSERT_TRUE(!s.store_held);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + ACE2K_TICK_PERIOD_MS, true),
              ACE2K_LINKPROOF_RUN);
}

/* The deadline tick comes just before the mask: the check then sees EXPIRED, and the proof is
 * refused.  The halt and the state agree. */
TEST(the_deadline_tick_just_before_the_final_window_refuses_the_proof)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .record = &r,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS,
        .tick_at = ACE2K_FAKE_TICK_BEFORE_HOLD,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EREFUSED);
    ASSERT_EQ(s.ticks, 1);
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_ENTER_RECOVERY);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_EXPIRED);
    ASSERT_EQ(s.holds, 1);
    ASSERT_EQ(s.releases, 1);
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS + ACE2K_TICK_PERIOD_MS, true),
              ACE2K_LINKPROOF_ENTER_RECOVERY);
}

/* The same window in a retry of a store refused busy. */
TEST(the_deadline_tick_inside_a_retrys_final_window_runs_after_the_proof)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = {
        .busy = 1,
        .record = &r,
        .lp = &lp,
        .tick_now_ms = ACE2K_LINKPROOF_WINDOW_MS,
        .tick_at = ACE2K_FAKE_TICK_IN_WINDOW,
    };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 0U);
    ASSERT_EQ(ace2k_linkproof_proven(&lp), -ACE2K_EBUSY);
    ASSERT_EQ(s.holds, 0); /* a refused store reaches no final window */
    ASSERT_EQ(ace2k_linkproof_tick(&lp, ACE2K_LINKPROOF_WINDOW_MS - 1U, true), ACE2K_LINKPROOF_RUN);
    ASSERT_TRUE(ace2k_linkproof_retry_wanted(&lp));
    ASSERT_EQ(ace2k_linkproof_retry(&lp), 0);
    ASSERT_EQ(s.ticks, 1);
    ASSERT_TRUE(!halt_with_proven(&s, &lp));
    ASSERT_EQ(s.action, ACE2K_LINKPROOF_RUN);
    ASSERT_EQ(ace2k_linkproof_state(&lp), ACE2K_LINKPROOF_PROVEN);
    ASSERT_EQ(s.releases, 1);
    ASSERT_TRUE(!s.store_held);
}

TEST(remaining_counts_down_from_the_window)
{
    struct ace2k_linkproof lp;
    struct ace2k_config_record r;
    struct ace2k_fake_store s = { .record = &r };
    ace2k_config_defaults(&r);
    ace2k_linkproof_init(&lp, &fake_ops, &s, &r, "0.1.0", 100U);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 100U), ACE2K_LINKPROOF_WINDOW_MS);
    ASSERT_EQ(ace2k_linkproof_remaining_ms(&lp, 1100U), ACE2K_LINKPROOF_WINDOW_MS - 1000U);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
