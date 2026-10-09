#include "test.h"
#include "heat/airflow.h"

/* A recording fake of the airflow board: every write is recorded with the fake clock the test
 * advances, and every flap write that drives both coils of a flap high is a violation. */
struct ace2k_fake_airflow_io {
    uint32_t now_ms;
    int fan_writes;
    bool fan_on;
    uint8_t fan_pins; /* what fan_read returns */
    int flap_writes[ACE2K_FLAP_COUNT];
    bool open_coil[ACE2K_FLAP_COUNT];
    bool close_coil[ACE2K_FLAP_COUNT];
    int open_pulses[ACE2K_FLAP_COUNT];  /* rising edges of the opening coil */
    int close_pulses[ACE2K_FLAP_COUNT]; /* rising edges of the closing coil */
    uint32_t coil_on_ms[ACE2K_FLAP_COUNT];
    uint32_t last_on_duration_ms[ACE2K_FLAP_COUNT];
    int violations;
    int overlaps; /* writes that left a coil of both flaps driven */
};

static void fake_fan_write(void *ctx, bool on)
{
    struct ace2k_fake_airflow_io *f = ctx;
    f->fan_writes++;
    f->fan_on = on;
}

static uint8_t fake_fan_read(void *ctx)
{
    const struct ace2k_fake_airflow_io *f = ctx;
    return f->fan_pins;
}

static bool fake_flap_on(const struct ace2k_fake_airflow_io *f, enum ace2k_flap flap)
{
    if (f->open_coil[flap]) {
        return true;
    }
    return f->close_coil[flap];
}

static void fake_flap_write(void *ctx, enum ace2k_flap flap, bool open_coil, bool close_coil)
{
    struct ace2k_fake_airflow_io *f = ctx;
    bool was_on = f->open_coil[flap];
    if (f->close_coil[flap]) {
        was_on = true;
    }
    bool now_on = open_coil;
    if (close_coil) {
        now_on = true;
    }
    f->flap_writes[flap]++;
    if (open_coil && close_coil) {
        f->violations++;
    }
    if (open_coil && !f->open_coil[flap]) {
        f->open_pulses[flap]++;
    }
    if (close_coil && !f->close_coil[flap]) {
        f->close_pulses[flap]++;
    }
    if (now_on && !was_on) {
        f->coil_on_ms[flap] = f->now_ms;
    }
    if (!now_on && was_on) {
        f->last_on_duration_ms[flap] = f->now_ms - f->coil_on_ms[flap];
    }
    f->open_coil[flap] = open_coil;
    f->close_coil[flap] = close_coil;
    if (fake_flap_on(f, ACE2K_FLAP_BOTTOM) && fake_flap_on(f, ACE2K_FLAP_REAR)) {
        f->overlaps++;
    }
}

static const struct ace2k_airflow_ops ace2k_fake_ops = {
    .fan_write = fake_fan_write,
    .fan_read = fake_fan_read,
    .flap_write = fake_flap_write,
};

/* Neither flap pulsing nor waiting to. */
static bool flaps_idle(const struct ace2k_airflow *a)
{
    for (unsigned i = 0; i < ACE2K_FLAP_COUNT; i++) {
        if (a->flap_run[i].set || a->flap_wait[i].set) {
            return false;
        }
    }
    return true;
}

#define COOL_MC 25000
#define WARM_MC 43000 /* between the off and on thresholds */
#define HOT_MC  46000

static void setup(struct ace2k_airflow *a, struct ace2k_fake_airflow_io *f)
{
    *f = (struct ace2k_fake_airflow_io){ 0 };
    ace2k_airflow_init(a, &ace2k_fake_ops, f);
}

/* One tick at f->now_ms with both NTCs at the given values, both valid. */
static void tick(struct ace2k_airflow *a, struct ace2k_fake_airflow_io *f, int32_t left,
                 int32_t right)
{
    ace2k_airflow_tick(a, left, right, true, true, f->now_ms);
}

/* Ticks every 10 ms from f->now_ms for ms milliseconds, NTCs cool. */
static void run_cool(struct ace2k_airflow *a, struct ace2k_fake_airflow_io *f, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 10U) {
        f->now_ms += 10U;
        tick(a, f, COOL_MC, COOL_MC);
    }
}

TEST(init_writes_fans_off_and_releases_every_coil_once)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(f.fan_writes, 1);
    ASSERT_TRUE(!f.fan_on);
    ASSERT_EQ(f.flap_writes[ACE2K_FLAP_BOTTOM], 1);
    ASSERT_EQ(f.flap_writes[ACE2K_FLAP_REAR], 1);
    ASSERT_TRUE(!f.open_coil[0] && !f.close_coil[0] && !f.open_coil[1] && !f.close_coil[1]);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(!ace2k_airflow_fans_commanded(&a));
}

TEST(the_flaps_are_unknown_at_boot)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN);
}

TEST(rule7_forces_the_fans_on_above_45_whatever_the_owner_asks)
{
    const enum ace2k_airflow_owner owners[] = {
        ACE2K_AIRFLOW_OWNER_MANUAL,
        ACE2K_AIRFLOW_OWNER_DRYER,
    };
    for (size_t i = 0; i < sizeof owners / sizeof owners[0]; i++) {
        struct ace2k_airflow a;
        struct ace2k_fake_airflow_io f;
        setup(&a, &f);
        ASSERT_EQ(ace2k_airflow_fans(&a, owners[i], true, 60U, f.now_ms), 0);
        f.now_ms += 10U;
        tick(&a, &f, HOT_MC, COOL_MC);
        ASSERT_TRUE(f.fan_on);
        /* the owner asks off: accepted as an owner change, but the fans stay on */
        ASSERT_EQ(ace2k_airflow_fans(&a, owners[i], false, 0U, f.now_ms), 0);
        ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
        ASSERT_TRUE(f.fan_on);
        f.now_ms += 10U;
        tick(&a, &f, COOL_MC, HOT_MC);
        ASSERT_TRUE(f.fan_on);
        ASSERT_TRUE(ace2k_airflow_fans_commanded(&a));
    }
    /* and with no owner at all */
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.now_ms += 10U;
    tick(&a, &f, HOT_MC, COOL_MC);
    ASSERT_TRUE(f.fan_on);
}

TEST(rule7_holds_through_release_and_the_manual_expiry)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    f.now_ms += 10U;
    tick(&a, &f, HOT_MC, HOT_MC);
    ace2k_airflow_release(&a, ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(f.fan_on);

    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 1U, f.now_ms), 0);
    for (int i = 0; i < 200; i++) { /* 2 s, past the 1 s run */
        f.now_ms += 10U;
        tick(&a, &f, HOT_MC, COOL_MC);
    }
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(f.fan_on);
}

TEST(an_invalid_ntc_counts_as_hot)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.now_ms += 10U;
    ace2k_airflow_tick(&a, COOL_MC, COOL_MC, false, true, f.now_ms);
    ASSERT_TRUE(f.fan_on);
    /* invalid never releases, even reading cool */
    for (int i = 0; i < 100; i++) {
        f.now_ms += 10U;
        ace2k_airflow_tick(&a, COOL_MC, COOL_MC, true, false, f.now_ms);
    }
    ASSERT_TRUE(f.fan_on);
    f.now_ms += 10U;
    tick(&a, &f, COOL_MC, COOL_MC);
    ASSERT_TRUE(!f.fan_on);
}

TEST(rule7_switches_on_strictly_above_45_and_off_strictly_below_42)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.now_ms += 10U;
    tick(&a, &f, ACE2K_AIRFLOW_FAN_ON_MC, ACE2K_AIRFLOW_FAN_ON_MC);
    ASSERT_TRUE(!f.fan_on);
    f.now_ms += 10U;
    tick(&a, &f, ACE2K_AIRFLOW_FAN_ON_MC + 1, COOL_MC);
    ASSERT_TRUE(f.fan_on);
    /* between the thresholds: kept on */
    f.now_ms += 10U;
    tick(&a, &f, WARM_MC, WARM_MC);
    ASSERT_TRUE(f.fan_on);
    /* at 42.000 exactly: still kept */
    f.now_ms += 10U;
    tick(&a, &f, ACE2K_AIRFLOW_FAN_OFF_MC, COOL_MC);
    ASSERT_TRUE(f.fan_on);
    /* one side still between: kept */
    f.now_ms += 10U;
    tick(&a, &f, ACE2K_AIRFLOW_FAN_OFF_MC - 1, WARM_MC);
    ASSERT_TRUE(f.fan_on);
    f.now_ms += 10U;
    tick(&a, &f, ACE2K_AIRFLOW_FAN_OFF_MC - 1, ACE2K_AIRFLOW_FAN_OFF_MC - 1);
    ASSERT_TRUE(!f.fan_on);
    /* warm again but not above 45: stays off (hysteresis) */
    f.now_ms += 10U;
    tick(&a, &f, WARM_MC, WARM_MC);
    ASSERT_TRUE(!f.fan_on);
}

TEST(fans_write_only_on_change)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    run_cool(&a, &f, 1000U);
    ASSERT_EQ(f.fan_writes, 1);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    run_cool(&a, &f, 1000U);
    ASSERT_EQ(f.fan_writes, 2);
}

TEST(fan_read_is_the_pins_as_the_board_reads_them)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.fan_pins = 0x2U;
    ASSERT_EQ(ace2k_airflow_fans_read(&a), 0x2U);
    f.fan_pins = 0x3U;
    ASSERT_EQ(ace2k_airflow_fans_read(&a), 0x3U);
}

TEST(owners_refuse_and_take_over_as_the_rules_say)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_NONE, true, 10U, f.now_ms), -ACE2K_EINVAL);
    /* the dryer takes over a manual run */
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 10U, f.now_ms), 0);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_DRYER);
    /* manual is refused while the dryer owns — fans and flaps */
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, false, 0U, f.now_ms),
              -ACE2K_EBUSY);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 10U, f.now_ms),
              -ACE2K_EBUSY);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, true, f.now_ms),
        -ACE2K_EBUSY);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_REAR], 0);
    /* another owner's release is ignored */
    ace2k_airflow_release(&a, ACE2K_AIRFLOW_OWNER_MANUAL);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_DRYER);
    /* the dryer's own flap pulse is accepted */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, true, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_COUNT, true, f.now_ms),
        -ACE2K_EINVAL);
    ace2k_airflow_release(&a, ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    run_cool(&a, &f, 20U);
    ASSERT_TRUE(!f.fan_on);
}

TEST(a_manual_run_is_bounded_and_ends_by_itself)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 0U, f.now_ms),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true,
                                 ACE2K_AIRFLOW_FAN_MANUAL_MAX_S + 1U, f.now_ms),
              -ACE2K_EINVAL);
    ASSERT_TRUE(!f.fan_on);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true,
                                 ACE2K_AIRFLOW_FAN_MANUAL_MAX_S, f.now_ms),
              0);
    ASSERT_TRUE(f.fan_on);
    run_cool(&a, &f, (ACE2K_AIRFLOW_FAN_MANUAL_MAX_S * 1000U) - 10U);
    ASSERT_TRUE(f.fan_on);
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.fan_on);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
}

TEST(heat_holds_refuses_every_way_of_switching_the_fans_off)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 1U, f.now_ms), 0);
    a.heat_holds = true;
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, false, 0U, f.now_ms),
              -ACE2K_EBUSY);
    run_cool(&a, &f, 3000U); /* past the 1 s run: the expiry waits for the lease to end */
    ASSERT_TRUE(f.fan_on);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_MANUAL);
    a.heat_holds = false;
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.fan_on);
}

/* An owner's release is never refused: the ownership goes, and heat's hold alone keeps the fans
 * commanded on (rule 2) until it clears — the fans then follow rule 7 with no owner left. */
TEST(a_release_under_heat_holds_drops_the_owner_and_keeps_the_fans_on)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    a.heat_holds = true;
    ace2k_airflow_release(&a, ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    run_cool(&a, &f, 1000U);
    ASSERT_TRUE(f.fan_on);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, false, 0U, f.now_ms),
              -ACE2K_EBUSY); /* an explicit off is still refused while heat holds */
    a.heat_holds = false;
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.fan_on);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
}

/* Rule 2's belt: fans on by rule 7 alone (no owner), heat holding a lease; the NTCs fall below
 * 42 °C and rule 7 lets go — the fans stay commanded on until heat_holds clears. */
TEST(heat_holds_keeps_the_fans_on_when_rule7_lets_go)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.now_ms += 10U;
    tick(&a, &f, HOT_MC, HOT_MC);
    ASSERT_TRUE(f.fan_on);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    a.heat_holds = true;
    run_cool(&a, &f, 1000U);
    ASSERT_TRUE(!a.thermal_on);
    ASSERT_TRUE(f.fan_on);
    ASSERT_TRUE(ace2k_airflow_fans_commanded(&a));
    a.heat_holds = false;
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.fan_on);
}

TEST(a_pulse_drives_one_coil_for_the_pulse_time_then_releases_both)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    ASSERT_TRUE(f.open_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_TRUE(!f.close_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_REAR] && !f.close_coil[ACE2K_FLAP_REAR]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN); /* travelling */
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS - 10U);
    ASSERT_TRUE(f.open_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_BOTTOM] && !f.close_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_EQ(f.last_on_duration_ms[ACE2K_FLAP_BOTTOM], ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN); /* ran to its end */
    /* the closing coil, on the other flap */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_NONE, ACE2K_FLAP_REAR, false, f.now_ms),
        0);
    ASSERT_TRUE(f.close_coil[ACE2K_FLAP_REAR] && !f.open_coil[ACE2K_FLAP_REAR]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.last_on_duration_ms[ACE2K_FLAP_REAR], ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(f.violations, 0);
}

TEST(a_repeated_pulse_drives_the_coil_again)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, false,
                                           f.now_ms),
                  0);
        run_cool(&a, &f, ACE2K_FLAP_PULSE_MS + 20U);
    }
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_REAR], 3);
    ASSERT_EQ(f.violations, 0);
}

TEST(a_pulse_during_another_is_queued_depth_one_latest_wins)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    run_cool(&a, &f, 50U);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    ASSERT_TRUE(a.flap_wait[ACE2K_FLAP_BOTTOM].set);
    /* a manual pulse is never queued: busy, on either flap */
    ASSERT_EQ(ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, false,
                                       f.now_ms),
              -ACE2K_EBUSY);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, false, f.now_ms),
        -ACE2K_EBUSY);
    /* the running pulse is not shortened by the requests */
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 1);
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_BOTTOM], 0);
    run_cool(&a, &f, 3U * ACE2K_FLAP_PULSE_MS);
    /* two pulses in all: the first, and the latest queued request (open) */
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 2);
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_BOTTOM], 0);
    ASSERT_TRUE(!a.flap_wait[ACE2K_FLAP_BOTTOM].set);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_BOTTOM] && !f.close_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(f.violations, 0);
}

TEST(a_reversal_queued_behind_a_pulse_never_drives_both_coils)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    for (int round = 0; round < 10; round++) {
        (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR,
                                       (round % 2) == 0, f.now_ms);
        (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR,
                                       (round % 2) != 0, f.now_ms);
        run_cool(&a, &f, 30U);
    }
    run_cool(&a, &f, 4U * ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.violations, 0);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_REAR] && !f.close_coil[ACE2K_FLAP_REAR]);
}

/* The two flaps never pulse together, whoever asks: the rear asked during the bottom's pulse
 * waits for it and runs right after, for its full length. */
TEST(a_pulse_on_one_flap_waits_while_the_other_flap_pulses)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, true, f.now_ms),
        0);
    ASSERT_TRUE(a.flap_wait[ACE2K_FLAP_REAR].set);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_REAR]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN); /* waiting */
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS - 10U);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_REAR]);
    run_cool(&a, &f, 10U); /* the bottom's pulse ends; the rear's starts on the same tick */
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_TRUE(f.open_coil[ACE2K_FLAP_REAR]);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);
    ASSERT_EQ(f.last_on_duration_ms[ACE2K_FLAP_REAR], ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 1);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_REAR], 1);
    ASSERT_EQ(f.overlaps, 0);
    ASSERT_EQ(f.violations, 0);
}

/* The dryer's pulses are never refused: NONE and MANUAL yield to the dryer, whether they hold the
 * outputs or not. */
TEST(a_dryer_pulse_is_never_refused_by_the_manual_or_no_owner)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false, f.now_ms),
        0);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_MANUAL, true, 10U, f.now_ms), 0);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_MANUAL);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms),
        0);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, true, f.now_ms);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    run_cool(&a, &f, 4U * ACE2K_FLAP_PULSE_MS);
    ASSERT_TRUE(flaps_idle(&a));
}

/* A weaker owner never replaces a stronger owner's request on a flap: the dryer's boot close
 * queued on the rear, a manual opening of the rear is refused, and the close runs. */
TEST(a_manual_request_never_replaces_a_dryer_request_queued_on_the_flap)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms),
        0);
    ASSERT_TRUE(a.flap_wait[ACE2K_FLAP_REAR].set);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, true, f.now_ms),
        -ACE2K_EBUSY);
    /* nor over the dryer's pulse running on the bottom */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        -ACE2K_EBUSY);
    run_cool(&a, &f, 3U * ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_REAR], 1);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_REAR], 0);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 0);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    /* once both are over, a manual request is accepted again; the dryer outranks a manual one */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, true, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms),
        0);
    run_cool(&a, &f, 3U * ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(f.overlaps, 0);
}

/* When a pulse ends with both flaps waiting, the other flap goes first: a flap asked again and
 * again cannot hold the other back. */
TEST(a_waiting_flap_goes_before_a_repeat_on_the_flap_that_just_pulsed)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true,
                                   f.now_ms);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false,
                                   f.now_ms);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_TRUE(f.close_coil[ACE2K_FLAP_REAR]);
    ASSERT_TRUE(!f.close_coil[ACE2K_FLAP_BOTTOM]);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_TRUE(f.close_coil[ACE2K_FLAP_BOTTOM]);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_BOTTOM], 1);
    ASSERT_EQ(f.close_pulses[ACE2K_FLAP_REAR], 1);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(f.overlaps, 0);
}

/* A barrage of requests on both flaps from every owner that may pulse: never both at once, never
 * both coils of one flap. */
TEST(no_mix_of_requests_ever_pulses_both_flaps_at_once)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    for (uint32_t i = 0; i < 200U; i++) {
        enum ace2k_flap flap = (i % 3U) == 0U ? ACE2K_FLAP_BOTTOM : ACE2K_FLAP_REAR;
        /* every owner in turn: NONE and MANUAL never queue, the dryer does */
        static const enum ace2k_airflow_owner owners[] = {
            ACE2K_AIRFLOW_OWNER_NONE,
            ACE2K_AIRFLOW_OWNER_MANUAL,
            ACE2K_AIRFLOW_OWNER_DRYER,
            ACE2K_AIRFLOW_OWNER_DRYER,
        };
        enum ace2k_airflow_owner who = owners[i % 4U];
        (void)ace2k_airflow_flap_pulse(&a, who, flap, (i % 5U) < 2U, f.now_ms);
        run_cool(&a, &f, 10U * (i % 7U));
    }
    run_cool(&a, &f, 4U * ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(f.overlaps, 0);
    ASSERT_EQ(f.violations, 0);
    ASSERT_TRUE(!a.flap_run[ACE2K_FLAP_BOTTOM].set && !a.flap_run[ACE2K_FLAP_REAR].set);
}

/* A flap known closed reads unknown from the moment a request for it is accepted — waiting or
 * running — to the end of that pulse, and then the pulse's own direction; pending names who asked,
 * whatever the other owners have waiting. */
TEST(a_known_flap_reads_unknown_from_the_request_to_the_end_of_its_pulse)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_EQ(ace2k_airflow_flaps_report(&a), (uint8_t)((uint32_t)ACE2K_FLAP_CLOSED << 2U));
    ASSERT_TRUE(!ace2k_airflow_flap_pending(&a, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER));
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true,
                                   f.now_ms);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, true, f.now_ms);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN); /* waiting */
    ASSERT_TRUE(ace2k_airflow_flap_pending(&a, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER));
    ASSERT_TRUE(!ace2k_airflow_flap_pending(&a, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_MANUAL));
    ASSERT_TRUE(ace2k_airflow_flap_pending(&a, ACE2K_FLAP_BOTTOM, ACE2K_AIRFLOW_OWNER_MANUAL));
    ASSERT_TRUE(!ace2k_airflow_flap_pending(&a, ACE2K_FLAP_BOTTOM, ACE2K_AIRFLOW_OWNER_DRYER));
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS); /* the bottom ends; the rear runs */
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_OPEN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_UNKNOWN); /* running */
    ASSERT_TRUE(ace2k_airflow_flap_pending(&a, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER));
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);
    ASSERT_TRUE(!ace2k_airflow_flap_pending(&a, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER));
    ASSERT_EQ(ace2k_airflow_flaps_report(&a),
              (uint8_t)((uint32_t)ACE2K_FLAP_OPEN | ((uint32_t)ACE2K_FLAP_OPEN << 2U)));
    /* a shutdown drops a request that never ran: that flap keeps the position it had */
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false,
                                   f.now_ms);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms);
    ASSERT_TRUE(a.flap_wait[ACE2K_FLAP_REAR].set);
    ace2k_airflow_shutdown(&a);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN); /* cut */
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_OPEN);      /* never ran */
    ASSERT_EQ(f.overlaps, 0);
}

/* heat's hold survives the shutdown: heat's binding recomputes it every tick, and a gate stuck
 * high is heat on — with both NTCs cool (rule 7 off) the fans are never written off, not even
 * for one tick. */
TEST(a_shutdown_keeps_heat_holds_and_the_fans_on_with_cool_ntcs)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    a.heat_holds = true;
    f.now_ms += 10U;
    tick(&a, &f, COOL_MC, COOL_MC);
    ASSERT_TRUE(f.fan_on);
    ace2k_airflow_shutdown(&a);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(a.heat_holds);
    ASSERT_TRUE(f.fan_on);
    ASSERT_TRUE(ace2k_airflow_fans_commanded(&a));
    run_cool(&a, &f, 1000U);
    ASSERT_TRUE(f.fan_on);
    a.heat_holds = false; /* heat's binding: the gate reads low again */
    run_cool(&a, &f, 10U);
    ASSERT_TRUE(!f.fan_on);
}

/* The guard's veto of the fanless bootloader: the dryer owning the fans, heat holding them, or
 * rule 7 holding them on a measured reading keep the unit out of it; rule 7 holding them only
 * for NTCs that read invalid does not (a broken sensor must not make the unit unrecoverable). */
TEST(the_fans_are_required_while_the_dryer_heat_or_a_measured_rule_7_holds_them)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    f.now_ms += 10U;
    ace2k_airflow_tick(&a, COOL_MC, COOL_MC, true, true, f.now_ms);
    ASSERT_TRUE(!ace2k_airflow_fans_required(&a));
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    ASSERT_TRUE(ace2k_airflow_fans_required(&a)); /* the dryer's cycle, any temperature */
    ace2k_airflow_release(&a, ACE2K_AIRFLOW_OWNER_DRYER);
    ASSERT_TRUE(!ace2k_airflow_fans_required(&a));
    a.heat_holds = true;
    ASSERT_TRUE(ace2k_airflow_fans_required(&a)); /* heat active */
    a.heat_holds = false;
    ace2k_airflow_tick(&a, HOT_MC, COOL_MC, true, true, f.now_ms);
    ASSERT_TRUE(ace2k_airflow_fans_required(&a)); /* above 45 °C */
    ace2k_airflow_tick(&a, 43000, COOL_MC, true, true, f.now_ms);
    ASSERT_TRUE(ace2k_airflow_fans_required(&a)); /* 43 °C, still held (hysteresis) */
    ace2k_airflow_tick(&a, 43000, 0, true, false, f.now_ms);
    ASSERT_TRUE(ace2k_airflow_fans_required(&a)); /* one NTC invalid, the other still warm */
    ace2k_airflow_tick(&a, 0, 0, false, false, f.now_ms);
    ASSERT_TRUE(a.thermal_on);                     /* rule 7 holds the fans (invalid = hot) */
    ASSERT_TRUE(!ace2k_airflow_fans_required(&a)); /* but nothing is measured warm */
    ace2k_airflow_tick(&a, 41000, 41000, true, true, f.now_ms);
    ASSERT_TRUE(!a.thermal_on);
    ASSERT_TRUE(!ace2k_airflow_fans_required(&a));
}

TEST(shutdown_releases_owners_and_coils_and_keeps_rule7)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(ace2k_airflow_fans(&a, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, f.now_ms), 0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    f.now_ms += 10U;
    tick(&a, &f, HOT_MC, COOL_MC);
    ace2k_airflow_shutdown(&a);
    ASSERT_EQ(a.owner, ACE2K_AIRFLOW_OWNER_NONE);
    ASSERT_TRUE(!a.flap_run[ACE2K_FLAP_BOTTOM].set && !a.flap_wait[ACE2K_FLAP_BOTTOM].set);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_BOTTOM] && !f.close_coil[ACE2K_FLAP_BOTTOM]);
    ASSERT_TRUE(f.fan_on); /* hot: rule 7 keeps them */
    /* the rule keeps running after the shutdown */
    run_cool(&a, &f, 20U);
    ASSERT_TRUE(!f.fan_on);
    f.now_ms += 10U;
    tick(&a, &f, COOL_MC, HOT_MC);
    ASSERT_TRUE(f.fan_on);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 0); /* the queued pulse never ran */
    ASSERT_EQ(f.violations, 0);
}

/* A pulse the shutdown cuts may leave its flap mid-travel: its position is published unknown; a
 * flap at rest keeps the last commanded one. */
TEST(a_shutdown_mid_pulse_makes_that_flaps_position_unknown)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, false,
                                   f.now_ms);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS + 20U);
    (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true,
                                   f.now_ms);
    run_cool(&a, &f, 50U);
    ace2k_airflow_shutdown(&a);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_BOTTOM), ACE2K_FLAP_UNKNOWN);
    ASSERT_EQ(ace2k_airflow_flap_pos(&a, ACE2K_FLAP_REAR), ACE2K_FLAP_CLOSED);
    ASSERT_TRUE(!f.open_coil[ACE2K_FLAP_BOTTOM] && !f.close_coil[ACE2K_FLAP_BOTTOM]);
}

/* v0.5.0 answered a manual pulse queued behind another "accepted", and the dryer could replace it
 * before it ran.  A manual pulse is now answered for what happens: busy while anything runs or
 * waits, nothing queued. */
TEST(a_manual_pulse_while_another_runs_is_busy_and_never_runs_later)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    ASSERT_TRUE(f.open_coil[ACE2K_FLAP_BOTTOM]); /* nothing running: it pulses at once */
    run_cool(&a, &f, 10U);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_REAR, true, f.now_ms),
        -ACE2K_EBUSY);
    ASSERT_TRUE(!a.flap_wait[ACE2K_FLAP_REAR].set);
    int rear_writes = f.flap_writes[ACE2K_FLAP_REAR];
    run_cool(&a, &f, 2000U);
    ASSERT_EQ(f.flap_writes[ACE2K_FLAP_REAR], rear_writes);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_REAR], 0);
    ASSERT_TRUE(flaps_idle(&a));
    /* another owner's pulse started from the queue makes it busy too */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_BOTTOM, false, f.now_ms),
        0);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, ACE2K_FLAP_REAR, false, f.now_ms),
        0);
    run_cool(&a, &f, ACE2K_FLAP_PULSE_MS); /* the bottom ends, the rear runs: nothing waits */
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        -ACE2K_EBUSY);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 1);
    run_cool(&a, &f, 2000U);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_BOTTOM], 1);
    ASSERT_EQ(f.overlaps, 0);
}

/* NONE follows the manual rule: no caller passes it today, but 0 always means "runs now" for
 * any owner but the dryer. */
TEST(an_ownerless_pulse_while_another_runs_is_busy_and_never_queued)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_NONE, ACE2K_FLAP_BOTTOM, true, f.now_ms),
        0);
    ASSERT_TRUE(f.open_coil[ACE2K_FLAP_BOTTOM]); /* nothing running: it pulses at once */
    run_cool(&a, &f, 10U);
    ASSERT_EQ(
        ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_NONE, ACE2K_FLAP_REAR, true, f.now_ms),
        -ACE2K_EBUSY);
    ASSERT_TRUE(!a.flap_wait[ACE2K_FLAP_REAR].set);
    run_cool(&a, &f, 2000U);
    ASSERT_EQ(f.open_pulses[ACE2K_FLAP_REAR], 0);
    ASSERT_TRUE(flaps_idle(&a));
}

/* Trying to make a reply lie: manual requests thrown in among the dryer's queued pulses.
 * A manual pulse answered 0 drives its coil on that very call; one answered busy drives nothing,
 * then or later — no manual request ever waits. */
TEST(no_manual_pulse_is_answered_accepted_and_left_to_run_later)
{
    struct ace2k_airflow a;
    struct ace2k_fake_airflow_io f;
    setup(&a, &f);
    int accepted = 0;
    int manual_edges = 0;
    for (uint32_t i = 0; i < 300U; i++) {
        enum ace2k_flap flap = (i % 3U) == 0U ? ACE2K_FLAP_BOTTOM : ACE2K_FLAP_REAR;
        bool open = (i % 5U) < 2U;
        if ((i % 4U) == 0U) {
            (void)ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_DRYER, flap, open, f.now_ms);
        } else {
            int before = f.open_pulses[flap] + f.close_pulses[flap];
            int rc = ace2k_airflow_flap_pulse(&a, ACE2K_AIRFLOW_OWNER_MANUAL, flap, open, f.now_ms);
            int edges = f.open_pulses[flap] + f.close_pulses[flap] - before;
            if (rc == 0) {
                accepted++;
                manual_edges += edges;
                ASSERT_EQ(edges, 1);
                ASSERT_TRUE(a.flap_run[flap].set &&
                            a.flap_run[flap].who == ACE2K_AIRFLOW_OWNER_MANUAL);
            } else {
                ASSERT_EQ(rc, -ACE2K_EBUSY);
                ASSERT_EQ(edges, 0);
            }
        }
        for (unsigned k = 0; k < ACE2K_FLAP_COUNT; k++) {
            ASSERT_TRUE(!a.flap_wait[k].set || a.flap_wait[k].who != ACE2K_AIRFLOW_OWNER_MANUAL);
        }
        run_cool(&a, &f, 10U * (i % 29U)); /* now and then past every pulse */
    }
    run_cool(&a, &f, 4U * ACE2K_FLAP_PULSE_MS);
    ASSERT_TRUE(accepted > 0);
    ASSERT_EQ(manual_edges, accepted);
    ASSERT_EQ(f.overlaps, 0);
    ASSERT_EQ(f.violations, 0);
    ASSERT_TRUE(flaps_idle(&a));
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
