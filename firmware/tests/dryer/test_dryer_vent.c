/* The vent's decision and the humidity guard, and the four recorded cycles replayed through it:
 * dryer cycles at 65, 55 and 45 °C and a 4 h PETG cycle at 65 °C, measured on the unit.  The CSVs
 * are read from tests/data/, relative to firmware/ — make runs the tests from there. */
#include <math.h>
#include <stdlib.h>

#include "dryer/dryer_vent.h"
#include "test.h"

#define CSV_DIR      "tests/data/"
#define CSV_FIELDS   32
#define CSV_LINE_MAX 512

static double magnus_ah_cg(double t_c, double rh_pct)
{
    double e = 6.112 * exp(17.67 * t_c / (t_c + 243.5));
    return 216.7 * e / (273.15 + t_c) * rh_pct; /* g/m³ × 100 at RH in % */
}

TEST(the_table_follows_the_formula_from_0_to_90_c)
{
    for (int half = 0; half < 180; half++) {
        double t = half * 0.5;
        uint32_t got = ace2k_dryer_vent_ah_cg((int32_t)(t * 1000.0), 500U);
        double want = magnus_ah_cg(t, 50.0);
        double diff = fabs((double)got - want);
        ASSERT_TRUE(diff <= (want * 0.01) + 2.0);
    }
    ASSERT_EQ(ace2k_dryer_vent_ah_cg(-5000, 500U), ace2k_dryer_vent_ah_cg(0, 500U));
    ASSERT_EQ(ace2k_dryer_vent_ah_cg(95000, 1000U), ace2k_dryer_vent_ah_cg(89999, 1000U));
}

/* An RH above 100 % reads as saturation, never more. */
TEST(an_rh_above_full_scale_is_clamped_to_saturation)
{
    uint32_t full = ace2k_dryer_vent_ah_cg(30000, ACE2K_DRYER_RH_FULL_PCT10);
    ASSERT_EQ(ace2k_dryer_vent_ah_cg(30000, 1200U), full);
    ASSERT_EQ(ace2k_dryer_vent_ah_cg(30000, UINT16_MAX), full);
    ASSERT_TRUE(ace2k_dryer_vent_ah_cg(30000, 999U) < full);
}

static struct ace2k_dryer_vent_sample sample(int32_t chamber_mc, uint16_t rh_pct10)
{
    return (struct ace2k_dryer_vent_sample){
        .chamber_mc = chamber_mc,
        .rh_pct10 = rh_pct10,
        .chamber_valid = true,
        .rh_valid = true,
    };
}

/* STARTING as the dryer does it (dryer.c): the reference picked from the reading, with no room
 * kept from an earlier cycle — the reading's own — then a new cycle on it. */
static void vent_begin(struct ace2k_dryer_vent *v, const struct ace2k_dryer_vent_sample *room,
                       uint32_t now_ms)
{
    struct ace2k_dryer_vent_room kept = { .kept = false };
    uint32_t room_cg = ace2k_dryer_vent_room_pick(&kept, ace2k_dryer_vent_sample_cg(room), now_ms);
    ace2k_dryer_vent_begin_cg(v, room_cg, now_ms);
}

/* Steps every 10 ms from `from` to `to` with a fixed reading; the first non-HOLD answer and its
 * time. */
static enum ace2k_dryer_vent_action run(struct ace2k_dryer_vent *v, int32_t target_mc,
                                        struct ace2k_dryer_vent_sample s, uint32_t from,
                                        uint32_t to, uint32_t *at)
{
    for (uint32_t t = from; t <= to; t += 10U) {
        enum ace2k_dryer_vent_action a = ace2k_dryer_vent_step(v, target_mc, &s, t);
        if (a != ACE2K_DRYER_VENT_HOLD) {
            *at = t;
            return a;
        }
    }
    *at = 0U;
    return ACE2K_DRYER_VENT_HOLD;
}

TEST(nothing_before_five_minutes_even_at_the_target)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(30000, 500U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 1000U);
    uint32_t at;
    /* the room above the target (15 °C): near at once, but not before 5 min */
    ASSERT_EQ(run(&v, 15000, sample(30000, 500U), 1000U, 300990U, &at), ACE2K_DRYER_VENT_HOLD);
    ASSERT_EQ(run(&v, 15000, sample(30000, 500U), 301000U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    ASSERT_EQ(at, 301000U);
    ASSERT_TRUE(v.vented);
}

TEST(a_chamber_still_rising_and_far_from_the_target_holds)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 500U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    for (uint32_t t = 0U; t <= 20U * 60000U; t += 10U) {
        /* rising 0.2 °C a minute: 1 °C over the 5 min window, from 25 °C toward a 65 °C target */
        struct ace2k_dryer_vent_sample s = sample(25000 + (int32_t)(t / 300U), 500U);
        ASSERT_EQ(ace2k_dryer_vent_step(&v, 65000, &s, t), ACE2K_DRYER_VENT_HOLD);
    }
}

TEST(a_chamber_that_stopped_rising_opens)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 500U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    uint32_t at;
    /* flat at 58 °C, a 65 °C target: 7 °C short, risen 0 over the window */
    ASSERT_EQ(run(&v, 65000, sample(58000, 300U), 0U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    ASSERT_EQ(at, 300000U);
}

/* One step per 10 ms from `from` to `to`, the chamber invalid on the sample at `bad_ms`; every
 * answer before `open_ms` HOLD, OPEN at `open_ms`. */
static void opens_at_despite_a_gap(int32_t target_mc, int32_t chamber_mc, uint32_t bad_ms,
                                   uint32_t open_ms)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 500U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    for (uint32_t t = 0U; t < open_ms; t += 10U) {
        struct ace2k_dryer_vent_sample s = sample(chamber_mc, 300U);
        s.chamber_valid = t != bad_ms;
        ASSERT_EQ(ace2k_dryer_vent_step(&v, target_mc, &s, t), ACE2K_DRYER_VENT_HOLD);
    }
    struct ace2k_dryer_vent_sample s = sample(chamber_mc, 300U);
    ASSERT_EQ(ace2k_dryer_vent_step(&v, target_mc, &s, open_ms), ACE2K_DRYER_VENT_OPEN);
}

/* An invalid sample restarts the "stopped rising" window: flat at 58 °C toward 65 °C, the sample
 * at 270 s invalid — the window is judged over 11 valid samples from 300 s, so it opens at 600 s,
 * not at 330 s over a window stretched across the gap. */
TEST(an_invalid_chamber_sample_restarts_the_rise_window)
{
    opens_at_despite_a_gap(65000, 58000, 270000U, 600000U);
}

/* No opening on no reading: at the target since the entry, the sample at 300 s invalid — the
 * stale reading before it decides nothing; the next valid sample opens. */
TEST(an_invalid_chamber_sample_makes_no_open_decision)
{
    opens_at_despite_a_gap(15000, 30000, 300000U, 330000U);
}

/* Vented with the room at 25 °C / 60 % (13.81 g/m³): the chamber at 55 °C / 10 % reads 10.47 —
 * drier than the room — so the guard closes after 60 s; at 55 °C / 30 % (31.41) it reopens, but
 * not before 10 min after the close. */
TEST(the_guard_closes_drier_than_the_room_and_reopens_at_most_every_ten_minutes)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 600U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    uint32_t at;
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), 0U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    uint32_t t0 = at + 10U;
    ASSERT_EQ(run(&v, 55000, sample(55000, 100U), t0, t0 + 200000U, &at), ACE2K_DRYER_VENT_CLOSE);
    ASSERT_TRUE(at - t0 >= ACE2K_DRYER_VENT_GUARD_HOLD_MS);
    ASSERT_TRUE(at - t0 <= ACE2K_DRYER_VENT_GUARD_HOLD_MS + (2U * ACE2K_DRYER_VENT_SAMPLE_MS));
    uint32_t closed = at;
    ASSERT_TRUE(!v.open);
    /* humid again at once: held until 10 min after the close */
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), closed + 10U, closed + 599990U, &at),
              ACE2K_DRYER_VENT_HOLD);
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), closed + 600000U, closed + 700000U, &at),
              ACE2K_DRYER_VENT_OPEN);
    ASSERT_TRUE(v.open);
}

/* Vented with the room's reference `dip_cg` above the chamber's reading (55 °C / 30 %): the
 * guard's answer over 30 min of that reading, and when. */
static enum ace2k_dryer_vent_action guard_on_a_dip(uint32_t room_cg_over, uint32_t *at,
                                                   uint32_t *t0)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample s = sample(55000, 300U);
    ace2k_dryer_vent_begin_cg(&v, ace2k_dryer_vent_sample_cg(&s) + room_cg_over, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    ASSERT_EQ(run(&v, 55000, s, 0U, 400000U, at), ACE2K_DRYER_VENT_OPEN);
    *t0 = *at + 10U;
    return run(&v, 55000, s, *t0, *t0 + (30U * 60000U), at);
}

/* Measured on the unit (2026-10-01): a 0.5 g/m³ dip below the room closed the flaps; under the 1 g/m³ close
 * margin it holds them open for 30 min. */
TEST(a_half_gram_dip_below_the_room_does_not_close)
{
    uint32_t at;
    uint32_t t0;
    ASSERT_EQ(guard_on_a_dip(50U, &at, &t0), ACE2K_DRYER_VENT_HOLD);
}

/* Exactly at the margin is not "more than" it. */
TEST(a_dip_of_exactly_the_margin_does_not_close)
{
    uint32_t at;
    uint32_t t0;
    ASSERT_EQ(guard_on_a_dip(ACE2K_DRYER_VENT_GUARD_CLOSE_MARGIN_CG, &at, &t0),
              ACE2K_DRYER_VENT_HOLD);
}

TEST(a_dip_of_1_1_grams_held_60_s_closes)
{
    uint32_t at;
    uint32_t t0;
    ASSERT_EQ(guard_on_a_dip(110U, &at, &t0), ACE2K_DRYER_VENT_CLOSE);
    ASSERT_TRUE(at - t0 >= ACE2K_DRYER_VENT_GUARD_HOLD_MS);
    ASSERT_TRUE(at - t0 <= ACE2K_DRYER_VENT_GUARD_HOLD_MS + (2U * ACE2K_DRYER_VENT_SAMPLE_MS));
}

/* A room's reference below the margin (0.5 g/m³) never wraps: a bone-dry chamber holds open. */
TEST(a_room_below_the_margin_never_closes)
{
    struct ace2k_dryer_vent v;
    ace2k_dryer_vent_begin_cg(&v, 50U, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    uint32_t at;
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), 0U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    ASSERT_EQ(run(&v, 55000, sample(55000, 0U), at + 10U, at + (30U * 60000U), &at),
              ACE2K_DRYER_VENT_HOLD);
    ASSERT_TRUE(v.open);
}

TEST(an_invalid_rh_or_chamber_freezes_the_guard)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 600U);
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    uint32_t at;
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), 0U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    uint32_t t1 = at; /* run() zeroes `at` when nothing happens */
    struct ace2k_dryer_vent_sample dry = sample(55000, 100U);
    /* one valid drier sample: the guard's hold starts */
    ASSERT_EQ(run(&v, 55000, dry, t1 + 10U, t1 + 30010U, &at), ACE2K_DRYER_VENT_HOLD);
    dry.rh_valid = false;
    ASSERT_EQ(run(&v, 55000, dry, t1 + 30020U, t1 + 900000U, &at), ACE2K_DRYER_VENT_HOLD);
    dry.rh_valid = true;
    dry.chamber_valid = false;
    ASSERT_EQ(run(&v, 55000, dry, t1 + 900010U, t1 + 1800000U, &at), ACE2K_DRYER_VENT_HOLD);
    /* valid again: the hold restarts — no close before 60 s of valid drier samples */
    dry.chamber_valid = true;
    uint32_t back = t1 + 1800010U;
    ASSERT_EQ(run(&v, 55000, dry, back, back + 200000U, &at), ACE2K_DRYER_VENT_CLOSE);
    ASSERT_TRUE(at - back >= ACE2K_DRYER_VENT_GUARD_HOLD_MS);
    ASSERT_TRUE(at - back <= ACE2K_DRYER_VENT_GUARD_HOLD_MS + (2U * ACE2K_DRYER_VENT_SAMPLE_MS));
}

TEST(a_room_read_invalid_at_the_start_disables_the_guard)
{
    struct ace2k_dryer_vent v;
    struct ace2k_dryer_vent_sample room = sample(25000, 600U);
    room.rh_valid = false;
    vent_begin(&v, &room, 0U);
    ace2k_dryer_vent_heating(&v, 0U);
    uint32_t at;
    ASSERT_EQ(run(&v, 55000, sample(55000, 300U), 0U, 400000U, &at), ACE2K_DRYER_VENT_OPEN);
    ASSERT_EQ(run(&v, 55000, sample(55000, 10U), at + 10U, at + 900000U, &at),
              ACE2K_DRYER_VENT_HOLD);
}

/* ---- the room's reference across cycles ---- */

#define ROOM_WET_CG 1381U /* 25 °C / 60 % */
#define ROOM_DRY_CG 230U  /* 25 °C / 10 %: a chamber still dry from the cycle before */
#define HOUR_MS     3600000U

/* The first start after a boot takes the reading, and keeps it stamped. */
TEST(the_first_start_takes_the_room_reading)
{
    struct ace2k_dryer_vent_room room = { .kept = false };
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_WET_CG, 1000U), ROOM_WET_CG);
    ASSERT_TRUE(room.kept);
    ASSERT_EQ(room.taken_ms, 1000U);
}

/* A cycle ended 3 h 59 min before: the kept reference; 5 h before: a fresh reading. */
TEST(a_start_within_four_hours_of_the_last_end_reuses_the_reference)
{
    struct ace2k_dryer_vent_room room = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&room, ROOM_WET_CG, 0U);
    ace2k_dryer_vent_room_ended(&room, 2U * HOUR_MS);
    uint32_t t = (2U * HOUR_MS) + ACE2K_DRYER_VENT_ROOM_REUSE_MS - 60000U;
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_DRY_CG, t), ROOM_WET_CG);
    ASSERT_EQ(room.taken_ms, 0U); /* not re-stamped */
    ace2k_dryer_vent_room_ended(&room, 10U * HOUR_MS);
    t = (10U * HOUR_MS) + (5U * HOUR_MS);
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_DRY_CG, t), ROOM_DRY_CG);
    ASSERT_EQ(room.cg, ROOM_DRY_CG);
    ASSERT_EQ(room.taken_ms, t);
}

/* A reference taken more than 24 h ago is taken again, even on a quick restart. */
TEST(a_reference_older_than_a_day_is_taken_again)
{
    struct ace2k_dryer_vent_room room = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&room, ROOM_WET_CG, 0U);
    uint32_t end = ACE2K_DRYER_VENT_ROOM_MAX_AGE_MS - (HOUR_MS / 2U);
    ace2k_dryer_vent_room_ended(&room, end);
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_DRY_CG, end + HOUR_MS), ROOM_DRY_CG);
}

/* The stamps are expired long before the ms clock wraps: a reference kept and an end stamped,
 * then the unit idle for 49.7 days (its dryer ticking) — a start then takes a fresh reading, the
 * old stamps never aliasing back into the 4 h and 24 h windows. */
TEST(the_room_stamps_expire_before_the_clock_wraps)
{
    struct ace2k_dryer_vent_room room = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&room, ROOM_WET_CG, 0U);
    ace2k_dryer_vent_room_ended(&room, HOUR_MS);
    for (uint32_t t = 2U * HOUR_MS; t < UINT32_MAX - HOUR_MS; t += HOUR_MS) { /* after the stamps */
        ace2k_dryer_vent_room_expire(&room, t);
    }
    ASSERT_TRUE(!room.kept);
    ASSERT_TRUE(!room.ended);
    uint32_t wrapped = HOUR_MS + 600000U; /* 10 min after the old end, modulo 2^32 */
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_DRY_CG, wrapped), ROOM_DRY_CG);
    /* within the windows, expiring changes nothing */
    struct ace2k_dryer_vent_room fresh = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&fresh, ROOM_WET_CG, 0U);
    ace2k_dryer_vent_room_ended(&fresh, HOUR_MS);
    ace2k_dryer_vent_room_expire(&fresh, 2U * HOUR_MS);
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&fresh, ROOM_DRY_CG, 2U * HOUR_MS), ROOM_WET_CG);
}

/* No cycle ended yet (a start after a boot, or a reading that was invalid): nothing to reuse. */
TEST(no_reference_is_reused_without_an_ended_cycle_or_a_known_one)
{
    struct ace2k_dryer_vent_room room = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&room, ROOM_WET_CG, 0U);
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&room, ROOM_DRY_CG, 1000U), ROOM_DRY_CG);
    struct ace2k_dryer_vent_room unknown = { .kept = false };
    (void)ace2k_dryer_vent_room_pick(&unknown, ACE2K_DRYER_VENT_AH_UNKNOWN, 0U);
    ASSERT_TRUE(!unknown.kept);
    ace2k_dryer_vent_room_ended(&unknown, 1000U);
    ASSERT_EQ(ace2k_dryer_vent_room_pick(&unknown, ROOM_DRY_CG, 2000U), ROOM_DRY_CG);
}

/* ---- the recorded cycles ---- */

/* Splits a CSV line in place, keeping empty fields (strtok would merge them). */
static int split(char *line, char **f, int max)
{
    int n = 0;
    char *p = line;
    while (n < max) {
        f[n++] = p;
        char *c = strchr(p, ',');
        if (!c) {
            break;
        }
        *c = '\0';
        p = c + 1;
    }
    char *nl = strpbrk(f[n - 1], "\r\n");
    if (nl) {
        *nl = '\0';
    }
    return n;
}

static int column(char **f, int n, const char *name)
{
    for (int i = 0; i < n; i++) {
        if (strcmp(f[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

// NOLINTNEXTLINE(readability-identifier-naming)
struct replay_out {
    bool opened, guarded;
    double open_min;
};

static struct ace2k_dryer_vent_sample row_sample(char **f, int i_c, int i_rh)
{
    double c = strtod(f[i_c], NULL);
    double rh = strtod(f[i_rh], NULL);
    return sample((int32_t)lround(c * 1000.0), (uint16_t)lround(rh * 10.0));
}

/* Records a step's answer: the first OPEN is the vent, anything else not HOLD is the guard. */
static void record(struct replay_out *out, enum ace2k_dryer_vent_action a, uint32_t now)
{
    if (a == ACE2K_DRYER_VENT_OPEN && !out->opened) {
        out->opened = true;
        out->open_min = now / 60000.0;
    } else if (a != ACE2K_DRYER_VENT_HOLD) {
        out->guarded = true;
    }
}

static void replay(const char *path, struct replay_out *out)
{
    *out = (struct replay_out){ .opened = false };
    FILE *fp = fopen(path, "r");
    ASSERT_TRUE(fp != NULL);
    if (!fp) {
        return;
    }
    char line[CSV_LINE_MAX];
    char *f[CSV_FIELDS];
    if (!fgets(line, sizeof line, fp)) {
        fclose(fp);
        return;
    }
    int n = split(line, f, CSV_FIELDS);
    int i_t = column(f, n, "t_s");
    int i_st = column(f, n, "state");
    int i_tg = column(f, n, "target_c");
    int i_c = column(f, n, "env_c");
    int i_rh = column(f, n, "env_rh");
    ASSERT_TRUE(i_t >= 0 && i_st >= 0 && i_tg >= 0 && i_c >= 0 && i_rh >= 0);
    struct ace2k_dryer_vent v;
    bool begun = false;
    bool heating = false;
    double t0 = 0.0;
    while (fgets(line, sizeof line, fp)) {
        split(line, f, CSV_FIELDS);
        const char *st = f[i_st];
        bool is_heating = strcmp(st, "heating") == 0;
        double t = strtod(f[i_t], NULL);
        struct ace2k_dryer_vent_sample s = row_sample(f, i_c, i_rh);
        if (!begun && (strcmp(st, "starting") == 0 || is_heating)) {
            vent_begin(&v, &s, 0U);
            begun = true;
        }
        if (!heating && is_heating) {
            heating = true;
            t0 = t;
            ace2k_dryer_vent_heating(&v, 0U);
        }
        if (!heating || !is_heating) {
            continue;
        }
        uint32_t now = (uint32_t)lround((t - t0) * 1000.0);
        int32_t target = (int32_t)lround(strtod(f[i_tg], NULL) * 1000.0);
        record(out, ace2k_dryer_vent_step(&v, target, &s, now), now);
    }
    fclose(fp);
}

static void assert_replay(const char *file, double open_min)
{
    struct replay_out o;
    replay(file, &o);
    ASSERT_TRUE(o.opened);
    ASSERT_TRUE(fabs(o.open_min - open_min) <= 0.5);
    ASSERT_TRUE(!o.guarded);
}

TEST(the_b4_petg_cycle_opens_at_37_min)
{
    assert_replay(CSV_DIR "dryer-b4/petg-65c-4h.csv", 37.0);
}

TEST(the_b3_65_c_cycle_opens_at_27_5_min)
{
    assert_replay(CSV_DIR "dryer-b3/65c.csv", 27.5);
}

TEST(the_b3_55_c_cycle_opens_at_16_min)
{
    assert_replay(CSV_DIR "dryer-b3/55c.csv", 16.0);
}

TEST(the_b3_45_c_cycle_opens_at_12_min)
{
    assert_replay(CSV_DIR "dryer-b3/45c.csv", 12.0);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
