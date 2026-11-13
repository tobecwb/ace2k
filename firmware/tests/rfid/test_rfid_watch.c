#include "test.h"
#include "rfid/watch.h"
#include "rfid/watch_internal.h"
#include "fake_field.h"

/* The world: the fake field of each reader behind the real iso14443a engine, the reader-level
 * control (configure, field) as a fake, the lanes' inputs, one clock.  Lanes 1–2 (0, 1) are
 * reader B's (index 1), lanes 3–4 reader A's. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct wworld {
    struct fake_field ff;
    struct ace2k_iso14443a iso;
    struct ace2k_rfid_watch w;
    bool field[2], stuck_on[2], configured[2];
    int configures[2];
    int offs[2];         /* field_set(…, false) calls: each try to switch the field off */
    bool lost[2];        /* every field write is lost: the field keeps its state */
    int configure_rc[2]; /* 0, or the probe's error: the soft reset never completed */
    struct ace2k_rfid_watch_inputs in;
    uint32_t now;
    int max_iso_calls, max_configures;
    /* ww_drive: each lane's own speed in um/s (+ toward the printer) while no confirmation runs on
     * it, and where the confirmation's current leg began. */
    int32_t own_um_s[ACE2K_LANE_COUNT];
    int32_t leg_from_um[ACE2K_LANE_COUNT];
    uint8_t confirm_last, reverse_last;
};

static int ww_configure(void *ctx, uint8_t r)
{
    struct wworld *ww = ctx;
    ww->configures[r]++;
    if (ww->configure_rc[r] != 0) {
        return ww->configure_rc[r];
    }
    ww->configured[r] = true;
    ww->field[r] = ww->stuck_on[r];
    return 0;
}

static bool ww_needs(void *ctx, uint8_t r)
{
    if (((struct wworld *)ctx)->configured[r]) {
        return false;
    }
    return true;
}

static void ww_field_set(void *ctx, uint8_t r, bool on)
{
    struct wworld *ww = ctx;
    if (!on) {
        ww->offs[r]++;
    }
    if (ww->lost[r]) {
        return;
    }
    ww->field[r] = on;
    if (ww->stuck_on[r]) {
        ww->field[r] = true;
    }
}

static bool ww_field_driven(void *ctx, uint8_t r)
{
    return ((struct wworld *)ctx)->field[r];
}

static const struct ace2k_rfid_watch_ops ww_ops = {
    .configure = ww_configure,
    .needs_configure = ww_needs,
    .field_set = ww_field_set,
    .field_driven = ww_field_driven,
};

static void ww_init(struct wworld *ww)
{
    memset(ww, 0, sizeof *ww);
    ace2k_iso14443a_init(&ww->iso, &fld_ops, &ww->ff);
    ace2k_rfid_watch_init(&ww->w, &ww->iso, &ww_ops, ww);
    ww->in.link_ok = true;
    ww->now = 1000;
}

/* n task runs, 10 ms apart; the largest number of iso ops calls and of configures in one run. */
static void ww_steps(struct wworld *ww, int n)
{
    for (int i = 0; i < n; i++) {
        int before = 0;
        int cfg = ww->configures[0] + ww->configures[1];
        for (int r = 0; r < 2; r++) {
            before += ww->ff.rd[r].starts + ww->ff.rd[r].polls;
        }
        ww->now += 10;
        ace2k_rfid_watch_step(&ww->w, ww->now, &ww->in);
        int after = 0;
        for (int r = 0; r < 2; r++) {
            after += ww->ff.rd[r].starts + ww->ff.rd[r].polls;
        }
        if (after - before > ww->max_iso_calls) {
            ww->max_iso_calls = after - before;
        }
        int dcfg = ww->configures[0] + ww->configures[1] - cfg;
        if (dcfg > ww->max_configures) {
            ww->max_configures = dcfg;
        }
    }
}

/* The next event of the given type, the others before it dropped; false when none. */
static bool ww_next(struct wworld *ww, uint8_t type, struct ace2k_rfid_event *e)
{
    while (ace2k_rfid_watch_peek_event(&ww->w, e)) {
        ace2k_rfid_watch_drop_event(&ww->w);
        if (e->type == type) {
            return true;
        }
    }
    return false;
}

/* Steps one task run at a time until an event of the type is out (at most max runs): a session's
 * idle timer starts at its opening, so a test that times it must stop right there. */
static bool ww_until(struct wworld *ww, uint8_t type, int max, struct ace2k_rfid_event *e)
{
    for (int i = 0; i < max; i++) {
        ww_steps(ww, 1);
        if (ww_next(ww, type, e)) {
            return true;
        }
    }
    return false;
}

static void drain(struct wworld *ww)
{
    struct ace2k_rfid_event e;
    while (ace2k_rfid_watch_peek_event(&ww->w, &e)) {
        ace2k_rfid_watch_drop_event(&ww->w);
    }
}

/* The next confirmation trace of the kind, the others before it dropped; false when none. */
static bool ww_trace(struct wworld *ww, uint8_t what, struct ace2k_rfid_confirm_trace *t)
{
    while (ace2k_rfid_watch_peek_trace(&ww->w, t)) {
        ace2k_rfid_watch_drop_trace(&ww->w);
        if (t->what == what) {
            return true;
        }
    }
    return false;
}

static const uint8_t ua[4] = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t ub[4] = { 0x11, 0x22, 0x33, 0x44 };
static const uint8_t ud[4] = { 0x21, 0x32, 0x43, 0x54 };
static const uint8_t uc[7] = { 0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
static const uint8_t key1[6] = { 1, 1, 2, 3, 4, 5 }; /* fake_field's sector-1 key A */

/* A tag in reader B's field (lanes 1–2), present or not yet. */
static struct ftag *tag_b(struct wworld *ww, const uint8_t uid[4], bool present)
{
    struct ftag *t = add_mifare(&ww->ff.rd[1], uid);
    t->absent = true;
    if (present) {
        t->absent = false;
    }
    return t;
}

/* Advances the lane's travel by um, evenly over the given task runs. */
static void ww_travel(struct wworld *ww, uint8_t lane, int32_t um, int steps)
{
    int32_t start = ww->in.travel_um[lane];
    for (int i = 1; i <= steps; i++) {
        ww->in.travel_um[lane] = start + (int32_t)(((int64_t)um * i) / steps);
        ww_steps(ww, 1);
    }
}

/* The lane moves alone, outside a load.  Tag t comes into view and leaves, and the lane travels a
 * full turn of its spool.  Tag t is left away: its return is the lane's (passive watching). */
static void turn_out(struct wworld *ww, uint8_t lane, struct ftag *t)
{
    ww->in.moving = (uint8_t)(1U << lane);
    t->absent = false;
    ww_steps(ww, 30);
    t->absent = true;
    ww_steps(ww, 40); /* the leave takes ACE2K_RFID_DEBOUNCE_POLLS inventories */
    ww_travel(ww, lane, ACE2K_RFID_PASSIVE_MIN_UM, 10);
}

/* turn_out, then tag t comes back. */
static void turn_in(struct wworld *ww, uint8_t lane, struct ftag *t)
{
    turn_out(ww, lane, t);
    t->absent = false;
}

/* A tag on a lane's spool: it is in view while the lane's travel is within [in_lo_um, in_hi_um]. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct lanetag {
    struct ftag *t;
    uint8_t lane;
    int32_t in_lo_um, in_hi_um;
};

/* The mask with the bit set or cleared. */
static uint8_t with_bit(uint8_t mask, uint8_t bit, bool on)
{
    return (uint8_t)(on ? (mask | bit) : (mask & (uint8_t)~bit));
}

/* The feed echoes the epoch of the confirmation its legs serve, with its confirm_out. */
static void ww_out_epoch(struct wworld *ww)
{
    ww->in.confirm_out_epoch = ace2k_rfid_watch_confirm_epoch_bits(&ww->w);
}

/* One lane's 10 ms of ww_drive: its speed, its travel, its moving and reverse bits, and its
 * confirm_out when a confirmation leg reached its longest. */
static void ww_drive_lane(struct wworld *ww, uint8_t lane, uint8_t conf, uint8_t rev)
{
    const struct ace2k_rfid_confirm_cfg *cfg = &ww->w.cfg;
    uint8_t bit = (uint8_t)(1U << lane);
    bool confirming = (conf & bit) != 0U;
    int32_t v = ww->own_um_s[lane];
    if (confirming) {
        bool r = (rev & bit) != 0U;
        bool was_rev = (ww->reverse_last & bit) != 0U;
        if ((ww->confirm_last & bit) == 0U || r != was_rev) {
            ww->leg_from_um[lane] = ww->in.travel_um[lane]; /* a new leg */
        }
        v = r ? -cfg->speed_um_s : cfg->speed_um_s;
    }
    ww->in.travel_um[lane] += v / 100; /* 10 ms */
    ww->in.moving = with_bit(ww->in.moving, bit, v != 0);
    ww->in.reverse = with_bit(ww->in.reverse, bit, v < 0);
    int32_t leg = ww->in.travel_um[lane] - ww->leg_from_um[lane];
    bool out = (bool)(leg >= cfg->leg_max_um || leg <= -cfg->leg_max_um);
    ww->in.confirm_out = with_bit(ww->in.confirm_out, bit, (bool)(confirming && out));
}

/* The feed, as the confirmation sees it, for the given task runs.  A lane with a confirmation
 * moves at cfg.speed_um_s in the asked direction; a leg that reaches cfg.leg_max_um raises its
 * confirm_out for one run.  Any other lane moves at its own speed.  Then each tag bound to a lane
 * is in view or not by that lane's travel.  A tag bound to no lane keeps what the test set. */
static void ww_drive(struct wworld *ww, const struct lanetag *tags, int n, int steps)
{
    for (int s = 0; s < steps; s++) {
        uint8_t conf = ace2k_rfid_watch_confirm_bits(&ww->w);
        uint8_t rev = ace2k_rfid_watch_confirm_reverse_bits(&ww->w);
        ww->in.confirm_out = 0;
        ww_out_epoch(ww); /* this feed sees each confirmation at once */
        for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
            ww_drive_lane(ww, lane, conf, rev);
        }
        ww->confirm_last = conf;
        ww->reverse_last = rev;
        for (int i = 0; i < n; i++) {
            int32_t at = ww->in.travel_um[tags[i].lane];
            tags[i].t->absent = (bool)(at < tags[i].in_lo_um || at > tags[i].in_hi_um);
        }
        ww_steps(ww, 1);
    }
}

/* ww_drive until an event of the type is out, at most max task runs. */
static bool ww_drive_until(struct wworld *ww, const struct lanetag *tags, int n, uint8_t type,
                           int max, struct ace2k_rfid_event *e)
{
    for (int i = 0; i < max; i++) {
        ww_drive(ww, tags, n, 1);
        if (ww_next(ww, type, e)) {
            return true;
        }
    }
    return false;
}

TEST(a_lane_with_a_strand_is_pending_and_nothing_turns_a_field_on_until_it_moves)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ww_steps(&ww, 20);
    ASSERT_TRUE(!ww.field[0] && !ww.field[1]);
    ASSERT_EQ(ww.configures[1], 0);
    ww.in.moving = 0x01;
    ww_steps(&ww, 2); /* configure, then the field */
    ASSERT_EQ(ww.configures[1], 1);
    ASSERT_TRUE(ww.field[1] && !ww.field[0]);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ww.in.moving = 0;
    ww_steps(&ww, 2);
    ASSERT_TRUE(!ww.field[1]);
    ASSERT_TRUE(!ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ww.max_iso_calls <= 2);
    ASSERT_TRUE(ww.max_configures <= 1);
}

/* A tag that enters during the load is only a candidate.  The lane is asked back, the tag leaves,
 * the lane is asked forward, the tag comes back: it followed, and it is the lane's. */
TEST(the_load_reads_a_tag_once_it_follows_the_lanes_reversal)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 }; /* in view over 116 mm of the lane's travel */
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    ww_drive(&ww, &lt, 1, 20); /* the field on, inventories of an empty field: the baseline */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_SEARCHING);
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, &lt, 1, 1); /* the spool turns the tag in front of the antenna */
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0x01);
    ASSERT_TRUE(!t->absent);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* it only entered: not yet the lane's */
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 500, &e));
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.tag.uid_len, 4);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_EQ(e.tag.sak, 0x08);
    uint8_t session = e.session;
    ASSERT_TRUE(session != 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_hold_bits(&ww.w), 0x01);
    ww.in.moving = 0; /* the feed pauses the lane on the hold */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x7, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 30);
    for (uint8_t b = 4; b <= 6; b++) {
        ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
        ASSERT_EQ(e.lane, 0);
        ASSERT_EQ(e.session, session);
        ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
        ASSERT_EQ(e.block, b);
        ASSERT_EQ(e.data[0], (uint8_t)(b * 16));
    }
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_READ);
    ASSERT_EQ(ace2k_rfid_watch_read_bits(&ww.w), 0x01);
    ASSERT_EQ(ace2k_rfid_watch_hold_bits(&ww.w), 0);
    ww.in.loading = 0;
    ww_steps(&ww, 3);
    ASSERT_TRUE(!ww.field[1]); /* a read lane needs no field */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ASSERT_TRUE(ww.max_iso_calls <= 2);
}

/* A new tag that only enters while lane 1 moves is nobody's.  Once it has left and come back
 * after a turn of lane 1's spool, it is lane 1's. */
TEST(the_neighbours_resting_tag_is_never_taken_and_one_that_turns_in_is)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ub, true); /* in the field from the start: lane 2's spool, at rest */
    struct ftag *mine = tag_b(&ww, ua, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01; /* lane 1 alone */
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    mine->absent = false;
    ww_steps(&ww, 30);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* it only entered */
    turn_in(&ww, 0, mine);
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* The bench, 2026-11-02: a spool sat on bay 2's holder with no strand in bay 2 yet, its tag in
 * the field.  Bay 1's strand went in.  An empty insert sensor does not mean no spool on the bay, so
 * the resting tag is nobody's.  Only the tag that comes back after a turn of lane 1's spool is
 * lane 1's. */
TEST(with_the_neighbour_empty_a_resting_tag_is_not_the_lanes_and_one_that_turns_in_is)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ub, true);                      /* bay 2's spool, at rest */
    struct ftag *mine = tag_b(&ww, ua, false); /* bay 1's spool, its tag away from the antenna */
    ww.in.insert = 0x01;                       /* lane 2 has no strand */
    ww_steps(&ww, 30);                         /* lane 1 inserted and still */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ww.in.moving = 0x01;
    ww_steps(&ww, 60); /* lane 1 moves, the resting tag stays */
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    turn_in(&ww, 0, mine); /* lane 1's spool turns its tag in, out, and in again */
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* Lane 2 reads its own tag first (moving alone, its tag back after a turn of its spool), so that
 * lane 1 can be judged with a read neighbour. */
static uint8_t read_lane2(struct wworld *ww, struct ftag *t2)
{
    turn_in(ww, 1, t2);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww->w, 1, e.session, true), 0);
    drain(ww);
    return e.session;
}

/* Lanes 1 and 2 inserted, lane 2 read and its tag turned out of view, then both still until
 * reader B's field is off: the next move starts a new baseline of an empty field. */
static void lane2_read_and_still(struct wworld *ww)
{
    struct ftag *z = tag_b(ww, ud, false);
    ww->in.insert = 0x03;
    ww_steps(ww, 1);
    (void)read_lane2(ww, z);
    z->absent = true;
    ww->in.moving = 0;
    ww_steps(ww, 5);
    ASSERT_TRUE(!ww->field[1]);
}

/* Runs task steps until the job in progress on reader B, if any, has ended, then until k more
 * inventories of reader B have started and ended. */
static void inventories_b(struct wworld *ww, int k)
{
    int i = 0;
    while (i < 200 && ace2k_iso14443a_busy(&ww->iso, ACE2K_RFID_READER_B)) {
        ww_steps(ww, 1);
        i++;
    }
    for (int done = 0; done < k; done++) {
        while (i < 200 && !ace2k_iso14443a_busy(&ww->iso, ACE2K_RFID_READER_B)) {
            ww_steps(ww, 1);
            i++;
        }
        while (i < 200 && ace2k_iso14443a_busy(&ww->iso, ACE2K_RFID_READER_B)) {
            ww_steps(ww, 1);
            i++;
        }
    }
    ASSERT_TRUE(i < 200);
}

/* Takes tag t out of reader B's field for exactly k inventories, then puts it back.  Each one
 * starts and ends with t absent. */
static void absent_for(struct wworld *ww, struct ftag *t, int k)
{
    inventories_b(ww, 0);
    t->absent = true;
    inventories_b(ww, k);
    t->absent = false;
}

/* The neighbour's resting tag drops out of a few inventories while only lane 1 moves (a tag at
 * the edge of the field).  Fewer than ACE2K_RFID_DEBOUNCE_POLLS is no leave and no entry. */
TEST(a_resting_tag_missing_from_fewer_than_the_debounce_inventories_is_not_attributed)
{
    for (int k = 1; k < (int)ACE2K_RFID_DEBOUNCE_POLLS; k++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *t = tag_b(&ww, ub, true); /* lane 2's spool at rest */
        ww.in.insert = 0x03;
        ww_steps(&ww, 1);
        ww.in.moving = 0x01;
        ww_steps(&ww, 30);
        absent_for(&ww, t, k);
        ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM; /* a whole turn, had it left */
        ww_steps(&ww, 30);
        absent_for(&ww, t, k);
        ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
        ww_steps(&ww, 30);
        struct ace2k_rfid_event e;
        ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
        ASSERT_TRUE(!ww.w.l[0].has_tag);
        ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    }
}

/* Absent from ACE2K_RFID_DEBOUNCE_POLLS inventories in a row, it left, and its return enters.  The
 * lane travelled a turn meanwhile, so the return is the lane's. */
TEST(a_tag_absent_for_the_debounce_inventories_left_and_its_return_enters)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ub, true);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    absent_for(&ww, t, (int)ACE2K_RFID_DEBOUNCE_POLLS);
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ub, 4) == 0);
}

/* A tag never seen in this field enters at its first inventory after the baseline.  With lane 2
 * read, it is lane 1's at once while lane 1 moves. */
TEST(a_tag_first_seen_after_the_baseline_enters_and_is_the_moving_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    lane2_read_and_still(&ww);
    ww.in.moving = 0x01;
    inventories_b(&ww, (int)ACE2K_RFID_BASELINE_POLLS);
    t->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 10, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* Failed inventories count toward the baseline too: a run of them does not stretch it.  A tag
 * first seen after BASELINE_POLLS failed inventories enters, and with lane 2 read it is lane
 * 1's. */
TEST(failed_inventories_count_toward_the_baseline)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    lane2_read_and_still(&ww);
    ww.in.moving = 0x01;
    ww.ff.rd[1].override_frame = ACE2K_FAKE_FRAME_WAKE;
    ww.ff.rd[1].override_poll = ACE2K_RFID_POLL_ERROR;
    inventories_b(&ww, (int)ACE2K_RFID_BASELINE_POLLS);
    ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].baseline_polls, ACE2K_RFID_BASELINE_POLLS);
    ww.ff.rd[1].override_frame = ACE2K_FAKE_FRAME_NONE;
    t->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 10, &e));
    ASSERT_EQ(e.lane, 0);
}

/* A lone spool whose tag was in front of the antenna when the field came on: the tag is in the
 * baseline.  The spool turns it away for more than ACE2K_RFID_DEBOUNCE_POLLS inventories and back,
 * after a whole turn, while only lane 1 moves: it re-enters, and is lane 1's. */
TEST(a_baseline_tag_that_leaves_and_returns_while_only_this_lane_moves_is_the_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *mine = tag_b(&ww, ua, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30); /* the baseline holds it: nothing yet */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    absent_for(&ww, mine, (int)ACE2K_RFID_DEBOUNCE_POLLS);
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
    ASSERT_TRUE(!ww.w.l[0].has_tag); /* leaving alone gives nothing */
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* The bench case.  Both spools rest in view when the field comes on for lane 1's move.  The first
 * inventory reads only lane 1's tag, and the second reads both.  Lane 2's tag was there at rest,
 * so it is not lane 1's.  When lane 2 then moves alone, lane 1's tag is not lane 2's either. */
TEST(a_resting_tag_that_the_first_inventory_missed_is_at_rest)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ua, true);
    struct ftag *theirs = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    inventories_b(&ww, 1); /* reads lane 1's tag only */
    theirs->absent = false;
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
    ww.in.moving = 0x02;
    ww_steps(&ww, 60);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag && !ww.w.l[1].has_tag);
}

/* A tag first read in the last inventory of the baseline was there at rest too. */
TEST(a_tag_first_read_in_the_last_baseline_inventory_is_at_rest)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    inventories_b(&ww, (int)ACE2K_RFID_BASELINE_POLLS - 1);
    t->absent = false;
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
}

/* The operator lifts lane 2's spool off its holder while lane 1 moves: its tag leaves the field
 * and never comes back.  It is nobody's, and lane 2 can still take it later. */
TEST(a_neighbours_spool_lifted_off_during_this_lanes_motion_is_attributed_to_nobody)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *theirs = tag_b(&ww, ub, true);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    theirs->absent = true; /* lifted off */
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
    turn_in(&ww, 1, theirs); /* back on the holder, lane 2 moves alone and turns it */
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
}

/* A baseline tag that leaves and comes back while both lanes move is nobody's.  So is one that
 * stays in view the whole time this lane moves (the bench test above). */
TEST(a_baseline_tag_that_leaves_while_the_neighbour_moves_too_is_nobodys)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, true);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x03;
    ww_steps(&ww, 30);
    t->absent = true;
    ww_steps(&ww, 40);
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM; /* a whole turn of either spool */
    ww.in.travel_um[1] += ACE2K_RFID_PASSIVE_MIN_UM;
    t->absent = false;
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
}

/* Rule 9: a search ends no_tag only when it saw no UID at all.  A UID in view that could not be
 * attributed (the neighbour's spool at rest) leaves the lane pending. */
TEST(a_search_that_saw_a_uid_it_could_not_attribute_ends_pending_not_no_tag)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ub, true); /* lane 2's spool at rest, in view the whole search */
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.in.moving = 0x01;
    ww_steps(&ww, 40);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* Lane 1's search is paused (not moving) while lane 2 moves alone and takes a tag that comes back
 * after a turn of its spool.  That tag was the neighbour's in the same inventory, so lane 1 did
 * not see a UID of its own. */
TEST(a_tag_the_neighbour_takes_in_the_same_inventory_does_not_count_as_seen)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ud, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    turn_out(&ww, 1, z);
    ww.in.loading = 0x01; /* lane 1 searching, its motor paused */
    ww_steps(&ww, 2);
    z->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(!ww.w.l[0].seen);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 1, e.session, true), 0);
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_NO_TAG);
}

/* The neighbour's own read tag in view is never this lane's, so it does not count as seen. */
TEST(a_search_that_saw_only_the_neighbours_read_tag_ends_no_tag)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ud, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    (void)read_lane2(&ww, z); /* lane 2 read, its tag stays in view */
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    ww.in.loading = 0x01;
    ww.in.moving = 0x01;
    ww_steps(&ww, 40);
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_NO_TAG);
}

TEST(with_both_moving_a_new_tag_is_the_lanes_only_when_the_neighbour_is_read)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *y = tag_b(&ww, ua, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x03;
    ww_steps(&ww, 20);
    y->absent = false;
    ww_steps(&ww, 40);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* both pending: whose? ignored */
    y->absent = true;
    struct ftag *z = tag_b(&ww, ub, false);
    (void)read_lane2(&ww, z);
    ww.in.moving = 0x03;
    ww_steps(&ww, 20);
    y->absent = false;
    ww_steps(&ww, 40);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

TEST(a_read_on_command_never_attributes_and_answers_ambiguous_or_the_unchanged_state)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x00;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_NO_FILAMENT);
    ww.in.insert = 0x01;
    ww.in.link_ok = false;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_NO_LINK);
    ww.in.link_ok = true;
    ww_steps(&ww, 1); /* the watch reads the link on its step */
    drain(&ww);
    tag_b(&ww, ub, true);
    /* lane 2 empty: its bay may still hold a spool, so the one tag is nobody's */
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ASSERT_TRUE(ace2k_rfid_watch_lane_busy(&ww.w, 0)); /* the probe pending: a MOVE=1 is busy */
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_KIND_AMBIGUOUS);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
    /* lane 2 occupied and not read: ambiguous, nothing changes */
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_KIND_AMBIGUOUS);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    /* no tag at all: the unchanged state */
    ww.ff.rd[1].t[0].absent = true;
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
}

/* A read on command waits for the whole baseline.  The first inventory sees only the read
 * neighbour's tag, the second one other tag too, and the third the neighbour's again: ambiguous,
 * not the unchanged state of the first inventory alone. */
TEST(a_read_on_command_counts_every_inventory_of_the_baseline)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    (void)read_lane2(&ww, z); /* lane 2 read: its tag stays in the field */
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    ASSERT_TRUE(!ww.field[1]);
    struct ftag *other = tag_b(&ww, ua, false);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    inventories_b(&ww, 1);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_STATE, &e)); /* one inventory does not answer */
    other->absent = false;
    inventories_b(&ww, 1);
    other->absent = true;
    inventories_b(&ww, 1);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_KIND_AMBIGUOUS);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
}

/* A read on command never attributes, even with the neighbour read and one other tag in the field:
 * a lane that has not moved stays pending. */
TEST(a_read_on_command_with_a_read_neighbour_and_one_other_tag_is_ambiguous)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    (void)read_lane2(&ww, z); /* lane 2 read: its tag stays in the field */
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    tag_b(&ww, ua, true);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 50); /* the baseline: three inventories of two tags */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_KIND_AMBIGUOUS);
    drain(&ww);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
    ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].session_open, false);
}

static uint8_t open_session_on_lane1(struct wworld *ww, struct ftag **t)
{
    *t = tag_b(ww, ua, false);
    ww->in.insert = 0x01;
    ww_steps(ww, 1);
    turn_in(ww, 0, *t);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    drain(ww);
    return e.session;
}

TEST(a_read_on_command_on_a_reading_lane_is_busy_and_the_session_holds_the_tag)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_BUSY); /* reading */
    ASSERT_TRUE(ace2k_rfid_watch_lane_busy(&ww.w, 0));                 /* the session open */
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 1));
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 4));
    ASSERT_TRUE(ace2k_rfid_watch_tag(&ww.w, 0)->uid_len == 4); /* the tag the session holds */
    ASSERT_TRUE(ace2k_rfid_watch_tag(&ww.w, 4) == NULL);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, false), 0);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0)); /* closed: a MOVE=1 may start */
}

/* Lane 1's tag was attributed by its motion and its session ended without a read: the lane keeps
 * the UID.  A read on command that sees it reopens its session, as an inventory would. */
TEST(a_read_on_command_that_sees_the_lanes_own_kept_uid_reopens_its_session)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, false), 0);
    ww_steps(&ww, 5);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
}

/* The reader's table full of UIDs inside their debounce window: none is dropped, and a new UID
 * is not tracked, so it does not enter in that inventory.  Once the others are gone, it does, and
 * with lane 2 read it is lane 1's. */
TEST(a_new_uid_with_the_table_full_does_not_enter_until_there_is_room)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    lane2_read_and_still(&ww);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30); /* the baseline: an empty field */
    for (int i = 0; i < 20 && ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B); i++) {
        ww_steps(&ww, 1);
    }
    struct ace2k_rfid_watch_reader *rd = &ww.w.r[ACE2K_RFID_READER_B];
    for (uint8_t j = 0; j < ACE2K_RFID_TRACKED_MAX; j++) {
        rd->tracked[j] = (struct ace2k_iso14443a_tag){ .uid_len = 4, .uid = { 0xA0, j, 0, 0 } };
        rd->absent[j] = 1; /* away, inside the window */
    }
    rd->tracked_count = ACE2K_RFID_TRACKED_MAX;
    t->absent = false;
    uint32_t last = rd->t_last_inventory_ms;
    for (int i = 0; i < 20 && rd->t_last_inventory_ms == last; i++) {
        ww_steps(&ww, 1);
    }
    for (int i = 0; i < 20 && ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B); i++) {
        ww_steps(&ww, 1);
    }
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* one inventory: not tracked, not entered */
    ASSERT_EQ(rd->tracked_count, ACE2K_RFID_TRACKED_MAX);
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e)); /* the others left: now it enters */
    ASSERT_EQ(e.lane, 0);
}

TEST(a_session_with_no_host_step_ends_pending_after_half_a_second)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    ww_steps(&ww, 45);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ww_steps(&ww, 10);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              -ACE2K_EREFUSED); /* a stale host: ignored */
}

TEST(a_tag_gone_during_a_step_ends_the_session_lost_and_its_return_reopens_it)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    t->absent = true; /* the lane coasted past it */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_TAG_GONE);
    ASSERT_EQ(e.block, 4);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0x01);
    ww.in.moving = 0x01; /* the reacquire */
    t->absent = false;
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* its own known UID, whatever moved */
    ASSERT_TRUE(e.session != session);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0);
}

TEST(the_load_ends_no_tag_only_when_its_search_ran_out_and_no_uid_was_seen)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_NO_TAG);
    /* a new insertion: a UID seen and given up, then the search ran out: pending (rule 9) */
    ww.in.insert = 0;
    ww.in.exhausted = 0;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_UNKNOWN);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 1000, &e)); /* it followed */
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, e.session, false), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING); /* back to the search */
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

TEST(a_field_that_stays_on_latches_the_fault_and_the_ceiling_switches_it_off)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 5);
    ww.stuck_on[1] = true;
    ww.in.moving = 0;
    ww_steps(&ww, 2);
    ASSERT_TRUE(ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B));
    /* the ceiling: a lane in its load that does not move for two minutes */
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww_steps(&ww, 5);
    ASSERT_TRUE(ww.field[1]);
    ww_steps(&ww, (int)(ACE2K_RFID_FIELD_MAX_MS / 10U));
    ASSERT_TRUE(!ww.field[1]);
    ww_steps(&ww, 100);
    ASSERT_TRUE(!ww.field[1]); /* stays off … */
    ww.in.moving = 0x01;
    ww_steps(&ww, 3);
    ASSERT_TRUE(ww.field[1]); /* … until the lane moves */
}

/* A switch-off that is lost (the read-back still shows a driver on) is tried again at every step,
 * and the field is reported on until it reads back off.  The fault latches and stays. */
TEST(a_switch_off_that_reads_back_on_is_tried_again_and_reported_on_until_it_reads_off)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 5);
    ASSERT_TRUE(ww.field[1]);
    int offs = ww.offs[1];
    ww.stuck_on[1] = true; /* the next switch-offs are lost */
    ww.in.moving = 0;
    ww_steps(&ww, 1);
    ASSERT_EQ(ww.offs[1], offs + 1);
    ASSERT_TRUE(ww.field[1]);
    ASSERT_TRUE(ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(
        ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B)); /* the read-back, not the command */
    /* no read runs, so the reader is free for a probe: its soft reset switches the field off */
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_B));
    ww_steps(&ww, 3);
    ASSERT_EQ(ww.offs[1], offs + 4); /* tried again at every step */
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ww.stuck_on[1] = false; /* the next one goes through */
    ww_steps(&ww, 1);
    ASSERT_EQ(ww.offs[1], offs + 5);
    ASSERT_TRUE(!ww.field[1]);
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B)); /* latched */
    ww_steps(&ww, 10);
    ASSERT_EQ(ww.offs[1], offs + 5); /* off: nothing more to switch off */
    ASSERT_TRUE(ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B));
}

/* A probe's soft reset switches the field off even when the watch's own writes are lost: the
 * next step reads it back off and the switch-off is no longer pending.  The next switch-on
 * configures the reader again. */
TEST(a_probes_soft_reset_clears_a_pending_switch_off)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 5);
    ww.lost[1] = true; /* every write to the field is lost from now on */
    ww.in.moving = 0;
    ww_steps(&ww, 3);
    ASSERT_TRUE(ww.field[1]);
    ASSERT_TRUE(ww.w.r[ACE2K_RFID_READER_B].off_pending);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_B)); /* a probe may run */
    ww.configured[1] = false; /* the probe: a soft reset, both drivers off */
    ww.field[1] = false;
    int offs = ww.offs[1];
    ww_steps(&ww, 1);
    ASSERT_EQ(ww.offs[1], offs + 1); /* the switch-off once more, its write lost again */
    ASSERT_TRUE(!ww.w.r[ACE2K_RFID_READER_B].off_pending);
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B)); /* latched */
    ww_steps(&ww, 10);
    ASSERT_EQ(ww.offs[1], offs + 1); /* off: nothing more to switch off */
    int configures = ww.configures[1];
    ww.lost[1] = false;
    ww.in.moving = 0x01;
    ww_steps(&ww, 2); /* configure, then the field */
    ASSERT_EQ(ww.configures[1], configures + 1);
    ASSERT_TRUE(ww.field[1]);
}

/* A lane that moves again while a switch-off is still lost turns the field on as usual: the
 * reader is configured if it needs it, and the first inventory is a new baseline. */
TEST(a_lane_moving_while_a_switch_off_is_lost_turns_the_field_on_as_usual)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 10);
    ww.stuck_on[1] = true;
    ww.in.moving = 0;
    ww_steps(&ww, 2);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    t->absent = false; /* a tag arrives at rest, while the field should be off */
    ww_steps(&ww, 5);
    ww.stuck_on[1] = false;
    ww.in.moving = 0x01; /* lane 1 alone */
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww.field[1]);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* at rest: in the baseline, not lane 1's */
}

/* A soft reset of a reader under its field (a probe from another path) clears the reader and its
 * antenna drivers.  The next step drops the field, the one after configures, and no job runs on
 * the reader until it is configured again. */
TEST(a_reader_reset_under_its_field_drops_the_field_and_runs_no_job_until_configured)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 20);
    for (int i = 0; i < 20 && ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B); i++) {
        ww_steps(&ww, 1);
    }
    ASSERT_TRUE(!ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    int configures = ww.configures[1];
    int starts = ww.ff.rd[1].starts;
    ww.configured[1] = false; /* the soft reset: the settings and the field are gone */
    ww.field[1] = false;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B)); /* dropped */
    ASSERT_TRUE(!ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_B)); /* it reads back off */
    bool ran_unconfigured = false;
    for (int i = 0; i < 20; i++) {
        bool was_configured = ww.configured[1];
        int before = ww.ff.rd[1].starts;
        ww_steps(&ww, 1);
        if (!was_configured && ww.ff.rd[1].starts != before) {
            ran_unconfigured = true;
        }
    }
    ASSERT_TRUE(!ran_unconfigured);
    ASSERT_EQ(ww.configures[1], configures + 1);
    ASSERT_TRUE(ww.field[1]);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ww.ff.rd[1].starts > starts); /* the inventories go on, on a configured reader */
}

/* A soft reset that lands in the middle of an inventory: the reader lost its settings, so the
 * inventory's result is dropped.  No tag is taken from it, and the reader is configured again
 * before its next job. */
TEST(an_inventory_that_straddles_a_reset_is_dropped)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    turn_out(&ww, 0, t); /* the tag's return would be the lane's */
    drain(&ww);
    for (int i = 0; i < 20 && !ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B); i++) {
        ww_steps(&ww, 1);
    }
    ASSERT_TRUE(ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B)); /* an inventory runs */
    t->absent = false;
    ww.configured[1] = false; /* the soft reset, in the middle of it */
    ww.field[1] = false;
    int configures = ww.configures[1];
    struct ace2k_rfid_event e;
    bool tag_unconfigured = false;
    for (int i = 0; i < 20 && ww.configures[1] == configures; i++) {
        ww_steps(&ww, 1);
        if (ww_next(&ww, ACE2K_RFID_EV_TAG, &e)) {
            tag_unconfigured = true;
        }
    }
    ASSERT_TRUE(!tag_unconfigured);
    ASSERT_EQ(ww.configures[1], configures + 1);
    /* The field came back with the tag in it, so the tag rests in the new baseline.  The spool
     * turns it out and in again: then a configured reader reads it. */
    ww_steps(&ww, 5);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    t->absent = true;
    ww_steps(&ww, 30); /* away for more than ACE2K_RFID_DEBOUNCE_POLLS inventories */
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
    t->absent = false;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
}

/* A soft reset that lands in the middle of a host step: the step ends as failed, with no data,
 * and the session stays open for the host's next step. */
TEST(a_step_that_straddles_a_reset_ends_failed_with_no_data)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x7, key1,
                                            ww.now),
              0);
    for (int i = 0; i < 20 && !ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B); i++) {
        ww_steps(&ww, 1);
    }
    ASSERT_TRUE(ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B)); /* the step runs */
    ww.configured[1] = false; /* the soft reset, in the middle of it */
    ww.field[1] = false;
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.session, session);
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_ERROR);
    ASSERT_EQ(e.block, 4); /* the step's first block */
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
}

TEST(the_insert_falling_ends_the_session_and_the_link_down_opens_none)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.insert = 0;
    ww_steps(&ww, 1);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_UNKNOWN);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), -ACE2K_EREFUSED);
    ww_init(&ww);
    session = open_session_on_lane1(&ww, &t);
    ww.in.link_ok = false;
    ww_steps(&ww, 1);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    t->absent = true;
    ww_steps(&ww, 10);
    t->absent = false;
    ww_steps(&ww, 30);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
}

TEST(the_watch_holds_no_key_once_the_step_has_begun)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 1);
    const uint8_t *k = ww.w.r[ACE2K_RFID_READER_B].step_job.key;
    ASSERT_TRUE(k[0] == 0 && k[1] == 0 && k[2] == 0 && k[3] == 0 && k[4] == 0 && k[5] == 0);
    /* a step out of bounds is a host bug */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, 3, 1, 1, key1, ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 16, 1, key1,
                                            ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(
        ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_NTAG, 4, 4, key1, ww.now),
        -ACE2K_EINVAL);
}

TEST(one_session_reads_two_mifare_sectors_in_two_steps)
{
    /* each read job ends with HLTA, so the second step's WUPA wakes the tag again: both OK, the
     * session still open, nothing lost (the regression of a job that left its tag active) */
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    static const uint8_t key2[6] = { 2, 1, 2, 3, 4, 5 }; /* fake_field's sector-2 key A */
    struct ace2k_rfid_event e;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.block, 4);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 2, 0x1, key2,
                                            ww.now),
              0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.block, 8);
    ASSERT_EQ(e.data[0], 128); /* block 8, byte 0 */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
}

TEST(a_reader_that_never_answers_costs_one_start_and_one_poll_per_run_with_a_lane_moving)
{
    /* A reader that never answers does not delay the lane's speed loop: the lane's speed loop runs
     * in the 10 ms tick, the timer interrupt, and the watch runs in the rfid task, so a watch step
     * cannot delay it by construction; what the task can do is loop or wait on a mute reader — this
     * shows it does neither: every run returns, and each reader sees at most one start and one poll
     * per run, whatever the number of runs. */
    struct wworld ww;
    ww_init(&ww);
    ww.ff.rd[ACE2K_RFID_READER_B].mute = true;
    tag_b(&ww, ua, true); /* a tag there, never heard */
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww.in.loading = 0x01;
    for (int i = 0; i < 500; i++) {
        int starts[2];
        int polls[2];
        for (int r = 0; r < 2; r++) {
            starts[r] = ww.ff.rd[r].starts;
            polls[r] = ww.ff.rd[r].polls;
        }
        ww_steps(&ww, 1); /* returns: the step never loops on the mute reader */
        for (int r = 0; r < 2; r++) {
            ASSERT_TRUE(ww.ff.rd[r].starts - starts[r] <= 1);
            ASSERT_TRUE(ww.ff.rd[r].polls - polls[r] <= 1);
        }
    }
    ASSERT_TRUE(ww.ff.rd[ACE2K_RFID_READER_B].starts > 1); /* jobs ended in error, retried */
    ASSERT_TRUE(ww.max_configures <= 1);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
}

TEST(an_ntag_session_reads_twelve_pages_in_one_step)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = add_ntag(&ww.ff.rd[1], uc);
    t->absent = true;
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    turn_in(&ww, 0, t);
    ww_steps(&ww, 40);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.tag.uid_len, 7);
    ASSERT_EQ(
        ace2k_rfid_watch_step_request(&ww.w, 0, e.session, ACE2K_RFID_OP_NTAG, 4, 3, 0, ww.now), 0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.block, 4);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.block, 8);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.block, 12);
    ASSERT_EQ(e.data[0], 48);
}

TEST(a_wrong_key_is_auth_failed_the_session_stays_open_and_the_next_key_reads)
{
    /* the fake's tag stays silent on a wrong key, as a real one does: not a lost tag */
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    static const uint8_t wrong[6] = { 9, 9, 9, 9, 9, 9 };
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1,
                                            wrong, ww.now),
              0);
    ww_steps(&ww, 20);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_AUTH_FAILED);
    ASSERT_EQ(e.block, 4);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.session, session);
}

TEST(a_malformed_inventory_takes_no_tag_and_the_tag_is_taken_once_it_answers_well)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    turn_out(&ww, 0, t); /* the tag's return will be the lane's */
    ww.ff.rd[1].hostile = ACE2K_FAKE_HOSTILE_BCC;
    t->absent = false;
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww.field[1]); /* still looking while the lane moves */
    ww.ff.rd[1].hostile = ACE2K_FAKE_HOSTILE_NONE;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_TRUE(ww.max_iso_calls <= 2);
}

TEST(a_malformed_read_answer_is_an_error_step_and_the_session_stays_open)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.moving = 0;
    ww.ff.rd[1].hostile = ACE2K_FAKE_HOSTILE_READ_LAST_BITS;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 20);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_ERROR); /* no bytes of the malformed answer */
    ASSERT_EQ(e.block, 4);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0);
    ww.ff.rd[1].hostile = ACE2K_FAKE_HOSTILE_NONE;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.data[0], 64); /* block 4 */
}

TEST(a_session_closed_under_a_running_step_opens_the_neighbours_only_after_it_and_no_bytes_cross)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    tag_b(&ww, ub, true);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x7, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 1); /* the step's job begins */
    ASSERT_TRUE(ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B));
    /* lane 2's UID was attributed while lane 1's session held the reader */
    ww.w.l[1].want = true;
    ww.w.l[1].wanted = (struct ace2k_iso14443a_tag){
        .uid = { 0x11, 0x22, 0x33, 0x44 },
        .uid_len = 4,
        .atqa = 0x0004,
        .sak = 0x08,
    };
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ASSERT_TRUE(!ww.w.r[ACE2K_RFID_READER_B].session_open); /* the job runs: the want waits */
    struct ace2k_rfid_event e;
    bool opened = false;
    for (int i = 0; i < 40 && !opened; i++) {
        ww_steps(&ww, 1);
        while (ace2k_rfid_watch_peek_event(&ww.w, &e)) {
            ace2k_rfid_watch_drop_event(&ww.w);
            ASSERT_TRUE(e.type != ACE2K_RFID_EV_DATA); /* lane 1's bytes go nowhere */
            if (e.type == ACE2K_RFID_EV_TAG) {
                ASSERT_EQ(e.lane, 1);
                ASSERT_TRUE(!ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B) ||
                            ww.w.r[ACE2K_RFID_READER_B].session_open);
                opened = true;
            }
        }
    }
    ASSERT_TRUE(opened);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0);
    ww_steps(&ww, 40); /* longer than the old step's job, shorter than the session's idle end */
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_READING);
}

TEST(a_failing_configure_backs_off_gives_up_answers_reads_at_once_and_the_health_run_revives_it)
{
    struct wworld ww;
    ww_init(&ww);
    ww.configure_rc[ACE2K_RFID_READER_B] = -ACE2K_EIO;
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 1);
    ASSERT_EQ(ww.configures[1], 1);
    ww_steps(&ww, (int)(ACE2K_RFID_CONFIGURE_RETRY_MS / 10U) - 1);
    ASSERT_EQ(ww.configures[1], 1); /* no retry before the backoff */
    ww_steps(&ww, 1);
    ASSERT_EQ(ww.configures[1], 2);
    ASSERT_TRUE(!ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_B));
    ww_steps(&ww, (int)(ACE2K_RFID_CONFIGURE_RETRY_MS / 10U));
    ASSERT_EQ(ww.configures[1], (int)ACE2K_RFID_CONFIGURE_TRIES);
    ASSERT_TRUE(ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_A));
    ww_steps(&ww, 500);
    ASSERT_EQ(ww.configures[1], (int)ACE2K_RFID_CONFIGURE_TRIES); /* given up */
    ASSERT_TRUE(!ww.field[1]);
    /* a read on command on a dead reader: accepted and answered at once, the state unchanged */
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    /* the health run: tried again, and it works now */
    ww.configure_rc[ACE2K_RFID_READER_B] = 0;
    ace2k_rfid_watch_clear_dead(&ww.w, ACE2K_RFID_READER_B);
    ww_steps(&ww, 2);
    ASSERT_EQ(ww.configures[1], (int)ACE2K_RFID_CONFIGURE_TRIES + 1);
    ASSERT_TRUE(ww.field[1]);
    ASSERT_TRUE(!ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ww.max_configures <= 1);
}

TEST(a_read_on_command_no_inventory_decides_is_answered_at_probe_max_and_the_field_goes_off)
{
    struct wworld ww;
    ww_init(&ww);
    ww.ff.rd[ACE2K_RFID_READER_B].mute = true; /* every inventory ends in error */
    tag_b(&ww, ua, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    struct ace2k_rfid_event e;
    ww_steps(&ww, (int)(ACE2K_RFID_PROBE_MAX_MS / 10U) - 1);
    ASSERT_TRUE(ww.field[1]);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ww_steps(&ww, 1);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ww_steps(&ww, (int)ACE2K_ISO_POLL_MAX + 2); /* the running inventory gives up */
    ASSERT_TRUE(!ww.field[1]);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK); /* not busy any more */
}

TEST(a_read_on_command_after_the_ceiling_brings_the_field_back_answers_and_lets_it_go)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01; /* in its load, not moving: the ceiling switches the field off */
    ww_steps(&ww, 5);
    ww_steps(&ww, (int)(ACE2K_RFID_FIELD_MAX_MS / 10U));
    ASSERT_TRUE(!ww.field[1]);
    ww.in.loading = 0;
    ww_steps(&ww, 2);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    bool on = false;
    bool answered = false;
    struct ace2k_rfid_event e;
    for (int i = 0; i < (int)(ACE2K_RFID_PROBE_MAX_MS / 10U) && !answered; i++) {
        ww_steps(&ww, 1);
        if (ww.field[1]) {
            on = true;
        }
        answered = ww_next(&ww, ACE2K_RFID_EV_STATE, &e);
    }
    ASSERT_TRUE(on);
    ASSERT_TRUE(answered);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING); /* no tag in the field */
    ww_steps(&ww, 3);
    ASSERT_TRUE(!ww.field[1]);
}

/* A scripted search start: counts the calls and answers what it is told. */
struct ace2k_fake_search {
    int calls;
    uint8_t lane;
    enum ace2k_rfid_read_refusal answer;
};

static enum ace2k_rfid_read_refusal fake_search_start(void *ctx, uint8_t lane)
{
    struct ace2k_fake_search *f = ctx;
    f->calls++;
    f->lane = lane;
    return f->answer;
}

TEST(a_read_with_motion_asks_the_search_only_on_a_free_lane_and_forgets_only_after_it)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    tag_b(&ww, ub, true); /* nobody's: a read on command attributes nothing */
    struct ace2k_fake_search f = { .answer = ACE2K_RFID_READ_OK };
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    /* the probe pending: busy, and the search is not asked */
    ASSERT_EQ(ace2k_rfid_watch_read_with_motion(&ww.w, 0, fake_search_start, &f),
              ACE2K_RFID_READ_BUSY);
    ww_steps(&ww, 30);
    drain(&ww);
    ww.ff.rd[1].t[0].absent = true;
    /* lane 1 moves and its own tag comes back after a turn: a session */
    struct ftag *mine = tag_b(&ww, ua, false);
    turn_in(&ww, 0, mine);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ww.in.moving = 0;
    /* the session open: busy too */
    ASSERT_EQ(ace2k_rfid_watch_read_with_motion(&ww.w, 0, fake_search_start, &f),
              ACE2K_RFID_READ_BUSY);
    ASSERT_EQ(f.calls, 0);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, e.session, true), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    /* a refusal: asked once, and the lane keeps its tag */
    f.answer = ACE2K_RFID_READ_NO_FILAMENT;
    ASSERT_EQ(ace2k_rfid_watch_read_with_motion(&ww.w, 0, fake_search_start, &f),
              ACE2K_RFID_READ_NO_FILAMENT);
    ASSERT_EQ(f.calls, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    /* started: the lane forgets its tag and waits for the next one */
    f.answer = ACE2K_RFID_READ_OK;
    ASSERT_EQ(ace2k_rfid_watch_read_with_motion(&ww.w, 0, fake_search_start, &f),
              ACE2K_RFID_READ_OK);
    ASSERT_EQ(f.calls, 2);
    ASSERT_EQ(f.lane, 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* The ring keeps the oldest events: once full, a new event is dropped and counted. */
/* The distance check of the confirmation's window and of passive watching, both ways. */
TEST(two_positions_apart_within_bounds_either_way)
{
    ASSERT_TRUE(ace2k_rfid_watch_apart(100, 110, 0, 20));
    ASSERT_TRUE(ace2k_rfid_watch_apart(110, 100, 0, 20));
    ASSERT_TRUE(!ace2k_rfid_watch_apart(100, 130, 0, 20));
    ASSERT_TRUE(!ace2k_rfid_watch_apart(130, 100, 0, 20));
    ASSERT_TRUE(ace2k_rfid_watch_apart(100, 150, 30, 60));
    ASSERT_TRUE(ace2k_rfid_watch_apart(150, 100, 30, 60));
    ASSERT_TRUE(!ace2k_rfid_watch_apart(100, 110, 30, 60));
    ASSERT_TRUE(!ace2k_rfid_watch_apart(110, 100, 30, 60));
    ASSERT_TRUE(ace2k_rfid_watch_apart(INT32_MAX, INT32_MIN, 0, INT32_MAX) == false);
}

/* A read with motion on a lane out of range that the search answered ok forgets nothing. */
TEST(a_read_with_motion_out_of_range_touches_no_lane)
{
    struct wworld ww;
    ww_init(&ww);
    struct ace2k_fake_search f = { .answer = ACE2K_RFID_READ_OK };
    ASSERT_EQ(ace2k_rfid_watch_read_with_motion(&ww.w, ACE2K_LANE_COUNT, fake_search_start, &f),
              ACE2K_RFID_READ_OK);
    ASSERT_EQ(f.calls, 1);
}

TEST(a_full_event_ring_keeps_the_oldest_events_and_counts_the_dropped_ones)
{
    struct wworld ww;
    ww_init(&ww);
    for (int i = 0; i < 20; i++) {
        ww.in.insert = 0x01; /* pending */
        ww_steps(&ww, 1);
        ww.in.insert = 0; /* unknown */
        ww_steps(&ww, 1);
    }
    struct ace2k_rfid_event e;
    int kept = 0;
    while (ace2k_rfid_watch_peek_event(&ww.w, &e)) {
        ASSERT_EQ(e.kind, kept % 2 == 0 ? ACE2K_RFID_PENDING : ACE2K_RFID_UNKNOWN);
        ace2k_rfid_watch_drop_event(&ww.w);
        kept++;
    }
    ASSERT_EQ(kept, (int)ACE2K_RFID_EVENT_RING - 1);
    ASSERT_EQ(ww.w.dropped, 40U - (ACE2K_RFID_EVENT_RING - 1U));
}

TEST(dropping_from_an_empty_ring_changes_nothing_and_the_next_event_is_kept)
{
    struct wworld ww;
    ww_init(&ww);
    struct ace2k_rfid_event e;
    ace2k_rfid_watch_drop_event(&ww.w);
    ASSERT_TRUE(!ace2k_rfid_watch_peek_event(&ww.w, &e));
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ASSERT_TRUE(ace2k_rfid_watch_peek_event(&ww.w, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ace2k_rfid_watch_drop_event(&ww.w);
    ASSERT_TRUE(!ace2k_rfid_watch_peek_event(&ww.w, &e));
}

/* A read on command waiting on a reader that just gave up is answered there, not at its
 * deadline. */
TEST(a_read_on_command_pending_when_the_reader_dies_is_answered_at_once)
{
    struct wworld ww;
    ww_init(&ww);
    ww.configure_rc[ACE2K_RFID_READER_B] = -ACE2K_EIO;
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    for (int i = 0; i < 200 && ww.configures[1] < 2; i++) {
        ww_steps(&ww, 1);
    }
    ASSERT_EQ(ww.configures[1], 2);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    uint32_t asked = ww.now;
    struct ace2k_rfid_event e;
    bool answered = false;
    for (int i = 0; i < (int)(ACE2K_RFID_PROBE_MAX_MS / 10U) && !answered; i++) {
        ww_steps(&ww, 1);
        answered = ww_next(&ww, ACE2K_RFID_EV_STATE, &e);
        if (answered) {
            ASSERT_TRUE(ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_B));
        }
    }
    ASSERT_TRUE(answered);
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww.now - asked <= ACE2K_RFID_CONFIGURE_RETRY_MS + 10U);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
}

/* The strand leaving the lane that has no session leaves the neighbour's session open. */
TEST(the_neighbour_emptied_during_a_session_leaves_the_session_open)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(e.kind, ACE2K_RFID_UNKNOWN);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
}

/* A load that starts on a no_tag lane searches again; one that starts on a read lane leaves it
 * read. */
TEST(a_load_from_no_tag_searches_again_and_a_load_on_a_read_lane_leaves_it_read)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww_steps(&ww, 2);
    ww.in.loading = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_NO_TAG);
    ww.in.exhausted = 0;
    ww.in.loading = 0x01;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
    ww.in.loading = 0;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ww.in.moving = 0;
    ww.in.loading = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ww.in.loading = 0;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
}

TEST(forget_takes_a_no_tag_lane_back_to_pending_and_leaves_any_other_state_alone)
{
    struct wworld ww;
    ww_init(&ww);
    ace2k_rfid_watch_forget(&ww.w, ACE2K_LANE_COUNT); /* out of range: nothing */
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    drain(&ww);
    ace2k_rfid_watch_forget(&ww.w, 0);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ace2k_rfid_watch_peek_event(&ww.w, &e)); /* pending stays pending, no event */
    ww.in.loading = 0x01;
    ww_steps(&ww, 2);
    ace2k_rfid_watch_forget(&ww.w, 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
    ww.in.loading = 0;
    ww.in.exhausted = 0x01;
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_NO_TAG);
    drain(&ww);
    ace2k_rfid_watch_forget(&ww.w, 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
}

TEST(a_lane_or_reader_out_of_range_reads_as_unknown_off_and_healthy)
{
    struct wworld ww;
    ww_init(&ww);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, ACE2K_LANE_COUNT), ACE2K_RFID_UNKNOWN);
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_COUNT));
    ASSERT_TRUE(!ace2k_rfid_watch_fault(&ww.w, ACE2K_RFID_READER_COUNT));
    ASSERT_TRUE(!ace2k_rfid_watch_dead(&ww.w, ACE2K_RFID_READER_COUNT));
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_COUNT));
    ace2k_rfid_watch_clear_dead(&ww.w, ACE2K_RFID_READER_COUNT); /* nothing to clear */
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, ACE2K_LANE_COUNT), ACE2K_RFID_READ_NO_FILAMENT);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, ACE2K_LANE_COUNT, 1, true), -ACE2K_EINVAL);
}

/* The health probe leaves a reader alone while its field is on or a job runs on it. */
TEST(a_reader_is_busy_with_its_field_on_or_a_job_running_and_free_otherwise)
{
    struct wworld ww;
    ww_init(&ww);
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 2);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_B));
    ASSERT_TRUE(!ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_A));
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&ww.iso, ACE2K_RFID_READER_A, &job), 0);
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_A));
    ASSERT_TRUE(ace2k_rfid_watch_reader_busy(&ww.w, ACE2K_RFID_READER_A));
}

/* Session ids run 1..255: after 255 comes 1, since 0 means no session. */
TEST(the_session_id_after_255_is_1_never_0)
{
    struct wworld ww;
    ww_init(&ww);
    ww.w.next_session = 255;
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(session, 1);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, 0, true), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
}

TEST(a_step_is_refused_out_of_range_for_another_lane_or_session_and_while_one_waits_or_runs)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, ACE2K_LANE_COUNT, session,
                                            ACE2K_RFID_OP_MIFARE_A, 1, 1, key1, ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_B + 1U, 1, 1,
                                            key1, ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 1, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              -ACE2K_EREFUSED); /* the neighbour has no session */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, (uint8_t)(session + 1U),
                                            ACE2K_RFID_OP_MIFARE_A, 1, 1, key1, ww.now),
              -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              -ACE2K_EREFUSED); /* one waits */
    ww_steps(&ww, 1);
    ASSERT_TRUE(ace2k_iso14443a_busy(&ww.iso, ACE2K_RFID_READER_B));
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              -ACE2K_EREFUSED); /* one runs */
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 1, session, true), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, (uint8_t)(session + 1U), true), -ACE2K_EREFUSED);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
}

TEST(a_mifare_step_with_key_b_reads_the_sector)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    const uint8_t key_b[6] = { 1, 9, 8, 7, 6, 5 }; /* fake_field's sector-1 key B */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_B, 1, 0x1,
                                            key_b, ww.now),
              0);
    ww_steps(&ww, 20);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.block, 4);
    ASSERT_EQ(e.data[0], 64);
}

/* Lane 2 reads the tag `kept`, which then leaves.  With no host step the session ends pending,
 * the UID kept.  Then lane 1 moves alone, and `kept` comes back together with the new tag
 * `fresh`.  The fake's inventory lists 11 22 33 44 before 01 02 03 04, and lane 1 is attributed
 * before lane 2.  e is lane 1's tag event. */
static void lane2s_kept_tag_returns_as_lane1_moves(struct wworld *ww, const uint8_t kept[4],
                                                   const uint8_t fresh[4], struct ftag **x,
                                                   struct ftag **y, struct ace2k_rfid_event *e)
{
    *y = tag_b(ww, kept, false);
    *x = tag_b(ww, fresh, false);
    ww->in.insert = 0x03;
    ww_steps(ww, 1);
    turn_in(ww, 1, *y);
    struct ace2k_rfid_event first;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &first));
    ASSERT_EQ(first.lane, 1);
    (*y)->absent = true;
    ww_steps(ww, 60);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 1), ACE2K_RFID_PENDING);
    drain(ww);
    turn_out(ww, 0, *x);
    (*x)->absent = false;
    (*y)->absent = false;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, e));
    ASSERT_EQ(e->lane, 0);
}

/* One inventory sees a new tag for the moving lane 1 and lane 2's own known tag back.  The new
 * tag is listed first, so lane 1 takes it and opens a session.  Lane 2 waits for it to close,
 * then opens its own. */
static uint8_t lane2_waits_behind_lane1(struct wworld *ww, struct ftag **x, struct ftag **y)
{
    struct ace2k_rfid_event e;
    lane2s_kept_tag_returns_as_lane1_moves(ww, ua, ub, x, y, &e);
    ASSERT_TRUE(memcmp(e.tag.uid, ub, 4) == 0);
    ASSERT_TRUE(ww->w.l[1].want);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 1), ACE2K_RFID_PENDING);
    return e.session;
}

TEST(a_tag_claimed_while_the_neighbours_session_is_open_gets_its_session_when_that_one_closes)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x;
    struct ftag *y;
    uint8_t session = lane2_waits_behind_lane1(&ww, &x, &y);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_READING);
}

TEST(a_waiting_claim_is_dropped_when_the_link_goes_down)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x;
    struct ftag *y;
    (void)lane2_waits_behind_lane1(&ww, &x, &y);
    ww.in.link_ok = false;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ww.w.l[1].want);
    x->absent = true;
    y->absent = true;
    ww.in.link_ok = true;
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
}

/* Lane 2's kept UID stays lane 2's, whichever the anticollision lists first: the UID the watch
 * knows as a lane's own is never its neighbour's, even when it enters while only the neighbour
 * moves.  The moving lane 1 goes on to the new tag, and lane 2 waits for its session.  Up to
 * v0.12.0 the inventory's order decided: with the kept UID listed first, lane 1 took it and the
 * new tag went to no lane. */
static void kept_uid_stays_lane2s(const uint8_t kept[4], const uint8_t fresh[4])
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x;
    struct ftag *y;
    struct ace2k_rfid_event e;
    lane2s_kept_tag_returns_as_lane1_moves(&ww, kept, fresh, &x, &y, &e);
    ASSERT_TRUE(memcmp(e.tag.uid, fresh, 4) == 0); /* lane 1: the new tag */
    ASSERT_TRUE(ww.w.l[1].want);                   /* lane 2: its own, waiting */
    ASSERT_TRUE(memcmp(ww.w.l[1].wanted.uid, kept, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, e.session, true), 0);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(memcmp(e.tag.uid, kept, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_READING);
}

TEST(a_kept_uid_stays_its_lanes_whichever_the_anticollision_lists_first)
{
    kept_uid_stays_lane2s(ua, ub); /* the new tag listed first */
    kept_uid_stays_lane2s(ub, ua); /* the kept UID listed first */
}

/* A read on command that sees only the neighbour's kept UID: that UID is never this lane's, so
 * no UID is left, and the answer is the lane's unchanged state, not ambiguous (up to v0.12.0 a
 * kept UID counted as a candidate).  No session opens for this lane. */
TEST(a_read_on_command_that_sees_only_the_neighbours_kept_uid_answers_the_unchanged_state)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *kept = tag_b(&ww, ua, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    turn_in(&ww, 1, kept);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    kept->absent = true;
    ww_steps(&ww, 60);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING); /* the UID kept */
    ww.in.moving = 0;
    ww_steps(&ww, 1);
    drain(&ww);
    kept->absent = false; /* back in the field, alone */
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    /* the UID back in the field is lane 2's own again: its session, never lane 1's */
    while (ww_next(&ww, ACE2K_RFID_EV_TAG, &e)) {
        ASSERT_EQ(e.lane, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* Lane 1 reads spool X, and the host gives up on it: the session ends with lane 1 pending and X
 * kept.  A leftover piece of filament keeps lane 1's insert sensor covered.  X goes to bay 2, and
 * lane 2 loads.  When X turns past the antenna, it is lane 1's own UID, so lane 1 opens a session
 * on it again and lane 2 gets nothing.  The host gives up once more, with X out of the field. */
static void lane1_keeps_x_while_lane2_loads(struct wworld *ww, struct ftag **x)
{
    *x = tag_b(ww, ua, false);
    ww->in.insert = 0x03;
    ww_steps(ww, 1);
    turn_in(ww, 0, *x);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ww->in.moving = 0;
    (*x)->absent = true; /* the spool leaves bay 1 */
    ASSERT_EQ(ace2k_rfid_watch_done(&ww->w, 0, e.session, false), 0);
    ww_steps(ww, 5);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 0), ACE2K_RFID_PENDING); /* X kept */
    ww->in.moving = 0x02;
    ww->in.loading = 0x02;
    ww_steps(ww, 20);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 1), ACE2K_RFID_SEARCHING);
    (*x)->absent = false; /* X turns past the antenna, in bay 2 */
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    (*x)->absent = true;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww->w, 0, e.session, false), 0);
    ww_steps(ww, 5);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 0), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww->w, 1), ACE2K_RFID_SEARCHING);
    drain(ww);
}

/* A forget on lane 1 also drops the UID it keeps.  The next time X enters while lane 2 loads, X is
 * a candidate of lane 2, and lane 2's once it follows lane 2's reversal.  Up to v0.12.0 the forget
 * did nothing on a pending lane, and lane 1 took X again. */
TEST(a_forget_releases_the_kept_uid_and_the_neighbour_can_then_take_it)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x;
    lane1_keeps_x_while_lane2_loads(&ww, &x);
    ace2k_rfid_watch_forget(&ww.w, 0);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ace2k_rfid_watch_peek_event(&ww.w, &e)); /* pending stays pending, no event */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_tag(&ww.w, 0)->uid_len, 0);
    ww_steps(&ww, 30); /* X away for more than ACE2K_RFID_DEBOUNCE_POLLS inventories */
    /* X left while lane 2 moved, so it is already a candidate: lane 2 goes back a whole leg without
     * it, then forward until X follows. */
    struct lanetag lt = { x, 1, 20000, 136000 }; /* X on lane 2's spool, turned past the antenna */
    ww.own_um_s[1] = 20000;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 4000, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* Lane 1 reads X, and X leaves during a step: the session ends lost, with X kept.  Lane 2 reads Y,
 * and the host gives up: Y is kept for lane 2.  A forget on lane 2 drops Y and touches nothing of
 * lane 1's: its kept UID, its want and its lost bit stay.  When X comes back while lane 2 moves, X
 * is lane 1's again. */
TEST(a_forget_on_the_other_lane_leaves_the_kept_uid_where_it_is)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x = tag_b(&ww, ua, false);
    struct ftag *y = tag_b(&ww, ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    turn_in(&ww, 0, x);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    x->absent = true;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, e.session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 30);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0x01);
    turn_in(&ww, 1, y);
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    y->absent = true;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 1, e.session, false), 0);
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    ASSERT_TRUE(memcmp(ace2k_rfid_watch_tag(&ww.w, 1)->uid, ub, 4) == 0); /* Y kept */
    struct ace2k_rfid_watch_lane before = ww.w.l[0];
    ace2k_rfid_watch_forget(&ww.w, 1);
    ASSERT_EQ(ace2k_rfid_watch_tag(&ww.w, 1)->uid_len, 0); /* lane 2's own UID dropped */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    ASSERT_EQ(ww.w.l[0].tag.uid_len, before.tag.uid_len);
    ASSERT_TRUE(memcmp(ww.w.l[0].tag.uid, before.tag.uid, sizeof before.tag.uid) == 0);
    ASSERT_TRUE(ww.w.l[0].has_tag && before.has_tag);
    ASSERT_EQ(ww.w.l[0].want, before.want);
    ASSERT_EQ(ww.w.l[0].wanted.uid_len, before.wanted.uid_len);
    ASSERT_TRUE(memcmp(ww.w.l[0].wanted.uid, before.wanted.uid, sizeof before.wanted.uid) == 0);
    ASSERT_EQ(ace2k_rfid_watch_lost_bits(&ww.w), 0x01);
    ww.in.moving = 0x02;
    ww_steps(&ww, 20);
    x->absent = false;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* A forget on a searching lane changes nothing: its kept UID is forgotten once the search ends.
 * The lane saw its tag, so a search that runs out still never gives no_tag. */
TEST(a_forget_on_a_searching_lane_changes_nothing_and_its_search_never_ends_no_tag)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x = tag_b(&ww, ua, false);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    struct lanetag lt = { x, 0, 100000, 216000 };
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 1000, &e)); /* it followed */
    x->absent = true;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, e.session, false), 0);
    ww_steps(&ww, 5);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
    drain(&ww);
    ace2k_rfid_watch_forget(&ww.w, 0);
    ASSERT_TRUE(!ace2k_rfid_watch_peek_event(&ww.w, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
    ASSERT_TRUE(memcmp(ace2k_rfid_watch_tag(&ww.w, 0)->uid, ua, 4) == 0);
    ww.in.loading = 0;
    ww.in.moving = 0;
    ww.in.exhausted = 0x01; /* the search ran its whole reach */
    ww_steps(&ww, 2);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* Lane 2's own UID comes back while lane 1's session is open, so lane 2 waits for its turn (a
 * want).  A forget on lane 2 drops the waiting UID too: when lane 1's session ends, no session
 * opens for lane 2, and it stays pending.  The next move of lane 2 reads the tag again. */
TEST(a_forget_drops_a_waiting_uid_and_the_lanes_next_move_reads_the_tag_again)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *x;
    struct ftag *y;
    uint8_t session = lane2_waits_behind_lane1(&ww, &x, &y);
    ace2k_rfid_watch_forget(&ww.w, 1);
    ASSERT_TRUE(!ww.w.l[1].want);
    ASSERT_EQ(ww.w.l[1].wanted.uid_len, 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    y->absent = true;
    ww.in.moving = 0;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ww_steps(&ww, 5);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* no session for lane 2 */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    ww.in.moving = 0x02; /* lane 2's next move */
    ww_steps(&ww, 20);
    y->absent = false;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* A forget leaves an open session alone: its steps still read the tag that opened it.  It also
 * leaves a waiting read on command alone, and the next inventory still answers it. */
TEST(a_forget_leaves_an_open_session_and_a_waiting_read_on_command_alone)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ace2k_rfid_watch_forget(&ww.w, 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_TRUE(memcmp(ace2k_rfid_watch_tag(&ww.w, 0)->uid, ua, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ww.in.moving = 0;
    t->absent = true;
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, false), 0);
    ww_steps(&ww, 5);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ace2k_rfid_watch_forget(&ww.w, 0); /* drops the kept UID; the read still waits */
    ASSERT_TRUE(ace2k_rfid_watch_lane_busy(&ww.w, 0));
    ASSERT_EQ(ace2k_rfid_watch_tag(&ww.w, 0)->uid_len, 0);
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e)); /* no UID in the field: the state */
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
}

/* Lanes 3–4 are reader A's, the reader at NSS PB12.  A strand that moves in lane 3 turns on
 * reader A's field and never reader B's, and a tag that enters there is lane 3's. */
TEST(lane_3_moving_turns_on_reader_as_field_never_reader_bs_and_its_tag_is_lane_3s)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = add_mifare(&ww.ff.rd[ACE2K_RFID_READER_A], ua);
    t->absent = true;
    ww.in.insert = 0x04;
    ww_steps(&ww, 1);
    ww.in.loading = 0x04;
    ww.in.moving = 0x04;
    ww_steps(&ww, 2); /* configure, then the field */
    ASSERT_EQ(ww.configures[ACE2K_RFID_READER_A], 1);
    ASSERT_EQ(ww.configures[ACE2K_RFID_READER_B], 0);
    ASSERT_TRUE(ww.field[ACE2K_RFID_READER_A] && !ww.field[ACE2K_RFID_READER_B]);
    ASSERT_TRUE(ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_A));
    ASSERT_TRUE(!ace2k_rfid_watch_field(&ww.w, ACE2K_RFID_READER_B));
    ww_steps(&ww, 20); /* inventories of an empty field: the baseline */
    ww.own_um_s[2] = 20000;
    struct lanetag lt = { t, 2, 100000, 216000 };
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 1000, &e)); /* it followed */
    ASSERT_EQ(e.lane, 2);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 2), ACE2K_RFID_READING);
    ASSERT_TRUE(!ww.field[ACE2K_RFID_READER_B]);
    ASSERT_EQ(ww.configures[ACE2K_RFID_READER_B], 0);
}

TEST(a_second_read_on_command_while_one_waits_is_busy)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_BUSY);
}

/* The link went down after the read on command was accepted: its single tag is not taken, and
 * the answer is ambiguous.  A read on command attributes nothing, with the link up or down. */
TEST(a_read_on_command_decided_after_the_link_went_down_answers_ambiguous_and_opens_no_session)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ud, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    (void)read_lane2(&ww, z); /* a read neighbour, and one other tag */
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    tag_b(&ww, ua, true);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww.in.link_ok = false;
    ww_steps(&ww, 50); /* the baseline: three inventories of two tags */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ace2k_rfid_watch_peek_event(&ww.w, &e));
    ASSERT_EQ(e.type, ACE2K_RFID_EV_STATE);
    ASSERT_EQ(e.kind, ACE2K_RFID_KIND_AMBIGUOUS);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

TEST(with_the_insert_unknown_no_tag_is_taken_no_state_moves_and_a_read_is_busy)
{
    /* The insert readings stopped: neither the lane's bit nor its neighbour's says anything, so
     * nothing is attributed on them, and the frozen bit dropping empties nothing. */
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ub, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    drain(&ww);
    ww.in.insert_unknown = true;
    ww.in.moving = 0x01; /* a move the feed is ending on the same tick */
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_BUSY);
    ww.in.insert = 0x00;
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
    /* the readings back, the strand still there: the resting tag is nobody's, and it is the
     * lane's once its motion turns it out and in again */
    ww.in.insert = 0x01;
    ww.in.insert_unknown = false;
    ww.in.moving = 0x01;
    ww_steps(&ww, 40);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ww.ff.rd[1].t[0].absent = true;
    ww_steps(&ww, 30); /* away for more than ACE2K_RFID_DEBOUNCE_POLLS inventories */
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM; /* a whole turn of the spool */
    ww.ff.rd[1].t[0].absent = false;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 40, &e));
    ASSERT_EQ(e.lane, 0);
}

TEST(a_session_on_a_reader_that_stops_answering_still_ends_after_half_a_second_with_no_key)
{
    /* A soft reset clears the reader under an open session, and every configure after it fails.
     * The step the host sent never starts.  The session still ends on its idle time, and the key
     * of that step goes with it. */
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ww.configure_rc[ACE2K_RFID_READER_B] = -ACE2K_EIO;
    ww.configured[1] = false;
    ww.field[1] = false;
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1, key1,
                                            ww.now),
              0);
    int starts = ww.ff.rd[1].starts;
    ww_steps(&ww, 45);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ww_steps(&ww, 10);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
    const uint8_t *k = ww.w.r[ACE2K_RFID_READER_B].step_job.key;
    ASSERT_TRUE(k[0] == 0 && k[1] == 0 && k[2] == 0 && k[3] == 0 && k[4] == 0 && k[5] == 0);
    ASSERT_EQ(ww.ff.rd[1].starts, starts); /* the step never ran */
}

/* Lane 2 (index 1) has filament and moves alone while its tag comes back after a turn of its
 * spool.  True when a TAG came. */
static bool lane2_tag_turns_in(bool by_hand)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    ww.in.insert = 0x02;
    ww_steps(&ww, 1);
    if (by_hand) {
        ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    }
    turn_in(&ww, 1, t);
    struct ace2k_rfid_event e;
    bool got = ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e);
    if (by_hand) {
        ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
        ww.in.insert = 0;
        ww_steps(&ww, 1);
        ww.in.insert = 0x02;
        ww_steps(&ww, 1);
        ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
        ASSERT_TRUE(!ww.w.l[1].by_hand);
        ASSERT_TRUE(ace2k_rfid_watch_eligible(&ww.w, 1));
        /* a lane with no filament, or out of range, is refused */
        ace2k_rfid_watch_set_by_hand(&ww.w, 0);
        ace2k_rfid_watch_set_by_hand(&ww.w, 9);
        ASSERT_TRUE(!ww.w.l[0].by_hand);
    }
    return got;
}

TEST(a_lane_set_by_hand_takes_no_tag_until_its_strand_leaves)
{
    ASSERT_TRUE(lane2_tag_turns_in(false)); /* the same sequence, not by hand: the tag is taken */
    ASSERT_TRUE(!lane2_tag_turns_in(true));
}

/* The host names lane 2 by hand while the watch has not yet made the lane pending
 * (here the insert readings are frozen, so the lane stays unknown).  The insert sensor reads
 * filament, so the mark is taken, and the lane takes no tag once it moves.  The mark still goes
 * when the strand leaves. */
TEST(by_hand_is_taken_while_the_lane_is_still_unknown)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    ww.in.insert = 0x02;
    ww.in.insert_unknown = true;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_UNKNOWN);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    ASSERT_TRUE(ww.w.l[1].by_hand);
    ww.in.insert_unknown = false;
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    turn_in(&ww, 1, t);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    ww.in.insert = 0;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ww.w.l[1].by_hand);
}

/* A link loss keeps a lane set by hand, so a host that stays up through it does
 * not lose the mark.  Only the strand leaving clears it.  The in-use mask still dies with the
 * link. */
TEST(a_link_loss_keeps_a_lane_set_by_hand)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    ww.in.insert = 0x02;
    ww_steps(&ww, 1);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    ace2k_rfid_watch_set_in_use(&ww.w, 0x02);
    ww.in.link_ok = false;
    ww_steps(&ww, 1);
    ww.in.link_ok = true;
    ww_steps(&ww, 1);
    ASSERT_TRUE(ww.w.l[1].by_hand);
    uint8_t m = 0xFF;
    ASSERT_TRUE(!ace2k_rfid_watch_in_use(&ww.w, &m));
    turn_in(&ww, 1, t);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ww.in.insert = 0;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ww.w.l[1].by_hand);
}

/* The marks: a by-hand command on a lane with no filament is refused, and only the strand
 * leaving clears a mark. */
TEST(a_mark_needs_filament_and_goes_only_when_its_strand_leaves)
{
    struct wworld ww;
    ww_init(&ww);
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1); /* no filament: refused */
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0);
    ww.in.insert = 0x06;
    ww_steps(&ww, 1);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    ace2k_rfid_watch_set_by_hand(&ww.w, 2);
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0x06);
    ww.in.insert = 0x04; /* lane 2's strand leaves: its mark goes */
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0x04);
}

TEST(by_hand_on_a_read_lane_keeps_its_uid_from_the_neighbour)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, session, true), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ace2k_rfid_watch_set_by_hand(&ww.w, 0);
    ASSERT_TRUE(ww.w.l[0].by_hand);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READ);
    ASSERT_TRUE(ace2k_rfid_watch_tag(&ww.w, 0)->uid_len == 4);
    ASSERT_TRUE(memcmp(ace2k_rfid_watch_tag(&ww.w, 0)->uid, ua, 4) == 0);
    /* the neighbour moves with the same tag in view: it is not the neighbour's */
    ww.in.insert = 0x03;
    ww.in.moving = 0x02;
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 1), ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww.w.l[0].by_hand);
    ASSERT_TRUE(!ace2k_rfid_watch_eligible(&ww.w, 0));
    ASSERT_TRUE(memcmp(ace2k_rfid_watch_tag(&ww.w, 0)->uid, ua, 4) == 0);
}

/* Bench 2026-11-12: the host's forget (ACEK_RFID_FORGET, the U1's Read tag) drops the lane's
 * by-hand mark too: a forgotten lane can be read again. */
TEST(the_hosts_forget_clears_the_by_hand_mark)
{
    struct wworld ww;
    ww_init(&ww);
    ww.in.insert = 0x02;
    ww_steps(&ww, 1);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0x02);
    ace2k_rfid_watch_host_forget(&ww.w, 1);
    ASSERT_TRUE(!ww.w.l[1].by_hand);
    ASSERT_EQ(ace2k_rfid_watch_by_hand_bits(&ww.w), 0);
    ASSERT_TRUE(ace2k_rfid_watch_eligible(&ww.w, 1));
    ace2k_rfid_watch_host_forget(&ww.w, 9); /* out of range: nothing */
}

/* Bench 2026-11-12: lane 2 is set by hand (the host restored its read after a restart) and
 * still.  Lane 1 loads, and a new UID enters: it can only be lane 1's, so it is taken at once,
 * with no confirmation. */
TEST(beside_a_still_lane_set_by_hand_a_new_uid_is_the_loading_lanes_at_once)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ace2k_rfid_watch_set_by_hand(&ww.w, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    struct ace2k_rfid_event e;
    bool got = false;
    for (int i = 0; i < 1000 && !got; i++) {
        got = ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 1, &e);
        ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    }
    ASSERT_TRUE(got);
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
    ASSERT_TRUE(ww.w.l[1].by_hand);
}

TEST(the_in_use_mask_ages_out_and_dies_with_the_link)
{
    struct wworld ww;
    ww_init(&ww);
    uint8_t m = 0xFF;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ace2k_rfid_watch_in_use(&ww.w, &m));
    ASSERT_EQ(m, 0);
    uint32_t t = ww.now;
    ace2k_rfid_watch_set_in_use(&ww.w, 0x05);
    ace2k_rfid_watch_step(&ww.w, t + 2999, &ww.in);
    ASSERT_TRUE(ace2k_rfid_watch_in_use(&ww.w, &m));
    ASSERT_EQ(m, 0x05);
    ace2k_rfid_watch_step(&ww.w, t + 3001, &ww.in);
    ASSERT_TRUE(!ace2k_rfid_watch_in_use(&ww.w, &m));
    ASSERT_EQ(m, 0);
    ww.now = t + 3010;
    ace2k_rfid_watch_step(&ww.w, ww.now, &ww.in);
    ace2k_rfid_watch_set_in_use(&ww.w, 0x02);
    ASSERT_TRUE(ace2k_rfid_watch_in_use(&ww.w, &m));
    ww.in.link_ok = false;
    ww_steps(&ww, 1);
    ww.in.link_ok = true;
    ww_steps(&ww, 1);
    ASSERT_TRUE(!ace2k_rfid_watch_in_use(&ww.w, &m));
    ASSERT_EQ(m, 0);
}

/* The field report: lane 1 runs (a print's assist) with its own tag at rest in the field.  A spool
 * is hung on bay 2, no strand in bay 2 yet, and its tag enters the field.  It is not lane 1's: a
 * tag that only enters proves nothing about the bay it came from. */
TEST(a_spool_hung_on_the_neighbour_bay_while_this_lane_runs_is_not_this_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ua, true);                      /* lane 1's own spool, at rest */
    struct ftag *hung = tag_b(&ww, ub, false); /* the spool about to be hung on bay 2 */
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01; /* lane 1 runs (a print's assist) */
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    hung->absent = false; /* hung on bay 2 */
    ww_steps(&ww, 30);
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
}

/* Passive watching: lane 1 moves alone, outside a load.  Its tag leaves the field and comes back
 * after the lane travelled the minimum: a full turn of its spool.  It is lane 1's. */
TEST(
    a_tag_that_leaves_and_returns_after_the_minimum_travel_while_only_this_lane_moves_is_this_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *mine = tag_b(&ww, ua, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    mine->absent = true;
    ww_steps(&ww, 40); /* it left */
    ww_travel(&ww, 0, ACE2K_RFID_PASSIVE_MIN_UM, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag); /* leaving alone gives nothing */
    mine->absent = false;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* A tag that comes back before the minimum travel is nobody's.  Its next leave is measured anew,
 * so two short absences do not add up to a turn. */
TEST(a_tag_that_flickers_back_before_the_minimum_travel_is_nobodys)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    for (int k = 0; k < 2; k++) {
        t->absent = true;
        ww_steps(&ww, 40); /* it left */
        ww_travel(&ww, 0, ACE2K_RFID_PASSIVE_MIN_UM - 1, 30);
        t->absent = false;
        ww_steps(&ww, 30);
    }
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* A leave is measured within one insertion of the lane and one spell of the field.  The lane's
 * strand leaving its insert sensor drops its records, and so does the field going off. */
TEST(a_leave_before_the_strand_left_or_the_field_went_off_does_not_count)
{
    for (int field_off = 0; field_off < 2; field_off++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *t = tag_b(&ww, ua, true);
        ww.in.insert = 0x01;
        ww_steps(&ww, 1);
        ww.in.moving = 0x01;
        ww_steps(&ww, 30);
        t->absent = true;
        ww_steps(&ww, 40); /* it left */
        ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].gone_count, 1);
        if (field_off != 0) {
            ww.in.moving = 0;
            ww_steps(&ww, 5);
            ASSERT_TRUE(!ww.field[1]);
            ww.in.moving = 0x01;
            ww_steps(&ww, 40); /* a new baseline, the tag still away */
        } else {
            ww.in.insert = 0;
            ww_steps(&ww, 1);
            ww.in.insert = 0x01;
            ww_steps(&ww, 1);
        }
        ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].gone_count, 0);
        ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
        t->absent = false;
        ww_steps(&ww, 30);
        struct ace2k_rfid_event e;
        ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
        ASSERT_TRUE(!ww.w.l[0].has_tag);
    }
}

/* The table of leaves is full.  One record is of a UID back in view: it waits for no return, so a
 * new leave gives it up, not the oldest.  The oldest, a tag still away, later comes back after a
 * turn and is the lane's. */
TEST(a_full_table_of_leaves_gives_up_a_uid_back_in_view_before_the_oldest)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *mine = tag_b(&ww, ua, true);
    tag_b(&ww, ub, true); /* in view throughout */
    struct ftag *other = tag_b(&ww, ud, true);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 40);
    mine->absent = true;
    ww_steps(&ww, 40); /* it left: the oldest record */
    struct ace2k_rfid_watch_reader *rd = &ww.w.r[ACE2K_RFID_READER_B];
    ASSERT_EQ(rd->gone_count, 1);
    /* the rest of the table: a record of ub, back in view, then UIDs away */
    rd->gone[1] = (struct ace2k_iso14443a_tag){ .uid_len = 4, .uid = { 0x11, 0x22, 0x33, 0x44 } };
    rd->gone_at_um[1] = 0;
    rd->gone_lane[1] = 0;
    for (uint8_t j = 2; j < ACE2K_RFID_TRACKED_MAX; j++) {
        rd->gone[j] = (struct ace2k_iso14443a_tag){ .uid_len = 4, .uid = { 0xA0, j, 0, 0 } };
        rd->gone_at_um[j] = 0;
        rd->gone_lane[j] = 0;
    }
    rd->gone_count = ACE2K_RFID_TRACKED_MAX;
    other->absent = true;
    ww_steps(&ww, 40); /* a new leave, the table full */
    ASSERT_EQ(rd->gone_count, ACE2K_RFID_TRACKED_MAX);
    bool ub_kept = false;
    bool ud_kept = false;
    for (uint8_t j = 0; j < rd->gone_count; j++) {
        ub_kept = (bool)(ub_kept || memcmp(rd->gone[j].uid, ub, 4) == 0);
        ud_kept = (bool)(ud_kept || memcmp(rd->gone[j].uid, ud, 4) == 0);
    }
    ASSERT_TRUE(!ub_kept && ud_kept);
    ASSERT_TRUE(memcmp(rd->gone[0].uid, ua, 4) == 0); /* the oldest survives */
    ww.in.travel_um[0] += ACE2K_RFID_PASSIVE_MIN_UM;
    mine->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* Lane 2 read and printing, lane 1 inserted.  A new UID that enters while lane 1 moves can only be
 * lane 1's: lane 2's spool holds lane 2's tag.  It is taken at once, in a load or outside one. */
TEST(with_the_neighbour_read_a_new_uid_entering_while_this_lane_moves_is_taken_at_once)
{
    for (int loading = 0; loading < 2; loading++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *z = tag_b(&ww, ud, false);
        struct ftag *y = tag_b(&ww, ub, false);
        ww.in.insert = 0x03;
        ww_steps(&ww, 1);
        (void)read_lane2(&ww, z); /* lane 2 read, its tag stays in view */
        ww.in.moving = 0x03;      /* lane 2 prints, lane 1 moves */
        ww.in.loading = loading != 0 ? 0x01 : 0;
        ww_steps(&ww, 20);
        y->absent = false;
        struct ace2k_rfid_event e;
        ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
        ASSERT_EQ(e.lane, 0);
        ASSERT_TRUE(memcmp(e.tag.uid, ub, 4) == 0);
    }
}

/* A tag in reader A's field (lanes 3–4), present or not yet. */
static struct ftag *tag_a(struct wworld *ww, const uint8_t uid[4], bool present)
{
    struct ftag *t = add_mifare(&ww->ff.rd[ACE2K_RFID_READER_A], uid);
    t->absent = (bool)!present;
    return t;
}

/* S4.  Lane 3 prints with its black tag, its material set by hand, so it is not read.  Lane 4 loads
 * with a yellow tag.  Black is in view the whole time, or lane 3's slow print brings it to the edge
 * of the field during lane 4's load, or it is out of reach.  Yellow follows lane 4's reversals and
 * is lane 4's.  Black never follows lane 4, so it is never lane 4's. */
TEST(s4_the_tag_that_follows_the_lane_is_its_own_the_neighbours_turning_one_is_not)
{
    for (int where = 0; where < 3; where++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *yellow = tag_a(&ww, ua, false);
        struct ftag *black = tag_a(&ww, ub, where == 0);
        ww.in.insert = 0x0C;
        ww_steps(&ww, 1);
        ace2k_rfid_watch_set_by_hand(&ww.w, 2);
        ww.in.loading = 0x08;
        ww.own_um_s[2] = 1000;  /* lane 3's print: slow, one way */
        ww.own_um_s[3] = 20000; /* lane 4's load */
        struct lanetag tags[2] = {
            { yellow, 3, 200000, 316000 },
            { black, 2, 5000, 121000 }, /* lane 3 brings black into view after 5 s */
        };
        int n = where == 1 ? 2 : 1;
        bool black_tried = false;
        struct ace2k_rfid_event e = { 0 };
        bool got = false;
        for (int i = 0; i < 8000 && !got; i++) {
            got = ww_drive_until(&ww, tags, n, ACE2K_RFID_EV_TAG, 1, &e);
            const struct ace2k_rfid_confirm *c = &ww.w.r[ACE2K_RFID_READER_A].confirm;
            if (c->on && c->uid.uid_len == 4 && memcmp(c->uid.uid, ub, 4) == 0) {
                black_tried = true;
            }
        }
        ASSERT_TRUE(got);
        ASSERT_EQ(e.lane, 3);
        ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
        ASSERT_EQ(black_tried, where == 1); /* at the edge, black entered: tried, never followed */
        ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 2), ACE2K_RFID_PENDING);
        ASSERT_EQ(ace2k_rfid_watch_tag(&ww.w, 2)->uid_len, 0);
    }
}

/* S7.  Lane 3 loads with its green tag in view at rest.  A spool hung on bay 4, with no strand,
 * holds a black tag: in view from the start, or hung during the load.  Green follows lane 3 and is
 * lane 3's.  Black never follows, so it is nobody's; hung during the load, it is a candidate that
 * lane 3 runs a whole leg back for, in vain. */
TEST(s7_a_hung_tag_that_does_not_follow_is_nobodys)
{
    for (int hung_late = 0; hung_late < 2; hung_late++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *green = tag_a(&ww, ua, true);
        struct ftag *black = tag_a(&ww, ub, hung_late == 0);
        ww.in.insert = 0x04;
        ww_steps(&ww, 1);
        ww.in.loading = 0x04;
        ww.own_um_s[2] = 20000;
        struct lanetag lt = { green, 2, -50000, 66000 };
        ww_drive(&ww, &lt, 1, 100);
        if (hung_late != 0) {
            black->absent = false; /* a hand hangs the spool while lane 3 moves */
            ww_drive(&ww, &lt, 1, 20);
            const struct ace2k_rfid_confirm *c = &ww.w.r[ACE2K_RFID_READER_A].confirm;
            ASSERT_TRUE(c->on && memcmp(c->uid.uid, ub, 4) == 0);
        }
        struct ace2k_rfid_event e;
        ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 6000, &e));
        ASSERT_EQ(e.lane, 2);
        ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
        ASSERT_TRUE(!black->absent);
        ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 3), ACE2K_RFID_UNKNOWN);
    }
}

/* Review: the neighbour's spool comes into view while lane 4 loads, so its tag is a candidate.  It
 * is lifted off its holder while lane 4 goes back: a change, but lane 4's forward leg never brings
 * it back.  The leg runs out and the tag is nobody's. */
TEST(a_neighbours_spool_lifted_off_during_a_confirmation_is_not_attributed)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *other = tag_a(&ww, ub, false);
    ww.in.insert = 0x0C;
    ww_steps(&ww, 1);
    ww.in.loading = 0x08;
    ww.own_um_s[3] = 20000;
    ww_drive(&ww, NULL, 0, 30);
    other->absent = false; /* lane 3's spool turned by hand: its tag comes into view */
    for (int i = 0; i < 50 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, NULL, 0, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x08);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0x08);
    ww_drive(&ww, NULL, 0, 50);
    other->absent = true; /* lifted off while lane 4 goes back */
    for (int i = 0; i < 100 && ace2k_rfid_watch_confirm_reverse_bits(&ww.w) != 0; i++) {
        ww_drive(&ww, NULL, 0, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x08); /* asked forward now */
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_drive_until(&ww, NULL, 0, ACE2K_RFID_EV_TAG, 2500, &e));
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0); /* the forward leg ran out */
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 3), ACE2K_RFID_SEARCHING);
    struct ace2k_rfid_confirm_trace t;
    ASSERT_TRUE(ww_trace(&ww, ACE2K_RFID_TRACE_LEG_OUT, &t));
    ASSERT_EQ(t.lane, 3);
    ASSERT_TRUE(!ww.w.l[3].has_tag && !ww.w.l[2].has_tag);
}

/* The tags the test's lanes got on this run: each one is lifted away from its bay, and its session
 * is given up. */
static void take_tags(struct wworld *ww, struct lanetag tags[2], uint8_t *got)
{
    struct ace2k_rfid_event e;
    while (ww_next(ww, ACE2K_RFID_EV_TAG, &e)) {
        ASSERT_TRUE(e.lane == 2 || e.lane == 3);
        ASSERT_TRUE(memcmp(e.tag.uid, e.lane == 2 ? ua : ub, 4) == 0);
        *got = (uint8_t)(*got | (1U << e.lane));
        struct lanetag *mine = &tags[e.lane - 2U];
        mine->in_lo_um = 1; /* lifted away: never in view again */
        mine->in_hi_um = 0;
        mine->t->absent = true;
        ASSERT_EQ(ace2k_rfid_watch_done(&ww->w, e.lane, e.session, false), 0);
    }
}

/* The test's feed: a lane whose candidate waits, or that has its tag, holds still. */
static void hold_waiting(struct wworld *ww, const struct ace2k_rfid_confirm *w, uint8_t got)
{
    for (uint8_t lane = 2; lane < 4; lane++) {
        bool held = (bool)((w->on && w->lane == lane) || (got & (1U << lane)) != 0U);
        ww->own_um_s[lane] = held ? 0 : 20000;
    }
}

/* Review: lanes 3 and 4 both load, and both their tags come into view, in the same inventory or
 * one just after the other.  One confirmation runs on the reader at a time, and the other lane's
 * candidate waits for it; this test's feed holds a lane still while its candidate waits.  In the
 * same inventory the first lane may try the other lane's tag first: that tag does not follow it,
 * the leg runs out, and the waiting lane gets its turn.  Each tag ends up with its own lane.  Once
 * a lane has its tag, the host gives up on the read with the spool lifted away, so its kept UID
 * stays out of the field. */
TEST(two_candidates_on_one_reader_are_confirmed_one_at_a_time)
{
    for (int staggered = 0; staggered < 2; staggered++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *t3 = tag_a(&ww, ua, false);
        struct ftag *t4 = tag_a(&ww, ub, false);
        ww.in.insert = 0x0C;
        ww_steps(&ww, 1);
        ww.in.loading = 0x0C;
        int32_t lo4 = staggered != 0 ? 102000 : 100000;
        struct lanetag tags[2] = { { t3, 2, 100000, 216000 }, { t4, 3, lo4, lo4 + 116000 } };
        const struct ace2k_rfid_confirm *w = &ww.w.r[ACE2K_RFID_READER_A].waiting;
        uint8_t got = 0;
        uint8_t waited = 0;
        for (int i = 0; i < 12000 && got != 0x0C; i++) {
            hold_waiting(&ww, w, got);
            ww_drive(&ww, tags, 2, 1);
            uint8_t bits = ace2k_rfid_watch_confirm_bits(&ww.w);
            ASSERT_TRUE(bits != 0x0C);
            if (w->on) {
                waited = (uint8_t)(waited | (1U << w->lane));
                ASSERT_TRUE((bits & (1U << w->lane)) == 0U);
            }
            take_tags(&ww, tags, &got);
        }
        ASSERT_EQ(got, 0x0C);
        ASSERT_TRUE(waited != 0);
        if (staggered != 0) {
            ASSERT_EQ(waited, 0x08); /* lane 3 had its candidate first; lane 4's waited */
        }
    }
}

/* Review: the link drops mid-confirmation.  It ends at once, and nothing restarts it while the lane
 * makes no change. */
TEST(a_link_drop_clears_the_confirmation)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, &lt, 1, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ww.in.link_ok = false;
    ww_drive(&ww, &lt, 1, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0);
    ww.in.link_ok = true;
    ww.own_um_s[0] = 0; /* the feed stops the load on the link drop */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 200, &e));
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_SEARCHING);
}

/* A leg that runs out (confirm_out) ends the confirmation, and the tag is not the lane's.  It stays
 * a candidate: when it leaves the field later in the same load, a confirmation starts again, and
 * this time the tag follows. */
TEST(confirm_out_fails_the_confirmation_and_a_later_change_starts_it_again)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, &lt, 1, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ww.in.confirm_out = 0x01; /* the feed had no room to go back */
    ww_out_epoch(&ww);
    ww_steps(&ww, 1);
    ww.in.confirm_out = 0;
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    /* the load goes on forward, the tag in view, until it leaves the far side of its window */
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, &lt, 1, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ASSERT_TRUE(ww.in.travel_um[0] > 216000);
    ASSERT_TRUE(!ww.w.r[ACE2K_RFID_READER_B].confirm.present);
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 1000, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ua, 4) == 0);
}

/* A confirmation on lane 1 ends on its leg running out, and a new one starts on
 * the same lane in the same step.  The feed still shows the old leg out until it has seen the new
 * confirmation: that stale out does not end the new one, nor burn another try. */
TEST(a_stale_confirm_out_does_not_end_the_next_confirmation)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww.w) == 0; i++) {
        ww_drive(&ww, &lt, 1, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ASSERT_EQ(ww.w.l[0].confirms, 1);
    ww.in.confirm_out = 0x01; /* the first confirmation's leg ran out */
    ww_out_epoch(&ww);
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    /* the same step, a new candidate: a second confirmation starts */
    struct ace2k_iso14443a_tag cand = { .uid_len = 4 };
    memcpy(cand.uid, ua, 4);
    ASSERT_TRUE(ace2k_rfid_watch_confirm_offer(&ww.w, 0, &cand));
    ASSERT_EQ(ww.w.l[0].confirms, 2);
    /* the feed has not seen it yet: its out is the first one's */
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x01);
    ASSERT_EQ(ww.w.l[0].confirms, 2);
    /* the feed's own leg for it runs out: that ends it */
    ww_out_epoch(&ww);
    ww_steps(&ww, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
}

/* Lanes 1 and 2 have strands in, lane 1 loads, and its tag t enters: a confirmation runs on lane 1,
 * asking it back. */
static void confirm_running_on_lane1(struct wworld *ww, const struct lanetag *lt)
{
    ww->in.insert = 0x03;
    ww_steps(ww, 1);
    ww->in.loading = 0x01;
    ww->own_um_s[0] = 20000;
    for (int i = 0; i < 1000 && ace2k_rfid_watch_confirm_bits(&ww->w) == 0; i++) {
        ww_drive(ww, lt, 1, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww->w), 0x01);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww->w), 0x01);
    ww_drive(ww, lt, 1, 5); /* well inside the first leg */
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww->w), 0x01);
}

/* After the confirmation ended, the lane is driven on so the tag would have followed: nothing is
 * attributed and no confirmation comes back. */
static void nothing_follows(struct wworld *ww, const struct lanetag *lt)
{
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_drive_until(ww, lt, 1, ACE2K_RFID_EV_TAG, 300, &e));
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww->w), 0);
    ASSERT_TRUE(!ww->w.l[0].has_tag);
}

/* The bench's trace of a confirmation: its start, each change counted with the lane's travel and
 * direction, and its success. */
TEST(a_confirmation_traces_its_start_each_counted_change_and_its_success)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 3000, &e));
    struct ace2k_rfid_confirm_trace t;
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &t));
    ASSERT_EQ(t.what, ACE2K_RFID_TRACE_START);
    ASSERT_EQ(t.lane, 0);
    ace2k_rfid_watch_drop_trace(&ww.w);
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &t));
    ASSERT_EQ(t.what, ACE2K_RFID_TRACE_FLIP); /* the tag left while the lane went back */
    ASSERT_TRUE(t.reverse);
    int32_t first = t.travel_um;
    ace2k_rfid_watch_drop_trace(&ww.w);
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &t));
    ASSERT_EQ(t.what, ACE2K_RFID_TRACE_FLIP); /* and came back as it went forward */
    ASSERT_TRUE(!t.reverse);
    ASSERT_TRUE(ace2k_rfid_watch_apart(t.travel_um, first, 0, ww.w.cfg.window_um));
    ace2k_rfid_watch_drop_trace(&ww.w);
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &t));
    ASSERT_EQ(t.what, ACE2K_RFID_TRACE_TAKEN);
    ace2k_rfid_watch_drop_trace(&ww.w);
    ASSERT_TRUE(!ace2k_rfid_watch_peek_trace(&ww.w, &t));
}

/* A confirmation (4 traces: start, two changes, taken) run with the trace ring already full and
 * `lost` traces counted before it; returns the first trace waiting after it. */
static struct ace2k_rfid_confirm_trace confirm_with_full_traces(uint8_t lost)
{
    struct wworld ww;
    ww_init(&ww);
    ww.w.trace_head = (uint8_t)(ACE2K_RFID_TRACE_RING - 1U); /* full: the next slot is the tail */
    ww.w.trace_tail = 0;
    ww.w.trace_lost = lost;
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.own_um_s[0] = 20000;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_drive_until(&ww, &lt, 1, ACE2K_RFID_EV_TAG, 3000, &e));
    struct ace2k_rfid_confirm_trace t;
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &t));
    ace2k_rfid_watch_drop_trace(&ww.w);
    struct ace2k_rfid_confirm_trace next;
    ASSERT_TRUE(ace2k_rfid_watch_peek_trace(&ww.w, &next));
    ASSERT_EQ(next.lost, 0); /* the count went with the trace sent */
    return t;
}

/* A trace that finds the ring full is counted, and the count goes with the next trace sent. */
TEST(a_trace_dropped_on_a_full_ring_is_counted_and_told_with_the_next_one)
{
    ASSERT_EQ(confirm_with_full_traces(0).lost, 4);
}

/* The count of the lost traces stops at 255. */
TEST(the_count_of_lost_traces_saturates)
{
    ASSERT_EQ(confirm_with_full_traces(253).lost, 255);
}

/* A change that the lane did not make (here the lane stands still) is traced as rejected, and the
 * confirmation ends. */
TEST(a_change_while_the_lane_stands_is_traced_as_the_wrong_way)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    confirm_running_on_lane1(&ww, &lt);
    ww.in.moving = 0;
    lt.t->absent = true;
    ww_steps(&ww, 30);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    struct ace2k_rfid_confirm_trace t;
    ASSERT_TRUE(ww_trace(&ww, ACE2K_RFID_TRACE_WRONG_WAY, &t));
    ASSERT_EQ(t.lane, 0);
    ASSERT_EQ(t.travel_um, ww.in.travel_um[0]);
}

/* The feed's load of the lane ends mid-confirmation: the confirmation ends with it. */
TEST(the_load_ending_clears_the_confirmation)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    confirm_running_on_lane1(&ww, &lt);
    ww.in.loading = 0;
    ww_drive(&ww, &lt, 1, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0);
    ww.own_um_s[0] = -9000; /* the lane still goes back and forth: it is not a load any more */
    ww_drive(&ww, &lt, 1, 100);
    ww.own_um_s[0] = 9000;
    nothing_follows(&ww, &lt);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_PENDING);
}

/* The strand of the lane leaves the insert sensor mid-confirmation: the confirmation ends, and the
 * lane forgets everything. */
TEST(the_strand_leaving_clears_the_confirmation)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    confirm_running_on_lane1(&ww, &lt);
    ww.in.insert = 0x02;
    ww_drive(&ww, &lt, 1, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_UNKNOWN);
    nothing_follows(&ww, &lt);
}

/* The neighbour comes to hold the candidate's UID mid-confirmation (here a UID it keeps from a
 * session that ended without a read).  Rule 1 decides: the UID is never this lane's, so the
 * confirmation ends. */
TEST(the_neighbour_taking_the_candidate_clears_the_confirmation)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ua, false);
    struct lanetag lt = { t, 0, 100000, 216000 };
    confirm_running_on_lane1(&ww, &lt);
    ww.w.l[1].tag = (struct ace2k_iso14443a_tag){ .uid_len = 4, .sak = 0x08 };
    memcpy(ww.w.l[1].tag.uid, ua, 4);
    ww.w.l[1].has_tag = true;
    ww_drive(&ww, &lt, 1, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
    /* The lane is driven on; its own UID in view, the neighbour may reopen a session on it, but
     * this lane never gets it and no confirmation comes back. */
    for (int i = 0; i < 300; i++) {
        ww_drive(&ww, &lt, 1, 1);
        struct ace2k_rfid_event e;
        while (ww_next(&ww, ACE2K_RFID_EV_TAG, &e)) {
            ASSERT_EQ(e.lane, 1);
        }
        ASSERT_TRUE((ace2k_rfid_watch_confirm_bits(&ww.w) & 0x01U) == 0U);
    }
    ASSERT_TRUE(!ww.w.l[0].has_tag);
}

/* Lane 4 loads with nothing of its own in view, and lane 3's tag t comes into view: a confirmation
 * runs on lane 4, asking it back. */
static void confirm_running_on_lane4(struct wworld *ww, struct ftag *t)
{
    ww_drive(ww, NULL, 0, 30);
    t->absent = false;
    for (int i = 0; i < 50 && ace2k_rfid_watch_confirm_bits(&ww->w) == 0; i++) {
        ww_drive(ww, NULL, 0, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww->w), 0x08);
    ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww->w), 0x08);
}

/* The neighbour's spool is lifted off while lane 4 goes back, and hung again
 * once lane 4's forward leg has passed the window of that change.  The tag came back at another
 * lane position, so it did not follow lane 4, and no later confirmation makes it lane 4's. */
TEST(a_spool_lifted_and_hung_again_away_from_the_window_is_not_attributed)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *other = tag_a(&ww, ub, false);
    ww.in.insert = 0x0C;
    ww_steps(&ww, 1);
    ww.in.loading = 0x08;
    ww.own_um_s[3] = 20000;
    confirm_running_on_lane4(&ww, other);
    ww_drive(&ww, NULL, 0, 50);
    other->absent = true; /* lifted off while lane 4 goes back */
    for (int i = 0; i < 100 && ace2k_rfid_watch_confirm_reverse_bits(&ww.w) != 0; i++) {
        ww_drive(&ww, NULL, 0, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x08); /* asked forward now */
    int32_t lifted_at = ww.in.travel_um[3];
    while (ww.in.travel_um[3] < lifted_at + ww.w.cfg.window_um + 10000) {
        ww_drive(&ww, NULL, 0, 1);
    }
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0x08); /* the leg has not run out */
    other->absent = false;                                 /* hung again */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_drive_until(&ww, NULL, 0, ACE2K_RFID_EV_TAG, 3000, &e));
    ASSERT_TRUE(!ww.w.l[3].has_tag && !ww.w.l[2].has_tag);
    struct ace2k_rfid_confirm_trace t;
    ASSERT_TRUE(ww_trace(&ww, ACE2K_RFID_TRACE_OUTSIDE, &t)); /* the change came elsewhere */
    ASSERT_EQ(t.lane, 3);
}

/* The tag on a spool that turns with a lane: in view over the first window_um of each rev_um of
 * that lane's travel. */
static void spin_tag(struct ftag *t, int32_t travel_um, int32_t window_um, int32_t rev_um)
{
    int32_t at = travel_um % rev_um;
    if (at < 0) {
        at += rev_um;
    }
    t->absent = (bool)(at >= window_um);
}

/* Lane 3, set by hand, runs a swap at 50 or 60 mm/s while lane 4 loads.  Lane
 * 3's tag (a 116 mm window on the shortest spool, 432 mm round) comes and goes on lane 3's time,
 * so its changes never come twice within the window of lane 4's travel with the tag in view.  It
 * is never lane 4's. */
TEST(a_neighbours_tag_turning_fast_is_never_attributed_to_the_loading_lane)
{
    for (int32_t speed = 50000; speed <= 60000; speed += 10000) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *t3 = tag_a(&ww, ub, false);
        ww.in.insert = 0x0C;
        ww_steps(&ww, 1);
        ace2k_rfid_watch_set_by_hand(&ww.w, 2);
        ww.in.loading = 0x08;
        ww.own_um_s[3] = 20000;
        ww.own_um_s[2] = speed;
        int confirms = 0;
        for (int i = 0; i < 6000; i++) {
            spin_tag(t3, ww.in.travel_um[2], 116000, 432000);
            uint8_t before = ace2k_rfid_watch_confirm_bits(&ww.w);
            ww_drive(&ww, NULL, 0, 1);
            if (before == 0 && ace2k_rfid_watch_confirm_bits(&ww.w) != 0) {
                confirms++;
            }
            struct ace2k_rfid_event e;
            ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
        }
        ASSERT_TRUE(confirms > 0); /* lane 4 did try the tag */
        ASSERT_TRUE(!ww.w.l[3].has_tag);
    }
}

/* A tag hung on bay 3 with no strand sits at the edge of the field, and drops
 * out and back every 5 s, whatever lane 4 does.  Lane 4, loading, reverses for it each time, but
 * its changes come 45 mm of lane 4's travel apart: it never follows. */
TEST(a_tag_at_the_edge_dropping_out_and_back_is_not_attributed)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *edge = tag_a(&ww, ub, false);
    ww.in.insert = 0x08;
    ww_steps(&ww, 1);
    ww.in.loading = 0x08;
    ww.own_um_s[3] = 20000;
    ww_drive(&ww, NULL, 0, 30);
    int confirms = 0;
    for (int i = 0; i < 4000; i++) {
        if (i % 500 == 0) {
            edge->absent = (bool)!edge->absent;
        }
        uint8_t before = ace2k_rfid_watch_confirm_bits(&ww.w);
        ww_drive(&ww, NULL, 0, 1);
        if (before == 0 && ace2k_rfid_watch_confirm_bits(&ww.w) != 0) {
            confirms++;
        }
        struct ace2k_rfid_event e;
        ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    }
    ASSERT_TRUE(confirms > 0);
    ASSERT_TRUE(!ww.w.l[3].has_tag);
}

/* Lane 1 moves alone outside a load, with tag t in view, and lane 2 has a strand in. */
static void passive_lane1(struct wworld *ww, struct ftag **t)
{
    *t = tag_b(ww, ua, true);
    ww->in.insert = 0x03;
    ww_steps(ww, 1);
    ww->in.moving = 0x01;
    ww_steps(ww, 30);
}

/* The tag leaves while lane 2 moves too.  Lane 2's spool may have turned it
 * away, so no leave is recorded for lane 1, and its return after a full turn of lane 1, lane 2
 * still by then, is nobody's. */
TEST(a_leave_while_the_neighbour_moves_is_not_recorded_for_passive_watching)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = NULL;
    passive_lane1(&ww, &t);
    ww.in.moving = 0x03;
    t->absent = true;
    ww_steps(&ww, 40); /* it left, both lanes moving */
    ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].gone_count, 0);
    ww.in.moving = 0x01;
    ww_travel(&ww, 0, ACE2K_RFID_PASSIVE_MIN_UM, 30);
    t->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
}

/* The tag leaves with lane 2 still, then lane 2 moves before it comes back.
 * Lane 2's motion drops lane 1's record, so the return, lane 2 still again, is nobody's. */
TEST(the_neighbour_moving_between_the_leave_and_the_return_drops_the_record)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = NULL;
    passive_lane1(&ww, &t);
    t->absent = true;
    ww_steps(&ww, 40); /* it left, lane 2 still */
    ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].gone_count, 1);
    ww.in.moving = 0x03;
    ww_steps(&ww, 1); /* lane 2 moves for one run */
    ww.in.moving = 0x01;
    ASSERT_EQ(ww.w.r[ACE2K_RFID_READER_B].gone_count, 0);
    ww_travel(&ww, 0, ACE2K_RFID_PASSIVE_MIN_UM, 30);
    t->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_TRUE(!ww.w.l[0].has_tag);
}

/* A return after more than cfg.passive_max_um of lane 1's travel is no single
 * turn of its spool, so it is nobody's; one just within it is lane 1's. */
TEST(a_return_past_the_passive_maximum_is_nobodys)
{
    for (int past = 0; past < 2; past++) {
        struct wworld ww;
        ww_init(&ww);
        struct ftag *t = NULL;
        passive_lane1(&ww, &t);
        t->absent = true;
        ww_steps(&ww, 40); /* it left */
        int32_t over = past != 0 ? 1000 : -1000;
        ww_travel(&ww, 0, ww.w.cfg.passive_max_um + over, 30);
        t->absent = false;
        struct ace2k_rfid_event e;
        ASSERT_EQ(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e), past == 0);
        ASSERT_EQ(ww.w.l[0].has_tag, past == 0);
    }
}

/* The host reads lane 1 with motion while it has lane 1 in use.  A candidate
 * comes into view, but nothing in this watch moves an in-use lane: no confirmation, so the feed
 * gets no reverse leg.  And a confirmation that runs ends once the lane is in use. */
TEST(no_confirmation_runs_on_a_lane_the_host_has_in_use)
{
    struct wworld ww;
    ww_init(&ww);
    struct lanetag lt = { tag_b(&ww, ua, false), 0, 100000, 216000 };
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01; /* the read with motion's search */
    ww.own_um_s[0] = 20000;
    for (int i = 0; i < 1500; i++) {
        ace2k_rfid_watch_set_in_use(&ww.w, 0x01); /* the host's refresh */
        ww_drive(&ww, &lt, 1, 1);
        ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
        ASSERT_EQ(ace2k_rfid_watch_confirm_reverse_bits(&ww.w), 0);
    }
    ASSERT_TRUE(!ww.w.l[0].has_tag);
    /* the same without the mask: a confirmation runs; the mask then ends it */
    struct wworld w2;
    ww_init(&w2);
    struct lanetag lt2 = { tag_b(&w2, ua, false), 0, 100000, 216000 };
    confirm_running_on_lane1(&w2, &lt2);
    ace2k_rfid_watch_set_in_use(&w2.w, 0x01);
    ww_drive(&w2, &lt2, 1, 1);
    ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&w2.w), 0);
}

/* A load runs at most ACE2K_RFID_CONFIRM_PER_LOAD_MAX confirmations.  A tag at
 * the edge that never follows would otherwise make the lane go back and forth for the whole load;
 * past the cap the lane is left to the tie-break.  The next load starts the count again. */
TEST(a_load_runs_at_most_the_capped_number_of_confirmations)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *edge = tag_a(&ww, ub, false);
    ww.in.insert = 0x08;
    ww_steps(&ww, 1);
    for (int load = 0; load < 2; load++) {
        ww.in.loading = 0x08;
        ww.own_um_s[3] = 20000;
        ww_drive(&ww, NULL, 0, 30);
        for (int i = 0; i < 9000; i++) {
            if (i % 500 == 0) {
                edge->absent = (bool)!edge->absent;
            }
            ww_drive(&ww, NULL, 0, 1);
            if (i >= 7000) { /* the cap is reached by then: no confirmation any more */
                ASSERT_EQ(ace2k_rfid_watch_confirm_bits(&ww.w), 0);
            }
        }
        ASSERT_EQ(ww.w.l[3].confirms, ACE2K_RFID_CONFIRM_PER_LOAD_MAX);
        ww.in.loading = 0; /* the load ends */
        ww.own_um_s[3] = 0;
        ww_drive(&ww, NULL, 0, 5);
    }
}

/* Passive watching measures the travel both ways: a lane rolled back a full turn (a host rollback)
 * gets its tag that leaves and comes back, as one fed forward does. */
TEST(a_return_after_a_full_turn_backwards_is_the_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = NULL;
    passive_lane1(&ww, &t);
    ww.in.reverse = 0x01;
    t->absent = true;
    ww_steps(&ww, 40); /* it left */
    ww_travel(&ww, 0, -ACE2K_RFID_PASSIVE_MIN_UM, 30);
    t->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
