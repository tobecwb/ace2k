#include "test.h"
#include "rfid/watch.h"
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
    int configure_rc[2]; /* 0, or the probe's error: the soft reset never completed */
    struct ace2k_rfid_watch_inputs in;
    uint32_t now;
    int max_iso_calls, max_configures;
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
    ww->field[r] = on;
    if (ww->stuck_on[r]) {
        ww->field[r] = true;
    }
}

static bool ww_field_driven(void *ctx, uint8_t r)
{
    return ((struct wworld *)ctx)->field[r];
}

static const struct ace2k_rfid_watch_ops ace2k_ww_ops = {
    .configure = ww_configure,
    .needs_configure = ww_needs,
    .field_set = ww_field_set,
    .field_driven = ww_field_driven,
};

static void ww_init(struct wworld *ww)
{
    memset(ww, 0, sizeof *ww);
    ace2k_iso14443a_init(&ww->iso, &ace2k_fld_ops, &ww->ff);
    ace2k_rfid_watch_init(&ww->w, &ww->iso, &ace2k_ww_ops, ww);
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

static const uint8_t ace2k_ua[4] = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t ace2k_ub[4] = { 0x11, 0x22, 0x33, 0x44 };
static const uint8_t ace2k_uc[7] = { 0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
static const uint8_t ace2k_key1[6] = { 1, 1, 2, 3, 4, 5 }; /* fake_field's sector-1 key A */

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

TEST(the_load_reads_a_tag_that_enters_while_the_lane_moves)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t = tag_b(&ww, ace2k_ua, false);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.in.moving = 0x01;
    ww_steps(&ww, 20); /* the field on, inventories of an empty field: the baseline */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_PENDING);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_STATE, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_SEARCHING);
    t->absent = false; /* the spool turned the tag in front of the antenna */
    ASSERT_TRUE(ww_until(&ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(e.tag.uid_len, 4);
    ASSERT_TRUE(memcmp(e.tag.uid, ace2k_ua, 4) == 0);
    ASSERT_EQ(e.tag.sak, 0x08);
    uint8_t session = e.session;
    ASSERT_TRUE(session != 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&ww.w, 0), ACE2K_RFID_READING);
    ASSERT_EQ(ace2k_rfid_watch_hold_bits(&ww.w), 0x01);
    ww.in.moving = 0; /* the feed pauses the lane on the hold */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x7,
                                            ace2k_key1, ww.now),
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

TEST(the_neighbours_resting_tag_is_never_taken_and_a_new_one_is)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ace2k_ub, true); /* in the field from the start: lane 2's spool, at rest */
    struct ftag *mine = tag_b(&ww, ace2k_ua, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01; /* lane 1 alone */
    ww_steps(&ww, 60);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    mine->absent = false;
    ww_steps(&ww, 30);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ace2k_ua, 4) == 0);
}

TEST(with_the_neighbour_empty_a_resting_tag_is_the_lanes)
{
    struct wworld ww;
    ww_init(&ww);
    tag_b(&ww, ace2k_ub, true);
    ww.in.insert = 0x01; /* lane 2 has no strand */
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ace2k_ub, 4) == 0);
}

/* Lane 2 reads its own tag first (moving alone, its tag entering), so that lane 1 can be judged
 * with a read neighbour. */
static uint8_t read_lane2(struct wworld *ww, struct ftag *t2)
{
    ww->in.moving = 0x02;
    ww_steps(ww, 20);
    t2->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    ASSERT_EQ(e.lane, 1);
    ASSERT_EQ(ace2k_rfid_watch_done(&ww->w, 1, e.session, true), 0);
    drain(ww);
    return e.session;
}

TEST(with_both_moving_a_new_tag_is_the_lanes_only_when_the_neighbour_is_read)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *y = tag_b(&ww, ace2k_ua, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    ww.in.moving = 0x03;
    ww_steps(&ww, 20);
    y->absent = false;
    ww_steps(&ww, 40);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(!ww_next(&ww, ACE2K_RFID_EV_TAG, &e)); /* both pending: whose? ignored */
    y->absent = true;
    struct ftag *z = tag_b(&ww, ace2k_ub, false);
    (void)read_lane2(&ww, z);
    ww.in.moving = 0x03;
    ww_steps(&ww, 20);
    y->absent = false;
    ww_steps(&ww, 40);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ace2k_ua, 4) == 0);
}

TEST(a_read_on_command_attributes_by_the_neighbour_or_answers_ambiguous)
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
    tag_b(&ww, ace2k_ub, true);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0));
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ASSERT_TRUE(ace2k_rfid_watch_lane_busy(&ww.w, 0)); /* the probe pending: a MOVE=1 is busy */
    ww_steps(&ww, 30);                                 /* lane 2 empty: the tag is lane 1's */
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_BUSY); /* reading */
    ASSERT_TRUE(ace2k_rfid_watch_lane_busy(&ww.w, 0));                 /* the session open */
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 1));
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 4));
    ASSERT_EQ(ace2k_rfid_watch_done(&ww.w, 0, e.session, false), 0);
    ASSERT_TRUE(!ace2k_rfid_watch_lane_busy(&ww.w, 0)); /* closed: a MOVE=1 may start */
    drain(&ww);
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

TEST(a_read_on_command_with_a_read_neighbour_takes_the_other_tag)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *z = tag_b(&ww, ace2k_ub, false);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    (void)read_lane2(&ww, z); /* lane 2 read: its tag stays in the field */
    ww.in.moving = 0;
    ww_steps(&ww, 5);
    tag_b(&ww, ace2k_ua, true);
    ASSERT_EQ(ace2k_rfid_watch_probe(&ww.w, 0), ACE2K_RFID_READ_OK);
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
    ASSERT_EQ(e.lane, 0);
    ASSERT_TRUE(memcmp(e.tag.uid, ace2k_ua, 4) == 0);
}

static uint8_t open_session_on_lane1(struct wworld *ww, struct ftag **t)
{
    *t = tag_b(ww, ace2k_ua, false);
    ww->in.insert = 0x01;
    ww_steps(ww, 1);
    ww->in.moving = 0x01;
    ww_steps(ww, 20);
    (*t)->absent = false;
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_until(ww, ACE2K_RFID_EV_TAG, 30, &e));
    drain(ww);
    return e.session;
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
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1,
                                            ace2k_key1, ww.now),
              -ACE2K_EREFUSED); /* a stale host: ignored */
}

TEST(a_tag_gone_during_a_step_ends_the_session_lost_and_its_return_reopens_it)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    t->absent = true; /* the lane coasted past it */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1,
                                            ace2k_key1, ww.now),
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
    struct ftag *t = tag_b(&ww, ace2k_ua, false);
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.loading = 0x01;
    ww.in.moving = 0x01;
    ww_steps(&ww, 20);
    t->absent = false;
    ww_steps(&ww, 30);
    struct ace2k_rfid_event e;
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_TAG, &e));
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
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 1,
                                            ace2k_key1, ww.now),
              0);
    ww_steps(&ww, 1);
    const uint8_t *k = ww.w.r[ACE2K_RFID_READER_B].step_job.key;
    ASSERT_TRUE(k[0] == 0 && k[1] == 0 && k[2] == 0 && k[3] == 0 && k[4] == 0 && k[5] == 0);
    /* a step out of bounds is a host bug */
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, 3, 1, 1, ace2k_key1, ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 16, 1,
                                            ace2k_key1, ww.now),
              -ACE2K_EINVAL);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_NTAG, 4, 4, ace2k_key1,
                                            ww.now),
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
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1,
                                            ace2k_key1, ww.now),
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
    /* A reader that never answers does not delay the lane's speed loop: the lane's speed loop runs in the 10 ms tick, the timer interrupt,
     * and the watch runs in the rfid task, so a watch step cannot delay it by construction;
     * what the task can do is loop or wait on a mute reader — this shows it does neither: every
     * run returns, and each reader sees at most one start and one poll per run, whatever the
     * number of runs. */
    struct wworld ww;
    ww_init(&ww);
    ww.ff.rd[ACE2K_RFID_READER_B].mute = true;
    tag_b(&ww, ace2k_ua, true); /* a tag there, never heard */
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
    struct ftag *t = add_ntag(&ww.ff.rd[1], ace2k_uc);
    t->absent = true;
    ww.in.insert = 0x01;
    ww_steps(&ww, 1);
    ww.in.moving = 0x01;
    ww_steps(&ww, 20);
    t->absent = false;
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
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x1,
                                            ace2k_key1, ww.now),
              0);
    ww_steps(&ww, 20);
    ASSERT_TRUE(ww_next(&ww, ACE2K_RFID_EV_DATA, &e));
    ASSERT_EQ(e.kind, ACE2K_RFID_DATA_OK);
    ASSERT_EQ(e.session, session);
}

TEST(a_session_closed_under_a_running_step_opens_the_neighbours_only_after_it_and_no_bytes_cross)
{
    struct wworld ww;
    ww_init(&ww);
    struct ftag *t;
    uint8_t session = open_session_on_lane1(&ww, &t);
    tag_b(&ww, ace2k_ub, true);
    ww.in.insert = 0x03;
    ww_steps(&ww, 1);
    drain(&ww);
    ASSERT_EQ(ace2k_rfid_watch_step_request(&ww.w, 0, session, ACE2K_RFID_OP_MIFARE_A, 1, 0x7,
                                            ace2k_key1, ww.now),
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
    tag_b(&ww, ace2k_ua, true);
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

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
