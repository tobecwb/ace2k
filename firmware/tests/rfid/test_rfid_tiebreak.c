#include "test.h"
#include "rfid/watch.h"
#include "rfid/watch_tiebreak.h"
#include "fake_field.h"

/* The world: the fake field of each reader behind the real iso14443a engine and the real watch,
 * the real tie-break scheduler, and a fake feed that runs the moves of a read with motion as the
 * binding would start them.  Lanes 3-4 (2, 3) are reader A's (index 0), lanes 1-2 reader B's.
 * The fake feed.  A move goes forward at the confirmation's speed until its reach, the
 * confirmation's longest leg as the feed gives a tie-break, then back to where it began at
 * FWD_UM_S.  A tie-break that runs its reach is not exhausted: the lane stays pending.  A lane
 * held at the head goes back once it has pushed held_um: its buffer is full.  While the watch
 * asks a confirmation, the lane runs the asked leg at cfg.speed_um_s.  A back leg never goes behind
 * where the move began: it is out of room, as the feed reports.  A lane whose tag the watch
 * attributed goes back.  The test answers the session at once with a read. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct tbworld {
    struct fake_field ff;
    struct ace2k_iso14443a iso;
    struct ace2k_rfid_watch w;
    struct ace2k_rfid_tiebreak tb;
    bool field[ACE2K_RFID_READER_COUNT];
    struct ace2k_rfid_watch_inputs in;
    uint32_t now;
    /* the host: the mask it refreshes every second while mask_on, and the scheduler as the binding
     * runs it while sched_on */
    bool mask_on, sched_on;
    uint8_t mask;
    uint32_t t_mask;
    /* the fake feed */
    uint8_t phase[ACE2K_LANE_COUNT]; /* 0 idle, 1 forward, 2 back */
    int32_t from_um[ACE2K_LANE_COUNT], reach_um[ACE2K_LANE_COUNT], held_um[ACE2K_LANE_COUNT];
    int32_t leg_from_um[ACE2K_LANE_COUNT];
    uint8_t confirm_last, reverse_last;
    bool refuse; /* the feed refuses the next starts as busy */
    int starts[ACE2K_LANE_COUNT], attempts[ACE2K_LANE_COUNT];
    int32_t max_um[ACE2K_LANE_COUNT]; /* the farthest a lane went past where its move began */
    /* the tags the watch attributed, by lane: the UID's first byte, 0 when none */
    uint8_t got[ACE2K_LANE_COUNT];
    int tags_out;
};

#define FWD_UM_S      20000
#define TB_STEPS_2S   200 /* task runs in 2 s */
#define PHASE_IDLE    0U
#define PHASE_FORWARD 1U
#define PHASE_BACK    2U

/* A tag on a lane's spool: in view while that lane's travel is within [in_lo_um, in_hi_um]. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct spooltag {
    struct ftag *t;
    uint8_t lane;
    int32_t in_lo_um, in_hi_um;
};

static int tb_configure(void *ctx, uint8_t r)
{
    ((struct tbworld *)ctx)->field[r] = false;
    return 0;
}

static bool tb_needs(void *ctx, uint8_t r)
{
    (void)ctx;
    (void)r;
    return false;
}

static void tb_field_set(void *ctx, uint8_t r, bool on)
{
    ((struct tbworld *)ctx)->field[r] = on;
}

static bool tb_field_driven(void *ctx, uint8_t r)
{
    return ((struct tbworld *)ctx)->field[r];
}

static const struct ace2k_rfid_watch_ops tb_ops = {
    .configure = tb_configure,
    .needs_configure = tb_needs,
    .field_set = tb_field_set,
    .field_driven = tb_field_driven,
};

static void tb_init(struct tbworld *tw)
{
    memset(tw, 0, sizeof *tw);
    ace2k_iso14443a_init(&tw->iso, &fld_ops, &tw->ff);
    ace2k_rfid_watch_init(&tw->w, &tw->iso, &tb_ops, tw);
    tw->in.link_ok = true;
    tw->now = 1000;
    tw->sched_on = true;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        tw->reach_um[lane] = tw->w.cfg.leg_max_um;
    }
}

/* The fake feed's start of a tie-break: the binding's start function. */
static enum ace2k_rfid_read_refusal tb_start(void *ctx, uint8_t lane)
{
    struct tbworld *tw = ctx;
    tw->attempts[lane]++;
    if (tw->refuse) {
        return ACE2K_RFID_READ_BUSY;
    }
    tw->starts[lane]++;
    tw->phase[lane] = PHASE_FORWARD;
    tw->from_um[lane] = tw->in.travel_um[lane];
    return ACE2K_RFID_READ_OK;
}

static uint8_t bit_of_lane(uint8_t lane)
{
    return (uint8_t)(1U << lane);
}

/* The forward phase of a lane for 10 ms: its speed, and whether the move turns back. */
static int32_t tb_forward(struct tbworld *tw, uint8_t lane, uint8_t conf, uint8_t rev)
{
    uint8_t bit = bit_of_lane(lane);
    int32_t pushed = tw->in.travel_um[lane] - tw->from_um[lane];
    uint8_t state = ace2k_rfid_watch_state(&tw->w, lane);
    bool done = (bool)(state == ACE2K_RFID_READING || state == ACE2K_RFID_READ);
    if (done || (tw->held_um[lane] > 0 && pushed >= tw->held_um[lane]) ||
        pushed >= tw->reach_um[lane]) {
        tw->phase[lane] = PHASE_BACK;
        return 0;
    }
    if ((conf & bit) == 0U) {
        return tw->w.cfg.speed_um_s;
    }
    bool r = (rev & bit) != 0U;
    bool was_rev = (tw->reverse_last & bit) != 0U;
    if ((tw->confirm_last & bit) == 0U || r != was_rev) {
        tw->leg_from_um[lane] = tw->in.travel_um[lane]; /* a new leg */
    }
    int32_t leg = tw->in.travel_um[lane] - tw->leg_from_um[lane];
    bool out = (bool)(leg >= tw->w.cfg.leg_max_um || leg <= -tw->w.cfg.leg_max_um);
    if (r && pushed <= 0) {
        out = true; /* no room behind where the move began */
    }
    if (out) {
        tw->in.confirm_out |= bit;
        return 0;
    }
    return r ? -tw->w.cfg.speed_um_s : tw->w.cfg.speed_um_s;
}

/* The back phase of a lane for 10 ms: its speed, 0 once it is where its move began. */
static int32_t tb_back(struct tbworld *tw, uint8_t lane)
{
    if (tw->in.travel_um[lane] - (FWD_UM_S / 100) <= tw->from_um[lane]) {
        tw->in.travel_um[lane] = tw->from_um[lane]; /* back exactly where it began */
        tw->phase[lane] = PHASE_IDLE;
        return 0;
    }
    return -FWD_UM_S;
}

/* The mask with the bit set or cleared. */
static uint8_t with_bit(uint8_t mask, uint8_t bit, bool on)
{
    return (uint8_t)(on ? (mask | bit) : (mask & (uint8_t)~bit));
}

/* The fake feed for 10 ms. */
static void tb_feed(struct tbworld *tw)
{
    uint8_t conf = ace2k_rfid_watch_confirm_bits(&tw->w);
    uint8_t rev = ace2k_rfid_watch_confirm_reverse_bits(&tw->w);
    tw->in.confirm_out = 0;
    tw->in.confirm_out_epoch = ace2k_rfid_watch_confirm_epoch_bits(&tw->w); /* seen at once */
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        uint8_t bit = bit_of_lane(lane);
        if (tw->phase[lane] == PHASE_IDLE) {
            continue; /* not the feed's: the test sets its bits */
        }
        int32_t v = 0;
        if (tw->phase[lane] == PHASE_FORWARD) {
            v = tb_forward(tw, lane, conf, rev);
        } else {
            v = tb_back(tw, lane);
        }
        tw->in.travel_um[lane] += v / 100;
        int32_t past = tw->in.travel_um[lane] - tw->from_um[lane];
        if (past > tw->max_um[lane]) {
            tw->max_um[lane] = past;
        }
        tw->in.moving = with_bit(tw->in.moving, bit, v != 0);
        tw->in.reverse = with_bit(tw->in.reverse, bit, v < 0);
        tw->in.loading = with_bit(tw->in.loading, bit, tw->phase[lane] != PHASE_IDLE);
    }
    tw->confirm_last = conf;
    tw->reverse_last = rev;
}

/* The host's side: a session is answered at once with a read, and the attributed UID noted. */
static void tb_host(struct tbworld *tw)
{
    struct ace2k_rfid_event e;
    while (ace2k_rfid_watch_peek_event(&tw->w, &e)) {
        ace2k_rfid_watch_drop_event(&tw->w);
        if (e.type == ACE2K_RFID_EV_TAG) {
            tw->got[e.lane] = e.tag.uid[0];
            tw->tags_out++;
            (void)ace2k_rfid_watch_done(&tw->w, e.lane, e.session, true);
        }
    }
    if (tw->mask_on && tw->now - tw->t_mask >= 1000U) {
        ace2k_rfid_watch_set_in_use(&tw->w, tw->mask);
        tw->t_mask = tw->now;
    }
}

/* n task runs of 10 ms: the feed, the watch step, the host, then the scheduler as the binding runs
 * it. */
static void tb_steps(struct tbworld *tw, const struct spooltag *tags, int ntags, int n)
{
    for (int s = 0; s < n; s++) {
        tb_feed(tw);
        for (int i = 0; i < ntags; i++) {
            int32_t at = tw->in.travel_um[tags[i].lane];
            tags[i].t->absent = (bool)(at < tags[i].in_lo_um || at > tags[i].in_hi_um);
        }
        tw->now += 10;
        ace2k_rfid_watch_step(&tw->w, tw->now, &tw->in);
        tb_host(tw);
        ace2k_rfid_tiebreak_note(&tw->tb, &tw->w, &tw->in, tw->now);
        if (!tw->sched_on) {
            continue;
        }
        uint8_t idle = 0;
        for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
            if (tw->phase[lane] == PHASE_IDLE && (tw->in.moving & bit_of_lane(lane)) == 0U) {
                idle |= bit_of_lane(lane);
            }
        }
        int8_t lane = ace2k_rfid_tiebreak_pick(&tw->tb, &tw->w, idle, tw->now);
        if (lane >= 0 && tb_start(tw, (uint8_t)lane) == ACE2K_RFID_READ_OK) {
            ace2k_rfid_tiebreak_tried(&tw->tb, (uint8_t)lane);
        }
    }
}

/* The host sends its mask now, and then every second. */
static void tb_mask(struct tbworld *tw, uint8_t mask)
{
    tw->mask = mask;
    tw->mask_on = true;
    tw->t_mask = tw->now;
    ace2k_rfid_watch_set_in_use(&tw->w, mask);
}

static const uint8_t green[4] = { 0x61, 0x02, 0x03, 0x04 };
static const uint8_t black[4] = { 0x71, 0x22, 0x33, 0x44 };

/* A tag in reader A's field (lanes 3–4). */
static struct ftag *tag_a(struct tbworld *tw, const uint8_t uid[4])
{
    return add_mifare(&tw->ff.rd[ACE2K_RFID_READER_A], uid);
}

/* After power-on: lanes 3 and 4 have strands, pending, and the host's mask says none is in use. */
static void lanes_3_and_4_pending(struct tbworld *tw)
{
    tw->in.insert = 0x0C;
    tb_steps(tw, NULL, 0, 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw->w, 2), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw->w, 3), ACE2K_RFID_PENDING);
    tb_mask(tw, 0);
}

/* S5.  Lane 3's tip sits in the head, its green tag in view; lane 4's black tag is away.  Lane 3
 * goes first: its buffer fills, it goes back exactly what it pushed, the green tag never changed,
 * and lane 3 stays pending.  Then lane 4 moves: the black tag appears, follows its reversal, and is
 * lane 4's. */
TEST(s5_a_lane_held_at_the_head_stays_pending_and_the_other_gets_its_tag)
{
    struct tbworld tw;
    tb_init(&tw);
    struct spooltag tags[2] = {
        { tag_a(&tw, green), 2, -500000, 500000 }, /* the head holds the tip: always in view */
        { tag_a(&tw, black), 3, 60000, 176000 },
    };
    tw.held_um[2] = 20000;
    lanes_3_and_4_pending(&tw);
    tb_steps(&tw, tags, 2, TB_STEPS_2S + 20);
    ASSERT_EQ(tw.starts[2], 1); /* the lower lane first */
    ASSERT_EQ(tw.starts[3], 0);
    tb_steps(&tw, tags, 2, 3000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(tw.starts[3], 1);
    ASSERT_TRUE(tw.max_um[2] <= 20000 + (FWD_UM_S / 100));
    ASSERT_EQ(tw.in.travel_um[2], 0); /* back exactly what it pushed */
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    ASSERT_EQ(tw.got[2], 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 3), ACE2K_RFID_READ);
    ASSERT_EQ(tw.got[3], black[0]);
    ASSERT_EQ(tw.tags_out, 1);
}

/* S6.  As S5 with no tag in view: lane 3 held, nothing; lane 4 moved, the black tag appears and
 * follows, and it is lane 4's.  Lane 3 stays pending. */
TEST(s6_no_tag_in_view_the_moved_lane_gets_the_tag_that_appears)
{
    struct tbworld tw;
    tb_init(&tw);
    struct spooltag tags[1] = { { tag_a(&tw, black), 3, 60000, 176000 } };
    tw.held_um[2] = 20000;
    lanes_3_and_4_pending(&tw);
    tb_steps(&tw, tags, 1, 4000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(tw.starts[3], 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 3), ACE2K_RFID_READ);
    ASSERT_EQ(tw.got[3], black[0]);
    ASSERT_EQ(tw.tags_out, 1);
}

/* S8.  Lane 3 pending with its green tag in view, and the black tag of a spool hung on bay 4 with
 * no strand, in view too.  Lane 3's tip is in the head: it is held, goes back, and nothing is
 * attributed.  Inconclusive: lane 3 stays pending and is not tried again. */
TEST(s8_held_at_the_head_with_a_uid_in_view_is_inconclusive)
{
    struct tbworld tw;
    tb_init(&tw);
    struct spooltag tags[1] = { { tag_a(&tw, green), 2, -500000, 500000 } };
    (void)tag_a(&tw, black); /* the hung spool: in view, at rest */
    tw.held_um[2] = 20000;
    tw.in.insert = 0x04;
    tb_steps(&tw, tags, 1, 1);
    tb_mask(&tw, 0);
    tb_steps(&tw, tags, 1, 4000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(tw.starts[3], 0);
    ASSERT_EQ(tw.in.travel_um[2], 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    ASSERT_EQ(tw.tags_out, 0);
}

/* S9.  Two spools go in, lane 3 then lane 4, half a second apart.  Their loads yield to each other
 * and park short, with no tag seen.  The tie-break then moves lane 3: its green tag follows, it is
 * lane 3's.  Then lane 4: its black tag is lane 4's.  No swap. */
TEST(s9_two_insertions_close_together_end_with_both_identified_no_swap)
{
    struct tbworld tw;
    tb_init(&tw);
    struct spooltag tags[2] = {
        { tag_a(&tw, green), 2, 50000, 166000 },
        { tag_a(&tw, black), 3, 80000, 196000 },
    };
    tb_mask(&tw, 0);
    tw.sched_on = false; /* the automatic loads first */
    tw.in.insert = 0x04;
    tw.held_um[2] = 6000; /* each load stops short, its search not run */
    tw.phase[2] = PHASE_FORWARD;
    tb_steps(&tw, tags, 2, 50);
    tw.in.insert = 0x0C;
    tw.held_um[3] = 6000;
    tw.phase[3] = PHASE_FORWARD;
    tb_steps(&tw, tags, 2, 100); /* half a second later, and both back */
    tw.held_um[2] = 0;
    tw.held_um[3] = 0;
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 3), ACE2K_RFID_PENDING);
    ASSERT_EQ(tw.tags_out, 0);
    tw.sched_on = true;
    tb_steps(&tw, tags, 2, 4000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(tw.starts[3], 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_READ);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 3), ACE2K_RFID_READ);
    ASSERT_EQ(tw.got[2], green[0]);
    ASSERT_EQ(tw.got[3], black[0]);
    ASSERT_EQ(tw.tags_out, 2);
}

/* Review Focus 3.  No mask since the connection, a mask older than the host's refresh, or a link
 * drop since the mask: nothing moves on its own. */
TEST(no_tiebreak_without_a_valid_mask)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.held_um[2] = 20000; /* lane 3 stays pending after each try */
    tw.in.insert = 0x04;
    tb_steps(&tw, NULL, 0, 400);
    ASSERT_EQ(tw.attempts[2], 0); /* no mask yet */
    tb_mask(&tw, 0);
    tb_steps(&tw, NULL, 0, 2);
    ASSERT_EQ(tw.starts[2], 1); /* quiet long enough: it starts at once */
    tb_steps(&tw, NULL, 0, 300);
    /* The host stops refreshing.  1.5 s later a new strand goes in: its quiet time would end at
     * 3.5 s, after the mask went stale at 3 s. */
    tw.mask_on = false;
    tb_steps(&tw, NULL, 0, 150);
    tw.in.insert = 0;
    tb_steps(&tw, NULL, 0, 1);
    tw.in.insert = 0x04;
    tb_steps(&tw, NULL, 0, 600);
    ASSERT_EQ(tw.attempts[2], 1);
    uint8_t m = 0xFF;
    ASSERT_TRUE(!ace2k_rfid_watch_in_use(&tw.w, &m));
    /* The host refreshes again, then the link drops and comes back: the mask is gone until the
     * host sends one again. */
    tb_mask(&tw, 0);
    tw.in.link_ok = false;
    tb_steps(&tw, NULL, 0, 1);
    tw.in.link_ok = true;
    tw.mask_on = false;
    tb_steps(&tw, NULL, 0, 400);
    ASSERT_EQ(tw.attempts[2], 1);
    tb_mask(&tw, 0);
    tb_steps(&tw, NULL, 0, 2);
    ASSERT_EQ(tw.starts[2], 2);
}

/* A lane is tried once per insertion on its reader.  A start the feed refuses does not count.  A
 * strand going in at the neighbour lets both lanes be tried again; one going in on the other
 * reader does not. */
TEST(each_lane_is_tried_once_per_insertion)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.held_um[2] = 20000;
    tw.held_um[3] = 20000;
    tw.held_um[0] = 20000;
    tw.in.insert = 0x04;
    tb_mask(&tw, 0);
    tw.refuse = true;
    tb_steps(&tw, NULL, 0, TB_STEPS_2S + 50);
    ASSERT_TRUE(tw.attempts[2] > 1); /* refused: tried again */
    ASSERT_EQ(tw.starts[2], 0);
    tw.refuse = false;
    tb_steps(&tw, NULL, 0, 2);
    ASSERT_EQ(tw.starts[2], 1);
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    tw.in.insert = 0x05; /* a strand in lane 1, on the other reader */
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[0], 1);
    ASSERT_EQ(tw.starts[2], 1);
    tw.in.insert = 0x0D; /* a strand in lane 4, on lane 3's reader */
    tb_steps(&tw, NULL, 0, 3000);
    ASSERT_EQ(tw.starts[2], 2);
    ASSERT_EQ(tw.starts[3], 1);
    ASSERT_EQ(tw.starts[0], 1);
}

/* The conditions of the pick, one at a time, from a lane that may start. */
TEST(the_pick_waits_for_every_condition_and_takes_the_lower_lane)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.sched_on = false;
    tw.in.insert = 0x0C;
    tb_mask(&tw, 0);
    tb_steps(&tw, NULL, 0, TB_STEPS_2S + 10);
    const uint8_t all = 0x0F;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 2); /* the lower lane */
    /* A busy neighbour that is not in use stops both lanes. */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x0B, tw.now), -1); /* lane 3 busy */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x04, tw.now), -1); /* lane 4 busy */
    tb_mask(&tw, 0x08); /* lane 4 in use: lane 3 goes beside it, busy or not */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x04, tw.now), 2);
    tb_mask(&tw, 0x04); /* lane 3 in use: never picked, and lane 4 goes beside it */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x08, tw.now), 3);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x04, tw.now), -1); /* lane 4 busy */
    tb_mask(&tw, 0);
    ace2k_rfid_tiebreak_tried(&tw.tb, 2);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 3);
    ace2k_rfid_tiebreak_tried(&tw.tb, 3);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.tb.tried = 0;
    tw.w.r[ACE2K_RFID_READER_A].dead = true;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.w.r[ACE2K_RFID_READER_A].dead = false;
    tw.w.l[3].probe = true; /* a read on command on the neighbour */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.w.l[3].probe = false;
    tw.w.r[ACE2K_RFID_READER_A].session_open = true;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.w.r[ACE2K_RFID_READER_A].session_open = false;
    /* A lane of the reader moving, not in use, restarts the quiet time; one in use does not. */
    tw.in.moving = 0x08;
    tb_steps(&tw, NULL, 0, 1);
    tw.in.moving = 0;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tb_steps(&tw, NULL, 0, TB_STEPS_2S - 2);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tb_steps(&tw, NULL, 0, 2);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 2);
    tb_mask(&tw, 0x08);
    tw.in.moving = 0x08;
    tb_steps(&tw, NULL, 0, 1);
    tw.in.moving = 0;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 2);
    tb_mask(&tw, 0);
    /* The sensors unknown, the link down, a lane set by hand. */
    tw.in.insert_unknown = true;
    tb_steps(&tw, NULL, 0, 1);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.in.insert_unknown = false;
    tb_steps(&tw, NULL, 0, 1);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 2);
    tw.in.link_ok = false;
    tb_steps(&tw, NULL, 0, 1);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
    tw.in.link_ok = true;
    tb_steps(&tw, NULL, 0, 1);
    tb_mask(&tw, 0); /* the drop took the mask with it: the host sends it again */
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 2);
    ace2k_rfid_watch_set_by_hand(&tw.w, 2);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), 3);
    ace2k_rfid_watch_set_by_hand(&tw.w, 3);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, all, tw.now), -1);
}

/* A tie-break that changes nothing within its reach (no tag in view at all)
 * goes back and leaves the lane pending, never no_tag: one confirmation leg is too short to tell a
 * spool with no tag.  It went no farther than the confirmation's longest leg. */
TEST(a_tiebreak_that_changes_nothing_leaves_the_lane_pending)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.in.insert = 0x04;
    tb_steps(&tw, NULL, 0, 1);
    tb_mask(&tw, 0);
    tb_steps(&tw, NULL, 0, 4000);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_TRUE(tw.max_um[2] <= tw.w.cfg.leg_max_um + (tw.w.cfg.speed_um_s / 100));
    ASSERT_EQ(tw.in.travel_um[2], 0);
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
}

/* A tie-break that started spends the lane's try, however it ends: the lane is not picked again.
 * The host's read with motion on the lane, or a strand going in on its reader, starts it over.  An
 * out-of-range lane is ignored. */
TEST(a_started_tiebreak_spends_the_try_until_the_hosts_read_or_an_insertion)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.held_um[2] = 20000;
    tw.in.insert = 0x04;
    tb_steps(&tw, NULL, 0, 1);
    tb_mask(&tw, 0);
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[2], 1);
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[2], 1); /* spent */
    ASSERT_EQ(ace2k_rfid_watch_state(&tw.w, 2), ACE2K_RFID_PENDING);
    ace2k_rfid_tiebreak_restart(&tw.tb, 2); /* the host's read with motion ran */
    ace2k_rfid_tiebreak_restart(&tw.tb, ACE2K_LANE_COUNT);
    ace2k_rfid_tiebreak_tried(&tw.tb, ACE2K_LANE_COUNT);
    ASSERT_EQ(tw.tb.tried, 0);
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[2], 2);
    tw.in.insert = 0x0C; /* a strand in lane 4: the reader's tries start over */
    tb_steps(&tw, NULL, 0, 2000);
    ASSERT_EQ(tw.starts[2], 3);
}

/* Lane 3 is pending and keeps the green UID from a session that ended without a
 * read.  Its tie-break starts and is pre-empted on its way: the UID stays lane 3's kept one, so
 * rule 1 still keeps it from lane 4, which the same tag follows. */
TEST(a_tiebreak_keeps_its_lanes_kept_uid_from_the_neighbour)
{
    struct tbworld tw;
    tb_init(&tw);
    struct spooltag tags[1] = { { tag_a(&tw, green), 3, 60000, 176000 } };
    lanes_3_and_4_pending(&tw);
    struct ace2k_rfid_watch_lane *l3 = &tw.w.l[2];
    l3->tag = (struct ace2k_iso14443a_tag){ .uid_len = 4 };
    memcpy(l3->tag.uid, green, 4);
    l3->has_tag = true;
    tb_steps(&tw, tags, 1, TB_STEPS_2S + 20);
    ASSERT_EQ(tw.starts[2], 1);
    ASSERT_EQ(tw.phase[2], PHASE_FORWARD);
    tw.phase[2] = PHASE_BACK; /* a host start pre-empted it: back to where it began */
    tb_steps(&tw, tags, 1, 100);
    ASSERT_EQ(tw.phase[2], PHASE_IDLE);
    ASSERT_TRUE(l3->has_tag);
    ASSERT_TRUE(memcmp(l3->tag.uid, green, 4) == 0);
    tb_steps(&tw, tags, 1, 4000);
    ASSERT_EQ(tw.starts[3], 1); /* lane 4 moved, and green followed it */
    ASSERT_TRUE(tw.got[3] != green[0]);
    ASSERT_TRUE(ace2k_rfid_watch_state(&tw.w, 3) != ACE2K_RFID_READ);
}

/* A read on command on the reader holds its tie-break off: the lane is picked once it ends. */
TEST(a_read_on_command_on_the_reader_holds_its_tiebreak)
{
    struct tbworld tw;
    tb_init(&tw);
    tw.sched_on = false;
    tw.in.insert = 0x01;
    tb_steps(&tw, NULL, 0, 1);
    tb_mask(&tw, 0);
    const uint32_t quiet = tw.now + ACE2K_RFID_TIEBREAK_QUIET_MS;
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x0F, quiet), 0);
    ASSERT_EQ(ace2k_rfid_watch_probe(&tw.w, 0), ACE2K_RFID_READ_OK);
    ASSERT_EQ(ace2k_rfid_tiebreak_pick(&tw.tb, &tw.w, 0x0F, quiet), -1);
}

/* One set of the confirmation's settings, in the order of the wire. */
static int cfg_set6(struct ace2k_rfid_watch *w, uint32_t leg, uint32_t speed, uint32_t passive,
                    uint32_t follows, uint32_t window, uint32_t passive_max)
{
    const uint32_t v[ACE2K_RFID_CFG_COUNT] = { leg, speed, passive, follows, window, passive_max };
    return ace2k_rfid_watch_confirm_set(w, v);
}

/* The same, with a passive maximum that every passive minimum here leaves room under. */
static int cfg_set(struct ace2k_rfid_watch *w, uint32_t leg, uint32_t speed, uint32_t passive,
                   uint32_t follows, uint32_t window)
{
    return cfg_set6(w, leg, speed, passive, follows, window, 2000000);
}

/* The host's settings of the confirmation: in bounds they are taken whole, out of bounds nothing
 * is. */
TEST(the_confirmation_settings_are_taken_only_within_their_bounds)
{
    struct tbworld tw;
    tb_init(&tw);
    struct ace2k_rfid_watch *w = &tw.w;
    ASSERT_EQ(w->cfg.leg_max_um, ACE2K_RFID_CONFIRM_LEG_MAX_UM);
    ASSERT_EQ(w->cfg.speed_um_s, ACE2K_RFID_CONFIRM_SPEED_UM_S);
    ASSERT_EQ(w->cfg.passive_min_um, ACE2K_RFID_PASSIVE_MIN_UM);
    ASSERT_EQ(w->cfg.follows, ACE2K_RFID_CONFIRM_FOLLOWS);
    ASSERT_EQ(w->cfg.window_um, ACE2K_RFID_CONFIRM_WINDOW_UM);
    ASSERT_EQ(w->cfg.passive_max_um, ACE2K_RFID_PASSIVE_MAX_UM);
    ASSERT_EQ(cfg_set6(w, 10000, 9000, 50000, 1, 5000, 50001), 0);
    ASSERT_EQ(w->cfg.passive_max_um, 50001);
    ASSERT_EQ(cfg_set(w, 600000, 80000, 1000000, 3, 100000), 0);
    ASSERT_EQ(w->cfg.leg_max_um, 600000);
    ASSERT_EQ(w->cfg.speed_um_s, 80000);
    ASSERT_EQ(w->cfg.passive_min_um, 1000000);
    ASSERT_EQ(w->cfg.follows, 3);
    ASSERT_EQ(w->cfg.window_um, 100000);
    ASSERT_EQ(cfg_set(w, 9999, 9000, 300000, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 600001, 9000, 300000, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 8999, 300000, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 80001, 300000, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 49999, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 1000001, 1, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 0, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 4, 30000), -ACE2K_EINVAL);
    /* %c arrives whole, so above 255 is host-reachable: 257 must not wrap to 1. */
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 256, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 257, 30000), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 1, 4999), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set(w, 150000, 9000, 300000, 1, 100001), -ACE2K_EINVAL);
    ASSERT_EQ(cfg_set6(w, 150000, 9000, 300000, 1, 30000, 300000), -ACE2K_EINVAL); /* = min */
    ASSERT_EQ(cfg_set6(w, 150000, 9000, 300000, 1, 30000, 2000001), -ACE2K_EINVAL);
    ASSERT_EQ(w->cfg.leg_max_um, 600000); /* nothing taken */
    ASSERT_EQ(w->cfg.speed_um_s, 80000);
    ASSERT_EQ(w->cfg.passive_min_um, 1000000);
    ASSERT_EQ(w->cfg.follows, 3);
    ASSERT_EQ(w->cfg.window_um, 100000);
    ASSERT_EQ(w->cfg.passive_max_um, 2000000);
}

int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
