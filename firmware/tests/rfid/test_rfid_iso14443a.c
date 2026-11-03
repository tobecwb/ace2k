#include "test.h"
#include "rfid/iso14443a.h"
#include "fake_field.h"

// NOLINTNEXTLINE(readability-identifier-naming)
struct rig {
    struct fake_field ff;
    struct ace2k_iso14443a iso;
    int max_calls_in_a_step;
};

static void rig_init(struct rig *g)
{
    memset(g, 0, sizeof *g);
    ace2k_iso14443a_init(&g->iso, &fld_ops, &g->ff);
}

/* Steps reader 0 until the job ends (at most n steps); the largest number of ops calls one step
 * made is kept.  True when the job ended. */
static bool run(struct rig *g, int n)
{
    for (int i = 0; i < n; i++) {
        int before = g->ff.rd[0].starts + g->ff.rd[0].polls;
        bool ended = ace2k_iso14443a_step(&g->iso, 0);
        int calls = g->ff.rd[0].starts + g->ff.rd[0].polls - before;
        if (calls > g->max_calls_in_a_step) {
            g->max_calls_in_a_step = calls;
        }
        if (ended) {
            return true;
        }
    }
    return false;
}

static const uint8_t uid_m1[4] = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t uid_m2[4] = { 0x01, 0x02, 0x03, 0x84 };
static const uint8_t uid_m3[4] = { 0x11, 0x22, 0x33, 0x44 };
static const uint8_t uid_m4[4] = { 0x21, 0x22, 0x33, 0x44 };
static const uint8_t uid_n1[7] = { 0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

static bool only_allowed_firsts(const struct fld *f)
{
    for (int i = 0; i < f->nfirst && i < 1024; i++) {
        if (!ace2k_iso14443a_frame_allowed(f->firsts[i])) {
            return false;
        }
    }
    return true;
}

TEST(crc_a_matches_the_standards_examples)
{
    uint8_t c[2];
    const uint8_t a[2] = { 0x00, 0x00 };
    const uint8_t b[2] = { 0x12, 0x34 };
    const uint8_t h[2] = { 0x50, 0x00 };
    ace2k_iso14443a_crc_a(a, 2, c);
    ASSERT_TRUE(c[0] == 0xA0 && c[1] == 0x1E);
    ace2k_iso14443a_crc_a(b, 2, c);
    ASSERT_TRUE(c[0] == 0x26 && c[1] == 0xCF);
    ace2k_iso14443a_crc_a(h, 2, c);
    ASSERT_TRUE(c[0] == 0x57 && c[1] == 0xCD);
}

TEST(the_gate_lets_nine_first_bytes_through_and_refuses_every_other_without_a_start)
{
    struct rig g;
    rig_init(&g);
    int allowed = 0;
    for (int v = 0; v < 256; v++) {
        uint8_t frame[4] = { (uint8_t)v, 0x04, 0, 0 };
        int rc = ace2k_iso14443a_send(&g.iso, 0, ACE2K_RFID_CMD_TRANSCEIVE, frame, 4, 0);
        if (ace2k_iso14443a_frame_allowed((uint8_t)v)) {
            allowed++;
            ASSERT_EQ(rc, 0);
        } else {
            ASSERT_EQ(rc, -ACE2K_EREFUSED);
        }
    }
    ASSERT_EQ(allowed, 9);
    ASSERT_EQ(g.ff.rd[0].starts, 9); /* the refused 247 never reached the reader */
    const uint8_t writes[] = { 0xA0, 0xA2, 0xB0, 0xC0, 0xC1, 0xC2, 0x1B, 0xA9, 0xAC };
    for (unsigned i = 0; i < sizeof writes; i++) {
        ASSERT_TRUE(!ace2k_iso14443a_frame_allowed(writes[i]));
    }
}

TEST(an_inventory_of_an_empty_field_ends_ok_with_no_tag_after_a_wupa)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(ace2k_iso14443a_busy(&g.iso, 0));
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), -ACE2K_EREFUSED); /* one job at a time */
    ASSERT_TRUE(run(&g, 20));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->tag_count, 0);
    ASSERT_EQ(g.ff.rd[0].firsts[0], 0x52);
    ASSERT_TRUE(!ace2k_iso14443a_busy(&g.iso, 0));
}

TEST(an_inventory_finds_a_mifare_and_an_ntag_through_the_cascade)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    add_ntag(&g.ff.rd[0], uid_n1);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 60));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->tag_count, 2);
    bool m = false, n = false;
    for (int i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        if (t->uid_len == 4 && memcmp(t->uid, uid_m1, 4) == 0 && t->sak == 0x08) {
            m = true;
        }
        if (t->uid_len == 7 && memcmp(t->uid, uid_n1, 7) == 0 && t->sak == 0x00) {
            n = true;
        }
    }
    ASSERT_TRUE(m && n);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 3); /* both halted */
    ASSERT_EQ(g.ff.rd[0].t[1].state, 3);
    ASSERT_TRUE(only_allowed_firsts(&g.ff.rd[0]));
    ASSERT_TRUE(g.max_calls_in_a_step <= 2);
    /* the next inventory wakes the halted ones again */
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 60));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 2);
}

TEST(an_inventory_resolves_collisions_in_the_last_and_the_first_byte)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    add_mifare(&g.ff.rd[0], uid_m2); /* differ in bit 32 of the level */
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 80));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 2);
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m3);
    add_mifare(&g.ff.rd[0], uid_m4); /* differ in bit 5 */
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 80));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 2);
    ASSERT_TRUE(!ace2k_iso14443a_same_uid(&o->tags[0], &o->tags[1]));
    ASSERT_TRUE(only_allowed_firsts(&g.ff.rd[0]));
}

TEST(an_ntag_read_returns_twelve_pages_from_the_first)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 3 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, uid_n1, 7);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->data_count, 3);
    ASSERT_EQ(o->data_first[0], 4);
    ASSERT_EQ(o->data_first[2], 12);
    ASSERT_EQ(o->data[0][0], 16);  /* page 4, byte 0 */
    ASSERT_EQ(o->data[2][15], 63); /* page 15, byte 3 */
    ASSERT_TRUE(g.max_calls_in_a_step <= 2);
    /* the read job ends with HLTA: the tag is halted, a later WUPA wakes it */
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], 0x50);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 3);
    /* a group count of 0 or 4 is refused */
    job.count = 4;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), -ACE2K_EINVAL);
    /* a reader out of range has no result */
    ASSERT_TRUE(ace2k_iso14443a_result(&g.iso, ACE2K_RFID_READER_COUNT) == NULL);
}

TEST(a_mifare_read_with_the_right_key_the_wrong_key_and_the_key_wiped)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    struct ace2k_iso14443a_job job = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x7,
        .key_type = 0,
        .key = { 1, 1, 2, 3, 4, 5 },
    };
    job.target.uid_len = 4;
    memcpy(job.target.uid, uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    int checked = 0;
    for (int i = 0; i < 40 && ace2k_iso14443a_busy(&g.iso, 0); i++) {
        (void)ace2k_iso14443a_step(&g.iso, 0);
        /* WUPA, SELECT, MFAuthent: from the third start on, while the job still runs, the
         * engine holds no key (rule 7) */
        if (ace2k_iso14443a_busy(&g.iso, 0) && g.ff.rd[0].starts >= 3) {
            const uint8_t *k = g.iso.r[0].job.key;
            ASSERT_TRUE(k[0] == 0 && k[1] == 0 && k[2] == 0 && k[3] == 0 && k[4] == 0 && k[5] == 0);
            checked++;
        }
    }
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK); /* the key was right: it reached the tag before the wipe */
    ASSERT_TRUE(checked > 0);
    ASSERT_EQ(o->data_count, 3);
    ASSERT_EQ(o->data_first[0], 4);
    ASSERT_EQ(o->data[0][0], 64);   /* block 4 */
    ASSERT_EQ(o->data[2][15], 111); /* block 6 */
    ASSERT_EQ(g.ff.rd[0].crypto_offs, 1);
    /* HLTA ends the read job, sent while Crypto1 was still on: the tag is halted */
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], 0x50);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 3);
    /* the wrong key — the second job's WUPA wakes the halted tag: AUTH_FAILED, not TAG_GONE
     * (the regression of a job that left its tag active).  The tag stays silent, as a real
     * one does: the reader's timer ends MFAuthent, and a silent authentication is a wrong key */
    job.key[0] = 9;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_AUTH_FAILED);
    ASSERT_EQ(g.ff.rd[0].crypto_offs, 2);
    /* the next key tried wakes the tag the failure left idle: the right one reads */
    job.key[0] = 1;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_OK);
    /* only blocks 0..2 of a sector: never its trailer */
    job.count = 0x8;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), -ACE2K_EINVAL);
    ASSERT_TRUE(only_allowed_firsts(&g.ff.rd[0]));
}

TEST(an_authentication_that_ends_without_crypto1_is_auth_failed_too)
{
    /* the explicit path: MFAuthent done, MFCrypto1On not set */
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].auth_nak = true;
    struct ace2k_iso14443a_job job = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x1,
        .key = { 9, 1, 2, 3, 4, 5 },
    };
    job.target.uid_len = 4;
    memcpy(job.target.uid, uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_AUTH_FAILED);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
}

TEST(a_target_not_in_the_field_is_tag_gone_and_a_mute_reader_an_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m3);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_MIFARE_READ, .arg = 1, .count = 1 };
    job.target.uid_len = 4;
    memcpy(job.target.uid, uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_TAG_GONE);
    g.ff.rd[0].mute = true;
    struct ace2k_iso14443a_job inv = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &inv), 0);
    ASSERT_TRUE(!run(&g, (int)ACE2K_ISO_POLL_MAX)); /* still waiting */
    ASSERT_TRUE(run(&g, 2));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_ERROR);
    ASSERT_TRUE(g.max_calls_in_a_step <= 2);
}

/* A read job that ends in ERROR sends no HLTA: its tag stays ACTIVE (AUTHENTICATED for
 * MIFARE), and takes the next job's WUPA as an unexpected command — it drops to idle without an
 * answer.  The read's WUPA is sent once more on that silence, and the retried job reads. */
static int count_wupa_pairs(const struct fld *f)
{
    int pairs = 0;
    for (int i = 1; i < f->nfirst && i < 1024; i++) {
        if (f->firsts[i - 1] == 0x52 && f->firsts[i] == 0x52) {
            pairs++;
        }
    }
    return pairs;
}

TEST(a_tag_left_active_by_an_errored_read_is_read_by_the_retried_step)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].bad_reads = 1;
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 1 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, uid_n1, 7);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_ERROR);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 2); /* no HLTA: still ACTIVE */
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->data_count, 1);
    ASSERT_EQ(o->data[0][0], 16);
    ASSERT_EQ(count_wupa_pairs(&g.ff.rd[0]), 1); /* the first WUPA met silence, the second woke */
    ASSERT_TRUE(g.max_calls_in_a_step <= 2);

    /* MIFARE: the tag left AUTHENTICATED by the errored job is read the same way */
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].bad_reads = 1;
    struct ace2k_iso14443a_job mf = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x1,
        .key = { 1, 1, 2, 3, 4, 5 },
    };
    mf.target.uid_len = 4;
    memcpy(mf.target.uid, uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &mf), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_ERROR);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 2);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &mf), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_OK);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data[0][0], 64);
    ASSERT_EQ(count_wupa_pairs(&g.ff.rd[0]), 1);
    ASSERT_TRUE(g.max_calls_in_a_step <= 2);
}

TEST(an_inventory_ends_on_its_first_silence_and_a_read_on_its_second)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_iso14443a_job inv = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &inv), 0);
    ASSERT_TRUE(run(&g, 20));
    ASSERT_EQ(g.ff.rd[0].starts, 1); /* one WUPA */
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 1 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, uid_n1, 7);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 20));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_TAG_GONE);
    ASSERT_EQ(g.ff.rd[0].starts, 3); /* two WUPAs, then TAG_GONE */
}

/* The hostile tag: each test below sets one malformed answer in the fake field and expects the
 * refusal the engine documents for it.  The sanitizers stop the test on any access out of range;
 * engine_in_bounds() checks every index the engine keeps, and only_allowed_firsts() that no frame
 * left outside the gate. */
static bool engine_in_bounds(const struct ace2k_iso14443a_engine *e)
{
    if (e->out.tag_count > ACE2K_ISO_INVENTORY_MAX || e->out.data_count > ACE2K_ISO_READS_MAX) {
        return false;
    }
    if (e->cur.uid_len > ACE2K_ISO_UID_MAX || e->level >= ACE2K_ISO_LEVELS_MAX) {
        return false;
    }
    if (e->known_bits > ACE2K_ISO_CL_BITS || e->reads_done > e->reads_total) {
        return false;
    }
    return e->reads_total <= ACE2K_ISO_READS_MAX;
}

/* An inventory of reader 0's field to its end; its result. */
static uint8_t inventory(struct rig *g)
{
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g->iso, 0, &job), 0);
    ASSERT_TRUE(run(g, 80));
    ASSERT_TRUE(engine_in_bounds(&g->iso.r[0]));
    ASSERT_TRUE(only_allowed_firsts(&g->ff.rd[0]));
    return ace2k_iso14443a_result(&g->iso, 0)->result;
}

/* A lost HLTA leaves the tag active: the next REQA drops it to idle without an answer, and a
 * later REQA selects it again.  The inventory lists each UID once, and a pair of tags whose every
 * HLTA is lost still ends, after ACE2K_ISO_INVENTORY_MAX selections. */
TEST(an_inventory_lists_a_tag_once_when_its_halt_is_lost)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m3);
    add_mifare(&g.ff.rd[0], uid_m4);
    g.ff.rd[0].lose_halts = 1;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 2);
    ASSERT_TRUE(!ace2k_iso14443a_same_uid(&o->tags[0], &o->tags[1]));
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m3);
    add_mifare(&g.ff.rd[0], uid_m4);
    g.ff.rd[0].lose_halts = 1000;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 2);
    ASSERT_TRUE(!ace2k_iso14443a_same_uid(&o->tags[0], &o->tags[1]));
}

/* A read of one group of four pages of the NTAG n1, from page 4, to its end; its result. */
static uint8_t ntag_read(struct rig *g)
{
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 1 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, uid_n1, 7);
    ASSERT_EQ(ace2k_iso14443a_begin(&g->iso, 0, &job), 0);
    ASSERT_TRUE(run(g, 40));
    ASSERT_TRUE(engine_in_bounds(&g->iso.r[0]));
    ASSERT_TRUE(only_allowed_firsts(&g->ff.rd[0]));
    return ace2k_iso14443a_result(&g->iso, 0)->result;
}

/* A triple-size UID whose last cascade level starts with the cascade tag (0x88). */
static const uint8_t uid_t1[10] = {
    0x05, 0x11, 0x22, 0x33, 0x44, 0x55, 0x88, 0x77, 0x66, 0x99,
};

static struct ftag *add_triple(struct fld *f)
{
    struct ftag *t = add_ntag(f, uid_n1);
    t->uid_len = 10;
    memcpy(t->uid, uid_t1, 10);
    return t;
}

TEST(an_atqa_of_one_byte_is_not_taken_and_the_tag_is_still_found)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_ATQA_SHORT;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 1);
    ASSERT_EQ(o->tags[0].atqa, 0);
    ASSERT_TRUE(memcmp(o->tags[0].uid, uid_m1, 4) == 0);
    /* the same tag with a two-byte answer gives its ATQA */
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_NONE;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tags[0].atqa, 0x0004);
}

TEST(an_atqa_of_three_bytes_is_not_taken_and_the_tag_is_still_found)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_ATQA_LONG;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 1);
    ASSERT_EQ(o->tags[0].atqa, 0);
    ASSERT_TRUE(memcmp(o->tags[0].uid, uid_m1, 4) == 0);
}

TEST(a_wrong_bcc_ends_the_inventory_in_error_before_any_selection)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_BCC;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 1); /* woken, never selected */
}

TEST(a_collision_at_a_bit_already_known_ends_the_inventory_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m3);
    add_mifare(&g.ff.rd[0], uid_m4); /* a real collision at bit 5 first */
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_COLL_AT_KNOWN;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.iso.r[0].known_bits, 5);
    /* refused on its first answer, not after the anticollision's iteration cap */
    ASSERT_EQ(g.ff.rd[0].starts, 3); /* WUPA and two anticollision frames */
    ASSERT_EQ(g.iso.r[0].iterations, 1);
}

TEST(a_collision_past_the_four_uid_bytes_ends_the_inventory_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_COLL_PAST_UID;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.iso.r[0].known_bits, 0);
    ASSERT_EQ(g.ff.rd[0].starts, 2); /* WUPA and one anticollision frame */
}

TEST(a_sak_without_its_crc_ends_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_SHORT;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
}

TEST(a_sak_of_two_bytes_with_a_valid_crc_ends_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_LONG;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
}

TEST(a_sak_with_a_broken_crc_ends_an_inventory_and_a_read_in_error)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_CRC;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    /* a read selects its target the same way: no READ goes out after the broken SAK */
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_CRC;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
    for (int i = 0; i < g.ff.rd[0].nfirst; i++) {
        ASSERT_TRUE(g.ff.rd[0].firsts[i] != ACE2K_ISO_READ);
    }
}

TEST(a_sak_that_claims_a_cascade_without_the_cascade_tag_ends_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1); /* a single-size UID: no CT in its level */
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_CASCADE;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.iso.r[0].level, 0);
    ASSERT_EQ(g.iso.r[0].cur.uid_len, 0);
}

TEST(a_sak_that_asks_for_a_fourth_cascade_level_ends_in_error)
{
    struct rig g;
    rig_init(&g);
    add_triple(&g.ff.rd[0]);
    /* the tag as it is: three levels, a ten-byte UID */
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 1);
    ASSERT_EQ(o->tags[0].uid_len, 10);
    ASSERT_TRUE(memcmp(o->tags[0].uid, uid_t1, 10) == 0);
    /* its last level answered "cascade" again: refused at the third level */
    rig_init(&g);
    add_triple(&g.ff.rd[0]);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_SAK_CASCADE;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.iso.r[0].level, ACE2K_ISO_LEVELS_MAX - 1U);
    ASSERT_EQ(g.iso.r[0].cur.uid_len, 6);
}

TEST(a_read_answer_of_fifteen_bytes_and_a_valid_crc_ends_the_read_in_error)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_READ_SHORT;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], ACE2K_ISO_READ); /* no HLTA after it */
}

TEST(a_read_answer_with_a_partial_last_byte_ends_the_read_in_error)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_READ_LAST_BITS;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], ACE2K_ISO_READ);
}

/* A tag cannot send this one: the reader refuses a FIFO of more than 18 bytes.  It stands for a
 * receive path that reports one byte more than the buffer holds.  The length check must refuse
 * it before the CRC_A check reads past the buffer, which the sanitizers would stop. */
TEST(a_read_answer_one_byte_longer_than_the_buffer_ends_the_read_in_error)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_READ_LONG;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], ACE2K_ISO_READ);
}

/* The engine's other refusals and endings: arguments it does not take, a reader that answers
 * with an error, silence or a collision where a tag should answer, and a field full of tags. */
TEST(a_frame_of_no_bytes_is_refused_and_never_reaches_the_reader)
{
    struct rig g;
    rig_init(&g);
    const uint8_t frame[1] = { ACE2K_ISO_REQA };
    ASSERT_EQ(ace2k_iso14443a_send(&g.iso, 0, ACE2K_RFID_CMD_TRANSCEIVE, frame, 0, 0),
              -ACE2K_EREFUSED);
    ASSERT_EQ(g.ff.rd[0].starts, 0);
}

TEST(two_uids_of_different_lengths_are_never_the_same_tag)
{
    struct ace2k_iso14443a_tag a = { .uid_len = 4 };
    struct ace2k_iso14443a_tag b = { .uid_len = 7 };
    memcpy(a.uid, uid_n1, 4);
    memcpy(b.uid, uid_n1, 7); /* the first four bytes are the same */
    ASSERT_TRUE(!ace2k_iso14443a_same_uid(&a, &b));
    ASSERT_TRUE(!ace2k_iso14443a_same_uid(&b, &a));
    b.uid_len = 4;
    ASSERT_TRUE(ace2k_iso14443a_same_uid(&a, &b));
}

TEST(a_read_takes_a_triple_size_target_and_refuses_any_other_uid_length)
{
    struct rig g;
    rig_init(&g);
    add_triple(&g.ff.rd[0]);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 1 };
    job.target.uid_len = 10;
    memcpy(job.target.uid, uid_t1, 10);
    ASSERT_TRUE(ace2k_iso14443a_job_valid(&job));
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 60));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->data_count, 1);
    ASSERT_EQ(o->data[0][0], 16); /* page 4, byte 0 */
    const uint8_t bad_lengths[] = { 0, 3, 5, 9, 11 };
    for (unsigned i = 0; i < sizeof bad_lengths; i++) {
        job.target.uid_len = bad_lengths[i];
        ASSERT_TRUE(!ace2k_iso14443a_job_valid(&job));
        ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), -ACE2K_EINVAL);
    }
}

TEST(a_read_of_no_page_group_or_no_block_is_refused)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_iso14443a_job ntag = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 0 };
    ntag.target.uid_len = 7;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &ntag), -ACE2K_EINVAL);
    struct ace2k_iso14443a_job mf = { .kind = ACE2K_ISO_JOB_MIFARE_READ, .arg = 1, .count = 0 };
    mf.target.uid_len = 4;
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &mf), -ACE2K_EINVAL);
    ASSERT_TRUE(!ace2k_iso14443a_busy(&g.iso, 0));
    ASSERT_EQ(g.ff.rd[0].starts, 0);
}

TEST(a_job_of_no_known_kind_is_refused)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NONE };
    ASSERT_TRUE(!ace2k_iso14443a_job_valid(&job));
    job.kind = ACE2K_ISO_JOB_MIFARE_READ + 1U;
    job.target.uid_len = 4;
    job.count = 1;
    ASSERT_TRUE(!ace2k_iso14443a_job_valid(&job));
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), -ACE2K_EINVAL);
    ASSERT_TRUE(!ace2k_iso14443a_busy(&g.iso, 0));
}

TEST(a_reader_out_of_range_takes_no_job_and_is_never_busy)
{
    struct rig g;
    rig_init(&g);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, ACE2K_RFID_READER_COUNT, &job), -ACE2K_EINVAL);
    ASSERT_TRUE(!ace2k_iso14443a_busy(&g.iso, ACE2K_RFID_READER_COUNT));
    ASSERT_TRUE(!ace2k_iso14443a_step(&g.iso, ACE2K_RFID_READER_COUNT));
}

TEST(stepping_a_reader_with_no_job_starts_nothing)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    ASSERT_TRUE(!ace2k_iso14443a_step(&g.iso, 0));
    ASSERT_EQ(g.ff.rd[0].starts + g.ff.rd[0].polls, 0);
}

TEST(a_mifare_read_with_key_b_authenticates_with_key_b)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    struct ace2k_iso14443a_job job = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x1,
        .key_type = 1,
        .key = { 1, 9, 8, 7, 6, 5 }, /* fake_field's sector-1 key B */
    };
    job.target.uid_len = 4;
    memcpy(job.target.uid, uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->data[0][0], 64); /* block 4 */
    bool auth_b = false;
    for (int i = 0; i < g.ff.rd[0].nfirst; i++) {
        ASSERT_TRUE(g.ff.rd[0].firsts[i] != ACE2K_ISO_AUTH_A);
        if (g.ff.rd[0].firsts[i] == ACE2K_ISO_AUTH_B) {
            auth_b = true;
        }
    }
    ASSERT_TRUE(auth_b);
}

TEST(a_reader_error_on_the_wake_up_ends_the_job_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_WAKE;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_ERROR;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.ff.rd[0].starts, 1); /* the WUPA alone */
}

/* Two tags of different ATQA answer a wake-up at once: the reader sees a collision there. The
 * ATQA is not taken, and the anticollision still finds the tags. */
TEST(a_collision_in_the_wake_up_answer_goes_on_to_the_anticollision)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_WAKE;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_COLLISION;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->tag_count, 1);
    ASSERT_EQ(o->tags[0].atqa, 0);
    ASSERT_TRUE(memcmp(o->tags[0].uid, uid_m1, 4) == 0);
}

/* The inventory keeps at most four tags. After the fourth it ends at once, with no REQA to look
 * for a fifth. */
TEST(an_inventory_ends_after_its_fourth_tag_without_waking_another)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    add_mifare(&g.ff.rd[0], uid_m2);
    add_mifare(&g.ff.rd[0], uid_m3);
    add_mifare(&g.ff.rd[0], uid_m4);
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, ACE2K_ISO_INVENTORY_MAX);
    ASSERT_EQ(g.ff.rd[0].firsts[g.ff.rd[0].nfirst - 1], ACE2K_ISO_HLTA);
}

TEST(a_target_that_leaves_before_its_read_answer_is_tag_gone)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_READ;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_TIMEOUT;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_TAG_GONE);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
}

TEST(a_reader_error_on_a_read_ends_the_read_in_error)
{
    struct rig g;
    rig_init(&g);
    add_ntag(&g.ff.rd[0], uid_n1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_READ;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_ERROR;
    ASSERT_EQ(ntag_read(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
}

/* The engine never retries a start the reader refused. The job ends in error on that step, and
 * the reader is free for the next job. */
TEST(a_start_the_reader_refuses_ends_the_job_in_error_on_that_step)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].start_rc = -ACE2K_EINVAL;
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(ace2k_iso14443a_step(&g.iso, 0));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_ERROR);
    ASSERT_TRUE(!ace2k_iso14443a_busy(&g.iso, 0));
    ASSERT_EQ(g.ff.rd[0].starts, 1);
    ASSERT_EQ(g.ff.rd[0].polls, 0);
}

TEST(an_anticollision_answer_one_byte_too_long_ends_the_inventory_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_CL_LONG;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 1); /* woken, never selected */
}

TEST(an_anticollision_answer_with_a_partial_last_byte_ends_the_inventory_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].hostile = ACE2K_FAKE_HOSTILE_CL_LAST_BITS;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    ASSERT_EQ(g.ff.rd[0].t[0].state, 1);
}

/* Silence where the anticollision expects an answer: the tags have gone, and an inventory that
 * found nothing more ends OK. A reader error there ends it in error. */
TEST(silence_in_the_anticollision_ends_the_inventory_ok_and_a_reader_error_in_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_ANTICOLL;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_TIMEOUT;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_OK);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    rig_init(&g);
    add_mifare(&g.ff.rd[0], uid_m1);
    g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_ANTICOLL;
    g.ff.rd[0].override_poll = ACE2K_RFID_POLL_ERROR;
    ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
}

TEST(a_reader_error_or_a_collision_on_a_selection_ends_in_error)
{
    struct rig g;
    const enum ace2k_rfid_poll polls[] = { ACE2K_RFID_POLL_ERROR, ACE2K_RFID_POLL_COLLISION };
    for (unsigned i = 0; i < sizeof polls / sizeof polls[0]; i++) {
        rig_init(&g);
        add_mifare(&g.ff.rd[0], uid_m1);
        g.ff.rd[0].override_frame = ACE2K_FAKE_FRAME_SELECT;
        g.ff.rd[0].override_poll = polls[i];
        ASSERT_EQ(inventory(&g), ACE2K_ISO_ERROR);
        ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 0);
    }
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
