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
    ace2k_iso14443a_init(&g->iso, &ace2k_fld_ops, &g->ff);
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

static const uint8_t ace2k_uid_m1[4] = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t ace2k_uid_m2[4] = { 0x01, 0x02, 0x03, 0x84 };
static const uint8_t ace2k_uid_m3[4] = { 0x11, 0x22, 0x33, 0x44 };
static const uint8_t ace2k_uid_m4[4] = { 0x21, 0x22, 0x33, 0x44 };
static const uint8_t ace2k_uid_n1[7] = { 0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

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
    add_mifare(&g.ff.rd[0], ace2k_uid_m1);
    add_ntag(&g.ff.rd[0], ace2k_uid_n1);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 60));
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(&g.iso, 0);
    ASSERT_EQ(o->result, ACE2K_ISO_OK);
    ASSERT_EQ(o->tag_count, 2);
    bool m = false, n = false;
    for (int i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        if (t->uid_len == 4 && memcmp(t->uid, ace2k_uid_m1, 4) == 0 && t->sak == 0x08) {
            m = true;
        }
        if (t->uid_len == 7 && memcmp(t->uid, ace2k_uid_n1, 7) == 0 && t->sak == 0x00) {
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
    add_mifare(&g.ff.rd[0], ace2k_uid_m1);
    add_mifare(&g.ff.rd[0], ace2k_uid_m2); /* differ in bit 32 of the level */
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 80));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->tag_count, 2);
    rig_init(&g);
    add_mifare(&g.ff.rd[0], ace2k_uid_m3);
    add_mifare(&g.ff.rd[0], ace2k_uid_m4); /* differ in bit 5 */
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
    add_ntag(&g.ff.rd[0], ace2k_uid_n1);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 3 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, ace2k_uid_n1, 7);
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
}

TEST(a_mifare_read_with_the_right_key_the_wrong_key_and_the_key_wiped)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], ace2k_uid_m1);
    struct ace2k_iso14443a_job job = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x7,
        .key_type = 0,
        .key = { 1, 1, 2, 3, 4, 5 },
    };
    job.target.uid_len = 4;
    memcpy(job.target.uid, ace2k_uid_m1, 4);
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
    add_mifare(&g.ff.rd[0], ace2k_uid_m1);
    g.ff.rd[0].auth_nak = true;
    struct ace2k_iso14443a_job job = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x1,
        .key = { 9, 1, 2, 3, 4, 5 },
    };
    job.target.uid_len = 4;
    memcpy(job.target.uid, ace2k_uid_m1, 4);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 40));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_AUTH_FAILED);
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->data_count, 0);
}

TEST(a_target_not_in_the_field_is_tag_gone_and_a_mute_reader_an_error)
{
    struct rig g;
    rig_init(&g);
    add_mifare(&g.ff.rd[0], ace2k_uid_m3);
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_MIFARE_READ, .arg = 1, .count = 1 };
    job.target.uid_len = 4;
    memcpy(job.target.uid, ace2k_uid_m1, 4);
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
    add_ntag(&g.ff.rd[0], ace2k_uid_n1);
    g.ff.rd[0].bad_reads = 1;
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_NTAG_READ, .arg = 4, .count = 1 };
    job.target.uid_len = 7;
    memcpy(job.target.uid, ace2k_uid_n1, 7);
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
    add_mifare(&g.ff.rd[0], ace2k_uid_m1);
    g.ff.rd[0].bad_reads = 1;
    struct ace2k_iso14443a_job mf = {
        .kind = ACE2K_ISO_JOB_MIFARE_READ,
        .arg = 1,
        .count = 0x1,
        .key = { 1, 1, 2, 3, 4, 5 },
    };
    mf.target.uid_len = 4;
    memcpy(mf.target.uid, ace2k_uid_m1, 4);
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
    memcpy(job.target.uid, ace2k_uid_n1, 7);
    ASSERT_EQ(ace2k_iso14443a_begin(&g.iso, 0, &job), 0);
    ASSERT_TRUE(run(&g, 20));
    ASSERT_EQ(ace2k_iso14443a_result(&g.iso, 0)->result, ACE2K_ISO_TAG_GONE);
    ASSERT_EQ(g.ff.rd[0].starts, 3); /* two WUPAs, then TAG_GONE */
}

// NOLINTNEXTLINE(readability-identifier-naming)
int main(int argc, char **argv)
{
    return TEST_MAIN(argc, argv);
}
