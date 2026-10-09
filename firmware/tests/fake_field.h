/* A field of fake ISO 14443-A tags per reader, answering at the frame level behind
 * struct ace2k_iso14443a_ops — shared by the iso14443a and the watch tests.  Header-only: every
 * function is static, each test file gets its own copy. */
#ifndef ACE2K_TEST_FAKE_FIELD_H
#define ACE2K_TEST_FAKE_FIELD_H
#include <string.h>
#include "rfid/iso14443a.h"

/* Up to four tags per reader, answering at the frame level as ISO 14443-3 describes: REQA wakes
 * the idle ones, WUPA the idle and the halted, anticollision frames get the remaining bits of
 * the cascade level with the first differing bit reported as a collision, a SELECT that matches
 * one tag moves it on (the others back to idle), HLTA halts the active tag, READ answers four
 * NTAG pages or an authenticated MIFARE block, MFAuthent checks the sector's key A and, like a
 * real tag, stays silent on a wrong one (auth_nak: the reader's AUTH_FAILED instead).  An
 * ACTIVE tag takes a REQA or a WUPA as an unexpected command: it drops to idle without an
 * answer.  bad_reads: that many READ answers go out with a broken CRC_A.  Every first byte the
 * engine sends is logged. */
#define MAXTAGS 4
// NOLINTNEXTLINE(readability-identifier-naming)
struct ftag {
    uint8_t uid[10];
    uint8_t uid_len;
    uint16_t atqa;
    uint8_t sak;
    bool ntag;
    bool absent;
    uint8_t pages[64][4];
    uint8_t blocks[64][16];
    uint8_t key_a[16][6];
    int state; /* 0 idle, 1 ready, 2 active, 3 halted */
    int level;
    int authed_sector;
};

// NOLINTNEXTLINE(readability-identifier-naming)
struct fld {
    struct ftag t[MAXTAGS];
    int n;
    enum ace2k_rfid_poll pending;
    struct ace2k_rfid_rx rx;
    bool mute;
    bool
        auth_nak; /* a wrong key ends MFAuthent without MFCrypto1On; default: silence (a real tag) */
    int bad_reads; /* the next READ answers with a broken CRC_A: the engine's ERROR */
    int starts, polls, crypto_offs;
    uint8_t firsts[1024];
    int nfirst;
};

/* One field per reader: the ops' ctx; each reader's frames reach only its own tags. */
// NOLINTNEXTLINE(readability-identifier-naming)
struct fake_field {
    struct fld rd[ACE2K_RFID_READER_COUNT];
};

static int levels_of(const struct ftag *t)
{
    if (t->uid_len == 4) {
        return 1;
    }
    return t->uid_len == 7 ? 2 : 3;
}

/* The five bytes of cascade level L: CT and three UID bytes, or the last four; then the BCC. */
static void cl_of(const struct ftag *t, int level, uint8_t cl[5])
{
    int base = 3 * level;
    if (level < levels_of(t) - 1) {
        cl[0] = 0x88;
        cl[1] = t->uid[base];
        cl[2] = t->uid[base + 1];
        cl[3] = t->uid[base + 2];
    } else {
        for (int i = 0; i < 4; i++) {
            cl[i] = t->uid[base + i];
        }
    }
    cl[4] = (uint8_t)((uint32_t)cl[0] ^ cl[1] ^ cl[2] ^ cl[3]);
}

static bool bit_of(const uint8_t *b, int i)
{
    return (((uint32_t)b[i / 8] >> (uint32_t)(i % 8)) & 1U) != 0;
}

static void with_crc(struct ace2k_rfid_rx *rx)
{
    ace2k_iso14443a_crc_a(rx->data, rx->len, &rx->data[rx->len]);
    rx->len += 2;
}

static bool crc_ok(const uint8_t *tx, uint8_t len)
{
    uint8_t c[2];
    ace2k_iso14443a_crc_a(tx, (uint8_t)(len - 2), c);
    if (c[0] != tx[len - 2]) {
        return false;
    }
    return c[1] == tx[len - 1];
}

static enum ace2k_rfid_poll wake(struct fld *f, bool wupa)
{
    int first = -1;
    for (int i = 0; i < f->n; i++) {
        struct ftag *t = &f->t[i];
        if (!t->absent && t->state == 2) {
            t->state = 0; /* an unexpected command to an ACTIVE tag: back to idle, no answer */
            continue;
        }
        if (t->absent || !(t->state == 0 || (wupa && t->state == 3))) {
            continue;
        }
        t->state = 1;
        t->level = 0;
        if (first < 0) {
            first = i;
        }
    }
    if (first < 0) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    f->rx.data[0] = (uint8_t)f->t[first].atqa;
    f->rx.data[1] = (uint8_t)((uint32_t)f->t[first].atqa >> 8U);
    f->rx.len = 2;
    return ACE2K_RFID_POLL_DONE;
}

static bool ready_at(const struct ftag *t, int level)
{
    if (t->absent || t->state != 1) {
        return false;
    }
    return t->level == level;
}

/* The ready tags at this level whose first `known` bits match the frame's, their cascade levels
 * copied into cl[]; the count. */
static int match_candidates(const struct fld *f, const uint8_t *tx, int level, int known,
                            uint8_t cl[MAXTAGS][5])
{
    int nc = 0;
    for (int i = 0; i < f->n; i++) {
        if (!ready_at(&f->t[i], level)) {
            continue;
        }
        cl_of(&f->t[i], level, cl[nc]);
        bool match = true;
        for (int b = 0; b < known && match; b++) {
            match = bit_of(cl[nc], b) == bit_of(&tx[2], b);
        }
        if (match) {
            nc++;
        }
    }
    return nc;
}

static bool differs_at(const uint8_t cl[MAXTAGS][5], int nc, int b)
{
    for (int c = 1; c < nc; c++) {
        if (bit_of(cl[c], b) != bit_of(cl[0], b)) {
            return true;
        }
    }
    return false;
}

/* The first bit from `known` on where the candidates differ, 1-based; 0 when none does. */
static int first_collision(const uint8_t cl[MAXTAGS][5], int nc, int known)
{
    for (int b = known; b < 32; b++) {
        if (differs_at(cl, nc, b)) {
            return b + 1;
        }
    }
    return 0;
}

/* The first candidate's remaining bytes, the low `nbits` of the first one cleared (RxAlign). */
static void fill_answer(struct fld *f, const uint8_t cl[5], int nbytes, int nbits)
{
    uint32_t mask = (1U << (uint32_t)nbits) - 1U;
    f->rx.len = (uint8_t)(5 - nbytes);
    for (int i = 0; i < f->rx.len; i++) {
        f->rx.data[i] = cl[nbytes + i];
    }
    f->rx.data[0] = (uint8_t)((uint32_t)f->rx.data[0] & ~mask);
}

static enum ace2k_rfid_poll anticoll(struct fld *f, const uint8_t *tx, int level)
{
    int nbytes = (int)((uint32_t)tx[1] >> 4U) - 2;
    int nbits = (int)((uint32_t)tx[1] & 7U);
    int known = (nbytes * 8) + nbits;
    uint8_t cl[MAXTAGS][5];
    int nc = match_candidates(f, tx, level, known, cl);
    if (nc == 0) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    int coll = first_collision(cl, nc, known);
    fill_answer(f, cl[0], nbytes, nbits);
    if (coll != 0) {
        f->rx.coll_pos = (uint8_t)coll;
        return ACE2K_RFID_POLL_COLLISION;
    }
    return ACE2K_RFID_POLL_DONE;
}

static enum ace2k_rfid_poll select_tag(struct fld *f, const uint8_t *tx, uint8_t len, int level)
{
    if (len != 9 || !crc_ok(tx, len)) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    int hit = -1;
    for (int i = 0; i < f->n; i++) {
        struct ftag *t = &f->t[i];
        if (!ready_at(t, level)) {
            continue;
        }
        uint8_t cl[5];
        cl_of(t, level, cl);
        if (hit < 0 && memcmp(cl, &tx[2], 5) == 0) {
            hit = i;
        } else {
            t->state = 0; /* not selected: back to idle */
        }
    }
    if (hit < 0) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    struct ftag *t = &f->t[hit];
    if (level < levels_of(t) - 1) {
        t->level = level + 1;
        f->rx.data[0] = 0x04;
    } else {
        t->state = 2;
        f->rx.data[0] = t->sak;
    }
    f->rx.len = 1;
    with_crc(&f->rx);
    return ACE2K_RFID_POLL_DONE;
}

static struct ftag *active(struct fld *f)
{
    for (int i = 0; i < f->n; i++) {
        if (!f->t[i].absent && f->t[i].state == 2) {
            return &f->t[i];
        }
    }
    return 0;
}

static enum ace2k_rfid_poll read_tag(struct fld *f, const uint8_t *tx, uint8_t len)
{
    struct ftag *t = active(f);
    if (!t || len != 4 || !crc_ok(tx, len)) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    uint8_t addr = tx[1];
    if (t->ntag) {
        for (int i = 0; i < 16; i++) {
            f->rx.data[i] = t->pages[(addr + (i / 4)) % 64][i % 4];
        }
    } else if (t->authed_sector == addr / 4) {
        memcpy(f->rx.data, t->blocks[addr], 16);
    } else {
        f->rx.data[0] = 0x04; /* NAK: four bits */
        f->rx.len = 1;
        f->rx.last_bits = 4;
        return ACE2K_RFID_POLL_DONE;
    }
    f->rx.len = 16;
    with_crc(&f->rx);
    if (f->bad_reads > 0) {
        f->bad_reads--;
        f->rx.data[16] ^= 0xFFU;
    }
    return ACE2K_RFID_POLL_DONE;
}

static enum ace2k_rfid_poll auth(struct fld *f, const uint8_t *tx, uint8_t len)
{
    struct ftag *t = active(f);
    if (!t || t->ntag || len != 12) {
        return ACE2K_RFID_POLL_TIMEOUT;
    }
    int sector = tx[1] / 4;
    bool uid_ok = memcmp(&tx[8], &t->uid[t->uid_len - 4], 4) == 0;
    if (tx[0] == 0x60 && uid_ok && memcmp(&tx[2], t->key_a[sector], 6) == 0) {
        t->authed_sector = sector;
        return ACE2K_RFID_POLL_DONE;
    }
    t->state = 0; /* a failed authentication leaves the tag idle: the next WUPA wakes it */
    return f->auth_nak ? ACE2K_RFID_POLL_AUTH_FAILED : ACE2K_RFID_POLL_TIMEOUT;
}

static int f_start(void *ctx, uint8_t reader, uint8_t cmd, const uint8_t *tx, uint8_t len,
                   uint8_t last_bits)
{
    struct fld *f = &((struct fake_field *)ctx)->rd[reader];
    f->starts++;
    f->firsts[f->nfirst++ % 1024] = tx[0];
    f->rx = (struct ace2k_rfid_rx){ 0 };
    if (cmd == ACE2K_RFID_CMD_MF_AUTHENT) {
        f->pending = auth(f, tx, len);
        return 0;
    }
    if ((tx[0] == 0x26 || tx[0] == 0x52) && len == 1 && last_bits == 7) {
        f->pending = wake(f, tx[0] == 0x52);
    } else if (tx[0] == 0x93 || tx[0] == 0x95 || tx[0] == 0x97) {
        int level = (tx[0] - 0x93) / 2;
        f->pending = tx[1] == 0x70 ? select_tag(f, tx, len, level) : anticoll(f, tx, level);
    } else if (tx[0] == 0x50 && len == 4 && crc_ok(tx, len)) {
        struct ftag *t = active(f);
        if (t) {
            t->state = 3;
        }
        f->pending = ACE2K_RFID_POLL_TIMEOUT;
    } else if (tx[0] == 0x30) {
        f->pending = read_tag(f, tx, len);
    } else {
        f->pending = ACE2K_RFID_POLL_TIMEOUT;
    }
    return 0;
}

static enum ace2k_rfid_poll f_poll(void *ctx, uint8_t reader, struct ace2k_rfid_rx *rx)
{
    struct fld *f = &((struct fake_field *)ctx)->rd[reader];
    f->polls++;
    if (f->mute) {
        return ACE2K_RFID_POLL_BUSY;
    }
    *rx = f->rx;
    enum ace2k_rfid_poll p = f->pending;
    f->pending = ACE2K_RFID_POLL_ERROR;
    return p;
}

static void f_crypto_off(void *ctx, uint8_t reader)
{
    struct fld *f = &((struct fake_field *)ctx)->rd[reader];
    f->crypto_offs++;
    for (int i = 0; i < f->n; i++) {
        f->t[i].authed_sector = -1;
    }
}

static const struct ace2k_iso14443a_ops ace2k_fld_ops
    __attribute__((unused)) = { .start = f_start, .poll = f_poll, .crypto_off = f_crypto_off };

__attribute__((unused)) static struct ftag *add_mifare(struct fld *f, const uint8_t uid[4])
{
    struct ftag *t = &f->t[f->n++];
    *t = (struct ftag){ .uid_len = 4, .atqa = 0x0004, .sak = 0x08, .authed_sector = -1 };
    memcpy(t->uid, uid, 4);
    for (int b = 0; b < 64; b++) {
        for (int i = 0; i < 16; i++) {
            t->blocks[b][i] = (uint8_t)((b * 16) + i);
        }
    }
    for (int s = 0; s < 16; s++) {
        uint8_t k[6] = { (uint8_t)s, 1, 2, 3, 4, 5 };
        memcpy(t->key_a[s], k, 6);
    }
    return t;
}

__attribute__((unused)) static struct ftag *add_ntag(struct fld *f, const uint8_t uid[7])
{
    struct ftag *t = &f->t[f->n++];
    *t = (struct ftag){
        .uid_len = 7,
        .atqa = 0x0044,
        .sak = 0x00,
        .ntag = true,
        .authed_sector = -1,
    };
    memcpy(t->uid, uid, 7);
    for (int p = 0; p < 64; p++) {
        for (int i = 0; i < 4; i++) {
            t->pages[p][i] = (uint8_t)((p * 4) + i);
        }
    }
    return t;
}

#endif
