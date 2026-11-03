/* The anticollision and the selection of the engine.  It works one cascade level at a time.  It
 * resolves collisions bit by bit, and builds the UID from the levels.  It was split from
 * iso14443a.c, and the seam is iso14443a_internal.h. */
#include "rfid/iso14443a_internal.h"

static uint8_t levels_of(uint8_t uid_len)
{
    return (uint8_t)((uid_len - 1U) / (ACE2K_ISO_CL_UID_BYTES - 1U));
}

static uint8_t sel_code(uint8_t level)
{
    return (uint8_t)(ACE2K_ISO_SEL_CL1 + (2U * level));
}

static uint8_t bcc_of(const uint8_t *cl)
{
    return (uint8_t)((uint32_t)cl[0] ^ cl[1] ^ cl[2] ^ cl[3]);
}

/* The cascade level of a known UID.  It is CT and three bytes, or the last four bytes, followed by
 * the BCC. */
static void cl_of_target(struct ace2k_iso14443a_engine *e)
{
    const struct ace2k_iso14443a_tag *t = &e->job.target;
    uint8_t base = (uint8_t)(e->level * (ACE2K_ISO_CL_UID_BYTES - 1U));
    if (e->level + 1U < levels_of(t->uid_len)) {
        e->cl[0] = ACE2K_ISO_CT;
        for (uint8_t i = 1; i < ACE2K_ISO_CL_UID_BYTES; i++) {
            e->cl[i] = t->uid[base + i - 1U];
        }
    } else {
        for (uint8_t i = 0; i < ACE2K_ISO_CL_UID_BYTES; i++) {
            e->cl[i] = t->uid[base + i];
        }
    }
    e->cl[ACE2K_ISO_CL_UID_BYTES] = bcc_of(e->cl);
}

int ace2k_iso14443a_send_anticoll(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    uint8_t full = (uint8_t)(e->known_bits / ACE2K_BITS_PER_BYTE);
    uint8_t part = (uint8_t)(e->known_bits % ACE2K_BITS_PER_BYTE);
    uint8_t tx[2 + ACE2K_ISO_CL_BYTES];
    tx[0] = sel_code(e->level);
    tx[1] = (uint8_t)(((2U + full) << ACE2K_ISO_NVB_BYTES_SHIFT) | part);
    uint8_t n = (uint8_t)(full + (part != 0 ? 1U : 0U));
    for (uint8_t i = 0; i < n; i++) {
        tx[2 + i] = e->cl[i];
    }
    return ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_TRANSCEIVE, tx, (uint8_t)(2U + n),
                                part);
}

int ace2k_iso14443a_send_select(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->job.kind != ACE2K_ISO_JOB_INVENTORY) {
        cl_of_target(e);
    }
    uint8_t tx[ACE2K_ISO_SELECT_FRAME];
    tx[0] = sel_code(e->level);
    tx[1] = ACE2K_ISO_NVB_SELECT;
    for (uint8_t i = 0; i < ACE2K_ISO_CL_BYTES; i++) {
        tx[2 + i] = e->cl[i];
    }
    ace2k_iso14443a_crc_a(tx, 2 + ACE2K_ISO_CL_BYTES, &tx[2 + ACE2K_ISO_CL_BYTES]);
    return ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_TRANSCEIVE, tx, ACE2K_ISO_SELECT_FRAME,
                                0);
}

void ace2k_iso14443a_begin_level(struct ace2k_iso14443a_engine *e)
{
    for (uint8_t i = 0; i < ACE2K_ISO_CL_BYTES; i++) {
        e->cl[i] = 0;
    }
    e->known_bits = 0;
    e->iterations = 0;
    e->phase = e->job.kind == ACE2K_ISO_JOB_INVENTORY ? ACE2K_ISO_PH_ANTICOLL : ACE2K_ISO_PH_SELECT;
}

/* The received bits land after the known ones.  So the first byte shares its low `part` bits with
 * the last byte sent (RxAlign). */
static void merge(struct ace2k_iso14443a_engine *e, const struct ace2k_rfid_rx *rx)
{
    uint8_t full = (uint8_t)(e->known_bits / ACE2K_BITS_PER_BYTE);
    uint8_t part = (uint8_t)(e->known_bits % ACE2K_BITS_PER_BYTE);
    uint32_t keep = (1U << part) - 1U;
    for (uint8_t i = 0; i < rx->len && full + i < ACE2K_ISO_CL_BYTES; i++) {
        uint8_t v = rx->data[i];
        if (i == 0 && part != 0) {
            v = (uint8_t)((e->cl[full] & keep) | (v & ~keep));
        }
        e->cl[full + i] = v;
    }
}

static void on_anticoll_done(struct ace2k_iso14443a *self, uint8_t reader,
                             const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    uint8_t full = (uint8_t)(e->known_bits / ACE2K_BITS_PER_BYTE);
    merge(e, rx);
    if (full + rx->len != ACE2K_ISO_CL_BYTES || rx->last_bits != 0 ||
        bcc_of(e->cl) != e->cl[ACE2K_ISO_CL_UID_BYTES]) {
        ace2k_iso14443a_end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    e->known_bits = ACE2K_ISO_CL_BITS;
    e->phase = ACE2K_ISO_PH_SELECT;
}

/* A collision at bit p.  The bits before it are shared.  From now on, bit p is known and taken as
 * 1.  The tags with a 0 there drop out of this round, and a later REQA finds them. */
static void on_anticoll_collision(struct ace2k_iso14443a *self, uint8_t reader,
                                  const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    uint8_t p = rx->coll_pos;
    if (p <= e->known_bits || p > ACE2K_ISO_COLL_BITS_MAX ||
        ++e->iterations > ACE2K_ISO_ANTICOLL_MAX) {
        ace2k_iso14443a_end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    merge(e, rx);
    uint8_t bit = (uint8_t)(p - 1U);
    e->cl[bit / ACE2K_BITS_PER_BYTE] |= (uint8_t)(1U << (bit % ACE2K_BITS_PER_BYTE));
    e->known_bits = p;
}

void ace2k_iso14443a_on_anticoll(struct ace2k_iso14443a *self, uint8_t reader,
                                 enum ace2k_rfid_poll p, const struct ace2k_rfid_rx *rx)
{
    if (p == ACE2K_RFID_POLL_DONE) {
        on_anticoll_done(self, reader, rx);
    } else if (p == ACE2K_RFID_POLL_COLLISION) {
        on_anticoll_collision(self, reader, rx);
    } else if (p == ACE2K_RFID_POLL_TIMEOUT) {
        ace2k_iso14443a_on_silence(self, reader);
    } else {
        ace2k_iso14443a_end(self, reader, ACE2K_ISO_ERROR);
    }
}

static void append_uid(struct ace2k_iso14443a_engine *e, const uint8_t *src, uint8_t n)
{
    for (uint8_t i = 0; i < n && e->cur.uid_len < ACE2K_ISO_UID_MAX; i++) {
        e->cur.uid[e->cur.uid_len++] = src[i];
    }
}

/* Returns true when the inventory already lists the tag just selected. */
static bool already_listed(const struct ace2k_iso14443a_engine *e)
{
    for (uint8_t i = 0; i < e->out.tag_count; i++) {
        if (ace2k_iso14443a_same_uid(&e->out.tags[i], &e->cur)) {
            return true;
        }
    }
    return false;
}

/* The tag answered its selection with a final SAK.  An inventory records the tag and halts it.  A
 * read goes on to its pages or to its authentication.
 * A tag whose HLTA was lost stays active.  It drops to idle at the next REQA without an answer, and
 * a later REQA selects it again.  The inventory lists it once, halts it again, and counts the
 * selection.  on_halt() ends the inventory after ACE2K_ISO_INVENTORY_MAX selections, so tags that
 * never halt cannot keep it going. */
static void on_selected(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->job.kind == ACE2K_ISO_JOB_INVENTORY) {
        e->selected++;
        if (e->out.tag_count < ACE2K_ISO_INVENTORY_MAX && !already_listed(e)) {
            e->out.tags[e->out.tag_count++] = e->cur;
        }
        e->phase = ACE2K_ISO_PH_HALT;
        return;
    }
    e->phase = e->job.kind == ACE2K_ISO_JOB_MIFARE_READ ? ACE2K_ISO_PH_AUTH : ACE2K_ISO_PH_READ;
}

void ace2k_iso14443a_on_select(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                               const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (p == ACE2K_RFID_POLL_TIMEOUT) {
        ace2k_iso14443a_on_silence(self, reader);
        return;
    }
    if (p != ACE2K_RFID_POLL_DONE || rx->len != ACE2K_ISO_SAK_FRAME ||
        !ace2k_iso14443a_crc_ok(rx)) {
        ace2k_iso14443a_end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    uint8_t sak = rx->data[0];
    if (sak & ACE2K_ISO_SAK_CASCADE) {
        if (e->cl[0] != ACE2K_ISO_CT || e->level + 1U >= ACE2K_ISO_LEVELS_MAX) {
            ace2k_iso14443a_end(self, reader, ACE2K_ISO_ERROR);
            return;
        }
        append_uid(e, &e->cl[1], ACE2K_ISO_CL_UID_BYTES - 1U);
        e->level++;
        ace2k_iso14443a_begin_level(e);
        return;
    }
    append_uid(e, e->cl, ACE2K_ISO_CL_UID_BYTES);
    e->cur.sak = sak;
    on_selected(self, reader);
}
