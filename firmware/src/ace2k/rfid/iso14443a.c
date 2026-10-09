#include "rfid/iso14443a.h"

enum ace2k_iso_phase {
    ACE2K_ISO_PH_IDLE = 0,
    ACE2K_ISO_PH_WAKE = 1,
    ACE2K_ISO_PH_ANTICOLL = 2,
    ACE2K_ISO_PH_SELECT = 3,
    ACE2K_ISO_PH_HALT = 4,
    ACE2K_ISO_PH_AUTH = 5,
    ACE2K_ISO_PH_READ = 6,
};

#define ACE2K_ISO_BYTE_MASK   0xFFU
#define ACE2K_ISO_CRC_SHIFT_A 4U
#define ACE2K_ISO_CRC_SHIFT_B 3U
#define ACE2K_ISO_NAK_BITS    4U

void ace2k_iso14443a_init(struct ace2k_iso14443a *self, const struct ace2k_iso14443a_ops *ops,
                          void *ctx)
{
    *self = (struct ace2k_iso14443a){ .ops = ops, .ctx = ctx };
}

void ace2k_iso14443a_crc_a(const uint8_t *data, uint8_t len, uint8_t out[2])
{
    uint32_t crc = ACE2K_ISO_CRC_A_INIT;
    for (uint8_t i = 0; i < len; i++) {
        uint32_t b = (data[i] ^ (crc & ACE2K_ISO_BYTE_MASK)) & ACE2K_ISO_BYTE_MASK;
        b = (b ^ (b << ACE2K_ISO_CRC_SHIFT_A)) & ACE2K_ISO_BYTE_MASK;
        crc = (crc >> ACE2K_BITS_PER_BYTE) ^ (b << ACE2K_BITS_PER_BYTE) ^
              (b << ACE2K_ISO_CRC_SHIFT_B) ^ (b >> ACE2K_ISO_CRC_SHIFT_A);
    }
    out[0] = (uint8_t)(crc & ACE2K_ISO_BYTE_MASK);
    out[1] = (uint8_t)((crc >> ACE2K_BITS_PER_BYTE) & ACE2K_ISO_BYTE_MASK);
}

bool ace2k_iso14443a_frame_allowed(uint8_t first)
{
    switch (first) {
    case ACE2K_ISO_REQA:
    case ACE2K_ISO_WUPA:
    case ACE2K_ISO_SEL_CL1:
    case ACE2K_ISO_SEL_CL2:
    case ACE2K_ISO_SEL_CL3:
    case ACE2K_ISO_HLTA:
    case ACE2K_ISO_READ:
    case ACE2K_ISO_AUTH_A:
    case ACE2K_ISO_AUTH_B:
        return true;
    default:
        return false;
    }
}

int ace2k_iso14443a_send(struct ace2k_iso14443a *self, uint8_t reader, uint8_t cmd,
                         const uint8_t *tx, uint8_t len, uint8_t last_bits)
{
    if (len == 0 || !ace2k_iso14443a_frame_allowed(tx[0])) {
        return -ACE2K_EREFUSED;
    }
    return self->ops->start(self->ctx, reader, cmd, tx, len, last_bits);
}

bool ace2k_iso14443a_same_uid(const struct ace2k_iso14443a_tag *a,
                              const struct ace2k_iso14443a_tag *b)
{
    if (a->uid_len != b->uid_len) {
        return false;
    }
    for (uint8_t i = 0; i < a->uid_len; i++) {
        if (a->uid[i] != b->uid[i]) {
            return false;
        }
    }
    return true;
}

static uint8_t levels_of(uint8_t uid_len)
{
    return (uint8_t)((uid_len - 1U) / (ACE2K_ISO_CL_UID_BYTES - 1U));
}

static bool uid_len_ok(uint8_t uid_len)
{
    /* early returns instead of `a || b`: in C that result is an int, and the lint refuses one
     * as a bool */
    if (uid_len == ACE2K_ISO_UID_SINGLE || uid_len == ACE2K_ISO_UID_DOUBLE) {
        return true;
    }
    return uid_len == ACE2K_ISO_UID_MAX;
}

static bool read_job_ok(const struct ace2k_iso14443a_job *job)
{
    if (!uid_len_ok(job->target.uid_len)) {
        return false;
    }
    if (job->kind == ACE2K_ISO_JOB_NTAG_READ) {
        if (job->count < 1U || job->count > ACE2K_ISO_READS_MAX) {
            return false;
        }
        return job->arg <= ACE2K_ISO_NTAG_LAST_FIRST_PAGE;
    }
    if (job->arg >= ACE2K_ISO_MIFARE_SECTORS || job->count == 0) {
        return false;
    }
    if ((job->count & ~(uint32_t)ACE2K_ISO_DATA_BLOCKS_MASK) != 0) {
        return false;
    }
    return job->key_type <= 1U;
}

bool ace2k_iso14443a_job_valid(const struct ace2k_iso14443a_job *job)
{
    if (job->kind == ACE2K_ISO_JOB_INVENTORY) {
        return true;
    }
    if (job->kind == ACE2K_ISO_JOB_NTAG_READ || job->kind == ACE2K_ISO_JOB_MIFARE_READ) {
        return read_job_ok(job);
    }
    return false;
}

static void plan_reads(struct ace2k_iso14443a_engine *e)
{
    e->reads_total = 0;
    for (uint8_t i = 0; i < ACE2K_ISO_READS_MAX; i++) {
        if (e->job.kind == ACE2K_ISO_JOB_NTAG_READ && i < e->job.count) {
            e->read_addr[e->reads_total++] =
                (uint8_t)(e->job.arg + (i * ACE2K_ISO_NTAG_PAGES_PER_READ));
        } else if (e->job.kind == ACE2K_ISO_JOB_MIFARE_READ &&
                   (((uint32_t)e->job.count >> i) & 1U) != 0) {
            e->read_addr[e->reads_total++] =
                (uint8_t)((e->job.arg * ACE2K_ISO_BLOCKS_PER_SECTOR) + i);
        }
    }
}

int ace2k_iso14443a_begin(struct ace2k_iso14443a *self, uint8_t reader,
                          const struct ace2k_iso14443a_job *job)
{
    if (reader >= ACE2K_RFID_READER_COUNT || !ace2k_iso14443a_job_valid(job)) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->phase != ACE2K_ISO_PH_IDLE) {
        return -ACE2K_EREFUSED;
    }
    *e = (struct ace2k_iso14443a_engine){ .job = *job, .phase = ACE2K_ISO_PH_WAKE };
    plan_reads(e);
    return 0;
}

bool ace2k_iso14443a_busy(const struct ace2k_iso14443a *self, uint8_t reader)
{
    if (reader >= ACE2K_RFID_READER_COUNT) {
        return false;
    }
    return self->r[reader].phase != ACE2K_ISO_PH_IDLE;
}

const struct ace2k_iso14443a_out *ace2k_iso14443a_result(const struct ace2k_iso14443a *self,
                                                         uint8_t reader)
{
    return &self->r[reader].out;
}

/* The job is over: the crypto unit cleared after any MIFARE job (after its HLTA, on success),
 * the job — and its key, if an authentication never started — wiped, the result kept. */
static void end(struct ace2k_iso14443a *self, uint8_t reader, uint8_t result)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->job.kind == ACE2K_ISO_JOB_MIFARE_READ) {
        self->ops->crypto_off(self->ctx, reader);
    }
    e->out.result = result;
    ace2k_wipe(&e->job, sizeof e->job); /* kind back to ACE2K_ISO_JOB_NONE */
    e->phase = ACE2K_ISO_PH_IDLE;
    e->waiting = false;
}

static uint8_t sel_code(uint8_t level)
{
    return (uint8_t)(ACE2K_ISO_SEL_CL1 + (2U * level));
}

static uint8_t bcc_of(const uint8_t *cl)
{
    return (uint8_t)((uint32_t)cl[0] ^ cl[1] ^ cl[2] ^ cl[3]);
}

/* The cascade level of a known UID: CT and three bytes, or the last four; then the BCC. */
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

static int send_wake(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    /* an inventory wakes every tag once (WUPA, the halted ones too), then only the idle ones
     * (REQA): each tag found is halted, so the next REQA finds the next; a read wakes its target
     * whatever state an earlier job left it in */
    bool wupa = true;
    if (e->job.kind == ACE2K_ISO_JOB_INVENTORY && e->woken) {
        wupa = false;
    }
    const uint8_t tx[1] = { wupa ? ACE2K_ISO_WUPA : ACE2K_ISO_REQA };
    e->woken = true;
    return ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_TRANSCEIVE, tx, 1,
                                ACE2K_ISO_SHORT_FRAME_BITS);
}

static int send_anticoll(struct ace2k_iso14443a *self, uint8_t reader)
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

static int send_select(struct ace2k_iso14443a *self, uint8_t reader)
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

static int send_halt(struct ace2k_iso14443a *self, uint8_t reader)
{
    uint8_t tx[ACE2K_ISO_READ_CMD_FRAME] = { ACE2K_ISO_HLTA, 0x00 };
    ace2k_iso14443a_crc_a(tx, 2, &tx[2]);
    return ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_TRANSCEIVE, tx,
                                ACE2K_ISO_READ_CMD_FRAME, 0);
}

static void wipe_key(struct ace2k_iso14443a_engine *e)
{
    ace2k_wipe(e->job.key, sizeof e->job.key);
}

static int send_auth(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    const struct ace2k_iso14443a_tag *t = &e->job.target;
    uint8_t tx[ACE2K_ISO_AUTH_FRAME];
    tx[0] = e->job.key_type != 0 ? ACE2K_ISO_AUTH_B : ACE2K_ISO_AUTH_A;
    tx[1] = (uint8_t)(e->job.arg * ACE2K_ISO_BLOCKS_PER_SECTOR);
    for (uint8_t i = 0; i < ACE2K_ISO_KEY_BYTES; i++) {
        tx[2 + i] = e->job.key[i];
    }
    for (uint8_t i = 0; i < ACE2K_ISO_AUTH_UID_BYTES; i++) {
        tx[2 + ACE2K_ISO_KEY_BYTES + i] = t->uid[t->uid_len - ACE2K_ISO_AUTH_UID_BYTES + i];
    }
    int rc =
        ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_MF_AUTHENT, tx, ACE2K_ISO_AUTH_FRAME, 0);
    wipe_key(e);               /* the reader's FIFO has it now; the engine keeps none (rule 7) */
    ace2k_wipe(tx, sizeof tx); /* volatile: a plain store to a dying local is dead code */
    return rc;
}

static int send_read(struct ace2k_iso14443a *self, uint8_t reader)
{
    const struct ace2k_iso14443a_engine *e = &self->r[reader];
    uint8_t tx[ACE2K_ISO_READ_CMD_FRAME] = { ACE2K_ISO_READ, e->read_addr[e->reads_done] };
    ace2k_iso14443a_crc_a(tx, 2, &tx[2]);
    return ace2k_iso14443a_send(self, reader, ACE2K_RFID_CMD_TRANSCEIVE, tx,
                                ACE2K_ISO_READ_CMD_FRAME, 0);
}

static int send_phase(struct ace2k_iso14443a *self, uint8_t reader)
{
    switch (self->r[reader].phase) {
    case ACE2K_ISO_PH_WAKE:
        return send_wake(self, reader);
    case ACE2K_ISO_PH_ANTICOLL:
        return send_anticoll(self, reader);
    case ACE2K_ISO_PH_SELECT:
        return send_select(self, reader);
    case ACE2K_ISO_PH_HALT:
        return send_halt(self, reader);
    case ACE2K_ISO_PH_AUTH:
        return send_auth(self, reader);
    case ACE2K_ISO_PH_READ:
        return send_read(self, reader);
    default:
        return -ACE2K_EINVAL;
    }
}

static bool crc_ok(const struct ace2k_rfid_rx *rx)
{
    uint8_t c[2];
    ace2k_iso14443a_crc_a(rx->data, (uint8_t)(rx->len - 2U), c);
    if (c[0] != rx->data[rx->len - 2U]) {
        return false;
    }
    return c[1] == rx->data[rx->len - 1U];
}

/* No answer: an inventory has found every tag there is (OK, whatever it holds); a read's
 * target has left. */
static void on_silence(struct ace2k_iso14443a *self, uint8_t reader)
{
    bool inventory = self->r[reader].job.kind == ACE2K_ISO_JOB_INVENTORY;
    end(self, reader, inventory ? ACE2K_ISO_OK : ACE2K_ISO_TAG_GONE);
}

static void begin_level(struct ace2k_iso14443a_engine *e)
{
    for (uint8_t i = 0; i < ACE2K_ISO_CL_BYTES; i++) {
        e->cl[i] = 0;
    }
    e->known_bits = 0;
    e->iterations = 0;
    e->phase = e->job.kind == ACE2K_ISO_JOB_INVENTORY ? ACE2K_ISO_PH_ANTICOLL : ACE2K_ISO_PH_SELECT;
}

/* A read's first silence after its WUPA wakes once more before it says TAG_GONE: a tag an
 * earlier job left ACTIVE or AUTHENTICATED (a job that ended in error sends no HLTA) takes that
 * WUPA as an unexpected command and drops to IDLE or HALT without an answer — the second WUPA
 * wakes it.  The retry is the next step's one exchange (rule 6); an inventory's silence is
 * final. */
static void on_wake(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                    const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (p == ACE2K_RFID_POLL_TIMEOUT) {
        if (e->job.kind != ACE2K_ISO_JOB_INVENTORY && !e->rewoken) {
            e->rewoken = true; /* the phase stays WAKE: the WUPA again */
            return;
        }
        on_silence(self, reader);
        return;
    }
    if (p != ACE2K_RFID_POLL_DONE && p != ACE2K_RFID_POLL_COLLISION) {
        end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    e->cur = (struct ace2k_iso14443a_tag){ 0 };
    if (p == ACE2K_RFID_POLL_DONE && rx->len == 2) {
        e->cur.atqa = ace2k_get_u16_le(rx->data);
    }
    e->level = 0;
    begin_level(e);
}

/* The received bits land after the known ones: the first byte shares its low `part` bits with
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
        end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    e->known_bits = ACE2K_ISO_CL_BITS;
    e->phase = ACE2K_ISO_PH_SELECT;
}

/* A collision at bit p: the bits before it are shared, p is known from now on and taken as 1 —
 * the tags with a 0 there drop out of this round and are found by a later REQA. */
static void on_anticoll_collision(struct ace2k_iso14443a *self, uint8_t reader,
                                  const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    uint8_t p = rx->coll_pos;
    if (p <= e->known_bits || p > ACE2K_ISO_COLL_BITS_MAX ||
        ++e->iterations > ACE2K_ISO_ANTICOLL_MAX) {
        end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    merge(e, rx);
    uint8_t bit = (uint8_t)(p - 1U);
    e->cl[bit / ACE2K_BITS_PER_BYTE] |= (uint8_t)(1U << (bit % ACE2K_BITS_PER_BYTE));
    e->known_bits = p;
}

static void on_anticoll(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                        const struct ace2k_rfid_rx *rx)
{
    if (p == ACE2K_RFID_POLL_DONE) {
        on_anticoll_done(self, reader, rx);
    } else if (p == ACE2K_RFID_POLL_COLLISION) {
        on_anticoll_collision(self, reader, rx);
    } else if (p == ACE2K_RFID_POLL_TIMEOUT) {
        on_silence(self, reader);
    } else {
        end(self, reader, ACE2K_ISO_ERROR);
    }
}

static void append_uid(struct ace2k_iso14443a_engine *e, const uint8_t *src, uint8_t n)
{
    for (uint8_t i = 0; i < n && e->cur.uid_len < ACE2K_ISO_UID_MAX; i++) {
        e->cur.uid[e->cur.uid_len++] = src[i];
    }
}

/* The tag answered its selection with a final SAK: an inventory records and halts it, a read
 * goes on to its pages or its authentication. */
static void on_selected(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->job.kind == ACE2K_ISO_JOB_INVENTORY) {
        if (e->out.tag_count < ACE2K_ISO_INVENTORY_MAX) {
            e->out.tags[e->out.tag_count++] = e->cur;
        }
        e->phase = ACE2K_ISO_PH_HALT;
        return;
    }
    e->phase = e->job.kind == ACE2K_ISO_JOB_MIFARE_READ ? ACE2K_ISO_PH_AUTH : ACE2K_ISO_PH_READ;
}

static void on_select(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                      const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (p == ACE2K_RFID_POLL_TIMEOUT) {
        on_silence(self, reader);
        return;
    }
    if (p != ACE2K_RFID_POLL_DONE || rx->len != ACE2K_ISO_SAK_FRAME || !crc_ok(rx)) {
        end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    uint8_t sak = rx->data[0];
    if (sak & ACE2K_ISO_SAK_CASCADE) {
        if (e->cl[0] != ACE2K_ISO_CT || e->level + 1U >= ACE2K_ISO_LEVELS_MAX) {
            end(self, reader, ACE2K_ISO_ERROR);
            return;
        }
        append_uid(e, &e->cl[1], ACE2K_ISO_CL_UID_BYTES - 1U);
        e->level++;
        begin_level(e);
        return;
    }
    append_uid(e, e->cl, ACE2K_ISO_CL_UID_BYTES);
    e->cur.sak = sak;
    on_selected(self, reader);
}

/* HLTA has no answer: the next step moves on without polling it — the next start's Idle ends
 * the reader's wait for one.  A read job's HLTA is its last frame: the job ends OK (end() then
 * clears the crypto unit); an inventory wakes the next tag. */
static void on_halt(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->job.kind != ACE2K_ISO_JOB_INVENTORY) {
        end(self, reader, ACE2K_ISO_OK);
        return;
    }
    if (e->out.tag_count >= ACE2K_ISO_INVENTORY_MAX) {
        end(self, reader, ACE2K_ISO_OK);
        return;
    }
    e->phase = ACE2K_ISO_PH_WAKE;
}

/* A silent authentication is a wrong key: a MIFARE Classic tag does not answer an
 * authentication with a key other than the sector's, so the reader's timer ends it; the SELECT
 * just before proved the tag was there (if it left since, the next step's WUPA says TAG_GONE).
 * Any other ending but MFCrypto1On — an error, or the command done without it — is the same. */
static void on_auth(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p)
{
    if (p == ACE2K_RFID_POLL_DONE) {
        self->r[reader].phase = ACE2K_ISO_PH_READ;
        return;
    }
    end(self, reader, ACE2K_ISO_AUTH_FAILED);
}

static void on_read(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                    const struct ace2k_rfid_rx *rx)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (p == ACE2K_RFID_POLL_TIMEOUT) {
        on_silence(self, reader);
        return;
    }
    /* a NAK is four bits; anything but 16 bytes and a good CRC_A is an error */
    if (p != ACE2K_RFID_POLL_DONE || rx->len != ACE2K_ISO_READ_FRAME || rx->last_bits != 0 ||
        !crc_ok(rx)) {
        end(self, reader, ACE2K_ISO_ERROR);
        return;
    }
    uint8_t k = e->reads_done;
    for (uint8_t i = 0; i < ACE2K_ISO_BLOCK_BYTES; i++) {
        e->out.data[k][i] = rx->data[i];
    }
    e->out.data_first[k] = e->read_addr[k];
    e->out.data_count = ++e->reads_done;
    if (e->reads_done == e->reads_total) {
        /* halt the tag before the job ends — for MIFARE while Crypto1 is on, so the reader
         * encrypts the HLTA — or the next job's WUPA finds it still ACTIVE and gets no answer */
        e->phase = ACE2K_ISO_PH_HALT;
    }
}

static void handle(struct ace2k_iso14443a *self, uint8_t reader, enum ace2k_rfid_poll p,
                   const struct ace2k_rfid_rx *rx)
{
    switch (self->r[reader].phase) {
    case ACE2K_ISO_PH_WAKE:
        on_wake(self, reader, p, rx);
        break;
    case ACE2K_ISO_PH_ANTICOLL:
        on_anticoll(self, reader, p, rx);
        break;
    case ACE2K_ISO_PH_SELECT:
        on_select(self, reader, p, rx);
        break;
    case ACE2K_ISO_PH_AUTH:
        on_auth(self, reader, p);
        break;
    case ACE2K_ISO_PH_READ:
        on_read(self, reader, p, rx);
        break;
    default:
        end(self, reader, ACE2K_ISO_ERROR);
        break;
    }
}

/* The exchange running: polled, or given up; HLTA is never polled.  True when the step may go
 * on to start the next exchange. */
static bool poll_running(struct ace2k_iso14443a *self, uint8_t reader)
{
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->phase == ACE2K_ISO_PH_HALT) {
        e->waiting = false;
        on_halt(self, reader);
        return e->phase != ACE2K_ISO_PH_IDLE;
    }
    struct ace2k_rfid_rx rx;
    enum ace2k_rfid_poll p = self->ops->poll(self->ctx, reader, &rx);
    if (p == ACE2K_RFID_POLL_BUSY) {
        if (++e->polls >= ACE2K_ISO_POLL_MAX) {
            end(self, reader, ACE2K_ISO_ERROR);
        }
        return false;
    }
    e->waiting = false;
    handle(self, reader, p, &rx);
    return e->phase != ACE2K_ISO_PH_IDLE;
}

bool ace2k_iso14443a_step(struct ace2k_iso14443a *self, uint8_t reader)
{
    if (!ace2k_iso14443a_busy(self, reader)) {
        return false;
    }
    struct ace2k_iso14443a_engine *e = &self->r[reader];
    if (e->waiting && !poll_running(self, reader)) {
        return e->phase == ACE2K_ISO_PH_IDLE;
    }
    if (send_phase(self, reader) != 0) {
        end(self, reader, ACE2K_ISO_ERROR);
        return true;
    }
    e->waiting = true;
    e->polls = 0;
    return false;
}
