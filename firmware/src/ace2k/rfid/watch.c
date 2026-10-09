#include "rfid/watch.h"

static bool bit(uint8_t mask, uint8_t lane)
{
    return (((uint32_t)mask >> lane) & 1U) != 0;
}

/* The two lanes of a reader: B serves 0 and 1, A 2 and 3. */
static uint8_t first_lane(uint8_t reader)
{
    return reader == ACE2K_RFID_READER_B ? 0U : 2U;
}

void ace2k_rfid_watch_init(struct ace2k_rfid_watch *self, struct ace2k_iso14443a *iso,
                           const struct ace2k_rfid_watch_ops *ops, void *ctx)
{
    *self = (struct ace2k_rfid_watch){ .iso = iso, .ops = ops, .ctx = ctx };
}

static void push(struct ace2k_rfid_watch *self, const struct ace2k_rfid_event *e)
{
    uint8_t next = (uint8_t)((self->head + 1U) % ACE2K_RFID_EVENT_RING);
    if (next == self->tail) {
        self->dropped++;
        return;
    }
    self->ring[self->head] = *e;
    self->head = next;
}

static void push_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t kind)
{
    struct ace2k_rfid_event e = { .type = ACE2K_RFID_EV_STATE, .lane = lane, .kind = kind };
    push(self, &e);
}

static void set_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t state)
{
    if (self->l[lane].state == state) {
        return;
    }
    self->l[lane].state = state;
    if (state != ACE2K_RFID_READING) { /* a session's opening is its tag event */
        push_state(self, lane, state);
    }
}

/* Through ace2k_wipe(): the local job of step_request dies right after, and a plain zeroing
 * would be a dead store (rule 7). */
static void wipe_job(struct ace2k_iso14443a_job *job)
{
    ace2k_wipe(job, sizeof *job);
}

/* The state a lane goes back to when its tag is not read: searching while the load runs. */
static uint8_t unread_state(const struct ace2k_rfid_watch_lane *l)
{
    return l->loading_last ? (uint8_t)ACE2K_RFID_SEARCHING : (uint8_t)ACE2K_RFID_PENDING;
}

static void open_session(struct ace2k_rfid_watch *self, uint8_t lane,
                         const struct ace2k_iso14443a_tag *tag, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    if (++self->next_session == 0U) { /* explicit: cppcheck misreads a separate wrap test */
        self->next_session = 1;       /* 0 is no session */
    }
    rd->session_open = true;
    rd->session_lane = lane;
    rd->session_id = self->next_session;
    rd->t_session_ms = now_ms;
    rd->step_pending = false;
    l->tag = *tag;
    l->has_tag = true;
    l->lost = false;
    l->want = false;
    set_state(self, lane, ACE2K_RFID_READING);
    struct ace2k_rfid_event e = {
        .type = ACE2K_RFID_EV_TAG,
        .lane = lane,
        .session = rd->session_id,
        .tag = *tag,
    };
    push(self, &e);
}

static bool eligible(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    uint8_t s = self->l[lane].state;
    if (!self->in.link_ok || !bit(self->in.insert, lane)) {
        return false;
    }
    if (s == ACE2K_RFID_PENDING || s == ACE2K_RFID_SEARCHING) {
        return true;
    }
    return false;
}

/* A lane whose UID was attributed while the reader's session was busy gets its turn — once no
 * session is open and no job runs on the reader: a step of the session just closed may still
 * be running, and its result must not land in the next session. */
static void serve_wants(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    bool busy = ace2k_iso14443a_busy(self->iso, reader);
    for (uint8_t i = 0; i < 2U; i++) {
        uint8_t lane = (uint8_t)(first_lane(reader) + i);
        if (!self->l[lane].want) {
            continue;
        }
        if (!eligible(self, lane)) {
            self->l[lane].want = false; /* the strand left, the link went, the lane was read */
        } else if (!self->r[reader].session_open && !busy) {
            open_session(self, lane, &self->l[lane].wanted, now_ms); /* clears the want */
        }
    }
}

/* The session ends; a lane still reading goes to `state`. */
static void close_session(struct ace2k_rfid_watch *self, uint8_t reader, uint8_t state,
                          uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (!rd->session_open) {
        return;
    }
    rd->session_open = false;
    rd->step_pending = false;
    wipe_job(&rd->step_job);
    if (self->l[rd->session_lane].state == ACE2K_RFID_READING) {
        set_state(self, rd->session_lane, state);
    }
    serve_wants(self, reader, now_ms);
}

/* A read on command no inventory decided: the host's wait ends on the lane's unchanged state. */
static void answer_probe(struct ace2k_rfid_watch *self, uint8_t lane)
{
    self->l[lane].probe = false;
    push_state(self, lane, self->l[lane].state);
}

static void answer_probes(struct ace2k_rfid_watch *self, uint8_t reader)
{
    for (uint8_t i = 0; i < 2U; i++) {
        uint8_t lane = (uint8_t)(first_lane(reader) + i);
        if (self->l[lane].probe) {
            answer_probe(self, lane);
        }
    }
}

static void forget_tag(struct ace2k_rfid_watch_lane *l)
{
    l->tag = (struct ace2k_iso14443a_tag){ 0 };
    l->has_tag = false;
    l->lost = false;
    l->probe = false;
    l->want = false;
}

/* The strand left the mouth: the lane forgets everything, its session included. */
static void lane_emptied(struct ace2k_rfid_watch *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->session_open && rd->session_lane == lane) {
        close_session(self, ace2k_rfid_reader_of(lane), ACE2K_RFID_UNKNOWN, now_ms);
    }
    forget_tag(&self->l[lane]);
    set_state(self, lane, ACE2K_RFID_UNKNOWN);
}

/* The feed's load starts or ends on the lane. */
static void lane_load_edge(struct ace2k_rfid_watch *self, uint8_t lane,
                           const struct ace2k_rfid_watch_inputs *in)
{
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    bool loading = bit(in->loading, lane);
    if (loading && !l->loading_last &&
        (l->state == ACE2K_RFID_PENDING || l->state == ACE2K_RFID_NO_TAG)) {
        set_state(self, lane, ACE2K_RFID_SEARCHING);
    }
    if (!loading && l->loading_last && l->state == ACE2K_RFID_SEARCHING) {
        uint8_t next = ACE2K_RFID_PENDING;
        if (bit(in->exhausted, lane) && !l->has_tag) {
            next = ACE2K_RFID_NO_TAG; /* rule 9: no UID seen in the whole search */
        }
        set_state(self, lane, next);
    }
    l->loading_last = loading;
}

static void lane_update(struct ace2k_rfid_watch *self, uint8_t lane,
                        const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms)
{
    if (!bit(in->insert, lane)) {
        if (self->l[lane].state != ACE2K_RFID_UNKNOWN || self->l[lane].has_tag) {
            lane_emptied(self, lane, now_ms);
        }
        self->l[lane].loading_last = false;
        return;
    }
    if (self->l[lane].state == ACE2K_RFID_UNKNOWN) {
        set_state(self, lane, ACE2K_RFID_PENDING); /* inserted, or present at boot */
    }
    lane_load_edge(self, lane, in);
}

static bool lane_watched(const struct ace2k_rfid_watch *self, uint8_t lane,
                         const struct ace2k_rfid_watch_inputs *in)
{
    uint8_t s = self->l[lane].state;
    if (s != ACE2K_RFID_PENDING && s != ACE2K_RFID_SEARCHING) {
        return false;
    }
    if (bit(in->moving, lane) || bit(in->loading, lane)) {
        return true;
    }
    return false;
}

static bool reader_needed(const struct ace2k_rfid_watch *self, uint8_t reader,
                          const struct ace2k_rfid_watch_inputs *in)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (rd->session_open) {
        return true;
    }
    for (uint8_t i = 0; i < 2U; i++) {
        uint8_t lane = (uint8_t)(first_lane(reader) + i);
        if (self->l[lane].probe || self->l[lane].want) {
            return true;
        }
        if (!rd->suppressed && lane_watched(self, lane, in)) {
            return true;
        }
    }
    return false;
}

static bool any_lane_moving(uint8_t reader, const struct ace2k_rfid_watch_inputs *in)
{
    uint8_t first = first_lane(reader);
    if (bit(in->moving, first) || bit(in->moving, (uint8_t)(first + 1U))) {
        return true;
    }
    return false;
}

static void field_off(struct ace2k_rfid_watch *self, uint8_t reader)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (!rd->field) {
        return;
    }
    self->ops->field_set(self->ctx, reader, false);
    if (self->ops->field_driven(self->ctx, reader)) {
        rd->fault = true; /* rule 2: latched, reported through the health mask */
    }
    rd->field = false;
}

/* A configure is a soft reset polled for up to 150 ms (rfid/reader.h): a failed one is not
 * tried again before ACE2K_RFID_CONFIGURE_RETRY_MS, and ACE2K_RFID_CONFIGURE_TRIES in a row
 * make the reader dead — its reads on command are answered, the health bit fails. */
static void configure(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (rd->dead) {
        return;
    }
    if (rd->configure_failures > 0U &&
        ace2k_time_since(now_ms, rd->t_configure_failed_ms) < ACE2K_RFID_CONFIGURE_RETRY_MS) {
        return;
    }
    if (self->ops->configure(self->ctx, reader) == 0) {
        rd->configure_failures = 0;
        return;
    }
    rd->t_configure_failed_ms = now_ms;
    if (++rd->configure_failures >= ACE2K_RFID_CONFIGURE_TRIES) {
        rd->dead = true;
        answer_probes(self, reader);
    }
}

/* Configure first when a soft reset cleared the reader (the next run turns the field on). */
static void field_on(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (self->ops->needs_configure(self->ctx, reader)) {
        configure(self, reader, now_ms);
        return;
    }
    self->ops->field_set(self->ctx, reader, true);
    rd->field = true;
    rd->t_last_moving_ms = now_ms;
    /* the first inventory once the tags have powered up (ISO/IEC 14443-3: at least 5 ms of
     * field before the first command), on the next task run */
    rd->t_last_inventory_ms = now_ms - ACE2K_RFID_POLL_MS + ACE2K_RFID_POWER_UP_MS;
    rd->baseline_taken = false;
    rd->present_count = 0;
}

/* The ceiling (rule 2): a field on with no lane of the reader moving and no session. */
static bool ceiling_reached(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms,
                            const struct ace2k_rfid_watch_inputs *in)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (any_lane_moving(reader, in)) {
        rd->t_last_moving_ms = now_ms;
        rd->suppressed = false;
        return false;
    }
    if (!rd->field || rd->session_open ||
        ace2k_time_since(now_ms, rd->t_last_moving_ms) < ACE2K_RFID_FIELD_MAX_MS) {
        return false;
    }
    rd->suppressed = true;
    return true;
}

static bool in_list(const struct ace2k_iso14443a_tag *list, uint8_t n,
                    const struct ace2k_iso14443a_tag *t)
{
    for (uint8_t i = 0; i < n; i++) {
        if (ace2k_iso14443a_same_uid(&list[i], t)) {
            return true;
        }
    }
    return false;
}

/* Attribution rule 1 (protocol.md, "Watching and attribution"): the neighbour's tag, read or being read, is never this lane's. */
static bool owned_by_neighbour(const struct ace2k_rfid_watch *self, uint8_t lane,
                               const struct ace2k_iso14443a_tag *t)
{
    const struct ace2k_rfid_watch_lane *n = &self->l[ace2k_rfid_neighbour(lane)];
    if (n->state != ACE2K_RFID_READ && n->state != ACE2K_RFID_READING) {
        return false;
    }
    return ace2k_iso14443a_same_uid(&n->tag, t);
}

/* Attribution rules 2–4 and the lane's own known UID, for a tag the inventory saw. */
static bool claims(const struct ace2k_rfid_watch *self, uint8_t lane,
                   const struct ace2k_iso14443a_tag *t, bool entered)
{
    const struct ace2k_rfid_watch_inputs *in = &self->in;
    uint8_t n = ace2k_rfid_neighbour(lane);
    if (self->l[lane].has_tag && ace2k_iso14443a_same_uid(&self->l[lane].tag, t)) {
        return true;
    }
    if (!bit(in->insert, n)) {
        return true;
    }
    if (!entered || !bit(in->moving, lane)) {
        return false;
    }
    if (!bit(in->moving, n) || self->l[n].state == ACE2K_RFID_READ) {
        return true;
    }
    return false;
}

static void take(struct ace2k_rfid_watch *self, uint8_t lane, const struct ace2k_iso14443a_tag *t,
                 uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->session_open) {
        self->l[lane].want = true;
        self->l[lane].wanted = *t;
        return;
    }
    open_session(self, lane, t, now_ms);
}

static void attribute_lane(struct ace2k_rfid_watch *self, uint8_t reader, uint8_t lane,
                           const struct ace2k_iso14443a_out *o, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    for (uint8_t i = 0; i < o->tag_count; i++) {
        const struct ace2k_iso14443a_tag *t = &o->tags[i];
        bool entered = true;
        if (in_list(rd->present, rd->present_count, t)) {
            entered = false;
        }
        if (!owned_by_neighbour(self, lane, t) && claims(self, lane, t, entered)) {
            take(self, lane, t, now_ms);
            return;
        }
    }
}

/* A read on command: attribution rules 1–2, and a neighbour whose tag is read owns no other UID. */
static void probe_lane(struct ace2k_rfid_watch *self, uint8_t lane,
                       const struct ace2k_iso14443a_out *o, uint32_t now_ms)
{
    self->l[lane].probe = false;
    uint8_t n = ace2k_rfid_neighbour(lane);
    const struct ace2k_iso14443a_tag *only = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < o->tag_count; i++) {
        if (!owned_by_neighbour(self, lane, &o->tags[i])) {
            only = &o->tags[i];
            count++;
        }
    }
    bool decided = false;
    if (!bit(self->in.insert, n) || self->l[n].state == ACE2K_RFID_READ) {
        decided = true;
    }
    if (count == 1 && decided && self->in.link_ok) {
        take(self, lane, only, now_ms);
        return;
    }
    push_state(self, lane, count == 0 ? self->l[lane].state : (uint8_t)ACE2K_RFID_KIND_AMBIGUOUS);
}

static void keep_present(struct ace2k_rfid_watch_reader *rd, const struct ace2k_iso14443a_out *o)
{
    rd->present_count = o->tag_count;
    for (uint8_t i = 0; i < o->tag_count; i++) {
        rd->present[i] = o->tags[i];
    }
}

static void inventory_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(self->iso, reader);
    if (o->result != ACE2K_ISO_OK) {
        return; /* a partial inventory changes nothing */
    }
    if (!rd->baseline_taken) {
        rd->baseline_taken = true; /* the tags that were there at rest */
        keep_present(rd, o);
    }
    for (uint8_t i = 0; i < 2U; i++) {
        uint8_t lane = (uint8_t)(first_lane(reader) + i);
        if (self->l[lane].probe) {
            probe_lane(self, lane, o, now_ms);
        } else if (eligible(self, lane)) {
            attribute_lane(self, reader, lane, o, now_ms);
        }
    }
    keep_present(rd, o);
}

static void step_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(self->iso, reader);
    rd->step_running = false;
    if (!rd->session_open || rd->session_lane != rd->step_lane ||
        rd->session_id != rd->step_session) {
        return; /* its session ended meanwhile: the bytes go nowhere, no lane is lost */
    }
    struct ace2k_rfid_event e = {
        .type = ACE2K_RFID_EV_DATA,
        .lane = rd->session_lane,
        .session = rd->session_id,
    };
    for (uint8_t i = 0; i < o->data_count; i++) {
        e.block = o->data_first[i];
        for (uint8_t b = 0; b < ACE2K_ISO_BLOCK_BYTES; b++) {
            e.data[b] = o->data[i][b];
        }
        push(self, &e);
    }
    rd->t_session_ms = now_ms;
    if (o->result == ACE2K_ISO_OK) {
        return;
    }
    e = (struct ace2k_rfid_event){
        .type = ACE2K_RFID_EV_DATA,
        .lane = rd->session_lane,
        .session = rd->session_id,
        .kind = o->result,
        .block = rd->step_first,
    };
    push(self, &e);
    if (o->result == ACE2K_ISO_TAG_GONE) {
        self->l[rd->session_lane].lost = true;
        close_session(self, reader, unread_state(&self->l[rd->session_lane]), now_ms);
    }
}

static void begin_inventory(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    if (ace2k_iso14443a_begin(self->iso, reader, &job) == 0) {
        self->r[reader].t_last_inventory_ms = now_ms;
    }
}

/* Nothing running on the reader: the host's step, the session's idle end, or an inventory. */
static void next_job(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (rd->session_open && rd->step_pending) {
        if (ace2k_iso14443a_begin(self->iso, reader, &rd->step_job) == 0) {
            rd->step_running = true;
            rd->step_lane = rd->session_lane;
            rd->step_session = rd->session_id;
        }
        rd->step_pending = false;
        wipe_job(&rd->step_job); /* the engine has its copy; the watch keeps no key (rule 7) */
        return;
    }
    if (rd->session_open) {
        if (ace2k_time_since(now_ms, rd->t_session_ms) >= ACE2K_RFID_SESSION_IDLE_MS) {
            close_session(self, reader, unread_state(&self->l[rd->session_lane]), now_ms);
        }
        return;
    }
    serve_wants(self, reader, now_ms); /* one deferred while the closed session's job ran */
    if (rd->session_open) {
        return;
    }
    if (ace2k_time_since(now_ms, rd->t_last_inventory_ms) >= ACE2K_RFID_POLL_MS) {
        begin_inventory(self, reader, now_ms);
    }
}

static void reader_step(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms,
                        const struct ace2k_rfid_watch_inputs *in)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (ace2k_iso14443a_busy(self->iso, reader)) {
        if (ace2k_iso14443a_step(self->iso, reader)) {
            if (rd->step_running) {
                step_done(self, reader, now_ms);
            } else {
                inventory_done(self, reader, now_ms);
            }
        }
        return;
    }
    if (ceiling_reached(self, reader, now_ms, in) || !reader_needed(self, reader, in)) {
        field_off(self, reader);
        return;
    }
    if (!rd->field) {
        field_on(self, reader, now_ms);
        return;
    }
    next_job(self, reader, now_ms);
}

void ace2k_rfid_watch_step(struct ace2k_rfid_watch *self, uint32_t now_ms,
                           const struct ace2k_rfid_watch_inputs *in)
{
    self->in = *in;
    self->now_ms = now_ms;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        lane_update(self, lane, in, now_ms);
        if (self->l[lane].probe &&
            ace2k_time_since(now_ms, self->l[lane].t_probe_ms) >= ACE2K_RFID_PROBE_MAX_MS) {
            answer_probe(self, lane); /* no inventory decided it in time */
        }
    }
    if (!in->link_ok) {
        for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
            if (self->r[r].session_open) {
                close_session(self, r, unread_state(&self->l[self->r[r].session_lane]), now_ms);
            }
        }
    }
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        reader_step(self, r, now_ms, in);
    }
    self->primed = true;
}

static bool step_args_ok(uint8_t op, uint8_t arg, uint8_t count, struct ace2k_iso14443a_job *job)
{
    if (op == ACE2K_RFID_OP_NTAG) {
        job->kind = ACE2K_ISO_JOB_NTAG_READ;
    } else if (op == ACE2K_RFID_OP_MIFARE_A || op == ACE2K_RFID_OP_MIFARE_B) {
        job->kind = ACE2K_ISO_JOB_MIFARE_READ;
        job->key_type = op == ACE2K_RFID_OP_MIFARE_B ? 1U : 0U;
    } else {
        return false;
    }
    job->arg = arg;
    job->count = count;
    job->target.uid_len = ACE2K_ISO_UID_SINGLE; /* for the bounds check; the session's below */
    return ace2k_iso14443a_job_valid(job);
}

int ace2k_rfid_watch_step_request(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session,
                                  uint8_t op, uint8_t arg, uint8_t count, const uint8_t *key,
                                  uint32_t now_ms)
{
    struct ace2k_iso14443a_job job = { 0 };
    if (lane >= ACE2K_LANE_COUNT || !step_args_ok(op, arg, count, &job)) {
        return -ACE2K_EINVAL;
    }
    struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (!rd->session_open || rd->session_lane != lane || rd->session_id != session ||
        rd->step_pending || rd->step_running) {
        return -ACE2K_EREFUSED;
    }
    job.target = self->l[lane].tag;
    if (job.kind == ACE2K_ISO_JOB_MIFARE_READ) {
        for (uint8_t i = 0; i < ACE2K_ISO_KEY_BYTES; i++) {
            job.key[i] = key[i];
        }
    }
    rd->step_job = job;
    rd->step_first = op == ACE2K_RFID_OP_NTAG ? arg : (uint8_t)(arg * ACE2K_ISO_BLOCKS_PER_SECTOR);
    rd->step_pending = true;
    rd->t_session_ms = now_ms;
    wipe_job(&job);
    return 0;
}

int ace2k_rfid_watch_done(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t session, bool read)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return -ACE2K_EINVAL;
    }
    uint8_t reader = ace2k_rfid_reader_of(lane);
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (!rd->session_open || rd->session_lane != lane || rd->session_id != session) {
        return -ACE2K_EREFUSED;
    }
    uint8_t state = read ? (uint8_t)ACE2K_RFID_READ : unread_state(&self->l[lane]);
    /* the last step's clock, not the session's: close_session may open the next lane's session,
     * whose idle timer must start now, not at this one's last step */
    close_session(self, reader, state, self->now_ms);
    return 0;
}

enum ace2k_rfid_read_refusal ace2k_rfid_watch_probe(struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT || !bit(self->in.insert, lane)) {
        return ACE2K_RFID_READ_NO_FILAMENT;
    }
    if (!self->in.link_ok) {
        return ACE2K_RFID_READ_NO_LINK;
    }
    if (self->l[lane].state == ACE2K_RFID_READING || self->l[lane].probe) {
        return ACE2K_RFID_READ_BUSY;
    }
    struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->dead) {
        push_state(self, lane, self->l[lane].state); /* accepted, answered at once */
        return ACE2K_RFID_READ_OK;
    }
    self->l[lane].probe = true;
    self->l[lane].t_probe_ms = self->now_ms;
    rd->suppressed = false;
    return ACE2K_RFID_READ_OK;
}

void ace2k_rfid_watch_forget(struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return;
    }
    uint8_t s = self->l[lane].state;
    if (s == ACE2K_RFID_READ || s == ACE2K_RFID_NO_TAG) {
        forget_tag(&self->l[lane]);
        set_state(self, lane, ACE2K_RFID_PENDING);
    }
}

bool ace2k_rfid_watch_lane_busy(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return false;
    }
    if (self->l[lane].probe) {
        return true;
    }
    return self->l[lane].state == ACE2K_RFID_READING;
}

uint8_t ace2k_rfid_watch_state(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].state : (uint8_t)ACE2K_RFID_UNKNOWN;
}

const struct ace2k_iso14443a_tag *ace2k_rfid_watch_tag(const struct ace2k_rfid_watch *self,
                                                       uint8_t lane)
{
    return &self->l[lane < ACE2K_LANE_COUNT ? lane : 0U].tag;
}

static uint8_t bits_where(const struct ace2k_rfid_watch *self, uint8_t state)
{
    uint8_t mask = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (self->l[lane].state == state) {
            mask = (uint8_t)(mask | (1U << lane));
        }
    }
    return mask;
}

uint8_t ace2k_rfid_watch_hold_bits(const struct ace2k_rfid_watch *self)
{
    return bits_where(self, ACE2K_RFID_READING);
}

uint8_t ace2k_rfid_watch_read_bits(const struct ace2k_rfid_watch *self)
{
    return bits_where(self, ACE2K_RFID_READ);
}

uint8_t ace2k_rfid_watch_lost_bits(const struct ace2k_rfid_watch *self)
{
    uint8_t mask = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (self->l[lane].lost) {
            mask = (uint8_t)(mask | (1U << lane));
        }
    }
    return mask;
}

bool ace2k_rfid_watch_field(const struct ace2k_rfid_watch *self, uint8_t reader)
{
    if (reader < ACE2K_RFID_READER_COUNT && self->r[reader].field) {
        return true;
    }
    return false;
}

bool ace2k_rfid_watch_fault(const struct ace2k_rfid_watch *self, uint8_t reader)
{
    if (reader < ACE2K_RFID_READER_COUNT && self->r[reader].fault) {
        return true;
    }
    return false;
}

bool ace2k_rfid_watch_dead(const struct ace2k_rfid_watch *self, uint8_t reader)
{
    if (reader < ACE2K_RFID_READER_COUNT && self->r[reader].dead) {
        return true;
    }
    return false;
}

void ace2k_rfid_watch_clear_dead(struct ace2k_rfid_watch *self, uint8_t reader)
{
    if (reader < ACE2K_RFID_READER_COUNT) {
        self->r[reader].dead = false;
        self->r[reader].configure_failures = 0;
    }
}

bool ace2k_rfid_watch_reader_busy(const struct ace2k_rfid_watch *self, uint8_t reader)
{
    if (reader >= ACE2K_RFID_READER_COUNT) {
        return false;
    }
    if (self->r[reader].field || ace2k_iso14443a_busy(self->iso, reader)) {
        return true;
    }
    return false;
}

bool ace2k_rfid_watch_peek_event(const struct ace2k_rfid_watch *self, struct ace2k_rfid_event *out)
{
    if (self->tail == self->head) {
        return false;
    }
    *out = self->ring[self->tail];
    return true;
}

void ace2k_rfid_watch_drop_event(struct ace2k_rfid_watch *self)
{
    if (self->tail != self->head) {
        self->tail = (uint8_t)((self->tail + 1U) % ACE2K_RFID_EVENT_RING);
    }
}
