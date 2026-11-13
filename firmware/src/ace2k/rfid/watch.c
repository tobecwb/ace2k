#include "rfid/watch_internal.h"

void ace2k_rfid_watch_init(struct ace2k_rfid_watch *self, struct ace2k_iso14443a *iso,
                           const struct ace2k_rfid_watch_ops *ops, void *ctx)
{
    *self = (struct ace2k_rfid_watch){
        .iso = iso,
        .ops = ops,
        .ctx = ctx,
        .cfg = { .leg_max_um = ACE2K_RFID_CONFIRM_LEG_MAX_UM,
                 .speed_um_s = ACE2K_RFID_CONFIRM_SPEED_UM_S,
                 .passive_min_um = ACE2K_RFID_PASSIVE_MIN_UM,
                 .follows = ACE2K_RFID_CONFIRM_FOLLOWS,
                 .window_um = ACE2K_RFID_CONFIRM_WINDOW_UM,
                 .passive_max_um = ACE2K_RFID_PASSIVE_MAX_UM, },
    };
}

void ace2k_rfid_watch_push(struct ace2k_rfid_watch *self, const struct ace2k_rfid_event *e)
{
    uint8_t next = (uint8_t)((self->head + 1U) % ACE2K_RFID_EVENT_RING);
    if (next == self->tail) {
        self->dropped++;
        return;
    }
    self->ring[self->head] = *e;
    self->head = next;
}

void ace2k_rfid_watch_push_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t kind)
{
    struct ace2k_rfid_event e = { .type = ACE2K_RFID_EV_STATE, .lane = lane, .kind = kind };
    ace2k_rfid_watch_push(self, &e);
}

void ace2k_rfid_watch_set_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t state)
{
    if (self->l[lane].state == state) {
        return;
    }
    self->l[lane].state = state;
    if (state != ACE2K_RFID_READING) { /* a session opening is reported by its tag event */
        ace2k_rfid_watch_push_state(self, lane, state);
    }
}

/* A read on command that no inventory decided.  The host's wait ends on the lane's unchanged
 * state. */
static void answer_probe(struct ace2k_rfid_watch *self, uint8_t lane)
{
    self->l[lane].probe = false;
    ace2k_rfid_watch_push_state(self, lane, self->l[lane].state);
}

static void answer_probes(struct ace2k_rfid_watch *self, uint8_t reader)
{
    for (uint8_t i = 0; i < ACE2K_RFID_LANES_PER_READER; i++) {
        uint8_t lane = (uint8_t)(ace2k_rfid_watch_first_lane(reader) + i);
        if (self->l[lane].probe) {
            answer_probe(self, lane);
        }
    }
}

/* The lane drops its UID: the tag attributed to it, and the tag that waits for the reader's session
 * (want, and its target wanted). */
static void drop_uid(struct ace2k_rfid_watch_lane *l)
{
    l->tag = (struct ace2k_iso14443a_tag){ 0 };
    l->has_tag = false;
    l->want = false;
    l->wanted = (struct ace2k_iso14443a_tag){ 0 };
}

static void forget_tag(struct ace2k_rfid_watch_lane *l)
{
    drop_uid(l);
    l->lost = false;
    l->probe = false;
    l->seen = false;
}

/* The filament left the bay entrance.  The lane forgets everything, its session included. */
static void lane_emptied(struct ace2k_rfid_watch *self, uint8_t lane, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[ace2k_rfid_reader_of(lane)];
    if (rd->session_open && rd->session_lane == lane) {
        ace2k_rfid_watch_close_session(self, ace2k_rfid_reader_of(lane), ACE2K_RFID_UNKNOWN,
                                       now_ms);
    }
    forget_tag(&self->l[lane]);
    ace2k_rfid_watch_set_state(self, lane, ACE2K_RFID_UNKNOWN);
}

/* The feed's load starts or ends on the lane. */
static void lane_load_edge(struct ace2k_rfid_watch *self, uint8_t lane,
                           const struct ace2k_rfid_watch_inputs *in)
{
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    bool loading = ace2k_rfid_watch_bit(in->loading, lane);
    if (loading && !l->loading_last) {
        l->confirms = 0; /* a new load: its own cap of confirmations */
    }
    if (loading && !l->loading_last &&
        (l->state == ACE2K_RFID_PENDING || l->state == ACE2K_RFID_NO_TAG)) {
        l->seen = false;
        ace2k_rfid_watch_set_state(self, lane, ACE2K_RFID_SEARCHING);
    }
    if (!loading && l->loading_last && l->state == ACE2K_RFID_SEARCHING) {
        uint8_t next = ACE2K_RFID_PENDING;
        if (ace2k_rfid_watch_bit(in->exhausted, lane) && !l->has_tag && !l->seen) {
            next = ACE2K_RFID_NO_TAG; /* rule 9: no UID seen in the whole search */
        }
        ace2k_rfid_watch_set_state(self, lane, next);
    }
    l->loading_last = loading;
}

static void lane_update(struct ace2k_rfid_watch *self, uint8_t lane,
                        const struct ace2k_rfid_watch_inputs *in, uint32_t now_ms)
{
    if (ace2k_rfid_watch_bit(in->moving, lane)) {
        /* The neighbour's leaves were measured with this lane still: passive watching needs it
         * still from the leave to the return. */
        ace2k_rfid_watch_drop_gone(&self->r[ace2k_rfid_reader_of(lane)],
                                   ace2k_rfid_neighbour(lane));
    }
    if (in->insert_unknown) {
        return; /* a frozen reading neither empties a lane nor makes one pending */
    }
    if (!ace2k_rfid_watch_bit(in->insert, lane)) {
        if (self->l[lane].state != ACE2K_RFID_UNKNOWN || self->l[lane].has_tag) {
            lane_emptied(self, lane, now_ms);
        }
        self->l[lane].loading_last = false;
        self->l[lane].by_hand = false;
        ace2k_rfid_watch_drop_gone(&self->r[ace2k_rfid_reader_of(lane)], lane);
        return;
    }
    if (self->l[lane].state == ACE2K_RFID_UNKNOWN) {
        ace2k_rfid_watch_set_state(self, lane,
                                   ACE2K_RFID_PENDING); /* inserted, or present at boot */
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
    if (ace2k_rfid_watch_bit(in->moving, lane) || ace2k_rfid_watch_bit(in->loading, lane)) {
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
    for (uint8_t i = 0; i < ACE2K_RFID_LANES_PER_READER; i++) {
        uint8_t lane = (uint8_t)(ace2k_rfid_watch_first_lane(reader) + i);
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
    uint8_t first = ace2k_rfid_watch_first_lane(reader);
    if (ace2k_rfid_watch_bit(in->moving, first) ||
        ace2k_rfid_watch_bit(in->moving, (uint8_t)(first + 1U))) {
        return true;
    }
    return false;
}

/* A switch-off that reads back on is tried again at every step.  One lost SPI write must not leave
 * the field on.  The fault latches on the first such switch-off, and stays until a restart. */
static void field_off(struct ace2k_rfid_watch *self, uint8_t reader)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (!rd->field && !rd->off_pending) {
        return;
    }
    self->ops->field_set(self->ctx, reader, false);
    rd->field = false;
    rd->gone_count = 0; /* a UID's leave is measured within one spell of the field */
    rd->off_pending = self->ops->field_driven(self->ctx, reader);
    if (rd->off_pending) {
        rd->fault = true; /* rule 2: latched, reported through the health mask */
    }
}

/* A configure is a soft reset, polled for up to 150 ms (rfid/reader.h).  A failed configure is not
 * tried again before ACE2K_RFID_CONFIGURE_RETRY_MS.  ACE2K_RFID_CONFIGURE_TRIES failures in a row
 * make the reader dead.  Its reads on command are still answered, and the health bit fails. */
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

/* Configure first when a soft reset cleared the reader.  The next run turns the field on. */
static void field_on(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (self->ops->needs_configure(self->ctx, reader)) {
        configure(self, reader, now_ms);
        return;
    }
    self->ops->field_set(self->ctx, reader, true);
    rd->field = true;
    rd->off_pending = false; /* a lost switch-off no longer matters, the field is wanted on */
    rd->t_last_moving_ms = now_ms;
    /* The first inventory runs on the next task run, once the tags have powered up.  ISO/IEC
     * 14443-3 asks for at least 5 ms of field before the first command. */
    rd->t_last_inventory_ms = now_ms - ACE2K_RFID_POLL_MS + ACE2K_RFID_POWER_UP_MS;
    rd->baseline_polls = 0;
    rd->tracked_count = 0;
    rd->gone_count = 0;
}

/* The time limit on the field (rule 2).  It is reached when the field stayed on with no lane of the
 * reader moving and no session. */
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

static void begin_inventory(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    struct ace2k_iso14443a_job job = { .kind = ACE2K_ISO_JOB_INVENTORY };
    if (ace2k_iso14443a_begin(self->iso, reader, &job) == 0) {
        self->r[reader].t_last_inventory_ms = now_ms;
    }
}

/* A session the host left idle ends.  It is checked before the reader's own state, so a reader
 * that a soft reset cleared, or a dead one, cannot keep it open, nor the key of a step that never
 * started (rule 7). */
static void end_idle_session(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (rd->session_open &&
        ace2k_time_since(now_ms, rd->t_session_ms) >= ACE2K_RFID_SESSION_IDLE_MS) {
        ace2k_rfid_watch_close_session(
            self, reader, ace2k_rfid_watch_unread_state(&self->l[rd->session_lane]), now_ms);
    }
}

/* Nothing runs on the reader.  Start the host's step, or start an inventory. */
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
        ace2k_rfid_watch_wipe_job(
            &rd->step_job); /* the engine has its copy; the watch keeps no key (rule 7) */
        return;
    }
    if (rd->session_open) {
        return;
    }
    ace2k_rfid_watch_serve_wants(self, reader,
                                 now_ms); /* one deferred while the closed session's job ran */
    if (rd->session_open) {
        return;
    }
    if (ace2k_time_since(now_ms, rd->t_last_inventory_ms) >= ACE2K_RFID_POLL_MS) {
        begin_inventory(self, reader, now_ms);
    }
}

/* A soft reset during the job clears the reader's settings.  The reader can then report the
 * exchange as ended, but the result does not come from the tags.  The result of an inventory is
 * dropped, and a step ends as failed with no data.  The next free step switches the field off
 * (reader_step). */
static void job_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    bool reset = self->ops->needs_configure(self->ctx, reader);
    if (self->r[reader].step_running) {
        ace2k_rfid_watch_step_done(self, reader, now_ms, reset);
    } else if (!reset) {
        ace2k_rfid_watch_inventory_done(self, reader, now_ms);
    }
}

static void reader_step(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms,
                        const struct ace2k_rfid_watch_inputs *in)
{
    const struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (ace2k_iso14443a_busy(self->iso, reader)) {
        if (ace2k_iso14443a_step(self->iso, reader)) {
            job_done(self, reader, now_ms);
        }
        return;
    }
    end_idle_session(self, reader, now_ms);
    if (ceiling_reached(self, reader, now_ms, in) || !reader_needed(self, reader, in)) {
        field_off(self, reader);
        return;
    }
    if (!rd->field) {
        field_on(self, reader, now_ms);
        return;
    }
    if (self->ops->needs_configure(self->ctx, reader)) {
        /* A soft reset under the field cleared the reader.  No job runs on it until the field is
         * switched off here, and the next switch-on has configured it again. */
        field_off(self, reader);
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
        self->in_use_set = false;
        self->in_use = 0;
        /* A lane set by hand keeps its mark: only its strand leaving clears it (lane_update). */
        for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
            if (self->r[r].session_open) {
                ace2k_rfid_watch_close_session(
                    self, r, ace2k_rfid_watch_unread_state(&self->l[self->r[r].session_lane]),
                    now_ms);
            }
        }
    }
    for (uint8_t r = 0; r < ACE2K_RFID_READER_COUNT; r++) {
        ace2k_rfid_watch_confirm_step(self, r);
        reader_step(self, r, now_ms, in);
    }
}

/* A read or no_tag lane goes back to pending.  Its state event answers a read on command that
 * waits.  A pending lane can keep the UID of a session that ended without a read, or wait with a
 * UID for the reader's session.  The forget drops both, so the neighbour's-UID rule stops guarding
 * that UID for the neighbour.  The pending lane stays pending, and no event goes out.  Its lost
 * bit stays for the feed's load, and a read on command that waits stays too: the next inventory
 * answers it.  A searching lane is left alone, so its search still never ends no_tag after it saw
 * its tag.  A forget after the search ends drops its UID.  A reading lane is left alone too,
 * because the steps of its session read the tag that opened it. */
void ace2k_rfid_watch_forget(struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return;
    }
    struct ace2k_rfid_watch_lane *l = &self->l[lane];
    uint8_t s = l->state;
    if (s == ACE2K_RFID_READ || s == ACE2K_RFID_NO_TAG) {
        forget_tag(l);
        ace2k_rfid_watch_set_state(self, lane, ACE2K_RFID_PENDING);
    } else if (s == ACE2K_RFID_PENDING) {
        drop_uid(l);
    }
}

void ace2k_rfid_watch_host_forget(struct ace2k_rfid_watch *self, uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return;
    }
    ace2k_rfid_watch_forget(self, lane);
    self->l[lane].by_hand = false;
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

enum ace2k_rfid_read_refusal ace2k_rfid_watch_read_with_motion(struct ace2k_rfid_watch *self,
                                                               uint8_t lane,
                                                               ace2k_rfid_search_start_fn start,
                                                               void *ctx)
{
    if (ace2k_rfid_watch_lane_busy(self, lane)) {
        return ACE2K_RFID_READ_BUSY;
    }
    enum ace2k_rfid_read_refusal r = start(ctx, lane);
    if (r == ACE2K_RFID_READ_OK) {
        ace2k_rfid_watch_forget(self, lane);
        if (lane < ACE2K_LANE_COUNT) {
            self->l[lane].confirms = 0; /* the host's read: its own cap of confirmations */
        }
    }
    return r;
}

void ace2k_rfid_watch_set_by_hand(struct ace2k_rfid_watch *self, uint8_t lane)
{
    /* Whatever the lane's state: right after an insertion the lane may not be pending yet, and
     * the mark must already stand when its load starts. */
    if (lane < ACE2K_LANE_COUNT && ace2k_rfid_watch_bit(self->in.insert, lane)) {
        self->l[lane].by_hand = true;
    }
}

uint8_t ace2k_rfid_watch_by_hand_bits(const struct ace2k_rfid_watch *self)
{
    uint8_t mask = 0;
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        if (self->l[lane].by_hand) {
            mask = (uint8_t)(mask | (1U << lane));
        }
    }
    return mask;
}

void ace2k_rfid_watch_set_in_use(struct ace2k_rfid_watch *self, uint8_t mask)
{
    self->in_use = mask;
    self->in_use_set = true;
    self->t_in_use_ms = self->now_ms;
}

bool ace2k_rfid_watch_in_use(const struct ace2k_rfid_watch *self, uint8_t *mask)
{
    if (!self->in_use_set ||
        ace2k_time_since(self->now_ms, self->t_in_use_ms) > ACE2K_RFID_IN_USE_STALE_MS) {
        *mask = 0;
        return false;
    }
    *mask = self->in_use;
    return true;
}

uint8_t ace2k_rfid_watch_state(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].state : (uint8_t)ACE2K_RFID_UNKNOWN;
}

const struct ace2k_iso14443a_tag *ace2k_rfid_watch_tag(const struct ace2k_rfid_watch *self,
                                                       uint8_t lane)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return NULL;
    }
    return &self->l[lane].tag;
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
    if (reader >= ACE2K_RFID_READER_COUNT) {
        return false;
    }
    if (self->r[reader].field || self->r[reader].off_pending) {
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
    /* The field that the watch wants on counts, not a switch-off still pending.  No read runs
     * during that switch-off, and the soft reset of a probe is itself a switch-off. */
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
