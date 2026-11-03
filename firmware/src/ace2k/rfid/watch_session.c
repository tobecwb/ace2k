/* The read sessions of the watch with the host.  It opens a session for a lane, runs the read steps
 * that the host asks for, and closes the session.  It was split from watch.c, and the seam is
 * watch_internal.h. */
#include "rfid/watch_internal.h"

void ace2k_rfid_watch_open_session(struct ace2k_rfid_watch *self, uint8_t lane,
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
    ace2k_rfid_watch_set_state(self, lane, ACE2K_RFID_READING);
    struct ace2k_rfid_event e = {
        .type = ACE2K_RFID_EV_TAG,
        .lane = lane,
        .session = rd->session_id,
        .tag = *tag,
    };
    ace2k_rfid_watch_push(self, &e);
}

/* A lane whose UID was attributed while the reader's session was busy gets its turn.  It waits
 * until no session is open and no job runs on the reader.  A step of the session just closed may
 * still be running, and its result must not land in the next session. */
void ace2k_rfid_watch_serve_wants(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms)
{
    bool busy = ace2k_iso14443a_busy(self->iso, reader);
    for (uint8_t i = 0; i < ACE2K_RFID_LANES_PER_READER; i++) {
        uint8_t lane = (uint8_t)(ace2k_rfid_watch_first_lane(reader) + i);
        if (!self->l[lane].want) {
            continue;
        }
        if (!ace2k_rfid_watch_eligible(self, lane)) {
            self->l[lane].want = false; /* the filament left, the link went, or the lane was read */
        } else if (!self->r[reader].session_open && !busy) {
            ace2k_rfid_watch_open_session(self, lane, &self->l[lane].wanted,
                                          now_ms); /* clears the want */
        }
    }
}

/* The session ends.  A lane that is still reading goes to `state`. */
void ace2k_rfid_watch_close_session(struct ace2k_rfid_watch *self, uint8_t reader, uint8_t state,
                                    uint32_t now_ms)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    if (!rd->session_open) {
        return;
    }
    rd->session_open = false;
    rd->step_pending = false;
    ace2k_rfid_watch_wipe_job(&rd->step_job);
    if (self->l[rd->session_lane].state == ACE2K_RFID_READING) {
        ace2k_rfid_watch_set_state(self, rd->session_lane, state);
    }
    ace2k_rfid_watch_serve_wants(self, reader, now_ms);
}

void ace2k_rfid_watch_step_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms,
                                bool reset)
{
    struct ace2k_rfid_watch_reader *rd = &self->r[reader];
    const struct ace2k_iso14443a_out *o = ace2k_iso14443a_result(self->iso, reader);
    rd->step_running = false;
    if (!rd->session_open || rd->session_lane != rd->step_lane ||
        rd->session_id != rd->step_session) {
        return; /* its session ended meanwhile; the bytes go nowhere, no lane is lost */
    }
    struct ace2k_rfid_event e = {
        .type = ACE2K_RFID_EV_DATA,
        .lane = rd->session_lane,
        .session = rd->session_id,
    };
    uint8_t result = reset ? (uint8_t)ACE2K_ISO_ERROR : o->result;
    uint8_t data_count = reset ? 0U : o->data_count; /* bytes read across a reset are not kept */
    for (uint8_t i = 0; i < data_count; i++) {
        e.block = o->data_first[i];
        for (uint8_t b = 0; b < ACE2K_ISO_BLOCK_BYTES; b++) {
            e.data[b] = o->data[i][b];
        }
        ace2k_rfid_watch_push(self, &e);
    }
    rd->t_session_ms = now_ms;
    if (result == ACE2K_ISO_OK) {
        return;
    }
    e = (struct ace2k_rfid_event){
        .type = ACE2K_RFID_EV_DATA,
        .lane = rd->session_lane,
        .session = rd->session_id,
        .kind = result,
        .block = rd->step_first,
    };
    ace2k_rfid_watch_push(self, &e);
    if (result == ACE2K_ISO_TAG_GONE) {
        self->l[rd->session_lane].lost = true;
        ace2k_rfid_watch_close_session(
            self, reader, ace2k_rfid_watch_unread_state(&self->l[rd->session_lane]), now_ms);
    }
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
    job->target.uid_len = ACE2K_ISO_UID_SINGLE; /* for the bounds check; the tag is set below */
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
    ace2k_rfid_watch_wipe_job(&job);
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
    uint8_t state = read ? (uint8_t)ACE2K_RFID_READ : ace2k_rfid_watch_unread_state(&self->l[lane]);
    /* Use the last step's clock, not that of the session.  close_session may open the session of
     * the next lane, and its idle timer must start now, not at the last step of this session. */
    ace2k_rfid_watch_close_session(self, reader, state, self->now_ms);
    return 0;
}
