/* What: the seam between watch.c, watch_session.c and watch_attrib.c.
 * How: plain functions on struct ace2k_rfid_watch.  None of them is part of the public interface.
 * Depends on: rfid/watch.h. */
#ifndef ACE2K_RFID_WATCH_INTERNAL_H
#define ACE2K_RFID_WATCH_INTERNAL_H
#include "rfid/watch.h"

/* The lane's bit in a mask with one bit per lane. */
static inline bool ace2k_rfid_watch_bit(uint8_t mask, uint8_t lane)
{
    return (((uint32_t)mask >> lane) & 1U) != 0;
}

/* The two lanes of a reader.  B serves 0 and 1, and A serves 2 and 3. */
static inline uint8_t ace2k_rfid_watch_first_lane(uint8_t reader)
{
    return reader == ACE2K_RFID_READER_B ? 0U : ACE2K_RFID_LANES_PER_READER;
}

/* This goes through ace2k_wipe().  The local job of step_request dies right after, so a plain
 * zeroing would be a dead store that the compiler drops (rule 7). */
static inline void ace2k_rfid_watch_wipe_job(struct ace2k_iso14443a_job *job)
{
    ace2k_wipe(job, sizeof *job);
}

/* The state that a lane goes back to when its tag is not read.  It is searching while the load
 * runs, and pending otherwise. */
static inline uint8_t ace2k_rfid_watch_unread_state(const struct ace2k_rfid_watch_lane *l)
{
    return l->loading_last ? (uint8_t)ACE2K_RFID_SEARCHING : (uint8_t)ACE2K_RFID_PENDING;
}

/* A lane whose tag an inventory may attribute.  The lane has filament in, the link is up, and the
 * lane is pending or searching. */
static inline bool ace2k_rfid_watch_eligible(const struct ace2k_rfid_watch *self, uint8_t lane)
{
    uint8_t s = self->l[lane].state;
    if (!self->in.link_ok || self->in.insert_unknown ||
        !ace2k_rfid_watch_bit(self->in.insert, lane)) {
        return false;
    }
    if (s == ACE2K_RFID_PENDING || s == ACE2K_RFID_SEARCHING) {
        return true;
    }
    return false;
}

/* watch.c: the event ring and the state of each lane */
void ace2k_rfid_watch_push(struct ace2k_rfid_watch *self, const struct ace2k_rfid_event *e);
void ace2k_rfid_watch_push_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t kind);
/* A new state pushes its state event, except READING.  The opening of a session is reported by its
 * tag event. */
void ace2k_rfid_watch_set_state(struct ace2k_rfid_watch *self, uint8_t lane, uint8_t state);

/* watch_session.c */
void ace2k_rfid_watch_open_session(struct ace2k_rfid_watch *self, uint8_t lane,
                                   const struct ace2k_iso14443a_tag *tag, uint32_t now_ms);
void ace2k_rfid_watch_serve_wants(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms);
void ace2k_rfid_watch_close_session(struct ace2k_rfid_watch *self, uint8_t reader, uint8_t state,
                                    uint32_t now_ms);
/* The job of a host step ended on the reader.  This sends its data events, and ends the session if
 * the tag left.  With reset, the reader was soft-reset during the job.  Then the step ends as
 * failed with no data.  It sends one data event of kind ACE2K_RFID_DATA_ERROR, and the session
 * stays open. */
void ace2k_rfid_watch_step_done(struct ace2k_rfid_watch *self, uint8_t reader, uint32_t now_ms,
                                bool reset);

/* watch_attrib.c: an inventory ended on the reader */
void ace2k_rfid_watch_inventory_done(struct ace2k_rfid_watch *self, uint8_t reader,
                                     uint32_t now_ms);

#endif
