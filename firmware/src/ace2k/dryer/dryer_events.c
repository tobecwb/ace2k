/* The dryer's events. The tick writes them into a ring, and the task reads them. Each event
 * is held until the host acknowledges it. A full ring drops the new event and counts it. There is
 * one producer (the tick) and one consumer (the task). dryer.c keeps the states, and the two
 * files share dryer_internal.h. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

#define DRYER_SEQ_HALF 128U /* an acknowledgement covers an event at most this far behind it */

static uint8_t ring_next(uint8_t i)
{
    return (uint8_t)((i + 1U) % ACE2K_DRYER_EVENT_RING);
}

void ace2k_dryer_emit(struct ace2k_dryer *self, uint8_t kind, uint8_t arg)
{
    uint8_t head = self->ring_head;
    uint8_t next = ring_next(head);
    if (next == self->ring_tail) {
        self->events_lost++;
        return;
    }
    self->ring[head] = (struct ace2k_dryer_event){ .kind = kind, .arg = arg, .seq = self->ev_seq };
    self->ev_seq++;
    ACE2K_COMPILER_BARRIER();
    self->ring_head = next;
}

bool ace2k_dryer_event_next(const struct ace2k_dryer *self, struct ace2k_dryer_event *out)
{
    uint8_t i = self->ring_sent;
    if (i == self->ring_head) {
        return false;
    }
    ACE2K_COMPILER_BARRIER();
    *out = self->ring[i];
    return true;
}

void ace2k_dryer_event_sent(struct ace2k_dryer *self)
{
    uint8_t i = self->ring_sent;
    if (i == self->ring_head) {
        return;
    }
    if (i == self->ring_sent_hi) {
        self->ring_sent_hi = ring_next(i); /* a first send, so the high-water mark moves */
    }
    self->ring_sent = ring_next(i);
}

static uint8_t ring_prev(uint8_t i)
{
    return (uint8_t)((i + ACE2K_DRYER_EVENT_RING - 1U) % ACE2K_DRYER_EVENT_RING);
}

/* True when the ack is for an event the host cannot have seen. That is when seq is after the
 * newest event ever sent, or when no unacknowledged event was ever sent. A resend does not lower
 * the mark. So an ack for an event sent before the resend is honoured, even when it lands after
 * the resend. */
static bool ack_beyond_sent(const struct ace2k_dryer *self, uint8_t seq)
{
    if (self->ring_sent_hi == self->ring_tail) {
        return true;
    }
    uint8_t newest = self->ring[ring_prev(self->ring_sent_hi)].seq;
    uint8_t ahead = (uint8_t)(seq - newest);
    if (ahead == 0U) {
        return false;
    }
    return ahead < DRYER_SEQ_HALF;
}

void ace2k_dryer_event_ack(struct ace2k_dryer *self, uint8_t seq)
{
    if (ack_beyond_sent(self, seq)) {
        return; /* ignored whole: otherwise events never sent would leave, uncounted */
    }
    while (self->ring_tail != self->ring_sent_hi) {
        uint8_t tail = self->ring_tail;
        if ((uint8_t)(seq - self->ring[tail].seq) >= DRYER_SEQ_HALF) {
            return; /* the oldest held event is after seq */
        }
        if (self->ring_sent == tail) {
            self->ring_sent = ring_next(tail); /* the cursor of a resend is not past it yet */
        }
        self->ring_tail = ring_next(tail);
    }
}

void ace2k_dryer_event_resend(struct ace2k_dryer *self)
{
    self->ring_sent = self->ring_tail;
}

bool ace2k_dryer_event_resend_due(const struct ace2k_dryer *self, bool subscribed,
                                  uint32_t last_send_ms, uint32_t now_ms)
{
    if (!subscribed || !ace2k_dryer_event_unacked(self)) {
        return false;
    }
    return ace2k_time_since(now_ms, last_send_ms) >= ACE2K_DRYER_EVENT_RESEND_MS;
}

uint8_t ace2k_dryer_event_oldest(const struct ace2k_dryer *self)
{
    uint8_t tail = self->ring_tail;
    if (tail == self->ring_head) {
        return self->ev_seq; /* none held: the seq of the next one */
    }
    ACE2K_COMPILER_BARRIER();
    return self->ring[tail].seq;
}

bool ace2k_dryer_event_unacked(const struct ace2k_dryer *self)
{
    return self->ring_tail != self->ring_head;
}
