#include "link/link_turnaround.h"

void ace2k_link_turnaround_init(struct ace2k_link_turnaround *self, uint32_t idle_ticks)
{
    self->idle_ticks = idle_ticks;
    self->last_rx = 0;
    self->rx_seen = false;
    self->last_frame = 0;
    self->frame_seen = false;
    self->lost = false;
    self->tx_pending = false;
    self->deferred = 0;
    self->rearmed = 0;
    self->max_gap = 0;
}

static void note_gap(struct ace2k_link_turnaround *self, uint32_t gap)
{
    if (gap > self->max_gap) {
        self->max_gap = gap;
    }
}

void ace2k_link_turnaround_rx(struct ace2k_link_turnaround *self, uint32_t now)
{
    self->last_rx = now;
    self->rx_seen = true;
}

void ace2k_link_turnaround_frame(struct ace2k_link_turnaround *self, uint32_t now)
{
    if (self->frame_seen) {
        note_gap(self, now - self->last_frame);
    }
    self->last_frame = now;
    self->frame_seen = true;
    self->lost = false;
}

void ace2k_link_turnaround_gap_clear(struct ace2k_link_turnaround *self)
{
    self->max_gap = 0;
}

/* The age is unsigned on purpose.  A signed difference would turn negative half a wrap after the
 * last frame, and the age would read as recent.  The unsigned age reads small again only a whole
 * wrap later.  By then the latch holds: the first call that finds the silence long enough sets
 * it. */
bool ace2k_link_turnaround_link_ok(struct ace2k_link_turnaround *self, uint32_t now,
                                   uint32_t lost_ticks)
{
    if (!self->frame_seen || self->lost) {
        return false;
    }
    uint32_t age = now - self->last_frame;
    if (age >= lost_ticks) {
        self->lost = true;
        note_gap(self, age);
        return false;
    }
    return true;
}

/* The line is busy only within one idle window on either side of the last byte.  The difference
 * is signed because the tick counter wraps.  A byte a few ticks "after" the caller's clock comes
 * from the receive interrupt racing the call, and the line stays busy.  A host silent for longer
 * than half the range of the counter is quiet, not busy again.  A unit that never received
 * a byte owns the line.  last_rx is the one read that the caller made. */
static bool line_quiet(const struct ace2k_link_turnaround *self, uint32_t now, uint32_t last_rx)
{
    if (!self->rx_seen) {
        return true;
    }
    int32_t since_rx = (int32_t)(now - last_rx);
    int32_t idle = (int32_t)self->idle_ticks;
    if (since_rx < -idle) {
        return true;
    }
    return since_rx >= idle;
}

enum ace2k_link_verdict ace2k_link_turnaround_tx_request(struct ace2k_link_turnaround *self,
                                                         uint32_t now, uint32_t *wait_until)
{
    uint32_t last_rx = self->last_rx;
    if (line_quiet(self, now, last_rx)) {
        self->tx_pending = false; /* a late timer finds NOTHING */
        return ACE2K_LINK_START;
    }
    if (!self->tx_pending) {
        self->tx_pending = true;
        self->deferred++; /* one wait, however many requests join it */
    }
    *wait_until = last_rx + self->idle_ticks;
    return ACE2K_LINK_WAIT;
}

enum ace2k_link_verdict ace2k_link_turnaround_timer(struct ace2k_link_turnaround *self,
                                                    uint32_t now, uint32_t *wait_until)
{
    if (!self->tx_pending) {
        return ACE2K_LINK_NOTHING;
    }
    uint32_t last_rx = self->last_rx;
    if (line_quiet(self, now, last_rx)) {
        self->tx_pending = false;
        return ACE2K_LINK_START;
    }
    self->rearmed++;
    *wait_until = last_rx + self->idle_ticks;
    return ACE2K_LINK_WAIT;
}
