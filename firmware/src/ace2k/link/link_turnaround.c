#include "link/link_turnaround.h"

void ace2k_link_turnaround_init(struct ace2k_link_turnaround *self, uint32_t idle_ticks)
{
    self->idle_ticks = idle_ticks;
    self->last_rx = 0;
    self->rx_seen = false;
    self->lost = false;
    self->tx_pending = false;
    self->deferred = 0;
    self->rearmed = 0;
}

void ace2k_link_turnaround_rx(struct ace2k_link_turnaround *self, uint32_t now)
{
    self->last_rx = now;
    self->rx_seen = true;
    self->lost = false;
}

/* The age is unsigned on purpose: a signed difference would turn negative half a wrap after the
 * last byte and read as recent; the unsigned one reads small again a whole wrap later, and the
 * latch — set on the first call that finds the silence long enough — holds through it. */
bool ace2k_link_turnaround_link_ok(struct ace2k_link_turnaround *self, uint32_t now,
                                   uint32_t lost_ticks)
{
    if (!self->rx_seen || self->lost) {
        return false;
    }
    if (now - self->last_rx >= lost_ticks) {
        self->lost = true;
        return false;
    }
    return true;
}

/* Busy only within one idle window either side of the last byte (signed: the tick counter
 * wraps): a byte a few ticks "after" the caller's clock is the receive interrupt racing the call
 * and stays busy; a host silent for longer than half the counter's range is quiet, not busy
 * again.  A unit that never received a byte owns the line.  last_rx is the caller's one read. */
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
