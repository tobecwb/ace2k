/* What: the half-duplex turnaround and the presence of the host on the link.  The turnaround
 * decides whether a transmission may start on a line that the host may still be using.  The
 * presence says whether the host has been heard from lately.
 * How: two clocks.
 * The line clock is the tick of the last received byte.  Any byte counts, also a byte with a
 * framing, noise, parity or overrun error.  A transmit request inside the idle window after that
 * byte waits until the window ends.  It waits again while bytes keep arriving.  The line is busy
 * only within one idle window on either side of the last byte.  This bounds the wait to two
 * windows.  It also keeps a host that has been silent for a long time from reading as busy once
 * the tick counter has moved half its range.  A unit that has never received a byte owns the line.
 * The host clock is the tick of the last valid frame, whose CRC and sequence Klipper has checked.
 * Bytes alone do not prove a host.  Noise on the line is bytes too, and so is a converter that
 * keeps sending after the host died.  The longest interval between two valid frames is kept
 * too, so that the margin of the lost window can be measured: it is the same silence that
 * link_ok() judges, dated at the same point.
 * link_ok() latches a silence that reached lost_ticks, until the next frame.  The age is an
 * unsigned tick difference, so it reads small again one counter wrap later.  The latch keeps that
 * small age from reading as a host.
 * Depends on: <stdbool.h>, <stdint.h>.  Times are ticks of the caller's clock (uint32_t, wrapping),
 * never milliseconds.  Differences are compared signed, as Klipper's timers do.
 * Contexts (set by the binding): rx() runs from the receive interrupt.  frame(), tx_request() and
 * link_ok() run with interrupts masked, so nothing preempts them.  timer() runs from the timer
 * interrupt, which the receive interrupt may preempt, so timer() reads last_rx only once. */
#ifndef ACE2K_LINK_TURNAROUND_H
#define ACE2K_LINK_TURNAROUND_H
#include <stdbool.h>
#include <stdint.h>

enum ace2k_link_verdict {
    ACE2K_LINK_START,   /* raise the line now */
    ACE2K_LINK_WAIT,    /* ask again at *wait_until */
    ACE2K_LINK_NOTHING, /* timer(): no request was pending */
};

struct ace2k_link_turnaround {
    uint32_t idle_ticks; /* the silence that ends a host transmission */
    /* These fields are shared between the contexts listed above.  rx() writes last_rx and
     * rx_seen.  frame() writes last_frame and frame_seen, and clears lost.  link_ok() sets lost.
     * tx_request() sets and clears tx_pending, and timer() clears it.  frame() and link_ok() raise
     * max_gap, and gap_clear() zeroes it. */
    volatile uint32_t last_rx;    /* tick of the last received byte */
    volatile bool rx_seen;        /* a byte has arrived since init */
    volatile uint32_t last_frame; /* tick of the last valid frame from the host */
    volatile bool frame_seen;     /* a valid frame has arrived since init */
    volatile bool lost;           /* the silence reached lost_ticks; cleared by the next frame */
    volatile bool tx_pending;     /* a request is waiting for the line */
    uint32_t deferred;            /* waits for the line, not requests; statistics, read-only */
    uint32_t rearmed;             /* waits made longer because bytes kept arriving */
    uint32_t max_gap;             /* the longest silence between two valid frames, in ticks */
};

void ace2k_link_turnaround_init(struct ace2k_link_turnaround *self, uint32_t idle_ticks);

/* Records that a byte arrived at `now`.  Called from the receive interrupt, for a flawed byte
 * too.  It updates the line clock only, because a byte says nothing about the host. */
void ace2k_link_turnaround_rx(struct ace2k_link_turnaround *self, uint32_t now);

/* Records that a valid frame from the host arrived at `now`.  Klipper's console task calls it
 * once the CRC and the sequence of the frame are checked.  It ends a latched loss.  The interval
 * since the previous frame raises max_gap; the first frame ends no interval. */
void ace2k_link_turnaround_frame(struct ace2k_link_turnaround *self, uint32_t now);

/* Starts max_gap again from 0.  The next frame still ends the interval that began at the last
 * frame before the clear, and that interval counts.  The link state is untouched. */
void ace2k_link_turnaround_gap_clear(struct ace2k_link_turnaround *self);

/* Returns true while the host is there.  That means a valid frame has arrived, and at no call so
 * far has the silence since the last frame reached lost_ticks.  Once it has, the answer is false
 * until the next frame, whatever `now` says later.  The bindings call it on their 10 ms tick, so
 * at least once per lost_ticks.  A loss is therefore seen long before the counter wraps.  The
 * caller reads `now` with interrupts masked, after last_frame can no longer change, so the age is
 * never negative.  The call that latches a loss raises max_gap to that age.  So a gap that ended a
 * loss never reads below lost_ticks, even when it lasted longer than one counter wrap and its
 * unsigned difference reads small again. */
bool ace2k_link_turnaround_link_ok(struct ace2k_link_turnaround *self, uint32_t now,
                                   uint32_t lost_ticks);

/* Asks to start a transmission at `now`.
 * START: raise the line now.  Any pending wait is over.
 * WAIT: the request is recorded.  Ask again at *wait_until, which is last_rx + idle_ticks. */
enum ace2k_link_verdict ace2k_link_turnaround_tx_request(struct ace2k_link_turnaround *self,
                                                         uint32_t now, uint32_t *wait_until);

/* Handles the timer that fired at `now`.
 * START: the pending request is used up.  Raise the line.
 * WAIT: bytes kept arriving.  Arm the timer again for *wait_until.
 * NOTHING: no request was pending. */
enum ace2k_link_verdict ace2k_link_turnaround_timer(struct ace2k_link_turnaround *self,
                                                    uint32_t now, uint32_t *wait_until);

#endif
