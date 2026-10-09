/* What: the half-duplex turnaround — whether a transmission may start on a line the host may
 * still be using — and the link's presence: whether the host has been heard from lately.
 * How: the tick of the last received byte; a transmit request inside the idle window after it
 * is deferred to the end of that window, and re-deferred while bytes keep arriving.  The line
 * is busy only within one idle window either side of the last byte, which bounds the wait to
 * two windows and keeps a long-silent host from reading as busy once the tick counter has moved
 * half its range.  A unit that has never received a byte owns the line.  link_ok() latches a
 * silence that reached lost_ticks until the next byte: the age is an unsigned tick difference
 * that reads small again one counter wrap later, and the latch keeps that from reading as a host.
 * Depends on: <stdbool.h>, <stdint.h>.  Times are ticks of the caller's clock (uint32_t,
 * wrapping; differences compared signed, as Klipper's timers do), never milliseconds.
 * Contexts (the binding's): rx() from the receive interrupt, tx_request() and link_ok() from any
 * context with interrupts masked, timer() from the timer interrupt; the receive interrupt may
 * preempt the other two, so last_rx is read once per call. */
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
    uint32_t last_rx;    /* tick of the last received byte */
    bool rx_seen;        /* a byte has arrived since init */
    bool lost;           /* the silence reached lost_ticks; cleared by the next byte */
    bool tx_pending;     /* a request is waiting for the line */
    uint32_t deferred;   /* waits for the line, not requests (statistics, read-only) */
    uint32_t rearmed;    /* waits extended because bytes kept arriving */
};

void ace2k_link_turnaround_init(struct ace2k_link_turnaround *self, uint32_t idle_ticks);

/* Receive interrupt: a byte arrived at `now`; a latched loss is over. */
void ace2k_link_turnaround_rx(struct ace2k_link_turnaround *self, uint32_t now);

/* True while the host is there: a byte has arrived, and the silence since the last one has not
 * reached lost_ticks at any call so far.  Once it has, false until the next byte, whatever `now`
 * says later.  Called at least once per lost_ticks (the bindings' 10 ms tick), so a loss is seen
 * long before the counter wraps; `now` is read after last_rx can no longer change (interrupts
 * masked), so the age is never negative. */
bool ace2k_link_turnaround_link_ok(struct ace2k_link_turnaround *self, uint32_t now,
                                   uint32_t lost_ticks);

/* A transmission is requested at `now`.  START: raise the line (any pending wait is over).
 * WAIT: the request is recorded; ask again at *wait_until = last_rx + idle_ticks. */
enum ace2k_link_verdict ace2k_link_turnaround_tx_request(struct ace2k_link_turnaround *self,
                                                         uint32_t now, uint32_t *wait_until);

/* The timer fired at `now`.  START: the pending request is consumed, raise the line.  WAIT: bytes
 * kept arriving, re-arm at *wait_until.  NOTHING: no request was pending. */
enum ace2k_link_verdict ace2k_link_turnaround_timer(struct ace2k_link_turnaround *self,
                                                    uint32_t now, uint32_t *wait_until);

#endif
