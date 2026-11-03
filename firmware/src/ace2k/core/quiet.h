/* What: a stopped unit goes quiet: after Klipper's shutdown it drops every report and event its
 * bindings send, until the reset.
 * How: the transmit try (ace2k_board/tx.c) asks ace2k_quiet_drop() before each frame.
 * Depends on: core/util.h (ace2k_time_after). */
#ifndef ACE2K_QUIET_H
#define ACE2K_QUIET_H
#include <stdbool.h>
#include <stdint.h>
#include "core/util.h"

/* On the half-duplex link a frame of the host that meets a frame of the unit is lost, and
 * Klipper's reset (FIRMWARE_RESTART) is sent once, with no retransmit. A unit that is reporting
 * loses that reset to such a collision: 1 restart in 20, measured on the unit, and about 1 in 13
 * in the U1's klippy logs of 2026-10-24
 * (docs/protocol.md, "Two consequences to know"). Klipper's own frames still go out: its replies
 * to the host, and its statistics every 5 s, which are rare. The frames of the shutdown itself
 * (the stopped moves, the dryer's stop) leave for this long first. */
#define ACE2K_QUIET_GRACE_MS 1000U

/* True when a frame is to be dropped. shut: Klipper is in shutdown; shut_ms: when it began;
 * quiet: the latch, the caller's. A dropped frame counts as sent, so nothing stays held: the
 * dryer still stores its log, and no task is woken again for it. The latch keeps a stop longer
 * than half the clock's range quiet. Klipper's clear_shutdown, with no reset, ends it. */
static inline bool ace2k_quiet_drop(bool shut, uint32_t now_ms, uint32_t shut_ms, bool *quiet)
{
    if (!shut) {
        *quiet = false;
        return false;
    }
    if (!*quiet && ace2k_time_after(now_ms, shut_ms + ACE2K_QUIET_GRACE_MS)) {
        *quiet = true;
    }
    return *quiet;
}

#endif
