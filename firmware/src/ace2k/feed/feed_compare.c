/* The motor × filament comparator of the feed: the
 * standstill on its own base, the partial over its window, the load's grace.  It returns the
 * trigger that tripped; the kind — and whether the trip is an error at all — is the mode's to
 * decide from the buffer, in feed.c.  Split from feed.c, which keeps the modes, the tick and the
 * events; the seam is feed_internal.h.  Runs in the feed's tick, interrupt context. */
#include "feed/feed_internal.h"

/* The partial trigger's window begins here: at every close that found the strand within the
 * allowance, and with the standstill's base below at a mode's start, through a load's grace and
 * at each of the assist's bursts. */
static void window_restart(struct ace2k_feed *self, uint8_t lane)
{
    self->l[lane].fg_window = self->lane->lane[lane].fg_raw;
    self->l[lane].enc_um_window = ace2k_lane_encoder_um(self->lane, lane);
}

/* The standstill's base: the strand moved along the command just now, or a fresh start. */
static void still_restart(struct ace2k_feed *self, uint8_t lane)
{
    self->l[lane].fg_still = self->lane->lane[lane].fg_raw;
    self->l[lane].enc_um_still = ace2k_lane_encoder_um(self->lane, lane);
}

void ace2k_feed_compare_restart(struct ace2k_feed *self, uint8_t lane)
{
    window_restart(self, lane);
    still_restart(self, lane);
}

/* A load's first ACE2K_FEED_LOAD_GRACE_UM of motor: the operator's hand and the pinch catching
 * the strand are not a slip. */
static bool in_load_grace(const struct ace2k_feed *self, uint8_t lane)
{
    return ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - self->l[lane].fg_start) <
           ACE2K_FEED_LOAD_GRACE_UM;
}

/* The filament's travel since a base along the command, in micrometres: the encoder's signed
 * travel, negated for a move toward the spool; a strand moving the wrong way reads as none. */
static uint32_t filament_along_um(const struct ace2k_feed *self, uint8_t lane, int32_t base_um,
                                  bool reverse)
{
    int32_t travel = ace2k_lane_encoder_um(self->lane, lane) - base_um;
    int32_t along = reverse ? -travel : travel;
    return along > 0 ? (uint32_t)along : 0;
}

/* The standstill: stall_check_um of motor since the strand
 * last moved — a tip held in a hot end while the rollback grinds, a tangle that stops everything;
 * under a second of motor at 30 mm/s.  "Moved" is ACE2K_FEED_STANDSTILL_UM along the command
 * since the base, more than one count at every scale, and refreshes the base; a strand creeping
 * a count at a time under that is still.  Its own base, not the partial's window: a jam after
 * the strand has followed for a while is caught stall_check_um of motor later, wherever the
 * window stands. */
static bool standstill_trips(struct ace2k_feed *self, uint8_t lane, bool reverse)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (filament_along_um(self, lane, l->enc_um_still, reverse) >= ACE2K_FEED_STANDSTILL_UM) {
        still_restart(self, lane);
        return false;
    }
    return ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - l->fg_still) >=
           self->th.stall_check_um;
}

/* The partial: at slip_check_um of motor over the window, the filament short by more than
 * slip_allow_um; within the allowance the window restarts and the move goes on.  A strand the
 * head pulls ahead of the motor (the assist) reads as more than the motor and is never short —
 * the comparison is written so, not as a difference that would wrap. */
static bool partial_trips(struct ace2k_feed *self, uint8_t lane, bool reverse)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t motor_um = ace2k_lane_fg_to_um(self->lane->lane[lane].fg_raw - l->fg_window);
    if (motor_um < self->th.slip_check_um) {
        return false; /* the window is still open */
    }
    uint32_t filament_um = filament_along_um(self, lane, l->enc_um_window, reverse);
    if ((uint64_t)motor_um > (uint64_t)filament_um + self->th.slip_allow_um) {
        return true;
    }
    window_restart(self, lane);
    return false;
}

/* The comparator, every tick for every lane in a mode the feed entered whose motor is commanded
 * — the assist modes included; a lane the lane core moves for anyone else (the bench, a test) is
 * not judged against bases it never took.  A load's grace is not judged either.  The standstill
 * is asked first: it is the sharper of the two, and a strand that stood still through a window's
 * close is a standstill before it is a shortfall. */
enum ace2k_feed_trip ace2k_feed_compare(struct ace2k_feed *self, uint8_t lane)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_mode_runs(l->mode) || !ace2k_lane_is_moving(self->lane, lane)) {
        return ACE2K_FEED_TRIP_NONE;
    }
    if (l->mode == ACE2K_FEED_LOADING && in_load_grace(self, lane)) {
        ace2k_feed_compare_restart(self, lane);
        return ACE2K_FEED_TRIP_NONE;
    }
    bool reverse = ace2k_lane_is_reverse(self->lane, lane);
    if (standstill_trips(self, lane, reverse)) {
        return ACE2K_FEED_TRIP_STANDSTILL;
    }
    return partial_trips(self, lane, reverse) ? ACE2K_FEED_TRIP_PARTIAL : ACE2K_FEED_TRIP_NONE;
}
