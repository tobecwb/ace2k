/* The feed's comparator, which compares the motor with the filament.  It holds the standstill on
 * its own base, the partial over its window, and the grace of the load.  It returns the trigger
 * that tripped.  The mode decides the kind from the buffer, and whether the trip is an error at
 * all, in feed_judge.c (ace2k_feed_on_trip).  This was split from feed.c, which keeps the modes,
 * the tick and the events.  The seam is feed_internal.h.  It runs in the feed's tick, in interrupt
 * context. */
#include "feed/feed_internal.h"

/* The window of the partial trigger begins here.  It begins at every close that found the filament
 * within the allowance.  With the base of the standstill below, it also begins at the start of a
 * mode, through the grace of a load, and at each burst of the assist. */
static void window_restart(struct ace2k_feed *self, uint8_t lane)
{
    self->l[lane].fg_window_pulses = ace2k_lane_fg_raw(self->lane, lane);
    self->l[lane].enc_um_window = ace2k_lane_encoder_um(self->lane, lane);
}

/* The standstill's base: the filament moved along the command just now, or a fresh start. */
static void still_restart(struct ace2k_feed *self, uint8_t lane)
{
    self->l[lane].fg_still_pulses = ace2k_lane_fg_raw(self->lane, lane);
    self->l[lane].enc_um_still = ace2k_lane_encoder_um(self->lane, lane);
}

void ace2k_feed_compare_restart(struct ace2k_feed *self, uint8_t lane)
{
    window_restart(self, lane);
    still_restart(self, lane);
}

/* The first ACE2K_FEED_LOAD_GRACE_UM of motor of a load from the bay.  The hand of the operator,
 * and the pinch catching the filament, are not a slip.  A read with motion has no grace, because
 * its filament is gripped from the start.  Up to v0.12.0 the grace covered it too. */
static bool in_load_grace(const struct ace2k_feed *self, uint8_t lane)
{
    return (bool)(self->l[lane].grace &&
                  ace2k_feed_motor_since_um(self, lane, self->l[lane].fg_start_pulses) <
                      ACE2K_FEED_LOAD_GRACE_UM);
}

/* The standstill: stall_check_um of motor since the filament last moved.  It catches a tip held in
 * a hot end while the rollback grinds, or a tangle that stops everything.  That is under a second
 * of motor at 30 mm/s.  "Moved" means ACE2K_FEED_STANDSTILL_UM along the command since the base,
 * which is more than one count at every scale, and it refreshes the base.  A filament that creeps a
 * count at a time under that is still.  The standstill has its own base, not the partial's window.
 * So a jam after the filament has followed for a while is caught stall_check_um of motor later,
 * wherever the window stands. */
static bool standstill_trips(struct ace2k_feed *self, uint8_t lane, bool reverse)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (ace2k_feed_strand_along_um(self, lane, l->enc_um_still, reverse) >=
        ACE2K_FEED_STANDSTILL_UM) {
        still_restart(self, lane);
        return false;
    }
    return ace2k_feed_motor_since_um(self, lane, l->fg_still_pulses) >= self->th.stall_check_um;
}

/* The partial: at slip_check_um of motor over the window, the filament is short by more than
 * slip_allow_um.  Within the allowance the window restarts and the move goes on.  Filament that the
 * head pulls ahead of the motor, in the assist, reads as more than the motor and is never short.
 * The comparison is written for that, not as a difference that would wrap. */
static bool partial_trips(struct ace2k_feed *self, uint8_t lane, bool reverse)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    uint32_t motor_um = ace2k_feed_motor_since_um(self, lane, l->fg_window_pulses);
    if (motor_um < self->th.slip_check_um) {
        return false; /* the window is still open */
    }
    uint32_t filament_um = ace2k_feed_strand_along_um(self, lane, l->enc_um_window, reverse);
    if ((uint64_t)motor_um > (uint64_t)filament_um + self->th.slip_allow_um) {
        return true;
    }
    window_restart(self, lane);
    return false;
}

/* The comparator runs every tick, for every lane in a mode that the feed entered and whose motor is
 * commanded, the assist modes included.  A lane that the lane core moves for anyone else, such as
 * the bench or a test, is not judged against bases that the feed never took.  The grace of a load
 * from the bay is not judged either.  The standstill is asked first.  It is the sharper of the two,
 * and a filament that stood still through the close of a window is a standstill before it is a
 * shortfall. */
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

enum ace2k_feed_thresholds_fault ace2k_feed_thresholds_check(const struct ace2k_feed_thresholds *th)
{
    if (th->slip_check_um == 0 || th->slip_allow_um == 0 || th->stall_check_um == 0) {
        return ACE2K_FEED_THRESHOLDS_ZERO;
    }
    if (th->slip_allow_um >= th->slip_check_um) {
        return ACE2K_FEED_THRESHOLDS_ALLOW_NOT_UNDER_CHECK;
    }
    if (th->stall_check_um >= ACE2K_FEED_ASSIST_BURST_UM ||
        th->slip_check_um >= ACE2K_FEED_ASSIST_BURST_UM) {
        return ACE2K_FEED_THRESHOLDS_NOT_UNDER_BURST;
    }
    return ACE2K_FEED_THRESHOLDS_OK;
}

int ace2k_feed_thresholds_set(struct ace2k_feed *self, const struct ace2k_feed_thresholds *th)
{
    if (ace2k_feed_thresholds_check(th) != ACE2K_FEED_THRESHOLDS_OK) {
        return -ACE2K_EINVAL;
    }
    self->th = *th;
    return 0;
}
