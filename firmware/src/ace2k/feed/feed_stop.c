/* The feed's armed stop.  The host arms it for its next start on a lane.  That one move then also
 * ends when the tip is held, or when the plunger is at rest.
 * Held means the plunger taut with the filament still, on a move toward the spool.  A taut plunger
 * alone is no stop.  A thick tip dragged through a narrow tube pulls the plunger taut while the
 * path ahead is free, and the filament still comes.  Toward the head, a taut plunger with the
 * filament still is a tangle, never a stop, so held is not judged there.  Held is judged only
 * while a move toward the spool runs.  The reverse assist ends its take-up as soon as the plunger
 * reads taut, and then waits with no move, so held never ends it.  A start takes held only for a
 * rollback and an unload, and at rest for any mode but the follow (feed_start.c).  At rest is
 * judged in either direction.
 * Held ends the move after its window, or on a comparator trip with the plunger taut and the
 * filament still.  So a held tip never ends stuck.  The window is the held_ms that the arming
 * carried, or the setting of the unit for 0.  Each move has its own, so a fast move and a slow one
 * on the same unit never share it.
 * An armed stop goes before the tip snag's reverse tolerance.  A tip that catches the tube after
 * the filament came back freely ends the move held.  An at-rest stop ends it when the touch of the
 * tolerance brings the plunger back to rest.  The filament_um of the event tells the two apart.  A
 * tip held at the head comes back no more than the buffer's slack.  A catch after free travel comes
 * back at least the tolerance's free_um.
 * This was split from feed.c, which keeps the modes and the tick.  The seam is feed_internal.h.
 * The step runs in the feed's tick (interrupt context), and the setters run under the caller's
 * mask. */
#include "feed/feed_internal.h"

int ace2k_feed_stop_when(struct ace2k_feed *self, uint8_t lane, bool held, bool at_rest,
                         uint32_t held_ms)
{
    if (lane >= ACE2K_LANE_COUNT ||
        (held_ms != 0U &&
         !ace2k_within_u32(held_ms, ACE2K_FEED_HELD_MS_MIN, ACE2K_FEED_HELD_MS_MAX))) {
        return -ACE2K_EINVAL;
    }
    if (ace2k_feed_tiebreak_on(&self->l[lane])) {
        self->l[lane].yield = true; /* the tie-break goes back at the next tick, as at a start */
    }
    if (self->l[lane].mode != ACE2K_FEED_IDLE) {
        return -ACE2K_EBUSY; /* a stop armed under a running move would never be the host's */
    }
    self->l[lane].stop_next = (struct ace2k_feed_stop_on){
        .held = held,
        .at_rest = at_rest,
        .held_ms = (uint16_t)held_ms,
    };
    return 0;
}

struct ace2k_feed_stop_on ace2k_feed_stop_take(struct ace2k_feed *self, uint8_t lane)
{
    const struct ace2k_feed_stop_on none = { .held = false, .at_rest = false };
    if (lane >= ACE2K_LANE_COUNT) {
        return none;
    }
    struct ace2k_feed_stop_on armed = self->l[lane].stop_next;
    self->l[lane].stop_next = none;
    return armed;
}

/* The held window, for one tick.  It opens when the plunger reads taut on a move toward the spool,
 * and closes on any other tick.  A filament that moved held_um since the window opened is not held,
 * and the window starts again from here.  It returns true once the window has lasted the move's own
 * held_ms, or that of the unit when the arming carried 0. */
static bool held_now(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                     uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!ace2k_feed_bit(in->pushed, lane) || !ace2k_lane_is_reverse(self->lane, lane)) {
        l->held_open = false;
        return false;
    }
    uint32_t moved_um = ace2k_abs_i32(ace2k_feed_strand_since_um(self, lane, l->held_enc0_um));
    if (!l->held_open || moved_um >= self->held.held_um) {
        l->held_open = true;
        l->held_t0_ms = now_ms;
        l->held_enc0_um = ace2k_lane_encoder_um(self->lane, lane);
        return false;
    }
    const uint32_t window_ms = l->stop_on.held_ms != 0U ? l->stop_on.held_ms : self->held.held_ms;
    return ace2k_time_after(now_ms, l->held_t0_ms + window_ms);
}

bool ace2k_feed_stop_step(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (!l->stop_on.held && !l->stop_on.at_rest) {
        return false;
    }
    /* At rest is not judged on the mode's first tick.  A move started with the plunger at rest
     * runs that tick, and ends on the next one if the plunger still reads rest. */
    bool first = (bool)!l->stop_judged;
    l->stop_judged = true;
    if (l->stop_on.at_rest && !first && ace2k_feed_bit(in->rest, lane)) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_AT_REST, now_ms);
        return true;
    }
    if (l->stop_on.held && held_now(self, lane, in, now_ms)) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_HELD, now_ms);
        return true;
    }
    return false;
}

bool ace2k_feed_stop_on_trip(struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_trip trip,
                             const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!l->stop_on.held || !ace2k_feed_bit(in->pushed, lane) ||
        !ace2k_lane_is_reverse(self->lane, lane)) {
        return false;
    }
    /* Still means the standstill, or at least the partial's allowance in motor since the
     * standstill's base.  At any partial trip, the filament is under 1.5 mm since that base,
     * because the standstill runs first and refreshes it.  A filament that moves but slips keeps
     * refreshing that base. */
    if (trip == ACE2K_FEED_TRIP_STANDSTILL ||
        ace2k_feed_motor_since_um(self, lane, l->fg_still_pulses) >= self->th.slip_allow_um) {
        ace2k_feed_finish(self, lane, ACE2K_FEED_HELD, now_ms);
        return true;
    }
    /* A partial that carries a shortfall from before the filament stopped gets no verdict yet.  It
     * asks again on the next tick, for at most slip_allow_um more motor.  A slip then ends
     * stuck. */
    return ace2k_feed_motor_since_um(self, lane, l->fg_window_pulses) <
           self->th.slip_check_um + self->th.slip_allow_um;
}

int ace2k_feed_held_set(struct ace2k_feed *self, uint32_t held_um, uint32_t held_ms)
{
    if (!ace2k_within_u32(held_um, ACE2K_FEED_HELD_UM_MIN, ACE2K_FEED_HELD_UM_MAX) ||
        !ace2k_within_u32(held_ms, ACE2K_FEED_HELD_MS_MIN, ACE2K_FEED_HELD_MS_MAX)) {
        return -ACE2K_EINVAL;
    }
    self->held = (struct ace2k_feed_held_cfg){ .held_um = held_um, .held_ms = held_ms };
    return 0;
}
