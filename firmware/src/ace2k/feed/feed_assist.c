/* The feed's autonomous behaviours, with the cycles of both assists as chosen from tests on the
 * unit.
 * - The forward assist: bounded bursts at the armed speed while the filament is taut.  A burst
 *   stops when the buffer is back at rest or reads full.  It carries the notice of its taut
 *   episode.
 * - The reverse assist's take-ups, while the switches read the buffer full.
 * - The follow's arming.  Its step and the feed-forward are in feed_follow.c.
 * - The load's automatic trigger, on the insert's rising edge.  The load itself is in feed_load.c.
 * This was split from feed.c, which keeps the modes and the tick.  The seam is feed_internal.h.
 * The tick runs in interrupt context, and the starts run from task context with interrupts
 * masked. */
#include "feed/feed_internal.h"

/* A move of the assist itself.  It is bounded (rule 2), and it takes the bases of the comparator
 * afresh.  So filament that the head pulled ahead during the last move is not held against this
 * one.  With lands false, it starts without the lane's landing, as for a reverse dose of the
 * feed-forward.  It returns false when the lane refused it because it still moves.  A waiting lane
 * never gets that answer.  The next tick asks again. */
bool ace2k_feed_assist_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                            uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms, bool lands)
{
    int rc = lands ? ace2k_lane_move(self->lane, lane, dir, length_um, speed_um_s, now_ms)
                   : ace2k_lane_move_unlanded(self->lane, lane, dir, length_um, speed_um_s, now_ms);
    if (rc != 0) {
        return false;
    }
    self->l[lane].sub = ACE2K_FEED_SUB_MOVING;
    ace2k_feed_compare_restart(self, lane);
    return true;
}

/* A burst or a take-up: a move of the assist itself, counted in bursts.  A dose of the feed-forward
 * is counted apart, in doses. */
void ace2k_feed_assist_counted_move(struct ace2k_feed *self, uint8_t lane, enum ace2k_lane_dir dir,
                                    uint32_t length_um, uint32_t speed_um_s, uint32_t now_ms)
{
    if (ace2k_feed_assist_move(self, lane, dir, length_um, speed_um_s, now_ms, true)) {
        self->l[lane].bursts++;
    }
}

/* One forward burst: at most ACE2K_FEED_ASSIST_BURST_UM, at the armed speed. */
void ace2k_feed_assist_burst(struct ace2k_feed *self, uint8_t lane, uint32_t now_ms)
{
    ace2k_feed_assist_counted_move(self, lane, ACE2K_LANE_FORWARD, ACE2K_FEED_ASSIST_BURST_UM,
                                   self->l[lane].speed_um_s, now_ms);
}

/* The forward assist keeps the buffer at rest.  This was chosen from tests on the unit.  Observed
 * on the unit: a cycle that ran until the buffer read full kept the filament compressed in the
 * tube, and every burst that the head did not use ran to its bound.
 * The plunger gives three readings.
 * - pushed: the filament is taut, because the head has pulled.  It starts a burst.
 * - rest: the spring is centred.  It ends the burst.
 * - The shared full reading ends the burst too, as a safety.  It is one input for the four lanes.
 *   So the plunger of another lane at its pulled end pauses this lane, and the taut of this lane
 *   restarts it only once that reading is gone.
 * Nothing else starts a burst, neither rest nor a timer.  A burst that reaches its bound while
 * still taut chains into the next one on that tick (on_result, in feed.c). */
static void assist_forward(struct ace2k_feed *self, uint8_t lane,
                           const struct ace2k_feed_inputs *in, uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    bool taut = ace2k_feed_bit(in->pushed, lane);
    if (l->sub == ACE2K_FEED_SUB_MOVING) {
        if (in->pulled_any || ace2k_feed_bit(in->rest, lane)) {
            ace2k_feed_stop_discard(self, lane, now_ms); /* the end of a burst is not an event */
            l->sub = ACE2K_FEED_SUB_WAITING;
        }
        return;
    }
    if (taut && !in->pulled_any) {
        ace2k_feed_assist_burst(self, lane, now_ms);
    }
}

/* The taut episode of a forward assist.  The behind notice means that a burst reached its bound
 * with the filament still taut (on_result, in feed.c).  It goes out once per episode.  The episode
 * ends when the plunger leaves its pushed end, and that arms the notice again.  A filament that is
 * taut and still under command is no concern of the episode.  The comparator's standstill ends it
 * as tangled, stall_check_um of motor after the filament stopped.  This was chosen from tests on
 * the unit. */
static void assist_taut(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in)
{
    if (!ace2k_feed_bit(in->pushed, lane)) {
        self->l[lane].behind_sent = false;
    }
}

/* The reverse assist keeps the buffer at rest too.  This was chosen from tests on the unit.
 * Observed on the unit: a creep at rest, meant to keep the filament taut against the head, ran on
 * with nothing holding the filament.  56 creeps took up 267 mm, and would have unloaded the lane.
 * The full of this lane means that the head pushed filament back.  It starts a take-up of
 * ACE2K_FEED_ASSIST_BACK_FULL_UM at the armed speed.  Take-ups chain while the plunger still reads
 * full when a move ends (on_result, in feed.c).  Rest ends the take-up.  Taut ends it at once too,
 * as before.  Taut toward the spool means that the head holds the filament, which is the normal end
 * of an unload from the nozzle, and a drive that goes on grinds it.  At rest, nothing happens.  At
 * taut, nothing happens.  It runs until the host stops it. */
static void assist_back(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                        uint32_t now_ms)
{
    struct ace2k_feed_lane *l = &self->l[lane];
    if (l->sub == ACE2K_FEED_SUB_MOVING) {
        if (ace2k_feed_bit(in->pushed, lane) || ace2k_feed_bit(in->rest, lane)) {
            ace2k_feed_stop_discard(self, lane, now_ms); /* the end of a take-up is not an event */
            l->sub = ACE2K_FEED_SUB_WAITING;
        }
        return;
    }
    if (ace2k_feed_lane_full(in, lane)) {
        ace2k_feed_assist_counted_move(self, lane, ACE2K_LANE_REVERSE,
                                       ACE2K_FEED_ASSIST_BACK_FULL_UM, l->speed_um_s, now_ms);
    }
}

enum ace2k_feed_refusal ace2k_feed_assist_start(struct ace2k_feed *self, uint8_t lane,
                                                enum ace2k_feed_cmd cmd, uint32_t speed_um_s,
                                                uint8_t seq, uint32_t now_ms)
{
    if (!ace2k_lane_speed_in_bounds(speed_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    struct ace2k_feed_lane *l = &self->l[lane];
    uint8_t mode = ACE2K_FEED_ASSISTING;
    if (cmd == ACE2K_FEED_CMD_ASSIST_BACK) {
        mode = ACE2K_FEED_ASSISTING_BACK;
    } else if (cmd == ACE2K_FEED_CMD_ASSIST_BOTH) {
        mode = ACE2K_FEED_FOLLOWING;
    }
    ace2k_feed_enter(self, lane, mode, seq, now_ms); /* the follow's direction is none, and rest */
    l->follow_end_ms = now_ms;
    l->speed_um_s = speed_um_s;
    l->length_um = 0;
    l->sub = ACE2K_FEED_SUB_WAITING;
    /* Start the first move at once when the readings of the last tick ask for one.  It is the
     * tick's own step, run on that snapshot, so arming and the tick read the buffer the same way.
     * Forward, that needs the filament taut and the buffer not full.  Reverse, it needs the buffer
     * of this lane full.  The follow takes either.  Otherwise the lane waits for the first pull or
     * push of the head. */
    if (mode == ACE2K_FEED_ASSISTING_BACK) {
        assist_back(self, lane, &self->last, now_ms);
    } else if (mode == ACE2K_FEED_FOLLOWING) {
        ace2k_feed_follow_step(self, lane, &self->last, now_ms);
    } else {
        assist_forward(self, lane, &self->last, now_ms);
    }
    return ACE2K_FEED_ACCEPTED;
}

void ace2k_feed_auto_tick(struct ace2k_feed *self, uint8_t lane, const struct ace2k_feed_inputs *in,
                          bool rose, uint32_t now_ms)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (!in->link_ok) {
        return; /* nothing starts without a host (rule 6); the tick has ended what was armed */
    }
    /* The automatic load: filament put in at the bay entrance of a lane that was idle through this
     * tick, with seq 0, unless the lane waits (ACE2K_FEED_GRIP_WAIT).  It is a mode of the unit
     * itself, which leaves the retry key of the last host start as it was. */
    if (rose && l->mode == ACE2K_FEED_IDLE && self->load.auto_load &&
        self->grip_um[lane] != ACE2K_FEED_GRIP_WAIT) {
        ace2k_feed_begin_load(self, lane, self->load.park_um, self->load.speed_um_s, 0, now_ms);
        ace2k_feed_yield_others(self, lane); /* another lane moves: the tag search yields */
    }
    switch (l->mode) {
    case ACE2K_FEED_LOADING:
        ace2k_feed_load_tick(self, lane, in, now_ms);
        break;
    case ACE2K_FEED_ASSISTING:
        assist_forward(self, lane, in, now_ms);
        assist_taut(self, lane, in);
        break;
    case ACE2K_FEED_ASSISTING_BACK:
        assist_back(self, lane, in, now_ms);
        break;
    case ACE2K_FEED_FOLLOWING:
        ace2k_feed_follow_step(self, lane, in, now_ms);
        assist_taut(self, lane, in);
        break;
    default:
        break;
    }
}
