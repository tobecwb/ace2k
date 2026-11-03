/* The feed's start command.  It holds the refusals that come before every command, the transport's
 * retry, and the moves that a start begins.  A feed, a rollback and an unload start here.  The
 * assists, the follow among them, start through feed_assist.c, and the load through feed_load.c.
 * This was split from feed.c, which keeps the modes, the tick and the events.  The seam is
 * feed_internal.h.  Task context, with interrupts masked by the binding. */
#include "feed/feed_internal.h"

static enum ace2k_feed_refusal map_move_rc(int rc)
{
    if (rc == 0) {
        return ACE2K_FEED_ACCEPTED;
    }
    return rc == -ACE2K_EREFUSED ? ACE2K_FEED_REFUSED_BUSY : ACE2K_FEED_REFUSED_BOUNDS;
}

/* The command needs filament at the bay entrance.
 * - A feed has nothing to push without one.  With the stop on the encoder, it would only end
 *   tangled once stall_check_um of motor had turned.  The wheels of lanes 2–4 are not dragged
 *   (docs/hardware.md "Motors").
 * - An unload would rewind the whole budget and end in unload_incomplete.
 * - An assist or a load has nothing to assist or to load.
 * A rollback is the one exception, because it is what brings a piece parked past the sensor back
 * out (measured on the unit). */
static bool needs_strand(enum ace2k_feed_cmd cmd)
{
    return cmd != ACE2K_FEED_CMD_ROLLBACK;
}

/* The refusals that come before every command, in this order.  The tag search's start takes them
 * too (feed_load.c), so the two give the same answers.
 * 1. The lane.
 * 2. The link.
 * 3. The error state.
 * 4. Any mode other than idle.
 * 5. A move that the lane core runs for someone else, such as the bench commands of a build that
 *    carries them.  An assist or a load armed over it would judge that move as its own.
 * 6. The insert sensors unknown: the ADC scan stopped.  Every command, the rollback too, because
 *    each one judges the insert while it runs, and the tick would stop it at once.
 * 7. The filament at the bay entrance, for the commands that need one. */
enum ace2k_feed_refusal ace2k_feed_start_refusal(const struct ace2k_feed *self, uint8_t lane,
                                                 enum ace2k_feed_cmd cmd)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    if (!self->last.link_ok) {
        return ACE2K_FEED_REFUSED_NO_LINK;
    }
    if (self->l[lane].mode == ACE2K_FEED_ERROR) {
        return ACE2K_FEED_REFUSED_IN_ERROR;
    }
    if (self->l[lane].mode != ACE2K_FEED_IDLE || ace2k_lane_is_moving(self->lane, lane)) {
        return ACE2K_FEED_REFUSED_BUSY;
    }
    if (self->last.insert_unknown) {
        return ACE2K_FEED_REFUSED_SENSORS_STALE;
    }
    if (needs_strand(cmd) && !ace2k_feed_bit(self->last.insert, lane)) {
        return ACE2K_FEED_REFUSED_NO_FILAMENT;
    }
    return ACE2K_FEED_ACCEPTED;
}

/* The modes that an armed stop is meant for (ace2k_feed_stop_when).  Held only on a rollback and an
 * unload, the two that move toward the spool from start to end: a load judges held on its return
 * and its reacquire, and would end idle in the middle of its search.  At rest on every mode but the
 * follow, which keeps the plunger at rest and would end on its second tick.  The host refuses the
 * same, so this fires only for a host with a bug or an older host. */
static bool stop_fits(enum ace2k_feed_cmd cmd, struct ace2k_feed_stop_on stop)
{
    if (stop.held && cmd != ACE2K_FEED_CMD_ROLLBACK && cmd != ACE2K_FEED_CMD_UNLOAD) {
        return false;
    }
    if (stop.at_rest && cmd == ACE2K_FEED_CMD_ASSIST_BOTH) {
        return false;
    }
    return true;
}

/* Klipper resends a query whose response it did not get.  The rule is the one of
 * ace2k_feed_start_tip in feed.h, and `free_tip` here is the effective tip.  The window follows the
 * end of the last mode of the lane, whether done, stopped or an error.  A lane in error keeps it
 * too, and a mode of the unit itself may have run since.  The same seq with anything else changed
 * is a new command, because the counter of the host wrapped, or because of a bug in the host.  Seq
 * 0 is the firmware's own, and never a retry. */
static bool is_retry(const struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_cmd cmd,
                     uint32_t length_um, uint32_t speed_um_s, uint8_t seq, bool free_tip,
                     uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || seq == 0) {
        return false;
    }
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (seq != l->last_seq || (uint8_t)cmd != l->last_cmd || length_um != l->last_length_um ||
        speed_um_s != l->last_speed_um_s || free_tip != l->last_free_tip) {
        return false;
    }
    if (l->seq == seq && l->mode != ACE2K_FEED_ERROR) {
        return true; /* the mode this start entered still runs */
    }
    return ace2k_time_since(now_ms, l->t_end_ms) <= ACE2K_FEED_RETRY_WINDOW_MS;
}

/* A feed, a rollback or an unload: one lane move.  The mode is entered once the lane took the move.
 * The length of an unload is a budget, ACE2K_FEED_UNLOAD_MAX_UM when the command gives none. */
static enum ace2k_feed_refusal start_move(struct ace2k_feed *self, uint8_t lane,
                                          enum ace2k_feed_cmd cmd, uint32_t length_um,
                                          uint32_t speed_um_s, uint8_t seq, bool free_tip,
                                          uint32_t now_ms)
{
    enum ace2k_lane_dir dir = cmd == ACE2K_FEED_CMD_FEED ? ACE2K_LANE_FORWARD : ACE2K_LANE_REVERSE;
    uint8_t mode = ACE2K_FEED_UNLOADING;
    uint32_t travel_um = length_um;
    if (cmd == ACE2K_FEED_CMD_FEED) {
        mode = ACE2K_FEED_FEEDING;
    } else if (cmd == ACE2K_FEED_CMD_ROLLBACK) {
        mode = ACE2K_FEED_ROLLING_BACK;
    } else if (length_um == 0) {
        travel_um = ACE2K_FEED_UNLOAD_MAX_UM;
    }
    enum ace2k_feed_refusal verdict =
        map_move_rc(ace2k_lane_move(self->lane, lane, dir, travel_um, speed_um_s, now_ms));
    if (verdict == ACE2K_FEED_ACCEPTED) {
        ace2k_feed_enter(self, lane, mode, seq, now_ms);
        self->l[lane].speed_um_s = speed_um_s;
        self->l[lane].length_um = length_um;
        self->l[lane].free_tip = free_tip;
    }
    return verdict;
}

enum ace2k_feed_refusal ace2k_feed_start(struct ace2k_feed *self, uint8_t lane,
                                         enum ace2k_feed_cmd cmd, uint32_t length_um,
                                         uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
{
    return ace2k_feed_start_tip(self, lane, cmd, length_um, speed_um_s, seq, false, now_ms);
}

enum ace2k_feed_refusal ace2k_feed_start_tip(struct ace2k_feed *self, uint8_t lane,
                                             enum ace2k_feed_cmd cmd, uint32_t length_um,
                                             uint32_t speed_um_s, uint8_t seq, bool free_tip,
                                             uint32_t now_ms)
{
    /* The stop armed for this start goes with it, whatever the answer (ace2k_feed_stop_when). */
    const struct ace2k_feed_stop_on stop = ace2k_feed_stop_take(self, lane);
    if (self->shut_down) {
        return ACE2K_FEED_REFUSED_NO_LINK; /* after a Klipper shutdown, not even a retry */
    }
    /* The effective tip, see ace2k_feed_start_tip.  The retry key and start_move take it. */
    const bool tip =
        (bool)((cmd == ACE2K_FEED_CMD_FEED || cmd == ACE2K_FEED_CMD_ROLLBACK) && free_tip);
    if (is_retry(self, lane, cmd, length_um, speed_um_s, seq, tip, now_ms)) {
        /* The same command once more, so there is nothing to start.  But it is a host start, so a
         * grip that the host set since that start ends here too, as the host believes. */
        self->grip_um[lane] = 0;
        return ACE2K_FEED_ACCEPTED;
    }
    enum ace2k_feed_refusal verdict = ace2k_feed_start_refusal(self, lane, cmd);
    if (verdict != ACE2K_FEED_ACCEPTED) {
        return verdict;
    }
    if (!stop_fits(cmd, stop)) {
        return ACE2K_FEED_REFUSED_BOUNDS; /* before the grip and the move: nothing changes */
    }
    /* An accepted host start on the lane ends its grip.  It is cleared before the start, so that a
     * load that the host starts never takes it, because ace2k_feed_begin_load reads it.  It is put
     * back when the bounds of the command refuse the start, because a refused start leaves the
     * grip. */
    const uint32_t grip_um = self->grip_um[lane];
    self->grip_um[lane] = 0;
    switch (cmd) {
    case ACE2K_FEED_CMD_FEED:
    case ACE2K_FEED_CMD_ROLLBACK:
    case ACE2K_FEED_CMD_UNLOAD:
        verdict = start_move(self, lane, cmd, length_um, speed_um_s, seq, tip, now_ms);
        break;
    case ACE2K_FEED_CMD_ASSIST:
    case ACE2K_FEED_CMD_ASSIST_BACK:
    case ACE2K_FEED_CMD_ASSIST_BOTH:
        verdict = ace2k_feed_assist_start(self, lane, cmd, speed_um_s, seq, now_ms);
        break;
    case ACE2K_FEED_CMD_LOAD:
        verdict = ace2k_feed_load_start(self, lane, length_um, speed_um_s, seq, now_ms);
        break;
    default:
        verdict = ACE2K_FEED_REFUSED_BOUNDS;
        break;
    }
    if (verdict != ACE2K_FEED_ACCEPTED) {
        self->grip_um[lane] = grip_um;
    } else {
        struct ace2k_feed_lane *l = &self->l[lane];
        /* The retry key, see ace2k_feed_start_tip.  The lane's speed_um_s follows a set_speed. */
        l->last_seq = seq;
        l->last_cmd = (uint8_t)cmd;
        l->last_length_um = length_um;
        l->last_speed_um_s = speed_um_s;
        l->last_free_tip = tip;
        l->stop_on = stop;                   /* the entry cleared the mode's: this start's own */
        ace2k_feed_yield_others(self, lane); /* a start on this lane: a search elsewhere yields */
    }
    return verdict;
}
