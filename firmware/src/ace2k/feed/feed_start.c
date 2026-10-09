/* The start command of the feed: the refusals that precede
 * every command, the transport's retry, and the moves a start begins — a feed, a rollback, an
 * unload here, the assists — the follow among them — and the load through feed_assist.c and
 * feed_load.c.  Split from feed.c, which keeps the modes, the tick and the events; the seam is
 * feed_internal.h.  Task context, interrupts masked by the binding. */
#include "feed/feed_internal.h"

static enum ace2k_feed_refusal map_move_rc(int rc)
{
    if (rc == 0) {
        return ACE2K_FEED_ACCEPTED;
    }
    return rc == -ACE2K_EREFUSED ? ACE2K_FEED_REFUSED_BUSY : ACE2K_FEED_REFUSED_BOUNDS;
}

/* The command wants a strand at the mouth: a feed has nothing to push without one and, with the
 * stop on the encoder, would only end tangled once stall_check_um of motor had turned (the
 * wheels of lanes 2–4 are not dragged: docs/hardware.md "Motors"); an unload would rewind the
 * whole budget and end in unload_incomplete; an assist or a load has nothing to assist or to
 * load.  A rollback is the one exception: it is what brings a piece
 * parked past the sensor back out (measured on the unit). */
static bool needs_strand(enum ace2k_feed_cmd cmd)
{
    return cmd != ACE2K_FEED_CMD_ROLLBACK;
}

/* The refusals that precede every command, in this order: the lane, the link, the error state,
 * any other mode than idle, a move the lane core runs for someone else (the bench's commands on
 * a build that carries them — an assist or a load armed over it would judge that move as its
 * own), and the strand at the mouth for the commands that need one. */
static enum ace2k_feed_refusal start_refusal(const struct ace2k_feed *self, uint8_t lane,
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
    if (needs_strand(cmd) && !ace2k_feed_bit(self->last.insert, lane)) {
        return ACE2K_FEED_REFUSED_NO_FILAMENT;
    }
    return ACE2K_FEED_ACCEPTED;
}

/* Klipper re-sends a query whose response it did not get.  A start that repeats the lane's last
 * host start whole — seq, command, length and speed — is that retry, not a second command: while
 * the mode that start entered still runs, or within the window after the lane's last mode ended
 * (done, stopped or an error — a lane in error keeps the window too; a mode of the unit's own
 * may have run since).  The same seq with anything else changed is a new command (the host's
 * counter wrapped, or a host bug) and takes the normal path.  Seq 0 is the firmware's own and
 * never a retry. */
static bool is_retry(const struct ace2k_feed *self, uint8_t lane, enum ace2k_feed_cmd cmd,
                     uint32_t length_um, uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || seq == 0) {
        return false;
    }
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (seq != l->last_seq || (uint8_t)cmd != l->last_cmd || length_um != l->last_length_um ||
        speed_um_s != l->last_speed_um_s) {
        return false;
    }
    if (l->seq == seq && l->mode != ACE2K_FEED_ERROR) {
        return true; /* the mode this start entered still runs */
    }
    return ace2k_time_since(now_ms, l->t_end_ms) <= ACE2K_FEED_RETRY_WINDOW_MS;
}

/* A feed, a rollback or an unload: one lane move, the mode entered once the lane took it.  The
 * unload's length is a budget, ACE2K_FEED_UNLOAD_MAX_UM when the command gives none. */
static enum ace2k_feed_refusal start_move(struct ace2k_feed *self, uint8_t lane,
                                          enum ace2k_feed_cmd cmd, uint32_t length_um,
                                          uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
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
    }
    return verdict;
}

enum ace2k_feed_refusal ace2k_feed_start(struct ace2k_feed *self, uint8_t lane,
                                         enum ace2k_feed_cmd cmd, uint32_t length_um,
                                         uint32_t speed_um_s, uint8_t seq, uint32_t now_ms)
{
    if (self->shut_down) {
        return ACE2K_FEED_REFUSED_NO_LINK; /* after a Klipper shutdown, not even a retry */
    }
    if (is_retry(self, lane, cmd, length_um, speed_um_s, seq, now_ms)) {
        /* the same command once more: nothing to start — but a host start, so the grip a host
         * set since that start ends here too, as the host believes */
        self->grip_um[lane] = 0;
        return ACE2K_FEED_ACCEPTED;
    }
    enum ace2k_feed_refusal verdict = start_refusal(self, lane, cmd);
    if (verdict != ACE2K_FEED_ACCEPTED) {
        return verdict;
    }
    /* any host start on the lane ends its grip */
    self->grip_um[lane] = 0;
    switch (cmd) {
    case ACE2K_FEED_CMD_FEED:
    case ACE2K_FEED_CMD_ROLLBACK:
    case ACE2K_FEED_CMD_UNLOAD:
        verdict = start_move(self, lane, cmd, length_um, speed_um_s, seq, now_ms);
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
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    if (verdict == ACE2K_FEED_ACCEPTED) {
        struct ace2k_feed_lane *l = &self->l[lane];
        /* the retry key: this start as it asked (the lane's speed_um_s follows a set_speed) */
        l->last_seq = seq;
        l->last_cmd = (uint8_t)cmd;
        l->last_length_um = length_um;
        l->last_speed_um_s = speed_um_s;
        ace2k_feed_yield_others(self, lane); /* a start on this lane: a search elsewhere yields */
    }
    return verdict;
}
