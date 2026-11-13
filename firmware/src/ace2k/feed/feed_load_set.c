/* The load's settings and read-backs: the grip, the park and the wait of a lane, the bounds and
 * the settings of a load, the legs of the confirmation, the search's reach, and how the last
 * search ended.  The load itself runs in feed_load.c.  Interrupts masked by the binding. */
#include "feed/feed_internal.h"

int ace2k_feed_grip_set(struct ace2k_feed *self, uint8_t lane, uint32_t grip_um)
{
    if (lane >= ACE2K_LANE_COUNT ||
        (grip_um != 0 &&
         !ace2k_within_u32(grip_um, ACE2K_FEED_GRIP_UM_MIN, ACE2K_FEED_GRIP_UM_MAX))) {
        return -ACE2K_EINVAL;
    }
    self->grip_um[lane] = grip_um;
    return 0;
}

/* Puts one of the grip slot's reserved values on a lane: the park or the wait. */
static int slot_set(struct ace2k_feed *self, uint8_t lane, uint32_t value)
{
    if (lane >= ACE2K_LANE_COUNT) {
        return -ACE2K_EINVAL;
    }
    self->grip_um[lane] = value;
    return 0;
}

int ace2k_feed_park_set(struct ace2k_feed *self, uint8_t lane)
{
    return slot_set(self, lane, ACE2K_FEED_GRIP_PARK);
}

int ace2k_feed_wait_set(struct ace2k_feed *self, uint8_t lane)
{
    return slot_set(self, lane, ACE2K_FEED_GRIP_WAIT);
}

/* The bounds of a load, for the command and for the settings alike: a parking distance that the
 * lane can move in one move, and a pull speed within the lane's bounds. */
static bool load_in_bounds(uint32_t park_um, uint32_t speed_um_s)
{
    return (bool)(park_um != 0 && park_um <= ACE2K_LANE_MOVE_MAX_UM &&
                  ace2k_lane_speed_in_bounds(speed_um_s));
}

enum ace2k_feed_refusal ace2k_feed_load_start(struct ace2k_feed *self, uint8_t lane,
                                              uint32_t length_um, uint32_t speed_um_s, uint8_t seq,
                                              uint32_t now_ms)
{
    uint32_t park_um = length_um != 0 ? length_um : self->load.park_um;
    uint32_t pull_um_s = speed_um_s != 0 ? speed_um_s : self->load.speed_um_s;
    if (!load_in_bounds(park_um, pull_um_s)) {
        return ACE2K_FEED_REFUSED_BOUNDS;
    }
    ace2k_feed_begin_load(self, lane, park_um, pull_um_s, seq, now_ms);
    return ACE2K_FEED_ACCEPTED;
}

int ace2k_feed_load_set(struct ace2k_feed *self, const struct ace2k_feed_load_cfg *cfg)
{
    if (!load_in_bounds(cfg->park_um, cfg->speed_um_s)) {
        return -ACE2K_EINVAL;
    }
    self->load = *cfg;
    return 0;
}

void ace2k_feed_confirm_cfg_set(struct ace2k_feed *self, uint32_t leg_max_um, uint32_t speed_um_s)
{
    self->confirm_leg_um =
        leg_max_um < ACE2K_LANE_MOVE_MAX_UM ? leg_max_um : ACE2K_LANE_MOVE_MAX_UM;
    if (speed_um_s < ACE2K_LANE_SPEED_MIN_UM_S) {
        speed_um_s = ACE2K_LANE_SPEED_MIN_UM_S;
    } else if (speed_um_s > ACE2K_LANE_SPEED_MAX_UM_S) {
        speed_um_s = ACE2K_LANE_SPEED_MAX_UM_S;
    }
    self->confirm_speed_um_s = speed_um_s;
}

int ace2k_feed_search_set(struct ace2k_feed *self, uint32_t search_um)
{
    if (search_um > ACE2K_FEED_SEARCH_MAX_UM) {
        return -ACE2K_EINVAL;
    }
    self->search_um = search_um;
    return 0;
}

bool ace2k_feed_search_retry(const struct ace2k_feed *self, uint8_t lane, uint8_t seq,
                             uint32_t now_ms)
{
    if (lane >= ACE2K_LANE_COUNT || seq == 0) {
        return false;
    }
    const struct ace2k_feed_lane *l = &self->l[lane];
    if (seq != l->search_seq) {
        return false;
    }
    if (l->search_on) {
        return true; /* the search that read started still runs */
    }
    return ace2k_time_since(now_ms, l->search_end_ms) <= ACE2K_FEED_RETRY_WINDOW_MS;
}

uint8_t ace2k_feed_search_end(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].search_end : (uint8_t)ACE2K_FEED_SEARCH_NONE;
}

bool ace2k_feed_search_exhausted(const struct ace2k_feed *self, uint8_t lane)
{
    return (bool)(ace2k_feed_search_end(self, lane) == ACE2K_FEED_SEARCH_LENGTH &&
                  !self->l[lane].autonomous);
}
