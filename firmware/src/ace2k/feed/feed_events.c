/* What the host reads of the feed.  This is the event ring, with the push of its producers, the
 * notices, and the peek and drop of the binding.  It also holds the lane accessors that the state
 * reports publish.  This was split from feed.c, which keeps the modes and the tick.  The seam is
 * feed_internal.h.  The pushes run in the tick or under the caller's mask, and the reads run in
 * task context. */
#include "feed/feed_internal.h"

void ace2k_feed_push_event(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint8_t mode,
                           uint32_t motor_um, int32_t filament_um)
{
    uint8_t next = (uint8_t)((self->head + 1U) % ACE2K_FEED_EVENT_RING);
    if (next == self->tail) {
        self->dropped++;
        return;
    }
    struct ace2k_feed_event *e = &self->ring[self->head];
    e->lane = lane;
    e->kind = kind;
    e->mode = mode;
    e->seq = self->l[lane].seq;
    e->motor_um = motor_um;
    e->filament_um = filament_um;
    ACE2K_COMPILER_BARRIER(); /* payload before index */
    self->head = next;
}

/* The mode's odometers: the tach and the encoder since the mode started. */
void ace2k_feed_odometers(const struct ace2k_feed *self, uint8_t lane, uint32_t *motor_um,
                          int32_t *filament_um)
{
    const struct ace2k_feed_lane *l = &self->l[lane];
    *motor_um = ace2k_feed_motor_since_um(self, lane, l->fg_start_pulses);
    *filament_um = ace2k_feed_strand_since_um(self, lane, l->enc_um_start);
}

void ace2k_feed_notice(struct ace2k_feed *self, uint8_t lane, uint8_t kind)
{
    uint32_t motor_um;
    int32_t filament_um;
    ace2k_feed_odometers(self, lane, &motor_um, &filament_um);
    ace2k_feed_push_event(self, lane, kind, self->l[lane].mode, motor_um, filament_um);
}

void ace2k_feed_notice_at(struct ace2k_feed *self, uint8_t lane, uint8_t kind, uint32_t motor_um,
                          int32_t filament_um)
{
    ace2k_feed_push_event(self, lane, kind, self->l[lane].mode, motor_um, filament_um);
}

bool ace2k_feed_peek_event(const struct ace2k_feed *self, struct ace2k_feed_event *out)
{
    if (self->tail == self->head) {
        return false;
    }
    *out = self->ring[self->tail];
    return true;
}

void ace2k_feed_drop_event(struct ace2k_feed *self)
{
    if (self->tail == self->head) {
        return;
    }
    ACE2K_COMPILER_BARRIER(); /* the copy, if any, before the release */
    self->tail = (uint8_t)((self->tail + 1U) % ACE2K_FEED_EVENT_RING);
}

bool ace2k_feed_pop_event(struct ace2k_feed *self, struct ace2k_feed_event *out)
{
    if (!ace2k_feed_peek_event(self, out)) {
        return false;
    }
    ace2k_feed_drop_event(self);
    return true;
}

bool ace2k_feed_has_events(const struct ace2k_feed *self)
{
    return self->tail != self->head;
}

static bool one_lane_busy(const struct ace2k_feed *self, uint8_t lane)
{
    uint8_t mode = self->l[lane].mode;
    return (bool)(mode != ACE2K_FEED_IDLE && mode != ACE2K_FEED_ERROR);
}

bool ace2k_feed_lane_busy(const struct ace2k_feed *self, uint8_t lane)
{
    if (lane == ACE2K_LANE_ALL) {
        for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
            if (one_lane_busy(self, i)) {
                return true;
            }
        }
        return false;
    }
    if (lane >= ACE2K_LANE_COUNT) {
        return false;
    }
    return one_lane_busy(self, lane);
}

uint8_t ace2k_feed_mode(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].mode : 0;
}

enum ace2k_led_lane_pattern ace2k_feed_led_pattern(uint8_t mode, bool insert)
{
    switch ((enum ace2k_feed_mode)mode) {
    case ACE2K_FEED_ERROR:
        return ACE2K_LED_LANE_ERROR;
    case ACE2K_FEED_ASSISTING:
    case ACE2K_FEED_ASSISTING_BACK:
    case ACE2K_FEED_FOLLOWING:
        return ACE2K_LED_LANE_LOADED;
    case ACE2K_FEED_FEEDING:
    case ACE2K_FEED_ROLLING_BACK:
    case ACE2K_FEED_UNLOADING:
    case ACE2K_FEED_LOADING:
        return ACE2K_LED_LANE_MOVING;
    case ACE2K_FEED_IDLE:
    default:
        return insert ? ACE2K_LED_LANE_LOADED : ACE2K_LED_LANE_OFF;
    }
}

uint8_t ace2k_feed_error(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].error_kind : 0;
}

uint16_t ace2k_feed_bursts(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].bursts : 0;
}

struct ace2k_feed_ff_counts ace2k_feed_ff_counts(const struct ace2k_feed *self, uint8_t lane)
{
    struct ace2k_feed_ff_counts c = { 0 };
    if (lane < ACE2K_LANE_COUNT) {
        c.doses = self->l[lane].doses;
        c.back_doses = self->l[lane].back_doses;
        c.taut_fixes = self->l[lane].taut_fixes;
        c.full_fixes = self->l[lane].full_fixes;
        c.epoch = self->l[lane].ff_epoch;
    }
    return c;
}

uint8_t ace2k_feed_seq(const struct ace2k_feed *self, uint8_t lane)
{
    return lane < ACE2K_LANE_COUNT ? self->l[lane].last_seq : 0;
}
