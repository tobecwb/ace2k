#include "heat/airflow.h"

#define MS_PER_S         1000U
#define AIRFLOW_POS_BITS 2U /* ace2k_airflow_flaps_report: bits per flap */

_Static_assert(ACE2K_FLAP_PULSE_MS > 0U && ACE2K_FLAP_PULSE_MS <= ACE2K_FLAP_PULSE_MAX_MS,
               "a flap pulse is bounded");

/* The fans are wanted while the owner asks, heat holds them (heat_holds = ace2k_heat_active():
 * a lease, a pending half, an open pulse check, or the gate reading high in any state — rule 2:
 * rule 7's release never drops them then) or rule 7 demands. */
static bool fans_wanted(const struct ace2k_airflow *self)
{
    if (self->owner_on || self->heat_holds) {
        return true;
    }
    return self->thermal_on;
}

/* Writes the fans only when the demand changed. */
static void apply_fans(struct ace2k_airflow *self)
{
    bool want = fans_wanted(self);
    if (want == self->commanded) {
        return;
    }
    self->commanded = want;
    self->ops->fan_write(self->ctx, want);
}

void ace2k_airflow_init(struct ace2k_airflow *self, const struct ace2k_airflow_ops *ops, void *ctx)
{
    *self = (struct ace2k_airflow){ 0 };
    self->ops = ops;
    self->ctx = ctx;
    self->owner = ACE2K_AIRFLOW_OWNER_NONE;
    ops->fan_write(ctx, false);
    for (uint32_t i = 0; i < ACE2K_FLAP_COUNT; i++) {
        ops->flap_write(ctx, (enum ace2k_flap)i, false, false);
        self->flap_done[i] = ACE2K_FLAP_UNKNOWN;
    }
}

/* The fans or the flaps may be driven by who: nobody holds them, the manual owner holds them
 * (anyone takes over from it), or who holds them. */
static bool may_drive(const struct ace2k_airflow *self, enum ace2k_airflow_owner who)
{
    if (self->owner == ACE2K_AIRFLOW_OWNER_NONE) {
        return true;
    }
    if (self->owner == ACE2K_AIRFLOW_OWNER_MANUAL) {
        return true;
    }
    return self->owner == who;
}

/* A MANUAL run carries a duration; the dryer's does not. */
static bool timed(enum ace2k_airflow_owner who)
{
    return who == ACE2K_AIRFLOW_OWNER_MANUAL;
}

static void give_up(struct ace2k_airflow *self)
{
    self->owner = ACE2K_AIRFLOW_OWNER_NONE;
    self->owner_on = false;
    apply_fans(self);
}

int ace2k_airflow_fans(struct ace2k_airflow *self, enum ace2k_airflow_owner who, bool on,
                       uint32_t seconds, uint32_t now_ms)
{
    if (who == ACE2K_AIRFLOW_OWNER_NONE) {
        return -ACE2K_EINVAL;
    }
    if (!may_drive(self, who)) {
        return -ACE2K_EBUSY;
    }
    if (!on) {
        if (self->heat_holds) {
            return -ACE2K_EBUSY;
        }
        give_up(self);
        return 0;
    }
    if (timed(who)) {
        if (seconds == 0U || seconds > ACE2K_AIRFLOW_FAN_MANUAL_MAX_S) {
            return -ACE2K_EINVAL;
        }
        self->manual_until_ms = now_ms + (seconds * MS_PER_S);
    }
    self->owner = who;
    self->owner_on = true;
    apply_fans(self);
    return 0;
}

/* The ownership goes even while heat holds the fans: fans_wanted() keeps them on for heat. */
void ace2k_airflow_release(struct ace2k_airflow *self, enum ace2k_airflow_owner who)
{
    if (self->owner != who) {
        return;
    }
    give_up(self);
}

static void flap_start(struct ace2k_airflow *self, enum ace2k_flap flap,
                       const struct ace2k_flap_request *req, uint32_t now_ms)
{
    bool close_coil = true;
    if (req->open) {
        close_coil = false;
    }
    self->ops->flap_write(self->ctx, flap, req->open, close_coil);
    self->flap_run[flap] = *req;
    self->flap_until_ms[flap] = now_ms + ACE2K_FLAP_PULSE_MS;
}

static enum ace2k_flap other_flap(enum ace2k_flap flap)
{
    if (flap == ACE2K_FLAP_BOTTOM) {
        return ACE2K_FLAP_REAR;
    }
    return ACE2K_FLAP_BOTTOM;
}

/* A stronger owner's pulse runs or waits on the flap: who may not replace it. */
static bool outranked(const struct ace2k_airflow *self, enum ace2k_flap flap,
                      enum ace2k_airflow_owner who)
{
    if (self->flap_run[flap].set && self->flap_run[flap].who > who) {
        return true;
    }
    if (!self->flap_wait[flap].set) {
        return false;
    }
    return self->flap_wait[flap].who > who;
}

/* A pulse runs or waits on either flap. */
static bool flap_busy(const struct ace2k_airflow *self)
{
    for (uint32_t i = 0U; i < (uint32_t)ACE2K_FLAP_COUNT; i++) {
        if (self->flap_run[i].set || self->flap_wait[i].set) {
            return true;
        }
    }
    return false;
}

/* NONE and MANUAL are never queued: their pulse runs now or is refused, so an accepted one is
 * answered for what happens.  Only the dryer queues.  The owners are in the order of
 * precedence (airflow.h), NONE and MANUAL the two lowest: exactly those at or below MANUAL. */
static bool never_queued(enum ace2k_airflow_owner who)
{
    return who <= ACE2K_AIRFLOW_OWNER_MANUAL;
}

int ace2k_airflow_flap_pulse(struct ace2k_airflow *self, enum ace2k_airflow_owner who,
                             enum ace2k_flap flap, bool open, uint32_t now_ms)
{
    if ((uint32_t)flap >= ACE2K_FLAP_COUNT) {
        return -ACE2K_EINVAL;
    }
    if (!may_drive(self, who) || outranked(self, flap, who)) {
        return -ACE2K_EBUSY;
    }
    if (never_queued(who) && flap_busy(self)) {
        return -ACE2K_EBUSY;
    }
    struct ace2k_flap_request req = {
        .set = true,
        .open = open,
        .who = who,
    };
    if (self->flap_run[flap].set || self->flap_run[other_flap(flap)].set) {
        self->flap_wait[flap] = req;
        return 0;
    }
    flap_start(self, flap, &req, now_ms);
    return 0;
}

/* An NTC counts as hot above ACE2K_AIRFLOW_FAN_ON_MC, or when its reading is invalid. */
static bool ntc_hot(int32_t mc, bool valid)
{
    if (!valid) {
        return true;
    }
    return mc > ACE2K_AIRFLOW_FAN_ON_MC;
}

static bool ntc_cool(int32_t mc, bool valid)
{
    if (!valid) {
        return false;
    }
    return mc < ACE2K_AIRFLOW_FAN_OFF_MC;
}

/* A valid NTC at or above the release threshold. */
static bool ntc_measured_warm(int32_t mc, bool valid)
{
    if (!valid) {
        return false;
    }
    return mc >= ACE2K_AIRFLOW_FAN_OFF_MC;
}

static void thermal_rule(struct ace2k_airflow *self, int32_t left_mc, int32_t right_mc,
                         bool left_valid, bool right_valid)
{
    self->measured_warm = ntc_measured_warm(left_mc, left_valid);
    if (ntc_measured_warm(right_mc, right_valid)) {
        self->measured_warm = true;
    }
    if (ntc_hot(left_mc, left_valid) || ntc_hot(right_mc, right_valid)) {
        self->thermal_on = true;
        return;
    }
    if (ntc_cool(left_mc, left_valid) && ntc_cool(right_mc, right_valid)) {
        self->thermal_on = false;
    }
}

static void owner_expiry(struct ace2k_airflow *self, uint32_t now_ms)
{
    if (!timed(self->owner) || self->heat_holds) {
        return;
    }
    if (!ace2k_time_after(now_ms, self->manual_until_ms)) {
        return;
    }
    give_up(self);
}

/* A pulse that ran to its end publishes where it left the flap; then the other flap's waiting
 * request starts first (no starving), else this flap's own. */
static void flap_service(struct ace2k_airflow *self, enum ace2k_flap flap, uint32_t now_ms)
{
    struct ace2k_flap_request *run = &self->flap_run[flap];
    if (!run->set || !ace2k_time_after(now_ms, self->flap_until_ms[flap])) {
        return;
    }
    self->ops->flap_write(self->ctx, flap, false, false);
    self->flap_done[flap] = run->open ? ACE2K_FLAP_OPEN : ACE2K_FLAP_CLOSED;
    run->set = false;
    enum ace2k_flap next = other_flap(flap);
    if (!self->flap_wait[next].set) {
        next = flap;
    }
    if (self->flap_wait[next].set) {
        struct ace2k_flap_request req = self->flap_wait[next];
        self->flap_wait[next].set = false;
        flap_start(self, next, &req, now_ms);
    }
}

void ace2k_airflow_tick(struct ace2k_airflow *self, int32_t ntc_left_mc, int32_t ntc_right_mc,
                        bool left_valid, bool right_valid, uint32_t now_ms)
{
    thermal_rule(self, ntc_left_mc, ntc_right_mc, left_valid, right_valid);
    owner_expiry(self, now_ms);
    apply_fans(self);
    for (uint32_t i = 0; i < ACE2K_FLAP_COUNT; i++) {
        flap_service(self, (enum ace2k_flap)i, now_ms);
    }
}

void ace2k_airflow_shutdown(struct ace2k_airflow *self)
{
    for (uint32_t i = 0; i < ACE2K_FLAP_COUNT; i++) {
        if (self->flap_run[i].set) {
            self->ops->flap_write(self->ctx, (enum ace2k_flap)i, false, false);
            self->flap_done[i] = ACE2K_FLAP_UNKNOWN; /* cut mid-travel, perhaps */
        }
        self->flap_run[i].set = false;
        self->flap_wait[i].set = false;
    }
    give_up(self);
}

enum ace2k_flap_pos ace2k_airflow_flap_pos(const struct ace2k_airflow *self, enum ace2k_flap flap)
{
    if (self->flap_run[flap].set || self->flap_wait[flap].set) {
        return ACE2K_FLAP_UNKNOWN;
    }
    return self->flap_done[flap];
}

uint8_t ace2k_airflow_flaps_report(const struct ace2k_airflow *self)
{
    return (uint8_t)((uint32_t)ace2k_airflow_flap_pos(self, ACE2K_FLAP_BOTTOM) |
                     ((uint32_t)ace2k_airflow_flap_pos(self, ACE2K_FLAP_REAR) << AIRFLOW_POS_BITS));
}

bool ace2k_airflow_flap_pending(const struct ace2k_airflow *self, enum ace2k_flap flap,
                                enum ace2k_airflow_owner who)
{
    if (self->flap_run[flap].set && self->flap_run[flap].who == who) {
        return true;
    }
    if (!self->flap_wait[flap].set) {
        return false;
    }
    return self->flap_wait[flap].who == who;
}

bool ace2k_airflow_fans_required(const struct ace2k_airflow *self)
{
    if (self->owner == ACE2K_AIRFLOW_OWNER_DRYER || self->heat_holds) {
        return true;
    }
    if (!self->thermal_on) {
        return false;
    }
    return self->measured_warm; /* rule 7 holds on a reading, not only on an invalid NTC */
}

bool ace2k_airflow_fans_commanded(const struct ace2k_airflow *self)
{
    return self->commanded;
}

uint8_t ace2k_airflow_fans_read(const struct ace2k_airflow *self)
{
    return self->ops->fan_read(self->ctx);
}
