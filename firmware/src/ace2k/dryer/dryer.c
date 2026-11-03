/* The dryer's states: IDLE → STARTING → HEATING → COOLDOWN → IDLE, and FAULT from any of
 * them. There is one function per state. The heat latch and the cutout are checked in every state,
 * before the work of the state itself. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

/* ---- helpers ---- */

int32_t ace2k_dryer_hotter_mc(const struct ace2k_dryer_inputs *in)
{
    if (in->ntc_left_mc > in->ntc_right_mc) {
        return in->ntc_left_mc;
    }
    return in->ntc_right_mc;
}

/* A reading as the vent sees it. A chamber reading older than ACE2K_DRYER_CHAMBER_STALE_MS counts
 * as no reading, and its RH goes with it. */
static struct ace2k_dryer_vent_sample vent_sample(const struct ace2k_dryer_inputs *in)
{
    struct ace2k_dryer_vent_sample s = {
        .chamber_mc = in->chamber_mc,
        .rh_pct10 = in->chamber_rh_pct10,
        .chamber_valid = false,
        .rh_valid = false,
    };
    if (ace2k_dryer_chamber_fresh(in)) {
        s.chamber_valid = true;
        s.rh_valid = in->rh_valid;
    }
    return s;
}

static void set_state(struct ace2k_dryer *self, uint8_t state, uint32_t now_ms)
{
    self->state = state;
    self->state_since_ms = now_ms;
}

/* ---- the flaps ---- */

_Static_assert(3U * ACE2K_FLAP_PULSE_MAX_MS <= ACE2K_DRYER_FLAPS_TIMEOUT_MS,
               "a sequence's pulses, and one already running, fit in its bound");

/* Begins a sequence that pulses both flaps the same way, the bottom one asked first. Airflow runs
 * one flap at a time, so the two never pulse together. The dryer closes the flaps at boot, at the
 * start and at the end of a cool-down. It opens them at the vent and at the entry into a
 * cool-down. There is no position cache, so a flap already in place is pulsed again. A new
 * sequence replaces the one in flight. */
static void flaps_begin(struct ace2k_dryer *self, bool open, uint32_t now_ms)
{
    self->flaps = (struct ace2k_dryer_flaps){
        .next = ACE2K_FLAP_BOTTOM,
        .result = ACE2K_DRYER_FLAPS_RUNNING,
        .open = open,
        .since_ms = now_ms,
    };
}

/* Asks for the flaps not asked yet, in order. Returns false while one is refused. Today no owner
 * outranks the dryer, because NONE and MANUAL yield to it. So a refusal cannot happen. The retry
 * is kept as a defence against a stronger owner in the future. A refused flap is asked again
 * every tick, within the sequence's bound (ACE2K_DRYER_FLAPS_TIMEOUT_MS). */
static bool flaps_ask(struct ace2k_dryer *self, uint32_t now_ms)
{
    struct ace2k_dryer_flaps *seq = &self->flaps;
    while (seq->next < ACE2K_FLAP_COUNT) {
        if (ace2k_airflow_flap_pulse(self->airflow, ACE2K_AIRFLOW_OWNER_DRYER,
                                     (enum ace2k_flap)seq->next, seq->open, now_ms) != 0) {
            return false;
        }
        seq->next++;
    }
    return true;
}

/* True while a request of the dryer still runs or waits on a flap. Requests of other owners do
 * not count. */
static bool flaps_own_pending(const struct ace2k_dryer *self)
{
    if (ace2k_airflow_flap_pending(self->airflow, ACE2K_FLAP_BOTTOM, ACE2K_AIRFLOW_OWNER_DRYER)) {
        return true;
    }
    return ace2k_airflow_flap_pending(self->airflow, ACE2K_FLAP_REAR, ACE2K_AIRFLOW_OWNER_DRYER);
}

/* Runs every tick, in every state. It asks for the flaps of the sequence in flight. The sequence
 * is over once both of the dryer's pulses have ended. It times out after
 * ACE2K_DRYER_FLAPS_TIMEOUT_MS. */
static void flaps_step(struct ace2k_dryer *self, uint32_t now_ms)
{
    struct ace2k_dryer_flaps *seq = &self->flaps;
    if (seq->result != ACE2K_DRYER_FLAPS_RUNNING) {
        return;
    }
    if (flaps_ask(self, now_ms) && !flaps_own_pending(self)) {
        seq->result = ACE2K_DRYER_FLAPS_OVER;
        return;
    }
    if (ace2k_time_since(now_ms, seq->since_ms) >= ACE2K_DRYER_FLAPS_TIMEOUT_MS) {
        seq->result = ACE2K_DRYER_FLAPS_TIMED_OUT;
    }
}

static bool flaps_running(const struct ace2k_dryer *self)
{
    return self->flaps.result == ACE2K_DRYER_FLAPS_RUNNING;
}

static uint8_t map_heat_reason(uint8_t reason)
{
    switch (reason) {
    case ACE2K_HEAT_FANS:
        return ACE2K_DRYER_FAULT_FANS;
    case ACE2K_HEAT_NTC_INVALID:
    case ACE2K_HEAT_NTC_OVER:
        return ACE2K_DRYER_FAULT_NTC;
    case ACE2K_HEAT_MAINS_ABSENT:
    case ACE2K_HEAT_MAINS_IMPLAUSIBLE:
        return ACE2K_DRYER_FAULT_MAINS;
    case ACE2K_HEAT_CUTOUT:
        return ACE2K_DRYER_FAULT_CUTOUT;
    default:
        return ACE2K_DRYER_FAULT_HEAT;
    }
}

/* ---- the cooling, shared by COOLDOWN and FAULT ---- */

static bool both_valid(const struct ace2k_dryer_inputs *in)
{
    if (!in->left_valid) {
        return false;
    }
    return in->right_valid;
}

/* Starts a window, with both NTCs as its references. When either side is invalid, there is no
 * reference to take. Then the window restarts on the first tick both read valid (cool_window()). */
static void cool_arm(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in, uint32_t now_ms)
{
    self->cool_ref_mc[0] = in->ntc_left_mc;
    self->cool_ref_mc[1] = in->ntc_right_mc;
    self->cool_ref_ms = now_ms;
    self->cool_rearm = false;
    if (!both_valid(in)) {
        self->cool_rearm = true;
    }
}

static void cool_begin(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                       uint32_t now_ms)
{
    /* The flaps open only when the heater ran in this cycle. A fault on a cold unit, such as a
     * cutout glitch or a heat latch at IDLE, only closes them, at its end. */
    self->cool = ACE2K_DRYER_COOL_WAITING;
    if (self->heated) {
        self->cool = ACE2K_DRYER_COOL_OPENING;
        flaps_begin(self, true, now_ms);
    }
    cool_arm(self, in, now_ms);
}

/* True when one side is cool over the window. It is at or below 45 °C. It rose no more than
 * ACE2K_DRYER_COOL_RISE_MC, which is the NTC's noise. It fell less than
 * ACE2K_DRYER_COOL_FALL_MC. So it is steady, or falling slowly. */
static bool side_cool(int32_t now_mc, int32_t ref_mc)
{
    if (now_mc > ACE2K_DRYER_COOL_MC) {
        return false;
    }
    int32_t fell = ref_mc - now_mc;
    if (fell < -ACE2K_DRYER_COOL_RISE_MC) {
        return false; /* it rose: this is the overshoot after the heater stops */
    }
    return fell < ACE2K_DRYER_COOL_FALL_MC;
}

/* True when a 30 s window just closed with both NTCs valid, and both cool by side_cool(). The
 * first window closes 30 s after cool_begin(). A window that closes with either side invalid does
 * not roll over. Its references would be values from before the outage. The first window after
 * the side came back would then judge a rising outlet against them. Instead, the window restarts
 * from fresh references on the first tick both sides read valid again. */
static bool cool_window(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                        uint32_t now_ms)
{
    if (self->cool_rearm) {
        if (both_valid(in)) {
            cool_arm(self, in, now_ms);
        }
        return false;
    }
    if (ace2k_time_since(now_ms, self->cool_ref_ms) < ACE2K_DRYER_COOL_MIN_MS) {
        return false;
    }
    if (!both_valid(in)) {
        self->cool_rearm = true;
        return false;
    }
    int32_t ref_l = self->cool_ref_mc[0];
    int32_t ref_r = self->cool_ref_mc[1];
    cool_arm(self, in, now_ms);
    if (!side_cool(in->ntc_left_mc, ref_l)) {
        return false;
    }
    return side_cool(in->ntc_right_mc, ref_r);
}

/* True when the cooling is over: by the cool rule, or after 10 min in a hot ambient. The hot
 * ambient raises its notice in COOLDOWN only. */
static bool cool_over(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                      uint32_t now_ms)
{
    if (cool_window(self, in, now_ms)) {
        return true;
    }
    if (ace2k_time_since(now_ms, self->state_since_ms) < ACE2K_DRYER_HOT_AMBIENT_MS) {
        return false;
    }
    if (self->state == ACE2K_DRYER_COOLDOWN) {
        self->notices |= ACE2K_DRYER_NOTICE_HOT_AMBIENT;
        ace2k_dryer_emit(self, ACE2K_DRYER_EV_HOT_AMBIENT, 0U);
    }
    return true;
}

/* COOLDOWN and FAULT. First the flaps open, after heating. The fans then draw room air across the
 * heater, which is still hot, and push the humid air out. The factory firmware closes the flaps
 * at the stop instead. Then the dryer waits for the cool rule, or 10 min in a hot ambient. Then
 * the flaps close, and the fans go to airflow's fan rule (a fan never off above 45 °C).
 * Returns true once the fans are released. */
static bool tick_cooling(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                         uint32_t now_ms)
{
    switch (self->cool) {
    case ACE2K_DRYER_COOL_OPENING:
        if (!flaps_running(self)) {
            self->cool = ACE2K_DRYER_COOL_WAITING;
        }
        return false;
    case ACE2K_DRYER_COOL_WAITING:
        if (cool_over(self, in, now_ms)) {
            self->cool = ACE2K_DRYER_COOL_CLOSING;
            flaps_begin(self, false, now_ms);
        }
        return false;
    case ACE2K_DRYER_COOL_CLOSING:
        if (flaps_running(self)) {
            return false;
        }
        ace2k_airflow_release(self->airflow, ACE2K_AIRFLOW_OWNER_DRYER);
        self->heated = false;
        self->cool = ACE2K_DRYER_COOL_RELEASED;
        return true;
    default:
        return true;
    }
}

/* ---- transitions ---- */

static void enter_cooldown(struct ace2k_dryer *self, uint8_t kind,
                           const struct ace2k_dryer_inputs *in, uint32_t now_ms)
{
    ace2k_heat_release(self->heat);
    self->ctl.duty_cpct = 0U;
    ace2k_dryer_emit(self, kind, 0U);
    cool_begin(self, in, now_ms);
    set_state(self, ACE2K_DRYER_COOLDOWN, now_ms);
}

static void enter_fault(struct ace2k_dryer *self, uint8_t reason,
                        const struct ace2k_dryer_inputs *in, uint32_t now_ms)
{
    if (self->state == ACE2K_DRYER_FAULT) {
        return; /* the first reason stays */
    }
    ace2k_heat_abort(self->heat); /* a fault: no half-cycle after it, not even a pending one */
    self->ctl.duty_cpct = 0U;
    self->fault = reason;
    /* This cannot fail. The dryer outranks every other owner, and a fan-on is never refused for
     * heat. */
    (void)ace2k_airflow_fans(self->airflow, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, now_ms);
    ace2k_dryer_log_fault(self, reason, in);
    ace2k_dryer_emit(self, ACE2K_DRYER_EV_FAULT, reason);
    cool_begin(self, in, now_ms);
    set_state(self, ACE2K_DRYER_FAULT, now_ms);
}

static void begin_heating(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                          uint32_t now_ms)
{
    self->heating_since_ms = now_ms;
    self->heated = true;
    self->ctl.renew_next_ms = now_ms;
    self->ctl.step_next_ms = now_ms;
    ace2k_dryer_protect_begin(self, in, now_ms);
    ace2k_dryer_vent_heating(&self->vent, now_ms);
    set_state(self, ACE2K_DRYER_HEATING, now_ms);
}

/* ---- states ---- */

/* STARTING waits for the flaps to close and for both fans to read high. One timeout covers both.
 * After ACE2K_DRYER_START_TIMEOUT_MS, fans that do not both read high fault FANS. Otherwise,
 * flaps not closed fault FLAPS: their sequence timed out or is still running. The fans are
 * checked first. */
static void tick_starting(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                          uint32_t now_ms)
{
    uint32_t age = ace2k_time_since(now_ms, self->state_since_ms);
    bool flaps_done = self->flaps.result == ACE2K_DRYER_FLAPS_OVER;
    bool fans_both = ace2k_airflow_fans_read(self->airflow) == ACE2K_DRYER_FANS_BOTH;
    if (!flaps_done || !fans_both) {
        if (age >= ACE2K_DRYER_START_TIMEOUT_MS) {
            uint8_t reason = fans_both ? ACE2K_DRYER_FAULT_FLAPS : ACE2K_DRYER_FAULT_FANS;
            enter_fault(self, reason, in, now_ms);
        }
        return;
    }
    if (!self->start_logged) {
        ace2k_dryer_log_cycle_open(self);
        self->start_logged = true;
    }
    /* The open mark reaches the flash before the first lease. A flash that never takes it does
     * not hold the cycle back past the timeout. The log is a record, not a safety function. */
    if (ace2k_dryer_log_dirty(self) && age < ACE2K_DRYER_START_TIMEOUT_MS) {
        return;
    }
    begin_heating(self, in, now_ms);
}

/* The reason of a refused lease, as heat names it. OK, which means only a store in progress,
 * maps to the generic heat fault. */
static uint8_t refused_reason(const struct ace2k_heat *heat)
{
    return map_heat_reason(ace2k_heat_refusal_reason(heat));
}

/* Renews the lease. On -ACE2K_EAGAIN the dryer waits and asks again next tick. Any other refusal
 * faults with heat's reason. A wait past ACE2K_DRYER_NO_LEASE_MAX_MS faults with the reason
 * that heat names at that moment. */
static void renew_lease(struct ace2k_dryer *self, uint8_t duty_pct,
                        const struct ace2k_dryer_inputs *in, uint32_t now_ms)
{
    if (!ace2k_time_after(now_ms, self->ctl.renew_next_ms)) {
        return;
    }
    self->ctl.renew_next_ms = now_ms + ACE2K_DRYER_LEASE_RENEW_MS;
    if (duty_pct == 0U) {
        self->ctl.leaseless = false; /* no lease is wanted, so this is not a wait */
        ace2k_heat_release(self->heat);
        return;
    }
    int rc = ace2k_heat_lease(self->heat, duty_pct, ACE2K_HEAT_LEASE_MAX_MS, now_ms);
    if (rc == 0) {
        self->ctl.leaseless = false;
        return;
    }
    if (rc != -ACE2K_EAGAIN) {
        enter_fault(self, refused_reason(self->heat), in, now_ms);
        return;
    }
    self->ctl.renew_next_ms = now_ms;
    if (!self->ctl.leaseless) {
        self->ctl.leaseless = true;
        self->ctl.leaseless_since_ms = now_ms;
        return;
    }
    if (ace2k_time_since(now_ms, self->ctl.leaseless_since_ms) >= ACE2K_DRYER_NO_LEASE_MAX_MS) {
        uint8_t reason = refused_reason(self->heat);
        enter_fault(self, reason, in, now_ms);
    }
}

/* dryer_vent decides. The first opening is the cycle's vent, and it sends its event. The
 * guard's moves are silent. */
static void tick_vent(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                      uint32_t now_ms)
{
    struct ace2k_dryer_vent_sample s = vent_sample(in);
    bool first = true; /* long form: in C `!b` is an int, and the lint refuses it as a bool */
    if (self->vent.vented) {
        first = false;
    }
    enum ace2k_dryer_vent_action a = ace2k_dryer_vent_step(
        &self->vent, (int32_t)self->target_c * ACE2K_DRYER_MC_PER_C, &s, now_ms);
    if (a == ACE2K_DRYER_VENT_HOLD) {
        return;
    }
    flaps_begin(self, a == ACE2K_DRYER_VENT_OPEN, now_ms);
    if (a == ACE2K_DRYER_VENT_OPEN && first) {
        ace2k_dryer_emit(self, ACE2K_DRYER_EV_VENTED, 0U);
    }
}

static void tick_heating(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                         uint32_t now_ms)
{
    if (ace2k_time_after(now_ms, self->end_ms)) {
        ace2k_dryer_log_cycle_close(self, true);
        enter_cooldown(self, ACE2K_DRYER_EV_DONE, in, now_ms);
        return;
    }
    uint8_t fault = ace2k_dryer_protect_check(self, in, now_ms);
    if (fault != ACE2K_DRYER_FAULT_NONE) {
        enter_fault(self, fault, in, now_ms);
        return;
    }
    uint8_t duty = ace2k_dryer_ctl_step(self, in, now_ms);
    renew_lease(self, duty, in, now_ms);
    if (self->state != ACE2K_DRYER_HEATING) {
        return;
    }
    tick_vent(self, in, now_ms);
}

static void tick_cooldown(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                          uint32_t now_ms)
{
    if (tick_cooling(self, in, now_ms)) {
        ace2k_dryer_vent_room_ended(&self->room, now_ms);
        set_state(self, ACE2K_DRYER_IDLE, now_ms);
    }
}

/* The reason of the cutout or of the heat latch, checked in every state. */
static uint8_t latch_reason(const struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    if (in->cutout) {
        return ACE2K_DRYER_FAULT_CUTOUT;
    }
    if (self->heat->state == ACE2K_HEAT_LATCHED) {
        return map_heat_reason(self->heat->reason);
    }
    return ACE2K_DRYER_FAULT_NONE;
}

/* A tripped cutout clears only on a power cycle, also across a reset. So a cutout seen in any
 * state, whatever fault came first, reaches the page at once. */
static void persist_cutout(struct ace2k_dryer *self)
{
    self->cutout_seen = true;
    if ((self->log->flags & ACE2K_DRYER_LOG_CUTOUT) == 0U) {
        self->log->flags |= ACE2K_DRYER_LOG_CUTOUT;
        ace2k_dryer_log_mark(self);
    }
}

/* A reset never resumes a cycle: a cycle open at the reset is logged as interrupted. A tripped
 * cutout clears only on a power cycle: a persisted cutout holds the dryer in FAULT until a
 * power-on reset clears it. The log is marked only when the boot changed it. It is not marked on
 * every power cycle, so there is no page erase at each power-on. */
static void boot_log(struct ace2k_dryer *self, bool power_on_reset)
{
    uint32_t boot = ace2k_dryer_log_boot(self->log, power_on_reset);
    if ((boot & ACE2K_DRYER_LOG_BOOT_CUTOUT) != 0U) {
        self->state = ACE2K_DRYER_FAULT;
        self->fault = ACE2K_DRYER_FAULT_CUTOUT;
        self->cutout_seen = true;
        self->cool = ACE2K_DRYER_COOL_WAITING; /* no heating here, so nothing to open */
        /* There are no inputs yet. The first window starts on the first tick both NTCs read
         * valid. So it is timed from the boot, and judged against what the unit reads, not
         * against zeros. */
        self->cool_rearm = true;
        /* The fans follow airflow's fan rule from the first tick. So a unit reset while hot
         * keeps them on. */
        /* The condition the unit booted in goes first in the ring. */
        ace2k_dryer_emit(self, ACE2K_DRYER_EV_FAULT, ACE2K_DRYER_FAULT_CUTOUT);
    }
    if ((boot & ACE2K_DRYER_LOG_BOOT_INTERRUPTED) != 0U) {
        ace2k_dryer_emit(self, ACE2K_DRYER_EV_INTERRUPTED, 0U);
        ace2k_dryer_log_mark(self);
    }
    if ((boot & ACE2K_DRYER_LOG_BOOT_CUTOUT_CLEARED) != 0U) {
        ace2k_dryer_log_mark(self); /* a cleared cutout flag must reach the page too */
    }
}

/* ---- the interface ---- */

void ace2k_dryer_init(struct ace2k_dryer *self, struct ace2k_heat *heat,
                      struct ace2k_airflow *airflow, struct ace2k_dryer_log *log,
                      bool power_on_reset, uint32_t now_ms)
{
    *self = (struct ace2k_dryer){ 0 };
    self->heat = heat;
    self->airflow = airflow;
    self->log = log;
    set_state(self, ACE2K_DRYER_IDLE, now_ms);
    /* The flaps hold their position when unpowered. A cool-down cut short by a reset leaves them
     * open, and nothing on the page says where they are. So the boot closes both from the first
     * tick, in any state. */
    flaps_begin(self, false, now_ms);
    boot_log(self, power_on_reset);
}

/* True when the mains is present but off-band: a dip in progress. */
static bool mains_dipping(const struct ace2k_dryer_inputs *in)
{
    if (!in->mains_present) {
        return false;
    }
    if (in->mains_plausible) {
        return false;
    }
    return true;
}

/* Judges the mains for a start, and for the clear of a mains fault:
 * - absent: NO_MAINS at once;
 * - present, but no window measured since it came back: MEASURING, a transient refusal;
 * - implausible while heat's bucket is above its restart level: NO_MAINS.
 * A dip alone is ridden out, and a new cycle always begins with some tolerance left. */
static enum ace2k_dryer_refusal mains_refusal(const struct ace2k_dryer *self)
{
    if (!self->last.mains_present) {
        return ACE2K_DRYER_NO_MAINS;
    }
    if (!ace2k_heat_mains_measured(self->heat)) {
        return ACE2K_DRYER_MEASURING;
    }
    return ace2k_heat_mains_recovered(self->heat) ? ACE2K_DRYER_OK : ACE2K_DRYER_NO_MAINS;
}

static enum ace2k_dryer_refusal start_inputs_refusal(const struct ace2k_dryer *self)
{
    const struct ace2k_dryer_inputs *in = &self->last;
    if (in->cutout || self->cutout_seen || (self->log->flags & ACE2K_DRYER_LOG_CUTOUT) != 0U) {
        return ACE2K_DRYER_CUTOUT;
    }
    if (!self->have_inputs || !in->left_valid || !in->right_valid ||
        !ace2k_dryer_chamber_fresh(in)) {
        return ACE2K_DRYER_SENSORS;
    }
    enum ace2k_dryer_refusal mains = mains_refusal(self);
    if (mains != ACE2K_DRYER_OK) {
        return mains;
    }
    if (self->heat->state == ACE2K_HEAT_LATCHED) {
        return ACE2K_DRYER_FAULTED;
    }
    enum ace2k_airflow_owner owner = self->airflow->owner;
    if (owner != ACE2K_AIRFLOW_OWNER_NONE && owner != ACE2K_AIRFLOW_OWNER_DRYER) {
        return ACE2K_DRYER_BUSY;
    }
    return ACE2K_DRYER_OK;
}

static enum ace2k_dryer_refusal start_state_refusal(const struct ace2k_dryer *self,
                                                    uint8_t target_c, uint16_t minutes)
{
    if (self->shut) {
        return ACE2K_DRYER_SHUTDOWN;
    }
    if (self->state == ACE2K_DRYER_FAULT) {
        return ACE2K_DRYER_FAULTED;
    }
    if (self->state != ACE2K_DRYER_IDLE) {
        return ACE2K_DRYER_BUSY;
    }
    return ace2k_dryer_request_range_refusal(target_c, minutes);
}

enum ace2k_dryer_refusal ace2k_dryer_start(struct ace2k_dryer *self, uint8_t target_c,
                                           uint16_t minutes, uint32_t now_ms)
{
    enum ace2k_dryer_refusal res = start_state_refusal(self, target_c, minutes);
    if (res == ACE2K_DRYER_OK) {
        res = start_inputs_refusal(self);
    }
    if (res != ACE2K_DRYER_OK) {
        return res;
    }
    self->target_c = target_c;
    self->highest_target_c = target_c;
    self->end_ms = now_ms + ((uint32_t)minutes * ACE2K_DRYER_MS_PER_MIN);
    self->notices = 0U;
    /* Valid, because start_inputs_refusal checked it. The ticks lower it from here
     * (track_start_chamber). */
    self->start_chamber_mc = self->last.chamber_mc;
    /* Judged once, here: the chamber started above the target. */
    if (self->start_chamber_mc > (int32_t)target_c * ACE2K_DRYER_MC_PER_C) {
        self->notices |= ACE2K_DRYER_NOTICE_AMBIENT_ABOVE;
    }
    self->mains_dips = 0U;
    self->phase_rejects_start = self->heat->phase_rejects;
    /* A dip already in progress at the start counts once it is ridden out. */
    self->mains_dip_open = mains_dipping(&self->last);
    self->fault = ACE2K_DRYER_FAULT_NONE;
    struct ace2k_dryer_vent_sample room = vent_sample(&self->last);
    uint32_t room_cg =
        ace2k_dryer_vent_room_pick(&self->room, ace2k_dryer_vent_sample_cg(&room), now_ms);
    ace2k_dryer_vent_begin_cg(&self->vent, room_cg, now_ms);
    self->start_logged = false;
    self->heated = false;
    flaps_begin(self, false, now_ms); /* replaces a boot close still in flight */
    self->prot.insert_prev = self->last.insert_mask;
    ace2k_dryer_ctl_reset(self, now_ms);
    /* This cannot fail. The dryer outranks every other owner, and the start refused any owner
     * other than NONE. A fan-on is never refused for heat. */
    (void)ace2k_airflow_fans(self->airflow, ACE2K_AIRFLOW_OWNER_DRYER, true, 0U, now_ms);
    set_state(self, ACE2K_DRYER_STARTING, now_ms);
    return ACE2K_DRYER_OK;
}

enum ace2k_dryer_refusal ace2k_dryer_stop(struct ace2k_dryer *self, uint32_t now_ms)
{
    if (self->state == ACE2K_DRYER_FAULT) {
        return ACE2K_DRYER_FAULTED;
    }
    if (!ace2k_dryer_running_state(self->state)) {
        return ACE2K_DRYER_OK; /* nothing to stop: a repeated stop does nothing */
    }
    /* STARTING opens the cycle once the flaps and fans are ready. A stop before that has nothing
     * to close and nothing to write to the page. So a cycle that never began costs no erase. */
    if (self->state == ACE2K_DRYER_HEATING || self->start_logged) {
        ace2k_dryer_log_cycle_close(self, false);
    }
    enter_cooldown(self, ACE2K_DRYER_EV_STOPPED, &self->last, now_ms);
    return ACE2K_DRYER_OK;
}

enum ace2k_dryer_refusal ace2k_dryer_retarget(struct ace2k_dryer *self, uint8_t target_c,
                                              uint16_t minutes, uint32_t now_ms)
{
    if (self->state == ACE2K_DRYER_FAULT) {
        return ACE2K_DRYER_FAULTED;
    }
    if (!ace2k_dryer_running_state(self->state)) {
        return ACE2K_DRYER_BUSY;
    }
    enum ace2k_dryer_refusal res = ace2k_dryer_request_range_refusal(target_c, minutes);
    if (res != ACE2K_DRYER_OK) {
        return res;
    }
    self->target_c = target_c;
    if (target_c > self->highest_target_c) {
        self->highest_target_c = target_c;
    }
    self->end_ms = now_ms + ((uint32_t)minutes * ACE2K_DRYER_MS_PER_MIN);
    self->notices &= (uint8_t)~ACE2K_DRYER_NOTICE_LOWERED;
    return ACE2K_DRYER_OK;
}

void ace2k_dryer_lower_target(struct ace2k_dryer *self, uint8_t target_c)
{
    if (ace2k_dryer_running_state(self->state) && target_c < self->target_c) {
        self->target_c = target_c;
    }
}

/* Whether the fault's cause is still present, read on the last inputs. */
static enum ace2k_dryer_refusal cause_refusal(const struct ace2k_dryer *self)
{
    const struct ace2k_dryer_inputs *in = &self->last;
    int32_t ceiling = ace2k_dryer_chamber_ceiling_mc(self);
    switch (self->fault) {
    case ACE2K_DRYER_FAULT_NTC:
        return (in->left_valid && in->right_valid) ? ACE2K_DRYER_OK : ACE2K_DRYER_SENSORS;
    case ACE2K_DRYER_FAULT_MAINS:
        return mains_refusal(self);
    case ACE2K_DRYER_FAULT_CHAMBER_STALE:
        return ace2k_dryer_chamber_fresh(in) ? ACE2K_DRYER_OK : ACE2K_DRYER_SENSORS;
    case ACE2K_DRYER_FAULT_CHAMBER_OVER:
        return in->chamber_mc < ceiling ? ACE2K_DRYER_OK : ACE2K_DRYER_NOT_COOL;
    default:
        return ACE2K_DRYER_OK;
    }
}

/* Heat kept its latch. What holds the latch names the refusal. ace2k_heat_clear_blocker checks,
 * in this order: the gate reading high, the cutout, a hard limit or an invalid NTC, then the mains
 * bucket. A gate reading high answers FAULTED. So the host is told the cause, not just to clear
 * again. */
static enum ace2k_dryer_refusal heat_clear_refusal(const struct ace2k_heat *heat)
{
    switch (ace2k_heat_clear_blocker(heat)) {
    case ACE2K_HEAT_CUTOUT:
        return ACE2K_DRYER_CUTOUT;
    case ACE2K_HEAT_NTC_INVALID:
        return ACE2K_DRYER_SENSORS;
    case ACE2K_HEAT_NTC_OVER:
        return ACE2K_DRYER_NOT_COOL;
    case ACE2K_HEAT_MAINS_ABSENT:
    case ACE2K_HEAT_MAINS_IMPLAUSIBLE:
        return ACE2K_DRYER_NO_MAINS;
    default:
        return ACE2K_DRYER_FAULTED;
    }
}

enum ace2k_dryer_refusal ace2k_dryer_clear(struct ace2k_dryer *self, uint32_t now_ms)
{
    if (self->shut) {
        /* The tick no longer runs, so nothing that a clear judges is current. */
        return ACE2K_DRYER_SHUTDOWN;
    }
    if ((self->log->flags & ACE2K_DRYER_LOG_CUTOUT) != 0U) {
        return ACE2K_DRYER_CUTOUT; /* only a power-on reset clears a tripped cutout */
    }
    if (self->state != ACE2K_DRYER_FAULT) {
        return ACE2K_DRYER_OK;
    }
    if (self->fault == ACE2K_DRYER_FAULT_CUTOUT || self->last.cutout || self->cutout_seen) {
        return ACE2K_DRYER_CUTOUT; /* only a power cycle clears a tripped cutout */
    }
    if (self->cool != ACE2K_DRYER_COOL_RELEASED) {
        return ACE2K_DRYER_NOT_COOL;
    }
    enum ace2k_dryer_refusal res = cause_refusal(self);
    if (res != ACE2K_DRYER_OK) {
        return res;
    }
    if (self->heat->state == ACE2K_HEAT_LATCHED && ace2k_heat_clear(self->heat) != 0) {
        return heat_clear_refusal(self->heat);
    }
    self->fault = ACE2K_DRYER_FAULT_NONE;
    ace2k_dryer_emit(self, ACE2K_DRYER_EV_CLEARED, 0U);
    ace2k_dryer_vent_room_ended(&self->room, now_ms);
    set_state(self, ACE2K_DRYER_IDLE, now_ms);
    return ACE2K_DRYER_OK;
}

/* Counts a dip that did not fault the cycle. The mains read off-band, but present, in STARTING or
 * HEATING, or already at the start. Then it read plausible again, with the cycle still running
 * and no fault. The dip is counted, and the notice raised, when it ends. An absent mains is never
 * a dip, because it faults at once. */
static void dip_ridden_out(struct ace2k_dryer *self)
{
    self->mains_dip_open = false;
    if (self->mains_dips != UINT16_MAX) {
        self->mains_dips++;
    }
    self->notices |= ACE2K_DRYER_NOTICE_MAINS_DIP;
}

static void track_mains_dip(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    if (!ace2k_dryer_running_state(self->state)) {
        /* The cycle left STARTING or HEATING with a dip still open. The dip counts as ridden out,
         * unless a fault ended the cycle. An end without a fault is the time up, or a stop. */
        if (self->mains_dip_open && self->state != ACE2K_DRYER_FAULT) {
            dip_ridden_out(self);
        }
        self->mains_dip_open = false;
        return;
    }
    if (!in->mains_present) {
        self->mains_dip_open = false;
        return;
    }
    if (!in->mains_plausible) {
        self->mains_dip_open = true;
        return;
    }
    if (self->mains_dip_open) {
        dip_ridden_out(self);
    }
}

/* In STARTING and HEATING, raises ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE once heat has judged at
 * least ACE2K_DRYER_PHASE_REJECTS_NOTICE edges out of phase since the start. */
static void track_phase_rejects(struct ace2k_dryer *self)
{
    if (!ace2k_dryer_running_state(self->state)) {
        return;
    }
    uint32_t rejects = self->heat->phase_rejects - self->phase_rejects_start;
    if (rejects >= ACE2K_DRYER_PHASE_REJECTS_NOTICE) {
        self->notices |= ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE;
    }
}

/* STARTING and HEATING: start_chamber_mc follows the lowest valid chamber reading since the start.
 * So chamber_over's ceiling only ever moves down. After a warm restart that began above the
 * target, the ceiling closes in as the chamber cools. */
static void track_start_chamber(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    if (!ace2k_dryer_chamber_fresh(in)) {
        return;
    }
    if (in->chamber_mc < self->start_chamber_mc) {
        self->start_chamber_mc = in->chamber_mc;
    }
}

/* An invalid side reads as its last valid value. The binding holds it too, but the core does not
 * rely on that. A 0 passed for one invalid tick would reach the controller as a fall. It would
 * reach the response window as a rise back from 0, 30 s later. The valid bits stay as they came,
 * so the sensor check still counts the invalid ticks. */
static void hold_ntcs(struct ace2k_dryer *self, struct ace2k_dryer_inputs *in)
{
    if (in->left_valid) {
        self->ntc_held_mc[0] = in->ntc_left_mc;
    } else {
        in->ntc_left_mc = self->ntc_held_mc[0];
    }
    if (in->right_valid) {
        self->ntc_held_mc[1] = in->ntc_right_mc;
    } else {
        in->ntc_right_mc = self->ntc_held_mc[1];
    }
}

void ace2k_dryer_tick(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *raw,
                      uint32_t now_ms)
{
    self->last = *raw;
    hold_ntcs(self, &self->last);
    self->have_inputs = true;
    const struct ace2k_dryer_inputs *in = &self->last;
    ace2k_dryer_vent_room_expire(&self->room, now_ms);
    if (in->cutout) {
        persist_cutout(self); /* sticky and on the page: only a power cycle clears it */
    }
    if (self->shut) {
        return;
    }
    uint8_t latch = latch_reason(self, in);
    if (latch != ACE2K_DRYER_FAULT_NONE) {
        enter_fault(self, latch, in, now_ms);
    }
    if (ace2k_dryer_running_state(self->state)) {
        ace2k_dryer_protect_spool(self, in);
        track_start_chamber(self, in); /* before the protections read the ceiling */
    }
    switch (self->state) {
    case ACE2K_DRYER_STARTING:
        tick_starting(self, in, now_ms);
        break;
    case ACE2K_DRYER_HEATING:
        tick_heating(self, in, now_ms);
        break;
    case ACE2K_DRYER_COOLDOWN:
        tick_cooldown(self, in, now_ms);
        break;
    case ACE2K_DRYER_FAULT:
        (void)tick_cooling(self, in, now_ms);
        break;
    default:
        break;
    }
    flaps_step(self, now_ms);  /* a sequence begun by the state in this tick is asked now */
    track_mains_dip(self, in); /* after the states: a dip that ended in a fault is not counted */
    track_phase_rejects(self);
    /* No mark here: the counters reach the page at the cycle's end. Only the ticks on which
     * heat may fire count as heating. A lease silenced by a debounced excursion, such as a mains
     * dip or an NTC glitch, heats nothing. */
    if (ace2k_heat_firing(self->heat)) {
        ace2k_dryer_log_account(self->log, self->heat->duty_pct, ACE2K_DRYER_TICK_MS);
    }
}

void ace2k_dryer_shutdown(struct ace2k_dryer *self)
{
    ace2k_heat_shutdown(self->heat); /* immediate: no pending half-cycle (heat.h) */
    /* No flap pulse from here on, because the tick returns first. Airflow's shutdown cut the
     * running pulse, and its flap is published as unknown. The flaps stay where they are until
     * the close at the next boot. */
    self->ctl.duty_cpct = 0U;
    self->shut = true;
    /* The dryer's claim goes. The fans follow airflow's fan rule, and heat's hold
     * while the heater may still conduct. */
    ace2k_airflow_release(self->airflow, ACE2K_AIRFLOW_OWNER_DRYER);
    /* A fault stays a fault, with its reason. The status never reads idle with a fault code. */
    if (self->state != ACE2K_DRYER_FAULT) {
        self->state = ACE2K_DRYER_IDLE;
    }
}

void ace2k_dryer_status(const struct ace2k_dryer *self, struct ace2k_dryer_status *out,
                        uint32_t now_ms)
{
    *out = (struct ace2k_dryer_status){ 0 };
    out->state = self->state;
    out->target_c = self->target_c;
    out->fault = self->fault;
    out->notices = self->notices;
    out->fans_read = ace2k_airflow_fans_read(self->airflow);
    out->flaps = ace2k_airflow_flaps_report(self->airflow);
    out->events_lost = self->events_lost > UINT16_MAX ? UINT16_MAX : (uint16_t)self->events_lost;
    out->oldest = ace2k_dryer_event_oldest(self);
    if (!ace2k_dryer_running_state(self->state)) {
        return;
    }
    out->duty_pct = ace2k_dryer_duty_pct(self->ctl.duty_cpct);
    out->drive_dc = (uint16_t)(ace2k_dryer_drive_mc(self) / ACE2K_DRYER_MC_PER_DC);
    if (!ace2k_time_after(now_ms, self->end_ms)) {
        out->remaining_min =
            (uint16_t)(ace2k_time_since(self->end_ms, now_ms) / ACE2K_DRYER_MS_PER_MIN);
    }
}
