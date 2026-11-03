/* The dryer's protections, on top of the absolute limits of the heat layer. While heating
 * they check: not heating, an impossible response, sides apart, the ceiling and the staleness of
 * the chamber, the NTCs and the mains, and the lid (the chamber must follow the heater). The file
 * also holds the spool rule. Every threshold is an initial value that the simulator set. dryer.h
 * marks those still to re-tune. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

#define PROTECT_PCT 100

void ace2k_dryer_protect_begin(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                               uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    uint8_t insert_prev = p->insert_prev;
    *p = (struct ace2k_dryer_protect){ 0 };
    p->insert_prev = insert_prev;
    p->start_mc[0] = in->ntc_left_mc;
    p->start_mc[1] = in->ntc_right_mc;
    p->heating_since_ms = now_ms;
    p->sample_next_ms = now_ms;
    p->win_next_ms = now_ms;
    /* The lid keeps its own reference. start_chamber_mc is the lowest chamber since the start,
     * the floor of the ceiling. The lid needs the chamber at the entry into HEATING. */
    p->lid_ref_mc = in->chamber_mc;
    p->lid_ref_set = ace2k_dryer_chamber_fresh(in);
    p->lid_next_ms = now_ms;
}

/* The sensor checks:
 * - An NTC invalid for ACE2K_DRYER_NTC_BAD_SAMPLES samples in a row, 100 ms apart, faults.
 * - An absent mains faults at once.
 * - An implausible mains faults once heat's bucket holds it (heat.h). Both layers use that
 *   one persistence, with a lease or without. A lone off-band mains window is no fault.
 * The NTC debounce applies only to the stretches of HEATING with no lease held: a duty of 0, or
 * a refused renewal still waiting. While heat holds a lease, its own leaky bucket judges an
 * invalid NTC and latches first (heat.h). The dryer then faults on that latch. */
static uint8_t check_sensors(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    if (!in->mains_present || ace2k_heat_mains_implausible_held(self->heat)) {
        return ACE2K_DRYER_FAULT_MAINS;
    }
    if (!ace2k_time_after(now_ms, p->sample_next_ms)) {
        return ACE2K_DRYER_FAULT_NONE;
    }
    p->sample_next_ms = now_ms + ACE2K_DRYER_NTC_SAMPLE_MS;
    if (in->left_valid && in->right_valid) {
        p->ntc_bad = 0U;
        return ACE2K_DRYER_FAULT_NONE;
    }
    p->ntc_bad++;
    if (p->ntc_bad >= ACE2K_DRYER_NTC_BAD_SAMPLES) {
        return ACE2K_DRYER_FAULT_NTC;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

/* True when a condition has held without a break for hold_ms. */
static bool held(bool now_true, bool *was, uint32_t *since_ms, uint32_t now_ms, uint32_t hold_ms)
{
    if (!now_true) {
        *was = false;
        return false;
    }
    if (!*was) {
        *was = true;
        *since_ms = now_ms;
        return false;
    }
    return ace2k_time_since(now_ms, *since_ms) >= hold_ms;
}

static uint8_t check_chamber(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    if (!ace2k_dryer_chamber_fresh(in)) {
        return ACE2K_DRYER_FAULT_CHAMBER_STALE;
    }
    if (in->chamber_mc > ACE2K_DRYER_CHAMBER_ABS_MAX_MC) {
        return ACE2K_DRYER_FAULT_CHAMBER_OVER;
    }
    /* The ceiling uses the cycle's highest target. So a lowering by the spool rule never
     * makes the chamber "over". */
    int32_t ceiling = ace2k_dryer_chamber_ceiling_mc(self);
    if (held(in->chamber_mc > ceiling, &p->chamber_over, &p->chamber_over_since_ms, now_ms,
             ACE2K_DRYER_CHAMBER_OVER_MS)) {
        return ACE2K_DRYER_FAULT_CHAMBER_OVER;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

static uint8_t check_sides(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                           uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    bool apart =
        ace2k_abs_i32(in->ntc_left_mc - in->ntc_right_mc) > (uint32_t)ACE2K_DRYER_SIDES_APART_MC;
    if (held(apart, &p->sides_apart, &p->sides_since_ms, now_ms, ACE2K_DRYER_SIDES_MS)) {
        return ACE2K_DRYER_FAULT_SIDES;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

/* Returns the duty that drove the rise, in %·s. The window's samples count in full. They
 * are the seconds up to the newest − 1, each applied over the second that followed it; the newest
 * has not acted yet. The samples of the lag before them count at ACE2K_DRYER_RISE_LAG_WEIGHT_PCT,
 * because the NTC answers the gate 10–15 s late. Slots older than HEATING hold 0, because
 * STARTING fired nothing. */
static int32_t driving_duty_sum(const struct ace2k_dryer_protect *p, uint8_t newest)
{
    int32_t window = 0;
    int32_t lag = 0;
    for (uint32_t back = 1U; back < ACE2K_DRYER_RISE_SLOTS; back++) {
        uint32_t i = (newest + ACE2K_DRYER_RISE_SLOTS - back) % ACE2K_DRYER_RISE_SLOTS;
        if (back <= ACE2K_DRYER_RISE_WINDOW_S) {
            window += p->win_duty[i];
        } else {
            lag += p->win_duty[i];
        }
    }
    return window + ((lag * ACE2K_DRYER_RISE_LAG_WEIGHT_PCT) / PROTECT_PCT);
}

/* Samples the hotter NTC and the applied duty once a second, in a ring of LAG + WINDOW + 1 slots.
 * The rise over the last WINDOW seconds is compared with the ceiling for the duty that drove it.
 * The applied duty is 0 while heat fires nothing: no lease yet, a refused renewal, or a mains dip
 * or an NTC excursion that silences the lease. So the ceiling counts only the heat that went in. */
static uint8_t check_response(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                              uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    if (!ace2k_time_after(now_ms, p->win_next_ms)) {
        return ACE2K_DRYER_FAULT_NONE;
    }
    p->win_next_ms = now_ms + ACE2K_DRYER_RISE_SAMPLE_MS;
    p->win_ntc_mc[p->win_idx] = ace2k_dryer_hotter_mc(in);
    p->win_duty[p->win_idx] =
        ace2k_heat_firing(self->heat) ? ace2k_dryer_duty_pct(self->ctl.duty_cpct) : 0U;
    p->win_idx = (uint8_t)((p->win_idx + 1U) % ACE2K_DRYER_RISE_SLOTS);
    if (p->win_n < ACE2K_DRYER_RISE_SLOTS) {
        p->win_n++;
    }
    if (p->win_n <= ACE2K_DRYER_RISE_WINDOW_S) {
        return ACE2K_DRYER_FAULT_NONE; /* the window is not full yet */
    }
    uint8_t newest = (uint8_t)((p->win_idx + ACE2K_DRYER_RISE_SLOTS - 1U) % ACE2K_DRYER_RISE_SLOTS);
    uint8_t first = (uint8_t)((newest + ACE2K_DRYER_RISE_SLOTS - ACE2K_DRYER_RISE_WINDOW_S) %
                              ACE2K_DRYER_RISE_SLOTS);
    int32_t limit = ((ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT * driving_duty_sum(p, newest) *
                      ACE2K_DRYER_RISE_MARGIN_PCT) /
                     PROTECT_PCT) +
                    ACE2K_DRYER_RISE_FLOOR_MC;
    if (p->win_ntc_mc[newest] - p->win_ntc_mc[first] > limit) {
        return ACE2K_DRYER_FAULT_RESPONSE;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

/* A side counts as heating once it rose 5 °C above its start, or came within 2 °C of the drive
 * target. Both sides must do so within 300 s of the entry into HEATING. */
static void rise_mark(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    struct ace2k_dryer_protect *p = &self->prot;
    int32_t drive = ace2k_dryer_drive_mc(self);
    const int32_t ntc[2] = { in->ntc_left_mc, in->ntc_right_mc };
    for (uint8_t s = 0; s < 2U; s++) {
        if (ntc[s] - p->start_mc[s] >= ACE2K_DRYER_NOT_HEATING_RISE_MC ||
            drive - ntc[s] <= ACE2K_DRYER_NOT_HEATING_NEAR_MC) {
            p->rise_met[s] = true;
        }
    }
}

static uint8_t check_heating(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    if (p->rise_checked) {
        return ACE2K_DRYER_FAULT_NONE;
    }
    rise_mark(self, in);
    if (ace2k_time_since(now_ms, p->heating_since_ms) < ACE2K_DRYER_NOT_HEATING_MS) {
        return ACE2K_DRYER_FAULT_NONE;
    }
    p->rise_checked = true;
    if (!p->rise_met[0] || !p->rise_met[1]) {
        return ACE2K_DRYER_FAULT_NOT_HEATING;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

/* The lid: the chamber must follow the heater (dryer.h, "The lid"). The reference is the chamber
 * at the entry into HEATING. A stale chamber faults CHAMBER_STALE in check_chamber, before this
 * check runs. A sample is skipped, and the hold reset, when the heater is less than
 * LID_MIN_SPAN above the reference, or the current target less than LID_MIN_ROOM above it. */
static uint8_t check_lid(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                         uint32_t now_ms)
{
    struct ace2k_dryer_protect *p = &self->prot;
    if (!ace2k_dryer_chamber_fresh(in)) {
        return ACE2K_DRYER_FAULT_NONE; /* check_chamber judges a stale chamber */
    }
    if (!p->lid_ref_set) {
        /* A guard only: a chamber stale at the entry faults CHAMBER_STALE on the next tick,
         * unless a fresh reading arrives on that very tick. */
        p->lid_ref_mc = in->chamber_mc;
        p->lid_ref_set = true;
    }
    if (ace2k_time_since(now_ms, p->heating_since_ms) < ACE2K_DRYER_LID_AFTER_MS ||
        !ace2k_time_after(now_ms, p->lid_next_ms)) {
        return ACE2K_DRYER_FAULT_NONE;
    }
    p->lid_next_ms = now_ms + ACE2K_DRYER_LID_SAMPLE_MS;
    int32_t span = ((in->ntc_left_mc + in->ntc_right_mc) / 2) - p->lid_ref_mc;
    int32_t room = ((int32_t)self->target_c * ACE2K_DRYER_MC_PER_C) - p->lid_ref_mc;
    int32_t rise = in->chamber_mc - p->lid_ref_mc;
    bool open = false;
    if (span >= ACE2K_DRYER_LID_MIN_SPAN_MC && room >= ACE2K_DRYER_LID_MIN_ROOM_MC) {
        open = rise * PROTECT_PCT < ACE2K_DRYER_LID_RATIO_PCT * span;
    }
    if (held(open, &p->lid_open, &p->lid_open_since_ms, now_ms, ACE2K_DRYER_LID_HOLD_MS)) {
        return ACE2K_DRYER_FAULT_LID;
    }
    return ACE2K_DRYER_FAULT_NONE;
}

uint8_t ace2k_dryer_protect_check(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                                  uint32_t now_ms)
{
    uint8_t f = check_sensors(self, in, now_ms);
    if (f == ACE2K_DRYER_FAULT_NONE) {
        f = check_chamber(self, in, now_ms);
    }
    if (f == ACE2K_DRYER_FAULT_NONE) {
        f = check_sides(self, in, now_ms);
    }
    if (f == ACE2K_DRYER_FAULT_NONE) {
        f = check_response(self, in, now_ms);
    }
    if (f == ACE2K_DRYER_FAULT_NONE) {
        f = check_heating(self, in, now_ms);
    }
    if (f == ACE2K_DRYER_FAULT_NONE) {
        f = check_lid(self, in, now_ms);
    }
    return f;
}

void ace2k_dryer_protect_spool(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    struct ace2k_dryer_protect *p = &self->prot;
    uint32_t fresh = ace2k_dryer_insert_fresh(&p->insert_prev, in);
    if (fresh == 0U || self->target_c <= ACE2K_DRYER_SPOOL_SAFE_C) {
        return;
    }
    uint8_t lane = 1U;
    while ((fresh & 1U) == 0U) {
        fresh >>= 1U;
        lane++;
    }
    self->target_c = ACE2K_DRYER_SPOOL_SAFE_C;
    self->notices |= ACE2K_DRYER_NOTICE_LOWERED;
    ace2k_dryer_emit(self, ACE2K_DRYER_EV_LOWERED, lane);
}
