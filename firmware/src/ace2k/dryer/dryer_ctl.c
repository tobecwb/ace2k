/* The cascade controller: an inner PID on the hotter outlet
 * NTC toward the drive target, every ACE2K_DRYER_PID_PERIOD_MS, in 0.01 % of duty; an outer
 * loop that trims the drive target from the chamber, at most once per outer period and only
 * with the chamber steady.  Fixed point throughout: millidegrees in, cpct out. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

void ace2k_dryer_ctl_reset(struct ace2k_dryer *self, uint32_t now_ms)
{
    struct ace2k_dryer_ctl *c = &self->ctl;
    *c = (struct ace2k_dryer_ctl){ 0 };
    c->step_next_ms = now_ms;
    c->renew_next_ms = now_ms;
    c->outer_next_ms = now_ms + ACE2K_DRYER_OUTER_PERIOD_MS;
    c->ch_min_mc = INT32_MAX;
    c->ch_max_mc = INT32_MIN;
}

int32_t ace2k_dryer_drive_mc(const struct ace2k_dryer *self)
{
    int32_t base = (int32_t)self->target_c * ACE2K_DRYER_MC_PER_C;
    int32_t drive = base + ACE2K_DRYER_DRIVE_OFFSET_MC + self->ctl.trim_mc;
    int32_t cap = base + ACE2K_DRYER_DRIVE_CAP_MC;
    if (cap > ACE2K_DRYER_DRIVE_ABS_MAX_MC) {
        cap = ACE2K_DRYER_DRIVE_ABS_MAX_MC;
    }
    if (drive > cap) {
        return cap;
    }
    return drive;
}

/* Integrate only when the output is not pinned in the error's direction (anti-windup). */
static bool integrate_allowed(int32_t out_cpct, int32_t err_mc)
{
    if (out_cpct >= ACE2K_DRYER_DUTY_MAX_CPCT && err_mc > 0) {
        return false;
    }
    if (out_cpct <= 0 && err_mc < 0) {
        return false;
    }
    return true;
}

static void pid_step(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in)
{
    struct ace2k_dryer_ctl *c = &self->ctl;
    int32_t err = ace2k_dryer_drive_mc(self) - ace2k_dryer_hotter_mc(in);
    int32_t derr = c->primed ? err - c->prev_err_mc : 0;
    c->prev_err_mc = err;
    c->primed = true;
    int32_t p = err * ACE2K_DRYER_KP_CPCT_PER_C / ACE2K_DRYER_MC_PER_C;
    int32_t d = derr * ACE2K_DRYER_KD_CPCT_PER_C / ACE2K_DRYER_MC_PER_C;
    int32_t out = p + (c->integ_m / ACE2K_DRYER_MC_PER_C) + d;
    if (integrate_allowed(out, err)) {
        c->integ_m = ace2k_clamp_i32(c->integ_m + (err * ACE2K_DRYER_KI_CPCT_PER_C), 0,
                                     ACE2K_DRYER_DUTY_MAX_CPCT * ACE2K_DRYER_MC_PER_C);
        out = p + (c->integ_m / ACE2K_DRYER_MC_PER_C) + d;
    }
    c->duty_cpct = (uint16_t)ace2k_clamp_i32(out, 0, ACE2K_DRYER_DUTY_MAX_CPCT);
}

/* The outer loop: the chamber's range over the period; a steady period moves the trim by half
 * the chamber's error, within ±ACE2K_DRYER_TRIM_MAX_MC.  A period that closes on a tick with no
 * valid chamber reading takes no step: an invalid reading is not a temperature. */
static void outer_step(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                       uint32_t now_ms)
{
    struct ace2k_dryer_ctl *c = &self->ctl;
    if (in->chamber_valid) {
        if (in->chamber_mc < c->ch_min_mc) {
            c->ch_min_mc = in->chamber_mc;
        }
        if (in->chamber_mc > c->ch_max_mc) {
            c->ch_max_mc = in->chamber_mc;
        }
    }
    if (!ace2k_time_after(now_ms, c->outer_next_ms)) {
        return;
    }
    c->outer_next_ms = now_ms + ACE2K_DRYER_OUTER_PERIOD_MS;
    bool seen = c->ch_max_mc >= c->ch_min_mc;
    if (in->chamber_valid && seen && c->ch_max_mc - c->ch_min_mc < ACE2K_DRYER_CHAMBER_STEADY_MC) {
        int32_t target_mc = (int32_t)self->target_c * ACE2K_DRYER_MC_PER_C;
        int32_t step = (target_mc - in->chamber_mc) / ACE2K_DRYER_TRIM_GAIN_DIV;
        c->trim_mc =
            ace2k_clamp_i32(c->trim_mc + step, -ACE2K_DRYER_TRIM_MAX_MC, ACE2K_DRYER_TRIM_MAX_MC);
    }
    c->ch_min_mc = INT32_MAX;
    c->ch_max_mc = INT32_MIN;
}

uint8_t ace2k_dryer_ctl_step(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms)
{
    struct ace2k_dryer_ctl *c = &self->ctl;
    outer_step(self, in, now_ms);
    if (ace2k_time_after(now_ms, c->step_next_ms)) {
        c->step_next_ms = now_ms + ACE2K_DRYER_PID_PERIOD_MS;
        pid_step(self, in);
    }
    return (uint8_t)((c->duty_cpct + (ACE2K_DRYER_CPCT_PER_PCT / 2U)) / ACE2K_DRYER_CPCT_PER_PCT);
}
