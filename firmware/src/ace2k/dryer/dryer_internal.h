/* What: the shared part of the dryer files: dryer.c (the states), dryer_events.c (the event
 * ring), dryer_ctl.c (the controller), dryer_log.c (the counters), dryer_protect.c (the
 * protections) and dryer_auto.c (the automatic modes).
 * How: plain functions on struct ace2k_dryer. Nothing here is part of the public interface.
 * Depends on: dryer.h. */
#ifndef ACE2K_DRYER_INTERNAL_H
#define ACE2K_DRYER_INTERNAL_H
#include "dryer/dryer.h"

#define ACE2K_DRYER_MS_PER_MIN 60000U
#define ACE2K_DRYER_MC_PER_DC  100 /* millidegrees in 0.1 °C, the unit of the log and the status */

/* The ceiling of chamber_over. Its base is the cycle's highest target, or the lowest valid
 * chamber reading since the start when that was warmer. The ceiling is the base plus
 * ACE2K_DRYER_CHAMBER_OVER_MC. The chamber part only ever moves down; a retarget to a higher
 * target raises the target part. The fault's clear (cause_refusal) reads the same value. */
static inline int32_t ace2k_dryer_chamber_ceiling_mc(const struct ace2k_dryer *self)
{
    int32_t base = (int32_t)self->highest_target_c * ACE2K_DRYER_MC_PER_C;
    if (self->start_chamber_mc > base) {
        base = self->start_chamber_mc;
    }
    return base + ACE2K_DRYER_CHAMBER_OVER_MC;
}

/* A cycle runs: STARTING or HEATING.  The dryer's states and the automatic modes ask the same. */
static inline bool ace2k_dryer_running_state(uint8_t state)
{
    return (bool)(state == ACE2K_DRYER_STARTING || state == ACE2K_DRYER_HEATING);
}

/* The range of a cycle's request: a target within the dryer's bounds and a time of at least one
 * minute, at most ACE2K_DRYER_MINUTES_MAX.  ACE2K_DRYER_RANGE outside it, else ACE2K_DRYER_OK.  The
 * starts and the automatic modes' profile take the same range. */
static inline enum ace2k_dryer_refusal ace2k_dryer_request_range_refusal(uint8_t target_c,
                                                                         uint16_t minutes)
{
    if (target_c < ACE2K_DRYER_TARGET_MIN_C || target_c > ACE2K_DRYER_TARGET_MAX_C) {
        return ACE2K_DRYER_RANGE;
    }
    if (minutes == 0U || minutes > ACE2K_DRYER_MINUTES_MAX) {
        return ACE2K_DRYER_RANGE;
    }
    return ACE2K_DRYER_OK;
}

/* The controller's duty in whole percent, rounded. The lease that heat fires, the status
 * report and the response window all use this same number. */
static inline uint8_t ace2k_dryer_duty_pct(uint16_t duty_cpct)
{
    return (uint8_t)((duty_cpct + (ACE2K_DRYER_CPCT_PER_PCT / 2U)) / ACE2K_DRYER_CPCT_PER_PCT);
}

/* True when the chamber reading counts: it is valid, and no older than the staleness bound that
 * env.h sets. Every reader of the chamber in the dryer asks this. So a stale reading counts as no
 * reading everywhere. */
static inline bool ace2k_dryer_chamber_fresh(const struct ace2k_dryer_inputs *in)
{
    return ace2k_env_chamber_fresh(in->chamber_valid, in->chamber_age_ms);
}

/* The lanes where a strand just arrived at the insert sensor, with no host to judge it: the
 * insert mask's new bits since *prev, or 0 while the link is up. *prev takes the new mask. The
 * spool rule (dryer_protect.c) and the automatic modes' profile rule (dryer_auto.c) share it. */
static inline uint8_t ace2k_dryer_insert_fresh(uint8_t *prev, const struct ace2k_dryer_inputs *in)
{
    uint8_t fresh = (uint8_t)(in->insert_mask & (uint8_t)~*prev);
    *prev = in->insert_mask;
    return in->link_ok ? 0U : fresh;
}

/* dryer_events.c: puts an event into the ring, from the tick. A full ring counts it as lost. */
void ace2k_dryer_emit(struct ace2k_dryer *self, uint8_t kind, uint8_t arg);

/* dryer.c */
int32_t ace2k_dryer_hotter_mc(const struct ace2k_dryer_inputs *in);

/* dryer_ctl.c */
void ace2k_dryer_ctl_reset(struct ace2k_dryer *self, uint32_t now_ms);
/* The drive target: target + offset + trim, capped at target + cap and at the absolute 73 °C. */
int32_t ace2k_dryer_drive_mc(const struct ace2k_dryer *self);
/* Runs every HEATING tick: the bookkeeping of the outer loop, and a PID step when one is due.
 * Returns the duty in %. */
uint8_t ace2k_dryer_ctl_step(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms);

/* dryer_log.c */
/* Marks that the log changed and must reach the page. */
static inline void ace2k_dryer_log_mark(struct ace2k_dryer *self)
{
    self->log_gen++;
}
void ace2k_dryer_log_cycle_open(struct ace2k_dryer *self);
void ace2k_dryer_log_cycle_close(struct ace2k_dryer *self, bool completed);
void ace2k_dryer_log_fault(struct ace2k_dryer *self, uint8_t reason,
                           const struct ace2k_dryer_inputs *in);

/* dryer_protect.c */
/* Resets the state of the protections at the entry into HEATING. The insert mask is kept. */
void ace2k_dryer_protect_begin(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                               uint32_t now_ms);
/* HEATING only. Returns the first protection that trips, or ACE2K_DRYER_FAULT_NONE. */
uint8_t ace2k_dryer_protect_check(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                                  uint32_t now_ms);
/* STARTING and HEATING: new filament at an insert sensor, with no host, lowers the target. */
void ace2k_dryer_protect_spool(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in);

#endif
