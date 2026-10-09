/* What: the seam between the dryer's files — dryer.c (states), dryer_ctl.c (the controller),
 * dryer_log.c (the counters) and dryer_protect.c (the protections).
 * How: plain functions on struct ace2k_dryer; nothing here is part of the public interface.
 * Depends on: dryer.h. */
#ifndef ACE2K_DRYER_INTERNAL_H
#define ACE2K_DRYER_INTERNAL_H
#include "dryer/dryer.h"

#define ACE2K_DRYER_MS_PER_MIN 60000U

/* |v|; every caller passes a difference of two temperatures in millidegrees, far from INT32_MIN. */
static inline int32_t ace2k_dryer_abs_i32(int32_t v)
{
    if (v < 0) {
        return -v;
    }
    return v;
}

/* chamber_over's ceiling: the cycle's highest target, or the chamber's lowest valid reading since
 * the start when that was warmer, + ACE2K_DRYER_CHAMBER_OVER_MC.
 * It only ever moves down within a cycle.  The fault's clear (cause_refusal) reads the same. */
static inline int32_t ace2k_dryer_chamber_ceiling_mc(const struct ace2k_dryer *self)
{
    int32_t base = (int32_t)self->highest_target_c * ACE2K_DRYER_MC_PER_C;
    if (self->start_chamber_mc > base) {
        base = self->start_chamber_mc;
    }
    return base + ACE2K_DRYER_CHAMBER_OVER_MC;
}

/* dryer.c */
void ace2k_dryer_emit(struct ace2k_dryer *self, uint8_t kind, uint8_t arg);
int32_t ace2k_dryer_hotter_mc(const struct ace2k_dryer_inputs *in);

/* dryer_ctl.c */
void ace2k_dryer_ctl_reset(struct ace2k_dryer *self, uint32_t now_ms);
/* The drive target: target + offset + trim, capped at target + cap and at the absolute 73 °C. */
int32_t ace2k_dryer_drive_mc(const struct ace2k_dryer *self);
/* Every HEATING tick: the outer loop's bookkeeping, a PID step when due; the duty in %. */
uint8_t ace2k_dryer_ctl_step(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                             uint32_t now_ms);

/* dryer_log.c */
/* The log changed and must reach the page. */
static inline void ace2k_dryer_log_mark(struct ace2k_dryer *self)
{
    self->log_gen++;
}
void ace2k_dryer_log_cycle_open(struct ace2k_dryer *self);
void ace2k_dryer_log_cycle_close(struct ace2k_dryer *self, bool completed);
void ace2k_dryer_log_fault(struct ace2k_dryer *self, uint8_t reason,
                           const struct ace2k_dryer_inputs *in);

/* dryer_protect.c */
/* Resets the protections' state at the entry into HEATING (the insert mask is kept). */
void ace2k_dryer_protect_begin(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                               uint32_t now_ms);
/* HEATING only: the first protection that trips, or ACE2K_DRYER_FAULT_NONE. */
uint8_t ace2k_dryer_protect_check(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                                  uint32_t now_ms);
/* STARTING and HEATING: a strand newly at an insert sensor with no host lowers the target. */
void ace2k_dryer_protect_spool(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in);

#endif
