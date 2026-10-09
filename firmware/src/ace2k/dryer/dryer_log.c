/* The dryer's log: the counters, the
 * heating and full-power seconds, the ring of the last eight entries, the boot's interrupted
 * cycle and persisted cutout.  The pure helpers work on the log alone; the dryer-level functions
 * below them mark the log for the binding to store. */
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"

#define MS_PER_S     1000U
#define PCT_MS_PER_S 100000U /* 100 % × 1 000 ms: one full-power second */
#define MC_PER_DC    100

/* ---- the log alone ---- */

void ace2k_dryer_log_cycle_start(struct ace2k_dryer_log *log)
{
    log->cycles_started++;
    log->flags |= ACE2K_DRYER_LOG_CYCLE_OPEN;
}

void ace2k_dryer_log_cycle_end(struct ace2k_dryer_log *log, bool completed)
{
    log->flags &= ~ACE2K_DRYER_LOG_CYCLE_OPEN;
    if (completed) {
        log->cycles_completed++;
    }
}

void ace2k_dryer_log_account(struct ace2k_dryer_log *log, uint8_t duty_pct, uint32_t dt_ms)
{
    log->heat_ms_frac += dt_ms;
    while (log->heat_ms_frac >= MS_PER_S) {
        log->heat_ms_frac -= MS_PER_S;
        log->heat_s++;
    }
    log->full_power_ms_frac += (uint32_t)duty_pct * dt_ms;
    while (log->full_power_ms_frac >= PCT_MS_PER_S) {
        log->full_power_ms_frac -= PCT_MS_PER_S;
        log->full_power_s++;
    }
}

static int16_t to_dc(int32_t mc, bool valid)
{
    if (!valid) {
        return ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC;
    }
    int32_t dc = ace2k_clamp_i32(mc / MC_PER_DC, INT16_MIN + 1, INT16_MAX);
    return (int16_t)dc;
}

/* The three readings in 0.1 °C; no inputs at all is every reading unknown. */
static void entry_temperatures(struct ace2k_dryer_log_entry *e, const struct ace2k_dryer_inputs *in)
{
    if (in == NULL) {
        e->ntc_left_dc = ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC;
        e->ntc_right_dc = ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC;
        e->chamber_dc = ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC;
        return;
    }
    e->ntc_left_dc = to_dc(in->ntc_left_mc, in->left_valid);
    e->ntc_right_dc = to_dc(in->ntc_right_mc, in->right_valid);
    e->chamber_dc = to_dc(in->chamber_mc, in->chamber_valid);
}

void ace2k_dryer_log_record(struct ace2k_dryer_log *log, uint8_t kind, uint8_t reason,
                            const struct ace2k_dryer_inputs *in)
{
    for (uint32_t i = ACE2K_DRYER_LOG_ENTRIES - 1U; i > 0U; i--) {
        log->entry[i] = log->entry[i - 1U];
    }
    struct ace2k_dryer_log_entry *e = &log->entry[0];
    e->kind = kind;
    e->reason = reason;
    /* the open cycle's number; 0 when none is open (a fault in STARTING before the open mark,
     * in IDLE or in COOLDOWN), never the previous cycle's */
    e->cycle = (log->flags & ACE2K_DRYER_LOG_CYCLE_OPEN) != 0U ? (uint16_t)log->cycles_started : 0U;
    e->heat_s = log->heat_s;
    entry_temperatures(e, in);
    e->reserved = 0;
    if (kind == ACE2K_DRYER_EV_FAULT) {
        log->faults++;
        if (reason == ACE2K_DRYER_FAULT_CUTOUT) {
            log->flags |= ACE2K_DRYER_LOG_CUTOUT;
        }
    }
}

uint32_t ace2k_dryer_log_boot(struct ace2k_dryer_log *log, bool power_on_reset)
{
    uint32_t out = 0;
    log->heat_ms_frac = 0;
    log->full_power_ms_frac = 0;
    if ((log->flags & ACE2K_DRYER_LOG_CYCLE_OPEN) != 0U) {
        ace2k_dryer_log_record(log, ACE2K_DRYER_EV_INTERRUPTED, 0, NULL);
        log->flags &= ~ACE2K_DRYER_LOG_CYCLE_OPEN;
        out |= ACE2K_DRYER_LOG_BOOT_INTERRUPTED;
    }
    if ((log->flags & ACE2K_DRYER_LOG_CUTOUT) != 0U) {
        if (power_on_reset) {
            log->flags &= ~ACE2K_DRYER_LOG_CUTOUT;
            out |= ACE2K_DRYER_LOG_BOOT_CUTOUT_CLEARED;
        } else {
            out |= ACE2K_DRYER_LOG_BOOT_CUTOUT;
        }
    }
    return out;
}

/* ---- the dryer over the log ---- */

void ace2k_dryer_log_cycle_open(struct ace2k_dryer *self)
{
    ace2k_dryer_log_cycle_start(self->log);
    ace2k_dryer_log_mark(self);
}

void ace2k_dryer_log_cycle_close(struct ace2k_dryer *self, bool completed)
{
    ace2k_dryer_log_cycle_end(self->log, completed);
    ace2k_dryer_log_mark(self);
}

/* The entry (a fault counts in record; a cutout sets the persisted flag), the open cycle closed. */
void ace2k_dryer_log_fault(struct ace2k_dryer *self, uint8_t reason,
                           const struct ace2k_dryer_inputs *in)
{
    ace2k_dryer_log_record(self->log, ACE2K_DRYER_EV_FAULT, reason, in);
    ace2k_dryer_log_cycle_end(self->log, false);
    ace2k_dryer_log_mark(self);
}

bool ace2k_dryer_log_dirty(const struct ace2k_dryer *self)
{
    return self->log_gen != self->log_stored_gen;
}

uint32_t ace2k_dryer_log_generation(const struct ace2k_dryer *self)
{
    return self->log_gen;
}

void ace2k_dryer_log_stored(struct ace2k_dryer *self, uint32_t generation)
{
    self->log_stored_gen = generation; /* a store of an older generation leaves dirty set */
}
