/* What: the autonomous dryer. It holds the drying policy on top of the heat layer and the airflow
 * outputs: the states, the cascade controller, the vent (dryer_vent.h), the cool-down, the
 * protections, the spool rule and the counters of its log.
 * How: the binding, or the test rig, owns one struct ace2k_dryer and one struct ace2k_dryer_log.
 * It calls ace2k_dryer_tick() every 10 ms with one struct ace2k_dryer_inputs, after
 * ace2k_airflow_tick() and ace2k_heat_tick(). Start, stop and clear run from command context,
 * inside the binding's irq bracket. Events go through a ring with one producer (the tick)
 * and one consumer (the task), as in sensors.h. Each event stays in the ring until the host
 * acknowledges it. A full ring drops the new event and counts it in the status. A change of the
 * log bumps a generation. The binding then stores the log from a task, with the gate idle,
 * because a page erase stalls the core. Temperatures are in millidegrees. Inside the controller
 * the duty is in 0.01 %; at the heat layer it is in whole percent. No floats.
 * Depends on: <stdbool.h>, <stdint.h>, airflow.h, dryer_log.h, dryer_vent.h, env.h, heat.h,
 * util.h. */
#ifndef ACE2K_DRYER_H
#define ACE2K_DRYER_H
#include <stdbool.h>
#include <stdint.h>

#include "heat/airflow.h"
#include "heat/heat.h"
#include "dryer/dryer_vent.h"
#include "dryer/dryer_log.h"
#include "env/env.h"
#include "core/util.h"

/* The request. */
#define ACE2K_DRYER_TARGET_MIN_C     15U /* the range of the factory firmware */
#define ACE2K_DRYER_TARGET_MAX_C     65U
#define ACE2K_DRYER_MINUTES_MAX      1440U /* 24 h */
#define ACE2K_DRYER_DRIVE_OFFSET_MC  7000  /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DRIVE_CAP_MC     8000  /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DRIVE_ABS_MAX_MC 73000
#define ACE2K_DRYER_SPOOL_SAFE_C     45U

/* The controller, in HEATING. Kp is 1.0 %/°C, Ki is 0.01 %/°C per period and Kd is 5.0 %/°C per
 * period. The values are written in 0.01 % of duty (cpct) per °C. */
#define ACE2K_DRYER_PID_PERIOD_MS  2000U /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KP_CPCT_PER_C  100   /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KI_CPCT_PER_C  1     /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KD_CPCT_PER_C  500   /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DUTY_MAX_CPCT  9000  /* 90 %, the heat layer's own ceiling */
#define ACE2K_DRYER_CPCT_PER_PCT   100U
#define ACE2K_DRYER_LEASE_RENEW_MS 500U /* a lease of 2 000 ms is renewed four times over */
/* The dryer wants heat (duty > 0), but every lease is refused, and it has waited this long. That
 * is a fault. The wait's usual cause is heat's mains bucket, held above its restart
 * level by a mains off-band about one window in three (-ACE2K_EAGAIN). Otherwise the cause is
 * whatever refused the lease. A normal wait is much shorter. The bucket drains from just below
 * its latch to the restart level in 2.0 s (203 ticks). One more off-band window during that wait
 * adds 1 s. A lasting off-band fills the bucket in 1.5 s and faults as a latch. So 10 s is three
 * times the longest normal wait. A duty of 0 means no lease is wanted, and it resets the wait. */
#define ACE2K_DRYER_NO_LEASE_MAX_MS   10000U  /* initial value */
#define ACE2K_DRYER_TRIM_MAX_MC       2000    /* initial value; re-tune on the bench */
#define ACE2K_DRYER_TRIM_GAIN_DIV     2       /* trim += (target − chamber) / 2 per step */
#define ACE2K_DRYER_OUTER_PERIOD_MS   600000U /* initial value; re-tune on the bench */
#define ACE2K_DRYER_CHAMBER_STEADY_MC 800     /* initial value; re-tune on the bench */

/* STARTING. */
#define ACE2K_DRYER_START_TIMEOUT_MS 3000U         /* initial value */
#define ACE2K_DRYER_FANS_BOTH ACE2K_HEAT_FANS_BOTH /* ace2k_airflow_fans_read(): left | right */

/* COOLDOWN. */
#define ACE2K_DRYER_COOL_MC     ACE2K_AIRFLOW_FAN_ON_MC /* a fan is never off above this */
#define ACE2K_DRYER_COOL_MIN_MS 30000U /* also the window of the falling rate; initial value */
/* 1 °C/min over 30 s. Steady means it fell less than this, or rose by up to
 * ACE2K_DRYER_COOL_RISE_MC. */
#define ACE2K_DRYER_COOL_FALL_MC 500
/* A rise up to this value over the window is the NTC's noise, and it reads as steady. A larger
 * rise is the overshoot after a stop. Initial value; re-tune on the bench. */
#define ACE2K_DRYER_COOL_RISE_MC   200
#define ACE2K_DRYER_HOT_AMBIENT_MS 600000U /* 10 min; initial value */

/* The protections. dryer_protect.c implements them. */
#define ACE2K_DRYER_NOT_HEATING_MS      300000U /* 5 min, the deadline of the factory firmware */
#define ACE2K_DRYER_NOT_HEATING_RISE_MC 5000    /* initial value; re-tune on the bench */
#define ACE2K_DRYER_NOT_HEATING_NEAR_MC 2000    /* initial value; re-tune on the bench */
#define ACE2K_DRYER_RISE_WINDOW_S       30U     /* initial value; re-tune on the bench */
/* The outlet NTC answers the gate 10 to 15 s late (docs/hardware.md "Heater"). So a rise over the
 * window was also driven by the duty of the lag before it. That is the overshoot after a stop.
 * The duty of the lag counts at ACE2K_DRYER_RISE_LAG_WEIGHT_PCT. The measured overshoot is
 * +3.8 °C after 50 %. That is 15 % of the full ceiling over 15 s at 50 % (25.5 °C). A weight of
 * 25 % covers it with margin. It still catches a blocked duct within a minute on the simulator,
 * although a blocked duct also looks like a rise after the duty fell. */
#define ACE2K_DRYER_RISE_LAG_S            15U
#define ACE2K_DRYER_RISE_LAG_WEIGHT_PCT   25 /* initial value; re-tune on the bench */
#define ACE2K_DRYER_RISE_SLOTS            (ACE2K_DRYER_RISE_WINDOW_S + ACE2K_DRYER_RISE_LAG_S + 1U)
#define ACE2K_DRYER_RISE_SAMPLE_MS        1000U
#define ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT 35 /* from tools/thermal_fit.py; re-tune on the bench */
#define ACE2K_DRYER_RISE_MARGIN_PCT                                                                \
    100 /* the ceiling above already carries the fit's 1.5x; re-tune on the bench */
#define ACE2K_DRYER_RISE_FLOOR_MC  2000   /* initial value; re-tune on the bench */
#define ACE2K_DRYER_SIDES_APART_MC 15000  /* initial value */
#define ACE2K_DRYER_SIDES_MS       60000U /* initial value; re-tune on the bench */
/* The chamber ceiling. The chamber faults at 10 °C over its base for 30 s (dryer_internal.h), or
 * at once at 80 °C (docs/rules.md). Initial values. The AHT20 is rated to 85 °C. */
#define ACE2K_DRYER_CHAMBER_OVER_MC    10000
#define ACE2K_DRYER_CHAMBER_OVER_MS    30000U
#define ACE2K_DRYER_CHAMBER_ABS_MAX_MC 80000
#define ACE2K_DRYER_CHAMBER_STALE_MS   ACE2K_ENV_CHAMBER_STALE_MS /* env.h sets the one bound */
#define ACE2K_DRYER_NTC_SAMPLE_MS      100U
#define ACE2K_DRYER_NTC_BAD_SAMPLES    5U /* five samples, as the factory firmware does */

/* The lid. With the lid open the fans blow room air over the chamber sensor, so the chamber
 * does not follow the heater (measured 2026-11-01, four runs at 45 and 55 °C). The reference is
 * the chamber at the entry into HEATING; a stale chamber faults CHAMBER_STALE before the lid is
 * judged. The ratio of the chamber's rise to the heater's rise, both from that reference, is
 * judged once a second from LID_AFTER on; below LID_RATIO for LID_HOLD it faults. The lowest
 * closed case measured on the unit was a warm closed box at 0.22 over 45 min; the first runs of
 * 2026-11-01 gave 0.27 at 8 min.
 * Re-tune on the bench. */
#define ACE2K_DRYER_LID_AFTER_MS    480000U /* a warm closed box separates by the 8th minute */
#define ACE2K_DRYER_LID_HOLD_MS     120000U /* "open" held without a break */
#define ACE2K_DRYER_LID_SAMPLE_MS   1000U   /* the lid is judged once a second */
#define ACE2K_DRYER_LID_RATIO_PCT   10      /* below this the chamber does not follow */
#define ACE2K_DRYER_LID_MIN_SPAN_MC 5000    /* a smaller heater rise is noise: not judged */
/* A chamber already within 5 °C of the current target has no room to rise: not judged. */
#define ACE2K_DRYER_LID_MIN_ROOM_MC 5000

/* The flaps. Every sequence pulses both flaps the same way, and asks for the bottom one first
 * (flaps_begin in dryer.c). A sequence is over once the two requests of the dryer have ended in
 * airflow, or this long after it began. Airflow itself publishes as unknown whatever the sequence
 * left unfinished. Two pulses, plus one already running when asked, take at most
 * 3 × ACE2K_FLAP_PULSE_MAX_MS. */
#define ACE2K_DRYER_FLAPS_TIMEOUT_MS 2000U

enum ace2k_dryer_flaps_result {
    ACE2K_DRYER_FLAPS_OVER = 0,  /* both its pulses ran to their end, or none was begun */
    ACE2K_DRYER_FLAPS_RUNNING,   /* asking, or waiting for its pulses to end */
    ACE2K_DRYER_FLAPS_TIMED_OUT, /* ACE2K_DRYER_FLAPS_TIMEOUT_MS passed first */
};

/* The one flap sequence in flight. A new sequence replaces it. Airflow keeps a request of the
 * dryer waiting on a flap until a later request of the dryer replaces it. */
struct ace2k_dryer_flaps {
    uint8_t next;   /* the flap to ask next (enum ace2k_flap); ACE2K_FLAP_COUNT once both were */
    uint8_t result; /* enum ace2k_dryer_flaps_result */
    bool open;
    uint32_t since_ms;
};

/* The course of the cool-down, and of the fault. */
enum ace2k_dryer_cool {
    ACE2K_DRYER_COOL_RELEASED = 0, /* flaps closed, fans handed to airflow's fan rule */
    ACE2K_DRYER_COOL_OPENING,      /* the flaps opening, after heating */
    ACE2K_DRYER_COOL_WAITING,      /* the cool rule, or 10 min in a hot ambient */
    ACE2K_DRYER_COOL_CLOSING,      /* the flaps closing */
};

enum ace2k_dryer_state {
    ACE2K_DRYER_IDLE = 0,
    ACE2K_DRYER_STARTING = 1,
    ACE2K_DRYER_HEATING = 2,
    ACE2K_DRYER_COOLDOWN = 3,
    ACE2K_DRYER_FAULT = 4,
};

enum ace2k_dryer_fault {
    ACE2K_DRYER_FAULT_NONE = 0,
    ACE2K_DRYER_FAULT_NOT_HEATING = 1,
    ACE2K_DRYER_FAULT_RESPONSE = 2,
    ACE2K_DRYER_FAULT_SIDES = 3,
    ACE2K_DRYER_FAULT_CHAMBER_OVER = 4,
    ACE2K_DRYER_FAULT_CHAMBER_STALE = 5,
    ACE2K_DRYER_FAULT_NTC = 6,
    ACE2K_DRYER_FAULT_MAINS = 7,
    ACE2K_DRYER_FAULT_CUTOUT = 8,
    ACE2K_DRYER_FAULT_HEAT = 9, /* heat latched for another reason */
    ACE2K_DRYER_FAULT_FANS = 10,
    ACE2K_DRYER_FAULT_FLAPS = 11, /* STARTING: the closing flaps not at rest within the timeout */
    ACE2K_DRYER_FAULT_LID = 12,   /* HEATING: the chamber does not follow the heater */
};

enum ace2k_dryer_refusal {
    ACE2K_DRYER_OK = 0,
    ACE2K_DRYER_BUSY = 1,
    ACE2K_DRYER_RANGE = 2,
    ACE2K_DRYER_FAULTED = 3,
    ACE2K_DRYER_SENSORS = 4,
    ACE2K_DRYER_NO_MAINS = 5,
    ACE2K_DRYER_CUTOUT = 6,
    ACE2K_DRYER_NOT_COOL = 7,
    ACE2K_DRYER_SHUTDOWN = 8,  /* start, clear: a Klipper shutdown happened; only a reset helps */
    ACE2K_DRYER_MEASURING = 9, /* mains back, its first window not measured yet; transient */
};

enum ace2k_dryer_event_kind {
    ACE2K_DRYER_EV_DONE = 0,
    ACE2K_DRYER_EV_STOPPED = 1,
    ACE2K_DRYER_EV_FAULT = 2,
    ACE2K_DRYER_EV_LOWERED = 3,
    ACE2K_DRYER_EV_INTERRUPTED = 4,
    ACE2K_DRYER_EV_HOT_AMBIENT = 5,
    ACE2K_DRYER_EV_VENTED = 6,
    ACE2K_DRYER_EV_CLEARED = 7,
    /* The automatic modes (dryer_auto.h). */
    ACE2K_DRYER_EV_AUTO_STARTED = 8,  /* arg: the cycle's target in °C */
    ACE2K_DRYER_EV_AUTO_DRY = 9,      /* arg: the cold box's RH in whole % */
    ACE2K_DRYER_EV_AUTO_FAILED = 10,  /* arg: the failed cycles in a row */
    ACE2K_DRYER_EV_AUTO_STOPPED = 11, /* arg: the failed cycles in a row */
    ACE2K_DRYER_EV_AUTO_OFF = 12,     /* arg: 0, a manual stop turned the humidity mode off */
    ACE2K_DRYER_EV_STOPPED_LINK = 13, /* arg: 0, a bound cycle ended with the host gone */
};

#define ACE2K_DRYER_NOTICE_HOT_AMBIENT 0x1U
#define ACE2K_DRYER_NOTICE_LOWERED     0x2U
/* Set by the binding, not by the core: the log's last store failed on the flash
 * (dryer_cmds.c). */
#define ACE2K_DRYER_NOTICE_LOG_UNSTORED 0x4U
/* A mains dip happened in this cycle and did not fault it. The mains read off-band in STARTING or
 * HEATING, or already at the start. The dip is counted when the mains reads plausible again while
 * the cycle goes on. It is also counted when the cycle ends without a fault while the dip is still
 * open. So this notice shows a dip that heat's bucket tolerated (heat.h). It does not say
 * that the dip ended. The count is mains_dips. */
#define ACE2K_DRYER_NOTICE_MAINS_DIP 0x8U
/* In this cycle, heat judged at least ACE2K_DRYER_PHASE_REJECTS_NOTICE zero-cross edges out of
 * phase (phase_rejects in heat.h). The cause is noise at the input, or a half-cycle asymmetry past
 * heat's tolerance. The heater skips those cycles, and this notice makes them visible. */
#define ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE 0x10U
/* The chamber started above the target. Nothing heats until the chamber falls below it, because
 * the drive target sits below the outlets. The fans and the flaps follow the cycle. This is judged
 * once, at the start. */
#define ACE2K_DRYER_NOTICE_AMBIENT_ABOVE 0x20U
#define ACE2K_DRYER_PHASE_REJECTS_NOTICE 10U /* a missed edge costs two; ten is a pattern */
#define ACE2K_DRYER_EVENT_RING           8U  /* one slot stays free, so it holds seven events */

/* What one tick sees of the world. The binding, or the test rig, fills it. */
struct ace2k_dryer_inputs {
    /* The NTCs in milli-°C, each with its valid bit. When a side is invalid, ace2k_dryer_tick()
     * replaces its value with the last valid value of that side, or 0 before the first one. It
     * does so whatever the binding passed. The reason: the controller and the protections read
     * the values without the valid bits, and a 0 read for one tick would look like a fall and
     * then a rise. */
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    int32_t chamber_mc;
    bool chamber_valid;
    uint32_t chamber_age_ms;
    uint16_t chamber_rh_pct10; /* the RH of the chamber sensor, in 0.1 % */
    bool rh_valid;
    bool mains_present, mains_plausible;
    bool cutout;
    bool link_ok;
    uint8_t insert_mask; /* bit n: lane n+1 has filament at its insert sensor */
};

struct ace2k_dryer_event {
    uint8_t kind; /* enum ace2k_dryer_event_kind */
    uint8_t arg;  /* the fault's reason, the lowered lane (1–4), or 0 */
    uint8_t seq;  /* a running 8-bit number: the host acknowledges this */
};

#define ACE2K_DRYER_LOG_BOOT_INTERRUPTED    0x1U /* an open cycle became an entry */
#define ACE2K_DRYER_LOG_BOOT_CUTOUT         0x2U /* the cutout flag is kept: not a power-on */
#define ACE2K_DRYER_LOG_BOOT_CUTOUT_CLEARED 0x4U /* a power-on reset cleared the flag */
#define ACE2K_DRYER_TICK_MS                 10U  /* the period of ace2k_dryer_tick() */

/* The log helpers. They are pure and work on the log alone. The dryer-level functions of
 * dryer_log.c call them, and so do the tests. */
void ace2k_dryer_log_cycle_start(struct ace2k_dryer_log *log);
void ace2k_dryer_log_cycle_end(struct ace2k_dryer_log *log, bool completed);
/* Adds dt_ms of heating at duty_pct to heat_s and full_power_s. The remainders are kept to the
 * millisecond. */
void ace2k_dryer_log_account(struct ace2k_dryer_log *log, uint8_t duty_pct, uint32_t dt_ms);
/* Writes an entry at [0]. The other entries shift, and the oldest is dropped. A fault is counted.
 * A cutout fault sets ACE2K_DRYER_LOG_CUTOUT. in may be NULL, and then every temperature is
 * unknown. */
void ace2k_dryer_log_record(struct ace2k_dryer_log *log, uint8_t kind, uint8_t reason,
                            const struct ace2k_dryer_inputs *in);
/* Runs at boot. An open cycle becomes an interrupted entry, and its flag is cleared. A power-on
 * reset clears the cutout flag; any other reset keeps it. Returns ACE2K_DRYER_LOG_BOOT_* bits. */
uint32_t ace2k_dryer_log_boot(struct ace2k_dryer_log *log, bool power_on_reset);

/* The controller's state. It is zeroed at every start. */
struct ace2k_dryer_ctl {
    int32_t integ_m;        /* Σ error × Ki, in 0.001 cpct */
    int32_t prev_hotter_mc; /* the derivative's memory: the hotter outlet at the last step */
    bool primed;            /* prev_hotter_mc holds a real value */
    uint16_t duty_cpct;     /* the last output, 0–9 000 */
    uint32_t step_next_ms;
    uint32_t renew_next_ms;
    bool leaseless; /* heat is wanted (duty > 0), every lease since was refused, still waiting */
    uint32_t leaseless_since_ms; /* the first of those refusals */
    int32_t trim_mc;             /* the correction of the outer loop, ±ACE2K_DRYER_TRIM_MAX_MC */
    uint32_t outer_next_ms;
    int32_t ch_min_mc, ch_max_mc; /* the chamber over the current outer window */
};

/* The state of the protections. It is reset at every entry into HEATING (dryer_protect.c). */
struct ace2k_dryer_protect {
    int32_t start_mc[2]; /* left, right at the entry into HEATING */
    bool rise_met[2];
    bool rise_checked;
    uint32_t heating_since_ms;
    uint32_t sample_next_ms;
    uint8_t ntc_bad;
    int32_t win_ntc_mc[ACE2K_DRYER_RISE_SLOTS];
    uint8_t win_duty[ACE2K_DRYER_RISE_SLOTS];
    uint8_t win_idx, win_n;
    uint32_t win_next_ms;
    bool sides_apart;
    uint32_t sides_since_ms;
    bool chamber_over;
    uint32_t chamber_over_since_ms;
    uint8_t insert_prev; /* the insert mask of the previous tick, for the spool rule */
    int32_t lid_ref_mc;  /* the chamber at the entry into HEATING */
    bool lid_ref_set;
    bool lid_open;
    uint32_t lid_open_since_ms;
    uint32_t lid_next_ms;
};

struct ace2k_dryer {
    struct ace2k_heat *heat;
    struct ace2k_airflow *airflow;
    struct ace2k_dryer_log *log;
    uint8_t state;   /* enum ace2k_dryer_state */
    uint8_t fault;   /* enum ace2k_dryer_fault, the first reason */
    uint8_t notices; /* ACE2K_DRYER_NOTICE_* */
    /* The cycle's mains dips that the dryer rode out (ACE2K_DRYER_NOTICE_MAINS_DIP). Kept in
     * the core only. The status frame has no field for it, and the dictionary is unchanged. */
    uint16_t mains_dips;
    bool mains_dip_open; /* a dip in progress; counted once the mains reads plausible again */
    uint32_t phase_rejects_start; /* heat's phase_rejects at the start */
    uint8_t target_c;
    uint8_t highest_target_c; /* the cycle's highest target: the base of the chamber ceiling */
    int32_t start_chamber_mc; /* lowest valid chamber reading since the start: the floor of the
                               * ceiling (dryer_internal.h) */
    bool shut;                /* a Klipper shutdown happened: nothing moves any more */
    /* The cutout input was seen asserted at some tick, in any state, since init. This flag is
     * sticky, whatever fault came first, because only a power-on reset clears a tripped cutout.
     * While it is set, clear answers ACE2K_DRYER_CUTOUT, and start refuses as it does for
     * ACE2K_DRYER_LOG_CUTOUT. The tick sets that persisted flag, and marks the log, the moment it
     * sees the input. */
    bool cutout_seen;
    uint32_t state_since_ms;
    uint32_t end_ms;
    uint32_t heating_since_ms;
    struct ace2k_dryer_flaps flaps;
    bool start_logged; /* STARTING wrote the cycle's open mark */
    struct ace2k_dryer_vent vent;
    struct ace2k_dryer_vent_room room; /* the humidity guard's room reference, kept on restart */
    bool heated;            /* this cycle reached HEATING, so its cool-down opens the flaps */
    uint8_t cool;           /* enum ace2k_dryer_cool */
    int32_t cool_ref_mc[2]; /* left, right at the window's start */
    uint32_t cool_ref_ms;
    /* An NTC read invalid when a window closed, or at the entry into the cool-down. The references
     * are stale, and the window restarts on the first tick both sides read valid again. */
    bool cool_rearm;
    /* The inputs of the last tick. An invalid NTC is held at ntc_held_mc. */
    struct ace2k_dryer_inputs last;
    bool have_inputs;
    int32_t ntc_held_mc[2]; /* last valid value of each NTC, left then right; 0 before the first */
    struct ace2k_dryer_ctl ctl;
    struct ace2k_dryer_protect prot;
    struct ace2k_dryer_event ring[ACE2K_DRYER_EVENT_RING];
    volatile uint8_t ring_head, ring_tail;
    volatile uint8_t ring_sent; /* the next event to send; tail ≤ sent ≤ head */
    /* One past the newest event ever sent. A resend does not lower it.
     * tail ≤ sent ≤ sent_hi ≤ head. */
    volatile uint8_t ring_sent_hi;
    uint8_t ev_seq; /* the seq of the next event */
    uint32_t events_lost;
    uint32_t log_gen;        /* bumped by every change that must reach the page */
    uint32_t log_stored_gen; /* the generation the binding last stored */
};

/* Sets the dryer to IDLE. Call it after heat and airflow are initialised. log is the page as read
 * at boot. Both flaps are closed from the first tick, whatever the state. A cycle left open on the
 * page is logged as interrupted and never resumed. A cutout kept on the page holds the dryer in
 * FAULT CUTOUT, unless power_on_reset is true, which clears it. Only a power cycle clears a
 * tripped cutout. */
void ace2k_dryer_init(struct ace2k_dryer *self, struct ace2k_heat *heat,
                      struct ace2k_airflow *airflow, struct ace2k_dryer_log *log,
                      bool power_on_reset, uint32_t now_ms);

/* Starts a cycle at target_c for minutes. The dryer goes to STARTING and owns the fans. The
 * refusals are judged on the inputs of the last tick. The first one that applies wins, in this
 * order:
 * - SHUTDOWN: a Klipper shutdown happened.
 * - FAULTED: the dryer is in FAULT.
 * - BUSY: the dryer is not IDLE.
 * - RANGE: target_c is outside ACE2K_DRYER_TARGET_MIN_C to ACE2K_DRYER_TARGET_MAX_C, or minutes
 *   is 0 or above ACE2K_DRYER_MINUTES_MAX.
 * - CUTOUT: the cutout is asserted, was seen since boot, or is kept on the page.
 * - SENSORS: no tick yet, an outlet NTC is invalid, or the chamber is invalid or stale.
 * - NO_MAINS: the mains is absent.
 * - MEASURING: the mains is back, but no window is measured yet. Try again.
 * - NO_MAINS: heat's mains bucket is above its restart level.
 * - FAULTED: heat is latched.
 * - BUSY: another owner holds the fans.
 * A refusal changes nothing. */
enum ace2k_dryer_refusal ace2k_dryer_start(struct ace2k_dryer *self, uint8_t target_c,
                                           uint16_t minutes, uint32_t now_ms);

/* Stops a cycle in STARTING or HEATING. The dryer goes to COOLDOWN with a stopped event. In FAULT
 * it answers FAULTED, because the cool-down of a fault is not stopped. In IDLE or COOLDOWN it
 * answers OK and does nothing. */
enum ace2k_dryer_refusal ace2k_dryer_stop(struct ace2k_dryer *self, uint32_t now_ms);

/* Replaces the running cycle's request in place: STARTING or HEATING only. The target becomes
 * target_c and the end now + minutes. highest_target_c only ever rises within a cycle: it keeps
 * the higher of the two, so a higher target raises the chamber ceiling. The lowered notice is
 * cleared, because the new request is the user's. The ambient-above notice is not judged again:
 * it stays as the start judged it. Refusals, the first that applies: FAULTED in FAULT, BUSY in
 * any other state but STARTING and HEATING, RANGE as ace2k_dryer_start judges it. A refusal
 * changes nothing. */
enum ace2k_dryer_refusal ace2k_dryer_retarget(struct ace2k_dryer *self, uint8_t target_c,
                                              uint16_t minutes, uint32_t now_ms);

/* Lowers the running cycle's target to target_c, as the spool rule does: in STARTING or HEATING,
 * and only when target_c is lower. The end, the chamber ceiling's base and the notices stay, and
 * no event goes out. target_c must be in the dryer's range (the caller checks it). */
void ace2k_dryer_lower_target(struct ace2k_dryer *self, uint8_t target_c);

/* Leaves FAULT for IDLE, with a cleared event. The refusals, the first one that applies:
 * - SHUTDOWN: the dryer is shut down. This is checked first, in every state.
 * - CUTOUT: the cutout is on the page. This is checked in every state, and only a power-on reset
 *   clears it.
 * - Outside FAULT, it answers OK and does nothing.
 * - CUTOUT: the cutout is the fault's reason, is asserted, or was seen since boot. Only a
 *   power cycle clears it.
 * - NOT_COOL: the cool-down is not over.
 * - The cause is still present: SENSORS for an NTC or a stale chamber, NO_MAINS or MEASURING for
 *   the mains, NOT_COOL for a chamber still over its ceiling.
 * - Then heat's own clear. When heat refuses, the answer names what holds its latch:
 *   CUTOUT, SENSORS, NOT_COOL, NO_MAINS, or else FAULTED. */
enum ace2k_dryer_refusal ace2k_dryer_clear(struct ace2k_dryer *self, uint32_t now_ms);
/* Runs every 10 ms, after airflow and heat. */
void ace2k_dryer_tick(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *raw,
                      uint32_t now_ms);
/* The Klipper shutdown. Heat shuts down: no pending half-cycle, and no lease ever again. The fans
 * are left to airflow's fan rule. Nothing moves afterwards, not even a flap pulse. The flaps
 * stay where they are until the close at the next boot. A flap whose pulse the shutdown cut is
 * published as unknown. A start answers ACE2K_DRYER_SHUTDOWN until a reset. The dryer is left
 * IDLE, or in FAULT with its reason when it was in a fault. The cool-down no longer runs, so a
 * clear answers ACE2K_DRYER_SHUTDOWN and changes nothing until a reset. */
void ace2k_dryer_shutdown(struct ace2k_dryer *self);
/* True when the log changed and must be stored. The binding stores it from a task, only when
 * ace2k_heat_busy() is false, through ace2k_config_binding_store(). This is because the flash is
 * written only with the gate idle. The handshake: read the generation, store, then report that
 * generation as stored. A change logged meanwhile leaves the log dirty. */
bool ace2k_dryer_log_dirty(const struct ace2k_dryer *self);
/* The log's current generation. Every change that must reach the page bumps it. */
uint32_t ace2k_dryer_log_generation(const struct ace2k_dryer *self);
void ace2k_dryer_log_stored(struct ace2k_dryer *self, uint32_t generation);
/* The events, oldest first, read from the task. next() returns the next event not yet sent since
 * the last resend. Call sent() once the event was handed to the transport. An event leaves the
 * ring only through ack(), which takes every event up to seq, in 8-bit order. resend() puts the
 * cursor back on the oldest unacknowledged event.
 * Some acks are ignored whole: an ack whose seq is after the newest event ever sent, and any ack
 * while no unacknowledged event was ever sent. A resend does not lower the mark of the newest
 * event sent. Such an ack cannot come from a host that saw those events. Taking it would drop
 * events that were never sent, and nothing would count them.
 * Call ack() and resend() from a command or the task, never from the tick. */
bool ace2k_dryer_event_next(const struct ace2k_dryer *self, struct ace2k_dryer_event *out);
void ace2k_dryer_event_sent(struct ace2k_dryer *self);
void ace2k_dryer_event_ack(struct ace2k_dryer *self, uint8_t seq);
void ace2k_dryer_event_resend(struct ace2k_dryer *self);
bool ace2k_dryer_event_unacked(const struct ace2k_dryer *self);
/* The binding's timer resend, besides the resend that every query asks for. It is due when
 * the host listens, an event is unacknowledged, and nothing was sent for
 * ACE2K_DRYER_EVENT_RESEND_MS. */
#define ACE2K_DRYER_EVENT_RESEND_MS 5000U
bool ace2k_dryer_event_resend_due(const struct ace2k_dryer *self, bool subscribed,
                                  uint32_t last_send_ms, uint32_t now_ms);
/* The seq of the oldest unacknowledged event, or the next seq to assign when no event is held.
 * After a connect, this is the host's base: the oldest field of the state report. */
uint8_t ace2k_dryer_event_oldest(const struct ace2k_dryer *self);

struct ace2k_dryer_status {
    uint8_t state, target_c, duty_pct, fault, notices, flaps;
    uint16_t drive_dc, remaining_min;
    uint8_t fans_read;
    uint16_t events_lost; /* events dropped on a full ring since the boot, saturating */
    uint8_t oldest;       /* ace2k_dryer_event_oldest() */
};
/* flaps: bits 0–1 bottom, bits 2–3 rear (enum ace2k_flap_pos). */
void ace2k_dryer_status(const struct ace2k_dryer *self, struct ace2k_dryer_status *out,
                        uint32_t now_ms);

#endif
