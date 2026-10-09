/* What: the autonomous dryer — the drying policy over the heat layer and the airflow outputs:
 * the states, the cascade controller, the vent (dryer_vent.h), the cool-down, the protections,
 * the spool rule and the counters of its log.
 * How: the binding (or the test rig) owns one struct ace2k_dryer and one struct
 * ace2k_dryer_log; ace2k_dryer_tick() every 10 ms, after ace2k_airflow_tick() and
 * ace2k_heat_tick(), with one struct ace2k_dryer_inputs; start / stop / clear from command
 * context under the binding's irq bracket; events through a single-producer (tick) /
 * single-consumer (task) ring, as sensors.h, each held until the host acknowledges it (a full ring
 * drops the new event and counts it in the status); a change of the log bumps a generation for the
 * binding to store from a task with the gate idle (a page erase stalls the core).  Temperatures in millidegrees,
 * duty in 0.01 % inside the controller, whole percent at the heat layer.  No floats.
 * Depends on: <stdbool.h>, <stdint.h>, airflow.h, dryer_vent.h, heat.h, util.h. */
#ifndef ACE2K_DRYER_H
#define ACE2K_DRYER_H
#include <stdbool.h>
#include <stdint.h>

#include "heat/airflow.h"
#include "heat/heat.h"
#include "dryer/dryer_vent.h"
#include "core/util.h"

/* The request. */
#define ACE2K_DRYER_TARGET_MIN_C     15U
#define ACE2K_DRYER_TARGET_MAX_C     65U
#define ACE2K_DRYER_MINUTES_MAX      1440U /* 24 h */
#define ACE2K_DRYER_DRIVE_OFFSET_MC  7000  /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DRIVE_CAP_MC     8000  /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DRIVE_ABS_MAX_MC 73000
#define ACE2K_DRYER_SPOOL_SAFE_C     45U

/* The controller (HEATING).  Kp 1.0 %/°C, Ki 0.01 %/°C per period, Kd 5.0 %/°C per
 * period, written in 0.01 % of duty (cpct) per °C. */
#define ACE2K_DRYER_PID_PERIOD_MS  2000U /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KP_CPCT_PER_C  100   /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KI_CPCT_PER_C  1     /* initial value; re-tune on the bench */
#define ACE2K_DRYER_KD_CPCT_PER_C  500   /* initial value; re-tune on the bench */
#define ACE2K_DRYER_DUTY_MAX_CPCT  9000  /* 90 %, the heat layer's own ceiling */
#define ACE2K_DRYER_CPCT_PER_PCT   100U
#define ACE2K_DRYER_LEASE_RENEW_MS 500U /* a lease of 2 000 ms renewed four times over */
/* Wanting heat (duty > 0) with every lease refused and waiting this long is a fault: the wait's
 * cause is heat's mains bucket held above its restart level by a mains off-band about one window
 * in three (-ACE2K_EAGAIN), else what refused it.  A normal wait is far shorter: the bucket drains
 * from just below its latch to the restart level in 2.0 s (203 ticks), a further off-band window
 * during that wait adds 1 s, and a lasting off-band fills the bucket in 1.5 s and faults as a
 * latch; 10 s is three times the longest normal wait.  A duty of 0 (no lease wanted) resets it. */
#define ACE2K_DRYER_NO_LEASE_MAX_MS   10000U  /* initial value */
#define ACE2K_DRYER_TRIM_MAX_MC       2000    /* initial value; re-tune on the bench */
#define ACE2K_DRYER_TRIM_GAIN_DIV     2       /* trim += (target − chamber) / 2 per step */
#define ACE2K_DRYER_OUTER_PERIOD_MS   600000U /* initial value; re-tune on the bench */
#define ACE2K_DRYER_CHAMBER_STEADY_MC 800     /* initial value; re-tune on the bench */

/* STARTING. */
#define ACE2K_DRYER_START_TIMEOUT_MS 3000U         /* initial value */
#define ACE2K_DRYER_FANS_BOTH ACE2K_HEAT_FANS_BOTH /* ace2k_airflow_fans_read(): left | right */

/* COOLDOWN. */
#define ACE2K_DRYER_COOL_MC      45000
#define ACE2K_DRYER_COOL_MIN_MS  30000U /* also the falling-rate window */
#define ACE2K_DRYER_COOL_FALL_MC 500    /* 1 °C/min over 30 s: fell less, and did not rise */
/* A rise up to this over the window is the NTC's noise and reads as steady; more is the overshoot
 * after a stop.  Initial value; re-tune on the bench. */
#define ACE2K_DRYER_COOL_RISE_MC   200
#define ACE2K_DRYER_HOT_AMBIENT_MS 600000U

/* The protections (dryer_protect.c implements them). */
#define ACE2K_DRYER_NOT_HEATING_MS      300000U
#define ACE2K_DRYER_NOT_HEATING_RISE_MC 5000
#define ACE2K_DRYER_NOT_HEATING_NEAR_MC 2000
#define ACE2K_DRYER_RISE_WINDOW_S       30U
/* The gate → outlet NTC lag (docs/hardware.md "Heater": 10–15 s): a rise over the window was
 * also driven by the duty of the lag before it — the overshoot after a stop.  That duty counts at
 * ACE2K_DRYER_RISE_LAG_WEIGHT_PCT: the measured overshoot, +3.8 °C after 50 %, is 15 % of the
 * full ceiling over 15 s at 50 % (25.5 °C); 25 % covers it with margin and keeps a blocked duct,
 * which also looks like a rise after the duty fell, caught within a minute on the simulator. */
#define ACE2K_DRYER_RISE_LAG_S            15U
#define ACE2K_DRYER_RISE_LAG_WEIGHT_PCT   25 /* initial value; re-tune on the bench */
#define ACE2K_DRYER_RISE_SLOTS            (ACE2K_DRYER_RISE_WINDOW_S + ACE2K_DRYER_RISE_LAG_S + 1U)
#define ACE2K_DRYER_RISE_SAMPLE_MS        1000U
#define ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT 35 /* from tools/thermal_fit.py; re-tune on the bench */
#define ACE2K_DRYER_RISE_MARGIN_PCT                                                                \
    100 /* the ceiling above already carries the fit's 1.5x; re-tune on the bench */
#define ACE2K_DRYER_RISE_FLOOR_MC      2000  /* initial value; re-tune on the bench */
#define ACE2K_DRYER_SIDES_APART_MC     15000 /* initial value */
#define ACE2K_DRYER_SIDES_MS           60000U
#define ACE2K_DRYER_CHAMBER_OVER_MC    10000
#define ACE2K_DRYER_CHAMBER_OVER_MS    30000U
#define ACE2K_DRYER_CHAMBER_ABS_MAX_MC 80000
#define ACE2K_DRYER_CHAMBER_STALE_MS   5000U
#define ACE2K_DRYER_NTC_SAMPLE_MS      100U
#define ACE2K_DRYER_NTC_BAD_SAMPLES    5U

/* The flaps: every sequence pulses both the same way, the bottom asked first (dryer.c
 * flaps_begin).  A sequence is over once the dryer's own two requests have ended in airflow, or
 * this long after it began — whatever it left unfinished airflow publishes unknown by itself.  Two
 * pulses, and one already running when asked, take at most 3 × ACE2K_FLAP_PULSE_MAX_MS. */
#define ACE2K_DRYER_FLAPS_TIMEOUT_MS 2000U

enum ace2k_dryer_flaps_result {
    ACE2K_DRYER_FLAPS_OVER = 0,  /* both of its pulses ran to their end (or none was begun) */
    ACE2K_DRYER_FLAPS_RUNNING,   /* asking, or waiting for its pulses to end */
    ACE2K_DRYER_FLAPS_TIMED_OUT, /* ACE2K_DRYER_FLAPS_TIMEOUT_MS passed first */
};

/* The one flap sequence in flight; a new one replaces it (airflow keeps a request of the dryer's
 * waiting on a flap until a later one of the dryer's replaces it). */
struct ace2k_dryer_flaps {
    uint8_t next;   /* the flap asked next (enum ace2k_flap); ACE2K_FLAP_COUNT once both were */
    uint8_t result; /* enum ace2k_dryer_flaps_result */
    bool open;
    uint32_t since_ms;
};

/* The cool-down's (and the fault's) course. */
enum ace2k_dryer_cool {
    ACE2K_DRYER_COOL_RELEASED = 0, /* the flaps closed, the fans handed to rule 7 */
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
    ACE2K_DRYER_FAULT_FLAPS = 11, /* STARTING: the flaps' close not at rest within the timeout */
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
    ACE2K_DRYER_SHUTDOWN = 8,  /* start: a Klipper shutdown happened; only a reset dries again */
    ACE2K_DRYER_MEASURING = 9, /* the mains back, its first window not measured yet: transient */
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
};

#define ACE2K_DRYER_NOTICE_HOT_AMBIENT 0x1U
#define ACE2K_DRYER_NOTICE_LOWERED     0x2U
/* set by the binding, not the core: the log's last store failed on the flash (dryer_cmds.c) */
#define ACE2K_DRYER_NOTICE_LOG_UNSTORED 0x4U
/* A mains dip occurred this cycle and did not fault it: off-band in STARTING or HEATING (or
 * already at the start), counted when the mains reads plausible again with the cycle going on, or
 * when the cycle ends without a fault with the dip still open — a dip heat's bucket tolerated
 * (heat.h), made visible.  It does not say the dip ended.  The count is mains_dips. */
#define ACE2K_DRYER_NOTICE_MAINS_DIP 0x8U
/* At least ACE2K_DRYER_PHASE_REJECTS_NOTICE zero-cross edges judged out of phase by heat this
 * cycle (heat.h phase_rejects): noise at the input, or a half-cycle asymmetry past heat's
 * tolerance — the heater skips those cycles, the notice makes it visible. */
#define ACE2K_DRYER_NOTICE_EDGES_OFF_PHASE 0x10U
/* The chamber started above the target: nothing heats until it falls below (the drive target sits
 * below the outlets), the fans and the flaps follow the cycle.  Judged once, at the start. */
#define ACE2K_DRYER_NOTICE_AMBIENT_ABOVE 0x20U
#define ACE2K_DRYER_PHASE_REJECTS_NOTICE 10U /* a missed edge costs two; ten is a pattern */
#define ACE2K_DRYER_EVENT_RING           8U  /* one slot kept free: seven events */

/* One tick's view of the world, filled by the binding (or the test rig). */
struct ace2k_dryer_inputs {
    int32_t ntc_left_mc, ntc_right_mc;
    bool left_valid, right_valid;
    int32_t chamber_mc;
    bool chamber_valid;
    uint32_t chamber_age_ms;
    uint16_t chamber_rh_pct10; /* the chamber sensor's RH, 0.1 % */
    bool rh_valid;
    bool mains_present, mains_plausible;
    bool cutout;
    bool link_ok;
    uint8_t insert_mask; /* bit n: lane n+1 has a strand at its insert sensor */
};

struct ace2k_dryer_event {
    uint8_t kind; /* enum ace2k_dryer_event_kind */
    uint8_t arg;  /* the fault's reason, the lowered lane (1–4), or 0 */
    uint8_t seq;  /* running, 8-bit: what the host acknowledges */
};

struct ace2k_dryer_log_entry { /* 16 bytes on the page */
    uint8_t kind;              /* ACE2K_DRYER_EV_FAULT or ACE2K_DRYER_EV_INTERRUPTED */
    uint8_t reason;            /* enum ace2k_dryer_fault, 0 for interrupted */
    uint16_t cycle;            /* the cycle open at the time (cycles_started), 0 if none was */
    uint32_t heat_s;           /* heat_s at the time */
    int16_t ntc_left_dc, ntc_right_dc, chamber_dc; /* 0.1 °C */
    uint16_t reserved;
};

#define ACE2K_DRYER_LOG_ENTRIES 8U
struct ace2k_dryer_log {
    uint32_t cycles_started, cycles_completed, heat_s, full_power_s, faults;
    uint32_t flags; /* ACE2K_DRYER_LOG_CYCLE_OPEN, ACE2K_DRYER_LOG_CUTOUT */
    /* sub-second remainders of the two time counters; not persisted */
    uint32_t full_power_ms_frac;
    uint32_t heat_ms_frac;
    struct ace2k_dryer_log_entry entry[ACE2K_DRYER_LOG_ENTRIES]; /* [0] newest */
};
#define ACE2K_DRYER_LOG_CYCLE_OPEN 0x1U
#define ACE2K_DRYER_LOG_CUTOUT     0x2U

#define ACE2K_DRYER_LOG_TEMP_UNKNOWN_DC     INT16_MIN /* a reading that was invalid or absent */
#define ACE2K_DRYER_LOG_BOOT_INTERRUPTED    0x1U      /* an open cycle became an entry */
#define ACE2K_DRYER_LOG_BOOT_CUTOUT         0x2U      /* the cutout flag is kept: rule 10 */
#define ACE2K_DRYER_LOG_BOOT_CUTOUT_CLEARED 0x4U      /* a power-on reset cleared the flag */
#define ACE2K_DRYER_TICK_MS                 10U       /* the period of ace2k_dryer_tick() */

/* The log helpers — pure, on the log alone; the dryer-level functions of dryer_log.c call them,
 * the tests too. */
void ace2k_dryer_log_cycle_start(struct ace2k_dryer_log *log);
void ace2k_dryer_log_cycle_end(struct ace2k_dryer_log *log, bool completed);
/* dt_ms of heating at duty_pct: heat_s and full_power_s, remainders kept to the millisecond. */
void ace2k_dryer_log_account(struct ace2k_dryer_log *log, uint8_t duty_pct, uint32_t dt_ms);
/* An entry at [0], the others shifted, the oldest dropped; a fault counts; a cutout fault sets
 * ACE2K_DRYER_LOG_CUTOUT.  in may be NULL (every temperature unknown). */
void ace2k_dryer_log_record(struct ace2k_dryer_log *log, uint8_t kind, uint8_t reason,
                            const struct ace2k_dryer_inputs *in);
/* At boot: an open cycle becomes an interrupted entry (flag cleared); the cutout flag is
 * cleared by a power-on reset and kept otherwise.  ACE2K_DRYER_LOG_BOOT_* bits. */
uint32_t ace2k_dryer_log_boot(struct ace2k_dryer_log *log, bool power_on_reset);

/* The controller's state, zeroed at every start. */
struct ace2k_dryer_ctl {
    int32_t integ_m;     /* Σ error × Ki: 0.001 cpct */
    int32_t prev_err_mc; /* the derivative's memory */
    bool primed;         /* prev_err_mc holds a real value */
    uint16_t duty_cpct;  /* the last output, 0–9 000 */
    uint32_t step_next_ms;
    uint32_t renew_next_ms;
    bool leaseless; /* wanting heat (duty > 0), every lease since refused and waiting */
    uint32_t leaseless_since_ms; /* the first of those refusals */
    int32_t trim_mc;             /* the outer loop's correction, ±ACE2K_DRYER_TRIM_MAX_MC */
    uint32_t outer_next_ms;
    int32_t ch_min_mc, ch_max_mc; /* the chamber over the current outer window */
};

/* The protections' state, reset at every entry into HEATING (dryer_protect.c). */
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
    uint8_t insert_prev; /* the insert mask of the previous tick (the spool rule) */
};

struct ace2k_dryer {
    struct ace2k_heat *heat;
    struct ace2k_airflow *airflow;
    struct ace2k_dryer_log *log;
    uint8_t state;   /* enum ace2k_dryer_state */
    uint8_t fault;   /* enum ace2k_dryer_fault, the first reason */
    uint8_t notices; /* ACE2K_DRYER_NOTICE_* */
    /* The cycle's mains dips ridden out (ACE2K_DRYER_NOTICE_MAINS_DIP).  Kept in the core only —
     * the status frame has no field for it (the dictionary is unchanged). */
    uint16_t mains_dips;
    bool mains_dip_open; /* a dip in progress, counted once the mains reads plausible again */
    uint32_t phase_rejects_start; /* heat's phase_rejects at the start */
    uint8_t target_c;
    uint8_t highest_target_c; /* the cycle's highest target: the chamber ceiling's base */
    int32_t start_chamber_mc; /* the chamber's lowest valid reading since the start: the ceiling's
                               * floor (dryer_internal.h) */
    bool shut;                /* a Klipper shutdown happened: nothing moves any more */
    /* The cutout input seen asserted at any tick, in any state, since init — sticky, whatever
     * fault came first (rule 10: only a power-on reset clears a tripped cutout).  While set, clear
     * answers ACE2K_DRYER_CUTOUT and start refuses as for ACE2K_DRYER_LOG_CUTOUT; the tick sets
     * that persisted flag (and marks the log) the moment the input is seen. */
    bool cutout_seen;
    uint32_t state_since_ms;
    uint32_t end_ms;
    uint32_t heating_since_ms;
    struct ace2k_dryer_flaps flaps;
    bool start_logged; /* STARTING wrote the cycle's open mark */
    struct ace2k_dryer_vent vent;
    struct ace2k_dryer_vent_room room; /* the guard's room reference, kept across a quick restart */
    bool heated;            /* this cycle reached HEATING: its cool-down opens the flaps */
    uint8_t cool;           /* enum ace2k_dryer_cool */
    int32_t cool_ref_mc[2]; /* left, right at the window's start */
    uint32_t cool_ref_ms;
    /* an NTC read invalid at a window's close (or at the cool-down's entry): the references are
     * stale, and the window restarts on the first tick both sides read valid again */
    bool cool_rearm;
    struct ace2k_dryer_inputs last;
    bool have_inputs;
    struct ace2k_dryer_ctl ctl;
    struct ace2k_dryer_protect prot;
    struct ace2k_dryer_event ring[ACE2K_DRYER_EVENT_RING];
    volatile uint8_t ring_head, ring_tail;
    volatile uint8_t ring_sent; /* the next event to send; tail ≤ sent ≤ head */
    /* past the newest event ever sent (a resend does not lower it); tail ≤ sent ≤ sent_hi ≤ head */
    volatile uint8_t ring_sent_hi;
    uint8_t ev_seq; /* the next event's seq */
    uint32_t events_lost;
    uint32_t log_gen;        /* bumped by every change that must reach the page */
    uint32_t log_stored_gen; /* the generation the binding last stored */
};

void ace2k_dryer_init(struct ace2k_dryer *self, struct ace2k_heat *heat,
                      struct ace2k_airflow *airflow, struct ace2k_dryer_log *log,
                      bool power_on_reset, uint32_t now_ms);
enum ace2k_dryer_refusal ace2k_dryer_start(struct ace2k_dryer *self, uint8_t target_c,
                                           uint16_t minutes, uint32_t now_ms);
enum ace2k_dryer_refusal ace2k_dryer_stop(struct ace2k_dryer *self, uint32_t now_ms);
enum ace2k_dryer_refusal ace2k_dryer_clear(struct ace2k_dryer *self, uint32_t now_ms);
/* Every 10 ms, after airflow and heat. */
void ace2k_dryer_tick(struct ace2k_dryer *self, const struct ace2k_dryer_inputs *in,
                      uint32_t now_ms);
/* The Klipper shutdown: heat shut down (no pending half, no lease ever again), the fans left to
 * rule 7, nothing moves afterwards — no flap pulse either: the flaps stay where they are (a flap
 * whose pulse the shutdown cut is published unknown) until the next boot's close; a start
 * answers ACE2K_DRYER_SHUTDOWN until a reset. */
void ace2k_dryer_shutdown(struct ace2k_dryer *self);
/* True when the log changed and must be stored (the binding stores from a task, only with
 * !ace2k_heat_busy — rule 13, through ace2k_config_binding_store()).  The handshake: read the
 * generation, store, then report that generation stored; a change logged meanwhile leaves the
 * log dirty. */
bool ace2k_dryer_log_dirty(const struct ace2k_dryer *self);
uint32_t ace2k_dryer_log_generation(const struct ace2k_dryer *self);
void ace2k_dryer_log_stored(struct ace2k_dryer *self, uint32_t generation);
/* The events, oldest first, from the task: next() the next one not yet sent since the last
 * resend; sent() once it was handed to the transport.  An event leaves the ring only through
 * ack(), which takes every event up to seq (8-bit order); resend() puts the cursor back on the
 * oldest unacknowledged one.  An ack whose seq is after the newest event ever sent — a resend
 * does not lower that mark — or any ack while no unacknowledged event was ever sent, is ignored
 * whole: it cannot come from a host that saw those events, and taking it would drop events never
 * sent, uncounted.  ack() and resend() from a command or the task;
 * never from the tick. */
bool ace2k_dryer_event_next(const struct ace2k_dryer *self, struct ace2k_dryer_event *out);
void ace2k_dryer_event_sent(struct ace2k_dryer *self);
void ace2k_dryer_event_ack(struct ace2k_dryer *self, uint8_t seq);
void ace2k_dryer_event_resend(struct ace2k_dryer *self);
bool ace2k_dryer_event_unacked(const struct ace2k_dryer *self);
/* The binding's timer resend (besides the one every query asks): due once the host listens, an
 * event is unacknowledged and nothing was sent for ACE2K_DRYER_EVENT_RESEND_MS. */
#define ACE2K_DRYER_EVENT_RESEND_MS 5000U
bool ace2k_dryer_event_resend_due(const struct ace2k_dryer *self, bool subscribed,
                                  uint32_t last_send_ms, uint32_t now_ms);
/* The seq of the oldest unacknowledged event, or the next seq to be assigned when none is held:
 * the host's base after a connect (the state report's oldest). */
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
