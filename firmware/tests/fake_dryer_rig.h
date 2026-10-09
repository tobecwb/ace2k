/* What: the dryer's test plant — the real airflow and heat cores on recording fakes, the thermal
 * model of fake_thermal.h, and a clock that runs the 10 ms tick with the 60 Hz zero-cross edges
 * between ticks, in the order the binding uses (heat's hold on the fans, airflow, heat, dryer).
 * How: rig_init(), then rig_tick() / rig_run_ms(); per-field overrides replace the model's
 * readings; every dryer event is drained into ev[] after each tick, as the binding's task would;
 * the log is "stored" whenever it is dirty and the gate is idle; the gate drops
 * 6 ms after a fire, as TIM7 does.  Header-only, every function static inline.
 * Depends on: airflow.h, heat.h, dryer.h, dryer_internal.h, fake_thermal.h. */
#ifndef ACE2K_TEST_FAKE_DRYER_RIG_H
#define ACE2K_TEST_FAKE_DRYER_RIG_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "heat/airflow.h"
#include "dryer/dryer.h"
#include "dryer/dryer_internal.h"
#include "fake_thermal.h"
#include "heat/heat.h"

#define RIG_TICK_MS             10U
#define RIG_TICK_US             10000U
#define RIG_HALF_CYCLE_US       8333U /* 60 Hz, two edges per cycle */
#define RIG_MAINS_HZ10          600U  /* what the mains measures of it */
#define RIG_MAINS_HZ10_OFF_BAND 585U  /* the bench's dip: one window read 58.5 Hz */
#define RIG_PLANT_MS            100U
#define RIG_EDGES_PER_PLANT     12U
#define RIG_EV_MAX              64U
#define RIG_MIN_MS              60000U
/* Both flaps' pulses, one after the other (dryer.c flaps_begin): two pulses and two ticks of
 * margin after each. */
#define RIG_FLAP_PAIR_MS (2U * (ACE2K_FLAP_PULSE_MS + (2U * RIG_TICK_MS)))

// NOLINTNEXTLINE(readability-identifier-naming)
struct rig {
    struct fake_thermal plant;
    struct ace2k_airflow airflow;
    struct ace2k_heat heat;
    struct ace2k_dryer dryer;
    struct ace2k_dryer_log log;
    /* the outputs */
    bool fans_on, fan_fail, coil_both;
    uint8_t
        plant_blocked; /* fan_mask bits whose duct is blocked: the fan reads high, no air moves */
    uint32_t flap_pulses;
    uint32_t flap_opens[ACE2K_FLAP_COUNT], flap_closes[ACE2K_FLAP_COUNT]; /* pulses per coil */
    bool flap_coil_on[ACE2K_FLAP_COUNT]; /* a coil of the flap is driven now */
    bool flap_overlap;                   /* both flaps were driven at the same time */
    bool flap_claimed; /* a position published for a flap whose coil was driven at a tick's end */
    /* the flap counts before the last tick, kept when that tick changed the dryer's state (a
     * sequence the new state began is asked in the same tick) */
    bool entered_by_tick;
    uint32_t entry_pulses, entry_opens[ACE2K_FLAP_COUNT], entry_closes[ACE2K_FLAP_COUNT];
    bool gate, gate_stuck; /* gate_stuck: the pin reads high whatever heat commands */
    uint32_t gate_on_us, edge_us, next_edge_us;
    /* the clock */
    uint32_t now_ms, now_us, plant_ms, fired_mark;
    /* the world */
    bool heater_dead, dryer_frozen;
    bool mains_present, mains_plausible, cutout, link_ok, chamber_valid;
    bool mains_unmeasured; /* no window measured yet (a boot): hz10 0 */
    uint32_t chamber_age_ms;
    uint8_t insert_mask;
    uint16_t rh_pct10; /* the chamber's RH, 0.1 % (the model has none): 30 % by default */
    bool rh_valid;
    bool ovr_ntc, ovr_left_valid, ovr_right_valid, ovr_chamber;
    int32_t ovr_left_mc, ovr_right_mc, ovr_chamber_mc;
    /* what was seen */
    int32_t max_ntc_mc;
    int32_t max_drive_mc;  /* the drive target, over every HEATING tick */
    uint32_t max_duty_pct; /* the controller's output and the heat layer's duty, the larger */
    uint32_t stores;
    struct ace2k_dryer_event ev[RIG_EV_MAX];
    uint32_t ev_n;
    struct ace2k_dryer_inputs last_in;
};

static inline void rig_fan_write(void *ctx, bool on)
{
    ((struct rig *)ctx)->fans_on = on;
}

static inline uint8_t rig_fan_read(void *ctx)
{
    const struct rig *r = ctx;
    if (r->fan_fail || !r->fans_on) {
        return 0;
    }
    return (uint8_t)ACE2K_DRYER_FANS_BOTH;
}

static inline void rig_flap_write(void *ctx, enum ace2k_flap flap, bool open_coil, bool close_coil)
{
    struct rig *r = ctx;
    if (open_coil && close_coil) {
        r->coil_both = true;
    }
    if (open_coil || close_coil) {
        r->flap_pulses++;
    }
    if (open_coil && !close_coil) {
        r->flap_opens[flap]++;
    }
    if (close_coil && !open_coil) {
        r->flap_closes[flap]++;
    }
    r->flap_coil_on[flap] = false;
    if (open_coil || close_coil) {
        r->flap_coil_on[flap] = true;
    }
    if (r->flap_coil_on[ACE2K_FLAP_BOTTOM] && r->flap_coil_on[ACE2K_FLAP_REAR]) {
        r->flap_overlap = true;
    }
}

// NOLINTNEXTLINE(readability-identifier-naming)
static const struct ace2k_airflow_ops rig_airflow_ops = {
    .fan_write = rig_fan_write,
    .fan_read = rig_fan_read,
    .flap_write = rig_flap_write,
};

static inline void rig_gate_fire(void *ctx)
{
    struct rig *r = ctx;
    r->gate = true;
    r->gate_on_us = r->edge_us;
}

static inline void rig_gate_off(void *ctx)
{
    ((struct rig *)ctx)->gate = false;
}

static inline bool rig_gate_read(void *ctx)
{
    const struct rig *r = ctx;
    if (r->gate) {
        return true;
    }
    return r->gate_stuck;
}

// NOLINTNEXTLINE(readability-identifier-naming)
static const struct ace2k_heat_ops rig_heat_ops = {
    .gate_fire = rig_gate_fire,
    .gate_off = rig_gate_off,
    .gate_read = rig_gate_read,
};

/* TIM7's one-pulse end: the gate falls ACE2K_HEAT_GATE_PULSE_US after the fire. */
static inline void rig_drop_gate(struct rig *r, uint32_t now_us)
{
    if (r->gate && now_us - r->gate_on_us >= ACE2K_HEAT_GATE_PULSE_US) {
        r->gate = false;
    }
}

static inline void rig_edges(struct rig *r, uint32_t end_us)
{
    while ((int32_t)(end_us - r->next_edge_us) > 0) {
        rig_drop_gate(r, r->next_edge_us);
        r->edge_us = r->next_edge_us;
        ace2k_heat_zerocross(&r->heat, r->next_edge_us);
        r->next_edge_us += RIG_HALF_CYCLE_US;
    }
    rig_drop_gate(r, end_us);
}

/* The model advances every 100 ms with the fraction of half-cycles heat fired in that time. */
static inline void rig_plant(struct rig *r)
{
    if (r->now_ms - r->plant_ms < RIG_PLANT_MS) {
        return;
    }
    r->plant_ms = r->now_ms;
    uint32_t delta = r->heat.fired - r->fired_mark;
    r->fired_mark = r->heat.fired;
    uint32_t duty = delta * 100U / RIG_EDGES_PER_PLANT;
    if (duty > 100U) {
        duty = 100U;
    }
    if (r->heater_dead) {
        duty = 0U;
    }
    unsigned fan_mask = 0U;
    if (r->fans_on && !r->fan_fail) {
        fan_mask = FAKE_THERMAL_FANS_BOTH & ~(unsigned)r->plant_blocked;
    }
    fake_thermal_step(&r->plant, duty, fan_mask, RIG_PLANT_MS);
}

static inline void rig_readings(const struct rig *r, struct ace2k_dryer_inputs *in)
{
    memset(in, 0, sizeof *in);
    in->ntc_left_mc = r->ovr_ntc ? r->ovr_left_mc : fake_thermal_ntc_mc(&r->plant, 0U);
    in->ntc_right_mc = r->ovr_ntc ? r->ovr_right_mc : fake_thermal_ntc_mc(&r->plant, 1U);
    in->left_valid = true;
    in->right_valid = true;
    if (r->ovr_ntc) {
        in->left_valid = r->ovr_left_valid;
        in->right_valid = r->ovr_right_valid;
    }
    in->chamber_mc = r->ovr_chamber ? r->ovr_chamber_mc : fake_thermal_chamber_mc(&r->plant);
    in->chamber_valid = r->chamber_valid;
    in->chamber_age_ms = r->chamber_age_ms;
    in->mains_present = r->mains_present;
    in->mains_plausible = r->mains_plausible;
    in->cutout = r->cutout;
    in->link_ok = r->link_ok;
    in->insert_mask = r->insert_mask;
    in->chamber_rh_pct10 = r->rh_pct10;
    in->rh_valid = r->rh_valid;
}

static inline void rig_drain(struct rig *r)
{
    struct ace2k_dryer_event ev;
    while (ace2k_dryer_event_next(&r->dryer, &ev)) {
        if (r->ev_n < RIG_EV_MAX) {
            r->ev[r->ev_n++] = ev;
        }
        ace2k_dryer_event_sent(&r->dryer);
        ace2k_dryer_event_ack(&r->dryer, ev.seq);
    }
}

static inline void rig_track(struct rig *r, const struct ace2k_dryer_inputs *in)
{
    if (in->left_valid && in->ntc_left_mc > r->max_ntc_mc) {
        r->max_ntc_mc = in->ntc_left_mc;
    }
    if (in->right_valid && in->ntc_right_mc > r->max_ntc_mc) {
        r->max_ntc_mc = in->ntc_right_mc;
    }
    if (r->dryer.state == ACE2K_DRYER_HEATING) {
        int32_t drive = ace2k_dryer_drive_mc(&r->dryer);
        if (drive > r->max_drive_mc) {
            r->max_drive_mc = drive;
        }
    }
    uint32_t duty =
        (r->dryer.ctl.duty_cpct + ACE2K_DRYER_CPCT_PER_PCT - 1U) / ACE2K_DRYER_CPCT_PER_PCT;
    if (r->heat.duty_pct > duty) {
        duty = r->heat.duty_pct;
    }
    if (duty > r->max_duty_pct) {
        r->max_duty_pct = duty;
    }
}

/* A flap whose coil is driven is travelling: its published position must be unknown. */
static inline void rig_check_flaps(struct rig *r)
{
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        if (r->flap_coil_on[f] &&
            ace2k_airflow_flap_pos(&r->airflow, (enum ace2k_flap)f) != ACE2K_FLAP_UNKNOWN) {
            r->flap_claimed = true;
        }
    }
}

/* Before a tick: the counts a state entered by it starts from. */
static inline void rig_mark_entry(struct rig *r)
{
    r->entry_pulses = r->flap_pulses;
    for (unsigned f = 0; f < ACE2K_FLAP_COUNT; f++) {
        r->entry_opens[f] = r->flap_opens[f];
        r->entry_closes[f] = r->flap_closes[f];
    }
}

/* What the mains measured: nothing yet (a boot), the train's frequency, or off-band. */
static inline uint16_t rig_hz10(const struct rig *r, const struct ace2k_dryer_inputs *in)
{
    if (r->mains_unmeasured) {
        return 0U;
    }
    return in->mains_plausible ? RIG_MAINS_HZ10 : RIG_MAINS_HZ10_OFF_BAND;
}

/* The mains' measured flag as the binding reads it: a window measured, the mains present.  The
 * rig models no measurement window: measured simply follows present — a return reads measured
 * at once, unlike mains.c — unless the test sets mains_unmeasured by hand (a boot, or the window
 * after an outage). */
static inline bool rig_mains_measured(const struct rig *r, const struct ace2k_dryer_inputs *in)
{
    if (r->mains_unmeasured) {
        return false;
    }
    return in->mains_present;
}

static inline void rig_tick(struct rig *r)
{
    uint8_t state_before = r->dryer.state;
    rig_mark_entry(r);
    rig_edges(r, r->now_us + RIG_TICK_US);
    r->now_us += RIG_TICK_US;
    r->now_ms += RIG_TICK_MS;
    rig_plant(r);
    struct ace2k_dryer_inputs in;
    rig_readings(r, &in);
    r->airflow.heat_holds = ace2k_heat_active(&r->heat); /* as the binding, before airflow */
    ace2k_airflow_tick(&r->airflow, in.ntc_left_mc, in.ntc_right_mc, in.left_valid, in.right_valid,
                       r->now_ms);
    struct ace2k_heat_inputs hi = {
        .ntc_left_mc = in.ntc_left_mc,
        .ntc_right_mc = in.ntc_right_mc,
        .left_valid = in.left_valid,
        .right_valid = in.right_valid,
        .mains_present = in.mains_present,
        .mains_hz10 = rig_hz10(r, &in),
        .mains_measured = rig_mains_measured(r, &in),
        .cutout = in.cutout,
        .fans_read = ace2k_airflow_fans_read(&r->airflow),
        .fans_commanded = ace2k_airflow_fans_commanded(&r->airflow),
    };
    ace2k_heat_tick(&r->heat, &hi, r->now_ms, r->now_us);
    rig_check_flaps(r);
    if (!r->dryer_frozen) {
        ace2k_dryer_tick(&r->dryer, &in, r->now_ms);
    }
    rig_check_flaps(r);
    r->entered_by_tick = r->dryer.state != state_before;
    if (ace2k_dryer_log_dirty(&r->dryer) && !ace2k_heat_busy(&r->heat)) {
        r->stores++;
        ace2k_dryer_log_stored(&r->dryer, ace2k_dryer_log_generation(&r->dryer));
    }
    rig_drain(r);
    rig_track(r, &in);
    r->last_in = in;
}

static inline void rig_run_ms(struct rig *r, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += RIG_TICK_MS) {
        rig_tick(r);
    }
}

/* Runs until the dryer is in `state`, at most max_ms; true when it got there. */
static inline bool rig_run_until(struct rig *r, uint8_t state, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += RIG_TICK_MS) {
        if (r->dryer.state == state) {
            return true;
        }
        rig_tick(r);
    }
    return r->dryer.state == state;
}

static inline uint32_t rig_events(const struct rig *r, uint8_t kind)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < r->ev_n; i++) {
        if (r->ev[i].kind == kind) {
            n++;
        }
    }
    return n;
}

/* A powered unit at `ambient_mc`, one second of ticks behind it (every input published). */
static inline void rig_init(struct rig *r, int32_t ambient_mc)
{
    memset(r, 0, sizeof *r);
    fake_thermal_init(&r->plant, ambient_mc);
    ace2k_airflow_init(&r->airflow, &rig_airflow_ops, r);
    ace2k_heat_init(&r->heat, &rig_heat_ops, r);
    ace2k_dryer_init(&r->dryer, &r->heat, &r->airflow, &r->log, true, 0U);
    r->mains_present = true;
    r->mains_plausible = true;
    r->link_ok = true;
    r->chamber_valid = true;
    r->rh_pct10 = 300U;
    r->rh_valid = true;
    r->next_edge_us = RIG_HALF_CYCLE_US / 2U;
    r->max_ntc_mc = INT32_MIN;
    r->max_drive_mc = INT32_MIN;
    rig_run_ms(r, 1000U);
}

/* Starts a cycle and runs until HEATING (at most the start timeout). */
static inline enum ace2k_dryer_refusal rig_start(struct rig *r, uint8_t target_c, uint16_t minutes)
{
    enum ace2k_dryer_refusal res = ace2k_dryer_start(&r->dryer, target_c, minutes, r->now_ms);
    if (res == ACE2K_DRYER_OK) {
        (void)rig_run_until(r, ACE2K_DRYER_HEATING, ACE2K_DRYER_START_TIMEOUT_MS + 100U);
    }
    return res;
}

#endif
