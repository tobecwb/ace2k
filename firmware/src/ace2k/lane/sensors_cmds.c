// Binding of the sensors module.  Every tick, it feeds the last ADC pass and the ten digital levels
// to the core.  A task sends the events and the periodic report, because sendf never runs in timer
// context.  The bands come from the factory page at init, and ace2k_sensors_thresholds_set
// overrides them.
// The tick and the task share the instance.  The ring has one producer, the tick, and one consumer,
// the task.  The report reads fields that the tick writes.  A torn read costs one stale value in
// one report, never a fault.
#include "lane/sensors_cmds.h"
#include "core/config_cmds.h"
#include "ace2k_board/adc_scan.h"
#include "ace2k_board/args.h" // ACE2K_ARG_U16
#include "lane/lane_args.h"   // ACE2K_ARG_LANE
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "core/report_cmds.h" // ace2k_report_query
#include "ace2k_board/tx.h"   // ACE2K_SENDF
#include "core/util.h"        // ace2k_put_u16_le
#include "board/gpio.h"       // gpio_in_setup, gpio_in_read
#include "board/irq.h"        // irq_save, irq_restore
#include "command.h"          // DECL_COMMAND, sendf, shutdown, DECL_CONSTANT_STR
#include "sched.h"            // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake

// The host reserves every RESERVE_PINS_* constant, so a [gcode_button] cannot claim a switch input.
// pins.h names the pins (docs/hardware.md "Digital inputs").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_sensors", "PB4,PB5,PD7,PE0,PC12,PD0,PD1,PD2,PE1,PD15");

// The core tells a stall's catch-up by the tick's period.
_Static_assert(ACE2K_SENSORS_TICK_PERIOD_MS == ACE2K_TICK_PERIOD_MS,
               "the sensors' tick period is the board's");

#define ACE2K_SENSORS_AGE_MAX         255U
#define ACE2K_SENSORS_REPORT_PHASE_MS 0U /* the report phase (report.h) */
// Every debounced edge is a frame.  The unit holds its replies while a burst from the host is still
// arriving (the link turnaround).  So a burst of edges plus the due report can exceed the transmit
// buffer of Klipper.  Every frame of this task is tried, never dropped (tx.h, report.h).  A frame
// that Klipper refuses is held for the next tick.  The report is held by its flag, and an event at
// the ring's tail.  The wake of the next tick brings the task back, because the tick is the only
// waker of the task.  The due report goes first, then the events, oldest first, while they fit.  An
// edge leaves on the first tick where its frame fits.  Normally that is the next one, within 10 ms.
#define ACE2K_SENSORS_EVENT_FMT "ace2k_sensors_event lane=%c kind=%c level=%c"

// docs/hardware.md "Digital inputs": the Hall switches are active low.  A pull-up cannot hurt an
// open-collector output.  This is an initial value, because the input mode of the factory firmware
// is not recorded.  The cutout input idles low as a floating input, and is left floating.
static const struct {
    uint8_t pin;
    int8_t pull_up;
} inputs_cfg[ACE2K_SENSORS_IN_COUNT] = {
    [ACE2K_SENSORS_IN_REST1] = { ACE2K_PIN_REST1, 1 },
    [ACE2K_SENSORS_IN_REST1 + 1] = { ACE2K_PIN_REST2, 1 },
    [ACE2K_SENSORS_IN_REST1 + 2] = { ACE2K_PIN_REST3, 1 },
    [ACE2K_SENSORS_IN_REST1 + 3] = { ACE2K_PIN_REST4, 1 },
    [ACE2K_SENSORS_IN_PUSHED1] = { ACE2K_PIN_PUSHED1, 1 },
    [ACE2K_SENSORS_IN_PUSHED1 + 1] = { ACE2K_PIN_PUSHED2, 1 },
    [ACE2K_SENSORS_IN_PUSHED1 + 2] = { ACE2K_PIN_PUSHED3, 1 },
    [ACE2K_SENSORS_IN_PUSHED1 + 3] = { ACE2K_PIN_PUSHED4, 1 },
    [ACE2K_SENSORS_IN_PULLED] = { ACE2K_PIN_PULLED, 1 },
    [ACE2K_SENSORS_IN_CUTOUT] = { ACE2K_PIN_CUTOUT, 0 },
};

static struct gpio_in ace2k_sensors_gpio[ACE2K_SENSORS_IN_COUNT];
static struct ace2k_sensors ace2k_sensors_instance;
static struct task_wake ace2k_sensors_wake;
static struct ace2k_report ace2k_sensors_report;
static volatile bool ace2k_sensors_report_due;
static bool ace2k_sensors_ready;

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    // The rest zeroed first.  ace2k_adc_scan_latest() leaves the age untouched while no pass
    // exists, and the core must never see an undefined age.
    struct ace2k_sensors_inputs in = { .now_ms = now_ms };
    uint16_t raw[ACE2K_ADC_COUNT];
    in.adc_valid = ace2k_adc_scan_latest(raw, &in.adc_age_ms);
    for (uint32_t i = 0; i < ACE2K_ADC_COUNT; i++) {
        in.mv[i] = in.adc_valid ? ace2k_adc_raw_to_mv(raw[i]) : 0;
    }
    for (uint32_t i = 0; i < ACE2K_SENSORS_IN_COUNT; i++) {
        if (gpio_in_read(ace2k_sensors_gpio[i])) {
            in.levels |= (uint16_t)(1U << i);
        }
    }
    ace2k_sensors_tick(&ace2k_sensors_instance, &in);
    if (ace2k_report_due(&ace2k_sensors_report, now_ms)) {
        ace2k_sensors_report_due = true;
    }
    // a report that the last pass held, because its frame did not fit, leaves now
    if (ace2k_sensors_has_events(&ace2k_sensors_instance) || ace2k_sensors_report_due) {
        sched_wake_task(&ace2k_sensors_wake);
    }
}

// Returns true when the report was queued.  It returns false when the report did not fit, and then
// its flag holds it for the next tick.  The arrays are packed before the frame is tried, because a
// try needs the encoded frame.
static bool report(void)
{
    const struct ace2k_sensors *s = &ace2k_sensors_instance;
    uint8_t insert_mv[2 * ACE2K_LANE_COUNT];
    uint8_t empty_mv[2 * ACE2K_LANE_COUNT];
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_put_u16_le(insert_mv + (2 * i), s->lane[i].insert_mv);
        ace2k_put_u16_le(empty_mv + (2 * i), s->lane[i].empty_mv);
    }
    uint32_t age = s->adc_age_ms > ACE2K_SENSORS_AGE_MAX ? ACE2K_SENSORS_AGE_MAX : s->adc_age_ms;
    return ACE2K_SENDF(
        "ace2k_sensors_state switches=%hu insert_mv=%*s empty_mv=%*s aux_mv=%hu age_ms=%c",
        ace2k_sensors_switches(s), (uint8_t)sizeof insert_mv, insert_mv, (uint8_t)sizeof empty_mv,
        empty_mv, s->aux_mv, (uint8_t)age);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
void ace2k_sensors_task(void)
{
    if (!sched_check_wake(&ace2k_sensors_wake)) {
        return;
    }
    // The due report goes first, then the events, oldest first.  Each event is tried from the
    // ring's tail, and dropped from it only once queued.
    if (ace2k_sensors_report_due) {
        (void)ace2k_report_try(&ace2k_sensors_report_due, report);
    }
    struct ace2k_sensors_event ev;
    while (ace2k_sensors_peek_event(&ace2k_sensors_instance, &ev)) {
        if (!ACE2K_SENDF(ACE2K_SENSORS_EVENT_FMT, ev.lane, ev.kind, ev.level)) {
            break;
        }
        ace2k_sensors_drop_event(&ace2k_sensors_instance);
    }
}
DECL_TASK(ace2k_sensors_task);

static void ensure_ready(void)
{
    if (ace2k_sensors_ready) {
        return;
    }
    for (uint32_t i = 0; i < ACE2K_SENSORS_IN_COUNT; i++) {
        ace2k_sensors_gpio[i] = gpio_in_setup(inputs_cfg[i].pin, inputs_cfg[i].pull_up);
    }
    ace2k_sensors_init(&ace2k_sensors_instance, ace2k_config_binding_factory()->insert);
    // A full registry is a programming error.  From init context this ends in a fault and a
    // watchdog reset.  Either way, the failure is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_sensors_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_sensors_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_sensors_binding_init);

const struct ace2k_sensors *ace2k_sensors_binding(void)
{
    ensure_ready();
    return &ace2k_sensors_instance;
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_sensors_cmd_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_sensors_report, &ace2k_sensors_report_due, args[0],
                       ACE2K_SENSORS_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_sensors_cmd_query, "ace2k_sensors_query rest_ticks=%u");

// A malformed threshold is a bug in the host, not a condition of the sensor.  Klipper's way to
// refuse a bad configuration command is a shutdown with a message.  So is a value too wide for its
// field: a lane above 3, or a millivolt value above 65535.
// cppcheck-suppress constParameterPointer
void ace2k_sensors_cmd_thresholds_set(uint32_t *args)
{
    ensure_ready();
    uint8_t lane = ACE2K_ARG_LANE(
        args[0], "ace2k_sensors_thresholds_set: lane out of range or low_mv >= high_mv");
    uint16_t low_mv = ACE2K_ARG_U16(args[1], "ace2k_sensors_thresholds_set: low_mv out of range");
    uint16_t high_mv = ACE2K_ARG_U16(args[2], "ace2k_sensors_thresholds_set: high_mv out of range");
    // The tick reads the band in the timer interrupt.  The three stores run in one masked step, so
    // the tick never sees low above high.
    irqstatus_t flag = irq_save();
    int rc =
        ace2k_sensors_set_band(&ace2k_sensors_instance, lane, low_mv, high_mv, ACE2K_CAL_CONFIG);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_sensors_thresholds_set: lane out of range or low_mv >= high_mv");
    }
}
DECL_COMMAND(ace2k_sensors_cmd_thresholds_set,
             "ace2k_sensors_thresholds_set lane=%c low_mv=%hu high_mv=%hu");

// Sets the lane's band back to the one the unit started with, from its factory page or the default,
// with that source.  The host sends it at every connect, as an init command, for a lane with no
// laneN_insert_mv.  So removing the key needs no MCU restart.  A lane out of range is a bug in the
// host.
// cppcheck-suppress constParameterPointer
void ace2k_sensors_cmd_thresholds_reset(uint32_t *args)
{
    ensure_ready();
    const uint8_t lane =
        ACE2K_ARG_LANE(args[0], "ace2k_sensors_thresholds_reset: lane out of range");
    const struct ace2k_insert_band *start = &ace2k_config_binding_factory()->insert[lane];
    irqstatus_t flag = irq_save();
    int rc = ace2k_sensors_set_band(&ace2k_sensors_instance, lane, start->low_mv, start->high_mv,
                                    start->source);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_sensors_thresholds_reset: the starting band is invalid");
    }
}
DECL_COMMAND(ace2k_sensors_cmd_thresholds_reset, "ace2k_sensors_thresholds_reset lane=%c");

void ace2k_sensors_cmd_thresholds_query(uint32_t *args)
{
    (void)args;
    ensure_ready();
    // One frame carries the four lanes, not one frame per lane.  The transmit buffer holds
    // 96 bytes, and a frame that does not fit is dropped silently.  So a burst of frames in one
    // task pass would lose its tail.  low_mv and high_mv are four uint16, little-endian.  source is
    // four uint8.
    uint8_t low_mv[2 * ACE2K_LANE_COUNT];
    uint8_t high_mv[2 * ACE2K_LANE_COUNT];
    uint8_t source[ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        const struct ace2k_insert_band *b = ace2k_sensors_band(&ace2k_sensors_instance, lane);
        ace2k_put_u16_le(low_mv + (2 * lane), b->low_mv);
        ace2k_put_u16_le(high_mv + (2 * lane), b->high_mv);
        source[lane] = (uint8_t)b->source;
    }
    sendf("ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s", (uint8_t)sizeof low_mv,
          low_mv, (uint8_t)sizeof high_mv, high_mv, (uint8_t)sizeof source, source);
}
DECL_COMMAND(ace2k_sensors_cmd_thresholds_query, "ace2k_sensors_thresholds_query");
