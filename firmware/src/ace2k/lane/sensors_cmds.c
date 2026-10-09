// Binding of the sensors module: the last ADC pass and the ten digital levels every tick; the
// events and the periodic report sent from a task (sendf never runs in timer context); the
// bands from the factory page at init, overridden by ace2k_sensors_thresholds_set.
// The tick and the task share the instance: the ring is single-producer (tick) /
// single-consumer (task); the report reads fields the tick writes — a torn read costs one
// stale value in one report, never a fault.
#include "lane/sensors_cmds.h"
#include "core/config_cmds.h"
#include "ace2k_board/adc_scan.h"
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "core/report.h"
#include "core/tx.h"    // ACE2K_SENDF
#include "core/util.h"  // ace2k_put_u16_le
#include "board/gpio.h" // gpio_in_setup, gpio_in_read
#include "board/irq.h"  // irq_save, irq_restore
#include "board/misc.h" // timer_from_us
#include "command.h"    // DECL_COMMAND, sendf, shutdown, DECL_CONSTANT_STR
#include "sched.h"      // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake

// The host reserves every RESERVE_PINS_* constant, so a [gcode_button] cannot claim a switch
// input (pins.h names them; docs/hardware.md "Digital inputs").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_sensors", "PB4,PB5,PD7,PE0,PC12,PD0,PD1,PD2,PE1,PD15");

#define ACE2K_SENSORS_AGE_MAX         255U
#define ACE2K_SENSORS_REPORT_PHASE_MS 0U /* the phase: report.h */
// Every debounced edge is a frame, and the unit holds its replies while a host burst is still
// arriving (the link turnaround), so a burst of edges plus the due report can exceed Klipper's
// transmit buffer.  Every frame of this task is tried, never dropped (tx.h, report.h): a frame
// Klipper refuses is held — the report by its flag, an event at the ring's tail — for the next
// tick, whose wake brings the task back (the tick is the task's only waker).  The due report
// goes first, then the events, oldest first, while they fit; an edge leaves on the first tick
// its frame fits, normally the next one, within 10 ms.
#define ACE2K_SENSORS_EVENT_FMT "ace2k_sensors_event lane=%c kind=%c level=%c"

// docs/hardware.md "Digital inputs": the Hall switches are active low — a pull-up cannot hurt
// an open-collector output (initial value; the factory firmware's input mode is not recorded);
// the cutout input idles low as a floating input and is left floating.
static const struct {
    uint8_t pin;
    int8_t pull_up;
} ace2k_sensors_inputs_cfg[ACE2K_SENSORS_IN_COUNT] = {
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
    // zeroed first: ace2k_adc_scan_latest() leaves the age untouched while no pass exists, and
    // the core must never see an indeterminate one
    struct ace2k_sensors_inputs in = { 0 };
    uint16_t raw[ACE2K_ADC_COUNT];
    in.adc_valid = ace2k_adc_scan_latest(raw, &in.adc_age_ms);
    for (uint8_t i = 0; i < ACE2K_ADC_COUNT; i++) {
        in.mv[i] = in.adc_valid ? ace2k_adc_raw_to_mv(raw[i]) : 0;
    }
    for (uint8_t i = 0; i < ACE2K_SENSORS_IN_COUNT; i++) {
        if (gpio_in_read(ace2k_sensors_gpio[i])) {
            in.levels |= (uint16_t)(1U << i);
        }
    }
    ace2k_sensors_tick(&ace2k_sensors_instance, now_ms, &in);
    if (ace2k_report_due(&ace2k_sensors_report, now_ms)) {
        ace2k_sensors_report_due = true;
    }
    // a report the last pass held (its frame did not fit) leaves on this one
    if (ace2k_sensors_has_events(&ace2k_sensors_instance) || ace2k_sensors_report_due) {
        sched_wake_task(&ace2k_sensors_wake);
    }
}

// True when the report was queued; false when it did not fit and its flag holds it for the next
// tick.  The arrays are packed before the frame is tried: a try needs the encoded frame.
static bool report(void)
{
    const struct ace2k_sensors *s = &ace2k_sensors_instance;
    uint8_t insert_mv[2 * ACE2K_LANE_COUNT];
    uint8_t empty_mv[2 * ACE2K_LANE_COUNT];
    for (uint8_t i = 0; i < ACE2K_LANE_COUNT; i++) {
        ace2k_put_u16_le(insert_mv + 2 * i, s->lane[i].insert_mv);
        ace2k_put_u16_le(empty_mv + 2 * i, s->lane[i].empty_mv);
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
    // the due report first, then the events, oldest first, each tried from the ring's tail and
    // dropped from it only once queued
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
    for (uint8_t i = 0; i < ACE2K_SENSORS_IN_COUNT; i++) {
        ace2k_sensors_gpio[i] =
            gpio_in_setup(ace2k_sensors_inputs_cfg[i].pin, ace2k_sensors_inputs_cfg[i].pull_up);
    }
    ace2k_sensors_init(&ace2k_sensors_instance, ace2k_config_binding_factory()->insert);
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
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
    uint32_t rest_ms = args[0] / timer_from_us(1000U);
    // The tick reads the pair in the timer interrupt; both stores in one step, so it never
    // sees the new period against the old deadline.
    irqstatus_t flag = irq_save();
    ace2k_report_set(&ace2k_sensors_report, rest_ms, ace2k_tick_now_ms(),
                     ACE2K_SENSORS_REPORT_PHASE_MS);
    ace2k_sensors_report_due = false; // a report held from before the query: the new grid's
    irq_restore(flag);
}
DECL_COMMAND(ace2k_sensors_cmd_query, "ace2k_sensors_query rest_ticks=%u");

// A malformed threshold is a host bug, not a sensor condition: Klipper's idiom for a bad
// configuration command is a shutdown with a message.
// cppcheck-suppress constParameterPointer
void ace2k_sensors_cmd_thresholds_set(uint32_t *args)
{
    ensure_ready();
    // The tick reads the band in the timer interrupt; the three stores in one step, so it
    // never sees low above high.
    irqstatus_t flag = irq_save();
    int rc = ace2k_sensors_set_band(&ace2k_sensors_instance, (uint8_t)args[0], (uint16_t)args[1],
                                    (uint16_t)args[2], ACE2K_CAL_CONFIG);
    irq_restore(flag);
    if (rc != 0) {
        shutdown("ace2k_sensors_thresholds_set: lane out of range or low_mv >= high_mv");
    }
}
DECL_COMMAND(ace2k_sensors_cmd_thresholds_set,
             "ace2k_sensors_thresholds_set lane=%c low_mv=%hu high_mv=%hu");

void ace2k_sensors_cmd_thresholds_query(uint32_t *args)
{
    (void)args;
    ensure_ready();
    // One frame for the four lanes, not one per lane: the transmit buffer holds 96 bytes and
    // a frame that does not fit is dropped silently, so a burst of frames in one task pass
    // loses its tail.  low_mv, high_mv: four uint16 little-endian; source: four uint8.
    uint8_t low_mv[2 * ACE2K_LANE_COUNT];
    uint8_t high_mv[2 * ACE2K_LANE_COUNT];
    uint8_t source[ACE2K_LANE_COUNT];
    for (uint8_t lane = 0; lane < ACE2K_LANE_COUNT; lane++) {
        const struct ace2k_insert_band *b = ace2k_sensors_band(&ace2k_sensors_instance, lane);
        ace2k_put_u16_le(low_mv + 2 * lane, b->low_mv);
        ace2k_put_u16_le(high_mv + 2 * lane, b->high_mv);
        source[lane] = (uint8_t)b->source;
    }
    sendf("ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s", (uint8_t)sizeof low_mv,
          low_mv, (uint8_t)sizeof high_mv, high_mv, (uint8_t)sizeof source, source);
}
DECL_COMMAND(ace2k_sensors_cmd_thresholds_query, "ace2k_sensors_thresholds_query");
