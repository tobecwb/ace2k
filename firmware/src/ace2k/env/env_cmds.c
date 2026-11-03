// Binding of the env module.
// - Every tick, it feeds the NTC millivolts and the raw reference from the ADC scan.
// - A task runs the chamber sensor's transaction over Klipper's software I²C. The bus
//   is set up here, without a host oid, because the dryer needs the chamber reading with no host.
// - The same task sends the periodic report.
// The tick and the task share the instance in the same way as in the sensors binding.
#include "env/env_cmds.h"
#include "ace2k_board/adc_scan.h"
#include "ace2k_board/pins.h"
#include "ace2k_board/tick.h"
#include "ace2k_board/tx.h"   // ACE2K_SENDF
#include "core/report_cmds.h" // ace2k_report_query
#include "core/util.h"
#include "board/irq.h"    // irq_save, irq_restore
#include "board/misc.h"   // timer_from_us
#include "command.h"      // DECL_COMMAND, sendf, shutdown, DECL_CONSTANT_STR
#include "i2c_software.h" // i2c_software_setup, i2c_software_write, i2c_software_read
#include "i2ccmds.h"      // I2C_BUS_SUCCESS
#include "sched.h"        // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake

// The host reserves every RESERVE_PINS_* constant, so no [i2c] section can claim the bus of the
// chamber sensor. pins.h names the pair (docs/hardware.md "Chamber sensor").
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_env", "PE14,PE15");

// env counts its stale limit in scan periods, and the age it gets comes in tick steps.
_Static_assert(ACE2K_ENV_ADC_SCAN_PERIOD_MS == ACE2K_ADC_SCAN_PERIOD_MS,
               "the env stale limit is counted in scan periods");
_Static_assert(ACE2K_TICK_PERIOD_MS == ACE2K_ADC_SCAN_PERIOD_MS,
               "the age's step is the tick, one scan period");

#define ACE2K_ENV_I2C_HALF_PERIOD_US 5U /* 100 kHz */
#define ACE2K_ENV_I2C_MAX_WRITE      4U
#define ACE2K_ENV_REPORT_PHASE_MS    30U /* the phase: report.h */

static struct i2c_software *ace2k_env_bus;
static struct ace2k_env ace2k_env_instance;
static struct task_wake ace2k_env_wake;
static struct ace2k_report ace2k_env_report;
static volatile bool ace2k_env_report_due;
static bool ace2k_env_ready;

// Named after the bus, not the operation, because Klipper's board/gpio.h (behind i2ccmds.h)
// already declares i2c_write() and i2c_read() for its hardware I²C.
static int bus_write(void *ctx, uint8_t addr, const uint8_t *data, uint8_t len)
{
    (void)ctx;
    (void)addr; /* the bus was set up for ACE2K_ENV_AHT_ADDR */
    uint8_t buf[ACE2K_ENV_I2C_MAX_WRITE];
    if (len > ACE2K_ENV_I2C_MAX_WRITE) {
        return -ACE2K_EINVAL;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = data[i];
    }
    return i2c_software_write(ace2k_env_bus, len, buf) == I2C_BUS_SUCCESS ? 0 : -ACE2K_EIO;
}

static int bus_read(void *ctx, uint8_t addr, uint8_t *data, uint8_t len)
{
    (void)ctx;
    (void)addr;
    return i2c_software_read(ace2k_env_bus, 0, NULL, len, data) == I2C_BUS_SUCCESS ? 0 : -ACE2K_EIO;
}

static const struct ace2k_env_ops board_ops = {
    .i2c_write = bus_write,
    .i2c_read = bus_read,
};

static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    uint16_t raw[ACE2K_ADC_COUNT];
    // The age is the time since the last pass completed, on the tick clock (adc_scan.h). Until
    // the first pass, env keeps its NTCs invalid from init.
    uint32_t age_ms = 0;
    if (ace2k_adc_scan_latest(raw, &age_ms)) {
        ace2k_env_tick(&ace2k_env_instance, now_ms, age_ms,
                       ace2k_adc_raw_to_mv(raw[ACE2K_ADC_NTC_LEFT]),
                       ace2k_adc_raw_to_mv(raw[ACE2K_ADC_NTC_RIGHT]), raw[ACE2K_ADC_VREFINT]);
    }
    // Wake for a step of the chamber state machine or a due report, as the ticks of the other
    // bindings do. Not every 10 ms.
    bool wake = ace2k_env_task_due(&ace2k_env_instance, now_ms);
    if (ace2k_report_due(&ace2k_env_report, now_ms)) {
        ace2k_env_report_due = true;
    }
    // A report that the last pass held, because its frame did not fit, leaves on this one.
    if (wake || ace2k_env_report_due) {
        sched_wake_task(&ace2k_env_wake);
    }
}

// True when the report was queued. False when it did not fit, and its flag holds it for the next
// tick (tx.h; report.h gives the order).
static bool report(void)
{
    const struct ace2k_env *e = &ace2k_env_instance;
    return ACE2K_SENDF("ace2k_env_state ptc_left_mc=%i ptc_right_mc=%i chamber_mc=%i"
                       " chamber_rh_pct10=%hu vdda_mv=%hu valid=%c",
                       e->ptc_left_mc, e->ptc_right_mc, e->chamber_mc, e->chamber_rh_pct10,
                       e->vdda_mv, e->valid);
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static. The task
// function of the core is ace2k_env_task(); this one belongs to the binding.
void ace2k_env_binding_task(void)
{
    if (!sched_check_wake(&ace2k_env_wake)) {
        return;
    }
    ace2k_env_task(&ace2k_env_instance, ace2k_tick_now_ms());
    if (ace2k_env_report_due) {
        (void)ace2k_report_try(&ace2k_env_report_due, report);
    }
}
DECL_TASK(ace2k_env_binding_task);

static void ensure_ready(void)
{
    if (ace2k_env_ready) {
        return;
    }
    ace2k_env_bus =
        i2c_software_setup(ACE2K_PIN_I2C_SCL, ACE2K_PIN_I2C_SDA,
                           timer_from_us(ACE2K_ENV_I2C_HALF_PERIOD_US), ACE2K_ENV_AHT_ADDR);
    // The AHT20 wants 40 ms of power before its first command (Aosong datasheet). That is met:
    // the bootloader ran long before this init, and a software reset keeps the sensor powered.
    ace2k_env_init(&ace2k_env_instance, &board_ops, NULL, ace2k_tick_now_ms());
    // A full registry is a programming error. From init context this ends in a fault and a
    // watchdog reset. Either way it is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_env_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_env_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_env_binding_init);

const struct ace2k_env *ace2k_env_binding(void)
{
    ensure_ready();
    return &ace2k_env_instance;
}

// Klipper's generated dispatch declares every handler as void (*)(uint32_t *).
// cppcheck-suppress constParameterPointer
void ace2k_env_cmd_query(uint32_t *args)
{
    ensure_ready();
    ace2k_report_query(&ace2k_env_report, &ace2k_env_report_due, args[0],
                       ACE2K_ENV_REPORT_PHASE_MS);
}
DECL_COMMAND(ace2k_env_cmd_query, "ace2k_env_query rest_ticks=%u");
