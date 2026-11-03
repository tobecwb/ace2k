// Binding of heat: the board's gate as ops, the ordered tick, the zero-cross hook, the vetoes
// and the shutdown. The tick and the zero-cross interrupt share the instance under the contract of
// heat.h.
#include "heat/heat_cmds.h"
#include "heat/airflow_cmds.h"
#include "core/config_cmds.h"
#include "env/env_cmds.h"
#include "core/guard_cmds.h"
#include "heat/mains_cmds.h"
#include "lane/sensors_cmds.h"
#include "ace2k_board/gate.h"
#include "ace2k_board/tick.h"
#include "ace2k_board/zerocross.h"
#include "command.h" // DECL_CONSTANT_STR, shutdown
#include "sched.h"   // DECL_INIT, DECL_SHUTDOWN, sched_is_shutdown

// The host reserves every RESERVE_PINS_* constant, so no [output_pin] can claim the gate (PC8).
// An image without the heater reserves PC8 in ace2k_board/tick.c instead. The cutout's latch-reset
// pin (PC9) is reserved in every image by ace2k_board/tick.c.
DECL_CONSTANT_STR("RESERVE_PINS_ace2k_heat", "PC8");

static struct ace2k_heat ace2k_heat_instance;
static struct ace2k_airflow *ace2k_heat_air;
static const struct ace2k_env *ace2k_heat_env;
static const struct ace2k_mains *ace2k_heat_mains;
static const struct ace2k_sensors *ace2k_heat_sensors;
static bool ace2k_heat_ready;
static struct ace2k_heat_world ace2k_heat_world_now;
static ace2k_heat_after_fn volatile ace2k_heat_after;

static void gate_fire(void *ctx)
{
    (void)ctx;
    ace2k_gate_fire();
}

static void gate_off(void *ctx)
{
    (void)ctx;
    ace2k_gate_off();
}

static bool gate_read(void *ctx)
{
    (void)ctx;
    return ace2k_gate_read();
}

static const struct ace2k_heat_ops board_ops = {
    .gate_fire = gate_fire,
    .gate_off = gate_off,
    .gate_read = gate_read,
};

// The zero-cross interrupt, after the lockout. The hook hands over the Klipper timer ticks read
// at the handler's entry. heat gets them on the gate's wrap-safe microsecond clock. So
// the half-period it measures between two edges (heat.h) is between their interrupt entries,
// whatever the handler spent before the call. The delay from an edge to its entry remains.
// A resync edge is the one that the pending bit held through a flash stall, timed late. It first
// restarts heat's edge history (ace2k_heat_resync in heat.h).
static void edge(uint32_t now_ticks, bool resync)
{
    if (sched_is_shutdown()) {
        return;
    }
    if (resync) {
        ace2k_heat_resync(&ace2k_heat_instance);
    }
    ace2k_heat_zerocross(&ace2k_heat_instance, ace2k_gate_us_at(now_ticks));
}

// Reads the world once per tick, for heat and the dryer's hook alike: one reading, one place.
static void read_world(struct ace2k_heat_world *w)
{
    const struct ace2k_env *env = ace2k_heat_env;
    const struct ace2k_mains *mains = ace2k_heat_mains;
    // env_step() in tests/heat/test_heat.c copies the NTC lines below. Change both.
    w->left_valid = ace2k_env_valid_bit(env, ACE2K_ENV_VALID_NTC_LEFT);
    w->right_valid = ace2k_env_valid_bit(env, ACE2K_ENV_VALID_NTC_RIGHT);
    if (w->left_valid) {
        w->ntc_left_mc = env->ptc_left_mc;
    }
    if (w->right_valid) {
        w->ntc_right_mc = env->ptc_right_mc;
    }
    w->mains_present = mains->present;
    w->mains_plausible = ace2k_mains_plausible(mains);
    w->mains_hz10 = mains->hz10;
    w->mains_measured = ace2k_mains_measured(mains);
    w->switches = ace2k_sensors_switches(ace2k_heat_sensors);
    w->cutout = ((uint32_t)w->switches & (1U << ACE2K_SENSORS_STATE_CUTOUT_BIT)) != 0U;
}

static void fill_inputs(struct ace2k_heat_inputs *in, const struct ace2k_heat_world *w,
                        const struct ace2k_airflow *air)
{
    in->ntc_left_mc = w->ntc_left_mc;
    in->ntc_right_mc = w->ntc_right_mc;
    in->left_valid = w->left_valid;
    in->right_valid = w->right_valid;
    in->mains_present = w->mains_present;
    in->mains_hz10 = w->mains_hz10;
    in->mains_measured = w->mains_measured;
    in->cutout = w->cutout;
    in->fans_read = ace2k_airflow_fans_read(air);
    in->fans_commanded = ace2k_airflow_fans_commanded(air);
}

// Timer context, every 10 ms. It runs airflow, heat and the dryer's hook, in that order, in
// one callback. So no module sees the state of another one tick late.
static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    uint32_t now_us = ace2k_gate_now_us();
    struct ace2k_airflow *air = ace2k_heat_air;
    // The flash is written only with the gate idle. So a store in progress (config_cmds.c) holds
    // every lease path below.
    ace2k_heat_set_inhibit(&ace2k_heat_instance, ace2k_config_binding_storing());
    // A pending half counts, and so does a gate reading high in any state (stuck high = heat on).
    ace2k_airflow_set_heat_holds(air, ace2k_heat_active(&ace2k_heat_instance));
    ace2k_airflow_binding_tick(now_ms);
    if (sched_is_shutdown()) {
        ace2k_heat_shutdown(&ace2k_heat_instance); // returns at once when already shut
    }
    read_world(&ace2k_heat_world_now);
    struct ace2k_heat_inputs in;
    fill_inputs(&in, &ace2k_heat_world_now, air);
    ace2k_heat_tick(&ace2k_heat_instance, &in, now_ms, now_us);
    ace2k_heat_after_fn after = ace2k_heat_after;
    if (after != NULL) {
        after(now_ms);
        // A lease the dryer took, or dropped, in its hook holds the fans from this tick on. So a
        // fan-off request between two ticks is refused at once.
        ace2k_airflow_set_heat_holds(air, ace2k_heat_active(&ace2k_heat_instance));
    }
}

// The guard's veto: no bootloader entry while the heater may conduct. That is a lease held,
// an odd half pending, a pulse not seen over, or the gate reading high, even when latched. The
// bootloader drives no fans.
static bool bootloader_veto(void *ctx)
{
    (void)ctx;
    return ace2k_heat_active(&ace2k_heat_instance);
}

// The veto of the config store: the flash is written only with the gate idle, so that an erase
// never leaves a pulse high. The gate is busy with a lease held, an odd half pending, or a pulse
// not seen over. A latched gate was commanded off for good. So the latch's log entry may be
// written even with the gate stuck high.
static bool store_veto(void *ctx)
{
    (void)ctx;
    return ace2k_heat_busy(&ace2k_heat_instance);
}

static void ensure_ready(void)
{
    if (ace2k_heat_ready) {
        return;
    }
    // Every input binding is ready before the tick is registered. So the tick only reads them,
    // and never initialises one from the timer interrupt. Airflow's binding fetches env's
    // pointer here too.
    ace2k_heat_air = ace2k_airflow_binding();
    ace2k_heat_env = ace2k_env_binding();
    ace2k_heat_mains = ace2k_mains_binding();
    ace2k_heat_sensors = ace2k_sensors_binding();
    ace2k_gate_init(); // gate low and timer ready before heat's init drives gate_off
    ace2k_heat_init(&ace2k_heat_instance, &board_ops, NULL);
    // A full registry is a programming error. From init context this ends in a fault and a
    // watchdog reset. Either way it is loud.
    if (ace2k_guard_register_veto(ace2k_guard_binding(), bootloader_veto, NULL) != 0) {
        shutdown("ace2k: guard veto slots exhausted");
    }
    if (ace2k_config_binding_register_store_veto(store_veto, NULL) != 0) {
        shutdown("ace2k: config store veto taken");
    }
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_zerocross_set_hook(edge); // last: from here on the interrupt may call heat
    ace2k_heat_ready = true;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_heat_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_heat_binding_init);

struct ace2k_heat *ace2k_heat_binding(void)
{
    ensure_ready();
    return &ace2k_heat_instance;
}

const struct ace2k_heat_world *ace2k_heat_binding_world(void)
{
    return &ace2k_heat_world_now;
}

void ace2k_heat_binding_set_after_hook(ace2k_heat_after_fn fn)
{
    ace2k_heat_after = fn;
}

// Interrupts are disabled here (Klipper's run_shutdown). The lease goes first: heat stops
// deciding, for good. Every later lease is refused until a reset, so nothing ever "fires" into the
// floated pin after a clear_shutdown. Then the gate is floated. The tick survives the shutdown
// (tick.c), and it keeps running airflow's fan rule. Before init, nothing was configured.
void ace2k_heat_binding_shutdown(void)
{
    if (!ace2k_heat_ready) {
        return;
    }
    ace2k_heat_shutdown(&ace2k_heat_instance);
    ace2k_gate_float();
}
DECL_SHUTDOWN(ace2k_heat_binding_shutdown);
