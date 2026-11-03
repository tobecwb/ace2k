// Binding of the link proof.  It holds these parts.
// - ace2k_version, the query that proves the image.
// - ace2k_linkproof_state.
// - The tick that expires the window.
// - The task that stores again a proof that the store refused busy.
// - The LED pattern that shows which state the image is in.
#include "autoconf.h" // CONFIG_ACE2K_LED
#include "link/linkproof.h"
#include "core/config_cmds.h" // ace2k_config_binding, ace2k_config_binding_store(_vetoed)
#include "core/guard_cmds.h"
#include "ace2k_board/serial.h" // ace2k_link_ok
#include "ace2k_board/tick.h"
#include "board/irq.h" // irq_save, irq_restore
#include "command.h"   // DECL_COMMAND_FLAGS, sendf, shutdown
#include "sched.h"     // DECL_INIT, DECL_TASK, sched_wake_task, sched_check_wake
#if CONFIG_ACE2K_LED
#include "lane/led_cmds.h"
#endif

#ifndef ACE2K_VERSION
#error "ACE2K_VERSION is defined by src/ace2k/Makefile"
#endif

static const char running_version[] = ACE2K_VERSION;
// The proof copies the version into the record's field, and init() compares the whole string.  So a
// longer string would never prove.  A describe with a tag, -dirty and a config suffix can reach 36
// characters.
_Static_assert(sizeof(running_version) <= ACE2K_CONFIG_VERSION_LEN,
               "ACE2K_VERSION must be at most 31 characters (the config record's field)");
// The version is also a dictionary constant.  So a host tool can read the version of an image from
// the dictionary compiled into it, without running it.
// cppcheck-suppress syntaxError
DECL_CONSTANT_STR("ACE2K_VERSION", ACE2K_VERSION);
// Klipper's DECL_CONSTANT encodes its value with signed literals inside the macro.  So the lint's
// signed-bitwise check is off for these declarations only.
// NOLINTBEGIN(bugprone-signed-bitwise)
// The window is a constant too.  The flasher reads it from the image.  It uses it to say how long
// an image that it could not prove waits for a host.
DECL_CONSTANT("ACE2K_LINKPROOF_WINDOW_MS", ACE2K_LINKPROOF_WINDOW_MS);
// Since v0.12.0, this image takes every key of printer.cfg as an init command.  So the keys are out
// of the config checksum of Klipper.  The constant is declared here, beside the version, so that
// every image has it, whatever subsystems the image carries.  A host that meets an image without
// it keeps the keys as config commands.
DECL_CONSTANT("ACE2K_KEYS_AS_INIT", 1);
// Since v0.13.0, a proof that the store refused busy keeps the window from expiring while the host
// keeps talking.  The host spoke, so the unit waits for the flash however long the heater holds it,
// and never resets into recovery for that while the link lives.  The host reads this constant to
// say so when it sees the window end.
DECL_CONSTANT("ACE2K_LINKPROOF_RETRY_PAST_WINDOW", 1);
// NOLINTEND(bugprone-signed-bitwise)
static struct ace2k_linkproof ace2k_linkproof_instance;
static bool ace2k_linkproof_ready;
static struct task_wake ace2k_linkproof_wake;

static int store(void *ctx)
{
    (void)ctx;
    return ace2k_config_binding_store();
}

// The tick runs in the timer interrupt, so masking the interrupts holds it.  The pair never
// nests (see linkproof.h), so one saved word is enough.
static irqstatus_t ace2k_linkproof_irq_flag;

static void hold_tick(void *ctx)
{
    (void)ctx;
    ace2k_linkproof_irq_flag = irq_save();
}

static void release_tick(void *ctx)
{
    (void)ctx;
    irq_restore(ace2k_linkproof_irq_flag);
}

static const struct ace2k_linkproof_ops board_ops = {
    .store = store,
    .hold_tick = hold_tick,
    .release_tick = release_tick,
};

static void show_state(void)
{
#if CONFIG_ACE2K_LED
    ace2k_led_binding_set_pattern(ace2k_linkproof_state(&ace2k_linkproof_instance) ==
                                          ACE2K_LINKPROOF_PROVEN
                                      ? ACE2K_LED_INTRO
                                      : ACE2K_LED_WAITING_HOST);
#endif
}

// An expired window asks for the halt into recovery.  The halt goes through the same vetoes as
// ace2k_bootloader_enter (the dryer's rule 12).  For example, a veto objects when the heater's gate
// reads high, when a lease is held, or when a lane moves.  While a veto objects, the halt waits.
// It is not dropped.  The expired state stays, and the core answers ENTER_RECOVERY on every tick.
// So the halt is asked again every 10 ms, and it happens on the first tick where no veto objects.
// Meanwhile the unit keeps running and answering.  ace2k_linkproof_state reports proven=0 and
// remaining_ms=0.  A gate stuck high holds the halt until the gate reads low, which takes a power
// cycle.  That is the only safe way out, because the bootloader drives no fans.
//
// A proof that the store refused busy is stored again by the task, never here, because a store is
// flash work.  While it waits and the host keeps talking, the core does not expire the window, even
// past the deadline.  The tick hands the core ace2k_link_ok(), the link that the motion rules use
// too.  A host silent for the link-loss time past the deadline expires the window.  The
// task is woken only on a tick where the store's veto lets it through.  The dryer's log waits for
// the heater in the same way.  So a long lease does not wake the task on every tick.
static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    struct ace2k_linkproof *lp = &ace2k_linkproof_instance;
    if (ace2k_linkproof_tick(lp, now_ms, ace2k_link_ok()) == ACE2K_LINKPROOF_ENTER_RECOVERY) {
        if (ace2k_guard_bootloader_allowed(ace2k_guard_binding())) {
            ace2k_guard_binding_request_halt_ms(0);
        }
        return;
    }
    if (ace2k_linkproof_retry_wanted(lp) && !ace2k_config_binding_store_vetoed()) {
        sched_wake_task(&ace2k_linkproof_wake);
    }
}

static void ensure_ready(void)
{
    if (ace2k_linkproof_ready) {
        return;
    }
    struct ace2k_config *cfg = ace2k_config_binding();
    ace2k_linkproof_init(&ace2k_linkproof_instance, &board_ops, NULL, &cfg->record, running_version,
                         ace2k_tick_now_ms());
    // A full registry is a programming error.  From init context this ends in a fault and a
    // watchdog reset.  Either way, the failure is loud.
    if (ace2k_tick_register(tick, NULL) != 0) {
        shutdown("ace2k: tick slots exhausted");
    }
    ace2k_linkproof_ready = true;
    show_state();
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_linkproof_binding_init(void)
{
    ensure_ready();
}
DECL_INIT(ace2k_linkproof_binding_init);

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.  When the
// retry is refused, it waits for the next tick that wakes the task.
void ace2k_linkproof_binding_task(void)
{
    if (!sched_check_wake(&ace2k_linkproof_wake)) {
        return;
    }
    if (ace2k_linkproof_retry(&ace2k_linkproof_instance) == 0) {
        show_state();
    }
}
DECL_TASK(ace2k_linkproof_binding_task);

void ace2k_linkproof_cmd_version(uint32_t *args)
{
    (void)args;
    ensure_ready();
    sendf("ace2k_version_response version=%s", running_version);
    if (ace2k_linkproof_proven(&ace2k_linkproof_instance) == 0) {
        show_state();
    }
}
DECL_COMMAND_FLAGS(ace2k_linkproof_cmd_version, HF_IN_SHUTDOWN, "ace2k_version");

void ace2k_linkproof_cmd_state(uint32_t *args)
{
    (void)args;
    ensure_ready();
    uint32_t now = ace2k_tick_now_ms();
    sendf("ace2k_linkproof_state_response proven=%c remaining_ms=%u",
          ace2k_linkproof_state(&ace2k_linkproof_instance) == ACE2K_LINKPROOF_PROVEN ? 1 : 0,
          ace2k_linkproof_remaining_ms(&ace2k_linkproof_instance, now));
}
DECL_COMMAND_FLAGS(ace2k_linkproof_cmd_state, HF_IN_SHUTDOWN, "ace2k_linkproof_state");
