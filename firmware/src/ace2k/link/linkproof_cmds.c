// Binding of the link proof: ace2k_version (the proof), ace2k_linkproof_state, the tick that
// expires the window, the LED pattern that shows which state the image is in.
#include "autoconf.h" // CONFIG_ACE2K_LED
#include "link/linkproof.h"
#include "core/util.h" // ace2k_str_copy
#include "core/config_cmds.h"
#include "core/guard_cmds.h"
#include "ace2k_board/tick.h"
#include "command.h" // DECL_COMMAND_FLAGS, sendf, shutdown
#include "sched.h"   // DECL_INIT
#if CONFIG_ACE2K_LED
#include "lane/led_cmds.h"
#endif

#ifndef ACE2K_VERSION
#error "ACE2K_VERSION is defined by src/ace2k/Makefile"
#endif

static const char ace2k_running_version[] = ACE2K_VERSION;
// store_proof copies it into the record's field and init() compares in full: a longer string
// (a describe with a tag, -dirty and a config suffix can reach 36) would never prove.
_Static_assert(sizeof(ace2k_running_version) <= ACE2K_CONFIG_VERSION_LEN,
               "ACE2K_VERSION must be at most 31 characters (the config record's field)");
// The version as a dictionary constant too, so a host tool reads an image's version from the
// dictionary compiled into it, without running it.
// cppcheck-suppress syntaxError
DECL_CONSTANT_STR("ACE2K_VERSION", ACE2K_VERSION);
static struct ace2k_linkproof ace2k_linkproof_instance;
static bool ace2k_linkproof_ready;

static int store_proof(void *ctx, const char *version)
{
    (void)ctx;
    struct ace2k_config *cfg = ace2k_config_binding();
    ace2k_str_copy(cfg->record.proven_version, ACE2K_CONFIG_VERSION_LEN, version);
    cfg->record.flags |= ACE2K_CONFIG_FLAG_LINK_PROVEN;
    return ace2k_config_binding_store();
}

static const struct ace2k_linkproof_ops ace2k_linkproof_board_ops = { .store_proof = store_proof };

static void show_state(void)
{
#if CONFIG_ACE2K_LED
    ace2k_led_binding_set_pattern(ace2k_linkproof_state(&ace2k_linkproof_instance) ==
                                          ACE2K_LINKPROOF_PROVEN
                                      ? ACE2K_LED_INTRO
                                      : ACE2K_LED_WAITING_HOST);
#endif
}

// An expired window asks for the halt into recovery — through the same vetoes as
// ace2k_bootloader_enter (the dryer's rule 12): while one objects (the heater's gate reading high, a lease,
// a lane moving) the halt is deferred, not dropped.  The expired state is sticky and the core
// answers ENTER_RECOVERY on every tick, so the halt is asked again every 10 ms and follows on
// the first tick no veto objects.  Meanwhile the unit keeps running and answering —
// ace2k_linkproof_state reports proven=0, remaining_ms=0; a gate stuck high defers it until the
// gate reads low (a power cycle), the only safe way out, since the bootloader drives no fans.
static void tick(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    if (ace2k_linkproof_tick(&ace2k_linkproof_instance, now_ms) != ACE2K_LINKPROOF_ENTER_RECOVERY) {
        return;
    }
    if (ace2k_guard_bootloader_allowed(ace2k_guard_binding())) {
        ace2k_guard_binding_request_halt_ms(0);
    }
}

static void ensure_ready(void)
{
    if (ace2k_linkproof_ready) {
        return;
    }
    const struct ace2k_config *cfg = ace2k_config_binding();
    ace2k_linkproof_init(&ace2k_linkproof_instance, &ace2k_linkproof_board_ops, NULL, &cfg->record,
                         ace2k_running_version, ace2k_tick_now_ms());
    // A full registry is a programming error; from init context this ends in a fault and a
    // watchdog reset — loud either way.
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

void ace2k_linkproof_cmd_version(uint32_t *args)
{
    (void)args;
    ensure_ready();
    sendf("ace2k_version_response version=%s", ace2k_running_version);
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
