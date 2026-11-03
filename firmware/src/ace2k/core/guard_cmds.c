// Binding of guard: the veto registry, ace2k_bootloader_enter, and the halt.
//
// The halt is the only code in the image that stops feeding the watchdog. It runs from a task,
// never from the command handler or a timer, for two reasons. First, the response that announces
// it must leave the wire before the halt. Second, the link proof can then request it from its tick
// without halting inside an interrupt.
#include "core/guard_cmds.h"
#include "board/irq.h"  // irq_disable, irq_enable
#include "board/misc.h" // timer_read_time, timer_from_us
#include "command.h"    // DECL_COMMAND_FLAGS, sendf
#include "sched.h"      // DECL_TASK, DECL_SHUTDOWN, sched_add_timer, sched_wake_tasks

#define ACE2K_GUARD_ENTER_HALT_DELAY_MS 50U /* enough for the 10-byte response at 250000 baud */

static struct ace2k_guard ace2k_guard_instance;
static bool ace2k_guard_ready;
static struct timer ace2k_guard_halt_timer;
static bool ace2k_guard_halt_armed;
static volatile uint8_t ace2k_guard_halt_requested;
static volatile uint8_t ace2k_guard_halt_timer_fired; /* the armed timer ran (halt_event) */

struct ace2k_guard *ace2k_guard_binding(void)
{
    if (!ace2k_guard_ready) {
        ace2k_guard_init(&ace2k_guard_instance);
        ace2k_guard_ready = true;
    }
    return &ace2k_guard_instance;
}

static uint_fast8_t halt_event(struct timer *t)
{
    (void)t;
    ace2k_guard_halt_timer_fired = 1;
    ace2k_guard_halt_requested = 1;
    sched_wake_tasks();
    return SF_DONE;
}

void ace2k_guard_binding_request_halt_ms(uint32_t delay_ms)
{
    if (delay_ms == 0) {
        ace2k_guard_halt_requested = 1;
        sched_wake_tasks();
        return;
    }
    // Armed once. A second accepted ace2k_bootloader_enter inside the window must not add the
    // timer to Klipper's list a second time, because the list would point at itself. Once a
    // halt is requested, nothing needs to request it again. Command context only. The zero-delay
    // path above, which the tick also uses, only sets a flag and needs no guard.
    if (ace2k_guard_halt_armed) {
        return;
    }
    ace2k_guard_halt_armed = true;
    ace2k_guard_halt_timer.func = halt_event;
    ace2k_guard_halt_timer.waketime = timer_read_time() + timer_from_us(delay_ms * 1000U);
    sched_add_timer(&ace2k_guard_halt_timer);
}

// Drops a requested halt that a veto now objects to. The request is cleared. A timer that has
// already fired is disarmed, so a later ace2k_bootloader_enter can arm it again.
static void halt_withdrawn(void)
{
    ace2k_guard_halt_requested = 0;
    if (ace2k_guard_halt_timer_fired) {
        ace2k_guard_halt_timer_fired = 0;
        ace2k_guard_halt_armed = false;
    }
}

// Klipper calls every DECL_TASK target by name from generated code, so it is not static.
// The vetoes are asked once more with interrupts masked, right before the halt. The request came
// from the tick of the link proof, or from ace2k_bootloader_enter's accepted answer 50 ms
// earlier. Between the request and this task, a lease may have been taken, a lane started, or a
// PTC heated. Nothing can change after the mask.
// A veto that objects withdraws the request, and the unit runs on. The expired state of the link
// proof asks again on its next tick (every 10 ms), and halts on the first one no veto objects to.
// An ace2k_bootloader_enter whose halt was withdrawn is lost: the host sees the unit still
// answering, and asks again.
void ace2k_guard_halt_task(void)
{
    if (!ace2k_guard_halt_requested) {
        return;
    }
    irq_disable();
    if (!ace2k_guard_bootloader_allowed(ace2k_guard_binding())) {
        halt_withdrawn();
        irq_enable();
        return;
    }
    for (;;) {
        // The independent watchdog is no longer fed, so a reset into recovery follows.
    }
}
DECL_TASK(ace2k_guard_halt_task);

// Klipper's sched_timer_reset() drops every user timer on a shutdown, the armed halt timer
// included. But the host was already told accepted=1, and must not have to guess. So a halt that
// is armed and not yet requested is armed again, for a full window from now. Runs with interrupts
// disabled; sched_add_timer is safe there.
void ace2k_guard_shutdown(void)
{
    if (!ace2k_guard_halt_armed || ace2k_guard_halt_requested) {
        return;
    }
    ace2k_guard_halt_timer.waketime =
        timer_read_time() + timer_from_us(ACE2K_GUARD_ENTER_HALT_DELAY_MS * 1000U);
    sched_add_timer(&ace2k_guard_halt_timer);
}
DECL_SHUTDOWN(ace2k_guard_shutdown);

void ace2k_guard_cmd_bootloader_enter(uint32_t *args)
{
    (void)args;
    uint8_t accepted = ace2k_guard_bootloader_allowed(ace2k_guard_binding()) ? 1 : 0;
    sendf("ace2k_bootloader_enter_response accepted=%c", accepted);
    if (accepted) {
        ace2k_guard_binding_request_halt_ms(ACE2K_GUARD_ENTER_HALT_DELAY_MS);
    }
}
DECL_COMMAND_FLAGS(ace2k_guard_cmd_bootloader_enter, HF_IN_SHUTDOWN, "ace2k_bootloader_enter");
