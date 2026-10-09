// ace2k board tick: one scheduler timer every 10 ms feeds the core ticks and the ms clock.
// The callbacks run in the timer interrupt: they count, compare and set flags; policy that
// needs task context (a flash write, the halt) is deferred to a task by the binding.
// A flash erase or program stalls the core for longer than one period (code runs from the
// same flash).  flash.c holds interrupts off across the operation and declares the stall to
// Klipper's timer dispatcher before releasing them, so the dispatcher catches every overdue
// timer up instead of shutting down; on that pass this event catches its own periods up —
// skipping the ones the stall swallowed, keeping the phase and the ms clock honest — instead
// of rescheduling behind the clock, which Klipper refuses past 1 ms.
#include "ace2k_board/tick.h"
#include "ace2k/core/util.h" // ACE2K_EFULL
#include "board/irq.h"       // irq_save, irq_restore
#include "board/misc.h"      // timer_read_time, timer_from_us, timer_is_before
#include "sched.h"           // sched_add_timer, DECL_INIT, DECL_SHUTDOWN

static struct timer ace2k_tick_timer;
static volatile uint32_t ace2k_tick_ms;
static ace2k_tick_fn ace2k_tick_fns[ACE2K_TICK_SLOTS];
static void *ace2k_tick_ctxs[ACE2K_TICK_SLOTS];
static uint8_t ace2k_tick_count;

static uint_fast8_t tick_event(struct timer *t)
{
    uint32_t now = ace2k_tick_ms + ACE2K_TICK_PERIOD_MS;
    t->waketime += timer_from_us(ACE2K_TICK_PERIOD_MS * 1000U);
    // Catch up after a stall: skip whole periods, keeping the phase and the ms clock honest, and
    // run the callbacks once for all of them.
    while (timer_is_before(t->waketime, timer_read_time())) {
        t->waketime += timer_from_us(ACE2K_TICK_PERIOD_MS * 1000U);
        now += ACE2K_TICK_PERIOD_MS;
    }
    ace2k_tick_ms = now;
    for (uint8_t i = 0; i < ace2k_tick_count; i++) {
        ace2k_tick_fns[i](ace2k_tick_ctxs[i], now);
    }
    return SF_RESCHEDULE;
}

int ace2k_tick_register(ace2k_tick_fn fn, void *ctx)
{
    if (ace2k_tick_count >= ACE2K_TICK_SLOTS) {
        return -ACE2K_EFULL;
    }
    // tick_event reads count and then the slot from the interrupt; the three stores are made
    // one step against it, so it never sees the count ahead of the slot.
    irqstatus_t flag = irq_save();
    ace2k_tick_fns[ace2k_tick_count] = fn;
    ace2k_tick_ctxs[ace2k_tick_count] = ctx;
    ace2k_tick_count++;
    irq_restore(flag);
    return 0;
}

uint32_t ace2k_tick_now_ms(void)
{
    return ace2k_tick_ms;
}

// Klipper calls every DECL_INIT target by name from generated code, so it is not static.
void ace2k_tick_init(void)
{
    ace2k_tick_timer.func = tick_event;
    ace2k_tick_timer.waketime = timer_read_time() + timer_from_us(ACE2K_TICK_PERIOD_MS * 1000U);
    sched_add_timer(&ace2k_tick_timer);
}
DECL_INIT(ace2k_tick_init);

// Klipper drops every user timer on a shutdown (sched_timer_reset, before the shutdown
// functions run), this one included; the tick must outlive a shutdown so that the link-proof
// window keeps running and the halt can still be requested.  Runs with interrupts disabled;
// sched_add_timer is safe there (Klipper's own timer_reset re-adds its timer the same way).
void ace2k_tick_shutdown(void)
{
    ace2k_tick_timer.waketime = timer_read_time() + timer_from_us(ACE2K_TICK_PERIOD_MS * 1000U);
    sched_add_timer(&ace2k_tick_timer);
}
DECL_SHUTDOWN(ace2k_tick_shutdown);
