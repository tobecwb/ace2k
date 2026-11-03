/* What: the interrupt priorities of the board code, in one place.  A lower number preempts a
 * higher one.
 * How: each driver passes its level to ACE2K_ARMCM_ENABLE_IRQ.  The host link comes first, so
 * no received byte waits.  The motor tach, the zero-cross and the gate timer share the next
 * level, so none of them preempts another.  Klipper's timer dispatch runs below them all, at
 * ACE2K_IRQ_PRIO_KLIPPER_TIMER (Klipper sets it in generic/armcm_timer.c).
 * Depends on: nothing. */
#ifndef ACE2K_BOARD_IRQ_PRIO_H
#define ACE2K_BOARD_IRQ_PRIO_H

/* UART4.  A byte arrives every 40 µs at 250000 baud. */
#define ACE2K_IRQ_PRIO_LINK 0U
/* TIM4, the motor tach.  The handler is a few loads and stores per pulse. */
#define ACE2K_IRQ_PRIO_FG 1U
/* EXTI0, the mains zero-cross.  The handler reads the clock, filters the edge and calls the
 * hook.  The hook decides the heater's half-cycle and fires a gate pulse at most.  It runs at
 * most 120 times a second, plus the rejected edges. */
#define ACE2K_IRQ_PRIO_ZEROCROSS 1U
/* TIM7, the end of a gate pulse.  Same level as the zero-cross, which fires the gate. */
#define ACE2K_IRQ_PRIO_GATE 1U
/* Klipper's SysTick, which runs the timer dispatch.  Not set here. */
#define ACE2K_IRQ_PRIO_KLIPPER_TIMER 2U

_Static_assert(ACE2K_IRQ_PRIO_LINK < ACE2K_IRQ_PRIO_FG, "the link preempts the tach");
_Static_assert(ACE2K_IRQ_PRIO_GATE == ACE2K_IRQ_PRIO_ZEROCROSS,
               "the zero-cross and the gate timer never preempt each other");
_Static_assert(ACE2K_IRQ_PRIO_ZEROCROSS < ACE2K_IRQ_PRIO_KLIPPER_TIMER &&
                   ACE2K_IRQ_PRIO_FG < ACE2K_IRQ_PRIO_KLIPPER_TIMER,
               "the board's interrupts preempt Klipper's timer dispatch");

#endif
