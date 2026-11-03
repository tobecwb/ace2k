/* What: what the serial driver tells the rest of the image about the host link:
 * - the idle time it waits for after the last received byte, and the two counters that measure
 *   how often it waited (the half-duplex turnaround);
 * - whether the host has been heard from lately, by a valid frame. That is link_ok of the motion
 *   rules;
 * - the longest silence between two valid frames, the margin of that judgement.
 * How: ace2k_serial_link_counters() copies the counters with interrupts masked, because the
 * receive and timer interrupts write them. The binding link/link_turnaround_cmds.c answers the
 * queries with them. The lane binding samples ace2k_link_ok() once per tick, and publishes it for
 * the feed binding (ace2k_lane_binding_link_ok()). In an image with the RS-485 driver it is
 * ace2k_serial_link_ok() of the driver; in one without, it is true. The turnaround instance and
 * its timer stay private to serial.c.
 * Depends on: <stdbool.h>, <stdint.h>, autoconf.h. Board code: the implementation is Klipper's
 * serial hook file, compiled only with CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485. */
#ifndef ACE2K_BOARD_SERIAL_H
#define ACE2K_BOARD_SERIAL_H
#include <stdbool.h>
#include <stdint.h>
#include "autoconf.h" /* CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485 */

/* The silence that ends a host transmission: 2.5 characters at 250000 8N1. One character is
 * 40 µs. Frames inside a host burst are back-to-back, so any longer gap means the host is done.
 * The counters measure this assumption. */
#define ACE2K_LINK_IDLE_US 100U

/* Since boot: how often the unit waited for the line before transmitting, and how often a wait
 * was extended because bytes kept arriving. */
void ace2k_serial_link_counters(uint32_t *deferred, uint32_t *rearmed);

/* This long without a valid frame from the host is a lost link. A host that is there sends a
 * clock query about once a second (CONTRIBUTING.md "Safety invariants"). Bytes alone do not count,
 * because noise, or a converter that keeps sending after the host died, is bytes too. */
#define ACE2K_LINK_LOST_MS 3000U

/* False before the first valid frame. True from then on, until a call finds no new frame for
 * ACE2K_LINK_LOST_MS. False from that call until the next frame. The loss is latched, because the
 * age is a 32-bit tick difference that reads small again one wrap later (35.8 s at 120 MHz). The
 * bindings ask every 10 ms, so a loss is seen long before the wrap. From any context. */
bool ace2k_serial_link_ok(void);

/* The longest silence between two valid frames since boot or the last clear, in milliseconds.
 * It is the silence that ACE2K_LINK_LOST_MS judges, dated at the same point, so it says how close
 * the host came to a lost link. The interval in progress counts once a frame ends it, and the
 * query is such a frame: the interval that the query itself ends is in the answer. A gap that
 * ended a loss never reads below ACE2K_LINK_LOST_MS. The first frame after boot ends no interval.
 * The clear does not touch the link state. Task context. */
uint32_t ace2k_serial_link_gap_ms(void);
void ace2k_serial_link_gap_clear(void);

/* link_ok of the bindings: nothing starts or keeps moving without it. An image without the
 * RS-485 driver (the deaf-test image) has no serial.c. Kconfig refuses it the motors, because
 * ACE2K_MOTOR depends on the driver's option. So the branch that reads the link as up only
 * ever feeds the read-only lane tick. */
static inline bool ace2k_link_ok(void)
{
#if CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485
    return ace2k_serial_link_ok();
#else
    return true;
#endif
}

#endif
