/* What: what the serial driver tells the rest of the image about the host link — the idle time
 * it waits for after the last received byte and the two counters that measure how often it
 * waited (the half-duplex turnaround), and whether the host has been heard from lately (the
 * motion rules' link_ok).
 * How: ace2k_serial_link_counters() copies the counters with interrupts masked (the receive and
 * timer interrupts write them); the binding ace2k/link_turnaround_cmds.c answers the query with
 * them.  ace2k_link_ok() is what the lane binding samples once per tick and publishes for the
 * feed binding (ace2k_lane_binding_link_ok()): the driver's ace2k_serial_link_ok() in an image
 * with the RS-485 driver, true in one without.  The turnaround instance and its timer stay
 * private to serial.c.
 * Depends on: <stdbool.h>, <stdint.h>, autoconf.h (board code; the implementation is Klipper's
 * serial hook file, compiled only with CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485). */
#ifndef ACE2K_BOARD_SERIAL_H
#define ACE2K_BOARD_SERIAL_H
#include <stdbool.h>
#include <stdint.h>
#include "autoconf.h" /* CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485 */

/* The silence that ends a host transmission: 2.5 characters at 250000 8N1 (one character is
 * 40 µs).  Frames inside a host burst are back-to-back, so any longer gap means the host is done
 * (the counters measure the assumption). */
#define ACE2K_LINK_IDLE_US 100U

/* Since boot: how often the unit waited for the line before transmitting, and how often a wait
 * was extended because bytes kept arriving. */
void ace2k_serial_link_counters(uint32_t *deferred, uint32_t *rearmed);

/* Silence on the receive side this long is a lost link: a host that is there sends a clock
 * query about once a second (CONTRIBUTING.md "Safety invariants"). */
#define ACE2K_LINK_LOST_MS 3000U

/* False before the first byte; true from then on until a call finds the receive side silent
 * for ACE2K_LINK_LOST_MS, and false from that call until the next byte — latched, because the
 * age is a 32-bit tick difference that reads small again one wrap later (35.8 s at 120 MHz),
 * and the bindings ask every 10 ms, so a loss is seen long before the wrap.  From any context. */
bool ace2k_serial_link_ok(void);

/* The bindings' link_ok: nothing starts or keeps moving without
 * it.  An image without the RS-485 driver (the deaf-test image) has no serial.c, and Kconfig
 * refuses it the motors (ACE2K_MOTOR depends on the driver's option), so the arm that reads the
 * link as up only ever feeds the read-only lane tick. */
static inline bool ace2k_link_ok(void)
{
#if CONFIG_STM32_SERIAL_UART4_PC11_PC10_RS485
    return ace2k_serial_link_ok();
#else
    return true;
#endif
}

#endif
