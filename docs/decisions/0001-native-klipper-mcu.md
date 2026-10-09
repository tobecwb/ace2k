# 0001 — ace2k is a native Klipper MCU

Date: 2026-09-09 · Status: accepted

## Context
The ACE 2 Pro is a four-lane filament unit with a dryer and RFID readers. A proprietary serial
protocol drives it. The goal is to use it with a Klipper printer (Snapmaker U1 first), with
firmware we control. A standalone firmware would need, before any ACE behaviour exists: a
scheduler, a framed protocol with CRC and retransmit, drivers for UART, SPI, ADC, PWM and timers,
and a host integration.

## Decision
The unit becomes a Klipper MCU. It runs Klipper's `src/` (scheduler, command dispatch, generic
drivers) plus our modules. It speaks Klipper's serial protocol and appears as `[mcu ace2k]`. A thin
`klippy/extras/` module integrates it.

## Consequences
- Framing, dictionary, timers, GPIO/ADC/PWM/SPI come for free.
- One MCU per serial port.
- The host halts the print if the link drops. So the firmware never calls `shutdown()` for a
  physical event.
- Klipper's build system and licence (GPL-3.0) apply.

## Alternatives rejected
- A standalone firmware with its own protocol: five to ten times the firmware effort.
- Patches on the factory binary: not our code, not maintainable, and not the point.
