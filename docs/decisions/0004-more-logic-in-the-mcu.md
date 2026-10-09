# 0004 — More logic in the MCU

Date: 2026-09-14 · Status: accepted; the RFID part amended by
[0012](0012-rfid-transport-in-the-mcu-brands-on-the-host.md); where the fan/heater interlock lives
amended by [0013](0013-heat-and-dryer-two-layers.md)

## Context
A typical Klipper board is "dumb": the host decides everything. The ACE is different:
- some behaviours must work without a host (drying, with spool rotation);
- some deadlines are under 100 ms (the motor speed loop, slip detection, buffer switches, mains
  zero-cross);
- it has safety interlocks (fan/heater, thermal cutout);
- its core can run at 120 MHz, with headroom to spare.

## Decision
The MCU owns:
- the autonomous dryer: PID cascade, fans, flaps, spool rotation with a lane lockout, and the
  setpoint persisted in its config page;
- the insert sequence: grab, sweep past the antenna, read, park, report. It is bounded and
  abortable;
- the RFID pipeline, with every format decoder and the UID in every result;
- every interlock.

The host owns configuration, G-code, the slot/spool model, Spoolman, printer-side sequencing and
jam *policy*. The MCU reports facts and enforces invariants. The host decides intent.

## Consequences
The C is the larger part of the project. It is written to the core/binding rules so it stays
testable. The host module is thin. Klipper's `[heater_generic]` is not used for the dryer.

## Alternatives rejected
- The dryer as a Klipper `[heater_generic]`: free PID and graphs, but no drying without a host.
- Tag decoding in Python over host-driven SPI: no reading without a host, and a chattier link.
