# 0002 — One unit per serial port

Date: 2026-09-09 · Status: accepted

## Context
The original protocol addresses units on a shared RS-485 bus. Klipper's MCU protocol has no
address field.

## Decision
Each ace2k unit has its own USB–RS-485 adapter and its own `[mcu]` section. Chaining is out of
scope.

## Consequences
Four units need four adapters. The chain port is unused. Nothing in the firmware or the host
module models a bus.

## Alternatives rejected
An addressing layer on top of Klipper's protocol. It would need a fork of the MCU's `command.c` and
of the host's `msgproto.py`, for a feature the target printer does not need.
