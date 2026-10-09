# 0009 — First-boot link proof: an unspoken-to image resets itself into recovery

Date: 2026-09-14 (bench-verified 2026-09-14/15) · Status: accepted

## Context
Consider a valid image that boots, feeds its watchdog and never answers on the host link. It cannot
be replaced over the wire: the bootloader validates it and jumps to it on every power cycle. Until
now that case needed the debug header. The first image with a real serial driver is exactly the
kind of image that can be valid and deaf.

## Decision
The config page records which image version last proved the host link.
- An image whose version is not the recorded one starts a 180 s window.
- The first answered `ace2k_version` stores the proof.
- An image whose window expires stops feeding the watchdog. It resets into the bootloader's
  persistent recovery, where the updater works.
- A proven image never resets itself for lack of a host.

The proof is `ace2k_version`, not Klipper's `identify`. The parser offers no hook for `identify`
without a patch. The host module queries the version anyway when Klipper identifies the MCU
(`klippy:mcu_identify`). That event comes ahead of the connect handlers, which stop at the first
config error elsewhere. A proof that arrives after the window has expired is refused: the halt the
expiry requested stands.

## Consequences
- The `[ace2k]` section is part of the link. With a bare `[mcu ace2k]`, the unit falls into
  recovery three minutes after a fresh flash.
- The LEDs show the state: a chase while the window runs, an intro sweep once proven (a lane-1
  heartbeat up to v0.11.0).
- Every image starts with a blank config page. A reflash erases it, whichever path the upload takes
  (`docs/hardware.md`, "Memory map"). So the first `ace2k_version` after every flash writes the
  proof: one flash write per flash, and one flash erase stall the scheduler must absorb.
- The bench must include a deliberately deaf build, to keep the mechanism honest. On the first
  one, the window measured 181 s.

## Alternatives rejected
- Trigger on `identify`: a patch to Klipper's `command.c`, for a signal the host module supplies
  anyway.
- A separate host command to arm the proof: one more thing to forget, no gain.
- No proof at all: the debug header would be the only way back from a deaf image.
