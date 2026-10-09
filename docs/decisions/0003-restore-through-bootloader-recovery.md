# 0003 — Factory firmware restored through the bootloader's recovery mode

Date: 2026-09-14 (mechanism verified on the bench 2026-09-12) · Status: accepted

## Context
The unit must always be returnable to the factory firmware over the wire. The unit's bootloader is
never overwritten. On every reset it validates the application and jumps to it. After a watchdog
reset, or when the application is invalid, it stays in a persistent recovery mode. That mode
accepts an upload with the original protocol.

## Decision
ace2k implements no part of the original update protocol. A command, `ace2k_bootloader_enter`,
stops feeding the watchdog. The chip resets into recovery, and
[hakimio](https://github.com/hakimio)'s updater restores the factory image. Klipper's `reset` stays
a plain system reset, so `FIRMWARE_RESTART` never lands in recovery. The command is refused while
the dryer heats or a lane moves.

## Consequences
- The application has no flash-write code except for its own config page.
- A genuine hang reaches recovery by the same watchdog.
- The project does not distribute the factory image.

## Alternatives rejected
An in-application updater speaking the original protocol: hundreds of lines with flash access,
made redundant by the bootloader's own receiver.
