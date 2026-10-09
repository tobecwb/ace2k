# 0007 — The name: ace2k

Date: 2026-09-14 · Status: accepted

## Context
The name is the prefix of every C symbol, every Python module, the `printer.cfg` section and the
repository. It must say which device it is for: the ACE **2** Pro, not the first-generation ACE
Pro, which is a different and incompatible unit. It must not collide with existing projects around
the ACE.

## Decision
`ace2k` — ACE 2 + Klipper. `ace2k/feed.c` with `ace2k_feed_start()`, `extras/ace2k.py`,
`[ace2k]`, `[mcu ace2k]`.

## Consequences
Short prefixes in C. The docs use the device's official name everywhere: "ACE 2 Pro" (Anycubic's
own product URL).

## Alternatives rejected
- `acek`: reads as the first-generation ACE Pro.
- `ace2-open`: already the name of a patch project by [hakimio](https://github.com/hakimio) and
  [Simon-CR](https://github.com/Simon-CR).
- `openace`, `libreace2`: longer prefixes, weaker link to Klipper.
