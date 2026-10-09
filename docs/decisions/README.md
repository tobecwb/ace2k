# Decisions

Architecture Decision Records: one file per decision that changes the architecture, numbered in
sequence. Each has the same four headings: Context, Decision, Consequences, Alternatives rejected.
Each record carries a `Date:` and a `Status:` line; a reversed decision gets a new record and the
old one's status becomes `superseded by NNNN` — its text is never rewritten.

| # | Decision | Date |
|---|---|---|
| 0001 | [ace2k is a native Klipper MCU](0001-native-klipper-mcu.md) | 2026-09-09 |
| 0002 | [One unit per serial port (no chaining)](0002-one-unit-per-serial-port.md) | 2026-09-09 |
| 0003 | [Factory firmware restored through the bootloader's recovery mode](0003-restore-through-bootloader-recovery.md) | 2026-09-14 |
| 0004 | [More logic in the MCU: autonomous dryer, insert sequence, RFID pipeline, interlocks](0004-more-logic-in-the-mcu.md) | 2026-09-14 |
| 0005 | [Own tree with Klipper as a pinned submodule and one patch](0005-own-tree-klipper-submodule.md) | 2026-09-14 |
| 0006 | [Pure core + thin binding per subsystem; the core is host-tested](0006-pure-core-thin-binding.md) | 2026-09-14 |
| 0007 | [The name: ace2k](0007-name-ace2k.md) | 2026-09-14 |
| 0008 | [Credits and sourcing policy](0008-credits-and-sourcing-policy.md) | 2026-09-14 |
| 0009 | [First-boot link proof: an unspoken-to image resets itself into recovery](0009-first-boot-link-proof.md) | 2026-09-14 |
| 0010 | [Factory calibration read at boot; `printer.cfg` overrides](0010-factory-calibration-read-at-boot.md) | 2026-09-16 |
| 0011 | [feed: bounded moves and events, never a shutdown](0011-feed-bounded-moves-and-events.md) | 2026-09-21 |
| 0012 | [RFID: the transport in the MCU, the brands on the host](0012-rfid-transport-in-the-mcu-brands-on-the-host.md) | 2026-09-26 |
| 0013 | [The heater in two layers: `heat` owns the gate, `dryer` owns the policy](0013-heat-and-dryer-two-layers.md) | 2026-09-27 |
