# 0006 — Pure core + thin binding per subsystem

Date: 2026-09-14 · Status: accepted

## Context
Klipper's in-tree modules mix policy and register access in one file. They are small, and the host
does the thinking. Ours carry state machines, a PID, interlocks and decoders. They must be tested
without a unit on the desk.

## Decision
Each subsystem has two parts:
- a pure core (`<m>.c/.h`): no registers, no Klipper includes. Hardware goes through a
  `struct ace2k_<m>_ops` of function pointers. Time and readings are arguments;
- a thin Klipper binding (`<m>_cmds.c`): commands, the `oid`, scheduler timers, the `ops`
  implementation, events.

The core compiles with the host `cc` and is unit-tested with fakes. The host-test build has no
Klipper on its include path, so the compiler enforces the rule.

## Consequences
- A core and a binding per subsystem, plus the core's test file.
- One indirect call per hardware access.
- Every safety invariant has a test that tries to violate it.
- The binding is where the code "speaks Klipper".

The bootstrap tree's one binding was the pattern's smallest instance: a version-reporting commands
file, since removed. The per-subsystem `*_cmds.c` files, such as `core/health_cmds.c`, are the
pattern now.

## Alternatives rejected
- Klipper-idiomatic modules: not host-testable, policy and hardware mixed.
- A C++ core: foreign to Klipper's C build, and it closes the door to offering the processor port
  upstream.
