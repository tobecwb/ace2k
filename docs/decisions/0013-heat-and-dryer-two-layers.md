# 0013 — The heater in two layers: `heat` owns the gate, `dryer` owns the policy

Date: 2026-09-27 · Status: accepted · Amends: 0004 (where the fan/heater interlock lives)

## Context
ADR 0004 put the autonomous dryer in the MCU, with "every interlock", and the engineering guide
placed the fan/heater interlock in `dryer.c`. Specifying the dryer showed what that costs. The
controller, the state machine, the cool-down, the protections and the interlock would share one
module and one state. So a bug in the policy (a stuck state, a controller that keeps asking for
heat) could hold the triac on through the very code meant to stop it. The heater draws
≈ 340–380 W from the mains. The 115 °C cutout on the heatsink bounds the worst case, but it is a
last resort, not a design margin.

## Decision
Two layers, plus the airflow module:
- **`heat`** is the only code that drives the triac gate (PC8), through its `ops`. It knows nothing
  about drying. It fires only inside a **lease**: a window of at most 2 s that its caller must
  renew. Before every gate pulse it checks absolute limits no caller can relax:
  - both fans commanded on and reading high;
  - a duty of at most 90 %;
  - both outlet NTCs valid and below 85 °C;
  - the mains present and at a plausible frequency;
  - the cutout input clear.

  A violation drops the gate in the same half-cycle and latches.
- **`dryer`** carries the policy: states, the cascade controller, the vent, the cool-down, the
  protections that need a model of the heater, the log. It reaches the heater only by asking
  `heat` for leases.
- **`airflow`** owns the fans and the flaps. It keeps the fans on while an outlet NTC reads above
  45 °C, in every state.

## Consequences
- The interlock is enforced in `heat.c`, and the policy lives in `dryer.c`. The guide's §4.5 and
  the module map say so.
- A policy bug can at worst stop renewing the lease, or ask for a duty `heat` refuses. Within 2 s
  the gate stops by itself, in the zero-cross interrupt, whatever the tasks do.
- Each layer is host-tested alone, `heat` with a test per limit that tries to break it.
- Defence in depth, bottom up: the cutout, the watchdog (a reset floats the gate, which is off),
  `heat`'s limits and lease, `dryer`'s protections.
- The cost is one more module and one more `ops` table.

## Alternatives rejected
- One `dryer` module with the interlock inside (the guide as written): less code, one state for
  everything, and a failure of the policy is a failure of the protection.
- A separate safety task auditing the outputs every tick: the rules written twice, and a second
  scheduler path to prove, for one dangerous load.
