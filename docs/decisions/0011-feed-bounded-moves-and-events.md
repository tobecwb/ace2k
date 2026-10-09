# 0011 — feed: bounded moves and events, never a shutdown

Date: 2026-09-21 (bench-verified 2026-09-20/21) · Status: accepted

## Context
The lanes move for the first time. A jam, a tangle, a filament that runs out, a motor that stalls:
these are the ordinary weather of a filament changer. The unit is the part of the printer a host
cannot see into, so what it says is all the host knows.

The original Anycubic protocol, as observed, handles those cases badly:
- it answers with a status code that the next mode overwrites. There are seven codes, and a host
  ever sees only four of them;
- a STOP leaves an assist's motor running;
- there is no command to leave an error.

Klipper's own idiom for anything wrong is `shutdown()`, which halts the whole printer. That is the
right answer to a host bug, not to a spool. And a jam is not an instant. It is a state the operator
clears by hand, and during it the lane must refuse to move and say why it stopped.

## Decision
- **Bounded moves.** Every motor run is a bounded move of the lane core: a length of at most
  2 000 mm, and a deadline of 3/2 the expected time plus a second. No code path raises the run pin
  outside it. An assist's bursts and a load's steps are such moves.
- **Events, not shutdowns.** Every physical condition is a lane state plus an event, never a
  `shutdown()`. The event carries the mode's odometers: the motor's travel and the filament's. The
  bindings shut down only for a malformed host command (a lane, a mode, a scale out of range).
- **Explicit clear.** Leaving the error state is explicit: `ace2k_feed_clear`. There is one
  exception, decided at the bench: the filament leaving the insert sensor ends any error as a
  `runout`, because the error was about the filament.
- **The link.** Motion stops on link loss (3 s without a host byte) and on a Klipper shutdown.
  Nothing starts without a host.
- **The comparator.** It compares the motor's travel with the filament's on the encoder, with two
  triggers: a standstill and a partial. The buffer (the spring-loaded plunger between the unit and
  the head) gives three readings: full (the plunger at its pulled end), rest (centred) and taut
  (at its pushed end). They turn a trip into `stuck` or `tangled`. The lane's own buffer full ends
  a forward move at once.
- **The assists.** Both assists keep the plunger at rest, at the speed the host armed.
- **The load** is bounded and abortable at every step, the settle included.
- **Settings.** The tunables live in `printer.cfg` and reach the unit at connect. The unit never
  retries a move on its own.

## Consequences
The host owns the policy on a jam. It gets the kind, the mode, the odometers and the start's
sequence, and decides whether to clear, retry or stop the print. The choreography of a filament
change (the extruder side) builds on these events in a later change.

What the bench taught reshaped the mechanism without touching the rule:
- A move's length is the filament's travel on the encoder, not the motor's. The drive gear slips
  a few per cent under a spool.
- The plunger at rest is both assists' target. A cycle that fed until the buffer read full kept
  the filament compressed in the tube. A creep at rest would have unloaded the lane.
- The sensor decides: a filament gone is a runout, not an error to clear.
- The comparator supersedes the assist's own stall. The encoder answers before a timer would, so
  the timer was taken out and its kind left reserved.
- The transport tries a frame and holds one that does not fit, rather than dropping it. A start
  repeated whole is the retry it is.

The odometer (a per-lane total of what was delivered) is the next step. The wire carries the
travel per mode only.

## Alternatives rejected
- A shutdown on a jam: it halts the print for a condition a hand clears in seconds, and loses the
  odometers.
- A status word the next mode overwrites, as the original protocol does: the host misses the error
  that mattered.
- A blind retry in the firmware: a forced back-and-forth for a filament caught at a tube's exit
  belongs to a later change, with the host deciding.
- An assist cycle aimed at the buffer's full end: rejected at the bench, because the filament
  compressed in the tube and the plunger locked.
- An implicit clear on the next command: a macro that keeps going would never see the error.
