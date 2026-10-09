# ace2k — architecture

This page explains how the firmware is put together:

- the target;
- the layers on the MCU;
- the split between the MCU and the host;
- how the tree relates to Klipper;
- the module map.

The rules for writing code in it are in [`../CONTRIBUTING.md`](../CONTRIBUTING.md). Decisions and
their reasons are in [`decisions/`](decisions/).

## Target and non-goals

**ace2k makes the Anycubic ACE 2 Pro a native Klipper MCU.** The unit appears as `[mcu ace2k]` in
`printer.cfg`. It speaks Klipper's own serial protocol over its RS-485 host port. A `[ace2k]` host
module in `klippy/extras/` drives it. The primary target is the Snapmaker U1, which runs Klipper.
Any Klipper printer with a spare USB–RS-485 adapter is in scope.

Unlike a typical Klipper board, the ACE keeps real behaviour in the MCU:

- it dries filament on its own, with no host attached (a cascade controller, fans, flaps);
- it runs the whole insert sequence (grab, sweep the tag past the antenna, read, park, report);
- it reads and decodes RFID tags itself;
- it enforces every safety interlock in firmware.

The host module is thin: configuration, G-code, the slot/spool model, and the printer-side
sequencing of a filament change.

Non-goals:

- talking to an Anycubic printer;
- compatibility with the original Anycubic protocol, except what is needed to restore the factory
  firmware (see "Reversibility");
- chaining several units on one port.

## Architecture

### Layers on the MCU

Four layers, bottom-up. What separates them is **who may include what**.

```
┌────────────────────────────────────────────────────────────────┐
│ host  klippy/extras/ace2k*.py         config, G-code, the      │
│       (Python, in the printer's Klipper)  slot/spool model     │
└───────────────▲────────────────────────────────────────────────┘
                │ Klipper protocol: commands, query responses, events (sendf)
┌───────────────┴────────────────────────────────────────────────┐
│ L3  bindings   ace2k/*/*_cmds.c      DECL_COMMAND, oid, sched  │
│                                       timers, the `ops`        │
│                                       implementations, sendf   │
├────────────────────────────────────────────────────────────────┤
│ L2  core       ace2k/*/*.c (PURE)     state machines, PID,     │
│                                       interlocks, tag decoders │
│                no registers, no Klipper includes               │
├────────────────────────────────────────────────────────────────┤
│ L1  board      stm32/gd32f30x.c       the generic port: 120 MHz│
│                ace2k_board/*.c        clock, ADC; plus what    │
│                                       Klipper's HAL lacks:     │
│                                       quadrature encoders, FG  │
│                                       capture, zero-cross/triac│
├────────────────────────────────────────────────────────────────┤
│ L0  Klipper    sched, command, generic/, stm32/ HAL            │
│                (submodule; one small patch — see               │
│                "Overlay, not fork")                            │
└────────────────────────────────────────────────────────────────┘
```

Dependency rules — these are what make the code modular in practice:

- **L2 includes nothing from L0, L1 or L3.** It reaches hardware only through a
  `struct ace2k_<module>_ops` of function pointers, supplied by the binding. Time (`now_ms`) and
  sensor readings arrive as arguments. That is why the core compiles on a laptop with `cc` and is
  unit-tested with fakes (see CONTRIBUTING.md, "Tests"). The host-test build has no Klipper tree
  on its include path. So a Klipper include in the core is a build error, not a review comment.
- **L3 has no logic.** It only translates:
  - a Klipper command into a core call;
  - a `sched` timer into `ace2k_<module>_tick()`;
  - an `ops` call into Klipper's `gpio_*` / `gpio_adc_*` / `gpio_pwm_*` / `spi_*` helpers or an
    `ace2k_board` driver;
  - a core event into a `sendf`.

  An `if` that encodes policy inside a `_cmds.c` file is in the wrong layer.
- **L1 does not know what an ACE is.** `stm32/gd32f30x.c` is a generic port of the GD32F303 (a
  register-compatible STM32F103 with its own clock tree). It is written in Klipper's own style, so
  it can be offered upstream. `ace2k_board/` knows the pins and the timers of this board, not the
  policy.

### The MCU / host split

The rule: **the MCU reports facts and enforces invariants; the host decides intent.**

The MCU owns anything that

- must work without a host — the dryer (its cascade controller in `dryer`, the heater under
  `heat`'s interlock, the fans and flaps in `airflow`) with its log persisted in the MCU's own
  config page;
- has a deadline under 100 ms — the motor speed loop, the slip comparator, the reaction to the
  buffer switches, the mains zero-cross;
- is a safety interlock — fan/heater, the thermal-cutout fault input, the lockout of a lane in use,
  the watchdog;
- is the RFID pipeline end to end — reader activation, ISO 14443A, the decoders of every known
  format, and **the tag UID in every result**, whatever the format;
- is the insert sequence — bounded, abortable by the host at every step.

The host owns `printer.cfg`, G-code, the slot/spool model, Spoolman, the extruder-side sequencing of
a filament change, what the user sees, and the *policy* on a jam. The MCU emits `slip`, `stall`,
`stuck`, `tangled` as events with the measured deficit. **It never calls `shutdown()` for a
physical event**: a jam is a state to act on, not a reason to kill the print.

Communication is Klipper's:

- commands host → MCU (`ace2k_feed_start oid=%c mode=%c length_um=%u speed_um_s=%u`);
- query responses;
- unsolicited events MCU → host via `sendf` (`ace2k_feed_event oid=%c kind=%c deficit_um=%i`).

The exact dictionary is [`docs/protocol.md`](protocol.md). It is treated as the firmware's public
API: changing it is a versioned, documented change. The safety rules each subsystem is built to
are numbered in [`docs/rules.md`](rules.md). Code comments cite them as "rule N" inside that
subsystem's files.

### Reversibility — back to the factory firmware over the wire

ace2k does not speak the original Anycubic protocol. Returning to the factory firmware rests on two
properties of the unit's bootloader, which ace2k never writes:

- it validates the application on every reset and jumps to it;
- **after a watchdog reset it stays in a persistent recovery mode**, which accepts a firmware
  upload over the same serial link.

So:

```
ace2k running ──(1) G-code ACE_RESTORE_STOCK ──▶ MCU: motion stopped, heater off;
                                                 refused while the heater is leased or a lane moves
                                           (2) stops feeding the watchdog, interrupts off, spins
                                           (3) ≤ 0.5 s later the watchdog resets the chip
bootloader ────(4) sees the watchdog-reset flag ──▶ recovery mode, persistent, original protocol
host ──────────(5) klipper stopped; hakimio's updater uploads the factory image ──▶ ~30 s
bootloader ────(6) verifies the image, installs it, resets ──▶ factory firmware runs
```

- **(1)** `ace2k_bootloader_enter` is a normal Klipper command, exposed as G-code by the host
  module. On a bench without a printer, Klipper's own `klippy/console.py` connects to any Klipper
  MCU and lets you type it. No tool of ours is needed.
- **(2)–(3)** Klipper's STM32 port already arms the independent watchdog at ~0.4 s and feeds it
  from a scheduler task. "Enter the bootloader" simply means to stop feeding it. No flash write is
  involved: **the application has no flash-write code at all except for its own config page**
  (see CONTRIBUTING.md, "Safety invariants that live in code").
- **(4)** This is why Klipper's `reset` command (what `FIRMWARE_RESTART` sends) must remain a plain
  system reset. Without the watchdog flag, the bootloader validates the application and re-enters
  it. Recovery is reached only through the watchdog, and only on explicit request. A genuine hang
  reaches the same place by the same mechanism: that is the safety net.
- **(5)** Klipper reports "Lost communication with MCU 'ace2k'". This is expected;
  [the flashing guide](flashing.md) says to stop the service first. The factory image is
  Anycubic's and **is not distributed by this project**. The flashing guide points to hakimio's
  instructions for extracting it from Anycubic's own update package.
- **(6)** The unit's calibration and material records live in flash pages ace2k never touches. So
  the unit returns byte-identical to factory. The flashing guide includes the "read the calibration
  before, compare after" check.
- **The other direction (factory → ace2k)** uses the factory firmware's own update command:
  hakimio's `ace2-ota-update.py <port> ace2k-X.Y.Z.bin --version ace2k-X.Y.Z`. The image carries
  the 8-byte trailer the bootloader expects (`firmware/tools/mkimage.py` appends it as the last
  build step). The updater computes the checksum it announces by itself. The version string
  announced to the factory updater is limited to 11 characters (`ace2k-1.2.3` fits).

Heater consequence, fixed as a rule: after the reset, the bootloader leaves every pin floating, so
the fans stop with a warm PTC. The 115 °C thermal cutout bounds the worst case. The command is
**refused while the heater holds a lease (the dryer heating), allowed in IDLE and COOLDOWN**. The
recovery path outranks a de-energised, still-warm heater losing its fans.

## Layout and modules

### Overlay, not fork

New files (`src/ace2k/`, `src/ace2k_board/`, `src/stm32/gd32f30x.c`) are linked into the Klipper
tree by `make overlay`. Changes to files that already exist in Klipper are one patch in
`firmware/patches/`: today 83 lines added over ten files (`firmware/patches/README.md` gives the
count and the list). Among them:

- `src/stm32/Kconfig` gets the `MACH_GD32F303` processor entry (its CMSIS header name, the clock
  default — 120 MHz under `ACE2K_CLOCK_120M`, 72 MHz when unset — 256 KB flash, 48 KB RAM), the
  UART4 / RS-485 serial choice and a `source "src/ace2k/Kconfig"` line;
- `src/stm32/Makefile` gets `-mcpu=cortex-m4` for the part, our clock file and serial driver, and
  `-include src/ace2k/Makefile`;
- `src/stm32/stm32f1.c` and `internal.h` route `armcm_main()` to our clock setup and
  `get_pclock_frequency()` to the /4 buses;
- `src/generic/armcm_timer.c` gains `timer_note_stall()` for the flash routine;
- `src/stm32/adc.c`, `src/i2c_software.[ch]`, `src/generic/serial_irq.c` and `misc.h` carry the
  rest.

The `ACE2K_*` options live in our own `src/ace2k/Kconfig`, not in the patch. The build resets the
submodule to the pin, applies the patch as a temporary commit, and removes it again after the
build. Anyone reading the repository sees only our code. Moving the pin is a short procedure (see
CONTRIBUTING.md, "Build").

The port pieces that Klipper lacks are in the tree:

- the high-driver clock sequence the GD32F303 needs above 100 MHz, and the clock case in
  `stm32f1.c`;
- the ADC prescaler at 120 MHz;
- UART4 with the RS-485 driver-enable (PA11, high before the first bit, low on
  transmit-complete). Klipper's serial driver has no such notion.

The clock and the ADC prescaler are in `src/stm32/gd32f30x.c`, the serial driver in
`src/ace2k_board/serial.c`, and the routing in the patch.

### Modules of the core

A subsystem is `<name>.h` (the public API, documented) + `<name>.c` (pure) + `<name>_cmds.c`
(binding) + `tests/<group>/test_<name>.c`, all inside the group directory of the subsystem. The
four lanes are **one** implementation, with `struct ace2k_lane lanes[ACE2K_LANE_COUNT]` and a
`const` table of per-lane pins. They are never four copies.

| Module | Responsibility | Hardware, via `ops` |
|---|---|---|
| `lane` | one motor: duty, run/enable, direction; FG tach pulse count; filament encoder count; the 10 Hz speed loop | pwm, run/enable, dir, fg, encoder |
| `feed` | per-lane motion state machine: feed / rollback / assist / rollback-assist; gating by the buffer switches; the slip comparator; the stall timeout; **all seven error states exposed** (the original firmware never reports `stuck` or `tangled`); explicit error clear; STOP always stops the motor; **the insert sequence** (grab → sweep → read → park → report), bounded and abortable | `lane`, `sensors`, `rfid` |
| `sensors` | INSERT (ADC, hysteresis, IIR, per-unit thresholds), EMPTY (the original firmware never reports it), `BUF_RST` / `BUF_BACK` / `HUB`: the three plunger positions per lane (rest, pushed, pulled — the pulled-end halls of the four lanes share one input), debounced; edges become events | adc, gpio |
| `airflow` | the two fans together, on / off, read back from their pins (no tachometer); the flaps by pulse, no cache, the position as last commanded (unknown at boot); an owner (none, manual, dryer); **the fan rule**: never off while an outlet NTC reads above 45 °C (or reads invalid), in any state, after a shutdown too | fans, flap coils |
| `heat` | the only code that touches the triac gate: the lease (≤ 2 000 ms, renewed from above), full-cycle firing from the zero-cross edge, **the absolute limits** (fans on and reading high, duty ≤ 90 %, NTCs valid and < 85 °C, mains present and plausible, cutout clear) judged every tick, nothing fired on a verdict older than three half-cycles, the gate never left high; latches with its first reason; a cutout sticky until a power-on reset (ADR 0013) | gate (PC8 + a one-pulse timer) |
| `dryer` | IDLE / STARTING / HEATING / COOLDOWN / FAULT; the cascade controller; the vent; the cool-down; the protections (not heating, impossible response, sides apart, chamber ceiling, stale chamber, NTCs, mains, fans); a 24 h maximum; the spool rule with no host; clearable faults, the cutout's by power cycle only; a reset never resumes; the persistent log. Spool rotation is a later change | `heat`, `airflow`, `env`, `mains`, `sensors` |
| `env` | the chamber T/RH sensor (I²C), NTC mV → m°C, filtering | i2c |
| `rfid/reader` | the RC522-class reader at register level | spi, rst |
| `rfid/iso14443a` | REQA, anticollision, SELECT cascade, MIFARE authentication, NTAG reads | `reader` |
| `rfid/tag_<format>.c` | **one decoder per format**: `match()` + `decode()` → `struct ace2k_spool` | — |
| `rfid/registry` | the table of formats; `ace2k_rfid_identify()` walks it and returns the spool record **and always the UID** | — |
| `led` | the four lane LEDs, named patterns | gpio |
| `config` | the persisted MCU record (the link proof; the dryer's counters and fault log) with a CRC, in **one 2 KB page at the top of the application slot (`0x08023800`)** — never the unit's own calibration pages, never the bootloader | flash (guarded) |
| `guard` | the flash-write allow-list (only the config page), the pin allow-list, the gating of `ace2k_bootloader_enter` | — |

Cross-cutting:

- `ace2k_board/pins.h` names every pin once, grouped by subsystem, each with the evidence of its
  measurement;
- `ace2k/util.h` holds the shared helpers (see CONTRIBUTING.md, "Files and modules").

**The heater's path.** One binding, `heat_cmds.c`, registers one 10 ms tick callback. It runs, in
this order:

1. `airflow` (the fan rule);
2. `heat` (read the inputs, judge them, check the gate, expire the lease);
3. the dryer's hook (the policy, which renews the lease).

Because of that order, no layer sees another's state a tick late.

The zero-cross interrupt refuses an edge closer than 7 000 µs to the last accepted one. This is
the bounce lockout; the refused edges are counted (`rejects`). The interrupt hands each accepted
edge to `heat` through the hook `zerocross.c` exposes. `heat` reads only what the tick published,
as whole words, and fires through its `ops`. The one-pulse timer's interrupt only drops the gate.

The tick and the ADC scan survive a Klipper shutdown, so the fan rule keeps running after one.
The shutdown handlers drop the lease and the gate first, and the edge hook fires nothing after a
shutdown.

### Extension paths

**A new spool brand** (the case that motivated the rules):

1. `src/ace2k/rfid/tag_<brand>.c` implements
   `const struct ace2k_tag_format ace2k_tag_<brand> = { .name, .match, .decode }`.
2. One line in `rfid/registry.c`.
3. `tests/rfid/test_tag_<brand>.c` with a tag dump as fixture (synthetic UID, see CONTRIBUTING.md,
   "Tests").

Nothing else changes: the `ace2k_rfid_result` event already carries `format` and the fields of
`struct ace2k_spool`; the host module already displays them. The same "table + one file per case"
shape is used for feed error kinds and LED patterns.

### The host side

The host side mirrors the MCU modules, one file per concern:

- `ace2k.py` (the `[ace2k]` section, the `[mcu]` binding, G-code);
- `ace2k_dryer.py`;
- `ace2k_rfid.py` (the spool model);
- `ace2k_feed.py` (load/unload sequencing with the extruder).

It is thin, because the policy lives in the MCU.

## Repository layout

    firmware/   the MCU image: src/ace2k/<group>/ (core + bindings, one directory per
                subsystem group), src/ace2k_board and
                src/stm32/gd32f30x.c, the Klipper submodule, patches, config, tests, tools
    klippy/     the host module(s) for the printer's Klipper
    config/     printer.cfg snippets
    docs/       the documentation
    scripts/    doctor, hooks

The sources are grouped by subsystem under `firmware/src/ace2k/`: `core/` (config, guard, health,
report, tx, util), `link/`, `lane/` (lane, sensors, LEDs), `feed/`, `heat/` (heater, airflow,
mains), `dryer/`, `env/` and `rfid/`. `firmware/tests/<group>/` mirrors them. Includes are written
relative to `src/ace2k/` (`#include "feed/feed.h"`). Board code uses `ace2k/<group>/<file>.h`. The
host's `klippy/extras/` stays flat: Klipper loads each extras module as a top-level Python module,
so it cannot be nested.
