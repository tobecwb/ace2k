# Contributing to ace2k

These are the rules every change follows. They keep the code modular, testable without hardware,
and safe on a unit that switches mains power. Read [`docs/architecture.md`](docs/architecture.md)
first: it describes the layers these rules refer to. `make lint` enforces most of the rules below.
The review checklist at the end covers what a machine cannot see.

## C rules

### Naming — the prefix is the namespace, C has no other

| What | Rule | Example |
|---|---|---|
| File | `snake_case`; the directory is the namespace, no prefix in the file name | `ace2k/dryer/dryer.c`, `ace2k/rfid/watch.c` |
| Public function (declared in a `.h`) | `ace2k_<module>_<verb>()` | `ace2k_dryer_start()`, `ace2k_lane_set_duty()` |
| Private function | `static`, no prefix, verb first | `enter_cooldown()`, `plausible_mv()` |
| Type | `struct ace2k_<module>`, **no `typedef`** of structs — seeing that it is a struct is information | `struct ace2k_dryer` |
| Enum | `enum ace2k_<module>_<thing>`, values `ACE2K_<MODULE>_<VALUE>` | `ACE2K_DRYER_KEEPING` |
| Constant | `ACE2K_<MODULE>_<NAME>_<UNIT>` — **the unit is always in the name** | `ACE2K_DRYER_MAX_TEMP_C`, `ACE2K_FEED_SLIP_WINDOW_UM`, `ACE2K_HEAT_GATE_PULSE_US` |
| Pin | only in `ace2k_board/pins.h`, with its evidence; **no `GPIO('E', 12)` literal anywhere else** | `#define ACE2K_PIN_FAN_LEFT GPIO('E', 12) /* docs/hardware.md "Fans" */` |
| Boolean | `bool` from `<stdbool.h>`, named as a predicate | `is_heating`, `fans_on` |
| Variable | `snake_case`, with the unit when it is a quantity | `ntc_mv`, `travel_um`, `deadline_ms` |

Units: `_ms` `_us` (time), `_mv` (millivolt), `_mc` (millidegree Celsius, for measurements),
`_c` (whole degrees, for user-facing limits, converted once at the boundary), `_um` (micrometre),
`_um_s` (speed), `_pct` (percent, 0–100), `_hz`. Distances are in micrometres, not millimetres,
so that they stay integers.

**No magic numbers.** Every number that is policy (a limit, a timeout, a threshold, a gain) is a
named constant with its reason beside it. It goes at the top of the `.h` if it is public, or of the
`.c` if it is private. `clang-tidy` enforces this (see [Lints](#lints)).

### Files and modules — the `.h` is the interface, the `.c` is the class

- One module = `<m>.h` + `<m>.c` + `<m>_cmds.c` + `tests/<group>/test_<m>.c`, in its group directory
  under `src/ace2k/`. Everything not declared in the `.h` is `static`.
- Every `.h` opens with three lines: **what it does, how it is used, what it depends on.** If you
  cannot write the three lines, the module is cut wrong.
- **No mutable global state.** Every module operates on a `struct ace2k_<m> *self` passed as the
  first argument. Bindings own the instances. Per-lane state is an array in one struct, indexed by
  lane.
- **Hardware only through `ops`**: `struct ace2k_dryer_ops { void (*fans_set)(void *ctx, bool on);
  … }`, with `ctx` supplied by the binding. The core never reads the clock or a register: `now_ms`
  and readings are arguments.
- The core includes only `<stdint.h>`, `<stdbool.h>`, `<stddef.h>` and other core headers.
- A `.c` over ~500 lines is a sign of a wrong cut. Split it (`feed.c` + `feed_insert.c` sharing a
  private `feed_internal.h`).
- Shared helpers (`ace2k_time_after()`, `ace2k_clamp_i32()`, `ACE2K_ARRAY_SIZE()`, mV ↔ m°C, CRC)
  exist **once**, in `ace2k/core/util.h`. Grep before you write one. If Klipper's `src/generic/`
  already has it, the binding uses Klipper's. The header's name shadows a BSD `<util.h>` that the
  macOS SDK ships. With the project's include path, ours wins. A missing include path shows up as
  undeclared-function errors, not as a missing-file error.
- Include order: the module's own header first, then the C standard headers, then project headers.
  Header guards `ACE2K_<MODULE>_H`.

### Style and control flow — the formatter decides, not the review

- C11. Four spaces, no tabs. **100 columns** in `src/ace2k/` and `src/ace2k_board/`. **79** in
  `src/stm32/gd32f30x.c`, which follows Klipper's own style because it is an upstream candidate.
  One `.clang-format` per directory enforces each. Brace placement is never discussed in review.
- Guard clauses and early return; nesting depth at most 3; no `goto`.
- **No `malloc`** anywhere. Storage is static or lives in the instance struct. Klipper's
  `alloc_chunk` is allowed in bindings only, at configuration time only.
- **Integers with explicit units by default** (mV, m°C, µm, ms). They are deterministic, and tests
  can check them with exact equality. Use `float` only where the mathematics asks for it (the PID
  and the NTC conversion), and **never in an ISR**.
- **An ISR does the minimum**: count, timestamp, set a flag. Policy runs in the tick. Variables
  shared with an ISR are `volatile` and a single aligned word. The ISR/main contract is written at
  the top of the module.
- Functions that can fail return `int`: `0` on success, a negative `-ACE2K_E<NAME>` otherwise. No
  error return is ignored silently.
- Time is `uint32_t now_ms`. A deadline is always compared with `ace2k_time_after(now, deadline)`,
  which survives the 49-day wrap. Never write `now >= deadline` by hand.
- Klipper's `shutdown()` is called only from a binding, and **only for a programming error** (an
  invalid oid). A physical event (slip, cutout, an open NTC) is a state plus an event, never a
  shutdown.

### Documentation and sources

- Everything — code, comments, documentation, commit messages — is written in English.
- A comment explains **why and under what constraint**, not what the line does. A comment that
  restates the code is deleted in review.
- Every `.h` has the three-line block, then a prose comment per public function: one sentence on
  what it does, then units, bounds and what happens on error. It also has a comment for each struct
  field that is not obvious. Use prose blocks (`/* … */`), no Doxygen tags.
- **Sources.** Every fact about the unit (a pin, a timing, a threshold, a behaviour) is stated
  with its source. A source is a measurement on the unit recorded in
  [`docs/hardware.md`](docs/hardware.md), a public datasheet, or a published work listed in
  `docs/credits.md`. Protocol facts point at [hakimio](https://github.com/hakimio)'s published
  schema or at observed behaviour of the original firmware. Every constant with a physical meaning
  cites its evidence, and **the evidence is always a measurement or an observed behaviour**. Values
  that are initial guesses say so (`/* initial value; re-tune on the bench */`). They are listed as
  open items in the pull request that adds them.
- Every decision that changes the architecture becomes an ADR in
  [`docs/decisions/`](docs/decisions/).

### Safety invariants that live in code, not in prose

- The single flash erase/program routine refuses any address outside the ace2k config page. It is
  the only flash-writing code in the application.
- `pins.h` classifies every pin. The triac gate is driven only by `dryer` through its `ops`. The
  cutout latch-reset pin is **never driven by any image**.
- The fan/heater interlock is in `dryer.c`: fans on before the first gate pulse, and kept on
  through a temperature-gated cool-down. It is not in the binding, and not in Python.
- The cutout fault input latches a fault in every dryer state, not only while heating.
- Motion stops on link loss, and `lane` enforces it:
  - Its tick takes `link_ok` as an argument. The binding samples it once per tick from the serial
    driver. Klipper's clock queries arrive about once a second. 3 s without a received byte is a
    lost link, latched until the next byte.
  - A false `link_ok` ends every running move on that tick, with the motor stopped.
  - `feed` receives the same sample after the lane's tick. Without it, `feed` refuses to start
    anything (a move, an assist, a load). It also ends what was armed with no move running: an
    assist waiting between bursts, a load in its settle.
  - The dryer continues on link loss, inside its own limits. It is autonomous by design.
- `ace2k_bootloader_enter` is refused while the dryer is HEATING or a lane is moving.
- No tag is ever written. Every frame to a tag leaves through `ace2k_iso14443a_send()`. Its gate
  lets through only the first bytes of activation and read: `REQA`, `WUPA`, the three `SELECT`
  codes, `HLTA`, `READ`, the two MIFARE authentications. No host command carries a raw frame. A
  MIFARE key is wiped the moment its authentication starts.
- A reader's field is on only while `rfid/watch` needs it, and every switch-off is read back. A
  field that stays on latches a fault that fails the reader's health bit. RST is never driven low.
- The load's search for the spool's tag moves only through `feed`'s bounded moves, with every
  motion rule of the feed:
  - its reach is capped at `ACE2K_FEED_SEARCH_MAX_UM` (1000 mm);
  - it goes past the parking point (where the load stops the filament) only with every other lane
    idle;
  - it yields the moment another lane is given a start;
  - it brings the filament back to the parking point, unless a STOP, an error, a runout, a lost
    link or a shutdown ended it.

### Credits

[`docs/credits.md`](docs/credits.md) lists every source with a link. A file that derives from a
source opens with a credit line:

- **hakimio** — the ACE 2 Pro protocol, its published schema, the interactive shell and the OTA
  updater [the flashing guide](docs/flashing.md) relies on. Credit the protocol work and the
  tools.
- **[Simon-CR](https://github.com/Simon-CR)** (`ace2-pro-firmware-research`) — the reader
  protocol, the spool-rotation concept, UID passthrough and the motion-yield rule; the Anycubic tag
  layout with **DnG-Crafts**.
- **OpenRFID** (suchmememanyskill) — the Snapmaker and Elegoo tag layouts and the Snapmaker public
  keys; an RFID project in its own right.
- **Snapmaker `u1-klipper`** — the open reader driver where the Snapmaker key is public.
- **Bambu-Research-Group / RFID-Tag-Guide** — Bambu key derivation and layout.
- **Elegoo** — the official tag guide.
- **OpenSpool** — the NDEF format.
- **Klipper** — everything in L0.

A *layout* or a *key* is a fact and needs a credit. *Code* ported from a project needs a
GPL-3.0-compatible licence and keeps its original header.

## Python rules

These are short, because the host is thin by the decision in
[The MCU / host split](docs/architecture.md#the-mcu--host-split).

- **Klipper's conventions first**: one module = one class with `load_config(config)` (or
  `load_config_prefix` for named sections); dependencies via `printer.lookup_object()`; G-code via
  `gcode.register_command()`; state via `get_status(eventtime)`. Anyone who has read a Klipper
  `extras/` module will recognise ours.
- **Namespace**: files `ace2k*.py`, classes `Ace2k<Thing>`, section `[ace2k]`, G-codes `ACE_*`,
  status keys `snake_case`.
- **Talk to the MCU only through the dictionary**: `mcu.lookup_command()` /
  `mcu.lookup_query_command()` to send, `mcu.register_response()` for events. No manual parsing, no
  bytes by hand. The command and event names are the firmware's public API
  ([`docs/protocol.md`](docs/protocol.md)).
- **Never block the reactor.** Use `reactor.pause()`, timers and completions, never `time.sleep()`.
  Klipper is single-threaded: a 2 s block pauses the whole print.
- **Do not duplicate what the MCU enforces.** Interlocks, limits and timeouts live in C. Python
  shows state, turns user intent into commands, and sequences the printer side (extruder, tip
  forming, the change itself). A safety rule found in Python is in the wrong place.
- **Style**: Python 3, PEP 8 at 100 columns, `ruff` for lint **and** formatting (one tool), type
  hints on public signatures. English docstrings have the same shape as the C prose: what, units,
  what happens on error.
- **Tests**: `pytest` with a fake `mcu` object for the sequencing logic. The fake records the
  commands sent and injects events. Klipper's batch mode (`test/klippy/`) is for integration, once
  a `.dict` exists.
- **Install**: the `.py` files are copied into the printer's `klippy/extras/`, and the `.cfg` files
  are included from `printer.cfg`. Nothing else in Klipper is touched. The Snapmaker U1 adapter's
  install guide shows the commands.

## Tooling

Principle: **every rule of [C rules](#c-rules) and [Python rules](#python-rules) that a machine
can check, a machine checks.** Human review is kept for what machines miss (see "Human review
checklist").

### Build

Every target is run from `firmware/`, or as `make -C firmware <target>` from the tree root.

| Target | Does |
|---|---|
| `make` | resets the submodule to its pin → applies `patches/*.patch` (`git apply --check` first) as one deterministic temporary commit → `overlay` (symlinks `src/ace2k`, `src/ace2k_board`, `src/stm32/gd32f30x.c` into `klipper/src/`, excluded from the submodule's status) → copies `config/ace2k.config` → `make -C klipper` → `mkimage.py` → `build/ace2k-<version>.bin`, `.elf`, `.dict`, size report → resets the submodule again |
| `make test` | host tests of the core (`cc`, no Klipper, no hardware) + `pytest` |
| `make lint` | `clang-format --dry-run`, `clang-tidy`, `cppcheck`, `ruff check` |
| `make lint-selftest` | proves the `clang-tidy` rules fire: runs them on `tests/lint-fixtures/bad_style.c` and requires the findings the fixture was written for |
| `make coverage` | `llvm-cov` report of the host tests on `src/ace2k/` (built with `clang` — `COVCC=` to point elsewhere) |
| `make format` | `clang-format -i`, `ruff format` |
| `make doctor` | checks the toolchain (`arm-none-eabi-gcc`, `llvm` for clang-format/clang-tidy, `cppcheck`, `ruff`) and the submodule pin |
| `make hooks` | installs `scripts/pre-commit`, which runs `make format-check` only, on the staged copy of the tree (what will be committed, not the working tree) and only when something it checks is staged; resolves the firmware directory from the repository root at run time; refuses to overwrite a hook it did not write itself |
| `make klipper-reset` | puts the submodule back on its pin (a failed `make` does this by itself; an interrupted one does not); refuses to discard uncommitted edits under `firmware/klipper/` unless `FORCE=1` |

There is one patch to Klipper. `firmware/patches/README.md` lists every hunk. It carries:

- the GD32F303 hooks: the processor entry, the clock file's routing, the stall notice for the flash
  writes, the internal reference as an ADC pseudo-pin, the software I²C setup a module can own;
- `console_try_sendf`, the transmit call that says whether a frame was queued. With it, a binding
  can hold a frame that does not fit instead of losing it (`src/ace2k/core/tx.h`).

The patch stays under 100 added lines (83 today, over ten files). A hook that would grow it past
that is a Klipper change to argue upstream.

To move to a newer Klipper:

1. Change the sha in `firmware/klipper.pin`.
2. Run `git -C firmware/klipper fetch && git -C firmware/klipper checkout <sha>`.
3. Run `git add firmware/klipper`, so the committed submodule pointer and `klipper.pin` agree.
4. Check that every patch in `firmware/patches/` still applies
   (`git -C firmware/klipper apply --check ../patches/0001-gd32f303-and-ace2k-hooks.patch`).
5. Rebuild and re-run `make test lint`.

Version: `git describe --tags --match 'v[0-9]*'` → SemVer `vX.Y.Z`, baked into the image and
returned by `ace2k_version`. The match keeps another project's tags in the same repository out of
it. Their names would also overflow the 31 characters the config record holds.

Size budget: the application slot is 112 KB. **The build fails above 96 KB**, to keep a reserve
for the config page and growth.

Edit only under `firmware/src/`, `firmware/config/`, `firmware/patches/`, `firmware/tests/` and
`firmware/tools/`. `firmware/klipper/` is disposable and reset on every build.

Toolchain: Klipper needs a cross toolchain **with newlib** (`sched.c` includes `<setjmp.h>`, and
the link uses `libc_nano`). Use the Arm GNU Toolchain (`brew install --cask gcc-arm-embedded` on
macOS, `gcc-arm-none-eabi` + `libnewlib-arm-none-eabi` on Debian/Ubuntu), not Homebrew's
`arm-none-eabi-gcc` formula. `scripts/doctor.sh` checks for `setjmp.h`. The build applies the patch
as a deterministic temporary commit in the submodule, and resets it afterwards. A dirty Klipper
tree would stamp the build host's name and time into the dictionary. If a build step fails, the
submodule is left on the temporary patch commit. The next `make build` (or `make clean`) resets
it.

### Formatting

There are two `.clang-format` files:

- `firmware/src/ace2k/`: 100 columns, 4 spaces, Linux brace style (function braces on their own
  line, block braces on the same line);
- `firmware/src/stm32/`: 79 columns, Klipper's style, so the port can go upstream without
  reformatting.

`tests/` and (when it exists) `src/ace2k_board/` use the `src/ace2k` style through a
`.clang-format` symlink. The formatter does not reflow comments (`ReflowComments: false`), so wrap
a long comment by hand. Python: `ruff format`. `scripts/pre-commit` runs the format check before
each commit.

### Lints

- **Compiler**: Klipper's `-Wall`, plus `-Wextra -Werror` on our own objects
  (`src/ace2k/Makefile`), on the host tests and in clang-tidy (`WarningsAsErrors`). The host build
  of the core is stricter still: `-Wshadow -Wstrict-prototypes -Wmissing-prototypes
  -Wdouble-promotion -Wundef -Wvla`. The core compiles both ways, so the stricter side rules.
- **`clang-tidy`** with a curated set: `bugprone-*`, `misc-*`, `performance-*`, `readability-*`.
  This includes `readability-magic-numbers` ("Naming": no magic numbers, checked mechanically),
  `readability-function-cognitive-complexity` (threshold 15) and `readability-identifier-naming`.
  The last one is configured with the table in "Naming": `ace2k_` prefix on globals, `UPPER_CASE`
  for enum values and constants, `snake_case` for functions and variables. It runs on the
  host-compilable C: the core, `util.h`, the tests. The cross compiler and `cppcheck` check the
  bindings. `tests/.clang-tidy` turns `readability-magic-numbers` off under `tests/`, because test
  literals are the point of a test. It runs on our directories only; Klipper's tree is excluded.
- **`cppcheck --enable=warning,style,performance,portability`**. Suppressions are inline and
  justified.
- **`ruff check`** on every Python file (see [Python rules](#python-rules)). The rule set is
  `ruff.toml` at the root of the tree.

### Tests

- **Core (C, host)**: a minimal harness of our own in `tests/test.h`, with zero dependencies,
  compiled with `cc`. It has named tests, `ASSERT_EQ` / `ASSERT_TRUE` / `ASSERT_STR_EQ` reporting
  `file:line`, and a `-k` filter. `ASSERT_EQ` compares through `long long`: no `uint64_t` ≥ 2^63,
  no floating point. Add a dedicated assertion when a module needs one.
  Every core module has `test_<m>.c`. **Every safety invariant has a test that tries to violate it
  and expects refusal.** The test uses a recording fake of the `ops` that counts violations on
  every call. Every tag decoder has real-dump fixtures under `tests/fixtures/`, **with a synthetic
  UID, never a real one**. The test re-encodes the dump: UID replaced, BCC recomputed, derived keys
  re-derived.
- **Coverage**: `llvm-cov` on `src/ace2k/`, reported in CI. It becomes a gate (80 %) from the
  first release.
- **Python**: `pytest` with the fake `mcu` (see [Python rules](#python-rules)).
- **Bench**: not automatable. Each subsystem has an acceptance checklist, written with its change.
  It is run on a unit, with the date and the unit noted in the pull request. The whole firmware is
  bench-tested before every release.

### CI

On push and pull request: `make doctor` → `make lint lint-selftest` → `make test` → `make coverage`
(reported in the job log) → `make build` (cross-build on the runner; artefacts
`ace2k-<version>.bin`, `.elf`, `.dict`, size).

On a `vX.Y.Z` tag: a release with the `.bin`, the `.dict` and their `SHA256SUMS`. The `.elf` stays
a build artefact, because its debug information carries the build machine's paths. The changelog
is written by hand in the release note.

Our objects, the host tests and clang-tidy are `-Werror`: one warning there is a red build.

## Process and safety

### The life of a feature

1. **A written scope**, in the issue or the pull request. It states:
   - the goal;
   - what is already known and **is not re-derived** (pins, measured constants, observed
     behaviour);
   - the binding safety rules;
   - verifiable acceptance criteria;
   - the deliverables, in commit order.
2. **A plan in small increments.** Each increment is a flashable image with its own version, and
   **each passes the host tests before any hardware**. Write the tests first for the core. Write
   the `ops` fake with the module.
3. **A build flag per subsystem** (`CONFIG_ACE2K_SENSORS`, `_ENV`, `_LANE`, `_MAINS`, `_MOTOR`,
   `_FEED`, `_HEAT`, `_DRYER`, `_RFID`, `_RFID_READ`, `_LED`, `_HEALTH`, in
   `firmware/src/ace2k/Kconfig`). An image without the flag **does not contain** the code that
   drives that hardware. Only the release configuration is built and checked (`make
   flag-check`). A new subsystem's bring-up enables it on its own first.
4. **Branch per feature, pull request to `main`, green CI, human review** with the
   "Human review checklist".
5. **Bench** against the scope's acceptance checklist. The measured values and the result go in
   the pull request. A constant that changes goes into `docs/hardware.md`.
6. **Docs in the same pull request**: `protocol.md` if a command or event changed, `hardware.md` if
   a pin or constant changed, an ADR if a decision changed.

**Definition of done**: host tests pass; lint clean; bench checklist filled; docs updated; **the
reversibility drill still passes** (see [Bench and recovery rules](#bench-and-recovery-rules)).

### Bring-up order of any subsystem

Read-only (sensors, counters) → **bounded** outputs (a maximum duration per command, a maximum
duty) → closed loop → autonomous behaviour. Within the dryer, the heater comes last, and never
without the interlock. Motors start at 20 % duty: below that, the motor's integrated driver does
not start (observed on the unit).

### Bench and recovery rules

- Before you flash any image on a unit, read its calibration through the factory firmware and keep
  the baseline **per unit**. After a restore, compare. Per-unit data (UID, calibration, serial
  numbers) **never enters the repository**.
- **One operator, one script on the port, a "go" before every physical drive.** Check for a process
  on the port with `lsof`, not with `pkill`.
- **Never claim a physical state the firmware cannot read back.** The firmware reports what it
  measures: fans and gate from the input register, not from what it wrote. The docs say what is
  expected *and* ask what was observed.
- A reading that contradicts the model (temperature rising during cool-down) is investigated, not
  explained away.
- **Every image that drives a motor or the heater** has the watchdog on, a bounded duration per
  command, motion stop on link loss, and the dryer's own limits (maximum duration, runaway checks,
  the cutout).
- **Reversibility drill at every release**, on the development unit: `ace2k_bootloader_enter` →
  recovery → factory firmware restored over the wire → calibration identical → ace2k flashed again
  through the factory updater. No passing drill, no release.

### Human review checklist — what the machine does not catch

- Is the module in the right layer? A Klipper include in the core: no. Policy in a `_cmds.c`: no.
  A safety rule in Python: no.
- Does the `.h` answer "what / how / depends on" in three lines?
- Names carry units; no duplicated helper; no per-lane copy.
- Does every new invariant have a test that tries to violate it?
- Do comments say why; does every cited evidence point to a measurement?
- Commit message: English, imperative, prefixed by the module
  (`dryer: refuse start while a lane is moving`), **no trailers**.

### Releases

- SemVer `vX.Y.Z`.
- A changelog by module.
- `.bin` + `sha256` + `.dict`.
- The bench checklist and the reversibility drill, dated in the release note.
