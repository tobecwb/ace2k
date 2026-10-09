# ace2k

Open firmware for the **Anycubic ACE 2 Pro**. It turns the unit into a native **Klipper** MCU.
You declare the unit as `[mcu ace2k]` in `printer.cfg`, and add a small `[ace2k]` host module.

The unit does these jobs on its own:

- It dries filament.
- It loads filament: when you insert a filament in a bay, the unit pulls it in and parks it.
- It finds and reads the RFID tag of each spool. The host then decodes what the tag says.

The main target is the Snapmaker U1.

ace2k should work with any Klipper printer. The unit becomes a Klipper MCU, and the `[ace2k]` host
module does not depend on the printer. But it has only been tested on a Snapmaker U1:

- On the U1, the adapter [ace2k-u1](https://github.com/tobecwb/ace2k-u1) connects the printer's
  own filament functions to the unit.
- On another printer, you control the lanes from your own macros with the `ACE_*` commands
  ([`docs/commands.md`](docs/commands.md)).

**Spool tags from almost any brand.** Spool tags come in two kinds, NTAG and MIFARE Classic, and
the firmware reads both. The host decodes what the tag says, with one small Python decoder per
brand.

- Decoded today: Anycubic, Bambu Lab and Snapmaker.
- Written, but turned off until they are tested on a real tag: Elegoo and Creality.
- Another brand can be added on the host if its tag layout is known. For a MIFARE tag, the key
  (or how to derive it) must be known too. This needs no firmware change and no reflash
  ([`docs/features.md`](docs/features.md#spool-tags), "Adding a brand").

Nothing is ever written to a tag.

**Status: Release Candidate `v0.12.0-rc.1`.** The long-run print test (burn-in) has not been done
yet.

**How this was built.** AI tools were used in developing this firmware. Every feature and every
change was verified on real hardware, over many hours of bench testing, before it was accepted.
Human review of the source code is still in progress.

**A note on the language.** English is not the author's native language, so some terms in this
documentation may read a little oddly. Corrections are welcome.

## Read this first

**Risks.** Flashing third-party firmware onto your ACE 2 Pro carries a small but real risk of
damaging it. What limits that risk:
- ace2k never writes to the bootloader or to the unit's factory calibration pages; it only reads them.
- If a flash is interrupted, the unit's bootloader stays in recovery mode. It accepts a new image
  over the same cable ([`docs/flashing.md`](docs/flashing.md)).
- The heater runs only with both fans on, and it stops if a temperature sensor fails. The unit's
  own 115 °C thermal cutout stays in place.

The main remaining risk is the dryer, because it switches mains power. See the warning below.

**Disclaimer.** This software is provided "as is", without warranty of any kind (see `LICENSE`).
You use it at your own risk. The author is not responsible for any damage to your unit, your
printer, your filament or anything else that results from installing or running it. Installing it
may void your warranty. This project is not affiliated with or endorsed by Anycubic or Snapmaker.

**Tested on.** One Anycubic ACE 2 Pro unit on a Snapmaker U1 running the paxx extended firmware
`1.6.0-paxx12-22`, at 127 V / 60 Hz. Nothing else has been tested.

> [!WARNING]
> **Connect only one unit to the printer.** ace2k has been tested with one ACE 2 Pro only. With
> two or more units, the behaviour is unknown. This applies to every combination:
>
> - all units on ace2k;
> - one unit on ace2k and another on the factory firmware;
> - units in a daisy chain, whatever firmware they run.
>
> Support for several units is planned but not implemented yet
> ([`docs/limitations.md`](docs/limitations.md#planned)).

> [!WARNING]
> **Do not leave a drying cycle unattended.** The dryer switches mains power to a heater. Only the
> author has tested it, for at most four hours of continuous drying, and only at 127 V / 60 Hz.
> Nobody has run it on 220–240 V / 50 Hz mains yet. If that is your mains, you will be the first.
>
> The firmware has its own protections:
>
> - The heater runs only with both fans on.
> - The heater stops at 85 °C at the outlets.
> - The heater stops when the chamber goes past its limit (the target + 10 °C, or 80 °C).
> - A lost sensor, lost mains or stopped fans end the cycle.
>
> Automated tests cover these protections. They have **not been triggered on purpose on a real
> unit**, because that could damage it. The author has only a few units, and they are very
> expensive where the author lives. If everything else fails, the unit's own 115 °C thermal cutout
> is a last protection, in hardware.

## Documentation

To install:

1. Build the USB cable ([`docs/cable.md`](docs/cable.md)).
2. Flash the unit ([`docs/flashing.md`](docs/flashing.md)).

To use it, read [`docs/features.md`](docs/features.md), then [`docs/commands.md`](docs/commands.md).

All the documents:

- [`docs/cable.md`](docs/cable.md) — the USB cable: Anycubic's own cable with a USB plug
- [`docs/flashing.md`](docs/flashing.md) — flash the unit, and go back to the factory firmware
- [`docs/commands.md`](docs/commands.md) — every `ACE_*` command and the flashing tool
- [`docs/features.md`](docs/features.md) — configuration, the follow, the tail, the grip, the
  feed-forward, encoder calibration, spool tags, drying, the lane LEDs
- [`docs/differences-from-stock.md`](docs/differences-from-stock.md) — what behaves differently
  from the factory firmware
- [`docs/limitations.md`](docs/limitations.md) — known limitations and what is planned
- [`docs/logging.md`](docs/logging.md) — recording a long run, and what to attach to a problem
  report
- [`docs/hardware.md`](docs/hardware.md) — the board: pins, sensors, links
- [`docs/protocol.md`](docs/protocol.md) — the host–unit protocol
- [`docs/architecture.md`](docs/architecture.md) — layers, the MCU/host split, the module map
- [`docs/rules.md`](docs/rules.md) — the numbered design rules the code comments cite
- [`docs/decisions/`](docs/decisions/) — why things are the way they are
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — the rules: C, Python, tooling, process, safety

## The Snapmaker U1

On the U1, a small adapter connects the printer's filament feed to the unit's lanes:
[`ace2k-u1`](https://github.com/tobecwb/ace2k-u1). It is a separate project with its own install
guide.

## Build

    git clone --recurse-submodules https://github.com/tobecwb/ace2k.git && cd ace2k
    git submodule update --init firmware/klipper   # in a clone made without --recurse-submodules
    scripts/doctor.sh          # the toolchain: Arm GNU Toolchain (with newlib), llvm, cppcheck,
                               # ruff, pytest
    make -C firmware build     # → firmware/build/ace2k-<version>.bin  (+ .elf, .dict)
    make -C firmware test      # host tests: the core with cc, the tools with pytest
    make -C firmware lint      # format check, clang-tidy, cppcheck, ruff
    make -C firmware lint-selftest   # proves the clang-tidy rules fire on tests/lint-fixtures/bad_style.c
    make -C firmware coverage        # llvm-cov report on src/ace2k/

Run every target from `firmware/`, or as `make -C firmware <target>` from the tree root.

Klipper is a git submodule, pinned in `firmware/klipper.pin`. `make build` applies one small patch
to it (`firmware/patches/`) and overlays `firmware/src/`. Nothing under `firmware/klipper/` is
edited by hand.

## Credits

ace2k is built on the published work of **[hakimio](https://github.com/hakimio)** (the ACE 2 Pro
protocol) and **[Simon-CR](https://github.com/Simon-CR)** (the reader protocol and the
spool-rotation concepts), among others. See [`docs/credits.md`](docs/credits.md).

## License

GPL-3.0 (see `LICENSE`).
