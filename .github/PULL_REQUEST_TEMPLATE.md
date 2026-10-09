## What and why

<!-- What the change does, and the issue or the scope it answers. -->

## Safety

Does this change touch the heater, the fans, the thermal cutout, the motors, the link-loss stop,
flash writes, the bootloader entry or the RFID readers?

- [ ] No.
- [ ] Yes. The new or changed invariant has a host test that tries to violate it.
- [ ] Yes. It was run on a unit. The bench values and the result are below.

## Checks

- [ ] `make -C firmware test` and `make -C firmware lint` pass.
- [ ] Docs updated in this pull request (`protocol.md`, `hardware.md`, an ADR) where they apply.
- [ ] Commit messages follow [CONTRIBUTING.md](https://github.com/tobecwb/ace2k/blob/main/CONTRIBUTING.md).
- [ ] No per-unit data (UID, calibration, serial numbers) is in the change.

## Bench

<!-- Only for changes that drive hardware: the unit, the image, what was measured. -->
