# Changelog

## v0.12.0-rc.1 — 2026-10-09

First public release (Release Candidate).

ace2k is open firmware that makes the Anycubic ACE 2 Pro a native Klipper MCU, declared as
`[mcu ace2k]` with a thin `[ace2k]` host module. In firmware it:

- loads a newly inserted filament and follows the spool, with tail handling and a grip on that
  filament;
- feeds forward from the host and calibrates its encoders automatically;
- finds and reads each spool's RFID tag, the host decoding what it says;
- dries filament, with the heater interlocked to both fans and a sensor-fault stop;
- shows each lane's state on the lane LEDs, and flashes the unit from one command, with a way
  back to the factory firmware.

Primary target: the Snapmaker U1. Tested on one unit at 127 V / 60 Hz.

See [`docs/features.md`](docs/features.md), [`docs/limitations.md`](docs/limitations.md) and
[`docs/differences-from-stock.md`](docs/differences-from-stock.md).

The long-run print test (burn-in) is pending.
