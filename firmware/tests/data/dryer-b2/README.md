# Dryer heater runs, 2026-09-27

The heater driven by ace2k on the development unit, at 127 V / 60 Hz, unit closed and empty, both
exhaust flaps closed. The image was a test build: the heater answers only the
bench commands (`ace2k_heat_run duty_pct=D ms=T`, `ace2k_heat_stop`), the fans are switched on
before the first gate pulse and held by the thermal fan rule (on above 45 °C at an outlet NTC, off
once both read below 42 °C). The host read `ace2k_heat_state` and `ace2k_env_state` once a second
and wrote one row per read. The four `run-*`
files are also data `firmware/tools/thermal_fit.py` can fit.

**Read `ntc1` as the RIGHT outlet NTC (PA4) and `ntc2` as the LEFT one (PA5)**, as in
`../heater-2026-09-13/`; `docs/hardware.md` ("Analogue inputs") names them by side.

| File | What |
|---|---|
| `run-10-2s.csv` | 10 % for 2 s from a cold unit (22 half-cycles), then the fans' 30 s; the file runs on through the idle wait and into the 20 % run below (from `t_s` 122.3 it repeats `run-20-120s.csv`) |
| `run-20-120s.csv` | 20 % for 120 s (2 878 half-cycles), then the cool-down, then idle |
| `run-35-to50.csv` | 35 %, stopped when the right NTC read 50 °C (about 71 s of heating, 2 958 half-cycles), the cool-down held by the fan rule until both NTCs < 42 °C; from `t_s` 318.7 a fans-only cool-down before the next run (no heating) |
| `run-50-to50.csv` | 50 %, stopped when the right NTC read 50 °C (about 37 s of heating, 2 288 half-cycles), the cool-down |
| `cut-a-expiry.csv` | cut-off (a): 20 % for 3 s, the lease left to expire |
| `cut-b-link.csv` | cut-off (b): 20 % for 60 s, the host's USB cable pulled. From `t_s` 10 the logger repeats the last values it had received; from `t_s` 14 it marks the heater's columns `?`. The count the MCU really stopped at is the first row of `cut-c-restart.csv` |
| `cut-c-restart.csv` | cut-off (c): 20 % for 60 s, `FIRMWARE_RESTART` at about 23 s; the file ends at the restart |
| `cut-c2-restart-hot.csv` | cut-off (c2): 35 %, logged up to the `FIRMWARE_RESTART` at the right NTC ≈ 48 °C |
| `cut-c2-after.csv` | cut-off (c2) after the restart: the gate idle, the fans held by the fan rule with no owner, until both NTCs < 42 °C |

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the logger started, about one row a second |
| `state` | — | the heater's state: `leased` (a bench lease in force: heating) or `idle` |
| `duty` | % | the lease's duty (0 when idle) |
| `fired` | half-cycles | gate pulses fired since the image booted — **cumulative across the runs of one boot**, reset by a restart; the difference between two rows is the half-cycles fired between them |
| `reason` | — | the heater's latched fault, `ok` in every row (it never latched) |
| `zc_hz` | Hz | the mains frequency as measured from the zero-cross (60.0 on 60 Hz mains; two half-cycles per cycle) |
| `ntc1_c` | °C | the **right** outlet NTC (PA4) |
| `ntc2_c` | °C | the **left** outlet NTC (PA5) |
| `env_c`, `env_rh` | °C, % | the chamber sensor |
| `cutout` | 0/1 | the thermal-cutout fault input — 0 in every row |
| `fan_l`, `fan_r` | 0/1 | the left and right fan pins read back high |

A cold unit's chamber sensor reads 1.2–1.8 °C above the two outlet NTCs at rest (see the first
rows of `run-10-2s.csv`); the offset was accepted as the idle state, not corrected.
