# Dryer 4 h cycle, 2026-09-29

A PETG spool dried at 65 °C for 4 h by the release image R3 (91 964 B, sha256 `fbc256e2…`)
on the development unit, at 127 V / 60 Hz, lid closed. This page says what the columns are. As in `../dryer-b3/`, two kinds of file: the dryer log, read
from the unit by the host, and the plug log, read from a smart plug's energy meter the unit was
plugged into.

**Two clocks.** The dryer log's `wall` is **UTC** (the host container's clock); the plug log's
`wall` is **local time, UTC−3**. 22:58:26 in `petg-65c-4h.csv` (the first `heating` row) is
19:58:26 in `power-petg-65c-4h.csv`.

**Read `ntc1` as the RIGHT outlet NTC (PA4) and `ntc2` as the LEFT one (PA5)**, as in
`../dryer-b3/`; `docs/hardware.md` ("Analogue inputs") names them by side.

## Files

| File | Image | What |
|---|---|---|
| `petg-65c-4h.csv` | R3 | 6 s of idle, `ACE_DRY TEMP=65 DURATION=240` from a warm unit, 4 h of heating (the vent at 100 min), the 10 min cool-down ending on the `hot_ambient` hand-over, the fans held by the fan rule until they stop, a few seconds of idle |
| `power-petg-65c-4h.csv` | — | the plug through `petg-65c-4h.csv` |

## The dryer log

One row about every second, the same columns as `../dryer-b3/16d-45c-10min.csv`.

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the logger started |
| `wall` | — | UTC time of the row |
| `state` | — | the dryer: `idle`, `starting`, `heating`, `cooldown` (no `fault` in this file) |
| `target_c` | °C | the cycle's target; `None` when `idle` |
| `drive_c` | °C | the outlet target the inner loop drives the hotter NTC to (target + 7 °C, trimmed by the chamber loop, capped at target + 8 °C and 73 °C); `None` outside `starting` / `heating` |
| `duty` | % | the heater's duty |
| `remaining_min` | min | what is left of the cycle; `None` when `idle` |
| `fault` | — | the dryer's fault; `None` in every row of this file |
| `notices` | — | the dryer's notices, by name, separated by a vertical bar, empty when none (only `hot_ambient` in this file, from the end of the cool-down) |
| `ntc1_c` | °C | the **right** outlet NTC (PA4) |
| `ntc2_c` | °C | the **left** outlet NTC (PA5) |
| `env_c`, `env_rh` | °C, % | the chamber sensor: temperature and relative humidity |
| `zc_hz` | Hz | the mains frequency, measured from the zero-cross edges' own times, in steps of 0.1 Hz; 59.9 or 60.0 in every row |
| `rejects` | edges | zero-cross edges refused by the input's lockout **since boot** — cumulative, reset by a restart |
| `cutout` | 0/1 | the thermal-cutout fault input — 0 in every row |
| `fan_l`, `fan_r` | 0/1 | the left and right fan pins read back high |
| `flap_bottom`, `flap_rear` | — | `open`, `closed`, or `unknown` while a pulse on that flap runs |

## The plug log

One row every 1–3.5 s, read-only from the plug.

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the logger started |
| `wall` | — | **local time (UTC−3)** of the reading; the plug refreshes its reading more slowly than the logger asks, so the same reading can appear on two rows with the same `wall` |
| `volt_v` | V | the mains voltage at the plug, 122.5–130.1 V; 0.0 on a failed read (11 rows during the heating; their `power_w` is read normally) |
| `current_ma` | mA | the current through the plug; 0 on one of those 11 rows |
| `power_w` | W | the active power through the plug: heater, fans and the unit's electronics together. It reads 0 with the unit idle |

The energy figures derived from these files integrate `power_w` over `wall` (trapezoids) from the
start of heating to the end of the cool-down, and to the fans' stop.
