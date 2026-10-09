# Dryer cycles, 2026-09-28 and 2026-09-29

The dryer of the release image driving the development unit, at 127 V / 60 Hz, lid closed.
This page says what the columns are.
Two kinds of file sit here: the dryer logs, read from the unit by the host, and the plug logs,
read from a smart plug's energy meter the unit was plugged into.

**Two clocks.** The dryer logs' `wall` is **UTC** (the host container's clock); the plug logs'
`wall` is **local time, UTC−3**. 13:28 in `45c.csv` is 10:28 in `power-45c.csv`.

**Read `ntc1` as the RIGHT outlet NTC (PA4) and `ntc2` as the LEFT one (PA5)**, as in
`../dryer-b2/` and `../heater-2026-09-13/`; `docs/hardware.md` ("Analogue inputs") names them by
side.

The Image column names the build the file was recorded with.

## Files

| File | Image | What |
|---|---|---|
| `45c.csv` | R1 | 45 °C for 30 min from a cold unit, the cool-down, a minute of idle. No `rejects` column |
| `3e-tuya-45c-10min.csv` | R1 | a 45 °C / 10 min start on the smart plug: fault `mains` within a second of the start |
| `3f-wall-45c-10min.csv` | R1 | the same start with the unit plugged straight into the wall socket (fresh boot, `rejects` from 0): the same fault |
| `55c.csv` | R2 | 55 °C for 60 min: the host cable pulled for about 2 min at ~10 min, pulled again at ~20 min with filament pushed into lane 1 (the target lowered to 45 °C), the cool-down |
| `go5-stop.csv` | R2 | 55 °C, `ACE_DRY_STOP` at 5 min, the cool-down |
| `go6-m112.csv` | R2 | 55 °C, `M112` at 5 min, then `FIRMWARE_RESTART`, the fans held by the fan rule until they stop |
| `65c.csv` | R2 | 65 °C for 2 h from a cold unit: the vent at 70 min, the 10 min cool-down ending on the `hot_ambient` hand-over, the fans held by the fan rule until they stop |
| `16d-45c-10min.csv` | R3 | 2026-09-29: 45 °C for 10 min from a cold unit on the release that measures the mains from the edges' own times, the cool-down, a few seconds of idle |
| `power-45c.csv` | — | the plug through `45c.csv` |
| `power-55c.csv` | — | the plug through `55c.csv` |
| `power-65c.csv` | — | the plug through `65c.csv` |
| `power-3e-tuya.csv` | — | the plug through `3e-tuya-45c-10min.csv`: the fans' 30 s after the fault, nothing else |
| `power-16d-45c.csv` | — | the plug through `16d-45c-10min.csv` |

No plug log exists for `3f` (the plug was out of the circuit), `go5` or `go6`.

## The dryer logs

One row about every second: `ace2k_dryer_state`, `ace2k_env_state`, the zero-cross and the airflow
state as the host last received them.

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the logger started |
| `wall` | — | UTC time of the row; `ERR` on a row where the logger could not reach Klipper (the host restarting), the rest of the row then carries the error |
| `state` | — | the dryer: `idle`, `starting`, `heating`, `cooldown`, `fault`; `None` on the rows right after a host restart, before the first report |
| `target_c` | °C | the cycle's target (after a lowering, the lowered one); `None` when `idle` |
| `drive_c` | °C | the outlet target the inner loop drives the hotter NTC to (target + 7 °C, trimmed by the chamber loop); `None` outside `starting` / `heating` |
| `duty` | % | the heater's duty |
| `remaining_min` | min | what is left of the cycle; 0 in `fault`, `None` when `idle` |
| `fault` | — | the fault that put the dryer in `fault` (`mains` in this set), `None` otherwise |
| `notices` | — | the dryer's notices, by name, separated by a vertical bar, empty when none (`lowered`, `mains_dip`, `hot_ambient` in this set; they stay up until the next start) |
| `ntc1_c` | °C | the **right** outlet NTC (PA4) |
| `ntc2_c` | °C | the **left** outlet NTC (PA5) |
| `env_c`, `env_rh` | °C, % | the chamber sensor: temperature and relative humidity |
| `zc_hz` | Hz | the mains frequency measured over the last 1 s window, in steps of 0.5 Hz up to R2; in `16d-45c-10min.csv` (R3) measured from the edges' own times, in steps of 0.1 Hz; 59.0–61.0 is plausible on 60 Hz mains |
| `rejects` | edges | zero-cross edges refused by the input's lockout **since boot** — cumulative, reset by a restart. **Only from `3e` on**: `45c.csv` has no such column |
| `cutout` | 0/1 | the thermal-cutout fault input — 0 in every row |
| `fan_l`, `fan_r` | 0/1 | the left and right fan pins read back high |
| `flap_bottom`, `flap_rear` | — | `open`, `closed`, or `unknown` while a pulse on that flap runs; `None` before the first report after a restart |

While the host cable is out the host keeps its last reports: the rows repeat the last values
received (in `55c.csv`, `t_s` 613–745 and 1213–1272) until the host's restart row (`ERR`).

## The plug logs

One row every 1–3.5 s, read-only from the plug.

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the logger started |
| `wall` | — | **local time (UTC−3)** of the reading; the plug refreshes its reading more slowly than the logger asks, so the same reading can appear on two rows with the same `wall` |
| `volt_v` | V | the mains voltage at the plug; 0.0 on a failed read (one row in `power-3e-tuya.csv`) |
| `current_ma` | mA | the current through the plug |
| `power_w` | W | the active power through the plug: heater, fans and the unit's electronics together. It reads 0 with the unit idle |

The energy figures derived from these files integrate `power_w` over `wall` (trapezoids) from the
start of heating to the end of the cool-down.
