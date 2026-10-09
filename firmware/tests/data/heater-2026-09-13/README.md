# Heater runs, 2026-09-13

Four runs of the heater on the development unit, logged at 127 V / 60 Hz with the unit closed and
empty, by a test image that drove the triac gate under a fan-first interlock (both fans on before
the first gate pulse, kept on through a cool-down that ended with both outlet NTCs below 40 °C and
at least 30 s elapsed). The host re-issued 2 s heating requests about once a second, polled the
state about every 0.26–0.28 s, and read the chamber sensor every 5 s (its columns repeat between
reads). Each file starts at the first sample after the request and ends at the sample that saw the
heater idle again. The runs are the data `firmware/tools/thermal_fit.py` fits the thermal
simulator to (the dryer's thermal simulator).

**Read `ntc1` as the RIGHT outlet NTC (PA4) and `ntc2` as the LEFT one (PA5).** The test image
numbered them by ADC channel; `docs/hardware.md` ("Analogue inputs") names them by side.

| File | Duty | Heating | How it ended |
|---|---|---|---|
| `run-20-10s.csv` | 20 % | 10 s | the host's 10 s limit; then the cool-down |
| `run-20-120s.csv` | 20 % | 120 s | the host's 120 s limit; then the cool-down |
| `run-35-120s.csv` | 35 % | 73.1 s | the right NTC reached 50 °C, the host's stop; then the cool-down |
| `run-50-120s.csv` | 50 % | 39.6 s | the right NTC reached 50 °C, the host's stop; then the cool-down |

| Column | Unit | Meaning |
|---|---|---|
| `t_s` | s | time since the heating request |
| `state` | — | the test image's heater state: `HEATING`, `COOLDOWN`, `IDLE` |
| `duty` | % | the duty in force (0 once cooling) |
| `fired` | half-cycles | gate pulses fired since the image booted — cumulative across runs; the difference between two rows is the half-cycles fired between them |
| `zc_hz` | Hz | the zero-cross edge rate (two edges per mains cycle: 120 on 60 Hz mains) |
| `ntc1_mv`, `ntc1_c` | mV, °C | the **right** outlet NTC (PA4): the millivolts and the temperature the test image converted with the model of `docs/hardware.md` |
| `ntc2_mv`, `ntc2_c` | mV, °C | the **left** outlet NTC (PA5) |
| `env_c`, `env_rh` | °C, % | the chamber sensor |
| `pd15` | 0/1 | the thermal-cutout fault input — 0 in every row |
| `fans` | bits | bit 0 left fan pin high, bit 1 right fan pin high — 3 = both on |
| `gate` | 0/1 | the gate pin at the sample — a 6 ms pulse is caught only now and then |
| `fault` | — | the test image's latched fault — `none` in every row |

Clamp meter on the mains lead (average-responding, not true-RMS): 0.9–1.0 A at 35 %, 1.5 A peak at
50 % — about 2.7–3.0 A at full conduction, 340–380 W at 127 V. The 20 % reading (0.1–0.2 A) is not
usable: the meter under-reads short bursts.
