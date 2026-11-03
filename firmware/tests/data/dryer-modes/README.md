# Cool-downs from the printer's history, 2026-10-22 to 2026-10-25

Three dryer cycles and the hours after them, as the U1 adapter's history recorded them: one sample
of each unit every 30 s, at 127 V / 60 Hz, lid closed. The cold-box rule of the humidity mode
(`docs/features.md`, "Drying on its own") is replayed on them.

| Column | What |
|---|---|
| `t_s` | seconds from the file's first row |
| `chamber_c` | the chamber sensor, °C |
| `rh_pct` | the chamber's relative humidity, %; empty when the history had none |
| `state` | the dryer's state: `idle`, `starting`, `heating`, `cooldown`, `fault` |

The history has gaps: the printer was off from about 20:00 on 2026-10-23 to 15:20 on 2026-10-24.
A replay holds the last row through a gap.

## Files

| File | What |
|---|---|
| `cool-24h-45c.csv` | 24 h at 45 °C during a print, then the closed box for a day and a half. The chamber 45 → 28.8 °C, RH 26.5 % when the rule reads it cold |
| `cool-30min-55c.csv` | 30 min at 55 °C with no spool, then about 5 h. RH 68.3 % when it reads cold |
| `cool-10min-45c.csv` | 10 min at 45 °C, then about 3 h. RH 57.4 % when it reads cold |

## Where the rule reads cold

Reduced 2026-10-26 with the rule of the brief: one sample a minute from the first idle minute, a
window of 31 samples (30 min), cold when the window is full and the chamber is steady across it:
it neither fell nor rose 0.5 °C. The first reduction asked only that it fell less than 0.5 °C, which
also read a warming chamber as cold. The steady rule gives the same hours on all three files,
because no chamber here warms by 0.5 °C over a window after the cycle.

| File | cold, after the dryer went idle |
|---|---|
| `cool-24h-45c.csv` | 3.92 h |
| `cool-30min-55c.csv` | 2.95 h |
| `cool-10min-45c.csv` | 1.92 h |
