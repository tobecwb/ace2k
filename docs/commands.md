# G-code command reference

The `[ace2k]` host module adds the `ACE_*` G-code commands below. Each one answers on the Klipper
console: an `ace2k:` line, or an error that stops the macro or print that sent it.

- Parameters are written `NAME=value`. The name is case-insensitive.
- A value outside its range is rejected before anything is sent to the unit.

Where to read more:

- the ideas behind the commands (the follow, the feed-forward, the tip snag, the tag reader, the
  dryer): [features.md](features.md);
- what goes over the wire: [protocol.md](protocol.md);
- the config keys that set the defaults: [`config/ace2k.cfg`](../config/ace2k.cfg).

Conventions used in the entries:

- **Lane** is `1`–`4`, the bay number.
- **Speed** and **length** bounds come from the firmware the host is connected to. The host reads
  them from the firmware's dictionary when it connects. In the firmware as built: speed
  9–70 mm/s, one move at most 2000 mm.
- A **refusal** is a G-code error with the message shown. `ace2k: ` starts the refusals the module
  raises itself. A number outside its range (LENGTH, PARK, TEMP, SECONDS, LANE and the like) is
  rejected earlier by Klipper's own parameter error, without that prefix.
- "Firmware built without X" means the image on the unit lacks that part (`CONFIG_ACE2K_*`). The
  command then answers with that error and sends nothing.
- A `*_SET` command stages the values it changes for `SAVE_CONFIG`. They are in use at once, and
  Klipper writes them to `printer.cfg` when you run `SAVE_CONFIG`.

Commands that start a move (`ACE_FEED`, `ACE_ROLLBACK`, `ACE_UNLOAD`, `ACE_LOAD`, `ACE_ASSIST`,
`ACE_CALIBRATE_ENCODER`) share these refusals, in addition to their own:

- `ace2k: lane N MODE refused: REASON` when the unit refuses the start. `MODE` is `feed`,
  `rollback`, `unload`, `load`, `assist`, `assist_back` or `assist_both`. `REASON` is one of
  `busy` (the lane is already moving), `in_error` (clear it with `ACE_CLEAR`), `no_filament`,
  `no_link`, `bounds` or `other_lane`.
- `ace2k: no feed state report within S s (the unit reports at feed_report_hz = H Hz; check the
  link)` when the first start of a session finds no state report from the unit.
- `ace2k: SPEED must be within MIN..MAX mm/s`.
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

A waiting command (`WAIT=1`, the default where it exists) returns when the move's final event
arrives. It turns an error event into a G-code error. It holds the G-code queue, as `G4` does.

- If no final event comes within the move's budget (1.5 times the expected time plus 5 s), the
  command sends a stop and fails with
  `ace2k: lane N MODE: no completion event within S s; a stop was sent`.
- A failed move reads `ace2k: lane N MODE failed: KIND — motor X mm, filament Y mm`. The error
  kinds are `stuck`, `tangled`, `motor_stalled`, `timeout`, `assist_overrun` and
  `unload_incomplete`.
- The other final kinds (`done`, `loaded`, `unloaded`, `stopped`, `stopped_link`,
  `stopped_shutdown`, `runout`, `tail_out`, `blocked`) are reported without an error.
- The notices (`behind`, `snag`, `tail`) do not end a move.

With `WAIT=0` the command answers `ace2k: lane N MODE started` at once.

## Status and health

### `ACE_STATUS`

Prints the unit's state on the console:

- the version, and whether the link is proven;
- one line per lane: the insert switch with its millivolt reading and band, the empty reading, the
  buffer's rest and pushed switches, the encoder in mm, the motor tach count;
- the feed, tag and dryer lines, when the firmware carries them;
- the cutout and auxiliary readings, the mains frequency and presence;
- the PTC, chamber, humidity and supply readings.

The same data is in `printer.ace2k` for macros.

No parameters. No refusals of its own.

```
ACE_STATUS
```

### `ACE_HEALTH`

Reports the unit's power-on self-test: whether it passed, which checks are failing, which have
failed since boot or the last clear, and the cause of the last reset. `RUN=1` runs the test again.
`CLEAR=1` clears the latched faults, then reports.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `RUN` | 0 | integer, nonzero runs | re-run the self-test; answers `self-test started; results in about 2 s` and returns, and the result appears in the next `ACE_HEALTH` and in `printer.ace2k.health` |
| `CLEAR` | 0 | integer, nonzero clears | clear the latched faults before reporting |

Refused when:

- `ace2k: firmware built without CONFIG_ACE2K_HEALTH`;
- with `RUN=1` and no self-test read yet: `ace2k: <reason>; self-test not started`, if the unit
  does not answer the first read.

Without `RUN`, a query the unit does not answer is not an error. The verdict then reads
`unknown (no answer from the unit)`.

```
ACE_HEALTH RUN=1
```

### `ACE_COUNTERS_RESET`

Zeroes the encoder and motor tach counters of one lane, or of all lanes. The counters measure how
far the moves went, so the reset is refused while a lane moves.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | `ALL` | 1–4 or `ALL` | the lane to zero |

Refused when:

- `ace2k: firmware built without CONFIG_ACE2K_LANE`;
- `ace2k: a lane is moving or busy; the counters are the moves' odometers`;
- `ace2k: counters reset refused: a lane is moving or busy` (the unit's own check);
- `ace2k: this firmware has no counters reset` (an older image).

A firmware that takes the reset without confirming it answers
`counters reset sent (...); this firmware does not confirm it`.

```
ACE_COUNTERS_RESET LANE=2
```

### `ACE_CALIBRATION_SAVE`

Stages the insert-switch thresholds the unit is using as `laneN_insert_mv` keys. After
`SAVE_CONFIG` they appear in `printer.cfg`, and you can edit them there.

No parameters. If the unit has not reported its thresholds yet, the command prints a console
warning, not an error: `!! ace2k: thresholds not received yet`.

```
ACE_CALIBRATION_SAVE
SAVE_CONFIG
```

## Lanes and moves

### `ACE_FEED`

Feeds a lane's filament toward the printer by a length, at a speed, then stops. The length is
measured on the lane's motor and checked against its encoder.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | required | above 0, at most 2000 mm | mm to feed |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACE_FEED LANE=1 LENGTH=100 SPEED=50
```

### `ACE_ROLLBACK`

Rewinds a lane's filament by a length. When the end of the filament leaves the motor, the unit
runs a short extra move past it (the tail move). A waiting command counts that move in its budget.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | required | above 0, at most 2000 mm | mm to rewind |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACE_ROLLBACK LANE=1 LENGTH=50
```

### `ACE_UNLOAD`

Rewinds a lane until the filament leaves the entrance of the bay. `LENGTH` is only a budget. The
move ends when the filament has left, or fails with `unload_incomplete` if the budget runs out
first.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | 0 | 0–2000 mm | the budget; 0 leaves it to the firmware's limit of 2000 mm |
| `SPEED` | `unload_speed` (30) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACE_UNLOAD LANE=1
```

### `ACE_LOAD`

Pulls a lane's filament from the bay in to the parking point. If the firmware carries the tag
reader, the load reads the tag on the way. A waited load allows for the tag search past the
parking point and back.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | `load_park_mm` (300) | above 0, at most 2000 mm | the parking distance |
| `SPEED` | `load_speed` (30) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACE_LOAD LANE=3 WAIT=0
```

### `ACE_STOP`

Stops the motor of one lane, or of every lane. A stop never clears an error state.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | all lanes | 1–4 | the lane; absent stops every lane |

Answers `ace2k: stop sent to lane N` or `every lane`. Refused:
`ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_STOP LANE=2
```

### `ACE_CLEAR`

Takes a lane out of its error state (after `stuck`, `tangled`, `motor_stalled` and the like), so
the lane accepts a start again.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |

Refused: `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_CLEAR LANE=2
```

### `ACE_SPEED`

Changes the speed of a move that is running on a lane.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | the new speed. With `SPEED` absent the speed becomes the configured `feed_speed`, not "unchanged" |

Refused: `ace2k: SPEED must be within MIN..MAX mm/s`;
`ace2k: firmware built without CONFIG_ACE2K_FEED`. The command does not check that a move is
running.

```
ACE_SPEED LANE=1 SPEED=20
```

## Assist, follow and feed-forward

In short:

- the **assist** watches the buffer's plunger and keeps it at rest;
- the **follow** is the assist in both directions (`ACE_ASSIST DIR=BOTH`);
- the **feed-forward** adds doses ahead of the pull, from Klipper's planned moves. It works only
  while the lane is in the follow.

The whole picture, with an example, is in
[`features.md`](features.md#how-the-unit-keeps-up-with-the-print-head), "How the unit keeps up
with the print head".

| | Watches | Moves the filament | Set with |
|---|---|---|---|
| Assist, forward | the buffer | toward the head only | `ACE_ASSIST` |
| Assist, back | the buffer | back only | `ACE_ASSIST DIR=BACK` |
| **Follow** | the buffer | both ways | `ACE_ASSIST DIR=BOTH`, tuned by `ACE_FOLLOW_SET` |
| Feed-forward | Klipper's planned extruder moves | toward the head, ahead of the pull | `ACE_FEED_FORWARD`, `ACE_FF_SET` — only during the follow |

### `ACE_ASSIST`

Arms a lane's buffer assist. The assist feeds the filament while the buffer asks for more, until
it is stopped. `DIR` picks the kind:

- forward (the default);
- `BACK`, the reverse;
- `BOTH`, the follow. It keeps the buffer at rest whichever way the head moves the filament.

`OFF=1` disarms it.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `DIR` | forward | `FORWARD`, `BACK`, `BOTH` (case-insensitive) | the kind of assist |
| `SPEED` | `assist_speed` (50) | 9–70 mm/s | assist speed |
| `OFF` | 0 | integer, nonzero disarms | stop the lane's assist (sends a stop) |
| `SOFT` | 0 | 0–1 | 1 reports a refusal as `ace2k: lane N not armed — <reason>` instead of raising, for print hooks; `OFF=1` is the same with or without it |

Refused:

- `ace2k: DIR must be BACK or BOTH (or absent for the forward assist)`;
- the shared refusals above;
- for `DIR=BOTH` on an image without the follow:
  `ace2k: the unit's firmware has no follow tail (or no follow); flash v0.11.0`.

```
ACE_ASSIST LANE=2 DIR=BOTH SOFT=1
```

### `ACE_FOLLOW_SET`

Sets the follow's three values. With no parameter, it prints the values in effect. Each value is
rounded to the unit's resolution before it is checked and sent.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `FLIP_MS` | 200 (`follow_flip_ms`) | 50–2000 ms, integer | how long the follow waits before a move against the last one |
| `TAKE_MM` | 15 (`follow_take_mm`) | 5–30 mm | the take-up on a lane's full |
| `TAIL_MM` | 2000 (`follow_tail_mm`) | 200–2000 mm | how far the motor runs after a runout before the follow ends |

The defaults and bounds are the firmware's, read from its dictionary at connect. Refused:

- `ace2k: NAME (value) must be within LOW..HIGH`;
- `ace2k: the unit's firmware has no follow tail (or no follow); flash v0.11.0`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_FOLLOW_SET FLIP_MS=300
```

### `ACE_FEED_FORWARD`

Switches the feed-forward on or off, for one lane or every lane. The feed-forward takes the
extrusion Klipper has planned and feeds it to the lane in the follow, in small doses, ahead of the
pull. With no parameter, the command prints each lane's state and mapped extruder.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | all lanes | 1–4 | the lane |
| `ON` | 0 | 0–1 | 1 switches on |
| `OFF` | 0 | 0–1 | 1 switches off |

Refused:

- `ace2k: ON=1 or OFF=1, not both`;
- `ace2k: the unit's firmware has no feed-forward; flash v0.9.0`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

The initial state comes from the `feed_forward` key (on by default).

```
ACE_FEED_FORWARD LANE=1 OFF=1
```

### `ACE_FF_SET`

Sets the feed-forward's values: two that the unit uses (dose size and speed) and three that the
host uses. With no parameter, it prints the values in effect. Every given value is checked before
any is applied.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `CHUNK_MM` | 3 (`ff_chunk_mm`) | 1–10 mm | a dose's length (the unit's) |
| `PULSE` | 15 (`ff_pulse`) | 9–70 mm/s | a dose's speed (the unit's) |
| `WINDOW_MS` | 500 (`ff_window_ms`) | 100–5000 ms, integer | the planner window the host meters over |
| `LEAD_MS` | 0 (`ff_lead_ms`) | 0–5000 ms, integer | how far ahead of the extruder the doses run; a new lead restarts each extruder's sweep |
| `DEADBAND` | 0.3 (`ff_deadband`) | 0.0–10.0 mm/s | the rate below which no dose is sent |

Refused:

- `ace2k: NAME (value) must be within LOW..HIGH` (or `must be a finite number`);
- `ace2k: the unit's firmware has no feed-forward; flash v0.9.0`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_FF_SET CHUNK_MM=2 LEAD_MS=200
```

### `ACE_SNAG_SET`

Sets the tip snag's six values. A filament tip can catch at the entrance of the plunger's tube.
These values set the limited tolerance that lets a feed, a load or an unload go on past it. With
no parameter, the command prints the values in effect.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `FWD` | 20 (`snag_fwd_mm`) | 5–50 mm | motor travel past a full before the touch |
| `LAG` | 5 (`snag_lag_mm`) | 1–20 mm, under `FWD` | the filament behind the motor that counts as a jam |
| `DUTY` | 0 (`snag_duty_pct`) | 0–100, integer | duty above the feed-forward that counts as a jam; 0 is off |
| `BACK` | 5 (`snag_back_mm`) | 2–20 mm | the touch |
| `REST` | 500 (`snag_rest_ms`) | 100–2000 ms, integer | the wait for the plunger at rest |
| `FREE` | 50 (`snag_free_mm`) | 20–200 mm | in reverse, the filament back before a taut trip is forgiven |

The defaults are the firmware's, read at connect. Refused:

- `ace2k: NAME (value) must be within LOW..HIGH`;
- `ace2k: LAG (x) must be under FWD (y) mm`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_SNAG_SET FWD=25 LAG=6
```

## Calibration

### `ACE_CALIBRATE_ENCODER`

Calibrates a lane's encoder scale (mm of filament per encoder count). There are two ways.

**By hand** (`AUTO` absent):

1. `LENGTH=` feeds that much and keeps the encoder's reading.
2. Measure the filament that came out.
3. `MEASURED=` computes the scale and stages `laneN_encoder_scale` for `SAVE_CONFIG`. The unit
   takes it at the restart.

**Automatic** (`AUTO=1`): the command arms the follow on the lane if it is idle. It extrudes
through the lane's extruder in 25 mm chunks. It fits a line through the marks the encoder and the
extruder give, and puts the new scale in use at once (staged for `SAVE_CONFIG`). It needs:

- the lane's `laneN_extruder`;
- that extruder active, and hot enough to extrude;
- a filament in the lane.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `AUTO` | 0 | 0–1 | 1 runs the automatic calibration |
| `LENGTH` | 200 by hand, 500 automatic | by hand above 0 to 2000 mm; automatic 200–2000 mm | by hand the feed; automatic the extruded length |
| `SPEED` | 4 (automatic only) | above 0 mm/s | the extrusion speed |
| `MEASURED` | none | above 0 mm | by hand, the measured filament; the scale is `MEASURED` over the counts the feed read |

By hand, the feed runs at `feed_speed`, and the command waits for it. The new scale must lie within
the unit's window, 0.9874–1.4810 mm per count (plus or minus 20 % of 1.2342).

Refused (by hand):

- the shared refusals above;
- `ace2k: lane N calibration feed blocked — ...; clear the path and feed again`;
- `ace2k: run ACE_CALIBRATE_ENCODER LANE=n LENGTH=<mm> first` (no feed kept for the lane);
- `ace2k: the calibration feed moved no filament; nothing to scale`;
- `ace2k: lane N: X mm/count is outside the window the unit accepts, ...; nothing staged — check
  the measurement`.

Refused (automatic), each before anything is sent:

- `ace2k: a calibration is already running`;
- `ace2k: the unit's firmware has no feed-forward (its taut count); flash v0.9.0`;
- `ace2k: a print is printing; calibrate between prints` (or `paused`);
- `ace2k: lane N has no extruder (laneN_extruder)`;
- `ace2k: lane N: no extruder NAME`;
- `ace2k: NAME is not the active extruder; select it first`;
- `ace2k: NAME is too cold to extrude; heat it for the filament`;
- `ace2k: lane N has no filament`;
- `ace2k: lane N is in error (...); ACE_CLEAR first`;
- `ace2k: lane N is busy (MODE)` (a mode other than idle or following);
- `ace2k: lane N did not enter the follow`.

After the run:

- `ace2k: lane N: aborted — REASON; nothing changed` (the buffer went full, the lane left the
  follow, Klipper shut down);
- or a refusal of the result with `nothing changed`: fewer than 10 marks, the encoder did not
  follow the extruder, scatter above 1 mm, or a scale outside the unit's window.

A change above 3 % is applied, with a warning to check the extruder's own calibration.

```
ACE_CALIBRATE_ENCODER LANE=1 AUTO=1
ACE_CALIBRATE_ENCODER LANE=1 LENGTH=200
ACE_CALIBRATE_ENCODER LANE=1 MEASURED=198.5
```

## Tags

### `ACE_RFID_READ`

Reads the RFID tag of a lane's spool. By default it reads the tag in front of the antenna. With
`MOVE=1` it turns the spool to find the tag: it feeds the filament out up to `rfid_search_mm`, and
back. A lane already read is read again. The command prints the outcome and the decoded spool
record. A waited command waits 5 s (tag in front of the antenna) or the move's budget (`MOVE=1`).

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `MOVE` | 0 | 0–1 | 1 searches by moving the filament |
| `DUMP` | 0 | 0–1 | 1 saves the tag's raw bytes as `ace2k_tag_laneN_<date-time>.json` in the log directory |
| `WAIT` | 1 | 0–1 | 0 returns at once (`tag read started`) |

Refused:

- `ace2k: lane N tag read refused: REASON`, with `REASON` `busy`, `no_filament`, `no_link`,
  `other_lane_moving` or `unsupported`;
- `ace2k: lane N: no tag outcome within S s`;
- for `MOVE=1`, `ace2k: lane N tag search failed: KIND; ...` when the load ends in an error kind;
- `ace2k: firmware built without CONFIG_ACE2K_RFID_READ`.

Two lanes share each antenna. Without `MOVE`, the answer is `ambiguous` when the other lane on the
same antenna holds a spool whose tag has not been read. The command then asks for `MOVE=1`.

```
ACE_RFID_READ LANE=1 MOVE=1
```

### `ACE_RFID_FORGET`

Forgets the tag read on a lane, in the unit and in the host, so the next move reads it again.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |

Refused: `ace2k: firmware built without CONFIG_ACE2K_RFID_READ`.

```
ACE_RFID_FORGET LANE=1
```

### `ACE_LOAD_SET`

Sets the automatic load's values: what the unit does when you insert a filament. The values are
staged for `SAVE_CONFIG`. The command sends the three values to the unit and prints them.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `PARK` | `load_park_mm` (300) | above 0, at most 2000 mm | the parking distance |
| `SPEED` | `load_speed` (30) | 9–70 mm/s | the load speed |
| `AUTO` | `auto_load` (on) | 0–1 | 1 pulls an inserted filament in by itself |

Refused: `ace2k: SPEED must be within MIN..MAX mm/s`;
`ace2k: firmware built without CONFIG_ACE2K_FEED`. A rejected value changes nothing. With no
parameter, the command sends the current values again and prints them.

```
ACE_LOAD_SET PARK=290 AUTO=1
```

### `ACE_GRIP`

Sets a lane's grip. The lane's next automatic load then pulls only that far, without the tag
search, and ends loaded. Use it for a spool whose tag cannot be read.

- The load uses up the grip.
- Any start on the lane clears it.
- It does not survive a host restart.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `MM` | the firmware's default (40) | 0, or 10–45 mm | the grip; 0 clears it |

Refused:

- `ace2k: a grip of X mm is outside the unit's 10..45 mm (0 clears)`;
- `ace2k: the unit's firmware has no grip; flash v0.11.0`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACE_GRIP LANE=3 MM=30
```

## Dryer, fans and flaps

The unit controls the heater and every protection. A cycle runs to its end even with no host.
Every refusal below comes from the unit, not from the host.

### `ACE_DRY`

Starts a drying cycle at a chamber target, for a duration.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `TEMP` | required | 15–65, integer degrees C | the target |
| `DURATION` | required | 1 minute to 24 hours: a bare number is minutes, or `90m`, `4h`, `1.5h` | the cycle length |

Refused:

- an out-of-range `TEMP`: Klipper's own parameter check rejects it before the module runs
  (`Error on 'ACE_DRY': TEMP must have minimum of 15` / `maximum of 65`);
- `TEMP` absent: the module's own `ace2k: ACE_DRY needs TEMP=<15-65>`;
- `ace2k: ACE_DRY needs DURATION=<minutes, or with an h / m suffix>, 1 min to 24 h`;
- `ace2k: dryer start refused: TEXT`, with TEXT as in the table below;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT` or `CONFIG_ACE2K_DRYER`;
- `ace2k: ...: no answer from the unit (...)`.

The table lists the refusal texts of `ACE_DRY`, `ACE_DRY_STOP` and `ACE_DRY_CLEAR`. The console
prints them after `ace2k: dryer start refused: ` (or `stop` / `clear`). A reason code not in the
table prints `reason N`.

| Code | Text |
|---|---|
| 1 | busy — a cycle or its cool-down is running (ACE_DRY_STOP first), or the fans are held by a manual run (ACE_FAN ON=0 first) |
| 2 | out of range — 15–65 °C, 1 min to 24 h |
| 3 | the dryer is in a fault — ACE_DRY_CLEAR once it is cool |
| 4 | a temperature sensor is not valid |
| 5 | the mains is absent or its frequency implausible |
| 6 | the thermal cutout tripped — power-cycle the unit |
| 7 | the heater is still cooling down |
| 8 | the unit went through a Klipper shutdown — FIRMWARE_RESTART before drying again |
| 9 | the mains has just come back — its frequency is being measured; try ACE_DRY again in a few seconds |

```
ACE_DRY TEMP=55 DURATION=4h
```

### `ACE_DRY_STOP`

Stops the cycle. The unit cools the heater down with the fans before it releases them.

No parameters. Refused: `ace2k: dryer stop refused: TEXT` (table under `ACE_DRY`); the
firmware-built-without errors.

```
ACE_DRY_STOP
```

### `ACE_DRY_CLEAR`

Clears a dryer fault once the heater is cool and the cause is gone. A tripped thermal cutout does
not clear this way: it needs a power cycle.

No parameters. Refused: `ace2k: dryer clear refused: TEXT` (table under `ACE_DRY`); the
firmware-built-without errors.

```
ACE_DRY_CLEAR
```

### `ACE_DRYER_LOG`

Prints the unit's dryer log:

- cycles started and completed, and faults;
- hours of heating, and the energy estimate at `heater_watts` (default 360 W);
- a note if the thermal cutout tripped;
- the last eight faults or interrupted cycles, with their temperatures.

If the log changes while it is read, the read is repeated, up to three times.

No parameters. Refused: `ace2k: firmware built without CONFIG_ACE2K_HEAT` or `CONFIG_ACE2K_DRYER`;
`ace2k: ...: no answer from the unit (...)`.

```
ACE_DRYER_LOG
```

### `ACE_FAN`

Runs both fans by hand. There is no control for a single fan. `ON=0` releases the manual request.
It does not switch the fans off: the unit keeps them on while an outlet NTC reads above 45 C, and
while the heater holds them.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `ON` | required | 0–1 | 1 runs the fans, 0 releases the request |
| `SECONDS` | 60 | 1–600, integer | how long the fans run (with `ON=1` only) |

Refused:

- `ace2k: ACE_FAN needs ON=0 or ON=1`;
- `ace2k: fans refused: busy — the dryer holds the fans and flaps, or another flap pulse is running
  (try again)` or `out of range`;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT`.

```
ACE_FAN ON=1 SECONDS=120
```

### `ACE_FLAP`

Pulses one exhaust flap open or closed. The flap cannot report its position, so the answer says
`(no read-back: look at it)`.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `WHICH` | required | `bottom` or `rear` | the flap |
| `OPEN` | required | 0–1 | 1 opens, 0 closes |

The unit sets the pulse length. Refused:

- `ace2k: ACE_FLAP needs WHICH=bottom or WHICH=rear`;
- `ace2k: ACE_FLAP needs OPEN=0 or OPEN=1`;
- `ace2k: ACE_FLAP takes no MS=; the unit pulses for its compiled length`;
- `ace2k: WHICH flap refused: busy — ...` (the dryer holds the flaps, or another pulse is running)
  or `out of range`;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT`.

```
ACE_FLAP WHICH=rear OPEN=1
```

## Firmware

### `ACE_RESTORE_STOCK`

Resets the unit into its bootloader's recovery mode, so the factory firmware can be written back
over the cable. After it answers, stop Klipper and run `ace2k_flash.py` (below), or follow the
manual procedure in [flashing.md](flashing.md).

No parameters. The unit refuses while the dryer heats or a lane moves. This is not an error but a
console warning, `!! ace2k: refused — a subsystem is busy (heating or moving)`, and nothing
changes.

```
ACE_RESTORE_STOCK
```

## `ace2k_flash.py`

`firmware/tools/ace2k_flash.py` flashes the unit from the Klipper host:

- factory firmware to ace2k;
- ace2k to a newer ace2k;
- ace2k back to the factory firmware;
- out of the bootloader's recovery.

It runs on the host (Python 3.9+, pyserial). It identifies the unit before it changes anything,
shows a plan, and asks before it writes. The three paths, the order of its steps, its refusals and
the manual equivalent are in [flashing.md](flashing.md).

    python3 ace2k_flash.py IMAGE.bin [--port PORT] [--version V] [--moonraker URL]
                           [--klipper-stop CMD] [--klipper-start CMD]
                           [--calibration-dir DIR] [--yes]

| Argument | Default | Meaning |
|---|---|---|
| `image` | required | the image: an ace2k `.bin`, or the factory `.bin` |
| `--port` | from Klipper's config | the unit's serial port |
| `--version` | read from the image | the image's version, when the image does not carry it (always for the factory image) |
| `--moonraker` | `http://localhost:7125` | Moonraker's URL, used to read the config and Klipper's state |
| `--klipper-stop` | detected | the command that stops Klipper on this host (give both stop and start, or neither) |
| `--klipper-start` | detected | the command that starts Klipper on this host |
| `--calibration-dir` | `.` | where the factory-firmware calibration file is written |
| `--yes` | off | do not ask for confirmation |

Exit status:

- 0: flashed and verified;
- 1: refused or failed;
- 2: flashed but not verifiable on this host.
