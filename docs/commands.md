# G-code command reference

The `[ace2k]` host module adds the `ACEK_*` G-code commands below. Each one answers on the Klipper
console: an `ace2k:` line, or an error that stops the macro or print that sent it.

- Parameters are written `NAME=value`. The name is case-insensitive.
- Every command also takes `UNIT=`, which the tables do not repeat. The rules are in
  [Several units](#several-units).
- A value outside its range is rejected before anything is sent to the unit.

Where to read more:

- the ideas behind the commands (the follow, the feed-forward, the tip snag, the tag reader, the
  dryer): [features.md](features.md);
- what goes over the wire: [protocol.md](protocol.md);
- the config keys that set the defaults: [`config/ace2k.cfg`](../config/ace2k.cfg).

Up to v0.12.0-rc.1 the commands were named `ACE_*`. Each one keeps its name after the prefix:
`ACE_FEED` is now `ACEK_FEED`, `ACE_DRY` is now `ACEK_DRY`. The old names have no alias. This
leaves them free for other ACE modules, since Klipper stops when two modules register the same
command. Rename the commands in your macros, your slicer's G-code and `printer.cfg`.

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
- A command that asks the unit waits for its answer. One question is in flight at a time; the
  others wait their turn. When the unit does not answer, the command fails with
  `ace2k: WHAT: no answer from the unit; check the link (...)`. `WHAT` names the question, for
  example `lane 1 feed`, `counters reset` or `ace2k_dryer_start`. `ACEK_HEALTH` without `RUN`
  reports the health unknown instead.
- A `*_SET` command stages the values it changes for `SAVE_CONFIG`. They are in use at once, and
  Klipper writes them to `printer.cfg` when you run `SAVE_CONFIG`.

Commands that start a move (`ACEK_FEED`, `ACEK_ROLLBACK`, `ACEK_UNLOAD`, `ACEK_LOAD`, `ACEK_ASSIST`,
`ACEK_CALIBRATE_ENCODER`) share these refusals, in addition to their own:

- `ace2k: lane N MODE refused: REASON` when the unit refuses the start. `MODE` is `feed`,
  `rollback`, `unload`, `load`, `assist`, `assist_back` or `assist_both`. `REASON` is one of
  `busy` (the lane is already moving), `in_error` (clear it with `ACEK_CLEAR`), `no_filament`,
  `no_link`, `bounds` or `other_lane`.
- `ace2k: no feed state report within S s (the unit reports at feed_report_hz = H Hz; check the
  link)` when the first start of a session finds no state report from the unit.
- `ace2k: lane N MODE: no answer from the unit; check the link (...); lane N stopped in case it
  started` when the unit does not answer the start. The move may have started, so the lane is
  stopped.
- `ace2k: lane N MODE: the unit answered for lane M; lane N stopped in case it started` for a
  stale answer. The lane is stopped for the same reason.
- `ace2k: lane N MODE refused: reason R` for a reason this host does not know.
- `ace2k: SPEED must be within MIN..MAX mm/s`.
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

A waiting command (`WAIT=1`, the default where it exists) returns when the move's final event
arrives. It turns an error event into a G-code error. It holds the G-code queue, as `G4` does.

- If no final event comes within the move's budget (1.5 times the expected time plus 5 s), the
  command sends a stop and fails with
  `ace2k: lane N MODE: no completion event within S s; a stop was sent`.
- A failed move reads `ace2k: lane N MODE failed: KIND — motor X mm, filament Y mm`. The error
  kinds are `stuck`, `tangled`, `motor_stalled`, `timeout`, `assist_overrun` and
  `unload_incomplete`. `stopped_sensors` fails the command the same way, though it leaves the lane
  idle and not in error: the unit stopped the lane because its insert sensors are stale (the ADC
  scan stopped), and it refuses every start (`sensors_stale`) until they read again.
- The other final kinds (`done`, `loaded`, `unloaded`, `stopped`, `stopped_link`,
  `stopped_shutdown`, `runout`, `tail_out`, `blocked`) are reported without an error.
- The notices (`behind`, `snag`, `tail`) do not end a move.

With `WAIT=0` the command answers `ace2k: lane N MODE started` at once.

## Several units

A printer can drive several units. Each is a section `[ace2k <name>]` with its own `mcu:`
([features.md](features.md#several-units)). The commands below are the same on every unit. These
rules tell the host which unit a command is for.

- **`UNIT=name`** picks the unit. Every command takes it. The name is the one in the section,
  compared without regard to case. `LANE` stays `1`–`4`, inside that unit: `UNIT=b LANE=3` is
  lane 3 of unit b.
- **With one unit**, `UNIT=` is optional. When you give it, it must name that unit. A config with
  a single `[ace2k]` section has no name, and nothing changes: no `UNIT=`, the same console lines.
- **With several units and no `UNIT=`**, three commands given no other parameter run on every
  unit, in the order of the config:
  - `ACEK_STOP` stops every lane of every unit;
  - `ACEK_STATUS` and `ACEK_HEALTH` print each unit in turn.

  Given any other parameter they are about one unit, so without `UNIT=` they are refused like any
  other command, and nothing runs: `ACEK_STOP LANE=n` names a lane, and `ACEK_HEALTH RUN=1` or
  `ACEK_HEALTH CLEAR=1` would run the self-test or clear the faults of a unit you did not name:
  `ace2k: ACEK_HEALTH needs UNIT=<a|b>: this printer has several units`.

  Any other command is refused and sends nothing, because it would move a lane or heat a unit that
  you did not name:
  `ace2k: ACEK_FEED needs UNIT=<a|b>: this printer has several units`. The `<a|b>` lists the
  names.
- **An unknown name** is refused: `ace2k: no unit x; units: a, b`. A single `[ace2k]` has no
  name to give: `ace2k: no unit x; this printer has one unit: omit UNIT=`.
- **An empty `UNIT=`** (or one of spaces only) counts as no `UNIT=` at all.
- **A command that fails on one unit during a run on every unit** does not stop the others: a
  refusal, or an internal error (a link that drops while `ACEK_STOP` sends). Once every unit has
  run, each unit's refusal is printed as an error line (`!! ace2k a: no answer`), and the last
  one is the command's error. Every such line names its unit, Klipper's own errors too, which
  lack the `ace2k:` start (`!! ace2k a: Error on …`). An internal error is not turned into a refusal: once every unit has
  run and the refusals are printed, the first one goes on to Klipper as it came, and Klipper
  shuts down on it, as it does when the command names the unit.
- **The console names the unit.** With several units every line a unit prints starts with the
  section name: `ace2k b: lane 3 follow on (follow_default)`. With one unit, lines start with
  `ace2k:` as before. The text of a refusal is the same, with that start. A line that starts
  otherwise with one unit takes the unit after `ace2k`: `ace2k b: dryer: 2 cycles started, …`
  (`ACEK_DRYER_LOG`), `ace2k b: 0.12.0, link proven: yes` (`ACEK_STATUS`), `ace2k b: health: OK`
  (`ACEK_HEALTH`). klippy.log names the unit the same way, and its lane references read `b:3`
  (`ace2k b: lane b:3 tag …`, `ace2k b: dryer: a filament was inserted on lane b:2 …`).
- **Lanes in the coordinator's lines** read `a:3`, the unit and the lane, when there are several
  units (`ace2k a: lanes a:1, b:3 share extruder: choose one with ACEK_LANE_ACTIVE`), and `3` with
  one.
- **`ACEK_LANE_ACTIVE`** works across units: one extruder can be fed by lanes of different units,
  and the choice is for the whole printer ([`ACEK_LANE_ACTIVE`](#acek_lane_active)). Until a lane
  is chosen, none of them follows: not by default, not with `ACEK_ASSIST DIR=BOTH`, not from a
  host module such as the U1 adapter.

Mistakes in the config stop Klipper at its start, with a message that names the section:

- `[ace2k]` together with `[ace2k <name>]`. Name every unit, or none;
- two sections with the same name (`ace2k: two sections name unit a`);
- a name with anything but letters, digits, `_` and `-`
  (`ace2k: unit name 'left unit' may only hold letters, digits, '_' and '-'`);
- a named section without `mcu:`;
- two sections on one `mcu:`: one MCU is one unit;
- a `[temperature_sensor]` of type `ace2k` with several units and no `unit:`, with a `unit:` that
  names no unit, or placed before a unit section: it must come after every unit section. The
  wording is the commands' own, after the sensor's name:
  `ace2k: sensor temperature_sensor x: needs unit:<a|b>: this printer has several units`,
  `ace2k: sensor temperature_sensor x: no unit c; units: a, b`. An empty `unit:` counts as none.
  With one `[ace2k]`, any `unit:` (`ace2k` too) is refused:
  `ace2k: sensor temperature_sensor x: no unit ace2k; this printer has one unit: omit unit:`.

When you rename `[ace2k]` to `[ace2k a]` to add a second unit, rename the saved block at the end
of `printer.cfg` too. `SAVE_CONFIG` writes values there, the encoder calibration among them, under
`#*# [ace2k]`. Change that line to `#*# [ace2k a]`. A saved `[ace2k]` left beside `[ace2k a]` is
the two forms together, and Klipper stops at start with
`ace2k: [ace2k] together with [ace2k <name>]: …`.

## Status and health

### `ACEK_STATUS`

Prints the unit's state on the console:

- the version, and whether the link is proven;
- on the next line, `uid` and the unit's identifier, the value for the `uid:` key of its section
  ([features.md](features.md#several-units)); absent on a firmware built without it;
- one line per lane: the insert switch with its millivolt reading and band, the empty reading, the
  buffer's rest and pushed switches, the encoder in mm, the motor tach count;
- the feed, tag and dryer lines, and the dryer's mode line, when the firmware carries them; for each extruder that several
  lanes feed, a line naming its lanes and the active one (`extruder: lanes 1, 2 — active lane 1`,
  or `active none (ACEK_LANE_ACTIVE)`). Each lane's feed line ends with `follow=`: the follow's
  state, or `off (<why>)` when something keeps it off. A lane that follows by another start
  than `follow_default`'s says who started it: `armed (by a host module)` (the U1 adapter's),
  `armed (by a G-code)` (`ACEK_ASSIST DIR=BOTH`), `armed (by ACEK_LANE_ACTIVE)`,
  `armed (by ACEK_CALIBRATE_ENCODER)`, `armed (by the unit)` (a follow the unit started itself)
  or `armed (by an earlier start)` (one from before a Klipper restart). Examples of `off`:
  `follow=off (held: not in [ace2k_u1] lanes)` or
  `follow=off (feeds no extruder, follow_lanes: extruder)`
  ([features.md](features.md#the-follow-is-on-by-default));
- the cutout and auxiliary readings, the mains frequency and presence;
- the PTC, chamber, humidity and supply readings.

The same data is in `printer.ace2k` for macros. `printer.ace2k.sensors_reports` counts the
sensor reports received since Klipper started (10 a second on a proven link). It tells two
readings of the switches apart: a reading with the same count is the same report.

No parameters. No refusals of its own.

```
ACEK_STATUS
```

### `ACEK_HEALTH`

Reports the unit's power-on self-test: whether it passed, which checks are failing, which have
failed since boot or the last clear, and the cause of the last reset. Its last line is the longest
silence of the host on the link since boot or the last clear, beside the silence that the unit
takes for a lost link: `longest host silence since boot or last clear: 1020 ms (the link is lost
at 3000 ms)`. An image without the measure reads `n/a`. Klipper asks the unit's clock about once
a second, so a value near 1000 ms is the normal one; a value close to the lost window is a host
that came near to a lost link. A silence the unit lived through while Klipper itself restarted
counts too, so clear it once Klipper is connected before measuring. `RUN=1` runs the test again.
`CLEAR=1` clears the latched faults and the longest silence, then reports. A clear sets the
latched faults to what the last pass fails. So with both, `CLEAR=1 RUN=1`, the order is: run the test, read the new result,
then clear. A new pass that is good then reads `none`, and one that fails keeps only its own
failures. If the unit never reports a new pass, nothing is cleared, because a clear then would
keep what the old pass failed. The console then says
`ace2k: health not cleared: the unit did not report a new self-test`. If the unit does not answer
that read, nothing is cleared either, and the console says
`ace2k: health not cleared: the unit did not answer`. Run `ACEK_HEALTH CLEAR=1` alone to clear
anyway.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `RUN` | 0 | integer, nonzero runs | re-run the self-test; answers `self-test started; results in about 2 s` and returns, and the result appears in the next `ACEK_HEALTH` and in `printer.ace2k.health` |
| `CLEAR` | 0 | integer, nonzero clears | clear the latched faults and the longest host silence before reporting; with `RUN=1`, after the new result is read |
| `UNIT` | the only unit | a unit name | the unit; absent, with several units, reports every unit, and a `RUN=` or `CLEAR=` without it is refused |

Refused when:

- `ace2k: firmware built without CONFIG_ACE2K_HEALTH`;
- with `RUN=1` and no self-test read yet: `ace2k: <reason>; self-test not started`, if the unit
  does not answer the first read.

Without `RUN`, a query the unit does not answer is not an error. The verdict then reads
`unknown (no answer from the unit)`.

```
ACEK_HEALTH RUN=1
```

### `ACEK_COUNTERS_RESET`

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
ACEK_COUNTERS_RESET LANE=2
```

### `ACEK_CALIBRATION_SAVE`

Stages the insert-switch thresholds the unit is using as `laneN_insert_mv` keys. After
`SAVE_CONFIG` they appear in `printer.cfg`, and you can edit them there.

No parameters. If the unit has not reported its thresholds yet, the command fails with
`ace2k: thresholds not received yet` and stages nothing.

```
ACEK_CALIBRATION_SAVE
SAVE_CONFIG
```

## Lanes and moves

### `ACEK_FEED`

Feeds a lane's filament toward the printer by a length, at a speed, then stops. The length is
measured on the lane's motor and checked against its encoder.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | required | above 0, at most 2000 mm | mm to feed |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | speed |
| `FREE` | 0 | 0–1 | 1 when nothing holds the tip: a resistance that presses the plunger does not stop the push while the filament follows, a filament that stops does. It is not `ACEK_SNAG_SET`'s `FREE`, the reverse free travel in mm |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACEK_FEED LANE=1 LENGTH=100 SPEED=50
```

### `ACEK_ROLLBACK`

Rewinds a lane's filament by a length. When the end of the filament leaves the motor, the unit
runs a short extra move past it (the tail move). A waiting command counts that move in its budget.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | required | above 0, at most 2000 mm | mm to rewind |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | speed |
| `FREE` | 0 | 0–1 | 1 when nothing holds the tip (it is out of the extruder): each time the tip catches on its way back, the unit frees it and goes on, as long as the filament came back at least `snag_free_mm` since the last catch. Without it, the second catch stops the rollback with an error. Never use it with the tip still in the extruder. It is not `ACEK_SNAG_SET`'s `FREE`, the reverse free travel in mm |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above. With `FREE=1` on a unit whose firmware does not read it:
`ace2k: the unit's firmware ignores FREE on a rollback; flash v0.12.0`, and nothing moves.

```
ACEK_ROLLBACK LANE=1 LENGTH=50
```

### `ACEK_UNLOAD`

Rewinds a lane until the filament leaves the entrance of the bay. `LENGTH` is only a budget. The
move ends when the filament has left, or fails with `unload_incomplete` if the budget runs out
first. Once the bay's sensor no longer sees the filament, the motor turns `unload_clear_mm` more
(20 mm by default), so the end of the filament does not stop on the sensor. That travel can go
past `LENGTH`. Nothing in that stretch is a failure: the move ends `unloaded` (see "The clear of an
unload" below).

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | 0 | 0–2000 mm | the budget; 0 leaves it to the firmware's limit of 2000 mm |
| `SPEED` | `unload_speed` (70) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACEK_UNLOAD LANE=1
```

### `ACEK_LOAD`

Pulls a lane's filament from the bay in to the parking point. If the firmware carries the tag
reader, the load reads the tag on the way. A waited load allows for the tag search past the
parking point and back.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `LENGTH` | `load_park_mm` (300) | above 0, at most 2000 mm | the parking distance |
| `SPEED` | `load_speed` (60) | 9–70 mm/s | speed |
| `WAIT` | 1 | integer | 0 returns at once |

Refused: the shared refusals above.

```
ACEK_LOAD LANE=3 WAIT=0
```

### `ACEK_STOP`

Stops the motor of one lane, or of every lane. A stop never clears an error state.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | all lanes | 1–4 | the lane; absent stops every lane |
| `UNIT` | the only unit | a unit name | the unit to stop; absent, with several units, stops every lane of every unit, and a `LANE=` without it is refused |

Answers `ace2k: stop sent to lane N` or `every lane`. A stopped lane stays out of the follow's
default until its filament is taken out and put back. Refused:
`ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACEK_STOP LANE=2
```

### `ACEK_CLEAR`

Takes a lane out of its error state (after `stuck`, `tangled`, `motor_stalled` and the like), so
the lane accepts a start again.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |

Refused: `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACEK_CLEAR LANE=2
```

### `ACEK_SPEED`

Changes the speed of a move that is running on a lane. These moves take a new speed:
`ACEK_FEED`, `ACEK_ROLLBACK`, `ACEK_UNLOAD`, `ACEK_LOAD` and the one-way `ACEK_ASSIST` (forward
or `DIR=BACK`). The follow (`ACEK_ASSIST DIR=BOTH`, or the follow that ace2k arms by default)
does not. It keeps the speed it was armed with. To change that speed, stop the follow with
`ACEK_STOP LANE=N` and arm it again with `ACEK_ASSIST LANE=N DIR=BOTH SPEED=<mm/s>`. The follow
that ace2k arms by default uses `assist_speed`.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `SPEED` | `feed_speed` (30) | 9–70 mm/s | the new speed. With `SPEED` absent the speed becomes the configured `feed_speed`, not "unchanged" |

The unit confirms the new speed or refuses it. A refused speed is an error, and the unit keeps it
for no later move.

Refused:

- `ace2k: lane N speed refused: no move running` when the lane is idle;
- `ace2k: lane N speed refused: the lane is in error`;
- `ace2k: lane N speed refused: the speed is out of bounds`;
- `ace2k: lane N speed refused: the lane is following; set the follow's speed instead` when the
  lane is in the follow. Nothing changes. With a firmware older than v0.13.0, which would take
  the speed, ace2k refuses it itself with the same text and sends nothing;
- `ace2k: lane N speed: no answer from the unit; check the link (...)`;
- `ace2k: lane N speed: the unit answered for lane M; try again`, for a stale answer;
- `ace2k: lane N speed refused: reason R` for a reason this host does not know;
- `ace2k: SPEED must be within MIN..MAX mm/s`, before anything is sent;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACEK_SPEED LANE=1 SPEED=20
```

## Assist, follow and feed-forward

In short:

- the **assist** watches the buffer's plunger and keeps it at rest;
- the **follow** is the assist in both directions (`ACEK_ASSIST DIR=BOTH`);
- the **feed-forward** adds doses ahead of the pull, from Klipper's planned moves. It works only
  while the lane is in the follow.

The whole picture, with an example, is in
[`features.md`](features.md#how-the-unit-keeps-up-with-the-print-head), "How the unit keeps up
with the print head".

The follow is on by default. At Klipper's start ace2k arms it on every lane with filament, and on a
lane whose filament you insert later, once it is free. `follow_default: False` in `[ace2k]` turns
that off, and `follow_lanes: extruder` keeps it to the lanes that feed an extruder. After `ACEK_STOP` or `ACEK_ASSIST … OFF=1` the lane stays off until its filament is taken
out and put back. A move you start on a lane in the follow (`ACEK_FEED`, `ACEK_ROLLBACK`,
`ACEK_LOAD`, a one-way `ACEK_ASSIST`, `ACEK_RFID_READ … MOVE=1`) pauses the follow and brings it
back when the move ends cleanly (`ace2k: lane N follow paused for the feed`, then
`ace2k: lane N follow on again`). `ACEK_UNLOAD` stops it for good (`follow stopped for the
unload`). When several lanes feed one extruder, only the lane chosen with
[`ACEK_LANE_ACTIVE`](#acek_lane_active) is followed.
[`features.md`](features.md#the-follow-is-on-by-default), "The follow is on by default", has the
details.

| | Watches | Moves the filament | Set with |
|---|---|---|---|
| Assist, forward | the buffer | toward the head only | `ACEK_ASSIST` |
| Assist, back | the buffer | back only | `ACEK_ASSIST DIR=BACK` |
| **Follow** | the buffer | both ways | `ACEK_ASSIST DIR=BOTH`, tuned by `ACEK_FOLLOW_SET` |
| Feed-forward | Klipper's planned extruder moves | toward the head, ahead of the pull | `ACEK_FEED_FORWARD`, `ACEK_FF_SET` — only during the follow |

### `ACEK_ASSIST`

Arms a lane's buffer assist. The assist feeds the filament while the buffer asks for more, until
it is stopped. `DIR` picks the kind:

- forward (the default);
- `BACK`, the reverse;
- `BOTH`, the follow. It keeps the buffer at rest whichever way the head moves the filament.

`OFF=1` disarms it. The lane then stays out of the follow's default until its filament is taken
out and put back.

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
- for `DIR=BOTH` on a lane whose extruder other lanes feed too, on any unit, when it is not that
  extruder's active lane: `ace2k: lane N shares EXTRUDER and is not its active lane: choose it
  with ACEK_LANE_ACTIVE LANE=N` (`LANE=N UNIT=u` with several units). Nothing is sent. Only one
  lane follows an extruder; [`ACEK_LANE_ACTIVE`](#acek_lane_active) chooses it.

```
ACEK_ASSIST LANE=2 DIR=BOTH SOFT=1
```

### `ACEK_LANE_ACTIVE`

Makes a lane the active lane of its extruder. When several lanes name one extruder
(`laneN_extruder`, or the lane map a host module sets), only the active lane is followed by default
and fed by the feed-forward. The command stops the follow on the extruder's other lanes, as
`ACEK_STOP` would, and arms the follow on this lane. A lane with its own extruder, or with none, is
simply armed. The choice lasts until Klipper restarts.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `UNIT` | the only unit | a unit name | the unit that owns the lane; required with several units |

The group of lanes that share an extruder spans all the units. Lane n of unit b becomes the active
lane, and the follow stops on the other lanes of that extruder, on whichever unit they are. With
several units the lanes in the answer read `b:3`. The command arms nothing until the other
followers have ended.

No other way arms a lane of a shared extruder that is not its active lane. `ACEK_ASSIST DIR=BOTH`
on it is refused ([`ACEK_ASSIST`](#acek_assist)), and so is a host module's follow
([protocol.md](protocol.md), `start_move`). The follow's default, a re-arm after a move and an
insertion wait for the choice.

Answers `ace2k: lane N active on EXTRUDER`, followed by `; lanes A, B off` when it stopped others
(`no extruder` for a lane without one). Refused:

- `ace2k: lane N has no filament`;
- `ace2k: lane N is in error (KIND); ACEK_CLEAR first`;
- `ace2k: lane N not made active: lanes A, B stopped but did not end within 5 s; nothing armed;
  EXTRUDER's active lane stays lane M, or none` when another follower does not end in time: the
  previous choice is kept;
- `ace2k: lane N is busy (MODE)` while another move runs on it;
- the unit's refusal of the follow, and `ace2k: the unit's firmware has no follow tail (or no
  follow); flash v0.11.0`.

An internal error in the stop of another follower (its unit's link down, say) arms nothing and
keeps the choice, then goes on to Klipper as it came, and Klipper shuts down on it.

```
ACEK_LANE_ACTIVE LANE=2
```

### `ACEK_FOLLOW_SET`

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
ACEK_FOLLOW_SET FLIP_MS=300
```

### `ACEK_FEED_FORWARD`

Switches the feed-forward on or off, for one lane or every lane. The feed-forward takes the
extrusion Klipper has planned and feeds it to the lane in the follow, in small doses, ahead of the
pull. With no parameter, the command prints each lane's switch, its mapped extruder and its
mode (`following`, `idle`, …; `unknown` before the unit's first report).

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

A lane gets doses only while it is in the follow. With `ON=1`, a lane switched on is said, with
the command that arms it, when it could follow and does not. That is a lane with filament in its
bay and an extruder mapped, idle or in error, while no other lane of that extruder follows, on
any unit. A lane in error needs `ACEK_CLEAR` first:

```
ace2k: lane 1 is not in the follow: its feed-forward doses start once it follows — ACEK_LANE_ACTIVE LANE=1 arms it
```

```
ACEK_FEED_FORWARD LANE=1 OFF=1
```

### `ACEK_FF_SET`

Sets the feed-forward's values: two that the unit uses (dose size and speed) and three that the
host uses. With no parameter, it prints the values in effect. Every given value is checked before
any is applied.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `CHUNK_MM` | 3 (`ff_chunk_mm`) | 1–10 mm | a dose's length (the unit's) |
| `PULSE` | 15 (`ff_pulse`) | 9–70 mm/s | retired, the next version removes it: a reverse dose's least speed (the unit's); a forward dose runs at the unit's floor speed, as every move of 10 mm or less does |
| `WINDOW_MS` | 500 (`ff_window_ms`) | 100–5000 ms, integer | the planner window the host meters over; one shorter than `ff_period_ms` counts as one period |
| `LEAD_MS` | 0 (`ff_lead_ms`) | 0–5000 ms, integer | how far ahead of the extruder the doses run; a new lead drops each extruder's sweep, credit and open retraction; at any lead each planned move counts once, credited or taken back |
| `DEADBAND` | 0.3 (`ff_deadband`) | 0.0–10.0 mm/s | the rate below which no dose is sent |
| `RETRACT_MIN_MM` | 10 (`ff_retract_min_mm`) | 5–17 mm | a contiguous retraction this long or more is taken back by the lane; a shorter one is credited |

**`ff_pulse` and `PULSE` are retired.** Every forward dose runs at the unit's floor speed, so they
only set a reverse dose's least speed. They still work in this version, and the next version
removes them; the unit's own value (15 mm/s) is used then. With `ff_pulse` in `printer.cfg`, Klipper
lists it among its warnings at the start (Fluidd and Mainsail show it), and `klippy.log` says why.
`ACEK_FF_SET` with `PULSE` says it too, after its line:

- `ace2k: ff_pulse is retired: every forward dose runs at the floor speed, so it sets only a
  reverse dose's least speed. The next version removes it, with ACEK_FF_SET's PULSE, and the unit's
  own value is used. Remove it from printer.cfg.`

Refused:

- `ace2k: NAME (value) must be within LOW..HIGH` (or `must be a finite number`);
- `ace2k: the unit's firmware has no feed-forward; flash v0.9.0`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

The lane's status carries `ff_rate` (mm/s, the last one sent), signed: negative while the lane
takes a long retraction back. It also carries `ff_back_doses`, beside `ff_doses`: the reverse doses
since the follow was armed; `None` on an image without the reverse.

On an image without the reverse no negative rate is ever sent: a long retraction is credited as
any other. The host says so once at ready, in a console line that is not a refusal:

- `ace2k: the unit's firmware does not take back a long retraction; flash v0.12.0`.

```
ACEK_FF_SET CHUNK_MM=2 LEAD_MS=200
```

### `ACEK_SNAG_SET`

Sets the tip snag's six values. A filament tip can catch at the entrance of the plunger's tube.
These values set the limited tolerance that lets a feed, a rollback, an unload or a read with
motion (`ACEK_RFID_READ MOVE=1`) go on past it. A load from the bay and a feed with `FREE=1` judge
a full plunger by the filament in windows of `FWD` instead: from the window's start, the filament
falling more than `LAG` behind the motor at any tick ends the push. A filament that never falls
that far behind has covered at least `(FWD − LAG) / FWD` of the motor's travel when the window
closes (75 % with the defaults), and the push goes on. With no parameter, the command prints the
values in effect.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `FWD` | 20 (`snag_fwd_mm`) | 5–50 mm | motor travel past a full before the touch |
| `LAG` | 5 (`snag_lag_mm`) | 1.5–20 mm, under `FWD` | the filament behind the motor that counts as a jam |
| `DUTY` | 0 (`snag_duty_pct`) | 0–100, integer | duty above the feed-forward that counts as a jam; 0 is off |
| `BACK` | 5 (`snag_back_mm`) | 2–20 mm | the touch |
| `REST` | 500 (`snag_rest_ms`) | 100–2000 ms, integer | the wait for the plunger at rest |
| `FREE` | 50 (`snag_free_mm`) | 20–200 mm | in reverse, the filament back before a taut trip is forgiven |

The defaults are the firmware's, read at connect. So are the ranges, on an image since v0.13.0;
an older image does not publish them, and the host uses its own copy of this table. The same
ranges check the `snag_*` keys of `printer.cfg` at connect. A `snag_lag_mm` from 1 up to 1.5 in
`printer.cfg` is raised to 1.5, with a warning in `klippy.log`. So is a `LAG` from 1 up to 1.5,
with the warning on the console too, and 1.5 is what is staged for `SAVE_CONFIG`. The next version
refuses such a value. A `LAG` under 1 is refused. The warning:

- `ace2k: snag_lag_mm 1.2 is under the least lag of 1.5 mm, so 1.5 mm is used. The next version
  refuses it.`

Refused:

- `ace2k: NAME (value) must be within LOW..HIGH`;
- `ace2k: LAG (x) must be under FWD (y) mm`;
- `ace2k: firmware built without CONFIG_ACE2K_FEED`.

```
ACEK_SNAG_SET FWD=25 LAG=6
```

### The held stop's keys (no command)

A host module can end a move when the buffer's plunger shows that the filament tip is held, for
example in the print head during an unload. The unit judges this itself. A host module can ask
for it only on a rollback or an unload. Two keys of `[ace2k]` tune it. No G-code sets them, and
`SAVE_CONFIG` never writes them.

| Key | Default | Range | Meaning |
|---|---|---|---|
| `held_mm` | 3 | 1.5–20 mm | a filament that moves this much starts the wait again: it is not held |
| `held_ms` | 550 | 50–2000 ms, integer | how long the plunger must stay pulled tight with the filament still |

The defaults and the ranges are the firmware's, read at connect. A value out of range stops
Klipper's start with a config error, such as `ace2k: held_mm (1.4) must be within 1.5..20`. Both
values go to the unit at every connect. With a firmware without this stop, ace2k ignores the keys,
and `klippy.log` gets one line:

- `ace2k: held_mm, held_ms ignored: this image cannot stop on the plunger; flash v0.13.0`

Keep friction from looking like a held tip. A filament dragged slowly through the tube can move
less than `held_mm` in `held_ms` and end the move `held` with nothing holding it. The defaults are
safe down to the lane's least speed, 9 mm/s. A shorter `held_ms` or a longer `held_mm` makes the
slowest safe speed higher: with `held_mm: 3`, keep `held_ms` at 520 or more, and with
`held_ms: 550`, keep `held_mm` at 4.4 or less. Past that, Klipper still starts, and `klippy.log`
gets one line that gives the speed and the keys to change, such as:

- `ace2k: with held_mm 4.5 and held_ms 550, friction can end a move held below 10.8 mm/s, more than
  95 % of the lane's least speed (9 mm/s); raise held_ms or lower held_mm`

### The clear of an unload (no command)

An unload ends when the bay's sensor no longer sees the filament. The end of the filament could
then stop right on the sensor, which could read it again. So the motor turns a little more before
the unload ends. One key of `[ace2k]` sets how much. No G-code sets it, and `SAVE_CONFIG` never
writes it.

| Key | Default | Range | Meaning |
|---|---|---|---|
| `unload_clear_mm` | 20 | 0–100 mm | the motor travel past the bay's sensor before the unload ends; 0 ends it at the sensor |

The travel is the motor's, and it can go past the unload's `LENGTH`. When the move ends on its
length inside it, one more move turns the rest. A stall inside it ends the unload `unloaded` at
once, with no second try. An end that has already left the drive gear is not moved by it. The
default and the range are the firmware's, read at connect. A value out of range stops Klipper's
start with a config error, such as `ace2k: unload_clear_mm (120) must be within 0..100`. The value
goes to the unit at every connect. With a firmware without the clear, the unload ends at the
sensor, ace2k ignores the key, and `klippy.log` gets one line:

- `ace2k: unload_clear_mm ignored: this image does not clear an unload past the bay's sensor;
  flash v0.13.0`

## Calibration

### `ACEK_CALIBRATE_ENCODER`

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
- `ace2k: run ACEK_CALIBRATE_ENCODER LANE=n LENGTH=<mm> first` (no feed kept for the lane);
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
- `ace2k: lane N is in error (...); ACEK_CLEAR first`;
- `ace2k: lane N is busy (MODE)` (a mode other than idle or following);
- `ace2k: lane N shares EXTRUDER and is not its active lane: choose it with ACEK_LANE_ACTIVE
  LANE=N` (`LANE=N UNIT=u` with several units): the run follows the lane, and only the active lane
  of a shared extruder follows;
- `ace2k: lane N did not enter the follow`.

After the run:

- `ace2k: lane N: aborted — REASON; nothing changed` (the buffer went full, the lane left the
  follow, Klipper shut down);
- or a refusal of the result with `nothing changed`: fewer than 10 marks, the encoder did not
  follow the extruder, scatter above 1 mm, or a scale outside the unit's window.

A change above 3 % is applied, with a warning to check the extruder's own calibration.

```
ACEK_CALIBRATE_ENCODER LANE=1 AUTO=1
ACEK_CALIBRATE_ENCODER LANE=1 LENGTH=200
ACEK_CALIBRATE_ENCODER LANE=1 MEASURED=198.5
```

## Tags

### `ACEK_RFID_READ`

Reads the RFID tag of a lane's spool. By default it only looks at the antenna: a tag is given to a
lane only while that lane moves, so without `MOVE` no new tag is given (since v0.13.0). Only the
lane's own tag, given earlier by its motion, can be read again. With
`MOVE=1` it turns the spool to find the tag: it feeds the filament out up to `rfid_search_mm`, and
back. That search keeps the tip snag's tolerance ([`ACEK_SNAG_SET`](#acek_snag_set)): the touch,
the wait for rest, once per read. It does not take a load from the bay's free tip, because the
filament may sit in an extruder, where a filament that follows the motor proves nothing. A lane
already read is read again. The command prints the outcome and the decoded spool
record. With `MOVE=1`, another lane of the unit that only follows, with its motor at rest, does
not refuse the read (since v0.13.0); if that follow moves during the search, the search gives way
and comes back, and the lane stays `pending`. A waited command waits 5 s for the field report (`MOVE=0`) or the move's budget (`MOVE=1`).

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |
| `MOVE` | 0 | 0–1 | 0 only reports what is in the field and gives no new tag (the lane's own tag, given earlier by its motion, can be read again); 1 searches by moving the filament, with the tip snag's tolerance (the filament may be in an extruder), and is the only way to attribute a tag on command |
| `DUMP` | 0 | 0–1 | 1 saves the tag's raw bytes as `ace2k_tag_laneN_<date-time>.json` in the log directory |
| `WAIT` | 1 | 0–1 | 0 returns at once (`tag read started`) |

Refused:

- `ace2k: lane N tag read refused: REASON`, with `REASON` `busy`, `no_filament`, `no_link`,
  `other_lane_moving` or `unsupported`;
- `ace2k: lane N: no tag outcome within S s`;
- for `MOVE=1`, `ace2k: lane N tag search failed: KIND; ...` when the load ends in an error kind;
- `ace2k: firmware built without CONFIG_ACE2K_RFID_READ`.

Without `MOVE`, a tag in the field that is not the other lane's answers `ambiguous`, and nothing
changes. The command then asks for `MOVE=1`. With no such tag, it prints the lane's state.

```
ACEK_RFID_READ LANE=1 MOVE=1
```

### `ACEK_RFID_FORGET`

Forgets the tag of a lane, in the unit and in the host, so the next move reads it again. The
other lane on the same reader may then take that tag.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `LANE` | required | 1–4 | the lane |

Refused: `ace2k: firmware built without CONFIG_ACE2K_RFID_READ`.

```
ACEK_RFID_FORGET LANE=1
```

### `ACEK_LOAD_SET`

Sets the automatic load's values: what the unit does when you insert a filament. The values are
staged for `SAVE_CONFIG`. The command sends the three values to the unit and prints them.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `PARK` | `load_park_mm` (300) | above 0, at most 2000 mm | the parking distance |
| `SPEED` | `load_speed` (60) | 9–70 mm/s | the load speed |
| `AUTO` | `auto_load` (on) | 0–1 | 1 pulls an inserted filament in by itself |

Refused: `ace2k: SPEED must be within MIN..MAX mm/s`;
`ace2k: firmware built without CONFIG_ACE2K_FEED`. A rejected value changes nothing. With no
parameter, the command prints the values in effect. It stages and sends nothing.

```
ACEK_LOAD_SET PARK=290 AUTO=1
```

### `ACEK_GRIP`

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
ACEK_GRIP LANE=3 MM=30
```

## Dryer, fans and flaps

The unit controls the heater and every protection. A cycle runs to its end even with no host.
Every refusal below comes from the unit, not from the host.

### `ACEK_DRY`

Starts a drying cycle at a chamber target, for a duration.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `TEMP` | required | 15–65, integer degrees C | the target |
| `DURATION` | required | 1 minute to 24 hours: a bare number is minutes, or `90m`, `4h`, `1.5h` | the cycle length |

Refused:

- an out-of-range `TEMP`: Klipper's own parameter check rejects it before the module runs
  (`Error on 'ACEK_DRY': TEMP must have minimum of 15` / `maximum of 65`);
- `TEMP` absent: the module's own `ace2k: ACEK_DRY needs TEMP=<15-65>`;
- `ace2k: ACEK_DRY needs DURATION=<minutes, or with an h / m suffix>, 1 min to 24 h`;
- `ace2k: dryer start refused: TEXT`, with TEXT as in the table below;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT` or `CONFIG_ACE2K_DRYER`;
- `ace2k: ...: no answer from the unit; check the link (...)`.

The table lists the refusal texts of `ACEK_DRY`, `ACEK_DRY_STOP` and `ACEK_DRY_CLEAR`. The console
prints them after `ace2k: dryer start refused: ` (or `stop` / `clear`). A reason code not in the
table prints `reason N`.

| Code | Text |
|---|---|
| 1 | busy — a cycle or its cool-down is running (ACEK_DRY_STOP first), or the fans are held by a manual run (ACEK_FAN ON=0 first) |
| 2 | out of range — 15–65 °C, 1 min to 24 h |
| 3 | the dryer is in a fault — ACEK_DRY_CLEAR once it is cool |
| 4 | a temperature sensor is not valid |
| 5 | the mains is absent or its frequency implausible |
| 6 | the thermal cutout tripped — power-cycle the unit |
| 7 | the heater is still cooling down |
| 8 | the unit went through a Klipper shutdown — FIRMWARE_RESTART before drying again |
| 9 | the mains has just come back — its frequency is being measured; try ACEK_DRY again in a few seconds |

```
ACEK_DRY TEMP=55 DURATION=4h
```

### `ACEK_DRY_STOP`

Stops the cycle. The unit cools the heater down with the fans before it releases them.

No parameters. Refused: `ace2k: dryer stop refused: TEXT` (table under `ACEK_DRY`); the
firmware-built-without errors.

```
ACEK_DRY_STOP
```

### `ACEK_DRY_CLEAR`

Clears a dryer fault once the heater is cool and the cause is gone. A tripped thermal cutout does
not clear this way: it needs a power cycle.

No parameters. Refused: `ace2k: dryer clear refused: TEXT` (table under `ACEK_DRY`); the
firmware-built-without errors.

```
ACEK_DRY_CLEAR
```

### `ACEK_DRYER_LOG`

Prints the unit's dryer log:

- cycles started and completed, and faults;
- hours of heating, and the energy estimate at `heater_watts` (default 360 W);
- a note if the thermal cutout tripped;
- the last eight faults or interrupted cycles, with their temperatures.

If the log changes while it is read, the read is repeated, up to three times.

No parameters. Refused: `ace2k: firmware built without CONFIG_ACE2K_HEAT` or `CONFIG_ACE2K_DRYER`;
`ace2k: ...: no answer from the unit; check the link (...)`.

```
ACEK_DRYER_LOG
```

### `ACEK_FAN`

Runs both fans by hand. There is no control for a single fan. `ON=0` releases the manual request.
It does not switch the fans off: the unit keeps them on while an outlet NTC reads above 45 C, and
while the heater holds them.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `ON` | required | 0–1 | 1 runs the fans, 0 releases the request |
| `SECONDS` | 60 | 1–600, integer | how long the fans run (with `ON=1` only) |

Refused:

- `ace2k: ACEK_FAN needs ON=0 or ON=1`;
- `ace2k: ACEK_FAN SECONDS applies only with ON=1`: `ON=0` with `SECONDS` leaves the fans as they
  are;
- `ace2k: fans refused: busy — the dryer holds the fans and flaps, or another flap pulse is running
  (try again)` or `out of range`;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT`.

```
ACEK_FAN ON=1 SECONDS=120
```

### `ACEK_FLAP`

Pulses one exhaust flap open or closed. The flap cannot report its position, so the answer says
`(no read-back: look at it)`.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `WHICH` | required | `bottom` or `rear` | the flap |
| `OPEN` | required | 0–1 | 1 opens, 0 closes |

The unit sets the pulse length. Refused:

- `ace2k: ACEK_FLAP needs WHICH=bottom or WHICH=rear`;
- `ace2k: ACEK_FLAP needs OPEN=0 or OPEN=1`;
- `ace2k: ACEK_FLAP takes no MS=; the unit pulses for its compiled length`;
- `ace2k: WHICH flap refused: busy — ...` (the dryer holds the flaps, or another pulse is running)
  or `out of range`;
- `ace2k: firmware built without CONFIG_ACE2K_HEAT`.

```
ACEK_FLAP WHICH=rear OPEN=1
```

### `ACEK_DRY_MODE`

Chooses the dryer's mode. The unit keeps it across a power loss.

> [!CAUTION]
> **The humidity mode turns a mains heater on by itself, with nobody there.** The unit starts a
> drying cycle whenever its box stays humid, even with the printer off. The mode runs only with
> `unattended_drying: True`. Set it only with the unit in a ventilated place, with nothing that
> burns next to it, and nothing on the lid or over the vents. Read the dryer warnings in
> [README](../README.md#read-this-first) first.

| Parameter | Default | Range | Meaning |
|---|---|---|---|
| `MODE` | required | `manual`, `humidity`, `print` | the mode |

- `manual`: the dryer runs only when asked (`ACEK_DRY`).
- `humidity`: the unit dries on its own when the box's humidity stays at or above the limit,
  with or without the printer. It needs `unattended_drying: True` in the unit's section. Choosing it again
  also turns it back on after it stopped on its failed cycles.
- `print`: the host dries during a print: it starts a cycle at the print's start when the box is
  cold and at or above the limit, or after the hold during the print, and stops it at the print's end.
  It needs `[virtual_sdcard]` (the host reads `print_stats`), and says so once in the log if that
  is missing (only while the mode is `print`).

The unit's section takes these options:

| Option | Default | Range | Meaning |
|---|---|---|---|
| `unattended_drying` | `False` | `True` / `False` | allow the humidity mode: the unit then heats on its own, with the printer off |
| `dry_humidity_limit` | 40 | 5–95 % | the box's humidity at or above which the automatic modes dry |
| `dry_max_failures` | 4 | 1–20 | failed cycles in a row that stop the humidity mode |

For the bench only, and unset in normal use: `bench_dry_cycle_minutes` (the automatic cycle's time
instead of the material's, 1–1440), `bench_dry_hold_s`, `bench_dry_cold_window_s` and
`bench_dry_link_s` (the hold at or above the limit, the cold box's window and a bound cycle's link time,
in seconds, at most 4294967; 0 keeps the unit's default).

The host sends the limits and the bench timings to the unit after each connect, on its first
report. A removed bench option is undone by a Klipper restart. Only a changed timing restarts what
it times (a new window, the cold box's window; a new hold, the hold): a restart with the same
options leaves the box cold and the hold running. A maximum lowered to the failures already
counted stops the humidity mode.

With `unattended_drying: False`, a unit that still holds the humidity mode is set back to manual
on its first report after a connect, and the console says `the humidity mode is off in
printer.cfg: unit set back to manual`. If the unit does not take it, the console says so once, and
the host tries again every 5 s.

The console also gets one line for each event of the automatic modes, for example
`ace2k dryer: automatic cycle started at 45 °C: the box stayed at or above the limit`,
`ace2k dryer: automatic cycle done but the box is still humid (2 failed in a row)`,
`ace2k dryer: the humidity mode stopped after 4 failed cycles in a row: check the lid or the
limit`, `ace2k dryer: the humidity mode is off: an automatic cycle was stopped by hand (choose it
again to turn it back on)` or `ace2k dryer: the print mode's cycle ended: the host was gone for
the link time` (5 min). With several units the head names the unit (`ace2k b: dryer: …`).

`ACEK_STATUS` shows one line for the mode, for example `dryer mode: humidity — waiting: box 35 %,
limit 40 %`, `dryer mode: humidity — stopped after 4 failed cycles: check the lid` or
`dryer mode: print`.

Refused:

- `ace2k: the dryer mode is manual, humidity or print`;
- `ace2k: the humidity mode is off in printer.cfg (unattended_drying: True to allow it)`;
- `ace2k: dryer mode MODE refused: range`;
- `ace2k: firmware without the dryer's modes — flash a newer image`, or
  `ace2k: firmware built without CONFIG_ACE2K_HEAT` / `CONFIG_ACE2K_DRYER`;
- `ace2k: ...: no answer from the unit; check the link (...)`.

```
ACEK_DRY_MODE MODE=print
```

## Firmware

### `ACEK_RESTORE_STOCK`

Resets the unit into its bootloader's recovery mode, so the factory firmware can be written back
over the cable. After it answers, stop Klipper and run `ace2k_flash.pyz` (below), or follow the
manual procedure in [flashing.md](flashing.md).

No parameters. The unit refuses while the dryer heats or a lane moves: the command fails with
`ace2k: refused — a subsystem is busy (heating or moving)`, and nothing changes.

```
ACEK_RESTORE_STOCK
```

## `ace2k_flash.pyz`

`ace2k_flash.pyz` flashes the unit from the Klipper host:

- factory firmware to ace2k;
- ace2k to a newer ace2k;
- ace2k back to the factory firmware;
- out of the bootloader's recovery.

It is one file. The release's host bundle, `ace2k-host-X.Y.Z.tar.gz`, carries it in
`firmware/tools/`. `make -C firmware flasher-pyz` builds it from the source, into
`firmware/build/`. It runs on the host (Python 3.9+, pyserial). It identifies the unit before it
changes anything, shows a plan, and asks before it writes. The three paths, the order of its
steps, its refusals and the manual equivalent are in [flashing.md](flashing.md).

    python3 ace2k_flash.pyz IMAGE.bin [--port PORT] [--version V] [--moonraker URL]
                            [--klipper-stop CMD] [--klipper-start CMD]
                            [--calibration-dir DIR] [--yes] [--config-follows]

| Argument | Default | Meaning |
|---|---|---|
| `image` | required | the image: an ace2k `.bin`, or the factory `.bin` |
| `--port` | from Klipper's config (one unit section) | the unit's serial port; required with several `[ace2k <name>]` sections. A port that is not the adapter in the unit's cable (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_*`, or a path that leads to one) gets one warning line before anything is sent, and the run goes on. Never give the port of a printer head |
| `--version` | read from the image | the image's version, when the image does not carry it (always for the factory image) |
| `--moonraker` | `http://localhost:7125` | Moonraker's URL, used to read the config and Klipper's state; it starts with `http://` or `https://` |
| `--klipper-stop` | detected | the command that stops Klipper on this host (give both stop and start, or neither) |
| `--klipper-start` | detected | the command that starts Klipper on this host |
| `--calibration-dir` | `.` | where the factory-firmware calibration file is written |
| `--yes` | off | do not ask for confirmation |
| `--config-follows` | off | the caller writes `[ace2k]` and starts Klipper after this run (an installer) |

Exit status:

- 0: flashed and verified;
- 1: failed, or refused after something was done (Klipper stopped, the restore command sent, the
  update started);
- 2: flashed but not verifiable on this host. When this run stopped Klipper and could not start
  it again, the status stays 2 and the last line says `Klipper did NOT start`;
- 3: refused, and nothing was changed (at most read-only queries reached the unit, and Klipper
  was neither stopped nor started);
- 4: nothing answered on the port, and nothing that changes the unit was sent (Klipper may have
  been stopped and started again to free the port; if that start fails: 1). The unit is off, or
  runs ace2k with no Klipper connected to it.

Ctrl-C: 3 before any change, 1 after.

[`flashing.md`](flashing.md) has the full table.
