# Features

This page explains how ace2k behaves:

- how to set up `printer.cfg`;
- how a lane keeps up with a printing head;
- what happens when a spool runs out;
- how to calibrate the unit's encoder;
- how spool tags are read;
- how the unit dries filament;
- what the lane LEDs mean.

Every command named here is documented, with its parameters and refusals, in
[`commands.md`](commands.md).

## Configuration

`config/ace2k.cfg` is the part you add to your `printer.cfg`. It contains:

- the unit as a Klipper MCU (`[mcu ace2k]`);
- the `[ace2k]` section. It is part of the link: with a bare `[mcu ace2k]`, a freshly flashed unit
  falls back into recovery ([`protocol.md`](protocol.md), "The link proof");
- the include of `config/ace2k_tags.cfg`, which lists the spool brands;
- the three temperatures, as sensors.

For the Snapmaker U1, see the [`ace2k-u1`](https://github.com/tobecwb/ace2k-u1) project.

    [mcu ace2k]
    serial: /dev/serial/by-id/<the unit's USB adapter>
    baud: 250000
    restart_method: command

    [ace2k]
    #lane1_insert_mv: 600, 1100    # optional; the factory calibration is used when absent
    #feed_speed: 30                # mm/s, the default of ACE_FEED and ACE_ROLLBACK
    #unload_speed: 30
    #load_speed: 30
    #load_park_mm: 300             # the load's parking distance: the path from the drive to the outlet
    #                              # (323 mm on the development unit) minus a margin
    #auto_load: True               # pull a filament in to the parking point when it is inserted
    #assist_speed: 50
    #slip_check_mm: 50             # the supervisors' thresholds; an absent key leaves the firmware's
    #slip_allow_mm: 15
    #stall_check_mm: 20            # slip_check_mm and stall_check_mm under 100 mm, an assist's burst
    #snag_fwd_mm: 20               # the tip snag: motor past a full before the touch (5–50)
    #snag_lag_mm: 5                # the filament behind the motor that is a jam (1–20, under fwd)
    #snag_duty_pct: 0              # the duty above the feed-forward that is a jam (0 off, 0–100)
    #snag_back_mm: 5               # the touch (2–20)
    #snag_rest_ms: 500             # the wait for the plunger at rest (100–2000)
    #snag_free_mm: 50              # reverse: the filament back before a taut trip is forgiven (20–200)
    #follow_flip_ms: 200           # the follow: the wait before a move against the last one (50–2000)
    #follow_take_mm: 15            # the follow's take-up on this lane's full (5–30)
    #follow_tail_mm: 2000          # the follow's tail: motor after a runout before it ends (200–2000)
    #lane1_extruder: extruder      # the feed-forward: the extruder lane 1 feeds (lane1..lane4; several
    #                              # lanes may name one); a lane without one has no feed-forward
    #feed_forward: True            # ACE_FEED_FORWARD ON=1 / OFF=1 switches it live
    #ff_chunk_mm: 3                # the unit's dose (1–10)
    #ff_pulse: 15                  # mm/s, a dose's speed (the lane's speed bounds, 9–70)
    #ff_period_ms: 100             # how often the host works out the rate (20–1000)
    #ff_window_ms: 500             # the stretch of planned moves the rate is the mean of (100–5000)
    #ff_lead_ms: 0                 # how far ahead of now that stretch starts (0–5000)
    #ff_deadband: 0.3              # mm/s: a smaller change of the rate is not sent (0–10)
    #ff_notice_every: 20           # corrections one way before a console notice (1–10000)
    #lane1_encoder_scale: 1.2342   # mm per encoder count, lane1..lane4 (ACE_CALIBRATE_ENCODER)
    #feed_report_hz: 1             # 0.05–10 Hz; 1, 2, 5 and 10 are exact, the rest snap to the 100 ms grid
    #rfid_search_mm: 750          # the load's search for a spool's tag past the parking point,
    #                             # 100–1000 mm of filament (unset: the firmware's default)
    #rfid_mifare_order: bambu, snapmaker, creality
    #heater_watts: 360            # the dryer's heater power, for ACE_DRYER_LOG's energy

    [include ace2k_tags.cfg]

## Commands in brief

`LANE` is 1–4. Lengths are in mm. Speeds are in mm/s, within the unit's bounds (9–70).

The motion commands ([`ACE_FEED`](commands.md#ace_feed), [`ACE_ROLLBACK`](commands.md#ace_rollback),
[`ACE_LOAD`](commands.md#ace_load), [`ACE_UNLOAD`](commands.md#ace_unload)) wait for the move's
final event by default. They then print how far the motor and the filament travelled. If the move
ends in an error kind, the command fails with a G-code error. With `WAIT=0` the command returns at
once, and the event goes to the log.

Sometimes a feed or a load meets something that cannot move: the filament is stopped inside the
unit, or held in the tube further on. The move then stops without an error. The console shows
`ace2k: lane n feed blocked — motor … mm, filament … mm` (`load blocked` for a load). The lane
is idle where it stopped, and there is nothing to clear. A loose piece left in the tube does not
block a move: the push moves it ahead.

- To read the status, use [`ACE_STATUS`](commands.md#ace_status) and
  [`ACE_HEALTH`](commands.md#ace_health).
- To get a lane out of an error, use [`ACE_CLEAR`](commands.md#ace_clear).
- The whole list is in [`commands.md`](commands.md).

## How the unit keeps up with the print head

Between the unit and the head sits the buffer: a spring-loaded plunger the filament runs through.
When the head pulls the filament, the plunger moves toward one end. When the head pushes the
filament back, the plunger moves toward the other end.

ace2k has three ways to respond:

- **Assist.** This is the general mechanism. The unit watches the plunger and runs the lane's
  motor to bring it back to rest. You choose the direction with
  [`ACE_ASSIST`](commands.md#ace_assist) `DIR=`:
  - **forward** (the default): the unit only *feeds*. When the head pushes filament back, the unit
    does not take it up. The buffer absorbs it.
  - **back** (`DIR=BACK`): the unit only *takes up*.
  - **both** (`DIR=BOTH`): the unit feeds when the head pulls, and takes up when the head pushes
    back. **This is the follow.**
- **Follow.** The follow is not a separate feature: it is the assist in both directions. It exists
  because a printer sometimes pulls and pushes within a few seconds. It does so when it forms the
  tip during an unload, and when it retracts during a print. A one-way assist would let filament
  pile up in the buffer. [`ACE_FOLLOW_SET`](commands.md#ace_follow_set) only changes its settings.
  On the Snapmaker U1, the adapter uses the follow, never the one-way assists.
- **Feed-forward.** It works on top of the follow, and it runs on the host. The assists *react*:
  the plunger has to move before the unit does anything. The feed-forward *anticipates*: the host
  reads the moves Klipper has already planned for the extruder. The unit then feeds that filament
  in small doses, just before the head needs it
  ([`ACE_FEED_FORWARD`](commands.md#ace_feed_forward), [`ACE_FF_SET`](commands.md#ace_ff_set)). It
  works only while the lane is in the follow.

| | Watches | Moves the filament | Set with |
|---|---|---|---|
| Assist, forward | the buffer | toward the head only | `ACE_ASSIST` |
| Assist, back | the buffer | back only | `ACE_ASSIST DIR=BACK` |
| **Follow** | the buffer | both ways | `ACE_ASSIST DIR=BOTH`, tuned by `ACE_FOLLOW_SET` |
| Feed-forward | Klipper's planned extruder moves | toward the head, ahead of the pull | `ACE_FEED_FORWARD`, `ACE_FF_SET` — only during the follow |

Example during a print: the extruder is about to pull 5 mm.

- With the follow alone, the head first pulls the plunger, and then the unit feeds. The correction
  comes after the fact.
- With the feed-forward on, the host sees the 5 mm in Klipper's plan. The unit feeds them as the
  head starts to pull. The follow only corrects what is left.

Sometimes the tip of the filament catches at the entrance of the plunger's tube.
[`ACE_SNAG_SET`](commands.md#ace_snag_set) sets how the unit handles this. Instead of ending the
feed, the load or the unload as `stuck`, the unit makes one short move back (the "touch"), within
set limits, and announces it.

[`ACE_STOP`](commands.md#ace_stop) ends any assist.

## When a spool runs out during a print

This section is about a lane in the follow, which is how a lane feeds a print.

The sensor that detects filament sits at the entrance of each bay. The motor that moves the
filament sits further inside the unit. So when a spool runs out, or the filament is cut at the bay,
the end of the filament is still inside the unit, in the motor.

The follow does not stop at that point. The lane goes on feeding that last piece of filament: this
is its **tail**. The console shows the `tail` notice. During the tail, the lane:

- still feeds when the head pulls the filament tight, and still sends its feed-forward doses, with
  no filament at the bay's sensor;
- never takes filament back, so the head prints with the filament left in the tube.

The tail ends with `tail_out`: the lane goes idle, with no error. This happens when:

- the filament no longer follows the motor, because its end has left the motor; or
- the motor has run `follow_tail_mm` (2 000 mm) since the spool ran out.

[`ACE_STOP`](commands.md#ace_stop) ends the tail like any follow.

**Insert the new filament after the `tail_out`.** The tail ignores the bay. Until the tail is out,
the end of the old filament holds the motor's gear. A filament put in meanwhile rests behind it,
and the gear cannot grip it. No load starts on it at the `tail_out` either. In that case, pull it
out and push it in again: it then loads like any filament.

Then the printer's own runout sensor at the head, if it has one, sees the end of the filament.
Nothing turns the tail off. The lane's status carries `tail` (True while in it).
[`protocol.md`](protocol.md), "The follow", has the full details.

## Inserting a new spool while the head still holds the old filament

After a `tail_out` during a print, the printer's head still holds the old piece of filament. It
pulls that piece at print speed. If you insert a new filament now, a full automatic load would
catch up with that piece and push it. A full load means the pull to the parking point and the tag
search. The load can then end `blocked`, and the printer's own tangle detection can pause the
print.

A lane's **grip** prevents this. It turns the lane's next automatic load into a short pull, only
until the motor's gear holds the filament (40 mm by default), with no tag search. The load ends
`loaded`, and the new filament waits there. The old piece is never touched. The printer's own
runout brings the new filament on once the old piece has run out.

The grip is set:

- by a host module that knows what the head holds (`set_lane_grip`; the `ace2k-u1` adapter does
  this);
- or by hand, with [`ACE_GRIP`](commands.md#ace_grip).

The grip is cleared:

- by the load, which uses it up;
- by any accepted start on the lane, or a tag read with motion;
- at every connect.

The tag of a gripped spool is not searched for. Read it with the tag in front of the antenna
([`ACE_RFID_READ`](commands.md#ace_rfid_read) `LANE=n`), or let it be read on the lane's next move.
[`protocol.md`](protocol.md#the-grip), "The grip", has the full details.

## Feeding ahead of the extruder (feed-forward)

A lane in the follow feeds only once the head has pulled its filament tight. Between those bursts,
the filament stays stretched.

With the feed-forward, the host reads the moves Klipper has planned for the extruder the lane
feeds, a little ahead of the head. The unit then feeds that filament out in small doses, ahead of
the pull: `ff_chunk_mm` (3 mm) at `ff_pulse` (15 mm/s). The plunger stays near rest.
[`ACE_FEED_FORWARD`](commands.md#ace_feed_forward) switches it on and off.
[`ACE_FF_SET`](commands.md#ace_ff_set) tunes it.

The follow still corrects on top of it:

- when the filament is pulled tight, the follow starts its burst;
- when this lane's buffer is full, the follow takes filament up.

Either correction drops the filament the lane still owed. The feed-forward works only on a lane in
the follow ([`ACE_ASSIST`](commands.md#ace_assist) `… DIR=BOTH`, or an adapter such as
`ace2k-u1`). Stopping the follow ends it. [`protocol.md`](protocol.md), "The feed-forward", has the
full details.

- **Which extruder each lane feeds.** `laneN_extruder` names the extruder lane N feeds
  (`extruder`, `extruder1`, …). Several lanes may name the same extruder, for example on a
  single-extruder printer with more than one spool. Only the lane in the follow is fed. If two of
  them are in the follow at once, the host cannot tell which to feed: neither gets a rate, a
  console line says so, and each keeps the plain follow. A lane without a key has no feed-forward,
  unless a host module maps it (the host interface's `set_lane_extruder`; the U1 adapter maps every
  lane it hooks to its head's extruder). A key wins over that.
- **The rate.** Every `ff_period_ms`, the host takes the mean extrusion speed of the planned moves
  in a window of `ff_window_ms`, starting `ff_lead_ms` from now. A retraction is not lost: it is
  credited against the next extrusion, so that much of the next extrusion is held back. The host
  sends the rate when it changed by more than `ff_deadband`, or fell to 0.
- **The status.** `printer.ace2k.lanes[n]` carries `ff_on`, `ff_extruder`, `ff_rate` (mm/s, the
  last one sent), and the unit's counters since the lane's mode began: `ff_doses`, `ff_taut_fixes`
  and `ff_full_fixes`.
- **The notices.** The host counts the corrections in each direction since the last notice.
  Corrections in opposite directions cancel. They count only while the lane is metering (its
  feed-forward on, and a rate above 0 sent). Every `ff_notice_every` corrections in one direction,
  the host prints a console line:

      ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale (ACE_CALIBRATE_ENCODER)

or the same with `full corrections`. A dose is a length measured on the lane's encoder. So **a
steady stream of notices in one direction means the encoder scale is wrong. Run
[`ACE_CALIBRATE_ENCODER`](commands.md#ace_calibrate_encoder)** ("Calibrating the encoder", below).
The lane keeps working meanwhile. Nothing corrects the scale on its own.
- **An older image** has no feed-forward. The console shows `ace2k: the unit's firmware has no
  feed-forward; flash v0.9.0` once, and `ACE_FEED_FORWARD` and `ACE_FF_SET` are refused. An image
  older than v0.11.0 also has no follow for this host ("When a spool runs out during a print",
  above), so there is nothing to feed ahead for.

## Calibrating the encoder

A lane measures its lengths on its filament encoder: a feed, a rollback, and every feed-forward
dose. So the encoder's scale (mm per count, `laneN_encoder_scale`, 1.2342 by default) sets how much
filament the lane delivers. A wrong scale shows as feed-forward corrections that all go one way.

[`ACE_CALIBRATE_ENCODER`](commands.md#ace_calibrate_encoder) `LANE=n AUTO=1` measures the scale
against the extruder the lane feeds. It takes about two minutes at the defaults (500 mm at
4 mm/s). The extruder is the reference on purpose. The feed-forward needs the lane to agree with
what the extruder pulls. So even an extruder that is off its own calibration gives the right lane
scale.

- **What it needs.**
  - The filament loaded to the head, held by the extruder's gears.
  - The lane mapped to an extruder (`laneN_extruder`, or `set_lane_extruder` from an adapter such
    as `ace2k-u1`).
  - That extruder active, and hot enough to extrude the filament. The command never heats and
    never changes tool.
  - The head where it may extrude, over a purge spot. The command never moves X, Y or Z.
  - The lane idle or in the follow.
  - No print printing or paused.
  - An image with the feed-forward (its `taut` count makes the marks) and the follow's tail:
    v0.11.0 or later. On an older one the arm is refused: `the unit's firmware has no follow tail
    (or no follow); flash v0.11.0`.
- **What it does.**
  1. On an idle lane, it arms the follow (as [`ACE_ASSIST`](commands.md#ace_assist)
     `LANE=n DIR=BOTH`, at `assist_speed`). A lane already in the follow stays in it.
  2. It holds the lane's feed-forward for the run: no rate is sent, what the lane owed is dropped,
     and the [`ACE_FEED_FORWARD`](commands.md#ace_feed_forward) switch is not touched. So only the
     follow feeds. The unit reports the lane counters and the feed-forward's counters at 10 Hz.
  3. It extrudes `LENGTH` in 25 mm chunks at `SPEED`. Each chunk is queued once the last one has
     run. This runs under `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE`, so the G-code modes and the
     E position the next G-code sees stay as they were.
  4. Each time the extruder pulls the plunger from rest back to tight, the follow starts its burst
     and the unit counts that `taut`. Each one gives a mark: the extruder's position at that
     moment, against the encoder, which has not moved since the burst before. The run's first
     `taut` is dropped, because where the plunger started is unknown.
  5. However the run ends, it then puts everything back. The hold is released, the counters return
     to their own rates, and the follow is stopped only if the run armed it.
  6. Only then are the marks judged. A least-squares line of the encoder against the extruder
     gives the new scale: the scale in use divided by the line's slope.
- **What it prints**, for example:

      ace2k: lane 1: calibrating against extruder — 500 mm at 4 mm/s
      ace2k: lane 1: 31 marks over 487.3 mm of extruder; the encoder read 482.6 mm
      ace2k: lane 1: scale 1.2342 → 1.2462 mm/count (+0.97 %), scatter 0.12 mm — in use now; SAVE_CONFIG to keep it

  After a change above 3 %, it also prints
  `ace2k: lane 1: a change above 3 % — check the extruder's own calibration too`. The scatter is
  the marks' RMS distance from the line.

  The new scale is in use at once: [`ACE_STATUS`](commands.md#ace_status) shows
  `scale=1.2462 (calibrated)`. It is also staged as `laneN_encoder_scale`: **`SAVE_CONFIG` keeps
  it**. At every connect, every lane's scale is sent to the unit again, the saved one or the
  default. So a calibration you did not save is undone by a Klipper restart, like any unsaved
  setting. The lane's `encoder_mm` (`printer.ace2k.lanes[n]`) goes on across the change without a
  jump ([`protocol.md`](protocol.md), "Host interface").
- **What it refuses.** Each refusal gives its reason, before anything is sent:
  - a print printing or paused;
  - a lane without an extruder, or whose extruder is not the active one (`select it first`) or is
    too cold to extrude;
  - a lane without filament, in error ([`ACE_CLEAR`](commands.md#ace_clear) first) or busy in
    another mode;
  - an image without the feed-forward;
  - a calibration already running;
  - a `LENGTH` or a `SPEED` out of its bounds.

  If the unit refuses to arm the follow, the command ends with the unit's reason. If the follow is
  not seen within 3 s, the command ends with `did not enter the follow`, and the arm is undone.
  With a slow `feed_report_hz`, that wait is two state-report periods and 0.5 s, when that is
  longer. Nothing is extruded either way.
- **What aborts it.** While it extrudes, it checks every quarter second for:
  - the buffer gone full (the shared full switch, or the lane's own full count `ff_full_fixes`
    rising);
  - the lane in error, out of the follow, or entering a mode again;
  - Klipper shut down.

  The running chunk ends, and no other is queued. The console shows
  `ace2k: lane 1: aborted — the buffer went full; nothing changed`, or the reason found. After a
  shutdown nothing is sent and no G-code runs. A G-code error of Klipper's own (the nozzle cooled
  below its minimum, an extrusion limit) ends the run with Klipper's message. Everything is
  restored and nothing changed.
- **What it will not apply.** The result is refused with `nothing changed` when:
  - there are fewer than 10 marks. The line says how many were dropped. Check that the extruder
    grips the filament. A larger `LENGTH` gives more marks;
  - the scatter is above 1 mm (the filament slipped);
  - the scale is outside the unit's window, 0.9874..1.4810 (±20 % of 1.2342);
  - the encoder did not follow the extruder.

  Each check uses the value as printed: the scale to 4 decimals, the scatter to 0.01 mm, the change
  to 0.01 %. So a line never contradicts its verdict.
- **The manual mode** remains, for a lane without an extruder:
  1. `ACE_CALIBRATE_ENCODER LANE=n LENGTH=200` feeds 200 mm with the end of the filament free
     (unloaded from the head, the tube off it).
  2. Measure what came out.
  3. `ACE_CALIBRATE_ENCODER LANE=n MEASURED=<mm>` stages the scale for `SAVE_CONFIG`. The unit takes
     it at the restart.

## Spool tags

With `[include ace2k_tags.cfg]` in `printer.cfg` (copy `config/ace2k_tags.cfg` next to it), the
unit reads the RFID tag of the spool in each lane, and the host names the spool.

- **When you insert a filament**, the load pulls it in while the lane's reader watches. When the
  tag passes the antenna, the lane stops, the tag is read, and the load goes on. If the tag has not
  passed by the parking point, the unit looks further: up to `rfid_search_mm` of filament. That is
  one turn of the largest spool that fits, plus the tag's window (750 mm, measured). It does this
  only while every other lane is idle. The filament always comes back to the parking point. A
  waited [`ACE_LOAD`](commands.md#ace_load) allows for that travel.
- **A lane whose tag was not read** (the search gave way to another lane, or the host was not
  there) is watched on every later move, such as a feed or an assist. Its tag is read on the fly.
  [`ACE_RFID_READ`](commands.md#ace_rfid_read) asks for a read at any time.
- **Two lanes share each antenna** (1–2, 3–4). A tag is given to a lane only when that lane's
  motion explains it. A read without motion answers *ambiguous* when the other lane holds a spool
  whose tag was not read. Use `MOVE=1` then.
- **The record** (brand, material, colour, weight, temperatures, what the tag carries) is in
  [`ACE_STATUS`](commands.md#ace_status), in `printer.ace2k.lanes[n].tag` (`state`, `uid`,
  `source`, `record`), and in the Klipper event `ace2k:tag_read`. Each is also a log line. A tag
  that no brand recognises still counts, by its UID. Every record is cached by UID in
  `ace2k_tags_cache.json` beside `printer.cfg`. When a tag is read again with the UID alone (the
  read could not keep up with the move, or was given up), it takes the cached record, marked
  `cached`. Deleting the file empties the cache.
  [`ACE_RFID_FORGET`](commands.md#ace_rfid_forget) does not touch it.
- **The brands.** Anycubic (NTAG213), Bambu Lab and Snapmaker (MIFARE Classic 1K, tried in the
  order of `rfid_mifare_order`). Each was checked against a real tag. Elegoo (NTAG213) and
  Creality (MIFARE Classic 1K) are in the tree but ship **disabled** (`enabled: False` in
  `ace2k_tags.cfg`): no spool on hand carried a tag, so neither format has been checked against a
  real one. Every brand parameter, such as a salt or a key, lives in `ace2k_tags.cfg`. Each section
  names the `source:` it was published in. The firmware carries none.
  A **Snapmaker** spool's tag sits next to the hub, out of reach of the bay's antenna. It reads only
  with the tag taken out and held at the antenna. Its record's RSA signature is not checked in this
  release (`unchecked`). The Creality and Elegoo modules are written from their published sources.
  Enable them at your own risk. With a dump of a real tag (`ACE_RFID_READ LANE=n DUMP=1`), a later
  version can turn them back on by default.

**Adding a brand** needs no firmware change and no reflash:

1. Write a module in `klippy/extras/ace2k_tags/`: a `TagFormat` subclass (`ace2k_tags/base.py`)
   with its `name`, its `chip` (`"ntag"` or `"mifare"`), the parameters it `needs`, and
   `claims(chip)`, `steps(chip, got)` and `decode(chip, got)` → a `SpoolRecord`. A MIFARE brand
   lists its `sectors` and derives a key in `key_for(uid, sector)`; `steps` is then shared. End the
   module with `FORMAT = <the class>`.
2. Add one line to `ace2k_tags/registry.py`'s `MODULES`.
3. Add one `[ace2k_tag <name>]` section to `config/ace2k_tags.cfg`: `enabled`, `source`, the
   parameters.
4. Add a test to `firmware/tests/python/test_ace2k_tags.py` with a synthetic dump. Use a made-up
   UID, never a real tag's. `ACE_RFID_READ LANE=n DUMP=1` writes a session's raw bytes, with the
   UID, to a JSON file in Klipper's log directory. That is the material a module is written from.
   That file is yours and never enters a repository.

## Drying filament

The unit dries on its own. [`ACE_DRY`](commands.md#ace_dry) `TEMP=55 DURATION=4h`:

1. closes the exhaust flaps and runs both fans;
2. holds the chamber near 55 °C. The heater's outlets are driven to at most 8 °C above the target,
   never above 73 °C, and the heater itself stops at 85 °C;
3. opens its flaps once the chamber is warm (near the target, or as warm as the heater can make
   it), and keeps them open;
4. at the end, cools the heater down with the fans, then stops them and closes the flaps.

If the printer goes away during a cycle, the cycle still runs to its end. The events it raised
meanwhile reach the host once it is back. [`ACE_STATUS`](commands.md#ace_status) and
`printer.ace2k.dryer` show the state. Every dryer event is a log line and the Klipper event
`ace2k:dryer`.

A target below the chamber's temperature is accepted. The notice `ambient_above` says the chamber
started above the target. Nothing heats until the chamber falls below it.

What keeps it safe, in layers:

- **The heater module's own limits.** Only one module drives the heater. It does so in leases of
  at most 2 s, which the dryer must keep renewing. Every 10 ms it checks that:
  - both fans are on and reading high;
  - the duty is at most 90 %;
  - both outlet sensors are valid and below 85 °C;
  - the mains is present and plausible;
  - the thermal cutout is clear.

  It fires nothing on a check older than three half-cycles. If a limit is broken, it stops the
  heat and latches.
- **The dryer's protections.** Each of these stops the heat, cools down, is logged, and waits for
  [`ACE_DRY_CLEAR`](commands.md#ace_dry_clear):
  - a heater that does not heat;
  - a temperature that rises faster than the heater can explain. This stands in for a stopped fan,
    because the fans have no speed sensor;
  - the two outlets far apart;
  - the chamber 10 °C above the target (or above the chamber's lowest reading since the start,
    when that was warmer), or above 80 °C;
  - a sensor gone quiet;
  - the mains lost;
  - fans that do not come on.

  [`ACE_DRY_STOP`](commands.md#ace_dry_stop) ends a cycle.
  [`ACE_DRYER_LOG`](commands.md#ace_dryer_log) shows its counters and faults. A cycle lasts at most
  24 h: a longer duration is refused.
- **The fans never stop while an outlet reads above 45 °C.** This holds after a fault, a stop, a
  Klipper shutdown or a reboot.
- **A reset never resumes a cycle.** It is logged as interrupted.
- **A tripped 115 °C thermal cutout** is the hardware's last protection. It keeps the dryer in
  fault until the unit is power-cycled. A software restart does not clear it.
- **A filament inserted with no printer connected** lowers a hotter target to 45 °C, safe for PLA.
  A spool put on its holder without inserting the filament is not seen.

Stay near the unit the first times it dries, and keep its vents clear.

## What the lane LEDs mean

The four lane LEDs are on **PE13 / PE9 / PC15 / PE5**, lanes 1→4, low = on
([`hardware.md`](hardware.md) has the pin-level detail). Steady means a state. Blinking asks you
to take a look. Every lane blinks on one shared clock, so lanes in the same state blink together.

| What | LEDs |
|---|---|
| lane empty | off |
| filament present — idle, assisting or following | on, steady |
| moving — feed, rollback, load, unload | blinking, 1 Hz |
| lane in error | blinking, 5 Hz |
| boot of a proven image, every lane empty | dark 300 ms (the insert sensors settle), then the intro: 1 → 4 → 1 twice, 150 ms a step, once |
| unproven image waiting for the host | chase 1 → 4, 250 ms a step, over every lane |

For comparison, the factory firmware idles with all four LEDs on steady. The bootloader in
recovery strobes all four fast (about 17 Hz). A loaded lane at rest looks like the factory
firmware's idle. To tell them apart, watch for the intro at boot (with an empty unit), or use
[`ACE_STATUS`](commands.md#ace_status).
