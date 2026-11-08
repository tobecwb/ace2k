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
    #uid: <the unit's UID>         # optional; refuse another unit on this serial: (ACEK_STATUS shows it)
    #lane1_insert_mv: 600, 1100    # optional; the factory calibration is used when absent
    #feed_speed: 30                # mm/s, the default of ACEK_FEED and ACEK_ROLLBACK
    #unload_speed: 80             # mm/s; left out: 80, or the unit's top speed when lower
    #load_speed: 60
    #load_park_mm: 300             # the load's parking distance: the path from the drive to the outlet
    #                              # (323 mm on the development unit) minus a margin
    #auto_load: True               # pull a filament in to the parking point when it is inserted
    #assist_speed: 50
    #slip_check_mm: 50             # the supervisors' thresholds; an absent key leaves the firmware's
    #slip_allow_mm: 15
    #stall_check_mm: 20            # slip_check_mm and stall_check_mm under 100 mm, an assist's burst
    #snag_fwd_mm: 20               # the tip snag: motor past a full before the touch (5–50)
    #snag_lag_mm: 5                # the filament behind the motor that is a jam (1.5–20, under fwd)
    #snag_duty_pct: 0              # the duty above the feed-forward that is a jam (0 off, 0–100)
    #snag_back_mm: 5               # the touch (2–20)
    #snag_rest_ms: 500             # the wait for the plunger at rest (100–2000)
    #snag_free_mm: 50              # reverse: the filament back before a taut trip is forgiven (20–200)
    #held_mm: 3                    # the held stop: the filament moving this much is not held (1.5–20)
    #held_ms: 550                  # the held stop: the plunger pulled tight this long (50–2000)
    #unload_clear_mm: 20           # an unload: the motor past the bay's sensor before it ends (0–100)
    #follow_default: True          # arm the follow on every lane with filament, at Klipper's start and
#                              # after an insertion; False: only ACEK_ASSIST DIR=BOTH (or an adapter)
    #follow_lanes: all             # which lanes follow_default arms: all; extruder: only a lane that
    #                              # feeds an extruder (laneN_extruder below, or an adapter's map)
#follow_flip_ms: 200           # the follow: the wait before a move against the last one (50–2000)
    #follow_take_mm: 15            # the follow's take-up on this lane's full (5–30)
    #follow_tail_mm: 2000          # the follow's tail: motor after a runout before it ends (200–2000)
    #lane1_extruder: extruder      # the feed-forward: the extruder lane 1 feeds (lane1..lane4; several
    #                              # lanes may name one: ACEK_LANE_ACTIVE picks the one followed); a
#                              # lane without one has no feed-forward
    #feed_forward: True            # ACEK_FEED_FORWARD ON=1 / OFF=1 switches it live
    #ff_chunk_mm: 3                # the unit's dose (1–10)
    #ff_pulse: 15                  # retired, the next version removes it: mm/s, a reverse
    #                              # dose's least speed (9–80); a forward dose runs at the floor
    #ff_period_ms: 100             # how often the host works out the rate (20–1000)
    #ff_window_ms: 500             # the stretch of planned moves the rate is the mean of (100–5000)
    #ff_lead_ms: 0                 # how far ahead of now that stretch starts (0–5000)
    #ff_deadband: 0.3              # mm/s: a smaller change of the rate is not sent (0–10)
    #ff_notice_every: 20           # corrections one way before a console notice (1–10000)
    #lane1_encoder_scale: 1.2342   # mm per encoder count, lane1..lane4 (ACEK_CALIBRATE_ENCODER)
    #feed_report_hz: 1             # 0.05–10 Hz; 1, 2, 5 and 10 are exact, the rest snap to the 100 ms grid
    #rfid_search_mm: 750          # the load's search for a spool's tag past the parking point,
    #                             # 100–1000 mm of filament (unset: the firmware's default)
    #rfid_mifare_order: bambu, snapmaker, creality
    #heater_watts: 360            # the dryer's heater power, for ACEK_DRYER_LOG's energy

    [include ace2k_tags.cfg]

## Commands in brief

`LANE` is 1–4. Lengths are in mm. Speeds are in mm/s, within the unit's bounds (9–80).

The motion commands ([`ACEK_FEED`](commands.md#acek_feed),
[`ACEK_ROLLBACK`](commands.md#acek_rollback), [`ACEK_LOAD`](commands.md#acek_load),
[`ACEK_UNLOAD`](commands.md#acek_unload)) wait for the move's final event by default. They then
print how far the motor and the filament travelled. If the move ends in an error kind, the command
fails with a G-code error. With `WAIT=0` the command returns at once, and the event goes to the log.

Sometimes a feed or a load meets something that cannot move: the filament is stopped inside the
unit, or held in the tube further on. The move then stops without an error. The console shows
`ace2k: lane n feed blocked — motor … mm, filament … mm` (`load blocked` for a load). The lane
is idle where it stopped, and there is nothing to clear. A loose piece left in the tube does not
block a move: the push moves it ahead.

- To read the status, use [`ACEK_STATUS`](commands.md#acek_status) and
  [`ACEK_HEALTH`](commands.md#acek_health).
- To get a lane out of an error, use [`ACEK_CLEAR`](commands.md#acek_clear).
- The whole list is in [`commands.md`](commands.md).

## Several units

A printer can have more than one ACE 2 Pro. Each unit needs its own USB connection and its own
`[mcu]` section. Units are not chained on one port
([decision 0002](decisions/0002-one-unit-per-serial-port.md)).

Each unit runs everything a single unit runs: the feed, the follow, the feed-forward, the tags, the
dryer and the sensors. Nothing in it depends on another unit. What spans units is kept by one
coordinator, the printer object `ace2k_units`: which lane feeds each extruder, the one active lane
of an extruder that several lanes feed, and the `UNIT=` of the commands.

To use several units, name each section and give it its `mcu:`. The key is required in a named
section. `config/ace2k.cfg` has the example, commented out.

    [mcu ace2k_a]
    serial: /dev/serial/by-id/<unit a's USB adapter>
    baud: 250000
    restart_method: command

    [ace2k a]
    mcu: ace2k_a
    uid: <unit a's UID>
    lane1_extruder: extruder

    [mcu ace2k_b]
    serial: /dev/serial/by-id/<unit b's USB adapter>
    baud: 250000
    restart_method: command

    [ace2k b]
    mcu: ace2k_b
    uid: <unit b's UID>
    lane3_extruder: extruder

    [temperature_sensor ace_a_chamber]
    sensor_type: ace2k
    unit: a
    sensor: chamber

- **Name every unit or none.** `[ace2k]` alone is one unit, as always. `[ace2k]` together with
  `[ace2k <name>]` is a config error. The names are compared without regard to case, and each
  section has its own `mcu:`. A name holds only letters, digits, `_` and `-` (`a`, `left`,
  `unit_2`); any other is a config error.
- **Give each unit its `uid:`.** The `serial:` of a unit's `[mcu]` is the USB adapter in its
  cable, not the unit. Swap two cables and `[ace2k a]` drives unit b: its lanes feed other heads,
  and each lane's encoder calibration lands on the wrong unit. `uid:` is the unit's own
  identifier, the 24 hexadecimal digits `ACEK_STATUS` prints on the line after the version (also
  `printer["ace2k a"].uid`). Spaces, colons, dashes and either case are accepted. At every
  connect the host compares it with the unit that answers, once every unit has answered (so each
  freshly flashed unit still gets its link proof). Another unit stops Klipper with one line per
  unit that does not match, naming the section whose `uid:` the unit has:
  `ace2k a: the unit on this serial port is <uid> (the uid: of [ace2k b]), not the uid: <uid> of
  [ace2k a]. The cables of two units may be swapped …`. Put the cables back, or fix the
  `serial:`, and restart. With `uid:` set, a unit that cannot tell its identifier (a firmware
  built without the query, or an answer of the wrong length) is an error too, which says so.
  Without `uid:` nothing is checked. The key works on a lone `[ace2k]` as well.
- **Sensors say which unit they read.** A `[temperature_sensor]` of type `ace2k` takes
  `unit: a`. With one unit the key may be left out. With several it is required. With one `[ace2k]`
  section, which has no name, leave it out: any `unit:` there is a config error that says to omit it. Put the unit
  sections before the sensor sections in `printer.cfg`: Klipper reads them in order, and a sensor
  before a unit section stops with `sensor NAME must come after every [ace2k] section`.
- **Name each sensor after its unit**, so Fluidd and Mainsail show which unit it reads:
  `ace_a_chamber`, `ace_a_ptc_left` and `ace_a_ptc_right` for unit a, `ace_b_chamber` and so on
  for unit b. ace2k does not read the name. `config/ace2k.cfg` gives a lone `[ace2k]` the names of
  unit a, so they stay the same when you add unit b. A renamed sensor is a new sensor to Klipper
  and to the web pages: its graph starts empty.
- **Commands name the unit** with `UNIT=`. The rules are in
  [commands.md](commands.md#several-units). With no `UNIT=`, `ACEK_STOP` stops all, `ACEK_STATUS`
  and `ACEK_HEALTH` show all, and any other command is refused, `ACEK_STOP LANE=n` included.
- **The console names the unit.** A line from unit b starts `ace2k b:`.
- **A unit's objects have its name.** In a macro, unit b's status is `printer["ace2k b"]`. Its feed
  object is `printer.lookup_object("ace2k b").feed`. The coordinator's status is
  `printer.ace2k_units`: `units` lists the section names as written (`"ace2k a"`, `"ace2k b"`;
  `"ace2k"` for a lone section) and `active_lanes` the active lane of each shared extruder (`b:3`,
  or `None` before you choose). A unit's own `active_lanes` (`printer["ace2k a"].active_lanes`)
  covers the shared extruders its lanes feed: its own lane's number when the active lane is its
  own, the other unit's lane as `b:3` when it is not, and `None` only before you choose.
- **Tags are cached per unit.** `ace2k_tags_cache.json` serves a lone `[ace2k]`. A named unit
  keeps `ace2k_tags_cache_<name>.json` beside it. When you rename `[ace2k]` to add units, keep
  that unit's section first: it takes over the old tag cache. At its first start the first unit
  in the config reads `ace2k_tags_cache.json` and writes it to its own file at once, so the
  cached tags are not lost; from then on it reads only its own file. The old file is then renamed
  `ace2k_tags_cache.json.adopted`: its data is kept but never read again, so it is taken over
  once only, even if the unit's own file is deleted or the sections are reordered later. The
  other units start with an empty cache.
- **A lost link stops Klipper.** As with any MCU, a unit whose USB connection drops shuts Klipper
  down. This holds for every unit ([limitations](limitations.md)).

Lanes of different units can feed one extruder. See "Several lanes on one extruder" below; the
rules are the same across units, and you choose the active lane with
[`ACEK_LANE_ACTIVE`](commands.md#acek_lane_active) `UNIT=b LANE=3`.

## How the unit keeps up with the print head

Between the unit and the head sits the buffer: a spring-loaded plunger the filament runs through.
When the head pulls the filament, the plunger moves toward one end. When the head pushes the
filament back, the plunger moves toward the other end.

ace2k has three ways to respond:

- **Assist.** This is the general mechanism. The unit watches the plunger and runs the lane's
  motor to bring it back to rest. You choose the direction with
  [`ACEK_ASSIST`](commands.md#acek_assist) `DIR=`:
  - **forward** (the default): the unit only *feeds*. When the head pushes filament back, the unit
    does not take it up. The buffer absorbs it.
  - **back** (`DIR=BACK`): the unit only *takes up*.
  - **both** (`DIR=BOTH`): the unit feeds when the head pulls, and takes up when the head pushes
    back. **This is the follow.**
- **Follow.** The follow is not a separate feature: it is the assist in both directions. It exists
  because a printer sometimes pulls and pushes within a few seconds. It does so when it forms the
  tip during an unload, and when it retracts during a print. A one-way assist would let filament
  pile up in the buffer. [`ACEK_FOLLOW_SET`](commands.md#acek_follow_set) only changes its settings.
  On the Snapmaker U1, the adapter uses the follow, never the one-way assists.
- **Feed-forward.** It works on top of the follow, and it runs on the host. The assists *react*:
  the plunger has to move before the unit does anything. The feed-forward *anticipates*: the host
  reads the moves Klipper has already planned for the extruder. The unit then feeds that filament
  in small doses, just before the head needs it
  ([`ACEK_FEED_FORWARD`](commands.md#acek_feed_forward), [`ACEK_FF_SET`](commands.md#acek_ff_set)).
  It works only while the lane is in the follow.

| | Watches | Moves the filament | Set with |
|---|---|---|---|
| Assist, forward | the buffer | toward the head only | `ACEK_ASSIST` |
| Assist, back | the buffer | back only | `ACEK_ASSIST DIR=BACK` |
| **Follow** | the buffer | both ways | `ACEK_ASSIST DIR=BOTH`, tuned by `ACEK_FOLLOW_SET` |
| Feed-forward | Klipper's planned extruder moves | toward the head, ahead of the pull | `ACEK_FEED_FORWARD`, `ACEK_FF_SET` — only during the follow |

Example during a print: the extruder is about to pull 5 mm.

- With the follow alone, the head first pulls the plunger, and then the unit feeds. The correction
  comes after the fact.
- With the feed-forward on, the host sees the 5 mm in Klipper's plan. The unit feeds them as the
  head starts to pull. The follow only corrects what is left.

### The follow is on by default

ace2k puts every lane that has filament in the follow by itself. You do not need a macro or a load
for that. It works the same on the Snapmaker U1 and on any other Klipper printer.

- **At Klipper's start** (a restart, or a reset of the unit), every lane whose insert sensor sees
  filament goes into the follow. An empty lane stays off. A lane in error is left alone.
- **When you insert a filament** later, its lane goes into the follow once it is free. If the unit
  loads the new filament by itself (`auto_load`), the follow starts after that load.
- **The console says it**: `ace2k: lane N follow on (follow_default)`. The lane's
  `follow_default` in `printer.ace2k.lanes[n]` reads `waiting`, `arming`, `armed` or `off`.
- **A stop of yours wins.** After `ACEK_STOP` or `ACEK_ASSIST … OFF=1`, ace2k leaves the lane
  off. It arms it again only after the filament is taken out and put back, or at the next Klipper
  start.
- **A move you start on a lane in the follow pauses the follow.** `ACEK_FEED`, `ACEK_ROLLBACK`,
  `ACEK_LOAD`, `ACEK_RFID_READ … MOVE=1` and a hand calibration
  feed stop the follow first, then run. The console says `ace2k: lane N follow paused for the
  feed` (or the load, and so on). When the move ends cleanly and the filament is still in the bay,
  the follow comes back: `ace2k: lane N follow on again`.
  - If the move ends in an error, ends `blocked`, or is stopped, the follow stays off:
    `ace2k: lane N follow stays off — the feed ended stuck`. Fix the cause, then take the filament
    out and put it back, or arm the follow by hand.
  - `ACEK_UNLOAD` stops the follow and does not bring it back, because the filament leaves:
    `ace2k: lane N follow stopped for the unload`. The next insertion arms the lane as usual.
  - A one-way `ACEK_ASSIST` (forward or back) takes the follow's place: `ace2k: lane N follow
    stopped for the assist`. The follow does not come back by itself.
  - With `follow_default: False`, or on a lane that is not the active one of its extruder, the
    move stops the follow (`follow stopped for the feed`) and the follow does not come back.
  - If the follow does not stop within 5 s, the move does not start, and the follow comes back
    once the lane is idle: `ace2k: lane N feed not started: the follow was stopped but did not end
    within 5 s; it comes back once the lane is idle`.
- **The head does not matter.** The follow keeps the buffer at rest whether the filament reaches
  the head or not. The feed-forward feeds the lane in the follow from its extruder's moves, when the
  lane has one (`laneN_extruder`, below).
- **To switch it off**, set `follow_default: False` in `[ace2k]`. Then nothing is armed by itself,
  as before.
- **Only the lanes that feed an extruder**, with `follow_lanes: extruder` in `[ace2k]`. A lane
  that feeds no head then stays off, so a plunger moved by hand, or a tube come loose, never pushes
  its filament. A lane feeds an extruder when it has `laneN_extruder`, or when an adapter maps it
  (a U1 adapter maps the lanes of its `lanes:`). The default, `all`, arms every lane with
  filament. An adapter can also hold a lane out of the follow, with a reason: a U1 adapter holds
  the lanes outside its `lanes:`.
- **Why a lane does not follow.** `ACEK_STATUS` ends each lane's feed line with `follow=`: `armed`,
  `arming`, `waiting`, `off`, or `off (<why>)` when something keeps the follow off. The reasons:
  `follow_default: False`; `held: <the adapter's reason>` (for example
  `held: not in [ace2k_u1] lanes`), or `held by a host module`; `feeds no extruder, follow_lanes:
  extruder`; `extruder1: no lane chosen, ACEK_LANE_ACTIVE`, or `extruder1: lane 2 is its active
  lane`. The same reason is in `printer.ace2k.lanes[n].follow_why` (None when nothing keeps it off).

**Several lanes on one extruder.** When two or more lanes name the same extruder
(`laneN_extruder`), only one of them can be in use. ace2k follows only the **active** lane of that
extruder, and the feed-forward feeds only that lane.

- At Klipper's start no lane of such a group is armed. The console says once
  `ace2k: lanes 1, 2 share extruder: choose one with ACEK_LANE_ACTIVE`.
- [`ACEK_LANE_ACTIVE`](commands.md#acek_lane_active) `LANE=n` makes lane n the active lane. It
  stops the follow on the other lanes of that extruder and arms lane n.
- The choice lasts until Klipper restarts. It is not saved.
- When the lanes of a group change (a host module maps a lane to the extruder or away from it),
  the group is a new one. The console says the "choose one" line again. The active lane stays
  only while it is still in the group. A lane mapped away and back is not active again until you
  run `ACEK_LANE_ACTIVE` for it.
- Lanes with their own extruder, and lanes with no `laneN_extruder`, are armed at the start as
  above. On the Snapmaker U1 each lane has its own head, so nothing changes there.
- Lanes of different units may share an extruder. The group, the active lane and the console lines
  then span the units ("Several units", above), and the lanes read `a:1`, `b:3`. The "choose one"
  line is said only for a group with a lane on a unit whose `follow_default` is on: a group of
  lanes the follow never arms by itself has nothing to choose.
- `ACEK_STATUS` names the active lane of each shared extruder. `printer.ace2k.lanes[n].active` is
  `False` for a lane of a shared extruder that is not the active one, and `printer.ace2k.active_lanes`
  gives the active lane per shared extruder (`None` before you choose).

Sometimes the tip of the filament catches at the entrance of the plunger's tube.
[`ACEK_SNAG_SET`](commands.md#acek_snag_set) sets how the unit handles this. A feed, a rollback,
an unload or a read with motion that meets it does not end at once: the unit makes one short move
back (the "touch"), within set limits, and announces it. A load from the bay, and a feed started
with `FREE=1`, have no touch: they judge a full plunger by whether the filament follows.

[`ACEK_STOP`](commands.md#acek_stop) ends any assist.

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

[`ACEK_STOP`](commands.md#acek_stop) ends the tail like any follow.

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
- or by hand, with [`ACEK_GRIP`](commands.md#acek_grip).

The grip is cleared:

- by the load, which uses it up;
- by any accepted start on the lane, or a tag read with motion;
- at every connect.

The tag of a gripped spool is not searched for. It is read on the lane's next move, or with
[`ACEK_RFID_READ`](commands.md#acek_rfid_read) `LANE=n MOVE=1`. [`protocol.md`](protocol.md#the-grip), "The grip", has the full details.

## Feeding ahead of the extruder (feed-forward)

A lane in the follow feeds only once the head has pulled its filament tight. Between those bursts,
the filament stays stretched.

With the feed-forward, the host reads the moves Klipper has planned for the extruder the lane
feeds, a little ahead of the head. The unit then feeds that filament out in small doses, ahead of
the pull: `ff_chunk_mm` (3 mm) at the unit's floor speed, as every move of 10 mm or less runs.
The plunger stays near rest.
[`ACEK_FEED_FORWARD`](commands.md#acek_feed_forward) switches it on and off.
[`ACEK_FF_SET`](commands.md#acek_ff_set) tunes it.

The follow still corrects on top of it:

- when the filament is pulled tight, the follow starts its burst;
- when this lane's buffer is full, the follow takes filament up.

Either correction drops the filament the lane still owed. The feed-forward works only on a lane in
the follow ([`ACEK_ASSIST`](commands.md#acek_assist) `… DIR=BOTH`, or an adapter such as
`ace2k-u1`). Stopping the follow ends it. [`protocol.md`](protocol.md), "The feed-forward", has the
full details.

- **Which extruder each lane feeds.** `laneN_extruder` names the extruder lane N feeds
  (`extruder`, `extruder1`, …). Several lanes may name the same extruder, for example on a
  single-extruder printer with more than one spool. Only the lane in the follow is fed. If two of
  them are in the follow at once, the host cannot tell which to feed: neither gets a rate, a
  console line says so, and each keeps the plain follow. With `follow_default` on, ace2k follows only
  the active lane of such an extruder (`ACEK_LANE_ACTIVE`, "The follow is on by default", above), so
  this should not happen in normal use. Across units the rule is the same: a unit feeds an
  extruder only when its own lane is the only lane of that extruder in the follow, on any unit.
  A lane whose follow has ended (its final event came) is not in the follow, even before its
  unit's next report says so.
  If two are, the console says it once. When they are on more than one unit, the line starts with
  the first unit that has a follower and says `both` for two lanes, `all` for more
  (`ace2k a: extruder extruder: lanes a:1, b:3 both in follow — no feed-forward`). It is said
  once for the whole conflict: a lane that leaves while two others still follow changes nothing,
  and the line comes again only after one follower at most is left and a new conflict starts.
  When they are
  all on one unit, that unit prints its own line, as with one unit, and the others print nothing. A lane without
  a key has no feed-forward, unless a host module maps it (the host interface's
  `set_lane_extruder`; the U1 adapter maps every lane it hooks to its head's extruder). A key wins
  over that.
- **The rate.** Every `ff_period_ms`, the host takes the mean extrusion speed of the planned moves
  in a window of `ff_window_ms`, starting `ff_lead_ms` from now. A retraction is not lost: it is
  credited against the next extrusion, so that much of the next extrusion is held back. The host
  sends the rate when it changed by more than `ff_deadband`, or fell to 0.
- **The status.** `printer.ace2k.lanes[n]` carries `ff_on`, `ff_extruder`, `ff_rate` (mm/s, the last
  one sent; negative while the lane takes a long retraction back), and the unit's counters since the
  lane's mode began: `ff_doses`, `ff_back_doses` (`None` on an image without the reverse),
  `ff_taut_fixes` and `ff_full_fixes`.
- **The notices.** The host counts the corrections in each direction since the last notice.
  Corrections in opposite directions cancel. They count only while the lane is metering (its
  feed-forward on, and a rate above 0 sent). Every `ff_notice_every` corrections in one direction,
  the host prints a console line:

      ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale (ACEK_CALIBRATE_ENCODER)

or the same with `full corrections`. A dose is a length measured on the lane's encoder. So **a
steady stream of notices in one direction means the encoder scale is wrong. Run
[`ACEK_CALIBRATE_ENCODER`](commands.md#acek_calibrate_encoder)** ("Calibrating the encoder", below).
The lane keeps working meanwhile. Nothing corrects the scale on its own.
- **An older image** has no feed-forward. The console shows `ace2k: the unit's firmware has no
  feed-forward; flash v0.9.0` once, and `ACEK_FEED_FORWARD` and `ACEK_FF_SET` are refused. An image
  older than v0.11.0 also has no follow for this host ("When a spool runs out during a print",
  above), so there is nothing to feed ahead for.

## Calibrating the encoder

A lane measures its lengths on its filament encoder: a feed, a rollback, and every feed-forward
dose. So the encoder's scale (mm per count, `laneN_encoder_scale`, 1.2342 by default) sets how much
filament the lane delivers. A wrong scale shows as feed-forward corrections that all go one way.

[`ACEK_CALIBRATE_ENCODER`](commands.md#acek_calibrate_encoder) `LANE=n AUTO=1` measures the scale
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
  1. On an idle lane, it arms the follow (as [`ACEK_ASSIST`](commands.md#acek_assist)
     `LANE=n DIR=BOTH`, at `assist_speed`). A lane already in the follow stays in it.
  2. It holds the lane's feed-forward for the run: no rate is sent, what the lane owed is dropped,
     and the [`ACEK_FEED_FORWARD`](commands.md#acek_feed_forward) switch is not touched. So only the
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

  The new scale is in use at once: [`ACEK_STATUS`](commands.md#acek_status) shows
  `scale=1.2462 (calibrated)`. It is also staged as `laneN_encoder_scale`: **`SAVE_CONFIG` keeps
  it**. At every connect, every lane's scale is sent to the unit again, the saved one or the
  default. So a calibration you did not save is undone by a Klipper restart, like any unsaved
  setting. The lane's `encoder_mm` (`printer.ace2k.lanes[n]`) goes on across the change without a
  jump ([`protocol.md`](protocol.md), "Host interface").
- **What it refuses.** Each refusal gives its reason, before anything is sent:
  - a print printing or paused;
  - a lane without an extruder, or whose extruder is not the active one (`select it first`) or is
    too cold to extrude;
  - a lane without filament, in error ([`ACEK_CLEAR`](commands.md#acek_clear) first) or busy in
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
  1. `ACEK_CALIBRATE_ENCODER LANE=n LENGTH=200` feeds 200 mm with the end of the filament free
     (unloaded from the head, the tube off it).
  2. Measure what came out.
  3. `ACEK_CALIBRATE_ENCODER LANE=n MEASURED=<mm>` stages the scale for `SAVE_CONFIG`. The unit
     takes it at the restart.

## Spool tags

With `[include ace2k_tags.cfg]` in `printer.cfg` (copy `config/ace2k_tags.cfg` next to it), the
unit reads the RFID tag of the spool in each lane, and the host names the spool.

- **When you insert a filament**, the load pulls it in while the lane's reader watches. When the
  tag passes the antenna, the lane stops, the tag is read, and the load goes on. If the tag has not
  passed by the parking point, the unit looks further: up to `rfid_search_mm` of filament. That is
  one turn of the largest spool that fits, plus the tag's window (750 mm, measured). It does this
  only while every other lane is idle. The filament always comes back to the parking point. A
  waited [`ACEK_LOAD`](commands.md#acek_load) allows for that travel.
- **A lane whose tag was not read** (the search gave way to another lane, or the host was not
  there) is watched on every later move, such as a feed or an assist. Its tag is read on the fly.
  [`ACEK_RFID_READ`](commands.md#acek_rfid_read) asks for a read at any time. With `MOVE=1` it is
  taken while the other lanes only follow, at rest. If one of them moves during the search, the
  search gives way to it at once and comes back, and the lane stays `pending`.
- **Two lanes share each antenna** (1–2, 3–4). A tag is given to a lane only when that lane's
  motion explains it: a tag read while that lane moves. A lane that has not moved stays
  `pending`. A read without motion gives no new tag. It reads again only the lane's own tag, given
  earlier by its motion. Another tag in the field answers *ambiguous*; with none, the lane's state
  is answered unchanged. Use `MOVE=1` then.
- **The record** (brand, material, colour, weight, temperatures, what the tag carries) is in
  [`ACEK_STATUS`](commands.md#acek_status), in `printer.ace2k.lanes[n].tag` (`state`, `uid`,
  `source`, `record`), and in the Klipper event `ace2k:tag_read`. Each is also a log line. A tag
  that no brand recognises still counts, by its UID. Every record is cached by UID in
  `ace2k_tags_cache.json` beside `printer.cfg`. When a tag is read again with the UID alone (the
  read could not keep up with the move, was given up, or no brand's key opened it this time), it
  takes the cached record, marked `cached`. Deleting the file empties the cache.
  [`ACEK_RFID_FORGET`](commands.md#acek_rfid_forget) does not touch it.
- **Tags across a restart.** The last read of each lane is kept
  beside the cache, in `ace2k_tags_cache.lanes.json` (`ace2k_tags_cache_<name>.lanes.json` for a
  named unit). A restart that resets the unit leaves every full bay `pending`. Such a lane takes
  its last read back: its `tag` shows `state` `read` and `source` `restored`. It counts as the
  lane's tag until the next read of the lane replaces it, and `ace2k:tag_read` is sent for it. The
  unit has not seen the tag yet, so the lane's next load still searches for it. The kept read
  goes when the bay is seen empty (by hand or by a move), on
  [`ACEK_RFID_FORGET`](commands.md#acek_rfid_forget), on a read with motion, when a search ends
  `no_tag`, and when another tag is seen on the lane. A spool swapped while the printer was off
  keeps the old read until its tag is read again. If the file cannot be written, the change is
  undone and tried again a minute later at the earliest.
- **The brands.** Anycubic (NTAG213), Bambu Lab and Snapmaker (MIFARE Classic 1K, tried in the
  order of `rfid_mifare_order`). Each was checked against a real tag. Elegoo (NTAG213) and
  Creality (MIFARE Classic 1K) are in the tree but ship **disabled** (`enabled: False` in
  `ace2k_tags.cfg`): no spool on hand carried a tag, so neither format has been checked against a
  real one. Every brand parameter, such as a salt or a key, lives in `ace2k_tags.cfg`. Each section
  names the `source:` it was published in. The firmware carries none.
  A **Snapmaker** spool's tag sits next to the hub, out of reach of the bay's antenna. It reads only
  with the tag taken out and held at the antenna. Its record's RSA signature is not checked in this
  release (`unchecked`). The Creality and Elegoo modules are written from their published sources.
  Enable them at your own risk. With a dump of a real tag (`ACEK_RFID_READ LANE=n DUMP=1`), a later
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
   UID, never a real tag's. `ACEK_RFID_READ LANE=n DUMP=1` writes a session's raw bytes, with the
   UID, to a JSON file in Klipper's log directory. That is the material a module is written from.
   That file is yours and never enters a repository.

## Drying filament

The unit dries on its own. [`ACEK_DRY`](commands.md#acek_dry) `TEMP=55 DURATION=4h`:

1. closes the exhaust flaps and runs both fans;
2. holds the chamber near 55 °C. The heater's outlets are driven to at most 8 °C above the target,
   never above 73 °C, and the heater itself stops at 85 °C;
3. opens its flaps once the chamber is warm (near the target, or as warm as the heater can make
   it), and keeps them open;
4. at the end, cools the heater down with the fans, then stops them and closes the flaps.

If the printer goes away during a cycle, the cycle still runs to its end. The events it raised
meanwhile reach the host once it is back. [`ACEK_STATUS`](commands.md#acek_status) and
`printer.ace2k.dryer` show the state. Every dryer event is a log line and the Klipper event
`ace2k:dryer`.

### Drying on its own

> [!CAUTION]
> **The humidity mode turns a mains heater on by itself, with nobody there.** The unit starts a
> drying cycle whenever its box stays humid, even with the printer off. The mode runs only with
> `unattended_drying: True`. Set it only with the unit in a ventilated place, with nothing that
> burns next to it, and nothing on the lid or over the vents. Read the dryer warnings in
> [README](../README.md#read-this-first) first.

The dryer has three modes. They choose when a cycle starts. The cycle itself is the same.

Why: a box left alone gets humid again. The automatic modes dry it for you, so you do not start
a cycle by hand each time.

- **Manual.** The dryer runs only when you start it with `ACEK_DRY`. This is the default.
- **Humidity.** The unit dries when the box stays humid, with the printer on or off. It reads
  the box only when it is cold: the dryer idle and the chamber steady (it neither fell nor rose
  0.5 °C in 30 min). If the humidity stays at or above the limit (40 % by default) for 30 min, it
  starts a cycle. When the box is cold
  again, it reads the humidity once more. Below the limit, the cycle worked. At or above it, the
  cycle failed. After four failed cycles in a row (`dry_max_failures`) the mode stops, and says
  so in `ACEK_STATUS`. A cycle of its own that stops with the fault `lid` also counts as a failed
  cycle, at once. Check that the lid is closed, then choose the mode again.
- **Print.** The host dries during a print. At the print's start, if the box is cold and at or above
  the limit, it starts a cycle. If not, it starts one when the humidity stays at or above the
  limit for 30 min during the print. It stops the cycle at the print's end. It needs `[virtual_sdcard]`.
  If the host link is down for 5 min, the unit ends the cycle.

The temperature follows the most fragile material among the lanes with a strand. The material
comes from the spool's tag, or from Edit spool on the U1. A lane with no known material counts as
PLA (45 °C). A cycle of the humidity mode lasts 6 h. A cycle of the print mode lasts up to 24 h,
starts again if it ends by its time, and stops at the print's end.

What to do:

1. Choose the mode: [`ACEK_DRY_MODE`](commands.md#acek_dry_mode) `MODE=print`. The unit keeps it
   across a power loss.
2. For the humidity mode, first add `unattended_drying: True` to the unit's section in
   `printer.cfg`. The default is `False`, and then the mode is refused. The unit then heats with
   nobody there, so keep it in a ventilated place, with nothing on the lid or over the vents.
3. To change the limit or the number of failed cycles, set `dry_humidity_limit` or
   `dry_max_failures` in the same section.

A cycle you start with `ACEK_DRY` always wins: while an automatic cycle is starting or heating, it replaces it. If you stop an
automatic cycle by hand, the humidity mode turns off. In the print mode the dryer then stays off
until the print ends.

The unit keeps the mode and its counts in its config page. If you flash an older image, it still
reads the page, and it drops only these settings: the mode returns to manual.

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
  [`ACEK_DRY_CLEAR`](commands.md#acek_dry_clear):
  - a heater that does not heat;
  - a temperature that rises faster than the heater can explain. This stands in for a stopped fan,
    because the fans have no speed sensor;
  - the two outlets far apart;
  - the chamber 10 °C above the target (or above the chamber's lowest reading since the start,
    when that was warmer), or above 80 °C;
  - a sensor gone quiet;
  - the mains lost;
  - fans that do not come on.
  - a chamber that does not follow the heater. With the lid open, the fans blow room air over the
    chamber sensor and the box does not warm. From the 8th minute of heating, the chamber's rise
    must be at least 10 % of the outlets' rise, both measured from the chamber's reading at the
    entry into heating. Below that for 2 min, the dryer stops with the fault `lid`. It is not
    judged while the outlets are less than 5 °C above that reading, or while the target is less
    than 5 °C above it. A chamber that starts near its target has no room to rise, so an open lid
    on such a box is not caught. The numbers were measured on the unit with the lid open and
    closed.

  [`ACEK_DRY_STOP`](commands.md#acek_dry_stop) ends a cycle.
  [`ACEK_DRYER_LOG`](commands.md#acek_dryer_log) shows its counters and faults. A cycle lasts at
  most 24 h: a longer duration is refused.
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
| every lane empty and idle (after the intro, if one plays) | all four flash together for 100 ms every 10 s, the first flash 10 s after the last lane went dark |
| boot of a proven image, every lane empty | dark 300 ms (the insert sensors settle), then the intro: 1 → 4 → 1 twice, 150 ms a step, once |
| unproven image waiting for the host | chase 1 → 4, 250 ms a step, over every lane |

The short flash of all four tells you the unit is on when no lane has filament. You see it only
while every lane is empty and idle. When you insert a filament, it stops at once and the lane shows
its own state. A lane in error always shows its error: the flash never covers it. Drying does not
change it.

`ace2k_flash.pyz --identify` blinks all four together for 10 s, so you can tell which unit is on a cable.

For comparison, the factory firmware idles with all four LEDs on steady. The bootloader in
recovery strobes all four fast (about 17 Hz). A loaded lane at rest looks like the factory
firmware's idle. To tell them apart, watch for the intro at boot (with an empty unit), or use
[`ACEK_STATUS`](commands.md#acek_status).
