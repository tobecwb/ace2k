# Changelog

## v0.14.0 — 2026-11-08

The unit's top speed is now 80 mm/s, the same as the factory firmware. The host interface is
unchanged, at version 28.

### Changed

- **The top speed is 80 mm/s** (it was 70). The dictionary constants
  `ACE2K_LANE_SPEED_MAX_UM_S` and `ACE2K_FEED_FF_PULSE_UM_S_MAX` are 80000. With a full spool, the
  motor often runs at or near 100 % duty at 80 mm/s, and the lane then runs a little slower than
  asked. Nothing stops a move for that
  ([`hardware.md`](docs/hardware.md), "Motors").
- **`unload_speed` defaults to 80 mm/s** (it was 70). If the unit's top speed is lower, the
  default is that top speed. So an older image with a top speed of 70 mm/s unloads at 70, and
  `klippy.log` says `unload_speed defaults to the unit's top speed, 70 mm/s`. A value you set is
  still checked at connect. For example, `unload_speed: 80` with an older image stops Klipper
  with `unload_speed must be within 9..70 mm/s`.
- **A warning for a small `snag_duty_pct`.** At 80 mm/s a loaded lane can run at 100 % duty, 11
  points above what the feed-forward asks. A `snag_duty_pct` from 1 to 11 may then stop healthy
  moves. Klipper now warns about such a value at connect, in `klippy.log`. Nothing is refused.
  The default, 0, turns this guard off.
- **Tested on the paxx extended firmware `2.0.0-paxx12-24`** on the Snapmaker U1.

## v0.13.0 — 2026-11-03

The changes since `v0.12.0-rc.1`. There was no final `v0.12.0`: this release follows the release
candidate. The host interface is at version 28. [`protocol.md`](docs/protocol.md), "Host
interface", lists what each version added.

### Breaking

- **A read without motion no longer gives a tag to a lane** (2026-11-02, host interface 28).
  Two lanes share each tag reader. `ACEK_RFID_READ` without `MOVE` now only reports what is in the
  reader's field. A tag goes to a lane only with `MOVE=1`, or when that lane moves. If you used
  `MOVE=0` to read a spool at rest, move the spool instead
  ([`commands.md`](docs/commands.md), `ACEK_RFID_READ`).
- **The temperature sensors are named after their unit** (2026-10-22). The example
  `config/ace2k.cfg` names them `ace_a_chamber`, `ace_a_ptc_left` and `ace_a_ptc_right`. A config
  with the old names still works. If you rename them, Klipper treats them as new sensors: their
  graphs start empty, and a macro that reads an old name must change too
  ([`features.md`](docs/features.md), "Several units").
- **The flasher is one file, `ace2k_flash.pyz`** (2026-10-20). Run it with
  `python3 ace2k_flash.pyz`. Its options are unchanged. `firmware/tools/ace2k_flash.py` is gone
  ([`flashing.md`](docs/flashing.md)).
- **Every G-code command is named `ACEK_*`.** See the entry below, under "Earlier changes since
  the release candidate".

### Removed in the next version

- **`ff_pulse`** (2026-10-24). It now only sets the least speed of a reverse dose. It still works,
  and Klipper warns about it at the start. Remove it from `printer.cfg`
  ([`commands.md`](docs/commands.md), `ACEK_FF_SET`).
- **A `snag_lag_mm` from 1 mm up to 1.5 mm** (2026-10-20). The least lag is now 1.5 mm. A smaller
  value is raised to 1.5 mm, with a warning in `klippy.log`. The next version refuses it. Set
  `snag_lag_mm` to 1.5 or more.

### New

- **The dryer's modes** (2026-10-26). `ACEK_DRY_MODE MODE=manual|humidity|print`. In the humidity
  mode the unit dries by itself whenever its box stays humid, even with the printer off. It needs
  `unattended_drying: True` in the unit's section: read the dryer warnings in the
  [README](README.md#read-this-first) first. In the print mode the host dries during a print. The
  options `dry_humidity_limit` and `dry_max_failures` set the limit and the failed cycles
  ([`features.md`](docs/features.md), "Drying on its own").
- **A new dryer fault, `lid` (12)** (2026-11-01). From the 8th minute of heating, the cycle stops
  when the chamber rises less than 10 % as far as the heater outlets for 2 min. The lid is likely
  open. A chamber that starts within 5 °C of its target is not judged.
- **A unit can show itself** (2026-10-27). `ace2k_identify` blinks the unit's four LEDs together
  for 10 s. The flasher has `--identify --port PORT` for the same.
- **The flasher works with Klipper stopped** (2026-10-27). It sends the unit into recovery over
  its cable, so an installer stops Klipper once for several units.
- **A full bay keeps its last tag across a restart** (2026-10-31). After a restart, a bay that
  still holds a spool shows its last tag read until the next read.

### Changed

- **`load_speed` defaults to 60 mm/s** (it was 30), and **`unload_speed` to 70 mm/s** (it was 30).
- **An unload turns 20 mm more past the bay's sensor** (`unload_clear_mm`, 0 to 100), so the end
  of the filament does not stop on the sensor.
- **Fewer tags given to the wrong lane.** An empty bay no longer gives its reader's tag to the
  other lane, and a lane's known tag stays that lane's.
- **`ACEK_HEALTH CLEAR=1 RUN=1` clears after the new result**, not before it.
- **Many small fixes** from the code reviews of the firmware, the host and the flasher: the link,
  the tag readers, the dryer's controller and the flasher's checks.

### Earlier changes since the release candidate

- **Editing an ace2k key no longer resets the unit** (2026-10-17). `laneN_insert_mv`,
  `laneN_encoder_scale` (the calibration `SAVE_CONFIG` stores included), `slip_check_mm`,
  `slip_allow_mm`, `stall_check_mm` and `rfid_search_mm` were config commands: they entered
  Klipper's config checksum, so changing one and restarting reset the unit, and Klipper restarted
  inside its own process, which fails on the Snapmaker U1 (`Device or resource busy` in its RFID
  reader). They are init commands now, sent at every connect with the key's value or the default;
  a lane without `laneN_insert_mv` goes back to its own band with the new
  `ace2k_sensors_thresholds_reset`, and the supervisors' defaults are read from the image's
  dictionary. An image change, which the image says with the constant `ACE2K_KEYS_AS_INIT`: with
  an older image the host keeps every key a config command, as before, so updating only the host
  leaves Klipper's checksum as it was
  ([`protocol.md`](docs/protocol.md)).

- **One FIRMWARE_RESTART for boards still stopped from before** (2026-10-17). When the first
  stop of a start of Klipper's process is a board reporting it was already stopped (`Command
  request`, what an emergency stop or the shutdown of every board when a unit was lost leaves),
  ace2k looks at every MCU three seconds later; if Klipper is not ready and every stopped one
  reports the same, it sends `FIRMWARE_RESTART` once and says so in `klippy.log`. A unit keeps such a
  stop across a restart of Klipper's service and across a power cycle of the printer, since it
  has its own supply. Any board stopped for another reason leaves it to you; never at a start a
  `FIRMWARE_RESTART` or a `RESTART` made ([`limitations.md`](docs/limitations.md), "Several
  units").

- **A unit still stopped from an earlier shutdown** (2026-10-17). After an emergency stop, or a
  Klipper shutdown when one unit was lost, every unit stays stopped until it is reset: a unit has
  its own power, so a restart of Klipper's service or of the printer does not clear it, and
  Klipper stops again at start. `FIRMWARE_RESTART` resets it ([`limitations.md`](docs/limitations.md),
  "Several units"). At that start the unit's identifier query went unanswered and added a second
  error beside Klipper's `MCU shutdown`. The unit now answers it after a shutdown too, as it
  answers `ace2k_version` (an image change), and a query left unanswered is a warning in the log;
  with `uid:` set, the error says the identifier was not read.

- **The park: a load to the parking point with no tag search** (2026-10-17, host interface 15).
  A lane whose filament shares a tube with another lane's (a splitter before the head) must not
  run the tag search, which can reach the splitter; the grip was the only load without it, and
  its 40 mm left the filament where a roll back of 100 mm takes it out of the unit. The new
  command `ace2k_feed_lane_park` (`feed.set_lane_park`) makes a lane's next automatic load pull to
  the parking point, read the tag on the way and stop there ([`protocol.md`](docs/protocol.md),
  "The park"). It takes the grip's place on the unit and is cleared the same ways. Needs this
  release's image; `feed.has_park` says whether the unit has it.

- **A unit's `uid:`** (2026-10-17). The `serial:` of a unit's `[mcu]` names the USB adapter in its
  cable, not the unit: with two cables swapped, `[ace2k a]` drove unit b, so its lanes fed other
  heads and its encoder calibration went to the wrong unit. The new optional key `uid:` holds the
  unit's identifier, which `ACEK_STATUS` now prints on the line after the version. At every
  connect the host compares the two, and another unit stops Klipper with a message that says the
  cables may be swapped. Without `uid:` nothing changes ([`features.md`](docs/features.md),
  "Several units").

- **The host interface is stable** (2026-10-17). [`protocol.md`](docs/protocol.md), "What a
  caller can rely on", now says what a host module may depend on: the objects, the calls, the
  status fields and the events of the "Host interface" section. A new `API_VERSION` only adds. A
  change that would break a caller is made only when nothing else will do, is announced one release
  ahead with its replacement already working, and comes in a minor version with a **Breaking**
  line. A caller requires a minimum `API_VERSION`, never an exact one, and ignores what it does not
  know.

- **The flasher proves a new image itself; the window is 180 s again** (2026-10-16). A newly
  written image goes back to recovery if no host talks to it within its window, and only Klipper
  talked to it: Klipper had to be started within the window, which went wrong more than once.
  Now `ace2k_flash.py`, right after the write, asks the new image for its version as Klipper does
  at connect (its dictionary, then `ace2k_version`, at 250000 baud). That answer is the image's
  link proof, so it stays whatever Klipper does next, and units flashed one after another are
  each proven by their own flash. An image that does not answer is a failure the tool names. The
  window went back from 600 s to 180 s: it only guards an image that cannot talk, now that the
  flasher proves the others ([`docs/flashing.md`](docs/flashing.md), step 8).

- **A new active lane starts the feed-forward afresh** (2026-10-16). The feed-forward keeps its
  account per extruder: what it still has to take back of a long retraction, and the extruder's
  planned moves. When `ACEK_LANE_ACTIVE` (or a host module's `hand_over`) chose another lane during
  a long retraction, the new lane could be pulled back, or fed, for the old lane's filament. Now a
  change of the active lane, and a `release`, make every unit's feed-forward forget that account
  and the moves planned so far. The new lane gets doses only for the moves planned after the
  change ([`rules.md`](docs/rules.md), the feed-forward's rule 5).

- **A rollback with a free tip goes on through every catch** (2026-10-16). A thick tip can catch
  the tube twice on its way back. The unit forgave the first catch only, and the second ended the
  rollback `stuck`. A rollback started with `free_tip` (host interface 14, `ACEK_ROLLBACK FREE=1`)
  now forgives every catch, as long as the filament came back at least `snag_free_mm` (50 mm)
  past the last one. A tip held from the start, as by an extruder's gear, still ends it `stuck`
  at once, and an unload never reads the flag. The image says so with the constant
  `ACE2K_FEED_FREE_ROLLBACK`; with an older image, `ACEK_ROLLBACK FREE=1` is refused and nothing
  moves ([`protocol.md`](docs/protocol.md), "The free tip").

- **Host interface 13: the calls a timer may make in place of three G-codes** (2026-10-16). A
  host module cannot run a G-code from a timer, so three of them were out of its reach.
  `feed.set_speed(lane, speed_mm_s)` does what `ACEK_SPEED` does. The dryer's `start(target_c,
  minutes)`, `stop()` and `clear()` do what `ACEK_DRY`, `ACEK_DRY_STOP` and `ACEK_DRY_CLEAR` do; a
  refusal raises `DryerRefused`, with the refusal's name. `Units.release(extruder, pause_wait_s)`
  undoes the choice of a shared extruder's active lane: it stops that extruder's followers, the
  active lane included, and leaves no lane active. The G-codes and the firmware are unchanged
  ([`protocol.md`](docs/protocol.md), "Host interface").

- **`ace2k_flash.py` finds a named unit** (2026-10-16). With `[ace2k <name>]` sections the tool
  read only `[ace2k]`, did not see the unit under Klipper, and refused (`held open`). It now reads
  every unit section and picks the one whose MCU uses `--port`, and its restore carries
  `UNIT=<name>`. Without `--port`, several units are listed with their ports, and nothing changes.

- **A tag the cache knows is not read as its UID alone** (2026-10-16). When every brand declined a
  MIFARE tag, its session ended `read` with the UID alone, even when the cache held that UID's
  record from an earlier read: the adapter of a printer then saw a tag with no material. A UID the
  cache knows was decoded before, so every brand declining now is noise. The session now ends
  unread, as an NTAG's refused pages do: the lane keeps the cached record, and the next move reads
  the tag again.

- **A freshly flashed image waits 600 s for the host, not 180 s** (2026-10-16). Several units on
  one printer are flashed with Klipper stopped, one after another, and Klipper must start before
  the first unit's window runs out; 600 s leaves room for four. An image no host ever talks to now
  takes ten minutes to fall back into recovery ([`docs/flashing.md`](docs/flashing.md)).

- New: `printer.ace2k.sensors_reports` counts the sensor reports the unit has sent since Klipper
  started. A macro can use it to tell a new reading of the switches from an old one.

- **The follow is on by default.** At Klipper's start, ace2k turns the follow on for every lane
  that has filament. An empty lane stays off. When you insert a filament later, its lane goes into
  the follow once it is free, after the unit's own load if it runs one. The console says
  `ace2k: lane N follow on (follow_default)`. Your commands win: after `ACEK_STOP`,
  `ACEK_ASSIST … OFF=1`, an unload or any other move you start on a lane, that lane stays off until
  you take the filament out and put it back. To run a move on a lane in the follow, stop it first
  with `ACEK_STOP LANE=n`. To keep the old behaviour, set `follow_default: False` in `[ace2k]`. See
  "The follow is on by default" in [`docs/features.md`](docs/features.md#the-follow-is-on-by-default).
- **A move you start on a lane in the follow pauses the follow.** `ACEK_FEED`, `ACEK_ROLLBACK`,
  `ACEK_LOAD`, a one-way `ACEK_ASSIST` and `ACEK_RFID_READ … MOVE=1` stop the follow, run, and turn
  it back on when the move ends well. The console says `ace2k: lane N follow paused for the feed`,
  then `ace2k: lane N follow on again`. After an error, `blocked` or a stop, the follow stays off.
  `ACEK_UNLOAD` stops it and does not turn it back on. Before, such a move was refused `busy`.
- **Several lanes on one extruder: one active lane.** When lanes name the same extruder, ace2k
  follows only the lane you choose with the new command `ACEK_LANE_ACTIVE LANE=n`. After a
  restart none of them is armed, and the console asks you to choose. The choice is not saved.
  `ACEK_STATUS` shows the active lane of each shared extruder.
- The host interface for other Klipper modules is now version 8 (`API_VERSION`). It adds the
  follow's default (`follow_default`, `set_lane_follow`, `default_move`), sends one start per lane
  at a time, and names who stopped a lane (`stop_by`). See "Host interface" in
  [`docs/protocol.md`](docs/protocol.md#host-interface).

- **The LEDs show that the unit is on when every lane is empty.** Before, an empty unit stayed
  dark after its start, and you could not tell whether it was on. Now all four LEDs flash together,
  briefly, every 10 seconds. The first flash comes 10 seconds after the last lane is emptied, or
  after the short sweep at start. When you insert a filament, the flashing stops at once and the
  lane shows its own state. A lane in error always shows its error. Drying does not change the
  flashing. The short sweep now plays once at start. Before, an empty unit played it again each
  time Klipper connected. See "What the lane LEDs mean" in
  [`docs/features.md`](docs/features.md#what-the-lane-leds-mean).

- **Breaking: every G-code command is now named `ACEK_*`.** The old `ACE_*` names are gone, with
  no alias. For example, `ACE_FEED` is now `ACEK_FEED`, `ACE_DRY` is `ACEK_DRY` and `ACE_STATUS`
  is `ACEK_STATUS`. This leaves the `ACE_*` names free for other ACE modules. Klipper stops when
  two modules use the same command name. Update your macros, your slicer's start and end G-code,
  and every line of `printer.cfg` that calls one of these commands. Klipper answers
  `Unknown command` to an old name and does nothing. The section `[ace2k]` and its settings do
  not change.
- The host interface for other Klipper modules is now version 7 (`API_VERSION`). It adds
  `rfid.start_read`, a tag read another module can call directly. See "Host interface" in
  [`docs/protocol.md`](docs/protocol.md#host-interface).
- `ace2k_flash.py` works while Klipper still runs the old host module. It first reads the list
  of commands Klipper knows. It sends `ACEK_RESTORE_STOCK` when the list has it, and
  `ACE_RESTORE_STOCK` when the list has only the old name.

- `ACEK_SPEED` now waits for the unit's answer. A speed the unit refuses is an error that names the
  reason: no move running, the lane is in error, or the speed is out of bounds. It used to report
  success on an idle lane, where the speed did nothing. The host does not connect to an older
  image that does not answer.
- A speed out of bounds is refused. It used to shut the unit down.
- The host asks the unit one question at a time. A question the unit does not answer fails with
  `ace2k: …: no answer from the unit; check the link (…)`. A start with no answer stops that
  lane, so no move is left running unseen.
- `ACEK_LOAD_SET` with no parameters prints the values in effect. It used to say "staged for
  SAVE_CONFIG" with nothing staged.
- `ACEK_FAN ON=0` with `SECONDS` is an error. `SECONDS` applies only with `ON=1`; it used to be
  ignored.
- `ACEK_RESTORE_STOCK` and `ACEK_CALIBRATION_SAVE` refuse with a G-code error, so a macro stops
  there. They used to print a console line and let the macro go on. `ace2k_flash.py` still
  recognises the refusal and changes nothing.
- The flashing guide has a new step: going back to ace2k after restoring the factory firmware.
- The burn-in logger and its report, the two scripts that recorded the long-run test, are no
  longer part of this repository, nor are their tests and `docs/logging.md`. What to attach to a
  problem report is now in the README ("Reporting a problem").

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
