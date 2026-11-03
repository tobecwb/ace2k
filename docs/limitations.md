# Known limitations

This page lists what you can run into today, and what to do about it.
What may come next is in the [roadmap](roadmap.md).

It is the whole list. If something is not here, it either works as documented or is not known to
us. The Snapmaker U1 items belong to the [`ace2k-u1`](https://github.com/tobecwb/ace2k-u1)
adapter. Its [install guide](https://github.com/tobecwb/ace2k-u1/blob/main/docs/install.md) covers
them in more detail.

## What has not been tested yet

- **One long-run test.** The long-run print test (burn-in) ran on the release candidate: two
  prints, about 9 hours, with all four lanes of one unit in use. The changes since then were tested
  on the bench, in shorter runs. Stay near the printer for the first prints.
- **Mains: 127 V / 60 Hz only.** Everything was tested on 127 V at 60 Hz. 220–240 V and 50 Hz have
  never been tried, and the dryer switches mains power. The firmware accepts both frequencies, but
  that comes from the design, not from a measurement. Do not leave a drying cycle unattended. If
  your mains is 220–240 V / 50 Hz, expect to be the first to try it.
- **The humidity mode heats with nobody there.** With `unattended_drying: True`, the unit starts a
  drying cycle on its own whenever its box stays humid, even with the printer off. The heater runs
  on mains power, and its protections have never been triggered on purpose on a real unit. Turn
  the mode on only with the unit in a ventilated place, with nothing that burns next to it, and
  nothing on the lid or over the vents ([README](../README.md#read-this-first)).

## Several units

- **A lost USB link to any unit shuts Klipper down.** Each unit is an MCU, and Klipper stops when
  one of its MCUs is lost. With several units, losing any one of them stops the printer. Power
  each unit and its cable so that this does not happen in a print.
- **A warning `configured for 120Mhz but running at 0Mhz` can appear when Klipper's service
  restarts.** A restart of the service does not reset the units: they keep running and sending
  their reports. At the connect, Klipper asks each unit for its clock a few times. On the
  half-duplex link those questions and answers can meet the unit's reports, and one early clock
  sample comes back spoiled. Klipper then warns once, and its estimate of that unit's clock takes
  about 8 to 10 minutes to settle at 120 MHz. Nothing fails meanwhile: no ace2k command depends on
  the unit's exact time. Only the timestamps of the temperature and humidity readings are a little
  off during those minutes. A `FIRMWARE_RESTART` resets the units and avoids it.
- **A unit keeps a Klipper shutdown until it is reset.** After an emergency stop (`M112`), or
  once Klipper shut down because one unit was lost, every unit stays stopped. A unit has its own
  power supply, so restarting Klipper's service, or turning the printer off and on, does not
  clear it. ace2k handles the common case itself, at a start of Klipper's process (the service
  started, or the printer powered on). When the first stop of that start is a board reporting
  that it was already stopped (`Command request`, the stop such a shutdown leaves), ace2k looks
  at every MCU three seconds later, the printer's own boards included. If Klipper is not ready
  and every stopped MCU reports `Command request`, ace2k sends one `FIRMWARE_RESTART`, which
  resets every MCU, and says so in `klippy.log`. A first stop of the host's own (a lost link: a
  fault of this start), a board stopped for any other reason (a fault), or a reason that cannot be
  read leaves everything to you, with a line in `klippy.log` naming the boards. Never at a start
  that a `FIRMWARE_RESTART` or a `RESTART` made, so it never tries twice, and an emergency stop
  with the printer running stays yours, as in Klipper. A unit can also be found stopped before it
  sends its stop: Klipper then ends in `MCU error during connect` with `Can not update MCU
  'ace2k_a' config as it is shutdown`. At a start of Klipper's process that is a unit stopped
  from before, so ace2k makes the same check, counts that unit as stopped by `Command request`
  (it sent no reason), and sends the one `FIRMWARE_RESTART`; `klippy.log` says the unit was
  found at connect. The same error on a board that is not an ace2k unit does nothing. If Klipper
  still stops, read the reason in `klippy.log`, then send `FIRMWARE_RESTART` yourself.
- **The sensors and the units come in a set order.** Put every `[ace2k <name>]` section before the
  `[temperature_sensor]` sections of type `ace2k` in `printer.cfg`. A sensor placed before a unit
  section stops Klipper at start. With several units each sensor needs its `unit:`.
- **A unit on the original firmware cannot be mixed in.** Every unit of the printer runs ace2k.
  A unit left on the factory firmware is not driven, and not supported beside the others.
- **Units cannot be chained.** Each unit needs its own USB connection. Units chained on one cable
  are not supported ([decision 0002](decisions/0002-one-unit-per-serial-port.md)).
- **The lane of a shared extruder is chosen by hand.** Nothing switches the active lane during a
  print ([roadmap](roadmap.md)).

## Changing a spool on the Snapmaker U1

- **A new spool inserted while the head still holds a piece of the old filament stops short.** By
  design, the unit pulls the new filament in until its gear holds it (40 mm by default) and the
  load ends `loaded`
  ([the grip](features.md#inserting-a-new-spool-while-the-head-still-holds-the-old-filament)).
  In practice it stops at about 10 mm, and the load ends `blocked`: it stopped against an obstacle
  and waits there. The new filament then
  waits. The U1's own runout handling brings it in once the old piece is out. If the end of the old
  piece sits right at the gear, the new filament never reaches the gear, and the U1's first load
  of that head fails. Resume the print, or load again: the second try works.
- **After a Klipper restart, the adapter guesses what the head holds.** Say a new filament waits
  in the bay, behind an old piece that is still in the head, when Klipper restarts. The adapter
  then takes the old piece for the lane's own filament. A wrong guess ends in a lane error and the
  print pauses, which is the safe side. Fix the cause. Then run `ACEK_CLEAR LANE=n`
  ([`ACEK_CLEAR`](commands.md#acek_clear)), or resume from the U1.
- **A new filament pushed against the old one too early reads as a tangle.** Say you push a new
  filament against the old one before the end of the old one has left the bay's sensor. The bay
  then never reads empty, so ace2k never starts feeding the last piece (the tail). The follow takes
  the pushed filament for a tangle, and the print pauses. Before you insert a new filament, wait
  for the console line that says the end is out of the unit. After a pause, pull the new filament
  back and push it in again.
- **A bay that runs empty in the middle of a print.** Nobody has measured what the U1 does when a
  bay runs empty mid-print: whether it clears the head by itself, and what follows. The adapter's
  install guide describes its own side:
  [When a spool runs out during a print](https://github.com/tobecwb/ace2k-u1/blob/main/docs/install.md#when-a-spool-runs-out-during-a-print).
  Watch the first such runout yourself.

## Moving filament

- **A long unload may end marked `stuck`.** A long unload can pull the end of the filament past
  the motor. The lane can then end in the error `stuck`, even though the eject worked and the
  filament is out of the bay. Clear it with [`ACEK_CLEAR`](commands.md#acek_clear) `LANE=n`. The
  lane then accepts a start again.
- **The tip of the filament catches at the bay's entry.** The unit tolerates one such catch per
  move. Two catches in one move end as `stuck`. A crushed or kinked tube reads as a tangle. Free the
  filament or straighten the tube, then run [`ACEK_CLEAR`](commands.md#acek_clear). The duty guard
  of the catch handling is off by default ([`ACEK_SNAG_SET`](commands.md#acek_snag_set)).

## Drying

- **The outlet runs hot.** Outlet air reaches about the target plus 7 °C. That is near where PLA
  softens. For PLA, choose a target well below its limit, and check the first cycles.
- **The flap position is not read back.** The firmware pulses a flap open or closed, and cannot
  tell whether it moved. If a flap sticks, drying will not report it.
- **A spool put on its holder without its filament inserted is not seen.** When you insert a
  filament with no printer connected, the unit lowers a hotter target to 45 °C (safe for PLA). A
  spool put on its holder without inserting the filament is not seen, so that protection does not
  apply.
- **An event can appear twice after a restart.** A dryer event may be logged twice after a host
  restart or a second reset. More than seven unacknowledged events are lost. This does not affect
  drying.
- **Mains-signal rejections.** The unit watches the mains through its zero-cross input, and counts
  the signals it rejects there (`rejects`, in the unit's mains state). That counter climbs while
  the unit dries, and also while it idles. Its cause is unknown, and it has no effect on drying.
- **The dryer's protections were tested in software only.** Automated tests cover a stopped fan
  and a loose temperature sensor, but these were never provoked on a real unit. The fans have no
  speed sensor. The unit infers a stopped fan from a temperature that rises faster than the heater
  can explain. The unit's own 115 °C thermal cutout stays in place as the last line.

## Tags

- **Elegoo and Creality are off.** Both formats ship disabled. No spool with such a tag was
  available, so neither has been read from a real tag. You can enable them at your own risk in
  `ace2k_tags.cfg`. A dump from a real tag ([`ACEK_RFID_READ`](commands.md#acek_rfid_read)
  `LANE=n DUMP=1`) helps turn them on by default.
- **A Snapmaker spool's tag is out of the antenna's reach.** The tag sits next to the hub. It reads
  only when you take the tag out and hold it at the bay's antenna. This comes from where the tag
  sits; no setting changes it.
- **A firmware restart forgets the tags already read.** A lane reads its tag again on its next
  move, or on [`ACEK_RFID_READ`](commands.md#acek_rfid_read) with `MOVE=1`. The cached records in
  `ace2k_tags_cache.json` are kept. Until then, a lane whose bay stayed full keeps its last read,
  marked `restored` ([`features.md`](features.md#spool-tags), "Tags across a restart"). A spool
  swapped while the printer was off keeps the old read until its tag is read again.
- **The U1 can undo a colour set from a tag.** On the Snapmaker U1, the adapter can set only a
  head's colour from a tag. The U1's own reader can then write its own value back over it.
