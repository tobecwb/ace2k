# Known limitations

ace2k is a Release Candidate. This page lists what you can run into today, and what to do about it.
What may come next is in the [roadmap](roadmap.md).

It is the whole list. If something is not here, it either works as documented or is not known to
us. The Snapmaker U1 items belong to the [`ace2k-u1`](https://github.com/tobecwb/ace2k-u1)
adapter. Its [install guide](https://github.com/tobecwb/ace2k-u1/blob/main/docs/install.md) covers
them in more detail.

## What has not been tested yet

- **No long-run test yet.** There has been no long print with all four lanes in use. Short prints,
  loads, unloads and drying cycles have been run. Hours of continuous use have not. Expect rough
  edges, and stay near the printer for the first prints.
- **Mains: 127 V / 60 Hz only.** Everything was tested on 127 V at 60 Hz. 220–240 V and 50 Hz have
  never been tried, and the dryer switches mains power. The firmware accepts both frequencies, but
  that comes from the design, not from a measurement. Do not leave a drying cycle unattended. If
  your mains is 220–240 V / 50 Hz, expect to be the first to try it.

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
  print pauses, which is the safe side. Fix the cause. Then run `ACE_CLEAR LANE=n`
  ([`ACE_CLEAR`](commands.md#ace_clear)), or resume from the U1.
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
  filament is out of the bay. Clear it with [`ACE_CLEAR`](commands.md#ace_clear) `LANE=n`. The lane
  then accepts a start again.
- **The tip of the filament catches at the bay's entry.** The unit tolerates one such catch per
  move. Two catches in one move end as `stuck`. A crushed or kinked tube reads as a tangle. Free the
  filament or straighten the tube, then run [`ACE_CLEAR`](commands.md#ace_clear). The duty guard
  of the catch handling is off by default ([`ACE_SNAG_SET`](commands.md#ace_snag_set)).

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
  `ace2k_tags.cfg`. A dump from a real tag ([`ACE_RFID_READ`](commands.md#ace_rfid_read)
  `LANE=n DUMP=1`) helps turn them on by default.
- **A Snapmaker spool's tag is out of the antenna's reach.** The tag sits next to the hub. It reads
  only when you take the tag out and hold it at the bay's antenna. This comes from where the tag
  sits; no setting changes it.
- **A firmware restart forgets the tags already read.** A lane reads its tag again on its next
  move, or on [`ACE_RFID_READ`](commands.md#ace_rfid_read). The cached records in
  `ace2k_tags_cache.json` are kept.
- **The U1 can undo a colour set from a tag.** On the Snapmaker U1, the adapter can set only a
  head's colour from a tag. The U1's own reader can then write its own value back over it.
