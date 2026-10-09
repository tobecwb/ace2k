# 0010 — Factory calibration read at boot; `printer.cfg` overrides

Date: 2026-09-16 · Status: accepted

## Context
The insert sensors are analogue, and the factory calibrates them per unit: two millivolt values
per lane, kept in the unit's own flash pages below the application slot. ace2k never writes those
pages. Its own config page does not survive a reflash. So any copy the firmware kept would have to
be re-entered after every flash. A copy in `printer.cfg` follows the serial port, which is the
cable, not the unit: swap two cables and each unit runs the other's numbers.

## Decision
The firmware reads the factory pages at every boot. It tries the primary page's checksum, then the
backup's, then falls back to the built-in defaults. It applies the factory firmware's own rule for
a degenerate pair. It uses those bands unless `printer.cfg` names others for a lane.

The values in use and their source are reported (`ace2k_sensors_thresholds`, `ACE_STATUS`).
`ACE_CALIBRATION_SAVE` stages them for `SAVE_CONFIG`, when a user wants them in the file to edit.
The pages are read through plain memory reads. The flash allow-list refuses them to the write
routines, and a test proves it.

## Consequences
- The calibration travels with the unit.
- A unit whose pages are unreadable runs the defaults and says so.
- The public documentation states the page layout as a measured fact: the checksum reproduces on a
  real unit's page, and the pairs equal what the factory firmware reports.

## Alternatives rejected
- Copying the values into `printer.cfg` by hand: per-unit data in a file that follows the cable.
- Binding a `printer.cfg` copy to the device identifier, with a warning: it solves the swap, but
  keeps the copying.

Both were rejected on 2026-09-16 in favour of reading the source.
