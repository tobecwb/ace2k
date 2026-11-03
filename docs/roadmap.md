# Roadmap

What may come next in ace2k. No dates are promised. What does not work today is in
[Known limitations](limitations.md).

- Virtual tools for several units: the G-code's `T0`…`T7` mapped to a unit and a lane, and a lane
  swapped inside a head during a print. Today you choose the lane of a shared extruder by hand with
  [`ACEK_LANE_ACTIVE`](commands.md#acek_lane_active).
- The `ace2k-u1` web page showing every unit. It follows the first unit today.
- Chaining units on one port is not planned
  ([decision 0002](decisions/0002-one-unit-per-serial-port.md)).
- A remaining-filament estimate per spool.
- Rotating the spool while drying (rotisserie).
- Writing tags.
- Variable fan speed (today the fans are on or off only).
- An automatic baseline for the insert sensor.
- A fixed address per unit.
- A per-lane odometer.
