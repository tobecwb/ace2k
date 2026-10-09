# Differences from the factory firmware

This page lists what behaves differently on a unit running ace2k than on one running the factory
firmware. The factory firmware is Anycubic's V1.1.31, the firmware the unit ships with.

Every row says what the factory firmware does, what ace2k does, and why it matters.

- The factory-firmware column is observed behaviour, measured on the unit.
- The ace2k column is documented in [`features.md`](features.md), [`commands.md`](commands.md),
  [`protocol.md`](protocol.md) and [`hardware.md`](hardware.md).

The unit is the same hardware either way, and you can undo the change. The factory firmware can be
written back over the wire ([`flashing.md`](flashing.md)).

## Link and integration

| Factory firmware | ace2k | Why |
|---|---|---|
| Speaks Anycubic's own serial protocol. A printer that knows it polls the unit and sends it commands. | The unit is a native Klipper MCU. Klipper talks to it directly, and a host module adds `ACE_*` G-codes ([`commands.md`](commands.md)). | Any Klipper printer can drive the unit. There is no protocol bridge in between. |
| The original protocol is the only way in. | The original protocol is not spoken, except what is needed to write the factory firmware back. | A printer that only knows the original protocol cannot use a unit running ace2k. |
| Settings that persist (names, calibration, thresholds) are written to the unit's flash. | Settings live in `printer.cfg`, and are sent to the unit at every connect. `SAVE_CONFIG` keeps them. | Tuning does not wear the flash, and the unit's configuration is in the same place as the rest of the printer's. |
| The unit tracks a host address that falls back to 0 after about 3.5 s without a valid frame. A lane that is assisting at that moment resets the whole unit. | No such timeout: the host link is Klipper's. A cycle the unit is running (a dry) does not depend on the host staying connected. | A slow host cannot reset a unit that is moving filament. |
| Several units can share one bus, each with its own address (up to four). | One unit per serial port. Chaining several units on one port is a non-goal ([decision 0002](decisions/0002-one-unit-per-serial-port.md)). | ace2k was built and tested on a single-unit setup. |
| Takes updates over its serial protocol (the firmware's own OTA commands). | `ace2k_flash` flashes the unit from the Klipper host, in both directions ([`flashing.md`](flashing.md)). `ACE_RESTORE_STOCK` puts the unit in recovery for the way back. | Going back to the factory firmware is one command, and the unit's factory calibration is kept. |

## Lanes

| Factory firmware | ace2k | Why |
|---|---|---|
| Feed and rollback are length-bounded moves. Speeds from 0 to 100 mm/s are accepted. | Feed, rollback, load and unload are length-bounded moves at 9–70 mm/s, the range the lane can hold. A move that gets no final event is stopped and reported. | A request the lane cannot do well is refused before anything moves. |
| The assist always runs at one fixed speed, whatever the host asks. | `assist_speed` is a setting (default 50 mm/s). `ACE_SPEED` changes the speed of a running move. | The point of an assist is to match the speed of the print head. |
| The assist works one way: forward while the head pulls. A reverse assist exists, but as a separate mode. | Three assists on the buffer's plunger: forward, back, and the **follow**. The follow does both in one start, for a head that pulls and pushes back within one phase, such as a printer forming the filament's tip ([`features.md`](features.md), "The assists"). | One start covers a whole print, including retractions. |
| A lane in the assist only reacts to the head: the filament is fed once it is pulled tight. | **Feed-forward**: with a lane mapped to an extruder, the host reads the moves Klipper has planned. The unit then feeds small doses ahead of the pull, and the follow corrects what is left ([`features.md`](features.md), "Feed-forward"). | Measured on the unit, the follow's corrections fell from 8.7 to 0.08 a minute. The plunger stays near rest. |
| No handling of a filament tip that catches at the entry of the plunger's tube. A feed that stops is judged by the slip check, and can end as a feed error. | **Tip snag**: the unit announces it (the `snag` notice), moves back once by a short, bounded distance and retries, instead of ending the move ([`ACE_SNAG_SET`](commands.md#ace_snag_set)). | A good filament is not written off because its tip caught. |
| The filament encoder's scale is a fixed constant. | Each lane has its own scale. `ACE_CALIBRATE_ENCODER LANE=n AUTO=1` measures it against the extruder the lane feeds, in about two minutes, and stages it for `SAVE_CONFIG` ([`features.md`](features.md), "Calibrating the encoder"). | Every length the lane delivers, and every feed-forward dose, is measured on the encoder. A wrong scale shows as steady corrections in one direction. |
| A filament inserted at a bay is pulled to a fixed parking point by the unit itself. | The automatic load does the same, to `load_park_mm` (default 300 mm), and searches past it for the spool's tag. A `grip` turns the next load into a short pull, when the printer's head still holds the old piece ([`features.md`](features.md), "The grip"). | The parking distance is the path measured on the unit. A load during a print does not run into the old piece. |
| A feed against an obstacle keeps pushing until the slip check trips, after about 123 mm. | A feed or a load that meets an obstacle stops without an error and waits where it stopped (`blocked`). Nothing needs clearing. | The filament is not pushed at full force against a tube that holds it. |
| Errors: the feed, assist and motor errors are reported. The stuck and tangled kinds are computed but never shown. A stop does not clear any error. | Stuck, tangled, motor stalled, timeout, assist overrun and unload incomplete are all reported as lane states and events, with the motor's and the filament's travel. `ACE_CLEAR` leaves an error. `ACE_STOP` stops the motor and never clears an error. | You see what happened, and you can recover without a power cycle. |
| The slip check trips after about 123 mm of motor travel, with up to 111 mm of slip allowed (the defaults). | The supervisors check every 50 mm, allow 15 mm, and treat a motor that turned 20 mm with no filament movement as a stall (all settable). | A jam is noticed within centimetres, not decimetres. |
| A filament runs out during an assist. The unit does not act on it (the host can poll the sensor). It keeps assisting an empty lane until a 4-second timer returns it to ready. | A runout is an event (`runout`). In the follow, the lane goes on feeding the last piece (the **tail**), so the head prints with what is left in the tube. It ends with `tail_out` ([`features.md`](features.md), "The tail"). | The host and the printer can act on it: pause, swap, load the next spool. |
| Faults are error states in the lane's status. | A jam, slip, tangle or runout is a lane state and an event, and **never a Klipper shutdown**. A malformed configuration still is. | A filament problem does not kill a print. A macro decides what happens next. |

## Dryer

| Factory firmware | ace2k | Why |
|---|---|---|
| The host starts a dry with a target and a duration, in a set range (15–65 °C). | `ACE_DRY TEMP=15..65 DURATION=1 minute..24 h`: the same temperature range, with a hard 24 hour cap. | The factory firmware has no maximum duration. |
| The exhaust flaps open and close at the start of a cycle and at every stop. They open once mid-cycle to vent, when the heater outlets reach the target and the chamber has settled or two hours have passed. At the stop, the fans are cut before the flaps move. | Closes the flaps and runs both fans. Opens the flaps once the chamber is warm (near the target, or as warm as the heater can make it), and keeps them open. At the end, the fans cool the heater before they stop and the flaps close. | A chamber is not sealed while still hot, and the vent follows the chamber's measured warmth. |
| No interlock between the fans and the heater: a host command can stop the fans while the heater is on. | **The fan rule**: the heater is only driven with both fans on. The fans never stop while an outlet reads above 45 °C. That holds after a fault, a stop, a Klipper shutdown or a reboot alike. | Heating without airflow overheats the heater modules. |
| Protections: a heater that does not heat stops after five minutes below 30 °C; a cross-thermistor slope check; a sensor out of its valid window for five samples latches a sensor fault in seconds; the 115 °C hardware cutout; a 90 % duty cap and a drive ceiling of target + 8 °C. Every fault latches until the unit is power-cycled. No check of the fans, and no chamber ceiling. | The same duty cap and drive ceiling. Plus a gate that checks every 10 ms that: both fans are on; both outlet sensors are valid and below 85 °C; the mains is present and plausible; the cutout is clear. Plus the dryer's own checks: a heater that does not heat; a temperature rising faster than the heater explains (the stand-in for a stopped fan); outlets far apart; the chamber 10 °C above the target or above 80 °C; a silent sensor; lost mains; fans that do not come on. | A direct check catches a stopped fan or lost mains, instead of waiting for the heater's temperature. The chamber has its own ceiling. |
| After a dryer fault the unit stays in it. A new dry is acknowledged and silently ignored. | `ACE_DRY_CLEAR` leaves a fault once the heater is cool. `ACE_DRY` is refused, with the reason. A tripped 115 °C thermal cutout still needs a power cycle. | You are told why, and a recoverable fault is recoverable. |
| Debug commands set a raw heater target or duty, with no gate and no timer. | No such command. The only way to heat is a dry cycle. | The gate cannot be bypassed from G-code. |
| No record of past cycles. | The unit keeps a log in its own config page: cycles, faults, hours of heating and the last eight faults ([`ACE_DRYER_LOG`](commands.md#ace_dryer_log)). | You can see faults after the fact. |
| The cycle runs to its own timer. | The cycle runs to its end with no host attached. A reset **never resumes** a cycle: it is logged as interrupted. | A dry does not restart by itself after a power glitch. |
| No rule about what the unit does with no printer attached. | When you insert a filament with no printer connected, a hotter target is lowered to 45 °C, safe for PLA. | A forgotten spool is not baked. |
| Fan and flap commands are low-level, and can leave a flap coil energised. | `ACE_FAN` runs both fans for at most 600 s. `ACE_FLAP` pulses a flap for a fixed 200 ms. Both respect the dryer's ownership of the fans and flaps. | A flap actuator cannot be left energised. |

## Spool tags

| Factory firmware | ace2k | Why |
|---|---|---|
| Reads **only Anycubic's own NTAG tags**. A Bambu Lab MIFARE tag fails the read, and so does any other format. | Reads and decodes **Anycubic** (NTAG213), **Bambu Lab** and **Snapmaker** (MIFARE Classic 1K) tags, each checked against a real tag. Elegoo (NTAG213) and Creality (MIFARE Classic 1K) decoders exist, but are **off** until they have been read on a real tag. A tag no brand recognises still counts, by its UID. | A mixed shelf of spools works. |
| The reader runs in the unit and decides what the tag says. | The unit's firmware does the reading (the transport). The host decodes it. A new brand is a host module and a config section, with no reflash ([`features.md`](features.md), [Spool tags](features.md#spool-tags)). | Support for a new brand ships without touching the unit. |
| Two lanes share an antenna, and a conflict is reported as a collision. | A tag is given to a lane only when that lane's motion explains it. A read without motion that cannot tell is answered *ambiguous*. | You never get one lane's spool on another lane. |
| Does not write tags. | Only reads tags; **nothing is ever written to one**. | A bad G-code cannot damage a spool's tag. |

A Snapmaker spool's tag sits next to the hub, out of the bay antenna's reach. It reads only when you
take the tag out and hold it at the antenna. No firmware can fix this: it comes from where the tag
sits. Its record's RSA signature is not checked in this release.

## Safety and the unit itself

| Factory firmware | ace2k | Why |
|---|---|---|
| The independent watchdog is never enabled. A hung application stays hung. | The application feeds an independent watchdog. A hang resets the unit into the bootloader's recovery, which also accepts a full reflash. | A hung unit is recoverable over the wire. |
| No self-test or health report. | `ACE_HEALTH` reports a self-test, the latched faults and the cause of the last reset. | A broken sensor shows as a failing check, not a mystery. |
| A host command rewrites the factory calibration (the insert sensors' thresholds). A bad checksum falls back to defaults without saying so. | ace2k **reads** the factory calibration at every boot: the primary copy, then the backup, then built-in defaults. It says which one it used. It **never writes it**. A `printer.cfg` value overrides it per lane. | A calibration that took a lot of work to measure cannot be lost in a tuning session. |
| Commands that write the unit's flash exist (slot status, material names, the calibration). | None of them exists. The only flash ace2k writes is its own config page (the dryer log). The factory calibration pages are only read. | Tuning cannot overwrite what the unit shipped with. |
| A malformed frame can run past the receive buffer. | The link is Klipper's, with its own bounds, framing and checks. | A bad frame cannot corrupt the unit's state. |
| The unit answers only when polled. | The unit is a normal Klipper MCU. It reports on a schedule, and sends events without being asked. | Faults reach the host as they happen. |

## Indicators

| Factory firmware | ace2k | Why |
|---|---|---|
| All four lane LEDs are on steady when idle. A command can flash them in a pattern. | A steady LED shows a state; a blinking one asks you to look. An empty lane is off, a loaded lane is on steady, a moving lane blinks at 1 Hz, and a lane in error blinks at 5 Hz. Every lane blinks on one shared clock. An empty unit plays a short intro at boot (1 → 4 → 1 twice). | The LEDs tell you the lane's state at a glance ([`features.md`](features.md), "Lane LEDs"). |

## Not in ace2k (yet)

This is what the factory firmware has, or the original protocol offers, that ace2k does not. The
list of what may come next is in [`limitations.md#planned`](limitations.md#planned).

- **Automatic rotation of filament in the lanes** (the factory firmware's `auto_roll`, shipped
  off). Not implemented. The factory version walks filament out of lanes that are not threaded.
  That is why it is off by default.
- **Fan speed control.** The factory firmware has PWM steps. But its command, in those steps,
  drives both fans whatever fan is selected, and speed 0 does not stop them. ace2k runs both fans
  on or off, never one alone. The fans have no tachometer, so a stopped fan is only seen in the
  temperatures.
- **The original protocol**, and with it a printer that only speaks it. The unit is a Klipper MCU.
- **Several units on one port.** The factory firmware supports up to four on one bus.
- **Printer-side integration beyond Klipper.** Snapmaker U1 support is the separate
  [`ace2k-u1`](https://github.com/tobecwb/ace2k-u1) project.
- **Writing tags.** Neither firmware does it. It is planned.
- **The remaining-filament estimate.** The factory firmware reports a remainder read from the tag.
  It is planned.
- **Elegoo and Creality tags.** They are off until a tag of each has been read on a real unit.
- **Raw heater, output or LED access** from the host. This is by design: the heater has one gate,
  and it belongs to the dryer.
