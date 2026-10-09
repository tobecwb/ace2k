# Design rules

These are the safety rules the firmware is built to. They are numbered per subsystem. Inside a
subsystem's own files, code comments and tests cite them as "rule N": the dryer's rule 7 in
`heat/`, `dryer/` and `ace2k_board/gate.c`, the feed's rule 2 in `feed/` and `lane/`. Anywhere
else, the comment names the subsystem ("the dryer's rule 13"). The additional sub-lists (the
follow, the tip snag, feed-forward, the tail, encoder calibration, the vent and cool-down) are cited
by their name. Each rule has a host test that tries to violate it.

Terms used below:

- A **lane** is one of the four filament paths: a bay, its insert sensor and its motor.
- The **buffer** is the spring-loaded plunger the filament runs through between the unit and the
  print head. It gives three readings. *Full*: the plunger is at its pulled end, because the lane
  pushed too much filament. *Rest*: the spring is centred. *Taut*: the plunger is at its pushed
  end, because the head pulls the filament tight. The full switches of the four lanes share one
  input: that is the **shared full** reading. **This lane's full** (or the lane's own full) is the
  shared full while this lane's plunger is neither at rest nor taut.
- The **comparator** compares the motor's travel with the filament's travel on the lane's encoder.
  A **trip** is the comparator finding that the filament does not follow the motor.
- The **follow** is the assist in both directions. It keeps the plunger at rest while the head
  prints: a forward **burst** when the buffer reads taut, a **take-up** (a short move back) when
  it reads full.
- The **feed-forward** works on top of the follow. The host reads the moves Klipper has planned
  for the extruder, and the lane feeds that filament in small **doses** just before the head pulls.
- The **tail** is the last piece of filament after the spool runs out. The lane feeds it to the
  head until its end has passed the motor (the **tail-out**).
- The **parking point** is where the automatic load stops the filament inside the unit.
- A **lease** is a short window, renewed by the dryer, inside which `heat` may fire the heater.

## Core: flash, link proof, bootloader entry

1. The only flash the firmware writes is the configuration page. The guard refuses any erase or
   program outside it. The refusal is a return value the caller handles, never a `shutdown`.
2. A proven image never resets itself for lack of a host. Once the record says "proven by this
   image", no host-absence timeout exists. (An image is proven once it has answered the host's
   `ace2k_version` over the link.)
3. An unproven image resets into the bootloader's recovery after the window. It resets only into
   recovery, through the watchdog, never through a system reset.
4. The proof is bound to the image. The record carries the image's version string. A different
   string means unproven, whatever the page says.
5. `ace2k_bootloader_enter` is the only path that stops feeding the watchdog. It consults the
   guard's veto list, and it is refused while any registered subsystem objects.
6. The RS-485 driver-enable pin is low whenever the transmitter is idle. It is high from just
   before the first byte until transmit-complete of the last, never longer.
7. The minimal image configures no pin but the host link's, the driver-enable and the LEDs. The
   pin table is the allow-list, and a test walks it.
8. The response to `ace2k_bootloader_enter` leaves the wire before the halt. So the host never has
   to guess whether the command landed.

## Read-only subsystems: sensors, environment, mains, readers, health

1. No pin outside the allow-list is configured. The allow-list test also asserts that these pins
   are absent: the heater gate, the cutout latch reset, the motor, fan and flap pins.
2. The reader reset pins are driven high only (the level the external pull-up gives anyway). A
   reader is reset through its command register. The pin is never pulsed.
3. Reading the cutout input never drives the latch reset. No image has a latch-reset path.
4. No flash write but the link proof. The factory calibration pages are read through plain memory
   reads, and the guard refuses their addresses to the erase and program routines.
5. Nothing here vetoes `ace2k_bootloader_enter`.
6. No sensor condition calls `shutdown()`. An invalid reading is a cleared `valid` bit and a
   health bit. A fault is state.
7. I2C and SPI transfers run in task context, never in a timer, because both busy-wait.
8. An image built without a subsystem's `CONFIG_ACE2K_*` option contains none of its code.

## Feed: the lanes' motors and their modes

1. Each motor pin enters the pin table with its measurement. The allow-list test keeps asserting
   that these are absent: the heater gate, the cutout latch reset, the fan pins, the flap pins and
   the reader field.
2. Every run of a motor is a bounded move of the lane: a length of at most 2 000 mm, and a
   deadline of 1.5 times the expected time plus a second. Assist bursts and the load's steps are
   such moves. No code path raises a run pin outside the lane's move function.
3. Duty stays between the measured floor and 100 %. The core refuses a lower value.
4. Direction never changes while the run pin is high. A move sets direction, then run, then the
   drive signal. A reversal is two moves with a stop between them.
5. A stop always stops. From every state of the lane and of the feed, the run pin reads 0 and the
   drive signal is at its stop level on that tick.
6. Nothing moves without a host, and nothing keeps moving without one. 3 s without a received
   byte, or a Klipper shutdown, stops every motor at once. A move, an assist or a load is refused
   while the link is not up.
7. No physical event calls `shutdown()`. A jam, a tangle, a stalled motor, a runout or a buffer
   that never fills is a lane state plus an event. The only `shutdown()` calls answer a malformed
   host command.
8. Leaving the error state is explicit: only `ace2k_feed_clear` does it. No motion command, no
   stop and no sensor edge clears an error. There is one exception: when the filament leaves the
   insert sensor, an error raised about the filament itself ends, with a runout.
9. A stop that arrives before a load's motion starts cancels the load.
10. `ace2k_bootloader_enter` is refused while any lane moves, and accepted when all are idle.
11. An image built without `CONFIG_ACE2K_MOTOR` contains no code that drives a motor. An image
    built with it but without `CONFIG_ACE2K_FEED` carries only the bench commands. An image with
    both has no bench command.

### The follow (additional rules)

1. No hunting. After a move of the follow ends, a move in the opposite direction starts only
   once the lane has read rest, or once `ACE2K_FEED_FOLLOW_FLIP_MS` (200 ms) has passed since that
   move ended, whichever comes first. A move in the same direction may start at once.
2. The forward safety stop stands. No forward burst starts while the shared full reading is set,
   and a running burst ends on it. Another lane's full never starts a take-up on a lane that is
   at rest or taut.
3. Every move is bounded: 100 mm forward, 15 mm back, chained only while the reading that started
   it persists. The comparator judges each move, as it judges the assists' moves.
4. It runs until the host stops it, or until a runout, an error, the link or a shutdown ends it.

### The tip snag (additional rules)

The tip snag is the tip of the filament catching at the entrance of the plunger's tube. These rules
are cited by name ("the tip snag's rule N"). Rule 6 amends the feed's statement that the unit
never retries a move on its own: a tolerance is a bounded, announced resume.

1. The forward tolerance is bounded. Once this lane's buffer reads full in a feed or in a load's
   pull, the motor turns at most `ACE2K_FEED_SNAG_FWD_UM` (initially 20 mm) more before a verdict.
   If the filament falls `ACE2K_FEED_SNAG_LAG_UM` (initially 5 mm) behind the motor in that
   stretch, the move ends `stuck` on that tick.
2. The duty guard, once set, ends the tolerance at once. During the tolerance, a duty more than
   the guard's points above the feed-forward duty of the lane's setpoint is `stuck`. The guard is
   settable at run time and off by default.
3. The plunger must come back. After each release manoeuvre, the lane waits at most
   `ACE2K_FEED_SNAG_REST_MS` (initially 500 ms) for its rest reading. If it is not at rest by
   then, the result is `stuck`, with the motor stopped.
4. The reverse tolerance needs free travel. A comparator trip toward the spool with the plunger
   taut is forgiven only when all of these hold: the mode is a rollback or an unload; the insert
   sensor reads present; the filament has come back at least `ACE2K_FEED_SNAG_FREE_UM` (initially
   50 mm) on the encoder since the mode started. Anything else stays `stuck` at once.
5. One tolerance per mode. A second full (forward) or a second forgivable trip (reverse) in the
   same mode is `stuck` at once.
6. Every tolerance is announced and bounded. Each one sends the `snag` notice before the move
   resumes. Every motion it adds is one bounded lane move. A stop, the link, a shutdown, a runout
   and the supervisors end it as they end any move.
7. Nothing else changes. The assists, a load's return and reacquire, the tail rule and the
   comparator's thresholds behave as before.
8. The tag search never ends in an error. In the search, every verdict of the tolerance that is
   `stuck` elsewhere is instead the `obstacle` return to the parking point.

### The feed-forward (additional rules)

1. The feed-forward only acts on a lane in follow. Disarming the follow ends it, and the base rate
   returns to 0.
2. A dose is a bounded move (`ff_chunk` at `ff_pulse`). The comparator judges it, and this lane's
   full or the shared full ends it, as for a burst.
3. The debt is capped at two doses, so a wrong rate never accumulates into a long feed.
4. Ambiguity moves nothing. When two lanes of the same extruder are armed in follow at once,
   neither gets a base rate, and a console line says so. Each keeps the plain follow.

### The tail (additional rules)

1. The tail always runs. A lane in the follow whose insert sensor falls enters the tail. No
   setting disables it.
2. The tail only feeds: bursts and doses forward, never a take-up. A take-up would pull the loose
   end of the filament back out of the motor's reach.
3. The tail is bounded. It ends at its tail-out, at `tail_max` (2 000 mm of motor, settable), on a
   stop, on the link or on a shutdown.

### Encoder calibration (host command, additional rules)

1. The command moves only the extruder: never X, Y or Z, never a tool change, never a heater.
2. Never during a print: it is refused while one is printing or paused.
3. Only the lane's own extruder, which must be the active one and hot enough to extrude.
4. An abort stops the extrusion at the end of the current 25 mm chunk. These cause an abort: this
   lane's full or the shared full, a lane error, the lane leaving the follow, or a shutdown.
5. Nothing it cannot defend is applied. Too few marks, too much scatter, or a scale outside the
   unit's window leaves the scale as it was.
6. Everything is restored in every ending: the lane's feed-forward, the report rates, and the
   follow (disarmed only if the command armed it).

## RFID: the readers and the tag search

1. No tag is ever written. Every frame to a tag leaves through one function. That function checks
   the frame's first byte against the read-only set and refuses anything else. The read-only set
   is REQA, WUPA, the anticollision and select cascade levels, HLTA, READ and the two
   authentication commands. No host command carries a raw frame.
2. The field is on only while a lane is watched, searched or read. When the last lane that needs
   it stops, the field is switched off and read back off. A reader whose field does not read back
   off fails its health bit and latches. A field on for longer than `ACE2K_RFID_FIELD_MAX_MS` with
   no lane moving is switched off regardless.
3. The search moves only through the feed's bounded moves, and inherits every motion rule of the
   feed. Its length is `rfid_search_mm`, accepted up to 1 000 mm and never above. A larger value
   is a host bug (`shutdown`).
4. The search goes past the parking point only while every other lane of the unit is idle. It
   yields, back to the parking point, the moment another lane is given a start or begins its
   automatic load.
5. The filament comes back to the parking point after every search that went past it, whatever
   ended the search. The exceptions are a stop (which only stops), an error of the lane's
   supervisors, a lost link and a shutdown.
6. A reader step never holds the task. The wait for a tag's answer runs on the chip's own timer
   and is polled from the 10 ms tick. The lane's speed loop and supervisors keep running on time
   while a read runs.
7. The firmware keeps a key only for the one authentication it was sent for. A key is never in a
   report, an event, a log or the configuration page.
8. A lane is never given a tag its motion does not explain. A tag that cannot be attributed is
   ignored, and a read on command answers `ambiguous` and changes nothing.
9. A lane that saw a tag is never reported `no_tag`. A search that saw a UID and failed to read it
   ends `pending`.
10. The reader reset stays high. No image writes it low.
11. An image built without `CONFIG_ACE2K_RFID_READ` contains no code that turns a field on.

## Dryer: the heater, the fans, the flaps and the cycle

1. Only `heat` drives the heater gate, through its ops. No other module includes the gate's board
   code. No image drives the cutout's latch reset: it is in no allow-list and no table.
2. No gate pulse without both fans commanded on and both fan pins reading high. `heat` checks this
   before every pulse, and `airflow` refuses to switch a fan off while `heat` holds a lease.
3. The gate fires only inside a live lease. The dryer grants `heat` a duty for at most
   `ACE2K_HEAT_LEASE_MAX_MS` (2 000 ms). The zero-cross interrupt fires nothing past the lease's
   end, whatever the tasks are doing.
4. Absolute limits, checked by `heat` before every pulse and independent of the dryer: duty at
   most 90 %; both outlet NTCs valid and below 85 degC; the mains present and plausible; the
   cutout input not asserted. A violation stops the gate in the same half-cycle and latches
   `heat` with its reason.
5. The gate is never left high. A gate still high 8 ms after a pulse, or high outside a pulse at a
   tick, latches `gate_stuck`. The gate is written low before it is configured, and floated at
   every end of heating.
6. A fault always stops the heat and always cools down. Every latch drops the lease at once, and
   the fans stay on until the cool-down rule (rule 7) releases them.
7. A fan never switches off while an outlet NTC reads above 45 degC. This holds in any dryer
   state, including idle, fault and after a boot. Fans switch on above 45 degC and may switch off
   below 42 degC. An invalid NTC counts as hot.
8. The cycle is bounded: at most 24 h, refused above; target 15 to 65 degC, refused outside; the
   drive target never above 73 degC.
9. A reset never resumes a cycle. After any reset the dryer is idle with the gate low, and a cycle
   running at the reset is logged `interrupted`.
10. A tripped cutout clears only on a power cycle. It is persisted, and cleared at boot only when
    the reset cause is power-on. `ace2k_dryer_clear` does not clear it, and neither does a
    firmware restart. The cutout input asserted at boot is a fault.
11. A Klipper shutdown stops the heat and keeps rule 7. The shutdown handler drops the gate, and
    the fan rule keeps running afterwards. A lost host link is not a shutdown: the cycle goes on.
12. `ace2k_bootloader_enter` is refused while `heat` holds a lease. It is allowed in cool-down and
    idle.
13. The flash is written only with the gate idle, never while a lease is live. So a page erase
    never stalls the zero-cross interrupt in the middle of firing.
14. A flap coil is energised only by a pulse of `ACE2K_FLAP_PULSE_MS` (at most 400 ms), and
    released after it. The two coils of one flap are never driven together. No command holds a
    coil.
15. The release image carries no direct heat command. `ace2k_heat_run` exists only in the bench
    image (`CONFIG_ACE2K_HEAT` without `CONFIG_ACE2K_DRYER`), and `make flag-check` proves it.

### The vent and the cool-down (additional rules)

These are cited by name ("the vent's rule N"). The dryer's rules 1 to 15 above stand unchanged.

1. The vent never closes the flaps during heating, except by the humidity guard. The guard moves
   a flap at most once per `ACE2K_DRYER_VENT_GUARD_MIN_MS` (initially 10 min).
2. A held event is never lost silently. An event leaves the ring only on the host's
   acknowledgement. An event that does not fit is counted in a number the host reads in every
   state report.
3. A target below the room temperature never latches a fault by itself. The chamber ceiling is
   measured from the higher of the target and the chamber temperature at the start. A rise of
   10 degC above that still latches the fault in 30 s, and 80 degC absolute still latches it.
4. The cool-down ends only on a reading that is not rising. A rise above
   `ACE2K_DRYER_COOL_RISE_MC` (0.2 degC, the NTC's noise) over the window counts as rising.
5. A command is answered for what happens. No reply says accepted for a flap pulse that may not
   run. No start or clear is accepted while the mains is not yet measured.
6. The feed's counters reset lands before any base is taken.
