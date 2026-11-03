# Protocol

ace2k speaks Klipper's MCU protocol. The host learns the commands and responses below from the
dictionary the MCU sends when it connects (`firmware/build/ace2k-<version>.dict`). This page is the
firmware's public API: any change here is a versioned, documented change.

## Conventions

- Commands are named `ace2k_<module>_<verb>` (`ace2k_feed_start`). When a command addresses an
  instance, `oid=%c` comes first. `ace2k_version` is the query for the whole tree, and the one
  command without the `<module>_<verb>` shape.
- A query's response is named `<command>_response`. The read-only queries are the exception: their
  answer is named for what it carries (`ace2k_health_state`, `ace2k_rfid_reader_state`,
  `ace2k_rfid_lane_state`, `ace2k_sensors_thresholds`).
- Periodic reports are named `ace2k_<module>_state` (the lane counters' report is
  `ace2k_lane_counters`). `ace2k_<module>_query rest_ticks=%u` starts them. `rest_ticks` is in
  clock ticks, as Klipper's own `query_analog_in` takes it; 0 stops them, and so does any period
  under 1 ms. A longer one is rounded to the nearest millisecond, half up. A report is not a query
  response: the host subscribes to it, and a lost frame raises no timeout ("Reports", below).
- Unsolicited events are named `ace2k_<module>_event`, with a `kind=%c` field that names the event.
  The two frames of an RFID read session are the exception: they are named for what they carry
  (`ace2k_rfid_tag`, `ace2k_rfid_data`; see [RFID sessions](#rfid-sessions), below).
- Every quantity carries its unit in the field name: `_ms`, `_um`, `_um_s`, `_mv`, `_mc`
  (millidegrees Celsius), `_pct`, `_pct10` (tenths of a percent), `hz10` (tenths of a hertz).
- Arrays travel as `%*s` byte strings. The four lanes are packed little-endian, lane 1 first:
  `uint16 × 4` for millivolts, `int32 × 4` and `uint32 × 4` for the counters, one byte per lane
  for a source. The device identifier is 12 bytes, in the order they lie in the part.
- A jam, a slip or a fault is an event and a state, never a Klipper `shutdown`. A malformed
  argument is a host bug, not a device condition, so it takes Klipper's own way out: a `shutdown`
  whose message names the command. That covers a lane or reader index out of range, and a band
  with `low_mv >= high_mv`. Klipper's parser keeps all 32 bits of every integer argument, even
  for `%c` and `%hu`. So a value too large for its field is out of range too: `lane=256` is a
  `shutdown`, never lane 0, and a flag (`free_tip`, `auto_load`, `move`, `result`, `clear`) takes
  only 0 or 1. The airflow commands and `ace2k_dryer_start` answer such a value with their `range`
  reason instead, as they answer every other value out of range.

## Dictionary — current

| Command | Response | Since |
|---|---|---|
| `ace2k_version` | `ace2k_version_response version=%s` — the ace2k release string. **The first answered `ace2k_version` is the proof of the host link.** An image that is flashed and never queried resets itself into the bootloader's recovery after 180 s (see below). `ace2k_flash.pyz` asks it right after it writes the image | v0.1.0 |
| `ace2k_linkproof_state` | `ace2k_linkproof_state_response proven=%c remaining_ms=%u` — `proven` is 1 once this image has been queried. `remaining_ms` is the time left in the window while it has not | v0.1.0 |
| `ace2k_link_query` | `ace2k_link_state deferred=%u rearmed=%u idle_us=%u` — counters since boot: how often the unit had to wait for the line before it transmitted, how often a wait was extended because bytes kept arriving, and the idle time it waits for. Read-only. These are the counters of the half-duplex turnaround ("Link", below) | v0.2.0 |
| `ace2k_link_gap_query` | `ace2k_link_gap_state max_ms=%u lost_ms=%u` — `max_ms`: the longest silence of the host, in milliseconds, since boot or the last `ace2k_link_gap_clear`: the longest interval between two valid frames, the same silence that takes the link for lost, dated at the same point (a frame is a valid one once Klipper has checked its CRC and its sequence; bytes alone are not). The first frame after boot ends no interval. An interval still running counts once a frame ends it, and the query is such a frame: the interval it ends is in the answer. A silence that ended a lost link never reads below `lost_ms`, even one longer than the clock's wrap (35.8 s). `lost_ms`: the silence after which the unit takes the link for lost (3000). Read-only. It measures the margin of that window: Klipper's clock query comes about once a second | v0.13.0 |
| `ace2k_link_gap_clear` | none. Sets `max_ms` to 0. The next frame still ends the interval that began before the clear, and that interval counts. The link state is untouched. The host sends it with `ace2k_health_clear` (`ACEK_HEALTH CLEAR=1`) | v0.13.0 |
| `ace2k_bootloader_enter` | `ace2k_bootloader_enter_response accepted=%c` — the response is sent first. When `accepted` is 1, the unit stops feeding its watchdog 50 ms later and resets into the bootloader's recovery mode (`flashing.md`). The command is refused (`accepted=0`) while a subsystem objects: the heater is active (a lease, its last half-cycle, or the gate reading high, even latched); the fans are required (a dryer cycle until its cool-down hands them back, or an outlet NTC that reads valid and is still at or above 42 °C while the 45 °C rule holds them: the bootloader drives no fans); or a lane is moving. Both outlet NTCs reading invalid is no veto by itself: the unit then accepts recovery, because a broken sensor must never make the unit impossible to reflash, although the recovery mode drives no fans. The unit asks the vetoes again, with interrupts masked, right before the reset. A veto that appeared in those 50 ms cancels the reset, and the unit keeps running: ask again | v0.1.0 |
| `ace2k_identify` | `ace2k_identify_response ms=%u` — the four lane LEDs blink together at 2 Hz for `ms` (10 000) ms, over every other pattern, then the normal pattern comes back. A second one starts the time again. Nothing else changes. Accepted after a Klipper shutdown too. The installer and the flasher (`--identify`) use it to show the user which unit is which | v0.13.0 |
| `ace2k_config_state` | `ace2k_config_state_response page=%c calibration_source=%c` — `page`: the state of the ace2k config page: 0 valid, 1 blank (every boot after a flash), 2 corrupt (also from a store that failed after erasing the page, since v0.13.0). `calibration_source`: where the insert bands came from at boot: 0 the built-in defaults, 1 the unit's factory page (`hardware.md`, "Factory calibration page") | v0.2.0 |
| `ace2k_sensors_query rest_ticks=%u` | periodic `ace2k_sensors_state switches=%hu insert_mv=%*s empty_mv=%*s aux_mv=%hu age_ms=%c` — `switches`: bits 0–3 *insert* (filament present) for lanes 1–4, bits 4–7 *rest*, bits 8–11 *pushed*, bit 12 *pulled* (any lane), bit 13 *cutout*; every bit is debounced. `insert_mv`, `empty_mv`: `uint16 × 4` little-endian, raw millivolts from the last ADC pass. `aux_mv`: the auxiliary input (PC1). `age_ms`: the age of that pass when the report is sent, capped at 255. The host asks for 10 Hz | v0.2.0 |
| — | `ace2k_sensors_event lane=%c kind=%c level=%c` on every debounced edge — `kind`: 0 insert, 1 rest, 2 pushed, 3 pulled, 4 cutout. `lane`: 0–3, or 255 for the two shared inputs (pulled, cutout). `level`: 1 = asserted (for *insert*: filament present). Since v0.13.0 an *insert* edge goes out only from a fresh ADC pass: while the scan is stopped no insert event is sent ("Stale insert sensors", below) | v0.2.0 |
| `ace2k_sensors_thresholds_set lane=%c low_mv=%hu high_mv=%hu` | none. Sets the insert band of one lane in millivolts, `low_mv < high_mv`, with source *config*. It takes effect from the next tick, and the debounced state is kept. The host sends one per `laneN_insert_mv` key of `printer.cfg`, and `ace2k_sensors_thresholds_reset` for every other lane, as **init** commands at every connect: they stay out of Klipper's config checksum, so a changed or removed override never resets the MCU (since v0.12.0; before, a config command, and a change restarted the MCU — on the U1 that restart fails). An image that takes every key this way names the dictionary constant `ACE2K_KEYS_AS_INIT` (declared beside the version, whatever subsystems it carries): the bands, `ace2k_lane_scale_set`, `ace2k_feed_thresholds_set` and `ace2k_rfid_search_set`. With an older image the host keeps them all config commands, sent only for a key, as before, so updating the host alone leaves the checksum as it was | v0.2.0 |
| `ace2k_sensors_thresholds_reset lane=%c` | none. The lane's band back to the one the unit started with, its factory page's or the default, with that source. Sent at every connect as an init command for a lane with no `laneN_insert_mv`. A lane above 3 is a host bug (`shutdown`) | v0.12.0 |
| `ace2k_sensors_thresholds_query` | `ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s` — one frame for the four lanes. `low_mv`, `high_mv`: `uint16 × 4` little-endian, the bands in use. `source`: one byte per lane: 0 default (600 / 1100 mV), 1 factory, 2 config | v0.2.0 |
| `ace2k_env_query rest_ticks=%u` | periodic `ace2k_env_state ptc_left_mc=%i ptc_right_mc=%i chamber_mc=%i chamber_rh_pct10=%hu vdda_mv=%hu valid=%c` — the two outlet NTCs and the chamber temperature in millidegrees Celsius, the chamber humidity in tenths of a percent, and the supply computed from the internal reference. `valid` bits: 1 `ntc_left`, 2 `ntc_right`, 4 `chamber`, 8 `vdda`. `chamber` is set while the last good reading is at most 5 s old (2 s up to v0.12.0), the bound the dryer uses too. A field whose bit is clear carries its last value or 0, never a sentinel. When the last ADC pass is more than 100 ms old, the two outlet NTCs and `vdda` read invalid, with 0. The 100 ms is an initial value and may change after bench tuning. The host asks for 1 Hz | v0.2.0 |
| `ace2k_lane_counters_query rest_ticks=%u` | periodic `ace2k_lane_counters encoder_um=%*s fg=%*s` — `encoder_um`: `int32 × 4` little-endian, the filament encoder in micrometres at the lane's scale (1234.2 µm per count until `ace2k_lane_scale_set` names another). It is signed, positive toward the printer (`hardware.md`, "Counters"), and saturates at ±2³¹. `fg`: `uint32 × 4` little-endian, the motor's tach pulses since boot or the last reset. The host asks for 1 Hz. During an automatic encoder calibration (`ACEK_CALIBRATE_ENCODER … AUTO=1`, since v0.10.0) it asks for 10 Hz, and for 1 Hz again once the calibration ends | v0.2.0 |
| `ace2k_lane_counters_reset lane=%c` | `ace2k_lane_counters_reset_response lane=%c accepted=%c` — `lane`: 0–3, or 255 for all four. `accepted` 1: the lane's encoder count is zeroed and its tach count re-based on the next tick. The re-base happens at once if a move starts before that tick. Since v0.6.0 it also happens before any feed mode takes its odometer base, so an assist or a load armed in the same tick counts from 0. `accepted` 0: nothing changed, because the lane (for 255, any lane) is moving or busy in a feed mode. The counters are the move's odometers, and the host first refuses `ACEK_COUNTERS_RESET` on its own view of the lanes. This is a query: a reset whose response was lost is sent again by the transport and may land twice. That gives two zeroes, nothing else | v0.2.0; the response since v0.3.0 |
| `ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c free_tip=%c` | `ace2k_feed_start_response lane=%c accepted=%c reason=%c` — `mode`: 0 feed, 1 rollback, 2 assist, 3 assist_back (the reverse assist), 4 unload, 5 load, 6 assist_both (the follow, since v0.8.0: [The follow](#the-follow), below). `length_um`: for a feed or a rollback, its length, 1 .. `ACE2K_LANE_MOVE_MAX_UM`, as **the filament's travel on the encoder** (under a spool the motor turns a few per cent more: `hardware.md`, "Counters"); for an unload, its budget (0: 2 000 mm); for a load, its parking distance (0: the configured one); ignored by the assists. `speed_um_s`: within `ACE2K_LANE_SPEED_MIN_UM_S` .. `ACE2K_LANE_SPEED_MAX_UM_S`; the assists need one, and a load's 0 means the configured one. `seq`: the host's sequence for this start, 1–255 ("The start's sequence", below). `free_tip`: 1 when nothing holds the filament's front tip — a push to the head, not a filament in an extruder — read by a feed: its buffer full is judged by the filament, in windows ("The tip snag", below); and by a rollback, on an image with the constant `ACE2K_FEED_FREE_ROLLBACK`: every catch past `free_um` from the last is forgiven, not the first only ("The free tip", below). 0 otherwise, and ignored by every other mode: a load from the bay has a free tip on its own, the read with motion never, an unload never (since v0.12.0). Above 1 is a host bug: `shutdown`. An image without it: see "An older image", below. `reason`: 0 ok; 1 `busy` (the lane is in a mode, or the lane core is moving; the lane core is the layer under the feed that runs each motor move); 2 `in_error`; 3 `no_filament` (no filament at the insert sensor, for a feed, an unload, an assist or a load; a rollback is accepted without one, because it brings back out a piece parked past the sensor); 4 `no_link`; 5 `bounds` (the length, the speed or the mode; since v0.13.0 also a start that does not fit the stop armed for it, "A stop armed for the next start", below); 6 `other_lane` (never returned by a start: it belongs to the read with motion, "The read with motion", below; since v0.4.0); 7 `no_move` (never returned by a start: it belongs to `ace2k_feed_speed_set`, below; since v0.12.0); 8 `following` (never returned by a start: it belongs to `ace2k_feed_speed_set`, below; since v0.13.0); 9 `sensors_stale` (every mode, the rollback too, and the read with motion's search: the insert sensors are stale because the ADC scan stopped, so the lane cannot judge its filament; since v0.13.0, "Stale insert sensors", below). A lane above 3, a mode above 6 or a `seq` above 255 is a host bug: `shutdown`. With the reader's field in the image, a load whose tag is not read by the parking point searches further and comes back before it ends `loaded` ("The load's search", below) | v0.3.0; mode 6 since v0.8.0; `free_tip` since v0.12.0 |
| `ace2k_feed_stop_when lane=%c held=%c at_rest=%c held_ms=%hu` | `ace2k_feed_stop_when_response lane=%c accepted=%c reason=%c` — arms a stop for the lane's **next** `ace2k_feed_start` ("A stop armed for the next start", below). `held` 1: for a rollback or an unload, the move also ends `held` (19) when the plunger stays taut while the filament moves less than `held_um` for the move's window, or sooner when the comparator trips with the plunger taut and the filament still; toward the head it is not judged, and a tangle stays `tangled`. `at_rest` 1: for any mode but the follow, the move also ends `at_rest` (20) when the plunger reads rest, in either direction. A start that does not fit its stop is refused `bounds` (since v0.13.0). Both 1: whichever comes first. Both 0: disarms. `held_ms` is this move's window: 50–2 000 ms, or 0 for the unit's own `held_ms` (`ace2k_feed_held_set`). It goes with the arming, so two moves on the unit never share a window, and the next start takes it with the arming. `accepted` 1: armed, `reason` 0. `accepted` 0: nothing armed. Either `held_ms` is not 0 and is outside 50–2 000 (`reason` 5 `bounds`, whatever the lane's mode), or the lane is not idle (`reason` is the start's code, 1 `busy` when the lane runs a mode, 2 `in_error`). Send it right before the start, and `held=0 at_rest=0 held_ms=0` on any path that arms and then does not start. A tick with the link down clears an arming no start took. A lane above 3, or `held` or `at_rest` above 1, is a host bug: `shutdown`. `ace2k_feed_start` keeps its format, so an older host never sends this and works unchanged. The `held_ms` field came before any release that carries the command, so there is only this format | v0.13.0 |
| `ace2k_feed_held_set held_um=%u held_ms=%hu` | none. The held stop's values. `held_um`: a filament that moves this much starts the window again, so it is not held; 1 500–20 000 µm. `held_ms`: how long the plunger must stay taut with the filament still, for an arming that carries `held_ms=0`; 50–2 000 ms. The firmware's own values are 3 000 µm and 550 ms, each an initial value for the bench (dictionary constants `ACE2K_FEED_HELD_UM` and `ACE2K_FEED_HELD_MS`, with `…_MIN` and `…_MAX`). The least `held_um` is more than one encoder count at every scale, and the default is at least two, so one count of jitter never restarts the window. A held tip ends `held` at any speed and any `held_ms`: a comparator trip that comes first, with the plunger taut and the filament still, ends the move `held` too. The friction floor: a filament dragged taut at a speed above `ceil(held_um / count) × count / held_ms` restarts the window in time, so it is never `held`. The floor is against **the filament's** speed, which trails the motor's under a spool. With the defaults it is 8.1 mm/s at the widest scale (6.7 mm/s at 1 234.2 µm a count), at least 5 % under the lane's least speed of 9 mm/s; the firmware checks this for its own values when it is built. A host that sets other values keeps the floor under the filament's speed in its slowest armed move. A value out of its bounds is a host bug (`shutdown`). Taken at any time, never `busy`. Read where it is used, so a change applies on the next tick of a window. RAM only: an MCU restart returns to the firmware's values | v0.13.0 |
| `ace2k_feed_stop lane=%c` | none; lane 255 = every lane. The motor stops whatever the mode. A moving or assisting lane goes idle with `stopped`. A load still in its settle is cancelled, with nothing ever moved. A lane in error stays in error (its motor stopped when the error latched): a stop never clears an error | v0.3.0 |
| `ace2k_feed_clear lane=%c` | none. Moves the lane from `error` to `idle`; a no-op in any other mode. A clear ends no mode, so the retry window ("The start's sequence") stays the one of the failure | v0.3.0 |
| `ace2k_feed_speed_set lane=%c speed_um_s=%u` | `ace2k_feed_speed_set_response lane=%c accepted=%c reason=%c` — a new speed for the lane's current mode. A feed, a rollback, an unload, a load and the two one-way assists take a speed; the follow does not. A running move of the command's own takes it, with its deadline computed again from now. A move the mode makes for itself keeps the speed it started at: a load's reacquire (backwards at the floor speed) and a tip snag's touch (since v0.13.0; before, any running move took it). The mode keeps the new speed for its next command move all the same: a one-way assist's next burst, a settling load's pull, a feed resumed after a touch. `accepted` 1: the mode took the speed. `reason`: the start's codes ("`ace2k_feed_start`", above) — 0 ok; 2 `in_error` (the lane is in error); 5 `bounds` (the speed, judged first, whatever the mode); 7 `no_move` (the lane is idle); 8 `following` (the lane is following, a burst, a take-up, a dose or the tail running or not: the follow keeps the speed it was armed with, and a new one is set by arming it again). A refused speed is kept for no later move. A lane above 3 is a host bug: `shutdown` | v0.3.0; the response since v0.12.0 (before it: none, silent on an idle lane or a lane in error, `shutdown` on a speed out of bounds); `following` since v0.13.0 (before it: taken, and the follow's later bursts and take-ups ran at it until it was armed again; a dose running at that instant ran at it too) |
| `ace2k_feed_thresholds_set slip_check_um=%u slip_allow_um=%u stall_check_um=%u` | none. The supervisors' thresholds ("Feed events", below), in effect from the next tick. The firmware's own values are 50 000 / 15 000 / 20 000 µm. The host sends it at every connect as an **init** command, with the keys of `printer.cfg` or these values, which it reads from the dictionary (`ACE2K_FEED_SLIP_CHECK_UM`, `ACE2K_FEED_SLIP_ALLOW_UM`, `ACE2K_FEED_STALL_CHECK_UM`, all three or none), so a change never resets the MCU (since v0.12.0, `ACE2K_KEYS_AS_INIT`; before, and with an older image, a config command sent only with a key). A default that breaks the limits below is named as the image's, not a key's. These are host bugs (`shutdown`), and the host refuses all of them before it sends: a zero; `slip_allow_um ≥ slip_check_um` (the partial trigger could never trip); a `stall_check_um` or `slip_check_um` at or above `ACE2K_FEED_ASSIST_BURST_UM`. The last limit exists because every forward assist burst takes the comparator's bases afresh, and chains into the next burst while the filament is taut. A window that long would never close, and a filament held during an assist would be ground on burst after burst | v0.3.0 |
| `ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c back_um=%u rest_ms=%u free_um=%u` | none. The tip snag's values ("Feed events", below). `fwd_um`: how far the motor runs past a full reading before the touch; also the free tip's window. `lag_um`: how far the filament may fall behind the motor during the watch before it is a jam; also the free tip's limit within a window. `duty_pct`: how far the duty may rise above the feed-forward duty of the loop's setpoint before it is a jam (0 = this guard off). `back_um`: the length of the touch. `rest_ms`: the wait for the plunger to come to rest. `free_um`: how far the filament must have come back in a reverse mode before a taut trip is forgiven. The firmware's own values are 20 000 / 5 000 µm, 0 %, 5 000 µm, 500 ms, 50 000 µm (dictionary constants `ACE2K_FEED_SNAG_*`, read by the host). The bounds: `fwd_um` 5 000–50 000; `lag_um` 1 500–20 000 (1 000–20 000 before v0.13.0) and under `fwd_um`; `duty_pct` 0–100; `back_um` 2 000–20 000; `rest_ms` 100–2 000; `free_um` 20 000–200 000. Since v0.13.0 they are in the dictionary too, named as the follow's are (`…_MIN`, `…_MAX`; `duty_pct` has only `ACE2K_FEED_SNAG_DUTY_PCT_MAX`). The least lag is more than one encoder count at every scale: the lag is measured from a point taken anywhere in a count, and the filament reads nothing until its next count, so a lag of one count or less would stop a filament that follows the motor exactly. This is an **init** command. The host sends all six at every connect: the `snag_*` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_load_set`. So a value set live never outlives the session. Since v0.13.0 the host checks the keys at connect against the dictionary's bounds (a config error), or against its own copy for an older image. A `snag_lag_mm` or an `ACEK_SNAG_SET LAG` from 1 mm up to 1.5 mm is raised to 1.5 mm with a warning, and the next version refuses it. The host sends the command again on every `ACEK_SNAG_SET`. Any value out of its bounds, or `lag_um ≥ fwd_um`, is a host bug (`shutdown`). The host rounds every value to the unit's resolution (µm, %, ms), and refuses all of these, and a non-finite value, before it sends. Taken at any time, never `busy`. Each value is read where it is used, so a change during a running tolerance mixes two sets, every value of both within its bounds. RAM only: an MCU restart returns to the firmware's values until the host's next connect sends its own | v0.7.0 |
| `ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u` | none. The follow's values ("The follow", below). `flip_ms`: after a move of the follow ends, the wait before a move the other way may start without the plunger reading rest first (rule 1). `take_um`: one take-up of the follow (the reverse assist keeps its own 15 mm). `tail_um`: the tail's bound, as motor travel since the tail began ("The tail", since v0.11.0). The firmware's own values are 200 ms, 15 000 µm and 2 000 000 µm, each marked as an initial value for the bench (dictionary constants `ACE2K_FEED_FOLLOW_FLIP_MS`, `ACE2K_FEED_FOLLOW_TAKE_UM` and `ACE2K_FEED_FOLLOW_TAIL_UM`, read by the host). The bounds: `flip_ms` 50–2 000, `take_um` 5 000–30 000, `tail_um` 200 000–2 000 000. They are compiled beside the defaults, and are in the dictionary too (`…_MIN`, `…_MAX`). This is an **init** command. On an image with the follow, the host sends all three at every connect: the `follow_*` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_snag_set`. It sends it again on every `ACEK_FOLLOW_SET`. A value out of its bounds is a host bug (`shutdown`). The host checks the keys against the dictionary's bounds at connect (a config error), and `ACEK_FOLLOW_SET`'s arguments before it sends (a G-code error), each rounded to the unit's resolution (ms, µm) first. Taken at any time, never `busy`. Each value is read where it is used: the next take-up, the next flip, the next tick of a tail. RAM only: an MCU restart returns to the firmware's values until the host's next connect sends its own | v0.8.0; `tail_um` since v0.11.0 |
| `ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u` | none. The feed-forward's values ("The feed-forward", below). `chunk_um`: one dose, and half the cap on the debt. `pulse_um_s`: the speed a forward dose is asked at, and the least speed of a reverse dose. Since v0.13.0 a forward dose runs whole at the floor speed, because no `chunk_um` the bounds allow is longer than the lane's landing (`ACE2K_LANE_LANDING_UM`), so this value no longer sets a forward dose's speed. A reverse dose has no landing ("The feed-forward", below). The firmware's own values are 3 000 µm and 15 000 µm/s, each marked as an initial value for the bench (dictionary constants `ACE2K_FEED_FF_CHUNK_UM` and `ACE2K_FEED_FF_PULSE_UM_S`, read by the host). The bounds: `chunk_um` 1 000–10 000; `pulse_um_s` the lane's speed bounds (9 000–70 000). They are compiled beside the defaults, and are in the dictionary too (`…_MIN`, `…_MAX`). This is an **init** command. On an image with the feed-forward, the host sends both at every connect: the `ff_chunk_mm` and `ff_pulse` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_follow_set`. It sends it again on every `ACEK_FF_SET` that changes one. A value out of its bounds is a host bug (`shutdown`). The host checks the keys against the dictionary's bounds at connect (a config error), and `ACEK_FF_SET`'s arguments before it sends (a G-code error), each rounded to the µm first. Taken at any time, never `busy`. Each value is read where it is used: the next dose, the next accrual's cap. RAM only | v0.9.0 |
| `ace2k_feed_base lane=%c rate_um_s=%i clear=%c` | none. The lane's base rate for the feed-forward ("The feed-forward", below), in µm/s, **signed**: positive is the filament the head is about to use, negative a long retraction the lane takes back, each up to `ACE2K_LANE_SPEED_MAX_UM_S`. 0 stops the accrual. What is owed either way is still delivered, in whole doses (a residual under one dose stays owed). `clear=1` also drops what is owed, with the sub-micrometre remainder; a dose already running still ends at its chunk. The host sends `clear=1` in three cases: for a lane remapped to another extruder; for each lane of an extruder with two or more lanes in follow; and, since v0.10.0, for the lane an automatic encoder calibration holds, at the hold ("The feed-forward"). Every other send is `clear=0`. Only a lane in mode 8 `following` takes it; any other mode ignores it silently. A base sent as the follow ends is a race, not a bug. Arming the follow starts at 0, and every end of the mode returns it to 0. A lane above 3, a rate beyond the lane's top speed either way, or a `clear` other than 0 or 1 is a host bug (`shutdown`). The host clamps the rate to `feed.speed_max` before it sends. RAM only | v0.9.0; signed since v0.12.0 |
| `ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c` | none. The load's parking distance (the firmware's own value is 300 000 µm: the measured path from the drive gear to the outlet, minus a margin; `hardware.md`, "Motors"), its speed (30 000 µm/s), and whether a filament put in at the entrance of an idle lane is pulled in on its own (1). An init command at connect and on every `ACEK_LOAD_SET`. `park_um` must be 1 .. `ACE2K_LANE_MOVE_MAX_UM`, the speed within the bounds and `auto_load` 0 or 1, else it is a host bug (`shutdown`) | v0.3.0 |
| `ace2k_feed_lane_grip lane=%c grip_um=%u` | none. Sets the lane's **grip** ("The grip", below): while it is set, the lane's next automatic load pulls only `grip_um`, with no tag search, and ends `loaded`. `grip_um` 0 clears it; otherwise 10 000 .. 45 000 µm (`ACE2K_FEED_GRIP_UM_MIN` .. `ACE2K_FEED_GRIP_UM_MAX`). The top bound stops 5 mm short of a load's first 50 mm, which the comparator does not judge, so a grip never meets the comparator. The host's default is 40 000 µm (`ACE2K_FEED_GRIP_UM`). The host reads all three dictionary constants. A lane above 3 or a grip out of its bounds is a host bug (`shutdown`). The host sends it with `grip_um=0` for every lane as an **init** command at every connect, so a grip never outlives the session. Taken at any time. RAM only | v0.11.0 |
| `ace2k_feed_lane_park lane=%c` | none. Sets the lane's **park** ("The park", below), in the grip's place: the lane's next automatic load pulls to the parking point, reads the tag on the way, does no tag search past it, and ends `loaded`. `ace2k_feed_lane_grip` with `grip_um=0` clears it, so the connect's clearing of every grip clears it too; a grip set after it replaces it. A lane above 3 is a host bug (`shutdown`). Taken at any time. RAM only | v0.12.0 |
| `ace2k_feed_lane_wait lane=%c` | none. Sets the lane's **wait** ("The wait", below), in the grip's place: an insert edge on the lane then starts no automatic load, and the filament stays where the hand left it. An insertion does not use it up, so it holds through any number of them. `ace2k_feed_lane_grip` with `grip_um=0` clears it, and so does any accepted host start on the lane (a refused one leaves it); a grip or a park set after it replaces it. A lane above 3 is a host bug (`shutdown`). Taken at any time. RAM only | v0.13.0 |
| `ace2k_feed_query rest_ticks=%u` | periodic `ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s` — one frame with the four lanes packed. `mode`: one byte per lane (0 idle, 1 feeding, 2 rolling_back, 3 unloading, 4 loading, 5 assisting, 6 assisting_back, 7 error, 8 following — the follow, since v0.8.0). `error`: one byte per lane, the kind that put the lane in error; 0 outside an error. `speed_um_s`: `uint32 × 4` little-endian, the speed the last loop measured on the tach (0 when idle). `duty_pct`: one byte per lane, the duty in effect. `bursts`: `uint16 × 4` little-endian, an assist's bursts or take-ups since it was armed; in the follow both are counted together. A feed-forward dose is not a burst: `ace2k_feed_ff_state` counts it. The count is kept until the lane's next mode starts. `seq`: one byte per lane, the sequence of the last start the unit accepted from the host on that lane (0 until one), kept through idle and error. A host that reconnects seeds its counter from it ("The start's sequence"). 40 bytes of payload, about 52 framed. That is over half of the transmit buffer, so on a busy tick the report may be held for the next one ("Reports"). The host asks for 1 Hz (`feed_report_hz`, 0.05–10 Hz, snapped to the 100 ms grid) | v0.3.0; mode 8 since v0.8.0 |
| `ace2k_feed_ff_query rest_ticks=%u` | periodic `ace2k_feed_ff_state doses=%*s back=%*s taut=%*s full=%*s epoch=%*s` — the feed-forward's counters, one frame with the four lanes packed. `doses`: `uint16 × 4` little-endian, the doses started. `back`: `uint16 × 4`, the reverse doses started (since v0.12.0). `taut`: `uint16 × 4`, the taut episodes whose burst started (the filament short of the head). `full`: `uint16 × 4`, this lane's full episodes whose take-up started (too much filament). Each correction is counted once per episode, when its burst or take-up actually starts, not while a taut reading waits under another lane's full. `epoch`: one byte per lane, stepped (wrapping) every time the lane enters a mode, from a host start or the unit's own (such as the automatic load). That is when the counters reset, so the host restarts its tally exactly when the epoch moves. Like `bursts`, the counters are kept after the mode ends, until the next entry. 36 bytes of payload (28 on an image without the reverse). Its own phase is 40 ms ("Reports"). On an image with the feed-forward, the host asks for it at the state report's rate (`feed_report_hz`), as an init command. During an automatic encoder calibration (since v0.10.0; its marks are the lane's `taut` steps) it asks for 10 Hz, and for the state report's rate again once the calibration ends | v0.9.0; `back` since v0.12.0 |
| — | `ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c` — sent at the end of every mode; it is also the one notice a running mode sends: [Feed events](#feed-events), below | v0.3.0 |
| `ace2k_lane_motor_query rest_ticks=%u` | periodic `ace2k_lane_motor fwd_um=%*s back_um=%*s` — the motor travel of each lane since boot, split by direction: `fwd_um` toward the printer, `back_um` toward the spool, `uint32 × 4` little-endian each, in micrometres (about 12.35 tach pulses per mm). The tach has no direction, so the unit counts each pulse in the direction it last drove the motor: the pulses that coast after a stop count in the direction of the move that stopped, and the pulses before the first move since boot (or of a motor turned with no move) count forward. A counters reset leaves both. Each wraps at 2³² µm (about 4.3 km); the host takes the difference of two reports modulo 2³², which is exact but across the wrap of the pulse count itself (2³² pulses, about 348 km). So the travel each way between two reports is exact, which the unsigned `fg` and the setpoint `speed_um_s` cannot give. Off at boot; `rest_ticks` 0 stops it. Phase 20 ms. The host asks for 1 Hz only while a module watches the motor's direction (`motor_watch`, "Host interface"), never as an init command | v0.13.0 |
| `ace2k_lane_scale_set lane=%c um_per_count_x10=%u` | none. The lane's encoder scale in tenths of a micrometre per count (default 12342). Accepted within ±20 % of the default (9874 .. 14810), at any time. The unit keeps the count and converts all of it with the scale in force, so a change moves `encoder_um`, from the next report on, by the count times the change. It is an **init** command for every lane at every connect, with the `laneN_encoder_scale` key's scale (`ACEK_CALIBRATE_ENCODER` stages one for `SAVE_CONFIG`) or the default, so a scale set live never outlives the session, and a saved or removed key never resets the MCU (since v0.12.0, `ACE2K_KEYS_AS_INIT`; from v0.3.0 to v0.11.0, and with an older image, the key is a config command too, and a change restarts the MCU). `ACEK_CALIBRATE_ENCODER … AUTO=1` also sends it live (`README.md`, "Calibrating the encoder"), and the host keeps its `encoder_mm` continuous across that change ("Host interface"). A connect that changes a lane's scale makes that lane's `encoder_um` jump once; this happens after a calibration that was not saved, which a Klipper restart undoes. A value outside the window, or a lane out of range, is a host bug (`shutdown`) | v0.3.0 |
| `ace2k_mains_query rest_ticks=%u` | periodic `ace2k_mains_state hz10=%hu present=%c rejects=%u` — `hz10`: the mains frequency in tenths of a hertz, from the zero-cross edges of the last 1 s window (two edges per cycle). `present`: 1 while an edge arrived within the last 100 ms. `rejects`: since boot, the edges the interrupt refused because they came less than 7 ms after the previous accepted one; such an edge is a slow edge crossing the input's threshold twice (v0.5.0). **The measurement.** The frequency comes from the accepted edges' own times: an even number of half-periods over the time they took, within a run of edges. It is published once a second. The published value is the longest measurement, when it holds at least 90 half-periods. But if any run over at least 10 half-periods reads off-band, the publication is off-band: it is the run furthest from the bands. **Flash writes.** A flash write holds interrupts off, and all its edges but one are lost. The write records whether the zero-cross input's pending bit held an edge at its end. If it did, the next edge is that late edge: it ends the run, and the next run starts after it. So a write never shortens a measurement. **Gaps.** Every gap between edges is checked: between two accepted edges however short their run, and around a write (from the run's last edge to the write's start, and from its end to the next run's first edge). Each gap is checked together with the gap before it. The two span an interval between edges of the same polarity, which is at most a full period on intact mains. The check is on the full period, not the half, because the zero-cross detector's two half-cycles differ when its threshold sits off the mains' zero. The lockout admits up to ±3 ms at 50 Hz; this is unmeasured at 127 V and at 230 V. A half-period bound would read clean 50 Hz mains as missing edges. After a write, the gap is paired with the lockout instead: the write hid how many edges it held, and a real half-cycle is at least one lockout long. A pair over 125 % of the period missed an edge, and `hz10` reads 1. The period is the last plausible `hz10`'s, or 57 Hz's when there is none; 57 Hz lies between 49 Hz's period and 61 Hz's plus a lockout, so either band is judged right. An edge the lockout refused is a bounce, not an edge of the mains, and ends no gap. Runs of one publication that disagree by more than 1 Hz also read 1 (a short run that a dip shortened can read in the other band). **Stale readings.** `hz10` keeps its last value while no run is long enough, for at most 3 s. Then a measurement of at least 10 half-periods is published, or 2 when there is none. 1 and 2 are not frequencies: both are off-band, and 0 still means none measured yet. The host shows them by name (`mains_note` "missed edge" / "stale", `mains_hz` None). **Windows with the mains absent.** A window in which the mains read absent is counted as a whole (its edges over its length). Every such window holds moments with no edge: before the boot's first edge, through an outage, or as little as up to 10 ms when the mains came back just after the window opened. So its count can read a few edges short (one edge at 60 Hz reads 59.5 Hz). Any counted window that reads plausible, whichever of its ticks read absent, publishes 0 (none measured). The heater takes no period from it, and the next window, which is measured, is the first plausible one (v0.6.0; v0.5.0 published such a count, up to 1 Hz off). The one reject the late edge can cause is not counted. An outage that begins inside a write reads absent up to the write's length (tens of ms) later (v0.5.0). The host asks for 1 Hz | v0.2.0; rejects v0.5.0 |
| `ace2k_fan_set on=%c seconds=%hu` | `ace2k_airflow_response op=%c accepted=%c reason=%c` (`op` 0) — `on` 1: both fans on for `seconds` (1–600), then off. `on` 0: the manual request is released at once and `seconds` is otherwise ignored; the fans go off only if nothing else keeps them on. `reason`: 0 ok; 1 `busy` (the dryer holds the fans, or an off while the heater holds a lease); 2 `range` (`seconds` 0 or above 600 with `on` 1, `seconds` above 65535 with either `on`, or `on` above 1). The dryer does not take the fans over from a manual run: its start is refused `busy` while a manual run holds them (`ACEK_FAN ON=0` first). A fan never switches off while an outlet NTC reads above 45 °C, whoever asks ("The dryer", below): an accepted off leaves them on until that rule releases them | v0.5.0 |
| `ace2k_flap_pulse flap=%c open=%c` | the same response (`op` 1) — one pulse of the flap's opening coil (`open` 1) or closing coil, for the compiled length (200 ms, `ACE2K_FLAP_PULSE_MS`, measured on the unit; never above 400 ms); then both coils are released. `flap`: 0 bottom, 1 rear. Every command pulses: there is no cache. A pulse asked while any flap's pulse runs or waits (the other flap's, or the dryer's boot close) answers 1 `busy`. Nothing is queued, so nothing moves later (v0.6.0; v0.5.0 queued it behind the running pulse and answered accepted). The published position is where the last pulse that ran to its end left the flap. It reads `unknown` while a pulse on that flap runs or waits. It is never read back (`hardware.md`, "Exhaust flaps"). `reason`: 1 `busy` (the dryer holds the outputs, or another pulse runs or waits); 2 `range` (`flap` above 1, or `open` above 1) | v0.5.0; `busy` for a pulse running v0.6.0 |
| `ace2k_airflow_query rest_ticks=%u` | periodic `ace2k_airflow_state fans_cmd=%c fans_read=%c owner=%c flaps=%c` — `fans_cmd`: 1 while the fans are commanded on. `fans_read`: bit 0 left, bit 1 right, set when the fan's pin reads high. This is the only read-back there is; there is no tachometer. `owner`: 0 `none`, 1 `manual`, 3 `dryer` (2 is not used). `flaps`: two bits per flap, bottom in bits 0–1, rear in bits 2–3: 0 unknown (no pulse has run to its end since boot, a pulse on the flap is running or waiting, or a shutdown cut a pulse), 1 open, 2 closed. The host asks for 1 Hz; phase 10 ms | v0.5.0 |
| `ace2k_dryer_start target_c=%c minutes=%hu` | `ace2k_dryer_response op=%c accepted=%c reason=%c` (`op` 0) — starts a cycle at `target_c` (15–65 °C) for `minutes` (1–1 440). `reason` 0 is ok; any other value is a refusal ("The dryer", refusals), and nothing starts. The bounds are the constants `ACE2K_DRYER_TARGET_MIN_C`, `ACE2K_DRYER_TARGET_MAX_C` and `ACE2K_DRYER_MINUTES_MAX`. Since v0.13.0 the start goes through the automatic modes: over a cycle of the humidity mode or a bound cycle still in `starting` or `heating`, it changes that cycle's target and time in place, and the cycle becomes manual ("The dryer's modes") | v0.5.0 |
| `ace2k_dryer_stop` | the same response (`op` 1) — ends a cycle in `starting` or `heating`: the heat stops at once, and the fans cool the heater down (`cooldown`). With no cycle running (`idle`, `cooldown`) nothing changes and the answer is 0 ok: a stop is idempotent. In `fault` it answers 3 `faulted`. Since v0.13.0 the stop goes through the automatic modes: the stop of a humidity-mode cycle sets the mode to manual (event 12 `auto_off`), and the stop of a bound cycle leaves the mode ("The dryer's modes") | v0.5.0 |
| `ace2k_dryer_clear` | the same response (`op` 2) — moves the dryer from `fault` to `idle`, once the fault's cool-down has ended and its cause is gone. Otherwise it answers 7 `not_cool`, 4 `sensors`, 5 `no_mains` or 3 `faulted`. It never clears after a tripped cutout (6 `cutout`, in any state: a power cycle clears it). Outside `fault` nothing changes and the answer is 0 ok | v0.5.0 |
| `ace2k_dryer_query rest_ticks=%u` | periodic `ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c remaining_min=%hu fans=%c flaps=%c fault=%c notices=%c lost=%hu oldest=%c` — `state` and `fault`: see "The dryer"; `fault` is the fault that put the dryer in `fault`, 0 otherwise. `target_c`: the target of the current or last cycle (after a lowering to 45 °C, the lowered one); 0 before the first cycle since boot. `drive_dc`: the outlet target the inner loop drives to, in tenths of a degree: the target + 7 °C, trimmed by the chamber loop by at most ±2 °C, never above the target + 8 °C nor 73 °C. `duty`: the heater's duty in percent (0–90), rounded to the nearest percent as the heater fires it (cut down to the percent below up to v0.12.0). `remaining_min`: the time left in the cycle. `drive_dc`, `duty` and `remaining_min` are 0 outside `starting` and `heating`. `fans`: bit 0 left, bit 1 right, set when the pin reads high, as in `fans_read`. `flaps`: as in `ace2k_airflow_state`. `notices`: bits ("The dryer"). `lost`: the events dropped on a full ring since boot (v0.6.0). `oldest`: the `seq` of the oldest event not yet acknowledged, or the next `seq` to be assigned when none is held; the host's base after a connect (v0.6.0). Every query opens the events, and sends again every event not yet acknowledged, oldest first: the unit holds them until the first query arrives ("The dryer", events). The host asks for 1 Hz, as an init command; phase 90 ms | v0.5.0; `lost`, `oldest` v0.6.0 |
| — | `ace2k_dryer_event kind=%c arg=%c seq=%c` — see [The dryer](#the-dryer), event kinds. `arg`: the fault for `fault`, the lane (1–4) for `lowered`, the value in the event table for 8–11 (v0.13.0), 0 otherwise. `seq`: the event's 8-bit sequence, one per event, wrapping; the same event sent again carries the same `seq` | v0.5.0; `seq` v0.6.0 |
| `ace2k_dryer_event_ack seq=%c` | none. Tells the unit that the host has every event up to `seq` (in 8-bit order: the events at most 127 behind it). Those events leave the ring. An event not yet acknowledged is sent again after every `ace2k_dryer_query` and every 5 s (`ACE2K_DRYER_EVENT_RESEND_MS`). So the host handles the events in sequence, and acknowledges the last one it handled in order. It never acknowledges one past a gap, which would drop the missing one. An event after a gap waits for the resend; an event already handled is acknowledged again and ignored. The unit ignores the whole acknowledgement when its `seq` is after the newest event ever sent (a resend does not lower that mark), or when no unacknowledged event was ever sent: it cannot come from a host that saw those events. A `seq` above 255 is a host bug (`shutdown`) | v0.6.0 |
| `ace2k_dryer_log_query index=%c` | `ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u` — the persistent log ("The dryer", the log). `index` 0 is the counters; 1–8 are the entries, newest first. An `index` above 8 is a host bug (`shutdown`) | v0.5.0 |
| `ace2k_dryer_start_bound target_c=%c minutes=%hu` | `ace2k_dryer_response op=%c accepted=%c reason=%c` (`op` 3) — a start bound to the host, for the print mode. The same checks and refusals as `ace2k_dryer_start`, and one more: 1 `busy` when the stored mode is not `print`, and nothing starts. The host treats it as any refused start: logged once per print, tried again after the hold. The cycle stops on its own once the host link has been down for 5 min (event 13 `stopped_link`) | v0.13.0 |
| `ace2k_dryer_stop_bound` | `ace2k_dryer_response op=%c accepted=%c reason=%c` (`op` 4) — the print mode's stop. It stops the running cycle only while it is bound to the host and in `starting` or `heating`, as `ace2k_dryer_stop` would, and the mode stays. Over a manual or a humidity-mode cycle, or with nothing running, it answers ok and does nothing. So a stop that the host decided from a report one poll old never ends a cycle that `ACEK_DRY` took over. The host checks for it at connect: an image without it gets the plain `ace2k_dryer_stop`, as before | v0.13.0 |
| `ace2k_dryer_auto_mode mode=%c` | `ace2k_dryer_auto_response op=%c accepted=%c reason=%c` (`op` 0) — the user's choice of mode: 0 `manual`, 1 `humidity`, 2 `print`. Setting `humidity` clears `stopped` and the failure count, even when it was already the mode. The same mode again otherwise changes nothing: no write of the page, and the hold at or above the limit runs on. A change of mode, or `humidity` again, drops an outcome still waiting for a cold box: it counts neither way. A change of mode while a humidity-mode or bound cycle runs stops that cycle. Stored on the config page. `reason`: 0 ok, 1 `range` (a mode above 2; nothing changes) | v0.13.0 |
| `ace2k_dryer_auto_limits limit_pct=%c max_failures=%c` | the same response (`op` 1) — the RH limit in whole percent (5–95) and the failed cycles in a row that stop the humidity mode (1–20). The host sends it from the first report after a connect when the unit's differ from its config, and again, at most every 5 s, while they still differ. Stored on the config page. A maximum at or below the failures already counted stops the humidity mode on the next tick (event 11 `auto_stopped`). `reason` 1 `range`: a value out of its bounds; nothing changes | v0.13.0 |
| `ace2k_dryer_auto_profile target_c=%c minutes=%hu` | the same response (`op` 2) — the temperature and the time of an automatic cycle, within the dryer's range (15–65 °C, 1–1 440 min). Stored on the config page. A temperature below the target of a running humidity-mode or bound cycle lowers that target at once; the end stays, and no event goes out. It never raises a cycle and never changes a manual one. `reason` 1 `range`: outside it; nothing changes | v0.13.0 |
| `ace2k_dryer_auto_timing hold_s=%u cold_window_s=%u link_s=%u` | the same response (`op` 3) — the bench's overrides of the hold at or above the limit (30 min), the cold window (30 min) and the link time of a bound cycle (5 min), in seconds. 0 keeps the default; a value below 1 s is raised to 1 s. RAM only: a reset brings the defaults back. The host sends it once per connect, on the first report, with 0 for every bench option that is not set, so a removed option is undone by a Klipper restart; a failed send is tried again 5 s later. Only a change restarts what it times: a different window empties the cold-box window, a different hold restarts the hold, and a different link time restarts nothing. So the same timing at every connect leaves the box cold and the hold running. `reason` 1 `range`: a value whose milliseconds do not fit 32 bits; nothing changes | v0.13.0 |
| `ace2k_dryer_auto_query rest_ticks=%u` | periodic `ace2k_dryer_auto_state mode=%c limit=%c max=%c failures=%c flags=%c profile_c=%c profile_min=%hu cold=%c own=%c bound=%c rh=%hu above_s=%u` — the modes' state ("The dryer's modes"). `mode`, `limit` (%), `max` and `failures`: as stored. `flags`: 0x1 `stopped`, the humidity mode stopped on its failures. `profile_c` and `profile_min`: the profile. `cold`: 1 when the box reads cold. `own`: 1 while the running cycle is the humidity mode's. `bound`: 1 while the running cycle is bound to the host. `rh`: the last valid chamber RH read with the dryer idle, in tenths of a percent; a cycle does not refresh it. `above_s`: how long the cold box has been at or above the limit, in seconds, 0 when it is not. While the chamber or RH reading is invalid, `cold` and `above_s` read 0, so the host starts nothing on a bad reading; inside, the cold box and the hold run on through a glitch shorter than 60 s. The host asks for 1 Hz; phase 90 ms, the dryer's | v0.13.0 |
| `ace2k_rfid_reader_query reader=%c` | `ace2k_rfid_reader_state reader=%c version=%c` — a soft reset of that reader, then a read of its version register: `0x18` on this unit's parts; 0 when the reset did not complete within 150 ms, which a bus that reads all ones also gives; `0x00` for a bus that reads all zeros. `reader`: 0 = A (lanes 3–4), 1 = B (lanes 1–2). The probe runs in task context and may take 150 ms. Since v0.13.0 a reader that the tag watch uses (its field wanted on, or an exchange with a tag running) is not reset, because the reset would cut a read. The answer then carries the version read at that reader's last soft reset, which is the watch's own configuration before its field came on. A reader whose switch-off has not read back off is not in use, so it is probed: the soft reset switches its field off too. The health self-test follows the same rule ("Health bits", bits 4 and 5) | v0.2.0 |
| `ace2k_rfid_query rest_ticks=%u` | periodic `ace2k_rfid_state state=%*s field=%c` — `state`: one byte per lane, the lane's tag state ("Tag states", below). `field`: bits 0x01 / 0x02 the field on at reader A / B; 0x04 / 0x08 a field fault at A / B (the field read back on after it was switched off; latched until the MCU restarts); 0x10 / 0x20 reader A / B dead (three failed configurations in a row, [Tag states](#tag-states)). The UIDs are not in the report, because four 10-byte UIDs do not fit one frame; `ace2k_rfid_lane_query` gives one. The host asks for 1 Hz; phase 60 ms | v0.4.0 |
| `ace2k_rfid_read lane=%c move=%c seq=%c` | `ace2k_rfid_read_response lane=%c accepted=%c reason=%c` — a read on command. `move` 0: an inventory of the lane's reader decides once the field's baseline is done, without motion ("The read on command"). `move` 1: the read with motion, the load's search from where the filament is, and back ("The read with motion"). `reason`: 0 ok; 1 `busy` (a session open on the lane, or a read on command pending on it; for `move=1` also the lane in a feed mode, moving or in error); 2 `no_filament`; 3 `no_link`; 4 `other_lane_moving` (`move=1`: another lane of the unit busy; since v0.13.0 a lane in the follow with its motor at rest does not count); 5 `unsupported` (`move=1` in an image without the feed, or with no search configured). A lane above 3 or a `move` above 1 is a host bug: `shutdown`. `seq` (since v0.13.0): the host's key for a read with motion, 1–255 per lane, a new one for each read; 0 is no key, and a read without motion ignores it. Klipper sends a query again, with the same bytes, when its answer is lost. A read with motion whose `seq` is the lane's last accepted one is that copy, not a second read: while the search it started runs, or within 1 s after that search ended (`ACE2K_FEED_RETRY_WINDOW_MS`) with no other mode entered on the lane since, it is answered `accepted=1`, and nothing else happens: no search starts, the lane's tag is not forgotten. A `seq` above 255 is a host bug: `shutdown`. An image before v0.13.0 has the command without `seq`: the host then sends it without one, and a lost answer can make the unit search twice | v0.4.0; `seq` v0.13.0 |
| `ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s` | none; the unit answers with the step's `ace2k_rfid_data` frames. One step of the lane's open session ("RFID sessions"). `op` 0: read NTAG pages; `arg` is the first page (0–244), `blocks` the number of groups of four pages (1–3), `key` is ignored. `op` 1 / 2: authenticate MIFARE Classic sector `arg` (0–15) with key A / B (`key` is six bytes), then read the data blocks whose bits are set in `blocks` (bits 0–2, never the trailer). Host bugs (`shutdown`): an `op` above 2, a MIFARE step with a key of another length, a lane, sector, page or block mask out of bounds, a `session` above 255. A step for a session that is not open, or sent while the session's previous step waits or runs, comes from a stale host and is ignored. Each accepted step restarts the session's 500 ms idle timer | v0.4.0 |
| `ace2k_rfid_done lane=%c session=%c result=%c` | none. The host ends the session. `result` 1: read (the lane becomes `read`). `result` 0: give up (back to `pending`, or to `searching` inside its load). A session that is not open is ignored. A lane above 3, a `session` above 255 or a `result` above 1 is a host bug (`shutdown`) | v0.4.0 |
| `ace2k_rfid_forget lane=%c` | none. Returns a `read` or `no_tag` lane to `pending`, and forgets its UID: use it when the tag on the spool was swapped. The next move of the lane reads it again. On a `read` or `no_tag` lane, the state event (`pending`) also answers a read on command that waits on the lane. On a `pending` lane it drops the UID the lane keeps from a session that ended without a read, and a UID that waits for the reader's session: the lane stays `pending`, no event goes out, and a read on command that waits on the lane stays and is answered by the next inventory. After that, the lane's tag counts as an unknown tag: the lane's next move reads it again, and the neighbour may take it by the attribution rules (since v0.13.0; before, a no-op there). A `searching` or `reading` lane is left alone. A lane above 3 is a host bug (`shutdown`) | v0.4.0 |
| `ace2k_rfid_lane_query lane=%c` | `ace2k_rfid_lane_state lane=%c state=%c uid=%*s` — the lane's tag state and, while the lane is `reading` or `read`, the attributed UID (4, 7 or 10 bytes; empty otherwise). A UID is per-unit data: it belongs in a log, never in a document | v0.4.0 |
| `ace2k_rfid_search_set search_um=%u` | none. The reach of the load's search, as filament travelled since the load began ("The load's search"). The unit starts from `ACE2K_FEED_SEARCH_DEFAULT_UM`. The host sends it at every connect as an **init** command, `rfid_search_mm` or the default, so a change never resets the MCU (since v0.12.0, `ACE2K_KEYS_AS_INIT`; before, and with an older image, a config command sent only with the key). 0, or a value above `ACE2K_FEED_SEARCH_MAX_UM`, is a host bug (`shutdown`). Only in an image with the feed | v0.4.0 |
| — | `ace2k_rfid_tag lane=%c session=%c uid=%*s atqa=%hu sak=%c` — a session opened on the lane: its number (1–255, never 0), the tag's UID, ATQA and SAK ("RFID sessions") | v0.4.0 |
| — | `ace2k_rfid_data lane=%c session=%c block=%c status=%c data=%*s` — one frame per 16 bytes a step read (`status` 0, `block` the MIFARE block or the first of four NTAG pages, `data` 16 bytes), and one frame for a step that failed (`status` 1 `auth_failed`, 2 `tag_gone`, 3 `error`; `block` the step's first block or page; `data` empty) ("RFID sessions") | v0.4.0 |
| — | `ace2k_rfid_event lane=%c kind=%c session=%c` — one of three things: the lane's new tag state (`kind` 0–5; the opening of a session is announced by its `ace2k_rfid_tag`, not here); the unchanged state, as the answer to a read on command that no inventory decided; or 6 `ambiguous` ("Tag states"). `session` is 0 in this release | v0.4.0 |
| `ace2k_health_query` | `ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c` — `now` and `latched`: the two fault masks ("Health bits", below). `post_ms`: the tick time in milliseconds of the last full pass (0: none yet; the first runs 2 s after boot). `reset_cause`: the cause of the last reset | v0.2.0 |
| `ace2k_health_run` | none. Takes a new snapshot of the counters, and runs the full pass again 2 s later: every bit, the reader probes included | v0.2.0 |
| `ace2k_health_clear` | none. Sets `latched` to `now` | v0.2.0 |
| `ace2k_uid_query` | `ace2k_uid_response uid=%*s` — the MCU's 96-bit device identifier, 12 bytes in the order they lie in the part. The host prints them as 24 hex digits. Per-unit data: it belongs in a log, never in a document. Answered after a Klipper shutdown too, since v0.12.0 (a unit keeps a shutdown across a host restart, and the host reads it at every connect) | v0.2.0 |

The dictionary's constants also list the pins the firmware owns:

- `RESERVE_PINS_serial` and `RESERVE_PINS_ace2k_led`, since v0.1.0;
- `RESERVE_PINS_ace2k_adc`, `_sensors`, `_env`, `_lane`, `_mains` and `_rfid`, since v0.2.0;
- `RESERVE_PINS_ace2k_motor` and `_remap`, since v0.3.0;
- in an image with the dryer's outputs, since v0.5.0: `RESERVE_PINS_ace2k_airflow` (PE12, PE8,
  PD3, PD4, PD5, PD6) and `RESERVE_PINS_ace2k_heat` (PC8; before v0.13.0 also PC9 in an image
  without the lane);
- `RESERVE_PINS_ace2k_cutout` (PC9) in every image, since v0.13.0. Before, PC9 was in
  `RESERVE_PINS_ace2k_lane` or `RESERVE_PINS_ace2k_heat`, and an image with neither did not
  reserve it;
- `RESERVE_PINS_ace2k_gate` (PC8, the triac gate) in an image without the heater, since v0.13.0.
  An image with the heater reserves PC8 in `RESERVE_PINS_ace2k_heat`, so every image reserves it
  exactly once. Before, an image without the heater reserved it in `RESERVE_PINS_ace2k_lane`, and
  an image with neither the heater nor the lane did not reserve it.

`hardware.md` lists each pin. The host refuses a `printer.cfg` section that claims one of them.

Since v0.3.0, three of the lists go beyond the pins the firmware drives or reads. There are two
reasons, both in Klipper's F1 port (`src/ace2k_board/motor.c` and `encoder.c` say which pin is
whose):

- `RESERVE_PINS_ace2k_motor` names the twelve motor lines and every other channel pin of TIM2. A
  host `[output_pin]` with `hardware_pwm` on one of them would reprogram the whole timer, and the
  four motors' PWM with it. In the same way, `RESERVE_PINS_ace2k_lane` names the other channel pins
  of the timers the lane counts on. In an image without the sensors, PA2 / PA3 join the motor's
  list and PB0 / PB1 / PD15 the lane's; in an image without the readers, PD12 / PD13 join the
  lane's. (Klipper refuses a pin reserved under two names.)
- `RESERVE_PINS_ace2k_remap` (PA13, PA14, PD8, PD9, none of them driven by ace2k) names the pins
  whose host configuration would rewrite the remap register from a shadow copy that never saw the
  TIM2 full remap. That would put two PWM channels on lane 4's encoder inputs in the middle of a
  move. In an image with the dryer's outputs (v0.5.0), PD5 and PD6, the rear flap's pins, leave
  this list for the airflow's list.

Every image also names, since v0.12.0:

- `ACE2K_VERSION`, the release string `ace2k_version` answers. A host tool reads an image's version
  from the dictionary compiled into it, without running it;
- `ACE2K_LINKPROOF_WINDOW_MS` 180000, the link proof's window ("The link proof", below). The flasher
  reads it to say how long an image it could not prove waits for a host;
- since v0.13.0, `ACE2K_LINKPROOF_RETRY_PAST_WINDOW` 1: a proof the heater put off waits past the
  window while the host keeps talking, and does not end in recovery ("The link proof", below). The
  module reads it to keep watching the state after the window.

The bounds of a move are constants too, so the host can check them without its own copy
(`hardware.md`, "Motors", says where each comes from):

- `ACE2K_LANE_SPEED_MIN_UM_S` 9000, `ACE2K_LANE_SPEED_MAX_UM_S` 70000 and `ACE2K_LANE_MOVE_MAX_UM`
  2000000;
- `ACE2K_LANE_DUTY_MIN_PCT` 20 (provisional: the loaded floor is at most 20 %, and nothing below
  it has been measured);
- `ACE2K_LANE_LANDING_UM` 10000: the last 10 mm of a move, or the whole of a shorter one, at the
  floor speed, which a waited G-code budgets as the firmware's deadline does.

An image with the feed adds:

- `ACE2K_FEED_ASSIST_BURST_UM` 100000, the longest forward assist burst, which bounds the
  comparator's windows (`ace2k_feed_thresholds_set`);
- since v0.8.0, the follow's values with their bounds (`ace2k_feed_follow_set`):
  `ACE2K_FEED_FOLLOW_FLIP_MS` 200, `ACE2K_FEED_FOLLOW_FLIP_MS_MIN` 50,
  `ACE2K_FEED_FOLLOW_FLIP_MS_MAX` 2000, `ACE2K_FEED_FOLLOW_TAKE_UM` 15000,
  `ACE2K_FEED_FOLLOW_TAKE_UM_MIN` 5000, `ACE2K_FEED_FOLLOW_TAKE_UM_MAX` 30000, and since v0.11.0
  the tail's `ACE2K_FEED_FOLLOW_TAIL_UM` 2000000, `ACE2K_FEED_FOLLOW_TAIL_UM_MIN` 200000 and
  `ACE2K_FEED_FOLLOW_TAIL_UM_MAX` 2000000. The host reads its defaults and bounds there. To the
  host, an image without them, the tail's included, has no follow;
- since v0.9.0, the feed-forward's values with their bounds (`ace2k_feed_ff_set`):
  `ACE2K_FEED_FF_CHUNK_UM` 3000, `ACE2K_FEED_FF_CHUNK_UM_MIN` 1000, `ACE2K_FEED_FF_CHUNK_UM_MAX`
  10000, `ACE2K_FEED_FF_PULSE_UM_S` 15000, `ACE2K_FEED_FF_PULSE_UM_S_MIN` 9000 and
  `ACE2K_FEED_FF_PULSE_UM_S_MAX` 70000 (the lane's speed bounds). An image without them, or whose
  `ace2k_feed_ff_state` carries no `epoch`, has no feed-forward;
- since v0.13.0, the tip snag's bounds beside its defaults (`ace2k_feed_snag_set`):
  `ACE2K_FEED_SNAG_FWD_UM_MIN` 5000, `ACE2K_FEED_SNAG_FWD_UM_MAX` 50000,
  `ACE2K_FEED_SNAG_LAG_UM_MIN` 1500, `ACE2K_FEED_SNAG_LAG_UM_MAX` 20000,
  `ACE2K_FEED_SNAG_DUTY_PCT_MAX` 100 (the duty's least value is 0, with no constant),
  `ACE2K_FEED_SNAG_BACK_UM_MIN` 2000, `ACE2K_FEED_SNAG_BACK_UM_MAX` 20000,
  `ACE2K_FEED_SNAG_REST_MS_MIN` 100, `ACE2K_FEED_SNAG_REST_MS_MAX` 2000,
  `ACE2K_FEED_SNAG_FREE_UM_MIN` 20000 and `ACE2K_FEED_SNAG_FREE_UM_MAX` 200000. The defaults
  (`ACE2K_FEED_SNAG_FWD_UM` and the others) are there since v0.7.0;
- since v0.12.0, the reverse (`ACE2K_FEED_FF_REVERSE`): an image without it keeps the unsigned
  `ace2k_feed_base` and the frame without `back`; the host credits every retraction there and says
  once `ace2k: the unit's firmware does not take back a long retraction; flash v0.12.0`;
- since v0.13.0, `ACE2K_FEED_REFUSED_FOLLOWING` 8, the reason `ace2k_feed_speed_set` gives on a
  following lane. An image without it takes the speed there, so the host refuses it itself.
- since v0.13.0, the held stop's values with their bounds (`ace2k_feed_held_set`):
  `ACE2K_FEED_HELD_UM` 3000, `ACE2K_FEED_HELD_UM_MIN` 1500, `ACE2K_FEED_HELD_UM_MAX` 20000,
  `ACE2K_FEED_HELD_MS` 550, `ACE2K_FEED_HELD_MS_MIN` 50 and `ACE2K_FEED_HELD_MS_MAX` 2000. An
  image that takes `ace2k_feed_stop_when` has them. The host takes the stop only when the
  dictionary has `ace2k_feed_stop_when`, its answer and `ace2k_feed_held_set` with these formats,
  and all six constants. The constants without the commands mean another commit, and the connect
  is refused.

An image with the reader's field and the feed adds the reach of the load's search:

- `ACE2K_FEED_SEARCH_MAX_UM` 1000000, the ceiling `ace2k_rfid_search_set` accepts;
- `ACE2K_FEED_SEARCH_DEFAULT_UM` 750000, the reach the unit starts with. That is one revolution of
  the largest spool that fits (at most π × 200 mm ≈ 628 mm of filament), plus the longest tag
  window measured (102–116 mm on an Anycubic tag; `hardware.md`, "RFID readers"). It is the host's
  value for `rfid_search_mm` when `printer.cfg` does not set it. A waited `ACEK_LOAD`'s budget and a
  `MOVE=1` read's wait are computed from it.

PC9, the cutout's latch-reset pin, is never driven or configured by any image. Every image
reserves it exactly once, in `RESERVE_PINS_ace2k_cutout` (since v0.13.0), so no host section can
claim it.

## Reports

Every report binding keeps one deadline (`firmware/src/ace2k/core/report.h`). The query sets the
period: `rest_ticks` of the MCU's clock, rounded to the nearest millisecond (since v0.13.0 for every
report; up to v0.12.0 only the feed's two rounded, and the others cut the period down to the
millisecond below). Each module has its own phase into the period: sensors 0 ms, airflow 10 ms, the
lane's motor report 20 ms (since v0.13.0), env 30 ms, the feed-forward's counters 40 ms (since
v0.9.0), lane 50 ms, rfid 60 ms, mains 70 ms, feed 80 ms, dryer 90 ms. The dryer's modes report shares the
dryer's 90 ms (since v0.13.0): the dryer's task sends it right after the dryer's report. The 10 ms
tick raises the report when the deadline passes, and a task sends it.

**A full transmit buffer.** The unit's transmit buffer holds 96 bytes. Klipper refuses a frame that
does not fit it, judged on the frame's real encoded length. The bindings try each report and event,
and hold one that was refused for the next tick: a report by its due flag, an event in its ring.
The due report goes before the events, and 24 bytes are kept free behind their frames for a command
response. So the bindings' frames are held, not dropped. Two things can still be lost:

- events, when a ring overflows past 15 pending events. The firmware counts them, in a `dropped`
  counter per ring, but this build does not report them on the wire;
- a command response, which is a plain send and is dropped against a full buffer (below, "a burst's
  replies must fit"). The transport's query retry covers this: a repeated `ace2k_feed_start` is
  accepted as the same command.

**Timing.** An event leaves on the first tick its frame fits, normally the next one (≤ 10 ms). A
held report goes out one tick late. The feed's report, at 80 ms, then lands on 90 with the dryer
report and the modes report, and the three are tried in turn on that tick; a frame that does not
fit then leaves at 0 ms, beside the sensors' report. The dryer's events wait behind both of the
dryer's reports. The rfid's report then lands on 70 with the mains report, and the two are tried
in turn on that tick. Because the due report goes before the
events, a state report can show a lane idle before that lane's terminal event has left. The event
leaves on the first tick its frame fits: normally the same tick as the report, one tick later under
buffer pressure. A host must not read "idle with no event yet" as a lost event.

**Phases.** The phases keep the periodic reports on distinct ticks. They hold only for periods that
are **multiples of 100 ms**: the host snaps a `feed_report_hz` that is off that grid onto it. The
deadlines are anchored to the absolute 100 ms grid, not to the tick a query arrived on. **Every
report is one frame per module, never one per lane.** `ace2k_sensors_thresholds`,
`ace2k_lane_counters`, `ace2k_lane_motor`, `ace2k_rfid_state`, `ace2k_feed_state` and
`ace2k_feed_ff_state` pack the four lanes, so a report is one try, sent whole or held whole. After a stall (the link proof's flash
write), a deadline moves whole periods ahead: the reports the stall swallowed are skipped, not
replayed, and the grid keeps its phase. The reports survive a Klipper shutdown (the tick timer is
re-armed). The host sends every report query as an init command, so an MCU reset restarts them.
The lane's motor report is the exception: it is off at boot, and the host turns it on and off while
a module watches the motor's direction.

## Health bits

`ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c`: a set bit in `now` means the
check **fails** right now. A set bit in `latched` means it has failed since boot or since
`ace2k_health_clear`. The first full pass runs 2 s after boot, because the counters must be quiet
and the zero-cross window must have filled. After it, a periodic pass every second evaluates every
bit again, except the boot-only ones, which keep their value. `ace2k_health_run` takes a new
snapshot of the counters and runs a full pass 2 s later: every bit, the reader probes included.
What a read-only image cannot verify has no bit: the fans, the flaps, the motors and the heater
gate, the reader antennas, the transceiver, and a Hall switch stuck open.

| Bit | Name | Fails when | Evaluated |
|---|---|---|---|
| 0, 1 | `ntc_left`, `ntc_right` | the filtered reading (α = 1/8) is outside 100–3200 mV | boot, every second |
| 2 | `chamber` | no valid chamber reading in the last 5 s (none yet counts; 2 s up to v0.12.0) | boot, every second |
| 3 | `chamber_plausible` | the last chamber reading is outside −40…85 °C or above 100 % RH, or none has arrived yet | boot, every second |
| 4, 5 | `reader_a`, `reader_b` | the version register is not `0x18` after a soft reset (0 when the reset did not complete within 150 ms, which a bus that reads all ones also gives; `0x00` for a bus that reads all zeros). Since v0.4.0 also a field fault (the field read back on after it was switched off) or a dead reader ("Tag states") | probed at boot and on run, one reader per task run, so a full pass takes three task runs; also probed by `ace2k_rfid_reader_query`, with the same rule since v0.13.0. The bit follows the last probe, and follows the field fault and the dead flag at every periodic pass. A pass leaves a reader alone while its field is wanted on or an exchange runs on it (a soft reset would cut a read), and keeps that reader's last verdict. A run clears a dead reader first, so the watch tries it again |
| 6 | `zerocross` | no zero-cross edge in the last 100 ms | boot, every second |
| 7 | `mains_hz` | the frequency last published (`ace2k_mains_state` `hz10`, once a second) is not 49.0–51.0 or 59.0–61.0 Hz, the markers 1 and 2 included | boot, every second |
| 8 | `cutout` | the cutout input reads high (debounced); it idles low | boot, every second |
| 9–12 | `insert1`–`insert4` | the lane's raw insert reading is ≤ 20 mV or ≥ 3250 mV (a disconnected or shorted lever board; initial values), or no ADC pass exists yet | boot, every second |
| 13–16 | `buffer1`–`buffer4` | *rest* and *pushed* asserted together. Enabled in the release build: over 134 debounced events on the four lanes, measured on the unit, they never were | boot, every second |
| 17–20 | `encoder1`–`encoder4` | the board's raw count moved by more than 2 during the 2 s window | boot, on run |
| 21–24 | `fg1`–`fg4` | any tach pulse during the 2 s window | boot, on run |
| 25 | `sensors_fresh` | the last ADC pass is more than 50 ms old, or none exists. A pass waits 50 ms after a tick catch-up (a flash stall), so a stall does not read as staleness. Past 100 ms (`ACE2K_ENV_ADC_STALE_MS`) the readings it feeds turn invalid as well: the two NTCs and VDDA, and since v0.13.0 the insert sensors, which stop every running lane (`stopped_sensors`) and refuse every start (`sensors_stale`) until the scan is back ("Stale insert sensors", below) | boot, every second |
| 26 | `vdda` | the supply, from the internal 1.2 V reference, is outside 3135–3465 mV (3.3 V ± 5 %) | boot, every second |
| 27 | `clock` | the core is not on the PLL, or the PLL is not fed from the crystal, or the crystal is not ready | boot, on run |
| 28 | `watchdog` | the independent watchdog does not read back as running with its expected configuration: the low-speed oscillator on, no prescaler / reload update left in flight, and prescaler and reload at their reset values /4 and 0xFFF rather than the bootloader's longer timeout. The bootloader arms it before it jumps to the application, so the bit attests a running watchdog, not who armed it (`hardware.md`) | boot, on run |
| 29 | `config_page` | the ace2k config page is corrupt (magic, layout or CRC wrong, or a store failed after erasing it); blank is fine | boot, on run |
| 30 | `image_crc` | the image's CRC-32 does not match the one `firmware/tools/mkimage.py` appended after it, before the 8-byte trailer. Computed once, at the first full pass; a run reuses the result | boot |

Reset causes: 0 unknown, 1 power-on, 2 reset pin, 3 software, 4 independent watchdog, 5 window
watchdog, 6 low-power. The flags are read once at boot and then cleared, so every boot reports its
own cause. Left alone they are sticky, and every later boot would keep reporting the oldest reset.
Measured on the unit: `software` after a reflash through recovery (the bootloader's own reset after
it commits the image) and after Klipper's `reset` (what `FIRMWARE_RESTART` sends with
`restart_method: command`); `power-on` after a power cycle. The application cannot reach cause 4 on
this unit: the bootloader traps a watchdog reset into its recovery mode.

## Feed events

Every mode of a lane ends with one event, and a running mode can send one notice:
`ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c`.

- `mode` is the lane's mode when the event happened: 0 idle, 1 `feeding`, 2 `rolling_back`,
  3 `unloading`, 4 `loading`, 5 `assisting`, 6 `assisting_back`, 7 `error`, 8 `following`. A
  `runout` that ends an error names the mode that failed, not the error state it left.
- `motor_um` is the tach's travel since that mode started. `filament_um` is the encoder's, signed,
  positive toward the printer. Their difference is the deficit: what the motor turned that the
  filament did not follow. Both cover every move of the mode: an assist's bursts, the follow's
  bursts and doses, a load's return, an unload's tail, a feed resumed after a touch.
- `seq` is the sequence of the start that entered the mode, or 0 for a motion the unit started
  itself ("The start's sequence", below).

**The supervisors** behind the error kinds run every tick in every mode whose motor is commanded.
That includes an assist's bursts, but not a load from the bay's first 50 mm: the hand and the pinch
catching the filament there are not a slip. The read with motion has no such grace: its filament
is already gripped, so the supervisors judge it from its first millimetre (since v0.13.0; up to
v0.12.0 it had the 50 mm too).

The *comparator* checks the motor against the filament, with two triggers:

- the *standstill* trips when `stall_check_um` (20 mm) of motor has turned since the filament last
  moved 1.5 mm in the commanded direction. 1.5 mm is more than one encoder count at every accepted
  scale;
- the *partial* trips when, over a window of `slip_check_um` (50 mm) of motor, the filament fell
  short by more than `slip_allow_um` (15 mm). Otherwise the window restarts.

The thresholds are set with `ace2k_feed_thresholds_set`. The standstill is checked first.

The buffer decides the kind. Its plunger has three readings:

- *full*: the plunger at its pulled end. The lane pushed too much filament.
- *rest*: the spring centred.
- *taut*: the plunger at its pushed end. The head pulls the filament tight.

A forward trip with the lane's own buffer full is `stuck`; without it, `tangled`. (In a feed or a
load's pull it is `blocked` instead, below.) A trip toward the spool is `stuck`, with one exception.
A standstill with the plunger at rest is no fault: the end of the filament (its *tail*) has passed
the drive gear (`unload_incomplete`, in the table).

Two readings end a move without the comparator:

- In `feeding` and in a load's pull, the lane's own buffer full ends the move as `blocked`, unless
  the tip snag's tolerance (below) forgives it. (A free tip judges it in windows instead: "The free
  tip", below.) The plunger's ≈ 18 mm of travel at the spring's
  force is the early warning, before anything grinds. (This was decided on the unit: a feed into a
  held tube ran to done with the motor pushing at full force.) In a load's tag search it is the
  search's obstacle return instead. Up to v0.10.0 it was `stuck`.
- In every running mode, the insert sensor falling ends the mode (`runout`, or `unloaded` in an
  unload). The follow is the exception: there the sensor falling begins the tail instead (since
  v0.11.0, "The tail", below). In the tail, every comparator trip is its end, `tail_out`, never an
  error kind, and so is a move's deadline once the end has left the encoder (since v0.13.0).

The lane core's own outcomes come through as `motor_stalled` and `timeout`, in every mode. Two
deadlines are not `timeout`: a reverse move's with the plunger at rest (the unload's tail), and a
deadline in the follow's tail once the end has left the encoder (`tail_out`, "The tail", below).

**Blocked** (since v0.11.0). A push toward the head that meets something that cannot move stops
without an error and waits. A push toward the head is a feed, or a load's pull to its parking point
(the automatic load's included). These end it as `blocked` (18):

- a forward comparator trip, the standstill or the partial, whatever the buffer reads;
- in a feed without `free_tip`, this lane's full reading when the tip snag's tolerance does not
  forgive it: the lag, the duty, the plunger not back at rest, a second full, or a touch that the
  lane refuses, that stalls or that times out;
- in a feed with `free_tip` and in a load from the bay's pull, the filament falling more than
  `lag_um` behind the motor within a window while the plunger reads full ("The free tip", below).

The event carries the mode's odometers. The lane is `idle`, with no error to clear. The filament
stays where it stopped: inside the unit (the filament stopped at the encoder) or in the tube ahead
(this lane's plunger at its pulled end). A load `blocked` before its parking point leaves the
filament parked where it stopped; nothing retries it. In the tag search the same obstacle is the
search's obstacle return, which ends `loaded`, as before. The move's own `motor_stalled` and
`timeout` stay errors, in a feed and a load's pull too. Every other mode keeps its verdicts: the
assists, the follow, a rollback, an unload, a load's return and reacquire. A loose piece left in
the tube is no obstacle: it moves, and the push takes it ahead of the filament.

**The tip snag** (since v0.7.0). A filament tip that is thicker than the filament can catch the
entry of the tube that the plunger carries. It then drags the plunger to an end with nothing
holding the filament (`hardware.md`, the buffer's readings).

*Forward* (a feed started without `free_tip`, and the read with motion's search), this lane's full
reading no longer ends the move at once. It starts a *watch*: the move runs on, and every tick
judges it from the motor and encoder counts taken at that full reading.

- The filament more than `lag_um` (5 mm) behind the motor is `stuck`. In a feed it is `blocked`
  instead, as is every verdict of this list that says `stuck` ("Blocked", above); in the read with
  motion's search, the obstacle return ("In the tag search", below).
- The duty more than `duty_pct` above the feed-forward duty of the setpoint the last speed loop
  used is `stuck`. The default, 0, leaves this guard off. It was built as a second guard, for a hold
  the filament slips through, but it did not prove effective on the unit. The lag ended every jam
  first, and a fixed excess has no margin: measured on the unit, ≤ 9 points on a healthy move
  against 10–14 for a jam at its stop. It is kept for a possible future case. For that case the rise
  since the full reading, which cancels a spool's drag, would be the better reading; it is not
  built.
- The plunger back at rest on its own is the `snag` notice, and the move goes on, untouched.
- `fwd_um` (20 mm) of motor turned, or the move's length reached during the watch, is the *touch*.
  The lane stops, then moves `back_um` (5 mm) against the mode's direction, at the mode's speed.
  Then the plunger must read rest within `rest_ms` (500 ms), else it is `stuck`. At rest, the unit
  sends the `snag` notice and resumes the move for what is left of the mode's length on the
  encoder.
- A second full in the same mode is `stuck` at once; in a feed, `blocked`.

*Toward the spool* (a rollback, an unload), the same touch runs the other way. It applies to a
comparator trip with the plunger taut and the insert sensor present, after the filament came back
at least `free_um` (50 mm) in the mode. The touch is a forward move of `back_um`. Then come rest
within `rest_ms`, the notice, and the reverse move resumed for what is left (for an unload: its
budget, with the insert sensor still its goal). A trip before that free travel stays `stuck` at
once: a tip held in the head is held from the start, and comes back at most the plunger's slack.
A rollback with `free_tip` forgives every such trip, not the first only ("The free tip", below).

With under 2 mm left to resume, a feed or a rollback ends `done`, and an unload ends
`unload_incomplete` (its budget spent). The tail at rest, a load's return and reacquire, and the
assist are unchanged.

**In the tag search** no verdict is an error. A load from the bay's search has a free tip, so the
tolerance above does not run there ("The free tip", below). The read with motion's search keeps
it: its filament may sit in an extruder, where a filament that follows the motor proves nothing.
There every verdict that is `stuck` above is the search's obstacle instead: the return to where
the read began, and `loaded`, no error. That covers the lag, the duty, the plunger not back at
rest, a second full, and a touch that the lane refuses, that stalls or that times out. It also
covers the search's own move stalled or past its deadline during the watch (`motor_stalled` and
`timeout` elsewhere). In either search, a tag read while the tolerance or a window runs ends the
search when the plunger is back at rest (on its own, or after the touch), with the return and
`found`, instead of searching on. It is `found` too if the tolerance or the window then ends on a
verdict. A start given to another lane ends the search at once, whatever its phase, with the
return and `yield`. So does another lane's motor that starts on its own, such as a follow's move
(since v0.13.0).

**One tolerance per mode**: a second catch in the same mode, either way, is `stuck` — but in a
rollback with `free_tip` ("The free tip", below). While a watch,
a touch or the wait for rest runs, the comparator is not asked: the lag is the sharper test, and
the touch moves nothing the comparator should judge. The comparator restarts its bases at the
resume. The touch is one bounded lane move, which STOP, the link, a shutdown and a runout end as
they end any move. A touch that stalls or times out, or a resume that the lane refuses, is `stuck`.
In a feed it is `blocked`; in the read with motion's search, the obstacle return. A `done` after a
tolerance carries the whole mode's odometers, as every final event does ("Feed events", above).
The values can be set at run time (`ace2k_feed_snag_set`, above).

**The free tip** (since v0.12.0). A feed started with `free_tip`, and a load from the bay (the
automatic load and the load command: its pull and its tag search), judge this lane's full by the
filament alone. From each full, every `fwd_um` of motor is a window. At every tick, from the
window's start, the filament falling more than `lag_um` behind the motor ends the push `blocked`
(the tag search: the obstacle return). A filament that never falls that far behind has covered at
least `(fwd_um − lag_um) / fwd_um` of the motor's travel when the window closes (75 % with the
defaults). A window passed with the filament following, or the plunger back at rest, is
resistance: the `snag` notice, once in the mode, and the push goes on. There is no touch, no wait
for rest and no limit on the number of fulls. The comparator runs under it. A stall in the
search's free phase is the obstacle return, and so is a deadline. Without `free_tip` a feed keeps
the tolerance above. So does the read with motion: it can run on a lane whose filament sits in an
extruder, where the filament following the motor proves nothing.

A rollback started with `free_tip` (an image with `ACE2K_FEED_FREE_ROLLBACK`) keeps the touch
toward the spool above, without its limit of one. Every taut trip with the insert sensor present is
forgiven when the filament has come back at least `free_um` past where the last forgiven catch
tripped (the first: from the mode's start). Each forgiven catch is so `free_um` of progress, and a
tip held from the start, which comes back at most the plunger's slack, is still `stuck` at once. A
catch short of that travel is `stuck`, as is a touch whose plunger does not come back to rest. The
`snag` notice goes out once in the mode, for the first catch. An unload never reads the flag: a
catch it does not forgive is the sign that the head still holds the tip.

| Kind | Name | When | The lane after |
|---|---|---|---|
| 0 | `done` | a feed or a rollback travelled its length on the encoder | idle |
| 1 | `stopped` | `ace2k_feed_stop` in a running mode; a load still in its settle is cancelled with nothing moved | idle |
| 2 | `stopped_link` | 3 s without a valid frame from the host (bytes alone do not count since v0.13.0: noise on the line is bytes too). This applies alike to a running move, an assist waiting between bursts and a load in its settle. The event goes out to no one. `ace2k_link_gap_query` reports the longest silence so far, to measure the margin of the 3 s (since v0.13.0) | idle |
| 3 | `stopped_shutdown` | a Klipper shutdown. This applies alike to a running move, an assist waiting between bursts and a load in its settle (a lane in error stays in error, with no event). Delivered during the shutdown. From then on nothing starts until the MCU restarts: no burst, no load, no automatic load, and every start is refused `no_link`. The host's bytes still arrive after a shutdown, so the link alone would not stop them | idle |
| 4 | `runout` | the insert sensor fell during any running mode except an unload and the follow (in a load: the filament withdrawn by hand; in the follow it starts the tail, 16). Also during an error, which it ends (below) | idle |
| 5 | `loaded` | a load's pull reached the parking distance. With the reader's field in the image, the event waits until the search and the return are over too; for the read with motion, until the filament is back where it started ("The load's search"). A gripped automatic load ends `loaded` once its pull reached `grip_um` ("The grip") | idle |
| 6 | `unloaded` | the insert sensor fell during an unload | idle |
| 7 | `stuck` | the filament does not pass ahead, in a mode other than a feed or a load's pull. (Those end `blocked`, 18, since v0.11.0. Up to v0.10.0 they also ended `stuck`: the lane's own buffer full with the filament lagging or the duty over its guard, or a second time in the mode, as in the tip snag above; and at once until v0.7.0.) The cases: a touch whose plunger does not come back to rest, or a second catch in the mode (in a rollback with `free_tip`, a catch short of `free_um` past the last); a comparator trip forward with the buffer full (the follow, the forward assist, a load's forward return); any comparator trip toward the spool, except the standstill at rest (13) and a tip snag's first forgiven one (a rollback with `free_tip`: each one forgiven); any comparator trip in a load's return or reacquire, the standstill at rest included (a filament that will not come back is held, not a tail past the drive gear; since v0.4.0). Example: a tip held in a hot end while a rollback grinds, the filament taut (measured on the unit: ≈ 20 mm of motor after the encoder stopped) | error |
| 8 | `tangled` | a comparator trip forward with the buffer not full, in the same modes as `stuck`'s forward trip (in a feed or a load's pull it is `blocked` since v0.11.0; in the tag search, its obstacle return). The filament does not come from the spool: a tangle, the end of the filament tied to the spool, a spool that will not turn (measured on the unit: 14.6 mm of slack, then ≈ 20 mm of motor with the encoder still) | error |
| 9 | `motor_stalled` | the tach silent for 1 s with the motor commanded (not provoked on the bench: that would mean holding a motor) | error |
| 10 | `timeout` | the lane's deadline (3/2 of the expected time plus 1 s) passed with the filament short of the length: no filament in the lane, or a slip past the comparator's allowance. A reverse move's deadline with the plunger at rest is the tail instead (13), and a deadline in the follow's tail once the end has left the encoder (20 mm since the tail began, the filament still (under 1.5 mm) while the motor turned at least 1.5 mm) is `tail_out` (17, since v0.13.0) | error |
| 11 | `assist_stall` | **reserved — superseded by `tangled`, never sent.** A forward assist's filament that is taut and still under command is the comparator's standstill (20 mm of motor after the filament stopped), and ends `tangled` in `assisting` (measured on the unit: the spool held while the filament was pulled taut). A timed rule for the same case came second at any burst speed above about 10 mm/s and was removed. The number stays so nothing shifts | — |
| 12 | `assist_overrun` | **reserved, not produced.** A burst that reaches its bound while the filament is still taut chains into the next burst: the head is pulling, which is no fault (decided at the bench). The number stays so nothing shifts | — |
| 13 | `unload_incomplete` | a reverse move (an unload, a rollback) ended with the filament still at the sensor and nothing left to drive it. Three cases: the tail is past the drive gear (a standstill, or the deadline, with the plunger at rest: nothing pulls on the buffer); the move went on for the tail's motor and the filament did not come off the sensor (100 mm for an unload, for the spool to wind the tail out; for a rollback, what its length still owed when the tail began: the length less the filament's travel on the encoder and less the motor turned since the filament last moved, the travel the encoder did not see. So a rollback never pulls more than its length, with one exception: when the encoder stops counting, the motor can still turn up to the comparator's standstill, 20 mm, before the move ends, as in a feed. Since v0.13.0; up to v0.12.0 a rollback had the 100 mm too); or the unload's budget was spent first. A hand is needed (measured on the unit: a lane whose spool does not wind the tail out) | error |
| 14 | `behind` | a notice; the lane goes on. A forward burst reached its 100 mm bound with the filament still taut: the head uses filament faster than a burst delivers it. Sent once per taut episode. It is not sent for the taut reading itself, which is a burst's normal trigger (decided at the bench: fourteen notices in a minute of ordinary pulling) | unchanged |
| 15 | `snag` | a notice; the lane goes on. A filament tip caught the tube's entry, and the tip snag's tolerance (above) let the move go on: the plunger came back to rest on its own in the watch, or after the touch. Carries the mode's odometers where the tip caught (the watch's start forward, the forgiven trip in reverse). Or a free tip's resistance proved: a window passed with the filament following, or the plunger back at rest ("The free tip", above). Once per mode at most | unchanged |
| 16 | `tail` | a notice; the lane goes on. The insert sensor fell in `following`: the filament ran out, or was cut, at the bay, and its tail still lies between the insert sensor and the drive gear. The follow feeds on as the tail ("The tail", below). Once per tail: the insert sensor rising and falling again during it changes nothing (since v0.11.0) | unchanged (`following`) |
| 17 | `tail_out` | the tail ended, with no error. Either a comparator trip in the tail (the standstill or the partial, whatever the buffer reads: the tail has left the drive gear and the filament no longer follows the motor), a move's deadline in the tail once the end has left the encoder (20 mm since the tail began, the filament still (under 1.5 mm) while the motor turned at least 1.5 mm; since v0.13.0), or `tail_um` of motor since the tail began (since v0.11.0). No motor attempt follows it: a filament put in at the bay during the tail starts no load (the insert sensor is already set, so there is no edge) | idle |
| 18 | `blocked` | a push toward the head met something that cannot move; no error. In `feeding` or a load's pull: a forward comparator trip (the standstill or the partial), or, in a feed without `free_tip`, this lane's full reading that the tip snag's tolerance does not forgive; in a feed with `free_tip` and in a load from the bay's pull, the filament falling more than `lag_um` behind the motor within a window while the plunger reads full ("Blocked", above). The filament waits where it stopped; a load's filament waits at the point it reached (since v0.11.0) | idle |
| 19 | `held` | a move toward the spool armed with the held stop (`ace2k_feed_stop_when … held=1`): the plunger stayed taut for `held_ms` while the filament moved less than `held_um`, or the comparator tripped with the plunger taut and the filament still. The tip is held ahead, for example in the head. No error (since v0.13.0) | idle |
| 20 | `at_rest` | a move armed with the at-rest stop (`ace2k_feed_stop_when … at_rest=1`): the plunger read rest, from the move's second tick on. No error (since v0.13.0) | idle |
| 21 | `stopped_sensors` | the insert sensors went stale: the ADC scan stopped, and their reading is frozen ("Stale insert sensors", below). A running mode could not see its filament run out, so it stops. This applies alike to a running move, an assist waiting between bursts, the follow and its tail, and a load in its settle or its search. A lane in error stays in error, with no event. No error (since v0.13.0) | idle |

**Kinds 7–13 put the lane in `error`.** The motor stops, the kind goes into the state report's
`error` byte, and every start on that lane is refused `in_error`. Two things end the error:

- `ace2k_feed_clear`;
- the filament leaving the insert sensor. The error was about the filament, and without a filament
  there is none. The lane goes idle with a `runout` that names the mode that failed and carries the
  failed start's sequence. (Decided at the bench: a filament withdrawn by the lever during a load's
  pull tripped `tangled` a moment before the sensor cleared, and the lane sat in error with nothing
  in it.)

Neither ends a mode, so the retry window ("The start's sequence") stays the one of the failure. Up
to v0.12.0 the filament leaving restarted that window, and the failed start resent within 1 s of the
`runout` read as its retry: accepted, with nothing started.

`ace2k_feed_stop` stops the motor in every mode and never clears an error. No event calls a Klipper
`shutdown`. The unit never retries a move on its own, except for the tip snag's one announced
tolerance per mode (v0.7.0). The events say why a length was not delivered, and the host decides
what happens next.

For readers who know the original Anycubic protocol:

- its `FEED_ERROR`, `ROLLBACK_ERROR` and `PRELOAD_ERROR` are `stuck` / `tangled`, with `mode`
  saying where. Since v0.11.0 an obstacle in a feed or a load's pull is `blocked`, no error;
- `STUCK_ERROR` and `TANGLED_ERROR` are the same two, now delivered instead of being overwritten by
  the mode's own code;
- `ASSIST_ERROR` is `tangled` in `assisting` (the comparator's verdict; `assist_stall` is reserved
  and never sent);
- `MOTOR_ERROR` is `motor_stalled` / `timeout`.

`runout`, `unloaded`, `unload_incomplete`, `behind`, `snag`, `tail`, `tail_out`, `blocked`, `held`
and `at_rest` have no counterpart. The original protocol has no clear command, and its STOP leaves
an assist's motor running. Here both do what they say.

### Stale insert sensors

The insert sensors are read from the ADC scan, every 10 ms. When the scan stops, its last pass
stays in place and only its age grows. Past 100 ms (`ACE2K_ENV_ADC_STALE_MS`, the bound that also
turns the NTCs and VDDA invalid) the insert readings are **unknown** (since v0.13.0):

- their filter and debounce stand still, and no insert event goes out. A spool that runs out
  meanwhile cannot show, so nothing may act on the frozen reading;
- every running lane stops with `stopped_sensors` (kind 21), and goes idle, with no error;
- every start is refused `sensors_stale` (reason 9), and so is the read with motion's search
  (its `ace2k_rfid_read` answers `busy`). No automatic load starts;
- the tag watch changes no lane's state and attributes no tag on the frozen reading, and a read
  on command answers `busy`. One accepted just before is answered at the next inventory, with no
  tag attributed (a read on command attributes none);
- the dryer's spool rule, which lowers the target when filament is put in with no host, sees no
  edge: the frozen mask neither rises nor falls. The rule only ever lowers a target.

A flash write (the link proof, the dryer's log, a config store) holds the core, and the scan with
it, for tens of ms, without stopping the scan. The pass from before the write can pass the bound
on the tick's catch-up, and for the two periods the next pass takes. So for 50 ms after a catch-up
(the settle that health bit 25 waits too) a stale pass keeps the last verdict: readings known
before the write stay known, on the values from just before it, and nothing stops. A scan still
stopped after those 50 ms goes unknown as above. The auxiliary input (`aux_mv`) is taken from the
same passes as the insert readings, so a frozen pass does not refresh it either.

When fresh passes come back, the insert filters start from the first one, not from the frozen
value. An edge is judged only between two known readings, so the frozen value and the first fresh
one never make a load or a runout by themselves. A real change since then makes its event as
always, two ticks later. Nothing that stopped starts again on its own. The health bit 25
(`sensors_fresh`) shows the stall from 50 ms. A stopped scan cannot be provoked on the bench without
a debugger; the firmware's tests cover it (`tests/lane/test_sensors.c`,
`tests/feed/test_feed_sensors.c`, `tests/rfid/test_rfid_watch.c`).

## The start's sequence

The host numbers its starts per lane, from 1 to 255 and back to 1. Every event of the mode a start
entered echoes its `seq`. So a waiting G-code completes on its own event, and a late event of an
earlier move passes by. 0 is a motion the unit started itself (the automatic load), never a host's.

The state report's `seq` is the last start the unit accepted from the host on that lane. It is kept
through idle and error. A lane running a host start's move reports that start's `seq`; a mode of
the unit's own leaves it as it was. A host that reconnects seeds its counter from it before its
first start of the session. That way its first start never repeats a move the unit still runs or
has just ended. The host waits for the session's first state report (up to two periods) before it
starts anything.

Klipper sends again a query whose response it did not get. A start that repeats the lane's last
accepted host start whole (its `seq`, mode, length and speed) is that retry, not a second command.
It is answered `accepted=1` and nothing starts, while the mode that start entered still runs, or
within 1 s after it ended (done, stopped or an error; a lane in error keeps the window, and a clear
does not restart it). The host's completion is served by the event that went out, or goes out
later. The same `seq` with anything else changed is a new command and takes the normal path: it is
refused `busy` while the lane runs. A mode of the unit's own, running or ending on the lane, neither
closes nor re-opens the window. This is what makes a lost start response harmless: the response is
a plain send, dropped against a full transmit buffer ("Reports"), and the retry is accepted as the
same command.

## A stop armed for the next start

Since v0.13.0 a move can stop on the buffer's plunger, in the unit, every 10 ms. The host sends
`ace2k_feed_stop_when` and then `ace2k_feed_start`. The stop belongs to that one start:

- The next `ace2k_feed_start` on the lane takes it, whatever its answer. A start that is refused,
  or that is the transport's retry of the last start, drops it. A start that is accepted keeps it
  for the mode it enters, until that mode ends.
- The start must fit the stop. `held` goes only with a rollback or an unload, and `at_rest` with
  every mode but the follow (`assist_both`). A start that does not fit is refused `bounds` before
  anything moves: the stop is dropped as by any refused start, and the lane's grip stays. A load
  judges held on its return and its reacquire, toward the spool in the middle of its search, so a
  held tip there would end it idle with the search half done. The follow keeps the plunger at rest,
  so `at_rest` would end it on its second tick. The host refuses both before it sends anything, so
  only a host with a bug meets this refusal.
- A mode the unit enters on its own (the automatic load) neither takes nor uses it. It stays armed
  for the host's next start.
- A lane that is not idle refuses it (`accepted=0`, with the reason `busy` or `in_error`), so a
  stop is never armed under a running move.
- A tick with the link down clears an arming that no start took. The host sends
  `ace2k_feed_stop_when` right before its `ace2k_feed_start`, and `held=0 at_rest=0 held_ms=0` on
  any path that arms and then does not start: an arming left behind would stop the lane's next
  move of any mode.

**Held.** A taut plunger alone is not a stop. A thick tip dragged through a narrow tube pulls the
plunger taut while the path ahead is free, and the filament still comes. So the unit opens a
window when the plunger reads taut, and closes it when the plunger leaves taut. A filament that
moves `held_um` (3 mm) or more since the window opened starts the window again. When the window
has lasted the move's `held_ms`, the move ends `held`. Each arming carries its own `held_ms`, and
0 there means the unit's (550 ms unless `ace2k_feed_held_set` changed it). So a fast move and a
slow one on the same unit each keep their own window. The pull against a held tip is about the
motor's speed times the window, so a host picks a shorter window for a faster move, while the
friction floor below stays under that move's speed. A comparator trip that comes before, with the
plunger taut and the filament still, ends the move `held` too: the standstill, or the partial
when the filament has moved less than 1.5 mm over at least `slip_allow_um` of motor. A partial
that trips sooner, because the filament trailed the motor before it stopped, waits: the unit
asks again on the next ticks, for at most `slip_allow_um` more motor. A filament that still
comes but slips keeps the comparator's verdict, `stuck`, at most `slip_allow_um` of motor later
than unarmed.

Held is judged only while a move toward the spool runs. A start takes it only on a rollback and
an unload, the modes that run that way from start to end, and every other mode refuses it
(above). A load's return runs toward the spool too, but in the middle of the load's search. The
reverse assist ends its take-up as soon as the plunger reads taut, and then waits with no move
running, so held could never end it. Toward the head (a feed, an assist, the follow, a load's pull)
a taut plunger with the filament still is a tangle, not a stop. A rollback's touch forward during
the tip snag is not judged for held either. At rest is judged in either direction. Friction is
told from a held tip only above the floor speed given under `ace2k_feed_held_set`, worked out with
the move's own window; with the defaults that is below the lane's least speed.

**At rest.** The plunger reading rest ends the move `at_rest`. The move's first tick is not
judged. So a move that starts with the plunger already at rest runs one tick (10 ms), and ends on
the next one if the plunger still reads rest.

The stop is judged after the link check and before the autonomous steps, the tip snag, the tails
and the comparator. A held tip therefore ends `held` and never reaches `stuck`, at any speed. Both
kinds leave the lane idle, with no error, and carry the mode's odometers like every final event.
A move that is not armed runs exactly as before.

**Before the reverse tip snag.** An armed stop goes before the reverse tip-snag tolerance ("The
tip snag", above). A rollback whose tip catches the tube after the filament came back freely
ends `held`, where an unarmed one would make its touch forward and resume. An armed `at_rest`
ends the move when that touch brings the plunger back to rest. The final event's `filament_um`
tells the cases apart: a tip held at the head comes back no more than the buffer's slack, and a
catch after free travel comes back at least `free_um`.

## The shared full input

The four buffers' pulled-end switches are wired-OR onto one input (`hardware.md`, "Digital
inputs"). So the unit knows that *some* plunger is at its pulled end (a buffer is full), but not
which one. Two rules read this input in two ways:

- **The safety stop.** A forward burst (the forward assist's or the follow's) does not start while
  any lane reads full, and stops on that reading. A plunger the unit cannot attribute may be this
  lane's, and a burst into a full buffer compresses the filament.
- **This lane's full.** Everything that must act on *this* lane's full reading reads the shared
  input together with the lane's own rest and pushed switches. That is: the tip snag's watch of a
  feed, a load's pull and its tag search; the take-up of the reverse assist and of the follow; and
  the comparator's choice between `stuck` and `tangled`. A plunger at rest or at its pushed end is
  not the one at its pulled end. So for such a lane the full reading belongs to another lane, and is
  no verdict on this one.

The consequence: with two lanes bursting forward at once (the forward assist or the follow, on
either lane), one lane's full plunger pauses the other lane's bursts until it clears. The restart
is per lane, from its own taut reading. One lane assisting, which is the printing case, is
unaffected. Observed on the unit: another lane's plunger held at its pulled end did not stop this
lane's load. The assist side of this coupling is documented, not exercised: it takes three hands.

Reading the lane's own switches rules out a plunger at rest or taut, and no more: the shared input
cannot name the lane. A plunger between its switches reads neither. So with two lanes moving
filament at once, another lane's full reading looks like this lane's own while this plunger is
mid-travel, and every reader of *this* lane's full reading takes it as such:

- In the reverse assist and the follow (below), it may start one take-up here. That take-up is
  bounded like any other, ends when this lane reads taut (or rest), and the comparator judges it as
  any take-up.
- In a feed, a load's pull or its tag search, it is the full reading ahead, and starts the tip
  snag's watch (a free tip's windows: a feed with `free_tip`, a load from the bay).
- A forward comparator trip on this lane, in a mode that still judges it, is judged `stuck` instead
  of `tangled`: the comparator chooses the kind from the same reading. (A feed or a load's pull ends
  `blocked` either way.)

A lane at rest or taut is never affected. With one filament moving at a time, as on the U1, the
case does not arise.

## The follow

Since v0.8.0. The follow is the third assist. `ace2k_feed_start` mode 6 (`assist_both`; the G-code
`ACEK_ASSIST … DIR=BOTH`) arms it, and the lane is then in mode 8 `following`. It keeps the buffer
at rest whichever way the head moves the filament. It has one arm, the cycles of the two assists,
and one speed (the armed one) for both directions. It is made for a head that pushes **and** pulls
within one phase, such as a printer's tip forming, tens of millimetres forward and back. The
forward assist never takes up, the reverse assist never feeds, and the filament cannot turn the
motor backwards. So between moves the plunger's travel is the only give. The two assists are
unchanged.

**The cycle.** While it waits, the lane reads its plunger every tick:

- *Taut*, with the shared full not set, starts a forward burst of at most
  `ACE2K_FEED_ASSIST_BURST_UM` (100 mm), like the forward assist's.
- *This lane's full* ("The shared full input") starts a take-up of `take_um` (15 mm).
- *Rest* starts nothing: it is the target.
- *Taut* while the shared full is set is a pause. The burst starts from this lane's own taut
  reading once the full clears.

A burst ends at *rest* or on the shared full (the safety stop). A take-up ends at *rest* or *taut*.
Neither end is an event. A move that reaches its bound with its reading still there chains into
the next move in the same direction on that tick, as the two assists' moves do. The `behind`
notice and its taut episode are the forward assist's. Arming runs the first step at once on the
last tick's readings, as the two assists do: taut and no full starts a burst; this lane's full
starts a take-up; otherwise the lane waits for the head's first pull or push.

**Rule 1, no hunting.** After a move of the follow ends, a move the other way starts only once the
lane has read rest, or once `flip_ms` (200 ms) has passed since that move ended, whichever comes
first. A move in the same direction may start at once. A move ends where its reading ends it, at
its bound, or (a take-up only) at the comparator's standstill with the plunger at rest (below). At
arming, no direction is held.

**The shared full** stays the safety stop: no burst while it is set, and a running burst ends on
it. A take-up starts only on this lane's full reading, so another lane's full never starts one on a
lane at rest or taut. On a lane mid-travel it may start one ("The shared full input", above).

**The comparator** judges each move by the direction it runs. A burst is a forward move: a trip
with this lane's full is `stuck`, without it `tangled`. A take-up runs toward the spool: a trip is
`stuck`, except the standstill with the plunger at rest. That one is no fault and no tail: the
filament came back to rest, so the take-up ends and the lane waits, as the reverse assist's does.
That end counts as the pass through rest for rule 1. The tip snag's tolerance does not apply, as in
the two assists.

**It runs until the host stops it** (`ace2k_feed_stop`, `stopped`), or until one of these ends it:
the end of its tail (`tail_out`, below), an error kind, the link (`stopped_link`) or a shutdown
(`stopped_shutdown`). It is a feed mode wherever the two assists are: a start on the lane is
refused `busy`, and so is a counters reset.

**The tail** (since v0.11.0). The insert sensor sits at the entrance of the bay, and the encoder at
the drive gear. A filament that runs out at the bay, or is cut there, still runs through the drive
gear until its end (the *tail*) passes it, and the head goes on pulling it. Up to v0.10.0 the
follow ended on that `runout`. That left the drive gear holding the tail (the motor cannot be
turned from outside), while the head's own sensor still saw the filament. So in `following` the
insert sensor falling does not end the mode. The lane enters its **tail**, sends the `tail` notice
(16), and stays in mode 8. In the tail:

- the follow feeds as before: its forward bursts on taut, and the feed-forward's doses. Neither
  reads the insert sensor (`no_filament` refuses a host's start, never a move the lane starts
  itself). The shared full still pauses both;
- **nothing is taken up**. This lane's full starts no take-up, because a take-up would pull the
  loose tail back out of the drive gear's reach. A take-up running when the tail begins is stopped
  at once (a take-up's end is no event). The tip snag's tolerance stays out, as in the plain follow;
- **every comparator trip is the tail-out**: the standstill (`stall_check_um`, 20 mm of motor
  without the filament moving 1.5 mm) as well as the partial, whatever the buffer reads. The tail
  has left the drive gear, and the filament no longer follows the motor. The lane goes idle with
  `tail_out` (17): no error kind, nothing to clear. Past the drive gear the filament is free in the
  tube, pulled by the head alone. A move's deadline in the tail is the same end, `tail_out`, when
  both hold: the filament's net travel on the encoder since the tail began is at least 20 mm
  (`ACE2K_FEED_TAIL_ENC_UM`; on the unit the end left after about 35–37 mm), and through that move
  the filament stood still (under 1.5 mm) while the motor turned at least 1.5 mm. A short forward
  move, such as a dose of the feed-forward, never reaches its length once the end has left the
  encoder. Otherwise the deadline stays `timeout`: a filament that stops before 20 mm is a slip or
  a lost encoder upstream of the end, and one that still counted through the move is not the end
  leaving (since v0.13.0; before, every deadline in the tail was `timeout`);
- **it is bounded**: `tail_um` (2 000 mm, `tail_max`) of motor since the tail began also ends it as
  `tail_out`. This covers a head that pulls all along: the filament follows the motor and never
  trips the comparator. `ace2k_feed_stop`, the link and a shutdown end it as they end the follow;
- **it ignores the bay**. The old filament's end holds the drive gear until the tail is out. So a
  filament put in at the bay meanwhile rests behind it and can never be gripped (observed on the
  unit: with a filament put in during the tail, the follow on the resumed print ended `tangled`).
  The insert sensor rising during the tail changes nothing, and neither does it falling again: no
  second notice, no return to the plain follow. The tail runs until its tail-out, `tail_um`, a
  STOP, the link or a shutdown;
- **no motor attempt at the tail-out**: the lane goes idle. A filament put in during the tail, its
  insert sensor already set, gets no automatic load, because that starts on an insert edge only.
  Pulled out and pushed in again (a fresh edge, the gear free), it loads as any filament.

No setting turns the tail off. The two assists still end on `runout`.

**The run-time values**, `flip_ms`, `take_um` and `tail_um`, are `ace2k_feed_follow_set`'s
(above). They come from the `[ace2k]` keys `follow_flip_ms`, `follow_take_mm` and `follow_tail_mm`
(200–2 000 mm, since v0.11.0), and from `ACEK_FOLLOW_SET FLIP_MS= TAKE_MM= TAIL_MM=` (`README.md`).
Their defaults are initial values for the bench.

### The follow on by default

Host only (`klippy/extras/ace2k_follow.py`, since v0.12.0): the unit arms nothing by itself. With
the `[ace2k]` key `follow_default` (`True` by default) the host starts the follow
(`assist_both`, at `assist_speed`) on its own:

- **at the start**: on the first state report after `klippy:ready`, every lane whose insert sensor
  is set;
- **at an insertion**: the insert sensor clear, then set, while the lane is not in the follow's
  tail. The host waits two state reports first, so that the unit's own load (`auto_load`), which
  starts at the insert edge, shows in a report before anything is sent.

Per lane this is an episode. The host starts the follow only on a lane the state report just
received shows `idle`, with no error, with the insert set and no start in flight on it. A `busy`
refusal (the report was behind the unit) is tried again at a later report. The episode ends when
the follow is armed, on any other refusal or no answer (one console line), on a lane found in
error, when the insert clears, and on any start or stop on the lane that is not the default's own:
a G-code, a host module's call, a read with motion. So the default arms a lane once per start or
insertion, never under a move someone else started, and never after `ACEK_STOP` or
`ACEK_ASSIST … OFF=1` until the filament is taken out and put back. A module that runs its own
flow on an insertion takes the lane's insertions with `set_lane_follow(lane, on_insert=False)`
("Host interface", below). `follow_default: False` arms nothing.

**A G-code move on a lane in the follow** (`ACEK_FEED`, `ACEK_ROLLBACK`, `ACEK_LOAD`,
`ACEK_UNLOAD`, a one-way `ACEK_ASSIST`, the hand calibration feed, `ACEK_RFID_READ … MOVE=1`) stops
the follow (a host stop: its `stopped` event carries `stop_by` `host`), waits up to 5 s for that
event, retries the unit's `busy` while the lane winds down (up to 5 s, every 0.25 s), then starts.
Whether a follow runs is read from the host's own record, not from the 1 Hz report alone: the last
start the unit accepted and whether its final event came, and whether any final event came since
the last report. A follow whose end already came (`done`, `tail_out`, an error) is nothing to
pause, and one in its tail is left alone. A follow that does not end within the 5 s is an error
(`ace2k: lane N feed not started: the follow was stopped but did not end within 5 s; it comes back
once the lane is idle`): the move is not started and the follow is given back as after a refusal.
A one-way `ACEK_ASSIST` replaces the follow: it stops it and never re-arms, as an unload. The
re-arm still pending is dropped when the insert clears, on any stop of the lane, and when the unit
accepts another start on it. The follow is armed again at the next state report showing the lane
idle after the move's own final event — the event of the move's seq, registered before its query; for a read with motion,
the unit's load it runs (seq 0, `loading`) — when that event is `done` or `loaded`, the insert is set, `follow_default` is on and the lane
is eligible (below); after an error, `blocked` or `stopped` it stays off, one console line. An
unload never re-arms. A move the unit refuses gives the follow back. Host-module starts
(`start_move`) are never paused for: on a lane in the follow the unit refuses them `busy`, as
before.

**One active lane per extruder.** Lanes whose extruder (`laneN_extruder`, or `set_lane_extruder`)
is the same form a group; the groups are kept per extruder. In a group only the lane chosen with
`ACEK_LANE_ACTIVE` is eligible for the default and its re-arm; none is at the start, and a console
line asks for the choice once per group. The choice is not saved. A group whose lanes change
(`set_lane_extruder`) is a new group: its line is said again, and its choice is kept only while the
chosen lane is still in it (a lane mapped out and back is not chosen again); an unchanged group
keeps both. A lane alone on its extruder, or
without one, is always eligible. The lane's `active` in the status is `False` for a lane of a group
that is not the chosen one, and `printer.ace2k.active_lanes` maps each shared extruder to its active
lane (a number, or `None`).

## The feed-forward

Since v0.9.0. With the follow alone, a lane feeds only once the head has pulled the filament taut:
a burst, then nothing until the next taut reading. Between bursts the filament is stretched. The
feed-forward has the lane meter out what the head is about to use, ahead of the pull, so the
plunger stays near rest. The follow stays on top of it, as the correction. A print uses about
1–6 mm/s of filament, under the motor's floor (`ACE2K_LANE_SPEED_MIN_UM_S`). So the lane cannot
run continuously at that rate: it **meters**, in bounded doses. The host works out the rate from
the moves Klipper has planned for the extruder the lane feeds, and sends it with `ace2k_feed_base`.
The unit does the rest on its 10 ms tick. It acts only on a lane in mode 8 `following`. Arming
starts at a base rate of 0, and every end of the mode (a stop, an error, the link) returns it to 0.

**The debt.** Each tick of a lane in the follow with a base rate adds the rate × the time since the
last tick to what the lane owes the head: 10 ms, or more for a tick that came late, at most 1 s (up
to v0.12.0 always 10 ms, so a late tick owed less). The sub-micrometre remainder is carried, so a
slow rate is not lost to the tick. The debt is capped at two doses (`2 × chunk_um`), so a wrong rate
never builds up into a long feed.

**The dose.** A dose starts when the debt reaches `chunk_um` (3 mm) and all of these hold: no move
is running, the plunger is not taut, no lane reads full (this lane or another), and rule 1 allows a
forward move ("The follow"). The lane then starts a forward move of `chunk_um` at `pulse_um_s`
(15 mm/s). A dose starts at rest too, where the follow itself starts nothing: it is metered ahead of
the pull. It ends at its chunk, on the shared full (the safety stop, as for a burst) or on taut;
never on rest. When it ends, the filament's travel on the encoder pays a debt of its own direction,
floored at 0. Only a dose stopped once a chunk was owed the other way books its travel against
the debt's direction, as is, capped at two chunks: the lane then owes that much more the other way.
Any other dose's travel against the debt's direction is not booked. The
comparator judges a forward dose as a burst (a forward move), and a reverse dose ("A long retraction
is taken back", below) by the motor's direction as a reverse move, as it judges the follow's
take-up. It raises no `behind` notice, and `bursts` does not count it.

**The corrections** are the follow's, unchanged:

- *Taut* (the filament short of the head) zeroes the debt and starts the follow's burst. A running
  dose ends on it, and the burst starts on the same tick.
- *This lane's full* (too much filament) zeroes the debt and starts the follow's take-up. No dose
  runs meanwhile.
- *The shared full* pauses the doses, as it pauses the bursts.

Rule 1 holds for both. A base rate of 0 stops the accrual and nothing else. The debt is filament the
head has already pulled, so it is still delivered, in whole doses under the same conditions. A
residual under one dose stays owed until the next accrual pays it with a dose. A lane never given a
rate has an empty debt, and is the plain follow.

**The counters**, per lane since its last entry into a mode: `doses` started, `taut` corrections
and `full` corrections. A correction is counted once per episode, when its burst or take-up
actually starts (a taut reading held under another lane's full counts once its burst starts). They
travel in their own frame, `ace2k_feed_ff_state`, with each lane's `epoch`. The epoch steps
(wrapping) at every entry into a mode, from a host start or the unit's own (the automatic load
included), and that is when the counters reset. Like `bursts`, the counters are kept after the mode
ends. The host restarts its tally exactly when a lane's epoch moves. A session's first frame is
only its baseline: after a Klipper restart, the totals from before it say nothing.

**The miscalibration signature.** A dose's length is the filament's travel on the encoder. So a
wrong encoder scale makes every dose wrong by the same factor. If the lane meters short, the head
keeps pulling it taut; if it meters long, its plunger keeps reaching full. The follow absorbs the
error, so the lane still works, and the host reports it. Every `ff_notice_every` (20) corrections
in the same direction since the last notice, it prints a console line. Corrections in both
directions cancel. They are counted only while the lane is metering: its feed-forward on, with a
rate above 0 sent (the plain follow corrects on every taut reading). The line:

    ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale (ACEK_CALIBRATE_ENCODER)

(or `full corrections`). A steady stream of them in one direction means the scale is off. Run
`ACEK_CALIBRATE_ENCODER`; its automatic mode (since v0.10.0) reads the scale off the lane's extruder
(`README.md`, "Calibrating the encoder"). Nothing corrects the scale on its own.

**The host's side** (`klippy/extras/ace2k_ff.py`):

- **The map.** `[ace2k]` `lane1_extruder` … `lane4_extruder`, or `set_lane_extruder` ("Host
  interface", below), names the extruder each lane feeds; a key wins. Several lanes may name one
  extruder. A lane without one has no feed-forward.
- **The hook.** At ready, the module wraps each mapped extruder's `process_move` (newer Klipper)
  or `move` (older trees, the U1's among them). The original runs first. The wrapper records each
  extruding move's print time, duration and extruder distance, and never raises. An extruder
  Klipper does not have, or one with neither method, gets a console line (`ace2k: lane n: no
  extruder <name> — no feed-forward on it`), and that lane has no feed-forward.
- **The rate.** Every `ff_period_ms` (100), the tick's share is the signed distance the recorded
  moves cover in `[now + ff_lead_ms, now + ff_lead_ms + ff_window_ms]` (print time; 0 and 500 ms
  by default). Each move counts pro rata of its overlap, times the print time since the last tick
  divided by the window. The window slides that much each tick, so the shares add up to the net E.
  A tick that the reactor ran late sweeps the time it missed, in steps of one period. A window
  shorter than `ff_period_ms` counts as one period: otherwise the moments between two ticks'
  windows would never be metered.
- **Retractions** are credited against the next extrusion, not lost. A negative share adds to the
  extruder's credit (at most 50 mm), with a rate of 0. A positive share pays the credit back first,
  off the window's mean, which is the rate. The credit is dropped when no lane of the extruder
  follows, or two do; when its lane in follow is switched off, held or remapped; and at a shutdown.
- **Sending.** The rate goes to the one lane of that extruder in `following`, when it differs from
  the last rate sent by more than `ff_deadband` (0.3 mm/s), rises from 0 or reaches 0. (A slow
  print's rate under the deadband is still sent.) Only the lane in follow is fed.
- **Two lanes in follow.** Two or more of the extruder's lanes in follow at once is ambiguous.
  Neither gets a rate. Each lane is sent 0 with `clear=1` at the episode's start (and again if a
  lane was given a rate since), so what it owes is dropped. The module prints one console line per
  episode (`ace2k: extruder <name>: lanes 1 and 2 both in follow — no feed-forward`). Each lane
  keeps the plain follow.
  A lane counts as in follow while its last report says `following` and no final event of that
  motion has come since; across units the coordinator counts the other units' lanes the same way.
- **Calibration.** An automatic encoder calibration (since v0.10.0) holds its lane for the run. The
  lane gets no rate and is not metering, so its corrections count toward no notice. What it owes is
  dropped at the hold (`clear=1`). The `ACEK_FEED_FORWARD` switch is untouched. The release sends
  nothing itself: the next tick works out the rate as usual.
- After a Klipper or unit shutdown the host sends nothing.

**A long retraction is taken back** (since v0.12.0). A print's retractions (≈ 0.5–7 mm) fit in the
buffer (rest → full ≈ 18 mm) and are credited. An unload's tip forming pulls back 30–35 mm at up to
55 mm/s, which fills the buffer in under half a second. A contiguous retraction — recorded moves
with E < 0 not broken by one with E > 0, nor by more than a window (at least a period) with none —
of `ff_retract_min_mm` (10 mm) or more is not credited: its mean over one period (`ff_period_ms`)
from now, not led by `ff_lead_ms`, the retraction's own rate, goes out as a negative base rate;
everything else is still the window's mean. Wherever the extruder's credit is dropped (above), its
open retraction ends too, and a retraction after it starts a new one.
Each recorded move is classed, long or not, the first time the window or the long part's period
reaches it, and counts in that part only, whatever the window, the period and the lead: a move
already credited stays credited when its retraction becomes long later, and no move is both
credited and taken back.
The unit accrues a negative debt and, at one chunk owed back, runs a **reverse dose** of one chunk
at the rate's speed, never under `ff_pulse` nor over the lane's top speed. Since v0.13.0 a reverse
dose has no landing: it runs at that speed to its end, so that it keeps up with the retraction. Its
stop is not a target, because the filament's real travel is booked against the debt. A forward
dose lands as every move of 10 mm or less does, so it runs at the floor speed. A reverse dose ends at
its chunk or at once on taut (the head holds the filament); rest does not end it. A dose either way
is stopped once a chunk is owed the other way, so a forward dose of the extrusion before does not
hold back the first reverse one. None starts in the tail. Taut or this lane's full zero the debt
either way, and the follow's own correction takes over. The reverse doses are counted in
`ace2k_feed_ff_state`'s `back`. The notices count corrections only while the rate is above 0, at the
counters frame's resolution (1 Hz): a frame counts if a rate above 0 was in force at some moment of
its interval.

**An older image**, without `free_tip` on `ace2k_feed_start` (since v0.12.0): a host with interface
12 refuses it at connect (another format). Reflash the unit.

**An older image**, without the constant `ACE2K_FEED_FREE_ROLLBACK` (since v0.12.0): its rollback
ignores `free_tip` and forgives one catch, as before. The host connects; `ACEK_ROLLBACK FREE=1`
answers `ace2k: the unit's firmware ignores FREE on a rollback; flash v0.12.0`, nothing sent, and
`feed.has_free_rollback` is false.

**An older image**, without the feed-forward's constants or whose `ace2k_feed_ff_state` has no
`epoch`, has no feed-forward. Nothing is sent to it, and `ff_chunk_mm` and `ff_pulse` are left
unused. `ACEK_FF_SET` and `ACEK_FEED_FORWARD` answer `ace2k: the unit's firmware has no
feed-forward; flash v0.9.0`. With `feed_forward` on, the same line is printed once on the console at
ready.

**The run-time values.** The unit's, `chunk_um` and `pulse_um_s`, are `ace2k_feed_ff_set`'s (above),
from the `[ace2k]` keys `ff_chunk_mm` and `ff_pulse` (`ff_pulse` is retired: the next version
removes it, and the host then sends the unit's own `pulse_um_s`). The host's are `ff_period_ms`, `ff_window_ms`,
`ff_lead_ms`, `ff_deadband`, `ff_retract_min_mm` and `ff_notice_every`. `ACEK_FF_SET CHUNK_MM=
PULSE= WINDOW_MS= LEAD_MS= DEADBAND= RETRACT_MIN_MM=` changes them live (a new lead drops each
extruder's sweep, credit and open retraction, so the next tick starts afresh). `ACEK_FEED_FORWARD
[LANE=] ON=1|OFF=1` switches the feed-forward (`README.md`). The defaults are initial values for the
bench.

## Tag states

Since v0.4.0 the unit watches the tags of its four lanes (ADR 0012: the transport in the MCU, the
brands on the host). Each lane has a tag state. It appears in the state report's `state` bytes, in
`ace2k_rfid_lane_state` and in the events:

| State | Name | Meaning |
|---|---|---|
| 0 | `unknown` | no filament at the insert sensor |
| 1 | `pending` | a filament is in, its tag not read yet — watched on every move of the lane |
| 2 | `searching` | the feed's load runs on the lane (its settle, its pull, its search, its return), the tag not read yet |
| 3 | `reading` | a session with the host is open on the lane ("RFID sessions") |
| 4 | `read` | a tag is attributed to the lane and the host confirmed its read; its UID is kept |
| 5 | `no_tag` | the load's search ran its whole reach, no UID was attributed to the lane during this insertion, and no UID but the neighbour's was in view during the search |

How a lane moves between states:

- A filament at the insert sensor, inserted or present at boot, makes an `unknown` lane `pending`.
- The sensor falling returns any lane to `unknown`. Its UID is forgotten, and its session, if one
  is open, ends.
- A load starting on a `pending` or `no_tag` lane makes it `searching`. The load's end makes it
  `pending` again, or `no_tag` when the search ran its whole reach, no UID was ever attributed
  to the lane during this insertion, and no UID was in view during the search but the neighbour's
  (rule 1, or one that the neighbour's reader waits for to open a session). **A lane that saw a tag is never `no_tag`.** Since v0.13.0 that includes a UID seen
  but not attributed, such as a spool at rest on the other bay; up to v0.12.0 such a search ended
  `no_tag`.
- `ace2k_rfid_forget` and the read with motion return a `read` or `no_tag` lane to `pending`.
  On a `pending` lane, the forget drops a kept UID and leaves the state as it is. After it, the
  lane counts as one that saw no UID during this insertion. On a `searching` lane it does nothing,
  so a search that saw its tag still never ends `no_tag`.
- A `no_tag` lane is not watched. A later load, a read on command or a forget looks again.

Every change is an `ace2k_rfid_event` with the new state as its `kind`, except the opening of a
session, whose notice is its `ace2k_rfid_tag`. `kind` 6, `ambiguous`, is never a state. It is the
answer of a read on command that saw a UID it could not attribute, and it changes nothing.

**The field.** A lane is *watched* while it is `pending` or `searching` and its motor runs or its
load runs. A reader's field is on while one of its two lanes is watched, a session is open on it, a
read on command waits on it, or a lane's attributed tag waits for the reader's session. Otherwise it
is switched off and **read back off** (both antenna drivers disabled; `hardware.md`, "RFID
readers"). A field that reads back on latches the reader's field fault (the state report's `field`
bits 0x04 / 0x08, the health bit 4 or 5) until the MCU restarts. Since v0.13.0 the unit then
switches the field off again at every task run until it reads back off; until then, the state
report's `field` bit 0x01 / 0x02 stays set, because the field is still on (before, it was cleared at
the first switch-off). A field on for 120 s, with no lane of its reader moving and no session, is
switched off. It stays off until a lane of the reader moves or a read on command asks. Before it
switches a field on, the unit configures the reader: a soft reset, and the settings the reset
clears. It does so again after any soft reset since (the health probe's). Since v0.13.0, a reader
found reset while its field is on gets no exchange: its field is switched off, and the next
switch-on configures it again. An exchange that such a reset lands in is not trusted: an inventory's
result is dropped, and a host step ends with status 3 `error` and no data ("RFID sessions"). A
configuration that fails is not tried again before 1 s. Three failures in a row make the reader
**dead**: no field, every read on command answered at once, the state report's `field` bit 0x10 /
0x20 and the health bit set. This lasts until `ace2k_health_run` (`ACEK_HEALTH RUN=1`) or a restart
gives it another try.

**Watching and attribution.** A reader with its field on takes an *inventory* every 50 ms: every tag
in the field, at most four, by ISO 14443-A anticollision, each tag halted once found. At this period
every passage of a tag at 30 mm/s was seen, on Anycubic and Bambu Lab spools. The first inventory
runs 10 ms after the field comes on (the tags' power-up). The first 3 inventories
(`ACE2K_RFID_BASELINE_POLLS`) are the baseline, failed ones included: a tag seen in any of them was
there at rest (since v0.13.0; it was the first inventory alone, which can miss one of two tags in
view). A resting tag that all 3 miss can still be taken for one that enters, the same margin as the
re-entry below. Two lanes share each antenna (`hardware.md`, "RFID readers"). Take a lane *m* whose
neighbour on the same reader is *n*. A new UID is given to *m* only when it is read while *m* moves;
a lane that has not moved stays `pending`, whatever *n* holds (since v0.13.0):

1. a UID attributed to *n* during this insertion (`read`, `reading`, kept from a session that
   ended with *n* `pending`) is never *m*'s, even when it enters while only *m* moves (rule 4).
   *m* goes on to the next tag of the same inventory (since v0.13.0; up to v0.12.0 only a tag
   `read` or `reading` counted here, and for a kept UID the anticollision's order decided which
   lane took it). A kept UID is held until *n*'s filament leaves its insert sensor, or until
   `ace2k_rfid_forget` on *n* while *n* is `pending` (since v0.13.0; before, the forget left a
   kept UID in place). After that forget, *n*'s tag counts as an unknown tag, and rules 2–5 apply
   to it as to any other, so it can enter while only *m* moves and be *m*'s;
2. *m*'s own UID, once known during this insertion, is *m*'s;
3. removed in v0.13.0. Up to v0.12.0, with no filament at *n*'s insert sensor, any UID in the
   field was *m*'s. An empty insert sensor does not mean an empty bay: a spool can sit on *n*'s
   holder with its filament not in yet, and *m* took that spool's tag;
4. a UID that **enters** the field while *m* moves and *n* does not is *m*'s. A UID never seen in
   this field enters at its first inventory after the baseline; one that was seen re-enters only
   after 3 inventories in a row without it (`ACE2K_RFID_DEBOUNCE_POLLS`, 150 ms, since v0.13.0). A
   UID that stays in view through the whole motion is nobody's;
5. with both lanes moving, a UID that enters is *m*'s only when *n*'s tag is `read`.

Anything else is ignored. No UID is attributed without a link. A UID attributed while the reader's
session is busy with the other lane waits for that session to end.

**A pending lane on the move.** In any feed mode except the load (a feed, a rollback, an assist), a
`pending` lane is watched, and its tag is read on the fly, without stopping the motor. A session
that the tag outruns ends with the lane `pending`. The load stops for its session ("The load's
search").

## RFID sessions

A tag attributed to a lane that is `pending` or `searching` opens a **session** with the host. The
unit sends `ace2k_rfid_tag` (the lane, a session number from 1 to 255 and back to 1, never 0, the
UID, the ATQA and the SAK), and the lane becomes `reading`. **One session per reader at a time.** A
tag attributed to the other lane of the reader waits for the session to end. A new session never
opens while an exchange still runs on the reader (a step of the session that just ended).

**The steps.** The host answers with `ace2k_rfid_step`s, one at a time. Each step names an
operation, never a raw frame: read up to three groups of four NTAG pages, or authenticate one
MIFARE Classic sector with the key the step carries and read up to three of its data blocks. The
unit runs a step as one job on the reader: wake the tag (`WUPA`), select it, authenticate, read. A
job that read all its blocks ends with `HLTA`, so the tag is halted and the next step's `WUPA` wakes
it. Every 16 bytes read are answered with one `ace2k_rfid_data` frame, status 0, with `block` the
MIFARE block (sector × 4 + its index) or the first of the four NTAG pages. A step that fails sends
the frames of what it did read, then one frame with `data` empty, `block` the step's first block or
page, and the status:

| Status | Name | When | The session after |
|---|---|---|---|
| 0 | `ok` | 16 bytes read | open |
| 1 | `auth_failed` | the key did not open the sector. A silent authentication counts as a wrong key: a MIFARE Classic tag does not answer an authentication with another key, the selection just before proved the tag was there, and a tag gone since is caught by the next step's wake-up | open; the next step wakes and selects the tag again |
| 2 | `tag_gone` | the tag answered neither its wake-up nor its selection | ended by the unit: the lane `pending` (`searching` in its load), its tag marked lost |
| 3 | `error` | a malformed answer, a CRC error, a NAK, a reader that did not answer; since v0.13.0 also a reader soft-reset while the step ran (no data frame then) | open |

A step's job belongs to the lane and the session it was sent for. A job whose session ended while
it ran sends nothing, and its result reaches no later session. The unit keeps a step's key only
until the step's job begins, and never sends a key.

**The end.** The host ends the session with `ace2k_rfid_done`: `read` makes the lane `read`; `give
up` returns it to `pending` (`searching` in its load). The unit ends the session itself, with the
lane `pending` (`searching` in its load), in three cases: 500 ms with no step from the host
(counted from the opening, the last step or the last step's result; since v0.13.0 also while the
reader is cleared by a soft reset or dead, and a step that never began goes with its key),
`tag_gone`, and a lost link.
The filament leaving the insert sensor ends it with the lane `unknown`. A host that is not there
reads no tag: its sessions end within half a second, and the lane waits for its next move.

**The host's side** (`klippy/extras/ace2k_rfid.py`). It offers the chip to the brand modules that
may carry it, by SAK and UID length: an NTAG (seven bytes, SAK 0) to the NTAG brands; a MIFARE
Classic 1K (SAK 0x08) to the MIFARE brands in `rfid_mifare_order`. It sends each candidate's steps
and hands the bytes back. It ends the session `read` as soon as a module decodes a record, or, once
every candidate has declined, with the UID alone — unless the cache knows that UID: a brand's
keys decoded it before, so every brand declining now is noise (a key refused at the edge of the
antenna's reach). The session then ends `give up`, as an NTAG's pages that refuse twice: the lane
keeps the cached record of its UID, unlatched, and the next move reads the tag again. A tag
rewritten under the same UID is so never latched with its old record.

- Only `auth_failed` sends the session on to the next brand: the key was that brand's, and the tag
  is not.
- An `error` re-sends the same step once. A second `error` on it ends the session. For an NTAG
  whose UID the cache does not know, it ends `read` with the UID alone (pages that refuse twice: a
  smaller chip, a password). Otherwise it ends `give up`, and the lane keeps the cached record of
  its UID if there is one.
- A step with no frame within 0.5 s is given up too.
- `tag_gone` ends the host's session with the cached record, if any.

At connect, every rfid format the module sends or waits for is checked against the dictionary. A
format from another commit refuses the connect, and a firmware without the reader's field disables
the tag G-codes.

## The load's search

With the reader's field and the feed in the image, the load (`ace2k_feed_start` mode 5, or the
automatic load) reads the spool's tag on its way in. Every phase is one of the feed's bounded moves,
under the mode `loading`, with the load's `seq` (0 for the automatic load). A STOP, the link, a
shutdown, a runout and the supervisors end it as they end any mode.

1. **The pull** to the parking point runs with the lane watched. A session on the lane **pauses**
   it: the lane stops, the session runs, and the pull resumes for what is left. A tag that stopped
   answering (`tag_gone`) is looked for once per load, backwards, at the floor speed, up to 120 mm.
   That is a stop's overshoot plus the longest tag window: 1.2 mm of overshoot at 30 mm/s, and a
   window measured at 102–116 mm on an Anycubic tag. This applies in the search too. Then the phase
   resumes.
2. **The search** runs when the pull reached the parking point with the lane's tag not read, the
   reach is past the parking point, and **every other lane of the unit is idle**. It goes forward,
   watched, until the first of these:
   - the tag is read (the session pauses the search as it pauses the pull);
   - the lane's own buffer full, when the tip snag does not forgive it: the filament met
     something past the outlet. The search ends; this is **not** the load's `blocked`, which it
     only is before the parking point. (Since v0.12.0 a full reading opens the free tip's windows,
     and the search goes on for what is left of its reach. The read with motion keeps the tip
     snag's tolerance instead: [Feed events](#feed-events));
   - the reach: `rfid_search_mm` of filament travelled since the load began (then `no_tag`, if no
     UID was attributed or seen, as above);
   - another lane is given a start, another lane's automatic load begins, or another lane's motor
     starts: a follow's burst, take-up, dose or tail (the search yields, the lane `pending`; the
     motor since v0.13.0). The follow's move is never held for the search.
3. **The return** to the parking point, measured on the encoder, at the load's speed. The load ends
   `loaded`.

A comparator trip in the return or the reacquire is `stuck`, whatever the buffer reads. A STOP at
any phase stops where the filament is (`stopped`), with the lane `pending` if its tag was not read.
An error, a runout, a lost link and a shutdown end the load where they find it, as in any mode.

**The read on command** (`ace2k_rfid_read` with `move=0`). It is refused without a filament
(`no_filament`), without a link (`no_link`), and while a session is open on the lane or a read on
command already waits on it (`busy`). Once accepted, it turns the lane's field on. The inventory
that completes the baseline answers it, or the next one when the baseline is already done, and a UID
seen in any inventory of the baseline counts (since v0.13.0). Since v0.13.0 it **attributes
nothing**: a tag goes to a lane only when it is read while that lane moves, and a lane that has not
moved stays `pending`.

- no UID but the neighbour's (`read`, `reading`, or kept, rule 1 above) answers the lane's
  unchanged state (since v0.13.0; up to v0.12.0 a kept UID of the neighbour's alone answered
  `ambiguous`);
- the lane's own UID, attributed earlier by its motion (kept from a session that ended with the
  lane `pending`), reopens its session, as an inventory would (rule 2 above);
- any other UID answers `ambiguous` and changes nothing. No session opens.

Up to v0.12.0 one UID that was not the neighbour's opened a session when the neighbour had no
filament or its tag was `read`.

The answer is an `ace2k_rfid_event` (the unchanged state, or `ambiguous`) at the latest
**2000 ms** after the command. A read that no inventory decided by then is answered with the lane's
unchanged state. A read on a dead reader is answered at once, the same way.

**The read with motion** (`ace2k_rfid_read` with `move=1`, in an image with the feed) runs phases
2 and 3 from where the filament is: forward up to `rfid_search_mm`, then back to where it started.
Its search keeps the tip snag's tolerance (the watch, the touch, the wait for rest, once per read),
not the free tip's windows: it can run on a lane whose filament sits in an extruder, where the
filament following the motor proves nothing ("The tip snag", above).
It is refused `busy` while a session is open on the lane or a read on command waits on it. This is
checked before anything else, so a refusal changes nothing. Then come the feed's refusals:
`no_link`, `busy` (the lane in a mode, moving or in error), `no_filament`, `other_lane_moving`
(another lane of the unit busy), `unsupported` (no search configured). Once accepted, it is an
**explicit re-read**. A `read` or `no_tag` lane forgets its tag and becomes `pending` (an
`ace2k_rfid_event`), then `searching` as the unit's own load (`seq` 0, mode `loading`) starts at the
search.

Since v0.13.0 a lane in the follow whose motor is at rest does not make the read with motion
`other_lane_moving`: the search needs only its own motor and its reader, and a follow at rest holds
neither. On the U1 the heads that do not print keep their lanes in the follow, so the read of a
new spool is no longer refused for them. Any other mode on another lane, a forward assist waiting
on its buffer included, still refuses it, and so does a follow whose motor runs. When such a
follow then moves (a burst, a take-up, a dose or its tail), the search yields on that tick or the
next: it goes back to where the read began, ends `loaded`, and leaves the lane `pending`. The
follow's move is never held, so a printing head is never short of filament for the search. The
load's own search, after the pull, still needs every other lane idle, a follow at rest included
(phase 2 above).

The host's `ACEK_RFID_READ MOVE=1` waits for the feed's terminal event of that load: the
`ace2k_feed_event` with `seq` 0 and mode `loading` that is not a notice. Then it asks for the lane's
state (`ace2k_rfid_lane_query`): `read` with the record its session produced, `pending`, or
`no_tag`. The tag events on the way only update the lane's status. Its wait is the travel (out to
the reach and back, plus an allowance for the sessions' pauses) at the feed's current load speed,
within the budget of any waited move.

## The grip

Since v0.11.0. Sometimes a filament is put in at a bay while the printer's head still holds the
previous piece, for example after a tail-out during a print. The unit must not chase that piece.
The automatic load's pull to the parking point and its tag search caught up with the old piece,
still moving at print speed, and pushed it (observed at the bench: the load ended `blocked`, and
the printer's own tangle detection paused the print moments later). The **grip** is how the host
says so, per lane: `ace2k_feed_lane_grip lane=%c grip_um=%u` (the dictionary, above).

- While a lane's grip is set, its **next automatic load** pulls only `grip_um` (40 mm by the host's
  default), at the load's speed, **without the tag search**, and ends `loaded` after a pull of
  `grip_um`. The automatic load is the one an insert edge starts on an idle lane with `auto_load`
  on, `seq` 0. The drive gear takes the filament, and the filament waits there. The load takes the
  grip as it begins, so that load uses up the grip however it ends.
- The pull is the load's pull. A read session on the lane pauses it, and it resumes for what is left
  of the grip, never past it. The filament falling more than `lag_um` behind the motor within a
  window while this lane's buffer reads full ("The free tip", above) ends it `blocked`, with
  nothing after. A STOP, the link, a shutdown, a runout and the supervisors
  end it as they end any mode. A grip never meets the comparator: its top bound, 45 mm, stops short
  of a load's first 50 mm, which the comparator does not judge.
- A **host start** on the lane that passes the start's refusals clears the grip: a feed, a
  rollback, an unload, a load (`ACEK_LOAD` pulls to the parking point, never the grip), an assist,
  the follow. So does an accepted **read with motion**. A refused start leaves the grip, whatever
  refused it (up to v0.12.0 a start refused for its own length, speed or park cleared it), as does
  another lane's start. `grip_um=0` clears it.
- RAM only. An MCU restart drops every grip, and the host clears every lane's grip as an init
  command at every connect. So a host restart that leaves the MCU running (`RESTART`,
  `SAVE_CONFIG`) does not keep a grip either.

The grip changes nothing else. The tag of a gripped spool is not searched for. It is read only if
it passes the antenna within the grip, or on the lane's next move. A read without motion
(`ace2k_rfid_read` with `move=0`, above) attributes nothing (since v0.13.0).

## The park

Since v0.12.0. A lane whose filament goes into a tube that another lane's filament shares (a
splitter before the head) must not run the tag search: the search goes up to `rfid_search_mm`
past the parking point and can reach the splitter. The grip was the only load without the search,
and its 40 mm left the filament where a roll back of 100 mm takes it out of the unit. The **park**
is the other answer: `ace2k_feed_lane_park lane=%c` (the dictionary, above).

- It takes the grip's place on the lane: while it is set, the lane's **next automatic load** pulls
  to the parking point (`load_park_mm`) as any load does, the tag read on the way when it passes
  the antenna, **without the search** past it, and ends `loaded` there.
- That load is a whole load: the comparator judges it, a full reading on the way is `blocked`, and
  a read session pauses it and it resumes for what is left.
- It is used up and cleared as a grip is: by the load it takes, by a host start on the lane, by an
  accepted read with motion, and by `ace2k_feed_lane_grip` with `grip_um=0`. A grip set after it
  replaces it, and the park set after a grip replaces the grip.
- RAM only, cleared at every connect with the grips.

## The wait

Since v0.13.0. A head can still hold a piece of filament that the lane does not own, and the end
of that piece may sit at the gear. An automatic load on an insert edge would then push on it. The
**wait** keeps the lane from loading: `ace2k_feed_lane_wait lane=%c` (the dictionary, above).

- It takes the grip's place on the lane: while it is set, an insert edge on the lane starts no
  automatic load, and the filament stays where the hand left it.
- An insertion does not use it up. It holds through any number of insertions.
- It is cleared by an accepted host start on the lane (a refused one leaves it), by an accepted
  read with motion, and by `ace2k_feed_lane_grip` with `grip_um=0`. A grip or a park set after it
  replaces it.
- RAM only, cleared at every connect with the grips.

## The dryer

Since v0.5.0 the unit dries on its own (ADR 0013: `heat` under `dryer`). Two layers drive the
heater.

**`heat`** is the only code that touches the triac gate. It fires whole mains cycles from the
zero-cross interrupt, inside a **lease** of at most 2 000 ms that the layer above must renew. It
holds its own limits, independently of the layer above:

- both fans commanded on and reading high;
- the duty at most 90 %;
- both outlet NTCs valid and below 85 °C;
- the mains present and at 50 or 60 ± 1 Hz;
- the thermal cutout input clear.

Every 10 ms tick judges these limits and publishes a verdict. The interrupt fires only on a good
verdict from a tick at most three half-cycles old. A tick that finds a limit broken stops the gate
and latches at once, except in the two cases below.

**An outlet NTC reading invalid.** The firing stops at once, on the first tick that sees it. The
gate latches once the invalid readings fill a leaky bucket: each invalid tick adds 4, each valid
tick drains 1, and the latch is at 20. That is 5 invalid ticks in a row (50 ms), or an NTC flapping
invalid more than one tick in five, sustained (4 invalid / 1 valid latches on the 7th tick, 1 / 1
on the 13th). A lone glitch of up to 4 ticks does not end a cycle: the firing resumes once the
reading is valid again.

**The mains read implausible** (present, but outside the two bands) is treated the same way, with
its own bucket, kept in every state. The firing stops on the first tick that sees it. Each
implausible tick adds 2 and each plausible tick drains 1, and the latch is at 300. That is 150
implausible ticks in a row (1.5 s), or one 1 s window off-band in two, sustained (latched in the
second off-band window). The tolerance is about one window in three, sustained: +2 × 100 ticks
against −1 × 200 balances, so it never latches. It is slightly less when the windows run long: a
window lasts 100–101 ticks, and an off-band window a tick longer than its two plausible ones creeps
up by a few units a period, to a latch after minutes. Through the tolerated dips the heater fires
only in the windows read plausible, and the dryer raises its `mains_dip` notice once it has ridden
a dip out. One window read off-band never latches (a burst of noise costs the zero-cross input a few
edges to its lockout, observed on the bench). The firing resumes inside the live lease once the
next window reads plausible.

**Leases and starts while the mains is off.** A lease asked while the mains has been off-band for
less than that (the bucket not full) is refused as transient, and the dryer asks again on the next
tick. A lease asked once the bucket is full is refused, and the dryer faults `mains`. While the
mains is absent, or the bucket is above its restart level, whatever the last reading:

- a start and the clear of the dryer's `mains` fault are refused (`no_mains`);
- a new lease and the clear of the gate's latch on the mains are held.

The restart level is 96: the latch level less one whole window of 101 ticks, and a tick of margin.
So a new cycle always rides out one whole off-band window, and a clear never resets the bucket. No
window measured yet (the first second after a boot) does not fill the bucket.

**After an outage** (v0.6.0) the mains counts as measured again only once a window publishes a
plausible frequency. Until then (from boot, and from any tick the mains reads absent), a start and
the clear of a `mains` fault are refused `measuring` (9, transient: ask again in a second or two),
and a new lease is refused as transient. A live lease is never refused for it. A dip with the mains
present (off-band, not absent) keeps it measured.

**One dip, one off-band window** (v0.6.0). A dip leaves one long gap between edges. The gap check
sees it twice, half a period apart: paired with the gap before it, and paired with the gap after
it. When a publication fell between the two, two windows read off-band and could latch a live
lease. Now a window that publishes `missed edge` for a gap remembers it: the next window does not
count the same gap again, and its measurement starts past the gap. A lasting off-band frequency, or
a second dip, still reads off-band in every window it is in.

**A refused clear.** A clear that the gate refuses names what holds it, in this order: the gate
reading high (`faulted`: the heater may be on), the cutout, a limit out of bounds (an NTC over or
invalid, the mains absent), then the mains bucket, for a latch on the mains only. An invalid NTC is
reported ahead of an implausible mains when both hold. The mains absent latches at once.

**The phase check.** Every zero-cross is judged against the edge two before it. That edge has the
same polarity, so an asymmetry between the input's two half-cycles cancels. The edge is in phase
only when the gap is the period of the measured frequency (10 000 000 / `hz10` µs: 16 667 at
60 Hz, 20 000 at 50 Hz) within 600 µs. The gap is timed between the two edges' interrupt entries
(the firmware masks interrupts for microseconds at a time). The budget is the frequency's
measurement step (118–122 edges in a 1 s window on the bench, ±2 edges = ±278 µs of period at
60 Hz, ±400 µs at 50 Hz), plus a few µs of jitter. Nothing fires before a plausible frequency is
measured. A noise spike that the 7 ms lockout let through in place of a zero-cross comes 1.1–1.3 ms
short of a 60 Hz period and 2.8–3 ms short of a 50 Hz one: out of phase.

- An edge out of phase (such a spike, or an edge after a missed one) starts no cycle. Neither does
  the in-phase edge right after it, because that second half would be judged against the spike.
- As a cycle's second half, an edge out of phase abandons the cycle rather than fire two halves of
  one polarity. The first half, already fired, stays alone, as with a missed edge.
- The dryer raises its `edges_off_phase` notice once 10 edges were judged out of phase in a cycle.
- A noise edge closer to the true zero-cross still passes, with the polarity right, early by at
  most the tolerance plus the frequency's misread: about 0.9 ms at 60 Hz, 1.0 ms at 50 Hz.

**Stopping the gate.** A release between the two halves (a stop, the cool-down, a duty of 0) still
fires the second half on a good verdict, and never a new first half. A release never cuts a pulse
already started (the pulse timer ends it, 6 ms); only a latch, an abort or a shutdown drive the
gate off. A latch, a Klipper shutdown, a dryer fault and a tick whose verdict is not good stop the
gate at once. Their first action is a single store to the one word the zero-cross interrupt reads
to decide each edge, and no zero-cross after that store fires anything, not even that second half.
(A first half that fired before it stays alone.) No lease starts while the config page is being
written. Once the gate is latched (or the MCU shut down), a pulse still reading high no longer
holds a config store back, so the latch's log entry is written. But a gate reading high, latched or
not, still refuses the bootloader entry and keeps the fans on: a gate stuck high is heat on. A
latched gate is not cleared while it reads high either. A release with no first half fired stops at
once.

**`dryer`** carries the policy: the states below, the cascade controller, the vent, the cool-down,
the protections, the log. A fan never switches off while an outlet NTC reads above 45 °C, in every
state, after a boot and after a Klipper shutdown. The fans switch on above 45 °C, and may switch
off once both NTCs read below 42 °C; an invalid NTC counts as hot.

**The flaps.** The two flaps never pulse together, whoever asks (the dryer, `ace2k_flap_pulse`):

- A dryer's pulse asked while either flap's pulse runs is held, one per flap, the latest request
  winning. It never wins over a stronger owner's request (the dryer, then manual): a weaker request
  on a flap where a stronger one runs or waits answers `busy`. A held pulse starts when the running
  pulse ends, the other flap's held request first.
- A manual pulse is never held. Asked while any flap's pulse runs or waits, it answers `busy`, and
  nothing moves later (v0.6.0).
- The dryer asks for the bottom flap, then the rear one, in the same tick. With both flaps at rest,
  the bottom's pulse starts and the rear's follows when it ends. If a flap is already pulsing when
  the dryer asks, airflow starts the other flap's held request first, so the order can be rear,
  then bottom.

A flap's position is published only when a pulse on it has run to its end. It reads `unknown` from
the moment a pulse is asked until that pulse ends, so no position is ever reported that was not
reached. A dryer sequence is over when the dryer's own two pulses have ended (another owner's
pulses do not hold it), or 2 s after it began. (In `starting` the flaps then fault `flaps` at the
3 s timeout.) At every boot the dryer pulses both flaps closed once, a boot held in `fault` by a
persisted cutout included. The flaps hold their position unpowered, and a cool-down cut short by a
reset or a power loss can leave them open. No owner is taken. A cool-down or a fault closes them
again at its own end, even when it began while the boot's close was running. After a Klipper
shutdown no flap moves: they stay where they are until the next boot's close. A flap whose pulse
the shutdown cut reads `unknown` (0) in `flaps`.

**The cycle.**

1. `ace2k_dryer_start` → `starting`. Both flaps are pulsed closed and the fans come on. (A manual
   run holding the fans refuses the start `busy`: `ACEK_FAN ON=0` first.) Once both fans read high
   and the cycle's open mark is on the log page → `heating`. Both fans not high within 3 s: fault
   `fans`. A mark the flash has not taken by then does not hold the cycle back: the log is a
   record, not a safety function.
2. In `heating`, the inner loop drives the hotter outlet NTC to the target + 7 °C (never above the
   target + 8 °C nor 73 °C). It recomputes the duty every 2 s and renews a 2 000 ms lease every
   500 ms. The outer loop trims that by at most ±2 °C, once every 10 min, from the chamber sensor,
   when the chamber held within 0.8 °C over those 10 min.
3. At the end of the duration, or on `ace2k_dryer_stop` → `cooldown` ("The cool-down", below).

**The vent** (v0.6.0). Once the chamber is warm, both flaps are pulsed open, and they stay open to
the end of the cycle. The dryer samples the chamber every 30 s from the entry into `heating`. From
the first sample at least 5 min into `heating`, it opens on the first sample where the chamber:

- reads at least the target − 3 °C (`ACE2K_DRYER_VENT_NEAR_MC`): near its target; or
- rose less than 0.5 °C over the last 5 min (`ACE2K_DRYER_VENT_RISE_MC`): as warm as the heater can
  make it.

An invalid chamber sample decides nothing and restarts that window, so the chamber is judged over
valid samples only. Replayed on the recorded cycles, the vent opens at 37.0 min (PETG, 65 °C),
27.5 min (empty, 65 °C), 16.0 min (55 °C) and 12.0 min (45 °C). v0.5.0 vented once the chamber had
held within 0.5 °C for 20 min (or after 2 h): an hour late, or not at all.

**The humidity guard.** At `starting` the dryer records the chamber's absolute humidity (from its
temperature and relative humidity) as the room's. The exception is a quick restart: when the last
cycle ended (its cool-down done, or its fault cleared) less than 4 h ago, and the reference kept
from an earlier start was taken less than 24 h ago, it keeps that reference, because a chamber
still dry from the cycle before reads too low. (Initial values; RAM only: the first start after a
boot reads the chamber.) Once vented, the flaps close only if the chamber reads more than 1 g/m³
drier than the room reference for 60 s (initial value). They reopen once it reads at least 2 g/m³
above it for 60 s. The guard moves them at most once per 10 min. An invalid humidity or chamber
reading freezes the guard where it is and restarts its 60 s (an RH above 100 % counts as 100 %).
The guard is expected to fire only in a room more humid than ~30 g/m³; it never did in the recorded
cycles.

**The cool-down.** At the end of the duration, or on `ace2k_dryer_stop`, the dryer goes to
`cooldown`. The heat goes off at once. Both flaps are pulsed open (again, if the vent already
opened them), so the fans draw room air across the still-hot heater and push the humid air out.
This happens only when the cycle reached `heating`: a stop in `starting` opens nothing. The fans
stay on until both outlet NTCs read valid, at most 45 °C, and steady or falling slowly: over the
last 30 s (at least 30 s), neither rose more than 0.2 °C (the sensor's noise; an initial value) and
neither fell 0.5 °C or more. This is judged at the 30 s window's two ends only:

- a spike between them that is back by the window's end does not hold the cool-down;
- one side rising is enough to hold it;
- an NTC invalid at a window's end restarts the window from fresh readings, once both read valid
  again.

An outlet still rising after the heater stops (its overshoot) holds the cool-down (v0.6.0; v0.5.0
passed a rising reading). Then both flaps are pulsed closed and the fans released → `idle`. A
cool-down that has not ended after 10 min (a hot room) closes the flaps and hands the fans to the
45 °C rule all the same (`hot_ambient`).

**The link, a shutdown, a reset.** A lost host link changes nothing: the cycle runs to its end. A
Klipper shutdown stops the heat and leaves the fans to the 45 °C rule alone (the dryer gives up its
claim on them). The dryer then refuses every start (`shutdown`), and the gate every lease, until the
MCU restarts. It reports `idle`, or `fault` with its reason when it was in a fault (since v0.13.0;
before, `idle` with the fault's code). **A reset never resumes a cycle**: the unit boots `idle` and
logs the cycle `interrupted`.

| State | Name | Meaning |
|---|---|---|
| 0 | `idle` | no cycle; the fans only under the 45 °C rule or by hand |
| 1 | `starting` | the flaps closing, the fans coming on — no heat yet |
| 2 | `heating` | the cycle runs |
| 3 | `cooldown` | the heat off, the flaps open and the fans on until the outlets are cool, then the flaps closed |
| 4 | `fault` | a protection tripped: the heat off at once (no half-cycle after it), the flaps and the fans cooling as in `cooldown`. The flaps are opened only when the cycle had reached `heating`: a fault on a cold unit, such as a cutout glitch at `idle`, only closes them. `ace2k_dryer_clear` leaves it once that cool-down has ended and the cause is gone |

The protections 1–7 are checked while `heating`; the cutout and the gate's latch in every state.

| Fault | Name | Trips when |
|---|---|---|
| 1 | `not_heating` | 300 s into `heating`, an outlet NTC has neither risen 5 °C above its value at the start of `heating` nor come within 2 °C of the drive target |
| 2 | `response` | the hotter outlet rose faster than the heater can explain: over the last 30 s, more than 35 m°C/s per % of the duty applied over them (and over the 15 s before them at a quarter weight, because the outlet NTC answers the gate 10–15 s late: the overshoot after a stop), plus 2 °C. This stands in for a stopped fan or a blocked duct, since no fan's rotation is read |
| 3 | `sides` | the two outlet NTCs more than 15 °C apart for 60 s |
| 4 | `chamber_over` | the chamber above its ceiling for 30 s, or above 80 °C. The ceiling is the cycle's highest target or the chamber's lowest reading since the start, whichever is higher, + 10 °C. The chamber's part only ever moves down within a cycle; the target's part rises when a retarget asks a higher target, and never falls (v0.6.0; v0.5.0 counted from the target alone, so a target 10 °C below the room latched it with nothing heating; the retarget since v0.13.0) |
| 5 | `chamber_stale` | the chamber reading implausible, or none newer than 5 s |
| 6 | `ntc` | an outlet NTC invalid for 5 samples 100 ms apart while no lease is held (a duty of 0, a renewal waiting); or the gate latched on an invalid NTC (its leaky bucket, above) or on one at 85 °C |
| 7 | `mains` | the mains absent, at once; implausible once the gate's bucket fills (1.5 s off-band, above); or the gate latched on either |
| 8 | `cutout` | the thermal cutout input asserted — in any state, at boot too. **Cleared only by a power-on reset** ("The cutout", below) |
| 9 | `heat` | the gate latched for a reason of its own — `gate_stuck` or `duty` (the gate's limits, below) — or refused a lease with no other reason |
| 10 | `fans` | both fans not reading high 3 s into `starting`, or the gate latched on the fans |
| 11 | `flaps` | the flaps' closing pulses not both over 3 s into `starting` (their sequence gives up after 2 s; they take at most 0.8 s) |
| 12 | `lid` | the chamber did not follow the heater for 2 min from the 8th minute of heating: the lid is likely open. The chamber's rise must be at least 10 % of the outlets' mean rise, both from the chamber at the entry into `heating` (a stale chamber faults `chamber_stale` first). It is not judged while the heater is less than 5 °C above that reference, or while the current target is less than 5 °C above it: a chamber already near its target has no room to rise, so an open lid on such a box is not caught (v0.13.0) |

The rise-rate ceiling of fault 2, and the thermal simulator the host tests run whole cycles on, are
fitted to four heater runs recorded on the unit. The simulator's chamber loss to the room is
assumed, not fitted, so the chamber's accuracy and the outlets' overshoot are settled on the unit.
Every threshold here is an initial value that the bench confirms or re-tunes.

A refusal leaves everything as it was. The start checks, in this order, the state, the range, then
the inputs:

| Refusal | Name | Meaning |
|---|---|---|
| 1 | `busy` | start: the dryer not `idle` (a cycle or a cool-down running), or the fans held by a manual run (`ACEK_FAN ON=0` first) |
| 2 | `range` | start: `target_c` outside 15–65, `minutes` outside 1–1 440 |
| 3 | `faulted` | start: the dryer in `fault`, or the gate latched; stop: the dryer in `fault`; clear: the gate's latch would not clear for a reason none of the refusals below names |
| 4 | `sensors` | start: an outlet NTC invalid, or no plausible chamber reading within 5 s; clear of a `ntc` or `chamber_stale` fault: that sensor still not valid; clear of any fault the gate latched: an outlet NTC invalid |
| 5 | `no_mains` | start, or clear of a `mains` fault or of any fault the gate latched: the mains absent or implausible |
| 6 | `cutout` | clear: the cutout tripped since the last power-on reset (in any state); start: the same, when the dryer is not already in `fault` |
| 7 | `not_cool` | clear: the fault's cool-down not ended yet, or (`chamber_over`) the chamber still above its ceiling, or (a fault the gate latched) an outlet NTC still at 85 °C or more |
| 8 | `shutdown` | start: a Klipper shutdown happened since the MCU started; the gate refuses every lease until a reset (`FIRMWARE_RESTART`) |
| 9 | `measuring` | start, or clear of a `mains` fault: the mains present but no plausible window published since boot or since it last read absent — transient, ask again (v0.6.0) |

| Event | Name | `arg` |
|---|---|---|
| 0 | `done` | 0 — the duration ran out; `cooldown` follows |
| 1 | `stopped` | 0 — `ace2k_dryer_stop` ended a cycle, or `ace2k_dryer_stop_bound` a bound one (v0.13.0) |
| 2 | `fault` | the fault |
| 3 | `lowered` | the lane (1–4): while `starting` or `heating`, with the host link down and the target above 45 °C, a filament reached that lane's insert sensor. The target is now 45 °C (the chamber ceiling keeps the cycle's highest). A spool put on its holder without a filament is not seen |
| 4 | `interrupted` | 0 — sent after a boot that found a cycle open: a reset or a power loss ended it, and it was not resumed |
| 5 | `hot_ambient` | 0 — the cool-down had not ended after 10 min: the flaps closed, the fans left to the 45 °C rule, the dryer `idle`; no fault |
| 6 | `vented` | 0 — the flaps pulsed open for the cycle's vent. Sent on the first opening only; the humidity guard's moves are silent (v0.6.0) |
| 7 | `cleared` | 0 — `ace2k_dryer_clear` accepted |
| 8 | `auto_started` | the cycle's target in °C — the humidity mode started a cycle (v0.13.0) |
| 9 | `auto_dry` | the box's RH in whole % — the first cold reading after a humidity-mode cycle read below the limit; the failure count is 0 again (v0.13.0) |
| 10 | `auto_failed` | the failed cycles in a row — that reading was at or above the limit, or the cycle ended in the `lid` fault (v0.13.0) |
| 11 | `auto_stopped` | the failed cycles in a row — the count reached the maximum, or the maximum was lowered to the count: the humidity mode is `stopped` until it is set again (v0.13.0) |
| 12 | `auto_off` | 0 — `ace2k_dryer_stop` ended a humidity-mode cycle, and the mode is now manual (v0.13.0) |
| 13 | `stopped_link` | 0 — a bound cycle stopped because the host link stayed down for 5 min (v0.13.0) |

**The events are held until the host acknowledges them** (v0.6.0). None is sent before the host's
first `ace2k_dryer_query`. The host sends that query as an init command, so the events of a boot
(an `interrupted`, a cutout) reach the host that connects. Each event carries a `seq`, and stays in
the ring until an `ace2k_dryer_event_ack` covers it. The unacknowledged ones are sent again, oldest
first, after every `ace2k_dryer_query` and every 5 s. So an event emitted with the host link down,
or one whose frame was lost, arrives once the host is back. The ring holds seven; an event that
does not fit is dropped and counted in the state's `lost`. A boot into `fault` with the cutout
persisted is the ring's first event: `fault` with `arg` 8, acknowledged like any other. v0.5.0 sent
each event once and dropped it as sent.

**The host's side.** After a connect, the host takes its base from the first `ace2k_dryer_state`
report's `oldest`. An event that arrives before that report is held, neither handled nor
acknowledged, and taken in order from the base once the report is in. From then on the host handles
the events strictly in sequence. An event after a gap makes it send one `ace2k_dryer_query` (once
per gap), so the unit resends at once instead of 5 s later.

`notices` bits:

- 0x1 `hot_ambient`: the last cool-down ended on the 10 min hand-over. Cleared at the next start.
- 0x2 `lowered`: the target was lowered this cycle. Cleared at the next start.
- 0x4 `log_unstored`: the log's last store failed on the flash. Since v0.13.0 the same write
  carries the modes' block ("The dryer's modes"), so the bit covers it too. The unit tries it
  again after a back-off of 1 s, doubling at every failure up to 60 s, and the bit clears with the
  first store that succeeds.
- 0x8 `mains_dip`: the mains read off-band at least once in `starting` or `heating` this cycle, and
  did not fault it. Set when the mains reads plausible again during the cycle, or when the cycle
  ends without a fault with the dip still open; so it does not say the dip ended. Cleared at the
  next start. The firmware counts the dips; the frame carries only the bit.
- 0x10 `edges_off_phase`: at least 10 zero-cross edges judged out of phase this cycle. That is noise
  at the input, or a half-cycle asymmetry past the gate's tolerance; the heater skips those cycles.
  Cleared at the next start.
- 0x20 `ambient_above` (v0.6.0): the chamber started above the target, and nothing heats until it
  falls below. The cycle runs, the controller asks no duty while the drive target is below the
  outlets, and the fans and the flaps follow the cycle. Judged once, at the start, and cleared at
  the next start.

**The cutout.** Once the cutout is seen asserted, in any state and whatever fault came first, it is
sticky in both layers. The gate latches on it and refuses every lease. The dryer holds `fault`
`cutout`: a start answers 3 `faulted` and a clear 6 `cutout`. It is also written to the log's
flags at once, so a software reset (`FIRMWARE_RESTART`, the watchdog) boots back into `fault`
`cutout`. Only a power-on reset clears it.

**The airflow.** `owner`: 0 `none`; 1 `manual` (`ace2k_fan_set`, `ace2k_flap_pulse`); 3 `dryer`
(from the start until the cool-down has closed the flaps); 2 is not used. A manual command is
refused (`busy`) while `dryer` holds the outputs. A manual flap pulse is also refused while any
flap's pulse runs or waits (v0.6.0). The 45 °C rule overrides every owner's off.

**The gate's limits** (the reasons `heat` latches on; the dryer maps each to its fault):
1 `fans` (not commanded on, or not both reading high), 2 `ntc_invalid`, 3 `ntc_over` (85 °C),
4 `mains_absent`, 5 `mains_implausible` (after its bucket, above: 1.5 s off-band), 6 `cutout`,
7 `gate_stuck` (the gate read high outside a pulse, or still high 8 ms after one), 8 `duty` (above
90 %).

No image drives or configures PC9, the cutout's latch reset. It stays reserved exactly once in
every image (`RESERVE_PINS_ace2k_cutout`), so the host cannot claim it either.

**The log** is persisted in the ace2k config page (record layout 2; a layout-1 page migrates on its
next store, with the log zeroed). It is written only with the gate idle, so a page erase never
stalls a firing. Read it with `ace2k_dryer_log_query`:

| `index` | `a` | `b` | `c` | `d` | `e` | `f` |
|---|---|---|---|---|---|---|
| 0 | cycles started | cycles completed | seconds of heating (a lease held and the gate free to fire: not through a mains dip or an NTC glitch) | full-power seconds (the duty-weighted heating time; × the heater's power = energy) | faults | flags: 0x1 a cycle open (seen at boot as `interrupted`), 0x2 the cutout tripped (cleared by a power-on reset) |
| 1–8 | kind \| reason << 8 \| cycle << 16 (kind 2 `fault` or 4 `interrupted`, reason the fault or 0, cycle the number of the cycle open at the time — the count of cycles started — or 0 when none was open: a fault in `starting` before the cycle's open mark, in `idle` or in `cooldown`; kind 0: an empty slot) | seconds of heating at the time | left outlet | right outlet | chamber | 0 |

The three temperatures of an entry are in tenths of a degree: a signed 16-bit value, sign-extended
to 32 bits on the wire. −32768 is a reading that was invalid or absent, as is every temperature of
an `interrupted` entry. The cycle is marked open on the page before the first lease, and closed at
the cycle's end. A second reset that comes before the boot's cleared mark is stored finds the mark
still set, and logs the same cycle `interrupted` twice.

### The dryer's modes

Since v0.13.0 the unit runs three modes beside the dryer: `manual` (0), `humidity` (1) and `print`
(2). The modes decide like a user. They start and stop the dryer, and they read its state. Their
tick runs every 10 ms, right after the dryer's.

**The host's start and stop go through them.** `ace2k_dryer_start` over a humidity-mode cycle or a
bound cycle that is still in `starting` or `heating` does not start a second cycle. It changes the
running cycle's target and time in place, and the cycle becomes manual: it counts for nothing.
Otherwise it is a plain start. Either way, an accepted start drops an outcome still waiting for a
cold box. `ace2k_dryer_stop` of a humidity-mode cycle sets the mode to manual,
and event 12 `auto_off` follows. That stop is never counted as a completed cycle. The stop of a
bound cycle leaves the mode as it was.

**The cold box.** The chamber is sampled once a minute over a 30 min window. The box reads cold
when the dryer is idle, the chamber and its RH are valid, and the chamber is steady over the window:
it neither fell nor rose 0.5 °C. A warming box (the sun, a room heating up) is not cold. The window starts again after a reset, when the dryer leaves idle (a manual start too),
and when the host sends a different window. An invalid or stale reading takes no sample and decides
nothing: no outcome and no start, and the hold waits. The window and the hold are kept through a
glitch shorter than 60 s; readings that stay invalid for 60 s empty the window and end the hold.
The hold at or above the limit starts again when `ace2k_dryer_auto_mode` changes the mode or re-enables
`humidity`, and when the host sends a different hold. A Klipper restart that sends the same timing
restarts neither.

**The humidity mode.** With the box cold, an RH at or above the limit for 30 min in a row starts a
cycle at the profile (event 8 `auto_started`). The limit itself starts one, because an outcome at
the limit is a failure. After that cycle completes, the first cold reading decides
its outcome. Below the limit, the failure count goes back to 0 (event 9 `auto_dry`). At or above it,
the count goes up by one (event 10 `auto_failed`). A `lid` fault (12) on an own automatic cycle also counts, at once and as a failure: event 10 follows the fault event on the next tick, with no reading. When the count reaches the maximum, or the host
lowers the maximum to the count, the mode is `stopped` (event 11 `auto_stopped`) until the host sets
`humidity` again. A cycle that ended in a reset counts neither way, nor does one that ended in a fault other than `lid`.

**The print mode.** The unit only keeps the mode, and reports `cold` and `above_s`. The host
decides, starts a bound cycle with `ace2k_dryer_start_bound` and stops it with
`ace2k_dryer_stop_bound`. A bound cycle stops on its own
once the host link has been down for 5 min (event 13 `stopped_link`).

The host watches Klipper's `print_stats`, so the mode needs `[virtual_sdcard]`. With the mode
`print` and the dryer idle:

- At the start of a print, with the box cold and its RH at or above the limit, the host starts a
  bound cycle at the profile's temperature for 1440 min.
- During the print, when the RH has stayed at or above the limit for the hold (1800 s), it starts
  one.
- At the end of the print (`complete`, `cancelled`, `error` or `standby`) it stops the cycle that
  it started. After a host restart in a print, it adopts a cycle that the unit reports as bound.
- A pause is not an end.
- The end of the print stops the cycle only if the unit's report still shows it as bound. A
  cycle that `ACEK_DRY` started over it is not stopped. A bound cycle found with no print
  running (a Klipper restart ends the print, the unit keeps the cycle) is stopped. A stop that
  fails is tried again every 5 s, until the report shows the cycle unbound or over. Each of these
  stops is `ace2k_dryer_stop_bound`, so the unit itself spares a cycle that `ACEK_DRY` took over
  after the report the host read. On an image without that command, the host sends
  `ace2k_dryer_stop`.
- After a start the host waits 5 s before it starts again, because the dryer reads idle until its
  next report.
- The log line that the mode needs `[virtual_sdcard]` is written only in the `print` mode.
- If the cycle is stopped by hand, loses the link (`stopped_link`) or faults, the host starts no
  other until the print ends. The same holds when `ACEK_DRY` over the cycle makes it manual: the
  reports show it running and no longer bound, 5 s or more after the start, on two polls at least
  1 s apart. One poll is not enough, because the modes' report and the dryer's come in separately:
  a cycle that ends by its time can read unbound in one and still running in the other.
- A report that comes in while a start or a stop waits for its answer does nothing. A cycle that ends by
  its time (`done`) is started again.
- A start that the unit refuses is logged once per print, and tried again after the hold.

**The profile with no host.** With the host link down, a filament that reaches an empty lane's
insert sensor lowers a stored profile above 45 °C to 45 °C and 360 min, as the dryer lowers a
running cycle.

**What the page keeps.** The mode, the limit, the maximum, the failure count, the profile and the
`stopped` flag are stored in a marked block of the config page. The block is written with the
dryer's log, in the same write of the page: only with the heater's gate idle, and with the log's
back-off after a flash failure. So a failed store of the block shows as the `log_unstored` notice
of `ace2k_dryer_state`. Whether the running cycle is the mode's (`own`) or bound to the host
(`bound`) is kept in RAM only. So a reset never resumes a cycle and never scores one.

## The link is half-duplex

The unit's RS-485 pair is half-duplex, and the unit is deaf while it transmits: the transceiver's
driver-enable on PA11 also disables its receiver (`hardware.md`, "Host link"). Klipper's transport
assumes a full-duplex link.

Since the link turnaround (`v0.2.0`), the unit raises its driver-enable only once the line has been
quiet for 100 µs (2.5 characters) after the last received byte. So it never answers over the end of
a host burst: the configuration at connect, a retransmitted window, any run of frames the host
sends back-to-back. Before the turnaround, the unit acknowledged the first frame of a burst while
the rest was still on the wire, and `klippy` stalled at connect about one time in two. Measured on
the unit: before, bursts of six frames were processed 1 of 6 in every round, with none of the 30
responses seen; after the turnaround, 30 of 30 were processed.

What remains is the other half of the collision. The host's cable cannot sense the line, so a host
frame that starts while the unit is transmitting a report is lost on both sides. The transport's
retransmission heals it: **about 1.3 % of the host's frames over an hour with every report
active**, measured on the unit. (The shorter sample before the turnaround read ≈ 2 %. The
difference is within what the samples can tell: the term the turnaround removes was the smaller
one.)

Two consequences to know:

- Klipper's `reset`, which `FIRMWARE_RESTART` sends, is the one frame the transport never
  retransmits. A unit that is reporting can lose it to such a collision (about 1 restart in 13 in
  the U1's logs), and `klippy` reports `Failed automated reset of MCU`: repeat the
  `FIRMWARE_RESTART`. A stopped unit (after a Klipper shutdown) goes quiet: it sends the frames
  of the shutdown itself for one second, then no report or event until its reset. What still
  goes out is Klipper's own: its replies to the host and its statistics every 5 s. So the
  restart that clears a stop, ace2k's automatic one or yours, almost always takes it the first
  time.
- The unit holds its replies until a host burst has ended, so a burst's replies must fit Klipper's
  96-byte transmit buffer. The configuration burst at connect (46 bytes of replies) fits; six
  queries with 12-byte answers lose the sixth.

Shorter or host-polled report traffic is the lever for the residual, and is left to a later change.
`ace2k_link_query` counts the waits.

## Who can flash the unit

The unit does not check who sends a frame. Any program that can send frames on the unit's link
can flash it. `ace2k_bootloader_enter` resets the unit into the bootloader's recovery mode. The
recovery mode takes an image from whoever sends it. It checks only the trailer at the end of the
image, and that check is not a signature. The new image then decides how the heater, the fans and
the motors run, so none of the rules of the old image hold any more. The command is refused only
while one of the vetoes in its row of the command table holds: the heater, the fans or a moving
lane. They protect the hardware during the reset. They do not decide who may flash. This is how
`ace2k_flash.pyz` works (`flashing.md`). The host link is trusted by design.

## The link proof

A valid image that boots but is never spoken to would otherwise be unrecoverable over the wire: the
bootloader validates it and jumps to it on every power cycle. So an image keeps, on its config
page, which version last proved the link. A different version (a fresh flash) starts a 180 s
window, shown on the lane LEDs as a one-lane chase. The first answered `ace2k_version` stores the
proof. The LEDs then leave the chase for the intro sweep (every lane empty) or the lanes' own
states (`hardware.md`, "Lane LEDs"). The unit writes no flash while the heater runs. If the heater
runs when the proof comes, the unit stores the proof by itself as soon as the heater lets it. Until
then `ace2k_linkproof_state` reports `proven=0`. That wait has no deadline while the host keeps
talking: the host spoke, so the window does not end in recovery while the proof waits for the
flash, however long the heater holds it. Klipper's clock query, about once a second, is enough
talk. `remaining_ms` reads 0 past the deadline. Past the deadline, the unit judges the link on
every tick, as the motion rules do: 3 s without a valid frame is a lost link. A lost link ends the
wait (Klipper stopped, the cable pulled), and the window, already past, ends in recovery as below.
The end is final: a host that comes back finds the window ended, and its `ace2k_version` no longer
proves the image. A silence before the deadline counts for nothing once the host talks again by
the deadline. A store that fails on the flash also ends the wait, in the same way. The image names
`ACE2K_LINKPROOF_RETRY_PAST_WINDOW`. A test image of v0.13.0 without it retried only inside the
window, and a refusal that outlasted it still ended in recovery. An image before v0.13.0 does not
retry: the proof stays unstored until the host asks again, and the window ends in recovery.

An image whose window runs out resets into recovery, where the updater works. The reset is asked
through the same vetoes as `ace2k_bootloader_enter`: the heater active, the fans required (a dryer
cycle, or an outlet NTC still at or above 42 °C), a lane moving. Each veto is bounded: a cool-down,
a move's own deadline. While one objects, the reset is deferred and asked again every 10 ms, and
checked once more with interrupts masked right before it. The unit keeps answering meanwhile
(`ace2k_linkproof_state` reports `proven=0 remaining_ms=0`). The reset follows on the first tick
no veto objects. A gate stuck high defers it until a power cycle. A proven image never resets
itself for lack of a host.

Consequence for hosts: **the `[ace2k]` section is part of the link, not an option.** A `klippy`
configured with `[mcu ace2k]` alone never sends `ace2k_version`, and a freshly flashed unit that
the flasher did not prove will drop into recovery three minutes after boot. `ace2k_flash.pyz` proves
the image itself right after it writes it: it reads the dictionary (`identify`) and asks
`ace2k_version`, as Klipper does at connect, on the unit's port at 250000 baud. The module queries at MCU identify time, before the
connect handlers (which stop at the first config error). It retries the proof once when the state
comes back unproven. A `proven=0` after that means the store failed on the unit, or the heater
ran, and the module logs a warning. The module then watches the state: once Klipper is ready, it
reads `ace2k_linkproof_state` every 2 s, and never sends `ace2k_version` again. It turns
`link_proven` true when the unit stores the proof later. It logs one warning when it reads the end
of the window with the proof unstored. With `ACE2K_LINKPROOF_RETRY_PAST_WINDOW` it goes on
watching after that, because the unit waits for the heater while the host keeps talking, and
Klipper's own clock queries are that talk. With an older image it stops there (a
veto, such as the heater, holds the reset). Without a veto the unit resets at once, and Klipper
reports the lost MCU instead. It stops after five unanswered reads in a row, at shutdown and at disconnect. On a bench
without Klipper, `klippy/console.py` lets you type `ace2k_version`.

The module's G-codes:

- always: `ACEK_STATUS`, `ACEK_RESTORE_STOCK`, `ACEK_HEALTH`, `ACEK_COUNTERS_RESET` and
  `ACEK_CALIBRATION_SAVE`;
- with the feed: `ACEK_FEED`, `ACEK_ROLLBACK`, `ACEK_UNLOAD`, `ACEK_LOAD`, `ACEK_ASSIST`,
  `ACEK_STOP`, `ACEK_CLEAR`, `ACEK_SPEED`, `ACEK_CALIBRATE_ENCODER`, `ACEK_LOAD_SET`,
  `ACEK_SNAG_SET`, `ACEK_FOLLOW_SET` and `ACEK_LANE_ACTIVE`;
- with the reader's field: `ACEK_RFID_READ` and `ACEK_RFID_FORGET`;
- with the dryer's outputs: `ACEK_FAN` and `ACEK_FLAP`;
- with the dryer: `ACEK_DRY`, `ACEK_DRY_STOP`, `ACEK_DRY_CLEAR` and `ACEK_DRYER_LOG`.

`README.md` lists each with its arguments. Klipper's G-code parser reads a command name as letters
and underscores only, so a digit cannot appear in one. That is why the G-codes are `ACEK_*` while
the section stays `[ace2k]`. Up to v0.12.0-rc.1 they were `ACE_*`. The prefix
changed to leave the `ACE_*` names free for other ACE modules: Klipper stops on a command
registered twice.

## Host interface

Another Klipper host module can drive the lanes through the feed object of `[ace2k]`, not through
the G-codes: a G-code cannot be run from a timer or an async callback, and these calls can. The
module is `klippy/extras/ace2k_feed.py`; the object is `printer.lookup_object("ace2k").feed`.

`API_VERSION` names this interface. It is stable: what a caller can rely on, and how a change
that would break one is announced, is in [What a caller can rely on](#what-a-caller-can-rely-on).
It is now `28`:

- `1` until v0.8.0, which added `assist_both`;
- `2` until v0.9.0, which added `set_lane_extruder`;
- `3` until v0.11.0, which added the tail (the kinds `tail` and `tail_out`, the lane's `tail` in
  the status);
- then `4` during that release's work, until the kind `blocked`, and the lane's `tail` no longer
  ending on the insert rising, made it `5`;
- `6` with `set_lane_grip` and the grip's bounds;
- `7` with the `ACEK_*` command names (a caller that runs or wraps `ACEK_CLEAR` or
  `ACEK_RFID_READ` checks for it) and `rfid.start_read`;
- `8` with the follow on by default (`follow_default`, `set_lane_follow`, `default_move`,
  `follow_paused`, `hold_follow`, `gone_after_s`, the lane's `follow_default` and `active` in the
  status), one start in flight per lane, and the events' `stop_by`;
- `9` with several units: the events' `unit`, the coordinator `ace2k_units`, and the unit's feed
  reached by its section name (below);
- `10` with the signed `ff_rate` and `ff_back_doses` in the lane status (the reverse);
- `11` with `Units.hand_over`: a host module chooses a shared extruder's active lane (below);
- `12` with `start_move`'s `free_tip` (a feed whose front tip nothing holds, judged by the
  filament; a format change of `ace2k_feed_start`, so a host with `12` refuses an older image);
- `13` with the calls a timer may make in place of three G-codes: `feed.set_speed` for
  `ACEK_SPEED`, the dryer's `start`, `stop` and `clear` for `ACEK_DRY`, `ACEK_DRY_STOP` and
  `ACEK_DRY_CLEAR`, and `Units.release`, which undoes a shared extruder's choice of active lane;
- `14` with `start_move`'s `free_tip` on a rollback (every catch past `free_um` forgiven, on an
  image with `feed.has_free_rollback`) and `ACEK_ROLLBACK`'s `FREE`, and a change of a shared
  extruder's active lane (`hand_over`, `release`) making the feed-forward forget the extruder;
- `15` with `set_lane_park` and `has_park`: a lane's next automatic load to the parking point with
  no tag search ("The park");
- `16` with `feed.snag` documented ("The tip snag").
- `17` with `feed.move_wait_s`: how long a move may take before a caller gives up on it. Its
  keyword `reverse=True` counts the tail that the unit runs after a rollback or an unload.
- `18`: the snag's limits are read from the unit's dictionary at connect (the host's own copy for
  an older image), and a `snag_lag_mm` from 1 up to 1.5 is raised to 1.5. No new call or
  attribute.
- `19`: `feed.set_speed` on a following lane raises `FeedRefused` with the new reason
  `following`. The follow keeps the speed it was armed with. To change it, stop the follow and
  arm it again at the new speed (`stop`, then `start_move(lane, "assist_both", 0.0, speed)`).
  The follow's default arms at the speed given to `set_lane_follow`. An older image would take
  the speed, so the host refuses it itself there, with the same exception. No new call or
  attribute.
- `20` with `start_move`'s `stop_when` and `held_ms`, `feed.has_stop_when`, `feed.held_ms_min`,
  `feed.held_ms_max`, `feed.held_restart_mm` and the final kinds `held` and `at_rest`: a move that
  ends on the buffer's plunger, judged in the unit, with its own held window ("A stop armed for
  the next start"), the `[ace2k]` keys `held_mm` and `held_ms`, and the events' `tag_search`.
  `held_ms`, the three attributes and `tag_search` came before any release with interface 20, so
  `20` has this one shape.
- `21` with `hold_follow`'s `why`, the lane's `follow_why` in the status (why follow_default does
  not arm it, or None), and the `[ace2k]` key `follow_lanes` (`extruder`: follow_default arms only
  a lane that feeds an extruder).
- `22`: the tag read carries a `seq` (`ace2k_rfid_read … seq=%c`, an image since v0.13.0), so a read
  with motion that the transport sends again is the same read and never a second search. An older
  image gets the read without it. No new call or attribute.
- `23`: the health in the status (`printer.ace2k.health`) carries `link_gap_ms`, the longest
  silence of the host on the link since boot or the last clear (`ace2k_link_gap_query`), and
  `link_lost_ms`, the silence the unit takes for a lost link. Both are `None` before the first
  read, after a read whose silence query went unanswered (the self-test's verdict of that read
  stands, and the poll counts no failure), and always on an image without the measure (before
  v0.13.0). `ACEK_HEALTH` prints it,
  and its `CLEAR=1` clears it with the faults.
- `24`: the final kind `stopped_sensors` and the refusal `sensors_stale` (an image since v0.13.0,
  "Stale insert sensors"). The unit stops a running lane, and refuses every start, while its insert
  sensors are stale. `stopped_sensors` leaves the lane idle with no error, yet a waited move and a
  waited `ACEK_RFID_READ … MOVE=1` fail on it, the follow is not armed again after it, and the host
  says one line per stopped lane (`ace2k: lane N stopped (MODE) — insert sensors stale (the ADC
  scan stopped): lanes stopped, starts refused`). An older image never sends either. No new call
  or attribute.
- `25`: the motor's travel each way (`ace2k_lane_motor_query`, an image since v0.13.0). The lane
  status carries `motor_fwd_mm` and `motor_back_mm`, the travel toward the printer and toward the
  spool since boot, in millimetres (three decimals; continuous across the 2³² µm wrap of the
  report), updated at 1 Hz while a watch is on. Both are `None` while no module watches, before
  the first report of a watch, and always on an older image: a watch reads from its own first
  report, so the difference of two of its readings never holds the travel between two watches. The
  status also carries `motor_reports`, the count of motor reports received since Klipper started,
  so a watcher can tell a new report from a second with no travel. `feed.has_motor_travel` says
  whether the image has the report. `feed.motor_watch(on)` turns the report on (`True`) and off
  (`False`); the calls are counted, so the report stops when the last watcher turns it off. A call
  whose command raises is not counted, and a connect forgets every watcher (the unit starts with
  the report off). On an older image it does nothing and returns `False`.
- `26` with `set_lane_wait` and `has_wait`: an insert edge on a lane that starts no automatic load,
  kept through re-insertions until a start or a grip of 0 clears it ("The wait", an image since
  v0.13.0).
- `27` with the dryer's modes (an image since v0.13.0, "The dryer's modes"): the mode object
  `ace2k_dryer_mode` with `dryer_mode.set_mode(mode)` and `dryer_mode.set_material_source(fn)`,
  the mode's fields in the dryer's status, and the events 8–13 of `ace2k:dryer` by name (below);
- `28` when a read without motion (`rfid.start_read`, `ACEK_RFID_READ` with `MOVE=0`) stopped
  attributing a tag: it only reports what is in the field (an image since v0.13.0, "The read on
  command").

**Several units.** A config with one unnamed `[ace2k]` is unchanged: the objects keep the names
`ace2k`, `ace2k_feed`, `ace2k_rfid`, `ace2k_dryer` and `ace2k_dryer_mode`. A named section
`[ace2k <name>]` is one unit, and its objects take the name after a space:
`printer.lookup_object("ace2k <name>").feed`, and `ace2k_feed <name>`, `ace2k_rfid <name>`,
`ace2k_dryer <name>`, `ace2k_dryer_mode <name>`. The name keeps the section's own
spelling (`[ace2k A]` gives `ace2k A` and `ace2k_feed A`); `UNIT=` and `unit:` ignore case. A caller written for one unit keeps
working; to serve several, it asks the coordinator which unit it is talking to.

`printer.lookup_object("ace2k_units")` is the coordinator. It exists whenever an ace2k section does.

- `units.units` is the list of unit objects, in the order of the config. A unit has `name` (lower
  case; `None` for `[ace2k]`), `section` (`ace2k` or `ace2k <name>`), `mcu`, `feed`, `rfid` and
  `dryer`.
- `units.find(name)` returns the unit called `name`, without regard to case. `None` asks for the
  only unit and finds nothing when there are several. An unknown name returns `None`.
- `units.unit_for(gcmd)` returns the unit a G-code command names with `UNIT=`, or the only unit.
  It raises the command's error for an unknown name, and for no name when there are several units.
  A host module that registers its own G-code reads `UNIT=` with it.
- `units.lane_text(unit, lane_index)` is a lane as the console names it: `a:3` with several units,
  `3` with one.
- `units.several()` is true when the config declares more than one unit.
- `units.hand_over(unit, lane_index, pause_wait_s)` (since `API_VERSION` 11) makes the lane the
  active one of its shared extruder, as `ACEK_LANE_ACTIVE` does, but arms nothing: the module moves
  the lane first, then arms it. The extruder's other followers, and lanes in a one-way assist,
  are stopped first, as `ACEK_LANE_ACTIVE` stops them (`stop_by` `command`). One that has not
  ended within `pause_wait_s` raises `RuntimeError`, and the choice stays as it was. Since
  `API_VERSION` 14 a choice that changes the active lane makes every unit's feed-forward forget
  what it recorded for the extruder, once the other followers ended, as `ACEK_LANE_ACTIVE` and a
  `release` that ends do (`rules.md`, the feed-forward's rule 5). The extruder's moves planned
  before the call count as the leaving lane's: a module plans the new lane's moves after it.
- `units.release(extruder_name, pause_wait_s)` (since `API_VERSION` 13) undoes the choice: no lane
  of that shared extruder is active any more. Then every lane in the follow of that extruder, the
  active one included, on any unit, is stopped (`stop_by` `host`), and its end seen. A one-way
  assist is left running: the choice does not govern it. Until a new choice, no follow starts on
  any of them, the default's included, and the console's "choose one" line is said again. An
  extruder that no two lanes share has nothing to release. While `ACEK_LANE_ACTIVE` is arming a
  lane of that extruder, the call raises `RuntimeError` and does nothing. A follower that has not
  ended within `pause_wait_s` raises `RuntimeError`, and the choice is put back, unless another
  was made in the meantime.
- Its status, `printer.ace2k_units`: `units` (the section names as written: `"ace2k a"`, or
  `"ace2k"` for a lone section) and `active_lanes` (extruder to the active lane, `b:3`, or `None`
  before the choice).
- A unit's own `active_lanes` (`printer["ace2k a"].active_lanes`) maps each shared extruder one
  of its lanes feeds to the active lane: the unit's own lane number (`3`) when the lane is its
  own, the other unit's lane as text (`"b:3"`) when it is another unit's, and `None` only before
  the choice. With one unit it is the number or `None`, as before.

The events carry the unit. `"ace2k:feed_event"`, `"ace2k:dryer"` and `"ace2k:tag_read"` are sent
to host modules as a copy of the dict with one more key, `"unit"`: the unit's name, or `None` for
`[ace2k]`. A module that serves one unit filters on it. The event the unit keeps for its status
(`last_event`) has the shape it always had, with no `"unit"`.

The module is `klippy/extras/ace2k_units.py`. The tag cache of a lone `[ace2k]` is
`ace2k_tags_cache.json`; a named unit keeps `ace2k_tags_cache_<name>.json`. The first declared
unit, while it has no file of its own, reads `ace2k_tags_cache.json` if it exists and writes it to
its own file at once, so renaming `[ace2k]` to `[ace2k a]` keeps the cached tags. Once its own
file is written the old file is renamed `ace2k_tags_cache.json.adopted`, its data kept and never
read again: it is adopted once ever, even if the unit's own file is deleted or the sections are
reordered later (a unit's file that cannot be written leaves the old file in place, for the next
start). When you rename `[ace2k]` to add units, keep that
unit's section first: it takes over the old tag cache. The other units start with an empty cache.
Beside each cache, `<cache>.lanes.json` keeps the last read of each lane (the lane number to its
record). After a restart that reset the unit, a `pending` lane with a kept read takes it back,
with `source` `restored`, and `lane_status` shows it `read` while the unit holds the lane
`pending`; `"ace2k:tag_read"` is sent for it. A lane the unit still holds `read` takes its kept
read when the cache has no record of that UID (a tag read by its UID alone). A caller that must
know whether the unit has seen the tag since the restart checks `source`: the unit's next load
still searches. The read goes when the unit's sensor report shows the bay empty, on the
`unknown` event, on a forget, on an accepted read with motion, on `no_tag`, and when a session
opens on the lane with another UID. A session that ends with the same UID alone confirms it
(`source` `cached`), and a `no_tag` no longer drops it. Memory always matches the file: a write
that fails is undone, logged once, and not tried again for 60 s (`ace2k_rfid_kept.py`).

It changes with any change a caller must check for, a call or a mode added as much as one changed,
so a caller checks it once at connect. It is a module constant and an attribute of the feed object:
a caller that holds only the object reads `printer.lookup_object("ace2k").feed.API_VERSION`.

- `feed.start_move(lane_index, mode, length_mm, speed_mm_s, free_tip=False, *, stop_when=(),
  held_ms=None)` →
  a `Move`. Starts `mode` on `lane_index` (0–3) and returns once the unit answered; it does not
  wait for the move to end. `free_tip` (since `API_VERSION` 12) is true for a feed whose front tip
  nothing holds (a push to a head); since `API_VERSION` 14 also for a rollback whose tip is out of
  the extruder, which then forgives every catch past `free_um` from the last ("The free tip"). An
  image without it (`feed.has_free_rollback` false) ignores it on a rollback: one catch, as
  before. Every other mode ignores it; an unload always, so a tip the head still holds ends it
  `stuck`.
  `mode` is one of `feed`, `rollback`, `assist`, `assist_back`, `assist_both` (the follow:
  `start_move(lane, "assist_both", 0.0, speed)`), `unload`, `load`. Lengths and speeds are those
  the G-codes take, in mm and mm/s: the speed within the dictionary's bounds
  (`feed.speed_min`..`feed.speed_max`), and `length_mm` at most `feed.move_max_mm`. The length is 0
  for an assist; 0 for an unload takes the unit's default budget. A speed or length out of bounds
  is the unit's refusal (`bounds`); the host does not check them here. The start carries the lane's
  next sequence, as a G-code's does ("The start's sequence").
  - `stop_when` (since `API_VERSION` 20, a keyword only) names the stops on the buffer's plunger
    that also end the move, judged in the unit every 10 ms ("A stop armed for the next start"):
    `"held"` and `"at_rest"`, one or both, in any collection (a set, a tuple). `"held"` ends a move
    toward the spool whose tip is held: the plunger stays pulled tight while the filament stands
    still, and the move ends with the kind `held`. It is meant for a rollback or an unload that
    pulls a tip out of the head. `"at_rest"` ends the move when the plunger is back at rest, in
    either direction, with the kind `at_rest`. Both kinds are final and no error, and the lane is
    idle after them. The default, empty, sends the start exactly as before, and so does `None`.
    Otherwise the host sends `ace2k_feed_stop_when` and then the start, and the stop belongs to
    that start only. The image must have it: `feed.has_stop_when` says so from connect on.
    `"held"` is taken only on `rollback` and `unload`, the modes that move toward the spool from
    start to end. The reverse assist (`assist_back`) ends its take-up as soon as the plunger reads
    taut and then waits with no move, so held could never end it. Taking more modes later would
    only add to the interface. `"at_rest"` is taken on every mode but `assist_both`.
  - `held_ms` (since `API_VERSION` 20, a keyword only) is the held window of this move, in whole
    milliseconds within `feed.held_ms_min`..`feed.held_ms_max` (50–2 000). It goes with the
    arming, so it belongs to this move only: two moves on the unit never share a window. `None`,
    the default, sends 0, and the unit uses its own `held_ms` (the `[ace2k]` key). The pull against
    a held tip is about the speed times the window, so a faster move takes a shorter window. The
    window must still tell friction from a held tip: its friction floor,
    `feed.held_restart_mm / held_ms`, must stay at or under 95 % of `speed_mm_s`. A window with
    the unit's own `held_ms` is checked once, at connect, against the lane's least speed.
  - With `stop_when`: `ValueError` for anything but a collection of those two names (a plain
    string, a number and an element that cannot be hashed included); for `"held"` on another mode
    (`ace2k: stop_when held on load: held is taken only on rollback and unload`); and
    for `"at_rest"` on `assist_both` (`ace2k: stop_when at_rest on assist_both: the follow keeps
    the plunger at rest, so at_rest would end it at once`). `ValueError` for a `held_ms` that is
    not a whole number (`ace2k: held_ms 120.5: a whole number of milliseconds`), is out of the
    bounds (`ace2k: held_ms 49: the unit takes 50 to 2000`), comes without `"held"` (`ace2k:
    held_ms 120 without held: the window is held's own`) or fails the friction floor (`ace2k:
    held_ms 60 at 50 mm/s: friction can end it held below 74.0 mm/s, more than 95 % of its speed;
    raise held_ms`). `RuntimeError` (`ace2k: this image
    cannot stop on the plunger`) when `feed.has_stop_when` is false. Nothing is sent in those
    cases, nor for a length or a speed that is not a finite number. The unit refuses the stop on a
    lane that is not idle: `FeedRefused` with the start's `.mode` and `.reason` `busy`, or
    `in_error` (`feed.clear` the lane first), and no start is sent. A window it does not take is
    `FeedRefused` with `.reason` `bounds`, and no start is sent. When the unit does not answer
    the stop, or answers for another lane, the host sends `held=0 at_rest=0 held_ms=0` to disarm
    it and raises `NoAnswerError`, with no start sent. When the start then gets no answer, the lane is
    stopped as for any start and the stop is disarmed too. When such a disarm is refused or gets
    no answer, the lane's next start, through this interface or a G-code, first sends its own stop
    (`held=0 at_rest=0 held_ms=0` when it has none), so a stop left behind never ends it.
  - Raises `FeedRefused` when the unit refuses: `.reason` is the refusal's name (`busy`,
    `in_error`, `no_filament`, `no_link`, `bounds`, `other_lane`, and since `API_VERSION` 26
    `sensors_stale`), `.lane` and `.mode` the start's.
    The class is defined in `ace2k_link.py`. `ace2k_feed.py` exports it too, so a caller imports it
    with `from ace2k_feed import FeedRefused`.
    Since `API_VERSION` 9 the host refuses `assist_both` itself, nothing sent, with `.reason`
    `not_active`, on a lane whose extruder other lanes feed too, on any unit, when it is not that
    extruder's active lane (`ACEK_LANE_ACTIVE`). The coordinator's `may_follow(unit, lane)`
    (`printer.lookup_object("ace2k_units")`) answers the same question before a module arms, and
    its `shared_lanes(unit, lane)` gives the lanes that share the lane's extruder as the console
    names them (`"a:4, b:1"`), or `None` when the extruder is the lane's own.
    Nothing is left registered: a move already running on the lane (refused `busy`) stays the
    lane's tracked move, and completes on its own event. Since `API_VERSION` 8 a start on a lane
    whose earlier start still waits for the unit's answer is refused `busy` on the host, nothing
    sent: one start in flight per lane, whoever sends it (another module, a G-code, the follow's
    default).
  - Raises `ValueError` for a lane or mode outside the lists above (nothing sent). Raises
    `RuntimeError` when the firmware was built without the feed, when no state report came within
    the wait a G-code allows before its first start, when the mode is `assist_both` on an image
    without the follow (`the unit's firmware has no follow tail (or no follow); flash v0.11.0`,
    nothing sent), or when the unit did not answer the start, or answered for another lane
    (`ace2k: lane N MODE: no answer from the unit; check the link (...); lane N stopped in case it
    started`). The move may have started, so the lane is stopped first. The feed object's
    `has_follow` says, from connect on, whether the image has it.
    Since v0.11.0 that means the follow with its tail. An image older than v0.11.0, whose
    dictionary lacks the tail's constants, has no follow to the host: `ACEK_FOLLOW_SET`,
    `ACEK_ASSIST … DIR=BOTH` and an automatic encoder calibration's arm are refused with that line,
    and a `follow_*` key is a config error. The plain feeds and the two assists work as before.
- `Move`: `.lane`, `.mode`, `.seq`; `.done()` (true once the final event landed); `.result()` (that
  event, or `None` before; never blocks); `.wait(timeout_s)` (blocks the calling greenlet until the
  event or `timeout_s`; returns the event or `None`). A move completes on the first event of its
  lane that echoes its sequence and is not a notice: `done`, `stopped`, an error kind and the rest
  of [Feed events](#feed-events). `behind`, `snag` and `tail` do not complete it. A follow's
  `tail_out` does. `held` and `at_rest` (since `API_VERSION` 20) complete the move armed with
  them, as final kinds that are no error. `stopped_sensors` (since `API_VERSION` 26) completes
  the move too, with no error state on the lane. `blocked` completes a feed or a load. It is final, not
  an error: a waited `ACEK_FEED` or `ACEK_LOAD` answers `ace2k: lane n feed blocked — motor …
  mm, filament … mm` (`load blocked` for a load), not a G-code error. One move per lane is
  tracked. A later start on the same lane, through this interface or a waiting G-code, takes the
  lane's slot, and the earlier `Move` then never completes. A timeout stops nothing; `stop` is the
  caller's.
- `feed.stop(lane_index, by="host")` sends `ace2k_feed_stop` (`LANE_ALL` = 255, every lane); it
  never clears an error. `by` (since `API_VERSION` 8) labels the stop on the `stopped` event of the
  motion it stops: `"host"` for a host module, `"command"` for a G-code (`ACEK_STOP` and
  `ACEK_ASSIST … OFF=1` pass it). The label is kept by that motion's seq and only on a lane with a
  motion running or a start in flight (an idle lane's stop brings no event and leaves none); a
  `"command"` is never replaced by a `"host"` stop of the same motion. `feed.clear(lane_index)`
  sends `ace2k_feed_clear`, leaving the lane's error state. Both raise `ValueError` for a lane
  outside 0–3 (`LANE_ALL` is accepted by `stop` only), nothing sent, and `RuntimeError` on an image
  without the feed, as every other call does. On such an image `stop` and `clear` always raised.
  Only the type of the error is new: it was an `AttributeError` before, and it is `RuntimeError`
  now, like the other calls.
- A lane number is an `int` from 0 to 3. `True` and `False` were never valid lanes. Every call
  that takes a lane now refuses them with the same `ValueError` as any other lane outside 0–3.
  Before, most calls took `True` as lane 1 and `False` as lane 0.
- `feed.set_speed(lane_index, speed_mm_s)` (since `API_VERSION` 13) changes the speed of the move
  running on `lane_index` (0–3), as `ACEK_SPEED` does, and returns once the unit took it. It waits
  for the unit's answer only. Raises `ValueError` for a lane outside 0–3 or a speed outside
  `feed.speed_min`..`feed.speed_max` (nothing sent); `FeedRefused` with `.mode` `speed` when the
  unit refuses it: `.reason` `no_move` (nothing runs on the lane: an idle lane keeps no speed for
  a later move), `in_error`, `following` (since `API_VERSION` 19: the lane is following, and the
  follow keeps the speed it was armed with; on an image without `ACE2K_FEED_REFUSED_FOLLOWING`
  the host raises it itself and sends nothing) or `bounds`; `NoAnswerError` (a `RuntimeError`)
  when the unit did not answer, or answered for another lane; `RuntimeError` on an image without
  the feed. A feed, a rollback, an unload, a load and the two one-way assists take a new speed.
  The follow does not.
- The dryer (since `API_VERSION` 13), on the unit's dryer object
  (`printer.lookup_object("ace2k_dryer")`, or `ace2k_dryer <name>`; the coordinator's unit carries
  it as `dryer`): `dryer.start(target_c, minutes)` starts a cycle at `target_c` (whole degrees,
  15–65) for `minutes` (whole, 1–1 440), as `ACEK_DRY` does; `dryer.stop()` stops it, as
  `ACEK_DRY_STOP` does; `dryer.clear()` leaves a fault, as `ACEK_DRY_CLEAR` does. Each returns once
  the unit took it, and waits for the unit's answer only. Each raises `DryerRefused`
  (`ace2k_dryer.DryerRefused`) when the unit refuses: `.what` is `start`, `stop` or `clear`,
  `.code` the unit's number, and `.reason` the refusal's name ("The dryer", refusals: `busy`, `range`, `faulted`, `sensors`,
  `no_mains`, `cutout`, `not_cool`, `shutdown`, `measuring`). `measuring` is transient: ask again
  in a few seconds. Each raises `ValueError` for an argument out of range (nothing sent),
  `NoAnswerError` (a `RuntimeError`) when the unit did not answer, and `RuntimeError` on an image
  without the dryer. The state stays in `printer["ace2k_dryer <name>"]`, and the changes come as
  the `ace2k:dryer` event.
- The dryer's mode (since `API_VERSION` 27), on the unit's mode object
  (`printer.lookup_object("ace2k_dryer_mode")`, or `ace2k_dryer_mode <name>`; the unit carries it
  as `dryer_mode`):
  - `dryer_mode.set_mode(mode)` chooses the mode, as `ACEK_DRY_MODE` does: `"manual"`,
    `"humidity"` or `"print"`. It returns once the unit took it. It raises `ValueError` for another
    name, and `RuntimeError` for `"humidity"` without `unattended_drying: True` or on an image
    without the modes; nothing is sent then. It raises `ace2k_dryer_mode.ModeRefused` (a
    `RuntimeError`, `.reason` `range`) when the unit refuses, and `NoAnswerError` when it does not
    answer. The unit keeps the mode across a power loss. A humidity mode command also clears the
    mode's `stopped` state and its failure count.
  - `dryer_mode.set_material_source(fn)` gives each lane's material for the profile:
    `fn(lane_index)` returns the material's name, or `None` to use the tag's. `None` as `fn` goes
    back to the tags. The profile is the lowest drying temperature of the lanes with a strand at
    the insert sensor, by the material's family (PLA 45 °C, PETG 60, ABS 65, TPU 60, all 360 min;
    `PLA+` and `PLA-CF` are PLA; an unknown name and no strand are PLA). The host compares it with
    the unit's on a report, at most every 5 s, and sends it when it differs. So a more fragile
    strand reaches the unit within about 6 s (the next report, then the 5 s wait), and lowers a
    running automatic cycle there.
  - The connect: the only init command is the report query (`ace2k_dryer_auto_query`). On the
    first report, and on a report at most every 5 s after it, the host sends with a query, and
    checks the answer of, in this order: the limits when the unit's differ from the config, the
    timings once per connect, the mode `manual` while the unit holds `humidity` and
    `unattended_drying` is False (said on the console; a failure is said once and tried again),
    and the profile when it differs. A failed request is logged once and tried again 5 s later.
    The host never sends `humidity` at connect: that command clears the mode's `stopped` state.
  - `dryer_mode.start_bound(target_c, minutes)` is internal (the print mode's): a start the unit
    ends when the host link stays down for 5 min. `dryer_mode.stop_bound()` is internal too: it
    stops the running cycle only if the unit still holds it bound.
  - The status: `printer["ace2k"].dryer` gains `mode` (`manual`, `humidity`, `print`),
    `humidity_allowed` (the `unattended_drying` option), `limit` (%), `max_failures`,
    `failures`, `stopped` (the humidity mode stopped after its failures), `cold` (the box reads
    cold), `above_s` (seconds the RH has stayed at or above the limit), `own` (the running cycle is the
    humidity mode's), `bound` (it is bound to the host), `profile`
    (`{"temp": °C, "minutes": m}`) and `rh` (the unit's last idle RH, the one it judges, in whole
    %, rounded half up), as the unit last reported them. All but `humidity_allowed`
    are `None` before the first report and always on an older image.
  - The events 8–13 of `ace2k:dryer` carry their names: `auto_started` (`arg` °C),
    `auto_dry` (`arg` the cold RH, %), `auto_failed` (`arg` the failures in a row),
    `auto_stopped` (`arg` the failures), `auto_off` and `stopped_link`. Each of these six is also
    said once on the console, with the line the log gets; a resent event is not said again.
- `feed.set_lane_extruder(lane_index, extruder_name)` (since `API_VERSION` 3) maps `lane_index`
  (0–3) to the extruder it feeds, for the feed-forward ("The feed-forward"); `None` unmaps it. A
  `laneN_extruder` key of `[ace2k]` wins: the call is then only logged. Called after ready, it
  hooks that extruder's planned moves at once. Moving a lane that already has a rate sent to
  another extruder returns its rate to 0 first. Several lanes may name one extruder. A plain call:
  it sends nothing but that 0, and never yields. Raises `ValueError` for a lane outside 0–3. On an
  image without the feed-forward the map is kept and nothing is hooked.
- `feed.set_lane_grip(lane_index, grip_mm)` (since `API_VERSION` 6) sets the grip of `lane_index`
  (0–3) ("The grip"). Its next automatic load pulls only `grip_mm`, without the tag search, and
  ends `loaded`. That load uses up the grip, and any start on the lane, or a read with motion,
  clears it. `0` clears it. A plain send that never yields. Raises `ValueError`, nothing sent, for a
  lane outside 0–3 or a grip outside `feed.grip_min_mm`..`feed.grip_max_mm` (the unit would shut
  down on it). Raises `RuntimeError` when the firmware has no feed, or no grip (`the unit's firmware
  has no grip; flash v0.11.0`). The feed object's `has_grip` says, from connect on, whether the
  image has it. `feed.grip_mm`, `feed.grip_min_mm` and `feed.grip_max_mm` are the dictionary's
  default and bounds in mm (40, 10 and 45 mm: `ACE2K_FEED_GRIP_UM`, `_MIN`, `_MAX`), or `None`
  without it. The host clears every lane's grip as an init command at every connect.
- `feed.set_lane_park(lane_index)` (since `API_VERSION` 15) sets the park of `lane_index` (0–3)
  ("The park"). Its next automatic load pulls to the parking point, reading the tag on the way,
  with no tag search past it, and ends `loaded`. It takes the grip's place: used up by that load,
  cleared by any start on the lane, by a read with motion and by `set_lane_grip(lane_index, 0)`,
  and replaced by a grip set after it. A plain send that never yields. Raises `ValueError`,
  nothing sent, for a lane outside 0–3. Raises `RuntimeError` when the firmware has no feed, or no
  park (`the unit's firmware has no park; flash v0.12.0 or later (0.12.0-rc.1 has none)`). The feed object's `has_park` says, from
  connect on, whether the image has it (only an image with the grip can).
- `feed.has_stop_when` (since `API_VERSION` 20) is true, from connect on, when the image can stop a
  move on the plunger: its dictionary has `ace2k_feed_stop_when`, its answer, `ace2k_feed_held_set`
  and the `ACE2K_FEED_HELD_*` constants, with `ace2k_feed_stop_when` in its format with
  `held_ms`. It is false before connect and on an older image. The constants without the
  commands, or a command in another format, refuse the connect.
- `feed.held_ms_min` and `feed.held_ms_max` (since `API_VERSION` 20) are the bounds of a move's
  held window in ms (`ACE2K_FEED_HELD_MS_MIN`, `_MAX`: 50 and 2 000). `feed.held_restart_mm` is the
  filament travel that restarts a window: `held_mm` rounded up to whole encoder counts at the
  widest scale (1 481 µm a count), 4.443 mm with the default 3 mm. A window of `held_ms` lets
  friction end a move `held` below `held_restart_mm × 1000 / held_ms` mm/s. All three are `None`
  before connect and on an image without the stop.
- The `[ace2k]` keys `held_mm` and `held_ms` (since `API_VERSION` 20) tune the held stop: a
  filament that moves `held_mm` or more starts the window again, and the plunger must stay pulled
  tight for `held_ms` ("A stop armed for the next start"). The defaults and the limits are the
  unit's (`ACE2K_FEED_HELD_UM`, `ACE2K_FEED_HELD_MS`, each with `_MIN` and `_MAX`: 3 mm in
  1.5–20 mm, 550 ms in 50–2 000 ms). A key out of the limits is a config error at connect. Both
  values go to the unit as an init command at every connect, so they are not part of the
  config's CRC. No G-code sets them, and `SAVE_CONFIG` stages nothing for them. On an image
  without the stop they are ignored, with one warning line in `klippy.log`. Values that let
  friction end a slow move `held` also get one warning line: the friction floor,
  `ceil(held_um / count) × count / held_ms` at the widest scale (1 481 µm a count), above 95 % of
  the lane's least speed.
- `feed.move_wait_s(lane_index, length_mm, speed_mm_s, *, reverse=False)` (since `API_VERSION` 17)
  returns, in seconds, how long a move of `length_mm` at `speed_mm_s` on `lane_index` (0–3) may
  take before a caller gives up on it. It is the budget ace2k's own waited G-codes use: the
  expected travel time, with the landing (the last `ACE2K_LANE_LANDING_UM` of it, or the whole of a
  shorter move) at the unit's floor speed `speed_min`, times 1.5, plus 5 s. A plain calculation
  that sends nothing; it does not check the speed against the unit's bounds. Raises `ValueError`
  for a lane outside 0–3, a negative or non-finite length, a speed not above 0 or non-finite, and
  `RuntimeError` when the firmware has no feed.
  - `reverse` is a keyword only. Pass `reverse=True` for a rollback or an unload. Once the
    filament leaves the motor, the unit can go on with a tail: a second move of up to
    `ACE2K_FEED_UNLOAD_TAIL_UM` (100 mm) of motor, with its own landing and its own deadline (a
    rollback's tail is at most what its length still owes, since v0.13.0). With
    `reverse=True` the budget counts that tail, the same way the waited `ACEK_ROLLBACK` and
    `ACEK_UNLOAD` count it. Without it a reverse move that runs into its tail can outlast the
    budget. The default `False` gives the same number as before the keyword existed.
- `feed.snag` (since `API_VERSION` 16) is a dict of the tip snag's values in effect ("The tip
  snag"). Its keys are the six `snag_*` keys of `[ace2k]`, in the units of the key: `snag_fwd_mm`
  (mm), `snag_lag_mm` (mm), `snag_duty_pct` (%), `snag_back_mm` (mm), `snag_rest_ms` (ms) and
  `snag_free_mm` (mm). The values are numbers. It is empty until the host connects to the unit.
  From then on it holds the `printer.cfg` value of each key, or the firmware's default when the key
  is not there (20, 5, 0, 5, 500 and 50). `ACEK_SNAG_SET` replaces it with the new values. A caller
  reads it and never writes to it. A caller that needs the current values reads `feed.snag` again
  and does not keep the dict.
- `feed.follow_default` (since `API_VERSION` 8) is the `[ace2k]` key: whether ace2k arms the
  follow by itself ("The follow on by default", above).
- `feed.set_lane_follow(lane_index, speed_mm_s=None, on_insert=True)` (since `API_VERSION` 8) sets
  how that default arms `lane_index` (0–3): at `speed_mm_s` (`None`: `assist_speed`) and, with
  `on_insert=False`, only at the start. A module that runs its own flow when a filament is inserted
  (the U1 adapter's preload) takes the insertions that way and arms the lane itself once its flow
  ends. A plain call, from config time on; it sends nothing. Raises `ValueError` for a lane outside
  0–3 or a speed not above 0.
- `feed.follow_paused(lane_index)` (since `API_VERSION` 8) is true while a G-code move holds the
  lane: from the stop of its follow until that move's own final event (or its refusal, or the
  pause's timeout). A module that arms the lane's follow itself checks it first and arms nothing
  meanwhile; the move's end re-arms or not by ace2k's rule.
- `feed.hold_follow(lane_index, held)` (since `API_VERSION` 8): while held, the follow's default
  arms nothing on the lane — not at the start, not at an insertion, not after a G-code move. The
  U1 adapter holds a lane whose filament waits behind another piece. A plain call.
- `feed.gone_after_s` (since `API_VERSION` 8): the span after which a start the unit accepted,
  reported idle (or in error) without its final event, is over — `GONE_REPORTS` (2) state reports
  after its acceptance. ace2k then drops it from its record (a G-code move is no longer paused for
  it, `default_move` no longer returns it); a module that tracks the lane's motions applies the
  same span. Inside ace2k, a final event of that start that comes before the next state report,
  with no start accepted on the lane meanwhile, still ends that start: the unit can send a due
  report just before the final event of the same pass. ace2k then re-arms a follow that the move
  paused (unless a stop came meanwhile), and the event carries the `stop_by` of the stop sent to
  the move. The span and the rule did not change.
- `feed.default_move(lane_index)` (since `API_VERSION` 8) is the `Move` of the follow the default
  armed on `lane_index` while its final event has not come, else `None`. A module that tracks its
  lanes' motions adopts it from there. Raises `ValueError` for a lane outside 0–3.
- `rfid.start_read(lane_index)` (since `API_VERSION` 7, on `printer.lookup_object("ace2k").rfid`)
  asks the unit to report what is in the field of `lane_index`'s antenna (0–3), without motion,
  and does not wait: what `ACEK_RFID_READ LANE=n MOVE=0 WAIT=0` does. Since `API_VERSION` 28 (an
  image since v0.13.0) a read without motion attributes no new tag: the answer is `ambiguous` or the
  unchanged state, unless the lane's own UID, attributed earlier by its motion, is in the field and
  reopens its session ("The read on command"). Callable from a timer. Every error it raises is a `ValueError` or a `RuntimeError`:
  `ValueError` for a lane outside 0–3 (nothing sent); `RuntimeError` when the firmware has no tag
  reader or the unit refuses (`ace2k: lane N tag read refused: REASON`); `NoAnswerError`, a
  `RuntimeError` subclass, when the unit does not answer.
- Every event, final or notice, is also the Klipper event `"ace2k:feed_event"` with
  `(lane_index, event)`. `event` is a dict with `kind`, `mode` (the names of
  [Feed events](#feed-events)), `motor_mm`, `filament_mm`, `seq` and `stop_by`: the same dict
  `Move.result()` returns. `stop_by` (since `API_VERSION` 8) is who sent the stop a `stopped` event
  answers: `"command"` (a G-code), `"host"` (a host module, ace2k itself included: the stop after a
  start with no answer) or `None` (no stop of this session sent to that motion). It is `None` on
  every other kind. The motion's final event, whatever its kind, takes the label. A final event
  completes its `Move` first, and is sent as the Klipper event after. So a listener sees the move
  already done, and a listener that raises cannot keep it from completing. Since `API_VERSION` 9
  the dict handed to listeners also carries `unit`. Since `API_VERSION` 20 it also carries
  `tag_search`: `True` on the `loaded` event that ends a read with motion (the search went out and
  came back, so `filament_mm` is about 0), `False` on every other event.
- A lane's `follow_default` in the status (`printer.ace2k.lanes[n]`, since `API_VERSION` 8):
  `waiting` (the default will arm the lane once it is free), `arming` (its start in flight),
  `armed` (its follow is the lane's current motion) or `off`; its `active`: `False` for a lane of
  an extruder several lanes share that is not the one chosen with `ACEK_LANE_ACTIVE`, else `True`.
- A lane's `tail` in the status (`printer.ace2k.lanes[n]`, since `API_VERSION` 4) is True from the
  lane's `tail` notice until the lane leaves `following`: its `tail_out` or any final event, or a
  state report showing another mode. Since `API_VERSION` 5 the insert rising no longer ends it: a
  filament pushed in at the bay during the tail changes nothing.
- A lane's `encoder_mm` in the status (`printer.ace2k.lanes[n]`, three decimals) is the unit's
  `encoder_um` in millimetres, plus a host offset that keeps it continuous across a live scale
  change (since v0.10.0: an automatic encoder calibration's, `ace2k_lane_scale_set`). The unit
  converts its whole count with the new scale, so `encoder_um` jumps by the count times the change:
  1 % of the reading for a 1 % change. The offset is the count at the change times the old scale
  less the new. It goes in with the first frame at the new scale (a frame the unit sent before it
  took the change still reads at the old one), and at the latest with the first frame received a
  report period (1 s) and 0.2 s after the change. A counters reset drops it, and a change still
  pending with it. A module that reads `encoder_mm` as motion sees none that did not happen;
  `encoder_um` beside it stays the unit's own reading.

Contexts: G-code handlers, reactor timers and async callbacks. All of them run in reactor greenlets.
`start_move` sends a query and yields until the unit answers (and, before the session's first state
report, until that report), so it must not be called from the serial thread or a `get_status`.
Every query of the host modules to the unit holds one mutex: a `start_move` from a timer waits
while a G-code's query is in flight, and the reverse. Klipper keeps one handler per response name,
so two overlapping queries would take each other's answer.
`stop`, `clear` and `set_lane_grip` are plain sends and never yield.

### What a caller can rely on

This interface is stable. A module written against it keeps working with every later ace2k, until
a change is announced as described below. This section says what is covered, what may change in a
new version, what never changes without notice, and what a caller does on its side.

**What is covered.** Everything this section names, and nothing else:

- the objects and their lookup names (`ace2k`, `ace2k <name>`, their `feed`, `rfid`, `dryer` and
  `dryer_mode`, `ace2k_units`);
- every call and attribute listed here, with its parameters (their order, units and defaults), what
  it returns and the exceptions it raises: `feed.*`, `rfid.start_read`, `dryer.start` / `stop` /
  `clear`, `dryer_mode.set_mode` / `set_material_source`, `units.*`, and the `Move` object;
- the status fields named here (`printer.ace2k`, `printer["ace2k <name>"]`, their `lanes[n]`
  fields, `printer.ace2k_units`), with their units and their values;
- the Klipper events `"ace2k:feed_event"`, `"ace2k:dryer"` and `"ace2k:tag_read"`, the keys of
  their dicts, and the event kinds and mode names of [Feed events](#feed-events).

**What is not covered.** It may change in any version, with no notice:

- any name that starts with `_`, and any attribute, function or module this section does not list;
- the text of console and log lines. Never parse it: read the status, the `Move` or the events;
- the MCU commands and responses between the host module and the firmware (the tables above). The
  host module and the image are released together, and the host refuses an image it does not
  match.

The G-code commands are a separate surface, described in [`commands.md`](commands.md).

**What a new version may do.** A new `API_VERSION` only adds:

- a new call, attribute, status field, event key, event kind or mode;
- a new parameter, always optional, whose default keeps the earlier behaviour;
- a new exception that is a subclass of one already documented (as `NoAnswerError` is a
  `RuntimeError`), so a caller that catches the documented one still catches it;
- a new value of a field whose values this section says may grow (a new event kind, a new refusal
  reason).

`API_VERSION` rises only with a public release, and only when that release adds to this section: by
one, whatever the number of additions it carries. Each addition is listed under that number in the
list at the top of this section, and has a line in the changelog.

**What never changes without notice.** These are breaking changes:

- removing or renaming anything that is covered;
- changing a parameter's order, unit, meaning or default, or making an optional parameter required;
- changing what a call returns, or the exception it raises for a given case (other than a subclass,
  above);
- changing a status field's or an event key's unit, type or meaning, or the meaning of one of its
  values;
- changing what a call does in a way a caller would have to adapt to: what it moves, stops, waits
  for or leaves running.

A breaking change follows four steps:

1. It is made only when nothing else will do: a safety problem, or a behaviour that cannot be kept
   correct.
2. It is announced one release ahead. That release's changelog has a **Deprecated** line saying what
   goes, why, and what replaces it, and the replacement already works in that release, beside the
   old form.
3. The release that makes the change raises `API_VERSION` and the minor version (`v0.13.0`, never
   `v0.12.1`). Its changelog has a **Breaking** line saying what a caller must change.
4. The list at the top of this section says, at that number, what changed and what to do instead.

A safety fix may skip step 2 when keeping the old behaviour for one more release would risk the
hardware or the print. It still has the **Breaking** line, with its reason.

**What a caller does.**

- Read `API_VERSION` once at connect. Require a **minimum**, the lowest version that has everything
  the module uses, never an exact number: a later version only adds, so a higher number keeps
  working.
- Ignore a status field, event key, event kind or mode that you do not know.
- Bound your own wait for a move with `feed.move_wait_s` (`API_VERSION` 17 or later) rather than a
  multiple of its length over its speed: the move's slow landing at the unit's floor speed is part
  of its time, and it is the same budget ace2k's own G-codes wait by. Pass `reverse=True` for a
  rollback or an unload, so the budget counts its tail. A timeout stops nothing.
- Arm a stop (`stop_when`, `API_VERSION` 20 or later) only on a move you mean to end on the
  plunger: it applies to whatever mode the start enters. `held` is for a rollback or an unload,
  such as an unload that pulls a tip out of the head. Check `feed.has_stop_when`
  first, and keep a path for an image without it.
- Wait on a `Move` (`done()`, `result()`, `wait()`) rather than on a list of final kinds. The host
  module decides which events end a move, so a new kind needs no change on your side.

```python
api = printer.lookup_object("ace2k").feed
if getattr(api, "API_VERSION", 0) < 13:
    raise config.error("this module needs the ace2k host interface 13 or later")
```

**The record so far.** From 1 to 20 no covered call, field or key was removed or renamed. Two
versions changed something that already existed. Both came before this rule, and both are in the
list above: `5` (a lane's `tail` no longer ends when the insert rises) and `7` (the `ACEK_*`
G-code names, which matter to a caller that runs a G-code).
