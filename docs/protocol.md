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
  clock ticks, as Klipper's own `query_analog_in` takes it; 0 stops them. A report is not a query
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
  with `low_mv >= high_mv`.

## Dictionary — current

| Command | Response | Since |
|---|---|---|
| `ace2k_version` | `ace2k_version_response version=%s` — the ace2k release string. **The first answered `ace2k_version` is the proof of the host link.** An image that is flashed and never queried resets itself into the bootloader's recovery after 180 s (see below) | v0.1.0 |
| `ace2k_linkproof_state` | `ace2k_linkproof_state_response proven=%c remaining_ms=%u` — `proven` is 1 once this image has been queried. `remaining_ms` is the time left in the window while it has not | v0.1.0 |
| `ace2k_link_query` | `ace2k_link_state deferred=%u rearmed=%u idle_us=%u` — counters since boot: how often the unit had to wait for the line before it transmitted, how often a wait was extended because bytes kept arriving, and the idle time it waits for. Read-only. These are the counters of the half-duplex turnaround ("Link", below) | v0.2.0 |
| `ace2k_bootloader_enter` | `ace2k_bootloader_enter_response accepted=%c` — the response is sent first. When `accepted` is 1, the unit stops feeding its watchdog 50 ms later and resets into the bootloader's recovery mode (`flashing.md`). The command is refused (`accepted=0`) while a subsystem objects: the heater is active (a lease, its last half-cycle, or the gate reading high, even latched); the fans are required (a dryer cycle until its cool-down hands them back, or an outlet NTC that reads valid and is still at or above 42 °C while the 45 °C rule holds them: the bootloader drives no fans); or a lane is moving. The unit asks the vetoes again, with interrupts masked, right before the reset. A veto that appeared in those 50 ms cancels the reset, and the unit keeps running: ask again | v0.1.0 |
| `ace2k_config_state` | `ace2k_config_state_response page=%c calibration_source=%c` — `page`: the state of the ace2k config page: 0 valid, 1 blank (every boot after a flash), 2 corrupt. `calibration_source`: where the insert bands came from at boot: 0 the built-in defaults, 1 the unit's factory page (`hardware.md`, "Factory calibration page") | v0.2.0 |
| `ace2k_sensors_query rest_ticks=%u` | periodic `ace2k_sensors_state switches=%hu insert_mv=%*s empty_mv=%*s aux_mv=%hu age_ms=%c` — `switches`: bits 0–3 *insert* (filament present) for lanes 1–4, bits 4–7 *rest*, bits 8–11 *pushed*, bit 12 *pulled* (any lane), bit 13 *cutout*; every bit is debounced. `insert_mv`, `empty_mv`: `uint16 × 4` little-endian, raw millivolts from the last ADC pass. `aux_mv`: the auxiliary input (PC1). `age_ms`: the age of that pass when the report is sent, capped at 255. The host asks for 10 Hz | v0.2.0 |
| — | `ace2k_sensors_event lane=%c kind=%c level=%c` on every debounced edge — `kind`: 0 insert, 1 rest, 2 pushed, 3 pulled, 4 cutout. `lane`: 0–3, or 255 for the two shared inputs (pulled, cutout). `level`: 1 = asserted (for *insert*: filament present) | v0.2.0 |
| `ace2k_sensors_thresholds_set lane=%c low_mv=%hu high_mv=%hu` | none. Sets the insert band of one lane in millivolts, `low_mv < high_mv`, with source *config*. It takes effect from the next tick, and the debounced state is kept. The host sends one per `laneN_insert_mv` key of `printer.cfg` as a **config** command. So a changed or removed override restarts the MCU, which then reads its factory page again | v0.2.0 |
| `ace2k_sensors_thresholds_query` | `ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s` — one frame for the four lanes. `low_mv`, `high_mv`: `uint16 × 4` little-endian, the bands in use. `source`: one byte per lane: 0 default (600 / 1100 mV), 1 factory, 2 config | v0.2.0 |
| `ace2k_env_query rest_ticks=%u` | periodic `ace2k_env_state ptc_left_mc=%i ptc_right_mc=%i chamber_mc=%i chamber_rh_pct10=%hu vdda_mv=%hu valid=%c` — the two outlet NTCs and the chamber temperature in millidegrees Celsius, the chamber humidity in tenths of a percent, and the supply computed from the internal reference. `valid` bits: 1 `ntc_left`, 2 `ntc_right`, 4 `chamber`, 8 `vdda`. A field whose bit is clear carries its last value or 0, never a sentinel. The host asks for 1 Hz | v0.2.0 |
| `ace2k_lane_counters_query rest_ticks=%u` | periodic `ace2k_lane_counters encoder_um=%*s fg=%*s` — `encoder_um`: `int32 × 4` little-endian, the filament encoder in micrometres at the lane's scale (1234.2 µm per count until `ace2k_lane_scale_set` names another). It is signed, positive toward the printer (`hardware.md`, "Counters"), and saturates at ±2³¹. `fg`: `uint32 × 4` little-endian, the motor's tach pulses since boot or the last reset. The host asks for 1 Hz. During an automatic encoder calibration (`ACE_CALIBRATE_ENCODER … AUTO=1`, since v0.10.0) it asks for 10 Hz, and for 1 Hz again once the calibration ends | v0.2.0 |
| `ace2k_lane_counters_reset lane=%c` | `ace2k_lane_counters_reset_response lane=%c accepted=%c` — `lane`: 0–3, or 255 for all four. `accepted` 1: the lane's encoder count is zeroed and its tach count re-based on the next tick. The re-base happens at once if a move starts before that tick. Since v0.6.0 it also happens before any feed mode takes its odometer base, so an assist or a load armed in the same tick counts from 0. `accepted` 0: nothing changed, because the lane (for 255, any lane) is moving or busy in a feed mode. The counters are the move's odometers, and the host first refuses `ACE_COUNTERS_RESET` on its own view of the lanes. This is a query: a reset whose response was lost is sent again by the transport and may land twice. That gives two zeroes, nothing else | v0.2.0; the response since v0.3.0 |
| `ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c` | `ace2k_feed_start_response lane=%c accepted=%c reason=%c` — `mode`: 0 feed, 1 rollback, 2 assist, 3 assist_back (the reverse assist), 4 unload, 5 load, 6 assist_both (the follow, since v0.8.0: [The follow](#the-follow), below). `length_um`: for a feed or a rollback, its length, 1 .. `ACE2K_LANE_MOVE_MAX_UM`, as **the filament's travel on the encoder** (under a spool the motor turns a few per cent more: `hardware.md`, "Counters"); for an unload, its budget (0: 2 000 mm); for a load, its parking distance (0: the configured one); ignored by the assists. `speed_um_s`: within `ACE2K_LANE_SPEED_MIN_UM_S` .. `ACE2K_LANE_SPEED_MAX_UM_S`; the assists need one, and a load's 0 means the configured one. `seq`: the host's sequence for this start, 1–255 ("The start's sequence", below). `reason`: 0 ok; 1 `busy` (the lane is in a mode, or the lane core is moving; the lane core is the layer under the feed that runs each motor move); 2 `in_error`; 3 `no_filament` (no filament at the insert sensor, for a feed, an unload, an assist or a load; a rollback is accepted without one, because it brings back out a piece parked past the sensor); 4 `no_link`; 5 `bounds` (the length, the speed or the mode); 6 `other_lane` (never returned by a start: it belongs to the read with motion, "The read with motion", below; since v0.4.0). A lane above 3 or a mode above 6 is a host bug: `shutdown`. With the reader's field in the image, a load whose tag is not read by the parking point searches further and comes back before it ends `loaded` ("The load's search", below) | v0.3.0; mode 6 since v0.8.0 |
| `ace2k_feed_stop lane=%c` | none; lane 255 = every lane. The motor stops whatever the mode. A moving or assisting lane goes idle with `stopped`. A load still in its settle is cancelled, with nothing ever moved. A lane in error stays in error (its motor stopped when the error latched): a stop never clears an error | v0.3.0 |
| `ace2k_feed_clear lane=%c` | none. Moves the lane from `error` to `idle`; a no-op in any other mode. A clear ends no mode, so the retry window ("The start's sequence") stays the one of the failure | v0.3.0 |
| `ace2k_feed_speed_set lane=%c speed_um_s=%u` | none. A new speed for the lane's current mode. A running move takes it, with its deadline computed again from now. An assist's next burst and a settling load's pull take it too. Silent on an idle lane or a lane in error. A speed out of bounds is a host bug (`shutdown`) | v0.3.0 |
| `ace2k_feed_thresholds_set slip_check_um=%u slip_allow_um=%u stall_check_um=%u` | none. The supervisors' thresholds ("Feed events", below), in effect from the next tick. The firmware's own values are 50 000 / 15 000 / 20 000 µm. This is a **config** command: the host sends it only when `printer.cfg` carries a threshold key, so a change restarts the MCU, as the insert bands do. These are host bugs (`shutdown`), and the host refuses all of them before it sends: a zero; `slip_allow_um ≥ slip_check_um` (the partial trigger could never trip); a `stall_check_um` or `slip_check_um` at or above `ACE2K_FEED_ASSIST_BURST_UM`. The last limit exists because every forward assist burst takes the comparator's bases afresh, and chains into the next burst while the filament is taut. A window that long would never close, and a filament held during an assist would be ground on burst after burst | v0.3.0 |
| `ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c back_um=%u rest_ms=%u free_um=%u` | none. The tip snag's values ("Feed events", below). `fwd_um`: how far the motor runs past a full reading before the touch. `lag_um`: how far the filament may fall behind the motor during the watch before it is a jam. `duty_pct`: how far the duty may rise above the feed-forward duty of the loop's setpoint before it is a jam (0 = this guard off). `back_um`: the length of the touch. `rest_ms`: the wait for the plunger to come to rest. `free_um`: how far the filament must have come back in a reverse mode before a taut trip is forgiven. The firmware's own values are 20 000 / 5 000 µm, 0 %, 5 000 µm, 500 ms, 50 000 µm (dictionary constants `ACE2K_FEED_SNAG_*`, read by the host). The bounds: `fwd_um` 5 000–50 000; `lag_um` 1 000–20 000 and under `fwd_um`; `duty_pct` 0–100; `back_um` 2 000–20 000; `rest_ms` 100–2 000; `free_um` 20 000–200 000. This is an **init** command. The host sends all six at every connect: the `snag_*` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_load_set`. So a value set live never outlives the session. The host sends it again on every `ACE_SNAG_SET`. Any value out of its bounds, or `lag_um ≥ fwd_um`, is a host bug (`shutdown`). The host rounds every value to the unit's resolution (µm, %, ms), and refuses all of these, and a non-finite value, before it sends. Taken at any time, never `busy`. Each value is read where it is used, so a change during a running tolerance mixes two sets, every value of both within its bounds. RAM only: an MCU restart returns to the firmware's values until the host's next connect sends its own | v0.7.0 |
| `ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u` | none. The follow's values ("The follow", below). `flip_ms`: after a move of the follow ends, the wait before a move the other way may start without the plunger reading rest first (rule 1). `take_um`: one take-up of the follow (the reverse assist keeps its own 15 mm). `tail_um`: the tail's bound, as motor travel since the tail began ("The tail", since v0.11.0). The firmware's own values are 200 ms, 15 000 µm and 2 000 000 µm, each marked as an initial value for the bench (dictionary constants `ACE2K_FEED_FOLLOW_FLIP_MS`, `ACE2K_FEED_FOLLOW_TAKE_UM` and `ACE2K_FEED_FOLLOW_TAIL_UM`, read by the host). The bounds: `flip_ms` 50–2 000, `take_um` 5 000–30 000, `tail_um` 200 000–2 000 000. They are compiled beside the defaults, and are in the dictionary too (`…_MIN`, `…_MAX`). This is an **init** command. On an image with the follow, the host sends all three at every connect: the `follow_*` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_snag_set`. It sends it again on every `ACE_FOLLOW_SET`. A value out of its bounds is a host bug (`shutdown`). The host checks the keys against the dictionary's bounds at connect (a config error), and `ACE_FOLLOW_SET`'s arguments before it sends (a G-code error), each rounded to the unit's resolution (ms, µm) first. Taken at any time, never `busy`. Each value is read where it is used: the next take-up, the next flip, the next tick of a tail. RAM only: an MCU restart returns to the firmware's values until the host's next connect sends its own | v0.8.0; `tail_um` since v0.11.0 |
| `ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u` | none. The feed-forward's values ("The feed-forward", below). `chunk_um`: one dose, and half the cap on the debt. `pulse_um_s`: a dose's speed. The firmware's own values are 3 000 µm and 15 000 µm/s, each marked as an initial value for the bench (dictionary constants `ACE2K_FEED_FF_CHUNK_UM` and `ACE2K_FEED_FF_PULSE_UM_S`, read by the host). The bounds: `chunk_um` 1 000–10 000; `pulse_um_s` the lane's speed bounds (9 000–70 000). They are compiled beside the defaults, and are in the dictionary too (`…_MIN`, `…_MAX`). This is an **init** command. On an image with the feed-forward, the host sends both at every connect: the `ff_chunk_mm` and `ff_pulse` keys of `printer.cfg` over the firmware's values, whether the keys are there or not, as for `ace2k_feed_follow_set`. It sends it again on every `ACE_FF_SET` that changes one. A value out of its bounds is a host bug (`shutdown`). The host checks the keys against the dictionary's bounds at connect (a config error), and `ACE_FF_SET`'s arguments before it sends (a G-code error), each rounded to the µm first. Taken at any time, never `busy`. Each value is read where it is used: the next dose, the next accrual's cap. RAM only | v0.9.0 |
| `ace2k_feed_base lane=%c rate_um_s=%u clear=%c` | none. The lane's base rate for the feed-forward ("The feed-forward", below): the filament the head is about to use, in µm/s, from 0 up to `ACE2K_LANE_SPEED_MAX_UM_S`. 0 stops the accrual. What is owed is still delivered, in whole doses (a residual under one dose stays owed). `clear=1` also drops what is owed, with the sub-micrometre remainder; a dose already running still ends at its chunk. The host sends `clear=1` in three cases: for a lane remapped to another extruder; for each lane of an extruder with two or more lanes in follow; and, since v0.10.0, for the lane an automatic encoder calibration holds, at the hold ("The feed-forward"). Every other send is `clear=0`. Only a lane in mode 8 `following` takes it; any other mode ignores it silently. A base sent as the follow ends is a race, not a bug. Arming the follow starts at 0, and every end of the mode returns it to 0. A lane above 3, a rate above the lane's top speed, or a `clear` other than 0 or 1 is a host bug (`shutdown`). The host clamps the rate to `feed.speed_max` before it sends. RAM only | v0.9.0 |
| `ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c` | none. The load's parking distance (the firmware's own value is 300 000 µm: the measured path from the drive gear to the outlet, minus a margin; `hardware.md`, "Motors"), its speed (30 000 µm/s), and whether a filament put in at the entrance of an idle lane is pulled in on its own (1). An init command at connect and on every `ACE_LOAD_SET`. `park_um` must be 1 .. `ACE2K_LANE_MOVE_MAX_UM` and the speed within the bounds, else it is a host bug (`shutdown`) | v0.3.0 |
| `ace2k_feed_lane_grip lane=%c grip_um=%u` | none. Sets the lane's **grip** ("The grip", below): while it is set, the lane's next automatic load pulls only `grip_um`, with no tag search, and ends `loaded`. `grip_um` 0 clears it; otherwise 10 000 .. 45 000 µm (`ACE2K_FEED_GRIP_UM_MIN` .. `ACE2K_FEED_GRIP_UM_MAX`). The top bound stops 5 mm short of a load's first 50 mm, which the comparator does not judge, so a grip never meets the comparator. The host's default is 40 000 µm (`ACE2K_FEED_GRIP_UM`). The host reads all three dictionary constants. A lane above 3 or a grip out of its bounds is a host bug (`shutdown`). The host sends it with `grip_um=0` for every lane as an **init** command at every connect, so a grip never outlives the session. Taken at any time. RAM only | v0.11.0 |
| `ace2k_feed_query rest_ticks=%u` | periodic `ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s` — one frame with the four lanes packed. `mode`: one byte per lane (0 idle, 1 feeding, 2 rolling_back, 3 unloading, 4 loading, 5 assisting, 6 assisting_back, 7 error, 8 following — the follow, since v0.8.0). `error`: one byte per lane, the kind that put the lane in error; 0 outside an error. `speed_um_s`: `uint32 × 4` little-endian, the speed the last loop measured on the tach (0 when idle). `duty_pct`: one byte per lane, the duty in effect. `bursts`: `uint16 × 4` little-endian, an assist's bursts or take-ups since it was armed; in the follow both are counted together. A feed-forward dose is not a burst: `ace2k_feed_ff_state` counts it. The count is kept until the lane's next mode starts. `seq`: one byte per lane, the sequence of the last start the unit accepted from the host on that lane (0 until one), kept through idle and error. A host that reconnects seeds its counter from it ("The start's sequence"). 40 bytes of payload, about 52 framed. That is over half of the transmit buffer, so on a busy tick the report may be held for the next one ("Reports"). The host asks for 1 Hz (`feed_report_hz`, 0.05–10 Hz, snapped to the 100 ms grid) | v0.3.0; mode 8 since v0.8.0 |
| `ace2k_feed_ff_query rest_ticks=%u` | periodic `ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s` — the feed-forward's counters, one frame with the four lanes packed. `doses`: `uint16 × 4` little-endian, the doses started. `taut`: `uint16 × 4`, the taut episodes whose burst started (the filament short of the head). `full`: `uint16 × 4`, this lane's full episodes whose take-up started (too much filament). Each correction is counted once per episode, when its burst or take-up actually starts, not while a taut reading waits under another lane's full. `epoch`: one byte per lane, stepped (wrapping) every time the lane enters a mode, from a host start or the unit's own (such as the automatic load). That is when the three counters reset, so the host restarts its tally exactly when the epoch moves. Like `bursts`, the counters are kept after the mode ends, until the next entry. 28 bytes of payload. Its own phase is 40 ms ("Reports"). On an image with the feed-forward, the host asks for it at the state report's rate (`feed_report_hz`), as an init command. During an automatic encoder calibration (since v0.10.0; its marks are the lane's `taut` steps) it asks for 10 Hz, and for the state report's rate again once the calibration ends | v0.9.0 |
| — | `ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c` — sent at the end of every mode; it is also the one notice a running mode sends: [Feed events](#feed-events), below | v0.3.0 |
| `ace2k_lane_scale_set lane=%c um_per_count_x10=%u` | none. The lane's encoder scale in tenths of a micrometre per count (default 12342). Accepted within ±20 % of the default (9874 .. 14810), at any time. The unit keeps the count and converts all of it with the scale in force, so a change moves `encoder_um`, from the next report on, by the count times the change. It is a **config** command per `laneN_encoder_scale` key of `printer.cfg` (`ACE_CALIBRATE_ENCODER` stages one for `SAVE_CONFIG`). Since v0.10.0 it is also an **init** command for every lane at every connect, with the key's scale or the default, so a scale set live never outlives the session. `ACE_CALIBRATE_ENCODER … AUTO=1` also sends it live (`README.md`, "Calibrating the encoder"), and the host keeps its `encoder_mm` continuous across that change ("Host interface"). A connect that changes a lane's scale makes that lane's `encoder_um` jump once; this happens after a calibration that was not saved, which a Klipper restart undoes. A value outside the window, or a lane out of range, is a host bug (`shutdown`) | v0.3.0 |
| `ace2k_mains_query rest_ticks=%u` | periodic `ace2k_mains_state hz10=%hu present=%c rejects=%u` — `hz10`: the mains frequency in tenths of a hertz, from the zero-cross edges of the last 1 s window (two edges per cycle). `present`: 1 while an edge arrived within the last 100 ms. `rejects`: since boot, the edges the interrupt refused because they came less than 7 ms after the previous accepted one; such an edge is a slow edge crossing the input's threshold twice (v0.5.0). **The measurement.** The frequency comes from the accepted edges' own times: an even number of half-periods over the time they took, within a run of edges. It is published once a second. The published value is the longest measurement, when it holds at least 90 half-periods. But if any run over at least 10 half-periods reads off-band, the publication is off-band: it is the run furthest from the bands. **Flash writes.** A flash write holds interrupts off, and all its edges but one are lost. The write records whether the zero-cross input's pending bit held an edge at its end. If it did, the next edge is that late edge: it ends the run, and the next run starts after it. So a write never shortens a measurement. **Gaps.** Every gap between edges is checked: between two accepted edges however short their run, and around a write (from the run's last edge to the write's start, and from its end to the next run's first edge). Each gap is checked together with the gap before it. The two span an interval between edges of the same polarity, which is at most a full period on intact mains. The check is on the full period, not the half, because the zero-cross detector's two half-cycles differ when its threshold sits off the mains' zero. The lockout admits up to ±3 ms at 50 Hz; this is unmeasured at 127 V and at 230 V. A half-period bound would read clean 50 Hz mains as missing edges. After a write, the gap is paired with the lockout instead: the write hid how many edges it held, and a real half-cycle is at least one lockout long. A pair over 125 % of the period missed an edge, and `hz10` reads 1. The period is the last plausible `hz10`'s, or 57 Hz's when there is none; 57 Hz lies between 49 Hz's period and 61 Hz's plus a lockout, so either band is judged right. An edge the lockout refused is a bounce, not an edge of the mains, and ends no gap. Runs of one publication that disagree by more than 1 Hz also read 1 (a short run that a dip shortened can read in the other band). **Stale readings.** `hz10` keeps its last value while no run is long enough, for at most 3 s. Then a measurement of at least 10 half-periods is published, or 2 when there is none. 1 and 2 are not frequencies: both are off-band, and 0 still means none measured yet. The host shows them by name (`mains_note` "missed edge" / "stale", `mains_hz` None). **Windows with the mains absent.** A window in which the mains read absent is counted as a whole (its edges over its length). Every such window holds moments with no edge: before the boot's first edge, through an outage, or as little as up to 10 ms when the mains came back just after the window opened. So its count can read a few edges short (one edge at 60 Hz reads 59.5 Hz). Any counted window that reads plausible, whichever of its ticks read absent, publishes 0 (none measured). The heater takes no period from it, and the next window, which is measured, is the first plausible one (v0.6.0; v0.5.0 published such a count, up to 1 Hz off). The one reject the late edge can cause is not counted. An outage that begins inside a write reads absent up to the write's length (tens of ms) later (v0.5.0). The host asks for 1 Hz | v0.2.0; rejects v0.5.0 |
| `ace2k_fan_set on=%c seconds=%hu` | `ace2k_airflow_response op=%c accepted=%c reason=%c` (`op` 0) — `on` 1: both fans on for `seconds` (1–600), then off. `on` 0: the manual request is released at once and `seconds` is ignored; the fans go off only if nothing else keeps them on. `reason`: 0 ok; 1 `busy` (the dryer holds the fans, or an off while the heater holds a lease); 2 `range` (`seconds` 0 or above 600 with `on` 1). The dryer does not take the fans over from a manual run: its start is refused `busy` while a manual run holds them (`ACE_FAN ON=0` first). A fan never switches off while an outlet NTC reads above 45 °C, whoever asks ("The dryer", below): an accepted off leaves them on until that rule releases them | v0.5.0 |
| `ace2k_flap_pulse flap=%c open=%c` | the same response (`op` 1) — one pulse of the flap's opening coil (`open` 1) or closing coil, for the compiled length (200 ms, `ACE2K_FLAP_PULSE_MS`, measured on the unit; never above 400 ms); then both coils are released. `flap`: 0 bottom, 1 rear. Every command pulses: there is no cache. A pulse asked while any flap's pulse runs or waits (the other flap's, or the dryer's boot close) answers 1 `busy`. Nothing is queued, so nothing moves later (v0.6.0; v0.5.0 queued it behind the running pulse and answered accepted). The published position is where the last pulse that ran to its end left the flap. It reads `unknown` while a pulse on that flap runs or waits. It is never read back (`hardware.md`, "Exhaust flaps"). `reason`: 1 `busy` (the dryer holds the outputs, or another pulse runs or waits); 2 `range` (`flap` above 1) | v0.5.0; `busy` for a pulse running v0.6.0 |
| `ace2k_airflow_query rest_ticks=%u` | periodic `ace2k_airflow_state fans_cmd=%c fans_read=%c owner=%c flaps=%c` — `fans_cmd`: 1 while the fans are commanded on. `fans_read`: bit 0 left, bit 1 right, set when the fan's pin reads high. This is the only read-back there is; there is no tachometer. `owner`: 0 `none`, 1 `manual`, 3 `dryer` (2 is not used). `flaps`: two bits per flap, bottom in bits 0–1, rear in bits 2–3: 0 unknown (no pulse has run to its end since boot, a pulse on the flap is running or waiting, or a shutdown cut a pulse), 1 open, 2 closed. The host asks for 1 Hz; phase 10 ms | v0.5.0 |
| `ace2k_dryer_start target_c=%c minutes=%hu` | `ace2k_dryer_response op=%c accepted=%c reason=%c` (`op` 0) — starts a cycle at `target_c` (15–65 °C) for `minutes` (1–1 440). `reason` 0 is ok; any other value is a refusal ("The dryer", refusals), and nothing starts. The bounds are the constants `ACE2K_DRYER_TARGET_MIN_C`, `ACE2K_DRYER_TARGET_MAX_C` and `ACE2K_DRYER_MINUTES_MAX` | v0.5.0 |
| `ace2k_dryer_stop` | the same response (`op` 1) — ends a cycle in `starting` or `heating`: the heat stops at once, and the fans cool the heater down (`cooldown`). With no cycle running (`idle`, `cooldown`) nothing changes and the answer is 0 ok: a stop is idempotent. In `fault` it answers 3 `faulted` | v0.5.0 |
| `ace2k_dryer_clear` | the same response (`op` 2) — moves the dryer from `fault` to `idle`, once the fault's cool-down has ended and its cause is gone. Otherwise it answers 7 `not_cool`, 4 `sensors`, 5 `no_mains` or 3 `faulted`. It never clears after a tripped cutout (6 `cutout`, in any state: a power cycle clears it). Outside `fault` nothing changes and the answer is 0 ok | v0.5.0 |
| `ace2k_dryer_query rest_ticks=%u` | periodic `ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c remaining_min=%hu fans=%c flaps=%c fault=%c notices=%c lost=%hu oldest=%c` — `state` and `fault`: see "The dryer"; `fault` is the fault that put the dryer in `fault`, 0 otherwise. `target_c`: the target of the current or last cycle (after a lowering to 45 °C, the lowered one); 0 before the first cycle since boot. `drive_dc`: the outlet target the inner loop drives to, in tenths of a degree: the target + 7 °C, trimmed by the chamber loop by at most ±2 °C, never above the target + 8 °C nor 73 °C. `duty`: the heater's duty in percent (0–90). `remaining_min`: the time left in the cycle. `drive_dc`, `duty` and `remaining_min` are 0 outside `starting` and `heating`. `fans`: bit 0 left, bit 1 right, set when the pin reads high, as in `fans_read`. `flaps`: as in `ace2k_airflow_state`. `notices`: bits ("The dryer"). `lost`: the events dropped on a full ring since boot (v0.6.0). `oldest`: the `seq` of the oldest event not yet acknowledged, or the next `seq` to be assigned when none is held; the host's base after a connect (v0.6.0). Every query opens the events, and sends again every event not yet acknowledged, oldest first: the unit holds them until the first query arrives ("The dryer", events). The host asks for 1 Hz, as an init command; phase 90 ms | v0.5.0; `lost`, `oldest` v0.6.0 |
| — | `ace2k_dryer_event kind=%c arg=%c seq=%c` — see [The dryer](#the-dryer), event kinds. `arg`: the fault for `fault`, the lane (1–4) for `lowered`, 0 otherwise. `seq`: the event's 8-bit sequence, one per event, wrapping; the same event sent again carries the same `seq` | v0.5.0; `seq` v0.6.0 |
| `ace2k_dryer_event_ack seq=%c` | none. Tells the unit that the host has every event up to `seq` (in 8-bit order: the events at most 127 behind it). Those events leave the ring. An event not yet acknowledged is sent again after every `ace2k_dryer_query` and every 5 s (`ACE2K_DRYER_EVENT_RESEND_MS`). So the host handles the events in sequence, and acknowledges the last one it handled in order. It never acknowledges one past a gap, which would drop the missing one. An event after a gap waits for the resend; an event already handled is acknowledged again and ignored. The unit ignores the whole acknowledgement when its `seq` is after the newest event ever sent (a resend does not lower that mark), or when no unacknowledged event was ever sent: it cannot come from a host that saw those events | v0.6.0 |
| `ace2k_dryer_log_query index=%c` | `ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u` — the persistent log ("The dryer", the log). `index` 0 is the counters; 1–8 are the entries, newest first. An `index` above 8 is a host bug (`shutdown`) | v0.5.0 |
| `ace2k_rfid_reader_query reader=%c` | `ace2k_rfid_reader_state reader=%c version=%c` — a soft reset of that reader, then a read of its version register: `0x18` on this unit's parts; 0 when the reset did not complete within 150 ms; `0x00` / `0xFF` for a mute bus. `reader`: 0 = A (lanes 3–4), 1 = B (lanes 1–2). The probe runs in task context and may take 150 ms | v0.2.0 |
| `ace2k_rfid_query rest_ticks=%u` | periodic `ace2k_rfid_state state=%*s field=%c` — `state`: one byte per lane, the lane's tag state ("Tag states", below). `field`: bits 0x01 / 0x02 the field on at reader A / B; 0x04 / 0x08 a field fault at A / B (the field read back on after it was switched off; latched until the MCU restarts); 0x10 / 0x20 reader A / B dead (three failed configurations in a row, [Tag states](#tag-states)). The UIDs are not in the report, because four 10-byte UIDs do not fit one frame; `ace2k_rfid_lane_query` gives one. The host asks for 1 Hz; phase 60 ms | v0.4.0 |
| `ace2k_rfid_read lane=%c move=%c` | `ace2k_rfid_read_response lane=%c accepted=%c reason=%c` — a read on command. `move` 0: the next inventory of the lane's reader decides, without motion ("The read on command"). `move` 1: the read with motion, the load's search from where the filament is, and back ("The read with motion"). `reason`: 0 ok; 1 `busy` (a session open on the lane, or a read on command pending on it; for `move=1` also the lane in a feed mode, moving or in error); 2 `no_filament`; 3 `no_link`; 4 `other_lane_moving` (`move=1`: another lane of the unit busy); 5 `unsupported` (`move=1` in an image without the feed, or with no search configured). A lane above 3 or a `move` above 1 is a host bug: `shutdown` | v0.4.0 |
| `ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s` | none; the unit answers with the step's `ace2k_rfid_data` frames. One step of the lane's open session ("RFID sessions"). `op` 0: read NTAG pages; `arg` is the first page (0–244), `blocks` the number of groups of four pages (1–3), `key` is ignored. `op` 1 / 2: authenticate MIFARE Classic sector `arg` (0–15) with key A / B (`key` is six bytes), then read the data blocks whose bits are set in `blocks` (bits 0–2, never the trailer). Host bugs (`shutdown`): an `op` above 2, a MIFARE step with a key of another length, a lane, sector, page or block mask out of bounds. A step for a session that is not open, or sent while the session's previous step waits or runs, comes from a stale host and is ignored. Each accepted step restarts the session's 500 ms idle timer | v0.4.0 |
| `ace2k_rfid_done lane=%c session=%c result=%c` | none. The host ends the session. `result` 1: read (the lane becomes `read`). `result` 0: give up (back to `pending`, or to `searching` inside its load). A session that is not open is ignored. A lane above 3 or a `result` above 1 is a host bug (`shutdown`) | v0.4.0 |
| `ace2k_rfid_forget lane=%c` | none. Returns a `read` or `no_tag` lane to `pending`, and forgets its UID: use it when the tag on the spool was swapped. The next move of the lane reads it again. A no-op in any other state. A lane above 3 is a host bug (`shutdown`) | v0.4.0 |
| `ace2k_rfid_lane_query lane=%c` | `ace2k_rfid_lane_state lane=%c state=%c uid=%*s` — the lane's tag state and, while the lane is `reading` or `read`, the attributed UID (4, 7 or 10 bytes; empty otherwise). A UID is per-unit data: it belongs in a log, never in a document | v0.4.0 |
| `ace2k_rfid_search_set search_um=%u` | none. The reach of the load's search, as filament travelled since the load began ("The load's search"). The unit starts from `ACE2K_FEED_SEARCH_DEFAULT_UM`. This is a **config** command: the host sends it only when `printer.cfg` carries `rfid_search_mm`, so a change restarts the MCU, which starts from the default again. 0, or a value above `ACE2K_FEED_SEARCH_MAX_UM`, is a host bug (`shutdown`). Only in an image with the feed | v0.4.0 |
| — | `ace2k_rfid_tag lane=%c session=%c uid=%*s atqa=%hu sak=%c` — a session opened on the lane: its number (1–255, never 0), the tag's UID, ATQA and SAK ("RFID sessions") | v0.4.0 |
| — | `ace2k_rfid_data lane=%c session=%c block=%c status=%c data=%*s` — one frame per 16 bytes a step read (`status` 0, `block` the MIFARE block or the first of four NTAG pages, `data` 16 bytes), and one frame for a step that failed (`status` 1 `auth_failed`, 2 `tag_gone`, 3 `error`; `block` the step's first block or page; `data` empty) ("RFID sessions") | v0.4.0 |
| — | `ace2k_rfid_event lane=%c kind=%c session=%c` — one of three things: the lane's new tag state (`kind` 0–5; the opening of a session is announced by its `ace2k_rfid_tag`, not here); the unchanged state, as the answer to a read on command that no inventory decided; or 6 `ambiguous` ("Tag states"). `session` is 0 in this release | v0.4.0 |
| `ace2k_health_query` | `ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c` — `now` and `latched`: the two fault masks ("Health bits", below). `post_ms`: the tick time in milliseconds of the last full pass (0: none yet; the first runs 2 s after boot). `reset_cause`: the cause of the last reset | v0.2.0 |
| `ace2k_health_run` | none. Takes a new snapshot of the counters, and runs the full pass again 2 s later: every bit, the reader probes included | v0.2.0 |
| `ace2k_health_clear` | none. Sets `latched` to `now` | v0.2.0 |
| `ace2k_uid_query` | `ace2k_uid_response uid=%*s` — the MCU's 96-bit device identifier, 12 bytes in the order they lie in the part. The host prints them as 24 hex digits. Per-unit data: it belongs in a log, never in a document | v0.2.0 |

The dictionary's constants also list the pins the firmware owns:

- `RESERVE_PINS_serial` and `RESERVE_PINS_ace2k_led`, since v0.1.0;
- `RESERVE_PINS_ace2k_adc`, `_sensors`, `_env`, `_lane`, `_mains` and `_rfid`, since v0.2.0;
- `RESERVE_PINS_ace2k_motor` and `_remap`, since v0.3.0;
- in an image with the dryer's outputs, since v0.5.0: `RESERVE_PINS_ace2k_airflow` (PE12, PE8,
  PD3, PD4, PD5, PD6) and `RESERVE_PINS_ace2k_heat` (PC8, and PC9 in an image without the lane).

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

The bounds of a move are constants too, so the host can check them without its own copy
(`hardware.md`, "Motors", says where each comes from):

- `ACE2K_LANE_SPEED_MIN_UM_S` 9000, `ACE2K_LANE_SPEED_MAX_UM_S` 70000 and `ACE2K_LANE_MOVE_MAX_UM`
  2000000;
- `ACE2K_LANE_DUTY_MIN_PCT` 20 (provisional: the loaded floor is at most 20 %, and nothing below
  it has been measured);
- `ACE2K_LANE_LANDING_UM` 10000: the last 10 mm of a move at the floor speed, which a waited G-code
  budgets as the firmware's deadline does.

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
  `ace2k_feed_ff_state` carries no `epoch`, has no feed-forward.

An image with the reader's field and the feed adds the reach of the load's search:

- `ACE2K_FEED_SEARCH_MAX_UM` 1000000, the ceiling `ace2k_rfid_search_set` accepts;
- `ACE2K_FEED_SEARCH_DEFAULT_UM` 750000, the reach the unit starts with. That is one revolution of
  the largest spool that fits (at most π × 200 mm ≈ 628 mm of filament), plus the longest tag
  window measured (102–116 mm on an Anycubic tag; `hardware.md`, "RFID readers"). It is the host's
  value for `rfid_search_mm` when `printer.cfg` does not set it. A waited `ACE_LOAD`'s budget and a
  `MOVE=1` read's wait are computed from it.

PC9, the cutout's latch-reset pin, is never driven or configured by any image. Every image with the
lane or the heater reserves it exactly once, so no host section can claim it: in
`RESERVE_PINS_ace2k_lane` (with PC8, the triac gate, in an image without the dryer's outputs), or
in `RESERVE_PINS_ace2k_heat` in an image without the lane.

## Reports

Every report binding keeps one deadline (`firmware/src/ace2k/core/report.h`). The query sets the
period. Each module has its own phase into the period: sensors 0 ms, airflow 10 ms, env 30 ms, the
feed-forward's counters 40 ms (since v0.9.0), lane 50 ms, rfid 60 ms, mains 70 ms, feed 80 ms,
dryer 90 ms. The 10 ms tick raises the report when the deadline passes, and a task sends it.

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
report, and the two are tried in turn on that tick. The rfid's report then lands on 70 with the
mains report, and the two are tried in turn on that tick. Because the due report goes before the
events, a state report can show a lane idle before that lane's terminal event has left. The event
leaves on the first tick its frame fits: normally the same tick as the report, one tick later under
buffer pressure. A host must not read "idle with no event yet" as a lost event.

**Phases.** The phases keep the periodic reports on distinct ticks. They hold only for periods that
are **multiples of 100 ms**: the host snaps a `feed_report_hz` that is off that grid onto it. The
deadlines are anchored to the absolute 100 ms grid, not to the tick a query arrived on. **Every
report is one frame per module, never one per lane.** `ace2k_sensors_thresholds`,
`ace2k_lane_counters`, `ace2k_rfid_state`, `ace2k_feed_state` and `ace2k_feed_ff_state` pack the
four lanes, so a report is one try, sent whole or held whole. After a stall (the link proof's flash
write), a deadline moves whole periods ahead: the reports the stall swallowed are skipped, not
replayed, and the grid keeps its phase. The reports survive a Klipper shutdown (the tick timer is
re-armed). The host sends every report query as an init command, so an MCU reset restarts them.

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
| 2 | `chamber` | no valid chamber reading in the last 2 s (none yet counts) | boot, every second |
| 3 | `chamber_plausible` | the last chamber reading is outside −40…85 °C or above 100 % RH, or none has arrived yet | boot, every second |
| 4, 5 | `reader_a`, `reader_b` | the version register is not `0x18` after a soft reset (0: the reset did not complete; `0x00` / `0xFF`: a mute bus). Since v0.4.0 also a field fault (the field read back on after it was switched off) or a dead reader ("Tag states") | probed at boot and on run, one reader per task run, so a full pass takes three task runs; also probed by `ace2k_rfid_reader_query`. The bit follows the last probe, and follows the field fault and the dead flag at every periodic pass. A pass leaves a reader alone while its field is on or an exchange runs on it (a soft reset would cut a read), and keeps that reader's last verdict. A run clears a dead reader first, so the watch tries it again |
| 6 | `zerocross` | no zero-cross edge in the last 100 ms | boot, every second |
| 7 | `mains_hz` | the frequency last published (`ace2k_mains_state` `hz10`, once a second) is not 49.0–51.0 or 59.0–61.0 Hz, the markers 1 and 2 included | boot, every second |
| 8 | `cutout` | the cutout input reads high (debounced); it idles low | boot, every second |
| 9–12 | `insert1`–`insert4` | the lane's raw insert reading is ≤ 20 mV or ≥ 3250 mV (a disconnected or shorted lever board; initial values), or no ADC pass exists yet | boot, every second |
| 13–16 | `buffer1`–`buffer4` | *rest* and *pushed* asserted together. Enabled in the release build: over 134 debounced events on the four lanes, measured on the unit, they never were | boot, every second |
| 17–20 | `encoder1`–`encoder4` | the board's raw count moved by more than 2 during the 2 s window | boot, on run |
| 21–24 | `fg1`–`fg4` | any tach pulse during the 2 s window | boot, on run |
| 25 | `sensors_fresh` | the last ADC pass is more than 50 ms old, or none exists. A pass waits 50 ms after a tick catch-up (a flash stall), so a stall does not read as staleness | boot, every second |
| 26 | `vdda` | the supply, from the internal 1.2 V reference, is outside 3135–3465 mV (3.3 V ± 5 %) | boot, every second |
| 27 | `clock` | the core is not on the PLL, or the PLL is not fed from the crystal, or the crystal is not ready | boot, on run |
| 28 | `watchdog` | the independent watchdog does not read back as running with its expected configuration: the low-speed oscillator on, no prescaler / reload update left in flight, and prescaler and reload at their reset values /4 and 0xFFF rather than the bootloader's longer timeout. The bootloader arms it before it jumps to the application, so the bit attests a running watchdog, not who armed it (`hardware.md`) | boot, on run |
| 29 | `config_page` | the ace2k config page is corrupt (magic, layout or CRC wrong); blank is fine | boot, on run |
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
  filament did not follow.
- `seq` is the sequence of the start that entered the mode, or 0 for a motion the unit started
  itself ("The start's sequence", below).

**The supervisors** behind the error kinds run every tick in every mode whose motor is commanded.
That includes an assist's bursts, but not a load's first 50 mm: the hand and the pinch catching the
filament there are not a slip.

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
  the tip snag's tolerance (below) forgives it. The plunger's ≈ 18 mm of travel at the spring's
  force is the early warning, before anything grinds. (This was decided on the unit: a feed into a
  held tube ran to done with the motor pushing at full force.) In a load's tag search it is the
  search's obstacle return instead. Up to v0.10.0 it was `stuck`.
- In every running mode, the insert sensor falling ends the mode (`runout`, or `unloaded` in an
  unload). The follow is the exception: there the sensor falling begins the tail instead (since
  v0.11.0, "The tail", below). In the tail, every comparator trip is its end, `tail_out`, never an
  error kind.

The lane core's own outcomes come through as `motor_stalled` and `timeout`, in every mode.

**Blocked** (since v0.11.0). A push toward the head that meets something that cannot move stops
without an error and waits. A push toward the head is a feed, or a load's pull to its parking point
(the automatic load's included). These end it as `blocked` (18):

- a forward comparator trip, the standstill or the partial, whatever the buffer reads;
- this lane's full reading, when the tip snag's tolerance does not forgive it: the lag, the duty,
  the plunger not back at rest, a second full, or a touch that the lane refuses, that stalls or
  that times out.

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

*Forward* (a feed, a load's pull or its tag search), this lane's full reading no longer ends the
move at once. It starts a *watch*: the move runs on, and every tick judges it from the motor and
encoder counts taken at that full reading.

- The filament more than `lag_um` (5 mm) behind the motor is `stuck`. In a feed or a load's pull it
  is `blocked` instead, as is every verdict of this list that says `stuck` ("Blocked", above).
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
  encoder. For a load, that is its pull, or its search for what is left of its reach.
- A second full in the same mode is `stuck` at once.

*Toward the spool* (a rollback, an unload), the same touch runs the other way. It applies to a
comparator trip with the plunger taut and the insert sensor present, after the filament came back
at least `free_um` (50 mm) in the mode. The touch is a forward move of `back_um`. Then come rest
within `rest_ms`, the notice, and the reverse move resumed for what is left (for an unload: its
budget, with the insert sensor still its goal). A trip before that free travel stays `stuck` at
once: a tip held in the head is held from the start, and comes back at most the plunger's slack.

With under 2 mm left to resume, a feed or a rollback ends `done`, and an unload ends
`unload_incomplete` (its budget spent). The tail at rest, a load's return and reacquire, and the
assist are unchanged.

**In a load's tag search**, every verdict that is `stuck` above is the search's obstacle instead:
the return to the parking point and `loaded`, no error ("The load's search"; added after the unit
showed it). That covers the lag, the duty, the plunger not back at rest, a second full (the load is
one mode: a tolerance spent in the pull leaves none for the search), and a touch that the lane
refuses, that stalls or that times out. It also covers the search's own move stalled or past its
deadline during the watch (`motor_stalled` and `timeout` elsewhere). A tag read while the tolerance
runs ends the search when the plunger is back at rest (on its own in the watch, or after the
touch), with the return and `found`, instead of searching on. It is `found` too if the tolerance
then ends on a verdict. A start given to another lane ends the tolerance at once, whatever its
phase, with the return and `yield`.

**One tolerance per mode**: a second catch in the same mode, either way, is `stuck`. While a watch,
a touch or the wait for rest runs, the comparator is not asked: the lag is the sharper test, and
the touch moves nothing the comparator should judge. The comparator restarts its bases at the
resume. The touch is one bounded lane move, which STOP, the link, a shutdown and a runout end as
they end any move. A touch that stalls or times out, or a resume that the lane refuses, is `stuck`.
In a feed it is `blocked`, and so is a touch's stall or timeout in a load's pull. In the tag search
it is the obstacle return. A load's resume that the lane refuses ends the load `loaded`, as for any
load move. A `done` after a tolerance carries the whole mode's odometers, not the last move's. The
values can be set at run time (`ace2k_feed_snag_set`, above).

| Kind | Name | When | The lane after |
|---|---|---|---|
| 0 | `done` | a feed or a rollback travelled its length on the encoder | idle |
| 1 | `stopped` | `ace2k_feed_stop` in a running mode; a load still in its settle is cancelled with nothing moved | idle |
| 2 | `stopped_link` | 3 s without a host byte. This applies alike to a running move, an assist waiting between bursts and a load in its settle. The event goes out to no one | idle |
| 3 | `stopped_shutdown` | a Klipper shutdown. This applies alike to a running move, an assist waiting between bursts and a load in its settle (a lane in error stays in error, with no event). Delivered during the shutdown. From then on nothing starts until the MCU restarts: no burst, no load, no automatic load, and every start is refused `no_link`. The host's bytes still arrive after a shutdown, so the link alone would not stop them | idle |
| 4 | `runout` | the insert sensor fell during any running mode except an unload and the follow (in a load: the filament withdrawn by hand; in the follow it starts the tail, 16). Also during an error, which it ends (below) | idle |
| 5 | `loaded` | a load's pull reached the parking distance. With the reader's field in the image, the event waits until the search and the return are over too; for the read with motion, until the filament is back where it started ("The load's search"). A gripped automatic load ends `loaded` once its pull reached `grip_um` ("The grip") | idle |
| 6 | `unloaded` | the insert sensor fell during an unload | idle |
| 7 | `stuck` | the filament does not pass ahead, in a mode other than a feed or a load's pull. (Those end `blocked`, 18, since v0.11.0. Up to v0.10.0 they also ended `stuck`: the lane's own buffer full with the filament lagging or the duty over its guard, or a second time in the mode, as in the tip snag above; and at once until v0.7.0.) The cases: a touch whose plunger does not come back to rest, or a second catch in the mode; a comparator trip forward with the buffer full (the follow, the forward assist, a load's forward return); any comparator trip toward the spool, except the standstill at rest (13) and a tip snag's first forgiven one; any comparator trip in a load's return or reacquire, the standstill at rest included (a filament that will not come back is held, not a tail past the drive gear; since v0.4.0). Example: a tip held in a hot end while a rollback grinds, the filament taut (measured on the unit: ≈ 20 mm of motor after the encoder stopped) | error |
| 8 | `tangled` | a comparator trip forward with the buffer not full, in the same modes as `stuck`'s forward trip (in a feed or a load's pull it is `blocked` since v0.11.0; in the tag search, its obstacle return). The filament does not come from the spool: a tangle, the end of the filament tied to the spool, a spool that will not turn (measured on the unit: 14.6 mm of slack, then ≈ 20 mm of motor with the encoder still) | error |
| 9 | `motor_stalled` | the tach silent for 1 s with the motor commanded (not provoked on the bench: that would mean holding a motor) | error |
| 10 | `timeout` | the lane's deadline (3/2 of the expected time plus 1 s) passed with the filament short of the length: no filament in the lane, or a slip past the comparator's allowance. A reverse move's deadline with the plunger at rest is the tail instead (13) | error |
| 11 | `assist_stall` | **reserved — superseded by `tangled`, never sent.** A forward assist's filament that is taut and still under command is the comparator's standstill (20 mm of motor after the filament stopped), and ends `tangled` in `assisting` (measured on the unit: the spool held while the filament was pulled taut). A timed rule for the same case came second at any burst speed above about 10 mm/s and was removed. The number stays so nothing shifts | — |
| 12 | `assist_overrun` | **reserved, not produced.** A burst that reaches its bound while the filament is still taut chains into the next burst: the head is pulling, which is no fault (decided at the bench). The number stays so nothing shifts | — |
| 13 | `unload_incomplete` | a reverse move (an unload, a rollback) ended with the filament still at the sensor and nothing left to drive it. Three cases: the tail is past the drive gear (a standstill, or the deadline, with the plunger at rest: nothing pulls on the buffer); the move went on for 100 mm of motor for the spool to wind the tail out, and it did not; or the unload's budget was spent first. A hand is needed (measured on the unit: a lane whose spool does not wind the tail out) | error |
| 14 | `behind` | a notice; the lane goes on. A forward burst reached its 100 mm bound with the filament still taut: the head uses filament faster than a burst delivers it. Sent once per taut episode. It is not sent for the taut reading itself, which is a burst's normal trigger (decided at the bench: fourteen notices in a minute of ordinary pulling) | unchanged |
| 15 | `snag` | a notice; the lane goes on. A filament tip caught the tube's entry, and the tip snag's tolerance (above) let the move go on: the plunger came back to rest on its own in the watch, or after the touch. Carries the mode's odometers where the tip caught (the watch's start forward, the forgiven trip in reverse). Once per mode at most | unchanged |
| 16 | `tail` | a notice; the lane goes on. The insert sensor fell in `following`: the filament ran out, or was cut, at the bay, and its tail still lies between the insert sensor and the drive gear. The follow feeds on as the tail ("The tail", below). Once per tail: the insert sensor rising and falling again during it changes nothing (since v0.11.0) | unchanged (`following`) |
| 17 | `tail_out` | the tail ended, with no error. Either a comparator trip in the tail (the standstill or the partial, whatever the buffer reads: the tail has left the drive gear and the filament no longer follows the motor), or `tail_um` of motor since the tail began (since v0.11.0). No motor attempt follows it: a filament put in at the bay during the tail starts no load (the insert sensor is already set, so there is no edge) | idle |
| 18 | `blocked` | a push toward the head met something that cannot move; no error. In `feeding` or a load's pull: a forward comparator trip (the standstill or the partial), or this lane's full reading that the tip snag's tolerance does not forgive ("Blocked", above). The filament waits where it stopped; a load's filament waits at the point it reached (since v0.11.0) | idle |

**Kinds 7–13 put the lane in `error`.** The motor stops, the kind goes into the state report's
`error` byte, and every start on that lane is refused `in_error`. Two things end the error:

- `ace2k_feed_clear`;
- the filament leaving the insert sensor. The error was about the filament, and without a filament
  there is none. The lane goes idle with a `runout` that names the mode that failed and carries the
  failed start's sequence. (Decided at the bench: a filament withdrawn by the lever during a load's
  pull tripped `tangled` a moment before the sensor cleared, and the lane sat in error with nothing
  in it.)

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

`runout`, `unloaded`, `unload_incomplete`, `behind`, `snag`, `tail`, `tail_out` and `blocked` have
no counterpart. The original protocol has no clear command, and its STOP leaves an assist's motor
running. Here both do what they say.

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
  snag's watch.
- A forward comparator trip on this lane, in a mode that still judges it, is judged `stuck` instead
  of `tangled`: the comparator chooses the kind from the same reading. (A feed or a load's pull ends
  `blocked` either way.)

A lane at rest or taut is never affected. With one filament moving at a time, as on the U1, the
case does not arise.

## The follow

Since v0.8.0. The follow is the third assist. `ace2k_feed_start` mode 6 (`assist_both`; the G-code
`ACE_ASSIST … DIR=BOTH`) arms it, and the lane is then in mode 8 `following`. It keeps the buffer
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
  tube, pulled by the head alone;
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
(200–2 000 mm, since v0.11.0), and from `ACE_FOLLOW_SET FLIP_MS= TAKE_MM= TAIL_MM=` (`README.md`).
Their defaults are initial values for the bench.

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

**The debt.** Each tick of a lane in the follow with a base rate adds the rate × 10 ms to what the
lane owes the head. The sub-micrometre remainder is carried, so a slow rate is not lost to the
tick. The debt is capped at two doses (`2 × chunk_um`), so a wrong rate never builds up into a long
feed.

**The dose.** A dose starts when the debt reaches `chunk_um` (3 mm) and all of these hold: no move
is running, the plunger is not taut, no lane reads full (this lane or another), and rule 1 allows
a forward move ("The follow"). The lane then starts a forward move of `chunk_um` at `pulse_um_s`
(15 mm/s). A dose starts at rest too, where the follow itself starts nothing: it is metered ahead
of the pull. It ends at its chunk, on the shared full (the safety stop, as for a burst) or on taut;
never on rest. When it ends, the filament's travel on the encoder pays the debt, floored at 0. The
comparator judges it as a burst (a forward move). It raises no `behind` notice, and `bursts` does
not count it.

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

    ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale (ACE_CALIBRATE_ENCODER)

(or `full corrections`). A steady stream of them in one direction means the scale is off. Run
`ACE_CALIBRATE_ENCODER`; its automatic mode (since v0.10.0) reads the scale off the lane's extruder
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
  A tick that the reactor ran late sweeps the time it missed, in steps of one period.
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
- **Calibration.** An automatic encoder calibration (since v0.10.0) holds its lane for the run. The
  lane gets no rate and is not metering, so its corrections count toward no notice. What it owes is
  dropped at the hold (`clear=1`). The `ACE_FEED_FORWARD` switch is untouched. The release sends
  nothing itself: the next tick works out the rate as usual.
- After a Klipper or unit shutdown the host sends nothing.

**An older image**, without the feed-forward's constants or whose `ace2k_feed_ff_state` has no
`epoch`, has no feed-forward. Nothing is sent to it, and `ff_chunk_mm` and `ff_pulse` are left
unused. `ACE_FF_SET` and `ACE_FEED_FORWARD` answer `ace2k: the unit's firmware has no feed-forward;
flash v0.9.0`. With `feed_forward` on, the same line is printed once on the console at ready.

**The run-time values.** The unit's, `chunk_um` and `pulse_um_s`, are `ace2k_feed_ff_set`'s
(above), from the `[ace2k]` keys `ff_chunk_mm` and `ff_pulse`. The host's are `ff_period_ms`,
`ff_window_ms`, `ff_lead_ms`, `ff_deadband` and `ff_notice_every`. `ACE_FF_SET CHUNK_MM= PULSE=
WINDOW_MS= LEAD_MS= DEADBAND=` changes them live (a new lead drops each extruder's sweep and
credit, so the next tick starts afresh). `ACE_FEED_FORWARD [LANE=] ON=1|OFF=1` switches the
feed-forward (`README.md`). The defaults are initial values for the bench.

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
| 5 | `no_tag` | the load's search ran its whole reach and no UID was attributed to the lane during this insertion |

How a lane moves between states:

- A filament at the insert sensor, inserted or present at boot, makes an `unknown` lane `pending`.
- The sensor falling returns any lane to `unknown`. Its UID is forgotten, and its session, if one
  is open, ends.
- A load starting on a `pending` or `no_tag` lane makes it `searching`. The load's end makes it
  `pending` again, or `no_tag` when the search ran its whole reach and no UID was ever attributed
  to the lane during this insertion. **A lane that saw a tag is never `no_tag`.**
- `ace2k_rfid_forget` and the read with motion return a `read` or `no_tag` lane to `pending`.
- A `no_tag` lane is not watched. A later load, a read on command or a forget looks again.

Every change is an `ace2k_rfid_event` with the new state as its `kind`, except the opening of a
session, whose notice is its `ace2k_rfid_tag`. `kind` 6, `ambiguous`, is never a state. It is the
answer of a read on command that saw a UID it could not attribute, and it changes nothing.

**The field.** A lane is *watched* while it is `pending` or `searching` and its motor runs or its
load runs. A reader's field is on while one of its two lanes is watched, a session is open on it, a
read on command waits on it, or a lane's attributed tag waits for the reader's session. Otherwise
it is switched off and **read back off** (both antenna drivers disabled; `hardware.md`, "RFID
readers"). A field that reads back on latches the reader's field fault (the state report's `field`
bits 0x04 / 0x08, the health bit 4 or 5) until the MCU restarts. A field on for 120 s, with no lane
of its reader moving and no session, is switched off. It stays off until a lane of the reader moves
or a read on command asks. Before it switches a field on, the unit configures the reader: a soft
reset, and the settings the reset clears. It does so again after any soft reset since (the health
probe's). A configuration that fails is not tried again before 1 s. Three failures in a row make
the reader **dead**: no field, every read on command answered at once, the state report's `field`
bit 0x10 / 0x20 and the health bit set. This lasts until `ace2k_health_run` (`ACE_HEALTH RUN=1`) or
a restart gives it another try.

**Watching and attribution.** A reader with its field on takes an *inventory* every 50 ms: every
tag in the field, at most four, by ISO 14443-A anticollision, each tag halted once found. At this
period every passage of a tag at 30 mm/s was seen, on Anycubic and Bambu Lab spools. The first
inventory runs 10 ms after the field comes on (the tags' power-up), and it is the baseline: the
tags that were there at rest. Two lanes share each antenna (`hardware.md`, "RFID readers"). Take a
lane *m* whose neighbour on the same reader is *n*. A UID is given to *m* only when *m*'s situation
explains it:

1. a UID attributed to *n* (`read`, or `reading`) is never *m*'s;
2. *m*'s own UID, once known during this insertion, is *m*'s;
3. with no filament at *n*'s insert sensor, any UID in the field is *m*'s;
4. a UID that **enters** the field — absent from the previous inventory — while *m* moves and *n*
   does not is *m*'s;
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
| 3 | `error` | a malformed answer, a CRC error, a NAK, a reader that did not answer | open |

A step's job belongs to the lane and the session it was sent for. A job whose session ended while
it ran sends nothing, and its result reaches no later session. The unit keeps a step's key only
until the step's job begins, and never sends a key.

**The end.** The host ends the session with `ace2k_rfid_done`: `read` makes the lane `read`; `give
up` returns it to `pending` (`searching` in its load). The unit ends the session itself, with the
lane `pending` (`searching` in its load), in three cases: 500 ms with no step from the host
(counted from the opening, the last step or the last step's result), `tag_gone`, and a lost link.
The filament leaving the insert sensor ends it with the lane `unknown`. A host that is not there
reads no tag: its sessions end within half a second, and the lane waits for its next move.

**The host's side** (`klippy/extras/ace2k_rfid.py`). It offers the chip to the brand modules that
may carry it, by SAK and UID length: an NTAG (seven bytes, SAK 0) to the NTAG brands; a MIFARE
Classic 1K (SAK 0x08) to the MIFARE brands in `rfid_mifare_order`. It sends each candidate's steps
and hands the bytes back. It ends the session `read` as soon as a module decodes a record, or, once
every candidate has declined, with the UID alone.

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
   - the lane's own buffer full, when the tip snag's tolerance does not forgive it: the filament
     met something past the outlet. The search ends; this is **not** the load's `blocked`, which it
     only is before the parking point. (Since v0.7.0 a full reading starts the watch, the touch and
     the wait for rest, and the search goes on for what is left of its reach:
     [Feed events](#feed-events));
   - the reach: `rfid_search_mm` of filament travelled since the load began (then `no_tag`, if no
     UID was attributed);
   - another lane is given a start, or another lane's automatic load begins (the search yields, the
     lane `pending`).
3. **The return** to the parking point, measured on the encoder, at the load's speed. The load ends
   `loaded`.

A comparator trip in the return or the reacquire is `stuck`, whatever the buffer reads. A STOP at
any phase stops where the filament is (`stopped`), with the lane `pending` if its tag was not read.
An error, a runout, a lost link and a shutdown end the load where they find it, as in any mode.

**The read on command** (`ace2k_rfid_read` with `move=0`). It is refused without a filament
(`no_filament`), without a link (`no_link`), and while a session is open on the lane or a read on
command already waits on it (`busy`). Once accepted, it turns the lane's field on, and the next
inventory decides:

- one UID in the field that is not the neighbour's, when the neighbour has no filament or its tag
  is `read`, opens a session (whatever the lane's state);
- no UID answers the lane's unchanged state;
- anything else answers `ambiguous` and changes nothing.

The answer is an `ace2k_rfid_event` (a session's outcome, or the unchanged state) at the latest
**2000 ms** after the command. A read that no inventory decided by then is answered with the lane's
unchanged state. A read on a dead reader is answered at once, the same way.

**The read with motion** (`ace2k_rfid_read` with `move=1`, in an image with the feed) runs phases
2 and 3 from where the filament is: forward up to `rfid_search_mm`, then back to where it started.
It is refused `busy` while a session is open on the lane or a read on command waits on it. This is
checked before anything else, so a refusal changes nothing. Then come the feed's refusals:
`no_link`, `busy` (the lane in a mode, moving or in error), `no_filament`, `other_lane_moving`
(another lane of the unit busy), `unsupported` (no search configured). Once accepted, it is an
**explicit re-read**. A `read` or `no_tag` lane forgets its tag and becomes `pending` (an
`ace2k_rfid_event`), then `searching` as the unit's own load (`seq` 0, mode `loading`) starts at the
search.

The host's `ACE_RFID_READ MOVE=1` waits for the feed's terminal event of that load: the
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
  of the grip, never past it. This lane's buffer full that the tip snag's tolerance does not forgive
  ends it `blocked`, with nothing after. A STOP, the link, a shutdown, a runout and the supervisors
  end it as they end any mode. A grip never meets the comparator: its top bound, 45 mm, stops short
  of a load's first 50 mm, which the comparator does not judge.
- A **host start** on the lane that passes the start's refusals clears the grip: a feed, a
  rollback, an unload, a load (`ACE_LOAD` pulls to the parking point, never the grip), an assist,
  the follow. So does an accepted **read with motion**. A refused start leaves the grip, as does
  another lane's start. `grip_um=0` clears it.
- RAM only. An MCU restart drops every grip, and the host clears every lane's grip as an init
  command at every connect. So a host restart that leaves the MCU running (`RESTART`,
  `SAVE_CONFIG`) does not keep a grip either.

The grip changes nothing else. The tag of a gripped spool is not searched for. It is read only if
it passes the antenna within the grip, by a read without motion (`ace2k_rfid_read` with `move=0`,
above), or on the lane's next move.

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
   run holding the fans refuses the start `busy`: `ACE_FAN ON=0` first.) Once both fans read high
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
MCU restarts. **A reset never resumes a cycle**: the unit boots `idle` and logs the cycle
`interrupted`.

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
| 4 | `chamber_over` | the chamber above its ceiling for 30 s, or above 80 °C. The ceiling is the cycle's highest target or the chamber's lowest reading since the start, whichever is higher, + 10 °C. It only ever moves down within a cycle (v0.6.0; v0.5.0 counted from the target alone, so a target 10 °C below the room latched it with nothing heating) |
| 5 | `chamber_stale` | the chamber reading implausible, or none newer than 5 s |
| 6 | `ntc` | an outlet NTC invalid for 5 samples 100 ms apart while no lease is held (a duty of 0, a renewal waiting); or the gate latched on an invalid NTC (its leaky bucket, above) or on one at 85 °C |
| 7 | `mains` | the mains absent, at once; implausible once the gate's bucket fills (1.5 s off-band, above); or the gate latched on either |
| 8 | `cutout` | the thermal cutout input asserted — in any state, at boot too. **Cleared only by a power-on reset** ("The cutout", below) |
| 9 | `heat` | the gate latched for a reason of its own — `gate_stuck` or `duty` (the gate's limits, below) — or refused a lease with no other reason |
| 10 | `fans` | both fans not reading high 3 s into `starting`, or the gate latched on the fans |
| 11 | `flaps` | the flaps' closing pulses not both over 3 s into `starting` (their sequence gives up after 2 s; they take at most 0.8 s) |

The rise-rate ceiling of fault 2, and the thermal simulator the host tests run whole cycles on, are
fitted to four heater runs recorded on the unit. The simulator's chamber loss to the room is
assumed, not fitted, so the chamber's accuracy and the outlets' overshoot are settled on the unit.
Every threshold here is an initial value that the bench confirms or re-tunes.

A refusal leaves everything as it was. The start checks, in this order, the state, the range, then
the inputs:

| Refusal | Name | Meaning |
|---|---|---|
| 1 | `busy` | start: the dryer not `idle` (a cycle or a cool-down running), or the fans held by a manual run (`ACE_FAN ON=0` first) |
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
| 1 | `stopped` | 0 — `ace2k_dryer_stop` ended a cycle |
| 2 | `fault` | the fault |
| 3 | `lowered` | the lane (1–4): while `starting` or `heating`, with the host link down and the target above 45 °C, a filament reached that lane's insert sensor. The target is now 45 °C (the chamber ceiling keeps the cycle's highest). A spool put on its holder without a filament is not seen |
| 4 | `interrupted` | 0 — sent after a boot that found a cycle open: a reset or a power loss ended it, and it was not resumed |
| 5 | `hot_ambient` | 0 — the cool-down had not ended after 10 min: the flaps closed, the fans left to the 45 °C rule, the dryer `idle`; no fault |
| 6 | `vented` | 0 — the flaps pulsed open for the cycle's vent. Sent on the first opening only; the humidity guard's moves are silent (v0.6.0) |
| 7 | `cleared` | 0 — `ace2k_dryer_clear` accepted |

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
- 0x4 `log_unstored`: the log's last store failed on the flash. The unit tries it again after a
  back-off of 1 s, doubling at every failure up to 60 s, and the bit clears with the first store
  that succeeds.
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
every image with the lane or the heater (in the lane's list, or in the heater's in an image without
the lane), so the host cannot claim it either.

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
  retransmits. So about once in twenty restarts it is lost to such a collision, and `klippy`
  reports `Failed automated reset of MCU`. Repeat the `FIRMWARE_RESTART`.
- The unit holds its replies until a host burst has ended, so a burst's replies must fit Klipper's
  96-byte transmit buffer. The configuration burst at connect (46 bytes of replies) fits; six
  queries with 12-byte answers lose the sixth.

Shorter or host-polled report traffic is the lever for the residual, and is left to a later change.
`ace2k_link_query` counts the waits.

## The link proof

A valid image that boots but is never spoken to would otherwise be unrecoverable over the wire: the
bootloader validates it and jumps to it on every power cycle. So an image keeps, on its config
page, which version last proved the link. A different version (a fresh flash) starts a 180 s
window, shown on the lane LEDs as a one-lane chase. The first answered `ace2k_version` stores the
proof. The LEDs then leave the chase for the intro sweep (every lane empty) or the lanes' own
states (`hardware.md`, "Lane LEDs").

An image whose window runs out resets into recovery, where the updater works. The reset is asked
through the same vetoes as `ace2k_bootloader_enter`: the heater active, the fans required (a dryer
cycle, or an outlet NTC still at or above 42 °C), a lane moving. Each veto is bounded: a cool-down,
a move's own deadline. While one objects, the reset is deferred and asked again every 10 ms, and
checked once more with interrupts masked right before it. The unit keeps answering meanwhile
(`ace2k_linkproof_state` reports `proven=0 remaining_ms=0`). The reset follows on the first tick
no veto objects. A gate stuck high defers it until a power cycle. A proven image never resets
itself for lack of a host.

Consequence for hosts: **the `[ace2k]` section is part of the link, not an option.** A `klippy`
configured with `[mcu ace2k]` alone never sends `ace2k_version`, and a freshly flashed unit will
drop into recovery three minutes after boot. The module queries at MCU identify time, before the
connect handlers (which stop at the first config error). It retries the proof once when the state
comes back unproven. A `proven=0` after that means the store failed on the unit, and the module
logs a warning. On a bench without Klipper, `klippy/console.py` lets you type `ace2k_version`.

The module's G-codes:

- always: `ACE_STATUS`, `ACE_RESTORE_STOCK`, `ACE_HEALTH`, `ACE_COUNTERS_RESET` and
  `ACE_CALIBRATION_SAVE`;
- with the feed: `ACE_FEED`, `ACE_ROLLBACK`, `ACE_UNLOAD`, `ACE_LOAD`, `ACE_ASSIST`, `ACE_STOP`,
  `ACE_CLEAR`, `ACE_SPEED`, `ACE_CALIBRATE_ENCODER`, `ACE_LOAD_SET`, `ACE_SNAG_SET` and
  `ACE_FOLLOW_SET`;
- with the reader's field: `ACE_RFID_READ` and `ACE_RFID_FORGET`;
- with the dryer's outputs: `ACE_FAN` and `ACE_FLAP`;
- with the dryer: `ACE_DRY`, `ACE_DRY_STOP`, `ACE_DRY_CLEAR` and `ACE_DRYER_LOG`.

`README.md` lists each with its arguments. Klipper's G-code parser reads a command name as letters
and underscores only, so a digit cannot appear in one. That is why the G-codes are `ACE_*` while
the section stays `[ace2k]`.

## Host interface

Another Klipper host module can drive the lanes through the feed object of `[ace2k]`, not through
the G-codes: a G-code cannot be run from a timer or an async callback, and these calls can. The
module is `klippy/extras/ace2k_feed.py`; the object is `printer.lookup_object("ace2k").feed`.

`API_VERSION` names this interface. It is now `6`:

- `1` until v0.8.0, which added `assist_both`;
- `2` until v0.9.0, which added `set_lane_extruder`;
- `3` until v0.11.0, which added the tail (the kinds `tail` and `tail_out`, the lane's `tail` in
  the status);
- then `4` during that release's work, until the kind `blocked`, and the lane's `tail` no longer
  ending on the insert rising, made it `5`;
- `6` with `set_lane_grip` and the grip's bounds.

It changes with any change a caller must check for, a call or a mode added as much as one changed,
so a caller checks it once at connect. It is a module constant and an attribute of the feed object:
a caller that holds only the object reads `printer.lookup_object("ace2k").feed.API_VERSION`.

- `feed.start_move(lane_index, mode, length_mm, speed_mm_s)` → a `Move`. Starts `mode` on
  `lane_index` (0–3) and returns once the unit answered; it does not wait for the move to end.
  `mode` is one of `feed`, `rollback`, `assist`, `assist_back`, `assist_both` (the follow:
  `start_move(lane, "assist_both", 0.0, speed)`), `unload`, `load`. Lengths and speeds are those
  the G-codes take, in mm and mm/s: the speed within the dictionary's bounds
  (`feed.speed_min`..`feed.speed_max`), and `length_mm` at most `feed.move_max_mm`. The length is 0
  for an assist; 0 for an unload takes the unit's default budget. A speed or length out of bounds
  is the unit's refusal (`bounds`); the host does not check them here. The start carries the lane's
  next sequence, as a G-code's does ("The start's sequence").
  - Raises `FeedRefused` when the unit refuses: `.reason` is the refusal's name (`busy`,
    `in_error`, `no_filament`, `no_link`, `bounds`, `other_lane`), `.lane` and `.mode` the start's.
    Nothing is left registered: a move already running on the lane (refused `busy`) stays the
    lane's tracked move, and completes on its own event.
  - Raises `ValueError` for a lane or mode outside the lists above (nothing sent). Raises
    `RuntimeError` when the firmware was built without the feed, when no state report came within
    the wait a G-code allows before its first start, or when the mode is `assist_both` on an image
    without the follow (`the unit's firmware has no follow tail (or no follow); flash v0.11.0`,
    nothing sent). The feed object's `has_follow` says, from connect on, whether the image has it.
    Since v0.11.0 that means the follow with its tail. An image older than v0.11.0, whose
    dictionary lacks the tail's constants, has no follow to the host: `ACE_FOLLOW_SET`,
    `ACE_ASSIST … DIR=BOTH` and an automatic encoder calibration's arm are refused with that line,
    and a `follow_*` key is a config error. The plain feeds and the two assists work as before.
- `Move`: `.lane`, `.mode`, `.seq`; `.done()` (true once the final event landed); `.result()` (that
  event, or `None` before; never blocks); `.wait(timeout_s)` (blocks the calling greenlet until the
  event or `timeout_s`; returns the event or `None`). A move completes on the first event of its
  lane that echoes its sequence and is not a notice: `done`, `stopped`, an error kind and the rest
  of [Feed events](#feed-events). `behind`, `snag` and `tail` do not complete it. A follow's
  `tail_out` does. `blocked` completes a feed or a load. It is final, not an error: a waited
  `ACE_FEED` or `ACE_LOAD` answers `ace2k: lane n feed blocked — motor … mm, filament … mm`
  (`load blocked` for a load), not a G-code error. One move per lane is tracked. A later start on
  the same lane, through this interface or a waiting G-code, takes the lane's slot, and the earlier
  `Move` then never completes. A timeout stops nothing; `stop` is the caller's.
- `feed.stop(lane_index)` sends `ace2k_feed_stop` (`LANE_ALL` = 255, every lane); it never clears
  an error. `feed.clear(lane_index)` sends `ace2k_feed_clear`, leaving the lane's error state. Both
  raise `ValueError` for a lane outside 0–3 (`LANE_ALL` is accepted by `stop` only), nothing sent.
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
- Every event, final or notice, is also the Klipper event `"ace2k:feed_event"` with
  `(lane_index, event)`. `event` is a dict with `kind`, `mode` (the names of
  [Feed events](#feed-events)), `motor_mm`, `filament_mm` and `seq`: the same dict `Move.result()`
  returns. A final event completes its `Move` first, and is sent as the Klipper event after. So a
  listener sees the move already done, and a listener that raises cannot keep it from completing.
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
`stop`, `clear` and `set_lane_grip` are plain sends and never yield.
