"""ace2k — the Anycubic ACE 2 Pro as a Klipper MCU: host module.

At MCU identify time it queries ``ace2k_version`` — the firmware's proof that the host link
works (``docs/protocol.md``) — and the link-proof state, retrying the proof once when the unit
reports it unproven, then the unit's device identifier (``ace2k_uid_query``).  At connect it
sends the optional insert thresholds as config commands (so a changed or removed override
restarts the MCU, which then re-reads its factory page) and starts the periodic reports
(``ace2k_sensors_state`` at 10 Hz; ``ace2k_env_state``, ``ace2k_lane_counters`` and
``ace2k_mains_state`` at 1 Hz) as init commands, then asks for the bands in use (one frame for
the four lanes; the answer is a report frame with no timeout of its own, so it is asked once
more 1.5 s later if it has not arrived, and a warning follows 1.5 s after that); the reports
land in ``get_status()``, the switch events in the log.  Once Klipper is ready it polls the
self-test (``ace2k_health_query``) every 10 s, the first time 5 s in — after the firmware's own
2 s boot window; a poll the unit does not answer is logged and retried, five in a row stop the
poll, and the health reads unknown (``ok`` None, ``stale`` True) until the unit answers again.
The poll, the re-read after ``RUN=1`` and ``ACE_HEALTH`` take turns on one reactor mutex:
Klipper keys a query's response handler by its name, so two in flight would take each other's
answer.
A subsystem the firmware was built without (its query command is missing from the dictionary)
is logged and left out: its init command, subscriptions and polls are skipped and its status
fields stay None.
G-codes:
``ACE_STATUS``, ``ACE_RESTORE_STOCK`` (sends ``ace2k_bootloader_enter``; the unit resets into
the bootloader's recovery, see ``docs/flashing.md``), ``ACE_CALIBRATION_SAVE`` (stages the
insert thresholds in use for ``SAVE_CONFIG``), ``ACE_COUNTERS_RESET`` (zeroes the encoder and
tach counters of one lane or all four; refused, here and by the unit, while a lane moves or
is busy — an older firmware takes it without confirming),
``ACE_HEALTH`` (the self-test's two masks by name, or
``not run yet`` before the firmware's first full pass, ``unknown`` when the unit does not
answer; ``RUN=1`` re-runs it and re-reads the result 2.5 s later — and again, up to three
times a second apart, while the unit still reports the pass read before the run — read by
``RUN=1`` itself when no poll has landed yet; ``CLEAR=1`` clears the latched faults);
the feed G-codes: ``ace2k_feed.py``; the tag G-codes: ``ace2k_rfid.py``; the dryer, fan and
flap G-codes: ``ace2k_dryer.py``.
Section::

    [ace2k]
    mcu: ace2k        # the [mcu <name>] section of the unit (default ace2k; "mcu" = the primary)
    #lane1_insert_mv: 600, 1100   # optional, lane1..lane4: the insert band in millivolts,
                                  # low < high; the factory calibration is used when absent

``[ace2k]`` also registers the ``ace2k`` temperature sensor type (``ace2k_env.py``), so it must
come before the ``[temperature_sensor]`` sections that use it.
"""

import logging
import struct
import threading

import msgproto  # klippy's own: the message parser's error class

LANES = 4
KIND_NAMES = {0: "insert", 1: "rest", 2: "pushed", 3: "pulled", 4: "cutout"}
CAL_NAMES = {0: "default", 1: "factory", 2: "config"}
VALID_NAMES = {0x1: "ntc_left", 0x2: "ntc_right", 0x4: "chamber", 0x8: "vdda"}
# ace2k_mains_state hz10 values that are not frequencies (docs/protocol.md): shown by name
MAINS_MARKERS = {1: "missed edge", 2: "stale"}
SENSORS_REPORT_S = 0.1
ENV_REPORT_S = 1.0
LANE_REPORT_S = 1.0
# A live scale change settles in a report period (LANE_REPORT_S) and this margin: a lane-counters
# frame received that long after the change went out is at the new scale, whatever it reads
# (_follow_scale_change).  The real bound is the link's latency, not a period: the unit packs each
# frame when it sends it, so a frame at the old scale left the unit before the change reached it,
# and arrives within one round trip of the link after the change went out.  The settle time is a
# generous ceiling on that.
SCALE_CHANGE_MARGIN_S = 0.2
MAINS_REPORT_S = 1.0
LANE_ALL = 255  # the firmware's "every lane" index of ace2k_lane_counters_reset
RESET_CMD = "ace2k_lane_counters_reset lane=%c"
FORMAT_MISMATCH = "format mismatch"  # in the parser's text for a name carried with another format
FORMAT_HINT = (
    "the unit's firmware and this module are from different commits; update one to match the other"
)
# the unit's answer since the feed was added; a v0.2.0 dictionary has the command without it
RESET_RESP = "ace2k_lane_counters_reset_response lane=%c accepted=%c"
VALID_CHAMBER = 0x4  # the env report's chamber bit: the humidity is a reading only with it set
# One entry per firmware subsystem: the flag that compiles it, its query command (the probe: a
# firmware built without the flag has no such command in its dictionary) and its report period.
# The four report queries share the signature "rest_ticks=%u"; the health query takes no
# argument and is polled from the host (period None: no init command).
SUBSYSTEMS = (
    ("sensors", "CONFIG_ACE2K_SENSORS", "ace2k_sensors_query", SENSORS_REPORT_S),
    ("env", "CONFIG_ACE2K_ENV", "ace2k_env_query", ENV_REPORT_S),
    ("lane", "CONFIG_ACE2K_LANE", "ace2k_lane_counters_query", LANE_REPORT_S),
    ("mains", "CONFIG_ACE2K_MAINS", "ace2k_mains_query", MAINS_REPORT_S),
    ("health", "CONFIG_ACE2K_HEALTH", "ace2k_health_query", None),
)
# The self-test's bits, in the firmware's order (docs/protocol.md "Health bits").
HEALTH_BITS = [
    "ntc_left",
    "ntc_right",
    "chamber",
    "chamber_plausible",
    "reader_a",
    "reader_b",
    "zerocross",
    "mains_hz",
    "cutout",
    "insert1",
    "insert2",
    "insert3",
    "insert4",
    "buffer1",
    "buffer2",
    "buffer3",
    "buffer4",
    "encoder1",
    "encoder2",
    "encoder3",
    "encoder4",
    "fg1",
    "fg2",
    "fg3",
    "fg4",
    "sensors_fresh",
    "vdda",
    "clock",
    "watchdog",
    "config_page",
    "image_crc",
]
RESET_CAUSES = {
    0: "unknown",
    1: "power-on",
    2: "reset pin",
    3: "software",
    4: "watchdog",
    5: "window watchdog",
    6: "low-power",
}
HEALTH_FIRST_POLL_S = 5.0  # after the firmware's 2 s boot window has run its full evaluation
HEALTH_POLL_S = 10.0
HEALTH_POLL_FAILURES_MAX = 5  # consecutive unanswered polls before the poll gives up
HEALTH_RERUN_POLL_S = 2.5  # after RUN=1: the 2 s window, then the three task steps of the pass
# The re-read can land before the new pass (the window is up to 2010 ms, then a 50 ms settle,
# two reader probes of up to 150 ms each and the transit): while post_ms is the one read before
# the run, it is asked again, a bounded number of times.
HEALTH_RERUN_RETRIES = 3
HEALTH_RERUN_RETRY_S = 1.0
THRESHOLDS_REQUERY_S = 1.5  # the thresholds frame is a report, not a reply: one bounded re-ask


def bit_names(mask):
    return [name for i, name in enumerate(HEALTH_BITS) if mask & (1 << i)]


class Ace2k:
    def __init__(self, config):
        self.printer = config.get_printer()
        mcu_name = config.get("mcu", "ace2k")
        # Klipper registers the primary MCU as "mcu" and the others as "mcu <name>"
        self.mcu = self.printer.lookup_object("mcu" if mcu_name == "mcu" else "mcu " + mcu_name)
        self.version = None
        self.link_proven = None
        # klippy:mcu_identify, not klippy:connect: the connect handlers run in registration
        # order and stop at the first error, so a config error elsewhere would leave a fresh
        # unit unproven — and it resets into recovery when its window expires.  The MCU objects
        # register their own mcu_identify handlers first (they load the unit's dictionary), so
        # the queries below are usable when this one runs.
        self.printer.register_event_handler("klippy:mcu_identify", self._on_mcu_identify)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("ACE_STATUS", self.cmd_ACE_STATUS, desc=self.cmd_ACE_STATUS_help)
        gcode.register_command(
            "ACE_RESTORE_STOCK",
            self.cmd_ACE_RESTORE_STOCK,
            desc=self.cmd_ACE_RESTORE_STOCK_help,
        )
        self.section = config.get_name()
        self.thresholds = {}
        for lane in range(LANES):
            key = f"lane{lane + 1}_insert_mv"
            pair = config.getintlist(key, None, count=2)
            if pair is not None:
                low, high = pair
                if not 0 <= low < high <= 3300:
                    raise config.error(
                        f"{self.section}: {key} must be 'low, high' millivolts, low < high"
                    )
                self.thresholds[lane] = (low, high)
        self.lanes = [
            dict(
                insert=None,
                insert_mv=None,
                empty_mv=None,
                rest=None,
                pushed=None,
                low_mv=None,
                high_mv=None,
                source=None,
                encoder_um=None,
                fg=None,
            )
            for _ in range(LANES)
        ]
        self.pulled_any = self.cutout = None
        self.aux_mv = self.age_ms = None
        self.mains_hz = self.mains_present = None
        self.mains_note = self.mains_rejects = None
        self.env = dict(
            ptc_left=None,
            ptc_right=None,
            chamber=None,
            humidity=None,
            vdda=None,
            valid=[],
            valid_mask=0,
        )
        self.env_listeners = []
        self.counter_listeners = []  # cb(receive_time, encoder_um) per lane-counters frame
        # µm added to a lane's reported encoder: encoder_mm continuous across a live scale change
        self.encoder_offset_um = [0] * LANES
        # per lane, a live scale change no frame has shown yet: (old scale, new scale, the
        # reference's count, the time past which any frame is at the new scale)
        self.scale_pending = [None] * LANES
        # the serial thread's frames against the reactor's scale changes, resets and status reads
        self.encoder_lock = threading.Lock()
        # ok None means "no verdict": stale False while the firmware has not run its first full
        # pass, stale True once a query went unanswered (the unit is shut down, busy or gone)
        self.health = dict(
            ok=None, failing=[], latched=[], post_ms=None, reset_cause=None, stale=False
        )
        self.health_poll_failures = 0
        self.health_answers = 0  # answered health queries, for the failure rule below
        self.health_rerun_post_ms = None  # the pass read before the last RUN=1
        self.health_rerun_left = 0  # re-queries left while post_ms has not moved
        self.thresholds_requeried = False
        self.uid = None
        self.present = set()  # the subsystems the firmware carries, known at config time
        self.reset_confirmed = False  # the firmware answers a counters reset (RESET_RESP)
        self.reactor = self.printer.get_reactor()
        # One timer for the re-read after RUN=1, re-armed on every run: a timer registered per
        # run would stay in the reactor's list forever once done.
        self.health_rerun_timer = self.reactor.register_timer(self._requery_health)
        # One health query in flight at a time (_query_health)
        self.health_mutex = self.reactor.mutex()
        self.printer.load_object(config, "ace2k_env")
        self.feed = self.printer.load_object(config, "ace2k_feed")
        # the feed's automatic encoder calibration reads the lane counters and sets their period
        # through this object, which keeps the reported encoder continuous when it sets a scale
        self.feed.unit = self
        self.register_counters_listener(self.feed.cal.on_counters)
        self.rfid = self.printer.load_object(config, "ace2k_rfid")
        self.dryer = self.printer.load_object(config, "ace2k_dryer")
        self.mcu.register_config_callback(self._build_config)
        # The MCU objects are created before the other sections, so the MCU's own connect
        # handler (its config and init commands) runs before this one.
        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        gcode.register_command(
            "ACE_CALIBRATION_SAVE",
            self.cmd_ACE_CALIBRATION_SAVE,
            desc=self.cmd_ACE_CALIBRATION_SAVE_help,
        )
        gcode.register_command(
            "ACE_COUNTERS_RESET",
            self.cmd_ACE_COUNTERS_RESET,
            desc=self.cmd_ACE_COUNTERS_RESET_help,
        )
        gcode.register_command("ACE_HEALTH", self.cmd_ACE_HEALTH, desc=self.cmd_ACE_HEALTH_help)

    def _on_mcu_identify(self):
        """Prove the link (the firmware stores the proof on the first answered query)."""
        version_cmd = self.mcu.lookup_query_command(
            "ace2k_version", "ace2k_version_response version=%s"
        )
        state_cmd = self.mcu.lookup_query_command(
            "ace2k_linkproof_state",
            "ace2k_linkproof_state_response proven=%c remaining_ms=%u",
        )
        self.version = version_cmd.send()["version"].decode()
        state = state_cmd.send()
        if not state["proven"]:
            # The unit answered but did not store the proof (its flash write failed): one more
            # query, then the state as it stands.
            version_cmd.send()
            state = state_cmd.send()
        self.link_proven = bool(state["proven"])
        if not self.link_proven:
            logging.warning(
                "ace2k %s: the link proof is not stored after a retry (remaining_ms=%d); the "
                "store failed on the unit (a flash error) and the unit resets into the "
                "bootloader's recovery when the window expires — see docs/protocol.md",
                self.version,
                state["remaining_ms"],
            )
        logging.info("ace2k %s connected, link proven: %s", self.version, self.link_proven)
        # The device identifier: part of the health subsystem, so a firmware built without it
        # has no such command — then uid stays None.
        if self.mcu.try_lookup_command("ace2k_uid_query") is not None:
            uid_cmd = self.mcu.lookup_query_command("ace2k_uid_query", "ace2k_uid_response uid=%*s")
            self.uid = uid_cmd.send()["uid"].hex()
            logging.info("ace2k unit %s", self.uid)

    def _build_config(self):
        """The thresholds as config commands, the periodic queries as init commands (Klipper
        sends those at every connect, so after an MCU reset too), and the report subscriptions —
        each only for a subsystem the firmware carries."""
        self.present = set()
        self.reset_confirmed = False
        for name, flag, query, report_s in SUBSYSTEMS:
            probe = query if report_s is None else f"{query} rest_ticks=%u"
            if self.mcu.try_lookup_command(probe) is None:
                logging.warning("ace2k: firmware built without %s; %s disabled", flag, name)
                continue
            self.present.add(name)
            if report_s is None:
                continue
            ticks = self.mcu.seconds_to_clock(report_s)
            self.mcu.add_config_cmd(f"{query} rest_ticks={ticks}", is_init=True)
        if "sensors" in self.present:
            for lane, (low, high) in sorted(self.thresholds.items()):
                # A config command, not an init command: it enters Klipper's config checksum,
                # so editing or removing a laneN_insert_mv key and restarting resets the MCU,
                # which then re-reads its factory page — a removal must reach the MCU too.
                self.mcu.add_config_cmd(
                    f"ace2k_sensors_thresholds_set lane={lane} low_mv={low} high_mv={high}"
                )
            self._subscribe(
                self._handle_sensors_state,
                "ace2k_sensors_state switches=%hu insert_mv=%*s empty_mv=%*s aux_mv=%hu age_ms=%c",
            )
            self._subscribe(
                self._handle_sensors_event, "ace2k_sensors_event lane=%c kind=%c level=%c"
            )
            self._subscribe(
                self._handle_thresholds,
                "ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s",
            )
        if "env" in self.present:
            self._subscribe(
                self._handle_env_state,
                "ace2k_env_state ptc_left_mc=%i ptc_right_mc=%i chamber_mc=%i"
                " chamber_rh_pct10=%hu vdda_mv=%hu valid=%c",
            )
        if "lane" in self.present:
            self._subscribe(self._handle_lane_counters, "ace2k_lane_counters encoder_um=%*s fg=%*s")
            self.reset_confirmed = self._reset_confirmation()
        if "mains" in self.present:
            self._subscribe(
                self._handle_mains_state, "ace2k_mains_state hz10=%hu present=%c rejects=%u"
            )

    def _reset_confirmation(self):
        """Whether the firmware answers a counters reset (RESET_RESP in its dictionary, probed
        with probe_format).  Another format: a firmware this module does not know — a config error
        at connect, as a subscription on it would be.  Absent, or any other failure: an older
        firmware; the reset goes out unconfirmed, and the warning quotes the parser's text."""
        verdict, detail = probe_format(self.mcu, RESET_RESP)
        if verdict == "ok":
            return True
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, RESET_RESP, detail)
        logging.warning(
            "ace2k: firmware answers no counters reset; ACE_COUNTERS_RESET is sent unconfirmed"
            " (%s)",
            detail,
        )
        return False

    def _subscribe(self, handler, msgformat):
        """A subscription for a report the dictionary carries as this module knows it (probed
        with probe_format).  Another format: a firmware from another commit, a config error at
        connect.  Absent while the subsystem's query is present: this report is not in that
        firmware — logged, nothing registered, the fields it would fill stay None."""
        verdict, detail = probe_format(self.mcu, msgformat)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, msgformat, detail)
        if verdict == "absent":
            logging.warning(
                "ace2k: firmware carries no %s; its fields stay empty (%s)",
                msgformat.split()[0],
                detail,
            )
            return
        subscribe_response(self.mcu, handler, msgformat)

    def _on_connect(self):
        """Ask for the bands in use once the subscriptions are live.

        Not an init command: Klipper activates a subscription made from the config callback
        only after the init commands have gone out, so the answer to a query among them would
        not be seen.  The answer is one frame for the four lanes — a report, not a query
        response, so a lost frame raises no timeout: a timer asks once more if it has not
        arrived, then warns.
        """
        if "sensors" in self.present:
            self.mcu.lookup_command("ace2k_sensors_thresholds_query").send()
            self.reactor.register_timer(
                self._requery_thresholds, self.reactor.monotonic() + THRESHOLDS_REQUERY_S
            )

    def _thresholds_missing(self):
        return any(lane["low_mv"] is None for lane in self.lanes)

    def _requery_thresholds(self, eventtime):
        # One re-query, then one warning; no further retries.
        if not self._thresholds_missing():
            return self.reactor.NEVER
        if not self.thresholds_requeried:
            self.thresholds_requeried = True
            self.mcu.lookup_command("ace2k_sensors_thresholds_query").send()
            return eventtime + THRESHOLDS_REQUERY_S
        logging.warning(
            "ace2k: the insert thresholds did not arrive after a re-query; the band fields of "
            "ACE_STATUS stay empty and ACE_CALIBRATION_SAVE has nothing to stage"
        )
        return self.reactor.NEVER

    def _on_ready(self):
        """Start the self-test poll once everything is up; nothing without the subsystem."""
        if "health" in self.present:
            self.reactor.register_timer(
                self._poll_health, self.reactor.monotonic() + HEALTH_FIRST_POLL_S
            )

    def _poll_health(self, eventtime):
        # A query the unit does not answer (it is shut down, or gone) raises the printer's
        # command error; a poll that let it through would take the reactor down.  One failure
        # is logged, the health reads unknown (not the last good answer; _query_health marks
        # it) and the poll goes on (a unit busy elsewhere answers the next one); after
        # HEALTH_POLL_FAILURES_MAX in a row the poll stops.  A success resets the streak.
        try:
            self._query_health()
        except self.printer.command_error as e:
            self.health_poll_failures += 1
            if self.health_poll_failures >= HEALTH_POLL_FAILURES_MAX:
                logging.warning(
                    "ace2k: health poll stopped after %d failures: %s",
                    self.health_poll_failures,
                    e,
                )
                return self.reactor.NEVER
            if self.health_poll_failures == 1:
                logging.warning(
                    "ace2k: health poll failed, retrying every %.0f s: %s", HEALTH_POLL_S, e
                )
            return eventtime + HEALTH_POLL_S
        self.health_poll_failures = 0
        return eventtime + HEALTH_POLL_S

    def _requery_health(self, eventtime):
        # The re-read after RUN=1: the result lands in get_status() and the next ACE_HEALTH; a
        # unit that does not answer is logged, not raised into the reactor.  An answer with the
        # post_ms read before the run is the previous pass, not the new one: asked again, up to
        # HEALTH_RERUN_RETRIES times, then a warning.
        try:
            h = self._query_health()
        except self.printer.command_error as e:
            logging.warning("ace2k: health re-query after RUN failed: %s", e)
            return self.reactor.NEVER
        if h["post_ms"] != self.health_rerun_post_ms:
            return self.reactor.NEVER
        if self.health_rerun_left > 0:
            self.health_rerun_left -= 1
            return eventtime + HEALTH_RERUN_RETRY_S
        logging.warning(
            "ace2k: the self-test still reports the previous pass (post_ms=%s) %.1f s after "
            "RUN; the unit did not run a new one",
            h["post_ms"],
            HEALTH_RERUN_POLL_S + HEALTH_RERUN_RETRIES * HEALTH_RERUN_RETRY_S,
        )
        return self.reactor.NEVER

    def _health_unknown(self):
        """No answer from the unit: no verdict.  What the last answer said about the latched
        faults and the reset stays, marked stale."""
        self.health = dict(self.health, ok=None, failing=[], stale=True)

    def _query_health(self):
        """One query in flight at a time: Klipper keys the response handler by the response's
        name, so two overlapping queries (the poll, the re-read after RUN, the G-code) would
        take each other's answer and one would time out.  The callers queue on the mutex.  A
        query that fails raises the printer's command error and marks the health unknown —
        unless an answer landed since it started, which the mutex makes impossible and the
        answer count guards against all the same: a caller queued behind a slow one must never
        wipe a fresh verdict."""
        with self.health_mutex:
            answers_before = self.health_answers
            try:
                params = self.mcu.lookup_query_command(
                    "ace2k_health_query",
                    "ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c",
                ).send()
            except self.printer.command_error:
                if self.health_answers == answers_before:
                    self._health_unknown()
                raise
            self.health_answers += 1
            # post_ms is the tick at which the firmware's last full evaluation ran; 0 means
            # none has yet (the first runs 2 s after boot), so `now` says nothing: ok None,
            # failing [].
            evaluated = params["post_ms"] != 0
            self.health = dict(
                ok=params["now"] == 0 if evaluated else None,
                failing=bit_names(params["now"]) if evaluated else [],
                latched=bit_names(params["latched"]),
                post_ms=params["post_ms"],
                reset_cause=RESET_CAUSES.get(params["reset_cause"], "unknown"),
                stale=False,
            )
            return self.health

    # The handlers run in the serial thread: they store and hand work to the reactor.
    def _handle_sensors_state(self, params):
        sw = params["switches"]
        insert_mv = struct.unpack(f"<{LANES}H", params["insert_mv"])
        empty_mv = struct.unpack(f"<{LANES}H", params["empty_mv"])
        for i, lane in enumerate(self.lanes):
            lane["insert"] = bool(sw & (1 << i))
            lane["rest"] = bool(sw & (1 << (4 + i)))
            lane["pushed"] = bool(sw & (1 << (8 + i)))
            lane["insert_mv"] = insert_mv[i]
            lane["empty_mv"] = empty_mv[i]
        self.pulled_any = bool(sw & (1 << 12))
        self.cutout = bool(sw & (1 << 13))
        self.aux_mv = params["aux_mv"]
        self.age_ms = params["age_ms"]

    def _handle_sensors_event(self, params):
        lane = params["lane"]
        logging.info(
            "ace2k sensors event: lane=%s kind=%s level=%d",
            "-" if lane == 255 else lane + 1,
            KIND_NAMES.get(params["kind"], params["kind"]),
            params["level"],
        )

    def _handle_thresholds(self, params):
        low_mv = struct.unpack(f"<{LANES}H", params["low_mv"])
        high_mv = struct.unpack(f"<{LANES}H", params["high_mv"])
        source = params["source"]
        for i, lane in enumerate(self.lanes):
            lane["low_mv"] = low_mv[i]
            lane["high_mv"] = high_mv[i]
            lane["source"] = CAL_NAMES.get(source[i], source[i])

    def _handle_env_state(self, params):
        valid = params["valid"]
        # the frame carries the last chamber value even when the chamber bit is clear; that is
        # not a humidity
        humidity = params["chamber_rh_pct10"] / 10.0 if valid & VALID_CHAMBER else None
        self.env = dict(
            ptc_left=params["ptc_left_mc"] / 1000.0,
            ptc_right=params["ptc_right_mc"] / 1000.0,
            chamber=params["chamber_mc"] / 1000.0,
            humidity=humidity,
            vdda=params["vdda_mv"] / 1000.0,
            valid_mask=valid,
            valid=[name for bit, name in sorted(VALID_NAMES.items()) if valid & bit],
        )
        sent_time = params["#sent_time"]
        self.reactor.register_async_callback(
            lambda eventtime, env=self.env, st=sent_time: self._notify_env(env, st)
        )

    def _notify_env(self, env, sent_time):
        print_time = self.mcu.estimated_print_time(sent_time)
        for listener in self.env_listeners:
            listener(print_time, env)

    def register_env_listener(self, cb):
        self.env_listeners.append(cb)

    def register_counters_listener(self, cb):
        """cb(receive_time, encoder_um): every lane-counters frame, on the serial thread."""
        self.counter_listeners.append(cb)

    def _handle_lane_counters(self, params):
        encoder_um = struct.unpack(f"<{LANES}i", params["encoder_um"])
        fg = struct.unpack(f"<{LANES}I", params["fg"])
        receive_time = params.get("#receive_time", 0.0)
        with self.encoder_lock:
            for i, lane in enumerate(self.lanes):
                if self.scale_pending[i] is not None:
                    self._follow_scale_change(i, encoder_um[i], receive_time)
                lane["encoder_um"] = encoder_um[i]
                lane["fg"] = fg[i]
        for cb in self.counter_listeners:
            cb(receive_time, encoder_um)

    def _follow_scale_change(self, lane, raw, receive_time):
        """A frame on a lane whose scale changed live, no frame at the new scale seen yet
        (encoder_lock held).  Received past the settle time, or nearer the reference's count read
        at the new scale than at the old, it is the first at the new scale: the offset goes in
        with it, so it reads on where the last left off.  Otherwise it reads at the old offset:
        the unit sent it before it took the change.  The value test assumes the scale's jump —
        about 1 % of the lane's travel since boot — dominates the motion within one frame; when
        it does not, the settle time still ends the wait.  The count stays the reference's: a
        frame read wrongly as one at the old scale never moves it."""
        old, new, count, settled = self.scale_pending[lane]
        if receive_time > settled or (
            abs(raw - count * new * 1000.0) < abs(raw - count * old * 1000.0)
        ):
            self.encoder_offset_um[lane] += int(round(count * (old - new) * 1000.0))
            self.scale_pending[lane] = None

    def _handle_mains_state(self, params):
        hz10 = params["hz10"]
        # a marker is no measurement: no frequency, its name instead
        self.mains_note = MAINS_MARKERS.get(hz10)
        self.mains_hz = None if self.mains_note else hz10 / 10.0
        self.mains_present = bool(params["present"])
        self.mains_rejects = params["rejects"]

    def set_counters_period(self, seconds):
        """The lane counters' period from now on (an automatic calibration runs them at 10 Hz);
        nothing on a firmware built without them."""
        if "lane" in self.present:
            ticks = self.mcu.seconds_to_clock(seconds)
            self.mcu.lookup_command("ace2k_lane_counters_query rest_ticks=%u").send([ticks])

    def keep_encoder_continuous(self, lane, old_scale, new_scale):
        """A live scale change makes the unit convert its whole count with the new scale; an
        offset keeps the reported encoder where it was, so a reader of encoder_mm sees no motion
        that did not happen.  Called before the scale is sent, so the last frame read — the
        reference — is at the old scale.  The offset goes in with the first frame at the new
        scale (_follow_scale_change): a frame the unit sent before it took the change still reads
        at the old one."""
        with self.encoder_lock:
            raw = self.lanes[lane]["encoder_um"]
            if raw is None:
                return  # no reading yet: nothing to keep
            pending = self.scale_pending[lane]
            if pending is not None:
                # no frame at the scale this one replaces yet: the change runs from the one before
                old_scale, _, count, _ = pending
            else:
                count = raw / (old_scale * 1000.0)
            if int(round(count * (old_scale - new_scale) * 1000.0)):
                settled = self.reactor.monotonic() + LANE_REPORT_S + SCALE_CHANGE_MARGIN_S
                self.scale_pending[lane] = (old_scale, new_scale, count, settled)
            else:
                self.scale_pending[lane] = None  # no offset to keep: no count, or no change

    def _drop_scale_changes(self, index):
        """A counters reset of lane index (LANE_ALL: every lane): the count starts again from 0,
        and the offset kept across a live scale change goes with it, as does a change pending."""
        with self.encoder_lock:
            for i in range(LANES) if index == LANE_ALL else [index]:
                self.encoder_offset_um[i] = 0
                self.scale_pending[i] = None

    def _lane_status(self, index, lane):
        """The lane's fields plus the encoder in millimetres (three decimals; None before the
        first report; continuous across a live scale change, encoder_um as the unit reports it),
        merged with the feed module's fields for the lane."""
        with self.encoder_lock:  # the reading and its offset from one frame
            status = dict(lane)
            um = lane["encoder_um"]
            if um is not None:
                um += self.encoder_offset_um[index]
        status["encoder_mm"] = None if um is None else round(um / 1000.0, 3)
        status.update(self.feed.lane_status(index))
        status["tag"] = self.rfid.lane_status(index)
        return status

    def get_status(self, eventtime):
        return {
            "version": self.version,
            "link_proven": self.link_proven,
            "lanes": [self._lane_status(i, lane) for i, lane in enumerate(self.lanes)],
            "pulled_any": self.pulled_any,
            "cutout": self.cutout,
            "aux_mv": self.aux_mv,
            "sensors_age_ms": self.age_ms,
            "mains_hz": self.mains_hz,
            # why mains_hz is None after a report: "missed edge" or "stale"; else None
            "mains_note": self.mains_note,
            "mains_present": self.mains_present,
            "mains_rejects": self.mains_rejects,
            "ptc_left": self.env["ptc_left"],
            "ptc_right": self.env["ptc_right"],
            "chamber": self.env["chamber"],
            "humidity": self.env["humidity"],
            "vdda": self.env["vdda"],
            "env_valid": self.env["valid"],
            "health": dict(self.health),
            # each reader's field, fault and dead flags; None before the first report
            "rfid_readers": self.rfid.reader_flags(),
            # the dryer, the fans and the flaps (ace2k_dryer.py); every field None before a report
            "dryer": self.dryer.status(),
            "uid": self.uid,
        }

    cmd_ACE_STATUS_help = (
        "Report the ace2k firmware state: link, lanes, switches, counters, mains, temperatures"
    )

    def cmd_ACE_STATUS(self, gcmd):
        proven = "yes" if self.link_proven else "no"
        lines = [f"ace2k {self.version}, link proven: {proven}"]
        for i, lane in enumerate(self.lanes):
            status = self._lane_status(i, lane)
            lines.append(
                f"lane {i + 1}: insert={lane['insert']} ({lane['insert_mv']} mV, band"
                f" {lane['low_mv']}-{lane['high_mv']} mV {lane['source']})"
                f" empty={lane['empty_mv']} mV rest={lane['rest']} pushed={lane['pushed']}"
                f" encoder={status['encoder_mm']} mm fg={lane['fg']}"
            )
        if self.feed.present:
            lines.extend(self.feed.status_lines())
        if self.rfid.present:
            lines.extend(self.rfid.status_lines())
        if self.dryer.airflow_present:
            lines.extend(self.dryer.status_lines())
        lines.append(
            f"pulled_any={self.pulled_any} cutout={self.cutout} aux={self.aux_mv} mV"
            f" sensors_age={self.age_ms} ms"
        )
        if self.mains_present is None:
            mains = "no report yet"
        else:
            mains = "present" if self.mains_present else "absent"
        hz = self.mains_note or f"{self.mains_hz} Hz"
        lines.append(f"mains={hz} ({mains}) rejects={self.mains_rejects}")
        e = self.env
        valid = ",".join(e["valid"]) or "none"
        lines.append(
            f"ptc_left={e['ptc_left']} C ptc_right={e['ptc_right']} C chamber={e['chamber']} C"
            f" humidity={e['humidity']} % vdda={e['vdda']} V valid={valid}"
        )
        gcmd.respond_info("\n".join(lines))

    cmd_ACE_RESTORE_STOCK_help = (
        "Reset the unit into its bootloader's recovery mode so the factory firmware can be "
        "restored over the wire (docs/flashing.md)"
    )

    def cmd_ACE_RESTORE_STOCK(self, gcmd):
        cmd = self.mcu.lookup_query_command(
            "ace2k_bootloader_enter", "ace2k_bootloader_enter_response accepted=%c"
        )
        params = cmd.send()
        if not params["accepted"]:
            gcmd.respond_raw("!! ace2k: refused — a subsystem is busy (heating or moving)")
            return
        gcmd.respond_info(
            "ace2k: entering the bootloader's recovery mode; stop Klipper, then run the "
            "updater (docs/flashing.md)"
        )

    cmd_ACE_CALIBRATION_SAVE_help = (
        "Stage the insert thresholds in use for SAVE_CONFIG, so they can be edited in printer.cfg"
    )

    def cmd_ACE_CALIBRATION_SAVE(self, gcmd):
        if any(lane["low_mv"] is None for lane in self.lanes):
            gcmd.respond_raw("!! ace2k: thresholds not received yet")
            return
        configfile = self.printer.lookup_object("configfile")
        for i, lane in enumerate(self.lanes):
            configfile.set(
                self.section, f"lane{i + 1}_insert_mv", f"{lane['low_mv']}, {lane['high_mv']}"
            )
        gcmd.respond_info("ace2k: thresholds staged; run SAVE_CONFIG to write them to printer.cfg")

    cmd_ACE_COUNTERS_RESET_help = "Zero the encoder and tach counters: LANE=<1-4> or LANE=ALL"

    def cmd_ACE_COUNTERS_RESET(self, gcmd):
        # a gcmd.error, which Klipper reports to the console; the unguarded lookup would raise
        # the message parser's error, and Klipper answers that with a shutdown
        if "lane" not in self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_LANE")
        if self.feed.any_busy():
            raise gcmd.error(
                "ace2k: a lane is moving or busy; the counters are the moves' odometers"
            )
        lane = gcmd.get("LANE", "ALL").upper()
        # a value outside 1..4 (or not a number) is gcmd's own error, before anything is sent
        index = LANE_ALL if lane == "ALL" else gcmd.get_int("LANE", minval=1, maxval=LANES) - 1
        which = "all lanes" if index == LANE_ALL else f"lane {index + 1}"
        if not self.reset_confirmed:
            # an older firmware takes the reset without answering: sent, not confirmed — and
            # one older still has no reset at all
            cmd = self.mcu.try_lookup_command(RESET_CMD)
            if cmd is None:
                raise gcmd.error("ace2k: this firmware has no counters reset")
            cmd.send([index])
            self._drop_scale_changes(index)
            gcmd.respond_info(
                f"ace2k: counters reset sent ({which}); this firmware does not confirm it"
            )
            return
        # the unit refuses on its own picture too — the backstop for a move accepted since the
        # last report, or one the unit started itself
        params = self.mcu.lookup_query_command(RESET_CMD, RESET_RESP).send([index])
        if not params["accepted"]:
            raise gcmd.error("ace2k: counters reset refused: a lane is moving or busy")
        self._drop_scale_changes(index)
        gcmd.respond_info(f"ace2k: counters reset ({which})")

    cmd_ACE_HEALTH_help = (
        "Report the self-test (RUN=1 re-runs it, CLEAR=1 clears the latched faults)"
    )

    def cmd_ACE_HEALTH(self, gcmd):
        if "health" not in self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_HEALTH")
        if gcmd.get_int("CLEAR", 0):
            self.mcu.lookup_command("ace2k_health_clear").send()
        if gcmd.get_int("RUN", 0):
            # the pass read so far: the re-read knows the new one by a post_ms that moved.  With
            # none read yet (before the first poll, or after it gave up) the current pass is
            # read first — a re-read with nothing to compare against would take the boot pass
            # for the new one; a unit that does not answer gets no run.
            if self.health["post_ms"] is None:
                try:
                    self._query_health()
                except self.printer.command_error as e:
                    raise gcmd.error(f"ace2k: {e}; self-test not started") from None
            self.health_rerun_post_ms = self.health["post_ms"]
            self.health_rerun_left = HEALTH_RERUN_RETRIES
            self.mcu.lookup_command("ace2k_health_run").send()
            # the re-read when the pass is done: the 2 s window, then the probe steps
            self.reactor.update_timer(
                self.health_rerun_timer, self.reactor.monotonic() + HEALTH_RERUN_POLL_S
            )
            gcmd.respond_info("ace2k: self-test started; results in about 2 s")
            return
        try:
            h = self._query_health()
        except self.printer.command_error:
            # the G-code reports the state rather than failing: the unit did not answer, and
            # the query marked the health unknown
            h = self.health
        if h["ok"] is None:
            verdict = "unknown (no answer from the unit)" if h["stale"] else "not run yet"
        elif h["ok"]:
            verdict = "OK"
        else:
            verdict = "FAILING " + ", ".join(h["failing"])
        latched = ", ".join(h["latched"]) or "none"
        when = "" if h["ok"] is None else f"; self-test at {h['post_ms']} ms"
        lines = [
            f"health: {verdict}",
            f"failed since boot or last clear: {latched}",
            f"last reset: {h['reset_cause']}{when}",
        ]
        gcmd.respond_info("\n".join(lines))


def probe_format(mcu, msgformat):
    """How the unit's dictionary answers for msgformat: ("ok", "") when it carries it exactly;
    ("differs", text) when it carries the name with another format; ("absent", text) otherwise.

    Klipper's message parser (msgproto.lookup_command) raises its msgproto.error with "Command
    format mismatch: <asked> vs <have>" for the second case and "Unknown command: <name>" for a
    name it lacks; mcu.check_valid_response() and try_lookup_command() swallow both, so the
    wrapper's own exception is read here — built on the format, never sent; the parser indexes
    responses by name and format as it does commands, so a response format probes the same way.
    Within that class only the mismatch wording is matched, and everything else is "absent": the
    benign outcome (a firmware built without the subsystem, or an older one, which the callers
    handle), and a rewording of the parser's text in a Klipper update must never turn a connect
    that worked into one that fails — the callers quote the text in their log line, so a
    rewording shows there.  Any other exception is not the dictionary's answer (a serial-state
    error, an API change) and propagates.
    """
    try:
        mcu.lookup_command(msgformat)
    except msgproto.error as e:
        text = str(e)
        return ("differs" if FORMAT_MISMATCH in text else "absent"), text
    return "ok", ""


def subscribe_response(mcu, cb, msgformat):
    """Register cb for the unsolicited report msgformat, on any Klipper this module runs on.

    The pinned Klipper has mcu.register_serial_response(cb, msgformat), which keeps the name
    before the first space and checks the whole format against the dictionary.  Older Klipper
    trees — Snapmaker's U1 fork among them — have only mcu.register_response(cb, name, oid),
    which checks nothing: the format is checked here first with mcu.lookup_command, which raises
    the message parser's msgproto.error for a format the dictionary lacks or carries
    differently, so a mismatch is refused on both, never a report silently dropped.  Both end in
    the serial reader's own register_response: the callback runs in the same thread either way.
    """
    register = getattr(mcu, "register_serial_response", None)
    if register is not None:
        register(cb, msgformat)
        return
    mcu.lookup_command(msgformat)
    mcu.register_response(cb, msgformat.split()[0])


def refuse_format(printer, section, msgformat, detail, verdict="differs", beside=None):
    """The config error that refuses a connect over probe_format's verdict on msgformat —
    RETURNED, never raised here, so that every call site reads ``raise refuse_format(...)`` and
    no path can fall through one.  "differs": the name is on the unit with another format.
    "absent": the name is missing from the dictionary; with ``beside``, the format that is
    there, the text says so — a binding declares both in one file, so this too is a firmware from
    another commit.  Either way the text carries the name, the parser's text and the hint: the
    unit's firmware and this module are from different commits, and either may be the stale
    one."""
    name = msgformat.split()[0]
    if verdict == "absent":
        sentence = f"{name} is not in the unit's dictionary"
        if beside is not None:
            sentence += f" though {beside.split()[0]} is"
    else:
        sentence = f"{name} has another format on the unit"
    return printer.config_error(f"{section}: {sentence} ({detail}); {FORMAT_HINT}")


def load_config(config):
    return Ace2k(config)
