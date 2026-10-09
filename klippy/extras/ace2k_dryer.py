"""ace2k_dryer — the dryer, the fans and the flaps of the ace2k firmware on the host.

Thin by design: the unit owns the heater, the interlock and every protection, and runs a cycle to
its end with no host.  This module sends the requests, waits for each answer and names a refusal
in words, shows the state, and passes the events on.  G-codes:

- ``ACE_DRY TEMP=<15-65> DURATION=<minutes, or 4h / 90m / 1.5h; at most 24 h>`` — start a cycle;
- ``ACE_DRY_STOP`` — stop it: the unit cools the heater down with the fans before it releases them;
- ``ACE_DRY_CLEAR`` — leave a fault once the heater is cool and the cause is gone (a tripped
  thermal cutout clears only with a power cycle, never with this);
- ``ACE_FAN ON=0|1 [SECONDS=<1-600>]`` and ``ACE_FLAP WHICH=bottom|rear OPEN=0|1`` — the outputs by
  hand, refused while the dryer holds them;
- ``ACE_DRYER_LOG`` — the unit's counters and its last eight faults or interrupted cycles.

The ``ace2k_dryer_state`` and ``ace2k_airflow_state`` reports (1 Hz) land in ``get_status()``
(``printer.ace2k.dryer``) and ``ACE_STATUS``; every ``ace2k_dryer_event`` is a log line and the
Klipper event ``ace2k:dryer`` (one dict: ``kind``, ``arg``, and ``reason`` or ``lane`` by name).
The unit holds its events until the first ``ace2k_dryer_query``, so that query is always an init
command: a unit that booted with its thermal cutout tripped says so then, as its first event
(``seq`` 0).  Every event carries a sequence number (0 at each unit boot, wrapping at 255).  The
unit's ``ace2k_dryer_event_ack`` is cumulative — it drops every event up to the one named — so the
host handles events strictly in sequence and acknowledges only the last one it handled: an event
that arrives after a gap is left for the unit's resend (after every query and every 5 s), which
starts from the missing one — the host asks for it at once with a query, once per gap; an event
at or behind the last handled one is acknowledged again but neither logged nor passed on twice.
After a connect the base comes from the unit: the first ``ace2k_dryer_state`` report carries
``oldest``, the seq of its oldest unacknowledged event (the next seq when it holds none).  An
event that arrives before that report is held — neither handled nor acknowledged — and taken in
order from the base once the report is in.
Events the unit had to drop (its queue was full) are counted in the state report:
``events_lost``, logged once as information at a connect and as a warning when it grows.  Just
after klippy:ready the log's counters are read once — a cycle run or a fault logged while no host
was there shows in them.
Loaded by ``[ace2k]``; a firmware built without ``CONFIG_ACE2K_HEAT`` is logged and every G-code
here answers an error; one without ``CONFIG_ACE2K_DRYER`` keeps the fan and flap G-codes.

    [ace2k]
    #heater_watts: 360     # the heater's power, for the log's energy (docs/hardware.md "Heater":
                           # ≈ 340–380 W measured at 127 V)
"""

import logging

try:
    from .ace2k import probe_format, refuse_format, subscribe_response
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k import probe_format, refuse_format, subscribe_response

TARGET_MIN_C = 15
TARGET_MAX_C = 65
MINUTES_MAX = 24 * 60
FAN_SECONDS_MAX = 600  # the unit's ACE2K_AIRFLOW_FAN_MANUAL_MAX_S
FAN_SECONDS_DEFAULT = 60
HEATER_WATTS = 360.0
REPORT_S = 1.0
READY_SEED_S = 0.1  # the log read, just after klippy:ready (a query blocks; its handlers may not)
LOG_ENTRIES = 8
LOG_READ_ATTEMPTS = 3  # a log read that saw an entry appended is read again, at most this often
LOG_TEMP_UNKNOWN_DC = -32768  # the unit's INT16_MIN: a reading that was invalid or absent

STATE_NAMES = {0: "idle", 1: "starting", 2: "heating", 3: "cooldown", 4: "fault"}
FAULT_NAMES = {
    1: "not_heating",
    2: "response",
    3: "sides",
    4: "chamber_over",
    5: "chamber_stale",
    6: "ntc",
    7: "mains",
    8: "cutout",
    9: "heat",
    10: "fans",
    11: "flaps",
}
REFUSAL_TEXT = {
    1: "busy — a cycle or its cool-down is running (ACE_DRY_STOP first), or the fans are held by a"
    " manual run (ACE_FAN ON=0 first)",
    2: "out of range — 15–65 °C, 1 min to 24 h",
    3: "the dryer is in a fault — ACE_DRY_CLEAR once it is cool",
    4: "a temperature sensor is not valid",
    5: "the mains is absent or its frequency implausible",
    6: "the thermal cutout tripped — power-cycle the unit",
    7: "the heater is still cooling down",
    8: "the unit went through a Klipper shutdown — FIRMWARE_RESTART before drying again",
    9: "the mains has just come back — its frequency is being measured; try ACE_DRY again in a"
    " few seconds",
}
EVENT_NAMES = {
    0: "done",
    1: "stopped",
    2: "fault",
    3: "lowered",
    4: "interrupted",
    5: "hot_ambient",
    6: "vented",
    7: "cleared",
}
NOTICE_LOG_UNSTORED = 0x4
NOTICE_AMBIENT_ABOVE = 0x20  # judged once, as the cycle started: not a live reading
NOTICE_BITS = (
    (0x1, "hot_ambient"),
    (0x2, "lowered"),
    (NOTICE_LOG_UNSTORED, "log_unstored"),
    (0x8, "mains_dip"),
    (0x10, "edges_off_phase"),
    (NOTICE_AMBIENT_ABOVE, "ambient_above"),
)
NOTICE_BITS_KNOWN = 0x3F
NOTICE_BITS_ALL = 8  # the field is one byte
FLAP_POS = {0: "unknown", 1: "open", 2: "closed"}
FLAPS = {"bottom": 0, "rear": 1}
OWNER_NAMES = {0: "none", 1: "manual", 3: "dryer"}  # the unit's values; 2 is not used
AIRFLOW_REFUSAL_TEXT = {
    1: "busy — the dryer holds the fans and flaps, or another flap pulse is running (try again)",
    2: "out of range",
}
LOG_CUTOUT = 0x2
LOG_CYCLE_OPEN = 0x1

FAN_FMT = "ace2k_fan_set on=%c seconds=%hu"
FLAP_FMT = "ace2k_flap_pulse flap=%c open=%c"  # the unit pulses for its compiled length
AIRFLOW_RESP = "ace2k_airflow_response op=%c accepted=%c reason=%c"
AIRFLOW_QUERY = "ace2k_airflow_query rest_ticks=%u"
AIRFLOW_STATE = "ace2k_airflow_state fans_cmd=%c fans_read=%c owner=%c flaps=%c"
START_FMT = "ace2k_dryer_start target_c=%c minutes=%hu"
STOP_FMT = "ace2k_dryer_stop"
CLEAR_FMT = "ace2k_dryer_clear"
DRYER_RESP = "ace2k_dryer_response op=%c accepted=%c reason=%c"
DRYER_QUERY = "ace2k_dryer_query rest_ticks=%u"
DRYER_STATE = (
    "ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c remaining_min=%hu fans=%c"
    " flaps=%c fault=%c notices=%c lost=%hu oldest=%c"
)
DRYER_EVENT = "ace2k_dryer_event kind=%c arg=%c seq=%c"
EVENT_ACK_FMT = "ace2k_dryer_event_ack seq=%c"
SEQ_MOD = 256  # the unit's sequence is 8-bit
LOG_QUERY = "ace2k_dryer_log_query index=%c"
LOG_FMT = "ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u"
GCODES = (
    "ACE_DRY",
    "ACE_DRY_STOP",
    "ACE_DRY_CLEAR",
    "ACE_FAN",
    "ACE_FLAP",
    "ACE_DRYER_LOG",
)


def parse_minutes(text):
    """DURATION as minutes: a bare number is minutes; an ``h`` or ``m`` suffix names the unit
    (``4h``, ``90m``, ``1.5h``).  None when it does not parse or lies outside 1..MINUTES_MAX."""
    if text is None:
        return None
    s = str(text).strip().lower()
    scale = 1.0
    if s.endswith("h"):
        s, scale = s[:-1], 60.0
    elif s.endswith("m"):
        s = s[:-1]
    try:
        value = float(s)
    except ValueError:
        return None
    if value != value or value in (float("inf"), float("-inf")):  # nan, inf
        return None
    minutes = round(value * scale)
    return minutes if 1 <= minutes <= MINUTES_MAX else None


def format_minutes(minutes):
    if minutes is None:
        return "?"
    hours, rest = divmod(int(minutes), 60)
    if hours and rest:
        return f"{hours} h {rest} min"
    return f"{hours} h" if hours else f"{rest} min"


def signed32(value):
    """A value the unit sends as %u of a signed quantity sign-extended to 32 bits."""
    return value - (1 << 32) if value & 0x80000000 else value


def format_dc(value):
    """A log temperature in 0.1 °C, sent sign-extended; the unit's unknown marker reads "?"."""
    dc = signed32(value)
    return "?" if dc == LOG_TEMP_UNKNOWN_DC else f"{dc / 10.0:.1f} °C"


def notices_of(bits):
    """The notice names, the known bits first; a bit this module does not know is kept by
    number, never dropped and never an error (a newer firmware may add one)."""
    names = [name for bit, name in NOTICE_BITS if bits & bit]
    names.extend(
        f"notice_0x{1 << n:02x}"
        for n in range(NOTICE_BITS_ALL)
        if bits & (1 << n) & ~NOTICE_BITS_KNOWN
    )
    return names


def flaps_of(bits):
    return {
        "bottom": FLAP_POS.get(bits & 0x3, "unknown"),
        "rear": FLAP_POS.get((bits >> 2) & 0x3, "unknown"),
    }


def fans_of(bits):
    return {"left": bool(bits & 0x1), "right": bool(bits & 0x2)}


class Ace2kDryer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        mcu_name = config.get("mcu", "ace2k")
        self.mcu = self.printer.lookup_object("mcu" if mcu_name == "mcu" else "mcu " + mcu_name)
        self.section = config.get_name()
        self.heater_watts = config.getfloat("heater_watts", HEATER_WATTS, above=0.0)
        self.airflow_present = self.dryer_present = False
        self.fields = dict(
            state=None,
            target=None,
            drive=None,
            duty=None,
            remaining=None,
            fans=None,
            flaps=None,
            fault=None,
            notices=[],
            owner=None,
            fans_commanded=None,
            events_lost=0,
        )
        self.log_counters = None
        self.log_unstored = False
        self.ack_cmd = None
        self.query_cmd = None
        # the next sequence to handle; None until the first state report after a connect
        self.next_seq = None
        # the events that arrived before that report, in arrival order
        self.held = []
        # the next_seq a gap already asked a resend for (once per gap)
        self.gap_asked = None
        # a state report was seen since the connect (the lost count stated once)
        self.lost_seen = False
        # One request in flight at a time: Klipper keys a query's response handler by the
        # response's name, and start, stop and clear share ace2k_dryer_response (fan and flap
        # share ace2k_airflow_response).
        self.mutex = self.reactor.mutex()
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        gcode = self.printer.lookup_object("gcode")
        for name in GCODES:
            gcode.register_command(
                name, getattr(self, "cmd_" + name), desc=getattr(self, "cmd_" + name + "_help")
            )

    # --- configuration -------------------------------------------------------------------

    def _check_all(self, formats, beside):
        """Every other format of a binding the probe found: one from another commit refuses the
        connect here instead of raising later in a G-code or a reactor callback."""
        for fmt in formats:
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(self.printer, self.section, fmt, detail, verdict, beside)

    def _probe(self, fmt):
        verdict, detail = probe_format(self.mcu, fmt)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, fmt, detail)
        return verdict == "ok", detail

    def _report(self, query):
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.mcu.add_config_cmd(f"{query.split()[0]} rest_ticks={ticks}", is_init=True)

    def _forget_events(self):
        """On every connect — from _build_config, which Klipper runs at each one, a host restart
        over a unit already configured included — the base is forgotten: the unit's sequence
        restarts at 0 at its boot (its boot's cutout is seq 0), and after a host restart it goes on
        from where it was.  The new base is the first state report's ``oldest``.  An event a
        previous host handled but whose acknowledgement had not landed is handled again."""
        self.next_seq = None
        self.held = []
        self.gap_asked = None
        self.lost_seen = False

    def _build_config(self):
        self.airflow_present = self.dryer_present = False
        self._forget_events()
        present, detail = self._probe(FAN_FMT)
        if not present:
            logging.warning(
                "ace2k: firmware built without CONFIG_ACE2K_HEAT; the dryer, fan and flap"
                " G-codes are disabled (%s)",
                detail,
            )
            return
        self._check_all((AIRFLOW_RESP, AIRFLOW_QUERY, AIRFLOW_STATE, FLAP_FMT), FAN_FMT)
        self.airflow_present = True
        self._report(AIRFLOW_QUERY)
        subscribe_response(self.mcu, self._handle_airflow, AIRFLOW_STATE)
        self._build_dryer()

    def _build_dryer(self):
        present, detail = self._probe(START_FMT)
        if not present:
            logging.warning(
                "ace2k: firmware built without CONFIG_ACE2K_DRYER; ACE_DRY, ACE_DRY_STOP,"
                " ACE_DRY_CLEAR and ACE_DRYER_LOG are disabled (%s)",
                detail,
            )
            return
        self._check_all(
            (
                STOP_FMT,
                CLEAR_FMT,
                DRYER_RESP,
                DRYER_QUERY,
                DRYER_STATE,
                DRYER_EVENT,
                EVENT_ACK_FMT,
                LOG_QUERY,
                LOG_FMT,
            ),
            START_FMT,
        )
        self.dryer_present = True
        self.ack_cmd = self.mcu.lookup_command(EVENT_ACK_FMT)
        self.query_cmd = self.mcu.lookup_command(DRYER_QUERY)
        # Always sent: the unit holds its events (a boot's cutout among them) until this query.
        self._report(DRYER_QUERY)
        subscribe_response(self.mcu, self._handle_state, DRYER_STATE)
        subscribe_response(self.mcu, self._handle_event, DRYER_EVENT)

    def _on_ready(self):
        """Schedule the log read: Klipper forbids a blocking query inside a klippy:ready
        handler, so it runs from a timer just after."""
        if self.dryer_present:
            self.reactor.register_timer(self._seed_log, self.reactor.monotonic() + READY_SEED_S)

    def _seed_log(self, eventtime):
        """The log's counters, once: what the unit did while no host was there."""
        try:
            with self.mutex:
                params = self.mcu.lookup_query_command(LOG_QUERY, LOG_FMT).send([0])
        except self.printer.command_error as e:
            logging.warning("ace2k: dryer log query failed: %s", e)
            return self.reactor.NEVER
        self.log_counters = self._counters(params)
        c = self.log_counters
        logging.info(
            "ace2k dryer log: %d cycles started, %d completed, %d faults, %.1f h of heating",
            c["cycles_started"],
            c["cycles_completed"],
            c["faults"],
            c["heat_s"] / 3600.0,
        )
        if c["cutout"]:
            logging.warning(
                "ace2k dryer: the thermal cutout tripped; the dryer stays refused until the unit"
                " is power-cycled (a restart does not clear it)"
            )
        return self.reactor.NEVER

    # --- reports and events (the serial thread) -------------------------------------------

    def _handle_state(self, params):
        state = STATE_NAMES.get(params["state"], params["state"])
        running = state != "idle"
        notices = notices_of(params["notices"])
        unstored = "log_unstored" in notices
        if unstored and not self.log_unstored:
            logging.warning(
                "ace2k dryer: the unit cannot store its log on the flash; it keeps retrying"
            )
        self.log_unstored = unstored
        lost = params["lost"]
        self._note_lost(lost)
        self.fields.update(
            state=state,
            target=params["target_c"] if running and params["target_c"] else None,
            drive=params["drive_dc"] / 10.0 if running and params["drive_dc"] else None,
            duty=params["duty"],
            remaining=params["remaining_min"] if running else None,
            fans=fans_of(params["fans"]),
            flaps=flaps_of(params["flaps"]),
            fault=FAULT_NAMES.get(params["fault"]),
            notices=notices,
            events_lost=lost,
        )
        if self.next_seq is None:
            self._take_base(params["oldest"])

    def _take_base(self, base):
        """The first report since a connect: the base, then the events held until it, from the
        base on in sequence order (a fresh event can arrive ahead of the resend)."""
        self.next_seq = base
        held, self.held = self.held, []
        held.sort(key=lambda p: (p["seq"] - base) % SEQ_MOD)
        for params in held:
            self._take(params)

    def _note_lost(self, lost):
        """The first report since a connect states the unit's count once, as information — it
        counts since the unit's boot, and a host restart would warn about it again; only a count
        that grows within the session is a warning."""
        if not self.lost_seen:
            self.lost_seen = True
            if lost:
                logging.info("ace2k dryer: %d events lost since the unit started", lost)
            return
        if lost > self.fields["events_lost"]:
            logging.warning(
                "ace2k dryer: %d events lost — the unit's queue was full before the host"
                " acknowledged them",
                lost,
            )

    def _handle_airflow(self, params):
        self.fields.update(
            fans_commanded=bool(params["fans_cmd"]),
            fans=fans_of(params["fans_read"]),
            owner=OWNER_NAMES.get(params["owner"], params["owner"]),
            flaps=flaps_of(params["flaps"]),
        )

    def _ack(self, seq):
        self.reactor.register_async_callback(lambda eventtime, seq=seq: self.ack_cmd.send([seq]))

    def _ask_resend(self):
        """A gap within a connection: a query makes the unit resend from the missing event at
        once instead of 5 s later — once per gap."""
        if self.gap_asked == self.next_seq:
            return
        self.gap_asked = self.next_seq
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.reactor.register_async_callback(lambda eventtime: self.query_cmd.send([ticks]))

    def _handle_event(self, params):
        if self.next_seq is None:
            self.held.append(params)  # the base not known yet: the first state report sets it
            return
        self._take(params)

    def _take(self, params):
        seq = params["seq"]
        if seq != self.next_seq:
            # behind: a resend already handled; ahead: one before it was lost and comes again in
            # the unit's resend, asked for at once.  Either way only the last handled one is
            # acknowledged — the acknowledgement is cumulative.
            if 0 < (seq - self.next_seq) % SEQ_MOD < SEQ_MOD // 2:
                self._ask_resend()
            self._ack((self.next_seq - 1) % SEQ_MOD)
            return
        self.next_seq = (seq + 1) % SEQ_MOD
        self._ack(seq)
        kind = EVENT_NAMES.get(params["kind"], params["kind"])
        arg = params["arg"]
        event = {"kind": kind, "arg": arg}
        if kind == "fault":
            event["reason"] = FAULT_NAMES.get(arg, arg)
            text = f"fault {event['reason']} — the heater stopped; the fans cool it down"
            if event["reason"] == "cutout":
                text += " (the thermal cutout: power-cycle the unit)"
        elif kind == "lowered":
            event["lane"] = arg  # the unit sends the lane 1-based
            text = (
                f"a filament was inserted on lane {arg} with no host:"
                " the target is lowered to 45 °C"
            )
        else:
            text = {
                "done": "cycle done",
                "stopped": "cycle stopped",
                "interrupted": "a cycle was interrupted by a reset and not resumed",
                "hot_ambient": "an NTC stays above 45 °C with the fans on (hot ambient)",
                "vented": "vented",
                "cleared": "fault cleared",
            }.get(kind, f"event {kind} ({arg})")
        level = logging.WARNING if kind in ("fault", "lowered", "interrupted") else logging.INFO
        logging.log(level, "ace2k dryer: %s", text)
        self.reactor.register_async_callback(
            lambda eventtime, event=event: self.printer.send_event("ace2k:dryer", event)
        )

    # --- status -----------------------------------------------------------------------------

    def status(self):
        return dict(self.fields)

    def _dryer_line(self):
        f = self.fields
        if f["state"] is None:
            return "dryer: no report yet"
        if f["state"] == "idle":
            line = "dryer: idle"
        else:
            line = f"dryer: {f['state']}"
            if f["target"] is not None:
                line += f" {f['target']} °C"
            if f["drive"] is not None:
                line += f" (drive {f['drive']:.1f} °C, duty {f['duty']} %)"
            if f["remaining"] is not None and f["state"] in ("starting", "heating"):
                line += f", {format_minutes(f['remaining'])} left"
            if f["fault"] is not None:
                line += f"; fault {f['fault']}"
        if f["notices"]:
            line += "; notices " + ", ".join(f["notices"])
        if f["events_lost"]:
            line += f"; {f['events_lost']} dryer events lost (the unit's queue was full)"
        return line

    def status_lines(self):
        f = self.fields
        lines = []
        if self.dryer_present:
            lines.append(self._dryer_line())
            if "log_unstored" in f["notices"]:
                lines.append(
                    "!! dryer: the unit cannot store its log on the flash (it keeps retrying)"
                )
            if "ambient_above" in f["notices"]:
                lines.append(
                    "dryer: the chamber started above the target: nothing heats until it falls"
                    " below (judged as the cycle started, not a live reading)"
                )
        fans = f["fans"]
        read = (
            "no report yet"
            if fans is None
            else f"L {'on' if fans['left'] else 'off'} R {'on' if fans['right'] else 'off'}"
        )
        commanded = {None: "?", True: "on", False: "off"}[f["fans_commanded"]]
        flaps = f["flaps"] or {"bottom": "unknown", "rear": "unknown"}
        lines.append(
            f"airflow: fans commanded {commanded}, read {read} (owner {f['owner']});"
            f" flaps bottom {flaps['bottom']}, rear {flaps['rear']} (as last commanded)"
        )
        return lines

    # --- the G-codes -------------------------------------------------------------------------

    def _require_airflow(self, gcmd):
        if not self.airflow_present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_HEAT")

    def _require_dryer(self, gcmd):
        self._require_airflow(gcmd)
        if not self.dryer_present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_DRYER")

    def _query(self, gcmd, fmt, resp, data=()):
        try:
            with self.mutex:
                return self.mcu.lookup_query_command(fmt, resp).send(list(data))
        except self.printer.command_error as e:
            raise gcmd.error(f"ace2k: {fmt.split()[0]}: no answer from the unit ({e})") from None

    def _dryer_request(self, gcmd, what, fmt, data=()):
        params = self._query(gcmd, fmt, DRYER_RESP, data)
        if not params["accepted"]:
            reason = REFUSAL_TEXT.get(params["reason"], f"reason {params['reason']}")
            raise gcmd.error(f"ace2k: dryer {what} refused: {reason}")

    def _airflow_request(self, gcmd, what, fmt, data):
        params = self._query(gcmd, fmt, AIRFLOW_RESP, data)
        if not params["accepted"]:
            reason = AIRFLOW_REFUSAL_TEXT.get(params["reason"], f"reason {params['reason']}")
            raise gcmd.error(f"ace2k: {what} refused: {reason}")

    cmd_ACE_DRY_help = "Dry: TEMP=<15-65> °C for DURATION=<minutes, or 4h / 90m; at most 24 h>"

    def cmd_ACE_DRY(self, gcmd):
        self._require_dryer(gcmd)
        temp = gcmd.get_int("TEMP", None, minval=TARGET_MIN_C, maxval=TARGET_MAX_C)
        if temp is None:
            raise gcmd.error("ace2k: ACE_DRY needs TEMP=<15-65>")
        minutes = parse_minutes(gcmd.get("DURATION", None))
        if minutes is None:
            raise gcmd.error(
                "ace2k: ACE_DRY needs DURATION=<minutes, or with an h / m suffix>, 1 min to 24 h"
            )
        self._dryer_request(gcmd, "start", START_FMT, [temp, minutes])
        gcmd.respond_info(f"ace2k: drying at {temp} °C for {format_minutes(minutes)}")

    cmd_ACE_DRY_STOP_help = "Stop drying; the fans run until the heater is cool"

    def cmd_ACE_DRY_STOP(self, gcmd):
        self._require_dryer(gcmd)
        self._dryer_request(gcmd, "stop", STOP_FMT)
        gcmd.respond_info("ace2k: dryer stopping — the unit cools the heater down with the fans")

    cmd_ACE_DRY_CLEAR_help = "Leave a dryer fault once the heater is cool (not a tripped cutout)"

    def cmd_ACE_DRY_CLEAR(self, gcmd):
        self._require_dryer(gcmd)
        self._dryer_request(gcmd, "clear", CLEAR_FMT)
        gcmd.respond_info("ace2k: dryer fault cleared")

    cmd_ACE_FAN_help = "Both fans by hand: ON=0|1 [SECONDS=<1-600>]; refused while drying"

    def cmd_ACE_FAN(self, gcmd):
        self._require_airflow(gcmd)
        on = gcmd.get_int("ON", None, minval=0, maxval=1)
        if on is None:
            raise gcmd.error("ace2k: ACE_FAN needs ON=0 or ON=1")
        seconds = 0
        if on:
            seconds = gcmd.get_int("SECONDS", FAN_SECONDS_DEFAULT, minval=1, maxval=FAN_SECONDS_MAX)
        self._airflow_request(gcmd, "fans", FAN_FMT, [on, seconds])
        if on:
            gcmd.respond_info(f"ace2k: fans on for {seconds} s")
            return
        gcmd.respond_info(self._fan_release_text())

    def _fan_release_text(self):
        """ON=0 releases the manual request; it does not switch the fans off.  The unit keeps them
        on while an outlet NTC reads above 45 °C (or invalid) and while the heater holds them."""
        text = (
            "ace2k: manual fan request released — the unit keeps the fans on while an outlet"
            " NTC is above 45 °C or the heater holds them"
        )
        commanded = self.fields["fans_commanded"]
        if commanded is None:
            return text
        last = "on" if commanded else "off"
        return f"{text} (last report, before this request: commanded {last})"

    cmd_ACE_FLAP_help = "One exhaust flap by hand: WHICH=bottom|rear OPEN=0|1; refused while drying"

    def cmd_ACE_FLAP(self, gcmd):
        self._require_airflow(gcmd)
        which = (gcmd.get("WHICH", "") or "").strip().lower()
        if which not in FLAPS:
            raise gcmd.error("ace2k: ACE_FLAP needs WHICH=bottom or WHICH=rear")
        opening = gcmd.get_int("OPEN", None, minval=0, maxval=1)
        if opening is None:
            raise gcmd.error("ace2k: ACE_FLAP needs OPEN=0 or OPEN=1")
        if gcmd.get("MS", None) is not None:
            raise gcmd.error(
                "ace2k: ACE_FLAP takes no MS=; the unit pulses for its compiled length"
            )
        self._airflow_request(gcmd, f"{which} flap", FLAP_FMT, [FLAPS[which], opening])
        verb = "opened" if opening else "closed"
        gcmd.respond_info(f"ace2k: {which} flap pulsed {verb} (no read-back: look at it)")

    cmd_ACE_DRYER_LOG_help = "The dryer's counters and its last eight faults or interrupted cycles"

    def _counters(self, params):
        return dict(
            cycles_started=params["a"],
            cycles_completed=params["b"],
            heat_s=params["c"],
            full_power_s=params["d"],
            faults=params["e"],
            cycle_open=bool(params["f"] & LOG_CYCLE_OPEN),
            cutout=bool(params["f"] & LOG_CUTOUT),
        )

    def _entry_line(self, n, params):
        a = params["a"]
        kind = EVENT_NAMES.get(a & 0xFF, a & 0xFF)
        reason = (a >> 8) & 0xFF
        what = f"fault {FAULT_NAMES.get(reason, reason)}" if kind == "fault" else kind
        temps = ", ".join(
            f"{name} {format_dc(params[key])}"
            for name, key in (("left", "c"), ("right", "d"), ("chamber", "e"))
        )
        cycle = a >> 16
        when = f"cycle {cycle}" if cycle else "no cycle open"
        return f"#{n} {when} at {params['b'] / 3600.0:.1f} h: {what} — {temps}"

    @staticmethod
    def _log_mark(params):
        """What changes when an entry is appended: the cycle and fault counts and the flags —
        not the heating seconds, which advance through every cycle."""
        return tuple(params[k] for k in ("a", "b", "e", "f"))

    def _read_log(self, gcmd):
        """The counters, the entries, the counters again: a snapshot when the two agree."""
        first = self._query(gcmd, LOG_QUERY, LOG_FMT, [0])
        entries = []
        for n in range(1, LOG_ENTRIES + 1):
            params = self._query(gcmd, LOG_QUERY, LOG_FMT, [n])
            if params["a"] & 0xFF == 0:  # kind 0 (done) is never logged: an empty slot
                break
            entries.append(params)
        last = self._query(gcmd, LOG_QUERY, LOG_FMT, [0])
        return last, entries, self._log_mark(first) == self._log_mark(last)

    def cmd_ACE_DRYER_LOG(self, gcmd):
        self._require_dryer(gcmd)
        for _ in range(LOG_READ_ATTEMPTS):
            counters, entries, stable = self._read_log(gcmd)
            if stable:
                break
        c = self._counters(counters)
        self.log_counters = c
        energy_wh = c["full_power_s"] * self.heater_watts / 3600.0
        lines = [
            f"ace2k dryer: {c['cycles_started']} cycles started, {c['cycles_completed']}"
            f" completed, {c['faults']} fault{'s' if c['faults'] != 1 else ''};"
            f" {c['heat_s'] / 3600.0:.1f} h of heating, {energy_wh:.0f} Wh at"
            f" {self.heater_watts:g} W"
        ]
        if not stable:
            lines.append(
                f"the log changed while it was read ({LOG_READ_ATTEMPTS} tries):"
                " the entries below may mix two states"
            )
        if c["cutout"]:
            lines.append("the thermal cutout tripped: power-cycle the unit to clear it")
        for n, params in enumerate(entries, start=1):
            lines.append(self._entry_line(n, params))
        gcmd.respond_info("\n".join(lines))


def load_config(config):
    # loaded by [ace2k] with printer.load_object(config, "ace2k_dryer"); the keys live in [ace2k]
    return Ace2kDryer(config.getsection("ace2k"))
