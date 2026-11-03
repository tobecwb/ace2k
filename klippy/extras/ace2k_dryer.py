"""The dryer, the fans and the flaps of the ace2k firmware, on the host.

This module is thin by design. The unit owns the heater, the interlock and every protection, and
it runs a cycle to its end without a host. This module sends the requests and waits for each
answer. It names a refusal in words, shows the state, and passes the events on.

G-codes:

- ``ACEK_DRY TEMP=<15-65> DURATION=<minutes, or 4h / 90m / 1.5h; at most 24 h>`` starts a cycle.
- ``ACEK_DRY_STOP`` stops the cycle. The unit cools the heater down with the fans before it
  releases them.
- ``ACEK_DRY_CLEAR`` leaves a fault, once the heater is cool and the cause is gone. A tripped
  thermal cutout clears only with a power cycle, never with this command.
- ``ACEK_FAN ON=0|1 [SECONDS=<1-600>]`` and ``ACEK_FLAP WHICH=bottom|rear OPEN=0|1`` control the
  outputs by hand. They are refused while the dryer holds them.
- ``ACEK_DRYER_LOG`` shows the counters of the unit and its last eight faults or interrupted
  cycles.

Another host module can start, stop and clear the dryer with ``start(target_c, minutes)``,
``stop()`` and ``clear()`` on this object. These work where a G-code cannot be used, such as in a
timer or an async callback. A refusal raises ``DryerRefused``. See ``docs/protocol.md``.

Reports and events
------------------
The reports ``ace2k_dryer_state`` and ``ace2k_airflow_state`` arrive at 1 Hz. They go into
``get_status()`` (``printer.ace2k.dryer``) and into ``ACEK_STATUS``. Every ``ace2k_dryer_event``
is written to the log and sent as the Klipper event ``ace2k:dryer``. The event is one dict with
``kind``, ``arg``, and ``reason`` or ``lane`` by name.

The unit holds its events until the first ``ace2k_dryer_query``. So that query is always an init
command. A unit that booted with its thermal cutout tripped says so then, as its first event
(``seq`` 0).

Event order and acknowledgement
-------------------------------
Every event carries a sequence number. It is 0 at each boot of the unit and wraps at 255.
The ``ace2k_dryer_event_ack`` of the unit is cumulative. It drops every event up to the one that
it names. So the host handles the events strictly in sequence, and it acknowledges only the last
event that it handled.

- An event that arrives after a gap is left for the resend of the unit. The unit resends after
  every query and every 5 s, and it starts from the missing event. The host asks for it at once
  with a query, once per gap.
- An event at or behind the last handled one is acknowledged again. It is not logged or passed on
  twice.
- After a connect, the base comes from the unit. The first ``ace2k_dryer_state`` report carries
  ``oldest``. This is the seq of its oldest event without an acknowledgement. If it holds none,
  it is the next seq. An event that arrives before that report is held. It is neither handled
  nor acknowledged. It is taken in order from the base once the report has arrived.

When the queue of the unit is full, it has to drop events. It counts them in the state report as
``events_lost``. The host logs this once as information at a connect, and as a warning when the
count grows.

Just after klippy:ready, the host reads the counters of the log once. A cycle that ran, or a
fault that was logged while no host was there, shows in them.

``[ace2k]`` loads this module. If the firmware is built without ``CONFIG_ACE2K_HEAT``, a log line
says so, and every G-code here answers with an error. A firmware without ``CONFIG_ACE2K_DRYER``
keeps the fan and flap G-codes.

Configuration::

    [ace2k]
    #heater_watts: 360     # the power of the heater, used for the energy in the log
                           # (see docs/hardware.md, "Heater"; measured about 340-380 W at 127 V)
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from typing import TYPE_CHECKING, Callable, Protocol, TypedDict

try:
    from .ace2k_link import (
        NoAnswerError,
        ask,
        probe_format,
        refuse_format,
        subscribe_response,
        whole,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        NoAnswerError,
        ask,
        probe_format,
        refuse_format,
        subscribe_response,
        whole,
    )

if TYPE_CHECKING:
    from .ace2k_klipper_types import MCU, Command, Config, GCodeCommand, Params, QueryCommand

    class UnitProto(Protocol):
        """``ace2k.Ace2k``, the unit that the dryer belongs to."""

        @property
        def mcu(self) -> MCU: ...
        @property
        def name(self) -> str | None: ...
        def head(self, part: str | None = None) -> str: ...
        def say(self, line: str) -> None: ...
        def lane_text(self, lane: int) -> str: ...
        def register_command(
            self, name: str, func: Callable[[GCodeCommand], None], desc: str | None = None
        ) -> None: ...


TARGET_MIN_C = 15
TARGET_MAX_C = 65
MINUTES_MAX = 24 * 60
FAN_SECONDS_MAX = 600  # ACE2K_AIRFLOW_FAN_MANUAL_MAX_S of the unit
FAN_SECONDS_DEFAULT = 60
HEATER_WATTS = 360.0
REPORT_S = 1.0
READY_SEED_S = 0.1  # delay of the log read after klippy:ready; a query blocks, a handler must not
LOG_ENTRIES = 8
LOG_READ_ATTEMPTS = 3  # a log read that saw a new entry is repeated, at most this many times
LOG_TEMP_UNKNOWN_DC = -32768  # INT16_MIN of the unit: the reading was invalid or missing

# The dryer modes' names by the unit's code. ace2k_dryer_print and ace2k_dryer_mode re-export both.
MODE_NAMES = {0: "manual", 1: "humidity", 2: "print"}
MODE_CODES = {v: k for k, v in MODE_NAMES.items()}
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
    12: "lid",
}
# The console text of a fault whose cause the owner can act on. Other faults read the common text.
FAULT_HINTS = {
    "lid": "fault lid — the chamber does not follow the heater: is the lid open?"
    " The heater stopped; the fans cool it down (ACEK_DRY_CLEAR once it is cool)",
}
REFUSAL_TEXT = {
    1: "busy — a cycle or its cool-down is running (ACEK_DRY_STOP first), or the fans are held by a"
    " manual run (ACEK_FAN ON=0 first)",
    2: "out of range — 15–65 °C, 1 min to 24 h",
    3: "the dryer is in a fault — ACEK_DRY_CLEAR once it is cool",
    4: "a temperature sensor is not valid",
    5: "the mains is absent or its frequency implausible",
    6: "the thermal cutout tripped — power-cycle the unit",
    7: "the heater is still cooling down",
    8: "the unit went through a Klipper shutdown — FIRMWARE_RESTART before drying again",
    9: "the mains has just come back — its frequency is being measured; try ACEK_DRY again in a"
    " few seconds",
}
# The refusals by the names that the firmware uses (see "The dryer" in docs/protocol.md).
# They are used for DryerRefused.reason.
REFUSAL_NAMES = {
    1: "busy",
    2: "range",
    3: "faulted",
    4: "sensors",
    5: "no_mains",
    6: "cutout",
    7: "not_cool",
    8: "shutdown",
    9: "measuring",
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
    8: "auto_started",
    9: "auto_dry",
    10: "auto_failed",
    11: "auto_stopped",
    12: "auto_off",
    13: "stopped_link",
}
# The events logged as warnings; the others are information.
WARN_EVENTS = ("fault", "lowered", "interrupted", "auto_stopped", "stopped_link")
# The events of the automatic modes are said on the console too: no G-code asked for them.
SAID_EVENTS = (
    "auto_started",
    "auto_dry",
    "auto_failed",
    "auto_stopped",
    "auto_off",
    "stopped_link",
)
NOTICE_LOG_UNSTORED = 0x4
NOTICE_AMBIENT_ABOVE = 0x20  # judged once, when the cycle started; it is not a live reading
NOTICE_BITS = (
    (0x1, "hot_ambient"),
    (0x2, "lowered"),
    (NOTICE_LOG_UNSTORED, "log_unstored"),
    (0x8, "mains_dip"),
    (0x10, "edges_off_phase"),
    (NOTICE_AMBIENT_ABOVE, "ambient_above"),
)
NOTICE_BITS_KNOWN = 0x3F
NOTICE_BITS_ALL = 8  # the field is one byte wide
FLAP_POS = {0: "unknown", 1: "open", 2: "closed"}
FLAPS = {"bottom": 0, "rear": 1}
OWNER_NAMES = {0: "none", 1: "manual", 3: "dryer"}  # values of the unit; 2 is not used
AIRFLOW_REFUSAL_TEXT = {
    1: "busy — the dryer holds the fans and flaps, or another flap pulse is running (try again)",
    2: "out of range",
}
LOG_CUTOUT = 0x2
LOG_CYCLE_OPEN = 0x1

FAN_FMT = "ace2k_fan_set on=%c seconds=%hu"
FLAP_FMT = "ace2k_flap_pulse flap=%c open=%c"  # the unit pulses for a length set at build time
FAN_NAME = FAN_FMT.split(maxsplit=1)[0]
FLAP_NAME = FLAP_FMT.split(maxsplit=1)[0]
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
SEQ_MOD = 256  # the sequence number of the unit is 8 bits wide
LOG_QUERY = "ace2k_dryer_log_query index=%c"
LOG_NAME = LOG_QUERY.split(maxsplit=1)[0]
LOG_FMT = "ace2k_dryer_log index=%c a=%u b=%u c=%u d=%u e=%u f=%u"
GCODES = (
    "ACEK_DRY",
    "ACEK_DRY_STOP",
    "ACEK_DRY_CLEAR",
    "ACEK_FAN",
    "ACEK_FLAP",
    "ACEK_DRYER_LOG",
)


class Fields(TypedDict):
    """The last reports of the dryer and of the airflow, as status() publishes them.

    Other code reads the keys by name, which vulture cannot see, so some keys carry a noqa."""

    state: str | int | None  # a name from STATE_NAMES, or the number of a state not known here
    target: int | None  # noqa: V107
    drive: float | None  # noqa: V107
    duty: int | None
    remaining: int | None  # noqa: V107
    fans: dict[str, bool] | None
    flaps: dict[str, str] | None
    fault: str | None  # noqa: V107
    notices: list[str]
    owner: str | int | None
    fans_commanded: bool | None  # noqa: V107
    events_lost: int  # noqa: V107


class Counters(TypedDict):
    """The counters of the dryer log (its entry 0).

    Other code reads the keys by name, which vulture cannot see, so some keys carry a noqa."""

    cycles_started: int  # noqa: V107
    cycles_completed: int  # noqa: V107
    heat_s: int  # noqa: V107
    full_power_s: int  # noqa: V107
    faults: int  # noqa: V107
    cycle_open: bool  # noqa: V107
    cutout: bool


def parse_minutes(text: object) -> int | None:
    """Return DURATION as whole minutes, or None.

    A bare number means minutes. An ``h`` or ``m`` suffix names the unit (``4h``, ``90m``,
    ``1.5h``). The result is None if the text cannot be parsed or is outside 1..MINUTES_MAX."""
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
    if not math.isfinite(value):  # nan, inf
        return None
    minutes = round(value * scale)
    return minutes if 1 <= minutes <= MINUTES_MAX else None


def check_cycle(target_c: int, minutes: int) -> None:
    """Raise ValueError unless target_c is whole °C in 15..65 and minutes whole in 1..MINUTES_MAX.

    The message names the argument that is out of range. Nothing is sent."""
    if not whole(target_c) or not TARGET_MIN_C <= target_c <= TARGET_MAX_C:
        raise ValueError(f"ace2k: dryer target {target_c!r} outside 15–65 °C")
    if not whole(minutes) or not 1 <= minutes <= MINUTES_MAX:
        raise ValueError(f"ace2k: dryer minutes {minutes!r} outside 1–{MINUTES_MAX}")


def format_minutes(minutes: int | None) -> str:
    """Return minutes as text such as "1 h 30 min". Return "?" for None."""
    if minutes is None:
        return "?"
    hours, rest = divmod(int(minutes), 60)
    if hours and rest:
        return f"{hours} h {rest} min"
    return f"{hours} h" if hours else f"{rest} min"


def signed32(value: int) -> int:
    """Convert a value that the unit sends as %u back to a signed 32-bit number.

    The unit sends a signed quantity sign-extended to 32 bits."""
    return value - (1 << 32) if value & 0x80000000 else value


def format_dc(value: int) -> str:
    """Return a log temperature as text. The unit sends it in 0.1 °C, sign-extended.

    The marker of the unit for an unknown value is shown as "?"."""
    dc = signed32(value)
    return "?" if dc == LOG_TEMP_UNKNOWN_DC else f"{dc / 10.0:.1f} °C"


def notices_of(bits: int) -> list[str]:
    """Return the notice names, with the known bits first.

    A bit that this module does not know is kept by its number. It is never dropped and never an
    error, because a newer firmware may add one."""
    names = [name for bit, name in NOTICE_BITS if bits & bit]
    names.extend(
        f"notice_0x{1 << n:02x}"
        for n in range(NOTICE_BITS_ALL)
        if bits & (1 << n) & ~NOTICE_BITS_KNOWN
    )
    return names


def flaps_of(bits: int) -> dict[str, str]:
    return {
        "bottom": FLAP_POS.get(bits & 0x3, "unknown"),
        "rear": FLAP_POS.get((bits >> 2) & 0x3, "unknown"),
    }


def fans_of(bits: int) -> dict[str, bool]:
    return {"left": bool(bits & 0x1), "right": bool(bits & 0x2)}


class DryerRefused(Exception):  # noqa: N818 — the interface's name (API_VERSION 13)
    """The unit refused a host call.

    ``what`` is start, stop or clear. ``reason`` is the name from REFUSAL_NAMES, or "reason N"
    for a code that this host does not know. ``code`` is the number from the unit."""

    def __init__(self, what: str, code: int) -> None:
        self.what = what
        self.code = code
        self.reason = REFUSAL_NAMES.get(code, f"reason {code}")
        super().__init__(f"ace2k: dryer {what} refused: {self.reason}")


class Ace2kDryer:
    """The dryer, the fans and the flaps of one unit."""

    def __init__(self, config: Config, unit: UnitProto) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.unit = unit
        self.mcu = unit.mcu  # the MCU of the unit, resolved once from the mcu key of its section
        self.section = config.get_name()
        self.heater_watts = config.getfloat("heater_watts", HEATER_WATTS, above=0.0)
        self.airflow_present = self.dryer_present = False
        self.fields = Fields(
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
        self.log_counters: Counters | None = None
        self.log_unstored = False
        # format -> its query, built at connect when the image has the dryer
        self.requests: dict[str, QueryCommand] = {}
        # format -> its query, built at connect together with the fans and flaps
        self.airflow_queries: dict[str, QueryCommand] = {}
        self.log_query: QueryCommand | None = None  # the query of the dryer log, built at connect
        self.ack_cmd: Command | None = None
        self.query_cmd: Command | None = None
        # The next sequence number to handle. It is None until the first state report after a
        # connect.
        self.next_seq: int | None = None
        self.last_event_was_lid_fault = False
        # The events that arrived before that report, in the order of arrival.
        self.held: list[Params] = []
        # The next_seq for which a gap has already asked for a resend (once per gap).
        self.gap_asked: int | None = None
        # True once a state report has been seen since the connect. The lost count is stated once.
        self.lost_seen = False
        # Only one request can be in flight at a time. Start, stop and clear share
        # ace2k_dryer_response, and fan and flap share ace2k_airflow_response. ace2k_link.ask
        # holds the one query mutex of the unit.
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        for name in GCODES:
            unit.register_command(
                name, getattr(self, "cmd_" + name), getattr(self, "cmd_" + name + "_help")
            )

    # --- configuration -------------------------------------------------------------------

    def _check_all(self, formats: Sequence[str], beside: str) -> None:
        """Check every other format of a binding that the probe found.

        A format from another commit refuses the connect here. Otherwise it would raise later,
        in a G-code or in a reactor callback."""
        for fmt in formats:
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(self.printer, self.section, fmt, (verdict, detail), beside)

    def _probe(self, fmt: str) -> tuple[bool, str]:
        """Return (present, detail) for a format.

        A format with another layout refuses the connect."""
        verdict, detail = probe_format(self.mcu, fmt)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, fmt, (verdict, detail))
        return verdict == "ok", detail

    def _report(self, query: str) -> None:
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.mcu.add_config_cmd(f"{query.split(maxsplit=1)[0]} rest_ticks={ticks}", is_init=True)

    def _forget_events(self) -> None:
        """Forget the event base at every connect.

        _build_config calls this. Klipper runs it at each connect, including a host restart over
        a unit that is already configured. The sequence of the unit restarts at 0 when it boots,
        and the cutout of that boot is seq 0. After a host restart, the sequence goes on from
        where it was. The new base is ``oldest`` of the first state report. An event that a
        previous host handled, but whose acknowledgement had not arrived, is handled again."""
        self.next_seq = None
        self.last_event_was_lid_fault = False
        self.held = []
        self.gap_asked = None
        self.lost_seen = False

    def _build_config(self) -> None:
        self.airflow_present = self.dryer_present = False
        self._forget_events()
        present, detail = self._probe(FAN_FMT)
        if not present:
            logging.warning(
                "%s firmware built without CONFIG_ACE2K_HEAT; the dryer, fan and flap"
                " G-codes are disabled (%s)",
                self.unit.head(),
                detail,
            )
            return
        self._check_all((AIRFLOW_RESP, AIRFLOW_QUERY, AIRFLOW_STATE, FLAP_FMT), FAN_FMT)
        self.airflow_present = True
        # Built once, like the requests of the dryer.
        self.airflow_queries = {
            fmt: self.mcu.lookup_query_command(fmt, AIRFLOW_RESP) for fmt in (FAN_FMT, FLAP_FMT)
        }
        self._report(AIRFLOW_QUERY)
        subscribe_response(self.mcu, self._handle_airflow, AIRFLOW_STATE)
        self._build_dryer()

    def _build_dryer(self) -> None:
        present, detail = self._probe(START_FMT)
        if not present:
            logging.warning(
                "%s firmware built without CONFIG_ACE2K_DRYER; ACEK_DRY, ACEK_DRY_STOP,"
                " ACEK_DRY_CLEAR and ACEK_DRYER_LOG are disabled (%s)",
                self.unit.head(),
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
        # Built once. Every request reuses them, whether it comes from a G-code or from a host
        # module in a timer.
        self.requests = {
            fmt: self.mcu.lookup_query_command(fmt, DRYER_RESP)
            for fmt in (START_FMT, STOP_FMT, CLEAR_FMT)
        }
        self.log_query = self.mcu.lookup_query_command(LOG_QUERY, LOG_FMT)
        self.ack_cmd = self.mcu.lookup_command(EVENT_ACK_FMT)
        self.query_cmd = self.mcu.lookup_command(DRYER_QUERY)
        # Always sent. The unit holds its events until this query, including the cutout of a boot.
        self._report(DRYER_QUERY)
        subscribe_response(self.mcu, self._handle_state, DRYER_STATE)
        subscribe_response(self.mcu, self._handle_event, DRYER_EVENT)

    def _on_ready(self) -> None:
        """Schedule the read of the log.

        Klipper does not allow a blocking query inside a klippy:ready handler. So the read runs
        from a timer, just after."""
        if self.dryer_present:
            self.reactor.register_timer(self._seed_log, self.reactor.monotonic() + READY_SEED_S)

    def _seed_log(self, _eventtime: float) -> float:
        """Read the counters of the log once.

        They show what the unit did while no host was there."""
        assert self.log_query is not None  # scheduled only after the connect built it
        try:
            params = ask(self.printer, self.mcu, self.log_query, [0], "dryer log")
        except NoAnswerError as e:
            logging.warning("%s dryer log query failed: %s", self.unit.head(), e)
            return self.reactor.NEVER
        self.log_counters = self._counters(params)
        c = self.log_counters
        logging.info(
            "%s %d cycles started, %d completed, %d faults, %.1f h of heating",
            self.unit.head("dryer log"),
            c["cycles_started"],
            c["cycles_completed"],
            c["faults"],
            c["heat_s"] / 3600.0,
        )
        if c["cutout"]:
            logging.warning(
                "%s the thermal cutout tripped; the dryer stays refused until the unit"
                " is power-cycled (a restart does not clear it)",
                self.unit.head("dryer"),
            )
        return self.reactor.NEVER

    # --- reports and events (the serial thread) -------------------------------------------

    def _handle_state(self, params: Params) -> None:
        state = STATE_NAMES.get(params["state"], params["state"])
        running = state != "idle"
        notices = notices_of(params["notices"])
        unstored = "log_unstored" in notices
        if unstored and not self.log_unstored:
            logging.warning(
                "%s the unit cannot store its log on the flash; it keeps retrying",
                self.unit.head("dryer"),
            )
        self.log_unstored = unstored
        lost = params["lost"]
        self._note_lost(lost)
        self.fields.update(
            {
                "state": state,
                "target": params["target_c"] if running and params["target_c"] else None,
                "drive": params["drive_dc"] / 10.0 if running and params["drive_dc"] else None,
                "duty": params["duty"],
                "remaining": params["remaining_min"] if running else None,
                "fans": fans_of(params["fans"]),
                "flaps": flaps_of(params["flaps"]),
                "fault": FAULT_NAMES.get(params["fault"]),
                "notices": notices,
                "events_lost": lost,
            }
        )
        if self.next_seq is None:
            self._take_base(params["oldest"])

    def _take_base(self, base: int) -> None:
        """Take the base from the first report since a connect, then handle the held events.

        The held events are handled from the base on, in sequence order. A new event can arrive
        ahead of the resend."""
        self.next_seq = base
        held, self.held = self.held, []
        held.sort(key=lambda p: (p["seq"] - base) % SEQ_MOD)
        for params in held:
            self._take(params)

    def _note_lost(self, lost: int) -> None:
        """Report the count of lost events.

        The first report since a connect states the count once, as information. The unit counts
        since its boot, and a host restart would warn about it again. Only a count that grows
        within the session is a warning."""
        if not self.lost_seen:
            self.lost_seen = True
            if lost:
                logging.info(
                    "%s %d events lost since the unit started", self.unit.head("dryer"), lost
                )
            return
        if lost > self.fields["events_lost"]:
            logging.warning(
                "%s %d events lost — the unit's queue was full before the host acknowledged them",
                self.unit.head("dryer"),
                lost,
            )

    def _handle_airflow(self, params: Params) -> None:
        self.fields.update(
            {
                "fans_commanded": bool(params["fans_cmd"]),
                "fans": fans_of(params["fans_read"]),
                "owner": OWNER_NAMES.get(params["owner"], params["owner"]),
                "flaps": flaps_of(params["flaps"]),
            }
        )

    def _ack(self, seq: int) -> None:
        self.reactor.register_async_callback(lambda _eventtime: self._send_ack(seq))

    def _send_ack(self, seq: int) -> None:
        assert self.ack_cmd is not None  # built by the connect that subscribed to the events
        self.ack_cmd.send([seq])

    def _send_query(self, ticks: int) -> None:
        assert self.query_cmd is not None  # built by the connect that subscribed to the events
        self.query_cmd.send([ticks])

    def _ask_resend(self) -> None:
        """Ask the unit to resend after a gap within a connection.

        A query makes the unit resend from the missing event at once and not 5 s later. This is
        done once per gap."""
        if self.gap_asked == self.next_seq:
            return
        self.gap_asked = self.next_seq
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.reactor.register_async_callback(lambda _eventtime: self._send_query(ticks))

    def _handle_event(self, params: Params) -> None:
        if self.next_seq is None:
            self.held.append(params)  # the base is not known yet; the first state report sets it
            return
        self._take(params)

    def _take(self, params: Params) -> None:
        """Handle one event. If it is not the next in sequence, acknowledge the last handled event
        again, and ask for a resend if events were skipped."""
        seq = params["seq"]
        assert self.next_seq is not None  # an event is taken only after the base is known
        if seq != self.next_seq:
            # An event behind the expected one is a resend that was already handled. An event
            # ahead of it means that an earlier one was lost. It comes again in the resend of the
            # unit, which the host asks for at once. In both cases only the last handled event is
            # acknowledged, because the acknowledgement is cumulative.
            if 0 < (seq - self.next_seq) % SEQ_MOD < SEQ_MOD // 2:
                self._ask_resend()
            self._ack((self.next_seq - 1) % SEQ_MOD)
            return
        self.next_seq = (seq + 1) % SEQ_MOD
        self._ack(seq)
        code: int = params["kind"]
        kind = EVENT_NAMES.get(code, code)
        arg = params["arg"]
        event: dict[str, object] = {"kind": kind, "arg": arg}
        # An automatic cycle that ended in the lid fault is followed at once by auto_failed;
        # only a lid fault right before it changes the line. Any event clears the memory.
        after_lid_fault, self.last_event_was_lid_fault = self.last_event_was_lid_fault, False
        if kind == "fault":
            reason = event["reason"] = FAULT_NAMES.get(arg, arg)
            self.last_event_was_lid_fault = reason == "lid"
            text = FAULT_HINTS.get(str(reason)) or (
                f"fault {reason} — the heater stopped; the fans cool it down"
            )
            if reason == "cutout":
                text += " (the thermal cutout: power-cycle the unit)"
        elif kind == "lowered":
            event["lane"] = arg  # the unit sends the lane number starting at 1
            text = (
                f"a filament was inserted on lane {self.unit.lane_text(arg - 1)} with no host:"
                " the target is lowered to 45 °C"
            )
        else:
            texts: dict[str | int, str] = {
                "done": "cycle done",
                "stopped": "cycle stopped",
                "interrupted": "a cycle was interrupted by a reset and not resumed",
                "hot_ambient": "an NTC stays above 45 °C with the fans on (hot ambient)",
                "vented": "vented",
                "cleared": "fault cleared",
                "auto_started": f"automatic cycle started at {arg} °C: the box stayed at or"
                " above the limit",
                "auto_dry": f"automatic cycle done: the box reads {arg} % cold, under the limit",
                "auto_failed": f"automatic cycle done but the box is still humid ({arg} failed in"
                " a row)",
                "auto_stopped": f"the humidity mode stopped after {arg} failed cycles in a row:"
                " check the lid or the limit",
                "auto_off": "the humidity mode is off: an automatic cycle was stopped by hand"
                " (choose it again to turn it back on)",
                "stopped_link": "the print mode's cycle ended: the host was gone for the link time",
            }
            if kind == "auto_failed" and after_lid_fault:
                texts[kind] = f"automatic cycle ended by the lid fault ({arg} failed in a row)"
            text = texts.get(kind, f"event {kind} ({arg})")
        line = f"{self.unit.head('dryer')} {text}"
        logging.log(logging.WARNING if kind in WARN_EVENTS else logging.INFO, "%s", line)
        said = kind in SAID_EVENTS
        self.reactor.register_async_callback(lambda _eventtime: self._pass_on(event, line, said))

    def _pass_on(self, event: dict[str, object], line: str, said: bool) -> None:
        """On the reactor: say the line on the console if asked, and send the Klipper event."""
        if said:
            self.unit.say(line)
        self.printer.send_event("ace2k:dryer", dict(event, unit=self.unit.name))

    # --- status -----------------------------------------------------------------------------

    def status(self) -> dict[str, object]:
        return dict(self.fields)

    def _dryer_line(self) -> str:
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

    def status_lines(self) -> list[str]:
        f = self.fields
        lines: list[str] = []
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

    def absent(self, dryer: bool) -> str | None:
        """Return, in words, what the image lacks, or None if it has everything.

        With ``dryer`` False, only the fans and flaps are needed. With ``dryer`` True, the dryer
        is needed too."""
        if not self.airflow_present:
            return "ace2k: firmware built without CONFIG_ACE2K_HEAT"
        if dryer and not self.dryer_present:
            return "ace2k: firmware built without CONFIG_ACE2K_DRYER"
        return None

    def _require_airflow(self, gcmd: GCodeCommand) -> None:
        absent = self.absent(False)
        if absent:
            raise gcmd.error(absent)

    def _require_dryer(self, gcmd: GCodeCommand) -> None:
        absent = self.absent(True)
        if absent:
            raise gcmd.error(absent)

    def _query(
        self, gcmd: GCodeCommand, query: QueryCommand, name: str, data: Sequence[int] = ()
    ) -> Params:
        try:
            params: Params = ask(self.printer, self.mcu, query, data, name)
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from None
        return params

    def _dryer_request(
        self, gcmd: GCodeCommand, what: str, fmt: str, data: Sequence[int] = ()
    ) -> None:
        try:
            self._request(what, fmt, data)
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from None
        except DryerRefused as e:
            text = REFUSAL_TEXT.get(e.code, f"reason {e.code}")
            raise gcmd.error(f"ace2k: dryer {what} refused: {text}") from None

    def _request(self, what: str, fmt: str, data: Sequence[int] = ()) -> None:
        """Send one dryer request and wait for the answer.

        A refusal raises DryerRefused. No answer raises NoAnswerError."""
        params = ask(self.printer, self.mcu, self.requests[fmt], data, fmt.split(maxsplit=1)[0])
        if not params["accepted"]:
            raise DryerRefused(what, params["reason"])

    # --- the host interface ---------------------------------------------------

    def _require_host(self) -> None:
        absent = self.absent(True)
        if absent:
            raise RuntimeError(absent)

    def start(self, target_c: int, minutes: int) -> None:
        """Host interface: start a cycle, as ACEK_DRY does.

        ``target_c`` is whole °C from 15 to 65. ``minutes`` is whole minutes from 1 to 1440.
        This can be called from G-code handlers, timers and async callbacks. It waits only for the
        answer of the unit, and it returns None once the unit has taken the request.

        Raises ValueError for an argument out of range, and sends nothing. Raises DryerRefused if
        the unit refuses. Raises NoAnswerError (a RuntimeError) if the unit does not answer.
        Raises RuntimeError on an image without the dryer."""
        self._require_host()
        check_cycle(target_c, minutes)
        self._request("start", START_FMT, [target_c, minutes])

    def stop(self) -> None:
        """Host interface: stop the cycle, as ACEK_DRY_STOP does.

        The unit cools the heater down with the fans. If no cycle runs, nothing changes. It raises
        the same errors as ``start``."""
        self._require_host()
        self._request("stop", STOP_FMT)

    def clear(self) -> None:
        """Host interface: leave a fault, as ACEK_DRY_CLEAR does.

        This works once the heater is cool and the cause is gone. It never clears a tripped
        cutout. Outside a fault, nothing changes. It raises the same errors as ``start``."""
        self._require_host()
        self._request("clear", CLEAR_FMT)

    def _airflow_request(
        self, gcmd: GCodeCommand, what: str, fmt: str, name: str, data: Sequence[int]
    ) -> None:
        params = self._query(gcmd, self.airflow_queries[fmt], name, data)
        if not params["accepted"]:
            reason = AIRFLOW_REFUSAL_TEXT.get(params["reason"], f"reason {params['reason']}")
            raise gcmd.error(f"ace2k: {what} refused: {reason}")

    cmd_ACEK_DRY_help = "Dry: TEMP=<15-65> °C for DURATION=<minutes, or 4h / 90m; at most 24 h>"

    def cmd_ACEK_DRY(self, gcmd: GCodeCommand) -> None:
        self._require_dryer(gcmd)
        temp = gcmd.get_int("TEMP", None, minval=TARGET_MIN_C, maxval=TARGET_MAX_C)
        if temp is None:
            raise gcmd.error("ace2k: ACEK_DRY needs TEMP=<15-65>")
        minutes = parse_minutes(gcmd.get("DURATION", None))
        if minutes is None:
            raise gcmd.error(
                "ace2k: ACEK_DRY needs DURATION=<minutes, or with an h / m suffix>, 1 min to 24 h"
            )
        self._dryer_request(gcmd, "start", START_FMT, [temp, minutes])
        gcmd.respond_info(f"ace2k: drying at {temp} °C for {format_minutes(minutes)}")

    cmd_ACEK_DRY_STOP_help = "Stop drying; the fans run until the heater is cool"

    def cmd_ACEK_DRY_STOP(self, gcmd: GCodeCommand) -> None:
        self._require_dryer(gcmd)
        self._dryer_request(gcmd, "stop", STOP_FMT)
        gcmd.respond_info("ace2k: dryer stopping — the unit cools the heater down with the fans")

    cmd_ACEK_DRY_CLEAR_help = "Leave a dryer fault once the heater is cool (not a tripped cutout)"

    def cmd_ACEK_DRY_CLEAR(self, gcmd: GCodeCommand) -> None:
        self._require_dryer(gcmd)
        self._dryer_request(gcmd, "clear", CLEAR_FMT)
        gcmd.respond_info("ace2k: dryer fault cleared")

    cmd_ACEK_FAN_help = "Both fans by hand: ON=0|1 [SECONDS=<1-600>]; refused while drying"

    def cmd_ACEK_FAN(self, gcmd: GCodeCommand) -> None:
        self._require_airflow(gcmd)
        on = gcmd.get_int("ON", None, minval=0, maxval=1)
        if on is None:
            raise gcmd.error("ace2k: ACEK_FAN needs ON=0 or ON=1")
        if not on and gcmd.get("SECONDS", None) is not None:
            raise gcmd.error("ace2k: ACEK_FAN SECONDS applies only with ON=1")
        seconds = 0
        if on:
            seconds = gcmd.get_int("SECONDS", FAN_SECONDS_DEFAULT, minval=1, maxval=FAN_SECONDS_MAX)
        self._airflow_request(gcmd, "fans", FAN_FMT, FAN_NAME, [on, seconds])
        if on:
            gcmd.respond_info(f"ace2k: fans on for {seconds} s")
            return
        gcmd.respond_info(self._fan_release_text())

    def _fan_release_text(self) -> str:
        """Return the text for ON=0.

        ON=0 releases the manual request. It does not switch the fans off. The unit keeps them on
        while an outlet NTC reads above 45 °C or is invalid, and while the heater holds them."""
        text = (
            "ace2k: manual fan request released — the unit keeps the fans on while an outlet"
            " NTC is above 45 °C or the heater holds them"
        )
        commanded = self.fields["fans_commanded"]
        if commanded is None:
            return text
        last = "on" if commanded else "off"
        return f"{text} (last report, before this request: commanded {last})"

    cmd_ACEK_FLAP_help = (
        "One exhaust flap by hand: WHICH=bottom|rear OPEN=0|1; refused while drying"
    )

    def cmd_ACEK_FLAP(self, gcmd: GCodeCommand) -> None:
        self._require_airflow(gcmd)
        which = (gcmd.get("WHICH", "") or "").strip().lower()
        if which not in FLAPS:
            raise gcmd.error("ace2k: ACEK_FLAP needs WHICH=bottom or WHICH=rear")
        opening = gcmd.get_int("OPEN", None, minval=0, maxval=1)
        if opening is None:
            raise gcmd.error("ace2k: ACEK_FLAP needs OPEN=0 or OPEN=1")
        if gcmd.get("MS", None) is not None:
            raise gcmd.error(
                "ace2k: ACEK_FLAP takes no MS=; the unit pulses for its compiled length"
            )
        self._airflow_request(gcmd, f"{which} flap", FLAP_FMT, FLAP_NAME, [FLAPS[which], opening])
        verb = "opened" if opening else "closed"
        gcmd.respond_info(f"ace2k: {which} flap pulsed {verb} (no read-back: look at it)")

    cmd_ACEK_DRYER_LOG_help = "The dryer's counters and its last eight faults or interrupted cycles"

    def _counters(self, params: Params) -> Counters:
        return Counters(
            cycles_started=params["a"],
            cycles_completed=params["b"],
            heat_s=params["c"],
            full_power_s=params["d"],
            faults=params["e"],
            cycle_open=bool(params["f"] & LOG_CYCLE_OPEN),
            cutout=bool(params["f"] & LOG_CUTOUT),
        )

    def _entry_line(self, n: int, params: Params) -> str:
        """Return the console line for entry ``n`` of the dryer log."""
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
    def _log_mark(params: Params) -> tuple[int, ...]:
        """Return the values that change when an entry is appended.

        They are the cycle count, the fault count and the flags. The heating seconds are not
        included, because they advance during every cycle."""
        return tuple(params[k] for k in ("a", "b", "e", "f"))

    def _read_log(self, gcmd: GCodeCommand) -> tuple[Params, list[Params], bool]:
        """Read the counters, the entries, and the counters again.

        The result is a consistent snapshot when the two readings of the counters agree."""
        query = self.log_query
        assert query is not None  # ACEK_DRYER_LOG runs only on an image with the dryer
        first = self._query(gcmd, query, LOG_NAME, [0])
        entries = []
        for n in range(1, LOG_ENTRIES + 1):
            params = self._query(gcmd, query, LOG_NAME, [n])
            if params["a"] & 0xFF == 0:  # kind 0 (done) is never logged, so this is an empty slot
                break
            entries.append(params)
        last = self._query(gcmd, query, LOG_NAME, [0])
        return last, entries, self._log_mark(first) == self._log_mark(last)

    def cmd_ACEK_DRYER_LOG(self, gcmd: GCodeCommand) -> None:
        self._require_dryer(gcmd)
        for _ in range(LOG_READ_ATTEMPTS):
            counters, entries, stable = self._read_log(gcmd)
            if stable:
                break
        c = self._counters(counters)
        self.log_counters = c
        energy_wh = c["full_power_s"] * self.heater_watts / 3600.0
        lines = [
            f"{self.unit.head('dryer')} {c['cycles_started']} cycles started,"
            f" {c['cycles_completed']}"
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
