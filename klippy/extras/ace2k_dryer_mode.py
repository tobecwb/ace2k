"""The dryer's modes on the host: manual, humidity and print.

The unit runs the humidity mode on its own, with no host (see "The dryer's modes" in
docs/protocol.md). This module chooses the mode, sends the limits and the profile, and shows what
the unit reports. The print mode is the host's (``ace2k_dryer_print.py``).

G-codes:

- ``ACEK_DRY_MODE MODE=manual|humidity|print`` chooses the mode. The unit keeps it across a power
  loss. ``humidity`` needs ``unattended_drying: True``.

At connect the host only asks for the 1 Hz report. On the first report, and then on a report at
most every 5 s, it brings the unit in line with the config, in this order:

1. the limits, when the unit's differ from ``dry_humidity_limit`` and ``dry_max_failures``;
2. the timings, once per connect (0 is the unit's default, so a removed bench option is undone);
3. a unit that holds the humidity mode while ``unattended_drying`` is False is set back to manual;
4. the profile, when it differs from the unit's.

The host never sends the humidity mode itself at connect, because the unit clears its "stopped
after failures" state on every humidity mode command.

The profile is the temperature and the time of an automatic cycle. The host works it out from the
lanes that hold a strand: the most fragile material wins, and no strand counts as PLA.

Another host module can choose the mode with ``set_mode(mode)`` and give each lane's material with
``set_material_source(fn)``. See ``docs/protocol.md``, "Host interface".

Configuration::

    [ace2k]
    #unattended_drying: False  # allow the humidity mode: the unit then heats on its own,
                               # with the printer off, for days or months
    #dry_humidity_limit: 40    # the box's RH, %, from which the automatic modes dry
    #dry_max_failures: 4       # failed cycles in a row that stop the humidity mode

    # For the bench only: shorter timings. Unset in normal use.
    #bench_dry_cycle_minutes:   # the automatic cycle's time instead of the material's
    #bench_dry_hold_s:          # the hold at or above the limit (default 1800)
    #bench_dry_cold_window_s:   # the cold box's window (default 1800)
    #bench_dry_link_s:          # a bound cycle's link time (default 300)

A failed request is logged once and tried again 5 s later.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from typing import TYPE_CHECKING, Callable, NamedTuple, Protocol

try:
    from .ace2k_dryer import (
        DRYER_RESP,
        MINUTES_MAX,
        MODE_CODES,
        MODE_NAMES,
        DryerRefused,
        check_cycle,
    )
    from .ace2k_dryer_print import PrintWatch
    from .ace2k_link import (
        NoAnswerError,
        ask,
        probe_format,
        probe_optional,
        refuse_format,
        subscribe_response,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_dryer import (  # type: ignore[import-not-found, no-redef]
        DRYER_RESP,
        MINUTES_MAX,
        MODE_CODES,
        MODE_NAMES,
        DryerRefused,
        check_cycle,
    )
    from ace2k_dryer_print import PrintWatch  # type: ignore[import-not-found, no-redef]
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        NoAnswerError,
        ask,
        probe_format,
        probe_optional,
        refuse_format,
        subscribe_response,
    )

if TYPE_CHECKING:
    from .ace2k_dryer import Ace2kDryer
    from .ace2k_klipper_types import (
        MCU,
        Config,
        GCodeCommand,
        LaneStatus,
        Params,
        QueryCommand,
    )

    class RfidProto(Protocol):
        """``ace2k_rfid.Ace2kRfid``, as the profile reads a lane's tag."""

        @property
        def lanes(self) -> list[LaneStatus]: ...

    class UnitProto(Protocol):
        """``ace2k.Ace2k``, the unit that the dryer belongs to."""

        @property
        def mcu(self) -> MCU: ...
        @property
        def name(self) -> str | None: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...
        @property
        def rfid(self) -> RfidProto: ...
        def head(self, part: str | None = None) -> str: ...
        def say(self, line: str) -> None: ...
        def register_command(
            self, name: str, func: Callable[[GCodeCommand], None], desc: str | None = None
        ) -> None: ...


HUMIDITY = MODE_CODES["humidity"]
STOPPED = 0x1  # the humidity mode stopped after its failed cycles
PROFILES = (("PLA", 45), ("PETG", 60), ("ABS", 65), ("TPU", 60))  # the page's presets
PROFILE_MINUTES = 360
DEFAULT_PROFILE = (45, PROFILE_MINUTES)  # PLA: an unknown material, or no strand
PROFILE_POLL_S = 5.0
REPORT_S = 1.0
LIMIT_DEFAULT = 40
FAILURES_DEFAULT = 4


class BenchTiming(NamedTuple):
    """The bench timings in seconds, 0 for the unit's own value, in the order of TIMING_FMT."""

    hold_s: int
    cold_window_s: int
    link_s: int


BENCH_TIMING = tuple(f"bench_dry_{name}" for name in BenchTiming._fields)  # their config keys
MODE_FMT = "ace2k_dryer_auto_mode mode=%c"
LIMITS_FMT = "ace2k_dryer_auto_limits limit_pct=%c max_failures=%c"
PROFILE_FMT = "ace2k_dryer_auto_profile target_c=%c minutes=%hu"
TIMING_FMT = "ace2k_dryer_auto_timing hold_s=%u cold_window_s=%u link_s=%u"
AUTO_RESP = "ace2k_dryer_auto_response op=%c accepted=%c reason=%c"
AUTO_QUERY = "ace2k_dryer_auto_query rest_ticks=%u"
AUTO_STATE = (
    "ace2k_dryer_auto_state mode=%c limit=%c max=%c failures=%c flags=%c profile_c=%c"
    " profile_min=%hu cold=%c own=%c bound=%c rh=%hu above_s=%u"
)
BOUND_FMT = "ace2k_dryer_start_bound target_c=%c minutes=%hu"
STOP_BOUND_FMT = "ace2k_dryer_stop_bound"  # an image before it: the plain stop
GATE_TEXT = "the humidity mode is off in printer.cfg (unattended_drying: True to allow it)"
OLD_IMAGE_TEXT = "ace2k: firmware without the dryer's modes — flash a newer image"
MODE_TEXT = "ace2k: the dryer mode is manual, humidity or print"
AUTO_REFUSAL_NAMES = {1: "range"}
SET_BACK_TEXT = "the humidity mode is off in printer.cfg: unit set back to manual"
KEPT_TEXT = "the humidity mode is off in printer.cfg, but the unit kept it"
TIMING_MAX_S = 4294967  # the unit's limit: the milliseconds must fit 32 bits
# The op each request is answered with (the start bound's on the dryer's response).
OPS = {MODE_FMT: 0, LIMITS_FMT: 1, PROFILE_FMT: 2, TIMING_FMT: 3, BOUND_FMT: 3, STOP_BOUND_FMT: 4}


def family_temp(material: str | None) -> int:
    """Return the drying temperature of a material's family. An unknown name is PLA."""
    name = (material or "").strip().upper()
    for family, temp in PROFILES:
        if name.startswith(family):
            return temp
    return DEFAULT_PROFILE[0]


class ModeRefused(RuntimeError):  # noqa: N818 — named as the dryer's DryerRefused
    """The unit refused a request of the modes.

    ``what`` is mode or profile. ``reason`` is the name from AUTO_REFUSAL_NAMES, or "reason N"
    for a code that this host does not know."""

    def __init__(self, what: str, code: int) -> None:
        self.what = what
        self.reason = AUTO_REFUSAL_NAMES.get(code, f"reason {code}")
        super().__init__(f"ace2k: dryer {what} refused: {self.reason}")


def rh_pct(state: Params) -> int:
    """The unit's last idle RH in whole %, half up (the report carries tenths)."""
    return int((state["rh"] + 5) // 10)


def mode_name(state: Params) -> object:
    """The report's mode by name, or its code for a mode this host does not know."""
    return MODE_NAMES.get(state["mode"], state["mode"])


# The mode's status keys in their order, each with how it is read from the report.
STATUS_FIELDS: tuple[tuple[str, Callable[[Params], object]], ...] = (
    ("mode", mode_name),
    ("limit", lambda s: s["limit"]),
    ("max_failures", lambda s: s["max"]),
    ("failures", lambda s: s["failures"]),
    ("stopped", lambda s: bool(s["flags"] & STOPPED)),
    ("cold", lambda s: bool(s["cold"])),
    ("above_s", lambda s: s["above_s"]),
    ("own", lambda s: bool(s["own"])),
    ("bound", lambda s: bool(s["bound"])),
    ("profile", lambda s: {"temp": s["profile_c"], "minutes": s["profile_min"]}),
    ("rh", rh_pct),
)


def cycles(n: int) -> str:
    return f"{n} failed cycle{'s' if n != 1 else ''}"


class Ace2kDryerMode:
    """The dryer's mode of one unit."""

    def __init__(self, config: Config, unit: UnitProto, dryer: Ace2kDryer) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.unit = unit
        self.dryer = dryer
        self.mcu = unit.mcu
        self.section = config.get_name()
        self.allowed = bool(config.getboolean("unattended_drying", False))
        self.limit = config.getint("dry_humidity_limit", LIMIT_DEFAULT, minval=5, maxval=95)
        self.max_failures = config.getint("dry_max_failures", FAILURES_DEFAULT, minval=1, maxval=20)
        self.bench_minutes = config.getint(
            "bench_dry_cycle_minutes", None, minval=1, maxval=MINUTES_MAX
        )
        self.bench = BenchTiming(
            *(config.getint(k, 0, minval=0, maxval=TIMING_MAX_S) for k in BENCH_TIMING)
        )
        self.present = False
        # The last report of the unit, None until the first one after a connect.
        self.state: Params | None = None
        self.source: Callable[[int], str | None] | None = None  # set_material_source
        # format -> its query, built at connect when the image has the modes
        self.requests: dict[str, QueryCommand] = {}
        # The reactor time from which a report brings the unit in line again.
        self.next_refresh = 0.0
        self.timing_sent = False  # once per connect
        # The requests that failed since they last went through: each is logged once.
        self.failing: set[str] = set()
        self.print_watch = PrintWatch(self)
        self.printer.register_event_handler("ace2k:dryer", self._on_dryer_event)
        self.mcu.register_config_callback(self._build_config)
        unit.register_command("ACEK_DRY_MODE", self.cmd_ACEK_DRY_MODE, self.cmd_ACEK_DRY_MODE_help)

    # --- configuration -------------------------------------------------------------------

    def _build_config(self) -> None:
        self.present = False
        self.state = None
        self.requests = {}
        self.next_refresh = 0.0
        self.timing_sent = False
        self.failing = set()
        if not self.dryer.dryer_present:
            return
        verdict, detail = probe_format(self.mcu, MODE_FMT)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, MODE_FMT, (verdict, detail))
        if verdict != "ok":
            logging.info(
                "%s firmware without the dryer's modes; ACEK_DRY_MODE is disabled (%s)",
                self.unit.head(),
                detail,
            )
            return
        rest = (LIMITS_FMT, PROFILE_FMT, TIMING_FMT, AUTO_RESP, AUTO_QUERY, AUTO_STATE, BOUND_FMT)
        for fmt in rest:
            found = probe_format(self.mcu, fmt)
            if found[0] != "ok":
                raise refuse_format(self.printer, self.section, fmt, found, MODE_FMT)
        self.present = True
        self.requests = {
            fmt: self.mcu.lookup_query_command(fmt, AUTO_RESP)
            for fmt in (MODE_FMT, LIMITS_FMT, PROFILE_FMT, TIMING_FMT)
        }
        self.requests[BOUND_FMT] = self.mcu.lookup_query_command(BOUND_FMT, DRYER_RESP)
        if probe_optional(self.printer, self.mcu, self.section, STOP_BOUND_FMT, MODE_FMT):
            self.requests[STOP_BOUND_FMT] = self.mcu.lookup_query_command(
                STOP_BOUND_FMT, DRYER_RESP
            )
        # The only init command: the limits and the timings go out with ask on the first report,
        # so their answers are read and checked.
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.mcu.add_config_cmd(f"ace2k_dryer_auto_query rest_ticks={ticks}", is_init=True)
        subscribe_response(self.mcu, self._handle_state, AUTO_STATE)

    # --- the report (the serial thread) --------------------------------------------------

    def _handle_state(self, params: Params) -> None:
        self.state = params
        self.reactor.register_async_callback(self._on_report)

    def _on_report(self, eventtime: float) -> None:
        """On the reactor: bring the unit in line with the config, at most every PROFILE_POLL_S.

        The wait also keeps a request that the next report has not shown yet from going twice."""
        self.print_watch.poll(eventtime)  # the print mode, on every report (1 Hz)
        if eventtime < self.next_refresh:
            return
        self.next_refresh = eventtime + PROFILE_POLL_S
        state = self.state
        assert state is not None  # scheduled by a report
        self._attempt("limits", self._sync_limits)
        if not self.timing_sent:
            self.timing_sent = self._attempt(
                "timing", lambda: self._send(TIMING_FMT, self.bench, "timing")
            )
        if state["mode"] == HUMIDITY and not self.allowed:
            self._set_back()
        self._attempt("profile", self._refresh_profile)

    def _on_dryer_event(self, event: dict[str, object]) -> None:
        """The dryer's events of this unit go to the print mode."""
        if event.get("unit") == self.unit.name:
            self.print_watch.on_dryer_event(event)

    def _attempt(
        self,
        what: str,
        request: Callable[[], None],
        report: Callable[[RuntimeError], None] | None = None,
    ) -> bool:
        """Run one request. Return True if it went through.

        A failure is reported once, until the request goes through again: ``report`` gets the
        error, and without it the failure is a log line."""
        try:
            request()
        except RuntimeError as e:  # NoAnswerError or ModeRefused
            if what not in self.failing:
                self.failing.add(what)
                if report is None:
                    logging.warning("%s %s not sent: %s", self.unit.head("dryer"), what, e)
                else:
                    report(e)
            return False
        self.failing.discard(what)
        return True

    def _sync_limits(self) -> None:
        state = self.state
        assert state is not None  # called from a report
        if (state["limit"], state["max"]) != (self.limit, self.max_failures):
            self._send(LIMITS_FMT, [self.limit, self.max_failures], "limits")

    def _set_back(self) -> None:
        """Set a stored humidity mode back to manual, as ``unattended_drying`` is False.

        Both outcomes are said on the console too: the user chose the mode, not this host."""
        if self._attempt(
            "set_back",
            lambda: self._send(MODE_FMT, [MODE_CODES["manual"]], "mode"),
            report=lambda e: self._say(f"{KEPT_TEXT}: {e}"),
        ):
            self._say(SET_BACK_TEXT)

    def _say(self, words: str) -> None:
        text = f"{self.unit.head('dryer')} {words}"
        logging.warning("%s", text)
        self.unit.say(text)

    # --- the profile ---------------------------------------------------------------------

    def set_material_source(self, fn: Callable[[int], str | None] | None) -> None:
        """Host interface: give each lane's material.

        ``fn(lane_index)`` returns the material's name, or None to use the tag's. None as ``fn``
        goes back to the tags."""
        self.source = fn

    def material(self, lane: int) -> str | None:
        """Return the material of a lane: the source's, else the tag's, else None.

        A source that raises counts as None. A material that is not text counts as no material."""
        name: object = None
        if self.source is not None:
            try:
                name = self.source(lane)
            except Exception as e:  # another module's code: never let it stop the report
                logging.warning(
                    "%s material source failed for lane %d: %s",
                    self.unit.head("dryer"),
                    lane + 1,
                    e,
                )
        if name is None:
            record = self.unit.rfid.lanes[lane]["record"]
            name = None if record is None else record.get("material")
        return name if isinstance(name, str) else None

    def profile_temp(self) -> int:
        """Return the profile's °C: the lowest temperature of the lanes that hold a strand."""
        temps = [
            family_temp(self.material(i))
            for i, lane in enumerate(self.unit.lanes)
            if lane["insert"]
        ]
        return min(temps) if temps else DEFAULT_PROFILE[0]

    def profile(self) -> tuple[int, int]:
        """Return (°C, minutes): profile_temp, and the bench minutes or PROFILE_MINUTES."""
        return self.profile_temp(), self.bench_minutes or PROFILE_MINUTES

    def _refresh_profile(self) -> None:
        """Send the profile when it differs from the unit's. Nothing before the first report."""
        state = self.state
        if not self.present or state is None:
            return
        want = self.profile()
        if want != (state["profile_c"], state["profile_min"]):
            self._send(PROFILE_FMT, list(want), "profile")

    # --- the host interface --------------------------------------------------------------

    def _ask(self, fmt: str, data: Sequence[int]) -> Params:
        """Send one request and return its answer. An answer for another request is none."""
        name = fmt.split(maxsplit=1)[0]
        params: Params = ask(self.printer, self.mcu, self.requests[fmt], data, name)
        if params["op"] != OPS[fmt]:
            raise NoAnswerError(f"ace2k: {name}: answer for another request (op {params['op']})")
        return params

    def _send(self, fmt: str, data: Sequence[int], what: str) -> None:
        """Send one request of the modes and wait for its answer.

        A refusal raises ModeRefused, and no answer NoAnswerError."""
        params = self._ask(fmt, data)
        if not params["accepted"]:
            raise ModeRefused(what, params["reason"])

    def _require(self) -> None:
        absent = self.dryer.absent(True)
        if absent:
            raise RuntimeError(absent)
        if not self.present:
            raise RuntimeError(OLD_IMAGE_TEXT)

    def set_mode(self, mode: str) -> None:
        """Host interface: choose the mode, as ACEK_DRY_MODE does.

        Raises ValueError for a name other than manual, humidity or print, and RuntimeError for
        humidity without ``unattended_drying: True`` or on an image without the modes; nothing is
        sent then. Raises ModeRefused (a RuntimeError) if the unit refuses, and NoAnswerError (a
        RuntimeError) if it does not answer."""
        code = MODE_CODES.get(mode)
        if code is None:
            raise ValueError(MODE_TEXT)
        self._require()
        if code == HUMIDITY and not self.allowed:
            raise RuntimeError(f"ace2k: {GATE_TEXT}")
        self._send(MODE_FMT, [code], "mode")

    def start_bound(self, target_c: int, minutes: int) -> None:
        """Start a cycle bound to the host: the unit ends it when the link stays down.

        For the print mode. Raises as ``Ace2kDryer.start`` does."""
        self._require()
        check_cycle(target_c, minutes)
        params = self._ask(BOUND_FMT, [target_c, minutes])
        if not params["accepted"]:
            raise DryerRefused("start", params["reason"])

    def stop_bound(self) -> None:
        """Stop the running cycle only if it is bound to the host. For the print mode.

        The unit decides, so a stop decided from an old report never ends a cycle that the user
        took over. An image without this command gets the dryer's plain stop. Raises as
        ``Ace2kDryer.stop`` does."""
        if STOP_BOUND_FMT not in self.requests:
            self.dryer.stop()
            return
        self._require()
        params = self._ask(STOP_BOUND_FMT, [])
        if not params["accepted"]:
            raise DryerRefused("stop", params["reason"])

    # --- status --------------------------------------------------------------------------

    def status(self) -> dict[str, object]:
        """The mode's fields of ``printer.ace2k.dryer``. A field is None before the first report."""
        s = self.state
        fields: dict[str, object] = {"humidity_allowed": self.allowed}
        fields.update((key, None if s is None else read(s)) for key, read in STATUS_FIELDS)
        return fields

    def status_line(self) -> str:
        s = self.state
        if s is None:
            return "dryer mode: no report yet"
        mode = mode_name(s)
        line = f"dryer mode: {mode}"
        if mode != "humidity":
            return line
        if s["flags"] & STOPPED:
            return f"{line} — stopped after {cycles(s['failures'])}: check the lid"
        if s["own"]:
            detail = "drying (auto)"
        elif not s["cold"]:
            detail = "box not steady yet, humidity not read"
        else:
            detail = f"waiting: box {rh_pct(s)} %, limit {s['limit']} %"
        if s["failures"]:
            detail += f" ({cycles(s['failures'])})"
        return f"{line} — {detail}"

    # --- the G-code ----------------------------------------------------------------------

    cmd_ACEK_DRY_MODE_help = "The dryer's mode: MODE=manual|humidity|print"

    def cmd_ACEK_DRY_MODE(self, gcmd: GCodeCommand) -> None:
        mode = str(gcmd.get("MODE", "") or "").strip().lower()
        try:
            self.set_mode(mode)
        except ModeRefused as e:
            raise gcmd.error(f"ace2k: dryer mode {mode} refused: {e.reason}") from None
        except (ValueError, RuntimeError) as e:
            raise gcmd.error(str(e)) from None
        gcmd.respond_info(f"{self.unit.head()} dryer mode {mode}")
