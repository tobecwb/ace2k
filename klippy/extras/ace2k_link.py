"""The helpers that every ace2k host module shares.

They are the constants of the link to the unit (lane count and report periods) and the query
function that holds the unit's one mutex. They also include the dictionary probes that refuse a
firmware from another commit, and the refusal that a start can end in.

This module imports no other ace2k module, so every other module can import it. ``ace2k.py``
re-exports its names.
"""

from __future__ import annotations

import weakref
from collections.abc import Sequence
from typing import TYPE_CHECKING, Callable, TypedDict

import msgproto  # klippy's own: the message parser's error class

if TYPE_CHECKING:
    from .ace2k_klipper_types import MCU, Command, Mutex, Params, Printer, QueryCommand


class EnvState(TypedDict):
    """The last environment report of the unit (``ace2k.Ace2k.env``), as its listeners get it.

    Temperatures are in degrees C, the chamber humidity is in %, and VDDA is in volts. A value is
    None before the first report. The humidity is also None while the chamber reading is invalid.
    ``valid`` holds the names of the valid readings, and ``valid_mask`` the same as bits.
    Other code reads the keys by name, which vulture cannot see, so some keys carry a noqa."""

    ptc_left: float | None  # noqa: V107
    ptc_right: float | None  # noqa: V107
    chamber: float | None  # noqa: V107
    humidity: float | None
    vdda: float | None  # noqa: V107
    valid: list[str]
    valid_mask: int  # noqa: V107


LANES = 4
SENSORS_REPORT_S = 0.1
ENV_REPORT_S = 1.0
LANE_REPORT_S = 1.0
MAINS_REPORT_S = 1.0
LANE_ALL = 255  # the lane index that means "every lane" in ace2k_lane_counters_reset
PAIR = 2  # a message says "both" for two lanes, and "all" for more
FORMAT_MISMATCH = "format mismatch"  # in the parser's text when a name has another format
FORMAT_HINT = (
    "the unit's firmware and this module are from different commits; update one to match the other"
)


NO_FEED = "firmware built without CONFIG_ACE2K_FEED"  # callers add a prefix
NO_LANE = "ace2k: no lane %s"  # the message that check_lane formats


def constant_or_none(mcu: MCU, name: str) -> float | None:
    """Return the firmware constant ``name``, or None when the image does not declare it.

    Klipper raises msgproto.error for a missing constant, and the test fake raises KeyError.
    Any other error is a real fault and is not hidden."""
    try:
        return mcu.get_constant_float(name)
    except (msgproto.error, KeyError):
        return None


def constants_or_none(mcu: MCU, names: Sequence[str]) -> dict[str, float] | None:
    """Return every constant in ``names``, or None when the image lacks any of them.

    The result is all of them or none, never a mix."""
    found: dict[str, float] = {}
    for name in names:
        value = constant_or_none(mcu, name)
        if value is None:
            return None
        found[name] = value
    return found


def bounded_constants(
    mcu: MCU, settings: Sequence[tuple[str, float]]
) -> dict[str, tuple[float, float, float]] | None:
    """Return, for each (name, scale) in ``settings``, the constant with its _MIN and _MAX.

    Each value is divided by the scale. The result is None when the image lacks any of the
    constants."""
    found = constants_or_none(
        mcu, [name + suffix for name, _scale in settings for suffix in ("", "_MIN", "_MAX")]
    )
    if found is None:
        return None
    return {
        name: (found[name] / scale, found[name + "_MIN"] / scale, found[name + "_MAX"] / scale)
        for name, scale in settings
    }


def check_lane(index: int, message: str) -> None:
    """Raise ValueError(message % (index,)) unless 0 <= index < LANES.

    A bool is not a lane. The index is passed in a tuple, so a tuple or another value that is not
    a number still gives the ValueError with its message."""
    if not whole(index) or not 0 <= index < LANES:
        raise ValueError(message % (index,))


class FeedRefused(Exception):  # noqa: N818 — the interface's name (API_VERSION 1)
    """The unit refused a start.

    ``reason`` is a name from REFUSAL_NAMES (busy, in_error, no_filament, no_link, bounds,
    other_lane). It can also be not_active, which the host raises itself, without sending
    anything, for a follow on a lane that is not the active lane of a shared extruder.
    A refused set_speed has ``mode`` "speed" and ``reason`` no_move, in_error, following or
    bounds. The host raises following itself, without sending anything, for an older image that
    would take the speed."""

    def __init__(self, lane: int, mode: str, reason: str) -> None:
        super().__init__(f"ace2k: lane {lane + 1} {mode} refused: {reason}")
        self.lane = lane
        self.mode = mode
        self.reason = reason


def whole(value: object) -> bool:
    """Return True if ``value`` is a whole number and not a bool.

    Host calls use it for lanes, and dryer calls use it for degrees and minutes."""
    return isinstance(value, int) and not isinstance(value, bool)


class NoAnswerError(RuntimeError):
    """A query that the unit did not answer within the retries of the transport."""


# One mutex per unit, shared by the queries of every module (see ask).
_QUERY_MUTEXES: weakref.WeakKeyDictionary[MCU, Mutex] = weakref.WeakKeyDictionary()


def query_mutex(printer: Printer, mcu: MCU) -> Mutex:
    """Return the mutex that every query to this unit holds while it is in flight."""
    mutex = _QUERY_MUTEXES.get(mcu)
    if mutex is None:
        mutex = _QUERY_MUTEXES[mcu] = printer.get_reactor().mutex()
    return mutex


def ask(
    printer: Printer, mcu: MCU, query: QueryCommand, data: Sequence[object], what: str
) -> Params:
    """Send a query and return its answer, with one query to the unit in flight at a time.

    Klipper keeps one handler per response name. Two overlapping queries with the same name, for
    example one from a timer and one from a G-code, would take each other's answer.
    If the transport fails (the printer's command_error, or the unit stays silent through the
    retries), this raises NoAnswerError. Its text names what was asked. The bare error is not
    raised, because it would take down the reactor when a timer raises it."""
    with query_mutex(printer, mcu):
        try:
            reply: Params = query.send(list(data))
        except printer.command_error as e:
            raise NoAnswerError(
                f"ace2k: {what}: no answer from the unit; check the link ({e})"
            ) from e
        return reply


def probe_format(mcu: MCU, msgformat: str) -> tuple[str, str]:
    """Say how the unit's dictionary answers for ``msgformat``.

    Return ("ok", "") when the dictionary has exactly this format. Return ("differs", text) when
    it has the name with another format. Return ("absent", text) in every other case.

    Klipper's message parser (msgproto.lookup_command) raises msgproto.error. The text is
    "Command format mismatch: <asked> vs <have>" for a name with another format, and
    "Unknown command: <name>" for a missing name. mcu.check_valid_response() and
    try_lookup_command() swallow both, so this function reads the exception itself. It only
    builds the command from the format and never sends it. The parser treats response formats
    like command formats, so they are probed the same way.

    Only the wording of the mismatch is matched. Everything else counts as "absent", which is the
    harmless case: a firmware built without the subsystem, or an older one, and the callers
    handle it. A Klipper update that rewords the text must not turn a working connect into a
    failing one. The callers put the text in their log line, so a rewording shows up there.
    Any other exception is not an answer from the dictionary (for example a serial error or an
    API change), so it propagates.
    """
    try:
        mcu.lookup_command(msgformat)
    except msgproto.error as e:
        text = str(e)
        return ("differs" if FORMAT_MISMATCH in text else "absent"), text
    return "ok", ""


def subscribe_response(mcu: MCU, cb: Callable[[Params], object], msgformat: str) -> None:
    """Register ``cb`` for the report ``msgformat`` that the unit sends on its own.

    This works on every Klipper that this module runs on. The pinned Klipper has
    mcu.register_serial_response(cb, msgformat). It keeps the name before the first space and
    checks the whole format against the dictionary. Older Klipper trees, such as the Snapmaker
    U1 fork, have only mcu.register_response(cb, name, oid), which checks nothing. So this
    function first calls mcu.lookup_command. It raises msgproto.error when the dictionary lacks
    the format or has another one. A mismatch is therefore refused on both, and no report is
    dropped silently. Both paths end in the serial reader's register_response, so the callback
    runs in the same thread either way.
    """
    register = getattr(mcu, "register_serial_response", None)
    if register is not None:
        register(cb, msgformat)
        return
    mcu.lookup_command(msgformat)
    mcu.register_response(cb, msgformat.split(maxsplit=1)[0])


def refuse_format(
    printer: Printer,
    section: str,
    msgformat: str,
    probe: tuple[str, str],
    beside: str | None = None,
) -> Exception:
    """Return the config error that refuses a connect, based on the answer of probe_format.

    ``probe`` is the verdict and the text that probe_format returned for ``msgformat``.
    The error is returned and not raised, so that every call site reads
    ``raise refuse_format(...)`` and no code path can fall through.

    With "differs", the name is on the unit with another format. With "absent", the name is
    missing from the dictionary. If ``beside`` is given, it is a format that is present, and the
    text says so. One source file declares both, so this also means a firmware from another
    commit. In both cases the text has the name, the parser's text and a hint: the firmware and
    this module are from different commits, and either one may be the old one."""
    verdict, detail = probe
    name = msgformat.split(maxsplit=1)[0]
    if verdict == "absent":
        sentence = f"{name} is not in the unit's dictionary"
        if beside is not None:
            sentence += f" though {beside.split()[0]} is"
    else:
        sentence = f"{name} has another format on the unit"
    return printer.config_error(f"{section}: {sentence} ({detail}); {FORMAT_HINT}")


def probe_optional(
    printer: Printer, mcu: MCU, section: str, msgformat: str, beside: str | None = None
) -> bool:
    """Say whether the unit's dictionary has ``msgformat``, for a format an older image may lack.

    Return True when the dictionary has exactly this format and False when the name is absent.
    Raise the config error of refuse_format (with ``beside``) when the name has another format,
    which means a firmware from another commit."""
    probe = probe_format(mcu, msgformat)
    if probe[0] == "differs":
        raise refuse_format(printer, section, msgformat, probe, beside)
    return probe[0] == "ok"


def optional_command(
    printer: Printer, mcu: MCU, section: str, msgformat: str, beside: str | None = None
) -> Command | None:
    """Return the command for ``msgformat``, or None when an older image lacks it.

    It raises as probe_optional does for a name with another format."""
    if not probe_optional(printer, mcu, section, msgformat, beside):
        return None
    return mcu.lookup_command(msgformat)
