"""What the flasher does on the Klipper host.

This module covers:

- Moonraker, the flasher's way to talk to Klipper;
- the restore command, and what Klipper prints after it;
- what Klipper reports about the unit;
- which programs hold the unit's port open;
- how to stop and start Klipper's service.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Literal, cast

from .wire import RefusedError

if TYPE_CHECKING:
    from collections.abc import Callable
    from typing import Any, Protocol

    from .wire import Json

    class MoonrakerProto(Protocol):
        """The Moonraker calls the flasher makes: Moonraker, or a simulated host in a test."""

        def state(self) -> str | None: ...
        def objects(self) -> Json | None: ...
        def gcode(self, script: str) -> Literal[True] | MoonrakerError | None: ...
        def gcode_commands(self) -> set[str] | None: ...
        def console_lines(self) -> list[tuple[float, str]] | None: ...


# The unit refuses ACEK_RESTORE_STOCK while the dryer heats or a lane moves.  The ace2k host
# module then raises "ace2k: refused — a subsystem is busy (heating or moving)".  With several
# units ([ace2k <name>]) the text starts with the unit's name: "ace2k a: refused — …".  Only an
# error that starts this way is a refusal.
# A unit's name may hold letters, digits, "_" and "-", as NAME_RE in ace2k_units.py allows.
REFUSAL_RE = re.compile(r"ace2k(?: [A-Za-z0-9_-]+)?: refused")
# Every error the module raises starts with "ace2k: " (or "ace2k a: ").  An error without it
# comes from Moonraker or Klipper itself: not connected, not ready, an unknown command, or a
# proxy.  Then the module never ran.  The tool reads this from the message, because the message
# is all it sees.  The HTTP status does not help: it is 400 both for a G-code error and for
# Klipper's "not ready".
MODULE_RE = re.compile(r"ace2k(?: [A-Za-z0-9_-]+)?: ")
# The restore command, and its old name on a host module from before the ACEK_ prefix (up to
# v0.12.0-rc.1).  The tool sends the name that Klipper's command list holds.  The list is read
# first, because an unknown command is not an error in Klipper: Klipper prints an info line, and
# the script succeeds.
RESTORE_COMMAND = "ACEK_RESTORE_STOCK"
RESTORE_COMMAND_OLD = "ACE_RESTORE_STOCK"
# Klipper's answer to a command that no module registered (klippy/gcode.py, cmd_default).  It is
# an info line on the console.  The tool looks for it only when it could not read the command
# list.
UNKNOWN_COMMAND = 'Unknown command:"{}"'


@dataclass(frozen=True)
class MoonrakerError:
    """Moonraker's error answer to a request, with its message.

    For a G-code script, the message is the G-code error.
    """

    message: str

    @property
    def first_line(self) -> str:
        """The line Klipper prints on the console as `!! <first line>`."""
        lines = self.message.strip().split("\n")
        return lines[0].strip()

    @property
    def is_refusal(self) -> bool:
        """Tell whether the unit refused the command."""
        return REFUSAL_RE.match(self.first_line) is not None

    @property
    def ran(self) -> bool:
        """Tell whether the ace2k module ran and raised the error.

        Examples are a refusal, or an answer from the unit that never came.
        """
        return MODULE_RE.match(self.first_line) is not None


def http_error_message(e: urllib.error.HTTPError) -> str:
    """Return the message of Moonraker's error answer ({"error": {"message": ...}}).

    When the answer holds no message, it returns the HTTP status.
    """
    try:
        error = json.loads(e.read()).get("error")
    except (OSError, ValueError, AttributeError):
        error = None
    if isinstance(error, dict):
        error = error.get("message")
    return error if isinstance(error, str) and error else f"HTTP {e.code}"


# A 2xx answer whose body the tool cannot read.  The request still ran.
ANSWERED = object()


def as_dict(value: object) -> Json:
    """Return value when it is a JSON object, else an empty object.

    So a part of Moonraker's answer without the expected shape counts as missing.
    """
    return cast("Json", value) if isinstance(value, dict) else {}


class Moonraker:
    """Moonraker's HTTP interface, as the flasher uses it."""

    def __init__(self, url: str, opener: Callable[..., Any] = urllib.request.urlopen) -> None:
        self.url, self._open = url.rstrip("/"), opener

    def _call(self, path: str, post: bool = False, errors: bool = False) -> object:
        """Return the result of the request.

        By default it returns None when Moonraker does not answer, and also for an error answer.

        With errors=True, an error answer is a MoonrakerError that carries its message.  A 2xx
        answer without a readable result is ANSWERED.  Only a failed connection returns None.
        """
        req = urllib.request.Request(self.url + path, data=b"" if post else None)
        try:
            with self._open(req, timeout=10) as r:
                body = r.read()
        except urllib.error.HTTPError as e:
            with e:  # the error holds a response body; close it once read
                return MoonrakerError(http_error_message(e)) if errors else None
        except (urllib.error.URLError, OSError, ValueError):
            return None
        try:
            result = json.loads(body).get("result")
        except (ValueError, AttributeError):
            result = None
        return ANSWERED if errors and result is None else result

    def state(self) -> str | None:
        """Return Klipper's state as Moonraker sees it, or None when Moonraker does not answer.

        The state is one of ready, startup, shutdown, error and disconnected.
        """
        state = as_dict(self._call("/server/info")).get("klippy_state")
        return state if isinstance(state, str) else None

    def objects(self) -> Json | None:
        """Return the status dict, or None.

        It returns None when Moonraker does not answer the query, or answers something that is
        not a status.

        Units with a name in the config ([ace2k <name>]) are queried in a second request.
        Klipper's config lists the section name in lower case, but the object keeps the name as
        written ("ace2k A").  So the object names come from Moonraker's list of objects.
        """
        res = self._call("/printer/objects/query?print_stats&ace2k&configfile=settings")
        status = res.get("status", {}) if isinstance(res, dict) else None
        if not isinstance(status, dict):
            return None
        named = unit_sections(config_settings(status)) - {"ace2k"}
        if named:
            listed = self._call("/printer/objects/list")
            listed = listed.get("objects") if isinstance(listed, dict) else None
            if isinstance(listed, list):
                objects = sorted(o for o in listed if isinstance(o, str) and o.lower() in named)
            else:  # no list: use the config's lower-case names, right for a lower-case section
                objects = sorted(named)
            more: object = None
            if objects:
                query = "&".join(urllib.parse.quote(o) for o in objects)
                more = self._call("/printer/objects/query?" + query)
            if isinstance(more, dict) and isinstance(more.get("status"), dict):
                status.update(more["status"])
        return status

    def gcode(self, script: str) -> Literal[True] | MoonrakerError | None:
        """Run a G-code script.

        It returns True when the script ran.  It returns a MoonrakerError when Moonraker
        answered with an error; a G-code error comes back this way, with its message.  It
        returns None when the request got no answer.  Then the script may or may not have
        reached Klipper.
        """
        res = self._call(
            "/printer/gcode/script?script=" + urllib.parse.quote(script), post=True, errors=True
        )
        return res if res is None or isinstance(res, MoonrakerError) else True

    def gcode_commands(self) -> set[str] | None:
        """Return the commands that Klipper lists with a description, or None.

        The list comes from /printer/gcode/help, as {CMD: help}.  It returns None when Moonraker
        does not answer, or answers something else.
        """
        res = self._call("/printer/gcode/help")
        return set(res) if isinstance(res, dict) else None

    def console_lines(self) -> list[tuple[float, str]] | None:
        """Return the newest console lines as (time, message), or None.

        It returns None when Moonraker does not answer, or answers something that is not a list
        of lines.  A line without a numeric time or a text message is skipped.
        """
        res = self._call("/server/gcode_store?count=20")
        store = res.get("gcode_store", []) if isinstance(res, dict) else None
        if not isinstance(store, list):
            return None
        lines: list[tuple[float, str]] = []
        for entry in store:
            e = as_dict(entry)
            t, m = e.get("time", 0.0), e.get("message", "")
            if e and isinstance(t, (int, float)) and not isinstance(t, bool) and isinstance(m, str):
                lines.append((float(t), m))
        return lines


class RestoreRequest:
    """One restore command sent through Moonraker, and what Klipper says after it.

    The console is tied to the request by time. Only lines newer than the mark count, and the mark
    is the newest line before the command was sent. With no console store to mark, the console is
    not read at all.
    """

    def __init__(self, mr: MoonrakerProto, name: str, suffix: str, checked: bool) -> None:
        """Prepare the request.  Nothing is sent yet.

        mr is the Moonraker the command goes through.  The command is name followed by suffix,
        which is " UNIT=<name>" or "".  checked is False when Klipper's command list could not
        be read, so the name was not checked against it.
        """
        self.mr = mr
        self.name = name
        self.suffix = suffix
        self.unlisted = ""  # the note added to a refusal when the name was not checked
        if not checked:
            self.unlisted = (
                f"  Klipper's command list could not be read, so {name} was sent"
                f" unchecked; a host module up to v0.12.0-rc.1 names it {RESTORE_COMMAND_OLD}."
            )
        self.mark: float | None = None
        self.answer: object = None

    def mark_console(self) -> None:
        """Mark the newest console line.  With no console store, there is no mark.

        A second mark comes after the fall back to the old name.  If the console is empty then,
        the earlier mark stays.
        """
        store = self.mr.console_lines()
        default = float("-inf") if self.mark is None else self.mark
        self.mark = max((t for t, _ in store), default=default) if store is not None else None

    def send(self) -> None:
        """Send the command.  The answer is True, a MoonrakerError, or None for no answer."""
        self.answer = self.mr.gcode(self.name + self.suffix)

    def fall_back_to_old_name(self) -> None:
        """Send the command again under its old name, with a new console mark."""
        self.name = RESTORE_COMMAND_OLD
        self.unlisted = (
            f"  Klipper's command list could not be read and {RESTORE_COMMAND} was"
            f" unknown, so {self.name} was sent."
        )
        self.mark_console()
        self.send()

    @property
    def error(self) -> MoonrakerError | None:
        """Moonraker's error answer to the command, or None."""
        return self.answer if isinstance(self.answer, MoonrakerError) else None

    def left_ready(self) -> bool:
        """Tell whether Klipper answers and its state is no longer ready."""
        state = self.mr.state()
        return state is not None and state != "ready"

    def unknown(self) -> bool:
        """Tell whether Klipper printed its unknown-command line for the name sent, after the
        mark."""
        text = UNKNOWN_COMMAND.format(self.name)
        mark = self.mark
        assert mark is not None  # called only after a console mark was taken
        return any(t > mark and text in m for t, m in self.mr.console_lines() or [])

    def new_refusals(self) -> list[str]:
        """Return the refusals printed on the console after the mark.  Without a mark, none."""
        # The console counts only when the answer cannot decide.  That is a script that ran
        # without error, because an older ace2k only printed its refusal, or no answer at all.
        if self.mark is None:
            return []
        lines = self.mr.console_lines() or []
        return [
            m for t, m in lines if t > self.mark and m.startswith("!! ") and REFUSAL_RE.match(m[3:])
        ]

    def refused(self, line: str) -> RefusedError:
        """Return the error to raise when Klipper refused the command with line."""
        return RefusedError(
            f"{self.name} was refused: {line} — Klipper still reports ready, so "
            "nothing was stopped.  If all four LEDs strobe fast, the unit did reset: run the "
            "tool again (it identifies recovery)."
        )


@dataclass
class KlipperView:
    """What Klipper reports about the unit and about the printer."""

    state: str | None  # None: Moonraker does not answer
    printing: bool | None  # None: unknown, because the print query failed while Klipper is ready
    ace2k_configured: bool
    ace2k_port: str | None
    ace2k_version: str | None
    ace2k_unit: str | None = None  # the unit's name ([ace2k <name>]); None for [ace2k]
    # Several units and none chosen: each unit's name and its port
    ace2k_units: dict[str | None, str | None] = field(default_factory=dict)

    @classmethod
    def no_unit(cls, state: str | None, printing: bool | None) -> KlipperView:
        """Return a view with no unit.

        Use it when Klipper does not answer, or when its config has no unit section.
        """
        return cls(
            state=state,
            printing=printing,
            ace2k_configured=False,
            ace2k_port=None,
            ace2k_version=None,
        )

    def configured_on(self, port: str) -> bool:
        """Tell whether Klipper's config has the unit on port (the same device, links followed)."""
        return self.ace2k_configured and same_device(self.ace2k_port, port)


def config_settings(status: Json) -> Json:
    """Return the config's settings in a status dict.  Return {} when they are missing or are
    not an object."""
    return as_dict(as_dict(status.get("configfile")).get("settings"))


def unit_sections(settings: Json) -> set[str]:
    """Return the config's unit sections: [ace2k] and every [ace2k <name>].

    Sections such as [ace2k_feed] are not unit sections.  Klipper lists the section names in
    lower case.
    """
    return {s for s in settings if s == "ace2k" or s.startswith("ace2k ")}


def _unit_ports(settings: Json) -> dict[str, str | None]:
    """Map each unit section of the config to the serial port of its MCU.

    The port is None when the section names no MCU, or when that MCU has no serial.
    """
    units: dict[str, str | None] = {}
    for section in sorted(unit_sections(settings)):
        mcu = as_dict(settings[section]).get("mcu", "ace2k" if section == "ace2k" else None)
        serial = None
        if isinstance(mcu, str):  # the MCU's section is in lower case, the value as written
            name = mcu.strip().lower()
            serial = as_dict(settings.get("mcu" if name == "mcu" else f"mcu {name}")).get("serial")
        units[section] = serial if isinstance(serial, str) else None
    return units


def _choose_unit(units: dict[str, str | None], port: str | None) -> str | None:
    """Return the unit section to report.

    That is the only section, else the one whose port is port, else None.
    """
    if len(units) == 1:
        return next(iter(units))
    if port is None:
        return None
    return next((s for s, p in units.items() if same_device(p, port)), None)


def _short_name(section: str, default: str | None = None) -> str | None:
    """Return the name of an [ace2k <name>] section, or default for the [ace2k] section."""
    return section.split(" ", 1)[1] if " " in section else default


def _reported_version(status: Json, section: str) -> str | None:
    """Return the version that Klipper's object for the unit section reports, or None."""
    # The object's name keeps the config's spelling ("ace2k A").  The config lists it in lower
    # case.
    found: object = next((v for k, v in status.items() if k.lower() == section), {})
    version = as_dict(found).get("version")
    return version if isinstance(version, str) else None


def klipper_view(moonraker: MoonrakerProto, port: str | None = None) -> KlipperView:
    """Return what Klipper reports about the unit on port.

    With one unit section, the view is of that unit, whatever its port.  With several, it is of
    the unit whose MCU has port as its serial.  When none has, ace2k_units lists them all.
    """
    state = moonraker.state()
    if state is None:
        return KlipperView.no_unit(None, False)
    status = moonraker.objects()
    if status is None:
        # Not ready means not printing.  Ready with no answer means the print state is unknown.
        return KlipperView.no_unit(state, None if state == "ready" else False)
    settings = config_settings(status)
    printing = as_dict(status.get("print_stats")).get("state") in ("printing", "paused")
    units = _unit_ports(settings)
    if not units:
        return KlipperView.no_unit(state, printing)
    chosen = _choose_unit(units, port)
    if chosen is None:
        return KlipperView(
            state=state,
            printing=printing,
            ace2k_configured=True,
            ace2k_port=None,
            ace2k_version=None,
            ace2k_units={_short_name(s, s): p for s, p in units.items()},
        )
    return KlipperView(
        state=state,
        printing=printing,
        ace2k_configured=True,
        ace2k_port=units[chosen],
        ace2k_version=_reported_version(status, chosen),
        ace2k_unit=_short_name(chosen),
    )


def same_device(a: str | None, b: str | None) -> bool:
    """Tell whether the two paths name the same device, after following links."""
    return a is not None and b is not None and os.path.realpath(a) == os.path.realpath(b)


def _cmdline(d: Path) -> str:
    """Return the command line of the process whose /proc directory is d, or ""."""
    try:
        return (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _holds(fd: Path, target: str) -> bool:
    """Tell whether the file descriptor link fd points to target."""
    try:
        link = os.readlink(fd)
    except OSError:
        return False
    return link == target or (link.startswith("/") and os.path.realpath(link) == target)


def port_holders(port: str, proc: Path) -> list[tuple[int, str]]:
    """Return (pid, command line) of every process that holds the port open.

    A process whose fd directory cannot be read is skipped, unless it is Klipper.  For Klipper it
    is then unknown whether it holds the port.  The tool refuses, rather than open a port that
    Klipper may hold.
    """
    if not proc.is_dir():
        raise RefusedError(f"cannot see who holds {port}: no {proc} (run this on the Klipper host)")
    target = os.path.realpath(port)
    holders: list[tuple[int, str]] = []
    for d in proc.glob("[0-9]*"):
        try:
            fds = list((d / "fd").iterdir())
        except PermissionError:
            if is_klippy(_cmdline(d)):
                raise RefusedError(
                    f"cannot see which ports Klipper (pid {d.name}) holds: "
                    "run this as root or as Klipper's user"
                ) from None
            continue
        except OSError:  # the process has exited
            continue
        if any(_holds(fd, target) for fd in fds):
            holders.append((int(d.name), _cmdline(d)))
    return holders


def is_klippy(cmdline: str) -> bool:
    """Tell whether a command line is Klipper's."""
    return "klippy" in cmdline


# The name, under /dev/serial/by-id, of the converter in the unit's own cable
ADAPTER_GLOB = "usb-1a86_USB_Single_Serial_*"


def candidate_ports(by_id: Path) -> list[str]:
    """Return the ports of the converter in the unit's own cable.

    That converter is a CH343 named usb-1a86_USB_Single_Serial_* (docs/flashing.md).  A printer
    whose mainboard has a CH340 shows up as usb-1a86_USB2.0-Serial-*, and is not listed.
    """
    return sorted(str(p) for p in by_id.glob(ADAPTER_GLOB))


# A stop or start command of Klipper's service that runs longer than this is taken as hung.  It
# is ended, and counts as failed.  systemd waits 90 s (DefaultTimeoutStopSec) before it kills a
# service that ignores the stop, so the limit is above that.  A query of systemctl about the
# service answers much faster, so its limit is shorter.
SERVICE_TIMEOUT_S = 120.0
SERVICE_QUERY_TIMEOUT_S = 10.0


@dataclass
class Service:
    """The commands that stop and start Klipper's service."""

    stop: list[str]
    start: list[str]


def find_service(
    stop_cmd: str | None,
    start_cmd: str | None,
    which: Callable[[str], str | None],
    run: Callable[..., subprocess.CompletedProcess[Any]],
    initd: Path,
) -> Service | None:
    """Return how to stop and start Klipper's service on this host, or None when it is unknown.

    The commands passed with --klipper-stop and --klipper-start come first, and both must be
    given.  Then a klipper.service of systemd, then the init script at initd.  A systemctl that
    hangs or cannot run is refused, and nothing changes.
    """
    if stop_cmd or start_cmd:
        if not (stop_cmd and start_cmd):
            raise RefusedError("--klipper-stop and --klipper-start go together")
        return Service(shlex.split(stop_cmd), shlex.split(start_cmd))
    if which("systemctl"):
        cmd = ["systemctl", "cat", "klipper.service"]
        try:
            r = run(cmd, capture_output=True, timeout=SERVICE_QUERY_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise RefusedError(
                f"`{shlex.join(cmd)}` did not finish within {SERVICE_QUERY_TIMEOUT_S:.0f} s; "
                "nothing changed"
            ) from None
        except OSError as e:
            raise RefusedError(
                f"`{shlex.join(cmd)}` could not run ({e.strerror or e}); nothing changed"
            ) from None
        if r.returncode == 0:
            return Service(["systemctl", "stop", "klipper"], ["systemctl", "start", "klipper"])
    if initd.exists():
        return Service([str(initd), "stop"], [str(initd), "start"])
    return None
