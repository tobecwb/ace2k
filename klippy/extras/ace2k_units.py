"""Handles what spans all the units.

That is the names of the units, the ``UNIT=`` parameter of the commands, the label on the console,
and the one active lane per extruder when several lanes feed that extruder (this last part is used
by ``ace2k_follow.py`` and ``ace2k_ff.py``).

Every ``[ace2k]`` or ``[ace2k <name>]`` section is one unit (``ace2k.py``). The first unit to load
creates this object, the printer object ``ace2k_units``, from the sections that the config
declares. Then every unit adds itself.

``[ace2k]`` alone is one unit, as it always was. Several units must each have a name, made of
letters, digits, ``_`` and ``-``. Using both forms together is a config error. Two sections on one
``mcu:`` are an error too.

Every ``ACEK_*`` command that the modules of a unit register goes through here. It is registered
once with Klipper. It reads ``UNIT=`` and runs the handler of that unit.

With one unit, ``UNIT=`` may be left out. With several units, a command without it is refused.
The exceptions are ``ACEK_STOP``, ``ACEK_STATUS`` and ``ACEK_HEALTH`` when they get no parameter at
all. They run on every unit in turn. With any other parameter (``ACEK_STOP LANE=n``,
``ACEK_HEALTH CLEAR=1``) they are about one unit, so they need ``UNIT=``.

With several units, every console line that a unit prints names the unit. A leading ``ace2k:``
becomes ``ace2k <name>:``. This holds for the output of a command, for its error, and for the lines
printed outside a command (``say``). When a command runs on every unit, an error line without that
prefix gets the unit as a prefix. Klipper's own "Error on ..." is an example.

Other lines of a unit are spelled differently: "ace2k dryer: ...", its klippy.log lines, and the
first lines of ACEK_STATUS and ACEK_HEALTH. They are built from the start of the unit's own lines
(``head``, ``lead``, ``own``). So with several units, every line of a unit starts with
"ace2k <name>:" and names its lanes like "a:2".
"""

from __future__ import annotations

import logging
import re
from collections.abc import Iterable, Mapping, Sequence
from typing import TYPE_CHECKING, Callable, Protocol, TypeVar

try:
    from .ace2k_link import LANES, PAIR, FeedRefused
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_link import LANES, PAIR, FeedRefused  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_feed_motion import LaneMotion
    from .ace2k_feed_proto import Move
    from .ace2k_klipper_types import (
        Completion,
        Config,
        GCode,
        GCodeAttr,
        GCodeCommand,
        LaneStatus,
        Printer,
        Reactor,
    )

    T = TypeVar("T")
    Key = tuple[int, int]  # a lane across the units: (index of the unit in Units.units, lane)
    Handler = Callable[[GCodeCommand], None]  # the handler of a command in one unit

    class FeedForwardProto(Protocol):
        """The feed-forward (``ace2k_ff.FeedForward``), as the coordinator reads lane extruders."""

        def refresh_map(self) -> None: ...
        def extruder_of(self, lane: int) -> str | None: ...
        def forget(self, extruder_name: str) -> None: ...

    class LaneFollowProto(Protocol):
        """``ace2k_follow.LaneFollow``: the speed a host module asked for (None: assist_speed)."""

        @property
        def speed(self) -> float | None: ...

    class FollowDefaultProto(Protocol):
        """The default follow (``ace2k_follow.FollowDefault``), as ACEK_LANE_ACTIVE arms a lane."""

        @property
        def enabled(self) -> bool: ...
        @property
        def lanes(self) -> Sequence[LaneFollowProto]: ...
        def armed_by_command(self, lane: int, move: Move) -> None: ...

    class FeedProto(Protocol):
        """The feed (``ace2k_feed.Ace2kFeed``), as the coordinator stops, arms and asks lanes."""

        @property
        def ff(self) -> FeedForwardProto: ...
        @property
        def follow_auto(self) -> FollowDefaultProto: ...
        @property
        def reactor(self) -> Reactor: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...
        @property
        def motion(self) -> list[LaneMotion]: ...
        @property
        def present(self) -> bool: ...
        @property
        def has_follow(self) -> bool: ...
        @property
        def assist_speed_mm_s(self) -> float: ...
        def in_follow(self, index: int) -> bool: ...
        def follow_running(self, index: int) -> bool: ...
        def retry_busy(self, send: Callable[[], T], lane: int) -> T: ...
        def start_follow_as(self, owner: str, lane_index: int, speed_mm_s: float) -> Move: ...
        def start_move(
            self,
            lane_index: int,
            mode: str,
            length_mm: float,
            speed_mm_s: float,
            free_tip: bool = False,
        ) -> Move: ...
        def stop(self, lane_index: int, by: str = "host") -> None: ...

    class UnitProto(Protocol):
        """One unit (``ace2k.Ace2k``), as the coordinator knows it.

        Its feed exists once the unit has loaded. A unit still loading has none, so use getattr.
        """

        @property
        def mcu(self) -> object: ...
        @property
        def section(self) -> str: ...
        @property
        def name(self) -> str | None: ...
        @property
        def feed(self) -> FeedProto: ...


SECTION = "ace2k"
# The name of a unit is one token. UNIT=, the "a:3" on the console and the file name of the tag
# cache all take it as it is.
NAME_RE = re.compile(r"[A-Za-z0-9_-]+")
# On a printer with several units, these commands run on every unit when they get no UNIT= and no
# other parameter. With any parameter (LANE=, RUN=, CLEAR= and so on) they are refused like any
# other command.
ALL_UNITS = ("ACEK_STOP", "ACEK_STATUS", "ACEK_HEALTH")
# A process start can meet boards that are still stopped from an earlier shutdown.
# LEFTOVER_REASON is what such a board reports (an emergency stop, or the shutdown of every board
# when one unit was lost, as measured on the U1). LEFTOVER_CHECK_S is how long after the MCUs are
# identified every board is checked. By then each board has reported its stop (on the U1 they did
# within 0.1 s).
LEFTOVER_REASON = "Command request"
LEFTOVER_CHECK_S = 3.0
# Klipper passes the first shutdown of a start on as klippy:analyze_shutdown. Its older releases
# (the one on the U1) use klippy:notify_mcu_shutdown. This module listens to both, and the first
# one decides.
SHUTDOWN_EVENTS = ("klippy:analyze_shutdown", "klippy:notify_mcu_shutdown")
# A board can also be found stopped before it sends any shutdown notice. Then Klipper's connect
# reads the stop in the board's config answer, fails with the error that CONNECT_STOPPED_RE
# matches ("MCU error during connect"), and passes it on as MCU_ERROR_EVENT. On the U1 this was
# seen after the service had been stopped for about two minutes or more.
MCU_ERROR_EVENT = "klippy:notify_mcu_error"
CONNECT_STOPPED_RE = re.compile(r"Can not update MCU '([^']+)' config as it is shutdown")


def unit_name(section: str) -> str | None:
    """Return the unit a section declares: None for [ace2k], the lower-case name otherwise."""
    parts = section.split(None, 1)
    return parts[1].strip().lower() if len(parts) > 1 else None


def unit_names(config: Config) -> list[str | None]:
    """Return the units the config declares, in file order (see ``unit_name``).

    Sections of other modules with the same prefix, such as [ace2k_tag ...], are not units.
    """
    return [
        unit_name(section.get_name())
        for section in config.get_prefix_sections(SECTION)
        if section.get_name().split(None, 1)[0] == SECTION
    ]


def relabel(text: str, label: str) -> str:
    """Return `text` with the leading "ace2k:" of each line replaced by `label`.

    Other lines stay as they are. A line that is already labelled does not start with "ace2k:",
    so a second pass changes nothing.
    """
    if label == SECTION:
        return text
    prefix = SECTION + ":"
    return "\n".join(
        label + line[len(SECTION) :] if line.startswith(prefix) else line
        for line in str(text).split("\n")
    )


def labelled(text: object, label: str) -> str:
    """Return `text` with every line naming the unit.

    A leading "ace2k:" is relabelled. A labelled line stays as it is. Any other line, such as
    Klipper's own "Error on ...", gets the prefix "label: ". This is for the error of a unit in a
    run on every unit, where nothing else tells which unit it came from.
    """

    def one(line: str) -> str:
        if line.startswith(SECTION + ":"):
            return label + line[len(SECTION) :]
        if line.startswith(label + ":") or not line.strip():
            return line
        return f"{label}: {line}"

    return "\n".join(one(line) for line in str(text).split("\n"))


# The label at the start of a console line: "ace2k:", or "ace2k <name>:" for a unit (several units).
PREFIX_RE = re.compile(SECTION + r"(?: [A-Za-z0-9_-]+)?:\s*")


def unprefixed(text: object) -> str:
    """Return `text` without a leading "ace2k:" or "ace2k <name>:" and any space after it.

    This is for a reason that is quoted inside a line that already says it.
    """
    text = str(text)
    match = PREFIX_RE.match(text)
    return text[match.end() :] if match else text


class UnitError(Exception):
    """Raised by ``Units.resolve`` when a UNIT= or unit: names no unit (the reason, no subject)."""

    def __init__(self, reason: str, omitted: bool = False) -> None:
        super().__init__(reason)
        self.omitted = omitted  # no name was given, and there are several units


class UnitGcmd:
    """A G-code command as one unit sees it with several units. Its console lines name the unit."""

    def __init__(self, gcmd: GCodeCommand, label: str) -> None:
        self._gcmd = gcmd
        self._label = label
        self.error = gcmd.error

    def __getattr__(self, name: str) -> GCodeAttr:
        return getattr(self._gcmd, name)

    def respond_info(self, msg: str, *args: bool, **kwargs: bool) -> None:
        self._gcmd.respond_info(relabel(msg, self._label), *args, **kwargs)


class Units:
    def __init__(self, printer: Printer, names: Iterable[str | None]) -> None:
        self.printer = printer
        self.names = list(names)
        if None in self.names and len(self.names) > 1:
            raise printer.config_error(
                "ace2k: [ace2k] together with [ace2k <name>]: with several units, name every one"
                " ([ace2k <name>])"
            )
        for name in self.names:
            if name is not None and not NAME_RE.fullmatch(name):
                raise printer.config_error(
                    f"ace2k: unit name '{name}' may only hold letters, digits, '_' and '-'"
                )
            if self.names.count(name) > 1:
                raise printer.config_error(f"ace2k: two sections name unit {name}")
        self.units: list[UnitProto] = []
        self.index: dict[UnitProto, int] = {}  # unit -> its place in units (key[0] of a lane)
        self.commands: dict[str, dict[UnitProto, Handler]] = {}  # command name -> {unit: handler}
        self.gcode: GCode = printer.lookup_object("gcode")
        # The shared extruders across every unit. They are built again each time the lane map
        # changes (see map_changed: a feed attached, or a lane remapped), and reused until then.
        self._groups: dict[str, list[Key]] = {}
        # Maps an extruder to the (unit index, lane) chosen with ACEK_LANE_ACTIVE. In a group of
        # lanes that name one extruder, on any unit, only the chosen lane is followed. No lane is
        # chosen until the operator does it, and the choice is not saved.
        self.active: dict[str, Key] = {}
        # The (unit index, lane) that ACEK_LANE_ACTIVE is arming, before it records the choice. It
        # is the one follow start allowed on a lane of a shared extruder that is not yet the active
        # lane.
        self.choosing: Key | None = None
        self.group_said: set[str] = set()  # extruders whose "choose one" line was printed
        # One FIRMWARE_RESTART happens when a process start meets boards that are still stopped
        # from an earlier shutdown (see _on_first_shutdown, _on_mcu_error and _check_leftover).
        # It never happens at a start that a FIRMWARE_RESTART or a RESTART made.
        self.ready = False
        self.first_shutdown_seen = False
        self.leftover_armed = False  # the check of every board is armed (once per process)
        self.leftover_vetoed = False  # the first shutdown was a fault of this start
        self.found_at_connect: str | None = None  # the unit's MCU that the connect found stopped
        printer.register_event_handler("klippy:ready", self._on_ready)
        for event in SHUTDOWN_EVENTS:
            printer.register_event_handler(event, self._on_first_shutdown)
        printer.register_event_handler(MCU_ERROR_EVENT, self._on_mcu_error)
        # The "choose one" lines may have changed, because the map changed or the follow of a unit
        # became ready. The next tick walks the groups (see say_groups). No tick does it before.
        self.groups_dirty = True
        self.shared_said: set[str] = set()  # extruders whose followers on several units were said

    # --- the units -----------------------------------------------------------------------

    def add(self, unit: UnitProto) -> None:
        for other in self.units:
            if other.mcu is unit.mcu:
                raise self.printer.config_error(
                    f"{unit.section}: its mcu is {other.section}'s too: one unit per MCU"
                )
        self.index[unit] = len(self.units)
        self.units.append(unit)
        # Its lanes join the shared extruders once its feed is attached (see Ace2k, map_changed).

    def several(self) -> bool:
        return len(self.names) > 1

    def label(self, unit: UnitProto) -> str:
        """Return the console prefix: "ace2k" with one unit, the section name with several."""
        return unit.section if self.several() else SECTION

    def lead(self, unit: UnitProto, words: str) -> str:
        """Return a line of the unit that starts with `words` after the module name.

        It is "ace2k <words>" with one unit, and "ace2k <name>: <words>" with several.
        """
        return f"{self.label(unit)}: {words}" if self.several() else f"{SECTION} {words}"

    def head(self, unit: UnitProto, part: str | None = None) -> str:
        """Return the start of a line of the unit, up to its colon.

        With one unit it is "ace2k:", or "ace2k dryer:" with part "dryer". With several units it is
        "ace2k <name>:", or "ace2k <name>: dryer:".
        """
        return self.lead(unit, part) + ":" if part else self.label(unit) + ":"

    def own(self, unit: UnitProto, text: str) -> str:
        """Return a line that names no module, such as "health: OK" of ACEK_HEALTH.

        With several units, the line follows "ace2k <name>: ".
        """
        return f"{self.label(unit)}: {text}" if self.several() else text

    def lane_text(self, unit: UnitProto, lane: int) -> str:
        """Return a lane as the console names it: "a:3" with several units, "3" with one."""
        return f"{unit.name}:{lane + 1}" if self.several() else str(lane + 1)

    def lane_args(self, unit: UnitProto, lane: int) -> str:
        """Return the lane arguments of a printed command, "LANE=3 UNIT=b" or "LANE=3"."""
        args = f"LANE={lane + 1}"
        return f"{args} UNIT={unit.name}" if self.several() else args

    @staticmethod
    def given(name: object) -> str | None:
        """Return `name` as given by a UNIT= or unit:, or None if it is omitted, empty or blank."""
        if name is None or not str(name).strip():
            return None
        return str(name).strip()

    def find(self, name: object) -> UnitProto | None:
        """Return the unit called `name` (any case), or None if there is none.

        An omitted, empty or blank name asks for the only unit, and finds nothing with several.
        """
        wanted = self.given(name)
        if wanted is None:
            return self.units[0] if len(self.units) == 1 else None
        wanted = wanted.lower()
        return next((unit for unit in self.units if unit.name == wanted), None)

    def resolve(self, name: object, key: str = "UNIT=") -> UnitProto | None:
        """Return the unit that a UNIT= (key "UNIT=") or a sensor option unit: (key "unit:") names.

        The name is checked against the units that the config declares. The result is the unit, or
        None for a declared unit that is not loaded yet.

        A name that matches no unit raises UnitError. The reason is the same for both keys, and the
        caller places it after its own subject. The reasons are:
        - "needs UNIT=<a|b>: this printer has several units" (name omitted, empty or blank);
        - "no unit x; units: a, b";
        - "no unit x; this printer has one unit: omit UNIT=" (any name for the unnamed [ace2k]).
        """
        wanted = self.given(name)
        if wanted is None:
            if len(self.names) > 1:
                raise UnitError(
                    f"needs {key}<{self.declared_text('|')}>: this printer has several units",
                    omitted=True,
                )
            return self.units[0] if self.units else None
        if self.names == [None]:
            # The one unnamed [ace2k] has no name to give, not even "ace2k".
            raise UnitError(f"no unit {wanted}; this printer has one unit: omit {key}")
        if wanted.lower() not in self.names:
            raise UnitError(f"no unit {wanted}; units: {self.declared_text(', ')}")
        return self.find(wanted)

    def declared_text(self, sep: str) -> str:
        """Return the declared names, joined by `sep`. Only used when every unit is named."""
        return sep.join(name or SECTION for name in self.names)

    def unit_for(self, gcmd: GCodeCommand) -> UnitProto | None:
        """Return the unit named by UNIT=, or the only unit. Otherwise raise a G-code error."""
        try:
            return self.resolve(gcmd.get("UNIT", None))
        except UnitError as e:
            subject = f"ace2k: {gcmd.get_command()} " if e.omitted else "ace2k: "
            raise gcmd.error(subject + str(e)) from None

    def say(self, unit: UnitProto, line: str) -> None:
        """Print a console line for the unit outside a command. The line is also logged."""
        try:
            self.gcode.respond_info(relabel(line, self.label(unit)))
        except Exception:
            logging.info("%s", relabel(line, self.label(unit)))

    # --- the commands --------------------------------------------------------------------

    def register_command(
        self, unit: UnitProto, name: str, func: Handler, desc: str | None = None
    ) -> None:
        handlers = self.commands.get(name)
        if handlers is None:
            handlers = self.commands[name] = {}

            def dispatch(gcmd: GCodeCommand, name: str = name) -> None:
                self._dispatch(name, gcmd)

            self.gcode.register_command(name, dispatch, desc=desc)
        handlers[unit] = func

    def _dispatch(self, name: str, gcmd: GCodeCommand) -> None:
        handlers = self.commands[name]
        if name in ALL_UNITS and self.several() and not self._parameters(gcmd):
            self._run_all(name, gcmd, handlers)
            return
        unit = self.unit_for(gcmd)  # a command without UNIT= is refused here
        # Every unit has loaded before Klipper runs a command, so the name finds a loaded unit.
        assert unit is not None
        func = handlers.get(unit)
        if func is None:
            raise gcmd.error(f"{self.label(unit)}: {name} is not available on this unit")
        self._run(unit, func, gcmd)

    def _parameters(self, gcmd: GCodeCommand) -> dict[str, str]:
        """Return the parameters of the command. An empty or blank UNIT= counts as omitted."""
        params = dict(gcmd.get_command_parameters())
        if "UNIT" in params and self.given(params["UNIT"]) is None:
            del params["UNIT"]
        return params

    def _run(self, unit: UnitProto, func: Handler, gcmd: GCodeCommand) -> None:
        if not self.several():
            func(gcmd)
            return
        label = self.label(unit)
        try:
            func(UnitGcmd(gcmd, label))
        except gcmd.error as e:
            raise gcmd.error(relabel(str(e), label)) from None

    def _run_all(
        self, name: str, gcmd: GCodeCommand, handlers: Mapping[UnitProto, Handler]
    ) -> None:
        # Every unit runs even if another unit raised an error, such as a link that is down while
        # ACEK_STOP sends. The errors are reported only after each unit has had its turn.
        errors: list[str] = []
        internal: list[Exception] = []
        for unit in self.units:
            func = handlers.get(unit)
            if func is None:
                continue
            try:
                self._run(unit, func, gcmd)
            except gcmd.error as e:
                # Every line names its unit, also the lines without "ace2k:" (a parse error).
                errors.append(labelled(e, self.label(unit)))
            except Exception as e:
                if self.several():
                    logging.exception("%s: %s failed", self.label(unit), name)
                else:
                    logging.exception("ace2k: %s on %s failed", name, unit.section)
                internal.append(e)
        if not errors and not internal:
            return
        # Klipper marks only the first line of an error with "!! ". So the error of each unit
        # except the last is printed as one here, and the last one is the error of the command.
        # An internal error, which is not a G-code error, is raised as it came, after the
        # refusals are printed. Klipper shuts down on it, as it does when the command names one
        # unit.
        told = errors if internal else errors[:-1]
        for error in told:
            for line in error.strip().split("\n"):
                logging.warning("%s", line)
                gcmd.respond_raw("!! " + line.strip())
        if internal:
            raise internal[0]
        raise gcmd.error(errors[-1])

    # --- one active lane per extruder ------------------------------------------------------

    def map_changed(self) -> None:
        """Build the map of extruders again, because the extruder of a lane may have changed.

        This happens when the feed of a unit loads or when set_lane_extruder is called.
        A group whose lanes changed is a new group. Its "choose one" line is said again, and its
        active lane is kept only while it is still in the group. A lane that is mapped out and back
        is not active again until ACEK_LANE_ACTIVE. A group that did not change keeps both.
        """
        lanes: dict[str, list[Key]] = {}
        for i, unit in enumerate(self.units):
            if getattr(unit, "feed", None) is None:
                continue  # a unit that is still loading has no feed yet
            unit.feed.ff.refresh_map()
            for lane in range(LANES):
                name = unit.feed.ff.extruder_of(lane)
                if name:
                    lanes.setdefault(name, []).append((i, lane))
        groups = {n: sorted(m) for n, m in lanes.items() if len(m) > 1}
        for name in set(groups) | set(self._groups):
            keys = groups.get(name)
            if keys == self._groups.get(name):
                continue
            self.group_said.discard(name)
            if self.active.get(name) not in (keys or ()):
                gone = self.active.pop(name, None)
                if gone is not None and self._mapped_elsewhere(gone, name):
                    # The active lane left the extruder. What was recorded for it does not apply
                    # to the lane that stays on the extruder (alone, it follows by default).
                    self._forget_ff(name)
        self._groups = groups
        self.groups_dirty = True

    def note_ready(self) -> None:
        """Note that the follow of a unit is ready, so its groups may be said at the next tick."""
        self.groups_dirty = True

    def groups(self) -> dict[str, list[Key]]:
        """Return the extruders that several lanes name: {extruder: [(unit index, lane), ...]}."""
        return self._groups

    def key(self, unit: UnitProto, lane: int) -> Key:
        return (self.index[unit], lane)

    def member(self, key: Key) -> tuple[UnitProto, int]:
        return self.units[key[0]], key[1]

    def members_text(self, keys: Iterable[Key]) -> str:
        return ", ".join(self.lane_text(*self.member(k)) for k in keys)

    def lanes_phrase(self, keys: Sequence[Key]) -> str:
        """Return "lane a:1" for one lane, and "lanes a:1, b:2" for several."""
        return f"lane{'s' if len(keys) > 1 else ''} {self.members_text(keys)}"

    def group_of(self, unit: UnitProto, lane: int) -> tuple[str, list[Key]] | tuple[None, None]:
        """Return the shared group of the lane as (extruder, [(unit index, lane), ...]).

        Return (None, None) if the lane is in no group.
        """
        name = unit.feed.ff.extruder_of(lane)
        keys = self.groups().get(name) if name else None
        return (name, keys) if name and keys else (None, None)

    def eligible(self, unit: UnitProto, lane: int) -> bool:
        """Tell whether the lane may be followed by default.

        It may if it has its own extruder or none, or is the lane chosen for its shared one.
        """
        name, keys = self.group_of(unit, lane)
        return name is None or self.active.get(name) == self.key(unit, lane)

    def shared_lanes(self, unit: UnitProto, lane: int) -> str | None:
        """Return the lanes that feed the extruder of the lane ("a:4, b:1"), if several share it.

        Return None if the lane has its own extruder, or none.
        """
        _name, keys = self.group_of(unit, lane)
        return self.members_text(keys) if keys else None

    def may_follow(self, unit: UnitProto, lane: int) -> bool:
        """Tell whether a follow may start on the lane.

        It may if the lane is eligible, or if ACEK_LANE_ACTIVE is arming it. Every follow start
        asks this: the default follow, ACEK_ASSIST DIR=BOTH and start_move of a host module.
        A host module also asks it before it arms.
        """
        return self.eligible(unit, lane) or self.choosing == self.key(unit, lane)

    def active_map(self) -> dict[str, str | None]:
        """Return the active lane of each extruder as text such as "b:3", or None if none."""
        out: dict[str, str | None] = {}
        for name, keys in self.groups().items():
            key = self.active.get(name)
            out[name] = self.lane_text(*self.member(key)) if key in keys else None
        return out

    def unit_active_map(self, unit: UnitProto) -> dict[str, int | str | None]:
        """Return the groups that a lane of the unit is in, for the status of the unit.

        It maps an extruder to its active lane: the lane number in this unit, the text of a lane
        in another unit ("b:3"), or None if no lane is chosen.
        """
        out: dict[str, int | str | None] = {}
        mine = self.index[unit]
        for name, keys in self.groups().items():
            if not any(i == mine for i, _ in keys):
                continue
            key = self.active.get(name)
            if key not in keys:
                out[name] = None
            elif key[0] == mine:
                out[name] = key[1] + 1
            else:
                out[name] = self.lane_text(*self.member(key))
        return out

    def unit_groups(self, unit: UnitProto) -> dict[str, list[int]]:
        """Return the lanes of the unit in a shared group, as {extruder: [lane, ...]}."""
        mine = self.index[unit]
        return {
            name: [lane for i, lane in keys if i == mine]
            for name, keys in self.groups().items()
            if any(i == mine for i, _ in keys)
        }

    def _mapped_elsewhere(self, key: Key, name: str) -> bool:
        """Tell whether the lane `key` is no longer on extruder `name`, or its unit is gone."""
        i, lane = key
        if i >= len(self.units) or getattr(self.units[i], "feed", None) is None:
            return True
        return self.units[i].feed.ff.extruder_of(lane) != name

    def _forget_ff(self, name: str) -> None:
        """Make every feed-forward forget what it recorded for extruder `name`.

        Call it when the active lane of the extruder changes (see ``FeedForward.forget``). Then no
        lane is dosed or taken back for the filament of another lane.
        """
        for other in self.units:
            feed: FeedProto | None = getattr(other, "feed", None)
            if feed is not None:
                feed.ff.forget(name)

    def _forget_if_changed(self, name: str | None, key: Key) -> None:
        """Make the feed-forward forget extruder `name` if `key` is a new choice.

        Call this before `key` becomes the active lane of `name`, after the other followers ended.
        If `key` is already the choice, nothing is forgotten.
        """
        if name and self.active.get(name) != key:
            self._forget_ff(name)

    def choose(self, unit: UnitProto, lane: int) -> None:
        """Make the lane the active lane of its extruder group, if it is in one."""
        name, _keys = self.group_of(unit, lane)
        if name is not None:
            self.active[name] = self.key(unit, lane)

    def followers_elsewhere(self, name: str, unit: UnitProto | None) -> list[Key]:
        """Return the keys (unit index, lane) of lanes of other units that follow extruder `name`.

        With `unit` None, lanes of every unit count. A follow whose final event has come does not
        count, even though the last report of that unit still says "following".
        """
        return [
            (i, lane)
            for i, other in enumerate(self.units)
            if other is not unit and getattr(other, "feed", None) is not None
            for lane in range(LANES)
            if other.feed.ff.extruder_of(lane) == name and other.feed.in_follow(lane)
        ]

    def say_shared(self, name: str, keys: Iterable[Key]) -> None:
        """Say once per conflict that lanes of more than one unit follow one extruder.

        If all followers are on one unit, that unit says it in its own line. The line is labelled
        with the first unit that has a follower, so it names a unit in the conflict.
        """
        if name in self.shared_said:
            return
        self.shared_said.add(name)
        ordered = sorted(keys)
        which = "both" if len(ordered) == PAIR else "all"
        self.say(
            self.member(ordered[0])[0],
            f"ace2k: extruder {name}: lanes {self.members_text(ordered)} {which} in follow"
            " — no feed-forward",
        )

    def clear_shared(self, name: str, followers: int) -> None:
        """End the conflict on extruder `name` when at most one follower is left on all units.

        `followers` is how many lanes of every unit follow it, as the caller counted. The next
        conflict is then said again. Followers of one unit leaving ends nothing while lanes of
        other units still conflict.
        """
        if followers <= 1:
            self.shared_said.discard(name)

    def follows_by_default(self, unit: UnitProto) -> bool:
        """Tell whether the unit arms its lanes by itself.

        It does if it has a feed with the follow, and follow_default is on. That is what its own
        check needs to run.
        """
        feed: FeedProto | None = getattr(unit, "feed", None)
        return feed is not None and feed.present and feed.has_follow and feed.follow_auto.enabled

    def say_groups(self) -> None:
        """Say once, for each shared extruder with no lane chosen, which lanes share it.

        Only a group with a member on a unit that follows by default is said. The follow would never
        arm any other group, so it has nothing to choose. The group is said once a member on such a
        unit joins, after its feed has loaded. The walk runs only when something that it reads may
        have changed (``groups_dirty``).
        """
        if not self.groups_dirty:
            return
        self.groups_dirty = False
        self._walk_groups()

    def _walk_groups(self) -> None:
        for name, keys in self.groups().items():
            if self.active.get(name) in keys or name in self.group_said:
                continue
            if not any(self.follows_by_default(self.units[i]) for i in {i for i, _ in keys}):
                continue
            self.group_said.add(name)
            self.say(
                self.member(keys[0])[0],
                f"ace2k: lanes {self.members_text(keys)} share {name}: choose one with"
                " ACEK_LANE_ACTIVE",
            )

    def lane_active(
        self, unit: UnitProto, index: int, gcmd: GCodeCommand, pause_wait_s: float
    ) -> None:
        """Run ACEK_LANE_ACTIVE after the unit has checked the lane.

        First the other followers of the group, on any unit, are stopped as ACEK_STOP would, and
        their ends are seen. Then this lane is armed. Then the choice is recorded.
        """
        feed = unit.feed
        n = self.lane_text(unit, index)
        name, _keys = self.group_of(unit, index)
        before = self.active_map().get(name) if name else None
        kept = f"; {name}'s active lane stays {f'lane {before}' if before else 'none'}"
        # Stop the others and see their ends before anything else. Two lanes of one extruder in
        # the follow would feed neither.
        stopped, late = self._stop_others(unit, index, pause_wait_s)
        if late:
            raise gcmd.error(
                f"ace2k: lane {n} not made active: {self.lanes_phrase(late)}"
                f" stopped but did not end within {pause_wait_s:g} s;"
                f" nothing armed{kept}"
            )
        me = self.key(unit, index)
        # Do this after the others ended and before the arm. The new lane must not be dosed for the
        # moves of the lane that leaves. If the arm is refused later, the choice stays as it was,
        # but the lane was stopped with the others. The feed-forward drops the account of an
        # extruder with no follower anyway.
        self._forget_if_changed(name, me)
        auto = feed.follow_auto
        if not feed.follow_running(index):
            speed = auto.lanes[index].speed or feed.assist_speed_mm_s
            self.choosing = me  # for this arm only; the choice is recorded once it is in
            try:
                move = feed.retry_busy(
                    lambda: feed.start_follow_as("ACEK_LANE_ACTIVE", index, speed), index
                )
            except (FeedRefused, RuntimeError) as e:
                why = unprefixed(e)
                off = f"; lanes {self.members_text(stopped)} stopped" if stopped else ""
                raise gcmd.error(f"ace2k: lane {n} not made active — {why}{off}{kept}") from e
            finally:
                self.choosing = None
            auto.armed_by_command(index, move)
        self.choose(unit, index)  # only now, when the others ended and this lane follows
        line = f"ace2k: lane {n} active on {name or feed.ff.extruder_of(index) or 'no extruder'}"
        if stopped:
            line += f"; {self.lanes_phrase(stopped)} off"
        gcmd.respond_info(line)

    def _stop_others(
        self, unit: UnitProto, index: int, pause_wait_s: float
    ) -> tuple[list[Key], list[Key]]:
        """Stop the other followers of the group of the lane, on any unit, and wait for them.

        Return the keys that were stopped and the keys that did not end in time.
        """
        _name, keys = self.group_of(unit, index)
        return self._stop_followers(keys or [], self.key(unit, index), pause_wait_s)

    def _stop_followers(
        self,
        keys: Iterable[Key],
        skip: Key | None,
        pause_wait_s: float,
        by: str = "command",
        assists: bool = True,
    ) -> tuple[list[Key], list[Key]]:
        """Stop the followers among `keys`, except `skip` (a key or None), and wait for their ends.

        With `assists`, lanes in a one-way assist are stopped too. The stops are made as ACEK_STOP
        would make them, and `by` labels them.

        Return the keys stopped and the keys that did not end in time. The waiters of a late end are
        taken back. An internal error in a stop is raised as it came.
        """
        stopped: list[Key] = []
        waits: list[tuple[Key, Completion]] = []
        for key in keys:
            if key == skip:
                continue
            ofeed = self.member(key)[0].feed
            lane = key[1]
            assisting = ofeed.lanes[lane]["mode"] in ("assisting", "assisting_back")
            if not (ofeed.follow_running(lane) or (assists and assisting)):
                continue
            waiter = ofeed.reactor.completion()
            ofeed.motion[lane].final_waiters.append(waiter)
            waits.append((key, waiter))
            try:
                ofeed.stop(lane, by=by)
            except Exception:
                # The stop refuses nothing, so whatever it raises is an internal error, such as the
                # link of that unit being down. Nothing is armed, the choice stays, and no waiter
                # of this call is left behind. The error is raised as it came, so Klipper shuts
                # down on it.
                self._unwait(waits)
                raise
            stopped.append(key)
        deadline = self.printer.get_reactor().monotonic() + pause_wait_s
        late = [key for key, waiter in waits if waiter.wait(deadline, None) is None]
        if late:
            self._unwait(waits)
        return stopped, late

    def hand_over(self, unit: UnitProto, index: int, pause_wait_s: float) -> None:
        """Let a host module choose the active lane of the shared extruder of the lane.

        This is part of the host interface (11). The other followers of the group are stopped and
        their ends are seen. Then the choice is recorded. Nothing is armed. A host module that
        moves the lane before it arms it uses this.

        A lane with its own extruder, or none, has nothing to choose. A follower that does not end
        in time raises RuntimeError and leaves the choice as it was. Followers already stopped
        stay stopped.
        """
        name, _keys = self.group_of(unit, index)
        if name is None:
            return
        _stopped, late = self._stop_others(unit, index, pause_wait_s)
        if late:
            raise RuntimeError(
                f"ace2k: lane {self.lane_text(unit, index)} not handed the head:"
                f" {self.lanes_phrase(late)} stopped but did"
                f" not end within {pause_wait_s:g} s; the others stay stopped, nothing chosen"
            )
        self._forget_if_changed(name, self.key(unit, index))
        self.choose(unit, index)

    # The bare marker below is for vulture, which does not accept the code form for methods.
    def release(self, extruder_name: str, pause_wait_s: float) -> None:  # noqa
        """Let a host module undo the choice of the active lane of a shared extruder.

        This is part of the host interface (13). After the call, no lane of the shared extruder is
        active. Every lane in its follow, the active one included, on any unit, is stopped
        (stop_by "host") and its end is seen. A one-way assist does not belong to the choice, so it
        is left running.

        The default follow arms none of the lanes until a new choice, and the "choose one" line of
        the group is said again. An extruder that no two lanes share has nothing to release.
        While ACEK_LANE_ACTIVE arms a lane of the group, the release is refused with RuntimeError.
        A follower that does not end within pause_wait_s raises RuntimeError and puts the choice
        back, unless another choice was made meanwhile. Followers already stopped stay stopped.
        """
        keys = self.groups().get(extruder_name)
        if not keys:
            return
        if self.choosing in keys:
            raise RuntimeError(
                f"ace2k: {extruder_name} not released: ACEK_LANE_ACTIVE is arming lane"
                f" {self.members_text([self.choosing])}; nothing done"
            )
        # The choice goes first. While the followers wind down, no lane of the group is eligible.
        # So neither the default follow nor the re-arm of a move starts a follow there meanwhile.
        before = self.active.pop(extruder_name, None)
        _stopped, late = self._stop_followers(keys, None, pause_wait_s, by="host", assists=False)
        if late:
            if (
                before is not None
                and extruder_name not in self.active
                and before in self.groups().get(extruder_name, ())
            ):
                self.active[extruder_name] = before
            raise RuntimeError(
                f"ace2k: {extruder_name} not released: {self.lanes_phrase(late)}"
                f" stopped but did not end within {pause_wait_s:g} s;"
                " the others stay stopped, the choice is kept"
            )
        # Do this only once every follower ended. While they wind down, the lane that leaves keeps
        # its own account. If a follower is late, the choice goes back with the account untouched.
        if before is not None and extruder_name not in self.active:
            self._forget_ff(extruder_name)
        self.group_said.discard(extruder_name)
        self.groups_dirty = True

    def _unwait(self, waits: Iterable[tuple[Key, Completion]]) -> None:
        """Take back the end waiters that ``_stop_followers`` registered, as (key, waiter) pairs."""
        for key, waiter in waits:
            ofeed = self.member(key)[0].feed
            if waiter in ofeed.motion[key[1]].final_waiters:
                ofeed.motion[key[1]].final_waiters.remove(waiter)

    # --- status --------------------------------------------------------------------------

    def get_status(self, _eventtime: float) -> dict[str, object]:
        return {"units": [unit.section for unit in self.units], "active_lanes": self.active_map()}

    # --- a board still stopped from an earlier shutdown ----------------------------------------

    def _on_ready(self) -> None:
        self.ready = True

    def _on_first_shutdown(self, _msg: str, details: Mapping[str, object] | None = None) -> None:
        """Arm the check for leftover boards on the first shutdown of this process.

        This applies at a process start (start_reason "startup"), when the service started or the
        printer was powered on, and Klipper is not ready yet. A board that reports LEFTOVER_REASON
        was already stopped when this start reached it. Then the check of every board is armed to
        run LEFTOVER_CHECK_S later (see ``_check_leftover``).

        Two other cases are a fault of this start, so nothing happens. One is a first shutdown that
        comes from the host itself, such as a lost link or a spontaneous restart. It has no "mcu"
        in details. The host then stops every board, and each board then reports LEFTOVER_REASON
        too, so the reason alone cannot tell this case apart. The other is a board that reports
        another reason.

        This never happens at a start that a FIRMWARE_RESTART or a RESTART made. It is one try. An
        emergency stop while the printer is up stays with the operator.
        """
        if self.first_shutdown_seen:
            return
        self.first_shutdown_seen = True
        details = details or {}
        if "mcu" not in details or details.get("reason") != LEFTOVER_REASON:
            self.leftover_vetoed = True
            return
        self._arm_leftover()

    def _on_mcu_error(self, _msg: str, details: Mapping[str, object] | None = None) -> None:
        """Arm the same check when the connect finds the MCU of a unit already stopped.

        The board sent no shutdown notice, so its reason is not known. At a process start it can
        only be a stop from before this start. The MCU of another section never arms the check.
        """
        found = CONNECT_STOPPED_RE.search(str((details or {}).get("error", "")))
        if found is None or found.group(1) not in self._unit_mcu_names():
            return
        self.found_at_connect = found.group(1)
        self._arm_leftover()

    def _unit_mcu_names(self) -> set[str]:
        """Return the names that Klipper's errors give the MCUs of the units ("ace2k_a")."""
        mine = [unit.mcu for unit in self.units]
        return {
            name.split(" ", 1)[-1]
            for name, mcu in self.printer.lookup_objects("mcu")
            if any(mcu is own for own in mine)
        }

    def _arm_leftover(self) -> None:
        """Arm the check of every board LEFTOVER_CHECK_S later, once, at a process start only."""
        if self.leftover_armed or self.leftover_vetoed or self.ready:
            return
        if self.printer.get_start_args().get("start_reason") != "startup":
            return
        self.leftover_armed = True
        reactor = self.printer.get_reactor()
        reactor.register_timer(self._check_leftover, reactor.monotonic() + LEFTOVER_CHECK_S)

    def _check_leftover(self, _eventtime: float) -> float:
        """Request one FIRMWARE_RESTART if this start stopped only on leftover boards.

        A leftover board is still stopped from an earlier shutdown. The restart is requested only
        if Klipper is not ready, at least one MCU is stopped, and every stopped MCU reports
        LEFTOVER_REASON. A board keeps such a stop while it has power. A unit has its own supply,
        so neither a restart of the service nor a power cycle of the printer clears the stop.

        If a stopped MCU has another reason (a fault), or its reason cannot be read, the operator
        decides. The result is logged either way. There is no second try. A unit that the connect
        found stopped (see _on_mcu_error) counts as a leftover board unless its own notice says
        otherwise. A first shutdown that was a fault of this start vetoes the restart.
        """
        never = self.printer.get_reactor().NEVER
        if self.ready or self.leftover_vetoed:
            return never
        stopped: list[tuple[str, object]] = []
        for name, mcu in self.printer.lookup_objects("mcu"):
            down, reason = mcu_stop(mcu)
            if down:
                stopped.append((name.split(" ", 1)[-1], reason))
        found = self.found_at_connect
        if found is not None and found not in (name for name, _ in stopped):
            stopped.append((found, LEFTOVER_REASON))
        if not stopped:
            return never
        names = ", ".join(name for name, _ in stopped)
        other = [f"'{name}' ({reason!r})" for name, reason in stopped if reason != LEFTOVER_REASON]
        if other:
            logging.warning(
                "ace2k: no automatic FIRMWARE_RESTART: MCU %s stopped for another reason than"
                " an earlier shutdown's %r",
                ", ".join(other),
                LEFTOVER_REASON,
            )
            return never
        logging.warning(
            "ace2k: MCU %s still stopped from an earlier shutdown (%s%s): one FIRMWARE_RESTART",
            names,
            LEFTOVER_REASON,
            f"; {found} found at connect, its reason not sent" if found is not None else "",
        )
        self.printer.request_exit("firmware_restart")
        return never


def mcu_stop(mcu: object) -> tuple[bool, object]:
    """Return (stopped, reason) of a Klipper MCU object, or (False, None) if it has neither.

    The values are read from its own fields (the Klipper of the U1) or from its connect helper
    (current Klipper).
    """
    for obj in (mcu, getattr(mcu, "_conn_helper", None)):
        is_shutdown = getattr(obj, "is_shutdown", None)
        if callable(is_shutdown):
            getter = getattr(obj, "get_shutdown_msg", None)
            reason = getter() if callable(getter) else getattr(obj, "_shutdown_msg", None)
            return bool(is_shutdown()), reason
    return False, None


def attach(config: Config) -> Units:
    """Return the coordinator of the printer. The first unit creates it from the config."""
    printer = config.get_printer()
    units: Units | None = printer.lookup_object("ace2k_units", None)
    if units is None:
        units = Units(printer, unit_names(config))
        printer.add_object("ace2k_units", units)
    return units
