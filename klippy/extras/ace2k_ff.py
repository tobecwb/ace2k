"""The feed-forward on the host: the lane meters out what its extruder is about to use.

The unit cannot run a lane at the steady rate of a print (about 1 to 6 mm/s, which is below the
floor speed of the motor). So it meters the filament out in small doses. Only a lane in
``following`` takes them (see ``ace2k_feed_base``). This module tells the unit the rate.

How the rate is worked out
--------------------------
The toolhead of Klipper hands each extruding move to the active extruder as it flushes the move
from its look-ahead queue, ahead of real time. This module wraps ``process_move`` of that extruder
(newer Klipper) or ``move`` (older trees, including the U1). The wrapper records
``(print_time, duration, extruder distance)`` for each move.

Every ``ff_period_ms``, the module works out the share of one tick as follows:

1. It takes the signed extruder distance that the recorded moves cover in the window
   ``[now + ff_lead_ms, now + ff_lead_ms + ff_window_ms]`` (print time). A window shorter than
   ``ff_period_ms`` counts as one period. Otherwise the moments between the windows of two ticks
   would never be metered.
2. It multiplies this distance by the print time since the last tick of the extruder, divided by
   the window.

The window slides by that print time, so the shares add up to the net E of the extruder. If the
reactor runs a tick late, the tick sweeps the missed time in steps of one period, and each step
has its own window.

Retractions
-----------
A negative share is a retraction. It is credited to the extruder, up to ``CREDIT_MAX_MM``, and the
rate of the tick is 0. A positive share pays the credit back first, from the mean of the window.
So a retraction is held back from the next extrusion and is not lost. A floor at 0 would overfeed
by the length of the retraction at every travel move.

A contiguous retraction of ``ff_retract_min_mm`` or more is not credited. Such a retraction is
the tip forming of an unload, and not a retraction of a print. Its own rate goes out as a negative
rate, and the lane takes the filament back. The rate is its mean over one period from now, without
``ff_lead_ms``. This needs an image with the reverse (``ACE2K_FEED_FF_REVERSE``). Without it, the
retraction is credited like any other.

A run is the recorded moves with E < 0 between two moves with E > 0. The XY moves of a wipe
between them are not recorded. A gap of more than one window (at least one period) ends the run.
Every drop of the credit (see below) also ends it. A run that grows long keeps what was already
credited of it. Each move is classed the first time that the window, or the period of the long
part, reaches it, and it counts in that part only. So, for any window, period and lead, no move is
both credited and taken back.

The credit is dropped in these cases:

- the extruder has no lane in ``following``, or has two;
- the lane in the follow is switched off, held or remapped;
- a shutdown.

An idle lane of the extruder changes nothing.

Which lane gets the rate
------------------------
The rate goes to the one lane of the extruder that is in ``following``. If there is none, nothing
is sent. If there are two or more, there is no rate. Each of those lanes is told once to drop what
it owes, and a console line says so. This module does not guess which lane the filament comes
from.

With several units, the followers are counted across all of them (see ``ace2k_units``). A unit
doses an extruder only when its own lane is the one follower.

The module sends a rate when it differs from the last one by more than ``ff_deadband``, when it
rises from 0, or when it falls to 0. A rate of 0 stops the accrual, but the unit still delivers
what the lane owes. A lane that is remapped while it follows is told to drop what it owes,
because that filament belonged to the old extruder.

An automatic encoder calibration (``ace2k_encoder_cal.py``) holds its lane for the run. While the
lane is held, there is no rate and no metering. What the lane owes is dropped at the start of the
hold. The switch of the user is not changed.

Configuration and notices
-------------------------
The map of lanes to extruders is the key ``[ace2k] laneN_extruder``. Several lanes may name one
extruder. Host modules can set a lane without a key through ``Ace2kFeed.set_lane_extruder``.

The correction counters of the unit are the taut catch-ups and the full take-ups of the follow
while it meters. They appear in the status of the lanes. The module prints a console line after
every ``ff_notice_every`` corrections in the same direction since the last notice. It counts only
counters whose interval saw the lane metering (the feed-forward on and a rate above 0 in force).
A lane that keeps correcting in one direction meters a wrong length, so its encoder scale is off.

Example configuration::

    [ace2k]
    #lane1_extruder: extruder   # the extruder that lane 1 feeds (lane1..lane4); without it, no
    #                           # feed-forward
    #feed_forward: True         # ACEK_FEED_FORWARD ON=1 / OFF=1 switches it live
    #ff_period_ms: 100          # how often the rate is worked out (20-1000)
    #ff_window_ms: 500          # the stretch of planned moves that the rate is the mean of
    #                           # (100-5000); it is raised to ff_period_ms if it is shorter
    #ff_lead_ms: 0              # how far ahead of now the window starts (0-5000)
    #ff_deadband: 0.3           # mm/s: a smaller change is not sent (0-10)
    #ff_retract_min_mm: 10      # mm: a contiguous retraction this long is taken back (5-17)
    #ff_notice_every: 20        # corrections in one direction before a console notice (1-10000)

``ff_chunk_mm`` and ``ff_pulse`` are values of the unit, and ``ace2k_feed.py`` handles them.
``ff_pulse`` is retired: this module says so at the start (FF_PULSE_RETIRED), and the next version
removes it.
``ace2k_feed.py`` loads this module. An image without the feed-forward never calls it.
"""

from __future__ import annotations

import collections
import logging
import math
import threading
from collections.abc import Iterable, Sequence
from typing import TYPE_CHECKING, Callable, Protocol, TypeVar, Union, cast

try:
    from .ace2k_feed_proto import (  # noqa: F401 — re-exported
        BASE_FMT,
        BASE_FMT_FORWARD,
        NO_FF,
        REVERSE_CONSTANT,
    )
    from .ace2k_link import LANES, NO_LANE, PAIR, check_lane
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — the formats re-exported
        BASE_FMT,
        BASE_FMT_FORWARD,
        NO_FF,
        REVERSE_CONSTANT,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANES,
        NO_LANE,
        PAIR,
        check_lane,
    )

if TYPE_CHECKING:
    from .ace2k_feed_motion import LaneMotion
    from .ace2k_klipper_types import MCU, Command, Config, LaneStatus, Printer, ToolheadMove
    from .ace2k_units import Key, Units
    from .ace2k_units import UnitProto as CoordinatedUnit

    class UnitProto(CoordinatedUnit, Protocol):
        """``ace2k.Ace2k``, as the feed-forward uses it to name lanes and ask the coordinator."""

        @property
        def units(self) -> Units: ...
        def head(self, part: str | None = None) -> str: ...
        def lane_text(self, lane: int) -> str: ...
        def say(self, line: str) -> None: ...

    class FeedProto(Protocol):
        """``ace2k_feed.Ace2kFeed``, the feed that the feed-forward doses."""

        @property
        def unit(self) -> UnitProto: ...
        @property
        def present(self) -> bool: ...
        @property
        def has_ff(self) -> bool: ...
        @property
        def has_ff_reverse(self) -> bool: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...
        @property
        def motion(self) -> list[LaneMotion]: ...
        @property
        def speed_max(self) -> float | None: ...  # None until the connect step
        @property
        def ff_base_cmd(self) -> Command | None: ...  # None until connect, or without feed-forward
        def in_follow(self, index: int) -> bool: ...
        def open_seq(self, index: int) -> int | None: ...
        def follow_running(self, index: int) -> bool: ...
        def lane_unavailable(
            self, index: int, allow: tuple[str, ...] | None = ("idle",)
        ) -> str | None: ...
        def _lane_args(self, index: int) -> str: ...


COUNT_MASK = 0xFFFF  # the correction counters of the unit are uint16
MOVES_MAX = 4096  # recorded moves kept per extruder, in case the timer stops removing old ones
# NO_FF, BASE_FMT, BASE_FMT_FORWARD and REVERSE_CONSTANT are in ace2k_feed_proto.
NO_REVERSE = "the unit's firmware does not take back a long retraction; flash v0.12.0"
# ff_pulse no longer sets a forward dose's speed. It still works in this version, and the next one
# removes it (the rule of the host interface: a warning one version before).
FF_PULSE_KEY = "ff_pulse"
FF_PULSE_RETIRED = (
    "ff_pulse is retired: every forward dose runs at the floor speed, so it sets only a reverse"
    " dose's least speed. The next version removes it, with ACEK_FF_SET's PULSE, and the unit's"
    " own value is used. Remove it from printer.cfg."
)
# Host-side keys: name, default, minval, maxval.
PERIOD_MS = ("ff_period_ms", 100, 20, 1000)
WINDOW_MS = ("ff_window_ms", 500, 100, 5000)
LEAD_MS = ("ff_lead_ms", 0, 0, 5000)
DEADBAND = ("ff_deadband", 0.3, 0.0, 10.0)
NOTICE_EVERY = ("ff_notice_every", 20, 1, 10000)
# The most retraction, in mm, that an extruder may have credited. It is far above the retractions
# that a slicer plans (about 0.5 to 7 mm). So it limits only a runaway, such as a very long
# negative E, and never a normal print.
CREDIT_MAX_MM = 50.0
# The most window steps that one tick sweeps after the reactor ran late: 5 s at the default period.
SWEEP_STEPS_MAX = 50
# A contiguous retraction of this length (mm) or more is not a retraction of a print (those are
# about 0.5 to 7 mm). The lane takes it back together with the extruder. A shorter retraction is
# credited. The upper limit is below the buffer size of about 18 mm. So any retraction shorter
# than the limit fits in the buffer, and the rest absorbs the start of a long one.
RETRACT_MIN_MM = ("ff_retract_min_mm", 10.0, 5.0, 17.0)
RUN_SLOT, CLASS_SLOT = 3, 4  # the slots of a recorded move for its Retraction and its class
ROUNDS_TO_ZERO_MM_S = 0.0005  # a rate below this rounds to 0 µm/s, which is what the unit gets


class Retraction:
    """A contiguous retraction: the recorded moves with E < 0 between two moves with E > 0.

    Moves without E are not recorded, so the XY moves of a wipe do not break a retraction.
    Each move must start no later than one window (at least one period) after the end of the
    last move (``end_pt``, in print time). A later move opens a new run.

    ``total`` (mm, positive) grows as the moves are recorded. A move is long once the total of its
    run reaches the threshold. A run can become long after some of its moves were already
    credited."""

    __slots__ = ("total", "end_pt")

    def __init__(self) -> None:
        self.total = 0.0
        self.end_pt = 0.0


N = TypeVar("N", int, float)  # the type of a key: from config.getint or config.getfloat
# A planned move as it is recorded: [print_time, duration, e_distance, its Retraction or None,
# its class]. The class is True for long, False for the rest, and None until it is frozen.
# The move is a list, so that window_parts can freeze the class in place. Each slot has its own
# type, so read a slot with a cast.
PlannedMove = list[Union[float, Retraction, None]]
Span = tuple[float, float]  # (start_pt, end_pt), print time


def _overlap(pt: float, dur: float, start_pt: float, end_pt: float) -> float:
    """Return the part (0..1) of a move that lies inside [start_pt, end_pt].

    The move starts at ``pt`` and lasts ``dur``."""
    lo, hi = max(pt, start_pt), min(pt + dur, end_pt)
    return (hi - lo) / dur if hi > lo else 0.0


def window_parts(
    moves: Iterable[PlannedMove],
    long_span: Span,
    rest_span: Span,
    min_mm: float,
    freeze: bool = False,
) -> tuple[float, float]:
    """Return (long, rest) in one pass over the moves.

    Each move is (print_time, duration, e_distance[, run[, long]]). ``long`` is the signed
    extruder distance (mm) that the moves of a contiguous retraction of ``min_mm`` or more command
    inside ``long_span``. ``rest`` is the same for all the other moves inside ``rest_span``.
    Each span is (start_pt, end_pt). A move counts in proportion to its overlap with the span.

    A move with a frozen class (its fifth slot is not None) counts in that part only. Otherwise
    the total of its run decides. With ``freeze``, a recorded move (a list) keeps its class from
    the first span of either part that reaches it. So each moment of a move counts in one part
    only, whatever the window, the period and the lead. A run that turns long later leaves the
    move in the rest, as credited."""
    long_part = rest = 0.0
    for move in moves:
        pt, dur = cast(float, move[0]), cast(float, move[1])
        if dur <= 0.0:
            continue
        in_long = _overlap(pt, dur, *long_span)
        in_rest = _overlap(pt, dur, *rest_span)
        is_long = cast("bool | None", move[CLASS_SLOT]) if len(move) > CLASS_SLOT else None
        if is_long is None:
            run = cast("Retraction | None", move[RUN_SLOT]) if len(move) > RUN_SLOT else None
            is_long = run is not None and run.total >= min_mm - 1e-9
            if freeze and (in_long > 0.0 or in_rest > 0.0) and len(move) > CLASS_SLOT:
                move[CLASS_SLOT] = is_long
        if is_long:
            long_part += cast(float, move[2]) * in_long
        else:
            rest += cast(float, move[2]) * in_rest
    return long_part, rest


def choose_lane(
    lanes_of_extruder: Iterable[int], modes: Sequence[str | None]
) -> tuple[int | None, list[int]]:
    """Return (lane, following) for an extruder.

    ``following`` lists the lanes of the extruder that are in "following", in order. ``lane`` is
    the one lane in that list if there is exactly one. Otherwise it is None, for none or for
    several lanes."""
    following = [lane for lane in lanes_of_extruder if modes[lane] == "following"]
    return (following[0] if len(following) == 1 else None), following


def lanes_text(lanes: Sequence[int]) -> str:
    """Return the lane numbers as text for a console line, such as "lanes 1 and 2 both"."""
    names = [str(lane + 1) for lane in lanes]
    if len(names) == PAIR:
        return f"lanes {names[0]} and {names[1]} both"
    return f"lanes {', '.join(names[:-1])} and {names[-1]} all"


class FeedForward:
    """The feed-forward of one unit: it meters the extruders' needs out to the lanes."""

    def __init__(self, printer: Printer, feed: FeedProto, config: Config) -> None:
        self.printer = printer
        self.feed = feed
        self.reactor = printer.get_reactor()
        self.keys: dict[int, str] = {}  # lane -> extruder name, from [ace2k]; wins over runtime
        for lane in range(LANES):
            name = config.get(f"lane{lane + 1}_extruder", None)
            if name:
                self.keys[lane] = name.strip()
        self.runtime: dict[int, str] = {}  # lane -> extruder name, from set_lane_extruder
        # extruder name -> its lanes in order, kept up to date by refresh_map
        self.lanes_of: dict[str, list[int]] = {}
        self.refresh_map()
        if config.get(FF_PULSE_KEY, None) is not None:
            config.deprecate(FF_PULSE_KEY)  # Klipper lists it among its warnings at the start
            logging.warning("%s %s", feed.unit.head(), FF_PULSE_RETIRED)
        self.enabled = config.getboolean("feed_forward", True)
        self.on = [self.enabled] * LANES
        self.held: set[int] = set()  # lanes held by an encoder calibration: no rate, no metering
        self.period_ms = self._read(PERIOD_MS, config.getint)
        self.window_ms = self._read(WINDOW_MS, config.getint)
        self.lead_ms = self._read(LEAD_MS, config.getint)
        self.deadband_mm_s = self._read(DEADBAND, config.getfloat)
        self.notice_every = self._read(NOTICE_EVERY, config.getint)
        self.retract_min_mm = self._read(RETRACT_MIN_MM, config.getfloat)
        # extruder name -> deque of [print_time, duration, e_distance, its Retraction or None,
        # its class]. The class is True for long, False for the rest, and None until it is frozen.
        self.moves: dict[str, collections.deque[PlannedMove]] = {}
        # extruder name -> its open Retraction, until a move with E > 0 arrives
        self.runs: dict[str, Retraction] = {}
        # extruder name -> mm retracted, held back from its next extrusion
        self.credit: dict[str, float] = {}
        # extruder name -> the window start (print time) that its last share swept to
        self.swept: dict[str, float] = {}
        self.rates = [0.0] * LANES  # mm/s that was sent last; 0 once the lane has left the follow
        # The start sequence of the lane when the rate went out. If a new start came since then
        # (a new arm, between two state reports), the rate of the unit is 0 again.
        self.sent_seq: list[int | None] = [None] * LANES
        # Extruders for which the console already reported the current episode of two lanes.
        self.ambiguous: set[str] = set()
        self.record_failed = False
        self.clock: MCU | None = None  # the MCU of the toolhead, which gives the print time
        self.timer: object | None = None  # the handle of the tick timer in the reactor
        self.ready = False
        # For the notices: the last counters of the unit for each lane, and the net count since the
        # last notice.
        self.last_counts = [(0, 0)] * LANES
        # The epoch of the unit that the last counters belong to.
        self.epochs: list[int | None] = [None] * LANES
        self.net = [0] * LANES
        # True if the lane metered at some moment since its last counters. The tick and a send set
        # it while a rate above 0 is in force. The next counters consume it.
        self.metered = [False] * LANES
        self.metered_lock = threading.Lock()  # protects the swap on the serial thread and the mark
        printer.register_event_handler("klippy:ready", self._on_ready)

    @staticmethod
    def _read(spec: tuple[str, N, N, N], read: Callable[..., N]) -> N:
        """Read one config key from its (key, default, min, max) spec."""
        key, default, low, high = spec
        return read(key, default, minval=low, maxval=high)

    # --- the map -------------------------------------------------------------------------

    def extruder_of(self, lane: int) -> str | None:
        """Return the extruder name of the lane, or None. A config key wins over a runtime map."""
        return self.keys.get(lane) or self.runtime.get(lane)

    def refresh_map(self) -> None:
        """Work out the lanes of each extruder again, after the map changed.

        The map changes through set_lane_extruder and through map_changed of the coordinator."""
        lanes_of: dict[str, list[int]] = {}
        for lane in range(LANES):
            name = self.extruder_of(lane)
            if name:
                lanes_of.setdefault(name, []).append(lane)
        self.lanes_of = lanes_of

    def set_lane_extruder(self, lane_index: int, extruder_name: str | None) -> None:
        """Map lane ``lane_index`` (0..3) to ``extruder_name``. None removes the mapping.

        A laneN_extruder config key takes priority. After the printer is ready, the extruder is
        hooked at once."""
        check_lane(lane_index, NO_LANE)
        before = self.extruder_of(lane_index)
        if extruder_name:
            self.runtime[lane_index] = extruder_name
        else:
            self.runtime.pop(lane_index, None)
        self.refresh_map()
        self.feed.unit.units.map_changed()  # tell the coordinator, which tracks shared extruders
        if lane_index in self.keys:
            logging.info(
                "%s lane %s keeps laneN_extruder %s (asked %s)",
                self.feed.unit.head(),
                self.feed.unit.lane_text(lane_index),
                self.keys[lane_index],
                extruder_name,
            )
            return
        after = self.extruder_of(lane_index)
        if after != before:
            # A credited retraction is filament of the follower of the extruder. Drop it when that
            # follower is the lane that was remapped, or when no lane is left in the follow.
            for name in (before, after):
                if name:
                    self._drop_if_owner(lane_index, name)
            if self._may_owe(lane_index):
                # The rate of the old extruder is not the rate of this one, and neither is what the
                # lane owes. Clear both.
                self._send(lane_index, 0.0, clear=True)
        if self.ready and extruder_name:
            self._hook_lane(lane_index)

    # --- the planner hook ----------------------------------------------------------------

    def _on_ready(self) -> None:
        self.ready = True
        if not self.feed.present:
            return
        if not self.feed.has_ff:
            if self.enabled:
                self._say(f"ace2k: {NO_FF}")
            return
        if self.enabled and not self.feed.has_ff_reverse:
            self._say(f"ace2k: {NO_REVERSE}")
        for lane in range(LANES):
            if self.extruder_of(lane):
                self._hook_lane(lane)

    def _hook_lane(self, lane: int) -> None:
        """Hook the extruder of the lane, once per extruder.

        If it cannot be hooked, a console line says so."""
        if not self.feed.has_ff:
            return
        name = self.extruder_of(lane)
        assert name is not None  # every caller hooks a lane that has a mapping
        if name in self.moves:
            return
        try:
            obj = self.printer.lookup_object(name)
        except Exception:  # a missing object is a config error in Klipper, and a KeyError in tests
            obj = None
        if obj is None:
            self._say(f"ace2k: lane {lane + 1}: no extruder {name} — no feed-forward on it")
            return
        attr = next((a for a in ("process_move", "move") if callable(getattr(obj, a, None))), None)
        if attr is None:
            self._say(
                f"ace2k: lane {lane + 1}: {name} has neither process_move nor move — no"
                " feed-forward on it"
            )
            return
        if self.clock is None:
            self.clock = self.printer.lookup_object("mcu")
        self.moves[name] = collections.deque(maxlen=MOVES_MAX)
        original = getattr(obj, attr)

        # This wraps the hook of Klipper. Its arguments and its result are passed on unchanged.
        def wrapper(
            print_time: float, move: ToolheadMove, *args: object, **kwargs: object
        ) -> object:
            # Call the original first. Any error it raises is an error of Klipper. The recording
            # never raises.
            result = original(print_time, move, *args, **kwargs)
            try:
                e_distance = move.axes_d[3]
                if e_distance:  # most moves have no E: they skip the sum and the call
                    duration = move.accel_t + move.cruise_t + move.decel_t
                    self.note_move(name, print_time, duration, e_distance)
            except Exception:
                if not self.record_failed:
                    self.record_failed = True
                    logging.exception(
                        "%s feed-forward: a planned move of %s could not be recorded",
                        self.feed.unit.head(),
                        name,
                    )
            return result

        setattr(obj, attr, wrapper)
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._tick, self.reactor.monotonic())

    def note_move(self, name: str, print_time: float, duration: float, e_distance: float) -> bool:
        """Record one move of the extruder ``name``, as the hook records a planned move.

        This is for a module that moves an extruder's stepper outside the extruder's own move,
        such as a parked head. The lane meters the move out as it does a planned move, and takes
        a long retraction back. ``print_time`` is the start of the move, ``duration`` its length
        in seconds and ``e_distance`` its E in mm (negative for a retraction).

        Return False if the extruder is not hooked: nothing is recorded. Return True otherwise.
        A move without E records nothing."""
        moves = self.moves.get(name)
        if moves is None:
            return False
        if not e_distance:
            return True
        run = None
        if e_distance < 0.0:
            run = self.runs.get(name)
            # A gap of more than one window (at least one period) without E ends the run. A
            # retraction of the print, long after the retraction of an unload, is not part of it.
            if run is None or print_time > run.end_pt + self._window_s():
                run = self.runs[name] = Retraction()
            run.total -= e_distance
            run.end_pt = print_time + duration
        else:
            self.runs.pop(name, None)  # an extrusion ends the retraction
        # The last slot is the class (long or not). window_parts freezes it once a span of either
        # part reaches the move.
        moves.append([print_time, duration, e_distance, run, None])
        return True

    # --- the rate ------------------------------------------------------------------------

    def _tick(self, eventtime: float) -> float:
        """Run one tick of the timer and return the time of the next tick."""
        if not self.feed.has_ff:
            return self.reactor.NEVER
        if self.printer.is_shutdown():
            # Klipper or the unit has shut down. The modes are stale, so nothing may be sent.
            self.credit.clear()
            self.swept.clear()
            self.runs.clear()
            return self.reactor.NEVER
        try:
            self._update(eventtime)
        except Exception:
            logging.exception("%s feed-forward: the rate update failed", self.feed.unit.head())
        return eventtime + self.period_ms / 1000.0

    def _update(self, eventtime: float) -> None:
        """Work out and send the rate of every hooked extruder."""
        assert self.clock is not None  # the tick runs once the extruder of a lane is hooked
        # A follow whose final event has come is over, even if the last report still says it
        # follows.
        modes = [self._mode(lane) for lane in range(LANES)]
        for lane in range(LANES):
            self._mark(lane)  # the rate that was held since the last tick
            if modes[lane] != "following" or self.feed.motion[lane].seq != self.sent_seq[lane]:
                self.rates[lane] = 0.0  # outside the follow the unit drops the rate and arms at 0
        pt_now = self.clock.estimated_print_time(eventtime)
        for name, moves in self.moves.items():
            self._update_extruder(name, moves, modes, pt_now)
            # The moves are recorded in planning order. The print time of the toolhead only moves
            # forward, and each move starts where the last one ended. So the ends are in order,
            # and the moves in the past are a prefix of the queue. The next share sweeps from this
            # start on, and never before now.
            while moves and cast(float, moves[0][0]) + cast(float, moves[0][1]) < pt_now:
                moves.popleft()

    def _update_extruder(
        self,
        name: str,
        moves: collections.deque[PlannedMove],
        modes: Sequence[str | None],
        pt_now: float,
    ) -> None:
        """Work out the rate of one extruder and send it to its lane, if there is one."""
        lane, following = choose_lane(self.lanes_of.get(name, ()), modes)
        units = self.feed.unit.units
        others = units.followers_elsewhere(name, self.feed.unit)
        if self._conflict(name, following, others):
            return
        self.ambiguous.discard(name)
        # The coordinator ends the conflict only when at most one follower is left on any unit.
        units.clear_shared(name, len(following) + len(others))
        if others:
            self._drop(name)  # the one follower of the extruder is a lane of another unit
            return
        if lane is None or not self.on[lane] or lane in self.held:
            self._drop(name)
            return
        rate = self._spend(name, moves, pt_now)
        if rate is not None:
            self._maybe_send(lane, rate)

    def _conflict(self, name: str, following: Sequence[int], others: list[Key]) -> bool:
        """Return True if more than one lane of any unit follows for the extruder.

        Then there is no feed-forward on it, and this function has already dealt with the
        conflict. Return False for an ordinary tick."""
        if len(following) + len(others) <= 1:  # the lanes of every unit that follow for ``name``
            return False
        units = self.feed.unit.units
        self._drop(name)
        if not following:
            # The conflict is between other units only. This unit sends nothing, says nothing and
            # marks nothing. Its own episode, if it had one, is over. If its lanes join later,
            # they start a new episode. Then what they owe is cleared and its own line is said.
            self.ambiguous.discard(name)
            return True
        # It is not known which lane the filament comes from. So what each lane owes is dropped,
        # once at the start of the episode, and again for a lane that has had a rate since.
        for other in following:
            if self.rates[other] or name not in self.ambiguous:
                self._send(other, 0.0, clear=True)
        mine = [units.key(self.feed.unit, other) for other in following]
        if len({i for i, _ in mine + others}) > 1:
            # The followers are on more than one unit. The coordinator says it once per conflict,
            # whichever unit sees it first.
            units.say_shared(name, mine + others)
        elif name not in self.ambiguous:
            # All followers belong to this unit, so it says its own line, as with one unit.
            self._say(
                f"ace2k: extruder {name}: {lanes_text(following)} in follow — no feed-forward"
            )
        self.ambiguous.add(name)
        return True

    def _maybe_send(self, lane: int, rate: float) -> None:
        """Send the rate to the lane if it changed enough."""
        if abs(rate) < ROUNDS_TO_ZERO_MM_S:
            rate = 0.0  # the unit gets µm/s, so a rate that rounds to 0 is 0
        last = self.rates[lane]
        # Send if the change is past the deadband. Also send a start, a stop or a change of
        # direction, whatever its size. The rate of a slow print can stay under the deadband, and
        # it is still the rate of the lane. The credit is spent as if that rate were in force.
        if (
            abs(rate - last) > self.deadband_mm_s
            or (rate == 0.0) != (last == 0.0)
            or (rate < 0.0) != (last < 0.0)
        ):
            self._send(lane, rate)

    def _window_s(self) -> float:
        """Return the window in seconds. It is never shorter than one period.

        A shorter window, held for a whole period, would leave the moments between its samples
        neither metered nor credited. It would also break the retraction of a wipe into short
        runs at its gaps."""
        return max(self.window_ms, self.period_ms) / 1000.0

    def _spend(
        self, name: str, moves: collections.deque[PlannedMove], pt_now: float
    ) -> float | None:
        """Return the rate of this tick in mm/s, or None if no print time has passed.

        The share of the tick is the total of the window (the window from ``pt_now`` plus the
        lead), multiplied by the print time swept since the last tick of the extruder, divided by
        the window. The first tick sweeps one period. A sweep longer than one period, because the
        reactor was late, is cut into steps of at most one period. Each step has its own window.
        So every planned moment counts once, whatever the ticks do.

        A negative share of the rest is credited, and that part of the rate is 0. A positive share
        pays the credit back first.

        The part of a long retraction is its mean over one period from the print time of each
        step, without the lead. This is the own rate of the retraction when the head runs it. It
        is negative, it is taken over the span that was swept, and it is added as it is."""
        period = self.period_ms / 1000.0
        window = self._window_s()
        lead = self.lead_ms / 1000.0
        start = pt_now + lead
        last = self.swept.get(name, start - period)
        if start <= last:
            return None
        self.swept[name] = start
        elapsed = start - last
        steps = min(SWEEP_STEPS_MAX, max(1, math.ceil(elapsed / period - 1e-9)))
        step = elapsed / steps
        # The lane takes back the part of the long retractions, and it is never credited. The
        # rest works as usual.
        min_mm = self.retract_min_mm if self.feed.has_ff_reverse else math.inf
        amount = back = now = 0.0
        for k in range(1, steps + 1):
            at = start if k == steps else last + k * step
            # The long part is taken over one period from the print time, without the lead. It is
            # the own rate of the retraction when the head runs it. A reverse dose ahead of it
            # pulls the strand toward taut.
            long_part, now = window_parts(
                moves, (at - lead, at - lead + period), (at, at + window), min_mm, freeze=True
            )
            amount += now * step / window
            back += long_part * step / period
        back_rate = back / elapsed  # at most 0: the mean of the long retractions over the span
        credit = self.credit.get(name, 0.0)
        if amount < 0.0:
            self.credit[name] = min(CREDIT_MAX_MM, credit - amount)
            return back_rate
        use = min(credit, amount)
        self.credit[name] = credit - use
        # The mean of the rest now, minus the credit paid back, plus the long retractions
        # (at most 0).
        return max(0.0, now / window - use / elapsed) + back_rate

    def _followers(self, name: str) -> list[int]:
        """Return the lanes of the extruder that are in the follow."""
        return [
            lane
            for lane in range(LANES)
            if self.extruder_of(lane) == name and self.feed.lanes[lane]["mode"] == "following"
        ]

    def _drop_if_owner(self, lane: int, name: str) -> None:
        """Drop the credit and sweep of the extruder if the lane is its follower.

        Also drop them if no lane of the extruder is left in the follow. A credited retraction is
        filament of the follower of the extruder."""
        if self.feed.lanes[lane]["mode"] == "following" or not self._followers(name):
            self._drop(name)

    def forget(self, name: str) -> None:
        """Forget what was recorded for a shared extruder. The coordinator calls this on every unit.

        The active lane of the shared extruder ``name`` is about to change. What was recorded
        belongs to the lane that leaves. So the credit, the sweep and the open retraction are
        dropped. The moves recorded so far are dropped too. They were planned with the filament of
        that lane in the head, and their long retractions are marked for it. So the next lane is
        neither dosed nor taken back for them. Moves recorded from now on belong to the new lane.
        The follow covers what the forgotten moves would have dosed."""
        self._drop(name)
        moves = self.moves.get(name)
        if moves is not None:
            moves.clear()

    def _drop(self, name: str) -> None:
        """Forget the credit, the sweep and the open retraction of the extruder.

        Its next tick starts afresh, and a retraction from now on opens a new run."""
        self.credit.pop(name, None)
        self.swept.pop(name, None)
        self.runs.pop(name, None)

    def set_lead(self, lead_ms: int) -> None:
        """Set a new lead, in ms, for ACEK_FF_SET LEAD_MS.

        A new lead moves the start of every window. So the sweep, the credit and the open
        retraction of each extruder are dropped, and its next tick starts afresh. If they were
        kept, a lower lead would leave the sweep ahead of the new start. Then no rate, not even 0,
        would go out until the print time caught up with it. A new window needs nothing, because
        the sweep follows the start of the windows."""
        if lead_ms != self.lead_ms:
            self.swept.clear()
            self.credit.clear()
            self.runs.clear()  # the credit is gone, so an open run starts afresh too
        self.lead_ms = lead_ms

    def _may_owe(self, lane: int) -> bool:
        """Return True if the unit may hold a debt on the lane.

        That is the case when a rate is in force, or when the lane is in the follow. A rate of 0
        stops the accrual, but the unit still pays what is owed. After a shutdown of Klipper or of
        the unit, the answer is always False, because the modes are stale and nothing may be
        sent."""
        if self.printer.is_shutdown():
            return False
        if self.rates[lane]:
            return True
        return bool(self.feed.has_ff and self.feed.lanes[lane]["mode"] == "following")

    def _send(self, lane: int, rate: float, clear: bool = False) -> None:
        """Send a rate (mm/s) to the lane with the command ace2k_feed_base.

        The rate is limited to the top speed of the lane in both directions, because the unit
        shuts down above it. A rate of 0 only stops the accrual. What the lane owes is filament
        that the head has already pulled, and the unit still pays it. With ``clear``, the unit
        drops that debt too. Use it for a lane that no longer feeds its extruder alone."""
        # Used only on an image with the feed-forward, whose connect step built both.
        assert self.feed.speed_max is not None and self.feed.ff_base_cmd is not None
        top = int(round(self.feed.speed_max * 1000.0))
        low = -top if self.feed.has_ff_reverse else 0  # the unsigned command is never below 0
        um = max(low, min(int(round(rate * 1000.0)), top))
        self._mark(lane)  # the rate that this replaces was in force until now
        self.feed.ff_base_cmd.send([lane, um, 1 if clear else 0])
        self.rates[lane] = um / 1000.0
        self.sent_seq[lane] = self.feed.motion[lane].seq

    def _metering(self, lane: int) -> bool:
        """Return True if the lane is metering forward now."""
        return bool(
            # Only a forward rate counts. A long retraction fills the buffer by design, and it
            # says nothing about the scale.
            self.on[lane] and self.feed.has_ff and self.rates[lane] > 0.0 and lane not in self.held
        )

    def _mark(self, lane: int) -> None:
        """Remember that the lane metered since its last counters."""
        if self._metering(lane):
            with self.metered_lock:
                self.metered[lane] = True

    # --- the switch ----------------------------------------------------------------------

    def set_on(self, lanes: Iterable[int], on: bool) -> None:
        """Switch the feed-forward of the lanes on or off, for ACEK_FEED_FORWARD.

        Switching off sends a rate of 0 once to a lane that has a rate. After that, no rate goes
        out until it is switched on. A lane that is off can still get a clear, for example when
        it is one of two lanes of an extruder in the follow, or when it is remapped. A clear moves
        nothing, so it is not a feed."""
        for lane in lanes:
            self.on[lane] = on
            name = self.extruder_of(lane)
            if not on and name:
                # The credit belongs to the follower of the extruder. Drop it if that follower is
                # this lane, or if no lane of the extruder is left in the follow.
                self._drop_if_owner(lane, name)
            if not on and self.rates[lane]:
                self._send(lane, 0.0)

    def hold(self, lane: int, held: bool) -> None:
        """Hold or release the feed-forward of a lane for an automatic calibration.

        While the lane is held, there is no rate and no metering. The switch of the user does not
        change. At the start of the hold, what the lane owes is dropped, because a dose would move
        the filament between the marks of the run. The credit and the sweep of the extruder are
        dropped too, if the lane is its follower or no follower is left."""
        if not held:
            self.held.discard(lane)
            return
        self.held.add(lane)
        name = self.extruder_of(lane)
        if name:
            self._drop_if_owner(lane, name)
        if self._may_owe(lane):
            self._send(lane, 0.0, clear=True)

    # --- the notices (the serial thread) -------------------------------------------------

    def on_counts(self, taut: Sequence[int], full: Sequence[int], epoch: Sequence[int]) -> None:
        """Take in the correction counters of the unit, and print a notice when there are many.

        ``taut`` and ``full`` are the corrections of each lane since the last entry (uint16).
        ``epoch`` (uint8) is the epoch of that entry. The unit steps it at every entry, whether the
        host or the unit started it.

        A new epoch in the report means a new count, and all of it is new. Within one epoch the
        counters only grow, and a wrap is counted. The first report of the session is the
        baseline, and counting starts from it. After a restart of Klipper, the unit still has
        totals from before the restart, and they do not belong to this session.

        A report adds to the tally only if the lane metered at some moment since the previous
        report. This means that its feed-forward was on and a rate above 0 was in force. The plain
        follow corrects on every taut signal, which says nothing about the scale. A report that
        does not count still moves the baseline, so its corrections are never credited later."""
        for lane in range(LANES):
            if self.epochs[lane] is None:
                self.epochs[lane] = epoch[lane]
                self.last_counts[lane] = (taut[lane], full[lane])
                with self.metered_lock:
                    self.metered[lane] = self._metering(lane)
                continue
            prev_taut, prev_full = self.last_counts[lane]
            rearmed = epoch[lane] != self.epochs[lane]
            self.epochs[lane] = epoch[lane]
            if rearmed:
                d_taut, d_full = taut[lane], full[lane]
            else:
                d_taut = (taut[lane] - prev_taut) & COUNT_MASK
                d_full = (full[lane] - prev_full) & COUNT_MASK
            self.last_counts[lane] = (taut[lane], full[lane])
            with self.metered_lock:
                metered = self.metered[lane]
                self.metered[lane] = self._metering(lane)  # a lane that still meters stays set
            if not metered:
                continue
            self.net[lane] += d_taut - d_full
            if abs(self.net[lane]) >= self.notice_every:
                which = "taut" if self.net[lane] > 0 else "full"
                count = abs(self.net[lane])
                self.net[lane] = 0
                line = (
                    f"{self.feed.unit.head()} lane {self.feed.unit.lane_text(lane)}: {count}"
                    f" {which} corrections since the last notice"
                    " — check the encoder scale (ACEK_CALIBRATE_ENCODER)"
                )
                logging.info("%s", line)

                def say(_eventtime: float, line: str = line) -> None:
                    self._say(line)

                self.reactor.register_async_callback(say)

    # --- status --------------------------------------------------------------------------

    def lane_status(self, lane: int) -> dict[str, object]:
        """Return the feed-forward status of the lane."""
        return dict(
            ff_rate=self.rates[lane],
            ff_on=bool(self.on[lane] and self.feed.has_ff),
            ff_extruder=self.extruder_of(lane),
        )

    def settings_text(self) -> str:
        """Return the host-side settings as text for a console line."""
        window = f"window {self.window_ms} ms"
        in_effect = round(self._window_s() * 1000.0)
        if in_effect != self.window_ms:
            window += f" ({in_effect} ms in effect)"  # the window was raised to the period
        return (
            f"{window}, lead {self.lead_ms} ms, deadband {self.deadband_mm_s:g} mm/s"
            f", long retraction {self.retract_min_mm:g} mm"
        )

    def _mode(self, lane: int) -> str | None:
        """Return the mode of the lane as the rate sees it.

        A follow whose final event has come counts as idle. Before the first report of the unit,
        the mode is None, and the console shows "unknown"."""
        mode: str | None = self.feed.lanes[lane]["mode"]
        return "idle" if mode == "following" and not self.feed.in_follow(lane) else mode

    def switch_line(self) -> str:
        """Return the console line that shows the switch of each lane."""
        states = " ".join(
            f"lane {lane + 1} {'on' if self.on[lane] else 'off'}"
            f" ({self.extruder_of(lane) or 'unmapped'}, {self._mode(lane) or 'unknown'})"
            for lane in range(LANES)
        )
        return f"{self.feed.unit.head()} feed-forward {states}"

    def not_following_lines(self, lanes: Iterable[int]) -> list[str]:
        """Return a console line for each lane that could follow but does not.

        ACEK_FEED_FORWARD ON=1 uses this on an image with the follow. The doses of such a lane
        start only after it follows, and without the line the switch would be silent while
        nothing moves. A lane qualifies if ACEK_LANE_ACTIVE can take it, which means:

        - it has filament in its bay and an extruder is mapped;
        - it is idle or in error (the line then says to run ACEK_CLEAR first);
        - no command is in flight and no other motion is open (a lane that is moving is on its
          way, and a mode that is not reported yet is not known);
        - no lane of any unit follows for that extruder, because only one lane of an extruder
          follows."""
        feed, unit = self.feed, self.feed.unit
        lines: list[str] = []
        for lane in lanes:
            name = self.extruder_of(lane)
            if not name or feed.lane_unavailable(lane, ("idle", "error")) is not None:
                continue
            busy = feed.open_seq(lane) is not None and not feed.follow_running(lane)
            if feed.motion[lane].inflight or busy or unit.units.followers_elsewhere(name, None):
                continue
            args = feed._lane_args(lane)
            arm = f"ACEK_LANE_ACTIVE {args} arms it"
            if self._mode(lane) == "error":
                arm = f"ACEK_CLEAR {args}, then {arm}"
            lines.append(
                f"{unit.head()} lane {unit.lane_text(lane)} is not in the follow: its"
                f" feed-forward doses start once it follows — {arm}"
            )
        return lines

    def _say(self, line: str) -> None:
        """Print a line on the console through the unit."""
        self.feed.unit.say(line)
