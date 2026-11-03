"""Calibrates the encoder of a lane automatically, using the extruder of the lane as the reference.

The follow feeds the filament. The extruder pulls the plunger from rest back to taut, at the speed
of the extrusion. A taut is a sharp and repeatable plunger position. Between two of them, the
filament that passes the encoder is exactly what the extruder took.

A mark pairs two readings. It is made at each increase of the taut count of the unit
(``ace2k_feed_ff_state``). The switch itself closes for less than one report, so the count is
used. The mark pairs the encoder reading with the position of the extruder. The encoder must
have been still since the burst before: the last two readings before the mark are equal.
A least-squares line through the marks gives the slope. The new scale is the scale in use
divided by that slope.

``ACEK_CALIBRATE_ENCODER LANE=n AUTO=1 [LENGTH=500] [SPEED=4]`` runs it.

Before anything is sent, the command refuses to run, and gives the reason, in these cases.
- The firmware image has no feed-forward.
- A print is printing or paused.
- The lane has no extruder.
- That extruder is not the active one, or it is too cold to extrude.
- The lane has no filament, is in error, or is busy in a mode other than idle and the follow.
- A run is already going.

A run does these steps.
1. It arms the follow if the lane is idle, at ``assist_speed`` as ``ACEK_ASSIST DIR=BOTH`` does.
2. It holds the feed-forward of the lane, so the follow alone feeds.
3. It raises both reports to 10 Hz.
4. It extrudes LENGTH in chunks of 25 mm, between SAVE_GCODE_STATE and RESTORE_GCODE_STATE. Only
   the extruder moves.

Each chunk is queued after the last one has run. So an abort lets the running chunk end and queues
no other. These conditions abort the run.
- A full: the shared switch is set, or the full count of the lane rises.
- The lane is in error or out of the follow.
- An epoch change.
- A shutdown.

Every ending does these things.
- It releases the hold.
- It puts the reports back at their own rates.
- It stops the follow, but only if the run armed it.
- It restores the G-code state. After a shutdown it sends nothing and runs no G-code, because
  Klipper refuses G-code then, and the state does not outlive the restart.

Then the marks are judged. The scale is either applied or refused. An applied scale is in use at
once and staged for SAVE_CONFIG. A refusal says "nothing changed".
"""

from __future__ import annotations

import collections
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Protocol, TypedDict, Union

try:
    from .ace2k_link import LANE_REPORT_S, FeedRefused
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_link import LANE_REPORT_S, FeedRefused  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_feed_proto import Move
    from .ace2k_klipper_types import (
        MCU,
        Extruder,
        GCode,
        GCodeCommand,
        LaneStatus,
        Printer,
        PrintStats,
        ToolHead,
    )

    class FeedForwardProto(Protocol):
        """The feed-forward (``ace2k_ff.FeedForward``), as the run holds the dosing of a lane."""

        def hold(self, lane: int, held: bool) -> None: ...
        def extruder_of(self, lane: int) -> str | None: ...

    class UnitProto(Protocol):
        """The unit (``ace2k.Ace2k``), as the run raises its lane counters and reads the switch."""

        @property
        def pulled_any(self) -> bool | None: ...
        def set_counters_period(self, seconds: float) -> None: ...

    class FeedProto(Protocol):
        """The feed (``ace2k_feed.Ace2kFeed``) that the calibration runs on."""

        @property
        def lanes(self) -> list[LaneStatus]: ...
        @property
        def ff(self) -> FeedForwardProto: ...
        @property
        def unit(self) -> UnitProto: ...
        @property
        def has_ff(self) -> bool: ...
        @property
        def scale_low(self) -> float: ...
        @property
        def scale_high(self) -> float: ...
        @property
        def assist_speed_mm_s(self) -> float: ...
        @property
        def report_period_ms(self) -> int: ...
        def lane_unavailable(
            self, index: int, allow: tuple[str, ...] | None = ("idle",)
        ) -> str | None: ...
        def not_active_text(self, index: int) -> str | None: ...
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
        def apply_scale(self, index: int, scale: float) -> None: ...
        def set_counts_period(self, seconds: float) -> None: ...


# A frame that the serial thread queues for a run. It is either a reading of the lane counters
# (receive time, encoder mm) or a counters frame (receive time, taut count of the lane, epoch).
Frame = Union[tuple[Literal["reading"], float, float], tuple[Literal["counts"], float, int, int]]
Point = tuple[float, float]  # (x, y); a mark is (extruder mm, encoder mm)

COUNT_MASK = 0xFFFF  # the taut count of the unit is a uint16
MIN_MARKS = 10  # with fewer marks, the line rests on too little travel to defend 0.1 %
MAX_SCATTER_MM = 1.0  # an RMS distance from the line above this means the filament slipped
# A larger change is applied with a warning. The extruder may be off its own calibration.
WARN_CHANGE = 0.03
CHUNK_MM = 25.0  # the step of the extrusion; an abort stops at the end of the running chunk
LENGTH_MM = (500.0, 200.0, 2000.0)  # default, minimum, maximum
SPEED_MM_S = 4.0
REPORT_S = 0.1  # period of the feed-forward counters and the lane counters (10 Hz) during the run
POLL_S = 0.25  # how often the frames become marks; the history of the extruder is short
# The state report must show the arm of the follow within this time. At a slower feed_report_hz,
# the time is two periods of the report plus ARM_WAIT_MARGIN_S, because one report may be lost.
ARM_WAIT_S = 3.0
ARM_WAIT_MARGIN_S = 0.5
GCODE_STATE = "ACE2K_ENCODER_CAL"  # the name for SAVE_GCODE_STATE around the run
SHUTDOWN = "Klipper shut down"  # reason for an abort
FULL = "the buffer went full"  # reason for an abort
STILL_READINGS = 2  # for a still encoder, the last two readings before a mark are equal


class Verdict(TypedDict, total=False):
    """The verdict that ``judge`` gives on the marks of a run.

    The docstring of ``judge`` says which keys are present, and when.
    The keys are read by name, which vulture does not see. The noqa on a key says so.
    """

    n: int
    refusal: str | None  # noqa: V107
    warn: bool  # noqa: V107
    scatter: float  # noqa: V107
    span_e: float  # noqa: V107
    span_enc: float  # noqa: V107
    new: float
    change: float


class EpochChanged(Exception):  # noqa: N818 — a condition of the run, not an error class
    """Raised when the lane re-entered a mode during the run, so its counters restarted."""


class MarkTracker:
    """Turns the frames of a run, in the order they arrive, into marks.

    ``reading(pt, encoder_mm)`` takes a reading of the lane counters.
    ``counts(pt, taut, epoch)`` takes a counters frame with the taut count and the epoch of the
    lane. It returns the mark that the frame closes, as (instant, encoder_mm), or None.
    """

    def __init__(self) -> None:
        # (pt, encoder_mm), since the frame in which the last episode was seen
        self.readings: list[Point] = []
        self.prev: tuple[float, int, int] | None = None  # (pt, taut, epoch) of the last frame
        # The frame in which the last episode was seen. Older readings may predate its burst.
        self.after: float | None = None
        self.episodes = 0  # taut episodes seen during the run
        self.dropped = 0  # marks dropped: two increments in one interval, or no still reading

    def reading(self, pt: float, encoder_mm: float) -> None:
        self.readings.append((pt, encoder_mm))

    def counts(self, pt: float, taut: int, epoch: int) -> Point | None:
        prev, self.prev = self.prev, (pt, taut, epoch)
        if prev is None:
            return None  # this frame is the baseline
        if epoch != prev[2]:
            raise EpochChanged()
        step = (taut - prev[1]) & COUNT_MASK
        if not step:
            return None
        lo, after = prev[0], self.after
        self.episodes += step
        encoder = self._still_before(lo, after)
        self.after = pt
        self.readings = [r for r in self.readings if r[0] > pt]
        if self.episodes == step:
            self.dropped += step - 1  # any other episodes in its interval are dropped marks
            return None  # first episode of the run: the start position of the plunger is unknown
        if step > 1 or encoder is None:
            self.dropped += step
            return None
        return ((lo + pt) / 2.0, encoder)

    def _still_before(self, lo: float, after: float | None) -> float | None:
        """Return the last reading before `lo`, if it equals the one before it.

        Both readings must be after `after`. Otherwise return None.
        """
        before = [r for r in self.readings if r[0] < lo and (after is None or r[0] > after)]
        if len(before) < STILL_READINGS:
            return None
        (_, a), (_, b) = before[-2], before[-1]
        return b if a == b else None


def fit_line(points: Sequence[Point]) -> tuple[float, float, float]:
    """Fit a least-squares line of y on x through the points (x, y).

    Return the slope, the intercept and the RMS of the residuals. The fit needs at least two points
    with different x. If every x is equal there is no line, and the slope is 0.
    """
    n = len(points)
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    sxx = sum((x - mx) ** 2 for x, _ in points)
    if sxx <= 0.0:
        return 0.0, my, 0.0
    slope = sum((x - mx) * (y - my) for x, y in points) / sxx
    icpt = my - slope * mx
    rms = math.sqrt(sum((y - (icpt + slope * x)) ** 2 for x, y in points) / n)
    return slope, icpt, rms


def judge(
    marks: Sequence[Point], scale: float, low: float, high: float, dropped: int | None = None
) -> Verdict:
    """Judge the marks of a run and return a verdict.

    `marks` are pairs (extruder_mm, encoder_mm) for a lane at `scale` (mm per count). The window
    `low` to `high` is the range of scales that the unit accepts.

    The verdict is a dict with these keys.
    - ``n``, ``refusal`` and ``warn``. ``refusal`` is None when the scale may be applied.
    - ``scatter``, ``span_e`` and ``span_enc``, if there are enough marks for a line.
    - ``new`` and ``change``, unless the slope is 0 or below. Then the encoder did not follow,
      and the run is refused.

    Each check uses the value as it is shown and sent, so no printed line can contradict the
    verdict. ``new`` is rounded to 4 decimals. It is staged so, and sent in units of 0.1 µm per
    count. ``change`` is computed from that ``new`` and rounded to 0.01 %. The scatter is rounded
    to 0.01 mm.

    `dropped` is optional (``MarkTracker.dropped``). The refusal for too few marks names it. Many
    taut episodes with dropped marks point at the encoder, not at the extruder.
    """
    n = len(marks)
    verdict = Verdict(n=n, refusal=None, warn=False)
    if n < MIN_MARKS:
        noun = "mark" if n == 1 else "marks"
        lost = "" if dropped is None else f" ({dropped} dropped)"
        verdict["refusal"] = (
            f"{n} {noun}, at least {MIN_MARKS} needed{lost} — is the extruder gripping the"
            " filament? a larger LENGTH gives more"
        )
        return verdict
    slope, _icpt, rms = fit_line(marks)
    verdict.update(
        {
            "scatter": rms,
            "span_e": marks[-1][0] - marks[0][0],
            "span_enc": marks[-1][1] - marks[0][1],
        }
    )
    if slope <= 0.0:
        verdict["refusal"] = "the encoder did not follow the extruder"
        return verdict
    new = round(scale / slope, 4)  # the scale as it is staged and sent
    change = new / scale - 1.0
    verdict.update({"new": new, "change": change})
    shown = round(rms, 2)  # so a refusal never reads 1.00 mm
    if shown > MAX_SCATTER_MM:
        verdict["refusal"] = (
            f"scatter {shown:.2f} mm, more than {MAX_SCATTER_MM:g} mm: the filament slipped"
        )
    elif not low <= new <= high:
        verdict["refusal"] = (
            f"{new:.4f} mm/count is outside the unit's window {low:.4f}..{high:.4f}"
        )
    else:
        # A change that shows as +3.00 % never carries the warning.
        verdict["warn"] = abs(round(change * 100.0, 2)) > WARN_CHANGE * 100.0
    return verdict


class _Sink:
    """What the serial thread collects for one run.

    It holds the lane, the frames of the run and the full count of the lane. The full count is
    the pair (epoch, count) of the first counters frame of the run. ``full_seen`` says whether a
    later frame of that epoch carried another count, which means the follow took up a full during
    the run.

    The baseline is the first frame of the run, not the status at its start. A run that armed the
    follow restarted the counters of the unit, and the status may still show the session before.
    """

    __slots__ = ("lane", "frames", "full", "full_seen")

    def __init__(self, lane: int, frames: collections.deque[Frame]) -> None:
        self.lane = lane
        self.frames = frames
        self.full: tuple[int, int] | None = None
        self.full_seen = False

    def note_full(self, epoch: int, count: int) -> None:
        """Note the full count of a counters frame.

        Only counts in the epoch of the baseline matter. A count that a new epoch restarted is the
        re-entry on which the tracker aborts. It is not a full.
        """
        if self.full is None:
            self.full = (epoch, count)
        elif epoch == self.full[0] and count != self.full[1]:
            self.full_seen = True


@dataclass
class _Run:
    """What the polls of one run work on.

    It holds the sink, the extruder that the marks are read off, the tracker that turns frames
    into marks, and the marks so far.
    """

    sink: _Sink
    extruder: Extruder
    tracker: MarkTracker
    marks: list[Point]


class EncoderCalibration:
    """Implements ``ACEK_CALIBRATE_ENCODER AUTO=1``.

    The frames reach it on the serial thread while a run holds a lane. ``run()`` drives the run on
    the reactor.
    """

    def __init__(self, printer: Printer, feed: FeedProto) -> None:
        self.printer = printer
        self.feed = feed
        self.reactor = printer.get_reactor()
        # The frames are ("counts", rt, taut, epoch) or ("reading", rt, mm).
        self.frames: collections.deque[Frame] = collections.deque()
        # The _Sink of the run while a run holds a lane, and None between runs. It is one
        # attribute, so the serial thread reads the lane and the frames of its run in one go.
        # Each run collects into its own sink and its own deque. So a frame read during one run
        # goes to that run, never to the next one.
        self._sink: _Sink | None = None

    @property
    def lane(self) -> int | None:
        """The lane that a run holds, or None between runs."""
        sink = self._sink
        return None if sink is None else sink.lane

    @lane.setter
    def lane(self, lane: int | None) -> None:
        self._sink = None if lane is None else _Sink(lane, self.frames)

    def on_ff_state(self, receive_time: float, taut: Sequence[int], epoch: Sequence[int]) -> None:
        sink = self._sink
        if sink is not None:
            lane = sink.lane
            # Note the full count of the lane for this same frame (ace2k_feed stores the counters
            # first) before the frame is queued. Once it is queued, a poll may drain it at any
            # moment.
            sink.note_full(epoch[lane], self.feed.lanes[lane]["ff_full_fixes"])
            sink.frames.append(("counts", receive_time, taut[lane], epoch[lane]))

    def on_counters(self, receive_time: float, encoder_um: Sequence[int]) -> None:
        sink = self._sink
        if sink is not None:
            sink.frames.append(("reading", receive_time, encoder_um[sink.lane] / 1000.0))

    # --- the run (on the reactor) -----------------------------------------------------------

    def run(self, gcmd: GCodeCommand, index: int) -> None:
        """Run ``ACEK_CALIBRATE_ENCODER AUTO=1`` (with LENGTH and SPEED) on lane `index`, 0 to 3."""
        default, least, most = LENGTH_MM
        length = gcmd.get_float("LENGTH", default, minval=least, maxval=most)
        speed = gcmd.get_float("SPEED", SPEED_MM_S, above=0.0)
        extruder = self._check(gcmd, index)
        n = index + 1
        # The scale of the readings, for the whole run.
        scale = self.feed.lanes[index]["encoder_scale"]
        gcmd.respond_info(
            f"ace2k: lane {n}: calibrating against {extruder.get_name()} — {length:g} mm at"
            f" {speed:g} mm/s"
        )
        tracker = MarkTracker()
        marks: list[Point] = []
        reason = None
        armed = held = listening = False
        try:
            if self.feed.lanes[index]["mode"] != "following":
                self._arm(gcmd, index)
                armed = True
                self._wait_following(gcmd, index)
            if self.printer.is_shutdown():
                reason = SHUTDOWN  # during the wait for the arm, so nothing more goes out
            else:
                held = True
                self.feed.ff.hold(index, True)
                listening = True
                run = _Run(self._listen(index), extruder, tracker, marks)
                reason = self._extrude(run, length, speed)
        finally:
            # Every ending does this before any verdict. A scale then goes out with the reports back
            # at their own rates and the follow as the user had it. Nothing is sent after a
            # shutdown.
            if listening:
                self._unlisten()
            if held:
                self.feed.ff.hold(index, False)
            if armed and not self.printer.is_shutdown():
                self.feed.stop(index)
        if reason is not None:
            raise gcmd.error(f"ace2k: lane {n}: aborted — {reason}; nothing changed")
        v = judge(marks, scale, self.feed.scale_low, self.feed.scale_high, tracker.dropped)
        if "span_e" in v:
            gcmd.respond_info(
                f"ace2k: lane {n}: {v['n']} marks over {v['span_e']:.1f} mm of extruder; the"
                f" encoder read {v['span_enc']:.1f} mm"
            )
        if v["refusal"] is not None:
            raise gcmd.error(f"ace2k: lane {n}: {v['refusal']}; nothing changed")
        self.feed.apply_scale(index, v["new"])
        gcmd.respond_info(
            f"ace2k: lane {n}: scale {scale:.4f} → {v['new']:.4f} mm/count"
            f" ({v['change'] * 100.0:+.2f} %), scatter {v['scatter']:.2f} mm — in use now;"
            " SAVE_CONFIG to keep it"
        )
        if v["warn"]:
            gcmd.respond_info(
                f"ace2k: lane {n}: a change above {WARN_CHANGE * 100.0:g} % — check the"
                " extruder's own calibration too"
            )

    def _check(self, gcmd: GCodeCommand, index: int) -> Extruder:
        """Return the extruder to calibrate against.

        If something stands in the way, raise a G-code error that names it. This happens before
        anything is sent.
        """
        n = index + 1
        if self.lane is not None:
            raise gcmd.error("ace2k: a calibration is already running")
        if not self.feed.has_ff:
            raise gcmd.error(
                "ace2k: the unit's firmware has no feed-forward (its taut count); flash v0.9.0"
            )
        stats: PrintStats | None = self.printer.lookup_object("print_stats", None)
        if stats is not None:
            state = stats.get_status(self.reactor.monotonic()).get("state")
            if state in ("printing", "paused"):
                raise gcmd.error(f"ace2k: a print is {state}; calibrate between prints")
        name = self.feed.ff.extruder_of(index)
        if not name:
            raise gcmd.error(f"ace2k: lane {n} has no extruder (lane{n}_extruder)")
        extruder: Extruder | None = self.printer.lookup_object(name, None)
        if extruder is None or not hasattr(extruder, "find_past_position"):
            raise gcmd.error(f"ace2k: lane {n}: no extruder {name}")
        toolhead: ToolHead = self.printer.lookup_object("toolhead")
        if toolhead.get_extruder() is not extruder:
            raise gcmd.error(f"ace2k: {name} is not the active extruder; select it first")
        if not extruder.get_heater().can_extrude:
            raise gcmd.error(f"ace2k: {name} is too cold to extrude; heat it for the filament")
        lane = self.feed.lanes[index]
        unavailable = self.feed.lane_unavailable(index, ("idle", "following"))
        if unavailable == "no_filament":
            raise gcmd.error(f"ace2k: lane {n} has no filament")
        if unavailable == "error":
            raise gcmd.error(f"ace2k: lane {n} is in error ({lane['error']}); ACEK_CLEAR first")
        if unavailable == "busy":
            raise gcmd.error(f"ace2k: lane {n} is busy ({lane['mode']})")
        # The run follows the lane, so it must be the active lane.
        why = self.feed.not_active_text(index)
        if why is not None:
            raise gcmd.error(why)
        return extruder

    def _arm(self, gcmd: GCodeCommand, index: int) -> None:
        """Arm the follow on the idle lane, at assist_speed (the default of ACEK_ASSIST DIR=BOTH).

        If the unit refuses, or the host cannot start it, raise a G-code error, as ACEK_ASSIST does.
        """
        try:
            self.feed.start_follow_as("ACEK_CALIBRATE_ENCODER", index, self.feed.assist_speed_mm_s)
        except (FeedRefused, RuntimeError) as e:
            raise gcmd.error(str(e)) from e

    def _wait_following(self, gcmd: GCodeCommand, index: int) -> None:
        """Wait until the state report shows the arm of the follow.

        The wait lasts ARM_WAIT_S, or two periods of the report plus ARM_WAIT_MARGIN_S if that is
        longer. After that, raise a G-code error. A shutdown ends the wait. The run then aborts, and
        nothing more is sent.
        """
        period_s = self.feed.report_period_ms / 1000.0
        deadline = self.reactor.monotonic() + max(ARM_WAIT_S, 2 * period_s + ARM_WAIT_MARGIN_S)
        while self.feed.lanes[index]["mode"] != "following":
            if self.printer.is_shutdown():
                return
            now = self.reactor.monotonic()
            if now >= deadline:
                raise gcmd.error(f"ace2k: lane {index + 1} did not enter the follow")
            self.reactor.pause(now + 0.1)

    def _listen(self, index: int) -> _Sink:
        """Start to collect the frames of the lane, and return the sink of the run.

        The frames go into a sink and a deque that belong to this run. Both reports are set to
        10 Hz.
        """
        self.frames = collections.deque()
        sink = self._sink = _Sink(index, self.frames)  # from here the run holds the lane
        if not self.printer.is_shutdown():
            self.feed.set_counts_period(REPORT_S)
            self.feed.unit.set_counters_period(REPORT_S)
        return sink

    def _unlisten(self) -> None:
        """Stop collecting frames, and put both reports back at their own rates."""
        self.lane = None
        if not self.printer.is_shutdown():
            self.feed.set_counts_period(self.feed.report_period_ms / 1000.0)
            self.feed.unit.set_counters_period(LANE_REPORT_S)

    def _extrude(self, run: _Run, length: float, speed: float) -> str | None:
        """Extrude `length` in chunks of CHUNK_MM, and turn the frames into marks meanwhile.

        The G-code state is saved first. Each chunk is queued after the last one has run.
        Return the reason for an abort, or None. A condition seen while a chunk runs lets that
        chunk end and queues no other. A condition present before the first chunk queues none.
        """
        gcode: GCode = self.printer.lookup_object("gcode")
        toolhead: ToolHead = self.printer.lookup_object("toolhead")
        clock: MCU = self.printer.lookup_object("mcu")  # converts to print time

        def poll() -> str | None:
            return self._poll(run, clock)

        gcode.run_script_from_command(f"SAVE_GCODE_STATE NAME={GCODE_STATE}\nM83")
        try:
            done = 0.0
            reason = poll()
            while reason is None and done < length - 1e-6:
                step = min(CHUNK_MM, length - done)
                gcode.run_script_from_command(f"G1 E{step:.3f} F{speed * 60.0:.1f}")
                done += step
                end_pt = toolhead.get_last_move_time()
                while reason is None:
                    now = self.reactor.monotonic()
                    if clock.estimated_print_time(now) >= end_pt:
                        break
                    self.reactor.pause(now + POLL_S)
                    reason = poll()
            toolhead.wait_moves()
            if reason is None:
                # Wait for the last frames.
                self.reactor.pause(self.reactor.monotonic() + 2 * REPORT_S)
                reason = poll()
        finally:
            # Not after a shutdown. Klipper refuses G-code then, and its error would replace the
            # line of the abort. The G-code state does not outlive the restart anyway.
            if not self.printer.is_shutdown():
                gcode.run_script_from_command(f"RESTORE_GCODE_STATE NAME={GCODE_STATE}")
        return reason

    def _poll(self, run: _Run, clock: MCU) -> str | None:
        """Check the abort conditions, then turn every frame received so far into marks.

        The position of the extruder is looked up as each frame arrives. Then check the full count
        of the lane again. Return the reason for an abort, or None.
        """
        sink = run.sink
        n = sink.lane + 1
        reason = self._abort_reason(sink, n)
        if reason is not None:
            return reason
        frames = sink.frames
        while frames:
            frame = frames.popleft()
            pt = clock.estimated_print_time(frame[1])
            if frame[0] == "reading":
                run.tracker.reading(pt, frame[2])
                continue
            try:
                mark = run.tracker.counts(pt, frame[2], frame[3])
            except EpochChanged:
                return f"lane {n} re-entered a mode"
            if mark is not None:
                instant, encoder_mm = mark
                run.marks.append((run.extruder.find_past_position(instant), encoder_mm))
        # Check again. A frame queued after the first check was drained with the rest. Its full
        # count was noted before it was queued (see on_ff_state). So no drained frame carries a
        # full that was not seen. This also holds at the last poll, just before the verdict.
        if sink.full_seen:
            return FULL
        return None

    def _abort_reason(self, sink: _Sink, n: int) -> str | None:
        """Return what aborts the run before its frames are drained, or None.

        A shutdown, a full, or the lane being in error or out of the follow aborts the run.
        """
        if self.printer.is_shutdown():
            return SHUTDOWN
        # The switch is sampled at 10 Hz, but the full count of the lane is exact. A full that the
        # follow takes up within one tick shows only in the count.
        if self.feed.unit.pulled_any or sink.full_seen:
            return FULL
        lane = self.feed.lanes[sink.lane]
        if lane["mode"] == "error":
            return f"lane {n} in error ({lane['error']})"
        if lane["mode"] != "following":
            return f"lane {n} left the follow ({lane['mode']})"
        return None
