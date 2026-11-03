"""Turns the follow on by default: every lane with filament is armed, on any printer.

At the start of a session, every lane whose insert sensor sees filament is armed in the follow
(``ACEK_ASSIST DIR=BOTH``). A session starts at the first state report after ``klippy:ready``.
That is after a start of Klipper, a restart or a reset of the unit. Later, a lane whose filament
is inserted is armed too, once the lane is free.

Each lane has an *episode*. An episode opens in two cases.
- At the start of the session.
- At an insertion, when the insert sensor goes clear and then set while the lane is not in the
  tail of the follow. Filament pushed in at that time rests behind the old end, and nothing loads
  it.

An episode closes in these cases.
- This module armed the lane.
- The unit refused the arm for any reason except ``busy``.
- The lane is found in error.
- The filament leaves.
- Anything other than this module starts or stops the lane. That includes ``ACEK_STOP``,
  ``ACEK_ASSIST ... OFF=1``, a feed, an unload, a read with motion and the flow of a host module.

So a manual command wins until the filament is taken out and put back. Nothing is ever re-armed
under a move that someone else started.

The follow starts only on a lane that meets all of these conditions in the last state report.
- The lane is ``idle`` with no error.
- The insert sensor is set.
- No start is in flight on the lane.

After an insertion, the module waits ``INSERT_REPORTS`` reports first. The unit's own load
(``auto_load``) starts at the edge of the insert sensor. It must show in a report before this
module sends anything. The follow then waits for the end of that load.

The check runs on the reactor at each state report, so it reads the mode that the report brought.
It is never entered twice at the same time.

A host module can run its own flow on the insertions of a lane. It takes them over with
``set_lane_follow(lane, on_insert=False)``. The preload of the U1 adapter does this. Then the lane
is armed only at the start of the session, and the module arms it when its flow ends.
``set_lane_follow`` also sets the speed at which the lane is armed (``assist_speed`` by default).

    [ace2k]
    #follow_default: True       # arm the follow on every lane with filament; False: by hand only
    #follow_lanes: all          # all: every lane; extruder: only a lane that feeds an extruder
    #                           # (laneN_extruder, or the lane map a host module sets)

``follow_lanes: extruder`` leaves out a lane that feeds no extruder, so its filament is never
pushed when its plunger moves by hand. A host module can also keep the follow off a lane with
``hold_follow``, with a reason (a U1 adapter holds the lanes outside its ``lanes:``).

The module prints one console line for each arm (``ace2k: lane N follow on (follow_default)``)
and for each failure other than ``busy``.

The status of the lane shows ``follow_default`` as one of these.
- ``waiting``: an episode is open.
- ``arming``: the start is in flight.
- ``armed``: the follow is the current motion of the lane.
- ``off``.

It also shows ``follow_why``: why the module does not arm the lane (``why_not``), or None.
``ACEK_STATUS`` prints both on the lane's line (``follow_text``). A lane that follows by another
start than this module's reads ``armed (by <who>)`` there (FeedMotion.follow_starter), while
``follow_default`` stays this module's own state.

``ace2k_feed.py`` loads this module.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple, Protocol

try:
    from .ace2k_link import (
        LANE_ALL,
        LANES,
        NO_LANE,
        FeedRefused,
        check_lane,
    )
    from .ace2k_units import unprefixed
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANE_ALL,
        LANES,
        NO_LANE,
        FeedRefused,
        check_lane,
    )
    from ace2k_units import unprefixed  # type: ignore[import-not-found, no-redef]

FOLLOW_LANES = {
    "all": False,
    "extruder": True,
}  # the follow_lanes key: only a lane with an extruder
INSERT_REPORTS = 2  # state reports to wait after an insertion, so the unit's own load shows first
# The ends of a G-code move after which the follow of its lane is armed again. Only a clean end
# counts. An error does not. A stop does not, because ACEK_STOP keeps the lane off. A blocked end
# does not, because the filament is held somewhere and a follow would push against it. An unload
# does not, because the filament leaves.
REARM_KINDS = frozenset({"done", "loaded"})


class Rearm(NamedTuple):
    """The end of a G-code move that stopped the follow of a lane (see LaneFollow.rearm)."""

    what: str  # the move, as the console line names it
    seq: int  # the start of the move, registered before its query
    mode: str | None  # for a read with motion, the mode of the load the unit runs (seq 0)
    # True: arm the follow again at a clean end. False: a stop, so its end only lifts the hold.
    # "report": stopped meanwhile, so its end prints only the line, at an end that is not clean.
    rearm: bool | str


def _as_report(pending: Rearm | None) -> Rearm | None:
    """Return the rearm turned into "report" when it would arm again, else None."""
    if pending is not None and pending.rearm is True:
        return pending._replace(rearm="report")
    return None


if TYPE_CHECKING:
    from .ace2k_feed_motion import Event, LaneMotion
    from .ace2k_feed_proto import Move
    from .ace2k_klipper_types import Config, LaneStatus, Printer
    from .ace2k_units import UnitProto as CoordinatedUnit
    from .ace2k_units import Units

    class UnitProto(CoordinatedUnit, Protocol):
        """The unit (``ace2k.Ace2k``), as the follow reads its sensors and asks the coordinator."""

        @property
        def units(self) -> Units: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...  # the sensors of each lane
        def head(self, part: str | None = None) -> str: ...
        def say(self, line: str) -> None: ...

    class ExtruderMap(Protocol):
        """The feed-forward (``ace2k_ff.FeedForward``), as the follow reads a lane's extruder."""

        def extruder_of(self, lane: int) -> str | None: ...

    class FeedProto(Protocol):
        """The feed (``ace2k_feed.Ace2kFeed``) whose lanes the follow arms."""

        @property
        def unit(self) -> UnitProto: ...
        @property
        def present(self) -> bool: ...
        @property
        def has_follow(self) -> bool: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...
        @property
        def motion(self) -> list[LaneMotion]: ...
        @property
        def reports(self) -> int: ...
        @property
        def assist_speed_mm_s(self) -> float: ...
        @property
        def ff(self) -> ExtruderMap: ...
        def follow_running(self, index: int) -> bool: ...
        def open_seq(self, index: int) -> int | None: ...
        def follow_starter(self, index: int) -> str | None: ...
        def start_follow_as(self, owner: str, lane_index: int, speed_mm_s: float) -> Move: ...
        def start_move(
            self,
            lane_index: int,
            mode: str,
            length_mm: float,
            speed_mm_s: float,
            free_tip: bool = False,
        ) -> Move: ...


@dataclass
class LaneFollow:
    """What the default follow knows about one lane."""

    want: bool  # the episode of the lane is open (at first, the one from the start)
    on_insert: bool = True  # an insertion opens an episode (no module took it over)
    speed: float | None = None  # the speed a host module asked for; None means assist_speed
    last_insert: bool | None = None  # the insert sensor at the last check
    rise_report: int | None = None  # the report count at an insertion; None for the start episode
    move: Move | None = None  # the follow that this module armed, while it may still run
    arming: bool = False  # the start is in flight
    # The end of a G-code move that stopped the follow of the lane. The follow is armed again at
    # that clean end (REARM_KINDS).
    rearm: Rearm | None = None
    # The rearm of a move that the lost-event rule counted over (on_lost). Its final event may still
    # come, right after the report that counted it, and then it is the end of that move.
    lost_rearm: Rearm | None = None
    again: bool = False  # the next arm is that re-arm, and the line says "follow on again"
    # A G-code move is stopping the follow of the lane and has not registered its end yet.
    pausing: bool = False
    held: bool = False  # a module holds the lane, so nothing is armed (see hold_follow)
    held_why: str | None = None  # the reason the module gave for its hold, if any


class FollowDefault:
    def __init__(self, printer: Printer, feed: FeedProto, config: Config) -> None:
        self.printer = printer
        self.feed = feed
        self.enabled = config.getboolean("follow_default", True)
        # Every lane, or only a lane that feeds an extruder (see in_follow_lanes).
        self.only_fed = config.getchoice("follow_lanes", FOLLOW_LANES, "all")
        self.lanes = [LaneFollow(want=self.enabled) for _ in range(LANES)]
        self.ready = False
        self.running = False
        printer.register_event_handler("klippy:ready", self._on_ready)

    def _on_ready(self) -> None:
        self.ready = True
        self._units().note_ready()  # the lines about its groups, at the next tick

    # --- what the feed tells this module -------------------------------------------------------

    def note_start(self, lane: int) -> None:
        """Note a start on the lane that is not this module's.

        This applies once the start is sent, whatever the unit answers. The episode of the lane
        is over.
        """
        follow = self.lanes[lane]
        if not follow.arming:
            follow.want = False
            follow.again = False

    def note_stop(self, lane_index: int) -> None:
        """Note a stop of the lane, whoever sent it. LANE_ALL means every lane.

        The episode is over, and so is the pending re-arm of a G-code move.
        """
        for lane in range(LANES) if lane_index == LANE_ALL else [lane_index]:
            follow = self.lanes[lane]
            follow.want = False
            follow.again = False
            pending = follow.rearm
            self.end_pause(lane)
            # The paused move stopped. There is no re-arm and no hold. Its end still tells the
            # operator that the follow stays off.
            follow.rearm = _as_report(pending)
            # The same for a move counted over; one that would not arm again stays as it is.
            follow.lost_rearm = _as_report(follow.lost_rearm) or follow.lost_rearm

    def note_accepted(self, lane: int, seq: int) -> None:
        """Note that the unit accepted a start on the lane.

        A pending re-arm for another motion is then stale.
        """
        follow = self.lanes[lane]
        pending = follow.rearm
        if pending is not None and pending.seq != seq:
            follow.rearm = None
        follow.lost_rearm = None  # a new start: the end of a move counted over can no longer come

    def on_lost(self, lane: int, seq: int) -> None:
        """Note that the start `seq` ended without its final event.

        This is the lost-event rule of ``ace2k_feed``. The pause is over. Its rearm is kept as
        lost_rearm: the unit can send a due state report just before the final event of the same
        pass, and a final event with that seq that comes before the next report, with no start
        accepted meanwhile, still ends the move (on_final). A stop meanwhile cancels its re-arm,
        as it does a pending one (note_stop).
        """
        follow = self.lanes[lane]
        pending = follow.rearm
        if pending is not None and pending.seq == seq:
            follow.rearm = None
            follow.lost_rearm = pending

    def forget_lost(self, lane: int) -> None:
        """Forget the rearm of a move counted over: its final event did not come in time."""
        self.lanes[lane].lost_rearm = None

    def hold(self, lane: int, held: bool, why: str | None = None) -> None:
        check_lane(lane, NO_LANE)
        follow = self.lanes[lane]
        follow.held = bool(held)
        follow.held_why = why if held else None

    # --- the host interface ----------------------------------------------------------------------

    def set_lane(
        self, lane_index: int, speed_mm_s: float | None = None, on_insert: bool = True
    ) -> None:
        check_lane(lane_index, NO_LANE)
        if speed_mm_s is not None and not speed_mm_s > 0.0:
            raise ValueError(f"ace2k: a follow speed of {speed_mm_s} mm/s")
        follow = self.lanes[lane_index]
        follow.speed = speed_mm_s
        given_back = on_insert and not follow.on_insert
        follow.on_insert = bool(on_insert)
        if not on_insert and follow.rise_report is not None:
            follow.want = False  # an insertion episode is open, and now it belongs to the module
            follow.rise_report = None
        if given_back and not self.feed.follow_running(lane_index):
            # The module may have left an insertion unseen meanwhile. The lane is armed at the
            # next report if its filament is in and it is idle (see _check), as at the start.
            follow.want = True
            follow.again = False
            follow.rise_report = None

    def current(self, lane: int) -> Move | None:
        """Return the follow that this module armed on the lane, until its final event comes."""
        follow = self.lanes[lane]
        move = follow.move
        # The follow is gone if it is done, or over (its end was lost, see the rule in
        # ace2k_feed), or if a later start that the unit accepted took its place.
        # A refused start changes nothing.
        if move is not None and (move.done() or self.feed.open_seq(lane) != move.seq):
            follow.move = move = None
        return move

    # --- one active lane per extruder: the coordinator's job, across every unit ------------------

    def _units(self) -> Units:
        return self.feed.unit.units

    def groups(self) -> dict[str, list[int]]:
        """Return the lanes of this unit in a shared group, as {extruder: [lane, ...]}.

        A group can span several units (see ``ace2k_units``).
        """
        return self._units().unit_groups(self.feed.unit)

    def eligible(self, lane: int) -> bool:
        """Tell whether the lane may be followed by default.

        It may if it has its own extruder, or none, or if it is the lane chosen for its shared
        extruder.
        """
        return self._units().eligible(self.feed.unit, lane)

    def active_map(self) -> dict[str, int | str | None]:
        """Return the active lane of every shared extruder that a lane of this unit feeds.

        The map goes from the extruder to one of three values. It is the active lane number of
        this unit, or the active lane of another unit written as "b:3", or None if no lane is
        chosen.
        """
        return self._units().unit_active_map(self.feed.unit)

    def _say_groups(self) -> None:
        self._units().say_groups()

    # --- a G-code move on a lane in the follow -------------------------------------------------

    def in_follow_lanes(self, lane: int) -> bool:
        """Tell whether follow_lanes admits the lane.

        With follow_lanes: extruder, only a lane with an extruder (laneN_extruder, or the lane map
        a host module sets) is admitted."""
        return not self.only_fed or self.feed.ff.extruder_of(lane) is not None

    def _may_arm(self, lane: int) -> bool:
        """Tell whether the lane may be armed: eligible, and admitted by follow_lanes."""
        return self.eligible(lane) and self.in_follow_lanes(lane)

    def will_rearm(self, lane: int) -> bool:
        """Tell whether a follow paused for a G-code move comes back at its clean end."""
        return self.enabled and self._may_arm(lane)

    def begin_pause(self, lane: int) -> None:
        """Note that a G-code move is stopping the follow of the lane (is_paused is then true)."""
        self.lanes[lane].pausing = True

    def paused(
        self, lane: int, what: str, seq: int, mode: str | None = None, rearm: bool = True
    ) -> None:
        """Note that the G-code move that stopped the follow of the lane is sent as start `seq`.

        The pause lasts until the clean end of the motion with that seq, and that mode if one is
        given. Only that end arms the follow again, and only if `rearm` is true.
        """
        follow = self.lanes[lane]
        follow.pausing = False
        follow.rearm = Rearm(what, seq, mode, rearm)
        follow.lost_rearm = None

    def end_pause(self, lane: int) -> None:
        follow = self.lanes[lane]
        follow.pausing = False
        follow.rearm = None

    def is_paused(self, lane: int) -> bool:
        follow = self.lanes[lane]
        pending = follow.rearm
        return follow.pausing or (pending is not None and pending.rearm != "report")

    def give_back(self, lane: int) -> None:
        """Note that the paused move did not start.

        The follow comes back at the next report that shows the lane idle, with filament and no
        error.
        """
        follow = self.lanes[lane]
        self.end_pause(lane)
        follow.want = True
        follow.again = True
        follow.rise_report = None

    def on_final(self, lane: int, event: Event) -> None:
        """Handle a final event of the lane. This runs on the reactor.

        The end of the paused move re-arms the follow, or not.
        """
        follow = self.lanes[lane]
        pending = follow.rearm
        if pending is None:
            pending = follow.lost_rearm  # a move counted over whose end came after all
        if pending is None:
            return
        what, seq, mode, rearm = pending
        if event.get("seq") != seq or (mode is not None and event.get("mode") != mode):
            return  # not the end of this move (a late end of the follow, or an older move)
        follow.rearm = follow.lost_rearm = None
        if not rearm:
            return  # a stop and not a pause, so the end of the move only lifts the hold
        n = lane + 1
        if rearm == "report":  # stopped meanwhile: print only the line, at an end that is not clean
            if event.get("kind") not in REARM_KINDS:
                self._say(
                    f"ace2k: lane {n} follow stays off — the {what} ended {event.get('kind')}"
                )
            return
        if event.get("kind") in REARM_KINDS:
            follow.want = True  # armed at the next report that shows the lane idle (see _check)
            follow.again = True
            follow.rise_report = None
            return
        self._say(f"ace2k: lane {n} follow stays off — the {what} ended {event.get('kind')}")

    def why_not(self, lane: int) -> str | None:
        """Return why this module does not arm the lane, or None when nothing keeps it off.

        The reasons, in this order: follow_default off, a host module's hold, follow_lanes, and a
        shared extruder whose active lane is another or none."""
        if not self.enabled:
            return "follow_default: False"
        follow = self.lanes[lane]
        if follow.held:
            return f"held: {follow.held_why}" if follow.held_why else "held by a host module"
        if not self.in_follow_lanes(lane):
            return "feeds no extruder, follow_lanes: extruder"
        if self.eligible(lane):
            return None
        name = self.feed.ff.extruder_of(lane)
        active = self.active_map().get(name or "")
        if active is None:
            return f"{name}: no lane chosen, ACEK_LANE_ACTIVE"
        return f"{name}: lane {active} is its active lane"

    def follow_text(self, lane: int) -> str:
        """Return the follow of the lane for ACEK_STATUS: its state, or off with the reason.

        A lane that follows by another start than this module's reads "armed (by <who>)", who as
        FeedMotion.follow_starter names it. After a shutdown that text is not given: the last
        report's mode is stale then.
        """
        state = self.lane_state(lane)
        if state not in ("armed", "arming") and not self.printer.is_shutdown():
            starter = self.feed.follow_starter(lane)
            if starter is not None:
                return f"armed (by {starter})"
        why = None if state in ("armed", "arming") else self.why_not(lane)
        return state if why is None else f"off ({why})"

    def lane_state(self, lane: int) -> str:
        if not self.enabled:
            return "off"
        follow = self.lanes[lane]
        if follow.arming:
            return "arming"
        if self.current(lane) is not None:
            return "armed"
        return "waiting" if follow.want else "off"

    # --- the check, at each state report (on the reactor) -----------------------------------------

    def tick(self, _eventtime: float | None = None) -> None:
        feed = self.feed
        if not (self.enabled and self.ready) or self.running:
            return
        if not feed.present or not feed.has_follow:
            return
        self.running = True
        try:
            if self.printer.is_shutdown():
                return  # the modes are stale, so nothing may be sent
            self._say_groups()
            for lane in range(LANES):
                self._check(lane, feed.unit.lanes[lane].get("insert"), feed.lanes[lane])
        except Exception:  # never let an error reach the reactor
            logging.exception("%s follow_default: the check failed", self.feed.unit.head())
        finally:
            self.running = False

    def _check(self, lane: int, insert: bool | None, status: LaneStatus) -> None:
        if not self._note_insert(lane, insert, status):
            return
        if not self.lanes[lane].want or not self._may_arm(lane):
            return  # a lane of a shared extruder waits for ACEK_LANE_ACTIVE, or feeds none
        if self._ready_to_arm(lane, status):
            self._arm(lane)

    def _note_insert(self, lane: int, insert: bool | None, status: LaneStatus) -> bool:
        """Keep the insert sensor's record of the lane, and open the episode of an insertion.

        Return True when the filament is in and no module holds the lane, so the arm gate is
        checked next."""
        follow = self.lanes[lane]
        before = follow.last_insert
        if insert is not None:
            follow.last_insert = insert
        if insert is not True:
            if insert is False:
                follow.want = False  # the filament left, so there is no episode until it is back
                follow.rise_report = None
                self.end_pause(lane)  # and no re-arm of a move, because that filament has gone
            return False
        if follow.held:
            return False  # a module holds the lane, and its filament waits behind another piece
        if before is False and follow.on_insert and status["mode"] != "following":
            # An insertion. Inside the follow, it belongs to the tail of the follow, which ignores
            # the bay.
            follow.want = True
            follow.again = False
            follow.rise_report = self.feed.reports
        return True

    def _ready_to_arm(self, lane: int, status: LaneStatus) -> bool:
        """Tell whether the last report lets the open episode of the lane arm it now.

        An error closes the episode. Another mode, a start in flight, or the reports still to
        wait after an insertion only defer it."""
        follow = self.lanes[lane]
        mode = status["mode"]
        if mode == "error" or status.get("error") is not None:
            # Never arm over an error. A reinsertion or a restart opens the episode again.
            follow.want = False
            return False
        if mode != "idle" or self.feed.motion[lane].inflight:
            return False  # another mode is active (the unit's own load or a move): wait its end
        rise = follow.rise_report
        return rise is None or self.feed.reports >= rise + INSERT_REPORTS

    def _arm(self, lane: int) -> None:
        follow = self.lanes[lane]
        feed = self.feed
        n = lane + 1
        speed = follow.speed or feed.assist_speed_mm_s
        follow.arming = True
        try:
            move = feed.start_follow_as("follow_default", lane, speed)
        except FeedRefused as err:
            if err.reason == "busy":
                return  # the report was behind the unit, so try again at a later report
            self._closed(lane, err.reason)
            return
        except (RuntimeError, ValueError) as err:  # no answer, no report, or the speed was refused
            self._closed(lane, unprefixed(err))
            return
        finally:
            follow.arming = False
        follow.want = False
        follow.rise_report = None
        follow.move = move
        again, follow.again = follow.again, False
        self._say(f"ace2k: lane {n} follow on " + ("again" if again else "(follow_default)"))

    def armed_by_command(self, lane: int, move: Move) -> None:
        """Note that ACEK_LANE_ACTIVE armed the lane.

        This module adopts the follow, so a module that tracks the motions of the lane can take
        it over. The episode of the lane is over.
        """
        follow = self.lanes[lane]
        follow.want = False
        follow.rise_report = None
        follow.move = move

    def _closed(self, lane: int, why: str) -> None:
        follow = self.lanes[lane]
        follow.want = False
        follow.rise_report = None
        self._say(f"ace2k: lane {lane + 1} follow not armed (follow_default) — {why}")

    def _say(self, line: str) -> None:
        self.feed.unit.say(line)  # this also logs the line
