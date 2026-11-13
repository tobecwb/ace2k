"""The lane motions on the host: what the host knows about each lane, and the starts and stops.

``LaneMotion`` is the record of one lane. It holds the sequence number of the last start that was
sent, whether a start is in flight, and who sent the stop that the running motion answers. It also
holds the last start that the unit accepted and whether its end has come, what the last state
report showed, the lengths of the starts that are still running, the last final event, and the
waits that a final event completes. ``Ace2kFeed.motion`` holds one record for each lane.

``FeedMotion`` is the part of ``Ace2kFeed`` that moves the lanes. It has the handlers of the state
report and of the events, and the host interface calls ``start_move``, ``set_speed``, ``stop`` and
``clear``. It answers questions about the record (``open_seq``, ``in_follow``, ``is_following``
and ``follow_running``). ``with_follow_paused`` pauses the follow of a lane before a G-code sends a
start. It also has the move G-codes ``ACEK_FEED``, ``ACEK_ROLLBACK``, ``ACEK_UNLOAD``,
``ACEK_LOAD``, ``ACEK_STOP``, ``ACEK_CLEAR`` and ``ACEK_SPEED``.

``Ace2kFeed`` inherits it. The attributes that both parts use are declared in ``FeedBase``
(``ace2k_feed_base.py``), and the others at the top of the class. This module imports only
``ace2k_feed_base``, ``ace2k_link`` and ``ace2k_feed_proto``. ``ace2k_feed.py`` re-exports the
names that other modules and tests import from there.
"""

from __future__ import annotations

import logging
import math
import struct
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Protocol, TypeVar, cast

try:
    from .ace2k_feed_proto import (
        FAILED_KIND_NAMES,
        KIND_NAMES,
        MODE_NAMES,
        NO_STOP,
        NOTICE_KINDS,
        REFUSAL_NAMES,
        STALE_SENSORS_KIND,
        STALE_SENSORS_TEXT,
        STOP_KINDS,
        Move,
        to_mm,
        to_um,
    )
    from .ace2k_link import (
        LANE_ALL,
        LANES,
        NO_FEED,
        NO_LANE,
        FeedRefused,
        NoAnswerError,
        ask,
        check_lane,
        whole,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]
        FAILED_KIND_NAMES,
        KIND_NAMES,
        MODE_NAMES,
        NO_STOP,
        NOTICE_KINDS,
        REFUSAL_NAMES,
        STALE_SENSORS_KIND,
        STALE_SENSORS_TEXT,
        STOP_KINDS,
        Move,
        to_mm,
        to_um,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANE_ALL,
        LANES,
        NO_FEED,
        NO_LANE,
        FeedRefused,
        NoAnswerError,
        ask,
        check_lane,
        whole,
    )

try:
    from .ace2k_feed_base import FeedBase
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_base import FeedBase  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_klipper_types import (
        Command,
        Completion,
        FeedEvent,
        GCodeCommand,
        Params,
        Reactor,
    )

    # A feed event, as the host keeps it (FeedMotion._handle_event) and as Move.result() returns it.
    Event = FeedEvent
    Before = Callable[[int], None]  # called with the seq of a start, just before its query

    class FollowDefaultProto(Protocol):
        """``ace2k_follow.FollowDefault``, as the motions of the feed use it."""

        enabled: bool
        ready: bool

        def tick(self, eventtime: float | None = None) -> None: ...
        def note_start(self, lane: int) -> None: ...
        def note_stop(self, lane_index: int) -> None: ...
        def note_accepted(self, lane: int, seq: int) -> None: ...
        def on_lost(self, lane: int, seq: int) -> None: ...
        def forget_lost(self, lane: int) -> None: ...
        def on_final(self, lane: int, event: Event) -> None: ...
        def will_rearm(self, lane: int) -> bool: ...
        def begin_pause(self, lane: int) -> None: ...
        def paused(
            self, lane: int, what: str, seq: int, mode: str | None = None, rearm: bool = True
        ) -> None: ...
        def give_back(self, lane: int) -> None: ...
        def end_pause(self, lane: int) -> None: ...


T = TypeVar("T")

MODE_ERROR = 7
KIND_TAIL = 16
CMD_MODES = {
    "feed": 0,
    "rollback": 1,
    "assist": 2,
    "assist_back": 3,
    "unload": 4,
    "load": 5,
    "assist_both": 6,
}
# A stop armed on the plunger (start_move's stop_when): none, by default.
NO_STOP_KINDS: frozenset[str] = frozenset()
# The modes that take held: the two that move toward the spool from start to end. The unit
# judges held only while a move toward the spool runs. A load and the follow move that way only in
# parts that the unit chooses, so held is refused there. The reverse assist ends its take-up as
# soon as the plunger reads taut and then waits with no move, so held could never end it. Allowing
# more modes later only adds to the interface.
HELD_MODES = frozenset({"rollback", "unload"})
# A move's own held window (start_move's held_ms) is refused when friction alone could end the
# move held: when the friction floor, held_restart_mm / held_ms, passes this share of the move's
# speed. The filament trails the motor under a spool, so the floor keeps a margin under the speed.
# The firmware keeps its own defaults under the same share (feed.h, ACE2K_FEED_HELD_FLOOR_PCT).
HELD_FLOOR_SHARE = 0.95
# The follow keeps the plunger at rest, so at_rest would end it on its second tick.
AT_REST_REFUSED = "the follow keeps the plunger at rest, so at_rest would end it at once"
# The texts for a refused speed change. The unit answers in_error, no_move, following or bounds.
SPEED_REFUSAL_TEXT = {
    "no_move": "no move running",
    "in_error": "the lane is in error",
    "following": "the lane is following; set the follow's speed instead",
    "bounds": "the speed is out of bounds",
}
NO_FOLLOW = "the unit's firmware has no follow tail (or no follow); flash v0.11.0"
# A rollback that forgives every catch past snag_free (free_tip on a rollback).
NO_FREE_ROLLBACK = "the unit's firmware ignores FREE on a rollback; flash v0.12.0"
WAIT_FACTOR = 1.5  # the timeout of a waited move is this factor times the expected time …
WAIT_MARGIN_S = 5.0  # … plus this
SEQ_MAX = 255  # the sequence wraps from here to 1; 0 is for motions that the unit starts itself
COMMANDED_KEEP = 8  # how many start lengths are kept per lane until their event arrives
# A G-code move on a lane in the follow stops the follow first. PAUSE_WAIT_S is how long to wait
# for it to end.
PAUSE_WAIT_S = 5.0
# A start that the unit accepted may never get its final event (it was lost). It counts as over
# when a state report shows the lane idle or in error, this many reports after the acceptance.
GONE_REPORTS = 2
# A busy refusal from the unit's own work on a lane (its tie-break, which the refusal sends back,
# its own load, or a lane winding down after a stop) is tried again every BUSY_POLL_S for up to
# BUSY_S (retry_busy).
BUSY_S = 30.0
BUSY_POLL_S = 0.25
# A start before the first state report of the session waits for it, for this many report periods
# plus WAIT_MARGIN_S. It is two, so that the next report covers a first report that was lost on
# the way (for example by a full transmit buffer or the half-duplex turnaround).
STATE_WAIT_PERIODS = 2
# The extra travel that a waited load, or a read with MOVE=1, allows for the read sessions that
# pause it. It is a few sessions of up to 0.5 s each, at the load speed of 30 mm/s.
SESSION_ALLOWANCE_MM = 60.0


def stop_kind_set(stop_when: Iterable[str] | None) -> frozenset[str] | None:
    """Return start_move's ``stop_when`` as a set of names from STOP_KINDS, or None if it is not.

    None counts as empty. A string, anything that is not a collection, and a collection with an
    element that cannot be hashed or is not a stop's name all give None."""
    if stop_when is None:
        return NO_STOP_KINDS
    if isinstance(stop_when, str):
        return None
    try:
        kinds = frozenset(stop_when)
    except TypeError:
        return None
    return kinds if kinds <= set(STOP_KINDS) else None


@dataclass(frozen=True)
class Start:
    """A start to send: mode (a name from CMD_MODES), length, speed, free_tip and stop_when.

    The last two are as in start_move.

    For a G-code that waits, ``expected_mm`` is the travel that the wait allows for (None means
    the length). ``tail_mm`` is the travel past the bay entrance, a second move of its own.
    ``clear_mm`` is an unload's clear past the bay's sensor, which can be a third move."""

    mode: str
    length_mm: float
    speed_mm_s: float
    free_tip: bool = False
    expected_mm: float | None = None
    tail_mm: float = 0.0
    clear_mm: float = 0.0
    stop_when: frozenset[str] = NO_STOP_KINDS
    held_ms: int = 0  # the held window sent with the arming; 0: the unit's held_ms
    # who asked for it, as ACEK_STATUS names a follow's starter (FeedMotion.follow_starter)
    owner: str = "a G-code"


@dataclass(frozen=True)
class FollowPause:
    """A G-code move that pauses the follow of the lane (see FeedMotion.with_follow_paused).

    ``what`` names the move in the words of the console. ``seq`` and ``mode`` describe the end of
    the move, when they are known before it is sent. For example, a read with motion ends with the
    load of the unit, seq 0 and mode "loading". If ``seq`` is None, the end is that of the start
    that the move sends, in any mode. ``rearm`` says if the follow comes back at a clean end."""

    what: str
    seq: int | None = None
    mode: str | None = None
    rearm: bool = True


@dataclass
class LaneMotion:
    """The motion record of one lane."""

    # The sequence number of the last start that was sent. Until this session sends a start on
    # the lane, every state report sets it to the seq that the unit holds for the lane (the last
    # start on it, kept through idle and error, and 0 after power-up). So the first start gets
    # that seq plus 1, and never repeats the seq of a running move or of one that has just ended.
    # (Within about a second of its end, the unit takes a repeat of its last start as a retry by
    # the transport.) After the first start, the host owns the counter. next_seq() keeps it in
    # the range 1..255.
    seq: int = 0
    started: bool = False  # this session has sent a start on the lane
    # A start from this host is waiting for the answer of the unit. A second start on the lane
    # meanwhile is refused as busy, and nothing is sent. One start per lane can be in flight.
    inflight: bool = False
    # A disarm of the stop on the plunger was refused or got no answer, so a stop may still be
    # armed for the next start. That start then first sends its own stop (0 0 when it has none).
    # An accepted answer clears this.
    stop_unknown: bool = False
    # (seq, who) of the stop sent to the running motion of the lane: "command" for a G-code,
    # "host" for a host module, the feed included. The stopped event with that seq carries it as
    # stop_by. It is set only for a lane with a motion or a start in flight. A second stop before
    # the event never changes "command" to "host".
    stop_by: tuple[int, str] | None = None
    # The start that the lost-event rule counted over, its stop_by, and the report count then. Its
    # final event may still come right after the report that counted it: up to the next report,
    # with no start accepted meanwhile, it then answers that stop as ever.
    lost_seq: int | None = None
    lost_stop_by: tuple[int, str] | None = None
    lost_report: int = 0
    # The last start that the unit accepted (seq, mode), and the report count at that time.
    accepted_seq: int | None = None
    accepted_mode: str | None = None
    # Who asked for that start (Start.owner).
    accepted_owner: str | None = None
    accepted_report: int = 0
    # True when the final event of that start came, or it was counted as over. It is kept apart
    # from final_seq, which motions that the unit starts itself (seq 0) overwrite later.
    accepted_ended: bool = True
    final_seq: int | None = None  # the seq of the last final event
    report_seq: int | None = None  # the seq that the last state report showed
    # True when a final event came after the last state report. The motion that the report
    # showed is then over.
    ended_since_report: bool = False
    # Completions that the next final event of the lane completes. A G-code waits on one while the
    # follow that it stopped ends.
    final_waiters: list[Completion] = field(default_factory=list)
    commanded_mm: dict[int, float] = field(default_factory=dict)  # {seq: length}, until the end
    # The last final event (not a notice). If a start is refused after the end of the running
    # move arrived during its round trip, this event completes that running move.
    final_event: Event | None = None
    tracked: tuple[int, Completion] | None = None  # the seq and completion of the waiting G-code
    # True while a read with motion (ace2k_rfid.py) runs on the lane. Its search is the unit's own
    # load, which ends "loaded" after going out and coming back. ace2k_rfid logs that end with the
    # tag outcome, so the event handler leaves out its own line for it. ace2k_rfid sets the flag
    # before the read, sets it again once the unit accepts the read, and puts it back as it was if
    # the unit refuses the read or does not answer. The flag is cleared at the search's final
    # event, when the bay empties (the unknown tag event, or a state report that shows it and came
    # after the read was accepted), and when the unit accepts a host start on the lane, which
    # it does only on an idle lane. A G-code that stops waiting for the search leaves it set,
    # because a slow search still ends. The flag is set and cleared on the reactor, and the event
    # handler reads it on the serial thread.
    tag_search: bool = False

    def next_seq(self) -> int:
        """Return the next sequence number, 1 to 255, wrapping. It is never 0 (the unit's own)."""
        self.seq = self.seq % SEQ_MAX + 1
        return self.seq

    def open_seq(self, mode: object) -> int | None:
        """Return the seq of the current motion of the lane, or None when nothing runs.

        This is the last accepted start whose final event has not come. Otherwise it is the motion
        that the last report showed (``mode`` is the mode in that report), if no final event has
        ended it since."""
        acc = self.accepted_seq
        if acc is not None and not self.accepted_ended:
            return acc
        if mode in (None, "idle", "error"):
            return None
        if self.ended_since_report:
            return None  # the motion in that report has ended since
        return self.report_seq

    def stopped_seq(self, mode: object) -> int | None:
        """Return the seq that a stop sent now would end: the motion or the start in flight."""
        seq = self.open_seq(mode)
        if seq is None and self.inflight:
            seq = self.seq  # the start in flight, which carries this seq
        return seq

    def give_back(
        self, seq: int, completion: Completion | None, previous: tuple[int, Completion] | None
    ) -> None:
        """Make ``previous`` the tracked move again, after a start that got no mode or no answer."""
        if completion is not None and self.tracked == (seq, completion):
            self.tracked = previous
            # The running move may have ended during the round trip, and its event was handled
            # while this start held the slot. Complete the move from that event. (A final event
            # still to come finds the slot gone.)
            final = self.final_event
            if previous is not None and final is not None and final["seq"] == previous[0]:
                self.complete(final)

    def complete(self, event: Event) -> None:
        """Complete the tracked move with its final event if the sequences match, only once."""
        pending = self.tracked
        if pending is not None and pending[0] == event["seq"]:
            self.tracked = None
            pending[1].complete(event)


class FeedMotion(FeedBase):
    """The lane motions of ``Ace2kFeed``."""

    SESSION_ALLOWANCE_MM = SESSION_ALLOWANCE_MM

    # More attributes that the feed sets in its constructor: the reactor, the default follow
    # and the motion records.
    reactor: Reactor
    follow_auto: FollowDefaultProto
    motion: list[LaneMotion]
    reports: int
    state_reported: bool
    first_report: Completion

    # --- reports and events (the serial thread) -------------------------------------------

    def _handle_state(self, params: Params) -> None:
        mode = params["mode"]
        error = params["error"]
        speed = struct.unpack(f"<{LANES}I", params["speed_um_s"])
        duty = params["duty_pct"]
        bursts = struct.unpack(f"<{LANES}H", params["bursts"])
        seqs = list(params["seq"])
        modes = [MODE_NAMES.get(m, m) for m in mode]
        for i, lane in enumerate(self.lanes):
            lane["mode"] = modes[i]
            lane["error"] = KIND_NAMES.get(error[i], error[i]) if mode[i] == MODE_ERROR else None
            lane["speed"] = speed[i] / 1000.0
            lane["duty"] = duty[i]
            lane["bursts"] = bursts[i]
            if lane["mode"] != "following":
                lane["tail"] = False  # the lane left the follow; its final event was lost
        # The motion bookkeeping runs on the reactor, in order with the events and the stops. The
        # serial thread writes nothing that open_seq and stop read. The seeding runs on the
        # reactor thread too, where _start runs. A _start could run between the check and the
        # write, and the next start would get a seq that is already used.
        # The seeding is needed only while a lane follows the reports. state_reported and each
        # started flag change from False to True once and never back. So a stale read here can
        # only cause one extra seeding, never skip one. The default follow looks at the lanes
        # after both.
        seed = not self.state_reported or not all(m.started for m in self.motion)
        tick = self.follow_auto.enabled and self.follow_auto.ready

        def on_report(eventtime: float) -> None:
            self._on_report(seqs, modes)
            if seed:
                self._seed(seqs)
            if tick:
                self.follow_auto.tick(eventtime)

        self.reactor.register_async_callback(on_report)

    def _on_report(self, seqs: Sequence[int], modes: Sequence[object]) -> None:
        """Update the motion records from a state report, on the reactor.

        A start that the unit accepted without a final event counts as over when a report,
        GONE_REPORTS reports after the acceptance, shows the lane idle or in error."""
        self.reports += 1
        for i in range(LANES):
            m = self.motion[i]
            if m.lost_seq is not None and self.reports > m.lost_report:
                self._forget_lost(i)  # the late end of a start counted over came too late
            m.report_seq = seqs[i]
            m.ended_since_report = False
            acc = m.accepted_seq
            if (
                acc is not None
                and not m.accepted_ended
                and modes[i] in ("idle", "error")
                and self.reports >= m.accepted_report + GONE_REPORTS
            ):
                logging.warning(
                    "%s lane %s: start %d reported %s without its final event; counted over",
                    self.unit.head("feed"),
                    self.unit.lane_text(i),
                    acc,
                    modes[i],
                )
                m.final_seq = acc
                m.accepted_ended = True
                m.lost_seq = acc
                m.lost_stop_by = None
                m.lost_report = self.reports
                if m.stop_by is not None and m.stop_by[0] == acc:
                    m.lost_stop_by, m.stop_by = m.stop_by, None
                self.follow_auto.on_lost(i, acc)

    def _forget_lost(self, index: int) -> None:
        """Forget the start of the lane that was counted over: its final event no longer ends it."""
        m = self.motion[index]
        m.lost_seq = m.lost_stop_by = None
        self.follow_auto.forget_lost(index)

    def _seed(self, seqs: Sequence[int]) -> None:
        """Set the seq of every lane not started on in this session to the seq of the unit.

        This runs on the reactor thread. The first report also releases a start that waits."""
        for i in range(LANES):
            if not self.motion[i].started:
                self.motion[i].seq = seqs[i]
        if not self.state_reported:
            self.state_reported = True
            self.first_report.complete(True)

    def _handle_event(self, params: Params) -> None:
        index = params["lane"]
        kind = params["kind"]
        seq = params["seq"]
        event: Event = dict(
            kind=KIND_NAMES.get(kind, kind),
            mode=MODE_NAMES.get(params["mode"], params["mode"]),
            motor_mm=to_mm(params["motor_um"]),
            filament_mm=to_mm(params["filament_um"]),
            seq=seq,
            stop_by=None,
            tag_search=False,
        )
        lane = self.lanes[index]
        lane["last_event"] = event
        if kind == KIND_TAIL:
            # The unit no longer sees filament at the insert. The tail runs until its own final
            # event. A filament pushed into the bay in the meantime changes nothing.
            lane["tail"] = True
        elif kind not in NOTICE_KINDS:
            lane["tail"] = False  # a final event ends the mode, and the tail with it
        search_end = (
            self.motion[index].tag_search
            and event["kind"] == "loaded"
            and seq == 0
            and event["mode"] == "loading"
        )
        event["tag_search"] = search_end  # the lane's last= in ACEK_STATUS says the search
        if not search_end:  # ace2k_rfid logs the end of a read with motion itself
            logging.info(
                "%s lane %s %s (%s) — motor %.1f mm, filament %.1f mm",
                self.unit.head("feed"),
                self.unit.lane_text(index),
                event["kind"],
                event["mode"],
                event["motor_mm"],
                event["filament_mm"],
            )

        def dispatch(_eventtime: float) -> None:
            self._dispatch(index, event, kind)

        self.reactor.register_async_callback(dispatch)

    def _dispatch(self, index: int, event: Event, kind: int) -> None:
        # This runs on the reactor thread, like the G-codes. The completion is taken and completed
        # in one step, which is safe against a _start that registers one and then yields. Only the
        # event with the sequence of the waiting G-code completes it. A final event with another
        # sequence (stale, or seq 0 for a motion that the unit started itself) completes nothing.
        # The completion comes first, so a listener of "ace2k:feed_event" already sees the move as
        # done and cannot hold it back.
        if kind not in NOTICE_KINDS:
            seq = event["seq"]
            # The motion record is updated here and not on the serial thread, to stay in order
            # with the stops.
            m = self.motion[index]
            m.final_seq = seq
            m.ended_since_report = True
            if seq == m.accepted_seq:
                m.accepted_ended = True
            label = m.stop_by
            if seq != 0 and seq == m.lost_seq:  # the end of a start counted over: it came after all
                logging.info(
                    "%s lane %s: start %d's final event came after it was counted over",
                    self.unit.head("feed"),
                    self.unit.lane_text(index),
                    seq,
                )
                label, m.lost_seq, m.lost_stop_by = m.lost_stop_by, None, None
            if label is not None and label[0] == seq:  # this event answers the stop sent to it
                if m.stop_by is label:
                    m.stop_by = None  # a newer start's stop, if any, stays
                if event["kind"] == "stopped":
                    event["stop_by"] = label[1]
            # The commanded length of the start that this event ends. It is None for a motion that
            # the unit started itself (seq 0), or for a start whose entry was removed long ago.
            self.lanes[index]["last_move"] = dict(event, commanded_mm=m.commanded_mm.pop(seq, None))
            m.final_event = event
            m.complete(event)
            waiters, m.final_waiters = m.final_waiters, []
            for waiter in waiters:
                waiter.complete(event)
            self.follow_auto.on_final(index, event)
        # Send a copy, so the stored event keeps its shape. Only host modules get ``unit``.
        self.printer.send_event("ace2k:feed_event", index, dict(event, unit=self.unit.name))
        if event["kind"] == STALE_SENSORS_KIND:
            # The unit cannot read its insert sensors, so it stopped the lane rather than run on
            # a frozen reading. Nothing starts until they read again: the cause is said here.
            self.unit.say(
                f"ace2k: lane {index + 1} stopped ({event['mode']}) — {STALE_SENSORS_TEXT}"
            )
        if event["kind"] == "snag":
            # A notice and not an error. The tip caught, and the unit went on, either after a
            # small move back (the tolerance) or, with a free tip, with the filament following.
            self.unit.say(
                f"ace2k: lane {index + 1} snag ({event['mode']}) — the tip caught at motor"
                f" {event['motor_mm']:.1f} mm, filament {event['filament_mm']:.1f} mm; went on"
            )

    # --- the host interface (see API_VERSION) -----------------------------------------------------

    def start_move(  # noqa: PLR0913 — the interface grows by optional parameters (API_VERSION 20)
        self,
        lane_index: int,
        mode: str,
        length_mm: float,
        speed_mm_s: float,
        free_tip: bool = False,
        *,
        stop_when: Iterable[str] | None = NO_STOP_KINDS,
        held_ms: int | None = None,
    ) -> Move:
        """Host interface: start ``mode`` (a name from CMD_MODES) on lane ``lane_index`` (0..3).

        This does not wait. It can be called from G-code handlers, timers and async callbacks, and
        it returns a Move.

        ``free_tip`` says that nothing holds the front tip of the filament, for example when
        there is no extruder and no head. A feed then judges "full" by the strand. A rollback
        forgives every catch that comes back past snag_free, and not only the first. An image
        without has_free_rollback forgives only one. A load from the bay already gets a free tip
        from the unit, and the read with motion never does, so free_tip has no effect on them. An
        unload still stops at its one catch, so a tip that the head holds always ends it as stuck.

        ``stop_when`` names the stops on the plunger that also end the move: "held" (a move toward
        the spool whose tip is held: the final kind "held") and "at_rest" (the plunger at rest:
        the final kind "at_rest"). It is empty by default (None counts as empty), and the start is
        then sent as before. Otherwise the host first sends the stop, armed for this start only,
        and then the start. "held" is taken only on "rollback" and "unload", and "at_rest" on every
        mode but "assist_both".

        ``held_ms`` is the held window of this move only, in whole ms within held_ms_min and
        held_ms_max. It needs "held" in ``stop_when``. None sends 0, and the unit uses its own
        held_ms. A window whose friction floor (held_restart_mm / held_ms) passes 95 % of
        ``speed_mm_s`` is refused, because friction alone could then end the move held.

        Raises FeedRefused when the unit refuses the start or the stop (busy, or in_error: clear the
        lane first; bounds for a window it does not take). Raises ValueError for an unknown lane,
        mode or stop, for a stop that the mode does not take, and for a held_ms that is not a whole
        number in its bounds, comes without "held" or fails the friction floor. Nothing is sent
        then. Raises RuntimeError when the firmware has no feed, has not reported
        yet, has no follow (for "assist_both"), cannot stop on the plunger (has_stop_when is False),
        or did not answer."""
        return self._start_move(
            "a host module",
            lane_index,
            mode,
            length_mm,
            speed_mm_s,
            free_tip=free_tip,
            stop_when=stop_when,
            held_ms=held_ms,
        )

    def start_follow_as(self, owner: str, lane_index: int, speed_mm_s: float) -> Move:
        """Arm the follow of the lane as start_move(lane_index, "assist_both", 0.0, speed_mm_s)
        does, for ace2k's own callers. ``owner`` names them in ACEK_STATUS (follow_starter): a
        start_move of another module is "a host module"."""
        return self._start_move(owner, lane_index, "assist_both", 0.0, speed_mm_s)

    def _start_move(  # noqa: PLR0913 — start_move's parameters and the owner
        self,
        owner: str,
        lane_index: int,
        mode: str,
        length_mm: float,
        speed_mm_s: float,
        *,
        free_tip: bool = False,
        stop_when: Iterable[str] | None = NO_STOP_KINDS,
        held_ms: int | None = None,
    ) -> Move:
        """start_move, with ``owner`` carried by the start itself. A start of another lane
        during this start's round trip keeps its own owner."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        if mode not in CMD_MODES:
            raise ValueError(f"ace2k: no lane {lane_index} / mode {mode!r}")
        # This message is formatted here and not in check_lane, because it names the mode as well
        # as the lane. Callers and tests read the text.
        check_lane(lane_index, f"ace2k: no lane %s / mode {mode!r}")
        kinds = self._stop_kinds(mode, stop_when)
        window = self._held_window(kinds, held_ms, speed_mm_s)
        completion = self.reactor.completion()
        start = Start(
            mode, length_mm, speed_mm_s, free_tip, stop_when=kinds, held_ms=window, owner=owner
        )
        seq = self._send_start(lane_index, start, completion)
        return Move(lane_index, mode, seq, completion, self.reactor)

    def _stop_kinds(self, mode: str, stop_when: Iterable[str] | None) -> frozenset[str]:
        """Return the stops of start_move's ``stop_when`` as a set, once they are checked.

        None counts as empty. Raises ValueError for a stop_when that is not a collection of names
        from STOP_KINDS, and for a stop that the mode does not take. Raises RuntimeError when the
        image cannot stop on the plunger."""
        kinds = stop_kind_set(stop_when)
        if kinds is None:
            raise ValueError(
                f"ace2k: stop_when {stop_when!r}: the kinds are {' and '.join(STOP_KINDS)}"
            )
        if kinds and not self.has_stop_when:
            raise RuntimeError(f"ace2k: {NO_STOP}")
        if "held" in kinds and mode not in HELD_MODES:
            raise ValueError(
                f"ace2k: stop_when held on {mode}: held is taken only on rollback and unload"
            )
        if "at_rest" in kinds and mode == "assist_both":
            raise ValueError(f"ace2k: stop_when at_rest on {mode}: {AT_REST_REFUSED}")
        return kinds

    def _held_window(self, kinds: frozenset[str], held_ms: int | None, speed_mm_s: float) -> int:
        """Return start_move's ``held_ms`` as the value to send (0 for None), once it is checked.

        Raises ValueError for a value that is not a whole number within the unit's bounds, for a
        window without "held", and for a window whose friction floor passes HELD_FLOOR_SHARE of
        the move's speed. _stop_kinds runs first, so the image has the stop here when "held" is
        set."""
        if held_ms is None:
            return 0
        if "held" not in kinds:
            raise ValueError(f"ace2k: held_ms {held_ms!r} without held: the window is held's own")
        if isinstance(held_ms, bool) or not isinstance(held_ms, int):
            raise ValueError(f"ace2k: held_ms {held_ms!r}: a whole number of milliseconds")
        # The image has the stop (see the docstring), so the connect read these.
        assert self.held_ms_min is not None and self.held_ms_max is not None
        assert self.held_restart_mm is not None
        if not self.held_ms_min <= held_ms <= self.held_ms_max:
            raise ValueError(
                f"ace2k: held_ms {held_ms}: the unit takes {self.held_ms_min} to {self.held_ms_max}"
            )
        floor = self.held_restart_mm * 1000.0 / held_ms
        if floor > HELD_FLOOR_SHARE * speed_mm_s:
            raise ValueError(
                f"ace2k: held_ms {held_ms} at {speed_mm_s:g} mm/s: friction can end it held below"
                f" {floor:.1f} mm/s, more than {HELD_FLOOR_SHARE * 100:g} % of its speed; raise"
                " held_ms"
            )
        return held_ms

    def set_speed(self, lane_index: int, speed_mm_s: float) -> None:
        """Host interface: change the speed (mm/s) of the move that runs on lane ``lane_index``.

        ACEK_SPEED does the same. This can be called from G-code handlers, timers and async
        callbacks. It waits only for the answer of the unit, and returns None once the unit has
        taken the speed.

        Raises ValueError for a lane outside 0..3 or a speed outside the limits of the unit, and
        sends nothing. Raises FeedRefused, with mode "speed" and reason no_move, in_error, following
        or bounds, when the unit refuses. An idle lane keeps nothing for a later move. A following
        lane keeps the speed its follow was armed with. On an older image, which would take the
        speed there, the host raises the same FeedRefused itself and sends nothing. Raises
        NoAnswerError (a RuntimeError) if the unit does not answer, and RuntimeError on an image
        without the feed."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        if not whole(lane_index):
            raise ValueError(f"ace2k: no lane {lane_index!r}")
        check_lane(lane_index, "ace2k: no lane %r")
        bounds = self.speed_out(speed_mm_s)
        if bounds:
            raise ValueError(f"ace2k: speed {speed_mm_s!r} outside {bounds}")
        if not self.has_following_refusal and self.is_following(lane_index):
            # An older image would take the speed and keep it for the follow's later bursts.
            # So the host refuses it here, as a newer image does.
            raise FeedRefused(lane_index, "speed", "following")
        # The unit answers whether a move took the speed. An idle lane, or one in error, keeps
        # nothing for a later move, and the answer says so.
        what = f"lane {lane_index + 1} speed"
        assert self.speed_query is not None  # the connect built it, because the feed is present
        params = ask(
            self.printer, self.mcu, self.speed_query, [lane_index, to_um(speed_mm_s)], what
        )
        if params["lane"] != lane_index:
            # This is a stale answer. It is not an answer for this lane.
            raise NoAnswerError(
                f"ace2k: {what}: the unit answered for lane {params['lane'] + 1}; try again"
            )
        if not params["accepted"]:
            reason = REFUSAL_NAMES.get(params["reason"], f"reason {params['reason']}")
            raise FeedRefused(lane_index, "speed", reason)

    def stop(self, lane_index: int, by: str = "host") -> None:
        """Host interface: stop the motor of lane ``lane_index`` (LANE_ALL: every lane).

        It never clears an error. ``by`` is written to the next stopped event of the lane as
        stop_by: "host" for a host module, "command" for a G-code. Raises ValueError for another
        lane, and RuntimeError when the firmware has no feed."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        if lane_index != LANE_ALL:
            check_lane(lane_index, NO_LANE)
        for i in range(LANES) if lane_index == LANE_ALL else [lane_index]:
            m = self.motion[i]
            seq = m.stopped_seq(self.lanes[i]["mode"])
            if seq is None:
                continue  # nothing runs there, so no event will answer and no label is left
            label = m.stop_by
            if label is not None and label[0] == seq and label[1] == "command":
                continue  # a stop by the operator stays the stop of the operator
            m.stop_by = (seq, by)
        self.follow_auto.note_stop(lane_index)  # any stop ends the episode of the default follow
        # A feed that is not connected has no command, so the send fails here.
        cast("Command", self.stop_cmd).send([lane_index])

    def clear(self, lane_index: int) -> None:
        """Host interface: take lane ``lane_index`` out of its error state.

        Raises ValueError for another lane, and RuntimeError when the firmware has no feed."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        check_lane(lane_index, NO_LANE)
        # A feed that is not connected has no command, so the send fails here.
        cast("Command", self.clear_cmd).send([lane_index])

    def open_seq(self, index: int) -> int | None:
        """Return the seq of the current motion of the lane, or None when nothing runs.

        This is the last accepted start whose final event has not come. Otherwise it is the motion
        that the last report showed, if no final event has ended it since. The report keeps the
        seq of the lane's last host start through the unit's own modes, and a seq from before a
        Klipper restart too, so a reported seq is not always a move of this host (see
        retry_busy)."""
        return self.motion[index].open_seq(self.lanes[index]["mode"])

    def in_follow(self, index: int) -> bool:
        """Return True if the lane counts as a follower of its extruder.

        The last report must say "following", and its motion must not have ended since. A follow
        whose final event came before the next report is over."""
        return self.lanes[index]["mode"] == "following" and self.open_seq(index) is not None

    def follow_starter(self, index: int) -> str | None:
        """Return who started the lane's follow, or None when the lane does not follow (in_follow).

        For this host's last accepted start, a follow, it is its Start.owner: "a host module" for
        a start_move of another module, "a G-code" for a G-code's, or the name that ace2k's own
        caller gave (start_follow_as). A follow that the unit started itself (seq 0) is "the unit";
        one that no start of this host explains is "an earlier start" (before a restart)."""
        if not self.in_follow(index):
            return None
        m = self.motion[index]
        seq = self.open_seq(index)
        if seq == m.accepted_seq:
            return m.accepted_owner if m.accepted_mode == "assist_both" else None
        return "the unit" if seq == 0 else "an earlier start"

    def is_following(self, index: int) -> bool:
        """Return True if the current motion is a follow that has not ended, its tail included.

        A follow that the unit accepted counts before its first report."""
        seq = self.open_seq(index)
        if seq is None:
            return False
        m = self.motion[index]
        if seq == m.accepted_seq:
            return m.accepted_mode == "assist_both"
        return bool(self.lanes[index]["mode"] == "following")

    def follow_running(self, index: int) -> bool:
        """Return True if the current motion is a follow that has not ended or entered its tail."""
        return self.is_following(index) and not self.lanes[index]["tail"]

    def retry_busy(self, send: Callable[[], T], lane: int) -> T:
        """Call ``send()`` again while the unit refuses it as busy, and return its result.

        A busy refusal is an error with ``reason`` "busy" (the text is not checked). Unless a
        start of this host on ``lane`` was accepted and its final event has not come, the busy is
        the unit's own work: its tie-break, which the refusal sends back, its own load, or a lane
        winding down after a stop. That holds whatever seq the report shows, which may be the one
        of an ended host start or one from before a Klipper restart. The call is then repeated
        every BUSY_POLL_S for up to BUSY_S. Any other error, a busy refusal with this host's
        start open, and the last busy refusal are raised."""
        give_up = self.reactor.monotonic() + BUSY_S
        while True:
            try:
                return send()
            # Whatever send raises, only a busy refusal is tried again.
            except Exception as e:  # noqa: PERF203 — the retry needs the try inside the loop
                now = self.reactor.monotonic()
                m = self.motion[lane]
                own = m.accepted_seq is not None and not m.accepted_ended
                busy = getattr(e, "reason", None) == "busy" and not own
                if not busy or now + BUSY_POLL_S > give_up:
                    raise
                self.reactor.pause(now + BUSY_POLL_S)

    def move_wait_s(
        self, lane_index: int, length_mm: float, speed_mm_s: float, *, reverse: bool = False
    ) -> float:
        """Host interface: return how long, in seconds, a caller should wait for a move.

        The move has ``length_mm`` at ``speed_mm_s`` on lane ``lane_index`` (0..3). This is the
        budget of the waiting G-codes of ace2k: the expected time times a factor, plus a margin
        (see _move_s). It only calculates and sends nothing.

        With ``reverse`` the move is a rollback or an unload. The unit can run a tail after it
        (``tail_mm``), and the budget counts the tail as ACEK_ROLLBACK and ACEK_UNLOAD do. It
        also counts an unload's clear past the bay's sensor (``unload_clear_mm``), as ACEK_UNLOAD
        does, because the caller may wait for an unload.

        Raises ValueError for a lane outside 0..3, a negative or non-finite length, or a speed that
        is not above 0 or not finite. Raises RuntimeError if the firmware has no feed."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        check_lane(lane_index, NO_LANE)
        finite = math.isfinite(length_mm) and math.isfinite(speed_mm_s)
        if not finite or length_mm < 0 or speed_mm_s <= 0:
            raise ValueError(f"ace2k: no estimate for {length_mm!r} mm at {speed_mm_s!r} mm/s")
        if not reverse:
            return self._wait_timeout(length_mm, speed_mm_s)
        return self._wait_timeout(
            length_mm, speed_mm_s, self._reverse_tail_mm(), self.unload_clear_mm
        )

    def load_wait_s(self, travel_mm: float) -> float:
        """Return how long a wait for ``travel_mm`` of the load may take, in seconds.

        It uses the current load speed (load_speed, or the last ACEK_LOAD_SET). The MOVE=1 read
        in ace2k_rfid.py uses this."""
        return self._wait_timeout(travel_mm, self.load_speed_mm_s)

    # --- the start ----------------------------------------------------------------------------

    def speed_out(self, speed: object) -> str | None:
        """Return None if ``speed`` (mm/s) is within the unit limits, else the limits as text.

        The text is the form that the messages print. A speed that is not a number is never within
        the limits. A bool also counts as not a number."""
        # Only a feed that the connect found is asked: the G-codes, set_speed and the connect.
        assert self.speed_min is not None and self.speed_max is not None
        if (
            isinstance(speed, (int, float))
            and not isinstance(speed, bool)
            and self.speed_min <= speed <= self.speed_max
        ):
            return None
        return f"{self.speed_min:g}..{self.speed_max:g} mm/s"

    def _move_s(self, travel_mm: float, speed_mm_s: float) -> float:
        """Return the expected time of one move in seconds, as the firmware budgets it.

        The landing is the last ACE2K_LANE_LANDING_UM, or all of a shorter move, at the lowest
        speed. The rest is at the commanded speed."""
        # Used only for a waited move or a read with motion, on a feed that the connect found.
        assert self.landing_mm is not None and self.speed_min is not None
        if travel_mm > self.landing_mm:
            return (travel_mm - self.landing_mm) / speed_mm_s + self.landing_mm / self.speed_min
        return travel_mm / self.speed_min

    def _wait_timeout(
        self, travel_mm: float, speed_mm_s: float, tail_mm: float = 0.0, clear_mm: float = 0.0
    ) -> float:
        """Return the timeout that matches the deadline of the firmware, in seconds.

        It is WAIT_FACTOR times the expected time, plus WAIT_MARGIN_S. The tail of an unload or
        rollback (``tail_mm``) counts as a second move, and the clear of an unload (``clear_mm``)
        as a third, each with its own landing."""
        expected = self._move_s(travel_mm, speed_mm_s)
        if tail_mm:
            expected += self._move_s(tail_mm, speed_mm_s)
        if clear_mm:
            expected += self._move_s(clear_mm, speed_mm_s)
        return WAIT_FACTOR * expected + WAIT_MARGIN_S

    def _reverse_tail_mm(self) -> float:
        """Return the tail that the unit can run after a rollback or an unload, in mm.

        Once the filament leaves the motor, the unit carries on for up to this travel. The
        waits of ACEK_ROLLBACK, ACEK_UNLOAD and move_wait_s(reverse=True) count it."""
        assert self.tail_mm is not None  # the connect found the feed
        return self.tail_mm

    def _load_budget_mm(self, park_mm: float) -> float:
        """Return how far a waited load may travel, in mm.

        This is the pull. With the tag search it also covers the search past the parking point,
        the way back, and the pauses of the read sessions."""
        search_mm = self.search_mm
        if not self.has_search or search_mm is None or search_mm <= park_mm:
            return park_mm
        return park_mm + 2 * (search_mm - park_mm) + self.SESSION_ALLOWANCE_MM

    def _wait_first_report(self) -> None:
        """Wait for the first state report of the session, if it has not arrived.

        The wait is STATE_WAIT_PERIODS periods plus WAIT_MARGIN_S. Raise RuntimeError if no
        report arrives in that time."""
        if self.state_reported:
            return
        timeout = STATE_WAIT_PERIODS / self.report_hz + WAIT_MARGIN_S
        if self.first_report.wait(self.reactor.monotonic() + timeout, None) is None:
            raise RuntimeError(
                f"ace2k: no feed state report within {timeout:g} s (the unit reports at"
                f" feed_report_hz = {self.report_hz:.3g} Hz; check the link)"
            )

    def _send_start(
        self,
        index: int,
        start: Start,
        completion: Completion | None,
        before_send: Before | None = None,
    ) -> int:
        """Send the start with the next sequence of the lane, and return that sequence.

        ``completion`` (or None) is registered before the send. A refusal raises FeedRefused and
        registers nothing. Before the first state report of the session, the counter is not yet
        set past a move that the unit may still run. So the start waits for that report, for
        STATE_WAIT_PERIODS periods plus WAIT_MARGIN_S, and raises RuntimeError if none arrives.
        The follow on an image without a follow also raises RuntimeError and sends nothing."""
        mode = start.mode
        if mode == "assist_both" and not self.has_follow:
            raise RuntimeError(f"ace2k: {NO_FOLLOW}")
        if mode == "assist_both" and not self.unit.units.may_follow(self.unit, index):
            # The lane belongs to a shared extruder, on any unit, and is not its active lane.
            # Only one lane per extruder can be in the follow. ACEK_LANE_ACTIVE chooses it.
            raise FeedRefused(index, mode, "not_active")
        if self.motion[index].inflight:
            # Another start on the lane waits for its answer. The unit would refuse this one as
            # busy, and two queries in flight would race.
            raise FeedRefused(index, mode, "busy")
        self.follow_auto.note_start(index)  # any start except the default follow ends its episode
        self.motion[index].inflight = True
        try:
            return self._send_start_now(index, start, completion, before_send)
        finally:
            self.motion[index].inflight = False

    def _send_start_now(
        self, index: int, start: Start, completion: Completion | None, before_send: Before | None
    ) -> int:
        # A length or a speed that cannot be sent (NaN, an infinity) fails here, before anything
        # is sent or taken.
        length_um, speed_um = to_um(start.length_mm), to_um(start.speed_mm_s)
        self._wait_first_report()
        m = self.motion[index]
        armed = bool(start.stop_when) or m.stop_unknown
        if armed:
            # Right before the start, and before anything is registered, so a refusal leaves
            # nothing behind. Only the host's next start on the lane takes it. A lane whose last
            # disarm was not confirmed sends this start's own stop, 0 0 when it has none.
            self._arm_stop(index, start)
        m.started = True  # from here on the host owns the counter, and the reports leave it alone
        seq = m.next_seq()
        what = f"lane {index + 1} {start.mode}"
        # Every start checks that the feed is present first (start_move, _require), so the
        # connect has built the query.
        assert self.start_query is not None
        try:
            if before_send is not None:
                before_send(seq)  # before the query: the move may end during the round trip
            # Track this move until the unit answers; a busy refusal gives the old tracking back.
            previous = m.tracked
            if completion is not None:
                m.tracked = (seq, completion)
            # Store the length before the send, because the final event of a short move can
            # arrive during the round trip. The oldest entries go once more than COMMANDED_KEEP
            # wait, so a lost event never grows the map. The serial thread removes entries too,
            # so the keys are copied first and the entries are removed with a default.
            lengths = m.commanded_mm
            lengths[seq] = start.length_mm
            for old in list(lengths)[:-COMMANDED_KEEP]:
                lengths.pop(old, None)
            data = [index, CMD_MODES[start.mode], length_um, speed_um, seq, int(start.free_tip)]
            params = ask(self.printer, self.mcu, self.start_query, data, what)
            if params["lane"] != index:
                # This is a late answer to an earlier start. It is not an answer to this start.
                raise NoAnswerError(
                    f"ace2k: {what}: the unit answered for lane {params['lane'] + 1}"
                )
        except NoAnswerError as e:
            # The unit may have started the move. Stop it, as for a waited move that times out,
            # so it does not run untracked. The length stays, in case an event arrives. The lane's
            # stop goes before the disarm, so on a failing link it never waits behind the
            # disarm's timeout.
            m.give_back(seq, completion, previous)
            self.stop(index)
            if armed:
                self._disarm_stop(index)  # the start may not have reached the unit
            raise NoAnswerError(f"{e}; lane {index + 1} stopped in case it started") from e
        except Exception:
            # Anything else between the arming and the start's send: no start took the stop.
            if armed:
                self._disarm_stop(index)
            raise
        if not params["accepted"]:
            m.give_back(seq, completion, previous)
            lengths.pop(seq, None)  # it entered no mode; the entries of earlier starts stay
            reason = REFUSAL_NAMES.get(params["reason"], f"reason {params['reason']}")
            raise FeedRefused(index, start.mode, reason)
        m.accepted_seq = seq
        m.accepted_mode = start.mode
        m.accepted_owner = start.owner
        m.lost_seq = m.lost_stop_by = None  # a new start ends the window of one counted over
        m.tag_search = False  # an accepted start means the lane was idle: no search runs there
        m.accepted_report = self.reports
        # Its end may have arrived during the round trip, before this line.
        m.accepted_ended = m.final_seq == seq
        self.follow_auto.note_accepted(index, seq)
        return seq

    def _arm_stop(self, index: int, start: Start) -> None:
        """Arm the stops of ``start.stop_when`` for the next start on the lane (0 0 for none).

        An accepted answer clears the lane's stop_unknown. Raises FeedRefused when the unit
        refuses them: it arms nothing on a lane that is not idle. Raises NoAnswerError when it
        does not answer, or answers for another lane. The unit may then have armed them, so they
        are disarmed first."""
        what = f"lane {index + 1} {start.mode} stop"
        flags = [1 if kind in start.stop_when else 0 for kind in STOP_KINDS]
        try:
            params = self._ask_stop_when(index, [*flags, start.held_ms], what)
        except NoAnswerError:
            self._disarm_stop(index)
            raise
        if not params["accepted"]:
            reason = REFUSAL_NAMES.get(params["reason"], f"reason {params['reason']}")
            raise FeedRefused(index, start.mode, reason)
        self.motion[index].stop_unknown = False

    def _disarm_stop(self, index: int) -> None:
        """Disarm a stop that no start may have taken, so it cannot end the lane's next move.

        This is for a path that armed a stop and then lost the start. When the unit refuses (the
        lane runs a mode, most likely the start that took the stop) or does not answer, the lane's
        stop_unknown is set, so its next start first sends its own stop. No answer also gets a
        log line."""
        m = self.motion[index]
        try:
            params = self._ask_stop_when(index, [0, 0, 0], f"lane {index + 1} stop disarm")
        except NoAnswerError as e:
            m.stop_unknown = True
            logging.warning(
                "%s lane %s: the stop may still be armed for the next start (%s)",
                self.unit.head(),
                self.unit.lane_text(index),
                e,
            )
            return
        m.stop_unknown = not params["accepted"]

    def _ask_stop_when(self, index: int, flags: list[int], what: str) -> Params:
        """Send ace2k_feed_stop_when with ``flags`` (held, at_rest, held_ms) and return the answer.

        Raises NoAnswerError when the unit does not answer, or answers for another lane."""
        assert self.stop_when_query is not None  # the connect built it, as has_stop_when is True
        params = ask(self.printer, self.mcu, self.stop_when_query, [index, *flags], what)
        if params["lane"] != index:
            raise NoAnswerError(f"ace2k: {what}: the unit answered for lane {params['lane'] + 1}")
        return params

    def pause_follow(
        self, gcmd: GCodeCommand, index: int, what: str, rearm: bool = True
    ) -> str | None:
        """Stop the follow on a lane before a G-code move, and wait for the follow to end.

        The stop counts as a host stop, never as a stop by the operator. Return None when no
        follow runs on the lane (its end was seen, or it is in its tail). Return "paused" when the
        follow comes back at the clean end of the move (``rearm`` is set and follow_default
        applies to the lane), and "stopped" otherwise. If the follow does not end within
        PAUSE_WAIT_S, raise a G-code error. The move does not start, and a follow that was to
        come back is given back."""
        if not self.follow_running(index):
            return None
        n = index + 1
        rearm = rearm and self.follow_auto.will_rearm(index)
        waiter = self.reactor.completion()
        self.motion[index].final_waiters.append(waiter)
        self.stop(index)
        self.follow_auto.begin_pause(index)  # after the stop, which clears an older pause
        state = "paused" if rearm else "stopped"
        gcmd.respond_info(f"ace2k: lane {n} follow {state} for the {what}")
        if waiter.wait(self.reactor.monotonic() + PAUSE_WAIT_S, None) is None:
            if waiter in self.motion[index].final_waiters:
                self.motion[index].final_waiters.remove(waiter)
            if rearm:
                self.follow_auto.give_back(index)
            else:
                self.follow_auto.end_pause(index)
            after = "it comes back once the lane is idle" if rearm else "it stays off"
            raise gcmd.error(
                f"ace2k: lane {n} {what} not started: the follow was stopped but did not end"
                f" within {PAUSE_WAIT_S:g} s; {after}"
            )
        return state

    def with_follow_paused(
        self,
        gcmd: GCodeCommand,
        lane: int,
        pause: FollowPause,
        send: Callable[[Before | None], T],
        *,
        retry: bool = True,
    ) -> T:
        """Call ``send(before)`` for a G-code move on the lane, and return its result.

        ``pause`` is a FollowPause that names the move. If a follow runs on the lane, it is paused
        first (see pause_follow). The pause lasts until the move ends: the end with seq
        ``pause.seq``, registered before the send, or, if that is None, the end with the seq that
        ``send`` passes to ``before(seq)`` just before its query. ``before`` is None when there is
        nothing to register.

        A busy refusal from the unit's own work is tried again (see retry_busy), unless ``retry``
        is False. If the send
        fails, the follow comes back (when "paused") or the pause ends (when "stopped"), and the
        error is raised unchanged."""
        state = self.pause_follow(gcmd, lane, pause.what, rearm=pause.rearm)

        def sent(arg: Before | None) -> T:
            return self.retry_busy(lambda: send(arg), lane) if retry else send(arg)

        if state is None:
            return sent(None)

        def before(seq: int) -> None:
            # Only the end of the move with this seq ends the pause. If the follow was paused, the
            # end also re-arms it.
            self.follow_auto.paused(lane, pause.what, seq, pause.mode, rearm=state == "paused")

        if pause.seq is not None:
            before(pause.seq)
        try:
            return sent(before if pause.seq is None else None)
        except (FeedRefused, RuntimeError):
            if state == "paused":
                self.follow_auto.give_back(lane)
            else:
                self.follow_auto.end_pause(lane)
            raise

    def _start(
        self, gcmd: GCodeCommand, index: int, start: Start, wait: int, *, retry: bool = True
    ) -> Event | None:
        """Send the start (see _send_start) for a G-code. If ``wait`` is set, wait for its event.

        ``retry`` False answers a busy refusal at once (see with_follow_paused).

        A refusal or a missing state report becomes a G-code error. When waiting, return the event
        with the sequence of the start. On a timeout, stop the lane and raise a G-code error.
        A follow on the lane is paused first (see pause_follow). It comes back when the move ends
        cleanly, but not after an unload, because the filament leaves."""
        mode = start.mode
        completion = self.reactor.completion() if wait else None
        what = "assist" if mode == "assist_back" else mode

        def send(before: Before | None) -> int:
            return self._send_start(index, start, completion, before)

        try:
            if mode == "assist_both":
                # Asking for the follow again (ACEK_ASSIST DIR=BOTH) is not a move, so it is not
                # paused. The unit refuses it as busy.
                send(None)
            else:
                # After an unload the filament has left, and a one-way assist replaces the follow.
                # So the follow does not re-arm.
                rearm = mode not in ("unload", "assist", "assist_back")
                self.with_follow_paused(
                    gcmd, index, FollowPause(what, rearm=rearm), send, retry=retry
                )
        except (FeedRefused, RuntimeError) as e:
            raise gcmd.error(str(e)) from e
        if completion is None:
            gcmd.respond_info(f"ace2k: lane {index + 1} {mode} started")
            return None
        travel = start.expected_mm if start.expected_mm is not None else start.length_mm
        timeout = self._wait_timeout(travel, start.speed_mm_s, start.tail_mm, start.clear_mm)
        event: Event | None = completion.wait(self.reactor.monotonic() + timeout, None)
        if event is None:
            # The unit may still move (slower ACEK_SPEED, long landing). Stop it.
            self.motion[index].tracked = None
            self.stop(index)
            raise gcmd.error(
                f"ace2k: lane {index + 1} {mode}: no completion event within {timeout:.0f} s;"
                " a stop was sent"
            )
        summary = f"motor {event['motor_mm']} mm, filament {event['filament_mm']} mm"
        if event["kind"] in FAILED_KIND_NAMES:
            raise gcmd.error(f"ace2k: lane {index + 1} {mode} failed: {event['kind']} — {summary}")
        gcmd.respond_info(f"ace2k: lane {index + 1} {mode} {event['kind']} — {summary}")
        return event

    # --- the move G-codes ---------------------------------------------------------------------

    def _require(self, gcmd: GCodeCommand) -> None:
        if not self.present:
            raise gcmd.error(f"ace2k: {NO_FEED}")

    def _lane(self, gcmd: GCodeCommand) -> int:
        return gcmd.get_int("LANE", minval=1, maxval=LANES) - 1

    def _speed(self, gcmd: GCodeCommand, default: float) -> float:
        speed = gcmd.get_float("SPEED", default)
        bounds = self.speed_out(speed)
        if bounds:
            raise gcmd.error(f"ace2k: SPEED must be within {bounds}")
        return speed

    cmd_ACEK_FEED_help = (
        "Feed LENGTH=<mm> on LANE=<1-4> at SPEED=<mm/s>; FREE=1 when nothing holds the tip (its"
        " buffer full judged by the strand); WAIT=0 returns at once"
    )

    def cmd_ACEK_FEED(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.feed_speed_mm_s)
        free_tip = gcmd.get_int("FREE", 0, minval=0, maxval=1)
        start = Start("feed", length, speed, bool(free_tip))
        self._start(gcmd, index, start, gcmd.get_int("WAIT", 1))

    cmd_ACEK_ROLLBACK_help = (
        "Rewind LENGTH=<mm> on LANE=<1-4> at SPEED=<mm/s>; FREE=1 when nothing holds the tip"
        " (every catch past snag_free forgiven); WAIT=0 returns at once"
    )

    def cmd_ACEK_ROLLBACK(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.feed_speed_mm_s)
        wait = gcmd.get_int("WAIT", 1)
        free_tip = gcmd.get_int("FREE", 0, minval=0, maxval=1)
        if free_tip and not self.has_free_rollback:
            raise gcmd.error(f"ace2k: {NO_FREE_ROLLBACK}")
        # Once the filament leaves the motor, the unit runs the tail of the unload. The wait
        # counts it.
        start = Start("rollback", length, speed, bool(free_tip), tail_mm=self._reverse_tail_mm())
        self._start(gcmd, index, start, wait)

    cmd_ACEK_UNLOAD_help = (
        "Rewind LANE=<1-4> until the filament leaves the bay entrance (LENGTH=<budget mm>, SPEED=)"
    )

    def cmd_ACEK_UNLOAD(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        budget = gcmd.get_float("LENGTH", 0.0, minval=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.unload_speed_mm_s)
        expected = budget if budget else self.move_max_mm
        wait = gcmd.get_int("WAIT", 1)
        # The unit runs the tail past the motor, as it does for a rollback. The wait counts it.
        tail_mm = self._reverse_tail_mm()
        clear_mm = self.unload_clear_mm
        start = Start(
            "unload", budget, speed, expected_mm=expected, tail_mm=tail_mm, clear_mm=clear_mm
        )
        self._start(gcmd, index, start, wait)

    cmd_ACEK_LOAD_help = (
        "Pull the filament of LANE=<1-4> in to the parking point (LENGTH=<mm>, SPEED=)"
    )

    def cmd_ACEK_LOAD(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", self.load_park_mm, above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.load_speed_mm_s)
        start = Start("load", length, speed, expected_mm=self._load_budget_mm(length))
        self._start(gcmd, index, start, gcmd.get_int("WAIT", 1))

    cmd_ACEK_STOP_help = "Stop the motor of LANE=<1-4>, or every lane"

    def cmd_ACEK_STOP(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = LANE_ALL if gcmd.get("LANE", None) is None else self._lane(gcmd)
        self.stop(index, by="command")
        which = "every lane" if index == LANE_ALL else f"lane {index + 1}"
        gcmd.respond_info(f"ace2k: stop sent to {which}")

    cmd_ACEK_CLEAR_help = "Leave the error state on LANE=<1-4>"

    def cmd_ACEK_CLEAR(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        self.clear(index)
        gcmd.respond_info(f"ace2k: lane {index + 1} cleared")

    cmd_ACEK_SPEED_help = "Change the speed of a running move on LANE=<1-4>: SPEED=<mm/s>"

    def cmd_ACEK_SPEED(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        speed = self._speed(gcmd, self.feed_speed_mm_s)
        try:
            self.set_speed(index, speed)
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from e
        except FeedRefused as e:
            text = SPEED_REFUSAL_TEXT.get(e.reason, e.reason)
            raise gcmd.error(f"ace2k: lane {index + 1} speed refused: {text}") from e
        gcmd.respond_info(f"ace2k: lane {index + 1} speed {speed:g} mm/s")
