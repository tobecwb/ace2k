"""The follow on by default (ace2k_follow.py) and the two host rules it leans on: one start in
flight per lane, and the stop's author on every stopped event."""

from __future__ import annotations

from typing import TYPE_CHECKING, NoReturn

import ace2k_feed
import ace2k_follow
import pytest
from test_ace2k_extras import FakeCommandError, FakeGcmd
from test_ace2k_feed import event_params, make_follow, state_frame
from test_ace2k_ff import make_ff, set_modes

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence
    from typing import Any

    from ace2k_feed import Ace2kFeed
    from ace2k_klipper_types import Params
    from test_ace2k_extras import FakeMcu, FakePrinter, Responses

    # A connected feed with the follow: (feed, printer, mcu, responses)
    Rig = tuple[Ace2kFeed, FakePrinter, FakeMcu, Responses]


def start_queries(mcu: FakeMcu) -> list[list[Any]]:
    return [data for name, data in mcu.queries if name == "ace2k_feed_start"]


# --- one start in flight per lane ----------------------------------------------------------------


def test_a_second_start_on_a_lane_while_its_start_is_in_flight_is_refused_busy_unsent() -> None:
    # the start-up race: a second trigger arms the lane during the first start's round trip
    feed, printer, mcu, responses = make_follow()
    nested = []

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        if not nested:
            try:
                feed.start_move(lane, "assist_both", 0.0, 70.0)
                nested.append("sent")
            except ace2k_feed.FeedRefused as err:
                nested.append(err.reason)
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    assert nested == ["busy"]
    assert len(start_queries(mcu)) == 1
    assert move.seq == 1 and feed.motion[0].tracked[0] == 1  # the first start's slot is untouched
    # once answered, the lane takes a start again (the unit decides)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "feed", 10.0, 30.0)
    assert len(start_queries(mcu)) == 2


def test_another_lane_is_not_held_by_a_start_in_flight() -> None:
    feed, printer, mcu, responses = make_follow()
    nested = []

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        if lane == 0 and not nested:
            nested.append(feed.start_move(1, "assist_both", 0.0, 70.0))
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    feed.start_move(0, "assist_both", 0.0, 70.0)
    assert [q[0] for q in start_queries(mcu)] == [0, 1] and nested


def test_a_start_that_failed_leaves_the_lane_free() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 3}
    with pytest.raises(ace2k_feed.FeedRefused):
        feed.start_move(0, "feed", 10.0, 30.0)
    responses["ace2k_feed_start"] = RuntimeError("anything else")
    with pytest.raises(RuntimeError):
        feed.start_move(0, "feed", 10.0, 30.0)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.start_move(0, "feed", 10.0, 30.0)
    assert len(start_queries(mcu)) == 3 and not any(m.inflight for m in feed.motion)


def test_a_gcode_start_during_an_interface_start_is_a_gcode_error_naming_busy() -> None:
    feed, printer, mcu, responses = make_follow()
    caught = []

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        if not caught:
            with pytest.raises(Exception, match="lane 1 feed refused: busy") as err:
                feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
            caught.append(err)
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    feed.start_move(0, "assist_both", 0.0, 70.0)
    assert caught and len(start_queries(mcu)) == 1


# --- who stopped the lane ------------------------------------------------------------------------


def stopped_event(feed: Ace2kFeed, mcu: FakeMcu, lane: int, seq: int) -> Params:
    mcu.subscriptions["ace2k_feed_event"](event_params(lane, 1, 8, 1000, 1000, seq=seq))
    return feed.lanes[lane]["last_event"]


def test_a_stopped_event_names_the_stop_by_a_gcode_or_by_the_host() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    assert stopped_event(feed, mcu, 0, move.seq)["stop_by"] == "command"
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "OFF": "1"}))
    assert stopped_event(feed, mcu, 0, move.seq)["stop_by"] == "command"
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    feed.stop(0)  # a host module's
    event = stopped_event(feed, mcu, 0, move.seq)
    assert event["stop_by"] == "host" and move.result()["stop_by"] == "host"
    # the Klipper event carries the same dict
    assert printer.events[-1] == ("ace2k:feed_event", (0, dict(event, unit=None)))


def running(feed: Ace2kFeed, responses: Responses, lane: int, mode: str = "feed") -> int:
    """A start on the lane, accepted: its motion runs; returns its seq."""
    responses["ace2k_feed_start"] = {"lane": lane, "accepted": 1, "reason": 0}
    return feed.start_move(lane, mode, 50.0, 30.0).seq


def test_a_stop_of_every_lane_labels_only_the_lanes_running_something() -> None:
    # ACEK_STOP with lane 3 idle leaves no label there; a later stopped on lane 3 that
    # no stop of this session sent (stale) carries none
    feed, printer, mcu, responses = make_follow()
    seq = running(feed, responses, 1)
    feed.cmd_ACEK_STOP(FakeGcmd())  # every lane, by a G-code
    assert stopped_event(feed, mcu, 1, seq)["stop_by"] == "command"
    assert feed.motion[2].stop_by is None
    later = running(feed, responses, 2)
    assert stopped_event(feed, mcu, 2, later)["stop_by"] is None  # nobody here stopped it
    again = running(feed, responses, 2)
    feed.stop(2)  # a host stop of the running motion
    assert stopped_event(feed, mcu, 2, again)["stop_by"] == "host"


def test_an_operators_stop_is_never_downgraded_by_a_host_stop_before_its_event() -> None:
    feed, printer, mcu, responses = make_follow()
    seq = running(feed, responses, 0, "assist_both")
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    feed.stop(0)  # the adapter's, within the round trip
    assert stopped_event(feed, mcu, 0, seq)["stop_by"] == "command"
    seq = running(feed, responses, 0, "assist_both")
    feed.stop(0)
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))  # the operator after the host: the operator's
    assert stopped_event(feed, mcu, 0, seq)["stop_by"] == "command"


def test_a_label_belongs_to_the_seq_it_stopped() -> None:
    feed, printer, mcu, responses = make_follow()
    seq = running(feed, responses, 0)
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    assert stopped_event(feed, mcu, 0, seq - 1 or 255)["stop_by"] is None  # an older motion's
    assert stopped_event(feed, mcu, 0, seq)["stop_by"] == "command"


def test_the_hosts_own_stop_after_a_start_with_no_answer_is_the_hosts() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = FakeCommandError("Unable to obtain 'ace2k_feed_start_response'")
    with pytest.raises(RuntimeError, match="stopped in case it started"):
        feed.start_move(0, "assist_both", 0.0, 70.0)
    assert stopped_event(feed, mcu, 0, 1)["stop_by"] == "host"


def test_only_stopped_carries_an_author_and_the_motions_final_event_takes_it() -> None:
    feed, printer, mcu, responses = make_follow()
    seq = running(feed, responses, 0)
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 7, 8, 1000, 1000, seq=seq))  # stuck
    assert feed.lanes[0]["last_event"]["stop_by"] is None
    assert feed.motion[0].stop_by is None  # that stop was answered by the motion's end
    seq = running(feed, responses, 0)
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 15, 8, 1000, 1000, seq=seq))  # notice
    assert stopped_event(feed, mcu, 0, seq)["stop_by"] == "command"  # a notice takes nothing


# --- the follow on by default ------------------------------------------------------------------

IDLE, LOADING, ERROR, FOLLOWING = 0, 4, 7, 8


def started(mcu: FakeMcu) -> list[tuple[int, int, int]]:
    """The starts sent, as (lane, mode, speed_um_s)."""
    return [(d[0], d[1], d[3]) for d in start_queries(mcu)]


def rig(
    options: dict[str, Any] | None = None,
    inserts: Sequence[bool] = (True, True, True, True),
    extruders: dict[int, str] | None = None,
) -> Rig:
    """A connected feed with the follow, klippy:ready fired, the inserts set; every start
    accepted for its lane; extruders: lane -> the extruder a host module maps it to."""
    feed, printer, mcu, responses = make_follow(options=options)

    def respond() -> dict[str, int]:
        return {"lane": mcu.queries[-1][1][0], "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    printer.shutdown = False
    printer.is_shutdown = lambda: printer.shutdown  # Klipper's printer.is_shutdown()
    for lane, name in (extruders or {}).items():
        feed.set_lane_extruder(lane, name)
    printer.handlers["klippy:ready"]()
    for lane, insert in enumerate(inserts):
        feed.unit.lanes[lane]["insert"] = insert
    return feed, printer, mcu, responses


def report(
    mcu: FakeMcu, modes: Sequence[int] = (IDLE,) * 4, errors: Sequence[int] = (0,) * 4
) -> None:
    mcu.subscriptions["ace2k_feed_state"](
        state_frame(list(modes), list(errors), [0] * 4, [0] * 4, [0] * 4)
    )


def deliver(mcu: FakeMcu, lane: int, kind: int, mode: int, seq: int = 0) -> None:
    mcu.subscriptions["ace2k_feed_event"](event_params(lane, kind, mode, 1000, 1000, seq=seq))


def test_at_the_start_every_lane_with_filament_idle_is_armed_once() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, True, False, True))
    report(mcu, modes=(IDLE, IDLE, IDLE, ERROR), errors=(0, 0, 0, 7))
    assert started(mcu) == [(0, 6, 50000), (1, 6, 50000)]  # at assist_speed, the follow
    assert [feed.lane_status(i)["follow_default"] for i in range(4)] == [
        "armed",
        "armed",
        "off",  # empty
        "off",  # in error: never over it
    ]
    report(mcu, modes=(FOLLOWING, FOLLOWING, IDLE, IDLE))  # the error cleared meanwhile
    report(mcu, modes=(FOLLOWING, FOLLOWING, IDLE, IDLE))
    assert len(started(mcu)) == 2  # once per episode; the cleared lane waits for a reinsertion
    assert feed.default_move(0).seq == 1 and feed.default_move(2) is None


def test_the_console_names_each_arm() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    assert "ace2k: lane 1 follow on (follow_default)" in printer.objects["gcode"].output


def test_the_default_keeps_one_record_per_lane_its_episode_open_as_configured() -> None:
    for enabled in (True, False):
        feed, printer, mcu, _ = rig(options={"follow_default": enabled})
        lanes = feed.follow_auto.lanes
        assert len(lanes) == ace2k_feed.LANES and len({id(lane) for lane in lanes}) == len(lanes)
        assert all(lane.want is enabled and lane.on_insert for lane in lanes)
        feed.set_lane_follow(1, 12.5, on_insert=False)
        assert lanes[1].speed == 12.5 and not lanes[1].on_insert
        assert lanes[0].speed is None and lanes[0].on_insert  # the other lanes are their own


def test_follow_default_off_arms_nothing() -> None:
    feed, printer, mcu, _ = rig(options={"follow_default": False})
    assert feed.follow_default is False
    for _ in range(4):
        report(mcu)
    assert started(mcu) == []
    assert feed.lane_status(0)["follow_default"] == "off"


def test_nothing_is_armed_before_klippy_ready() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    printer.is_shutdown = lambda: False
    feed.unit.lanes[0]["insert"] = True
    report(mcu)
    assert started(mcu) == []
    printer.handlers["klippy:ready"]()
    report(mcu)
    assert started(mcu)[0][:2] == (0, 6)


def test_a_lane_busy_at_the_start_is_armed_once_its_move_ended_and_a_report_shows_it_idle() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    assert started(mcu) == [] and feed.lane_status(0)["follow_default"] == "waiting"
    deliver(mcu, 0, 5, LOADING)  # loaded, the unit's own (seq 0)
    assert started(mcu) == []  # nothing is looked at before the next report
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]


def test_an_insertion_with_the_units_own_load_is_armed_after_that_load() -> None:
    feed, printer, mcu, _ = rig(inserts=(False, False, False, False))
    report(mcu)
    feed.unit.lanes[0]["insert"] = True
    report(mcu)  # the edge: the report still idle, the unit's load starting
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    assert started(mcu) == []
    deliver(mcu, 0, 5, LOADING)
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]


def test_an_insertion_without_a_load_waits_two_reports() -> None:
    feed, printer, mcu, _ = rig(inserts=(False, False, False, False))
    report(mcu)
    feed.unit.lanes[0]["insert"] = True
    report(mcu)
    report(mcu)
    assert started(mcu) == []
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]


def test_a_filament_pushed_in_during_the_tail_is_no_insertion() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    seq = feed.default_move(0).seq
    feed.unit.lanes[0]["insert"] = False  # ran out: the tail
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    feed.unit.lanes[0]["insert"] = True  # a filament pushed in at the bay
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    deliver(mcu, 0, 17, FOLLOWING, seq=seq)  # tail_out
    for _ in range(4):
        report(mcu)
    assert len(started(mcu)) == 1
    feed.unit.lanes[0]["insert"] = False  # pulled out and pushed in again: an insertion
    report(mcu)
    feed.unit.lanes[0]["insert"] = True
    for _ in range(3):
        report(mcu)
    assert len(started(mcu)) == 2


@pytest.mark.parametrize(
    "manual",
    [
        lambda feed: feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"})),
        lambda feed: feed.cmd_ACEK_STOP(FakeGcmd()),
        lambda feed: feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "OFF": "1"})),
        lambda feed: feed.stop(0),  # a host module's
        lambda feed: feed.start_move(0, "unload", 0.0, 30.0),  # a host module's flow
        lambda feed: feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"})),
    ],
)
def test_any_other_start_or_stop_closes_the_episode_until_a_reinsertion(
    manual: Callable[[Ace2kFeed], object],
) -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))  # not armed yet
    manual(feed)
    sent = len(started(mcu))
    deliver(mcu, 0, 1, LOADING)  # whatever ran ended
    for _ in range(4):
        report(mcu)
    assert len(started(mcu)) == sent and feed.lane_status(0)["follow_default"] == "off"
    feed.unit.lanes[0]["insert"] = False
    report(mcu)
    feed.unit.lanes[0]["insert"] = True
    for _ in range(3):
        report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000) and len(started(mcu)) == sent + 1


def test_a_stop_of_the_armed_follow_by_hand_leaves_it_off_until_a_reinsertion() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    seq = feed.default_move(0).seq
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    deliver(mcu, 0, 1, FOLLOWING, seq=seq)
    assert feed.default_move(0) is None
    for _ in range(4):
        report(mcu)
    assert len(started(mcu)) == 1


def test_a_lane_taken_by_a_module_is_armed_at_the_start_only_at_its_speed() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    feed.set_lane_follow(0, 70.0, on_insert=False)
    feed.set_lane_follow(1, 70.0, on_insert=False)
    report(mcu)
    assert started(mcu) == [(0, 6, 70000)]
    feed.unit.lanes[1]["insert"] = True  # an insertion on a lane the module took
    for _ in range(4):
        report(mcu)
    assert started(mcu) == [(0, 6, 70000)]
    feed.set_lane_follow(1, on_insert=True)  # given back: the next insertion is ace2k's
    feed.unit.lanes[1]["insert"] = False
    report(mcu)
    feed.unit.lanes[1]["insert"] = True
    for _ in range(3):
        report(mcu)
    assert started(mcu)[-1] == (1, 6, 50000)


def test_set_lane_follow_refuses_a_bad_lane_or_speed() -> None:
    feed, printer, mcu, _ = rig()
    for args in ((4,), (-1,), (0, 0.0), (0, -5.0)):
        with pytest.raises(ValueError):
            feed.set_lane_follow(*args)
    with pytest.raises(ValueError):
        feed.default_move(4)


def test_a_busy_refusal_is_tried_again_and_another_one_closes_with_a_line() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, True, False, False))
    replies = {0: [1, 0], 1: [3]}  # lane 1: busy, then accepted; lane 2: no_filament

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        reason = replies[lane].pop(0)
        return {"lane": lane, "accepted": int(reason == 0), "reason": reason}

    responses["ace2k_feed_start"] = respond
    report(mcu)
    assert feed.lane_status(0)["follow_default"] == "waiting"
    assert feed.lane_status(1)["follow_default"] == "off"
    report(mcu)
    assert feed.lane_status(0)["follow_default"] == "armed"
    assert [lane for lane, _m, _s in started(mcu)] == [0, 1, 0]
    assert (
        "ace2k: lane 2 follow not armed (follow_default) — no_filament"
        in printer.objects["gcode"].output
    )


def test_no_answer_closes_the_episode_with_a_line_and_the_lane_is_stopped() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    responses["ace2k_feed_start"] = FakeCommandError("Unable to obtain 'ace2k_feed_start_response'")
    report(mcu)
    lines = [m for m in printer.objects["gcode"].output if "not armed" in m]
    assert len(lines) == 1 and lines[0].startswith(
        "ace2k: lane 1 follow not armed (follow_default)"
    )
    assert "no answer from the unit" in lines[0] and "ace2k: ace2k:" not in lines[0]
    report(mcu)
    assert len(start_queries(mcu)) == 1


def test_the_status_shows_arming_while_the_start_is_in_flight() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    seen = []

    def respond() -> dict[str, int]:
        seen.append(feed.lane_status(0)["follow_default"])
        return {"lane": 0, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    report(mcu)
    assert seen == ["arming"]


def test_a_shutdown_arms_nothing() -> None:
    feed, printer, mcu, _ = rig()
    printer.shutdown = True
    report(mcu)
    assert started(mcu) == []


def test_a_read_with_motion_closes_the_episode() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    feed.follow_auto.note_start(0)  # what ace2k_rfid's MOVE=1 read does before it sends
    deliver(mcu, 0, 5, LOADING)
    report(mcu)
    assert started(mcu) == []


# --- a G-code move on a lane in the follow --------------------------------------------------------

FEEDING, UNLOADING = 1, 3


def following_rig(
    options: dict[str, Any] | None = None,
    inserts: Sequence[bool] = (True, False, False, False),
) -> Rig:
    """Lane 1 armed by the default and reported following; a stop sent to a lane brings that
    lane's stopped event at once (the unit's answer), with the seq of its last start."""
    feed, printer, mcu, responses = rig(options=options, inserts=inserts)
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = feed.stop

    def stop_and_answer(lane_index: int, by: str = "host") -> None:
        stop(lane_index, by)
        lanes = range(4) if lane_index == ace2k_feed.LANE_ALL else [lane_index]
        for lane in lanes:
            if feed.lanes[lane]["mode"] in ("following", "feeding"):
                deliver(mcu, lane, 1, FOLLOWING, seq=feed.motion[lane].seq)

    feed.stop = stop_and_answer
    return feed, printer, mcu, responses


def lines(printer: FakePrinter) -> list[str]:
    return printer.objects["gcode"].output


def gcmd_lines(gcmd: object) -> list[str]:
    return getattr(gcmd, "lines", [])


def test_a_feed_on_a_following_lane_pauses_the_follow_moves_and_arms_it_again() -> None:
    feed, printer, mcu, _ = following_rig()
    follow_seq = feed.motion[0].seq
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)
    assert started(mcu)[-1] == (0, 0, 30000)  # the feed, after the follow's stop
    assert feed.lanes[0]["last_event"]["stop_by"] == "host"  # never a stop by hand
    assert feed.lanes[0]["last_event"]["seq"] == follow_seq
    assert "ace2k: lane 1 follow paused for the feed" in gcmd_lines(gcmd)
    report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    assert len(started(mcu)) == 2  # not while the feed runs
    deliver(mcu, 0, 0, FEEDING, seq=feed.motion[0].seq)  # done
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)
    assert "ace2k: lane 1 follow on again" in lines(printer)
    assert feed.default_move(0) is not None


def test_a_waited_feed_whose_end_comes_within_the_round_trip_still_rearms() -> None:
    feed, printer, mcu, responses = following_rig()

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        seq = mcu.queries[-1][1][4]
        deliver(mcu, lane, 0, FEEDING, seq=seq)  # a short move: its done before the answer
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "5"}))
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)


def test_an_unload_stops_the_follow_and_does_not_arm_it_again() -> None:
    feed, printer, mcu, _ = following_rig()
    gcmd = FakeGcmd({"LANE": "1", "WAIT": "0"})
    feed.cmd_ACEK_UNLOAD(gcmd)
    assert "ace2k: lane 1 follow stopped for the unload" in gcmd_lines(gcmd)
    assert started(mcu)[-1][:2] == (0, 4)
    deliver(mcu, 0, 6, UNLOADING, seq=feed.motion[0].seq)  # unloaded
    feed.unit.lanes[0]["insert"] = False
    for _ in range(3):
        report(mcu)
    assert len(started(mcu)) == 2
    feed.unit.lanes[0]["insert"] = True  # the next insertion arms, as always
    for _ in range(3):
        report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)


@pytest.mark.parametrize("kind, name", [(7, "stuck"), (18, "blocked"), (1, "stopped")])
def test_a_move_that_ends_in_error_blocked_or_stopped_leaves_the_follow_off(
    kind: int, name: str
) -> None:
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    deliver(mcu, 0, kind, FEEDING, seq=feed.motion[0].seq)
    for _ in range(3):
        report(mcu, modes=(ERROR if kind == 7 else IDLE, IDLE, IDLE, IDLE))
    assert len(started(mcu)) == 2
    assert f"ace2k: lane 1 follow stays off — the feed ended {name}" in lines(printer)


def test_acek_stop_during_the_paused_move_keeps_the_lane_off() -> None:
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))  # answered stopped, the feed's seq
    for _ in range(3):
        report(mcu)
    assert len(started(mcu)) == 2


def test_with_follow_default_off_the_move_stops_the_follow_and_does_not_arm_it_again() -> None:
    feed, printer, mcu, responses = rig(options={"follow_default": False}, inserts=(True,) * 4)
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))  # by hand
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = feed.stop
    feed.stop = lambda lane, by="host": (stop(lane, by), deliver(mcu, lane, 1, FOLLOWING, 1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)
    assert "ace2k: lane 1 follow stopped for the feed" in gcmd_lines(gcmd)
    deliver(mcu, 0, 0, FEEDING, seq=feed.motion[0].seq)
    for _ in range(3):
        report(mcu)
    assert [m for _l, m, _s in started(mcu)] == [6, 0]


def test_a_follow_that_does_not_end_is_an_error_and_nothing_starts() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))  # stop sent, its event never comes
    with pytest.raises(Exception) as err:
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    assert str(err.value) == (
        "ace2k: lane 1 feed not started: the follow was stopped but did not end within 5 s; it"
        " comes back once the lane is idle"
    )
    assert len(started(mcu)) == 1 and feed.motion[0].final_waiters == []
    # As the refusal says, the follow comes back: the next report that shows the lane idle arms it
    # again.
    deliver(mcu, 0, 1, FOLLOWING, seq=feed.motion[0].seq)  # the stop's event, late
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)
    assert "ace2k: lane 1 follow on again" in lines(printer)


def test_an_unload_whose_follow_does_not_end_is_an_error_and_the_follow_stays_off() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))  # stop sent, its event never comes
    with pytest.raises(Exception) as err:
        feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "1", "WAIT": "0"}))
    assert str(err.value) == (
        "ace2k: lane 1 unload not started: the follow was stopped but did not end within 5 s;"
        " it stays off"
    )
    assert len(started(mcu)) == 1 and not feed.follow_paused(0)


def test_a_refused_unload_ends_the_pause_and_does_not_give_the_follow_back() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 5}  # bounds
    gcmd = FakeGcmd({"LANE": "1", "WAIT": "0"})
    with pytest.raises(Exception, match="refused: bounds"):
        feed.cmd_ACEK_UNLOAD(gcmd)
    assert "ace2k: lane 1 follow stopped for the unload" in gcmd_lines(gcmd)
    assert not feed.follow_paused(0)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    for _ in range(3):
        report(mcu)
    assert started(mcu) == [(0, 6, 50000), (0, 4, 80000)]  # the follow is not armed again


def test_the_busy_of_the_lane_winding_down_is_retried() -> None:
    feed, printer, mcu, responses = following_rig()
    replies = [1, 1, 0]

    def respond() -> dict[str, int]:
        reason = replies.pop(0)
        return {"lane": 0, "accepted": int(reason == 0), "reason": reason}

    responses["ace2k_feed_start"] = respond
    before = printer.reactor.now
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    assert replies == [] and printer.reactor.now == pytest.approx(before + 0.5)


def test_a_refused_move_gives_the_follow_back() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 5}  # bounds
    with pytest.raises(Exception, match="refused: bounds"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)
    assert "ace2k: lane 1 follow on again" in lines(printer)


def test_the_follow_asked_again_on_a_following_lane_is_not_paused() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(Exception, match="busy"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    assert ace2k_feed.LANE_ALL not in [d[0] for f, d in mcu.commands if "stop" in f]
    assert not [f for f, d in mcu.commands if f.startswith("ace2k_feed_stop")]


def test_a_host_modules_start_is_not_paused_for() -> None:
    # the adapter's flows keep their own handling: a start through the interface on a lane in
    # the follow is the unit's busy refusal, no stop sent
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(ace2k_feed.FeedRefused):
        feed.start_move(0, "feed", 10.0, 30.0)
    assert not [f for f, d in mcu.commands if f.startswith("ace2k_feed_stop")]


# --- one active lane per extruder ----------------------------------------------------------------

SHARED = {0: "extruder", 1: "extruder"}  # lanes 1 and 2 feed one extruder


def test_lanes_sharing_an_extruder_wait_for_a_choice_and_the_others_arm() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED)
    report(mcu)
    report(mcu)
    assert [lane for lane, _m, _s in started(mcu)] == [2, 3]
    said = [m for m in lines(printer) if "share" in m]
    assert said == ["ace2k: lanes 1, 2 share extruder: choose one with ACEK_LANE_ACTIVE"]
    assert [feed.lane_status(i)["active"] for i in range(4)] == [False, False, True, True]
    assert feed.follow_auto.active_map() == {"extruder": None}
    assert "extruder: lanes 1, 2 — active none (ACEK_LANE_ACTIVE)" in feed.status_lines()
    assert feed.unit.get_status(0)["active_lanes"] == {"extruder": None}


def test_acek_lane_active_arms_the_chosen_lane_and_stops_the_other() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(True, True, False, False))
    report(mcu)
    gcmd = FakeGcmd({"LANE": "1"})
    feed.cmd_ACEK_LANE_ACTIVE(gcmd)
    assert started(mcu) == [(0, 6, 50000)]
    assert gcmd_lines(gcmd)[-1] == "ace2k: lane 1 active on extruder"
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = feed.stop
    feed.stop = lambda lane, by="host": (stop(lane, by), deliver(mcu, lane, 1, FOLLOWING, 1))
    gcmd = FakeGcmd({"LANE": "2"})
    feed.cmd_ACEK_LANE_ACTIVE(gcmd)
    assert gcmd_lines(gcmd)[-1] == "ace2k: lane 2 active on extruder; lane 1 off"
    assert feed.lanes[0]["last_event"]["stop_by"] == "command"
    assert started(mcu)[-1] == (1, 6, 50000)
    assert feed.follow_auto.active_map() == {"extruder": 2}
    assert "extruder: lanes 1, 2 — active lane 2" in feed.status_lines()
    # a reinsertion on the lane not chosen arms nothing
    report(mcu, modes=(IDLE, FOLLOWING, IDLE, IDLE))
    feed.unit.lanes[0]["insert"] = False
    report(mcu, modes=(IDLE, FOLLOWING, IDLE, IDLE))
    feed.unit.lanes[0]["insert"] = True
    for _ in range(3):
        report(mcu, modes=(IDLE, FOLLOWING, IDLE, IDLE))
    assert len(started(mcu)) == 2


def test_acek_lane_active_is_refused_without_filament_or_on_a_busy_lane() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(False, True, False, False))
    report(mcu, modes=(IDLE, FEEDING, IDLE, IDLE))
    with pytest.raises(Exception, match="lane 1 has no filament"):
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "1"}))
    with pytest.raises(Exception, match=r"lane 2 is busy \(feeding\)"):
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))
    assert started(mcu) == [] and feed.follow_auto.active_map() == {"extruder": None}


def test_acek_lane_active_is_refused_on_a_lane_in_error_or_without_the_follow() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED)
    report(mcu, modes=(ERROR, IDLE, IDLE, IDLE), errors=(7, 0, 0, 0))
    with pytest.raises(Exception, match=r"lane 1 is in error \(.*\); ACEK_CLEAR first"):
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "1"}))
    feed.has_follow = False  # an image without the follow
    with pytest.raises(Exception, match="has no follow"):
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))


def test_acek_lane_active_on_a_lane_with_its_own_extruder_or_none_simply_arms_it() -> None:
    feed, printer, mcu, _ = rig(extruders={2: "extruder2"})
    feed.cmd_ACEK_STOP(FakeGcmd())  # every lane off before the start's arm
    report(mcu)
    assert started(mcu) == []
    for lane, name in (("3", "extruder2"), ("4", "no extruder")):
        gcmd = FakeGcmd({"LANE": lane})
        feed.cmd_ACEK_LANE_ACTIVE(gcmd)
        assert gcmd_lines(gcmd)[-1] == f"ace2k: lane {lane} active on {name}"
    assert [lane for lane, _m, _s in started(mcu)] == [2, 3]


def test_a_following_lane_made_active_is_not_started_again() -> None:
    feed, printer, mcu, _ = rig(inserts=(True, False, False, False))
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "1"}))
    assert len(started(mcu)) == 1


def test_a_paused_move_on_a_shared_lane_not_chosen_does_not_rearm() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(True, True, False, False))
    feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))
    report(mcu, modes=(IDLE, FOLLOWING, IDLE, IDLE))
    units = feed.unit.units
    units.active["extruder"] = units.key(feed.unit, 0)  # the choice moved on, lane 2 not stopped
    stop = feed.stop
    feed.stop = lambda lane, by="host": (stop(lane, by), deliver(mcu, lane, 1, FOLLOWING, 1))
    gcmd = FakeGcmd({"LANE": "2", "LENGTH": "10", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)
    assert "ace2k: lane 2 follow stopped for the feed" in gcmd_lines(gcmd)
    deliver(mcu, 1, 0, FEEDING, seq=feed.motion[1].seq)
    for _ in range(3):
        report(mcu)
    assert [m for lane, m, _s in started(mcu) if lane == 1] == [6, 0]


def test_a_follow_start_on_a_shared_lane_not_chosen_is_refused_not_active_and_sends_nothing() -> (
    None
):
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(True, True, False, False))
    feed.cmd_ACEK_STOP(FakeGcmd())  # every lane off before the start's arm
    report(mcu)
    with pytest.raises(ace2k_feed.FeedRefused) as refused:
        feed.start_move(0, "assist_both", 0.0, 70.0)
    assert refused.value.reason == "not_active"
    assert str(refused.value) == "ace2k: lane 1 assist_both refused: not_active"
    assert started(mcu) == []
    # any other mode is not the follow: the lane is free for it
    feed.start_move(0, "feed", 10.0, 70.0)
    assert started(mcu) == [(0, 0, 70000)]
    # the lane chosen follows; the other one is still refused
    deliver(mcu, 0, 0, IDLE, seq=feed.motion[0].seq)
    report(mcu)
    feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))
    assert started(mcu)[-1] == (1, 6, 50000)
    with pytest.raises(ace2k_feed.FeedRefused, match="not_active"):
        feed.start_move(0, "assist_both", 0.0, 70.0)
    assert len(started(mcu)) == 2


def test_acek_assist_both_on_a_shared_lane_not_chosen_names_acek_lane_active() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(True, True, False, False))
    feed.cmd_ACEK_STOP(FakeGcmd())
    report(mcu)
    with pytest.raises(Exception) as refused:
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2", "DIR": "BOTH"}))
    assert str(refused.value) == (
        "ace2k: lane 2 shares extruder and is not its active lane: choose it with"
        " ACEK_LANE_ACTIVE LANE=2"
    )
    gcmd = FakeGcmd({"LANE": "2", "DIR": "BOTH", "SOFT": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert gcmd_lines(gcmd) == [
        "ace2k: lane 2 not armed — lane 2 shares extruder and is not its active lane: choose it"
        " with ACEK_LANE_ACTIVE LANE=2"
    ]
    # the one-way assists are not the follow
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2"}))
    assert started(mcu) == [(1, 2, 50000)]
    # nor is a lane with its own extruder held back
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "3", "DIR": "BOTH"}))
    assert started(mcu)[-1][:2] == (2, 6)


def test_the_feed_forward_never_sees_two_lanes_of_one_extruder_in_the_follow() -> None:
    feed, printer, mcu = make_ff(
        options={"lane1_extruder": "extruder", "lane2_extruder": "extruder"}
    )

    def respond() -> dict[str, int]:
        return {"lane": mcu.queries[-1][1][0], "accepted": 1, "reason": 0}

    mcu.responses["ace2k_feed_start"] = respond
    feed.unit.lanes[0]["insert"] = feed.unit.lanes[1]["insert"] = True
    set_modes(mcu, [0, 0, 0, 0])
    stop = feed.stop
    feed.stop = lambda lane, by="host": (stop(lane, by), deliver(mcu, lane, 1, FOLLOWING, 1))
    for chosen in (1, 2, 1):
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": str(chosen)}))
        set_modes(mcu, [FOLLOWING if lane == chosen - 1 else 0 for lane in range(4)])
        feed.ff._update(printer.reactor.now)
    out = printer.objects["gcode"].output
    assert not [m for m in out if "in follow — no feed-forward" in m]


def test_a_read_with_motion_on_a_following_lane_pauses_the_follow_and_arms_it_again() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_rfid_read"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    printer.objects["ace2k_rfid"].cmd_ACEK_RFID_READ(gcmd)
    assert "ace2k: lane 1 follow paused for the tag read" in gcmd_lines(gcmd)
    assert [d[:2] for name, d in mcu.queries if name == "ace2k_rfid_read"] == [[0, 1]]
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    deliver(mcu, 0, 5, LOADING)  # the read's motion ends loaded (the unit's own seq)
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)
    assert "ace2k: lane 1 follow on again" in lines(printer)


# --- the default follow around refused starts, ended follows and tag reads ----------------------


def stop_commands(mcu: FakeMcu) -> list[list[Any]]:
    return [d for f, d in mcu.commands if f.startswith("ace2k_feed_stop")]


def test_a_refused_start_does_not_lose_the_running_default_follow() -> None:
    # ACEK_ASSIST DIR=BOTH refused busy on a lane the default follows bumps the seq;
    # the default's follow still runs and is still handed over
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    move = feed.default_move(0)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(Exception, match="busy"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    assert feed.default_move(0) is move
    assert feed.lane_status(0)["follow_default"] == "armed"


@pytest.mark.parametrize("ending", [(17, "tail_out"), (9, "motor_stalled"), (0, "done")])
@pytest.mark.parametrize("gcode", ["unload", "feed"])
def test_a_follow_whose_end_already_came_is_nothing_to_pause(
    ending: tuple[int, str], gcode: str
) -> None:
    # The report still says following, but the follow's final event has come
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    report(mcu)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    seq = feed.motion[0].seq
    if ending[1] == "tail_out":
        deliver(mcu, 0, 16, FOLLOWING, seq=seq)  # the tail notice
    deliver(mcu, 0, ending[0], FOLLOWING, seq=seq)
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"})
    getattr(feed, f"cmd_ACEK_{gcode.upper()}")(gcmd)  # no wait for an end that came
    assert stop_commands(mcu) == []
    assert not [line for line in gcmd_lines(gcmd) if "follow" in line]
    assert started(mcu)[-1][:2] == (0, 4 if gcode == "unload" else 0)


def test_a_paused_tag_read_rearms_only_at_its_own_end() -> None:
    # The read with motion's end is the unit's load (seq 0, loading); an unrelated
    # done on the lane re-arms nothing
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_rfid_read"] = {"lane": 0, "accepted": 1, "reason": 0}
    printer.objects["ace2k_rfid"].cmd_ACEK_RFID_READ(
        FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    )
    deliver(mcu, 0, 0, FEEDING, seq=99)  # some other motion's done
    report(mcu)
    assert len(started(mcu)) == 1
    deliver(mcu, 0, 5, LOADING, seq=0)  # the read's load ends
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)


def test_a_paused_tag_read_refused_busy_is_retried_by_its_reason() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_rfid_read"] = [
        {"lane": 0, "accepted": 0, "reason": 1},
        {"lane": 0, "accepted": 1, "reason": 0},
    ]
    printer.objects["ace2k_rfid"].cmd_ACEK_RFID_READ(
        FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    )
    # the second, after the refusal, is a new read: a new seq
    sent = [d for name, d in mcu.queries if name == "ace2k_rfid_read"]
    assert [d[:2] for d in sent] == [[0, 1], [0, 1]] and sent[1][2] == sent[0][2] % 255 + 1


def test_retry_busy_retries_by_the_reason_only() -> None:
    feed, printer, mcu, _ = rig()
    tries = []

    def send() -> str:
        tries.append(1)
        if len(tries) < 3:
            raise ace2k_feed.FeedRefused(0, "feed", "busy")
        return "sent"

    assert feed.retry_busy(send) == "sent" and len(tries) == 3

    def busy_text() -> NoReturn:
        raise RuntimeError("refused: busy")  # the text, no reason: not retried

    with pytest.raises(RuntimeError):
        feed.retry_busy(busy_text)


def shared_following(responses_stop: bool = True) -> Rig:
    """Lanes 1 and 2 on one extruder, lane 1 made active and reported following."""
    feed, printer, mcu, responses = rig(extruders=SHARED, inserts=(True, True, False, False))
    report(mcu)
    feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "1"}))
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    return feed, printer, mcu, responses


def test_acek_lane_active_keeps_the_choice_when_the_other_follow_does_not_end() -> None:
    feed, printer, mcu, responses = shared_following()  # stops are never answered here
    sent = len(started(mcu))
    with pytest.raises(Exception) as err:
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))
    assert str(err.value) == (
        "ace2k: lane 2 not made active: lane 1 stopped but did not end within 5 s; nothing"
        " armed; extruder's active lane stays lane 1"
    )
    assert len(started(mcu)) == sent and feed.follow_auto.active_map() == {"extruder": 1}
    assert feed.motion[0].final_waiters == []


def test_acek_lane_active_keeps_the_choice_when_the_arm_fails() -> None:
    feed, printer, mcu, responses = shared_following()
    stop = feed.stop
    feed.stop = lambda lane, by="host": (stop(lane, by), deliver(mcu, lane, 1, FOLLOWING, 1))
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 0, "reason": 5}  # bounds
    with pytest.raises(Exception) as err:
        feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "2"}))
    assert str(err.value) == (
        "ace2k: lane 2 not made active — lane 2 assist_both refused: bounds; lanes 1 stopped;"
        " extruder's active lane stays lane 1"
    )
    assert feed.follow_auto.active_map() == {"extruder": 1}


def test_acek_lane_active_arms_at_the_lanes_own_follow_speed() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED, inserts=(True, True, False, False))
    feed.set_lane_follow(0, 70.0, on_insert=False)
    report(mcu)
    feed.cmd_ACEK_LANE_ACTIVE(FakeGcmd({"LANE": "1"}))
    assert started(mcu) == [(0, 6, 70000)]


def test_the_groups_are_built_again_only_when_the_lane_map_changes() -> None:
    feed, printer, mcu, _ = rig(extruders=SHARED)
    units = feed.unit.units
    first = units.groups()
    assert units.groups() is first  # built again only when the lane map changes
    feed.set_lane_extruder(2, "extruder")
    assert feed.follow_auto.groups() == {"extruder": [0, 1, 2]}


# --- lost final events, pauses, holds and insertions given back ----------------------------------


def test_a_follow_whose_final_event_was_lost_is_over_once_reports_show_the_lane_idle() -> None:
    # Two state reports after the start showing the lane idle end the motion
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    report(mcu)  # the default arms lane 1
    assert feed.lane_status(0)["follow_default"] == "armed"
    report(mcu)  # idle: its final event was lost on the way
    assert feed.open_seq(0) is not None  # one report may be older than the start
    report(mcu)
    assert feed.open_seq(0) is None and feed.default_move(0) is None
    assert feed.lane_status(0)["follow_default"] == "off"  # (c) no longer armed
    assert feed.gone_after_s == 2.0  # the rule's span, for a module that applies it too
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)  # (a) no pause, no 5 s wait
    assert stop_commands(mcu) == [] and started(mcu)[-1] == (0, 0, 30000)
    assert not [line for line in gcmd_lines(gcmd) if "follow" in line]


def test_a_pause_is_exposed_until_the_moves_own_end() -> None:
    # The guard a module checks before arming the lane itself
    feed, printer, mcu, responses = following_rig()
    seen = []

    def respond() -> dict[str, int]:
        seen.append(feed.follow_paused(0))
        return {"lane": 0, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    assert not feed.follow_paused(0)
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    assert seen == [True] and feed.follow_paused(0)
    report(mcu)  # an idle report while the paused move has not ended
    assert feed.follow_paused(0)
    deliver(mcu, 0, 0, FEEDING, seq=feed.motion[0].seq)
    assert not feed.follow_paused(0)


def test_a_one_way_assist_on_a_following_lane_stops_the_follow_for_good() -> None:
    # Once the assist ends, the default does not arm the follow again
    feed, printer, mcu, responses = following_rig()
    gcmd = FakeGcmd({"LANE": "1", "DIR": "BACK"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert "ace2k: lane 1 follow stopped for the assist" in gcmd_lines(gcmd)
    deliver(mcu, 0, 0, 6, seq=feed.motion[0].seq)  # the assist ends cleanly (say, by its own end)
    for _ in range(3):
        report(mcu)
    assert [m for _l, m, _s in started(mcu)] == [6, 3]


@pytest.mark.parametrize("how", ["insert_clears", "stop"])
def test_a_stale_tag_read_rearm_never_fires_at_a_later_auto_load(how: str) -> None:
    # The read's (0, loading) entry, its end lost; much later the unit's own load ends
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_rfid_read"] = {"lane": 0, "accepted": 1, "reason": 0}
    printer.objects["ace2k_rfid"].cmd_ACEK_RFID_READ(
        FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    )
    if how == "stop":
        feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    else:
        feed.unit.lanes[0]["insert"] = False
        report(mcu)
        feed.unit.lanes[0]["insert"] = True  # a new filament: the unit loads it itself
    assert not feed.follow_paused(0)
    report(mcu, modes=(LOADING, IDLE, IDLE, IDLE))
    deliver(mcu, 0, 5, LOADING, seq=0)  # that automatic load ends, much later
    report(mcu)
    assert "ace2k: lane 1 follow on again" not in lines(printer)


def test_a_start_accepted_that_is_not_the_paused_moves_own_clears_the_rearm() -> None:
    feed, printer, mcu, responses = following_rig()
    responses["ace2k_rfid_read"] = {"lane": 0, "accepted": 1, "reason": 0}
    printer.objects["ace2k_rfid"].cmd_ACEK_RFID_READ(
        FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    )
    feed.start_move(0, "feed", 10.0, 30.0)  # another start, accepted on the lane
    assert not feed.follow_paused(0)


def test_a_held_lane_is_never_armed_by_the_default() -> None:
    # A module says the lane's filament waits behind another piece
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    feed.hold_follow(0, True)
    for _ in range(3):
        report(mcu)
    assert started(mcu) == []
    feed.hold_follow(0, False)
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]
    with pytest.raises(ValueError):
        feed.hold_follow(4, True)


def test_a_held_lane_says_why_in_its_status() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, True, False, False))
    feed.hold_follow(0, True, "not in [ace2k_u1] lanes")
    feed.hold_follow(1, True)  # no reason given (a host before interface 21)
    report(mcu)
    assert started(mcu) == []
    assert feed.lane_status(0)["follow_why"] == "held: not in [ace2k_u1] lanes"
    assert feed.lane_status(1)["follow_why"] == "held by a host module"
    feed.hold_follow(0, False)
    assert feed.lane_status(0)["follow_why"] is None
    feed.hold_follow(0, True)  # held again without a reason: the old one is gone
    assert feed.lane_status(0)["follow_why"] == "held by a host module"


def test_follow_lanes_extruder_arms_only_the_lanes_that_feed_an_extruder() -> None:
    feed, printer, mcu, responses = rig(
        options={"follow_lanes": "extruder", "lane1_extruder": "extruder"},
        extruders={2: "extruder1"},  # a host module's map
    )
    report(mcu)
    assert started(mcu) == [(0, 6, 50000), (2, 6, 50000)]
    assert feed.lane_status(1)["follow_why"] == "feeds no extruder, follow_lanes: extruder"
    assert feed.lane_status(1)["follow_default"] == "waiting"
    # a lane mapped later is armed at the next report
    feed.set_lane_extruder(1, "extruder2")
    report(mcu)
    assert started(mcu)[-1] == (1, 6, 50000)
    assert feed.follow_auto.will_rearm(3) is False  # no extruder: a paused follow stays off


def test_follow_lanes_all_is_the_default_and_a_wrong_value_is_a_config_error() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]  # no extruder named, followed as before
    assert feed.lane_status(0)["follow_why"] is None
    with pytest.raises(Exception, match="follow_lanes"):
        make_follow(options={"follow_lanes": "some"})


def test_the_status_line_of_a_lane_says_its_follow_and_why_not() -> None:
    feed, printer, mcu, responses = rig(
        options={"follow_lanes": "extruder"},
        inserts=(True, True, True, True),
        extruders={0: "extruder", 2: "extruder1", 3: "extruder1"},
    )
    feed.hold_follow(0, True, "not in [ace2k_u1] lanes")
    report(mcu)
    lines = [line for line in feed.status_lines() if line.startswith("lane ")]
    assert [line.split(" last=")[1].split(" follow=")[1] for line in lines] == [
        "off (held: not in [ace2k_u1] lanes)",
        "off (feeds no extruder, follow_lanes: extruder)",
        "off (extruder1: no lane chosen, ACEK_LANE_ACTIVE)",
        "off (extruder1: no lane chosen, ACEK_LANE_ACTIVE)",
    ]
    feed.hold_follow(0, False)
    report(mcu)
    assert feed.status_lines()[0].endswith(" follow=armed")


def test_the_status_line_of_a_lane_another_start_follows_says_who_started_it() -> None:
    """A lane that follows by a host module's own start (ace2k_u1's), or by a G-code."""
    feed, printer, mcu, responses = rig(inserts=(True, True, True, True))
    feed.hold_follow(0, True, "not in [ace2k_u1] lanes")
    feed.hold_follow(1, True, "not in [ace2k_u1] lanes")
    feed.start_move(0, "assist_both", 0.0, 50.0)
    report(mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    assert feed.status_lines()[0].endswith(" follow=armed (by a host module)")
    # a follow started by hand
    feed.hold_follow(2, True, "not in [ace2k_u1] lanes")
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "3", "DIR": "both"}))
    report(mcu, modes=(FOLLOWING, IDLE, FOLLOWING, IDLE))
    assert feed.status_lines()[2].endswith(" follow=armed (by a G-code)")
    # a follow the unit started itself (seq 0), and one no start of this host explains
    report(mcu, modes=(FOLLOWING, FOLLOWING, FOLLOWING, IDLE))
    assert feed.status_lines()[1].endswith(" follow=armed (by the unit)")
    feed.motion[1].report_seq = 7  # a start of a host before a restart
    assert feed.status_lines()[1].endswith(" follow=armed (by an earlier start)")
    # ace2k's own callers name themselves
    feed.motion[1].report_seq = 0
    feed.hold_follow(3, True, "not in [ace2k_u1] lanes")
    feed.start_follow_as("ACEK_LANE_ACTIVE", 3, 50.0)
    report(mcu, modes=(FOLLOWING, FOLLOWING, FOLLOWING, FOLLOWING))
    assert feed.status_lines()[3].endswith(" follow=armed (by ACEK_LANE_ACTIVE)")
    # a feed accepted while the last report still says following is no follow
    deliver(mcu, 3, 1, FOLLOWING, seq=feed.motion[3].accepted_seq)  # done
    feed.start_move(3, "feed", 10.0, 20.0)
    assert feed.status_lines()[3].endswith(" follow=off (held: not in [ace2k_u1] lanes)")
    # its final event came before the next report: it is over
    deliver(mcu, 0, 1, FOLLOWING, seq=feed.motion[0].accepted_seq)  # done
    assert feed.status_lines()[0].endswith(" follow=off (held: not in [ace2k_u1] lanes)")
    # after a shutdown the stale mode says nothing
    printer.shutdown = True
    assert feed.status_lines()[1].endswith(" follow=off (held: not in [ace2k_u1] lanes)")
    # with follow_default off, a lane another module follows still says so
    off, _printer, off_mcu, _responses = rig(
        options={"follow_default": False}, inserts=(True, False, False, False)
    )
    off.start_move(0, "assist_both", 0.0, 50.0)
    report(off_mcu, modes=(FOLLOWING, IDLE, IDLE, IDLE))
    assert off.status_lines()[0].endswith(" follow=armed (by a host module)")


def test_a_start_during_another_lanes_named_start_keeps_its_own_owner() -> None:
    # ace2k's own start waits for its answer while a host module starts another lane
    feed, printer, mcu, responses = rig(inserts=(True, True, True, True))
    feed.hold_follow(0, True, "not in [ace2k_u1] lanes")
    feed.hold_follow(1, True, "not in [ace2k_u1] lanes")
    nested = []

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        if lane == 0 and not nested:
            nested.append(feed.start_move(1, "assist_both", 0.0, 50.0))
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    feed.start_follow_as("ACEK_LANE_ACTIVE", 0, 50.0)
    report(mcu, modes=(FOLLOWING, FOLLOWING, IDLE, IDLE))
    assert feed.status_lines()[0].endswith(" follow=armed (by ACEK_LANE_ACTIVE)")
    assert feed.status_lines()[1].endswith(" follow=armed (by a host module)")


def test_the_status_line_says_follow_default_off() -> None:
    feed, printer, mcu, responses = rig(options={"follow_default": False})
    assert feed.status_lines()[0].endswith(" follow=off (follow_default: False)")
    assert feed.lane_status(0)["follow_why"] == "follow_default: False"


def test_insertions_given_back_open_an_episode_for_a_lane_inserted_meanwhile() -> None:
    # Taken at connect; a filament inserted before the hook failed; given back
    feed, printer, mcu, responses = rig(inserts=(False, False, False, False))
    feed.set_lane_follow(0, 70.0, on_insert=False)
    report(mcu)
    feed.unit.lanes[0]["insert"] = True
    for _ in range(3):
        report(mcu)
    assert started(mcu) == []
    feed.set_lane_follow(0, None, on_insert=True)  # the hook failed: given back
    report(mcu)
    assert started(mcu) == [(0, 6, 50000)]


def test_the_events_state_is_applied_on_the_reactor_in_order_with_a_stop() -> None:
    # An event received while a stop is being sent — the serial thread writes none of
    # the state the stop reads; the reactor applies the event after the stop, which labels it
    feed, printer, mcu, responses = make_follow()
    seq = running(feed, responses, 0, "assist_both")
    queued = []
    printer.reactor.register_async_callback = lambda cb, waketime=None: queued.append(cb)
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 1, 8, 1000, 1000, seq=seq))
    assert feed.motion[0].final_seq != seq and feed.open_seq(0) == seq  # nothing applied yet
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))  # the reactor, before the event's turn
    for cb in queued:
        cb(0.0)
    assert feed.lanes[0]["last_event"]["stop_by"] == "command"
    assert feed.motion[0].stop_by is None and feed.open_seq(0) is None


# --- the insertion after an eject ----------------------------------------------------------------


def test_acek_stop_of_a_paused_move_still_says_the_follow_stays_off() -> None:
    # a WAIT=0 rollback on a following lane, ACEK_STOP 0.8 s later, then the unit's stopped
    # event: the operator is told the follow stays off
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "40", "SPEED": "20", "WAIT": "0"}))
    seq = feed.motion[0].seq
    report(mcu, modes=(2, IDLE, IDLE, IDLE))  # rolling_back
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    deliver(mcu, 0, 1, 2, seq=seq)  # the unit's stopped (rolling_back)
    assert feed.lanes[0]["last_event"]["stop_by"] == "command"
    assert "ace2k: lane 1 follow stays off — the rollback ended stopped" in lines(printer)
    assert not feed.follow_paused(0)  # the operator's stop lifts the hold at once
    for _ in range(3):
        report(mcu)
    assert [m for _l, m, _s in started(mcu)] == [6, 1]


def test_acek_stop_before_the_stopped_event_still_says_it() -> None:
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "40", "SPEED": "20", "WAIT": "0"}))
    seq = feed.motion[0].seq
    feed.stop = ace2k_feed.Ace2kFeed.stop.__get__(feed)  # the stop's event comes later
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    assert not [m for m in lines(printer) if "stays off" in m]
    deliver(mcu, 0, 1, 2, seq=seq)
    assert "ace2k: lane 1 follow stays off — the rollback ended stopped" in lines(printer)


def test_the_units_own_load_after_a_answered_start_is_no_lost_event(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # the eject's unload (start N) ended by its own event; the insertion's
    # automatic load is the unit's (seq 0) and ends loaded with seq 0 — start N must not be taken
    # for open again and "counted over"
    feed, printer, mcu, responses = rig(inserts=(False, False, False, True))
    report(mcu)
    seq = running(feed, responses, 3, "unload")
    deliver(mcu, 3, 6, UNLOADING, seq=seq)  # unloaded, its own end
    report(mcu)
    report(mcu, modes=(IDLE, IDLE, IDLE, LOADING))  # the insertion: the unit's own load
    deliver(mcu, 3, 5, LOADING, seq=0)  # loaded, the unit's seq
    with caplog.at_level("WARNING"):
        for _ in range(3):
            report(mcu)
    assert not [r for r in caplog.records if "without its final event" in r.getMessage()]
    assert feed.open_seq(3) is None


def test_a_real_loss_is_still_counted_over_after_a_seq_0_event(
    caplog: pytest.LogCaptureFixture,
) -> None:
    feed, printer, mcu, responses = rig(inserts=(False, False, False, True))
    report(mcu)
    running(feed, responses, 3, "feed")  # its end is lost
    deliver(mcu, 3, 5, LOADING, seq=0)  # some motion of the unit's own ends meanwhile
    with caplog.at_level("WARNING"):
        for _ in range(3):
            report(mcu)
    assert [r for r in caplog.records if "without its final event" in r.getMessage()]
    assert feed.open_seq(3) is None


def test_a_stop_label_of_a_start_counted_over_is_dropped(caplog: pytest.LogCaptureFixture) -> None:
    feed, printer, mcu, responses = rig(inserts=(False, False, False, True))
    report(mcu)
    seq = running(feed, responses, 3, "feed")  # its end, and its stop's, are lost
    feed.stop(3, by="command")
    assert feed.motion[3].stop_by == (seq, "command")
    with caplog.at_level("WARNING"):
        for _ in range(3):
            report(mcu)
    assert [r for r in caplog.records if "without its final event" in r.getMessage()]
    assert feed.motion[3].stop_by is None  # no later event of that start can answer it


def test_a_final_event_after_its_move_was_counted_over_still_arms_the_follow_again(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """The unit sends a due state report before the final event of the same pass. The move ran
    past GONE_REPORTS reports, so that idle report counts it over. The final event that comes
    next is still the end of that move: the follow that the feed paused is armed again."""
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    seq = feed.motion[0].seq
    for _ in range(3):
        report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    with caplog.at_level("INFO"):
        report(mcu)  # idle, its final event still on the way: counted over
        deliver(mcu, 0, 0, FEEDING, seq=seq)  # done, right after
    assert [r for r in caplog.records if "without its final event" in r.getMessage()]
    assert [r for r in caplog.records if "came after it was counted over" in r.getMessage()]
    report(mcu)
    assert started(mcu)[-1] == (0, 6, 50000)
    assert "ace2k: lane 1 follow on again" in lines(printer)


def test_a_stopped_event_after_its_move_was_counted_over_keeps_who_stopped_it() -> None:
    feed, printer, mcu, responses = rig(inserts=(True, False, False, False))
    report(mcu)
    seq = running(feed, responses, 0, "feed")
    for _ in range(3):
        report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    feed.stop(0, "host")
    report(mcu)  # idle before the stopped event: counted over
    deliver(mcu, 0, 1, FEEDING, seq=seq)  # stopped
    assert feed.lanes[0]["last_event"]["stop_by"] == "host"


def test_a_late_event_of_another_seq_restores_nothing() -> None:
    """A final event whose seq is not the counted-over start's restores no re-arm."""
    feed, printer, mcu, _ = following_rig()
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    seq = feed.motion[0].seq
    for _ in range(4):
        report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    report(mcu)
    deliver(mcu, 0, 0, FEEDING, seq=(seq % 255) + 1)  # another seq
    report(mcu)
    assert "ace2k: lane 1 follow on again" not in lines(printer)


def counted_over_feed(caplog: pytest.LogCaptureFixture | None = None) -> tuple[Rig, int]:
    """An ACEK_FEED that paused the follow, counted over by an idle report before its event."""
    rig_ = following_rig()
    feed, printer, mcu, _ = rig_
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    seq = feed.motion[0].seq
    for _ in range(3):
        report(mcu, modes=(FEEDING, IDLE, IDLE, IDLE))
    report(mcu)
    return rig_, seq


def test_a_stop_after_the_count_over_keeps_the_follow_off() -> None:
    (feed, printer, mcu, _), seq = counted_over_feed()
    feed.follow_auto.note_stop(0)  # ACEK_STOP on the lane, its own event lost
    deliver(mcu, 0, 0, FEEDING, seq=seq)  # the feed's late done
    report(mcu)
    assert "ace2k: lane 1 follow on again" not in lines(printer)


def test_a_late_event_after_the_next_report_ends_nothing(caplog: pytest.LogCaptureFixture) -> None:
    (feed, printer, mcu, _), seq = counted_over_feed()
    report(mcu)  # the window closes
    with caplog.at_level("INFO"):
        deliver(mcu, 0, 0, FEEDING, seq=seq)
    report(mcu)
    assert "ace2k: lane 1 follow on again" not in lines(printer)
    assert not [r for r in caplog.records if "came after it was counted over" in r.getMessage()]
    assert feed.motion[0].lost_seq is None


def test_a_new_start_accepted_after_the_count_over_closes_its_window() -> None:
    (feed, printer, mcu, responses), seq = counted_over_feed()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.start_move(0, "feed", 5.0, 30.0)
    assert feed.motion[0].lost_seq is None
    assert feed.follow_auto.lanes[0].lost_rearm is None


def test_a_stop_keeps_only_the_report_of_a_rearm_and_a_counted_over_move_as_it_was() -> None:
    feed, _printer, _mcu, _ = rig()
    follow = feed.follow_auto
    lane = follow.lanes[0]
    follow.paused(0, "feed", 7)
    lane.lost_rearm = ace2k_follow.Rearm("load", 5, None, True)
    follow.note_stop(0)
    assert lane.rearm == ("feed", 7, None, "report")
    assert lane.lost_rearm == ("load", 5, None, "report")
    follow.paused(0, "stop", 8, rearm=False)
    lane.lost_rearm = ace2k_follow.Rearm("load", 6, None, False)
    follow.note_stop(0)
    assert lane.rearm is None  # a stop's end has nothing left to say
    assert lane.lost_rearm == ("load", 6, None, False)
