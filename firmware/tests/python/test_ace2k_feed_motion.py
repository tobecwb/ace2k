"""ace2k_feed_motion.py: one lane's motion record — its sequence, its open motion, the starts it
gives back and the waits it completes — and the leaf rule of the module."""

from __future__ import annotations

from ace2k_feed_motion import SEQ_MAX, LaneMotion
from test_ace2k_link import EXTRAS, _sibling_imports

MOTION = EXTRAS / "ace2k_feed_motion.py"
BASE = EXTRAS / "ace2k_feed_base.py"
ACCEPTED, REPORTED, SENT = 7, 3, 9  # a start's seq the unit accepted, one a report showed, one sent


class FakeCompletion:
    """The reactor's completion: what it was completed with, in order."""

    def __init__(self) -> None:
        self.results: list[object] = []

    def complete(self, result: object) -> None:
        self.results.append(result)


def final(seq: int, kind: str = "done") -> dict[str, object]:
    return {"kind": kind, "mode": "feeding", "motor_mm": 10.0, "filament_mm": 9.9, "seq": seq}


def test_the_motion_module_imports_only_the_leaf_modules() -> None:
    assert {*_sibling_imports(MOTION)} <= {"ace2k_link", "ace2k_feed_proto", "ace2k_feed_base"}
    assert not _sibling_imports(BASE)  # the feed's shared declarations: a leaf


def test_the_sequence_wraps_from_255_to_1_and_is_never_0() -> None:
    lane = LaneMotion()
    assert lane.next_seq() == 1
    lane.seq = SEQ_MAX - 1
    assert lane.next_seq() == SEQ_MAX
    assert lane.next_seq() == 1
    assert all(lane.next_seq() != 0 for _ in range(600))


def test_two_lanes_never_share_state() -> None:
    first, second = LaneMotion(), LaneMotion()
    first.next_seq()
    first.started = first.inflight = True
    first.final_waiters.append(FakeCompletion())
    first.commanded_mm[1] = 100.0
    assert second == LaneMotion()
    assert second.final_waiters == [] and second.commanded_mm == {}


def test_the_open_motion_is_the_accepted_start_until_its_end_then_the_reported_one() -> None:
    lane = LaneMotion(accepted_seq=ACCEPTED, accepted_ended=False, report_seq=REPORTED)
    assert lane.open_seq("idle") == ACCEPTED  # its end not seen: open whatever the report says
    lane.accepted_ended = True
    assert lane.open_seq("feeding") == REPORTED  # the report shows a motion running
    for mode in (None, "idle", "error"):
        assert lane.open_seq(mode) is None
    lane.ended_since_report = True
    assert lane.open_seq("feeding") is None  # a final event since the report: over


def test_a_stop_ends_the_open_motion_else_the_start_in_flight() -> None:
    lane = LaneMotion(seq=SENT, report_seq=REPORTED)
    assert lane.stopped_seq("following") == REPORTED
    assert lane.stopped_seq("idle") is None
    lane.inflight = True
    assert lane.stopped_seq("idle") == SENT


def test_complete_resolves_the_tracked_wait_of_that_seq_once() -> None:
    waiting = FakeCompletion()
    lane = LaneMotion(tracked=(SENT, waiting))
    lane.complete(final(REPORTED))  # another seq: a stale event passes by
    assert waiting.results == [] and lane.tracked == (SENT, waiting)
    event = final(SENT)
    lane.complete(event)
    lane.complete(event)  # the slot was taken first: never twice
    assert waiting.results == [event] and lane.tracked is None


def test_give_back_restores_the_move_before_and_completes_it_if_its_end_came_meanwhile() -> None:
    earlier, refused = FakeCompletion(), FakeCompletion()
    lane = LaneMotion(tracked=(SENT, refused))
    lane.give_back(SENT, refused, (ACCEPTED, earlier))
    assert lane.tracked == (ACCEPTED, earlier) and earlier.results == []
    # the earlier move's end landed during the refused start's round trip
    ended = final(ACCEPTED)
    lane.final_event = ended
    lane.tracked = (SENT, refused)
    lane.give_back(SENT, refused, (ACCEPTED, earlier))
    assert earlier.results == [ended] and lane.tracked is None and refused.results == []


def test_give_back_of_a_start_with_nothing_before_frees_the_slot() -> None:
    refused = FakeCompletion()
    lane = LaneMotion(tracked=(SENT, refused), final_event=final(SENT))
    lane.give_back(SENT, refused, None)
    assert lane.tracked is None and refused.results == []


def test_give_back_leaves_a_slot_another_start_holds() -> None:
    mine, other = FakeCompletion(), FakeCompletion()
    lane = LaneMotion(tracked=(ACCEPTED, other))
    lane.give_back(SENT, mine, None)
    lane.give_back(SENT, None, None)  # an unwaited start registered nothing
    assert lane.tracked == (ACCEPTED, other)
