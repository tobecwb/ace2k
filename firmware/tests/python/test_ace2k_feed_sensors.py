"""The insert sensors gone stale (the ADC scan stopped): the unit stops every running lane with
``stopped_sensors`` and refuses every start with ``sensors_stale`` (API_VERSION 24). The host
names both, says one line per stopped lane, fails a waited move and a waited tag search, and the
follow is not armed again on its own."""

from __future__ import annotations

import ace2k_feed
import ace2k_follow
import pytest
from test_ace2k_extras import RESPONSES, FakeGcmd
from test_ace2k_feed import accepted, event_params, feed_of, make_feed
from test_ace2k_rfid import feed_event, make_rfid, queue_calls, rfid_of

KIND_STOPPED_SENSORS = 21
REFUSED_SENSORS_STALE = 9
MODE_FOLLOWING = 8


def test_the_kind_and_the_refusal_have_their_names() -> None:
    assert ace2k_feed.KIND_NAMES[KIND_STOPPED_SENSORS] == "stopped_sensors"
    assert ace2k_feed.REFUSAL_NAMES[REFUSED_SENSORS_STALE] == "sensors_stale"
    assert ace2k_feed.STALE_SENSORS_KIND == "stopped_sensors"
    # a final kind, with no error state on the lane
    assert KIND_STOPPED_SENSORS not in ace2k_feed.ERROR_KINDS
    assert KIND_STOPPED_SENSORS not in ace2k_feed.NOTICE_KINDS
    # the follow is not armed again after it
    assert "stopped_sensors" not in ace2k_follow.REARM_KINDS


def test_a_stopped_follow_says_why_in_one_line() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    mcu.subscriptions["ace2k_feed_event"](
        event_params(1, KIND_STOPPED_SENSORS, MODE_FOLLOWING, 52000, 51000, seq=4)
    )
    assert feed.lanes[1]["last_move"]["kind"] == "stopped_sensors"
    assert printer.objects["gcode"].output == [
        "ace2k: lane 2 stopped (following) — insert sensors stale (the ADC scan stopped):"
        " lanes stopped, starts refused"
    ]


def test_a_waited_move_that_ends_stopped_sensors_is_a_gcode_error() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = accepted(
        mcu, event_params(0, KIND_STOPPED_SENSORS, 1, 20000, 19000, seq=1)
    )
    with pytest.raises(Exception, match="lane 1 feed failed: stopped_sensors — motor 20.0 mm"):
        feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))


def test_a_start_refused_for_stale_sensors_names_the_reason() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": REFUSED_SENSORS_STALE}
    with pytest.raises(Exception, match=r"^ace2k: lane 1 feed refused: sensors_stale$"):
        feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))


def test_a_tag_search_that_ends_stopped_sensors_is_an_error() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_event(module, 0, "stopped_sensors"))
    with pytest.raises(Exception, match="tag search failed: stopped_sensors"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )
    assert rfid_of(module).lanes[0]["state"] == "pending"
