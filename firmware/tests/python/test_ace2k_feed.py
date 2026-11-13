"""ace2k_feed.py: the config and init commands, the subscriptions, the state and event decoding,
the eleven G-codes with the waiting and the errors, the calibration flow — against the fake
printer of test_ace2k_extras."""

from __future__ import annotations

import logging
import pathlib
import re
import struct
from typing import TYPE_CHECKING, NoReturn

import ace2k
import ace2k_feed
import ace2k_feed_proto as proto
import pytest
from test_ace2k_extras import (
    DEFAULT_CONSTANTS,
    RESPONSES,
    FakeCommandError,
    FakeCompletion,
    FakeConfig,
    FakeGcmd,
    lane_counters_frame,
    make,
    make_units,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence
    from typing import Any

    from ace2k import Ace2k
    from ace2k_feed import Ace2kFeed
    from ace2k_klipper_types import Params
    from test_ace2k_extras import FakeCommand, FakeMcu, FakePrinter, FakeQuery, Responses

OLD_START_FMT = "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u"  # before the seq

FEED_KNOWN = {
    "ace2k_version",
    "ace2k_linkproof_state",
    "ace2k_feed_start",
    "ace2k_feed_start_response",
    "ace2k_feed_speed_set",
    "ace2k_feed_speed_set_response",
    "ace2k_feed_state",
    "ace2k_feed_event",
}


def feed_of(module: Ace2k) -> Ace2kFeed:
    return module.feed


def make_feed(responses: Responses, **kwargs: object) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    """make() as if the session's first state report had arrived with every lane idle (the
    seeding: test_the_sequence_is_seeded_from_the_state_report_until_the_lanes_first_start)."""
    module, printer, mcu = make(responses, **kwargs)
    feed_of(module)._seed([0] * ace2k_feed.LANES)
    return module, printer, mcu


def nothing_tracked(feed: Ace2kFeed) -> bool:
    """No lane holds a waiting G-code's completion."""
    return all(m.tracked is None for m in feed.motion)


def state_frame(
    modes: Sequence[int],
    errors: Sequence[int],
    speeds_um_s: Sequence[int],
    duties: Sequence[int],
    bursts: Sequence[int],
    seqs: Sequence[int] = (0, 0, 0, 0),
) -> dict[str, bytes]:
    return {
        "mode": bytes(modes),
        "error": bytes(errors),
        "speed_um_s": struct.pack("<4I", *speeds_um_s),
        "duty_pct": bytes(duties),
        "bursts": struct.pack("<4H", *bursts),
        "seq": bytes(seqs),
    }


IDLE_FRAME = state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4)


def event_params(
    lane: int, kind: int, mode: int, motor_um: int, filament_um: int, seq: int = 0
) -> dict[str, int]:
    """An event frame; seq 0 is the unit's own (a motion it started itself), a host start's
    events carry its sequence — 1 for the first start on a lane."""
    return {
        "lane": lane,
        "kind": kind,
        "mode": mode,
        "motor_um": motor_um,
        "filament_um": filament_um,
        "seq": seq,
    }


def accepted(mcu: FakeMcu, *events: Params) -> Callable[[], dict[str, int]]:
    """The start's response; with events, the event handler fires them in order while the query
    is in flight (as the unit's events would arrive before a waiting G-code checks its
    completion)."""

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]  # the answer echoes the lane the start asked for
        for event in events:
            mcu.subscriptions["ace2k_feed_event"](event)
        return {"lane": lane, "accepted": 1, "reason": 0}

    return respond


def test_the_feed_reads_its_keys_from_its_own_units_section() -> None:
    # the unit builds its feed from its own section: [ace2k] or [ace2k <name>]
    (a, b), printer, mcus = make_units(options={"a": {"feed_speed": 40.0}})
    assert a.feed.section == "ace2k a" and a.feed.feed_speed_mm_s == 40.0
    assert b.feed.section == "ace2k b" and b.feed.feed_speed_mm_s == 30.0


def test_build_config_sends_scales_thresholds_load_and_query() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(
        responses,
        options={
            "lane2_encoder_scale": 1.25,
            "slip_allow_mm": 10,
            "load_park_mm": 380,
            "auto_load": False,
        },
    )
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    init = [c for c, is_init in mcu.config_cmds if is_init]
    # nothing in Klipper's config checksum: editing a key never resets the MCU
    assert cfg == []
    assert "ace2k_lane_scale_set lane=1 um_per_count_x10=12500" in init
    assert (
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=10000 stall_check_um=20000"
        in init
    )
    assert "ace2k_feed_load_set park_um=380000 speed_um_s=60000 auto_load=0" in init
    assert any(c.startswith("ace2k_feed_query rest_ticks=") for c in init)
    assert "ace2k_feed_state" in mcu.subscriptions and "ace2k_feed_event" in mcu.subscriptions
    assert feed.lanes[1]["encoder_scale"] == 1.25 and feed.lanes[1]["scale_source"] == "config"
    assert feed.lanes[0]["scale_source"] == "default"


def test_slip_allow_at_or_above_slip_check_is_a_config_error_naming_both_keys() -> None:
    # the pair as the unit sees it, at connect: an absent key is the image's default (50 / 15 mm)
    def connect(options: dict[str, Any]) -> None:
        module, printer, mcu = make_feed(dict(RESPONSES), options=options)
        mcu.config_callback()

    with pytest.raises(
        Exception,
        match=r"slip_allow_mm \(50\) must be under the image's default slip_check_mm \(50\)",
    ):
        connect({"slip_allow_mm": 50})  # equal to the default window
    with pytest.raises(Exception, match="the image's default slip_allow_mm.*slip_check_mm"):
        connect({"slip_check_mm": 10})  # under the default allowance
    with pytest.raises(Exception, match=r"slip_allow_mm \(60\).*slip_check_mm \(40\)"):
        connect({"slip_check_mm": 40, "slip_allow_mm": 60})
    # one under the other, both given: sent as asked
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"slip_check_mm": 40, "slip_allow_mm": 39}
    )
    mcu.config_callback()
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert (
        "ace2k_feed_thresholds_set slip_check_um=40000 slip_allow_um=39000 stall_check_um=20000"
        in init
    )


def test_the_thresholds_are_three_and_the_reserved_assist_stall_kind_still_decodes() -> None:
    # the timed assist stall is gone from the unit: no key, no argument on the wire
    assert set(ace2k_feed.THRESHOLD_DEFAULTS) == {
        "slip_check_mm",
        "slip_allow_mm",
        "stall_check_mm",
    }
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"stall_check_mm": 25, "assist_stall_ms": 500}
    )
    mcu.config_callback()
    sent = [c for c, is_init in mcu.config_cmds if c.startswith("ace2k_feed_thresholds_set")]
    assert sent == [
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=15000 stall_check_um=25000"
    ]
    # kind 11 is reserved on the wire, never sent; a unit that did would still be named
    assert ace2k_feed.KIND_NAMES[11] == "assist_stall"


def test_a_comparator_window_as_long_as_an_assists_burst_is_a_config_error() -> None:
    # the burst read from the dictionary: 100 mm by default, and whatever the firmware says
    for key in ("stall_check_mm", "slip_check_mm"):
        options = {key: 100}
        if key == "slip_check_mm":
            options["slip_allow_mm"] = 15
        module, printer, mcu = make_feed(dict(RESPONSES), options=options)
        with pytest.raises(Exception, match=rf"{key} \(100\) must be under 100 mm"):
            mcu.config_callback()
        module, printer, mcu = make_feed(dict(RESPONSES), options={key: 99})
        mcu.config_callback()
        init = [c for c, is_init in mcu.config_cmds if is_init]
        assert any(c.startswith("ace2k_feed_thresholds_set") and "=99000" in c for c in init)
    # a firmware with another burst: the bound follows it
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_ASSIST_BURST_UM=60000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"stall_check_mm": 60}, constants=constants
    )
    with pytest.raises(Exception, match=r"stall_check_mm \(60\) must be under 60 mm"):
        mcu.config_callback()


def test_the_defaults_sent_are_the_images_own_when_its_dictionary_names_them() -> None:
    constants = dict(
        DEFAULT_CONSTANTS,
        ACE2K_FEED_SLIP_CHECK_UM=40000,
        ACE2K_FEED_SLIP_ALLOW_UM=12000,
        ACE2K_FEED_STALL_CHECK_UM=18000,
    )
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    mcu.config_callback()
    sent = [c for c, _ in mcu.config_cmds if c.startswith("ace2k_feed_thresholds_set")]
    assert sent == [
        "ace2k_feed_thresholds_set slip_check_um=40000 slip_allow_um=12000 stall_check_um=18000"
    ]


def test_the_defaults_are_checked_against_the_images_burst_as_the_keys_are() -> None:
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_ASSIST_BURST_UM=45000)
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    with pytest.raises(
        Exception, match=r"the image's default slip_check_mm \(50\) must be under 45 mm"
    ):
        mcu.config_callback()


def test_without_a_threshold_key_the_defaults_go_out_at_every_connect() -> None:
    # a removed key reaches the unit as the default, with no MCU restart
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    sent = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_thresholds_set")]
    assert sent == [
        (
            "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=15000"
            " stall_check_um=20000",
            True,
        )
    ]


def test_a_firmware_without_feed_is_logged_and_the_gcodes_refuse(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), known=set(FEED_KNOWN) - {"ace2k_feed_start"})
    mcu.config_callback()
    feed = feed_of(module)
    assert not feed.present
    assert "without CONFIG_ACE2K_FEED" in caplog.text
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed.cmd_ACEK_FEED(gcmd)


def test_a_start_or_event_with_another_format_is_a_config_error_asking_for_a_reflash() -> None:
    # the unit runs a feed image from before the sequence: its start exists without seq=%c —
    # not "built without CONFIG_ACE2K_FEED", which would send the operator to Kconfig
    module, printer, mcu = make_feed(dict(RESPONSES), formats={"ace2k_feed_start": OLD_START_FMT})
    with pytest.raises(Exception, match="ace2k_feed_start has another format.*different commits"):
        mcu.config_callback()
    # the v0.12 start, without free_tip
    v012 = ace2k_feed.START_FMT.replace(" free_tip=%c", "")
    module, printer, mcu = make_feed(dict(RESPONSES), formats={"ace2k_feed_start": v012})
    with pytest.raises(Exception, match="ace2k_feed_start has another format.*different commits"):
        mcu.config_callback()
    # the same for a response this module subscribes to
    old_event = ace2k_feed.EVENT_FMT.replace(" seq=%c", "")
    module, printer, mcu = make_feed(dict(RESPONSES), formats={"ace2k_feed_event": old_event})
    with pytest.raises(Exception, match="ace2k_feed_event has another format.*different commits"):
        mcu.config_callback()
    # a report absent while the start is present is the same case: the binding declares the
    # start, the state and the event in one file, so this is another commit's firmware too —
    # said as such, with the parser's text and the same hint, not as "another format"
    for missing in ("ace2k_feed_event", "ace2k_feed_state", "ace2k_feed_speed_set_response"):
        module, printer, mcu = make_feed(dict(RESPONSES), known=set(FEED_KNOWN) - {missing})
        with pytest.raises(
            Exception,
            match=rf"{missing} is not in the unit's dictionary though ace2k_feed_start is"
            rf" \(Unknown command: {missing}\); .*different commits",
        ):
            mcu.config_callback()


def test_an_image_without_the_speed_answer_is_refused_before_anything_is_registered() -> None:
    # the probe sits beside the start's, before any subscription or config command of the feed
    module, printer, mcu = make_feed(
        dict(RESPONSES), known=set(FEED_KNOWN) - {"ace2k_feed_speed_set_response"}
    )
    with pytest.raises(Exception, match="ace2k_feed_speed_set_response is not in the unit"):
        mcu.config_callback()
    assert not [name for name in mcu.subscriptions if name.startswith("ace2k_feed")]
    assert not [cmd for cmd, _init in mcu.config_cmds if cmd.startswith("ace2k_feed")]


def test_a_speed_set_response_with_another_format_is_refused_at_connect() -> None:
    # an image from before the speed set answered has its command with the same format and no
    # response (the loop of the test above): a query would fail in the G-code, so the connect
    # names it; a response with another format is another commit's image as well
    old_resp = "ace2k_feed_speed_set_response lane=%c accepted=%c"
    module, printer, mcu = make_feed(
        dict(RESPONSES), formats={"ace2k_feed_speed_set_response": old_resp}
    )
    with pytest.raises(Exception, match="ace2k_feed_speed_set_response has another format"):
        mcu.config_callback()


def test_a_speed_outside_the_dictionary_bounds_is_a_config_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), options={"feed_speed": 150})
    with pytest.raises(Exception, match="feed_speed"):
        mcu.config_callback()


OLD_TOP = {**DEFAULT_CONSTANTS, "ACE2K_LANE_SPEED_MAX_UM_S": 70000}  # an image before 80 mm/s


def test_the_default_unload_speed_follows_an_older_images_top_speed() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=OLD_TOP)
    mcu.config_callback()  # no config error
    assert feed_of(module).unload_speed_mm_s == 70.0


def test_the_default_unload_speed_is_80_on_an_image_with_that_top_speed() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    assert feed_of(module).unload_speed_mm_s == 80.0


def test_an_unload_speed_set_above_an_older_images_top_speed_is_a_config_error() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"unload_speed": 80}, constants=OLD_TOP
    )
    with pytest.raises(Exception, match=r"unload_speed must be within 9\.\.70 mm/s"):
        mcu.config_callback()


@pytest.mark.parametrize(
    ("duty", "constants", "warned"),
    [
        (11, DEFAULT_CONSTANTS, True),
        (1, DEFAULT_CONSTANTS, True),
        (12, DEFAULT_CONSTANTS, False),
        (0, DEFAULT_CONSTANTS, False),
        (11, OLD_TOP, False),
    ],
)
def test_a_duty_guard_that_may_fire_at_the_top_speed_is_warned_about(
    caplog: pytest.LogCaptureFixture, duty: int, constants: dict[str, int], warned: bool
) -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"snag_duty_pct": duty}, constants=constants
    )
    with caplog.at_level(logging.WARNING):
        mcu.config_callback()  # never refused
    lines = [r.getMessage() for r in caplog.records if "snag_duty_pct" in r.getMessage()]
    assert len(lines) == (1 if warned else 0)
    if warned:
        assert "may fire on healthy moves" in lines[0]


def test_state_frame_lands_in_status() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([1, 7, 0, 5], [0, 7, 0, 0], [30000, 0, 0, 50000], [42, 0, 0, 60], [0, 0, 0, 12])
    )
    lanes = module.get_status(0)["lanes"]
    assert lanes[0]["mode"] == "feeding" and lanes[0]["speed"] == 30.0 and lanes[0]["duty"] == 42
    assert lanes[1]["mode"] == "error" and lanes[1]["error"] == "stuck"
    assert lanes[0]["error"] is None
    assert lanes[3]["mode"] == "assisting" and lanes[3]["bursts"] == 12
    gcmd = FakeGcmd()
    module.cmd_ACEK_STATUS(gcmd)
    assert any("mode=feeding" in line for line in gcmd.lines)


def test_an_event_is_logged_stored_and_raised_as_a_klipper_event(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    with caplog.at_level("INFO", logger="root"):  # the event line is an INFO log
        mcu.subscriptions["ace2k_feed_event"](event_params(2, 7, 1, 120000, 15000))
    ev = feed.lanes[2]["last_event"]
    assert ev == dict(
        kind="stuck",
        mode="feeding",
        motor_mm=120.0,
        filament_mm=15.0,
        seq=0,
        stop_by=None,
        tag_search=False,
    )
    assert feed.lanes[2]["last_move"]["commanded_mm"] is None  # nothing this host started
    assert "lane 3 stuck (feeding)" in caplog.text
    assert printer.events == [("ace2k:feed_event", (2, dict(ev, unit=None)))]
    assert "unit" not in ev and "unit" not in feed.lane_status(2)["last_event"]
    mcu.subscriptions["ace2k_feed_event"](event_params(2, 14, 5, 5000, 8000))
    assert feed.lanes[2]["last_event"]["kind"] == "behind"
    assert feed.lanes[2]["last_move"]["kind"] == "stuck"  # a notice is not a move's end


def test_feed_waits_for_done_and_reports_the_numbers() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 300000, 298500, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "300"})
    feed.cmd_ACEK_FEED(gcmd)
    assert mcu.sent[-1] == "ace2k_feed_start"
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 300.0 mm, filament 298.5 mm"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 300.0
    assert feed.lanes[0]["last_move"]["seq"] == 1
    assert nothing_tracked(feed)


def test_the_start_arguments_are_micrometres_and_the_mode_number() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, options={"feed_speed": 40})
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data: Sequence[Any], minclock: int = 0, reqclock: int = 0) -> dict[str, int]:
            sent.append(list(data))
            mcu.subscriptions["ace2k_feed_event"](event_params(data[0], 0, 1, 1, 1, data[4]))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    feed.start_query = Query()  # the start's query, built at connect
    # the last argument is the lane's sequence: 1 for its first start, 2 for the second
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "2", "LENGTH": "12.5"}))
    assert sent[-1] == [1, 0, 12500, 40000, 1, 0]
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "2", "LENGTH": "10", "SPEED": "25"}))
    assert sent[-1] == [1, 1, 10000, 25000, 2, 0]
    feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "3"}))
    assert sent[-1] == [2, 4, 0, 80000, 1, 0]
    feed.cmd_ACEK_LOAD(FakeGcmd({"LANE": "4"}))
    assert sent[-1] == [3, 5, 300000, 60000, 1, 0]
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1"}))
    assert sent[-1] == [0, 2, 0, 50000, 1, 0]
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "back", "SPEED": "20"}))
    assert sent[-1] == [0, 3, 0, 20000, 2, 0]


def test_acek_feed_free_sends_the_sixth_argument() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, options={"feed_speed": 40})
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data: Sequence[Any], minclock: int = 0, reqclock: int = 0) -> dict[str, int]:
            sent.append(list(data))
            mcu.subscriptions["ace2k_feed_event"](event_params(data[0], 0, 1, 1, 1, data[4]))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    feed.start_query = Query()
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "FREE": "1"}))
    assert sent[-1][5] == 1
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10"}))
    assert sent[-1][5] == 0
    with pytest.raises(Exception, match="FREE"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "FREE": "2"}))


def free_rollback_feed(free_rollback: bool) -> tuple[Ace2kFeed, list[list[Any]]]:
    """A feed on an image with (or without) ACE2K_FEED_FREE_ROLLBACK, its starts sent as data."""
    responses = dict(RESPONSES)
    constants = dict(DEFAULT_CONSTANTS)
    if free_rollback:
        constants["ACE2K_FEED_FREE_ROLLBACK"] = 1
    module, printer, mcu = make_feed(responses, options={"feed_speed": 40}, constants=constants)
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data: Sequence[Any], minclock: int = 0, reqclock: int = 0) -> dict[str, int]:
            sent.append(list(data))
            mcu.subscriptions["ace2k_feed_event"](event_params(data[0], 0, 2, 1, 1, data[4]))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    feed.start_query = Query()
    return feed, sent


def test_acek_rollback_free_sends_the_sixth_argument_on_an_image_that_reads_it() -> None:
    feed, sent = free_rollback_feed(True)
    assert feed.has_free_rollback is True
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "10", "FREE": "1"}))
    assert sent[-1][1] == 1 and sent[-1][5] == 1
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "10"}))
    assert sent[-1][5] == 0
    with pytest.raises(Exception, match="FREE"):
        feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "10", "FREE": "2"}))


def test_acek_rollback_free_on_an_image_that_ignores_it_is_refused_with_nothing_sent() -> None:
    feed, sent = free_rollback_feed(False)
    assert feed.has_free_rollback is False
    with pytest.raises(Exception, match="ignores FREE on a rollback; flash v0.12.0"):
        feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "10", "FREE": "1"}))
    assert sent == []
    feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "10"}))  # without FREE, as before
    assert sent[-1][5] == 0


def test_start_move_free_tip_sends_the_sixth_argument() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.start_move(0, "feed", 10.0, 40.0, free_tip=True)
    assert mcu.queries[-1][1][5] == 1
    feed.start_move(0, "rollback", 10.0, 40.0)
    assert mcu.queries[-1][1][5] == 0
    feed.start_move(0, "rollback", 10.0, 40.0, free_tip=True)  # read by the unit since 14
    assert mcu.queries[-1][1][5] == 1
    assert feed.API_VERSION == 29


def test_a_refusal_is_a_gcode_error_naming_the_reason() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 2}
    with pytest.raises(Exception, match="refused: in_error"):
        feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert nothing_tracked(feed_of(module))


def test_a_soft_assist_names_a_refusal_instead_of_raising() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 0, "reason": 1}
    # without SOFT the unit's refusal is the G-code error, as for every start
    with pytest.raises(Exception, match="lane 2 assist refused: busy"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2"}))
    # with SOFT=1 the same refusal is a line naming the lane and the reason, and no error; a print
    # hook never waits, so a busy is not sent again
    sent = len(mcu.sent)
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert len(mcu.sent) == sent + 1 and nothing_tracked(feed)
    assert gcmd.lines == ["ace2k: lane 2 not armed — lane 2 assist refused: busy"]
    # a host-side refusal (a speed out of bounds) is named the same way, nothing sent
    sent = len(mcu.sent)
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1", "SPEED": "9999"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert len(mcu.sent) == sent and len(gcmd.lines) == 1
    assert gcmd.lines[0].startswith("ace2k: lane 2 not armed — SPEED must be within")
    # an accepted start with SOFT=1 is the same as without
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert gcmd.lines == ["ace2k: lane 2 assist started"]
    # SOFT takes 0 or 1 only, and OFF=1 with SOFT=1 disarms as without it
    with pytest.raises(Exception, match="SOFT"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2", "SOFT": "2"}))
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1", "OFF": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    assert gcmd.lines == ["ace2k: lane 2 assist off"]


def test_an_error_event_is_a_gcode_error_and_a_missing_one_a_timeout() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 8, 1, 60000, 4000, seq=1))
    with pytest.raises(Exception, match="failed: tangled — motor 60.0 mm, filament 4.0 mm"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    # 100 mm at 30 mm/s: 90 mm at 30 and the 10 mm landing at the 9 mm/s floor — 1.5 × (90 / 30
    # + 10 / 9) + 5 = 11.17 s, printed whole
    with pytest.raises(Exception, match="no completion event within 11 s; a stop was sent"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert nothing_tracked(feed)
    # the host gave up on a move it still believes is running: the lane is stopped
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # a waited unload counts the tail as a second move: 50 mm and the 100 mm tail at 9 mm/s —
    # 1.5 × (50 / 9 + 100 / 9) + 5 = 30 s
    with pytest.raises(Exception, match="unload: no completion event within 30 s"):
        feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "1", "LENGTH": "50", "SPEED": "9"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # a rollback runs the same tail once the filament leaves the drive: 30 s too, not 13.33 s
    with pytest.raises(Exception, match="rollback: no completion event within 30 s"):
        feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "50", "SPEED": "9"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])


def test_move_wait_s_is_the_waited_moves_budget_with_its_landing() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    # 100 mm at 30 mm/s: (90 / 30 + 10 / 9) × 1.5 + 5, the landing at the floor speed
    assert feed.move_wait_s(0, 100, 30) == pytest.approx(11.17, abs=0.005)
    assert feed.move_wait_s(3, 100, 30) == feed._wait_timeout(100, 30)
    # a move shorter than the landing runs at the floor speed throughout
    assert feed.move_wait_s(0, 5, 70) == pytest.approx(5.83, abs=0.005)


def waited_budget_s(
    monkeypatch: pytest.MonkeyPatch, feed: Ace2kFeed, run: Callable[[], object]
) -> float:
    """The budget, in seconds, that the waited G-code run() gives its move: the deadline of its
    completion's wait less the clock, the move left with no final event."""
    deadlines = []
    real_wait = FakeCompletion.wait

    def wait(
        self: FakeCompletion, waketime: float | None = None, waketime_result: object = None
    ) -> object:
        deadlines.append(waketime)
        return real_wait(self, waketime, waketime_result)

    monkeypatch.setattr(FakeCompletion, "wait", wait)
    with pytest.raises(Exception, match="no completion event within"):
        run()
    monkeypatch.setattr(FakeCompletion, "wait", real_wait)
    return deadlines[-1] - feed.reactor.monotonic()


def test_move_wait_s_reverse_is_the_waited_rollbacks_budget_with_its_tail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    # 950 mm at 70 mm/s and the 100 mm tail, each with its 10 mm landing at the 9 mm/s floor:
    # 1.5 × (940 / 70 + 10 / 9 + 90 / 70 + 10 / 9) + 5 = 30.40 s
    rollback = waited_budget_s(
        monkeypatch,
        feed,
        lambda: feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "950", "SPEED": "70"})),
    )
    assert feed.move_wait_s(0, 950, 70, reverse=True) == pytest.approx(rollback)
    assert rollback == pytest.approx(30.40, abs=0.005)
    unload = waited_budget_s(
        monkeypatch,
        feed,
        lambda: feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "1", "LENGTH": "950", "SPEED": "70"})),
    )
    assert feed.move_wait_s(0, 950, 70, reverse=True) == pytest.approx(unload)
    # without reverse the tail is left out, as before: 1.5 × (940 / 70 + 10 / 9) + 5 = 26.81 s
    assert feed.move_wait_s(0, 950, 70) == pytest.approx(26.81, abs=0.005)
    assert feed.move_wait_s(0, 950, 70, reverse=False) == feed.move_wait_s(0, 950, 70)
    assert feed.move_wait_s(0, 950, 70, reverse=True) > feed.move_wait_s(0, 950, 70)


def test_move_wait_s_reverse_refuses_as_without_it() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    with pytest.raises(ValueError, match="no lane"):
        feed.move_wait_s(4, 100, 30, reverse=True)
    with pytest.raises(ValueError):
        feed.move_wait_s(0, 100, 0, reverse=True)
    with pytest.raises(TypeError):
        feed.move_wait_s(0, 100, 30, True)  # a keyword only
    feed.present = False
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.move_wait_s(0, 100, 30, reverse=True)


def test_move_wait_s_refuses_what_it_cannot_estimate() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    for lane in (4, -1, True):
        with pytest.raises(ValueError, match="no lane"):
            feed.move_wait_s(lane, 100, 30)
    nan, inf = float("nan"), float("inf")
    bad = ((-1, 30), (100, 0), (100, -5), (100, nan), (nan, 30), (inf, 30), (100, inf))
    for length, speed in bad:
        with pytest.raises(ValueError):
            feed.move_wait_s(0, length, speed)
    feed.present = False
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.move_wait_s(0, 100, 30)


def test_the_unload_wait_counts_the_tail_as_a_second_move() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.tail_mm == 100.0
    assert feed._wait_timeout(50, 9, tail_mm=100) == pytest.approx(30.0, abs=0.005)
    # 100 mm at 30 mm/s, the tail too: 2 × (90 / 30 + 10 / 9) × 1.5 + 5 = 17.33 s
    assert feed._wait_timeout(100, 30, tail_mm=100) == pytest.approx(17.33, abs=0.005)
    assert feed._wait_timeout(100, 30) == pytest.approx(11.17, abs=0.005)  # no tail: as before


def test_a_dictionary_without_the_tail_is_a_config_error() -> None:
    constants = {k: v for k, v in DEFAULT_CONSTANTS.items() if k != "ACE2K_FEED_UNLOAD_TAIL_UM"}
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    with pytest.raises(Exception, match="ACE2K_FEED_UNLOAD_TAIL_UM"):
        mcu.config_callback()


def test_the_waited_timeout_budgets_the_landing_at_the_floor_speed() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.landing_mm == DEFAULT_CONSTANTS["ACE2K_LANE_LANDING_UM"] / 1000 == 10.0
    # longer than the landing: 90 mm at 30 mm/s and the 10 mm landing at the 9 mm/s floor —
    # 1.5 × (90 / 30 + 10 / 9) + 5 = 1.5 × 4.111 + 5 = 11.17 s; whole at 30 mm/s it would have
    # been 1.5 × 100 / 30 + 5 = 10.0 s
    assert feed._wait_timeout(100, 30) == pytest.approx(11.17, abs=0.005)
    assert feed._wait_timeout(100, 30) > 10.0
    # no longer than the landing: the whole move at the floor speed, whatever was asked —
    # 5 mm: 1.5 × 5 / 9 + 5 = 5.83 s at 70 mm/s as at 30; the landing's own 10 mm: 1.5 × 10 / 9
    # + 5 = 6.67 s
    assert feed._wait_timeout(5, 70) == pytest.approx(5.83, abs=0.005)
    assert feed._wait_timeout(5, 70) == feed._wait_timeout(5, 30)
    assert feed._wait_timeout(feed.landing_mm, 70) == pytest.approx(6.67, abs=0.005)


def test_wait_zero_returns_at_once() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"})
    feed_of(module).cmd_ACEK_FEED(gcmd)
    assert gcmd.lines == ["ace2k: lane 1 feed started"]
    assert nothing_tracked(feed_of(module))


def test_a_second_start_on_a_busy_lane_is_sent_and_the_units_refusal_is_the_error() -> None:
    # the host keeps no in-flight flag: a start on a lane whose previous move has not reported
    # its end goes out, and the unit's own busy refusal is what stops it
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert not feed.any_busy()  # the 1 Hz report alone tells busy
    sent = len(mcu.sent)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(Exception, match="lane 1 feed refused: busy"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert len(mcu.sent) == sent + 1 and nothing_tracked(feed)
    # the refused start took a sequence (2) but entered no mode: its entry is gone, the first
    # start's stays, and the running move's done (seq 1) still carries the commanded length
    assert feed.motion[0].commanded_mm == {1: 100.0}
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 100000, 99000, seq=1))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    assert feed.motion[0].commanded_mm == {} and feed.motion[0].seq == 2


def test_each_starts_length_is_kept_until_its_own_event_and_the_map_is_pruned() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    # a second start accepted before the first move's done lands (the unit saw the first end):
    # the first's done still publishes the first's length, the second's its own
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "50", "WAIT": "0"}))
    assert feed.motion[0].commanded_mm == {1: 100.0, 2: 50.0}
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 100000, 99000, seq=1))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 50000, 49500, seq=2))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0
    assert feed.motion[0].commanded_mm == {}
    # a notice pops nothing; a unit-started terminal event (seq 0) pops nothing either
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "20", "WAIT": "0"}))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 14, 5, 5000, 8000, seq=3))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 5, 4, 400000, 399000, seq=0))
    assert feed.motion[0].commanded_mm == {3: 20.0}
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None
    # starts whose events never come are pruned to the newest COMMANDED_KEEP (8): after nine
    # more (seqs 4..12) the oldest, 3 and 4, are gone and 5..12 remain
    for length in range(9):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": str(length + 1), "WAIT": "0"}))
    assert list(feed.motion[0].commanded_mm) == list(range(5, 13))
    assert len(feed.motion[0].commanded_mm) == ace2k_feed.COMMANDED_KEEP == 8
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 1000, 1000, seq=3))
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None  # pruned: unknown, not wrong


def test_a_stale_event_does_not_complete_the_wait_and_the_matching_one_does() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    # a WAIT=0 feed (seq 1) whose done is late; then a waited feed (seq 2) — the unit accepts
    # it (the first move ended on its side) and the stale done lands during the round trip,
    # before the waited move's own
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    stale = event_params(0, 0, 1, 100000, 99000, seq=1)
    own = event_params(0, 0, 1, 50000, 49000, seq=2)

    class Query:
        def send(self, data: Sequence[Any], minclock: int = 0, reqclock: int = 0) -> dict[str, int]:
            mcu.subscriptions["ace2k_feed_event"](stale)
            # the stale done sits in last_event with its own start's length, the wait pending
            assert feed.lanes[0]["last_event"]["seq"] == 1
            assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
            assert feed.motion[0].tracked[0] == 2 and not feed.motion[0].tracked[1].test()
            mcu.subscriptions["ace2k_feed_event"](own)
            return {"lane": 0, "accepted": 1, "reason": 0}

    feed.start_query = Query()  # the start's query, built at connect
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "50"})
    feed.cmd_ACEK_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 50.0 mm, filament 49.0 mm"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0
    assert nothing_tracked(feed)
    # every event, stale or not, reached the other modules
    assert [ev["seq"] for name, (lane, ev) in printer.events] == [1, 2]


def test_a_unit_started_motion_completes_no_wait_and_carries_no_commanded_length() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    # seq 0 is the unit's own (an automatic load): it neither completes a pending wait nor
    # blocks it, and its last_move has no commanded length
    responses["ace2k_feed_start"] = accepted(
        mcu,
        event_params(0, 5, 4, 400000, 399000, seq=0),
        event_params(0, 0, 1, 100000, 99500, seq=1),
    )
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACEK_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 100.0 mm, filament 99.5 mm"
    loaded, done = [ev for name, (lane, ev) in printer.events]
    assert loaded["kind"] == "loaded" and loaded["seq"] == 0
    assert done["seq"] == 1 and feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 6, 3, 300000, -299000, seq=0))
    assert feed.lanes[0]["last_move"]["kind"] == "unloaded"
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None


def test_the_sequence_is_seeded_from_the_state_report_until_the_lanes_first_start() -> None:
    # on a klippy RESTART the unit may still run the previous session's move, or have just ended
    # it; the report's seq is the last start issued on the lane, kept through idle, and the
    # counter of a lane this session has not started on follows every report, so the first
    # start is reported + 1 — never the running move's, nor a repeat of the one that just ended
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data: Sequence[Any], minclock: int = 0, reqclock: int = 0) -> dict[str, int]:
            sent.append(list(data))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    feed.start_query = Query()  # the start's query, built at connect
    # no report yet this session and none coming: the start waits its bound, then errors with
    # nothing sent (test_a_start_before_the_first_state_report_waits_for_it)
    with pytest.raises(Exception, match="no feed state report within 7 s"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent == [] and not feed.motion[2].started
    # lane 3 (index 2) runs a move from the earlier session with seq 41: the next start is 42;
    # lane 2 (index 1) is idle and its last start was 5 — the report keeps it, so the counter
    # follows it too; lane 4 (index 3) never started since the unit powered up: 0
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0, 0, 1, 0], [0] * 4, [0, 0, 30000, 0], [0, 0, 40, 0], [0] * 4, (0, 5, 41, 0))
    )
    assert [m.seq for m in feed.motion] == [0, 5, 41, 0]
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][0] == 2 and sent[-1][4] == 42
    # 255 wraps to 1
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (255, 0, 0, 0))
    )
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][0] == 0 and sent[-1][4] == 1
    # once a lane has been started on, a later report does not reseed it: lanes 1 and 3 keep
    # their own counters (1 and 42 → 2 and 43), lane 2 — never started — still follows (99)
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (7, 99, 7, 0))
    )
    assert [m.seq for m in feed.motion] == [1, 99, 42, 0]
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][4] == 2
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][4] == 43
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "2", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][4] == 100


def test_a_start_before_the_first_state_report_waits_for_it_then_sends_reported_plus_one() -> None:
    # a macro that fires at startup, before the unit's first 1 Hz report: the start is parked
    # on the report, not refused — the frame lands while the G-code waits, and the start goes
    # out seeded from it (every lane idle, seq 0: the first start sends 1)
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu)
    waited_until = []

    def frame_lands(waketime: float) -> None:
        waited_until.append(waketime)
        mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)

    printer.reactor.while_waiting[feed.first_report] = [frame_lands]
    assert not feed.state_reported
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)
    assert feed.state_reported and [m.seq for m in feed.motion] == [1, 0, 0, 0]
    # exactly one start, none before the report landed
    assert mcu.queries == [("ace2k_feed_start", [0, 0, 100000, 30000, 1, 0])]
    assert gcmd.lines[-1] == "ace2k: lane 1 feed started"
    # the bound: two report periods of the configured rate plus the 5 s margin — 7 s at 1 Hz
    assert waited_until == [printer.reactor.now + 2 / 1.0 + ace2k_feed.WAIT_MARGIN_S]
    # a later start waits on nothing for the report: the sentinel on first_report never runs —
    # not for a WAIT=0 start, nor for a WAIT=1 one, whose wait is on its own event's completion
    printer.reactor.while_waiting[feed.first_report] = [
        lambda waketime: pytest.fail("waited for the report again")
    ]
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert mcu.queries[-1][1][4] == 2
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 100000, 99500, seq=3))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACEK_FEED(gcmd)
    assert mcu.queries[-1][1][4] == 3 and nothing_tracked(feed)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 100.0 mm, filament 99.5 mm"
    # none arriving within the bound: the error names the bound, computed from the configured
    # rate — 2 / 5 Hz + 5 = 5.4 s — and the rate; nothing is sent
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses, options={"feed_report_hz": 5})
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu)
    with pytest.raises(
        Exception,
        match=r"no feed state report within 5\.4 s \(the unit reports at feed_report_hz = 5 Hz;"
        r" check the link\)",
    ):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert mcu.queries == [] and not feed.motion[0].started and nothing_tracked(feed)


def test_a_report_seeds_through_the_reactor_only_while_a_lane_still_follows_it() -> None:
    # the seeding is a reactor callback per report; once the first report has been seen and
    # every lane has been started on, a report has nothing to seed and schedules none
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    scheduled = []  # the seeding callbacks run (each report also books its motion record)
    seed = feed._seed

    def counting(seqs: Sequence[int]) -> None:
        scheduled.append(seqs)
        seed(seqs)

    feed._seed = counting
    report = mcu.subscriptions["ace2k_feed_state"]
    report(IDLE_FRAME)
    assert len(scheduled) == 1 and feed.state_reported
    responses["ace2k_feed_start"] = accepted(mcu)
    for lane in ("1", "2", "3"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": lane, "LENGTH": "100", "WAIT": "0"}))
    report(IDLE_FRAME)  # lane 4 still follows the reports
    assert len(scheduled) == 2
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "4", "LENGTH": "100", "WAIT": "0"}))
    report(IDLE_FRAME)
    report(IDLE_FRAME)
    assert len(scheduled) == 2 and [m.seq for m in feed.motion] == [1, 1, 1, 1]
    assert feed.lanes[0]["mode"] == "idle"  # the frame still lands in the status


def test_the_report_period_is_snapped_to_the_100_ms_grid(caplog: pytest.LogCaptureFixture) -> None:
    # the report phases hold only for periods on the 100 ms grid (docs/protocol.md "Reports"):
    # 1, 2, 5 and 10 Hz are exact, anything else is snapped and the effective rate logged; the
    # floor, 0.05 Hz, is a 20 s period (2.4e9 ticks at 120 MHz, under the 2^32 the unit's
    # rest_ticks carries) — and the wait of a start before the first report grows with it
    with pytest.raises(Exception, match="feed_report_hz"):
        make_feed(dict(RESPONSES), options={"feed_report_hz": 0.02})
    for hz, period_ms in ((3, 300), (10, 100), (0.4, 2500), (1, 1000), (0.05, 20000)):
        caplog.clear()
        module, printer, mcu = make_feed(dict(RESPONSES), options={"feed_report_hz": hz})
        mcu.config_callback()
        feed = feed_of(module)
        init = [c for c, is_init in mcu.config_cmds if is_init]
        assert f"ace2k_feed_query rest_ticks={mcu.seconds_to_clock(period_ms / 1000.0)}" in init
        assert feed.report_period_ms == period_ms
        assert feed.report_hz == pytest.approx(1000.0 / period_ms)
        assert ("snapped" in caplog.text) == (hz == 3), (hz, caplog.text)
        if hz == 3:
            assert "feed_report_hz 3 snapped to 3.33 Hz (a 300 ms period on the 100 ms grid)" in (
                caplog.text
            )


def test_the_feed_keeps_one_motion_record_per_lane() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    feed = feed_of(module)
    assert len(feed.motion) == ace2k_feed.LANES
    assert len({id(m) for m in feed.motion}) == ace2k_feed.LANES


def test_stop_then_feed_on_the_same_lane_is_not_refused_by_the_host() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "500", "WAIT": "0"}))
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "1"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # the stopped event of seq 1 has not landed yet: the next feed goes out all the same, and
    # completes on its own seq 2 when the stopped event of seq 1 arrives first
    responses["ace2k_feed_start"] = accepted(
        mcu, event_params(0, 1, 1, 20000, 19000, seq=1), event_params(0, 0, 1, 50000, 49000, seq=2)
    )
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "50"})
    feed.cmd_ACEK_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 50.0 mm, filament 49.0 mm"
    assert nothing_tracked(feed)


def test_every_terminal_kind_completes_a_wait_on_its_sequence_but_a_notice_does_not() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    seq = 0
    # stopped, runout, stuck, motor_stalled, unload_incomplete, tail_out, blocked
    for kind in (1, 4, 7, 9, 13, 17, 18):
        seq += 1
        # a behind notice with the same seq first: it leaves the wait pending
        responses["ace2k_feed_start"] = accepted(
            mcu,
            event_params(2, 14, 5, 5000, 8000, seq=seq),
            event_params(2, kind, 1, 1000, 1000, seq=seq),
        )
        gcmd = FakeGcmd({"LANE": "3", "LENGTH": "100"})
        if kind in ace2k_feed.ERROR_KINDS:
            with pytest.raises(Exception, match=f"failed: {ace2k_feed.KIND_NAMES[kind]}"):
                feed.cmd_ACEK_FEED(gcmd)
        else:
            feed.cmd_ACEK_FEED(gcmd)
            assert gcmd.lines[-1].startswith(f"ace2k: lane 3 feed {ace2k_feed.KIND_NAMES[kind]}")
        assert nothing_tracked(feed)
        assert feed.lanes[2]["last_move"]["kind"] == ace2k_feed.KIND_NAMES[kind]
    # a notice alone: last_event, not last_move, and nothing to complete
    mcu.subscriptions["ace2k_feed_event"](event_params(2, 14, 5, 5000, 8000, seq=seq))
    assert feed.lanes[2]["last_event"]["kind"] == "behind"
    assert feed.lanes[2]["last_move"]["kind"] == "blocked"


def test_a_gcode_speed_outside_the_bounds_is_refused_before_sending() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    # the bounds the fake dictionary carries, as the module prints them
    low = DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MIN_UM_S"] / 1000
    high = DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MAX_UM_S"] / 1000
    with pytest.raises(Exception, match=re.escape(f"SPEED must be within {low:g}..{high:g}")):
        feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "SPEED": "5"}))
    assert "ace2k_feed_start" not in mcu.sent


def test_stop_clear_and_assist_off_send_their_commands() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.cmd_ACEK_STOP(FakeGcmd())
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [255])
    feed.cmd_ACEK_STOP(FakeGcmd({"LANE": "2"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    feed.cmd_ACEK_CLEAR(FakeGcmd({"LANE": "3"}))
    assert mcu.commands[-1] == ("ace2k_feed_clear lane=%c", [2])
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "4", "OFF": "1"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [3])


def speed_queried(mcu: FakeMcu) -> list[list[Any]]:
    return [data for name, data in mcu.queries if name == "ace2k_feed_speed_set"]


@pytest.mark.parametrize(
    "params, speed_um", [({"LANE": "3", "SPEED": "45"}, 45000), ({"LANE": "3"}, 30000)]
)
def test_speed_is_a_query_and_the_units_acceptance_prints_the_line(
    params: dict[str, str], speed_um: int
) -> None:
    # no state report needed: the unit itself says whether a move took the speed
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 2, "accepted": 1, "reason": 0})
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcmd = FakeGcmd(params)
    feed_of(module).cmd_ACEK_SPEED(gcmd)
    assert speed_queried(mcu) == [[2, speed_um]]
    assert gcmd.lines == [f"ace2k: lane 3 speed {speed_um / 1000:g} mm/s"]


@pytest.mark.parametrize(
    "reason, text",
    [(7, "no move running"), (2, "the lane is in error"), (5, "the speed is out of bounds")],
)
def test_a_refused_speed_is_a_gcode_error_naming_the_reason(reason: int, text: str) -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 1, "accepted": 0, "reason": reason})
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    gcmd = FakeGcmd({"LANE": "2", "SPEED": "45"})
    with pytest.raises(Exception, match=f"ace2k: lane 2 speed refused: {text}$"):
        feed_of(module).cmd_ACEK_SPEED(gcmd)
    assert speed_queried(mcu) == [[1, 45000]] and gcmd.lines == []


def test_a_speed_out_of_the_hosts_bounds_is_refused_before_the_query() -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 0, "accepted": 1, "reason": 0})
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    with pytest.raises(Exception, match="SPEED must be within"):
        feed_of(module).cmd_ACEK_SPEED(FakeGcmd({"LANE": "1", "SPEED": "500"}))
    assert speed_queried(mcu) == []


def test_the_refusal_names_mirror_the_firmware_header() -> None:
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    body = re.search(r"enum ace2k_feed_refusal \{(.*?)\};", header, re.S).group(1)
    values = {
        int(v): n.lower() for n, v in re.findall(r"ACE2K_FEED_(?:REFUSED_)?(\w+) = (\d+)", body)
    }
    values[0] = "ok"  # ACE2K_FEED_ACCEPTED
    assert values == ace2k_feed.REFUSAL_NAMES


def test_any_busy_takes_an_unknown_mode_code_as_busy_and_idle_error_or_no_report_as_not() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert not feed.any_busy()  # no report yet: every lane None
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0, 7, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4)
    )
    assert not feed.any_busy()  # idle and error
    frame = state_frame([42, 0, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4)
    mcu.subscriptions["ace2k_feed_state"](frame)
    assert feed.any_busy() and feed.lanes[0]["mode"] == 42


NO_ANSWER = "Unable to obtain 'ace2k_feed_speed_set_response' response"


def test_a_speed_query_with_no_answer_is_a_gcode_error_naming_the_link() -> None:
    # the transport's own failure (the printer's command_error) never reaches Klipper as is
    responses = dict(RESPONSES, ace2k_feed_speed_set=FakeCommandError(NO_ANSWER))
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    gcmd = FakeGcmd({"LANE": "1", "SPEED": "45"})
    with pytest.raises(Exception) as caught:
        feed_of(module).cmd_ACEK_SPEED(gcmd)
    assert not isinstance(caught.value, FakeCommandError)
    assert str(caught.value).startswith(
        "ace2k: lane 1 speed: no answer from the unit; check the link"
    )
    assert gcmd.lines == []


# --- set_speed: the speed of a running move, from a host module (API_VERSION 13) --------------


def test_set_speed_sends_the_query_and_returns_on_acceptance() -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 2, "accepted": 1, "reason": 0})
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    assert feed_of(module).set_speed(2, 45.0) is None
    assert speed_queried(mcu) == [[2, 45000]]


@pytest.mark.parametrize("reason, name", [(7, "no_move"), (2, "in_error"), (5, "bounds")])
def test_a_refused_set_speed_raises_feed_refused_with_the_reason_name(
    reason: int, name: str
) -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 1, "accepted": 0, "reason": reason})
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    with pytest.raises(ace2k_feed.FeedRefused) as caught:
        feed_of(module).set_speed(1, 45.0)
    assert (caught.value.lane, caught.value.mode, caught.value.reason) == (1, "speed", name)
    assert str(caught.value) == f"ace2k: lane 2 speed refused: {name}"


@pytest.mark.parametrize(
    "lane, speed",
    [(4, 45.0), (-1, 45.0), (1.0, 45.0), (True, 45.0), (0, 5.0), (0, 500.0), (0, True)],
)
def test_set_speed_out_of_bounds_is_a_value_error_before_the_query(
    lane: float, speed: float
) -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set={"lane": 0, "accepted": 1, "reason": 0})
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    with pytest.raises(ValueError):
        feed_of(module).set_speed(lane, speed)
    assert speed_queried(mcu) == []


def test_set_speed_unanswered_or_without_the_feed_raises() -> None:
    responses = dict(RESPONSES, ace2k_feed_speed_set=FakeCommandError(NO_ANSWER))
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    with pytest.raises(ace2k.NoAnswerError, match="lane 1 speed: no answer from the unit"):
        feed_of(module).set_speed(0, 45.0)
    module, printer, mcu = make_feed(dict(RESPONSES), known=set(FEED_KNOWN) - {"ace2k_feed_start"})
    mcu.config_callback()
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed_of(module).set_speed(0, 45.0)


def stops_sent(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt == "ace2k_feed_stop lane=%c"]


def test_a_start_with_no_answer_stops_the_lane_and_is_an_error_for_gcode_and_host_interface() -> (
    None
):
    # the unit may have started the move: the lane is stopped rather than left running untracked
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = FakeCommandError("Unable to obtain 'ace2k_feed_start_response'")
    with pytest.raises(Exception) as caught:
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert not isinstance(caught.value, FakeCommandError)
    assert str(caught.value).startswith(
        "ace2k: lane 1 feed: no answer from the unit; check the link"
    )
    assert str(caught.value).endswith("; lane 1 stopped in case it started")
    assert stops_sent(mcu) == [[0]]
    assert nothing_tracked(feed)  # the waited start registered nothing that stays
    with pytest.raises(RuntimeError, match="lane 2 feed: no answer .*lane 2 stopped in case"):
        feed.start_move(1, "feed", 100.0, 30.0)
    assert stops_sent(mcu) == [[0], [1]]


def test_a_start_answer_for_another_lane_is_an_error_and_the_lane_is_stopped() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 2, "accepted": 1, "reason": 0}
    with pytest.raises(
        Exception,
        match="ace2k: lane 1 feed: the unit answered for lane 3; lane 1 stopped in case it started",
    ):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert stops_sent(mcu) == [[0]] and nothing_tracked(feed)
    with pytest.raises(RuntimeError, match="lane 2 feed: the unit answered for lane 3"):
        feed.start_move(1, "feed", 100.0, 30.0)


def test_overlapping_starts_on_two_lanes_queue_on_one_mutex_and_each_take_their_own_answer() -> (
    None
):
    # a timer's start_move (the U1 adapter's) runs while a G-code's start waits for its answer:
    # the second query queues on the unit's query mutex, and each gets its own lane's answer
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    nested = []

    def respond() -> dict[str, int]:
        lane = mcu.queries[-1][1][0]
        if lane == 0 and not nested:
            nested.append(feed.start_move(1, "feed", 50.0, 30.0))
        return {"lane": lane, "accepted": 1, "reason": 0}

    responses["ace2k_feed_start"] = respond
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"})
    feed.cmd_ACEK_FEED(gcmd)
    assert gcmd.lines == ["ace2k: lane 1 feed started"]
    assert nested[0].lane == 1 and nested[0].seq == 1
    assert ace2k.query_mutex(printer, mcu).contended == 1


@pytest.mark.parametrize("cmd", ["start", "speed"])
def test_an_unknown_refusal_reason_is_named_by_its_number(cmd: str) -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    answer = {"lane": 0, "accepted": 0, "reason": 99}
    if cmd == "start":
        responses["ace2k_feed_start"] = answer
        with pytest.raises(Exception, match=r"^ace2k: lane 1 feed refused: reason 99$"):
            feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    else:
        responses["ace2k_feed_speed_set"] = answer
        with pytest.raises(Exception, match=r"^ace2k: lane 1 speed refused: reason 99$"):
            feed.cmd_ACEK_SPEED(FakeGcmd({"LANE": "1", "SPEED": "45"}))


def test_a_speed_answer_for_another_lane_is_an_error_not_the_answer() -> None:
    # a stale answer — another lane's — is never taken for this one, accepted or not
    for answer in (
        {"lane": 2, "accepted": 1, "reason": 0},
        {"lane": 2, "accepted": 0, "reason": 7},
    ):
        responses = dict(RESPONSES, ace2k_feed_speed_set=answer)
        module, printer, mcu = make_feed(responses)
        mcu.config_callback()
        gcmd = FakeGcmd({"LANE": "1", "SPEED": "45"})
        with pytest.raises(Exception, match="ace2k: lane 1 speed: the unit answered for lane 3"):
            feed_of(module).cmd_ACEK_SPEED(gcmd)
        assert gcmd.lines == []


def test_the_start_and_speed_queries_are_built_once_at_connect() -> None:
    responses = dict(RESPONSES, ace2k_feed_start={"lane": 0, "accepted": 1, "reason": 0})
    module, printer, mcu = make_feed(responses)
    looked_up = []
    lookup = mcu.lookup_query_command

    def counting(msgformat: str, respformat: str, *args: object, **kwargs: object) -> FakeQuery:
        looked_up.append(msgformat.split(maxsplit=1)[0])
        return lookup(msgformat, respformat, *args, **kwargs)

    mcu.lookup_query_command = counting
    mcu.config_callback()
    assert looked_up.count("ace2k_feed_speed_set") == 1
    assert looked_up.count("ace2k_feed_start") == 1
    for _ in range(2):
        feed_of(module).cmd_ACEK_SPEED(FakeGcmd({"LANE": "1", "SPEED": "45"}))
        feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "10", "WAIT": "0"}))
    assert looked_up.count("ace2k_feed_speed_set") == 1
    assert looked_up.count("ace2k_feed_start") == 1
    assert speed_queried(mcu) == [[0, 45000], [0, 45000]]


def test_calibration_feeds_then_stages_the_scale() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "200"})
    feed.cmd_ACEK_CALIBRATE_ENCODER(gcmd)
    # one unit: the command as it always was, no UNIT=
    assert gcmd.lines[-1].endswith("then ACEK_CALIBRATE_ENCODER LANE=1 MEASURED=<mm>")
    assert feed.calibration[0] == (200.0, 1.2342)
    gcmd = FakeGcmd({"LANE": "1", "MEASURED": "197"})
    feed.cmd_ACEK_CALIBRATE_ENCODER(gcmd)
    # 200.0 mm / 1.2342 = 162.05 counts; 197 / 162.05 = 1.2157
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.2157")]
    with pytest.raises(Exception, match="run ACEK_CALIBRATE_ENCODER LANE=2 LENGTH=<mm> first"):
        feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "2", "MEASURED": "100"}))


def test_a_final_event_of_several_moves_is_reported_as_the_whole_mode() -> None:
    """The unit's final event carries the whole mode's travel, every move of it. An unload of 50
    mm whose deadline passed with the plunger at rest went on with a tail move of 100 mm, and that
    move ran its length: 140 mm of filament and 150 mm of motor in all. The waited G-code's error,
    the lane's last move and ACEK_STATUS show those figures as they came."""
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 13, 3, 150000, -140000, seq=1))
    with pytest.raises(
        Exception, match="unload failed: unload_incomplete — motor 150.0 mm, filament -140.0 mm"
    ):
        feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "1", "LENGTH": "50"}))
    move = feed.lanes[0]["last_move"]
    assert (move["motor_mm"], move["filament_mm"], move["commanded_mm"]) == (150.0, -140.0, 50.0)
    assert (
        "last=unload_incomplete (unloading, motor 150.0 mm, filament -140.0 mm)"
        in (feed.status_lines()[0])
    )


def test_a_calibration_feed_that_ran_in_several_moves_samples_the_whole_feed() -> None:
    """A calibration feed whose tip caught and went on after a touch, then lost the link: its
    final event carries the whole feed (205 mm), not the move after the touch."""
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 2, 1, 212000, 205000, seq=1))
    feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    assert feed.calibration[0] == (205.0, 1.2342)


def test_a_blocked_calibration_feed_is_an_error_and_keeps_no_sample() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    assert feed.calibration[0] == (200.0, 1.2342)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 18, 1, 42000, 9000, seq=2))
    with pytest.raises(
        Exception, match="lane 1 calibration feed blocked — motor 42.0 mm, filament 9.0 mm"
    ):
        feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    assert 0 not in feed.calibration  # neither this reading nor the earlier one
    with pytest.raises(Exception, match="LENGTH=<mm> first"):
        feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "197"}))
    assert printer.objects["configfile"].calls == []


def test_a_measurement_outside_the_scale_window_stages_nothing() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    # 120 / 162.05 counts = 0.7405 mm/count: the unit would shut down on it at the restart
    with pytest.raises(Exception, match=r"0.7405 mm/count is outside the window .*0.9874..1.4810"):
        feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "120"}))
    assert printer.objects["configfile"].calls == []
    feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "230"}))  # 1.4193: inside
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.4193")]


def test_load_set_sends_and_stages_what_was_given() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.cmd_ACEK_LOAD_SET(FakeGcmd({"PARK": "350", "AUTO": "0"}))
    assert mcu.commands[-1] == (
        "ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c",
        [350000, 60000, 0],
    )
    assert printer.objects["configfile"].calls == [
        ("ace2k", "load_park_mm", "350"),
        ("ace2k", "auto_load", "False"),
    ]
    with pytest.raises(Exception, match="SPEED must be within"):
        feed.cmd_ACEK_LOAD_SET(FakeGcmd({"SPEED": "500"}))


def test_load_set_with_a_rejected_speed_changes_nothing() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    # PARK and AUTO are valid, SPEED is not: the whole command is refused before anything is
    # assigned, staged for SAVE_CONFIG or sent
    with pytest.raises(Exception, match="SPEED must be within"):
        feed.cmd_ACEK_LOAD_SET(FakeGcmd({"PARK": "250", "SPEED": "200", "AUTO": "0"}))
    assert feed.load_park_mm == 300.0
    assert feed.load_speed_mm_s == 60.0
    assert feed.auto_load is True
    assert printer.objects["configfile"].calls == []
    assert mcu.commands == commands


def test_load_set_alone_prints_the_values_in_effect_and_stages_and_sends_nothing() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    gcmd = FakeGcmd()
    feed.cmd_ACEK_LOAD_SET(gcmd)
    assert gcmd.lines == ["ace2k: load park 300 mm, speed 60 mm/s, auto True"]
    assert printer.objects["configfile"].calls == []
    assert mcu.commands == commands


def test_counters_reset_is_refused_while_a_lane_moves() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0])
    )
    with pytest.raises(Exception, match="a lane is moving or busy"):
        module.cmd_ACEK_COUNTERS_RESET(FakeGcmd())
    # a move accepted since the last 1 Hz report is the unit's to refuse (its veto is the
    # backstop for the report's blind window): the host sends the reset
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0])
    )
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    feed_of(module).cmd_ACEK_FEED(FakeGcmd({"LANE": "2", "LENGTH": "100", "WAIT": "0"}))
    assert not feed_of(module).any_busy()
    module.cmd_ACEK_COUNTERS_RESET(FakeGcmd())
    assert mcu.queries[-1] == ("ace2k_lane_counters_reset", [255])


def test_every_feed_gcode_name_is_letters_and_underscores_only() -> None:
    for name in ace2k_feed.GCODES:
        assert name.replace("_", "").isalpha() and name.isupper()


def test_a_load_waits_for_the_search_when_the_firmware_has_one() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), options={"rfid_search_mm": 600})
    feed = feed_of(module)
    mcu.config_callback()
    with_search = feed._load_budget_mm(300.0)
    assert with_search == pytest.approx(300.0 + 2 * 300.0 + feed.SESSION_ALLOWANCE_MM)
    absent = {"ace2k_rfid_search_set": "Unknown command: ace2k_rfid_search_set"}
    module, printer, mcu = make_feed(dict(RESPONSES), lookup_errors=absent)
    mcu.config_callback()
    assert feed_of(module)._load_budget_mm(300.0) == 300.0


def test_an_unset_search_takes_the_firmwares_default_and_750_without_it() -> None:
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_SEARCH_DEFAULT_UM=800000)
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    feed = feed_of(module)
    mcu.config_callback()
    allowance = feed.SESSION_ALLOWANCE_MM
    assert feed._load_budget_mm(300.0) == pytest.approx(300.0 + 2 * 500.0 + allowance)
    module, printer, mcu = make_feed(dict(RESPONSES))  # an image without the constant
    feed = feed_of(module)
    mcu.config_callback()
    allowance = feed.SESSION_ALLOWANCE_MM
    assert feed._load_budget_mm(300.0) == pytest.approx(300.0 + 2 * 450.0 + allowance)


# --- the tip snag ------------------------------------------------


def test_the_snag_values_go_out_as_an_init_command_at_every_connect() -> None:
    # an init command, keys or not: a config command reaches only an unconfigured MCU, so after
    # ACEK_SNAG_SET and a RESTART the unit would keep the live values the host no longer shows
    options = {"snag_fwd_mm": 25, "snag_duty_pct": 12}
    module, printer, mcu = make_feed(dict(RESPONSES), options=options)
    mcu.config_callback()
    snag = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_snag_set")]
    assert snag == [
        (
            "ace2k_feed_snag_set fwd_um=25000 lag_um=5000 duty_pct=12 back_um=5000 rest_ms=500"
            " free_um=50000",
            True,
        )
    ]
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    snag = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_snag_set")]
    assert snag == [
        (
            "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
            " free_um=50000",
            True,
        )
    ]
    assert feed_of(module).snag["snag_fwd_mm"] == 20.0  # the firmware's default, in effect


def test_feed_snag_keys_and_units_are_the_documented_ones() -> None:
    # feed.snag is a documented host-interface attribute (API_VERSION 16): empty before the
    # connect, then the six keys in the documented units
    module, printer, mcu = make_feed(dict(RESPONSES))
    assert feed_of(module).snag == {}
    mcu.config_callback()
    assert feed_of(module).snag == {
        "snag_fwd_mm": 20.0,
        "snag_lag_mm": 5.0,
        "snag_duty_pct": 0,
        "snag_back_mm": 5.0,
        "snag_rest_ms": 500,
        "snag_free_mm": 50.0,
    }
    assert ace2k_feed.API_VERSION == 29


def test_a_snag_lag_at_or_above_the_tolerance_is_a_config_error() -> None:
    # the pair as the unit sees it: an absent key is the firmware's default (fwd 20 / lag 5 mm)
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_lag_mm": 20})
    with pytest.raises(Exception, match=r"snag_lag_mm \(20\).*snag_fwd_mm \(20\)"):
        mcu.config_callback()
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_fwd_mm": 5})
    with pytest.raises(Exception, match=r"snag_lag_mm \(5\).*snag_fwd_mm \(5\)"):
        mcu.config_callback()
    assert not any(c.startswith("ace2k_feed_snag_set") for c, _ in mcu.config_cmds)


def test_a_snag_key_out_of_bounds_is_a_config_error() -> None:
    # checked at connect, against the bounds of the image or this module's own table
    for constants in (DEFAULT_CONSTANTS, WITH_SNAG_BOUNDS):
        for key, value in (
            ("snag_fwd_mm", 60),
            ("snag_lag_mm", 0.5),
            ("snag_lag_mm", 0.999),
            ("snag_duty_pct", 101),
            ("snag_back_mm", 1),
            ("snag_rest_ms", 2001),
            ("snag_free_mm", 10),
        ):
            module, printer, mcu = make_feed(
                dict(RESPONSES), options={key: value}, constants=constants
            )
            with pytest.raises(Exception, match=key):
                mcu.config_callback()
            assert not any(c.startswith("ace2k_feed_snag_set") for c, _ in mcu.config_cmds)


# The snag's defaults and bounds as an image since v0.13.0 publishes them (feed_cmds.c).
SNAG_BOUND_CONSTANTS = {
    "ACE2K_FEED_SNAG_FWD_UM_MIN": 5000,
    "ACE2K_FEED_SNAG_FWD_UM_MAX": 50000,
    "ACE2K_FEED_SNAG_LAG_UM_MIN": 1500,
    "ACE2K_FEED_SNAG_LAG_UM_MAX": 20000,
    "ACE2K_FEED_SNAG_DUTY_PCT_MAX": 100,
    "ACE2K_FEED_SNAG_BACK_UM_MIN": 2000,
    "ACE2K_FEED_SNAG_BACK_UM_MAX": 20000,
    "ACE2K_FEED_SNAG_REST_MS_MIN": 100,
    "ACE2K_FEED_SNAG_REST_MS_MAX": 2000,
    "ACE2K_FEED_SNAG_FREE_UM_MIN": 20000,
    "ACE2K_FEED_SNAG_FREE_UM_MAX": 200000,
}
WITH_SNAG_BOUNDS = dict(DEFAULT_CONSTANTS, **SNAG_BOUND_CONSTANTS)
SNAG_LINE = (
    "fwd 5..50 mm, lag 1.5..20 mm, duty 0..100 %, back 2..20 mm, rest 100..2000 ms, free 20..200 mm"
)


def old_lag_warning(value: float) -> str:
    """The warning for a snag_lag_mm from 1 up to 1.5, as the unit gets it (to the µm)."""
    shown = ace2k_feed.setting_text(round(value * 1000) / 1000)
    return (
        f"ace2k: snag_lag_mm {shown} is under the least lag of 1.5 mm, so 1.5 mm is used."
        " The next version refuses it."
    )


def test_the_published_snag_bounds_are_the_headers_under_the_follows_naming() -> None:
    src = pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed"
    header = (src / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_SNAG_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    }
    published = dict(
        re.findall(
            r'^DECL_CONSTANT\("(ACE2K_FEED_SNAG_\w+)", (ACE2K_FEED_SNAG_\w+)\);',
            (src / "feed_cmds.c").read_text(),
            flags=re.M,
        )
    )
    bounds = {name: define for name, define in published.items() if name.endswith(("_MIN", "_MAX"))}
    # the fake carries the firmware's own values, under the names the host reads
    assert {name: defines[define] for name, define in bounds.items()} == SNAG_BOUND_CONSTANTS
    for name, define in bounds.items():
        stem, unit, end = name.rsplit("_", 2)  # ACE2K_FEED_SNAG_LAG, UM, MIN
        assert define == f"{stem}_{end}_{unit}", name  # the C name keeps its own order


def test_the_host_snag_bounds_are_the_firmwares() -> None:
    # the host's own table is the fallback for an image without the published bounds; it is
    # the bounds of this release's feed.h
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_SNAG_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    }
    checked = set()
    for key, _arg, scale, low, high in ace2k_feed.SNAG_SETTINGS:
        name = ace2k_feed.SNAG_DEFAULT_CONSTANTS[key]
        assert name in defines, name  # the default the host reads from the dictionary
        assert DEFAULT_CONSTANTS[name] == defines[name], name  # the fake carries the firmware's
        unit = name.rsplit("_", 1)[1]  # UM, PCT, MS
        stem = name[: -len(unit) - 1]
        low_name, high_name = f"{stem}_MIN_{unit}", f"{stem}_MAX_{unit}"
        if low_name in defines:
            assert defines[low_name] == round(low * scale), key
            checked.add(low_name)
        else:
            assert low == 0, f"{key}: no {low_name} in feed.h for a floor of {low}"
        assert high_name in defines, f"{key}: no {high_name} in feed.h"
        assert defines[high_name] == round(high * scale), key
        checked.add(high_name)
    bounds = {n for n in defines if "_MIN_" in n or "_MAX_" in n}
    assert bounds == checked  # every bound of the header is one the host mirrors


def test_the_snag_bounds_come_from_the_unit_when_it_publishes_them(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # a dictionary whose bounds differ from this module's table: the unit's are the ones used
    constants = dict(WITH_SNAG_BOUNDS, ACE2K_FEED_SNAG_FWD_UM_MAX=60000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"snag_fwd_mm": 55}, constants=constants
    )
    feed = feed_of(module)
    with caplog.at_level("INFO", logger="root"):
        mcu.config_callback()
    assert feed.snag["snag_fwd_mm"] == 55.0
    line = SNAG_LINE.replace("fwd 5..50", "fwd 5..60")
    assert [r.getMessage() for r in caplog.records if "snag bounds" in r.getMessage()] == [
        f"ace2k: snag bounds from the unit: {line}"
    ]
    assert any(c.startswith("ace2k_feed_snag_set fwd_um=55000 ") for c, _ in mcu.config_cmds)
    feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "58"}))
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [58000, 5000, 0, 5000, 500, 50000])
    with pytest.raises(Exception, match=r"FWD \(61\) must be within 5..60"):
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "61"}))
    # the image's bound refuses a key this module's table would take
    constants = dict(WITH_SNAG_BOUNDS, ACE2K_FEED_SNAG_BACK_UM_MIN=3000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"snag_back_mm": 2.5}, constants=constants
    )
    with pytest.raises(Exception, match=r"snag_back_mm \(2.5\) must be within 3..20"):
        mcu.config_callback()


def test_without_the_published_bounds_the_host_uses_its_own_table(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # an image before v0.13.0: the defaults from the dictionary, the bounds from this module;
    # a dictionary that has only some of the bounds is the same, never a mix
    partial = dict(
        DEFAULT_CONSTANTS, ACE2K_FEED_SNAG_FWD_UM_MIN=5000, ACE2K_FEED_SNAG_FWD_UM_MAX=60000
    )
    for constants in (DEFAULT_CONSTANTS, partial):
        module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
        feed = feed_of(module)
        caplog.clear()
        with caplog.at_level("INFO", logger="root"):
            mcu.config_callback()
        assert [r.getMessage() for r in caplog.records if "snag bounds" in r.getMessage()] == [
            "ace2k: snag bounds from this module, as the unit's firmware does not publish them:"
            f" {SNAG_LINE}"
        ]
        with pytest.raises(Exception, match=r"FWD \(55\) must be within 5..50"):
            feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "55"}))
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_fwd_mm": 55})
    with pytest.raises(Exception, match=r"snag_fwd_mm \(55\) must be within 5..50"):
        mcu.config_callback()


@pytest.mark.parametrize("constants", [DEFAULT_CONSTANTS, WITH_SNAG_BOUNDS])
@pytest.mark.parametrize("value", [1.0, 1.2, 1.4994])
def test_a_snag_lag_from_1_to_under_1_5_is_raised_to_1_5_with_one_warning(
    caplog: pytest.LogCaptureFixture, constants: dict[str, float], value: float
) -> None:
    # 1.0 up to 1.5 was a valid lag before v0.13.0; 1.5 is valid on an older image too (1..20)
    with caplog.at_level("INFO", logger="root"):
        module, printer, mcu = make_feed(
            dict(RESPONSES), options={"snag_lag_mm": value}, constants=constants
        )
        mcu.config_callback()
    feed = feed_of(module)
    assert feed.snag["snag_lag_mm"] == 1.5
    assert any(
        c.startswith("ace2k_feed_snag_set fwd_um=20000 lag_um=1500 ") for c, _ in mcu.config_cmds
    )
    warnings = [r for r in caplog.records if r.levelname == "WARNING"]
    assert [r.getMessage() for r in warnings] == [old_lag_warning(value)]
    assert printer.objects["configfile"].calls == []  # printer.cfg is left as it is


def test_a_snag_lag_of_1_5_or_more_or_under_1_is_not_raised(
    caplog: pytest.LogCaptureFixture,
) -> None:
    for value in (1.5, 1.4996, 5):
        with caplog.at_level("WARNING", logger="root"):
            module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_lag_mm": value})
            mcu.config_callback()
        assert feed_of(module).snag["snag_lag_mm"] == round(value * 1000) / 1000
    assert not [r for r in caplog.records if "snag_lag_mm" in r.getMessage()]
    # under 1 mm was refused before v0.13.0 too, and still is
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_lag_mm": 0.999})
    with pytest.raises(Exception, match=r"snag_lag_mm \(0.999\) must be within 1.5..20"):
        mcu.config_callback()


@pytest.mark.parametrize("constants", [DEFAULT_CONSTANTS, WITH_SNAG_BOUNDS])
@pytest.mark.parametrize("value", [1.0, 1.2, 1.4994])
def test_snag_set_raises_a_lag_from_1_to_under_1_5_and_stages_1_5(
    caplog: pytest.LogCaptureFixture, constants: dict[str, float], value: float
) -> None:
    # the same raise as printer.cfg; 1.5 is staged, so SAVE_CONFIG writes no value that the next
    # version refuses
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    mcu.config_callback()
    feed = feed_of(module)
    gcmd = FakeGcmd({"LAG": str(value)})
    with caplog.at_level("WARNING", logger="root"):
        feed.cmd_ACEK_SNAG_SET(gcmd)
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [20000, 1500, 0, 5000, 500, 50000])
    assert feed.snag["snag_lag_mm"] == 1.5
    assert printer.objects["configfile"].calls == [("ace2k", "snag_lag_mm", "1.5")]
    warning = old_lag_warning(value)
    assert [r.getMessage() for r in caplog.records if r.levelname == "WARNING"] == [warning]
    assert gcmd.lines == [
        warning,
        "ace2k: snag fwd 20 mm, lag 1.5 mm, duty 0 %, back 5 mm, rest 500 ms, free 50 mm;"
        " staged for SAVE_CONFIG",
    ]


def test_snag_set_refuses_a_lag_under_1_and_warns_of_nothing_it_refuses(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    with caplog.at_level("WARNING", logger="root"):
        with pytest.raises(Exception, match=r"LAG \(0.999\) must be within 1.5..20"):
            feed.cmd_ACEK_SNAG_SET(FakeGcmd({"LAG": "0.999"}))
        # a raised lag that another argument makes refused: no warning, nothing changes
        with pytest.raises(Exception, match=r"DUTY \(101\) must be within 0..100"):
            feed.cmd_ACEK_SNAG_SET(FakeGcmd({"LAG": "1.2", "DUTY": "101"}))
    assert mcu.commands == commands and printer.objects["configfile"].calls == []
    assert not [r for r in caplog.records if r.levelname == "WARNING"]


def test_snag_set_sends_all_six_and_stages_what_was_given() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    gcmd = FakeGcmd({"FWD": "30", "DUTY": "15"})
    feed.cmd_ACEK_SNAG_SET(gcmd)
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [30000, 5000, 15, 5000, 500, 50000])
    assert printer.objects["configfile"].calls == [
        ("ace2k", "snag_fwd_mm", "30"),
        ("ace2k", "snag_duty_pct", "15"),
    ]
    assert gcmd.lines == [
        "ace2k: snag fwd 30 mm, lag 5 mm, duty 15 %, back 5 mm, rest 500 ms, free 50 mm;"
        " staged for SAVE_CONFIG"
    ]
    # the values in effect carry on: a later set changes only what it names
    feed.cmd_ACEK_SNAG_SET(FakeGcmd({"LAG": "12.5"}))
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [30000, 12500, 15, 5000, 500, 50000])


def test_snag_set_alone_prints_and_sends_nothing_and_a_bad_lag_changes_nothing() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    gcmd = FakeGcmd()
    feed.cmd_ACEK_SNAG_SET(gcmd)
    assert mcu.commands == commands
    assert gcmd.lines == [
        "ace2k: snag fwd 20 mm, lag 5 mm, duty 0 %, back 5 mm, rest 500 ms, free 50 mm"
    ]
    with pytest.raises(Exception, match="LAG"):  # above its bound (20 mm)
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"LAG": "25"}))
    with pytest.raises(Exception, match=r"LAG \(20\) must be under FWD \(20\)"):
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"LAG": "20"}))  # in bounds, not under the forward
    with pytest.raises(Exception, match=r"LAG \(8\) must be under FWD \(6\)"):
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "6", "LAG": "8"}))
    with pytest.raises(Exception, match="FWD"):  # out of bounds, with a good one beside it
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"DUTY": "10", "FWD": "60"}))
    assert mcu.commands == commands
    assert printer.objects["configfile"].calls == []
    assert feed.snag["snag_lag_mm"] == 5.0 and feed.snag["snag_duty_pct"] == 0


def test_snag_set_on_a_firmware_without_the_feed_is_a_gcode_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed_of(module).cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "30"}))
    assert mcu.commands == []


def test_a_snag_notice_is_printed_and_completes_no_wait(caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    pending = printer.reactor.completion()
    feed.motion[0].tracked = (3, pending)  # a waiting G-code on the move the notice belongs to
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_feed_event"](event_params(0, 15, 1, 41214, 41440, 3))
    ev = dict(
        kind="snag",
        mode="feeding",
        motor_mm=41.214,
        filament_mm=41.44,
        seq=3,
        stop_by=None,
        tag_search=False,
    )
    assert feed.lanes[0]["last_event"] == ev
    assert "lane 1 snag (feeding)" in caplog.text
    assert printer.events == [("ace2k:feed_event", (0, dict(ev, unit=None)))]
    assert printer.objects["gcode"].output == [
        "ace2k: lane 1 snag (feeding) — the tip caught at motor 41.2 mm, filament 41.4 mm; went on"
    ]
    assert feed.lanes[0]["last_move"] is None  # a notice is not a move's end
    assert not pending.test() and feed.motion[0].tracked == (3, pending)


def test_a_snag_value_is_judged_at_the_units_resolution() -> None:
    # the unit checks whole µm: 5.0004 and 5.0001 are both 5000 µm, a lag equal to the forward
    # travel that would shut the MCU down — refused by the host, nothing sent or staged
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    with pytest.raises(Exception, match=r"LAG \(5\) must be under FWD \(5\)"):
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FWD": "5.0004", "LAG": "5.0001"}))
    assert mcu.commands == commands and printer.objects["configfile"].calls == []
    # the same pair in printer.cfg: a config error at connect
    options = {"snag_fwd_mm": 5.0004, "snag_lag_mm": 5.0001}
    module, printer, mcu = make_feed(dict(RESPONSES), options=options)
    with pytest.raises(Exception, match=r"snag_lag_mm \(5\).*snag_fwd_mm \(5\)"):
        mcu.config_callback()
    # a value that rounds onto a bound is within it; one that rounds outside is not
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_back_mm": 1.9996})
    mcu.config_callback()
    assert feed_of(module).snag["snag_back_mm"] == 2.0
    assert any("back_um=2000 " in c for c, _ in mcu.config_cmds)
    _module, _printer, refused = make_feed(dict(RESPONSES), options={"snag_back_mm": 1.9994})
    with pytest.raises(Exception, match=r"snag_back_mm \(1.999\) must be within 2..20"):
        refused.config_callback()
    feed = feed_of(module)
    with pytest.raises(Exception, match=r"FREE \(200.001\) must be within 20..200"):
        feed.cmd_ACEK_SNAG_SET(FakeGcmd({"FREE": "200.0006"}))


def test_a_staged_snag_value_round_trips_exactly() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    gcmd = FakeGcmd({"FWD": "19.99999", "LAG": "4.0006", "FREE": "199.999", "BACK": "2.5"})
    feed.cmd_ACEK_SNAG_SET(gcmd)
    sent = mcu.commands[-1][1]
    assert sent == [20000, 4001, 0, 2500, 500, 199999]
    staged = {key: value for _, key, value in printer.objects["configfile"].calls}
    assert staged == {
        "snag_fwd_mm": "20",
        "snag_lag_mm": "4.001",
        "snag_back_mm": "2.5",
        "snag_free_mm": "199.999",
    }
    # the staged strings read back (SAVE_CONFIG, restart) give the same command at connect
    module, printer, mcu = make_feed(dict(RESPONSES), options=staged)
    mcu.config_callback()
    cfg = [c for c, is_init in mcu.config_cmds if is_init]
    assert (
        "ace2k_feed_snag_set fwd_um=20000 lag_um=4001 duty_pct=0 back_um=2500 rest_ms=500"
        " free_um=199999" in cfg
    )
    assert gcmd.lines[-1].startswith("ace2k: snag fwd 20 mm, lag 4.001 mm, duty 0 %, back 2.5 mm")


@pytest.mark.parametrize(
    "key, arg",
    [
        ("snag_fwd_mm", "FWD"),
        ("snag_lag_mm", "LAG"),
        ("snag_back_mm", "BACK"),
        ("snag_free_mm", "FREE"),
    ],
)
def test_a_non_finite_snag_value_is_refused_not_raised(key: str, arg: str) -> None:
    # Klipper's float parser takes inf and nan; round() would raise on them, an internal error
    for text in ("inf", "-inf", "nan"):
        module, printer, mcu = make_feed(dict(RESPONSES), options={key: float(text)})
        with pytest.raises(FakeConfig.error, match=rf"{key} \(.*\) must be a finite number"):
            mcu.config_callback()
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    snag = dict(feed.snag)
    for text in ("inf", "-inf", "nan"):
        with pytest.raises(FakeGcmd.error, match=rf"{arg} \(.*\) must be a finite number"):
            feed.cmd_ACEK_SNAG_SET(FakeGcmd({arg: text, "DUTY": "10"}))
    assert mcu.commands == commands and printer.objects["configfile"].calls == []
    assert feed.snag == snag


# --- the follow -----------------------------------------------------

FOLLOW_CONSTANTS = {
    "ACE2K_FEED_FOLLOW_FLIP_MS": 200,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MIN": 50,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MAX": 2000,
    "ACE2K_FEED_FOLLOW_TAKE_UM": 15000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MIN": 5000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MAX": 30000,
    "ACE2K_FEED_FOLLOW_TAIL_UM": 2000000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MIN": 200000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MAX": 2000000,
}
WITH_FOLLOW = dict(DEFAULT_CONSTANTS, **FOLLOW_CONSTANTS)
FOLLOW_FMT = "ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u"


def make_follow(
    responses: Responses | None = None, **kwargs: object
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu, Responses]:
    """A connected feed on an image that carries the follow."""
    responses = dict(RESPONSES) if responses is None else responses
    module, printer, mcu = make_feed(responses, constants=WITH_FOLLOW, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu, responses


def test_the_follow_constants_mirror_the_firmware_header() -> None:
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_FOLLOW_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    }
    assert defines == FOLLOW_CONSTANTS  # the fake here carries the firmware's own values


def test_the_follow_mode_names() -> None:
    assert ace2k_feed.CMD_MODES["assist_both"] == 6
    assert ace2k_feed.MODE_NAMES[8] == "following"


def test_assist_dir_both_starts_the_follow_at_the_given_speed() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = accepted(mcu)
    gcmd = FakeGcmd({"LANE": "1", "DIR": "BOTH", "SPEED": "70"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert mcu.queries[-1][1] == [0, 6, 0, 70000, 1, 0]
    assert gcmd.lines == ["ace2k: lane 1 assist_both started"]
    # lower case as the other directions, and the default speed without SPEED
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2", "DIR": "both"}))
    assert mcu.queries[-1][1] == [1, 6, 0, 50000, 1, 0]
    # DIR=FORWARD names the forward assist, as no DIR
    feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "3", "DIR": "FORWARD"}))
    assert mcu.queries[-1][1][:2] == [2, 2]


def test_assist_dir_both_soft_and_off_as_the_other_directions() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 0, "reason": 1}
    gcmd = FakeGcmd({"LANE": "2", "DIR": "BOTH", "SOFT": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert gcmd.lines == ["ace2k: lane 2 not armed — lane 2 assist_both refused: busy"]
    with pytest.raises(Exception, match="lane 2 assist_both refused: busy"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "2", "DIR": "BOTH"}))
    gcmd = FakeGcmd({"LANE": "2", "DIR": "BOTH", "OFF": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    assert gcmd.lines == ["ace2k: lane 2 assist off"]


def test_an_unknown_assist_direction_is_an_error_naming_back_and_both() -> None:
    feed, printer, mcu, responses = make_follow()
    sent = len(mcu.queries)
    with pytest.raises(Exception, match="BACK or BOTH"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "SIDEWAYS"}))
    assert len(mcu.queries) == sent


def test_the_follow_on_an_image_without_it_is_refused_before_anything_is_sent() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))  # the default fake: no follow constants
    mcu.config_callback()
    feed = feed_of(module)
    assert not feed.has_follow
    assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    with pytest.raises(Exception, match=r"no follow tail \(or no follow\); flash v0.11.0"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    gcmd = FakeGcmd({"LANE": "1", "DIR": "BOTH", "SOFT": "1"})
    feed.cmd_ACEK_ASSIST(gcmd)
    assert gcmd.lines == [
        "ace2k: lane 1 not armed — the unit's firmware has no follow tail (or no follow);"
        " flash v0.11.0"
    ]
    with pytest.raises(RuntimeError, match="no follow"):
        feed.start_move(0, "assist_both", 0.0, 70.0)
    with pytest.raises(Exception, match="no follow"):
        feed.cmd_ACEK_FOLLOW_SET(FakeGcmd({"FLIP_MS": "300"}))
    assert mcu.queries == [] and not any(c[0] == FOLLOW_FMT for c in mcu.commands)
    # a follow key on such an image is a config error at connect
    module, printer, mcu = make_feed(dict(RESPONSES), options={"follow_flip_ms": 300})
    with pytest.raises(
        Exception, match=r"follow_flip_ms.*no follow tail \(or no follow\); flash v0.11.0"
    ):
        mcu.config_callback()


def test_start_move_assist_both_returns_a_move_and_following_is_busy() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    assert isinstance(move, ace2k_feed.Move) and move.mode == "assist_both"
    assert mcu.queries[-1][1] == [0, 6, 0, 70000, 1, 0]
    assert not feed.any_busy()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([8, 0, 0, 0], [0] * 4, [70000, 0, 0, 0], [40, 0, 0, 0], [0] * 4, (1, 0, 0, 0))
    )
    assert feed.lane_status(0)["mode"] == "following"
    assert feed.any_busy()
    # the events carry the mode too
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 1, 8, 9000, 8000, seq=move.seq))
    assert move.done() and move.result()["mode"] == "following"


# --- a speed on a following lane: refused, by the unit or by the host (API_VERSION 19) -----------

FOLLOWING_SPEED = (
    "ace2k: lane 1 speed refused: the lane is following; set the follow's speed instead"
)
WITH_FOLLOWING_REFUSAL = dict(WITH_FOLLOW, ACE2K_FEED_REFUSED_FOLLOWING=8)


def following_lane(
    constants: dict[str, int],
) -> tuple[Ace2kFeed, FakeMcu, Responses, ace2k_feed.Move]:
    """A connected feed whose lane 1 follows: the start accepted, then reported as following."""
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, constants=constants)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 50.0)
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([8, 0, 0, 0], [0] * 4, [50000, 0, 0, 0], [40, 0, 0, 0], [0] * 4, (1, 0, 0, 0))
    )
    return feed, mcu, responses, move


def test_the_image_says_whether_it_refuses_a_speed_on_a_following_lane() -> None:
    assert ace2k_feed.REFUSAL_NAMES[8] == "following"
    feed, mcu, responses, move = following_lane(WITH_FOLLOWING_REFUSAL)
    assert feed.has_following_refusal is True
    feed, mcu, responses, move = following_lane(WITH_FOLLOW)
    assert feed.has_following_refusal is False


def test_a_speed_on_a_following_lane_is_refused_by_the_unit_with_the_follows_text() -> None:
    feed, mcu, responses, move = following_lane(WITH_FOLLOWING_REFUSAL)
    responses["ace2k_feed_speed_set"] = {"lane": 0, "accepted": 0, "reason": 8}
    gcmd = FakeGcmd({"LANE": "1", "SPEED": "50"})
    with pytest.raises(Exception, match=f"^{re.escape(FOLLOWING_SPEED)}$"):
        feed.cmd_ACEK_SPEED(gcmd)
    assert speed_queried(mcu) == [[0, 50000]] and gcmd.lines == []
    with pytest.raises(ace2k_feed.FeedRefused) as caught:
        feed.set_speed(0, 50.0)
    assert (caught.value.lane, caught.value.mode, caught.value.reason) == (0, "speed", "following")
    assert speed_queried(mcu) == [[0, 50000], [0, 50000]]


def test_a_speed_on_a_following_lane_is_refused_by_the_host_for_an_older_image() -> None:
    # an older image would take the speed: the host refuses it itself, with the same text
    feed, mcu, responses, move = following_lane(WITH_FOLLOW)
    responses["ace2k_feed_speed_set"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "SPEED": "50"})
    with pytest.raises(Exception, match=f"^{re.escape(FOLLOWING_SPEED)}$"):
        feed.cmd_ACEK_SPEED(gcmd)
    with pytest.raises(ace2k_feed.FeedRefused) as caught:
        feed.set_speed(0, 50.0)
    assert (caught.value.lane, caught.value.mode, caught.value.reason) == (0, "speed", "following")
    assert str(caught.value) == "ace2k: lane 1 speed refused: following"
    assert speed_queried(mcu) == [] and gcmd.lines == []
    # the bounds are still judged first, as the unit does
    with pytest.raises(ValueError):
        feed.set_speed(0, 500.0)
    # in the tail the lane still follows: refused the same
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 16, 8, 9000, 8000, seq=move.seq))
    assert feed.lanes[0]["tail"] is True
    with pytest.raises(Exception, match=f"^{re.escape(FOLLOWING_SPEED)}$"):
        feed.cmd_ACEK_SPEED(FakeGcmd({"LANE": "1", "SPEED": "50"}))
    assert speed_queried(mcu) == []


def test_the_hosts_own_refusal_reads_the_follow_before_its_first_report() -> None:
    responses = dict(RESPONSES, ace2k_feed_start={"lane": 0, "accepted": 1, "reason": 0})
    module, printer, mcu = make_feed(responses, constants=WITH_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    feed.start_move(0, "assist_both", 0.0, 50.0)
    assert feed.lanes[0]["mode"] != "following"  # no report since the start
    with pytest.raises(ace2k_feed.FeedRefused, match="following"):
        feed.set_speed(0, 50.0)
    assert speed_queried(mcu) == []


def test_an_older_image_still_gets_the_speed_of_a_lane_that_does_not_follow() -> None:
    feed, mcu, responses, move = following_lane(WITH_FOLLOW)
    # the follow ends: its final event, and the lane is no longer following to the host
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 1, 8, 9000, 8000, seq=move.seq))
    responses["ace2k_feed_speed_set"] = {"lane": 0, "accepted": 0, "reason": 7}
    with pytest.raises(Exception, match="^ace2k: lane 1 speed refused: no move running$"):
        feed.cmd_ACEK_SPEED(FakeGcmd({"LANE": "1", "SPEED": "50"}))
    assert speed_queried(mcu) == [[0, 50000]]
    # a feed: the speed is sent once the start is accepted, before and after its first report,
    # though the last report before it still said following
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.start_move(0, "feed", 100.0, 30.0)
    assert feed.lanes[0]["mode"] == "following"
    responses["ace2k_feed_speed_set"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "SPEED": "50"})
    feed.cmd_ACEK_SPEED(gcmd)
    assert speed_queried(mcu)[-1] == [0, 50000]
    assert gcmd.lines == ["ace2k: lane 1 speed 50 mm/s"]
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([1, 0, 0, 0], [0] * 4, [50000, 0, 0, 0], [40, 0, 0, 0], [0] * 4, (2, 0, 0, 0))
    )
    feed.set_speed(0, 40.0)
    assert speed_queried(mcu) == [[0, 50000], [0, 50000], [0, 40000]]


def test_the_follow_values_go_out_as_an_init_command_at_every_connect() -> None:
    feed, printer, mcu, _ = make_follow()
    follow = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_follow_set")]
    assert follow == [("ace2k_feed_follow_set flip_ms=200 take_um=15000 tail_um=2000000", True)]
    assert feed.follow == {"follow_flip_ms": 200, "follow_take_mm": 15.0, "follow_tail_mm": 2000.0}
    feed, printer, mcu, _ = make_follow(
        options={"follow_flip_ms": 300, "follow_take_mm": 20.0004, "follow_tail_mm": 750.5}
    )
    follow = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_follow_set")]
    assert follow == [("ace2k_feed_follow_set flip_ms=300 take_um=20000 tail_um=750500", True)]


def test_a_follow_key_out_of_the_dictionarys_bounds_is_a_config_error() -> None:
    for key, value in (
        ("follow_flip_ms", 49),
        ("follow_flip_ms", 2001),
        ("follow_take_mm", 4.999),
        ("follow_take_mm", 30.001),
        ("follow_take_mm", float("nan")),
        ("follow_tail_mm", 199.999),
        ("follow_tail_mm", 2000.001),
    ):
        module, printer, mcu = make_feed(
            dict(RESPONSES), options={key: value}, constants=WITH_FOLLOW
        )
        with pytest.raises(Exception, match=key):
            mcu.config_callback()
        assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    # the bounds are the dictionary's: an image with a wider window takes the value
    constants = dict(WITH_FOLLOW, ACE2K_FEED_FOLLOW_FLIP_MS_MAX=3000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"follow_flip_ms": 2500}, constants=constants
    )
    mcu.config_callback()
    assert (
        "ace2k_feed_follow_set flip_ms=2500 take_um=15000 tail_um=2000000",
        True,
    ) in mcu.config_cmds


def test_follow_set_sends_both_and_stages_what_was_given() -> None:
    feed, printer, mcu, _ = make_follow()
    gcmd = FakeGcmd({"FLIP_MS": "300", "TAKE_MM": "20", "TAIL_MM": "1500"})
    feed.cmd_ACEK_FOLLOW_SET(gcmd)
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 20000, 1500000])
    assert printer.objects["configfile"].calls == [
        ("ace2k", "follow_flip_ms", "300"),
        ("ace2k", "follow_take_mm", "20"),
        ("ace2k", "follow_tail_mm", "1500"),
    ]
    assert gcmd.lines == [
        "ace2k: follow flip 300 ms, take 20 mm, tail 1500 mm; staged for SAVE_CONFIG"
    ]
    # the values in effect carry on: a later set changes only what it names
    feed.cmd_ACEK_FOLLOW_SET(FakeGcmd({"TAKE_MM": "12.5"}))
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 12500, 1500000])
    feed.cmd_ACEK_FOLLOW_SET(FakeGcmd({"TAIL_MM": "200"}))
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 12500, 200000])


def test_follow_set_alone_prints_and_a_value_out_of_bounds_changes_nothing() -> None:
    feed, printer, mcu, _ = make_follow()
    commands = list(mcu.commands)
    gcmd = FakeGcmd()
    feed.cmd_ACEK_FOLLOW_SET(gcmd)
    assert gcmd.lines == ["ace2k: follow flip 200 ms, take 15 mm, tail 2000 mm"]
    for params in (
        {"FLIP_MS": "49"},
        {"FLIP_MS": "2001"},
        {"TAKE_MM": "4.9"},
        {"TAKE_MM": "31"},
        {"TAKE_MM": "inf"},
        {"TAIL_MM": "199.9"},
        {"TAIL_MM": "2001"},
        {"TAIL_MM": "nan"},
        {"FLIP_MS": "300", "TAKE_MM": "40"},  # a good one beside a bad one
        {"TAKE_MM": "20", "TAIL_MM": "5000"},
    ):
        with pytest.raises(Exception, match="FLIP_MS|TAKE_MM|TAIL_MM"):
            feed.cmd_ACEK_FOLLOW_SET(FakeGcmd(params))
    assert mcu.commands == commands
    assert printer.objects["configfile"].calls == []
    assert feed.follow == {"follow_flip_ms": 200, "follow_take_mm": 15.0, "follow_tail_mm": 2000.0}


def test_follow_set_on_a_firmware_without_the_feed_is_a_gcode_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed_of(module).cmd_ACEK_FOLLOW_SET(FakeGcmd({"FLIP_MS": "300"}))
    assert mcu.commands == []


# --- the tail --------------------------------------------------------

V0_10_FOLLOW = {k: v for k, v in WITH_FOLLOW.items() if "_FOLLOW_TAIL_" not in k}


def test_the_tail_kinds_a_notice_and_a_final_that_is_no_error() -> None:
    assert ace2k_feed.KIND_NAMES[16] == "tail" and ace2k_feed.KIND_NAMES[17] == "tail_out"
    assert 16 in ace2k_feed.NOTICE_KINDS and 16 not in ace2k_feed.ERROR_KINDS
    assert 17 not in ace2k_feed.NOTICE_KINDS and 17 not in ace2k_feed.ERROR_KINDS


# --- blocked ---------------------------------------------------------


def test_blocked_a_final_kind_that_is_no_error() -> None:
    assert ace2k_feed.KIND_NAMES[18] == "blocked"
    assert 18 not in ace2k_feed.ERROR_KINDS and 18 not in ace2k_feed.NOTICE_KINDS


def test_blocked_completes_a_move_the_lane_idle_without_an_error() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    move = feed.start_move(1, "feed", 100.0, 30.0)
    assert not move.done()
    mcu.subscriptions["ace2k_feed_event"](event_params(1, 18, 1, 42000, 9000, seq=move.seq))
    assert move.done() and nothing_tracked(feed)
    assert move.result()["kind"] == "blocked" and move.result()["mode"] == "feeding"
    assert move.result()["motor_mm"] == 42.0 and move.result()["filament_mm"] == 9.0
    assert feed.lanes[1]["last_move"]["kind"] == "blocked"
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (0, 1, 0, 0))
    )
    assert feed.lanes[1]["mode"] == "idle" and feed.lanes[1]["error"] is None


def test_a_waited_feed_or_load_ending_blocked_answers_a_line_naming_the_odometers() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 18, 1, 42000, 9000, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACEK_FEED(gcmd)  # no G-code error
    assert gcmd.lines[-1] == "ace2k: lane 1 feed blocked — motor 42.0 mm, filament 9.0 mm"
    responses["ace2k_feed_start"] = accepted(mcu, event_params(3, 18, 4, 180000, 175500, seq=1))
    gcmd = FakeGcmd({"LANE": "4"})
    feed.cmd_ACEK_LOAD(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 4 load blocked — motor 180.0 mm, filament 175.5 mm"
    assert nothing_tracked(feed)


def test_the_tail_notice_completes_no_move_and_the_tail_out_completes_the_follow() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    deliver = mcu.subscriptions["ace2k_feed_event"]
    deliver(event_params(0, 16, 8, 400000, 398000, seq=move.seq))
    assert not move.done() and feed.motion[0].tracked[0] == move.seq
    assert feed.lanes[0]["last_event"]["kind"] == "tail"
    assert feed.lanes[0]["last_move"] is None
    deliver(event_params(0, 17, 8, 650000, 610000, seq=move.seq))
    assert move.done() and nothing_tracked(feed)
    assert move.result()["kind"] == "tail_out" and move.result()["mode"] == "following"
    assert move.result()["motor_mm"] == 650.0 and move.result()["filament_mm"] == 610.0
    assert feed.lanes[0]["last_move"]["kind"] == "tail_out"
    assert feed.lanes[0]["error"] is None


def test_a_waited_follow_ends_on_tail_out_without_a_gcode_error() -> None:
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = accepted(
        mcu,
        event_params(0, 16, 8, 400000, 398000, seq=1),
        event_params(0, 17, 8, 650000, 610000, seq=1),
    )
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    assert move.done() and move.result()["kind"] == "tail_out"


def test_the_lanes_tail_from_the_notice_to_leaving_the_follow() -> None:
    feed, printer, mcu, responses = make_follow()
    state = mcu.subscriptions["ace2k_feed_state"]
    deliver = mcu.subscriptions["ace2k_feed_event"]
    following = state_frame([8, 0, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    idle = state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    assert all(feed.lane_status(i)["tail"] is False for i in range(ace2k_feed.LANES))
    state(following)
    assert feed.lane_status(0)["tail"] is False
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True and feed.lane_status(1)["tail"] is False
    state(following)
    assert feed.lane_status(0)["tail"] is True  # still following
    state(idle)  # the lane left following (its final event lost on the way)
    assert feed.lane_status(0)["tail"] is False
    # the final event ends it too, the report not yet in
    state(following)
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True
    deliver(event_params(0, 17, 8, 9000, 8000, seq=1))
    assert feed.lane_status(0)["tail"] is False
    # a notice of another kind leaves it as it is
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    deliver(event_params(0, 14, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True


def sensors_frame(inserts: Sequence[bool]) -> dict[str, object]:
    """One ace2k_sensors_state frame with the given lanes' inserts up, nothing else."""
    switches = sum(1 << i for i, up in enumerate(inserts) if up)
    return {
        "switches": switches,
        "insert_mv": b"\x00" * 8,
        "empty_mv": b"\x00" * 8,
        "aux_mv": 0,
        "age_ms": 0,
        "#sent_time": 1.0,
    }


def make_tail_unit() -> tuple[Ace2k, Ace2kFeed, FakeMcu]:
    """[ace2k] with the feed on an image with the follow, lane 1 following."""
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.unit is module
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([8, 0, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    )
    return module, feed, mcu


def test_the_lanes_tail_outlives_a_filament_pushed_in_at_the_bay() -> None:
    # the tail ignores the bay: the insert rising during the tail
    # changes nothing, the tail ends on its final event or the lane leaving the follow
    module, feed, mcu = make_tail_unit()
    sensors = mcu.subscriptions["ace2k_sensors_state"]
    deliver = mcu.subscriptions["ace2k_feed_event"]
    sensors(sensors_frame([True, True, True, True]))
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True
    sensors(sensors_frame([False, True, True, True]))  # the fall reported
    assert feed.lane_status(0)["tail"] is True
    sensors(sensors_frame([True, True, True, True]))  # a filament pushed in at the bay
    assert feed.lane_status(0)["tail"] is True
    assert module.get_status(0.0)["lanes"][0]["tail"] is True
    sensors(sensors_frame([False, True, True, True]))  # and out again, fast
    sensors(sensors_frame([True, True, True, True]))
    assert feed.lanes[0]["tail"] is True
    deliver(event_params(0, 17, 8, 650000, 610000, seq=1))  # the tail-out ends it
    assert feed.lane_status(0)["tail"] is False
    assert module.get_status(0.0)["lanes"][0]["tail"] is False
    assert feed.lanes[0]["last_move"]["kind"] == "tail_out"


def test_the_feed_holds_no_insert_watch_of_its_own() -> None:
    module, feed, mcu = make_tail_unit()
    assert not hasattr(feed, "on_inserts") and not hasattr(feed, "tail_insert_down")
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 16, 8, 1000, 1000, seq=1))
    module.lanes[0]["insert"] = True  # whatever the insert reads, the status only reads
    assert feed.lane_status(0)["tail"] is True and feed.lanes[0]["tail"] is True


def test_an_image_without_the_tail_refuses_only_the_follow_with_the_flash_line() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, constants=V0_10_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present and not feed.has_follow
    assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    with pytest.raises(Exception, match=r"no follow tail \(or no follow\); flash v0.11.0"):
        feed.cmd_ACEK_FOLLOW_SET(FakeGcmd({"TAIL_MM": "500"}))
    with pytest.raises(Exception, match="flash v0.11.0"):
        feed.cmd_ACEK_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    assert not any(c[0].startswith("ace2k_feed_follow_set") for c in mcu.commands)
    # the rest of the feed works on that image
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries[-1][1] == [0, 0, 50000, 30000, 1, 0] and not move.done()
    # a follow key there, the tail's or another, is a config error naming the flash
    for key, value in (("follow_tail_mm", 500.0), ("follow_flip_ms", 300)):
        module, printer, mcu = make_feed(
            dict(RESPONSES), options={key: value}, constants=V0_10_FOLLOW
        )
        with pytest.raises(
            Exception, match=rf"{key}.*no follow tail \(or no follow\); flash v0.11.0"
        ):
            mcu.config_callback()


# --- the host interface -----------------------------------------------------------------------


def test_api_version() -> None:
    assert ace2k_feed.API_VERSION == 29


def test_the_feed_object_carries_the_api_version() -> None:
    # a caller holds printer.lookup_object("ace2k").feed, not the module
    module, printer, mcu = make_feed(dict(RESPONSES))
    assert feed_of(module).API_VERSION == ace2k_feed.API_VERSION == 29


def test_start_move_returns_a_move_completed_by_its_final_event() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries[-1][1] == [0, 0, 50000, 30000, 1, 0]
    assert move.lane == 0 and move.mode == "feed" and move.seq == 1
    assert not move.done() and move.result() is None
    assert feed.motion[0].tracked[0] == 1
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 50000, 49500, seq=move.seq))
    assert move.done() and nothing_tracked(feed)
    assert move.result()["kind"] == "done" and move.result()["filament_mm"] == 49.5
    assert move.wait(1.0)["kind"] == "done"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0


def test_start_move_refused_raises_feed_refused_with_the_reason() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 3}
    with pytest.raises(ace2k_feed.FeedRefused) as err:
        feed.start_move(0, "feed", 50.0, 30.0)
    assert err.value.reason == "no_filament" and err.value.lane == 0
    assert str(err.value) == "ace2k: lane 1 feed refused: no_filament"
    assert feed.motion[0].tracked is None and feed.motion[0].commanded_mm == {}


def test_a_notice_or_another_sequence_does_not_complete_a_move() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    move = feed.start_move(1, "assist", 0.0, 50.0)
    deliver = mcu.subscriptions["ace2k_feed_event"]
    deliver(event_params(1, 15, 2, 5000, 4000, seq=move.seq))  # snag, a notice
    deliver(event_params(1, 14, 2, 5000, 4000, seq=move.seq))  # behind, a notice
    deliver(event_params(1, 16, 8, 5000, 4000, seq=move.seq))  # tail, a notice
    deliver(event_params(1, 1, 2, 5000, 4000, seq=0))  # a motion the unit started itself
    assert not move.done() and move.result() is None
    assert move.wait(0.0) is None
    deliver(event_params(1, 1, 2, 9000, 8000, seq=move.seq))  # stopped
    assert move.done() and move.result()["kind"] == "stopped"


def test_start_move_without_the_feed_or_with_a_bad_mode_raises_and_sends_nothing() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    with pytest.raises(ValueError):
        feed.start_move(0, "sideways", 50.0, 30.0)
    with pytest.raises(ValueError):
        feed.start_move(4, "feed", 50.0, 30.0)
    assert "ace2k_feed_start" not in mcu.sent and nothing_tracked(feed)
    feed.present = False
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.start_move(0, "feed", 50.0, 30.0)


def test_stop_and_clear_without_the_feed_raise_runtime_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.present = False
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.stop(0)
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.stop(ace2k_feed.LANE_ALL)
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.clear(0)


@pytest.mark.parametrize("lane", [True, False])
def test_every_host_call_refuses_a_bool_as_a_lane(lane: bool) -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    # the lane is judged before the image's support
    feed.has_grip = feed.has_park = feed.has_wait = True
    sent = len(mcu.commands)
    calls = [
        lambda: feed.start_move(lane, "feed", 50.0, 30.0),
        lambda: feed.set_speed(lane, 30.0),
        lambda: feed.stop(lane),
        lambda: feed.clear(lane),
        lambda: feed.set_lane_extruder(lane, None),
        lambda: feed.set_lane_follow(lane),
        lambda: feed.follow_paused(lane),
        lambda: feed.hold_follow(lane, True),
        lambda: feed.default_move(lane),
        lambda: feed.set_lane_grip(lane, 0.0),
        lambda: feed.set_lane_park(lane),
        lambda: feed.set_lane_wait(lane),
        lambda: feed.apply_scale(lane, 1.2),
    ]
    for call in calls:
        with pytest.raises(ValueError):
            call()
    assert len(mcu.commands) == sent


def test_stop_and_clear_send_their_commands() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.stop(2)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [2])
    feed.clear(1)
    assert mcu.commands[-1] == ("ace2k_feed_clear lane=%c", [1])
    feed.stop(ace2k_feed.LANE_ALL)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [255])
    # another lane is refused before anything is sent; LANE_ALL is a stop's only
    sent = list(mcu.commands)
    for bad in (4, -1):
        with pytest.raises(ValueError):
            feed.stop(bad)
        with pytest.raises(ValueError):
            feed.clear(bad)
    with pytest.raises(ValueError):
        feed.clear(ace2k_feed.LANE_ALL)
    assert mcu.commands == sent


def test_a_refused_start_leaves_the_running_move_tracked() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    first = feed.start_move(0, "feed", 500.0, 30.0)
    # a second start on the busy lane: the unit refuses it, and the first move is still the
    # lane's — through the interface and through a waiting G-code alike
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert feed.motion[0].tracked[0] == first.seq
    with pytest.raises(Exception, match="lane 1 feed refused: busy"):
        feed.cmd_ACEK_FEED(FakeGcmd({"LANE": "1", "LENGTH": "50"}))
    assert feed.motion[0].tracked[0] == first.seq
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 500000, 499000, seq=first.seq))
    assert first.done() and first.result()["kind"] == "done" and nothing_tracked(feed)
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 500.0


def test_a_running_moves_end_inside_a_refused_starts_round_trip_completes_it_once() -> None:
    # the earlier move ends just as the next start goes out: its final event is dispatched
    # while the refused start holds the lane's slot, and the refusal completes it from that event
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    first = feed.start_move(0, "feed", 50.0, 30.0)
    completed = []
    original = first._completion.complete

    def counting(result: object) -> None:
        completed.append(result)
        original(result)

    first._completion.complete = counting
    final = event_params(0, 0, 1, 50000, 49000, seq=first.seq)

    def busy_with_the_end_in_flight() -> dict[str, int]:
        mcu.subscriptions["ace2k_feed_event"](final)
        assert not first.done()  # dispatched while the refused start held the slot
        return {"lane": 0, "accepted": 0, "reason": 1}

    responses["ace2k_feed_start"] = busy_with_the_end_in_flight
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert first.done() and first.result()["kind"] == "done" and nothing_tracked(feed)
    assert first.result() is feed.lanes[0]["last_event"]
    # the same event dispatched again (a _dispatch still queued) completes nothing twice
    feed._dispatch(0, first.result(), 0)
    assert len(completed) == 1


def test_a_move_is_done_before_the_klipper_event_fires() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 2, "accepted": 1, "reason": 0}
    move = feed.start_move(2, "rollback", 40.0, 30.0)
    seen = []

    def send_event(name: str, *args: object) -> NoReturn:
        seen.append(move.done())
        raise RuntimeError("a listener that fails")

    printer.send_event = send_event
    with pytest.raises(RuntimeError, match="a listener that fails"):
        mcu.subscriptions["ace2k_feed_event"](event_params(2, 0, 2, 40000, 39000, seq=move.seq))
    assert seen == [True] and move.result()["kind"] == "done"


def test_start_move_before_any_state_report_is_a_runtime_error() -> None:
    # no report within the bound (the G-code path's text is the same, as
    # test_a_start_before_the_first_state_report_waits_for_it_then_sends_reported_plus_one checks)
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    with pytest.raises(RuntimeError, match=r"no feed state report within 7 s"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries == [] and not feed.motion[0].started and nothing_tracked(feed)
    # the report landing during the wait: the start goes out, seeded from it
    printer.reactor.while_waiting[feed.first_report] = [
        lambda waketime: mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)
    ]
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert move.seq == 1 and mcu.queries[-1][1] == [0, 0, 50000, 30000, 1, 0]


# --- the feed-forward ------------------------------------------

FF_CONSTANTS = {
    "ACE2K_FEED_FF_CHUNK_UM": 3000,
    "ACE2K_FEED_FF_CHUNK_UM_MIN": 1000,
    "ACE2K_FEED_FF_CHUNK_UM_MAX": 10000,
    "ACE2K_FEED_FF_PULSE_UM_S": 15000,
    "ACE2K_FEED_FF_PULSE_UM_S_MIN": 9000,
    "ACE2K_FEED_FF_PULSE_UM_S_MAX": 80000,
}
WITH_FF = dict(WITH_FOLLOW, **FF_CONSTANTS)
FF_SET_FMT = "ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u"


def make_ff_feed(
    responses: Responses | None = None, **kwargs: object
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu, Ace2k]:
    """A connected feed on an image that carries the feed-forward."""
    responses = dict(RESPONSES) if responses is None else responses
    module, printer, mcu = make_feed(responses, constants=WITH_FF, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu, module


def ff_state_frame(
    doses: Sequence[int],
    taut: Sequence[int],
    full: Sequence[int],
    epoch: Sequence[int] = (0, 0, 0, 0),
) -> dict[str, bytes]:
    return {
        "doses": struct.pack("<4H", *doses),
        "taut": struct.pack("<4H", *taut),
        "full": struct.pack("<4H", *full),
        "epoch": bytes(epoch),
    }


def test_the_ff_constants_mirror_the_firmware_header() -> None:
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = dict(
        (name, int(value))
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_FF_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    )
    # the pulse's bounds are the lane's speed bounds, by name in the header
    assert defines == {k: v for k, v in FF_CONSTANTS.items() if "PULSE_UM_S_M" not in k}
    assert (
        FF_CONSTANTS["ACE2K_FEED_FF_PULSE_UM_S_MIN"]
        == DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MIN_UM_S"]
    )
    assert (
        FF_CONSTANTS["ACE2K_FEED_FF_PULSE_UM_S_MAX"]
        == DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MAX_UM_S"]
    )


def test_the_ff_values_and_query_go_out_as_init_commands_at_every_connect() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    init = [c for c, is_init in mcu.config_cmds if is_init and c.startswith("ace2k_feed_")]
    assert init == [
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=15000 stall_check_um=20000",
        "ace2k_feed_load_set park_um=300000 speed_um_s=60000 auto_load=1",
        "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k_feed_follow_set flip_ms=200 take_um=15000 tail_um=2000000",
        "ace2k_feed_ff_set chunk_um=3000 pulse_um_s=15000",
        "ace2k_feed_query rest_ticks=120000000",
        "ace2k_feed_ff_query rest_ticks=120000000",
    ]
    assert feed.has_ff and "ace2k_feed_ff_state" in mcu.subscriptions
    feed, printer, mcu, _ = make_ff_feed(options={"ff_chunk_mm": 4.5, "ff_pulse": 20})
    assert ("ace2k_feed_ff_set chunk_um=4500 pulse_um_s=20000", True) in mcu.config_cmds


FF_PULSE_RETIRED = (
    "ace2k: ff_pulse is retired: every forward dose runs at the floor speed, so it sets only a"
    " reverse dose's least speed. The next version removes it, with ACEK_FF_SET's PULSE, and the"
    " unit's own value is used. Remove it from printer.cfg."
)


def test_an_ff_pulse_in_printer_cfg_is_said_retired_and_still_sent(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level("WARNING", logger="root"):
        feed, printer, mcu, _ = make_ff_feed(options={"ff_pulse": 20})
    assert printer.objects["configfile"].deprecated == [("ace2k", "ff_pulse")]
    assert [r.getMessage() for r in caplog.records if "ff_pulse" in r.getMessage()] == [
        FF_PULSE_RETIRED
    ]
    assert any(c.endswith("pulse_um_s=20000") for c, _ in mcu.config_cmds)  # it still works


def test_without_ff_pulse_nothing_is_said_retired(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level("WARNING", logger="root"):
        feed, printer, mcu, _ = make_ff_feed(options={"ff_chunk_mm": 4})
    assert printer.objects["configfile"].deprecated == []
    assert not [r for r in caplog.records if "ff_pulse" in r.getMessage()]


def test_ff_set_with_pulse_says_it_is_retired_and_still_sends_it() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    gcmd = FakeGcmd({"PULSE": "20"})
    feed.cmd_ACEK_FF_SET(gcmd)
    assert gcmd.lines[-1] == FF_PULSE_RETIRED
    assert mcu.commands[-1] == (FF_SET_FMT, [3000, 20000])  # it still works
    gcmd = FakeGcmd({"CHUNK_MM": "4"})
    feed.cmd_ACEK_FF_SET(gcmd)
    assert not [line for line in gcmd.lines if "retired" in line]


def test_an_ff_key_out_of_the_dictionarys_bounds_is_a_config_error() -> None:
    for key, value in (
        ("ff_chunk_mm", 0.9),
        ("ff_chunk_mm", 10.001),
        ("ff_pulse", 8.9),
        ("ff_pulse", 81),
    ):
        module, printer, mcu = make_feed(dict(RESPONSES), options={key: value}, constants=WITH_FF)
        with pytest.raises(Exception, match=key):
            mcu.config_callback()


def test_an_image_without_the_ff_constants_gets_no_ff_command() -> None:
    feed, printer, mcu, _ = make_follow(options={"ff_chunk_mm": 4})
    assert not feed.has_ff
    assert not any("_ff_" in c for c, _ in mcu.config_cmds)
    assert "ace2k_feed_ff_state" not in mcu.subscriptions
    with pytest.raises(Exception, match="flash v0.9.0"):
        feed.cmd_ACEK_FF_SET(FakeGcmd({"CHUNK_MM": "4"}))
    assert mcu.commands == []


def test_ff_set_sends_both_checks_the_bounds_and_tunes_the_host_side() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    gcmd = FakeGcmd({"CHUNK_MM": "4", "PULSE": "20"})
    feed.cmd_ACEK_FF_SET(gcmd)
    assert mcu.commands[-1] == (FF_SET_FMT, [4000, 20000])
    assert ("ace2k", "ff_chunk_mm", "4") in printer.objects["configfile"].calls
    feed.cmd_ACEK_FF_SET(FakeGcmd({"PULSE": "12.5"}))
    assert mcu.commands[-1] == (FF_SET_FMT, [4000, 12500])
    commands = list(mcu.commands)
    for params in (
        {"CHUNK_MM": "0.5"},
        {"CHUNK_MM": "11"},
        {"PULSE": "8"},
        {"PULSE": "81"},
        {"PULSE": "nan"},
        {"CHUNK_MM": "3", "PULSE": "100"},  # a good one beside a bad one
        {"WINDOW_MS": "10"},
        {"LEAD_MS": "-1"},
        {"DEADBAND": "-0.1"},
    ):
        with pytest.raises(Exception, match="CHUNK_MM|PULSE|WINDOW_MS|LEAD_MS|DEADBAND"):
            feed.cmd_ACEK_FF_SET(FakeGcmd(params))
    assert mcu.commands == commands
    # the host-side values: nothing sent to the unit
    feed.cmd_ACEK_FF_SET(FakeGcmd({"WINDOW_MS": "800", "LEAD_MS": "100", "DEADBAND": "0.5"}))
    assert mcu.commands == commands
    assert (feed.ff.window_ms, feed.ff.lead_ms, feed.ff.deadband_mm_s) == (800, 100, 0.5)
    gcmd = FakeGcmd()
    feed.cmd_ACEK_FF_SET(gcmd)
    assert gcmd.lines == [
        "ace2k: feed-forward chunk 4 mm, pulse 12.5 mm/s, window 800 ms, lead 100 ms,"
        " deadband 0.5 mm/s, long retraction 10 mm"
    ]
    # a window under the period is raised to it: both are said
    feed.ff.period_ms = 1000
    feed.cmd_ACEK_FF_SET(FakeGcmd({"WINDOW_MS": "100"}))
    gcmd = FakeGcmd()
    feed.cmd_ACEK_FF_SET(gcmd)
    assert gcmd.lines == [
        "ace2k: feed-forward chunk 4 mm, pulse 12.5 mm/s, window 100 ms (1000 ms in effect),"
        " lead 100 ms, deadband 0.5 mm/s, long retraction 10 mm"
    ]


def test_the_ff_frame_lands_in_lane_status() -> None:
    feed, printer, mcu, module = make_ff_feed()
    lane = module.get_status(0)["lanes"][2]
    assert (lane["ff_doses"], lane["ff_taut_fixes"], lane["ff_full_fixes"]) == (None, None, None)
    assert lane["ff_rate"] == 0.0 and lane["ff_on"] is True and lane["ff_extruder"] is None
    mcu.subscriptions["ace2k_feed_ff_state"](
        ff_state_frame((1, 2, 300, 4), (0, 0, 7, 0), (0, 0, 5, 0))
    )
    lane = module.get_status(0)["lanes"][2]
    assert (lane["ff_doses"], lane["ff_taut_fixes"], lane["ff_full_fixes"]) == (300, 7, 5)


# --- the automatic encoder calibration ------------------

SCALE_FMT = "ace2k_lane_scale_set lane=%c um_per_count_x10=%u"


def test_apply_scale_puts_the_scale_in_use_keeps_the_encoder_continuous_and_stages_it() -> None:
    feed, printer, mcu, module = make_ff_feed()
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1000000, 0, 0, 0], [0] * 4))
    feed.apply_scale(0, 1.2468)
    assert mcu.commands == [(SCALE_FMT, [0, 12468])]
    # the first frame at the new scale: the offset reckoned from the scale it replaces,
    # 810.24 counts × (1.2342 - 1.2468) mm
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1010209, 0, 0, 0], [0] * 4))
    assert module.encoder_offset_um == [-10209, 0, 0, 0]
    assert module.get_status(0.0)["lanes"][0]["encoder_mm"] == 1000.0
    lane = feed.lanes[0]
    assert (lane["encoder_scale"], lane["scale_source"]) == (1.2468, "calibrated")
    assert feed.scales == {0: 1.2468}
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.2468")]
    assert "scale=1.2468 (calibrated)" in feed.status_lines()[0]
    feed.apply_scale(2, 1.25)  # staged to four decimals, as the manual mode stages it
    assert mcu.commands[-1] == (SCALE_FMT, [2, 12500])
    assert printer.objects["configfile"].calls[-1] == ("ace2k", "lane3_encoder_scale", "1.2500")
    # lane 3 at 0: no offset to keep, nothing pending
    assert module.encoder_offset_um == [-10209, 0, 0, 0] and module.scale_pending == [None] * 4


def test_apply_scale_takes_the_reference_before_the_send() -> None:
    # the unit cannot be on the new scale before it has the command: a frame handled between the
    # reference and the send is at the old scale, and one at the new scale can only come after
    feed, printer, mcu, module = make_ff_feed()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    old_raw, new_raw = 810 * 12342 // 10, 810 * 12468 // 10  # 810 counts at each scale
    counters(lane_counters_frame([old_raw, 0, 0, 0], [0] * 4))
    lookup = mcu.lookup_command

    def lookup_racing(msgformat: str, cq: object = None) -> FakeCommand:
        cmd = lookup(msgformat, cq)
        if msgformat == SCALE_FMT:
            send = cmd.send

            def racing_send(data: Sequence[Any] = (), minclock: int = 0, reqclock: int = 0) -> None:
                counters(lane_counters_frame([old_raw, 0, 0, 0], [0] * 4))  # was on its way
                send(data)
                counters(lane_counters_frame([new_raw, 0, 0, 0], [0] * 4))  # taken at once

            cmd.send = racing_send
        return cmd

    mcu.lookup_command = lookup_racing
    feed.apply_scale(0, 1.2468)
    readings = [module.get_status(0.0)["lanes"][0]["encoder_mm"]]
    for _ in range(3):  # at rest
        counters(lane_counters_frame([new_raw, 0, 0, 0], [0] * 4))
        readings.append(module.get_status(0.0)["lanes"][0]["encoder_mm"])
    assert readings == [999.702] * 4 and module.scale_pending[0] is None
    assert mcu.commands == [(SCALE_FMT, [0, 12468])]


def test_apply_scale_refuses_a_lane_or_a_scale_the_unit_would_not_take() -> None:
    feed, printer, mcu, module = make_ff_feed()
    for index, scale in ((4, 1.2468), (-1, 1.2468), (0, 1.49), (0, 0.98), (0, float("nan"))):
        with pytest.raises(ValueError, match=r"no lane|outside the unit's window"):
            feed.apply_scale(index, scale)
    assert mcu.commands == [] and printer.objects["configfile"].calls == []
    assert feed.lanes[0]["encoder_scale"] == 1.2342 and feed.scales == {}
    # the window's own ends are the unit's: 9874 and 14810 tenths of a µm per count
    feed.apply_scale(1, feed.scale_low)
    feed.apply_scale(2, feed.scale_high)
    assert mcu.commands == [(SCALE_FMT, [1, 9874]), (SCALE_FMT, [2, 14810])]


def test_every_connect_sends_each_lanes_scale_so_a_live_one_ends_with_the_session() -> None:
    # the session: lane 1's scale applied live and staged, never saved
    feed, printer, mcu, _ = make_ff_feed(options={"lane2_encoder_scale": 1.25})
    feed.apply_scale(0, 1.2468)
    # a host restart finds the unit configured, its CRC unchanged: Klipper sends it only the
    # init commands, which carry every lane's scale as the new session holds it
    feed, printer, mcu, _ = make_ff_feed(options={"lane2_encoder_scale": 1.25})
    scales = [(c, is_init) for c, is_init in mcu.config_cmds if c.startswith("ace2k_lane_scale")]
    assert [c for c, is_init in scales if is_init] == [
        "ace2k_lane_scale_set lane=0 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=1 um_per_count_x10=12500",
        "ace2k_lane_scale_set lane=2 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=3 um_per_count_x10=12342",
    ]
    assert feed.lanes[0]["encoder_scale"] == 1.2342
    # never a config command: editing or removing the key (SAVE_CONFIG's calibration
    # included) never resets the MCU
    assert [c for c, is_init in scales if not is_init] == []


def test_apply_scale_sends_and_stages() -> None:
    module, printer, mcu = make(dict(RESPONSES))
    mcu.config_callback()  # the command is built there
    feed = module.feed
    feed.apply_scale(1, 1.25)
    assert feed.mcu.commands == [(SCALE_FMT, [1, 12500])]
    assert printer.objects["configfile"].calls == [("ace2k", "lane2_encoder_scale", "1.2500")]


def test_set_counts_period_sends_the_ff_query_only_on_an_image_with_the_feed_forward() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    feed.set_counts_period(0.1)
    assert mcu.commands == [("ace2k_feed_ff_query rest_ticks=%u", [12000000])]
    feed, printer, mcu, _ = make_follow()
    feed.set_counts_period(0.1)
    assert mcu.commands == [] and not feed.has_ff


def test_the_ff_frame_reaches_the_calibration_with_its_receive_time() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    heard = []
    feed.cal.on_ff_state = lambda rt, taut, epoch: heard.append((rt, taut, tuple(epoch)))
    frame = ff_state_frame((0, 0, 0, 0), (5, 6, 7, 8), (0, 0, 0, 0), epoch=(1, 2, 3, 4))
    mcu.subscriptions["ace2k_feed_ff_state"](dict(frame, **{"#receive_time": 12.5}))
    mcu.subscriptions["ace2k_feed_ff_state"](frame)  # none in the frame: 0.0
    assert heard == [(12.5, (5, 6, 7, 8), (1, 2, 3, 4)), (0.0, (5, 6, 7, 8), (1, 2, 3, 4))]


def test_the_calibration_keeps_its_lanes_frames_only_while_a_run_holds_it() -> None:
    feed, printer, mcu, _ = make_ff_feed()
    counts = ff_state_frame((0, 0, 0, 0), (5, 6, 7, 8), (0, 0, 0, 0), epoch=(1, 2, 3, 4))
    reading = lane_counters_frame([1000, 2000, 2468, 4000], [0] * 4)
    mcu.subscriptions["ace2k_feed_ff_state"](dict(counts, **{"#receive_time": 12.0}))
    mcu.subscriptions["ace2k_lane_counters"](dict(reading, **{"#receive_time": 12.1}))
    assert list(feed.cal.frames) == []  # between runs
    feed.cal.lane = 2
    mcu.subscriptions["ace2k_feed_ff_state"](dict(counts, **{"#receive_time": 12.2}))
    mcu.subscriptions["ace2k_lane_counters"](dict(reading, **{"#receive_time": 12.3}))
    assert list(feed.cal.frames) == [("counts", 12.2, 7, 3), ("reading", 12.3, 2.468)]


# --- the grip ---------------------------------------------------------

GRIP_CONSTANTS = {
    "ACE2K_FEED_GRIP_UM": 40000,
    "ACE2K_FEED_GRIP_UM_MIN": 10000,
    "ACE2K_FEED_GRIP_UM_MAX": 45000,
}
WITH_GRIP = dict(WITH_FOLLOW, **GRIP_CONSTANTS)
GRIP_FMT = "ace2k_feed_lane_grip lane=%c grip_um=%u"


def make_grip(**kwargs: object) -> tuple[Ace2kFeed, FakePrinter, FakeMcu]:
    """A connected feed on an image that carries the grip."""
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_GRIP, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu


def test_the_grip_constants_mirror_the_firmware_header() -> None:
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = dict(re.findall(r"^#define\s+(ACE2K_FEED_\w+)\s+(\d+)U\b", header, flags=re.M))
    assert int(defines["ACE2K_FEED_GRIP_UM"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM"]
    assert int(defines["ACE2K_FEED_GRIP_UM_MIN"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MIN"]
    assert int(defines["ACE2K_FEED_GRIP_UM_MAX"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MAX"]
    # the upper bound stops short of the load's grace (strict there): the whole grip never meets
    # the comparator
    assert GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MAX"] < int(defines["ACE2K_FEED_LOAD_GRACE_UM"])


def test_the_grip_is_read_from_the_dictionary_and_every_lanes_cleared_at_every_connect() -> None:
    # the grip lives in the unit's RAM: a host restart that leaves the MCU running sends it only
    # the init commands, which clear a grip the last session set
    for _ in range(2):  # a first connect, and a host restart
        feed, printer, mcu = make_grip()
        assert feed.has_grip
        assert (feed.grip_mm, feed.grip_min_mm, feed.grip_max_mm) == (40.0, 10.0, 45.0)
        grips = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_lane_grip")]
        assert grips == [(f"ace2k_feed_lane_grip lane={lane} grip_um=0", True) for lane in range(4)]
        assert mcu.commands == []


def test_an_image_without_the_grip_gets_no_clear_at_connect() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    assert not any(c.startswith("ace2k_feed_lane_grip") for c, _ in mcu.config_cmds)


def test_set_lane_grip_sends_the_lanes_grip_and_zero_clears() -> None:
    feed, printer, mcu = make_grip()
    feed.set_lane_grip(2, 40.0)
    feed.set_lane_grip(0, 10.0)
    feed.set_lane_grip(3, 45.0)
    feed.set_lane_grip(2, 0)
    assert mcu.commands == [
        (GRIP_FMT, [2, 40000]),
        (GRIP_FMT, [0, 10000]),
        (GRIP_FMT, [3, 45000]),
        (GRIP_FMT, [2, 0]),
    ]


def test_set_lane_grip_refuses_a_grip_out_of_bounds_or_another_lane_before_sending() -> None:
    feed, printer, mcu = make_grip()
    for grip in (9.999, 45.001, 50.0, 5.0, 300.0):
        with pytest.raises(ValueError, match=r"outside the unit's 10\.\.45 mm \(0 clears\)"):
            feed.set_lane_grip(0, grip)
    for lane in (-1, 4, 255):
        with pytest.raises(ValueError, match="no lane"):
            feed.set_lane_grip(lane, 40.0)
    assert mcu.commands == []


def test_an_image_without_the_grip_says_flash_and_sends_nothing() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present and not feed.has_grip
    with pytest.raises(RuntimeError, match="no grip; flash v0.11.0"):
        feed.set_lane_grip(0, 40.0)
    with pytest.raises(Exception, match="no grip; flash v0.11.0"):
        feed.cmd_ACEK_GRIP(FakeGcmd({"LANE": "1", "MM": "40"}))
    assert mcu.commands == []


def test_a_grip_command_of_another_format_is_refused_at_connect() -> None:
    old = "ace2k_feed_lane_grip lane=%c grip_um=%hu"
    module, printer, mcu = make_feed(
        dict(RESPONSES), constants=WITH_GRIP, formats={"ace2k_feed_lane_grip": old}
    )
    with pytest.raises(Exception, match="ace2k_feed_lane_grip has another format"):
        mcu.config_callback()


PARK_FMT = "ace2k_feed_lane_park lane=%c"


def test_set_lane_park_sends_the_lane_and_has_park_says_the_image_carries_it() -> None:
    feed, printer, mcu = make_grip()
    assert feed.has_park
    feed.set_lane_park(3)
    feed.set_lane_park(0)
    assert mcu.commands == [(PARK_FMT, [3]), (PARK_FMT, [0])]
    for lane in (-1, 4, 255):
        with pytest.raises(ValueError, match="no lane"):
            feed.set_lane_park(lane)
    assert len(mcu.commands) == 2


def test_an_image_with_the_grip_and_no_park_says_flash_and_sends_nothing() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_GRIP,
        lookup_errors={"ace2k_feed_lane_park": "Unknown command: ace2k_feed_lane_park"},
    )
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.has_grip and not feed.has_park
    with pytest.raises(
        RuntimeError, match=r"no park; flash v0\.12\.0 or later \(0\.12\.0-rc\.1 has none\)"
    ):
        feed.set_lane_park(0)
    assert mcu.commands == []


def test_a_park_command_of_another_format_is_refused_at_connect() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_GRIP,
        formats={"ace2k_feed_lane_park": "ace2k_feed_lane_park lane=%hu"},
    )
    with pytest.raises(Exception, match="ace2k_feed_lane_park has another format"):
        mcu.config_callback()


def test_an_image_without_the_grip_has_no_park() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    assert not feed_of(module).has_park


WAIT_FMT = "ace2k_feed_lane_wait lane=%c"


def test_set_lane_wait_sends_the_lane_and_has_wait_says_the_image_carries_it() -> None:
    feed, printer, mcu = make_grip()
    assert feed.has_wait
    feed.set_lane_wait(2)
    feed.set_lane_wait(0)
    assert mcu.commands == [(WAIT_FMT, [2]), (WAIT_FMT, [0])]
    for lane in (-1, 4, 255):
        with pytest.raises(ValueError, match="no lane"):
            feed.set_lane_wait(lane)
    assert len(mcu.commands) == 2


def test_an_image_with_the_grip_and_no_wait_says_flash_and_sends_nothing() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_GRIP,
        lookup_errors={"ace2k_feed_lane_wait": "Unknown command: ace2k_feed_lane_wait"},
    )
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.has_grip and feed.has_park and not feed.has_wait
    with pytest.raises(RuntimeError, match=r"no wait; flash v0\.13\.0 or later"):
        feed.set_lane_wait(0)
    assert mcu.commands == []


def test_a_wait_command_of_another_format_is_refused_at_connect() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_GRIP,
        formats={"ace2k_feed_lane_wait": "ace2k_feed_lane_wait lane=%hu"},
    )
    with pytest.raises(Exception, match="ace2k_feed_lane_wait has another format"):
        mcu.config_callback()


def test_an_image_without_the_grip_has_no_wait() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    assert not feed_of(module).has_wait


def test_the_park_is_cleared_by_the_connects_grip_clear() -> None:
    # one slot on the unit: the init command that clears every grip clears a park too
    feed, printer, mcu = make_grip()
    grips = [c for c, _ in mcu.config_cmds if c.startswith("ace2k_feed_lane_grip")]
    assert grips == [f"ace2k_feed_lane_grip lane={lane} grip_um=0" for lane in range(4)]
    assert feed.has_wait  # the wait shares the slot, so these clears clear it too


def test_set_lane_grip_on_a_firmware_without_the_feed_is_a_runtime_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed_of(module).set_lane_grip(0, 40.0)


def test_ace_grip_sets_the_given_or_the_default_grip_and_mm_zero_clears() -> None:
    feed, printer, mcu = make_grip()
    gcmd = FakeGcmd({"LANE": "2", "MM": "30"})
    feed.cmd_ACEK_GRIP(gcmd)
    assert mcu.commands[-1] == (GRIP_FMT, [1, 30000])
    assert gcmd.lines == [
        "ace2k: lane 2 grip 30 mm: its next automatic load pulls only that, without the tag search"
    ]
    feed.cmd_ACEK_GRIP(FakeGcmd({"LANE": "1"}))  # the firmware's default
    assert mcu.commands[-1] == (GRIP_FMT, [0, 40000])
    gcmd = FakeGcmd({"LANE": "1", "MM": "0"})
    feed.cmd_ACEK_GRIP(gcmd)
    assert mcu.commands[-1] == (GRIP_FMT, [0, 0])
    assert gcmd.lines == ["ace2k: lane 1 grip cleared"]


def test_ace_grip_out_of_bounds_is_a_gcode_error_and_sends_nothing() -> None:
    feed, printer, mcu = make_grip()
    for mm in ("5", "46", "50"):
        with pytest.raises(Exception, match=r"outside the unit's 10\.\.45 mm"):
            feed.cmd_ACEK_GRIP(FakeGcmd({"LANE": "1", "MM": mm}))
    with pytest.raises(Exception, match="MM of at least 0"):
        feed.cmd_ACEK_GRIP(FakeGcmd({"LANE": "1", "MM": "-1"}))
    with pytest.raises(Exception, match="LANE of at most 4"):
        feed.cmd_ACEK_GRIP(FakeGcmd({"LANE": "5", "MM": "40"}))
    assert mcu.commands == []


def test_ace_grip_is_registered() -> None:
    assert "ACEK_GRIP" in ace2k_feed.GCODES
    feed, printer, mcu = make_grip()
    # registered once with Klipper by the coordinator, which hands it to the unit's feed
    assert "ACEK_GRIP" in printer.objects["gcode"].commands
    assert printer.objects["ace2k_units"].commands["ACEK_GRIP"] == {feed.unit: feed.cmd_ACEK_GRIP}


def test_lane_unavailable_names_the_first_reason_in_order() -> None:
    module, _printer, _mcu = make_feed([])
    feed = feed_of(module)
    module.lanes[0]["insert"] = False
    assert feed.lane_unavailable(0) == "no_filament"
    module.lanes[0]["insert"] = True
    feed.lanes[0]["mode"] = "idle"
    assert feed.lane_unavailable(0) is None
    feed.lanes[0]["mode"] = "error"
    assert feed.lane_unavailable(0) == "error"
    assert feed.lane_unavailable(0, None) == "error"
    assert feed.lane_unavailable(0, ("idle", "error")) is None
    feed.lanes[0]["mode"] = "feeding"
    assert feed.lane_unavailable(0) == "busy"
    assert feed.lane_unavailable(0, None) is None  # allow None leaves the mode alone
    feed.lanes[0]["mode"] = "following"  # a follow whose motion ended counts as idle
    assert not feed.in_follow(0)
    assert feed.lane_unavailable(0) is None


def test_a_parking_point_past_the_longest_move_is_a_config_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), options={"load_park_mm": 100000})
    with pytest.raises(Exception, match="load_park_mm must be at most"):
        mcu.config_callback()


def test_the_reverse_with_a_counters_frame_from_another_commit_is_refused() -> None:
    old = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s"  # no back
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=dict(WITH_FF, ACE2K_FEED_FF_REVERSE=1),
        formats={"ace2k_feed_ff_state": old},
    )
    with pytest.raises(Exception, match="ace2k_feed_ff_state"):
        mcu.config_callback()


def test_the_park_without_the_feed_is_a_runtime_error() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES))
    feed = feed_of(module)  # never connected: no feed
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.set_lane_park(0)


def test_a_calibration_feed_that_moved_no_filament_scales_nothing() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 0, seq=1))
    feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    with pytest.raises(Exception, match="moved no filament"):
        feed.cmd_ACEK_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "197"}))
    assert printer.objects["configfile"].calls == []


def test_feed_forward_on_and_off_together_is_refused() -> None:
    feed, printer, mcu, _module = make_ff_feed()
    with pytest.raises(Exception, match="ON=1 or OFF=1, not both"):
        feed.cmd_ACEK_FEED_FORWARD(FakeGcmd({"ON": "1", "OFF": "1"}))


# --- the stop armed on the plunger (held, at_rest; API_VERSION 20) ----------------------------

# The held stop's values with their bounds, as an image since v0.13.0 publishes them (feed_cmds.c).
HELD_CONSTANTS = {
    "ACE2K_FEED_HELD_UM": 3000,
    "ACE2K_FEED_HELD_UM_MIN": 1500,
    "ACE2K_FEED_HELD_UM_MAX": 20000,
    "ACE2K_FEED_HELD_MS": 550,
    "ACE2K_FEED_HELD_MS_MIN": 50,
    "ACE2K_FEED_HELD_MS_MAX": 2000,
}
WITH_HELD = dict(DEFAULT_CONSTANTS, **HELD_CONSTANTS)
STOP_WHEN = "ace2k_feed_stop_when"
NO_STOP = "ace2k: this image cannot stop on the plunger"
# An image whose dictionary lacks the stop's command, its answer and the held values' command.
WITHOUT_STOP = {
    name: f"Unknown command: {name}"
    for name in ("ace2k_feed_stop_when", "ace2k_feed_stop_when_response", "ace2k_feed_held_set")
}


def stop_feed(
    constants: dict[str, float] = WITH_HELD,
    lookup_errors: dict[str, str | BaseException] | None = None,
    options: dict[str, Any] | None = None,
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu, Responses]:
    """A connected feed whose starts and armings are accepted."""
    responses = dict(RESPONSES)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    responses[STOP_WHEN] = {"lane": 0, "accepted": 1, "reason": 0}
    module, printer, mcu = make_feed(
        responses, constants=constants, lookup_errors=lookup_errors, options=options
    )
    mcu.config_callback()
    return feed_of(module), printer, mcu, responses


def test_held_and_at_rest_are_final_kinds_that_are_no_error() -> None:
    assert ace2k_feed.KIND_NAMES[19] == "held" and ace2k_feed.KIND_NAMES[20] == "at_rest"
    for kind in (19, 20):
        assert kind not in ace2k_feed.ERROR_KINDS and kind not in ace2k_feed.NOTICE_KINDS


def test_the_stop_kinds_mirror_the_firmware_header() -> None:
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    kinds = {int(v): n.lower() for n, v in re.findall(r"ACE2K_FEED_(HELD|AT_REST) = (\d+)", header)}
    assert kinds == {19: "held", 20: "at_rest"}
    assert {k: ace2k_feed.KIND_NAMES[k] for k in kinds} == kinds


def test_the_stop_formats_and_the_held_constants_are_the_firmwares() -> None:
    src = pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed"
    cmds = (src / "feed_cmds.c").read_text()
    for fmt in (proto.STOP_WHEN_FMT, proto.STOP_WHEN_RESP, proto.HELD_SET_FMT):
        assert f'"{fmt}"' in cmds, fmt
    published = dict(
        re.findall(r'^DECL_CONSTANT\("(ACE2K_FEED_HELD_\w+)", (ACE2K_FEED_HELD_\w+)\);', cmds, re.M)
    )
    header = (src / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(r"^#define\s+(ACE2K_FEED_HELD_\w+)\s+(\d+)U\b", header, re.M)
    }
    # the fake carries the firmware's own values, under the names the host reads
    assert {name: defines[define] for name, define in published.items()} == HELD_CONSTANTS


def test_start_move_stop_when_arms_the_stop_then_sends_the_start() -> None:
    feed, printer, mcu, responses = stop_feed()
    assert feed.has_stop_when is True
    before = len(mcu.queries)
    move = feed.start_move(0, "rollback", 80, 30, stop_when={"held"})
    assert mcu.queries[before:] == [
        (STOP_WHEN, [0, 1, 0, 0]),
        ("ace2k_feed_start", [0, 1, 80000, 30000, 1, 0]),
    ]
    assert not move.done()
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 19, 2, 12000, 9000, seq=move.seq))
    assert move.done() and nothing_tracked(feed)
    assert move.result()["kind"] == "held" and move.result()["mode"] == "rolling_back"
    mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)
    assert feed.lanes[0]["error"] is None
    # at rest, in either direction, and both together
    before = len(mcu.queries)
    move = feed.start_move(0, "feed", 50.0, 30.0, stop_when=frozenset({"at_rest"}))
    assert mcu.queries[before:] == [
        (STOP_WHEN, [0, 0, 1, 0]),
        ("ace2k_feed_start", [0, 0, 50000, 30000, 2, 0]),
    ]
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 20, 1, 500, 400, seq=move.seq))
    assert move.done() and move.result()["kind"] == "at_rest"
    before = len(mcu.queries)
    feed.start_move(0, "unload", 0.0, 30.0, stop_when=("held", "at_rest"))
    assert mcu.queries[before:][0] == (STOP_WHEN, [0, 1, 1, 0])


def test_start_move_held_ms_goes_with_the_arming() -> None:
    feed, printer, mcu, responses = stop_feed()
    before = len(mcu.queries)
    feed.start_move(0, "rollback", 100.0, 50.0, stop_when={"held"}, held_ms=120)
    feed.start_move(0, "rollback", 40.0, 10.0, stop_when={"held", "at_rest"}, held_ms=600)
    feed.start_move(0, "rollback", 80.0, 10.0, stop_when={"held"}, held_ms=None)
    assert [q for q in mcu.queries[before:] if q[0] == STOP_WHEN] == [
        (STOP_WHEN, [0, 1, 0, 120]),
        (STOP_WHEN, [0, 1, 1, 600]),
        (STOP_WHEN, [0, 1, 0, 0]),  # None: the unit's own held_ms
    ]


def test_the_held_window_limits_are_read_at_connect() -> None:
    feed, printer, mcu, responses = stop_feed()
    assert (feed.held_ms_min, feed.held_ms_max) == (50, 2000)
    # three counts of 1 481 µm at the widest scale restart a 3 mm window
    assert feed.held_restart_mm == pytest.approx(4.443, abs=0.002)
    old, printer, mcu, responses = stop_feed(DEFAULT_CONSTANTS, WITHOUT_STOP)
    assert (old.held_ms_min, old.held_ms_max, old.held_restart_mm) == (None, None, None)
    wide, printer, mcu, responses = stop_feed(options={"held_mm": 1.5})
    assert wide.held_restart_mm == pytest.approx(2.962, abs=0.002)


@pytest.mark.parametrize(
    ("kinds", "held_ms", "why"),
    [
        ({"held"}, 49, "held_ms 49: the unit takes 50 to 2000"),
        ({"held"}, 2001, "held_ms 2001: the unit takes 50 to 2000"),
        ({"held"}, 0, "held_ms 0: the unit takes 50 to 2000"),
        ({"held"}, True, "held_ms True: a whole number of milliseconds"),
        ({"held"}, 120.5, "held_ms 120.5: a whole number of milliseconds"),
        ({"held"}, "120", "held_ms '120': a whole number of milliseconds"),
        ({"at_rest"}, 120, "held_ms 120 without held: the window is held's own"),
        ((), 120, "held_ms 120 without held: the window is held's own"),
    ],
)
def test_a_held_ms_the_unit_would_refuse_raises_with_nothing_sent(
    kinds: set[str], held_ms: object, why: str
) -> None:
    feed, printer, mcu, responses = stop_feed()
    sent = len(mcu.sent)
    with pytest.raises(ValueError, match=f"^ace2k: {re.escape(why)}$"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when=kinds, held_ms=held_ms)  # type: ignore[arg-type]
    assert mcu.sent[sent:] == [] and nothing_tracked(feed)


@pytest.mark.parametrize(
    ("speed", "held_ms", "refused"),
    [
        # the floor is 4 443 µm (three counts at the widest scale) over held_ms
        (50.0, 120, False),  # 37.0 mm/s, 74 % of the speed
        (50.0, 94, False),  # 47.3 mm/s, under 95 %
        (50.0, 93, True),  # 47.8 mm/s, over 95 %
        (50.0, 60, True),  # 74.1 mm/s: friction would always end the move held
        (10.0, 600, False),  # 7.4 mm/s
        (10.0, 468, False),  # 9.49 mm/s
        (10.0, 467, True),  # 9.51 mm/s
    ],
)
def test_a_window_whose_friction_floor_passes_95_percent_of_the_speed_is_refused(
    speed: float, held_ms: int, refused: bool
) -> None:
    feed, printer, mcu, responses = stop_feed()
    sent = len(mcu.sent)
    if refused:
        with pytest.raises(ValueError, match=r"friction can end it held below .* mm/s"):
            feed.start_move(0, "rollback", 80.0, speed, stop_when={"held"}, held_ms=held_ms)
        assert mcu.sent[sent:] == [] and nothing_tracked(feed)
    else:
        feed.start_move(0, "rollback", 80.0, speed, stop_when={"held"}, held_ms=held_ms)
        assert mcu.sent[sent:] == [STOP_WHEN, "ace2k_feed_start"]


def test_the_friction_floor_refusal_names_the_move() -> None:
    feed, printer, mcu, responses = stop_feed()
    with pytest.raises(ValueError) as err:
        feed.start_move(0, "rollback", 80.0, 50.0, stop_when={"held"}, held_ms=60)
    assert str(err.value) == (
        "ace2k: held_ms 60 at 50 mm/s: friction can end it held below 74.0 mm/s, more than 95 %"
        " of its speed; raise held_ms"
    )


def test_a_window_the_unit_refuses_is_feed_refused_bounds() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = {"lane": 0, "accepted": 0, "reason": 5}
    sent = len(mcu.sent)
    with pytest.raises(ace2k_feed.FeedRefused) as err:
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"}, held_ms=600)
    assert err.value.reason == "bounds"
    assert mcu.sent[sent:] == [STOP_WHEN] and nothing_tracked(feed)


def test_a_waited_gcode_move_ending_held_is_no_gcode_error() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 19, 2, 12000, 9000, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "80"})
    feed.cmd_ACEK_ROLLBACK(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 rollback held — motor 12.0 mm, filament 9.0 mm"
    assert STOP_WHEN not in mcu.sent  # a G-code never arms a stop


@pytest.mark.parametrize(
    ("constants", "lookup_errors"),
    [
        (WITH_HELD, None),  # an image with the stop
        (DEFAULT_CONSTANTS, None),  # the commands without the constants
        (DEFAULT_CONSTANTS, WITHOUT_STOP),  # an image before v0.13.0
    ],
)
def test_start_move_without_stop_when_sends_exactly_what_it_sent_before(
    constants: dict[str, float], lookup_errors: dict[str, str] | None
) -> None:
    feed, printer, mcu, responses = stop_feed(constants, lookup_errors)
    responses["ace2k_feed_start"] = accepted(mcu)  # each answer echoes its lane
    sent, queries, commands = len(mcu.sent), len(mcu.queries), len(mcu.commands)
    feed.start_move(0, "rollback", 80.0, 30.0)
    feed.start_move(1, "feed", 50.0, 30.0, free_tip=True)
    feed.start_move(2, "unload", 0.0, 30.0, stop_when=frozenset())
    # the frames of a start before API 20: the start alone, with the same arguments
    assert mcu.sent[sent:] == ["ace2k_feed_start"] * 3
    assert mcu.queries[queries:] == [
        ("ace2k_feed_start", [0, 1, 80000, 30000, 1, 0]),
        ("ace2k_feed_start", [1, 0, 50000, 30000, 1, 1]),
        ("ace2k_feed_start", [2, 4, 0, 30000, 1, 0]),
    ]
    assert mcu.commands[commands:] == []


@pytest.mark.parametrize(
    ("constants", "lookup_errors"),
    [
        (DEFAULT_CONSTANTS, None),  # the commands without the constants
        (DEFAULT_CONSTANTS, WITHOUT_STOP),  # an image before v0.13.0
    ],
)
def test_an_image_without_the_stop_refuses_stop_when_and_sends_nothing(
    constants: dict[str, float], lookup_errors: dict[str, str] | None
) -> None:
    feed, printer, mcu, responses = stop_feed(constants, lookup_errors)
    assert feed.has_stop_when is False
    sent = len(mcu.sent)
    for kinds in ({"held"}, {"at_rest"}, {"held", "at_rest"}):
        with pytest.raises(RuntimeError, match=f"^{re.escape(NO_STOP)}$"):
            feed.start_move(0, "rollback", 80.0, 30.0, stop_when=kinds)
    assert mcu.sent[sent:] == [] and nothing_tracked(feed)
    assert not any(c.startswith("ace2k_feed_held_set") for c, _ in mcu.config_cmds)
    feed.start_move(0, "rollback", 80.0, 30.0)  # without stop_when, as before
    assert mcu.sent[sent:] == ["ace2k_feed_start"]


def test_the_stop_is_off_until_the_connect_finds_it() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_HELD)
    assert feed_of(module).has_stop_when is False
    mcu.config_callback()
    assert feed_of(module).has_stop_when is True


def test_a_stop_command_with_another_format_is_refused_at_connect() -> None:
    for name, other in (
        ("ace2k_feed_stop_when", "ace2k_feed_stop_when lane=%c held=%c"),
        # the arming before it carried its own window
        ("ace2k_feed_stop_when", "ace2k_feed_stop_when lane=%c held=%c at_rest=%c"),
        ("ace2k_feed_stop_when_response", "ace2k_feed_stop_when_response lane=%c accepted=%c"),
        ("ace2k_feed_held_set", "ace2k_feed_held_set held_um=%u held_ms=%u"),
    ):
        module, printer, mcu = make_feed(
            dict(RESPONSES), constants=WITH_HELD, formats={name: other}
        )
        with pytest.raises(Exception, match=name):
            mcu.config_callback()


@pytest.mark.parametrize("mode", ["feed", "assist", "load", "assist_both", "assist_back"])
def test_held_on_any_mode_but_rollback_and_unload_is_refused_with_nothing_sent(
    mode: str,
) -> None:
    # a load and the follow move toward the spool only in parts that the unit chooses
    feed, printer, mcu, responses = stop_feed(dict(WITH_HELD, **FOLLOW_CONSTANTS))
    sent = len(mcu.sent)
    with pytest.raises(
        ValueError,
        match=f"^ace2k: stop_when held on {re.escape(mode)}: held is taken only on rollback and"
        " unload$",
    ):
        feed.start_move(0, mode, 50.0, 30.0, stop_when={"held", "at_rest"})
    assert mcu.sent[sent:] == [] and nothing_tracked(feed)


def test_held_on_the_reverse_assist_is_refused_with_nothing_sent() -> None:
    # The reverse assist ends its take-up as soon as the plunger reads taut, and then waits with
    # no move running. The unit judges held only while a move toward the spool runs, so held
    # could never end it.
    feed, printer, mcu, responses = stop_feed()
    sent = len(mcu.sent)
    with pytest.raises(
        ValueError,
        match="^ace2k: stop_when held on assist_back: held is taken only on rollback and unload$",
    ):
        feed.start_move(0, "assist_back", 0.0, 30.0, stop_when={"held"}, held_ms=600)
    assert mcu.sent[sent:] == [] and nothing_tracked(feed)
    assert not any(q[0] == STOP_WHEN for q in mcu.queries)


@pytest.mark.parametrize("mode", ["rollback", "unload"])
def test_held_is_taken_on_the_two_modes_toward_the_spool(mode: str) -> None:
    feed, printer, mcu, responses = stop_feed()
    feed.start_move(0, mode, 0.0, 30.0, stop_when={"held"})
    assert mcu.queries[-2] == (STOP_WHEN, [0, 1, 0, 0])


def test_at_rest_on_the_follow_is_refused_with_nothing_sent() -> None:
    feed, printer, mcu, responses = stop_feed(dict(WITH_HELD, **FOLLOW_CONSTANTS))
    sent = len(mcu.sent)
    with pytest.raises(
        ValueError,
        match="^ace2k: stop_when at_rest on assist_both: the follow keeps the plunger at rest, so"
        " at_rest would end it at once$",
    ):
        feed.start_move(0, "assist_both", 0.0, 30.0, stop_when={"at_rest"})
    assert mcu.sent[sent:] == [] and nothing_tracked(feed)


@pytest.mark.parametrize("mode", ["feed", "rollback", "assist", "assist_back", "unload", "load"])
def test_at_rest_is_taken_on_every_mode_but_the_follow(mode: str) -> None:
    feed, printer, mcu, responses = stop_feed()
    feed.start_move(0, mode, 0.0, 30.0, stop_when={"at_rest"})
    assert mcu.queries[-2] == (STOP_WHEN, [0, 0, 1, 0])


@pytest.mark.parametrize(
    "stop_when", [{"held", "tip"}, {"stuck"}, "held", {1}, 5, [["held"]], [{"at_rest"}]]
)
def test_an_unknown_stop_kind_is_a_value_error_with_nothing_sent(stop_when: object) -> None:
    for constants in (WITH_HELD, DEFAULT_CONSTANTS):  # judged before the image's support
        feed, printer, mcu, responses = stop_feed(constants)
        sent = len(mcu.sent)
        with pytest.raises(
            ValueError, match=r"^ace2k: stop_when .*: the kinds are held and at_rest"
        ):
            feed.start_move(0, "rollback", 80.0, 30.0, stop_when=stop_when)
        assert mcu.sent[sent:] == []


@pytest.mark.parametrize(("reason", "name"), [(1, "busy"), (2, "in_error")])
def test_a_refused_arming_raises_feed_refused_and_sends_no_start(reason: int, name: str) -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = {"lane": 0, "accepted": 0, "reason": reason}
    sent = len(mcu.sent)
    with pytest.raises(ace2k_feed.FeedRefused) as caught:
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    assert (caught.value.lane, caught.value.mode, caught.value.reason) == (0, "rollback", name)
    assert str(caught.value) == f"ace2k: lane 1 rollback refused: {name}"
    assert mcu.sent[sent:] == [STOP_WHEN] and nothing_tracked(feed)
    assert feed.motion[0].commanded_mm == {} and feed.motion[0].inflight is False
    # nothing was armed, so nothing is disarmed; the next start takes the next sequence
    responses[STOP_WHEN] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    assert move.seq == 1


def test_an_arming_with_no_answer_is_disarmed_and_no_start_is_sent() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = [
        FakeCommandError("Unable to obtain 'ace2k_feed_stop_when_response' response"),
        {"lane": 0, "accepted": 1, "reason": 0},
    ]
    queries = len(mcu.queries)
    with pytest.raises(
        RuntimeError, match="^ace2k: lane 1 rollback stop: no answer from the unit; check the link"
    ):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    # the unit may have armed it: held=0 at_rest=0 disarms it, and no start goes out
    assert mcu.queries[queries:] == [(STOP_WHEN, [0, 1, 0, 0]), (STOP_WHEN, [0, 0, 0, 0])]
    assert stops_sent(mcu) == [] and nothing_tracked(feed)


def test_an_arming_answered_for_another_lane_is_disarmed_and_no_start_is_sent() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = [
        {"lane": 2, "accepted": 1, "reason": 0},
        {"lane": 1, "accepted": 1, "reason": 0},
    ]
    queries = len(mcu.queries)
    with pytest.raises(
        RuntimeError, match="^ace2k: lane 2 unload stop: the unit answered for lane 3$"
    ):
        feed.start_move(1, "unload", 0.0, 30.0, stop_when={"at_rest"})
    assert mcu.queries[queries:] == [(STOP_WHEN, [1, 0, 1, 0]), (STOP_WHEN, [1, 0, 0, 0])]


def test_a_start_with_no_answer_after_its_arming_stops_and_disarms_the_lane() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses["ace2k_feed_start"] = FakeCommandError(
        "Unable to obtain 'ace2k_feed_start_response' response"
    )
    queries = len(mcu.queries)
    with pytest.raises(RuntimeError, match="lane 1 rollback: no answer .*lane 1 stopped in case"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    assert [q for q in mcu.queries[queries:] if q[0] == STOP_WHEN] == [
        (STOP_WHEN, [0, 1, 0, 0]),
        (STOP_WHEN, [0, 0, 0, 0]),
    ]
    assert stops_sent(mcu) == [[0]]
    # the stop first: on a failing link it never waits behind the disarm's timeout
    tail = mcu.sent[-2:]
    assert tail == ["ace2k_feed_stop", STOP_WHEN]
    # a start without stop_when that gets no answer disarms nothing, as before
    queries = len(mcu.queries)
    with pytest.raises(RuntimeError, match="no answer"):
        feed.start_move(0, "rollback", 80.0, 30.0)
    assert [q[0] for q in mcu.queries[queries:]] == ["ace2k_feed_start"]


def test_a_disarm_with_no_answer_is_logged_and_the_first_error_raised(
    caplog: pytest.LogCaptureFixture,
) -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = [
        {"lane": 0, "accepted": 1, "reason": 0},
        FakeCommandError("Unable to obtain 'ace2k_feed_stop_when_response' response"),
    ]
    responses["ace2k_feed_start"] = {"lane": 3, "accepted": 1, "reason": 0}
    caplog.set_level("WARNING", logger="root")
    with pytest.raises(RuntimeError, match="lane 1 rollback: the unit answered for lane 4"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    lines = [r.getMessage() for r in caplog.records if r.levelname == "WARNING"]
    assert len(lines) == 1
    assert lines[0].startswith("ace2k: lane 1: the stop may still be armed for the next start (")
    assert "no answer from the unit" in lines[0]
    # a disarm answered for another lane is logged the same way
    caplog.clear()
    responses[STOP_WHEN] = [
        {"lane": 0, "accepted": 1, "reason": 0},
        {"lane": 2, "accepted": 1, "reason": 0},
    ]
    with pytest.raises(RuntimeError, match="the unit answered for lane 4"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    lines = [r.getMessage() for r in caplog.records if r.levelname == "WARNING"]
    assert lines == [
        "ace2k: lane 1: the stop may still be armed for the next start (ace2k: lane 1 stop disarm:"
        " the unit answered for lane 3)"
    ]


@pytest.mark.parametrize(
    "disarm",
    [
        {"lane": 0, "accepted": 0, "reason": 1},  # busy: most likely the start took the stop
        FakeCommandError("Unable to obtain 'ace2k_feed_stop_when_response' response"),
        {"lane": 2, "accepted": 1, "reason": 0},  # answered for another lane
    ],
)
def test_a_disarm_not_accepted_makes_the_lanes_next_start_set_its_own_stop(
    disarm: object,
) -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = [{"lane": 0, "accepted": 1, "reason": 0}, disarm]
    responses["ace2k_feed_start"] = FakeCommandError("Unable to obtain a response")
    with pytest.raises(RuntimeError, match="no answer"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    assert [q for q in mcu.queries if q[0] == STOP_WHEN][-1] == (STOP_WHEN, [0, 0, 0, 0])
    responses[STOP_WHEN] = {"lane": 0, "accepted": 1, "reason": 0}
    responses["ace2k_feed_start"] = accepted(mcu)
    # the next start on the lane, even without stop_when, first clears what may be left
    queries = len(mcu.queries)
    move = feed.start_move(0, "rollback", 80.0, 30.0)
    assert mcu.queries[queries:] == [
        (STOP_WHEN, [0, 0, 0, 0]),
        ("ace2k_feed_start", [0, 1, 80000, 30000, move.seq, 0]),
    ]
    # the other lanes never had it
    queries = len(mcu.queries)
    other = feed.start_move(1, "rollback", 80.0, 30.0)
    assert mcu.queries[queries:] == [("ace2k_feed_start", [1, 1, 80000, 30000, other.seq, 0])]
    # once the unit accepted, the lane is back on the normal path
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 2, 80000, 80000, seq=move.seq))
    queries = len(mcu.queries)
    move = feed.start_move(0, "rollback", 80.0, 30.0)
    assert mcu.queries[queries:] == [("ace2k_feed_start", [0, 1, 80000, 30000, move.seq, 0])]


def test_a_lane_whose_stop_is_unknown_sends_its_own_flags_once_before_the_start() -> None:
    feed, printer, mcu, responses = stop_feed()
    responses[STOP_WHEN] = [
        {"lane": 0, "accepted": 1, "reason": 0},
        {"lane": 0, "accepted": 0, "reason": 1},
    ]
    responses["ace2k_feed_start"] = FakeCommandError("Unable to obtain a response")
    with pytest.raises(RuntimeError, match="no answer"):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    responses["ace2k_feed_start"] = accepted(mcu)
    # refused: no start goes out, and the lane stays unknown
    responses[STOP_WHEN] = {"lane": 0, "accepted": 0, "reason": 1}
    queries = len(mcu.queries)
    with pytest.raises(ace2k_feed.FeedRefused, match="^ace2k: lane 1 unload refused: busy$"):
        feed.start_move(0, "unload", 0.0, 30.0)
    assert mcu.queries[queries:] == [(STOP_WHEN, [0, 0, 0, 0])]
    # a start with its own stop sends that stop, once, and not 0 0 first
    responses[STOP_WHEN] = {"lane": 0, "accepted": 1, "reason": 0}
    queries = len(mcu.queries)
    move = feed.start_move(0, "unload", 0.0, 30.0, stop_when={"at_rest"})
    assert mcu.queries[queries:] == [
        (STOP_WHEN, [0, 0, 1, 0]),
        ("ace2k_feed_start", [0, 4, 0, 30000, move.seq, 0]),
    ]
    assert feed.motion[0].stop_unknown is False


def test_a_start_whose_numbers_cannot_be_sent_has_no_sequence_and_arms_nothing() -> None:
    for length, speed in ((float("nan"), 30.0), (float("inf"), 30.0), (80.0, float("nan"))):
        for kinds in (frozenset({"held"}), frozenset()):
            feed, printer, mcu, responses = stop_feed()
            sent = len(mcu.sent)
            with pytest.raises((ValueError, OverflowError)):
                feed.start_move(0, "rollback", length, speed, stop_when=kinds)
            assert mcu.sent[sent:] == [] and nothing_tracked(feed)
            assert feed.motion[0].commanded_mm == {} and feed.motion[0].stop_unknown is False
            assert feed.start_move(0, "rollback", 80.0, 30.0).seq == 1
    feed, printer, mcu, responses = stop_feed()
    with pytest.raises(OverflowError):
        feed.start_move(0, "rollback", 80.0, float("-inf"), stop_when={"at_rest"})
    assert mcu.sent == []


def test_an_error_between_the_arming_and_the_start_disarms_the_lane() -> None:
    feed, printer, mcu, responses = stop_feed()

    def boom(seq: int) -> NoReturn:
        raise RuntimeError(f"boom {seq}")

    start = ace2k_feed.Start("rollback", 80.0, 30.0, stop_when=frozenset({"held"}))
    queries = len(mcu.queries)
    with pytest.raises(RuntimeError, match="^boom 1$"):
        feed._send_start(0, start, None, boom)
    assert mcu.queries[queries:] == [(STOP_WHEN, [0, 1, 0, 0]), (STOP_WHEN, [0, 0, 0, 0])]
    assert feed.motion[0].inflight is False


def test_a_refused_start_after_its_arming_disarms_nothing() -> None:
    # the unit drops the arming with a refused start
    feed, printer, mcu, responses = stop_feed()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 3}
    queries = len(mcu.queries)
    with pytest.raises(
        ace2k_feed.FeedRefused, match="^ace2k: lane 1 rollback refused: no_filament$"
    ):
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when={"held"})
    assert mcu.queries[queries:] == [
        (STOP_WHEN, [0, 1, 0, 0]),
        ("ace2k_feed_start", [0, 1, 80000, 30000, 1, 0]),
    ]
    assert feed.motion[0].stop_unknown is False


def test_stop_when_none_is_no_stop() -> None:
    for constants, lookup_errors in ((WITH_HELD, None), (DEFAULT_CONSTANTS, WITHOUT_STOP)):
        feed, printer, mcu, responses = stop_feed(constants, lookup_errors)
        queries = len(mcu.queries)
        feed.start_move(0, "rollback", 80.0, 30.0, stop_when=None)
        assert mcu.queries[queries:] == [("ace2k_feed_start", [0, 1, 80000, 30000, 1, 0])]


def test_the_constants_without_the_stops_commands_refuse_the_connect() -> None:
    for name in WITHOUT_STOP:
        module, printer, mcu = make_feed(
            dict(RESPONSES), constants=WITH_HELD, lookup_errors={name: f"Unknown command: {name}"}
        )
        with pytest.raises(Exception, match=name):
            mcu.config_callback()
    module, printer, mcu = make_feed(
        dict(RESPONSES), constants=WITH_HELD, lookup_errors=WITHOUT_STOP
    )
    with pytest.raises(Exception, match="ace2k_feed_stop_when"):
        mcu.config_callback()


def test_the_held_values_go_out_as_an_init_command_at_every_connect() -> None:
    feed, printer, mcu, responses = stop_feed()
    held = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_held_set")]
    assert held == [("ace2k_feed_held_set held_um=3000 held_ms=550", True)]
    feed, printer, mcu, responses = stop_feed(options={"held_mm": 4.5004, "held_ms": 700})
    held = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_held_set")]
    assert held == [("ace2k_feed_held_set held_um=4500 held_ms=700", True)]
    assert printer.objects["configfile"].calls == []  # never staged for SAVE_CONFIG


def test_a_held_key_out_of_the_units_bounds_is_a_config_error() -> None:
    for key, value, text in (
        ("held_mm", 1.4996, r"held_mm \(1.5\)"),  # rounded to the µm, then judged: taken
        ("held_mm", 1.499, r"held_mm \(1.499\) must be within 1.5..20"),
        ("held_mm", 20.001, r"held_mm \(20.001\) must be within 1.5..20"),
        ("held_mm", float("nan"), r"held_mm \(nan\) must be a finite number"),
        ("held_ms", 49, r"held_ms \(49\) must be within 50..2000"),
        ("held_ms", 2001, r"held_ms \(2001\) must be within 50..2000"),
    ):
        responses = dict(RESPONSES)
        module, printer, mcu = make_feed(responses, constants=WITH_HELD, options={key: value})
        if value == 1.4996:
            mcu.config_callback()
            assert any(
                c == "ace2k_feed_held_set held_um=1500 held_ms=550" for c, _ in mcu.config_cmds
            )
            continue
        with pytest.raises(Exception, match=text):
            mcu.config_callback()
    # held_ms is an integer key: the config's getint refuses a fraction when the key is read
    with pytest.raises(ValueError, match=r"550\.5"):
        make_feed(dict(RESPONSES), constants=WITH_HELD, options={"held_ms": "550.5"})
    # the bounds are the unit's: a wider published bound takes a value this one would refuse
    constants = dict(WITH_HELD, ACE2K_FEED_HELD_UM_MAX=25000)
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants, options={"held_mm": 22})
    mcu.config_callback()
    assert any(c.startswith("ace2k_feed_held_set held_um=22000 ") for c, _ in mcu.config_cmds)


def test_held_keys_on_an_image_without_the_stop_are_ignored_with_one_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level("WARNING", logger="root"):
        feed, printer, mcu, responses = stop_feed(
            DEFAULT_CONSTANTS, WITHOUT_STOP, options={"held_mm": 4, "held_ms": 99999}
        )
    assert [r.getMessage() for r in caplog.records if r.levelname == "WARNING"] == [
        "ace2k: held_mm, held_ms ignored: this image cannot stop on the plunger; flash v0.13.0"
    ]
    assert not any(c.startswith("ace2k_feed_held_set") for c, _ in mcu.config_cmds)
    caplog.clear()
    with caplog.at_level("WARNING", logger="root"):
        stop_feed(DEFAULT_CONSTANTS, WITHOUT_STOP, options={"held_ms": 700})
        stop_feed(DEFAULT_CONSTANTS, WITHOUT_STOP)  # no key: nothing to say
    assert [r.getMessage() for r in caplog.records if r.levelname == "WARNING"] == [
        "ace2k: held_ms ignored: this image cannot stop on the plunger; flash v0.13.0"
    ]


def test_held_keys_that_let_friction_end_a_slow_move_held_get_one_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # the floor at the widest scale, 1481 µm a count: ceil(held_um / 1481) x 1481 / held_ms,
    # against 95 % of the lane's least speed, 9 mm/s
    caplog.set_level("WARNING", logger="root")
    for options, floor in (
        ({"held_mm": 4.5}, "10.8"),
        ({"held_ms": 519}, "8.6"),
        ({"held_mm": 20, "held_ms": 50}, "414.7"),
    ):
        caplog.clear()
        stop_feed(options=options)
        held_mm = options.get("held_mm", 3)
        held_ms = options.get("held_ms", 550)
        assert [r.getMessage() for r in caplog.records if r.levelname == "WARNING"] == [
            f"ace2k: with held_mm {held_mm} and held_ms {held_ms}, friction can end a move held"
            f" below {floor} mm/s, more than 95 % of the lane's least speed (9 mm/s); raise"
            " held_ms or lower held_mm"
        ]
    # the defaults and the two edges stay under it
    caplog.clear()
    for options in ({}, {"held_ms": 520}, {"held_mm": 4.443}):
        stop_feed(options=options)
    assert [r for r in caplog.records if r.levelname == "WARNING"] == []


# The clear of an unload with its bounds, as an image since v0.13.0 publishes them (feed_cmds.c).
CLEAR_CONSTANTS = {
    "ACE2K_FEED_UNLOAD_CLEAR_UM": 20000,
    "ACE2K_FEED_UNLOAD_CLEAR_UM_MIN": 0,
    "ACE2K_FEED_UNLOAD_CLEAR_UM_MAX": 100000,
}
WITH_CLEAR = dict(DEFAULT_CONSTANTS, **CLEAR_CONSTANTS)
CLEAR_SET = "ace2k_feed_unload_set"


def clear_cmds(mcu: FakeMcu) -> list[tuple[str, bool]]:
    return [(c, i) for c, i in mcu.config_cmds if c.startswith(CLEAR_SET)]


def test_the_clear_format_and_constants_are_the_firmwares() -> None:
    src = pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed"
    cmds = (src / "feed_cmds.c").read_text()
    assert f'"{proto.UNLOAD_SET_FMT}"' in cmds
    published = dict(
        re.findall(
            r'^DECL_CONSTANT\("(ACE2K_FEED_UNLOAD_CLEAR_\w+)", (ACE2K_FEED_UNLOAD_CLEAR_\w+)\);',
            cmds,
            re.M,
        )
    )
    header = (src / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_UNLOAD_(?:CLEAR|TAIL)_\w+)\s+(\d+)U\b", header, re.M
        )
    }
    # the largest clear is the tail's budget
    defines["ACE2K_FEED_UNLOAD_CLEAR_UM_MAX"] = defines["ACE2K_FEED_UNLOAD_TAIL_UM"]
    assert {name: defines[define] for name, define in published.items()} == CLEAR_CONSTANTS


def test_the_clear_goes_out_as_an_init_command_at_every_connect() -> None:
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_CLEAR)
    mcu.config_callback()
    assert clear_cmds(mcu) == [("ace2k_feed_unload_set clear_um=20000", True)]
    options = {"unload_clear_mm": 35.0004}
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_CLEAR, options=options)
    mcu.config_callback()
    assert clear_cmds(mcu) == [("ace2k_feed_unload_set clear_um=35000", True)]
    module, printer, mcu = make_feed(
        dict(RESPONSES), constants=WITH_CLEAR, options={"unload_clear_mm": 0}
    )
    mcu.config_callback()
    assert clear_cmds(mcu) == [("ace2k_feed_unload_set clear_um=0", True)]
    assert printer.objects["configfile"].calls == []  # never staged for SAVE_CONFIG


def test_a_clear_out_of_the_units_bounds_is_a_config_error() -> None:
    for value, text in (
        (-0.001, r"unload_clear_mm \(-0.001\) must be within 0..100"),
        (100.001, r"unload_clear_mm \(100.001\) must be within 0..100"),
        (float("nan"), r"unload_clear_mm \(nan\) must be a finite number"),
    ):
        module, printer, mcu = make_feed(
            dict(RESPONSES), constants=WITH_CLEAR, options={"unload_clear_mm": value}
        )
        with pytest.raises(Exception, match=text):
            mcu.config_callback()


def test_a_clear_key_on_an_image_without_it_is_ignored_with_one_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    for lookup_errors in (None, {CLEAR_SET: f"Unknown command: {CLEAR_SET}"}):
        caplog.clear()
        with caplog.at_level("WARNING", logger="root"):
            module, printer, mcu = make_feed(
                dict(RESPONSES),
                constants=DEFAULT_CONSTANTS,
                lookup_errors=lookup_errors,
                options={"unload_clear_mm": 30},
            )
            mcu.config_callback()
        assert [r.getMessage() for r in caplog.records if r.levelname == "WARNING"] == [
            "ace2k: unload_clear_mm ignored: this image does not clear an unload past the bay's"
            " sensor; flash v0.13.0"
        ]
        assert clear_cmds(mcu) == []
    caplog.clear()
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu = make_feed(dict(RESPONSES), constants=DEFAULT_CONSTANTS)
        mcu.config_callback()  # no key: nothing to say
    assert not [r for r in caplog.records if r.levelname == "WARNING"]


def test_a_clear_command_with_another_format_or_missing_beside_its_constants_is_refused() -> None:
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_CLEAR,
        formats={CLEAR_SET: "ace2k_feed_unload_set clear_um=%hu"},
    )
    with pytest.raises(Exception, match=CLEAR_SET):
        mcu.config_callback()
    module, printer, mcu = make_feed(
        dict(RESPONSES),
        constants=WITH_CLEAR,
        lookup_errors={CLEAR_SET: f"Unknown command: {CLEAR_SET}"},
    )
    with pytest.raises(Exception, match=CLEAR_SET):
        mcu.config_callback()


def test_the_wait_of_an_unload_counts_the_clear_as_a_move_of_its_own(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The clear turns past the bay's sensor, and may be a fresh move of its own. 950 mm at
    70 mm/s, the 100 mm tail and the 20 mm clear, each with its 10 mm landing at the 9 mm/s floor:
    1.5 × (940 / 70 + 10 / 9 + 90 / 70 + 10 / 9 + 10 / 70 + 10 / 9) + 5 = 32.29 s. A rollback runs
    no clear: 30.40 s, as before."""
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, constants=WITH_CLEAR)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    assert feed.move_wait_s(0, 950, 70, reverse=True) == pytest.approx(32.286, abs=0.005)
    unload = waited_budget_s(
        monkeypatch,
        feed,
        lambda: feed.cmd_ACEK_UNLOAD(FakeGcmd({"LANE": "1", "LENGTH": "950", "SPEED": "70"})),
    )
    assert unload == pytest.approx(32.286, abs=0.005)
    rollback = waited_budget_s(
        monkeypatch,
        feed,
        lambda: feed.cmd_ACEK_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "950", "SPEED": "70"})),
    )
    assert rollback == pytest.approx(30.40, abs=0.005)
    module, printer, mcu = make_feed(
        dict(RESPONSES), constants=WITH_CLEAR, options={"unload_clear_mm": 0}
    )
    mcu.config_callback()
    assert feed_of(module).move_wait_s(0, 950, 70, reverse=True) == pytest.approx(30.40, abs=0.005)


def test_the_feed_task_sends_its_events_before_its_state_report() -> None:
    # Bench 2026-11-12: a report due on the pass that ended a move went out before the move's final
    # event, so the host saw the lane idle without it and counted the start over. The task sends
    # the events first, and the report only once none is left behind it (feed_cmds.c).
    src = pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed_cmds.c"
    text = src.read_text()
    body = text[
        text.index("void ace2k_feed_task(void)") : text.index("DECL_TASK(ace2k_feed_task);")
    ]
    events = body.index("ace2k_feed_peek_event(")
    report = body.index("ace2k_report_try(&ace2k_feed_report_due, report)")
    assert events < report
    assert "ace2k_feed_has_events(" in body[events:report]  # nothing left behind the report
