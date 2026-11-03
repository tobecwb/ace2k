"""ace2k_dryer_mode.py: the mode G-code and its gate, the limits and the profile sent to the unit,
the state report, the events' names — against the fake printer of test_ace2k_extras."""

from __future__ import annotations

from typing import TYPE_CHECKING

import ace2k_dryer
import ace2k_feed_proto
import pytest
from test_ace2k_dryer import DRYER_RESPONSES, NO_DRYER, RELEASE, build, gcode, sent_to
from test_ace2k_extras import FakeCommandError, FakeGcmd, make_units

if TYPE_CHECKING:
    from typing import Any

    from ace2k import Ace2k
    from ace2k_dryer import Ace2kDryer
    from test_ace2k_extras import FakeMcu, FakePrinter

AUTO_NAMES = {
    "ace2k_dryer_auto_mode",
    "ace2k_dryer_auto_limits",
    "ace2k_dryer_auto_profile",
    "ace2k_dryer_auto_timing",
    "ace2k_dryer_auto_response",
    "ace2k_dryer_auto_query",
    "ace2k_dryer_auto_state",
    "ace2k_dryer_start_bound",
}
MODES = RELEASE | AUTO_NAMES
AUTO_OK = {"op": 0, "accepted": 1, "reason": 0}
RESPONSES = dict(
    DRYER_RESPONSES,
    ace2k_dryer_auto_mode=AUTO_OK,
    ace2k_dryer_auto_limits=dict(AUTO_OK, op=1),
    ace2k_dryer_auto_profile=dict(AUTO_OK, op=2),
    ace2k_dryer_auto_timing=dict(AUTO_OK, op=3),
    ace2k_dryer_start_bound={"op": 3, "accepted": 1, "reason": 0},
)
STATE = "ace2k_dryer_auto_state"
# An image with the print mode's own stop. MODES above is the image before it.
STOP_BOUND = "ace2k_dryer_stop_bound"
MODES_STOP_BOUND = MODES | {STOP_BOUND}
RESPONSES_STOP_BOUND = dict(RESPONSES, ace2k_dryer_stop_bound={"op": 4, "accepted": 1, "reason": 0})


def auto_frame(**over: int) -> dict[str, int]:
    frame = {
        "mode": 0,
        "limit": 40,
        "max": 4,
        "failures": 0,
        "flags": 0,
        "profile_c": 45,
        "profile_min": 360,
        "cold": 0,
        "own": 0,
        "bound": 0,
        "rh": 500,
        "above_s": 0,
    }
    frame.update(over)
    return frame


def modes(
    options: dict[str, Any] | None = None, responses: dict[str, Any] | None = None
) -> tuple[Ace2k, FakePrinter, FakeMcu, Ace2kDryer]:
    return build(known=MODES, responses=responses or RESPONSES, options=options)


def test_humidity_is_refused_without_unattended_drying() -> None:
    _module, printer, mcu, _dryer = modes()
    with pytest.raises(FakeGcmd.error, match=r"unattended_drying: True to allow it"):
        gcode(printer, "ACEK_DRY_MODE", MODE="humidity")
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == []


def test_humidity_is_sent_with_unattended_drying() -> None:
    _module, printer, mcu, _dryer = modes({"unattended_drying": True})
    gcmd = gcode(printer, "ACEK_DRY_MODE", MODE="humidity")
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[1]]
    assert gcmd.lines == ["ace2k: dryer mode humidity"]  # the unit's head


@pytest.mark.parametrize(("word", "code"), [("manual", 0), ("print", 2), (" Print ", 2)])
def test_manual_and_print_need_no_gate(word: str, code: int) -> None:
    _module, printer, mcu, _dryer = modes()
    gcode(printer, "ACEK_DRY_MODE", MODE=word)
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[code]]


@pytest.mark.parametrize("word", ["auto", ""])
def test_an_unknown_mode_is_refused_before_it_sends(word: str) -> None:
    _module, printer, mcu, _dryer = modes()
    with pytest.raises(FakeGcmd.error, match="manual, humidity or print"):
        gcode(printer, "ACEK_DRY_MODE", MODE=word)
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == []


def test_a_refused_mode_is_named() -> None:
    _module, printer, _mcu, _dryer = modes(
        responses=dict(RESPONSES, ace2k_dryer_auto_mode=dict(AUTO_OK, accepted=0, reason=1))
    )
    with pytest.raises(FakeGcmd.error, match="dryer mode print refused: range"):
        gcode(printer, "ACEK_DRY_MODE", MODE="print")


def test_the_interface_raises_for_a_bad_name_and_the_gate() -> None:
    module, _printer, mcu, _dryer = modes()
    with pytest.raises(ValueError, match="manual, humidity or print"):
        module.dryer_mode.set_mode("auto")
    with pytest.raises(RuntimeError, match="unattended_drying"):
        module.dryer_mode.set_mode("humidity")
    module.dryer_mode.set_mode("print")
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[2]]


def test_the_connect_sends_only_the_report_query() -> None:
    _module, _printer, mcu, _dryer = modes({"dry_humidity_limit": 35, "dry_max_failures": 6})
    init = [c for c, is_init in mcu.config_cmds if is_init and "_auto_" in c]
    assert init == ["ace2k_dryer_auto_query rest_ticks=120000000"]
    assert not any(
        c.startswith("ace2k_dryer_auto_") for c, is_init in mcu.config_cmds if not is_init
    )
    assert sent_to(mcu, "ace2k_dryer_auto_limits") == []
    assert STATE in mcu.subscriptions


def test_the_first_report_sends_the_limits_that_differ_and_the_timing() -> None:
    _module, _printer, mcu, _dryer = modes({"dry_humidity_limit": 35, "dry_max_failures": 6})
    mcu.subscriptions[STATE](auto_frame(limit=40, max=4))
    assert sent_to(mcu, "ace2k_dryer_auto_limits") == [[35, 6]]
    assert sent_to(mcu, "ace2k_dryer_auto_timing") == [[0, 0, 0]]  # always: 0 = the default
    names = [n for n, _ in mcu.queries if n.startswith("ace2k_dryer_auto_")]
    assert names[:2] == ["ace2k_dryer_auto_limits", "ace2k_dryer_auto_timing"]  # before the rest


def test_the_limits_the_unit_already_has_are_not_sent() -> None:
    _module, _printer, mcu, _dryer = modes()
    mcu.subscriptions[STATE](auto_frame(limit=40, max=4))
    assert sent_to(mcu, "ace2k_dryer_auto_limits") == []


def test_limits_that_still_differ_are_sent_again_after_five_seconds() -> None:
    module, _printer, mcu, _dryer = modes({"dry_humidity_limit": 35})
    mcu.subscriptions[STATE](auto_frame(limit=40))
    mcu.subscriptions[STATE](auto_frame(limit=40))  # within 5 s: not again
    assert sent_to(mcu, "ace2k_dryer_auto_limits") == [[35, 4]]
    module.dryer_mode._on_report(5.0)
    assert sent_to(mcu, "ace2k_dryer_auto_limits") == [[35, 4], [35, 4]]
    mcu.subscriptions[STATE](auto_frame(limit=35))
    module.dryer_mode._on_report(10.0)  # the unit has them now
    assert len(sent_to(mcu, "ace2k_dryer_auto_limits")) == 2


def test_refused_limits_are_logged_once(caplog: pytest.LogCaptureFixture) -> None:
    module, _printer, mcu, _dryer = modes(
        {"dry_humidity_limit": 35},
        responses=dict(
            RESPONSES, ace2k_dryer_auto_limits=dict(AUTO_OK, op=1, accepted=0, reason=1)
        ),
    )
    mcu.subscriptions[STATE](auto_frame())
    module.dryer_mode._on_report(5.0)
    assert caplog.text.count("dryer limits refused: range") == 1


def test_the_bench_options_send_the_timing() -> None:
    _module, _printer, mcu, _dryer = modes(
        {"bench_dry_hold_s": 60, "bench_dry_cold_window_s": 120, "bench_dry_link_s": 30}
    )
    mcu.subscriptions[STATE](auto_frame())
    assert sent_to(mcu, "ace2k_dryer_auto_timing") == [[60, 120, 30]]


def test_one_bench_option_sends_the_others_as_defaults() -> None:
    _module, _printer, mcu, _dryer = modes({"bench_dry_link_s": 30})
    mcu.subscriptions[STATE](auto_frame())
    assert sent_to(mcu, "ace2k_dryer_auto_timing") == [[0, 0, 30]]


def test_the_timing_is_sent_once_per_connect_and_again_after_a_failure() -> None:
    module, _printer, mcu, _dryer = modes(
        responses=dict(
            RESPONSES, ace2k_dryer_auto_timing=[FakeCommandError("timeout"), dict(AUTO_OK, op=3)]
        )
    )
    mcu.subscriptions[STATE](auto_frame())
    module.dryer_mode._on_report(5.0)  # the retry
    module.dryer_mode._on_report(10.0)  # sent: not again
    assert len(sent_to(mcu, "ace2k_dryer_auto_timing")) == 2
    mcu.config_callback()  # a reconnect: once more
    mcu.subscriptions[STATE](auto_frame())
    assert len(sent_to(mcu, "ace2k_dryer_auto_timing")) == 3


def test_the_bench_options_have_32_bit_milliseconds_as_their_bound() -> None:
    modes({"bench_dry_hold_s": 4294967})
    with pytest.raises(Exception, match="bench_dry_hold_s"):
        modes({"bench_dry_hold_s": 4294968})


def test_an_answer_for_another_request_is_no_answer() -> None:
    module, _printer, _mcu, _dryer = modes(
        responses=dict(RESPONSES, ace2k_dryer_auto_mode=dict(AUTO_OK, op=2))
    )
    with pytest.raises(RuntimeError, match="another request"):
        module.dryer_mode.set_mode("print")


def test_a_stored_humidity_mode_without_the_gate_is_set_back_to_manual(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu, _dryer = modes()
    mcu.subscriptions[STATE](auto_frame(mode=1))
    # the fake reactor runs an async callback at once (FakeReactor.register_async_callback)
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[0]]
    text = "the humidity mode is off in printer.cfg: unit set back to manual"
    assert text in caplog.text
    assert any(text in line for line in printer.objects["gcode"].output)  # and in the console
    mcu.subscriptions[STATE](auto_frame(mode=0))
    module.dryer_mode._on_report(5.0)  # manual now: nothing more
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[0]]


def test_a_failed_set_back_is_said_and_tried_again(caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu, _dryer = modes(
        responses=dict(RESPONSES, ace2k_dryer_auto_mode=[FakeCommandError("timeout"), AUTO_OK])
    )
    mcu.subscriptions[STATE](auto_frame(mode=1))
    assert "but the unit kept it" in caplog.text
    console = printer.objects["gcode"].output
    assert any("but the unit kept it" in line for line in console)
    mcu.subscriptions[STATE](auto_frame(mode=1))  # within 5 s: not yet
    assert len(sent_to(mcu, "ace2k_dryer_auto_mode")) == 1
    module.dryer_mode._on_report(5.0)
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[0], [0]]
    assert any("unit set back to manual" in line for line in console)


def test_a_reconnect_with_the_gate_off_sets_the_mode_back_again() -> None:
    _module, _printer, mcu, _dryer = modes()
    mcu.subscriptions[STATE](auto_frame(mode=1))
    mcu.config_callback()  # a reconnect; the unit holds humidity again (set by another host)
    mcu.subscriptions[STATE](auto_frame(mode=1))
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == [[0], [0]]


def test_a_reconnect_with_a_stopped_humidity_mode_and_the_gate_sends_no_mode() -> None:
    """The unit clears its "stopped" state on every humidity mode command: none is sent."""
    _module, _printer, mcu, _dryer = modes({"unattended_drying": True})
    mcu.subscriptions[STATE](auto_frame(mode=1, flags=1, failures=4))
    mcu.config_callback()  # a reconnect
    mcu.subscriptions[STATE](auto_frame(mode=1, flags=1, failures=4))
    assert sent_to(mcu, "ace2k_dryer_auto_mode") == []


@pytest.mark.parametrize(
    ("materials", "want"),
    [
        (["PLA", "PETG", None, None], (45, 360)),
        (["PETG", "PETG", "ABS", None], (60, 360)),
        (["abs", None, None, None], (65, 360)),
        (["PLA-CF", None, None, None], (45, 360)),
        (["petg", "TPU", None, None], (60, 360)),
        (["NYLON", None, None, None], (45, 360)),  # unknown → PLA
        ([None, None, None, None], (45, 360)),  # no strand anywhere → PLA
    ],
)
def test_the_profile_is_the_most_fragile_material(
    materials: list[str | None], want: tuple[int, int]
) -> None:
    module, _printer, mcu, _dryer = modes()
    for i, m in enumerate(materials):
        module.lanes[i]["insert"] = m is not None
    module.dryer_mode.set_material_source(lambda lane: materials[lane])
    mcu.subscriptions[STATE](auto_frame(profile_c=50, profile_min=100))
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile")[-1] == list(want)


def test_the_profile_is_not_sent_when_the_unit_has_it() -> None:
    module, _printer, mcu, _dryer = modes()
    mcu.subscriptions[STATE](auto_frame(profile_c=45, profile_min=360))
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == []


def test_no_profile_before_the_first_report() -> None:
    module, _printer, mcu, _dryer = modes()
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == []


def test_the_bench_minutes_replace_the_profile_minutes() -> None:
    _module, _printer, mcu, _dryer = modes({"bench_dry_cycle_minutes": 5})
    mcu.subscriptions[STATE](auto_frame())  # the report compares the profile
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == [[45, 5]]


def test_a_lane_without_a_strand_does_not_count() -> None:
    module, _printer, mcu, _dryer = modes()
    module.lanes[0]["insert"] = False
    module.lanes[1]["insert"] = True
    module.dryer_mode.set_material_source(lambda lane: ["PLA", "PETG", None, None][lane])
    mcu.subscriptions[STATE](auto_frame())
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile")[-1] == [60, 360]


def test_the_source_falls_back_to_the_tag() -> None:
    module, _printer, mcu, _dryer = modes()
    module.lanes[2]["insert"] = True
    module.rfid.lanes[2]["record"] = {"material": "PETG"}
    module.dryer_mode.set_material_source(lambda lane: None)
    mcu.subscriptions[STATE](auto_frame())
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile")[-1] == [60, 360]


def test_without_a_source_the_tag_is_read() -> None:
    module, _printer, mcu, _dryer = modes()
    module.lanes[0]["insert"] = True
    module.lanes[1]["insert"] = True  # no tag read: PLA
    module.rfid.lanes[0]["record"] = {"material": "ABS"}
    mcu.subscriptions[STATE](auto_frame())
    module.dryer_mode._refresh_profile()
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == []  # PLA 45 °C is the unit's already


def test_a_new_material_reaches_the_unit_within_five_seconds() -> None:
    # A more fragile strand lowers a running automatic cycle on the unit, so it must go promptly.
    module, _printer, mcu, _dryer = modes()
    module.lanes[0]["insert"] = True
    materials = ["ABS", "PLA", None, None]
    module.dryer_mode.set_material_source(lambda lane: materials[lane])
    mcu.subscriptions[STATE](auto_frame(profile_c=65))  # the unit has ABS's already
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == []
    module.lanes[1]["insert"] = True  # PLA enters
    module.dryer_mode._on_report(4.9)
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == []
    module.dryer_mode._on_report(5.0)
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == [[45, 360]]


def test_a_report_sends_the_profile_at_most_every_five_seconds(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, _printer, mcu, _dryer = modes(
        responses=dict(
            RESPONSES,
            ace2k_dryer_auto_profile=[
                dict(AUTO_OK, op=2),
                dict(AUTO_OK, op=2, accepted=0, reason=1),
            ],
        )
    )
    module.lanes[0]["insert"] = True
    module.dryer_mode.set_material_source(lambda lane: "ABS")
    mcu.subscriptions[STATE](auto_frame())  # the fake reactor's async callbacks run at time 0
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == [[65, 360]]
    mcu.subscriptions[STATE](auto_frame())  # not shown yet, and within 5 s: not sent again
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == [[65, 360]]
    module.dryer_mode._on_report(5.0)  # 5 s on: the unit refuses it, logged
    assert "profile not sent: ace2k: dryer profile refused: range" in caplog.text
    mcu.config_callback()  # a reconnect compares at once
    mcu.subscriptions[STATE](auto_frame())
    assert len(sent_to(mcu, "ace2k_dryer_auto_profile")) == 3


def test_a_raising_or_odd_material_counts_as_no_material(caplog: pytest.LogCaptureFixture) -> None:
    module, _printer, mcu, _dryer = modes()
    for lane in range(3):
        module.lanes[lane]["insert"] = True
    module.rfid.lanes[1]["record"] = {"material": 7}  # not a name: PLA
    module.rfid.lanes[2]["record"] = {"material": "ABS"}

    def source(lane: int) -> object:
        if lane == 0:
            raise KeyError("no spool")
        return 42 if lane == 1 else None  # not a name, then the tag

    module.dryer_mode.set_material_source(source)
    mcu.subscriptions[STATE](auto_frame(profile_c=65))
    assert sent_to(mcu, "ace2k_dryer_auto_profile") == [[45, 360]]
    assert "material source failed for lane 1" in caplog.text


def test_the_mode_reply_names_the_unit() -> None:
    units, printer, _mcus = make_units(responses=RESPONSES)
    for mcu in _mcus.values():
        mcu.known = MODES
        mcu.config_callback()
    gcmd = FakeGcmd({"UNIT": "b", "MODE": "print"})
    printer.objects["gcode"].commands["ACEK_DRY_MODE"](gcmd)
    assert gcmd.lines == ["ace2k b: dryer mode print"]


def test_the_status_carries_the_mode() -> None:
    module, _printer, mcu, _dryer = modes({"unattended_drying": True})
    mcu.subscriptions[STATE](auto_frame(mode=1, failures=1, cold=1, rh=352, above_s=0, own=0))
    d = module.get_status(0.0)["dryer"]
    assert d["mode"] == "humidity" and d["humidity_allowed"] is True
    assert d["failures"] == 1 and d["cold"] is True and d["limit"] == 40
    assert d["max_failures"] == 4 and d["stopped"] is False and d["above_s"] == 0
    assert d["own"] is False and d["bound"] is False
    assert d["profile"] == {"temp": 45, "minutes": 360}
    assert d["rh"] == 35  # the unit's idle cold reading, whole %, half up
    assert "state" in d  # the dryer's own fields stay


def test_the_status_rh_rounds_half_up() -> None:
    module, _printer, mcu, _dryer = modes()
    mcu.subscriptions[STATE](auto_frame(rh=355))
    assert module.get_status(0.0)["dryer"]["rh"] == 36
    mcu.subscriptions[STATE](auto_frame(rh=354))
    assert module.get_status(0.0)["dryer"]["rh"] == 35


def test_the_status_before_a_report() -> None:
    module, _printer, _mcu, _dryer = modes()
    d = module.get_status(0.0)["dryer"]
    assert d["humidity_allowed"] is False
    assert d["mode"] is None and d["profile"] is None and d["stopped"] is None
    assert d["rh"] is None


def test_ace_status_shows_the_mode_line() -> None:
    module, printer, mcu, _dryer = modes({"unattended_drying": True})
    mcu.subscriptions[STATE](auto_frame(mode=1, cold=1, rh=352))
    gcmd = gcode(printer, "ACEK_STATUS")
    text = "\n".join(gcmd.lines)
    assert "dryer mode: humidity — waiting: box 35 %, limit 40 %" in text


@pytest.mark.parametrize(
    ("frame", "line"),
    [
        ({}, "dryer mode: manual"),
        ({"mode": 1, "cold": 1, "rh": 345}, "waiting: box 35 %"),  # half up
        ({"mode": 1, "cold": 1, "rh": 344}, "waiting: box 34 %"),
        ({"mode": 2, "cold": 1}, "dryer mode: print"),
        ({"mode": 1, "flags": 1, "failures": 4}, "humidity — stopped after 4 failed cycles: check"),
        ({"mode": 1, "own": 1, "failures": 1}, "humidity — drying (auto) (1 failed cycle)"),
        ({"mode": 1, "cold": 0}, "humidity — box not steady yet, humidity not read"),
        ({"mode": 1, "cold": 1, "failures": 2}, "box 50 %, limit 40 % (2 failed cycles)"),
    ],
)
def test_the_mode_line(frame: dict[str, int], line: str) -> None:
    module, _printer, mcu, _dryer = modes({"unattended_drying": True})
    assert module.dryer_mode.status_line() == "dryer mode: no report yet"
    mcu.subscriptions[STATE](auto_frame(**frame))
    assert line in module.dryer_mode.status_line()


def test_start_bound_sends_the_bound_start() -> None:
    module, _printer, mcu, _dryer = modes()
    module.dryer_mode.start_bound(60, 1440)
    assert sent_to(mcu, "ace2k_dryer_start_bound") == [[60, 1440]]


@pytest.mark.parametrize(("target", "minutes"), [(70, 60), (60, 0), (True, 60), (60, 1441)])
def test_start_bound_refuses_bad_arguments(target: int, minutes: int) -> None:
    module, _printer, mcu, _dryer = modes()
    with pytest.raises(ValueError, match="outside"):
        module.dryer_mode.start_bound(target, minutes)
    assert sent_to(mcu, "ace2k_dryer_start_bound") == []


def test_a_refused_bound_start_raises_dryer_refused() -> None:
    module, _printer, _mcu, _dryer = modes(
        responses=dict(RESPONSES, ace2k_dryer_start_bound={"op": 3, "accepted": 0, "reason": 1})
    )
    with pytest.raises(ace2k_dryer.DryerRefused) as e:
        module.dryer_mode.start_bound(60, 60)
    assert (e.value.what, e.value.reason) == ("start", "busy")


def test_an_image_without_the_modes_keeps_every_dryer_gcode() -> None:
    module, printer, mcu, _dryer = build(known=RELEASE)
    assert not module.dryer_mode.present
    with pytest.raises(FakeGcmd.error, match="without the dryer's modes"):
        gcode(printer, "ACEK_DRY_MODE", MODE="print")
    with pytest.raises(RuntimeError, match="without the dryer's modes"):
        module.dryer_mode.start_bound(60, 60)
    gcode(printer, "ACEK_DRY", TEMP=45, DURATION=60)
    assert sent_to(mcu, "ace2k_dryer_start") == [[45, 60]]
    assert not any(c.startswith("ace2k_dryer_auto") for c, _ in mcu.config_cmds)
    gcmd = gcode(printer, "ACEK_STATUS")
    assert "dryer mode" not in "\n".join(gcmd.lines)


def test_an_image_without_the_dryer_names_the_dryer() -> None:
    _module, printer, _mcu, _dryer = build(known=NO_DRYER)
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_DRYER"):
        gcode(printer, "ACEK_DRY_MODE", MODE="manual")


def test_a_modes_format_from_another_commit_is_a_config_error() -> None:
    with pytest.raises(Exception, match="ace2k_dryer_auto_mode has another format"):
        build(known=MODES, formats={"ace2k_dryer_auto_mode": "ace2k_dryer_auto_mode m=%c"})
    with pytest.raises(Exception, match="ace2k_dryer_auto_state.*different commits"):
        build(known=MODES, formats={STATE: "ace2k_dryer_auto_state mode=%c"})
    with pytest.raises(Exception, match="ace2k_dryer_start_bound.*though ace2k_dryer_auto_mode"):
        build(known=MODES - {"ace2k_dryer_start_bound"})


def test_a_stop_bound_format_from_another_commit_is_a_config_error() -> None:
    with pytest.raises(Exception, match="ace2k_dryer_stop_bound.*different commits"):
        build(known=MODES_STOP_BOUND, formats={STOP_BOUND: "ace2k_dryer_stop_bound x=%c"})


def test_stop_bound_sends_the_units_own_stop() -> None:
    module, _p, mcu, _d = build(known=MODES_STOP_BOUND, responses=RESPONSES_STOP_BOUND)
    module.dryer_mode.stop_bound()
    assert sent_to(mcu, STOP_BOUND) == [[]]
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_stop_bound_refused_raises_as_the_stop_does() -> None:
    responses = dict(
        RESPONSES_STOP_BOUND, ace2k_dryer_stop_bound={"op": 4, "accepted": 0, "reason": 4}
    )
    module, _p, _mcu, _d = build(known=MODES_STOP_BOUND, responses=responses)
    with pytest.raises(ace2k_dryer.DryerRefused, match="dryer stop refused"):
        module.dryer_mode.stop_bound()


def test_stop_bound_answered_for_another_request_is_no_answer() -> None:
    responses = dict(
        RESPONSES_STOP_BOUND, ace2k_dryer_stop_bound={"op": 1, "accepted": 1, "reason": 0}
    )
    module, _p, _mcu, _d = build(known=MODES_STOP_BOUND, responses=responses)
    with pytest.raises(RuntimeError, match="answer for another request"):
        module.dryer_mode.stop_bound()


def test_stop_bound_on_an_image_without_it_is_the_plain_stop() -> None:
    module, _p, mcu, _d = modes()
    module.dryer_mode.stop_bound()
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]


def test_the_interface_version_is_27() -> None:
    assert ace2k_feed_proto.API_VERSION == 28


MODE_KEYS = [
    "humidity_allowed",
    "mode",
    "limit",
    "max_failures",
    "failures",
    "stopped",
    "cold",
    "above_s",
    "own",
    "bound",
    "profile",
    "rh",
]


def test_the_mode_keys_are_the_same_before_and_after_a_report() -> None:
    module, _printer, mcu, _dryer = modes()
    assert list(module.dryer_mode.status()) == MODE_KEYS
    mcu.subscriptions[STATE](auto_frame(mode=7))
    status = module.dryer_mode.status()
    assert list(status) == MODE_KEYS
    assert status["mode"] == 7  # a mode this host does not know stays its code
