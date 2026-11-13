"""ace2k_units.py: the coordinator — unit names, UNIT=, the console label, the dispatch."""

from __future__ import annotations

import json
import os
import types
from typing import TYPE_CHECKING, NoReturn

import ace2k
import ace2k_env
import ace2k_feed
import ace2k_units
import pytest
import test_ace2k_dryer as td
import test_ace2k_rfid as tr
from test_ace2k_extras import (
    RESPONSES,
    FakeGcmd,
    FakeMcu,
    FakePrinter,
    UnitConfig,
    fire_ready,
    make,
    make_units,
)
from test_ace2k_feed import WITH_FOLLOW, accepted, event_params, state_frame
from test_ace2k_ff import (
    BASE_FMT,
    WITH_FF,
    FakeClock,
    MoveExtruder,
    extrude,
    set_modes,
)
from test_ace2k_ff import FOLLOWING as FF_FOLLOWING
from test_ace2k_follow import FOLLOWING, IDLE, deliver, report, started

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Sequence
    from typing import Any

    from ace2k import Ace2k
    from ace2k_feed_proto import Move
    from ace2k_units import Key

    # two named units on one printer: ((a, b), printer, {name: mcu})
    TwoUnits = tuple[tuple[Ace2k, Ace2k], FakePrinter, dict[str, FakeMcu]]
    Handler = Callable[[FakeGcmd], None]


class Section:
    def __init__(self, name: str) -> None:
        self.name = name

    def get_name(self) -> str:
        return self.name


class SectionsConfig:
    def __init__(self, names: Iterable[str]) -> None:
        self.sections = [Section(n) for n in names]

    def get_prefix_sections(self, prefix: str) -> list[Section]:
        return [s for s in self.sections if s.get_name().startswith(prefix)]


class StubUnit:
    """What the coordinator reads off a unit: its name, section and MCU."""

    def __init__(self, name: str | None, mcu: object = None) -> None:
        self.name = name
        self.section = "ace2k" if name is None else f"ace2k {name}"
        self.mcu = mcu if mcu is not None else object()


def coordinator(
    names: Sequence[str | None],
) -> tuple[ace2k_units.Units, FakePrinter, list[StubUnit]]:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    units = ace2k_units.Units(printer, list(names))
    stubs = [StubUnit(n) for n in names]
    for stub in stubs:
        units.add(stub)
    return units, printer, stubs


def recorder(calls: list[tuple[str, FakeGcmd]], tag: str) -> Handler:
    def handler(gcmd: FakeGcmd) -> None:
        calls.append((tag, gcmd))
        gcmd.respond_info(f"ace2k: {tag} ran")

    return handler


def test_unit_names_reads_the_ace2k_sections_in_file_order() -> None:
    names = ["ace2k a", "ace2k_tag bambu", "ace2k B", "ace2k_u1"]
    assert ace2k_units.unit_names(SectionsConfig(names)) == ["a", "b"]
    assert ace2k_units.unit_names(SectionsConfig(["ace2k", "ace2k_tag x"])) == [None]


@pytest.mark.parametrize(
    "section, expected",
    [("ace2k", None), ("ace2k a", "a"), ("ace2k Left_1", "left_1"), ("ace2k  B ", "b")],
)
def test_unit_name_is_none_for_the_bare_section_and_the_lower_case_name_otherwise(
    section: str, expected: str | None
) -> None:
    assert ace2k_units.unit_name(section) == expected


def test_the_two_forms_together_are_a_config_error() -> None:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    with pytest.raises(Exception, match=r"\[ace2k\] together with \[ace2k <name>\]"):
        ace2k_units.Units(printer, [None, "a"])


@pytest.mark.parametrize("section", ["ace2k left unit", "ace2k a/b", "ace2k a.b", "ace2k a=b"])
def test_a_unit_name_beyond_letters_digits_underscore_and_dash_is_a_config_error(
    section: str,
) -> None:
    names = ace2k_units.unit_names(SectionsConfig([section, "ace2k c"]))
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    wanted = f"^ace2k: unit name '{names[0]}' may only hold letters, digits, '_' and '-'$"
    with pytest.raises(Exception, match=wanted.replace(".", r"\.")):
        ace2k_units.Units(printer, names)


def test_a_unit_name_of_letters_digits_underscore_and_dash_is_accepted() -> None:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    units = ace2k_units.Units(printer, ["left_1", "right-2"])
    assert units.names == ["left_1", "right-2"]


def test_two_units_on_one_mcu_is_a_config_error() -> None:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    units = ace2k_units.Units(printer, ["a", "b"])
    mcu = object()
    units.add(StubUnit("a", mcu))
    with pytest.raises(Exception, match="ace2k b: its mcu is ace2k a's too"):
        units.add(StubUnit("b", mcu))


def test_the_lane_map_is_built_once_per_unit_where_its_feed_is_attached(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    seen = []
    original = ace2k_units.Units.map_changed

    def counted(self: ace2k_units.Units) -> None:
        seen.append([getattr(u, "feed", None) is not None for u in self.units])
        original(self)

    monkeypatch.setattr(ace2k_units.Units, "map_changed", counted)
    make_units(names=("a", "b"))
    # one call per unit, each once that unit's feed exists — none from add()
    assert seen == [[True], [True, True]]


def test_relabel_rewrites_only_the_leading_prefix_and_is_idempotent() -> None:
    text = "ace2k: lane 1 on\nlane 2: x\nace2k_u1: y"
    once = ace2k_units.relabel(text, "ace2k b")
    assert once == "ace2k b: lane 1 on\nlane 2: x\nace2k_u1: y"
    assert ace2k_units.relabel(once, "ace2k b") == once
    assert ace2k_units.relabel(text, "ace2k") == text


def test_one_unit_passes_the_gcmd_through_and_accepts_its_own_name() -> None:
    units, printer, (a,) = coordinator(["a"])
    calls = []
    units.register_command(a, "ACEK_FEED", recorder(calls, "a"), "help")
    gcode = printer.objects["gcode"]
    gcmd = FakeGcmd({"LANE": "1"})
    gcode.commands["ACEK_FEED"](gcmd)
    assert calls[0][1] is gcmd and gcmd.lines == ["ace2k: a ran"]
    gcode.commands["ACEK_FEED"](FakeGcmd({"UNIT": "A", "LANE": "1"}))
    assert len(calls) == 2
    with pytest.raises(Exception, match="ace2k: no unit x; units: a"):
        gcode.commands["ACEK_FEED"](FakeGcmd({"UNIT": "x"}))


def test_several_units_refuse_a_lane_command_without_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_FEED", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_FEED", recorder(calls, "b"), "help")
    with pytest.raises(
        Exception, match=r"^ace2k: ACEK_FEED needs UNIT=<a\|b>: this printer has several units$"
    ):
        printer.objects["gcode"].commands["ACEK_FEED"](FakeGcmd({"LANE": "1"}))
    assert calls == []


def test_a_host_module_reading_unit_gets_the_same_refusal_as_a_command() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    with pytest.raises(
        Exception, match=r"^ace2k: ACEK_U1_EJECT needs UNIT=<a\|b>: this printer has several units$"
    ):
        units.unit_for(FakeGcmd({}, command="ACEK_U1_EJECT"))


def test_several_units_route_by_unit_and_label_the_lines_and_errors() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []

    def failing(gcmd: FakeGcmd) -> NoReturn:
        raise gcmd.error("ace2k: lane 1 refused")

    units.register_command(a, "ACEK_FEED", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_FEED", recorder(calls, "b"), "help")
    units.register_command(a, "ACEK_CLEAR", failing, "help")
    units.register_command(b, "ACEK_CLEAR", failing, "help")
    gcmd = FakeGcmd({"UNIT": "B", "LANE": "1"})
    printer.objects["gcode"].commands["ACEK_FEED"](gcmd)
    assert [tag for tag, _ in calls] == ["b"] and gcmd.lines == ["ace2k b: b ran"]
    with pytest.raises(Exception, match="^ace2k a: lane 1 refused$"):
        printer.objects["gcode"].commands["ACEK_CLEAR"](FakeGcmd({"UNIT": "a", "LANE": "1"}))


def test_stop_without_unit_stops_every_lane_of_every_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    seen = []

    def stop(tag: str) -> Handler:
        def handler(gcmd: FakeGcmd) -> None:
            seen.append((tag, gcmd.get("LANE", None)))

        return handler

    units.register_command(a, "ACEK_STOP", stop("a"), "help")
    units.register_command(b, "ACEK_STOP", stop("b"), "help")
    gcmd = FakeGcmd({})
    printer.objects["gcode"].commands["ACEK_STOP"](gcmd)
    assert seen == [("a", None), ("b", None)]
    assert gcmd.lines == []


def test_stop_with_a_lane_but_no_unit_is_refused_and_stops_nothing() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_STOP", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_STOP", recorder(calls, "b"), "help")
    with pytest.raises(
        Exception, match=r"^ace2k: ACEK_STOP needs UNIT=<a\|b>: this printer has several units$"
    ):
        printer.objects["gcode"].commands["ACEK_STOP"](FakeGcmd({"LANE": "2"}))
    assert calls == []


def test_stop_with_a_lane_and_a_unit_stops_that_lane_of_that_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    seen = []
    units.register_command(a, "ACEK_STOP", lambda g: seen.append(("a", g.get("LANE", None))), "h")
    units.register_command(b, "ACEK_STOP", lambda g: seen.append(("b", g.get("LANE", None))), "h")
    printer.objects["gcode"].commands["ACEK_STOP"](FakeGcmd({"UNIT": "b", "LANE": "2"}))
    assert seen == [("b", "2")]


def test_status_without_unit_runs_on_every_unit_and_an_error_does_not_stop_the_others() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []

    def broken(gcmd: FakeGcmd) -> NoReturn:
        raise gcmd.error("ace2k: no answer")

    units.register_command(a, "ACEK_STATUS", broken, "help")
    units.register_command(b, "ACEK_STATUS", recorder(calls, "b"), "help")
    with pytest.raises(Exception, match="^ace2k a: no answer$"):
        printer.objects["gcode"].commands["ACEK_STATUS"](FakeGcmd({}))
    assert [tag for tag, _ in calls] == ["b"]


class CommandError(Exception):
    """Klipper's gcmd.error: a class of its own, so another exception is not one."""


class KlipperGcmd(FakeGcmd):
    error = CommandError


def test_stop_without_unit_runs_every_unit_and_re_raises_an_internal_error_unchanged(
    caplog: pytest.LogCaptureFixture,
) -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    seen = []
    boom = RuntimeError("link down")

    def broken(gcmd: FakeGcmd) -> NoReturn:
        seen.append("a")
        raise boom

    def stop(gcmd: FakeGcmd) -> None:
        seen.append("b")

    units.register_command(a, "ACEK_STOP", broken, "help")
    units.register_command(b, "ACEK_STOP", stop, "help")
    with caplog.at_level("ERROR"), pytest.raises(RuntimeError) as raised:
        printer.objects["gcode"].commands["ACEK_STOP"](KlipperGcmd({}))
    # every unit ran, then the internal error goes on as it came: Klipper shuts down on it, as
    # it does when the command names the unit
    assert raised.value is boom
    assert seen == ["a", "b"]
    assert "link down" in caplog.text  # the traceback is logged
    # with several units the log line names the unit like the other multi-unit lines
    assert "ace2k a: ACEK_STOP failed" in caplog.text


def test_an_internal_error_on_one_unit_names_the_one_with_unit_too() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    boom = RuntimeError("link down")

    def broken(gcmd: FakeGcmd) -> NoReturn:
        raise boom

    units.register_command(a, "ACEK_STOP", broken, "help")
    units.register_command(b, "ACEK_STOP", lambda gcmd: None, "help")
    with pytest.raises(RuntimeError) as raised:
        printer.objects["gcode"].commands["ACEK_STOP"](KlipperGcmd({"UNIT": "a"}))
    assert raised.value is boom


def test_the_first_internal_error_is_raised_after_every_refusal_is_printed() -> None:
    units, printer, (a, b, c, d) = coordinator(["a", "b", "c", "d"])
    first, second = RuntimeError("link down"), KeyError("lane")

    def refused(gcmd: FakeGcmd) -> NoReturn:
        raise gcmd.error("ace2k: no answer")

    def broken(error: BaseException) -> Callable[[FakeGcmd], NoReturn]:
        def handler(gcmd: FakeGcmd) -> NoReturn:
            raise error

        return handler

    units.register_command(a, "ACEK_STATUS", refused, "help")
    units.register_command(b, "ACEK_STATUS", broken(first), "help")
    units.register_command(c, "ACEK_STATUS", recorder([], "c"), "help")
    units.register_command(d, "ACEK_STATUS", broken(second), "help")
    gcmd = KlipperGcmd({})
    with pytest.raises(RuntimeError) as raised:
        printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert raised.value is first
    assert [line for line in gcmd.lines if line.startswith("!! ")] == ["!! ace2k a: no answer"]
    assert "ace2k c: c ran" in gcmd.lines


def test_every_units_refusal_reaches_the_console_marked_as_an_error() -> None:
    units, printer, (a, b, c) = coordinator(["a", "b", "c"])

    def refused(text: str) -> Callable[[FakeGcmd], NoReturn]:
        def handler(gcmd: FakeGcmd) -> NoReturn:
            raise gcmd.error(text)

        return handler

    units.register_command(a, "ACEK_STATUS", refused("ace2k: no answer"), "help")
    units.register_command(b, "ACEK_STATUS", recorder([], "b"), "help")
    units.register_command(c, "ACEK_STATUS", refused("ace2k: link down"), "help")
    gcmd = KlipperGcmd({})
    with pytest.raises(CommandError) as raised:
        printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    # all but the last are printed as Klipper prints an error; the last is the command's error,
    # which Klipper prints the same way
    assert [line for line in gcmd.lines if line.startswith("!! ")] == ["!! ace2k a: no answer"]
    assert str(raised.value) == "ace2k c: link down"


def test_a_run_on_every_unit_labels_each_units_error_without_the_prefix_too() -> None:
    units, printer, (a, b, c) = coordinator(["a", "b", "c"])

    def parse_error(gcmd: FakeGcmd) -> NoReturn:
        raise gcmd.error("Error on 'ACEK_STATUS': unable to parse x")

    units.register_command(a, "ACEK_STATUS", parse_error, "help")
    units.register_command(b, "ACEK_STATUS", recorder([], "b"), "help")
    units.register_command(c, "ACEK_STATUS", parse_error, "help")
    gcmd = KlipperGcmd({})
    with pytest.raises(CommandError) as raised:
        printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert [line for line in gcmd.lines if line.startswith("!! ")] == [
        "!! ace2k a: Error on 'ACEK_STATUS': unable to parse x"
    ]
    assert str(raised.value) == "ace2k c: Error on 'ACEK_STATUS': unable to parse x"


def test_labelled_names_the_unit_on_every_line_once() -> None:
    text = "ace2k: no answer\nError on 'X'\nace2k b: already"
    assert ace2k_units.labelled(text, "ace2k b") == (
        "ace2k b: no answer\nace2k b: Error on 'X'\nace2k b: already"
    )
    assert ace2k_units.labelled(ace2k_units.labelled(text, "ace2k b"), "ace2k b") == (
        ace2k_units.labelled(text, "ace2k b")
    )


@pytest.mark.parametrize(
    "text, expected",
    [
        ("ace2k: link down", "link down"),
        ("ace2k:link down", "link down"),
        ("ace2k:  link down", "link down"),
        ("link down", "link down"),
        ("ace2k: x", "x"),
        ("ace2k:x", "x"),
        ("ace2k b: x", "x"),
        ("ace2k b: link down", "link down"),
        ("ace2k Unit_2-x:  link down", "link down"),
        ("lane ace2k: x", "lane ace2k: x"),
        ("ace2k b c: x", "ace2k b c: x"),
        ("ace2kb: x", "ace2kb: x"),
    ],
)
def test_unprefixed_strips_one_leading_ace2k_or_unit_label_with_or_without_the_space(
    text: str, expected: str
) -> None:
    assert ace2k_units.unprefixed(text) == expected
    assert ace2k_units.unprefixed(ValueError(text)) == expected


def test_a_lanes_key_is_its_units_place_without_a_search() -> None:
    units, _, (a, b, c) = coordinator(["a", "b", "c"])
    assert units.index == {a: 0, b: 1, c: 2}
    units.units = None  # a search through the list would fail now
    assert units.key(c, 3) == (2, 3)
    assert units.key(a, 0) == (0, 0)


def test_say_labels_a_line_printed_outside_a_command() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    units.say(b, "ace2k: lane 3 follow on")
    assert printer.objects["gcode"].output[-1] == "ace2k b: lane 3 follow on"
    one, printer1, (only,) = coordinator([None])
    one.say(only, "ace2k: lane 3 follow on")
    assert printer1.objects["gcode"].output[-1] == "ace2k: lane 3 follow on"


def test_two_sections_naming_one_unit_are_a_config_error() -> None:
    names = ace2k_units.unit_names(SectionsConfig(["ace2k A", "ace2k a"]))
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    with pytest.raises(Exception, match="ace2k: two sections name unit a"):
        ace2k_units.Units(printer, names)


def test_an_unknown_unit_lists_the_units() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    units.register_command(a, "ACEK_FEED", recorder([], "a"), "help")
    with pytest.raises(Exception, match="^ace2k: no unit x; units: a, b$"):
        printer.objects["gcode"].commands["ACEK_FEED"](FakeGcmd({"UNIT": "x"}))


def test_a_unit_name_on_the_one_unnamed_unit_says_to_omit_it() -> None:
    units, printer, (only,) = coordinator([None])
    units.register_command(only, "ACEK_FEED", recorder([], "only"), "help")
    with pytest.raises(
        Exception, match="^ace2k: no unit a; this printer has one unit: omit UNIT=$"
    ):
        printer.objects["gcode"].commands["ACEK_FEED"](FakeGcmd({"UNIT": "a"}))
    assert "None" not in units.declared_text("|")


@pytest.mark.parametrize("blank", ["", "  "])
def test_an_empty_or_blank_unit_counts_as_omitted(blank: str) -> None:
    one, printer1, (only,) = coordinator([None])
    calls = []
    one.register_command(only, "ACEK_FEED", recorder(calls, "only"), "help")
    printer1.objects["gcode"].commands["ACEK_FEED"](FakeGcmd({"UNIT": blank, "LANE": "1"}))
    assert [tag for tag, _ in calls] == ["only"]  # the one unit, as without UNIT=
    units, printer, (a, b) = coordinator(["a", "b"])
    units.register_command(a, "ACEK_FEED", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_FEED", recorder(calls, "b"), "help")
    with pytest.raises(
        Exception, match=r"^ace2k: ACEK_FEED needs UNIT=<a\|b>: this printer has several units$"
    ):
        printer.objects["gcode"].commands["ACEK_FEED"](FakeGcmd({"UNIT": blank, "LANE": "1"}))
    for name in ("ACEK_STOP", "ACEK_STATUS"):
        units.register_command(a, name, recorder(calls, "a"), "help")
        units.register_command(b, name, recorder(calls, "b"), "help")
        del calls[:]
        printer.objects["gcode"].commands[name](FakeGcmd({"UNIT": blank}))
        assert [tag for tag, _ in calls] == ["a", "b"]  # every unit, as without UNIT=


def test_health_without_unit_runs_on_every_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_HEALTH", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_HEALTH", recorder(calls, "b"), "help")
    printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd({}))
    assert [tag for tag, _ in calls] == ["a", "b"]


@pytest.mark.parametrize("params", [{"CLEAR": "1"}, {"RUN": "1"}, {"LANE": "2"}])
def test_health_with_a_parameter_but_no_unit_is_refused_and_runs_nowhere(
    params: dict[str, str],
) -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_HEALTH", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_HEALTH", recorder(calls, "b"), "help")
    with pytest.raises(
        Exception, match=r"^ace2k: ACEK_HEALTH needs UNIT=<a\|b>: this printer has several units$"
    ):
        printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd(params))
    assert calls == []


def test_health_with_a_parameter_and_a_unit_runs_on_that_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_HEALTH", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_HEALTH", recorder(calls, "b"), "help")
    printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd({"UNIT": "b", "CLEAR": "1"}))
    assert [tag for tag, _ in calls] == ["b"]


def test_status_plain_runs_on_every_unit() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])
    calls = []
    units.register_command(a, "ACEK_STATUS", recorder(calls, "a"), "help")
    units.register_command(b, "ACEK_STATUS", recorder(calls, "b"), "help")
    printer.objects["gcode"].commands["ACEK_STATUS"](FakeGcmd({}))
    assert [tag for tag, _ in calls] == ["a", "b"]


def test_an_error_without_the_prefix_is_re_raised_unchanged() -> None:
    units, printer, (a, b) = coordinator(["a", "b"])

    def failing(gcmd: FakeGcmd) -> NoReturn:
        raise gcmd.error("lane 1 refused")

    units.register_command(a, "ACEK_CLEAR", failing, "help")
    with pytest.raises(Exception, match="^lane 1 refused$"):
        printer.objects["gcode"].commands["ACEK_CLEAR"](FakeGcmd({"UNIT": "a"}))


def test_say_falls_back_to_the_log_when_the_console_raises(
    caplog: pytest.LogCaptureFixture,
) -> None:
    units, printer, (a, b) = coordinator(["a", "b"])

    def broken(msg: str) -> NoReturn:
        raise RuntimeError("no console")

    printer.objects["gcode"].respond_info = broken
    with caplog.at_level("INFO"):
        units.say(b, "ace2k: lane 3 follow on")
    assert "ace2k b: lane 3 follow on" in caplog.text


def test_one_unnamed_unit_keeps_its_object_names() -> None:
    module, printer, mcu = make(dict(RESPONSES))
    for name in ("ace2k_feed", "ace2k_rfid", "ace2k_dryer", "ace2k_units"):
        assert name in printer.objects
    assert printer.objects["ace2k_units"].units == [module]


def test_two_named_units_each_have_their_own_modules_and_mcu() -> None:
    (a, b), printer, mcus = make_units()
    for n, unit in (("a", a), ("b", b)):
        assert printer.objects[f"ace2k_feed {n}"] is unit.feed
        assert printer.objects[f"ace2k_rfid {n}"] is unit.rfid
        assert printer.objects[f"ace2k_dryer {n}"] is unit.dryer
        assert unit.feed.mcu is mcus[n] and unit.mcu is mcus[n]
    assert printer.objects["ace2k_units"].get_status(0) == {
        "units": ["ace2k a", "ace2k b"],
        "active_lanes": {},
    }


def test_a_section_named_in_capitals_keeps_its_spelling_on_every_object() -> None:
    (a, b), printer, mcus = make_units(names=("A", "b"))
    assert a.name == "a" and a.section == "ace2k A"
    assert printer.objects["ace2k A"] is a
    assert printer.objects["ace2k_feed A"] is a.feed
    assert printer.objects["ace2k_rfid A"] is a.rfid
    assert printer.objects["ace2k_dryer A"] is a.dryer
    assert "ace2k_feed a" not in printer.objects
    assert printer.objects["ace2k_units"].get_status(0)["units"] == ["ace2k A", "ace2k b"]
    calls = []
    units = printer.objects["ace2k_units"]
    units.register_command(a, "ACEK_TEST", recorder(calls, "a"), "help")
    printer.objects["gcode"].commands["ACEK_TEST"](FakeGcmd({"UNIT": "a"}))
    assert [tag for tag, _ in calls] == ["a"]


def test_a_named_unit_without_mcu_is_a_config_error() -> None:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    with pytest.raises(Exception, match=r"ace2k a: mcu: is required"):
        ace2k.load_config_prefix(UnitConfig(printer, "a", None))


def test_status_without_unit_reports_every_unit_by_name() -> None:
    (a, b), printer, mcus = make_units()
    for unit in (a, b):
        unit._on_mcu_identify()
    gcmd = FakeGcmd({})
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    headers = [line.split("\n")[0] for line in gcmd.lines]
    assert headers == ["ace2k a: 0.1.0, link proven: yes", "ace2k b: 0.1.0, link proven: yes"]


class SensorConfig(UnitConfig):
    def getchoice(self, key: str, choices: object) -> object:
        return self.options[key]

    def get_name(self) -> str:
        return "temperature_sensor x"


def sensor(printer: FakePrinter, options: dict[str, Any]) -> ace2k_env.Ace2kTemperature:
    return ace2k_env.Ace2kTemperature(SensorConfig(printer, None, None, options))


def test_a_temperature_sensor_names_its_unit() -> None:
    (a, b), printer, mcus = make_units()

    def sensor_(options: dict[str, Any]) -> ace2k_env.Ace2kTemperature:
        return sensor(printer, options)

    s = sensor_({"unit": "B", "sensor": "chamber"})
    assert s._on_env in b.env_listeners and s._on_env not in a.env_listeners
    # the coordinator's wording, after the sensor's own subject
    wanted = (
        r"^ace2k: sensor temperature_sensor x: needs unit:<a\|b>: this printer has several units$"
    )
    with pytest.raises(Exception, match=wanted):
        sensor_({"sensor": "chamber"})
    for blank in ("", "  "):  # an empty or blank unit: is as if omitted
        with pytest.raises(Exception, match=wanted):
            sensor_({"unit": blank, "sensor": "chamber"})
    with pytest.raises(
        Exception, match="^ace2k: sensor temperature_sensor x: no unit c; units: a, b$"
    ):
        sensor_({"unit": "c", "sensor": "chamber"})


@pytest.mark.parametrize("raw", ["ace2k", "a", "ACE2K"])
def test_a_sensor_naming_a_unit_on_the_one_unnamed_unit_is_told_to_omit_it(raw: str) -> None:
    module, printer, mcu = make(dict(RESPONSES))
    with pytest.raises(
        Exception,
        match=f"^ace2k: sensor temperature_sensor x: no unit {raw}; this printer has one unit:"
        " omit unit:$",
    ):
        sensor(printer, {"unit": raw, "sensor": "chamber"})
    assert module.env_listeners == []
    s = sensor(printer, {"sensor": "chamber"})  # without unit:, the one unit
    assert s._on_env in module.env_listeners
    blank = sensor(printer, {"unit": " ", "sensor": "chamber"})  # a blank unit: is omitted
    assert blank._on_env in module.env_listeners


def test_a_sensor_between_two_unit_sections_resolves_against_the_declared_units() -> None:
    # [ace2k a], [temperature_sensor x], [ace2k b]: b is declared but not loaded yet
    mcus = {n: FakeMcu(dict(RESPONSES)) for n in ("a", "b")}
    printer = FakePrinter(mcus["a"])
    for n, mcu in mcus.items():
        mcu.reactor = printer.reactor
        printer.objects[f"mcu ace2k_{n}"] = mcu
    sections = []
    for n in ("a", "b"):
        sections.append(UnitConfig(printer, n, f"ace2k_{n}", {}, sections))
    a = ace2k.load_config_prefix(sections[0])
    # without unit:, never bound to the one unit loaded so far
    with pytest.raises(Exception, match="needs unit:<a\\|b>: this printer has several units"):
        sensor(printer, {"sensor": "chamber"})
    with pytest.raises(Exception, match=r"must come after every \[ace2k\] section"):
        sensor(printer, {"unit": "b", "sensor": "chamber"})
    with pytest.raises(Exception, match="no unit c"):
        sensor(printer, {"unit": "c", "sensor": "chamber"})
    assert a.env_listeners == []


def test_a_sensor_before_any_unit_section_is_a_config_error() -> None:
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    with pytest.raises(Exception, match=r"ace2k: sensor temperature_sensor x must come after"):
        sensor(printer, {"sensor": "chamber"})


# --- one active lane per shared extruder, across units -----------------------------------------


def rig_units(
    options: dict[str, dict[str, Any]] | None = None,
    inserts: dict[str, Sequence[bool]] | None = None,
) -> TwoUnits:
    """Two connected units with the follow, every start accepted for its lane, klippy:ready
    fired, the inserts set: inserts {name: (lane1, …, lane4)}."""
    (a, b), printer, mcus = make_units(options=options, constants=WITH_FOLLOW)
    for unit, mcu in zip((a, b), mcus.values()):
        unit.feed._seed([0] * 4)
        mcu.config_callback()
        mcu.responses["ace2k_feed_start"] = lambda m=mcu: {
            "lane": m.queries[-1][1][0],
            "accepted": 1,
            "reason": 0,
        }
    printer.shutdown = False
    printer.is_shutdown = lambda: printer.shutdown
    printer.handlers["klippy:ready"]()
    for unit in (a, b):
        for lane, insert in enumerate((inserts or {}).get(unit.name, (False,) * 4)):
            unit.lanes[lane]["insert"] = insert
    return (a, b), printer, mcus


def report_units(mcus: dict[str, FakeMcu]) -> None:
    for mcu in mcus.values():
        report(mcu)


SHARED_ACROSS = {"a": {"lane1_extruder": "extruder"}, "b": {"lane3_extruder": "extruder"}}
INSERTS_ACROSS = {"a": (True, False, False, False), "b": (False, False, True, False)}


def test_a_shared_extruder_across_units_arms_neither_lane_and_says_so_once() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    report_units(mcus)
    assert started(mcus["a"]) == [] and started(mcus["b"]) == []
    said = [m for m in printer.objects["gcode"].output if "share" in m]
    assert said == ["ace2k a: lanes a:1, b:3 share extruder: choose one with ACEK_LANE_ACTIVE"]
    status = printer.objects["ace2k_units"].get_status(0)
    assert status["active_lanes"] == {"extruder": None}
    # nothing chosen: None on every unit, never another unit's lane
    assert a.get_status(0)["active_lanes"] == {"extruder": None}
    assert b.get_status(0)["active_lanes"] == {"extruder": None}


def shared_lines(printer: FakePrinter) -> list[str]:
    return [m for m in printer.objects["gcode"].output if "share" in m]


def test_a_group_of_a_unit_without_the_default_follow_is_not_announced_by_another_unit() -> None:
    # unit b's two lanes share extruder1 and b arms nothing by itself; unit a's tick must not
    # tell the operator to choose among them
    options = {
        "b": {"follow_default": False, "lane1_extruder": "extruder1", "lane2_extruder": "extruder1"}
    }
    (a, b), printer, mcus = rig_units(options=options, inserts={"b": (True, True, False, False)})
    report_units(mcus)
    report_units(mcus)
    assert shared_lines(printer) == []


def test_a_group_with_a_member_unit_with_the_default_follow_is_announced_once() -> None:
    options = {
        "a": {"follow_default": False, "lane1_extruder": "extruder1"},
        "b": {"lane3_extruder": "extruder1"},
    }
    (a, b), printer, mcus = rig_units(options=options, inserts=INSERTS_ACROSS)
    report_units(mcus)
    report_units(mcus)
    assert shared_lines(printer) == [
        "ace2k a: lanes a:1, b:3 share extruder1: choose one with ACEK_LANE_ACTIVE"
    ]


def test_lane_active_across_units_stops_the_other_unit_first() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    assert started(mcus["a"]) == [(0, 6, 50000)]
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = a.feed.stop
    a.feed.stop = lambda lane, by="host": (
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    gcmd = FakeGcmd({"UNIT": "b", "LANE": "3"})
    gcode.commands["ACEK_LANE_ACTIVE"](gcmd)
    assert gcmd.lines[-1] == "ace2k b: lane b:3 active on extruder; lane a:1 off"
    assert a.feed.lanes[0]["last_event"]["stop_by"] == "command"
    assert started(mcus["b"]) == [(2, 6, 50000)]
    assert printer.objects["ace2k_units"].get_status(0)["active_lanes"] == {"extruder": "b:3"}
    # each unit's own view: its own lane's number, or the other unit's lane as "unit:lane"
    assert a.get_status(0)["active_lanes"] == {"extruder": "b:3"}
    assert b.get_status(0)["active_lanes"] == {"extruder": 3}
    # ACEK_STATUS names the group across units on each unit that feeds it
    assert "extruder: lanes a:1, b:3 — active lane b:3" in a.feed.status_lines()
    assert "extruder: lanes a:1, b:3 — active lane b:3" in b.feed.status_lines()


def test_lane_active_arms_nothing_while_the_other_units_lane_has_not_ended() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    with pytest.raises(Exception, match="lane a:1 stopped but did not end"):
        gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "b", "LANE": "3"}))
    assert started(mcus["b"]) == []
    assert printer.objects["ace2k_units"].get_status(0)["active_lanes"] == {"extruder": "a:1"}


def test_a_host_follow_start_on_a_lane_shared_across_units_not_chosen_is_refused() -> None:
    # a host module (the U1 adapter) armed a lane of an extruder another unit's lane
    # feeds too, with nothing chosen — refused on the host, nothing sent to either unit
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    for unit, lane in ((a, 0), (b, 2)):
        with pytest.raises(ace2k_feed.FeedRefused) as refused:
            unit.feed.start_move(lane, "assist_both", 0.0, 50.0)
        assert refused.value.reason == "not_active"
    assert started(mcus["a"]) == [] and started(mcus["b"]) == []
    gcode = printer.objects["gcode"]
    with pytest.raises(Exception) as refused:
        gcode.commands["ACEK_ASSIST"](FakeGcmd({"UNIT": "b", "LANE": "3", "DIR": "BOTH"}))
    assert str(refused.value) == (
        "ace2k b: lane b:3 shares extruder and is not its active lane: choose it with"
        " ACEK_LANE_ACTIVE LANE=3 UNIT=b"
    )
    assert started(mcus["b"]) == []
    # chosen, it follows; the other unit's lane is still refused, with the choice kept
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    assert started(mcus["a"]) == [(0, 6, 50000)]
    with pytest.raises(ace2k_feed.FeedRefused, match="not_active"):
        b.feed.start_move(2, "assist_both", 0.0, 50.0)
    assert started(mcus["b"]) == []
    units = printer.objects["ace2k_units"]
    assert units.get_status(0)["active_lanes"] == {"extruder": "a:1"}
    assert units.may_follow(a, 0) and not units.may_follow(b, 2)
    assert units.may_follow(b, 0)  # a lane with no shared extruder
    assert units.shared_lanes(a, 0) == "a:1, b:3" == units.shared_lanes(b, 2)
    assert units.shared_lanes(b, 0) is None


def test_lane_active_allows_its_own_arm_only_for_the_lane_it_chooses() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    seen = []
    start = b.feed._start_move

    def start_move(owner: str, lane: int, mode: str, length: float, speed: float) -> Move:
        seen.append((units.may_follow(b, 2), units.may_follow(a, 0)))
        return start(owner, lane, mode, length, speed)

    b.feed._start_move = start_move
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "b", "LANE": "3"}))
    assert seen == [(True, False)]
    assert started(mcus["b"]) == [(2, 6, 50000)]
    assert units.choosing is None  # the allowance ends with the command


def test_lane_active_whose_arm_is_refused_leaves_no_allowance_behind() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    mcus["b"].responses["ace2k_feed_start"] = {"lane": 2, "accepted": 0, "reason": 5}
    with pytest.raises(Exception, match="not made active"):
        printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "b", "LANE": "3"}))
    units = printer.objects["ace2k_units"]
    assert units.choosing is None and not units.may_follow(b, 2)


def lane_active_with_a_failing_stop(error: BaseException) -> TwoUnits:
    options = {
        "a": {"lane1_extruder": "extruder", "lane2_extruder": "extruder"},
        "b": {"lane3_extruder": "extruder"},
    }
    inserts = {"a": (True, True, False, False), "b": (False, False, True, False)}
    (a, b), printer, mcus = rig_units(options=options, inserts=inserts)
    report_units(mcus)
    report(mcus["a"], modes=(FOLLOWING, FOLLOWING, IDLE, IDLE))
    stop = a.feed.stop

    def failing_stop(lane: int, by: str = "host") -> None:
        if lane == 1:
            raise error
        stop(lane, by)

    a.feed.stop = failing_stop
    return (a, b), printer, mcus


def test_lane_active_an_internal_error_in_another_units_stop_is_re_raised_unchanged() -> None:
    boom = RuntimeError("link down")
    (a, b), printer, mcus = lane_active_with_a_failing_stop(boom)
    units = printer.objects["ace2k_units"]
    units.active["extruder"] = units.key(a, 0)
    with pytest.raises(RuntimeError) as raised:
        printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](
            KlipperGcmd({"UNIT": "b", "LANE": "3"})
        )
    assert raised.value is boom
    assert all(m.final_waiters == [] for m in a.feed.motion)  # no waiter left behind
    assert started(mcus["b"]) == []  # nothing armed
    assert units.get_status(0)["active_lanes"] == {"extruder": "a:1"}  # the choice kept


def test_hand_over_stops_the_other_units_follower_and_arms_nothing() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = a.feed.stop
    a.feed.stop = lambda lane, by="host": (
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    units = printer.objects["ace2k_units"]
    units.hand_over(b, 2, 5.0)
    assert a.feed.lanes[0]["last_event"]["stop_by"] == "command"
    assert started(mcus["b"]) == []  # nothing armed: the caller pushes, then arms
    assert units.get_status(0)["active_lanes"] == {"extruder": "b:3"}
    assert units.may_follow(b, 2) and not units.may_follow(a, 0)


def spy_forget(
    units_list: Iterable[Ace2k],
    mcus: dict[str, FakeMcu] | None = None,
    armed_in: str | None = None,
) -> list[tuple[str, str, int | None]]:
    """Record each unit's FeedForward.forget(name) as (unit section, name, starts already sent on
    armed_in's MCU), the real forget still run."""
    calls = []
    for unit in units_list:
        real = unit.feed.ff.forget

        def forget(name: str, unit: Ace2k = unit, real: Callable[[str], None] = real) -> None:
            starts = len(started(mcus[armed_in])) if armed_in else None
            calls.append((unit.section, name, starts))
            real(name)

        unit.feed.ff.forget = forget
    return calls


def test_hand_over_to_another_lane_makes_every_units_feed_forward_forget_the_extruder() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = a.feed.stop
    a.feed.stop = lambda lane, by="host": (
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    calls = spy_forget((a, b))
    units = printer.objects["ace2k_units"]
    units.hand_over(b, 2, 5.0)
    assert [(section, name) for section, name, _ in calls] == [
        ("ace2k a", "extruder"),
        ("ace2k b", "extruder"),
    ]
    calls.clear()
    units.hand_over(b, 2, 5.0)  # the lane already chosen: nothing changes hands
    assert calls == []


def test_release_makes_every_units_feed_forward_forget_the_extruder() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = a.feed.stop
    a.feed.stop = lambda lane, by="host": (
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    calls = spy_forget((a, b))
    units = printer.objects["ace2k_units"]
    units.release("extruder", 5.0)
    assert [name for _s, name, _ in calls] == ["extruder", "extruder"]
    calls.clear()
    units.release("extruder", 5.0)  # nothing chosen: nothing to forget
    assert calls == []


def test_release_with_a_follower_that_does_not_end_forgets_nothing() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    calls = spy_forget((a, b))
    units = printer.objects["ace2k_units"]
    with pytest.raises(RuntimeError, match="the choice is kept"):
        units.release("extruder", 5.0)
    assert calls == []  # the choice back on a:1, its account untouched


def test_release_forgets_only_after_the_followers_ended() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    calls = spy_forget((a, b))
    stop = a.feed.stop
    seen = []
    a.feed.stop = lambda lane, by="host": (
        seen.append(len(calls)),  # nothing forgotten while it winds down
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    printer.objects["ace2k_units"].release("extruder", 5.0)
    assert seen == [0] and len(calls) == 2


def test_the_active_lane_mapped_to_another_extruder_makes_the_feed_forward_forget() -> None:
    (a, b), printer, mcus = rig_units(inserts=INSERTS_ACROSS)  # mapped at run time, no keys
    report_units(mcus)
    a.feed.set_lane_extruder(0, "extruder")
    b.feed.set_lane_extruder(2, "extruder")
    units = printer.objects["ace2k_units"]
    units.groups()  # the coordinator's walk sees the group
    units.active["extruder"] = units.key(a, 0)
    calls = spy_forget((a, b))
    # the other lane leaves: the group dissolves and its choice with it, but a:1, still on the
    # extruder, keeps its account
    b.feed.set_lane_extruder(2, "extruder1")
    assert calls == []
    b.feed.set_lane_extruder(2, "extruder")
    units.active["extruder"] = units.key(a, 0)
    a.feed.set_lane_extruder(0, "extruder1")  # the active lane leaves: b:3 left alone
    assert [name for _s, name, _ in calls if name == "extruder"] == ["extruder", "extruder"]


def test_lane_active_on_another_lane_forgets_before_it_arms_the_new_one() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    stop = a.feed.stop
    a.feed.stop = lambda lane, by="host": (
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    calls = spy_forget((a, b), mcus, armed_in="b")
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "b", "LANE": "3"}))
    assert [(name, starts) for _s, name, starts in calls] == [("extruder", 0), ("extruder", 0)]
    assert len(started(mcus["b"])) == 1  # then b:3 armed


def test_hand_over_with_a_follower_that_does_not_end_changes_nothing() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    units = printer.objects["ace2k_units"]
    with pytest.raises(RuntimeError, match="lane a:1 stopped but did not end"):
        units.hand_over(b, 2, 5.0)
    assert units.get_status(0)["active_lanes"] == {"extruder": "a:1"}
    assert all(not m.final_waiters for m in a.feed.motion)


def test_hand_over_of_a_lane_with_its_own_extruder_does_nothing() -> None:
    (a, b), printer, mcus = rig_units(options={"a": {"lane1_extruder": "extruder"}})
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    before = units.get_status(0)["active_lanes"]
    units.hand_over(a, 0, 5.0)
    assert units.get_status(0)["active_lanes"] == before
    assert started(mcus["a"]) == [] and started(mcus["b"]) == []


def test_release_stops_the_active_lanes_follow_and_forgets_the_choice() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    units = printer.objects["ace2k_units"]
    stop = a.feed.stop
    eligible_while_stopping = []
    a.feed.stop = lambda lane, by="host": (
        # the choice is gone before the stops: nothing re-arms the lane while it winds down
        eligible_while_stopping.append(units.may_follow(a, lane)),
        stop(lane, by),
        deliver(mcus["a"], lane, 1, FOLLOWING, 1),
    )
    gcode.output.clear()
    units.release("extruder", 5.0)
    assert eligible_while_stopping == [False]
    assert a.feed.lanes[0]["last_event"]["stop_by"] == "host"  # a host module's stop
    assert units.get_status(0)["active_lanes"] == {"extruder": None}
    assert not units.may_follow(a, 0) and not units.may_follow(b, 2)
    # a group with no active lane again: its "choose one" line is said again at the next walk
    units.say_groups()
    assert [m for m in gcode.output if "share" in m] == [
        "ace2k a: lanes a:1, b:3 share extruder: choose one with ACEK_LANE_ACTIVE"
    ]


def test_release_with_a_follower_that_does_not_end_keeps_the_choice() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    printer.objects["gcode"].commands["ACEK_LANE_ACTIVE"](FakeGcmd({"UNIT": "a", "LANE": "1"}))
    report(mcus["a"], modes=(FOLLOWING, IDLE, IDLE, IDLE))
    units = printer.objects["ace2k_units"]
    with pytest.raises(
        RuntimeError, match="extruder not released: lane a:1 stopped but did not end"
    ):
        units.release("extruder", 5.0)
    assert units.get_status(0)["active_lanes"] == {"extruder": "a:1"}
    assert all(not m.final_waiters for m in a.feed.motion)


def test_release_while_acek_lane_active_arms_a_lane_of_the_group_is_refused() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    units.hand_over(b, 2, 5.0)
    units.choosing = units.key(a, 0)  # ACEK_LANE_ACTIVE LANE=1 UNIT=a waits for its start's answer
    with pytest.raises(RuntimeError, match="ACEK_LANE_ACTIVE is arming lane a:1; nothing done"):
        units.release("extruder", 5.0)
    assert units.get_status(0)["active_lanes"] == {"extruder": "b:3"}


def test_release_leaves_a_one_way_assist_running() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    units.hand_over(a, 0, 5.0)
    report(mcus["b"], modes=(IDLE, IDLE, 5, IDLE))  # b:3 in a one-way assist, not the choice's
    before = [list(m.commands) for m in mcus.values()]
    units.release("extruder", 5.0)
    assert [list(m.commands) for m in mcus.values()] == before
    assert units.get_status(0)["active_lanes"] == {"extruder": None}


def test_release_with_nothing_following_forgets_the_choice_and_sends_no_stop() -> None:
    (a, b), printer, mcus = rig_units(options=SHARED_ACROSS, inserts=INSERTS_ACROSS)
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    units.hand_over(b, 2, 5.0)
    assert units.get_status(0)["active_lanes"] == {"extruder": "b:3"}
    before = [list(m.commands) for m in mcus.values()]
    units.release("extruder", 5.0)
    assert units.get_status(0)["active_lanes"] == {"extruder": None}
    assert [list(m.commands) for m in mcus.values()] == before


def test_release_of_an_extruder_no_lanes_share_does_nothing() -> None:
    (a, b), printer, mcus = rig_units(options={"a": {"lane1_extruder": "extruder"}})
    report_units(mcus)
    units = printer.objects["ace2k_units"]
    before = units.get_status(0)["active_lanes"]
    units.release("extruder", 5.0)
    units.release("nozzle_that_is_not_there", 5.0)
    assert units.get_status(0)["active_lanes"] == before
    assert started(mcus["a"]) == [] and started(mcus["b"]) == []


def make_ff_units(
    options: dict[str, dict[str, Any]], names: Sequence[str] = ("a", "b")
) -> tuple[tuple[Ace2k, ...], FakePrinter, dict[str, FakeMcu], MoveExtruder]:
    extruder = MoveExtruder()
    clock = FakeClock()
    units, printer, mcus = make_units(
        names=names,
        options=options,
        constants=WITH_FF,
        extra={"mcu": clock, "extruder": extruder},
    )
    printer.shutdown = False
    printer.is_shutdown = lambda: printer.shutdown
    for unit in units:
        unit.feed._seed([0] * 4)
    for mcu in mcus.values():
        mcu.config_callback()
    fire_ready(printer)
    return tuple(units), printer, mcus, extruder


def rates(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt == BASE_FMT]


SHARED_FF = {"a": {"lane1_extruder": "extruder"}, "b": {"lane3_extruder": "extruder"}}


def test_two_followers_across_units_get_no_rate_and_it_is_said_once() -> None:
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    before = len(printer.objects["gcode"].output)
    for t in (10.2, 10.3):
        a.feed.ff._tick(t)
        b.feed.ff._tick(t)
    assert rates(mcus["a"]) == [[0, 0, 1]] and rates(mcus["b"]) == [[2, 0, 1]]
    said = printer.objects["gcode"].output[before:]
    assert said == ["ace2k a: extruder extruder: lanes a:1, b:3 both in follow — no feed-forward"]


def test_feed_forward_on_names_the_unit_and_says_nothing_with_a_follower_on_another_unit() -> None:
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    a.lanes[0]["insert"] = b.lanes[2]["insert"] = True
    set_modes(mcus["a"], [0, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, 0, 0])
    gcmd = FakeGcmd({"UNIT": "b", "LANE": "3", "ON": "1"})
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](gcmd)
    assert gcmd.lines[0].startswith("ace2k b: feed-forward lane 1 on (unmapped, idle)")
    assert gcmd.lines[1:] == [
        "ace2k b: lane b:3 is not in the follow: its feed-forward doses start once it follows"
        " — ACEK_LANE_ACTIVE LANE=3 UNIT=b arms it"
    ]
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    b.feed.ff._tick(10.2)
    gcmd = FakeGcmd({"UNIT": "a", "LANE": "1", "ON": "1"})
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](gcmd)
    assert gcmd.lines[1:] == []  # b:3 follows that extruder
    set_modes(mcus["a"], [7, 0, 0, 0])  # a:1 in error, b:3 still following: nothing said
    gcmd = FakeGcmd({"UNIT": "a", "LANE": "1", "ON": "1"})
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](gcmd)
    assert gcmd.lines[1:] == []
    set_modes(mcus["b"], [0, 0, 0, 0])
    gcmd = FakeGcmd({"UNIT": "a", "LANE": "1", "ON": "1"})
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](gcmd)
    assert gcmd.lines[1:] == [
        "ace2k a: lane a:1 is not in the follow: its feed-forward doses start once it follows"
        " — ACEK_CLEAR LANE=1 UNIT=a, then ACEK_LANE_ACTIVE LANE=1 UNIT=a arms it"
    ]


def test_the_one_follower_on_another_unit_gets_the_rate_and_this_unit_sends_nothing() -> None:
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    a.feed.ff._tick(10.2)
    b.feed.ff._tick(10.2)
    assert rates(mcus["a"]) == []
    assert rates(mcus["b"]) and rates(mcus["b"])[-1][0] == 2 and rates(mcus["b"])[-1][1] > 0


def test_a_tick_scans_the_other_units_followers_once_per_extruder() -> None:
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    coordinator_ = printer.objects["ace2k_units"]
    scans = []
    real = coordinator_.followers_elsewhere

    def counted(name: str, unit: Ace2k | None) -> list[Key]:
        scans.append((name, unit))
        return real(name, unit)

    coordinator_.followers_elsewhere = counted
    b.feed.ff._tick(10.2)
    assert scans == [("extruder", b)]
    assert rates(mcus["b"]) and rates(mcus["b"])[-1][1] > 0


def test_clear_shared_ends_the_conflict_at_one_follower_at_most() -> None:
    units, printer, _ = coordinator(["a", "b"])
    for followers, said in ((2, True), (1, False), (0, False)):
        units.shared_said = {"extruder"}
        units.clear_shared("extruder", followers)
        assert ("extruder" in units.shared_said) is said


SHARED_THREE = {
    "a": {"lane1_extruder": "extruder"},
    "b": {"lane3_extruder": "extruder"},
    "c": {"lane2_extruder": "extruder"},
}


def test_three_followers_across_units_are_said_with_all_and_a_unit_in_the_conflict() -> None:
    (a, b, c), printer, mcus, extruder = make_ff_units(SHARED_THREE, names=("a", "b", "c"))
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    set_modes(mcus["c"], [0, FF_FOLLOWING, 0, 0])
    before = len(printer.objects["gcode"].output)
    for unit in (c, b, a):
        unit.feed.ff._tick(10.2)
    said = printer.objects["gcode"].output[before:]
    assert said == [
        "ace2k a: extruder extruder: lanes a:1, b:3, c:2 all in follow — no feed-forward"
    ]


def test_a_unit_without_a_follower_sends_nothing_and_the_line_names_a_unit_in_the_conflict() -> (
    None
):
    (a, b, c), printer, mcus, extruder = make_ff_units(SHARED_THREE, names=("a", "b", "c"))
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    set_modes(mcus["c"], [0, FF_FOLLOWING, 0, 0])
    before = len(printer.objects["gcode"].output)
    sent_a = list(mcus["a"].commands)
    for unit in (a, b, c):
        unit.feed.ff._tick(10.2)
    assert mcus["a"].commands == sent_a
    assert rates(mcus["a"]) == []
    said = printer.objects["gcode"].output[before:]
    assert said == ["ace2k b: extruder extruder: lanes b:3, c:2 both in follow — no feed-forward"]


def test_followers_all_on_one_unit_are_said_once_by_that_unit_alone() -> None:
    options = {
        "a": {"lane1_extruder": "extruder", "lane2_extruder": "extruder"},
        "b": {"lane3_extruder": "extruder"},
    }
    (a, b), printer, mcus, extruder = make_ff_units(options)
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcus["a"], [FF_FOLLOWING, FF_FOLLOWING, 0, 0])
    before = len(printer.objects["gcode"].output)
    for unit in (b, a, b, a):
        unit.feed.ff._tick(10.2)
    said = printer.objects["gcode"].output[before:]
    assert said == ["ace2k a: extruder extruder: lanes 1 and 2 both in follow — no feed-forward"]
    assert rates(mcus["b"]) == []


def test_a_unit_without_a_follower_in_a_conflict_marks_nothing_and_its_own_episode_clears() -> None:
    options = {
        "a": {"lane1_extruder": "extruder", "lane2_extruder": "extruder"},
        "b": {"lane3_extruder": "extruder", "lane4_extruder": "extruder"},
    }
    (a, b), printer, mcus, extruder = make_ff_units(options)
    extrude(extruder, 10.0, 6.0, 3.0)
    out = printer.objects["gcode"].output
    # b has two followers, a's lanes on the extruder are idle: a sends and says nothing
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, FF_FOLLOWING])
    before = len(out)
    sent_a = list(mcus["a"].commands)
    for t in (10.2, 10.4):
        a.feed.ff._tick(t)
        b.feed.ff._tick(t)
    assert mcus["a"].commands == sent_a
    assert out[before:] == [
        "ace2k b: extruder extruder: lanes 3 and 4 both in follow — no feed-forward"
    ]
    # b's lanes leave the follow and a's two become the followers: a's own episode starts —
    # what each of its lanes owes is cleared, and its own line is said
    set_modes(mcus["b"], [0, 0, 0, 0])
    set_modes(mcus["a"], [FF_FOLLOWING, FF_FOLLOWING, 0, 0])
    before = len(out)
    b.feed.ff._tick(10.6)
    a.feed.ff._tick(10.6)
    assert rates(mcus["a"]) == [[0, 0, 1], [1, 0, 1]]
    assert out[before:] == [
        "ace2k a: extruder extruder: lanes 1 and 2 both in follow — no feed-forward"
    ]


def test_the_ambiguity_ending_re_arms_the_line_and_the_remaining_follower_resumes() -> None:
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 4.0, 3.0)
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    out = printer.objects["gcode"].output
    before = len(out)
    a.feed.ff._tick(10.2)
    b.feed.ff._tick(10.2)
    assert len(out) == before + 1
    # a leaves the follow: b is the one follower and dosed again
    set_modes(mcus["a"], [0, 0, 0, 0])
    a.feed.ff._tick(10.4)
    b.feed.ff._tick(10.4)
    b.feed.ff._tick(10.6)
    assert rates(mcus["b"])[-1][0] == 2 and rates(mcus["b"])[-1][1] > 0
    assert len(out) == before + 1
    # a follows again: the conflict is said again
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    a.feed.ff._tick(10.8)
    b.feed.ff._tick(10.8)
    assert len(out) == before + 2


def report_seqs(mcu: FakeMcu, modes: Sequence[int], seqs: Sequence[int]) -> None:
    mcu.subscriptions["ace2k_feed_state"](
        state_frame(list(modes), [0] * 4, [0] * 4, [0] * 4, [0] * 4, seqs)
    )


def test_a_follow_whose_final_event_came_is_no_follower_while_the_report_still_says_following() -> (
    None
):
    # lane a:1's follow stopped (its final event in), ACEK_LANE_ACTIVE armed b:3, and
    # a's last report still says following — no conflict line, b:3 gets its doses
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 4.0, 3.0)
    out = printer.objects["gcode"].output
    report_seqs(mcus["a"], [FF_FOLLOWING, 0, 0, 0], [1, 0, 0, 0])
    a.feed.ff._tick(10.2)
    b.feed.ff._tick(10.2)
    mcus["a"].subscriptions["ace2k_feed_event"](event_params(0, 1, FF_FOLLOWING, 7600, 7400, seq=1))
    assert a.feed.lanes[0]["mode"] == "following"  # the report has not caught up
    report_seqs(mcus["b"], [0, 0, FF_FOLLOWING, 0], [0, 0, 1, 0])
    before = len(out)
    for t in (10.4, 10.6):
        a.feed.ff._tick(t)
        b.feed.ff._tick(t)
    assert [m for m in out[before:] if "in follow" in m] == []
    assert rates(mcus["b"]) and rates(mcus["b"])[-1][0] == 2 and rates(mcus["b"])[-1][1] > 0
    # the next report of a shows the lane idle: nothing changes
    report_seqs(mcus["a"], [0, 0, 0, 0], [1, 0, 0, 0])
    b.feed.ff._tick(10.8)
    assert [m for m in out[before:] if "in follow" in m] == []
    assert rates(mcus["b"])[-1][1] > 0


def test_a_follow_still_running_is_a_follower_and_the_conflict_is_said() -> None:
    # the same sequence without the final event: a:1 still follows, the conflict is real
    (a, b), printer, mcus, extruder = make_ff_units(SHARED_FF)
    extrude(extruder, 10.0, 4.0, 3.0)
    out = printer.objects["gcode"].output
    report_seqs(mcus["a"], [FF_FOLLOWING, 0, 0, 0], [1, 0, 0, 0])
    report_seqs(mcus["b"], [0, 0, FF_FOLLOWING, 0], [0, 0, 1, 0])
    before = len(out)
    a.feed.ff._tick(10.4)
    b.feed.ff._tick(10.4)
    assert out[before:] == [
        "ace2k a: extruder extruder: lanes a:1, b:3 both in follow — no feed-forward"
    ]


def test_one_unit_leaving_a_conflict_that_goes_on_does_not_re_arm_the_line() -> None:
    (a, b, c), printer, mcus, extruder = make_ff_units(SHARED_THREE, names=("a", "b", "c"))
    extrude(extruder, 10.0, 6.0, 3.0)
    out = printer.objects["gcode"].output
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    set_modes(mcus["c"], [0, FF_FOLLOWING, 0, 0])
    before = len(out)
    for unit in (a, b, c):
        unit.feed.ff._tick(10.2)
    assert len(out) == before + 1
    # b leaves the follow, a and c still conflict: b's tick ends nothing, whatever the order
    set_modes(mcus["b"], [0, 0, 0, 0])
    for t in (10.4, 10.6):
        for unit in (b, a, c):
            unit.feed.ff._tick(t)
    assert len(out) == before + 1
    # b follows again: still the same conflict, never said twice
    set_modes(mcus["b"], [0, 0, FF_FOLLOWING, 0])
    for unit in (b, c, a):
        unit.feed.ff._tick(10.8)
    assert len(out) == before + 1
    # the conflict ends (c alone in the follow), seen by b only, a unit with no follower left
    set_modes(mcus["a"], [0, 0, 0, 0])
    set_modes(mcus["b"], [0, 0, 0, 0])
    b.feed.ff._tick(11.0)
    assert len(out) == before + 1
    # a new conflict, a and c: said again
    set_modes(mcus["a"], [FF_FOLLOWING, 0, 0, 0])
    for unit in (a, c):
        unit.feed.ff._tick(11.2)
    assert out[before + 1 :] == [
        "ace2k a: extruder extruder: lanes a:1, c:2 both in follow — no feed-forward"
    ]


def test_feed_events_name_their_unit_and_rfid_ignores_another_units() -> None:
    (a, b), printer, mcus = make_units()
    for unit in (a, b):
        unit.feed._seed([0] * 4)
    for mcu in mcus.values():
        mcu.config_callback()
    mcus["b"].subscriptions["ace2k_feed_event"](event_params(2, 0, 1, 1000, 1000))
    name, (lane, event) = [e for e in printer.events if e[0] == "ace2k:feed_event"][-1]
    assert lane == 2 and event["unit"] == "b"
    before = dict(a.rfid.lanes[2])
    a.rfid._feed_event(2, dict(event))
    assert a.rfid.lanes[2] == before


def test_the_unnamed_units_events_carry_none_and_its_cache_keeps_its_name() -> None:
    module, printer, mcu = make(dict(RESPONSES))
    module.feed._seed([0] * 4)
    mcu.config_callback()
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 1000, 1000))
    event = [e for e in printer.events if e[0] == "ace2k:feed_event"][-1][1][1]
    assert event["unit"] is None
    assert module.rfid.cache_path.endswith("ace2k_tags_cache.json")


def test_a_named_units_tag_cache_is_its_own() -> None:
    (a, b), printer, mcus = make_units()
    assert a.rfid.cache_path.endswith("ace2k_tags_cache_a.json")
    assert b.rfid.cache_path.endswith("ace2k_tags_cache_b.json")


def test_the_first_named_unit_takes_the_legacy_tag_cache_once_and_the_second_does_not() -> None:
    (a, b), printer, mcus = make_units()
    legacy = os.path.join(a.rfid.config_dir, "ace2k_tags_cache.json")
    with open(legacy, "w") as f:
        json.dump({"0A0B0C0D": {"format": "uid", "uid": "0A0B0C0D"}}, f)
    assert b.rfid._load_cache() == {}  # the second unit never reads it
    assert a.rfid._load_cache() == {"0A0B0C0D": {"format": "uid", "uid": "0A0B0C0D"}}
    # from then on the first unit writes and reads its own file; the legacy one is set aside
    a.rfid.cache = {"01020304": {"format": "uid", "uid": "01020304"}}
    a.rfid._save_cache()
    assert a.rfid._load_cache() == {"01020304": {"format": "uid", "uid": "01020304"}}
    assert not os.path.exists(legacy)


def test_the_legacy_tag_cache_is_adopted_once_and_written_to_the_units_own_file() -> None:
    (a, b), printer, mcus = make_units()
    legacy = os.path.join(a.rfid.config_dir, "ace2k_tags_cache.json")
    record = {"0A0B0C0D": {"format": "uid", "uid": "0A0B0C0D"}}
    with open(legacy, "w") as f:
        json.dump(record, f)
    assert not os.path.exists(a.rfid.cache_path)
    assert a.rfid._load_cache() == record  # the start's load adopts it
    with open(a.rfid.cache_path) as f:
        assert json.load(f) == record  # at once: the unit's own file holds it
    # the legacy file is set aside, its data kept, never read again
    assert not os.path.exists(legacy)
    with open(legacy + ".adopted") as f:
        assert json.load(f) == record
    assert a.rfid._load_cache() == record


def test_the_legacy_tag_cache_is_adopted_once_ever_even_when_the_units_own_file_goes() -> None:
    (a, b), printer, mcus = make_units()
    legacy = os.path.join(a.rfid.config_dir, "ace2k_tags_cache.json")
    record = {"0A0B0C0D": {"format": "uid", "uid": "0A0B0C0D"}}
    with open(legacy, "w") as f:
        json.dump(record, f)
    assert a.rfid._load_cache() == record
    assert not os.path.exists(legacy) and os.path.exists(legacy + ".adopted")
    os.remove(a.rfid.cache_path)
    assert a.rfid._load_cache() == {}  # nothing adopted again
    assert not os.path.exists(a.rfid.cache_path)


def test_the_legacy_tag_cache_stays_in_place_when_the_units_own_file_cannot_be_written() -> None:
    (a, b), printer, mcus = make_units()
    legacy = os.path.join(a.rfid.config_dir, "ace2k_tags_cache.json")
    record = {"0A0B0C0D": {"format": "uid", "uid": "0A0B0C0D"}}
    with open(legacy, "w") as f:
        json.dump(record, f)
    a.rfid.cache_path = os.path.join(a.rfid.config_dir, "no-such-dir", "own.json")
    assert a.rfid._load_cache() == record
    assert os.path.exists(legacy)  # not adopted: the next start tries again
    assert not os.path.exists(legacy + ".adopted")


def test_groups_are_built_once_and_a_remap_after_a_lookup_is_seen() -> None:
    (a, b), printer, mcus = make_units(options=SHARED_ACROSS)
    printer.is_shutdown = lambda: False
    units = printer.objects["ace2k_units"]
    assert units.groups() == {"extruder": [(0, 0), (1, 2)]}
    reads = []
    for unit in (a, b):
        of = unit.feed.ff.extruder_of
        unit.feed.ff.extruder_of = lambda lane, of=of: reads.append(lane) or of(lane)
    units.groups()
    units.groups()
    assert reads == []  # a lookup reuses the map
    b.feed.set_lane_extruder(1, "extruder")
    assert units.groups() == {"extruder": [(0, 0), (1, 1), (1, 2)]}
    b.feed.set_lane_extruder(1, None)
    assert units.groups() == {"extruder": [(0, 0), (1, 2)]}


def test_a_shared_group_whose_lanes_change_is_announced_again_and_an_unchanged_one_is_not() -> None:
    options = dict(SHARED_ACROSS, b={"lane3_extruder": "extruder", "lane4_extruder": "extruder1"})
    options["a"] = {"lane1_extruder": "extruder", "lane2_extruder": "extruder1"}
    (a, b), printer, mcus = rig_units(options=options, inserts=INSERTS_ACROSS)
    report_units(mcus)
    assert shared_lines(printer) == [
        "ace2k a: lanes a:1, b:3 share extruder: choose one with ACEK_LANE_ACTIVE",
        "ace2k a: lanes a:2, b:4 share extruder1: choose one with ACEK_LANE_ACTIVE",
    ]
    b.feed.ff.runtime[1] = "extruder"  # b:2 joins extruder's group (as set_lane_extruder maps)
    printer.objects["ace2k_units"].map_changed()
    report_units(mcus)
    report_units(mcus)
    assert shared_lines(printer)[2:] == [
        "ace2k a: lanes a:1, b:2, b:3 share extruder: choose one with ACEK_LANE_ACTIVE"
    ]


def test_the_groups_line_walk_runs_only_after_the_map_changes_and_a_remap_still_says_it() -> None:
    options = {"a": {"lane1_extruder": "extruder"}, "b": {"lane3_extruder": "extruder1"}}
    (a, b), printer, mcus = rig_units(options=options, inserts=INSERTS_ACROSS)
    units = printer.objects["ace2k_units"]
    report_units(mcus)  # the first ticks: the walk runs once, nothing shared
    walks = []
    walk = units._walk_groups
    units._walk_groups = lambda: walks.append(1) or walk()
    for _ in range(3):
        report_units(mcus)
    assert walks == [] and shared_lines(printer) == []  # nothing changed: no walk
    b.feed.set_lane_extruder(1, "extruder")  # a remap: a:1 and b:2 now share extruder
    report_units(mcus)
    assert shared_lines(printer) == [
        "ace2k a: lanes a:1, b:2 share extruder: choose one with ACEK_LANE_ACTIVE"
    ]
    count = len(walks)
    report_units(mcus)
    report_units(mcus)
    assert len(walks) == count  # said once; no walk until the map changes again
    assert len(shared_lines(printer)) == 1


def test_a_remap_drops_the_active_lane_only_when_it_left_its_changed_group() -> None:
    options = {
        "a": {"lane1_extruder": "extruder", "lane2_extruder": "extruder1"},
        "b": {"lane4_extruder": "extruder1"},
    }
    (a, b), printer, mcus = make_units(options=options)
    printer.is_shutdown = lambda: False
    units = printer.objects["ace2k_units"]
    b.feed.set_lane_extruder(2, "extruder")
    units.choose(b, 2)
    units.choose(a, 1)
    assert units.active_map() == {"extruder": "b:3", "extruder1": "a:2"}
    # b:2 joins: the group changed but its active lane is still in it
    b.feed.set_lane_extruder(1, "extruder")
    assert units.active_map() == {"extruder": "b:3", "extruder1": "a:2"}
    # b:3 leaves: the choice goes; mapped back, it is not active again until chosen
    b.feed.set_lane_extruder(2, None)
    assert units.active_map() == {"extruder": None, "extruder1": "a:2"}
    b.feed.set_lane_extruder(2, "extruder")
    assert units.active_map() == {"extruder": None, "extruder1": "a:2"}
    assert not units.eligible(b, 2)
    # a group that dissolves and comes back is a change too
    b.feed.set_lane_extruder(1, None)
    units.choose(b, 2)
    b.feed.set_lane_extruder(2, None)  # extruder: a:1 alone, no group
    b.feed.set_lane_extruder(2, "extruder")
    assert units.active_map() == {"extruder": None, "extruder1": "a:2"}


def test_the_host_interface_version() -> None:
    assert ace2k_feed.API_VERSION == 29


# --- the commands the core prints for a lane ----------------------------------------------------


def test_the_calibration_hints_name_the_unit_with_several_units() -> None:
    (a, b), printer, mcus = make_units()
    b.feed._seed([0] * 4)
    mcus["b"].config_callback()
    mcus["b"].responses["ace2k_feed_start"] = accepted(
        mcus["b"], event_params(0, 0, 1, 200000, 200000, seq=1)
    )
    calibrate = printer.objects["gcode"].commands["ACEK_CALIBRATE_ENCODER"]
    gcmd = FakeGcmd({"UNIT": "b", "LANE": "1", "LENGTH": "200"})
    calibrate(gcmd)
    assert gcmd.lines[-1].endswith("then ACEK_CALIBRATE_ENCODER LANE=1 UNIT=b MEASURED=<mm>")
    with pytest.raises(
        Exception, match="run ACEK_CALIBRATE_ENCODER LANE=2 UNIT=b LENGTH=<mm> first"
    ):
        calibrate(FakeGcmd({"UNIT": "b", "LANE": "2", "MEASURED": "100"}))


def test_the_lanes_command_arguments_carry_unit_only_with_several_units() -> None:
    (a, b), printer, mcus = make_units()
    units = printer.objects["ace2k_units"]
    assert units.lane_args(b, 2) == "LANE=3 UNIT=b"
    module, printer, mcu = make(dict(RESPONSES))
    assert printer.objects["ace2k_units"].lane_args(module, 0) == "LANE=1"


# --- every line a unit writes names it ---------------------------------------------------------


def dryer_units() -> TwoUnits:
    frames = [td.log_frame(0, a=2, b=1, e=1), td.log_frame(1), td.log_frame(0, a=2, b=1, e=1)]
    responses = dict(td.DRYER_RESPONSES, ace2k_dryer_log_query=frames)
    (a, b), printer, mcus = make_units(responses=responses)
    for mcu in mcus.values():
        mcu.config_callback()
    return (a, b), printer, mcus


def test_the_dryer_log_names_its_unit() -> None:
    (a, b), printer, mcus = dryer_units()
    gcmd = FakeGcmd({"UNIT": "a"})
    printer.objects["gcode"].commands["ACEK_DRYER_LOG"](gcmd)
    assert gcmd.lines[-1].startswith("ace2k a: dryer: 2 cycles started, 1 completed, 1 fault;")


def test_a_dryer_event_log_line_names_its_unit(caplog: pytest.LogCaptureFixture) -> None:
    (a, b), printer, mcus = dryer_units()
    mcus["b"].subscriptions["ace2k_dryer_state"](td.state_frame())  # the events' base
    with caplog.at_level("INFO", logger="root"):
        mcus["b"].subscriptions["ace2k_dryer_event"]({"kind": 2, "arg": 8, "seq": 0})
        mcus["b"].subscriptions["ace2k_dryer_event"]({"kind": 3, "arg": 2, "seq": 1})
    lines = [r.getMessage() for r in caplog.records if "dryer" in r.getMessage()]
    assert lines and all(line.startswith("ace2k b: dryer: ") for line in lines)
    assert any(line.startswith("ace2k b: dryer: fault cutout") for line in lines)
    assert any("lane b:2" in line for line in lines)  # the unit's lane, as a:2


def test_a_tag_log_line_names_its_unit_and_lane(caplog: pytest.LogCaptureFixture) -> None:
    responses = dict(
        RESPONSES,
        ace2k_rfid_read={"lane": 0, "accepted": 1, "reason": 0},
        ace2k_rfid_lane_query={"lane": 0, "state": 0, "uid": b""},
    )
    (a, b), printer, mcus = make_units(responses=responses)
    for name, params in tr.sections().items():
        printer.add_object(f"ace2k_tag {name}", tr.Tag(name, params))
    for unit, mcu in zip((a, b), mcus.values(), strict=True):
        mcu.config_callback()
        unit.rfid._on_connect()
    mcu = mcus["a"]
    with caplog.at_level("INFO", logger="root"):
        tr.tag_event(mcu, 1, 7, tr.UID7, 0x0044, 0x00)
        pages = tr.ntag_image(tr.anycubic_image())
        for first in (4, 16, 28):
            for group in range(3):
                p = first + 4 * group
                payload = b"".join(pages.get(p + i, bytes(4)) for i in range(4))
                tr.data(mcu, 1, 7, p, payload)
    tags = [r.getMessage() for r in caplog.records if " tag " in r.getMessage()]
    assert any(line.startswith("ace2k a: lane a:2 tag ") for line in tags)
    assert all(line.startswith("ace2k a:") for line in tags)


def test_a_read_with_motion_logs_its_search_with_its_unit_and_lane(
    caplog: pytest.LogCaptureFixture,
) -> None:
    responses = dict(
        RESPONSES,
        ace2k_rfid_read={"lane": 0, "accepted": 1, "reason": 0},
        ace2k_rfid_lane_query={"lane": 0, "state": 5, "uid": b""},
    )
    (a, b), printer, mcus = make_units(responses=responses)
    for unit, mcu in zip((a, b), mcus.values(), strict=True):
        mcu.config_callback()
        unit.rfid._on_connect()
    mcu = mcus["a"]
    tr.queue_calls(
        printer,
        tr.rfid_event(mcu, 0, 2),  # searching
        tr.feed_frame(a, mcu, printer, 0, 5, tr.SEARCH_MOTOR_UM, 0),
    )
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "UNIT": "a", "MOVE": "1"})
        )
    lines = tr.log_lines(caplog)
    assert "ace2k a: lane a:1 tag search: about 740.6 mm out and back, no tag" in lines
    assert not [line for line in lines if "loaded (loading)" in line]


def test_one_unit_keeps_its_lines_as_they_were(caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = make(dict(RESPONSES))
    assert module.head() == "ace2k:" and module.head("dryer") == "ace2k dryer:"
    assert module.lead("0.1.0 connected") == "ace2k 0.1.0 connected"
    assert module.own("health: OK") == "health: OK" and module.lane_text(1) == "2"
    (a, b), printer, mcus = make_units()
    assert a.head() == "ace2k a:" and a.head("dryer") == "ace2k a: dryer:"
    assert a.lead("0.1.0 connected") == "ace2k a: 0.1.0 connected"
    assert b.own("health: OK") == "ace2k b: health: OK" and b.lane_text(1) == "b:2"


# --- one FIRMWARE_RESTART for a board still stopped from an earlier shutdown --------------------


class StoppedMcu:
    """A Klipper MCU object as the U1's Klipper has it: is_shutdown() and _shutdown_msg."""

    def __init__(self, down: bool = False, reason: str = "") -> None:
        self.down, self._shutdown_msg = down, reason

    def is_shutdown(self) -> bool:
        return self.down


class Helper:
    """Today's Klipper: the stop lives on the MCU's connect helper."""

    def __init__(self, down: bool, reason: str) -> None:
        self.down, self.reason = down, reason

    def is_shutdown(self) -> bool:
        return self.down

    def get_shutdown_msg(self) -> str:
        return self.reason


def at_start(
    start_reason: str = "startup", boards: dict[str, tuple[bool, str]] | None = None
) -> tuple[ace2k_units.Units, FakePrinter]:
    """Two units at a start; boards: MCU name -> (stopped, reason), the printer's own included."""
    (a, b), printer, mcus = make_units()
    printer.get_start_args = lambda: {"start_reason": start_reason}
    printer.exits = []
    printer.request_exit = printer.exits.append
    for name, (down, reason) in (boards or {}).items():
        printer.objects[name] = StoppedMcu(down, reason)
    units = printer.lookup_object("ace2k_units")
    return units, printer


def first_shutdown(
    printer: FakePrinter, details: dict[str, str], event: str = ace2k_units.SHUTDOWN_EVENTS[0]
) -> None:
    """Klipper's first shutdown of the start, through the registered handlers."""
    for cb in printer.handler_lists.get(event, []):
        cb('{"coded": "0003-0522-0000-0002", "msg": "MCU shutdown"}', details)


def check(units: ace2k_units.Units, printer: FakePrinter) -> float | None:
    timers = [t for t in printer.reactor.armed() if t[0] == units._check_leftover]
    if not timers:
        return None
    (timer,) = timers
    return timer[0](timer[1])


LEFT = (True, "Command request")
LEFTOVER = {"reason": "Command request", "mcu": "e1"}


@pytest.mark.parametrize("event", ace2k_units.SHUTDOWN_EVENTS)
def test_boards_still_stopped_at_a_process_start_get_one_firmware_restart(
    event: str, caplog: pytest.LogCaptureFixture
) -> None:
    """M112, then the init script's restart: the first shutdown
    a board's "Command request", every board still stopped so; FIRMWARE_RESTART resolved it."""
    units, printer = at_start(boards={"mcu": LEFT, "mcu e1": LEFT, "mcu ace2k_a": LEFT})
    first_shutdown(printer, LEFTOVER, event)
    first_shutdown(printer, {"reason": "Command request", "mcu": "mcu"}, event)  # only the first
    (timer,) = [t for t in printer.reactor.armed() if t[0] == units._check_leftover]
    assert timer[1] == printer.reactor.now + ace2k_units.LEFTOVER_CHECK_S
    assert check(units, printer) == printer.reactor.NEVER
    assert printer.exits == ["firmware_restart"]
    (line,) = [r.getMessage() for r in caplog.records if "earlier shutdown" in r.getMessage()]
    listed = line.split("MCU ", 1)[1].split(" still stopped")[0].split(", ")
    assert sorted(listed) == ["ace2k_a", "e1", "mcu"]


def test_a_host_shutdown_first_is_this_starts_fault_even_with_every_board_then_stopped() -> None:
    """A lost link at this start: the host's stop leaves every board reporting Command request,
    but the first shutdown was the host's own — nothing."""
    units, printer = at_start(boards={"mcu": LEFT, "mcu ace2k_a": LEFT})
    first_shutdown(printer, {})  # "Lost communication with MCU 'ace2k_a'": no mcu, no reason
    first_shutdown(printer, LEFTOVER)
    assert check(units, printer) is None and printer.exits == []


def test_a_boards_fault_first_is_left_to_the_operator() -> None:
    units, printer = at_start(boards={"mcu": (True, "ADC out of range")})
    first_shutdown(printer, {"reason": "ADC out of range", "mcu": "mcu"})
    assert check(units, printer) is None and printer.exits == []


def test_a_fault_on_any_board_at_the_look_leaves_it_to_the_operator(
    caplog: pytest.LogCaptureFixture,
) -> None:
    units, printer = at_start(
        boards={
            "mcu": (True, "ADC out of range"),
            "mcu e0": (True, "Timer too close"),
            "mcu ace2k_a": LEFT,
        }
    )
    first_shutdown(printer, {"reason": "Command request", "mcu": "ace2k_a"})
    check(units, printer)
    assert printer.exits == []
    assert "MCU 'mcu' ('ADC out of range'), 'e0' ('Timer too close') stopped" in caplog.text


def test_a_reason_that_cannot_be_read_leaves_it_to_the_operator() -> None:
    units, printer = at_start(boards={"mcu ace2k_a": (True, None)})
    first_shutdown(printer, {"reason": "Command request", "mcu": "ace2k_a"})
    check(units, printer)
    assert printer.exits == []


def test_todays_klipper_keeps_the_stop_on_the_connect_helper() -> None:
    units, printer = at_start()
    printer.objects["mcu ace2k_b"] = types.SimpleNamespace(
        _conn_helper=Helper(True, "Command request")
    )
    first_shutdown(printer, {"reason": "Command request", "mcu": "ace2k_b"})
    check(units, printer)
    assert printer.exits == ["firmware_restart"]


@pytest.mark.parametrize("start_reason", ["firmware_restart", "restart"])
def test_no_look_at_a_start_a_restart_made(start_reason: str) -> None:
    units, printer = at_start(start_reason, boards={"mcu": LEFT})
    first_shutdown(printer, LEFTOVER)
    assert check(units, printer) is None and printer.exits == []


def test_nothing_once_klipper_is_ready_or_when_nothing_is_stopped_at_the_look() -> None:
    units, printer = at_start(boards={"mcu": (False, "")})
    first_shutdown(printer, LEFTOVER)
    check(units, printer)
    assert printer.exits == []
    units, printer = at_start(boards={"mcu": LEFT})
    for cb in printer.handler_lists["klippy:ready"]:
        if getattr(cb, "__self__", None) is units:
            cb()
    first_shutdown(printer, LEFTOVER)
    assert check(units, printer) is None and printer.exits == []


# --- the same board found stopped by the connect, before any shutdown notice ---------------------


def connect_error(printer: FakePrinter, error: str) -> None:
    """Klipper's "MCU error during connect", through the registered handlers."""
    for cb in printer.handler_lists.get(ace2k_units.MCU_ERROR_EVENT, []):
        cb("MCU error during connect", {"error": error})


def stopped_at_connect(name: str) -> str:
    return f"Can not update MCU '{name}' config as it is shutdown"


def test_a_unit_found_stopped_at_connect_gets_one_firmware_restart(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Bench 2026-10-18: M112, the init script's restart, then "Can not update MCU 'ace2k_a'
    config as it is shutdown" and no shutdown notice at all; FIRMWARE_RESTART resolved it."""
    units, printer = at_start(boards={"mcu": (False, "")})
    connect_error(printer, stopped_at_connect("ace2k_a"))
    connect_error(printer, stopped_at_connect("ace2k_a"))  # armed once
    (timer,) = [t for t in printer.reactor.armed() if t[0] == units._check_leftover]
    assert timer[1] == printer.reactor.now + ace2k_units.LEFTOVER_CHECK_S
    assert check(units, printer) == printer.reactor.NEVER
    assert printer.exits == ["firmware_restart"]
    assert "MCU ace2k_a still stopped from an earlier shutdown" in caplog.text
    assert "found at connect" in caplog.text


@pytest.mark.parametrize(
    "error",
    [
        stopped_at_connect("mcu"),  # the printer's own board
        stopped_at_connect("e1"),
        "MCU 'ace2k_a' CRC does not match config",
        "Failed automated reset of MCU 'ace2k_a'",
        "",
    ],
)
def test_no_restart_for_another_boards_or_another_connect_error(error: str) -> None:
    units, printer = at_start(boards={"mcu": (False, ""), "mcu e1": (False, "")})
    connect_error(printer, error)
    assert check(units, printer) is None and printer.exits == []


@pytest.mark.parametrize("start_reason", ["firmware_restart", "restart"])
def test_no_second_restart_when_the_unit_comes_back_stopped(start_reason: str) -> None:
    """The start that the one FIRMWARE_RESTART made meets the board stopped again: nothing."""
    units, printer = at_start(start_reason)
    connect_error(printer, stopped_at_connect("ace2k_b"))
    assert check(units, printer) is None and printer.exits == []


def test_a_fault_on_another_board_at_the_look_still_leaves_it_to_the_operator() -> None:
    units, printer = at_start(boards={"mcu": (True, "Timer too close")})
    connect_error(printer, stopped_at_connect("ace2k_a"))
    check(units, printer)
    assert printer.exits == []


def test_a_host_shutdown_first_also_vetoes_the_connect_path() -> None:
    units, printer = at_start()
    first_shutdown(printer, {})  # this start's own fault
    connect_error(printer, stopped_at_connect("ace2k_a"))
    assert check(units, printer) is None and printer.exits == []
    units, printer = at_start()
    connect_error(printer, stopped_at_connect("ace2k_a"))
    first_shutdown(printer, {})
    check(units, printer)
    assert printer.exits == []


def test_a_notice_after_the_connect_error_keeps_one_check() -> None:
    units, printer = at_start()
    connect_error(printer, stopped_at_connect("ace2k_a"))
    first_shutdown(printer, {"reason": "Command request", "mcu": "ace2k_b"})
    assert len([t for t in printer.reactor.armed() if t[0] == units._check_leftover]) == 1
    check(units, printer)
    assert printer.exits == ["firmware_restart"]
