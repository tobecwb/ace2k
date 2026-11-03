"""ace2k_link is a leaf: it imports no other ace2k module, so every module can import it."""

from __future__ import annotations

import ast
from pathlib import Path

import ace2k
import ace2k_encoder_cal
import ace2k_feed
import ace2k_feed_proto
import ace2k_ff
import ace2k_follow
import ace2k_link
import ace2k_rfid
import ace2k_units
import msgproto
import pytest
from test_ace2k_extras import RESPONSES, FakeMcu, FakePrinter

EXTRAS = Path(__file__).resolve().parents[3] / "klippy" / "extras"
LINK = EXTRAS / "ace2k_link.py"
PROTO = EXTRAS / "ace2k_feed_proto.py"
FEED_PROTO_FUNCTIONS = {"Move", "to_mm", "to_um", "scale_x10", "firmware_search_mm"}


def _sibling_imports(path: Path) -> list[str]:
    """Run-time imports of ace2k modules (TYPE_CHECKING blocks never execute)."""
    tree = ast.parse(path.read_text())
    runtime = [
        node
        for stmt in tree.body
        if not (isinstance(stmt, ast.If) and ast.unparse(stmt.test) == "TYPE_CHECKING")
        for node in ast.walk(stmt)
    ]
    names = [a.name for n in runtime if isinstance(n, ast.Import) for a in n.names]
    names += [n.module or "" for n in runtime if isinstance(n, ast.ImportFrom)]
    return [n.removeprefix(".") for n in names if n.lstrip(".").startswith("ace2k")]


def test_the_link_module_imports_no_sibling() -> None:
    assert not _sibling_imports(LINK)


def test_the_feed_proto_module_imports_only_the_link() -> None:
    assert {*_sibling_imports(PROTO)} == {"ace2k_link"}


def test_the_feed_names_still_import_from_ace2k_feed() -> None:
    for name in vars(ace2k_feed_proto):
        if name != "TYPE_CHECKING" and (name.isupper() or name in FEED_PROTO_FUNCTIONS):
            assert getattr(ace2k_feed, name) is getattr(ace2k_feed_proto, name)
    assert ace2k_feed_proto.FeedRefused is ace2k_link.FeedRefused
    kinds = ace2k_feed_proto.KIND_NAMES
    assert {kinds[k] for k in ace2k_feed_proto.ERROR_KINDS} == ace2k_feed_proto.ERROR_KIND_NAMES
    assert {kinds[k] for k in ace2k_feed_proto.NOTICE_KINDS} == ace2k_feed_proto.NOTICE_KIND_NAMES


def test_the_old_names_still_import_from_ace2k() -> None:
    for name in (
        "ask",
        "probe_format",
        "refuse_format",
        "subscribe_response",
        "whole",
        "NoAnswerError",
        "LANES",
        "LANE_ALL",
        "LANE_REPORT_S",
    ):
        assert getattr(ace2k, name) is getattr(ace2k_link, name)


def test_the_modules_that_used_to_copy_the_names_share_them() -> None:
    for module in (ace2k_feed, ace2k_ff, ace2k_follow, ace2k_rfid, ace2k_units):
        assert module.LANES is ace2k_link.LANES
    assert ace2k_feed.LANE_ALL is ace2k_follow.LANE_ALL is ace2k_link.LANE_ALL
    assert ace2k_encoder_cal.LANE_REPORT_S is ace2k_link.LANE_REPORT_S
    assert ace2k_feed.FeedRefused is ace2k_link.FeedRefused


def test_ace2k_has_no_function_level_import_helper() -> None:
    assert not hasattr(ace2k, "_modules")


TWO = 2.0


class Mcu:
    def __init__(self, constants: dict[str, float], error: type[Exception] = KeyError) -> None:
        self.constants, self.error = constants, error

    def get_constant_float(self, name: str) -> float:
        if name not in self.constants:
            raise self.error(name)
        return self.constants[name]


def test_constant_or_none_returns_the_constant_or_none() -> None:
    assert ace2k_link.constant_or_none(Mcu({"A": TWO}), "A") == TWO
    assert ace2k_link.constant_or_none(Mcu({}), "A") is None
    assert ace2k_link.constant_or_none(Mcu({}, msgproto.error), "A") is None


def test_constant_or_none_lets_other_errors_through() -> None:
    with pytest.raises(ZeroDivisionError):
        ace2k_link.constant_or_none(Mcu({}, ZeroDivisionError), "A")


def test_constants_or_none_is_all_or_nothing() -> None:
    assert ace2k_link.constants_or_none(Mcu({"A": 1, "B": 2}), ["A", "B"]) == {"A": 1.0, "B": 2.0}
    assert ace2k_link.constants_or_none(Mcu({"A": 1}), ["A", "B"]) is None


def test_check_lane_raises_the_given_message() -> None:
    ace2k_link.check_lane(3, "lane %d out of range")
    with pytest.raises(ValueError, match="lane 4 out of range"):
        ace2k_link.check_lane(4, "lane %d out of range")


@pytest.mark.parametrize("lane", [True, False])
def test_check_lane_refuses_a_bool(lane: bool) -> None:
    with pytest.raises(ValueError, match="no lane"):
        ace2k_link.check_lane(lane, "no lane %s")


@pytest.mark.parametrize("lane", [(0,), (1, 2)])
def test_check_lane_refuses_a_tuple_with_its_message(lane: object) -> None:
    """A tuple is no lane: the ValueError carries the message, not a formatting error."""
    with pytest.raises(ValueError, match=r"^no lane \(") as err:
        ace2k_link.check_lane(lane, "no lane %s")
    assert str(err.value) == f"no lane {lane}"


def test_bounded_constants_scales_the_value_and_its_bounds() -> None:
    mcu = Mcu({"A": 1000, "A_MIN": 500, "A_MAX": 2000})
    assert ace2k_link.bounded_constants(mcu, [("A", 1000.0)]) == {"A": (1.0, 0.5, 2.0)}
    assert ace2k_link.bounded_constants(Mcu({"A": 1, "A_MIN": 0}), [("A", 1.0)]) is None


def _optional_mcu(
    known: set[str] | None = None, formats: dict[str, str] | None = None
) -> tuple[FakePrinter, FakeMcu]:
    mcu = FakeMcu(RESPONSES, known=known, formats=formats)
    return FakePrinter(mcu), mcu


OPT_FMT = "ace2k_lane_park lane=%c"


def test_optional_command_builds_the_command_the_dictionary_has() -> None:
    printer, mcu = _optional_mcu()
    cmd = ace2k_link.optional_command(printer, mcu, "ace2k", OPT_FMT)
    assert cmd is not None
    assert ace2k_link.probe_optional(printer, mcu, "ace2k", OPT_FMT) is True


def test_optional_command_is_none_for_an_absent_name() -> None:
    printer, mcu = _optional_mcu(known=set())
    assert ace2k_link.optional_command(printer, mcu, "ace2k", OPT_FMT) is None
    assert ace2k_link.probe_optional(printer, mcu, "ace2k", OPT_FMT) is False


def test_optional_command_refuses_another_format_beside_the_given_one() -> None:
    printer, mcu = _optional_mcu(formats={"ace2k_lane_park": "ace2k_lane_park lane=%u"})
    with pytest.raises(Exception, match="ace2k_lane_park has another format on the unit"):
        ace2k_link.optional_command(printer, mcu, "ace2k", OPT_FMT, "ace2k_feed_start x=%c")
    with pytest.raises(Exception, match="another format"):
        ace2k_link.probe_optional(printer, mcu, "ace2k", OPT_FMT)
