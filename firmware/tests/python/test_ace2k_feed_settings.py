"""The settings of the tip snag, the follow and the feed-forward: the command each sends to the
unit and the line each prints, for the firmware's defaults and for values off them, captured from
the feed before its three copies of the setting code became one, and the refusals and the
SAVE_CONFIG staging of their G-codes, word for word."""

from __future__ import annotations

import re
from dataclasses import replace

import ace2k_feed
import ace2k_feed_settings
import ace2k_ff
import pytest
from ace2k_feed_settings import FF, FOLLOW, SNAG
from test_ace2k_feed import FakeGcmd, make_ff_feed
from test_ace2k_link import EXTRAS, _sibling_imports

SETTINGS = EXTRAS / "ace2k_feed_settings.py"

# printer.cfg keys -> what the feed sends and prints at connect, captured from the feed as it was
CONNECT_CASES = [
    (
        {},
        "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k: snag fwd 20 mm, lag 5 mm, duty 0 %, back 5 mm, rest 500 ms, free 50 mm",
        "ace2k_feed_follow_set flip_ms=200 take_um=15000 tail_um=2000000",
        "ace2k: follow flip 200 ms, take 15 mm, tail 2000 mm",
        "ace2k_feed_ff_set chunk_um=3000 pulse_um_s=15000",
        "ace2k: feed-forward chunk 3 mm, pulse 15 mm/s, window 500 ms, lead 0 ms,"
        " deadband 0.3 mm/s, long retraction 10 mm",
    ),
    (
        {
            "snag_fwd_mm": 25,
            "snag_duty_pct": 12,
            "follow_flip_ms": 300,
            "follow_take_mm": 20.0004,
            "follow_tail_mm": 750.5,
            "ff_chunk_mm": 4.5,
            "ff_pulse": 20,
        },
        "ace2k_feed_snag_set fwd_um=25000 lag_um=5000 duty_pct=12 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k: snag fwd 25 mm, lag 5 mm, duty 12 %, back 5 mm, rest 500 ms, free 50 mm",
        "ace2k_feed_follow_set flip_ms=300 take_um=20000 tail_um=750500",
        "ace2k: follow flip 300 ms, take 20 mm, tail 750.5 mm",
        "ace2k_feed_ff_set chunk_um=4500 pulse_um_s=20000",
        "ace2k: feed-forward chunk 4.5 mm, pulse 20 mm/s, window 500 ms, lead 0 ms,"
        " deadband 0.3 mm/s, long retraction 10 mm",
    ),
    (
        {
            "snag_fwd_mm": 19.99999,
            "snag_lag_mm": 4.0006,
            "snag_back_mm": 2.5,
            "snag_rest_ms": 1999,
            "snag_free_mm": 199.999,
            "follow_flip_ms": 300,
            "follow_take_mm": 20.0004,
            "follow_tail_mm": 750.5,
            "ff_chunk_mm": 1.0006,
            "ff_pulse": 12.3456,
        },
        "ace2k_feed_snag_set fwd_um=20000 lag_um=4001 duty_pct=0 back_um=2500 rest_ms=1999"
        " free_um=199999",
        "ace2k: snag fwd 20 mm, lag 4.001 mm, duty 0 %, back 2.5 mm, rest 1999 ms, free 199.999 mm",
        "ace2k_feed_follow_set flip_ms=300 take_um=20000 tail_um=750500",
        "ace2k: follow flip 300 ms, take 20 mm, tail 750.5 mm",
        "ace2k_feed_ff_set chunk_um=1001 pulse_um_s=12346",
        "ace2k: feed-forward chunk 1.001 mm, pulse 12.346 mm/s, window 500 ms, lead 0 ms,"
        " deadband 0.3 mm/s, long retraction 10 mm",
    ),
]


@pytest.mark.parametrize(
    ("options", "snag_cmd", "snag_line", "follow_cmd", "follow_line", "ff_cmd", "ff_line"),
    CONNECT_CASES,
)
def test_the_commands_and_lines_are_the_ones_captured(  # noqa: PLR0913, PLR0917 — one case's columns
    options: dict[str, float],
    snag_cmd: str,
    snag_line: str,
    follow_cmd: str,
    follow_line: str,
    ff_cmd: str,
    ff_line: str,
) -> None:
    feed, _printer, mcu, _module = make_ff_feed(options=options)
    assert SNAG.cmd_text(feed.snag) == snag_cmd
    assert SNAG.line(feed.snag) == snag_line
    assert FOLLOW.cmd_text(feed.follow) == follow_cmd
    assert FOLLOW.line(feed.follow) == follow_line
    assert FF.cmd_text(feed.ff_values) == ff_cmd
    assert ff_line.startswith(FF.line(feed.ff_values) + ", ")  # then the host's own values
    assert feed._ff_line() == ff_line
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert snag_cmd in init and follow_cmd in init and ff_cmd in init


GCODE_REFUSALS = [
    ("SNAG", {"LAG": "25"}, "ace2k: LAG (25) must be within 1.5..20"),
    ("SNAG", {"FWD": "6", "LAG": "8"}, "ace2k: LAG (8) must be under FWD (6) mm"),
    ("SNAG", {"FREE": "200.0006"}, "ace2k: FREE (200.001) must be within 20..200"),
    ("SNAG", {"BACK": "nan"}, "ace2k: BACK (nan) must be a finite number"),
    ("SNAG", {"DUTY": "101"}, "ace2k: DUTY (101) must be within 0..100"),
    ("SNAG", {"REST": "99"}, "ace2k: REST (99) must be within 100..2000"),
    ("FOLLOW", {"FLIP_MS": "49"}, "ace2k: FLIP_MS (49) must be within 50..2000"),
    ("FOLLOW", {"TAKE_MM": "inf"}, "ace2k: TAKE_MM (inf) must be a finite number"),
    ("FOLLOW", {"TAIL_MM": "2000.0006"}, "ace2k: TAIL_MM (2000.001) must be within 200..2000"),
    ("FF", {"CHUNK_MM": "0.5"}, "ace2k: CHUNK_MM (0.5) must be within 1..10"),
    ("FF", {"PULSE": "nan"}, "ace2k: PULSE (nan) must be a finite number"),
    ("FF", {"PULSE": "80.0006"}, "ace2k: PULSE (80.001) must be within 9..80"),
    ("FF", {"WINDOW_MS": "10"}, "ace2k: WINDOW_MS (10) must be within 100..5000"),
    ("FF", {"DEADBAND": "-0.1"}, "ace2k: DEADBAND (-0.1) must be within 0..10"),
    ("FF", {"RETRACT_MIN_MM": "20"}, "ace2k: RETRACT_MIN_MM (20) must be within 5..17"),
]


@pytest.mark.parametrize(("group", "params", "text"), GCODE_REFUSALS)
def test_a_refused_set_says_the_captured_text_and_changes_nothing(
    group: str, params: dict[str, str], text: str
) -> None:
    feed, printer, mcu, _module = make_ff_feed()
    commands = list(mcu.commands)
    with pytest.raises(FakeGcmd.error) as refused:
        getattr(feed, f"cmd_ACEK_{group}_SET")(FakeGcmd(params))
    assert str(refused.value) == text
    assert mcu.commands == commands and printer.objects["configfile"].calls == []


CONFIG_REFUSALS = [
    ({"snag_fwd_mm": 60}, "ace2k: snag_fwd_mm (60) must be within 5..50"),
    ({"snag_lag_mm": 0.5}, "ace2k: snag_lag_mm (0.5) must be within 1.5..20"),
    ({"snag_back_mm": 1.9994}, "ace2k: snag_back_mm (1.999) must be within 2..20"),
    ({"snag_rest_ms": 2001}, "ace2k: snag_rest_ms (2001) must be within 100..2000"),
    ({"snag_fwd_mm": float("inf")}, "ace2k: snag_fwd_mm (inf) must be a finite number"),
    (
        {"snag_lag_mm": 20},
        "ace2k: snag_lag_mm (20) must be under snag_fwd_mm (20): a lag the watch never reaches",
    ),
    (
        {"snag_fwd_mm": 5.0004, "snag_lag_mm": 5.0001},
        "ace2k: snag_lag_mm (5) must be under snag_fwd_mm (5): a lag the watch never reaches",
    ),
    ({"follow_flip_ms": 49}, "ace2k: follow_flip_ms (49) must be within 50..2000"),
    ({"follow_take_mm": 30.001}, "ace2k: follow_take_mm (30.001) must be within 5..30"),
    ({"follow_tail_mm": float("nan")}, "ace2k: follow_tail_mm (nan) must be a finite number"),
    ({"ff_chunk_mm": 0.9}, "ace2k: ff_chunk_mm (0.9) must be within 1..10"),
    ({"ff_pulse": 81}, "ace2k: ff_pulse (81) must be within 9..80"),
    (
        {"slip_allow_mm": 50},
        "ace2k: slip_allow_mm (50) must be under the image's default slip_check_mm (50): the"
        " partial slip check could never trip",
    ),
    (
        {"slip_check_mm": 10},
        "ace2k: the image's default slip_allow_mm (15) must be under slip_check_mm (10): the"
        " partial slip check could never trip",
    ),
]


@pytest.mark.parametrize(("options", "text"), CONFIG_REFUSALS)
def test_a_refused_key_says_the_captured_text(options: dict[str, float], text: str) -> None:
    with pytest.raises(Exception, match=re.escape(text)) as refused:
        make_ff_feed(options=options)
    assert str(refused.value) == text


STAGING = [
    (
        "SNAG",
        {"FWD": "19.99999", "LAG": "4.0006", "FREE": "199.999", "BACK": "2.5", "REST": "700"},
        [
            ("ace2k", "snag_fwd_mm", "20"),
            ("ace2k", "snag_lag_mm", "4.001"),
            ("ace2k", "snag_back_mm", "2.5"),
            ("ace2k", "snag_rest_ms", "700"),
            ("ace2k", "snag_free_mm", "199.999"),
        ],
        (
            "ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c back_um=%u rest_ms=%u free_um=%u",
            [20000, 4001, 0, 2500, 700, 199999],
        ),
        "ace2k: snag fwd 20 mm, lag 4.001 mm, duty 0 %, back 2.5 mm, rest 700 ms,"
        " free 199.999 mm; staged for SAVE_CONFIG",
    ),
    (
        "FOLLOW",
        {"FLIP_MS": "300", "TAKE_MM": "20.0004", "TAIL_MM": "750.5"},
        [
            ("ace2k", "follow_flip_ms", "300"),
            ("ace2k", "follow_take_mm", "20"),
            ("ace2k", "follow_tail_mm", "750.5"),
        ],
        ("ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u", [300, 20000, 750500]),
        "ace2k: follow flip 300 ms, take 20 mm, tail 750.5 mm; staged for SAVE_CONFIG",
    ),
    (
        "FF",
        {
            "CHUNK_MM": "4.0006",
            "PULSE": "12.5",
            "WINDOW_MS": "800",
            "LEAD_MS": "100",
            "DEADBAND": "0.5",
            "RETRACT_MIN_MM": "12",
        },
        [
            ("ace2k", "ff_window_ms", "800"),
            ("ace2k", "ff_lead_ms", "100"),
            ("ace2k", "ff_deadband", "0.5"),
            ("ace2k", "ff_retract_min_mm", "12"),
            ("ace2k", "ff_chunk_mm", "4.001"),
            ("ace2k", "ff_pulse", "12.5"),
        ],
        ("ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u", [4001, 12500]),
        "ace2k: feed-forward chunk 4.001 mm, pulse 12.5 mm/s, window 800 ms, lead 100 ms,"
        " deadband 0.5 mm/s, long retraction 12 mm; staged for SAVE_CONFIG",
    ),
]


@pytest.mark.parametrize(("group", "params", "staged", "sent", "line"), STAGING)
def test_a_set_stages_sends_and_says_what_was_captured(
    group: str,
    params: dict[str, str],
    staged: list[tuple[str, str, str]],
    sent: tuple[str, list[int]],
    line: str,
) -> None:
    feed, printer, mcu, _module = make_ff_feed()
    gcmd = FakeGcmd(params)
    getattr(feed, f"cmd_ACEK_{group}_SET")(gcmd)
    assert printer.objects["configfile"].calls == staged
    assert mcu.commands[-1] == sent
    # PULSE is retired: its own line comes after (test_ace2k_feed.py, FF_PULSE_RETIRED)
    retired = [f"ace2k: {ace2k_ff.FF_PULSE_RETIRED}"] if "PULSE" in params else []
    assert gcmd.lines == [line, *retired]


def test_the_settings_module_imports_only_the_link_and_the_feed_proto() -> None:
    assert {*_sibling_imports(SETTINGS)} == {"ace2k_link", "ace2k_feed_proto"}


def test_the_setting_names_still_import_from_ace2k_feed() -> None:
    # the tables and the helpers that lived in ace2k_feed
    tables = ("_SETTINGS", "_KEYS", "_CONSTANTS", "_DEFAULTS")
    names = {n for n in vars(ace2k_feed_settings) if n.endswith(tables) or n.startswith("setting_")}
    assert {"SNAG_SETTINGS", "THRESHOLD_DEFAULTS", "setting_round"} <= names
    for name in names:
        assert getattr(ace2k_feed, name) is getattr(ace2k_feed_settings, name), name


def test_judge_names_the_first_key_out_of_its_bounds() -> None:
    values = {"snag_fwd_mm": 20.0, "snag_lag_mm": 25.0, "snag_rest_ms": 99}
    assert SNAG.judge(values) == "snag_lag_mm (25) must be within 1.5..20"
    assert SNAG.judge({"snag_fwd_mm": 20.0}) is None
    assert FOLLOW.judge({"follow_flip_ms": 1}) is None  # no bounds before connect


def test_bounded_takes_the_dictionarys_bounds_and_a_whole_keys_values_whole() -> None:
    found = {
        "ACE2K_FEED_FOLLOW_FLIP_MS": (200.0, 50.0, 2000.0),
        "ACE2K_FEED_FOLLOW_TAKE_UM": (15.0, 5.0, 30.0),
        "ACE2K_FEED_FOLLOW_TAIL_UM": (2000.0, 200.0, 2000.0),
    }
    group, defaults = FOLLOW.bounded(found)
    assert defaults == {"follow_flip_ms": 200, "follow_take_mm": 15.0, "follow_tail_mm": 2000.0}
    assert isinstance(defaults["follow_flip_ms"], int)
    assert group.bounds["follow_flip_ms"] == (50, 2000)
    assert group.judge({"follow_flip_ms": 49}) == "follow_flip_ms (49) must be within 50..2000"
    assert not FOLLOW.bounds  # the template keeps none


def test_a_group_with_bounds_refuses_a_key_without_one_and_a_mismatched_format() -> None:
    partial = replace(SNAG, bounds={"snag_fwd_mm": (1.0, 20.0)})
    with pytest.raises(KeyError, match="no bounds for snag_lag_mm"):
        partial.judge({"snag_lag_mm": 5.0})
    with pytest.raises(ValueError, match="settings for"):
        replace(SNAG, fmt="ace2k_feed_snag_set fwd_um=%u")
