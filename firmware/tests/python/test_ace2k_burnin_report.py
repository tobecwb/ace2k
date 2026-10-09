"""The burn-in report over synthetic logger rows."""

import importlib.util
import json
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "ace2k_burnin_report.py"
spec = importlib.util.spec_from_file_location("ace2k_burnin_report", SCRIPT)
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


def row(
    t,
    state="standby",
    version="0.7.0",
    mode1="idle",
    bursts1="0",
    dryer="idle",
    retx="0",
    invalid="0",
    failing="",
    enc1="0",
):
    r = {
        "time": str(t),
        "print_state": state,
        "version": version,
        "dryer_state": dryer,
        "link_bytes_retransmit": retx,
        "link_bytes_invalid": invalid,
        "health_failing": failing,
        "health_latched": "",
    }
    for n in range(1, 5):
        r[f"lane{n}_mode"] = mode1 if n == 1 else "idle"
        r[f"lane{n}_bursts"] = bursts1 if n == 1 else "0"
        r[f"lane{n}_encoder_um"] = enc1 if n == 1 else "0"
    return r


def test_prints_are_runs_of_printing_or_paused():
    rows = (
        [row(t) for t in range(0, 30, 10)]
        + [row(t, "printing", mode1="assisting", bursts1=str(t // 10)) for t in range(30, 90, 10)]
        + [row(90, "paused", mode1="assisting", bursts1="9")]
        + [row(t, "complete") for t in range(100, 130, 10)]
    )
    (p,) = report.prints(rows, events=[])
    assert p["start"] == 30.0
    assert round(p["hours"] * 3600) == 70  # 30 → 100, each row worth up to the next
    assert p["lanes"] == [1]
    assert p["bursts"] == {1: 9}


def test_a_gap_over_30_s_is_not_counted():
    rows = [row(0, "printing"), row(10, "printing"), row(500, "printing"), row(510, "complete")]
    (p,) = report.prints(rows, events=[])
    assert round(p["hours"] * 3600) == 20


def test_a_no_host_row_does_not_split_a_print():
    rows = [
        row(0, "printing"),
        row(10, "no_host", version=""),
        row(20, "printing"),
        row(30, "complete"),
    ]
    (p,) = report.prints(rows, events=[])
    assert round(p["hours"] * 3600) == 20


def test_days_count_connected_printing_link_and_dryer():
    rows = [
        row(0, retx="5"),
        row(10, "printing", retx="7", dryer="heating"),
        row(20, "printing", retx="1", dryer="heating"),  # Klipper restarted: a new base
        row(30, retx="4", dryer="cooldown", failing="ntc_left"),
        row(40, version="", retx="4"),
    ]
    (d,) = report.days(rows)
    assert round(d["connected_h"] * 3600) == 40
    assert round(d["printing_h"] * 3600) == 20
    assert d["retransmit"] == 2 + 3
    assert d["dryer_cycles"] == 1
    assert d["health"] == ["ntc_left"]


def test_totals_against_the_targets():
    t = report.totals(
        prints=[{"hours": 4.0, "lanes": [1, 2], "start": 0.0, "end": 1.0}] * 5
        + [{"hours": 1.0, "lanes": [3, 4], "start": 0.0, "end": 1.0}],
        days=[{"dryer_cycles": 2}],
        dryer_during_print=True,
    )
    assert t["printing_h"] == 21.0
    assert t["prints"] == 6
    assert t["lanes"] == [1, 2, 3, 4]
    assert t["met"] is True
    t = report.totals(prints=[], days=[], dryer_during_print=False)
    assert t["met"] is False


def test_render_lists_the_events_to_classify():
    events = [{"time": 40.0, "message": "ace2k: lane 1 stuck (feeding)"}]
    text = report.render([], [], report.totals([], [], False), events)
    assert "lane 1 stuck" in text
    assert "| class |" in text


def test_events_written_twice_are_listed_once(tmp_path):
    lines = [
        {
            "message": "ace2k-logger: console lines may have been missed",
            "time": 30.0,
            "type": "marker",
        },
        {"message": "ace2k: lane 1 stuck (feeding)", "time": 40.0, "type": "response"},
        {"message": "ace2k: lane 1 stuck (feeding)", "time": 40.0, "type": "response"},
        {"message": "ace2k: lane 2 stuck (feeding)", "time": 40.0, "type": "response"},
    ]
    path = tmp_path / "ace2k-events-1970-01-01.jsonl"
    path.write_text("".join(json.dumps(e) + "\n" for e in lines))
    events = report.load_events(str(tmp_path))
    assert [(e["time"], e["message"]) for e in events] == [
        (30.0, "ace2k-logger: console lines may have been missed"),
        (40.0, "ace2k: lane 1 stuck (feeding)"),
        (40.0, "ace2k: lane 2 stuck (feeding)"),
    ]
    text = report.render([], [], report.totals([], [], False), events)
    assert "console lines may have been missed" in text


def test_bursts_left_from_an_earlier_print_do_not_mark_a_lane_used():
    rows = (
        [row(0)]
        + [row(t, "printing", mode1="assisting", bursts1=str(t // 10)) for t in range(10, 50, 10)]
        + [row(t, "complete", bursts1="4") for t in range(50, 80, 10)]
        + [row(t, "printing", bursts1="4") for t in range(80, 120, 10)]
        + [row(120, "complete", bursts1="4")]
    )
    first, second = report.prints(rows, events=[])
    assert first["lanes"] == [1]
    assert first["bursts"] == {1: 4}
    assert second["lanes"] == []
    assert second["bursts"] == {}


def test_a_new_arm_in_a_print_counts_from_its_reset():
    rows = [row(0, bursts1="7")] + [
        row(10, "printing", bursts1="7"),
        row(20, "printing", bursts1="0"),
        row(30, "printing", bursts1="3"),
        row(40, "complete", bursts1="3"),
    ]
    (p,) = report.prints(rows, events=[])
    assert p["lanes"] == [1]
    assert p["bursts"] == {1: 3}


def test_assisting_back_marks_a_lane_used():
    rows = [row(0, "printing", mode1="assisting_back"), row(10, "complete")]
    (p,) = report.prints(rows, events=[])
    assert p["lanes"] == [1]
    assert p["bursts"] == {}


def _following_print(values, before="0", after=None, before_mode="following", modes=None, dt=10):
    """Lane 1 before, through and after a print, its encoder at values (um) in it, following
    unless modes names each in-print row's mode; rows dt seconds apart."""
    after = values[-1] if after is None else after
    modes = ["following"] * len(values) if modes is None else modes
    assert len(modes) == len(values), "one mode per value"
    return (
        [row(0, mode1=before_mode, enc1=before)]
        + [
            row(dt * (i + 1), "printing", mode1=m, enc1=v)
            for i, (v, m) in enumerate(zip(values, modes))
        ]
        + [row(dt * (len(values) + 1), "complete", mode1="following", enc1=after)]
    )


def _lanes(rows):
    (p,) = report.prints(rows, events=[])
    return p["lanes"]


def test_a_lane_following_whose_filament_moved_is_used_with_no_burst():
    # the follow with the feed-forward keeps bursts at 0 through hours of printing
    rows = _following_print([str(1000 + 100000 * i) for i in range(6)])  # +500 mm
    (p,) = report.prints(rows, events=[])
    assert p["lanes"] == [1]
    assert p["bursts"] == {}
    assert report.totals([p], [], False)["lanes"] == [1]


def test_a_loaded_head_following_with_its_filament_still_is_not_used():
    # an unused loaded head of a one-head print: following, the encoder flat or jittering
    assert _lanes(_following_print(["5000", "6200"] * 60)) == []


def test_back_and_forth_in_the_follow_is_not_used():
    assert _lanes(_following_print(["0", "6000"] * 4 + ["0"])) == []


def test_a_lane_following_only_outside_a_print_is_not_used():
    rows = [
        row(0, mode1="following", enc1="0"),
        row(10, "printing", enc1="0"),
        row(20, "printing", enc1="0"),
        row(30, "complete", mode1="following", enc1="900000"),
    ]
    assert _lanes(rows) == []


def test_slow_steps_add_up_whatever_the_period():
    # 1 s rows of 0.1 mm each: 25 mm over the print
    assert _lanes(_following_print([str(100 * i) for i in range(1, 251)], dt=1)) == [1]
    assert _lanes(_following_print([str(100 * i) for i in range(1, 151)], dt=1)) == []


def test_an_outage_in_a_stretch_keeps_the_movement_across_it():
    rows = [
        row(0, mode1="following", enc1="0"),
        row(10, "printing", mode1="following", enc1="0"),
        row(3000, "printing", mode1="following", enc1="2400000"),  # the logger was down
        row(3010, "complete", mode1="following", enc1="2400000"),
    ]
    assert _lanes(rows) == [1]


def test_a_no_host_or_unread_row_does_not_end_a_stretch():
    rows = _following_print(["0", "10000", "10000", "25000"])
    rows[2]["print_state"] = "no_host"
    rows[3]["lane1_mode"] = ""
    assert _lanes(rows) == [1]


def test_a_blank_encoder_row_in_a_stretch_keeps_the_movement():
    assert _lanes(_following_print(["0", "10000", "", "25000"])) == [1]


def test_a_reset_in_a_stretch_then_30_mm_is_used():
    assert _lanes(_following_print(["800000", "0", "15000", "30000"], before="800000")) == [1]


def test_an_advance_under_the_threshold_is_not_a_use():
    assert _lanes(_following_print(["0", "10000", "19999"])) == []


def test_a_feed_in_a_print_then_the_follow_flat_is_not_used():
    values = [str(150000 * i) for i in range(6)] + ["750000"] * 6  # +750 mm feeding, then still
    assert _lanes(_following_print(values, modes=["feeding"] * 6 + ["following"] * 6)) == []


def test_the_stretch_before_the_print_counts_from_its_last_row():
    assert _lanes(_following_print(["25000", "25000"], before="0")) == [1]
    # movement before the print's last pre-print row does not count
    rows = [row(0, mode1="following", enc1="0")] + _following_print(["600000"], before="600000")
    rows[1]["time"] = "5"
    assert _lanes(rows) == []


def test_a_stretch_does_not_begin_on_a_row_out_of_the_follow():
    assert _lanes(_following_print(["25000", "25000"], before="0", before_mode="feeding")) == []


def test_the_helper_refuses_modes_and_values_of_different_lengths():
    with pytest.raises(AssertionError):
        _following_print(["0", "1"], modes=["following"])


def test_a_row_without_a_dryer_state_does_not_start_a_new_cycle():
    rows = [
        row(0),
        row(10, dryer="heating"),
        row(20, "no_host", version="", dryer=""),
        row(30, dryer="heating"),
        row(40, dryer="cooldown"),
    ]
    (d,) = report.days(rows)
    assert d["dryer_cycles"] == 1


def _local(day, hh, mm, ss):
    return time.mktime(time.strptime(f"{day} {hh}:{mm}:{ss}", "%Y-%m-%d %H:%M:%S"))


def test_a_cycle_across_midnight_counts_once_and_link_rises_keep_the_midnight_step():
    t = _local("2026-10-03", 23, 59, 50)
    rows = [
        row(t, dryer="idle", retx="0"),
        row(t + 5, dryer="heating", retx="2"),
        row(t + 15, dryer="heating", retx="5"),
        row(t + 25, dryer="cooldown", retx="6"),
    ]
    d1, d2 = report.days(rows)
    assert d1["dryer_cycles"] + d2["dryer_cycles"] == 1
    assert d1["dryer_cycles"] == 1
    assert d1["retransmit"] == 2
    assert d2["retransmit"] == 4


def test_event_messages_are_escaped_in_the_table():
    events = [{"time": 40.0, "message": "ace2k: a | b\nc"}]
    text = report.render([], [], report.totals([], [], False), events)
    assert "| ace2k: a \\| b c |" in text


def test_truncated_lines_and_bad_rows_are_skipped_and_counted(tmp_path):
    good = json.dumps({"message": "ace2k: lane 1 stuck", "time": 40.0})
    (tmp_path / "ace2k-events-1970-01-01.jsonl").write_text(good + "\n" + good[:15] + "\n")
    (tmp_path / "ace2k-1970-01-01.csv").write_text(
        "time,print_state,version\n10.0,standby,0.7.0\n,standby,0.7.0\nx,standby,0.7.0\n"
    )
    skipped = {}
    rows = report.load_rows(str(tmp_path), skipped)
    events = report.load_events(str(tmp_path), skipped)
    assert len(rows) == 1
    assert len(events) == 1
    assert skipped == {"rows": 2, "events": 1}
    text = report.render([], [], report.totals([], [], False), events, skipped)
    assert "skipped: 1 event lines, 2 rows" in text
    assert "skipped:" not in report.render([], [], report.totals([], [], False), events, {})


def _lane1_print(baseline, values):
    rows = [row(0, bursts1=str(baseline))]
    rows += [row(10 * (i + 1), "printing", bursts1=str(v)) for i, v in enumerate(values)]
    rows += [row(10 * (len(values) + 1), "complete", bursts1=str(values[-1]))]
    (p,) = report.prints(rows, events=[])
    return p


def test_a_re_arm_in_a_print_adds_to_the_run_before_it():
    p = _lane1_print(0, [0, 20, 40, 0, 5])
    assert p["lanes"] == [1]
    assert p["bursts"] == {1: 45}


def test_the_first_run_counts_from_the_baseline_and_later_runs_from_0():
    p = _lane1_print(7, [7, 10, 0, 3])
    assert p["bursts"] == {1: 6}


def test_three_arms_in_a_print_add_up():
    p = _lane1_print(0, [12, 30, 0, 8, 0, 4, 25])
    assert p["bursts"] == {1: 30 + 8 + 25}
