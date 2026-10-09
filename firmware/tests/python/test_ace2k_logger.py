"""The burn-in logger: a status becomes one CSV row, the G-code store becomes new events."""

import csv
import importlib.util
import json
import urllib.error
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "ace2k_logger.py"
spec = importlib.util.spec_from_file_location("ace2k_logger", SCRIPT)
logger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logger)

LANE = dict(
    insert=True,
    rest=True,
    pushed=False,
    mode="assisting",
    error=None,
    speed=12.5,
    duty=31,
    bursts=7,
    encoder_um=-1234,
)
STATUS = {
    "print_stats": {"state": "printing"},
    "mcu ace2k": {
        "last_stats": {
            "bytes_retransmit": 9,
            "bytes_invalid": 0,
            "retransmit_seq": 3,
            "srtt": 0.002,
        }
    },
    "ace2k": {
        "version": "0.7.0",
        "link_proven": True,
        "ptc_left": 41.5,
        "ptc_right": 42.0,
        "chamber": 38.2,
        "humidity": 21.0,
        "mains_hz": 60.0,
        "mains_rejects": 4,
        "cutout": False,
        "health": {"ok": True, "failing": [], "latched": ["rfid_b"]},
        "lanes": [LANE, dict(LANE, mode="idle"), dict(LANE, insert=False), dict(LANE)],
        "dryer": {
            "state": "heating",
            "target": 45,
            "drive": 52.0,
            "duty": 30,
            "remaining": 118,
            "fault": None,
            "notices": ["vent"],
        },
    },
}


def test_a_full_status_fills_every_column():
    row = logger.row_from_status(STATUS, 1000.0)
    assert set(row) == set(logger.COLUMNS)
    assert row["time"] == "1000.0"
    assert row["print_state"] == "printing"
    assert row["link_proven"] == "1"
    assert row["lane1_mode"] == "assisting"
    assert row["lane2_mode"] == "idle"
    assert row["lane3_insert"] == "0"
    assert row["lane1_error"] == ""
    assert row["lane1_encoder_um"] == "-1234"
    assert row["health_latched"] == "rfid_b"
    assert row["dryer_state"] == "heating"
    assert row["dryer_notices"] == "vent"
    assert row["link_bytes_retransmit"] == "9"


def test_an_empty_status_leaves_every_column_empty_but_the_time():
    row = logger.row_from_status({}, 5.0)
    assert set(row) == set(logger.COLUMNS)
    assert row["time"] == "5.0"
    assert all(v == "" for k, v in row.items() if k != "time")


def test_no_host_row():
    row = logger.no_host_row(7.0)
    assert row["print_state"] == "no_host"
    assert all(v == "" for k, v in row.items() if k not in ("time", "print_state"))


def test_new_events_keeps_only_newer_ace2k_lines():
    store = [
        {"message": "ace2k: lane 1 snag (feeding)", "time": 10.0, "type": "response"},
        {"message": "ok", "time": 11.0, "type": "response"},
        {"message": "ace2k: lane 2 runout", "time": 12.0, "type": "response"},
    ]
    events, last = logger.new_events(store, 10.0)
    assert [e["message"] for e in events] == ["ace2k: lane 2 runout"]
    assert last == 12.0
    events, last = logger.new_events(store, 12.0)
    assert events == [] and last == 12.0


def test_append_writes_a_header_once_and_one_line_per_event(tmp_path):
    logger.append_row(tmp_path, logger.row_from_status(STATUS, 1000.0))
    logger.append_row(tmp_path, logger.row_from_status(STATUS, 1010.0))
    (csv_file,) = tmp_path.glob("ace2k-*.csv")
    with csv_file.open(newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["time"] for r in rows] == ["1000.0", "1010.0"]
    logger.append_events(tmp_path, [{"message": "ace2k: x", "time": 1.0, "type": "response"}])
    (ev_file,) = tmp_path.glob("ace2k-events-*.jsonl")
    assert json.loads(ev_file.read_text().splitlines()[0])["message"] == "ace2k: x"


def _rows(out):
    rows = []
    for path in sorted(out.glob("ace2k-[0-9]*.csv")):
        with path.open(newline="") as f:
            rows += list(csv.DictReader(f))
    return rows


def _events(out):
    lines = []
    for path in sorted(out.glob("ace2k-events-*.jsonl")):
        lines += [json.loads(line) for line in path.read_text().splitlines()]
    return lines


def _fetcher(status=None, store=None):
    """A fetch stand-in: each answer is a value to return or an exception to raise."""

    def fetch(url):
        answer = status if url.endswith(logger.QUERY) else store
        if isinstance(answer, Exception):
            raise answer
        return answer

    return fetch


def test_poll_once_without_moonraker_writes_one_no_host_row(tmp_path, monkeypatch):
    down = urllib.error.URLError("refused")
    monkeypatch.setattr(logger, "fetch", _fetcher(down, down))
    assert logger.poll_once(tmp_path, "http://x", 50.0, 100.0) == 50.0
    rows = _rows(tmp_path)
    assert len(rows) == 1 and rows[0]["print_state"] == "no_host"
    assert _events(tmp_path) == []


def test_poll_once_with_the_store_down_keeps_the_good_row_only(tmp_path, monkeypatch):
    fetch = _fetcher({"status": STATUS}, urllib.error.URLError("refused"))
    monkeypatch.setattr(logger, "fetch", fetch)
    assert logger.poll_once(tmp_path, "http://x", 50.0, 100.0) == 50.0
    rows = _rows(tmp_path)
    assert len(rows) == 1 and rows[0]["print_state"] == "printing"


def test_poll_once_with_a_malformed_status_writes_one_no_host_row(tmp_path, monkeypatch):
    fetch = _fetcher({"status": {"ace2k": "garbage"}}, {"gcode_store": []})
    monkeypatch.setattr(logger, "fetch", fetch)
    assert logger.poll_once(tmp_path, "http://x", 50.0, 100.0) == 50.0
    rows = _rows(tmp_path)
    assert len(rows) == 1 and rows[0]["print_state"] == "no_host"


def test_poll_once_marks_a_full_store_newer_than_the_last_line(tmp_path, monkeypatch):
    store = [
        {"message": f"ace2k: line {i}", "time": 60.0 + i, "type": "response"}
        for i in range(logger.STORE_COUNT)
    ]
    monkeypatch.setattr(logger, "fetch", _fetcher({"status": STATUS}, {"gcode_store": store}))
    last = logger.poll_once(tmp_path, "http://x", 50.0, 100.0)
    assert last == 60.0 + logger.STORE_COUNT - 1
    events = _events(tmp_path)
    assert events[0] == {"message": logger.MISSED, "time": 60.0, "type": "marker"}
    assert [e["message"] for e in events[1:]] == [e["message"] for e in store]


def test_poll_once_with_events_unwritable_keeps_last_event(tmp_path, monkeypatch):
    store = [{"message": "ace2k: x", "time": 70.0, "type": "response"}]
    monkeypatch.setattr(logger, "fetch", _fetcher({"status": STATUS}, {"gcode_store": store}))

    def broken(out, events):
        raise OSError("disk full")

    monkeypatch.setattr(logger, "append_events", broken)
    assert logger.poll_once(tmp_path, "http://x", 50.0, 100.0) == 50.0
    assert len(_rows(tmp_path)) == 1


def test_poll_once_survives_an_unwritable_row(tmp_path, monkeypatch):
    store = [{"message": "ace2k: x", "time": 70.0, "type": "response"}]
    monkeypatch.setattr(logger, "fetch", _fetcher({"status": STATUS}, {"gcode_store": store}))

    def broken(out, row):
        raise OSError("disk full")

    monkeypatch.setattr(logger, "append_row", broken)
    assert logger.poll_once(tmp_path, "http://x", 50.0, 100.0) == 70.0
    assert [e["message"] for e in _events(tmp_path)] == ["ace2k: x"]
