#!/usr/bin/env python3
"""The ace2k burn-in logger.

Runs on the printer beside Klipper and Moonraker. Every PERIOD seconds it reads the [ace2k]
status, the print state and Klipper's own statistics of the unit's MCU through Moonraker's HTTP
API and appends one CSV row to OUT/ace2k-YYYY-MM-DD.csv (the printer's local date); every
ace2k line of the G-code store not seen before goes to OUT/ace2k-events-YYYY-MM-DD.jsonl. It
never opens the unit's port and sends no G-code. Nothing it writes names the unit, the printer
or a print. Stdlib only; Python 3.7.

    python3 ace2k_logger.py --out /home/lava/printer_data/ace2k-burnin [--url URL] [--period S]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
import urllib.request

LANES = 4
QUERY = "/printer/objects/query?ace2k&print_stats&mcu%20ace2k"
STORE_COUNT = 200
STORE = f"/server/gcode_store?count={STORE_COUNT}"
MISSED = "ace2k-logger: console lines may have been missed"
TOP_FIELDS = (
    "version",
    "link_proven",
    "ptc_left",
    "ptc_right",
    "chamber",
    "humidity",
    "mains_hz",
    "mains_rejects",
    "cutout",
)
HEALTH_FIELDS = ("ok", "failing", "latched")
LANE_FIELDS = ("insert", "rest", "pushed", "mode", "error", "speed", "duty", "bursts", "encoder_um")
DRYER_FIELDS = ("state", "target", "drive", "duty", "remaining", "fault", "notices")
LINK_FIELDS = ("bytes_retransmit", "bytes_invalid", "retransmit_seq", "srtt")
COLUMNS = (
    ["time", "print_state"]
    + list(TOP_FIELDS)
    + ["health_" + f for f in HEALTH_FIELDS]
    + [f"lane{n}_{f}" for n in range(1, LANES + 1) for f in LANE_FIELDS]
    + ["dryer_" + f for f in DRYER_FIELDS]
    + ["link_" + f for f in LINK_FIELDS]
)


def cell(value):
    """One CSV cell: empty for None, 1/0 for a bool, ';'-joined for a list."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (list, tuple)):
        return ";".join(str(v) for v in value)
    return str(value)


def empty_row(now):
    row = dict.fromkeys(COLUMNS, "")
    row["time"] = f"{now:.1f}"
    return row


def no_host_row(now):
    row = empty_row(now)
    row["print_state"] = "no_host"
    return row


def row_from_status(status, now):
    """The CSV row for one Moonraker objects query's status; a missing object or field is an
    empty cell, never an error."""
    row = empty_row(now)
    ace = status.get("ace2k") or {}
    row["print_state"] = cell((status.get("print_stats") or {}).get("state"))
    for f in TOP_FIELDS:
        row[f] = cell(ace.get(f))
    health = ace.get("health") or {}
    for f in HEALTH_FIELDS:
        row["health_" + f] = cell(health.get(f))
    lanes = ace.get("lanes") or []
    for n in range(1, LANES + 1):
        lane = lanes[n - 1] if len(lanes) >= n and lanes[n - 1] else {}
        for f in LANE_FIELDS:
            row[f"lane{n}_{f}"] = cell(lane.get(f))
    dryer = ace.get("dryer") or {}
    for f in DRYER_FIELDS:
        row["dryer_" + f] = cell(dryer.get(f))
    stats = (status.get("mcu ace2k") or {}).get("last_stats") or {}
    for f in LINK_FIELDS:
        row["link_" + f] = cell(stats.get(f))
    return row


def new_events(store, last_time):
    """The G-code store's entries newer than last_time that mention ace2k, and the newest time
    of every entry (so a quiet console never re-reads old lines). A full store whose oldest entry
    is already newer than last_time may have lost lines in between: a marker event at that
    oldest time comes first."""
    newest = last_time
    events = []
    if len(store) >= STORE_COUNT:
        oldest = min(entry.get("time", 0.0) for entry in store)
        if oldest > last_time:
            events.append({"message": MISSED, "time": oldest, "type": "marker"})
    for entry in store:
        t = entry.get("time", 0.0)
        newest = max(newest, t)
        if t > last_time and "ace2k" in entry.get("message", ""):
            events.append(entry)
    return events, newest


def _day(now):
    return time.strftime("%Y-%m-%d", time.localtime(now))


def append_row(out, row):
    path = os.path.join(str(out), f"ace2k-{_day(float(row['time']))}.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow(row)


def append_events(out, events):
    for event in events:
        path = os.path.join(str(out), f"ace2k-events-{_day(event.get('time', 0.0))}.jsonl")
        with open(path, "a") as f:
            f.write(json.dumps(event, sort_keys=True) + "\n")


def fetch(url):
    with urllib.request.urlopen(url, timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))["result"]


def _warn(now, what, err):
    print(f"{now:.1f} ace2k-logger: {what}: {err}", file=sys.stderr)


def poll_once(out, url, last_event, now):
    """One round: one CSV row (no_host when the status cannot be read), then the new console
    lines. Returns the time of the newest console line seen and written; a round that cannot
    read or write them keeps last_event, so they are read again next round."""
    try:
        row = row_from_status(fetch(url + QUERY)["status"], now)
    except Exception:
        row = no_host_row(now)
    try:
        append_row(out, row)
    except OSError as err:
        _warn(now, "row not written", err)
    try:
        events, newest = new_events(fetch(url + STORE)["gcode_store"], last_event)
    except Exception:
        return last_event
    try:
        append_events(out, events)
    except OSError as err:
        _warn(now, "events not written", err)
        return last_event
    return newest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--url", default="http://127.0.0.1:7125")
    ap.add_argument("--period", type=float, default=10.0)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    last_event = time.time()
    while True:
        now = time.time()
        last_event = poll_once(args.out, args.url, last_event, now)
        time.sleep(max(0.0, args.period - (time.time() - now)))


if __name__ == "__main__":
    main()
