#!/usr/bin/env python3
"""The ace2k burn-in report: the logger's CSVs and events into Markdown.

    python3 ace2k_burnin_report.py DIR > report.md

DIR holds the logger's ace2k-YYYY-MM-DD.csv and ace2k-events-YYYY-MM-DD.jsonl files, copied
from the printer. Targets: at least 20 hours printing in at least
five prints, every lane used in a print, two drying cycles, one of them during a print. A
lane is used in a print when it assists in it, its bursts grow, or its filament advances at
least FOLLOW_MOVED_UM while it follows (see _Follow).
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import time

GAP_S = 30.0  # a row is worth the time to the next one up to this; beyond, the logger was down
LANES = 4
PRINTING = ("printing", "paused")
ASSIST_MODES = ("assisting", "assisting_back")  # a lane in one of these in a print was used
# The follow is the standing state of a loaded head, so a lane in it was used in a print only
# when its filament moved: the feed-forward keeps its bursts near 0, and an unused loaded head of
# a one-head print sits in it too.
FOLLOW_MODE = "following"
FOLLOW_MOVED_UM = 20000  # the least advance in the follow, over a print, that is a use
# A fall of the encoder by more than this in the follow is a counters reset, not a pull back
FOLLOW_RESET_UM = 50000
DRYING = ("starting", "heating")
NOT_DRYING = ("idle", "cooldown", "fault")
NO_HOST = "no_host"  # the logger could not reach Moonraker: the print state is unknown
TARGET_H = 20.0
TARGET_PRINTS = 5
TARGET_DRY = 2


def _time(value):
    """A row's or an event's time as a finite float, or None."""
    try:
        t = float(value)
    except (TypeError, ValueError):
        return None
    return t if math.isfinite(t) else None


def load_rows(directory, skipped=None):
    """Every CSV row in time order; a row with an empty or invalid time is skipped and counted
    in skipped["rows"]."""
    rows = []
    bad = 0
    for path in sorted(glob.glob(os.path.join(directory, "ace2k-[0-9]*.csv"))):
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                if _time(r.get("time")) is None:
                    bad += 1
                else:
                    rows.append(r)
    if skipped is not None:
        skipped["rows"] = skipped.get("rows", 0) + bad
    rows.sort(key=lambda r: float(r["time"]))
    return rows


def load_events(directory, skipped=None):
    """Every event once: the logger may append a line again after a partly failed write, so a
    repeated (time, message) keeps its first copy. Marker events stay, to be classified. A line
    that is not a JSON object with a valid time (a truncated write) is skipped and counted in
    skipped["events"]."""
    events = []
    seen = set()
    bad = 0
    for path in sorted(glob.glob(os.path.join(directory, "ace2k-events-*.jsonl"))):
        with open(path) as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except ValueError:
                    event = None
                if not isinstance(event, dict) or _time(event.get("time")) is None:
                    bad += 1
                    continue
                event["time"] = _time(event["time"])
                key = (event["time"], event.get("message", ""))
                if key in seen:
                    continue
                seen.add(key)
                events.append(event)
    if skipped is not None:
        skipped["events"] = skipped.get("events", 0) + bad
    events.sort(key=lambda e: e["time"])
    return events


def _int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _day(t):
    return time.strftime("%Y-%m-%d", time.localtime(t))


def _worth(rows, i):
    """Seconds row i stands for: the time to the next row, or 0 when that gap is over GAP_S
    (the logger was down) or there is no next row."""
    if i + 1 >= len(rows):
        return 0.0
    gap = float(rows[i + 1]["time"]) - float(rows[i]["time"])
    return gap if gap <= GAP_S else 0.0


class _Follow:
    """One lane's advance in the follow, kept across all rows (a stretch may begin before a
    print). A stretch is a series of rows with the lane following; a row in another mode ends
    it. A no_host row, a row with no lane mode (the unit not read) and a row with no encoder
    value are skipped and end nothing. A stretch's advance is its net, its last encoder value
    less its first, floored at 0, so back and forth and small pull backs cancel out whatever
    the logger's period. A fall by more than FOLLOW_RESET_UM is a counters reset: it closes the
    stretch at the value before it and opens another from the value after it.

    begin() opens a print: the advance counts from 0, and a stretch already open counts from
    its last value, the last row before the print. advance() is the sum of the nets since.

    Known limits: a live change of the encoder's scale mid-stretch moves the net by the count
    times the change; a count at the int32 limit (about 2147 m without a counters reset)
    reads flat; a real pull back of more than FOLLOW_RESET_UM while following reads as a
    counters reset, so the feed that takes it back counts again (an overcount, only for a lane
    that really moved); a real reset that falls by FOLLOW_RESET_UM or less reads as a pull
    back (an undercount of up to FOLLOW_RESET_UM); movement during no_host rows just before a
    print's first row counts toward that print."""

    def __init__(self):
        self.first = None  # the open stretch's first encoder value, or None
        self.last = None
        self.closed = 0  # the nets of the stretches closed since begin()

    def _close(self):
        if self.first is not None:
            self.closed += max(0, self.last - self.first)
        self.first = self.last = None

    def see(self, mode, encoder_um):
        if not mode:
            return
        if mode != FOLLOW_MODE:
            self._close()
            return
        if encoder_um is None:
            return
        if self.first is not None and encoder_um < self.last - FOLLOW_RESET_UM:
            self._close()
        if self.first is None:
            self.first = encoder_um
        self.last = encoder_um

    def begin(self):
        self.closed = 0
        if self.first is not None:
            self.first = self.last

    def advance(self):
        open_net = 0 if self.first is None else max(0, self.last - self.first)
        return self.closed + open_net


class _LaneInPrint:
    """One lane's use in one print: used when it was in an assist on any row of the print, or
    its bursts grew, or its filament advanced at least FOLLOW_MOVED_UM in the follow (_Follow).

    A lane's bursts counter keeps its value after an assist ends and is reset to 0 only when
    the lane's next mode starts, so the print is a series of runs: the first starts at the last
    value seen before the print began (the baseline), and every drop (a new arm, reset to 0)
    starts another from 0. The print's bursts are the sum over its runs of the highest value
    reached minus the run's start."""

    def __init__(self, baseline):
        self.advance = 0  # the advance in the follow, set when the print closes, in um
        self.prev = baseline
        self.start = baseline  # where the current run began
        self.top = baseline  # the highest value in the current run
        self.closed = 0  # the bursts of the runs ended by a drop
        self.assisting = False

    def see(self, mode, bursts):
        if mode in ASSIST_MODES:
            self.assisting = True
        if bursts is None:
            return
        if bursts < self.prev:
            self.closed += self.top - self.start
            self.start = 0
            self.top = bursts
        else:
            self.top = max(self.top, bursts)
        self.prev = bursts

    def bursts(self):
        return self.closed + self.top - self.start

    def used(self):
        return self.assisting or self.advance >= FOLLOW_MOVED_UM or self.bursts() > 0


def prints(rows, events):
    """Runs of printing/paused rows; a no_host row neither ends a run nor adds to it."""
    lanes = range(1, LANES + 1)
    out = []
    current = None
    last = dict.fromkeys(lanes, 0)  # each lane's last bursts value seen
    follow = {n: _Follow() for n in lanes}

    def close():
        for n in lanes:
            current["lane_use"][n].advance = follow[n].advance()
        out.append(current)

    for i, r in enumerate(rows):
        state = r["print_state"]
        if state in PRINTING:
            if current is None:
                current = {
                    "start": float(r["time"]),
                    "end": float(r["time"]),
                    "seconds": 0.0,
                    "lane_use": {n: _LaneInPrint(last[n]) for n in lanes},
                }
                for n in lanes:
                    follow[n].begin()
            current["seconds"] += _worth(rows, i)
            current["end"] = float(r["time"])
            for n in lanes:
                current["lane_use"][n].see(r.get(f"lane{n}_mode"), _int(r.get(f"lane{n}_bursts")))
        elif current is not None and state != NO_HOST:
            close()
            current = None
        if state != NO_HOST:
            for n in lanes:
                follow[n].see(r.get(f"lane{n}_mode"), _int(r.get(f"lane{n}_encoder_um")))
        for n in lanes:
            value = _int(r.get(f"lane{n}_bursts"))
            if value is not None:
                last[n] = value
    if current is not None:
        close()
    for p in out:
        lane_use = p.pop("lane_use")
        p["hours"] = p.pop("seconds") / 3600.0
        p["lanes"] = [n for n in sorted(lane_use) if lane_use[n].used()]
        p["bursts"] = {n: lane_use[n].bursts() for n in p["lanes"] if lane_use[n].bursts() > 0}
        p["events"] = [e for e in events if p["start"] <= e.get("time", 0.0) <= p["end"] + GAP_S]
    return out


def _increases(rows, column):
    """Per day, the rises of a counter that restarts from 0 with Klipper, in one walk over all
    rows: a rise between two samples belongs to the day of the later one; a drop starts a new
    base."""
    out = {}
    last = None
    for r in rows:
        v = _int(r.get(column))
        if v is None:
            continue
        if last is not None and v >= last:
            day = _day(float(r["time"]))
            out[day] = out.get(day, 0) + v - last
        last = v
    return out


def dryer_cycles(rows):
    """Per day, the drying cycles begun that day, in one walk over all rows: an entry into
    starting or heating from idle, cooldown or fault (or with no state known before it). A row
    with no dryer state (the logger could not read it) leaves the last known state unchanged."""
    out = {}
    prev = None
    for r in rows:
        state = r.get("dryer_state")
        if not state:
            continue
        if state in DRYING and (prev is None or prev in NOT_DRYING):
            day = _day(float(r["time"]))
            out[day] = out.get(day, 0) + 1
        prev = state
    return out


def days(rows):
    by_day = {}
    for i, r in enumerate(rows):
        by_day.setdefault(_day(float(r["time"])), []).append(i)
    retransmit = _increases(rows, "link_bytes_retransmit")
    invalid = _increases(rows, "link_bytes_invalid")
    cycles = dryer_cycles(rows)
    out = []
    for day in sorted(by_day):
        idx = by_day[day]
        health = set()
        for i in idx:
            r = rows[i]
            health.update(x for x in (r.get("health_failing") or "").split(";") if x)
            health.update(x for x in (r.get("health_latched") or "").split(";") if x)
        connected = sum(_worth(rows, i) for i in idx if rows[i].get("version"))
        printing = sum(_worth(rows, i) for i in idx if rows[i]["print_state"] in PRINTING)
        out.append(
            {
                "day": day,
                "connected_h": connected / 3600.0,
                "printing_h": printing / 3600.0,
                "retransmit": retransmit.get(day, 0),
                "invalid": invalid.get(day, 0),
                "health": sorted(health),
                "dryer_cycles": cycles.get(day, 0),
            }
        )
    return out


def dryer_during_print(rows):
    return any(r["print_state"] in PRINTING and r.get("dryer_state") == "heating" for r in rows)


def totals(prints, days, dryer_during_print):
    lanes = sorted({n for p in prints for n in p["lanes"]})
    t = {
        "printing_h": sum(p["hours"] for p in prints),
        "prints": len(prints),
        "lanes": lanes,
        "dryer_cycles": sum(d["dryer_cycles"] for d in days),
        "dryer_during_print": dryer_during_print,
    }
    t["met"] = (
        t["printing_h"] >= TARGET_H
        and t["prints"] >= TARGET_PRINTS
        and lanes == list(range(1, LANES + 1))
        and t["dryer_cycles"] >= TARGET_DRY
        and dryer_during_print
    )
    return t


def _when(t):
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(t))


def _cell(text):
    """Text safe in one Markdown table cell."""
    return " ".join(str(text).replace("|", "\\|").splitlines())


def render(prints, days, totals, events, skipped=None):
    lines = ["# ace2k burn-in report", "", "## Totals", ""]
    lines.append(
        f"- printing: {totals['printing_h']:.1f} h of {TARGET_H:.0f};"
        f" prints: {totals['prints']} of {TARGET_PRINTS}; lanes: {totals['lanes']};"
        f" dryer cycles: {totals['dryer_cycles']} of {TARGET_DRY};"
        f" one during a print: {'yes' if totals['dryer_during_print'] else 'no'}"
        f" — **{'targets met' if totals['met'] else 'targets not met'}**"
    )
    skipped = skipped or {}
    if skipped.get("events") or skipped.get("rows"):
        lines.append(
            f"- skipped: {skipped.get('events', 0)} event lines, {skipped.get('rows', 0)} rows"
        )
    lines += [
        "",
        "## Prints",
        "",
        "| start | hours | lanes | bursts | events |",
        "|---|---|---|---|---|",
    ]
    for p in prints:
        lines.append(
            f"| {_when(p['start'])} | {p['hours']:.2f} | {p['lanes']} | {p['bursts']}"
            f" | {len(p['events'])} |"
        )
    lines += [
        "",
        "## Days",
        "",
        "| day | connected h | printing h | retransmit B | invalid B | health | dryer |",
        "|---|---|---|---|---|---|---|",
    ]
    for d in days:
        lines.append(
            f"| {d['day']} | {d['connected_h']:.2f} | {d['printing_h']:.2f}"
            f" | {d['retransmit']} | {d['invalid']} | {';'.join(d['health']) or '-'}"
            f" | {d['dryer_cycles']} |"
        )
    lines += [
        "",
        "## Events to classify",
        "",
        "| time | event | class | note |",
        "|---|---|---|---|",
    ]
    for e in events:
        lines.append(f"| {_when(e.get('time', 0.0))} | {_cell(e.get('message', ''))} |  |  |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("directory")
    args = ap.parse_args(argv)
    skipped = {}
    rows = load_rows(args.directory, skipped)
    events = load_events(args.directory, skipped)
    p = prints(rows, events)
    d = days(rows)
    print(render(p, d, totals(p, d, dryer_during_print(rows)), events, skipped), end="")


if __name__ == "__main__":
    main()
