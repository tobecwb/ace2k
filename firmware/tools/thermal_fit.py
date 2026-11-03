#!/usr/bin/env python3
"""Fit the dryer's thermal model to the heater runs measured on the unit, for the host tests.

The model, per side s (left, right), with u the firing fraction (duty / 100 while the gate is
leased, 0 otherwise) and C the chamber:

    dP_s/dt = a_s * u * (1 + x_s * min(P_s - C, D_MAX)) - k_s * (P_s - C)
                                              the PTC and its heatsink, cooled by the fans' air
    dN_s/dt = (P_s - N_s) / tau_s             the outlet NTC, one lag behind the heatsink
    dC/dt   = h * ((N_l + N_r) / 2 - C) - m * (C - T_amb)

Two nodes per side give the lag from the gate to the NTC, and the overshoot after the gate
stops.  The chamber closes the loop.

The heater's power grows with its temperature above the air.  The runs rise about 0.12, 0.4 and
0.8 degC/s at 20, 35 and 50 % duty, which is steeper than the duty alone.  The model makes the
power grow linearly with P - C over the measured range.  Beyond that range it holds the power at
its highest measured value.  D_MAX is the largest P - C that the fitted model reaches in the
runs, rounded up.  So the model never extrapolates a growth that was not measured.

The fit runs in three stages:

1. Each side is fitted on its own, with the measured chamber as its inlet.
2. The chamber is fitted against the measured NTCs.
3. The coupled model is replayed at the firmware's 10 ms tick, and its error is reported per run.

The fit uses Nelder-Mead in log space, because every parameter is positive.  It is written out
here, so the tool needs only the standard library.

Usage (from firmware/):
    python3 tools/thermal_fit.py DATA_DIR [DATA_DIR ...] \
        --params tests/thermal_params.h --runs tests/thermal_runs.h [--check]

The committed headers name the directories they were fitted to, by their own names, and
tests/python/test_thermal_fit.py checks them against the same directories.

Which runs are fitted:

- Every `run-*.csv` of every directory given is fitted.  The directories are taken in the order
  given, and the runs by name within a directory.  Other files are not read.
- The model's ambient is the chamber temperature at the start of the first run.
- The warm starts listed in EXCLUDED_RUNS are left out.  They are recognised by their content,
  wherever they are read from.  The content is a SHA-256 over the file with its line endings
  normalised.  A file named as an excluded run but with other content stops the fit.  It is
  never fitted by mistake.

Two logging formats are read.  Their header tells them apart:

- The first heater runs (tests/data/heater-2026-09-13) have `state` HEATING, the `duty` in
  force, and a `fans` bit mask.
- The later dryer runs have `state` leased, `fan_l` and `fan_r`, and the running total of
  half-cycles `fired`.  The firing fraction of each interval is taken from that total.  It is
  what the gate really did between two rows, whatever the start and end of the lease.

How a run of the later format is cut:

- The run starts at the row before its fans first read high.  So the half-cycles fired while the
  fans came on are in the run.
- A row that cannot be timed is skipped.  That is a row with no mains frequency, a time that
  does not advance, or a value that is not a number.
- The run ends at a `latched` row.  That row is kept, with the firing before it.
- The run also ends where the drive is over.  A "quiet" row has neither fan reading high, the
  heater idle, and nothing fired since the row before.  The drive is over at a quiet row
  followed by a second quiet row.  It is also over at a quiet row followed by a row with the
  heater leased again, whatever its fans read.  That is a later, separate drive, and a file may
  go on into one.
- Any other row without both fans is a glitch in reading the fans back.  Only that row is
  skipped.  This happens while firing, and in the tail of the cool-down when the fans read high
  again on the next row with the heater still idle.

The tool prints the fitted parameters, and the RMS error of the coupled replay per run and per
signal.  It also prints the rise-rate ceiling that the dryer's protection against an impossible
response uses (ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT).
The data files are in tests/data/.  Each directory's README.md describes the columns.
"""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import math
import os
import sys
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    # One fitted row of a run: (t_s, u, ntc_left_c, ntc_right_c, chamber_c)
    Row = tuple[float, float, float, float, float]
    # One timed row of a dryer run file, as bench_row reads it: (t_s, zc_hz, fired, ntc_left_c,
    # ntc_right_c, chamber_c, fans_both, state)
    BenchRow = tuple[float, float, int, float, float, float, bool, str]

RUN_GLOB = "run-*.csv"
# Runs kept as data but not fitted.  The model starts every run with the heatsink at the
# temperature of its NTC (simulate_side, Model.set).  These two runs began warm, right after the
# fans had cooled the unit.  The heatsinks were still warmer than the NTCs the fans had just
# cooled.  So the model rises where the NTCs first fell.  The model can only replay a cold start.
# The key is the SHA-256 of the file, with its line endings normalised.  So a copy read from
# another path is still left out.  The value is the run's name, as run_paths() gives it.
EXCLUDED_RUNS = {
    "09c4eb4f08150e6da55cfda2b236e6672f465d1bf229a9ab56363f8286bc32f5": "dryer-b2/run-35-to50.csv",
    "a62905477af0a0fb339beedc1558e45ce67139608929827cd408f82d6a2d1b6f": "dryer-b2/run-50-to50.csv",
}
FIT_DT_S = 0.1  # the fit's integration step
TICK_DT_S = 0.01  # the replay's step: the firmware's 10 ms tick
FANS_BOTH = 3  # in the `fans` column of the first heater runs: both fan pins high
# In the later dryer runs, `zc_hz` is the mains frequency, with two half-cycles per cycle
HALVES_PER_EDGE = 2
RISE_DUTY_PCT = 90  # the firmware's duty ceiling
RISE_SPAN_S = 600.0  # long enough for the rise rate to peak
RISE_MARGIN = 1.5
MC_PER_C = 1000.0
NM_ITERS = 2000
CHAMBER_OFFSET_C = 7.0  # assumed: how far the outlet NTCs sit above the chamber when steady
CHAMBER_RISE_C = 30.0  # assumed: how far the chamber sits above the room when steady


def load_run(path: str) -> list[Row]:
    """Return the rows of one run with both fans on, as (t_s, u, ntc_left_c, ntc_right_c,
    chamber_c).

    In the file, ntc1 is the right outlet NTC and ntc2 the left one (README.md).
    """
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if rows and "fan_l" in rows[0]:
        return load_bench_run(rows)
    return load_test_image_run(rows)


def load_test_image_run(rows: list[dict[str, str]]) -> list[Row]:
    """Read a run in the format of the first heater runs.  u is the duty in force while
    HEATING, else 0."""
    out: list[Row] = []
    for r in rows:
        if int(r["fans"]) != FANS_BOTH:
            continue
        u = float(r["duty"]) / 100.0 if r["state"] == "HEATING" else 0.0
        out.append(
            (
                float(r["t_s"]),
                u,
                float(r["ntc2_c"]),
                float(r["ntc1_c"]),
                float(r["env_c"]),
            )
        )
    return out


def bench_row(r: dict[str, str]) -> BenchRow | None:
    """Return a row of a dryer run as (t_s, zc_hz, fired, ntc_left_c, ntc_right_c, chamber_c,
    fans_both, state).

    It returns None when a value is not a number.
    """
    try:
        return (
            float(r["t_s"]),
            float(r["zc_hz"]),
            int(r["fired"]),
            float(r["ntc2_c"]),
            float(r["ntc1_c"]),
            float(r["env_c"]),
            r["fan_l"] == "1" and r["fan_r"] == "1",
            r["state"],
        )
    except (KeyError, TypeError, ValueError):
        return None


def timed_rows(rows: list[dict[str, str]]) -> list[BenchRow]:
    """Return the rows that can be timed.

    Such a row has a number in every column, a mains frequency, and a time later than the row
    kept before it.
    """
    out: list[BenchRow] = []
    for r in rows:
        b = bench_row(r)
        if b is None or b[1] <= 0.0:
            continue
        if out and b[0] <= out[-1][0]:
            continue
        out.append(b)
    return out


def fans_stretch(rows: list[BenchRow]) -> list[BenchRow]:
    """Return the first stretch of rows with both fans on.

    The stretch starts at the row before the fans come on, so the half-cycles fired while they
    came on are counted.  It ends as the module's docstring says: at a `latched` row, which is
    kept, at two quiet rows, or at a quiet row followed by the heater leased again.
    """
    first = next((i for i, b in enumerate(rows) if b[6]), None)
    if first is None:
        return []
    seg = rows[max(first - 1, 0) : first + 1]
    quiet_seen = False
    prev = rows[first]
    for b in rows[first + 1 :]:
        if b[7] == "latched":
            seg.append(b)
            break
        if quiet_seen and b[7] == "leased":
            break  # a later, separate drive
        quiet = not b[6] and b[7] == "idle" and b[2] == prev[2]
        prev = b
        if b[6]:
            quiet_seen = False
            seg.append(b)
        elif quiet:
            if quiet_seen:
                break  # the drive is over
            quiet_seen = True
    return seg


def load_bench_run(rows: list[dict[str, str]]) -> list[Row]:
    """Read a run in the format of the later dryer runs, cut by fans_stretch.

    A row's u is the fraction of half-cycles fired between it and the next row.  It is rounded to
    a whole percent, as the host tests replay it.
    """
    seg = fans_stretch(timed_rows(rows))
    out: list[Row] = []
    for i, (t, zc, fired, left, right, cham, _, _) in enumerate(seg):
        u = 0.0
        if i + 1 < len(seg):
            halves = (seg[i + 1][0] - t) * zc * HALVES_PER_EDGE
            u = round(100.0 * (seg[i + 1][2] - fired) / halves) / 100.0
        out.append((t, u, left, right, cham))
    return out


def content_id(path: str) -> str:
    """Return the SHA-256 of the file, in hex, with its line endings normalised to LF."""
    with open(path, "rb") as f:
        data = f.read().replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


class ExcludedRunChangedError(Exception):
    """A file named as an excluded run holds other content.  Fitting it silently would be
    wrong."""


def excluded(name: str, path: str) -> bool:
    """Return True for the content of an excluded run.

    A file with the name of an excluded run but other content raises ExcludedRunChangedError.
    """
    if content_id(path) in EXCLUDED_RUNS:
        return True
    if name in EXCLUDED_RUNS.values():
        raise ExcludedRunChangedError(
            f"{path} is named as the excluded run {name} but its content differs; "
            "update EXCLUDED_RUNS or rename the file"
        )
    return False


def run_paths(data_dirs: list[str]) -> list[tuple[str, str]]:
    """Return (name, path) of every run in the directories, without the runs of EXCLUDED_RUNS.

    A run's name starts with the name of its directory.
    """
    found: list[tuple[str, str]] = []
    for d in data_dirs:
        base = os.path.basename(os.path.normpath(d))
        for n in sorted(glob.glob(os.path.join(d, RUN_GLOB))):
            name = f"{base}/{os.path.basename(n)}"
            if not excluded(name, n):
                found.append((name, n))
    return found


def simulate_side(
    p: Sequence[float], run: list[Row], side: int, dt: float = FIT_DT_S, d_max: float = math.inf
) -> list[float]:
    """Return the side's NTC at the time of every row, with the chamber taken from the run.

    This is stage 1 of the fit.
    """
    a, k, tau, x = p
    idx = 2 if side == 0 else 3
    pt = nt = run[0][idx]
    t = run[0][0]
    out = [nt]
    for i in range(1, len(run)):
        u, cham = run[i - 1][1], run[i - 1][4]
        while t < run[i][0] - 1e-9:
            h = min(dt, run[i][0] - t)
            d = pt - cham
            dp = a * u * (1.0 + x * min(d, d_max)) - k * d
            dn = (pt - nt) / tau
            pt += dp * h
            nt += dn * h
            t += h
        out.append(nt)
    return out


def simulate_chamber(q: Sequence[float], run: list[Row], dt: float = FIT_DT_S) -> list[float]:
    """Return the chamber at the time of every row, with the NTCs taken from the run.

    This is stage 2 of the fit.
    """
    h_c, m_c, t_amb = q
    c = run[0][4]
    t = run[0][0]
    out = [c]
    for i in range(1, len(run)):
        mean_n = (run[i - 1][2] + run[i - 1][3]) / 2.0
        while t < run[i][0] - 1e-9:
            h = min(dt, run[i][0] - t)
            c += (h_c * (mean_n - c) - m_c * (c - t_amb)) * h
            t += h
        out.append(c)
    return out


class Model:
    """The coupled model.  It uses the same equations as tests/fake_thermal.h."""

    def __init__(
        self, side_l: Sequence[float], side_r: Sequence[float], cham: Sequence[float], d_max: float
    ) -> None:
        self.x = (side_l[3], side_r[3])
        self.d_max = d_max
        self.a = (side_l[0], side_r[0])
        self.k = (side_l[1], side_r[1])
        self.tau = (side_l[2], side_r[2])
        self.h_c, self.m_c, self.t_amb = cham
        self.p = [0.0, 0.0]
        self.n = [0.0, 0.0]
        self.c = 0.0

    def set(self, left: float, right: float, chamber: float) -> None:
        """Start the model with each heatsink at its NTC's temperature, and the chamber's."""
        self.p = [left, right]
        self.n = [left, right]
        self.c = chamber

    def step(self, u: float, dt: float) -> None:
        """Advance the model by dt seconds at firing fraction u."""
        dp: list[float] = []
        for s in (0, 1):
            d = self.p[s] - self.c
            dp.append(self.a[s] * u * (1.0 + self.x[s] * min(d, self.d_max)) - self.k[s] * d)
        dn = [(self.p[s] - self.n[s]) / self.tau[s] for s in (0, 1)]
        mean_n = (self.n[0] + self.n[1]) / 2.0
        dc = self.h_c * (mean_n - self.c) - self.m_c * (self.c - self.t_amb)
        for s in (0, 1):
            self.p[s] += dp[s] * dt
            self.n[s] += dn[s] * dt
        self.c += dc * dt


def replay(model: Model, run: list[Row], dt: float = TICK_DT_S) -> tuple[list[float], float]:
    """Replay the run through the coupled model at the tick.

    It returns the RMS error in °C of the left NTC, the right NTC and the chamber, and the peak
    NTC error.  The first row sets the model, so its error is 0.  It still counts in the mean: the
    RMS is over every row of the run, the first included.
    """
    model.set(run[0][2], run[0][3], run[0][4])
    t = run[0][0]
    err = [[0.0, 0.0, 0.0] for _ in run]
    for i in range(1, len(run)):
        u = run[i - 1][1]
        while t < run[i][0] - 1e-9:
            h = min(dt, run[i][0] - t)
            model.step(u, h)
            t += h
        err[i] = [model.n[0] - run[i][2], model.n[1] - run[i][3], model.c - run[i][4]]
    rms = [math.sqrt(sum(e[j] ** 2 for e in err) / len(err)) for j in range(3)]
    peak = max(max(abs(e[0]), abs(e[1])) for e in err)
    return rms, peak


def _beyond(a: list[float], b: list[float], c: float) -> list[float]:
    """Return the point a + c * (a - b), element by element.  It lies past a, away from b.

    With c = 1 it reflects b through a.  With c = 2 it goes twice as far.
    """
    return [a[j] + c * (a[j] - b[j]) for j in range(len(a))]


def _toward(a: list[float], b: list[float], c: float) -> list[float]:
    """Return the point a + c * (b - a), element by element: the fraction c of the way from a
    to b."""
    return [a[j] + c * (b[j] - a[j]) for j in range(len(a))]


def _centroid(simplex: list[list[float]]) -> list[float]:
    """Return the mean of every point of the simplex except the last one, which is the worst."""
    n = len(simplex) - 1
    return [sum(simplex[i][j] for i in range(n)) / n for j in range(n)]


def _sort(simplex: list[list[float]], values: list[float]) -> tuple[list[list[float]], list[float]]:
    """Return the simplex and its values, sorted by value, the lowest first."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    return [simplex[i] for i in order], [values[i] for i in order]


def _step(
    f: Callable[[list[float]], float], simplex: list[list[float]], values: list[float]
) -> tuple[list[list[float]], list[float]]:
    """Take one Nelder-Mead step on a sorted simplex: reflect, expand, contract or shrink.

    It returns the new simplex and its values.  They are not sorted.
    """
    centroid = _centroid(simplex)
    worst = simplex[-1]
    refl = _beyond(centroid, worst, 1.0)
    fr = f(refl)
    if fr < values[0]:
        exp = _beyond(centroid, worst, 2.0)
        fe = f(exp)
        last = (exp, fe) if fe < fr else (refl, fr)
    elif fr < values[-2]:
        last = (refl, fr)
    else:
        con = _toward(centroid, worst, 0.5)
        fc = f(con)
        if fc < values[-1]:
            last = (con, fc)
        else:
            best = simplex[0]
            shrunk = [_toward(best, s, 0.5) for s in simplex[1:]]
            return [best, *shrunk], [values[0]] + [f(s) for s in shrunk]
    return [*simplex[:-1], last[0]], [*values[:-1], last[1]]


def nelder_mead(
    f: Callable[[list[float]], float],
    x0: Sequence[float],
    step: float = 0.3,
    iters: int = 600,
    tol: float = 1e-9,
) -> tuple[list[float], float]:
    """Minimise f over R^n.  Return the best point and its value.

    x0 and the result are in the caller's space, which here is the log of the parameters.
    """
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (step if j == i else 0.0) for j in range(n)] for i in range(n)]
    values = [f(x) for x in simplex]
    for _ in range(iters):
        simplex, values = _sort(simplex, values)
        if abs(values[-1] - values[0]) < tol:
            break
        simplex, values = _step(f, simplex, values)
    i = min(range(n + 1), key=lambda i: values[i])
    return simplex[i], values[i]


def sse(sim: list[float], run: list[Row], idx: int) -> float:
    """Return the sum of squared errors between sim and column idx of run."""
    return sum((s - r[idx]) ** 2 for s, r in zip(sim, run))


def fit_side(
    runs: list[list[Row]], side: int, x0: Sequence[float] = (1.0, 0.04, 15.0, 0.1)
) -> tuple[float, ...]:
    """Fit one side's parameters (a, k, tau, x) to all runs, and return them."""
    idx = 2 if side == 0 else 3

    def cost(lx: list[float]) -> float:
        p = [math.exp(v) for v in lx]
        return sum(sse(simulate_side(p, r, side), r, idx) for r in runs)

    best, _ = nelder_mead(cost, [math.log(v) for v in x0], iters=NM_ITERS)
    best, _ = nelder_mead(cost, best, step=0.1, iters=NM_ITERS)  # restart from the first best
    return tuple(math.exp(v) for v in best)


def max_excursion(p: Sequence[float], runs: list[list[Row]], side: int) -> float:
    """Return the largest P - C that the fitted side reaches in the runs.

    This is the range over which the heater's power was measured.
    """
    a, k, tau, x = p
    idx = 2 if side == 0 else 3
    top = 0.0
    for run in runs:
        pt = run[0][idx]
        t = run[0][0]
        for i in range(1, len(run)):
            u, cham = run[i - 1][1], run[i - 1][4]
            while t < run[i][0] - 1e-9:
                h = min(FIT_DT_S, run[i][0] - t)
                d = pt - cham
                pt += (a * u * (1.0 + x * d) - k * d) * h
                t += h
                top = max(top, pt - cham)
    return top


def fit_chamber(runs: list[list[Row]], x0: Sequence[float] = (0.01,)) -> tuple[float, ...]:
    """Fit the chamber and return (h, m, ambient).

    h is fitted.  m is assumed.  The ambient is the chamber at the start of the first run, when
    the unit is cold and at room temperature.

    The runs are too short, 150 s at most, to show the chamber's heat loss to the room.  So m is
    set, not fitted.  It is chosen so that a chamber CHAMBER_OFFSET_C below the outlet NTCs is in
    balance with a chamber CHAMBER_RISE_C above the room.  That offset is the drive target's
    offset over the chamber target, at a mid-range 55 degC in a 25 degC room.  Measure m on the
    unit, then run the fit again.
    """
    t_amb = runs[0][0][4]

    def q_of(lh: list[float]) -> list[float]:
        h_c = math.exp(lh[0])
        return [h_c, h_c * CHAMBER_OFFSET_C / CHAMBER_RISE_C, t_amb]

    def cost(lh: list[float]) -> float:
        return sum(sse(simulate_chamber(q_of(lh), r), r, 4) for r in runs)

    best, _ = nelder_mead(cost, [math.log(v) for v in x0], iters=NM_ITERS)
    best, _ = nelder_mead(cost, best, step=0.1, iters=NM_ITERS)
    return tuple(q_of(best))


def rise_ceiling(model: Model) -> int:
    """Return the rise-rate ceiling, in m°C/s per % of duty.

    It is the steepest NTC rise the model produces with both fans on at the duty ceiling, divided
    by that duty and multiplied by RISE_MARGIN.
    """
    model.set(model.t_amb, model.t_amb, model.t_amb)
    u = RISE_DUTY_PCT / 100.0
    steepest = 0.0
    steps = int(RISE_SPAN_S / TICK_DT_S)
    for _ in range(steps):
        before = max(model.n)
        model.step(u, TICK_DT_S)
        steepest = max(steepest, (max(model.n) - before) / TICK_DT_S)
    return int(round(steepest * MC_PER_C / RISE_DUTY_PCT * RISE_MARGIN))


def sources(data_dirs: list[str]) -> list[str]:
    """Return the comment lines that name the fitted data directories, one per line.

    Each directory is named by its path relative to firmware/, such as tests/data/dryer-b2.  A
    directory outside firmware/ is named by its own name only.  So no path of the machine ever
    lands in a generated header.
    """
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rel: list[str] = []
    for d in data_dirs:
        path = os.path.relpath(os.path.abspath(d), root)
        rel.append(os.path.basename(os.path.abspath(d)) if path.startswith("..") else path)
    return [f" *   {r}" for r in rel]


def render_params(model: Model, ceiling: int, data_dirs: list[str]) -> str:
    """Return the header of the fitted parameters: those of model, and the rise ceiling."""
    lines = [
        "/* Generated by tools/thermal_fit.py, do not edit.  The dryer's thermal model fitted to",
        " * the heater runs of",
        *sources(data_dirs),
        " * (tests/fake_thermal.h has the equations).",
        " * Units: a in degC/s at full duty, k in 1/s, tau in s, x in 1/degC, D_MAX in degC,",
        " * h and m in 1/s, ambient in degC.  m is assumed, not fitted (tools/thermal_fit.py). */",
        "#ifndef ACE2K_TEST_THERMAL_PARAMS_H",
        "#define ACE2K_TEST_THERMAL_PARAMS_H",
        "// clang-format off",
        "// NOLINTBEGIN",
    ]
    for name, s in (("LEFT", 0), ("RIGHT", 1)):
        a, k, tau, x = model.a[s], model.k[s], model.tau[s], model.x[s]
        lines.append(f"#define THERMAL_{name}_A   {a:.6f}")
        lines.append(f"#define THERMAL_{name}_K   {k:.6f}")
        lines.append(f"#define THERMAL_{name}_TAU {tau:.6f}")
        lines.append(f"#define THERMAL_{name}_X   {x:.6f}")
    lines.append(f"#define THERMAL_D_MAX {model.d_max:.1f}")
    lines.append(f"#define THERMAL_CHAMBER_H {model.h_c:.6f}")
    lines.append(f"#define THERMAL_CHAMBER_M {model.m_c:.6f}")
    lines.append(f"#define THERMAL_BENCH_AMBIENT_C {model.t_amb:.6f}")
    lines.append(
        f"#define THERMAL_RISE_MAX_MC_S_PER_PCT {ceiling} /* copied into dryer.h by hand */"
    )
    lines.append("// NOLINTEND")
    lines.append("// clang-format on")
    lines.append("#endif")
    return "\n".join(lines) + "\n"


def render_runs(names: list[str], runs: list[list[Row]], data_dirs: list[str]) -> str:
    """Return the header that holds the runs, for the C tests to replay."""
    lines = [
        "/* Generated by tools/thermal_fit.py, do not edit.  The heater runs of",
        *sources(data_dirs),
        " * the rows with both fans on: time in ms, the firing duty in %, the left and right",
        " * outlet NTCs and the chamber in millidegrees C. */",
        "#ifndef ACE2K_TEST_THERMAL_RUNS_H",
        "#define ACE2K_TEST_THERMAL_RUNS_H",
        "#include <stdint.h>",
        "// clang-format off",
        "// NOLINTBEGIN",
        "struct thermal_sample {",
        "    uint32_t t_ms;",
        "    uint8_t duty_pct;",
        "    int32_t left_mc, right_mc, chamber_mc;",
        "};",
        "struct thermal_run { const char *name; const struct thermal_sample *s; unsigned n; };",
    ]
    for k, run in enumerate(runs):
        t0 = run[0][0]
        lines.append(f"static const struct thermal_sample thermal_run{k}[] = {{")
        for t, u, nl, nr, c in run:
            lines.append(
                f"    {{ {int(round((t - t0) * 1000))}U, {int(round(u * 100))}U, "
                f"{int(round(nl * MC_PER_C))}, {int(round(nr * MC_PER_C))}, "
                f"{int(round(c * MC_PER_C))} }},"
            )
        lines.append("};")
    lines.append("static const struct thermal_run thermal_runs[] = {")
    for k, name in enumerate(names):
        lines.append(
            f'    {{ "{name}", thermal_run{k}, sizeof thermal_run{k} / sizeof thermal_run{k}[0] }},'
        )
    lines.append("};")
    lines.append("#define THERMAL_RUN_COUNT (sizeof thermal_runs / sizeof thermal_runs[0])")
    lines.append("// NOLINTEND")
    lines.append("// clang-format on")
    lines.append("#endif")
    return "\n".join(lines) + "\n"


def emit(path: str, text: str, check: bool) -> bool:
    """Write text to path, or with check compare path with it.

    It returns True when path is current, or now is.
    """
    if check:
        with open(path, encoding="utf-8") as f:
            if f.read() != text:
                print(
                    f"thermal_fit: {path} is out of date; regenerate it",
                    file=sys.stderr,
                )
                return False
        print(f"thermal_fit: {path} is current")
        return True
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"thermal_fit: wrote {path}")
    return True


def main(argv: list[str]) -> int:
    """Fit the model, print the results, and write or check the headers.  Return the status."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("data_dirs", nargs="+", metavar="data_dir")
    ap.add_argument("--params")
    ap.add_argument("--runs")
    ap.add_argument("--check", action="store_true", help="compare the headers instead of writing")
    args = ap.parse_args(argv)
    try:
        found = run_paths(args.data_dirs)
    except ExcludedRunChangedError as e:
        print(f"thermal_fit: {e}", file=sys.stderr)
        return 1
    if not found:
        print("thermal_fit: no run-*.csv in the directories given", file=sys.stderr)
        return 1
    names = [n for n, _ in found]
    runs = [load_run(p) for _, p in found]
    side_l = fit_side(runs, 0)
    side_r = fit_side(runs, 1)
    cham = fit_chamber(runs)
    d_max = math.ceil(max(max_excursion(side_l, runs, 0), max_excursion(side_r, runs, 1)))
    model = Model(side_l, side_r, cham, d_max)
    for name, p in (("left", side_l), ("right", side_r)):
        print(f"{name}: a={p[0]:.6f} k={p[1]:.6f} tau={p[2]:.3f} x={p[3]:.6f}")
    print(f"d_max={d_max}")
    print(f"chamber: h={cham[0]:.6f} m={cham[1]:.6f} ambient={cham[2]:.3f}")
    for name, run in zip(names, runs):
        rms, peak = replay(model, run)
        print(
            f"{name}: rms left={rms[0]:.3f} right={rms[1]:.3f} chamber={rms[2]:.3f} peak={peak:.3f}"
        )
    ceiling = rise_ceiling(Model(side_l, side_r, cham, d_max))
    print(f"ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT {ceiling}")
    ok = True
    if args.params:
        ok = (
            emit(
                args.params,
                render_params(model, ceiling, args.data_dirs),
                args.check,
            )
            and ok
        )
    if args.runs:
        ok = emit(args.runs, render_runs(names, runs, args.data_dirs), args.check) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
