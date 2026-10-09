#!/usr/bin/env python3
"""Fit the dryer's thermal model to the heater runs measured on the unit, for the host tests.

The model, per side s (left, right), with u the firing fraction (duty / 100 while the gate is
leased, 0 otherwise) and C the chamber:

    dP_s/dt = a_s * u * (1 + x_s * min(P_s - C, D_MAX)) - k_s * (P_s - C)
                                              the PTC and its heatsink, cooled by the fans' air
    dN_s/dt = (P_s - N_s) / tau_s             the outlet NTC, one lag behind the heatsink
    dC/dt   = h * ((N_l + N_r) / 2 - C) - m * (C - T_amb)

Two nodes per side give the lag from the gate to the NTC and the overshoot after the gate stops;
the chamber closes the loop.  The heater's power grows with its temperature above the air (the
runs rise ~0.12, ~0.4 and ~0.8 degC/s at 20, 35 and 50 %: steeper than the duty) — linear in
P - C over the measured range, and held at its highest measured value (D_MAX, the largest
P - C the fitted model reaches in the runs, rounded up) beyond it, so the model never
extrapolates a growth the bench did not see.  The sides are fitted first, each against the
measured chamber as its inlet; the chamber is fitted next, against the measured NTCs; the
coupled model is then replayed at the firmware's 10 ms tick and its error reported per run.
Nelder-Mead in log space (every parameter is positive), written out: the standard library only.

Usage (from firmware/):
    python3 tools/thermal_fit.py DATA_DIR [DATA_DIR ...] \
        --params tests/thermal_params.h --runs tests/thermal_runs.h [--check]

The committed headers name the directories they were fitted to, by their own names, and
tests/python/test_thermal_fit.py checks them against the same directories.

Every `run-*.csv` of every directory given is fitted, in the order given and by name within a
directory (the first run's starting chamber is the model's ambient), except the warm starts of
EXCLUDED_RUNS, which are recognised by their content (a SHA-256 over the file with its line endings
normalised), wherever they are read from — a file named as an excluded run whose content differs
stops the fit rather than be fitted; other files are not read.
Two logging formats are read, told apart by their header: the 2026-09-13 test image's
(`state` HEATING, the `duty` in force, a `fans` bit mask) and the dryer bench's (`state`
leased, `fan_l`/`fan_r`, and the cumulative `fired` count, from which each interval's firing
fraction is taken — what the gate actually did between two rows, whatever the lease's edges).
A bench run starts at the row before its fans first read high, so the half-cycles fired while
the fans came on are in the run; rows that cannot be timed (no mains frequency, a time that does
not advance, a value that is not a number) are skipped.  A run ends at a `latched` row (kept, with
the firing before it), or where the drive is over: a "quiet" row — both fans not reading high,
the heater idle, nothing fired since the row before it — followed by a second quiet row, or by a
row with the heater leased again, whatever its fans read (a later, separate drive; a file may go
on into one).  Any other row without
both fans is a fan read-back glitch and only that row is skipped — while firing, and in the
cool-down's tail when the fans read high again on the next row with the heater still idle.

Prints the fitted parameters, the RMS error of the coupled replay per run and signal, and the
rise-rate ceiling the dryer's impossible-response protection uses
(ACE2K_DRYER_RISE_MAX_MC_S_PER_PCT).
The data files are in tests/data/ (each directory's README.md has the columns).
"""

import argparse
import csv
import glob
import hashlib
import math
import os
import sys

RUN_GLOB = "run-*.csv"
# Runs kept as data but not fitted.  The model starts every run with the heatsink at its NTC's
# temperature (simulate_side, Model.set); these two began warm, straight after a fan cool-down,
# with the heatsinks still above the NTCs the fans had just cooled — the model rises where the
# NTCs first fell.  A cold start is what the model can replay.
# Keyed on the SHA-256 of the file (line endings normalised), so a copy read from another path is
# still left out; the name is the run's as run_paths() gives it.
EXCLUDED_RUNS = {
    "09c4eb4f08150e6da55cfda2b236e6672f465d1bf229a9ab56363f8286bc32f5": "dryer-b2/run-35-to50.csv",
    "a62905477af0a0fb339beedc1558e45ce67139608929827cd408f82d6a2d1b6f": "dryer-b2/run-50-to50.csv",
}
FIT_DT_S = 0.1  # the fit's integration step
TICK_DT_S = 0.01  # the replay's step: the firmware's 10 ms tick
FANS_BOTH = 3  # the 2026-09-13 runs' `fans` column: both fan pins high
HALVES_PER_EDGE = 2  # the dryer bench's `zc_hz` is the mains frequency: two half-cycles a cycle
RISE_DUTY_PCT = 90  # the firmware's duty ceiling
RISE_SPAN_S = 600.0  # long enough for the rise rate to peak
RISE_MARGIN = 1.5
MC_PER_C = 1000.0
NM_ITERS = 2000
CHAMBER_OFFSET_C = 7.0  # assumed: the outlet NTCs above the chamber at steady state
CHAMBER_RISE_C = 30.0  # assumed: the chamber above the room at steady state


def load_run(path):
    """Rows of one run with both fans on: (t_s, u, ntc_left_c, ntc_right_c, chamber_c).

    The file's ntc1 is the right outlet NTC, ntc2 the left (README.md)."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if rows and "fan_l" in rows[0]:
        return load_bench_run(rows)
    return load_test_image_run(rows)


def load_test_image_run(rows):
    """The 2026-09-13 format: u is the duty in force while HEATING."""
    out = []
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


def bench_row(r):
    """(t_s, zc_hz, fired, ntc_left_c, ntc_right_c, chamber_c, fans_both, state) of a row, or
    None when a value is not a number."""
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


def timed_rows(rows):
    """The rows that can be timed: a number in every column, a mains frequency, and a time after
    the previous kept row's."""
    out = []
    for r in rows:
        b = bench_row(r)
        if b is None or b[1] <= 0.0:
            continue
        if out and b[0] <= out[-1][0]:
            continue
        out.append(b)
    return out


def fans_stretch(rows):
    """The first stretch with both fans on, led by the row before it (the half-cycles fired while
    the fans came on are counted from there), to its end as the module's docstring says: a
    `latched` row (kept), two quiet rows, or a quiet row and then the heater leased again."""
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


def load_bench_run(rows):
    """The dryer bench's format (fans_stretch); a row's u is the fraction of half-cycles fired
    between it and the next row, rounded to a whole percent as the host tests replay it."""
    seg = fans_stretch(timed_rows(rows))
    out = []
    for i, (t, zc, fired, left, right, cham, _, _) in enumerate(seg):
        u = 0.0
        if i + 1 < len(seg):
            halves = (seg[i + 1][0] - t) * zc * HALVES_PER_EDGE
            u = round(100.0 * (seg[i + 1][2] - fired) / halves) / 100.0
        out.append((t, u, left, right, cham))
    return out


def content_id(path):
    """The SHA-256 of the file with its line endings normalised to LF, in hex."""
    with open(path, "rb") as f:
        data = f.read().replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


class ExcludedRunChangedError(Exception):
    """A file named as an excluded run holds other content: fitting it silently would be wrong."""


def excluded(name, path):
    """True for an excluded run's content; a file of an excluded run's name with other content
    raises ExcludedRunChangedError."""
    if content_id(path) in EXCLUDED_RUNS:
        return True
    if name in EXCLUDED_RUNS.values():
        raise ExcludedRunChangedError(
            f"{path} is named as the excluded run {name} but its content differs; "
            "update EXCLUDED_RUNS or rename the file"
        )
    return False


def run_paths(data_dirs):
    """(name, path) of every run in the directories, the directory's name leading the run's;
    the runs of EXCLUDED_RUNS left out."""
    found = []
    for d in data_dirs:
        base = os.path.basename(os.path.normpath(d))
        for n in sorted(glob.glob(os.path.join(d, RUN_GLOB))):
            name = f"{base}/{os.path.basename(n)}"
            if not excluded(name, n):
                found.append((name, n))
    return found


def simulate_side(p, run, side, dt=FIT_DT_S, d_max=math.inf):
    """The side's NTC at every row's time, the chamber taken from the run (fit stage 1)."""
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


def simulate_chamber(q, run, dt=FIT_DT_S):
    """The chamber at every row's time, the NTCs taken from the run (fit stage 2)."""
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
    """The coupled model; the same equations as tests/fake_thermal.h."""

    def __init__(self, side_l, side_r, cham, d_max):
        self.x = (side_l[3], side_r[3])
        self.d_max = d_max
        self.a = (side_l[0], side_r[0])
        self.k = (side_l[1], side_r[1])
        self.tau = (side_l[2], side_r[2])
        self.h_c, self.m_c, self.t_amb = cham
        self.p = [0.0, 0.0]
        self.n = [0.0, 0.0]
        self.c = 0.0

    def set(self, left, right, chamber):
        self.p = [left, right]
        self.n = [left, right]
        self.c = chamber

    def step(self, u, dt):
        dp = []
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


def replay(model, run, dt=TICK_DT_S):
    """Coupled replay at the tick; the RMS (°C) of left, right, chamber and the peak error."""
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


def nelder_mead(f, x0, step=0.3, iters=600, tol=1e-9):
    """Minimise f over R^n; x0 and the result in the caller's (log) space."""
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (step if j == i else 0.0) for j in range(n)] for i in range(n)]
    values = [f(x) for x in simplex]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: values[i])
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        if abs(values[-1] - values[0]) < tol:
            break
        centroid = [sum(simplex[i][j] for i in range(n)) / n for j in range(n)]
        worst = simplex[-1]
        refl = [centroid[j] + (centroid[j] - worst[j]) for j in range(n)]
        fr = f(refl)
        if fr < values[0]:
            exp = [centroid[j] + 2.0 * (centroid[j] - worst[j]) for j in range(n)]
            fe = f(exp)
            simplex[-1], values[-1] = (exp, fe) if fe < fr else (refl, fr)
        elif fr < values[-2]:
            simplex[-1], values[-1] = refl, fr
        else:
            con = [centroid[j] + 0.5 * (worst[j] - centroid[j]) for j in range(n)]
            fc = f(con)
            if fc < values[-1]:
                simplex[-1], values[-1] = con, fc
            else:
                best = simplex[0]
                simplex = [best] + [
                    [best[j] + 0.5 * (s[j] - best[j]) for j in range(n)] for s in simplex[1:]
                ]
                values = [values[0]] + [f(s) for s in simplex[1:]]
    i = min(range(n + 1), key=lambda i: values[i])
    return simplex[i], values[i]


def sse(sim, run, idx):
    return sum((s - r[idx]) ** 2 for s, r in zip(sim, run))


def fit_side(runs, side, x0=(1.0, 0.04, 15.0, 0.1)):
    idx = 2 if side == 0 else 3

    def cost(lx):
        p = [math.exp(v) for v in lx]
        return sum(sse(simulate_side(p, r, side), r, idx) for r in runs)

    best, _ = nelder_mead(cost, [math.log(v) for v in x0], iters=NM_ITERS)
    best, _ = nelder_mead(cost, best, step=0.1, iters=NM_ITERS)  # a restart from the first optimum
    return tuple(math.exp(v) for v in best)


def max_excursion(p, runs, side):
    """The largest P - C the fitted side reaches in the runs (the power's measured range)."""
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


def fit_chamber(runs, x0=(0.01,)):
    """h, fitted; m, assumed; the ambient is the first run's chamber at its start (the room, the
    unit cold).  The runs are too short (at most 150 s) for the chamber's loss to the room to show,
    so m is set, not fitted: a chamber CHAMBER_OFFSET_C below the outlet NTCs balances a chamber
    CHAMBER_RISE_C above the room — the drive target's offset over the chamber target, at a
    mid-range 55 degC in a 25 degC room.  A bench run measures it; the fit is re-run then."""
    t_amb = runs[0][0][4]

    def q_of(lh):
        h_c = math.exp(lh[0])
        return [h_c, h_c * CHAMBER_OFFSET_C / CHAMBER_RISE_C, t_amb]

    def cost(lh):
        return sum(sse(simulate_chamber(q_of(lh), r), r, 4) for r in runs)

    best, _ = nelder_mead(cost, [math.log(v) for v in x0], iters=NM_ITERS)
    best, _ = nelder_mead(cost, best, step=0.1, iters=NM_ITERS)
    return tuple(q_of(best))


def rise_ceiling(model):
    """The steepest NTC rise the model produces with both fans on at the duty ceiling, per duty-%,
    times the margin: m°C/s per %."""
    model.set(model.t_amb, model.t_amb, model.t_amb)
    u = RISE_DUTY_PCT / 100.0
    steepest = 0.0
    steps = int(RISE_SPAN_S / TICK_DT_S)
    for _ in range(steps):
        before = max(model.n)
        model.step(u, TICK_DT_S)
        steepest = max(steepest, (max(model.n) - before) / TICK_DT_S)
    return int(round(steepest * MC_PER_C / RISE_DUTY_PCT * RISE_MARGIN))


def sources(data_dirs):
    """The comment lines naming the data directories fitted, one per line, each as its path
    relative to firmware/ (tests/data/dryer-b2); a directory outside firmware/ is named by its
    own name only, so no machine path ever lands in a generated header."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rel = []
    for d in data_dirs:
        path = os.path.relpath(os.path.abspath(d), root)
        rel.append(os.path.basename(os.path.abspath(d)) if path.startswith("..") else path)
    return [f" *   {r}" for r in rel]


def render_params(side_l, side_r, cham, d_max, ceiling, data_dirs):
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
    for name, (a, k, tau, x) in (("LEFT", side_l), ("RIGHT", side_r)):
        lines.append(f"#define THERMAL_{name}_A   {a:.6f}")
        lines.append(f"#define THERMAL_{name}_K   {k:.6f}")
        lines.append(f"#define THERMAL_{name}_TAU {tau:.6f}")
        lines.append(f"#define THERMAL_{name}_X   {x:.6f}")
    lines.append(f"#define THERMAL_D_MAX {d_max:.1f}")
    lines.append(f"#define THERMAL_CHAMBER_H {cham[0]:.6f}")
    lines.append(f"#define THERMAL_CHAMBER_M {cham[1]:.6f}")
    lines.append(f"#define THERMAL_BENCH_AMBIENT_C {cham[2]:.6f}")
    lines.append(
        f"#define THERMAL_RISE_MAX_MC_S_PER_PCT {ceiling} /* copied into dryer.h by hand */"
    )
    lines.append("// NOLINTEND")
    lines.append("// clang-format on")
    lines.append("#endif")
    return "\n".join(lines) + "\n"


def render_runs(names, runs, data_dirs):
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


def emit(path, text, check):
    """Write path, or with check compare it; True when it is (now) current."""
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


def main(argv):
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
                render_params(side_l, side_r, cham, d_max, ceiling, args.data_dirs),
                args.check,
            )
            and ok
        )
    if args.runs:
        ok = emit(args.runs, render_runs(names, runs, args.data_dirs), args.check) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
