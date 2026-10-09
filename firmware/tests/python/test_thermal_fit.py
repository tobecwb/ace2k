"""tools/thermal_fit.py: the fit recovers a model it did not see, and the committed headers are
the fit of the committed runs."""

import subprocess
import sys
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2]
DATA = FIRMWARE / "tests" / "data" / "heater-2026-09-13"
DATA_B2 = FIRMWARE / "tests" / "data" / "dryer-b2"
sys.path.insert(0, str(FIRMWARE / "tools"))
import thermal_fit  # noqa: E402

KNOWN = (0.9, 0.04, 14.0, 0.1)  # a, k, tau, x: a side the fit has never seen
STEP_S = 0.5


def synthetic_run():
    """150 s at 35 % then 60 s off, the chamber held at 25 degC; the left NTC from the model."""
    rows = []
    for i in range(int(210 / STEP_S)):
        t = i * STEP_S
        u = 0.35 if t < 150 else 0.0
        rows.append((t, u, 25.0, 25.0, 25.0))
    left = thermal_fit.simulate_side(KNOWN, rows, 0)
    return [(t, u, n, 25.0, c) for (t, u, _, _, c), n in zip(rows, left)]


def test_the_fit_recovers_a_known_side():
    got = thermal_fit.fit_side([synthetic_run()], 0)
    for want, have in zip(KNOWN, got):
        assert abs(have - want) <= 0.05 * want, (KNOWN, got)


def test_a_side_at_rest_stays_at_its_chamber():
    rows = [(i * STEP_S, 0.0, 30.0, 30.0, 30.0) for i in range(100)]
    assert thermal_fit.simulate_side(KNOWN, rows, 0) == [30.0] * 100


BENCH_HEADER = "t_s,state,duty,fired,reason,zc_hz,ntc1_c,ntc2_c,env_c,env_rh,cutout,fan_l,fan_r\n"


def bench_row(t, state, fired, fans, right=30.0, left=31.0, zc="60.0"):
    duty = 20 if state == "leased" else 0
    return f"{t},{state},{duty},{fired},ok,{zc},{right},{left},28.0,60.0,0,{fans},{fans}\n"


def test_a_bench_run_takes_its_firing_from_the_fired_count(tmp_path):
    """24 half-cycles in a second of 120 is 20 %; the run starts at the row before the fans read
    high, so the 4 half-cycles fired while they came on are kept (4 in 120: 3 %); the rows before
    that are dropped, and the sides are read right = ntc1, left = ntc2."""
    path = tmp_path / "run-20-3s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(-1.0, "idle", 100, 0)
        + bench_row(0.0, "idle", 100, 0)
        + bench_row(1.0, "leased", 104, 1)
        + bench_row(2.0, "leased", 128, 1)
        + bench_row(3.0, "idle", 140, 1)
        + bench_row(4.0, "idle", 140, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert [r[1] for r in run] == [0.03, 0.2, 0.1, 0.0, 0.0]
    assert run[0][2:] == (31.0, 30.0, 28.0)


def test_a_bench_run_keeps_every_half_cycle_it_fired(tmp_path):
    """Every half-cycle fired from the row before the fans to the run's end is in the run (50 Hz
    mains: 100 half-cycles a second, so every interval's fraction is a whole percent)."""
    path = tmp_path / "run-10-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "idle", 0, 0, zc="50.0")
        + bench_row(1.0, "leased", 8, 1, zc="50.0")
        + bench_row(2.0, "leased", 20, 1, zc="50.0")
        + bench_row(3.0, "idle", 22, 1, zc="50.0")
        + bench_row(4.0, "idle", 22, 1, zc="50.0")
    )
    run = thermal_fit.load_run(str(path))
    halves = sum(r[1] * (n[0] - r[0]) * 100 for r, n in zip(run, run[1:]))
    assert round(halves) == 22


def test_a_bench_run_ends_at_its_first_stretch_of_fans(tmp_path):
    """A file that goes on into a later drive contributes only its first one: one quiet row (the
    fans off, the heater idle, nothing fired) followed by the heater leased again ends the run."""
    path = tmp_path / "run-10-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "idle", 12, 1)
        + bench_row(2.0, "idle", 12, 0)
        + bench_row(3.0, "leased", 12, 1)
        + bench_row(4.0, "leased", 36, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0]
    assert run[0][1] == 0.1


def test_two_quiet_rows_without_the_fans_end_a_bench_run(tmp_path):
    """The heater idle, nothing fired and the fans off twice in a row: the drive is over."""
    path = tmp_path / "run-10-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "idle", 12, 1)
        + bench_row(2.0, "idle", 12, 0)
        + bench_row(3.0, "idle", 12, 0)
        + bench_row(4.0, "idle", 12, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0]


def test_a_new_lease_after_a_quiet_row_ends_the_run_whatever_its_fans_read(tmp_path):
    """quiet → leased with the fans not reading high → latched: the run ends at the quiet row;
    the later drive's rows, its latch included, are not in it."""
    path = tmp_path / "run-10-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "idle", 12, 1)
        + bench_row(2.0, "idle", 12, 0)
        + bench_row(3.0, "leased", 12, 0)
        + bench_row(4.0, "latched", 30, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0]


def test_a_quiet_row_is_judged_against_the_row_before_it(tmp_path):
    """idle, fans off, fired moved (not quiet) → idle, fans off, flat twice (quiet, quiet): the
    run ends there, before the later lease."""
    path = tmp_path / "run-20-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "leased", 24, 1)
        + bench_row(2.0, "idle", 30, 0)
        + bench_row(3.0, "idle", 30, 0)
        + bench_row(4.0, "idle", 30, 0)
        + bench_row(5.0, "idle", 30, 0)
        + bench_row(6.0, "leased", 30, 1)
        + bench_row(7.0, "leased", 54, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0]


def test_a_glitch_in_the_cool_down_tail_is_skipped(tmp_path):
    """One quiet row whose next row has the fans high again, the heater still idle: a glitch,
    the cool-down goes on."""
    path = tmp_path / "run-20-2s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "leased", 24, 1)
        + bench_row(2.0, "idle", 48, 1)
        + bench_row(3.0, "idle", 48, 0)
        + bench_row(4.0, "idle", 48, 1)
        + bench_row(5.0, "idle", 48, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0, 2.0, 4.0, 5.0]


def test_a_latched_heater_ends_a_bench_run_with_its_firing(tmp_path):
    """The heater latched: the run ends at that row, the half-cycles fired up to it kept."""
    path = tmp_path / "run-20-3s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "leased", 24, 1)
        + bench_row(2.0, "latched", 36, 1)
        + bench_row(3.0, "idle", 36, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0, 2.0]
    assert [r[1] for r in run] == [0.2, 0.1, 0.0]


def test_a_single_fan_glitch_does_not_end_a_bench_run(tmp_path):
    """A row where a fan did not read back high while the run is still firing is a glitch: it is
    skipped, not the end of the run; its half-cycles fall in the interval around it."""
    path = tmp_path / "run-20-4s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "leased", 24, 1)
        + bench_row(2.0, "leased", 48, 0)
        + bench_row(3.0, "leased", 72, 1)
        + bench_row(4.0, "idle", 72, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0, 3.0, 4.0]
    assert [r[1] for r in run] == [0.2, 0.2, 0.0, 0.0]


def test_rows_that_cannot_be_timed_are_skipped(tmp_path):
    """No mains frequency, a repeated time or a value that is not a number: the row is skipped,
    never a division by zero."""
    path = tmp_path / "run-20-3s.csv"
    path.write_text(
        BENCH_HEADER
        + bench_row(0.0, "leased", 0, 1)
        + bench_row(1.0, "leased", 24, 1, zc="0.0")
        + bench_row(1.0, "leased", 24, 1)
        + bench_row(1.0, "leased", 30, 1)
        + bench_row(1.5, "leased", "?", 1)
        + bench_row(2.0, "leased", 48, 1)
    )
    run = thermal_fit.load_run(str(path))
    assert [r[0] for r in run] == [0.0, 1.0, 2.0]
    assert [r[1] for r in run] == [0.2, 0.2, 0.0]


def test_only_the_run_files_of_a_directory_are_fitted(tmp_path):
    for name in ("run-b.csv", "run-a.csv", "cut-a.csv", "README.md"):
        (tmp_path / name).write_text("")
    found = thermal_fit.run_paths([str(tmp_path)])
    assert [n for n, _ in found] == [f"{tmp_path.name}/run-a.csv", f"{tmp_path.name}/run-b.csv"]


def test_the_warm_starts_are_left_out_of_the_fit():
    names = [n for n, _ in thermal_fit.run_paths([str(DATA_B2)])]
    assert names == ["dryer-b2/run-10-2s.csv", "dryer-b2/run-20-120s.csv"]


def test_the_warm_starts_are_recognised_by_their_content_wherever_they_are(tmp_path):
    """A copy under another directory's name is still left out; the cold runs are still fitted."""
    copy = tmp_path / "elsewhere"
    copy.mkdir()
    for name in ("run-10-2s.csv", "run-35-to50.csv", "run-50-to50.csv"):
        (copy / name).write_bytes((DATA_B2 / name).read_bytes())
    names = [n for n, _ in thermal_fit.run_paths([str(copy)])]
    assert names == ["elsewhere/run-10-2s.csv"]


def test_an_excluded_run_with_other_line_endings_is_still_left_out(tmp_path):
    copy = tmp_path / "dryer-b2"
    copy.mkdir()
    data = (DATA_B2 / "run-35-to50.csv").read_bytes().replace(b"\n", b"\r\n")
    (copy / "run-35-to50.csv").write_bytes(data)
    assert thermal_fit.run_paths([str(copy)]) == []


def test_an_excluded_runs_name_with_other_content_stops_the_fit(tmp_path):
    """Never silently fitted: a file named as an excluded run whose content changed fails."""
    copy = tmp_path / "dryer-b2"
    copy.mkdir()
    (copy / "run-50-to50.csv").write_bytes((DATA_B2 / "run-50-to50.csv").read_bytes() + b"\n")
    try:
        thermal_fit.run_paths([str(copy)])
    except thermal_fit.ExcludedRunChangedError as e:
        assert "dryer-b2/run-50-to50.csv" in str(e)
    else:
        raise AssertionError("an altered excluded run was accepted")
    r = subprocess.run(
        [sys.executable, str(FIRMWARE / "tools" / "thermal_fit.py"), str(copy)],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 1
    assert "content differs" in r.stderr


def test_the_sources_are_the_paths_of_the_directories_relative_to_firmware(tmp_path):
    lines = thermal_fit.sources([str(DATA), str(DATA_B2) + "/"])
    assert lines == [" *   tests/data/heater-2026-09-13", " *   tests/data/dryer-b2"]
    assert thermal_fit.sources([str(tmp_path)]) == [f" *   {tmp_path.name}"]


def test_committed_headers_are_the_fit_of_the_committed_runs():
    r = subprocess.run(
        [
            sys.executable,
            str(FIRMWARE / "tools" / "thermal_fit.py"),
            str(DATA),
            str(DATA_B2),
            "--params",
            str(FIRMWARE / "tests" / "thermal_params.h"),
            "--runs",
            str(FIRMWARE / "tests" / "thermal_runs.h"),
            "--check",
        ],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stdout + r.stderr
