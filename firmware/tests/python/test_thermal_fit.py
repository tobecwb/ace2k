"""tools/thermal_fit.py: the fit recovers a model it did not see, and the committed headers are
the fit of the committed runs."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

FIRMWARE = Path(__file__).resolve().parents[2]
DATA = FIRMWARE / "tests" / "data" / "heater-2026-09-13"
DATA_B2 = FIRMWARE / "tests" / "data" / "dryer-b2"
sys.path.insert(0, str(FIRMWARE / "tools"))
import thermal_fit  # noqa: E402

if TYPE_CHECKING:
    from thermal_fit import Row

KNOWN = (0.9, 0.04, 14.0, 0.1)  # a, k, tau, x: a side the fit has never seen
STEP_S = 0.5


def synthetic_run() -> list[Row]:
    """150 s at 35 % then 60 s off, the chamber held at 25 degC; the left NTC from the model."""
    rows = []
    for i in range(int(210 / STEP_S)):
        t = i * STEP_S
        u = 0.35 if t < 150 else 0.0
        rows.append((t, u, 25.0, 25.0, 25.0))
    left = thermal_fit.simulate_side(KNOWN, rows, 0)
    return [(t, u, n, 25.0, c) for (t, u, _, _, c), n in zip(rows, left)]


def test_the_fit_recovers_a_known_side() -> None:
    got = thermal_fit.fit_side([synthetic_run()], 0)
    for want, have in zip(KNOWN, got):
        assert abs(have - want) <= 0.05 * want, (KNOWN, got)


def test_a_side_at_rest_stays_at_its_chamber() -> None:
    rows = [(i * STEP_S, 0.0, 30.0, 30.0, 30.0) for i in range(100)]
    assert thermal_fit.simulate_side(KNOWN, rows, 0) == [30.0] * 100


def test_the_minimiser_shrinks_toward_its_best_point_when_nothing_else_helps() -> None:
    # Best at 0, worst at 1. Reflecting (-1) and contracting (0.5) are both no better than the
    # worst, so the simplex shrinks toward 0: the worst point moves to 0.5 and is valued again.
    # The second step does the same from [0, 0.5]: -0.5, then 0.25 twice.
    table = {0.0: 1.0, 1.0: 2.0, -1.0: 5.0, 0.5: 3.0}
    calls: list[float] = []

    def f(x: list[float]) -> float:
        calls.append(x[0])
        return table.get(x[0], 10.0)

    got = thermal_fit.nelder_mead(f, [0.0], step=1.0, iters=2)
    assert calls == [0.0, 1.0, -1.0, 0.5, 0.5, -0.5, 0.25, 0.25]
    assert got == ([0.0], 1.0)


BENCH_HEADER = "t_s,state,duty,fired,reason,zc_hz,ntc1_c,ntc2_c,env_c,env_rh,cutout,fan_l,fan_r\n"


def bench_row(
    t: float,
    state: str,
    fired: int,
    fans: int,
    right: float = 30.0,
    left: float = 31.0,
    zc: str = "60.0",
) -> str:
    duty = 20 if state == "leased" else 0
    return f"{t},{state},{duty},{fired},ok,{zc},{right},{left},28.0,60.0,0,{fans},{fans}\n"


def test_a_bench_run_takes_its_firing_from_the_fired_count(tmp_path: Path) -> None:
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


def test_a_bench_run_keeps_every_half_cycle_it_fired(tmp_path: Path) -> None:
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


def test_a_bench_run_ends_at_its_first_stretch_of_fans(tmp_path: Path) -> None:
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


def test_two_quiet_rows_without_the_fans_end_a_bench_run(tmp_path: Path) -> None:
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


def test_a_new_lease_after_a_quiet_row_ends_the_run_whatever_its_fans_read(tmp_path: Path) -> None:
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


def test_a_quiet_row_is_judged_against_the_row_before_it(tmp_path: Path) -> None:
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


def test_a_glitch_in_the_cool_down_tail_is_skipped(tmp_path: Path) -> None:
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


def test_a_latched_heater_ends_a_bench_run_with_its_firing(tmp_path: Path) -> None:
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


def test_a_single_fan_glitch_does_not_end_a_bench_run(tmp_path: Path) -> None:
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


def test_rows_that_cannot_be_timed_are_skipped(tmp_path: Path) -> None:
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


def test_only_the_run_files_of_a_directory_are_fitted(tmp_path: Path) -> None:
    for name in ("run-b.csv", "run-a.csv", "cut-a.csv", "README.md"):
        (tmp_path / name).write_text("")
    found = thermal_fit.run_paths([str(tmp_path)])
    assert [n for n, _ in found] == [f"{tmp_path.name}/run-a.csv", f"{tmp_path.name}/run-b.csv"]


def test_the_warm_starts_are_left_out_of_the_fit() -> None:
    names = [n for n, _ in thermal_fit.run_paths([str(DATA_B2)])]
    assert names == ["dryer-b2/run-10-2s.csv", "dryer-b2/run-20-120s.csv"]


def test_the_warm_starts_are_recognised_by_their_content_wherever_they_are(tmp_path: Path) -> None:
    """A copy under another directory's name is still left out; the cold runs are still fitted."""
    copy = tmp_path / "elsewhere"
    copy.mkdir()
    for name in ("run-10-2s.csv", "run-35-to50.csv", "run-50-to50.csv"):
        (copy / name).write_bytes((DATA_B2 / name).read_bytes())
    names = [n for n, _ in thermal_fit.run_paths([str(copy)])]
    assert names == ["elsewhere/run-10-2s.csv"]


def test_an_excluded_run_with_other_line_endings_is_still_left_out(tmp_path: Path) -> None:
    copy = tmp_path / "dryer-b2"
    copy.mkdir()
    data = (DATA_B2 / "run-35-to50.csv").read_bytes().replace(b"\n", b"\r\n")
    (copy / "run-35-to50.csv").write_bytes(data)
    assert thermal_fit.run_paths([str(copy)]) == []


def test_an_excluded_runs_name_with_other_content_stops_the_fit(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
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
    assert thermal_fit.main([str(copy)]) == 1
    assert "content differs" in capsys.readouterr().err


def test_the_sources_are_the_paths_of_the_directories_relative_to_firmware(tmp_path: Path) -> None:
    lines = thermal_fit.sources([str(DATA), str(DATA_B2) + "/"])
    assert lines == [" *   tests/data/heater-2026-09-13", " *   tests/data/dryer-b2"]
    assert thermal_fit.sources([str(tmp_path)]) == [f" *   {tmp_path.name}"]


def test_committed_headers_are_the_fit_of_the_committed_runs(
    capsys: pytest.CaptureFixture[str],
) -> None:
    code = thermal_fit.main(
        [
            str(DATA),
            str(DATA_B2),
            "--params",
            str(FIRMWARE / "tests" / "thermal_params.h"),
            "--runs",
            str(FIRMWARE / "tests" / "thermal_runs.h"),
            "--check",
        ]
    )
    seen = capsys.readouterr()
    assert code == 0, seen.out + seen.err


def test_the_script_runs_as_a_program() -> None:
    subprocess.run(
        [sys.executable, str(FIRMWARE / "tools" / "thermal_fit.py"), "--help"],
        check=True,
        capture_output=True,
    )
