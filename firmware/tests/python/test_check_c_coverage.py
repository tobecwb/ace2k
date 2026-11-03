"""check_c_coverage.py: fails below either floor of the llvm-cov TOTAL row."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "check_c_coverage.py"
TOTAL = (
    "TOTAL                            5935               196    96.70%         634"
    "                 5    99.21%        6553               232    96.46%        2992"
    "               252    91.58%\n"
)
REPORT = "Filename  Regions\n-----\nsrc/a.c  1 0 100.00%\n" + TOTAL


def run(report: str, lines: str, branches: str) -> subprocess.CompletedProcess[str]:
    """Run the script on a report with the given floors."""
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--lines", lines, "--branches", branches],
        input=report,
        capture_output=True,
        text=True,
        check=False,
    )


def test_at_or_above_both_floors_passes_and_prints_both_values() -> None:
    result = run(REPORT, "96.4", "91.5")
    assert result.returncode == 0
    assert "lines 96.46 % (floor 96.4)" in result.stdout
    assert "branches 91.58 % (floor 91.5)" in result.stdout


def test_lines_below_the_floor_fail() -> None:
    assert run(REPORT, "99", "91.5").returncode == 1


def test_branches_below_the_floor_fail() -> None:
    assert run(REPORT, "96.4", "95").returncode == 1


def test_a_report_without_a_total_row_fails() -> None:
    assert run("nothing\n", "1", "1").returncode != 0


def test_figures_equal_to_the_floors_pass() -> None:
    assert run(REPORT, "96.46", "91.58").returncode == 0


def test_a_total_row_without_a_branch_figure_fails_clearly() -> None:
    row = TOTAL.replace("91.58%", "-")
    result = run("x\n" + row, "1", "1")
    assert result.returncode != 0
    assert "no branch figure" in result.stderr
