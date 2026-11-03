"""tools/ntc_table.py: known points of the curve and the committed header being current."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

FIRMWARE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FIRMWARE / "tools"))
import ntc_table  # noqa: E402


def test_known_points() -> None:
    assert ntc_table.millidegrees(1650) == 25000
    assert ntc_table.millidegrees(1000) == 45002
    assert ntc_table.millidegrees(100) == 130627
    assert ntc_table.millidegrees(3200) == -36823


HEADER = FIRMWARE / "src/ace2k/env/env_table.h"


def test_committed_header_is_current(capsys: pytest.CaptureFixture[str]) -> None:
    assert ntc_table.main(["--check", str(HEADER)]) == 0
    assert "is current" in capsys.readouterr().out


def test_an_outdated_header_fails_the_check(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    stale = tmp_path / "env_table.h"
    stale.write_text(HEADER.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    assert ntc_table.main(["--check", str(stale)]) == 1
    assert "out of date" in capsys.readouterr().err


def test_writing_produces_the_committed_header(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "env_table.h"
    assert ntc_table.main([str(out)]) == 0
    assert f"wrote {out}" in capsys.readouterr().out
    assert out.read_text(encoding="utf-8") == HEADER.read_text(encoding="utf-8")


def test_the_arguments_are_one_path() -> None:
    with pytest.raises(SystemExit) as stop:
        ntc_table.main([])
    assert "ntc_table.py OUT" in str(stop.value)


def test_the_script_runs_as_a_program() -> None:
    subprocess.run(
        [sys.executable, str(FIRMWARE / "tools/ntc_table.py"), "--check", str(HEADER)],
        check=True,
        capture_output=True,
    )
