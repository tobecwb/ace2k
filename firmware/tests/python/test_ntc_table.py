"""tools/ntc_table.py: known points of the curve and the committed header being current."""

import subprocess
import sys
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FIRMWARE / "tools"))
import ntc_table  # noqa: E402


def test_known_points():
    assert ntc_table.millidegrees(1650) == 25000
    assert ntc_table.millidegrees(1000) == 45002
    assert ntc_table.millidegrees(100) == 130627
    assert ntc_table.millidegrees(3200) == -36823


def test_committed_header_is_current():
    r = subprocess.run(
        [
            sys.executable,
            str(FIRMWARE / "tools/ntc_table.py"),
            "--check",
            str(FIRMWARE / "src/ace2k/env/env_table.h"),
        ],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stderr
