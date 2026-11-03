"""check_line_length.py: fails on a C line over the limit, counted in characters."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "check_line_length.py"


def run(*args: str) -> subprocess.CompletedProcess[str]:
    """Run the script with the given arguments."""
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        check=False,
    )


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    """A source tree with one file at the limit and a build directory with a long line."""
    (tmp_path / "src").mkdir()
    (tmp_path / "src" / "ok.c").write_text("/* " + "a" * 7 + " */\n", encoding="utf-8")
    (tmp_path / "build").mkdir()
    (tmp_path / "build" / "out.c").write_text("x" * 50 + "\n", encoding="utf-8")
    return tmp_path


def test_a_line_at_the_limit_passes(tree: Path) -> None:
    result = run("--max", "13", str(tree))
    assert result.returncode == 0
    assert "line-length: OK (1 files" in result.stdout


def test_a_line_over_the_limit_fails_and_is_named(tree: Path) -> None:
    (tree / "src" / "long.h").write_text("int a;\n" + "b" * 14 + "\n", encoding="utf-8")
    result = run("--max", "13", str(tree))
    assert result.returncode == 1
    assert f"{tree / 'src' / 'long.h'}:2: 14 characters (limit 13)" in result.stdout


def test_the_length_is_counted_in_characters_not_bytes(tree: Path) -> None:
    line = "/* 25 °C µm ° */"
    assert (len(line), len(line.encode("utf-8"))) == (16, 19)
    (tree / "src" / "units.c").write_text(line + "\n", encoding="utf-8")
    assert run("--max", "16", str(tree)).returncode == 0
    assert run("--max", "15", str(tree)).returncode == 1


def test_an_excluded_file_is_not_checked(tree: Path) -> None:
    (tree / "src" / "table.h").write_text("c" * 40 + "\n", encoding="utf-8")
    assert run("--max", "13", str(tree)).returncode == 1
    assert run("--max", "13", "--exclude", "table.h", str(tree)).returncode == 0


def test_other_files_are_not_checked(tree: Path) -> None:
    (tree / "src" / "notes.txt").write_text("d" * 40 + "\n", encoding="utf-8")
    assert run("--max", "13", str(tree)).returncode == 0


def test_a_root_under_a_directory_named_build_still_checks_its_files(tmp_path: Path) -> None:
    # Only a build directory below the scanned root is output; the checkout may sit in one.
    root = tmp_path / "build" / "checkout"
    (root / "src").mkdir(parents=True)
    (root / "src" / "long.c").write_text("d" * 14 + "\n", encoding="utf-8")
    (root / "build").mkdir()
    (root / "build" / "out.c").write_text("x" * 50 + "\n", encoding="utf-8")
    result = run("--max", "13", str(root))
    assert result.returncode == 1
    assert f"{root / 'src' / 'long.c'}:1: 14 characters (limit 13)" in result.stdout
    assert "out.c" not in result.stdout
