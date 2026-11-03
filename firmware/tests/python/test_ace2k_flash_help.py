"""ace2k_flash: its --help, byte for byte the copy stored in fixtures/ace2k_flash_help.txt, and
the one-file build of it, ace2k_flash.pyz.

The copy was taken from the single-file flasher with COLUMNS=80, before it became a package.
The command line must not change: its options and their order stay.  A help text or the exit
status list may say more, and then the copy is taken again.  Only the program's name in the
usage lines follows the name of the file that runs.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from flash_sim import FIRMWARE, fl

HELP = FIRMWARE / "tests" / "python" / "fixtures" / "ace2k_flash_help.txt"
OLD_NAME = "ace2k_flash.py"

spec = importlib.util.spec_from_file_location("build_pyz", FIRMWARE / "scripts" / "build_pyz.py")
assert spec is not None and spec.loader is not None
build_pyz = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_pyz)
# the package's own __main__ module, which `python3 -m ace2k_flash` and the .pyz run
entry = importlib.import_module("ace2k_flash.__main__")


def help_as(prog: str) -> str:
    """The stored help as it reads when the file that runs is named prog.

    argparse puts the name in the first usage line and lines up the next ones under it.
    """
    usage, rest = HELP.read_text().split("\n\n", 1)
    first, *more = usage.split("\n")
    pad = " " * (len(prog) - len(OLD_NAME))
    first = first.replace(f"usage: {OLD_NAME} ", f"usage: {prog} ", 1)
    return "\n".join([first, *(pad + line for line in more)]) + "\n\n" + rest


def run_help(path: Path) -> subprocess.CompletedProcess[str]:
    """`python3 -I <path> --help` at 80 columns, without colour.

    -I keeps the developer's environment and site packages out of the run.
    """
    env = {**os.environ, "COLUMNS": "80", "NO_COLOR": "1"}
    env.pop("FORCE_COLOR", None)
    env.pop("PYTHON_COLORS", None)
    return subprocess.run(
        [sys.executable, "-I", str(path), "--help"],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )


@pytest.fixture(name="pyz")
def fixture_pyz(tmp_path: Path) -> Path:
    """ace2k_flash.pyz built from the source tree, as `make flasher-pyz` builds it."""
    out = tmp_path / "ace2k_flash.pyz"
    build_pyz.build(FIRMWARE / "tools" / "ace2k_flash", out)
    return out


def test_the_help_is_the_stored_copy(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("COLUMNS", "80")
    monkeypatch.setattr("sys.argv", ["ace2k_flash.py"])
    with pytest.raises(SystemExit) as e:
        fl.main(["--help"])
    assert e.value.code == 0
    assert capsys.readouterr().out == HELP.read_text()


def test_the_package_entry_runs_the_command_line(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("COLUMNS", "80")
    monkeypatch.setattr("sys.argv", ["ace2k_flash.py", "--help"])
    with pytest.raises(SystemExit) as e:
        entry.run()
    assert e.value.code == 0
    assert capsys.readouterr().out == HELP.read_text()


def test_the_pyz_prints_the_stored_help(pyz: Path) -> None:
    result = run_help(pyz)
    assert result.returncode == 0, result.stderr
    assert result.stdout == help_as("ace2k_flash.pyz")


def test_the_pyz_under_the_old_name_prints_the_stored_help_byte_for_byte(
    pyz: Path, tmp_path: Path
) -> None:
    # Python runs a zip archive whatever its name: under the old file name the help is the copy
    old = tmp_path / "old" / OLD_NAME
    old.parent.mkdir()
    shutil.copy(pyz, old)
    result = run_help(old)
    assert result.returncode == 0, result.stderr
    assert result.stdout == HELP.read_text()


def test_the_parser_takes_its_name_from_the_file_that_runs(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    # Python 3.14's argparse names the interpreter too when the program is a zip ("python3.14
    # ace2k_flash.pyz"); the flasher passes the file's own name, as older versions print it
    seen: dict[str, object] = {}

    class Parser(fl.flasher._Parser):
        def __init__(self, **kwargs: object) -> None:
            seen.update(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(fl.flasher, "_Parser", Parser)
    monkeypatch.setattr("sys.argv", ["/somewhere/ace2k_flash.pyz"])
    with pytest.raises(SystemExit):
        fl.main(["--help"])
    assert seen.get("prog") == "ace2k_flash.pyz"
    assert capsys.readouterr().out.startswith("usage: ace2k_flash.pyz [-h]")


def test_a_usage_error_from_the_pyz_is_refused_not_unverified(pyz: Path) -> None:
    result = subprocess.run([sys.executable, str(pyz)], capture_output=True, text=True, check=False)
    assert result.returncode == fl.EXIT_UNCHANGED
    assert "error: the following arguments are required: image" in result.stderr


def test_two_builds_of_the_same_source_are_the_same_bytes(pyz: Path, tmp_path: Path) -> None:
    again = tmp_path / "again" / "ace2k_flash.pyz"
    build_pyz.build(FIRMWARE / "tools" / "ace2k_flash", again)
    assert again.read_bytes() == pyz.read_bytes()


def test_the_pyz_is_a_runnable_stored_archive(pyz: Path) -> None:
    data = pyz.read_bytes()
    assert data.startswith(b"#!/usr/bin/env python3\n")
    assert os.access(pyz, os.X_OK)
    # ace2k-u1's installer looks for this text to know the flasher takes the option: the
    # entries are stored, not compressed, so the text is in the file as written
    assert b'"--config-follows"' in data
