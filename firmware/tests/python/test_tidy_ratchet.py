"""The ratchet checker of `make tidy-bind`, over sample clang-tidy outputs."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "tidy_ratchet.py"
spec = importlib.util.spec_from_file_location("tidy_ratchet", SCRIPT)
ratchet = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ratchet)

FW = "/w/firmware"
LISTED = "# comment\nsrc/ace2k_board/fg.c: bugprone-signed-bitwise, readability-magic-numbers\n"


def diag(path: str, check: str, kind: str = "error", firmware: str = FW) -> str:
    return f"{firmware}/{path}:3:4: {kind}: text [{check},-warnings-as-errors]\n"


def run(
    tmp_path: Path, output: str, listed: str = LISTED, status: str = "1", firmware: str = FW
) -> int:
    r = tmp_path / "ratchet.txt"
    o = tmp_path / "out.txt"
    r.write_text(listed)
    o.write_text(output)
    return ratchet.main(["x", str(r), str(o), status, firmware])


CLEAN = diag("src/ace2k_board/fg.c", "bugprone-signed-bitwise") + diag(
    "src/ace2k_board/fg.c", "readability-magic-numbers"
)


def test_clean_pass(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert run(tmp_path, CLEAN) == 0
    assert "OK (1 ratchet entries)" in capsys.readouterr().out


def test_new_check_fails(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = CLEAN + diag("src/ace2k_board/fg.c", "readability-braces-around-statements")
    assert run(tmp_path, out) == 1
    assert (
        "fg.c: readability-braces-around-statements is not in the ratchet"
        in capsys.readouterr().out
    )


def test_unlisted_file_fails(tmp_path: Path) -> None:
    assert run(tmp_path, CLEAN + diag("src/ace2k_board/gate.c", "bugprone-signed-bitwise")) == 1


def test_fatal_error_fails(tmp_path: Path) -> None:
    out = CLEAN + f"{FW}/src/a.c:1:10: fatal error: 'x.h' file not found [clang-diagnostic-error]\n"
    assert run(tmp_path, out) == 1


def test_error_outside_src_fails(tmp_path: Path) -> None:
    out = CLEAN + f"{FW}/klipper/lib/y.h:5:1: error: bad [clang-diagnostic-error]\n"
    assert run(tmp_path, out) == 1


def test_empty_output_fails(tmp_path: Path) -> None:
    assert run(tmp_path, "", status="0") == 1  # every ratchet entry is stale


def test_crashed_tidy_fails(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert run(tmp_path, "clang-tidy: command not found\n", listed="", status="127") == 1
    assert "did not run" in capsys.readouterr().out


def test_empty_output_with_empty_ratchet_passes(tmp_path: Path) -> None:
    assert run(tmp_path, "", listed="", status="0") == 0


def test_stale_entry_fails(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = diag("src/ace2k_board/fg.c", "bugprone-signed-bitwise")
    assert run(tmp_path, out) == 1
    assert (
        "readability-magic-numbers is in the ratchet but no longer fires" in capsys.readouterr().out
    )


def test_a_checkout_under_a_src_directory_keys_its_own_src(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    # The key is the path below the firmware's own src, not below the first "src" of the path.
    firmware = "/home/me/src/ace2k/firmware"
    out = diag("src/ace2k_board/fg.c", "bugprone-signed-bitwise", firmware=firmware) + diag(
        "src/ace2k_board/fg.c", "readability-magic-numbers", firmware=firmware
    )
    assert run(tmp_path, out, firmware=firmware) == 0
    assert "OK (1 ratchet entries)" in capsys.readouterr().out


def test_a_finding_in_another_src_still_fails_under_its_full_path(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    # Klipper's own src is not the firmware's: its file never matches an entry, and fails.
    out = CLEAN + diag("klipper/src/ace2k_board/fg.c", "bugprone-signed-bitwise")
    assert run(tmp_path, out) == 1
    assert (
        f"{FW}/klipper/src/ace2k_board/fg.c: bugprone-signed-bitwise is not in the ratchet"
        in capsys.readouterr().out
    )


def test_the_firmware_defaults_to_the_directory_of_the_scripts(tmp_path: Path) -> None:
    firmware = str(SCRIPT.parents[1])
    out = diag("src/ace2k_board/fg.c", "bugprone-signed-bitwise", firmware=firmware) + diag(
        "src/ace2k_board/fg.c", "readability-magic-numbers", firmware=firmware
    )
    r = tmp_path / "ratchet.txt"
    o = tmp_path / "out.txt"
    r.write_text(LISTED)
    o.write_text(out)
    assert ratchet.main(["x", str(r), str(o), "1"]) == 0
