"""mkimage.py: the trailer, the CRC-32 before it, the padding, the refusals."""

from __future__ import annotations

import struct
import subprocess
import sys
import zlib
from pathlib import Path
from typing import NamedTuple

import pytest

FIRMWARE = Path(__file__).resolve().parents[2]
TOOL = FIRMWARE / "tools" / "mkimage.py"
sys.path.insert(0, str(FIRMWARE / "tools"))
import mkimage  # noqa: E402

MAGIC = bytes.fromhex("61a5635a65a5325a")
APP_BASE = 0x08008000
CONFIG_PAGE = 0x08023800


def raw(sp: int = 0x2000C000, entry: int = APP_BASE + 0x1001, size: int = 100) -> bytes:
    """A fake linked binary: vector table (SP, entry) then filler.

    `size` includes the 8 header bytes.
    """
    filler = bytes(range(256)) * (size // 256 + 1)
    return struct.pack("<II", sp, entry) + filler[: size - 8]


class Result(NamedTuple):
    returncode: int
    stdout: str
    stderr: str


def run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], data: bytes, *args: str
) -> tuple[Result, Path]:
    """mkimage.main in process; the result carries its return code and both streams."""
    src, out = tmp_path / "raw.bin", tmp_path / "out.bin"
    src.write_bytes(data)
    code = mkimage.main([str(src), str(out), *args])
    seen = capsys.readouterr()
    return Result(code, seen.out, seen.err), out


def test_the_script_runs_as_a_program() -> None:
    subprocess.run([sys.executable, str(TOOL), "--help"], check=True, capture_output=True)


def test_appends_magic_and_pads_to_four(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, out = run(tmp_path, capsys, raw(size=101))
    assert proc.returncode == 0, proc.stderr
    img = out.read_bytes()
    assert img[-8:] == MAGIC
    assert len(img) % 4 == 0
    assert img[:101] == raw(size=101)
    assert img[101:104] == b"\xff\xff\xff"


def test_keeps_a_four_aligned_image_unpadded(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    proc, out = run(tmp_path, capsys, raw(size=100))
    assert proc.returncode == 0
    assert len(out.read_bytes()) == 100 + 4 + 8  # raw, CRC-32, trailer


def test_image_carries_the_crc32_before_the_trailer(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    proc, out = run(tmp_path, capsys, raw(size=101))  # padded: the CRC covers the pad bytes too
    assert proc.returncode == 0, proc.stderr
    img = out.read_bytes()
    assert img[-8:] == MAGIC
    body, crc = img[:-12], struct.unpack("<I", img[-12:-8])[0]
    assert len(body) % 4 == 0
    assert body == raw(size=101) + b"\xff\xff\xff"
    assert crc == zlib.crc32(body) & 0xFFFFFFFF
    assert f"crc32=0x{crc:08X}" in proc.stdout


def test_refuses_sp_outside_sram(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, out = run(tmp_path, capsys, raw(sp=0x08008000))
    assert proc.returncode == 1 and "SP" in proc.stderr and not out.exists()


def test_refuses_non_thumb_entry(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, _ = run(tmp_path, capsys, raw(entry=APP_BASE + 0x1000))
    assert proc.returncode == 1 and "Thumb" in proc.stderr


def test_refuses_entry_outside_slot(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, _ = run(tmp_path, capsys, raw(entry=0x08024000 + 1))
    assert proc.returncode == 1 and "slot" in proc.stderr


def test_refuses_image_over_max_bytes(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, _ = run(tmp_path, capsys, raw(size=200), "--max-bytes", "150")
    assert proc.returncode == 1 and "exceeds the 150 B limit" in proc.stderr


def test_refuses_image_overlapping_config_page(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    size = CONFIG_PAGE - APP_BASE + 4
    proc, out = run(tmp_path, capsys, raw(size=size), "--max-bytes", str(2 * size))
    assert proc.returncode == 1 and not out.exists()
    assert "would overlap the config page at 0x08023800" in proc.stderr


def test_reports_size_and_sha(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    proc, _ = run(tmp_path, capsys, raw(size=100))
    assert "size=112" in proc.stdout and "sha256=" in proc.stdout


def test_an_unreadable_input_is_a_refusal(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    code = mkimage.main([str(tmp_path / "missing.bin"), str(tmp_path / "out.bin")])
    seen = capsys.readouterr()
    assert code == 1
    assert seen.err.startswith("mkimage: REFUSED — ") and "missing.bin" in seen.err
    assert not (tmp_path / "out.bin").exists()


def test_an_unwritable_output_is_a_refusal(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = tmp_path / "raw.bin"
    src.write_bytes(raw())
    code = mkimage.main([str(src), str(tmp_path / "no-such-dir" / "out.bin")])
    seen = capsys.readouterr()
    assert code == 1
    assert seen.err.startswith("mkimage: REFUSED — ") and "no-such-dir" in seen.err


def test_the_help_names_the_default_budget(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit):
        mkimage.main(["--help"])
    assert f"default {mkimage.DEFAULT_MAX_BYTES}" in capsys.readouterr().out
