"""mkimage.py: the trailer, the CRC-32 before it, the padding, the refusals."""

import struct
import subprocess
import sys
import zlib
from pathlib import Path

TOOL = Path(__file__).resolve().parents[2] / "tools" / "mkimage.py"
MAGIC = bytes.fromhex("61a5635a65a5325a")
APP_BASE = 0x08008000
CONFIG_PAGE = 0x08023800


def raw(sp=0x2000C000, entry=APP_BASE + 0x1001, size=100):
    """A fake linked binary: vector table (SP, entry) then filler.

    `size` includes the 8 header bytes.
    """
    filler = bytes(range(256)) * (size // 256 + 1)
    return struct.pack("<II", sp, entry) + filler[: size - 8]


def run(tmp_path, data, *args):
    src, out = tmp_path / "raw.bin", tmp_path / "out.bin"
    src.write_bytes(data)
    proc = subprocess.run(
        [sys.executable, str(TOOL), str(src), str(out), *args], capture_output=True, text=True
    )
    return proc, out


def test_appends_magic_and_pads_to_four(tmp_path):
    proc, out = run(tmp_path, raw(size=101))
    assert proc.returncode == 0, proc.stderr
    img = out.read_bytes()
    assert img[-8:] == MAGIC
    assert len(img) % 4 == 0
    assert img[:101] == raw(size=101)
    assert img[101:104] == b"\xff\xff\xff"


def test_keeps_a_four_aligned_image_unpadded(tmp_path):
    proc, out = run(tmp_path, raw(size=100))
    assert proc.returncode == 0
    assert len(out.read_bytes()) == 100 + 4 + 8  # raw, CRC-32, trailer


def test_image_carries_the_crc32_before_the_trailer(tmp_path):
    proc, out = run(tmp_path, raw(size=101))  # padded: the CRC covers the pad bytes too
    assert proc.returncode == 0, proc.stderr
    img = out.read_bytes()
    assert img[-8:] == MAGIC
    body, crc = img[:-12], struct.unpack("<I", img[-12:-8])[0]
    assert len(body) % 4 == 0
    assert body == raw(size=101) + b"\xff\xff\xff"
    assert crc == zlib.crc32(body) & 0xFFFFFFFF
    assert f"crc32=0x{crc:08X}" in proc.stdout


def test_refuses_sp_outside_sram(tmp_path):
    proc, out = run(tmp_path, raw(sp=0x08008000))
    assert proc.returncode == 1 and "SP" in proc.stderr and not out.exists()


def test_refuses_non_thumb_entry(tmp_path):
    proc, _ = run(tmp_path, raw(entry=APP_BASE + 0x1000))
    assert proc.returncode == 1 and "Thumb" in proc.stderr


def test_refuses_entry_outside_slot(tmp_path):
    proc, _ = run(tmp_path, raw(entry=0x08024000 + 1))
    assert proc.returncode == 1 and "slot" in proc.stderr


def test_refuses_image_over_max_bytes(tmp_path):
    proc, _ = run(tmp_path, raw(size=200), "--max-bytes", "150")
    assert proc.returncode == 1 and "exceeds the 150 B limit" in proc.stderr


def test_refuses_image_overlapping_config_page(tmp_path):
    size = CONFIG_PAGE - APP_BASE + 4
    proc, out = run(tmp_path, raw(size=size), "--max-bytes", str(2 * size))
    assert proc.returncode == 1 and not out.exists()
    assert "would overlap the config page at 0x08023800" in proc.stderr


def test_reports_size_and_sha(tmp_path):
    proc, _ = run(tmp_path, raw(size=100))
    assert "size=112" in proc.stdout and "sha256=" in proc.stdout
