"""ace2k_flash: the frame, the allowlist, protobuf, image recognition."""

from __future__ import annotations

import struct
import zlib
from pathlib import Path
from typing import NoReturn

import pytest
from flash_sim import ace2k_image, factory_image, fl, reply


def test_crc16_check_value() -> None:
    # CRC-16/MCRF4XX's catalogue check value
    assert fl.crc16(b"123456789") == 0x6F91


def test_frame_layout() -> None:
    f = fl.build_frame(fl.CMD_GET_INFO, b"", 0x0102)
    assert f[:2] == b"\xff\xaa"
    assert f[2:7] == bytes([0x00, 0x02, 0x01, 7, 0])
    assert struct.unpack_from("<H", f, 7)[0] == fl.crc16(f[2:7])
    assert f[-1] == 0xFE and len(f) == 10


@pytest.mark.parametrize("cmd", [1, 5, 15, 17, 18, 64, 250])  # 5: a query no step sends
def test_allowlist_refuses_before_anything_is_written(cmd: int) -> None:
    with pytest.raises(fl.RefusedError):
        fl.build_frame(cmd, b"", 1)

    class NoWrite:
        def clear(self) -> None:
            pass

        def write(self, data: bytes) -> NoReturn:
            raise AssertionError("written")

    with pytest.raises(fl.RefusedError):
        fl.Unit(NoWrite(), fl.Clock()).ask(cmd)


def test_payload_cap() -> None:
    fl.build_frame(fl.CMD_IAP_FIRMWARE, b"\x00" * 100, 1)
    with pytest.raises(fl.RefusedError):
        fl.build_frame(fl.CMD_IAP_FIRMWARE, b"\x00" * 101, 1)


def test_parse_skips_a_false_preamble_and_keeps_a_tail() -> None:
    real = reply(0x80, 5, 7, b"\x0a\x01A")
    noise = b"\x12\xff\xaa\x00\x00\x00\x07\xc8junk"  # FF AA announcing 200 bytes that never come
    buf = bytearray(noise + real + real[:4])
    frames = fl.parse_frames(buf)
    assert [(f.addr, f.seq, f.cmd, f.payload) for f in frames] == [(0x80, 5, 7, b"\x0a\x01A")]
    assert bytes(buf) == real[:4]


def test_parse_rejects_a_bad_crc() -> None:
    bad = bytearray(reply(0x80, 1, 7, b""))
    bad[-2] ^= 1
    assert fl.parse_frames(bad) == []


def test_protobuf_round_trip() -> None:
    data = fl.pb_uint(1, 300) + fl.pb_bytes(3, b"abc") + fl.pb_uint(1, 0)
    assert fl.pb_fields(data) == {1: [300, 0], 3: [b"abc"]}
    with pytest.raises(ValueError):
        fl.pb_fields(fl.pb_bytes(3, b"abc")[:-1])


def test_ace2k_image_carries_its_version(tmp_path: Path) -> None:
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image("0.12.0-3-gabcdef1"))
    img = fl.load_image(p, None)
    assert (img.kind, img.version, img.announce) == ("ace2k", "0.12.0-3-gabcdef1", "0.12.0-3-ga")
    assert fl.load_image(p, "0.12.0-3-gabcdef1").version == "0.12.0-3-gabcdef1"
    with pytest.raises(fl.RefusedError, match="says"):
        fl.load_image(p, "0.11.0")


def test_old_ace2k_image_needs_a_version(tmp_path: Path) -> None:
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image(with_constant=False))
    with pytest.raises(fl.RefusedError, match="--version"):
        fl.load_image(p, None)
    assert fl.load_image(p, "0.11.0").kind == "ace2k"


def test_factory_image_needs_a_version(tmp_path: Path) -> None:
    p = tmp_path / "f.bin"
    p.write_bytes(factory_image())
    with pytest.raises(fl.RefusedError, match="factory"):
        fl.load_image(p, None)
    img = fl.load_image(p, "1.1.31")
    assert (img.kind, img.version) == ("factory", "1.1.31")


def test_refused_images(tmp_path: Path) -> None:
    p = tmp_path / "x.bin"
    good = ace2k_image()
    for data in (good[:-1], good[:-8] + b"\x00" * 8, b"", b"\xff" * 0x1C001 + fl.TRAILER):
        p.write_bytes(data)
        with pytest.raises(fl.RefusedError):
            fl.load_image(p, "1.0.0")
    body = (
        struct.pack("<II", fl.SRAM_END, fl.APP_BASE + 0x1001) + b"\x01" * 56
    )  # ace2k's checksum, no dictionary
    p.write_bytes(body + struct.pack("<I", zlib.crc32(body)) + fl.TRAILER)
    with pytest.raises(fl.RefusedError, match="dictionary"):
        fl.load_image(p, None)
    with pytest.raises(fl.RefusedError):
        fl.load_image(tmp_path / "missing.bin", None)


def damaged(image: bytes, at: int = 100) -> bytes:
    """image with one byte changed, away from its vector table and its dictionary."""
    return image[:at] + bytes([image[at] ^ 0xFF]) + image[at + 1 :]


def first_stream(image: bytes) -> int:
    """Where the compressed dictionary of an ace2k image from ace2k_image starts."""
    return image.rindex(b"\x78\x9c")


def test_the_realistic_dictionary_is_like_a_real_one() -> None:
    # as in every real image, the first 64 compressed bytes give no output at all: a check that
    # reads only them finds no dictionary
    img = ace2k_image("0.12.0", realistic=True)
    at = first_stream(img)
    assert zlib.decompressobj().decompress(img[at : at + 64]) == b""
    assert len(img) - 12 - at > 400  # room for the damage 300 bytes into it


IMAGES = [
    ace2k_image("0.12.0"),
    ace2k_image(with_constant=False),
    ace2k_image("0.12.0", realistic=True),
    ace2k_image(with_constant=False, realistic=True),
]


@pytest.mark.parametrize("img", IMAGES, ids=["small", "small-old", "real", "real-old"])
def test_a_damaged_ace2k_image_is_refused_not_taken_as_factory(tmp_path: Path, img: bytes) -> None:
    # --version is what an older ace2k image needs, and what a factory image needs too.  The
    # damage is the same one the manual test on the unit uses: byte 100
    p = tmp_path / "a.bin"
    p.write_bytes(damaged(img))
    with pytest.raises(fl.RefusedError) as refused:
        fl.load_image(p, "0.12.0")
    assert str(refused.value) == (
        f"{p}: a damaged ace2k image (its checksum does not match its contents); "
        "download or build it again"
    )


# 300 bytes in: past a real dictionary's first ~80 compressed bytes (the zlib header and the
# Huffman table), where a flip leaves no output to recognise
@pytest.mark.parametrize(("realistic", "into"), [(False, 20), (True, 300)])
def test_an_ace2k_image_damaged_inside_its_dictionary_is_refused(
    tmp_path: Path, realistic: bool, into: int
) -> None:
    # a flipped byte inside the compressed dictionary: the dictionary no longer decompresses
    # whole, but its start still shows that this is a damaged ace2k image, not a factory one
    good = ace2k_image("0.12.0", realistic=realistic)
    at = first_stream(good) + into
    p = tmp_path / "a.bin"
    p.write_bytes(good[:at] + bytes([good[at] ^ 0xFF]) + good[at + 1 :])
    with pytest.raises(fl.RefusedError) as refused:
        fl.load_image(p, "0.12.0")
    assert str(refused.value) == (
        f"{p}: a damaged ace2k image (its checksum does not match its contents); "
        "download or build it again"
    )


def test_an_ace2k_image_that_reaches_the_config_page_is_refused(tmp_path: Path) -> None:
    # the config page is the slot's last 2 KB: an ace2k image ends below it
    most = 0x08023800 - fl.APP_BASE
    small = ace2k_image(size=100 * 1024)
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image(size=100 * 1024, extra=b"\0" * (most - len(small))))
    assert len(p.read_bytes()) == most and fl.load_image(p, None).kind == "ace2k"
    p.write_bytes(ace2k_image(size=100 * 1024, extra=b"\0" * (most - len(small) + 4)))
    with pytest.raises(fl.RefusedError) as refused:
        fl.load_image(p, None)
    assert str(refused.value) == (
        f"{p}: {most + 4} bytes; an ace2k image must end before the config page at the end "
        f"of the slot, so it takes at most {most} bytes; build it with `make build`, which "
        "checks the size"
    )


@pytest.mark.parametrize(
    "sp, entry",
    [
        (fl.SRAM_BASE - 4, fl.APP_BASE + 0x1001),  # stack pointer outside SRAM
        (fl.SRAM_END + 1, fl.APP_BASE + 0x1001),  # ... just above it
        (fl.SRAM_END, fl.APP_BASE + 0x1000),  # even reset vector: not Thumb
        (fl.SRAM_END, fl.APP_END + 1),  # entry past the application slot
    ],
)
def test_vector_table_must_point_into_the_unit(tmp_path: Path, sp: int, entry: int) -> None:
    p = tmp_path / "v.bin"
    p.write_bytes(struct.pack("<II", sp, entry) + factory_image()[8:])
    with pytest.raises(fl.RefusedError, match="vector table"):
        fl.load_image(p, "1.1.31")


def test_a_valid_vector_table_passes(tmp_path: Path) -> None:
    p = tmp_path / "v.bin"
    p.write_bytes(struct.pack("<II", fl.SRAM_END, fl.APP_BASE + 0x1001) + factory_image()[8:])
    assert fl.load_image(p, "1.1.31").kind == "factory"


def test_an_empty_embedded_version_counts_as_none(tmp_path: Path) -> None:
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image(""))
    with pytest.raises(fl.RefusedError) as refused:
        fl.load_image(p, None)
    assert str(refused.value) == (
        f"{p}: this ace2k image carries no version (an image from before v0.12 does not); "
        "pass --version"
    )
    img = fl.load_image(p, "0.12.0")
    assert (img.kind, img.version, img.embedded) == ("ace2k", "0.12.0", False)
