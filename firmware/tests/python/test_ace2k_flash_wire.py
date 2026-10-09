"""ace2k_flash: the frame, the allowlist, protobuf, image recognition."""

import struct
import zlib

import pytest
from flash_sim import ace2k_image, factory_image, fl, reply


def test_crc16_check_value():
    # CRC-16/MCRF4XX's catalogue check value
    assert fl.crc16(b"123456789") == 0x6F91


def test_frame_layout():
    f = fl.build_frame(fl.CMD_GET_INFO, b"", 0x0102)
    assert f[:2] == b"\xff\xaa"
    assert f[2:7] == bytes([0x00, 0x02, 0x01, 7, 0])
    assert struct.unpack_from("<H", f, 7)[0] == fl.crc16(f[2:7])
    assert f[-1] == 0xFE and len(f) == 10


@pytest.mark.parametrize("cmd", [1, 15, 17, 18, 64, 250])
def test_allowlist_refuses_before_anything_is_written(cmd):
    with pytest.raises(fl.RefusedError):
        fl.build_frame(cmd, b"", 1)

    class NoWrite:
        def clear(self):
            pass

        def write(self, data):
            raise AssertionError("written")

    with pytest.raises(fl.RefusedError):
        fl.Unit(NoWrite(), fl.Clock()).ask(cmd)


def test_payload_cap():
    fl.build_frame(fl.CMD_IAP_FIRMWARE, b"\x00" * 100, 1)
    with pytest.raises(fl.RefusedError):
        fl.build_frame(fl.CMD_IAP_FIRMWARE, b"\x00" * 101, 1)


def test_parse_skips_a_false_preamble_and_keeps_a_tail():
    real = reply(0x80, 5, 7, b"\x0a\x01A")
    noise = b"\x12\xff\xaa\x00\x00\x00\x07\xc8junk"  # FF AA announcing 200 bytes that never come
    buf = bytearray(noise + real + real[:4])
    frames = fl.parse_frames(buf)
    assert [(f.addr, f.seq, f.cmd, f.payload) for f in frames] == [(0x80, 5, 7, b"\x0a\x01A")]
    assert bytes(buf) == real[:4]


def test_parse_rejects_a_bad_crc():
    bad = bytearray(reply(0x80, 1, 7, b""))
    bad[-2] ^= 1
    assert fl.parse_frames(bad) == []


def test_protobuf_round_trip():
    data = fl.pb_uint(1, 300) + fl.pb_bytes(3, b"abc") + fl.pb_uint(1, 0)
    assert fl.pb_fields(data) == {1: [300, 0], 3: [b"abc"]}
    with pytest.raises(ValueError):
        fl.pb_fields(fl.pb_bytes(3, b"abc")[:-1])


def test_ace2k_image_carries_its_version(tmp_path):
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image("0.12.0-3-gabcdef1"))
    img = fl.load_image(p, None)
    assert (img.kind, img.version, img.announce) == ("ace2k", "0.12.0-3-gabcdef1", "0.12.0-3-ga")
    assert fl.load_image(p, "0.12.0-3-gabcdef1").version == "0.12.0-3-gabcdef1"
    with pytest.raises(fl.RefusedError, match="says"):
        fl.load_image(p, "0.11.0")


def test_old_ace2k_image_needs_a_version(tmp_path):
    p = tmp_path / "a.bin"
    p.write_bytes(ace2k_image(with_constant=False))
    with pytest.raises(fl.RefusedError, match="--version"):
        fl.load_image(p, None)
    assert fl.load_image(p, "0.11.0").kind == "ace2k"


def test_factory_image_needs_a_version(tmp_path):
    p = tmp_path / "f.bin"
    p.write_bytes(factory_image())
    with pytest.raises(fl.RefusedError, match="factory"):
        fl.load_image(p, None)
    img = fl.load_image(p, "1.1.31")
    assert (img.kind, img.version) == ("factory", "1.1.31")


def test_refused_images(tmp_path):
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


@pytest.mark.parametrize(
    "sp, entry",
    [
        (fl.SRAM_BASE - 4, fl.APP_BASE + 0x1001),  # stack pointer outside SRAM
        (fl.SRAM_END + 1, fl.APP_BASE + 0x1001),  # ... just above it
        (fl.SRAM_END, fl.APP_BASE + 0x1000),  # even reset vector: not Thumb
        (fl.SRAM_END, fl.APP_END + 1),  # entry past the application slot
    ],
)
def test_vector_table_must_point_into_the_unit(tmp_path, sp, entry):
    p = tmp_path / "v.bin"
    p.write_bytes(struct.pack("<II", sp, entry) + factory_image()[8:])
    with pytest.raises(fl.RefusedError, match="vector table"):
        fl.load_image(p, "1.1.31")


def test_a_valid_vector_table_passes(tmp_path):
    p = tmp_path / "v.bin"
    p.write_bytes(struct.pack("<II", fl.SRAM_END, fl.APP_BASE + 0x1001) + factory_image()[8:])
    assert fl.load_image(p, "1.1.31").kind == "factory"
