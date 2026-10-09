"""Snapmaker — MIFARE Classic 1K; key A of sector i is HMAC-SHA256(PRK, "key_a_<i>" + 0x01)[:6]
with PRK = HMAC-SHA256(salt, the first four UID bytes) (Snapmaker's u1-klipper reader); the
record in sectors 0–2, integers little-endian (OpenRFID's layout).  The record is RSA-signed;
the signature spans the sector trailers as the tag returns them, which this release never reads,
so it is reported "unchecked".  A Snapmaker spool's tag sits by the hub, out of the bay antenna's
reach (docs/hardware.md "RFID readers"): it reads with the tag held at the antenna."""

import struct

from .base import SpoolRecord, TagFormat, cstr
from .crypto import hmac_sha256

KEY_BYTES = 6
MAIN = {1: "PLA", 2: "PETG", 3: "ABS", 4: "TPU", 5: "PVA"}
SUB = {
    1: "Basic",
    2: "Matte",
    3: "SnapSpeed",
    4: "Silk",
    5: "Support",
    6: "HF",
    7: "95A",
    8: "95A HF",
}
IMAGE_BLOCKS = [b for b in range(11) if b % 4 != 3]


class Snapmaker(TagFormat):
    name = "snapmaker"
    chip = "mifare"
    needs = ("salt_a",)
    sectors = {0: 0b111, 1: 0b111, 2: 0b111}

    def key_for(self, uid, sector):
        prk = hmac_sha256(self.params["salt_a"].encode(), uid[:4])
        return hmac_sha256(prk, f"key_a_{sector}".encode() + b"\x01")[:KEY_BYTES]

    def decode(self, chip, got):
        if not self._all_read(got):
            return None
        img = bytearray(176)
        for blk in IMAGE_BLOCKS:
            img[blk * 16 : blk * 16 + 16] = got.blocks[blk]

        def u16(off):
            return struct.unpack_from("<H", img, off)[0]

        colors = img[72]
        return SpoolRecord(
            format=self.name,
            uid=chip.uid.hex().upper(),
            brand=cstr(img, 16, 16) or "Snapmaker",
            material=MAIN.get(u16(66)),
            name=SUB.get(u16(68)),
            color_rgba=(img[80:83].hex().upper() + "FF") if colors else None,
            diameter_mm=u16(128) / 100.0,
            weight_g=u16(130),
            length_m=u16(132),
            dry_temp_c=u16(144),
            dry_hours=u16(146),
            hotend_max_c=u16(148),
            hotend_min_c=u16(150),
            sku=str(struct.unpack_from("<I", img, 96)[0]),
            extra={"manufacturer": cstr(img, 32, 16), "signature": "unchecked"},
        )


FORMAT = Snapmaker
