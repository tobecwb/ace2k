"""Decodes the Snapmaker tag, a MIFARE Classic 1K.

The key A of sector i is the first 6 bytes of HMAC-SHA256(PRK, "key_a_<i>" + 0x01). PRK is
HMAC-SHA256(salt, first four bytes of the UID). This follows the u1-klipper reader of Snapmaker.

The record is in sectors 0 to 2, with little-endian integers. This follows the layout of
OpenRFID.

The record is RSA-signed. The signature covers the sector trailers as the tag returns them, and
this release never reads them. So the signature is reported as "unchecked".

The tag of a Snapmaker spool sits by the hub, out of reach of the antenna of the bay (see
"RFID readers" in docs/hardware.md). It reads only when the tag is held at the antenna.
"""

from __future__ import annotations

import struct

from .base import BLOCKS_PER_SECTOR, DATA_BLOCKS, Chip, Got, SpoolRecord, TagFormat, cstr
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
IMAGE_BLOCKS = [b for b in range(11) if b % BLOCKS_PER_SECTOR < DATA_BLOCKS]  # no trailers


class Snapmaker(TagFormat):
    name = "snapmaker"
    chip = "mifare"
    needs = ("salt_a",)
    sectors = {0: 0b111, 1: 0b111, 2: 0b111}

    def key_for(self, uid: bytes, sector: int) -> bytes:
        prk = hmac_sha256(self.text("salt_a").encode(), uid[:4])
        return hmac_sha256(prk, f"key_a_{sector}".encode() + b"\x01")[:KEY_BYTES]

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        if not self._all_read(got):
            return None
        img = bytearray(176)
        for blk in IMAGE_BLOCKS:
            img[blk * 16 : blk * 16 + 16] = got.blocks[blk]

        def u16(off: int) -> int:
            value: int = struct.unpack_from("<H", img, off)[0]
            return value

        colors = img[72]
        return SpoolRecord(
            format=self.name,
            uid=chip.uid_text,
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
