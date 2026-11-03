"""Decodes the Anycubic tag, an NTAG213 with the record from page 4.

The layout is the one of ACE-RFID by DnG-Crafts. The record starts with the magic bytes
7B 00 65 00. Then come the SKU, brand and material strings, a packed colour, the temperature
ranges, the diameter and the weight. All numbers are little-endian. The tag needs no key.
"""

from __future__ import annotations

import struct

from .base import Chip, Got, SpoolRecord, Step, TagFormat, cstr, ntag_steps

MAGIC = b"\x7b\x00\x65\x00"
FIRST_PAGE, LAST_PAGE = 4, 39  # the user area of the NTAG213


class Anycubic(TagFormat):
    name = "anycubic"
    chip = "ntag"

    def steps(self, _chip: Chip, got: Got) -> list[Step]:
        return ntag_steps(got, FIRST_PAGE, LAST_PAGE)

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        img = got.image(FIRST_PAGE, LAST_PAGE)
        if img is None or img[0:4] != MAGIC:
            return None
        packed = struct.unpack_from("<I", img, 64)[0]
        r, g, b = (packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF
        tmin, tmax = struct.unpack_from("<HH", img, 80)
        bmin, bmax = struct.unpack_from("<HH", img, 100)
        return SpoolRecord(
            format=self.name,
            uid=chip.uid_text,
            brand=cstr(img, 24, 20) or "Anycubic",
            material=cstr(img, 44, 20) or None,
            color_rgba=f"{r:02X}{g:02X}{b:02X}FF",
            diameter_mm=struct.unpack_from("<H", img, 104)[0] / 100.0,
            weight_g=struct.unpack_from("<I", img, 108)[0],
            hotend_min_c=tmin,
            hotend_max_c=tmax,
            bed_min_c=bmin,
            bed_max_c=bmax,
            sku=cstr(img, 4, 20) or None,
        )


FORMAT = Anycubic
