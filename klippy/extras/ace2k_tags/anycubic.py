"""Anycubic — NTAG213, the record from page 4 (DnG-Crafts ACE-RFID's layout): the magic
7B 00 65 00, then SKU, brand and material strings, a packed colour, the temperature ranges, the
diameter and the weight, little-endian.  No key."""

import struct

from .base import SpoolRecord, TagFormat, cstr, ntag_steps

MAGIC = b"\x7b\x00\x65\x00"
FIRST_PAGE, LAST_PAGE = 4, 39  # the NTAG213 user area


class Anycubic(TagFormat):
    name = "anycubic"
    chip = "ntag"

    def steps(self, chip, got):
        return ntag_steps(got, FIRST_PAGE, LAST_PAGE)

    def decode(self, chip, got):
        img = got.image(FIRST_PAGE, LAST_PAGE)
        if img is None or img[0:4] != MAGIC:
            return None
        packed = struct.unpack_from("<I", img, 64)[0]
        r, g, b = (packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF
        tmin, tmax = struct.unpack_from("<HH", img, 80)
        bmin, bmax = struct.unpack_from("<HH", img, 100)
        return SpoolRecord(
            format=self.name,
            uid=chip.uid.hex().upper(),
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
