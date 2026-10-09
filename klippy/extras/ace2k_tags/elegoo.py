"""Elegoo — NTAG213, a 32-byte record behind the marker EE EE EE EE (elegooofficial's
ELEGOO-RFID-Tag-Guide; OpenRFID's decoder, written against real tags, reads it at page 16 in a
4-byte-aligned layout — tried first — and the guide's packed layout at page 4 as the fallback).
Integers big-endian; the material in ASCII or in BCD of ASCII codes.  No key."""

import struct

from .base import SpoolRecord, TagFormat, ntag_steps

MARKER = b"\xee\xee\xee\xee"
FIRST_PAGE, LAST_PAGE = 4, 39
DIAMETER_RANGE = (1.0, 3.0)
WEIGHT_RANGE = (50, 5000)


def material(raw4):
    digits = [f"{b:02X}" for b in raw4 if b]
    bcd = "".join(chr(int(d)) for d in digits if d.isdigit() and 32 <= int(d) < 127)
    ascii_ = raw4.decode("ascii", "replace").strip("\0 ")
    return ascii_ if ascii_.isalnum() else bcd


def aligned(blk):
    return dict(
        material=material(blk[8:12]),
        color_rgba=blk[16:20].hex().upper(),
        hotend_min_c=struct.unpack_from(">H", blk, 20)[0],
        hotend_max_c=struct.unpack_from(">H", blk, 22)[0],
        diameter_mm=struct.unpack_from(">H", blk, 28)[0] / 100.0,
        weight_g=struct.unpack_from(">H", blk, 30)[0],
    )


def packed(blk):
    return dict(
        material=material(blk[7:11]),
        color_rgba=blk[15:18].hex().upper() + "FF",
        diameter_mm=struct.unpack_from(">H", blk, 18)[0] / 100.0,
        weight_g=struct.unpack_from(">H", blk, 20)[0],
    )


def plausible(rec):
    lo, hi = DIAMETER_RANGE
    wlo, whi = WEIGHT_RANGE
    return lo <= rec["diameter_mm"] <= hi and wlo <= rec["weight_g"] <= whi


class Elegoo(TagFormat):
    name = "elegoo"
    chip = "ntag"

    def steps(self, chip, got):
        return ntag_steps(got, FIRST_PAGE, LAST_PAGE)

    def decode(self, chip, got):
        img = got.image(FIRST_PAGE, LAST_PAGE)
        if img is None:
            return None
        for page, layout in ((16, aligned), (4, packed)):
            off = (page - FIRST_PAGE) * 4
            blk = img[off : off + 48]
            if len(blk) < 32 or blk[1:5] != MARKER:
                continue
            fields = layout(blk)
            if plausible(fields):
                return SpoolRecord(
                    format=self.name,
                    uid=chip.uid.hex().upper(),
                    brand="Elegoo",
                    extra={"layout": layout.__name__, "page": page},
                    **fields,
                )
        return None


FORMAT = Elegoo
