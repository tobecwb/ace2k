"""Decodes the Elegoo tag, an NTAG213 with a 32-byte record after the marker EE EE EE EE.

Two layouts exist. The decoder of OpenRFID was written against real tags. It reads the record at
page 16 in a layout aligned to 4 bytes, and this layout is tried first. The ELEGOO-RFID-Tag-Guide
of elegooofficial describes a packed layout at page 4, which is the fallback.

Integers are big-endian. The material is in ASCII, or in BCD of ASCII codes. The tag needs no key.
"""

from __future__ import annotations

import struct
from typing import Callable, TypedDict

from .base import Chip, Got, SpoolRecord, Step, TagFormat, ntag_steps

MARKER = b"\xee\xee\xee\xee"
FIRST_PAGE, LAST_PAGE = 4, 39
DIAMETER_RANGE = (1.0, 3.0)
WEIGHT_RANGE = (50, 5000)
RECORD_BYTES = 32
PRINTABLE = (32, 127)  # the ASCII codes that a BCD material may spell: space up to tilde


class _Fields(TypedDict):
    # The keys are read by name, which vulture does not see. The noqa on a key says so.
    material: str
    color_rgba: str
    diameter_mm: float  # noqa: V107
    weight_g: int


class Fields(_Fields, total=False):
    """The fields of the record as a layout reads them, before the plausibility check."""

    hotend_min_c: int  # noqa: V107
    hotend_max_c: int  # noqa: V107


def material(raw4: bytes) -> str:
    digits = [f"{b:02X}" for b in raw4 if b]
    lo, end = PRINTABLE
    bcd = "".join(chr(int(d)) for d in digits if d.isdigit() and lo <= int(d) < end)
    ascii_ = raw4.decode("ascii", "replace").strip("\0 ")
    return ascii_ if ascii_.isalnum() else bcd


def aligned(blk: bytes) -> Fields:
    return Fields(
        material=material(blk[8:12]),
        color_rgba=blk[16:20].hex().upper(),
        hotend_min_c=struct.unpack_from(">H", blk, 20)[0],
        hotend_max_c=struct.unpack_from(">H", blk, 22)[0],
        diameter_mm=struct.unpack_from(">H", blk, 28)[0] / 100.0,
        weight_g=struct.unpack_from(">H", blk, 30)[0],
    )


def packed(blk: bytes) -> Fields:
    return Fields(
        material=material(blk[7:11]),
        color_rgba=blk[15:18].hex().upper() + "FF",
        diameter_mm=struct.unpack_from(">H", blk, 18)[0] / 100.0,
        weight_g=struct.unpack_from(">H", blk, 20)[0],
    )


def plausible(rec: Fields) -> bool:
    lo, hi = DIAMETER_RANGE
    wlo, whi = WEIGHT_RANGE
    return lo <= rec["diameter_mm"] <= hi and wlo <= rec["weight_g"] <= whi


class Elegoo(TagFormat):
    name = "elegoo"
    chip = "ntag"

    def steps(self, _chip: Chip, got: Got) -> list[Step]:
        return ntag_steps(got, FIRST_PAGE, LAST_PAGE)

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        img = got.image(FIRST_PAGE, LAST_PAGE)
        if img is None:
            return None
        layouts: tuple[tuple[int, Callable[[bytes], Fields]], ...] = ((16, aligned), (4, packed))
        for page, layout in layouts:
            off = (page - FIRST_PAGE) * 4
            blk = img[off : off + 48]
            if len(blk) < RECORD_BYTES or blk[1:5] != MARKER:
                continue
            fields = layout(blk)
            if plausible(fields):
                return SpoolRecord(
                    format=self.name,
                    uid=chip.uid_text,
                    brand="Elegoo",
                    extra={"layout": layout.__name__, "page": page},
                    **fields,
                )
        return None


FORMAT = Elegoo
