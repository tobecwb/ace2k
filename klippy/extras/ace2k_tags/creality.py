"""Decodes the Creality tag, a MIFARE Classic 1K.

The layout follows K2-RFID by DnG-Crafts.

The key A of sector 1 is made in two steps. Repeat the first four bytes of the UID four times to
get 16 bytes. Encrypt them with AES-128-ECB under the uid key of the brand. The key is the first
six bytes of the result.

Blocks 4 to 6 hold 48 ASCII characters, encrypted with AES-128-ECB under the data key of the
brand. The characters are, in order:
- the date (5),
- the vendor id (4),
- the batch (2),
- the filament id (6),
- the colour (7, a leading 0 and then RRGGBB),
- the length in metres (4),
- the serial (6),
- a reserve (6),
- padding (8).

The material comes from the optional `materials` table of the section. The table has one line
per material, in the form `filament id = name`.
"""

from __future__ import annotations

from .base import Chip, Got, SpoolRecord, TagFormat
from .crypto import aes128_decrypt_block, aes128_encrypt_block

KEY_BYTES = 6
WIDTHS = (
    ("date", 5),
    ("vendor_id", 4),
    ("batch", 2),
    ("filament_id", 6),
    ("color", 7),
    ("length", 4),
    ("serial", 6),
    ("reserve", 6),
)
CREALITY_VENDOR = "0276"


def fields(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    pos = 0
    for key, width in WIDTHS:
        out[key] = text[pos : pos + width]
        pos += width
    return out


class Creality(TagFormat):
    name = "creality"
    chip = "mifare"
    needs = ("uid_key", "data_key")
    sectors = {1: 0b111}

    def key_for(self, uid: bytes, _sector: int) -> bytes:
        block = (uid[:4] * 4)[:16]
        return aes128_encrypt_block(bytes.fromhex(self.text("uid_key")), block)[:KEY_BYTES]

    def materials(self) -> dict[str, str]:
        table: dict[str, str] = {}
        for line in self.text("materials", "").splitlines():
            if "=" in line:
                fid, name = (p.strip() for p in line.split("=", 1))
                table[fid] = name
        return table

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        if not self._all_read(got):
            return None
        key = bytes.fromhex(self.text("data_key"))
        plain = b"".join(aes128_decrypt_block(key, got.blocks[b]) for b in (4, 5, 6))
        try:
            text = plain.decode("ascii")
        except UnicodeDecodeError:
            return None
        if not text.isalnum():
            return None
        f = fields(text)
        return SpoolRecord(
            format=self.name,
            uid=chip.uid_text,
            brand="Creality" if f["vendor_id"] == CREALITY_VENDOR else f"vendor {f['vendor_id']}",
            material=self.materials().get(f["filament_id"]),
            color_rgba=f["color"][1:7].upper() + "FF",
            length_m=int(f["length"]) if f["length"].isdigit() else None,
            sku=f["filament_id"],
            extra={k: f[k] for k in ("date", "vendor_id", "batch", "serial")},
        )


FORMAT = Creality
