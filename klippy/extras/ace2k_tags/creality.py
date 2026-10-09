"""Creality — MIFARE Classic 1K; the key A of sector 1 is the first six bytes of the four UID
bytes, repeated four times, encrypted with AES-128-ECB under the brand's uid key; blocks 4–6 hold
48 ASCII characters encrypted with AES-128-ECB under its data key (DnG-Crafts' K2-RFID).  The
characters: date (5), vendor id (4), batch (2), filament id (6), colour (7: a leading 0, then
RRGGBB), length in metres (4), serial (6), reserve (6), padding (8).  The material is looked up in
the section's optional `materials` table (filament id = name, one per line)."""

from .base import SpoolRecord, TagFormat
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


def fields(text):
    out, pos = {}, 0
    for key, width in WIDTHS:
        out[key] = text[pos : pos + width]
        pos += width
    return out


class Creality(TagFormat):
    name = "creality"
    chip = "mifare"
    needs = ("uid_key", "data_key")
    sectors = {1: 0b111}

    def key_for(self, uid, sector):
        block = (uid[:4] * 4)[:16]
        return aes128_encrypt_block(bytes.fromhex(self.params["uid_key"]), block)[:KEY_BYTES]

    def materials(self):
        table = {}
        for line in self.params.get("materials", "").splitlines():
            if "=" in line:
                fid, name = (p.strip() for p in line.split("=", 1))
                table[fid] = name
        return table

    def decode(self, chip, got):
        if not self._all_read(got):
            return None
        key = bytes.fromhex(self.params["data_key"])
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
            uid=chip.uid.hex().upper(),
            brand="Creality" if f["vendor_id"] == CREALITY_VENDOR else f"vendor {f['vendor_id']}",
            material=self.materials().get(f["filament_id"]),
            color_rgba=f["color"][1:7].upper() + "FF",
            length_m=int(f["length"]) if f["length"].isdigit() else None,
            sku=f["filament_id"],
            extra={k: f[k] for k in ("date", "vendor_id", "batch", "serial")},
        )


FORMAT = Creality
