"""Bambu Lab — MIFARE Classic 1K; the sixteen keys A are HKDF-SHA256 over the first four UID
bytes with the brand's salt and context, 96 bytes sliced into six (Bambu-Research-Group's
RFID-Tag-Guide).  The record: blocks 1–2 (variant, material id, type), 4–6 (name, colour,
weight, diameter, drying, bed and hot-end temperatures), 12 (production date), 14 (length)."""

import struct

from .base import SpoolRecord, TagFormat, cstr
from .crypto import hkdf_sha256

KEY_BYTES = 6
SECTORS = 16


class Bambu(TagFormat):
    name = "bambu"
    chip = "mifare"
    needs = ("hkdf_salt", "hkdf_info")
    sectors = {0: 0b110, 1: 0b111, 3: 0b101}

    def key_for(self, uid, sector):
        salt = bytes.fromhex(self.params["hkdf_salt"])
        info = bytes.fromhex(self.params["hkdf_info"])
        okm = hkdf_sha256(uid[:4], salt, info, KEY_BYTES * SECTORS)
        return okm[KEY_BYTES * sector : KEY_BYTES * sector + KEY_BYTES]

    def decode(self, chip, got):
        if not self._all_read(got):
            return None
        b = got.blocks
        diameter = struct.unpack_from("<f", b[5], 8)[0]
        dry_t, dry_h, _bed_type, bed_t, hot_max, hot_min = struct.unpack_from("<6H", b[6], 0)
        return SpoolRecord(
            format=self.name,
            uid=chip.uid.hex().upper(),
            brand="Bambu Lab",
            material=cstr(b[2], 0, 16) or None,
            name=cstr(b[4], 0, 16) or None,
            color_rgba=b[5][0:4].hex().upper(),
            diameter_mm=round(diameter, 3),
            weight_g=struct.unpack_from("<H", b[5], 4)[0],
            length_m=struct.unpack_from("<H", b[14], 4)[0],
            hotend_min_c=hot_min,
            hotend_max_c=hot_max,
            bed_min_c=bed_t,
            bed_max_c=bed_t,
            dry_temp_c=dry_t,
            dry_hours=dry_h,
            sku=cstr(b[1], 8, 8) or None,
            extra={"variant": cstr(b[1], 0, 8), "production": cstr(b[12], 0, 16)},
        )


FORMAT = Bambu
