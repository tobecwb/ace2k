"""Decodes the Bambu Lab tag, a MIFARE Classic 1K.

The sixteen keys A come from HKDF-SHA256 over the first four bytes of the UID, with the salt and
the context of the brand. The 96 bytes of output are cut into sixteen keys of six bytes. This
follows the RFID-Tag-Guide of Bambu-Research-Group.

The record is in these blocks.
- Blocks 1 and 2: the variant, the material id and the type.
- Blocks 4 to 6: the name, colour, weight, diameter, drying data, and the bed and hot-end
  temperatures.
- Block 12: the production date.
- Block 14: the length.
"""

from __future__ import annotations

import struct

from .base import Chip, Got, SpoolRecord, TagFormat, cstr
from .crypto import hkdf_sha256

KEY_BYTES = 6
SECTORS = 16


class Bambu(TagFormat):
    name = "bambu"
    chip = "mifare"
    needs = ("hkdf_salt", "hkdf_info")
    sectors = {0: 0b110, 1: 0b111, 3: 0b101}

    def key_for(self, uid: bytes, sector: int) -> bytes:
        salt = bytes.fromhex(self.text("hkdf_salt"))
        info = bytes.fromhex(self.text("hkdf_info"))
        okm = hkdf_sha256(uid[:4], salt, info, KEY_BYTES * SECTORS)
        return okm[KEY_BYTES * sector : KEY_BYTES * sector + KEY_BYTES]

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        if not self._all_read(got):
            return None
        b = got.blocks
        diameter = struct.unpack_from("<f", b[5], 8)[0]
        dry_t, dry_h, _bed_type, bed_t, hot_max, hot_min = struct.unpack_from("<6H", b[6], 0)
        return SpoolRecord(
            format=self.name,
            uid=chip.uid_text,
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
