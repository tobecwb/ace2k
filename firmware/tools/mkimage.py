#!/usr/bin/env python3
"""Wrap a linked Klipper binary into an image the ACE 2 Pro bootloader accepts.

    image = raw binary, padded to 4 bytes with 0xFF,
            + its CRC-32 (IEEE, little-endian; the firmware's self-test recomputes it over the
              padded binary),
            + the 8-byte trailer 61 A5 63 5A 65 A5 32 5A

The bootloader accepts an image whose initial stack pointer lies in SRAM and whose entry is a
Thumb address inside the application slot (0x08008000..0x08024000, 112 KB); the trailer must be
the last 8 bytes.  The checksum the updater announces is computed by the updater itself.

The last 2 KB page of the slot (0x08023800) is the firmware's config page: an image reaching into
it is refused unconditionally.  --max-bytes N refuses an image larger than N bytes (the project's
size budget, default 104 KB), independently of that limit.
"""

import argparse
import hashlib
import struct
import sys
import zlib
from typing import NoReturn

MAGIC = bytes.fromhex("61a5635a65a5325a")
APP_BASE, APP_END = 0x08008000, 0x08024000
CONFIG_PAGE = 0x08023800
SRAM_BASE, SRAM_END = 0x20000000, 0x2000C000
DEFAULT_MAX_BYTES = 104 * 1024


def fail(msg: str) -> NoReturn:
    print(f"mkimage: REFUSED — {msg}", file=sys.stderr)
    sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("raw", help="klipper.bin as linked")
    ap.add_argument("out", help="the image to write")
    ap.add_argument(
        "--max-bytes", type=int, default=DEFAULT_MAX_BYTES, help="size budget (default 106496)"
    )
    args = ap.parse_args()

    with open(args.raw, "rb") as f:
        raw = bytearray(f.read())
    if len(raw) < 8:
        fail("raw binary shorter than a vector table")
    raw += b"\xff" * ((-len(raw)) % 4)
    body = bytes(raw)
    crc = zlib.crc32(body) & 0xFFFFFFFF
    img = body + struct.pack("<I", crc) + MAGIC

    sp, entry = struct.unpack_from("<II", img, 0)
    if not (SRAM_BASE < sp <= SRAM_END):
        fail(f"initial SP 0x{sp:08X} outside SRAM 0x{SRAM_BASE:08X}..0x{SRAM_END:08X}")
    if not (entry & 1):
        fail(f"entry 0x{entry:08X} is not a Thumb address")
    if not (APP_BASE <= entry < APP_END):
        fail(f"entry 0x{entry:08X} outside the application slot")
    if len(img) > CONFIG_PAGE - APP_BASE:  # tighter than the slot itself, so the only slot check
        fail(f"image {len(img)} B would overlap the config page at 0x{CONFIG_PAGE:08X}")
    if len(img) > args.max_bytes:
        fail(f"image {len(img)} B exceeds the {args.max_bytes} B limit")

    with open(args.out, "wb") as f:
        f.write(img)
    digest = hashlib.sha256(img).hexdigest()
    print(
        f"mkimage: OK  {args.out}  size={len(img)}  sp=0x{sp:08X}  entry=0x{entry:08X}"
        f"  crc32=0x{crc:08X}  sha256={digest}"
    )


if __name__ == "__main__":
    main()
