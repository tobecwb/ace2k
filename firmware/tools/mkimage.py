#!/usr/bin/env python3
"""Wrap a linked Klipper binary into an image the ACE 2 Pro bootloader accepts.

The image is made of three parts, in this order:

    1. the raw binary, padded with 0xFF to a multiple of 4 bytes;
    2. the CRC-32 of the padded binary (IEEE, little-endian).  The firmware's self-test computes
       it again over the same bytes;
    3. the 8-byte trailer 61 A5 63 5A 65 A5 32 5A.

The bootloader accepts an image only when these hold:

- the initial stack pointer lies in SRAM;
- the entry point is a Thumb address inside the application slot (0x08008000..0x08024000,
  112 KB);
- the trailer is the last 8 bytes.

The updater computes the checksum it announces by itself.

The last 2 KB page of the slot (0x08023800) holds the firmware's config.  An image that reaches
into that page is always refused.  --max-bytes N also refuses an image larger than N bytes.  This
is the project's size budget, 104 KB by default, and it is checked apart from the config page.
"""

from __future__ import annotations

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
VECTOR_TABLE_BYTES = 8  # the initial stack pointer and the reset vector


class RefusedError(Exception):
    """An image the bootloader would not accept; main() prints it and returns 1."""


def fail(msg: str) -> NoReturn:
    raise RefusedError(msg)


def main(argv: list[str] | None = None) -> int:
    try:
        return build(argv)
    except RefusedError as refusal:
        print(f"mkimage: REFUSED — {refusal}", file=sys.stderr)
        return 1


def build(argv: list[str] | None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("raw", help="klipper.bin as linked")
    ap.add_argument("out", help="the image to write")
    ap.add_argument(
        "--max-bytes", type=int, default=DEFAULT_MAX_BYTES, help="size budget (default %(default)s)"
    )
    args = ap.parse_args(argv)

    try:
        with open(args.raw, "rb") as f:
            raw = bytearray(f.read())
    except OSError as e:
        fail(f"cannot read {args.raw}: {e.strerror or e}")
    if len(raw) < VECTOR_TABLE_BYTES:
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
    if len(img) > CONFIG_PAGE - APP_BASE:  # stricter than the slot's end, so no other slot check
        fail(f"image {len(img)} B would overlap the config page at 0x{CONFIG_PAGE:08X}")
    if len(img) > args.max_bytes:
        fail(f"image {len(img)} B exceeds the {args.max_bytes} B limit")

    try:
        with open(args.out, "wb") as f:
            f.write(img)
    except OSError as e:
        fail(f"cannot write {args.out}: {e.strerror or e}")
    digest = hashlib.sha256(img).hexdigest()
    print(
        f"mkimage: OK  {args.out}  size={len(img)}  sp=0x{sp:08X}  entry=0x{entry:08X}"
        f"  crc32=0x{crc:08X}  sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
