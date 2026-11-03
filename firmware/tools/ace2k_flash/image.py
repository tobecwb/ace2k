"""The images the flasher writes.

This module holds the layout of the unit's memory, and tells an ace2k image from a factory image.
"""

from __future__ import annotations

import json
import re
import struct
import zlib
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import TYPE_CHECKING, cast

from .wire import RefusedError, crc16

if TYPE_CHECKING:
    from .wire import Json

# --- Images (firmware/tools/mkimage.py, docs/hardware.md "Memory map") ------------------------
TRAILER = bytes.fromhex("61a5635a65a5325a")
# The unit's memory.  firmware/tools/mkimage.py checks an image against the same limits.
SRAM_BASE, SRAM_END = 0x20000000, 0x2000C000
APP_BASE, APP_END = 0x08008000, 0x08024000
APP_SLOT_BYTES = APP_END - APP_BASE  # 112 KB
# ace2k keeps its config in the last 2 KB page of the slot.  So an ace2k image must end before
# that page.
CONFIG_PAGE = 0x08023800
ACE2K_MAX_BYTES = CONFIG_PAGE - APP_BASE  # 110 KB
ANNOUNCE_MAX = 11  # a longer version string gets no answer to the announcement (flashing.md)
VECTOR_TABLE_BYTES = 8  # the initial stack pointer and the reset vector
# The shortest file that holds a vector table and the trailer
CONTAINER_MIN_BYTES = VECTOR_TABLE_BYTES + len(TRAILER)
# The shape of an older image's version string, as the build embeds it.  It is a release
# (0.11.0, 0.4.0-50-g52ae07e) or a bare commit (a1b2c3d).
# VERSION_CHARS lists the bytes a version string may hold.  The byte just before a version
# string that the image embeds must not be one of them.
VERSION_SHAPE = re.compile(
    r"[0-9]+\.[0-9]+(\.[0-9]+)?([-+.~][0-9A-Za-z.+~-]+)?|[0-9a-f]{7,40}(-[0-9A-Za-z.-]+)?"
)
VERSION_CHARS = frozenset(b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.+-_~")


@dataclass
class Image:
    """An image ready to write, with its kind and its version."""

    data: bytes
    kind: str  # "ace2k" or "factory"
    version: str
    embedded: bool = False  # the version came from the image, not from --version
    # How long, in seconds, a newly written image waits for a host to talk to it.  If no host
    # does, the image goes back to recovery.  The value comes from the image's dictionary.
    window_s: int | None = None

    @property
    def announce(self) -> str:
        return self.version[:ANNOUNCE_MAX]

    @cached_property
    def crc(self) -> int:
        """The CRC-16 of the image, worked out once.  Nothing changes data after it is read."""
        return crc16(self.data)

    def has_version(self, version: str) -> bool:
        """Tell whether version belongs to the image.

        It does when it is the image's own version.  For an image built before the version
        constant, it also does when the image embeds the version string whole.  The string must
        have a version's shape, no version character may come before it, and a NUL must end it.
        """
        if version == self.version:
            return True
        if self.embedded or not VERSION_SHAPE.fullmatch(version):
            return False
        whole = version.encode() + b"\0"
        at = self.data.find(whole)
        while at >= 0:
            if at == 0 or self.data[at - 1] not in VERSION_CHARS:
                return True
            at = self.data.find(whole, at + 1)
        return False


def embedded_dictionary(body: bytes) -> Json | None:
    """Return the Klipper data dictionary compiled into an ace2k image, or None."""
    for m in re.finditer(rb"\x78[\x01\x5e\x9c\xda]", body):
        try:
            raw = zlib.decompressobj().decompress(body[m.start() :])
        except zlib.error:
            continue
        if raw.startswith(b"{"):
            try:
                return cast("Json", json.loads(raw))  # it starts with "{", so it is an object
            except ValueError:
                continue
    return None


def has_dictionary_start(body: bytes) -> bool:
    """Tell whether body holds the start of a compressed Klipper dictionary.

    The dictionary may be damaged further on.  A start is a zlib stream whose first bytes
    decompress to the start of a JSON object.  Only the first 16 bytes of output are made.  The
    input is not cut short, because the first compressed bytes of a real dictionary are a table
    that gives no output on its own.
    """
    for m in re.finditer(rb"\x78[\x01\x5e\x9c\xda]", body):
        try:
            head = zlib.decompressobj().decompress(body[m.start() :], 16)
        except zlib.error:
            continue
        if head.startswith(b'{"'):
            return True
    return False


def vector_table_ok(data: bytes) -> bool:
    """Tell whether the vector table is valid.

    The initial stack pointer must lie in SRAM.  The reset vector must be a Thumb address inside
    the application slot.
    """
    sp, entry = cast("tuple[int, int]", struct.unpack_from("<II", data, 0))
    return SRAM_BASE < sp <= SRAM_END and APP_BASE <= entry < APP_END and entry & 1 == 1


def _read(path: Path) -> bytes:
    """Return the bytes of the file at path.  A file that cannot be read is refused."""
    try:
        return path.read_bytes()
    except OSError as e:
        raise RefusedError(f"{path}: {e.strerror}") from None


def _check_container(path: Path, data: bytes) -> None:
    """Refuse data unless it is an image for the unit.

    It must end with the bootloader's trailer, fit the application slot, and start with a vector
    table that points into the unit's memory.
    """
    if len(data) < CONTAINER_MIN_BYTES or data[-8:] != TRAILER:
        raise RefusedError(f"{path}: not an image for the unit (no bootloader trailer at its end)")
    if len(data) > APP_SLOT_BYTES:
        raise RefusedError(f"{path}: {len(data)} bytes, larger than the {APP_SLOT_BYTES}-byte slot")
    if not vector_table_ok(data):
        raise RefusedError(
            f"{path}: not an application for the unit "
            "(its vector table points outside the unit's memory)"
        )


def _window_s(dictionary: Json) -> int | None:
    """Return how long, in seconds, a newly written image waits for a host.

    The value comes from the image's dictionary.  It returns None for an image built before that
    constant existed.
    """
    try:
        return int(dictionary.get("config", {})["ACE2K_LINKPROOF_WINDOW_MS"]) // 1000
    except (KeyError, TypeError, ValueError):
        return None


def _ace2k_image(path: Path, data: bytes, version: str | None, dictionary: Json | None) -> Image:
    """Return an ace2k image, with the version it carries or the one passed with --version.

    These images are refused:

    - an image that reaches the config page;
    - an image with no dictionary;
    - an image with no version, when no --version is passed;
    - an image whose version differs from the one passed.
    """
    if len(data) > ACE2K_MAX_BYTES:
        raise RefusedError(
            f"{path}: {len(data)} bytes; an ace2k image must end before the config page at the "
            f"end of the slot, so it takes at most {ACE2K_MAX_BYTES} bytes; build it with "
            "`make build`, which checks the size"
        )
    if dictionary is None:
        raise RefusedError(f"{path}: has ace2k's checksum but no Klipper dictionary inside")
    config = dictionary.get("config")
    embedded = config.get("ACE2K_VERSION") if isinstance(config, dict) else None
    if not isinstance(embedded, str) or not embedded:  # an empty version is no version
        embedded = None
    if embedded is None and not version:
        raise RefusedError(
            f"{path}: this ace2k image carries no version (an image from before v0.12 does "
            "not); pass --version"
        )
    if embedded is not None and version and version != embedded:
        raise RefusedError(f"{path}: --version {version} but the image says {embedded}")
    named = cast(str, embedded or version)  # the checks above refuse an image with neither
    return Image(data, "ace2k", named, embedded is not None, _window_s(dictionary))


def _factory_image(path: Path, data: bytes, version: str | None, body: bytes) -> Image:
    """Return a factory image.  It carries no version that this tool reads, so --version is
    required.

    An image with a Klipper dictionary inside is an ace2k image whose checksum does not match.
    This holds even when the dictionary itself is damaged.  Such an image is damaged, and it is
    refused.
    """
    if embedded_dictionary(body) is not None or has_dictionary_start(body):
        raise RefusedError(
            f"{path}: a damaged ace2k image (its checksum does not match its contents); "
            "download or build it again"
        )
    if not version:
        raise RefusedError(f"{path}: a factory image; pass its version, e.g. --version 1.1.31")
    return Image(data, "factory", version)


def load_image(path: Path, version: str | None) -> Image:
    """Return the image in the file at path: an ace2k image or a factory image.

    version is the value of --version, or None.

    An image whose checksum matches its body is an ace2k image.  Any other image is a factory
    image, unless it carries a Klipper dictionary.  The refusals come in this order:

    - a file that cannot be read;
    - a file that is not an image for the unit: no bootloader trailer, larger than the
      application slot, or a vector table that points outside the unit's memory;
    - an ace2k image that reaches the config page;
    - an ace2k image with no Klipper dictionary inside;
    - an ace2k image with no version when no --version is passed;
    - an ace2k image whose version differs from --version;
    - an image with a Klipper dictionary inside whose checksum does not match: a damaged ace2k
      image;
    - a factory image when no --version is passed.
    """
    data = _read(path)
    _check_container(path, data)
    body = data[:-12]
    if struct.unpack_from("<I", data, len(data) - 12)[0] == zlib.crc32(body):
        return _ace2k_image(path, data, version, embedded_dictionary(body))
    return _factory_image(path, data, version, body)
