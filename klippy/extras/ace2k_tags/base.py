"""What every brand module shares.

This module defines the chip that the unit reported, a read step, what the session has read so
far, the shape of the record, and the base class that a brand module fills in.

A brand module answers three questions.
- ``claims(chip)``: can this chip carry my tag?
- ``steps(chip, got)``: what do I still need to read? An empty list means nothing more.
- ``decode(chip, got)``: what is the record? The answer is None when the bytes are not mine.

The session (``ace2k_rfid.py``) asks these questions, sends the steps to the unit and fills
`got`. The unit never sees a brand.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from typing import TYPE_CHECKING, Union, cast

if TYPE_CHECKING:
    from ..ace2k_klipper_types import Record

OP_NTAG = 0
OP_MIFARE_A = 1
OP_MIFARE_B = 2
KEY_A = 0
KEY_B = 1
SAK_NTAG = 0x00
SAK_MIFARE_1K = 0x08
NTAG_UID_BYTES = 7
PAGES_PER_READ = 4
READS_PER_STEP = 3
BLOCKS_PER_SECTOR = 4
DATA_BLOCKS = 3  # blocks 0 to 2 of a sector; the trailer is never read

# The section of a brand as the session passes it on. The options are strings, and the `enabled`
# flag is a bool.
TagParams = Mapping[str, Union[str, bool]]
# The extras that are specific to a brand. They are published with the record.
Extra = dict[str, Union[str, int]]


@dataclass(frozen=True)
class Chip:
    uid: bytes
    atqa: int
    sak: int

    @property
    def uid_text(self) -> str:
        """Return the UID as the upper-case hex string that the records and the cache use."""
        return self.uid.hex().upper()

    @property
    def is_ntag(self) -> bool:
        return self.sak == SAK_NTAG and len(self.uid) == NTAG_UID_BYTES

    @property
    def is_mifare_1k(self) -> bool:
        return self.sak == SAK_MIFARE_1K


@dataclass(frozen=True)
class Step:
    op: int  # OP_NTAG, OP_MIFARE_A, OP_MIFARE_B
    arg: int  # the first page, or the sector
    blocks: int  # NTAG: groups of four pages (1 to 3); MIFARE: mask of data blocks (bits 0-2)
    key: bytes = b""


class Got:
    """What a session has read so far.

    ``pages`` maps an NTAG page to its 4 bytes. ``blocks`` maps a MIFARE block to its 16 bytes.
    ``auth_failed`` holds the (sector, key type) pairs whose authentication failed.
    """

    def __init__(self) -> None:
        self.pages: dict[int, bytes] = {}
        self.blocks: dict[int, bytes] = {}
        self.auth_failed: set[tuple[int, int]] = set()

    def image(self, first_page: int, last_page: int) -> bytes | None:
        """Return the NTAG pages from first to last as one byte string.

        Return None if a page is missing.
        """
        if any(p not in self.pages for p in range(first_page, last_page + 1)):
            return None
        return b"".join(self.pages[p] for p in range(first_page, last_page + 1))


@dataclass
class SpoolRecord:
    # The record is published whole through as_dict(). So a field that no code reads by name is
    # not dead. The noqa on each field tells vulture so.
    format: str  # noqa: V107
    uid: str
    brand: str | None = None
    material: str | None = None
    name: str | None = None
    color_rgba: str | None = None
    diameter_mm: float | None = None  # noqa: V107
    weight_g: int | None = None
    length_m: int | None = None  # noqa: V107
    hotend_min_c: int | None = None  # noqa: V107
    hotend_max_c: int | None = None  # noqa: V107
    bed_min_c: int | None = None  # noqa: V107
    bed_max_c: int | None = None  # noqa: V107
    dry_temp_c: int | None = None  # noqa: V107
    dry_hours: int | None = None  # noqa: V107
    sku: str | None = None  # noqa: V107
    extra: Extra = field(default_factory=dict)  # noqa: V107

    def as_dict(self) -> Record:
        return asdict(self)

    def summary(self) -> str:
        parts = [p for p in (self.brand, self.material, self.name) if p]
        if self.color_rgba:
            parts.append("#" + self.color_rgba[:6])
        if self.weight_g:
            parts.append(f"{self.weight_g} g")
        return " ".join(parts) if parts else f"UID {self.uid}"


def ntag_steps(got: Got, first_page: int, last_page: int) -> list[Step]:
    """Return the NTAG reads still missing between two pages.

    A step reads up to three groups of four pages.
    """
    steps: list[Step] = []
    page = first_page
    while page <= last_page:
        span = [p for p in range(page, page + PAGES_PER_READ * READS_PER_STEP) if p <= last_page]
        missing = [p for p in span if p not in got.pages]
        if missing:
            groups = min(READS_PER_STEP, (last_page - page) // PAGES_PER_READ + 1)
            steps.append(Step(OP_NTAG, page, groups))
        page += PAGES_PER_READ * READS_PER_STEP
    return steps


def cstr(buf: bytes | bytearray, off: int, n: int) -> str:
    return bytes(buf[off : off + n]).split(b"\0", 1)[0].decode("utf-8", "replace").strip()


class TagFormat:
    """The base class of a brand.

    ``name`` is the name of the section. ``chip`` is "ntag" or "mifare". ``needs`` lists the
    parameters that the section must carry.
    A MIFARE brand lists its sectors in `sectors`, as a map from the sector to a mask of blocks.
    It makes the key of each sector in ``key_for()``. Then ``steps()`` is shared.
    """

    name = ""
    chip = ""
    needs: tuple[str, ...] = ()
    sectors: dict[int, int] = {}

    def __init__(self, params: TagParams) -> None:
        self.params = dict(params)

    def text(self, option: str, default: str | None = None) -> str:
        """Return the option `option` of the section, or `default` if the section lacks it.

        Every option except `enabled` is a string.
        """
        value = self.params[option] if default is None else self.params.get(option, default)
        return cast(str, value)

    def claims(self, chip: Chip) -> bool:
        return chip.is_ntag if self.chip == "ntag" else chip.is_mifare_1k

    def key_for(self, uid: bytes, sector: int) -> bytes:
        raise NotImplementedError

    def steps(self, chip: Chip, got: Got) -> list[Step]:
        for sector, mask in sorted(self.sectors.items()):
            if (sector, KEY_A) in got.auth_failed:
                return []  # a key of this brand did not open the tag, so it is not this brand
            wanted = [sector * BLOCKS_PER_SECTOR + i for i in range(DATA_BLOCKS) if mask >> i & 1]
            if any(b not in got.blocks for b in wanted):
                return [Step(OP_MIFARE_A, sector, mask, self.key_for(chip.uid, sector))]
        return []

    def decode(self, chip: Chip, got: Got) -> SpoolRecord | None:
        raise NotImplementedError

    def _all_read(self, got: Got) -> bool:
        return all(
            sector * BLOCKS_PER_SECTOR + i in got.blocks
            for sector, mask in self.sectors.items()
            for i in range(DATA_BLOCKS)
            if mask >> i & 1
        )
