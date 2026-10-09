"""What every brand module shares: the chip the unit reported, a step to read, what the session
has read so far, the record shape, and the base class a brand module fills in.

A brand module answers three questions: claims(chip) — may this chip carry my tag; steps(chip,
got) — what I still need (an empty list: nothing more); decode(chip, got) — the record, or None
when the bytes are not mine.  The session (ace2k_rfid.py) asks, sends the steps to the unit and
fills `got`; the unit never sees a brand."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

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
DATA_BLOCKS = 3  # blocks 0..2 of a sector; the trailer is never read


@dataclass(frozen=True)
class Chip:
    uid: bytes
    atqa: int
    sak: int

    @property
    def is_ntag(self):
        return self.sak == SAK_NTAG and len(self.uid) == NTAG_UID_BYTES

    @property
    def is_mifare_1k(self):
        return self.sak == SAK_MIFARE_1K


@dataclass(frozen=True)
class Step:
    op: int  # OP_NTAG, OP_MIFARE_A, OP_MIFARE_B
    arg: int  # the first page, or the sector
    blocks: int  # NTAG: groups of four pages (1..3); MIFARE: the data blocks' mask (bits 0..2)
    key: bytes = b""


class Got:
    """What a session has read: NTAG pages {page: 4 bytes}, MIFARE blocks {block: 16 bytes}, and
    the (sector, key type) pairs whose authentication failed."""

    def __init__(self):
        self.pages = {}
        self.blocks = {}
        self.auth_failed = set()

    def image(self, first_page, last_page):
        """The NTAG pages first..last as one byte string, or None while one is missing."""
        if any(p not in self.pages for p in range(first_page, last_page + 1)):
            return None
        return b"".join(self.pages[p] for p in range(first_page, last_page + 1))

    def mifare_image(self, blocks):
        """The listed blocks' bytes in order, zeros for a block not read."""
        return b"".join(self.blocks.get(b, bytes(16)) for b in blocks)


@dataclass
class SpoolRecord:
    format: str
    uid: str
    brand: str | None = None
    material: str | None = None
    name: str | None = None
    color_rgba: str | None = None
    diameter_mm: float | None = None
    weight_g: int | None = None
    length_m: int | None = None
    hotend_min_c: int | None = None
    hotend_max_c: int | None = None
    bed_min_c: int | None = None
    bed_max_c: int | None = None
    dry_temp_c: int | None = None
    dry_hours: int | None = None
    sku: str | None = None
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        return asdict(self)

    def summary(self):
        parts = [p for p in (self.brand, self.material, self.name) if p]
        if self.color_rgba:
            parts.append("#" + self.color_rgba[:6])
        if self.weight_g:
            parts.append(f"{self.weight_g} g")
        return " ".join(parts) if parts else f"UID {self.uid}"


def ntag_steps(got, first_page, last_page):
    """The NTAG reads still missing between two pages, up to three groups of four per step."""
    steps = []
    page = first_page
    while page <= last_page:
        span = [p for p in range(page, page + PAGES_PER_READ * READS_PER_STEP) if p <= last_page]
        missing = [p for p in span if p not in got.pages]
        if missing:
            groups = min(READS_PER_STEP, (last_page - page) // PAGES_PER_READ + 1)
            steps.append(Step(OP_NTAG, page, groups))
        page += PAGES_PER_READ * READS_PER_STEP
    return steps


def cstr(buf, off, n):
    return bytes(buf[off : off + n]).split(b"\0", 1)[0].decode("utf-8", "replace").strip()


class TagFormat:
    """A brand: name (the section's), chip ("ntag" or "mifare"), needs (the parameters its
    section must carry).  MIFARE brands list their sectors {sector: blocks mask} in `sectors`
    and derive a key per sector in key_for(); steps() is then shared."""

    name = ""
    chip = ""
    needs = ()
    sectors = {}

    def __init__(self, params):
        self.params = dict(params)

    def claims(self, chip):
        return chip.is_ntag if self.chip == "ntag" else chip.is_mifare_1k

    def key_for(self, uid, sector):
        raise NotImplementedError

    def steps(self, chip, got):
        for sector, mask in sorted(self.sectors.items()):
            if (sector, KEY_A) in got.auth_failed:
                return []  # a key of this brand did not open the tag: not this brand's
            wanted = [sector * BLOCKS_PER_SECTOR + i for i in range(DATA_BLOCKS) if mask >> i & 1]
            if any(b not in got.blocks for b in wanted):
                return [Step(OP_MIFARE_A, sector, mask, self.key_for(chip.uid, sector))]
        return []

    def decode(self, chip, got):
        raise NotImplementedError

    def _all_read(self, got):
        return all(
            sector * BLOCKS_PER_SECTOR + i in got.blocks
            for sector, mask in self.sectors.items()
            for i in range(DATA_BLOCKS)
            if mask >> i & 1
        )
