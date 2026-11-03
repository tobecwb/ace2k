"""Keeps the last tag read of each lane in a file, so that it outlives a restart.

``ace2k_rfid.py`` owns one KeptReads per unit. The file sits beside the tag cache, as
``<cache>.lanes.json``: the lane number (1-4) to the spool record of its last read.

After each ``save()``, what is in memory matches the file. A change that cannot be written is
undone in memory, logged once, and not tried again for RETRY_S. So a disk that refuses writes
costs one attempt a minute, not one a second. A change made inside the retry window is dropped
without a write.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import os
from typing import TYPE_CHECKING, Callable

try:
    from .ace2k_tags import base
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_tags import base  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_klipper_types import Record

SUFFIX = ".lanes.json"  # the file is "<cache without .json>.lanes.json"
RETRY_S = 60.0  # how long a write that failed waits before the next attempt
# The types a field of a spool record may hold, by the name of its annotation. None is always
# allowed but for format and uid. An int is a float too; a bool is neither.
FIELD_TYPES: dict[str, tuple[type, ...]] = {
    "str": (str,),
    "int": (int,),
    "float": (int, float),
    "Extra": (dict,),
}
REQUIRED = ("format", "uid")


def _field_types() -> dict[str, tuple[type, ...]]:
    """The allowed types of each field of SpoolRecord."""
    types: dict[str, tuple[type, ...]] = {}
    for f in dataclasses.fields(base.SpoolRecord):
        name = str(f.type).split("|")[0].strip()
        types[f.name] = FIELD_TYPES.get(name, (object,))
    return types


RECORD_TYPES = _field_types()


def valid_record(record: object) -> bool:
    """True when record has exactly the fields of a spool record, each of its type."""
    if not isinstance(record, dict) or set(record) != set(RECORD_TYPES):
        return False
    for name, types in RECORD_TYPES.items():
        value = record[name]
        if value is None and name not in REQUIRED:
            continue
        if isinstance(value, bool) or not isinstance(value, types):
            return False
    return True


class KeptReads:
    """The last read of each lane, by lane index (0-3), in memory and in its file.

    path is the file. head names the unit in the log. now is the reactor's clock.
    """

    def __init__(self, path: str, lanes: int, head: str, now: Callable[[], float]) -> None:
        self.path = path
        self.head = head
        self.now = now
        self.reads = self._load(lanes)
        self.saved = dict(self.reads)  # what the file holds
        # The lanes whose read was kept from before this start, until anything changes them.
        self.from_before = set(self.reads)
        self.retry_at = 0.0
        self.failed = False  # a write failed and was logged; logged again only after a success

    @staticmethod
    def path_for(cache_path: str) -> str:
        """The file of the kept reads beside the tag cache at cache_path."""
        return cache_path[: -len(".json")] + SUFFIX

    def _load(self, lanes: int) -> dict[int, Record]:
        """Read the file. A missing or unreadable file keeps nothing. A bad entry is left out."""
        try:
            with open(self.path) as f:
                data = json.load(f)
            items = list(data.items())
        except (OSError, ValueError, AttributeError):
            return {}
        reads: dict[int, Record] = {}
        for number, record in items:
            try:
                lane = int(number) - 1
            except ValueError:
                continue
            if 0 <= lane < lanes and valid_record(record):
                reads[lane] = record
        dropped = len(items) - len(reads)
        if dropped:
            logging.warning("%s %d bad lane reads left out of %s", self.head, dropped, self.path)
        return reads

    def get(self, lane: int) -> Record | None:
        return self.reads.get(lane)

    def restorable(self, lane: int) -> Record | None:
        """The read kept for the lane from before this start, if nothing changed it since."""
        return self.reads.get(lane) if lane in self.from_before else None

    def set(self, lane: int, record: Record | None, save: bool = True) -> None:
        """Keep record as the lane's last read (None drops it), and save unless told not to."""
        self.from_before.discard(lane)
        if record is None:
            self.reads.pop(lane, None)
        else:
            self.reads[lane] = record
        if save:
            self.save()

    def save(self) -> bool:
        """Write the reads if they changed. On a failure, memory goes back to the file's."""
        if self.reads == self.saved:
            return True
        if self.now() < self.retry_at:
            self.reads = dict(self.saved)
            return False
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({str(n + 1): r for n, r in sorted(self.reads.items())}, f, indent=1)
            os.replace(tmp, self.path)
        except OSError as e:
            self.reads = dict(self.saved)
            self.retry_at = self.now() + RETRY_S
            if not self.failed:
                logging.warning("%s kept lane reads not written: %s", self.head, e)
            self.failed = True
            return False
        self.saved = dict(self.reads)
        self.failed = False
        return True
