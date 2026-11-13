"""Reads the spool tags on the host, for the ace2k firmware.

The unit finds a tag in front of the antenna of a lane and reports it (``ace2k_rfid_tag``). This
module runs the read session.

1. It offers the chip to the brand modules (``ace2k_tags``) that may carry it.
2. It sends the steps of a candidate brand (``ace2k_rfid_step``). A step reads pages, or a sector
   with its key.
3. It collects the bytes (``ace2k_rfid_data``).
4. It ends the session (``ace2k_rfid_done``) as soon as a brand decodes a record. If no brand
   does, it ends the session with the UID alone.

A session is event-driven on the reactor. One timer gives up a step that gets no frame within
STEP_TIMEOUT_S. The unit ends a session 500 ms after the last step of the host.

If the unit ends a session itself because the tag is gone, or the host gives it up, the lane takes
the cached record of the UID, if there is one.

Every record becomes a log line and the Klipper event ``ace2k:tag_read`` (lane, record). The
records are kept by UID in ``ace2k_tags_cache.json`` beside printer.cfg. A named unit uses
``ace2k_tags_cache_<name>.json``. The first declared unit reads ``ace2k_tags_cache.json`` while it
has no file of its own.

The last read of each lane is kept beside the cache, in ``<cache>.lanes.json``
(``ace2k_rfid_kept.py``). A restart that resets the unit leaves a full bay ``pending``. Such a
lane takes its kept read back, marked ``restored``, and its status shows it ``read`` until the
next read replaces it. The lane is set by hand on the unit (``set_by_hand``): its strand never
left, so the unit gives it no tag, and a new tag on the neighbour's spool is the neighbour's. A
forget clears that mark, so the lane can be read again. The kept read goes when the bay is seen
empty, on a forget, on a read with motion, when a search ends with no tag, and when the unit reads
another tag on the lane.

The field byte of the state report names the field of each reader, a field fault and a dead reader
(``reader_flags()``, and ``rfid_readers`` in the status of ``[ace2k]``).

The G-codes are ``ACEK_RFID_READ LANE=n [MOVE=1] [DUMP=1] [WAIT=0]`` and
``ACEK_RFID_FORGET LANE=n``. A read on command ends at the next state event of the lane. If
nothing decided the state, the event carries the unchanged state. The unit refuses a read with
motion as busy while its own tie-break runs, and sends the tie-break back: the G-code sends the
read again every 0.25 s for up to 30 s (as the feed's G-codes send a start), and then reports the
refusal.

``ACEK_RFID_BY_HAND`` and ``ACEK_RFID_TUNE``, with ``set_in_use`` and ``set_by_hand``, belong to
the tag attribution (``ace2k_rfid_attrib.py``).

The brands and their parameters come from the ``[ace2k_tag]`` sections
(``config/ace2k_tags.cfg``). The module [ace2k] loads this one. If the firmware is built without
``CONFIG_ACE2K_RFID_READ``, this is logged and every G-code here answers with an error.

    [ace2k]
    #rfid_search_mm: 750          # how far the load searches for a tag, 100-1000 mm of filament
    #                             # (unset: the default of the firmware)
    #rfid_mifare_order: bambu, snapmaker, creality
"""

from __future__ import annotations

import json
import logging
import os
import random
import time
from typing import TYPE_CHECKING, Callable, Protocol, cast

try:
    from . import ace2k_tags  # noqa: F401 — the package, for Klipper's loader
    from .ace2k_feed_motion import (
        SESSION_ALLOWANCE_MM,
        FollowPause,
    )
    from .ace2k_feed_proto import (
        FAILED_KIND_NAMES,
        NOTICE_KIND_NAMES,
        SEARCH_FMT,
        firmware_search_mm,
        half_mm,
    )
    from .ace2k_link import (
        LANES,
        NoAnswerError,
        ask,
        check_lane,
        constant_or_none,
        probe_format,
        refuse_format,
        subscribe_response,
    )
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_feed_motion import (  # type: ignore[import-not-found, no-redef]
        SESSION_ALLOWANCE_MM,
        FollowPause,
    )
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]
        FAILED_KIND_NAMES,
        NOTICE_KIND_NAMES,
        SEARCH_FMT,
        firmware_search_mm,
        half_mm,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANES,
        NoAnswerError,
        ask,
        check_lane,
        constant_or_none,
        probe_format,
        refuse_format,
        subscribe_response,
    )

try:
    from .ace2k_rfid_attrib import TagAttribution
    from .ace2k_rfid_kept import KeptReads
    from .ace2k_tags import base, registry
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_rfid_attrib import TagAttribution  # type: ignore[import-not-found, no-redef]
    from ace2k_rfid_kept import KeptReads  # type: ignore[import-not-found, no-redef]
    from ace2k_tags import base, registry  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_feed import Ace2kFeed
    from .ace2k_feed_motion import Before, Event
    from .ace2k_klipper_types import (
        MCU,
        Command,
        Completion,
        Config,
        GCodeCommand,
        LaneStatus,
        Params,
        QueryCommand,
        Record,
    )
    from .ace2k_tags.base import Chip, SpoolRecord, Step, TagFormat
    from .ace2k_units import Units

    class UnitProto(Protocol):
        """The unit (``ace2k.Ace2k``) that the tag reader belongs to."""

        @property
        def mcu(self) -> MCU: ...
        @property
        def name(self) -> str | None: ...
        @property
        def feed(self) -> Ace2kFeed: ...
        @property
        def units(self) -> Units: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...
        def head(self, part: str | None = None) -> str: ...
        def lane_text(self, lane: int) -> str: ...
        def say(self, line: str) -> None: ...
        def add_key_cmd(self, text: str, has_key: bool) -> None: ...
        def register_command(
            self, name: str, func: Callable[[GCodeCommand], None], desc: str | None = None
        ) -> None: ...


# A read with motion pauses the follow of the lane until the unit's load that it runs ends.
READ_PAUSE = FollowPause("tag read", seq=0, mode="loading")
STATE_NAMES = {0: "unknown", 1: "pending", 2: "searching", 3: "reading", 4: "read", 5: "no_tag"}
# The event kinds 0-5 are the states of STATE_NAMES. Kind 6 is the one that is not a state.
KIND_UNKNOWN = 0  # the tag of the lane was forgotten
KIND_NO_TAG = 5
KIND_AMBIGUOUS = 6
# The reader that serves each lane. Reader B serves bays 1-2, and reader A serves bays 3-4.
READER_OF_LANE = ("b", "b", "a", "a")
# A read on command (MOVE=0) is answered by one state event: ambiguous, or the unchanged state of
# the lane. It attributes no tag (since v0.13.0: a tag goes only to a lane that moves).
# An event of any kind ends the wait.
# A read with motion (MOVE=1) waits for something else. It waits for the end of the unit's own
# load that carries its search, which is the terminal event of the feed (seq 0, mode loading). Then
# it asks the unit for the state of the lane. The tag events on the way only update the status of
# the lane. They are the pending state after a forget, and a read in the middle of the search.
# The field byte of the state report holds the field of each reader, a field fault, and a dead
# reader. A reader is dead after three failed configures in a row, until the next health run.
# Reader A serves lanes 3-4 and reader B serves lanes 1-2.
FIELD_BITS = (
    (0x01, "field_a"),
    (0x02, "field_b"),
    (0x04, "fault_a"),
    (0x08, "fault_b"),
    (0x10, "dead_a"),
    (0x20, "dead_b"),
)
REFUSALS = {1: "busy", 2: "no_filament", 3: "no_link", 4: "other_lane_moving", 5: "unsupported"}
# STATUS_ERROR documents the status enum of the firmware. No code compares against it (vulture's
# noqa on the line, as on the SpoolRecord fields)
STATUS_OK, STATUS_AUTH_FAILED, STATUS_TAG_GONE, STATUS_ERROR = 0, 1, 2, 3  # noqa: V107
# The read carries a seq since v0.13.0: Klipper sends a query again, with the same bytes, when its
# answer is lost, and the unit takes that copy of a read with motion as the same read, never as a
# second search. An older image has the read without it, and gets that one.
READ_FMT = "ace2k_rfid_read lane=%c move=%c seq=%c"
READ_FMT_UNKEYED = "ace2k_rfid_read lane=%c move=%c"
SEQ_MAX = 255  # a read's seq runs 1..255 per lane; 0 is no key
READ_RESP = "ace2k_rfid_read_response lane=%c accepted=%c reason=%c"
STEP_FMT = "ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s"
DONE_FMT = "ace2k_rfid_done lane=%c session=%c result=%c"
FORGET_FMT = "ace2k_rfid_forget lane=%c"
LANE_QUERY = "ace2k_rfid_lane_query lane=%c"
LANE_STATE = "ace2k_rfid_lane_state lane=%c state=%c uid=%*s"
STATE_FMT = "ace2k_rfid_state state=%*s field=%c"
TAG_FMT = "ace2k_rfid_tag lane=%c session=%c uid=%*s atqa=%hu sak=%c"
DATA_FMT = "ace2k_rfid_data lane=%c session=%c block=%c status=%c data=%*s"
EVENT_FMT = "ace2k_rfid_event lane=%c kind=%c session=%c"
REPORT_S = 1.0
READY_SEED_S = 0.1  # delay for the lane queries after klippy:ready (they block; handlers must not)
STEP_TIMEOUT_S = 0.5
PROBE_TIMEOUT_S = 5.0
# The wait of a MOVE=1 read without the feed module. It is the travel at the lowest speed of the
# lane (the constant of the firmware, else this value) plus the margin.
LANE_SPEED_FLOOR_MM_S = 9.0
SEARCH_MARGIN_S = 15.0
SEARCH_RANGE_MM = (100.0, 1000.0)
# The tag states in which the end of a read with motion waits for its outcome. The unit settles the
# state at its next look after the load's end, so the load's event can come first.
SEARCH_UNSETTLED = ("searching", "reading")
# The end of a read with motion, in plain words, for each settled tag state. A read state with a
# record names its format instead. A state this host does not know is named by its number.
SEARCH_OUTCOMES = {
    "no_tag": "no tag",
    "pending": "tag not read",
    "unknown": "bay emptied",
    "read": "tag read",
}
CACHE_FILE = "ace2k_tags_cache.json"
ADOPTED_SUFFIX = ".adopted"  # added to the old [ace2k] cache once a named unit has adopted it
PAGE_BYTES = 4


class _Session:
    def __init__(
        self, lane: int, sid: int, chip: Chip, candidates: list[TagFormat], dump: bool
    ) -> None:
        self.lane = lane
        self.sid = sid
        self.chip = chip
        self.candidates = candidates
        self.idx = 0
        self.got = base.Got()
        # The keys that every brand had refused, for the dump.
        self.auth_failed: set[tuple[int, int]] = set()
        self.step: Step | None = None
        self.expected = 0
        self.retried = False  # the current step was sent again once after an error
        self.deadline: float | None = None
        self.dump = dump


def _uid_record(chip: Chip) -> SpoolRecord:
    return base.SpoolRecord(format="uid", uid=chip.uid_text)


class ReadRefused(RuntimeError):  # noqa: N818 — named as the feed's FeedRefused
    """Raised when the unit refuses a tag read.

    ``reason`` is the name of the refusal, such as busy or no_filament.
    """

    def __init__(self, lane: int, reason: object) -> None:
        super().__init__(f"ace2k: lane {lane + 1} tag read refused: {reason}")
        self.reason = reason


class Ace2kRfid:
    # Built at connect, on an image with the reader (see _build_config). Absent before that.
    step_cmd: Command
    done_cmd: Command
    forget_cmd: Command
    read_query: QueryCommand
    lane_query: QueryCommand

    def __init__(self, config: Config, unit: UnitProto) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.unit = unit
        # The feed of the unit, built before this module. It provides the pause of the follow and
        # the budget for a move.
        self.feed = unit.feed
        self.mcu = unit.mcu  # the MCU of the unit, resolved once from `mcu:` in its section
        self.section = config.get_name()
        lo, hi = SEARCH_RANGE_MM
        self.search_mm = config.getfloat("rfid_search_mm", None, minval=lo, maxval=hi)
        order = config.get("rfid_mifare_order", ", ".join(registry.DEFAULT_MIFARE_ORDER))
        self.mifare_order = [n.strip() for n in order.split(",") if n.strip()]
        self.present = False
        self.formats: list[TagFormat] = []
        # The tag status of each lane, as get_status passes it to Klipper.
        self.lanes: list[LaneStatus] = [
            dict(state=None, uid=None, source=None, record=None) for _ in range(LANES)
        ]
        self.field: int | None = None
        self.sessions: dict[int, _Session] = {}
        # Maps a lane to the completion of a MOVE=0 read. Any state event ends it.
        self.waiters: dict[int, Completion] = {}
        # Maps a lane to the completion of a MOVE=1 read. The end of the load ends it.
        self.move_waiters: dict[int, Completion] = {}
        # Maps a lane to the motor travel (mm) of a read with motion that ended "loaded", until
        # the tag state of the lane settles and its line is logged (see _settle_search).
        self.search_ends: dict[int, float] = {}
        # The state reports received so far, counted on the serial thread, and per lane the count
        # when its last read with motion was accepted. A report counted by then was sampled before
        # the unit took the read, so its "unknown" does not end that search (see _handle_state).
        self.state_reports = 0
        self.search_from = [0] * LANES
        # The read's format (READ_FMT, or READ_FMT_UNKEYED on an older image, set at connect), and
        # the last seq of a read with motion on each lane. A Klipper RESTART keeps the unit, and its
        # key of the last read, so the count starts anywhere: a host that started from 1 every
        # time could send that key again.
        self.read_fmt = READ_FMT
        self.read_seq = [random.randrange(SEQ_MAX) for _ in range(LANES)]
        self.dump_next: set[int] = set()
        start = self.printer.get_start_args()
        self.config_dir = os.path.dirname(start.get("config_file", "") or ".")
        self.log_dir = os.path.dirname(start.get("log_file", "") or "") or self.config_dir
        name = CACHE_FILE
        # The cache that [ace2k] kept before its section had a name. The first declared unit
        # adopts it while it has no file of its own. It writes the cache to its own file at once,
        # and sets the old file aside as "<name>.adopted". So renaming [ace2k] to [ace2k a] keeps
        # the cached UIDs. The adoption happens only once. The file set aside is never read again.
        self.legacy_path: str | None = None
        if unit.name is not None:
            name = CACHE_FILE.replace(".json", f"_{unit.name}.json")
            if unit.units.names[0] == unit.name:
                self.legacy_path = os.path.join(self.config_dir, CACHE_FILE)
        self.cache_path = os.path.join(self.config_dir, name)
        self.cache = self._load_cache()
        # The last read of each lane, by lane index. It follows the lane's record (_set_record) and
        # is kept in a file, so a restart that resets the unit does not lose it (_restore).
        self.kept = KeptReads(
            KeptReads.path_for(self.cache_path),
            LANES,
            unit.head(),
            self.reactor.monotonic,
        )
        self.timer = self.reactor.register_timer(self._session_timeout)
        # The in-use mask, the lanes set by hand and the confirmation numbers (see its module).
        self.attrib = TagAttribution(config, unit)
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler("ace2k:feed_event", self._feed_event)
        for name in (
            "ACEK_RFID_READ",
            "ACEK_RFID_FORGET",
            "ACEK_RFID_BY_HAND",
            "ACEK_RFID_TUNE",
        ):
            unit.register_command(
                name, getattr(self, "cmd_" + name), getattr(self, "cmd_" + name + "_help")
            )

    # --- configuration -------------------------------------------------------------------

    def _build_config(self) -> None:
        verdict, detail = probe_format(self.mcu, READ_FMT)
        if verdict == "differs" and probe_format(self.mcu, READ_FMT_UNKEYED)[0] == "ok":
            # An image before the seq: a read with motion that Klipper sends again may search twice.
            self.read_fmt, verdict = READ_FMT_UNKEYED, "ok"
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, READ_FMT, (verdict, detail))
        if verdict == "absent":
            logging.warning(
                "%s firmware built without CONFIG_ACE2K_RFID_READ; the RFID G-codes are"
                " disabled (%s)",
                self.unit.head(),
                detail,
            )
            self.present = False
            return
        # Check here every other format that the module sends or waits for. A format from another
        # commit then refuses the connect, instead of raising later in a reactor callback.
        for fmt in (READ_RESP, STEP_FMT, DONE_FMT, FORGET_FMT, LANE_QUERY, LANE_STATE):
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(
                    self.printer, self.section, fmt, (verdict, detail), self.read_fmt
                )
        self.present = True
        try:
            self.attrib.build()
        except ValueError as e:
            raise self.printer.config_error(f"{self.section}: {e}") from None
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.mcu.add_config_cmd(f"ace2k_rfid_query rest_ticks={ticks}", is_init=True)
        if self.mcu.try_lookup_command(SEARCH_FMT) is not None:
            # An image that takes its keys as init commands gets, at every connect, the reach from
            # the key or the default of the unit. Any other image gets it only if the key is set,
            # as a config command (see Ace2k.keys_as_init).
            mm = self.search_mm
            if mm is None:
                mm = firmware_search_mm(self.mcu, None)  # not set: the unit keeps its own
            if mm is not None:
                um = int(round(mm * 1000.0))
                self.unit.add_key_cmd(
                    f"ace2k_rfid_search_set search_um={um}", self.search_mm is not None
                )
        elif self.search_mm is not None:
            logging.warning(
                "%s rfid_search_mm is set but the firmware has no load search"
                " (built without CONFIG_ACE2K_FEED); ignored",
                self.unit.head(),
            )
        for handler, fmt in (
            (self._handle_state, STATE_FMT),
            (self._handle_tag, TAG_FMT),
            (self._handle_data, DATA_FMT),
            (self._handle_event, EVENT_FMT),
        ):
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(
                    self.printer, self.section, fmt, (verdict, detail), self.read_fmt
                )
            subscribe_response(self.mcu, handler, fmt)
        # Built once. Every step, session end, read and forget reuses them.
        self.step_cmd = self.mcu.lookup_command(STEP_FMT)
        self.done_cmd = self.mcu.lookup_command(DONE_FMT)
        self.forget_cmd = self.mcu.lookup_command(FORGET_FMT)
        self.read_query = self.mcu.lookup_query_command(self.read_fmt, READ_RESP)
        self.lane_query = self.mcu.lookup_query_command(LANE_QUERY, LANE_STATE)

    def _on_connect(self) -> None:
        """Build the list of brands from the [ace2k_tag] sections that Klipper loaded."""
        tags = [obj for _name, obj in self.printer.lookup_objects("ace2k_tag")]
        if not tags:
            logging.warning(
                "%s no [ace2k_tag] section (include ace2k_tags.cfg); tags are read by UID only",
                self.unit.head(),
            )
        sections = {t.brand: dict(t.params, enabled=t.enabled) for t in tags}
        try:
            self.formats = registry.build(sections, self.mifare_order)
        except ValueError as e:
            raise self.printer.config_error(f"{self.section}: {e}") from None
        logging.info(
            "%s tag brands %s",
            self.unit.head(),
            ", ".join(f.name for f in self.formats) or "none",
        )

    def _on_ready(self) -> None:
        """Schedule the seeding of the lanes.

        Klipper forbids a blocking query (a reactor pause) inside a klippy:ready handler. So the
        lane queries run from a timer just after it.
        """
        if not self.present:
            return
        self.reactor.register_timer(self._seed_lanes, self.reactor.monotonic() + READY_SEED_S)

    def _seed_lanes(self, _eventtime: float) -> float:
        """Ask the unit for the tag state of each lane. Run once.

        A lane that was read takes the cached record of its UID. If there is none, the lane is
        forgotten, so that its next move reads the tag again.
        """
        for lane in range(LANES):
            try:
                params = ask(
                    self.printer, self.mcu, self.lane_query, [lane], f"lane {lane + 1} tag state"
                )
            except NoAnswerError as e:
                # A lost answer must not take the reactor down from this timer. The lane stays
                # unknown until its next report, and its cached record is not restored.
                logging.warning(
                    "%s lane %s tag state query failed: %s",
                    self.unit.head(),
                    self.unit.lane_text(lane),
                    e,
                )
                continue
            state = STATE_NAMES.get(params["state"], params["state"])
            self.lanes[lane]["state"] = state
            if state == "read":
                self._seed_read(lane, bytes(params["uid"]).hex().upper())
            else:
                self._restore(lane)
        self.kept.save()  # one write for the four lanes
        return self.reactor.NEVER

    def _seed_read(self, lane: int, uid: str) -> None:
        """Seed a lane that the unit holds read, after a restart that kept the unit running.

        It takes the cached record of its UID, else its kept read of that UID (a tag read by its
        UID alone is in no cache). With neither, the lane is forgotten, so that its next move reads
        the tag again, unless the unit holds it set by hand: that mark is the user's, and a forget
        would clear it. When the unit's mask of those lanes has not come yet, the forget waits for
        it.
        """
        cached = self.cache.get(uid)
        kept = self.kept.restorable(lane)
        if cached is not None:
            self._set_record(lane, uid, "cached", cached, save=False)
        elif kept is not None and kept.get("uid") == uid:
            self._take_kept(lane, kept)
        else:
            self.attrib.forget_unless_held(lane, lambda: self.forget_cmd.send([lane]))
            self._set_record(lane, None, None, None, save=False)

    def _restore(self, lane: int) -> None:
        """Give a pending lane with no record its kept read, after a restart.

        A unit reset by the restart has forgotten its tags, so a full bay is pending. The spool is
        taken to be the one read before. The read is marked ``restored``, and the next read of
        the lane replaces it. A spool swapped while the printer was off keeps the old read until
        then. The lane is set by hand on the unit: its strand never left, so it is defined.
        """
        kept = self.kept.restorable(lane)
        status = self.lanes[lane]
        if kept is None or status["record"] is not None or status["state"] != "pending":
            return
        if lane in self.sessions:
            return
        self._take_kept(lane, kept)
        # The unit holds the lane defined, as for a spool set by hand, so a tag on the
        # neighbour's spool is not taken for one that could be this lane's.
        self.attrib.set_by_hand(lane)

    def _take_kept(self, lane: int, kept: Record) -> None:
        """Give the lane its kept read, marked ``restored``, and announce it."""
        self.lanes[lane].update(uid=kept.get("uid"), source="restored", record=kept)
        self._announce(lane, kept)

    def _announce(self, lane: int, record: Record) -> None:
        """Send ace2k:tag_read for a read restored on the lane, as for a read."""
        self.printer.send_event("ace2k:tag_read", lane, dict(record, unit=self.unit.name))

    def _drop_kept(self, lane: int) -> None:
        """Drop the kept read of a lane, and the record it restored, if any."""
        if self.lanes[lane]["source"] == "restored":
            self.lanes[lane].update(uid=None, source=None, record=None)
        if self.kept.get(lane) is not None:
            self.kept.set(lane, None)

    # --- the cache and the dump -----------------------------------------------------------

    def _load_cache(self) -> dict[str, Record]:
        path = self.cache_path
        legacy = self.legacy_path
        adopt = False
        if legacy is not None and not os.path.exists(path):
            adopt = True
            path = legacy
        try:
            with open(path) as f:
                cache: dict[str, Record] = json.load(f)
        except (OSError, ValueError):
            return {}
        if adopt and self._save_cache(cache):
            # adopted once ever: written to the unit's own file, which every later start reads,
            # and the old file set aside (its data kept), so a later start without the unit's
            # file, or with the sections reordered, adopts nothing
            try:
                os.replace(path, path + ADOPTED_SUFFIX)
            except OSError as e:
                logging.warning("%s old tag cache %s not set aside: %s", self.unit.head(), path, e)
        return cache

    def _save_cache(self, cache: dict[str, Record] | None = None) -> bool:
        """The cache written to the unit's file; False when it could not be."""
        try:
            tmp = self.cache_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.cache if cache is None else cache, f, indent=1, sort_keys=True)
            os.replace(tmp, self.cache_path)
        except OSError as e:
            logging.warning("%s tag cache not written: %s", self.unit.head(), e)
            return False
        return True

    def _set_record(
        self,
        lane: int,
        uid: str | None,
        source: str | None,
        record: Record | None,
        save: bool = True,
    ) -> None:
        """Set the record of a lane, and keep it as the lane's last read (None drops it)."""
        self.lanes[lane].update(uid=uid, source=source, record=record)
        self.kept.set(lane, record, save)

    def _dump(self, s: _Session, fmt_name: str) -> None:
        path = os.path.join(
            self.log_dir, f"ace2k_tag_lane{s.lane + 1}_{time.strftime('%Y%m%d-%H%M%S')}.json"
        )
        body = {
            "uid": s.chip.uid_text,
            "atqa": s.chip.atqa,
            "sak": s.chip.sak,
            "decoded_as": fmt_name,
            "pages": {str(p): v.hex() for p, v in sorted(s.got.pages.items())},
            "blocks": {str(b): v.hex() for b, v in sorted(s.got.blocks.items())},
            "auth_failed": sorted(s.auth_failed | s.got.auth_failed),
        }
        try:
            with open(path, "w") as f:
                json.dump(body, f, indent=1)
            logging.info(
                "%s lane %s tag dump written to %s",
                self.unit.head(),
                self.unit.lane_text(s.lane),
                path,
            )
        except OSError as e:
            logging.warning("%s tag dump not written: %s", self.unit.head(), e)

    # --- reports and events (from the serial thread to the reactor) -----------------------------

    def _handle_state(self, params: Params) -> None:
        states = list(params["state"])
        field = params["field"]
        self.state_reports += 1
        number = self.state_reports

        def apply(_eventtime: float) -> None:
            for i, s in enumerate(states[:LANES]):
                self.lanes[i]["state"] = STATE_NAMES.get(s, s)
                if self.unit.lanes[i]["insert"] is False:
                    self._drop_kept(i)  # the bay reads empty: its spool is gone
                else:
                    self._restore(i)
                if s == KIND_UNKNOWN and number > self.search_from[i]:
                    # The bay is empty. The report covers a lost unknown event too. A report from
                    # before the unit took the lane's read with motion says nothing of its search.
                    self.feed.motion[i].tag_search = False
                self._settle_search(i)
            self.field = field

        self.reactor.register_async_callback(apply)

    def _handle_tag(self, params: Params) -> None:
        lane, sid = params["lane"], params["session"]
        chip = base.Chip(uid=bytes(params["uid"]), atqa=params["atqa"], sak=params["sak"])
        self.reactor.register_async_callback(lambda _e: self._session_start(lane, sid, chip))

    def _handle_data(self, params: Params) -> None:
        args = (params["lane"], params["session"], params["block"], params["status"])
        payload = bytes(params["data"])
        self.reactor.register_async_callback(lambda _e: self._session_data(*args, payload))

    def _handle_event(self, params: Params) -> None:
        lane, kind = params["lane"], params["kind"]
        self.reactor.register_async_callback(lambda _e: self._lane_event(lane, kind))

    def _lane_event(self, lane: int, kind: int) -> None:
        if kind != KIND_AMBIGUOUS:
            self.lanes[lane]["state"] = STATE_NAMES.get(kind, kind)
        if kind == KIND_UNKNOWN:
            self._set_record(lane, None, None, None)
            # The bay is empty, so no read with motion runs on the lane any more.
            self.feed.motion[lane].tag_search = False
        elif kind == KIND_NO_TAG and self.lanes[lane]["source"] in ("restored", None):
            # A search found no tag: the read from before goes. A record that the unit confirmed
            # by its UID in this insertion stays (the unit never ends no_tag after seeing one).
            self._drop_kept(lane)
        else:
            self._restore(lane)
        self._settle_search(lane)
        waiter = self.waiters.pop(lane, None)
        if waiter is not None:
            waiter.complete(kind)

    def _feed_event(self, lane: int, event: Event) -> None:
        """Handle the Klipper event of ``ace2k_feed.py``, on the reactor.

        It looks for the end of the unit's own load on a lane where a MOVE=1 read waits. Then the
        search is over, and the filament is back where it started.
        """
        if event.get("unit") != self.unit.name:
            return  # a lane of another unit
        if event["seq"] != 0 or event["mode"] != "loading" or event["kind"] in NOTICE_KIND_NAMES:
            return
        motion = self.feed.motion[lane]
        if motion.tag_search:
            motion.tag_search = False
            if event["kind"] == "loaded":
                self.search_ends[lane] = event["motor_mm"]
                self._settle_search(lane)
        waiter = self.move_waiters.pop(lane, None)
        if waiter is not None:
            waiter.complete(event)

    def _settle_search(self, lane: int) -> None:
        """Log the end of a read with motion on ``lane`` once its tag state has settled.

        The line gives the distance out, which is half the motor travel of the search, and the tag
        outcome. It replaces the "loaded" line of the feed, whose filament figure is the net of
        the way out and the way back, about 0. Nothing is logged while the state is still
        searching or reading, or when no search has ended on the lane.
        """
        motor_mm = self.search_ends.get(lane)
        state = self.lanes[lane]["state"]
        if motor_mm is None or state in SEARCH_UNSETTLED:
            return
        del self.search_ends[lane]
        record = self.lanes[lane]["record"]
        if state == "read" and record:
            outcome = f"tag {record['format']}"
        else:
            outcome = SEARCH_OUTCOMES.get(state, f"tag state {state}")
        logging.info(
            "%s lane %s tag search: about %.1f mm out and back, %s",
            self.unit.head(),
            self.unit.lane_text(lane),
            half_mm(motor_mm),
            outcome,
        )

    # --- the session ------------------------------------------------------------------------

    def _session_start(self, lane: int, sid: int, chip: Chip) -> None:
        candidates = [f for f in self.formats if f.claims(chip)]
        dump = lane in self.dump_next
        self.dump_next.discard(lane)
        s = _Session(lane, sid, chip, candidates, dump)
        restored = self.lanes[lane]["source"] == "restored"
        if restored and self.lanes[lane]["uid"] != chip.uid_text:
            self._drop_kept(lane)  # another tag: the restored read was of another spool
        self.sessions[lane] = s
        self.lanes[lane]["state"] = "reading"
        self._advance(s)

    def _send_step(self, s: _Session, step: Step, retry: bool = False) -> None:
        s.step = step
        s.retried = retry
        s.expected = step.blocks if step.op == base.OP_NTAG else bin(step.blocks).count("1")
        self.step_cmd.send([s.lane, s.sid, step.op, step.arg, step.blocks, step.key])
        s.deadline = self.reactor.monotonic() + STEP_TIMEOUT_S
        self.reactor.update_timer(self.timer, self._next_deadline())

    def _advance(self, s: _Session) -> None:
        while s.idx < len(s.candidates):
            fmt = s.candidates[s.idx]
            steps = fmt.steps(s.chip, s.got)
            if steps:
                self._send_step(s, steps[0])
                return
            record = fmt.decode(s.chip, s.got)
            if record is not None:
                self._finish(s, record, "tag", fmt.name)
                return
            # The bytes read stay, because they belong to the tag whichever key opened them. A
            # refused key belonged to this brand, so the next brand starts with none marked.
            s.auth_failed |= s.got.auth_failed
            s.got.auth_failed = set()
            s.idx += 1
        if s.chip.uid_text in self.cache:
            # The keys of a brand decoded this UID before, so every brand that declines now is
            # noise. An example is a key refused at the edge of the reach of the antenna. The UID
            # alone must not replace the record in the cache.
            # The session is given up, like NTAG pages that refuse twice. The lane keeps the cached
            # record unlatched, and the next move reads the tag again. So a tag that was rewritten
            # under the same UID is never latched with the old record.
            self._give_up(s)
            return
        self._finish(s, _uid_record(s.chip), "uid", "uid")

    def _store(self, s: _Session, block: int, payload: bytes) -> None:
        assert s.step is not None  # the frame belongs to the step in flight (see _session_data)
        if s.step.op == base.OP_NTAG:
            for i in range(len(payload) // PAGE_BYTES):
                s.got.pages[block + i] = payload[i * PAGE_BYTES : (i + 1) * PAGE_BYTES]
        else:
            s.got.blocks[block] = payload

    def _session_data(self, lane: int, sid: int, block: int, status: int, payload: bytes) -> None:
        s = self.sessions.get(lane)
        if s is None or s.sid != sid or s.step is None:
            return  # a frame of another session, or one that came after the end
        if status == STATUS_TAG_GONE:
            self._end(s)  # the unit has ended the session itself
            self._fallback(s)
            return
        if status == STATUS_AUTH_FAILED:
            # Only a refused key marks (sector, key) as failed. Then the other keys of the brand,
            # or the next brand, are tried.
            key_type = base.KEY_B if s.step.op == base.OP_MIFARE_B else base.KEY_A
            s.got.auth_failed.add((s.step.arg, key_type))
            s.step = None
            self._advance(s)
            return
        if status != STATUS_OK:
            # An error (a CRC, a NAK, no answer), on NTAG or MIFARE, is transient until it repeats.
            # So send the same step once more, and then give up. A brand that is right is never
            # declined because of noise.
            if not s.retried:
                self._send_step(s, s.step, retry=True)
            elif s.step.op == base.OP_NTAG and s.chip.uid_text not in self.cache:
                # Pages that refuse twice mean a smaller chip or a password. Finish with the UID
                # alone, unless the cache knows the UID. Noise must not replace that record (see
                # below).
                self._finish(s, _uid_record(s.chip), "uid", "uid")
            else:
                self._give_up(s)
            return
        self._store(s, block, payload)
        s.expected -= 1
        if s.expected <= 0:
            s.step = None
            self._advance(s)

    def _end(self, s: _Session) -> None:
        self.sessions.pop(s.lane, None)
        self.reactor.update_timer(self.timer, self._next_deadline())

    def _fallback(self, s: _Session) -> None:
        """Use the cached record of the UID, if there is one, when the UID is all there is."""
        uid = s.chip.uid_text
        cached = self.cache.get(uid)
        status = self.lanes[s.lane]
        if cached is not None:
            self._set_record(s.lane, uid, "cached", cached)
        elif status["source"] == "restored" and status["uid"] == uid:
            # The unit saw the restored read's UID: the read is this spool's, now a live one.
            self._set_record(s.lane, uid, "cached", status["record"])

    def _finish(self, s: _Session, record: SpoolRecord, source: str, fmt_name: str) -> None:
        self.done_cmd.send([s.lane, s.sid, 1])
        self._end(s)
        rec = record.as_dict()
        self._set_record(s.lane, record.uid, source, rec, save=False)
        if source == "tag":
            self.cache[record.uid] = rec
            # Write the file here and now, on the reactor. The file is small and a tag is read
            # rarely, so the block is brief. Nothing is lost if Klipper stops right after the read.
            self._save_cache()
        self.kept.save()  # with the cache, the lane's read
        if s.dump:
            self._dump(s, fmt_name)
        logging.info(
            "%s lane %s tag %s — %s",
            self.unit.head(),
            self.unit.lane_text(s.lane),
            record.uid,
            record.summary(),
        )
        self.printer.send_event("ace2k:tag_read", s.lane, dict(rec, unit=self.unit.name))

    def _next_deadline(self) -> float:
        deadlines = [s.deadline for s in self.sessions.values() if s.deadline is not None]
        return min(deadlines) if deadlines else self.reactor.NEVER

    def _give_up(self, s: _Session) -> None:
        """End the session from the host without a read, after a step timed out or failed twice.

        The lane takes the cached record of the UID, if there is one.
        """
        self.done_cmd.send([s.lane, s.sid, 0])
        if s.dump:
            self._dump(s, "none")
        self._end(s)
        self._fallback(s)

    def _session_timeout(self, eventtime: float) -> float:
        for s in list(self.sessions.values()):
            if s.deadline is not None and s.deadline <= eventtime:
                self._give_up(s)
        return self._next_deadline()

    # --- status -----------------------------------------------------------------------------

    def lane_status(self, index: int) -> dict[str, object]:
        """Return the tag status of a lane. A restored read shows ``read`` while the unit's lane
        is ``pending``: it speaks for the lane until the next read."""
        status: dict[str, object] = dict(self.lanes[index])
        if status["source"] == "restored" and status["state"] == "pending":
            status["state"] = "read"
        return status

    def reader_flags(self) -> list[str] | None:
        """Return the names of the flags in the field byte of the last state report.

        Return None before the first report.
        """
        if self.field is None:
            return None
        return [name for bit, name in FIELD_BITS if self.field & bit]

    def status_lines(self) -> list[str]:
        lines: list[str] = []
        for i in range(LANES):
            lane = self.lane_status(i)
            rec = cast("Record | None", lane["record"])
            what = "" if rec is None else " — " + base.SpoolRecord(**rec).summary()
            source = "" if lane["source"] in (None, "tag") else f" ({lane['source']})"
            lines.append(f"lane {i + 1}: tag {lane['state'] or 'unknown'}{what}{source}")
        flags = self.reader_flags()
        readers = "no report yet" if flags is None else (", ".join(flags) or "idle")
        lines.append(f"rfid readers: {readers}")
        return lines

    # --- the G-codes ------------------------------------------------------------------------

    def _require(self, gcmd: GCodeCommand) -> None:
        if not self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_RFID_READ")

    def _lane(self, gcmd: GCodeCommand) -> int:
        return gcmd.get_int("LANE", minval=1, maxval=LANES) - 1

    def _search_mm(self) -> float:
        """Return the reach of the search, in mm, that a MOVE=1 read waits for.

        It is rfid_search_mm if set. Otherwise it is the default of the firmware
        (ACE2K_FEED_SEARCH_DEFAULT_UM). An image that does not declare it gets 750 mm.
        """
        if self.search_mm is not None:
            return self.search_mm
        reach: float = firmware_search_mm(self.mcu)
        return reach

    def _move_timeout(self, search_mm: float) -> float:
        """Return the wait, in seconds, of a MOVE=1 read.

        The filament travels out to the reach of the search and back, plus the pauses of the read
        sessions. The speed is the one at which the unit searches. That is the current load speed
        of the feed (load_speed, or the last ACEK_LOAD_SET). The feed computes the wait with its
        own budget for a waited move. Without the feed module, the lowest speed of the lane is
        used.
        """
        travel: float = 2 * search_mm + SESSION_ALLOWANCE_MM
        feed = self.feed
        if feed is not None and feed.present:
            return feed.load_wait_s(travel)
        floor_um_s: float | None = constant_or_none(self.mcu, "ACE2K_LANE_SPEED_MIN_UM_S")
        floor = LANE_SPEED_FLOOR_MM_S if floor_um_s is None else floor_um_s / 1000.0
        return travel / floor + SEARCH_MARGIN_S

    cmd_ACEK_RFID_READ_help = (
        "Read the tag of LANE=<1-4>: MOVE=0 (default) only reports what is in the field, and a"
        " tag is attributed only with MOVE=1, which turns the"
        " spool up to rfid_search_mm and back (a lane already read is read again); DUMP=1 saves"
        " the bytes; WAIT=0 returns at once"
    )

    def cmd_ACEK_RFID_READ(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        lane = self._lane(gcmd)
        move = gcmd.get_int("MOVE", 0, minval=0, maxval=1)
        if gcmd.get_int("DUMP", 0, minval=0, maxval=1):
            self.dump_next.add(lane)
        wait = gcmd.get_int("WAIT", 1, minval=0, maxval=1)
        feed = self.feed if self.feed.present else None
        waiters = self.move_waiters if move else self.waiters
        completion = self.reactor.completion() if wait else None

        def send(_before: Before | None) -> None:
            if completion is not None and not completion.test():
                # Register before the command, and again before a retry unless an answer came
                # meanwhile. A dead reader answers with the unchanged state of the lane, and
                # that answer is sent ahead of the response to the command.
                waiters[lane] = completion
            self._send_read(lane, move)

        try:
            if move and feed is not None:
                # A read with motion on a lane in the follow pauses the follow. The follow comes
                # back at the clean end of the read, which is the end of the unit's load that it
                # runs (seq 0, loading).
                feed.with_follow_paused(gcmd, lane, READ_PAUSE, send)
            else:
                send(None)
        except RuntimeError as e:  # no answer (NoAnswerError), or a refusal by the unit
            waiters.pop(lane, None)
            self.dump_next.discard(lane)
            raise gcmd.error(str(e)) from None
        if move:
            # On accepting, the unit forgot the tag of a lane in state read or no_tag.
            # So does the host.
            self._set_record(lane, None, None, None)
        if not wait:
            gcmd.respond_info(f"ace2k: lane {lane + 1} tag read started")
            return
        timeout = PROBE_TIMEOUT_S if not move else self._move_timeout(self._search_mm())
        assert completion is not None  # waited: WAIT=1
        result = completion.wait(self.reactor.monotonic() + timeout, None)
        if lane not in self.sessions:
            # No session took the request (a dead reader, no tag, or the timeout). A later automatic
            # read must not write a dump that nobody asked for.
            self.dump_next.discard(lane)
        if result is None:
            waiters.pop(lane, None)
            raise gcmd.error(f"ace2k: lane {lane + 1}: no tag outcome within {timeout:.0f} s")
        if move:
            self._report_move(gcmd, lane, result)
            return
        if result == KIND_AMBIGUOUS:
            gcmd.respond_info(
                f"ace2k: lane {lane + 1}: ambiguous — a tag is read only while its lane moves;"
                " use MOVE=1"
            )
            return
        gcmd.respond_info(self._outcome_line(lane, STATE_NAMES.get(result, result)))

    def start_read(self, lane_index: int) -> None:
        """Read the tag of the lane in front of the antenna, without motion and without waiting.

        This is part of the host interface (API_VERSION 7). It does what
        ``ACEK_RFID_READ LANE=n MOVE=0 WAIT=0`` does, for `lane_index` 0 to 3. It asks the unit
        to report what is in the field of the lane's antenna. A timer may call it. Since v0.13.0
        the unit gives no tag to a read without motion, so the tag status does not change. The
        answer is ``ambiguous`` or the unchanged state.

        It raises ValueError for a lane outside 0 to 3, and sends nothing then. It raises
        RuntimeError if the firmware has no tag reader, or if the unit refuses (the message names
        the reason). It raises NoAnswerError, a kind of RuntimeError, if the unit does not answer.
        """
        check_lane(lane_index, f"ace2k: lane index %s outside 0..{LANES - 1}")
        if not self.present:
            raise RuntimeError("ace2k: firmware built without CONFIG_ACE2K_RFID_READ")
        self._send_read(lane_index, 0)

    def _send_read(self, lane: int, move: int) -> Params:
        """Send the read request, for the G-code and for start_read alike.

        Return the answer of the unit that accepts it. Raise NoAnswerError (a RuntimeError) if
        there is no answer. Raise RuntimeError that names the reason if the unit refuses.
        """
        if move:
            # A read with motion is a start on the lane, so the default follow leaves it alone.
            self.feed.follow_auto.note_start(lane)
        motion = self.feed.motion[lane]
        # A refusal or no answer puts the flag back as it was. A search that already runs on the
        # lane keeps it, so its end still gets this module's line.
        was_search = motion.tag_search
        if move:
            # Its load ends with a line of this module (see _settle_search). The flag is set
            # before the request, so it is in place before the load can end.
            motion.tag_search = True
        data = [lane, move]
        if self.read_fmt == READ_FMT:
            # A new seq for each read with motion; a read without motion carries none.
            seq = 0
            if move:
                seq = self.read_seq[lane] = self.read_seq[lane] % SEQ_MAX + 1
            data.append(seq)
        try:
            params: Params = ask(
                self.printer, self.mcu, self.read_query, data, f"lane {lane + 1} tag read"
            )
        except NoAnswerError:
            motion.tag_search = was_search
            raise
        if not params["accepted"]:
            motion.tag_search = was_search
            reason = REFUSALS.get(params["reason"], params["reason"])
            raise ReadRefused(lane, reason)
        if move:
            # A state report sampled before the unit took the read may show the bay unknown. Its
            # callback may have run during the round trip, so the flag is set again. One that runs
            # later was counted before the answer came, and search_from makes it pass by.
            motion.tag_search = True
            self.search_from[lane] = self.state_reports
        return params

    def _outcome_line(self, lane: int, state: object) -> str:
        rec = self.lanes[lane]["record"]
        what = base.SpoolRecord(**rec).summary() if rec else "no record"
        reader = READER_OF_LANE[lane]
        flags = self.reader_flags() or []
        dead = f" (reader {reader.upper()} dead)" if f"dead_{reader}" in flags else ""
        return f"ace2k: lane {lane + 1} tag {state} — {what}{dead}"

    def _report_move(self, gcmd: GCodeCommand, lane: int, event: Event) -> None:
        """Report the end of a MOVE=1 read.

        Ask the unit for the state of the lane as it holds it now, because the 1 Hz report may lag.
        If the load did not end as loaded, say how it ended. An error kind is reported as an error.
        """
        try:
            params = ask(
                self.printer, self.mcu, self.lane_query, [lane], f"lane {lane + 1} tag state"
            )
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from None
        state = STATE_NAMES.get(params["state"], params["state"])
        self.lanes[lane]["state"] = state
        self._settle_search(lane)
        line = self._outcome_line(lane, state)
        kind = event["kind"]
        if kind == "loaded":
            gcmd.respond_info(line)
            return
        if kind in FAILED_KIND_NAMES:
            raise gcmd.error(f"ace2k: lane {lane + 1} tag search failed: {kind}; {line}")
        gcmd.respond_info(f"ace2k: lane {lane + 1} tag search {kind}; {line}")

    cmd_ACEK_RFID_FORGET_help = (
        "Forget the tag of LANE=<1-4>: the next move reads it again, and the other lane may take it"
    )

    def cmd_ACEK_RFID_FORGET(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        lane = self._lane(gcmd)
        self.forget_cmd.send([lane])
        self.attrib.forget_by_hand(lane)  # the unit clears its mark with the forget
        self._set_record(lane, None, None, None)
        gcmd.respond_info(f"ace2k: lane {lane + 1} tag forgotten")

    def set_in_use(self, mask: int) -> None:
        """Tell the unit which lanes a print uses (4-bit mask). It does nothing on an old image."""
        self.attrib.set_in_use(mask)

    def set_by_hand(self, lane_index: int) -> None:
        """Tell the unit that the tag of a lane (0-3) was set by hand."""
        self.attrib.set_by_hand(lane_index)

    cmd_ACEK_RFID_BY_HAND_help = (
        "Say that the tag of LANE=<1-4> was set by hand: the antenna never changes it"
    )

    def cmd_ACEK_RFID_BY_HAND(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        lane = self._lane(gcmd)
        if not self.attrib.present:
            raise gcmd.error("ace2k: the firmware has no tag attribution")
        self.attrib.set_by_hand(lane)
        gcmd.respond_info(f"ace2k: lane {lane + 1} tag set by hand")

    cmd_ACEK_RFID_TUNE_help = (
        "Set how a tag is confirmed: LEG=<mm> SPEED=<mm/s> MIN=<mm> FOLLOWS=<1-3> WINDOW=<mm>"
        " MAX=<mm>;"
        " reports the values in force"
    )

    def cmd_ACEK_RFID_TUNE(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        gcmd.respond_info(self.attrib.tune(gcmd))
