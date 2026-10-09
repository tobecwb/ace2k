"""ace2k_rfid — the spool tags of the ace2k firmware on the host.

The unit finds a tag in front of a lane's antenna and reports it (``ace2k_rfid_tag``); this module
runs the read session: it offers the chip to the brand modules (``ace2k_tags``) that may carry it,
sends a candidate's steps (``ace2k_rfid_step``: pages, or a sector and its key), collects the
bytes (``ace2k_rfid_data``), and ends the session (``ace2k_rfid_done``) as soon as a brand
decodes a record, or with the UID alone when none does.  A session is event-driven on the
reactor; one timer gives up a step with no frame within STEP_TIMEOUT_S (the unit ends a session
500 ms after the host's last step).  A session the unit ends itself — its tag gone — or one given
up takes the cached record of its UID, if any.  Every record is a log line and the Klipper event
``ace2k:tag_read`` (lane, record), and is kept by UID in ``ace2k_tags_cache.json`` beside
printer.cfg.  The state report's field byte names each reader's field, a field fault and a dead
reader (``reader_flags()``; ``rfid_readers`` in ``[ace2k]``'s status).  G-codes:
``ACE_RFID_READ LANE=n [MOVE=1] [DUMP=1] [WAIT=0]`` — a read on command ends on the lane's next
state event, its unchanged state when nothing decided it — and ``ACE_RFID_FORGET LANE=n``.
The brands and their parameters come from the ``[ace2k_tag]`` sections
(``config/ace2k_tags.cfg``).  Loaded by ``[ace2k]``; a firmware built without
``CONFIG_ACE2K_RFID_READ`` is logged and every G-code here answers an error.

    [ace2k]
    #rfid_search_mm: 750          # the load's search for a tag, 100–1000 mm of filament (unset: the
    #                             # firmware's default)
    #rfid_mifare_order: bambu, snapmaker, creality
"""

import json
import logging
import os
import time

try:
    from . import ace2k_tags  # noqa: F401 — the package, for Klipper's loader
    from .ace2k import probe_format, refuse_format, subscribe_response
    from .ace2k_feed import (
        ERROR_KINDS,
        KIND_NAMES,
        NOTICE_KINDS,
        SESSION_ALLOWANCE_MM,
        firmware_search_mm,
    )
    from .ace2k_tags import base, registry
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k import probe_format, refuse_format, subscribe_response
    from ace2k_feed import (
        ERROR_KINDS,
        KIND_NAMES,
        NOTICE_KINDS,
        SESSION_ALLOWANCE_MM,
        firmware_search_mm,
    )
    from ace2k_tags import base, registry

LANES = 4
STATE_NAMES = {0: "unknown", 1: "pending", 2: "searching", 3: "reading", 4: "read", 5: "no_tag"}
KIND_AMBIGUOUS = 6
# A read on command (MOVE=0) is answered by one state event: the outcome of its session, or the
# lane's unchanged state when no inventory decided it (2 s, or a dead reader) — any kind ends its
# wait.  A read with motion (MOVE=1) waits instead for the end of the unit's own load that
# carries its search — the feed's terminal event, seq 0, mode loading — and then asks the unit
# for the lane's state: the tag events on the way (the forget's pending, a read mid-search) only
# update the lane's status.
# The state report's field byte: each reader's field, a field fault, a dead reader (three
# failed configures in a row, until the next health run).  Reader A serves lanes 3–4, B 1–2.
FIELD_BITS = (
    (0x01, "field_a"),
    (0x02, "field_b"),
    (0x04, "fault_a"),
    (0x08, "fault_b"),
    (0x10, "dead_a"),
    (0x20, "dead_b"),
)
REFUSALS = {1: "busy", 2: "no_filament", 3: "no_link", 4: "other_lane_moving", 5: "unsupported"}
STATUS_OK, STATUS_AUTH_FAILED, STATUS_TAG_GONE, STATUS_ERROR = 0, 1, 2, 3
READ_FMT = "ace2k_rfid_read lane=%c move=%c"
READ_RESP = "ace2k_rfid_read_response lane=%c accepted=%c reason=%c"
STEP_FMT = "ace2k_rfid_step lane=%c session=%c op=%c arg=%c blocks=%c key=%*s"
DONE_FMT = "ace2k_rfid_done lane=%c session=%c result=%c"
FORGET_FMT = "ace2k_rfid_forget lane=%c"
LANE_QUERY = "ace2k_rfid_lane_query lane=%c"
LANE_STATE = "ace2k_rfid_lane_state lane=%c state=%c uid=%*s"
SEARCH_FMT = "ace2k_rfid_search_set search_um=%u"
STATE_FMT = "ace2k_rfid_state state=%*s field=%c"
TAG_FMT = "ace2k_rfid_tag lane=%c session=%c uid=%*s atqa=%hu sak=%c"
DATA_FMT = "ace2k_rfid_data lane=%c session=%c block=%c status=%c data=%*s"
EVENT_FMT = "ace2k_rfid_event lane=%c kind=%c session=%c"
REPORT_S = 1.0
READY_SEED_S = 0.1  # the lane queries, just after klippy:ready (they block; its handlers may not)
STEP_TIMEOUT_S = 0.5
PROBE_TIMEOUT_S = 5.0
# a MOVE=1 read's wait without the feed module: its travel at the lane's floor speed (the
# firmware's constant, else this) plus the margin
LANE_SPEED_FLOOR_MM_S = 9.0
SEARCH_MARGIN_S = 15.0
SEARCH_RANGE_MM = (100.0, 1000.0)
CACHE_FILE = "ace2k_tags_cache.json"
PAGE_BYTES = 4


class _Session:
    def __init__(self, lane, sid, chip, candidates, dump):
        self.lane = lane
        self.sid = sid
        self.chip = chip
        self.candidates = candidates
        self.idx = 0
        self.got = base.Got()
        self.auth_failed = set()  # every brand's refused keys, for the dump
        self.step = None
        self.expected = 0
        self.retried = False  # the current step was re-sent once after an error
        self.deadline = None
        self.dump = dump


def _uid_record(chip):
    return base.SpoolRecord(format="uid", uid=chip.uid.hex().upper())


class Ace2kRfid:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        mcu_name = config.get("mcu", "ace2k")
        self.mcu = self.printer.lookup_object("mcu" if mcu_name == "mcu" else "mcu " + mcu_name)
        self.section = config.get_name()
        lo, hi = SEARCH_RANGE_MM
        self.search_mm = config.getfloat("rfid_search_mm", None, minval=lo, maxval=hi)
        order = config.get("rfid_mifare_order", ", ".join(registry.DEFAULT_MIFARE_ORDER))
        self.mifare_order = [n.strip() for n in order.split(",") if n.strip()]
        self.present = False
        self.formats = []
        self.lanes = [dict(state=None, uid=None, source=None, record=None) for _ in range(LANES)]
        self.field = None
        self.sessions = {}
        self.waiters = {}  # lane -> the completion of a MOVE=0 read, ended by any state event
        self.move_waiters = {}  # lane -> the completion of a MOVE=1 read, ended by the load's end
        self.dump_next = set()
        start = self.printer.get_start_args()
        self.config_dir = os.path.dirname(start.get("config_file", "") or ".")
        self.log_dir = os.path.dirname(start.get("log_file", "") or "") or self.config_dir
        self.cache_path = os.path.join(self.config_dir, CACHE_FILE)
        self.cache = self._load_cache()
        self.timer = self.reactor.register_timer(self._session_timeout)
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler("ace2k:feed_event", self._feed_event)
        gcode = self.printer.lookup_object("gcode")
        for name in ("ACE_RFID_READ", "ACE_RFID_FORGET"):
            gcode.register_command(
                name, getattr(self, "cmd_" + name), desc=getattr(self, "cmd_" + name + "_help")
            )

    # --- configuration -------------------------------------------------------------------

    def _build_config(self):
        verdict, detail = probe_format(self.mcu, READ_FMT)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, READ_FMT, detail)
        if verdict == "absent":
            logging.warning(
                "ace2k: firmware built without CONFIG_ACE2K_RFID_READ; the RFID G-codes are"
                " disabled (%s)",
                detail,
            )
            self.present = False
            return
        # every other format the module sends or waits for, checked here: one from another
        # commit refuses the connect instead of raising later in a reactor callback
        for fmt in (READ_RESP, STEP_FMT, DONE_FMT, FORGET_FMT, LANE_QUERY, LANE_STATE):
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(self.printer, self.section, fmt, detail, verdict, READ_FMT)
        self.present = True
        ticks = self.mcu.seconds_to_clock(REPORT_S)
        self.mcu.add_config_cmd(f"ace2k_rfid_query rest_ticks={ticks}", is_init=True)
        if self.search_mm is not None:
            if self.mcu.try_lookup_command(SEARCH_FMT) is not None:
                um = int(round(self.search_mm * 1000.0))
                self.mcu.add_config_cmd(f"ace2k_rfid_search_set search_um={um}")
            else:
                logging.warning(
                    "ace2k: rfid_search_mm is set but the firmware has no load search"
                    " (built without CONFIG_ACE2K_FEED); ignored"
                )
        for handler, fmt in (
            (self._handle_state, STATE_FMT),
            (self._handle_tag, TAG_FMT),
            (self._handle_data, DATA_FMT),
            (self._handle_event, EVENT_FMT),
        ):
            verdict, detail = probe_format(self.mcu, fmt)
            if verdict != "ok":
                raise refuse_format(self.printer, self.section, fmt, detail, verdict, READ_FMT)
            subscribe_response(self.mcu, handler, fmt)

    def _on_connect(self):
        """The brands, from the [ace2k_tag] sections Klipper loaded."""
        tags = [obj for _name, obj in self.printer.lookup_objects("ace2k_tag")]
        if not tags:
            logging.warning(
                "ace2k: no [ace2k_tag] section (include ace2k_tags.cfg); tags are read by UID only"
            )
        sections = {t.brand: dict(t.params, enabled=str(t.enabled)) for t in tags}
        try:
            self.formats = registry.build(sections, self.mifare_order)
        except ValueError as e:
            raise self.printer.config_error(f"{self.section}: {e}") from None
        logging.info("ace2k: tag brands %s", ", ".join(f.name for f in self.formats) or "none")

    def _on_ready(self):
        """Schedule the seeding: Klipper forbids a blocking query (a reactor pause) inside a
        klippy:ready handler, so the lane queries run from a timer just after it."""
        if not self.present:
            return
        self.reactor.register_timer(self._seed_lanes, self.reactor.monotonic() + READY_SEED_S)

    def _seed_lanes(self, eventtime):
        """The unit's picture of each lane: a read lane takes the cache's record of its UID, or is
        forgotten so that its next move reads it again.  Runs once."""
        query = self.mcu.lookup_query_command(LANE_QUERY, LANE_STATE)
        for lane in range(LANES):
            try:
                params = query.send([lane])
            except self.printer.command_error as e:
                # a lost answer must not take the reactor down from this timer: the lane
                # stays unknown until its next report, its cached record is not restored
                logging.warning("ace2k: lane %d tag state query failed: %s", lane + 1, e)
                continue
            state = STATE_NAMES.get(params["state"], params["state"])
            self.lanes[lane]["state"] = state
            if state != "read":
                continue
            uid = bytes(params["uid"]).hex().upper()
            cached = self.cache.get(uid)
            if cached is not None:
                self.lanes[lane].update(uid=uid, source="cached", record=cached)
            else:
                self.mcu.lookup_command(FORGET_FMT).send([lane])
        return self.reactor.NEVER

    # --- the cache and the dump -----------------------------------------------------------

    def _load_cache(self):
        try:
            with open(self.cache_path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _save_cache(self):
        try:
            tmp = self.cache_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.cache, f, indent=1, sort_keys=True)
            os.replace(tmp, self.cache_path)
        except OSError as e:
            logging.warning("ace2k: tag cache not written: %s", e)

    def _dump(self, s, fmt_name):
        path = os.path.join(
            self.log_dir, f"ace2k_tag_lane{s.lane + 1}_{time.strftime('%Y%m%d-%H%M%S')}.json"
        )
        body = {
            "uid": s.chip.uid.hex().upper(),
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
            logging.info("ace2k: lane %d tag dump written to %s", s.lane + 1, path)
        except OSError as e:
            logging.warning("ace2k: tag dump not written: %s", e)

    # --- reports and events (the serial thread → the reactor) -----------------------------

    def _handle_state(self, params):
        states = list(params["state"])
        field = params["field"]

        def apply(_eventtime):
            for i, s in enumerate(states[:LANES]):
                self.lanes[i]["state"] = STATE_NAMES.get(s, s)
            self.field = field

        self.reactor.register_async_callback(apply)

    def _handle_tag(self, params):
        lane, sid = params["lane"], params["session"]
        chip = base.Chip(uid=bytes(params["uid"]), atqa=params["atqa"], sak=params["sak"])
        self.reactor.register_async_callback(lambda e: self._session_start(lane, sid, chip))

    def _handle_data(self, params):
        args = (params["lane"], params["session"], params["block"], params["status"])
        payload = bytes(params["data"])
        self.reactor.register_async_callback(lambda e: self._session_data(*args, payload))

    def _handle_event(self, params):
        lane, kind = params["lane"], params["kind"]
        self.reactor.register_async_callback(lambda e: self._lane_event(lane, kind))

    def _lane_event(self, lane, kind):
        if kind != KIND_AMBIGUOUS:
            self.lanes[lane]["state"] = STATE_NAMES.get(kind, kind)
        if kind == 0:
            self.lanes[lane].update(uid=None, source=None, record=None)
        waiter = self.waiters.pop(lane, None)
        if waiter is not None:
            waiter.complete(kind)

    def _feed_event(self, lane, event):
        """ace2k_feed.py's Klipper event, in the reactor: the end of the unit's own load on a lane
        whose MOVE=1 read waits — the search is over, back where it started."""
        waiter = self.move_waiters.get(lane)
        if waiter is None or event["seq"] != 0 or event["mode"] != "loading":
            return
        if event["kind"] in {KIND_NAMES[k] for k in NOTICE_KINDS}:
            return
        del self.move_waiters[lane]
        waiter.complete(event)

    # --- the session ------------------------------------------------------------------------

    def _session_start(self, lane, sid, chip):
        candidates = [f for f in self.formats if f.claims(chip)]
        dump = lane in self.dump_next
        self.dump_next.discard(lane)
        s = _Session(lane, sid, chip, candidates, dump)
        self.sessions[lane] = s
        self.lanes[lane]["state"] = "reading"
        self._advance(s)

    def _send_step(self, s, step, retry=False):
        s.step = step
        s.retried = retry
        s.expected = step.blocks if step.op == base.OP_NTAG else bin(step.blocks).count("1")
        self.mcu.lookup_command(STEP_FMT).send(
            [s.lane, s.sid, step.op, step.arg, step.blocks, step.key]
        )
        s.deadline = self.reactor.monotonic() + STEP_TIMEOUT_S
        self.reactor.update_timer(self.timer, self._next_deadline())

    def _advance(self, s):
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
            # the bytes read stay (they are the tag's, whichever key opened them); a refused
            # key was this brand's own, so the next brand starts with none marked
            s.auth_failed |= s.got.auth_failed
            s.got.auth_failed = set()
            s.idx += 1
        self._finish(s, _uid_record(s.chip), "uid", "uid")

    def _store(self, s, block, payload):
        if s.step.op == base.OP_NTAG:
            for i in range(len(payload) // PAGE_BYTES):
                s.got.pages[block + i] = payload[i * PAGE_BYTES : (i + 1) * PAGE_BYTES]
        else:
            s.got.blocks[block] = payload

    def _session_data(self, lane, sid, block, status, payload):
        s = self.sessions.get(lane)
        if s is None or s.sid != sid or s.step is None:
            return  # another session's frame, or one after the end
        if status == STATUS_TAG_GONE:
            self._end(s)  # the unit has ended the session itself
            self._fallback(s)
            return
        if status == STATUS_AUTH_FAILED:
            # only a refused key marks (sector, key) failed: the brand's other keys, or the next
            # brand, are tried
            key_type = base.KEY_B if s.step.op == base.OP_MIFARE_B else base.KEY_A
            s.got.auth_failed.add((s.step.arg, key_type))
            s.step = None
            self._advance(s)
            return
        if status != STATUS_OK:
            # an error (a CRC, a NAK, no answer), NTAG or MIFARE: transient until it repeats —
            # the same step once more, then give up (a right brand is never declined for noise)
            if not s.retried:
                self._send_step(s, s.step, retry=True)
            elif s.step.op == base.OP_NTAG and s.chip.uid.hex().upper() not in self.cache:
                # pages that refuse twice (a smaller chip, a password): the UID alone — unless
                # the cache knows the UID, whose record noise must not replace (below)
                self._finish(s, _uid_record(s.chip), "uid", "uid")
            else:
                self._give_up(s)
            return
        self._store(s, block, payload)
        s.expected -= 1
        if s.expected <= 0:
            s.step = None
            self._advance(s)

    def _end(self, s):
        self.sessions.pop(s.lane, None)
        self.reactor.update_timer(self.timer, self._next_deadline())

    def _fallback(self, s):
        """The UID alone: the cache's record of it, if any."""
        uid = s.chip.uid.hex().upper()
        cached = self.cache.get(uid)
        if cached is not None:
            self.lanes[s.lane].update(uid=uid, source="cached", record=cached)

    def _finish(self, s, record, source, fmt_name):
        self.mcu.lookup_command(DONE_FMT).send([s.lane, s.sid, 1])
        self._end(s)
        rec = record.as_dict()
        self.lanes[s.lane].update(uid=record.uid, source=source, record=rec)
        if source == "tag":
            self.cache[record.uid] = rec
            self._save_cache()
        if s.dump:
            self._dump(s, fmt_name)
        logging.info("ace2k: lane %d tag %s — %s", s.lane + 1, record.uid, record.summary())
        self.printer.send_event("ace2k:tag_read", s.lane, rec)

    def _next_deadline(self):
        deadlines = [s.deadline for s in self.sessions.values() if s.deadline is not None]
        return min(deadlines) if deadlines else self.reactor.NEVER

    def _give_up(self, s):
        """The host ends the session unread (a step timed out, or failed twice): the cache's
        record of the UID, if any."""
        self.mcu.lookup_command(DONE_FMT).send([s.lane, s.sid, 0])
        if s.dump:
            self._dump(s, "none")
        self._end(s)
        self._fallback(s)

    def _session_timeout(self, eventtime):
        for s in list(self.sessions.values()):
            if s.deadline is not None and s.deadline <= eventtime:
                self._give_up(s)
        return self._next_deadline()

    # --- status -----------------------------------------------------------------------------

    def lane_status(self, index):
        return dict(self.lanes[index])

    def reader_flags(self):
        """The field byte of the last state report by name; None before the first."""
        if self.field is None:
            return None
        return [name for bit, name in FIELD_BITS if self.field & bit]

    def status_lines(self):
        lines = []
        for i, lane in enumerate(self.lanes):
            rec = lane["record"]
            what = "" if rec is None else " — " + base.SpoolRecord(**rec).summary()
            source = "" if lane["source"] in (None, "tag") else f" ({lane['source']})"
            lines.append(f"lane {i + 1}: tag {lane['state'] or 'unknown'}{what}{source}")
        flags = self.reader_flags()
        readers = "no report yet" if flags is None else (", ".join(flags) or "idle")
        lines.append(f"rfid readers: {readers}")
        return lines

    # --- the G-codes ------------------------------------------------------------------------

    def _require(self, gcmd):
        if not self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_RFID_READ")

    def _lane(self, gcmd):
        return gcmd.get_int("LANE", minval=1, maxval=LANES) - 1

    def _search_mm(self):
        """The search's reach a MOVE=1 read waits for: rfid_search_mm, else the firmware's
        default (ACE2K_FEED_SEARCH_DEFAULT_UM), else 750 mm for an image that does not declare
        it."""
        if self.search_mm is not None:
            return self.search_mm
        return firmware_search_mm(self.mcu)

    def _move_timeout(self, search_mm):
        """A MOVE=1 read's wait: out to the search's reach and back, plus the read sessions'
        pauses, at the speed the unit searches at — the feed's current load speed (load_speed, or
        the last ACE_LOAD_SET) through the feed's own budget of a waited move; without the feed
        module, at the lane's floor speed."""
        travel = 2 * search_mm + SESSION_ALLOWANCE_MM
        feeds = self.printer.lookup_objects("ace2k_feed")
        feed = feeds[0][1] if feeds else None
        if feed is not None and feed.present:
            return feed.load_wait_s(travel)
        try:
            floor = self.mcu.get_constant_float("ACE2K_LANE_SPEED_MIN_UM_S") / 1000.0
        except Exception:  # Klipper's msgproto error; KeyError in the tests' fake
            floor = LANE_SPEED_FLOOR_MM_S
        return travel / floor + SEARCH_MARGIN_S

    cmd_ACE_RFID_READ_help = (
        "Read the tag of LANE=<1-4>: in front of the antenna (default), or MOVE=1 to turn the"
        " spool up to rfid_search_mm and back (a lane already read is read again); DUMP=1 saves"
        " the bytes; WAIT=0 returns at once"
    )

    def cmd_ACE_RFID_READ(self, gcmd):
        self._require(gcmd)
        lane = self._lane(gcmd)
        move = gcmd.get_int("MOVE", 0, minval=0, maxval=1)
        if gcmd.get_int("DUMP", 0, minval=0, maxval=1):
            self.dump_next.add(lane)
        wait = gcmd.get_int("WAIT", 1, minval=0, maxval=1)
        completion = None
        waiters = self.move_waiters if move else self.waiters
        if wait:
            # registered before the command: a dead reader's answer, the lane's unchanged
            # state, is sent ahead of the command's response
            completion = self.reactor.completion()
            waiters[lane] = completion
        params = self.mcu.lookup_query_command(READ_FMT, READ_RESP).send([lane, move])
        if not params["accepted"]:
            waiters.pop(lane, None)
            self.dump_next.discard(lane)
            reason = REFUSALS.get(params["reason"], params["reason"])
            raise gcmd.error(f"ace2k: lane {lane + 1} tag read refused: {reason}")
        if move:
            # the unit forgot a read or no_tag lane's tag on accepting: so does the host
            self.lanes[lane].update(uid=None, source=None, record=None)
        if not wait:
            gcmd.respond_info(f"ace2k: lane {lane + 1} tag read started")
            return
        timeout = PROBE_TIMEOUT_S if not move else self._move_timeout(self._search_mm())
        result = completion.wait(self.reactor.monotonic() + timeout, None)
        if lane not in self.sessions:
            # no session took the request (a dead reader, no tag, the timeout): a later
            # automatic read must not write a dump nobody asked for
            self.dump_next.discard(lane)
        if result is None:
            waiters.pop(lane, None)
            raise gcmd.error(f"ace2k: lane {lane + 1}: no tag outcome within {timeout:.0f} s")
        if move:
            self._report_move(gcmd, lane, result)
            return
        if result == KIND_AMBIGUOUS:
            gcmd.respond_info(
                f"ace2k: lane {lane + 1}: ambiguous — the other lane of this antenna holds a"
                " spool whose tag is not read; use MOVE=1"
            )
            return
        gcmd.respond_info(self._outcome_line(lane, STATE_NAMES.get(result, result)))

    def _outcome_line(self, lane, state):
        rec = self.lanes[lane]["record"]
        what = base.SpoolRecord(**rec).summary() if rec else "no record"
        reader = "a" if lane >= 2 else "b"
        flags = self.reader_flags() or []
        dead = f" (reader {reader.upper()} dead)" if f"dead_{reader}" in flags else ""
        return f"ace2k: lane {lane + 1} tag {state} — {what}{dead}"

    def _report_move(self, gcmd, lane, event):
        """A MOVE=1 read's end: the lane's state as the unit holds it now, asked for (the 1 Hz
        report may lag); a load that did not end loaded is named — an error kind as an error."""
        try:
            params = self.mcu.lookup_query_command(LANE_QUERY, LANE_STATE).send([lane])
        except self.printer.command_error as e:
            raise gcmd.error(f"ace2k: lane {lane + 1}: tag state query failed: {e}") from None
        state = STATE_NAMES.get(params["state"], params["state"])
        self.lanes[lane]["state"] = state
        line = self._outcome_line(lane, state)
        kind = event["kind"]
        if kind == "loaded":
            gcmd.respond_info(line)
            return
        if kind in {KIND_NAMES[k] for k in ERROR_KINDS}:
            raise gcmd.error(f"ace2k: lane {lane + 1} tag search failed: {kind}; {line}")
        gcmd.respond_info(f"ace2k: lane {lane + 1} tag search {kind}; {line}")

    cmd_ACE_RFID_FORGET_help = "Forget the tag read on LANE=<1-4>: the next move reads it again"

    def cmd_ACE_RFID_FORGET(self, gcmd):
        self._require(gcmd)
        lane = self._lane(gcmd)
        self.mcu.lookup_command(FORGET_FMT).send([lane])
        self.lanes[lane].update(uid=None, source=None, record=None)
        gcmd.respond_info(f"ace2k: lane {lane + 1} tag forgotten")


def load_config(config):
    # loaded by [ace2k] with printer.load_object(config, "ace2k_rfid"); the keys live in [ace2k]
    return Ace2kRfid(config.getsection("ace2k"))
