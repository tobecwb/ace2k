"""ace2k_rfid.py: the config and init commands, the subscriptions, the event-driven session
against the brand modules, the G-codes, the cache, the seeding — with the fake printer of
test_ace2k_extras and synthetic UIDs."""

from __future__ import annotations

import json
import os
import re
from typing import TYPE_CHECKING, NoReturn

import ace2k
import ace2k_feed
import ace2k_rfid
import ace2k_rfid_kept
import pytest
from ace2k_rfid_kept import KeptReads
from ace2k_tags import base, registry
from test_ace2k_extras import (
    DEFAULT_CONSTANTS,
    RESPONSES,
    FakeCommandError,
    FakeGcmd,
    FakeReactor,
    fire_ready,
    make,
)
from test_ace2k_feed import IDLE_FRAME, event_params
from test_ace2k_tags import anycubic_image, bambu_blocks, ntag_image, sections

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping
    from typing import Any

    from ace2k import Ace2k
    from ace2k_klipper_types import Params
    from ace2k_rfid import Ace2kRfid
    from test_ace2k_extras import FakeCompletion, FakeMcu, FakePrinter, Responses

UID4 = bytes.fromhex("01020304")
UID7 = bytes.fromhex("04112233445566")
RFID_KNOWN_EXTRA = {
    "ace2k_rfid_read",
    "ace2k_rfid_read_response",
    "ace2k_rfid_step",
    "ace2k_rfid_done",
    "ace2k_rfid_forget",
    "ace2k_rfid_lane_query",
    "ace2k_rfid_lane_state",
    "ace2k_rfid_query",
    "ace2k_rfid_state",
    "ace2k_rfid_tag",
    "ace2k_rfid_data",
    "ace2k_rfid_event",
}


class Tag:
    """An [ace2k_tag] object as ace2k_tag.py builds it."""

    def __init__(self, brand: str, params: Mapping[str, str]) -> None:
        self.brand = brand
        self.enabled = str(params.get("enabled", "True")).lower() == "true"
        self.params = {k: v for k, v in params.items() if k != "enabled"}


def rfid_of(module: Ace2k) -> Ace2kRfid:
    return module.rfid


def make_rfid(
    responses: Responses | None = None,
    options: dict[str, Any] | None = None,
    with_tags: bool = True,
    **kwargs: object,
) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    responses = dict(RESPONSES, **(responses or {}))
    responses.setdefault("ace2k_rfid_read", {"lane": 0, "accepted": 1, "reason": 0})
    responses.setdefault("ace2k_rfid_lane_query", {"lane": 0, "state": 0, "uid": b""})
    module, printer, mcu = make(responses, options=options, **kwargs)
    if with_tags:
        for name, params in sections().items():
            printer.add_object(f"ace2k_tag {name}", Tag(name, params))
    mcu.config_callback()
    rfid_of(module)._on_connect()
    return module, printer, mcu


def steps_sent(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt.startswith("ace2k_rfid_step")]


def dones(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt.startswith("ace2k_rfid_done")]


def tag_event(mcu: FakeMcu, lane: int, session: int, uid: bytes, atqa: int, sak: int) -> None:
    mcu.subscriptions["ace2k_rfid_tag"](
        {"lane": lane, "session": session, "uid": uid, "atqa": atqa, "sak": sak}
    )


def data(
    mcu: FakeMcu, lane: int, session: int, block: int, payload: bytes, status: int = 0
) -> None:
    mcu.subscriptions["ace2k_rfid_data"](
        {"lane": lane, "session": session, "block": block, "status": status, "data": payload}
    )


def test_build_config_queries_subscribes_and_sends_the_search_length() -> None:
    module, printer, mcu = make_rfid(options={"rfid_search_mm": 700})
    init = [c for c, is_init in mcu.config_cmds if is_init]
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    assert any(c.startswith("ace2k_rfid_query rest_ticks=") for c in init)
    # an init command: nothing in Klipper's config checksum
    assert "ace2k_rfid_search_set search_um=700000" in init
    assert not any(c.startswith("ace2k_rfid_search_set") for c in cfg)
    for name in ("ace2k_rfid_state", "ace2k_rfid_tag", "ace2k_rfid_data", "ace2k_rfid_event"):
        assert name in mcu.subscriptions
    assert rfid_of(module).present


def test_without_rfid_search_mm_the_units_default_reach_goes_out_at_every_connect() -> None:
    """A removed key reaches the unit as its default, with no MCU restart."""
    module, printer, mcu = make_rfid(
        constants=dict(DEFAULT_CONSTANTS, ACE2K_FEED_SEARCH_DEFAULT_UM=750000)
    )
    sent = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_rfid_search_set")]
    assert sent == [("ace2k_rfid_search_set search_um=750000", True)]


def test_a_search_length_outside_its_range_is_a_config_error() -> None:
    with pytest.raises(Exception, match="rfid_search_mm"):
        make_rfid(options={"rfid_search_mm": 1500})


def test_an_ntag_session_reads_three_steps_and_ends_with_anycubics_record() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 1, 7, UID7, 0x0044, 0x00)
    assert steps_sent(mcu)[-1] == [1, 7, 0, 4, 3, b""]
    pages = ntag_image(anycubic_image())
    for first in (4, 16, 28):
        for group in range(3):
            p = first + 4 * group
            payload = b"".join(pages.get(p + i, bytes(4)) for i in range(4))
            data(mcu, 1, 7, p, payload)
    assert dones(mcu) == [[1, 7, 1]]
    tag = rfid_of(module).lane_status(1)
    assert tag["state"] in (None, "reading", "read")
    assert tag["source"] == "tag" and tag["record"]["format"] == "anycubic"
    assert tag["uid"] == UID7.hex().upper()
    assert ("ace2k:tag_read", (1, dict(tag["record"], unit=None))) in printer.events
    with open(rfid_of(module).cache_path) as f:
        cache = json.load(f)
    assert cache[UID7.hex().upper()]["format"] == "anycubic"


def test_a_mifare_session_tries_bambu_with_its_key_and_decodes() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 0, 3, UID4, 0x0004, 0x08)
    first = steps_sent(mcu)[-1]
    assert first[:5] == [0, 3, 1, 0, 0b110]
    assert first[5] == bytes.fromhex("6b0d673986de")
    blocks = bambu_blocks()
    data(mcu, 0, 3, 1, bytes(blocks[1]))
    data(mcu, 0, 3, 2, bytes(blocks[2]))
    assert steps_sent(mcu)[-1][2:5] == [1, 1, 0b111]
    for b in (4, 5, 6):
        data(mcu, 0, 3, b, bytes(blocks[b]))
    assert steps_sent(mcu)[-1][2:5] == [1, 3, 0b101]
    for b in (12, 14):
        data(mcu, 0, 3, b, bytes(blocks[b]))
    assert dones(mcu) == [[0, 3, 1]]
    assert rfid_of(module).lane_status(0)["record"]["brand"] == "Bambu Lab"


def test_an_auth_failure_moves_to_the_next_brand_and_none_left_is_uid_only() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 2, 9, UID4, 0x0004, 0x08)
    for _ in range(2):  # bambu, snapmaker (creality ships disabled): each fails its first sector
        step = steps_sent(mcu)[-1]
        data(mcu, 2, 9, step[3] * 4, b"", status=1)
    assert dones(mcu) == [[2, 9, 1]]
    tag = rfid_of(module).lane_status(2)
    assert tag["source"] == "uid" and tag["record"]["format"] == "uid"


def test_every_brand_declined_keeps_a_cached_record_of_the_uid() -> None:
    # a UID the cache knows was decoded before by its brand's keys: a search that ends with
    # every brand declined (a key refused at the edge of the antenna's reach) is noise, not a
    # UID-only tag — given up, as an NTAG's refused pages are: the lane keeps the cached record
    # unlatched, and the next move reads the tag again
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    uid_hex = UID4.hex().upper()
    with open(rfid.cache_path, "w") as f:
        json.dump({uid_hex: {"format": "bambu", "uid": uid_hex, "material": "PLA"}}, f)
    rfid.cache = rfid._load_cache()
    tag_event(mcu, 2, 9, UID4, 0x0004, 0x08)
    for _ in range(2):  # bambu, snapmaker: each fails its first sector
        step = steps_sent(mcu)[-1]
        data(mcu, 2, 9, step[3] * 4, b"", status=1)
    assert dones(mcu) == [[2, 9, 0]]  # not latched read: a rewritten tag is read again
    tag = rfid.lane_status(2)
    assert tag["source"] == "cached" and tag["record"]["material"] == "PLA"
    assert not any(name == "ace2k:tag_read" for name, _ in printer.events)
    assert rfid.cache[uid_hex]["format"] == "bambu"  # not replaced by the UID alone


def test_an_error_resends_the_step_once_and_a_second_gives_up_to_the_cache() -> None:
    # an error then the data: the same step once more, and the brand decodes
    module, printer, mcu = make_rfid()
    tag_event(mcu, 0, 8, UID4, 0x0004, 0x08)
    first = steps_sent(mcu)[-1]
    data(mcu, 0, 8, 0, b"", status=ace2k_rfid.STATUS_ERROR)
    assert steps_sent(mcu)[-1] == first and len(steps_sent(mcu)) == 2
    blocks = bambu_blocks()
    for b in (1, 2):
        data(mcu, 0, 8, b, bytes(blocks[b]))
    for b in (4, 5, 6):
        data(mcu, 0, 8, b, bytes(blocks[b]))
    for b in (12, 14):
        data(mcu, 0, 8, b, bytes(blocks[b]))
    assert dones(mcu) == [[0, 8, 1]]
    assert rfid_of(module).lane_status(0)["record"]["brand"] == "Bambu Lab"
    # two errors on the same step: give up, the cache fills in, no key marked failed
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    with open(rfid.cache_path, "w") as f:
        json.dump({UID4.hex().upper(): {"format": "bambu", "uid": UID4.hex().upper()}}, f)
    rfid.cache = rfid._load_cache()
    tag_event(mcu, 0, 9, UID4, 0x0004, 0x08)
    session = rfid.sessions[0]
    data(mcu, 0, 9, 0, b"", status=ace2k_rfid.STATUS_ERROR)
    data(mcu, 0, 9, 0, b"", status=ace2k_rfid.STATUS_ERROR)
    assert len(steps_sent(mcu)) == 2
    assert dones(mcu) == [[0, 9, 0]]
    assert not session.got.auth_failed
    tag = rfid.lane_status(0)
    assert tag["source"] == "cached" and tag["record"]["format"] == "bambu"


def test_a_step_with_no_answer_gives_up_and_the_cache_fills_in() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    with open(rfid.cache_path, "w") as f:
        json.dump({UID4.hex().upper(): {"format": "bambu", "uid": UID4.hex().upper()}}, f)
    rfid.cache = rfid._load_cache()
    tag_event(mcu, 0, 4, UID4, 0x0004, 0x08)
    timer = [t for t in printer.reactor.timers if t[0] == rfid._session_timeout][0]
    printer.reactor.now = timer[1] + 0.01
    timer[1] = rfid._session_timeout(printer.reactor.now)
    assert dones(mcu) == [[0, 4, 0]]
    tag = rfid.lane_status(0)
    assert tag["source"] == "cached" and tag["record"]["format"] == "bambu"


def test_a_tag_gone_frame_ends_the_session_without_done_and_stale_frames_are_ignored() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 0, 5, UID4, 0x0004, 0x08)
    data(mcu, 0, 99, 1, bytes(16))  # another session's frame
    assert dones(mcu) == []
    data(mcu, 0, 5, 0, b"", status=2)
    assert dones(mcu) == []
    assert 0 not in rfid_of(module).sessions


def test_ace_rfid_read_names_a_refusal_and_waits_for_the_outcome() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 2}]})
    gcmd = FakeGcmd({"LANE": "1"})
    with pytest.raises(Exception, match="no_filament"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)

    def outcome(_eventtime: float) -> None:
        mcu.subscriptions["ace2k_rfid_event"]({"lane": 0, "kind": 6, "session": 0})

    # the waiter's completion receives the ambiguous outcome while it waits
    orig = printer.reactor.completion

    def completion() -> FakeCompletion:
        c = orig()
        printer.reactor.while_waiting[c] = [outcome]
        return c

    printer.reactor.completion = completion
    gcmd = FakeGcmd({"LANE": "1"})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert any("ambiguous" in line for line in gcmd.lines)
    assert rfid.lanes[0]["state"] == "pending" or rfid.lanes[0]["state"] is None


def test_dump_writes_the_sessions_bytes_to_the_log_directory() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.dump_next.add(0)
    tag_event(mcu, 0, 6, UID4, 0x0004, 0x08)
    for _ in range(3):
        step = steps_sent(mcu)[-1]
        data(mcu, 0, 6, step[3] * 4, b"", status=1)
    files = [f for f in os.listdir(printer.start_dir) if f.startswith("ace2k_tag_")]
    assert len(files) == 1
    with open(os.path.join(printer.start_dir, files[0])) as f:
        dump = json.load(f)
    assert dump["uid"] == UID4.hex().upper() and dump["sak"] == 0x08


def test_the_ready_seeding_takes_the_cache_or_forgets() -> None:
    uid_hex = UID4.hex().upper()
    module, printer, mcu = make_rfid(
        {
            "ace2k_rfid_lane_query": [
                {"lane": 0, "state": 4, "uid": UID4},
                {"lane": 1, "state": 4, "uid": UID7},
                {"lane": 2, "state": 1, "uid": b""},
                {"lane": 3, "state": 0, "uid": b""},
            ]
        }
    )
    rfid = rfid_of(module)
    with open(rfid.cache_path, "w") as f:
        json.dump({uid_hex: {"format": "bambu", "uid": uid_hex}}, f)
    rfid.cache = rfid._load_cache()
    fire_ready(printer)
    assert rfid.lane_status(0)["source"] == "cached"
    forgets = [data for fmt, data in mcu.commands if fmt.startswith("ace2k_rfid_forget")]
    assert forgets == [[1]]


def test_status_lines_and_get_status_carry_the_tag() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 0, 3, UID4, 0x0004, 0x08)
    blocks = bambu_blocks()
    for b in (1, 2):
        data(mcu, 0, 3, b, bytes(blocks[b]))
    for b in (4, 5, 6):
        data(mcu, 0, 3, b, bytes(blocks[b]))
    for b in (12, 14):
        data(mcu, 0, 3, b, bytes(blocks[b]))
    status = module.get_status(0)
    assert status["lanes"][0]["tag"]["record"]["brand"] == "Bambu Lab"
    assert any("Bambu Lab" in line for line in rfid_of(module).status_lines())


def test_without_the_rfid_firmware_the_gcodes_answer_an_error() -> None:
    absent = {name: f"Unknown command: {name}" for name in RFID_KNOWN_EXTRA}
    module, printer, mcu = make(dict(RESPONSES), lookup_errors=absent)
    mcu.config_callback()
    rfid = rfid_of(module)
    assert not rfid.present
    assert "ace2k_rfid_tag" not in mcu.subscriptions
    with pytest.raises(Exception, match="CONFIG_ACE2K_RFID_READ"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd({"LANE": "1"}))
    assert registry.BRANDS  # the brands import without the firmware


def test_each_brand_tries_its_own_key_after_another_brands_refusal() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 2, 10, UID4, 0x0004, 0x08)
    bambu = steps_sent(mcu)[-1]
    data(mcu, 2, 10, 0, b"", status=ace2k_rfid.STATUS_AUTH_FAILED)
    snapmaker = steps_sent(mcu)[-1]
    # the same sector, the next brand's key: a refusal is the key's, not the sector's
    assert (snapmaker[3], bambu[3]) == (0, 0) and snapmaker[5] != bambu[5]
    session = rfid_of(module).sessions[2]
    assert session.auth_failed == {(0, 0)} and not session.got.auth_failed


def test_the_field_byte_names_the_dead_readers() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    assert module.get_status(0)["rfid_readers"] is None
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([1, 0, 0, 4]), "field": 0x21})
    assert module.get_status(0)["rfid_readers"] == ["field_a", "dead_b"]
    assert rfid.status_lines()[-1] == "rfid readers: field_a, dead_b"
    assert rfid.lane_status(3)["state"] == "read"


def queue_outcome(printer: FakePrinter, mcu: FakeMcu, *events: Params) -> None:
    """The state events the reactor delivers while ACEK_RFID_READ waits."""
    orig = printer.reactor.completion

    def completion() -> FakeCompletion:
        c = orig()
        printer.reactor.while_waiting[c] = [
            (lambda _e, ev=ev: mcu.subscriptions["ace2k_rfid_event"](ev)) for ev in events
        ]
        return c

    printer.reactor.completion = completion


def test_a_read_on_command_ends_on_the_lanes_unchanged_state_and_names_a_dead_reader() -> None:
    module, printer, mcu = make_rfid()
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([1, 1, 0, 0]), "field": 0x20})
    queue_outcome(printer, mcu, {"lane": 1, "kind": 1, "session": 0})
    gcmd = FakeGcmd({"LANE": "2"})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert gcmd.lines == ["ace2k: lane 2 tag pending — no record (reader B dead)"]


def queue_calls(printer: FakePrinter, *calls: Callable[[], object]) -> None:
    """What the reactor delivers while ACEK_RFID_READ waits: callables run in order."""
    orig = printer.reactor.completion

    def completion() -> FakeCompletion:
        c = orig()
        printer.reactor.while_waiting[c] = [(lambda _e, f=f: f()) for f in calls]
        return c

    printer.reactor.completion = completion


def feed_event(
    module: Ace2k, lane: int, kind: str, seq: int = 0, mode: str = "loading"
) -> Callable[[], None]:
    """ace2k_feed.py's Klipper event, as the printer dispatches it to ace2k_rfid."""
    event = dict(kind=kind, mode=mode, motor_mm=0.0, filament_mm=0.0, seq=seq)
    return lambda: rfid_of(module)._feed_event(lane, event)


def rfid_event(mcu: FakeMcu, lane: int, kind: int) -> Callable[[], object]:
    return lambda: mcu.subscriptions["ace2k_rfid_event"]({"lane": lane, "kind": kind, "session": 0})


def test_a_read_with_motion_on_a_read_lane_waits_through_the_search_for_the_new_outcome() -> None:
    uid_hex = UID4.hex().upper()
    module, printer, mcu = make_rfid(
        {"ace2k_rfid_lane_query": {"lane": 0, "state": 4, "uid": UID4}}
    )
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="read", uid="0A0B0C0D", source="cached", record={"format": "uid"})
    new_record = {"format": "uid", "uid": uid_hex}
    queue_calls(
        printer,
        rfid_event(mcu, 0, 1),  # the forget's pending: not an outcome
        rfid_event(mcu, 0, 2),  # searching
        lambda: rfid.lanes[0].update(uid=uid_hex, source="uid", record=new_record),
        rfid_event(mcu, 0, 4),  # read mid-search: the load still returns
        feed_event(module, 0, "behind"),  # a notice: not the end
        feed_event(module, 0, "loaded", seq=7),  # a host start's: not this search
        feed_event(module, 1, "loaded"),  # another lane's
        feed_event(module, 0, "loaded"),
    )
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert gcmd.lines == [f"ace2k: lane 1 tag read — {base.SpoolRecord(**new_record).summary()}"]
    assert rfid.move_waiters == {}
    # the outcome is asked for once the load has ended, after the read command
    names = [name for name, _data in mcu.queries]
    assert names[-2:] == ["ace2k_rfid_read", "ace2k_rfid_lane_query"]
    assert mcu.queries[-1][1] == [0]


def test_a_read_with_motion_forgets_the_old_record_on_acceptance() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="read", uid="0A0B0C0D", source="cached", record={"format": "uid"})
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert rfid.lane_status(0)["record"] is None and rfid.lanes[0]["uid"] is None


def test_a_read_with_motion_whose_load_stopped_names_it_with_the_queried_state() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_event(module, 0, "stopped"))
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert gcmd.lines == ["ace2k: lane 1 tag search stopped; ace2k: lane 1 tag pending — no record"]
    assert rfid_of(module).lanes[0]["state"] == "pending"


def test_a_read_with_motion_whose_load_failed_is_an_error_naming_the_kind() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_event(module, 0, "tangled"))
    with pytest.raises(Exception, match="tag search failed: tangled"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )


# The load of a read with motion, as the unit reports it: kind 5 (loaded), mode 4 (loading), seq 0.
# The figures are those of the refill's tag read in the U1 log: 1481.1 mm of motor out and back,
# 0.0 mm of filament net.
SEARCH_MOTOR_UM = 1481100
SEARCH_LINE = "ace2k: lane 1 tag search: about 740.6 mm out and back, no tag"


def feed_frame(
    module: Ace2k,
    mcu: FakeMcu,
    printer: FakePrinter,
    lane: int,
    kind: int,
    motor_um: int,
    filament_um: int,
) -> Callable[[], None]:
    """An event frame of the unit's own load (seq 0, mode loading) through the feed module.

    The fake printer only records the Klipper event, so this hands it to ace2k_rfid as Klipper
    would."""

    def deliver() -> None:
        mcu.subscriptions["ace2k_feed_event"](event_params(lane, kind, 4, motor_um, filament_um))
        name, (index, event) = printer.events[-1]
        assert name == "ace2k:feed_event"
        rfid_of(module)._feed_event(index, event)

    return deliver


def log_lines(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [r.getMessage() for r in caplog.records]


def test_a_read_with_motion_logs_its_search_and_no_tag_not_a_load(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 5, "uid": b""}})
    queue_calls(
        printer,
        rfid_event(mcu, 0, 1),  # the forget's pending
        rfid_event(mcu, 0, 2),  # searching
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0),
    )
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert gcmd.lines == ["ace2k: lane 1 tag no_tag — no record"]
    lines = log_lines(caplog)
    assert lines.count(SEARCH_LINE) == 1
    assert not [line for line in lines if "loaded (loading)" in line]


def test_a_read_with_motion_shows_as_a_search_in_the_lanes_last_move() -> None:
    """ACEK_STATUS's last= of the lane says the search, not a load whose filament figure is
    about 0. A later load of the unit's own still shows as loaded."""
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 5, "uid": b""}})
    queue_calls(
        printer,
        rfid_event(mcu, 0, 1),  # the forget's pending
        rfid_event(mcu, 0, 2),  # searching
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0),
    )
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd({"LANE": "1", "MOVE": "1"}))
    feed = rfid_of(module).feed
    assert "last=tag search (about 740.6 mm out and back)" in feed.status_lines()[0]
    feed_frame(module, mcu, printer, 0, 5, 800000, 790000)()
    assert "last=loaded (loading, motor 800.0 mm, filament 790.0 mm)" in feed.status_lines()[0]


def test_a_read_with_motion_that_read_a_tag_logs_the_tags_format(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid(
        {"ace2k_rfid_lane_query": {"lane": 0, "state": 4, "uid": UID7}}
    )
    rfid = rfid_of(module)
    record = {"format": "anycubic", "uid": UID7.hex().upper()}
    queue_calls(
        printer,
        rfid_event(mcu, 0, 2),  # searching
        lambda: rfid.lanes[0].update(uid=record["uid"], source="tag", record=record),
        rfid_event(mcu, 0, 4),  # read in the search: the load still comes back
        feed_frame(module, mcu, printer, 0, 5, 801000, 1500),
    )
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )
    lines = log_lines(caplog)
    assert "ace2k: lane 1 tag search: about 400.5 mm out and back, tag anycubic" in lines
    assert not [line for line in lines if "loaded (loading)" in line]


def test_a_read_with_motion_not_waited_logs_its_search_once_the_outcome_comes(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # The unit settles the tag state at its next look after the load's end, so the no_tag event
    # comes after the load's event. The line waits for it.
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        rfid_event(mcu, 0, 1)()
        rfid_event(mcu, 0, 2)()
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
        assert not [line for line in log_lines(caplog) if "tag search" in line]
        rfid_event(mcu, 0, 5)()  # no_tag
        rfid_event(mcu, 0, 5)()  # a repeat says nothing more
    lines = log_lines(caplog)
    assert lines.count(SEARCH_LINE) == 1
    assert not [line for line in lines if "loaded (loading)" in line]


def test_a_search_that_ends_with_the_tag_unread_names_the_lanes_state(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # A tag was seen in the search but not read, so the lane ends pending, not no_tag.
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(
        printer,
        rfid_event(mcu, 0, 2),  # searching
        feed_frame(module, mcu, printer, 0, 5, 400000, 200),
    )
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )
    assert "ace2k: lane 1 tag search: about 200.0 mm out and back, tag not read" in (
        log_lines(caplog)
    )


def test_a_bay_emptied_during_a_read_with_motion_leaves_the_next_load_a_load(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        rfid_event(mcu, 0, 0)()  # unknown: the filament left the bay, the read is over
        assert not rfid_of(module).feed.motion[0].tag_search
        rfid_event(mcu, 0, 1)()  # a new filament
        rfid_event(mcu, 0, 2)()
        feed_frame(module, mcu, printer, 0, 5, 300000, 299800)()  # its automatic load
        rfid_event(mcu, 0, 5)()
    lines = log_lines(caplog)
    assert "ace2k feed: lane 1 loaded (loading) — motor 300.0 mm, filament 299.8 mm" in lines
    assert not [line for line in lines if "tag search" in line]


AUTO_LOAD_LINE = "ace2k feed: lane 1 loaded (loading) — motor 300.0 mm, filament 299.8 mm"


def automatic_load(module: Ace2k, mcu: FakeMcu, printer: FakePrinter) -> None:
    """The unit's automatic load of a new filament on lane 1, to its end."""
    rfid_event(mcu, 0, 1)()  # pending
    rfid_event(mcu, 0, 2)()  # searching
    feed_frame(module, mcu, printer, 0, 5, 300000, 299800)()
    rfid_event(mcu, 0, 5)()


def assert_a_plain_load(caplog: pytest.LogCaptureFixture) -> None:
    lines = log_lines(caplog)
    assert AUTO_LOAD_LINE in lines
    assert not [line for line in lines if "tag search" in line]


@pytest.mark.parametrize("reason", [1, 6])  # busy, other_lane
def test_a_refused_read_with_motion_leaves_the_next_load_a_load(
    caplog: pytest.LogCaptureFixture, reason: int
) -> None:
    module, printer, mcu = make_rfid(
        {"ace2k_rfid_read": {"lane": 0, "accepted": 0, "reason": reason}}
    )
    with caplog.at_level("INFO"):
        with pytest.raises(Exception, match="refused"):
            printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
                FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
            )
        assert not rfid_of(module).feed.motion[0].tag_search
        automatic_load(module, mcu, printer)
    assert_a_plain_load(caplog)


def test_a_read_with_motion_with_no_answer_leaves_the_next_load_a_load(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid()
    mcu.responses["ace2k_rfid_read"] = FakeCommandError(
        "Unable to obtain 'ace2k_rfid_read_response' response"
    )
    with caplog.at_level("INFO"):
        with pytest.raises(Exception, match="no answer"):
            printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
                FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
            )
        assert not rfid_of(module).feed.motion[0].tag_search
        automatic_load(module, mcu, printer)
    assert_a_plain_load(caplog)


def test_a_refused_second_read_keeps_the_running_search_and_its_line(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid()
    mcu.responses["ace2k_rfid_read"] = [
        {"lane": 0, "accepted": 1, "reason": 0},
        {"lane": 0, "accepted": 0, "reason": 1},  # the second read: busy
    ]
    read = printer.lookup_object("gcode").commands["ACEK_RFID_READ"]
    with caplog.at_level("INFO"):
        read(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
        rfid_event(mcu, 0, 1)()
        rfid_event(mcu, 0, 2)()
        with pytest.raises(Exception, match="refused: busy"):
            read(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
        assert rfid_of(module).feed.motion[0].tag_search
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
        rfid_event(mcu, 0, 5)()
    lines = log_lines(caplog)
    assert [line for line in lines if "tag search" in line] == [SEARCH_LINE]
    assert not [line for line in lines if "loaded (loading)" in line]


def test_a_search_not_waited_settles_on_the_state_report_alone(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # The no_tag event is lost: the 1 Hz state report settles the line.
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        rfid_event(mcu, 0, 2)()
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
        mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([2, 0, 0, 0]), "field": 0})
        assert not [line for line in log_lines(caplog) if "tag search" in line]
        mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([5, 0, 0, 0]), "field": 0})
    assert log_lines(caplog).count(SEARCH_LINE) == 1


def test_a_search_that_ends_while_reading_waits_for_the_read(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    record = {"format": "bambu", "uid": UID4.hex().upper()}
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        rfid_event(mcu, 0, 3)()  # reading
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
        assert not [line for line in log_lines(caplog) if "tag search" in line]
        rfid.lanes[0].update(uid=record["uid"], source="tag", record=record)
        rfid_event(mcu, 0, 4)()  # read
    assert "ace2k: lane 1 tag search: about 740.6 mm out and back, tag bambu" in log_lines(caplog)


@pytest.mark.parametrize(
    "kind,outcome",
    [
        (0, "bay emptied"),  # unknown
        (4, "tag read"),  # read, but no record kept on the host
        (9, "tag state 9"),  # a state this host does not know
    ],
)
def test_a_search_names_other_outcomes_in_plain_words(
    caplog: pytest.LogCaptureFixture, kind: int, outcome: str
) -> None:
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        rfid_event(mcu, 0, 2)()
        feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
        rfid_event(mcu, 0, kind)()
    line = f"ace2k: lane 1 tag search: about 740.6 mm out and back, {outcome}"
    assert line in log_lines(caplog)


def test_a_state_report_of_an_empty_bay_ends_the_search(caplog: pytest.LogCaptureFixture) -> None:
    # The search's final event and the unknown event are both lost. The state report still
    # shows the bay empty, so the next automatic load is a load.
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 0, 0, 0]), "field": 0})
        assert not rfid_of(module).feed.motion[0].tag_search
        automatic_load(module, mcu, printer)
    assert_a_plain_load(caplog)


def search_to_its_end(module: Ace2k, mcu: FakeMcu, printer: FakePrinter) -> None:
    """The unit's search on lane 1 after a read with motion: searching, its load's end, no tag."""
    rfid_event(mcu, 0, 2)()
    feed_frame(module, mcu, printer, 0, 5, SEARCH_MOTOR_UM, 0)()
    rfid_event(mcu, 0, 5)()


def test_an_old_state_report_applied_after_the_read_keeps_the_search(
    caplog: pytest.LogCaptureFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    # A state report sampled before the unit took MOVE=1 shows the bay unknown. Klipper's reactor
    # can run its callback after the read's answer, so it reaches the host after the flag is set.
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    queued: list[Callable[[float], object]] = []
    monkeypatch.setattr(rfid.reactor, "register_async_callback", lambda cb, *a: queued.append(cb))
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 0, 0, 0]), "field": 0})
    monkeypatch.undo()
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        assert len(queued) == 1
        queued[0](0.0)
        assert rfid.feed.motion[0].tag_search
        search_to_its_end(module, mcu, printer)
    assert log_lines(caplog).count(SEARCH_LINE) == 1
    assert not [line for line in log_lines(caplog) if "loaded (loading)" in line]


def test_an_old_state_report_applied_during_the_read_keeps_the_search(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # The same report, received while the read is in flight: its callback runs before the answer.
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)

    def answer() -> dict[str, int]:
        mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 0, 0, 0]), "field": 0})
        return {"lane": 0, "accepted": 1, "reason": 0}

    mcu.responses["ace2k_rfid_read"] = answer
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        assert rfid.feed.motion[0].tag_search
        search_to_its_end(module, mcu, printer)
    assert log_lines(caplog).count(SEARCH_LINE) == 1


def test_an_accepted_host_start_ends_the_search(caplog: pytest.LogCaptureFixture) -> None:
    # The unit accepts a start only on an idle lane, so the search is over even if its final
    # event was lost.
    module, printer, mcu = make_rfid()
    mcu.responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
        )
        assert rfid_of(module).feed.motion[0].tag_search
        module.feed.start_move(0, "feed", 50.0, 30.0)
        assert not rfid_of(module).feed.motion[0].tag_search
        automatic_load(module, mcu, printer)
    assert_a_plain_load(caplog)


def test_an_automatic_load_keeps_its_loaded_line(caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = make_rfid()
    with caplog.at_level("INFO"):
        rfid_event(mcu, 0, 2)()
        feed_frame(module, mcu, printer, 0, 5, 300000, 299800)()
        rfid_event(mcu, 0, 5)()
    lines = log_lines(caplog)
    assert "ace2k feed: lane 1 loaded (loading) — motor 300.0 mm, filament 299.8 mm" in lines
    assert not [line for line in lines if "tag search" in line]


def test_a_read_with_motion_that_stopped_keeps_the_feed_line_and_the_next_load_is_a_load(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_frame(module, mcu, printer, 0, 1, 120000, 120000))  # stopped
    with caplog.at_level("INFO"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )
        feed_frame(module, mcu, printer, 0, 5, 300000, 299800)()  # a later automatic load
        rfid_event(mcu, 0, 5)()
    lines = log_lines(caplog)
    assert "ace2k feed: lane 1 stopped (loading) — motor 120.0 mm, filament 120.0 mm" in lines
    assert "ace2k feed: lane 1 loaded (loading) — motor 300.0 mm, filament 299.8 mm" in lines
    assert not [line for line in lines if "tag search" in line]


def test_a_read_with_motion_refused_busy_waits_for_nothing() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_read": {"lane": 0, "accepted": 0, "reason": 1}})
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="reading", uid="0A0B0C0D", source="uid", record={"format": "uid"})
    with pytest.raises(Exception, match="refused: busy"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "DUMP": "1"})
        )
    assert rfid.move_waiters == {} and rfid.dump_next == set()
    assert rfid.lanes[0]["record"] == {"format": "uid"}  # a refusal changes nothing


def test_a_dump_asked_with_motion_survives_the_forgets_pending_and_reaches_the_session() -> None:
    module, printer, mcu = make_rfid(
        {"ace2k_rfid_lane_query": {"lane": 0, "state": 3, "uid": UID7}}
    )
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="read", uid="0A0B0C0D", source="cached", record={"format": "uid"})
    queue_calls(
        printer,
        rfid_event(mcu, 0, 1),  # the forget's pending
        rfid_event(mcu, 0, 2),  # searching
        lambda: tag_event(mcu, 0, 21, UID7, 0x0044, 0x00),  # the session opens mid-search
        feed_event(module, 0, "loaded"),
    )
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
        FakeGcmd({"LANE": "1", "MOVE": "1", "DUMP": "1"})
    )
    assert rfid.sessions[0].dump
    assert rfid.dump_next == set()


def test_a_read_with_no_outcome_times_out_with_an_error() -> None:
    module, printer, mcu = make_rfid()
    with pytest.raises(Exception, match="no tag outcome"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd({"LANE": "1"}))
    assert rfid_of(module).waiters == {}


def test_forget_sends_the_command_and_clears_the_record() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.lanes[0].update(uid="01020304", source="cached", record={"format": "uid"})
    printer.lookup_object("gcode").commands["ACEK_RFID_FORGET"](FakeGcmd({"LANE": "1"}))
    assert [d for f, d in mcu.commands if f.startswith("ace2k_rfid_forget")] == [[0]]
    assert rfid.lane_status(0)["record"] is None


def test_a_lost_lane_answer_at_ready_leaves_that_lane_unknown(
    caplog: pytest.LogCaptureFixture,
) -> None:
    uid_hex = UID4.hex().upper()
    module, printer, mcu = make_rfid(
        {
            "ace2k_rfid_lane_query": [
                FakeCommandError("Unable to obtain 'ace2k_rfid_lane_state' response"),
                {"lane": 1, "state": 4, "uid": UID4},
                {"lane": 2, "state": 1, "uid": b""},
                {"lane": 3, "state": 0, "uid": b""},
            ]
        }
    )
    rfid = rfid_of(module)
    with open(rfid.cache_path, "w") as f:
        json.dump({uid_hex: {"format": "bambu", "uid": uid_hex}}, f)
    rfid.cache = rfid._load_cache()
    with caplog.at_level("WARNING"):
        fire_ready(printer)  # does not raise
    assert rfid.lane_status(0)["state"] is None and rfid.lane_status(0)["record"] is None
    assert rfid.lane_status(1)["source"] == "cached"
    assert any("lane 1 tag state query failed" in r.getMessage() for r in caplog.records)
    assert "ace2k: lane 1 tag state: no answer from the unit; check the link" in caplog.text


@pytest.mark.parametrize(
    "fmt",
    [
        ace2k_rfid.READ_RESP,
        ace2k_rfid.STEP_FMT,
        ace2k_rfid.DONE_FMT,
        ace2k_rfid.FORGET_FMT,
        ace2k_rfid.LANE_QUERY,
        ace2k_rfid.LANE_STATE,
    ],
)
def test_every_format_is_checked_at_connect(fmt: str) -> None:
    name = fmt.split(maxsplit=1)[0]
    with pytest.raises(Exception, match=name):
        make_rfid(formats={name: name + " other=%c"})


def test_ntag_pages_that_refuse_twice_end_with_the_uid_alone() -> None:
    module, printer, mcu = make_rfid()
    tag_event(mcu, 1, 11, UID7, 0x0044, 0x00)
    data(mcu, 1, 11, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    data(mcu, 1, 11, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    assert len(steps_sent(mcu)) == 2
    assert dones(mcu) == [[1, 11, 1]]
    tag = rfid_of(module).lane_status(1)
    assert tag["source"] == "uid" and tag["record"]["uid"] == UID7.hex().upper()


def test_a_dump_request_is_dropped_on_refusal_and_on_timeout() -> None:
    module, printer, mcu = make_rfid({"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 1}]})
    with pytest.raises(Exception, match="busy"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "DUMP": "1"})
        )
    assert rfid_of(module).dump_next == set()
    module, printer, mcu = make_rfid()
    with pytest.raises(Exception, match="no tag outcome"):
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](
            FakeGcmd({"LANE": "1", "DUMP": "1"})
        )
    assert rfid_of(module).dump_next == set()


def test_ntag_pages_that_refuse_twice_keep_a_cached_record() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    uid_hex = UID7.hex().upper()
    with open(rfid.cache_path, "w") as f:
        json.dump({uid_hex: {"format": "anycubic", "uid": uid_hex}}, f)
    rfid.cache = rfid._load_cache()
    tag_event(mcu, 1, 12, UID7, 0x0044, 0x00)
    data(mcu, 1, 12, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    data(mcu, 1, 12, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    assert dones(mcu) == [[1, 12, 0]]  # not latched read on noise
    tag = rfid.lane_status(1)
    assert tag["source"] == "cached" and tag["record"]["format"] == "anycubic"
    assert rfid.cache[uid_hex]["format"] == "anycubic"


def test_a_dump_request_is_dropped_when_the_wait_ends_without_a_session() -> None:
    module, printer, mcu = make_rfid()
    # a dead reader: the lane's unchanged state, at once, and no session
    queue_outcome(printer, mcu, {"lane": 0, "kind": 1, "session": 0})
    printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd({"LANE": "1", "DUMP": "1"}))
    assert rfid_of(module).dump_next == set()


def test_the_move_wait_takes_the_firmwares_search_default() -> None:
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_SEARCH_DEFAULT_UM=800000)
    module, printer, mcu = make_rfid(constants=constants)
    assert rfid_of(module)._search_mm() == 800.0
    module, printer, mcu = make_rfid()  # no constant: 750
    assert rfid_of(module)._search_mm() == 750.0
    module, printer, mcu = make_rfid(options={"rfid_search_mm": 700}, constants=constants)
    assert rfid_of(module)._search_mm() == 700.0  # printer.cfg wins


def test_the_move_wait_follows_the_feeds_load_speed() -> None:
    module, printer, mcu = make_rfid(options={"load_speed": 10})
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    with pytest.raises(Exception, match="no tag outcome") as err:
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    waited = float(re.search(r"within (\d+) s", str(err.value)).group(1))
    assert waited >= 2 * 750.0 / 10  # out and back at 10 mm/s: 150 s of travel alone
    feed = printer.lookup_object("ace2k_feed")
    assert waited == round(feed.load_wait_s(2 * 750.0 + ace2k_feed.SESSION_ALLOWANCE_MM))
    feed.load_speed_mm_s = 60.0  # an ACEK_LOAD_SET since: the next wait follows it
    with pytest.raises(Exception, match="no tag outcome") as err:
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](gcmd)
    assert float(re.search(r"within (\d+) s", str(err.value)).group(1)) < waited


def test_ready_only_schedules_the_seeding_and_the_timer_applies_it() -> None:
    uid_hex = UID4.hex().upper()
    module, printer, mcu = make_rfid(
        {"ace2k_rfid_lane_query": [{"lane": 0, "state": 4, "uid": UID4}]}
    )
    rfid = rfid_of(module)
    with open(rfid.cache_path, "w") as f:
        json.dump({uid_hex: {"format": "bambu", "uid": uid_hex}}, f)
    rfid.cache = rfid._load_cache()
    printer.handlers["klippy:ready"]()  # Klipper refuses a pause here: nothing may block
    assert not [q for q in mcu.queries if q[0] == "ace2k_rfid_lane_query"]
    (timer,) = [t for t in printer.reactor.timers if t[0] == rfid._seed_lanes]
    assert timer[1] == printer.reactor.now + ace2k_rfid.READY_SEED_S
    timer[1] = rfid._seed_lanes(timer[1])
    assert timer[1] == FakeReactor.NEVER  # once
    assert [q[1] for q in mcu.queries if q[0] == "ace2k_rfid_lane_query"] == [[0], [1], [2], [3]]
    assert rfid.lane_status(0)["source"] == "cached" and rfid.lane_status(0)["state"] == "read"


def test_the_fake_refuses_a_blocking_query_inside_a_ready_handler() -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    printer.reactor.pause_disabled = True  # as while klippy:ready handlers run
    with pytest.raises(RuntimeError, match="reactor pause disabled"):
        rfid._seed_lanes(printer.reactor.now)


def test_a_tag_read_with_no_answer_is_a_named_gcode_error_and_leaves_no_waiter() -> None:
    held = []
    module, printer, mcu = make_rfid()

    def silent() -> NoReturn:
        held.append(ace2k.query_mutex(printer, mcu).depth)
        raise FakeCommandError("Unable to obtain 'ace2k_rfid_read_response' response")

    mcu.responses["ace2k_rfid_read"] = silent
    rfid = rfid_of(module)
    for params in ({"LANE": "1"}, {"LANE": "1", "MOVE": "1", "DUMP": "1"}):
        with pytest.raises(Exception) as caught:
            printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd(params))
        assert not isinstance(caught.value, FakeCommandError)
        assert str(caught.value).startswith(
            "ace2k: lane 1 tag read: no answer from the unit; check the link"
        )
        assert rfid.waiters == {} and rfid.move_waiters == {} and not rfid.dump_next
    assert held == [1, 1]


def test_start_read_asks_a_read_in_front_of_the_antenna_and_does_not_wait() -> None:
    # the host interface's tag read (API_VERSION 7): what ACEK_RFID_READ LANE=n MOVE=0 WAIT=0
    # does, callable from a timer, its refusal a RuntimeError naming the reason
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.start_read(2)
    assert [q for q in mcu.queries if q[0] == "ace2k_rfid_read"][-1][1] == [2, 0, 0]  # no key
    assert 2 not in rfid.waiters
    with pytest.raises(ValueError):
        rfid.start_read(4)
    module, printer, mcu = make_rfid({"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 2}]})
    with pytest.raises(RuntimeError, match="lane 1 tag read refused: no_filament"):
        rfid_of(module).start_read(0)


def test_start_read_without_the_rfid_firmware_is_a_runtime_error() -> None:
    absent = {name: f"Unknown command: {name}" for name in RFID_KNOWN_EXTRA}
    module, printer, mcu = make(dict(RESPONSES), lookup_errors=absent)
    mcu.config_callback()
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_RFID_READ"):
        rfid_of(module).start_read(0)


def test_start_read_raises_only_runtime_or_value_errors() -> None:
    # the host interface's contract: no answer is NoAnswerError, a RuntimeError, like every
    # other failure but a bad lane (ValueError)
    assert issubclass(ace2k.NoAnswerError, RuntimeError)
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)

    def lost(*args: object, **kwargs: object) -> NoReturn:
        raise ace2k.NoAnswerError("ace2k: lane 1 tag read: no answer from the unit")

    rfid_module_ask = ace2k_rfid.ask
    ace2k_rfid.ask = lost
    try:
        with pytest.raises(RuntimeError, match="no answer from the unit"):
            rfid.start_read(0)
    finally:
        ace2k_rfid.ask = rfid_module_ask


def test_the_gcode_and_start_read_share_one_read_request() -> None:
    # one helper sends the read and maps the refusal: the G-code's and start_read's lines agree
    refused = {"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 2}] * 2}
    module, printer, mcu = make_rfid(dict(refused))
    rfid = rfid_of(module)
    with pytest.raises(RuntimeError) as direct:
        rfid.start_read(0)
    with pytest.raises(Exception) as by_gcode:
        printer.lookup_object("gcode").commands["ACEK_RFID_READ"](FakeGcmd({"LANE": "1"}))
    assert str(direct.value) == str(by_gcode.value) == "ace2k: lane 1 tag read refused: no_filament"
    assert hasattr(rfid, "_send_read")


# --- the read's retry key (seq) ------------------------------------------------------------------


def reads(mcu: FakeMcu) -> list[list[Any]]:
    return [data for name, data in mcu.queries if name == "ace2k_rfid_read"]


def test_a_read_with_motion_carries_a_seq_per_lane_from_1_to_255_and_round() -> None:
    """Klipper sends a query again when its answer is lost, with the same bytes. The seq makes the
    unit take that copy as the same read, not as a second search."""
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.read_seq = [0] * 4  # as if each lane's counter started at 0
    for _ in range(2):
        rfid.cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
    rfid.cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "2", "MOVE": "1", "WAIT": "0"}))
    rfid.cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "1", "MOVE": "0", "WAIT": "0"}))
    assert reads(mcu) == [[0, 1, 1], [0, 1, 2], [1, 1, 1], [0, 0, 0]]
    rfid.read_seq[0] = 255
    rfid.cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
    assert reads(mcu)[-1] == [0, 1, 1]  # 0 is never a key


def test_an_older_image_without_the_seq_still_reads() -> None:
    old = "ace2k_rfid_read lane=%c move=%c"
    module, printer, mcu = make_rfid(formats={"ace2k_rfid_read": old})
    rfid = rfid_of(module)
    assert rfid.present
    rfid.cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
    rfid.start_read(1)
    assert reads(mcu) == [[0, 1], [1, 0]]


def test_a_read_in_yet_another_format_is_refused() -> None:
    with pytest.raises(Exception, match="ace2k_rfid_read"):
        make_rfid(formats={"ace2k_rfid_read": "ace2k_rfid_read lane=%c"})


def test_the_seq_starts_anywhere_so_a_restarted_host_does_not_repeat_the_units_key(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A Klipper RESTART keeps the unit, and its key of the last read. A host that started from 1
    every time could send that key again, and the unit would take a new read as the old one."""
    monkeypatch.setattr(ace2k_rfid.random, "randrange", lambda n: 41)
    module, printer, mcu = make_rfid()
    rfid_of(module).cmd_ACEK_RFID_READ(FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"}))
    assert reads(mcu) == [[0, 1, 42]]


# --- the last read of a lane, kept across a restart -----------------------------------------


def read_anycubic(mcu: FakeMcu, lane: int = 1, session: int = 7) -> None:
    """An NTAG session on `lane` that ends with Anycubic's record."""
    tag_event(mcu, lane, session, UID7, 0x0044, 0x00)
    pages = ntag_image(anycubic_image())
    for first in (4, 16, 28):
        for group in range(3):
            p = first + 4 * group
            data(mcu, lane, session, p, b"".join(pages.get(p + i, bytes(4)) for i in range(4)))


def restarted(
    before: Ace2k, states: list[int], uid: bytes = b""
) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    """A new host after a restart: the lanes file of `before` is in its config directory, and the
    unit answers each lane's state from `states`, a read lane with `uid`. A pending lane is a full
    bay of a unit that the restart reset."""
    answers = [{"lane": i, "state": s, "uid": uid if s == 4 else b""} for i, s in enumerate(states)]
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": answers})
    rfid = rfid_of(module)
    with open(rfid_of(before).kept.path) as src, open(rfid.kept.path, "w") as dst:
        dst.write(src.read())
    rfid.kept = KeptReads(rfid.kept.path, 4, "ace2k", printer.reactor.monotonic)
    for lane in module.lanes:
        lane["insert"] = True
    fire_ready(printer)
    return module, printer, mcu


def test_a_lanes_read_is_kept_beside_the_cache() -> None:
    module, printer, mcu = make_rfid()
    read_anycubic(mcu)
    rfid = rfid_of(module)
    assert rfid.kept.path == rfid.cache_path[: -len(".json")] + ".lanes.json"
    with open(rfid.kept.path) as f:
        assert json.load(f)["2"]["format"] == "anycubic"


def test_a_full_bay_after_a_restart_takes_its_kept_read_shown_as_read() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    tag = rfid_of(module).lane_status(1)
    assert tag["state"] == "read" and tag["source"] == "restored"
    assert tag["record"]["format"] == "anycubic" and tag["uid"] == UID7.hex().upper()
    assert module.get_status(0)["lanes"][1]["tag"]["source"] == "restored"
    assert "(restored)" in rfid_of(module).status_lines()[1]
    # the unit still holds the lane pending: a load searches, and the read stays meanwhile
    rfid_event(mcu, 1, 2)()
    assert rfid_of(module).lane_status(1)["state"] == "searching"
    assert rfid_of(module).lane_status(1)["source"] == "restored"
    rfid_event(mcu, 1, 1)()  # the search ended without a read
    assert rfid_of(module).lane_status(1)["state"] == "read"


def test_a_bay_that_becomes_full_after_the_restart_takes_its_kept_read_then() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 0, 0, 0])  # the unit had not looked yet
    assert rfid_of(module).lane_status(1)["record"] is None
    rfid_event(mcu, 1, 1)()  # pending: the bay is full
    assert rfid_of(module).lane_status(1)["source"] == "restored"


def test_a_bay_seen_empty_drops_the_kept_read() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    rfid = rfid_of(module)
    module.lanes[1]["insert"] = False  # taken out by hand, or by a move
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 1, 0, 0]), "field": 0})
    assert rfid.lane_status(1)["record"] is None and rfid.kept.reads == {}
    with open(rfid.kept.path) as f:
        assert json.load(f) == {}
    module.lanes[1]["insert"] = True  # another spool goes in
    rfid_event(mcu, 1, 1)()
    assert rfid.lane_status(1)["record"] is None


def test_the_unknown_event_drops_the_kept_read() -> None:
    module, printer, mcu = make_rfid()
    read_anycubic(mcu)
    rfid_event(mcu, 1, 0)()
    assert rfid_of(module).kept.reads == {}


def test_a_new_read_replaces_the_restored_one() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    tag_event(mcu, 1, 9, UID4, 0x0044, 0x00)  # another tag, that no brand claims: its UID alone
    tag = rfid_of(module).lane_status(1)
    assert tag["source"] == "uid" and tag["uid"] == UID4.hex().upper()
    assert rfid_of(module).kept.get(1)["uid"] == UID4.hex().upper()


def test_the_same_tag_read_again_replaces_the_restored_read_as_a_tag() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    read_anycubic(mcu, session=9)
    assert rfid_of(module).lane_status(1)["source"] == "tag"


def test_a_search_that_finds_no_tag_drops_the_restored_read() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    rfid_event(mcu, 1, 5)()
    assert rfid_of(module).lane_status(1)["record"] is None and rfid_of(module).kept.reads == {}


@pytest.mark.parametrize(
    "params", [{"LANE": "2"}, {"LANE": "2", "MOVE": "1", "WAIT": "0"}], ids=["forget", "re-read"]
)
def test_a_forget_or_a_read_with_motion_drops_the_kept_read(params: dict[str, str]) -> None:
    module, printer, mcu = make_rfid()
    read_anycubic(mcu)
    name = "ACEK_RFID_READ" if "MOVE" in params else "ACEK_RFID_FORGET"
    printer.lookup_object("gcode").commands[name](FakeGcmd(params))
    assert rfid_of(module).kept.reads == {}
    module, printer, mcu = restarted(module, [0, 1, 0, 0])
    assert rfid_of(module).lane_status(1)["record"] is None


def test_a_read_lane_after_a_restart_takes_the_cache_not_the_kept_read() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 4, 0, 0], uid=UID4)
    # a UID the cache does not know is forgotten, and so is the kept read
    assert rfid_of(module).lane_status(1)["record"] is None and rfid_of(module).kept.reads == {}


def test_a_bad_lanes_file_or_entry_keeps_nothing_of_it(caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = make_rfid()
    path = rfid_of(module).kept.path
    good = base.SpoolRecord(format="uid", uid="01020304").as_dict()
    bodies = {
        "not json": {},
        "[1, 2]": {},
        json.dumps({"x": good, "9": good, "2": good}): {1: good},
        json.dumps({"1": {"format": "uid"}}): {},  # a field missing
        json.dumps({"1": dict(good, extra_key=1)}): {},  # a field too many
        json.dumps({"1": dict(good, uid=5)}): {},  # a uid that is not text
        json.dumps({"1": dict(good, weight_g=True)}): {},  # a bool is not a count
        json.dumps({"1": dict(good, diameter_mm=1)}): {0: dict(good, diameter_mm=1)},
    }
    with caplog.at_level("WARNING"):
        for body, reads in bodies.items():
            with open(path, "w") as f:
                f.write(body)
            assert KeptReads(path, 4, "ace2k", printer.reactor.monotonic).reads == reads
    assert "bad lane reads left out" in caplog.text


def test_a_lanes_file_that_cannot_be_written_keeps_memory_as_the_file_and_waits(
    caplog: pytest.LogCaptureFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    read_anycubic(mcu)
    assert rfid.kept.get(1) is not None
    real = ace2k_rfid_kept.os.replace
    calls: list[str] = []

    def refuse(src: str, dst: str) -> None:
        calls.append(dst)
        raise OSError("read-only")

    monkeypatch.setattr(ace2k_rfid_kept.os, "replace", refuse)
    module.lanes[1]["insert"] = False  # a live lane, not restored: its bay empties
    with caplog.at_level("WARNING"):
        for _ in range(5):  # five state reports, a second apart
            mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 1, 0, 0]), "field": 0})
            printer.reactor.now += 1.0
    assert len(calls) == 1  # one attempt, not one a second
    assert caplog.text.count("kept lane reads not written") == 1
    assert rfid.kept.get(1) is not None  # neither dropped: memory is what the file holds
    with open(rfid.kept.path) as f:
        assert "2" in json.load(f)
    monkeypatch.setattr(ace2k_rfid_kept.os, "replace", real)
    printer.reactor.now += ace2k_rfid_kept.RETRY_S
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([0, 1, 0, 0]), "field": 0})
    assert rfid.kept.reads == {}  # both dropped once the write goes through
    with open(rfid.kept.path) as f:
        assert json.load(f) == {}


def test_a_uid_only_read_the_unit_still_holds_is_restored_not_forgotten() -> None:
    before, _printer, mcu = make_rfid()
    tag_event(mcu, 1, 9, UID4, 0x0044, 0x00)  # no brand claims it: the UID alone, not cached
    module, printer, mcu = restarted(before, [0, 4, 0, 0], uid=UID4)
    tag = rfid_of(module).lane_status(1)
    assert tag["source"] == "restored" and tag["uid"] == UID4.hex().upper()
    assert [d for f, d in mcu.commands if f.startswith("ace2k_rfid_forget")] == []


def test_a_restore_sends_the_tag_read_event() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    events = [e for e in printer.events if e[0] == "ace2k:tag_read"]
    assert events == [("ace2k:tag_read", (1, dict(rfid_of(before).kept.get(1), unit=None)))]


def test_a_same_uid_session_that_gives_up_confirms_the_restored_read() -> None:
    before, _printer, mcu = make_rfid()
    read_anycubic(mcu)
    module, printer, mcu = restarted(before, [0, 1, 0, 0])
    rfid = rfid_of(module)
    rfid.cache.clear()  # so the give-up has no cached record to fall back on
    tag_event(mcu, 1, 9, UID7, 0x0044, 0x00)  # the same tag
    data(mcu, 1, 9, 4, b"", status=ace2k_rfid.STATUS_TAG_GONE)
    tag = rfid.lane_status(1)
    assert tag["source"] == "cached" and tag["record"]["format"] == "anycubic"
    rfid_event(mcu, 1, 5)()  # a no_tag after it does not drop the read the unit confirmed
    assert rfid.lane_status(1)["record"] is not None and rfid.kept.get(1) is not None


def test_the_seeding_writes_the_lanes_file_once(monkeypatch: pytest.MonkeyPatch) -> None:
    uids = [UID4, UID7]
    module, printer, mcu = make_rfid(
        {
            "ace2k_rfid_lane_query": [
                {"lane": i, "state": 4 if i < 2 else 0, "uid": uids[i] if i < 2 else b""}
                for i in range(4)
            ]
        }
    )
    rfid = rfid_of(module)
    for uid in uids:
        rfid.cache[uid.hex().upper()] = base.SpoolRecord(
            format="uid", uid=uid.hex().upper()
        ).as_dict()
    real = ace2k_rfid_kept.os.replace
    calls: list[str] = []

    def count(src: str, dst: str) -> None:
        calls.append(dst)
        real(src, dst)

    monkeypatch.setattr(ace2k_rfid_kept.os, "replace", count)
    fire_ready(printer)
    assert calls == [rfid.kept.path] and set(rfid.kept.reads) == {0, 1}
