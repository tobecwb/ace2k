"""ace2k_rfid.py: the config and init commands, the subscriptions, the event-driven session
against the brand modules, the G-codes, the cache, the seeding — with the fake printer of
test_ace2k_extras and synthetic UIDs."""

import json
import os
import re

import ace2k_feed
import ace2k_rfid
import pytest
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
from test_ace2k_tags import anycubic_image, bambu_blocks, ntag_image, sections

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

    def __init__(self, brand, params):
        self.brand = brand
        self.enabled = str(params.get("enabled", "True")).lower() == "true"
        self.params = {k: v for k, v in params.items() if k != "enabled"}


def rfid_of(module):
    return module.rfid


def make_rfid(responses=None, options=None, with_tags=True, **kwargs):
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


def steps_sent(mcu):
    return [data for fmt, data in mcu.commands if fmt.startswith("ace2k_rfid_step")]


def dones(mcu):
    return [data for fmt, data in mcu.commands if fmt.startswith("ace2k_rfid_done")]


def tag_event(mcu, lane, session, uid, atqa, sak):
    mcu.subscriptions["ace2k_rfid_tag"](
        {"lane": lane, "session": session, "uid": uid, "atqa": atqa, "sak": sak}
    )


def data(mcu, lane, session, block, payload, status=0):
    mcu.subscriptions["ace2k_rfid_data"](
        {"lane": lane, "session": session, "block": block, "status": status, "data": payload}
    )


def test_build_config_queries_subscribes_and_sends_the_search_length():
    module, printer, mcu = make_rfid(options={"rfid_search_mm": 700})
    init = [c for c, is_init in mcu.config_cmds if is_init]
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    assert any(c.startswith("ace2k_rfid_query rest_ticks=") for c in init)
    assert "ace2k_rfid_search_set search_um=700000" in cfg
    for name in ("ace2k_rfid_state", "ace2k_rfid_tag", "ace2k_rfid_data", "ace2k_rfid_event"):
        assert name in mcu.subscriptions
    assert rfid_of(module).present


def test_a_search_length_outside_its_range_is_a_config_error():
    with pytest.raises(Exception, match="rfid_search_mm"):
        make_rfid(options={"rfid_search_mm": 1500})


def test_an_ntag_session_reads_three_steps_and_ends_with_anycubics_record():
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
    assert ("ace2k:tag_read", (1, tag["record"])) in printer.events
    with open(rfid_of(module).cache_path) as f:
        cache = json.load(f)
    assert cache[UID7.hex().upper()]["format"] == "anycubic"


def test_a_mifare_session_tries_bambu_with_its_key_and_decodes():
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


def test_an_auth_failure_moves_to_the_next_brand_and_none_left_is_uid_only():
    module, printer, mcu = make_rfid()
    tag_event(mcu, 2, 9, UID4, 0x0004, 0x08)
    for _ in range(2):  # bambu, snapmaker (creality ships disabled): each fails its first sector
        step = steps_sent(mcu)[-1]
        data(mcu, 2, 9, step[3] * 4, b"", status=1)
    assert dones(mcu) == [[2, 9, 1]]
    tag = rfid_of(module).lane_status(2)
    assert tag["source"] == "uid" and tag["record"]["format"] == "uid"


def test_an_error_resends_the_step_once_and_a_second_gives_up_to_the_cache():
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


def test_a_step_with_no_answer_gives_up_and_the_cache_fills_in():
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


def test_a_tag_gone_frame_ends_the_session_without_done_and_stale_frames_are_ignored():
    module, printer, mcu = make_rfid()
    tag_event(mcu, 0, 5, UID4, 0x0004, 0x08)
    data(mcu, 0, 99, 1, bytes(16))  # another session's frame
    assert dones(mcu) == []
    data(mcu, 0, 5, 0, b"", status=2)
    assert dones(mcu) == []
    assert 0 not in rfid_of(module).sessions


def test_ace_rfid_read_names_a_refusal_and_waits_for_the_outcome():
    module, printer, mcu = make_rfid({"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 2}]})
    gcmd = FakeGcmd({"LANE": "1"})
    with pytest.raises(Exception, match="no_filament"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)

    def outcome(_eventtime):
        mcu.subscriptions["ace2k_rfid_event"]({"lane": 0, "kind": 6, "session": 0})

    # the waiter's completion receives the ambiguous outcome while it waits
    orig = printer.reactor.completion

    def completion():
        c = orig()
        printer.reactor.while_waiting[c] = [outcome]
        return c

    printer.reactor.completion = completion
    gcmd = FakeGcmd({"LANE": "1"})
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert any("ambiguous" in line for line in gcmd.lines)
    assert rfid.lanes[0]["state"] == "pending" or rfid.lanes[0]["state"] is None


def test_dump_writes_the_sessions_bytes_to_the_log_directory():
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


def test_the_ready_seeding_takes_the_cache_or_forgets():
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


def test_status_lines_and_get_status_carry_the_tag():
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


def test_without_the_rfid_firmware_the_gcodes_answer_an_error():
    absent = {name: f"Unknown command: {name}" for name in RFID_KNOWN_EXTRA}
    module, printer, mcu = make(dict(RESPONSES), lookup_errors=absent)
    mcu.config_callback()
    rfid = rfid_of(module)
    assert not rfid.present
    assert "ace2k_rfid_tag" not in mcu.subscriptions
    with pytest.raises(Exception, match="CONFIG_ACE2K_RFID_READ"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](FakeGcmd({"LANE": "1"}))
    assert registry.BRANDS  # the brands import without the firmware


def test_each_brand_tries_its_own_key_after_another_brands_refusal():
    module, printer, mcu = make_rfid()
    tag_event(mcu, 2, 10, UID4, 0x0004, 0x08)
    bambu = steps_sent(mcu)[-1]
    data(mcu, 2, 10, 0, b"", status=ace2k_rfid.STATUS_AUTH_FAILED)
    snapmaker = steps_sent(mcu)[-1]
    # the same sector, the next brand's key: a refusal is the key's, not the sector's
    assert (snapmaker[3], bambu[3]) == (0, 0) and snapmaker[5] != bambu[5]
    session = rfid_of(module).sessions[2]
    assert session.auth_failed == {(0, 0)} and not session.got.auth_failed


def test_the_field_byte_names_the_dead_readers():
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    assert module.get_status(0)["rfid_readers"] is None
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([1, 0, 0, 4]), "field": 0x21})
    assert module.get_status(0)["rfid_readers"] == ["field_a", "dead_b"]
    assert rfid.status_lines()[-1] == "rfid readers: field_a, dead_b"
    assert rfid.lane_status(3)["state"] == "read"


def queue_outcome(printer, mcu, *events):
    """The state events the reactor delivers while ACE_RFID_READ waits."""
    orig = printer.reactor.completion

    def completion():
        c = orig()
        printer.reactor.while_waiting[c] = [
            (lambda _e, ev=ev: mcu.subscriptions["ace2k_rfid_event"](ev)) for ev in events
        ]
        return c

    printer.reactor.completion = completion


def test_a_read_on_command_ends_on_the_lanes_unchanged_state_and_names_a_dead_reader():
    module, printer, mcu = make_rfid()
    mcu.subscriptions["ace2k_rfid_state"]({"state": bytes([1, 1, 0, 0]), "field": 0x20})
    queue_outcome(printer, mcu, {"lane": 1, "kind": 1, "session": 0})
    gcmd = FakeGcmd({"LANE": "2"})
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert gcmd.lines == ["ace2k: lane 2 tag pending — no record (reader B dead)"]


def queue_calls(printer, *calls):
    """What the reactor delivers while ACE_RFID_READ waits: callables run in order."""
    orig = printer.reactor.completion

    def completion():
        c = orig()
        printer.reactor.while_waiting[c] = [(lambda _e, f=f: f()) for f in calls]
        return c

    printer.reactor.completion = completion


def feed_event(module, lane, kind, seq=0, mode="loading"):
    """ace2k_feed.py's Klipper event, as the printer dispatches it to ace2k_rfid."""
    event = dict(kind=kind, mode=mode, motor_mm=0.0, filament_mm=0.0, seq=seq)
    return lambda: rfid_of(module)._feed_event(lane, event)


def rfid_event(mcu, lane, kind):
    return lambda: mcu.subscriptions["ace2k_rfid_event"]({"lane": lane, "kind": kind, "session": 0})


def test_a_read_with_motion_on_a_read_lane_waits_through_the_search_for_the_new_outcome():
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
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert gcmd.lines == [f"ace2k: lane 1 tag read — {base.SpoolRecord(**new_record).summary()}"]
    assert rfid.move_waiters == {}
    # the outcome is asked for once the load has ended, after the read command
    names = [name for name, _data in mcu.queries]
    assert names[-2:] == ["ace2k_rfid_read", "ace2k_rfid_lane_query"]
    assert mcu.queries[-1][1] == [0]


def test_a_read_with_motion_forgets_the_old_record_on_acceptance():
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="read", uid="0A0B0C0D", source="cached", record={"format": "uid"})
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1", "WAIT": "0"})
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert rfid.lane_status(0)["record"] is None and rfid.lanes[0]["uid"] is None


def test_a_read_with_motion_whose_load_stopped_names_it_with_the_queried_state():
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_event(module, 0, "stopped"))
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert gcmd.lines == ["ace2k: lane 1 tag search stopped; ace2k: lane 1 tag pending — no record"]
    assert rfid_of(module).lanes[0]["state"] == "pending"


def test_a_read_with_motion_whose_load_failed_is_an_error_naming_the_kind():
    module, printer, mcu = make_rfid({"ace2k_rfid_lane_query": {"lane": 0, "state": 1, "uid": b""}})
    queue_calls(printer, feed_event(module, 0, "tangled"))
    with pytest.raises(Exception, match="tag search failed: tangled"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1"})
        )


def test_a_read_with_motion_refused_busy_waits_for_nothing():
    module, printer, mcu = make_rfid({"ace2k_rfid_read": {"lane": 0, "accepted": 0, "reason": 1}})
    rfid = rfid_of(module)
    rfid.lanes[0].update(state="reading", uid="0A0B0C0D", source="uid", record={"format": "uid"})
    with pytest.raises(Exception, match="refused: busy"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](
            FakeGcmd({"LANE": "1", "MOVE": "1", "DUMP": "1"})
        )
    assert rfid.move_waiters == {} and rfid.dump_next == set()
    assert rfid.lanes[0]["record"] == {"format": "uid"}  # a refusal changes nothing


def test_a_dump_asked_with_motion_survives_the_forgets_pending_and_reaches_the_session():
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
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](
        FakeGcmd({"LANE": "1", "MOVE": "1", "DUMP": "1"})
    )
    assert rfid.sessions[0].dump
    assert rfid.dump_next == set()


def test_a_read_with_no_outcome_times_out_with_an_error():
    module, printer, mcu = make_rfid()
    with pytest.raises(Exception, match="no tag outcome"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](FakeGcmd({"LANE": "1"}))
    assert rfid_of(module).waiters == {}


def test_forget_sends_the_command_and_clears_the_record():
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    rfid.lanes[0].update(uid="01020304", source="cached", record={"format": "uid"})
    printer.lookup_object("gcode").commands["ACE_RFID_FORGET"](FakeGcmd({"LANE": "1"}))
    assert [d for f, d in mcu.commands if f.startswith("ace2k_rfid_forget")] == [[0]]
    assert rfid.lane_status(0)["record"] is None


def test_a_lost_lane_answer_at_ready_leaves_that_lane_unknown(caplog):
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
def test_every_format_is_checked_at_connect(fmt):
    name = fmt.split()[0]
    with pytest.raises(Exception, match=name):
        make_rfid(formats={name: name + " other=%c"})


def test_ntag_pages_that_refuse_twice_end_with_the_uid_alone():
    module, printer, mcu = make_rfid()
    tag_event(mcu, 1, 11, UID7, 0x0044, 0x00)
    data(mcu, 1, 11, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    data(mcu, 1, 11, 4, b"", status=ace2k_rfid.STATUS_ERROR)
    assert len(steps_sent(mcu)) == 2
    assert dones(mcu) == [[1, 11, 1]]
    tag = rfid_of(module).lane_status(1)
    assert tag["source"] == "uid" and tag["record"]["uid"] == UID7.hex().upper()


def test_a_dump_request_is_dropped_on_refusal_and_on_timeout():
    module, printer, mcu = make_rfid({"ace2k_rfid_read": [{"lane": 0, "accepted": 0, "reason": 1}]})
    with pytest.raises(Exception, match="busy"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](
            FakeGcmd({"LANE": "1", "DUMP": "1"})
        )
    assert rfid_of(module).dump_next == set()
    module, printer, mcu = make_rfid()
    with pytest.raises(Exception, match="no tag outcome"):
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](
            FakeGcmd({"LANE": "1", "DUMP": "1"})
        )
    assert rfid_of(module).dump_next == set()


def test_ntag_pages_that_refuse_twice_keep_a_cached_record():
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


def test_a_dump_request_is_dropped_when_the_wait_ends_without_a_session():
    module, printer, mcu = make_rfid()
    # a dead reader: the lane's unchanged state, at once, and no session
    queue_outcome(printer, mcu, {"lane": 0, "kind": 1, "session": 0})
    printer.lookup_object("gcode").commands["ACE_RFID_READ"](FakeGcmd({"LANE": "1", "DUMP": "1"}))
    assert rfid_of(module).dump_next == set()


def test_the_move_wait_takes_the_firmwares_search_default():
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_SEARCH_DEFAULT_UM=800000)
    module, printer, mcu = make_rfid(constants=constants)
    assert rfid_of(module)._search_mm() == 800.0
    module, printer, mcu = make_rfid()  # no constant: 750
    assert rfid_of(module)._search_mm() == 750.0
    module, printer, mcu = make_rfid(options={"rfid_search_mm": 700}, constants=constants)
    assert rfid_of(module)._search_mm() == 700.0  # printer.cfg wins


def test_the_move_wait_follows_the_feeds_load_speed():
    module, printer, mcu = make_rfid(options={"load_speed": 10})
    gcmd = FakeGcmd({"LANE": "1", "MOVE": "1"})
    with pytest.raises(Exception, match="no tag outcome") as err:
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    waited = float(re.search(r"within (\d+) s", str(err.value)).group(1))
    assert waited >= 2 * 750.0 / 10  # out and back at 10 mm/s: 150 s of travel alone
    feed = printer.lookup_object("ace2k_feed")
    assert waited == round(feed.load_wait_s(2 * 750.0 + ace2k_feed.SESSION_ALLOWANCE_MM))
    feed.load_speed = 60.0  # an ACE_LOAD_SET since: the next wait follows it
    with pytest.raises(Exception, match="no tag outcome") as err:
        printer.lookup_object("gcode").commands["ACE_RFID_READ"](gcmd)
    assert float(re.search(r"within (\d+) s", str(err.value)).group(1)) < waited


def test_ready_only_schedules_the_seeding_and_the_timer_applies_it():
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


def test_the_fake_refuses_a_blocking_query_inside_a_ready_handler():
    module, printer, mcu = make_rfid()
    rfid = rfid_of(module)
    printer.reactor.pause_disabled = True  # as while klippy:ready handlers run
    with pytest.raises(RuntimeError, match="reactor pause disabled"):
        rfid._seed_lanes(printer.reactor.now)
