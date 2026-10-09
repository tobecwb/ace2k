"""ace2k_dryer.py: the presence checks per image, the G-codes and their refusals, the reports,
the events, the log and its seeding — against the fake printer of test_ace2k_extras."""

import pytest
from test_ace2k_extras import (
    KNOWN_ALL_BUT_HEALTH,
    RESPONSES,
    FakeCommandError,
    FakeGcmd,
    FakeReactor,
    make,
)

AIRFLOW_NAMES = {
    "ace2k_fan_set",
    "ace2k_flap_pulse",
    "ace2k_airflow_response",
    "ace2k_airflow_query",
    "ace2k_airflow_state",
}
DRYER_NAMES = {
    "ace2k_dryer_start",
    "ace2k_dryer_stop",
    "ace2k_dryer_clear",
    "ace2k_dryer_response",
    "ace2k_dryer_query",
    "ace2k_dryer_state",
    "ace2k_dryer_event",
    "ace2k_dryer_event_ack",
    "ace2k_dryer_log_query",
    "ace2k_dryer_log",
}
RELEASE = set(KNOWN_ALL_BUT_HEALTH) | AIRFLOW_NAMES | DRYER_NAMES
# an image with the fans and flaps (CONFIG_ACE2K_HEAT) and without the dryer
NO_DRYER = set(KNOWN_ALL_BUT_HEALTH) | AIRFLOW_NAMES
EMPTY_LOG = {"index": 1, "a": 0, "b": 0, "c": 0, "d": 0, "e": 0, "f": 0}
OK = {"op": 0, "accepted": 1, "reason": 0}
DRYER_RESPONSES = dict(
    RESPONSES,
    ace2k_dryer_start=OK,
    ace2k_dryer_stop=dict(OK, op=1),
    ace2k_dryer_clear=dict(OK, op=2),
    ace2k_fan_set={"op": 0, "accepted": 1, "reason": 0},
    ace2k_flap_pulse={"op": 1, "accepted": 1, "reason": 0},
)


def dryer_of(printer):
    return printer.objects["ace2k_dryer"]


def build(known=None, responses=None, options=None, formats=None):
    module, printer, mcu = make(
        dict(responses or DRYER_RESPONSES), options=options, known=known, formats=formats
    )
    mcu.config_callback()
    return module, printer, mcu, dryer_of(printer)


def gcode(printer, name, **params):
    gcmd = FakeGcmd({k: str(v) for k, v in params.items()})
    printer.objects["gcode"].commands[name](gcmd)
    return gcmd


def sent_to(mcu, name):
    return [data for fmt, data in mcu.queries if fmt == name]


def state_frame(
    state=0,
    target=0,
    drive_dc=0,
    duty=0,
    remaining=0,
    fans=0,
    flaps=0,
    fault=0,
    notices=0,
    lost=0,
    oldest=0,
):
    return {
        "state": state,
        "target_c": target,
        "drive_dc": drive_dc,
        "duty": duty,
        "remaining_min": remaining,
        "fans": fans,
        "flaps": flaps,
        "fault": fault,
        "notices": notices,
        "lost": lost,
        "oldest": oldest,
    }


def test_a_release_dictionary_starts_the_two_reports():
    module, printer, mcu, dryer = build(known=RELEASE)
    assert dryer.airflow_present and dryer.dryer_present
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert "ace2k_airflow_query rest_ticks=120000000" in init
    # always sent: the unit holds its events until this query
    assert "ace2k_dryer_query rest_ticks=120000000" in init
    assert not any(c.startswith("ace2k_heat_") for c in init)
    assert {"ace2k_airflow_state", "ace2k_dryer_state", "ace2k_dryer_event"} <= set(
        mcu.subscriptions
    )
    assert not any(n.startswith("ace2k_heat_") for n in mcu.subscriptions)


def test_an_image_without_the_dryer_keeps_fans_and_flaps_and_refuses_ace_dry(caplog):
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu, dryer = build(known=NO_DRYER)
    assert dryer.airflow_present and not dryer.dryer_present
    assert "CONFIG_ACE2K_DRYER" in caplog.text
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_DRYER"):
        gcode(printer, "ACE_DRY", TEMP=55, DURATION=60)
    gcode(printer, "ACE_FAN", ON=1, SECONDS=10)
    assert sent_to(mcu, "ace2k_fan_set") == [[1, 10]]
    gcode(printer, "ACE_FLAP", WHICH="rear", OPEN=1)
    assert sent_to(mcu, "ace2k_flap_pulse") == [[1, 1]]
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert "ace2k_airflow_query rest_ticks=120000000" in init
    assert "ace2k_dryer_query rest_ticks=120000000" not in init


def test_a_firmware_without_the_heat_disables_every_gcode_here(caplog):
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu, dryer = build(known=set(KNOWN_ALL_BUT_HEALTH))
    assert not (dryer.airflow_present or dryer.dryer_present)
    assert "CONFIG_ACE2K_HEAT" in caplog.text
    for name, params in (
        ("ACE_DRY", {"TEMP": 55, "DURATION": 60}),
        ("ACE_DRY_STOP", {}),
        ("ACE_DRY_CLEAR", {}),
        ("ACE_FAN", {"ON": 1}),
        ("ACE_FLAP", {"WHICH": "rear", "OPEN": 1}),
        ("ACE_DRYER_LOG", {}),
    ):
        with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_HEAT"):
            gcode(printer, name, **params)
    assert not any(c.startswith(("ace2k_airflow_", "ace2k_dryer_")) for c, _ in mcu.config_cmds)
    assert mcu.queries == []
    gcmd = gcode(printer, "ACE_STATUS")
    assert "dryer:" not in gcmd.lines[0] and "airflow:" not in gcmd.lines[0]


def test_a_dryer_format_from_another_commit_is_a_config_error():
    old = "ace2k_dryer_state state=%c target_c=%c duty=%c"
    with pytest.raises(Exception, match="ace2k_dryer_state.*different commits"):
        build(formats={"ace2k_dryer_state": old})
    old_fan = "ace2k_fan_set on=%c"
    with pytest.raises(Exception, match="ace2k_fan_set has another format.*different commits"):
        build(formats={"ace2k_fan_set": old_fan})
    old_flap = "ace2k_flap_pulse flap=%c"
    with pytest.raises(Exception, match="ace2k_flap_pulse.*different commits"):
        build(formats={"ace2k_flap_pulse": old_flap})
    # a dryer binding missing one of its names beside the others: another commit too
    with pytest.raises(Exception, match="ace2k_dryer_log_query.*though ace2k_dryer_start"):
        build(known=RELEASE - {"ace2k_dryer_log_query"})


def test_the_v0_5_0_event_and_state_formats_are_a_config_error():
    with pytest.raises(Exception, match="ace2k_dryer_event.*different commits"):
        build(formats={"ace2k_dryer_event": "ace2k_dryer_event kind=%c arg=%c"})
    old_state = (
        "ace2k_dryer_state state=%c target_c=%c drive_dc=%hu duty=%c remaining_min=%hu fans=%c"
        " flaps=%c fault=%c notices=%c"
    )
    with pytest.raises(Exception, match="ace2k_dryer_state.*different commits"):
        build(formats={"ace2k_dryer_state": old_state})
    # an earlier v0.6.0 candidate's state frame, without oldest
    with pytest.raises(Exception, match="ace2k_dryer_state.*different commits"):
        build(formats={"ace2k_dryer_state": old_state + " lost=%hu"})
    # a v0.5.0 dictionary has no acknowledgement at all
    with pytest.raises(Exception, match="ace2k_dryer_event_ack.*though ace2k_dryer_start"):
        build(known=RELEASE - {"ace2k_dryer_event_ack"})


@pytest.mark.parametrize(
    "text, minutes",
    [("4h", 240), ("90m", 90), ("1.5h", 90), ("45", 45), ("24h", 1440), (" 2H ", 120)],
)
def test_ace_dry_sends_the_temperature_and_the_minutes(text, minutes):
    module, printer, mcu, dryer = build(known=RELEASE)
    gcmd = gcode(printer, "ACE_DRY", TEMP=55, DURATION=text)
    assert sent_to(mcu, "ace2k_dryer_start") == [[55, minutes]]
    assert "drying at 55 °C" in gcmd.lines[-1]


@pytest.mark.parametrize(
    "params, words",
    [
        ({"TEMP": 55, "DURATION": "0"}, "DURATION"),
        ({"TEMP": 55, "DURATION": "1441"}, "DURATION"),
        ({"TEMP": 55, "DURATION": "25h"}, "DURATION"),
        ({"TEMP": 55, "DURATION": "abc"}, "DURATION"),
        ({"TEMP": 55, "DURATION": "nan"}, "DURATION"),
        ({"TEMP": 55, "DURATION": "infh"}, "DURATION"),
        ({"TEMP": 55}, "DURATION"),
        ({"DURATION": "60"}, "TEMP"),
        ({"TEMP": 70, "DURATION": "60"}, "TEMP"),
        ({"TEMP": 14, "DURATION": "60"}, "TEMP"),
    ],
)
def test_ace_dry_refuses_bad_arguments_before_it_sends(params, words):
    module, printer, mcu, dryer = build(known=RELEASE)
    with pytest.raises(FakeGcmd.error, match=words):
        gcode(printer, "ACE_DRY", **params)
    assert sent_to(mcu, "ace2k_dryer_start") == []


@pytest.mark.parametrize(
    "reason, words",
    [
        (1, "held by a manual run"),
        (2, "out of range"),
        (3, "fault"),
        (4, "sensor"),
        (5, "mains"),
        (6, "power-cycle the unit"),
        (7, "cooling down"),
        (8, "Klipper shutdown — FIRMWARE_RESTART"),
        (9, "the mains has just come back — its frequency is being measured; try ACE_DRY again"),
        (99, "reason 99"),
    ],
)
def test_each_dryer_refusal_is_named(reason, words):
    responses = dict(DRYER_RESPONSES, ace2k_dryer_start={"op": 0, "accepted": 0, "reason": reason})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match=words):
        gcode(printer, "ACE_DRY", TEMP=55, DURATION=60)


def test_stop_and_clear_are_sent_and_a_refused_clear_is_named():
    module, printer, mcu, dryer = build(known=RELEASE)
    gcmd = gcode(printer, "ACE_DRY_STOP")
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]
    assert "cools the heater down" in gcmd.lines[-1]
    gcode(printer, "ACE_DRY_CLEAR")
    assert sent_to(mcu, "ace2k_dryer_clear") == [[]]
    responses = dict(DRYER_RESPONSES, ace2k_dryer_clear={"op": 2, "accepted": 0, "reason": 7})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="cooling down"):
        gcode(printer, "ACE_DRY_CLEAR")
    responses = dict(DRYER_RESPONSES, ace2k_dryer_clear={"op": 2, "accepted": 0, "reason": 6})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="power-cycle the unit"):
        gcode(printer, "ACE_DRY_CLEAR")


def printer_error():
    return FakeCommandError("Timeout on wait for 'ace2k_dryer_response' response")


def test_an_unanswered_request_is_a_gcode_error_not_a_crash():
    responses = dict(DRYER_RESPONSES, ace2k_dryer_stop=[printer_error()])
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="no answer from the unit"):
        gcode(printer, "ACE_DRY_STOP")


def test_the_state_report_lands_in_the_status_and_ace_status():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.config_callback()  # a reconnect
    mcu.subscriptions["ace2k_dryer_state"](
        state_frame(state=2, target=55, drive_dc=620, duty=40, remaining=192, fans=3, flaps=0b1010)
    )
    st = module.get_status(0.0)["dryer"]
    assert st["state"] == "heating" and st["target"] == 55 and st["drive"] == 62.0
    assert (st["duty"], st["remaining"]) == (40, 192)
    assert st["fans"] == {"left": True, "right": True}
    assert st["flaps"] == {"bottom": "closed", "rear": "closed"}
    assert (st["fault"], st["notices"]) == (None, [])
    gcmd = gcode(printer, "ACE_STATUS")
    assert "dryer: heating 55 °C (drive 62.0 °C, duty 40 %), 3 h 12 min left" in gcmd.lines[0]


def test_the_flaps_fault_is_named_apart_from_the_fans():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=4, target=55, fault=11))
    assert dryer.status()["fault"] == "flaps"
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=4, target=55, fault=10))
    assert dryer.status()["fault"] == "fans"


def test_a_fault_the_notices_and_an_unknown_flap_are_named():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](
        state_frame(state=4, target=65, fans=1, flaps=0b0001, fault=4, notices=0x3)
    )
    st = dryer.status()
    assert st["state"] == "fault" and st["fault"] == "chamber_over"
    assert st["notices"] == ["hot_ambient", "lowered"]
    assert st["fans"] == {"left": True, "right": False}
    assert st["flaps"] == {"bottom": "open", "rear": "unknown"}
    mcu.subscriptions["ace2k_dryer_state"](state_frame())
    st = dryer.status()
    assert (st["state"], st["target"], st["drive"]) == ("idle", None, None)


def test_a_mains_dip_ridden_out_is_named():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=55, notices=0x8))
    assert dryer.status()["notices"] == ["mains_dip"]
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=55, notices=0x9))
    assert dryer.status()["notices"] == ["hot_ambient", "mains_dip"]


def test_edges_off_phase_is_named():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=55, notices=0x10))
    assert dryer.status()["notices"] == ["edges_off_phase"]
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=55, notices=0x18))
    assert dryer.status()["notices"] == ["mains_dip", "edges_off_phase"]


def test_a_log_that_cannot_be_stored_is_a_notice_and_a_warning(caplog):
    module, printer, mcu, dryer = build(known=RELEASE)
    with caplog.at_level("WARNING", logger="root"):
        mcu.subscriptions["ace2k_dryer_state"](state_frame(notices=0x4))
        mcu.subscriptions["ace2k_dryer_state"](state_frame(notices=0x4))
    assert dryer.status()["notices"] == ["log_unstored"]
    assert caplog.text.count("cannot store its log") == 1  # once, on the change
    text = "\n".join(dryer.status_lines())
    assert "dryer: idle; notices log_unstored" in text
    assert "!! dryer: the unit cannot store its log" in text
    mcu.subscriptions["ace2k_dryer_state"](state_frame())
    assert "cannot store" not in "\n".join(dryer.status_lines())


def test_an_unknown_notice_bit_is_kept_by_number_not_an_error():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, notices=0x1 | 0x40 | 0x80))
    assert dryer.status()["notices"] == ["hot_ambient", "notice_0x40", "notice_0x80"]


def test_the_airflow_report_names_the_owner_and_the_read_back():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_airflow_state"](
        {"fans_cmd": 1, "fans_read": 2, "owner": 3, "flaps": 0b0110}
    )
    st = dryer.status()
    assert st["owner"] == "dryer" and st["fans_commanded"] is True
    assert st["fans"] == {"left": False, "right": True}
    assert st["flaps"] == {"bottom": "closed", "rear": "open"}
    text = "\n".join(dryer.status_lines())
    assert "fans commanded on, read L off R on (owner dryer)" in text
    mcu.subscriptions["ace2k_airflow_state"](
        {"fans_cmd": 0, "fans_read": 0, "owner": 1, "flaps": 0}
    )
    assert dryer.status()["owner"] == "manual"


def _connected(mcu, oldest=0):
    """The first state report after a connect: the unit's oldest unacknowledged event."""
    mcu.subscriptions["ace2k_dryer_state"](state_frame(oldest=oldest))


def test_every_event_is_a_klipper_event_and_a_log_line(caplog):
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 0)
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 2, "arg": 8, "seq": 0})
        # the unit's lane, 1-based
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 3, "arg": 2, "seq": 1})
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 0, "arg": 0, "seq": 2})
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 42, "arg": 5, "seq": 3})
    events = [params[0] for name, params in printer.events if name == "ace2k:dryer"]
    assert events[0] == {"kind": "fault", "arg": 8, "reason": "cutout"}
    assert events[1] == {"kind": "lowered", "arg": 2, "lane": 2}
    assert events[2] == {"kind": "done", "arg": 0}
    assert events[3] == {"kind": 42, "arg": 5}
    assert "ace2k dryer: fault cutout" in caplog.text and "power-cycle" in caplog.text
    assert "lane 2" in caplog.text and "45 °C" in caplog.text
    assert "ace2k dryer: cycle done" in caplog.text


ACK = "ace2k_dryer_event_ack seq=%c"


def test_every_event_is_acknowledged_and_a_resend_is_handled_once(caplog):
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 7)
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 0, "arg": 0, "seq": 7})
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 0, "arg": 0, "seq": 7})  # a resend
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 1, "arg": 0, "seq": 8})
    acks = [data for fmt, data in mcu.commands if fmt == ACK]
    assert acks == [[7], [7], [8]]
    events = [p[0] for name, p in printer.events if name == "ace2k:dryer"]
    assert [e["kind"] for e in events] == ["done", "stopped"]
    assert caplog.text.count("cycle done") == 1


def _event(mcu, seq, kind=6):
    mcu.subscriptions["ace2k_dryer_event"]({"kind": kind, "arg": 0, "seq": seq})


def _acks(mcu):
    return [d[0] for fmt, d in mcu.commands if fmt == ACK]


def _handled(printer):
    return [p[0] for name, p in printer.events if name == "ace2k:dryer"]


def _queries(mcu):
    return [d for fmt, d in mcu.commands if fmt == "ace2k_dryer_query rest_ticks=%u"]


def test_a_lost_event_holds_the_acknowledgement_until_its_resend_arrives():
    """The unit's acknowledgement is cumulative: acknowledging 3 with 2 lost would drop 2."""
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 1)
    _event(mcu, 1, kind=0)
    _event(mcu, 3, kind=1)  # 2 (a fault) lost on the way
    assert _acks(mcu) == [1, 1]
    assert [e["kind"] for e in _handled(printer)] == ["done"]
    # the unit resends from its oldest unacknowledged: 2, then 3
    _event(mcu, 2, kind=2)
    _event(mcu, 3, kind=1)
    assert _acks(mcu) == [1, 1, 2, 3]
    assert [e["kind"] for e in _handled(printer)] == ["done", "fault", "stopped"]
    # a later resend of all three handles nothing again
    for seq in (1, 2, 3):
        _event(mcu, seq)
    assert len(_handled(printer)) == 3
    assert _acks(mcu)[-3:] == [3, 3, 3]


def test_the_sequence_is_contiguous_across_the_wrap():
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 254)
    for seq in (254, 255, 0, 1):
        _event(mcu, seq)
    assert _acks(mcu) == [254, 255, 0, 1]
    assert len(_handled(printer)) == 4
    _event(mcu, 3)  # 2 lost
    assert _acks(mcu)[-1] == 1
    assert len(_handled(printer)) == 4


def test_events_before_the_first_state_report_are_held_then_handled_from_its_base():
    """After a connect the base is the first report's oldest: an event before it is neither
    handled nor acknowledged, then taken in order from the base — a fresh one that arrived ahead
    of the resend included."""
    module, printer, mcu, dryer = build(known=RELEASE)
    _event(mcu, 12, kind=1)  # fresh, ahead of the init query's resend
    _event(mcu, 9, kind=0)
    _event(mcu, 10, kind=2)
    assert _acks(mcu) == [] and _handled(printer) == []
    _connected(mcu, 9)
    assert [e["kind"] for e in _handled(printer)] == ["done", "fault"]
    _event(mcu, 11, kind=6)
    _event(mcu, 12, kind=1)
    assert [e["kind"] for e in _handled(printer)] == ["done", "fault", "vented", "stopped"]
    for seq in (9, 10, 11, 12):  # a later resend handles nothing again
        _event(mcu, seq)
    assert len(_handled(printer)) == 4
    assert _acks(mcu)[-1] == 12


def test_a_lost_first_resend_frame_is_still_handled():
    """The resend's first frame lost on the wire: the base is the unit's, so the next one is a
    gap — a query asks the resend, and the lost event is handled, never acknowledged unhandled."""
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 20)
    _event(mcu, 21, kind=0)  # 20 lost
    assert _handled(printer) == [] and _acks(mcu) == [19]
    assert len(_queries(mcu)) == 1
    _event(mcu, 20, kind=2)
    _event(mcu, 21, kind=0)
    assert [e["kind"] for e in _handled(printer)] == ["fault", "done"]
    assert _acks(mcu)[-2:] == [20, 21]


def test_a_host_restart_without_a_unit_reset_takes_the_base_from_the_unit():
    """The unit went on counting: its first report after the restart names 40, the oldest the old
    host had not acknowledged; 38 and 39 (acknowledged) are never handled again."""
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 40)
    _event(mcu, 39)  # a late frame from before: behind the base
    _event(mcu, 40)
    _event(mcu, 41)
    assert len(_handled(printer)) == 2
    assert _acks(mcu) == [39, 40, 41]


def test_the_base_is_taken_from_the_first_report_only():
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 5)
    _event(mcu, 5)
    _connected(mcu, 6)  # a later report changes nothing
    _event(mcu, 5)  # a resend: behind
    _event(mcu, 6)
    assert len(_handled(printer)) == 2


def test_a_gap_within_a_connection_asks_the_resend_at_once_and_once():
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 5)
    _event(mcu, 5)
    assert _queries(mcu) == []
    _event(mcu, 7)  # 6 lost
    _event(mcu, 8)  # the same gap: not asked again
    ticks = mcu.seconds_to_clock(1.0)
    assert _queries(mcu) == [[ticks]]
    for seq in (6, 7, 8):  # the resend the query asked for
        _event(mcu, seq)
    assert _acks(mcu)[-3:] == [6, 7, 8]
    assert len(_handled(printer)) == 4
    _event(mcu, 10)  # a new gap: asked again
    assert _queries(mcu) == [[ticks], [ticks]]
    _event(mcu, 4)  # behind: no query
    assert len(_queries(mcu)) == 2


def test_the_acknowledgement_and_the_gap_query_are_sent_from_the_reactor():
    """The event handler runs in the serial thread: its commands go out from the reactor, through
    register_async_callback, never sent from the handler itself."""
    module, printer, mcu, dryer = build(known=RELEASE)
    _connected(mcu, 5)
    deferred = []
    printer.reactor.register_async_callback = lambda cb, waketime=None: deferred.append(cb)
    _event(mcu, 5)
    _event(mcu, 7)  # a gap: an acknowledgement and a query
    assert _acks(mcu) == [] and _queries(mcu) == []
    for cb in deferred:
        cb(0.0)
    assert _acks(mcu) == [5, 5]
    assert len(_queries(mcu)) == 1


def test_a_reconnect_forgets_the_sequences_so_the_boot_cutout_is_handled(caplog):
    """The base is forgotten at the config callback, which Klipper runs at every connect (a host
    restart over a unit already configured included) — the module's only call site."""
    module, printer, mcu, dryer = build(known=RELEASE)
    assert dryer._forget_events not in printer.handler_lists.get("klippy:mcu_identify", [])
    _connected(mcu, 0)
    for seq in range(4):
        _event(mcu, seq)
    # the unit rebooted: its sequence restarts at 0, and its boot's cutout is seq 0
    mcu.config_callback()
    with caplog.at_level("WARNING", logger="root"):
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 2, "arg": 8, "seq": 0})
        _connected(mcu, 0)
    events = [p[0] for name, p in printer.events if name == "ace2k:dryer"]
    assert events[-1] == {"kind": "fault", "arg": 8, "reason": "cutout"}
    assert "fault cutout" in caplog.text
    assert [d for fmt, d in mcu.commands if fmt == ACK][-1] == [0]


def test_lost_events_are_shown_only_when_there_are_some(caplog):
    module, printer, mcu, dryer = build(known=RELEASE)
    assert dryer.status()["events_lost"] == 0
    mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=0))
    with caplog.at_level("WARNING", logger="root"):
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=3))
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=3))
    assert caplog.text.count("3 events lost") == 1  # once, as it grows
    assert dryer.status()["events_lost"] == 3
    assert any("3 dryer events lost" in line for line in dryer.status_lines())
    mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=0))
    assert dryer.status()["events_lost"] == 0
    assert not any("events lost" in line for line in dryer.status_lines())


def _lost_lines(caplog, level):
    return [
        r.getMessage()
        for r in caplog.records
        if r.levelname == level and "events lost" in r.getMessage()
    ]


@pytest.mark.parametrize("reconnect", [False, True])
def test_the_lost_count_at_a_connect_is_information_once_not_a_warning(reconnect, caplog):
    """The unit counts since its boot: after a host restart (a new session, or a reconnect) the
    first report's count is stated once as information; only a count that grows is a warning."""
    module, printer, mcu, dryer = build(known=RELEASE)
    if reconnect:
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=2))
        mcu.config_callback()
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=4))
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=4))
    assert _lost_lines(caplog, "WARNING") == []
    assert _lost_lines(caplog, "INFO") == ["ace2k dryer: 4 events lost since the unit started"]
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_dryer_state"](state_frame(lost=5))
    assert _lost_lines(caplog, "WARNING") == [
        "ace2k dryer: 5 events lost — the unit's queue was full before the host acknowledged them"
    ]


def test_ambient_above_and_measuring_are_named():
    module, printer, mcu, dryer = build(known=RELEASE)
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=15, notices=0x20))
    assert dryer.status()["notices"] == ["ambient_above"]
    text = "\n".join(dryer.status_lines())
    assert "notices ambient_above" in text
    assert "the chamber started above the target: nothing heats until it falls below" in text
    assert "judged as the cycle started, not a live reading" in text
    mcu.subscriptions["ace2k_dryer_state"](state_frame(state=2, target=15))
    assert "started above" not in "\n".join(dryer.status_lines())
    responses = dict(DRYER_RESPONSES, ace2k_dryer_start={"op": 0, "accepted": 0, "reason": 9})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="being measured"):
        gcode(printer, "ACE_DRY", TEMP=55, DURATION=60)


def test_ace_fan_and_ace_flap_send_and_name_a_refusal():
    module, printer, mcu, dryer = build(known=RELEASE)
    gcode(printer, "ACE_FAN", ON=1, SECONDS=30)
    gcode(printer, "ACE_FAN", ON=1)
    gcode(printer, "ACE_FAN", ON=0)
    assert sent_to(mcu, "ace2k_fan_set") == [[1, 30], [1, 60], [0, 0]]
    gcode(printer, "ACE_FLAP", WHICH="rear", OPEN=1)
    gcode(printer, "ACE_FLAP", WHICH="BOTTOM", OPEN=0)
    assert sent_to(mcu, "ace2k_flap_pulse") == [[1, 1], [0, 0]]
    with pytest.raises(FakeGcmd.error, match="WHICH"):
        gcode(printer, "ACE_FLAP", WHICH="side", OPEN=1)
    with pytest.raises(FakeGcmd.error, match="SECONDS of at most 600"):
        gcode(printer, "ACE_FAN", ON=1, SECONDS=601)
    responses = dict(DRYER_RESPONSES, ace2k_fan_set={"op": 0, "accepted": 0, "reason": 1})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="holds the fans and flaps"):
        gcode(printer, "ACE_FAN", ON=1)
    responses = dict(DRYER_RESPONSES, ace2k_flap_pulse={"op": 1, "accepted": 0, "reason": 2})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="rear flap refused: out of range"):
        gcode(printer, "ACE_FLAP", WHICH="rear", OPEN=1)
    # busy for a flap: the dryer's hold, or another pulse still running
    responses = dict(DRYER_RESPONSES, ace2k_flap_pulse={"op": 1, "accepted": 0, "reason": 1})
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    with pytest.raises(FakeGcmd.error, match="bottom flap refused: busy.*another flap pulse"):
        gcode(printer, "ACE_FLAP", WHICH="bottom", OPEN=1)


def test_ace_fan_off_says_the_request_was_released_not_that_the_fans_are_off():
    module, printer, mcu, dryer = build(known=RELEASE)
    gcmd = gcode(printer, "ACE_FAN", ON=0)
    assert "released" in gcmd.lines[-1]
    assert "45 °C" in gcmd.lines[-1]
    assert "fans off" not in gcmd.lines[-1]
    assert "last report" not in gcmd.lines[-1]  # no airflow report yet
    mcu.subscriptions["ace2k_airflow_state"](
        {"fans_cmd": 1, "fans_read": 3, "owner": 1, "flaps": 0}
    )
    gcmd = gcode(printer, "ACE_FAN", ON=0)
    assert "released" in gcmd.lines[-1] and "commanded on" in gcmd.lines[-1]
    assert "fans off" not in gcmd.lines[-1]


def test_the_release_flap_refuses_a_pulse_length_and_sends_nothing():
    module, printer, mcu, dryer = build(known=RELEASE)
    with pytest.raises(FakeGcmd.error, match="takes no MS="):
        gcode(printer, "ACE_FLAP", WHICH="rear", OPEN=1, MS=200)
    assert sent_to(mcu, "ace2k_flap_pulse") == []


def log_frame(index, a=0, b=0, c=0, d=0, e=0, f=0):
    return {"index": index, "a": a, "b": b, "c": c, "d": d, "e": e, "f": f}


def test_ace_dryer_log_prints_the_counters_the_energy_and_the_entries():
    fault = 2 | (4 << 8) | (12 << 16)  # kind fault, reason chamber_over, cycle 12
    interrupted = 4 | (0 << 8) | (11 << 16)
    minus_5 = (1 << 32) - 50  # -5.0 °C sign-extended to 32 bits
    unknown = (1 << 32) - 32768  # the unit's INT16_MIN: no valid reading
    responses = dict(
        DRYER_RESPONSES,
        ace2k_dryer_log_query=[
            log_frame(0, a=12, b=10, c=36000, d=7200, e=1, f=0),
            log_frame(1, a=fault, b=12240, c=721, d=700, e=765),
            log_frame(2, a=interrupted, b=9000, c=minus_5, d=300, e=unknown),
            log_frame(3, a=2 | (10 << 8), b=9000, c=300, d=300, e=250),  # fans, no cycle open
            log_frame(4),
            log_frame(0, a=12, b=10, c=36001, d=7201, e=1, f=0),  # heating went on: still stable
        ],
    )
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses, options={})
    gcmd = gcode(printer, "ACE_DRYER_LOG")
    text = gcmd.lines[-1]
    assert "12 cycles started, 10 completed, 1 fault;" in text
    assert "10.0 h of heating" in text and "720 Wh at 360 W" in text
    assert "#1 cycle 12 at 3.4 h: fault chamber_over" in text
    assert "left 72.1 °C, right 70.0 °C, chamber 76.5 °C" in text
    assert "#2 cycle 11 at 2.5 h: interrupted" in text and "left -5.0 °C" in text
    assert "chamber ?" in text
    assert "#3 no cycle open at 2.5 h: fault fans" in text
    assert "#4" not in text and "changed while it was read" not in text
    assert "10.0 h of heating" in text  # the second read's counters
    assert sent_to(mcu, "ace2k_dryer_log_query") == [[0], [1], [2], [3], [4], [0]]


def test_a_full_log_stops_after_eight_entries_and_names_the_cutout():
    entry = log_frame(1, a=2 | (8 << 8) | (3 << 16), b=3600, c=700, d=700, e=500)
    responses = dict(
        DRYER_RESPONSES,
        ace2k_dryer_log_query=[log_frame(0, a=3, b=1, e=9, f=0x2)]
        + [entry] * 8
        + [log_frame(0, a=3, b=1, e=9, f=0x2)],
    )
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    text = gcode(printer, "ACE_DRYER_LOG").lines[-1]
    assert "9 faults" in text and "power-cycle the unit" in text
    assert text.count("fault cutout") == 8
    assert sent_to(mcu, "ace2k_dryer_log_query") == [[n] for n in range(9)] + [[0]]


FAULT_FANS = 2 | (10 << 8) | (5 << 16)
FAULT_CHAMBER = 2 | (4 << 8) | (6 << 16)


def test_a_log_that_changes_while_read_is_read_again_until_it_holds():
    before = log_frame(0, a=5, b=4, e=1)
    after = log_frame(0, a=6, b=4, e=2)  # a fault appended during the first read
    responses = dict(
        DRYER_RESPONSES,
        ace2k_dryer_log_query=[
            before,
            log_frame(1, a=FAULT_FANS, b=3600),
            log_frame(2),
            after,
            after,
            log_frame(1, a=FAULT_CHAMBER, b=3700),
            log_frame(2, a=FAULT_FANS, b=3600),
            log_frame(3),
            after,
        ],
    )
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    text = gcode(printer, "ACE_DRYER_LOG").lines[-1]
    assert "6 cycles started, 4 completed, 2 faults" in text
    assert "#1 cycle 6 at 1.0 h: fault chamber_over" in text
    assert "#2 cycle 5 at 1.0 h: fault fans" in text
    assert "changed while it was read" not in text
    assert sent_to(mcu, "ace2k_dryer_log_query") == [[0], [1], [2], [0], [0], [1], [2], [3], [0]]
    assert dryer.log_counters["faults"] == 2


def test_a_log_that_never_holds_is_printed_from_the_last_read_with_a_note():
    counter = iter(range(1, 100))

    def counters():
        n = next(counter)
        return log_frame(0, a=n, e=n)

    frames = []  # every counters read differs from the one before it
    for _ in range(3):
        frames += [counters(), log_frame(1, a=FAULT_FANS, b=3600), log_frame(2), counters()]
    responses = dict(DRYER_RESPONSES, ace2k_dryer_log_query=frames)
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    text = gcode(printer, "ACE_DRYER_LOG").lines[-1]
    assert text.count("the log changed while it was read (3 tries)") == 1
    assert "6 cycles started" in text and "6 faults" in text  # the last read's counters
    assert text.count("fault fans") == 1
    assert len(sent_to(mcu, "ace2k_dryer_log_query")) == 12


def test_heater_watts_scales_the_energy():
    responses = dict(
        DRYER_RESPONSES,
        ace2k_dryer_log_query=[log_frame(0, d=3600), log_frame(1), log_frame(0, d=3600)],
    )
    module, printer, mcu, dryer = build(
        known=RELEASE, responses=responses, options={"heater_watts": 340}
    )
    assert "340 Wh at 340 W" in gcode(printer, "ACE_DRYER_LOG").lines[-1]


def test_the_counters_are_read_once_after_ready_from_a_timer(caplog):
    responses = dict(
        DRYER_RESPONSES, ace2k_dryer_log_query=[log_frame(0, a=3, b=2, f=0x2), EMPTY_LOG]
    )
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    printer.handlers["klippy:ready"]()  # the handler itself sends nothing (a pause is refused)
    assert sent_to(mcu, "ace2k_dryer_log_query") == []
    (seed,) = [t for t in printer.reactor.timers if t[0] == dryer._seed_log]
    with caplog.at_level("WARNING", logger="root"):
        seed[1] = seed[0](seed[1])  # what the reactor runs 0.1 s later
    assert seed[1] == FakeReactor.NEVER
    assert sent_to(mcu, "ace2k_dryer_log_query") == [[0]]
    assert dryer.log_counters["cycles_started"] == 3
    assert "thermal cutout" in caplog.text and "power-cycle" in caplog.text
    assert not [t for t in printer.reactor.armed() if t[0] == dryer._seed_log]


def test_an_unanswered_seed_is_a_warning_and_no_retry(caplog):
    responses = dict(DRYER_RESPONSES, ace2k_dryer_log_query=[printer_error()])
    module, printer, mcu, dryer = build(known=RELEASE, responses=responses)
    printer.handlers["klippy:ready"]()
    (seed,) = [t for t in printer.reactor.timers if t[0] == dryer._seed_log]
    with caplog.at_level("WARNING", logger="root"):
        assert seed[0](seed[1]) == FakeReactor.NEVER
    assert "dryer log query failed" in caplog.text and dryer.log_counters is None


def test_an_image_without_the_dryer_seeds_no_log():
    module, printer, mcu, dryer = build(known=NO_DRYER)
    printer.handlers["klippy:ready"]()
    assert not [t for t in printer.reactor.timers if t[0] == dryer._seed_log]


def test_the_module_is_idle_until_ready_and_has_no_timer_armed_before():
    module, printer, mcu, dryer = build(known=RELEASE)
    assert [t for t in printer.reactor.timers if t[1] != FakeReactor.NEVER] == []
