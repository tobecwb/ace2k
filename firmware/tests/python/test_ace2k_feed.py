"""ace2k_feed.py: the config and init commands, the subscriptions, the state and event decoding,
the eleven G-codes with the waiting and the errors, the calibration flow — against the fake
printer of test_ace2k_extras."""

import pathlib
import re
import struct

import ace2k_feed
import pytest
from test_ace2k_extras import (
    DEFAULT_CONSTANTS,
    RESPONSES,
    FakeConfig,
    FakeGcmd,
    FakeMcu,
    FakePrinter,
    lane_counters_frame,
    make,
)

OLD_START_FMT = "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u"  # before the seq

FEED_KNOWN = {
    "ace2k_version",
    "ace2k_linkproof_state",
    "ace2k_feed_start",
    "ace2k_feed_start_response",
    "ace2k_feed_state",
    "ace2k_feed_event",
}


def feed_of(module):
    return module.feed


def make_feed(responses, **kwargs):
    """make() as if the session's first state report had arrived with every lane idle (the
    seeding: test_the_sequence_is_seeded_from_the_state_report_until_the_lanes_first_start)."""
    module, printer, mcu = make(responses, **kwargs)
    feed_of(module)._seed([0] * ace2k_feed.LANES)
    return module, printer, mcu


def state_frame(modes, errors, speeds_um_s, duties, bursts, seqs=(0, 0, 0, 0)):
    return {
        "mode": bytes(modes),
        "error": bytes(errors),
        "speed_um_s": struct.pack("<4I", *speeds_um_s),
        "duty_pct": bytes(duties),
        "bursts": struct.pack("<4H", *bursts),
        "seq": bytes(seqs),
    }


IDLE_FRAME = state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4)


def event_params(lane, kind, mode, motor_um, filament_um, seq=0):
    """An event frame; seq 0 is the unit's own (a motion it started itself), a host start's
    events carry its sequence — 1 for the first start on a lane."""
    return {
        "lane": lane,
        "kind": kind,
        "mode": mode,
        "motor_um": motor_um,
        "filament_um": filament_um,
        "seq": seq,
    }


def accepted(mcu, *events):
    """The start's response; with events, the event handler fires them in order while the query
    is in flight (as the unit's events would arrive before a waiting G-code checks its
    completion)."""

    def respond():
        for event in events:
            mcu.subscriptions["ace2k_feed_event"](event)
        return {"lane": events[0]["lane"] if events else 0, "accepted": 1, "reason": 0}

    return respond


def test_load_config_reads_the_keys_from_the_ace2k_section():
    # [ace2k] loads the module with printer.load_object(config, "ace2k_feed"), which on Klipper
    # hands it the [ace2k_feed] section — one that does not exist.  The keys live in [ace2k]:
    # load_config asks for that section, so the keys are read (and marked as accessed for
    # Klipper's unused-option check) from the section that carries them.
    class RecordingConfig(FakeConfig):
        def __init__(self, printer):
            super().__init__(printer)
            self.sections_asked = []

        def getsection(self, name):
            self.sections_asked.append(name)
            return self

    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    config = RecordingConfig(printer)
    feed = ace2k_feed.load_config(config)
    assert config.sections_asked == ["ace2k"]
    assert isinstance(feed, ace2k_feed.Ace2kFeed) and feed.section == "ace2k"


def test_build_config_sends_scales_thresholds_load_and_query():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(
        responses,
        options={
            "lane2_encoder_scale": 1.25,
            "slip_allow_mm": 10,
            "load_park_mm": 380,
            "auto_load": False,
        },
    )
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert "ace2k_lane_scale_set lane=1 um_per_count_x10=12500" in cfg
    assert (
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=10000 stall_check_um=20000"
        in cfg
    )
    assert "ace2k_feed_load_set park_um=380000 speed_um_s=30000 auto_load=0" in init
    assert any(c.startswith("ace2k_feed_query rest_ticks=") for c in init)
    assert "ace2k_feed_state" in mcu.subscriptions and "ace2k_feed_event" in mcu.subscriptions
    assert feed.lanes[1]["encoder_scale"] == 1.25 and feed.lanes[1]["scale_source"] == "config"
    assert feed.lanes[0]["scale_source"] == "default"


def test_slip_allow_at_or_above_slip_check_is_a_config_error_naming_both_keys():
    # the pair as the unit sees it: an absent key is the firmware's default (50 / 15 mm)
    with pytest.raises(Exception, match="slip_allow_mm.*slip_check_mm"):
        make_feed(dict(RESPONSES), options={"slip_allow_mm": 50})  # equal to the default window
    with pytest.raises(Exception, match="slip_allow_mm.*slip_check_mm"):
        make_feed(dict(RESPONSES), options={"slip_check_mm": 10})  # under the default allowance
    with pytest.raises(Exception, match=r"slip_allow_mm \(60\).*slip_check_mm \(40\)"):
        make_feed(dict(RESPONSES), options={"slip_check_mm": 40, "slip_allow_mm": 60})
    # one under the other, both given: sent as asked
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"slip_check_mm": 40, "slip_allow_mm": 39}
    )
    mcu.config_callback()
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    assert (
        "ace2k_feed_thresholds_set slip_check_um=40000 slip_allow_um=39000 stall_check_um=20000"
        in cfg
    )


def test_the_thresholds_are_three_and_the_reserved_assist_stall_kind_still_decodes():
    # the timed assist stall is gone from the unit: no key, no argument on the wire
    assert set(ace2k_feed.THRESHOLD_DEFAULTS) == {
        "slip_check_mm",
        "slip_allow_mm",
        "stall_check_mm",
    }
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"stall_check_mm": 25, "assist_stall_ms": 500}
    )
    mcu.config_callback()
    cfg = [c for c, is_init in mcu.config_cmds if not is_init]
    assert cfg == [
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=15000 stall_check_um=25000"
    ]
    # kind 11 is reserved on the wire, never sent; a unit that did would still be named
    assert ace2k_feed.KIND_NAMES[11] == "assist_stall"


def test_a_comparator_window_as_long_as_an_assists_burst_is_a_config_error():
    # the burst read from the dictionary: 100 mm by default, and whatever the firmware says
    for key in ("stall_check_mm", "slip_check_mm"):
        options = {key: 100}
        if key == "slip_check_mm":
            options["slip_allow_mm"] = 15
        module, printer, mcu = make_feed(dict(RESPONSES), options=options)
        with pytest.raises(Exception, match=rf"{key} \(100\) must be under 100 mm"):
            mcu.config_callback()
        module, printer, mcu = make_feed(dict(RESPONSES), options={key: 99})
        mcu.config_callback()
        cfg = [c for c, is_init in mcu.config_cmds if not is_init]
        assert any(c.startswith("ace2k_feed_thresholds_set") and "=99000" in c for c in cfg)
    # a firmware with another burst: the bound follows it
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_ASSIST_BURST_UM=60000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"stall_check_mm": 60}, constants=constants
    )
    with pytest.raises(Exception, match=r"stall_check_mm \(60\) must be under 60 mm"):
        mcu.config_callback()


def test_no_thresholds_command_without_a_threshold_key():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    assert not any(c.startswith("ace2k_feed_thresholds_set") for c, _ in mcu.config_cmds)


def test_a_firmware_without_feed_is_logged_and_the_gcodes_refuse(caplog):
    module, printer, mcu = make_feed(dict(RESPONSES), known=set(FEED_KNOWN) - {"ace2k_feed_start"})
    mcu.config_callback()
    feed = feed_of(module)
    assert not feed.present
    assert "without CONFIG_ACE2K_FEED" in caplog.text
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed.cmd_ACE_FEED(gcmd)


def test_a_start_or_event_with_another_format_is_a_config_error_asking_for_a_reflash():
    # the unit runs a feed image from before the sequence: its start exists without seq=%c —
    # not "built without CONFIG_ACE2K_FEED", which would send the operator to Kconfig
    module, printer, mcu = make_feed(dict(RESPONSES), formats={"ace2k_feed_start": OLD_START_FMT})
    with pytest.raises(Exception, match="ace2k_feed_start has another format.*different commits"):
        mcu.config_callback()
    # the same for a response this module subscribes to
    old_event = ace2k_feed.EVENT_FMT.replace(" seq=%c", "")
    module, printer, mcu = make_feed(dict(RESPONSES), formats={"ace2k_feed_event": old_event})
    with pytest.raises(Exception, match="ace2k_feed_event has another format.*different commits"):
        mcu.config_callback()
    # a report absent while the start is present is the same case: the binding declares the
    # start, the state and the event in one file, so this is another commit's firmware too —
    # said as such, with the parser's text and the same hint, not as "another format"
    for missing in ("ace2k_feed_event", "ace2k_feed_state"):
        module, printer, mcu = make_feed(dict(RESPONSES), known=set(FEED_KNOWN) - {missing})
        with pytest.raises(
            Exception,
            match=rf"{missing} is not in the unit's dictionary though ace2k_feed_start is"
            rf" \(Unknown command: {missing}\); .*different commits",
        ):
            mcu.config_callback()


def test_a_speed_outside_the_dictionary_bounds_is_a_config_error():
    module, printer, mcu = make_feed(dict(RESPONSES), options={"feed_speed": 150})
    with pytest.raises(Exception, match="feed_speed"):
        mcu.config_callback()


def test_state_frame_lands_in_status():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([1, 7, 0, 5], [0, 7, 0, 0], [30000, 0, 0, 50000], [42, 0, 0, 60], [0, 0, 0, 12])
    )
    lanes = module.get_status(0)["lanes"]
    assert lanes[0]["mode"] == "feeding" and lanes[0]["speed"] == 30.0 and lanes[0]["duty"] == 42
    assert lanes[1]["mode"] == "error" and lanes[1]["error"] == "stuck"
    assert lanes[0]["error"] is None
    assert lanes[3]["mode"] == "assisting" and lanes[3]["bursts"] == 12
    gcmd = FakeGcmd()
    module.cmd_ACE_STATUS(gcmd)
    assert any("mode=feeding" in line for line in gcmd.lines)


def test_an_event_is_logged_stored_and_raised_as_a_klipper_event(caplog):
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    with caplog.at_level("INFO", logger="root"):  # the event line is an INFO log
        mcu.subscriptions["ace2k_feed_event"](event_params(2, 7, 1, 120000, 15000))
    ev = feed.lanes[2]["last_event"]
    assert ev == dict(kind="stuck", mode="feeding", motor_mm=120.0, filament_mm=15.0, seq=0)
    assert feed.lanes[2]["last_move"]["commanded_mm"] is None  # nothing this host started
    assert "lane 3 stuck (feeding)" in caplog.text
    assert printer.events == [("ace2k:feed_event", (2, ev))]
    mcu.subscriptions["ace2k_feed_event"](event_params(2, 14, 5, 5000, 8000))
    assert feed.lanes[2]["last_event"]["kind"] == "behind"
    assert feed.lanes[2]["last_move"]["kind"] == "stuck"  # a notice is not a move's end


def test_feed_waits_for_done_and_reports_the_numbers():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 300000, 298500, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "300"})
    feed.cmd_ACE_FEED(gcmd)
    assert mcu.sent[-1] == "ace2k_feed_start"
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 300.0 mm, filament 298.5 mm"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 300.0
    assert feed.lanes[0]["last_move"]["seq"] == 1
    assert feed.completions == {}


def test_the_start_arguments_are_micrometres_and_the_mode_number():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, options={"feed_speed": 40})
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data, minclock=0, reqclock=0):
            sent.append(list(data))
            mcu.subscriptions["ace2k_feed_event"](event_params(data[0], 0, 1, 1, 1, data[4]))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    mcu.lookup_query_command = lambda *a, **k: Query()
    # the last argument is the lane's sequence: 1 for its first start, 2 for the second
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "2", "LENGTH": "12.5"}))
    assert sent[-1] == [1, 0, 12500, 40000, 1]
    feed.cmd_ACE_ROLLBACK(FakeGcmd({"LANE": "2", "LENGTH": "10", "SPEED": "25"}))
    assert sent[-1] == [1, 1, 10000, 25000, 2]
    feed.cmd_ACE_UNLOAD(FakeGcmd({"LANE": "3"}))
    assert sent[-1] == [2, 4, 0, 30000, 1]
    feed.cmd_ACE_LOAD(FakeGcmd({"LANE": "4"}))
    assert sent[-1] == [3, 5, 300000, 30000, 1]
    feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "1"}))
    assert sent[-1] == [0, 2, 0, 50000, 1]
    feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "1", "DIR": "back", "SPEED": "20"}))
    assert sent[-1] == [0, 3, 0, 20000, 2]


def test_a_refusal_is_a_gcode_error_naming_the_reason():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 2}
    with pytest.raises(Exception, match="refused: in_error"):
        feed_of(module).cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert feed_of(module).completions == {}


def test_a_soft_assist_names_a_refusal_instead_of_raising():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 0, "reason": 1}
    # without SOFT the unit's refusal is the G-code error, as for every start
    with pytest.raises(Exception, match="lane 2 assist refused: busy"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "2"}))
    # with SOFT=1 the same refusal is a line naming the lane and the reason, and no error
    sent = len(mcu.sent)
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert len(mcu.sent) == sent + 1 and feed.completions == {}
    assert gcmd.lines == ["ace2k: lane 2 not armed — lane 2 assist refused: busy"]
    # a host-side refusal (a speed out of bounds) is named the same way, nothing sent
    sent = len(mcu.sent)
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1", "SPEED": "9999"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert len(mcu.sent) == sent and len(gcmd.lines) == 1
    assert gcmd.lines[0].startswith("ace2k: lane 2 not armed — SPEED must be within")
    # an accepted start with SOFT=1 is the same as without
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert gcmd.lines == ["ace2k: lane 2 assist started"]
    # SOFT takes 0 or 1 only, and OFF=1 with SOFT=1 disarms as without it
    with pytest.raises(Exception, match="SOFT"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "2", "SOFT": "2"}))
    gcmd = FakeGcmd({"LANE": "2", "SOFT": "1", "OFF": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    assert gcmd.lines == ["ace2k: lane 2 assist off"]


def test_an_error_event_is_a_gcode_error_and_a_missing_one_a_timeout():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 8, 1, 60000, 4000, seq=1))
    with pytest.raises(Exception, match="failed: tangled — motor 60.0 mm, filament 4.0 mm"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    # 100 mm at 30 mm/s: 90 mm at 30 and the 10 mm landing at the 9 mm/s floor — 1.5 × (90 / 30
    # + 10 / 9) + 5 = 11.17 s, printed whole
    with pytest.raises(Exception, match="no completion event within 11 s; a stop was sent"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert feed.completions == {}
    # the host gave up on a move it still believes is running: the lane is stopped
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # a waited unload counts the tail as a second move: 50 mm and the 100 mm tail at 9 mm/s —
    # 1.5 × (50 / 9 + 100 / 9) + 5 = 30 s
    with pytest.raises(Exception, match="unload: no completion event within 30 s"):
        feed.cmd_ACE_UNLOAD(FakeGcmd({"LANE": "1", "LENGTH": "50", "SPEED": "9"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # a rollback runs the same tail once the filament leaves the drive: 30 s too, not 13.33 s
    with pytest.raises(Exception, match="rollback: no completion event within 30 s"):
        feed.cmd_ACE_ROLLBACK(FakeGcmd({"LANE": "1", "LENGTH": "50", "SPEED": "9"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])


def test_the_unload_wait_counts_the_tail_as_a_second_move():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.tail_mm == 100.0
    assert feed._wait_timeout(50, 9, tail_mm=100) == pytest.approx(30.0, abs=0.005)
    # 100 mm at 30 mm/s, the tail too: 2 × (90 / 30 + 10 / 9) × 1.5 + 5 = 17.33 s
    assert feed._wait_timeout(100, 30, tail_mm=100) == pytest.approx(17.33, abs=0.005)
    assert feed._wait_timeout(100, 30) == pytest.approx(11.17, abs=0.005)  # no tail: as before


def test_a_dictionary_without_the_tail_is_a_config_error():
    constants = {k: v for k, v in DEFAULT_CONSTANTS.items() if k != "ACE2K_FEED_UNLOAD_TAIL_UM"}
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    with pytest.raises(Exception, match="ACE2K_FEED_UNLOAD_TAIL_UM"):
        mcu.config_callback()


def test_the_waited_timeout_budgets_the_landing_at_the_floor_speed():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.landing_mm == DEFAULT_CONSTANTS["ACE2K_LANE_LANDING_UM"] / 1000 == 10.0
    # longer than the landing: 90 mm at 30 mm/s and the 10 mm landing at the 9 mm/s floor —
    # 1.5 × (90 / 30 + 10 / 9) + 5 = 1.5 × 4.111 + 5 = 11.17 s; whole at 30 mm/s it would have
    # been 1.5 × 100 / 30 + 5 = 10.0 s
    assert feed._wait_timeout(100, 30) == pytest.approx(11.17, abs=0.005)
    assert feed._wait_timeout(100, 30) > 10.0
    # no longer than the landing: the whole move at the floor speed, whatever was asked —
    # 5 mm: 1.5 × 5 / 9 + 5 = 5.83 s at 70 mm/s as at 30; the landing's own 10 mm: 1.5 × 10 / 9
    # + 5 = 6.67 s
    assert feed._wait_timeout(5, 70) == pytest.approx(5.83, abs=0.005)
    assert feed._wait_timeout(5, 70) == feed._wait_timeout(5, 30)
    assert feed._wait_timeout(feed.landing_mm, 70) == pytest.approx(6.67, abs=0.005)


def test_wait_zero_returns_at_once():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"})
    feed_of(module).cmd_ACE_FEED(gcmd)
    assert gcmd.lines == ["ace2k: lane 1 feed started"]
    assert feed_of(module).completions == {}


def test_a_second_start_on_a_busy_lane_is_sent_and_the_units_refusal_is_the_error():
    # the host keeps no in-flight flag: a start on a lane whose previous move has not reported
    # its end goes out, and the unit's own busy refusal is what stops it
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert not feed.any_busy()  # the 1 Hz report alone tells busy
    sent = len(mcu.sent)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(Exception, match="lane 1 feed refused: busy"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert len(mcu.sent) == sent + 1 and feed.completions == {}
    # the refused start took a sequence (2) but entered no mode: its entry is gone, the first
    # start's stays, and the running move's done (seq 1) still carries the commanded length
    assert feed.commanded_mm[0] == {1: 100.0}
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 100000, 99000, seq=1))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    assert feed.commanded_mm[0] == {} and feed.seq[0] == 2


def test_each_starts_length_is_kept_until_its_own_event_and_the_map_is_pruned():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    # a second start accepted before the first move's done lands (the unit saw the first end):
    # the first's done still publishes the first's length, the second's its own
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "50", "WAIT": "0"}))
    assert feed.commanded_mm[0] == {1: 100.0, 2: 50.0}
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 100000, 99000, seq=1))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 50000, 49500, seq=2))
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0
    assert feed.commanded_mm[0] == {}
    # a notice pops nothing; a unit-started terminal event (seq 0) pops nothing either
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "20", "WAIT": "0"}))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 14, 5, 5000, 8000, seq=3))
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 5, 4, 400000, 399000, seq=0))
    assert feed.commanded_mm[0] == {3: 20.0}
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None
    # starts whose events never come are pruned to the newest COMMANDED_KEEP (8): after nine
    # more (seqs 4..12) the oldest, 3 and 4, are gone and 5..12 remain
    for length in range(9):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": str(length + 1), "WAIT": "0"}))
    assert list(feed.commanded_mm[0]) == list(range(5, 13))
    assert len(feed.commanded_mm[0]) == ace2k_feed.COMMANDED_KEEP == 8
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 1000, 1000, seq=3))
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None  # pruned: unknown, not wrong


def test_a_stale_event_does_not_complete_the_wait_and_the_matching_one_does():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    # a WAIT=0 feed (seq 1) whose done is late; then a waited feed (seq 2) — the unit accepts
    # it (the first move ended on its side) and the stale done lands during the round trip,
    # before the waited move's own
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    stale = event_params(0, 0, 1, 100000, 99000, seq=1)
    own = event_params(0, 0, 1, 50000, 49000, seq=2)

    class Query:
        def send(self, data, minclock=0, reqclock=0):
            mcu.subscriptions["ace2k_feed_event"](stale)
            # the stale done sits in last_event with its own start's length, the wait pending
            assert feed.lanes[0]["last_event"]["seq"] == 1
            assert feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
            assert feed.completions[0][0] == 2 and not feed.completions[0][1].test()
            mcu.subscriptions["ace2k_feed_event"](own)
            return {"lane": 0, "accepted": 1, "reason": 0}

    mcu.lookup_query_command = lambda *a, **k: Query()
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "50"})
    feed.cmd_ACE_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 50.0 mm, filament 49.0 mm"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0
    assert feed.completions == {}
    # every event, stale or not, reached the other modules
    assert [ev["seq"] for name, (lane, ev) in printer.events] == [1, 2]


def test_a_unit_started_motion_completes_no_wait_and_carries_no_commanded_length():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    # seq 0 is the unit's own (an automatic load): it neither completes a pending wait nor
    # blocks it, and its last_move has no commanded length
    responses["ace2k_feed_start"] = accepted(
        mcu,
        event_params(0, 5, 4, 400000, 399000, seq=0),
        event_params(0, 0, 1, 100000, 99500, seq=1),
    )
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACE_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 100.0 mm, filament 99.5 mm"
    loaded, done = [ev for name, (lane, ev) in printer.events]
    assert loaded["kind"] == "loaded" and loaded["seq"] == 0
    assert done["seq"] == 1 and feed.lanes[0]["last_move"]["commanded_mm"] == 100.0
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 6, 3, 300000, -299000, seq=0))
    assert feed.lanes[0]["last_move"]["kind"] == "unloaded"
    assert feed.lanes[0]["last_move"]["commanded_mm"] is None


def test_the_sequence_is_seeded_from_the_state_report_until_the_lanes_first_start():
    # on a klippy RESTART the unit may still run the previous session's move, or have just ended
    # it; the report's seq is the last start issued on the lane, kept through idle, and the
    # counter of a lane this session has not started on follows every report, so the first
    # start is reported + 1 — never the running move's, nor a repeat of the one that just ended
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    sent = []

    class Query:
        def send(self, data, minclock=0, reqclock=0):
            sent.append(list(data))
            return {"lane": data[0], "accepted": 1, "reason": 0}

    mcu.lookup_query_command = lambda *a, **k: Query()
    # no report yet this session and none coming: the start waits its bound, then errors with
    # nothing sent (test_a_start_before_the_first_state_report_waits_for_it)
    with pytest.raises(Exception, match="no feed state report within 7 s"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent == [] and not feed.started[2]
    # lane 3 (index 2) runs a move from the earlier session with seq 41: the next start is 42;
    # lane 2 (index 1) is idle and its last start was 5 — the report keeps it, so the counter
    # follows it too; lane 4 (index 3) never started since the unit powered up: 0
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0, 0, 1, 0], [0] * 4, [0, 0, 30000, 0], [0, 0, 40, 0], [0] * 4, (0, 5, 41, 0))
    )
    assert feed.seq == [0, 5, 41, 0]
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][0] == 2 and sent[-1][-1] == 42
    # 255 wraps to 1
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (255, 0, 0, 0))
    )
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][0] == 0 and sent[-1][-1] == 1
    # once a lane has been started on, a later report does not reseed it: lanes 1 and 3 keep
    # their own counters (1 and 42 → 2 and 43), lane 2 — never started — still follows (99)
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (7, 99, 7, 0))
    )
    assert feed.seq == [1, 99, 42, 0]
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][-1] == 2
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "3", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][-1] == 43
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "2", "LENGTH": "100", "WAIT": "0"}))
    assert sent[-1][-1] == 100


def test_a_start_before_the_first_state_report_waits_for_it_then_sends_reported_plus_one():
    # a macro that fires at startup, before the unit's first 1 Hz report: the start is parked
    # on the report, not refused — the frame lands while the G-code waits, and the start goes
    # out seeded from it (every lane idle, seq 0: the first start sends 1)
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu)
    waited_until = []

    def frame_lands(waketime):
        waited_until.append(waketime)
        mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)

    printer.reactor.while_waiting[feed.first_report] = [frame_lands]
    assert not feed.state_reported
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"})
    feed.cmd_ACE_FEED(gcmd)
    assert feed.state_reported and feed.seq == [1, 0, 0, 0]
    # exactly one start, none before the report landed
    assert mcu.queries == [("ace2k_feed_start", [0, 0, 100000, 30000, 1])]
    assert gcmd.lines[-1] == "ace2k: lane 1 feed started"
    # the bound: two report periods of the configured rate plus the 5 s margin — 7 s at 1 Hz
    assert waited_until == [printer.reactor.now + 2 / 1.0 + ace2k_feed.WAIT_MARGIN_S]
    # a later start waits on nothing for the report: the sentinel on first_report never runs —
    # not for a WAIT=0 start, nor for a WAIT=1 one, whose wait is on its own event's completion
    printer.reactor.while_waiting[feed.first_report] = [
        lambda waketime: pytest.fail("waited for the report again")
    ]
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "WAIT": "0"}))
    assert mcu.queries[-1][1][-1] == 2
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 100000, 99500, seq=3))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACE_FEED(gcmd)
    assert mcu.queries[-1][1][-1] == 3 and feed.completions == {}
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 100.0 mm, filament 99.5 mm"
    # none arriving within the bound: the error names the bound, computed from the configured
    # rate — 2 / 5 Hz + 5 = 5.4 s — and the rate; nothing is sent
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses, options={"feed_report_hz": 5})
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu)
    with pytest.raises(
        Exception,
        match=r"no feed state report within 5\.4 s \(the unit reports at feed_report_hz = 5 Hz;"
        r" check the link\)",
    ):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100"}))
    assert mcu.queries == [] and not feed.started[0] and feed.completions == {}


def test_a_report_seeds_through_the_reactor_only_while_a_lane_still_follows_it():
    # the seeding is a reactor callback per report; once the first report has been seen and
    # every lane has been started on, a report has nothing to seed and schedules none
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    scheduled = []
    run_at_once = printer.reactor.register_async_callback

    def counting(cb, waketime=None):
        scheduled.append(cb)
        run_at_once(cb, waketime)

    printer.reactor.register_async_callback = counting
    report = mcu.subscriptions["ace2k_feed_state"]
    report(IDLE_FRAME)
    assert len(scheduled) == 1 and feed.state_reported
    responses["ace2k_feed_start"] = accepted(mcu)
    for lane in ("1", "2", "3"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": lane, "LENGTH": "100", "WAIT": "0"}))
    report(IDLE_FRAME)  # lane 4 still follows the reports
    assert len(scheduled) == 2
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "4", "LENGTH": "100", "WAIT": "0"}))
    report(IDLE_FRAME)
    report(IDLE_FRAME)
    assert len(scheduled) == 2 and feed.seq == [1, 1, 1, 1]
    assert feed.lanes[0]["mode"] == "idle"  # the frame still lands in the status


def test_the_report_period_is_snapped_to_the_100_ms_grid(caplog):
    # the report phases hold only for periods on the 100 ms grid (docs/protocol.md "Reports"):
    # 1, 2, 5 and 10 Hz are exact, anything else is snapped and the effective rate logged; the
    # floor, 0.05 Hz, is a 20 s period (2.4e9 ticks at 120 MHz, under the 2^32 the unit's
    # rest_ticks carries) — and the wait of a start before the first report grows with it
    with pytest.raises(Exception, match="feed_report_hz"):
        make_feed(dict(RESPONSES), options={"feed_report_hz": 0.02})
    for hz, period_ms in ((3, 300), (10, 100), (0.4, 2500), (1, 1000), (0.05, 20000)):
        caplog.clear()
        module, printer, mcu = make_feed(dict(RESPONSES), options={"feed_report_hz": hz})
        mcu.config_callback()
        feed = feed_of(module)
        init = [c for c, is_init in mcu.config_cmds if is_init]
        assert f"ace2k_feed_query rest_ticks={mcu.seconds_to_clock(period_ms / 1000.0)}" in init
        assert feed.report_period_ms == period_ms
        assert feed.report_hz == pytest.approx(1000.0 / period_ms)
        assert ("snapped" in caplog.text) == (hz == 3), (hz, caplog.text)
        if hz == 3:
            assert "feed_report_hz 3 snapped to 3.33 Hz (a 300 ms period on the 100 ms grid)" in (
                caplog.text
            )


def test_the_sequence_wraps_from_255_to_1_and_is_never_0():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    assert feed._next_seq(2) == 1
    feed.seq[2] = 254
    assert feed._next_seq(2) == 255
    assert feed._next_seq(2) == 1
    assert feed.seq[0] == 0  # the other lanes' counters are their own
    assert all(feed._next_seq(0) != 0 for _ in range(600))


def test_stop_then_feed_on_the_same_lane_is_not_refused_by_the_host():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "500", "WAIT": "0"}))
    feed.cmd_ACE_STOP(FakeGcmd({"LANE": "1"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [0])
    # the stopped event of seq 1 has not landed yet: the next feed goes out all the same, and
    # completes on its own seq 2 when the stopped event of seq 1 arrives first
    responses["ace2k_feed_start"] = accepted(
        mcu, event_params(0, 1, 1, 20000, 19000, seq=1), event_params(0, 0, 1, 50000, 49000, seq=2)
    )
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "50"})
    feed.cmd_ACE_FEED(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 1 feed done — motor 50.0 mm, filament 49.0 mm"
    assert feed.completions == {}


def test_every_terminal_kind_completes_a_wait_on_its_sequence_but_a_notice_does_not():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    seq = 0
    # stopped, runout, stuck, motor_stalled, unload_incomplete, tail_out, blocked
    for kind in (1, 4, 7, 9, 13, 17, 18):
        seq += 1
        # a behind notice with the same seq first: it leaves the wait pending
        responses["ace2k_feed_start"] = accepted(
            mcu,
            event_params(2, 14, 5, 5000, 8000, seq=seq),
            event_params(2, kind, 1, 1000, 1000, seq=seq),
        )
        gcmd = FakeGcmd({"LANE": "3", "LENGTH": "100"})
        if kind in ace2k_feed.ERROR_KINDS:
            with pytest.raises(Exception, match=f"failed: {ace2k_feed.KIND_NAMES[kind]}"):
                feed.cmd_ACE_FEED(gcmd)
        else:
            feed.cmd_ACE_FEED(gcmd)
            assert gcmd.lines[-1].startswith(f"ace2k: lane 3 feed {ace2k_feed.KIND_NAMES[kind]}")
        assert feed.completions == {}
        assert feed.lanes[2]["last_move"]["kind"] == ace2k_feed.KIND_NAMES[kind]
    # a notice alone: last_event, not last_move, and nothing to complete
    mcu.subscriptions["ace2k_feed_event"](event_params(2, 14, 5, 5000, 8000, seq=seq))
    assert feed.lanes[2]["last_event"]["kind"] == "behind"
    assert feed.lanes[2]["last_move"]["kind"] == "blocked"


def test_a_gcode_speed_outside_the_bounds_is_refused_before_sending():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    # the bounds the fake dictionary carries, as the module prints them
    low = DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MIN_UM_S"] / 1000
    high = DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MAX_UM_S"] / 1000
    with pytest.raises(Exception, match=re.escape(f"SPEED must be within {low:g}..{high:g}")):
        feed_of(module).cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "100", "SPEED": "5"}))
    assert "ace2k_feed_start" not in mcu.sent


def test_stop_clear_speed_and_assist_off_send_their_commands():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.cmd_ACE_STOP(FakeGcmd())
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [255])
    feed.cmd_ACE_STOP(FakeGcmd({"LANE": "2"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    feed.cmd_ACE_CLEAR(FakeGcmd({"LANE": "3"}))
    assert mcu.commands[-1] == ("ace2k_feed_clear lane=%c", [2])
    feed.cmd_ACE_SPEED(FakeGcmd({"LANE": "1", "SPEED": "45"}))
    assert mcu.commands[-1] == ("ace2k_feed_speed_set lane=%c speed_um_s=%u", [0, 45000])
    feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "4", "OFF": "1"}))
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [3])


def test_calibration_feeds_then_stages_the_scale():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "200"})
    feed.cmd_ACE_CALIBRATE_ENCODER(gcmd)
    assert "MEASURED=<mm>" in gcmd.lines[-1]
    assert feed.calibration[0] == (200.0, 1.2342)
    gcmd = FakeGcmd({"LANE": "1", "MEASURED": "197"})
    feed.cmd_ACE_CALIBRATE_ENCODER(gcmd)
    # 200.0 mm / 1.2342 = 162.05 counts; 197 / 162.05 = 1.2157
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.2157")]
    with pytest.raises(Exception, match="LENGTH=<mm> first"):
        feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "2", "MEASURED": "100"}))


def test_a_blocked_calibration_feed_is_an_error_and_keeps_no_sample():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    assert feed.calibration[0] == (200.0, 1.2342)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 18, 1, 42000, 9000, seq=2))
    with pytest.raises(
        Exception, match="lane 1 calibration feed blocked — motor 42.0 mm, filament 9.0 mm"
    ):
        feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    assert 0 not in feed.calibration  # neither this reading nor the earlier one
    with pytest.raises(Exception, match="LENGTH=<mm> first"):
        feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "197"}))
    assert printer.objects["configfile"].calls == []


def test_a_measurement_outside_the_scale_window_stages_nothing():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 0, 1, 200000, 200000, seq=1))
    feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "LENGTH": "200"}))
    # 120 / 162.05 counts = 0.7405 mm/count: the unit would shut down on it at the restart
    with pytest.raises(Exception, match=r"0.7405 mm/count is outside the window .*0.9874..1.4810"):
        feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "120"}))
    assert printer.objects["configfile"].calls == []
    feed.cmd_ACE_CALIBRATE_ENCODER(FakeGcmd({"LANE": "1", "MEASURED": "230"}))  # 1.4193: inside
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.4193")]


def test_load_set_sends_and_stages_what_was_given():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.cmd_ACE_LOAD_SET(FakeGcmd({"PARK": "350", "AUTO": "0"}))
    assert mcu.commands[-1] == (
        "ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c",
        [350000, 30000, 0],
    )
    assert printer.objects["configfile"].calls == [
        ("ace2k", "load_park_mm", "350"),
        ("ace2k", "auto_load", "False"),
    ]
    with pytest.raises(Exception, match="SPEED must be within"):
        feed.cmd_ACE_LOAD_SET(FakeGcmd({"SPEED": "500"}))


def test_load_set_with_a_rejected_speed_changes_nothing():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    # PARK and AUTO are valid, SPEED is not: the whole command is refused before anything is
    # assigned, staged for SAVE_CONFIG or sent
    with pytest.raises(Exception, match="SPEED must be within"):
        feed.cmd_ACE_LOAD_SET(FakeGcmd({"PARK": "250", "SPEED": "200", "AUTO": "0"}))
    assert feed.load_park_mm == 300.0
    assert feed.load_speed == 30.0
    assert feed.auto_load is True
    assert printer.objects["configfile"].calls == []
    assert mcu.commands == commands


def test_counters_reset_is_refused_while_a_lane_moves():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0])
    )
    with pytest.raises(Exception, match="a lane is moving or busy"):
        module.cmd_ACE_COUNTERS_RESET(FakeGcmd())
    # a move accepted since the last 1 Hz report is the unit's to refuse (its veto is the
    # backstop for the report's blind window): the host sends the reset
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0])
    )
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    feed_of(module).cmd_ACE_FEED(FakeGcmd({"LANE": "2", "LENGTH": "100", "WAIT": "0"}))
    assert not feed_of(module).any_busy()
    module.cmd_ACE_COUNTERS_RESET(FakeGcmd())
    assert mcu.queries[-1] == ("ace2k_lane_counters_reset", [255])


def test_every_feed_gcode_name_is_letters_and_underscores_only():
    for name in ace2k_feed.GCODES:
        assert name.replace("_", "").isalpha() and name.isupper()


def test_a_load_waits_for_the_search_when_the_firmware_has_one():
    module, printer, mcu = make_feed(dict(RESPONSES), options={"rfid_search_mm": 600})
    feed = feed_of(module)
    mcu.config_callback()
    with_search = feed._load_budget_mm(300.0)
    assert with_search == pytest.approx(300.0 + 2 * 300.0 + feed.SESSION_ALLOWANCE_MM)
    absent = {"ace2k_rfid_search_set": "Unknown command: ace2k_rfid_search_set"}
    module, printer, mcu = make_feed(dict(RESPONSES), lookup_errors=absent)
    mcu.config_callback()
    assert feed_of(module)._load_budget_mm(300.0) == 300.0


def test_an_unset_search_takes_the_firmwares_default_and_750_without_it():
    constants = dict(DEFAULT_CONSTANTS, ACE2K_FEED_SEARCH_DEFAULT_UM=800000)
    module, printer, mcu = make_feed(dict(RESPONSES), constants=constants)
    feed = feed_of(module)
    mcu.config_callback()
    allowance = feed.SESSION_ALLOWANCE_MM
    assert feed._load_budget_mm(300.0) == pytest.approx(300.0 + 2 * 500.0 + allowance)
    module, printer, mcu = make_feed(dict(RESPONSES))  # an image without the constant
    feed = feed_of(module)
    mcu.config_callback()
    allowance = feed.SESSION_ALLOWANCE_MM
    assert feed._load_budget_mm(300.0) == pytest.approx(300.0 + 2 * 450.0 + allowance)


# --- the tip snag ------------------------------------------------


def test_the_snag_values_go_out_as_an_init_command_at_every_connect():
    # an init command, keys or not: a config command reaches only an unconfigured MCU, so after
    # ACE_SNAG_SET and a RESTART the unit would keep the live values the host no longer shows
    options = {"snag_fwd_mm": 25, "snag_duty_pct": 12}
    module, printer, mcu = make_feed(dict(RESPONSES), options=options)
    mcu.config_callback()
    snag = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_snag_set")]
    assert snag == [
        (
            "ace2k_feed_snag_set fwd_um=25000 lag_um=5000 duty_pct=12 back_um=5000 rest_ms=500"
            " free_um=50000",
            True,
        )
    ]
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    snag = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_snag_set")]
    assert snag == [
        (
            "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
            " free_um=50000",
            True,
        )
    ]
    assert feed_of(module).snag["snag_fwd_mm"] == 20.0  # the firmware's default, in effect


def test_a_snag_lag_at_or_above_the_tolerance_is_a_config_error():
    # the pair as the unit sees it: an absent key is the firmware's default (fwd 20 / lag 5 mm)
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_lag_mm": 20})
    with pytest.raises(Exception, match=r"snag_lag_mm \(20\).*snag_fwd_mm \(20\)"):
        mcu.config_callback()
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_fwd_mm": 5})
    with pytest.raises(Exception, match=r"snag_lag_mm \(5\).*snag_fwd_mm \(5\)"):
        mcu.config_callback()
    assert not any(c.startswith("ace2k_feed_snag_set") for c, _ in mcu.config_cmds)


def test_a_snag_key_out_of_bounds_is_a_config_error():
    for key, value in (
        ("snag_fwd_mm", 60),
        ("snag_lag_mm", 0.5),
        ("snag_duty_pct", 101),
        ("snag_back_mm", 1),
        ("snag_rest_ms", 2001),
        ("snag_free_mm", 10),
    ):
        with pytest.raises(Exception, match=key):
            make_feed(dict(RESPONSES), options={key: value})


def test_the_host_snag_bounds_are_the_firmwares():
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_SNAG_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    }
    checked = set()
    for key, _arg, scale, low, high in ace2k_feed.SNAG_SETTINGS:
        name = ace2k_feed.SNAG_DEFAULT_CONSTANTS[key]
        assert name in defines, name  # the default the host reads from the dictionary
        assert DEFAULT_CONSTANTS[name] == defines[name], name  # the fake carries the firmware's
        unit = name.rsplit("_", 1)[1]  # UM, PCT, MS
        stem = name[: -len(unit) - 1]
        low_name, high_name = f"{stem}_MIN_{unit}", f"{stem}_MAX_{unit}"
        if low_name in defines:
            assert defines[low_name] == round(low * scale), key
            checked.add(low_name)
        else:
            assert low == 0, f"{key}: no {low_name} in feed.h for a floor of {low}"
        assert high_name in defines, f"{key}: no {high_name} in feed.h"
        assert defines[high_name] == round(high * scale), key
        checked.add(high_name)
    bounds = {n for n in defines if "_MIN_" in n or "_MAX_" in n}
    assert bounds == checked  # every bound of the header is one the host mirrors


def test_snag_set_sends_all_six_and_stages_what_was_given():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    gcmd = FakeGcmd({"FWD": "30", "DUTY": "15"})
    feed.cmd_ACE_SNAG_SET(gcmd)
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [30000, 5000, 15, 5000, 500, 50000])
    assert printer.objects["configfile"].calls == [
        ("ace2k", "snag_fwd_mm", "30"),
        ("ace2k", "snag_duty_pct", "15"),
    ]
    assert gcmd.lines == [
        "ace2k: snag fwd 30 mm, lag 5 mm, duty 15 %, back 5 mm, rest 500 ms, free 50 mm;"
        " staged for SAVE_CONFIG"
    ]
    # the values in effect carry on: a later set changes only what it names
    feed.cmd_ACE_SNAG_SET(FakeGcmd({"LAG": "12.5"}))
    assert mcu.commands[-1] == (ace2k_feed.SNAG_FMT, [30000, 12500, 15, 5000, 500, 50000])


def test_snag_set_alone_prints_and_sends_nothing_and_a_bad_lag_changes_nothing():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    gcmd = FakeGcmd()
    feed.cmd_ACE_SNAG_SET(gcmd)
    assert mcu.commands == commands
    assert gcmd.lines == [
        "ace2k: snag fwd 20 mm, lag 5 mm, duty 0 %, back 5 mm, rest 500 ms, free 50 mm"
    ]
    with pytest.raises(Exception, match="LAG"):  # above its bound (20 mm)
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"LAG": "25"}))
    with pytest.raises(Exception, match=r"LAG \(20\) must be under FWD \(20\)"):
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"LAG": "20"}))  # in bounds, not under the forward
    with pytest.raises(Exception, match=r"LAG \(8\) must be under FWD \(6\)"):
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"FWD": "6", "LAG": "8"}))
    with pytest.raises(Exception, match="FWD"):  # out of bounds, with a good one beside it
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"DUTY": "10", "FWD": "60"}))
    assert mcu.commands == commands
    assert printer.objects["configfile"].calls == []
    assert feed.snag["snag_lag_mm"] == 5.0 and feed.snag["snag_duty_pct"] == 0


def test_snag_set_on_a_firmware_without_the_feed_is_a_gcode_error():
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed_of(module).cmd_ACE_SNAG_SET(FakeGcmd({"FWD": "30"}))
    assert mcu.commands == []


def test_a_snag_notice_is_printed_and_completes_no_wait(caplog):
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    pending = printer.reactor.completion()
    feed.completions[0] = (3, pending)  # a waiting G-code on the move the notice belongs to
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_feed_event"](event_params(0, 15, 1, 41214, 41440, 3))
    ev = dict(kind="snag", mode="feeding", motor_mm=41.214, filament_mm=41.44, seq=3)
    assert feed.lanes[0]["last_event"] == ev
    assert "lane 1 snag (feeding)" in caplog.text
    assert printer.events == [("ace2k:feed_event", (0, ev))]
    assert printer.objects["gcode"].output == [
        "ace2k: lane 1 snag (feeding) — the tip caught at motor 41.2 mm, filament 41.4 mm; went on"
    ]
    assert feed.lanes[0]["last_move"] is None  # a notice is not a move's end
    assert not pending.test() and feed.completions[0] == (3, pending)


def test_a_snag_value_is_judged_at_the_units_resolution():
    # the unit checks whole µm: 5.0004 and 5.0001 are both 5000 µm, a lag equal to the forward
    # travel that would shut the MCU down — refused by the host, nothing sent or staged
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    with pytest.raises(Exception, match=r"LAG \(5\) must be under FWD \(5\)"):
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"FWD": "5.0004", "LAG": "5.0001"}))
    assert mcu.commands == commands and printer.objects["configfile"].calls == []
    # the same pair in printer.cfg: a config error at connect
    options = {"snag_fwd_mm": 5.0004, "snag_lag_mm": 5.0001}
    module, printer, mcu = make_feed(dict(RESPONSES), options=options)
    with pytest.raises(Exception, match=r"snag_lag_mm \(5\).*snag_fwd_mm \(5\)"):
        mcu.config_callback()
    # a value that rounds onto a bound is within it; one that rounds outside is not
    module, printer, mcu = make_feed(dict(RESPONSES), options={"snag_back_mm": 1.9996})
    mcu.config_callback()
    assert feed_of(module).snag["snag_back_mm"] == 2.0
    assert any("back_um=2000 " in c for c, _ in mcu.config_cmds)
    with pytest.raises(Exception, match=r"snag_back_mm \(1.999\) must be within 2..20"):
        make_feed(dict(RESPONSES), options={"snag_back_mm": 1.9994})
    feed = feed_of(module)
    with pytest.raises(Exception, match=r"FREE \(200.001\) must be within 20..200"):
        feed.cmd_ACE_SNAG_SET(FakeGcmd({"FREE": "200.0006"}))


def test_a_staged_snag_value_round_trips_exactly():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    gcmd = FakeGcmd({"FWD": "19.99999", "LAG": "4.0006", "FREE": "199.999", "BACK": "2.5"})
    feed.cmd_ACE_SNAG_SET(gcmd)
    sent = mcu.commands[-1][1]
    assert sent == [20000, 4001, 0, 2500, 500, 199999]
    staged = {key: value for _, key, value in printer.objects["configfile"].calls}
    assert staged == {
        "snag_fwd_mm": "20",
        "snag_lag_mm": "4.001",
        "snag_back_mm": "2.5",
        "snag_free_mm": "199.999",
    }
    # the staged strings read back (SAVE_CONFIG, restart) give the same command at connect
    module, printer, mcu = make_feed(dict(RESPONSES), options=staged)
    mcu.config_callback()
    cfg = [c for c, is_init in mcu.config_cmds if is_init]
    assert (
        "ace2k_feed_snag_set fwd_um=20000 lag_um=4001 duty_pct=0 back_um=2500 rest_ms=500"
        " free_um=199999" in cfg
    )
    assert gcmd.lines[-1].startswith("ace2k: snag fwd 20 mm, lag 4.001 mm, duty 0 %, back 2.5 mm")


@pytest.mark.parametrize(
    "key, arg",
    [
        ("snag_fwd_mm", "FWD"),
        ("snag_lag_mm", "LAG"),
        ("snag_back_mm", "BACK"),
        ("snag_free_mm", "FREE"),
    ],
)
def test_a_non_finite_snag_value_is_refused_not_raised(key, arg):
    # Klipper's float parser takes inf and nan; round() would raise on them, an internal error
    for text in ("inf", "-inf", "nan"):
        with pytest.raises(FakeConfig.error, match=rf"{key} \(.*\) must be a finite number"):
            make_feed(dict(RESPONSES), options={key: float(text)})
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    commands = list(mcu.commands)
    snag = dict(feed.snag)
    for text in ("inf", "-inf", "nan"):
        with pytest.raises(FakeGcmd.error, match=rf"{arg} \(.*\) must be a finite number"):
            feed.cmd_ACE_SNAG_SET(FakeGcmd({arg: text, "DUTY": "10"}))
    assert mcu.commands == commands and printer.objects["configfile"].calls == []
    assert feed.snag == snag


# --- the follow -----------------------------------------------------

FOLLOW_CONSTANTS = {
    "ACE2K_FEED_FOLLOW_FLIP_MS": 200,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MIN": 50,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MAX": 2000,
    "ACE2K_FEED_FOLLOW_TAKE_UM": 15000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MIN": 5000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MAX": 30000,
    "ACE2K_FEED_FOLLOW_TAIL_UM": 2000000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MIN": 200000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MAX": 2000000,
}
WITH_FOLLOW = dict(DEFAULT_CONSTANTS, **FOLLOW_CONSTANTS)
FOLLOW_FMT = "ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u"


def make_follow(responses=None, **kwargs):
    """A connected feed on an image that carries the follow."""
    responses = dict(RESPONSES) if responses is None else responses
    module, printer, mcu = make_feed(responses, constants=WITH_FOLLOW, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu, responses


def test_the_follow_constants_mirror_the_firmware_header():
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = {
        name: int(value)
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_FOLLOW_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    }
    assert defines == FOLLOW_CONSTANTS  # the fake here carries the firmware's own values


def test_the_follow_mode_names():
    assert ace2k_feed.CMD_MODES["assist_both"] == 6
    assert ace2k_feed.MODE_NAMES[8] == "following"


def test_assist_dir_both_starts_the_follow_at_the_given_speed():
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    gcmd = FakeGcmd({"LANE": "1", "DIR": "BOTH", "SPEED": "70"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert mcu.queries[-1][1] == [0, 6, 0, 70000, 1]
    assert gcmd.lines == ["ace2k: lane 1 assist_both started"]
    # lower case as the other directions, and the default speed without SPEED
    feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "2", "DIR": "both"}))
    assert mcu.queries[-1][1] == [1, 6, 0, 50000, 1]
    # DIR=FORWARD names the forward assist, as no DIR
    feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "3", "DIR": "FORWARD"}))
    assert mcu.queries[-1][1][:2] == [2, 2]


def test_assist_dir_both_soft_and_off_as_the_other_directions():
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 0, "reason": 1}
    gcmd = FakeGcmd({"LANE": "2", "DIR": "BOTH", "SOFT": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert gcmd.lines == ["ace2k: lane 2 not armed — lane 2 assist_both refused: busy"]
    with pytest.raises(Exception, match="lane 2 assist_both refused: busy"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "2", "DIR": "BOTH"}))
    gcmd = FakeGcmd({"LANE": "2", "DIR": "BOTH", "OFF": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [1])
    assert gcmd.lines == ["ace2k: lane 2 assist off"]


def test_an_unknown_assist_direction_is_an_error_naming_back_and_both():
    feed, printer, mcu, responses = make_follow()
    sent = len(mcu.queries)
    with pytest.raises(Exception, match="BACK or BOTH"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "1", "DIR": "SIDEWAYS"}))
    assert len(mcu.queries) == sent


def test_the_follow_on_an_image_without_it_is_refused_before_anything_is_sent():
    module, printer, mcu = make_feed(dict(RESPONSES))  # the default fake: no follow constants
    mcu.config_callback()
    feed = feed_of(module)
    assert not feed.has_follow
    assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    with pytest.raises(Exception, match=r"no follow tail \(or no follow\); flash v0.11.0"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    gcmd = FakeGcmd({"LANE": "1", "DIR": "BOTH", "SOFT": "1"})
    feed.cmd_ACE_ASSIST(gcmd)
    assert gcmd.lines == [
        "ace2k: lane 1 not armed — the unit's firmware has no follow tail (or no follow);"
        " flash v0.11.0"
    ]
    with pytest.raises(RuntimeError, match="no follow"):
        feed.start_move(0, "assist_both", 0.0, 70.0)
    with pytest.raises(Exception, match="no follow"):
        feed.cmd_ACE_FOLLOW_SET(FakeGcmd({"FLIP_MS": "300"}))
    assert mcu.queries == [] and not any(c[0] == FOLLOW_FMT for c in mcu.commands)
    # a follow key on such an image is a config error at connect
    module, printer, mcu = make_feed(dict(RESPONSES), options={"follow_flip_ms": 300})
    with pytest.raises(
        Exception, match=r"follow_flip_ms.*no follow tail \(or no follow\); flash v0.11.0"
    ):
        mcu.config_callback()


def test_start_move_assist_both_returns_a_move_and_following_is_busy():
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    assert isinstance(move, ace2k_feed.Move) and move.mode == "assist_both"
    assert mcu.queries[-1][1] == [0, 6, 0, 70000, 1]
    assert not feed.any_busy()
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([8, 0, 0, 0], [0] * 4, [70000, 0, 0, 0], [40, 0, 0, 0], [0] * 4, (1, 0, 0, 0))
    )
    assert feed.lane_status(0)["mode"] == "following"
    assert feed.any_busy()
    # the events carry the mode too
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 1, 8, 9000, 8000, seq=move.seq))
    assert move.done() and move.result()["mode"] == "following"


def test_the_follow_values_go_out_as_an_init_command_at_every_connect():
    feed, printer, mcu, _ = make_follow()
    follow = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_follow_set")]
    assert follow == [("ace2k_feed_follow_set flip_ms=200 take_um=15000 tail_um=2000000", True)]
    assert feed.follow == {"follow_flip_ms": 200, "follow_take_mm": 15.0, "follow_tail_mm": 2000.0}
    feed, printer, mcu, _ = make_follow(
        options={"follow_flip_ms": 300, "follow_take_mm": 20.0004, "follow_tail_mm": 750.5}
    )
    follow = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_follow_set")]
    assert follow == [("ace2k_feed_follow_set flip_ms=300 take_um=20000 tail_um=750500", True)]


def test_a_follow_key_out_of_the_dictionarys_bounds_is_a_config_error():
    for key, value in (
        ("follow_flip_ms", 49),
        ("follow_flip_ms", 2001),
        ("follow_take_mm", 4.999),
        ("follow_take_mm", 30.001),
        ("follow_take_mm", float("nan")),
        ("follow_tail_mm", 199.999),
        ("follow_tail_mm", 2000.001),
    ):
        module, printer, mcu = make_feed(
            dict(RESPONSES), options={key: value}, constants=WITH_FOLLOW
        )
        with pytest.raises(Exception, match=key):
            mcu.config_callback()
        assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    # the bounds are the dictionary's: an image with a wider window takes the value
    constants = dict(WITH_FOLLOW, ACE2K_FEED_FOLLOW_FLIP_MS_MAX=3000)
    module, printer, mcu = make_feed(
        dict(RESPONSES), options={"follow_flip_ms": 2500}, constants=constants
    )
    mcu.config_callback()
    assert (
        "ace2k_feed_follow_set flip_ms=2500 take_um=15000 tail_um=2000000",
        True,
    ) in mcu.config_cmds


def test_follow_set_sends_both_and_stages_what_was_given():
    feed, printer, mcu, _ = make_follow()
    gcmd = FakeGcmd({"FLIP_MS": "300", "TAKE_MM": "20", "TAIL_MM": "1500"})
    feed.cmd_ACE_FOLLOW_SET(gcmd)
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 20000, 1500000])
    assert printer.objects["configfile"].calls == [
        ("ace2k", "follow_flip_ms", "300"),
        ("ace2k", "follow_take_mm", "20"),
        ("ace2k", "follow_tail_mm", "1500"),
    ]
    assert gcmd.lines == [
        "ace2k: follow flip 300 ms, take 20 mm, tail 1500 mm; staged for SAVE_CONFIG"
    ]
    # the values in effect carry on: a later set changes only what it names
    feed.cmd_ACE_FOLLOW_SET(FakeGcmd({"TAKE_MM": "12.5"}))
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 12500, 1500000])
    feed.cmd_ACE_FOLLOW_SET(FakeGcmd({"TAIL_MM": "200"}))
    assert mcu.commands[-1] == (FOLLOW_FMT, [300, 12500, 200000])


def test_follow_set_alone_prints_and_a_value_out_of_bounds_changes_nothing():
    feed, printer, mcu, _ = make_follow()
    commands = list(mcu.commands)
    gcmd = FakeGcmd()
    feed.cmd_ACE_FOLLOW_SET(gcmd)
    assert gcmd.lines == ["ace2k: follow flip 200 ms, take 15 mm, tail 2000 mm"]
    for params in (
        {"FLIP_MS": "49"},
        {"FLIP_MS": "2001"},
        {"TAKE_MM": "4.9"},
        {"TAKE_MM": "31"},
        {"TAKE_MM": "inf"},
        {"TAIL_MM": "199.9"},
        {"TAIL_MM": "2001"},
        {"TAIL_MM": "nan"},
        {"FLIP_MS": "300", "TAKE_MM": "40"},  # a good one beside a bad one
        {"TAKE_MM": "20", "TAIL_MM": "5000"},
    ):
        with pytest.raises(Exception, match="FLIP_MS|TAKE_MM|TAIL_MM"):
            feed.cmd_ACE_FOLLOW_SET(FakeGcmd(params))
    assert mcu.commands == commands
    assert printer.objects["configfile"].calls == []
    assert feed.follow == {"follow_flip_ms": 200, "follow_take_mm": 15.0, "follow_tail_mm": 2000.0}


def test_follow_set_on_a_firmware_without_the_feed_is_a_gcode_error():
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(Exception, match="CONFIG_ACE2K_FEED"):
        feed_of(module).cmd_ACE_FOLLOW_SET(FakeGcmd({"FLIP_MS": "300"}))
    assert mcu.commands == []


# --- the tail --------------------------------------------------------

V0_10_FOLLOW = {k: v for k, v in WITH_FOLLOW.items() if "_FOLLOW_TAIL_" not in k}


def test_the_tail_kinds_a_notice_and_a_final_that_is_no_error():
    assert ace2k_feed.KIND_NAMES[16] == "tail" and ace2k_feed.KIND_NAMES[17] == "tail_out"
    assert 16 in ace2k_feed.NOTICE_KINDS and 16 not in ace2k_feed.ERROR_KINDS
    assert 17 not in ace2k_feed.NOTICE_KINDS and 17 not in ace2k_feed.ERROR_KINDS


# --- blocked ---------------------------------------------------------


def test_blocked_a_final_kind_that_is_no_error():
    assert ace2k_feed.KIND_NAMES[18] == "blocked"
    assert 18 not in ace2k_feed.ERROR_KINDS and 18 not in ace2k_feed.NOTICE_KINDS


def test_blocked_completes_a_move_the_lane_idle_without_an_error():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    move = feed.start_move(1, "feed", 100.0, 30.0)
    assert not move.done()
    mcu.subscriptions["ace2k_feed_event"](event_params(1, 18, 1, 42000, 9000, seq=move.seq))
    assert move.done() and feed.completions == {}
    assert move.result()["kind"] == "blocked" and move.result()["mode"] == "feeding"
    assert move.result()["motor_mm"] == 42.0 and move.result()["filament_mm"] == 9.0
    assert feed.lanes[1]["last_move"]["kind"] == "blocked"
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (0, 1, 0, 0))
    )
    assert feed.lanes[1]["mode"] == "idle" and feed.lanes[1]["error"] is None


def test_a_waited_feed_or_load_ending_blocked_answers_a_line_naming_the_odometers():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = accepted(mcu, event_params(0, 18, 1, 42000, 9000, seq=1))
    gcmd = FakeGcmd({"LANE": "1", "LENGTH": "100"})
    feed.cmd_ACE_FEED(gcmd)  # no G-code error
    assert gcmd.lines[-1] == "ace2k: lane 1 feed blocked — motor 42.0 mm, filament 9.0 mm"
    responses["ace2k_feed_start"] = accepted(mcu, event_params(3, 18, 4, 180000, 175500, seq=1))
    gcmd = FakeGcmd({"LANE": "4"})
    feed.cmd_ACE_LOAD(gcmd)
    assert gcmd.lines[-1] == "ace2k: lane 4 load blocked — motor 180.0 mm, filament 175.5 mm"
    assert feed.completions == {}


def test_the_tail_notice_completes_no_move_and_the_tail_out_completes_the_follow():
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    deliver = mcu.subscriptions["ace2k_feed_event"]
    deliver(event_params(0, 16, 8, 400000, 398000, seq=move.seq))
    assert not move.done() and feed.completions[0][0] == move.seq
    assert feed.lanes[0]["last_event"]["kind"] == "tail"
    assert feed.lanes[0]["last_move"] is None
    deliver(event_params(0, 17, 8, 650000, 610000, seq=move.seq))
    assert move.done() and feed.completions == {}
    assert move.result()["kind"] == "tail_out" and move.result()["mode"] == "following"
    assert move.result()["motor_mm"] == 650.0 and move.result()["filament_mm"] == 610.0
    assert feed.lanes[0]["last_move"]["kind"] == "tail_out"
    assert feed.lanes[0]["error"] is None


def test_a_waited_follow_ends_on_tail_out_without_a_gcode_error():
    feed, printer, mcu, responses = make_follow()
    responses["ace2k_feed_start"] = accepted(
        mcu,
        event_params(0, 16, 8, 400000, 398000, seq=1),
        event_params(0, 17, 8, 650000, 610000, seq=1),
    )
    move = feed.start_move(0, "assist_both", 0.0, 70.0)
    assert move.done() and move.result()["kind"] == "tail_out"


def test_the_lanes_tail_from_the_notice_to_leaving_the_follow():
    feed, printer, mcu, responses = make_follow()
    state = mcu.subscriptions["ace2k_feed_state"]
    deliver = mcu.subscriptions["ace2k_feed_event"]
    following = state_frame([8, 0, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    idle = state_frame([0] * 4, [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    assert all(feed.lane_status(i)["tail"] is False for i in range(ace2k_feed.LANES))
    state(following)
    assert feed.lane_status(0)["tail"] is False
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True and feed.lane_status(1)["tail"] is False
    state(following)
    assert feed.lane_status(0)["tail"] is True  # still following
    state(idle)  # the lane left following (its final event lost on the way)
    assert feed.lane_status(0)["tail"] is False
    # the final event ends it too, the report not yet in
    state(following)
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True
    deliver(event_params(0, 17, 8, 9000, 8000, seq=1))
    assert feed.lane_status(0)["tail"] is False
    # a notice of another kind leaves it as it is
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    deliver(event_params(0, 14, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True


def sensors_frame(inserts):
    """One ace2k_sensors_state frame with the given lanes' inserts up, nothing else."""
    switches = sum(1 << i for i, up in enumerate(inserts) if up)
    return {
        "switches": switches,
        "insert_mv": b"\x00" * 8,
        "empty_mv": b"\x00" * 8,
        "aux_mv": 0,
        "age_ms": 0,
        "#sent_time": 1.0,
    }


def make_tail_unit():
    """[ace2k] with the feed on an image with the follow, lane 1 following."""
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.unit is module
    mcu.subscriptions["ace2k_feed_state"](
        state_frame([8, 0, 0, 0], [0] * 4, [0] * 4, [0] * 4, [0] * 4, (1, 0, 0, 0))
    )
    return module, feed, mcu


def test_the_lanes_tail_outlives_a_filament_pushed_in_at_the_bay():
    # the tail ignores the bay: the insert rising during the tail
    # changes nothing, the tail ends on its final event or the lane leaving the follow
    module, feed, mcu = make_tail_unit()
    sensors = mcu.subscriptions["ace2k_sensors_state"]
    deliver = mcu.subscriptions["ace2k_feed_event"]
    sensors(sensors_frame([True, True, True, True]))
    deliver(event_params(0, 16, 8, 1000, 1000, seq=1))
    assert feed.lane_status(0)["tail"] is True
    sensors(sensors_frame([False, True, True, True]))  # the fall reported
    assert feed.lane_status(0)["tail"] is True
    sensors(sensors_frame([True, True, True, True]))  # a filament pushed in at the bay
    assert feed.lane_status(0)["tail"] is True
    assert module.get_status(0.0)["lanes"][0]["tail"] is True
    sensors(sensors_frame([False, True, True, True]))  # and out again, fast
    sensors(sensors_frame([True, True, True, True]))
    assert feed.lanes[0]["tail"] is True
    deliver(event_params(0, 17, 8, 650000, 610000, seq=1))  # the tail-out ends it
    assert feed.lane_status(0)["tail"] is False
    assert module.get_status(0.0)["lanes"][0]["tail"] is False
    assert feed.lanes[0]["last_move"]["kind"] == "tail_out"


def test_the_feed_holds_no_insert_watch_of_its_own():
    module, feed, mcu = make_tail_unit()
    assert not hasattr(feed, "on_inserts") and not hasattr(feed, "tail_insert_down")
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 16, 8, 1000, 1000, seq=1))
    module.lanes[0]["insert"] = True  # whatever the insert reads, the status only reads
    assert feed.lane_status(0)["tail"] is True and feed.lanes[0]["tail"] is True


def test_an_image_without_the_tail_refuses_only_the_follow_with_the_flash_line():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses, constants=V0_10_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present and not feed.has_follow
    assert not any(c.startswith("ace2k_feed_follow_set") for c, _ in mcu.config_cmds)
    with pytest.raises(Exception, match=r"no follow tail \(or no follow\); flash v0.11.0"):
        feed.cmd_ACE_FOLLOW_SET(FakeGcmd({"TAIL_MM": "500"}))
    with pytest.raises(Exception, match="flash v0.11.0"):
        feed.cmd_ACE_ASSIST(FakeGcmd({"LANE": "1", "DIR": "BOTH"}))
    assert not any(c[0].startswith("ace2k_feed_follow_set") for c in mcu.commands)
    # the rest of the feed works on that image
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries[-1][1] == [0, 0, 50000, 30000, 1] and not move.done()
    # a follow key there, the tail's or another, is a config error naming the flash
    for key, value in (("follow_tail_mm", 500.0), ("follow_flip_ms", 300)):
        module, printer, mcu = make_feed(
            dict(RESPONSES), options={key: value}, constants=V0_10_FOLLOW
        )
        with pytest.raises(
            Exception, match=rf"{key}.*no follow tail \(or no follow\); flash v0.11.0"
        ):
            mcu.config_callback()


# --- the host interface -----------------------------------------------------------------------


def test_api_version():
    assert ace2k_feed.API_VERSION == 6


def test_the_feed_object_carries_the_api_version():
    # a caller holds printer.lookup_object("ace2k").feed, not the module
    module, printer, mcu = make_feed(dict(RESPONSES))
    assert feed_of(module).API_VERSION == ace2k_feed.API_VERSION == 6


def test_start_move_returns_a_move_completed_by_its_final_event():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries[-1][1] == [0, 0, 50000, 30000, 1]
    assert move.lane == 0 and move.mode == "feed" and move.seq == 1
    assert not move.done() and move.result() is None
    assert feed.completions[0][0] == 1
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 50000, 49500, seq=move.seq))
    assert move.done() and feed.completions == {}
    assert move.result()["kind"] == "done" and move.result()["filament_mm"] == 49.5
    assert move.wait(1.0)["kind"] == "done"
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 50.0


def test_start_move_refused_raises_feed_refused_with_the_reason():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 3}
    with pytest.raises(ace2k_feed.FeedRefused) as err:
        feed.start_move(0, "feed", 50.0, 30.0)
    assert err.value.reason == "no_filament" and err.value.lane == 0
    assert str(err.value) == "ace2k: lane 1 feed refused: no_filament"
    assert 0 not in feed.completions and feed.commanded_mm[0] == {}


def test_a_notice_or_another_sequence_does_not_complete_a_move():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 1, "accepted": 1, "reason": 0}
    move = feed.start_move(1, "assist", 0.0, 50.0)
    deliver = mcu.subscriptions["ace2k_feed_event"]
    deliver(event_params(1, 15, 2, 5000, 4000, seq=move.seq))  # snag, a notice
    deliver(event_params(1, 14, 2, 5000, 4000, seq=move.seq))  # behind, a notice
    deliver(event_params(1, 16, 8, 5000, 4000, seq=move.seq))  # tail, a notice
    deliver(event_params(1, 1, 2, 5000, 4000, seq=0))  # a motion the unit started itself
    assert not move.done() and move.result() is None
    assert move.wait(0.0) is None
    deliver(event_params(1, 1, 2, 9000, 8000, seq=move.seq))  # stopped
    assert move.done() and move.result()["kind"] == "stopped"


def test_start_move_without_the_feed_or_with_a_bad_mode_raises_and_sends_nothing():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    with pytest.raises(ValueError):
        feed.start_move(0, "sideways", 50.0, 30.0)
    with pytest.raises(ValueError):
        feed.start_move(4, "feed", 50.0, 30.0)
    assert "ace2k_feed_start" not in mcu.sent and feed.completions == {}
    feed.present = False
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed.start_move(0, "feed", 50.0, 30.0)


def test_stop_and_clear_send_their_commands():
    module, printer, mcu = make_feed(dict(RESPONSES))
    mcu.config_callback()
    feed = feed_of(module)
    feed.stop(2)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [2])
    feed.clear(1)
    assert mcu.commands[-1] == ("ace2k_feed_clear lane=%c", [1])
    feed.stop(ace2k_feed.LANE_ALL)
    assert mcu.commands[-1] == ("ace2k_feed_stop lane=%c", [255])
    # another lane is refused before anything is sent; LANE_ALL is a stop's only
    sent = list(mcu.commands)
    for bad in (4, -1):
        with pytest.raises(ValueError):
            feed.stop(bad)
        with pytest.raises(ValueError):
            feed.clear(bad)
    with pytest.raises(ValueError):
        feed.clear(ace2k_feed.LANE_ALL)
    assert mcu.commands == sent


def test_a_refused_start_leaves_the_running_move_tracked():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    first = feed.start_move(0, "feed", 500.0, 30.0)
    # a second start on the busy lane: the unit refuses it, and the first move is still the
    # lane's — through the interface and through a waiting G-code alike
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert feed.completions[0][0] == first.seq
    with pytest.raises(Exception, match="lane 1 feed refused: busy"):
        feed.cmd_ACE_FEED(FakeGcmd({"LANE": "1", "LENGTH": "50"}))
    assert feed.completions[0][0] == first.seq
    mcu.subscriptions["ace2k_feed_event"](event_params(0, 0, 1, 500000, 499000, seq=first.seq))
    assert first.done() and first.result()["kind"] == "done" and feed.completions == {}
    assert feed.lanes[0]["last_move"]["commanded_mm"] == 500.0


def test_a_running_moves_end_inside_a_refused_starts_round_trip_completes_it_once():
    # the earlier move ends just as the next start goes out: its final event is dispatched
    # while the refused start holds the lane's slot, and the refusal completes it from that event
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    first = feed.start_move(0, "feed", 50.0, 30.0)
    completed = []
    original = first._completion.complete

    def counting(result):
        completed.append(result)
        original(result)

    first._completion.complete = counting
    final = event_params(0, 0, 1, 50000, 49000, seq=first.seq)

    def busy_with_the_end_in_flight():
        mcu.subscriptions["ace2k_feed_event"](final)
        assert not first.done()  # dispatched while the refused start held the slot
        return {"lane": 0, "accepted": 0, "reason": 1}

    responses["ace2k_feed_start"] = busy_with_the_end_in_flight
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert first.done() and first.result()["kind"] == "done" and feed.completions == {}
    assert first.result() is feed.lanes[0]["last_event"]
    # the same event dispatched again (a _dispatch still queued) completes nothing twice
    feed._dispatch(0, first.result(), 0)
    assert len(completed) == 1


def test_a_move_is_done_before_the_klipper_event_fires():
    responses = dict(RESPONSES)
    module, printer, mcu = make_feed(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 2, "accepted": 1, "reason": 0}
    move = feed.start_move(2, "rollback", 40.0, 30.0)
    seen = []

    def send_event(name, *args):
        seen.append(move.done())
        raise RuntimeError("a listener that fails")

    printer.send_event = send_event
    with pytest.raises(RuntimeError, match="a listener that fails"):
        mcu.subscriptions["ace2k_feed_event"](event_params(2, 0, 2, 40000, 39000, seq=move.seq))
    assert seen == [True] and move.result()["kind"] == "done"


def test_start_move_before_any_state_report_is_a_runtime_error():
    # no report within the bound (the G-code path's text is the same, as
    # test_a_start_before_the_first_state_report_waits_for_it_then_sends_reported_plus_one checks)
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    feed = feed_of(module)
    responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
    with pytest.raises(RuntimeError, match=r"no feed state report within 7 s"):
        feed.start_move(0, "feed", 50.0, 30.0)
    assert mcu.queries == [] and not feed.started[0] and feed.completions == {}
    # the report landing during the wait: the start goes out, seeded from it
    printer.reactor.while_waiting[feed.first_report] = [
        lambda waketime: mcu.subscriptions["ace2k_feed_state"](IDLE_FRAME)
    ]
    move = feed.start_move(0, "feed", 50.0, 30.0)
    assert move.seq == 1 and mcu.queries[-1][1] == [0, 0, 50000, 30000, 1]


# --- the feed-forward ------------------------------------------

FF_CONSTANTS = {
    "ACE2K_FEED_FF_CHUNK_UM": 3000,
    "ACE2K_FEED_FF_CHUNK_UM_MIN": 1000,
    "ACE2K_FEED_FF_CHUNK_UM_MAX": 10000,
    "ACE2K_FEED_FF_PULSE_UM_S": 15000,
    "ACE2K_FEED_FF_PULSE_UM_S_MIN": 9000,
    "ACE2K_FEED_FF_PULSE_UM_S_MAX": 70000,
}
WITH_FF = dict(WITH_FOLLOW, **FF_CONSTANTS)
FF_SET_FMT = "ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u"


def make_ff_feed(responses=None, **kwargs):
    """A connected feed on an image that carries the feed-forward."""
    responses = dict(RESPONSES) if responses is None else responses
    module, printer, mcu = make_feed(responses, constants=WITH_FF, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu, module


def ff_state_frame(doses, taut, full, epoch=(0, 0, 0, 0)):
    return {
        "doses": struct.pack("<4H", *doses),
        "taut": struct.pack("<4H", *taut),
        "full": struct.pack("<4H", *full),
        "epoch": bytes(epoch),
    }


def test_the_ff_constants_mirror_the_firmware_header():
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = dict(
        (name, int(value))
        for name, value in re.findall(
            r"^#define\s+(ACE2K_FEED_FF_\w+)\s+(\d+)U\b", header, flags=re.M
        )
    )
    # the pulse's bounds are the lane's speed bounds, by name in the header
    assert defines == {k: v for k, v in FF_CONSTANTS.items() if "PULSE_UM_S_M" not in k}
    assert (
        FF_CONSTANTS["ACE2K_FEED_FF_PULSE_UM_S_MIN"]
        == DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MIN_UM_S"]
    )
    assert (
        FF_CONSTANTS["ACE2K_FEED_FF_PULSE_UM_S_MAX"]
        == DEFAULT_CONSTANTS["ACE2K_LANE_SPEED_MAX_UM_S"]
    )


def test_the_ff_values_and_query_go_out_as_init_commands_at_every_connect():
    feed, printer, mcu, _ = make_ff_feed()
    init = [c for c, is_init in mcu.config_cmds if is_init and c.startswith("ace2k_feed_")]
    assert init == [
        "ace2k_feed_load_set park_um=300000 speed_um_s=30000 auto_load=1",
        "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k_feed_follow_set flip_ms=200 take_um=15000 tail_um=2000000",
        "ace2k_feed_ff_set chunk_um=3000 pulse_um_s=15000",
        "ace2k_feed_query rest_ticks=120000000",
        "ace2k_feed_ff_query rest_ticks=120000000",
    ]
    assert feed.has_ff and "ace2k_feed_ff_state" in mcu.subscriptions
    feed, printer, mcu, _ = make_ff_feed(options={"ff_chunk_mm": 4.5, "ff_pulse": 20})
    assert ("ace2k_feed_ff_set chunk_um=4500 pulse_um_s=20000", True) in mcu.config_cmds


def test_an_ff_key_out_of_the_dictionarys_bounds_is_a_config_error():
    for key, value in (
        ("ff_chunk_mm", 0.9),
        ("ff_chunk_mm", 10.001),
        ("ff_pulse", 8.9),
        ("ff_pulse", 71),
    ):
        module, printer, mcu = make_feed(dict(RESPONSES), options={key: value}, constants=WITH_FF)
        with pytest.raises(Exception, match=key):
            mcu.config_callback()


def test_an_image_without_the_ff_constants_gets_no_ff_command():
    feed, printer, mcu, _ = make_follow(options={"ff_chunk_mm": 4})
    assert not feed.has_ff
    assert not any("_ff_" in c for c, _ in mcu.config_cmds)
    assert "ace2k_feed_ff_state" not in mcu.subscriptions
    with pytest.raises(Exception, match="flash v0.9.0"):
        feed.cmd_ACE_FF_SET(FakeGcmd({"CHUNK_MM": "4"}))
    assert mcu.commands == []


def test_ff_set_sends_both_checks_the_bounds_and_tunes_the_host_side():
    feed, printer, mcu, _ = make_ff_feed()
    gcmd = FakeGcmd({"CHUNK_MM": "4", "PULSE": "20"})
    feed.cmd_ACE_FF_SET(gcmd)
    assert mcu.commands[-1] == (FF_SET_FMT, [4000, 20000])
    assert ("ace2k", "ff_chunk_mm", "4") in printer.objects["configfile"].calls
    feed.cmd_ACE_FF_SET(FakeGcmd({"PULSE": "12.5"}))
    assert mcu.commands[-1] == (FF_SET_FMT, [4000, 12500])
    commands = list(mcu.commands)
    for params in (
        {"CHUNK_MM": "0.5"},
        {"CHUNK_MM": "11"},
        {"PULSE": "8"},
        {"PULSE": "71"},
        {"PULSE": "nan"},
        {"CHUNK_MM": "3", "PULSE": "100"},  # a good one beside a bad one
        {"WINDOW_MS": "10"},
        {"LEAD_MS": "-1"},
        {"DEADBAND": "-0.1"},
    ):
        with pytest.raises(Exception, match="CHUNK_MM|PULSE|WINDOW_MS|LEAD_MS|DEADBAND"):
            feed.cmd_ACE_FF_SET(FakeGcmd(params))
    assert mcu.commands == commands
    # the host-side values: nothing sent to the unit
    feed.cmd_ACE_FF_SET(FakeGcmd({"WINDOW_MS": "800", "LEAD_MS": "100", "DEADBAND": "0.5"}))
    assert mcu.commands == commands
    assert (feed.ff.window_ms, feed.ff.lead_ms, feed.ff.deadband) == (800, 100, 0.5)
    gcmd = FakeGcmd()
    feed.cmd_ACE_FF_SET(gcmd)
    assert gcmd.lines == [
        "ace2k: feed-forward chunk 4 mm, pulse 12.5 mm/s, window 800 ms, lead 100 ms,"
        " deadband 0.5 mm/s"
    ]


def test_the_ff_frame_lands_in_lane_status():
    feed, printer, mcu, module = make_ff_feed()
    lane = module.get_status(0)["lanes"][2]
    assert (lane["ff_doses"], lane["ff_taut_fixes"], lane["ff_full_fixes"]) == (None, None, None)
    assert lane["ff_rate"] == 0.0 and lane["ff_on"] is True and lane["ff_extruder"] is None
    mcu.subscriptions["ace2k_feed_ff_state"](
        ff_state_frame((1, 2, 300, 4), (0, 0, 7, 0), (0, 0, 5, 0))
    )
    lane = module.get_status(0)["lanes"][2]
    assert (lane["ff_doses"], lane["ff_taut_fixes"], lane["ff_full_fixes"]) == (300, 7, 5)


# --- the automatic encoder calibration ------------------

SCALE_FMT = "ace2k_lane_scale_set lane=%c um_per_count_x10=%u"


def test_apply_scale_puts_the_scale_in_use_keeps_the_encoder_continuous_and_stages_it():
    feed, printer, mcu, module = make_ff_feed()
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1000000, 0, 0, 0], [0] * 4))
    feed.apply_scale(0, 1.2468)
    assert mcu.commands == [(SCALE_FMT, [0, 12468])]
    # the first frame at the new scale: the offset reckoned from the scale it replaces,
    # 810.24 counts × (1.2342 - 1.2468) mm
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1010209, 0, 0, 0], [0] * 4))
    assert module.encoder_offset_um == [-10209, 0, 0, 0]
    assert module.get_status(0.0)["lanes"][0]["encoder_mm"] == 1000.0
    lane = feed.lanes[0]
    assert (lane["encoder_scale"], lane["scale_source"]) == (1.2468, "calibrated")
    assert feed.scales == {0: 1.2468}
    assert printer.objects["configfile"].calls == [("ace2k", "lane1_encoder_scale", "1.2468")]
    assert "scale=1.2468 (calibrated)" in feed.status_lines()[0]
    feed.apply_scale(2, 1.25)  # staged to four decimals, as the manual mode stages it
    assert mcu.commands[-1] == (SCALE_FMT, [2, 12500])
    assert printer.objects["configfile"].calls[-1] == ("ace2k", "lane3_encoder_scale", "1.2500")
    # lane 3 at 0: no offset to keep, nothing pending
    assert module.encoder_offset_um == [-10209, 0, 0, 0] and module.scale_pending == [None] * 4


def test_apply_scale_takes_the_reference_before_the_send():
    # the unit cannot be on the new scale before it has the command: a frame handled between the
    # reference and the send is at the old scale, and one at the new scale can only come after
    feed, printer, mcu, module = make_ff_feed()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    old_raw, new_raw = 810 * 12342 // 10, 810 * 12468 // 10  # 810 counts at each scale
    counters(lane_counters_frame([old_raw, 0, 0, 0], [0] * 4))
    lookup = mcu.lookup_command

    def lookup_racing(msgformat, cq=None):
        cmd = lookup(msgformat, cq)
        if msgformat == SCALE_FMT:
            send = cmd.send

            def racing_send(data=(), minclock=0, reqclock=0):
                counters(lane_counters_frame([old_raw, 0, 0, 0], [0] * 4))  # was on its way
                send(data)
                counters(lane_counters_frame([new_raw, 0, 0, 0], [0] * 4))  # taken at once

            cmd.send = racing_send
        return cmd

    mcu.lookup_command = lookup_racing
    feed.apply_scale(0, 1.2468)
    readings = [module.get_status(0.0)["lanes"][0]["encoder_mm"]]
    for _ in range(3):  # at rest
        counters(lane_counters_frame([new_raw, 0, 0, 0], [0] * 4))
        readings.append(module.get_status(0.0)["lanes"][0]["encoder_mm"])
    assert readings == [999.702] * 4 and module.scale_pending[0] is None
    assert mcu.commands == [(SCALE_FMT, [0, 12468])]


def test_apply_scale_refuses_a_lane_or_a_scale_the_unit_would_not_take():
    feed, printer, mcu, module = make_ff_feed()
    for index, scale in ((4, 1.2468), (-1, 1.2468), (0, 1.49), (0, 0.98), (0, float("nan"))):
        with pytest.raises(ValueError, match=r"no lane|outside the unit's window"):
            feed.apply_scale(index, scale)
    assert mcu.commands == [] and printer.objects["configfile"].calls == []
    assert feed.lanes[0]["encoder_scale"] == 1.2342 and feed.scales == {}
    # the window's own ends are the unit's: 9874 and 14810 tenths of a µm per count
    feed.apply_scale(1, feed.scale_low)
    feed.apply_scale(2, feed.scale_high)
    assert mcu.commands == [(SCALE_FMT, [1, 9874]), (SCALE_FMT, [2, 14810])]


def test_every_connect_sends_each_lanes_scale_so_a_live_one_ends_with_the_session():
    # the session: lane 1's scale applied live and staged, never saved
    feed, printer, mcu, _ = make_ff_feed(options={"lane2_encoder_scale": 1.25})
    feed.apply_scale(0, 1.2468)
    # a host restart finds the unit configured, its CRC unchanged: Klipper sends it only the
    # init commands, which carry every lane's scale as the new session holds it
    feed, printer, mcu, _ = make_ff_feed(options={"lane2_encoder_scale": 1.25})
    scales = [(c, is_init) for c, is_init in mcu.config_cmds if c.startswith("ace2k_lane_scale")]
    assert [c for c, is_init in scales if is_init] == [
        "ace2k_lane_scale_set lane=0 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=1 um_per_count_x10=12500",
        "ace2k_lane_scale_set lane=2 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=3 um_per_count_x10=12342",
    ]
    assert feed.lanes[0]["encoder_scale"] == 1.2342
    # the key stays a config command too: editing it restarts the MCU
    assert [c for c, is_init in scales if not is_init] == [
        "ace2k_lane_scale_set lane=1 um_per_count_x10=12500"
    ]


def test_apply_scale_on_a_feed_loaded_alone_sends_and_stages():
    printer = FakePrinter(FakeMcu(dict(RESPONSES)))
    feed = ace2k_feed.load_config(FakeConfig(printer))
    assert feed.unit is None
    feed.apply_scale(1, 1.25)
    assert feed.mcu.commands == [(SCALE_FMT, [1, 12500])]
    assert printer.objects["configfile"].calls == [("ace2k", "lane2_encoder_scale", "1.2500")]


def test_set_counts_period_sends_the_ff_query_only_on_an_image_with_the_feed_forward():
    feed, printer, mcu, _ = make_ff_feed()
    feed.set_counts_period(0.1)
    assert mcu.commands == [("ace2k_feed_ff_query rest_ticks=%u", [12000000])]
    feed, printer, mcu, _ = make_follow()
    feed.set_counts_period(0.1)
    assert mcu.commands == [] and not feed.has_ff


def test_the_ff_frame_reaches_the_calibration_with_its_receive_time():
    feed, printer, mcu, _ = make_ff_feed()
    heard = []
    feed.cal.on_ff_state = lambda rt, taut, epoch: heard.append((rt, taut, tuple(epoch)))
    frame = ff_state_frame((0, 0, 0, 0), (5, 6, 7, 8), (0, 0, 0, 0), epoch=(1, 2, 3, 4))
    mcu.subscriptions["ace2k_feed_ff_state"](dict(frame, **{"#receive_time": 12.5}))
    mcu.subscriptions["ace2k_feed_ff_state"](frame)  # none in the frame: 0.0
    assert heard == [(12.5, (5, 6, 7, 8), (1, 2, 3, 4)), (0.0, (5, 6, 7, 8), (1, 2, 3, 4))]


def test_the_calibration_keeps_its_lanes_frames_only_while_a_run_holds_it():
    feed, printer, mcu, _ = make_ff_feed()
    counts = ff_state_frame((0, 0, 0, 0), (5, 6, 7, 8), (0, 0, 0, 0), epoch=(1, 2, 3, 4))
    reading = lane_counters_frame([1000, 2000, 2468, 4000], [0] * 4)
    mcu.subscriptions["ace2k_feed_ff_state"](dict(counts, **{"#receive_time": 12.0}))
    mcu.subscriptions["ace2k_lane_counters"](dict(reading, **{"#receive_time": 12.1}))
    assert list(feed.cal.frames) == []  # between runs
    feed.cal.lane = 2
    mcu.subscriptions["ace2k_feed_ff_state"](dict(counts, **{"#receive_time": 12.2}))
    mcu.subscriptions["ace2k_lane_counters"](dict(reading, **{"#receive_time": 12.3}))
    assert list(feed.cal.frames) == [("counts", 12.2, 7, 3), ("reading", 12.3, 2.468)]


# --- the grip ---------------------------------------------------------

GRIP_CONSTANTS = {
    "ACE2K_FEED_GRIP_UM": 40000,
    "ACE2K_FEED_GRIP_UM_MIN": 10000,
    "ACE2K_FEED_GRIP_UM_MAX": 45000,
}
WITH_GRIP = dict(WITH_FOLLOW, **GRIP_CONSTANTS)
GRIP_FMT = "ace2k_feed_lane_grip lane=%c grip_um=%u"


def make_grip(**kwargs):
    """A connected feed on an image that carries the grip."""
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_GRIP, **kwargs)
    mcu.config_callback()
    return feed_of(module), printer, mcu


def test_the_grip_constants_mirror_the_firmware_header():
    header = (pathlib.Path(__file__).parents[2] / "src" / "ace2k" / "feed" / "feed.h").read_text()
    defines = dict(re.findall(r"^#define\s+(ACE2K_FEED_\w+)\s+(\d+)U\b", header, flags=re.M))
    assert int(defines["ACE2K_FEED_GRIP_UM"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM"]
    assert int(defines["ACE2K_FEED_GRIP_UM_MIN"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MIN"]
    assert int(defines["ACE2K_FEED_GRIP_UM_MAX"]) == GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MAX"]
    # the upper bound stops short of the load's grace (strict there): the whole grip never meets
    # the comparator
    assert GRIP_CONSTANTS["ACE2K_FEED_GRIP_UM_MAX"] < int(defines["ACE2K_FEED_LOAD_GRACE_UM"])


def test_the_grip_is_read_from_the_dictionary_and_every_lanes_cleared_at_every_connect():
    # the grip lives in the unit's RAM: a host restart that leaves the MCU running sends it only
    # the init commands, which clear a grip the last session set
    for _ in range(2):  # a first connect, and a host restart
        feed, printer, mcu = make_grip()
        assert feed.has_grip
        assert (feed.grip_mm, feed.grip_min_mm, feed.grip_max_mm) == (40.0, 10.0, 45.0)
        grips = [(c, i) for c, i in mcu.config_cmds if c.startswith("ace2k_feed_lane_grip")]
        assert grips == [(f"ace2k_feed_lane_grip lane={lane} grip_um=0", True) for lane in range(4)]
        assert mcu.commands == []


def test_an_image_without_the_grip_gets_no_clear_at_connect():
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    assert not any(c.startswith("ace2k_feed_lane_grip") for c, _ in mcu.config_cmds)


def test_set_lane_grip_sends_the_lanes_grip_and_zero_clears():
    feed, printer, mcu = make_grip()
    feed.set_lane_grip(2, 40.0)
    feed.set_lane_grip(0, 10.0)
    feed.set_lane_grip(3, 45.0)
    feed.set_lane_grip(2, 0)
    assert mcu.commands == [
        (GRIP_FMT, [2, 40000]),
        (GRIP_FMT, [0, 10000]),
        (GRIP_FMT, [3, 45000]),
        (GRIP_FMT, [2, 0]),
    ]


def test_set_lane_grip_refuses_a_grip_out_of_bounds_or_another_lane_before_sending():
    feed, printer, mcu = make_grip()
    for grip in (9.999, 45.001, 50.0, 5.0, 300.0):
        with pytest.raises(ValueError, match=r"outside the unit's 10\.\.45 mm \(0 clears\)"):
            feed.set_lane_grip(0, grip)
    for lane in (-1, 4, 255):
        with pytest.raises(ValueError, match="no lane"):
            feed.set_lane_grip(lane, 40.0)
    assert mcu.commands == []


def test_an_image_without_the_grip_says_flash_and_sends_nothing():
    module, printer, mcu = make_feed(dict(RESPONSES), constants=WITH_FOLLOW)
    mcu.config_callback()
    feed = feed_of(module)
    assert feed.present and not feed.has_grip
    with pytest.raises(RuntimeError, match="no grip; flash v0.11.0"):
        feed.set_lane_grip(0, 40.0)
    with pytest.raises(Exception, match="no grip; flash v0.11.0"):
        feed.cmd_ACE_GRIP(FakeGcmd({"LANE": "1", "MM": "40"}))
    assert mcu.commands == []


def test_a_grip_command_of_another_format_is_refused_at_connect():
    old = "ace2k_feed_lane_grip lane=%c grip_um=%hu"
    module, printer, mcu = make_feed(
        dict(RESPONSES), constants=WITH_GRIP, formats={"ace2k_feed_lane_grip": old}
    )
    with pytest.raises(Exception, match="ace2k_feed_lane_grip has another format"):
        mcu.config_callback()


def test_set_lane_grip_on_a_firmware_without_the_feed_is_a_runtime_error():
    module, printer, mcu = make_feed(dict(RESPONSES), known={"ace2k_version"})
    mcu.config_callback()
    with pytest.raises(RuntimeError, match="CONFIG_ACE2K_FEED"):
        feed_of(module).set_lane_grip(0, 40.0)


def test_ace_grip_sets_the_given_or_the_default_grip_and_mm_zero_clears():
    feed, printer, mcu = make_grip()
    gcmd = FakeGcmd({"LANE": "2", "MM": "30"})
    feed.cmd_ACE_GRIP(gcmd)
    assert mcu.commands[-1] == (GRIP_FMT, [1, 30000])
    assert gcmd.lines == [
        "ace2k: lane 2 grip 30 mm: its next automatic load pulls only that, without the tag search"
    ]
    feed.cmd_ACE_GRIP(FakeGcmd({"LANE": "1"}))  # the firmware's default
    assert mcu.commands[-1] == (GRIP_FMT, [0, 40000])
    gcmd = FakeGcmd({"LANE": "1", "MM": "0"})
    feed.cmd_ACE_GRIP(gcmd)
    assert mcu.commands[-1] == (GRIP_FMT, [0, 0])
    assert gcmd.lines == ["ace2k: lane 1 grip cleared"]


def test_ace_grip_out_of_bounds_is_a_gcode_error_and_sends_nothing():
    feed, printer, mcu = make_grip()
    for mm in ("5", "46", "50"):
        with pytest.raises(Exception, match=r"outside the unit's 10\.\.45 mm"):
            feed.cmd_ACE_GRIP(FakeGcmd({"LANE": "1", "MM": mm}))
    with pytest.raises(Exception, match="MM of at least 0"):
        feed.cmd_ACE_GRIP(FakeGcmd({"LANE": "1", "MM": "-1"}))
    with pytest.raises(Exception, match="LANE of at most 4"):
        feed.cmd_ACE_GRIP(FakeGcmd({"LANE": "5", "MM": "40"}))
    assert mcu.commands == []


def test_ace_grip_is_registered():
    assert "ACE_GRIP" in ace2k_feed.GCODES
    feed, printer, mcu = make_grip()
    assert printer.objects["gcode"].commands["ACE_GRIP"] == feed.cmd_ACE_GRIP
