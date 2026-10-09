"""ace2k_ff.py: the feed-forward on the host — the lane → extruder map, the planner hook, the
window's rate, the lane it goes to, the switch and the notices — against the fake printer of
test_ace2k_extras, with a fake extruder, fake planned moves and a fake print-time clock."""

import struct

import ace2k
import ace2k_feed
import ace2k_ff
import pytest
from test_ace2k_extras import (
    DEFAULT_CONSTANTS,
    RESPONSES,
    FakeConfig,
    FakeGcmd,
    FakeMcu,
    FakePrinter,
    fire_ready,
)

FF_CONSTANTS = {
    "ACE2K_FEED_FOLLOW_FLIP_MS": 200,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MIN": 50,
    "ACE2K_FEED_FOLLOW_FLIP_MS_MAX": 2000,
    "ACE2K_FEED_FOLLOW_TAKE_UM": 15000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MIN": 5000,
    "ACE2K_FEED_FOLLOW_TAKE_UM_MAX": 30000,
    "ACE2K_FEED_FOLLOW_TAIL_UM": 2000000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MIN": 200000,
    "ACE2K_FEED_FOLLOW_TAIL_UM_MAX": 2000000,
    "ACE2K_FEED_FF_CHUNK_UM": 3000,
    "ACE2K_FEED_FF_CHUNK_UM_MIN": 1000,
    "ACE2K_FEED_FF_CHUNK_UM_MAX": 10000,
    "ACE2K_FEED_FF_PULSE_UM_S": 15000,
    "ACE2K_FEED_FF_PULSE_UM_S_MIN": 9000,
    "ACE2K_FEED_FF_PULSE_UM_S_MAX": 70000,
}
WITH_FF = dict(DEFAULT_CONSTANTS, **FF_CONSTANTS)
BASE_FMT = "ace2k_feed_base lane=%c rate_um_s=%u clear=%c"
FOLLOWING = 8


class OptionConfig(FakeConfig):
    """FakeConfig whose get() also reads the options (the extruder names are strings)."""

    def get(self, key, default=None):
        if key in self.options:
            return self.options[key]
        return super().get(key, default)


class MoveExtruder:
    """The U1's Klipper: toolhead calls extruder.move(print_time, move)."""

    def __init__(self):
        self.calls = []

    def move(self, print_time, move):
        self.calls.append((print_time, move))


class ProcessExtruder:
    """Newer Klipper: process_move(print_time, move, ea_index)."""

    def __init__(self):
        self.calls = []

    def process_move(self, print_time, move, ea_index):
        self.calls.append((print_time, move, ea_index))


class NoMethodExtruder:
    pass


class FakeMove:
    def __init__(self, e, accel_t=0.0, cruise_t=0.1, decel_t=0.0):
        self.accel_t = accel_t
        self.cruise_t = cruise_t
        self.decel_t = decel_t
        self.axes_d = [1.0, 0.0, 0.0, e]


class FakeClock:
    """The toolhead's MCU: print time = eventtime + offset."""

    def __init__(self, offset=0.0):
        self.offset = offset

    def estimated_print_time(self, eventtime):
        return eventtime + self.offset


def make_ff(options=None, extruders=None, constants=WITH_FF, ready=True):
    mcu = FakeMcu(dict(RESPONSES), constants=constants)
    printer = FakePrinter(mcu)
    printer.add_object("mcu", FakeClock())
    printer.shutdown = False
    printer.is_shutdown = lambda: printer.shutdown  # Klipper's printer.is_shutdown()
    if extruders is None:
        extruders = {"extruder": MoveExtruder()}
    for name, obj in extruders.items():
        printer.add_object(name, obj)
    module = ace2k.load_config(OptionConfig(printer, options=dict(options or {})))
    feed = module.feed
    feed._seed([0] * ace2k_feed.LANES)
    mcu.config_callback()
    if ready:
        fire_ready(printer)
    return feed, printer, mcu


def set_modes(mcu, modes):
    mcu.subscriptions["ace2k_feed_state"](
        {
            "mode": bytes(modes),
            "error": bytes(4),
            "speed_um_s": bytes(16),
            "duty_pct": bytes(4),
            "bursts": bytes(8),
            "seq": bytes(4),
        }
    )


def ff_frame(mcu, doses=(0, 0, 0, 0), taut=(0, 0, 0, 0), full=(0, 0, 0, 0), epoch=(0, 0, 0, 0)):
    """The counters since each lane's last entry, and the entry's epoch."""
    mcu.subscriptions["ace2k_feed_ff_state"](
        {
            "doses": struct.pack("<4H", *doses),
            "taut": struct.pack("<4H", *taut),
            "full": struct.pack("<4H", *full),
            "epoch": bytes(epoch),
        }
    )


def extrude(extruder, start_pt, seconds, rate, step=0.1):
    """Steady extrusion at rate mm/s from start_pt for seconds, in moves of step s."""
    n = round(seconds / step)
    for i in range(n):
        extruder.move(start_pt + i * step, FakeMove(rate * step, cruise_t=step))


def bases(mcu):
    return [data for fmt, data in mcu.commands if fmt == BASE_FMT]


def output(printer):
    return printer.objects["gcode"].output


def make_metering(options=None, lanes=(0,)):
    """make_ff with each of lanes mapped to an extruder of its own, in follow and sent 3 mm/s:
    the unit metering, the only time its corrections count toward a notice."""
    names = {lane: "extruder" if lane == 0 else f"extruder{lane}" for lane in lanes}
    extruders = {name: MoveExtruder() for name in names.values()}
    opts = {f"lane{lane + 1}_extruder": name for lane, name in names.items()}
    opts.update(options or {})
    feed, printer, mcu = make_ff(opts, extruders=extruders)
    for extruder in extruders.values():
        extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING if lane in lanes else 0 for lane in range(4)])
    feed.ff._tick(10.2)
    assert all(feed.ff.rates[lane] > 0 for lane in lanes)
    return feed, printer, mcu


# --- the window's rate ---------------------------------------------------------------------


def test_window_rate_of_a_steady_extrusion():
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(20)]  # 3 mm/s over 2 s
    assert ace2k_ff.window_rate(moves, 10.5, 11.0) == pytest.approx(3.0, abs=0.01)


def test_window_rate_pro_rata_of_a_half_covered_window():
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]  # 10.0 .. 10.5
    assert ace2k_ff.window_rate(moves, 10.25, 10.75) == pytest.approx(1.5, abs=0.01)
    # one move straddling the window's start counts only its overlap
    assert ace2k_ff.window_rate([(9.9, 0.2, 0.6)], 10.0, 10.5) == pytest.approx(0.6)


def test_window_rate_a_retraction_lowers_it_never_below_zero():
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]
    with_retract = moves + [(10.2, 0.05, -0.5)]
    assert ace2k_ff.window_rate(with_retract, 10.0, 10.5) == pytest.approx(2.0, abs=0.01)
    assert ace2k_ff.window_rate([(10.0, 0.1, -2.0)], 10.0, 10.5) == 0.0
    assert ace2k_ff.window_rate([], 10.0, 10.5) == 0.0
    assert ace2k_ff.window_rate([(10.0, 0.0, 5.0)], 10.0, 10.5) == 0.0  # no duration


def test_window_total_is_signed_and_pro_rata():
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]
    assert ace2k_ff.window_total(moves, 10.0, 10.5) == pytest.approx(1.5)
    assert ace2k_ff.window_total([(10.0, 0.1, -2.0)], 10.0, 10.5) == pytest.approx(-2.0)
    assert ace2k_ff.window_total([(9.95, 0.1, -0.8)], 10.0, 10.5) == pytest.approx(-0.4)
    assert ace2k_ff.window_total([], 10.0, 10.5) == 0.0
    assert ace2k_ff.window_total([(10.0, 0.0, 5.0)], 10.0, 10.5) == 0.0  # no duration


def test_choose_lane():
    modes = ["following", "idle", "following", None]
    assert ace2k_ff.choose_lane([1, 3], modes) == (None, [])
    assert ace2k_ff.choose_lane([0, 1], modes) == (0, [0])
    assert ace2k_ff.choose_lane([0, 2], modes) == (None, [0, 2])


# --- the map -------------------------------------------------------------------------------


def test_the_keys_build_the_map_and_several_lanes_may_name_one_extruder():
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder", "lane3_extruder": "extruder"})
    assert [feed.lane_status(i)["ff_extruder"] for i in range(4)] == [
        "extruder",
        None,
        "extruder",
        None,
    ]


def test_set_lane_extruder_sets_a_lane_without_a_key_and_a_key_wins():
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders=extruders)
    feed.set_lane_extruder(1, "extruder1")
    feed.set_lane_extruder(0, "extruder1")  # lane 1 has a key: the key wins
    assert feed.lane_status(0)["ff_extruder"] == "extruder"
    assert feed.lane_status(1)["ff_extruder"] == "extruder1"
    # after ready the extruder is hooked at once
    assert "move" in vars(extruders["extruder1"])
    feed.set_lane_extruder(1, None)
    assert feed.lane_status(1)["ff_extruder"] is None
    for bad in (4, -1):
        with pytest.raises(ValueError):
            feed.set_lane_extruder(bad, "extruder")


def test_an_unknown_extruder_at_ready_is_a_console_line_and_that_lane_has_no_feed_forward():
    feed, printer, mcu = make_ff({"lane2_extruder": "extruder9"})
    assert any("lane 2" in line and "extruder9" in line for line in output(printer))
    set_modes(mcu, [0, FOLLOWING, 0, 0])
    feed.ff._tick(10.0)
    assert bases(mcu) == []


# --- the planner hook ----------------------------------------------------------------------


def test_the_wrapper_calls_the_original_first_and_records_only_extruding_moves():
    class SpyExtruder:
        """Records, at each call, how many moves the feed-forward held: the original runs
        before the record."""

        def __init__(self):
            self.ff = None
            self.seen = []

        def move(self, print_time, move):
            self.seen.append(len(self.ff.moves["extruder"]))

    extruder = SpyExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extruder.ff = feed.ff
    extruder.move(10.0, FakeMove(0.3, accel_t=0.02, cruise_t=0.06, decel_t=0.02))
    extruder.move(10.1, FakeMove(0.0))  # a travel move: not recorded
    extruder.move(10.2, FakeMove(-0.5))  # a retraction: recorded, negative
    assert extruder.seen == [0, 1, 1]
    assert list(feed.ff.moves["extruder"]) == [
        (10.0, pytest.approx(0.1), 0.3),
        (10.2, pytest.approx(0.1), -0.5),
    ]


def test_the_wrapper_on_process_move_and_neither_method_is_a_console_line():
    extruders = {"extruder": ProcessExtruder(), "extruder1": NoMethodExtruder()}
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane2_extruder": "extruder1"}, extruders=extruders
    )
    extruders["extruder"].process_move(10.0, FakeMove(0.3), 0)
    assert extruders["extruder"].calls[0][2] == 0
    assert list(feed.ff.moves["extruder"]) == [(10.0, 0.1, 0.3)]
    assert any("extruder1" in line and "lane 2" in line for line in output(printer))


def test_the_wrapper_survives_a_bad_move_object(caplog):
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extruder.move(10.0, object())  # no axes_d: recording fails, the call does not
    extruder.move(10.0, object())
    assert len(extruder.calls) == 2
    assert caplog.text.count("ace2k: feed-forward") == 1  # logged once, not per move


# --- the lane and the rate -----------------------------------------------------------------


def test_one_lane_in_follow_gets_the_rate_and_the_deadband_holds_small_changes():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 2.0, 3.0)
    feed.ff._tick(10.0)
    assert bases(mcu) == []  # not in follow: nothing
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.5)
    assert bases(mcu) == [[0, 3000, 0]]
    assert feed.lane_status(0)["ff_rate"] == pytest.approx(3.0)
    extrude(extruder, 12.0, 1.0, 3.2)
    feed.ff._tick(11.8)  # half 3.0, half 3.2: +0.1 mm/s
    feed.ff._tick(12.1)  # 3.2: +0.2 mm/s, under the deadband
    assert bases(mcu) == [[0, 3000, 0]]
    feed.ff._tick(13.5)  # nothing planned: 0, sent
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    feed.ff._tick(13.6)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]


def test_a_slow_rate_under_the_deadband_is_sent_from_zero():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 3.0, 0.25)  # a slow print: 0.25 mm/s, under the 0.3 deadband
    feed.ff._tick(10.0)
    assert bases(mcu) == [[0, 250, 0]]
    assert feed.lane_status(0)["ff_rate"] == pytest.approx(0.25)
    feed.ff._tick(10.1)  # the same rate: not sent again
    assert bases(mcu) == [[0, 250, 0]]
    feed.ff._tick(13.5)  # nothing planned: 0
    extruder.move(14.0, FakeMove(0.0001, cruise_t=0.5))  # 0.2 µm/s: rounds to 0, never sent
    feed.ff._tick(13.6)
    feed.ff._tick(13.7)
    assert bases(mcu) == [[0, 250, 0], [0, 0, 0]]


def test_a_rate_above_the_lanes_maximum_is_clamped():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 1.0, 100.0)
    feed.ff._tick(10.0)
    assert bases(mcu) == [[0, 70000, 0]]


def test_the_lead_shifts_the_window():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_lead_ms": 500}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.5, 0.5, 4.0)
    feed.ff._tick(10.0)
    assert bases(mcu) == [[0, 4000, 0]]


def test_lowering_the_lead_live_restarts_the_sweep_and_drops_the_credit():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_lead_ms": 2000}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 6.0, 3.0)
    feed.ff._tick(9.0)  # the window at 11.0: 3 mm/s
    assert bases(mcu) == [[0, 3000, 0]]
    feed.ff.credit["extruder"] = 0.4
    feed.cmd_ACE_FF_SET(FakeGcmd({"LEAD_MS": "0"}))
    assert feed.ff.lead_ms == 0
    assert "extruder" not in feed.ff.swept
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    # the window at 9.6, a fifth of it planned: 0.6 mm/s, sent at once — not held until print
    # time passes the sweep of the old lead
    feed.ff._tick(9.6)
    assert bases(mcu) == [[0, 3000, 0], [0, 600, 0]]


def test_a_new_window_alone_keeps_the_sweep_and_the_credit():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 3.0, 3.0)
    feed.ff._tick(10.0)
    feed.ff.credit["extruder"] = 0.4
    swept = feed.ff.swept["extruder"]
    feed.cmd_ACE_FF_SET(FakeGcmd({"WINDOW_MS": "800"}))
    assert feed.ff.swept["extruder"] == swept
    assert feed.ff.credit["extruder"] == 0.4


def test_two_lanes_of_one_extruder_in_follow_send_nothing_and_say_so_once():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane3_extruder": "extruder"},
        extruders={"extruder": extruder},
    )
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, FOLLOWING, 0])
    before = len(output(printer))
    feed.ff._tick(10.2)
    feed.ff._tick(10.3)
    # no rate; each lane in follow cleared once, at the episode's start: what either owes is
    # the extruder's, and which of them feeds it is not known
    assert bases(mcu) == [[0, 0, 1], [2, 0, 1]]
    lines = output(printer)[before:]
    assert lines == ["ace2k: extruder extruder: lanes 1 and 3 both in follow — no feed-forward"]
    # one leaves the follow: the other gets the rate; a new episode says so again
    set_modes(mcu, [0, 0, FOLLOWING, 0])
    feed.ff._tick(10.4)
    assert bases(mcu)[2:] == [[2, 3000, 0]]
    set_modes(mcu, [FOLLOWING, 0, FOLLOWING, 0])
    feed.ff._tick(10.5)
    feed.ff._tick(10.6)
    # the lane that had a rate is brought to 0 with a clear, the other cleared too
    assert bases(mcu)[3:] == [[0, 0, 1], [2, 0, 1]]
    assert len(output(printer)) == before + 2


def test_a_gap_sends_zero_without_a_clear_and_a_remap_clears():
    # a gap in the planned extrusion is 0, clear 0: the filament owed is the head's, still paid
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = make_ff(extruders=extruders)
    feed.set_lane_extruder(0, "extruder")
    extrude(extruders["extruder"], 10.0, 1.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    feed.ff._tick(11.5)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    # remapped in follow, the rate at 0: what the lane owes was the old extruder's — cleared
    feed.set_lane_extruder(0, "extruder1")
    assert bases(mcu)[2:] == [[0, 0, 1]]
    # remapped with a rate in force: brought to 0 with a clear
    extrude(extruders["extruder1"], 12.0, 1.0, 2.0)
    feed.ff._tick(12.0)
    assert bases(mcu)[3:] == [[0, 2000, 0]]
    feed.set_lane_extruder(0, "extruder")
    assert bases(mcu)[4:] == [[0, 0, 1]]
    # the same extruder again, or a lane not in follow: nothing
    feed.set_lane_extruder(0, "extruder")
    set_modes(mcu, [0, 0, 0, 0])
    feed.set_lane_extruder(0, "extruder1")
    assert len(bases(mcu)) == 5


def test_a_lane_leaving_the_follow_starts_again_from_zero():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    set_modes(mcu, [0, 0, 0, 0])  # disarmed: the unit dropped the rate
    feed.ff._tick(10.3)
    assert feed.lane_status(0)["ff_rate"] == 0.0
    set_modes(mcu, [FOLLOWING, 0, 0, 0])  # armed again: the rate goes out again
    feed.ff._tick(10.4)
    assert bases(mcu) == [[0, 3000, 0], [0, 3000, 0]]


def test_a_new_arm_between_two_state_reports_sends_the_rate_again():
    # the unit arms the follow at 0; the host sees the lane "following" throughout
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    feed.ff._tick(10.3)
    assert bases(mcu) == [[0, 3000, 0]]
    feed.seq[0] += 1  # a start went out on the lane: disarm and arm again
    feed.ff._tick(10.4)
    assert bases(mcu) == [[0, 3000, 0], [0, 3000, 0]]


def test_the_tick_reads_print_time_from_the_toolheads_mcu_and_drops_old_moves():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    printer.objects["mcu"].offset = 5.0
    extruder.move(1.0, FakeMove(1.0))  # long past (moves are recorded in planning order)
    extrude(extruder, 15.0, 0.5, 2.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    waketime = feed.ff._tick(10.0)  # print time 15.0
    assert bases(mcu) == [[0, 2000, 0]]
    assert waketime == pytest.approx(10.1)
    assert all(pt + dur >= 15.0 for pt, dur, _ in feed.ff.moves["extruder"])


# --- the retraction credit ----------------------------------------------------------------


def print_cycle(extruder, start_pt, cycles, extrude_s=2.0, rate=3.0, retract=0.8, travel_s=1.0):
    """cycles of: extrude for extrude_s at rate, retract, travel (no E) for travel_s,
    unretract.  Returns (the end's print time, the net E, the moves)."""
    moves = []
    pt = start_pt
    for _ in range(cycles):
        for i in range(round(extrude_s / 0.1)):
            moves.append((pt + i * 0.1, 0.1, rate * 0.1))
        pt += extrude_s
        moves.append((pt, 0.05, -retract))
        pt += 0.05 + travel_s
        moves.append((pt, 0.05, retract))
        pt += 0.05
    for pt_, dur, e in moves:
        extruder.move(pt_, FakeMove(e, cruise_t=dur))
    return pt, sum(e for _, _, e in moves), moves


def run_ticks(feed, first, last, period=0.1):
    """Tick every period from first to last; the mm the rates in force deliver, per tick."""
    delivered = []
    for k in range(round((last - first) / period) + 1):
        feed.ff._tick(first + k * period)
        delivered.append(feed.ff.rates[0] * period)
    return delivered


def test_retract_unretract_pairs_deliver_the_net_e():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    # 1 mm/s for 4 s, then a 0.8 mm retraction, 1 s of travel, the unretraction: ten times
    end, net, moves = print_cycle(extruder, 10.0, 10, extrude_s=4.0, rate=1.0)
    delivered = sum(run_ticks(feed, 9.5, end + 1.0))
    assert net == pytest.approx(40.0)
    assert delivered == pytest.approx(net, rel=0.01)
    # the floored rate alone over-feeds here by 0.6 mm a pair (measured on the unit)
    floored = sum(
        ace2k_ff.window_rate(moves, 9.5 + k * 0.1, 10.0 + k * 0.1) * 0.1
        for k in range(round((end + 1.0 - 9.5) / 0.1) + 1)
    )
    assert floored - net == pytest.approx(10 * 0.6, abs=0.05)


def test_a_lone_retraction_withholds_the_first_extrusion():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extruder.move(10.0, FakeMove(-0.8, cruise_t=0.05))
    extrude(extruder, 11.0, 2.0, 3.0)
    delivered = []
    for k in range(round((13.5 - 9.5) / 0.1) + 1):
        feed.ff._tick(9.5 + k * 0.1)
        if feed.ff.credit.get("extruder", 0.0) > 1e-9:
            assert feed.ff.rates[0] == 0.0  # owed back: nothing metered yet
        delivered.append(feed.ff.rates[0] * 0.1)
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-9)
    assert sum(delivered) == pytest.approx(6.0 - 0.8, abs=0.02)


def test_the_credit_is_capped():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extruder.move(10.0, FakeMove(-100.0, cruise_t=0.05))
    run_ticks(feed, 9.5, 10.6)
    assert feed.ff.credit["extruder"] == pytest.approx(ace2k_ff.CREDIT_MAX_MM)
    assert ace2k_ff.CREDIT_MAX_MM == 50.0
    extrude(extruder, 11.5, 20.0, 3.0)  # 60 mm, past every window ticked so far
    delivered = sum(run_ticks(feed, 10.7, 31.5))
    assert delivered == pytest.approx(60.0 - 50.0, abs=0.05)


def credited(options=None, extruders=None, lanes=(0,)):
    """make_ff, lanes keyed to extruder (none: lane 1 mapped at run time), lane 1 in follow, a
    0.8 mm retraction credited."""
    extruders = extruders or {"extruder": MoveExtruder()}
    opts = {f"lane{lane + 1}_extruder": "extruder" for lane in lanes}
    opts.update(options or {})
    feed, printer, mcu = make_ff(opts, extruders=extruders)
    if not lanes:
        feed.set_lane_extruder(0, "extruder")
    set_modes(mcu, [FOLLOWING if lane == 0 else 0 for lane in range(4)])
    extruders["extruder"].move(10.0, FakeMove(-0.8, cruise_t=0.05))
    run_ticks(feed, 9.5, 10.0)
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)
    return feed, printer, mcu


def test_the_credit_resets_on_leaving_the_follow():
    feed, printer, mcu = credited()
    set_modes(mcu, [0, 0, 0, 0])
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_when_the_feed_forward_is_turned_off():
    feed, printer, mcu = credited()
    printer.objects["gcode"].commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_on_a_remap():
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = credited(extruders=extruders, lanes=())
    feed.set_lane_extruder(0, "extruder")  # the same extruder: kept
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.8)
    feed.ff.credit["extruder1"] = 0.5
    feed.set_lane_extruder(0, "extruder1")
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    assert feed.ff.credit.get("extruder1", 0.0) == 0.0


def test_the_credit_resets_when_two_lanes_follow():
    feed, printer, mcu = credited(lanes=(0, 2))
    set_modes(mcu, [FOLLOWING, 0, FOLLOWING, 0])
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_after_a_shutdown():
    feed, printer, mcu = credited()
    printer.shutdown = True
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_is_kept_when_another_lane_of_its_extruder_is_switched_off():
    feed, printer, mcu = credited(lanes=(0, 2))
    gcode = printer.objects["gcode"]
    gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "3", "OFF": "1"}))  # idle: kept
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)
    gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))  # the follower
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_is_kept_when_an_idle_lane_is_remapped():
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = credited(extruders=extruders, lanes=())
    feed.set_lane_extruder(2, "extruder")  # an idle lane into the extruder: kept
    feed.set_lane_extruder(2, "extruder1")  # and out of it
    feed.set_lane_extruder(2, None)
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)
    # an extruder with no lane in follow keeps no credit
    feed.ff.credit["extruder1"] = 0.5
    feed.set_lane_extruder(2, "extruder1")
    assert feed.ff.credit.get("extruder1", 0.0) == 0.0
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)


def run_irregular(feed, times):
    """Tick at times; the mm the rates in force deliver, each until the next tick."""
    delivered = 0.0
    for now, after in zip(times, times[1:] + [times[-1]]):
        feed.ff._tick(now)
        delivered += feed.ff.rates[0] * (after - now)
    return delivered


def test_a_late_tick_keeps_the_net_e():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 6.0, 3.0)
    times = [round(9.5 + k * 0.1, 6) for k in range(round((17.0 - 9.5) / 0.1) + 1)]
    # the reactor late three times, by 3x the period
    for late in (11.0, 13.0, 15.0):
        times = [t for t in times if not late < t < late + 0.3]
    assert run_irregular(feed, times) == pytest.approx(18.0, rel=0.01)
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-6)


def test_a_gap_past_the_window_leaves_no_stale_credit():
    # retraction counted before the gap, its unretraction inside it
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extruder.move(10.0, FakeMove(-0.8, cruise_t=0.05))
    extruder.move(10.6, FakeMove(0.8, cruise_t=0.05))
    extrude(extruder, 12.0, 2.0, 3.0)
    run_ticks(feed, 9.5, 10.0)
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)
    feed.ff._tick(11.2)  # the reactor blocked past the unretraction
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-6)


def test_a_late_tick_never_credits_more_than_the_retraction():
    # the late tick's window holds the retraction, and so do the next regular ones
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extruder.move(10.05, FakeMove(-0.8, cruise_t=0.05))
    feed.ff._tick(9.0)
    peak = 0.0
    for now in (9.6, 9.7, 9.8, 9.9, 10.0, 10.1, 10.2):
        feed.ff._tick(now)
        peak = max(peak, feed.ff.credit.get("extruder", 0.0))
    assert peak <= 0.8 + 1e-6
    assert feed.ff.credit["extruder"] == pytest.approx(0.8, abs=1e-6)


# --- the switch ----------------------------------------------------------------------------


def test_feed_forward_off_sends_zero_once_and_on_resumes():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane2_extruder": "extruder"},
        extruders={"extruder": extruder},
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    gcode = printer.objects["gcode"]
    gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    assert feed.lane_status(0)["ff_on"] is False and feed.lane_status(1)["ff_on"] is True
    feed.ff._tick(10.3)
    feed.ff._tick(10.4)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"ON": "1"}))
    feed.ff._tick(10.5)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0], [0, 3000, 0]]
    gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"OFF": "1"}))  # every lane
    assert bases(mcu)[-1] == [0, 0, 0]
    assert all(not feed.lane_status(i)["ff_on"] for i in range(4))


def test_feed_forward_false_in_the_config_sends_nothing():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "feed_forward": False}, extruders={"extruder": extruder}
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    assert bases(mcu) == []
    assert feed.lane_status(0)["ff_on"] is False


# --- the hold (an automatic encoder calibration) -------------------------------------------


def test_a_hold_clears_the_lane_once_sends_no_rate_and_a_release_resumes_at_the_next_tick():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    assert bases(mcu) == [[0, 3000, 0]]
    feed.ff.hold(0, True)
    # what the lane owes is dropped with the rate: a dose would move the filament between marks
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 1]]
    for now in (10.3, 10.4, 10.5, 11.0):
        feed.ff._tick(now)
        assert not feed.ff._metering(0)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 1]]
    assert feed.lane_status(0)["ff_on"] is True and feed.lane_status(0)["ff_rate"] == 0.0
    feed.ff.hold(0, False)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 1]]  # the release sends nothing itself
    feed.ff._tick(11.1)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 1], [0, 3000, 0]]
    assert feed.ff._metering(0) and feed.lane_status(0)["ff_on"] is True


def test_a_hold_drops_the_credit_of_the_extruders_follower_or_of_an_extruder_without_one():
    feed, printer, mcu = credited(lanes=(0, 2))
    sent = len(bases(mcu))
    feed.ff.hold(2, True)  # idle beside the follower: nothing owed, the follower's credit kept
    assert feed.ff.credit["extruder"] == pytest.approx(0.8) and len(bases(mcu)) == sent
    feed.ff.hold(0, True)  # the follower, at 0 but in the follow: cleared, the credit dropped
    assert feed.ff.credit.get("extruder", 0.0) == 0.0 and "extruder" not in feed.ff.swept
    assert bases(mcu)[sent:] == [[0, 0, 1]]
    assert feed.ff.held == {0, 2}
    feed, printer, mcu = credited(lanes=(0, 2))
    set_modes(mcu, [0, 0, 0, 0])  # no lane of the extruder left in the follow
    feed.ff.hold(2, True)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_a_hold_after_a_shutdown_sends_nothing_and_the_lane_does_not_meter():
    feed, printer, mcu = make_metering()
    printer.shutdown = True  # the modes are stale: nothing may be sent, not even a clear
    feed.ff.hold(0, True)
    assert bases(mcu) == [[0, 3000, 0]] and feed.ff.rates[0] == pytest.approx(3.0)
    assert not feed.ff._metering(0)  # the rate recorded, the lane held all the same


def test_taut_corrections_while_held_say_nothing():
    feed, printer, mcu = make_metering()
    ff_frame(mcu)  # the baseline
    feed.ff.hold(0, True)
    ff_frame(mcu, taut=(3, 0, 0, 0))  # metered until the hold: credited
    assert feed.ff.net[0] == 3
    before = len(output(printer))
    for n in range(4, 60):
        ff_frame(mcu, taut=(n, 0, 0, 0))  # the plain follow under the hold: not
    assert feed.ff.net[0] == 3 and output(printer)[before:] == []


# --- the notices ---------------------------------------------------------------------------


def test_twenty_taut_corrections_are_one_notice():
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    for n in range(1, 21):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    lines = output(printer)[before:]
    assert lines == [
        "ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale"
        " (ACE_CALIBRATE_ENCODER)"
    ]
    assert feed.lane_status(0)["ff_taut_fixes"] == 20


def test_corrections_both_ways_cancel():
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(10, 0, 0, 0))
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(29, 0, 0, 0))
    assert output(printer)[before:] == []
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(30, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 20 full corrections since the last notice — check the encoder scale"
        " (ACE_CALIBRATE_ENCODER)"
    ]


def test_the_notice_names_the_net_count_when_a_frame_jumps_past_n():
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    ff_frame(mcu, taut=(19, 0, 0, 0))
    ff_frame(mcu, taut=(23, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 23 taut corrections since the last notice — check the encoder scale"
        " (ACE_CALIBRATE_ENCODER)"
    ]


def test_a_new_epoch_counts_from_zero():
    # re-armed and climbed past the old counts before the next frame: 15 → 18 is 18 new
    feed, printer, mcu = make_metering({"ff_notice_every": 30})
    before = len(output(printer))
    ff_frame(mcu, epoch=(3, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    ff_frame(mcu, taut=(18, 0, 0, 0), epoch=(5, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 33 taut corrections since the last notice — check the encoder scale"
        " (ACE_CALIBRATE_ENCODER)"
    ]


def test_a_start_landing_between_frames_is_counted_once():
    # the last arm left (0, 0); the start lands after a state report and before the counters:
    # its first counts come with its epoch, and the state report after them changes nothing
    feed, printer, mcu = make_metering()
    ff_frame(mcu, epoch=(4, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(2, 0, 0, 0), epoch=(5, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(3, 0, 0, 0), epoch=(5, 0, 0, 0))
    assert feed.ff.net[0] == 3


def test_a_self_started_load_then_a_re_arm_reset_once_each():
    feed, printer, mcu = make_metering()
    ff_frame(mcu, epoch=(3, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    ff_frame(mcu, epoch=(5, 0, 0, 0))  # the unit's own load: its counters at 0
    ff_frame(mcu, taut=(1, 0, 0, 0), epoch=(6, 0, 0, 0))  # armed again
    assert feed.ff.net[0] == 16
    # both between two frames: one new count all the same
    ff_frame(mcu, taut=(2, 0, 0, 0), epoch=(8, 0, 0, 0))
    assert feed.ff.net[0] == 18


def test_a_refused_start_adds_nothing_again():
    # busy: the unit never entered, its counters keep their totals — 15 is not 15 + 15
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu, epoch=(4, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    mcu.responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(ace2k_feed.FeedRefused, match="busy"):
        feed.start_move(0, "assist_both", 0.0, 10.0)
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    ff_frame(mcu, taut=(16, 0, 0, 0), epoch=(4, 0, 0, 0))
    assert feed.ff.net[0] == 16 and output(printer)[before:] == []


def test_the_sessions_first_counters_are_its_baseline():
    # Klipper restarted, the unit did not: what it counted before is not this session's
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(7, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(7, 0, 0, 0))
    assert feed.ff.net[0] == 0
    ff_frame(mcu, taut=(16, 0, 0, 0), epoch=(7, 0, 0, 0))
    assert feed.ff.net[0] == 1 and output(printer)[before:] == []


def test_totals_past_n_at_a_restart_say_nothing():
    feed, printer, mcu = make_metering(lanes=(0, 2))
    before = len(output(printer))
    ff_frame(mcu, taut=(25, 0, 0, 0), full=(0, 0, 40, 0), epoch=(7, 0, 3, 0))
    assert feed.ff.net == [0, 0, 0, 0] and output(printer)[before:] == []
    ff_frame(mcu, taut=(26, 0, 0, 0), full=(0, 0, 41, 0), epoch=(7, 0, 3, 0))
    assert feed.ff.net == [1, 0, -1, 0] and output(printer)[before:] == []


def test_a_counter_wrap_is_counted_across_it():
    feed, printer, mcu = make_metering({"ff_notice_every": 10})
    ff_frame(mcu, taut=(65530, 0, 0, 0))  # the baseline
    before = len(output(printer))
    ff_frame(mcu, taut=(3, 0, 0, 0))  # 65530 → 3 is 9 across the wrap, not a new arm
    assert feed.ff.net[0] == 9 and len(output(printer)) == before
    ff_frame(mcu, taut=(4, 0, 0, 0))
    assert len(output(printer)) == before + 1


def test_taut_corrections_with_the_lane_off_say_nothing():
    # the plain follow corrects on every taut: not the metering's signature
    feed, printer, mcu = make_metering()
    printer.objects["gcode"].commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    before = len(output(printer))
    ff_frame(mcu)  # the baseline
    for n in range(1, 41):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    assert output(printer)[before:] == [] and feed.ff.net[0] == 0
    assert feed.lane_status(0)["ff_taut_fixes"] == 40


def test_taut_corrections_with_feed_forward_false_say_nothing():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "feed_forward": False}, extruders={"extruder": extruder}
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    assert bases(mcu) == []
    before = len(output(printer))
    ff_frame(mcu)  # the baseline
    for n in range(1, 41):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    assert output(printer)[before:] == [] and feed.ff.net[0] == 0


def test_taut_corrections_at_rate_zero_say_nothing():
    # on, mapped, in follow, nothing planned: the unit is not metering
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    assert feed.lane_status(0)["ff_on"] is True and feed.ff.rates[0] == 0.0
    before = len(output(printer))
    ff_frame(mcu)  # the baseline
    for n in range(1, 41):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    assert output(printer)[before:] == [] and feed.ff.net[0] == 0
    assert feed.lane_status(0)["ff_taut_fixes"] == 40


def test_switching_on_mid_epoch_credits_only_what_comes_after():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "feed_forward": False}, extruders={"extruder": extruder}
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    before = len(output(printer))
    ff_frame(mcu, epoch=(4, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))  # off: not credited
    printer.objects["gcode"].commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "ON": "1"}))
    feed.ff._tick(10.2)
    assert bases(mcu) == [[0, 3000, 0]]
    ff_frame(mcu, taut=(17, 0, 0, 0), epoch=(4, 0, 0, 0))  # its interval was the plain follow's
    assert feed.ff.net[0] == 0
    for n in range(18, 37):
        ff_frame(mcu, taut=(n, 0, 0, 0), epoch=(4, 0, 0, 0))
    assert feed.ff.net[0] == 19
    ff_frame(mcu, taut=(37, 0, 0, 0), epoch=(4, 0, 0, 0))
    lines = [line for line in output(printer)[before:] if "corrections" in line]
    assert lines == [
        "ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale"
        " (ACE_CALIBRATE_ENCODER)"
    ]


def test_a_rate_falling_to_zero_just_before_the_frame_still_credits_its_interval():
    feed, printer, mcu = make_metering()
    ff_frame(mcu)  # the baseline
    ff_frame(mcu, taut=(3, 0, 0, 0))
    feed.ff._tick(13.5)  # nothing planned past the moves: 0 sent
    assert bases(mcu)[-1] == [0, 0, 0]
    ff_frame(mcu, taut=(5, 0, 0, 0))  # metered until just now: credited
    assert feed.ff.net[0] == 5
    ff_frame(mcu, taut=(9, 0, 0, 0))  # a whole interval at 0: not
    assert feed.ff.net[0] == 5


def test_a_rate_arriving_just_before_the_frame_does_not_credit_its_interval():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.0)  # nothing planned: the plain follow
    ff_frame(mcu)  # the baseline
    ff_frame(mcu, taut=(3, 0, 0, 0))
    extrude(extruder, 10.5, 3.0, 3.0)
    feed.ff._tick(10.6)
    assert bases(mcu) == [[0, 3000, 0]]
    ff_frame(mcu, taut=(8, 0, 0, 0))  # the interval was at 0 but for its last moment
    assert feed.ff.net[0] == 0
    feed.ff._tick(10.7)
    ff_frame(mcu, taut=(10, 0, 0, 0))
    assert feed.ff.net[0] == 2


def test_a_lane_switched_off_credits_its_last_interval_then_nothing():
    feed, printer, mcu = make_metering()
    ff_frame(mcu)  # the baseline
    ff_frame(mcu, taut=(4, 0, 0, 0))
    printer.objects["gcode"].commands["ACE_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    feed.ff._tick(10.3)
    ff_frame(mcu, taut=(6, 0, 0, 0))  # metered until the switch: credited
    assert feed.ff.net[0] == 6
    feed.ff._tick(10.4)
    for n in range(7, 50):
        ff_frame(mcu, taut=(n, 0, 0, 0))  # off for the whole interval: not
    assert feed.ff.net[0] == 6


def test_after_a_shutdown_the_tick_stops_and_sends_nothing():
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    printer.shutdown = True
    assert feed.ff._tick(10.2) == printer.reactor.NEVER
    assert bases(mcu) == []


def test_after_a_shutdown_a_remap_sends_no_clear():
    # the modes are stale after a shutdown: a lane last seen in follow, at 0 or with a rate
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = make_ff(extruders=extruders)
    feed.set_lane_extruder(0, "extruder")
    feed.set_lane_extruder(1, "extruder1")
    extrude(extruders["extruder"], 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, FOLLOWING, 0, 0])
    feed.ff._tick(10.2)
    assert bases(mcu) == [[0, 3000, 0]]
    printer.shutdown = True
    feed.set_lane_extruder(0, "extruder1")  # a rate in force
    feed.set_lane_extruder(1, "extruder")  # in follow, at 0
    assert bases(mcu) == [[0, 3000, 0]]
    assert feed.lane_status(0)["ff_extruder"] == "extruder1"


# --- an image without the feed-forward -----------------------------------------------------


def test_an_image_whose_ff_frame_has_no_epoch_is_off_with_one_line():
    old = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s"
    mcu = FakeMcu(dict(RESPONSES), constants=WITH_FF, formats={"ace2k_feed_ff_state": old})
    printer = FakePrinter(mcu)
    printer.add_object("mcu", FakeClock())
    printer.add_object("extruder", MoveExtruder())
    module = ace2k.load_config(OptionConfig(printer, options={"lane1_extruder": "extruder"}))
    feed = module.feed
    feed._seed([0] * ace2k_feed.LANES)
    mcu.config_callback()
    fire_ready(printer)
    assert not feed.has_ff
    assert "ace2k_feed_ff_state" not in mcu.subscriptions
    assert not any("ff" in c for c, _ in mcu.config_cmds)
    assert [line for line in output(printer) if "v0.9.0" in line] == [
        "ace2k: the unit's firmware has no feed-forward; flash v0.9.0"
    ]


def test_an_image_without_the_feed_forward_is_off_with_one_line_and_never_sent_to():
    extruder = MoveExtruder()
    constants = dict(DEFAULT_CONSTANTS, **{k: v for k, v in FF_CONSTANTS.items() if "FF" not in k})
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder"}, extruders={"extruder": extruder}, constants=constants
    )
    assert [line for line in output(printer) if "v0.9.0" in line] == [
        "ace2k: the unit's firmware has no feed-forward; flash v0.9.0"
    ]
    assert not any("ff" in c for c, _ in mcu.config_cmds)
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    gcode = printer.objects["gcode"]
    with pytest.raises(Exception, match="flash v0.9.0"):
        gcode.commands["ACE_FEED_FORWARD"](FakeGcmd({"OFF": "1"}))
    assert bases(mcu) == [] and not any("ff" in fmt for fmt, _ in mcu.commands)
    assert feed.lane_status(0)["ff_on"] is False
