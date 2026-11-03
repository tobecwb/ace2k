"""ace2k_ff.py: the feed-forward on the host — the lane → extruder map, the planner hook, the
window's rate, the lane it goes to, the switch and the notices — against the fake printer of
test_ace2k_extras, with a fake extruder, fake planned moves and a fake print-time clock."""

from __future__ import annotations

import math
import struct
from typing import TYPE_CHECKING

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

if TYPE_CHECKING:
    from collections.abc import Sequence
    from typing import Any

    from ace2k_feed import Ace2kFeed
    from ace2k_ff import FeedForward

    # a planned extruder move as the tests write it: (print time, duration s, E mm)
    Moves = Sequence[tuple[float, float, float]]


def window_total(moves: Moves, start_pt: float, end_pt: float) -> float:
    """Signed extruder distance (mm) the moves command inside [start_pt, end_pt]."""
    return ace2k_ff.window_parts(moves, (start_pt, end_pt), (start_pt, end_pt), math.inf)[1]


def window_rate(moves: Moves, start_pt: float, end_pt: float) -> float:
    """Mean extruder speed (mm/s) inside [start_pt, end_pt], floored at 0."""
    return max(0.0, window_total(moves, start_pt, end_pt) / (end_pt - start_pt))


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
    "ACE2K_FEED_FF_REVERSE": 1,
}
WITH_FF = dict(DEFAULT_CONSTANTS, **FF_CONSTANTS)
BASE_FMT = "ace2k_feed_base lane=%c rate_um_s=%i clear=%c"
BASE_FMT_FORWARD = "ace2k_feed_base lane=%c rate_um_s=%u clear=%c"
FF_STATE_FORWARD = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s"
WITHOUT_REVERSE = {k: v for k, v in WITH_FF.items() if k != "ACE2K_FEED_FF_REVERSE"}
OLD_FORMATS = {"ace2k_feed_base": BASE_FMT_FORWARD, "ace2k_feed_ff_state": FF_STATE_FORWARD}
FOLLOWING = 8


class OptionConfig(FakeConfig):
    """FakeConfig whose get() also reads the options (the extruder names are strings)."""

    def get(self, key: str, default: object = None) -> object:
        if key in self.options:
            return self.options[key]
        return super().get(key, default)


class MoveExtruder:
    """The U1's Klipper: toolhead calls extruder.move(print_time, move)."""

    def __init__(self) -> None:
        self.calls: list[tuple[float, FakeMove]] = []

    def move(self, print_time: float, move: FakeMove) -> None:
        self.calls.append((print_time, move))


class ProcessExtruder:
    """Newer Klipper: process_move(print_time, move, ea_index)."""

    def __init__(self) -> None:
        self.calls: list[tuple[float, FakeMove, int]] = []

    def process_move(self, print_time: float, move: FakeMove, ea_index: int) -> None:
        self.calls.append((print_time, move, ea_index))


class NoMethodExtruder:
    pass


class FakeMove:
    def __init__(
        self, e: float, accel_t: float = 0.0, cruise_t: float = 0.1, decel_t: float = 0.0
    ) -> None:
        self.accel_t = accel_t
        self.cruise_t = cruise_t
        self.decel_t = decel_t
        self.axes_d = [1.0, 0.0, 0.0, e]


class FakeClock:
    """The toolhead's MCU: print time = eventtime + offset."""

    def __init__(self, offset: float = 0.0) -> None:
        self.offset = offset

    def estimated_print_time(self, eventtime: float) -> float:
        return eventtime + self.offset


def make_ff(
    options: dict[str, Any] | None = None,
    extruders: dict[str, object] | None = None,
    constants: dict[str, int] = WITH_FF,
    ready: bool = True,
    formats: dict[str, str] | None = None,
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu]:
    mcu = FakeMcu(dict(RESPONSES), constants=constants, formats=formats)
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


def set_modes(mcu: FakeMcu, modes: Sequence[int]) -> None:
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


def ff_frame(
    mcu: FakeMcu,
    doses: Sequence[int] = (0, 0, 0, 0),
    taut: Sequence[int] = (0, 0, 0, 0),
    full: Sequence[int] = (0, 0, 0, 0),
    epoch: Sequence[int] = (0, 0, 0, 0),
    back: Sequence[int] | None = None,
) -> None:
    """The counters since each lane's last entry, and the entry's epoch; back with the reverse."""
    frame = {
        "doses": struct.pack("<4H", *doses),
        "taut": struct.pack("<4H", *taut),
        "full": struct.pack("<4H", *full),
        "epoch": bytes(epoch),
    }
    if back is not None:
        frame["back"] = struct.pack("<4H", *back)
    mcu.subscriptions["ace2k_feed_ff_state"](frame)


def extrude(
    extruder: MoveExtruder, start_pt: float, seconds: float, rate: float, step: float = 0.1
) -> None:
    """Steady extrusion at rate mm/s from start_pt for seconds, in moves of step s."""
    n = round(seconds / step)
    for i in range(n):
        extruder.move(start_pt + i * step, FakeMove(rate * step, cruise_t=step))


def bases(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt in (BASE_FMT, BASE_FMT_FORWARD)]


def output(printer: FakePrinter) -> list[str]:
    return printer.objects["gcode"].output


def make_metering(
    options: dict[str, Any] | None = None, lanes: Sequence[int] = (0,)
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu]:
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


def test_window_rate_of_a_steady_extrusion() -> None:
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(20)]  # 3 mm/s over 2 s
    assert window_rate(moves, 10.5, 11.0) == pytest.approx(3.0, abs=0.01)


def test_window_rate_pro_rata_of_a_half_covered_window() -> None:
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]  # 10.0 .. 10.5
    assert window_rate(moves, 10.25, 10.75) == pytest.approx(1.5, abs=0.01)
    # one move straddling the window's start counts only its overlap
    assert window_rate([(9.9, 0.2, 0.6)], 10.0, 10.5) == pytest.approx(0.6)


def test_window_rate_a_retraction_lowers_it_never_below_zero() -> None:
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]
    with_retract = moves + [(10.2, 0.05, -0.5)]
    assert window_rate(with_retract, 10.0, 10.5) == pytest.approx(2.0, abs=0.01)
    assert window_rate([(10.0, 0.1, -2.0)], 10.0, 10.5) == 0.0
    assert window_rate([], 10.0, 10.5) == 0.0
    assert window_rate([(10.0, 0.0, 5.0)], 10.0, 10.5) == 0.0  # no duration


def test_window_total_is_signed_and_pro_rata() -> None:
    moves = [(10.0 + i * 0.1, 0.1, 0.3) for i in range(5)]
    assert window_total(moves, 10.0, 10.5) == pytest.approx(1.5)
    assert window_total([(10.0, 0.1, -2.0)], 10.0, 10.5) == pytest.approx(-2.0)
    assert window_total([(10.0, 0.1, -2.0, None)], 10.0, 10.5) == pytest.approx(-2.0)
    assert window_total([(9.95, 0.1, -0.8)], 10.0, 10.5) == pytest.approx(-0.4)
    assert window_total([], 10.0, 10.5) == 0.0
    assert window_total([(10.0, 0.0, 5.0)], 10.0, 10.5) == 0.0  # no duration


def test_window_parts_separates_the_long_runs() -> None:
    run = ace2k_ff.Retraction()
    run.total = 12.0
    short = ace2k_ff.Retraction()
    short.total = 4.0
    moves = [(10.0, 0.1, 0.5, None), (10.1, 0.1, -12.0, run), (10.2, 0.1, -4.0, short)]
    assert ace2k_ff.window_parts(moves, (10.0, 10.5), (10.0, 10.5), 10.0) == pytest.approx(
        (-12.0, -3.5)
    )
    assert ace2k_ff.window_parts(moves, (10.0, 10.5), (10.0, 10.5), math.inf) == pytest.approx(
        (0.0, -15.5)
    )


def test_choose_lane() -> None:
    modes = ["following", "idle", "following", None]
    assert ace2k_ff.choose_lane([1, 3], modes) == (None, [])
    assert ace2k_ff.choose_lane([0, 1], modes) == (0, [0])
    assert ace2k_ff.choose_lane([0, 2], modes) == (None, [0, 2])


# --- the map -------------------------------------------------------------------------------


def test_the_keys_build_the_map_and_several_lanes_may_name_one_extruder() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder", "lane3_extruder": "extruder"})
    assert [feed.lane_status(i)["ff_extruder"] for i in range(4)] == [
        "extruder",
        None,
        "extruder",
        None,
    ]


def test_set_lane_extruder_sets_a_lane_without_a_key_and_a_key_wins() -> None:
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


def test_an_unknown_extruder_at_ready_is_a_console_line_and_that_lane_has_no_feed_forward() -> None:
    feed, printer, mcu = make_ff({"lane2_extruder": "extruder9"})
    assert any("lane 2" in line and "extruder9" in line for line in output(printer))
    set_modes(mcu, [0, FOLLOWING, 0, 0])
    feed.ff._tick(10.0)
    assert bases(mcu) == []


# --- the planner hook ----------------------------------------------------------------------


def test_the_wrapper_calls_the_original_first_and_records_only_extruding_moves() -> None:
    class SpyExtruder:
        """Records, at each call, how many moves the feed-forward held: the original runs
        before the record."""

        def __init__(self) -> None:
            self.ff: FeedForward | None = None
            self.seen: list[int] = []

        def move(self, print_time: float, move: FakeMove) -> None:
            self.seen.append(len(self.ff.moves["extruder"]))

    extruder = SpyExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extruder.ff = feed.ff
    extruder.move(10.0, FakeMove(0.3, accel_t=0.02, cruise_t=0.06, decel_t=0.02))
    extruder.move(10.1, FakeMove(0.0))  # a travel move: not recorded
    extruder.move(10.2, FakeMove(-0.5))  # a retraction: recorded, negative
    assert extruder.seen == [0, 1, 1]
    recorded = list(feed.ff.moves["extruder"])
    assert [tuple(move[:3]) for move in recorded] == [
        (10.0, pytest.approx(0.1), 0.3),
        (10.2, pytest.approx(0.1), -0.5),
    ]
    # an extrusion belongs to no retraction; a retraction to its run, which totals it
    assert recorded[0][3] is None and recorded[1][3].total == pytest.approx(0.5)


def test_the_wrapper_on_process_move_and_neither_method_is_a_console_line() -> None:
    extruders = {"extruder": ProcessExtruder(), "extruder1": NoMethodExtruder()}
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane2_extruder": "extruder1"}, extruders=extruders
    )
    extruders["extruder"].process_move(10.0, FakeMove(0.3), 0)
    assert extruders["extruder"].calls[0][2] == 0
    # (print time, duration, E, no run, its class not yet frozen)
    assert list(feed.ff.moves["extruder"]) == [[10.0, 0.1, 0.3, None, None]]
    assert any("extruder1" in line and "lane 2" in line for line in output(printer))


def test_the_wrapper_survives_a_bad_move_object(caplog: pytest.LogCaptureFixture) -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extruder.move(10.0, object())  # no axes_d: recording fails, the call does not
    extruder.move(10.0, object())
    assert len(extruder.calls) == 2
    assert caplog.text.count("ace2k: feed-forward") == 1  # logged once, not per move


# --- the lane and the rate -----------------------------------------------------------------


def test_one_lane_in_follow_gets_the_rate_and_the_deadband_holds_small_changes() -> None:
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


def test_a_slow_rate_under_the_deadband_is_sent_from_zero() -> None:
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


def test_a_rate_above_the_lanes_maximum_is_clamped() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 1.0, 100.0)
    feed.ff._tick(10.0)
    assert bases(mcu) == [[0, 70000, 0]]


def test_the_lead_shifts_the_window() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_lead_ms": 500}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.5, 0.5, 4.0)
    feed.ff._tick(10.0)
    assert bases(mcu) == [[0, 4000, 0]]


def test_lowering_the_lead_live_restarts_the_sweep_and_drops_the_credit() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_lead_ms": 2000}, extruders={"extruder": extruder}
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 6.0, 3.0)
    feed.ff._tick(9.0)  # the window at 11.0: 3 mm/s
    assert bases(mcu) == [[0, 3000, 0]]
    feed.ff.credit["extruder"] = 0.4
    feed.cmd_ACEK_FF_SET(FakeGcmd({"LEAD_MS": "0"}))
    assert feed.ff.lead_ms == 0
    assert "extruder" not in feed.ff.swept
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    # the window at 9.6, a fifth of it planned: 0.6 mm/s, sent at once — not held until print
    # time passes the sweep of the old lead
    feed.ff._tick(9.6)
    assert bases(mcu) == [[0, 3000, 0], [0, 600, 0]]


def test_a_new_window_alone_keeps_the_sweep_and_the_credit() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extrude(extruder, 10.0, 3.0, 3.0)
    feed.ff._tick(10.0)
    feed.ff.credit["extruder"] = 0.4
    swept = feed.ff.swept["extruder"]
    feed.cmd_ACEK_FF_SET(FakeGcmd({"WINDOW_MS": "800"}))
    assert feed.ff.swept["extruder"] == swept
    assert feed.ff.credit["extruder"] == 0.4


def test_three_lanes_of_one_extruder_in_follow_are_named_in_one_line() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane2_extruder": "extruder", "lane4_extruder": "extruder"},
        extruders={"extruder": extruder},
    )
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcu, [FOLLOWING, FOLLOWING, 0, FOLLOWING])
    before = len(output(printer))
    feed.ff._tick(10.2)
    assert bases(mcu) == [[0, 0, 1], [1, 0, 1], [3, 0, 1]]
    assert output(printer)[before:] == [
        "ace2k: extruder extruder: lanes 1, 2 and 4 all in follow — no feed-forward"
    ]


def test_two_lanes_of_one_extruder_in_follow_send_nothing_and_say_so_once() -> None:
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


def test_a_gap_sends_zero_without_a_clear_and_a_remap_clears() -> None:
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


def test_a_lane_leaving_the_follow_starts_again_from_zero() -> None:
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


def test_a_new_arm_between_two_state_reports_sends_the_rate_again() -> None:
    # the unit arms the follow at 0; the host sees the lane "following" throughout
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    feed.ff._tick(10.3)
    assert bases(mcu) == [[0, 3000, 0]]
    feed.motion[0].seq += 1  # a start went out on the lane: disarm and arm again
    feed.ff._tick(10.4)
    assert bases(mcu) == [[0, 3000, 0], [0, 3000, 0]]


def test_the_tick_reads_print_time_from_the_toolheads_mcu_and_drops_old_moves() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    printer.objects["mcu"].offset = 5.0
    extruder.move(1.0, FakeMove(1.0))  # long past (moves are recorded in planning order)
    extrude(extruder, 15.0, 0.5, 2.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    waketime = feed.ff._tick(10.0)  # print time 15.0
    assert bases(mcu) == [[0, 2000, 0]]
    assert waketime == pytest.approx(10.1)
    assert all(pt + dur >= 15.0 for pt, dur, *_ in feed.ff.moves["extruder"])


# --- the retraction credit ----------------------------------------------------------------


def print_cycle(
    extruder: MoveExtruder,
    start_pt: float,
    cycles: int,
    extrude_s: float = 2.0,
    rate: float = 3.0,
    retract: float = 0.8,
    travel_s: float = 1.0,
) -> tuple[float, float, list[tuple[float, float, float]]]:
    """cycles of: extrude for extrude_s at rate, retract, travel (no E) for travel_s,
    unretract.  Returns (the end's print time, the net E, the moves)."""
    moves = []
    pt = start_pt
    for _ in range(cycles):
        moves.extend((pt + i * 0.1, 0.1, rate * 0.1) for i in range(round(extrude_s / 0.1)))
        pt += extrude_s
        moves.append((pt, 0.05, -retract))
        pt += 0.05 + travel_s
        moves.append((pt, 0.05, retract))
        pt += 0.05
    for pt_, dur, e in moves:
        extruder.move(pt_, FakeMove(e, cruise_t=dur))
    return pt, sum(e for _, _, e in moves), moves


def run_ticks(feed: Ace2kFeed, first: float, last: float, period: float = 0.1) -> list[float]:
    """Tick every period from first to last; the mm the rates in force deliver, per tick."""
    delivered = []
    for k in range(round((last - first) / period) + 1):
        feed.ff._tick(first + k * period)
        delivered.append(feed.ff.rates[0] * period)
    return delivered


def test_retract_unretract_pairs_deliver_the_net_e() -> None:
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
        window_rate(moves, 9.5 + k * 0.1, 10.0 + k * 0.1) * 0.1
        for k in range(round((end + 1.0 - 9.5) / 0.1) + 1)
    )
    assert floored - net == pytest.approx(10 * 0.6, abs=0.05)


def test_a_lone_retraction_withholds_the_first_extrusion() -> None:
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


def test_the_credit_is_capped() -> None:
    # on an image without the reverse: with it, a 100 mm retraction is taken back, not credited
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "ff_deadband": 0.0},
        extruders={"extruder": extruder},
        constants=WITHOUT_REVERSE,
        formats=OLD_FORMATS,
    )
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    extruder.move(10.0, FakeMove(-100.0, cruise_t=0.05))
    run_ticks(feed, 9.5, 10.6)
    assert feed.ff.credit["extruder"] == pytest.approx(ace2k_ff.CREDIT_MAX_MM)
    assert ace2k_ff.CREDIT_MAX_MM == 50.0
    extrude(extruder, 11.5, 20.0, 3.0)  # 60 mm, past every window ticked so far
    delivered = sum(run_ticks(feed, 10.7, 31.5))
    assert delivered == pytest.approx(60.0 - 50.0, abs=0.05)


def credited(
    options: dict[str, Any] | None = None,
    extruders: dict[str, MoveExtruder] | None = None,
    lanes: Sequence[int] = (0,),
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu]:
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


def test_the_credit_resets_on_leaving_the_follow() -> None:
    feed, printer, mcu = credited()
    set_modes(mcu, [0, 0, 0, 0])
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_when_the_feed_forward_is_turned_off() -> None:
    feed, printer, mcu = credited()
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_on_a_remap() -> None:
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    feed, printer, mcu = credited(extruders=extruders, lanes=())
    feed.set_lane_extruder(0, "extruder")  # the same extruder: kept
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.8)
    feed.ff.credit["extruder1"] = 0.5
    feed.set_lane_extruder(0, "extruder1")
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    assert feed.ff.credit.get("extruder1", 0.0) == 0.0


def test_the_credit_resets_when_two_lanes_follow() -> None:
    feed, printer, mcu = credited(lanes=(0, 2))
    set_modes(mcu, [FOLLOWING, 0, FOLLOWING, 0])
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_resets_after_a_shutdown() -> None:
    feed, printer, mcu = credited()
    printer.shutdown = True
    feed.ff._tick(10.1)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_is_kept_when_another_lane_of_its_extruder_is_switched_off() -> None:
    feed, printer, mcu = credited(lanes=(0, 2))
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "3", "OFF": "1"}))  # idle: kept
    assert feed.ff.credit["extruder"] == pytest.approx(0.8)
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))  # the follower
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_the_credit_is_kept_when_an_idle_lane_is_remapped() -> None:
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


def run_irregular(feed: Ace2kFeed, times: list[float]) -> float:
    """Tick at times; the mm the rates in force deliver, each until the next tick."""
    delivered = 0.0
    for now, after in zip(times, times[1:] + [times[-1]]):
        feed.ff._tick(now)
        delivered += feed.ff.rates[0] * (after - now)
    return delivered


def test_a_late_tick_keeps_the_net_e() -> None:
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


def test_a_gap_past_the_window_leaves_no_stale_credit() -> None:
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


def test_a_late_tick_never_credits_more_than_the_retraction() -> None:
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


def test_feed_forward_off_sends_zero_once_and_on_resumes() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "lane2_extruder": "extruder"},
        extruders={"extruder": extruder},
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.2)
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    assert feed.lane_status(0)["ff_on"] is False and feed.lane_status(1)["ff_on"] is True
    feed.ff._tick(10.3)
    feed.ff._tick(10.4)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0]]
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"ON": "1"}))
    feed.ff._tick(10.5)
    assert bases(mcu) == [[0, 3000, 0], [0, 0, 0], [0, 3000, 0]]
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"OFF": "1"}))  # every lane
    assert bases(mcu)[-1] == [0, 0, 0]
    assert all(not feed.lane_status(i)["ff_on"] for i in range(4))


def switch(printer: FakePrinter, params: dict[str, str]) -> list[str]:
    gcmd = FakeGcmd(params)
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](gcmd)
    return gcmd.lines


def test_on_says_a_lane_with_filament_not_in_the_follow_gets_no_dose_yet() -> None:
    """ON=1 on a lane left out of the follow was taken in silence and the
    extrusion moved nothing; it says why, and how to arm it."""
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    set_modes(mcu, [0, 0, 0, 0])
    feed.unit.lanes[0]["insert"] = True
    lines = switch(printer, {"LANE": "1", "ON": "1"})
    assert lines[0].startswith("ace2k: feed-forward lane 1 on (extruder, idle)")
    assert lines[1:] == [
        "ace2k: lane 1 is not in the follow: its feed-forward doses start once it follows"
        " — ACEK_LANE_ACTIVE LANE=1 arms it"
    ]


def test_on_says_a_lane_in_error_needs_a_clear_first() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    set_modes(mcu, [7, 0, 0, 0])
    feed.unit.lanes[0]["insert"] = True
    assert switch(printer, {"LANE": "1", "ON": "1"})[1].endswith(
        "— ACEK_CLEAR LANE=1, then ACEK_LANE_ACTIVE LANE=1 arms it"
    )


def test_on_on_an_image_without_the_follow_says_so_once() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    set_modes(mcu, [0, 0, 0, 0])
    feed.unit.lanes[0]["insert"] = True
    feed.has_follow = False
    assert switch(printer, {"ON": "1"})[1:] == [
        "ace2k: the feed-forward doses only a lane in the follow: the unit's firmware has no"
        " follow tail (or no follow); flash v0.11.0"
    ]


def test_on_says_nothing_for_a_lane_with_a_command_in_flight() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    set_modes(mcu, [0, 0, 0, 0])
    feed.unit.lanes[0]["insert"] = True
    feed.motion[0].inflight = True  # a load just sent, not reported yet
    assert switch(printer, {"LANE": "1", "ON": "1"})[1:] == []


def test_on_says_nothing_more_for_a_lane_in_the_follow_or_with_no_filament() -> None:
    feed, printer, mcu = make_metering(lanes=(0,))
    feed.unit.lanes[0]["insert"] = True
    feed.unit.lanes[1]["insert"] = False
    lines = switch(printer, {"ON": "1"})
    assert len(lines) == 1
    assert "lane 1 on (extruder, following)" in lines[0]
    assert "lane 2 on (unmapped, idle)" in lines[0]


def test_on_says_nothing_for_a_lane_whose_advice_would_not_hold() -> None:
    """Unmapped (no extruder to meter), moving (on its way), not reported yet (not known), or
    another lane of its extruder following (only one lane of an extruder follows)."""
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    opts = {
        "lane1_extruder": "extruder",
        "lane2_extruder": "extruder",
        "lane3_extruder": "extruder1",
    }
    feed, printer, mcu = make_ff(opts, extruders=extruders)
    for lane in range(4):
        feed.unit.lanes[lane]["insert"] = True
    assert switch(printer, {"ON": "1"})[1:] == []  # no report yet: every mode unknown
    assert "lane 1 on (extruder, unknown)" in switch(printer, {})[0]
    extrude(extruders["extruder"], 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 1, 0])  # 2 behind lane 1, 3 feeding, 4 unmapped
    feed.ff._tick(10.2)
    assert switch(printer, {"ON": "1"})[1:] == []


def test_on_for_every_lane_names_each_lane_that_can_follow() -> None:
    extruders = {"extruder": MoveExtruder(), "extruder1": MoveExtruder()}
    opts = {"lane1_extruder": "extruder", "lane3_extruder": "extruder1"}
    feed, printer, mcu = make_ff(opts, extruders=extruders)
    set_modes(mcu, [0, 0, 0, 0])
    for lane, insert in enumerate((True, True, True, None)):
        feed.unit.lanes[lane]["insert"] = insert
    lines = switch(printer, {"ON": "1"})
    assert [line.split(" is ")[0] for line in lines[1:]] == ["ace2k: lane 1", "ace2k: lane 3"]


def test_off_and_the_state_alone_add_no_notice() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    feed.unit.lanes[0]["insert"] = True
    assert len(switch(printer, {"LANE": "1", "OFF": "1"})) == 1
    assert len(switch(printer, {})) == 1


def test_feed_forward_false_in_the_config_sends_nothing() -> None:
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


def test_a_hold_clears_the_lane_once_sends_no_rate_and_a_release_resumes_at_the_next_tick() -> None:
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


def test_a_hold_drops_the_credit_of_the_extruders_follower_or_of_an_extruder_without_one() -> None:
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


def test_a_hold_after_a_shutdown_sends_nothing_and_the_lane_does_not_meter() -> None:
    feed, printer, mcu = make_metering()
    printer.shutdown = True  # the modes are stale: nothing may be sent, not even a clear
    feed.ff.hold(0, True)
    assert bases(mcu) == [[0, 3000, 0]] and feed.ff.rates[0] == pytest.approx(3.0)
    assert not feed.ff._metering(0)  # the rate recorded, the lane held all the same


def test_taut_corrections_while_held_say_nothing() -> None:
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


def test_twenty_taut_corrections_are_one_notice() -> None:
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    for n in range(1, 21):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    lines = output(printer)[before:]
    assert lines == [
        "ace2k: lane 1: 20 taut corrections since the last notice — check the encoder scale"
        " (ACEK_CALIBRATE_ENCODER)"
    ]
    assert feed.lane_status(0)["ff_taut_fixes"] == 20


def test_corrections_both_ways_cancel() -> None:
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(10, 0, 0, 0))
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(29, 0, 0, 0))
    assert output(printer)[before:] == []
    ff_frame(mcu, taut=(10, 0, 0, 0), full=(30, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 20 full corrections since the last notice — check the encoder scale"
        " (ACEK_CALIBRATE_ENCODER)"
    ]


def test_the_notice_names_the_net_count_when_a_frame_jumps_past_n() -> None:
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu)  # the session's first counters: the baseline
    ff_frame(mcu, taut=(19, 0, 0, 0))
    ff_frame(mcu, taut=(23, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 23 taut corrections since the last notice — check the encoder scale"
        " (ACEK_CALIBRATE_ENCODER)"
    ]


def test_a_new_epoch_counts_from_zero() -> None:
    # re-armed and climbed past the old counts before the next frame: 15 → 18 is 18 new
    feed, printer, mcu = make_metering({"ff_notice_every": 30})
    before = len(output(printer))
    ff_frame(mcu, epoch=(3, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    ff_frame(mcu, taut=(18, 0, 0, 0), epoch=(5, 0, 0, 0))
    assert output(printer)[before:] == [
        "ace2k: lane 1: 33 taut corrections since the last notice — check the encoder scale"
        " (ACEK_CALIBRATE_ENCODER)"
    ]


def test_a_start_landing_between_frames_is_counted_once() -> None:
    # the last arm left (0, 0); the start lands after a state report and before the counters:
    # its first counts come with its epoch, and the state report after them changes nothing
    feed, printer, mcu = make_metering()
    ff_frame(mcu, epoch=(4, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(2, 0, 0, 0), epoch=(5, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(3, 0, 0, 0), epoch=(5, 0, 0, 0))
    assert feed.ff.net[0] == 3


def test_a_self_started_load_then_a_re_arm_reset_once_each() -> None:
    feed, printer, mcu = make_metering()
    ff_frame(mcu, epoch=(3, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))
    ff_frame(mcu, epoch=(5, 0, 0, 0))  # the unit's own load: its counters at 0
    ff_frame(mcu, taut=(1, 0, 0, 0), epoch=(6, 0, 0, 0))  # armed again
    assert feed.ff.net[0] == 16
    # both between two frames: one new count all the same
    ff_frame(mcu, taut=(2, 0, 0, 0), epoch=(8, 0, 0, 0))
    assert feed.ff.net[0] == 18


def test_a_refused_start_adds_nothing_again() -> None:
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


def test_the_sessions_first_counters_are_its_baseline() -> None:
    # Klipper restarted, the unit did not: what it counted before is not this session's
    feed, printer, mcu = make_metering()
    before = len(output(printer))
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(7, 0, 0, 0))
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(7, 0, 0, 0))
    assert feed.ff.net[0] == 0
    ff_frame(mcu, taut=(16, 0, 0, 0), epoch=(7, 0, 0, 0))
    assert feed.ff.net[0] == 1 and output(printer)[before:] == []


def test_totals_past_n_at_a_restart_say_nothing() -> None:
    feed, printer, mcu = make_metering(lanes=(0, 2))
    before = len(output(printer))
    ff_frame(mcu, taut=(25, 0, 0, 0), full=(0, 0, 40, 0), epoch=(7, 0, 3, 0))
    assert feed.ff.net == [0, 0, 0, 0] and output(printer)[before:] == []
    ff_frame(mcu, taut=(26, 0, 0, 0), full=(0, 0, 41, 0), epoch=(7, 0, 3, 0))
    assert feed.ff.net == [1, 0, -1, 0] and output(printer)[before:] == []


def test_a_counter_wrap_is_counted_across_it() -> None:
    feed, printer, mcu = make_metering({"ff_notice_every": 10})
    ff_frame(mcu, taut=(65530, 0, 0, 0))  # the baseline
    before = len(output(printer))
    ff_frame(mcu, taut=(3, 0, 0, 0))  # 65530 → 3 is 9 across the wrap, not a new arm
    assert feed.ff.net[0] == 9 and len(output(printer)) == before
    ff_frame(mcu, taut=(4, 0, 0, 0))
    assert len(output(printer)) == before + 1


def test_taut_corrections_with_the_lane_off_say_nothing() -> None:
    # the plain follow corrects on every taut: not the metering's signature
    feed, printer, mcu = make_metering()
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    before = len(output(printer))
    ff_frame(mcu)  # the baseline
    for n in range(1, 41):
        ff_frame(mcu, taut=(n, 0, 0, 0))
    assert output(printer)[before:] == [] and feed.ff.net[0] == 0
    assert feed.lane_status(0)["ff_taut_fixes"] == 40


def test_taut_corrections_with_feed_forward_false_say_nothing() -> None:
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


def test_taut_corrections_at_rate_zero_say_nothing() -> None:
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


def test_switching_on_mid_epoch_credits_only_what_comes_after() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder", "feed_forward": False}, extruders={"extruder": extruder}
    )
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    before = len(output(printer))
    ff_frame(mcu, epoch=(4, 0, 0, 0))  # the baseline
    ff_frame(mcu, taut=(15, 0, 0, 0), epoch=(4, 0, 0, 0))  # off: not credited
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "ON": "1"}))
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
        " (ACEK_CALIBRATE_ENCODER)"
    ]


def test_a_rate_falling_to_zero_just_before_the_frame_still_credits_its_interval() -> None:
    feed, printer, mcu = make_metering()
    ff_frame(mcu)  # the baseline
    ff_frame(mcu, taut=(3, 0, 0, 0))
    feed.ff._tick(13.5)  # nothing planned past the moves: 0 sent
    assert bases(mcu)[-1] == [0, 0, 0]
    ff_frame(mcu, taut=(5, 0, 0, 0))  # metered until just now: credited
    assert feed.ff.net[0] == 5
    ff_frame(mcu, taut=(9, 0, 0, 0))  # a whole interval at 0: not
    assert feed.ff.net[0] == 5


def test_a_rate_arriving_just_before_the_frame_does_not_credit_its_interval() -> None:
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


def test_a_lane_switched_off_credits_its_last_interval_then_nothing() -> None:
    feed, printer, mcu = make_metering()
    ff_frame(mcu)  # the baseline
    ff_frame(mcu, taut=(4, 0, 0, 0))
    printer.objects["gcode"].commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    feed.ff._tick(10.3)
    ff_frame(mcu, taut=(6, 0, 0, 0))  # metered until the switch: credited
    assert feed.ff.net[0] == 6
    feed.ff._tick(10.4)
    for n in range(7, 50):
        ff_frame(mcu, taut=(n, 0, 0, 0))  # off for the whole interval: not
    assert feed.ff.net[0] == 6


def test_after_a_shutdown_the_tick_stops_and_sends_nothing() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    extrude(extruder, 10.0, 3.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    printer.shutdown = True
    assert feed.ff._tick(10.2) == printer.reactor.NEVER
    assert bases(mcu) == []


def test_after_a_shutdown_a_remap_sends_no_clear() -> None:
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


def test_an_image_whose_ff_frame_has_no_epoch_is_off_with_one_line() -> None:
    old = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s"
    mcu = FakeMcu(dict(RESPONSES), constants=WITHOUT_REVERSE, formats={"ace2k_feed_ff_state": old})
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


def test_an_image_without_the_feed_forward_is_off_with_one_line_and_never_sent_to() -> None:
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
        gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"OFF": "1"}))
    assert bases(mcu) == [] and not any("ff" in fmt for fmt, _ in mcu.commands)
    assert feed.lane_status(0)["ff_on"] is False


# --- the reverse: which image ----------------------------------------------------------------


def test_an_image_with_the_reverse_takes_signed_bases_and_says_nothing() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"}, extruders={"extruder": extruder})
    assert feed.has_ff and feed.has_ff_reverse
    assert not any("long retraction" in line for line in output(printer))
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.5)
    assert [(fmt, data) for fmt, data in mcu.commands if "feed_base" in fmt] == [
        (BASE_FMT, [0, 3000, 0])
    ]


def test_an_image_without_the_reverse_keeps_the_old_formats_and_says_so_once() -> None:
    extruder = MoveExtruder()
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder"},
        extruders={"extruder": extruder},
        constants=WITHOUT_REVERSE,
        formats=OLD_FORMATS,
    )
    assert feed.has_ff and not feed.has_ff_reverse
    assert [line for line in output(printer) if "long retraction" in line] == [
        "ace2k: the unit's firmware does not take back a long retraction; flash v0.12.0"
    ]
    extrude(extruder, 10.0, 2.0, 3.0)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    feed.ff._tick(10.5)
    assert [(fmt, data) for fmt, data in mcu.commands if "feed_base" in fmt] == [
        (BASE_FMT_FORWARD, [0, 3000, 0])
    ]


def test_an_image_with_the_reverse_constant_and_an_old_base_is_refused() -> None:
    with pytest.raises(Exception, match="ace2k_feed_base"):
        make_ff(
            {"lane1_extruder": "extruder"},
            formats={"ace2k_feed_base": BASE_FMT_FORWARD},
        )


def test_the_reverse_doses_land_in_the_lane_status() -> None:
    feed, printer, mcu = make_ff({"lane1_extruder": "extruder"})
    ff_frame(mcu, doses=(4, 0, 0, 0), back=(7, 0, 0, 1))
    assert feed.lanes[0]["ff_back_doses"] == 7 and feed.lanes[3]["ff_back_doses"] == 1
    feed2, _p, mcu2 = make_ff(
        {"lane1_extruder": "extruder"}, constants=WITHOUT_REVERSE, formats=OLD_FORMATS
    )
    ff_frame(mcu2, doses=(4, 0, 0, 0))
    assert feed2.lanes[0]["ff_back_doses"] is None


def test_without_the_reverse_no_path_sends_a_negative_rate() -> None:
    feed, printer, mcu = make_ff(
        {"lane1_extruder": "extruder"}, constants=WITHOUT_REVERSE, formats=OLD_FORMATS
    )
    feed.ff._send(0, -5.0)
    assert bases(mcu) == [[0, 0, 0]]
    assert feed.ff.rates[0] == 0.0


# --- the reverse: long retractions ---------------------------------------------------------


def make_reverse(
    options: dict[str, Any] | None = None, **kwargs: object
) -> tuple[Ace2kFeed, FakePrinter, FakeMcu, MoveExtruder]:
    extruder = MoveExtruder()
    opts = {"lane1_extruder": "extruder", "ff_deadband": 0.0}
    opts.update(options or {})
    feed, printer, mcu = make_ff(opts, extruders={"extruder": extruder}, **kwargs)
    set_modes(mcu, [FOLLOWING, 0, 0, 0])
    return feed, printer, mcu, extruder


def run_rates(feed: Ace2kFeed, start: float, end: float, step: float = 0.1) -> list[float]:
    """Tick from start to end; the rate in force after each tick."""
    rates = []
    for k in range(round((end - start) / step) + 1):
        feed.ff._tick(start + k * step)
        rates.append(feed.ff.rates[0])
    return rates


def test_a_retraction_under_the_threshold_is_credited_and_never_reversed() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-8.0, cruise_t=0.2))
    run_rates(feed, 9.6, 10.6)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert feed.ff.credit["extruder"] == pytest.approx(8.0)


def test_a_retraction_of_12_mm_takes_it_back() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))  # 50 mm/s
    rates = run_rates(feed, 9.6, 11.0)
    assert min(rate for _lane, rate, _clear in bases(mcu)) < 0
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    assert sum(r * 0.1 for r in rates) == pytest.approx(-12.0, abs=0.3)


def test_a_wipe_split_retraction_is_one_run() -> None:
    feed, printer, mcu, extruder = make_reverse()
    for k in range(3):  # XY moves between them carry no E: never recorded
        extruder.move(10.0 + 0.3 * k, FakeMove(-5.0, cruise_t=0.1))
    rates = run_rates(feed, 9.6, 11.2)
    assert min(rates) < 0
    assert feed.ff.credit.get("extruder", 0.0) == 0.0
    assert sum(r * 0.1 for r in rates) == pytest.approx(-15.0, abs=0.4)


def test_an_unretraction_ends_the_run() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    extruder.move(10.5, FakeMove(6.0, cruise_t=0.1))
    extruder.move(11.0, FakeMove(-6.0, cruise_t=0.1))
    run_rates(feed, 9.6, 11.6)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))


def tip_forming(extruder: MoveExtruder, start_pt: float) -> float:
    """The U1's standard tip forming (CONTROL_RETRACT_ACTION): (E mm, F mm/min) in turn."""
    pt = start_pt
    for e, f in (
        (20, 400),
        (10, 1500),
        (-35, 3000),
        (-5, 50),
        (25, 1000),
        (15, 1500),
        (-35, 3300),
        (-5, 25),
        (-30, 1500),
    ):
        duration = abs(e) / (f / 60.0)
        extruder.move(pt, FakeMove(float(e), cruise_t=duration))
        pt += duration
    return pt


def test_the_tip_forming_is_taken_back_and_fed_again() -> None:
    feed, printer, mcu, extruder = make_reverse()
    end = tip_forming(extruder, 10.0)
    rates = run_rates(feed, 9.6, end + 1.0)
    assert min(rates) < 0 and max(rates) > 0
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-9)
    assert sum(r * 0.1 for r in rates) == pytest.approx(-40.0, abs=1.0)


def test_a_run_that_grows_long_keeps_what_was_already_credited() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.12))
    run_rates(feed, 9.6, 10.2)
    assert feed.ff.credit["extruder"] == pytest.approx(6.0)
    # within a window of the first's end: the same run, 12 mm, long from here
    extruder.move(10.55, FakeMove(-6.0, cruise_t=0.12))
    rates = run_rates(feed, 10.3, 11.5)
    assert min(rates) < 0
    assert feed.ff.credit["extruder"] == pytest.approx(6.0)


def test_a_held_lane_gets_no_negative_rate() -> None:
    feed, printer, mcu, extruder = make_reverse()
    feed.ff.hold(0, True)
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 11.0)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))


def test_the_threshold_is_live_and_bounded() -> None:
    feed, printer, mcu, extruder = make_reverse()
    gcode = printer.objects["gcode"]
    gcmd = FakeGcmd({"RETRACT_MIN_MM": "15"})
    gcode.commands["ACEK_FF_SET"](gcmd)
    assert feed.ff.retract_min_mm == 15.0
    assert any("staged" in line for line in gcmd.lines)  # the reply is the command's own
    assert ("ace2k", "ff_retract_min_mm", "15") in printer.objects["configfile"].calls
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 11.0)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert feed.ff.credit["extruder"] == pytest.approx(12.0)
    with pytest.raises(Exception, match="RETRACT_MIN_MM"):
        gcode.commands["ACEK_FF_SET"](FakeGcmd({"RETRACT_MIN_MM": "18"}))
    assert feed.ff.retract_min_mm == 15.0


def test_without_the_reverse_a_long_retraction_is_credited() -> None:
    feed, printer, mcu, extruder = make_reverse(constants=WITHOUT_REVERSE, formats=OLD_FORMATS)
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 11.0)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert feed.ff.credit["extruder"] == pytest.approx(12.0)


def test_a_long_retraction_goes_out_at_its_own_rate_not_ahead_of_it() -> None:
    # −35 mm at 55 mm/s: the long part is its mean over one period, so the rate reaches the
    # retraction's own speed and does not lead the move by half a window
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(10.0, cruise_t=0.4))  # +10 at 25 mm/s first
    extruder.move(10.4, FakeMove(-35.0, cruise_t=35.0 / 55.0))
    times = [round(9.6 + k * 0.1, 6) for k in range(round((11.6 - 9.6) / 0.1) + 1)]
    rates = run_rates(feed, 9.6, 11.6)
    assert min(rates) <= -50.0
    # no tick whose period ends before the retraction starts sends a negative rate
    for now, rate in zip(times, rates):
        if now + 0.1 <= 10.4 + 1e-9:
            assert rate >= 0.0, (now, rate)
    assert sum(r * 0.1 for r in rates) == pytest.approx(10.0 - 35.0, abs=0.5)


def test_a_late_tick_keeps_a_long_retractions_total() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-35.0, cruise_t=35.0 / 55.0))
    times = [round(9.6 + k * 0.1, 6) for k in range(round((11.6 - 9.6) / 0.1) + 1)]
    times = [t for t in times if not 10.0 < t < 10.4]  # the reactor late inside the move
    assert run_irregular(feed, times) == pytest.approx(-35.0, abs=0.3)
    assert feed.ff.credit.get("extruder", 0.0) == 0.0


def test_a_retraction_after_a_gap_opens_a_new_run() -> None:
    # −30 at t, then −2 at t + 2 s with no E between: the −2 is a print's retraction, credited
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-30.0, cruise_t=0.6))
    run_rates(feed, 9.6, 11.5)
    sent = len(bases(mcu))
    extruder.move(12.6, FakeMove(-2.0, cruise_t=0.05))
    run_rates(feed, 11.6, 13.5)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu)[sent:])
    assert feed.ff.credit["extruder"] == pytest.approx(2.0)


def test_a_shutdown_and_a_new_lead_close_the_open_run() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    assert "extruder" in feed.ff.runs
    feed.ff.set_lead(100)
    assert feed.ff.runs == {}
    extruder.move(10.2, FakeMove(-6.0, cruise_t=0.1))
    printer.shutdown = True
    feed.ff._tick(10.0)
    assert feed.ff.runs == {}


def test_a_lane_switched_off_gets_no_negative_rate() -> None:
    feed, printer, mcu, extruder = make_reverse()
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd({"LANE": "1", "OFF": "1"}))
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 11.0)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert feed.ff.rates[0] == 0.0


def test_two_lanes_in_follow_of_one_extruder_get_no_negative_rate() -> None:
    feed, printer, mcu, extruder = make_reverse({"lane3_extruder": "extruder"})
    set_modes(mcu, [FOLLOWING, 0, FOLLOWING, 0])
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 11.0)
    assert bases(mcu)  # each lane told once to drop what it owes
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert feed.ff.rates[0] == feed.ff.rates[2] == 0.0


def test_a_lead_does_not_send_the_long_part_ahead_of_the_retraction() -> None:
    # with a lead the rest's window starts ahead of now, but the long part is the retraction's
    # mean over the period from now: a reverse dose ahead of the head pulls the strand taut
    feed, printer, mcu, extruder = make_reverse({"ff_lead_ms": 300})
    extruder.move(10.0, FakeMove(10.0, cruise_t=0.4))  # +10 at 25 mm/s first
    extruder.move(10.4, FakeMove(-35.0, cruise_t=35.0 / 55.0))
    # from a window and a lead before the +10, so the whole of it is swept
    times = [round(9.0 + k * 0.1, 6) for k in range(round((11.6 - 9.0) / 0.1) + 1)]
    rates = run_rates(feed, 9.0, 11.6)
    assert min(rates) <= -50.0
    for now, rate in zip(times, rates):
        if now + 0.1 <= 10.4 + 1e-9:
            assert rate >= 0.0, (now, rate)
    assert sum(r * 0.1 for r in rates) == pytest.approx(10.0 - 35.0, abs=0.5)


def shared_reverse() -> tuple[Ace2kFeed, FakeMcu]:
    """Lanes 1 and 2 on one extruder, lane 1 following, a 12 mm retraction recorded and half
    taken back by lane 1; then lane 1 leaves the follow and lane 2 joins it."""
    feed, printer, mcu, extruder = make_reverse({"lane2_extruder": "extruder"})
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    run_rates(feed, 9.6, 10.1)
    assert min(rate for lane, rate, _clear in bases(mcu) if lane == 0) < 0
    return feed, mcu


def lane2_rates(feed: Ace2kFeed, mcu: FakeMcu) -> list[float]:
    set_modes(mcu, [0, FOLLOWING, 0, 0])
    for k in range(12):
        feed.ff._tick(10.2 + k * 0.1)
    return [rate for lane, rate, _clear in bases(mcu) if lane == 1]


def test_a_lane_that_joins_without_forget_is_taken_back_for_the_other_lanes_retraction() -> None:
    # what forget is for: the extruder's recorded retraction is the leaving lane's
    feed, mcu = shared_reverse()
    assert min(lane2_rates(feed, mcu), default=0.0) < 0


def test_forget_leaves_the_joining_lane_nothing_of_the_leaving_lanes_moves() -> None:
    feed, mcu = shared_reverse()
    feed.ff.forget("extruder")
    assert list(feed.ff.moves["extruder"]) == []
    assert "extruder" not in feed.ff.runs and "extruder" not in feed.ff.credit
    assert all(rate >= 0 for rate in lane2_rates(feed, mcu))


def test_forget_of_one_extruder_keeps_the_others() -> None:
    feed, printer, mcu, extruder = make_reverse()
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    run_rates(feed, 9.6, 10.3)
    assert feed.ff.credit["extruder"] > 0.0
    extruder.move(20.0, FakeMove(5.0, cruise_t=0.1))  # planned, still ahead
    feed.ff.forget("extruder1")  # not hooked: nothing recorded, nothing to forget
    assert feed.ff.credit["extruder"] > 0.0 and len(feed.ff.moves["extruder"]) == 1


@pytest.mark.parametrize("how", ["held", "off"])
def test_a_retraction_while_the_lane_drops_the_extruder_does_not_join_the_next(how: str) -> None:
    # −6 while the lane is held (or switched off), then −6 within a window once it is back: the
    # drop closed the open run, so the second is a print's retraction, credited
    feed, printer, mcu, extruder = make_reverse()
    gcode = printer.objects["gcode"]

    def release(on: bool) -> None:
        if how == "held":
            feed.ff.hold(0, not on)
        else:
            flag = {"ON": "1"} if on else {"OFF": "1"}
            gcode.commands["ACEK_FEED_FORWARD"](FakeGcmd(dict(LANE="1", **flag)))

    release(False)
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    run_rates(feed, 9.6, 10.3)
    release(True)
    extruder.move(10.35, FakeMove(-6.0, cruise_t=0.1))
    assert feed.ff.runs["extruder"].total == pytest.approx(6.0)  # a run of its own
    sent = len(bases(mcu))
    run_rates(feed, 10.4, 11.2)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu)[sent:])
    assert feed.ff.credit["extruder"] > 0.0


@pytest.mark.parametrize("lead_ms, split", [(0, 9.9), (300, 9.6), (1000, 8.9)])
def test_a_run_that_turns_long_late_counts_each_moment_once(lead_ms: int, split: float) -> None:
    # −6 recorded at 10.0, swept in part as the rest (credited) under the lead's window, then a
    # −6 joining within a window: the run turns long.  The first −6 keeps its class — credited,
    # never also taken back — and the second is taken back: what was sent plus what is still
    # credited is the planned −12, whatever the lead
    feed, printer, mcu, extruder = make_reverse({"ff_lead_ms": lead_ms})
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    rates = run_rates(feed, 7.0, split)
    assert feed.ff.credit["extruder"] > 0.0  # part of it already credited
    extruder.move(10.5, FakeMove(-6.0, cruise_t=0.1))
    assert feed.ff.runs["extruder"].total == pytest.approx(12.0)
    rates += run_rates(feed, split + 0.1, 12.0)
    taken = sum(r * 0.1 for r in rates)
    credited = feed.ff.credit.get("extruder", 0.0)
    assert taken == pytest.approx(-6.0, abs=0.3)
    assert credited == pytest.approx(6.0, abs=0.3)
    assert taken - credited == pytest.approx(-12.0, abs=0.3)


@pytest.mark.parametrize("lead_ms", [0, 300])
def test_the_long_retractions_integrals_hold_under_a_lead(lead_ms: int) -> None:
    # the 12 mm, the wipe-split 15 mm and the tip forming, ticked from well before them
    cases = [
        (lambda e: (e.move(10.0, FakeMove(-12.0, cruise_t=0.24)), 11.0)[1], -12.0, 0.3),
        (
            lambda e: (
                [e.move(10.0 + 0.3 * k, FakeMove(-5.0, cruise_t=0.1)) for k in range(3)],
                11.2,
            )[1],
            -15.0,
            0.4,
        ),
        (lambda e: tip_forming(e, 10.0) + 1.0, -40.0, 1.0),
    ]
    for plan, net, tol in cases:
        feed, printer, mcu, extruder = make_reverse({"ff_lead_ms": lead_ms})
        end = plan(extruder)
        rates = run_rates(feed, 8.5, end)
        assert min(rates) < 0
        assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-9)
        assert sum(r * 0.1 for r in rates) == pytest.approx(net, abs=tol)


@pytest.mark.parametrize("lead_ms", [0, 300])
def test_a_short_window_under_a_long_period_counts_each_moment_once(lead_ms: int) -> None:
    # window 100 ms, period 1000 ms: the long part's period reaches a move before the rest's
    # window does.  A −6 reached that way while its run is short, then a −6 joining within a
    # window: the first is credited and the second taken back, −12 in all, none of it twice
    opts = {"ff_lead_ms": lead_ms, "ff_window_ms": 100, "ff_period_ms": 1000}
    feed, printer, mcu, extruder = make_reverse(opts)
    extruder.move(10.0, FakeMove(-6.0, cruise_t=0.1))
    rates = run_rates(feed, 7.0, 9.1)
    extruder.move(10.15, FakeMove(-6.0, cruise_t=0.1))
    assert feed.ff.runs["extruder"].total == pytest.approx(12.0)
    rates += run_rates(feed, 9.2, 12.0)
    taken = sum(r * 0.1 for r in rates)
    credited = feed.ff.credit.get("extruder", 0.0)
    assert taken == pytest.approx(-6.0, abs=0.3)
    assert credited == pytest.approx(6.0, abs=0.3)
    assert taken - credited == pytest.approx(-12.0, abs=0.3)
    # the 12 mm, long from its first move: all of it taken back, nothing credited
    feed, printer, mcu, extruder = make_reverse(opts)
    extruder.move(10.0, FakeMove(-12.0, cruise_t=0.24))
    rates = run_rates(feed, 7.0, 12.0)
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-9)
    assert sum(r * 0.1 for r in rates) == pytest.approx(-12.0, abs=0.3)


def short_window_ff() -> tuple[Ace2kFeed, FakePrinter, FakeMcu, MoveExtruder]:
    """window 100 ms under a period of 1000 ms, ticked once a period as the reactor does."""
    return make_reverse({"ff_window_ms": 100, "ff_period_ms": 1000})


def test_a_window_under_the_period_meters_a_steady_extrusion() -> None:
    feed, printer, mcu, extruder = short_window_ff()
    extrude(extruder, 10.0, 10.0, 3.0)
    rates = run_rates(feed, 8.0, 22.0, step=1.0)
    assert sum(r * 1.0 for r in rates) == pytest.approx(30.0, abs=0.5)


def test_a_window_under_the_period_meters_short_bursts_between_its_samples() -> None:
    # 0.3 s at 5 mm/s every second, between where 100 ms windows a period apart would look
    feed, printer, mcu, extruder = short_window_ff()
    for k in range(10):
        extrude(extruder, 10.4 + k, 0.3, 5.0)
    rates = run_rates(feed, 8.0, 22.0, step=1.0)
    assert sum(r * 1.0 for r in rates) == pytest.approx(15.0, abs=0.5)


def test_a_window_under_the_period_nets_a_print_cycle_of_retractions() -> None:
    # each second: retract 0.8, travel, unretract 0.8, extrude 1.5 — a net of +1.5 a cycle
    feed, printer, mcu, extruder = short_window_ff()
    for k in range(10):
        extruder.move(10.2 + k, FakeMove(-0.8, cruise_t=0.05))
        extruder.move(10.4 + k, FakeMove(0.8, cruise_t=0.05))
        extrude(extruder, 10.5 + k, 0.3, 5.0)
    rates = run_rates(feed, 8.0, 22.0, step=1.0)
    assert all(rate >= 0 for _lane, rate, _clear in bases(mcu))
    assert sum(r * 1.0 for r in rates) == pytest.approx(15.0, abs=0.5)
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-6)


def test_a_window_under_the_period_keeps_a_wipe_split_retraction_one_run() -> None:
    # 0.3 s between the −5s: more than the 100 ms window, within the period it is raised to
    feed, printer, mcu, extruder = short_window_ff()
    for k in range(3):
        extruder.move(10.0 + 0.3 * k, FakeMove(-5.0, cruise_t=0.1))
    assert feed.ff.runs["extruder"].total == pytest.approx(15.0)
    rates = run_rates(feed, 8.0, 13.0, step=1.0)
    assert min(rates) < 0
    assert feed.ff.credit.get("extruder", 0.0) == pytest.approx(0.0, abs=1e-9)
    assert sum(r * 1.0 for r in rates) == pytest.approx(-15.0, abs=0.4)
