"""ace2k_encoder_cal.py: the automatic encoder calibration — the marks, the fit and the verdict
(pure), and the run against a simulated lane."""

import collections
import random
import re
import statistics
import struct

import ace2k
import ace2k_encoder_cal as cal
import ace2k_feed
import pytest
from test_ace2k_extras import FakeGcmd
from test_ace2k_ff import FOLLOWING, make_ff, set_modes


def test_the_baseline_and_a_quiet_frame_make_no_mark():
    t = cal.MarkTracker()
    assert t.counts(10.0, 7, 3) is None  # the baseline
    assert t.counts(10.1, 7, 3) is None  # no increment


def test_the_runs_first_episode_is_dropped_and_the_second_is_a_mark():
    t = cal.MarkTracker()
    t.counts(10.0, 7, 3)
    t.reading(10.05, 100.0)
    t.reading(10.15, 100.0)
    assert t.counts(10.2, 8, 3) is None  # the first episode: the plunger's start unknown
    # the burst: the encoder moves, then holds
    t.reading(10.25, 110.0)
    t.reading(10.35, 115.0)
    t.reading(10.45, 115.0)
    t.reading(10.55, 115.0)
    assert t.counts(13.0, 8, 3) is None
    mark = t.counts(13.1, 9, 3)
    assert mark == (pytest.approx(13.05), 115.0)
    assert t.episodes == 2 and t.dropped == 0


def test_a_mark_needs_a_still_reading_after_the_previous_episode():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)  # the first episode, seen in the frame at 10.1
    t.reading(10.05, 50.0)  # before that frame: may predate its burst
    t.reading(10.08, 50.0)
    t.reading(10.2, 60.0)  # only one reading after it
    assert t.counts(12.0, 1, 0) is None
    assert t.counts(12.1, 2, 0) is None  # no two readings after 10.1 before 12.0 that agree
    assert t.dropped == 1


def test_a_still_pair_from_before_the_previous_episodes_frame_is_not_used():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)  # the first episode, seen in the frame at 10.1
    t.reading(10.05, 50.0)  # still, but taken before that frame: may predate its burst
    t.reading(10.08, 50.0)
    assert t.counts(12.0, 1, 0) is None
    assert t.counts(12.1, 2, 0) is None
    assert t.dropped == 1


def test_a_mark_reads_the_encoder_before_the_earlier_frame():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)  # the first episode
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)
    assert t.counts(11.0, 1, 0) is None
    t.reading(11.05, 64.0)  # between the two frames: the burst may have started
    assert t.counts(11.1, 2, 0) == (pytest.approx(11.05), 60.0)


def test_a_moving_encoder_drops_the_mark():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)
    t.reading(10.2, 60.0)
    t.reading(10.3, 61.2)  # still moving
    assert t.counts(10.4, 1, 0) is None
    assert t.counts(10.5, 2, 0) is None
    assert t.dropped == 1


def test_the_last_reading_must_be_still_not_an_older_pair():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)  # the first episode
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)  # still here...
    t.reading(10.4, 61.2)  # ...but moving just before the earlier frame
    assert t.counts(11.0, 1, 0) is None
    assert t.counts(11.1, 2, 0) is None
    assert t.dropped == 1


def test_two_increments_in_one_interval_are_dropped():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    t.counts(10.1, 1, 0)
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)
    assert t.counts(11.0, 1, 0) is None
    assert t.counts(11.1, 3, 0) is None
    assert t.dropped == 2 and t.episodes == 3


def test_a_second_increment_in_the_first_interval_is_a_mark_dropped():
    t = cal.MarkTracker()
    t.counts(10.0, 0, 0)
    assert t.counts(10.1, 2, 0) is None  # the first episode, and a second in the same interval
    assert t.episodes == 2 and t.dropped == 1
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)
    assert t.counts(11.0, 2, 0) is None
    assert t.counts(11.1, 3, 0) == (pytest.approx(11.05), 60.0)  # the run goes on
    assert t.episodes == 3 and t.dropped == 1


def test_the_taut_count_wraps_at_uint16():
    t = cal.MarkTracker()
    t.counts(10.0, 65534, 0)
    t.counts(10.1, 65535, 0)  # the first episode
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)
    assert t.counts(11.0, 65535, 0) is None
    assert t.counts(11.1, 0, 0) == (pytest.approx(11.05), 60.0)
    assert t.episodes == 2 and t.dropped == 0


def test_two_increments_across_the_wrap_are_dropped():
    t = cal.MarkTracker()
    t.counts(10.0, 65534, 0)
    t.counts(10.1, 65535, 0)  # the first episode
    t.reading(10.2, 60.0)
    t.reading(10.3, 60.0)
    assert t.counts(11.0, 65535, 0) is None
    assert t.counts(11.1, 1, 0) is None  # 65535 -> 1: two increments across the wrap
    assert t.episodes == 3 and t.dropped == 2


def test_an_epoch_change_raises():
    t = cal.MarkTracker()
    t.counts(10.0, 5, 1)
    with pytest.raises(cal.EpochChanged):
        t.counts(10.1, 0, 2)


def test_fit_line_is_exact_on_a_line():
    points = [(x, 3.0 + 0.99 * x) for x in (0.0, 10.0, 20.0, 35.0)]
    slope, icpt, rms = cal.fit_line(points)
    assert slope == pytest.approx(0.99)
    assert icpt == pytest.approx(3.0)
    assert rms == pytest.approx(0.0, abs=1e-12)


def test_fit_line_rms_is_the_root_mean_square_of_the_residuals():
    # the line y = 3 + 0.99 x, each point 1 mm above or below it
    points = [(x, 3.0 + 0.99 * x + d) for x in (0.0, 10.0) for d in (1.0, -1.0)]
    slope, icpt, rms = cal.fit_line(points)
    assert slope == pytest.approx(0.99)
    assert icpt == pytest.approx(3.0)
    assert rms == pytest.approx(1.0)


def marks_for(scale_true, scale_in_use, n=31, step=15.0, noise=None):
    """(extruder_mm, encoder_mm): the encoder reads the filament at scale_in_use / scale_true."""
    k = scale_in_use / scale_true
    out = []
    for i in range(n):
        e = 10.0 + i * step
        enc = 1000.0 + e * k + (noise[i] if noise else 0.0)
        out.append((e, enc))
    return out


UNIT_LOW, UNIT_HIGH = 1.2342 * 0.8, 1.2342 * 1.2  # the unit's window: ±20 % of 1.2342


def paired_marks(offset):
    """Ten marks, in pairs `offset` mm above and below one line: the RMS off it is `offset`."""
    return [
        (e, 1000.0 + 0.99 * e + side * offset)
        for e in (10.0, 60.0, 110.0, 160.0, 210.0)
        for side in (1.0, -1.0)
    ]


def test_judge_recovers_the_scale():
    v = cal.judge(marks_for(1.2468, 1.2342), 1.2342, 0.9874, 1.4810)
    assert v["refusal"] is None
    assert v["new"] == pytest.approx(1.2468, rel=1e-9)
    assert v["change"] == pytest.approx(1.2468 / 1.2342 - 1.0)
    assert v["n"] == 31 and not v["warn"]


def test_judge_refuses_few_marks_scatter_and_the_window():
    assert "marks" in cal.judge(marks_for(1.2468, 1.2342, n=9), 1.2342, 0.9874, 1.4810)["refusal"]
    noisy = marks_for(1.2468, 1.2342, noise=[(-1) ** i * 1.5 for i in range(31)])
    assert "scatter" in cal.judge(noisy, 1.2342, 0.9874, 1.4810)["refusal"]
    assert "window" in cal.judge(marks_for(1.60, 1.2342), 1.2342, 0.9874, 1.4810)["refusal"]
    flat = [(10.0 + i, 1000.0) for i in range(12)]
    assert "follow" in cal.judge(flat, 1.2342, 0.9874, 1.4810)["refusal"]


def test_judge_warns_above_three_percent():
    v = cal.judge(marks_for(1.2342 * 1.05, 1.2342), 1.2342, 0.9874, 1.4810)
    assert v["refusal"] is None and v["warn"]
    assert v["new"] == 1.2959 and v["change"] == pytest.approx(1.2959 / 1.2342 - 1.0)


def test_judge_takes_exactly_ten_marks():
    v = cal.judge(marks_for(1.2468, 1.2342, n=10), 1.2342, 0.9874, 1.4810)
    assert v["refusal"] is None and v["n"] == 10


def test_judge_draws_its_lines_at_one_millimetre_and_three_percent_either_way():
    under = marks_for(1.2468, 1.2342, noise=[(-1) ** i * 0.95 for i in range(31)])
    v = cal.judge(under, 1.2342, 0.9874, 1.4810)
    assert v["refusal"] is None and v["scatter"] == pytest.approx(0.95, abs=0.01)
    for factor, warn in ((1.029, False), (1.031, True), (0.971, False), (0.969, True)):
        v = cal.judge(marks_for(1.2342 * factor, 1.2342), 1.2342, 0.9874, 1.4810)
        assert v["refusal"] is None and v["warn"] is warn, factor


def test_judge_refuses_just_above_one_millimetre_and_a_falling_line():
    over = marks_for(1.2468, 1.2342, noise=[(-1) ** i * 1.05 for i in range(31)])
    assert "scatter" in cal.judge(over, 1.2342, 0.9874, 1.4810)["refusal"]
    falling = [(10.0 + i, 1000.0 - i) for i in range(12)]
    v = cal.judge(falling, 1.2342, 0.9874, 1.4810)
    assert "follow" in v["refusal"]
    assert {"scatter", "span_e", "span_enc"} <= v.keys() and not {"new", "change"} & v.keys()


def test_judge_takes_the_scatter_to_the_hundredth_it_shows():
    v = cal.judge(paired_marks(1.004), 1.2342, 0.9874, 1.4810)
    assert v["scatter"] == pytest.approx(1.004)
    assert v["refusal"] is None  # shown as 1.00 mm: at most 1 mm
    v = cal.judge(paired_marks(1.006), 1.2342, 0.9874, 1.4810)
    assert v["refusal"].startswith("scatter 1.01 mm, more than 1 mm")


def test_judge_says_how_many_marks_it_had():
    def refusal(marks):
        return cal.judge(marks, 1.2342, 0.9874, 1.4810)["refusal"]

    assert refusal([]).startswith("0 marks, at least 10 needed")
    assert refusal([(10.0, 1000.0)]).startswith("1 mark, at least 10 needed")
    assert refusal(marks_for(1.2468, 1.2342, n=9)).startswith("9 marks, at least 10 needed")


def test_judge_takes_the_window_on_the_scale_it_sends():
    # the scale is staged and sent to 4 decimals: 1.481044 goes out as 1.4810, inside the window
    for true_scale, sent in ((1.481044, 1.4810), (0.987356, 0.9874)):
        v = cal.judge(marks_for(true_scale, 1.2342), 1.2342, UNIT_LOW, UNIT_HIGH)
        assert v["refusal"] is None and v["new"] == sent
    for true_scale, sent in ((1.48106, "1.4811"), (0.98734, "0.9873")):
        v = cal.judge(marks_for(true_scale, 1.2342), 1.2342, UNIT_LOW, UNIT_HIGH)
        assert v["refusal"] == f"{sent} mm/count is outside the unit's window 0.9874..1.4810"


def test_judge_warns_on_the_change_it_shows():
    def shown(v):
        return f"{v['change'] * 100.0:+.2f}"

    v = cal.judge(marks_for(1.2721, 1.2350), 1.2350, UNIT_LOW, UNIT_HIGH)  # +3.004 %
    assert v["new"] == 1.2721 and shown(v) == "+3.00" and not v["warn"]
    v = cal.judge(marks_for(1.2713, 1.2342), 1.2342, UNIT_LOW, UNIT_HIGH)  # +3.006 %
    assert v["new"] == 1.2713 and shown(v) == "+3.01" and v["warn"]
    # +3.004 % off 1.2342 goes out as 1.2713: the change is the sent scale's, +3.006 %
    v = cal.judge(marks_for(1.2342 * 1.03004, 1.2342), 1.2342, UNIT_LOW, UNIT_HIGH)
    assert v["new"] == 1.2713 and shown(v) == "+3.01" and v["warn"]


def test_judge_names_the_marks_dropped_when_told():
    v = cal.judge(marks_for(1.2468, 1.2342, n=8), 1.2342, 0.9874, 1.4810, dropped=3)
    assert v["refusal"] == (
        "8 marks, at least 10 needed (3 dropped) — is the extruder gripping the filament? a larger"
        " LENGTH gives more"
    )
    v = cal.judge([(10.0, 1000.0)], 1.2342, 0.9874, 1.4810, dropped=0)
    assert v["refusal"].startswith("1 mark, at least 10 needed (0 dropped) — ")
    # enough marks: the count dropped changes nothing
    v = cal.judge(marks_for(1.2468, 1.2342), 1.2342, 0.9874, 1.4810, dropped=5)
    assert v["refusal"] is None and v["new"] == 1.2468


# --- the run, against a simulated lane --------------------------------------------------------

ERROR = 7
TANGLED = 8  # an error kind, ace2k_feed.KIND_NAMES
SCALE_SET = "ace2k_lane_scale_set lane=%c um_per_count_x10=%u"
FF_QUERY = "ace2k_feed_ff_query"
COUNTERS_QUERY = "ace2k_lane_counters_query"
SAVE = "SAVE_GCODE_STATE NAME=ACE2K_ENCODER_CAL"
RESTORE = "RESTORE_GCODE_STATE NAME=ACE2K_ENCODER_CAL"
TICKS_10HZ, TICKS_1HZ = 12000000, 120000000


class GcodeError(Exception):
    """gcmd.error, as Klipper's: a G-code error.  Anything else raised from a G-code handler is an
    internal error, which shuts Klipper down."""


class Gcmd(FakeGcmd):
    error = GcodeError


def set_state(mcu, modes, errors):
    """A feed state report with an error kind per lane (set_modes carries none)."""
    mcu.subscriptions["ace2k_feed_state"](
        {
            "mode": bytes(modes),
            "error": bytes(errors),
            "speed_um_s": bytes(16),
            "duty_pct": bytes(4),
            "bursts": bytes(8),
            "seq": bytes(4),
        }
    )


class Heater:
    def __init__(self, hot):
        self.can_extrude = hot


class SimExtruder:
    """The lane's extruder: its name, its heater, the segments the rig planned, and every
    find_past_position call with the simulated time it was made at."""

    def __init__(self, rig, name="extruder", hot=True):
        self.rig = rig
        self.name = name
        self.heater = Heater(hot)
        self.plan = []  # (start_pt, end_pt, start_e, end_e)
        self.lookups = []  # (instant, now)

    def get_name(self):
        return self.name

    def get_heater(self):
        return self.heater

    def move(self, print_time, move):  # what the feed-forward would hook; nothing to do here
        pass

    def position(self, pt):
        last = 0.0
        for start, end, e0, e1 in self.plan:
            if pt < start:
                return e0
            if pt <= end:
                return e0 + (e1 - e0) * (pt - start) / (end - start)
            last = e1
        return last

    def find_past_position(self, pt):
        self.lookups.append((pt, self.rig.now))
        return self.position(pt)


class SimToolhead:
    def __init__(self, rig, extruder):
        self.rig = rig
        self.extruder = extruder

    def get_extruder(self):
        return self.extruder

    def get_last_move_time(self):
        return self.rig.plan_end

    def wait_moves(self):
        if self.rig.printer.shutdown:
            return  # as Klipper's: no pause once shut down
        self.rig.pause(max(self.rig.now, self.rig.plan_end))


class SimGcode:
    """Klipper's G-code dispatch as the run uses it: each line recorded as it runs, a G1 planned
    on the rig.  After a shutdown every line is refused with a G-code error, as Klipper's base
    handlers refuse a command that is not theirs; refuse_g1 = n refuses the n-th G1 (a G-code
    error, the move not planned)."""

    def __init__(self, rig):
        self.rig = rig
        self.scripts = []
        self.output = []
        self.refuse_g1 = None
        self.g1s = 0

    def respond_info(self, msg):
        self.output.append(msg)

    def run_script_from_command(self, script):
        for line in script.splitlines():
            if self.rig.printer.shutdown:
                raise GcodeError("Printer is shutdown")
            if line.startswith("G1 "):
                self.g1s += 1
                if self.g1s == self.refuse_g1:
                    raise GcodeError("Extrude below minimum temp")
            self.scripts.append(line)
            if line.startswith("G1 "):
                words = {w[0]: float(w[1:]) for w in line.split()[1:]}
                self.rig.queue_extrusion(words["E"], words["F"] / 60.0)


class SimStats:
    def __init__(self, state="standby"):
        self.state = state

    def get_status(self, eventtime):
        return {"state": self.state}


class SimLane:
    """Lane 1 in the follow and its extruder, stepped in 10 ms.  The plunger `p` is the filament
    short of rest in mm (0 rest, TRAVEL taut, below 0 toward full): the extruder raises it; at
    TRAVEL the taut count steps and a burst at BURST mm/s brings it back to 0, the filament
    counted by the encoder in whole counts at the true scale and reported at the scale in use.
    With overshoot, each burst stops past rest by a random 0..overshoot mm drawn from `seed`, as
    real bursts stop: identical bursts would make the count's rounding alias coherently.
    The counters frame (the lane's taut and full counts, `full` set by a test) and the lane
    counters at 10 Hz (phases 40 ms and 0), received 3 ms later; `at(t, fn)` runs fn once the
    simulated time reaches t.  At its first step after an ace2k_feed_start in the MCU's log the
    rig enters the follow — the epoch stepped, the counts from 0 — and shows it in the state
    report arm_delay later (arms=False: never); readings=False: no lane-counters frame ever
    arrives; counts_before_run=False: no counters frame until the run listens (at the state
    report's 1 Hz, none fell in the arm's wait).  Print time is reactor time (make_ff's
    clock)."""

    TRAVEL = 15.0
    BURST = 70.0
    DT = 0.01

    def __init__(
        self,
        true_scale=1.2468,
        start=0.0,
        slip_every=0,
        following=True,
        hot=True,
        options=None,
        readings=True,
        arms=True,
        overshoot=0.0,
        seed=0,
        arm_delay=0.0,
        counts_before_run=True,
    ):
        self.feed, self.printer, self.mcu = make_ff(
            options=dict({"lane1_extruder": "extruder"}, **(options or {})), extruders={}
        )
        self.mcu.responses["ace2k_feed_start"] = {"lane": 0, "accepted": 1, "reason": 0}
        self.unit = self.feed.unit
        self.reactor = self.printer.get_reactor()
        self.reactor.pause = self.pause
        self.now = self.reactor.now
        self.extruder = SimExtruder(self, hot=hot)
        self.gcode = SimGcode(self)
        self.stats = SimStats()
        self.printer.add_object("extruder", self.extruder)
        self.printer.add_object("toolhead", SimToolhead(self, self.extruder))
        self.printer.add_object("gcode", self.gcode)
        self.printer.add_object("print_stats", self.stats)
        self.unit.lanes[0]["insert"] = True
        self.unit.pulled_any = False
        self.true_scale = true_scale
        self.slip_every = slip_every
        self.readings = readings
        self.arms = arms
        self.overshoot = overshoot
        self.rng = random.Random(seed)
        self.arm_delay = arm_delay
        self.arm_at = None  # when the state report shows the follow the unit entered
        self.counts_before_run = counts_before_run
        self.p = start
        self.target = 0.0  # where the running burst stops
        self.counted = 0.0  # mm the encoder saw
        self.taut = 0
        self.full = 0
        self.epoch = 0
        self.bursting = False
        self.bursts = 0
        self.plan_end = self.now
        self.events = []  # (t, fn)
        self.armed_seen = len(self.mcu.sent)
        self.next_reading = self._next(0.0)
        self.next_counts = self._next(0.04)
        self.cmd = None
        if following:
            set_modes(self.mcu, [FOLLOWING, 0, 0, 0])

    def _next(self, phase):
        k = int((self.now - phase) / 0.1) + 1
        return k * 0.1 + phase

    def at(self, t, fn):
        self.events.append((t, fn))

    def queue_extrusion(self, e, speed):
        start = max(self.now, self.plan_end)
        e0 = self.extruder.position(start)
        self.extruder.plan.append((start, start + e / speed, e0, e0 + e))
        self.plan_end = start + e / speed

    def encoder_um(self):
        counts = int(self.counted / self.true_scale)
        scale = self.feed.lanes[0]["encoder_scale"]
        return int(round(counts * scale * 1000.0))

    def pause(self, waketime):
        while self.now + 1e-9 < waketime:
            self.step(min(self.DT, waketime - self.now))
        return waketime

    def step(self, dt):
        t0, t1 = self.now, self.now + dt
        started = "ace2k_feed_start" in self.mcu.sent[self.armed_seen :]
        self.armed_seen = len(self.mcu.sent)
        if started and self.arms:
            self.epoch = (self.epoch + 1) & 0xFF
            self.taut = self.full = 0
            self.arm_at = t0 + self.arm_delay
        if self.arm_at is not None and t0 + 1e-9 >= self.arm_at:
            self.arm_at = None
            set_modes(self.mcu, [FOLLOWING, 0, 0, 0])
        self.p += self.extruder.position(t1) - self.extruder.position(t0)
        if self.feed.lanes[0]["mode"] == "following":
            if self.bursting:
                fill = min(self.BURST * dt, self.p - self.target)
                self.p -= fill
                slip = self.slip_every and self.bursts % self.slip_every == 0
                self.counted += fill * (0.5 if slip else 1.0)
                if self.p <= self.target:
                    self.bursting = False
            elif self.p >= self.TRAVEL:
                self.taut = (self.taut + 1) & 0xFFFF
                self.bursts += 1
                self.bursting = True
                self.target = -self.rng.uniform(0.0, self.overshoot) if self.overshoot else 0.0
        self.now = self.reactor.now = t1
        for t, fn in list(self.events):
            if t <= t1:
                self.events.remove((t, fn))
                fn()
        # a frame goes out in the step that ends at its time: the 1 µs allowance absorbs the
        # rounding the two clocks accumulate (10 ms steps against 100 ms frames), which would
        # otherwise send it a step late, carrying the state of the step after its time (the
        # senders check it)
        while self.next_reading <= t1 + 1e-6:
            if self.readings:
                self.send_reading(self.next_reading)
            self.next_reading += 0.1
        while self.next_counts <= t1 + 1e-6:
            if self.counts_before_run or self.feed.cal.lane is not None:
                self.send_counts(self.next_counts)
            self.next_counts += 0.1

    def send_reading(self, t):
        assert abs(self.now - t) < 1e-3, f"the rig sent its {t:.3f} s frame at {self.now:.3f} s"
        self.unit._handle_lane_counters(
            {
                "encoder_um": struct.pack("<4i", self.encoder_um(), 0, 0, 0),
                "fg": bytes(16),
                "#receive_time": t + 0.003,
            }
        )

    def send_counts(self, t):
        assert abs(self.now - t) < 1e-3, f"the rig sent its {t:.3f} s frame at {self.now:.3f} s"
        self.feed._handle_ff_state(
            {
                "doses": bytes(8),
                "taut": struct.pack("<4H", self.taut, 0, 0, 0),
                "full": struct.pack("<4H", self.full, 0, 0, 0),
                "epoch": bytes([self.epoch, 0, 0, 0]),
                "#receive_time": t + 0.003,
            }
        )

    def run(self, **params):
        self.cmd = Gcmd(dict({"LANE": "1", "AUTO": "1"}, **params))
        self.feed.cmd_ACE_CALIBRATE_ENCODER(self.cmd)
        return self.cmd


def scale_sets(rig):
    return [data for fmt, data in rig.mcu.commands if fmt == SCALE_SET]


def periods(rig, query):
    return [data[0] for fmt, data in rig.mcu.commands if fmt.startswith(query)]


def extrusions(rig):
    return [line for line in rig.gcode.scripts if line.startswith("G1 ")]


def assert_restored(rig, ff_ticks=TICKS_1HZ):
    """Everything back after a run that got as far as listening: the hold released, both reports
    at their own rates again, no lane held by the run, the G-code state restored."""
    assert rig.feed.ff.held == set() and rig.feed.cal.lane is None
    assert periods(rig, FF_QUERY)[-2:] == [TICKS_10HZ, ff_ticks]
    assert periods(rig, COUNTERS_QUERY)[-2:] == [TICKS_10HZ, TICKS_1HZ]
    assert rig.gcode.scripts[-1] == RESTORE


@pytest.mark.parametrize(
    "start", [SimLane.TRAVEL, -5.0, SimLane.TRAVEL / 2], ids=["taut", "toward-full", "between"]
)
def test_the_scale_is_read_off_the_extruder_from_any_start(start):
    rig = SimLane(true_scale=1.2468, start=start)
    cmd = rig.run()
    ((lane, x10),) = scale_sets(rig)
    assert lane == 0 and x10 / 10000.0 == pytest.approx(1.2468, rel=1e-3)
    assert rig.feed.lanes[0]["encoder_scale"] == pytest.approx(1.2468, rel=1e-3)
    assert rig.feed.lanes[0]["scale_source"] == "calibrated"
    staged = ("ace2k", "lane1_encoder_scale", f"{x10 / 10000.0:.4f}")
    assert rig.printer.objects["configfile"].calls == [staged]
    # the three lines of the report, and nothing else
    first, marks_line, applied = cmd.lines
    assert first == "ace2k: lane 1: calibrating against extruder — 500 mm at 4 mm/s"
    assert re.fullmatch(
        r"ace2k: lane 1: \d\d marks over \d+\.\d mm of extruder; the encoder read \d+\.\d mm",
        marks_line,
    )
    assert re.fullmatch(
        r"ace2k: lane 1: scale 1\.2342 → 1\.24\d\d mm/count \(\+\d\.\d\d %\), scatter 0\.\d\d mm"
        r" — in use now; SAVE_CONFIG to keep it",
        applied,
    )


def test_only_the_extruder_moves_and_the_gcode_state_is_restored():
    rig = SimLane()
    rig.run()
    assert rig.gcode.scripts[:2] == [SAVE, "M83"]
    assert rig.gcode.scripts[-1] == RESTORE
    moves = rig.gcode.scripts[2:-1]
    assert moves == ["G1 E25.000 F240.0"] * 20
    assert not any(axis in line for line in moves for axis in ("X", "Y", "Z", "T"))


def test_length_and_speed_set_the_chunks_the_last_one_the_rest():
    rig = SimLane()
    cmd = rig.run(LENGTH="310", SPEED="5")
    assert extrusions(rig) == ["G1 E25.000 F300.0"] * 12 + ["G1 E10.000 F300.0"]
    assert cmd.lines[0] == "ace2k: lane 1: calibrating against extruder — 310 mm at 5 mm/s"
    assert len(scale_sets(rig)) == 1


@pytest.mark.parametrize(
    "params, match",
    [
        ({"LENGTH": "199"}, "LENGTH of at least 200"),
        ({"LENGTH": "2001"}, "LENGTH of at most 2000"),
        ({"SPEED": "0"}, "SPEED above 0"),
    ],
)
def test_length_and_speed_outside_their_bounds_are_refused_before_anything_moves(params, match):
    rig = SimLane()
    sent = len(rig.mcu.sent)
    with pytest.raises(GcodeError, match=match):
        rig.run(**params)
    assert rig.gcode.scripts == [] and rig.mcu.sent[sent:] == [] and rig.cmd.lines == []


def test_the_help_names_the_automatic_mode():
    assert "AUTO=1" in ace2k_feed.Ace2kFeed.cmd_ACE_CALIBRATE_ENCODER_help


def test_the_lane_counters_period_between_runs_is_ace2ks():
    assert cal.LANE_REPORT_S == ace2k.LANE_REPORT_S


def make_toolhead_on_another_extruder(rig):
    rig.printer.add_object("toolhead", SimToolhead(rig, SimExtruder(rig, name="extruder1")))


@pytest.mark.parametrize(
    "kwargs, setup, match",
    [
        (
            {},
            lambda rig: setattr(rig.feed, "has_ff", False),
            "the unit's firmware has no feed-forward \\(its taut count\\); flash v0.9.0",
        ),
        (
            {},
            lambda rig: setattr(rig.stats, "state", "printing"),
            "a print is printing; calibrate between prints",
        ),
        ({}, lambda rig: setattr(rig.stats, "state", "paused"), "a print is paused"),
        (
            {"options": {"lane1_extruder": ""}},
            None,
            "lane 1 has no extruder \\(lane1_extruder\\)",
        ),
        ({"options": {"lane1_extruder": "extruder1"}}, None, "lane 1: no extruder extruder1"),
        (
            {"options": {"lane1_extruder": "heater_bed"}},
            lambda rig: rig.printer.add_object("heater_bed", Heater(True)),
            "lane 1: no extruder heater_bed",
        ),
        (
            {},
            make_toolhead_on_another_extruder,
            "extruder is not the active extruder; select it first",
        ),
        ({"hot": False}, None, "extruder is too cold to extrude"),
        ({}, lambda rig: rig.unit.lanes[0].update(insert=False), "lane 1 has no filament"),
        ({}, lambda rig: rig.unit.lanes[0].update(insert=None), "lane 1 has no filament"),
        (
            {},
            lambda rig: set_state(rig.mcu, [ERROR, 0, 0, 0], [TANGLED, 0, 0, 0]),
            "lane 1 is in error \\(tangled\\); ACE_CLEAR first",
        ),
        ({}, lambda rig: set_modes(rig.mcu, [1, 0, 0, 0]), "lane 1 is busy \\(feeding\\)"),
        ({}, lambda rig: setattr(rig.feed.cal, "lane", 2), "a calibration is already running"),
    ],
    ids=[
        "no-feed-forward",
        "printing",
        "paused",
        "unmapped",
        "no-such-extruder",
        "not-an-extruder",
        "not-active",
        "cold",
        "no-filament",
        "filament-unknown",
        "in-error",
        "busy",
        "already-running",
    ],
)
def test_each_precondition_is_refused_with_its_reason_before_anything_is_sent(kwargs, setup, match):
    rig = SimLane(**kwargs)
    if setup is not None:
        setup(rig)
    sent = len(rig.mcu.sent)
    with pytest.raises(GcodeError, match=match):
        rig.run()
    assert rig.gcode.scripts == [] and rig.mcu.sent[sent:] == [] and scale_sets(rig) == []
    assert rig.cmd.lines == [] and rig.extruder.lookups == []


def test_a_cold_nozzle_is_refused_before_anything_moves():
    rig = SimLane(hot=False)
    with pytest.raises(GcodeError, match="too cold to extrude"):
        rig.run()
    assert rig.gcode.scripts == [] and scale_sets(rig) == []


def test_a_full_buffer_aborts_after_the_chunk_and_nothing_changes():
    rig = SimLane()
    rig.at(rig.now + 30.0, lambda: setattr(rig.unit, "pulled_any", True))
    with pytest.raises(GcodeError, match="aborted — the buffer went full; nothing changed"):
        rig.run()
    assert scale_sets(rig) == []
    # 30 s ≈ 120 mm: the fifth chunk was running; it ran to its end, and no sixth was queued
    assert len(extrusions(rig)) == 5 and rig.now >= rig.plan_end - 1e-6
    assert rig.feed.lanes[0]["encoder_scale"] == 1.2342
    assert rig.printer.objects["configfile"].calls == []
    assert_restored(rig)


@pytest.mark.parametrize(
    "event, reason",
    [
        (
            lambda rig: set_state(rig.mcu, [ERROR, 0, 0, 0], [TANGLED, 0, 0, 0]),
            "lane 1 in error \\(tangled\\)",
        ),
        (lambda rig: set_modes(rig.mcu, [0, 0, 0, 0]), "lane 1 left the follow \\(idle\\)"),
        (lambda rig: setattr(rig, "epoch", 9), "lane 1 re-entered a mode"),
    ],
    ids=["error", "out-of-the-follow", "epoch"],
)
def test_an_abort_lets_the_running_chunk_end_and_changes_nothing(event, reason):
    rig = SimLane()
    rig.at(rig.now + 30.0, lambda: event(rig))
    with pytest.raises(GcodeError, match=f"aborted — {reason}; nothing changed"):
        rig.run()
    assert scale_sets(rig) == [] and rig.printer.objects["configfile"].calls == []
    assert len(extrusions(rig)) == 5 and rig.now >= rig.plan_end - 1e-6
    assert_restored(rig)


def test_a_condition_already_there_aborts_before_any_extrusion():
    rig = SimLane()
    rig.unit.pulled_any = True
    with pytest.raises(GcodeError, match="aborted — the buffer went full; nothing changed"):
        rig.run()
    assert rig.gcode.scripts == [SAVE, "M83", RESTORE] and scale_sets(rig) == []
    assert_restored(rig)


def test_a_shutdown_aborts_and_nothing_is_sent_after_it():
    rig = SimLane(following=False)
    set_modes(rig.mcu, [0, 0, 0, 0])  # the run arms the follow: its stop is what must not go
    sent_at_shutdown = []

    def shut_down():
        rig.printer.shutdown = True
        sent_at_shutdown.append(len(rig.mcu.sent))

    rig.at(rig.now + 30.0, shut_down)
    with pytest.raises(GcodeError, match="aborted — Klipper shut down; nothing changed"):
        rig.run()
    (sent,) = sent_at_shutdown
    assert rig.mcu.sent[sent:] == []
    assert scale_sets(rig) == [] and len(extrusions(rig)) <= 6
    assert rig.feed.ff.held == set() and rig.feed.cal.lane is None
    # no G-code after it: Klipper would refuse it, and its error would replace the abort's line
    assert rig.gcode.scripts == [SAVE, "M83"] + extrusions(rig)


def test_a_shutdown_after_the_last_chunk_still_aborts_and_nothing_is_sent():
    # the last look before the verdict: a shutdown once the extrusion is over sends no scale
    rig = SimLane()
    toolhead = rig.printer.lookup_object("toolhead")
    wait_moves = toolhead.wait_moves
    sent_at_shutdown = []

    def wait_then_shut_down():
        wait_moves()
        rig.printer.shutdown = True
        sent_at_shutdown.append(len(rig.mcu.sent))

    toolhead.wait_moves = wait_then_shut_down
    with pytest.raises(GcodeError, match="aborted — Klipper shut down; nothing changed"):
        rig.run()
    assert len(extrusions(rig)) == 20 and RESTORE not in rig.gcode.scripts
    (sent,) = sent_at_shutdown
    assert rig.mcu.sent[sent:] == [] and scale_sets(rig) == []


@pytest.mark.parametrize(
    "rig, params, match",
    [
        # a long travel: 200 mm give too few tauts
        (lambda: type("LongTravel", (SimLane,), {"TRAVEL": 25.0})(), {"LENGTH": "200"}, None),
        (lambda: SimLane(slip_every=3), {}, "scatter \\d+\\.\\d\\d mm, more than 1 mm"),
        (lambda: SimLane(true_scale=1.60), {}, "1\\.[56]\\d\\d\\d mm/count is outside the unit's"),
    ],
    ids=["few-marks", "slipping", "outside-the-window"],
)
def test_a_verdict_it_cannot_defend_is_refused_and_changes_nothing(rig, params, match):
    rig = rig()
    with pytest.raises(GcodeError, match="nothing changed") as refused:
        rig.run(**params)
    if match is None:
        assert re.search(r"\d marks, at least 10 needed \(0 dropped\) — ", str(refused.value))
    else:
        assert re.search(match, str(refused.value))
    assert scale_sets(rig) == [] and rig.printer.objects["configfile"].calls == []
    assert rig.feed.lanes[0]["encoder_scale"] == 1.2342
    assert_restored(rig)


def test_marks_dropped_for_want_of_a_still_encoder_are_counted_in_the_refusal():
    rig = SimLane(readings=False)  # no lane counters: no mark has a still reading
    with pytest.raises(GcodeError) as refused:
        rig.run()
    found = re.search(r"0 marks, at least 10 needed \((\d+) dropped\) — ", str(refused.value))
    assert found and int(found.group(1)) >= 25  # every taut but the run's first
    assert scale_sets(rig) == []


def test_a_change_above_three_percent_is_applied_with_a_warning():
    rig = SimLane(true_scale=1.2342 * 1.05)
    cmd = rig.run()
    ((_, x10),) = scale_sets(rig)
    assert x10 / 10000.0 == pytest.approx(1.2342 * 1.05, rel=1e-3)
    assert cmd.lines[-1] == (
        "ace2k: lane 1: a change above 3 % — check the extruder's own calibration too"
    )
    assert re.search(r"→ 1\.29\d\d mm/count \(\+[45]\.\d\d %\)", cmd.lines[-2])


def test_every_ending_restores_the_hold_and_the_periods():
    rig = SimLane()
    rig.run()
    assert rig.feed.ff.held == set()
    assert periods(rig, FF_QUERY)[-2:] == [TICKS_10HZ, TICKS_1HZ]
    assert periods(rig, COUNTERS_QUERY)[-2:] == [TICKS_10HZ, TICKS_1HZ]
    assert rig.feed.cal.lane is None
    # and before the scale is applied: it goes out with both reports at their own rates again
    names = rig.mcu.sent
    applied = names.index("ace2k_lane_scale_set")
    for query in (FF_QUERY, COUNTERS_QUERY):
        assert len(names) - 1 - names[::-1].index(query) < applied


def test_the_feed_forward_counters_go_back_to_the_state_reports_own_rate():
    rig = SimLane(options={"feed_report_hz": 2})
    rig.run()
    assert_restored(rig, ff_ticks=60000000)  # 0.5 s


def test_the_run_holds_the_lanes_feed_forward_and_drops_what_it_owes():
    rig = SimLane()
    held = []
    rig.at(rig.now + 10.0, lambda: held.append(set(rig.feed.ff.held)))
    rig.run()
    assert held == [{0}]
    # the hold's clear went out before the reports were raised
    names = rig.mcu.sent
    assert names.index("ace2k_feed_base") < names.index(FF_QUERY)
    assert ("ace2k_feed_base lane=%c rate_um_s=%u clear=%c", [0, 0, 1]) in rig.mcu.commands


def test_the_run_arms_the_follow_and_disarms_what_it_armed():
    rig = SimLane(following=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.run()
    # the follow at the assists' speed (assist_speed 50 mm/s), then stopped at the end
    starts = [q for q in rig.mcu.queries if q[0] == "ace2k_feed_start"]
    assert starts == [("ace2k_feed_start", [0, 6, 0, 50000, 1])]
    assert "ace2k_feed_stop" in rig.mcu.sent
    assert len(scale_sets(rig)) == 1
    assert rig.mcu.sent.index("ace2k_feed_stop") < rig.mcu.sent.index("ace2k_lane_scale_set")


def test_a_lane_already_in_the_follow_is_left_in_it():
    rig = SimLane()
    rig.run()
    assert "ace2k_feed_start" not in rig.mcu.sent and "ace2k_feed_stop" not in rig.mcu.sent


def test_a_refused_arm_is_a_gcode_error_and_nothing_moves():
    rig = SimLane(following=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.mcu.responses["ace2k_feed_start"] = {"lane": 0, "accepted": 0, "reason": 1}
    with pytest.raises(GcodeError, match="ace2k: lane 1 assist_both refused: busy"):
        rig.run()
    assert rig.gcode.scripts == [] and "ace2k_feed_stop" not in rig.mcu.sent
    assert periods(rig, FF_QUERY) == [] and rig.feed.ff.held == set()
    assert rig.feed.cal.lane is None


def test_a_follow_that_never_comes_is_refused_and_the_arm_undone():
    rig = SimLane(following=False, arms=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    start = rig.now
    with pytest.raises(GcodeError, match="ace2k: lane 1 did not enter the follow"):
        rig.run()
    assert rig.now - start == pytest.approx(cal.ARM_WAIT_S, abs=0.11)
    assert rig.gcode.scripts == [] and "ace2k_feed_stop" in rig.mcu.sent
    assert periods(rig, FF_QUERY) == [] and rig.feed.ff.held == set()
    assert rig.feed.cal.lane is None


def test_positions_are_looked_up_as_the_frames_arrive():
    rig = SimLane()
    rig.run()
    assert rig.extruder.lookups
    assert all(now - instant <= 1.0 for instant, now in rig.extruder.lookups)


def test_a_frame_read_under_an_earlier_run_never_reaches_the_next():
    # the serial thread reads the run's lane, and is preempted before it appends: the run ends
    # and the next one starts listening before the frame lands
    rig = SimLane()
    seen = []
    rig.at(rig.now + 10.0, lambda: seen.append(rig.feed.cal._sink))
    rig.run()
    (sink,) = seen
    stale = ("counts", 0.0, 999, 77)  # an epoch the lane never had: a run that read it aborts
    rig.at(rig.now + 10.0, lambda: sink.frames.append(stale))
    rig.run()
    assert len(scale_sets(rig)) == 2
    # the same frame, appended by a listener that read the second run's lane, is that run's
    rig.at(rig.now + 10.0, lambda: rig.feed.cal._sink.frames.append(stale))
    with pytest.raises(GcodeError, match="aborted — lane 1 re-entered a mode"):
        rig.run()


class ShutDownAtTheArm(SimLane):
    """Klipper shuts down in the same wake as the state report that first shows the follow."""

    sent_at_shutdown = None

    def step(self, dt):
        before = self.feed.lanes[0]["mode"]
        super().step(dt)
        shows = before != "following" and self.feed.lanes[0]["mode"] == "following"
        if shows and not self.printer.shutdown:
            self.printer.shutdown = True
            self.sent_at_shutdown = len(self.mcu.sent)


def test_a_shutdown_as_the_arm_shows_aborts_and_nothing_is_sent_after_it():
    rig = ShutDownAtTheArm(following=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    with pytest.raises(GcodeError, match="aborted — Klipper shut down; nothing changed"):
        rig.run()
    assert rig.mcu.sent[rig.sent_at_shutdown :] == []  # no hold's clear, no report periods, no stop
    assert rig.gcode.scripts == [] and scale_sets(rig) == []
    assert rig.feed.ff.held == set() and rig.feed.cal.lane is None


def test_a_shutdown_while_the_follow_is_awaited_ends_the_wait():
    # the unit stopped with Klipper: the follow never shows, and the run does not wait it out
    rig = SimLane(following=False, arms=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    start = rig.now
    sent_at_shutdown = []

    def shut_down():
        rig.printer.shutdown = True
        sent_at_shutdown.append(len(rig.mcu.sent))

    rig.at(start + 1.0, shut_down)
    with pytest.raises(GcodeError, match="aborted — Klipper shut down; nothing changed"):
        rig.run()
    assert rig.now - start < 1.2
    (sent,) = sent_at_shutdown
    assert rig.mcu.sent[sent:] == [] and rig.gcode.scripts == []


def test_listening_sends_nothing_after_a_shutdown():
    rig = SimLane()
    rig.printer.shutdown = True
    sent = len(rig.mcu.sent)
    rig.feed.cal._listen(0)
    assert rig.feed.cal.lane == 0 and rig.mcu.sent[sent:] == []
    rig.feed.cal._unlisten()
    assert rig.feed.cal.lane is None and rig.mcu.sent[sent:] == []


SEEDS = range(40)


@pytest.mark.parametrize("travel", [15.0, 16.4])
def test_bursts_that_stop_as_real_ones_do_leave_the_scale_unbiased(travel):
    # every burst of the plain rig stops exactly at rest, so the count's rounding can alias
    # coherently in one run (at a TRAVEL of 16.4 mm, by -0.1 to -0.23 %); real bursts stop past
    # rest by a varying amount.  What 40 seeds resolve: a run spreads by about 0.06 %, so their
    # mean by about 0.01 %, and the mean's 0.02 % bound catches a bias of about 0.04 % or more
    # (a smaller one only by chance); the 0.3 % bound, any of the 40 runs off by more than that
    errors = []
    for seed in SEEDS:
        rig = type("Travel", (SimLane,), {"TRAVEL": travel})(overshoot=1.5, seed=seed)
        rig.run()
        ((_, x10),) = scale_sets(rig)
        errors.append(x10 / 10000.0 / 1.2468 - 1.0)
    assert abs(statistics.fmean(errors)) <= 0.0002  # 0.02 %
    assert max(abs(error) for error in errors) <= 0.003  # 0.3 %


def test_a_gcode_error_mid_extrusion_restores_everything_and_changes_nothing():
    rig = SimLane(following=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.gcode.refuse_g1 = 3
    with pytest.raises(GcodeError, match="Extrude below minimum temp"):
        rig.run()
    assert len(extrusions(rig)) == 2 and scale_sets(rig) == []
    assert "ace2k_feed_stop" in rig.mcu.sent  # the follow the run armed
    assert_restored(rig)


def test_the_arms_wait_spans_two_reports_at_a_slow_report_rate():
    # at 0.2 Hz the state report that shows the follow may come 5 s after the arm
    rig = SimLane(following=False, arm_delay=4.9, options={"feed_report_hz": 0.2})
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.run()
    assert len(scale_sets(rig)) == 1
    assert_restored(rig, ff_ticks=600000000)  # the counters back at 5 s


def test_a_follow_that_never_comes_at_a_slow_report_rate_is_refused_after_two_periods():
    rig = SimLane(following=False, arms=False, options={"feed_report_hz": 0.2})
    set_modes(rig.mcu, [0, 0, 0, 0])
    start = rig.now
    with pytest.raises(GcodeError, match="ace2k: lane 1 did not enter the follow"):
        rig.run()
    assert rig.now - start == pytest.approx(2 * 5.0 + cal.ARM_WAIT_MARGIN_S, abs=0.11)
    assert "ace2k_feed_stop" in rig.mcu.sent and rig.gcode.scripts == []


def test_the_lanes_own_full_count_aborts_where_the_switch_sample_saw_nothing():
    rig = SimLane()
    rig.at(rig.now + 30.0, lambda: setattr(rig, "full", 1))
    with pytest.raises(GcodeError, match="aborted — the buffer went full; nothing changed"):
        rig.run()
    assert rig.unit.pulled_any is False
    assert len(extrusions(rig)) == 5 and scale_sets(rig) == []
    assert_restored(rig)


def test_fulls_counted_before_the_run_are_not_the_runs():
    # a lane in the follow since a print, with fulls on its count: the run counts from its first
    # counters frame
    rig = SimLane()
    rig.full = 3
    rig.run()
    assert len(scale_sets(rig)) == 1


def test_a_full_count_from_before_the_arm_is_not_a_full_of_the_run():
    # the lane's last session left 5 fulls; the run's arm restarts the count at 0, but no counters
    # frame came in the arm's wait, so the status still shows 5 when the run starts listening
    rig = SimLane(following=False, counts_before_run=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.full = 5
    rig.send_counts(rig.now)
    rig.run()
    assert len(scale_sets(rig)) == 1


def test_a_full_in_an_armed_run_aborts_whatever_the_count_before_the_arm():
    rig = SimLane(following=False, counts_before_run=False)
    set_modes(rig.mcu, [0, 0, 0, 0])
    rig.full = 5
    rig.send_counts(rig.now)
    rig.at(rig.now + 30.0, lambda: setattr(rig, "full", 1))
    with pytest.raises(GcodeError, match="aborted — the buffer went full; nothing changed"):
        rig.run()
    assert scale_sets(rig) == []


def test_a_re_entry_that_restarts_the_full_count_is_no_full():
    # the counts restart at every entry into a mode: a lower count in a new epoch is the re-entry
    rig = SimLane()
    rig.full = 3

    def re_enter():
        rig.epoch, rig.taut, rig.full = 9, 0, 0

    rig.at(rig.now + 30.0, re_enter)
    with pytest.raises(GcodeError, match="aborted — lane 1 re-entered a mode; nothing changed"):
        rig.run()


class LandsOnDrain(collections.deque):
    """The run's frames, with one more landed by the serial thread the first time a poll looks
    for one to drain: after the poll's checks, before its drain."""

    def __init__(self, frames, land):
        super().__init__(frames)
        self.land = land

    def __bool__(self):
        land, self.land = self.land, None
        if land is not None:
            land()
        return len(self) > 0


@pytest.mark.parametrize("taut", [0, 1], ids=["a-full", "a-full-and-a-taut"])
def test_a_full_landing_between_the_last_polls_checks_and_its_drain_still_aborts(taut):
    # the extrusion is over and the last poll found no full; a frame with one lands before the
    # drain, which takes it with the rest — with a taut in it, as a mark.  Its full is looked for
    # again before the verdict, which would otherwise apply the scale (or, the mark off the line,
    # refuse it for its scatter)
    rig = SimLane()
    toolhead = rig.printer.lookup_object("toolhead")
    wait_moves = toolhead.wait_moves
    queues, looked_up = [], []

    def land_a_full():
        rig.taut += taut
        rig.full = 1
        rig.send_counts(rig.now)
        looked_up.append(len(rig.extruder.lookups))

    def wait_then_race():
        wait_moves()  # the next poll is the last
        sink = rig.feed.cal._sink
        sink.frames = LandsOnDrain(sink.frames, land_a_full)
        queues.append(sink.frames)

    toolhead.wait_moves = wait_then_race
    with pytest.raises(GcodeError, match="aborted — the buffer went full; nothing changed"):
        rig.run()
    (queue,), (before,) = queues, looked_up
    assert len(queue) == 0  # it landed, and the drain took it...
    assert len(rig.extruder.lookups) == before + taut  # ...as a mark when it carried a taut
    assert rig.unit.pulled_any is False and len(extrusions(rig)) == 20
    assert scale_sets(rig) == [] and rig.printer.objects["configfile"].calls == []
    assert_restored(rig)


def test_a_frames_full_is_noted_before_the_frame_is_queued():
    # the other side of the same window: once queued, a frame may be drained at any moment
    rig = SimLane()
    rig.feed.cal.lane = 0  # a run holding lane 1, as it listens
    sink = rig.feed.cal._sink
    noted = []

    class Noting(collections.deque):
        def append(self, frame):
            noted.append(sink.full_seen)
            super().append(frame)

    sink.frames = Noting()
    rig.send_counts(rig.now)  # the run's first frame: its baseline
    rig.full = 1
    rig.send_counts(rig.now)
    assert noted == [False, True] and len(sink.frames) == 2
