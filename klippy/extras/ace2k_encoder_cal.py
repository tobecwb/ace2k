"""The automatic encoder calibration: a lane's encoder scale
read off the lane's extruder.

The follow feeds the filament; the extruder pulls the plunger from rest back to taut at the
extrusion's speed, so a taut is a sharp, repeatable plunger position, and between two of them
the filament through the encoder is exactly what the extruder took.  A mark is an increment of the
unit's taut count (ace2k_feed_ff_state) — the switch itself closes for less than a report — at
which the encoder, still since the burst before, and the extruder's position are paired; a
least-squares line through the marks gives the scale: the scale in use over its slope.

``ACE_CALIBRATE_ENCODER LANE=n AUTO=1 [LENGTH=500] [SPEED=4]`` runs it.  Refused, each with its
reason and before anything is sent: an image without the feed-forward, a print printing or
paused, the lane without an extruder, that extruder not the active one or too cold to extrude,
the lane without filament, in error or busy in a mode other than idle and the follow, a run already
going.  The run arms the follow if the lane is idle (at assist_speed, as ACE_ASSIST DIR=BOTH),
holds the lane's feed-forward (the follow alone feeds), raises both reports to 10 Hz, and
extrudes LENGTH in 25 mm chunks under SAVE_GCODE_STATE / RESTORE_GCODE_STATE — only the extruder
moves — each chunk queued once the last has run, so an abort (a full: the shared switch, or the
lane's own full count rising; the lane in error or out of the follow; an epoch change; a
shutdown) lets the running chunk end and queues no other.  Every ending releases the hold, puts
the reports back at their own rates, stops the follow only if the run armed it and restores the
G-code state — after a shutdown, nothing is sent and no G-code run: Klipper refuses it then, and
the state does not outlive the restart.  Then the marks are judged, and the scale applied — in
use at once, staged for SAVE_CONFIG — or refused, with "nothing changed"."""

import collections
import math

COUNT_MASK = 0xFFFF  # the unit's taut count is uint16
MIN_MARKS = 10  # fewer, and the line rests on too little travel to defend 0.1 %
MAX_SCATTER_MM = 1.0  # RMS off the line above this: the filament slipped somewhere
# a larger change is applied, with a warning: an extruder off its own calibration
WARN_CHANGE = 0.03
CHUNK_MM = 25.0  # the extrusion's step: an abort stops at the end of the running one
LENGTH_MM = (500.0, 200.0, 2000.0)  # default, least, most
SPEED_MM_S = 4.0
REPORT_S = 0.1  # the feed-forward counters and the lane counters at 10 Hz for the run
# the lane counters' period between runs: ace2k.LANE_REPORT_S, which this module cannot import
# (ace2k loads ace2k_feed, which loads this one); a test holds the two equal
LANE_REPORT_S = 1.0
POLL_S = 0.25  # the frames turned into marks this often: the extruder's history is short
# The follow's arm is seen in the state report within this at least, or within two of the
# report's periods and ARM_WAIT_MARGIN_S at a slower feed_report_hz: one report may be lost
ARM_WAIT_S = 3.0
ARM_WAIT_MARGIN_S = 0.5
GCODE_STATE = "ACE2K_ENCODER_CAL"  # the SAVE_GCODE_STATE name around the run
SHUTDOWN = "Klipper shut down"  # the abort's reason
FULL = "the buffer went full"  # the abort's reason


class EpochChanged(Exception):  # noqa: N818 — a condition of the run, not an error class
    """The lane re-entered a mode during the run: its counters restarted."""


class MarkTracker:
    """The run's frames, in arrival order, turned into marks.  reading(pt, encoder_mm): a lane
    counters reading; counts(pt, taut, epoch): a counters frame with the lane's taut count and
    epoch — returns the mark the frame closes, (instant, encoder_mm), or None."""

    def __init__(self):
        self.readings = []  # (pt, encoder_mm) since the frame the last episode was seen in
        self.prev = None  # (pt, taut, epoch) of the last counters frame
        # the frame the last episode was seen in: older readings may predate its burst
        self.after = None
        self.episodes = 0  # taut episodes seen in the run
        self.dropped = 0  # marks dropped: two increments in one interval, or no still reading

    def reading(self, pt, encoder_mm):
        self.readings.append((pt, encoder_mm))

    def counts(self, pt, taut, epoch):
        prev, self.prev = self.prev, (pt, taut, epoch)
        if prev is None:
            return None  # the baseline
        if epoch != prev[2]:
            raise EpochChanged()
        step = (taut - prev[1]) & COUNT_MASK
        if not step:
            return None
        lo, after = prev[0], self.after
        self.episodes += step
        encoder = self._still_before(lo, after)
        self.after = pt
        self.readings = [r for r in self.readings if r[0] > pt]
        if self.episodes == step:
            self.dropped += step - 1  # any others in its interval are marks dropped
            return None  # the run's first episode: where the plunger started is unknown
        if step > 1 or encoder is None:
            self.dropped += step
            return None
        return ((lo + pt) / 2.0, encoder)

    def _still_before(self, lo, after):
        """The last reading before lo, equal to the one before it, both after `after`."""
        before = [r for r in self.readings if r[0] < lo and (after is None or r[0] > after)]
        if len(before) < 2:
            return None
        (_, a), (_, b) = before[-2], before[-1]
        return b if a == b else None


def fit_line(points):
    """Least squares of y on x over [(x, y)]: (slope, intercept, RMS of the residuals).  Needs at
    least two points with distinct x: with every x equal there is no line, and the slope is 0."""
    n = len(points)
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    sxx = sum((x - mx) ** 2 for x, _ in points)
    if sxx <= 0.0:
        return 0.0, my, 0.0
    slope = sum((x - mx) * (y - my) for x, y in points) / sxx
    icpt = my - slope * mx
    rms = math.sqrt(sum((y - (icpt + slope * x)) ** 2 for x, y in points) / n)
    return slope, icpt, rms


def judge(marks, scale, low, high, dropped=None):
    """The verdict on a run's marks [(extruder_mm, encoder_mm)] for a lane at `scale` (mm per
    count), the unit's window low..high: a dict — n, refusal (None when it may be applied) and
    warn; with enough marks for a line also scatter, span_e and span_enc; and new and change
    unless the slope is 0 or below (refused: the encoder did not follow).  Each check is made on
    the value as it is shown and sent, so no line can contradict the verdict: new to 4 decimals
    (staged so, sent in 0.1 µm/count), the change from that new and to 0.01 %, the scatter to
    0.01 mm.  dropped, when given (MarkTracker.dropped), is named in the too-few refusal: many
    taut episodes with their marks dropped point at the encoder, not at the extruder."""
    n = len(marks)
    verdict = dict(n=n, refusal=None, warn=False)
    if n < MIN_MARKS:
        noun = "mark" if n == 1 else "marks"
        lost = "" if dropped is None else f" ({dropped} dropped)"
        verdict["refusal"] = (
            f"{n} {noun}, at least {MIN_MARKS} needed{lost} — is the extruder gripping the"
            " filament? a larger LENGTH gives more"
        )
        return verdict
    slope, _icpt, rms = fit_line(marks)
    verdict.update(
        scatter=rms,
        span_e=marks[-1][0] - marks[0][0],
        span_enc=marks[-1][1] - marks[0][1],
    )
    if slope <= 0.0:
        verdict["refusal"] = "the encoder did not follow the extruder"
        return verdict
    new = round(scale / slope, 4)  # the scale as staged and sent
    verdict.update(new=new, change=new / scale - 1.0)
    shown = round(rms, 2)  # a refusal never reads 1.00 mm
    if shown > MAX_SCATTER_MM:
        verdict["refusal"] = (
            f"scatter {shown:.2f} mm, more than {MAX_SCATTER_MM:g} mm: the filament slipped"
        )
    elif not low <= new <= high:
        verdict["refusal"] = (
            f"{new:.4f} mm/count is outside the unit's window {low:.4f}..{high:.4f}"
        )
    else:  # a change shown as +3.00 % never carries the warning
        verdict["warn"] = abs(round(verdict["change"] * 100.0, 2)) > WARN_CHANGE * 100.0
    return verdict


class _Sink:
    """What the serial thread collects for one run: the lane, the run's frames, and the lane's
    full count — the (epoch, count) of the run's first counters frame, and whether a later frame
    of that epoch carried another: a full the follow took up during the run.  The baseline is the
    run's first frame, not the status at its start: a run that armed the follow restarted the
    unit's counters, and the status may still show the session before."""

    __slots__ = ("lane", "frames", "full", "full_seen")

    def __init__(self, lane, frames):
        self.lane = lane
        self.frames = frames
        self.full = None
        self.full_seen = False

    def note_full(self, epoch, count):
        """A counters frame's full count.  Within the baseline's epoch only: a count that a new
        epoch restarted is the re-entry the tracker aborts on, not a full."""
        if self.full is None:
            self.full = (epoch, count)
        elif epoch == self.full[0] and count != self.full[1]:
            self.full_seen = True


class EncoderCalibration:
    """ACE_CALIBRATE_ENCODER AUTO=1.  The frames reach it on the serial thread while a run
    holds a lane; run() drives the run on the reactor."""

    def __init__(self, printer, feed):
        self.printer = printer
        self.feed = feed
        self.reactor = printer.get_reactor()
        self.frames = collections.deque()  # ("counts", rt, taut, epoch) / ("reading", rt, mm)
        # the run's _Sink while a run holds a lane, None between runs: one attribute, so the
        # serial thread reads the lane and its run's frames in one go.  Each run collects into a
        # sink and a deque of its own, so a frame read under one run lands in that run's, never
        # in the next one's.
        self._sink = None

    @property
    def lane(self):
        """The lane a run holds; None between runs."""
        sink = self._sink
        return None if sink is None else sink.lane

    @lane.setter
    def lane(self, lane):
        self._sink = None if lane is None else _Sink(lane, self.frames)

    def on_ff_state(self, receive_time, taut, epoch):
        sink = self._sink
        if sink is not None:
            lane = sink.lane
            # the lane's full count of this same frame (ace2k_feed stores the counters first),
            # noted before the frame is queued: once queued, a poll may drain it at any moment
            sink.note_full(epoch[lane], self.feed.lanes[lane]["ff_full_fixes"])
            sink.frames.append(("counts", receive_time, taut[lane], epoch[lane]))

    def on_counters(self, receive_time, encoder_um):
        sink = self._sink
        if sink is not None:
            sink.frames.append(("reading", receive_time, encoder_um[sink.lane] / 1000.0))

    # --- the run (the reactor) -----------------------------------------------------------

    def run(self, gcmd, index):
        """ACE_CALIBRATE_ENCODER LANE=n AUTO=1 [LENGTH=] [SPEED=] on lane index (0..3)."""
        default, least, most = LENGTH_MM
        length = gcmd.get_float("LENGTH", default, minval=least, maxval=most)
        speed = gcmd.get_float("SPEED", SPEED_MM_S, above=0.0)
        extruder = self._check(gcmd, index)
        n = index + 1
        scale = self.feed.lanes[index]["encoder_scale"]  # the readings' scale, the whole run
        gcmd.respond_info(
            f"ace2k: lane {n}: calibrating against {extruder.get_name()} — {length:g} mm at"
            f" {speed:g} mm/s"
        )
        tracker = MarkTracker()
        marks = []
        reason = None
        armed = held = listening = False
        try:
            if self.feed.lanes[index]["mode"] != "following":
                self._arm(gcmd, index)
                armed = True
                self._wait_following(gcmd, index)
            if self.printer.is_shutdown():
                reason = SHUTDOWN  # during the arm's wait: nothing more goes out
            else:
                held = True
                self.feed.ff.hold(index, True)
                listening = True
                sink = self._listen(index)
                reason = self._extrude(sink, extruder, length, speed, tracker, marks)
        finally:
            # every ending, before any verdict — a scale goes out with the reports back at their
            # own rates and the follow as the user had it; nothing is sent after a shutdown
            if listening:
                self._unlisten()
            if held:
                self.feed.ff.hold(index, False)
            if armed and not self.printer.is_shutdown():
                self.feed.stop(index)
        if reason is not None:
            raise gcmd.error(f"ace2k: lane {n}: aborted — {reason}; nothing changed")
        v = judge(marks, scale, self.feed.scale_low, self.feed.scale_high, tracker.dropped)
        if "span_e" in v:
            gcmd.respond_info(
                f"ace2k: lane {n}: {v['n']} marks over {v['span_e']:.1f} mm of extruder; the"
                f" encoder read {v['span_enc']:.1f} mm"
            )
        if v["refusal"] is not None:
            raise gcmd.error(f"ace2k: lane {n}: {v['refusal']}; nothing changed")
        self.feed.apply_scale(index, v["new"])
        gcmd.respond_info(
            f"ace2k: lane {n}: scale {scale:.4f} → {v['new']:.4f} mm/count"
            f" ({v['change'] * 100.0:+.2f} %), scatter {v['scatter']:.2f} mm — in use now;"
            " SAVE_CONFIG to keep it"
        )
        if v["warn"]:
            gcmd.respond_info(
                f"ace2k: lane {n}: a change above {WARN_CHANGE * 100.0:g} % — check the"
                " extruder's own calibration too"
            )

    def _check(self, gcmd, index):
        """The extruder to calibrate against, or a G-code error naming what stands in the way —
        before anything is sent."""
        n = index + 1
        if self.lane is not None:
            raise gcmd.error("ace2k: a calibration is already running")
        if not self.feed.has_ff:
            raise gcmd.error(
                "ace2k: the unit's firmware has no feed-forward (its taut count); flash v0.9.0"
            )
        stats = self.printer.lookup_object("print_stats", None)
        if stats is not None:
            state = stats.get_status(self.reactor.monotonic()).get("state")
            if state in ("printing", "paused"):
                raise gcmd.error(f"ace2k: a print is {state}; calibrate between prints")
        name = self.feed.ff.extruder_of(index)
        if not name:
            raise gcmd.error(f"ace2k: lane {n} has no extruder (lane{n}_extruder)")
        extruder = self.printer.lookup_object(name, None)
        if extruder is None or not hasattr(extruder, "find_past_position"):
            raise gcmd.error(f"ace2k: lane {n}: no extruder {name}")
        if self.printer.lookup_object("toolhead").get_extruder() is not extruder:
            raise gcmd.error(f"ace2k: {name} is not the active extruder; select it first")
        if not extruder.get_heater().can_extrude:
            raise gcmd.error(f"ace2k: {name} is too cold to extrude; heat it for the filament")
        lane = self.feed.lanes[index]
        if self.feed.unit.lanes[index]["insert"] is not True:
            raise gcmd.error(f"ace2k: lane {n} has no filament")
        if lane["mode"] == "error":
            raise gcmd.error(f"ace2k: lane {n} is in error ({lane['error']}); ACE_CLEAR first")
        if lane["mode"] not in ("idle", "following"):
            raise gcmd.error(f"ace2k: lane {n} is busy ({lane['mode']})")
        return extruder

    def _arm(self, gcmd, index):
        """The follow on the idle lane, at assist_speed (ACE_ASSIST DIR=BOTH's default); the
        unit's refusal (or a host that cannot start it) a G-code error, as ACE_ASSIST's."""
        try:
            self.feed.start_move(index, "assist_both", 0.0, self.feed.assist_speed)
        except (self.feed.FeedRefused, RuntimeError) as e:
            raise gcmd.error(str(e)) from e

    def _wait_following(self, gcmd, index):
        """The arm seen in the state report within ARM_WAIT_S, or two of the report's periods and
        ARM_WAIT_MARGIN_S if longer, else a G-code error.  A shutdown ends the wait: the run
        aborts on it, and nothing more is sent."""
        period_s = self.feed.report_period_ms / 1000.0
        deadline = self.reactor.monotonic() + max(ARM_WAIT_S, 2 * period_s + ARM_WAIT_MARGIN_S)
        while self.feed.lanes[index]["mode"] != "following":
            if self.printer.is_shutdown():
                return
            now = self.reactor.monotonic()
            if now >= deadline:
                raise gcmd.error(f"ace2k: lane {index + 1} did not enter the follow")
            self.reactor.pause(now + 0.1)

    def _listen(self, index):
        """The lane's frames collected from here on, into a sink and a deque of the run's own,
        and both reports at 10 Hz.  The run's sink."""
        self.frames = collections.deque()
        self.lane = index
        if not self.printer.is_shutdown():
            self.feed.set_counts_period(REPORT_S)
            self.feed.unit.set_counters_period(REPORT_S)
        return self._sink

    def _unlisten(self):
        """No frame collected any more, and both reports back at their own rates."""
        self.lane = None
        if not self.printer.is_shutdown():
            self.feed.set_counts_period(self.feed.report_period_ms / 1000.0)
            self.feed.unit.set_counters_period(LANE_REPORT_S)

    def _extrude(self, sink, extruder, length, speed, tracker, marks):
        """LENGTH in CHUNK_MM chunks under the saved G-code state, each queued once the last has
        run, while the frames become marks.  The abort's reason, or None: a condition seen while
        a chunk runs lets that chunk end and queues no other; one there before the first, none."""
        gcode = self.printer.lookup_object("gcode")
        toolhead = self.printer.lookup_object("toolhead")
        clock = self.printer.lookup_object("mcu")  # the print time

        def poll():
            return self._poll(sink, tracker, marks, extruder, clock)

        gcode.run_script_from_command(f"SAVE_GCODE_STATE NAME={GCODE_STATE}\nM83")
        try:
            done = 0.0
            reason = poll()
            while reason is None and done < length - 1e-6:
                step = min(CHUNK_MM, length - done)
                gcode.run_script_from_command(f"G1 E{step:.3f} F{speed * 60.0:.1f}")
                done += step
                end_pt = toolhead.get_last_move_time()
                while reason is None:
                    now = self.reactor.monotonic()
                    if clock.estimated_print_time(now) >= end_pt:
                        break
                    self.reactor.pause(now + POLL_S)
                    reason = poll()
            toolhead.wait_moves()
            if reason is None:
                self.reactor.pause(self.reactor.monotonic() + 2 * REPORT_S)  # the last frames
                reason = poll()
        finally:
            # not after a shutdown: Klipper refuses G-code then, and its error would replace the
            # abort's own line; the G-code state does not outlive the restart anyway
            if not self.printer.is_shutdown():
                gcode.run_script_from_command(f"RESTORE_GCODE_STATE NAME={GCODE_STATE}")
        return reason

    def _poll(self, sink, tracker, marks, extruder, clock):
        """The abort conditions, then every frame received so far turned into marks — the
        extruder's position looked up as its frame arrives — then the lane's full count again.
        The abort's reason, or None."""
        n = sink.lane + 1
        if self.printer.is_shutdown():
            return SHUTDOWN
        # the switch is a 10 Hz sample, the lane's full count exact: a full taken up within a
        # tick shows only in the count
        if self.feed.unit.pulled_any or sink.full_seen:
            return FULL
        lane = self.feed.lanes[sink.lane]
        if lane["mode"] == "error":
            return f"lane {n} in error ({lane['error']})"
        if lane["mode"] != "following":
            return f"lane {n} left the follow ({lane['mode']})"
        frames = sink.frames
        while frames:
            kind, rt, *data = frames.popleft()
            pt = clock.estimated_print_time(rt)
            if kind == "reading":
                tracker.reading(pt, data[0])
                continue
            try:
                mark = tracker.counts(pt, data[0], data[1])
            except EpochChanged:
                return f"lane {n} re-entered a mode"
            if mark is not None:
                instant, encoder_mm = mark
                marks.append((extruder.find_past_position(instant), encoder_mm))
        # and again: a frame queued after the look above was drained with the rest, and its full
        # was noted before it was queued (on_ff_state), so none drained carries a full unseen —
        # at the last poll, just before the verdict, too
        if sink.full_seen:
            return FULL
        return None
