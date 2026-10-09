"""ace2k_ff — the feed-forward on the host: the lane meters what its extruder is about to use.

The unit cannot run a lane continuously at a print's rate (≈ 1–6 mm/s, under the motor's floor),
so it meters the filament out in small doses (``ace2k_feed_base``, taken only by a lane in
``following``).  This module tells it the rate: Klipper's toolhead hands each extruding move to
the active extruder as it flushes it from its look-ahead, ahead of real time — a wrapper on that
extruder's ``process_move`` (newer Klipper) or ``move`` (older trees, the U1's among them)
records ``(print_time, duration, extruder distance)``, and every ``ff_period_ms`` the signed
distance the recorded moves cover in ``[now + ff_lead_ms, now + ff_lead_ms + ff_window_ms]`` (print
time), times the print time since the extruder's last tick over the window, is the tick's share:
the window slides by that much, so the shares add up to the extruder's net E (a tick the reactor
ran late sweeps the time missed in steps of a period, each its own window).  A negative share — a
retraction — is credited to the extruder (up to ``CREDIT_MAX_MM``) and the tick's rate is 0; a
positive one pays the credit back first, off the window's mean.  A retraction is thus withheld
from the next extrusion, not lost: a floor at 0 would over-feed by its length at every travel.
The credit is dropped when the extruder has no lane in ``following``, two of them, its lane in
follow is switched off, held or remapped, and at a shutdown; an idle lane of it changes nothing.
The rate goes to the one lane of that extruder in ``following``; none, nothing is sent; two or
more, no rate, each lane told once to drop what it owes, and a console line — which of them the
filament comes from is not this module's to guess.  A rate is sent when it differs from the last
by more than ``ff_deadband``, rises from 0 or reaches 0; a 0 stops the accrual, and what the lane
owes is still delivered.  A lane remapped while it follows is told to drop what it owes: that
filament was the old extruder's.  An automatic encoder calibration (``ace2k_encoder_cal.py``) holds
its lane for the run: no rate and no metering, what the lane owes dropped at the hold, the user's
switch untouched.

The map is ``[ace2k] laneN_extruder`` (several lanes may name one extruder); host modules may set
a lane without a key through ``Ace2kFeed.set_lane_extruder``.  The unit's correction counters
(the follow's taut catch-ups and full take-ups while metering) land in the lanes' status, and every
``ff_notice_every`` corrections in the same direction since the last notice, counted only from
counters whose interval saw the lane metering (its feed-forward on, a rate above 0 in force),
print a console line: a lane that keeps correcting one way meters a wrong length — its encoder
scale is off.

    [ace2k]
    #lane1_extruder: extruder   # the extruder lane 1 feeds (lane1..lane4); none, no feed-forward
    #feed_forward: True         # ACE_FEED_FORWARD ON=1 / OFF=1 switches it live
    #ff_period_ms: 100          # how often the rate is worked out (20–1000)
    #ff_window_ms: 500          # the stretch of planned moves it is the mean of (100–5000)
    #ff_lead_ms: 0              # how far ahead of now the window starts (0–5000)
    #ff_deadband: 0.3           # mm/s: a smaller change is not sent (0–10)
    #ff_notice_every: 20        # corrections one way before a console notice (1–10000)

``ff_chunk_mm`` and ``ff_pulse``, the unit's own values, are ``ace2k_feed.py``'s.  Loaded by it;
an image without the feed-forward never hears from this module.
"""

import collections
import logging
import math
import threading

LANES = 4
COUNT_MASK = 0xFFFF  # the unit's correction counters are uint16
MOVES_MAX = 4096  # recorded moves kept per extruder, should the timer ever stop pruning them
NO_FF = "the unit's firmware has no feed-forward; flash v0.9.0"
BASE_FMT = "ace2k_feed_base lane=%c rate_um_s=%u clear=%c"
# host-side keys: name, default, minval, maxval
PERIOD_MS = ("ff_period_ms", 100, 20, 1000)
WINDOW_MS = ("ff_window_ms", 500, 100, 5000)
LEAD_MS = ("ff_lead_ms", 0, 0, 5000)
DEADBAND = ("ff_deadband", 0.3, 0.0, 10.0)
NOTICE_EVERY = ("ff_notice_every", 20, 1, 10000)
# the most retraction an extruder may have credited, mm: well past any retraction a slicer plans
# (≈ 0.5–6 mm), so it bounds only a runaway — a long negative E — not a print
CREDIT_MAX_MM = 50.0
# the most window steps one tick sweeps after the reactor ran late: 5 s at the default period
SWEEP_STEPS_MAX = 50


def window_total(moves, start_pt, end_pt):
    """Signed extruder distance (mm) the moves command inside [start_pt, end_pt]; each move
    (print_time, duration, e_distance) counts pro rata of its overlap."""
    total = 0.0
    for pt, dur, dist in moves:
        if dur <= 0.0:
            continue
        lo, hi = max(pt, start_pt), min(pt + dur, end_pt)
        if hi > lo:
            total += dist * (hi - lo) / dur
    return total


def window_rate(moves, start_pt, end_pt):
    """Mean extruder speed (mm/s) inside [start_pt, end_pt], floored at 0."""
    return max(0.0, window_total(moves, start_pt, end_pt) / (end_pt - start_pt))


def choose_lane(lanes_of_extruder, modes):
    """(lane, following): following is the extruder's lanes in "following", in order; lane is
    the one of them when there is exactly one, else None (none, or ambiguous)."""
    following = [lane for lane in lanes_of_extruder if modes[lane] == "following"]
    return (following[0] if len(following) == 1 else None), following


def lanes_text(lanes):
    names = [str(lane + 1) for lane in lanes]
    if len(names) == 2:
        return f"lanes {names[0]} and {names[1]} both"
    return f"lanes {', '.join(names[:-1])} and {names[-1]} all"


class FeedForward:
    def __init__(self, printer, feed, config):
        self.printer = printer
        self.feed = feed
        self.reactor = printer.get_reactor()
        self.keys = {}  # lane -> extruder name, from [ace2k]; wins over the runtime map
        for lane in range(LANES):
            name = config.get(f"lane{lane + 1}_extruder", None)
            if name:
                self.keys[lane] = name.strip()
        self.runtime = {}  # lane -> extruder name, from set_lane_extruder
        self.enabled = config.getboolean("feed_forward", True)
        self.on = [self.enabled] * LANES
        self.held = set()  # lanes an encoder calibration holds (hold): no rate, no metering
        self.period_ms = self._read(config, PERIOD_MS, config.getint)
        self.window_ms = self._read(config, WINDOW_MS, config.getint)
        self.lead_ms = self._read(config, LEAD_MS, config.getint)
        self.deadband = self._read(config, DEADBAND, config.getfloat)
        self.notice_every = self._read(config, NOTICE_EVERY, config.getint)
        self.moves = {}  # extruder name -> deque of (print_time, duration, e_distance)
        self.credit = {}  # extruder name -> mm retracted, withheld from its next extrusion
        self.swept = {}  # extruder name -> the window start its last share swept to (print time)
        self.rates = [0.0] * LANES  # mm/s last sent, 0 once the lane left the follow
        # the lane's start sequence when the rate went out: a start since (a new arm, between
        # two 1 Hz state reports) has the unit at 0 again
        self.sent_seq = [None] * LANES
        self.ambiguous = set()  # extruders whose current episode of two lanes was said
        self.record_failed = False
        self.clock = None  # the toolhead's MCU: print time
        self.timer = None
        self.ready = False
        # the notices: the unit's last counters per lane, and the net count since the last one
        self.last_counts = [(0, 0)] * LANES
        self.epochs = [None] * LANES  # the unit's count the last counters belong to
        self.net = [0] * LANES
        # the lane metered at some moment since its last counters: set by the tick and by a send
        # while a rate above 0 is in force, consumed by the next counters
        self.metered = [False] * LANES
        self.metered_lock = threading.Lock()  # the serial thread's swap against the tick's mark
        printer.register_event_handler("klippy:ready", self._on_ready)

    @staticmethod
    def _read(config, spec, read):
        key, default, low, high = spec
        return read(key, default, minval=low, maxval=high)

    # --- the map -------------------------------------------------------------------------

    def extruder_of(self, lane):
        return self.keys.get(lane) or self.runtime.get(lane)

    def set_lane_extruder(self, lane_index, extruder_name):
        """Map lane_index (0..3) to extruder_name (None: unmapped); a laneN_extruder key wins.
        After ready the extruder is hooked at once."""
        if not 0 <= lane_index < LANES:
            raise ValueError(f"ace2k: no lane {lane_index}")
        before = self.extruder_of(lane_index)
        if extruder_name:
            self.runtime[lane_index] = extruder_name
        else:
            self.runtime.pop(lane_index, None)
        if lane_index in self.keys:
            logging.info(
                "ace2k: lane %d keeps laneN_extruder %s (asked %s)",
                lane_index + 1,
                self.keys[lane_index],
                extruder_name,
            )
            return
        after = self.extruder_of(lane_index)
        if after != before:
            # a retraction credited is the filament of the extruder's follower: dropped when that
            # is the lane remapped, or when none is left in follow
            following = self.feed.lanes[lane_index]["mode"] == "following"
            for name in (before, after):
                if name and (following or not self._followers(name)):
                    self._drop(name)
            if self._may_owe(lane_index):
                # the old extruder's rate is not this one's, nor what the lane owes it: cleared
                self._send(lane_index, 0.0, clear=True)
        if self.ready and extruder_name:
            self._hook_lane(lane_index)

    # --- the planner hook ----------------------------------------------------------------

    def _on_ready(self):
        self.ready = True
        if not self.feed.present:
            return
        if not self.feed.has_ff:
            if self.enabled:
                self._say(f"ace2k: {NO_FF}")
            return
        for lane in range(LANES):
            if self.extruder_of(lane):
                self._hook_lane(lane)

    def _hook_lane(self, lane):
        """Hook the lane's extruder (once per extruder); a console line when it cannot be."""
        if not self.feed.has_ff:
            return
        name = self.extruder_of(lane)
        if name in self.moves:
            return
        try:
            obj = self.printer.lookup_object(name)
        except Exception:  # Klipper's config error for a missing object; KeyError in the tests
            obj = None
        if obj is None:
            self._say(f"ace2k: lane {lane + 1}: no extruder {name} — no feed-forward on it")
            return
        attr = next((a for a in ("process_move", "move") if callable(getattr(obj, a, None))), None)
        if attr is None:
            self._say(
                f"ace2k: lane {lane + 1}: {name} has neither process_move nor move — no"
                " feed-forward on it"
            )
            return
        if self.clock is None:
            self.clock = self.printer.lookup_object("mcu")
        moves = self.moves[name] = collections.deque(maxlen=MOVES_MAX)
        original = getattr(obj, attr)

        def wrapper(print_time, move, *args, **kwargs):
            # the original first: what it raises is Klipper's; the record never raises
            result = original(print_time, move, *args, **kwargs)
            try:
                e_distance = move.axes_d[3]
                if e_distance:
                    duration = move.accel_t + move.cruise_t + move.decel_t
                    moves.append((print_time, duration, e_distance))
            except Exception:
                if not self.record_failed:
                    self.record_failed = True
                    logging.exception(
                        "ace2k: feed-forward: a planned move of %s could not be recorded", name
                    )
            return result

        setattr(obj, attr, wrapper)
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._tick, self.reactor.monotonic())

    # --- the rate ------------------------------------------------------------------------

    def _tick(self, eventtime):
        if not self.feed.has_ff:
            return self.reactor.NEVER
        if self.printer.is_shutdown():
            # Klipper, or the unit, shut down: the modes are stale and nothing may be sent
            self.credit.clear()
            self.swept.clear()
            return self.reactor.NEVER
        try:
            self._update(eventtime)
        except Exception:
            logging.exception("ace2k: feed-forward: the rate update failed")
        return eventtime + self.period_ms / 1000.0

    def _update(self, eventtime):
        modes = [lane["mode"] for lane in self.feed.lanes]
        for lane in range(LANES):
            self._mark(lane)  # the rate held since the last tick
            if modes[lane] != "following" or self.feed.seq[lane] != self.sent_seq[lane]:
                self.rates[lane] = 0.0  # the unit drops the rate outside the follow, and arms at 0
        pt_now = self.clock.estimated_print_time(eventtime)
        start = pt_now + self.lead_ms / 1000.0
        for name, moves in self.moves.items():
            self._update_extruder(name, moves, modes, start)
            # recorded in planning order: the toolhead's print time only moves forward and each
            # move starts where the last ended, so the ends are ordered and the past is a prefix;
            # the next share sweeps from this start on, never before now
            while moves and moves[0][0] + moves[0][1] < pt_now:
                moves.popleft()

    def _update_extruder(self, name, moves, modes, start):
        lanes = [lane for lane in range(LANES) if self.extruder_of(lane) == name]
        lane, following = choose_lane(lanes, modes)
        if len(following) > 1:
            self._drop(name)
            # which lane the filament comes from is not known: what each owes is dropped, once
            # at the episode's start, and again for a lane that had a rate since
            for other in following:
                if self.rates[other] or name not in self.ambiguous:
                    self._send(other, 0.0, clear=True)
            if name not in self.ambiguous:
                self.ambiguous.add(name)
                self._say(
                    f"ace2k: extruder {name}: {lanes_text(following)} in follow — no feed-forward"
                )
            return
        self.ambiguous.discard(name)
        if lane is None or not self.on[lane] or lane in self.held:
            self._drop(name)
            return
        rate = self._spend(name, moves, start)
        if rate is None:
            return
        if rate < 0.0005:
            rate = 0.0  # what the unit would get, in µm/s: a rate that rounds to 0 is 0
        last = self.rates[lane]
        # past the deadband; or a start or a stop, whatever its size: a slow print's rate under
        # the deadband is still the lane's, and the credit is spent as if it were in force
        if abs(rate - last) > self.deadband or (rate == 0.0) != (last == 0.0):
            self._send(lane, rate)

    def _spend(self, name, moves, start):
        """The tick's rate (mm/s).  Its share is the window's total times the print time swept
        since the extruder's last tick, over the window (the first tick sweeps one period); a
        sweep longer than a period — the reactor late — is cut into steps of at most a period,
        each its own window, so every planned moment weighs once whatever the ticks do.  A
        negative share is credited and the rate is 0; a positive one pays the credit back first.
        None when no print time has passed."""
        window = self.window_ms / 1000.0
        period = self.period_ms / 1000.0
        last = self.swept.get(name, start - period)
        if start <= last:
            return None
        self.swept[name] = start
        elapsed = start - last
        steps = min(SWEEP_STEPS_MAX, max(1, math.ceil(elapsed / period - 1e-9)))
        step = elapsed / steps
        now = window_total(moves, start, start + window)
        amount = now * step / window
        for k in range(1, steps):
            at = last + k * step
            amount += window_total(moves, at, at + window) * step / window
        credit = self.credit.get(name, 0.0)
        if amount < 0.0:
            self.credit[name] = min(CREDIT_MAX_MM, credit - amount)
            return 0.0
        use = min(credit, amount)
        self.credit[name] = credit - use
        # the rate is the window's mean now, as ever, less the credit paid back
        return max(0.0, now / window - use / elapsed)

    def _followers(self, name):
        return [
            lane
            for lane in range(LANES)
            if self.extruder_of(lane) == name and self.feed.lanes[lane]["mode"] == "following"
        ]

    def _drop(self, name):
        """Forget the extruder's credit and sweep: its next tick starts afresh."""
        self.credit.pop(name, None)
        self.swept.pop(name, None)

    def set_lead(self, lead_ms):
        """ACE_FF_SET LEAD_MS: a new lead moves every window's start, so each extruder's sweep
        and credit are dropped and its next tick starts afresh.  Kept, a lower lead would leave
        the sweep ahead of the new start, and no rate — not even 0 — would go out until print
        time caught up with it.  (A new window needs nothing: the sweep is of window starts.)"""
        if lead_ms != self.lead_ms:
            self.swept.clear()
            self.credit.clear()
        self.lead_ms = lead_ms

    def _may_owe(self, lane):
        """The unit may hold a debt on the lane: a rate in force, or the lane in follow (a 0
        stops the accrual, and what is owed is still paid).  Never after a Klipper or unit
        shutdown: the modes are stale, and nothing may be sent."""
        if self.printer.is_shutdown():
            return False
        if self.rates[lane]:
            return True
        return bool(self.feed.has_ff and self.feed.lanes[lane]["mode"] == "following")

    def _send(self, lane, rate, clear=False):
        """ace2k_feed_base, clamped to the lane's top speed (the unit shuts down above it).  A 0
        stops the accrual only: what the lane owes is filament the head has already pulled, and is
        still paid.  clear drops it too — for a lane that no longer feeds its extruder alone."""
        um = min(int(round(rate * 1000.0)), int(round(self.feed.speed_max * 1000.0)))
        self._mark(lane)  # the rate it replaces was in force until now
        self.feed.mcu.lookup_command(BASE_FMT).send([lane, um, 1 if clear else 0])
        self.rates[lane] = um / 1000.0
        self.sent_seq[lane] = self.feed.seq[lane]

    def _metering(self, lane):
        return bool(
            self.on[lane] and self.feed.has_ff and self.rates[lane] > 0.0 and lane not in self.held
        )

    def _mark(self, lane):
        if self._metering(lane):
            with self.metered_lock:
                self.metered[lane] = True

    # --- the switch ----------------------------------------------------------------------

    def set_on(self, lanes, on):
        """ACE_FEED_FORWARD: off sends 0 once to a lane with a rate, then no rate until on.  A
        lane off may still be sent a clear (one of two lanes of an extruder in follow, or
        remapped): a clear moves nothing, so it is not a feed."""
        for lane in lanes:
            self.on[lane] = on
            name = self.extruder_of(lane)
            if not on and name:
                # the credit is the extruder's follower's: dropped if that is this lane, or if
                # no lane of it is left in follow
                followers = self._followers(name)
                if lane in followers or not followers:
                    self._drop(name)
            if not on and self.rates[lane]:
                self._send(lane, 0.0)

    def hold(self, lane, held):
        """An automatic calibration holds the lane's feed-forward: no rate and no metering while
        held, the user's switch untouched.  At the hold, what the lane owes is dropped (a dose
        would move the filament between the run's marks), and the extruder's credit and sweep with
        it when the lane is its follower or none is left."""
        if not held:
            self.held.discard(lane)
            return
        self.held.add(lane)
        name = self.extruder_of(lane)
        if name:
            followers = self._followers(name)
            if lane in followers or not followers:
                self._drop(name)
        if self._may_owe(lane):
            self._send(lane, 0.0, clear=True)

    # --- the notices (the serial thread) -------------------------------------------------

    def on_counts(self, taut, full, epoch):
        """The unit's taut and full corrections per lane since its last entry (uint16), with
        that entry's epoch (uint8), which the unit steps at every entry — a host start's or its
        own.  A new epoch in the frame is a new count, all of it new; within one, the counters
        only grow, a wrap counted across it.  The session's first frame is the baseline, counted
        from: a Klipper restart finds the unit's totals of before it, which are not this
        session's.  A frame adds to the tally only if the lane metered — its feed-forward on and
        a rate above 0 in force — at some moment since the previous frame: the plain follow
        corrects on every taut, which says nothing of the scale.  A frame that does not count
        still moves the baseline, so its corrections are never credited later."""
        for lane in range(LANES):
            if self.epochs[lane] is None:
                self.epochs[lane] = epoch[lane]
                self.last_counts[lane] = (taut[lane], full[lane])
                with self.metered_lock:
                    self.metered[lane] = self._metering(lane)
                continue
            prev_taut, prev_full = self.last_counts[lane]
            rearmed = epoch[lane] != self.epochs[lane]
            self.epochs[lane] = epoch[lane]
            if rearmed:
                d_taut, d_full = taut[lane], full[lane]
            else:
                d_taut = (taut[lane] - prev_taut) & COUNT_MASK
                d_full = (full[lane] - prev_full) & COUNT_MASK
            self.last_counts[lane] = (taut[lane], full[lane])
            with self.metered_lock:
                metered = self.metered[lane]
                self.metered[lane] = self._metering(lane)  # a lane still metering stays set
            if not metered:
                continue
            self.net[lane] += d_taut - d_full
            if abs(self.net[lane]) >= self.notice_every:
                which = "taut" if self.net[lane] > 0 else "full"
                count = abs(self.net[lane])
                self.net[lane] = 0
                line = (
                    f"ace2k: lane {lane + 1}: {count} {which} corrections since the last notice"
                    " — check the encoder scale (ACE_CALIBRATE_ENCODER)"
                )
                logging.info("%s", line)
                self.reactor.register_async_callback(lambda eventtime, line=line: self._say(line))

    # --- status --------------------------------------------------------------------------

    def lane_status(self, lane):
        return dict(
            ff_rate=self.rates[lane],
            ff_on=bool(self.on[lane] and self.feed.has_ff),
            ff_extruder=self.extruder_of(lane),
        )

    def settings_text(self):
        return (
            f"window {self.window_ms} ms, lead {self.lead_ms} ms, deadband {self.deadband:g} mm/s"
        )

    def switch_line(self):
        states = " ".join(
            f"lane {lane + 1} {'on' if self.on[lane] else 'off'}"
            f" ({self.extruder_of(lane) or 'unmapped'})"
            for lane in range(LANES)
        )
        return f"ace2k: feed-forward {states}"

    def _say(self, line):
        self.printer.lookup_object("gcode").respond_info(line)
