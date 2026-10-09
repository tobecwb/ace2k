"""ace2k_feed — the filament lanes of the ace2k firmware on the host.

The G-codes a macro needs — ``ACE_FEED``, ``ACE_ROLLBACK``, ``ACE_UNLOAD``, ``ACE_LOAD`` (waiting
by default: the command returns on the move's final event, and an error kind is a G-code error;
``WAIT=0`` returns at once), ``ACE_ASSIST`` (``DIR=BACK``, ``DIR=BOTH`` the follow,
``OFF=1``), ``ACE_STOP``, ``ACE_CLEAR``, ``ACE_SPEED``, ``ACE_CALIBRATE_ENCODER``,
``ACE_LOAD_SET``, ``ACE_SNAG_SET``, ``ACE_FOLLOW_SET`` (``FLIP_MS``, ``TAKE_MM``, ``TAIL_MM``),
``ACE_FF_SET``, ``ACE_FEED_FORWARD`` and ``ACE_GRIP`` (a lane's grip: its next automatic load
pulls only that much, without the tag search)
(``ace2k_ff.py``, the feed-forward: the planner's moves metered to the lane in follow) — and the
tunables of ``[ace2k]`` sent
to the unit: the encoder scales and the supervisors' thresholds as config commands (a change
restarts the MCU, as the insert thresholds do); every lane's encoder scale again, the load
settings, the tip snag's, the follow's and the feed-forward's values as init commands (at every
connect, so a value set live — a scale applied by the automatic calibration too — never outlives
the session), the last four again on every ``ACE_LOAD_SET`` / ``ACE_SNAG_SET`` /
``ACE_FOLLOW_SET`` / ``ACE_FF_SET``.  The ``ace2k_feed_state`` and ``ace2k_feed_ff_state`` reports
and the ``ace2k_feed_event`` events land in the lanes' status; every
event is a log line and the Klipper event ``ace2k:feed_event`` (lane, event) for other modules,
and a ``snag`` notice is printed to the console too.  A runout in the follow is its tail:
the ``tail`` notice, the lane's ``tail`` True in its status until the tail's final event or the lane
leaving ``following`` (a filament pushed in at the bay during the tail changes nothing), and the
final ``tail_out`` — no error.  A push toward the head that meets what cannot move ends ``blocked``,
no error either: the lane idle, the filament waiting where it stopped. Every start carries a
per-lane sequence (1..255) the unit echoes in the events of the move it started, and a waiting
G-code completes only on its own sequence — a late event of an earlier move, or one of a motion the
unit started itself (sequence 0), passes by; until a lane's first start of the session the counter
follows the sequence the state report shows for the lane — the last start issued on it, kept through
idle and error — so a new session continues after an earlier one and never shares a sequence with a
move still running or one that just ended (a start before the session's first report waits for it).
Loaded by ``[ace2k]``; a firmware built without ``CONFIG_ACE2K_FEED`` (no ``ace2k_feed_start`` in
its dictionary) is logged, and every G-code here then answers with an error.  A waiting G-code holds
the G-code queue as Klipper's own ``G4`` does; on a bench, ``WAIT=0`` and the event log are the
tools.  Other host modules start, stop and clear a lane's move, and map a lane to the extruder it
feeds, through the host interface (``API_VERSION``: ``start_move``, ``stop``, ``clear``,
``set_lane_extruder``, ``set_lane_grip``, ``Move``, ``FeedRefused``), callable where a G-code is not
— see ``docs/protocol.md``.

    [ace2k]
    #feed_speed: 30             # mm/s, the default of ACE_FEED and ACE_ROLLBACK
    #unload_speed: 30
    #load_speed: 30
    #load_park_mm: 300          # the load's parking distance: the path from the drive to the outlet
    #                            # (323 mm on the development unit) minus a margin
    #auto_load: True            # pull a filament in to the parking point when it is inserted
    #assist_speed: 50
    #slip_check_mm: 50          # the supervisors' thresholds; an absent key leaves the firmware's
    #slip_allow_mm: 15          # under slip_check_mm, or the partial slip check could never trip
    #stall_check_mm: 20
    #snag_fwd_mm: 20            # the tip snag: motor past a full before the touch (5–50)
    #snag_lag_mm: 5             # the filament behind the motor that is a jam (1–20, under fwd)
    #snag_duty_pct: 0           # the duty above the feed-forward that is a jam (0 off, 0–100)
    #snag_back_mm: 5            # the touch (2–20)
    #snag_rest_ms: 500          # the wait for the plunger at rest (100–2000)
    #snag_free_mm: 50           # reverse: the filament back before a taut trip is forgiven (20–200)
    #follow_flip_ms: 200        # the follow: the wait before a move against the last one (50–2000)
    #follow_take_mm: 15         # the follow's take-up on this lane's full (5–30)
    #follow_tail_mm: 2000       # the follow's tail: motor after a runout before it ends (200–2000)
    #ff_chunk_mm: 3             # the feed-forward's dose (1–10)
    #ff_pulse: 15               # mm/s, a dose's speed (the lane's speed bounds)
    #lane1_encoder_scale: 1.2342   # mm per encoder count, lane1..lane4 (ACE_CALIBRATE_ENCODER)
    #feed_report_hz: 1          # 0.05–10 Hz; 1, 2, 5 and 10 are exact, the rest snap to the grid

The tip snag's values are the firmware's own, settled at the bench.
``snag_duty_pct`` stays 0, the duty guard off: built as a second guard for a hold the filament
slips through, it did not prove effective there — the lag ended every jam first, and a fixed
excess over the feed-forward has no margin (≤ 9 points healthy, 10–14 a jam) — and it is kept for
a possible future case; the rise since the full, which cancels a spool's drag, would be the better
reading (not built).  ``snag_lag_mm: 12`` was tried there and is an option.

With the tag reader (``ace2k_rfid.py``, whose ``rfid_search_mm`` this module reads too) a load
whose tag is not read by the parking point searches on and comes back, and a waited ``ACE_LOAD``
allows for that travel; unset, the reach is the firmware's default.
"""

import logging
import math
import struct

try:
    from . import ace2k_encoder_cal, ace2k_ff
    from .ace2k import probe_format, refuse_format, subscribe_response
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    import ace2k_encoder_cal
    import ace2k_ff
    from ace2k import probe_format, refuse_format, subscribe_response

LANES = 4
LANE_ALL = 255
MODE_NAMES = {
    0: "idle",
    1: "feeding",
    2: "rolling_back",
    3: "unloading",
    4: "loading",
    5: "assisting",
    6: "assisting_back",
    7: "error",
    8: "following",
}
MODE_ERROR = 7
KIND_NAMES = {
    0: "done",
    1: "stopped",
    2: "stopped_link",
    3: "stopped_shutdown",
    4: "runout",
    5: "loaded",
    6: "unloaded",
    7: "stuck",
    8: "tangled",
    9: "motor_stalled",
    10: "timeout",
    11: "assist_stall",  # reserved, never sent: the comparator's tangled answers the case
    12: "assist_overrun",
    13: "unload_incomplete",
    14: "behind",
    15: "snag",
    16: "tail",  # a notice: the insert fell in the follow, which feeds on as the filament's tail
    17: "tail_out",  # final, no error: the tail has left the drive, or the tail's bound reached
    18: "blocked",  # final, no error: a feed or a load's pull met what cannot move; the lane idle
}
ERROR_KINDS = frozenset(range(7, 14))
NOTICE_KINDS = frozenset({14, 15, 16})
KIND_TAIL = 16
CMD_MODES = {
    "feed": 0,
    "rollback": 1,
    "assist": 2,
    "assist_back": 3,
    "unload": 4,
    "load": 5,
    "assist_both": 6,
}
# ACE_ASSIST's DIR: absent or FORWARD the forward assist, BACK the reverse, BOTH the follow
ASSIST_DIRS = {"": "assist", "FORWARD": "assist", "BACK": "assist_back", "BOTH": "assist_both"}
REFUSAL_NAMES = {
    0: "ok",
    1: "busy",
    2: "in_error",
    3: "no_filament",
    4: "no_link",
    5: "bounds",
    6: "other_lane",
}
DEFAULT_SCALE_MM = 1.2342  # the firmware's default: mm per encoder count
SCALE_TOLERANCE = 0.2  # the firmware accepts ±20 % of the default
# The firmware's defaults (docs/protocol.md): sent, all three, only when a key is present
THRESHOLD_DEFAULTS = {
    "slip_check_mm": 50.0,
    "slip_allow_mm": 15.0,
    "stall_check_mm": 20.0,
}
# The tip snag's values: config key, ACE_SNAG_SET argument, the
# multiplier to the unit's unit, and the bounds in the key's unit — the firmware's (feed.h
# ACE2K_FEED_SNAG_*_MIN/_MAX; a host test reads the header).  Sent, all six, as an init command
# at every connect, as the load settings.
SNAG_SETTINGS = (
    ("snag_fwd_mm", "FWD", 1000, 5.0, 50.0),
    ("snag_lag_mm", "LAG", 1000, 1.0, 20.0),
    ("snag_duty_pct", "DUTY", 1, 0, 100),
    ("snag_back_mm", "BACK", 1000, 2.0, 20.0),
    ("snag_rest_ms", "REST", 1, 100, 2000),
    ("snag_free_mm", "FREE", 1000, 20.0, 200.0),
)
# the firmware's defaults, read from its dictionary at connect
SNAG_DEFAULT_CONSTANTS = {
    "snag_fwd_mm": "ACE2K_FEED_SNAG_FWD_UM",
    "snag_lag_mm": "ACE2K_FEED_SNAG_LAG_UM",
    "snag_duty_pct": "ACE2K_FEED_SNAG_DUTY_PCT",
    "snag_back_mm": "ACE2K_FEED_SNAG_BACK_UM",
    "snag_rest_ms": "ACE2K_FEED_SNAG_REST_MS",
    "snag_free_mm": "ACE2K_FEED_SNAG_FREE_UM",
}
SNAG_INT_KEYS = frozenset({"snag_duty_pct", "snag_rest_ms"})
# The follow's values: config key, ACE_FOLLOW_SET argument, the
# multiplier to the unit's unit, and the dictionary constant of the default — the bounds are the
# constants beside it (<name>_MIN, <name>_MAX), read at connect.  Sent, both, as an init command
# at every connect, as the snag's values; an image without the constants — the tail's included
# (the command carries it since v0.11.0) — has no follow.
FOLLOW_SETTINGS = (
    ("follow_flip_ms", "FLIP_MS", 1, "ACE2K_FEED_FOLLOW_FLIP_MS"),
    ("follow_take_mm", "TAKE_MM", 1000, "ACE2K_FEED_FOLLOW_TAKE_UM"),
    ("follow_tail_mm", "TAIL_MM", 1000, "ACE2K_FEED_FOLLOW_TAIL_UM"),
)
FOLLOW_INT_KEYS = frozenset({"follow_flip_ms"})
FOLLOW_FMT = "ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u"
NO_FOLLOW = "the unit's firmware has no follow tail (or no follow); flash v0.11.0"
# The grip: a lane's next automatic load pulls only grip_um, without
# the tag search, and ends loaded.  Its default and bounds are the dictionary's (read at connect,
# with the command); an image without them has no grip — set_lane_grip and ACE_GRIP say so.
GRIP_FMT = "ace2k_feed_lane_grip lane=%c grip_um=%u"
GRIP_CONSTANT = "ACE2K_FEED_GRIP_UM"
NO_GRIP = "the unit's firmware has no grip; flash v0.11.0"
# The feed-forward's unit values, as the follow's: config key,
# ACE_FF_SET argument, the multiplier to the unit's unit, the dictionary constant of the default
# (<name>_MIN, <name>_MAX beside it).  Sent, both, as an init command at every connect; an image
# without the constants has no feed-forward — it is off, said once on the console, never sent to.
FF_SETTINGS = (
    ("ff_chunk_mm", "CHUNK_MM", 1000, "ACE2K_FEED_FF_CHUNK_UM"),
    ("ff_pulse", "PULSE", 1000, "ACE2K_FEED_FF_PULSE_UM_S"),
)
FF_FMT = "ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u"
FF_STATE_FMT = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s"
FF_QUERY_FMT = "ace2k_feed_ff_query rest_ticks=%u"
# ACE_FF_SET's host-side arguments: argument, attribute, (key, default, min, max), integer
FF_HOST_ARGS = (
    ("WINDOW_MS", "window_ms", ace2k_ff.WINDOW_MS, True),
    ("LEAD_MS", "lead_ms", ace2k_ff.LEAD_MS, True),
    ("DEADBAND", "deadband", ace2k_ff.DEADBAND, False),
)
# The unit takes every value whole in its own unit (µm, %, ms): a value is rounded to that
# resolution as it is read, and the bounds, lag < fwd, the value staged and the value sent are all
# judged on the rounded one — the firmware's own checks see exactly what the host checked.
SNAG_FMT = "ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c back_um=%u rest_ms=%u free_um=%u"
WAIT_FACTOR = 1.5  # a waited move's timeout: this × the expected time …
WAIT_MARGIN_S = 5.0  # … plus this
SEQ_MAX = 255  # the sequence wraps past it to 1; 0 is the unit's own (a motion it started itself)
COMMANDED_KEEP = 8  # starts whose length is kept per lane until their event lands
# A start before the session's first state report waits this many report periods for it, plus
# WAIT_MARGIN_S: two, so that a first report lost on the way — to a full transmit buffer, or to
# the half-duplex turnaround — is covered by the next
STATE_WAIT_PERIODS = 2
FEED_REPORT_HZ = 1.0
# The report phases (docs/protocol.md "Reports") keep the unit's periodic reports on different
# ticks only for periods on this grid, so the asked rate is snapped to it
REPORT_GRID_MS = 100
START_FMT = "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c"
START_RESP = "ace2k_feed_start_response lane=%c accepted=%c reason=%c"
STATE_FMT = "ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s"
EVENT_FMT = "ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c"
SCALE_FMT = "ace2k_lane_scale_set lane=%c um_per_count_x10=%u"
SEARCH_FMT = "ace2k_rfid_search_set search_um=%u"
DEFAULT_SEARCH_MM = 750.0  # only for an image that does not declare its own default
# what a waited load, or a MOVE=1 read, allows for the read sessions that pause it: a few of up
# to 0.5 s each, at the load's 30 mm/s
SESSION_ALLOWANCE_MM = 60.0
GCODES = (
    "ACE_FEED",
    "ACE_ROLLBACK",
    "ACE_UNLOAD",
    "ACE_LOAD",
    "ACE_ASSIST",
    "ACE_STOP",
    "ACE_CLEAR",
    "ACE_SPEED",
    "ACE_CALIBRATE_ENCODER",
    "ACE_LOAD_SET",
    "ACE_SNAG_SET",
    "ACE_FOLLOW_SET",
    "ACE_FF_SET",
    "ACE_FEED_FORWARD",
    "ACE_GRIP",
)


# the host interface below; bumped on any change a caller must check for, a call or a mode added
# as much as one changed (2: assist_both; 3: set_lane_extruder; 4: the tail — the kinds tail and
# tail_out, the lane's tail in the status; 5: the kind blocked, and the lane's tail no longer
# ending on the insert rising; 6: set_lane_grip, with has_grip and the grip's bounds)
API_VERSION = 6


class FeedRefused(Exception):  # noqa: N818 — the interface's name (API_VERSION 1)
    """The unit refused a start: .reason is the REFUSAL_NAMES name (busy, in_error, no_filament,
    no_link, bounds, other_lane)."""

    def __init__(self, lane, mode, reason):
        super().__init__(f"ace2k: lane {lane + 1} {mode} refused: {reason}")
        self.lane = lane
        self.mode = mode
        self.reason = reason


class Move:
    """A started move: completed by the final feed event that echoes its sequence."""

    def __init__(self, lane, mode, seq, completion, reactor=None):
        self.lane = lane
        self.mode = mode
        self.seq = seq
        self._completion = completion
        self._reactor = reactor

    def done(self):
        return self._completion.test()

    def result(self):
        """The final event dict once done, else None — never blocks."""
        return self._completion.wait(0.0, None) if self._completion.test() else None

    def wait(self, timeout_s, reactor=None):
        """Block (greenlet) until the final event or timeout_s; the event or None.  reactor
        defaults to the feed's."""
        reactor = reactor if reactor is not None else self._reactor
        return self._completion.wait(reactor.monotonic() + timeout_s, None)


def firmware_search_mm(mcu):
    """The firmware's default reach of the load's search (ACE2K_FEED_SEARCH_DEFAULT_UM), in mm;
    DEFAULT_SEARCH_MM when the image does not declare it."""
    try:
        return mcu.get_constant_float("ACE2K_FEED_SEARCH_DEFAULT_UM") / 1000.0
    except Exception:  # Klipper's msgproto error; KeyError in the tests' fake
        return DEFAULT_SEARCH_MM


def setting_round(key, value, scale):
    """A snag or follow value at the unit's resolution: a millimetre key to the whole µm, a whole
    key as it is; a non-finite one (inf, nan — Klipper's float parser takes both) as it is, for
    setting_bound_error to refuse: round() would raise on it."""
    if key in SNAG_INT_KEYS or key in FOLLOW_INT_KEYS or not math.isfinite(value):
        return value
    return round(value * scale) / scale


def setting_bound_error(key, value, low, high):
    """None when a rounded snag or follow value is within its bounds, else the reason, in the
    key's unit."""
    if not math.isfinite(value):
        return f"({value}) must be a finite number"
    if low <= value <= high:
        return None
    return f"({setting_text(value)}) must be within {setting_text(low)}..{setting_text(high)}"


def setting_text(value):
    """A snag or follow value as printed and staged: exact to the µm (three decimals at most, no
    trailing zeros), so SAVE_CONFIG writes back the value in effect."""
    if isinstance(value, int):
        return str(value)
    return f"{value:.3f}".rstrip("0").rstrip(".")


def to_mm(um):
    return round(um / 1000.0, 3)


def to_um(mm):
    return int(round(mm * 1000.0))


def scale_x10(scale):
    """A scale in mm per count as the unit takes it: in tenths of a µm per count."""
    return int(round(scale * 10000.0))


class Ace2kFeed:
    SESSION_ALLOWANCE_MM = SESSION_ALLOWANCE_MM
    API_VERSION = API_VERSION  # a caller holding only the feed object reads it here
    # and catches a refusal through it — ace2k_encoder_cal.py, which cannot import this module
    FeedRefused = FeedRefused

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        mcu_name = config.get("mcu", "ace2k")
        self.mcu = self.printer.lookup_object("mcu" if mcu_name == "mcu" else "mcu " + mcu_name)
        self.section = config.get_name()
        self.feed_speed = config.getfloat("feed_speed", 30.0, above=0.0)
        self.unload_speed = config.getfloat("unload_speed", 30.0, above=0.0)
        self.load_speed = config.getfloat("load_speed", 30.0, above=0.0)
        self.assist_speed = config.getfloat("assist_speed", 50.0, above=0.0)
        self.load_park_mm = config.getfloat("load_park_mm", 300.0, above=0.0)
        self.auto_load = config.getboolean("auto_load", True)
        # the load's tag search, the key ace2k_rfid.py sends to the unit; unset, the firmware's
        # default (read at connect) — only the ACE_LOAD budget reads it here
        self.search_mm = config.getfloat("rfid_search_mm", None, minval=100.0, maxval=1000.0)
        self.has_search = False
        self.thresholds = {}
        for key in THRESHOLD_DEFAULTS:
            value = config.getfloat(key, None, above=0.0)
            if value is not None:
                self.thresholds[key] = value
        if self.thresholds:
            # the pair as the unit will see it, an absent key at the firmware's default: the
            # partial slip check trips when the filament is short by more than slip_allow_mm over
            # slip_check_mm of motor — an allowance at or above the window can never be exceeded,
            # and the unit refuses the pair as a host bug (a shutdown), so it is refused here first
            th = dict(THRESHOLD_DEFAULTS, **self.thresholds)
            if th["slip_allow_mm"] >= th["slip_check_mm"]:
                raise config.error(
                    f"{self.section}: slip_allow_mm ({th['slip_allow_mm']:g}) must be under"
                    f" slip_check_mm ({th['slip_check_mm']:g}): the partial slip check could never"
                    " trip"
                )
        # the tip snag's keys; the lag against the forward travel is checked at connect, where
        # the firmware's defaults for the absent keys are known
        self.snag_keys = {}
        for key, _arg, scale, low, high in SNAG_SETTINGS:
            read = config.getint if key in SNAG_INT_KEYS else config.getfloat
            value = read(key, None)
            if value is None:
                continue
            value = setting_round(key, value, scale)  # the bounds judged on what the unit gets
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise config.error(f"{self.section}: {key} {reason}")
            self.snag_keys[key] = value
        self.snag = {}  # the values in effect, the firmware's defaults under the keys (at connect)
        # the follow's keys, rounded to the unit's resolution here; their bounds are the
        # dictionary's, so they are judged at connect
        self.follow_keys = {}
        for key, _arg, scale, _name in FOLLOW_SETTINGS:
            read = config.getint if key in FOLLOW_INT_KEYS else config.getfloat
            value = read(key, None)
            if value is not None:
                self.follow_keys[key] = setting_round(key, value, scale)
        self.has_follow = False  # the image carries the follow (its constants, at connect)
        self.follow = {}  # the values in effect, the firmware's defaults under the keys
        self.follow_bounds = {}  # key -> (low, high) in the key's unit, from the dictionary
        # the feed-forward's unit values, as the follow's: judged at connect against the
        # dictionary; on an image without them, ignored (the feed-forward is off)
        self.ff_keys = {}
        for key, _arg, scale, _name in FF_SETTINGS:
            value = config.getfloat(key, None)
            if value is not None:
                self.ff_keys[key] = setting_round(key, value, scale)
        self.has_ff = False  # the image carries the feed-forward (its constants, at connect)
        self.ff_values = {}  # the values in effect, the firmware's defaults under the keys
        self.ff_bounds = {}
        # the grip (at connect): the image carries it, its default and bounds in mm
        self.has_grip = False
        self.grip_mm = self.grip_min_mm = self.grip_max_mm = None
        # 0.05 Hz is a 20 s period, 2.4 × 10⁹ ticks at 120 MHz — under the 2^32 (35.8 s) the
        # unit's rest_ticks carries; a start before the session's first report waits two periods
        # plus the margin, so the floor also bounds that wait (45 s at 0.05 Hz)
        asked_hz = config.getfloat("feed_report_hz", FEED_REPORT_HZ, minval=0.05, maxval=10.0)
        # 1, 2, 5 and 10 Hz are on the grid; anything else moves to the nearest 100 ms period
        self.report_period_ms = max(
            REPORT_GRID_MS, round(1000.0 / asked_hz / REPORT_GRID_MS) * REPORT_GRID_MS
        )
        self.report_hz = 1000.0 / self.report_period_ms
        if abs(self.report_hz - asked_hz) > 1e-9:
            logging.warning(
                "ace2k: feed_report_hz %g snapped to %.3g Hz (a %d ms period on the %d ms grid)",
                asked_hz,
                self.report_hz,
                self.report_period_ms,
                REPORT_GRID_MS,
            )
        # the window the unit accepts a scale in: ace2k_lane_scale_set shuts the MCU down outside it
        self.scale_low = DEFAULT_SCALE_MM * (1.0 - SCALE_TOLERANCE)
        self.scale_high = DEFAULT_SCALE_MM * (1.0 + SCALE_TOLERANCE)
        self.scales = {}
        for lane in range(LANES):
            value = config.getfloat(
                f"lane{lane + 1}_encoder_scale", None, minval=self.scale_low, maxval=self.scale_high
            )
            if value is not None:
                self.scales[lane] = value
        self.lanes = [
            dict(
                mode=None,
                error=None,
                last_event=None,
                last_move=None,
                speed=None,
                duty=None,
                bursts=None,
                tail=False,  # the follow's tail: from the tail notice to its end
                ff_doses=None,
                ff_taut_fixes=None,
                ff_full_fixes=None,
                encoder_scale=self.scales.get(lane, DEFAULT_SCALE_MM),
                scale_source="config" if lane in self.scales else "default",
            )
            for lane in range(LANES)
        ]
        self.present = False
        self.speed_min = self.speed_max = self.move_max_mm = self.landing_mm = None
        self.tail_mm = None  # the motor an unload or a rollback runs past the slot mouth
        # lane -> (seq, completion): the waiting G-code's sequence and the completion it holds
        self.completions = {}
        # lane -> the last final (non-notice) event, the dict _dispatch completes with: a refused
        # start whose round trip swallowed the running move's end completes that move from it
        self.final_events = [None] * LANES
        # lane -> the sequence of the last start sent.  Until this session issues a start on a
        # lane, every state report seeds the counter with the seq the unit holds for the lane
        # (the last start issued on it, kept through idle and error; 0 before any since the
        # unit powered up), so the first start is reported + 1 and never the running move's
        # nor the one that just ended (the unit takes a repeat of its last start, within a
        # second of its end, as this transport's retry); after that the counter is the
        # host's and the reports leave it alone.  A start before the first report of the
        # session waits on first_report, which the first _seed() completes.  _next_seq() keeps
        # it in 1..255.
        self.seq = [0] * LANES
        self.started = [False] * LANES  # a start was issued on the lane this session
        self.state_reported = False  # a state report has arrived this session
        self.first_report = self.reactor.completion()
        # lane -> {seq: length_mm} of the starts whose terminal event has not landed
        self.commanded_mm = [{} for _ in range(LANES)]
        self.calibration = {}  # lane -> (filament_mm, scale_in_use) of the last calibration feed
        # the [ace2k] object, set by it once it has loaded this module: the lane counters' period
        # and the reported encoder kept continuous across a live scale change
        self.unit = None
        self.ff = ace2k_ff.FeedForward(self.printer, self, config)
        # ACE_CALIBRATE_ENCODER AUTO=1: the lane's scale read off its extruder
        self.cal = ace2k_encoder_cal.EncoderCalibration(self.printer, self)
        self.mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object("gcode")
        for name in GCODES:
            gcode.register_command(
                name, getattr(self, "cmd_" + name), desc=getattr(self, "cmd_" + name + "_help")
            )

    # --- configuration -------------------------------------------------------------------

    def _build_config(self):
        """The scales and the thresholds as config commands; every lane's scale again, the load
        settings, the snag values and the report query as init commands; the subscriptions — only
        for a firmware that carries the feed.  A start command present with another format is a
        firmware from another commit, refused at connect; one absent is a firmware built without
        the feed, logged."""
        verdict, detail = probe_format(self.mcu, START_FMT)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, START_FMT, detail)
        if verdict == "absent":
            logging.warning(
                "ace2k: firmware built without CONFIG_ACE2K_FEED; the feed G-codes are disabled"
                " (%s)",
                detail,
            )
            self.present = False
            return
        self.present = True
        self.speed_min = self.mcu.get_constant_float("ACE2K_LANE_SPEED_MIN_UM_S") / 1000.0
        self.speed_max = self.mcu.get_constant_float("ACE2K_LANE_SPEED_MAX_UM_S") / 1000.0
        self.move_max_mm = self.mcu.get_constant_float("ACE2K_LANE_MOVE_MAX_UM") / 1000.0
        self.landing_mm = self.mcu.get_constant_float("ACE2K_LANE_LANDING_UM") / 1000.0
        self.tail_mm = self.mcu.get_constant_float("ACE2K_FEED_UNLOAD_TAIL_UM") / 1000.0
        # a firmware with the reader's binding searches past the parking point for the tag
        self.has_search = self.mcu.try_lookup_command(SEARCH_FMT) is not None
        if self.search_mm is None:
            self.search_mm = firmware_search_mm(self.mcu)
        for key, speed in (
            ("feed_speed", self.feed_speed),
            ("unload_speed", self.unload_speed),
            ("load_speed", self.load_speed),
            ("assist_speed", self.assist_speed),
        ):
            if not self.speed_min <= speed <= self.speed_max:
                raise self.printer.config_error(
                    f"{self.section}: {key} must be within"
                    f" {self.speed_min:g}..{self.speed_max:g} mm/s"
                )
        if self.load_park_mm > self.move_max_mm:
            raise self.printer.config_error(
                f"{self.section}: load_park_mm must be at most {self.move_max_mm:g} mm"
            )
        # Every forward assist burst takes the comparator's bases afresh and chains into the next
        # while the filament is taut: a window as long as a burst never closes, and a filament held
        # during an assist would be ground on burst after burst.  The unit refuses such a window
        # as a host bug (a shutdown), so it is refused here first.
        burst_mm = self.mcu.get_constant_float("ACE2K_FEED_ASSIST_BURST_UM") / 1000.0
        for key in ("stall_check_mm", "slip_check_mm"):
            value = self.thresholds.get(key)
            if value is not None and value >= burst_mm:
                raise self.printer.config_error(
                    f"{self.section}: {key} ({value:g}) must be under {burst_mm:g} mm, an assist's"
                    " burst: a longer window never closes during an assist"
                )
        for lane, scale in sorted(self.scales.items()):
            # A config command: it enters Klipper's config checksum, so editing or removing the
            # key and restarting resets the MCU, which starts from its default again.
            self.mcu.add_config_cmd(self._scale_cmd_text(lane, scale))
        for lane in range(LANES):
            # And every lane's scale as an init command, the key's or the default: a host
            # restart finds the unit configured and sends it only the init commands, so a scale
            # applied live (the automatic calibration's, not saved) ends with the session.
            scale = self.lanes[lane]["encoder_scale"]
            self.mcu.add_config_cmd(self._scale_cmd_text(lane, scale), is_init=True)
        if self.thresholds:
            th = dict(THRESHOLD_DEFAULTS, **self.thresholds)
            self.mcu.add_config_cmd(
                f"ace2k_feed_thresholds_set slip_check_um={to_um(th['slip_check_mm'])}"
                f" slip_allow_um={to_um(th['slip_allow_mm'])}"
                f" stall_check_um={to_um(th['stall_check_mm'])}"
            )
        # unguarded, as ACE2K_FEED_UNLOAD_TAIL_UM: host and image are the same release
        defaults = {
            key: self.mcu.get_constant_float(name) / (1000.0 if key.endswith("_mm") else 1.0)
            for key, name in SNAG_DEFAULT_CONSTANTS.items()
        }
        self.snag = dict(defaults, **self.snag_keys)
        if self.snag["snag_lag_mm"] >= self.snag["snag_fwd_mm"]:
            # the unit refuses the set as a host bug (a shutdown), so it is refused here first
            raise self.printer.config_error(
                f"{self.section}: snag_lag_mm ({setting_text(self.snag['snag_lag_mm'])}) must be"
                f" under snag_fwd_mm ({setting_text(self.snag['snag_fwd_mm'])}): a lag the watch"
                " never reaches"
            )
        self.mcu.add_config_cmd(self._load_set_cmd(), is_init=True)
        # all six at every connect, keys or not, as the load settings: a config command reaches
        # only an unconfigured MCU, so a RESTART after ACE_SNAG_SET would leave the unit on the
        # live values while the host shows these
        self.mcu.add_config_cmd(self._snag_cmd_text(), is_init=True)
        self._build_follow()
        self._build_ff()
        self._build_grip()
        ticks = self.mcu.seconds_to_clock(self.report_period_ms / 1000.0)
        self.mcu.add_config_cmd(f"ace2k_feed_query rest_ticks={ticks}", is_init=True)
        self._subscribe(self._handle_state, STATE_FMT)
        self._subscribe(self._handle_event, EVENT_FMT)
        if self.has_ff:
            # the counters at the state report's rate, on the unit's own phase for them
            self.mcu.add_config_cmd(f"ace2k_feed_ff_query rest_ticks={ticks}", is_init=True)
            self._subscribe(self._handle_ff_state, FF_STATE_FMT)

    def _build_follow(self):
        """The follow's defaults and bounds from the dictionary, the keys judged against them, and
        both values as an init command — at every connect, as the snag's, so a RESTART after
        ACE_FOLLOW_SET never leaves the unit on the live values.  An image without the follow's
        constants has no follow: a key for it is a config error, and nothing is sent."""
        try:
            constants = {
                key: (
                    self.mcu.get_constant_float(name) / scale,
                    self.mcu.get_constant_float(name + "_MIN") / scale,
                    self.mcu.get_constant_float(name + "_MAX") / scale,
                )
                for key, _arg, scale, name in FOLLOW_SETTINGS
            }
        except Exception:  # Klipper's msgproto error; KeyError in the tests' fake
            self.has_follow = False
            if self.follow_keys:
                key = next(iter(self.follow_keys))
                raise self.printer.config_error(f"{self.section}: {key}: {NO_FOLLOW}") from None
            return
        follow = {}
        for key, (default, low, high) in constants.items():
            if key in FOLLOW_INT_KEYS:
                default, low, high = int(default), int(low), int(high)
            self.follow_bounds[key] = (low, high)
            value = self.follow_keys.get(key, default)
            # the unit refuses a value out of bounds as a host bug (a shutdown)
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise self.printer.config_error(f"{self.section}: {key} {reason}")
            follow[key] = value
        self.follow = follow
        self.has_follow = True
        self.mcu.add_config_cmd(self._follow_cmd_text(), is_init=True)

    def _build_ff(self):
        """The feed-forward's defaults and bounds from the dictionary, the keys judged against
        them, and both values as an init command at every connect, as the follow's.  An image
        without the constants has no feed-forward: nothing is sent to it, the keys are left
        unused, and ace2k_ff.py says so on the console at ready.  With the constants, the
        commands and the report must be there as this module knows them — another commit's
        firmware otherwise, refused."""
        try:
            constants = {
                key: (
                    self.mcu.get_constant_float(name) / scale,
                    self.mcu.get_constant_float(name + "_MIN") / scale,
                    self.mcu.get_constant_float(name + "_MAX") / scale,
                )
                for key, _arg, scale, name in FF_SETTINGS
            }
        except Exception:  # Klipper's msgproto error; KeyError in the tests' fake
            self.has_ff = False
            logging.info("ace2k: %s", ace2k_ff.NO_FF)
            return
        # the counters without their epoch: an image from before it, whose counts the host
        # cannot place — no feed-forward, as an image without one
        verdict, detail = probe_format(self.mcu, FF_STATE_FMT)
        if verdict != "ok":
            self.has_ff = False
            logging.info("ace2k: %s (%s)", ace2k_ff.NO_FF, detail)
            return
        for msgformat in (FF_FMT, ace2k_ff.BASE_FMT, FF_QUERY_FMT):
            verdict, detail = probe_format(self.mcu, msgformat)
            if verdict != "ok":
                raise refuse_format(
                    self.printer, self.section, msgformat, detail, verdict, START_FMT
                )
        values = {}
        for key, (default, low, high) in constants.items():
            self.ff_bounds[key] = (low, high)
            value = self.ff_keys.get(key, default)
            # the unit refuses a value out of bounds as a host bug (a shutdown)
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise self.printer.config_error(f"{self.section}: {key} {reason}")
            values[key] = value
        self.ff_values = values
        self.has_ff = True
        self.mcu.add_config_cmd(self._ff_cmd_text(), is_init=True)

    def _build_grip(self):
        """The grip's default and bounds from the dictionary, and every lane's grip cleared as an
        init command.  An image without the constants has no grip: nothing is sent,
        set_lane_grip and ACE_GRIP say to flash.  With them, the command must be there as this
        module knows it — another commit's firmware otherwise, refused."""
        try:
            grip, low, high = (
                self.mcu.get_constant_float(GRIP_CONSTANT + suffix) / 1000.0
                for suffix in ("", "_MIN", "_MAX")
            )
        except Exception:  # Klipper's msgproto error; KeyError in the tests' fake
            self.has_grip = False
            logging.info("ace2k: %s", NO_GRIP)
            return
        verdict, detail = probe_format(self.mcu, GRIP_FMT)
        if verdict != "ok":
            raise refuse_format(self.printer, self.section, GRIP_FMT, detail, verdict, START_FMT)
        self.grip_mm, self.grip_min_mm, self.grip_max_mm = grip, low, high
        self.has_grip = True
        # every lane's grip cleared at every connect: the grip lives in the unit's RAM, and a host
        # restart that leaves the MCU running (RESTART, SAVE_CONFIG) sends it only the init
        # commands — a grip set in the last session must never outlive it
        for lane in range(LANES):
            self.mcu.add_config_cmd(f"{GRIP_FMT.split()[0]} lane={lane} grip_um=0", is_init=True)

    def _ff_args(self):
        return [int(round(self.ff_values[key] * scale)) for key, _a, scale, _n in FF_SETTINGS]

    def _ff_cmd_text(self):
        chunk_um, pulse_um_s = self._ff_args()
        return f"ace2k_feed_ff_set chunk_um={chunk_um} pulse_um_s={pulse_um_s}"

    def _ff_line(self):
        v = {key: setting_text(value) for key, value in self.ff_values.items()}
        return (
            f"ace2k: feed-forward chunk {v['ff_chunk_mm']} mm, pulse {v['ff_pulse']} mm/s,"
            f" {self.ff.settings_text()}"
        )

    def _subscribe(self, handler, msgformat):
        """A subscription for a report the dictionary carries as this module knows it.  Called
        once the start command is known to be there, and the binding declares the start, the
        state report and the event in one file: a report with another format, or none at all, is
        a firmware from another commit either way — the config error names which, and the
        absent one names the start that is there."""
        verdict, detail = probe_format(self.mcu, msgformat)
        if verdict != "ok":
            raise refuse_format(self.printer, self.section, msgformat, detail, verdict, START_FMT)
        subscribe_response(self.mcu, handler, msgformat)

    def _scale_cmd_text(self, lane, scale):
        return f"{SCALE_FMT.split()[0]} lane={lane} um_per_count_x10={scale_x10(scale)}"

    def _load_set_cmd(self):
        return (
            f"ace2k_feed_load_set park_um={to_um(self.load_park_mm)}"
            f" speed_um_s={to_um(self.load_speed)} auto_load={1 if self.auto_load else 0}"
        )

    def _snag_args(self):
        return [int(round(self.snag[key] * scale)) for key, _a, scale, _l, _h in SNAG_SETTINGS]

    def _snag_cmd_text(self):
        names = ("fwd_um", "lag_um", "duty_pct", "back_um", "rest_ms", "free_um")
        args = " ".join(f"{n}={v}" for n, v in zip(names, self._snag_args()))
        return f"ace2k_feed_snag_set {args}"

    def _follow_args(self):
        return [int(round(self.follow[key] * scale)) for key, _a, scale, _n in FOLLOW_SETTINGS]

    def _follow_cmd_text(self):
        flip_ms, take_um, tail_um = self._follow_args()
        return f"ace2k_feed_follow_set flip_ms={flip_ms} take_um={take_um} tail_um={tail_um}"

    def _follow_line(self):
        f = {key: setting_text(value) for key, value in self.follow.items()}
        return (
            f"ace2k: follow flip {f['follow_flip_ms']} ms, take {f['follow_take_mm']} mm,"
            f" tail {f['follow_tail_mm']} mm"
        )

    def _snag_line(self):
        s = {key: setting_text(value) for key, value in self.snag.items()}
        return (
            f"ace2k: snag fwd {s['snag_fwd_mm']} mm, lag {s['snag_lag_mm']} mm,"
            f" duty {s['snag_duty_pct']} %, back {s['snag_back_mm']} mm,"
            f" rest {s['snag_rest_ms']} ms, free {s['snag_free_mm']} mm"
        )

    # --- reports and events (the serial thread) -------------------------------------------

    def _handle_state(self, params):
        mode = params["mode"]
        error = params["error"]
        speed = struct.unpack(f"<{LANES}I", params["speed_um_s"])
        duty = params["duty_pct"]
        bursts = struct.unpack(f"<{LANES}H", params["bursts"])
        for i, lane in enumerate(self.lanes):
            lane["mode"] = MODE_NAMES.get(mode[i], mode[i])
            lane["error"] = KIND_NAMES.get(error[i], error[i]) if mode[i] == MODE_ERROR else None
            lane["speed"] = speed[i] / 1000.0
            lane["duty"] = duty[i]
            lane["bursts"] = bursts[i]
            if lane["mode"] != "following":
                lane["tail"] = False  # the lane left the follow (its final event lost on the way)
        # the seeding on the reactor thread, where _start runs: a check-then-write here could
        # cross a _start between the check and the write and hand the next start a used seq.
        # Scheduled only while a lane still follows the reports: state_reported and each started
        # flag go False -> True once and never back, so a stale read here (the serial thread)
        # can only take a True for a False — one callback more, which seeds nothing — never
        # skip one that is owed.
        if not self.state_reported or not all(self.started):
            seqs = list(params["seq"])
            self.reactor.register_async_callback(lambda eventtime, seqs=seqs: self._seed(seqs))

    def _handle_ff_state(self, params):
        doses = struct.unpack(f"<{LANES}H", params["doses"])
        taut = struct.unpack(f"<{LANES}H", params["taut"])
        full = struct.unpack(f"<{LANES}H", params["full"])
        for i, lane in enumerate(self.lanes):
            lane["ff_doses"] = doses[i]
            lane["ff_taut_fixes"] = taut[i]
            lane["ff_full_fixes"] = full[i]
        self.ff.on_counts(taut, full, params["epoch"])
        # after the counters are stored: the calibration reads this frame's full count from them
        self.cal.on_ff_state(params.get("#receive_time", 0.0), taut, params["epoch"])

    def _seed(self, seqs):
        """The reactor thread: the unit's seq of every lane this session has not started on;
        the first frame of the session releases a start parked on first_report."""
        for i in range(LANES):
            if not self.started[i]:
                self.seq[i] = seqs[i]
        if not self.state_reported:
            self.state_reported = True
            self.first_report.complete(True)

    def _handle_event(self, params):
        index = params["lane"]
        kind = params["kind"]
        seq = params["seq"]
        event = dict(
            kind=KIND_NAMES.get(kind, kind),
            mode=MODE_NAMES.get(params["mode"], params["mode"]),
            motor_mm=to_mm(params["motor_um"]),
            filament_mm=to_mm(params["filament_um"]),
            seq=seq,
        )
        lane = self.lanes[index]
        lane["last_event"] = event
        if kind == KIND_TAIL:
            # the unit read the insert fall; the tail runs to its own final event, a filament pushed
            # in at the bay meanwhile changing nothing
            lane["tail"] = True
        elif kind not in NOTICE_KINDS:
            lane["tail"] = False  # a final event ends the mode, the tail with it
            # the commanded length of the start this event ends — None for a motion the unit
            # started itself (seq 0) or a start whose entry was pruned long ago
            lane["last_move"] = dict(event, commanded_mm=self.commanded_mm[index].pop(seq, None))
            self.final_events[index] = event
        logging.info(
            "ace2k feed: lane %d %s (%s) — motor %.1f mm, filament %.1f mm",
            index + 1,
            event["kind"],
            event["mode"],
            event["motor_mm"],
            event["filament_mm"],
        )
        self.reactor.register_async_callback(
            lambda eventtime, index=index, event=event, kind=kind: self._dispatch(
                index, event, kind
            )
        )

    def _dispatch(self, index, event, kind):
        # The reactor thread, as the G-codes: the completion is taken and completed in one step
        # against a _start that registers one and then yields in its send.  Only the event that
        # echoes the waiting G-code's sequence completes it; a terminal event with another — a
        # stale one from a move the host no longer waits for, or 0, a motion the unit started
        # itself — neither completes nor blocks anything.  The completion comes first, so a
        # listener of "ace2k:feed_event" sees the move already done and cannot hold it back.
        if kind not in NOTICE_KINDS:
            self._complete(index, event)
        self.printer.send_event("ace2k:feed_event", index, event)
        if event["kind"] == "snag":
            # a notice, not an error: the tip caught, the unit touched it back and went on
            self.printer.lookup_object("gcode").respond_info(
                f"ace2k: lane {index + 1} snag ({event['mode']}) — the tip caught at motor"
                f" {event['motor_mm']:.1f} mm, filament {event['filament_mm']:.1f} mm; went on"
            )

    def _complete(self, index, event):
        """Complete the lane's tracked move with its final event when the sequences match; the
        slot is taken first, so a move completes once."""
        pending = self.completions.get(index)
        if pending is not None and pending[0] == event["seq"]:
            del self.completions[index]
            pending[1].complete(event)

    # --- status ------------------------------------------------------------------------------

    def lane_status(self, index):
        status = dict(self.lanes[index])
        status.update(self.ff.lane_status(index))
        return status

    def any_busy(self):
        # the 1 Hz state report alone: a move accepted since the last one is what the unit's own
        # veto refuses — the backstop for this blind window
        return any(lane["mode"] not in (None, "idle", "error") for lane in self.lanes)

    def status_lines(self):
        lines = []
        for i, lane in enumerate(self.lanes):
            ev = lane["last_event"]
            if ev is None:
                last = "none"
            else:
                last = (
                    f"{ev['kind']} ({ev['mode']}, motor {ev['motor_mm']} mm,"
                    f" filament {ev['filament_mm']} mm)"
                )
            lines.append(
                f"lane {i + 1}: mode={lane['mode']} error={lane['error']}"
                f" speed={lane['speed']} mm/s duty={lane['duty']} % bursts={lane['bursts']}"
                f" scale={lane['encoder_scale']} ({lane['scale_source']}) last={last}"
            )
        return lines

    # --- the host interface (API_VERSION) -----------------------------------------------------

    def start_move(self, lane_index, mode, length_mm, speed_mm_s):
        """Host interface: start mode (a CMD_MODES name) on lane_index (0..3), not waiting.
        Callable from G-code handlers, timers and async callbacks. Returns a Move; raises
        FeedRefused, ValueError for an unknown lane or mode, or RuntimeError when the firmware
        has no feed, never reported, or has no follow for "assist_both"."""
        if not self.present:
            raise RuntimeError("ace2k: firmware built without CONFIG_ACE2K_FEED")
        if mode not in CMD_MODES or not 0 <= lane_index < LANES:
            raise ValueError(f"ace2k: no lane {lane_index} / mode {mode!r}")
        completion = self.reactor.completion()
        seq = self._send_start(lane_index, mode, length_mm, speed_mm_s, completion)
        return Move(lane_index, mode, seq, completion, self.reactor)

    def stop(self, lane_index):
        """Host interface: stop lane_index's motor (LANE_ALL = every lane); never clears an
        error.  ValueError for another lane."""
        if lane_index != LANE_ALL and not 0 <= lane_index < LANES:
            raise ValueError(f"ace2k: no lane {lane_index}")
        self.mcu.lookup_command("ace2k_feed_stop lane=%c").send([lane_index])

    def clear(self, lane_index):
        """Host interface: leave lane_index's error state.  ValueError for another lane."""
        if not 0 <= lane_index < LANES:
            raise ValueError(f"ace2k: no lane {lane_index}")
        self.mcu.lookup_command("ace2k_feed_clear lane=%c").send([lane_index])

    def set_lane_extruder(self, lane_index, extruder_name):
        """Host interface (API_VERSION 3): the extruder lane_index (0..3) feeds, for the
        feed-forward; None unmaps it.  A laneN_extruder key wins over it.  ValueError for
        another lane."""
        self.ff.set_lane_extruder(lane_index, extruder_name)

    def set_lane_grip(self, lane_index, grip_mm):
        """Host interface (API_VERSION 6): lane_index's (0..3) next automatic load pulls only
        grip_mm, without the tag search, and ends loaded; that load consumes the grip, and any
        start on the lane, or a read with motion, clears it.  0 clears it.  ValueError, before
        anything is sent, for another lane or a grip outside grip_min_mm..grip_max_mm (the unit
        shuts down on one); RuntimeError when the firmware has no feed or no grip."""
        if not self.present:
            raise RuntimeError("ace2k: firmware built without CONFIG_ACE2K_FEED")
        if not self.has_grip:
            raise RuntimeError(f"ace2k: {NO_GRIP}")
        if not 0 <= lane_index < LANES:
            raise ValueError(f"ace2k: no lane {lane_index}")
        grip_um = to_um(grip_mm)
        if grip_um and not to_um(self.grip_min_mm) <= grip_um <= to_um(self.grip_max_mm):
            raise ValueError(
                f"ace2k: a grip of {grip_mm:g} mm is outside the unit's"
                f" {self.grip_min_mm:g}..{self.grip_max_mm:g} mm (0 clears)"
            )
        self.mcu.lookup_command(GRIP_FMT).send([lane_index, grip_um])

    # --- the automatic encoder calibration (ace2k_encoder_cal.py) ----------------------------

    def set_counts_period(self, seconds):
        """The feed-forward counters' period from now on (10 Hz for an automatic calibration,
        the state report's afterwards)."""
        if self.has_ff:
            ticks = self.mcu.seconds_to_clock(seconds)
            self.mcu.lookup_command(FF_QUERY_FMT).send([ticks])

    def apply_scale(self, index, scale):
        """A lane's encoder scale in use at once, and staged for SAVE_CONFIG.  ValueError, before
        anything is sent, for a lane outside 0..3 or a scale outside the unit's window
        (scale_low..scale_high): the unit shuts down on one."""
        if not 0 <= index < LANES:
            raise ValueError(f"ace2k: no lane {index}")
        if not self.scale_low <= scale <= self.scale_high:
            raise ValueError(
                f"ace2k: {scale:.4f} mm/count is outside the unit's window"
                f" {self.scale_low:.4f}..{self.scale_high:.4f}"
            )
        old = self.lanes[index]["encoder_scale"]
        cmd = self.mcu.lookup_command(SCALE_FMT)
        if self.unit is not None:
            # the reference before the send: the unit cannot be on the new scale before it has
            # the command, so the last frame read now is at the old one
            self.unit.keep_encoder_continuous(index, old, scale)
        cmd.send([index, scale_x10(scale)])
        self.scales[index] = scale
        self.lanes[index]["encoder_scale"] = scale
        self.lanes[index]["scale_source"] = "calibrated"
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section, f"lane{index + 1}_encoder_scale", f"{scale:.4f}")

    # --- the G-codes -------------------------------------------------------------------------

    def _require(self, gcmd):
        if not self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_FEED")

    def _lane(self, gcmd):
        return gcmd.get_int("LANE", minval=1, maxval=LANES) - 1

    def _speed(self, gcmd, default):
        speed = gcmd.get_float("SPEED", default)
        if not self.speed_min <= speed <= self.speed_max:
            raise gcmd.error(
                f"ace2k: SPEED must be within {self.speed_min:g}..{self.speed_max:g} mm/s"
            )
        return speed

    def _next_seq(self, index):
        """The lane's next sequence: 1..255, wrapping past 255 to 1 — never 0, which marks a
        motion the unit started on its own."""
        self.seq[index] = self.seq[index] % SEQ_MAX + 1
        return self.seq[index]

    def _move_s(self, travel_mm, speed_mm_s):
        """One move's expected time, as the firmware budgets it: the landing (the last
        ACE2K_LANE_LANDING_UM, or the whole of a shorter move) at the floor speed, the rest at the
        commanded one."""
        if travel_mm > self.landing_mm:
            return (travel_mm - self.landing_mm) / speed_mm_s + self.landing_mm / self.speed_min
        return travel_mm / self.speed_min

    def _wait_timeout(self, travel_mm, speed_mm_s, tail_mm=0.0):
        """The mirror of the firmware's deadline, WAIT_FACTOR × the expected time plus
        WAIT_MARGIN_S; the tail of an unload or a rollback (tail_mm) is a second move of its own."""
        expected = self._move_s(travel_mm, speed_mm_s)
        if tail_mm:
            expected += self._move_s(tail_mm, speed_mm_s)
        return WAIT_FACTOR * expected + WAIT_MARGIN_S

    def _load_budget_mm(self, park_mm):
        """What a waited load may travel: the pull, and with the unit's tag search the search past
        the parking point, the way back and the sessions' pauses."""
        if not self.has_search or self.search_mm <= park_mm:
            return park_mm
        return park_mm + 2 * (self.search_mm - park_mm) + self.SESSION_ALLOWANCE_MM

    def load_wait_s(self, travel_mm):
        """How long a wait for travel_mm of the unit's load may take: the budget of a waited move
        at the current load speed (load_speed, or the last ACE_LOAD_SET) — the speed the unit's
        load, and its tag search, run at.  For ace2k_rfid.py's MOVE=1 read."""
        return self._wait_timeout(travel_mm, self.load_speed)

    def _send_start(self, index, mode, length_mm, speed_mm_s, completion):
        """The start, sent with the lane's next sequence; completion (or None) registered before
        the send. Returns the sequence; a refusal raises FeedRefused, registering nothing.
        Before the session's first state report the counter is not seeded past a move the unit
        may still be running, so the start waits for that report — STATE_WAIT_PERIODS of the
        configured rate plus WAIT_MARGIN_S — and only none within that is an error
        (RuntimeError).  The follow on an image without it is a RuntimeError, nothing sent."""
        if mode == "assist_both" and not self.has_follow:
            raise RuntimeError(f"ace2k: {NO_FOLLOW}")
        if not self.state_reported:
            timeout = STATE_WAIT_PERIODS / self.report_hz + WAIT_MARGIN_S
            if self.first_report.wait(self.reactor.monotonic() + timeout, None) is None:
                raise RuntimeError(
                    f"ace2k: no feed state report within {timeout:g} s (the unit reports at"
                    f" feed_report_hz = {self.report_hz:.3g} Hz; check the link)"
                )
        cmd = self.mcu.lookup_query_command(START_FMT, START_RESP)
        self.started[index] = True  # from here the counter is the host's; reports leave it alone
        seq = self._next_seq(index)
        # the lane's tracked move until the unit answers: a refusal (busy) gives it back, so
        # the move still running there completes on its own event
        previous = self.completions.get(index)
        if completion is not None:
            self.completions[index] = (seq, completion)
        # before the send: the final event of a short move can land during the round trip.  The
        # oldest entries go once more than COMMANDED_KEEP wait, so a lost event never grows the
        # map.  The serial thread pops entries meanwhile, so the keys are snapshotted first —
        # iterating the live dict could see its size change — and popped with a default — the
        # entry may be gone by then.
        lengths = self.commanded_mm[index]
        lengths[seq] = length_mm
        for old in list(lengths)[:-COMMANDED_KEEP]:
            lengths.pop(old, None)
        params = cmd.send([index, CMD_MODES[mode], to_um(length_mm), to_um(speed_mm_s), seq])
        if not params["accepted"]:
            if completion is not None and self.completions.get(index) == (seq, completion):
                if previous is None:
                    del self.completions[index]
                else:
                    self.completions[index] = previous
                    # the running move may have ended during the round trip, its event
                    # dispatched while this start held the slot: complete it from that event
                    # (a _dispatch still to come finds the slot gone)
                    final = self.final_events[index]
                    if final is not None and final["seq"] == previous[0]:
                        self._complete(index, final)
            lengths.pop(seq, None)  # entered no mode: the earlier starts' entries stay
            reason = REFUSAL_NAMES.get(params["reason"], params["reason"])
            raise FeedRefused(index, mode, reason)
        return seq

    def _start(self, gcmd, index, mode, length_mm, speed_mm_s, wait, expected_mm=None, tail_mm=0.0):
        """Send the start (_send_start); a refusal or no state report → a G-code error; waiting →
        the event that echoes the sequence, or a timeout that stops the lane."""
        completion = self.reactor.completion() if wait else None
        try:
            self._send_start(index, mode, length_mm, speed_mm_s, completion)
        except (FeedRefused, RuntimeError) as e:
            raise gcmd.error(str(e)) from e
        if not wait:
            gcmd.respond_info(f"ace2k: lane {index + 1} {mode} started")
            return None
        travel = expected_mm if expected_mm is not None else length_mm
        timeout = self._wait_timeout(travel, speed_mm_s, tail_mm)
        event = completion.wait(self.reactor.monotonic() + timeout, None)
        if event is None:
            # the unit is still moving as far as the host knows (a slower ACE_SPEED, a long
            # landing): stop it rather than leave a motor running behind an error
            self.completions.pop(index, None)
            self.stop(index)
            raise gcmd.error(
                f"ace2k: lane {index + 1} {mode}: no completion event within {timeout:.0f} s;"
                " a stop was sent"
            )
        summary = f"motor {event['motor_mm']} mm, filament {event['filament_mm']} mm"
        if event["kind"] in {KIND_NAMES[k] for k in ERROR_KINDS}:
            raise gcmd.error(f"ace2k: lane {index + 1} {mode} failed: {event['kind']} — {summary}")
        gcmd.respond_info(f"ace2k: lane {index + 1} {mode} {event['kind']} — {summary}")
        return event

    cmd_ACE_FEED_help = "Feed LENGTH=<mm> on LANE=<1-4> at SPEED=<mm/s>; WAIT=0 returns at once"

    def cmd_ACE_FEED(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.feed_speed)
        self._start(gcmd, index, "feed", length, speed, gcmd.get_int("WAIT", 1))

    cmd_ACE_ROLLBACK_help = (
        "Rewind LENGTH=<mm> on LANE=<1-4> at SPEED=<mm/s>; WAIT=0 returns at once"
    )

    def cmd_ACE_ROLLBACK(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.feed_speed)
        wait = gcmd.get_int("WAIT", 1)
        # once the filament leaves the drive the unit runs the unload's tail: the wait counts it
        self._start(gcmd, index, "rollback", length, speed, wait, tail_mm=self.tail_mm)

    cmd_ACE_UNLOAD_help = (
        "Rewind LANE=<1-4> until the filament leaves the slot mouth (LENGTH=<budget mm>, SPEED=)"
    )

    def cmd_ACE_UNLOAD(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        budget = gcmd.get_float("LENGTH", 0.0, minval=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.unload_speed)
        expected = budget if budget else self.move_max_mm
        wait = gcmd.get_int("WAIT", 1)
        # the unit runs the tail past the drive (a rollback too): the wait counts it
        self._start(gcmd, index, "unload", budget, speed, wait, expected, self.tail_mm)

    cmd_ACE_LOAD_help = (
        "Pull the filament of LANE=<1-4> in to the parking point (LENGTH=<mm>, SPEED=)"
    )

    def cmd_ACE_LOAD(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        length = gcmd.get_float("LENGTH", self.load_park_mm, above=0.0, maxval=self.move_max_mm)
        speed = self._speed(gcmd, self.load_speed)
        self._start(
            gcmd,
            index,
            "load",
            length,
            speed,
            gcmd.get_int("WAIT", 1),
            expected_mm=self._load_budget_mm(length),
        )

    cmd_ACE_ASSIST_help = (
        "Arm the assist on LANE=<1-4> at SPEED=<mm/s> (DIR=BACK for the reverse, DIR=BOTH for the"
        " follow); OFF=1 disarms; SOFT=1 reports a refusal instead of raising"
    )

    def cmd_ACE_ASSIST(self, gcmd):
        # SOFT=1 is for print hooks: a lane that cannot be armed is named, never raised, so the
        # print goes on.  Disarming (OFF=1) is the same with or without it.
        if not gcmd.get_int("SOFT", 0, minval=0, maxval=1) or gcmd.get_int("OFF", 0):
            self._assist(gcmd)
            return
        index = self._lane(gcmd)
        try:
            self._assist(gcmd)
        except gcmd.error as e:
            reason = str(e)
            if reason.startswith("ace2k: "):
                reason = reason[len("ace2k: ") :]
            gcmd.respond_info(f"ace2k: lane {index + 1} not armed — {reason}")

    def _assist(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        if gcmd.get_int("OFF", 0):
            self.stop(index)
            gcmd.respond_info(f"ace2k: lane {index + 1} assist off")
            return
        direction = gcmd.get("DIR", "").upper()
        if direction not in ASSIST_DIRS:
            raise gcmd.error("ace2k: DIR must be BACK or BOTH (or absent for the forward assist)")
        mode = ASSIST_DIRS[direction]
        speed = self._speed(gcmd, self.assist_speed)
        self._start(gcmd, index, mode, 0.0, speed, False)

    cmd_ACE_STOP_help = "Stop the motor of LANE=<1-4>, or every lane"

    def cmd_ACE_STOP(self, gcmd):
        self._require(gcmd)
        index = LANE_ALL if gcmd.get("LANE", None) is None else self._lane(gcmd)
        self.stop(index)
        which = "every lane" if index == LANE_ALL else f"lane {index + 1}"
        gcmd.respond_info(f"ace2k: stop sent to {which}")

    cmd_ACE_CLEAR_help = "Leave the error state on LANE=<1-4>"

    def cmd_ACE_CLEAR(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        self.clear(index)
        gcmd.respond_info(f"ace2k: lane {index + 1} cleared")

    cmd_ACE_SPEED_help = "Change the speed of a running move on LANE=<1-4>: SPEED=<mm/s>"

    def cmd_ACE_SPEED(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        speed = self._speed(gcmd, self.feed_speed)
        self.mcu.lookup_command("ace2k_feed_speed_set lane=%c speed_um_s=%u").send(
            [index, to_um(speed)]
        )
        gcmd.respond_info(f"ace2k: lane {index + 1} speed {speed:g} mm/s")

    cmd_ACE_CALIBRATE_ENCODER_help = (
        "LENGTH=<mm> feeds that much and keeps the encoder's reading; then measure the filament and"
        " run MEASURED=<mm> to stage laneN_encoder_scale for SAVE_CONFIG; or AUTO=1"
        " [LENGTH=<mm>] [SPEED=<mm/s>] extrudes through the lane's extruder and sets the scale"
        " from it"
    )

    def cmd_ACE_CALIBRATE_ENCODER(self, gcmd):
        self._require(gcmd)
        index = self._lane(gcmd)
        if gcmd.get_int("AUTO", 0, minval=0, maxval=1):
            # the automatic calibration: the scale read off the lane's extruder
            self.cal.run(gcmd, index)
            return
        measured = gcmd.get_float("MEASURED", None, above=0.0)
        if measured is None:
            length = gcmd.get_float("LENGTH", 200.0, above=0.0, maxval=self.move_max_mm)
            event = self._start(gcmd, index, "feed", length, self.feed_speed, True)
            if event["kind"] == "blocked":
                # the feed met what cannot move: its reading is no sample of the length; an
                # earlier sample goes too, so a MEASURED of this filament cannot scale against it
                self.calibration.pop(index, None)
                raise gcmd.error(
                    f"ace2k: lane {index + 1} calibration feed blocked — motor"
                    f" {event['motor_mm']} mm, filament {event['filament_mm']} mm; clear the path"
                    " and feed again"
                )
            scale = self.lanes[index]["encoder_scale"]
            self.calibration[index] = (event["filament_mm"], scale)
            gcmd.respond_info(
                f"ace2k: lane {index + 1} fed {length:g} mm, the encoder read"
                f" {event['filament_mm']} mm at {scale:.4f} mm/count; measure the filament"
                f" that came out, then ACE_CALIBRATE_ENCODER LANE={index + 1} MEASURED=<mm>"
            )
            return
        if index not in self.calibration:
            raise gcmd.error("ace2k: run ACE_CALIBRATE_ENCODER LANE=n LENGTH=<mm> first")
        filament_mm, scale = self.calibration[index]
        if filament_mm <= 0:
            raise gcmd.error("ace2k: the calibration feed moved no filament; nothing to scale")
        counts = filament_mm / scale
        new_scale = measured / counts
        if not self.scale_low <= new_scale <= self.scale_high:
            # staged, the value would restart Klipper into a config error at SAVE_CONFIG
            raise gcmd.error(
                f"ace2k: lane {index + 1}: {new_scale:.4f} mm/count is outside the window the"
                f" unit accepts, {self.scale_low:.4f}..{self.scale_high:.4f}"
                f" (±{SCALE_TOLERANCE:.0%} of {DEFAULT_SCALE_MM}); nothing staged — check the"
                " measurement"
            )
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section, f"lane{index + 1}_encoder_scale", f"{new_scale:.4f}")
        gcmd.respond_info(
            f"ace2k: lane {index + 1}: {counts:.0f} counts over {measured:g} mm ->"
            f" {new_scale:.4f} mm/count staged; run SAVE_CONFIG (the unit takes it at the restart)"
        )

    cmd_ACE_LOAD_SET_help = (
        "The automatic load's settings from now on: PARK=<mm> SPEED=<mm/s> AUTO=0/1 (staged for"
        " SAVE_CONFIG)"
    )

    def cmd_ACE_LOAD_SET(self, gcmd):
        self._require(gcmd)
        # Every argument read and checked before anything changes: a rejected one leaves the
        # settings, the staged SAVE_CONFIG values and the unit as they were
        park = gcmd.get_float("PARK", None, above=0.0, maxval=self.move_max_mm)
        speed = gcmd.get_float("SPEED", None)
        auto = gcmd.get_int("AUTO", None, minval=0, maxval=1)
        if speed is not None and not self.speed_min <= speed <= self.speed_max:
            raise gcmd.error(
                f"ace2k: SPEED must be within {self.speed_min:g}..{self.speed_max:g} mm/s"
            )
        configfile = self.printer.lookup_object("configfile")
        if park is not None:
            self.load_park_mm = park
            configfile.set(self.section, "load_park_mm", f"{park:g}")
        if speed is not None:
            self.load_speed = speed
            configfile.set(self.section, "load_speed", f"{speed:g}")
        if auto is not None:
            self.auto_load = bool(auto)
            configfile.set(self.section, "auto_load", "True" if auto else "False")
        self.mcu.lookup_command("ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c").send(
            [to_um(self.load_park_mm), to_um(self.load_speed), 1 if self.auto_load else 0]
        )
        gcmd.respond_info(
            f"ace2k: load park {self.load_park_mm:g} mm, speed {self.load_speed:g} mm/s, auto"
            f" {self.auto_load}; staged for SAVE_CONFIG"
        )

    cmd_ACE_SNAG_SET_help = (
        "The tip snag's values from now on: FWD=<mm> LAG=<mm> DUTY=<%> BACK=<mm> REST=<ms>"
        " FREE=<mm> (staged for SAVE_CONFIG); alone, the values in effect"
    )

    def cmd_ACE_SNAG_SET(self, gcmd):
        self._require(gcmd)
        # Every argument read and checked before anything changes, as ACE_LOAD_SET: a rejected
        # one leaves the values, the staged SAVE_CONFIG values and the unit as they were
        asked = {}
        for key, arg, scale, low, high in SNAG_SETTINGS:
            read = gcmd.get_int if key in SNAG_INT_KEYS else gcmd.get_float
            value = read(arg, None)
            if value is None:
                continue
            value = setting_round(key, value, scale)  # the bounds judged on what the unit gets
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise gcmd.error(f"ace2k: {arg} {reason}")
            asked[key] = value
        if not asked:
            gcmd.respond_info(self._snag_line())
            return
        values = dict(self.snag, **asked)
        if values["snag_lag_mm"] >= values["snag_fwd_mm"]:
            # the unit would take the set as a host bug (a shutdown)
            raise gcmd.error(
                f"ace2k: LAG ({setting_text(values['snag_lag_mm'])}) must be under FWD"
                f" ({setting_text(values['snag_fwd_mm'])}) mm"
            )
        self.snag = values
        configfile = self.printer.lookup_object("configfile")
        for key, value in asked.items():
            configfile.set(self.section, key, setting_text(value))
        self.mcu.lookup_command(SNAG_FMT).send(self._snag_args())
        gcmd.respond_info(self._snag_line() + "; staged for SAVE_CONFIG")

    cmd_ACE_FOLLOW_SET_help = (
        "The follow's values from now on: FLIP_MS=<ms> TAKE_MM=<mm> TAIL_MM=<mm> (staged for"
        " SAVE_CONFIG); alone, the values in effect"
    )

    def cmd_ACE_FOLLOW_SET(self, gcmd):
        self._require(gcmd)
        if not self.has_follow:
            raise gcmd.error(f"ace2k: {NO_FOLLOW}")
        # Every argument read and checked before anything changes, as ACE_SNAG_SET: a rejected
        # one leaves the values, the staged SAVE_CONFIG values and the unit as they were
        asked = {}
        for key, arg, scale, _name in FOLLOW_SETTINGS:
            read = gcmd.get_int if key in FOLLOW_INT_KEYS else gcmd.get_float
            value = read(arg, None)
            if value is None:
                continue
            value = setting_round(key, value, scale)  # the bounds judged on what the unit gets
            low, high = self.follow_bounds[key]
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise gcmd.error(f"ace2k: {arg} {reason}")
            asked[key] = value
        if not asked:
            gcmd.respond_info(self._follow_line())
            return
        self.follow = dict(self.follow, **asked)
        configfile = self.printer.lookup_object("configfile")
        for key, value in asked.items():
            configfile.set(self.section, key, setting_text(value))
        self.mcu.lookup_command(FOLLOW_FMT).send(self._follow_args())
        gcmd.respond_info(self._follow_line() + "; staged for SAVE_CONFIG")

    cmd_ACE_FF_SET_help = (
        "The feed-forward's values from now on: CHUNK_MM=<mm> PULSE=<mm/s> (the unit's)"
        " WINDOW_MS=<ms> LEAD_MS=<ms> DEADBAND=<mm/s> (the host's), staged for SAVE_CONFIG;"
        " alone, the values in effect"
    )

    def cmd_ACE_FF_SET(self, gcmd):
        self._require(gcmd)
        if not self.has_ff:
            raise gcmd.error(f"ace2k: {ace2k_ff.NO_FF}")
        # Every argument read and checked before anything changes, as ACE_FOLLOW_SET
        asked = {}
        for key, arg, scale, _name in FF_SETTINGS:
            value = gcmd.get_float(arg, None)
            if value is None:
                continue
            value = setting_round(key, value, scale)  # the bounds judged on what the unit gets
            low, high = self.ff_bounds[key]
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise gcmd.error(f"ace2k: {arg} {reason}")
            asked[key] = value
        host = {}
        for arg, attr, (key, _default, low, high), whole in FF_HOST_ARGS:
            value = (gcmd.get_int if whole else gcmd.get_float)(arg, None)
            if value is None:
                continue
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise gcmd.error(f"ace2k: {arg} {reason}")
            host[attr] = (key, value)
        if not asked and not host:
            gcmd.respond_info(self._ff_line())
            return
        configfile = self.printer.lookup_object("configfile")
        for attr, (key, value) in host.items():
            if attr == "lead_ms":
                self.ff.set_lead(value)  # a new lead restarts each extruder's sweep
            else:
                setattr(self.ff, attr, value)
            configfile.set(self.section, key, setting_text(value))
        if asked:
            self.ff_values = dict(self.ff_values, **asked)
            for key, value in asked.items():
                configfile.set(self.section, key, setting_text(value))
            self.mcu.lookup_command(FF_FMT).send(self._ff_args())
        gcmd.respond_info(self._ff_line() + "; staged for SAVE_CONFIG")

    cmd_ACE_FEED_FORWARD_help = (
        "Switch the feed-forward ON=1 or OFF=1 on LANE=<1-4>, or every lane; alone, the state"
    )

    def cmd_ACE_FEED_FORWARD(self, gcmd):
        self._require(gcmd)
        if not self.has_ff:
            raise gcmd.error(f"ace2k: {ace2k_ff.NO_FF}")
        lanes = range(LANES) if gcmd.get("LANE", None) is None else [self._lane(gcmd)]
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        off = gcmd.get_int("OFF", 0, minval=0, maxval=1)
        if on and off:
            raise gcmd.error("ace2k: ON=1 or OFF=1, not both")
        if on or off:
            self.ff.set_on(lanes, bool(on))
        gcmd.respond_info(self.ff.switch_line())

    cmd_ACE_GRIP_help = (
        "Grip on LANE=<1-4>: its next automatic load pulls only MM=<mm> (the firmware's default"
        " when absent), without the tag search; MM=0 clears"
    )

    def cmd_ACE_GRIP(self, gcmd):
        self._require(gcmd)
        if not self.has_grip:
            raise gcmd.error(f"ace2k: {NO_GRIP}")
        index = self._lane(gcmd)
        grip = gcmd.get_float("MM", self.grip_mm, minval=0.0)
        try:
            self.set_lane_grip(index, grip)
        except ValueError as e:
            raise gcmd.error(str(e)) from e
        if to_um(grip):
            gcmd.respond_info(
                f"ace2k: lane {index + 1} grip {grip:g} mm: its next automatic load pulls only"
                " that, without the tag search"
            )
        else:
            gcmd.respond_info(f"ace2k: lane {index + 1} grip cleared")


def load_config(config):
    # [ace2k] loads this module with printer.load_object(config, "ace2k_feed"), which hands it
    # the [ace2k_feed] section — one that does not exist.  The keys live in [ace2k]; reading
    # them through this wrapper puts them in Klipper's access tracking, so its unused-option
    # check passes.
    return Ace2kFeed(config.getsection("ace2k"))
