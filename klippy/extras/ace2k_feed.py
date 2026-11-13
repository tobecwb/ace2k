"""The filament lanes of the ace2k firmware, on the host.

This module provides the G-codes that a macro needs:

- ``ACEK_FEED``, ``ACEK_ROLLBACK``, ``ACEK_UNLOAD`` and ``ACEK_LOAD`` wait by default. The
  command returns on the final event of the move, and an error event becomes a G-code error.
  ``WAIT=0`` returns at once.
- ``ACEK_ASSIST`` arms the assist. ``DIR=BACK`` is the reverse, ``DIR=BOTH`` is the follow, and
  ``OFF=1`` disarms it.
- ``ACEK_STOP``, ``ACEK_CLEAR``, ``ACEK_SPEED`` and ``ACEK_CALIBRATE_ENCODER``.
- ``ACEK_LOAD_SET``, ``ACEK_SNAG_SET``, ``ACEK_FOLLOW_SET`` (``FLIP_MS``, ``TAKE_MM``,
  ``TAIL_MM``) and ``ACEK_FF_SET`` change the settings.
- ``ACEK_FEED_FORWARD`` controls the feed-forward (see ``ace2k_ff.py``). The feed-forward meters
  the moves of the planner out to the lane that is in the follow.
- ``ACEK_GRIP`` sets the grip of a lane. The next automatic load of the lane then pulls only that
  much, without the tag search.

Config keys
-----------
The module sends the tunable keys of ``[ace2k]`` to the unit as init commands at every connect.
They are never part of the config checksum of Klipper, so a changed key never resets the MCU.
The keys are:

- the thresholds of the supervisors,
- the encoder scale of each lane,
- the load settings,
- the values of the tip snag, the follow and the feed-forward.

The values go out as init commands at every connect. So a value that was set live, such as a scale
from the automatic calibration, never outlives the session. The load, snag, follow and
feed-forward values are sent again by every ``ACEK_LOAD_SET``, ``ACEK_SNAG_SET``,
``ACEK_FOLLOW_SET`` and ``ACEK_FF_SET``.

Reports and events
------------------
The reports ``ace2k_feed_state`` and ``ace2k_feed_ff_state`` and the events ``ace2k_feed_event``
update the status of the lanes. Every event is written to the log and sent as the Klipper event
``ace2k:feed_event`` (lane, event) for other modules. A ``snag`` notice is also printed to the
console.

When the filament runs out during the follow, the lane enters its tail. The ``tail`` notice
arrives, and the ``tail`` value of the lane is True in its status. It stays True until the final
event of the tail, or until the lane leaves ``following``. A filament that is pushed into the bay
during the tail changes nothing. The tail ends with the final event ``tail_out``, which is not an
error.

A push toward the head can meet something that cannot move. It then ends as ``blocked``, which is
also not an error. The lane is idle, and the filament waits where it stopped.

Sequence numbers
----------------
Every start carries a sequence number for its lane (1..255). The unit repeats it in the events of
the move that it started. A G-code that waits completes only on its own sequence. A late event
of an earlier move passes by. So does an event of a motion that the unit started itself
(sequence 0).

Until the first start on a lane in a session, the counter follows the sequence that the state
report shows for the lane. That is the last start on it, which the unit keeps through idle and
error. So a new session continues after an earlier one. It never shares a sequence with a move that
still runs or has just ended. A start before the first report of the session waits for it.

Other notes
-----------
``[ace2k]`` loads this module. If the firmware is built without ``CONFIG_ACE2K_FEED`` (there is no
``ace2k_feed_start`` in its dictionary), a log line says so, and every G-code here answers with an
error. A G-code that waits holds the G-code queue, as the ``G4`` of Klipper does. When you test
by hand, use ``WAIT=0`` and the event log.

Other host modules can use the host interface instead of G-codes. It can start, stop and clear
the move of a lane, change its speed, and map a lane to the extruder that it feeds. It consists
of ``start_move``, ``stop``, ``clear``, ``set_speed``, ``set_lane_extruder``, ``set_lane_grip``,
``set_lane_park``, ``set_lane_wait``, ``set_lane_follow``, ``default_move``, ``Move`` and
``FeedRefused``. These work where a G-code cannot be used. The version is ``API_VERSION``, and
``docs/protocol.md`` describes the interface.

Only one start can be in flight per lane. A start is refused as ``busy`` and nothing is sent if
the previous start of the lane still waits for its answer. A ``stopped`` event carries
``stop_by``, which says who sent the stop that it answers. It is ``"command"`` for a G-code and
``"host"`` for a host module.

``follow_default`` (see ``ace2k_follow.py``) arms the follow on every lane that has filament at
the start, and after an insertion.

Example configuration::

    [ace2k]
    #feed_speed: 30             # mm/s, the default of ACEK_FEED and ACEK_ROLLBACK
    #unload_speed: 80          # mm/s; left out: 80, or the unit's top speed when lower
    #load_speed: 60
    #load_park_mm: 300          # the parking distance of the load: the path from the motor to the
    #                           # outlet (323 mm on the development unit), minus a margin
    #auto_load: True            # pull in a filament to the parking point when it is inserted
    #assist_speed: 50
    #slip_check_mm: 50          # thresholds of the supervisors; a missing key leaves the firmware
    #slip_allow_mm: 15          # default; keep it under slip_check_mm, or the partial slip check
    #stall_check_mm: 20         # can never trip
    #snag_fwd_mm: 20            # tip snag: motor travel past "full" before the touch (5-50)
    #snag_lag_mm: 5             # filament lag behind the motor that counts as a jam (1.5-20, < fwd)
    #snag_duty_pct: 0           # motor duty above the feed-forward that counts as a jam (0-100)
    #snag_back_mm: 5            # the touch (2-20)
    #snag_rest_ms: 500          # wait for the plunger to be at rest (100-2000)
    #snag_free_mm: 50           # reverse: filament travel back before a taut trip is forgiven
    #                           # (20-200)
    #follow_default: True       # arm the follow on every lane with filament (ace2k_follow.py)
    #follow_lanes: all          # all, or extruder: only a lane that feeds an extruder
    #follow_flip_ms: 200        # follow: wait before a move against the last one (50-2000)
    #follow_take_mm: 15         # follow: take-up on the full signal of this lane (5-30)
    #follow_tail_mm: 2000       # follow: motor travel after a runout before the tail ends
    #                           # (200-2000)
    #unload_clear_mm: 20        # an unload: motor travel past the bay's sensor before it ends
    #                           # (0-100)
    #ff_chunk_mm: 3             # feed-forward: size of a dose (1-10)
    #ff_pulse: 15               # retired: the next version removes it. mm/s, least speed of a
    #                           # reverse dose; a forward dose runs at the floor speed
    #lane1_encoder_scale: 1.2342   # mm per encoder count, lane1..lane4 (ACEK_CALIBRATE_ENCODER)
    #feed_report_hz: 1          # 0.05-10 Hz; 1, 2, 5 and 10 are exact, others snap to the grid

The snag values are the values of the firmware. ``snag_duty_pct`` stays at 0, which means that
the duty guard is off. The guard was built as a second check for a hold that the filament slips
through, but it did not prove effective. The lag check ended every jam first, and a fixed excess
over the feed-forward has no safe margin. A healthy lane shows up to 9 points of excess, and a
jam shows 10 to 14. The key stays for a possible future case. A better signal would be the rise
since the full signal, which cancels the drag of a spool. That is not built. At the 80 mm/s top
speed a loaded lane can run at 100 % duty, 11 points above the feed-forward's 89 %, so the guard
is not usable there: a value of 11 or less fires on healthy moves at 80, and a larger one never
fires at that speed.
``snag_lag_mm: 12`` is an option.

With the tag reader (``ace2k_rfid.py``, whose ``rfid_search_mm`` this module also reads), a load
whose tag is not read by the parking point searches further and then comes back. A waited
``ACEK_LOAD`` allows time for that travel. If ``rfid_search_mm`` is not set, the reach is the
default of the firmware.
"""

from __future__ import annotations

import logging
import struct
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

try:
    from .ace2k_feed_connect import (
        DEFAULT_SCALE_MM,
        NO_GRIP,
        NO_PARK,
        NO_WAIT,
        SCALE_TOLERANCE,
    )
    from .ace2k_feed_motion import (  # noqa: F401 — re-exported
        CMD_MODES,
        COMMANDED_KEEP,
        GONE_REPORTS,
        NO_FOLLOW,
        PAUSE_WAIT_S,
        SESSION_ALLOWANCE_MM,
        WAIT_MARGIN_S,
        LaneMotion,
        Start,
    )
    from .ace2k_feed_proto import (  # noqa: F401 — re-exported
        API_VERSION,
        BASE_FMT,
        BASE_FMT_FORWARD,
        CLEAR_FMT,
        DEFAULT_SEARCH_MM,
        ERROR_KIND_NAMES,
        ERROR_KINDS,
        EVENT_FMT,
        FAILED_KIND_NAMES,
        FF_FMT,
        FF_QUERY_FMT,
        FF_STATE_FMT,
        FF_STATE_FMT_FORWARD,
        FOLLOW_FMT,
        GRIP_FMT,
        HELD_SET_FMT,
        KIND_NAMES,
        LOAD_SET_FMT,
        MODE_NAMES,
        NO_CLEAR,
        NO_FF,
        NO_STOP,
        NOTICE_KIND_NAMES,
        NOTICE_KINDS,
        PARK_FMT,
        REFUSAL_NAMES,
        REVERSE_CONSTANT,
        SCALE_FMT,
        SEARCH_FMT,
        SNAG_FMT,
        SPEED_FMT,
        SPEED_RESP,
        STALE_SENSORS_KIND,
        STALE_SENSORS_TEXT,
        START_FMT,
        START_RESP,
        STATE_FMT,
        STOP_FMT,
        STOP_KINDS,
        STOP_WHEN_FMT,
        STOP_WHEN_RESP,
        UNLOAD_SET_FMT,
        WAIT_FMT,
        Move,
        firmware_search_mm,
        half_mm,
        scale_x10,
        to_mm,
        to_um,
    )
    from .ace2k_feed_settings import (  # noqa: F401 — re-exported
        FF,
        FF_SETTINGS,
        FOLLOW,
        FOLLOW_INT_KEYS,
        FOLLOW_SETTINGS,
        HELD,
        HELD_SETTINGS,
        SNAG,
        SNAG_DEFAULT_CONSTANTS,
        SNAG_INT_KEYS,
        SNAG_SETTINGS,
        THRESHOLD_DEFAULT_CONSTANTS,
        THRESHOLD_DEFAULTS,
        UNLOAD,
        UNLOAD_SETTINGS,
        old_lag_text,
        raise_old_lag,
        read_thresholds,
        setting_bound_error,
        setting_round,
        setting_text,
        threshold_defaults,
        threshold_error,
        thresholds_cmd_text,
    )
    from .ace2k_link import (  # noqa: F401 — LANE_ALL, FeedRefused
        LANE_ALL,
        LANES,
        NO_FEED,
        NO_LANE,
        FeedRefused,
        check_lane,
    )
    from .ace2k_units import unprefixed
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_connect import (  # type: ignore[import-not-found, no-redef]
        DEFAULT_SCALE_MM,
        NO_GRIP,
        NO_PARK,
        NO_WAIT,
        SCALE_TOLERANCE,
    )
    from ace2k_feed_motion import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported
        CMD_MODES,
        COMMANDED_KEEP,
        GONE_REPORTS,
        NO_FOLLOW,
        PAUSE_WAIT_S,
        SESSION_ALLOWANCE_MM,
        WAIT_MARGIN_S,
        LaneMotion,
        Start,
    )
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported
        API_VERSION,
        BASE_FMT,
        BASE_FMT_FORWARD,
        CLEAR_FMT,
        DEFAULT_SEARCH_MM,
        ERROR_KIND_NAMES,
        ERROR_KINDS,
        EVENT_FMT,
        FAILED_KIND_NAMES,
        FF_FMT,
        FF_QUERY_FMT,
        FF_STATE_FMT,
        FF_STATE_FMT_FORWARD,
        FOLLOW_FMT,
        GRIP_FMT,
        HELD_SET_FMT,
        KIND_NAMES,
        LOAD_SET_FMT,
        MODE_NAMES,
        NO_CLEAR,
        NO_FF,
        NO_STOP,
        NOTICE_KIND_NAMES,
        NOTICE_KINDS,
        PARK_FMT,
        REFUSAL_NAMES,
        REVERSE_CONSTANT,
        SCALE_FMT,
        SEARCH_FMT,
        SNAG_FMT,
        SPEED_FMT,
        SPEED_RESP,
        STALE_SENSORS_KIND,
        STALE_SENSORS_TEXT,
        START_FMT,
        START_RESP,
        STATE_FMT,
        STOP_FMT,
        STOP_KINDS,
        STOP_WHEN_FMT,
        STOP_WHEN_RESP,
        UNLOAD_SET_FMT,
        WAIT_FMT,
        Move,
        firmware_search_mm,
        half_mm,
        scale_x10,
        to_mm,
        to_um,
    )
    from ace2k_feed_settings import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported
        FF,
        FF_SETTINGS,
        FOLLOW,
        FOLLOW_INT_KEYS,
        FOLLOW_SETTINGS,
        HELD,
        HELD_SETTINGS,
        SNAG,
        SNAG_DEFAULT_CONSTANTS,
        SNAG_INT_KEYS,
        SNAG_SETTINGS,
        THRESHOLD_DEFAULT_CONSTANTS,
        THRESHOLD_DEFAULTS,
        UNLOAD,
        UNLOAD_SETTINGS,
        old_lag_text,
        raise_old_lag,
        read_thresholds,
        setting_bound_error,
        setting_round,
        setting_text,
        threshold_defaults,
        threshold_error,
        thresholds_cmd_text,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — LANE_ALL and FeedRefused re-exported
        LANE_ALL,
        LANES,
        NO_FEED,
        NO_LANE,
        FeedRefused,
        check_lane,
    )
    from ace2k_units import unprefixed  # type: ignore[import-not-found, no-redef]

try:
    from . import ace2k_encoder_cal, ace2k_ff, ace2k_follow
    from .ace2k_feed_connect import FeedConnect
    from .ace2k_feed_motion import FeedMotion
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    import ace2k_encoder_cal  # type: ignore[import-not-found, no-redef]
    import ace2k_ff  # type: ignore[import-not-found, no-redef]
    import ace2k_follow  # type: ignore[import-not-found, no-redef]
    from ace2k_feed_connect import FeedConnect  # type: ignore[no-redef]
    from ace2k_feed_motion import FeedMotion  # type: ignore[no-redef]

if TYPE_CHECKING:
    from .ace2k_feed_base import UnitProto
    from .ace2k_klipper_types import Command, Config, GCodeCommand, Params

# The default of unload_speed, mm/s: the unit's top speed. An older image's top is lower.
UNLOAD_SPEED_DEFAULT = 80.0
# The DIR argument of ACEK_ASSIST. If it is missing or FORWARD, the assist goes forward.
# BACK is the reverse and BOTH is the follow.
ASSIST_DIRS = {"": "assist", "FORWARD": "assist", "BACK": "assist_back", "BOTH": "assist_both"}
# The tables of the thresholds, the snag, the follow and the feed-forward are in
# ace2k_feed_settings. The constants and texts of the keys are in ace2k_feed_connect.
# The host-side arguments of ACEK_FF_SET. Each entry is: argument, attribute,
# (key, default, min, max), and whether the value is an integer.
FF_HOST_ARGS = (
    ("WINDOW_MS", "window_ms", ace2k_ff.WINDOW_MS, True),
    ("LEAD_MS", "lead_ms", ace2k_ff.LEAD_MS, True),
    ("DEADBAND", "deadband_mm_s", ace2k_ff.DEADBAND, False),
    ("RETRACT_MIN_MM", "retract_min_mm", ace2k_ff.RETRACT_MIN_MM, False),
)
GCODES = (
    "ACEK_FEED",
    "ACEK_ROLLBACK",
    "ACEK_UNLOAD",
    "ACEK_LOAD",
    "ACEK_ASSIST",
    "ACEK_STOP",
    "ACEK_CLEAR",
    "ACEK_SPEED",
    "ACEK_CALIBRATE_ENCODER",
    "ACEK_LOAD_SET",
    "ACEK_SNAG_SET",
    "ACEK_FOLLOW_SET",
    "ACEK_FF_SET",
    "ACEK_FEED_FORWARD",
    "ACEK_GRIP",
    "ACEK_LANE_ACTIVE",
)


class Ace2kFeed(FeedConnect, FeedMotion):
    """The feed of one unit: its status, its settings and its G-codes.

    The keys and the connect step come from ``FeedConnect`` (ace2k_feed_connect.py). The lane
    motions come from ``FeedMotion`` (ace2k_feed_motion.py)."""

    API_VERSION = API_VERSION  # so that a caller with only the feed object can read it

    # The helpers that this feed builds. Each has its own class. The two parts declare what they
    # read.
    ff: ace2k_ff.FeedForward
    follow_auto: ace2k_follow.FollowDefault

    def __init__(self, config: Config, unit: UnitProto) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        # The unit object that built this feed. The feed uses it for the period of the lane
        # counters, and to keep the reported encoder continuous across a live scale change.
        self.unit = unit
        self.mcu = unit.mcu  # the MCU of the unit, resolved once from the mcu key of its section
        self.section = config.get_name()
        self.feed_speed_mm_s = config.getfloat("feed_speed", 30.0, above=0.0)
        # Left out, unload_speed is UNLOAD_SPEED_DEFAULT, or the unit's top speed when lower: the
        # connect lowers it to an older image's top. A value that is set is checked at connect.
        unload_speed = config.getfloat("unload_speed", None, above=0.0)
        self.unload_speed_set = unload_speed is not None
        self.unload_speed_mm_s = UNLOAD_SPEED_DEFAULT if unload_speed is None else unload_speed
        self.load_speed_mm_s = config.getfloat("load_speed", 60.0, above=0.0)
        self.assist_speed_mm_s = config.getfloat("assist_speed", 50.0, above=0.0)
        self.load_park_mm = config.getfloat("load_park_mm", 300.0, above=0.0)
        self.auto_load = config.getboolean("auto_load", True)
        # The reach of the tag search of the load. ace2k_rfid.py sends this key to the unit. If it
        # is not set, the firmware default is used (read at connect). Here only the time budget of
        # ACEK_LOAD reads it.
        self.search_mm = config.getfloat("rfid_search_mm", None, minval=100.0, maxval=1000.0)
        self.has_search = False
        self._init_settings(config)
        self._init_report_rate(config)
        self._init_scales(config)
        self.lanes = [
            dict(
                mode=None,
                error=None,
                last_event=None,
                last_move=None,
                speed=None,
                duty=None,
                bursts=None,
                tail=False,  # True in the tail of the follow, from the tail notice to its end
                ff_doses=None,
                ff_back_doses=None,
                ff_taut_fixes=None,
                ff_full_fixes=None,
                encoder_scale=self.scales.get(lane, DEFAULT_SCALE_MM),
                scale_source="config" if lane in self.scales else "default",
            )
            for lane in range(LANES)
        ]
        self.present = False
        self.speed_min = self.speed_max = self.move_max_mm = self.landing_mm = None
        self.tail_mm = None  # the motor travel of an unload or rollback past the bay entrance
        self.unload_clear_mm = 0.0  # an unload's travel past the bay's sensor (0: none)
        # The commands and queries that the connect builds. They are None until then.
        self.start_query = self.speed_query = self.stop_when_query = None
        self.stop_cmd = self.clear_cmd = self.scale_cmd = self.load_set_cmd = None
        self.ff_base_cmd = self.ff_query_cmd = self.grip_cmd = self.park_cmd = None
        self.wait_cmd = None
        # The motion record of each lane (ace2k_feed_motion.LaneMotion). It holds the sequences,
        # the label of the stop, the accepted start and the waits. The state reports set the
        # sequences until the first start of the lane in the session. A start before the first
        # report of the session waits on first_report, which the first _seed() completes.
        self.motion = [LaneMotion() for _ in range(LANES)]
        self.reports = 0  # state reports in this session (the clock of follow_default)
        self.state_reported = False  # a state report has arrived this session
        self.first_report = self.reactor.completion()
        # lane -> (filament_mm, scale_in_use) of the last calibration feed.
        self.calibration: dict[int, tuple[float, float]] = {}
        self.ff = ace2k_ff.FeedForward(self.printer, self, config)
        # ACEK_CALIBRATE_ENCODER AUTO=1 reads the scale of the lane from its extruder.
        self.cal = ace2k_encoder_cal.EncoderCalibration(self.printer, self)
        # follow_default arms the follow on every lane with filament, at the start and after an
        # insertion (see ace2k_follow.py).
        self.follow_auto = ace2k_follow.FollowDefault(self.printer, self, config)
        self.follow_default = self.follow_auto.enabled
        self.mcu.register_config_callback(self._build_config)
        for name in GCODES:
            unit.register_command(
                name, getattr(self, "cmd_" + name), getattr(self, "cmd_" + name + "_help")
            )

    # --- the counters of the feed-forward (the serial thread) ----------------------------------

    def _handle_ff_state(self, params: Params) -> None:
        doses = struct.unpack(f"<{LANES}H", params["doses"])
        taut = struct.unpack(f"<{LANES}H", params["taut"])
        full = struct.unpack(f"<{LANES}H", params["full"])
        raw = params.get("back")
        back = struct.unpack(f"<{LANES}H", raw) if raw is not None else (None,) * LANES
        for i, lane in enumerate(self.lanes):
            lane["ff_doses"] = doses[i]
            lane["ff_back_doses"] = back[i]
            lane["ff_taut_fixes"] = taut[i]
            lane["ff_full_fixes"] = full[i]
        self.ff.on_counts(taut, full, params["epoch"])
        # This comes after the counters are stored. The calibration reads the full count of this
        # report from them.
        self.cal.on_ff_state(params.get("#receive_time", 0.0), taut, params["epoch"])

    # --- status ------------------------------------------------------------------------------

    @property
    def has_motor_travel(self) -> bool:
        """True when the image reports the motor's travel each way (host interface 25)."""
        return self.unit.has_motor_travel

    def motor_watch(self, on: bool) -> bool:
        """Turn the 1 Hz motor report on or off for one watcher (see Ace2k.motor_watch).

        The lane status then carries motor_fwd_mm and motor_back_mm. It returns
        has_motor_travel; on an image without the report it does nothing."""
        return self.unit.motor_watch(on)

    def lane_status(self, index: int) -> dict[str, object]:
        status = dict(self.lanes[index])
        status.update(self.ff.lane_status(index))
        status["follow_default"] = self.follow_auto.lane_state(index)
        status["follow_why"] = self.follow_auto.why_not(index)
        status["active"] = self.follow_auto.eligible(index)
        return status

    def any_busy(self) -> bool:
        """Return True if any lane may be running a motion.

        This uses only the state report, which arrives at the state report rate (1 Hz by default).
        A move that the unit accepted since the last report is not seen here. The unit refuses a
        conflicting move itself, which is the safety net for this blind window.
        """
        # Idle, error and no report yet (None) mean that nothing runs. An unknown mode code may be
        # a motion, so it counts as busy.
        return any(lane["mode"] not in (None, "idle", "error") for lane in self.lanes)

    def status_lines(self) -> list[str]:
        lines: list[str] = []
        for i, lane in enumerate(self.lanes):
            ev = lane["last_event"]
            if ev is None:
                last = "none"
            elif ev.get("tag_search"):
                # A read with motion went out and came back: its filament figure is about 0.
                last = f"tag search (about {half_mm(ev['motor_mm'])} mm out and back)"
            else:
                last = (
                    f"{ev['kind']} ({ev['mode']}, motor {ev['motor_mm']} mm,"
                    f" filament {ev['filament_mm']} mm)"
                )
            lines.append(
                f"lane {i + 1}: mode={lane['mode']} error={lane['error']}"
                f" speed={lane['speed']} mm/s duty={lane['duty']} % bursts={lane['bursts']}"
                f" scale={lane['encoder_scale']} ({lane['scale_source']}) last={last}"
                f" follow={self.follow_auto.follow_text(i)}"
            )
        # The shared extruders that this unit feeds. Each line shows the whole group across every
        # unit, such as "1, 2" with one unit and "a:1, b:3" with several.
        units = self.unit.units
        active = units.active_map()
        groups = units.groups()
        for name in self.follow_auto.groups():
            lanes = units.members_text(groups[name])
            active_lane = active[name]
            chosen = f"lane {active_lane}" if active_lane is not None else "none (ACEK_LANE_ACTIVE)"
            lines.append(f"{name}: lanes {lanes} — active {chosen}")
        return lines

    # --- the host interface (see API_VERSION) -----------------------------------------------------

    def set_lane_extruder(self, lane_index: int, extruder_name: str | None) -> None:
        """Host interface: set the extruder that lane ``lane_index`` (0..3) feeds.

        The feed-forward uses this mapping. None removes it. A laneN_extruder config key takes
        priority over it. Raises ValueError for another lane."""
        self.ff.set_lane_extruder(lane_index, extruder_name)

    def set_lane_follow(
        self, lane_index: int, speed_mm_s: float | None = None, on_insert: bool = True
    ) -> None:
        """Host interface: set how follow_default arms lane ``lane_index`` (0..3).

        The follow is armed at ``speed_mm_s``. If it is None, assist_speed is used. With
        ``on_insert`` False, the lane is armed only at the start. A module that runs its own flow
        on the insertions of the lane can use this. It arms the lane itself when its flow ends.
        This can be called from config time on. It only stores the values and sends nothing.

        Raises ValueError for another lane or a speed that is not above 0. The unit has its own
        speed limits and refuses a speed outside them when the lane is armed."""
        self.follow_auto.set_lane(lane_index, speed_mm_s, on_insert)

    def follow_paused(self, lane_index: int) -> bool:
        """Host interface: return True while a G-code move holds lane ``lane_index``.

        In that case ace2k stopped the follow for the move, and the move has not ended. A module
        must not arm the lane in the meantime. Raises ValueError for another lane."""
        check_lane(lane_index, NO_LANE)
        return self.follow_auto.is_paused(lane_index)

    def hold_follow(self, lane_index: int, held: bool, why: str | None = None) -> None:
        """Host interface: while ``held`` is True, follow_default arms nothing on the lane.

        An example is a lane whose filament waits behind another piece. ``why`` (since
        API_VERSION 21) is the reason the lane's status and ACEK_STATUS show (``follow_why``).
        Raises ValueError for another lane."""
        self.follow_auto.hold(lane_index, held, why)

    def default_move(self, lane_index: int) -> Move | None:
        """Host interface: return the Move of the follow that follow_default armed on the lane.

        The Move is returned until its final event arrives. After that, return None. Raises
        ValueError for another lane."""
        check_lane(lane_index, NO_LANE)
        return self.follow_auto.current(lane_index)

    def set_lane_grip(self, lane_index: int, grip_mm: float) -> None:
        """Host interface: make the next automatic load of lane ``lane_index`` pull ``grip_mm``.

        That load skips the tag search and ends loaded. It uses up the grip. Any start on the
        lane, or a read with motion, also clears the grip. A value of 0 clears it.

        Raises ValueError, before anything is sent, for another lane or for a grip outside
        grip_min_mm..grip_max_mm. The unit would shut down on such a value. Raises RuntimeError
        when the firmware has no feed or no grip."""
        grip_cmd = self._slot_cmd(lane_index, self.has_grip, NO_GRIP, self.grip_cmd)
        grip_um = to_um(grip_mm)
        # With has_grip, the connect has read the limits.
        assert self.grip_min_mm is not None and self.grip_max_mm is not None
        if grip_um and not to_um(self.grip_min_mm) <= grip_um <= to_um(self.grip_max_mm):
            raise ValueError(
                f"ace2k: a grip of {grip_mm:g} mm is outside the unit's"
                f" {self.grip_min_mm:g}..{self.grip_max_mm:g} mm (0 clears)"
            )
        grip_cmd.send([lane_index, grip_um])

    def set_lane_park(self, lane_index: int) -> None:
        """Host interface: make the next automatic load of lane ``lane_index`` stop at the park.

        The load pulls to the parking point (load_park_mm) and reads the tag on the way, like any
        load. It does no tag search past that point, and it ends loaded. Use it for a bay whose
        search could reach a tube that another lane shares.

        The park uses the slot of the grip on the unit. The load uses it up. Any start on the
        lane, a read with motion, and set_lane_grip(lane_index, 0) clear it. A grip that is set
        later replaces it. This only sends a command and never yields.

        Raises ValueError, before anything is sent, for another lane. Raises RuntimeError when the
        firmware has no feed or no park (has_park)."""
        self._slot_cmd(lane_index, self.has_park, NO_PARK, self.park_cmd).send([lane_index])

    def set_lane_wait(self, lane_index: int) -> None:
        """Host interface: make an insert edge on lane ``lane_index`` start no automatic load.

        The filament then stays where the hand left it, however often it goes in. Use it for a
        lane whose head still holds a piece that the lane does not own.

        The wait uses the slot of the grip on the unit. An insertion does not use it up. Any start
        on the lane, a read with motion, and set_lane_grip(lane_index, 0) clear it. A grip or a
        park that is set later replaces it. This only sends a command and never yields.

        Raises ValueError, before anything is sent, for another lane. Raises RuntimeError when the
        firmware has no feed or no wait (has_wait)."""
        self._slot_cmd(lane_index, self.has_wait, NO_WAIT, self.wait_cmd).send([lane_index])

    def _slot_cmd(self, lane_index: int, has: bool, missing: str, cmd: Command | None) -> Command:
        """Return ``cmd``, a command that writes the grip slot of a lane, after the checks of
        set_lane_grip, set_lane_park and set_lane_wait.

        Raises RuntimeError when the firmware has no feed, or lacks the command (``has`` is
        False; the text says ``missing``). Raises ValueError for another lane."""
        if not self.present:
            raise RuntimeError(f"ace2k: {NO_FEED}")
        if not has:
            raise RuntimeError(f"ace2k: {missing}")
        check_lane(lane_index, NO_LANE)
        assert cmd is not None  # the connect built it, because ``has`` is set
        return cmd

    # --- the automatic encoder calibration (ace2k_encoder_cal.py) ----------------------------

    def set_counts_period(self, seconds: float) -> None:
        """Set the period of the feed-forward counters from now on, in seconds.

        The automatic calibration uses 10 Hz. Afterwards the period of the state report is used."""
        if self.has_ff:
            ticks = self.mcu.seconds_to_clock(seconds)
            assert self.ff_query_cmd is not None  # the connect built it, because has_ff is set
            self.ff_query_cmd.send([ticks])

    def apply_scale(self, index: int, scale: float) -> None:
        """Use the encoder scale of a lane at once, and stage it for SAVE_CONFIG.

        ``scale`` is in mm per count. Raises ValueError, before anything is sent, for a lane
        outside 0..3 or a scale outside the range that the unit accepts (scale_low..scale_high).
        The unit would shut down on such a scale."""
        check_lane(index, NO_LANE)
        if not self.scale_low <= scale <= self.scale_high:
            raise ValueError(
                f"ace2k: {scale:.4f} mm/count is outside the unit's window"
                f" {self.scale_low:.4f}..{self.scale_high:.4f}"
            )
        old = self.lanes[index]["encoder_scale"]
        cmd = self.scale_cmd
        assert cmd is not None  # only the calibration applies a scale, on a feed that it found
        # Set the reference before the send. The unit cannot use the new scale before it has the
        # command, so the last frame that was read until now used the old scale.
        self.unit.keep_encoder_continuous(index, old, scale)
        cmd.send([index, scale_x10(scale)])
        self.scales[index] = scale
        self.lanes[index]["encoder_scale"] = scale
        self.lanes[index]["scale_source"] = "calibrated"
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section, f"lane{index + 1}_encoder_scale", f"{scale:.4f}")

    # --- settings staged for SAVE_CONFIG -------------------------------------------------------

    def _stage(self, values: Mapping[str, float]) -> None:
        """Stage each key of ``values`` for SAVE_CONFIG, written as setting_text writes it."""
        configfile = self.printer.lookup_object("configfile")
        for key, value in values.items():
            configfile.set(self.section, key, setting_text(value))

    def _ff_host_asked(self, gcmd: GCodeCommand) -> dict[str, tuple[str, float]]:
        """Return the host-side arguments of ACEK_FF_SET that were given, each checked.

        The result maps an attribute name to (config key, value)."""
        host: dict[str, tuple[str, float]] = {}
        for arg, attr, (key, _default, low, high), whole in FF_HOST_ARGS:
            value = (gcmd.get_int if whole else gcmd.get_float)(arg, None)
            if value is None:
                continue
            reason = setting_bound_error(key, value, low, high)
            if reason is not None:
                raise gcmd.error(f"ace2k: {arg} {reason}")
            host[attr] = (key, value)
        return host

    # --- the G-codes -------------------------------------------------------------------------

    def _lane_args(self, index: int) -> str:
        """Return the lane arguments for a command that is printed ready to run.

        With several units, this includes UNIT=."""
        return self.unit.units.lane_args(self.unit, index)

    cmd_ACEK_ASSIST_help = (
        "Arm the assist on LANE=<1-4> at SPEED=<mm/s> (DIR=BACK for the reverse, DIR=BOTH for the"
        " follow); OFF=1 disarms; SOFT=1 reports a refusal instead of raising"
    )

    def cmd_ACEK_ASSIST(self, gcmd: GCodeCommand) -> None:
        # SOFT=1 is for print hooks. A lane that cannot be armed is named in a message and no error
        # is raised, so the print goes on. A busy lane is not waited for. Disarming (OFF=1) works
        # the same with or without it.
        if not gcmd.get_int("SOFT", 0, minval=0, maxval=1) or gcmd.get_int("OFF", 0):
            self._assist(gcmd)
            return
        index = self._lane(gcmd)
        try:
            self._assist(gcmd, retry=False)
        except gcmd.error as e:
            gcmd.respond_info(f"ace2k: lane {index + 1} not armed — {unprefixed(e)}")

    def _assist(self, gcmd: GCodeCommand, *, retry: bool = True) -> None:
        """Arm or disarm the assist. ``retry`` False (SOFT=1) answers a busy refusal at once."""
        self._require(gcmd)
        index = self._lane(gcmd)
        if gcmd.get_int("OFF", 0):
            self.stop(index, by="command")
            gcmd.respond_info(f"ace2k: lane {index + 1} assist off")
            return
        direction = gcmd.get("DIR", "").upper()
        if direction not in ASSIST_DIRS:
            raise gcmd.error("ace2k: DIR must be BACK or BOTH (or absent for the forward assist)")
        mode = ASSIST_DIRS[direction]
        speed = self._speed(gcmd, self.assist_speed_mm_s)
        if mode == "assist_both":
            why = self.not_active_text(index)
            if why is not None:
                raise gcmd.error(why)
        self._start(gcmd, index, Start(mode, 0.0, speed), False, retry=retry)

    def lane_unavailable(self, index: int, allow: tuple[str, ...] | None = ("idle",)) -> str | None:
        """Return the reason why the lane takes no new motion, or None if it does.

        The checks run in this order. "no_filament" means there is no filament. "error" means the
        lane is in error, unless "error" is in ``allow``. "busy" means that the mode of the lane
        is not in ``allow``. A follow whose end has come counts as idle. If ``allow`` is None, the
        mode is not checked. Each caller writes its own message."""
        if self.unit.lanes[index]["insert"] is not True:
            return "no_filament"
        mode = self.lanes[index]["mode"]
        if mode == "error" and "error" not in (allow or ()):
            return "error"
        if mode == "following" and not self.in_follow(index):
            mode = "idle"
        return None if allow is None or mode in allow else "busy"

    def not_active_text(self, index: int) -> str | None:
        """Return the text with which a G-code refuses the follow on the lane, or None.

        The text is returned when the lane shares its extruder (on any unit) and is not its active
        lane. It names ACEK_LANE_ACTIVE."""
        if self.unit.units.may_follow(self.unit, index):
            return None
        return (
            f"ace2k: lane {self.unit.lane_text(index)} shares {self.ff.extruder_of(index)}"
            f" and is not its active lane: choose it with ACEK_LANE_ACTIVE {self._lane_args(index)}"
        )

    cmd_ACEK_CALIBRATE_ENCODER_help = (
        "LENGTH=<mm> feeds that much and keeps the encoder's reading; then measure the filament and"
        " run MEASURED=<mm> to stage laneN_encoder_scale for SAVE_CONFIG; or AUTO=1"
        " [LENGTH=<mm>] [SPEED=<mm/s>] extrudes through the lane's extruder and sets the scale"
        " from it"
    )

    def cmd_ACEK_CALIBRATE_ENCODER(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        index = self._lane(gcmd)
        if gcmd.get_int("AUTO", 0, minval=0, maxval=1):
            # The automatic calibration reads the scale from the extruder of the lane.
            self.cal.run(gcmd, index)
            return
        measured = gcmd.get_float("MEASURED", None, above=0.0)
        if measured is None:
            length = gcmd.get_float("LENGTH", 200.0, above=0.0, maxval=self.move_max_mm)
            event = self._start(gcmd, index, Start("feed", length, self.feed_speed_mm_s), True)
            assert event is not None  # a waited start returns its final event
            if event["kind"] == "blocked":
                # The feed met something that cannot move, so its reading is not a sample of the
                # length. Remove an earlier sample too, so that a MEASURED value for this
                # filament cannot be scaled against it.
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
                " that came out, then ACEK_CALIBRATE_ENCODER"
                f" {self._lane_args(index)} MEASURED=<mm>"
            )
            return
        if index not in self.calibration:
            raise gcmd.error(
                f"ace2k: run ACEK_CALIBRATE_ENCODER {self._lane_args(index)} LENGTH=<mm> first"
            )
        filament_mm, scale = self.calibration[index]
        if filament_mm <= 0:
            raise gcmd.error("ace2k: the calibration feed moved no filament; nothing to scale")
        counts = filament_mm / scale
        new_scale = measured / counts
        if not self.scale_low <= new_scale <= self.scale_high:
            # If this value were staged, SAVE_CONFIG would restart Klipper into a config error.
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

    cmd_ACEK_LOAD_SET_help = (
        "The automatic load's settings from now on: PARK=<mm> SPEED=<mm/s> AUTO=0/1 (staged for"
        " SAVE_CONFIG); alone, the values in effect"
    )

    def cmd_ACEK_LOAD_SET(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        # Read and check every argument before anything changes. A rejected argument leaves the
        # settings, the values staged for SAVE_CONFIG, and the unit as they were.
        park = gcmd.get_float("PARK", None, above=0.0, maxval=self.move_max_mm)
        speed = gcmd.get_float("SPEED", None)
        auto = gcmd.get_int("AUTO", None, minval=0, maxval=1)
        if park is None and speed is None and auto is None:
            gcmd.respond_info(self._load_line())
            return
        if speed is not None and self.speed_out(speed):
            raise gcmd.error(f"ace2k: SPEED must be within {self.speed_out(speed)}")
        configfile = self.printer.lookup_object("configfile")
        if park is not None:
            self.load_park_mm = park
            configfile.set(self.section, "load_park_mm", f"{park:g}")
        if speed is not None:
            self.load_speed_mm_s = speed
            configfile.set(self.section, "load_speed", f"{speed:g}")
        if auto is not None:
            self.auto_load = bool(auto)
            configfile.set(self.section, "auto_load", "True" if auto else "False")
        assert self.load_set_cmd is not None  # the connect built it, as _require checked
        self.load_set_cmd.send(
            [to_um(self.load_park_mm), to_um(self.load_speed_mm_s), 1 if self.auto_load else 0]
        )
        gcmd.respond_info(self._load_line() + "; staged for SAVE_CONFIG")

    def _load_line(self) -> str:
        return (
            f"ace2k: load park {self.load_park_mm:g} mm, speed {self.load_speed_mm_s:g} mm/s, auto"
            f" {self.auto_load}"
        )

    cmd_ACEK_SNAG_SET_help = (
        "The tip snag's values from now on: FWD=<mm> LAG=<mm> DUTY=<%> BACK=<mm> REST=<ms>"
        " FREE=<mm> (staged for SAVE_CONFIG); alone, the values in effect"
    )

    def cmd_ACEK_SNAG_SET(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        # Read and check every argument before anything changes, as ACEK_LOAD_SET does. A rejected
        # argument leaves the values, the values staged for SAVE_CONFIG, and the unit as they were.
        # A LAG from 1 mm up to 1.5 mm is raised to 1.5 mm, as in printer.cfg, and 1.5 mm is
        # what is staged. So SAVE_CONFIG never writes a value that the next version refuses.
        raised: list[float] = []
        asked, values = self.snag_group.from_gcmd(
            gcmd, self.snag, lambda key, value: raise_old_lag(key, value, raised)
        )
        if not asked:
            gcmd.respond_info(SNAG.line(self.snag))
            return
        if values["snag_lag_mm"] >= values["snag_fwd_mm"]:
            # The unit would treat this as a host bug and shut down.
            raise gcmd.error(
                f"ace2k: LAG ({setting_text(values['snag_lag_mm'])}) must be under FWD"
                f" ({setting_text(values['snag_fwd_mm'])}) mm"
            )
        for lag in raised:
            warning = f"{self.unit.head()} {old_lag_text(lag)}"
            logging.warning("%s", warning)
            gcmd.respond_info(warning)
        self.snag = values
        self._stage(asked)
        SNAG.send(self.mcu, values)
        gcmd.respond_info(SNAG.line(values) + "; staged for SAVE_CONFIG")

    cmd_ACEK_FOLLOW_SET_help = (
        "The follow's values from now on: FLIP_MS=<ms> TAKE_MM=<mm> TAIL_MM=<mm> (staged for"
        " SAVE_CONFIG); alone, the values in effect"
    )

    def cmd_ACEK_FOLLOW_SET(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        if not self.has_follow:
            raise gcmd.error(f"ace2k: {NO_FOLLOW}")
        # Read and check every argument before anything changes, as ACEK_SNAG_SET does. A rejected
        # argument leaves the values, the values staged for SAVE_CONFIG, and the unit as they were.
        asked, values = self.follow_group.from_gcmd(gcmd, self.follow)
        if not asked:
            gcmd.respond_info(FOLLOW.line(self.follow))
            return
        self.follow = values
        self._stage(asked)
        FOLLOW.send(self.mcu, values)
        gcmd.respond_info(FOLLOW.line(values) + "; staged for SAVE_CONFIG")

    cmd_ACEK_FF_SET_help = (
        "The feed-forward's values from now on: CHUNK_MM=<mm> PULSE=<mm/s> (the unit's)"
        " WINDOW_MS=<ms> LEAD_MS=<ms> DEADBAND=<mm/s> (the host's), staged for SAVE_CONFIG;"
        " alone, the values in effect"
    )

    def cmd_ACEK_FF_SET(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        if not self.has_ff:
            raise gcmd.error(f"ace2k: {NO_FF}")
        # Read and check every argument before anything changes, as ACEK_FOLLOW_SET does.
        asked, values = self.ff_group.from_gcmd(gcmd, self.ff_values)
        host = self._ff_host_asked(gcmd)
        if not asked and not host:
            gcmd.respond_info(self._ff_line())
            return
        for attr, (key, value) in host.items():
            if attr == "lead_ms":
                self.ff.set_lead(int(value))  # a new lead restarts the sweep of each extruder
            else:
                setattr(self.ff, attr, value)
            self._stage({key: value})
        if asked:
            self.ff_values = values
            self._stage(asked)
            FF.send(self.mcu, values)
        gcmd.respond_info(self._ff_line() + "; staged for SAVE_CONFIG")
        if gcmd.get("PULSE", None) is not None:
            gcmd.respond_info(f"{self.unit.head()} {ace2k_ff.FF_PULSE_RETIRED}")

    cmd_ACEK_FEED_FORWARD_help = (
        "Switch the feed-forward ON=1 or OFF=1 on LANE=<1-4>, or every lane; alone, the state"
    )

    def cmd_ACEK_FEED_FORWARD(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        if not self.has_ff:
            raise gcmd.error(f"ace2k: {NO_FF}")
        lanes: Sequence[int] = (
            range(LANES) if gcmd.get("LANE", None) is None else [self._lane(gcmd)]
        )
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        off = gcmd.get_int("OFF", 0, minval=0, maxval=1)
        if on and off:
            raise gcmd.error("ace2k: ON=1 or OFF=1, not both")
        if on or off:
            self.ff.set_on(lanes, bool(on))
        gcmd.respond_info(self.ff.switch_line())
        if on and not self.has_follow:
            gcmd.respond_info(
                f"{self.unit.head()} the feed-forward doses only a lane in the follow: {NO_FOLLOW}"
            )
        elif on:
            for line in self.ff.not_following_lines(lanes):
                gcmd.respond_info(line)

    cmd_ACEK_LANE_ACTIVE_help = (
        "Make LANE=<1-4> the active lane of its extruder: the follow stopped on the other lanes"
        " that feed the same extruder, and armed on this one"
    )

    def cmd_ACEK_LANE_ACTIVE(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        if not self.has_follow:
            raise gcmd.error(f"ace2k: {NO_FOLLOW}")
        index = self._lane(gcmd)
        n = index + 1
        lane = self.lanes[index]
        why = self.lane_unavailable(index, None)
        if why == "no_filament":
            raise gcmd.error(f"ace2k: lane {n} has no filament")
        if why == "error":
            raise gcmd.error(f"ace2k: lane {n} is in error ({lane['error']}); ACEK_CLEAR first")
        if self.motion[index].inflight or (
            self.open_seq(index) is not None and not self.follow_running(index)
        ):
            raise gcmd.error(f"ace2k: lane {n} is busy ({lane['mode']})")
        # Stop the other followers of the group, on any unit, and wait until they have ended.
        # Then arm this lane.
        self.unit.units.lane_active(self.unit, index, gcmd, PAUSE_WAIT_S)

    cmd_ACEK_GRIP_help = (
        "Grip on LANE=<1-4>: its next automatic load pulls only MM=<mm> (the firmware's default"
        " when absent), without the tag search; MM=0 clears"
    )

    def cmd_ACEK_GRIP(self, gcmd: GCodeCommand) -> None:
        self._require(gcmd)
        if not self.has_grip:
            raise gcmd.error(f"ace2k: {NO_GRIP}")
        index = self._lane(gcmd)
        assert self.grip_mm is not None  # the connect read the default, because has_grip is set
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
