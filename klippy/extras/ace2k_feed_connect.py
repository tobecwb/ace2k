"""The feed keys of the ``[ace2k]`` section, and the feed setup at every connect.

``FeedConnect`` is the part of ``Ace2kFeed`` that reads the tunable keys of the section. They are
the thresholds, the keys of the snag, the follow, the feed-forward and the held stop, the rate of
the state report, and the encoder scale of each lane. In the config callback of the MCU, it probes
the dictionary of the firmware image for the feed, its limits and its capabilities. It sends the
keys as init commands and then subscribes to the reports.

``Ace2kFeed`` inherits it together with ``FeedMotion``, which provides the handlers that it
subscribes. The attributes that both parts use are declared in ``FeedBase``
(``ace2k_feed_base.py``). The other attributes and methods that this part needs from the feed are
declared at the top of the class.

It imports only ``ace2k_feed_base``, ``ace2k_link``, ``ace2k_feed_proto``,
``ace2k_feed_settings`` and two constants from ``ace2k_feed_motion``. ``ace2k_feed.py`` imports
the names that it still uses from here.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Mapping
from typing import TYPE_CHECKING, Callable, Protocol

try:
    from .ace2k_feed_motion import GONE_REPORTS, HELD_FLOOR_SHARE, NO_FOLLOW
    from .ace2k_feed_proto import (
        BASE_FMT,
        BASE_FMT_FORWARD,
        CLEAR_FMT,
        EVENT_FMT,
        FF_FMT,
        FF_QUERY_FMT,
        FF_STATE_FMT,
        FF_STATE_FMT_FORWARD,
        GRIP_FMT,
        HELD_SET_FMT,
        LOAD_SET_FMT,
        NO_CLEAR,
        NO_FF,
        NO_STOP,
        PARK_FMT,
        REVERSE_CONSTANT,
        SCALE_FMT,
        SEARCH_FMT,
        SPEED_FMT,
        SPEED_RESP,
        START_FMT,
        START_RESP,
        STATE_FMT,
        STOP_FMT,
        STOP_WHEN_FMT,
        STOP_WHEN_RESP,
        UNLOAD_SET_FMT,
        WAIT_FMT,
        firmware_search_mm,
        scale_x10,
        to_um,
    )
    from .ace2k_feed_settings import (
        FF,
        FOLLOW,
        HELD,
        SNAG,
        UNLOAD,
        SettingGroup,
        old_lag_text,
        raise_old_lag,
        read_thresholds,
        setting_text,
        snag_found,
        threshold_defaults,
        threshold_error,
        thresholds_cmd_text,
    )
    from .ace2k_link import (
        LANES,
        NO_FEED,
        bounded_constants,
        constant_or_none,
        optional_command,
        probe_format,
        refuse_format,
        subscribe_response,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_motion import (  # type: ignore[import-not-found, no-redef]
        GONE_REPORTS,
        HELD_FLOOR_SHARE,
        NO_FOLLOW,
    )
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]
        BASE_FMT,
        BASE_FMT_FORWARD,
        CLEAR_FMT,
        EVENT_FMT,
        FF_FMT,
        FF_QUERY_FMT,
        FF_STATE_FMT,
        FF_STATE_FMT_FORWARD,
        GRIP_FMT,
        HELD_SET_FMT,
        LOAD_SET_FMT,
        NO_CLEAR,
        NO_FF,
        NO_STOP,
        PARK_FMT,
        REVERSE_CONSTANT,
        SCALE_FMT,
        SEARCH_FMT,
        SPEED_FMT,
        SPEED_RESP,
        START_FMT,
        START_RESP,
        STATE_FMT,
        STOP_FMT,
        STOP_WHEN_FMT,
        STOP_WHEN_RESP,
        UNLOAD_SET_FMT,
        WAIT_FMT,
        firmware_search_mm,
        scale_x10,
        to_um,
    )
    from ace2k_feed_settings import (  # type: ignore[import-not-found, no-redef]
        FF,
        FOLLOW,
        HELD,
        SNAG,
        UNLOAD,
        SettingGroup,
        old_lag_text,
        raise_old_lag,
        read_thresholds,
        setting_text,
        snag_found,
        threshold_defaults,
        threshold_error,
        thresholds_cmd_text,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANES,
        NO_FEED,
        bounded_constants,
        constant_or_none,
        optional_command,
        probe_format,
        refuse_format,
        subscribe_response,
    )

try:
    from .ace2k_feed_base import FeedBase
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_base import FeedBase  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_klipper_types import Command, Config, Params

    class FeedForwardProto(Protocol):
        """``ace2k_ff.FeedForward``, as far as the line of the feed-forward needs it."""

        def settings_text(self) -> str: ...


DEFAULT_SCALE_MM = 1.2342  # the firmware default, in mm per encoder count
SCALE_TOLERANCE = 0.2  # the firmware accepts a scale within ±20 % of the default
# The grip: the next automatic load of a lane pulls only grip_um, skips the tag search, and ends
# loaded. The default and the limits come from the dictionary, read at connect together with the
# command. An image without them has no grip, and set_lane_grip and ACEK_GRIP say so.
GRIP_CONSTANT = "ACE2K_FEED_GRIP_UM"
NO_GRIP = "the unit's firmware has no grip; flash v0.11.0"
# The park uses the same slot on the unit as the grip. The next automatic load of a lane pulls to
# the parking point and reads the tag on the way. It does no tag search past that point, and ends
# loaded. A grip of 0 clears it, so clearing every grip at connect clears the park too.
# Only an image with the grip can have it (has_park is set from the command at connect).
NO_PARK = "the unit's firmware has no park; flash v0.12.0 or later (0.12.0-rc.1 has none)"
# The wait uses the same slot on the unit as the grip and the park. An insert edge on the lane then
# starts no automatic load, however often the filament goes in. A grip of 0 clears it, so clearing
# every grip at connect clears the wait too. Only an image with the grip can have it.
NO_WAIT = "the unit's firmware has no wait; flash v0.13.0 or later"
# The warnings of the held keys and the unload key on an image without their commands, each with
# the image that has them (see _commands_found). NO_STOP and NO_CLEAR alone are G-code errors too.
NO_HELD_KEYS = f"{NO_STOP}; flash v0.13.0"
NO_CLEAR_KEY = f"{NO_CLEAR}; flash v0.13.0"
# A rollback that forgives every catch past snag_free (free_tip on a rollback).
FREE_ROLLBACK_CONSTANT = "ACE2K_FEED_FREE_ROLLBACK"
# An image that refuses a speed on a following lane, with the reason "following".
FOLLOWING_REFUSAL_CONSTANT = "ACE2K_FEED_REFUSED_FOLLOWING"
FEED_REPORT_HZ = 1.0
# The unit keeps its periodic reports on different ticks only for periods on this grid
# (see "Reports" in docs/protocol.md). So the rate that was asked for is snapped to the grid.
REPORT_GRID_MS = 100
SNAP_EPSILON_HZ = 1e-9  # if the rate moved by more than this, a log line says it was snapped


class FeedConnect(FeedBase):
    """The config keys and the connect step of ``Ace2kFeed``."""

    # More attributes that the feed sets in its constructor: the section, the feed-forward, the
    # assist speed and the load switch.
    section: str
    ff: FeedForwardProto
    assist_speed_mm_s: float
    auto_load: bool
    # The handlers that it subscribes, and the speed check. They come from FeedMotion and the feed.
    _handle_state: Callable[[Params], None]
    _handle_event: Callable[[Params], None]
    _handle_ff_state: Callable[[Params], None]
    speed_out: Callable[[object], str | None]
    # The other commands, built in _build_config with the queries and commands of FeedBase.
    # They are None until then.
    scale_cmd: Command | None
    load_set_cmd: Command | None
    # Set from the keys (_init_*), then from the connect (_build_*).
    thresholds: dict[str, float]
    snag_keys: dict[str, float]
    snag_group: SettingGroup
    follow_keys: dict[str, float]
    follow_group: SettingGroup
    ff_keys: dict[str, float]
    held_keys: dict[str, float]
    unload_keys: dict[str, float]
    unload_clear_mm: float
    has_ff: bool
    has_ff_reverse: bool
    ff_base_fmt: str
    ff_state_fmt: str
    ff_group: SettingGroup
    ff_base_cmd: Command | None
    ff_query_cmd: Command | None
    grip_cmd: Command | None
    park_cmd: Command | None
    wait_cmd: Command | None
    has_grip: bool
    grip_mm: float | None
    grip_min_mm: float | None
    grip_max_mm: float | None
    has_park: bool
    has_wait: bool
    report_period_ms: int
    gone_after_s: float
    scale_low: float
    scale_high: float

    def _init_settings(self, config: Config) -> None:
        """Read the tunable keys of the section: thresholds, snag, follow, feed-forward and held.

        The capabilities of the image start as False. The connect sets them."""
        # The keys are checked at connect, against the defaults of the image (_build_thresholds).
        self.thresholds = read_thresholds(config)
        # The snag keys are rounded to the resolution of the unit here. Their limits come from
        # the dictionary, or from this module for an older image, so they are checked at connect,
        # with the lag against the forward travel.
        raised: list[float] = []
        self.snag_keys = SNAG.from_config(
            config, lambda key, value: raise_old_lag(key, value, raised)
        )
        for lag in raised:
            # Such a value was valid before v0.13.0. The next version refuses it.
            logging.warning("%s %s", self.unit.head(), old_lag_text(lag))
        # The values in effect: the keys, over the firmware defaults (set at connect).
        self.snag: dict[str, float] = {}
        self.snag_group = SNAG  # at connect, a copy with the limits in effect
        # The follow keys are rounded to the resolution of the unit here. Their limits come from
        # the dictionary, so they are checked at connect.
        self.follow_keys = FOLLOW.from_config(config)
        self.has_follow = False  # True when the image has the follow constants (set at connect)
        # The values in effect: the keys, over the firmware defaults.
        self.follow: dict[str, float] = {}
        self.follow_group = FOLLOW  # at connect, a copy with the limits from the dictionary
        # The feed-forward keys work like the follow keys. They are checked at connect against
        # the dictionary. An image without the constants ignores them, and the feed-forward is off.
        self.ff_keys = FF.from_config(config)
        self.has_ff = False  # True when the image has the feed-forward constants (set at connect)
        self.has_ff_reverse = False  # True when the image can take back a long retraction
        self.ff_base_fmt = BASE_FMT_FORWARD
        self.ff_state_fmt = FF_STATE_FMT_FORWARD
        # The values in effect: the keys, over the firmware defaults.
        self.ff_values: dict[str, float] = {}
        self.ff_group = FF  # at connect, a copy with the limits from the dictionary
        # The grip, set at connect: whether the image has it, and its default and limits in mm.
        self.has_grip = False
        self.grip_mm = self.grip_min_mm = self.grip_max_mm = None
        self.has_park = False  # the park, in the slot of the grip (set at connect with the grip)
        self.has_wait = False  # the wait, in the same slot (set at connect with the grip)
        # True when the image reads free_tip on a rollback (set from its constant at connect).
        # Then it forgives every taut trip past snag_free, not only one. Without it, the unit
        # ignores free_tip on a rollback.
        self.has_free_rollback = False
        # True when the image refuses a speed on a following lane (set from its constant at
        # connect). Without it, set_speed refuses such a lane itself.
        self.has_following_refusal = False
        # The held stop's keys are rounded here and checked at connect against the unit's limits.
        # has_stop_when is True when the image can stop a move on the plunger (set at connect).
        self.held_keys = HELD.from_config(config)
        self.has_stop_when = False
        self.held_ms_min = self.held_ms_max = None
        self.held_restart_mm = None
        # The clear of an unload, rounded here and checked at connect against the unit's limits.
        self.unload_keys = UNLOAD.from_config(config)

    def _init_report_rate(self, config: Config) -> None:
        """Read the period of the state report and snap it to the report grid."""
        # 0.05 Hz is a period of 20 s. That is 2.4 × 10⁹ ticks at 120 MHz, which is under the
        # 2^32 ticks (35.8 s) that rest_ticks of the unit can hold. A start before the first
        # report of the session waits two periods plus a margin. So the lowest rate also limits
        # that wait (45 s at 0.05 Hz).
        asked_hz = config.getfloat("feed_report_hz", FEED_REPORT_HZ, minval=0.05, maxval=10.0)
        # 1, 2, 5 and 10 Hz are on the grid. Any other rate moves to the nearest 100 ms period.
        self.report_period_ms = max(
            REPORT_GRID_MS, round(1000.0 / asked_hz / REPORT_GRID_MS) * REPORT_GRID_MS
        )
        self.report_hz = 1000.0 / self.report_period_ms
        # After this time, a motion that the unit reports as idle, with no final event, counts as
        # over. This is the rule of ace2k, and a module that tracks the motions of a lane applies
        # the same rule.
        self.gone_after_s = GONE_REPORTS / self.report_hz
        if abs(self.report_hz - asked_hz) > SNAP_EPSILON_HZ:
            logging.warning(
                "%s feed_report_hz %g snapped to %.3g Hz (a %d ms period on the %d ms grid)",
                self.unit.head(),
                asked_hz,
                self.report_hz,
                self.report_period_ms,
                REPORT_GRID_MS,
            )

    def _init_scales(self, config: Config) -> None:
        """Read the encoder scale key of every lane, within the range that the unit accepts."""
        # The range of scales that the unit accepts. ace2k_lane_scale_set shuts the MCU down for a
        # scale outside it.
        self.scale_low = DEFAULT_SCALE_MM * (1.0 - SCALE_TOLERANCE)
        self.scale_high = DEFAULT_SCALE_MM * (1.0 + SCALE_TOLERANCE)
        self.scales: dict[int, float] = {}
        for lane in range(LANES):
            value = config.getfloat(
                f"lane{lane + 1}_encoder_scale", None, minval=self.scale_low, maxval=self.scale_high
            )
            if value is not None:
                self.scales[lane] = value

    # --- configuration -------------------------------------------------------------------

    def _build_config(self) -> None:
        """Probe the firmware image and send its settings and subscriptions, if it has the feed.

        The thresholds and the scale keys go out as init commands on an image that takes its keys
        that way, and as config commands on an older image (see Ace2k.keys_as_init). The scale of
        every lane goes out again as an init command. So do the load settings, the snag values and
        the report query. Then the reports are subscribed.

        A start command with another format means a firmware from another commit, and the connect
        is refused. A missing start command means a firmware built without the feed, and a log
        line says so."""
        verdict, detail = probe_format(self.mcu, START_FMT)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, START_FMT, (verdict, detail))
        if verdict == "absent":
            logging.warning(
                "%s %s; the feed G-codes are disabled (%s)",
                self.unit.head(),
                NO_FEED,
                detail,
            )
            self.present = False
            return
        # The speed command and its answer are declared next to the start command. An image
        # without the answer is from another commit. It is refused before the feed registers
        # anything.
        for msgformat in (SPEED_FMT, SPEED_RESP):
            verdict, detail = probe_format(self.mcu, msgformat)
            if verdict != "ok":
                raise refuse_format(
                    self.printer, self.section, msgformat, (verdict, detail), START_FMT
                )
        self.present = True
        # Built once. Every start and every ACEK_SPEED reuse them.
        self.start_query = self.mcu.lookup_query_command(START_FMT, START_RESP)
        self.speed_query = self.mcu.lookup_query_command(SPEED_FMT, SPEED_RESP)
        self.speed_min = self.mcu.get_constant_float("ACE2K_LANE_SPEED_MIN_UM_S") / 1000.0
        self.speed_max = self.mcu.get_constant_float("ACE2K_LANE_SPEED_MAX_UM_S") / 1000.0
        self.move_max_mm = self.mcu.get_constant_float("ACE2K_LANE_MOVE_MAX_UM") / 1000.0
        self.landing_mm = self.mcu.get_constant_float("ACE2K_LANE_LANDING_UM") / 1000.0
        self.tail_mm = self.mcu.get_constant_float("ACE2K_FEED_UNLOAD_TAIL_UM") / 1000.0
        # A firmware with the reader binding searches for the tag past the parking point.
        self.has_search = self.mcu.try_lookup_command(SEARCH_FMT) is not None
        if self.search_mm is None:
            self.search_mm = firmware_search_mm(self.mcu)
        for key, speed in (
            ("feed_speed", self.feed_speed_mm_s),
            ("unload_speed", self.unload_speed_mm_s),
            ("load_speed", self.load_speed_mm_s),
            ("assist_speed", self.assist_speed_mm_s),
        ):
            bounds = self.speed_out(speed)
            if bounds:
                raise self.printer.config_error(f"{self.section}: {key} must be within {bounds}")
        if self.load_park_mm > self.move_max_mm:
            raise self.printer.config_error(
                f"{self.section}: load_park_mm must be at most {self.move_max_mm:g} mm"
            )
        # Every forward assist burst takes new base readings for the comparator and chains into
        # the next burst while the filament is taut. A window as long as a burst never closes.
        # A held filament would then be ground burst after burst during an assist. The unit treats
        # such a window as a host bug and shuts down, so it is refused here first.
        burst_mm = self.mcu.get_constant_float("ACE2K_FEED_ASSIST_BURST_UM") / 1000.0
        self._send_scales()
        self._build_thresholds(burst_mm)
        self._build_snag()
        self._build_follow()
        self._build_ff()
        self._build_grip()
        self._build_held()
        self._build_unload()
        self.has_free_rollback = self._has_constant(FREE_ROLLBACK_CONSTANT)
        self.has_following_refusal = self._has_constant(FOLLOWING_REFUSAL_CONSTANT)
        ticks = self.mcu.seconds_to_clock(self.report_period_ms / 1000.0)
        self.mcu.add_config_cmd(f"ace2k_feed_query rest_ticks={ticks}", is_init=True)
        self._subscribe(self._handle_state, STATE_FMT)
        self._subscribe(self._handle_event, EVENT_FMT)
        # The commands that the feed sends, built once with the queries and the subscriptions.
        self.stop_cmd = self.mcu.lookup_command(STOP_FMT)
        self.clear_cmd = self.mcu.lookup_command(CLEAR_FMT)
        self.scale_cmd = self.mcu.lookup_command(SCALE_FMT)
        self.load_set_cmd = self.mcu.lookup_command(LOAD_SET_FMT)
        if self.has_ff:
            # The counters report at the rate of the state report, on their own phase in the unit.
            self.mcu.add_config_cmd(f"ace2k_feed_ff_query rest_ticks={ticks}", is_init=True)
            self._subscribe(self._handle_ff_state, self.ff_state_fmt)

    def _build_thresholds(self, burst_mm: float) -> None:
        """Send the thresholds of the supervisors (see Ace2k.add_key_cmd).

        A missing key takes the default of the image. The values are checked as the unit will see
        them. Both windows must be shorter than ``burst_mm``, the burst of an assist, and the
        allowance must be smaller than the partial window. The unit treats a breach as a host bug
        and shuts down, so it is refused here first as a config error."""
        th = dict(threshold_defaults(self.mcu), **self.thresholds)
        if self.thresholds or self.unit.keys_as_init():
            reason = threshold_error(th, self.thresholds, burst_mm)
            if reason is not None:
                raise self.printer.config_error(f"{self.section}: {reason}")
        self.unit.add_key_cmd(thresholds_cmd_text(th), bool(self.thresholds))

    def _build_snag(self) -> None:
        """Work out the snag limits and values in effect, and send the values.

        The limits come from the dictionary. An older image does not publish them, and the copy
        in this module is used. A log line says which. The keys go over the firmware defaults,
        and each one is checked against its limits, then the lag against the forward travel.
        Then the load settings and all six snag values are sent as init commands."""
        found = snag_found(self.mcu)
        if found is None:
            self.snag_group, defaults = SNAG, SNAG.defaults_of(self.mcu)
            source = "from this module, as the unit's firmware does not publish them"
        else:
            self.snag_group, defaults = SNAG.bounded(found)
            source = "from the unit"
        logging.info(
            "%s snag bounds %s: %s", self.unit.head(), source, self.snag_group.bounds_text()
        )
        self.snag = self._judged(self.snag_group, defaults, self.snag_keys)
        if self.snag["snag_lag_mm"] >= self.snag["snag_fwd_mm"]:
            # The unit treats this as a host bug and shuts down, so it is refused here first.
            raise self.printer.config_error(
                f"{self.section}: snag_lag_mm ({setting_text(self.snag['snag_lag_mm'])}) must be"
                f" under snag_fwd_mm ({setting_text(self.snag['snag_fwd_mm'])}): a lag the watch"
                " never reaches"
            )
        self.mcu.add_config_cmd(self._load_set_cmd(), is_init=True)
        # All six go out at every connect, whether or not keys are set, like the load settings.
        # A config command reaches only an MCU that is not configured yet. After ACEK_SNAG_SET and
        # a RESTART, the unit would keep the live values while the host shows these.
        self.mcu.add_config_cmd(SNAG.cmd_text(self.snag), is_init=True)

    def _judged(
        self, group: SettingGroup, defaults: Mapping[str, float], keys: Mapping[str, float]
    ) -> dict[str, float]:
        """Return the values in effect: the keys, over the defaults from the dictionary.

        Each value must be within its limits. The unit treats a value out of bounds as a host bug
        and shuts down, so it is refused here as a config error."""
        values = dict(defaults, **keys)
        reason = group.judge(values)
        if reason is not None:
            raise self.printer.config_error(f"{self.section}: {reason}")
        return values

    def _build_follow(self) -> None:
        """Read the follow defaults and limits, check the keys, and send the values.

        The values go out as an init command at every connect, like the snag values. So a RESTART
        after ACEK_FOLLOW_SET never leaves the unit on the live values. An image without the
        follow constants has no follow. A follow key is then a config error, and nothing is sent."""
        found = bounded_constants(self.mcu, FOLLOW.constant_scales())
        if found is None:
            self.has_follow = False
            if self.follow_keys:
                key = next(iter(self.follow_keys))
                raise self.printer.config_error(f"{self.section}: {key}: {NO_FOLLOW}")
            return
        self.follow_group, defaults = FOLLOW.bounded(found)
        self.follow = self._judged(self.follow_group, defaults, self.follow_keys)
        self.has_follow = True
        self.mcu.add_config_cmd(FOLLOW.cmd_text(self.follow), is_init=True)

    def _build_ff(self) -> None:
        """Read the feed-forward defaults and limits, check the keys, and send the values.

        The defaults and limits come from the dictionary. The values go out as an init command at
        every connect, like the follow values.

        An image without the constants has no feed-forward. Nothing is sent to it, the keys stay
        unused, and ace2k_ff.py says so on the console when the printer is ready.

        With ACE2K_FEED_FF_REVERSE, the base rate is signed and the state report has a "back"
        field. Without it, the unsigned base rate and the old report are used, and there is no
        reverse. ace2k_ff.py says so when the printer is ready.

        If the constants are present, the commands and the report must have the formats that this
        module knows. Otherwise the firmware is from another commit, and the connect is
        refused."""
        found = bounded_constants(self.mcu, FF.constant_scales())
        if found is None:
            self.has_ff = False
            self.has_ff_reverse = False
            # Reset all three so they stay consistent after a reconnect to another image.
            self.ff_base_fmt = BASE_FMT_FORWARD
            self.ff_state_fmt = FF_STATE_FMT_FORWARD
            logging.info("%s %s", self.unit.head(), NO_FF)
            return
        reverse = self._has_constant(REVERSE_CONSTANT)
        self.ff_base_fmt = BASE_FMT if reverse else BASE_FMT_FORWARD
        self.ff_state_fmt = FF_STATE_FMT if reverse else FF_STATE_FMT_FORWARD
        # A report without the epoch field comes from an older image. The host cannot place its
        # counts in time, so it treats this like an image without a feed-forward.
        verdict, detail = probe_format(self.mcu, self.ff_state_fmt)
        if verdict != "ok":
            if reverse:  # the constant is there but the report has another format: another commit
                raise refuse_format(
                    self.printer, self.section, self.ff_state_fmt, (verdict, detail), START_FMT
                )
            self.has_ff = False
            self.has_ff_reverse = False
            logging.info("%s %s (%s)", self.unit.head(), NO_FF, detail)
            return
        for msgformat in (FF_FMT, self.ff_base_fmt, FF_QUERY_FMT):
            verdict, detail = probe_format(self.mcu, msgformat)
            if verdict != "ok":
                raise refuse_format(
                    self.printer, self.section, msgformat, (verdict, detail), START_FMT
                )
        self.ff_group, defaults = FF.bounded(found)
        self.ff_values = self._judged(self.ff_group, defaults, self.ff_keys)
        self.has_ff = True
        self.has_ff_reverse = reverse
        self.ff_base_cmd = self.mcu.lookup_command(self.ff_base_fmt)
        self.ff_query_cmd = self.mcu.lookup_command(FF_QUERY_FMT)
        self.mcu.add_config_cmd(FF.cmd_text(self.ff_values), is_init=True)

    def _has_constant(self, name: str) -> bool:
        """Return True if the dictionary of the image has the constant ``name``."""
        return constant_or_none(self.mcu, name) is not None

    def _build_grip(self) -> None:
        """Read the grip default and limits from the dictionary, and clear the grip of every lane.

        The clearing goes out as an init command. An image without the constants has no grip.
        Nothing is sent, and set_lane_grip and ACEK_GRIP tell the user to flash. If the constants
        are present, the command must have the format that this module knows. Otherwise the
        firmware is from another commit, and the connect is refused."""
        found = bounded_constants(self.mcu, [(GRIP_CONSTANT, 1000.0)])
        if found is not None:
            grip, low, high = found[GRIP_CONSTANT]
        else:
            self.has_grip = self.has_park = self.has_wait = False  # both use the grip slot
            logging.info("%s %s", self.unit.head(), NO_GRIP)
            return
        verdict, detail = probe_format(self.mcu, GRIP_FMT)
        if verdict != "ok":
            raise refuse_format(self.printer, self.section, GRIP_FMT, (verdict, detail), START_FMT)
        self.grip_mm, self.grip_min_mm, self.grip_max_mm = grip, low, high
        self.has_grip = True
        self.grip_cmd = self.mcu.lookup_command(GRIP_FMT)
        self.park_cmd = optional_command(self.printer, self.mcu, self.section, PARK_FMT, START_FMT)
        self.has_park = self.park_cmd is not None
        if not self.has_park:
            logging.info("%s %s", self.unit.head(), NO_PARK)
        self.wait_cmd = optional_command(self.printer, self.mcu, self.section, WAIT_FMT, START_FMT)
        self.has_wait = self.wait_cmd is not None
        if not self.has_wait:
            logging.info("%s %s", self.unit.head(), NO_WAIT)
        # Clear the grip of every lane at every connect. The grip lives in the RAM of the unit.
        # A host restart that leaves the MCU running (RESTART or SAVE_CONFIG) sends it only the
        # init commands. A grip from the last session must never outlive that session.
        for lane in range(LANES):
            self.mcu.add_config_cmd(f"{GRIP_FMT.split()[0]} lane={lane} grip_um=0", is_init=True)

    def _build_held(self) -> None:
        """Find the stop on the plunger, check the held keys, and send the held values.

        An image with the stop has its two commands, the answer and the ACE2K_FEED_HELD_*
        constants, which give the defaults and the limits. The values go out as an init command at
        every connect, like the follow values. They are never staged for SAVE_CONFIG. A command
        with another format, or the constants without the commands, mean a firmware from another
        commit, and the connect is refused. The commands without the constants are no stop. An
        image without the stop gets nothing, and a held key set for it is ignored with one
        warning line. Values that let friction end a slow move held get one warning line too."""
        found = bounded_constants(self.mcu, HELD.constant_scales())
        formats = (STOP_WHEN_FMT, STOP_WHEN_RESP, HELD_SET_FMT)
        have = self._commands_found(formats, found, self.held_keys, NO_HELD_KEYS)
        if have is None:
            self.has_stop_when = False
            self.held_ms_min = self.held_ms_max = None
            self.held_restart_mm = None
            return
        group, defaults = HELD.bounded(have)
        held = self._judged(group, defaults, self.held_keys)
        # The filament travel that restarts a window, at the widest scale the unit accepts (a
        # calibration can move a lane's scale there). start_move checks each window against it.
        count_um = scale_x10(self.scale_high) // 10
        restart_um = math.ceil(to_um(held["held_mm"]) / count_um) * count_um
        self.held_restart_mm = restart_um / 1000.0
        self._check_held_floor(held["held_mm"], held["held_ms"], restart_um)
        low, high = group.bounds["held_ms"]
        self.held_ms_min, self.held_ms_max = int(low), int(high)
        self.has_stop_when = True
        self.stop_when_query = self.mcu.lookup_query_command(STOP_WHEN_FMT, STOP_WHEN_RESP)
        self.mcu.add_config_cmd(HELD.cmd_text(held), is_init=True)

    def _commands_found(
        self,
        formats: tuple[str, ...],
        found: Mapping[str, tuple[float, float, float]] | None,
        keys: Mapping[str, float],
        missing: str,
    ) -> Mapping[str, tuple[float, float, float]] | None:
        """Return ``found`` when the image has the commands of a group of keys and its constants.

        A command with another format, or the constants (``found``) without a command, mean a
        firmware from another commit, and the connect is refused. Without either, the image lacks
        the group, and the result is None: the keys set for it are ignored with one warning line,
        which says ``missing``."""
        absent = False
        for msgformat in formats:
            verdict, detail = probe_format(self.mcu, msgformat)
            if verdict == "differs" or (verdict == "absent" and found is not None):
                raise refuse_format(
                    self.printer, self.section, msgformat, (verdict, detail), START_FMT
                )
            absent = absent or verdict == "absent"
        if found is not None and not absent:
            return found
        if keys:
            names = ", ".join(keys)
            logging.warning("%s %s ignored: %s", self.unit.head(), names, missing)
        return None

    def _send_scales(self) -> None:
        """Send the scale of every lane.

        An older image takes the scale keys as config commands (see Ace2k.keys_as_init). Every
        lane also gets its scale as an init command, from the key or the default. After a host
        restart the unit is already configured, and it gets only the init commands. So a scale
        that was applied live and not saved, such as one from the automatic calibration, ends
        with the session."""
        if not self.unit.keys_as_init():
            for lane, scale in sorted(self.scales.items()):
                self.mcu.add_config_cmd(self._scale_cmd_text(lane, scale))
        for lane in range(LANES):
            scale = self.lanes[lane]["encoder_scale"]
            self.mcu.add_config_cmd(self._scale_cmd_text(lane, scale), is_init=True)

    def _build_unload(self) -> None:
        """Find the clear of an unload, check its key, and send its value.

        An image with the clear has its command and the ACE2K_FEED_UNLOAD_CLEAR_* constants, which
        give the default and the limits. The value goes out as an init command at every connect,
        like the held values, and is never staged for SAVE_CONFIG. A command with another format,
        or the constants without the command, mean a firmware from another commit, and the connect
        is refused. An image without the clear ends an unload at the bay's sensor. It gets nothing,
        and a key set for it is ignored with one warning line."""
        found = bounded_constants(self.mcu, UNLOAD.constant_scales())
        formats = (UNLOAD_SET_FMT,)
        have = self._commands_found(formats, found, self.unload_keys, NO_CLEAR_KEY)
        if have is None:
            self.unload_clear_mm = 0.0
            return
        group, defaults = UNLOAD.bounded(have)
        values = self._judged(group, defaults, self.unload_keys)
        self.unload_clear_mm = values["unload_clear_mm"]
        self.mcu.add_config_cmd(UNLOAD.cmd_text(values), is_init=True)

    def _check_held_floor(self, held_mm: float, held_ms: float, restart_um: int) -> None:
        """Log one warning when friction alone can end a move at the lane's least speed held.

        A strand that comes at v restarts the held window when v is above the friction floor,
        restart_um / held_ms, where restart_um is ceil(held_um / count) x count at the widest
        scale. This covers a move armed with the unit's held_ms; start_move checks a move's own
        window against that move's speed. The floor must stay under 95 % of
        the lane's least speed, because the strand trails the motor under a spool. The defaults
        do. The values are the user's choice, as a caller may arm only fast moves, so this is a
        warning and not a config error."""
        floor = restart_um / held_ms  # µm/ms is mm/s
        assert self.speed_min is not None  # the connect read it before the stop
        if floor > self.speed_min * HELD_FLOOR_SHARE:
            logging.warning(
                "%s with held_mm %g and held_ms %g, friction can end a move held below %.1f mm/s,"
                " more than %g %% of the lane's least speed (%g mm/s); raise held_ms or lower"
                " held_mm",
                self.unit.head(),
                held_mm,
                held_ms,
                floor,
                HELD_FLOOR_SHARE * 100,
                self.speed_min,
            )

    def _ff_line(self) -> str:
        return f"{FF.line(self.ff_values)}, {self.ff.settings_text()}"

    def _subscribe(self, handler: Callable[[Params], None], msgformat: str) -> None:
        """Subscribe to a report whose format the dictionary has exactly as this module knows it.

        The start command is already known to be present. One source file declares the start, the
        state report and the event. So a report with another format, or a missing report, means a
        firmware from another commit. The config error says which of the two it is. For a missing
        report, it also names the start command that is present."""
        verdict, detail = probe_format(self.mcu, msgformat)
        if verdict != "ok":
            raise refuse_format(self.printer, self.section, msgformat, (verdict, detail), START_FMT)
        subscribe_response(self.mcu, handler, msgformat)

    def _scale_cmd_text(self, lane: int, scale: float) -> str:
        return f"{SCALE_FMT.split()[0]} lane={lane} um_per_count_x10={scale_x10(scale)}"

    def _load_set_cmd(self) -> str:
        return (
            f"{LOAD_SET_FMT.split()[0]} park_um={to_um(self.load_park_mm)}"
            f" speed_um_s={to_um(self.load_speed_mm_s)} auto_load={1 if self.auto_load else 0}"
        )
