"""The attributes that the two parts of ``Ace2kFeed`` share, declared once.

``FeedMotion`` (``ace2k_feed_motion.py``) and ``FeedConnect`` (``ace2k_feed_connect.py``) both use
the same attributes. They are the link, the lane status, the speeds, the load settings, the
capabilities of the firmware image, and the limits, queries and commands that the connect builds.
``FeedBase`` declares them for the type checker. Both parts inherit it, so the two lists cannot
drift apart. ``UnitProto`` describes the unit that the feed belongs to, as the parts see it.

This module imports no other ace2k module at run time. Only the class statement runs here. The
feed's constructor and its connect step set the attributes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Protocol

if TYPE_CHECKING:
    from .ace2k_klipper_types import MCU, Command, GCodeCommand, LaneStatus, Printer, QueryCommand
    from .ace2k_units import UnitProto as CoordinatedUnit
    from .ace2k_units import Units

    class UnitProto(CoordinatedUnit, Protocol):
        """The unit (``ace2k.Ace2k``) that the feed belongs to, as the feed sees it."""

        @property
        def mcu(self) -> MCU: ...
        @property
        def units(self) -> Units: ...
        @property
        def lanes(self) -> list[LaneStatus]: ...  # the sensors of each lane
        @property
        def pulled_any(self) -> bool | None: ...  # the shared switch; None until the first report
        def head(self, part: str | None = None) -> str: ...
        def lane_text(self, lane: int) -> str: ...
        def say(self, line: str) -> None: ...
        def keys_as_init(self) -> bool: ...
        def add_key_cmd(self, text: str, has_key: bool) -> None: ...
        def register_command(
            self, name: str, func: Callable[[GCodeCommand], None], desc: str | None = None
        ) -> None: ...
        def set_counters_period(self, seconds: float) -> None: ...
        @property
        def has_motor_travel(self) -> bool: ...
        def motor_watch(self, on: bool) -> bool: ...
        def keep_encoder_continuous(
            self, lane: int, old_scale: float, new_scale: float
        ) -> None: ...


class FeedBase:
    """The attributes that ``Ace2kFeed`` gives to both of its parts."""

    # Set by the feed's constructor: the link, the lane status, the speeds and the load settings.
    printer: Printer
    mcu: MCU
    unit: UnitProto
    lanes: list[LaneStatus]  # the status of each lane
    feed_speed_mm_s: float
    unload_speed_mm_s: float
    load_speed_mm_s: float
    load_park_mm: float
    search_mm: float | None  # from the config key, else the firmware default (set at connect)
    report_hz: float
    # What the firmware image can do. These stay False until the connect step finds them.
    present: bool
    has_follow: bool
    has_free_rollback: bool
    has_following_refusal: bool
    has_stop_when: bool
    # The held window's limits in ms and the filament travel that restarts it (mm), read at
    # connect from an image with the stop. None until then, and on an image without it.
    held_ms_min: int | None
    held_ms_max: int | None
    held_restart_mm: float | None
    has_search: bool
    # Set at connect: the unit's limits, queries and commands. They are None until then.
    start_query: QueryCommand | None
    speed_query: QueryCommand | None
    stop_when_query: QueryCommand | None
    stop_cmd: Command | None
    clear_cmd: Command | None
    speed_min: float | None
    speed_max: float | None
    move_max_mm: float | None
    landing_mm: float | None
    tail_mm: float | None
    unload_clear_mm: float
