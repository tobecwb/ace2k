"""The names and message formats of the unit's feed, shared by the feed modules.

This module holds the mode, event and refusal names that the unit reports. It also holds the
format of every feed command and report, the ``Move`` that a start returns, the conversions to and
from the unit's units, and the version of the host interface.

It imports only ``ace2k_link``. ``ace2k_feed.py`` re-exports every name, so ``ace2k_feed.X`` still
works for the modules and tests that import from there.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, overload

try:
    from .ace2k_link import (
        FeedRefused,  # noqa: F401 — re-exported
        constant_or_none,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported
        FeedRefused,
        constant_or_none,
    )

if TYPE_CHECKING:
    from .ace2k_klipper_types import MCU, Completion, FeedEvent, Reactor

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
    11: "assist_stall",  # reserved and never sent; "tangled" covers this case
    12: "assist_overrun",
    13: "unload_incomplete",
    14: "behind",
    15: "snag",
    16: "tail",  # a notice: no filament at the insert; the follow feeds on the end of it
    17: "tail_out",  # final, no error: the end has left the motor, or the tail limit was reached
    18: "blocked",  # final, no error: a feed or load pull met something that cannot move; lane idle
    19: "held",  # final, no error: an armed move toward the spool found the tip held; lane idle
    20: "at_rest",  # final, no error: an armed move found the plunger at rest; lane idle
    # final, no error state: the insert sensors went stale (the ADC scan stopped); lane idle
    21: "stopped_sensors",
}
ERROR_KINDS = frozenset(range(7, 14))
NOTICE_KINDS = frozenset({14, 15, 16})
REFUSAL_NAMES = {
    0: "ok",
    1: "busy",
    2: "in_error",
    3: "no_filament",
    4: "no_link",
    5: "bounds",
    6: "other_lane",
    7: "no_move",
    8: "following",  # a speed set on a following lane; the follow keeps its own speed
    9: "sensors_stale",  # the insert sensors are stale (the ADC scan stopped): no start
}
ERROR_KIND_NAMES = frozenset(KIND_NAMES[k] for k in ERROR_KINDS)
NOTICE_KIND_NAMES = frozenset(KIND_NAMES[k] for k in NOTICE_KINDS)
# A final kind that leaves the lane idle, not in error, yet ends the move because the unit cannot
# judge it: a waited move fails on it, and the follow is not armed again after it.
STALE_SENSORS_KIND = "stopped_sensors"
# The final kinds that fail a waited move: the errors, and the stale sensors.
FAILED_KIND_NAMES = ERROR_KIND_NAMES | {STALE_SENSORS_KIND}
STALE_SENSORS_TEXT = "insert sensors stale (the ADC scan stopped): lanes stopped, starts refused"
FOLLOW_FMT = "ace2k_feed_follow_set flip_ms=%u take_um=%u tail_um=%u"
GRIP_FMT = "ace2k_feed_lane_grip lane=%c grip_um=%u"
PARK_FMT = "ace2k_feed_lane_park lane=%c"
WAIT_FMT = "ace2k_feed_lane_wait lane=%c"
FF_FMT = "ace2k_feed_ff_set chunk_um=%u pulse_um_s=%u"
FF_STATE_FMT = "ace2k_feed_ff_state doses=%*s back=%*s taut=%*s full=%*s epoch=%*s"
FF_STATE_FMT_FORWARD = "ace2k_feed_ff_state doses=%*s taut=%*s full=%*s epoch=%*s"
FF_QUERY_FMT = "ace2k_feed_ff_query rest_ticks=%u"
SNAG_FMT = "ace2k_feed_snag_set fwd_um=%u lag_um=%u duty_pct=%c back_um=%u rest_ms=%u free_um=%u"
START_FMT = "ace2k_feed_start lane=%c mode=%c length_um=%u speed_um_s=%u seq=%c free_tip=%c"
START_RESP = "ace2k_feed_start_response lane=%c accepted=%c reason=%c"
SPEED_FMT = "ace2k_feed_speed_set lane=%c speed_um_s=%u"
SPEED_RESP = "ace2k_feed_speed_set_response lane=%c accepted=%c reason=%c"
STATE_FMT = "ace2k_feed_state mode=%*s error=%*s speed_um_s=%*s duty_pct=%*s bursts=%*s seq=%*s"
EVENT_FMT = "ace2k_feed_event lane=%c kind=%c mode=%c motor_um=%u filament_um=%i seq=%c"
SCALE_FMT = "ace2k_lane_scale_set lane=%c um_per_count_x10=%u"
SEARCH_FMT = "ace2k_rfid_search_set search_um=%u"
DEFAULT_SEARCH_MM = 750.0  # used only when the firmware image declares no default
LOAD_SET_FMT = "ace2k_feed_load_set park_um=%u speed_um_s=%u auto_load=%c"
STOP_FMT = "ace2k_feed_stop lane=%c"
# A stop on the plunger, armed for the lane's next start (start_move's stop_when), and the values
# of the held stop. An image with the stop has both commands and the ACE2K_FEED_HELD_* constants.
# The arming carries its move's own held window, held_ms (0: the unit's held_ms).
STOP_WHEN_FMT = "ace2k_feed_stop_when lane=%c held=%c at_rest=%c held_ms=%hu"
STOP_WHEN_RESP = "ace2k_feed_stop_when_response lane=%c accepted=%c reason=%c"
HELD_SET_FMT = "ace2k_feed_held_set held_um=%u held_ms=%hu"
STOP_KINDS = ("held", "at_rest")  # in the order of the arguments of STOP_WHEN_FMT
# The clear of an unload: the motor travel past the bay's sensor before the unload ends. An image
# with it has the command and the ACE2K_FEED_UNLOAD_CLEAR_* constants.
UNLOAD_SET_FMT = "ace2k_feed_unload_set clear_um=%u"
NO_CLEAR = "this image does not clear an unload past the bay's sensor"
NO_STOP = "this image cannot stop on the plunger"
CLEAR_FMT = "ace2k_feed_clear lane=%c"
NO_FF = "the unit's firmware has no feed-forward; flash v0.9.0"
# The base rate of the feed-forward. It is signed: positive feeds toward the head, negative takes
# back a long retraction.
BASE_FMT = "ace2k_feed_base lane=%c rate_um_s=%i clear=%c"
# For an image without ACE2K_FEED_FF_REVERSE: the rate is unsigned and never negative.
BASE_FMT_FORWARD = "ace2k_feed_base lane=%c rate_um_s=%u clear=%c"
REVERSE_CONSTANT = "ACE2K_FEED_FF_REVERSE"
# The version of the host interface below. It goes up on any change a caller must check for,
# including a new call or a new mode. docs/protocol.md, "Host interface", lists each version.
API_VERSION = 29


class Move:
    """A move that has started. It is complete when the final event with its sequence arrives."""

    def __init__(
        self,
        lane: int,
        mode: str,
        seq: int,
        completion: Completion,
        reactor: Reactor | None = None,
    ) -> None:
        self.lane = lane
        self.mode = mode
        self.seq = seq
        self._completion = completion
        self._reactor = reactor

    def done(self) -> bool:
        return self._completion.test()

    def result(self) -> FeedEvent | None:
        """Return the final event once the move is done, else None. Never blocks."""
        if not self._completion.test():
            return None
        result: FeedEvent | None = self._completion.wait(0.0, None)
        return result

    def wait(self, timeout_s: float, reactor: Reactor | None = None) -> FeedEvent | None:
        """Wait for the final event, for at most ``timeout_s`` seconds.

        Return the event, or None on timeout. ``reactor`` defaults to the feed's reactor."""
        reactor = reactor if reactor is not None else self._reactor
        assert reactor is not None
        result: FeedEvent | None = self._completion.wait(reactor.monotonic() + timeout_s, None)
        return result


@overload
def firmware_search_mm(mcu: MCU, fallback: float = DEFAULT_SEARCH_MM) -> float: ...
@overload
def firmware_search_mm(mcu: MCU, fallback: None) -> float | None: ...
def firmware_search_mm(mcu: MCU, fallback: float | None = DEFAULT_SEARCH_MM) -> float | None:
    """Return the firmware's default search reach for a load, in mm.

    The value comes from the constant ACE2K_FEED_SEARCH_DEFAULT_UM. If the image does not declare
    it, return ``fallback``."""
    um = constant_or_none(mcu, "ACE2K_FEED_SEARCH_DEFAULT_UM")
    return fallback if um is None else um / 1000.0


def to_mm(um: float) -> float:
    return round(um / 1000.0, 3)


def half_mm(motor_mm: float) -> float:
    """Return half of ``motor_mm``, rounded half up to 0.1 mm.

    A tag search goes out and comes back the same way, so the distance out is half the motor
    travel. The rounding works on whole micrometres, so 1481.1 mm gives 740.6 mm and not 740.5 mm.
    """
    return ((round(motor_mm * 1000.0) + 100) // 200) / 10.0


def to_um(mm: float) -> int:
    return int(round(mm * 1000.0))


def scale_x10(scale: float) -> int:
    """Convert a scale in mm per count to the unit's format, tenths of a µm per count."""
    return int(round(scale * 10000.0))
