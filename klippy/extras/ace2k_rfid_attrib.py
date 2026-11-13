"""Tells the unit which lanes are in use, which tags were set by hand, and how it confirms a tag.

The unit decides alone which lane a tag belongs to when two lanes share one antenna. It follows
the motion of the lane. This part of the host gives it three things.

1. The in-use mask: one bit per lane that a print uses. The unit leaves those lanes alone. The
   caller sets it with ``set_in_use``. Nothing is sent before the first call. After it, the last
   mask is sent again every second, because the unit treats a mask older than 3 s as absent.
2. The lanes set by hand (``set_by_hand``). The host keeps the lanes it wants set by hand. The unit
   announces the lanes it holds a mark on (``ace2k_rfid_by_hand_state``) every second and on any
   change. On each announcement the host sends the mark again for a wanted lane the unit does not
   hold (the unit refuses one right at an insertion, and forgets them all at a reset), and stops
   wanting a lane whose bit went from set to clear: the strand left, and the unit dropped the mark.
   A mark given while the strand goes in is sent though the bay still reads empty. A lane whose bay
   reads empty more than EMPTY_DROP_S after it was wanted is no longer wanted, so a mark never
   lands on a strand put in later. The host clears a mark on the unit only with a forget
   (``forget_by_hand``), which also ends its want. At start-up the host forgets a lane the unit
   holds read with no record (``forget_unless_held``), but never one whose bit is set in the last
   announcement: before the first one, the forget waits for it. A strand swapped while the link
   was down keeps the mark the unit held for the strand before: the unit cannot tell, and the
   host does not check.
3. The numbers of a confirmation, from ``printer.cfg``, sent at every connect and changed at run
   time by ``ACEK_RFID_TUNE``. The host checks them with the same ranges as the unit.

The unit traces each step of a confirmation for the bench (``ace2k_rfid_confirm_trace``), and
each trace becomes a line in klippy.log. The traces the unit could not keep are counted, and the
count becomes a line too.

An image without these commands is left alone: every call here does nothing.

    [ace2k]
    #rfid_confirm_leg_mm: 150     # longest single leg of a confirmation, 10-600 mm
    #rfid_confirm_speed: 9        # speed of the legs, mm/s, within the speed bounds of a lane
    #rfid_passive_min_mm: 300     # least lane travel between a tag's leave and return, 50-1000 mm
    #rfid_confirm_follows: 1      # reversals the tag must follow, 1-3
    #rfid_confirm_window_mm: 30   # most lane travel between the two changes of a reversal, 5-100 mm
    #rfid_passive_max_mm: 950     # most lane travel between a tag's leave and return, above the
                                  # minimum, up to 2000 mm
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Protocol

try:
    from .ace2k_link import LANES, constant_or_none, probe_format, subscribe_response
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        LANES,
        constant_or_none,
        probe_format,
        subscribe_response,
    )

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping, Sequence

    from .ace2k_klipper_types import MCU, Command, Config, GCodeCommand, Params

    class Unit(Protocol):
        """What this part needs of the unit (``ace2k.Ace2k``)."""

        @property
        def mcu(self) -> MCU: ...
        @property
        def lanes(self) -> Sequence[Mapping[str, object]]: ...
        def head(self, part: str | None = None) -> str: ...
        def add_key_cmd(self, text: str, has_key: bool) -> None: ...
        def say(self, line: str) -> None: ...
        def lane_text(self, lane: int) -> str: ...


IN_USE_FMT = "ace2k_rfid_in_use mask=%c"
BY_HAND_FMT = "ace2k_rfid_by_hand lane=%c"
BY_HAND_STATE_FMT = "ace2k_rfid_by_hand_state mask=%c"
CONFIRM_FMT = (
    "ace2k_rfid_confirm_set leg_max_um=%u speed_um_s=%u passive_min_um=%u follows=%c"
    " window_um=%u passive_max_um=%u"
)
REFUSED_FMT = (
    "ace2k_rfid_confirm_set_refused leg_max_um=%u speed_um_s=%u passive_min_um=%u follows=%c"
    " window_um=%u passive_max_um=%u"
)
TRACE_FMT = "ace2k_rfid_confirm_trace lane=%c what=%c travel_um=%i reverse=%c lost=%c"
# The names of a confirmation's traces, by the unit's code (protocol.md).
TRACE_NAMES = {
    0: "start",
    1: "flip",
    2: "wrong_way",
    3: "outside",
    4: "leg_out",
    5: "taken",
    6: "ended",
}
REFRESH_S = 1.0  # the unit takes a mask older than 3 s as no mask
# A wanted mark whose bay still reads empty this long after it was wanted is dropped: no strand
# came for it.
EMPTY_DROP_S = 5.0
LEG_RANGE_MM = (10.0, 600.0)
PASSIVE_RANGE_MM = (50.0, 1000.0)
FOLLOWS_RANGE = (1, 3)
WINDOW_RANGE_MM = (5.0, 100.0)
PASSIVE_MAX_MM = 2000.0
DEFAULT_LEG_MM = 150.0
DEFAULT_SPEED_MM_S = 9.0
DEFAULT_PASSIVE_MM = 300.0
DEFAULT_FOLLOWS = 1
DEFAULT_WINDOW_MM = 30.0
DEFAULT_PASSIVE_MAX_MM = 950.0
SPEED_MIN_CONSTANT = "ACE2K_LANE_SPEED_MIN_UM_S"
SPEED_MAX_CONSTANT = "ACE2K_LANE_SPEED_MAX_UM_S"
TUNE_KEYS = ("LEG", "SPEED", "MIN", "FOLLOWS", "WINDOW", "MAX")
WIRE_NAMES = (
    "leg_max_um",
    "speed_um_s",
    "passive_min_um",
    "follows",
    "window_um",
    "passive_max_um",
)
ALL_LANES_MASK = (1 << LANES) - 1
# The numbers of a confirmation in host units (mm, mm/s, count), in WIRE_NAMES order.
Numbers = tuple[float, float, float, int, float, float]


class TagAttribution:
    """The host side of the tag attribution of one unit."""

    def __init__(self, config: Config, unit: Unit) -> None:
        self.mcu = unit.mcu
        self.lanes = unit.lanes
        self.reactor = config.get_printer().get_reactor()
        self.head = unit.head
        self.lane_text = unit.lane_text
        self.add_key_cmd = unit.add_key_cmd
        self.say = unit.say
        # The unit's ranges; build() checks the speed against the lane's and the two passive bounds
        # together.
        leg = config.getfloat(
            "rfid_confirm_leg_mm", None, minval=LEG_RANGE_MM[0], maxval=LEG_RANGE_MM[1]
        )
        speed = config.getfloat("rfid_confirm_speed", None, above=0.0)
        passive = config.getfloat(
            "rfid_passive_min_mm", None, minval=PASSIVE_RANGE_MM[0], maxval=PASSIVE_RANGE_MM[1]
        )
        follows = config.getint(
            "rfid_confirm_follows", None, minval=FOLLOWS_RANGE[0], maxval=FOLLOWS_RANGE[1]
        )
        window = config.getfloat(
            "rfid_confirm_window_mm", None, minval=WINDOW_RANGE_MM[0], maxval=WINDOW_RANGE_MM[1]
        )
        passive_max = config.getfloat("rfid_passive_max_mm", None, maxval=PASSIVE_MAX_MM)
        for key, value in (
            ("rfid_confirm_leg_mm", leg),
            ("rfid_confirm_speed", speed),
            ("rfid_passive_min_mm", passive),
            ("rfid_confirm_window_mm", window),
            ("rfid_passive_max_mm", passive_max),
        ):
            if value is not None and not math.isfinite(value):  # nan passes Klipper's bounds
                raise config.error(f"{key} must be a finite number")
        self.keyed = any(v is not None for v in (leg, speed, passive, follows, window, passive_max))
        self.leg_mm: float = DEFAULT_LEG_MM if leg is None else leg
        self.speed_mm_s: float = DEFAULT_SPEED_MM_S if speed is None else speed
        self.passive_mm: float = DEFAULT_PASSIVE_MM if passive is None else passive
        self.follows: int = DEFAULT_FOLLOWS if follows is None else follows
        self.window_mm: float = DEFAULT_WINDOW_MM if window is None else window
        self.passive_max_mm: float = DEFAULT_PASSIVE_MAX_MM if passive_max is None else passive_max
        # The last set sent, in wire units, and whether a TUNE sent it. A refusal that matches it
        # makes the view unknown: the unit kept whatever it held before.
        self.sent: tuple[int, ...] = self.wire()
        self.sent_by_tune = False
        self.view_known = True
        self.present = False
        self.mask: int | None = None
        # The lanes the host wants set by hand, each with the time it was wanted, the unit's last
        # announced mask since the connect (None before the first), and the start-up forgets that
        # wait for that first one.
        self.wanted: dict[int, float] = {}
        self.held: int | None = None
        self.waiting_forgets: dict[int, Callable[[], None]] = {}
        self.in_use_cmd: Command | None = None
        self.by_hand_cmd: Command | None = None
        self.confirm_cmd: Command | None = None
        self.timer = self.reactor.register_timer(self._refresh)

    # --- configuration -------------------------------------------------------------------

    def speed_bounds(self) -> tuple[float | None, float | None]:
        """Return the lowest and highest lane speed (mm/s) the image declares, None if unknown."""
        lo = constant_or_none(self.mcu, SPEED_MIN_CONSTANT)
        hi = constant_or_none(self.mcu, SPEED_MAX_CONSTANT)
        return (None if lo is None else lo / 1000.0, None if hi is None else hi / 1000.0)

    def check(self, numbers: Numbers) -> None:
        """Raise ValueError, with the reason, if a value is outside what the unit accepts."""
        leg_mm, speed_mm_s, passive_mm, follows, window_mm, passive_max_mm = numbers
        for name, value, (lo, hi) in (
            ("leg", leg_mm, LEG_RANGE_MM),
            ("passive minimum", passive_mm, PASSIVE_RANGE_MM),
            ("window", window_mm, WINDOW_RANGE_MM),
        ):
            if not lo <= value <= hi:
                raise ValueError(f"{name} {value:g} mm is outside {lo:g}..{hi:g} mm")
        lo_s, hi_s = self.speed_bounds()
        if not math.isfinite(speed_mm_s):
            raise ValueError(f"speed {speed_mm_s:g} mm/s is not a finite number")
        if speed_mm_s <= 0 or (lo_s is not None and speed_mm_s < lo_s):
            raise ValueError(f"speed {speed_mm_s:g} mm/s is below the lane minimum")
        if hi_s is not None and speed_mm_s > hi_s:
            raise ValueError(f"speed {speed_mm_s:g} mm/s is above the lane maximum {hi_s:g}")
        if not FOLLOWS_RANGE[0] <= follows <= FOLLOWS_RANGE[1]:
            raise ValueError(f"follows {follows} is outside 1..3")
        # in wire units, as the unit compares them
        if not (
            math.isfinite(passive_max_mm)
            and round(passive_mm * 1000.0) < round(passive_max_mm * 1000.0) <= PASSIVE_MAX_MM * 1000
        ):
            raise ValueError(
                f"passive maximum {passive_max_mm:g} mm is outside"
                f" {passive_mm:g} (excluded)..{PASSIVE_MAX_MM:g} mm"
            )

    def confirm_text(self) -> str:
        return "ace2k_rfid_confirm_set" + "".join(
            f" {name}={value}" for name, value in zip(WIRE_NAMES, self.wire())
        )

    def build(self) -> None:
        """Look the commands up. Run at connect, on an image with the reader.

        Raises ValueError if the values of printer.cfg are outside the ranges of the unit."""
        self.check(self.values())
        self.in_use_cmd = self.mcu.try_lookup_command(IN_USE_FMT)
        self.by_hand_cmd = self.mcu.try_lookup_command(BY_HAND_FMT)
        self.confirm_cmd = self.mcu.try_lookup_command(CONFIRM_FMT)
        self.present = (
            None not in (self.in_use_cmd, self.by_hand_cmd, self.confirm_cmd)
            and probe_format(self.mcu, BY_HAND_STATE_FMT)[0] == "ok"
        )
        if not self.present:
            logging.info(
                "%s firmware without tag attribution; the in-use mask, the set-by-hand bit and"
                " the confirmation numbers are not sent",
                self.head(),
            )
            return
        self.add_key_cmd(self.confirm_text(), self.keyed)
        self.sent, self.sent_by_tune, self.view_known = self.wire(), False, True
        self.held = None  # the unit may have reset: its next announcement says what it holds
        subscribe_response(self.mcu, self._handle_by_hand_state, BY_HAND_STATE_FMT)
        if self.mask is not None:
            self._arm()  # set before the connect: start the resend now
        if probe_format(self.mcu, REFUSED_FMT)[0] == "ok":
            subscribe_response(self.mcu, self._handle_refused, REFUSED_FMT)
        if probe_format(self.mcu, TRACE_FMT)[0] == "ok":
            subscribe_response(self.mcu, self._handle_trace, TRACE_FMT)

    def _handle_trace(self, params: Params) -> None:
        """Log a step of a confirmation (the serial thread: logging only)."""
        if params["lost"]:
            logging.info("%s %d confirm traces lost", self.head(), params["lost"])
        what = TRACE_NAMES.get(params["what"], str(params["what"]))
        logging.info(
            "%s lane %s confirm %s at %.1f mm (%s)",
            self.head(),
            self.lane_text(params["lane"]),
            what,
            params["travel_um"] / 1000.0,
            "reverse" if params["reverse"] else "forward",
        )

    def values(self) -> Numbers:
        """Return the numbers in force, in host units (mm, mm/s, count), in WIRE_NAMES order."""
        return (
            self.leg_mm,
            self.speed_mm_s,
            self.passive_mm,
            self.follows,
            self.window_mm,
            self.passive_max_mm,
        )

    def wire(self) -> tuple[int, ...]:
        """Return the numbers in wire units (um, um/s, um, count, um, um), in WIRE_NAMES order."""
        return tuple(v if isinstance(v, int) else int(round(v * 1000.0)) for v in self.values())

    def _handle_refused(self, params: Params) -> None:
        echoed = tuple(params[name] for name in WIRE_NAMES)
        if echoed != self.sent:
            logging.warning(
                "%s the unit refused confirmation numbers %s, which are not the last sent %s;"
                " ignored",
                self.head(),
                echoed,
                self.sent,
            )
            return
        self.view_known = False
        named = " ".join(f"{name}={value}" for name, value in zip(WIRE_NAMES, echoed))
        line = (
            f"the unit refused the confirmation numbers {named};"
            " it keeps its previous numbers, which the host does not know"
        )
        logging.warning("%s %s", self.head(), line)
        if self.sent_by_tune:
            self.say(line)

    # --- the API -------------------------------------------------------------------------

    def set_in_use(self, mask: int) -> None:
        """Say which lanes a print uses (bit n = lane n+1). Sent now and every second."""
        if not isinstance(mask, int) or isinstance(mask, bool) or not 0 <= mask <= ALL_LANES_MASK:
            raise ValueError(f"in-use mask {mask!r} is not a 4-bit mask")
        self.mask = mask
        if self.present:
            self._send_mask()
            self._arm()

    def set_by_hand(self, lane_index: int) -> None:
        """Say that the tag of a lane was set by hand. Sent now, and again at each announcement
        of the unit that does not show it, also while the bay still reads empty (see
        _by_hand_state)."""
        if (
            not isinstance(lane_index, int)
            or isinstance(lane_index, bool)
            or not 0 <= lane_index < LANES
        ):
            raise ValueError(f"lane {lane_index!r} is not 0..{LANES - 1}")
        self.wanted[lane_index] = self.reactor.monotonic()
        if self.present:
            self._send_by_hand(lane_index)

    def forget_by_hand(self, lane_index: int) -> None:
        """Stop wanting a lane set by hand: a forget sent to the unit clears the unit's mark."""
        self.wanted.pop(lane_index, None)

    def forget_unless_held(self, lane_index: int, forget: Callable[[], None]) -> None:
        """Run ``forget`` for a lane unless the unit holds it set by hand.

        On an image without tag attribution it runs at once. Before the unit's first announcement
        since the connect, it waits for it."""
        if not self.present:
            forget()
        elif self.held is None:
            self.waiting_forgets[lane_index] = forget
        elif not self.held >> lane_index & 1:
            forget()

    def _handle_by_hand_state(self, params: Params) -> None:
        mask = params["mask"]
        self.reactor.register_async_callback(lambda _e: self._by_hand_state(mask))

    def _by_hand_state(self, mask: int) -> None:
        """Take the unit's announcement of the lanes it holds a mark on.

        A wanted lane whose bit went from set to clear is no longer wanted: its strand left. Nor
        is one whose bay reads empty more than EMPTY_DROP_S after it was wanted: no strand came.
        Any other wanted lane the unit does not hold is sent again. The forgets that waited for
        the first announcement run for the lanes it does not hold."""
        before, self.held = self.held, mask
        now = self.reactor.monotonic()
        for lane, since in sorted(self.wanted.items()):
            if mask >> lane & 1:
                continue
            gone = before is not None and before >> lane & 1
            if gone or (self._empty(lane) and now - since > EMPTY_DROP_S):
                del self.wanted[lane]
            else:
                self._send_by_hand(lane)
        waiting, self.waiting_forgets = self.waiting_forgets, {}
        for lane, forget in sorted(waiting.items()):
            if not mask >> lane & 1:
                forget()

    def _empty(self, lane: int) -> bool:
        return self.lanes[lane].get("insert") is False

    def _arm(self) -> None:
        self.reactor.update_timer(self.timer, self.reactor.monotonic() + REFRESH_S)

    def _send_by_hand(self, lane: int) -> None:
        assert self.by_hand_cmd is not None
        self.by_hand_cmd.send([lane])

    def _send_mask(self) -> None:
        if self.mask is not None:
            assert self.in_use_cmd is not None
            self.in_use_cmd.send([self.mask])

    def _refresh(self, eventtime: float) -> float:
        self._send_mask()
        return eventtime + REFRESH_S

    # --- the G-codes ---------------------------------------------------------------------

    def tune(self, gcmd: GCodeCommand) -> str:
        """Send the confirmation numbers named in the G-code. Return the console reply."""
        if not self.present:
            raise gcmd.error("ace2k: the firmware has no tag confirmation")
        given = {k for k in TUNE_KEYS if k in gcmd.get_command_parameters()}
        if not self.view_known:
            if not given:
                return "ace2k: tag confirmation: nothing sent; the unit refused the last numbers"
            if len(given) < len(TUNE_KEYS):
                raise gcmd.error(
                    "ace2k: the unit refused the last numbers; give LEG, SPEED, MIN, FOLLOWS,"
                    " WINDOW and MAX"
                )
        leg = gcmd.get_float("LEG", self.leg_mm)
        speed = gcmd.get_float("SPEED", self.speed_mm_s)
        passive = gcmd.get_float("MIN", self.passive_mm)
        follows = gcmd.get_int("FOLLOWS", self.follows)
        window = gcmd.get_float("WINDOW", self.window_mm)
        passive_max = gcmd.get_float("MAX", self.passive_max_mm)
        assert leg is not None and speed is not None and passive is not None
        assert follows is not None and window is not None and passive_max is not None
        try:
            self.check((leg, speed, passive, follows, window, passive_max))
        except ValueError as e:
            raise gcmd.error(f"ace2k: {e}") from None
        self.leg_mm, self.speed_mm_s, self.passive_mm = leg, speed, passive
        self.follows, self.window_mm, self.passive_max_mm = follows, window, passive_max
        self.sent, self.sent_by_tune, self.view_known = self.wire(), True, True
        assert self.confirm_cmd is not None
        self.confirm_cmd.send(list(self.sent))
        return (
            f"ace2k: tag confirmation sent: leg {leg:g} mm, speed {speed:g} mm/s,"
            f" passive minimum {passive:g} mm, follows {follows}, window {window:g} mm,"
            f" passive maximum {passive_max:g} mm"
        )
