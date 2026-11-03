"""The feed settings that the host reads from ``[ace2k]``, checks, sends and stages for SAVE_CONFIG.

They are the values of the tip snag, the follow, the feed-forward and the held stop, and the
thresholds of the supervisors.

A ``SettingGroup`` is a set of values that the unit takes in one command: six for the snag, three
for the follow, and two each for the feed-forward and the held stop. It reads the keys of the
group from printer.cfg and the arguments from the G-code. It rounds each value to the resolution
of the unit and checks it against its limits, the keys of printer.cfg at connect. It then gives
the text and arguments of the command, and the line that the G-code prints. The limits of the
snag, the follow, the feed-forward and the held stop come from the unit's dictionary at connect
(``SettingGroup.bounded``). For the snag, this module also keeps a copy of the firmware limits.
It is used for an older image, which does not publish them.

This module imports only ``ace2k_link`` and ``ace2k_feed_proto``. ``ace2k_feed.py`` re-exports
the tables and helpers, so ``ace2k_feed.X`` still works.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Callable

try:
    from .ace2k_feed_proto import (
        FF_FMT,
        FOLLOW_FMT,
        HELD_SET_FMT,
        SNAG_FMT,
        UNLOAD_SET_FMT,
        to_um,
    )
    from .ace2k_link import bounded_constants, constants_or_none
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_feed_proto import (  # type: ignore[import-not-found, no-redef]
        FF_FMT,
        FOLLOW_FMT,
        HELD_SET_FMT,
        SNAG_FMT,
        UNLOAD_SET_FMT,
        to_um,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        bounded_constants,
        constants_or_none,
    )

if TYPE_CHECKING:
    from .ace2k_klipper_types import MCU, Config, GCodeCommand

# The firmware defaults (see docs/protocol.md). A firmware image that takes its keys as init
# commands gets all three at every connect. Any other image gets one only when its key is set.
THRESHOLD_DEFAULTS = {
    "slip_check_mm": 50.0,
    "slip_allow_mm": 15.0,
    "stall_check_mm": 20.0,
}
# The defaults of the image, read from its dictionary when it names them.
THRESHOLD_DEFAULT_CONSTANTS = {
    "slip_check_mm": "ACE2K_FEED_SLIP_CHECK_UM",
    "slip_allow_mm": "ACE2K_FEED_SLIP_ALLOW_UM",
    "stall_check_mm": "ACE2K_FEED_STALL_CHECK_UM",
}
# The tip snag values. Each entry is: config key, ACEK_SNAG_SET argument, multiplier to the
# unit's unit, and the limits (low, high) in the unit of the key. The limits are those of the
# firmware (ACE2K_FEED_SNAG_*_MIN and _MAX in feed.h, and a host test reads the header). An image
# since v0.13.0 publishes them, and the host reads them at connect (snag_found). These limits
# are used only for an older image, which does not publish them.
# All six go to the unit as an init command at every connect, like the load settings.
SNAG_SETTINGS: tuple[tuple[str, str, int, float, float], ...] = (
    ("snag_fwd_mm", "FWD", 1000, 5.0, 50.0),
    ("snag_lag_mm", "LAG", 1000, 1.5, 20.0),
    ("snag_duty_pct", "DUTY", 1, 0, 100),
    ("snag_back_mm", "BACK", 1000, 2.0, 20.0),
    ("snag_rest_ms", "REST", 1, 100, 2000),
    ("snag_free_mm", "FREE", 1000, 20.0, 200.0),
)
# The firmware defaults, read from its dictionary at connect.
SNAG_DEFAULT_CONSTANTS = {
    "snag_fwd_mm": "ACE2K_FEED_SNAG_FWD_UM",
    "snag_lag_mm": "ACE2K_FEED_SNAG_LAG_UM",
    "snag_duty_pct": "ACE2K_FEED_SNAG_DUTY_PCT",
    "snag_back_mm": "ACE2K_FEED_SNAG_BACK_UM",
    "snag_rest_ms": "ACE2K_FEED_SNAG_REST_MS",
    "snag_free_mm": "ACE2K_FEED_SNAG_FREE_UM",
}
SNAG_INT_KEYS = frozenset({"snag_duty_pct", "snag_rest_ms"})
# The duty has no published least value. Its least value is 0.
SNAG_DUTY_KEY = "snag_duty_pct"
# The least lag was 1 mm before v0.13.0. It is 1.5 mm since then, the low limit of the table. In
# this version, a snag_lag_mm from 1 mm up to 1.5 mm, in printer.cfg or in ACEK_SNAG_SET, is
# raised to 1.5 mm with a warning. The next version refuses it.
SNAG_LAG_KEY = "snag_lag_mm"
SNAG_LAG_OLD_LEAST_MM = 1.0
SNAG_LAG_LEAST_MM = next(
    low for key, _arg, _scale, low, _high in SNAG_SETTINGS if key == SNAG_LAG_KEY
)
# The follow values. Each entry is: config key, ACEK_FOLLOW_SET argument, multiplier to the
# unit's unit, and the dictionary constant of the default. The limits are the constants with the
# same name plus _MIN and _MAX, read at connect. All values go to the unit as an init command at
# every connect, like the snag values. An image that lacks these constants, including the one for
# the tail, has no follow.
FOLLOW_SETTINGS: tuple[tuple[str, str, int, str], ...] = (
    ("follow_flip_ms", "FLIP_MS", 1, "ACE2K_FEED_FOLLOW_FLIP_MS"),
    ("follow_take_mm", "TAKE_MM", 1000, "ACE2K_FEED_FOLLOW_TAKE_UM"),
    ("follow_tail_mm", "TAIL_MM", 1000, "ACE2K_FEED_FOLLOW_TAIL_UM"),
)
FOLLOW_INT_KEYS = frozenset({"follow_flip_ms"})
# The feed-forward values, in the same form as the follow values. Each entry is: config key,
# ACEK_FF_SET argument, multiplier to the unit's unit, and the dictionary constant of the default
# (with _MIN and _MAX for the limits). Both values go to the unit as an init command at every
# connect. An image without the constants has no feed-forward. It stays off, the console says so
# once, and nothing is sent to it.
FF_SETTINGS: tuple[tuple[str, str, int, str], ...] = (
    ("ff_chunk_mm", "CHUNK_MM", 1000, "ACE2K_FEED_FF_CHUNK_UM"),
    ("ff_pulse", "PULSE", 1000, "ACE2K_FEED_FF_PULSE_UM_S"),
)
# The values of the held stop (start_move's stop_when), in the same form as the follow values.
# No G-code sets them, so the argument names are never read, and nothing is staged for
# SAVE_CONFIG. They come only from printer.cfg. Both go to the unit as an init command at every
# connect. An image without the stop does not get them.
HELD_SETTINGS: tuple[tuple[str, str, int, str], ...] = (
    ("held_mm", "HELD_MM", 1000, "ACE2K_FEED_HELD_UM"),
    ("held_ms", "HELD_MS", 1, "ACE2K_FEED_HELD_MS"),
)
# The clear of an unload, in the same form as the held values: past the bay's sensor, the unload
# turns this much more motor. It comes only from printer.cfg, goes to the unit as an init command at
# every connect, and is never staged for SAVE_CONFIG. An image without it does not get it.
UNLOAD_SETTINGS: tuple[tuple[str, str, int, str], ...] = (
    ("unload_clear_mm", "CLEAR_MM", 1000, "ACE2K_FEED_UNLOAD_CLEAR_UM"),
)
# The unit takes every value as a whole number in its own unit (µm, % or ms). So the host rounds a
# value to that resolution when it reads it. The limits, the rule lag < fwd, the staged value and
# the sent value are all judged on the rounded value. The checks in the firmware therefore see
# exactly what the host checked.

# Maps the unit of a command argument (the part of its name after the first "_", as in fwd_um)
# to the unit in which the G-code prints the value.
LINE_UNITS = {"um": "mm", "um_s": "mm/s", "pct": "%", "ms": "ms"}

# Reads one key or argument. It takes (name, whole) and returns the value, or None if it is absent.
Reader = Callable[[str, bool], "float | None"]
# Changes a value once it is read and rounded. It takes (key, value) and returns the value to use.
Adjust = Callable[[str, float], float]


def setting_round(value: float, scale: float, *, whole: bool) -> float:
    """Round a snag or follow value to the resolution of the unit.

    A millimetre key is rounded to a whole µm. A whole-number key (``whole`` is True, as the
    int_keys of the group say) is returned as it is. A value that is not finite (inf or nan, which
    Klipper's float parser accepts) is also returned as it is, so that setting_bound_error can
    refuse it. round() would raise an error on it."""
    if whole or not math.isfinite(value):
        return value
    return round(value * scale) / scale


def setting_bound_error(key: str, value: float, low: float, high: float) -> str | None:  # noqa: ARG001 — the signature the callers already use
    """Return None if a rounded snag or follow value is within its limits, else the reason.

    The reason is written in the unit of the key."""
    if not math.isfinite(value):
        return f"({value}) must be a finite number"
    if low <= value <= high:
        return None
    return f"({setting_text(value)}) must be within {setting_text(low)}..{setting_text(high)}"


def setting_text(value: float) -> str:
    """Write a snag or follow value as it is printed and staged.

    The text is exact to the µm: three decimals at most, without trailing zeros. So SAVE_CONFIG
    writes back the value that is in effect."""
    if isinstance(value, int):
        return str(value)
    return f"{value:.3f}".rstrip("0").rstrip(".")


@dataclass(frozen=True)
class SettingGroup:
    """A set of values that the unit takes in one command.

    ``settings`` holds (config key, G-code argument, multiplier to the unit's unit), in the order
    of the arguments of the command in ``fmt``. The keys in ``int_keys`` are whole numbers.
    ``bounds`` maps a key to its (low, high) in the unit of the key. A group without bounds checks
    nothing. This is the case for the follow and the feed-forward before connect. If a group has
    bounds, a key that is missing from them is an error. ``title`` names the group in its line.
    ``constants`` maps a key to the dictionary constant of its default."""

    settings: Sequence[tuple[str, str, float]]
    fmt: str
    int_keys: frozenset[str]
    bounds: Mapping[str, tuple[float, float]]
    title: str
    constants: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        arguments = len(self.fmt.split()) - 1
        if arguments != len(self.settings):
            raise ValueError(
                f"{self.title}: {len(self.settings)} settings for {arguments} arguments in"
                f" {self.fmt!r}"
            )

    def from_config(self, config: Config, adjust: Adjust | None = None) -> dict[str, float]:
        """Return the keys that printer.cfg sets, each rounded to the resolution of the unit.

        Nothing is checked here. The limits are known only at connect, and the keys are checked
        then (``judge``). ``adjust`` may change a rounded value (see raise_old_lag)."""

        def read(name: str, whole: bool) -> float | None:
            if whole:
                return config.getint(name, None)
            return config.getfloat(name, None)

        return self._asked(read, False, None, adjust)

    def from_gcmd(
        self, gcmd: GCodeCommand, current: Mapping[str, float], adjust: Adjust | None = None
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Read the G-code arguments, round and check each one, and change nothing yet.

        An argument out of bounds is a G-code error. Return (asked, values): the arguments that
        were given, and the values in effect with those arguments applied. ``adjust`` may change
        a rounded value before it is checked (see raise_old_lag)."""

        def read(name: str, whole: bool) -> float | None:
            if whole:
                return gcmd.get_int(name, None)
            return gcmd.get_float(name, None)

        def refuse(text: str) -> Exception:
            return gcmd.error(f"ace2k: {text}")

        asked = self._asked(read, True, refuse, adjust)
        return asked, dict(current, **asked)

    def _asked(
        self,
        read: Reader,
        by_arg: bool,
        refuse: Callable[[str], Exception] | None,
        adjust: Adjust | None,
    ) -> dict[str, float]:
        """Read every value and, when ``refuse`` is given, check each one as it is read.

        The first value out of bounds is refused. The error names it by its argument (if
        ``by_arg``) or by its key."""
        asked = {}
        for key, arg, scale in self.settings:
            name = arg if by_arg else key
            whole = key in self.int_keys
            value = read(name, whole)
            if value is None:
                continue
            # Judge the limits on the value that the unit gets.
            value = setting_round(value, scale, whole=whole)
            if adjust is not None:
                value = adjust(key, value)
            if refuse is not None:
                reason = self._reason(key, value)
                if reason is not None:
                    raise refuse(f"{name} {reason}")
            asked[key] = value
        return asked

    def _reason(self, key: str, value: float) -> str | None:
        if key not in self.bounds:
            raise KeyError(f"{self.title}: no bounds for {key}")  # never accept an unchecked value
        low, high = self.bounds[key]
        return setting_bound_error(key, value, low, high)

    def judge(self, values: Mapping[str, float]) -> str | None:
        """Return None if every value is within its limits.

        Otherwise return the first value that is not, as "<key> <reason>", with the reason from
        setting_bound_error."""
        if not self.bounds:
            return None  # no limits known before connect
        for key, _arg, _scale in self.settings:
            if key in values:
                reason = self._reason(key, values[key])
                if reason is not None:
                    return f"{key} {reason}"
        return None

    def args(self, values: Mapping[str, float]) -> list[int]:
        """Return the arguments of the command: each value as a whole number in the unit's unit."""
        return [int(round(values[key] * scale)) for key, _arg, scale in self.settings]

    def _params(self) -> list[str]:
        return [param.split("=")[0] for param in self.fmt.split()[1:]]

    def cmd_text(self, values: Mapping[str, float]) -> str:
        """Return the command as the text of an init command.

        The text has the command name and each argument, named as in ``fmt``."""
        pairs = " ".join(f"{p}={v}" for p, v in zip(self._params(), self.args(values)))
        return f"{self.fmt.split()[0]} {pairs}"

    def send(self, mcu: MCU, values: Mapping[str, float]) -> None:
        mcu.lookup_command(self.fmt).send(self.args(values))

    def _labelled(self) -> list[tuple[str, str, str]]:
        """Return (label, unit, key) for each value.

        The label and the unit come from the argument name in ``fmt``, such as fwd_um."""
        return [
            (label, LINE_UNITS[unit], key)
            for (label, unit), (key, _arg, _scale) in zip(
                (param.split("_", 1) for param in self._params()), self.settings
            )
        ]

    def line(self, values: Mapping[str, float]) -> str:
        """Return the values as the G-code prints them.

        Each value has the label from the argument name in ``fmt``, and the unit of its key."""
        parts = [
            f"{label} {setting_text(values[key])} {unit}" for label, unit, key in self._labelled()
        ]
        return f"ace2k: {self.title} " + ", ".join(parts)

    def bounds_text(self) -> str:
        """Return the limits as a log line prints them, such as "fwd 5..50 mm, lag 1.5..20 mm"."""
        parts = []
        for label, unit, key in self._labelled():
            low, high = self.bounds[key]
            parts.append(f"{label} {setting_text(low)}..{setting_text(high)} {unit}")
        return ", ".join(parts)

    def constant_scales(self) -> list[tuple[str, float]]:
        """Return (dictionary constant, multiplier) for each value, for bounded_constants."""
        return [(self.constants[key], scale) for key, _arg, scale in self.settings]

    def defaults_of(self, mcu: MCU) -> dict[str, float]:
        """Return the firmware defaults from the dictionary, in the units of the keys.

        There is no check for a missing constant, as with ACE2K_FEED_UNLOAD_TAIL_UM. The host and
        the image are from the same release."""
        return {
            key: mcu.get_constant_float(self.constants[key]) / scale
            for key, _arg, scale in self.settings
        }

    def bounded(
        self, found: Mapping[str, tuple[float, float, float]]
    ) -> tuple[SettingGroup, dict[str, float]]:
        """Return this group with the limits that bounded_constants found, and the defaults.

        The default and the limits of a whole-number key are whole numbers."""
        bounds: dict[str, tuple[float, float]] = {}
        defaults: dict[str, float] = {}
        for key, _arg, _scale in self.settings:
            default, low, high = found[self.constants[key]]
            if key in self.int_keys:
                default, low, high = int(default), int(low), int(high)
            bounds[key] = (low, high)
            defaults[key] = default
        return replace(self, bounds=bounds), defaults


SNAG = SettingGroup(
    tuple((key, arg, scale) for key, arg, scale, _low, _high in SNAG_SETTINGS),
    SNAG_FMT,
    SNAG_INT_KEYS,
    {key: (low, high) for key, _arg, _scale, low, high in SNAG_SETTINGS},
    "snag",
    SNAG_DEFAULT_CONSTANTS,
)
# The limits of the follow, the feed-forward and the held stop come from the dictionary at
# connect (FOLLOW.bounded).
FOLLOW = SettingGroup(
    tuple((key, arg, scale) for key, arg, scale, _name in FOLLOW_SETTINGS),
    FOLLOW_FMT,
    FOLLOW_INT_KEYS,
    {},
    "follow",
    {key: name for key, _arg, _scale, name in FOLLOW_SETTINGS},
)
FF = SettingGroup(
    tuple((key, arg, scale) for key, arg, scale, _name in FF_SETTINGS),
    FF_FMT,
    frozenset(),
    {},
    "feed-forward",
    {key: name for key, _arg, _scale, name in FF_SETTINGS},
)
HELD = SettingGroup(
    tuple((key, arg, scale) for key, arg, scale, _name in HELD_SETTINGS),
    HELD_SET_FMT,
    frozenset({"held_ms"}),
    {},
    "held",
    {key: name for key, _arg, _scale, name in HELD_SETTINGS},
)

UNLOAD = SettingGroup(
    tuple((key, arg, scale) for key, arg, scale, _name in UNLOAD_SETTINGS),
    UNLOAD_SET_FMT,
    frozenset(),
    {},
    "unload",
    {key: name for key, _arg, _scale, name in UNLOAD_SETTINGS},
)


def snag_found(mcu: MCU) -> dict[str, tuple[float, float, float]] | None:
    """Return the snag defaults and limits from the dictionary, for SNAG.bounded.

    The result maps each constant to (default, low, high) in the unit of its key. The duty has
    only a published high limit, and its low limit is 0. The result is None when the image lacks
    any of the constants, so the limits are never a mix of the image and this module."""
    duty_name = SNAG_DEFAULT_CONSTANTS[SNAG_DUTY_KEY]
    scales = {key: scale for key, _arg, scale in SNAG.settings}
    ranged = [(SNAG.constants[key], scale) for key, scale in scales.items() if key != SNAG_DUTY_KEY]
    found = bounded_constants(mcu, ranged)
    duty = constants_or_none(mcu, [duty_name, duty_name + "_MAX"])
    if found is None or duty is None:
        return None
    duty_scale = scales[SNAG_DUTY_KEY]  # in the unit of the key, as bounded_constants gives
    found[duty_name] = (duty[duty_name] / duty_scale, 0, duty[duty_name + "_MAX"] / duty_scale)
    return found


def raise_old_lag(key: str, value: float, raised: list[float]) -> float:
    """Return the value to use for a snag value that was read and rounded.

    A snag_lag_mm from 1 mm up to 1.5 mm gives 1.5 mm, and the value that was asked is added to
    ``raised``, for the warning (old_lag_text). Any other value is returned as it is. A lag under
    1 mm stays as it is, and the limits refuse it."""
    if key != SNAG_LAG_KEY or not SNAG_LAG_OLD_LEAST_MM <= value < SNAG_LAG_LEAST_MM:
        return value
    raised.append(value)
    return SNAG_LAG_LEAST_MM


def old_lag_text(value: float) -> str:
    """Return the warning for a lag that raise_old_lag raised, without the "ace2k:" head."""
    least = setting_text(SNAG_LAG_LEAST_MM)
    return (
        f"{SNAG_LAG_KEY} {setting_text(value)} is under the least lag of {least} mm, so {least} mm"
        " is used. The next version refuses it."
    )


def read_thresholds(config: Config) -> dict[str, float]:
    """Return the supervisor thresholds that printer.cfg sets.

    They are checked at connect against the defaults of the image (see threshold_error)."""
    read: Callable[..., float | None] = config.getfloat
    return {
        key: value
        for key in THRESHOLD_DEFAULTS
        if (value := read(key, None, above=0.0)) is not None
    }


def threshold_defaults(mcu: MCU) -> dict[str, float]:
    """Return the default supervisor thresholds.

    If the dictionary names all three, they are the defaults of the image. Otherwise they are the
    copy in this module. The result is never a mix of the two."""
    found = constants_or_none(mcu, list(THRESHOLD_DEFAULT_CONSTANTS.values()))
    if found is None:
        return dict(THRESHOLD_DEFAULTS)
    return {key: found[name] / 1000.0 for key, name in THRESHOLD_DEFAULT_CONSTANTS.items()}


def threshold_error(
    th: Mapping[str, float], keys: Mapping[str, float], burst_mm: float
) -> str | None:
    """Return None if the thresholds ``th`` are valid for the unit, else the reason.

    Both windows must be shorter than the burst of an assist. The allowance must be smaller than
    the partial window. The unit treats a breach of either rule as a host bug and shuts down.
    A value that no key set (it is not in ``keys``) is named as a default of the image. In that
    case the image is inconsistent, and printer.cfg is not at fault."""

    def named(key: str) -> str:
        if key in keys:
            return f"{key} ({th[key]:g})"
        return f"the image's default {key} ({th[key]:g})"

    for key in ("stall_check_mm", "slip_check_mm"):
        if th[key] >= burst_mm:
            return (
                f"{named(key)} must be under {burst_mm:g} mm, an assist's burst: a longer window"
                " never closes during an assist"
            )
    if th["slip_allow_mm"] >= th["slip_check_mm"]:
        return (
            f"{named('slip_allow_mm')} must be under {named('slip_check_mm')}: the partial slip"
            " check could never trip"
        )
    return None


def thresholds_cmd_text(th: Mapping[str, float]) -> str:
    return (
        f"ace2k_feed_thresholds_set slip_check_um={to_um(th['slip_check_mm'])}"
        f" slip_allow_um={to_um(th['slip_allow_mm'])}"
        f" stall_check_um={to_um(th['stall_check_mm'])}"
    )
