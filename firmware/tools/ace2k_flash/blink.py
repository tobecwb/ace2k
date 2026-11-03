"""The flasher's --identify: blink the unit on a port, so the user can tell which unit it is.

The factory firmware blinks with its own LED command, ace2k with ace2k_identify.  Nothing is
written to the unit, and Klipper is never stopped or started.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from . import host  # host.port_holders, looked up at the call: one place for the tests to patch
from .host import klipper_view, same_device
from .klipper import ACE2K_BAUD, KlipperUnit, has_call
from .wire import EXIT_NO_ANSWER, EXIT_NO_BLINK, EXIT_OK, RefusedError, Unit, flash_leds, identify

if TYPE_CHECKING:
    from .base import Deps
    from .wire import Json

ASK, ANSWER = "ace2k_identify", "ace2k_identify_response ms=%u"
IDENTIFY_ANSWER_S = 10.0  # the wait for ace2k's data dictionary, as the link proof waits


def blink(port: str, deps: Deps) -> int:
    """Blink the unit on port and return the exit status (the flasher's module docstring).

    A port that Klipper uses with ace2k, or that another program holds open, is refused before
    anything is sent.
    """
    view = klipper_view(deps.moonraker, port)
    if view.state == "ready" and view.ace2k_version and same_device(view.ace2k_port, port):
        raise RefusedError(f"Klipper is connected to the unit on {port}; nothing sent")
    holders = host.port_holders(port, deps.proc)
    if holders:
        listed = "; ".join(f"pid {p}: {c}" for p, c in holders)
        raise RefusedError(f"{port} is held open ({listed}); nothing sent")
    status = _blink_original(port, deps)
    return _blink_ace2k(port, deps) if status is None else status


def _blink_original(port: str, deps: Deps) -> int | None:
    """Blink a unit on the original protocol.  None: it does not answer there."""
    link = deps.open_link(port)
    try:
        seen = identify(Unit(link, deps.clock))
        if seen is None:
            return None
        if seen.kind != "factory":
            deps.out(f"The unit on {port} is in recovery (its LEDs strobe fast): it cannot blink.")
            return EXIT_NO_BLINK
        if not flash_leds(Unit(link, deps.clock)):
            deps.out(f"The unit on {port} did not answer the blink command: it cannot blink.")
            return EXIT_NO_BLINK
        deps.out(f"The unit on {port} blinks its four LEDs (the factory firmware).")
        return EXIT_OK
    finally:
        link.close()


def _blink_ace2k(port: str, deps: Deps) -> int:
    """Blink a unit that runs ace2k, over Klipper's protocol."""
    link = deps.open_link(port, ACE2K_BAUD)
    try:
        unit = KlipperUnit(link, deps.clock)
        dictionary = unit.dictionary(deps.clock.now() + IDENTIFY_ANSWER_S)
        if dictionary is None:
            deps.out(
                f"Nothing answered on {port}: not the factory firmware, not recovery, not ace2k."
            )
            return EXIT_NO_ANSWER
        problem = _identify_problem(unit, dictionary)
        if problem:
            deps.out(f"The unit on {port} {problem}: it cannot blink.")
            return EXIT_NO_BLINK
        deps.out(
            f"The unit on {port} blinks its four LEDs (ace2k {dictionary.get('version', '?')})."
        )
        return EXIT_OK
    finally:
        link.close()


def _identify_problem(unit: KlipperUnit, dictionary: Json) -> str:
    """Send ace2k_identify.  Return why it failed, or "" when the unit answered."""
    if not has_call(dictionary, ASK, ANSWER):
        return f"runs ace2k {dictionary.get('version', '?')}, older than ace2k_identify"
    if unit.call(dictionary, ASK, ANSWER, again=False) is None:  # one blink, never two
        return "did not answer ace2k_identify"
    return ""
