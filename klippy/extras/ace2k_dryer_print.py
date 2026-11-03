"""The dryer's print mode, on the host: a cycle bound to the print.

The unit cannot see a print, so the host watches Klipper's ``print_stats`` and drives the cycle.
In the ``print`` mode:

- At the start of a print, with the dryer idle, the box cold and its RH at or above the limit,
  the host starts a cycle bound to itself (the unit ends it when the host stays gone).
- During the print, when the RH has stayed at or above the limit for the hold (30 min), it starts
  one.
- At the end of the print (``complete``, ``cancelled``, ``error``, ``standby``) it stops the
  cycle that it started. The stop is the unit's ``ace2k_dryer_stop_bound``: the unit stops the
  cycle only if it is still bound, so a report one poll old never ends the user's cycle.
- A cycle that ends by hand, by a lost link or by a fault blocks further starts until the print
  ends. So does a manual start over its cycle, which makes that cycle manual: the host sees it
  running and no longer bound on two polls RETARGET_S apart, because the modes' report and the
  dryer's report come in separately. A cycle that ends by its time starts again (the cycle lasts
  24 h).
- A start that the unit refuses is logged once per print, and tried again after the hold.

A pause is not an end. A host restart in a print adopts a cycle that the unit reports as bound.
A bound cycle that runs with no print is stopped; a stop that fails is tried again every
STOP_RETRY_S until the report shows the cycle unbound or over. A poll that comes while a start or
a stop waits for its answer does nothing.
The mode needs ``[virtual_sdcard]``, which provides ``print_stats``.

The mode module (``ace2k_dryer_mode.py``) owns the watcher and drives ``poll`` from the unit's
1 Hz report, and gives it the dryer's events.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

try:
    from .ace2k_dryer import (
        MODE_CODES,
        MODE_NAMES,  # noqa: F401 — re-exported
        DryerRefused,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_dryer import (  # type: ignore[import-not-found, no-redef]
        MODE_CODES,
        MODE_NAMES,  # noqa: F401 — re-exported
        DryerRefused,
    )

if TYPE_CHECKING:
    from .ace2k_dryer_mode import Ace2kDryerMode

PRINT_MODE = MODE_CODES["print"]
CYCLE_MINUTES = 1440
HOLD_S = 1800
SETTLE_S = 5.0  # after a start, the time that its first state report may take
STOP_RETRY_S = 5.0  # between two tries to stop a bound cycle with no print
RETARGET_S = 1.0  # a retarget is believed when two polls this far apart both show it
ON_STATES = ("printing", "paused")
# The events that end its cycle and block further starts until the print ends.
BLOCKING = ("stopped", "stopped_link", "fault")
ACTIVE = ("starting", "heating")  # the dryer states of a running cycle


class PrintWatch:
    """Starts and stops the print mode's cycle around a print."""

    def __init__(self, mode: Ace2kDryerMode) -> None:
        self.mode = mode
        self.in_print = False
        self.blocked = False  # its cycle ended by hand, link or fault, or retargeted: no start
        self.ours = False  # it started the running cycle, or adopted a bound one
        self.restart_due = False  # its cycle ran out its time: start again
        self.refused_logged = False
        self.warned_no_stats = False
        self.orphan_seen = False
        self.stop_next = 0.0  # the time before which it does not try that stop again
        self.next_try = 0.0  # the time before which it does not start again
        self.settled = 0.0  # the time after which the report shows its cycle
        self.retarget_since: float | None = None  # the poll that first showed a retarget
        self.busy = False  # a start or a stop waits for its answer

    def _hold_s(self) -> int:
        return self.mode.bench.hold_s or HOLD_S

    def _idle(self) -> bool:
        return self.mode.dryer.fields["state"] == "idle"

    def poll(self, eventtime: float) -> None:
        """Look at the print and the last report of the unit. Called on every report.

        A request waits for its answer with the reactor running, so a report can come in then.
        That poll does nothing: it would send the same request again."""
        if self.busy:
            return
        self.busy = True
        try:
            self._poll(eventtime)
        finally:
            self.busy = False

    def _poll(self, eventtime: float) -> None:
        frame = self.mode.state
        if frame is None:
            return
        stats = self.mode.printer.lookup_object("print_stats", None)
        if stats is None:
            if not self.warned_no_stats and frame["mode"] == PRINT_MODE:
                self.warned_no_stats = True
                logging.warning(
                    "%s the print mode needs [virtual_sdcard]: print_stats is missing",
                    self.mode.unit.head("dryer"),
                )
            return
        on = stats.get_status(eventtime)["state"] in ON_STATES
        bound = self._running_bound(frame)
        if not on and not self.in_print:
            self._orphan(bound, eventtime)
        elif on and not self.in_print:
            self._print_started(frame, eventtime)
        elif not on and self.in_print:
            self._print_ended(bound, eventtime)
        elif on:
            self._during(frame, eventtime)

    def _running_bound(self, frame: dict[str, int]) -> bool:
        """True if the unit reports a bound cycle that runs: it belongs to a print."""
        return bool(frame["bound"]) and self.mode.dryer.fields["state"] in ACTIVE

    def _orphan(self, bound: bool, eventtime: float) -> None:
        """Stop a bound cycle when no print runs, until the report shows it stopped.

        A Klipper restart ends the print, but the unit keeps the cycle for the link time."""
        if not bound:
            self.orphan_seen = False
            return
        if self.orphan_seen and eventtime < self.stop_next:
            return
        if not self.orphan_seen:
            self.orphan_seen = True
            logging.warning(
                "%s stopping a bound cycle: no print runs", self.mode.unit.head("dryer")
            )
        self._stop(eventtime)

    def _stop(self, eventtime: float) -> None:
        self.stop_next = eventtime + STOP_RETRY_S  # the report shows the stop after this
        try:
            self.mode.stop_bound()
        except (DryerRefused, RuntimeError) as e:
            logging.warning("%s cycle not stopped: %s", self.mode.unit.head("dryer"), e)

    def _can_start(self, frame: dict[str, int]) -> bool:
        return frame["mode"] == PRINT_MODE and not self.blocked and self._idle()

    def _print_started(self, frame: dict[str, int], eventtime: float) -> None:
        self.in_print = True
        self.blocked = False
        self.retarget_since = None
        self.restart_due = False
        self.refused_logged = False
        self.next_try = 0.0
        if frame["bound"]:
            self.ours = True  # a host restart in a print: the unit still runs its cycle
            self.settled = eventtime
        elif self._can_start(frame) and frame["cold"] and frame["rh"] >= self.mode.limit * 10:
            self._start(eventtime)

    def _during(self, frame: dict[str, int], eventtime: float) -> None:
        if self._retargeted(frame, eventtime):
            # ACEK_DRY over its cycle made it manual: as a manual stop, it holds for this print.
            self.ours = False
            self.blocked = True
        if not self._can_start(frame) or eventtime < self.next_try:
            return
        if self.restart_due or frame["above_s"] >= self._hold_s():
            self._start(eventtime)

    def _retargeted(self, frame: dict[str, int], eventtime: float) -> bool:
        """True if its cycle runs, settled, and the unit no longer reports it bound.

        The modes' report and the dryer's come in separately: a cycle that ends by its time can
        read unbound in one and still running in the other. So it takes two polls RETARGET_S
        apart, with no ``done`` between them (that clears ``ours``)."""
        seen = (
            self.ours
            and eventtime >= self.settled
            and not frame["bound"]
            and self.mode.dryer.fields["state"] in ACTIVE
        )
        if not seen:
            self.retarget_since = None
            return False
        if self.retarget_since is None:
            self.retarget_since = eventtime
        return eventtime - self.retarget_since >= RETARGET_S

    def _print_ended(self, bound: bool, eventtime: float) -> None:
        # The unit's report decides: a manual start over the bound cycle clears `bound` there.
        if bound:
            self._stop(eventtime)
            self.orphan_seen = True  # tried again from _orphan while the report shows it
        self.in_print = False
        self.blocked = False
        self.ours = False
        self.restart_due = False

    def _start(self, eventtime: float) -> None:
        target = self.mode.profile_temp()
        minutes = self.mode.bench_minutes or CYCLE_MINUTES
        try:
            self.mode.start_bound(target, minutes)
        except (DryerRefused, RuntimeError) as e:
            self.next_try = eventtime + self._hold_s()
            if not self.refused_logged:
                self.refused_logged = True
                logging.warning("%s print cycle not started: %s", self.mode.unit.head("dryer"), e)
            return
        self.ours = True
        self.restart_due = False
        self.next_try = eventtime + SETTLE_S  # the dryer reads idle until its next report
        self.settled = eventtime + SETTLE_S

    def on_dryer_event(self, event: dict[str, object]) -> None:
        """The dryer's events: how its cycle ended decides whether the print starts it again."""
        if not (self.in_print and self.ours):
            return
        kind = event.get("kind")
        if kind == "done":
            self.ours = False
            self.restart_due = True
        elif kind in BLOCKING:
            self.ours = False
            self.blocked = True
