"""Reads the self-test of the unit on the host: the poll, the re-read after a run, and
``ACEK_HEALTH``.

``UnitHealth`` is the part of ``Ace2k`` that reads the self-test of the firmware
(``ace2k_health_query``). It reads it in three cases.

- Every ``HEALTH_POLL_S`` once Klipper is ready. The first poll comes ``HEALTH_FIRST_POLL_S``
  after that.
- After ``ACEK_HEALTH RUN=1``, until the pass read before the run has changed. With ``CLEAR=1``
  too, the faults are cleared once that read shows the new pass, and read once more.
- When ``ACEK_HEALTH`` itself runs.

Each read also asks for the longest silence of the host on the link (``ace2k_link_gap_query``),
when the image measures it. That is the longest interval between two valid frames since boot or
the last clear, beside the window after which the unit takes the link for lost. A clear of the
faults clears it too, so both count from the same moment. If only that query goes unanswered,
the read keeps its verdict and the silence reads unknown.

The three take turns on one reactor mutex. Klipper finds the response handler of a query by its
name, so two queries in flight would take each other's answer.

If the unit does not answer a poll, the health becomes unknown and the poll is tried again. After
``HEALTH_POLL_FAILURES_MAX`` failures in a row, the poll stops.

``Ace2k`` inherits this class. The attributes it reads from the unit are declared at the top of
the class.

This module imports only ``ace2k_link``. ``ace2k.py`` re-exports the names that other modules
and tests import from there.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Callable, Protocol, TypedDict

try:
    from .ace2k_link import NoAnswerError, ask, probe_optional
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]
        NoAnswerError,
        ask,
        probe_optional,
    )

if TYPE_CHECKING:
    from .ace2k_klipper_types import (
        MCU,
        Command,
        GCodeCommand,
        Mutex,
        Params,
        Printer,
        QueryCommand,
        Reactor,
    )

    class _Head(Protocol):
        """``Ace2k.head``: the start of each line of the unit, up to the colon.

        ``part`` is a module name.
        """

        def __call__(self, part: str | None = None) -> str: ...


HEALTH_QUERY = "ace2k_health_query"
HEALTH_STATE = "ace2k_health_state now=%u latched=%u post_ms=%u reset_cause=%c"
HEALTH_CLEAR = "ace2k_health_clear"
HEALTH_RUN = "ace2k_health_run"
# The longest silence between two valid frames, and the window that takes the link for lost. An
# image from before API_VERSION 23 has neither command.
LINK_GAP_QUERY = "ace2k_link_gap_query"
LINK_GAP_STATE = "ace2k_link_gap_state max_ms=%u lost_ms=%u"
LINK_GAP_CLEAR = "ace2k_link_gap_clear"
# The bits of the self-test, in the order of the firmware (see "Health bits" in docs/protocol.md).
HEALTH_BITS = [
    "ntc_left",
    "ntc_right",
    "chamber",
    "chamber_plausible",
    "reader_a",
    "reader_b",
    "zerocross",
    "mains_hz",
    "cutout",
    "insert1",
    "insert2",
    "insert3",
    "insert4",
    "buffer1",
    "buffer2",
    "buffer3",
    "buffer4",
    "encoder1",
    "encoder2",
    "encoder3",
    "encoder4",
    "fg1",
    "fg2",
    "fg3",
    "fg4",
    "sensors_fresh",
    "vdda",
    "clock",
    "watchdog",
    "config_page",
    "image_crc",
]
RESET_CAUSES = {
    0: "unknown",
    1: "power-on",
    2: "reset pin",
    3: "software",
    4: "watchdog",
    5: "window watchdog",
    6: "low-power",
}
HEALTH_FIRST_POLL_S = 5.0  # after the 2 s boot window of the firmware has run its full evaluation
HEALTH_POLL_S = 10.0
HEALTH_POLL_FAILURES_MAX = 5  # consecutive unanswered polls before the poll gives up
HEALTH_RERUN_POLL_S = 2.5  # after RUN=1: the 2 s window, then the three task steps of the pass
# The re-read can arrive before the new pass is done. The pass takes a window of up to 2010 ms,
# a 50 ms settle, two reader probes of up to 150 ms each, and the transit. While post_ms is still
# the value read before the run, the unit is asked again, up to this many times.
HEALTH_RERUN_RETRIES = 3
HEALTH_RERUN_RETRY_S = 1.0


class Health(TypedDict):
    """The self-test as the unit last answered it (``printer.ace2k.health``).

    ``ok`` is None when there is no verdict. Then ``stale`` is False while the firmware has not
    run its first full pass, and True once a query went unanswered.
    ``link_gap_ms`` is the longest silence of the host on the link since boot or the last clear,
    and ``link_lost_ms`` the silence that the unit takes for a lost link. Both are None before the
    first answer, after a read whose silence query went unanswered (the verdict of that read
    stands), and always on an image that does not measure the silence.
    The keys are read by name, which vulture does not see. The noqa on a key says so.
    """

    ok: bool | None  # noqa: V107
    failing: list[str]  # noqa: V107
    latched: list[str]
    post_ms: int | None  # noqa: V107
    reset_cause: str | None  # noqa: V107
    stale: bool  # noqa: V107
    link_gap_ms: int | None  # noqa: V107
    link_lost_ms: int | None  # noqa: V107


def bit_names(mask: int) -> list[str]:
    return [name for i, name in enumerate(HEALTH_BITS) if mask & (1 << i)]


class UnitHealth:
    """The self-test side of ``Ace2k``."""

    # What the unit provides: its link, its subsystems (set at connect) and its line heads.
    printer: Printer
    mcu: MCU
    reactor: Reactor
    present: set[str]
    section: str
    head: _Head
    own: Callable[[str], str]
    say: Callable[[str], None]  # a console line outside a command

    def _init_health(self) -> None:
        """Set the state of the self-test before any answer. The commands come at connect."""
        self.health = Health(
            ok=None,
            failing=[],
            latched=[],
            post_ms=None,
            reset_cause=None,
            stale=False,
            link_gap_ms=None,
            link_lost_ms=None,
        )
        self.health_poll_failures = 0
        self.health_rerun_post_ms: int | None = None  # the pass read before the last RUN=1
        self.health_rerun_left = 0  # re-queries left while post_ms has not moved
        self.health_rerun_clear = False  # CLEAR=1 came with RUN=1: clear after the re-read
        # Built at connect for a firmware with the self-test (_build_health). None until then.
        self.health_query: QueryCommand | None = None
        self.health_clear_cmd: Command | None = None
        self.health_run_cmd: Command | None = None
        # Built at connect for an image that measures the silence on the link. None otherwise.
        self.link_gap_query: QueryCommand | None = None
        self.link_gap_clear_cmd: Command | None = None

    def _init_health_timers(self) -> None:
        """Create the timer for the re-read after RUN=1, and the mutex for the queries.

        The one timer is armed again on every run. A timer registered for each run would stay in
        the list of the reactor forever once it was done.
        The mutex allows one health query in flight at a time (see ``_query_health``).
        """
        self.health_rerun_timer = self.reactor.register_timer(self._requery_health)
        self.health_mutex: Mutex = self.reactor.mutex()

    def _build_health(self) -> None:
        """Build the query and the commands of the self-test, once, at connect."""
        self.health_query = self.mcu.lookup_query_command(HEALTH_QUERY, HEALTH_STATE)
        self.health_clear_cmd = self.mcu.lookup_command(HEALTH_CLEAR)
        self.health_run_cmd = self.mcu.lookup_command(HEALTH_RUN)
        self._build_link_gap()

    def _build_link_gap(self) -> None:
        """Build the query and the clear of the longest silence, if the image has them.

        An older image has neither, and the silence stays None. Another format of the answer
        means a firmware from another commit, and the connect fails with a config error."""
        if not probe_optional(self.printer, self.mcu, self.section, LINK_GAP_STATE):
            return
        if self.mcu.try_lookup_command(LINK_GAP_QUERY) is None:
            return
        self.link_gap_query = self.mcu.lookup_query_command(LINK_GAP_QUERY, LINK_GAP_STATE)
        self.link_gap_clear_cmd = self.mcu.lookup_command(LINK_GAP_CLEAR)

    def _send_health_clear(self) -> None:
        """Clear the latched faults, and the longest silence with them."""
        assert self.health_clear_cmd is not None  # built at connect, like the run
        self.health_clear_cmd.send()
        if self.link_gap_clear_cmd is not None:
            self.link_gap_clear_cmd.send()

    def _on_ready(self) -> None:
        """Start the self-test poll once everything is up. Do nothing without the subsystem."""
        if "health" in self.present:
            self.reactor.register_timer(
                self._poll_health, self.reactor.monotonic() + HEALTH_FIRST_POLL_S
            )

    def _poll_health(self, eventtime: float) -> float:
        # If the unit does not answer (it is shut down or gone), ``ask`` raises NoAnswerError.
        # The poll must catch it, or the error would take the reactor down.
        # The first failure is logged. The health becomes unknown, not the last good answer
        # (_query_health marks that). The poll goes on, because a unit that is busy elsewhere
        # may answer the next one. After HEALTH_POLL_FAILURES_MAX failures in a row, the poll
        # stops. A success resets the count.
        try:
            self._query_health()
        except NoAnswerError as e:
            self.health_poll_failures += 1
            if self.health_poll_failures >= HEALTH_POLL_FAILURES_MAX:
                logging.warning(
                    "%s health poll stopped after %d failures: %s",
                    self.head(),
                    self.health_poll_failures,
                    e,
                )
                return self.reactor.NEVER
            if self.health_poll_failures == 1:
                logging.warning(
                    "%s health poll failed, retrying every %.0f s: %s",
                    self.head(),
                    HEALTH_POLL_S,
                    e,
                )
            return eventtime + HEALTH_POLL_S
        self.health_poll_failures = 0
        return eventtime + HEALTH_POLL_S

    def _requery_health(self, eventtime: float) -> float:
        # The re-read after RUN=1. The result shows in get_status() and in the next ACEK_HEALTH.
        # If the unit does not answer, the failure is logged and not raised into the reactor.
        # An answer with the same post_ms as before the run is the previous pass, not the new
        # one. The unit is asked again, up to HEALTH_RERUN_RETRIES times, and then a warning is
        # logged.
        # A CLEAR=1 that came with the run is sent only once the re-read shows the new pass. A
        # clear sets latched to the current pass, so sent before the new pass it would keep what
        # the old pass failed. If the re-read gets no answer, or the retries end on the previous
        # pass, nothing is cleared, and the console says so.
        try:
            h = self._query_health()
        except NoAnswerError as e:
            clear = " (faults not cleared)" if self.health_rerun_clear else ""
            logging.warning("%s health re-query after RUN failed%s: %s", self.head(), clear, e)
            self._drop_rerun_clear("the unit did not answer")
            return self.reactor.NEVER
        if h["post_ms"] != self.health_rerun_post_ms:
            self._clear_after_rerun()
            return self.reactor.NEVER
        if self.health_rerun_left > 0:
            self.health_rerun_left -= 1
            return eventtime + HEALTH_RERUN_RETRY_S
        logging.warning(
            "%s the self-test still reports the previous pass (post_ms=%s) %.1f s after "
            "RUN; the unit did not run a new one",
            self.head(),
            h["post_ms"],
            HEALTH_RERUN_POLL_S + HEALTH_RERUN_RETRIES * HEALTH_RERUN_RETRY_S,
        )
        self._drop_rerun_clear("the unit did not report a new self-test")
        return self.reactor.NEVER

    def _drop_rerun_clear(self, reason: str) -> None:
        """Drop the CLEAR=1 that came with the run, and say so on the console."""
        if not self.health_rerun_clear:
            return
        self.health_rerun_clear = False
        self.say(f"{self.head()} health not cleared: {reason}")

    def _clear_after_rerun(self) -> None:
        """Send the CLEAR=1 that came with the run, then read the faults it left latched."""
        if not self.health_rerun_clear:
            return
        self.health_rerun_clear = False
        self._send_health_clear()
        try:
            self._query_health()
        except NoAnswerError as e:
            logging.warning("%s health re-query after CLEAR failed: %s", self.head(), e)
            self.say(f"{self.head()} health cleared, but the unit did not answer the read after it")

    def _health_unknown(self) -> None:
        """Mark the health as unknown after an unanswered query.

        What the last answer said about the latched faults and the reset stays, marked stale.
        """
        self.health = {**self.health, "ok": None, "failing": [], "stale": True}

    def _query_health(self) -> Health:
        """Ask the unit for the self-test and store the answer.

        Only one query is in flight at a time. Klipper finds the response handler by the name of
        the response. So two overlapping queries (the poll, the re-read after RUN, the G-code)
        would take each other's answer, and one would time out. The callers queue on the mutex.

        If the query fails, ``ask`` raises NoAnswerError and the health becomes unknown. No answer
        can arrive meanwhile, so a caller queued behind a slow one never wipes a fresh verdict.
        The longest silence is asked on its own: if only it goes unanswered, the verdict stands,
        the silence reads None (unknown, not the last value, which a clear may have made wrong)
        and nothing raises.
        """
        # Every caller runs on a firmware with the self-test, and its connect built the query.
        assert self.health_query is not None
        with self.health_mutex:
            try:
                params: Params = ask(self.printer, self.mcu, self.health_query, (), "health")
            except NoAnswerError:
                self._health_unknown()
                raise
            gap = self._query_link_gap()
            # post_ms is the tick of the last full evaluation of the firmware. 0 means none has run
            # yet (the first runs 2 s after boot), so `now` says nothing: ok is None and failing
            # is [].
            evaluated = params["post_ms"] != 0
            self.health = Health(
                ok=params["now"] == 0 if evaluated else None,
                failing=bit_names(params["now"]) if evaluated else [],
                latched=bit_names(params["latched"]),
                post_ms=params["post_ms"],
                reset_cause=RESET_CAUSES.get(params["reset_cause"], "unknown"),
                stale=False,
                link_gap_ms=None if gap is None else gap["max_ms"],
                link_lost_ms=None if gap is None else gap["lost_ms"],
            )
            return self.health

    def _query_link_gap(self) -> Params | None:
        """Ask for the longest silence, or return None without the query or without an answer."""
        if self.link_gap_query is None:
            return None
        try:
            return ask(self.printer, self.mcu, self.link_gap_query, (), "link gap")
        except NoAnswerError as e:
            logging.warning("%s %s", self.head(), e)
            return None

    cmd_ACEK_HEALTH_help = (
        "Report the self-test (RUN=1 re-runs it, CLEAR=1 clears the latched faults)"
    )

    def cmd_ACEK_HEALTH(self, gcmd: GCodeCommand) -> None:
        if "health" not in self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_HEALTH")
        # With "health" present, the connect built the commands (_build_health).
        assert self.health_clear_cmd is not None and self.health_run_cmd is not None
        clear = bool(gcmd.get_int("CLEAR", 0))
        run = bool(gcmd.get_int("RUN", 0))
        # With RUN=1, the clear waits for the re-read of the new pass (_requery_health).
        if clear and not run:
            self._send_health_clear()
        if run:
            self._start_rerun(gcmd, clear)
            return
        try:
            h = self._query_health()
        except NoAnswerError:
            # The G-code reports the state and does not fail. The unit did not answer, and the
            # query marked the health unknown.
            h = self.health
        if h["ok"] is None:
            verdict = "unknown (no answer from the unit)" if h["stale"] else "not run yet"
        elif h["ok"]:
            verdict = "OK"
        else:
            verdict = "FAILING " + ", ".join(h["failing"])
        latched = ", ".join(h["latched"]) or "none"
        when = "" if h["ok"] is None else f"; self-test at {h['post_ms']} ms"
        lines = [
            self.own(f"health: {verdict}"),
            f"failed since boot or last clear: {latched}",
            f"last reset: {h['reset_cause']}{when}",
            self._link_gap_line(h),
        ]
        gcmd.respond_info("\n".join(lines))

    def _start_rerun(self, gcmd: GCodeCommand, clear: bool) -> None:
        """Start a new self-test pass (ACEK_HEALTH RUN=1) and arm its re-read.

        ``clear`` is CLEAR=1: the counters are cleared once the new pass is read. Raises the
        G-code error when the unit does not answer the read of the current pass."""
        assert self.health_run_cmd is not None  # the caller checked "health" is present
        # The re-read knows the new pass by a post_ms that has changed. So the pass read so far
        # is the one to compare with. If none was read yet (before the first poll, or after
        # the poll gave up), read the current pass first. Without it, the re-read would take
        # the boot pass for the new one. A unit that does not answer gets no run.
        if self.health["post_ms"] is None:
            try:
                self._query_health()
            except NoAnswerError as e:
                raise gcmd.error(f"{e}; self-test not started") from None
        self.health_rerun_post_ms = self.health["post_ms"]
        self.health_rerun_left = HEALTH_RERUN_RETRIES
        # A run that arrives while a clear waits keeps it: the clear was asked for.
        self.health_rerun_clear = self.health_rerun_clear or clear
        self.health_run_cmd.send()
        # Re-read when the pass is done: after the 2 s window, then the probe steps.
        self.reactor.update_timer(
            self.health_rerun_timer, self.reactor.monotonic() + HEALTH_RERUN_POLL_S
        )
        gcmd.respond_info("ace2k: self-test started; results in about 2 s")

    def _link_gap_line(self, h: Health) -> str:
        """The longest silence of the host, beside the window that would take the link for lost."""
        if h["link_gap_ms"] is None:
            if self.link_gap_query is None:
                return "longest host silence: n/a (the unit's firmware does not measure it)"
            return "longest host silence: unknown (no answer from the unit)"
        return (
            f"longest host silence since boot or last clear: {h['link_gap_ms']} ms"
            f" (the link is lost at {h['link_lost_ms']} ms)"
        )
