"""ace2k_dryer_print.py: the print mode's starts and stops around a print."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import ace2k
import ace2k_dryer
import ace2k_dryer_mode
import ace2k_dryer_print
from test_ace2k_dryer import build, sent_to, state_frame
from test_ace2k_dryer_mode import (
    MODES_STOP_BOUND,
    RESPONSES_STOP_BOUND,
    STATE,
    STOP_BOUND,
    auto_frame,
    modes,
)

if TYPE_CHECKING:
    import pytest
    from ace2k import Ace2k
    from ace2k_dryer import Ace2kDryer
    from ace2k_dryer_print import PrintWatch
    from test_ace2k_extras import FakeMcu, FakePrinter

DRYER_STATE = "ace2k_dryer_state"
START = "ace2k_dryer_start_bound"


class FakePrintStats:
    def __init__(self) -> None:
        self.state = "standby"

    def get_status(self, _eventtime: float) -> dict[str, object]:
        return {"state": self.state}


def setup(
    options: dict[str, object] | None = None, **frame: int
) -> tuple[Ace2k, FakePrinter, FakeMcu, Ace2kDryer, FakePrintStats, PrintWatch]:
    module, printer, mcu, dryer = modes(options)
    stats = printer.objects["print_stats"] = FakePrintStats()
    mcu.subscriptions[DRYER_STATE](state_frame(state=0))  # idle
    mcu.subscriptions[STATE](auto_frame(mode=2, **frame))
    return module, printer, mcu, dryer, stats, module.dryer_mode.print_watch


def setup_stop_bound(
    **frame: int,
) -> tuple[FakeMcu, FakePrintStats, PrintWatch]:
    """As setup, on an image with the print mode's own stop."""
    module, printer, mcu, _dryer = build(known=MODES_STOP_BOUND, responses=RESPONSES_STOP_BOUND)
    stats = printer.objects["print_stats"] = FakePrintStats()
    mcu.subscriptions[DRYER_STATE](state_frame(state=0))
    mcu.subscriptions[STATE](auto_frame(mode=2, **frame))
    return mcu, stats, module.dryer_mode.print_watch


def flaky_stop(dryer: Ace2kDryer, fails: int) -> list[int]:
    """Make the dryer's stop raise NoAnswerError `fails` times, then work. Returns the calls."""
    real = dryer.stop
    calls: list[int] = []

    def stop() -> None:
        calls.append(1)
        if len(calls) <= fails:
            raise ace2k.NoAnswerError("ace2k: dryer stop: no answer from the unit")
        real()

    dryer.stop = stop  # type: ignore[method-assign]
    return calls


def step(watch: PrintWatch, stats: FakePrintStats, state: str, t: float = 0.0) -> None:
    stats.state = state
    watch.poll(t)


def report(mcu: FakeMcu, **frame: int) -> None:
    mcu.subscriptions[STATE](auto_frame(mode=2, **frame))


def test_a_print_start_with_a_humid_cold_box_starts_a_bound_cycle() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == [[45, 1440]]


def test_a_box_at_the_limit_at_the_print_start_starts_a_cycle() -> None:
    # as the unit's start: an outcome at the limit is a failure, so the limit itself starts
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=400)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == [[45, 1440]]


def test_a_box_just_below_the_limit_at_the_print_start_waits() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=399)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []


def test_the_bench_cycle_time_replaces_the_day() -> None:
    _m, _p, mcu, _d, stats, watch = setup({"bench_dry_cycle_minutes": 3}, cold=1, rh=550)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == [[45, 3]]


def test_a_dry_box_at_the_start_starts_nothing_until_the_hold() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=350)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []
    report(mcu, cold=1, rh=470, above_s=1799)
    watch.poll(1.0)
    assert sent_to(mcu, START) == []
    report(mcu, cold=1, rh=470, above_s=1800)
    watch.poll(2.0)
    assert sent_to(mcu, START) == [[45, 1440]]


def test_a_warm_box_at_the_start_waits_for_the_hold() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=0, rh=550)
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []


def test_the_bench_hold_replaces_1800_s() -> None:
    _m, _p, mcu, _d, stats, watch = setup({"bench_dry_hold_s": 60}, cold=0, rh=550)
    step(watch, stats, "printing")
    report(mcu, cold=0, rh=550, above_s=60)
    watch.poll(1.0)
    assert len(sent_to(mcu, START)) == 1


def test_the_end_of_the_print_stops_its_cycle() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))  # heating
    report(mcu, cold=1, rh=550, bound=1)
    step(watch, stats, "complete", 1.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]


def test_the_end_with_the_cycle_already_over_stops_nothing() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    step(watch, stats, "cancelled", 1.0)  # the dryer is idle
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_a_stop_that_fails_is_logged(caplog: pytest.LogCaptureFixture) -> None:
    _m, _p, mcu, dryer, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, bound=1)
    dryer.absent = lambda _d: "no dryer"  # type: ignore[method-assign]
    with caplog.at_level(logging.WARNING):
        step(watch, stats, "error", 1.0)
    assert "cycle not stopped" in caplog.text


def test_a_pause_is_not_an_end() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    step(watch, stats, "paused", 1.0)
    step(watch, stats, "printing", 2.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == []
    assert len(sent_to(mcu, START)) == 1


def test_a_manual_stop_holds_until_the_print_ends() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    watch.on_dryer_event({"kind": "stopped"})
    report(mcu, cold=1, rh=550, above_s=4000)
    watch.poll(5.0)
    assert len(sent_to(mcu, START)) == 1
    step(watch, stats, "complete", 6.0)
    step(watch, stats, "printing", 7.0)
    assert len(sent_to(mcu, START)) == 2


def test_a_fault_or_a_lost_link_also_holds() -> None:
    for kind in ("fault", "stopped_link"):
        _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
        step(watch, stats, "printing")
        watch.on_dryer_event({"kind": kind})
        report(mcu, cold=1, rh=550, above_s=4000)
        watch.poll(5.0)
        assert len(sent_to(mcu, START)) == 1


def test_a_done_cycle_during_the_print_starts_again() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    watch.on_dryer_event({"kind": "done"})
    watch.poll(10.0)  # the dryer idle again (state frame 0), the first report settled
    assert len(sent_to(mcu, START)) == 2


def test_other_events_change_nothing() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    watch.on_dryer_event({"kind": "vented"})
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    watch.poll(1.0)
    assert len(sent_to(mcu, START)) == 1


def test_events_outside_a_print_are_ignored() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    watch.on_dryer_event({"kind": "done"})
    step(watch, stats, "printing")
    assert len(sent_to(mcu, START)) == 1


def test_the_event_of_this_unit_reaches_the_watcher() -> None:
    module, printer, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    (handler,) = printer.handler_lists["ace2k:dryer"]
    handler({"kind": "stopped", "unit": "elsewhere"})
    assert watch.ours
    handler({"kind": "stopped", "unit": module.name})
    assert not watch.ours
    assert watch.blocked


def test_a_host_restart_mid_print_adopts_the_bound_cycle() -> None:
    _m, _p, mcu, _d, stats, watch = setup(bound=1)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    stats.state = "printing"
    watch.poll(0.0)  # first poll: the print already running, the cycle bound
    assert sent_to(mcu, START) == []
    step(watch, stats, "complete", 1.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]


def test_another_mode_starts_nothing() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    mcu.subscriptions[STATE](auto_frame(mode=1, cold=1, rh=550))
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []


def test_a_running_dryer_is_not_started_over() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []


def test_no_report_yet_starts_nothing() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    watch.mode.state = None
    step(watch, stats, "printing")
    assert sent_to(mcu, START) == []


def test_no_print_stats_is_said_once_and_starts_nothing(caplog: pytest.LogCaptureFixture) -> None:
    _m, printer, mcu, _d, _stats, watch = setup(cold=1, rh=550)
    del printer.objects["print_stats"]
    with caplog.at_level(logging.WARNING):
        watch.poll(0.0)
        watch.poll(1.0)
    assert caplog.text.count("virtual_sdcard") == 1
    assert sent_to(mcu, START) == []


def test_no_print_stats_is_not_said_outside_the_print_mode(
    caplog: pytest.LogCaptureFixture,
) -> None:
    _m, printer, mcu, _d, _stats, watch = setup()
    mcu.subscriptions[STATE](auto_frame(mode=0))
    del printer.objects["print_stats"]
    with caplog.at_level(logging.WARNING):
        watch.poll(0.0)
    assert "virtual_sdcard" not in caplog.text


def test_a_refused_start_is_logged_once_and_tried_again_after_the_hold(
    caplog: pytest.LogCaptureFixture,
) -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    ok = mcu.responses[START]
    mcu.responses[START] = dict(ok, accepted=0, reason=3)
    with caplog.at_level(logging.WARNING):
        step(watch, stats, "printing", 10.0)
        report(mcu, cold=1, rh=550, above_s=1800)
        watch.poll(20.0)  # inside the hold since the refusal
        assert len(sent_to(mcu, START)) == 1
        watch.poll(10.0 + 1800.0)  # after it: refused again, not logged again
    assert len(sent_to(mcu, START)) == 2
    assert caplog.text.count("print cycle not started") == 1
    mcu.responses[START] = ok
    watch.poll(10.0 + 3600.0)
    assert len(sent_to(mcu, START)) == 3
    assert watch.ours


def test_a_manual_retarget_over_the_bound_cycle_is_not_stopped_at_the_end() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, bound=0)  # ACEK_DRY retargeted it: no longer bound, no event
    step(watch, stats, "complete", 1.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_a_mode_change_mid_print_leaves_a_later_manual_cycle_alone() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[STATE](auto_frame(mode=0, bound=0))  # the unit stopped its cycle
    watch.on_dryer_event({"kind": "stopped"})
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))  # a manual start later
    step(watch, stats, "complete", 1.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_a_bound_cycle_with_no_print_is_stopped_once(caplog: pytest.LogCaptureFixture) -> None:
    _m, _p, mcu, _d, stats, watch = setup(bound=1)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    with caplog.at_level(logging.WARNING):
        watch.poll(0.0)  # the host came back in standby, the unit kept the cycle
        watch.poll(1.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]
    assert "no print runs" in caplog.text
    mcu.subscriptions[DRYER_STATE](state_frame(state=0))
    report(mcu, bound=0)
    watch.poll(2.0)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, bound=1)
    watch.poll(3.0)
    assert len(sent_to(mcu, "ace2k_dryer_stop")) == 2


def test_an_orphan_stop_that_fails_is_tried_again_until_the_report_clears() -> None:
    _m, _p, mcu, dryer, stats, watch = setup(bound=1)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    calls = flaky_stop(dryer, 1)
    watch.poll(0.0)  # no answer
    watch.poll(1.0)  # too soon to try again
    assert len(calls) == 1
    watch.poll(5.0)
    assert len(calls) == 2
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]
    watch.poll(9.0)  # the report still shows it: tried again, throttled
    assert len(calls) == 2
    mcu.subscriptions[DRYER_STATE](state_frame(state=3))  # cooldown
    report(mcu, bound=0)
    watch.poll(20.0)
    assert len(calls) == 2


def test_a_stop_at_the_end_that_fails_is_tried_again() -> None:
    _m, _p, mcu, dryer, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, cold=1, rh=550, bound=1)
    calls = flaky_stop(dryer, 1)
    step(watch, stats, "complete", 10.0)  # no answer
    step(watch, stats, "complete", 11.0)
    assert len(calls) == 1
    step(watch, stats, "complete", 15.0)
    assert len(calls) == 2
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]


def test_a_manual_retarget_mid_print_holds_until_the_print_ends() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, cold=1, rh=550, bound=0)
    watch.poll(1.0)  # inside the settle time: the report may predate the start
    assert watch.ours
    watch.poll(10.0)  # running, no longer bound: seen once, the two reports may disagree
    assert watch.ours
    watch.poll(10.5)  # still under 1 s since
    assert watch.ours
    watch.poll(11.0)  # ACEK_DRY retargeted it: running, no longer bound, 1 s on
    assert not watch.ours
    assert watch.blocked
    mcu.subscriptions[DRYER_STATE](state_frame(state=0))  # the manual cycle ended
    report(mcu, cold=1, rh=550, above_s=4000)
    watch.poll(20.0)
    assert len(sent_to(mcu, START)) == 1
    step(watch, stats, "complete", 30.0)
    assert sent_to(mcu, "ace2k_dryer_stop") == []
    step(watch, stats, "printing", 40.0)
    assert len(sent_to(mcu, START)) == 2


def test_a_cycle_that_ends_by_its_time_starts_again_despite_a_stale_state() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))  # heating
    report(mcu, cold=1, rh=550, bound=1)
    watch.poll(6.0)
    # The cycle ends by its time: the modes' report shows it unbound before the dryer's report
    # shows it over.
    report(mcu, cold=1, rh=550, bound=0)
    watch.poll(10.0)
    assert not watch.blocked
    watch.on_dryer_event({"kind": "done"})
    mcu.subscriptions[DRYER_STATE](state_frame(state=0))  # idle
    watch.poll(11.0)
    assert not watch.blocked
    assert len(sent_to(mcu, START)) == 2


def test_a_retarget_seen_once_then_bound_again_starts_its_wait_over() -> None:
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, cold=1, rh=550, bound=0)
    watch.poll(10.0)  # seen once
    report(mcu, cold=1, rh=550, bound=1)
    watch.poll(11.0)  # consistent again: forgotten
    report(mcu, cold=1, rh=550, bound=0)
    watch.poll(12.0)  # seen once more, from here
    assert watch.ours
    watch.poll(13.0)
    assert watch.blocked


def test_a_poll_during_the_start_request_does_nothing() -> None:
    module, _p, mcu, _d, stats, watch = setup(cold=0, rh=550, above_s=1800)
    step(watch, stats, "printing")  # warm: no start at the print's start
    real = module.dryer_mode.start_bound
    calls: list[int] = []

    def start_bound(target_c: int, minutes: int) -> None:
        calls.append(1)
        watch.poll(1.5)  # the reactor runs a report while the request waits for its answer
        real(target_c, minutes)

    module.dryer_mode.start_bound = start_bound  # type: ignore[method-assign]
    watch.poll(1.0)  # the hold is over: it starts
    assert len(calls) == 1
    assert sent_to(mcu, START) == [[45, 1440]]


def test_a_poll_during_the_stop_request_does_nothing() -> None:
    _m, _p, mcu, dryer, stats, watch = setup(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, cold=1, rh=550, bound=1)
    real = dryer.stop
    calls: list[int] = []

    def stop() -> None:
        calls.append(1)
        watch.poll(10.5)  # a report while the stop waits for its answer
        real()

    dryer.stop = stop  # type: ignore[method-assign]
    step(watch, stats, "complete", 10.0)
    assert len(calls) == 1
    assert sent_to(mcu, "ace2k_dryer_stop") == [[]]


# ---- the print mode's own stop: it never ends a cycle the user retargeted -----------------------


def test_the_end_of_the_print_stops_with_the_bound_stop() -> None:
    mcu, stats, watch = setup_stop_bound(cold=1, rh=550)
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))  # heating
    report(mcu, cold=1, rh=550, bound=1)
    step(watch, stats, "complete", 1.0)
    assert sent_to(mcu, STOP_BOUND) == [[]]
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_an_orphan_is_stopped_with_the_bound_stop_and_tried_again() -> None:
    mcu, stats, watch = setup_stop_bound(bound=1)
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    step(watch, stats, "standby", 0.0)
    assert sent_to(mcu, STOP_BOUND) == [[]]
    watch.poll(5.0)  # the report still shows it bound: tried again
    assert sent_to(mcu, STOP_BOUND) == [[], []]
    assert sent_to(mcu, "ace2k_dryer_stop") == []


def test_a_refused_bound_stop_is_logged(caplog: pytest.LogCaptureFixture) -> None:
    mcu, stats, watch = setup_stop_bound(cold=1, rh=550)
    mcu.responses[STOP_BOUND] = {"op": 4, "accepted": 0, "reason": 4}
    step(watch, stats, "printing")
    mcu.subscriptions[DRYER_STATE](state_frame(state=2))
    report(mcu, cold=1, rh=550, bound=1)
    with caplog.at_level(logging.WARNING):
        step(watch, stats, "complete", 1.0)
    assert "cycle not stopped" in caplog.text


def test_a_bound_start_refused_busy_outside_the_print_mode_is_a_refused_start(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # The unit refuses a bound start busy when its mode is not print (a mode change the host has
    # not seen yet): logged once, tried again after the hold, as any refused start.
    _m, _p, mcu, _d, stats, watch = setup(cold=1, rh=550)
    mcu.responses[START] = dict(mcu.responses[START], accepted=0, reason=1)
    with caplog.at_level(logging.WARNING):
        step(watch, stats, "printing", 10.0)
        report(mcu, cold=1, rh=550, above_s=1800)
        watch.poll(20.0)
    assert len(sent_to(mcu, START)) == 1
    assert caplog.text.count("print cycle not started: ace2k: dryer start refused: busy") == 1
    assert not watch.ours


def test_the_mode_tables_live_in_the_dryer_module_and_import_from_both_mode_modules() -> None:
    for module in (ace2k_dryer_mode, ace2k_dryer_print):
        assert module.MODE_NAMES is ace2k_dryer.MODE_NAMES
        assert module.MODE_CODES is ace2k_dryer.MODE_CODES
    assert ace2k_dryer_print.PRINT_MODE == ace2k_dryer_print.MODE_CODES["print"] == 2
