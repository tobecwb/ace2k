"""ace2k_flash end to end against the simulated unit and host: every path, every refusal."""

from __future__ import annotations

import json
import os
import shlex
import subprocess
import zlib
from pathlib import Path
from typing import TYPE_CHECKING, Literal, NoReturn

import pytest
from flash_sim import (
    PAIRS,
    UID_TEXT,
    SimHost,
    SimUnit,
    ace2k_image,
    args,
    factory_image,
    fl,
    reply,
    setup,
)

if TYPE_CHECKING:
    from collections.abc import Callable
    from types import SimpleNamespace

    from flash_sim import FakeClock


def run(host: SimHost, a: SimpleNamespace) -> int:
    return fl.Flasher(a, host.deps()).run()


def patch_holders(
    monkeypatch: pytest.MonkeyPatch, holders: Callable[[str, Path], list[tuple[int, str]]]
) -> None:
    """Replace port_holders for every flasher module: each calls it through the host module."""
    monkeypatch.setattr(fl.host, "port_holders", holders)


def test_path_a_ace2k_to_ace2k(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.gcodes == ["ACEK_RESTORE_STOCK"]
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert host.running and unit.state == "ace2k" and unit.ace2k_version == "0.12.0"
    assert unit.received.count(fl.CMD_GET_INFO) >= 2  # the first query was lost to the noise
    assert "Klipper reports ace2k 0.12.0: done." in host.lines
    assert unit.closed


def test_a_new_host_module_is_asked_the_new_name_only(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.help_reads == 1 and host.gcodes == ["ACEK_RESTORE_STOCK"]
    assert not any("Unknown command" in m for m in host.console)


def test_an_old_host_module_is_sent_the_old_name_only(tmp_path: Path) -> None:
    # a host module up to v0.12.0-rc.1 lists ACE_RESTORE_STOCK only: nothing else is sent
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.host_module = "old"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.gcodes == ["ACE_RESTORE_STOCK"]
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.state == "ace2k" and unit.ace2k_version == "0.12.0"
    assert not any("Unknown command" in m for m in host.console)


def test_an_old_host_modules_refusal_names_the_old_command(tmp_path: Path) -> None:
    # the RC printed its refusal on the console alone
    clock, unit, host, image = setup(tmp_path)
    host.host_module, host.refuse_restore, host.refuse_form = "old", True, "console"
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.gcodes == ["ACE_RESTORE_STOCK"]
    assert "ACE_RESTORE_STOCK was refused: !! ace2k: refused" in host.text()
    assert host.commands == [] and host.running and unit.state == "ace2k"


def test_neither_name_listed_stops_at_once_and_sends_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.host_module = "none"
    start = clock.now()
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert clock.now() - start < fl.STATE_POLL_S
    assert host.gcodes == []
    assert "Klipper knows neither ACEK_RESTORE_STOCK nor ACE_RESTORE_STOCK" in host.text()
    assert "Nothing changed" in host.text()
    assert host.commands == [] and host.running and unit.state == "ace2k"


def test_an_unread_command_list_sends_the_new_name_and_goes_on(tmp_path: Path) -> None:
    # the help endpoint does not answer: the new name is sent unchecked, and on a new host the
    # flash goes on as before
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.help_unanswered = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.gcodes == ["ACEK_RESTORE_STOCK"]


def test_an_unread_command_list_and_console_on_an_old_host_says_so_in_the_end(
    tmp_path: Path,
) -> None:
    # the new name unknown to an old host, and no console to see it in: a script that succeeded,
    # no reset — the wait runs out, and the message says the list could not be read
    clock, unit, host, image = setup(tmp_path)
    host.host_module, host.help_unanswered, host.console_unanswered = "old", True, True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.gcodes == ["ACEK_RESTORE_STOCK"]
    text = host.text()
    assert "no refusal was printed" in text
    assert "Klipper's command list could not be read" in text
    assert "ACE_RESTORE_STOCK" in text  # the old name named for the operator to try
    assert host.commands == [] and host.running and unit.state == "ace2k"


def plan_step(host: SimHost) -> str:
    return next(line for line in host.lines if "the unit resets into the bootloader's" in line)


def test_the_plan_names_the_new_command_on_a_new_host(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert (
        plan_step(host) == "  - ACEK_RESTORE_STOCK (the unit resets into the bootloader's recovery)"
    )


def test_the_plan_names_the_old_command_on_an_old_host(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.host_module = "old"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert (
        plan_step(host) == "  - ACE_RESTORE_STOCK (the unit resets into the bootloader's recovery)"
    )


def test_the_plan_says_unchecked_without_a_command_list(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.help_unanswered = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert plan_step(host) == (
        "  - ACEK_RESTORE_STOCK, unchecked: the command list could not be read (the unit resets"
        " into the bootloader's recovery)"
    )


def test_neither_name_is_refused_before_the_plan_is_confirmed(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.host_module = "none"
    host.answers = []  # a question asked would raise: none must be
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert "Klipper knows neither ACEK_RESTORE_STOCK nor ACE_RESTORE_STOCK" in host.text()
    assert not any(line.startswith("Steps:") for line in host.lines)
    assert host.gcodes == [] and host.commands == []


def test_an_old_host_without_a_command_list_falls_back_and_flashes(tmp_path: Path) -> None:
    # the new name sent unchecked; Klipper's unknown-command line on the console after the mark:
    # the old name once, and the flash goes on
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.host_module, host.help_unanswered = "old", True
    sent, real = [], host.gcode

    def gcode(script: str) -> Literal[True] | fl.MoonrakerError | None:
        sent.append(clock.now())
        return real(script)

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.gcodes == ["ACEK_RESTORE_STOCK", "ACE_RESTORE_STOCK"]
    assert unit.state == "ace2k" and unit.ace2k_version == "0.12.0"
    assert sent[1] - sent[0] <= fl.UNKNOWN_SETTLE_S  # the bounded look, not the reset's wait


def test_path_a_to_factory_leaves_klipper_stopped(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert unit.state == "factory" and unit.version == "V1.1.31"
    assert not host.running
    assert [c[-1] for c in host.commands] == ["stop"]
    text = host.text()
    assert "Klipper is left STOPPED" in text
    assert "#*# [ace2k]" in text and "SAVE_CONFIG" in text
    assert "not compared" in text
    lines = host.lines
    wait = lines.index("Waiting for the factory firmware to start (up to 60 s)...")
    assert (
        lines.index("Written.")
        < wait
        < next(i for i, line in enumerate(lines) if line.startswith("The factory firmware answers"))
    )
    files = sorted((tmp_path / "cal").glob(f"ace-calibration-{UID_TEXT}-*.txt"))
    assert len(files) == 1


def test_calibration_compared_after_a_restore(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    cal = tmp_path / "cal"
    cal.mkdir()
    raw, pairs = fl.read_calibration(fl.Unit(_factory_twin(unit), unit.clock))
    (cal / f"ace-calibration-{UID_TEXT}-20261001-100000.txt").write_text(
        fl.calibration_text(raw, pairs)
    )
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert "Calibration intact (same as" in host.text()


def test_calibration_difference_is_reported(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    cal = tmp_path / "cal"
    cal.mkdir()
    raw, pairs = fl.read_calibration(fl.Unit(_factory_twin(unit), unit.clock))
    pairs[2] = (1, 2)
    (cal / f"ace-calibration-{UID_TEXT}-20261001-100000.txt").write_text(
        fl.calibration_text(raw, pairs)
    )
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert "differs from" in host.text() and "[2]" in host.text()


def _factory_twin(unit: SimUnit) -> SimUnit:
    return SimUnit(unit.clock, state="factory")


def test_first_install_klipper_in_error_holds_the_port(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    assert host.klippy_state() == "error"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0" and host.running
    assert len(list((tmp_path / "cal").glob(f"ace-calibration-{UID_TEXT}-*.txt"))) == 1
    assert host.gcodes == []


def test_factory_to_ace2k_without_config_is_unverified(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    a = args(image, tmp_path, port=host.port)
    assert run(host, a) == fl.EXIT_UNVERIFIED
    assert unit.state == "ace2k" and host.commands == [] and unit.proven
    flat = " ".join(host.text().split())
    assert "ace2k 0.12.0 answers and stored its proof: it stays." in flat
    assert flat.endswith(
        "The image is proven and stays: add [mcu ace2k] and [ace2k] (docs/flashing.md) and start"
        " Klipper."
    )
    assert "GOES BACK TO RECOVERY" not in flat


def test_an_image_not_proven_by_the_flasher_keeps_the_window_warning(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    opened = host.open_link

    def held_at_ace2k_baud(port: str, baud: int = fl.BAUD) -> SimUnit:
        if baud == fl.ACE2K_BAUD:  # Klipper took the port: its connect proves the image
            raise fl.RefusedError(f"{port} cannot be opened: busy")
        return opened(port, baud)

    host.open_link = held_at_ace2k_baud
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    flat = " ".join(host.text().split())
    assert "Klipper's connect proves the image instead" in flat
    assert "A NEWLY WRITTEN IMAGE GOES BACK TO RECOVERY IF NO HOST TALKS TO IT WITHIN 180 s" in flat
    assert flat.endswith(
        "If that window passes, the unit stays in recovery: finish the config, restart "
        "Klipper, and run this command again."
    )


def test_a_deaf_new_image_is_a_failure_that_names_recovery(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    unit.klipper_answers = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    flat = " ".join(host.text().split())
    assert "The new image does not answer on" in flat
    assert "It goes back to recovery by itself within 180 s of its start" in flat
    assert not unit.proven


def test_the_proof_follows_the_units_sequence_after_a_refusal(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    unit.mcu_seq = 5  # the image already spoke to someone: the first block is refused
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert unit.proven and unit.asked[-2:] == [77, 78]


def test_an_image_that_does_not_store_its_proof_is_a_failure(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    unit.proof_stores = False  # it answers its version, but proven=0
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    flat = " ".join(host.text().split())
    assert "The new image did not store its proof on" in flat
    assert "within 180 s of its start" in flat


def test_the_window_named_is_the_images_own(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0", window_ms=600000)
    )
    unit.klipper_answers = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert "within 600 s of its start" in " ".join(host.text().split())


def test_an_image_without_the_window_constant_names_both(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0", window_ms=None)
    )
    unit.klipper_answers = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert "within 180 s (600 s for some images) of its start" in " ".join(host.text().split())


def test_factory_to_ace2k_when_the_config_follows_says_so_briefly(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    a = args(image, tmp_path, port=host.port, config_follows=True)
    assert run(host, a) == fl.EXIT_UNVERIFIED
    assert unit.state == "ace2k" and host.commands == []
    text = host.text()
    assert "the caller adds it and starts Klipper" in " ".join(text.split())
    assert "NEWLY WRITTEN" not in text


def test_recovery_to_ace2k_operator_starts_klipper(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="recovery", image=ace2k_image("0.12.0"))
    host.running = False
    host._sync_proc()

    def start_when_asked(h: SimHost, line: str) -> None:
        if line.startswith("Start Klipper now. Waiting up to 90 s"):
            h.running = True
            h._sync_proc()

    host.on_line = start_when_asked
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_OK
    assert host.commands == []
    assert unit.ace2k_version == "0.12.0"
    assert any(line.startswith("Start Klipper now. Waiting up to 90 s") for line in host.lines)
    lines = host.lines
    wait = lines.index("Waiting 15 s for the bootloader to copy the image and start it...")
    assert (
        lines.index("Written.")
        < wait
        < next(i for i, line in enumerate(lines) if line.endswith("done."))
    )


def test_unidentified_changes_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert set(unit.received) == {fl.CMD_GET_INFO}
    assert host.commands == [] and host.running
    assert "nothing changed" in host.text()


def test_nothing_answering_on_a_free_port_has_its_own_status(tmp_path: Path) -> None:
    # the unit may run ace2k with no Klipper connected to it: ace2k-u1's installer reads this
    # status, so it must differ from every other refusal
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert f"Nothing on {host.port} answered" in host.text()
    assert "nothing changed" in host.text()


def test_nothing_answering_ends_the_dictionary_read_after_one_question(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # the dictionary read's long deadline is for a unit that answers slowly: a unit that is off
    # ends it after the first question's two waits (a silent line sends it once more), not after
    # the whole deadline
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    spans: list[float] = []
    enter = fl.route.enter_recovery

    def timed(*a: object) -> None:
        start = clock.now()
        try:
            enter(*a)
        finally:
            spans.append(clock.now() - start)

    monkeypatch.setattr(fl.route, "enter_recovery", timed)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert len(spans) == 1
    assert spans[0] <= 2.5 * fl.klipper.PROVE_TIMEOUT_S < fl.klipper.ENTER_DICTIONARY_S


def test_a_refusal_before_anything_was_done_has_its_own_status(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.answers = ["n"]
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.gcodes == [] and host.commands == []


def test_a_restore_that_did_not_run_is_unchanged(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.script_error, host.script_error_state = "Klippy Disconnected", "disconnected"
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.commands == [] and unit.state == "ace2k"


def test_a_refused_restore_may_have_reset_the_unit_so_it_is_a_failure(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED


def test_nothing_answering_after_only_klipper_was_stopped_is_still_no_answer(
    tmp_path: Path,
) -> None:
    # Klipper not ready on this port: stopped, nothing answered, started again.  Nothing that
    # changes the unit was sent, so the unit may still run ace2k: the same status 4
    clock, unit, host, image = setup(tmp_path, state="off", holds_when_not_ready=False)
    host.state_override = "error"
    assert run(host, args(image, tmp_path)) == fl.EXIT_NO_ANSWER
    assert "answered as the factory firmware or recovery" in host.text()
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert fl.CMD_IAP_UPGRADE not in unit.received


def test_a_failure_after_the_write_started_is_a_failure(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    real = unit._answer
    unit._answer = lambda f: None if f.cmd == fl.CMD_IAP_FIRMWARE else real(f)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED


def test_a_usage_error_is_refused_not_unverified(capsys: pytest.CaptureFixture[str]) -> None:
    # argparse's own status 2 would read as "written, not verified" to a caller
    with pytest.raises(SystemExit) as e:
        fl.main(["image.bin", "--no-such-option"])
    assert e.value.code == fl.EXIT_UNCHANGED


@pytest.mark.parametrize("url", ["localhost:7125", "ftp://localhost:7125", "http://"])
def test_a_moonraker_url_without_http_is_refused(
    capsys: pytest.CaptureFixture[str], url: str
) -> None:
    # such a URL never answers, and the tool would take Klipper for stopped
    with pytest.raises(SystemExit) as e:
        fl.main(["image.bin", "--moonraker", url])
    assert e.value.code == fl.EXIT_UNCHANGED
    assert capsys.readouterr().err.endswith(
        f"error: --moonraker {url}: give the URL with http:// or https:// and a host, "
        "such as http://localhost:7125\n"
    )


def test_the_help_names_every_exit_status(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit):
        fl.main(["--help"])
    text = capsys.readouterr().out
    for status in ("0 ", "1 ", "2 ", "3 ", "4 "):
        assert status in text
    assert "nothing was changed" in text and "read-only queries" in text
    assert "nothing answered" in text.lower()


@pytest.mark.parametrize("form", ["error", "console", "http"])
def test_restore_refused_changes_nothing(tmp_path: Path, form: str) -> None:
    # the refusal as a G-code error (Moonraker's HTTP error and Klipper's `!!` line), as the
    # `!!` line alone (an older ace2k), and as the HTTP error alone
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore, host.refuse_form = True, form
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.commands == [] and host.running and unit.state == "ace2k"
    assert host.gcodes == ["ACEK_RESTORE_STOCK"] and unit.received.count(fl.CMD_IAP_UPGRADE) == 0
    text = host.text()
    assert "ACEK_RESTORE_STOCK was refused: !! ace2k: refused — a subsystem is busy" in text
    assert "Klipper still reports ready, so nothing was stopped" in text
    assert "may or may not" not in text


def test_a_multi_line_refusal_is_counted_once_and_quotes_the_console(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore = True
    host.refusal_text = "ace2k: refused — a subsystem is busy (heating or moving)\nmore detail"
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert (
        "ACEK_RESTORE_STOCK was refused: !! ace2k: refused — a subsystem is busy (heating or"
        " moving) — Klipper" in text
    )
    assert "more detail" not in text and host.commands == []


NO_ANSWER = (
    "ace2k: bootloader enter: no answer from the unit; check the link (Unable to obtain"
    " 'ace2k_bootloader_enter_response' response)"
)


def test_the_units_lost_answer_waits_for_the_reset_then_names_it(tmp_path: Path) -> None:
    # the module's own error: the script ran, the unit may have reset — wait it out
    clock, unit, host, image = setup(tmp_path)
    host.script_error, host.script_error_console = NO_ANSWER, True
    start = clock.now()
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert clock.now() - start >= fl.RESTORE_WAIT_S
    text = host.text()
    assert "was refused" not in text and "no refusal was printed" not in text
    assert f"ACEK_RESTORE_STOCK failed: {NO_ANSWER}." in text
    assert "may or may not have reset" in text and "Nothing was stopped" in text
    assert host.commands == [] and host.running


@pytest.mark.parametrize(
    "error, state",
    [
        ("Klippy Disconnected", "disconnected"),  # Klipper left ready just before the POST
        ("\nPrinter is not ready\nThe klippy host software is attempting to connect.", "startup"),
        ("HTTP 503", None),  # a proxy in front of Moonraker
    ],
)
def test_a_script_that_did_not_run_stops_at_once(
    tmp_path: Path, error: str, state: str | None
) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.script_error, host.script_error_state = error, state
    start = clock.now()
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert clock.now() - start < fl.STATE_POLL_S
    first = error.strip().split("\n")[0]
    assert f"ACEK_RESTORE_STOCK did not run: {first}; nothing changed" in host.text()
    assert host.commands == [] and host.running and unit.state == "ace2k"


def test_an_http_refusal_reads_the_console_no_more(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "was refused" in host.text()
    assert host.console_reads == 1  # the mark before the POST, nothing after


def test_two_console_only_refusals_in_a_row_are_both_seen(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore, host.refuse_form = True, "console"
    for _ in range(2):
        host.answers = ["y"]
        assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
        assert "ACEK_RESTORE_STOCK was refused: !! ace2k: refused" in host.lines[-1]
    assert len(host.console) == 2 and host.commands == []


def test_another_clients_refusal_during_an_unanswered_post_is_not_this_one(tmp_path: Path) -> None:
    # no answer to this POST: the console cannot end the wait, and a slow reset still counts
    clock, unit, host, image = setup(tmp_path)
    host.script_unanswered, host.restore_lag = True, fl.REFUSAL_SETTLE_S + 2.0
    real = host.gcode

    def gcode(script: str) -> Literal[True] | fl.MoonrakerError | None:
        answer = real(script)
        host.console.append("!! ace2k: refused — a subsystem is busy (heating or moving)")
        return answer

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()


def test_another_script_error_then_a_reset_goes_on(tmp_path: Path) -> None:
    # the unit reset although the script errored (its answer lost): the reset is what counts
    clock, unit, host, image = setup(tmp_path)
    host.script_error, host.script_error_console = NO_ANSWER, True
    real = host._run_script

    def lost_answer(script: str) -> Literal[True] | fl.MoonrakerError:
        answer = real(script)
        host._reset_unit()
        return answer

    host._run_script = lost_answer
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()


def test_a_refusal_then_a_reset_within_the_settle_wait_goes_on(tmp_path: Path) -> None:
    # the module answers a refusal, yet the unit resets inside the settle wait: the reset is
    # what counts, so recovery is opened and the image written
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.refuse_restore = True
    real = host._run_script
    lag = fl.REFUSAL_SETTLE_S - 1.0

    def refused_then_reset(script: str) -> object:
        answer = real(script)
        at, fired = clock.now() + lag, []

        def hook() -> None:  # once: the new image must not be reset again
            if not fired and clock.now() >= at:
                fired.append(True)
                host._reset_unit()

        clock.hooks.append(hook)
        return answer

    host._run_script = refused_then_reset
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    text = host.text()
    assert "was refused" not in text and "Recovery answers (boot V1.0.2)." in text
    assert host.gcodes == ["ACEK_RESTORE_STOCK"]
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.received.count(fl.CMD_IAP_UPGRADE) == 1 and unit.ace2k_version == "0.12.0"


def test_an_unanswered_post_waits_and_says_it_may_not_have_arrived(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.script_unanswered, host.restore_silent = True, True
    start = clock.now()
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert clock.now() - start >= fl.RESTORE_WAIT_S
    text = host.text()
    assert "ACEK_RESTORE_STOCK got no answer: it may not have reached Moonraker." in text
    assert "may or may not have reset" in text and "no refusal was printed" not in text
    assert "was refused" not in text and host.commands == [] and host.running


def test_an_unanswered_post_that_ran_goes_on(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.script_unanswered = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK


def test_truncated_image_refused_before_the_port(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image()[:-3])
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.opened == 0 and host.gcodes == []


def test_printing_refused(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.printing = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.gcodes == [] and "print" in host.text()


def test_not_confirmed(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.answers = ["n"]
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.gcodes == [] and host.commands == []


def test_no_port_lists_candidates(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    text = host.text()
    assert "usb-1a86_USB_Single_Serial_TEST-if00" in text and "--port" in text
    assert host.opened == 0


def test_port_held_by_another_program(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    d = host.proc / "77" / "fd"
    d.mkdir(parents=True)
    (host.proc / "77" / "cmdline").write_bytes(b"minicom\0")
    (d / "3").symlink_to(host.port)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert "minicom" in host.text() and host.opened == 0


@pytest.mark.parametrize("case", ["ace2k_elsewhere", "moonraker_down"])
def test_port_held_by_a_klipper_that_is_not_to_be_stopped(tmp_path: Path, case: str) -> None:
    # Klipper holds the port, and the tool does not stop it: Klipper is ready with its ace2k on
    # another port, or Moonraker does not answer.  The port is refused like any held port.
    clock, unit, host, image = setup(tmp_path)
    if case == "ace2k_elsewhere":
        host.config_port = "/dev/serial/by-id/usb-1a86_ELSEWHERE-if00"
    else:
        host.moonraker_up = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert host.lines == [
        f"{host.port} is held open "
        "(pid 4242: /usr/bin/python3 /home/lava/klipper/klippy/klippy.py); nothing changed"
    ]
    assert host.opened == 0 and host.gcodes == [] and host.commands == []
    assert host.running and unit.state == "ace2k"


def test_image_refused_by_the_bootloader(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    unit.refuse_image = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "still answers" in host.text()
    assert host.running  # started again at exit


def test_no_way_to_stop_klipper(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.initd.unlink()
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.gcodes == [] and "--klipper-stop" in host.text()


def test_update_failure_restarts_klipper_and_says_rerun(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    real = unit._answer

    def drop_chunks(f: fl.Frame) -> bytes | None:
        return None if f.cmd == fl.CMD_IAP_FIRMWARE else real(f)

    unit._answer = drop_chunks
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "run the same command again" in host.text()
    assert host.running


def test_unknown_print_state_refused(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    assert host.klippy_state() == "ready"
    host.objects = lambda: None
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert host.gcodes == [] and host.commands == []
    assert "print state could not be read" in host.text()


def test_chained_units_refused_with_the_port_free(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, extra_units=1, image=ace2k_image("0.12.0")
    )
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.commands == []
    assert "2 units answered" in host.text()


def test_chained_units_refused_in_path_a(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, extra_units=1, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.running
    assert "2 units answered" in host.text()


def _first_install(
    tmp_path: Path, override: str | None = None
) -> tuple[FakeClock, SimUnit, SimHost, Path]:
    clock, unit, host, image = setup(
        tmp_path,
        state="factory",
        image=ace2k_image("0.12.0"),
        holds_when_not_ready=False,
    )
    host.state_override = override
    return clock, unit, host, image


def test_first_install_klipper_error_not_holding_the_port(tmp_path: Path) -> None:
    clock, unit, host, image = _first_install(tmp_path)
    assert host.klippy_state() == "error" and not list(host.proc.glob("*"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0" and host.running and host.gcodes == []
    assert len(list((tmp_path / "cal").glob(f"ace-calibration-{UID_TEXT}-*.txt"))) == 1


def test_first_install_klipper_in_startup(tmp_path: Path) -> None:
    clock, unit, host, image = _first_install(tmp_path, "startup")
    host.state_override = "startup"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0"


def test_first_install_without_a_service_refuses(tmp_path: Path) -> None:
    clock, unit, host, image = _first_install(tmp_path)
    host.initd.unlink()
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert "--klipper-stop" in host.text() and host.commands == [] and unit.received == []


def test_one_unit_refusal_after_the_reset_does_not_say_nothing_changed(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, extra_units=1, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "nothing changed" not in text
    assert "recovery" in text and "will not connect" in text and "run the tool again" in text


def test_one_unit_refusal_in_the_not_ready_path(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", extra_units=1, image=ace2k_image("0.12.0")
    )
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert "2 units answered" in host.text() and host.running


class _LinkFails:
    """Wraps the unit's link: write raises OSError on the Nth IAP_FIRMWARE frame."""

    def __init__(self, unit: SimUnit, nth: int) -> None:
        self.unit, self.nth, self.n = unit, nth, 0
        real = unit.write

        def write(data: bytes) -> None:
            for f in fl.parse_frames(bytearray(data)):
                if f.cmd == fl.CMD_IAP_FIRMWARE:
                    self.n += 1
                    if self.n == nth:
                        raise OSError("device disappeared")
            real(data)

        unit.write = write


def test_link_error_during_the_update(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    _LinkFails(unit, 3)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "run the same command again" in text and "device disappeared" in text
    assert host.running
    assert host.lines[-1].startswith("Starting Klipper again")


def test_link_error_outside_the_update_is_not_a_traceback(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    real = unit.write

    def boom(data: bytes) -> NoReturn:
        raise OSError("port gone")

    unit.write = boom
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert "Serial link error: port gone" in host.text()
    unit.write = real


def test_unwritable_calibration_dir_refused_before_the_update(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    (tmp_path / "cal").write_text("a file, not a directory")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "calibration could not be written" in host.text()
    assert fl.CMD_IAP_UPGRADE not in unit.received and host.running


def test_failed_start_is_reported_once(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.start_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-2:] == [
        f"Klipper did NOT start (exit 1): run {start_command(host)}",
        "Klipper did not start; the unit waits for it in ace2k",
    ]
    assert [c[-1] for c in host.commands] == ["stop", "start"]


def test_moonraker_silent_after_the_gcode_is_not_taken_as_not_ready(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    real = host.gcode

    def gcode(script: str) -> None:
        real(script)
        host.moonraker_up = False

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "Moonraker does not answer" in host.text() and host.commands == []


def test_plan_shows_the_stop_and_the_start_commands(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.answers = ["n"]
    run(host, args(image, tmp_path))
    text = host.text()
    assert f"{host.initd} stop" in text and f"{host.initd} start" in text


def test_plan_says_the_tool_starts_klipper_when_the_tool_stopped_it(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.answers = ["n"]
    run(host, args(image, tmp_path))
    text = host.text()
    assert f"start Klipper ({host.initd} start); check it reports ace2k 0.12.0" in text
    assert "you start (or restart) Klipper when asked" not in text


def test_plan_says_the_operator_starts_klipper_when_the_tool_did_not_stop_it(
    tmp_path: Path,
) -> None:
    clock, unit, host, image = setup(tmp_path, state="recovery", image=ace2k_image("0.12.0"))
    host.running = False
    host._sync_proc()
    host.answers = ["n"]
    run(host, args(image, tmp_path, port=host.port))
    text = host.text()
    assert (
        "you start (or restart) Klipper when asked; the tool checks it reports ace2k 0.12.0" in text
    )
    assert "start Klipper (" not in text


def test_factory_plan_says_klipper_is_left_stopped(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    host.answers = ["n"]
    run(host, args(image, tmp_path, version="1.1.31"))
    assert "Klipper is left stopped afterwards" in host.text()


def test_still_answers_names_the_factory_firmware(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    real = unit._schedule_commit

    def commit(linger: bool = False) -> None:
        real(linger)
        unit.commit = ("factory", "V1.1.31")

    unit._schedule_commit = commit
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "factory firmware answers after the update" in host.text()


def test_a_factory_firmware_that_keeps_the_ace2k_update_says_to_power_cycle(
    tmp_path: Path,
) -> None:
    # factory -> ace2k: the factory firmware took the update and still answers 15 s later, so
    # it holds the update and refuses the next one until it restarts
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    unit.linger_s = 1000.0
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        "The factory firmware answers after the update: the image was not taken.  Power-cycle "
        "the unit, then check the image and run the command again."
    )


def test_moonraker_down_is_unverified_with_its_own_reason(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    host.moonraker_up = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    text = host.text()
    assert "Moonraker does not answer here" in text
    assert "The image is proven and stays" in text and "GOES BACK TO RECOVERY" not in text


def test_stopped_but_still_held_is_refused_and_klipper_started(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.keep_holder = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "still held open" in host.text()
    assert host.running


# --- the restore wait, old console lines, undecodable replies and failed service commands -----
def test_restore_wait_covers_klippers_slow_notice(tmp_path: Path) -> None:
    # Klipper takes ~4-6 s to notice the reset; the wait must outlast that
    assert fl.RESTORE_WAIT_S == 15.0
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 8.0
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert unit.ace2k_version == "0.12.0"


def test_a_stale_console_error_is_not_this_refusal(tmp_path: Path) -> None:
    # An old `!!` line in the store, no new one, Klipper still ready: ambiguous, not refused
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! an old error from an hour ago")
    host.restore_silent = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "an old error" not in text and "was refused" not in text
    assert "still reports ready" in text and "may or may not have reset" in text
    assert "Nothing was stopped" in text and "all four strobing fast is recovery" in text
    assert host.commands == []


def test_a_new_error_after_an_old_one_is_the_refusal(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! an old error from an hour ago")
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "ACEK_RESTORE_STOCK was refused: !! ace2k: refused — " in text
    assert "an old error" not in text
    assert "Klipper still reports ready, so nothing was stopped" in text
    assert "If all four LEDs strobe fast, the unit did reset: run the tool again" in text


def test_the_same_error_text_again_counts_as_new(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! ace2k: refused — a subsystem is busy (heating or moving)")
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "was refused" in host.text() and "may or may not" not in host.text()


def test_an_undecodable_reply_mid_update_is_not_a_traceback(tmp_path: Path) -> None:
    # The third chunk's answer holds a truncated varint
    clock, unit, host, image = setup(tmp_path)
    real, n = unit._answer, [0]

    def garble(f: fl.Frame) -> bytes | None:
        if f.cmd == fl.CMD_IAP_FIRMWARE:
            n[0] += 1
            if n[0] == 3:
                return reply(0x00, f.seq, f.cmd, b"\x08\x80")  # a truncated varint
        return real(f)

    unit._answer = garble
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "an answer from the unit does not decode" in text
    assert "run the same command again" in text and host.running


def test_an_undecodable_announcement_reply(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    real = unit._answer
    unit._answer = lambda f: (
        reply(0x00, f.seq, f.cmd, b"\x08\x80") if f.cmd == fl.CMD_IAP_UPGRADE else real(f)
    )
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "does not decode" in host.text()


def test_an_older_image_is_recognised_by_its_own_version_string(tmp_path: Path) -> None:
    # --version 0.11, the image's own string says 0.11.0, Klipper reports that.  As in the
    # real v0.11.0 image, the bare string follows the dictionary's last byte
    img = ace2k_image(with_constant=False, after=b"0.11.0\x00")
    assert img[img.index(b"0.11.0\x00") - 1] not in b"\x00 "
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "0.11.0"
    assert run(host, args(image, tmp_path, version="0.11")) == fl.EXIT_OK
    assert any(line.endswith("done.") for line in host.lines)


def test_an_older_image_whose_bytes_lack_the_reported_string_fails(tmp_path: Path) -> None:
    img = ace2k_image(with_constant=False)
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "9.9.9"
    assert run(host, args(image, tmp_path, version="0.11")) == fl.EXIT_FAILED
    assert "The new image answers version 9.9.9, not 0.11" in host.text()


def test_an_embedded_version_must_match_exactly(tmp_path: Path) -> None:
    img = ace2k_image("0.12.0", extra=b"0.99.0")
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "0.99.0"
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED


def test_cannot_check_after_this_run_stopped_klipper_starts_it_again(tmp_path: Path) -> None:
    # Klipper not ready and holding the port, [ace2k] configured on ANOTHER port
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.config_port = "/dev/ttyElsewhere"
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert [c[-1] for c in host.commands] == ["stop", "start"] and host.running
    text = host.text()
    flat = " ".join(text.split())
    assert "Klipper was started again with its current config" in flat
    assert "no [ace2k] on this port" in flat
    assert "The image is proven and stays" in flat and "and restart Klipper." in flat
    warning = text.split("Klipper was started again")[1].splitlines()
    assert all(len(line) <= 95 for line in warning)


def test_a_stop_that_fails_leaves_klipper_alone_path_a(tmp_path: Path) -> None:
    # Under a ready Klipper: the restore already reset the unit, then the stop fails
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.stop_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "failed (exit 1): Klipper was not stopped" in text
    assert "had already reset the unit" in text and "nothing changed" not in text
    assert [c[-1] for c in host.commands] == ["stop"]  # no start attempt


def test_a_stop_that_fails_leaves_klipper_alone_not_ready_path(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.stop_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    text = host.text()
    assert "failed (exit 1): Klipper was not stopped; nothing changed" in text
    assert [c[-1] for c in host.commands] == ["stop"]


@pytest.mark.parametrize("which", ["stop", "start"])
def test_a_missing_service_command_is_not_a_traceback(tmp_path: Path, which: str) -> None:
    # The stop or the start command does not exist on the host
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.raise_on[which] = FileNotFoundError("no such command")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    if which == "stop":
        assert "Klipper was not stopped" in text and "no such command" in text
        assert [c[-1] for c in host.commands] == ["stop"]
    else:
        assert host.lines[-2:] == [
            f"Klipper did NOT start (no such command): run {start_command(host)}",
            "Klipper did not start; the unit waits for it in ace2k",
        ]


def test_the_reset_note_names_the_command_sent(tmp_path: Path) -> None:
    for module, name in (("new", "ACEK_RESTORE_STOCK"), ("old", "ACE_RESTORE_STOCK")):
        (tmp_path / module).mkdir()
        clock, unit, host, image = setup(tmp_path / module, image=ace2k_image("0.12.0"))
        host.host_module, host.stop_rc = module, 1
        assert run(host, args(image, tmp_path / module)) == fl.EXIT_FAILED
        assert f"  {name} had already reset the unit" in host.text(), module


def test_held_after_the_reset_says_the_unit_is_in_recovery(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Another program takes the port after Klipper let go
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real, calls = fl.port_holders, [0]

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        calls[0] += 1
        return real(port, proc) if calls[0] == 1 else [(99, "minicom")]

    patch_holders(monkeypatch, holders)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "held open by another program" in text and "nothing changed" not in text
    assert "had already reset the unit" in text


def test_restart_klipper_branch_when_it_is_up_but_not_ready(tmp_path: Path) -> None:
    # Klipper running, not ready after the flash, not stopped by this run
    clock, unit, host, image = setup(
        tmp_path, state="factory", image=ace2k_image("0.12.0"), holds_when_not_ready=False
    )
    host.state_override = "ready"  # as the pre-flash view sees it: no ace2k version yet

    def settle(h: SimHost, line: str) -> None:
        if line.startswith("Restart Klipper (FIRMWARE_RESTART"):
            h.state_override = None

    host.on_line = settle
    real = unit._schedule_commit

    def commit(linger: bool = False) -> None:
        real(linger)
        host.state_override = "startup"

    unit._schedule_commit = commit
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_OK
    assert any(line.startswith("Restart Klipper (FIRMWARE_RESTART") for line in host.lines)
    assert host.commands == []


def test_unverifiable_flash_with_a_failed_start_tries_once(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.config_port = "/dev/ttyElsewhere"
    host.start_rc = 1
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert "was started again" not in host.text()
    assert host.lines[-1].startswith("Klipper did NOT start (exit 1)")


def test_an_unrelated_error_line_does_not_end_the_wait(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 1.0
    real = host.gcode

    def gcode(script: str) -> Literal[True] | fl.MoonrakerError | None:
        host.console.append("!! some unrelated error")
        return real(script)

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()
    assert [c[-1] for c in host.commands] == ["stop", "start"]


def test_a_five_second_lag_after_an_unrelated_error_still_proceeds(tmp_path: Path) -> None:
    assert fl.REFUSAL_SETTLE_S == 6.0
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 5.0
    real = host.gcode

    def gcode(script: str) -> Literal[True] | fl.MoonrakerError | None:
        host.console.append("!! some unrelated error")
        return real(script)

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()


def test_a_named_unit_is_found_by_its_port_and_its_restore_names_it(tmp_path: Path) -> None:
    # [ace2k a] on this port beside [ace2k other]: the unit under Klipper is a, and the restore
    # names it (with several units ACEK_RESTORE_STOCK needs UNIT=)
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.unit_name = "a"
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_OK
    assert host.gcodes == ["ACEK_RESTORE_STOCK UNIT=a"]
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.state == "ace2k" and unit.ace2k_version == "0.12.0"
    assert "Klipper reports ace2k 0.12.0: done." in host.text()


def test_one_named_unit_alone_needs_no_port(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.unit_name = "a"
    status = host.objects()
    del status["configfile"]["settings"]["ace2k other"]
    host.objects = lambda: status if host.running else {}
    view = fl.klipper_view(host)
    assert (view.ace2k_unit, view.ace2k_port, view.ace2k_version) == ("a", host.port, "0.11.0")


def test_several_units_and_no_port_name_them_and_change_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.unit_name = "a"
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    text = host.text()
    assert "Klipper's config has several units:" in text
    assert f"[ace2k a] on {host.port}" in text and "[ace2k other] on " in text
    assert "--port" in text and host.gcodes == [] and host.commands == []


def test_a_named_units_refusal_is_read_as_a_refusal(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)
    host.unit_name = "a"
    host.refuse_restore = True
    host.refusal_text = "ace2k a: refused — a subsystem is busy (heating or moving)"
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    text = host.text()
    assert "ACEK_RESTORE_STOCK was refused: !! ace2k a: refused" in text
    assert "did not run" not in text and host.commands == []


# --- The refusals and failures on the update's own path ----------------------------------------
RESET_NOTE_TEXT = (
    "  ACEK_RESTORE_STOCK had already reset the unit into the bootloader's recovery (it stays "
    "there and accepts an update).  Klipper is being started again but will not connect until "
    "ace2k is written again: split the chain or check the port, then run the tool again "
    "(recovery is identified as such)."
)
LEFT_STOPPED = [
    "Klipper is left STOPPED: this run targets the factory firmware, which does not",
    "speak Klipper's protocol (if the update failed, the unit is in recovery).",
    "Remove [mcu ace2k] and [ace2k] from the config, including any #*# [ace2k]",
    "section in the SAVE_CONFIG block at the end of printer.cfg, then start Klipper:",
]


def start_command(host: SimHost) -> str:
    """The start command of the simulated host's init script, as the tool prints it."""
    return shlex.join([str(host.initd), "start"])


@pytest.mark.parametrize(
    ("shape", "expected", "last"),
    [
        # a new install, Klipper not yet configured: flash_by_port with --config-follows
        (
            "by_port",
            fl.EXIT_UNVERIFIED,
            "Klipper's config has no [ace2k] on this port yet: the caller adds it and starts "
            "Klipper.",
        ),
        # an update: ace2k under a ready Klipper (flash_from_ace2k)
        ("from_ace2k", fl.EXIT_OK, "Klipper reports ace2k 0.12.0: done."),
        # a removal back to the factory firmware
        ("stock", fl.EXIT_OK, "The factory firmware answers: V1.1.31."),
    ],
)
def test_yes_as_the_installer_passes_it_never_asks(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, shape: str, expected: int, last: str
) -> None:
    # the ace2k-u1 installer runs `ace2k_flash IMAGE [--port P] [--config-follows]
    # --calibration-dir D --yes` with no terminal on stdin: no question may be asked
    if shape == "by_port":
        clock, unit, host, image = setup(
            tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
        )
        argv = [str(image), "--port", host.port, "--config-follows"]
    elif shape == "from_ace2k":
        clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
        argv = [str(image)]
    else:
        clock, unit, host, image = setup(tmp_path, image=factory_image())
        argv = [str(image), "--version", "1.1.31"]
    argv += ["--calibration-dir", str(tmp_path / "cal"), "--yes"]

    def never_asked(prompt: str) -> str:
        raise AssertionError(f"asked {prompt!r} despite --yes")

    host.ask = never_asked
    # main() builds the real dependencies (a serial port); the simulated host replaces them
    monkeypatch.setattr(fl.flasher, "Flasher", lambda a, deps: fl.Flasher(a, host.deps()))
    assert fl.main(argv) == expected
    assert "Steps:" in host.lines  # the plan is still shown
    assert last in host.lines


def test_end_of_input_at_the_question_is_not_a_yes(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)

    def end_of_input(prompt: str) -> str:
        raise EOFError

    host.ask = end_of_input
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.lines[-1] == "Not confirmed; nothing changed."
    assert host.gcodes == [] and host.commands == [] and host.opened == 0


def _count_chunks(image: Path) -> int:
    return (len(image.read_bytes()) + fl.wire.CHUNK_BYTES - 1) // fl.wire.CHUNK_BYTES


@pytest.mark.parametrize(
    ("case", "failure"),
    [
        ("announce_unanswered", "no answer to the announcement"),
        ("announce_refused", "the announcement was refused (code 1)"),
        ("chunk_refused", "chunk 2/{count} refused (code 3)"),
    ],
)
def test_an_update_the_unit_does_not_take_fails_and_says_rerun(
    tmp_path: Path, case: str, failure: str
) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    if case == "announce_unanswered":
        unit.announce_code = None
    elif case == "announce_refused":
        unit.announce_code = 1
    else:
        unit.chunk_codes = {2: 3}
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-3:] == [
        f"The update failed: {failure.format(count=_count_chunks(image))}.",
        "The unit accepts a new update: run the same command again.",
        f"Starting Klipper again: {start_command(host)}",
    ]
    assert fl.CMD_IAP_FINISH not in unit.received
    chunks = unit.received.count(fl.CMD_IAP_FIRMWARE)
    assert chunks == (2 if case == "chunk_refused" else 0)
    assert host.running and unit.state == "recovery"


FOUND_FACTORY = (
    ": nothing was written.  The unit runs the factory firmware, which Klipper does not "
    "connect to: run the tool again, and it identifies the factory firmware and writes the image."
)


@pytest.mark.parametrize(
    ("after", "found"),
    [("off", "nothing." + RESET_NOTE_TEXT), ("factory", "the factory firmware" + FOUND_FACTORY)],
)
def test_no_recovery_after_the_restore_is_a_failure(tmp_path: Path, after: str, found: str) -> None:
    # Klipper left ready, so the restore seemed to work, but the unit is not in recovery
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))

    def reset_elsewhere() -> None:
        unit.state = after

    host._reset_unit = reset_elsewhere
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-2:] == [
        f"Expected the bootloader's recovery on {host.port}, found {found}",
        f"Starting Klipper again: {start_command(host)}",
    ]
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert [c[-1] for c in host.commands] == ["stop", "start"]


@pytest.mark.parametrize(
    ("case", "refusal"),
    [
        (
            "recovery",
            "The factory firmware did not answer within the wait (recovery still answers)",
        ),
        ("silent", "The factory firmware did not answer within the wait (nothing answers)"),
        ("other_version", "The factory firmware answers with V1.1.30, not 1.1.31"),
    ],
)
def test_a_factory_flash_that_does_not_verify_is_a_failure(
    tmp_path: Path, case: str, refusal: str
) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    if case == "recovery":
        unit.refuse_image = True  # the bootloader keeps the unit in recovery
    elif case == "silent":
        unit.commit_s = 1000.0  # still copying when the wait runs out
    else:
        unit.reports_version = "V1.1.30"
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_FAILED
    assert host.lines[-6:] == [refusal, *LEFT_STOPPED, f"  {start_command(host)}"]
    assert not host.running


def test_klipper_reporting_the_old_version_is_a_failure(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real = host.objects

    def old_version() -> dict[str, object]:
        status = real()
        if "ace2k" in status:
            status["ace2k"] = {"version": "0.11.0"}
        return status

    host.objects = old_version
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        "Klipper did not report ace2k 0.12.0 within 90 s: check ACEK_STATUS and klippy.log"
    )
    assert unit.ace2k_version == "0.12.0" and host.running


def test_a_port_that_does_not_exist_is_refused(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    port = str(tmp_path / "no-such-port")
    assert run(host, args(image, tmp_path, port=port)) == fl.EXIT_UNCHANGED
    assert host.lines == [f"{port} does not exist; nothing changed"]
    assert host.opened == 0


def test_a_calibration_unread_after_a_factory_flash_is_a_warning(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    real = unit._answer

    def no_calibration(f: fl.Frame) -> bytes | None:
        return None if f.cmd == fl.CMD_GET_MOTOR_STATUS else real(f)

    unit._answer = no_calibration
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert (
        "Warning: the calibration could not be read after the update: no answer to "
        "GET_MOTOR_STATUS (the calibration)"
    ) in host.lines


def _factory_to_factory(tmp_path: Path, old: str) -> tuple[SimUnit, SimHost, Path]:
    """A unit on the factory firmware old, its port free, and the factory image 1.1.31."""
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, version=old, image=factory_image()
    )
    return unit, host, image


def test_a_factory_to_factory_flash_waits_for_the_old_firmware_to_let_go(tmp_path: Path) -> None:
    # the old factory firmware answers for about 2 s after the finish: its V1.1.30 is not the
    # new firmware refusing to start
    unit, host, image = _factory_to_factory(tmp_path, "V1.1.30")
    a = args(image, tmp_path, port=host.port, version="1.1.31")
    assert run(host, a) == fl.EXIT_OK
    assert unit.version == "V1.1.31"
    assert "The factory firmware answers: V1.1.31." in host.lines


def test_a_factory_flash_is_not_verified_against_the_old_firmware(tmp_path: Path) -> None:
    # same version, and the bootloader refuses the image: the old firmware's answer right after
    # the finish must not count as the new one
    unit, host, image = _factory_to_factory(tmp_path, "V1.1.31")
    unit.refuse_image = True
    a = args(image, tmp_path, port=host.port, version="1.1.31")
    assert run(host, a) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        "The factory firmware did not answer within the wait (recovery still answers)"
    )


def test_one_lost_reply_of_the_old_firmware_is_not_taken_as_letting_go(tmp_path: Path) -> None:
    # the tool expects lost replies (identify asks five times): one lost reply while the old
    # firmware still answers must not end the wait, or its next answer is taken as the new one
    unit, host, image = _factory_to_factory(tmp_path, "V1.1.31")
    unit.refuse_image = True
    real, lost = unit._answer, []

    def answer(f: fl.Frame) -> bytes | None:
        out = real(f)
        if f.cmd == fl.CMD_GET_INFO and unit._lingering() and not lost:
            lost.append(f.seq)
            return None
        return out

    unit._answer = answer
    a = args(image, tmp_path, port=host.port, version="1.1.31")
    assert run(host, a) == fl.EXIT_FAILED
    assert lost
    assert host.lines[-1] == (
        "The factory firmware did not answer within the wait (recovery still answers)"
    )


@pytest.mark.parametrize("old", ["V1.1.30", "V1.1.31"])
def test_recovery_answering_during_the_copy_counts_as_letting_go(tmp_path: Path, old: str) -> None:
    # the bootloader may speak the protocol while it copies the image: recovery's answer ends
    # the wait for the old firmware at once, and the new firmware is waited for after it
    unit, host, image = _factory_to_factory(tmp_path, old)
    unit.recovery_while_committing = True
    a = args(image, tmp_path, port=host.port, version="1.1.31")
    assert run(host, a) == fl.EXIT_OK
    assert "The factory firmware answers: V1.1.31." in host.lines


def test_an_old_firmware_that_never_lets_go_is_a_failure(tmp_path: Path) -> None:

    unit, host, image = _factory_to_factory(tmp_path, "V1.1.31")
    unit.linger_s = 1000.0  # it never resets
    a = args(image, tmp_path, port=host.port, version="1.1.31")
    assert run(host, a) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        "The old factory firmware (V1.1.31) still answers 15 s after the update: it did not "
        "take the image.  Power-cycle the unit, then run the same command again."
    )


def test_a_damaged_ace2k_image_is_never_written(tmp_path: Path) -> None:
    good = ace2k_image("0.12.0")
    bad = good[:100] + bytes([good[100] ^ 0xFF]) + good[101:]
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False, image=bad)
    a = args(image, tmp_path, port=host.port, version="0.12.0")
    assert run(host, a) == fl.EXIT_UNCHANGED
    assert host.lines == [
        f"{image}: a damaged ace2k image (its checksum does not match its contents); "
        "download or build it again"
    ]
    assert host.opened == 0 and unit.received == [] and host.commands == []


def hangs(cmd: str) -> subprocess.TimeoutExpired:
    return subprocess.TimeoutExpired(cmd, 120.0)


@pytest.mark.parametrize("path", ["restore", "not_ready"])
def test_a_stop_that_hangs_is_ended_and_named(tmp_path: Path, path: str) -> None:
    state = "ace2k" if path == "restore" else "factory"
    clock, unit, host, image = setup(tmp_path, state=state, image=ace2k_image("0.12.0"))
    host.raise_on["stop"] = hangs("stop")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    stop = f"{host.initd} stop"
    first = f"`{stop}` did not finish within 120 s and was ended: Klipper may or may not be stopped"
    if path == "restore":
        assert host.lines[-1] == first + (
            ".  ACEK_RESTORE_STOCK had already reset the unit into the bootloader's recovery (it "
            "stays there and accepts an update).  Stop Klipper by hand, then run the tool again: "
            "it finds recovery and writes the image."
        )
    else:
        assert host.lines[-1] == first + (
            f".  If Klipper does not run now, start it with `{start_command(host)}`.  Find out "
            f"why `{stop}` hangs before you run the tool again."
        )
    assert [c[-1] for c in host.commands] == ["stop"]
    assert host.run_timeouts == [120.0]


@pytest.mark.parametrize("path", ["restore", "not_ready"])
def test_ctrl_c_during_the_klipper_stop_is_a_failure(tmp_path: Path, path: str) -> None:
    # systemctl hands the stop to systemd: Klipper may end up stopped although the command was
    # interrupted, so this is not "nothing changed"
    state = "ace2k" if path == "restore" else "factory"
    clock, unit, host, image = setup(tmp_path, state=state, image=ace2k_image("0.12.0"))
    host.raise_on["stop"] = KeyboardInterrupt()
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    first = f"`{host.initd} stop` was interrupted: Klipper may or may not be stopped."
    assert host.lines[-1].startswith(first)
    if path == "restore":
        assert "Stop Klipper by hand, then run the tool again" in host.lines[-1]
    else:
        assert (
            f"If Klipper does not run now, start it with `{start_command(host)}`"
            in (host.lines[-1])
        )


def test_a_start_that_hangs_is_ended_and_named(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.raise_on["start"] = hangs("start")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert (
        f"Klipper may not have started (`{host.initd} start` did not finish within 120 s): run "
        "it again or check Klipper."
    ) in host.lines
    assert all(t == 120.0 for t in host.run_timeouts)


def test_a_service_query_that_hangs_changes_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.systemd = True
    host.raise_on["klipper.service"] = hangs("systemctl cat klipper.service")
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.lines == [
        "`systemctl cat klipper.service` did not finish within 10 s; nothing changed"
    ]
    assert host.run_timeouts == [10.0] and host.gcodes == []


FACTORY_RETRY = (
    "The factory firmware may still hold this update and refuse a new one: power-cycle the "
    "unit, then run the same command again."
)
HELD_RETRY = (
    "The factory firmware probably holds an unfinished update from an earlier run: power-cycle "
    "the unit, then run the same command again."
)


@pytest.mark.parametrize(
    ("chunk_codes", "failure", "retry"),
    [
        ({3: None}, "chunk 3/{count}: no answer", FACTORY_RETRY),
        ({2: 1}, "chunk 2/{count} refused (code 1)", FACTORY_RETRY),
        ({1: 3}, "chunk 1/{count} refused (code 3)", HELD_RETRY),
    ],
    ids=["no_answer", "refused", "held"],
)
def test_a_failed_update_on_the_factory_firmware_says_to_power_cycle(
    tmp_path: Path, chunk_codes: dict[int, int | None], failure: str, retry: str
) -> None:
    # the factory firmware keeps an update it did not finish: a second announcement is taken and
    # ignored, and the next chunk must follow the last one it received
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    unit.chunk_codes = chunk_codes
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert host.lines[-2:] == [
        f"The update failed: {failure.format(count=_count_chunks(image))}.",
        retry,
    ]


def test_a_failure_on_the_factory_firmware_does_not_say_the_unit_is_in_recovery(
    tmp_path: Path,
) -> None:
    # factory image onto the factory firmware, Klipper stopped by this run and left stopped
    clock, unit, host, image = setup(
        tmp_path, state="factory", version="V1.1.30", image=factory_image()
    )
    unit.chunk_codes = {3: None}
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_FAILED
    assert host.lines[-7:] == [
        f"The update failed: chunk 3/{_count_chunks(image)}: no answer.",
        FACTORY_RETRY,
        "Klipper is left STOPPED: this run targets the factory firmware, which does not",
        "speak Klipper's protocol.",
        *LEFT_STOPPED[2:],
        f"  {start_command(host)}",
    ]


@pytest.mark.parametrize("kind", ["directory", "not_text", "not_pairs"])
def test_an_unreadable_earlier_calibration_is_a_warning_after_a_verified_flash(
    tmp_path: Path, kind: str
) -> None:
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    cal = tmp_path / "cal"
    cal.mkdir()
    earlier = cal / f"ace-calibration-{UID_TEXT}-20261001-100000.txt"
    if kind == "directory":
        earlier.mkdir()
        reason = "Is a directory"
    elif kind == "not_text":
        earlier.write_bytes(b"\xff\xfe not text")
        reason = "not a text file"
    else:
        earlier.write_text("0 600\n")
        reason = "calibration file line 1 does not parse"
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert "The factory firmware answers: V1.1.31." in host.lines
    saved = next(x for x in host.lines if x.startswith("Calibration saved: "))
    new = saved.removeprefix("Calibration saved: ")
    assert host.lines[host.lines.index(saved) + 1 :][:2] == [
        f"Warning: {earlier} could not be read ({reason}).",
        f"Warning: the calibration was not compared: compare {new} with an earlier file by hand.",
    ]


def test_a_link_failure_while_the_calibration_is_read_after_a_verified_flash(
    tmp_path: Path,
) -> None:
    # the factory firmware answered with the version: the flash is verified, and a link that
    # fails while the calibration is read again is a warning, not a failure
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    real = unit.write

    def write(data: bytes) -> None:
        if "The factory firmware answers: V1.1.31." in host.lines:
            raise OSError(5, "Input/output error")
        real(data)

    unit.write = write
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert (
        "Warning: the calibration could not be read after the update: [Errno 5] Input/output error"
    ) in host.lines
    assert not any("Serial link error" in x for x in host.lines)


def test_an_unreadable_newest_calibration_falls_back_to_an_older_one(tmp_path: Path) -> None:
    # a stray newest file must not cancel the check that catches a lost calibration
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    cal = tmp_path / "cal"
    cal.mkdir()
    older = cal / f"ace-calibration-{UID_TEXT}-20261001-100000.txt"
    older.write_text(fl.calibration_text(b"", PAIRS))
    newest = cal / f"ace-calibration-{UID_TEXT}-20261002-100000.txt"
    newest.mkdir()
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert f"Warning: {newest} could not be read (Is a directory)." in host.lines
    assert f"Calibration intact (same as {older.name})." in host.lines


def test_a_service_query_that_cannot_run_is_not_a_link_error(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.systemd = True
    host.raise_on["klipper.service"] = PermissionError(13, "Permission denied")
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.lines == [
        "`systemctl cat klipper.service` could not run (Permission denied); nothing changed"
    ]
    assert host.gcodes == []


def test_an_error_outside_the_link_is_not_called_a_link_error(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)

    def clock_gone() -> NoReturn:
        raise OSError("clock gone")

    host.now = clock_gone  # read when the calibration file is named
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert host.lines[-1] == "Error: clock gone"
    assert fl.CMD_IAP_UPGRADE not in unit.received


def test_klipper_that_does_not_start_again_turns_no_answer_into_a_failure(tmp_path: Path) -> None:
    # status 4 says Klipper may have been stopped and started again: not when the start failed
    clock, unit, host, image = setup(tmp_path, state="off", holds_when_not_ready=False)
    host.state_override = "error"
    host.start_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1] == f"Klipper did NOT start (exit 1): run {host.initd} start"
    assert [c[-1] for c in host.commands] == ["stop", "start"] and not host.running


def test_a_port_that_fails_to_close_still_gets_klipper_started(tmp_path: Path) -> None:
    # the cable's converter can vanish: closing its port then fails, after a failed update here
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    unit.chunk_codes = {2: 3}

    def close_fails() -> NoReturn:
        raise OSError(19, "No such device")

    unit.close = close_fails
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1] == f"Starting Klipper again: {start_command(host)}"
    assert [c[-1] for c in host.commands] == ["stop", "start"] and host.running


def test_ctrl_c_at_the_question_changes_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path)

    def interrupted(prompt: str) -> NoReturn:
        raise KeyboardInterrupt

    host.ask = interrupted
    assert run(host, args(image, tmp_path)) == fl.EXIT_UNCHANGED
    assert host.lines[-1] == "Interrupted; nothing changed."
    assert host.gcodes == [] and host.commands == [] and host.opened == 0


def test_ctrl_c_during_the_update_is_a_failure_and_klipper_is_started(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real = unit.write

    def write(data: bytes) -> None:
        if fl.CMD_IAP_FIRMWARE in [f.cmd for f in fl.parse_frames(bytearray(data))]:
            raise KeyboardInterrupt
        real(data)

    unit.write = write
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-3:] == [
        "Interrupted.",
        "The unit accepts a new update: run the same command again.",
        f"Starting Klipper again: {start_command(host)}",
    ]
    assert host.running


def test_a_collision_that_hides_a_chained_unit_is_seen_through(tmp_path: Path) -> None:
    # one DISCOVER round loses the second unit's reply on the bus: the next rounds see it
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, extra_units=1, image=ace2k_image("0.12.0")
    )
    unit.collisions = 1
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.lines[-1] == (
        f"2 units answered on {host.port}: this tool flashes one unit per port (a daisy chain "
        "is not supported); nothing changed"
    )


def test_a_port_that_cannot_be_opened_after_the_reset_says_the_unit_is_in_recovery(
    tmp_path: Path,
) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))

    def busy(port: str, baud: int = fl.BAUD) -> NoReturn:
        raise fl.RefusedError(f"{port} cannot be opened: [Errno 16] Device or resource busy")

    host.open_link = busy
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-2:] == [
        f"{host.port} cannot be opened: [Errno 16] Device or resource busy." + RESET_NOTE_TEXT,
        f"Starting Klipper again: {start_command(host)}",
    ]


def test_a_failed_stop_after_the_restore_does_not_say_klipper_is_started(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.stop_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        f"`{host.initd} stop` failed (exit 1): Klipper was not stopped.  ACEK_RESTORE_STOCK had "
        "already reset the unit into the bootloader's recovery (it stays there and accepts an "
        "update).  Klipper will not connect until ace2k is written again: split the chain or "
        "check the port, then run the tool again (recovery is identified as such)."
    )


@pytest.mark.parametrize("answer", ["", "0", "11.0", "1.0", ".0"])
def test_an_older_image_is_not_proven_by_a_piece_of_its_bytes(tmp_path: Path, answer: str) -> None:
    # the version an older image answers must be one of its own version strings, whole: these
    # are pieces of "0.11.0", and "0" also stands alone elsewhere in the image (a real image
    # carries "\x000\x00" too)
    img = ace2k_image(with_constant=False, extra=b"\x000\x00", after=b"0.11.0\x00")
    assert answer.encode() in img
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = answer
    assert run(host, args(image, tmp_path, version="0.11")) == fl.EXIT_FAILED
    assert f"The new image answers version {answer}, not 0.11" in host.text()


def test_a_program_that_opens_the_port_before_the_write_stops_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Moonraker down, the port free when the tool opened it: Klipper reopens it a moment later
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    host.moonraker_up = False
    real = fl.port_holders
    klippy = "/usr/bin/python3 /home/lava/klipper/klippy/klippy.py"

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        return [(4242, klippy)] if unit.received else real(port, proc)

    patch_holders(monkeypatch, holders)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.lines[-1] == (
        f"{host.port} was opened by another program after this tool opened it "
        f"({klippy}, pid 4242); nothing changed"
    )


@pytest.mark.parametrize("kind", ["ace2k", "factory"])
def test_a_program_that_opens_the_port_after_a_stop_leaves_klipper_started(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, kind: str
) -> None:
    # Klipper stopped by this run (not ready): nothing was written, so Klipper is started again,
    # even for the factory image, which would otherwise leave it stopped
    img = ace2k_image("0.12.0") if kind == "ace2k" else factory_image()
    clock, unit, host, image = setup(tmp_path, state="factory", version="V1.1.30", image=img)
    real = fl.port_holders

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        return [(99, "minicom")] if fl.CMD_DISCOVER in unit.received else real(port, proc)

    patch_holders(monkeypatch, holders)
    version = None if kind == "ace2k" else "1.1.31"
    assert run(host, args(image, tmp_path, version=version)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.lines[-2:] == [
        f"{host.port} was opened by another program after this tool opened it (minicom, pid 99)"
        "; nothing was written",
        f"Starting Klipper again: {start_command(host)}",
    ]


@pytest.mark.parametrize("what", ["refused", "link"])
def test_a_program_that_opens_the_port_during_the_write_is_named(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, what: str
) -> None:
    # the check before the write cannot close the window during it: a failure names the holder
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    real = fl.port_holders

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        return [(99, "minicom")] if fl.CMD_IAP_FIRMWARE in unit.received else real(port, proc)

    patch_holders(monkeypatch, holders)
    if what == "refused":
        unit.chunk_codes = {3: None}
    else:
        write = unit.write

        def broken(data: bytes) -> None:
            if unit.received.count(fl.CMD_IAP_FIRMWARE) >= 3:
                raise OSError(5, "Input/output error")
            write(data)

        unit.write = broken
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert (
        f"Another program opened {host.port} during the write (minicom, pid 99): it is the "
        "likely cause."
    ) in host.lines


def test_holders_that_cannot_be_read_after_a_failed_write_name_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    real = fl.port_holders

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        if fl.CMD_IAP_FIRMWARE in unit.received:
            raise fl.RefusedError("cannot see who holds the port")
        return real(port, proc)

    patch_holders(monkeypatch, holders)
    unit.chunk_codes = {3: None}
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert host.lines[-1] == FACTORY_RETRY


def test_the_tool_itself_holding_the_port_is_not_another_program(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    real = fl.port_holders

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        return [(os.getpid(), "python3 ace2k_flash.pyz")] if unit.received else real(port, proc)

    patch_holders(monkeypatch, holders)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert fl.CMD_IAP_UPGRADE in unit.received


def test_a_program_that_opens_the_port_after_the_restore_says_the_unit_is_in_recovery(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real = fl.port_holders

    def holders(port: str, proc: Path) -> list[tuple[int, str]]:
        return [(99, "minicom")] if unit.received else real(port, proc)

    patch_holders(monkeypatch, holders)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.lines[-2:] == [
        f"{host.port} was opened by another program after this tool opened it (minicom, pid 99)."
        + RESET_NOTE_TEXT,
        f"Starting Klipper again: {start_command(host)}",
    ]


def test_a_port_that_cannot_be_opened_on_a_free_port_changes_nothing(tmp_path: Path) -> None:
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )

    def busy(port: str, baud: int = fl.BAUD) -> NoReturn:
        raise fl.RefusedError(f"{port} cannot be opened: [Errno 16] Device or resource busy")

    host.open_link = busy
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert host.lines[-1] == f"{host.port} cannot be opened: [Errno 16] Device or resource busy"


def test_ctrl_c_after_the_restore_says_the_unit_may_be_in_recovery(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real = host.state

    def state() -> str | None:
        if host.gcodes:  # the restore was sent: Ctrl-C while the tool waits for the reset
            raise KeyboardInterrupt
        return real()

    host.state = state
    assert run(host, args(image, tmp_path, yes=True)) == fl.EXIT_FAILED
    assert host.lines[-2:] == [
        "Interrupted.",
        "The restore command was sent: the unit may be in the bootloader's recovery (the four "
        "LEDs strobing fast); run the tool again, and it identifies recovery.",
    ]


def test_ctrl_c_after_recovery_was_opened_gives_the_reset_note(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real = unit.write

    def write(data: bytes) -> None:
        if unit.state == "recovery":
            raise KeyboardInterrupt
        real(data)

    unit.write = write
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-3:] == [
        "Interrupted.",
        RESET_NOTE_TEXT.strip(),
        f"Starting Klipper again: {start_command(host)}",
    ]


WRITTEN_CUT_SHORT = (
    "The image was written, but checking it was cut short. Running the same command again "
    "writes the image again: after the plan and the question, or at once with --yes."
)


@pytest.mark.parametrize("what", ["link", "ctrl_c"])
def test_a_check_cut_short_after_the_write_says_what_to_do(tmp_path: Path, what: str) -> None:
    # the finish was answered: the update is not unfinished, so no retry or power-cycle line
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    real = unit.write

    def write(data: bytes) -> None:
        if fl.CMD_IAP_FINISH in unit.received:
            if what == "ctrl_c":
                raise KeyboardInterrupt
            raise OSError(5, "Input/output error")
        real(data)

    unit.write = write
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    first = (
        "Interrupted." if what == "ctrl_c" else "Serial link error: [Errno 5] Input/output error"
    )
    assert host.lines[-2:] == [first, WRITTEN_CUT_SHORT]


ADAPTER_WARNING = "does not look like the unit's adapter"


def test_a_port_of_another_kind_is_warned_about_before_anything_is_sent(tmp_path: Path) -> None:
    # a printer head's MCU, given by mistake: the tool warns once, then goes on as before and
    # finds nothing it can identify
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    other = tmp_path / "ttyACM0"
    other.write_text("")
    host.port = str(other)
    sent_at_warning = []

    def note(h: SimHost, line: str) -> None:
        if ADAPTER_WARNING in line:
            sent_at_warning.append((list(unit.received), list(h.commands), list(h.gcodes)))

    host.on_line = note
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert sent_at_warning == [([], [], [])]
    warning = next(line for line in host.lines if ADAPTER_WARNING in line)
    assert warning == (
        f"Warning: {other} {ADAPTER_WARNING} ({host.by_id}/usb-1a86_USB_Single_Serial_*). "
        "Never give the port of a printer head."
    )


def test_the_units_adapter_gets_no_warning_by_its_name_or_through_a_link(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert not any(ADAPTER_WARNING in line for line in host.lines)
    (tmp_path / "linked").mkdir()
    clock, unit, host, image = setup(tmp_path / "linked", state="off", configured=False)
    link = tmp_path / "ttyACM0"
    link.symlink_to(host.port)
    host.port = str(link)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_NO_ANSWER
    assert not any(ADAPTER_WARNING in line for line in host.lines)


def test_an_unproven_image_not_reported_ends_with_a_balanced_window_note(tmp_path: Path) -> None:
    # an image from before the window constant, whose port another program takes after the
    # write: the wait for Klipper's report runs out, and the refusal names both windows
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0", window_ms=None))
    real_open, real_objects = host.open_link, host.objects

    def taken_at_ace2k_baud(port: str, baud: int = fl.BAUD) -> SimUnit:
        if baud == fl.ACE2K_BAUD:
            raise fl.RefusedError(f"{port} is held open by another program")
        return real_open(port, baud)

    def old_version() -> dict[str, object]:
        status = real_objects()
        if "ace2k" in status:
            status["ace2k"] = {"version": "0.11.0"}
        return status

    host.open_link = taken_at_ace2k_baud
    host.objects = old_version
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1] == (
        "Klipper did not report ace2k 0.12.0 within 90 s: check ACEK_STATUS and klippy.log (a newly"
        " written image goes back to recovery if no host talks to it within 180 s (600 s for some"
        " images))"
    )


# --- ace2k on a free port: sent into recovery over its own cable ----------------------------------


def test_ace2k_on_a_free_port_is_flashed_over_its_cable(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()  # Klipper down: nothing holds the port
    a = args(image, tmp_path, port=host.port, yes=True, config_follows=True)
    assert fl.Flasher(a, host.deps()).run() == fl.EXIT_UNVERIFIED
    assert 80 in unit.asked and unit.state == "ace2k" and unit.proven
    assert "over its cable" in host.text()
    assert host.commands == []


def test_the_plan_over_the_cable_comes_before_the_command(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    host.answers = ["n"]
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert 80 not in unit.asked and unit.state == "ace2k"
    assert f"Unit on {host.port}: ace2k 0.11.0, over its cable" in host.text()
    unit.ace2k_version = None  # an image whose dictionary has no version of its own
    host.answers, host.lines = ["n"], []
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNCHANGED
    assert f"Unit on {host.port}: ace2k v0.13.0, over its cable" in host.text()


def test_a_veto_refuses_with_nothing_written(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    unit.enter_accepted = False
    a = args(image, tmp_path, port=host.port, yes=True)
    assert fl.Flasher(a, host.deps()).run() == fl.EXIT_UNCHANGED
    assert unit.state == "ace2k" and fl.CMD_IAP_UPGRADE not in unit.received
    assert "refused" in host.text()
    assert "a lane moves, or the heater or the fans are busy" in host.text()


@pytest.mark.parametrize("why", ["no_command", "no_dictionary"])
def test_ace2k_that_cannot_be_sent_into_recovery_is_no_answer(tmp_path: Path, why: str) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    if why == "no_command":
        unit.has_enter = False
    else:
        unit.klipper_answers = False
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_NO_ANSWER
    assert unit.state == "ace2k" and 80 not in unit.asked
    assert f"Nothing on {host.port} answered" in host.text()
    # an image older than the recovery command is still ace2k: the text must not deny it
    assert "no ace2k that this flasher can send to recovery" in host.text()
    assert "not ace2k" not in host.text()


def test_an_unanswered_enter_that_reset_the_unit_goes_on(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    unit.enter_accepted = None  # the answer is lost, the reset happens
    a = args(image, tmp_path, port=host.port, yes=True, config_follows=True)
    assert run(host, a) == fl.EXIT_UNVERIFIED
    assert unit.state == "ace2k" and unit.proven


def test_an_unanswered_enter_with_no_recovery_is_a_failure(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    unit.enter_accepted = None
    real = unit._klipper

    def lost(data: bytes) -> None:  # neither answer nor reset: the unit may run ace2k still
        state = unit.state
        real(data)
        unit.state = state

    unit._klipper = lost
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert "may still run ace2k" in host.text() and "strobing fast" in host.text()
    assert "had already reset" not in host.text()


def test_an_accepted_enter_that_was_withdrawn_does_not_claim_recovery(tmp_path: Path) -> None:
    # a veto in the 50 ms before the halt withdraws an accepted command: the unit stays in ace2k
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    real = unit._klipper

    def withdrawn(data: bytes) -> None:
        state = unit.state
        real(data)
        unit.state = state

    unit._klipper = withdrawn
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_FAILED
    text = host.text()
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert "ace2k_bootloader_enter was sent, but the unit may still run ace2k" in text
    assert "Nothing was written" in text
    assert "had already reset" not in text and "will not connect" not in text


def test_a_held_port_after_the_enter_does_not_claim_recovery(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    unit.enter_accepted = None
    real = host.open_link

    def taken(port: str, baud: int = fl.BAUD) -> SimUnit:
        if 80 in unit.asked:  # another program took the port after the command
            raise fl.RefusedError(f"{port} is held open by another program")
        return real(port, baud)

    host.open_link = taken
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_FAILED
    assert "may still run ace2k" in host.text() and "had already reset" not in host.text()


def test_ctrl_c_after_the_enter_names_the_command_sent(tmp_path: Path) -> None:
    clock, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    real_sleep = clock.sleep

    def interrupt(dt: float) -> None:
        if 80 in unit.asked:
            raise KeyboardInterrupt
        real_sleep(dt)

    clock.sleep = interrupt
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_FAILED
    assert "ace2k_bootloader_enter was sent" in host.text()
    assert "The restore command was sent" not in host.text()


def test_an_image_without_the_enter_answer_is_no_answer(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    real = unit._dictionary

    def no_answer() -> bytes:
        doc = json.loads(zlib.decompress(real()))
        del doc["responses"]["ace2k_bootloader_enter_response accepted=%c"]
        return zlib.compress(json.dumps(doc).encode())

    unit._dictionary = no_answer
    assert run(host, args(image, tmp_path, port=host.port, yes=True)) == fl.EXIT_NO_ANSWER
    assert 80 not in unit.asked


def test_the_plan_with_config_follows_says_the_caller_starts_klipper(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="factory", configured=False)
    a = args(image, tmp_path, port=host.port, yes=True, config_follows=True)
    assert run(host, a) == fl.EXIT_UNVERIFIED
    assert "  - the caller starts Klipper, which checks the new image; this run does not wait" in (
        host.lines
    )
    assert not any("when asked" in line for line in host.lines)


def test_the_factory_image_restores_ace2k_over_its_cable(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="ace2k", configured=False)
    host.stop_klipper_now()
    stock = tmp_path / "stock.bin"
    stock.write_bytes(factory_image())
    a = args(stock, tmp_path, port=host.port, yes=True, version="1.1.31")
    assert fl.Flasher(a, host.deps()).run() == fl.EXIT_OK
    assert unit.state == "factory"
    assert host.starts == 0  # Klipper was down, and stays down
    assert "over its cable" in host.text()


def test_config_follows_never_waits_for_klipper(tmp_path: Path) -> None:
    _, unit, host, image = setup(tmp_path, state="factory", configured=True)
    host.stop_klipper_now()  # the installer stopped it; the config still names this port
    a = args(image, tmp_path, port=host.port, yes=True, config_follows=True)
    assert fl.Flasher(a, host.deps()).run() == fl.EXIT_UNVERIFIED
    assert "the caller starts Klipper" in host.text() and "Waiting up to" not in host.text()
    assert host.starts == 0 and unit.proven
