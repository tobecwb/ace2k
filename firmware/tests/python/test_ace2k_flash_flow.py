"""ace2k_flash end to end against the simulated unit and host: every path, every refusal."""

import pytest
from flash_sim import UID_TEXT, ace2k_image, args, factory_image, fl, reply, setup


def run(host, a):
    return fl.Flasher(a, host.deps()).run()


def test_path_a_ace2k_to_ace2k(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert host.gcodes == ["ACE_RESTORE_STOCK"]
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert host.running and unit.state == "ace2k" and unit.ace2k_version == "0.12.0"
    assert unit.received.count(fl.CMD_GET_INFO) >= 2  # the first query was lost to the noise
    assert "Klipper reports ace2k 0.12.0: done." in host.lines
    assert unit.closed


def test_path_a_to_factory_leaves_klipper_stopped(tmp_path):
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


def test_calibration_compared_after_a_restore(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    cal = tmp_path / "cal"
    cal.mkdir()
    raw, pairs = fl.read_calibration(fl.Unit(_factory_twin(unit), unit.clock))
    (cal / f"ace-calibration-{UID_TEXT}-20261001-100000.txt").write_text(
        fl.calibration_text(raw, pairs)
    )
    assert run(host, args(image, tmp_path, version="1.1.31")) == fl.EXIT_OK
    assert "Calibration intact (same as" in host.text()


def test_calibration_difference_is_reported(tmp_path):
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


def _factory_twin(unit):
    from flash_sim import SimUnit

    return SimUnit(unit.clock, state="factory")


def test_first_install_klipper_in_error_holds_the_port(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    assert host.klippy_state() == "error"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0" and host.running
    assert len(list((tmp_path / "cal").glob(f"ace-calibration-{UID_TEXT}-*.txt"))) == 1
    assert host.gcodes == []


def test_factory_to_ace2k_without_config_is_unverified(tmp_path):
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    a = args(image, tmp_path, port=host.port)
    assert run(host, a) == fl.EXIT_UNVERIFIED
    assert unit.state == "ace2k" and host.commands == []
    assert "180 s" in host.text()
    assert " ".join(host.text().split()).endswith(
        "If that window passes, the unit stays in recovery: finish the config, restart "
        "Klipper, and run this command again."
    )


def test_recovery_to_ace2k_operator_starts_klipper(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="recovery", image=ace2k_image("0.12.0"))
    host.running = False
    host._sync_proc()

    def start_when_asked(h, line):
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


def test_unidentified_changes_nothing(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="off", configured=False)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert set(unit.received) == {fl.CMD_GET_INFO}
    assert host.commands == [] and host.running
    assert "nothing changed" in host.text()


def test_restore_refused_changes_nothing(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.commands == [] and host.running and unit.state == "ace2k"
    assert "busy" in host.text()


def test_truncated_image_refused_before_the_port(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=ace2k_image()[:-3])
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.opened == 0 and host.gcodes == []


def test_printing_refused(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.printing = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.gcodes == [] and "print" in host.text()


def test_not_confirmed(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.answers = ["n"]
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.gcodes == [] and host.commands == []


def test_no_port_lists_candidates(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "usb-1a86_USB_Single_Serial_TEST-if00" in text and "--port" in text
    assert host.opened == 0


def test_port_held_by_another_program(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    d = host.proc / "77" / "fd"
    d.mkdir(parents=True)
    (host.proc / "77" / "cmdline").write_bytes(b"minicom\0")
    (d / "3").symlink_to(host.port)
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert "minicom" in host.text() and host.opened == 0


def test_image_refused_by_the_bootloader(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    unit.refuse_image = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "still answers" in host.text()
    assert host.running  # started again at exit


def test_no_way_to_stop_klipper(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.initd.unlink()
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.gcodes == [] and "--klipper-stop" in host.text()


def test_update_failure_restarts_klipper_and_says_rerun(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    real = unit._answer

    def drop_chunks(f):
        return None if f.cmd == fl.CMD_IAP_FIRMWARE else real(f)

    unit._answer = drop_chunks
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "run the same command again" in host.text()
    assert host.running


def test_unknown_print_state_refused(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    assert host.klippy_state() == "ready"
    host.objects = lambda: None
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert host.gcodes == [] and host.commands == []
    assert "print state could not be read" in host.text()


def test_chained_units_refused_with_the_port_free(tmp_path):
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, extra_units=1, image=ace2k_image("0.12.0")
    )
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.commands == []
    assert "2 units answered" in host.text()


def test_chained_units_refused_in_path_a(tmp_path):
    clock, unit, host, image = setup(tmp_path, extra_units=1, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert host.running
    assert "2 units answered" in host.text()


def _first_install(tmp_path, override=None):
    clock, unit, host, image = setup(
        tmp_path,
        state="factory",
        image=ace2k_image("0.12.0"),
        holds_when_not_ready=False,
    )
    host.state_override = override
    return clock, unit, host, image


def test_first_install_klipper_error_not_holding_the_port(tmp_path):
    clock, unit, host, image = _first_install(tmp_path)
    assert host.klippy_state() == "error" and not list(host.proc.glob("*"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0" and host.running and host.gcodes == []
    assert len(list((tmp_path / "cal").glob(f"ace-calibration-{UID_TEXT}-*.txt"))) == 1


def test_first_install_klipper_in_startup(tmp_path):
    clock, unit, host, image = _first_install(tmp_path, "startup")
    host.state_override = "startup"
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert unit.ace2k_version == "0.12.0"


def test_first_install_without_a_service_refuses(tmp_path):
    clock, unit, host, image = _first_install(tmp_path)
    host.initd.unlink()
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "--klipper-stop" in host.text() and host.commands == [] and unit.received == []


def test_one_unit_refusal_after_the_reset_does_not_say_nothing_changed(tmp_path):
    clock, unit, host, image = setup(tmp_path, extra_units=1, image=ace2k_image("0.12.0"))
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "nothing changed" not in text
    assert "recovery" in text and "will not connect" in text and "run the tool again" in text


def test_one_unit_refusal_in_the_not_ready_path(tmp_path):
    clock, unit, host, image = setup(
        tmp_path, state="factory", extra_units=1, image=ace2k_image("0.12.0")
    )
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert fl.CMD_IAP_UPGRADE not in unit.received
    assert "2 units answered" in host.text() and host.running


class _LinkFails:
    """Wraps the unit's link: write raises OSError on the Nth IAP_FIRMWARE frame."""

    def __init__(self, unit, nth):
        self.unit, self.nth, self.n = unit, nth, 0
        real = unit.write

        def write(data):
            for f in fl.parse_frames(bytearray(data)):
                if f.cmd == fl.CMD_IAP_FIRMWARE:
                    self.n += 1
                    if self.n == nth:
                        raise OSError("device disappeared")
            real(data)

        unit.write = write


def test_link_error_during_the_update(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    _LinkFails(unit, 3)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "run the same command again" in text and "device disappeared" in text
    assert host.running
    assert host.lines[-1].startswith("Starting Klipper again")


def test_link_error_outside_the_update_is_not_a_traceback(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", configured=False)
    real = unit.write

    def boom(data):
        raise OSError("port gone")

    unit.write = boom
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_FAILED
    assert "Serial link error: port gone" in host.text()
    unit.write = real


def test_unwritable_calibration_dir_refused_before_the_update(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    (tmp_path / "cal").write_text("a file, not a directory")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "calibration could not be written" in host.text()
    assert fl.CMD_IAP_UPGRADE not in unit.received and host.running


def test_failed_start_is_reported_last(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.start_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert host.lines[-1].startswith("Klipper did NOT start (exit 1)")


def test_moonraker_silent_after_the_gcode_is_not_taken_as_not_ready(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    real = host.gcode

    def gcode(script):
        real(script)
        host.moonraker_up = False

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "Moonraker does not answer" in host.text() and host.commands == []


def test_plan_shows_the_stop_and_the_start_commands(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.answers = ["n"]
    run(host, args(image, tmp_path))
    text = host.text()
    assert f"{host.initd} stop" in text and f"{host.initd} start" in text


def test_plan_says_the_tool_starts_klipper_when_the_tool_stopped_it(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.answers = ["n"]
    run(host, args(image, tmp_path))
    text = host.text()
    assert f"start Klipper ({host.initd} start); check it reports ace2k 0.12.0" in text
    assert "you start (or restart) Klipper when asked" not in text


def test_plan_says_the_operator_starts_klipper_when_the_tool_did_not_stop_it(tmp_path):
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


def test_factory_plan_says_klipper_is_left_stopped(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=factory_image())
    host.answers = ["n"]
    run(host, args(image, tmp_path, version="1.1.31"))
    assert "Klipper is left stopped afterwards" in host.text()


def test_still_answers_names_the_factory_firmware(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    real = unit._schedule_commit

    def commit():
        real()
        unit.commit = ("factory", "V1.1.31")

    unit._schedule_commit = commit
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "factory firmware answers after the update" in host.text()


def test_moonraker_down_is_unverified_with_its_own_reason(tmp_path):
    clock, unit, host, image = setup(
        tmp_path, state="factory", configured=False, image=ace2k_image("0.12.0")
    )
    host.moonraker_up = False
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    text = host.text()
    assert "Moonraker does not answer here" in text and "180 s" in text
    assert "If that window passes, the unit stays in recovery" in text


def test_stopped_but_still_held_is_refused_and_klipper_started(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.keep_holder = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "still held open" in host.text()
    assert host.running


# --- review fixes ------------------------------------------------------------------------------
def test_restore_wait_covers_klippers_slow_notice(tmp_path):
    # R1: Klipper takes ~4-6 s to notice the reset; the wait must outlast that
    assert fl.RESTORE_WAIT_S == 15.0
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 8.0
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert unit.ace2k_version == "0.12.0"


def test_a_stale_console_error_is_not_this_refusal(tmp_path):
    # R2: an old `!!` line in the store, no new one, Klipper still ready: ambiguous, not refused
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! an old error from an hour ago")
    host.restore_silent = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "an old error" not in text and "was refused" not in text
    assert "still reports ready" in text and "may or may not have reset" in text
    assert "Nothing was stopped" in text and "all four strobing fast is recovery" in text
    assert host.commands == []


def test_a_new_error_after_an_old_one_is_the_refusal(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! an old error from an hour ago")
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "ACE_RESTORE_STOCK was refused: !! ace2k: refused — " in text
    assert "an old error" not in text
    assert "Klipper still reports ready, so nothing was stopped" in text
    assert "If all four LEDs strobe fast, the unit did reset: run the tool again" in text


def test_the_same_error_text_again_counts_as_new(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    host.console.append("!! ace2k: refused — a subsystem is busy (heating or moving)")
    host.refuse_restore = True
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "was refused" in host.text() and "may or may not" not in host.text()


def test_an_undecodable_reply_mid_update_is_not_a_traceback(tmp_path):
    # R3
    clock, unit, host, image = setup(tmp_path)
    real, n = unit._answer, [0]

    def garble(f):
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


def test_an_undecodable_announcement_reply(tmp_path):
    clock, unit, host, image = setup(tmp_path)
    real = unit._answer
    unit._answer = lambda f: (
        reply(0x00, f.seq, f.cmd, b"\x08\x80") if f.cmd == fl.CMD_IAP_UPGRADE else real(f)
    )
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    assert "does not decode" in host.text()


def test_an_older_image_is_recognised_by_its_own_version_string(tmp_path):
    # R4: --version 0.11, the image's own string says 0.11.0, Klipper reports that
    img = ace2k_image(with_constant=False, extra=b"\x00ace2k 0.11.0\x00")
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "0.11.0"
    assert run(host, args(image, tmp_path, version="0.11")) == fl.EXIT_OK
    assert any(line.endswith("done.") for line in host.lines)


def test_an_older_image_whose_bytes_lack_the_reported_string_fails(tmp_path):
    img = ace2k_image(with_constant=False)
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "9.9.9"
    assert run(host, args(image, tmp_path, version="0.11")) == fl.EXIT_FAILED
    assert "did not report" in host.text()


def test_an_embedded_version_must_match_exactly(tmp_path):
    img = ace2k_image("0.12.0", extra=b"0.99.0")
    clock, unit, host, image = setup(tmp_path, image=img)
    unit.reports_version = "0.99.0"
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED


def test_cannot_check_after_this_run_stopped_klipper_starts_it_again(tmp_path):
    # R5: Klipper not ready and holding the port, [ace2k] configured on ANOTHER port
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.config_port = "/dev/ttyElsewhere"
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert [c[-1] for c in host.commands] == ["stop", "start"] and host.running
    text = host.text()
    flat = " ".join(text.split())
    assert "Klipper was started again with its current config" in flat
    assert "no [ace2k] on this port" in flat
    assert "A NEWLY WRITTEN IMAGE GOES BACK TO RECOVERY IF NO HOST TALKS TO IT WITHIN 180 s" in flat
    assert "If that window passes, the unit stays in recovery" in flat
    assert "restart Klipper within that window" in flat
    warning = text.split("Klipper was started again")[1].splitlines()
    assert all(len(line) <= 95 for line in warning)


def test_a_stop_that_fails_leaves_klipper_alone_path_a(tmp_path):
    # R6
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.stop_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "failed (exit 1): Klipper was not stopped" in text
    assert "had already reset the unit" in text and "nothing changed" not in text
    assert [c[-1] for c in host.commands] == ["stop"]  # no start attempt


def test_a_stop_that_fails_leaves_klipper_alone_not_ready_path(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.stop_rc = 1
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "failed (exit 1): Klipper was not stopped; nothing changed" in text
    assert [c[-1] for c in host.commands] == ["stop"]


@pytest.mark.parametrize("which", ["stop", "start"])
def test_a_missing_service_command_is_not_a_traceback(tmp_path, which):
    # D1
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.raise_on[which] = FileNotFoundError("no such command")
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    if which == "stop":
        assert "Klipper was not stopped" in text and "no such command" in text
        assert [c[-1] for c in host.commands] == ["stop"]
    else:
        assert host.lines[-1].startswith("Klipper did NOT start (no such command): run ")
        assert str(host.initd) in host.lines[-1]


def test_held_after_the_reset_says_the_unit_is_in_recovery(tmp_path, monkeypatch):
    # D2: another program takes the port after Klipper let go
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    real, calls = fl.port_holders, [0]

    def holders(port, proc):
        calls[0] += 1
        return real(port, proc) if calls[0] == 1 else [(99, "minicom")]

    monkeypatch.setattr(fl, "port_holders", holders)
    assert run(host, args(image, tmp_path)) == fl.EXIT_FAILED
    text = host.text()
    assert "held open by another program" in text and "nothing changed" not in text
    assert "had already reset the unit" in text


def test_restart_klipper_branch_when_it_is_up_but_not_ready(tmp_path):
    # D3: Klipper running, not ready after the flash, not stopped by this run
    clock, unit, host, image = setup(
        tmp_path, state="factory", image=ace2k_image("0.12.0"), holds_when_not_ready=False
    )
    host.state_override = "ready"  # as the pre-flash view sees it: no ace2k version yet

    def settle(h, line):
        if line.startswith("Restart Klipper (FIRMWARE_RESTART"):
            h.state_override = None

    host.on_line = settle
    real = unit._schedule_commit

    def commit():
        real()
        host.state_override = "startup"

    unit._schedule_commit = commit
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_OK
    assert any(line.startswith("Restart Klipper (FIRMWARE_RESTART") for line in host.lines)
    assert host.commands == []


def test_unverifiable_flash_with_a_failed_start_tries_once(tmp_path):
    clock, unit, host, image = setup(tmp_path, state="factory", image=ace2k_image("0.12.0"))
    host.config_port = "/dev/ttyElsewhere"
    host.start_rc = 1
    assert run(host, args(image, tmp_path, port=host.port)) == fl.EXIT_UNVERIFIED
    assert [c[-1] for c in host.commands] == ["stop", "start"]
    assert "was started again" not in host.text()
    assert host.lines[-1].startswith("Klipper did NOT start (exit 1)")


def test_an_unrelated_error_line_does_not_end_the_wait(tmp_path):
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 1.0
    real = host.gcode

    def gcode(script):
        host.console.append("!! some unrelated error")
        real(script)

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()
    assert [c[-1] for c in host.commands] == ["stop", "start"]


def test_a_five_second_lag_after_an_unrelated_error_still_proceeds(tmp_path):
    assert fl.REFUSAL_SETTLE_S == 6.0
    clock, unit, host, image = setup(tmp_path, image=ace2k_image("0.12.0"))
    host.restore_lag = 5.0
    real = host.gcode

    def gcode(script):
        host.console.append("!! some unrelated error")
        real(script)

    host.gcode = gcode
    assert run(host, args(image, tmp_path)) == fl.EXIT_OK
    assert "was refused" not in host.text()
