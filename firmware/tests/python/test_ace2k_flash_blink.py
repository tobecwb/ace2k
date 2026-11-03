"""The flasher's --identify: blink the unit on a port, change nothing."""

from __future__ import annotations

from pathlib import Path

import pytest
from flash_sim import SimHost, args, fl, setup


def identify(tmp: Path, host: SimHost, **kw: object) -> int:
    a = args(None, tmp, port=host.port, identify=True, **kw)
    return fl.Flasher(a, host.deps()).run()


def test_factory_firmware_blinks_with_its_own_command(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="factory", configured=False)
    assert identify(tmp_path, host) == fl.EXIT_OK
    assert unit.blinks == 1
    assert set(unit.received) <= {fl.CMD_GET_INFO, fl.CMD_FLASH_LED}
    fields = fl.pb_fields(unit.led_payloads[0])
    assert [fields[n][0] for n in (1, 2, 3, 4, 5, 6)] == [0x0F, 2, 0, 4, 0, 0]
    assert "blinks its four LEDs (the factory firmware)" in host.text()


def test_ace2k_blinks_with_ace2k_identify(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="ace2k", configured=False)
    assert identify(tmp_path, host) == fl.EXIT_OK
    assert unit.blinks == 1 and 79 in unit.asked
    assert "blinks its four LEDs (ace2k" in host.text()


def test_ace2k_without_the_command_cannot_blink(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="ace2k", configured=False)
    unit.has_identify = False
    assert identify(tmp_path, host) == fl.EXIT_NO_BLINK
    assert "cannot blink" in host.text() and "older" in host.text()


def test_recovery_cannot_blink(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="recovery", configured=False)
    assert identify(tmp_path, host) == fl.EXIT_NO_BLINK
    assert unit.blinks == 0
    assert "recovery" in host.text()


def test_nothing_answers(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="off", configured=False)
    assert identify(tmp_path, host) == fl.EXIT_NO_ANSWER
    assert unit.blinks == 0


def test_klipper_ready_on_the_port_is_refused(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="ace2k", configured=True)
    assert identify(tmp_path, host) == fl.EXIT_UNCHANGED
    assert unit.blinks == 0 and unit.received == [] and unit.asked == []
    assert "Klipper" in host.text()


@pytest.mark.parametrize(
    "argv", [["--identify"], ["--identify", "--port", "/dev/x", "img.bin"], []]
)
def test_identify_needs_a_port_and_no_image(argv: list[str]) -> None:
    with pytest.raises(SystemExit) as e:
        fl.main(argv)
    assert e.value.code == fl.EXIT_UNCHANGED


def test_the_allowlist_gains_only_flash_led() -> None:
    assert fl.CMD_FLASH_LED == 70
    assert {0, 2, 3, 4, 7, 78, 70} == fl.wire.ALLOWED_COMMANDS
    assert fl.EXIT_NO_BLINK == 5


def test_factory_that_does_not_answer_the_blink_cannot_blink(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="factory", configured=False)
    unit.answers_blink = False
    assert identify(tmp_path, host) == fl.EXIT_NO_BLINK
    assert "did not answer the blink command" in host.text()
    assert "blinks its four LEDs" not in host.text()


def test_ace2k_that_does_not_answer_identify_cannot_blink(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="ace2k", configured=False)
    unit.answers_blink = False
    assert identify(tmp_path, host) == fl.EXIT_NO_BLINK
    assert "did not answer ace2k_identify" in host.text()
    assert "older than" not in host.text()
    assert "blinks its four LEDs" not in host.text()


def test_a_held_port_is_refused_and_nothing_is_sent(tmp_path: Path) -> None:
    _, unit, host, _ = setup(tmp_path, state="factory", configured=False)
    holder = host.proc / "777"
    (holder / "fd").mkdir(parents=True)
    (holder / "cmdline").write_bytes(b"/usr/bin/python3\0other.py\0")
    (holder / "fd" / "3").symlink_to(host.port)
    assert identify(tmp_path, host) == fl.EXIT_UNCHANGED
    assert unit.received == [] and unit.asked == [] and unit.blinks == 0
    assert "held open" in host.text()
