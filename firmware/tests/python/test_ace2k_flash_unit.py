"""ace2k_flash: talking to the unit and to the host, piece by piece."""

import io
import json
import os
import sys
import types
from types import SimpleNamespace

import pytest
from flash_sim import PAIRS, UID_TEXT, FakeClock, SimUnit, fl, reply


class Answer:
    """A link stand-in whose every query gets the same reply payload."""

    def __init__(self, payload: bytes):
        self.payload = payload

    def ask(self, cmd, payload=b"", timeout=1.0):
        return fl.Frame(0x80, 1, cmd, self.payload)


class EchoLink:
    """A line that returns the request's own bytes, ahead of the reply when it answers."""

    def __init__(self, answer: bool):
        self.answer, self.pending = answer, b""

    def clear(self):
        self.pending = b""

    def write(self, data: bytes):
        request = fl.parse_frames(bytearray(data))[0]
        body = reply(0x80, request.seq, request.cmd, b"\x0a\x01A") if self.answer else b""
        self.pending = data + body

    def read(self) -> bytes:
        data, self.pending = self.pending, b""
        return data


def test_identify_retries_past_a_lost_first_query():
    unit = SimUnit(FakeClock(), state="recovery")
    unit.swallow = 1
    seen = fl.identify(fl.Unit(unit, unit.clock))
    assert (seen.kind, seen.boot_version) == ("recovery", "V1.0.2")
    assert unit.received == [fl.CMD_GET_INFO, fl.CMD_GET_INFO]


def test_identify_factory_and_silence():
    unit = SimUnit(FakeClock(), state="factory")
    seen = fl.identify(fl.Unit(unit, unit.clock))
    assert (seen.kind, seen.version) == ("factory", "V1.1.31")
    off = SimUnit(FakeClock(), state="off")
    assert fl.identify(fl.Unit(off, off.clock)) is None
    assert off.received == [fl.CMD_GET_INFO] * fl.IDENTIFY_TRIES


def test_uid_and_calibration():
    unit = SimUnit(FakeClock(), state="factory")
    u = fl.Unit(unit, unit.clock)
    assert fl.read_uid(u) == UID_TEXT
    raw, pairs = fl.read_calibration(u)
    assert pairs == PAIRS
    text = fl.calibration_text(raw, pairs)
    assert text.splitlines()[0] == f"# GET_MOTOR_STATUS raw payload: {raw.hex()}"
    assert text.splitlines()[1] == " 0   600  1100"
    assert fl.parse_calibration(text) == PAIRS


def test_calibration_must_be_16_pairs():
    class Short:
        def ask(self, cmd, payload=b"", timeout=1.0):
            return fl.Frame(0x80, 1, cmd, fl.pb_bytes(1, fl.pb_uint(1, 5)))

    with pytest.raises(fl.RefusedError, match="1 pairs"):
        fl.read_calibration(Short())


def test_port_holders(tmp_path):
    port = tmp_path / "ttyACM0"
    port.write_text("")
    proc = tmp_path / "proc"
    (proc / "10" / "fd").mkdir(parents=True)
    (proc / "10" / "cmdline").write_bytes(b"python3\0/x/klippy/klippy.py\0")
    (proc / "10" / "fd" / "3").symlink_to(port)
    (proc / "11" / "fd").mkdir(parents=True)
    (proc / "11" / "cmdline").write_bytes(b"bash\0")
    (proc / "self").mkdir()
    assert fl.port_holders(str(port), proc) == [(10, "python3 /x/klippy/klippy.py")]
    assert fl.is_klippy("python3 /x/klippy/klippy.py")
    with pytest.raises(fl.RefusedError, match="Klipper host"):
        fl.port_holders(str(port), tmp_path / "noproc")


def test_candidate_ports(tmp_path):
    for name in (
        "usb-1a86_USB_Single_Serial_B-if00",
        "usb-1a86_USB_Single_Serial_A-if00",
        "usb-1a86_USB2.0-Serial-if00-port0",  # a printer's own CH340 mainboard: not the cable
        "usb-Klipper_x",
    ):
        (tmp_path / name).write_text("")
    assert [p.rsplit("/", 1)[1] for p in fl.candidate_ports(tmp_path)] == [
        "usb-1a86_USB_Single_Serial_A-if00",
        "usb-1a86_USB_Single_Serial_B-if00",
    ]


def test_find_service(tmp_path):
    initd = tmp_path / "S60klipper"

    def run_ok(cmd, capture_output=False):
        return SimpleNamespace(returncode=0)

    def run_fail(cmd, capture_output=False):
        return SimpleNamespace(returncode=1)

    systemd = fl.find_service(None, None, lambda n: "/bin/systemctl", run_ok, initd)
    assert systemd.stop == ["systemctl", "stop", "klipper"]
    assert fl.find_service(None, None, lambda n: None, run_ok, initd) is None
    initd.write_text("")
    s = fl.find_service(None, None, lambda n: "/bin/systemctl", run_fail, initd)
    assert (s.stop, s.start) == ([str(initd), "stop"], [str(initd), "start"])
    given = fl.find_service(
        "sudo service klipper stop", "sudo service klipper start", None, None, initd
    )
    assert given.stop == ["sudo", "service", "klipper", "stop"]
    with pytest.raises(fl.RefusedError):
        fl.find_service("x", None, None, None, initd)


def test_moonraker_client():
    replies = {
        "/server/info": {"klippy_state": "ready"},
        "/printer/objects/query?print_stats&ace2k&configfile=settings": {
            "status": {
                "configfile": {
                    "settings": {"ace2k": {"mcu": "unit"}, "mcu unit": {"serial": "/dev/x"}}
                },
                "print_stats": {"state": "paused"},
                "ace2k": {"version": "0.12.0"},
            }
        },
        "/server/gcode_store?count=20": {
            "gcode_store": [{"message": "ok"}, {"message": "!! ace2k: refused"}]
        },
    }
    seen = []

    def opener(req, timeout):
        path = req.full_url[len("http://m") :]
        seen.append((path, req.data))
        if path not in replies:
            return io.BytesIO(json.dumps({"result": "ok"}).encode())
        return io.BytesIO(json.dumps({"result": replies[path]}).encode())

    mr = fl.Moonraker("http://m/", opener)
    view = fl.klipper_view(mr)
    assert (view.state, view.printing, view.ace2k_port, view.ace2k_version) == (
        "ready",
        True,
        "/dev/x",
        "0.12.0",
    )
    assert mr.console_errors() == ["!! ace2k: refused"]
    mr.gcode("ACE_RESTORE_STOCK")
    assert seen[-1] == ("/printer/gcode/script?script=ACE_RESTORE_STOCK", b"")

    def down(req, timeout):
        raise OSError("refused")

    assert fl.klipper_view(fl.Moonraker("http://m", down)).state is None


@pytest.mark.parametrize("state, printing", [("ready", None), ("error", False)])
def test_unanswered_objects_query_is_unknown_not_idle(state, printing):
    # Moonraker answers the state but not the print query: when Klipper is ready that is unknown
    def opener(req, timeout):
        if req.full_url.endswith("/server/info"):
            return io.BytesIO(json.dumps({"result": {"klippy_state": state}}).encode())
        raise OSError("timed out")

    view = fl.klipper_view(fl.Moonraker("http://m", opener))
    assert (view.state, view.printing) == (state, printing)


@pytest.mark.skipif(os.geteuid() == 0, reason="root reads every /proc fd")
@pytest.mark.parametrize(
    "cmdline, refused",
    [(b"python3\0/home/klipper/klippy/klippy.py\0", True), (b"bash\0", False)],
)
def test_unreadable_fd_refuses_for_klipper_and_skips_others(tmp_path, cmdline, refused):
    port = tmp_path / "ttyACM0"
    port.write_text("")
    d = tmp_path / "proc" / "20"
    (d / "fd").mkdir(parents=True)
    (d / "cmdline").write_bytes(cmdline)
    (d / "fd").chmod(0)
    try:
        if refused:
            with pytest.raises(fl.RefusedError, match="pid 20"):
                fl.port_holders(str(port), tmp_path / "proc")
        else:
            assert fl.port_holders(str(port), tmp_path / "proc") == []
    finally:
        (d / "fd").chmod(0o755)


def test_serial_link_opens_exclusively_and_refuses_a_busy_port(monkeypatch):
    calls = []

    class SerialBusyError(Exception):
        pass

    class Serial:
        def __init__(self, port, baud, **kw):
            calls.append((port, baud, kw))
            if port == "busy":
                raise SerialBusyError("could not open port busy: Permission denied")

    fake = types.ModuleType("serial")
    fake.Serial, fake.SerialException = Serial, SerialBusyError
    monkeypatch.setitem(sys.modules, "serial", fake)
    fl.SerialLink("/dev/ok")
    assert calls == [("/dev/ok", fl.BAUD, {"timeout": 0, "exclusive": True})]
    with pytest.raises(fl.RefusedError, match="cannot be opened"):
        fl.SerialLink("busy")


def test_ask_skips_the_echo_of_its_own_request():
    clock = FakeClock()
    frame = fl.Unit(EchoLink(True), clock).ask(fl.CMD_GET_INFO)
    assert frame is not None and (frame.addr, frame.payload) == (0x80, b"\x0a\x01A")
    assert fl.Unit(EchoLink(False), clock).ask(fl.CMD_GET_INFO) is None


def test_count_units_sees_every_unit_on_the_port():
    for state in ("factory", "recovery"):
        for extra, want in ((0, 1), (1, 2)):
            unit = SimUnit(FakeClock(), state=state, extra_units=extra)
            assert fl.count_units(fl.Unit(unit, unit.clock)) == want, (state, extra)
    silent = SimUnit(FakeClock(), state="off")
    assert fl.count_units(fl.Unit(silent, silent.clock)) == 0


def test_discover_refuses_an_answer_that_does_not_decode():
    for payload in (b"\x0a", fl.pb_bytes(1, b"xy")):
        with pytest.raises(fl.RefusedError, match="DISCOVER does not decode"):
            fl.read_uid(Answer(payload))


def test_calibration_refuses_a_pair_that_is_not_a_number():
    with pytest.raises(fl.RefusedError, match="does not decode"):
        fl.read_calibration(Answer(fl.pb_bytes(1, fl.pb_bytes(1, b"xy"))))


@pytest.mark.parametrize("text, line", [(" 0   600  1100\n 1 x 2\n", 2), ("# ok\n 0 600\n", 2)])
def test_calibration_file_refuses_a_bad_line(text, line):
    with pytest.raises(fl.RefusedError, match=f"line {line} does not parse"):
        fl.parse_calibration(text)


def test_port_holders_resolves_the_port_once(tmp_path, monkeypatch):
    # R8: one realpath for the port; the fd links are compared raw when they already match
    real_dir = tmp_path.resolve()
    target = real_dir / "ttyACM0"
    target.write_text("")
    link = real_dir / "by-id-port"
    link.symlink_to(target)
    proc = real_dir / "proc"
    for pid in ("10", "11", "12"):
        (proc / pid / "fd").mkdir(parents=True)
        (proc / pid / "cmdline").write_bytes(b"holder\0")
    (proc / "10" / "fd" / "3").symlink_to(target)
    (proc / "11" / "fd" / "3").symlink_to(link)  # a link that needs resolving
    (proc / "12" / "fd" / "3").symlink_to("socket:[1]")  # not a path
    calls = []
    real = os.path.realpath
    monkeypatch.setattr(os.path, "realpath", lambda p: calls.append(p) or real(p))
    assert sorted(pid for pid, _ in fl.port_holders(str(link), proc)) == [10, 11]
    calls.clear()
    fl.port_holders(str(link), proc)
    assert len(calls) == 2  # the port once, and the one fd link that differs from it
