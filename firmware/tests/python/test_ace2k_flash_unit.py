"""ace2k_flash: talking to the unit and to the host, piece by piece."""

from __future__ import annotations

import importlib.util
import io
import json
import os
import sys
import types
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace
from typing import TYPE_CHECKING, NoReturn

import pytest
from flash_sim import PAIRS, UID_TEXT, FakeClock, SimUnit, fl, reply

if TYPE_CHECKING:
    from types import ModuleType


class Answer:
    """A link stand-in whose every query gets the same reply payload."""

    def __init__(self, payload: bytes) -> None:
        self.payload = payload

    def ask(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> fl.Frame:
        return fl.Frame(0x80, 1, cmd, self.payload)


class EchoLink:
    """A line that returns the request's own bytes, ahead of the reply when it answers."""

    def __init__(self, answer: bool) -> None:
        self.answer, self.pending = answer, b""

    def clear(self) -> None:
        self.pending = b""

    def write(self, data: bytes) -> None:
        request = fl.parse_frames(bytearray(data))[0]
        body = reply(0x80, request.seq, request.cmd, b"\x0a\x01A") if self.answer else b""
        self.pending = data + body

    def read(self) -> bytes:
        data, self.pending = self.pending, b""
        return data


def test_identify_retries_past_a_lost_first_query() -> None:
    unit = SimUnit(FakeClock(), state="recovery")
    unit.swallow = 1
    seen = fl.identify(fl.Unit(unit, unit.clock))
    assert (seen.kind, seen.boot_version) == ("recovery", "V1.0.2")
    assert unit.received == [fl.CMD_GET_INFO, fl.CMD_GET_INFO]


def test_identify_factory_and_silence() -> None:
    unit = SimUnit(FakeClock(), state="factory")
    seen = fl.identify(fl.Unit(unit, unit.clock))
    assert (seen.kind, seen.version) == ("factory", "V1.1.31")
    off = SimUnit(FakeClock(), state="off")
    assert fl.identify(fl.Unit(off, off.clock)) is None
    assert off.received == [fl.CMD_GET_INFO] * fl.IDENTIFY_TRIES


def test_uid_and_calibration() -> None:
    unit = SimUnit(FakeClock(), state="factory")
    u = fl.Unit(unit, unit.clock)
    assert fl.read_uid(u) == UID_TEXT
    raw, pairs = fl.read_calibration(u)
    assert pairs == PAIRS
    text = fl.calibration_text(raw, pairs)
    assert text.splitlines()[0] == f"# GET_MOTOR_STATUS raw payload: {raw.hex()}"
    assert text.splitlines()[1] == " 0   600  1100"
    assert fl.parse_calibration(text) == PAIRS


def test_calibration_must_be_16_pairs() -> None:
    class Short:
        def ask(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> fl.Frame:
            return fl.Frame(0x80, 1, cmd, fl.pb_bytes(1, fl.pb_uint(1, 5)))

    with pytest.raises(fl.RefusedError, match="1 pairs"):
        fl.read_calibration(Short())


def test_port_holders(tmp_path: Path) -> None:
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


def test_candidate_ports(tmp_path: Path) -> None:
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


def test_find_service(tmp_path: Path) -> None:
    initd = tmp_path / "S60klipper"

    def run_ok(
        cmd: list[str], capture_output: bool = False, timeout: float | None = None
    ) -> SimpleNamespace:
        return SimpleNamespace(returncode=0)

    def run_fail(
        cmd: list[str], capture_output: bool = False, timeout: float | None = None
    ) -> SimpleNamespace:
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


def test_moonraker_client() -> None:
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
            "gcode_store": [
                {"message": "ok", "time": 10.5, "type": "response"},
                {"message": "!! ace2k: refused", "time": 11.25, "type": "response"},
            ]
        },
    }
    seen = []

    def opener(req: urllib.request.Request, timeout: float) -> io.BytesIO:
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
    assert mr.console_lines() == [(10.5, "ok"), (11.25, "!! ace2k: refused")]
    mr.gcode("ACEK_RESTORE_STOCK")
    assert seen[-1] == ("/printer/gcode/script?script=ACEK_RESTORE_STOCK", b"")

    def down(req: urllib.request.Request, timeout: float) -> NoReturn:
        raise OSError("refused")

    assert fl.klipper_view(fl.Moonraker("http://m", down)).state is None
    assert fl.Moonraker("http://m", down).console_lines() is None
    assert fl.Moonraker("http://m", down).gcode("ACEK_RESTORE_STOCK") is None


def http_error(code: int, body: bytes) -> urllib.error.HTTPError:
    return urllib.error.HTTPError("http://m/x", code, "Bad Request", {}, io.BytesIO(body))


@pytest.mark.parametrize(
    "body, expected",
    [
        # Moonraker's answer to a script that raised a G-code error
        (
            {"error": {"code": 400, "message": "ace2k: refused — busy", "traceback": "..."}},
            "ace2k: refused — busy",
        ),
        ({"error": "Klippy Disconnected"}, "Klippy Disconnected"),
        (None, "HTTP 503"),  # no readable message: still an error, named by its status
    ],
)
def test_gcode_returns_the_error_moonraker_answers(
    body: dict[str, object] | None, expected: str
) -> None:
    raw = b"<html>" if body is None else json.dumps(body).encode()
    errors = []

    def opener(req: urllib.request.Request, timeout: float) -> NoReturn:
        errors.append(http_error(503 if body is None else 400, raw))
        raise errors[-1]

    answer = fl.Moonraker("http://m", opener).gcode("ACEK_RESTORE_STOCK")
    assert answer == fl.MoonrakerError(expected)
    assert errors[0].fp.closed  # the error's body is closed once read


def test_moonraker_error_first_line_and_refusal() -> None:
    multi = fl.MoonrakerError("ace2k: refused — busy\nmore")
    assert multi.first_line == "ace2k: refused — busy" and multi.is_refusal
    assert not fl.MoonrakerError(
        "ace2k: bootloader enter: no answer from the unit; check the link (x)"
    ).is_refusal
    assert not fl.MoonrakerError("Klippy Disconnected").is_refusal
    # ran: an error the ace2k module raised; anything else is Moonraker's or Klipper's own
    assert multi.ran
    assert fl.MoonrakerError("ace2k: bootloader enter: no answer from the unit; x").ran
    assert not fl.MoonrakerError("Klippy Disconnected").ran
    assert not fl.MoonrakerError("HTTP 503").ran
    assert not fl.MoonrakerError("\nPrinter is not ready\nThe klippy host ...").ran
    assert not fl.MoonrakerError('Unknown command:"ACEK_RESTORE_STOCK"').ran


def test_gcode_tells_ran_from_unanswered() -> None:
    def ran(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        return io.BytesIO(json.dumps({"result": "ok"}).encode())

    def lost(req: urllib.request.Request, timeout: float) -> NoReturn:
        raise TimeoutError("timed out")

    def garbled(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        return io.BytesIO(b"<html>ok</html>")  # a 200 the tool cannot read: it still ran

    assert fl.Moonraker("http://m", ran).gcode("ACEK_RESTORE_STOCK") is True
    assert fl.Moonraker("http://m", garbled).gcode("ACEK_RESTORE_STOCK") is True
    assert fl.Moonraker("http://m", lost).gcode("ACEK_RESTORE_STOCK") is None
    assert fl.Moonraker("http://m", garbled).state() is None  # the other endpoints: unchanged


def test_other_endpoints_keep_none_on_an_http_error() -> None:
    errors = []

    def opener(req: urllib.request.Request, timeout: float) -> NoReturn:
        errors.append(http_error(503, b"{}"))
        raise errors[-1]

    assert fl.Moonraker("http://m", opener).state() is None
    assert errors[0].fp.closed


@pytest.mark.parametrize("state, printing", [("ready", None), ("error", False)])
def test_unanswered_objects_query_is_unknown_not_idle(state: str, printing: bool | None) -> None:
    # Moonraker answers the state but not the print query: when Klipper is ready that is unknown
    def opener(req: urllib.request.Request, timeout: float) -> io.BytesIO:
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
def test_unreadable_fd_refuses_for_klipper_and_skips_others(
    tmp_path: Path, cmdline: bytes, refused: bool
) -> None:
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


def test_serial_link_opens_exclusively_and_refuses_a_busy_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []

    class SerialBusyError(Exception):
        pass

    class Serial:
        def __init__(self, port: str, baud: int, **kw: object) -> None:
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


def test_ask_skips_the_echo_of_its_own_request() -> None:
    clock = FakeClock()
    frame = fl.Unit(EchoLink(True), clock).ask(fl.CMD_GET_INFO)
    assert frame is not None and (frame.addr, frame.payload) == (0x80, b"\x0a\x01A")
    assert fl.Unit(EchoLink(False), clock).ask(fl.CMD_GET_INFO) is None


def test_count_units_sees_every_unit_on_the_port() -> None:
    for state in ("factory", "recovery"):
        for extra, want in ((0, 1), (1, 2)):
            unit = SimUnit(FakeClock(), state=state, extra_units=extra)
            assert fl.count_units(fl.Unit(unit, unit.clock)) == want, (state, extra)
    silent = SimUnit(FakeClock(), state="off")
    assert fl.count_units(fl.Unit(silent, silent.clock)) == 0


def test_discover_refuses_an_answer_that_does_not_decode() -> None:
    for payload in (b"\x0a", fl.pb_bytes(1, b"xy")):
        with pytest.raises(fl.RefusedError, match="DISCOVER does not decode"):
            fl.read_uid(Answer(payload))


def test_calibration_refuses_a_pair_that_is_not_a_number() -> None:
    with pytest.raises(fl.RefusedError, match="does not decode"):
        fl.read_calibration(Answer(fl.pb_bytes(1, fl.pb_bytes(1, b"xy"))))


@pytest.mark.parametrize("text, line", [(" 0   600  1100\n 1 x 2\n", 2), ("# ok\n 0 600\n", 2)])
def test_calibration_file_refuses_a_bad_line(text: str, line: int) -> None:
    with pytest.raises(fl.RefusedError, match=f"line {line} does not parse"):
        fl.parse_calibration(text)


def test_port_holders_resolves_the_port_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # One realpath for the port; the fd links are compared raw when they already match
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


def test_commands_reads_klippers_command_help() -> None:
    # Moonraker's GET /printer/gcode/help: {"result": {CMD: help}}, the commands registered with
    # a description (both restore names carry one)
    seen = []

    def opener(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        seen.append(req.full_url)
        help_ = {"ACEK_RESTORE_STOCK": "Reset the unit into recovery", "G28": "Home"}
        return io.BytesIO(json.dumps({"result": help_}).encode())

    assert fl.Moonraker("http://m", opener).gcode_commands() == {"ACEK_RESTORE_STOCK", "G28"}
    assert seen == ["http://m/printer/gcode/help"]

    def down(req: urllib.request.Request, timeout: float) -> NoReturn:
        raise OSError("refused")

    def refused(req: urllib.request.Request, timeout: float) -> NoReturn:
        raise http_error(404, b"{}")

    def garbled(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        return io.BytesIO(json.dumps({"result": ["ACEK_RESTORE_STOCK"]}).encode())

    assert fl.Moonraker("http://m", down).gcode_commands() is None
    assert fl.Moonraker("http://m", refused).gcode_commands() is None
    assert fl.Moonraker("http://m", garbled).gcode_commands() is None


def test_moonraker_queries_named_units_by_the_names_the_config_lists() -> None:
    settings = {
        "ace2k a": {"mcu": "ace2k_a"},
        "mcu ace2k_a": {"serial": "/dev/a"},
        "ace2k b": {"mcu": "ace2k_b"},
        "mcu ace2k_b": {"serial": "/dev/b"},
        "ace2k_feed a": {},  # a module's section, not a unit
    }
    replies = {
        "/server/info": {"klippy_state": "ready"},
        "/printer/objects/query?print_stats&ace2k&configfile=settings": {
            "status": {"configfile": {"settings": settings}, "print_stats": {"state": "standby"}}
        },
        # Klipper keeps the section's spelling for the object: [ace2k a] and [ace2k B]
        "/printer/objects/list": {"objects": ["ace2k a", "ace2k B", "ace2k_feed B", "mcu"]},
        "/printer/objects/query?ace2k%20B&ace2k%20a": {
            "status": {"ace2k a": {"version": "0.12.0"}, "ace2k B": {"version": "0.11.0"}}
        },
    }

    def opener(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        path = req.full_url[len("http://m") :]
        return io.BytesIO(json.dumps({"result": replies.get(path, "ok")}).encode())

    mr = fl.Moonraker("http://m/", opener)
    b = fl.klipper_view(mr, "/dev/b")
    assert (b.ace2k_unit, b.ace2k_port, b.ace2k_version, b.ace2k_configured) == (
        "b",
        "/dev/b",
        "0.11.0",
        True,
    )
    none = fl.klipper_view(mr)
    assert none.ace2k_port is None and none.ace2k_units == {"a": "/dev/a", "b": "/dev/b"}
    assert fl.klipper_view(mr, "/dev/elsewhere").ace2k_units == {"a": "/dev/a", "b": "/dev/b"}


def test_a_named_units_error_is_the_modules() -> None:
    # with several units the module names the unit: "ace2k a: …"
    refused = fl.MoonrakerError("ace2k a: refused — a subsystem is busy (heating or moving)")
    assert refused.is_refusal and refused.ran
    lost = fl.MoonrakerError("ace2k b: ace2k_bootloader_enter: no answer from the unit")
    assert lost.ran and not lost.is_refusal
    assert fl.MoonrakerError("ace2k: refused — busy").is_refusal
    other = fl.MoonrakerError("Klipper not ready")
    assert not other.ran and not other.is_refusal
    assert not fl.MoonrakerError("ace2k_feed a: x").ran  # a module's object name, not the unit's


def test_a_named_units_status_without_moonrakers_object_list_takes_the_configs_names() -> None:
    settings = {"ace2k a": {"mcu": "ace2k_a"}, "mcu ace2k_a": {"serial": "/dev/a"}}
    for listed in (None, ["not", "a", "dict"], {"objects": "nope"}):
        replies = {
            "/server/info": {"klippy_state": "ready"},
            "/printer/objects/query?print_stats&ace2k&configfile=settings": {
                "status": {"configfile": {"settings": settings}, "print_stats": {}}
            },
            "/printer/objects/query?ace2k%20a": {"status": {"ace2k a": {"version": "0.12.0"}}},
        }
        if listed is not None:
            replies["/printer/objects/list"] = listed

        def opener(
            req: urllib.request.Request, timeout: float, replies: dict[str, object] = replies
        ) -> io.BytesIO:
            path = req.full_url[len("http://m") :]
            if path not in replies:
                raise OSError("no answer")
            return io.BytesIO(json.dumps({"result": replies[path]}).encode())

        view = fl.klipper_view(fl.Moonraker("http://m/", opener), "/dev/a")
        assert (view.ace2k_unit, view.ace2k_version) == ("a", "0.12.0"), listed


# --- the new image's link: Klipper's protocol, checked against Klipper's own code ---------------


def klipper_msgproto() -> ModuleType:
    """Klipper's own klippy/msgproto.py, loaded by its path (another module may carry the name)."""
    path = Path(__file__).resolve().parents[2] / "klipper" / "klippy" / "msgproto.py"
    spec = importlib.util.spec_from_file_location("klipper_msgproto", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("data", [b"", b"\x00", b"\x05\x10\x01\x00", bytes(range(60))])
def test_the_block_crc_is_klippers(data: bytes) -> None:
    hi, lo = klipper_msgproto().crc16_ccitt(data)
    assert fl.klipper_crc(data) == (hi << 8) | lo


@pytest.mark.parametrize(
    "value", [0, 1, 0x5F, 0x60, 0x2FFF, 0x3000, 0x17FFFF, 0x180000, 65535, 2**32 - 1]
)
def test_the_integer_encoding_is_klippers(value: int) -> None:
    out = []
    klipper_msgproto().PT_uint32().encode(out, value)
    assert fl.klipper_vlq(value) == bytes(out)
    assert fl.parse_klipper_vlq(bytes(out), 0) == (value, len(out))


def test_a_block_round_trips_and_noise_before_it_is_dropped() -> None:
    payload = fl.klipper_vlq(1) + fl.klipper_vlq(40) + fl.klipper_vlq(40)
    block = fl.klipper_block(3, payload)
    assert block[0] == len(block) and block[1] == 0x13 and block[-1] == fl.KLIPPER_SYNC
    buf = bytearray(b"\x7e\xff\x02" + block + block[:4])
    assert fl.parse_klipper_blocks(buf) == [(3, payload)]
    assert bytes(buf) == block[:4]  # an incomplete block waits for its rest


def test_the_flasher_reads_the_dictionary_and_proves_the_image() -> None:
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    k = fl.KlipperUnit(unit, clock)
    dictionary = k.dictionary(clock.now() + 30.0)
    assert dictionary["commands"]["ace2k_version"] == 77
    assert k.version(dictionary) == "0.12.0" and k.proven(dictionary) is True


def test_the_dictionary_is_read_after_noise_on_the_line() -> None:
    """The factory protocol's probe leaves bytes the MCU drops up to the next sync byte. A
    question with no sync byte before it would go with them, unanswered."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.needs_sync = True
    k = fl.KlipperUnit(unit, clock)
    dictionary = k.dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77


def test_a_question_noise_took_whole_is_sent_again() -> None:
    """Noise can take a whole block, and the MCU then says nothing about it: no refusal comes
    before a valid block. The question goes again, once, at the same sequence."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.lose_blocks = 1
    writes: list[bytes] = []
    write = unit.write
    unit.write = lambda data: (writes.append(data), write(data))[1]
    k = fl.KlipperUnit(unit, clock)
    dictionary = k.dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77
    first, second = (fl.parse_klipper_blocks(bytearray(w))[0] for w in writes[:2])
    assert first == second == (0, first[1])  # the same identify, at the same sequence


@pytest.mark.parametrize(("expects", "lost"), [(1, 0), (5, 1), (1, 1)])
def test_a_session_before_and_a_lost_block_still_read_the_dictionary(
    expects: int, lost: int
) -> None:
    """A session before left the MCU waiting for another number: one that is the flasher's own
    next (its refusal looks like an acknowledgement), or any other after a block noise took."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.mcu_seq, unit.lose_blocks = expects, lost
    dictionary = fl.KlipperUnit(unit, clock).dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77


def test_a_block_a_collision_broke_does_not_keep_the_unit_deaf() -> None:
    """A collision on the half-duplex line can leave the MCU waiting for the rest of a block that
    never comes, up to its longest. The question that goes again first fills that wait."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.awaiting = fl.klipper.KLIPPER_MAX - 1
    dictionary = fl.KlipperUnit(unit, clock).dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77


def test_a_piece_of_the_dictionary_lost_twice_is_asked_again() -> None:
    """The unit already answered: a piece lost twice in a row (its question and the question
    again) is asked once more while the deadline allows."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.lose_at = {3, 4}
    dictionary = fl.KlipperUnit(unit, clock).dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77


def test_a_first_question_lost_twice_is_asked_again_when_the_unit_was_heard() -> None:
    """A refusal shows the image is there: its first question, lost twice after it, goes once
    more.  Nothing heard at all (a unit that is off) still ends the read at once."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.mcu_seq, unit.lose_at = 5, {1, 2}
    dictionary = fl.KlipperUnit(unit, clock).dictionary(clock.now() + 30.0)
    assert dictionary is not None and dictionary["commands"]["ace2k_version"] == 77


def test_a_question_lost_twice_is_no_answer() -> None:
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    unit.lose_blocks = 2
    assert fl.KlipperUnit(unit, clock).dictionary(clock.now() + 30.0) is None


def test_the_late_ack_of_the_question_before_is_not_a_refusal() -> None:
    """The MCU answers, then acknowledges: the acknowledgement of each question may come in
    while the next is asked. Each question still goes out once."""
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    writes = []
    write = unit.write
    unit.write = lambda data: (writes.append(data), write(data))[1]
    read = unit.read

    def answer_only(
        *a: object,
    ) -> bytes:  # the acknowledgement held back to the next question's read
        data = read()
        blocks = fl.parse_klipper_blocks(bytearray(data))
        unit.pending = b"".join(fl.klipper_block(q, b) for q, b in blocks if not b)
        return b"".join(fl.klipper_block(q, b) for q, b in blocks if b)

    unit.read = answer_only
    unit.clear = lambda: None  # the late acknowledgement survives into the next question
    k = fl.KlipperUnit(unit, clock)
    dictionary = k.dictionary(clock.now() + 30.0)
    assert dictionary is not None
    asked = [fl.parse_klipper_blocks(bytearray(w))[0][1] for w in writes]
    assert len(asked) == len(set(asked))  # no question sent twice


def test_an_answer_with_a_negative_message_id_is_recognised() -> None:
    """Klipper numbers an image's messages by count, and one byte holds -32 to 95: an older image
    that lacks a command can give an answer the id -1 (sent as 0x7f)."""

    class Answers:
        def __init__(self) -> None:
            self.out = b""

        def clear(self) -> None:
            pass

        def write(self, data: bytes) -> None:
            seq = fl.parse_klipper_blocks(bytearray(data))[0][0]
            nxt = (seq + 1) & 15
            body = fl.klipper_vlq(-1) + fl.klipper_vlq(1) + fl.klipper_vlq(0)
            self.out = fl.klipper_block(nxt, body) + fl.klipper_block(nxt, b"")

        def read(self) -> bytes:
            out, self.out = self.out, b""
            return out

    clock = FakeClock()
    assert fl.KlipperUnit(Answers(), clock).ask(fl.klipper_vlq(78), -1) == b"\x01\x00"


def test_a_truncated_answer_is_no_answer() -> None:
    class Short:
        def __init__(self) -> None:
            self.out = b""

        def clear(self) -> None:
            pass

        def write(self, data: bytes) -> None:
            seq = fl.parse_klipper_blocks(bytearray(data))[0][0]
            self.out = fl.klipper_block((seq + 1) & 15, fl.klipper_vlq(0))  # identify, no params

        def read(self) -> bytes:
            out, self.out = self.out, b""
            return out

    clock = FakeClock()
    assert fl.KlipperUnit(Short(), clock).dictionary(clock.now() + 5.0) is None


class BrokenLink:
    """A link whose port fails at every use."""

    def clear(self) -> None:
        pass

    def write(self, data: bytes) -> NoReturn:
        raise OSError(5, "Input/output error")

    def read(self) -> bytes:
        return b""

    def close(self) -> None:
        pass


def test_a_failure_of_the_link_is_a_link_error() -> None:
    # the flasher reports it as a link error; any other OSError keeps its own text
    clock = FakeClock()
    with pytest.raises(fl.LinkError, match=r"\[Errno 5\] Input/output error"):
        fl.Unit(BrokenLink(), clock).ask(fl.CMD_GET_INFO)
    with pytest.raises(fl.LinkError, match=r"\[Errno 5\] Input/output error"):
        fl.KlipperUnit(BrokenLink(), clock).ask(b"\x01", 0)


def moonraker_answering(replies: dict[str, object]) -> fl.Moonraker:
    """A Moonraker client whose server answers each path with its result in replies."""

    def opener(req: urllib.request.Request, timeout: float) -> io.BytesIO:
        path = req.full_url[len("http://m") :]
        return io.BytesIO(json.dumps({"result": replies.get(path)}).encode())

    return fl.Moonraker("http://m", opener)


INFO = "/server/info"
QUERY = "/printer/objects/query?print_stats&ace2k&configfile=settings"
STORE = "/server/gcode_store?count=20"


@pytest.mark.parametrize(
    "replies",
    [
        {INFO: ["ready"]},
        {INFO: {"klippy_state": 1}},
    ],
)
def test_a_state_of_an_unexpected_shape_is_no_answer(replies: dict[str, object]) -> None:
    mr = moonraker_answering(replies)
    assert mr.state() is None
    assert fl.klipper_view(mr).state is None


@pytest.mark.parametrize("result", [["status"], {"status": ["configfile"]}])
def test_objects_of_an_unexpected_shape_are_no_answer(result: object) -> None:
    mr = moonraker_answering({INFO: {"klippy_state": "error"}, QUERY: result})
    assert mr.objects() is None
    view = fl.klipper_view(mr)
    assert (view.state, view.printing, view.ace2k_configured) == ("error", False, False)


@pytest.mark.parametrize(
    ("status", "port"),
    [
        ({"configfile": ["settings"], "print_stats": ["state"]}, None),
        ({"configfile": {"settings": ["ace2k"]}}, None),
        (
            {"configfile": {"settings": {"ace2k": ["mcu"], "mcu ace2k": {"serial": "/dev/x"}}}},
            "/dev/x",
        ),
        ({"configfile": {"settings": {"ace2k": {}, "mcu ace2k": ["serial"]}}}, None),
        ({"configfile": {"settings": {"ace2k": {}, "mcu ace2k": {"serial": 7}}}}, None),
        ({"configfile": {"settings": {"ace2k": {}}}, "ace2k": ["version"]}, None),
    ],
)
def test_a_config_of_an_unexpected_shape_is_read_as_far_as_it_goes(
    status: dict[str, object], port: str | None
) -> None:
    mr = moonraker_answering({INFO: {"klippy_state": "ready"}, QUERY: {"status": status}})
    view = fl.klipper_view(mr)
    assert (view.printing, view.ace2k_port, view.ace2k_version) == (False, port, None)


def test_console_lines_of_an_unexpected_shape_are_skipped() -> None:
    assert moonraker_answering({STORE: ["lines"]}).console_lines() is None
    assert moonraker_answering({STORE: {"gcode_store": "lines"}}).console_lines() is None
    store = {
        "gcode_store": [
            {"message": "ok", "time": 10.5},
            {"message": "late", "time": "soon"},
            {"message": ["not text"], "time": 12.0},
            "not an entry",
        ]
    }
    assert moonraker_answering({STORE: store}).console_lines() == [(10.5, "ok")]


def test_a_unit_finds_its_mcu_whatever_the_case_of_the_name() -> None:
    # Klipper lists section names in lower case, but keeps the value of mcu: as written
    settings = {
        "ace2k a": {"mcu": "ACE_A"},
        "mcu ace_a": {"serial": "/dev/a"},
        "ace2k b": {"mcu": "ace_b"},
        "mcu ace_b": {"serial": "/dev/b"},
    }
    mr = moonraker_answering(
        {INFO: {"klippy_state": "ready"}, QUERY: {"status": {"configfile": {"settings": settings}}}}
    )
    assert fl.klipper_view(mr).ace2k_units == {"a": "/dev/a", "b": "/dev/b"}


@pytest.mark.parametrize("field", [b"", b"xy"])
@pytest.mark.parametrize("cmd", [fl.CMD_IAP_UPGRADE, fl.CMD_IAP_FIRMWARE])
def test_an_update_answer_without_a_numeric_code_is_a_failure(cmd: int, field: bytes) -> None:
    # field 1 is the answer's code, a number: bytes there are neither success nor a refusal
    class Answers:
        def ask(self, asked: int, payload: bytes = b"", timeout: float = 1.0) -> fl.Frame:
            return fl.Frame(0x80, 1, asked, fl.pb_bytes(1, field) if asked == cmd else b"")

    image = fl.image.Image(bytes(200), "ace2k", "0.12.0")
    with pytest.raises(fl.wire.UpdateError) as failed:
        fl.wire.update(Answers(), image, lambda line: None, FakeClock())
    what = "the announcement's answer" if cmd == fl.CMD_IAP_UPGRADE else "the answer to chunk 1/4"
    assert str(failed.value) == f"{what} carries no code: its field 1 is not a number"


def test_bootloader_enter_reads_the_answer_and_a_cut_answer_is_none(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    unit = SimUnit(clock, state="ace2k", ace2k_version="0.12.0")
    unit.baud = fl.ACE2K_BAUD
    k = fl.KlipperUnit(unit, clock)
    dictionary = k.dictionary(clock.now() + 30.0)
    unit.enter_accepted = False
    assert k.bootloader_enter(dictionary) is False and unit.state == "ace2k"
    monkeypatch.setattr(k, "call", lambda *_, **__: b"\x81")  # a VLQ cut after its first byte
    assert k.bootloader_enter(dictionary) is None


def test_has_call_needs_the_command_and_its_answer() -> None:
    has_call = fl.klipper.has_call
    full = {"commands": {"ask": 1}, "responses": {"answer x=%u": 2}}
    assert has_call(full, "ask", "answer x=%u")
    assert not has_call(full, "ask", "answer y=%u")
    assert not has_call({"responses": full["responses"]}, "ask", "answer x=%u")
    assert not has_call({}, "ask", "answer x=%u")
    no_id = {"commands": {"ask": None}, "responses": {"answer x=%u": 2}}
    assert not has_call(no_id, "ask", "answer x=%u")
    # call asks nothing for a missing id: the unit's link is never touched
    assert fl.klipper.KlipperUnit(None, None).call(no_id, "ask", "answer x=%u") is None


def test_configured_on_needs_the_config_and_the_same_device(tmp_path: Path) -> None:
    port = tmp_path / "ttyACM0"
    port.write_text("")
    link = tmp_path / "by-id"
    link.symlink_to(port)
    view = fl.host.KlipperView("ready", False, True, str(link), "0.12.0")
    assert view.configured_on(str(port))
    assert not view.configured_on(str(tmp_path / "ttyACM1"))
    assert not fl.host.KlipperView.no_unit("ready", False).configured_on(str(port))
