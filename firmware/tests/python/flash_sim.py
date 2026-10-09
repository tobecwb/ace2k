"""A simulated unit, Klipper host and Moonraker for ace2k_flash's tests.

SimUnit speaks the original protocol as the factory firmware and the bootloader's recovery do,
goes silent as ace2k does, and commits a staged image after the finish.  SimHost is Klipper (its
service, its state, the process holding the port under a fake /proc) and Moonraker in one.
Everything runs on a virtual clock: nothing sleeps.
"""

from __future__ import annotations

import datetime
import json
import struct
import sys
import zlib
from pathlib import Path
from types import SimpleNamespace

FIRMWARE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FIRMWARE / "tools"))
import ace2k_flash as fl  # noqa: E402

UID = (0x11111111, 0x22222222, 0x33333333)
UID_TEXT = "111111112222222233333333"
PAIRS = [(600 + i, 1100 + i) if i in (0, 2, 4, 6) else (0, 0) for i in range(16)]
# A valid vector table for the unit: the stack top at the end of SRAM, a Thumb entry in the slot
VECTOR_TABLE = struct.pack("<II", 0x2000C000, 0x08008000 + 0x1001)


def calibration_payload(pairs) -> bytes:
    out = b""
    for a, b in pairs:
        sub = (fl.pb_uint(1, a) if a else b"") + (fl.pb_uint(2, b) if b else b"")
        out += fl.pb_bytes(1, sub)
    return out


def ace2k_image(version="0.12.0", with_constant=True, size=3000, extra=b"") -> bytes:
    config = {"RESERVE_PINS_ace2k_led": "PE13,PE9,PC15,PE5"}
    if with_constant:
        config["ACE2K_VERSION"] = version
    blob = zlib.compress(json.dumps({"version": "v0.13.0", "config": config}).encode())
    body = VECTOR_TABLE + bytes(range(256)) * (size // 256) + extra + blob
    body += b"\xff" * ((-len(body)) % 4)
    return body + struct.pack("<I", zlib.crc32(body)) + fl.TRAILER


def factory_image(size=3000) -> bytes:
    body = VECTOR_TABLE + bytes((i * 7) & 0xFF for i in range(size))
    return body + b"\x00\x00\x00\x00" + fl.TRAILER


def reply(addr: int, seq: int, cmd: int, payload: bytes) -> bytes:
    body = bytes([addr]) + struct.pack("<H", seq) + bytes([cmd, len(payload)]) + payload
    return fl.PREAMBLE + body + struct.pack("<H", fl.crc16(body)) + bytes([fl.END])


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.hooks = []

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt
        for hook in self.hooks:
            hook()


class SimUnit:
    """state: factory | recovery | ace2k | off.  Implements the link interface too."""

    def __init__(
        self,
        clock: FakeClock,
        state="factory",
        version="V1.1.31",
        ace2k_version=None,
        extra_units: int = 0,
    ):
        self.clock, self.state, self.version = clock, state, version
        self.ace2k_version = ace2k_version
        self.reports_version = None  # what ace2k reports once committed, if not the image's own
        # other units chained on the same port: each answers DISCOVER with its own id
        self.extra_units = extra_units
        self.last_upload = "V0.0.0"
        self.swallow = 0  # frames to lose, as after Klipper's noise
        self.refuse_image = False
        self.commit_s = 8.0
        self.busy_until = None
        self.commit = None
        self.staged = bytearray()
        self.announced = None
        self.received = []  # every command id that reached the unit
        self.pending = b""
        self.closed = False
        self.opened = 0

    # link interface
    def clear(self) -> None:
        self._tick()
        self.pending = b""

    def read(self) -> bytes:
        self._tick()
        data, self.pending = self.pending, b""
        return data

    def close(self) -> None:
        self.closed = True

    def write(self, data: bytes) -> None:
        self._tick()
        for f in fl.parse_frames(bytearray(data)):
            self.received.append(f.cmd)
            if self.swallow:
                self.swallow -= 1
                continue
            out = self._answer(f)
            if out is not None:
                self.pending += out

    # behaviour
    def _tick(self) -> None:
        if self.busy_until is not None and self.clock.now() >= self.busy_until:
            self.busy_until = None
            self.state, version = self.commit
            if self.state == "ace2k":
                self.ace2k_version = self.reports_version or version
            elif self.state == "factory":
                self.version = version

    def _app(self) -> bool:
        return self.state == "factory"

    def _answer(self, f):
        if self.busy_until is not None or self.state in ("ace2k", "off"):
            return None
        addr = 0x80 if self._app() else 0x00
        if f.cmd == fl.CMD_GET_INFO:
            version = self.version if self._app() else self.last_upload
            return reply(
                addr, f.seq, f.cmd, fl.pb_bytes(1, version.encode()) + fl.pb_bytes(2, b"V1.0.2")
            )
        if f.cmd == fl.CMD_DISCOVER:
            # the factory firmware and the bootloader's recovery both answer DISCOVER
            others = [
                (0x44440000 + i, 0x55550000 + i, 0x66660000 + i) for i in range(self.extra_units)
            ]
            return b"".join(
                reply(addr, f.seq, f.cmd, b"".join(fl.pb_uint(n + 1, u) for n, u in enumerate(uid)))
                for uid in [UID, *others]
            )
        if f.cmd == fl.CMD_GET_MOTOR_STATUS and self._app():
            return reply(addr, f.seq, f.cmd, calibration_payload(PAIRS))
        if f.cmd == fl.CMD_IAP_UPGRADE:
            fields = fl.pb_fields(f.payload)
            self.announced = (fields[1][0], fields[2][0], fields[3][0].decode())
            self.staged = bytearray()
            return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 0))
        if f.cmd == fl.CMD_IAP_FIRMWARE:
            fields = fl.pb_fields(f.payload)
            if fields[1][0] != fl.STAGING_BASE + len(self.staged):
                return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 3))
            self.staged += fields[2][0]
            return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 0))
        if f.cmd == fl.CMD_IAP_FINISH:
            self._schedule_commit()
            return None if not self._app() else reply(addr, f.seq, f.cmd, fl.pb_uint(1, 0))
        return None

    def _schedule_commit(self) -> None:
        data = bytes(self.staged)
        size, crc, announce = self.announced
        self.last_upload = announce
        ok = len(data) == size and fl.crc16(data) == crc and not self.refuse_image
        if not ok:
            self.commit = ("recovery", None)
        elif struct.unpack_from("<I", data, len(data) - 12)[0] == zlib.crc32(data[:-12]):
            cfg = fl.embedded_dictionary(data[:-12])["config"]
            self.commit = ("ace2k", cfg.get("ACE2K_VERSION", announce))
        else:
            self.commit = ("factory", "V" + announce)
        self.busy_until = self.clock.now() + self.commit_s


class SimHost:
    """Klipper (service, state, the port's holder) and Moonraker."""

    def __init__(
        self,
        tmp: Path,
        clock: FakeClock,
        unit: SimUnit,
        configured=True,
        systemd=False,
        holds_when_not_ready=True,
    ):
        self.tmp, self.clock, self.unit = tmp, clock, unit
        # Klipper not ready closes the port after a failed identify (real behaviour)
        self.holds_when_not_ready = holds_when_not_ready
        self.state_override = None  # force Klipper's reported state, e.g. "startup"
        self.keep_holder = False  # the process keeps the port after the stop command
        self.start_rc = 0
        self.stop_rc = 0
        self.raise_on = {}  # "stop" / "start": an exception the command raises when run
        self.restore_lag = 0.0  # seconds Klipper takes to notice the reset
        self.restore_silent = False  # ACE_RESTORE_STOCK does nothing and prints nothing
        self.config_port = None  # the port [mcu ace2k] names, when not the unit's
        self.by_id = tmp / "by-id"
        self.by_id.mkdir()
        self.port = str(self.by_id / "usb-1a86_USB_Single_Serial_TEST-if00")
        Path(self.port).write_text("")
        self.proc = tmp / "proc"
        self.proc.mkdir()
        self.initd = tmp / "S60klipper"
        self.initd.write_text("")
        self.systemd = systemd
        self.configured = configured
        self.running = True
        self.moonraker_up = True
        self.printing = False
        self.refuse_restore = False
        self.console = []
        self.commands = []  # service commands run
        self.gcodes = []
        self.answers = ["y"]
        self.lines = []
        self.on_line = None
        self.opened = 0
        self._t = datetime.datetime(2026, 10, 8, 12, 0, 0)
        self._sync_proc()

    # Klipper's view of itself
    def klippy_state(self):
        if not self.running:
            return "disconnected"
        if not self.configured:
            return "ready"
        if self.state_override:
            return self.state_override
        return "ready" if self.unit.state == "ace2k" else "error"

    def _holds_port(self) -> bool:
        if not (self.running and self.configured):
            return False
        return self.holds_when_not_ready or self.klippy_state() == "ready"

    def _sync_proc(self) -> None:
        d = self.proc / "4242"
        if self._holds_port() and not d.exists():
            (d / "fd").mkdir(parents=True)
            (d / "cmdline").write_bytes(b"/usr/bin/python3\0/home/lava/klipper/klippy/klippy.py\0")
            (d / "fd" / "7").symlink_to(self.port)
        elif not self._holds_port() and d.exists() and not self.keep_holder:
            (d / "fd" / "7").unlink()
            (d / "fd").rmdir()
            (d / "cmdline").unlink()
            d.rmdir()

    # Moonraker
    def state(self):
        return self.klippy_state() if self.moonraker_up else None

    def objects(self) -> dict:
        if not (self.moonraker_up and self.running):
            return {}
        settings = {}
        if self.configured:
            settings = {"mcu ace2k": {"serial": self.config_port or self.port}, "ace2k": {}}
        status = {
            "configfile": {"settings": settings},
            "print_stats": {"state": "printing" if self.printing else "standby"},
        }
        if self.configured and self.klippy_state() == "ready":
            status["ace2k"] = {"version": self.unit.ace2k_version}
        return status

    def gcode(self, script: str) -> None:
        self.gcodes.append(script)
        if script == "ACE_RESTORE_STOCK" and self.unit.state == "ace2k":
            if self.refuse_restore:
                self.console.append("!! ace2k: refused — a subsystem is busy (heating or moving)")
                return
            if self.restore_silent:
                return
            if not self.restore_lag:
                self._reset_unit()
                return
            at, fired = self.clock.now() + self.restore_lag, []

            def hook() -> None:
                if not fired and self.clock.now() >= at:
                    fired.append(True)
                    self._reset_unit()

            self.clock.hooks.append(hook)

    def _reset_unit(self) -> None:
        if self.unit.state == "ace2k":
            self.unit.state = "recovery"
            self.unit.swallow = 1  # the noise Klipper keeps writing until it is stopped

    def console_errors(self) -> list:
        return [m for m in self.console if m.startswith("!!")]

    # the host's commands
    def run(self, cmd, capture_output=False):
        self.commands.append(list(cmd))
        if cmd[:2] == ["systemctl", "cat"]:
            return SimpleNamespace(returncode=0 if self.systemd else 1)
        if cmd[-1] in self.raise_on:
            raise self.raise_on[cmd[-1]]
        if cmd[-1] == "stop" and self.stop_rc:
            return SimpleNamespace(returncode=self.stop_rc)
        if cmd[-1] == "stop":
            self.running = False
        elif cmd[-1] == "start":
            if self.start_rc:
                return SimpleNamespace(returncode=self.start_rc)
            self.running = True
            self.state_override = None  # a fresh Klipper settles on the unit's real state
        self._sync_proc()
        return SimpleNamespace(returncode=0)

    def which(self, name):
        return "/usr/bin/systemctl" if (name == "systemctl" and self.systemd) else None

    def ask(self, prompt: str) -> str:
        return self.answers.pop(0)

    def out(self, line: str) -> None:
        self.lines.append(line)
        if self.on_line:
            self.on_line(self, line)

    def now(self):
        self._t += datetime.timedelta(seconds=1)
        return self._t

    def open_link(self, port: str):
        assert port == self.port
        self.opened += 1
        self.unit.closed = False
        return self.unit

    def deps(self) -> fl.Deps:
        return fl.Deps(
            open_link=self.open_link,
            moonraker=self,
            clock=self.clock,
            run=self.run,
            which=self.which,
            proc=self.proc,
            by_id=self.by_id,
            initd=self.initd,
            ask=self.ask,
            out=self.out,
            now=self.now,
        )

    def text(self) -> str:
        return "\n".join(self.lines)


def args(image: Path, tmp: Path, **kw) -> SimpleNamespace:
    base = dict(
        image=str(image),
        port=None,
        version=None,
        moonraker="http://sim",
        klipper_stop=None,
        klipper_start=None,
        calibration_dir=str(tmp / "cal"),
        yes=False,
    )
    base.update(kw)
    return SimpleNamespace(**base)


def setup(tmp: Path, state="ace2k", configured=True, image=None, **unit_kw):
    clock = FakeClock()
    host_kw = {k: unit_kw.pop(k) for k in ("holds_when_not_ready",) if k in unit_kw}
    unit = SimUnit(clock, state=state, **unit_kw)
    if state == "ace2k" and unit.ace2k_version is None:
        unit.ace2k_version = "0.11.0"
    host = SimHost(tmp, clock, unit, configured=configured, **host_kw)
    path = tmp / "image.bin"
    path.write_bytes(image if image is not None else ace2k_image())
    return clock, unit, host, path
