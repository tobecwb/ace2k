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
from typing import TYPE_CHECKING, Literal

FIRMWARE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FIRMWARE / "tools"))
import ace2k_flash as fl  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from ace2k_flash.wire import Json

UID = (0x11111111, 0x22222222, 0x33333333)
UID_TEXT = "111111112222222233333333"
PAIRS = [(600 + i, 1100 + i) if i in (0, 2, 4, 6) else (0, 0) for i in range(16)]
# A valid vector table for the unit: the stack top at the end of SRAM, a Thumb entry in the slot
VECTOR_TABLE = struct.pack("<II", 0x2000C000, 0x08008000 + 0x1001)


def calibration_payload(pairs: Sequence[tuple[int, int]]) -> bytes:
    out = b""
    for a, b in pairs:
        sub = (fl.pb_uint(1, a) if a else b"") + (fl.pb_uint(2, b) if b else b"")
        out += fl.pb_bytes(1, sub)
    return out


def realistic_dictionary() -> dict[str, object]:
    """Commands, responses and pin names, as a real dictionary holds them.  Text this varied
    makes zlib use a dynamic Huffman block, whose header alone takes more than its first 64
    compressed bytes, as in every real image."""
    names = ["config_digital_out", "set_digital_out", "queue_step", "set_next_step_dir"]
    names += ["reset_step_clock", "endstop_home", "trsync_start", "config_spi", "spi_transfer"]
    names += ["ace2k_lane_status", "ace2k_heat_set", "ace2k_fan_set", "get_uptime", "get_clock"]
    names += ["identify", "allocate_oids", "finalize_config", "get_config", "emergency_stop"]
    names += ["clear_shutdown", "config_buttons", "buttons_query", "ace2k_rfid_read"]
    replies = ["analog_in_state", "stepper_position", "endstop_state", "trsync_state"]
    replies += ["spi_transfer_response", "buttons_state", "ace2k_lane_event", "uptime", "clock"]
    replies += ["identify_response", "config", "shutdown", "is_shutdown", "stats"]
    return {
        "build_versions": "gcc: (Arm GNU Toolchain 13.3) binutils: 2.42",
        "commands": {f"{n} oid=%c pin=%u value=%c": 100 + i for i, n in enumerate(names)},
        "responses": {f"{n} oid=%c clock=%u value=%hu": 200 + i for i, n in enumerate(replies)},
        "enumerations": {
            "pin": {f"P{p}{i}": 16 * j + i for j, p in enumerate("ABCDE") for i in range(16)}
        },
    }


def ace2k_image(
    version: str = "0.12.0",
    with_constant: bool = True,
    size: int = 3000,
    extra: bytes = b"",
    window_ms: int | None = 180000,
    after: bytes = b"",
    realistic: bool = False,
) -> bytes:
    """An ace2k image.  extra goes before the compressed dictionary, after right after its last
    byte, where a real image keeps its version string (the bare `X.Y.Z\\0`).  realistic: the
    dictionary also holds what a real one does (realistic_dictionary)."""
    config = {"RESERVE_PINS_ace2k_led": "PE13,PE9,PC15,PE5"}
    if with_constant:
        config["ACE2K_VERSION"] = version
    if window_ms is not None:
        config["ACE2K_LINKPROOF_WINDOW_MS"] = window_ms
    dictionary: dict[str, object] = {"version": "v0.13.0", "config": config}
    if realistic:
        dictionary.update(realistic_dictionary())
    blob = zlib.compress(json.dumps(dictionary).encode())
    body = VECTOR_TABLE + bytes(range(256)) * (size // 256) + extra + blob + after
    body += b"\xff" * ((-len(body)) % 4)
    return body + struct.pack("<I", zlib.crc32(body)) + fl.TRAILER


def factory_image(size: int = 3000) -> bytes:
    body = VECTOR_TABLE + bytes((i * 7) & 0xFF for i in range(size))
    return body + b"\x00\x00\x00\x00" + fl.TRAILER


def reply(addr: int, seq: int, cmd: int, payload: bytes) -> bytes:
    body = bytes([addr]) + struct.pack("<H", seq) + bytes([cmd, len(payload)]) + payload
    return fl.PREAMBLE + body + struct.pack("<H", fl.crc16(body)) + bytes([fl.END])


class Console(list):
    """Moonraker's gcode store: the messages, each stamped with the time it was printed."""

    def __init__(self, clock: FakeClock) -> None:
        super().__init__()
        self.clock, self.times = clock, []

    def append(self, message: str) -> None:
        super().append(message)
        # strictly increasing, as Moonraker's own float times are in practice
        self.times.append(self.clock.now() + len(self.times) * 1e-6)


class FakeClock:
    def __init__(self) -> None:
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
        state: str = "factory",
        version: str = "V1.1.31",
        ace2k_version: str | None = None,
        extra_units: int = 0,
    ) -> None:
        self.clock, self.state, self.version = clock, state, version
        self.ace2k_version = ace2k_version
        # what the new firmware (ace2k or factory) reports once committed, if not the image's own
        self.reports_version = None
        # other units chained on the same port: each answers DISCOVER with its own id
        self.extra_units = extra_units
        # DISCOVER rounds whose replies collide on the bus: only this unit's reply arrives
        self.collisions = 0
        self.last_upload = "V0.0.0"
        self.swallow = 0  # frames to lose, as after Klipper's noise
        self.refuse_image = False
        # the code that answers the announcement (None: no answer), and the code that answers a
        # chunk by its number from 1 (None: no answer); a chunk not listed is taken
        self.announce_code: int | None = 0
        self.chunk_codes: dict[int, int | None] = {}
        self.commit_s = 8.0
        # the factory application answers this long after the finish: it checks the staged
        # image, writes the commit and only then resets into the bootloader
        self.linger_s = 2.0
        self.linger_until: float | None = None
        # the bootloader answers GET_INFO as recovery while it copies the image (it speaks the
        # protocol during a commit); False: it is silent then
        self.recovery_while_committing = False
        self.busy_until = None
        self.commit = None
        self.staged = bytearray()
        self.announced = None
        self.received = []  # every command id that reached the unit
        self.pending = b""
        self.closed = False
        self.opened = 0
        # ace2k over Klipper's protocol: whether it answers (False: a deaf image), the baud the
        # link was opened at, the MCU's next sequence, and whether its link was proven
        self.klipper_answers = True
        self.baud = fl.BAUD
        self.mcu_seq = 0
        # True: noise came in before (another protocol's bytes), and the MCU drops everything up
        # to the next sync byte, silently, as src/command.c does once it has sent its one nak
        self.needs_sync = False
        self.lose_blocks = 0  # blocks noise takes whole: dropped with no word back
        self.lose_at: set[int] = set()  # the same, for the blocks with these indexes (from 0)
        self.blocks = 0  # the blocks parsed so far
        # bytes a block broken by a collision still waits for: they complete it, its check fails,
        # and the MCU then drops all up to the next sync byte
        self.awaiting = 0
        self.proven = False
        self.proof_stores = True  # False: the store fails (or the window ran out): proven=0
        self.asked = []  # the Klipper message ids asked, in order
        self.blinks = 0  # FLASH_LED frames and ace2k_identify commands taken
        self.has_identify = True  # False: an ace2k image older than ace2k_identify
        self.has_enter = True  # False: an ace2k image older than ace2k_bootloader_enter
        self.led_payloads: list[bytes] = []
        self.answers_blink = True  # False: takes the blink command and gives no answer
        # ace2k_bootloader_enter: True accepts it (recovery 50 ms later), False is a veto (a lane
        # moves, the heater or the fans are busy), None takes it and gives no answer
        self.enter_accepted: bool | None = True

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
        if self.state == "ace2k" and self.busy_until is None:
            self._klipper(data)
            return
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
            self.busy_until = self.linger_until = None
            self.state, version = self.commit
            if self.state == "ace2k":
                self.ace2k_version = (
                    version if self.reports_version is None else self.reports_version
                )
            elif self.state == "factory":
                self.version = version if self.reports_version is None else self.reports_version

    def _dictionary(self) -> bytes:
        commands = {
            "identify offset=%u count=%c": 1,
            "ace2k_version": 77,
            "ace2k_linkproof_state": 78,
        }
        responses = {
            "identify_response offset=%u data=%.*s": 0,
            "ace2k_version_response version=%s": 300,
            "ace2k_linkproof_state_response proven=%c remaining_ms=%u": 301,
        }
        if self.has_enter:
            commands["ace2k_bootloader_enter"] = 80
            responses["ace2k_bootloader_enter_response accepted=%c"] = 303
        if self.has_identify:
            commands["ace2k_identify"] = 79
            responses["ace2k_identify_response ms=%u"] = 302
        doc = {"commands": commands, "responses": responses, "version": "v0.13.0"}
        if self.ace2k_version is not None:
            doc["config"] = {"ACE2K_VERSION": self.ace2k_version}
        return zlib.compress(json.dumps(doc).encode())

    def _synced(self, data: bytes) -> bytes:
        """The bytes the MCU parses: after noise, only those past the next sync byte."""
        if self.awaiting:
            taken = min(self.awaiting, len(data))
            self.awaiting -= taken
            data = data[taken:]
            self.needs_sync = self.needs_sync or not self.awaiting
        if not self.needs_sync:
            return data
        sync = data.find(bytes([fl.KLIPPER_SYNC]))
        if sync < 0:
            return b""
        self.needs_sync = False
        return data[sync + 1 :]

    def _klipper(self, data: bytes) -> None:
        """ace2k's side of Klipper's protocol, at its own baud: each block acknowledged with the
        sequence expected next (a block out of sequence refused so), identify and ace2k_version
        answered; ace2k_version proves the link."""
        if self.baud != fl.ACE2K_BAUD:  # another protocol's bytes: noise at the MCU's baud
            self.needs_sync = True
            return
        if not self.klipper_answers:
            return
        for seq, body in fl.parse_klipper_blocks(bytearray(self._synced(data))):
            index, self.blocks = self.blocks, self.blocks + 1
            if self.lose_blocks or index in self.lose_at:
                self.lose_blocks = max(0, self.lose_blocks - 1)
                continue
            if seq != self.mcu_seq:
                self.pending += fl.klipper_block(self.mcu_seq, b"")
                continue
            self.mcu_seq = (seq + 1) & fl.KLIPPER_SEQ_MASK
            msgid, pos = fl.parse_klipper_vlq(body, 0)
            self.asked.append(msgid)
            out = self._klipper_answer(msgid, body, pos)
            # as src/command.c: the answer first, then the acknowledgement
            if out is not None:
                self.pending += fl.klipper_block(self.mcu_seq, out)
            self.pending += fl.klipper_block(self.mcu_seq, b"")

    def _klipper_answer(self, msgid: int, body: bytes, pos: int) -> bytes | None:
        """The answer to one Klipper message, or None."""
        if msgid == 1:
            offset, pos = fl.parse_klipper_vlq(body, pos)
            count, pos = fl.parse_klipper_vlq(body, pos)
            chunk = self._dictionary()[offset : offset + count]
            out = fl.klipper_vlq(0) + fl.klipper_vlq(offset) + bytes([len(chunk)]) + chunk
        elif msgid == 77:
            self.proven = self.proof_stores
            text = (self.ace2k_version or "").encode()
            out = fl.klipper_vlq(300) + bytes([len(text)]) + text
        elif msgid == 78:
            out = fl.klipper_vlq(301) + fl.klipper_vlq(int(self.proven)) + fl.klipper_vlq(0)
        elif msgid == 79 and self.has_identify:
            self.blinks += 1
            out = None if not self.answers_blink else fl.klipper_vlq(302) + fl.klipper_vlq(10000)
        elif msgid == 80 and self.has_enter:
            out = None
            if self.enter_accepted is not None:
                out = fl.klipper_vlq(303) + fl.klipper_vlq(int(self.enter_accepted))
            if self.enter_accepted is not False:
                self.state = "recovery"  # 50 ms later the unit resets into recovery
        else:
            out = None
        return out

    def _app(self) -> bool:
        return self.state == "factory"

    def _lingering(self) -> bool:
        """The old factory application still runs after the finish."""
        return self.linger_until is not None and self.clock.now() < self.linger_until

    def _answer(self, f: fl.Frame) -> bytes | None:
        """The frame's answer, or None for silence."""
        busy = self.busy_until is not None and not self._lingering()
        if busy and self.recovery_while_committing and f.cmd == fl.CMD_GET_INFO:
            return reply(
                0x00,
                f.seq,
                f.cmd,
                fl.pb_bytes(1, self.last_upload.encode()) + fl.pb_bytes(2, b"V1.0.2"),
            )
        if busy or self.state in ("ace2k", "off"):
            return None
        handlers: dict[int, Callable[[int, fl.Frame], bytes | None]] = {
            fl.CMD_GET_INFO: self._get_info,
            fl.CMD_DISCOVER: self._discover,
            fl.CMD_GET_MOTOR_STATUS: self._motor_status,
            fl.CMD_FLASH_LED: self._flash_led,
            fl.CMD_IAP_UPGRADE: self._iap_upgrade,
            fl.CMD_IAP_FIRMWARE: self._iap_firmware,
            fl.CMD_IAP_FINISH: self._iap_finish,
        }
        handler = handlers.get(f.cmd)
        return None if handler is None else handler(0x80 if self._app() else 0x00, f)

    def _flash_led(self, addr: int, f: fl.Frame) -> bytes | None:
        if not self._app():
            return None  # recovery has no LEDs to drive
        self.blinks += 1
        self.led_payloads.append(f.payload)
        if not self.answers_blink:
            return None
        return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 1))

    def _get_info(self, addr: int, f: fl.Frame) -> bytes:
        version = self.version if self._app() else self.last_upload
        return reply(
            addr, f.seq, f.cmd, fl.pb_bytes(1, version.encode()) + fl.pb_bytes(2, b"V1.0.2")
        )

    def _discover(self, addr: int, f: fl.Frame) -> bytes:
        # the factory firmware and the bootloader's recovery both answer DISCOVER
        chained = self.extra_units
        if self.collisions:
            self.collisions -= 1
            chained = 0
        others = [(0x44440000 + i, 0x55550000 + i, 0x66660000 + i) for i in range(chained)]
        return b"".join(
            reply(addr, f.seq, f.cmd, b"".join(fl.pb_uint(n + 1, u) for n, u in enumerate(uid)))
            for uid in [UID, *others]
        )

    def _motor_status(self, addr: int, f: fl.Frame) -> bytes | None:
        if not self._app():
            return None
        return reply(addr, f.seq, f.cmd, calibration_payload(PAIRS))

    def _iap_upgrade(self, addr: int, f: fl.Frame) -> bytes | None:
        fields = fl.pb_fields(f.payload)
        self.announced = (fields[1][0], fields[2][0], fields[3][0].decode())
        self.staged = bytearray()
        if self.announce_code is None:
            return None
        return reply(addr, f.seq, f.cmd, fl.pb_uint(1, self.announce_code))

    def _iap_firmware(self, addr: int, f: fl.Frame) -> bytes | None:
        fields = fl.pb_fields(f.payload)
        number = len(self.staged) // fl.wire.CHUNK_BYTES + 1
        if number in self.chunk_codes:
            code = self.chunk_codes[number]
            return None if code is None else reply(addr, f.seq, f.cmd, fl.pb_uint(1, code))
        if fields[1][0] != fl.STAGING_BASE + len(self.staged):
            return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 3))
        self.staged += fields[2][0]
        return reply(addr, f.seq, f.cmd, fl.pb_uint(1, 0))

    def _iap_finish(self, addr: int, f: fl.Frame) -> bytes | None:
        self._schedule_commit(linger=self._app())
        return None if not self._app() else reply(addr, f.seq, f.cmd, fl.pb_uint(1, 0))

    def _schedule_commit(self, linger: bool = False) -> None:
        """Commit the staged image after commit_s.  linger: the factory application took the
        update and answers for linger_s more before it resets."""
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
        start = self.clock.now() + (self.linger_s if linger else 0.0)
        self.linger_until = start if linger else None
        self.busy_until = start + self.commit_s


class SimHost:
    """Klipper (service, state, the port's holder) and Moonraker."""

    def __init__(
        self,
        tmp: Path,
        clock: FakeClock,
        unit: SimUnit,
        configured: bool = True,
        systemd: bool = False,
        holds_when_not_ready: bool = True,
    ) -> None:
        self.tmp, self.clock, self.unit = tmp, clock, unit
        # Klipper not ready closes the port after a failed identify (real behaviour)
        self.holds_when_not_ready = holds_when_not_ready
        self.state_override = None  # force Klipper's reported state, e.g. "startup"
        self.keep_holder = False  # the process keeps the port after the stop command
        self.start_rc = 0
        self.stop_rc = 0
        # "stop" / "start" / "klipper.service" (systemctl cat): an exception the command raises
        self.raise_on: dict[str, BaseException] = {}
        self.run_timeouts: list[float | None] = []  # the timeout each command ran with
        self.restore_lag = 0.0  # seconds Klipper takes to notice the reset
        self.restore_silent = False  # ACEK_RESTORE_STOCK does nothing and prints nothing
        # the host module's command names: "new" (ACEK_*), "old" (ACE_*, up to v0.12.0-rc.1) or
        # "none" (neither), served by the command help; a name it does not register is
        # Klipper's unknown command: an info line on the console and a script that succeeds
        # (klippy/gcode.py)
        self.host_module = "new"
        self.help_unanswered = False  # Moonraker does not answer /printer/gcode/help
        self.console_unanswered = False  # nor /server/gcode_store
        self.help_reads = 0
        self.config_port = None  # the port [mcu ace2k] names, when not the unit's
        # several units: this unit is [ace2k <unit_name>] on [mcu ace2k_<unit_name>], beside a
        # second unit [ace2k other] on another port; None: the one [ace2k] section
        self.unit_name = None
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
        # how a refusal reaches the tool: "error" — a G-code error, which Moonraker answers as an
        # HTTP error and Klipper also prints as a `!!` line; "console" — the `!!` line alone (a
        # host on the older ace2k); "http" — the HTTP error alone (the line missed)
        self.refuse_form = "error"
        self.refusal_text = "ace2k: refused — a subsystem is busy (heating or moving)"
        # Moonraker answers the script with an error that is not the refusal (a proxy's 503,
        # Klipper disconnected, the unit's lost answer); with _console, Klipper printed it too;
        # with _state, Klipper is in that state from the POST on (it left ready just before)
        self.script_error = None
        self.script_error_console = False
        self.script_error_state = None
        self.script_unanswered = False  # the POST gets no answer; the script still runs
        self.console = Console(clock)
        self.console_reads = 0
        self.commands = []  # service commands run
        self.starts = 0  # start commands run
        self.gcodes = []
        self.answers = ["y"]
        self.lines = []
        self.on_line = None
        self.opened = 0
        self._t = datetime.datetime(2026, 10, 8, 12, 0, 0)
        self._sync_proc()

    # Klipper's view of itself
    def klippy_state(self) -> str:
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
    def state(self) -> str | None:
        return self.klippy_state() if self.moonraker_up else None

    def objects(self) -> Json:
        if not (self.moonraker_up and self.running):
            return {}
        settings = {}
        section = "ace2k" if self.unit_name is None else f"ace2k {self.unit_name}"
        if self.configured and self.unit_name is None:
            settings = {"mcu ace2k": {"serial": self.config_port or self.port}, "ace2k": {}}
        elif self.configured:
            settings = {
                f"mcu ace2k_{self.unit_name}": {"serial": self.config_port or self.port},
                section: {"mcu": f"ace2k_{self.unit_name}"},
                "mcu ace2k_other": {"serial": "/dev/serial/by-id/usb-1a86_OTHER-if00"},
                "ace2k other": {"mcu": "ace2k_other"},
            }
        status = {
            "configfile": {"settings": settings},
            "print_stats": {"state": "printing" if self.printing else "standby"},
        }
        if self.configured and self.klippy_state() == "ready":
            status[section] = {"version": self.unit.ace2k_version}
            if self.unit_name is not None:
                status["ace2k other"] = {"version": "0.0.1-other"}
        return status

    def gcode(self, script: str) -> Literal[True] | fl.MoonrakerError | None:
        """True: the script ran; fl.MoonrakerError: Moonraker's error answer; None: the POST got
        no answer (the script may still have run)."""
        self.gcodes.append(script)
        answer = self._run_script(script)
        return None if self.script_unanswered else answer

    def _run_script(  # noqa: PLR0911 — one answer per way the script ends
        self, script: str
    ) -> Literal[True] | fl.MoonrakerError:
        known = self._restore_name()
        if self.unit_name is not None:  # several units: the restore must name this one
            known = f"{known} UNIT={self.unit_name}"
        if script in ("ACEK_RESTORE_STOCK", "ACE_RESTORE_STOCK") and script != known:
            self.console.append(f'// Unknown command:"{script}"')
            return True
        if script == known and self.unit.state == "ace2k":
            if self.script_error is not None:
                if self.script_error_state is not None:
                    self.state_override = self.script_error_state
                # a G-code error that is not the refusal; Klipper prints its first line as `!!`
                if self.script_error_console:
                    self.console.append("!! " + self.script_error.split("\n")[0].strip())
                return fl.MoonrakerError(self.script_error)
            if self.refuse_restore:
                if self.refuse_form in ("error", "console"):
                    self.console.append("!! " + self.refusal_text.split("\n")[0].strip())
                if self.refuse_form in ("error", "http"):
                    return fl.MoonrakerError(self.refusal_text)
                return True  # an older ace2k: the script ran, the line was the refusal
            if self.restore_silent:
                return True
            if not self.restore_lag:
                self._reset_unit()
                return True
            at, fired = self.clock.now() + self.restore_lag, []

            def hook() -> None:
                if not fired and self.clock.now() >= at:
                    fired.append(True)
                    self._reset_unit()

            self.clock.hooks.append(hook)
        return True

    def _restore_name(self) -> str | None:
        return {"new": "ACEK_RESTORE_STOCK", "old": "ACE_RESTORE_STOCK"}.get(self.host_module)

    def gcode_commands(self) -> set[str] | None:
        """Moonraker's /printer/gcode/help, as the tool reads it: the names, or None."""
        self.help_reads += 1
        if self.help_unanswered or not self.moonraker_up or not self.running:
            return None
        names = {"G28", "SAVE_CONFIG"}
        if self.configured and self._restore_name():
            names.add(self._restore_name())
        return names

    def _reset_unit(self) -> None:
        if self.unit.state == "ace2k":
            self.unit.state = "recovery"
            self.unit.swallow = 1  # the noise Klipper keeps writing until it is stopped

    def console_lines(self) -> list[tuple[float, str]] | None:
        self.console_reads += 1
        if not self.moonraker_up or self.console_unanswered:
            return None
        return list(zip(self.console.times, self.console))

    # the host's commands
    def run(
        self, cmd: Sequence[str], capture_output: bool = False, timeout: float | None = None
    ) -> SimpleNamespace:
        self.commands.append(list(cmd))
        self.run_timeouts.append(timeout)
        if cmd[-1] in self.raise_on:
            raise self.raise_on[cmd[-1]]
        if cmd[:2] == ["systemctl", "cat"]:
            return SimpleNamespace(returncode=0 if self.systemd else 1)
        if cmd[-1] == "stop" and self.stop_rc:
            return SimpleNamespace(returncode=self.stop_rc)
        if cmd[-1] == "stop":
            self.running = False
        elif cmd[-1] == "start":
            self.starts += 1
            if self.start_rc:
                return SimpleNamespace(returncode=self.start_rc)
            self.running = True
            self.state_override = None  # a fresh Klipper settles on the unit's real state
        self._sync_proc()
        return SimpleNamespace(returncode=0)

    def stop_klipper_now(self) -> None:
        """Klipper stopped by someone else (an installer): nothing holds the port."""
        self.running = False
        self._sync_proc()

    def which(self, name: str) -> str | None:
        return "/usr/bin/systemctl" if (name == "systemctl" and self.systemd) else None

    def ask(self, prompt: str) -> str:
        return self.answers.pop(0)

    def out(self, line: str) -> None:
        self.lines.append(line)
        if self.on_line:
            self.on_line(self, line)

    def now(self) -> datetime.datetime:
        self._t += datetime.timedelta(seconds=1)
        return self._t

    def open_link(self, port: str, baud: int = fl.BAUD) -> SimUnit:
        assert port == self.port
        self.opened += 1
        self.unit.closed = False
        self.unit.baud = baud
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


def args(image: Path | None, tmp: Path, **kw: object) -> SimpleNamespace:
    base = dict(
        image=None if image is None else str(image),
        port=None,
        version=None,
        moonraker="http://sim",
        klipper_stop=None,
        klipper_start=None,
        calibration_dir=str(tmp / "cal"),
        yes=False,
        config_follows=False,
        identify=False,
    )
    base.update(kw)
    return SimpleNamespace(**base)


def setup(
    tmp: Path,
    state: str = "ace2k",
    configured: bool = True,
    image: bytes | None = None,
    **unit_kw: object,
) -> tuple[FakeClock, SimUnit, SimHost, Path]:
    clock = FakeClock()
    host_kw = {k: unit_kw.pop(k) for k in ("holds_when_not_ready",) if k in unit_kw}
    unit = SimUnit(clock, state=state, **unit_kw)
    if state == "ace2k" and unit.ace2k_version is None:
        unit.ace2k_version = "0.11.0"
    host = SimHost(tmp, clock, unit, configured=configured, **host_kw)
    path = tmp / "image.bin"
    path.write_bytes(image if image is not None else ace2k_image())
    return clock, unit, host, path
