#!/usr/bin/env python3
"""ace2k_flash: flash an ACE 2 Pro with an ace2k image or the factory image.

    python3 ace2k_flash.py IMAGE.bin [--port PORT] [--version V] [--yes]

Runs on the Klipper host (Linux, Python 3.9+, pyserial — Klipper's own dependency).  The unit is
identified before anything changes — the factory firmware, ace2k under Klipper, or the
bootloader's recovery — and the result is checked positively at the end.
The operator's page: docs/flashing.md.

The update sequence over the original Anycubic protocol (announce, 64-byte chunks, finish) was
worked out and first published by hakimio
(https://gist.github.com/hakimio/39c71fa7174e699c6470b7c79323b189).  This is an independent
implementation of it.

Exit status: 0 flashed and verified, 1 refused or failed, 2 flashed but not verifiable here.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import shlex
import shutil
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

# --- The original protocol (docs/protocol.md) -----------------------------------------------
BAUD = 230400
PREAMBLE = b"\xff\xaa"
END = 0xFE
MAX_PAYLOAD = 100  # the cap every tool of the original protocol keeps
RESPONSE_BIT = 0x80  # set in the address byte of the application's replies, not the bootloader's

CMD_DISCOVER = 0
CMD_IAP_UPGRADE = 2
CMD_IAP_FIRMWARE = 3
CMD_IAP_FINISH = 4
CMD_IAP_VERSION = 5
CMD_GET_INFO = 7
CMD_GET_MOTOR_STATUS = 78
# The only commands this tool sends.  15, 17 and 18 write the unit's calibration
# and configuration: never.
ALLOWED_COMMANDS = frozenset(
    {
        CMD_DISCOVER,
        CMD_IAP_UPGRADE,
        CMD_IAP_FIRMWARE,
        CMD_IAP_FINISH,
        CMD_IAP_VERSION,
        CMD_GET_INFO,
        CMD_GET_MOTOR_STATUS,
    }
)

# --- Images (firmware/tools/mkimage.py, docs/hardware.md "Memory map") ------------------------
TRAILER = bytes.fromhex("61a5635a65a5325a")
STAGING_BASE = 0x08024000
# The unit's memory, as firmware/tools/mkimage.py checks an image against it
SRAM_BASE, SRAM_END = 0x20000000, 0x2000C000
APP_BASE, APP_END = 0x08008000, 0x08024000
APP_SLOT_BYTES = APP_END - APP_BASE  # 112 KB
CHUNK_BYTES = 64
ANNOUNCE_MAX = 11  # a longer version string gets no answer to the announcement (flashing.md)

# --- Timing: initial values ---------------------------------------
IDENTIFY_TRIES = 5
IDENTIFY_TIMEOUT_S = 1.0
ANNOUNCE_TIMEOUT_S = 5.0
CHUNK_TIMEOUT_S = 2.0
CHUNK_PAUSE_S = 0.015
FINISH_TIMEOUT_S = 2.0
VERDICT_WAIT_S = 15.0  # recovery copies the image, then boots it
VERDICT_TRIES = 5
RESTORE_WAIT_S = 15.0  # ACE_RESTORE_STOCK: Klipper leaves `ready` ~4-6 s after the unit resets
REFUSAL_SETTLE_S = 6.0  # a `!!` line while Klipper is ready: the reset may still follow (4-6 s)
STOP_WAIT_S = 15.0  # Klipper stopped: its process lets go of the port
ACE2K_WAIT_S = 90.0  # Klipper started: ace2k reports its version
FACTORY_WAIT_S = 60.0
POLL_S = 0.005
STATE_POLL_S = 0.5

EXIT_OK, EXIT_FAILED, EXIT_UNVERIFIED = 0, 1, 2


class RefusedError(Exception):
    """A stop the operator must read; printed as is."""


class UpdateError(Exception):
    """The update itself went wrong; the unit accepts a new one."""


def crc16(data: bytes) -> int:
    """CRC-16 of the original protocol: init 0xFFFF, reflected 0x8408, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def build_frame(cmd: int, payload: bytes, seq: int) -> bytes:
    if cmd not in ALLOWED_COMMANDS:
        raise RefusedError(f"command {cmd} is not one this tool sends")
    if len(payload) > MAX_PAYLOAD:
        raise RefusedError(f"payload of {len(payload)} bytes is above the {MAX_PAYLOAD}-byte cap")
    body = bytes([0x00]) + struct.pack("<H", seq & 0xFFFF) + bytes([cmd, len(payload)]) + payload
    return PREAMBLE + body + struct.pack("<H", crc16(body)) + bytes([END])


@dataclass
class Frame:
    addr: int
    seq: int
    cmd: int
    payload: bytes


def parse_frames(buf: bytearray) -> list:
    """Every valid frame in buf, in order; consumes up to the end of the last one.

    A false preamble (noise that happens to read FF AA) is skipped rather than waited on, so a
    real frame after it is still found.
    """
    frames, pos, consumed = [], 0, 0
    while True:
        i = buf.find(PREAMBLE, pos)
        if i < 0 or len(buf) - i < 10:
            break
        n = buf[i + 6]
        total = n + 10
        if len(buf) - i >= total:
            f = bytes(buf[i : i + total])
            body = f[2 : 7 + n]
            if f[-1] == END and struct.unpack_from("<H", f, 7 + n)[0] == crc16(body):
                frames.append(Frame(f[2], f[3] | (f[4] << 8), f[5], f[7 : 7 + n]))
                pos = consumed = i + total
                continue
        pos = i + 1
    del buf[:consumed]
    return frames


def varint(value: int) -> bytes:
    out = bytearray()
    while value > 0x7F:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def pb_uint(field: int, value: int) -> bytes:
    return varint(field << 3) + varint(value)


def pb_bytes(field: int, data: bytes) -> bytes:
    return varint((field << 3) | 2) + varint(len(data)) + data


def _read_varint(data: bytes, pos: int) -> tuple:
    value, shift = 0, 0
    while True:
        if pos >= len(data):
            raise ValueError("truncated varint")
        byte = data[pos]
        pos += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, pos
        shift += 7


def pb_fields(data: bytes) -> dict:
    """field number → list of values: int for varints, bytes for length-delimited fields."""
    fields, pos = {}, 0
    while pos < len(data):
        tag, pos = _read_varint(data, pos)
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, pos = _read_varint(data, pos)
        elif wire == 2:
            size, pos = _read_varint(data, pos)
            if pos + size > len(data):
                raise ValueError("truncated field")
            value, pos = data[pos : pos + size], pos + size
        elif wire in (1, 5):
            width = 8 if wire == 1 else 4
            if pos + width > len(data):
                raise ValueError("truncated field")
            value, pos = data[pos : pos + width], pos + width
        else:
            raise ValueError(f"wire type {wire}")
        fields.setdefault(number, []).append(value)
    return fields


def _first(fields: dict, number: int, default):
    return fields.get(number, [default])[0]


def _text(fields: dict, number: int) -> str:
    value = _first(fields, number, b"")
    return value.decode(errors="replace") if isinstance(value, bytes) else ""


# --- Images ----------------------------------------------------------------------------------
@dataclass
class Image:
    data: bytes
    kind: str  # "ace2k" or "factory"
    version: str
    embedded: bool = False  # the version came from the image, not from --version

    @property
    def announce(self) -> str:
        return self.version[:ANNOUNCE_MAX]

    @property
    def crc(self) -> int:
        return crc16(self.data)


def embedded_dictionary(body: bytes):
    """The Klipper data dictionary compiled into an ace2k image, or None."""
    for m in re.finditer(rb"\x78[\x01\x5e\x9c\xda]", body):
        try:
            raw = zlib.decompressobj().decompress(body[m.start() :])
        except zlib.error:
            continue
        if raw.startswith(b"{"):
            try:
                return json.loads(raw)
            except ValueError:
                continue
    return None


def vector_table_ok(data: bytes) -> bool:
    """The initial stack pointer in SRAM and the reset vector, a Thumb address in the slot."""
    sp, entry = struct.unpack_from("<II", data, 0)
    return SRAM_BASE < sp <= SRAM_END and APP_BASE <= entry < APP_END and entry & 1 == 1


def load_image(path: Path, version) -> Image:
    try:
        data = path.read_bytes()
    except OSError as e:
        raise RefusedError(f"{path}: {e.strerror}") from None
    if len(data) < 16 or data[-8:] != TRAILER:
        raise RefusedError(f"{path}: not an image for the unit (no bootloader trailer at its end)")
    if len(data) > APP_SLOT_BYTES:
        raise RefusedError(f"{path}: {len(data)} bytes, larger than the {APP_SLOT_BYTES}-byte slot")
    if not vector_table_ok(data):
        raise RefusedError(
            f"{path}: not an application for the unit "
            "(its vector table points outside the unit's memory)"
        )
    body = data[:-12]
    if struct.unpack_from("<I", data, len(data) - 12)[0] == zlib.crc32(body):
        dictionary = embedded_dictionary(body)
        if dictionary is None:
            raise RefusedError(f"{path}: has ace2k's checksum but no Klipper dictionary inside")
        embedded = dictionary.get("config", {}).get("ACE2K_VERSION")
        if embedded is None and not version:
            raise RefusedError(
                f"{path}: an ace2k image from before v0.12 carries no version; pass --version"
            )
        if embedded is not None and version and version != embedded:
            raise RefusedError(f"{path}: --version {version} but the image says {embedded}")
        return Image(data, "ace2k", embedded or version, embedded is not None)
    if not version:
        raise RefusedError(f"{path}: a factory image; pass its version, e.g. --version 1.1.31")
    return Image(data, "factory", version)


# --- The unit --------------------------------------------------------------------------------
class Clock:
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


class SerialLink:
    def __init__(self, port: str):
        import serial  # pyserial: Klipper's own dependency

        try:
            self._ser = serial.Serial(port, BAUD, timeout=0, exclusive=True)
        except serial.SerialException as e:
            raise RefusedError(f"{port} cannot be opened: {e}") from None

    def write(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()

    def read(self) -> bytes:
        waiting = self._ser.in_waiting
        return self._ser.read(waiting) if waiting else b""

    def clear(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()


class Unit:
    def __init__(self, link, clock):
        self.link, self.clock, self._seq = link, clock, 0

    def ask(self, cmd: int, payload: bytes = b"", timeout: float = 1.0):
        """The unit's reply to one request, or None.  A forbidden command raises before the
        port is written."""
        found = self._exchange(cmd, payload, timeout, first=True)
        return found[0] if found else None

    def collect(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> list:
        """Every reply to one request within the timeout: units chained on one port all answer."""
        return self._exchange(cmd, payload, timeout, first=False)

    def _exchange(self, cmd: int, payload: bytes, timeout: float, first: bool) -> list:
        self._seq = self._seq % 0xFFFF + 1
        frame = build_frame(cmd, payload, self._seq)
        self.link.clear()
        self.link.write(frame)
        buf, found = bytearray(), []
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            data = self.link.read()
            if not data:
                self.clock.sleep(POLL_S)
                continue
            buf += data
            for f in parse_frames(buf):
                # the request's own echo, if the line returns it, is not a reply
                if f.cmd == cmd and (f.addr & RESPONSE_BIT or f.payload != payload):
                    found.append(f)
                    if first:
                        return found
        return found


@dataclass
class Seen:
    kind: str  # "factory" or "recovery"
    version: str
    boot_version: str


def classify(frame) -> Seen | None:
    try:
        fields = pb_fields(frame.payload)
    except ValueError:
        return None
    version, boot = _text(fields, 1), _text(fields, 2)
    if frame.addr & RESPONSE_BIT:
        return Seen("factory", version, boot)
    if boot:
        return Seen("recovery", version, boot)
    return None


def identify(unit: Unit) -> Seen | None:
    """GET_INFO, up to IDENTIFY_TRIES times: the first query after Klipper wrote to the port
    can be swallowed by the noise it left."""
    for _ in range(IDENTIFY_TRIES):
        frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
        if frame is not None:
            seen = classify(frame)
            if seen is not None:
                return seen
    return None


def read_uid(unit: Unit) -> str:
    frame = unit.ask(CMD_DISCOVER, timeout=IDENTIFY_TIMEOUT_S)
    if frame is None:
        raise RefusedError("no answer to DISCOVER (the unit's id)")
    try:
        fields = pb_fields(frame.payload)
    except ValueError:
        raise RefusedError("DISCOVER does not decode") from None
    words = [_first(fields, n, 0) for n in (1, 2, 3)]
    if not all(isinstance(w, int) for w in words):
        raise RefusedError("DISCOVER does not decode")
    return "".join(f"{w:08X}" for w in words)


def count_units(unit: Unit) -> int:
    """How many units answer DISCOVER on this port: a daisy chain answers at one address."""
    frames = unit.collect(CMD_DISCOVER, timeout=IDENTIFY_TIMEOUT_S)
    return len({f.payload for f in frames})


def read_calibration(unit: Unit) -> tuple:
    """(raw payload, 16 pairs) of GET_MOTOR_STATUS."""
    frame = unit.ask(CMD_GET_MOTOR_STATUS, timeout=IDENTIFY_TIMEOUT_S)
    if frame is None:
        raise RefusedError("no answer to GET_MOTOR_STATUS (the calibration)")
    pairs = []
    try:
        for sub in pb_fields(frame.payload).get(1, []):
            if not isinstance(sub, bytes):
                raise ValueError("not a message")
            fields = pb_fields(sub)
            a, b = _first(fields, 1, 0), _first(fields, 2, 0)
            if not (isinstance(a, int) and isinstance(b, int)):
                raise ValueError("a pair is not a number")
            pairs.append((a, b))
    except ValueError as e:
        raise RefusedError(f"GET_MOTOR_STATUS does not decode: {e}") from None
    if len(pairs) != 16:
        raise RefusedError(f"GET_MOTOR_STATUS decoded to {len(pairs)} pairs, not 16")
    return frame.payload, pairs


def calibration_text(raw: bytes, pairs: list) -> str:
    lines = [f"# GET_MOTOR_STATUS raw payload: {raw.hex()}"]
    lines += [f"{i:2d} {a:5d} {b:5d}" for i, (a, b) in enumerate(pairs)]
    return "\n".join(lines) + "\n"


def parse_calibration(text: str) -> list:
    pairs = []
    for n, line in enumerate(text.splitlines(), 1):
        if line.strip() and not line.startswith("#"):
            try:
                _, a, b = line.split()
                pairs.append((int(a), int(b)))
            except ValueError:
                raise RefusedError(f"calibration file line {n} does not parse") from None
    return pairs


def calibration_files(directory: Path, uid: str) -> list:
    return sorted(directory.glob(f"ace-calibration-{uid}-*.txt"))


# --- The host --------------------------------------------------------------------------------
class Moonraker:
    def __init__(self, url: str, opener=urllib.request.urlopen):
        self.url, self._open = url.rstrip("/"), opener

    def _call(self, path: str, post: bool = False):
        req = urllib.request.Request(self.url + path, data=b"" if post else None)
        try:
            with self._open(req, timeout=10) as r:
                return json.load(r).get("result")
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def state(self):
        """Klipper's state as Moonraker sees it (ready, startup, shutdown, error,
        disconnected), or None when Moonraker does not answer."""
        info = self._call("/server/info")
        return None if info is None else info.get("klippy_state")

    def objects(self) -> dict | None:
        """The status dict, or None when Moonraker does not answer the query."""
        res = self._call("/printer/objects/query?print_stats&ace2k&configfile=settings")
        return None if res is None else res.get("status", {})

    def gcode(self, script: str) -> None:
        self._call("/printer/gcode/script?script=" + urllib.parse.quote(script), post=True)

    def console_errors(self) -> list:
        res = self._call("/server/gcode_store?count=20")
        store = [] if res is None else res.get("gcode_store", [])
        return [e.get("message", "") for e in store if e.get("message", "").startswith("!!")]


@dataclass
class KlipperView:
    state: str | None  # None: Moonraker does not answer
    printing: bool | None  # None: unknown (the print query failed while Klipper is ready)
    ace2k_configured: bool
    ace2k_port: str | None
    ace2k_version: str | None


def klipper_view(moonraker) -> KlipperView:
    state = moonraker.state()
    if state is None:
        return KlipperView(None, False, False, None, None)
    status = moonraker.objects()
    if status is None:
        # not ready means not printing; ready with no answer means we cannot tell
        return KlipperView(state, None if state == "ready" else False, False, None, None)
    settings = status.get("configfile", {}).get("settings", {})
    ace2k = settings.get("ace2k")
    port = None
    if ace2k is not None:
        name = ace2k.get("mcu", "ace2k")
        port = settings.get("mcu" if name == "mcu" else f"mcu {name}", {}).get("serial")
    printing = status.get("print_stats", {}).get("state") in ("printing", "paused")
    version = status.get("ace2k", {}).get("version")
    return KlipperView(state, printing, ace2k is not None, port, version)


def same_device(a, b) -> bool:
    return a is not None and b is not None and os.path.realpath(a) == os.path.realpath(b)


def _cmdline(d: Path) -> str:
    try:
        return (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        return ""


def _holds(fd: Path, target: str) -> bool:
    try:
        link = os.readlink(fd)
    except OSError:
        return False
    return link == target or (link.startswith("/") and os.path.realpath(link) == target)


def port_holders(port: str, proc: Path) -> list:
    """(pid, command line) of every process holding the port open.

    A process whose fd directory cannot be read is skipped, unless it is Klipper: whether Klipper
    holds the port is then unknown, and the tool refuses rather than open a port it may hold.
    """
    if not proc.is_dir():
        raise RefusedError(f"cannot see who holds {port}: no {proc} (run this on the Klipper host)")
    target = os.path.realpath(port)
    holders = []
    for d in proc.glob("[0-9]*"):
        try:
            fds = list((d / "fd").iterdir())
        except PermissionError:
            if is_klippy(_cmdline(d)):
                raise RefusedError(
                    f"cannot see which ports Klipper (pid {d.name}) holds: "
                    "run this as root or as Klipper's user"
                ) from None
            continue
        except OSError:  # the process has exited
            continue
        if any(_holds(fd, target) for fd in fds):
            holders.append((int(d.name), _cmdline(d)))
    return holders


def is_klippy(cmdline: str) -> bool:
    return "klippy" in cmdline


def candidate_ports(by_id: Path) -> list:
    """The converter in the unit's own cable: a CH343 named usb-1a86_USB_Single_Serial_*
    (docs/flashing.md).  A printer's own CH340 mainboard enumerates as usb-1a86_USB2.0-Serial-*
    and is not listed."""
    return sorted(str(p) for p in by_id.glob("usb-1a86_USB_Single_Serial_*"))


@dataclass
class Service:
    stop: list
    start: list


def find_service(stop_cmd, start_cmd, which, run, initd: Path):
    if stop_cmd or start_cmd:
        if not (stop_cmd and start_cmd):
            raise RefusedError("--klipper-stop and --klipper-start go together")
        return Service(shlex.split(stop_cmd), shlex.split(start_cmd))
    if which("systemctl"):
        r = run(["systemctl", "cat", "klipper.service"], capture_output=True)
        if r.returncode == 0:
            return Service(["systemctl", "stop", "klipper"], ["systemctl", "start", "klipper"])
    if initd.exists():
        return Service([str(initd), "stop"], [str(initd), "start"])
    return None


# --- The flow --------------------------------------------------------------------------------
RESET_NOTE = (
    "  ACE_RESTORE_STOCK had already reset the unit into the bootloader's recovery (it stays "
    "there and accepts an update).  Klipper is being started again but will not connect until "
    "ace2k is written again: split the chain or check the port, then run the tool again "
    "(recovery is identified as such)."
)


@dataclass
class Deps:
    open_link: object  # port -> link
    moonraker: object
    clock: object
    run: object  # subprocess.run
    which: object  # shutil.which
    proc: Path
    by_id: Path
    initd: Path
    ask: object  # input
    out: object  # print
    now: object  # datetime.datetime.now


def update(unit: Unit, image: Image, out, clock) -> None:
    try:
        _update(unit, image, out, clock)
    except OSError as e:
        raise UpdateError(f"the serial link failed: {e}") from e


def _reply_code(reply) -> int:
    try:
        return _first(pb_fields(reply.payload), 1, 0)
    except ValueError as e:
        raise UpdateError(f"an answer from the unit does not decode: {e}") from None


def _update(unit: Unit, image: Image, out, clock) -> None:
    announce = (
        pb_uint(1, len(image.data)) + pb_uint(2, image.crc) + pb_bytes(3, image.announce.encode())
    )
    reply = unit.ask(CMD_IAP_UPGRADE, announce, ANNOUNCE_TIMEOUT_S)
    if reply is None:
        raise UpdateError("no answer to the announcement")
    code = _reply_code(reply)
    if code:
        raise UpdateError(f"the announcement was refused (code {code})")
    count = (len(image.data) + CHUNK_BYTES - 1) // CHUNK_BYTES
    for i in range(count):
        offset = i * CHUNK_BYTES
        payload = pb_uint(1, STAGING_BASE + offset) + pb_bytes(
            2, image.data[offset : offset + CHUNK_BYTES]
        )
        reply = unit.ask(CMD_IAP_FIRMWARE, payload, CHUNK_TIMEOUT_S)
        if reply is None:
            raise UpdateError(f"chunk {i + 1}/{count}: no answer")
        code = _reply_code(reply)
        if code:
            raise UpdateError(f"chunk {i + 1}/{count} refused (code {code})")
        clock.sleep(CHUNK_PAUSE_S)
        if (i + 1) % 64 == 0 or i + 1 == count:
            out(f"  {i + 1}/{count} chunks")
    # recovery commits and resets on the finish: an answer is not required
    unit.ask(CMD_IAP_FINISH, b"", FINISH_TIMEOUT_S)


class Flasher:
    def __init__(self, args, deps: Deps):
        self.args, self.d = args, deps
        self.out = deps.out
        self.stopped = False  # this run stopped Klipper
        self.restart_ok = True  # Klipper may be started again at exit
        self.service = None
        self.update_started = False
        self.start_tried = False  # a start attempt already failed and was reported

    # -- helpers --
    def _wait(self, seconds: float, done) -> bool:
        deadline = self.d.clock.now() + seconds
        while True:
            if done():
                return True
            if self.d.clock.now() >= deadline:
                return False
            self.d.clock.sleep(STATE_POLL_S)

    def _stop_klipper(self, port: str, after_restore: bool = False) -> None:
        cmd = shlex.join(self.service.stop)
        try:
            rc = getattr(self.d.run(self.service.stop), "returncode", 1)
            how = f"exit {rc}"
        except OSError as e:
            rc, how = 1, str(e)
        if rc != 0:
            tail = RESET_NOTE if after_restore else "; nothing changed"
            raise RefusedError(f"`{cmd}` failed ({how}): Klipper was not stopped{tail}")
        self.stopped = True
        if not self._wait(STOP_WAIT_S, lambda: not port_holders(port, self.d.proc)):
            raise RefusedError(f"Klipper was stopped but {port} is still held open")

    def _start_klipper(self) -> bool:
        cmd = shlex.join(self.service.start)
        try:
            rc = getattr(self.d.run(self.service.start), "returncode", 1)
            how = f"exit {rc}"
        except OSError as e:
            rc, how = 1, str(e)
        if rc != 0:
            self.out(f"Klipper did NOT start ({how}): run {cmd}")
            return False
        self.stopped = False
        return True

    def _resolve_port(self, view: KlipperView) -> str:
        if self.args.port:
            return self.args.port
        if view.ace2k_port:
            return view.ace2k_port
        lines = ["No port given and no [ace2k] MCU in Klipper's config to take it from."]
        ports = candidate_ports(self.d.by_id)
        if ports:
            lines.append("Adapters of the kind in the unit's cable:")
            for p in ports:
                holders = port_holders(p, self.d.proc) if self.d.proc.is_dir() else []
                note = "  (held open by Klipper)" if any(is_klippy(c) for _, c in holders) else ""
                lines.append(f"  {p}{note}")
        else:
            lines.append(f"No adapter of the kind in the unit's cable under {self.d.by_id}.")
        lines.append("Pass the unit's port: --port /dev/serial/by-id/usb-1a86_...")
        raise RefusedError("\n".join(lines))

    def _plan(self, image: Image, port: str, found: str, steps: list) -> None:
        self.out(f"Unit on {port}: {found}")
        self.out(
            f"Image: {image.kind} {image.version}, {len(image.data)} bytes, "
            f"CRC 0x{image.crc:04X}, announced as {image.announce!r}"
        )
        self.out("Steps:")
        for s in steps:
            self.out(f"  - {s}")
        if self.args.yes:
            return
        try:
            answer = self.d.ask("Proceed? [y/N] ")
        except EOFError:
            answer = ""
        if answer.strip().lower() not in ("y", "yes"):
            raise RefusedError("Not confirmed; nothing changed.")

    def _open_unit(self, port: str, reset: bool = False) -> Unit:
        if port_holders(port, self.d.proc):
            why = f"{port} is held open by another program"
            raise RefusedError(why + "." + RESET_NOTE if reset else why + "; nothing changed")
        self.link = self.d.open_link(port)
        return Unit(self.link, self.d.clock)

    # -- the flow --
    def run(self) -> int:
        self.link = None
        try:
            return self._run()
        except RefusedError as e:
            self.out(str(e))
            return EXIT_FAILED
        except UpdateError as e:
            self.out(f"The update failed: {e}.")
            self.out("The unit accepts a new update: run the same command again.")
            return EXIT_FAILED
        except OSError as e:
            self.out(f"Serial link error: {e}")
            if self.update_started:
                self.out("The unit accepts a new update: run the same command again.")
            return EXIT_FAILED
        finally:
            if self.link is not None:
                self.link.close()
            self._leave_klipper()

    def _leave_klipper(self) -> None:
        if not self.stopped or self.start_tried:
            return
        if self.restart_ok:
            self.out(f"Starting Klipper again: {shlex.join(self.service.start)}")
            self._start_klipper()
            return
        self.out("Klipper is left STOPPED: this run targets the factory firmware, which does not")
        self.out("speak Klipper's protocol (if the update failed, the unit is in recovery).")
        self.out("Remove [mcu ace2k] and [ace2k] from the config, including any #*# [ace2k]")
        self.out("section in the SAVE_CONFIG block at the end of printer.cfg, then start Klipper:")
        self.out(f"  {shlex.join(self.service.start)}")

    def _run(self) -> int:
        a, d = self.args, self.d
        image = load_image(Path(a.image), a.version)
        view = klipper_view(d.moonraker)
        port = self.port = self._resolve_port(view)
        if not Path(port).exists():
            raise RefusedError(f"{port} does not exist; nothing changed")
        if view.printing is None:
            raise RefusedError(
                "Klipper is ready but its print state could not be read; nothing changed"
            )
        if view.printing:
            raise RefusedError("A print is running or paused; nothing changed")
        self.service = find_service(a.klipper_stop, a.klipper_start, d.which, d.run, d.initd)
        stop_txt = shlex.join(self.service.stop) if self.service else None

        if view.state == "ready" and view.ace2k_version and same_device(view.ace2k_port, port):
            if self.service is None:
                raise RefusedError(
                    "ace2k runs under Klipper, but this tool does not know how to stop Klipper "
                    "here: pass --klipper-stop and --klipper-start; nothing changed"
                )
            self._plan(
                image,
                port,
                f"ace2k {view.ace2k_version}, under Klipper",
                [
                    "ACE_RESTORE_STOCK (the unit resets into the bootloader's recovery)",
                    f"stop Klipper: {stop_txt}",
                    f"write {image.kind} {image.version}",
                    *self._after_steps(image, tool_stops=True),
                ],
            )
            unit = self._enter_recovery(port)
            return self._flash(unit, image, view)

        holders = port_holders(port, d.proc)
        if holders and not all(is_klippy(c) for _, c in holders):
            listed = "; ".join(f"pid {p}: {c}" for p, c in holders)
            raise RefusedError(f"{port} is held open ({listed}); nothing changed")
        # Klipper's own config decides, not who holds the port this instant: a Klipper that is
        # not ready closes the port after a failed identify and re-opens it every few seconds
        configured_here = view.ace2k_configured and same_device(view.ace2k_port, port)
        not_ready = view.state in ("startup", "error", "shutdown")
        if (holders and view.state not in (None, "ready")) or (not_ready and configured_here):
            if self.service is None:
                raise RefusedError(
                    f"Klipper is running and not ready ({view.state}) and may hold {port}, but "
                    "this tool does not know how to stop it here: pass --klipper-stop and "
                    "--klipper-start; nothing changed"
                )
            self._plan(
                image,
                port,
                f"Klipper is not ready ({view.state}) and uses this port",
                [
                    f"stop Klipper: {stop_txt}",
                    "identify the unit; stop if it is neither the factory firmware nor recovery",
                    f"write {image.kind} {image.version}",
                    *self._after_steps(image, tool_stops=True),
                ],
            )
            self._stop_klipper(port)
            unit = self._open_unit(port)
            seen = identify(unit)
            if seen is None:
                raise RefusedError(
                    f"Nothing on {port} answered as the factory firmware or recovery.  If the "
                    "unit runs ace2k, get Klipper ready (FIRMWARE_RESTART) and run this again."
                )
            self._one_unit(unit, port)
            return self._flash_identified(unit, image, view, seen, confirmed=True)
        if holders:
            listed = "; ".join(f"pid {p}: {c}" for p, c in holders)
            raise RefusedError(f"{port} is held open ({listed}); nothing changed")

        unit = self._open_unit(port)
        seen = identify(unit)
        if seen is None:
            raise RefusedError(
                f"Nothing on {port} answered: not the factory firmware, not the bootloader's "
                "recovery, and Klipper does not report ace2k on it.  Check the port, the cable "
                "and the unit's power; nothing changed."
            )
        self._one_unit(unit, port)
        return self._flash_identified(unit, image, view, seen, confirmed=False)

    def _after_steps(self, image: Image, tool_stops: bool) -> list:
        start = f" ({shlex.join(self.service.start)})" if self.service else ""
        if image.kind == "factory":
            return [
                "check the factory firmware answers with that version; compare calibration",
                "Klipper is left stopped afterwards (the factory firmware does not speak its "
                f"protocol); start it later with{start or ' your start command'}",
            ]
        if tool_stops:
            return [f"start Klipper{start}; check it reports ace2k {image.version}"]
        return [
            "you start (or restart) Klipper when asked; the tool checks it reports ace2k "
            f"{image.version}"
        ]

    def _flash_identified(self, unit, image, view, seen: Seen, confirmed: bool) -> int:
        found = (
            f"factory firmware {seen.version}"
            if seen.kind == "factory"
            else f"bootloader recovery (boot {seen.boot_version}, last upload {seen.version!r})"
        )
        if not confirmed:
            steps = []
            if seen.kind == "factory":
                steps.append(f"save the calibration to {self.args.calibration_dir}")
            steps += [
                f"write {image.kind} {image.version}",
                *self._after_steps(image, tool_stops=False),
            ]
            self._plan(image, self.port, found, steps)
        else:
            self.out(f"Identified: {found}")
        if seen.kind == "factory":
            self._save_calibration(unit, compare=False)
        return self._flash(unit, image, view)

    def _enter_recovery(self, port: str) -> Unit:
        mr = self.d.moonraker
        before = Counter(mr.console_errors())
        mr.gcode("ACE_RESTORE_STOCK")

        def new_errors() -> list:
            seen, fresh = Counter(before), []
            for line in mr.console_errors():
                if seen[line] > 0:
                    seen[line] -= 1
                else:
                    fresh.append(line)
            return fresh

        def left_ready() -> bool:
            state = mr.state()
            return state is not None and state != "ready"

        # a refusal is printed at once; a reset takes Klipper seconds to notice
        self._wait(RESTORE_WAIT_S, lambda: left_ready() or bool(new_errors()))
        if not left_ready() and new_errors():
            self._wait(REFUSAL_SETTLE_S, left_ready)
        if not left_ready():
            fresh = new_errors()
            if fresh:
                raise RefusedError(
                    f"ACE_RESTORE_STOCK was refused: {fresh[-1]} — Klipper still reports ready, "
                    "so nothing was stopped.  If all four LEDs strobe fast, the unit did reset: "
                    "run the tool again (it identifies recovery)."
                )
            what = (
                "Moonraker does not answer"
                if mr.state() is None
                else f"Klipper still reports ready {RESTORE_WAIT_S:.0f} s after ACE_RESTORE_STOCK"
            )
            raise RefusedError(
                f"{what} and no refusal was printed: the unit may or may not have reset.  Nothing "
                "was stopped.  Look at the LEDs — all four strobing fast is recovery — and run "
                "the tool again (it identifies recovery on its own)."
            )
        self._stop_klipper(port, after_restore=True)
        unit = self._open_unit(port, reset=True)
        seen = identify(unit)
        if seen is None or seen.kind != "recovery":
            what = "nothing" if seen is None else f"the {seen.kind} firmware"
            raise RefusedError(
                f"Expected the bootloader's recovery on {port}, found {what}." + RESET_NOTE
            )
        self.out(f"Recovery answers (boot {seen.boot_version}).")
        self._one_unit(unit, port, reset=True)
        return unit

    def _one_unit(self, unit: Unit, port: str, reset: bool = False) -> None:
        n = count_units(unit)
        if n > 1:
            why = (
                f"{n} units answered on {port}: this tool flashes one unit per port "
                "(a daisy chain is not supported)"
            )
        elif n == 0:
            why = f"No unit answered DISCOVER on {port}"
        else:
            return
        if reset:
            raise RefusedError(why + "." + RESET_NOTE)
        raise RefusedError(why + "; nothing changed")

    def _flash(self, unit: Unit, image: Image, view: KlipperView) -> int:
        if image.kind == "factory":
            self.restart_ok = False
        self.update_started = True
        self.out(f"Writing {image.kind} {image.version} ...")
        update(unit, image, self.out, self.d.clock)
        self.out("Written.")
        if image.kind == "factory":
            return self._verify_factory(unit, image)
        return self._verify_ace2k(unit, image, view)

    def _verify_ace2k(self, unit: Unit, image: Image, view: KlipperView) -> int:
        self.out(
            f"Waiting {VERDICT_WAIT_S:.0f} s for the bootloader to copy the image and start it..."
        )
        self.d.clock.sleep(VERDICT_WAIT_S)
        for _ in range(VERDICT_TRIES):
            frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
            seen = None if frame is None else classify(frame)
            if seen is not None:
                if seen.kind == "factory":
                    raise RefusedError(
                        "The factory firmware answers after the update: the image was not taken.  "
                        "Check the image, then run the command again."
                    )
                raise RefusedError(
                    "The bootloader still answers after the update: the image was refused and "
                    "the unit is in recovery.  Check the image, then run the command again."
                )
        self.link.close()
        self.link = None
        here = view.ace2k_configured and same_device(view.ace2k_port, self.port)
        # Klipper stopped by hand: its config cannot be read, but it can be started and asked
        if view.state is None or not (here or view.state == "disconnected"):
            self.out("The bootloader let go of the unit; ace2k cannot be checked from here:")
            if self.stopped:
                self.start_tried = True
                if not self._start_klipper():
                    return EXIT_UNVERIFIED
                for line in (
                    "Klipper was started again with its current config, which has no [ace2k]",
                    "on this port: ace2k cannot be checked from here.",
                    "A NEWLY WRITTEN IMAGE GOES BACK TO RECOVERY IF NO HOST TALKS TO IT",
                    "WITHIN 180 s: add [mcu ace2k] and [ace2k] (docs/flashing.md) and",
                    "restart Klipper within that window.",
                    "If that window passes, the unit stays in recovery: finish the config,",
                    "restart Klipper, and run this command again.",
                ):
                    self.out(line)
                return EXIT_UNVERIFIED
            if view.state is None:
                self.out("Moonraker does not answer here.")
            else:
                self.out("Klipper's config has no [ace2k] on this port.")
            for line in (
                "A NEWLY WRITTEN IMAGE GOES BACK TO RECOVERY IF NO HOST TALKS TO IT",
                "WITHIN 180 s: add [mcu ace2k] and [ace2k] (docs/flashing.md) and",
                "start Klipper within that window.",
                "If that window passes, the unit stays in recovery: finish the config,",
                "restart Klipper, and run this command again.",
            ):
                self.out(line)
            return EXIT_UNVERIFIED
        if self.stopped:
            self.out(f"Starting Klipper: {shlex.join(self.service.start)}")
            if not self._start_klipper():
                raise RefusedError("Klipper did not start; the unit waits for it in ace2k")
        elif (live := klipper_view(self.d.moonraker).state) == "disconnected":
            self.out(
                "Start Klipper now. Waiting up to "
                f"{ACE2K_WAIT_S:.0f} s for it to report the new version (a newly written image "
                "goes back to recovery if no host talks to it within 180 s)."
            )
        elif live != "ready":
            how = f", or `{shlex.join(self.service.start)}`" if self.service else ""
            self.out(
                f"Restart Klipper (FIRMWARE_RESTART{how}). Waiting up to "
                f"{ACE2K_WAIT_S:.0f} s for it to report the new version (a newly written image "
                "goes back to recovery if no host talks to it within 180 s)."
            )
        mr = self.d.moonraker

        def reported() -> bool:
            v = klipper_view(mr)
            if not (v.ace2k_version and same_device(v.ace2k_port, self.port)):
                return False
            # an image from before the version constant: the firmware embeds its own string
            return v.ace2k_version == image.version or (
                not image.embedded and v.ace2k_version.encode() in image.data
            )

        if self._wait(ACE2K_WAIT_S, reported):
            self.out(f"Klipper reports ace2k {image.version}: done.")
            return EXIT_OK
        raise RefusedError(
            f"Klipper did not report ace2k {image.version} within {ACE2K_WAIT_S:.0f} s: check "
            "ACE_STATUS and klippy.log (a newly written image goes back to recovery if no host "
            "talks to it within 180 s of boot)"
        )

    def _verify_factory(self, unit: Unit, image: Image) -> int:
        want = image.version.lstrip("Vv")
        last = [None]

        def answered() -> bool:
            frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
            last[0] = None if frame is None else classify(frame)
            return last[0] is not None and last[0].kind == "factory"

        self.out(f"Waiting for the factory firmware to start (up to {FACTORY_WAIT_S:.0f} s)...")
        if not self._wait(FACTORY_WAIT_S, answered):
            what = "recovery still answers" if last[0] else "nothing answers"
            raise RefusedError(f"The factory firmware did not answer within the wait ({what})")
        got = last[0].version
        if got.lstrip("Vv") != want:
            raise RefusedError(f"The factory firmware answers with {got}, not {image.version}")
        self.out(f"The factory firmware answers: {got}.")
        try:
            self._save_calibration(unit, compare=True)
        except RefusedError as e:
            self.out(f"Warning: the calibration could not be read after the update: {e}")
        return EXIT_OK

    def _save_calibration(self, unit: Unit, compare: bool) -> None:
        directory = Path(self.args.calibration_dir)
        uid = read_uid(unit)
        raw, pairs = read_calibration(unit)
        earlier = calibration_files(directory, uid)
        stamp = self.d.now().strftime("%Y%m%d-%H%M%S")
        path = directory / f"ace-calibration-{uid}-{stamp}.txt"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            path.write_text(calibration_text(raw, pairs))
        except OSError as e:
            raise RefusedError(
                f"the calibration could not be written to {directory}: {e.strerror or e}"
            ) from e
        self.out(f"Calibration saved: {path}")
        if not compare:
            return
        if not earlier:
            self.out(f"No earlier calibration file of this unit in {directory}: not compared.")
            return
        before = parse_calibration(earlier[-1].read_text())
        differ = [i for i, (x, y) in enumerate(zip(before, pairs)) if x != y]
        if len(before) == len(pairs) and not differ:
            self.out(f"Calibration intact (same as {earlier[-1].name}).")
        else:
            self.out(f"WARNING: the calibration differs from {earlier[-1].name} in pairs {differ}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image", help="the image: an ace2k .bin, or the factory .bin")
    ap.add_argument("--port", help="the unit's serial port (default: from Klipper's config)")
    ap.add_argument("--version", help="the image's version, when the image does not carry it")
    ap.add_argument("--moonraker", default="http://localhost:7125", help="Moonraker's URL")
    ap.add_argument("--klipper-stop", help="the command that stops Klipper on this host")
    ap.add_argument("--klipper-start", help="the command that starts Klipper on this host")
    ap.add_argument("--calibration-dir", default=".", help="where calibration files go")
    ap.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    args = ap.parse_args(argv)
    deps = Deps(
        open_link=SerialLink,
        moonraker=Moonraker(args.moonraker),
        clock=Clock(),
        run=subprocess.run,
        which=shutil.which,
        proc=Path("/proc"),
        by_id=Path("/dev/serial/by-id"),
        initd=Path("/etc/init.d/S60klipper"),
        ask=input,
        out=print,
        now=datetime.datetime.now,
    )
    return Flasher(args, deps).run()


if __name__ == "__main__":
    sys.exit(main())
