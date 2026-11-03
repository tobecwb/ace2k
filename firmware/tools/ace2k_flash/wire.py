"""The original Anycubic protocol, as the flasher speaks it.

This module holds the frames and their CRC, and the few protobuf fields the flasher reads and
writes.  It holds the serial link and the questions the flasher asks the unit (GET_INFO,
DISCOVER, GET_MOTOR_STATUS).  It also holds the update itself: the announce, the 64-byte chunks
and the finish.  The errors the modules share (RefusedError, UpdateError) and the exit statuses
are defined here, because this module needs no other module of the flasher to run.
"""

from __future__ import annotations

import struct
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator
    from typing import Any, Protocol

    from .image import Image

    # A JSON object, as Moonraker and an image's dictionary give it
    Json = dict[str, Any]
    # The decoded fields of a protobuf message: field number to its values
    Fields = dict[int, list[int | bytes]]

    class LinkProto(Protocol):
        """The serial link to a unit: SerialLink, or a simulated unit in a test."""

        def write(self, data: bytes) -> None: ...
        def read(self) -> bytes: ...
        def clear(self) -> None: ...
        def close(self) -> None: ...

    class ClockProto(Protocol):
        """The clock that the waits use: Clock, or a virtual clock in a test.  Times are in
        seconds."""

        def now(self) -> float: ...
        def sleep(self, seconds: float) -> None: ...


# --- The original protocol (docs/protocol.md) -----------------------------------------------
BAUD = 230400
PREAMBLE = b"\xff\xaa"
END = 0xFE
MAX_PAYLOAD = 100  # the same limit that every tool of the original protocol keeps
# Set in the address byte of the replies of the factory firmware.  The bootloader does not set it.
RESPONSE_BIT = 0x80
# The bytes of a frame besides its payload: preamble 2, address 1, sequence 2, command 1,
# length 1, CRC 2, end 1
FRAME_OVERHEAD = 10

CMD_DISCOVER = 0
CMD_IAP_UPGRADE = 2
CMD_IAP_FIRMWARE = 3
CMD_IAP_FINISH = 4
CMD_GET_INFO = 7
CMD_FLASH_LED = 70
CMD_GET_MOTOR_STATUS = 78
# The only commands this tool sends.  Commands 15, 17 and 18 write the unit's calibration and
# configuration, so the tool never sends them.  FLASH_LED drives only the four lane LEDs
# (components 0x0F).
ALLOWED_COMMANDS = frozenset(
    {
        CMD_DISCOVER,
        CMD_IAP_UPGRADE,
        CMD_IAP_FIRMWARE,
        CMD_IAP_FINISH,
        CMD_GET_INFO,
        CMD_GET_MOTOR_STATUS,
        CMD_FLASH_LED,
    }
)

# The update writes the image in pieces of CHUNK_BYTES, to the staging area at STAGING_BASE
STAGING_BASE = 0x08024000
CHUNK_BYTES = 64
# The number of (a, b) calibration pairs in the answer to GET_MOTOR_STATUS
CALIBRATION_PAIRS = 16
# The code the unit returns for a chunk that does not follow the last one it received.  The
# factory firmware also returns it for the first chunk of a new run while it still holds an
# unfinished update.
CODE_FAILED = 3

# --- Timing of the exchanges with the unit.  The flasher's own waits are in base.py. ----------
IDENTIFY_TRIES = 5
# Units chained on one port all answer DISCOVER at the same time.  Two replies can collide on the
# bus, in about 1 % of the rounds with two units.  Several rounds make a missed unit unlikely.
DISCOVER_ROUNDS = 3
IDENTIFY_TIMEOUT_S = 1.0
ANNOUNCE_TIMEOUT_S = 5.0
CHUNK_TIMEOUT_S = 2.0
CHUNK_PAUSE_S = 0.015
FINISH_TIMEOUT_S = 2.0
POLL_S = 0.005

# --- The flasher's exit statuses (flasher.py's docstring says what each one means) ----------
# Here, at the bottom, so that flasher.py and blink.py both import them.
EXIT_OK, EXIT_FAILED, EXIT_UNVERIFIED, EXIT_UNCHANGED, EXIT_NO_ANSWER = 0, 1, 2, 3, 4
EXIT_NO_BLINK = 5  # --identify only


# --- Errors ---------------------------------------------------------------------------------
class RefusedError(Exception):
    """A stop that the user must read.  Its message is printed as it is.

    `unchanged` overrides the run's own record of whether anything was done.  With None, the
    run's record decides.
    """

    def __init__(self, message: str, unchanged: bool | None = None) -> None:
        super().__init__(message)
        self.unchanged = unchanged


class NothingAnsweredError(RefusedError):
    """Nothing answered as the factory firmware, as recovery or as ace2k (EXIT_NO_ANSWER).

    Nothing that changes the unit was sent, even when Klipper was stopped to free the port.
    """


class LinkError(OSError):
    """The serial link to the unit failed.

    This covers a read, a write or a clear of the port, but not its opening.  An OSError from
    anything else is not a LinkError.
    """


@contextmanager
def link_errors() -> Iterator[None]:
    """Raise an OSError of the block as a LinkError, with the same text."""
    try:
        yield
    except OSError as e:
        raise LinkError(str(e)) from e


class UpdateError(Exception):
    """The update itself went wrong.

    held is True when the first chunk was refused as out of order.  Then the unit already holds
    an unfinished update.
    """

    def __init__(self, message: str, held: bool = False) -> None:
        super().__init__(message)
        self.held = held


# --- Frames and protobuf fields -------------------------------------------------------------
def crc16(data: bytes) -> int:
    """Return the CRC-16 of the original protocol: init 0xFFFF, reflected 0x8408, no final
    XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def build_frame(cmd: int, payload: bytes, seq: int) -> bytes:
    """Return a frame of the original protocol, for address 0.

    A command this tool does not send, or a payload above MAX_PAYLOAD, is refused.
    """
    if cmd not in ALLOWED_COMMANDS:
        raise RefusedError(f"command {cmd} is not one this tool sends")
    if len(payload) > MAX_PAYLOAD:
        raise RefusedError(f"payload of {len(payload)} bytes is above the {MAX_PAYLOAD}-byte cap")
    body = bytes([0x00]) + struct.pack("<H", seq & 0xFFFF) + bytes([cmd, len(payload)]) + payload
    return PREAMBLE + body + struct.pack("<H", crc16(body)) + bytes([END])


@dataclass
class Frame:
    """A frame received from the unit."""

    addr: int
    seq: int
    cmd: int
    payload: bytes


def parse_frames(buf: bytearray) -> list[Frame]:
    """Return every valid frame in buf, in order, and remove from buf all bytes up to the end of
    the last one.

    A false preamble is noise that happens to read FF AA.  It is skipped, not waited on, so a
    real frame after it is still found.
    """
    frames: list[Frame] = []
    pos, consumed = 0, 0
    while True:
        i = buf.find(PREAMBLE, pos)
        if i < 0 or len(buf) - i < FRAME_OVERHEAD:
            break
        n = buf[i + 6]
        total = n + FRAME_OVERHEAD
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


# The protobuf wire types, and the largest value that one byte of a varint holds
PB_VARINT, PB_FIXED64, PB_LENGTH, PB_FIXED32 = 0, 1, 2, 5
VARINT_BYTE_MAX = 0x7F


def varint(value: int) -> bytes:
    """Return the protobuf varint encoding of a value that is not negative."""
    out = bytearray()
    while value > VARINT_BYTE_MAX:
        out.append((value & 0x7F) | 0x80)
        value >>= 7
    out.append(value)
    return bytes(out)


def pb_uint(field: int, value: int) -> bytes:
    """Return a protobuf varint field."""
    return varint(field << 3) + varint(value)


def pb_bytes(field: int, data: bytes) -> bytes:
    """Return a protobuf length-delimited field that holds data."""
    return varint((field << 3) | PB_LENGTH) + varint(len(data)) + data


def _read_varint(data: bytes, pos: int) -> tuple[int, int]:
    """Decode the varint at pos.  Return its value and the position after it.

    Data that ends in the middle of the varint raises ValueError.
    """
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


def pb_fields(data: bytes) -> Fields:
    """Decode a protobuf message into a map from field number to its list of values.

    A varint gives an int.  A length-delimited or fixed-width field gives bytes.  A message that
    is cut short, or that uses another wire type, raises ValueError.
    """
    fields: Fields = {}
    pos = 0
    value: int | bytes
    while pos < len(data):
        tag, pos = _read_varint(data, pos)
        number, wire = tag >> 3, tag & 7
        if wire == PB_VARINT:
            value, pos = _read_varint(data, pos)
        elif wire == PB_LENGTH:
            size, pos = _read_varint(data, pos)
            if pos + size > len(data):
                raise ValueError("truncated field")
            value, pos = data[pos : pos + size], pos + size
        elif wire in (PB_FIXED64, PB_FIXED32):
            width = 8 if wire == PB_FIXED64 else 4
            if pos + width > len(data):
                raise ValueError("truncated field")
            value, pos = data[pos : pos + width], pos + width
        else:
            raise ValueError(f"wire type {wire}")
        fields.setdefault(number, []).append(value)
    return fields


def _first(fields: Fields, number: int, default: int | bytes) -> int | bytes:
    """Return the first value of a field, or default when the field is missing."""
    return fields.get(number, [default])[0]


def _text(fields: Fields, number: int) -> str:
    """Return the first value of a field as text, or "" when it is missing or a number."""
    value = _first(fields, number, b"")
    return value.decode(errors="replace") if isinstance(value, bytes) else ""


# --- The unit --------------------------------------------------------------------------------
class Clock:
    """The real clock, in seconds."""

    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


class SerialLink:
    """The unit's serial port, opened for this program alone."""

    def __init__(self, port: str, baud: int = BAUD) -> None:
        # pyserial is a dependency of Klipper itself.  It is loaded only when a port opens, so
        # --help and the tests run without it.
        import serial  # noqa: PLC0415 — imported where a port opens

        try:
            self._ser = serial.Serial(port, baud, timeout=0, exclusive=True)
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
    """A unit on a serial link, asked over the original protocol."""

    def __init__(self, link: LinkProto, clock: ClockProto) -> None:
        self.link, self.clock, self._seq = link, clock, 0

    def ask(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> Frame | None:
        """Return the unit's reply to one request, or None.

        A command this tool does not send raises RefusedError before anything is written to the
        port.
        """
        found = self._exchange(cmd, payload, timeout, first=True)
        return found[0] if found else None

    def collect(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> list[Frame]:
        """Return every reply to one request within the timeout.  All units chained on one port
        answer."""
        return self._exchange(cmd, payload, timeout, first=False)

    def _exchange(self, cmd: int, payload: bytes, timeout: float, first: bool) -> list[Frame]:
        self._seq = self._seq % 0xFFFF + 1
        frame = build_frame(cmd, payload, self._seq)
        with link_errors():
            return self._send_and_collect(cmd, payload, frame, timeout, first)

    def _send_and_collect(
        self, cmd: int, payload: bytes, frame: bytes, timeout: float, first: bool
    ) -> list[Frame]:
        self.link.clear()
        self.link.write(frame)
        buf = bytearray()
        found: list[Frame] = []
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            data = self.link.read()
            if not data:
                self.clock.sleep(POLL_S)
                continue
            buf += data
            for f in parse_frames(buf):
                # If the line echoes the request back, that echo is not a reply
                if f.cmd == cmd and (f.addr & RESPONSE_BIT or f.payload != payload):
                    found.append(f)
                    if first:
                        return found
        return found


@dataclass
class Seen:
    """What answered GET_INFO: the factory firmware or the bootloader's recovery."""

    kind: str  # "factory" or "recovery"
    version: str
    boot_version: str


def classify(frame: Frame) -> Seen | None:
    """Tell from a GET_INFO reply what answered, or return None.

    A reply with the response bit set comes from the factory firmware.  A reply without it that
    carries a bootloader version comes from recovery.  Anything else is None.
    """
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
    """Ask GET_INFO, up to IDENTIFY_TRIES times, and return what answered, or None.

    It asks more than once because Klipper may have written to the port before.  The noise it
    left there can swallow the first query.
    """
    for _ in range(IDENTIFY_TRIES):
        frame = unit.ask(CMD_GET_INFO, timeout=IDENTIFY_TIMEOUT_S)
        if frame is not None:
            seen = classify(frame)
            if seen is not None:
                return seen
    return None


def read_uid(unit: Unit) -> str:
    """Return the unit's id from DISCOVER, as 24 hex digits.

    No answer, or an answer that does not decode, is refused.
    """
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
    return "".join(f"{w:08X}" for w in cast("list[int]", words))  # the check above: all numbers


def count_units(unit: Unit) -> int:
    """Return how many units answer DISCOVER on this port.

    All the units of a daisy chain answer at the same address.  The question is asked in
    DISCOVER_ROUNDS rounds, and each unit is counted once.  So a reply lost to a collision in
    one round is found in another.
    """
    seen: set[bytes] = set()
    for _ in range(DISCOVER_ROUNDS):
        seen |= {f.payload for f in unit.collect(CMD_DISCOVER, timeout=IDENTIFY_TIMEOUT_S)}
    return len(seen)


LED_COMPONENTS = 0x0F  # the four lane LEDs; the field's other bits drive other parts


def flash_leds(unit: Unit) -> bool:
    """Blink the four lane LEDs of a unit that runs the factory firmware.

    Two loops of five slow flashes, about 6 s.  It returns True when the unit answered.  Only the
    LED components are set: the field's other bits drive other parts of the unit.
    """
    payload = b"".join(
        pb_uint(n, v) for n, v in ((1, LED_COMPONENTS), (2, 2), (3, 0), (4, 4), (5, 0), (6, 0))
    )
    return unit.ask(CMD_FLASH_LED, payload, timeout=IDENTIFY_TIMEOUT_S) is not None


def read_calibration(unit: Unit) -> tuple[bytes, list[tuple[int, int]]]:
    """Ask GET_MOTOR_STATUS.  Return its raw payload and its 16 calibration pairs.

    No answer, an answer that does not decode, or a wrong number of pairs is refused.
    """
    frame = unit.ask(CMD_GET_MOTOR_STATUS, timeout=IDENTIFY_TIMEOUT_S)
    if frame is None:
        raise RefusedError("no answer to GET_MOTOR_STATUS (the calibration)")
    pairs: list[tuple[int, int]] = []
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
    if len(pairs) != CALIBRATION_PAIRS:
        raise RefusedError(
            f"GET_MOTOR_STATUS decoded to {len(pairs)} pairs, not {CALIBRATION_PAIRS}"
        )
    return frame.payload, pairs


def calibration_text(raw: bytes, pairs: list[tuple[int, int]]) -> str:
    """Return the text of a calibration file: the raw payload, then one pair per line."""
    lines = [f"# GET_MOTOR_STATUS raw payload: {raw.hex()}"]
    lines += [f"{i:2d} {a:5d} {b:5d}" for i, (a, b) in enumerate(pairs)]
    return "\n".join(lines) + "\n"


def parse_calibration(text: str) -> list[tuple[int, int]]:
    """Return the pairs of a calibration file.  Comment lines and empty lines are skipped.

    A line that does not parse is refused.
    """
    pairs: list[tuple[int, int]] = []
    for n, line in enumerate(text.splitlines(), 1):
        if line.strip() and not line.startswith("#"):
            try:
                _, a, b = line.split()
                pairs.append((int(a), int(b)))
            except ValueError:
                raise RefusedError(f"calibration file line {n} does not parse") from None
    return pairs


def calibration_files(directory: Path, uid: str) -> list[Path]:
    """Return the calibration files of the unit uid in directory, sorted by name.

    The name ends with the time it was saved, so the oldest file comes first.
    """
    return sorted(directory.glob(f"ace-calibration-{uid}-*.txt"))


# --- The update ------------------------------------------------------------------------------
def update(unit: Unit, image: Image, out: Callable[[str], None], clock: ClockProto) -> None:
    """Write image to the unit: the announce, the chunks, and the finish.

    out prints the progress.  A failure of the serial link raises UpdateError.
    """
    try:
        _update(unit, image, out, clock)
    except OSError as e:
        raise UpdateError(f"the serial link failed: {e}") from e


def _reply_code(reply: Frame, what: str) -> int:
    """Return the code in field 1 of a reply.  0 is success.

    An answer that does not decode, or whose field 1 holds bytes, is neither a success nor a
    refusal.  It raises UpdateError, and what names the answer in its message.
    """
    try:
        code = _first(pb_fields(reply.payload), 1, 0)
    except ValueError as e:
        raise UpdateError(f"an answer from the unit does not decode: {e}") from None
    if isinstance(code, bytes):
        raise UpdateError(f"{what} carries no code: its field 1 is not a number")
    return code


def _update(unit: Unit, image: Image, out: Callable[[str], None], clock: ClockProto) -> None:
    announce = (
        pb_uint(1, len(image.data)) + pb_uint(2, image.crc) + pb_bytes(3, image.announce.encode())
    )
    reply = unit.ask(CMD_IAP_UPGRADE, announce, ANNOUNCE_TIMEOUT_S)
    if reply is None:
        raise UpdateError("no answer to the announcement")
    code = _reply_code(reply, "the announcement's answer")
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
        code = _reply_code(reply, f"the answer to chunk {i + 1}/{count}")
        if code:
            raise UpdateError(
                f"chunk {i + 1}/{count} refused (code {code})",
                held=i == 0 and code == CODE_FAILED,
            )
        clock.sleep(CHUNK_PAUSE_S)
        if (i + 1) % 64 == 0 or i + 1 == count:
            out(f"  {i + 1}/{count} chunks")
    # On the finish, recovery takes the image and resets, so no answer is required
    unit.ask(CMD_IAP_FINISH, b"", FINISH_TIMEOUT_S)
