"""Klipper's MCU protocol, as much of it as the flasher needs.

The flasher uses it to ask a newly written ace2k image for its version, which is the image's
link proof.  It also sends an ace2k unit with no Klipper connected into the bootloader's recovery.
"""

from __future__ import annotations

import json
import zlib
from typing import TYPE_CHECKING, cast

from .wire import POLL_S, NothingAnsweredError, RefusedError, link_errors

if TYPE_CHECKING:
    from collections.abc import Callable

    from .wire import ClockProto, Json, LinkProto

# --- The new image's own link: Klipper's protocol -----------------------------------------------
# An ace2k image speaks only Klipper's MCU protocol (klippy/msgproto.py, src/command.c), at its
# own baud rate.  The flasher asks it the same questions Klipper asks when it connects.  First it
# reads the dictionary (identify), then it asks ace2k_version.  That question is the image's link
# proof.  An image that has proven its link never falls back into recovery (protocol.md, "The
# link proof").
ACE2K_BAUD = 250000  # CONFIG_SERIAL_BAUD of config/ace2k.config
KLIPPER_SYNC = 0x7E
KLIPPER_DEST = 0x10
KLIPPER_SEQ_MASK = 0x0F
KLIPPER_MIN, KLIPPER_MAX = 5, 64
IDENTIFY_RESPONSE_ID, IDENTIFY_ID = 0, 1  # fixed in every Klipper MCU
IDENTIFY_CHUNK = 40  # bytes of the dictionary per identify, the same as Klipper asks
ENTER_DICTIONARY_S = 30.0  # the longest read of the dictionary before ace2k_bootloader_enter
DICTIONARY_MAX = 65536  # a dictionary larger than this does not come from an ace2k image
PROVE_TIMEOUT_S = 1.0  # the wait for the answer to one question
PIECE_TIMEOUT_S = 0.3  # the same for a piece of the dictionary, once the image has answered
# The command that resets an idle unit into the bootloader's recovery, and its answer
BOOTLOADER_ENTER = "ace2k_bootloader_enter"
BOOTLOADER_ENTER_RESPONSE = "ace2k_bootloader_enter_response accepted=%c"
# The 7-bit groups of Klipper's variable-length encoding (VLQ), from the highest, as
# (shift, low, high).  A value outside [low, high) needs the group at that shift.  This is the
# same rule as the encoder in msgproto.
VLQ_GROUPS = (
    (28, -0x4000000, 0xC000000),
    (21, -0x80000, 0x180000),
    (14, -0x1000, 0x3000),
    (7, -0x20, 0x60),
)
VLQ_SIGN_BITS = 0x60  # when both bits are set in the first byte of a VLQ, the value is negative


def klipper_crc(data: bytes) -> int:
    """Return Klipper's crc16_ccitt over the header and payload of a block."""
    crc = 0xFFFF
    for byte in data:
        b = byte ^ (crc & 0xFF)
        b ^= (b << 4) & 0xFF
        crc = ((b << 8) | (crc >> 8)) ^ (b >> 4) ^ (b << 3)
        crc &= 0xFFFF
    return crc


def klipper_vlq(value: int) -> bytes:
    """Return Klipper's variable-length encoding of an integer (msgproto PT_uint32)."""
    out = bytearray()
    for shift, low, high in VLQ_GROUPS:
        if value >= high or value < low:
            out.append((value >> shift) & 0x7F | 0x80)
    out.append(value & 0x7F)
    return bytes(out)


def parse_klipper_vlq(data: bytes, pos: int) -> tuple[int, int]:
    """Decode the VLQ that starts at pos.

    It returns the value as an unsigned 32-bit number, and the position after it.  Data that
    ends in the middle of the VLQ raises IndexError.
    """
    c = data[pos]
    pos += 1
    value = c & 0x7F
    if (c & VLQ_SIGN_BITS) == VLQ_SIGN_BITS:
        value |= -0x20
    while c & 0x80:
        c = data[pos]
        pos += 1
        value = (value << 7) | (c & 0x7F)
    return value & 0xFFFFFFFF, pos


def klipper_block(seq: int, payload: bytes) -> bytes:
    """Return a block with sequence number seq that carries payload, ready to send to the MCU."""
    head = bytes([KLIPPER_MIN + len(payload), (seq & KLIPPER_SEQ_MASK) | KLIPPER_DEST]) + payload
    crc = klipper_crc(head)
    return head + bytes([crc >> 8, crc & 0xFF, KLIPPER_SYNC])


def parse_klipper_blocks(buf: bytearray) -> list[tuple[int, bytes]]:
    """Return the valid blocks at the front of buf, as (seq, payload), and remove them from buf.

    A byte that cannot start a valid block is dropped, and the parse tries again at the next byte.
    The MCU's own parser drops everything up to the next sync byte instead.
    """
    blocks: list[tuple[int, bytes]] = []
    while buf:
        if buf[0] == KLIPPER_SYNC:
            del buf[0]
            continue
        n = buf[0]
        if n < KLIPPER_MIN or n > KLIPPER_MAX:
            del buf[0]
            continue
        if len(buf) < n:
            break
        block = bytes(buf[:n])
        crc = (block[-3] << 8) | block[-2]
        ok = block[-1] == KLIPPER_SYNC and (block[1] & ~KLIPPER_SEQ_MASK) == KLIPPER_DEST
        if not ok or klipper_crc(block[:-3]) != crc:
            del buf[0]
            continue
        del buf[:n]
        blocks.append((block[1] & KLIPPER_SEQ_MASK, block[2:-3]))
    return blocks


def call_ids(dictionary: Json, command: str, response: str) -> tuple[int, int] | None:
    """Return the ids of the command and of its answer in an image's dictionary.

    It returns None when either is missing or has no id."""
    ask = dictionary.get("commands", {}).get(command)
    answer = dictionary.get("responses", {}).get(response)
    if ask is None or answer is None:
        return None
    return ask, answer


def has_call(dictionary: Json, command: str, response: str) -> bool:
    """Tell whether an image's dictionary has the command and its answer, each with an id."""
    return call_ids(dictionary, command, response) is not None


class KlipperUnit:
    """The new image, asked over Klipper's protocol.

    It asks one question at a time.  The answer is the first block that carries the wanted
    message, because the MCU sends one message per block.

    A block with no payload is the MCU's acknowledgement.  It carries the sequence number the MCU
    expects next.  When the MCU did not take a question, because it waited for another sequence
    number, the question is sent again with the number it expects.
    """

    def __init__(self, link: LinkProto, clock: ClockProto) -> None:
        self.link, self.clock, self.seq = link, clock, 0
        self.heard = False  # a valid block came in: an image is there, if deaf to a question

    def ask(
        self, payload: bytes, want: int, timeout: float = PROVE_TIMEOUT_S, again: bool = True
    ) -> bytes | None:
        """Return the parameter bytes of the answer whose message id is want, or None.

        The MCU answers first and acknowledges after (src/command.c).  So the acknowledgement of
        the question before may still come in.  It carries the sequence number of this
        question, which the MCU then takes, so it is not a refusal.  An acknowledgement with any
        other sequence number is a refusal.

        again: whether a question the MCU said nothing about, or took with no answer, goes once
        more.  Noise can take a whole block with no word back, and an acknowledgement of this
        question looks the same as a refusal from a session before that left the MCU waiting
        for the next number.  A command that must not run twice passes False.
        """
        with link_errors():
            return self._ask(payload, want, timeout, again)

    def _ask(self, payload: bytes, want: int, timeout: float, again: bool) -> bytes | None:
        self.link.clear()
        lost = False
        for _ in range(3):  # a refusal and a lost block can each cost a pass
            sent = self.seq
            # Sync bytes first: after noise (the factory protocol's probe) the MCU drops all up
            # to the next one, which would otherwise be this block's last byte.  After a lost
            # block, enough of them to fill any block a collision on the half-duplex line left
            # half read, which would otherwise take this one.  On a clean line the MCU skips
            # sync bytes before a block (src/command.c).
            sync = bytes([KLIPPER_SYNC]) * (KLIPPER_MAX + 1 if lost else 1)
            self.link.write(sync + klipper_block(sent, payload))
            self.seq = (sent + 1) & KLIPPER_SEQ_MASK
            answer, heard = self._wait(want, sent, timeout)
            if answer is not None:
                return answer
            if heard is not None and heard != self.seq:  # refused: the MCU expects heard
                self.seq = heard
            elif not again:
                return None
            elif heard is None:  # nothing heard: the block was lost, the MCU still expects it
                if lost:  # twice: nothing is there to answer
                    return None
                self.seq, lost = sent, True
        return None

    def _wait(self, want: int, sent: int, timeout: float) -> tuple[bytes | None, int | None]:
        """Read until the answer, an acknowledgement of the question or a refusal, or timeout.

        Return (the answer's parameter bytes or None, the sequence number the MCU named, or
        None when it named none but sent's, the late acknowledgement of the question before).
        """
        buf = bytearray()
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            data = self.link.read()
            if not data:
                self.clock.sleep(POLL_S)
                continue
            buf += data
            for seq, body in parse_klipper_blocks(buf):
                self.heard = True
                if not body:
                    if seq != sent:  # the answer comes before it: nothing more to wait for
                        return None, seq
                    continue
                try:
                    msgid, pos = parse_klipper_vlq(body, 0)
                except IndexError:
                    continue
                if msgid == want & 0xFFFFFFFF:  # an id from -32 to -1 decodes unsigned
                    return body[pos:], None
        return None, None

    def dictionary(self, deadline: float) -> Json | None:
        """Return the image's data dictionary (JSON compressed with zlib), read as Klipper reads
        it.

        It returns None when the image does not answer, answers something else, or the deadline
        passes.  Once anything came from the image, a piece lost on the line is asked again until
        the deadline: the image keeps sending its reports, and on the half-duplex line one of
        them can collide with a question.
        """
        self.heard = False  # this read has heard the image only from here on
        data = bytearray()
        while len(data) < DICTIONARY_MAX:
            if self.clock.now() >= deadline:
                return None
            q = klipper_vlq(IDENTIFY_ID) + klipper_vlq(len(data)) + klipper_vlq(IDENTIFY_CHUNK)
            params = self.ask(q, IDENTIFY_RESPONSE_ID, PIECE_TIMEOUT_S if data else PROVE_TIMEOUT_S)
            if params is None and self.heard:  # an image is there: the piece was lost, again
                continue
            if params is None:
                return None
            try:
                offset, pos = parse_klipper_vlq(params, 0)
                n = params[pos]
            except IndexError:
                return None
            chunk = params[pos + 1 : pos + 1 + n]
            if offset != len(data) or len(chunk) != n:
                return None
            data += chunk
            if len(chunk) < IDENTIFY_CHUNK:
                break
        try:
            # Klipper's dictionary is always an object.
            return cast("Json", json.loads(zlib.decompress(bytes(data))))
        except (zlib.error, ValueError):
            return None

    def version(self, dictionary: Json) -> str | None:
        """Ask ace2k_version, the link proof.  Return the version the image answers, or None."""
        ask = dictionary.get("commands", {}).get("ace2k_version")
        answer = dictionary.get("responses", {}).get("ace2k_version_response version=%s")
        if ask is None or answer is None:
            return None
        params = self.ask(klipper_vlq(ask), answer)
        if not params:
            return None
        return bytes(params[1 : 1 + params[0]]).decode("utf-8", "replace")

    def call(
        self, dictionary: Json, command: str, response: str, again: bool = True
    ) -> bytes | None:
        """Send a command with no parameters and return its answer's parameter bytes.

        It returns None when the image lacks the command or its answer, or does not answer.
        again: as ask's.
        """
        ids = call_ids(dictionary, command, response)
        if ids is None:
            return None
        return self.ask(klipper_vlq(ids[0]), ids[1], again=again)

    def bootloader_enter(self, dictionary: Json) -> bool | None:
        """Send ace2k_bootloader_enter.  Return whether the image accepted it.

        True: the unit resets into the bootloader's recovery about 50 ms later.  False: a veto,
        because a lane moves or the heater or the fans are busy; nothing changed.  None: no
        answer, so the unit may or may not reset.
        """
        # not again: an accepted command resets the unit, maybe before its answer is out
        params = self.call(dictionary, BOOTLOADER_ENTER, BOOTLOADER_ENTER_RESPONSE, again=False)
        try:
            return parse_klipper_vlq(params, 0)[0] == 1 if params else None
        except IndexError:  # an answer cut short counts as none
            return None

    def proven(self, dictionary: Json) -> bool | None:
        """Ask ace2k_linkproof_state.  Return whether the image stored its link proof.

        The image answers its version before it stores the proof.  The store can fail, or it can
        come after the image's waiting time has run out.  It returns None when the image does
        not answer.
        """
        ask = dictionary.get("commands", {}).get("ace2k_linkproof_state")
        answer = dictionary.get("responses", {}).get(
            "ace2k_linkproof_state_response proven=%c remaining_ms=%u"
        )
        if ask is None or answer is None:
            return None
        params = self.ask(klipper_vlq(ask), answer)
        if not params:
            return None
        try:
            return parse_klipper_vlq(params, 0)[0] == 1
        except IndexError:
            return None


def enter_recovery(
    link: LinkProto, clock: ClockProto, confirm: Callable[[str], None], port: str
) -> None:
    """Send an ace2k unit with no Klipper connected into the bootloader's recovery over link.

    It reads the dictionary, as Klipper does when it connects.  When nothing answers, or the
    image lacks ace2k_bootloader_enter or its answer, it raises NothingAnsweredError: nothing
    that changes the unit was sent.  Otherwise it calls confirm with the image's version
    (Klipper's, when the image has none).  confirm may raise to stop before the command.  Then
    it sends ace2k_bootloader_enter.  A veto (a lane moves, or the heater or the fans are busy)
    is a refusal with nothing changed.  It returns None both when the unit accepted and when no
    answer came (then the unit may or may not reset): the caller looks for recovery either way.
    """
    unit = KlipperUnit(link, clock)
    dictionary = unit.dictionary(clock.now() + ENTER_DICTIONARY_S) or {}
    if not has_call(dictionary, BOOTLOADER_ENTER, BOOTLOADER_ENTER_RESPONSE):
        raise NothingAnsweredError(
            f"Nothing on {port} answered that this flasher can use: not the factory firmware, "
            "not the bootloader's recovery, and no ace2k that this flasher can send to recovery "
            "(an image older than ace2k_bootloader_enter).  Check the port, the cable and the "
            "unit's power; nothing changed."
        )
    config = dictionary.get("config", {})  # an image's version, as the build embeds it
    confirm(str(config.get("ACE2K_VERSION") or dictionary.get("version", "?")))
    if unit.bootloader_enter(dictionary) is False:
        raise RefusedError(
            f"The unit on {port} refused to enter recovery: a lane moves, or the heater or "
            "the fans are busy.  Wait until it is idle, then run this again; nothing changed.",
            unchanged=True,
        )
