"""HKDF-SHA256 (RFC 5869) and HMAC-SHA256 from the standard library, and AES-128 in pure Python
(FIPS-197).

The host of Klipper has no cipher library that is sure to be there, and a tag needs only a few
blocks.
"""

from __future__ import annotations

import hashlib
import hmac
from collections.abc import Sequence

_RCON = (0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36)


def _xtime(a: int) -> int:
    a <<= 1
    return (a ^ 0x11B) if a & 0x100 else a


def _mul(a: int, b: int) -> int:
    r = 0
    while b:
        if b & 1:
            r ^= a
        a = _xtime(a)
        b >>= 1
    return r


def _tables() -> tuple[bytes, bytes]:
    """Return the S-box and its inverse.

    They come from the multiplicative inverse in GF(2^8) and the affine map.
    """
    exp, log = [0] * 255, [0] * 256
    p = 1
    for i in range(255):
        exp[i] = p
        log[p] = i
        p = _mul(p, 3)
    sbox, inv = [0] * 256, [0] * 256
    for x in range(256):
        y = 0 if x == 0 else exp[(255 - log[x]) % 255]
        s = y
        for shift in range(1, 5):
            s ^= ((y << shift) | (y >> (8 - shift))) & 0xFF
        s ^= 0x63
        sbox[x], inv[s] = s, x
    return bytes(sbox), bytes(inv)


_SBOX, _INV_SBOX = _tables()


def _expand(key: bytes) -> list[bytes]:
    words: list[list[int]] = [list(key[i : i + 4]) for i in range(0, 16, 4)]
    for i in range(4, 44):
        t = list(words[i - 1])
        if i % 4 == 0:
            t = [_SBOX[b] for b in t[1:] + t[:1]]
            t[0] ^= _RCON[i // 4 - 1]
        words.append([a ^ b for a, b in zip(words[i - 4], t)])
    return [bytes(sum(words[4 * r : 4 * r + 4], [])) for r in range(11)]


def _add(state: Sequence[int], rk: bytes) -> list[int]:
    return [a ^ b for a, b in zip(state, rk)]


def _shift(state: Sequence[int], inverse: bool = False) -> list[int]:
    out = [0] * 16
    for r in range(4):
        for c in range(4):
            src = r + 4 * ((c + r) % 4)
            if inverse:
                out[src] = state[r + 4 * c]
            else:
                out[r + 4 * c] = state[src]
    return out


def _mix(state: Sequence[int], coeffs: Sequence[int]) -> list[int]:
    out: list[int] = []
    for c in range(4):
        col = state[4 * c : 4 * c + 4]
        for r in range(4):
            v = 0
            for k in range(4):
                v ^= _mul(col[k], coeffs[(k - r) % 4])
            out.append(v)
    return out


_MIX = (2, 3, 1, 1)
_INV_MIX = (14, 11, 13, 9)


def aes128_encrypt_block(key: bytes, block: bytes) -> bytes:
    rks = _expand(key)
    s = _add(list(block), rks[0])
    for r in range(1, 10):
        s = _mix(_shift([_SBOX[b] for b in s]), _MIX)
        s = _add(s, rks[r])
    return bytes(_add(_shift([_SBOX[b] for b in s]), rks[10]))


def aes128_decrypt_block(key: bytes, block: bytes) -> bytes:
    rks = _expand(key)
    s = _add(list(block), rks[10])
    for r in range(9, 0, -1):
        s = [_INV_SBOX[b] for b in _shift(s, inverse=True)]
        s = _mix(_add(s, rks[r]), _INV_MIX)
    s = [_INV_SBOX[b] for b in _shift(s, inverse=True)]
    return bytes(_add(s, rks[0]))


def hmac_sha256(key: bytes, data: bytes) -> bytes:
    return hmac.new(key, data, hashlib.sha256).digest()


def hkdf_sha256(ikm: bytes, salt: bytes, info: bytes, length: int) -> bytes:
    prk = hmac_sha256(salt, ikm)
    out, t, i = b"", b"", 1
    while len(out) < length:
        t = hmac_sha256(prk, t + info + bytes([i]))
        out += t
        i += 1
    return out[:length]
