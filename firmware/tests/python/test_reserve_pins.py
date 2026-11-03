"""The RESERVE_PINS_* strings of the sources against the pins of pins.h, in every image.

Klipper's host refuses to configure a pin that the MCU reserves. The firmware spells its
reservations as strings ("PE12,PE8,..."), apart from the pin names of pins.h. These tests keep
the two in step. They read the sources the way the build does: the Kconfig dependencies decide
which images can be built, the Makefile decides which files each image compiles, and the #if
lines decide which string each file declares.
"""

from __future__ import annotations

import itertools
import re
from collections import Counter
from functools import cache
from pathlib import Path

FW = Path(__file__).resolve().parents[2]
SRC = FW / "src"

# Pins that an image reserves but that pins.h does not name. Nothing configures them.
RESERVED_UNNAMED = {
    # The cutout's latch-reset pin. It is never driven or configured, so it has no name.
    "PC9",
    # Reader A's interrupt line, unused, and TIM4's remapped channel 3.
    "PD14",
    # The debug port (SWD) and USART3's remapped pins.
    "PA13",
    "PA14",
    "PD8",
    "PD9",
    # The internal reference channel of the ADC, a pseudo-pin of Klipper's.
    "ADC_VREFINT",
}
LATCH_RESET = "PC9"
TRIAC_GATE = "PC8"

PIN_DEFINE = re.compile(r"^#define\s+(ACE2K_PIN_\w+)\s+ACE2K_PIN\('([A-Z])',\s*(\d+)\)", re.M)
CONFIG_LINE = re.compile(r"^config\s+(\w+)\s*$")
DEPENDS_LINE = re.compile(r"^\s*depends on\s+(.+?)\s*$")
SOURCE_LINE = re.compile(r"^\+?(?:serial-)?src-\$\(CONFIG_(\w+)\)\s*:?\+?=\s*(.+?)\s*$")
COMMENT_OR_STRING = re.compile(r'//[^\n]*|/\*.*?\*/|("(?:\\.|[^"\\])*")', re.S)
# A file that declares a reservation, however clang-format wraps the call
DECLARES = re.compile(r'DECL_CONSTANT_STR\(\s*"RESERVE_PINS_')
RESERVATION = re.compile(r'DECL_CONSTANT_STR\(\s*"(RESERVE_PINS_\w+)"\s*,(.*?)\)\s*;', re.S)
TOKEN = re.compile(r'"([^"]*)"|(\w+)')
ALWAYS_ON = {"MACH_GD32F303"}


def pins_h() -> dict[str, str]:
    """The pins pins.h names, as {"PE12": "ACE2K_PIN_FAN_LEFT", ...}."""
    text = (SRC / "ace2k_board" / "pins.h").read_text()
    return {f"P{port}{num}": name for name, port, num in PIN_DEFINE.findall(text)}


def kconfig() -> dict[str, list[str]]:
    """Each option of src/ace2k/Kconfig with the options it depends on."""
    deps: dict[str, list[str]] = {}
    current = None
    for line in (SRC / "ace2k" / "Kconfig").read_text().splitlines():
        if m := CONFIG_LINE.match(line):
            current = m.group(1)
            deps[current] = []
        elif current is not None and (m := DEPENDS_LINE.match(line)):
            deps[current] = [term.strip() for term in m.group(1).split("&&")]
    return deps


def sources() -> dict[str, str]:
    """Each source file under src/ that the build compiles, with the option that compiles it."""
    lines = (SRC / "ace2k" / "Makefile").read_text().splitlines()
    lines += (FW / "patches" / "0001-gd32f303-and-ace2k-hooks.patch").read_text().splitlines()
    files = {}
    for line in lines:
        if m := SOURCE_LINE.match(line):
            for name in m.group(2).split():
                files[name] = m.group(1)
    return files


def free_options() -> list[str]:
    """The options the Kconfig depends on but does not define, the console driver among them."""
    deps = kconfig()
    return sorted({d for ds in deps.values() for d in ds} - set(deps) - ALWAYS_ON)


def images() -> list[dict[str, bool]]:
    """Every set of options the Kconfig dependencies allow, ace2k itself on."""
    deps = kconfig()
    names = sorted(deps)
    free = free_options()
    found = []
    for values in itertools.product([False, True], repeat=len(names) + len(free)):
        cfg = dict(zip(names + free, values, strict=True)) | dict.fromkeys(ALWAYS_ON, True)
        if cfg["ACE2K"] and all(all(cfg[d] for d in deps[n]) for n in names if cfg[n]):
            found.append(cfg)
    return found


def release() -> dict[str, bool]:
    """The options of config/ace2k.config, the release image."""
    text = (FW / "config" / "ace2k.config").read_text()
    on = set(re.findall(r"^CONFIG_(\w+)=y$", text, re.M))
    return {name: name in on for name in [*kconfig(), *free_options(), *ALWAYS_ON]}


def active_lines(text: str, cfg: dict[str, bool]) -> list[str]:
    """The lines of text that the preprocessor keeps under cfg (#if CONFIG_X / #else only)."""
    kept = []
    stack: list[bool] = []
    for line in text.splitlines():
        directive = line.strip()
        if m := re.match(r"#if\s+CONFIG_(\w+)$", directive):
            stack.append(cfg.get(m.group(1), False))
        elif directive.startswith(("#if", "#elif")):
            raise ValueError(f"an #if this test cannot read: {directive}")
        elif directive == "#else":
            stack[-1] = not stack[-1]
        elif directive == "#endif":
            stack.pop()
        elif all(stack):
            kept.append(line)
    return kept


def strip_comments(text: str) -> str:
    return COMMENT_OR_STRING.sub(lambda m: m.group(1) or " ", text)


def expand(tokens: str, defines: dict[str, str]) -> str:
    """The string that a sequence of string literals and string macros makes."""
    out = []
    for literal, name in TOKEN.findall(tokens):
        out.append(literal if not name else defines[name])
    return "".join(out)


@cache
def reservations_in(path: str, flags: frozenset[str]) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """The RESERVE_PINS_* constants that the file declares with these options on."""
    text = (SRC / path).read_text()
    if not DECLARES.search(text):
        return ()
    lines = active_lines(strip_comments(text), dict.fromkeys(flags, True))
    defines = {}
    for line in lines:
        if m := re.match(r"\s*#define\s+(\w+)\s+((?:\"[^\"]*\"\s*)+)$", line):
            defines[m.group(1)] = expand(m.group(2), {})
    body = "\n".join(line for line in lines if not line.lstrip().startswith("#"))
    found = []
    for name, value in RESERVATION.findall(body):
        found.append((name, tuple(p for p in expand(value, defines).split(",") if p)))
    return tuple(found)


def reserved(cfg: dict[str, bool]) -> list[tuple[str, str]]:
    """(constant, pin) for every pin the image reserves, in declaration order."""
    flags = frozenset(name for name, on in cfg.items() if on)
    out = []
    for path, option in sorted(sources().items()):
        if cfg.get(option, False):
            for name, pins in reservations_in(path, flags):
                out.extend((name, pin) for pin in pins)
    return out


def test_every_file_with_a_reservation_is_one_the_build_compiles() -> None:
    declaring = {
        str(p.relative_to(SRC)) for p in SRC.rglob("*.[ch]") if DECLARES.search(p.read_text())
    }
    assert declaring, "no reservation found: the test reads the wrong tree"
    assert declaring <= set(sources()), declaring - set(sources())


def test_the_release_reserves_every_pin_of_pins_h() -> None:
    pins = {pin for _, pin in reserved(release())}
    missing = {pin: name for pin, name in pins_h().items() if pin not in pins}
    assert not missing, f"named in pins.h but not reserved: {missing}"


def test_every_reserved_pin_is_named_in_pins_h_or_listed_here() -> None:
    known = set(pins_h()) | RESERVED_UNNAMED
    for cfg in images():
        stray = {(name, pin) for name, pin in reserved(cfg) if pin not in known}
        assert not stray, f"reserved but not in pins.h: {sorted(stray)}"


def test_the_unnamed_pins_have_no_name_in_pins_h() -> None:
    assert not RESERVED_UNNAMED & set(pins_h())


def test_no_image_reserves_a_pin_twice() -> None:
    for cfg in images():
        counts = Counter(pin for _, pin in reserved(cfg))
        twice = {pin: n for pin, n in counts.items() if n > 1}
        on = sorted(name for name, value in cfg.items() if value)
        assert not twice, f"reserved more than once {twice} in the image with {on}"


def test_the_latch_reset_is_reserved_once_in_every_image() -> None:
    # Even an image with neither the lane nor the heater: the host must never drive it.
    for cfg in images():
        count = sum(pin == LATCH_RESET for _, pin in reserved(cfg))
        assert count == 1, sorted(name for name, value in cfg.items() if value)


def test_the_triac_gate_is_reserved_once_in_every_image() -> None:
    # Even an image without the heater: a host [output_pin] holding the gate high would run the
    # heater at full power with no fan interlock.
    for cfg in images():
        count = sum(pin == TRIAC_GATE for _, pin in reserved(cfg))
        assert count == 1, sorted(name for name, value in cfg.items() if value)


def test_the_release_is_one_of_the_images() -> None:
    assert release() in images()
