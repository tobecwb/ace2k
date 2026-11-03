"""The watchdog's prescaler and reload, the same in the chip glue, the board code and Klipper.

Klipper's stm32/watchdog.c writes the prescaler and the reload as literals. The chip glue
(stm32/gd32f30x.c) arms the watchdog with the same values before Klipper does, and the board code
(ace2k_board/sysinfo.c) checks that the watchdog runs with them. Each of the two ace2k files
defines the values on its own, because the chip glue cannot include a board header. These tests
read the three files and fail when the values differ.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

FW = Path(__file__).resolve().parents[2]

FILES = {
    "chip glue": FW / "src" / "stm32" / "gd32f30x.c",
    "board": FW / "src" / "ace2k_board" / "sysinfo.c",
    "Klipper": FW / "klipper" / "src" / "stm32" / "watchdog.c",
}
REGISTERS = ("PR", "RLR")

DEFINE = re.compile(r"^#define\s+(\w+)\s+(\w+)\b", re.M)
# A write (IWDG->PR = X) or a check (IWDG->PR == X).
USE = re.compile(r"\bIWDG->(PR|RLR)\s*==?\s*(\w+)")


def number(token: str) -> int:
    """The value of a C integer literal such as 0, 0U or 0x0FFFU."""
    return int(token.rstrip("uUlL"), 0)


def values(path: Path) -> dict[str, set[int]]:
    """For each register, the values the file writes to it or checks it against."""
    text = path.read_text()
    defines = dict(DEFINE.findall(text))
    found: dict[str, set[int]] = {reg: set() for reg in REGISTERS}
    for reg, token in USE.findall(text):
        found[reg].add(number(defines.get(token, token)))
    return found


@pytest.mark.parametrize("name", FILES)
def test_each_file_uses_one_value_per_register(name: str) -> None:
    found = values(FILES[name])
    for reg in REGISTERS:
        assert len(found[reg]) == 1, f"{name}: IWDG->{reg} with {sorted(found[reg])}"


def test_the_three_files_agree() -> None:
    seen = {name: values(path) for name, path in FILES.items()}
    for reg in REGISTERS:
        by_file = {name: sorted(found[reg]) for name, found in seen.items()}
        assert len({tuple(v) for v in by_file.values()}) == 1, f"IWDG->{reg}: {by_file}"
