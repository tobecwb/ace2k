"""Every G-code macro the configuration declares is reachable by its own name.

Klipper's G-code parser splits a line into letters and values: a command name ends at its first
digit, and whatever follows is read as a parameter. A macro named `_PRINT_START_X2Y` would answer
to `_PRINT_START_X2` with a parameter `Y`, so it could never be called. The upstream parser
splits on `([A-Z_]+|[A-Z*])`; the Snapmaker U1's tree on `([A-Z_]+|[A-Z*/])`. Both are checked.
The configuration of this tree declares no macro today; the check guards any that is added. The
U1's macros live in the ace2k-u1 tree, which carries its own copy of this check.
"""

import re
from pathlib import Path

import pytest

CONFIG = Path(__file__).resolve().parents[3] / "config"
SPLITTERS = {
    "upstream": re.compile("([A-Z_]+|[A-Z*])"),
    "snapmaker-u1": re.compile("([A-Z_]+|[A-Z*/])"),
}
SECTION = re.compile(r"^\[gcode_macro\s+([^\]\s]+)\s*\]", re.MULTILINE)


def macros():
    """(file name, macro name) for every [gcode_macro NAME] section under config/."""
    found = []
    for path in sorted(CONFIG.glob("*.cfg")):
        for name in SECTION.findall(path.read_text(encoding="utf-8")):
            found.append((path.name, name))
    return found


def klipper_command(line, splitter):
    """The command name Klipper's parser reads off a line, as klippy/gcode.py does."""
    parts = splitter.split(line.upper())
    if len(parts) >= 3:
        return parts[1] + parts[2].strip()
    return line.upper()


@pytest.mark.parametrize("flavour", sorted(SPLITTERS))
def test_every_macro_name_is_its_own_command(flavour):
    # one test over every macro, not one per macro: with none declared it still runs, not skips
    bad = []
    for cfg, name in macros():
        cmd = klipper_command(name, SPLITTERS[flavour])
        if cmd != name.upper():
            bad.append(f"{cfg}: [gcode_macro {name}] is read as command {cmd!r}")
    assert not bad, (
        f"Klipper's {flavour} parser splits these macro names — a digit in a macro name starts a "
        "parameter; rename them without digits: " + "; ".join(bad)
    )


@pytest.mark.parametrize("flavour", sorted(SPLITTERS))
def test_the_check_catches_a_digit_in_a_name(flavour):
    assert klipper_command("_PRINT_START_X2Y", SPLITTERS[flavour]) != "_PRINT_START_X2Y"
    assert klipper_command("ACE_EJECT", SPLITTERS[flavour]) == "ACE_EJECT"
