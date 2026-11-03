#!/usr/bin/env python3
"""Compare clang-tidy output with the ratchet file.

usage: tidy_ratchet.py RATCHET OUTPUT [CLANG_TIDY_EXIT_STATUS [FIRMWARE]]

RATCHET has one `path: check1, check2` line per file (`#` starts a comment), the path relative to
FIRMWARE, the directory of the Makefile (by default the parent of this script's directory). OUTPUT
is the clang-tidy text. Fails on a (file, check) pair not in the ratchet, a ratchet entry that no
longer fires, any compiler error anywhere, and a clang-tidy that did not run.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

MIN_ARGS_WITH_STATUS = 4  # program, ratchet, output, clang-tidy exit status
MIN_ARGS_WITH_FIRMWARE = 5  # and the firmware directory
FIRMWARE = Path(__file__).resolve().parents[1]

DIAG = re.compile(
    r"^(?P<path>\S+?):\d+:\d+: (?:fatal )?(?:warning|error): .*\[(?P<checks>[^\]]+)\]\s*$"
)
COMPILER_ERROR = "clang-diagnostic-error"


def load_ratchet(path: str) -> dict[str, set[str]]:
    allowed = {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            name, _, checks = line.partition(":")
            allowed[name.strip()] = {c.strip() for c in checks.split(",") if c.strip()}
    return allowed


def relative_to_src(path: str, firmware: str) -> str | None:
    """Return the ratchet key of a diagnostic's path, or None for a path the ratchet ignores.

    A file under the firmware's own src is keyed by its path from the firmware directory
    (src/...), whatever directories hold the checkout. Any other path with a src part (Klipper's
    own src, for one) is keyed by its full path, which no entry names, so its finding fails. A path
    with no src part is ignored."""
    full = os.path.normpath(path)
    src = os.path.join(os.path.normpath(firmware), "src")
    if full.startswith(src + os.sep):
        return "src/" + Path(full[len(src) + 1 :]).as_posix()
    if "src" in full.split(os.sep):
        return full
    return None


def main(argv: list[str]) -> int:
    """argv: RATCHET OUTPUT [CLANG_TIDY_EXIT_STATUS]; returns the process exit status."""
    allowed = load_ratchet(argv[1])
    status = int(argv[3]) if len(argv) >= MIN_ARGS_WITH_STATUS else 0
    firmware = argv[4] if len(argv) >= MIN_ARGS_WITH_FIRMWARE else str(FIRMWARE)
    found = set()
    errors = []
    parsed = 0
    with open(argv[2], encoding="utf-8", errors="replace") as f:
        for line in f:
            m = DIAG.match(line.rstrip("\n"))
            if not m:
                continue
            parsed += 1
            checks = [c.strip() for c in m.group("checks").split(",")]
            if COMPILER_ERROR in checks:
                errors.append(line.rstrip())
                continue
            rel = relative_to_src(m.group("path"), firmware)
            if rel is None:
                continue
            found.update((rel, c) for c in checks if c != "-warnings-as-errors")
    problems = []
    if status != 0 and parsed == 0:
        problems.append(f"clang-tidy did not run: exit status {status} and no diagnostics")
    problems += [
        f"{p}: {c} is not in the ratchet"
        for p, c in sorted(found)
        if c not in allowed.get(p, set())
    ]
    problems += [
        f"{p}: {c} is in the ratchet but no longer fires; remove it"
        for p, checks in sorted(allowed.items())
        for c in sorted(checks)
        if (p, c) not in found
    ]
    problems += [f"compiler error: {e}" for e in errors]
    if problems:
        for text in problems:
            print(f"tidy-bind: {text}")
        return 1
    print(f"tidy-bind: OK ({len(allowed)} ratchet entries)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
