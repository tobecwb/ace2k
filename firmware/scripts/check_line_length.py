"""Fail when a C source line is longer than the limit, counted in characters.

Usage: check_line_length.py --max N [--exclude NAME ...] PATH [PATH ...]

Each PATH is a file, or a directory that is searched for *.c and *.h files; a `build` directory
below it is skipped (the directory itself may sit under one). The length is counted in
characters, not bytes, so a line with "°" or "µ" is not over the limit only because those take
two bytes in UTF-8. clang-format wraps code at 100 columns but does not reflow comments, so this
check is what holds a comment to the limit. A file whose name is given with --exclude (a
generated table) is not checked. The script prints one line for each line over the limit and
exits with 1 if there is any.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SUFFIXES = {".c", ".h"}


def sources(paths: list[str], excluded: set[str]) -> list[Path]:
    """Return the .c and .h files under each path, sorted, without build output or exclusions."""
    found: list[Path] = []
    for path in map(Path, paths):
        if path.is_file():
            found.append(path)
        else:
            # Only the parts below the root: a checkout under a directory named build is checked.
            found.extend(
                p
                for p in path.rglob("*")
                if p.suffix in SUFFIXES and "build" not in p.relative_to(path).parts
            )
    return sorted(p for p in found if p.name not in excluded)


def long_lines(path: Path, limit: int) -> list[tuple[int, int]]:
    """Return (line number, length in characters) for each line of path over limit."""
    text = path.read_text(encoding="utf-8")
    return [
        (number, len(line))
        for number, line in enumerate(text.splitlines(), start=1)
        if len(line) > limit
    ]


def main(argv: list[str] | None = None) -> int:
    """Check every source under the paths; return 1 if a line is over the limit."""
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--max", type=int, required=True)
    parser.add_argument("--exclude", action="append", default=[])
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args(argv)
    files = sources(args.paths, set(args.exclude))
    over = [
        (path, number, length) for path in files for number, length in long_lines(path, args.max)
    ]
    for path, number, length in over:
        print(f"{path}:{number}: {length} characters (limit {args.max})")
    if over:
        return 1
    print(f"line-length: OK ({len(files)} files, at most {args.max} characters a line)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
