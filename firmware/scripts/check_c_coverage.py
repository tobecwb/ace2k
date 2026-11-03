"""Fail when the C core's line or branch coverage falls below its floor.

Usage: make coverage | check_c_coverage.py --lines N --branches M
"""

from __future__ import annotations

import argparse
import sys

LINES_COLUMN = 2
BRANCHES_COLUMN = 3


def totals(report: str) -> tuple[float, float]:
    """Return (lines %, branches %) from the TOTAL row of llvm-cov report."""
    for line in report.splitlines():
        if line.startswith("TOTAL"):
            cells = [c for c in line.split() if c.endswith("%")]
            if len(cells) <= BRANCHES_COLUMN:
                raise SystemExit("check_c_coverage: no branch figure in the TOTAL row")
            # regions, functions, lines, branches
            return float(cells[LINES_COLUMN].rstrip("%")), float(cells[BRANCHES_COLUMN].rstrip("%"))
    raise SystemExit("check_c_coverage: no TOTAL row in the report")


def main(argv: list[str] | None = None) -> int:
    """Read the report on stdin; return 1 if a coverage figure is under its floor."""
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--lines", type=float, required=True)
    parser.add_argument("--branches", type=float, required=True)
    args = parser.parse_args(argv)
    lines, branches = totals(sys.stdin.read())
    print(
        f"C coverage: lines {lines:.2f} % (floor {args.lines}), "
        f"branches {branches:.2f} % (floor {args.branches})"
    )
    return 0 if lines >= args.lines and branches >= args.branches else 1


if __name__ == "__main__":
    sys.exit(main())
