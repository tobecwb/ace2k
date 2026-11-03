"""Fail when a Python or C module has too many lines of code.

Usage: check_module_size.py --max-code-lines N --target-lines M PATH [PATH ...]

Each PATH is a file, or a directory that is searched for *.py, *.c and *.h files. In Python, a code
line is a physical line that holds part of a statement; blank lines, lines with only a comment and
docstrings do not count. In C, blank lines and lines with only a // comment or a /* ... */ block do
not count. The script prints one line for each module over N code lines and exits with 1 if there is
any such module. A module over M lines in total only gets a warning: the exit code does not change.
"""

from __future__ import annotations

import argparse
import ast
import io
import re
import sys
import tokenize
from pathlib import Path

SUFFIXES = {".py", ".c", ".h"}
C_SUFFIXES = {".c", ".h"}


def modules(paths: list[str]) -> list[Path]:
    """Return the .py, .c and .h files under each path, sorted, without those in __pycache__."""
    found: list[Path] = []
    for path in map(Path, paths):
        if path.is_file():
            found.append(path)
        else:
            found.extend(
                p for p in path.rglob("*") if p.suffix in SUFFIXES and "__pycache__" not in p.parts
            )
    return sorted(found)


def docstring(node: ast.AST) -> ast.stmt | None:
    """Return the statement that is the docstring of a module, class or function, if any."""
    body = getattr(node, "body", None)
    if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
        return None
    first = body[0] if body else None
    if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
        return first if isinstance(first.value.value, str) else None
    return None


def docstring_lines(tree: ast.Module) -> set[int]:
    """Return the line numbers of every docstring in the module."""
    found = [doc for doc in map(docstring, ast.walk(tree)) if doc is not None]
    return {n for doc in found for n in range(doc.lineno, (doc.end_lineno or doc.lineno) + 1)}


def comment_only_lines(source: str) -> set[int]:
    """Return the line numbers that hold a comment and nothing else."""
    lines = source.splitlines()
    found: set[int] = set()
    for token in tokenize.generate_tokens(io.StringIO(source).readline):
        if (
            token.type == tokenize.COMMENT
            and not lines[token.start[0] - 1][: token.start[1]].strip()
        ):
            found.add(token.start[0])
    return found


def code_lines(source: str) -> int:
    """Count the lines of code in a module's source."""
    skipped = docstring_lines(ast.parse(source)) | comment_only_lines(source)
    return sum(
        1
        for number, line in enumerate(source.splitlines(), 1)
        if line.strip() and number not in skipped
    )


C_TOKEN = re.compile(
    r"""("(?:\\.|[^"\\\n])*"|'(?:\\.|[^'\\\n])*')|//(?:\\\n|[^\n])*|/\*.*?\*/""", re.DOTALL
)


def blank_comment(match: re.Match[str]) -> str:
    """Keep a string or character literal; replace a comment by its newlines."""
    return match.group(1) or "\n" * match.group(0).count("\n")


def c_code_lines(source: str) -> int:
    """Count the lines of C code: lines left with text once comments are removed."""
    return sum(1 for line in C_TOKEN.sub(blank_comment, source).splitlines() if line.strip())


def report(module: Path, max_code_lines: int, target_lines: int) -> bool:
    """Print what is wrong with one module; return True if it is over the code-line limit."""
    source = module.read_text(encoding="utf-8")
    count = c_code_lines(source) if module.suffix in C_SUFFIXES else code_lines(source)
    total = len(source.splitlines())
    if total > target_lines:
        print(f"{module}: {total} lines (target {target_lines}), warning only")
    if count > max_code_lines:
        print(f"{module}: {count} code lines (limit {max_code_lines})")
    return count > max_code_lines


def main(argv: list[str] | None = None) -> int:
    """Check every module and return the exit code."""
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("--max-code-lines", type=int, required=True)
    parser.add_argument("--target-lines", type=int, required=True)
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args(argv)
    too_long = sum(
        report(module, args.max_code_lines, args.target_lines) for module in modules(args.paths)
    )
    return 1 if too_long else 0


if __name__ == "__main__":
    sys.exit(main())
