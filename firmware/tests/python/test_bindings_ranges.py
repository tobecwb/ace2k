"""Every command handler checks a host argument against its range before it narrows it.

Klipper's parser keeps all 32 bits of every integer argument, even for `%c` and `%hu`. So a
handler that casts `args[n]` to a narrower type before it checks the value lets an out-of-range
value wrap into a valid one: `lane=256` would become lane 0. These tests read every command
handler under src/ and fail on a narrowing that no check comes before. The handlers are not
compiled on the host, so the tests read the source text.

An argument is `args[n]` (by the handler's own parameter name), or a 32-bit local copied whole
from it (`uint32_t index = args[0];`). These uses count as a narrowing:

- a cast to a narrower type or to an enum: `(uint8_t)args[0]`, `(enum ace2k_flap)flap`;
- a narrower variable set from it: `uint8_t lane = args[0];`, or `lane = args[0];` when `lane` is
  declared narrower in the same handler. A comparison stored in it is no narrowing:
  `bool big = args[0] > 3U;` compares the whole value;
- a read as a truth value: `args[1] != 0U`, `args[1] ? a : b`, `!args[1]`, `!!args[1]`, the whole
  condition of an `if` or a `while` (`if (args[1])`), or an operand of `&&` or `||`
  (`args[1] && ready`). Redundant parentheses around the argument are skipped:
  `if ((args[1]))` is `if (args[1])`;
- the whole argument passed to a parameter narrower than 32 bits. The parameter types come from
  the headers under src/, Klipper's command.h and the file's own functions;
- the whole argument put in a struct field narrower than 32 bits (`.duty_pct = args[2]`). A
  member assignment outside an initializer (`c.small = args[0];`, `p->small = args[0];`) fails as
  an unknown field.

A narrowing is checked when an `ACE2K_ARG_*` helper of ace2k_board/args.h or lane/lane_args.h
takes the argument, or when the same handler compares the same argument with `<`, `<=`, `>` or
`>=` before it. A helper gives its value in its own type (`uint8_t`, `uint16_t` or `bool`), so
the place its result goes is read in the same way: a cast, a variable, a parameter or a field
narrower than the helper's type fails (`uint8_t lane = ACE2K_ARG_U16(...)`). An enum counts as a
byte, so `(enum ace2k_flap)ACE2K_ARG_U16(...)` fails as `(enum ace2k_flap)args[0]` does. A
callee, a field, a variable or a helper that the tests cannot find fails the test too.

Not covered. These forms pass, so a reviewer must still read each handler:

- a comparison counts as a check whatever its bound and whatever follows it:
  `if (args[0] > 1000U) { ... } take_u8(args[0]);` passes. The check must be a comparison with
  the right bound, followed by a shutdown or a refusal;
- arithmetic on an argument or on a helper's result is not read as a narrowing, so a narrowing
  of its result is missed: `(uint8_t)(args[0] + 0U)`, `uint8_t x = 1U + ACE2K_ARG_U16(...)`.
  Arithmetic alone (`args[0] / timer_from_us(1000U)`) is fine while its result keeps 32 bits;
- a chained copy: `uint32_t a = args[0]; uint32_t b = a; take_u8(b);`. Only a 32-bit local set
  straight from `args[n]` is followed;
- a truth read in a `for` condition, or in a condition that is neither an `if`, a `while`, `!`,
  `?:`, `&&` nor `||`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

import pytest

FW = Path(__file__).resolve().parents[2]
SRC = FW / "src"
KLIPPER_COMMAND_H = FW / "klipper" / "src" / "command.h"

NARROW_TYPES = ("uint8_t", "uint16_t", "int8_t", "int16_t", "bool", "char")
# The largest value of each type narrower than 32 bits; any other type holds all 32 bits.
TYPE_MAX = {
    "bool": 1,
    "char": 127,
    "int8_t": 127,
    "uint8_t": 255,
    "unsigned char": 255,
    "int16_t": 32767,
    "uint16_t": 65535,
}
WIDE_MAX = 2**32 - 1
# The type in which each ACE2K_ARG_* helper gives its value.
HELPER_TYPES = {
    "ACE2K_ARG_U8": "uint8_t",
    "ACE2K_ARG_U8_MAX": "uint8_t",
    "ACE2K_ARG_U16": "uint16_t",
    "ACE2K_ARG_FLAG": "bool",
    "ACE2K_ARG_LANE": "uint8_t",
    "ACE2K_ARG_LANE_OR_ALL": "uint8_t",
}
NARROW = "|".join(NARROW_TYPES)
KEYWORDS = {"if", "while", "for", "switch", "return", "sizeof", "else", "case", "do"}

COMMENT_OR_STRING = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"', re.S)
HANDLER = re.compile(r"\bvoid\s+(\w+)\s*\(\s*uint32_t\s*\*\s*(\w+)\s*\)\s*\{")
DECL_COMMAND = re.compile(r"\bDECL_COMMAND(?:_FLAGS)?\(\s*(\w+)\s*,")
CAST = re.compile(rf"\(\s*(?:const\s+)?(?:{NARROW}|unsigned\s+char|enum\s+\w+)\s*\)\s*$")
DECLARED = re.compile(rf"\b(?:{NARROW})\s+\w+\s*=\s*$")
NARROW_LOCAL = re.compile(rf"\b(?:{NARROW})\s+(\w+)\s*[;=]")
ASSIGNED = re.compile(r"(?<![=!<>.])(?<!->)\b(\w+)\s*=\s*$")
FIELD = re.compile(r"(?:\.|->)(\w+)\s*=\s*$")
INITIALIZER = re.compile(r"\bstruct\s+(\w+)\s+\w+\s*=\s*\{")
TRUTH_AFTER = re.compile(r"^\s*(?:!=\s*0[uU]?\b|\?)")
TRUTH_BEFORE = re.compile(r"(?<![!=])!+\s*$")
GROUPED = re.compile(r"(?:^|[^\w\]\s])\s*\(\s*$")
CONDITION_START = re.compile(r"(?:\b(?:if|while)\s*\(|&&|\|\|)\s*$")
OPERAND_END = re.compile(r"^\s*(?:\)|&&|\|\||\?)")
LOGIC_AFTER = re.compile(r"^\s*(?:&&|\|\|)")
BINARY_BEFORE = re.compile(r"(?:[<>=!]=|<<|>>|[<>+\-*/%^~]|(?<!&)&|(?<!\|)\|)\s*$")
COMPARED_AFTER = re.compile(r"^\s*(?:<=|>=|==|<(?!<)|>(?!>)|!=(?!\s*0[uU]?\b))")
HELPER_NAME = re.compile(r"\b(ACE2K_ARG_\w+)\(\s*$")
TYPED_CAST = re.compile(
    r"\(\s*(?:const\s+)?(unsigned\s+char|enum\s+\w+|\w+_t|bool|char|int|unsigned)\s*\)\s*$"
)
TYPED_DECLARED = re.compile(r"\b((?:unsigned\s+)?\w+)\s+\w+\s*=\s*$")
STRUCT = re.compile(r"\bstruct\s+(\w+)\s*\{([^{}]*)\}\s*;")
FIELD_DECL = re.compile(r"([\w\s]+?)\b(\w+)\s*(?:\[[^\]]*\])?\s*;")
PROTOTYPE = re.compile(
    r"(?:^|[;}\n])[ \t]*((?:(?:static|inline|const|struct|enum|unsigned)\s+)*\w+[\s\*]+)"
    r"(\w+)\s*\(([^;{}()]*)\)\s*[;{]"
)
PARAM = re.compile(r"^(?:[\w\s]+?[\s\*]+\w+(?:\s*\[[^\]]*\])?|void|\.\.\.)$")


@dataclass(frozen=True)
class Use:
    """A narrowing of an argument that no check comes before."""

    file: str
    handler: str
    expr: str
    how: str

    def __str__(self) -> str:
        return f"{self.file}: {self.handler}: {self.expr} ({self.how})"


@dataclass
class Scope:
    """What a handler can call and initialize: functions with their parameter types, and
    structs with their field types."""

    protos: dict[str, list[str]] = field(default_factory=dict)
    structs: dict[str, dict[str, str]] = field(default_factory=dict)

    def add(self, text: str) -> None:
        """Add the functions and structs that the C text declares."""
        self.protos.update(prototypes(text))
        self.structs.update(struct_fields(text))


def strip(text: str) -> str:
    """The C text with comments removed and every string literal emptied."""
    return COMMENT_OR_STRING.sub(lambda m: '""' if m.group(0).startswith('"') else " ", text)


def body_end(text: str, open_brace: int, pair: str = "{}") -> int:
    """The index just past the brace that closes the one at open_brace.

    pair names the opening and the closing character: "()" matches a parenthesis."""
    depth = 0
    for i in range(open_brace, len(text)):
        if text[i] == pair[0]:
            depth += 1
        elif text[i] == pair[1]:
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError("unbalanced braces")


def param_type(param: str) -> str:
    """The type of one C parameter declaration, without its name."""
    param = " ".join(re.sub(r"\[[^\]]*\]", "", param).split())
    if param in {"void", "..."}:
        return param
    return re.sub(r"\w+$", "", param).strip()


def prototypes(text: str) -> dict[str, list[str]]:
    """Each function the C text declares or defines, with its parameter types in order.

    A call such as `f(&x, 3);` looks like a prototype to the pattern. Its arguments are not
    declarations, so it is left out."""
    found: dict[str, list[str]] = {}
    for ret, name, params in PROTOTYPE.findall(strip(text)):
        if name in KEYWORDS or ret.split()[0] in KEYWORDS:
            continue
        parts = [" ".join(p.split()) for p in params.split(",")]
        if all(PARAM.match(p) for p in parts):
            found[name] = [param_type(p) for p in parts]
    return found


def struct_fields(text: str) -> dict[str, dict[str, str]]:
    """Each struct the C text defines, with its fields' types."""
    found: dict[str, dict[str, str]] = {}
    for name, body in STRUCT.findall(strip(text)):
        found[name] = {fname: " ".join(ftype.split()) for ftype, fname in FIELD_DECL.findall(body)}
    return found


@cache
def header_text() -> str:
    """Every header under src/, and Klipper's command.h, as one text."""
    paths = [*sorted(SRC.glob("ace2k/*/*.h")), *sorted(SRC.glob("ace2k_board/*.h"))]
    return "\n".join(p.read_text() for p in [*paths, KLIPPER_COMMAND_H])


def is_narrow(ctype: str) -> bool:
    """True for a type narrower than 32 bits, or an enum."""
    if "*" in ctype:
        return False
    words = ctype.replace("const", " ").split()
    return bool(words) and (
        words[-1] in NARROW_TYPES or words[0] == "enum" or words == ["unsigned", "char"]
    )


def call_context(body: str, pos: int, end: int) -> tuple[str, int] | None:
    """The callee and the parameter index when the use at pos..end is a whole call argument."""
    depth = 0
    commas = 0
    i = pos - 1
    while i >= 0:
        c = body[i]
        if c in ")]":
            depth += 1
        elif c == "[" or (c == "(" and depth):
            depth -= 1
        elif c == "(":
            break
        elif c == "," and depth == 0:
            commas += 1
        elif c in ";{}" and depth == 0:
            return None
        i -= 1
    if i < 0:
        return None
    before = body[i + 1 : pos].strip()
    if before and not before.endswith(","):
        return None
    if not body[end:].lstrip().startswith((",", ")")):
        return None
    m = re.search(r"(\w+)\s*$", body[:i])
    if m is None or m.group(1) in KEYWORDS:
        return None
    return m.group(1), commas


def enclosing_struct(body: str, pos: int) -> str | None:
    """The struct type of the initializer that the position lies in, if any."""
    last = None
    for m in INITIALIZER.finditer(body, 0, pos):
        last = m
    if last is None or "}" in body[last.end() : pos]:
        return None
    return last.group(1)


def truth_read(before: str, after: str) -> bool:
    """True when the use between before and after is read as a truth value."""
    if TRUTH_AFTER.match(after) or (TRUTH_BEFORE.search(before) and not after.startswith("=")):
        return True
    if CONDITION_START.search(before) and OPERAND_END.match(after):
        return True  # if (x), while (x), a && x), a || x)
    return bool(LOGIC_AFTER.match(after)) and not BINARY_BEFORE.search(before)


def local_narrowing(body: str, pos: int, end: int) -> str | None:
    """How the text around the use at pos..end narrows it: a cast, a narrower variable or a
    truth value. None when it does none of these."""
    before = body[:pos]
    after = body[end:]
    if CAST.search(before):
        return "cast"
    m = ASSIGNED.search(before)
    narrower = DECLARED.search(before) or (m and m.group(1) in set(NARROW_LOCAL.findall(body)))
    if narrower and not COMPARED_AFTER.match(after):
        return "narrower variable"
    if truth_read(before, after):
        return "truth value"
    return None


def field_narrowing(body: str, pos: int, scope: Scope) -> str | None:
    """How a struct field that the use at pos initializes narrows it, or None when it does not.

    An empty string when the use initializes no field."""
    m = FIELD.search(body[:pos])
    if m is None:
        return ""
    ftype = scope.structs.get(enclosing_struct(body, pos) or "", {}).get(m.group(1))
    if ftype is None:
        return f"unknown field .{m.group(1)}"
    return "narrower field" if is_narrow(ftype) else None


def call_narrowing(body: str, pos: int, end: int, scope: Scope) -> str | None:
    """How the parameter that takes the use at pos..end narrows it, or None when it does not."""
    call = call_context(body, pos, end)
    if call is None:
        return None
    callee, index = call
    params = scope.protos.get(callee)
    if params is None:
        return f"unknown callee {callee}()"
    if index >= len(params) or params[index] == "...":
        return f"variadic callee {callee}()"
    return "narrower parameter" if is_narrow(params[index]) else None


def unwrap(body: str, pos: int, end: int) -> tuple[int, int]:
    """The use at pos..end with its redundant parentheses: `((args[0]))` is read as `args[0]`.

    A parenthesis that follows a name or a `]` opens a call or a condition, so it stays."""
    while GROUPED.search(body[:pos]) and body[end:].lstrip().startswith(")"):
        pos = body.rindex("(", 0, pos)
        end = body.index(")", end) + 1
    return pos, end


def narrowing(body: str, pos: int, end: int, scope: Scope) -> str | None:
    """How the argument use at pos..end is narrowed, or None when it keeps its 32 bits."""
    pos, end = unwrap(body, pos, end)
    how = local_narrowing(body, pos, end)
    if how is not None:
        return how
    how = field_narrowing(body, pos, scope)
    if how != "":
        return how
    return call_narrowing(body, pos, end, scope)


def type_max(ctype: str) -> int:
    """The largest value the C type holds. An enum counts as a byte: every enum a command takes
    has its values below 256, so a 16-bit value cast to one is a narrowing."""
    words = ctype.replace("const", " ").split()
    if words and words[0] == "enum":
        return TYPE_MAX["uint8_t"]
    return TYPE_MAX.get(" ".join(words), WIDE_MAX)


def local_type(body: str, name: str) -> str | None:
    """The type the handler declares a local variable with, or None when it declares none."""
    m = re.search(rf"\b((?:unsigned\s+)?\w+)\s+{name}\s*[;=,]", body)
    return None if m is None else " ".join(m.group(1).split())


def helper_place(body: str, start: int, scope: Scope) -> tuple[str | None, str] | None:
    """The type of the cast, the variable or the field that takes the helper call at start,
    and what it is. None when the call is in none of these places."""
    before = body[:start]
    m = TYPED_CAST.search(before)
    if m:
        return m.group(1), "cast"
    m = TYPED_DECLARED.search(before)
    if m and m.group(1) not in KEYWORDS:
        return m.group(1), "variable"
    m = FIELD.search(before)
    if m:
        ftype = scope.structs.get(enclosing_struct(body, start) or "", {}).get(m.group(1))
        return ftype, "field" if ftype else f"unknown field .{m.group(1)}"
    m = ASSIGNED.search(before)
    if m:
        ctype = local_type(body, m.group(1))
        return ctype, "variable" if ctype else f"unknown variable {m.group(1)}"
    return None


def helper_target(body: str, start: int, end: int, scope: Scope) -> tuple[str | None, str]:
    """The type that the helper call at start..end is put in, and what holds it.

    The type is None when the result keeps its own type, or when the tests cannot find the
    callee, the field or the variable; then what names the unknown, or is empty."""
    place = helper_place(body, start, scope)
    if place is not None:
        return place
    call = call_context(body, start, end)
    if call is None:
        return None, ""
    params = scope.protos.get(call[0])
    if params is None:
        return None, f"unknown callee {call[0]}()"
    if call[1] >= len(params) or params[call[1]] == "...":
        return None, f"variadic callee {call[0]}()"
    return params[call[1]], "parameter"


def helper_narrowing(body: str, open_paren: int, name: str, scope: Scope) -> str | None:
    """How the place that the result of the helper call goes narrows it, or None.

    open_paren is the index of the call's opening parenthesis, name the helper's name."""
    start = open_paren - len(name)
    end = body_end(body, open_paren, "()")
    if name not in HELPER_TYPES:
        return f"unknown helper {name}"
    ctype, what = helper_target(body, start, end, scope)
    if ctype is None:
        return what or None
    if type_max(ctype) < type_max(HELPER_TYPES[name]):
        return f"{name} into a narrower {what}"
    return None


def checked_before(body: str, pos: int, expr_re: str) -> bool:
    """True when the handler compares the argument with a bound before pos."""
    left = rf"(?:{expr_re})\s*(?:<=|>=|<(?![<=])|(?<!-)>(?![>=]))"
    right = rf"(?:<=|>=|(?<!<)<|(?<![->])>)\s*(?:{expr_re})"
    return re.search(f"{left}|{right}", body[:pos]) is not None


def handler_uses(name: str, body: str, param: str, scope: Scope, file: str) -> list[Use]:
    """The unchecked narrowings of the arguments in one handler body."""
    arg_re = rf"\b{param}\s*\[([^\]]*)\]"
    aliases = re.findall(rf"\b(?:uint32_t|int32_t)\s+(\w+)\s*=\s*{param}\s*\[[^\]]*\]\s*;", body)
    found: list[Use] = []
    for pattern in [arg_re, *(rf"\b{a}\b" for a in aliases)]:
        for m in re.finditer(pattern, body):
            if pattern != arg_re and re.match(rf"\s*=\s*{param}\s*\[", body[m.end() :]):
                continue  # the alias's own declaration
            helper = HELPER_NAME.search(body[: m.start()])
            if helper:
                how = helper_narrowing(body, helper.end(1), helper.group(1), scope)
                if how is not None:
                    found.append(Use(file, name, " ".join(m.group(0).split()), how))
                continue
            how = narrowing(body, m.start(), m.end(), scope)
            if how is None:
                continue
            if pattern == arg_re:
                expr_re = rf"\b{param}\s*\[\s*{re.escape(m.group(1).strip())}\s*\]"
            else:
                expr_re = pattern
            if not checked_before(body, m.start(), expr_re):
                found.append(Use(file, name, " ".join(m.group(0).split()), how))
    return found


def file_uses(path: Path) -> tuple[list[Use], list[str], list[str]]:
    """The unchecked narrowings in one C file, its handlers, and the commands it declares."""
    raw = path.read_text()
    text = strip(raw)
    scope = Scope()
    scope.add(header_text())
    scope.add(raw)
    rel = str(path.relative_to(FW))
    uses: list[Use] = []
    handlers: list[str] = []
    for m in HANDLER.finditer(text):
        open_brace = m.end() - 1
        body = text[open_brace : body_end(text, open_brace)]
        handlers.append(m.group(1))
        uses += handler_uses(m.group(1), body, m.group(2), scope, rel)
    return uses, handlers, DECL_COMMAND.findall(text)


def binding_files() -> list[Path]:
    """Every C file under src/ace2k and src/ace2k_board that declares a command."""
    paths = [*sorted(SRC.glob("ace2k/*/*.c")), *sorted(SRC.glob("ace2k_board/*.c"))]
    return [p for p in paths if DECL_COMMAND.search(strip(p.read_text()))]


def test_every_declared_command_has_a_handler_these_tests_read() -> None:
    files = binding_files()
    assert files, "no binding declares a command"
    for path in files:
        _, handlers, commands = file_uses(path)
        missing = sorted(set(commands) - set(handlers))
        assert not missing, f"{path.name}: handlers not read: {missing}"


@pytest.mark.parametrize("path", binding_files(), ids=lambda p: p.name)
def test_no_argument_is_narrowed_before_it_is_checked(path: Path) -> None:
    uses, _, _ = file_uses(path)
    assert not uses, "narrowed before a range check:\n" + "\n".join(map(str, uses))


# The tests above only mean something if the reader finds every form. Each snippet below is the
# body of one handler. The reader must flag the bad ones and accept the good ones.
SNIPPET_DECLS = """
void take_u8(uint8_t lane);
void take_u32(uint32_t value);
void take_two(uint32_t a, uint16_t b);
bool ready(void);
uint32_t timer_from_us(uint32_t us);
struct cfg { uint32_t wide; uint8_t small; bool flag; };
"""

BAD = {
    "cast": "uint8_t x = (uint8_t)args[0];",
    "cast u16": "take_u32((uint16_t)args[1]);",
    "cast enum": "take_u32((enum ace2k_flap)args[0]);",
    "cast bool": "take_u32((bool)args[0]);",
    "declared": "uint8_t x = args[0];",
    "assigned": "uint16_t x; x = args[1];",
    "truth !=": "bool on = args[0] != 0U;",
    "truth ?": "take_u32(args[1] ? 1U : 2U);",
    "truth !": "if (!args[2]) { take_u32(0U); }",
    "parameter": "take_u8(args[0]);",
    "second parameter": "take_two(args[0], args[1]);",
    "field": "struct cfg c = { .wide = args[0], .small = args[1] };",
    "field bool": "struct cfg c = { .flag = args[2] != 0 };",
    "alias cast": "uint32_t i = args[0]; take_u32((uint8_t)i);",
    "check after": "uint8_t x = (uint8_t)args[0]; if (args[0] > 3U) { take_u32(0U); }",
    "other argument checked": "if (args[1] > 3U) { take_u32(0U); } take_u8(args[0]);",
    "shift is no check": "take_u32(args[0] << 2); take_u8(args[0]);",
    "unknown callee": "mystery(args[0]);",
    "unknown field": "struct cfg c = { .missing = args[0] };",
    "member assigned": "struct cfg c; c.small = args[0];",
    "member assigned through a pointer": "struct cfg c; struct cfg *p = &c; p->small = args[0];",
    "truth in redundant parentheses": "if ((args[0])) { take_u32(0U); }",
    "truth !!": "take_u32(!!args[0]);",
    "redundant parentheses on a cast": "take_u32((uint8_t)(args[0]));",
    "helper u16 into enum cast": 'take_u32((enum ace2k_flap)ACE2K_ARG_U16(args[0], "m"));',
    "truth if": "if (args[0]) { take_u32(0U); }",
    "truth while": "while (args[0]) { take_u32(0U); }",
    "truth && left": "if (args[0] && ready()) { take_u32(0U); }",
    "truth && right": "if (ready() && args[0]) { take_u32(0U); }",
    "truth || left": "if (args[0] || ready()) { take_u32(0U); }",
    "truth || right": "if (ready() || args[1]) { take_u32(0U); }",
    "truth set in a branch": "bool on = false; if (args[0]) { on = true; } take_u32(on);",
    "helper into narrower variable": 'uint8_t x = ACE2K_ARG_U16(args[0], "m");',
    "helper assigned narrower": 'uint8_t x; x = ACE2K_ARG_U16(args[0], "m");',
    "helper into narrower parameter": 'take_u8(ACE2K_ARG_U16(args[0], "m"));',
    "helper into narrower field": 'struct cfg c = { .small = ACE2K_ARG_U16(args[1], "m") };',
    "helper into narrower cast": 'take_u32((uint8_t)ACE2K_ARG_U16(args[0], "m"));',
    "helper into signed": 'int8_t x = ACE2K_ARG_U8(args[0], "m");',
    "helper byte into bool": 'bool x = ACE2K_ARG_U8(args[0], "m");',
    "unknown helper": 'uint8_t x = ACE2K_ARG_MYSTERY(args[0], "m");',
}

GOOD = {
    "checked cast": "if (args[0] > 3U) { take_u32(0U); } uint8_t x = (uint8_t)args[0];",
    "checked >=": "if (args[0] >= 4U) { take_u32(0U); } take_u8(args[0]);",
    "checked <": "if (args[0] < 4U) { take_u8((uint8_t)args[0]); }",
    "checked alias": "uint32_t i = args[0]; if (i <= 8U) { take_u8((uint8_t)i); }",
    "checked flag": "if (args[2] > 1U) { take_u32(0U); } bool f = args[2] != 0U;",
    "helper": 'uint8_t x = ACE2K_ARG_U8(args[0], "m");',
    "helper lane": 'take_u8(ACE2K_ARG_LANE(args[0], "m"));',
    "wide parameter": "take_u32(args[0]);",
    "wide field": "struct cfg c = { .wide = args[0] };",
    "wide field named as a narrow local": "uint8_t wide = 0U; struct cfg c = { .wide = args[0] };",
    "arithmetic": "uint32_t ms = args[0] / timer_from_us(1000U);",
    "signed same width": "take_u32((int32_t)args[1]);",
    "zero test": "if (args[0] == 0) { take_u32(0U); }",
    "wide alias": "uint32_t i = args[0]; take_u32(i);",
    "comparison into bool": "bool big = args[0] > 3U; take_u32(big);",
    "comparison assigned": "bool big; big = args[0] <= 3U; take_u32(big);",
    "comparison with a value": "bool five = args[0] != 5U; take_u32(five);",
    "comparison in a condition": "if (ready() && args[0] > 3U) { take_u32(0U); }",
    "checked then truth": "if (args[0] > 1U) { take_u32(0U); } if (args[0]) { take_u32(1U); }",
    "helper into same type": 'uint16_t x = ACE2K_ARG_U16(args[0], "m");',
    "helper into wider": 'uint32_t x = ACE2K_ARG_U8(args[0], "m");',
    "helper flag into bool": 'bool x = ACE2K_ARG_FLAG(args[0], "m");',
    "helper into field": 'struct cfg c = { .small = ACE2K_ARG_U8(args[1], "m") };',
    "helper into enum cast": 'take_u32((enum ace2k_flap)ACE2K_ARG_U8_MAX(args[0], 3U, "m"));',
    "parentheses of a call": "take_u32((args[0]));",
    "parentheses around a comparison": "if ((args[0] > 3U)) { take_u32(0U); }",
}


def snippet_uses(code: str) -> list[Use]:
    """The unchecked narrowings the reader finds in a handler whose body is code."""
    text = strip("void handler(uint32_t *args)\n{\n" + code + "\n}\n")
    open_brace = text.index("{")
    scope = Scope()
    scope.add(SNIPPET_DECLS)
    return handler_uses("handler", text[open_brace : body_end(text, open_brace)], "args", scope, "")


@pytest.mark.parametrize("code", BAD.values(), ids=list(BAD))
def test_the_reader_flags_a_narrowing_with_no_check_before_it(code: str) -> None:
    assert snippet_uses(code)


@pytest.mark.parametrize("code", GOOD.values(), ids=list(GOOD))
def test_the_reader_accepts_a_checked_or_wide_use(code: str) -> None:
    uses = snippet_uses(code)
    assert not uses, uses
