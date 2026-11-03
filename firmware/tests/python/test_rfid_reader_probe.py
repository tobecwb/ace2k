"""A probe of a reader never soft-resets a reader that the watch uses.

A probe is a soft reset of the reader, then a read of its version register. The reset clears the
reader's settings and switches its field off. So a probe of a reader whose field is on, or that
runs an exchange with a tag, would cut a read in the middle. The watch says when a reader is in
use (`ace2k_rfid_binding_reader_busy`). Two paths probe a reader: the health self-test and the
`ace2k_rfid_reader_query` command. Both go through `ace2k_rfid_reader_binding_probe`, and that one
function leaves a busy reader alone. The query then answers the version read at the last soft
reset of that reader.

The bindings are not compiled on the host, so these tests read the source text. The watch's own
configure also resets a reader, through the core's `ace2k_rfid_reader_probe`. It runs from the
watch's step, before the field is switched on, so it is not a probe of a reader in use. The core
tests cover it.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

FW = Path(__file__).resolve().parents[2]
SRC = FW / "src"
READER_CMDS = SRC / "ace2k" / "rfid" / "reader_cmds.c"

BINDING_PROBE = "ace2k_rfid_reader_binding_probe"
CORE_PROBE = "ace2k_rfid_reader_probe"
BUSY = "ace2k_rfid_binding_reader_busy"
QUERY = "ace2k_rfid_reader_cmd_query"


def body(text: str, name: str) -> str:
    """Return the body of the function `name` defined in `text`, braces included.

    Fail the test when the file defines no such function.
    """
    m = re.search(rf"^[A-Za-z_][\w \*]*\b{name}\s*\([^)]*\)\s*\{{", text, re.M)
    if m is None:
        pytest.fail(f"no definition of {name}()")
    start = m.end() - 1
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start : i + 1]
    pytest.fail(f"{name}() has no closing brace")


def calls(text: str, name: str) -> list[int]:
    """Return the offsets of every call of `name` in `text`."""
    return [m.start() for m in re.finditer(rf"\b{name}\s*\(", text)]


def binding_files() -> list[Path]:
    """Return the binding and board sources: every file that may talk to Klipper."""
    return sorted(SRC.glob("ace2k/*/*_cmds.c")) + sorted(SRC.glob("ace2k_board/*.c"))


# The whole guard, as written: the same reader, the condition not inverted nor cut short, the
# refusal, then the probe. The order checks below give clearer messages; this one pins the text.
GUARD = re.compile(
    r"#if CONFIG_ACE2K_RFID_READ\s*\n\s*if \(ace2k_rfid_binding_reader_busy\(reader\)\) \{"
    r"\s*return -ACE2K_EBUSY;\s*\}\s*#endif\s*\n\s*return ace2k_rfid_reader_probe\("
    r"&ace2k_rfid_reader_instance, reader\);"
)


def test_the_binding_probe_leaves_a_reader_the_watch_uses_alone() -> None:
    b = body(READER_CMDS.read_text(), BINDING_PROBE)
    assert GUARD.search(b), f"{BINDING_PROBE}(): the busy guard is not the one this test expects"
    busy = calls(b, BUSY)
    probe = calls(b, CORE_PROBE)
    assert busy, f"{BINDING_PROBE}() does not ask the watch whether the reader is busy"
    assert len(probe) == 1, f"{BINDING_PROBE}() calls {CORE_PROBE}() {len(probe)} times"
    assert busy[0] < probe[0], f"{BINDING_PROBE}() probes before it asks whether it may"
    between = b[busy[0] : probe[0]]
    assert re.search(r"\breturn\s+-ACE2K_EBUSY\s*;", between), (
        f"{BINDING_PROBE}() does not return -ACE2K_EBUSY on a busy reader"
    )
    # Without the watch, nothing uses a reader, and the busy check is not built.
    guard = b.rfind("#if CONFIG_ACE2K_RFID_READ", 0, busy[0])
    assert guard != -1, f"{BUSY}() outside #if CONFIG_ACE2K_RFID_READ"
    assert "#endif" in b[busy[0] : probe[0]], "the guard does not end before the probe"


def test_no_binding_resets_a_reader_but_through_the_binding_probe() -> None:
    reader_cmds = READER_CMDS.read_text()
    allowed = body(reader_cmds, BINDING_PROBE)
    offset = reader_cmds.index(allowed)
    for path in binding_files():
        text = path.read_text()
        for at in calls(text, CORE_PROBE) + calls(text, "ace2k_rfid_reader_soft_reset"):
            inside = path == READER_CMDS and offset <= at < offset + len(allowed)
            line = text.count("\n", 0, at) + 1
            assert inside, (
                f"{path.relative_to(FW)}:{line} resets a reader outside {BINDING_PROBE}()"
            )


@pytest.mark.parametrize(
    ("path", "caller"),
    [
        (READER_CMDS, QUERY),
        (SRC / "ace2k" / "core" / "health_cmds.c", "ace2k_health_binding_task"),
    ],
)
def test_each_probing_path_goes_through_the_binding_probe(path: Path, caller: str) -> None:
    b = body(path.read_text(), caller)
    assert calls(b, BINDING_PROBE), f"{caller}() does not probe through {BINDING_PROBE}()"
    assert not calls(b, CORE_PROBE), f"{caller}() calls {CORE_PROBE}() itself"


def test_the_query_answers_the_version_it_holds_after_the_probe() -> None:
    b = body(READER_CMDS.read_text(), QUERY)
    probe = calls(b, BINDING_PROBE)
    send = calls(b, "sendf")
    assert len(probe) == 1 and len(send) == 1, f"{QUERY}(): one probe and one answer"
    assert probe[0] < send[0], f"{QUERY}() answers before it probes"
    answer = b[send[0] :]
    assert re.search(r"\.version\[reader\]", answer), f"{QUERY}() does not answer the version"
    # The answer is sent whatever the probe returned: a busy reader still gets its answer.
    assert not re.search(r"\b(if|return)\b", b[probe[0] : send[0]]), (
        f"{QUERY}() can skip its answer after the probe"
    )
