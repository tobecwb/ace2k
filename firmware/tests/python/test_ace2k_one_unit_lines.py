"""The klippy.log lines a single unit writes, pinned to their exact text: the form the original
single-unit module wrote, whether the unit is `[ace2k]` or the only one named (`[ace2k a]`)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from test_ace2k_dryer import DRYER_RESPONSES, RELEASE, state_frame
from test_ace2k_extras import RESPONSES, SYNTHETIC_UID, make, make_units
from test_ace2k_feed import event_params
from test_ace2k_rfid import UID7, Tag, steps_sent, tag_event
from test_ace2k_tags import anycubic_image, ntag_image, sections

if TYPE_CHECKING:
    from ace2k import Ace2k
    from test_ace2k_extras import FakeMcu, FakePrinter, Responses


def build(
    named: bool, responses: Responses | None = None, known: set[str] | None = None
) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    """(module, printer, mcu) of one unit: [ace2k], or [ace2k a] alone."""
    if not named:
        return make(dict(responses or RESPONSES), known=known)
    (unit,), printer, mcus = make_units(["a"], responses=responses)
    mcus["a"].known = known
    return unit, printer, mcus["a"]


def lines(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [r.getMessage() for r in caplog.records]


@pytest.fixture(params=[False, True], ids=["unnamed", "named"])
def named(request: pytest.FixtureRequest) -> bool:
    return request.param


def test_the_connect_and_unit_lines(named: bool, caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = build(named)
    with caplog.at_level("INFO", logger="root"):
        printer.handlers["klippy:mcu_identify"]()
    got = lines(caplog)
    assert "ace2k 0.1.0 connected, link proven: True" in got
    assert f"ace2k unit {SYNTHETIC_UID.hex()}" in got


def test_the_sensors_event_line(named: bool, caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = build(named)
    mcu.config_callback()
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_sensors_event"]({"lane": 1, "kind": 3, "level": 1})
        mcu.subscriptions["ace2k_sensors_event"]({"lane": 255, "kind": 4, "level": 0})
    assert lines(caplog) == [
        "ace2k sensors event: lane=2 kind=pulled level=1",
        "ace2k sensors event: lane=- kind=cutout level=0",
    ]


def test_the_feed_event_line(named: bool, caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = build(named)
    mcu.config_callback()
    module.feed._seed([0] * 4)
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_feed_event"](event_params(2, 7, 1, 120000, 15000))
    assert "ace2k feed: lane 3 stuck (feeding) — motor 120.0 mm, filament 15.0 mm" in lines(caplog)


def test_the_rfid_tag_line(named: bool, caplog: pytest.LogCaptureFixture) -> None:
    responses = dict(RESPONSES)
    responses.setdefault("ace2k_rfid_read", {"lane": 0, "accepted": 1, "reason": 0})
    module, printer, mcu = build(named, responses)
    for name, params in sections().items():
        printer.add_object(f"ace2k_tag {name}", Tag(name, params))
    mcu.config_callback()
    module.rfid._on_connect()
    tag_event(mcu, 1, 7, UID7, 0x0044, 0x00)
    pages = ntag_image(anycubic_image())
    with caplog.at_level("INFO", logger="root"):
        for first in (4, 16, 28):
            for group in range(3):
                p = first + 4 * group
                payload = b"".join(pages.get(p + i, bytes(4)) for i in range(4))
                mcu.subscriptions["ace2k_rfid_data"](
                    {"lane": 1, "session": 7, "block": p, "status": 0, "data": payload}
                )
    assert steps_sent(mcu)
    (tag,) = [m for m in lines(caplog) if " tag " in m and "—" in m]
    assert tag.startswith(f"ace2k: lane 2 tag {UID7.hex().upper()} — ")


def test_the_dryer_lowered_line(named: bool, caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = build(named, DRYER_RESPONSES, RELEASE)
    mcu.config_callback()
    mcu.subscriptions["ace2k_dryer_state"](state_frame(oldest=0))
    with caplog.at_level("INFO", logger="root"):
        mcu.subscriptions["ace2k_dryer_event"]({"kind": 3, "arg": 2, "seq": 0})
    assert (
        "ace2k dryer: a filament was inserted on lane 2 with no host:"
        " the target is lowered to 45 °C" in lines(caplog)
    )
