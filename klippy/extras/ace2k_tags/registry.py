"""The brands that this package knows, and the list of brands that a session tries.

The list is built from the [ace2k_tag] sections. It holds the enabled brands whose section has
every parameter they need. The NTAG brands come first, then the MIFARE brands in the configured
order.
"""

from __future__ import annotations

import logging
from collections.abc import Mapping, Sequence

from . import anycubic, bambu, creality, elegoo, snapmaker
from .base import TagFormat, TagParams

MODULES = (anycubic, elegoo, bambu, snapmaker, creality)
BRANDS: dict[str, type[TagFormat]] = {m.FORMAT.name: m.FORMAT for m in MODULES}
DEFAULT_MIFARE_ORDER = ("bambu", "snapmaker", "creality")


def register(fmt: type[TagFormat]) -> None:
    """Add a brand from outside the package, such as a module of the user or a test."""
    BRANDS[fmt.name] = fmt


def _enabled(params: TagParams) -> bool:
    return str(params.get("enabled", "True")).strip().lower() in ("1", "true", "yes")


def _usable(name: str, sections: Mapping[str, TagParams]) -> TagFormat | None:
    params = sections.get(name)
    if params is None or not _enabled(params):
        return None
    missing = [k for k in BRANDS[name].needs if k not in params]
    if missing:
        logging.warning("ace2k_tag %s: missing %s; the brand is disabled", name, ", ".join(missing))
        return None
    return BRANDS[name](params)


def build(
    sections: Mapping[str, TagParams], mifare_order: Sequence[str] = DEFAULT_MIFARE_ORDER
) -> list[TagFormat]:
    """Return the formats to try, in order.

    ``sections`` maps a brand to its options. ``mifare_order`` lists the brand names in the order
    to try the MIFARE brands. Raises ValueError if it names a brand that does not exist.
    """
    unknown = [n for n in mifare_order if n not in BRANDS]
    if unknown:
        raise ValueError(f"rfid_mifare_order names no brand: {', '.join(unknown)}")
    ntag = [n for n, f in BRANDS.items() if f.chip == "ntag"]
    mifare = list(mifare_order) + [
        n for n, f in BRANDS.items() if f.chip == "mifare" and n not in mifare_order
    ]
    out: list[TagFormat] = []
    for name in ntag + mifare:
        fmt = _usable(name, sections)
        if fmt is not None:
            out.append(fmt)
    return out
