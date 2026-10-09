"""The brands this package knows, and the list a session tries, built from the [ace2k_tag]
sections: enabled brands whose section carries every parameter they need, NTAG brands first,
then the MIFARE brands in the configured order."""

import logging

from . import anycubic, bambu, creality, elegoo, snapmaker

MODULES = (anycubic, elegoo, bambu, snapmaker, creality)
BRANDS = {m.FORMAT.name: m.FORMAT for m in MODULES}
DEFAULT_MIFARE_ORDER = ("bambu", "snapmaker", "creality")


def register(fmt):
    """A brand from outside the package (a user's module, a test)."""
    BRANDS[fmt.name] = fmt


def _enabled(params):
    return str(params.get("enabled", "True")).strip().lower() in ("1", "true", "yes")


def _usable(name, sections):
    params = sections.get(name)
    if params is None or not _enabled(params):
        return None
    missing = [k for k in BRANDS[name].needs if k not in params]
    if missing:
        logging.warning("ace2k_tag %s: missing %s; the brand is disabled", name, ", ".join(missing))
        return None
    return BRANDS[name](params)


def build(sections, mifare_order=DEFAULT_MIFARE_ORDER):
    """sections: {brand: {option: value}}; mifare_order: brand names."""
    unknown = [n for n in mifare_order if n not in BRANDS]
    if unknown:
        raise ValueError(f"rfid_mifare_order names no brand: {', '.join(unknown)}")
    ntag = [n for n, f in BRANDS.items() if f.chip == "ntag"]
    mifare = list(mifare_order) + [
        n for n, f in BRANDS.items() if f.chip == "mifare" and n not in mifare_order
    ]
    out = []
    for name in ntag + mifare:
        fmt = _usable(name, sections)
        if fmt is not None:
            out.append(fmt)
    return out
