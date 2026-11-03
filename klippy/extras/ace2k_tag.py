"""Holds the settings of one `[ace2k_tag <brand>]` section, one section per spool brand.

A section has `enabled`, `source` and the parameters of the brand (salts, keys, tables).
They are kept as strings, and the brand module parses them.
Klipper loads this module for every such section, and `ace2k_rfid.py` collects the results.
The shipped sections are in `config/ace2k_tags.cfg`, which `printer.cfg` includes with
`[include ace2k_tags.cfg]`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .ace2k_klipper_types import Config


class Ace2kTag:
    def __init__(self, config: Config) -> None:
        self.brand = config.get_name().split()[-1]
        self.enabled = config.getboolean("enabled", True)
        self.params: dict[str, str] = {}
        for option in config.get_prefix_options(""):
            if option == "enabled":
                continue
            self.params[option] = config.get(option)  # read: marks it used for Klipper


def load_config_prefix(config: Config) -> Ace2kTag:
    return Ace2kTag(config)
