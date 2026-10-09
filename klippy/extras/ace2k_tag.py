"""ace2k_tag — one [ace2k_tag <brand>] section per spool brand: `enabled`, `source` and the
brand's own parameters (salts, keys, tables), read as strings for the brand module to parse.
Klipper loads it for every such section; ace2k_rfid.py collects them.  The shipped sections are
config/ace2k_tags.cfg (`[include ace2k_tags.cfg]` in printer.cfg)."""


class Ace2kTag:
    def __init__(self, config):
        self.brand = config.get_name().split()[-1]
        self.enabled = config.getboolean("enabled", True)
        self.params = {}
        for option in config.get_prefix_options(""):
            if option == "enabled":
                continue
            self.params[option] = config.get(option)  # read: marks it used for Klipper


def load_config_prefix(config):
    return Ace2kTag(config)
