"""Put klippy/extras on the path so the host modules import without a Klipper checkout, and
stand in for klippy's msgproto: the extras import it for the message parser's error class (what
mcu.lookup_command raises for a format the dictionary lacks or carries differently), and the fake
MCU raises the same class."""

import sys
import types
from pathlib import Path

EXTRAS = Path(__file__).resolve().parents[3] / "klippy" / "extras"
if str(EXTRAS) not in sys.path:
    sys.path.insert(0, str(EXTRAS))

if "msgproto" not in sys.modules:
    msgproto = types.ModuleType("msgproto")

    class error(Exception):  # noqa: N801, N818 — klippy's own name for it
        """msgproto.error as klippy declares it."""

    msgproto.error = error
    sys.modules["msgproto"] = msgproto
