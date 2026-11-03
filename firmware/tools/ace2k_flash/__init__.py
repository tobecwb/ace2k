"""ace2k_flash: flash an ACE 2 Pro with an ace2k image or the factory image.

The release ships this package as one file, ace2k_flash.pyz.  Run it with
`python3 ace2k_flash.pyz IMAGE.bin`, or from firmware/tools with `python3 -m ace2k_flash`.
flasher.py holds the command line and the flow.  docs/flashing.md is the user guide.

The modules, from the bottom up:

- blink.py: --identify, which blinks the unit on a port
- wire.py: the original Anycubic protocol, the serial link and the update
- image.py: the unit's memory and the images
- klipper.py: Klipper's MCU protocol, to ask a new ace2k image for its version
- host.py: Moonraker, the restore command, Klipper's view of the unit, the port and Klipper's
  service
- base.py: the run's state and the steps every route shares (the waits, Klipper's stop and
  start, the plan, the unit's port)
- route.py: the way into the bootloader's recovery for a unit that runs ace2k
- verify.py: the check after the write
- flasher.py: the command line and the flow

This file names what the tests use from all of them.
"""

from .base import (
    REFUSAL_SETTLE_S,
    RESTORE_WAIT_S,
    STATE_POLL_S,
    UNKNOWN_SETTLE_S,
    Deps,
)
from .blink import blink
from .flasher import Flasher, main
from .host import (
    Moonraker,
    MoonrakerError,
    candidate_ports,
    find_service,
    is_klippy,
    klipper_view,
    port_holders,
)
from .image import (
    APP_BASE,
    APP_END,
    SRAM_BASE,
    SRAM_END,
    TRAILER,
    embedded_dictionary,
    load_image,
)
from .klipper import (
    ACE2K_BAUD,
    KLIPPER_SEQ_MASK,
    KLIPPER_SYNC,
    KlipperUnit,
    klipper_block,
    klipper_crc,
    klipper_vlq,
    parse_klipper_blocks,
    parse_klipper_vlq,
)
from .wire import (
    BAUD,
    CMD_DISCOVER,
    CMD_FLASH_LED,
    CMD_GET_INFO,
    CMD_GET_MOTOR_STATUS,
    CMD_IAP_FINISH,
    CMD_IAP_FIRMWARE,
    CMD_IAP_UPGRADE,
    END,
    EXIT_FAILED,
    EXIT_NO_ANSWER,
    EXIT_NO_BLINK,
    EXIT_OK,
    EXIT_UNCHANGED,
    EXIT_UNVERIFIED,
    IDENTIFY_TRIES,
    PREAMBLE,
    STAGING_BASE,
    Clock,
    Frame,
    LinkError,
    RefusedError,
    SerialLink,
    Unit,
    build_frame,
    calibration_text,
    count_units,
    crc16,
    flash_leds,
    identify,
    parse_calibration,
    parse_frames,
    pb_bytes,
    pb_fields,
    pb_uint,
    read_calibration,
    read_uid,
)

__all__ = [
    "ACE2K_BAUD",
    "APP_BASE",
    "APP_END",
    "BAUD",
    "CMD_DISCOVER",
    "CMD_FLASH_LED",
    "CMD_GET_INFO",
    "CMD_GET_MOTOR_STATUS",
    "CMD_IAP_FINISH",
    "CMD_IAP_FIRMWARE",
    "CMD_IAP_UPGRADE",
    "END",
    "EXIT_FAILED",
    "EXIT_NO_ANSWER",
    "EXIT_NO_BLINK",
    "EXIT_OK",
    "EXIT_UNCHANGED",
    "EXIT_UNVERIFIED",
    "IDENTIFY_TRIES",
    "KLIPPER_SEQ_MASK",
    "KLIPPER_SYNC",
    "PREAMBLE",
    "REFUSAL_SETTLE_S",
    "RESTORE_WAIT_S",
    "SRAM_BASE",
    "SRAM_END",
    "STAGING_BASE",
    "STATE_POLL_S",
    "TRAILER",
    "UNKNOWN_SETTLE_S",
    "Clock",
    "Deps",
    "Flasher",
    "Frame",
    "KlipperUnit",
    "LinkError",
    "Moonraker",
    "MoonrakerError",
    "RefusedError",
    "SerialLink",
    "Unit",
    "build_frame",
    "blink",
    "calibration_text",
    "candidate_ports",
    "count_units",
    "crc16",
    "embedded_dictionary",
    "find_service",
    "flash_leds",
    "identify",
    "is_klippy",
    "klipper_block",
    "klipper_crc",
    "klipper_view",
    "klipper_vlq",
    "load_image",
    "main",
    "parse_calibration",
    "parse_frames",
    "parse_klipper_blocks",
    "parse_klipper_vlq",
    "pb_bytes",
    "pb_fields",
    "pb_uint",
    "port_holders",
    "read_calibration",
    "read_uid",
]
