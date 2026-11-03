"""The Anycubic ACE 2 Pro as a Klipper MCU: the host module.

Startup: when Klipper identifies the MCU, this module queries ``ace2k_version``. The firmware
takes this as proof that the host link works (see ``docs/protocol.md``). The module also reads the
link-proof state, and sends the proof once more if the unit reports it as not proven. Then it
reads the device identifier of the unit (``ace2k_uid_query``).

At connect, the module sends the insert band of every lane as an init command. The band is the
``laneN_insert_mv`` override, or ``ace2k_sensors_thresholds_reset`` to return to the band of the
unit. So a changed or removed override never resets the MCU. The periodic reports also start as
init commands: ``ace2k_sensors_state`` at 10 Hz, and ``ace2k_env_state``, ``ace2k_lane_counters``
and ``ace2k_mains_state`` at 1 Hz.

Then the module asks for the bands in use. The answer is one report frame for the four lanes, and
it has no timeout of its own. So the module asks once more after 1.5 s if the frame has not
arrived, and logs a warning after another 1.5 s. The reports go into ``get_status()``, and the
switch events go into the log.

Health poll: once Klipper is ready, the module polls the self-test (``ace2k_health_query``, see
``ace2k_health.py``) every 10 s. The first poll comes 5 s after ready, after the 2 s boot window
of the firmware. A poll that the unit does not answer is logged and tried again. After five
failures in a row the poll stops, and the health is unknown (``ok`` is None and ``stale`` is
True) until the unit answers again. The poll, the re-read after ``RUN=1`` and ``ACEK_HEALTH``
take turns on one reactor mutex. Klipper keys the response handler of a query by its name, so two
queries in flight would take each other's answer.

If the firmware was built without a subsystem, its query command is missing from the dictionary.
The module logs this and leaves the subsystem out. Its init command, subscriptions and polls are
skipped, and its status fields stay None.

G-codes:

- ``ACEK_STATUS`` prints the state.
- ``ACEK_RESTORE_STOCK`` sends ``ace2k_bootloader_enter``. The unit then resets into the recovery
  mode of the bootloader (see ``docs/flashing.md``).
- ``ACEK_CALIBRATION_SAVE`` stages the insert thresholds in use for ``SAVE_CONFIG``.
- ``ACEK_COUNTERS_RESET`` sets the encoder and tach counters of one lane, or of all four, to
  zero. The module and the unit both refuse it while a lane moves or is busy. An older firmware
  takes the command without confirming it.
- ``ACEK_HEALTH`` shows the two masks of the self-test by name. It shows ``not run yet`` before the
  first full pass of the firmware, and ``unknown`` when the unit does not answer. ``RUN=1`` runs
  the test again and reads the result 2.5 s later. If the unit still reports the pass that was
  read before the run, it reads again, up to three times, one second apart. ``RUN=1`` also reads
  the result itself when no poll has landed yet. ``CLEAR=1`` clears the latched faults.

The feed G-codes are in ``ace2k_feed.py``, the tag G-codes in ``ace2k_rfid.py``, the dryer,
fan and flap G-codes in ``ace2k_dryer.py``, and ``ACEK_DRY_MODE`` in ``ace2k_dryer_mode.py``.

Configuration::

    [ace2k]           # one unit; or, for several units, name each one:  [ace2k <name>]
    mcu: ace2k        # the [mcu <name>] section of the unit (the default is ace2k for [ace2k];
                      # required in [ace2k <name>]; "mcu" means the primary MCU)
    #lane1_insert_mv: 600, 1100   # optional, lane1..lane4: the insert band in millivolts,
                                  # low < high; without it the factory calibration is used

``[ace2k]`` also registers the temperature sensor type ``ace2k`` (see ``ace2k_env.py``). Every
unit section (``[ace2k]`` or each ``[ace2k <name>]``) must come before the
``[temperature_sensor]`` sections that use it.
"""

from __future__ import annotations

import logging
import struct
import threading
from collections.abc import Sequence
from typing import TYPE_CHECKING, Callable, cast

try:
    from .ace2k_health import (  # noqa: F401 — re-exported
        HEALTH_BITS,
        HEALTH_CLEAR,
        HEALTH_FIRST_POLL_S,
        HEALTH_POLL_FAILURES_MAX,
        HEALTH_POLL_S,
        HEALTH_QUERY,
        HEALTH_RERUN_POLL_S,
        HEALTH_RERUN_RETRIES,
        HEALTH_RERUN_RETRY_S,
        HEALTH_RUN,
        HEALTH_STATE,
        RESET_CAUSES,
        bit_names,
    )
    from .ace2k_link import (  # noqa: F401 — re-exported
        ENV_REPORT_S,
        FORMAT_HINT,
        FORMAT_MISMATCH,
        LANE_ALL,
        LANE_REPORT_S,
        LANES,
        MAINS_REPORT_S,
        SENSORS_REPORT_S,
        NoAnswerError,
        ask,
        constant_or_none,
        probe_format,
        query_mutex,
        refuse_format,
        subscribe_response,
        whole,
    )
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    from ace2k_health import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported: the names stay importable from here
        HEALTH_BITS,
        HEALTH_CLEAR,
        HEALTH_FIRST_POLL_S,
        HEALTH_POLL_FAILURES_MAX,
        HEALTH_POLL_S,
        HEALTH_QUERY,
        HEALTH_RERUN_POLL_S,
        HEALTH_RERUN_RETRIES,
        HEALTH_RERUN_RETRY_S,
        HEALTH_RUN,
        HEALTH_STATE,
        RESET_CAUSES,
        bit_names,
    )
    from ace2k_link import (  # type: ignore[import-not-found, no-redef]  # noqa: F401 — re-exported: the names stay importable from here
        ENV_REPORT_S,
        FORMAT_HINT,
        FORMAT_MISMATCH,
        LANE_ALL,
        LANE_REPORT_S,
        LANES,
        MAINS_REPORT_S,
        SENSORS_REPORT_S,
        NoAnswerError,
        ask,
        constant_or_none,
        probe_format,
        query_mutex,
        refuse_format,
        subscribe_response,
        whole,
    )

try:
    from . import ace2k_dryer, ace2k_dryer_mode, ace2k_feed, ace2k_rfid, ace2k_units
    from .ace2k_health import UnitHealth
except ImportError:  # the host tests import by bare name, with klippy/extras on the path
    import ace2k_dryer  # type: ignore[import-not-found, no-redef]
    import ace2k_dryer_mode  # type: ignore[import-not-found, no-redef]
    import ace2k_feed  # type: ignore[import-not-found, no-redef]
    import ace2k_rfid  # type: ignore[import-not-found, no-redef]
    import ace2k_units  # type: ignore[import-not-found, no-redef]
    from ace2k_health import UnitHealth  # type: ignore[no-redef]

if TYPE_CHECKING:
    from .ace2k_klipper_types import Command, Config, GCodeCommand, LaneStatus, Params, QueryCommand
    from .ace2k_link import EnvState

    EnvListener = Callable[[float, EnvState], None]  # (print time, report)
    CountersListener = Callable[[float, Sequence[int]], None]  # (receive time, encoder µm)
    # A live scale change that no frame has shown yet: old scale, new scale, the count of the
    # reference, and the time after which any frame is at the new scale.
    ScaleChange = tuple[float, float, float, float]


KIND_NAMES = {0: "insert", 1: "rest", 2: "pushed", 3: "pulled", 4: "cutout"}
CAL_NAMES = {0: "default", 1: "factory", 2: "config"}
VALID_NAMES = {0x1: "ntc_left", 0x2: "ntc_right", 0x4: "chamber", 0x8: "vdda"}
# ace2k_mains_state hz10 values that are not frequencies (docs/protocol.md): shown by name
MAINS_MARKERS = {1: "missed edge", 2: "stale"}
# A lane-counters frame that arrives more than one report period (LANE_REPORT_S) plus this margin
# after a live scale change went out is at the new scale, whatever it reads (see
# _follow_scale_change). The unit
# packs each frame when it sends it. So a frame at the old scale arrives within one round trip of
# the link after the change. The settle time is a generous upper limit on that.
SCALE_CHANGE_MARGIN_S = 0.2
RESET_CMD = "ace2k_lane_counters_reset lane=%c"
# The answer of the unit to the reset. An older dictionary has the command without this answer.
RESET_RESP = "ace2k_lane_counters_reset_response lane=%c accepted=%c"
THRESHOLDS_QUERY = "ace2k_sensors_thresholds_query"
LANE_COUNTERS_QUERY = "ace2k_lane_counters_query rest_ticks=%u"
# The motor's travel each way (an image since v0.13.0). It is off at boot, and runs at
# MOTOR_REPORT_S while a module watches it (motor_watch).
MOTOR_QUERY = "ace2k_lane_motor_query rest_ticks=%u"
MOTOR_REPORT = "ace2k_lane_motor fwd_um=%*s back_um=%*s"
MOTOR_REPORT_S = 1.0
U32 = 1 << 32
# The constant that says the image takes every key as an init command (see Ace2k.keys_as_init).
KEYS_AS_INIT_CONSTANT = "ACE2K_KEYS_AS_INIT"
# The constant that says a proof the heater put off waits past the window (see _watch_linkproof).
RETRY_PAST_WINDOW_CONSTANT = "ACE2K_LINKPROOF_RETRY_PAST_WINDOW"
VALID_CHAMBER = 0x4  # the chamber bit of the env report; the humidity is valid only if it is set
# One entry per firmware subsystem: the flag that compiles it, its query command and its report
# period. The query command is the probe. A firmware built without the flag has no such command
# in its dictionary. The four report queries share the signature "rest_ticks=%u". The health
# query takes no argument, and the host polls it (the period is None, so it has no init command).
SUBSYSTEMS = (
    ("sensors", "CONFIG_ACE2K_SENSORS", "ace2k_sensors_query", SENSORS_REPORT_S),
    ("env", "CONFIG_ACE2K_ENV", "ace2k_env_query", ENV_REPORT_S),
    ("lane", "CONFIG_ACE2K_LANE", LANE_COUNTERS_QUERY.split(maxsplit=1)[0], LANE_REPORT_S),
    ("mains", "CONFIG_ACE2K_MAINS", "ace2k_mains_query", MAINS_REPORT_S),
    ("health", "CONFIG_ACE2K_HEALTH", HEALTH_QUERY, None),
)
# The switch word of the sensors report. Bit i is the insert switch of lane i. Then comes a block
# of four bits for the plunger at rest of each lane, and a block of four for the plunger pushed in.
# The last two are single bits.
SWITCH_REST_SHIFT = 4
SWITCH_PUSHED_SHIFT = 8
SWITCH_PULLED_ANY_BIT = 12
SWITCH_CUTOUT_BIT = 13
THRESHOLDS_REQUERY_S = 1.5  # the thresholds frame is a report and not a reply, so ask once more
LINKPROOF_STATE = "ace2k_linkproof_state"
LINKPROOF_WATCH_S = 2.0  # how often an unproven link's state is read after Klipper is ready
LINKPROOF_WATCH_FAILURES_MAX = 5  # unanswered reads in a row before the watch gives up


UID_BYTES = 12  # the MCU's unique identifier, 96 bits
UID_DIGITS = 2 * UID_BYTES  # the number of hexadecimal digits in the uid: key
INSERT_MV_MAX = 3300  # the upper limit of a band: the 3.3 V reference of the ADC
SENSORS_NO_LANE = 255  # the lane value of a sensors event that belongs to no lane (the cutout)


def _parse_uid(text: str | None, section: str, error: type[Exception]) -> str | None:
    """Return the uid: key as 24 lower-case hex digits, or None if the key is absent.

    Spaces, colons, dashes and either case are accepted. Anything else raises ``error``."""
    if text is None:
        return None
    digits = "".join(text.split()).replace(":", "").replace("-", "").lower()
    if len(digits) != UID_DIGITS or any(c not in "0123456789abcdef" for c in digits):
        raise error(
            f"{section}: uid: must be the unit's 24 hexadecimal digits (ACEK_STATUS shows it)"
        )
    return digits


def _read_bands(config: Config, section: str) -> dict[int, tuple[int, int]]:
    """Read the laneN_insert_mv keys. Return lane -> (low, high) in mV."""
    thresholds: dict[int, tuple[int, int]] = {}
    for lane in range(LANES):
        key = f"lane{lane + 1}_insert_mv"
        pair = config.getintlist(key, None, count=2)
        if pair is not None:
            low, high = pair
            if not 0 <= low < high <= INSERT_MV_MAX:
                raise config.error(f"{section}: {key} must be 'low, high' millivolts, low < high")
            thresholds[lane] = (low, high)
    return thresholds


class Ace2k(UnitHealth):
    """One ACE 2 Pro unit: its link, its reports and its status."""

    # Set once the unit has built it. A unit that is still loading has none yet, so ace2k_units
    # reads it with getattr.
    feed: ace2k_feed.Ace2kFeed
    # The motor report (see motor_watch): its query on an image that has it.
    motor_cmd: Command | None
    # The modules that watch the motor report now; the report is on while there is one.
    motor_watchers: int
    # Per lane, the last motor report (forward µm, back µm) of the current watch, as the unit sent
    # it. None while no module watches, and before the first report of a watch.
    motor_last: list[tuple[int, int] | None]
    # Per lane, the travel since boot as of the first report of the current watch, unwrapped
    # across the 2^32 of the report from there.
    motor_um: list[list[int]]
    # The motor reports received since Klipper started. A watcher tells a new report by it.
    motor_reports: int

    def __init__(self, config: Config) -> None:
        self.printer = config.get_printer()
        self.section = config.get_name()
        self.name = ace2k_units.unit_name(self.section)  # None for a plain [ace2k] section
        mcu_name = config.get("mcu", None if self.name is not None else "ace2k")
        if mcu_name is None:
            raise config.error(f"{self.section}: mcu: is required in a named section")
        # Klipper registers the primary MCU as "mcu" and the others as "mcu <name>".
        self.mcu = self.printer.lookup_object("mcu" if mcu_name == "mcu" else "mcu " + mcu_name)
        self.units = ace2k_units.attach(config)
        self.units.add(self)
        # uid is the identifier of the unit itself (ACEK_STATUS shows it), checked at every
        # connect. The serial key of the MCU names the USB adapter in the cable. So two swapped
        # cables would give this section the other unit, with its lanes and calibration.
        self.expected_uid = _parse_uid(config.get("uid", None), self.section, config.error)
        self.version: str | None = None
        # None before the connect. It can turn from False to True later (see _watch_linkproof),
        # and never the other way.
        self.link_proven: bool | None = None
        self.linkproof_query: QueryCommand | None = None  # the state query, built at identify
        # Use klippy:mcu_identify and not klippy:connect. Connect handlers stop at the first
        # error, so a config error elsewhere would leave a new unit unproven. A unit resets into
        # recovery when its time window ends. The MCU objects register their own mcu_identify
        # handlers first and load the dictionary, so the queries below work when this one runs.
        self.printer.register_event_handler("klippy:mcu_identify", self._on_mcu_identify)
        self.register_command("ACEK_STATUS", self.cmd_ACEK_STATUS, self.cmd_ACEK_STATUS_help)
        self.register_command(
            "ACEK_RESTORE_STOCK", self.cmd_ACEK_RESTORE_STOCK, self.cmd_ACEK_RESTORE_STOCK_help
        )
        self.thresholds = _read_bands(config, self.section)
        self._init_reports()
        # The state of the self-test (see ace2k_health.py). There is no verdict before an answer.
        self._init_health()
        self.thresholds_requeried = False
        self.uid: str | None = None
        self._keys_as_init: bool | None = None  # the cached result of keys_as_init()
        # The length of an identifier answer, if it was not UID_BYTES.
        self.uid_answer_bytes: int | None = None
        # Why the identifier query got no answer (the text of the error).
        self.uid_unread: str | None = None
        self.present: set[str] = set()  # the subsystems of the firmware, known at config time
        self.reset_confirmed = False  # True if the firmware answers a counters reset (RESET_RESP)
        # The commands that are sent after the connect. _build_config builds each one once, for a
        # subsystem that the firmware has. They are None until then.
        self.thresholds_cmd: Command | None = None
        self.counters_period_cmd: Command | None = None
        self.motor_cmd = None
        self.motor_watchers = 0
        self.reset_query: QueryCommand | None = None
        self.reset_cmd: Command | None = None
        self.reactor = self.printer.get_reactor()
        self._init_health_timers()  # the re-read after RUN=1, and the one query in flight
        self._init_linkproof_watch()
        self.printer.load_object(config, "ace2k_env")  # the sensor type, loaded once per printer
        # Use the spelling of the section, as Klipper does for [ace2k <name>]. Every object of one
        # section gets the same suffix ("ace2k A", "ace2k_feed A"). UNIT= ignores case.
        suffix = self.section[len("ace2k") :]
        self.feed = ace2k_feed.Ace2kFeed(config, self)
        self.printer.add_object("ace2k_feed" + suffix, self.feed)
        self.units.map_changed()  # its laneN_extruder keys now join the shared extruders
        # The automatic encoder calibration of the feed reads the lane counters and sets their
        # period through this object. When the calibration sets a scale, this object keeps the
        # reported encoder continuous.
        self.register_counters_listener(self.feed.cal.on_counters)
        self.rfid = ace2k_rfid.Ace2kRfid(config, self)
        self.printer.add_object("ace2k_rfid" + suffix, self.rfid)
        self._add_dryer(config, suffix)
        self.mcu.register_config_callback(self._build_config)
        # The MCU objects are created before the other sections. So the connect handler of the MCU
        # (its config and init commands) runs before this one.
        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.register_command(
            "ACEK_CALIBRATION_SAVE",
            self.cmd_ACEK_CALIBRATION_SAVE,
            self.cmd_ACEK_CALIBRATION_SAVE_help,
        )
        self.register_command(
            "ACEK_COUNTERS_RESET", self.cmd_ACEK_COUNTERS_RESET, self.cmd_ACEK_COUNTERS_RESET_help
        )
        self.register_command("ACEK_HEALTH", self.cmd_ACEK_HEALTH, self.cmd_ACEK_HEALTH_help)

    def _add_dryer(self, config: Config, suffix: str) -> None:
        """Create the dryer and its mode, and add them as printer objects."""
        self.dryer = ace2k_dryer.Ace2kDryer(config, self)
        self.printer.add_object("ace2k_dryer" + suffix, self.dryer)
        self.dryer_mode = ace2k_dryer_mode.Ace2kDryerMode(config, self, self.dryer)
        self.printer.add_object("ace2k_dryer_mode" + suffix, self.dryer_mode)

    def _init_reports(self) -> None:
        """Set the report fields (lanes, mains, environment, listeners) before any report."""
        # The sensors and counters of each lane, as get_status hands them to Klipper.
        self.lanes: list[LaneStatus] = [
            dict(
                insert=None,
                insert_mv=None,
                empty_mv=None,
                rest=None,
                pushed=None,
                low_mv=None,
                high_mv=None,
                source=None,
                encoder_um=None,
                fg=None,
            )
            for _ in range(LANES)
        ]
        self.pulled_any: bool | None = None
        self.cutout: bool | None = None
        self.aux_mv: int | None = None
        self.age_ms: int | None = None
        # The number of sensors state reports received since the start. It tells two reports
        # apart.
        self.sensors_reports = 0
        self.mains_hz: float | None = None
        self.mains_present: bool | None = None
        self.mains_note: str | None = None
        self.mains_rejects: int | None = None
        self.env: EnvState = dict(
            ptc_left=None,
            ptc_right=None,
            chamber=None,
            humidity=None,
            vdda=None,
            valid=[],
            valid_mask=0,
        )
        self.env_listeners: list[EnvListener] = []
        # Each callback is called as cb(receive_time, encoder_um) for every lane-counters frame.
        self.counter_listeners: list[CountersListener] = []
        # The µm that is added to the reported encoder of a lane. It keeps encoder_mm continuous
        # across a live scale change.
        self.encoder_offset_um = [0] * LANES
        # Per lane, a live scale change that no frame has shown yet. It holds (old scale, new
        # scale, count of the reference, time after which any frame is at the new scale).
        self.scale_pending: list[ScaleChange | None] = [None] * LANES
        self.motor_last, self.motor_um = [None] * LANES, [[0, 0] for _ in range(LANES)]
        self.motor_reports = 0
        # Protects the frames on the serial thread (the counters and the motor reports) against
        # the scale changes, resets, motor watch changes and status reads on the reactor.
        self.encoder_lock = threading.Lock()

    def register_command(
        self, name: str, func: Callable[[GCodeCommand], None], desc: str | None = None
    ) -> None:
        """Register a G-code of this unit with the coordinator (UNIT=)."""
        self.units.register_command(self, name, func, desc)

    def say(self, line: str) -> None:
        """Print a console line outside a command. With several units, the line names this unit."""
        self.units.say(self, line)

    def head(self, part: str | None = None) -> str:
        """Return the start of the lines of this unit, "ace2k:" or "ace2k a:" (see Units.head)."""
        return self.units.head(self, part)

    def lead(self, words: str) -> str:
        return self.units.lead(self, words)

    def own(self, text: str) -> str:
        return self.units.own(self, text)

    def lane_text(self, lane: int) -> str:
        """Return the lane as written in the lines of this unit ("2", or "a:2" with several)."""
        return self.units.lane_text(self, lane)

    def _on_mcu_identify(self) -> None:
        """Prove the link. The firmware stores the proof when it answers the first query."""
        version_cmd = self.mcu.lookup_query_command(
            "ace2k_version", "ace2k_version_response version=%s"
        )
        state_cmd = self.mcu.lookup_query_command(
            LINKPROOF_STATE,
            "ace2k_linkproof_state_response proven=%c remaining_ms=%u",
        )
        self.linkproof_query = state_cmd
        self.linkproof_retry_past_window = bool(
            constant_or_none(self.mcu, RETRY_PAST_WINDOW_CONSTANT)
        )
        version: str = version_cmd.send()["version"].decode()
        self.version = version
        state = state_cmd.send()
        if not state["proven"]:
            # The unit answered but did not store the proof: its flash write failed, or the
            # heater held the flash. Send one more query, and then take the state as it is.
            version_cmd.send()
            state = state_cmd.send()
        self.link_proven = bool(state["proven"])
        if not self.link_proven:
            logging.warning(
                "%s the link proof is not stored after a retry (remaining_ms=%d): the store "
                "failed on the unit (a flash error), or the heater holds the flash; the module "
                "watches the state until the unit stores it — see docs/protocol.md",
                self.head(version),
                state["remaining_ms"],
            )
        logging.info("%s connected, link proven: %s", self.lead(version), self.link_proven)
        # The device identifier belongs to the health subsystem. A firmware built without it has
        # no such command, and then uid stays None.
        if self.mcu.try_lookup_command("ace2k_uid_query") is not None:
            uid_cmd = self.mcu.lookup_query_command("ace2k_uid_query", "ace2k_uid_response uid=%*s")
            try:
                self._take_uid(bytes(uid_cmd.send()["uid"]))
            except self.printer.command_error as err:
                # No answer: the link may be lost, or an older image may still be stopped from an
                # earlier shutdown (it keeps that across a host restart; a newer image answers).
                # Only warn here. Klipper's own connect check raises the error for a unit that is
                # shut down. If the uid key is set, _check_uids says the uid was not read.
                self.uid_unread = str(err)
                logging.warning("%s the unit's identifier was not read: %s", self.head(), err)
        # The check of the uid key waits for klippy:connect (see _check_uids). A config error that
        # is raised here would end the identify step of the units after this one, and leave them
        # unproven.

    def _init_linkproof_watch(self) -> None:
        """Create the timer of the link proof watch. It is armed at ready, for an unproven link."""
        self.linkproof_watch_failures = 0
        self.linkproof_window_warned = False
        # True when the image keeps the window open past its end (RETRY_PAST_WINDOW_CONSTANT),
        # read at identify.
        self.linkproof_retry_past_window = False
        self.linkproof_timer = self.reactor.register_timer(self._watch_linkproof)
        self.printer.register_event_handler("klippy:ready", self._start_linkproof_watch)
        self.printer.register_event_handler("klippy:shutdown", self._stop_linkproof_watch)
        self.printer.register_event_handler("klippy:disconnect", self._stop_linkproof_watch)

    def _start_linkproof_watch(self) -> None:
        if self.link_proven is False:
            self.reactor.update_timer(
                self.linkproof_timer, self.reactor.monotonic() + LINKPROOF_WATCH_S
            )

    def _stop_linkproof_watch(self) -> None:
        self.reactor.update_timer(self.linkproof_timer, self.reactor.NEVER)

    def _watch_linkproof(self, eventtime: float) -> float:
        """Read the state of an unproven link until the unit stores the proof or the window ends.

        A store the heater put off is made by the unit itself, later. Only the state is asked.
        A resent ace2k_version would make the unit erase and write its page again, and a flash
        error would only repeat. If the unit does not answer, the next pass asks again, up to
        LINKPROOF_WATCH_FAILURES_MAX in a row.

        An image with ACE2K_LINKPROOF_RETRY_PAST_WINDOW waits for the heater past the window while
        the host keeps talking, so the watch goes on after it, until the proof is stored or the
        unit stops answering. Klipper's own clock queries keep the link alive meanwhile. An older
        image resets into recovery at the end of the window, so the watch stops there.
        """
        assert self.linkproof_query is not None  # the watch starts only after identify
        try:
            state = ask(self.printer, self.mcu, self.linkproof_query, (), "link proof state")
        except NoAnswerError as e:
            self.linkproof_watch_failures += 1
            if self.linkproof_watch_failures >= LINKPROOF_WATCH_FAILURES_MAX:
                logging.warning(
                    "%s the link proof watch stopped after %d unanswered reads: %s",
                    self.head(),
                    self.linkproof_watch_failures,
                    e,
                )
                return self.reactor.NEVER
            if self.linkproof_watch_failures == 1:
                logging.warning("%s the link proof state was not read: %s", self.head(), e)
            return eventtime + LINKPROOF_WATCH_S
        self.linkproof_watch_failures = 0
        if state["proven"]:
            self.link_proven = True
            logging.info("%s the unit stored the link proof by itself", self.head())
            return self.reactor.NEVER
        if state["remaining_ms"] == 0:
            if not self.linkproof_retry_past_window:
                logging.warning(
                    "%s the window ended with the link proof unstored; the unit resets into the "
                    "bootloader's recovery as soon as no veto objects (the heater, the fans, a "
                    "lane moving) — see docs/protocol.md",
                    self.head(),
                )
                return self.reactor.NEVER
            if not self.linkproof_window_warned:
                self.linkproof_window_warned = True
                logging.warning(
                    "%s the window ended with the link proof unstored; while the host keeps "
                    "talking, the unit stores it when the heater lets the flash go, and stays. "
                    "After a flash error, or once the host is silent for 3 s, it resets into the "
                    "bootloader's recovery as soon as no veto objects — see docs/protocol.md",
                    self.head(),
                )
        return eventtime + LINKPROOF_WATCH_S

    def _take_uid(self, raw: bytes) -> None:
        """Store the identifier that the unit answered.

        If it is UID_BYTES long, it is the uid. Another length is a fault of the unit or of the
        link, and the uid stays unknown."""
        if len(raw) == UID_BYTES:
            self.uid = raw.hex()
            logging.info("%s %s", self.lead("unit"), self.uid)
            return
        self.uid_answer_bytes = len(raw)
        logging.warning(
            "%s the unit's identifier came back %d bytes long, not %d: uid unknown",
            self.head(),
            len(raw),
            UID_BYTES,
        )

    def _uid_problem(self) -> str | None:
        """Return a description of the problem, or None if ``uid:`` is not set or matches the unit.

        The problem is that another unit answered (named when ``uid:`` of another section is its
        uid), or that the uid could not be read."""
        if self.expected_uid is None:
            return None
        if self.uid is None:
            if self.uid_unread is not None:
                why = (
                    f"the unit did not answer its identifier query ({self.uid_unread}): it may be"
                    " stopped from an earlier shutdown (FIRMWARE_RESTART), or its link lost"
                )
            elif self.uid_answer_bytes is not None:
                why = (
                    f"the unit answered an identifier {self.uid_answer_bytes} bytes long, not"
                    f" {UID_BYTES} (a fault of the unit or of its link)"
                )
            else:
                why = (
                    "the unit's firmware has no identifier query (CONFIG_ACE2K_HEALTH); remove"
                    " uid: or flash a full image"
                )
            return f"{self.head()} uid: is set but cannot be checked: {why}"
        if self.uid == self.expected_uid:
            return None
        owner = next(
            (u.section for u in self._peers() if u is not self and u.expected_uid == self.uid),
            None,
        )
        whose = f" (the uid: of [{owner}])" if owner else ""
        return (
            f"{self.head()} the unit on this serial port is {self.uid}{whose}, not the uid:"
            f" {self.expected_uid} of [{self.section}]. The cables of two units may be swapped:"
            " check which unit each serial: reaches (ACEK_STATUS shows a unit's uid), then"
            " restart"
        )

    def _peers(self) -> list[Ace2k]:
        """Return the units that the coordinator holds, including this one. Each is an Ace2k."""
        return cast("list[Ace2k]", self.units.units)

    def _check_uids(self) -> None:
        """Check the uid key of every unit, after all units are identified and proven.

        The connect of the first unit raises one config error that lists every problem."""
        problems = [p for p in (u._uid_problem() for u in self._peers()) if p is not None]
        if problems:
            raise self.printer.config_error("\n".join(problems))

    def keys_as_init(self) -> bool:
        """Return True if the image takes every key of this unit as an init command.

        This is the case when its dictionary names ACE2K_KEYS_AS_INIT, whatever subsystems it has.
        Then the bands, the encoder scales, the supervisor thresholds and the reach of the tag
        search go out at every connect. They are not part of the config checksum of Klipper, so
        editing a key never resets the MCU. This matters on the U1, where a restart after a
        checksum mismatch fails.

        An older image keeps all of them as config commands, so a host update alone leaves the
        checksum as it was. The result is read once and kept."""
        if self._keys_as_init is None:
            self._keys_as_init = constant_or_none(self.mcu, KEYS_AS_INIT_CONSTANT) is not None
        return self._keys_as_init

    def add_key_cmd(self, text: str, has_key: bool) -> None:
        """Add a command that carries keys of printer.cfg.

        On an image that takes its keys as init commands (see keys_as_init), it is an init command
        at every connect, with the key or the default that the caller put in the text. On an older
        image it is a config command, and it is added only if ``has_key`` is True."""
        if self.keys_as_init():
            self.mcu.add_config_cmd(text, is_init=True)
        elif has_key:
            self.mcu.add_config_cmd(text)

    def _build_bands(self) -> None:
        """Add the band of every lane (see add_key_cmd).

        A lane without a key goes back to the default band of the unit. An older image gets only
        the keys, and a removal reaches it by an MCU reset."""
        for lane in range(LANES):
            band = self.thresholds.get(lane)
            if band is not None:
                low, high = band
                self.add_key_cmd(
                    f"ace2k_sensors_thresholds_set lane={lane} low_mv={low} high_mv={high}", True
                )
            else:
                self.add_key_cmd(f"ace2k_sensors_thresholds_reset lane={lane}", False)

    def _build_config(self) -> None:
        """Add the bands and periodic queries as init commands, and subscribe to the reports.

        Only subsystems that the firmware has are used."""
        self.present = set()
        self.reset_confirmed = False
        # The MCU (re)starts with its motor report off, so no watch is on.
        self.motor_watchers = 0
        self._forget_motor()
        for name, flag, query, report_s in SUBSYSTEMS:
            probe = query if report_s is None else f"{query} rest_ticks=%u"
            if self.mcu.try_lookup_command(probe) is None:
                logging.warning(
                    "%s firmware built without %s; %s disabled", self.head(), flag, name
                )
                continue
            self.present.add(name)
            if name == "sensors":
                self._build_bands()  # before the sensors reports start, as config commands were
            if report_s is None:
                continue
            ticks = self.mcu.seconds_to_clock(report_s)
            self.mcu.add_config_cmd(f"{query} rest_ticks={ticks}", is_init=True)
        if "sensors" in self.present:
            self._subscribe(
                self._handle_sensors_state,
                "ace2k_sensors_state switches=%hu insert_mv=%*s empty_mv=%*s aux_mv=%hu age_ms=%c",
            )
            self._subscribe(
                self._handle_sensors_event, "ace2k_sensors_event lane=%c kind=%c level=%c"
            )
            self._subscribe(
                self._handle_thresholds,
                "ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s",
            )
        if "env" in self.present:
            self._subscribe(
                self._handle_env_state,
                "ace2k_env_state ptc_left_mc=%i ptc_right_mc=%i chamber_mc=%i"
                " chamber_rh_pct10=%hu vdda_mv=%hu valid=%c",
            )
        if "lane" in self.present:
            self._subscribe(self._handle_lane_counters, "ace2k_lane_counters encoder_um=%*s fg=%*s")
            self.motor_cmd = self.mcu.try_lookup_command(MOTOR_QUERY)
            if self.motor_cmd is not None:
                self._subscribe(self._handle_lane_motor, MOTOR_REPORT)
            self.reset_confirmed = self._reset_confirmation()
        if "mains" in self.present:
            self._subscribe(
                self._handle_mains_state, "ace2k_mains_state hz10=%hu present=%c rejects=%u"
            )
        self._build_commands()

    def _build_commands(self) -> None:
        """Build the commands that are sent after the connect, once.

        The lookups are valid only now, because the dictionary of the unit is known."""
        if "sensors" in self.present:
            self.thresholds_cmd = self.mcu.lookup_command(THRESHOLDS_QUERY)
        if "lane" in self.present:
            self.counters_period_cmd = self.mcu.lookup_command(LANE_COUNTERS_QUERY)
            if self.reset_confirmed:
                self.reset_query = self.mcu.lookup_query_command(RESET_CMD, RESET_RESP)
            else:
                self.reset_cmd = self.mcu.try_lookup_command(RESET_CMD)  # None: no reset at all
        if "health" in self.present:
            self._build_health()

    def _reset_confirmation(self) -> bool:
        """Return True if the firmware answers a counters reset (RESET_RESP, see probe_format).

        Another format means an unknown firmware, and the connect fails with a config error. If
        the answer is absent, the firmware is older. Then the reset goes out without confirmation,
        and the warning quotes the text of the parser."""
        verdict, detail = probe_format(self.mcu, RESET_RESP)
        if verdict == "ok":
            return True
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, RESET_RESP, (verdict, detail))
        logging.warning(
            "%s firmware answers no counters reset; ACEK_COUNTERS_RESET is sent unconfirmed (%s)",
            self.head(),
            detail,
        )
        return False

    def _subscribe(self, handler: Callable[[Params], None], msgformat: str) -> None:
        """Subscribe to a report whose format the dictionary has exactly as this module knows it.

        Another format means a firmware from another commit, and the connect fails with a config
        error. If the report is absent while the query of its subsystem is present, the module
        logs it and registers nothing, and the fields of the report stay None."""
        verdict, detail = probe_format(self.mcu, msgformat)
        if verdict == "differs":
            raise refuse_format(self.printer, self.section, msgformat, (verdict, detail))
        if verdict == "absent":
            logging.warning(
                "%s firmware carries no %s; its fields stay empty (%s)",
                self.head(),
                msgformat.split(maxsplit=1)[0],
                detail,
            )
            return
        subscribe_response(self.mcu, handler, msgformat)

    def _on_connect(self) -> None:
        """Check the uid of every unit (see _check_uids), then ask for the bands in use.

        The question is not an init command, because Klipper activates a subscription from the
        config callback only after the init commands have gone out. The answer is one report
        frame, and a lost frame raises no timeout. So a timer asks once more, and then warns.
        """
        self._check_uids()
        if "sensors" in self.present:
            assert self.thresholds_cmd is not None  # _build_commands built it with the sensors
            self.thresholds_cmd.send()
            self.reactor.register_timer(
                self._requery_thresholds, self.reactor.monotonic() + THRESHOLDS_REQUERY_S
            )

    def _thresholds_missing(self) -> bool:
        return any(lane["low_mv"] is None for lane in self.lanes)

    def _requery_thresholds(self, eventtime: float) -> float:
        """Ask for the bands once more, then warn once. There are no further retries."""
        if not self._thresholds_missing():
            return self.reactor.NEVER
        if not self.thresholds_requeried:
            self.thresholds_requeried = True
            assert self.thresholds_cmd is not None  # _on_connect sets the timer only with sensors
            self.thresholds_cmd.send()
            return eventtime + THRESHOLDS_REQUERY_S
        logging.warning(
            "%s the insert thresholds did not arrive after a re-query; the band fields of "
            "ACEK_STATUS stay empty and ACEK_CALIBRATION_SAVE has nothing to stage",
            self.head(),
        )
        return self.reactor.NEVER

    # The handlers run on the serial thread. They store the data and hand the work to the reactor.
    def _handle_sensors_state(self, params: Params) -> None:
        sw = params["switches"]
        insert_mv = struct.unpack(f"<{LANES}H", params["insert_mv"])
        empty_mv = struct.unpack(f"<{LANES}H", params["empty_mv"])
        for i, lane in enumerate(self.lanes):
            lane["insert"] = bool(sw & (1 << i))
            lane["rest"] = bool(sw & (1 << (SWITCH_REST_SHIFT + i)))
            lane["pushed"] = bool(sw & (1 << (SWITCH_PUSHED_SHIFT + i)))
            lane["insert_mv"] = insert_mv[i]
            lane["empty_mv"] = empty_mv[i]
        self.pulled_any = bool(sw & (1 << SWITCH_PULLED_ANY_BIT))
        self.cutout = bool(sw & (1 << SWITCH_CUTOUT_BIT))
        self.aux_mv = params["aux_mv"]
        self.age_ms = params["age_ms"]
        self.sensors_reports += 1  # last, so a reader that sees the new count sees all fields

    def _handle_sensors_event(self, params: Params) -> None:
        lane = params["lane"]
        logging.info(
            "%s lane=%s kind=%s level=%d",
            self.head("sensors event"),
            "-" if lane == SENSORS_NO_LANE else self.lane_text(lane),
            KIND_NAMES.get(params["kind"], params["kind"]),
            params["level"],
        )

    def _handle_thresholds(self, params: Params) -> None:
        low_mv = struct.unpack(f"<{LANES}H", params["low_mv"])
        high_mv = struct.unpack(f"<{LANES}H", params["high_mv"])
        source = params["source"]
        for i, lane in enumerate(self.lanes):
            lane["low_mv"] = low_mv[i]
            lane["high_mv"] = high_mv[i]
            lane["source"] = CAL_NAMES.get(source[i], source[i])

    def _handle_env_state(self, params: Params) -> None:
        valid = params["valid"]
        # The report carries the last chamber value even when the chamber bit is clear. That value
        # is not a humidity.
        humidity = params["chamber_rh_pct10"] / 10.0 if valid & VALID_CHAMBER else None
        env: EnvState = dict(
            ptc_left=params["ptc_left_mc"] / 1000.0,
            ptc_right=params["ptc_right_mc"] / 1000.0,
            chamber=params["chamber_mc"] / 1000.0,
            humidity=humidity,
            vdda=params["vdda_mv"] / 1000.0,
            valid_mask=valid,
            valid=[name for bit, name in sorted(VALID_NAMES.items()) if valid & bit],
        )
        self.env = env
        sent_time = params["#sent_time"]
        self.reactor.register_async_callback(lambda _eventtime: self._notify_env(env, sent_time))

    def _notify_env(self, env: EnvState, sent_time: float) -> None:
        print_time = self.mcu.estimated_print_time(sent_time)
        for listener in self.env_listeners:
            listener(print_time, env)

    def register_env_listener(self, cb: EnvListener) -> None:
        self.env_listeners.append(cb)

    def register_counters_listener(self, cb: CountersListener) -> None:
        """Call ``cb(receive_time, encoder_um)`` on the serial thread for each frame."""
        self.counter_listeners.append(cb)

    def _handle_lane_counters(self, params: Params) -> None:
        encoder_um = struct.unpack(f"<{LANES}i", params["encoder_um"])
        fg = struct.unpack(f"<{LANES}I", params["fg"])
        receive_time = params.get("#receive_time", 0.0)
        with self.encoder_lock:
            for i, lane in enumerate(self.lanes):
                if self.scale_pending[i] is not None:
                    self._follow_scale_change(i, encoder_um[i], receive_time)
                lane["encoder_um"] = encoder_um[i]
                lane["fg"] = fg[i]
        for cb in self.counter_listeners:
            cb(receive_time, encoder_um)

    def _follow_scale_change(self, lane: int, raw: int, receive_time: float) -> None:
        """Handle a frame for a lane whose scale changed live, before any frame at the new scale.

        The caller holds encoder_lock. The frame is the first one at the new scale if it arrived
        after the settle time, or if its value is closer to the reference count read at the new
        scale than at the old scale. Then the offset is applied with this frame, so the reading
        continues where the last one stopped. Otherwise the frame is read with the old offset,
        because the unit sent it before it took the change.

        The value test assumes that the jump of the scale (about 1 % of the travel of the lane since
        boot) exceeds the motion within one frame. If not, the settle time still ends the wait.
        The reference count never changes, so a frame wrongly read as one at the old scale cannot
        move it."""
        pending = self.scale_pending[lane]
        assert pending is not None  # the caller checked that a change is pending
        old, new, count, settled = pending
        if receive_time > settled or (
            abs(raw - count * new * 1000.0) < abs(raw - count * old * 1000.0)
        ):
            self.encoder_offset_um[lane] += int(round(count * (old - new) * 1000.0))
            self.scale_pending[lane] = None

    def _handle_mains_state(self, params: Params) -> None:
        hz10 = params["hz10"]
        # A marker is not a measurement. Keep its name and no frequency.
        self.mains_note = MAINS_MARKERS.get(hz10)
        self.mains_hz = None if self.mains_note else hz10 / 10.0
        self.mains_present = bool(params["present"])
        self.mains_rejects = params["rejects"]

    def set_counters_period(self, seconds: float) -> None:
        """Set the period of the lane counters, in seconds (an automatic calibration uses 10 Hz).

        A firmware built without the lane counters does nothing."""
        if "lane" in self.present:
            ticks = self.mcu.seconds_to_clock(seconds)
            assert self.counters_period_cmd is not None  # built together with the lane counters
            self.counters_period_cmd.send([ticks])

    @property
    def has_motor_travel(self) -> bool:
        """True when the image reports the motor's travel each way (ace2k_lane_motor)."""
        return self.motor_cmd is not None

    def motor_watch(self, on: bool) -> bool:
        """Turn the motor report on (on=True) or off for one watcher, and return has_motor_travel.

        The report runs at MOTOR_REPORT_S while any watcher has it on: the first watch starts it,
        and the last one off stops it. An off with no watcher left does nothing. An image without
        the report does nothing either. A call whose send raises leaves the count as it was, so a
        later watch still turns the report on (or off). A connect forgets every watcher. The
        travel of a watch starts at its own first report: the readings stop when the last watcher
        goes, and a later watch never adds the travel between the two."""
        if self.motor_cmd is None:
            return False
        if on:
            if self.motor_watchers == 0:
                self._forget_motor()
                # Counted before the send, so its first report is taken; undone if it raises.
                self.motor_watchers = 1
                try:
                    self.motor_cmd.send([self.mcu.seconds_to_clock(MOTOR_REPORT_S)])
                except Exception:
                    self.motor_watchers = 0
                    raise
            else:
                self.motor_watchers += 1
        elif self.motor_watchers > 0:
            if self.motor_watchers == 1:
                self.motor_cmd.send([0])
            self.motor_watchers -= 1
            if self.motor_watchers == 0:
                self._forget_motor()
        return True

    def _forget_motor(self) -> None:
        """Drop the motor readings: the lane status reads None until the next watch's report."""
        with self.encoder_lock:
            self.motor_last = [None] * LANES

    def _handle_lane_motor(self, params: Params) -> None:
        fwd = struct.unpack(f"<{LANES}I", params["fwd_um"])
        back = struct.unpack(f"<{LANES}I", params["back_um"])
        with self.encoder_lock:
            if self.motor_watchers == 0:
                return  # sent before the last watcher stopped the report
            self.motor_reports += 1
            for i in range(LANES):
                last = self.motor_last[i]
                total = self.motor_um[i]
                if last is None:
                    total[0], total[1] = fwd[i], back[i]
                else:
                    total[0] += (fwd[i] - last[0]) % U32
                    total[1] += (back[i] - last[1]) % U32
                self.motor_last[i] = (fwd[i], back[i])

    def keep_encoder_continuous(self, lane: int, old_scale: float, new_scale: float) -> None:
        """Keep the reported encoder continuous across a live scale change.

        After a live scale change, the unit converts its whole count with the new scale. An offset
        keeps the reported encoder where it was, so a reader of encoder_mm sees no motion that did
        not happen. Call this before the scale is sent, so that the last frame read (the reference)
        is at the old scale. The offset is applied with the first frame at the new scale (see
        _follow_scale_change)."""
        with self.encoder_lock:
            raw = self.lanes[lane]["encoder_um"]
            if raw is None:
                return  # there is no reading yet, so there is nothing to keep
            pending = self.scale_pending[lane]
            if pending is not None:
                # No frame at the scale that this change replaces has come yet. So this change
                # starts from the previous one.
                old_scale, _, count, _ = pending
            else:
                count = raw / (old_scale * 1000.0)
            if int(round(count * (old_scale - new_scale) * 1000.0)):
                settled = self.reactor.monotonic() + LANE_REPORT_S + SCALE_CHANGE_MARGIN_S
                self.scale_pending[lane] = (old_scale, new_scale, count, settled)
            else:
                self.scale_pending[lane] = None  # no offset to keep: no count, or no change

    def _drop_scale_changes(self, index: int) -> None:
        """Drop the offset and pending scale change of lane ``index`` (or LANE_ALL)."""
        with self.encoder_lock:
            for i in range(LANES) if index == LANE_ALL else [index]:
                self.encoder_offset_um[i] = 0
                self.scale_pending[i] = None

    def _lane_status(self, index: int, lane: LaneStatus) -> LaneStatus:
        """Return the fields of the lane, with ``encoder_mm`` (three decimals) added.

        ``encoder_mm`` is None before the first report and continuous across a live scale change.
        ``encoder_um`` is as the unit reports it. The fields of the feed for the lane are merged."""
        with self.encoder_lock:  # read the value and its offset from the same frame
            status = dict(lane)
            um = lane["encoder_um"]
            if um is not None:
                um += self.encoder_offset_um[index]
            motor = None if self.motor_last[index] is None else list(self.motor_um[index])
        status["encoder_mm"] = None if um is None else round(um / 1000.0, 3)
        for key, i in (("motor_fwd_mm", 0), ("motor_back_mm", 1)):
            status[key] = None if motor is None else round(motor[i] / 1000.0, 3)
        status.update(self.feed.lane_status(index))
        status["tag"] = self.rfid.lane_status(index)
        return status

    def get_status(self, _eventtime: float) -> dict[str, object]:
        return {
            "version": self.version,
            "link_proven": self.link_proven,
            "lanes": [self._lane_status(i, lane) for i, lane in enumerate(self.lanes)],
            "pulled_any": self.pulled_any,
            "cutout": self.cutout,
            "aux_mv": self.aux_mv,
            "sensors_age_ms": self.age_ms,
            "sensors_reports": self.sensors_reports,
            "motor_reports": self.motor_reports,
            "mains_hz": self.mains_hz,
            # Why mains_hz is None after a report: "missed edge" or "stale". Otherwise None.
            "mains_note": self.mains_note,
            "mains_present": self.mains_present,
            "mains_rejects": self.mains_rejects,
            "ptc_left": self.env["ptc_left"],
            "ptc_right": self.env["ptc_right"],
            "chamber": self.env["chamber"],
            "humidity": self.env["humidity"],
            "vdda": self.env["vdda"],
            "env_valid": self.env["valid"],
            "health": dict(self.health),
            # The field, fault and dead flags of each reader. None before the first report.
            "rfid_readers": self.rfid.reader_flags(),
            # The dryer, the fans and the flaps (see ace2k_dryer.py), and the dryer's mode (see
            # ace2k_dryer_mode.py). Every field is None before a report.
            "dryer": dict(self.dryer.status(), **self.dryer_mode.status()),
            "uid": self.uid,
            # For each extruder that several lanes feed, the lane that is followed by default
            # (see ACEK_LANE_ACTIVE). None while no lane is chosen.
            "active_lanes": self.feed.follow_auto.active_map(),
        }

    cmd_ACEK_STATUS_help = (
        "Report the ace2k firmware state: link, lanes, switches, counters, mains, temperatures"
    )

    def cmd_ACEK_STATUS(self, gcmd: GCodeCommand) -> None:
        proven = "yes" if self.link_proven else "no"
        lines = [self.lead(f"{self.version}, link proven: {proven}")]
        if self.uid is not None:
            lines.append(f"uid {self.uid}")  # a line of its own, because tools read the line above
        for i, lane in enumerate(self.lanes):
            status = self._lane_status(i, lane)
            lines.append(
                f"lane {i + 1}: insert={lane['insert']} ({lane['insert_mv']} mV, band"
                f" {lane['low_mv']}-{lane['high_mv']} mV {lane['source']})"
                f" empty={lane['empty_mv']} mV rest={lane['rest']} pushed={lane['pushed']}"
                f" encoder={status['encoder_mm']} mm fg={lane['fg']}"
            )
        if self.feed.present:
            lines.extend(self.feed.status_lines())
        if self.rfid.present:
            lines.extend(self.rfid.status_lines())
        if self.dryer.airflow_present:
            lines.extend(self.dryer.status_lines())
        if self.dryer_mode.present:
            lines.append(self.dryer_mode.status_line())
        lines.append(
            f"pulled_any={self.pulled_any} cutout={self.cutout} aux={self.aux_mv} mV"
            f" sensors_age={self.age_ms} ms"
        )
        if self.mains_present is None:
            mains = "no report yet"
        else:
            mains = "present" if self.mains_present else "absent"
        hz = self.mains_note or f"{self.mains_hz} Hz"
        lines.append(f"mains={hz} ({mains}) rejects={self.mains_rejects}")
        e = self.env
        valid = ",".join(e["valid"]) or "none"
        lines.append(
            f"ptc_left={e['ptc_left']} C ptc_right={e['ptc_right']} C chamber={e['chamber']} C"
            f" humidity={e['humidity']} % vdda={e['vdda']} V valid={valid}"
        )
        gcmd.respond_info("\n".join(lines))

    cmd_ACEK_RESTORE_STOCK_help = (
        "Reset the unit into its bootloader's recovery mode so the factory firmware can be "
        "restored over the wire (docs/flashing.md)"
    )

    def cmd_ACEK_RESTORE_STOCK(self, gcmd: GCodeCommand) -> None:
        cmd = self.mcu.lookup_query_command(
            "ace2k_bootloader_enter", "ace2k_bootloader_enter_response accepted=%c"
        )
        try:
            params = ask(self.printer, self.mcu, cmd, (), "bootloader enter")
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from None
        if not params["accepted"]:
            # This is an error and not a console line, because a macro must stop here. The unit
            # did not reset.
            raise gcmd.error("ace2k: refused — a subsystem is busy (heating or moving)")
        gcmd.respond_info(
            "ace2k: entering the bootloader's recovery mode; stop Klipper, then run the "
            "updater (docs/flashing.md)"
        )

    cmd_ACEK_CALIBRATION_SAVE_help = (
        "Stage the insert thresholds in use for SAVE_CONFIG, so they can be edited in printer.cfg"
    )

    def cmd_ACEK_CALIBRATION_SAVE(self, gcmd: GCodeCommand) -> None:
        if any(lane["low_mv"] is None for lane in self.lanes):
            raise gcmd.error("ace2k: thresholds not received yet")
        configfile = self.printer.lookup_object("configfile")
        for i, lane in enumerate(self.lanes):
            configfile.set(
                self.section, f"lane{i + 1}_insert_mv", f"{lane['low_mv']}, {lane['high_mv']}"
            )
        gcmd.respond_info("ace2k: thresholds staged; run SAVE_CONFIG to write them to printer.cfg")

    cmd_ACEK_COUNTERS_RESET_help = "Zero the encoder and tach counters: LANE=<1-4> or LANE=ALL"

    def cmd_ACEK_COUNTERS_RESET(self, gcmd: GCodeCommand) -> None:
        # Raise a gcmd.error, which Klipper reports to the console. An unguarded lookup would
        # raise the error of the message parser, and Klipper answers that with a shutdown.
        if "lane" not in self.present:
            raise gcmd.error("ace2k: firmware built without CONFIG_ACE2K_LANE")
        if self.feed.any_busy():
            raise gcmd.error(
                "ace2k: a lane is moving or busy; the counters are the moves' odometers"
            )
        lane = gcmd.get("LANE", "ALL").upper()
        # A value outside 1..4, or one that is not a number, is an error of gcmd itself, before
        # anything is sent.
        index = LANE_ALL if lane == "ALL" else gcmd.get_int("LANE", minval=1, maxval=LANES) - 1
        which = "all lanes" if index == LANE_ALL else f"lane {index + 1}"
        if not self.reset_confirmed:
            # An older firmware takes the reset without answering, so the reset is sent but not
            # confirmed. A firmware that is older still has no reset at all.
            cmd = self.reset_cmd
            if cmd is None:
                raise gcmd.error("ace2k: this firmware has no counters reset")
            cmd.send([index])
            self._drop_scale_changes(index)
            gcmd.respond_info(
                f"ace2k: counters reset sent ({which}); this firmware does not confirm it"
            )
            return
        # The unit also refuses on its own view of the lanes. This is the safety net for a move
        # that was accepted since the last report, or one that the unit started itself.
        assert self.reset_query is not None  # _build_commands built it, because reset_confirmed
        try:
            params = ask(self.printer, self.mcu, self.reset_query, [index], "counters reset")
        except NoAnswerError as e:
            raise gcmd.error(str(e)) from None
        if not params["accepted"]:
            raise gcmd.error("ace2k: counters reset refused: a lane is moving or busy")
        self._drop_scale_changes(index)
        gcmd.respond_info(f"ace2k: counters reset ({which})")


def load_config(config: Config) -> Ace2k:
    return Ace2k(config)


def load_config_prefix(config: Config) -> Ace2k:
    return Ace2k(config)
