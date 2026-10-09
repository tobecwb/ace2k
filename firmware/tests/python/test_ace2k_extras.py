"""ace2k.py: the link proof and the device identifier at MCU identify time, the init commands
and subscriptions (each only for a subsystem the firmware carries), the decoding of the reports,
the thresholds re-query, the health poll and its failure streak, the five G-codes — against a
fake printer."""

import importlib
import os
import re
import struct
import tempfile

import ace2k
import msgproto  # the conftest's stand-in: the parser's error class
import pytest
from ace2k import RESET_CMD, RESET_RESP, probe_format, refuse_format


class FakeQuery:
    """A response, or a list of responses handed out in order (the last one repeats); an
    exception instance among them is raised, as a query the MCU never answers would raise; a
    callable is called at send time (what happens while the query is in flight) and its return
    is the response.  send(data) records the name in the shared log and (name, data) in the
    MCU's query list, so a test can check the arguments."""

    def __init__(self, log, queries, msgformat, response, reactor=None):
        self.log = log
        self.queries = queries
        self.msgformat = msgformat
        self.response = response
        self.reactor = reactor

    def send(self, data=(), minclock=0, reqclock=0):
        if self.reactor is not None:
            self.reactor.check_pause()  # a query blocks: refused where Klipper refuses a pause
        self.log.append(self.msgformat)
        self.queries.append((self.msgformat, list(data)))
        response = self.response
        if isinstance(response, list):
            response = response.pop(0) if len(response) > 1 else response[0]
        if callable(response):
            response = response()
        if isinstance(response, BaseException):
            raise response
        return response


class FakeCommand:
    """A command without a response: send(data) records the name in the shared log (as a query
    does) and (format, data) in the MCU's command list, so a test can check the arguments."""

    def __init__(self, log, commands, msgformat):
        self.log = log
        self.commands = commands
        self.msgformat = msgformat

    def send(self, data=(), minclock=0, reqclock=0):
        self.log.append(self.msgformat.split()[0])
        self.commands.append((self.msgformat, list(data)))


DEFAULT_CONSTANTS = {
    "ACE2K_LANE_SPEED_MIN_UM_S": 9000,
    "ACE2K_LANE_SPEED_MAX_UM_S": 70000,
    "ACE2K_LANE_MOVE_MAX_UM": 2000000,
    "ACE2K_LANE_DUTY_MIN_PCT": 20,
    "ACE2K_LANE_LANDING_UM": 10000,
    "ACE2K_FEED_ASSIST_BURST_UM": 100000,
    "ACE2K_FEED_UNLOAD_TAIL_UM": 100000,
    "ACE2K_FEED_SNAG_FWD_UM": 20000,
    "ACE2K_FEED_SNAG_LAG_UM": 5000,
    "ACE2K_FEED_SNAG_DUTY_PCT": 0,
    "ACE2K_FEED_SNAG_BACK_UM": 5000,
    "ACE2K_FEED_SNAG_REST_MS": 500,
    "ACE2K_FEED_SNAG_FREE_UM": 50000,
}


class FakeMcu:
    """known: the command and response names the dictionary carries (None: every one), which
    drives lookup_command(), lookup_query_command(), try_lookup_command(),
    check_valid_response() and register_serial_response() as the real MCU's dictionary does;
    formats: name -> the format the dictionary carries for a name whose format differs from
    the one asked (the parser's "Command format mismatch"); lookup_errors: name -> what the
    lookup raises instead — a string is a msgproto.error with that text (a parser wording this
    module does not know), an exception instance is raised as it is (a failure that is not the
    dictionary's answer); constants: the dictionary's constants (the lane bounds by default)."""

    def __init__(self, responses, known=None, constants=None, formats=None, lookup_errors=None):
        self.sent = []
        self.commands = []
        self.queries = []
        self.responses = responses
        self.known = known
        self.formats = dict(formats or {})
        self.lookup_errors = dict(lookup_errors or {})
        self.constants = dict(constants or DEFAULT_CONSTANTS)
        self.config_cmds = []
        # two objects register a config callback on the same MCU (ace2k.py and ace2k_feed.py),
        # as Klipper allows; config_callback() runs them all
        self.config_callbacks = []
        self.subscriptions = {}
        self.subscription_formats = []
        self.reactor = None  # the printer's, set by FakePrinter: a query checks it may block

    def config_callback(self):
        for cb in self.config_callbacks:
            cb()

    def _known(self, msgformat):
        name = msgformat.split()[0]
        if self.known is not None and name not in self.known:
            return False
        if name in self.lookup_errors:
            return False
        have = self.formats.get(name)
        return have is None or have == msgformat

    def _lookup(self, msgformat):
        """As Klipper's message parser (msgproto.lookup_command), a msgproto.error: "Unknown
        command: <name>" for a name the dictionary lacks, "Command format mismatch: <asked> vs
        <have>" for a name it carries with another format; lookup_errors' entry for a name listed
        there."""
        name = msgformat.split()[0]
        if self.known is not None and name not in self.known:
            raise msgproto.error(f"Unknown command: {name}")
        if name in self.lookup_errors:
            failure = self.lookup_errors[name]
            raise failure if isinstance(failure, BaseException) else msgproto.error(failure)
        have = self.formats.get(name)
        if have is not None and have != msgformat:
            raise msgproto.error(f"Command format mismatch: {msgformat} vs {have}")

    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        # as Klipper's wrapper: the message parser raises on a command or a response the
        # dictionary lacks — from a G-code handler, that is a shutdown
        for fmt in (msgformat, respformat):
            self._lookup(fmt)
        name = msgformat.split()[0]
        response = self.responses[name]
        if isinstance(response, list):
            # one queue per command, shared by every wrapper looked up for it
            self.responses[name] = response = list(response)
            return FakeQuery(self.sent, self.queries, name, response, self.reactor)
        return FakeQuery(self.sent, self.queries, name, response, self.reactor)

    def lookup_command(self, msgformat, cq=None):
        """As Klipper's wrapper: the parser's error for a format the dictionary lacks or carries
        differently, else a command that records what it sends."""
        self._lookup(msgformat)
        return FakeCommand(self.sent, self.commands, msgformat)

    def try_lookup_command(self, msgformat):
        return self.lookup_command(msgformat) if self._known(msgformat) else None

    def check_valid_response(self, msgformat):
        return self._known(msgformat)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init))

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def register_serial_response(self, cb, msgformat, oid=None):
        # as Klipper's wrapper: a format the dictionary lacks raises at registration
        assert self._known(msgformat), msgformat
        self.subscriptions[msgformat.split()[0]] = cb
        self.subscription_formats.append(msgformat)

    def seconds_to_clock(self, t):
        return int(t * 120000000)

    def estimated_print_time(self, t):
        return t

    def get_constant_float(self, name):
        return float(self.constants[name])


class LegacyFakeMcu(FakeMcu):
    """A Klipper older than the pinned one, as Snapmaker's U1 runs: no register_serial_response,
    only register_response(cb, name, oid), which registers at once and checks nothing (the
    format check is the caller's)."""

    register_serial_response = None

    def register_response(self, cb, msg, oid=None):
        self.subscriptions[msg] = cb
        self.subscription_formats.append(msg)


class FakeMutex:
    """Klipper's reactor mutex as a context manager that cannot suspend a caller: it counts the
    ones that entered while another held it (where the real one queues them until the holder
    leaves) and how deep the nesting is at any moment."""

    def __init__(self):
        self.depth = 0
        self.contended = 0

    def test(self):
        return self.depth > 0

    def __enter__(self):
        if self.depth:
            self.contended += 1
        self.depth += 1

    def __exit__(self, exc_type, exc, tb):
        self.depth -= 1


class FakeReactor:
    """Runs an async callback at once, as if the reactor had picked it up; records the timers.
    As Klipper's reactor, a timer is registered once (at NEVER unless a wake time is given) and
    re-armed with update_timer(); armed() lists the ones with a wake time, in registration
    order, as (callback, waketime).  mutex() hands out a FakeMutex and keeps it in mutexes.
    while_waiting maps a completion to what the reactor would pick up while a waiter is parked
    on it — a serial frame, a timer — as callables of the wake time, run once each, in order, by
    a wait() on that completion that finds it pending; a wait on another completion leaves them
    alone."""

    NEVER = 9999999999999999.0

    def __init__(self):
        self.timers = []  # every registered timer: a mutable [callback, waketime] pair
        self.mutexes = []
        self.now = 100.0
        self.while_waiting = {}
        self.pause_disabled = False  # set while klippy:ready handlers run, as Klipper does

    def check_pause(self):
        """Klipper's reactor refuses a pause (a blocking query, a completion's wait) inside a
        klippy:ready handler: the same error here, so a handler that blocks fails its test."""
        if self.pause_disabled:
            raise RuntimeError("Internal error - reactor pause disabled")

    def mutex(self, is_locked=False):
        assert not is_locked
        m = FakeMutex()
        self.mutexes.append(m)
        return m

    def register_async_callback(self, cb, waketime=None):
        cb(0.0)

    def register_timer(self, cb, waketime=NEVER):
        timer = [cb, waketime]
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        assert timer in self.timers
        timer[1] = waketime

    def armed(self):
        return [(cb, waketime) for cb, waketime in self.timers if waketime != self.NEVER]

    def monotonic(self):
        return self.now

    def completion(self):
        return FakeCompletion(self)


class FakeCompletion:
    """Klipper's ReactorCompletion without a greenlet: wait() returns the result once complete()
    ran, else the default — the tests inject the event from the query's send() callable, or
    through the reactor's while_waiting entry for this completion, which a pending wait() drains
    before it gives up."""

    def __init__(self, reactor=None):
        self.reactor = reactor
        self.result = None
        self.done = False

    def test(self):
        return self.done

    def complete(self, result):
        self.result = result
        self.done = True

    def wait(self, waketime=None, waketime_result=None):
        if self.reactor is not None:
            self.reactor.check_pause()
        queued = self.reactor.while_waiting.get(self, []) if self.reactor is not None else []
        while not self.done and queued:
            queued.pop(0)(waketime)
        return self.result if self.done else waketime_result


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.output = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg):
        self.output.append(msg)


class FakeGcmd:
    """The output lines, and the parameters of the G-code line as Klipper's GCodeCommand hands
    them out: get() the raw string, get_int() / get_float() parsed and range-checked, raising
    gcmd.error; both return None for an absent key when the default is None."""

    error = Exception

    def __init__(self, params=None):
        self.lines = []
        self.params = params or {}

    def respond_info(self, msg):
        self.lines.append(msg)

    def respond_raw(self, msg):
        self.lines.append(msg)

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=None, minval=None, maxval=None):
        value = self.params.get(key, default)
        if value is None:
            return None
        try:
            value = int(value)
        except (TypeError, ValueError):
            raise self.error(f"Unable to parse '{key}' as an integer") from None
        if minval is not None and value < minval:
            raise self.error(f"Must specify {key} of at least {minval}")
        if maxval is not None and value > maxval:
            raise self.error(f"Must specify {key} of at most {maxval}")
        return value

    def get_float(self, key, default=None, minval=None, maxval=None, above=None, below=None):
        value = self.params.get(key, default)
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise self.error(f"Unable to parse '{key}' as a float") from None
        if minval is not None and value < minval:
            raise self.error(f"Must specify {key} of at least {minval}")
        if maxval is not None and value > maxval:
            raise self.error(f"Must specify {key} of at most {maxval}")
        if above is not None and value <= above:
            raise self.error(f"Must specify {key} above {above}")
        if below is not None and value >= below:
            raise self.error(f"Must specify {key} below {below}")
        return value


class FakeHeaters:
    def __init__(self):
        self.factories = {}

    def add_sensor_factory(self, name, factory):
        self.factories[name] = factory


class FakeConfigfile:
    def __init__(self):
        self.calls = []

    def set(self, section, option, value):
        self.calls.append((section, option, value))


class FakeConfig:
    error = Exception

    def __init__(self, printer, mcu_name=None, options=None):
        self.printer = printer
        self.mcu_name = mcu_name
        self.options = options or {}

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "ace2k"

    def get(self, key, default=None):
        if key == "mcu" and self.mcu_name is not None:
            return self.mcu_name
        return default

    def getintlist(self, key, default=None, sep=",", count=None):
        value = self.options.get(key)
        if value is None:
            return default
        assert count is None or len(value) == count
        return list(value)

    def getint(self, key, default=None, minval=None, maxval=None):
        value = self.options.get(key, default)
        if value is None:
            return None
        value = int(value)
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise self.error(f"{key} out of range")
        return value

    def getfloat(self, key, default=None, minval=None, maxval=None, above=None, below=None):
        value = self.options.get(key, default)
        if value is None:
            return None
        value = float(value)
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise self.error(f"{key} out of range")
        if (above is not None and value <= above) or (below is not None and value >= below):
            raise self.error(f"{key} out of range")
        return value

    def getboolean(self, key, default=None):
        value = self.options.get(key, default)
        return None if value is None else bool(value)

    def getsection(self, name):
        # every section is this one: the options of [ace2k] are what the fake carries
        return self


class FakeCommandError(Exception):
    """The printer's command_error: what a query the MCU does not answer raises."""


class FakePrinter:
    command_error = FakeCommandError
    config_error = Exception

    def __init__(self, mcu):
        self.objects = {
            "mcu ace2k": mcu,
            "gcode": FakeGcode(),
            "heaters": FakeHeaters(),
            "configfile": FakeConfigfile(),
        }
        self.handlers = {}
        self.events = []
        self.reactor = FakeReactor()
        self.start_dir = tempfile.mkdtemp(prefix="ace2k-test-")
        mcu.reactor = self.reactor
        self.handler_lists = {}

    def send_event(self, event, *params):
        self.events.append((event, params))

    def add_object(self, name, obj):
        self.objects[name] = obj

    _REQUIRED = object()

    def lookup_object(self, name, default=_REQUIRED):
        # as Klipper's: a missing name raises (KeyError here) unless a default is given
        if default is self._REQUIRED or name in self.objects:
            return self.objects[name]
        return default

    def load_object(self, config, name):
        if name not in self.objects:
            self.objects[name] = importlib.import_module(name).load_config(config)
        return self.objects[name]

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, func):
        # Klipper runs every handler of an event, in registration order: [ace2k] and the
        # modules it loads (ace2k_rfid) both handle klippy:connect and klippy:ready
        fns = self.handler_lists.setdefault(event, [])
        fns.append(func)
        self.handlers[event] = lambda fns=fns, event=event: self._run_handlers(event, fns)

    def _run_handlers(self, event, fns):
        # Klipper disables the reactor's pause while the klippy:ready handlers run
        self.reactor.pause_disabled = event == "klippy:ready"
        try:
            return [f() for f in fns]
        finally:
            self.reactor.pause_disabled = False

    def lookup_objects(self, module=None):
        """As Klipper's: (name, object) for every object whose name is `module` or starts with
        `module + " "`."""
        return [
            (name, obj)
            for name, obj in self.objects.items()
            if module is None or name == module or name.startswith(module + " ")
        ]

    def get_start_args(self):
        return {
            "config_file": os.path.join(self.start_dir, "printer.cfg"),
            "log_file": os.path.join(self.start_dir, "klippy.log"),
        }


def make(
    responses,
    options=None,
    known=None,
    constants=None,
    formats=None,
    lookup_errors=None,
    mcu_class=None,
):
    mcu = (mcu_class or FakeMcu)(responses, known, constants, formats, lookup_errors)
    printer = FakePrinter(mcu)
    module = ace2k.load_config(FakeConfig(printer, options=options))
    return module, printer, mcu


def thresholds_frame(lows, highs, sources):
    """One ace2k_sensors_thresholds frame: four uint16 LE, four uint16 LE, four bytes."""
    return {
        "low_mv": struct.pack("<4H", *lows),
        "high_mv": struct.pack("<4H", *highs),
        "source": bytes(sources),
    }


def lane_counters_frame(encoder_um, fg):
    """One ace2k_lane_counters frame: four int32 LE, four uint32 LE."""
    return {"encoder_um": struct.pack("<4i", *encoder_um), "fg": struct.pack("<4I", *fg)}


def fire_ready(printer):
    """klippy:ready, then what the reactor runs next: the one-shot timers due 0.1 s later —
    ace2k_rfid's lane seeding and ace2k_dryer's log read — before any poll, so the timers left
    armed are [ace2k]'s own."""
    printer.handlers["klippy:ready"]()
    seeds = []
    rfid = printer.objects.get("ace2k_rfid")
    if rfid is not None:
        seeds.append(rfid._seed_lanes)
    dryer = printer.objects.get("ace2k_dryer")
    if dryer is not None:
        seeds.append(dryer._seed_log)
    for timer in printer.reactor.timers:
        if timer[0] in seeds and timer[1] != FakeReactor.NEVER:
            timer[1] = timer[0](timer[1])


def armed_timer(printer, cb):
    """The (callback, waketime) of the one armed timer with that callback."""
    (timer,) = [t for t in printer.reactor.armed() if t[0] == cb]
    return timer


SYNTHETIC_UID = bytes(range(0x11, 0x11 + 12))  # any 12 bytes; no real unit's
HEALTHY = {"now": 0, "latched": 0, "post_ms": 2010, "reset_cause": 1}
RESPONSES = {
    "ace2k_version": {"version": b"0.1.0"},
    "ace2k_linkproof_state": {"proven": 1, "remaining_ms": 0},
    "ace2k_bootloader_enter": {"accepted": 1},
    "ace2k_uid_query": {"uid": SYNTHETIC_UID},
    "ace2k_health_query": HEALTHY,
    "ace2k_lane_counters_reset": {"lane": 255, "accepted": 1},
    "ace2k_rfid_lane_query": {"lane": 0, "state": 0, "uid": b""},
    "ace2k_dryer_log_query": {"index": 0, "a": 0, "b": 0, "c": 0, "d": 0, "e": 0, "f": 0},
}
RFID_WARNINGS = (
    "CONFIG_ACE2K_RFID_READ",
    "[ace2k_tag]",
    "rfid_search_mm",
    "tag cache",
    "tag dump",
    # ace2k_dryer's own (a firmware without the heat, or without the dryer)
    "CONFIG_ACE2K_HEAT",
    "CONFIG_ACE2K_DRYER",
)
# The prefixes of the names the modules [ace2k] loads send and subscribe on their own
OTHER_MODULES = (
    "ace2k_rfid_",
    "ace2k_dryer_",
    "ace2k_airflow_",
    "ace2k_fan_",
    "ace2k_flap_",
)


def non_rfid_warnings(caplog):
    """The WARNING messages, ace2k_rfid's own left out (its firmware and section checks)."""
    return [
        r.getMessage()
        for r in caplog.records
        if r.levelname == "WARNING" and not any(w in r.getMessage() for w in RFID_WARNINGS)
    ]


def non_rfid_sent(mcu):
    """The names the MCU was sent, ace2k_rfid's and ace2k_dryer's own left out (their queries
    after ready)."""
    return [name for name in mcu.sent if not name.startswith(OTHER_MODULES)]


# The dictionary of a firmware with every subsystem but health (test_a_firmware_without_*).
KNOWN_ALL_BUT_HEALTH = {
    "ace2k_version",
    "ace2k_version_response",
    "ace2k_linkproof_state",
    "ace2k_linkproof_state_response",
    "ace2k_sensors_query",
    "ace2k_sensors_state",
    "ace2k_sensors_event",
    "ace2k_sensors_thresholds",
    "ace2k_sensors_thresholds_query",
    "ace2k_env_query",
    "ace2k_env_state",
    "ace2k_lane_counters_query",
    "ace2k_lane_counters",
    "ace2k_lane_counters_reset",
    "ace2k_lane_counters_reset_response",
    "ace2k_mains_query",
    "ace2k_mains_state",
}


def test_mcu_identify_queries_version_then_state_then_uid():
    module, printer, mcu = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    assert non_rfid_sent(mcu) == ["ace2k_version", "ace2k_linkproof_state", "ace2k_uid_query"]
    status = module.get_status(0.0)
    assert (status["version"], status["link_proven"]) == ("0.1.0", True)


def test_status_gcode_reports_version_and_proof():
    module, printer, _ = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_STATUS"](gcmd)
    assert gcmd.lines[0].startswith("ace2k 0.1.0, link proven: yes")


def test_restore_stock_sends_bootloader_enter():
    module, printer, mcu = make(RESPONSES)
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_RESTORE_STOCK"](gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_bootloader_enter"]
    assert gcmd.lines[0].startswith("ace2k: entering the bootloader")


def test_restore_stock_reports_a_refusal():
    responses = dict(RESPONSES)
    responses["ace2k_bootloader_enter"] = {"accepted": 0}
    module, printer, mcu = make(responses)
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_RESTORE_STOCK"](gcmd)
    assert gcmd.lines == ["!! ace2k: refused — a subsystem is busy (heating or moving)"]


def test_every_gcode_name_is_letters_and_underscores_only():
    # Klipper's gcode.py reads a command name as [A-Z_]+ (or the G1/M104 form): a digit ends the
    # name, so ACE2K_STATUS would be dispatched as "ACE2" — hence ACE_* while the section is [ace2k]
    _, printer, _ = make(RESPONSES)
    names = list(printer.objects["gcode"].commands)
    assert names == [  # the feed's fifteen register while [ace2k] loads ace2k_feed
        "ACE_STATUS",
        "ACE_RESTORE_STOCK",
        "ACE_FEED",
        "ACE_ROLLBACK",
        "ACE_UNLOAD",
        "ACE_LOAD",
        "ACE_ASSIST",
        "ACE_STOP",
        "ACE_CLEAR",
        "ACE_SPEED",
        "ACE_CALIBRATE_ENCODER",
        "ACE_LOAD_SET",
        "ACE_SNAG_SET",
        "ACE_FOLLOW_SET",
        "ACE_FF_SET",
        "ACE_FEED_FORWARD",
        "ACE_GRIP",
        "ACE_RFID_READ",
        "ACE_RFID_FORGET",
        "ACE_DRY",
        "ACE_DRY_STOP",
        "ACE_DRY_CLEAR",
        "ACE_FAN",
        "ACE_FLAP",
        "ACE_DRYER_LOG",
        "ACE_CALIBRATION_SAVE",
        "ACE_COUNTERS_RESET",
        "ACE_HEALTH",
    ]
    for name in names:
        assert re.fullmatch(r"[A-Z_]+", name), name


def test_an_unproven_link_is_retried_once_and_can_succeed():
    responses = dict(RESPONSES)
    responses["ace2k_linkproof_state"] = [
        {"proven": 0, "remaining_ms": 120000},
        {"proven": 1, "remaining_ms": 0},
    ]
    module, printer, mcu = make(responses)
    printer.handlers["klippy:mcu_identify"]()
    assert non_rfid_sent(mcu) == [
        "ace2k_version",
        "ace2k_linkproof_state",
        "ace2k_version",
        "ace2k_linkproof_state",
        "ace2k_uid_query",
    ]
    assert module.get_status(0.0)["link_proven"] is True


def test_a_link_still_unproven_after_the_retry_is_reported_and_warned_about(caplog):
    responses = dict(RESPONSES)
    responses["ace2k_linkproof_state"] = {"proven": 0, "remaining_ms": 120000}
    module, printer, mcu = make(responses)
    with caplog.at_level("WARNING", logger="root"):
        printer.handlers["klippy:mcu_identify"]()
    assert len(mcu.sent) == 5  # two proofs, the UID
    assert module.get_status(0.0)["link_proven"] is False
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1
    assert "0.1.0" in warnings[0]
    assert "120000" in warnings[0]
    assert "recovery" in warnings[0]


def test_the_primary_mcu_is_looked_up_without_a_suffix():
    mcu = FakeMcu(RESPONSES)
    printer = FakePrinter(mcu)
    printer.objects["mcu"] = mcu
    module = ace2k.load_config(FakeConfig(printer, mcu_name="mcu"))
    printer.handlers["klippy:mcu_identify"]()
    assert module.get_status(0.0)["version"] == "0.1.0"


def test_build_config_sends_thresholds_and_queries():
    module, printer, mcu = make(RESPONSES, options={"lane2_insert_mv": [700, 1300]})
    mcu.config_callback()
    # a config command (it enters Klipper's config checksum, so a removal restarts the MCU),
    # not an init command
    config_cmds = [c for c, is_init in mcu.config_cmds if not is_init]
    assert config_cmds == ["ace2k_sensors_thresholds_set lane=1 low_mv=700 high_mv=1300"]
    init_cmds = [c for c, is_init in mcu.config_cmds if is_init and not c.startswith(OTHER_MODULES)]
    assert not any(c.startswith("ace2k_sensors_thresholds_set") for c in init_cmds)
    assert "ace2k_sensors_thresholds_query" not in init_cmds  # asked at connect instead
    assert init_cmds == [  # the health query is polled from the host, not an init command
        # the feed's (ace2k_feed.py registers its config callback first, while [ace2k] loads it)
        "ace2k_lane_scale_set lane=0 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=1 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=2 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=3 um_per_count_x10=12342",
        "ace2k_feed_load_set park_um=300000 speed_um_s=30000 auto_load=1",
        "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k_feed_query rest_ticks=120000000",
        "ace2k_sensors_query rest_ticks=12000000",
        "ace2k_env_query rest_ticks=120000000",
        "ace2k_lane_counters_query rest_ticks=120000000",
        "ace2k_mains_query rest_ticks=120000000",
    ]
    assert module.present == {"sensors", "env", "lane", "mains", "health"}
    assert {n for n in mcu.subscriptions if not n.startswith(OTHER_MODULES)} == {
        "ace2k_sensors_state",
        "ace2k_sensors_event",
        "ace2k_env_state",
        "ace2k_sensors_thresholds",
        "ace2k_lane_counters",
        "ace2k_mains_state",
        "ace2k_feed_state",
        "ace2k_feed_event",
    }


def test_subscriptions_use_the_packed_per_module_formats():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert "ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s" in mcu.subscription_formats
    assert "ace2k_lane_counters encoder_um=%*s fg=%*s" in mcu.subscription_formats
    assert not any(
        "lane=%c encoder_um" in f or "thresholds lane=%c" in f for f in mcu.subscription_formats
    )


def test_connect_asks_for_the_thresholds_once_the_subscriptions_are_live():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"]
    # and arms the one bounded re-query 1.5 s on
    assert len(printer.reactor.armed()) == 1
    assert printer.reactor.armed()[0][1] == printer.reactor.now + 1.5


def test_a_lost_thresholds_frame_is_asked_for_once_more_then_warned_about(caplog):
    # the answer is a report frame, not a query response: nothing times out when it is lost
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(101.5) == 103.0  # still None: one re-query, the timer kept for the check
        assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"] * 2
        assert cb(103.0) == FakeReactor.NEVER  # still None: the warning, no third query
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"] * 2
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "thresholds did not arrive" in warnings[0]
    assert module.get_status(0.0)["lanes"][0]["low_mv"] is None


def test_a_thresholds_frame_that_arrives_means_no_re_query(caplog):
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    mcu.subscriptions["ace2k_sensors_thresholds"](thresholds_frame([600] * 4, [1100] * 4, [1] * 4))
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(101.5) == FakeReactor.NEVER
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"]  # the one from connect
    assert not non_rfid_warnings(caplog)


def test_a_thresholds_frame_that_arrives_after_the_re_query_ends_it_quietly(caplog):
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    cb, _ = printer.reactor.armed()[0]
    assert cb(101.5) == 103.0  # the re-query
    mcu.subscriptions["ace2k_sensors_thresholds"](thresholds_frame([600] * 4, [1100] * 4, [1] * 4))
    with caplog.at_level("WARNING", logger="root"):
        assert cb(103.0) == FakeReactor.NEVER
    assert not non_rfid_warnings(caplog)
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"] * 2


def test_the_thresholds_arrive_as_one_frame_for_the_four_lanes():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    mcu.subscriptions["ace2k_sensors_thresholds"](
        thresholds_frame([600, 700, 580, 3000], [1100, 1300, 1250, 3300], [1, 2, 1, 0])
    )
    lanes = module.get_status(0.0)["lanes"]
    assert [(lane["low_mv"], lane["high_mv"], lane["source"]) for lane in lanes] == [
        (600, 1100, "factory"),
        (700, 1300, "config"),
        (580, 1250, "factory"),
        (3000, 3300, "default"),
    ]


def test_a_firmware_without_lane_and_mains_is_logged_and_left_out(caplog):
    known = {
        "ace2k_sensors_query",
        "ace2k_sensors_state",
        "ace2k_sensors_event",
        "ace2k_sensors_thresholds",
        "ace2k_sensors_thresholds_query",
        "ace2k_env_query",
        "ace2k_env_state",
        "ace2k_health_query",
    }
    module, printer, mcu = make(RESPONSES, known=known)
    with caplog.at_level("WARNING", logger="root"):
        mcu.config_callback()
    init_cmds = [c for c, is_init in mcu.config_cmds if is_init]
    assert init_cmds == [
        "ace2k_sensors_query rest_ticks=12000000",
        "ace2k_env_query rest_ticks=120000000",
    ]
    assert not any(c.startswith("ace2k_lane_counters_query") for c in init_cmds)
    assert set(mcu.subscriptions) == {
        "ace2k_sensors_state",
        "ace2k_sensors_event",
        "ace2k_sensors_thresholds",
        "ace2k_env_state",
    }
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 3  # the feed (absent from `known` too) warns first: see ace2k_feed.py
    assert "CONFIG_ACE2K_FEED" in warnings[0]
    assert "CONFIG_ACE2K_LANE" in warnings[1] and "lane disabled" in warnings[1]
    assert "CONFIG_ACE2K_MAINS" in warnings[2] and "mains disabled" in warnings[2]
    printer.handlers["klippy:connect"]()  # the thresholds query still goes: sensors is present
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"]
    st = module.get_status(0.0)
    assert (st["mains_hz"], st["mains_present"]) == (None, None)
    assert st["lanes"][0]["encoder_mm"] is None and st["lanes"][0]["fg"] is None


def test_a_firmware_without_sensors_sends_no_thresholds_and_no_connect_query(caplog):
    known = {
        "ace2k_env_query",
        "ace2k_env_state",
        "ace2k_lane_counters_query",
        "ace2k_lane_counters",
        "ace2k_lane_counters_reset",
        "ace2k_lane_counters_reset_response",
        "ace2k_mains_query",
        "ace2k_mains_state",
        "ace2k_health_query",
    }
    module, printer, mcu = make(RESPONSES, options={"lane2_insert_mv": [700, 1300]}, known=known)
    with caplog.at_level("WARNING", logger="root"):
        mcu.config_callback()
    assert not any(c.startswith("ace2k_sensors") for c, _ in mcu.config_cmds)
    assert "ace2k_sensors_thresholds" not in mcu.subscriptions
    printer.handlers["klippy:connect"]()
    assert non_rfid_sent(mcu) == []
    assert printer.reactor.armed() == []  # no re-query timer either
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 2  # the feed's (absent from `known` too), then sensors
    assert "CONFIG_ACE2K_FEED" in warnings[0] and "CONFIG_ACE2K_SENSORS" in warnings[1]


def test_sensors_state_decodes_switches_and_millivolts():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    mcu.subscriptions["ace2k_sensors_state"](
        {
            "switches": 0b10000100010001,
            "insert_mv": b"\x20\x03" + b"\x00" * 6,
            "empty_mv": b"\x00" * 8,
            "aux_mv": 1280,
            "age_ms": 3,
            "#sent_time": 1.0,
        }
    )
    st = module.get_status(0.0)
    lane = st["lanes"][0]
    assert (lane["insert"], lane["insert_mv"], lane["rest"], lane["pushed"]) == (
        True,
        800,
        True,
        True,
    )
    assert lane["empty_mv"] == 0
    assert st["lanes"][1]["insert"] is False
    assert st["pulled_any"] is False and st["cutout"] is True and st["aux_mv"] == 1280
    assert st["sensors_age_ms"] == 3


def test_env_state_converts_units_and_notifies_listeners():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    seen = []
    module.register_env_listener(lambda pt, env: seen.append((pt, env)))
    mcu.subscriptions["ace2k_env_state"](
        {
            "ptc_left_mc": 25000,
            "ptc_right_mc": 24500,
            "chamber_mc": 23480,
            "chamber_rh_pct10": 670,
            "vdda_mv": 3301,
            "valid": 0xF,
            "#sent_time": 2.5,
        }
    )
    st = module.get_status(0.0)
    assert (st["ptc_left"], st["ptc_right"], st["chamber"], st["humidity"]) == (
        25.0,
        24.5,
        23.48,
        67.0,
    )
    assert st["vdda"] == 3.301 and st["env_valid"] == ["ntc_left", "ntc_right", "chamber", "vdda"]
    assert seen and seen[0][0] == 2.5 and seen[0][1]["chamber"] == 23.48


def test_humidity_is_none_while_the_chamber_bit_is_clear():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    seen = []
    module.register_env_listener(lambda pt, env: seen.append(env))
    frame = {
        "ptc_left_mc": 25000,
        "ptc_right_mc": 24500,
        "chamber_mc": 23480,
        "chamber_rh_pct10": 670,  # the last value, carried in an invalid frame
        "vdda_mv": 3301,
        "valid": 0xB,  # ntc_left, ntc_right, vdda — no chamber
        "#sent_time": 2.5,
    }
    mcu.subscriptions["ace2k_env_state"](frame)
    st = module.get_status(0.0)
    assert st["humidity"] is None and st["env_valid"] == ["ntc_left", "ntc_right", "vdda"]
    assert seen[-1]["humidity"] is None
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_STATUS"](gcmd)
    assert "humidity=None %" in gcmd.lines[0]
    frame["valid"] = 0xF
    mcu.subscriptions["ace2k_env_state"](frame)
    assert module.get_status(0.0)["humidity"] == 67.0


def test_lane_counters_and_mains_land_in_status():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    before = module.get_status(0.0)
    assert (before["lanes"][2]["encoder_mm"], before["lanes"][2]["fg"]) == (None, None)
    assert (before["mains_hz"], before["mains_present"]) == (None, None)
    assert before["mains_rejects"] is None
    mcu.subscriptions["ace2k_lane_counters"](
        lane_counters_frame([-1234, 0, 99970, 2**31 - 1], [2470, 0, 0, 2**32 - 1])
    )
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 600, "present": 1, "rejects": 0})
    st = module.get_status(0.0)
    assert st["lanes"][2]["encoder_um"] == 99970
    assert st["lanes"][2]["encoder_mm"] == 99.97
    assert st["lanes"][2]["fg"] == 0
    assert (st["lanes"][0]["encoder_mm"], st["lanes"][0]["fg"]) == (-1.234, 2470)
    assert (st["lanes"][1]["encoder_mm"], st["lanes"][1]["fg"]) == (0.0, 0)
    assert (st["lanes"][3]["encoder_um"], st["lanes"][3]["fg"]) == (2**31 - 1, 2**32 - 1)
    assert st["mains_hz"] == 60.0 and st["mains_present"] is True
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 0, "present": 0, "rejects": 7})
    st = module.get_status(0.0)
    assert st["mains_hz"] == 0.0 and st["mains_present"] is False
    assert st["mains_rejects"] == 7 and st["mains_note"] is None
    # the markers are no frequency: named, never 0.1 or 0.2 Hz
    for hz10, note in ((1, "missed edge"), (2, "stale")):
        mcu.subscriptions["ace2k_mains_state"]({"hz10": hz10, "present": 1, "rejects": 7})
        st = module.get_status(0.0)
        assert st["mains_hz"] is None and st["mains_note"] == note
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 600, "present": 1, "rejects": 7})
    st = module.get_status(0.0)
    assert st["mains_hz"] == 60.0 and st["mains_note"] is None


def test_counters_reset_gcode():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()  # the subsystems are known from the config callback on
    reset = printer.objects["gcode"].commands["ACE_COUNTERS_RESET"]
    gcmd = FakeGcmd({"LANE": "2"})
    reset(gcmd)
    assert mcu.queries == [("ace2k_lane_counters_reset", [1])]
    assert gcmd.lines == ["ace2k: counters reset (lane 2)"]
    gcmd = FakeGcmd()  # no LANE: every lane
    reset(gcmd)
    assert mcu.queries[-1] == ("ace2k_lane_counters_reset", [255])
    assert gcmd.lines == ["ace2k: counters reset (all lanes)"]
    gcmd = FakeGcmd({"LANE": "all"})  # case does not matter
    reset(gcmd)
    assert mcu.queries[-1] == ("ace2k_lane_counters_reset", [255])
    with pytest.raises(FakeGcmd.error):
        reset(FakeGcmd({"LANE": "5"}))
    with pytest.raises(FakeGcmd.error):
        reset(FakeGcmd({"LANE": "0"}))
    assert len(mcu.queries) == 3  # nothing sent for a bad lane


def test_counters_reset_refused_by_the_unit_is_a_gcode_error():
    # the host's own picture is a 1 Hz report: a move the unit accepted since, or started
    # itself, is only known to the unit, which answers accepted=0 and changes nothing
    responses = dict(RESPONSES)
    responses["ace2k_lane_counters_reset"] = {"lane": 1, "accepted": 0}
    module, printer, mcu = make(responses)
    mcu.config_callback()
    assert module.reset_confirmed
    gcmd = FakeGcmd({"LANE": "2"})
    with pytest.raises(FakeGcmd.error, match="counters reset refused: a lane is moving or busy"):
        printer.objects["gcode"].commands["ACE_COUNTERS_RESET"](gcmd)
    assert mcu.queries == [("ace2k_lane_counters_reset", [1])]
    assert gcmd.lines == []  # no "counters reset" line for a refusal


def test_counters_reset_on_a_firmware_without_the_response_is_sent_unconfirmed(caplog):
    # a v0.2.0 dictionary: the command, no response — a query wrapper on it would raise the
    # parser's error inside the G-code handler, which Klipper answers with a shutdown; the
    # absence is known at config time, logged, and the reset goes out as a plain command
    known = set(KNOWN_ALL_BUT_HEALTH) - {RESET_RESP.split()[0]}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    assert "answers no counters reset; ACE_COUNTERS_RESET is sent unconfirmed" in caplog.text
    gcmd = FakeGcmd({"LANE": "2"})
    printer.objects["gcode"].commands["ACE_COUNTERS_RESET"](gcmd)
    assert mcu.queries == [] and mcu.commands == [(RESET_CMD, [1])]
    assert gcmd.lines == ["ace2k: counters reset sent (lane 2); this firmware does not confirm it"]


def test_counters_reset_response_with_another_format_is_a_config_error(caplog):
    # the name is there but its format is not this module's: a firmware the host does not
    # know, refused at connect (as a subscribed response with another format is) — not the
    # unconfirmed fallback, which is for a firmware without the response
    have = "ace2k_lane_counters_reset_response lane=%c"
    module, printer, mcu = make(
        RESPONSES, known=KNOWN_ALL_BUT_HEALTH, formats={RESET_RESP.split()[0]: have}
    )
    with pytest.raises(Exception, match="ace2k_lane_counters_reset_response.*format mismatch"):
        mcu.config_callback()
    assert "sent unconfirmed" not in caplog.text


def test_a_failure_that_is_not_the_dictionarys_answer_propagates_from_the_config_callback():
    # probe_format reads msgproto.error only: a serial-state error or an API change out of
    # lookup_command must not be read as "older firmware" and disable a subsystem with a warning
    module, printer, mcu = make(
        RESPONSES,
        known=KNOWN_ALL_BUT_HEALTH,
        lookup_errors={RESET_RESP.split()[0]: RuntimeError("serial not connected")},
    )
    with pytest.raises(RuntimeError, match="serial not connected"):
        mcu.config_callback()
    with pytest.raises(RuntimeError):
        probe_format(mcu, RESET_RESP)


def test_a_report_with_another_format_is_a_config_error_and_an_absent_one_a_warning(caplog):
    counters = "ace2k_lane_counters encoder_um=%*s fg=%*s"
    module, printer, mcu = make(
        RESPONSES,
        known=KNOWN_ALL_BUT_HEALTH,
        formats={"ace2k_lane_counters": "ace2k_lane_counters fg=%*s"},
    )
    with pytest.raises(
        Exception, match="ace2k_lane_counters has another format.*different commits"
    ):
        mcu.config_callback()
    # the lane's query is there but this report is not: logged with the parser's text, nothing
    # registered, the fields it would fill stay None
    known = set(KNOWN_ALL_BUT_HEALTH) - {"ace2k_lane_counters"}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert "lane" in module.present and "ace2k_lane_counters" not in mcu.subscriptions
    assert "carries no ace2k_lane_counters" in caplog.text
    assert "Unknown command: ace2k_lane_counters" in caplog.text
    assert module.get_status(0.0)["lanes"][0]["encoder_mm"] is None
    assert counters.split()[0] not in mcu.subscriptions


def test_refuse_format_returns_the_error_and_words_each_verdict():
    # returned, never raised: every call site reads "raise refuse_format(...)"
    printer = FakePrinter(FakeMcu(RESPONSES))
    err = refuse_format(printer, "ace2k", RESET_RESP, "Command format mismatch: a vs b")
    assert isinstance(err, printer.config_error)
    assert str(err) == (
        "ace2k: ace2k_lane_counters_reset_response has another format on the unit (Command"
        " format mismatch: a vs b); " + ace2k.FORMAT_HINT
    )
    # absent, with the companion that is there named …
    err = refuse_format(
        printer,
        "ace2k",
        "ace2k_feed_state mode=%*s",
        "Unknown command: x",
        "absent",
        "ace2k_feed_start lane=%c",
    )
    assert str(err) == (
        "ace2k: ace2k_feed_state is not in the unit's dictionary though ace2k_feed_start is"
        " (Unknown command: x); " + ace2k.FORMAT_HINT
    )
    # … and without one: the clause is dropped, nothing dereferenced
    err = refuse_format(
        printer, "ace2k", "ace2k_feed_state mode=%*s", "Unknown command: x", "absent"
    )
    assert str(err) == (
        "ace2k: ace2k_feed_state is not in the unit's dictionary (Unknown command: x); "
        + ace2k.FORMAT_HINT
    )


def test_probe_format_tells_ok_differs_and_absent_and_defaults_to_absent():
    name = RESET_RESP.split()[0]
    assert probe_format(FakeMcu(RESPONSES), RESET_RESP) == ("ok", "")
    verdict, text = probe_format(FakeMcu(RESPONSES, known=set()), RESET_RESP)
    assert verdict == "absent" and text == f"Unknown command: {name}"
    verdict, text = probe_format(FakeMcu(RESPONSES, formats={name: f"{name} lane=%c"}), RESET_RESP)
    assert verdict == "differs" and text.startswith("Command format mismatch: ")
    # a wording this module does not know is the benign case, never a refusal
    verdict, text = probe_format(
        FakeMcu(RESPONSES, lookup_errors={name: "No such message in this dictionary"}), RESET_RESP
    )
    assert (verdict, text) == ("absent", "No such message in this dictionary")


def test_an_unrecognised_lookup_failure_falls_back_to_the_unconfirmed_reset(caplog):
    # a Klipper rewording of "Unknown command" must not block a connect that worked before: the
    # fallback, with the parser's text in the warning so the rewording is visible
    module, printer, mcu = make(
        RESPONSES,
        known=KNOWN_ALL_BUT_HEALTH,
        lookup_errors={RESET_RESP.split()[0]: "Nothing named that here"},
    )
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    assert "ACE_COUNTERS_RESET is sent unconfirmed (Nothing named that here)" in caplog.text
    gcmd = FakeGcmd({"LANE": "2"})
    printer.objects["gcode"].commands["ACE_COUNTERS_RESET"](gcmd)
    assert mcu.commands == [(RESET_CMD, [1])]


def test_counters_reset_on_a_firmware_without_the_command_is_a_gcode_error():
    # older still: lane counters, but no reset at all — a G-code error, never the parser's
    # exception out of the handler
    known = set(KNOWN_ALL_BUT_HEALTH) - {RESET_CMD.split()[0], RESET_RESP.split()[0]}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    with pytest.raises(FakeGcmd.error, match="this firmware has no counters reset"):
        printer.objects["gcode"].commands["ACE_COUNTERS_RESET"](FakeGcmd({"LANE": "2"}))
    assert mcu.commands == [] and mcu.queries == []


def test_counters_reset_is_refused_when_the_firmware_has_no_lane_counters():
    known = {"ace2k_sensors_query", "ace2k_env_query", "ace2k_mains_query"}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    # a gcmd.error, not a bare msgparser error: the latter is not a CommandError, and Klipper
    # answers one from a G-code handler with a shutdown
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_LANE"):
        printer.objects["gcode"].commands["ACE_COUNTERS_RESET"](FakeGcmd({"LANE": "2"}))
    assert mcu.commands == [] and mcu.queries == [] and mcu.sent == []


def test_each_lane_counters_frame_reaches_the_listeners_with_its_receive_time():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    heard = []
    module.register_counters_listener(lambda rt, encoder_um: heard.append((rt, encoder_um)))
    frame = lane_counters_frame([1000, -2000, 0, 7], [0] * 4)
    mcu.subscriptions["ace2k_lane_counters"](dict(frame, **{"#receive_time": 12.5}))
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1, 2, 3, 4], [0] * 4))
    assert heard == [(12.5, (1000, -2000, 0, 7)), (0.0, (1, 2, 3, 4))]  # none in the frame: 0.0


def test_the_feed_holds_the_unit_and_its_calibration_hears_the_counters():
    module, printer, mcu = make(RESPONSES)
    assert module.feed.unit is module
    assert module.counter_listeners == [module.feed.cal.on_counters]


def test_set_counters_period_sends_the_lane_counters_query_when_the_firmware_has_it():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    module.set_counters_period(0.1)
    queries = [c for c in mcu.commands if c[0].startswith("ace2k_lane_counters_query")]
    assert queries == [("ace2k_lane_counters_query rest_ticks=%u", [12000000])]
    module, printer, mcu = make(RESPONSES, known={"ace2k_sensors_query", "ace2k_env_query"})
    mcu.config_callback()
    module.set_counters_period(0.1)  # a firmware without the lane counters: nothing to set
    assert mcu.commands == [] and mcu.sent == []


def test_a_live_scale_change_keeps_encoder_mm_continuous():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    module.keep_encoder_continuous(0, 1.2342, 1.2468)  # no reading yet: nothing to keep
    assert module.scale_pending == [None] * 4
    counters(lane_counters_frame([1000000, 0, 0, 0], [0] * 4))  # at 1.2342 mm/count
    module.keep_encoder_continuous(0, 1.2342, 1.2468)
    assert module.encoder_offset_um == [0, 0, 0, 0]  # until a frame at the new scale
    # the unit converts its whole count with the new scale: the raw reading jumps by 1 %
    raw = round(1000000 * 1.2468 / 1.2342)
    counters(lane_counters_frame([raw, 0, 0, 0], [0] * 4))
    assert module.encoder_offset_um == [-10209, 0, 0, 0] and module.scale_pending == [None] * 4
    lane = module.get_status(0.0)["lanes"][0]
    assert (lane["encoder_um"], lane["encoder_mm"]) == (raw, 1000.0)
    counters(lane_counters_frame([raw + 1247, 0, 0, 0], [0] * 4))  # one count on, at 1.2468
    lane = module.get_status(0.0)["lanes"][0]
    assert lane["encoder_mm"] == pytest.approx(1001.2468, abs=0.001)
    # back to the old scale: the offsets add up to none
    counters(lane_counters_frame([raw, 0, 0, 0], [0] * 4))
    module.keep_encoder_continuous(0, 1.2468, 1.2342)
    counters(lane_counters_frame([1000000, 0, 0, 0], [0] * 4))
    assert module.encoder_offset_um == [0, 0, 0, 0]
    assert module.get_status(0.0)["lanes"][0]["encoder_mm"] == 1000.0


def test_a_frame_the_unit_sent_before_the_change_reads_on_at_the_old_offset():
    # a frame on its way when the scale went out still reads at the old scale: no jump, so no
    # phantom travel for a reader that adds up the steps
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(lane_counters_frame([1000000, 0, 0, 0], [0] * 4))
    module.keep_encoder_continuous(0, 1.2342, 1.2468)
    readings = []
    # at the old scale, the old scale 0.5 mm on, then the new scale (one count on in the last)
    for raw in (1000000, 1000500, 1010714, 1010714 + 1247):
        counters(lane_counters_frame([raw, 0, 0, 0], [0] * 4))
        readings.append(module.get_status(0.0)["lanes"][0]["encoder_mm"])
    assert readings[:2] == [1000.0, 1000.5]
    # the 0.5 mm moved in flight is reckoned at the reference's count: 1 % of it, 5 µm
    assert readings[2:] == [pytest.approx(1000.5, abs=0.006), pytest.approx(1001.75, abs=0.006)]
    # the reference's count, 810.24 × (1.2342 - 1.2468) mm: a frame read as old never moves it
    assert module.encoder_offset_um[0] == -10209


def test_a_change_still_pending_a_report_period_on_applies_with_the_next_frame():
    # the assumption failed: the lane went 5 counts back at the new scale before its first frame
    # there, more than half the 10.2 mm jump, so that frame reads as one at the old scale; any
    # frame received a report period and the margin after the change is at the new scale,
    # whatever it reads
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(dict(lane_counters_frame([999702, 0, 0, 0], [0] * 4), **{"#receive_time": 99.5}))
    module.keep_encoder_continuous(0, 1.2342, 1.2468)  # 810 counts, at reactor time 100.0
    back = lane_counters_frame([805 * 12468 // 10, 0, 0, 0], [0] * 4)  # 805 counts at 1.2468
    # the change settles at 101.2 (100.0, LANE_REPORT_S 1.0, SCALE_CHANGE_MARGIN_S 0.2): a frame
    # received up to it is still judged by its value, one just past it is at the new scale
    for receive_time in (100.5, 101.0, 101.199):
        counters(dict(back, **{"#receive_time": receive_time}))
        assert module.scale_pending[0] is not None  # read as frames at the old scale
    counters(dict(back, **{"#receive_time": 101.201}))
    assert module.scale_pending[0] is None
    # 810 counts at 1.2342, then 5 back at 1.2468
    assert module.get_status(0.0)["lanes"][0]["encoder_mm"] == 993.468


def test_a_second_change_before_a_frame_at_the_first_runs_from_the_scale_frames_showed():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(lane_counters_frame([1000000, 1000000, 0, 0], [0] * 4))  # at 1.2342 mm/count
    module.keep_encoder_continuous(0, 1.2342, 1.2468)
    module.keep_encoder_continuous(0, 1.2468, 1.25)  # no frame at 1.2468 yet
    module.keep_encoder_continuous(1, 1.2342, 1.2468)
    module.keep_encoder_continuous(1, 1.2468, 1.2342)  # and back: nothing to keep
    assert module.scale_pending[1] is None
    counters(lane_counters_frame([round(1000000 * 1.25 / 1.2342), 1000000, 0, 0], [0] * 4))
    readings = [lane["encoder_mm"] for lane in module.get_status(0.0)["lanes"]]
    assert readings == [1000.0, 1000.0, 0.0, 0.0]
    # 810.24 counts × (1.2342 - 1.25) mm, the two changes in one
    assert module.encoder_offset_um == [-12802, 0, 0, 0]


def test_an_accepted_counters_reset_drops_the_offset_of_the_lanes_it_reset():
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(lane_counters_frame([1000000] * 4, [0] * 4))
    for lane in range(4):
        module.keep_encoder_continuous(lane, 1.2342, 1.2468)
    reset = printer.objects["gcode"].commands["ACE_COUNTERS_RESET"]
    reset(FakeGcmd({"LANE": "2"}))  # a change still pending goes with the count
    assert module.scale_pending[1] is None and module.scale_pending[0] is not None
    # lane 2 counts from 0 again; the others read on where they were
    counters(lane_counters_frame([1010209, 0, 1010209, 1010209], [0] * 4))
    readings = [lane["encoder_mm"] for lane in module.get_status(0.0)["lanes"]]
    assert readings == [1000.0, 0.0, 1000.0, 1000.0]
    assert module.encoder_offset_um == [-10209, 0, -10209, -10209]
    module.keep_encoder_continuous(0, 1.2468, 1.25)
    responses["ace2k_lane_counters_reset"] = {"lane": 0, "accepted": 0}
    with pytest.raises(FakeGcmd.error, match="refused"):
        reset(FakeGcmd({"LANE": "1"}))
    assert module.encoder_offset_um == [-10209, 0, -10209, -10209]
    assert module.scale_pending[0] is not None
    responses["ace2k_lane_counters_reset"] = {"lane": 255, "accepted": 1}
    reset(FakeGcmd({"LANE": "ALL"}))
    assert module.encoder_offset_um == [0, 0, 0, 0] and module.scale_pending == [None] * 4


def test_an_unconfirmed_counters_reset_drops_the_offset_and_a_pending_change_too():
    known = set(KNOWN_ALL_BUT_HEALTH) - {RESET_RESP.split()[0]}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert not module.reset_confirmed
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(lane_counters_frame([1000000] * 4, [0] * 4))
    module.keep_encoder_continuous(0, 1.2342, 1.2468)
    counters(lane_counters_frame([1010209, 1000000, 1000000, 1000000], [0] * 4))
    module.keep_encoder_continuous(1, 1.2342, 1.2468)  # lane 2: still pending
    assert module.encoder_offset_um[0] == -10209 and module.scale_pending[1] is not None
    reset = printer.objects["gcode"].commands["ACE_COUNTERS_RESET"]
    reset(FakeGcmd({"LANE": "1"}))
    assert module.encoder_offset_um == [0, 0, 0, 0] and module.scale_pending[1] is not None
    reset(FakeGcmd({"LANE": "ALL"}))
    assert module.scale_pending == [None] * 4
    assert mcu.commands == [(RESET_CMD, [0]), (RESET_CMD, [255])]


def test_calibration_save_stages_every_lane():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    mcu.subscriptions["ace2k_sensors_thresholds"](
        thresholds_frame([600, 601, 602, 603], [1100] * 4, [1] * 4)
    )
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_CALIBRATION_SAVE"](gcmd)
    assert printer.objects["configfile"].calls[0] == ("ace2k", "lane1_insert_mv", "600, 1100")
    assert printer.objects["configfile"].calls[3] == ("ace2k", "lane4_insert_mv", "603, 1100")
    assert "SAVE_CONFIG" in gcmd.lines[0]
    assert module.get_status(0.0)["lanes"][2]["source"] == "factory"


def test_calibration_save_stages_nothing_before_the_thresholds_arrive():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_CALIBRATION_SAVE"](gcmd)
    assert printer.objects["configfile"].calls == []
    assert gcmd.lines == ["!! ace2k: thresholds not received yet"]


def test_status_gcode_prints_lanes_and_temperatures():
    module, printer, mcu = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_STATUS"](gcmd)
    text = gcmd.lines[0]
    assert text.startswith("ace2k 0.1.0, link proven: yes")
    assert "lane 1:" in text and "lane 4:" in text and "ptc_left=" in text
    assert "encoder=None mm fg=None" in text and "mains=None Hz (no report yet)" in text
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([0, 0, 0, 12342], [0, 0, 0, 7]))
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 499, "present": 1, "rejects": 0})
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_STATUS"](gcmd)
    text = gcmd.lines[0]
    assert "lane 4:" in text and "encoder=12.342 mm fg=7" in text
    assert "mains=49.9 Hz (present) rejects=0" in text
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 1, "present": 1, "rejects": 0})
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_STATUS"](gcmd)
    assert "mains=missed edge (present) rejects=0" in gcmd.lines[0]


def test_health_gcode_prints_names():
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = {
        "now": 1 << 2 | 1 << 30,
        "latched": 7,
        "post_ms": 2010,
        "reset_cause": 1,
    }
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_HEALTH"](gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_health_query"]
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: FAILING chamber, image_crc"
    assert lines[1] == "failed since boot or last clear: ntc_left, ntc_right, chamber"
    assert lines[2] == "last reset: power-on; self-test at 2010 ms"
    h = module.get_status(0.0)["health"]
    assert h["ok"] is False and h["failing"] == ["chamber", "image_crc"]
    assert h["latched"] == ["ntc_left", "ntc_right", "chamber"]
    assert (h["post_ms"], h["reset_cause"]) == (2010, "power-on")
    # every bit has a name, and the highest one is the image CRC
    assert len(ace2k.HEALTH_BITS) == 31 and ace2k.bit_names(1 << 30) == ["image_crc"]
    assert ace2k.bit_names(0) == []


def test_health_gcode_says_ok_when_nothing_fails():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_HEALTH"](gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: OK"
    assert lines[1] == "failed since boot or last clear: none"
    assert module.get_status(0.0)["health"]["ok"] is True


def test_health_run_and_clear_send_their_commands():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    gcmd = FakeGcmd({"RUN": "1"})
    health(gcmd)
    assert mcu.commands == [("ace2k_health_run", [])]
    # nothing read yet: the current pass is read first, so the re-read can tell the new one
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]
    assert gcmd.lines == ["ace2k: self-test started; results in about 2 s"]
    gcmd = FakeGcmd({"CLEAR": "1"})
    health(gcmd)
    assert mcu.commands[-1] == ("ace2k_health_clear", [])
    assert mcu.sent[-2:] == ["ace2k_health_clear", "ace2k_health_query"]  # cleared, then shown
    assert gcmd.lines[0].startswith("health: OK")
    gcmd = FakeGcmd({"CLEAR": "1", "RUN": "1"})
    health(gcmd)
    assert mcu.sent[-2:] == ["ace2k_health_clear", "ace2k_health_run"]


def test_health_gcode_is_refused_when_the_firmware_has_no_self_test():
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    mcu.config_callback()
    assert "health" not in module.present
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_HEALTH"):
        printer.objects["gcode"].commands["ACE_HEALTH"](FakeGcmd())
    assert mcu.commands == [] and mcu.sent == []


def test_uid_is_hex_in_status():
    module, printer, mcu = make(RESPONSES)
    assert module.get_status(0.0)["uid"] is None
    printer.handlers["klippy:mcu_identify"]()
    assert module.get_status(0.0)["uid"] == SYNTHETIC_UID.hex()
    assert module.get_status(0.0)["uid"] == "1112131415161718191a1b1c"


def test_uid_stays_none_when_the_firmware_has_no_self_test():
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    printer.handlers["klippy:mcu_identify"]()  # no exception, no UID query
    assert non_rfid_sent(mcu) == ["ace2k_version", "ace2k_linkproof_state"]
    assert module.get_status(0.0)["uid"] is None


def test_health_poll_starts_at_ready_and_repeats():
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert printer.reactor.armed() == []
    fire_ready(printer)
    assert len(printer.reactor.armed()) == 1
    cb, waketime = printer.reactor.armed()[0]
    assert waketime == printer.reactor.now + 5.0
    assert module.get_status(0.0)["health"]["ok"] is None  # nothing polled yet
    assert cb(105.0) == 115.0
    assert non_rfid_sent(mcu) == ["ace2k_health_query"]
    st = module.get_status(0.0)["health"]
    assert (st["ok"], st["failing"], st["latched"]) == (True, [], [])
    assert (st["post_ms"], st["reset_cause"]) == (2010, "power-on")


def test_health_poll_survives_one_failure_and_goes_on(caplog):
    # the printer's command_error is what a query the MCU does not answer raises (it is shut
    # down, busy, or gone); a poll that let it through would take the reactor down, and one
    # that stopped on the first would never see the unit come back
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
        HEALTHY,
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(105.0) == 115.0  # the timer is kept
        assert module.get_status(0.0)["health"]["ok"] is None
        assert cb(115.0) == 125.0
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_health_query"]
    assert module.get_status(0.0)["health"]["ok"] is True
    assert module.health_poll_failures == 0  # a success resets the streak
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "health poll failed" in warnings[0]


def test_health_poll_stops_after_five_consecutive_failures(caplog):
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = FakeCommandError(
        "Timeout on wait for 'ace2k_health_state' response"
    )
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        for i in range(4):
            assert cb(105.0 + 10 * i) == 115.0 + 10 * i  # four failures: still polling
        assert cb(145.0) == FakeReactor.NEVER  # the fifth stops it
    assert non_rfid_sent(mcu) == ["ace2k_health_query"] * 5
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 2  # once per streak, then the stop
    assert "health poll failed" in warnings[0]
    assert "health poll stopped after 5 failures" in warnings[1]
    assert module.get_status(0.0)["health"]["ok"] is None


def test_a_failed_poll_after_a_success_reads_unknown_not_healthy(caplog):
    # a unit that stopped answering must not keep reading as healthy from its last answer:
    # the verdict goes unknown (ok None, marked stale), what the last answer said about the
    # latched faults and the reset stays
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        {"now": 0, "latched": 1 << 2, "post_ms": 2010, "reset_cause": 4},
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(105.0) == 115.0
        h = module.get_status(0.0)["health"]
        assert (h["ok"], h["stale"], h["latched"]) == (True, False, ["chamber"])
        assert cb(115.0) == 125.0  # the failure: still polling
    h = module.get_status(0.0)["health"]
    assert h["ok"] is None and h["failing"] == [] and h["stale"] is True
    assert (h["latched"], h["post_ms"], h["reset_cause"]) == (["chamber"], 2010, "watchdog")


def test_overlapping_health_queries_take_turns_on_one_mutex():
    # Klipper keys a query's response handler by the response's name: the poll timer firing
    # while the G-code's own query is in flight would take its answer, the other would time
    # out and the health would read unknown.  Every caller queues on one reactor mutex.  The
    # fake cannot suspend a caller, so it records that the second one entered the mutex while
    # the first held it, and that no query was sent outside it.
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    mutex = module.health_mutex
    assert mutex in printer.reactor.mutexes
    depth_at_send = []
    poll_result = []

    def answer():
        depth_at_send.append(mutex.depth)
        if len(depth_at_send) == 1:
            poll_result.append(poll(105.0))  # the poll fires while this query is in flight
        return HEALTHY

    mcu.responses["ace2k_health_query"] = answer
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_HEALTH"](gcmd)
    assert gcmd.lines[0].startswith("health: OK")
    assert poll_result == [115.0]  # the poll completed and stays armed
    assert depth_at_send == [1, 2]  # both sent under the mutex; the second waited for it
    assert mutex.contended == 1 and mutex.depth == 0
    assert non_rfid_sent(mcu) == ["ace2k_health_query"] * 2
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"]) == (True, False)
    assert module.health_poll_failures == 0


def test_a_query_that_fails_after_another_answered_keeps_the_verdict(caplog):
    # belt and braces behind the mutex: a query that times out marks the health unknown only
    # when no answer landed since it started
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]

    def fail_after_an_answer():
        mcu.responses["ace2k_health_query"] = HEALTHY
        module._query_health()  # an answer lands while this query is still waiting
        raise FakeCommandError("Timeout on wait for 'ace2k_health_state' response")

    mcu.responses["ace2k_health_query"] = fail_after_an_answer
    with caplog.at_level("WARNING", logger="root"):
        assert poll(105.0) == 115.0  # the poll's own query failed: a failure of the streak
    assert module.health_poll_failures == 1
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "health poll failed" in warnings[0]
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"], h["post_ms"]) == (True, False, 2010)  # not wiped


def test_health_gcode_says_unknown_when_the_unit_does_not_answer():
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    gcmd = FakeGcmd()
    health(gcmd)
    assert gcmd.lines[0].startswith("health: OK")
    gcmd = FakeGcmd()
    health(gcmd)  # no exception: the G-code reports the unknown state
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: unknown (no answer from the unit)"
    assert lines[1] == "failed since boot or last clear: none"
    assert lines[2] == "last reset: power-on"  # from the last answer, no "self-test at"
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"], h["post_ms"]) == (None, True, 2010)


def test_health_not_run_yet_while_post_ms_is_zero():
    # post_ms = 0: the firmware's full evaluation has not run (the first is 2 s after boot),
    # so `now` says nothing about the unit yet
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = {"now": 0, "latched": 0, "post_ms": 0, "reset_cause": 3}
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACE_HEALTH"](gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: not run yet"
    assert lines[1] == "failed since boot or last clear: none"
    assert lines[2] == "last reset: software"  # the reset cause still printed, no "at 0 ms"
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["failing"], h["post_ms"], h["reset_cause"]) == (None, [], 0, "software")


def test_health_run_re_queries_once_after_the_pass():
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [HEALTHY, dict(HEALTHY, post_ms=4530)]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    assert poll(105.0) == 115.0  # the pass read before the run: post_ms 2010
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]  # no query with the run
    cb, waketime = armed_timer(printer, module._requery_health)
    assert waketime == printer.reactor.now + 2.5
    assert module.get_status(0.0)["health"]["post_ms"] == 2010
    assert cb(102.5) == FakeReactor.NEVER  # one shot: the pass moved
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_health_run", "ace2k_health_query"]
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["post_ms"]) == (True, 4530)


def test_health_run_before_any_poll_reads_the_current_pass_first():
    # RUN=1 before the first poll (the first 5 s after ready, or after the poll gave up): with
    # no pass read yet the re-read could not tell the new pass from the boot pass, so one query
    # establishes the baseline before the run, and the retry loop engages until post_ms moves
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the baseline, read by RUN=1 itself: post_ms 2010
        HEALTHY,  # the re-query 2.5 s after RUN: still the boot pass
        {"now": 1 << 6, "latched": 1 << 6, "post_ms": 14520, "reset_cause": 1},
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    assert module.get_status(0.0)["health"]["post_ms"] is None
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    gcmd = FakeGcmd({"RUN": "1"})
    health(gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]  # the query first
    assert module.health_rerun_post_ms == 2010
    assert gcmd.lines == ["ace2k: self-test started; results in about 2 s"]
    cb, waketime = armed_timer(printer, module._requery_health)
    assert waketime == printer.reactor.now + 2.5
    assert cb(102.5) == 103.5  # the boot pass again: once more in a second
    assert module.get_status(0.0)["health"]["post_ms"] == 2010
    assert cb(103.5) == FakeReactor.NEVER  # the new pass
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["failing"], h["post_ms"]) == (False, ["zerocross"], 14520)
    assert (
        non_rfid_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 2
    )


def test_health_run_is_refused_when_the_baseline_query_goes_unanswered():
    # no baseline, no run: a re-read with nothing to compare against would accept the boot
    # pass as the new one; the G-code says so and sends nothing
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = FakeCommandError(
        "Timeout on wait for 'ace2k_health_state' response"
    )
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    with pytest.raises(FakeGcmd.error, match="self-test not started"):
        health(FakeGcmd({"RUN": "1"}))
    assert non_rfid_sent(mcu) == ["ace2k_health_query"]
    assert mcu.commands == []
    assert printer.reactor.armed() == []  # the re-read timer is not armed
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"]) == (None, True)


def test_health_run_re_queries_again_while_the_unit_still_reports_the_previous_pass():
    # the re-query can land before the new pass has run (the window plus the probe steps come
    # close to the 2.5 s): a post_ms equal to the one read before RUN is the previous pass, so
    # it is asked again a second later, and the answer with a new post_ms ends it
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the poll before RUN: post_ms 2010
        HEALTHY,  # the re-query 2.5 s after RUN: still the previous pass
        {"now": 1 << 6, "latched": 1 << 6, "post_ms": 14520, "reset_cause": 1},
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    assert poll(105.0) == 115.0
    assert module.get_status(0.0)["health"]["post_ms"] == 2010
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    assert len(printer.reactor.armed()) == 2  # the poll and the re-query
    cb, waketime = armed_timer(printer, module._requery_health)
    assert waketime == printer.reactor.now + 2.5
    assert cb(102.5) == 103.5  # post_ms unchanged: once more in a second
    assert module.get_status(0.0)["health"]["post_ms"] == 2010
    assert cb(103.5) == FakeReactor.NEVER  # the new pass
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["failing"], h["post_ms"]) == (False, ["zerocross"], 14520)
    assert (
        non_rfid_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 2
    )


def test_health_run_re_query_gives_up_after_three_retries_with_a_warning(caplog):
    module, printer, mcu = make(RESPONSES)  # HEALTHY forever: post_ms never moves
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    assert poll(105.0) == 115.0
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    cb, _ = armed_timer(printer, module._requery_health)
    with caplog.at_level("WARNING", logger="root"):
        assert cb(102.5) == 103.5
        assert cb(103.5) == 104.5
        assert cb(104.5) == 105.5
        assert not non_rfid_warnings(caplog)
        assert cb(105.5) == FakeReactor.NEVER  # the third retry is the last
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "previous pass" in warnings[0]
    assert (
        non_rfid_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 4
    )


def test_health_run_re_arms_one_reactor_timer_instead_of_registering_one_per_run():
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [HEALTHY, dict(HEALTHY, post_ms=204010)]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    registered = len(printer.reactor.timers)  # the re-query timer exists from the start, unarmed
    assert printer.reactor.armed() == []
    health = printer.objects["gcode"].commands["ACE_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    printer.reactor.now = 200.0
    health(FakeGcmd({"RUN": "1"}))
    assert len(printer.reactor.timers) == registered  # nothing new registered
    assert len(printer.reactor.armed()) == 1
    cb, waketime = printer.reactor.armed()[0]
    assert waketime == 202.5  # re-armed from the second RUN
    assert cb(202.5) == FakeReactor.NEVER


def test_health_re_query_after_run_logs_a_failure_instead_of_raising(caplog):
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the baseline RUN=1 reads first
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    printer.objects["gcode"].commands["ACE_HEALTH"](FakeGcmd({"RUN": "1"}))
    assert module.get_status(0.0)["health"]["ok"] is True
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(102.5) == FakeReactor.NEVER
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "re-query after RUN failed" in warnings[0]
    assert module.get_status(0.0)["health"]["ok"] is None


def test_no_health_poll_when_the_firmware_has_no_self_test():
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    mcu.config_callback()
    fire_ready(printer)
    assert printer.reactor.armed() == []


def test_bad_threshold_pair_is_a_config_error():
    with pytest.raises(Exception, match="lane1_insert_mv"):
        make(RESPONSES, options={"lane1_insert_mv": [1100, 600]})


def test_the_sensor_type_is_registered_by_the_section():
    _, printer, _ = make(RESPONSES)
    assert "ace2k" in printer.objects["heaters"].factories


def test_subscribe_response_uses_the_newer_method_when_klipper_has_it():
    mcu = FakeMcu(RESPONSES)

    def cb(params):
        return None

    ace2k.subscribe_response(mcu, cb, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions["ace2k_env_state"] is cb
    assert mcu.subscription_formats == ["ace2k_env_state ptc_left_mc=%i"]


def test_subscribe_response_falls_back_to_register_response_by_name():
    mcu = LegacyFakeMcu(RESPONSES)

    def cb(params):
        return None

    ace2k.subscribe_response(mcu, cb, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions == {"ace2k_env_state": cb}
    assert mcu.subscription_formats == ["ace2k_env_state"]


def test_subscribe_response_on_a_legacy_klipper_refuses_a_format_the_dictionary_differs_on():
    mcu = LegacyFakeMcu(RESPONSES, formats={"ace2k_env_state": "ace2k_env_state other=%c"})
    with pytest.raises(msgproto.error):
        ace2k.subscribe_response(mcu, lambda p: None, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions == {}
