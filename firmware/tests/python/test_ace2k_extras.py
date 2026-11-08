"""ace2k.py: the link proof and the device identifier at MCU identify time, the init commands
and subscriptions (each only for a subsystem the firmware carries), the decoding of the reports,
the thresholds re-query, the health poll and its failure streak, the five G-codes — against a
fake printer."""

from __future__ import annotations

import importlib
import os
import re
import struct
import tempfile
from typing import TYPE_CHECKING, NoReturn

import ace2k
import msgproto  # the conftest's stand-in: the parser's error class
import pytest
from ace2k import RESET_CMD, RESET_RESP, probe_format, refuse_format

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence
    from types import TracebackType
    from typing import Any

    from ace2k import Ace2k
    from ace2k_klipper_types import Params

    # One answer a fake query gives: the response, an exception it raises, or a callable called
    # at send time whose return is the response
    Answer = Params | BaseException | Callable[[], Params]
    # What a test sets for a query: one answer, or a list of them handed out in order
    Canned = Answer | list[Answer]
    # The canned answers by command name. Any: a test also sets other values (None, a tuple)
    # that a module under test only stores
    Responses = dict[str, Any]
    # A timer as FakeReactor keeps it: [callback, waketime], re-armed in place
    Timer = list[Any]
    # An object FakePrinter holds: what a module's load_config returned, a fake, or what a test
    # added. Tests read its attributes, so it stays Any; as an alias it is not flagged by
    # ruff's ANN401, on purpose.
    PrinterObject = Any


class FakeQuery:
    """The MCU's canned response for the command, read when the query is sent (a query built
    at connect answers what the test set since): a response, or a list of responses handed out
    in order (the last one repeats; one queue per list, shared by every wrapper of the
    command); an exception instance among them is raised, as a query the MCU never answers
    would raise; a callable is called at send time (what happens while the query is in flight)
    and its return is the response.  send(data) records the name in the shared log and
    (name, data) in the MCU's query list, so a test can check the arguments."""

    def __init__(self, mcu: FakeMcu, msgformat: str) -> None:
        self.mcu = mcu
        self.log = mcu.sent
        self.queries = mcu.queries
        self.msgformat = msgformat

    @property
    def reactor(self) -> FakeReactor | None:
        return self.mcu.reactor

    @property
    def response(self) -> Canned:
        return self.mcu.canned(self.msgformat)

    def send(self, data: Sequence[Any] = (), minclock: int = 0, reqclock: int = 0) -> Params:
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

    def __init__(
        self, log: list[str], commands: list[tuple[str, list[Any]]], msgformat: str
    ) -> None:
        self.log = log
        self.commands = commands
        self.msgformat = msgformat

    def send(self, data: Sequence[Any] = (), minclock: int = 0, reqclock: int = 0) -> None:
        self.log.append(self.msgformat.split()[0])
        self.commands.append((self.msgformat, list(data)))


DEFAULT_CONSTANTS = {
    "ACE2K_KEYS_AS_INIT": 1,  # an image that takes every key as an init command (v0.12.0 on)
    "ACE2K_LANE_SPEED_MIN_UM_S": 9000,
    "ACE2K_LANE_SPEED_MAX_UM_S": 80000,
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

    def __init__(
        self,
        responses: Responses,
        known: set[str] | None = None,
        constants: dict[str, float] | None = None,
        formats: dict[str, str] | None = None,
        lookup_errors: dict[str, str | BaseException] | None = None,
    ) -> None:
        self.sent = []
        self.commands = []
        self.queries = []
        self.responses = responses
        self.queues = {}  # name -> (the list set, the queue handed out from it)
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

    def config_callback(self) -> None:
        for cb in self.config_callbacks:
            cb()

    def _known(self, msgformat: str) -> bool:
        name = msgformat.split(maxsplit=1)[0]
        if self.known is not None and name not in self.known:
            return False
        if name in self.lookup_errors:
            return False
        have = self.formats.get(name)
        return have is None or have == msgformat

    def _lookup(self, msgformat: str) -> None:
        """As Klipper's message parser (msgproto.lookup_command), a msgproto.error: "Unknown
        command: <name>" for a name the dictionary lacks, "Command format mismatch: <asked> vs
        <have>" for a name it carries with another format; lookup_errors' entry for a name listed
        there."""
        name = msgformat.split(maxsplit=1)[0]
        if self.known is not None and name not in self.known:
            raise msgproto.error(f"Unknown command: {name}")
        if name in self.lookup_errors:
            failure = self.lookup_errors[name]
            raise failure if isinstance(failure, BaseException) else msgproto.error(failure)
        have = self.formats.get(name)
        if have is not None and have != msgformat:
            raise msgproto.error(f"Command format mismatch: {msgformat} vs {have}")

    def lookup_query_command(
        self,
        msgformat: str,
        respformat: str,
        oid: int | None = None,
        cq: object = None,
        is_async: bool = False,
    ) -> FakeQuery:
        # as Klipper's wrapper: the message parser raises on a command or a response the
        # dictionary lacks — from a G-code handler, that is a shutdown
        for fmt in (msgformat, respformat):
            self._lookup(fmt)
        return FakeQuery(self, msgformat.split(maxsplit=1)[0])

    def canned(self, name: str) -> Canned:
        """The response set for name now; a list is copied once into the command's queue (the
        test's list left whole), and a new list set later starts a new queue."""
        response = self.responses[name]
        if not isinstance(response, list):
            return response
        source, queue = self.queues.get(name, (None, None))
        if source is not response:
            queue = list(response)
            self.queues[name] = (response, queue)
        return queue

    def lookup_command(self, msgformat: str, cq: object = None) -> FakeCommand:
        """As Klipper's wrapper: the parser's error for a format the dictionary lacks or carries
        differently, else a command that records what it sends."""
        self._lookup(msgformat)
        return FakeCommand(self.sent, self.commands, msgformat)

    def try_lookup_command(self, msgformat: str) -> FakeCommand | None:
        return self.lookup_command(msgformat) if self._known(msgformat) else None

    def check_valid_response(self, msgformat: str) -> bool:
        return self._known(msgformat)

    def add_config_cmd(self, cmd: str, is_init: bool = False, on_restart: bool = False) -> None:
        self.config_cmds.append((cmd, is_init))

    def register_config_callback(self, cb: Callable[[], object]) -> None:
        self.config_callbacks.append(cb)

    def register_serial_response(
        self, cb: Callable[[Params], object], msgformat: str, oid: int | None = None
    ) -> None:
        # as Klipper's wrapper: a format the dictionary lacks raises at registration
        assert self._known(msgformat), msgformat
        self.subscriptions[msgformat.split(maxsplit=1)[0]] = cb
        self.subscription_formats.append(msgformat)

    def seconds_to_clock(self, t: float) -> int:
        return int(t * 120000000)

    def estimated_print_time(self, t: float) -> float:
        return t

    def get_constant_float(self, name: str) -> float:
        return float(self.constants[name])


class LegacyFakeMcu(FakeMcu):
    """A Klipper older than the pinned one, as Snapmaker's U1 runs: no register_serial_response,
    only register_response(cb, name, oid), which registers at once and checks nothing (the
    format check is the caller's)."""

    register_serial_response = None

    def register_response(
        self, cb: Callable[[Params], object], msg: str, oid: int | None = None
    ) -> None:
        self.subscriptions[msg] = cb
        self.subscription_formats.append(msg)


class FakeMutex:
    """Klipper's reactor mutex as a context manager that cannot suspend a caller: it counts the
    ones that entered while another held it (where the real one queues them until the holder
    leaves) and how deep the nesting is at any moment."""

    def __init__(self) -> None:
        self.depth = 0
        self.contended = 0

    def test(self) -> bool:
        return self.depth > 0

    def __enter__(self) -> None:
        if self.depth:
            self.contended += 1
        self.depth += 1

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
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

    def __init__(self) -> None:
        self.timers = []  # every registered timer: a mutable [callback, waketime] pair
        self.mutexes = []
        self.now = 100.0
        self.while_waiting = {}
        self.pause_disabled = False  # set while klippy:ready handlers run, as Klipper does

    def check_pause(self) -> None:
        """Klipper's reactor refuses a pause (a blocking query, a completion's wait) inside a
        klippy:ready handler: the same error here, so a handler that blocks fails its test."""
        if self.pause_disabled:
            raise RuntimeError("Internal error - reactor pause disabled")

    def mutex(self, is_locked: bool = False) -> FakeMutex:
        assert not is_locked
        m = FakeMutex()
        self.mutexes.append(m)
        return m

    def register_async_callback(
        self, cb: Callable[[float], object], waketime: float | None = None
    ) -> None:
        cb(0.0)

    def register_timer(self, cb: Callable[[float], float], waketime: float = NEVER) -> Timer:
        timer = [cb, waketime]
        self.timers.append(timer)
        return timer

    def update_timer(self, timer: Timer, waketime: float) -> None:
        assert timer in self.timers
        timer[1] = waketime

    def armed(self) -> list[tuple[Callable[[float], float], float]]:
        return [(cb, waketime) for cb, waketime in self.timers if waketime != self.NEVER]

    def monotonic(self) -> float:
        return self.now

    def pause(self, waketime: float) -> float:
        """A greenlet's pause: the clock moves on to the wake time."""
        self.check_pause()
        self.now = max(self.now, waketime)
        return self.now

    def completion(self) -> FakeCompletion:
        return FakeCompletion(self)


class FakeCompletion:
    """Klipper's ReactorCompletion without a greenlet: wait() returns the result once complete()
    ran, else the default — the tests inject the event from the query's send() callable, or
    through the reactor's while_waiting entry for this completion, which a pending wait() drains
    before it gives up."""

    def __init__(self, reactor: FakeReactor | None = None) -> None:
        self.reactor = reactor
        self.result = None
        self.done = False

    def test(self) -> bool:
        return self.done

    def complete(self, result: object) -> None:
        self.result = result
        self.done = True

    def wait(self, waketime: float | None = None, waketime_result: object = None) -> object:
        if self.reactor is not None:
            self.reactor.check_pause()
        queued = self.reactor.while_waiting.get(self, []) if self.reactor is not None else []
        while not self.done and queued:
            queued.pop(0)(waketime)
        return self.result if self.done else waketime_result


class FakeGcode:
    def __init__(self) -> None:
        self.commands = {}
        self.output = []

    def register_command(
        self, name: str, func: Callable[[FakeGcmd], object], desc: str | None = None
    ) -> None:
        def run(gcmd: FakeGcmd) -> object:
            # Klipper builds the command with its name: get_command() answers it
            if getattr(gcmd, "command", None) is None:
                gcmd.command = name
            return func(gcmd)

        self.commands[name] = run

    def respond_info(self, msg: str) -> None:
        self.output.append(msg)


class FakeGcmd:
    """The output lines, and the parameters of the G-code line as Klipper's GCodeCommand hands
    them out: get() the raw string, get_int() / get_float() parsed and range-checked, raising
    gcmd.error; both return None for an absent key when the default is None."""

    error = Exception

    def __init__(self, params: dict[str, Any] | None = None, command: str | None = None) -> None:
        self.lines = []
        self.params = params or {}
        self.command = command  # the command's name, set by FakeGcode as it runs it

    def get_command(self) -> str | None:
        return self.command

    def respond_info(self, msg: str) -> None:
        self.lines.append(msg)

    def respond_raw(self, msg: str) -> None:
        self.lines.append(msg)

    def get(self, key: str, default: object = None) -> object:
        return self.params.get(key, default)

    def get_command_parameters(self) -> dict[str, Any]:
        return dict(self.params)

    def get_int(
        self,
        key: str,
        default: int | None = None,
        minval: int | None = None,
        maxval: int | None = None,
    ) -> int | None:
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

    def get_float(
        self,
        key: str,
        default: float | None = None,
        minval: float | None = None,
        maxval: float | None = None,
        above: float | None = None,
        below: float | None = None,
    ) -> float | None:
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
    def __init__(self) -> None:
        self.factories = {}

    def add_sensor_factory(self, name: str, factory: Callable[[Any], object]) -> None:
        self.factories[name] = factory


class FakeConfigfile:
    def __init__(self) -> None:
        self.calls = []
        self.deprecated = []  # (section, option) per deprecate, as Klipper lists its warnings

    def set(self, section: str, option: str, value: str) -> None:
        self.calls.append((section, option, value))

    def deprecate(self, section: str, option: str) -> None:
        self.deprecated.append((section, option))


class FakeConfig:
    error = Exception

    def __init__(
        self,
        printer: FakePrinter,
        mcu_name: str | None = None,
        options: dict[str, Any] | None = None,
    ) -> None:
        self.printer = printer
        self.mcu_name = mcu_name
        self.options = options or {}

    def get_printer(self) -> FakePrinter:
        return self.printer

    def get_name(self) -> str:
        return "ace2k"

    def get(self, key: str, default: object = None) -> object:
        if key == "mcu" and self.mcu_name is not None:
            return self.mcu_name
        return self.options.get(key, default)

    def getintlist(
        self,
        key: str,
        default: list[int] | None = None,
        sep: str = ",",
        count: int | None = None,
    ) -> list[int] | None:
        value = self.options.get(key)
        if value is None:
            return default
        assert count is None or len(value) == count
        return list(value)

    def getint(
        self,
        key: str,
        default: int | None = None,
        minval: int | None = None,
        maxval: int | None = None,
    ) -> int | None:
        value = self.options.get(key, default)
        if value is None:
            return None
        value = int(value)
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise self.error(f"{key} out of range")
        return value

    def getfloat(
        self,
        key: str,
        default: float | None = None,
        minval: float | None = None,
        maxval: float | None = None,
        above: float | None = None,
        below: float | None = None,
    ) -> float | None:
        value = self.options.get(key, default)
        if value is None:
            return None
        value = float(value)
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise self.error(f"{key} out of range")
        if (above is not None and value <= above) or (below is not None and value >= below):
            raise self.error(f"{key} out of range")
        return value

    def getboolean(self, key: str, default: bool | None = None) -> bool | None:
        value = self.options.get(key, default)
        return None if value is None else bool(value)

    def getchoice(self, key: str, choices: dict[str, bool], default: str | None = None) -> bool:
        value = self.options.get(key, default)
        if value not in choices:  # Klipper's own error
            raise self.error(f"Choice '{value}' for option '{key}' in section 'ace2k' is not valid")
        return choices[value]

    def deprecate(self, option: str) -> None:
        # Klipper's: only an option that the section sets is listed
        if option in self.options:
            self.printer.lookup_object("configfile").deprecate(self.get_name(), option)

    def getsection(self, name: str) -> FakeConfig:
        # every section is this one: the options of [ace2k] are what the fake carries
        return self

    def get_prefix_sections(self, prefix: str) -> list[FakeConfig]:
        # a single [ace2k]: the only section the fake carries
        return [self] if self.get_name().startswith(prefix) else []


class UnitConfig(FakeConfig):
    """[ace2k <name>] (name None: [ace2k]) among `sections`, every option in `options`."""

    def __init__(
        self,
        printer: FakePrinter,
        name: str | None,
        mcu_name: str | None,
        options: dict[str, Any] | None = None,
        sections: list[UnitConfig] | None = None,
    ) -> None:
        super().__init__(printer, mcu_name=mcu_name, options=options)
        self.name = name
        self.sections = sections if sections is not None else [self]

    def get_name(self) -> str:
        return "ace2k" if self.name is None else f"ace2k {self.name}"

    def get(self, key: str, default: object = None) -> object:
        if key == "mcu":
            return self.mcu_name if self.mcu_name is not None else default
        return self.options.get(key, default)

    def get_prefix_sections(self, prefix: str) -> list[FakeConfig]:
        return [s for s in self.sections if s.get_name().startswith(prefix)]


class FakeCommandError(Exception):
    """The printer's command_error: what a query the MCU does not answer raises."""


class FakePrinter:
    command_error = FakeCommandError
    config_error = Exception

    def __init__(self, mcu: FakeMcu) -> None:
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

    def send_event(self, event: str, *params: object) -> None:
        self.events.append((event, params))

    def is_shutdown(self) -> bool:  # as Klipper's printer.is_shutdown(); a test may replace it
        return False

    def add_object(self, name: str, obj: object) -> None:
        self.objects[name] = obj

    _REQUIRED = object()

    def lookup_object(self, name: str, default: object = _REQUIRED) -> PrinterObject:
        # as Klipper's: a missing name raises (KeyError here) unless a default is given
        if default is self._REQUIRED or name in self.objects:
            return self.objects[name]
        return default

    def load_object(self, config: FakeConfig, name: str) -> PrinterObject:
        if name not in self.objects:
            self.objects[name] = importlib.import_module(name).load_config(config)
        return self.objects[name]

    def get_reactor(self) -> FakeReactor:
        return self.reactor

    def register_event_handler(self, event: str, func: Callable[[], object]) -> None:
        # Klipper runs every handler of an event, in registration order: [ace2k] and the
        # modules it loads (ace2k_rfid) both handle klippy:connect and klippy:ready
        fns = self.handler_lists.setdefault(event, [])
        fns.append(func)
        self.handlers[event] = lambda fns=fns, event=event: self._run_handlers(event, fns)

    def _run_handlers(self, event: str, fns: list[Callable[[], object]]) -> list[object]:
        # Klipper disables the reactor's pause while the klippy:ready handlers run
        self.reactor.pause_disabled = event == "klippy:ready"
        try:
            return [f() for f in fns]
        finally:
            self.reactor.pause_disabled = False

    def lookup_objects(self, module: str | None = None) -> list[tuple[str, Any]]:
        """As Klipper's: (name, object) for every object whose name is `module` or starts with
        `module + " "`."""
        return [
            (name, obj)
            for name, obj in self.objects.items()
            if module is None or name == module or name.startswith(module + " ")
        ]

    def get_start_args(self) -> dict[str, str]:
        return {
            "config_file": os.path.join(self.start_dir, "printer.cfg"),
            "log_file": os.path.join(self.start_dir, "klippy.log"),
        }


def make(
    responses: Responses,
    options: dict[str, Any] | None = None,
    known: set[str] | None = None,
    constants: dict[str, float] | None = None,
    formats: dict[str, str] | None = None,
    lookup_errors: dict[str, str | BaseException] | None = None,
    mcu_class: type[FakeMcu] | None = None,
) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    mcu = (mcu_class or FakeMcu)(responses, known, constants, formats, lookup_errors)
    printer = FakePrinter(mcu)
    module = ace2k.load_config(FakeConfig(printer, options=options))
    return module, printer, mcu


def make_units(
    names: Sequence[str] = ("a", "b"),
    responses: Responses | None = None,
    options: dict[str, dict[str, Any]] | None = None,
    constants: dict[str, float] | None = None,
    extra: dict[str, object] | None = None,
) -> tuple[list[Ace2k], FakePrinter, dict[str, FakeMcu]]:
    """Several named units on one fake printer, each on its own MCU ("mcu ace2k_<name>"), loaded
    in order as Klipper loads [ace2k <name>].  options: {name: {key: value}}; extra: objects added
    to the printer before the units load (extruders, the toolhead's clock)."""
    mcus = {n: FakeMcu(dict(responses or RESPONSES), constants=constants) for n in names}
    printer = FakePrinter(mcus[names[0]])
    del printer.objects["mcu ace2k"]
    for n, mcu in mcus.items():
        mcu.reactor = printer.reactor
        printer.objects[f"mcu ace2k_{n}"] = mcu
    for key, obj in (extra or {}).items():
        printer.objects[key] = obj
    sections = []
    for n in names:
        opts = dict((options or {}).get(n, {}))
        sections.append(UnitConfig(printer, n, f"ace2k_{n}", opts, sections))
    units = []
    for section in sections:
        unit = ace2k.load_config_prefix(section)
        printer.objects[section.get_name()] = unit
        units.append(unit)
    return units, printer, mcus


def thresholds_frame(
    lows: Sequence[int], highs: Sequence[int], sources: Sequence[int]
) -> dict[str, bytes]:
    """One ace2k_sensors_thresholds frame: four uint16 LE, four uint16 LE, four bytes."""
    return {
        "low_mv": struct.pack("<4H", *lows),
        "high_mv": struct.pack("<4H", *highs),
        "source": bytes(sources),
    }


def lane_counters_frame(encoder_um: Sequence[int], fg: Sequence[int]) -> dict[str, bytes]:
    """One ace2k_lane_counters frame: four int32 LE, four uint32 LE."""
    return {"encoder_um": struct.pack("<4i", *encoder_um), "fg": struct.pack("<4I", *fg)}


def fire_ready(printer: FakePrinter) -> None:
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


def armed_timer(
    printer: FakePrinter, cb: Callable[[float], float]
) -> tuple[Callable[[float], float], float]:
    """The (callback, waketime) of the one armed timer with that callback."""
    (timer,) = [t for t in printer.reactor.armed() if t[0] == cb]
    return timer


SYNTHETIC_UID = bytes(range(0x11, 0x11 + 12))  # any 12 bytes; no real unit's
HEALTHY = {"now": 0, "latched": 0, "post_ms": 2010, "reset_cause": 1}
RESPONSES = {
    "ace2k_version": {"version": b"0.1.0"},
    "ace2k_feed_speed_set": {"lane": 0, "accepted": 1, "reason": 0},
    "ace2k_linkproof_state": {"proven": 1, "remaining_ms": 0},
    "ace2k_bootloader_enter": {"accepted": 1},
    "ace2k_uid_query": {"uid": SYNTHETIC_UID},
    "ace2k_health_query": HEALTHY,
    "ace2k_link_gap_query": {"max_ms": 980, "lost_ms": 3000},
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


def non_rfid_warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    """The WARNING messages, ace2k_rfid's own left out (its firmware and section checks)."""
    return [
        r.getMessage()
        for r in caplog.records
        if r.levelname == "WARNING" and not any(w in r.getMessage() for w in RFID_WARNINGS)
    ]


def non_rfid_sent(mcu: FakeMcu) -> list[str]:
    """The names the MCU was sent, ace2k_rfid's and ace2k_dryer's own left out (their queries
    after ready)."""
    return [name for name in mcu.sent if not name.startswith(OTHER_MODULES)]


def self_test_sent(mcu: FakeMcu) -> list[str]:
    """non_rfid_sent without the query and the clear of the longest silence on the link, which go
    out with every health read and clear (the test_health_*silence* tests check those)."""
    return [name for name in non_rfid_sent(mcu) if not name.startswith("ace2k_link_gap_")]


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


def test_mcu_identify_queries_version_then_state_then_uid() -> None:
    module, printer, mcu = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    assert non_rfid_sent(mcu) == ["ace2k_version", "ace2k_linkproof_state", "ace2k_uid_query"]
    status = module.get_status(0.0)
    assert (status["version"], status["link_proven"]) == ("0.1.0", True)


def test_status_gcode_reports_version_and_proof() -> None:
    module, printer, _ = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert gcmd.lines[0].startswith("ace2k 0.1.0, link proven: yes")


def test_restore_stock_sends_bootloader_enter() -> None:
    module, printer, mcu = make(RESPONSES)
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_RESTORE_STOCK"](gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_bootloader_enter"]
    assert gcmd.lines[0].startswith("ace2k: entering the bootloader")


def test_restore_stock_refusal_is_a_gcode_error() -> None:
    responses = dict(RESPONSES)
    responses["ace2k_bootloader_enter"] = {"accepted": 0}
    module, printer, mcu = make(responses)
    gcmd = FakeGcmd()
    # a G-code error, so a macro stops here instead of going on as if the unit had reset
    with pytest.raises(FakeGcmd.error) as e:
        printer.objects["gcode"].commands["ACEK_RESTORE_STOCK"](gcmd)
    assert str(e.value) == "ace2k: refused — a subsystem is busy (heating or moving)"
    assert non_rfid_sent(mcu) == ["ace2k_bootloader_enter"]  # the ask itself, nothing after
    assert gcmd.lines == []


def no_answer(name: str) -> FakeCommandError:
    return FakeCommandError(f"Unable to obtain '{name}' response")


@pytest.mark.parametrize(
    "gcode, params, response, text",
    [
        ("ACEK_RESTORE_STOCK", {}, "ace2k_bootloader_enter", "bootloader enter"),
        ("ACEK_COUNTERS_RESET", {"LANE": "1"}, "ace2k_lane_counters_reset", "counters reset"),
        ("ACEK_HEALTH", {"RUN": "1"}, "ace2k_health_query", "health"),
    ],
)
def test_a_gcode_query_with_no_answer_is_a_named_gcode_error_under_the_units_mutex(
    gcode: str, params: dict[str, str], response: str, text: str
) -> None:
    responses = dict(RESPONSES)
    held = []

    def silent() -> NoReturn:
        held.append(ace2k.query_mutex(printer, mcu).depth)
        raise no_answer(response)

    responses[response] = silent
    module, printer, mcu = make(responses)
    mcu.config_callback()
    with pytest.raises(Exception) as caught:
        printer.objects["gcode"].commands[gcode](FakeGcmd(params))
    assert not isinstance(caught.value, FakeCommandError)
    assert str(caught.value).startswith(
        f"ace2k: {text}: no answer from the unit; check the link (Unable to obtain"
    )
    assert held == [1]  # sent holding the one query mutex of the unit


def test_the_health_poll_names_a_lost_answer_the_same_way(caplog: pytest.LogCaptureFixture) -> None:
    responses = dict(RESPONSES, ace2k_health_query=no_answer("ace2k_health_state"))
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        cb(105.0)  # does not raise
    assert "ace2k: health: no answer from the unit; check the link" in caplog.text


def test_every_gcode_name_is_letters_and_underscores_only() -> None:
    # Klipper's gcode.py reads a command name as [A-Z_]+ (or the G1/M104 form): a digit ends the
    # name, so ACE2K_STATUS would be dispatched as "ACE2" — hence ACEK_* while the section is
    # [ace2k]
    _, printer, _ = make(RESPONSES)
    names = list(printer.objects["gcode"].commands)
    assert names == [  # the feed's sixteen register while [ace2k] loads ace2k_feed
        "ACEK_STATUS",
        "ACEK_RESTORE_STOCK",
        "ACEK_FEED",
        "ACEK_ROLLBACK",
        "ACEK_UNLOAD",
        "ACEK_LOAD",
        "ACEK_ASSIST",
        "ACEK_STOP",
        "ACEK_CLEAR",
        "ACEK_SPEED",
        "ACEK_CALIBRATE_ENCODER",
        "ACEK_LOAD_SET",
        "ACEK_SNAG_SET",
        "ACEK_FOLLOW_SET",
        "ACEK_FF_SET",
        "ACEK_FEED_FORWARD",
        "ACEK_GRIP",
        "ACEK_LANE_ACTIVE",
        "ACEK_RFID_READ",
        "ACEK_RFID_FORGET",
        "ACEK_DRY",
        "ACEK_DRY_STOP",
        "ACEK_DRY_CLEAR",
        "ACEK_FAN",
        "ACEK_FLAP",
        "ACEK_DRYER_LOG",
        "ACEK_DRY_MODE",
        "ACEK_CALIBRATION_SAVE",
        "ACEK_COUNTERS_RESET",
        "ACEK_HEALTH",
    ]
    for name in names:
        assert re.fullmatch(r"[A-Z_]+", name), name


def test_no_gcode_keeps_the_old_ace_prefix() -> None:
    # every command is ACEK_*, none ACE_*: those names stay free for other ACE modules, and
    # Klipper stops on a command registered twice
    _, printer, _ = make(RESPONSES)
    names = list(printer.objects["gcode"].commands)
    assert names
    assert [n for n in names if not n.startswith("ACEK_")] == []


def test_an_unproven_link_is_retried_once_and_can_succeed() -> None:
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


def test_a_link_still_unproven_after_the_retry_is_reported_and_warned_about(
    caplog: pytest.LogCaptureFixture,
) -> None:
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
    # either cause, and no reset foretold: a store the heater put off is made by the unit later
    assert "flash" in warnings[0]
    assert "heater" in warnings[0]
    assert "resets" not in warnings[0]


UNPROVEN = {"proven": 0, "remaining_ms": 120000}


def watch_timers(printer: FakePrinter) -> list[list[Any]]:
    """The armed timers of the link proof watch (see Ace2k._watch_linkproof)."""
    return [
        t
        for t in printer.reactor.timers
        if getattr(t[0], "__name__", "") == "_watch_linkproof" and t[1] != FakeReactor.NEVER
    ]


def run_watch(printer: FakePrinter) -> None:
    """Run the one armed pass of the watch, as the reactor would at its wake time."""
    (timer,) = watch_timers(printer)
    timer[1] = timer[0](timer[1])


def unproven_at_connect(
    later: list[object],
    caplog: pytest.LogCaptureFixture,
    constants: dict[str, float] | None = None,
) -> tuple[Ace2k, FakePrinter, FakeMcu]:
    """A unit unproven at connect and after the retry, then answering `later` to the watch, with
    Klipper ready.  The sends up to here are cleared."""
    responses = dict(RESPONSES)
    responses["ace2k_linkproof_state"] = [UNPROVEN, UNPROVEN, *later]
    module, printer, mcu = make(responses, constants=constants)
    printer.handlers["klippy:mcu_identify"]()
    fire_ready(printer)
    mcu.sent.clear()
    caplog.clear()
    return module, printer, mcu


def test_a_proof_the_unit_stores_later_makes_the_link_proven(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level("INFO", logger="root"):
        module, printer, mcu = unproven_at_connect([{"proven": 1, "remaining_ms": 0}], caplog)
        assert module.get_status(0.0)["link_proven"] is False
        assert len(watch_timers(printer)) == 1
        run_watch(printer)
    assert module.get_status(0.0)["link_proven"] is True
    assert non_rfid_sent(mcu) == ["ace2k_linkproof_state"]  # the state only, no ace2k_version
    infos = [r.getMessage() for r in caplog.records if "link proof" in r.getMessage()]
    assert len(infos) == 1
    assert caplog.records[-1].levelname == "INFO"
    assert watch_timers(printer) == []  # stopped: nothing more is sent
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert "link proven: yes" in gcmd.lines[0]


def test_an_older_image_reaches_the_end_of_the_window_and_is_warned_of_the_recovery_once(
    caplog: pytest.LogCaptureFixture,
) -> None:
    passes = [{"proven": 0, "remaining_ms": ms} for ms in (4000, 2000, 0)]
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu = unproven_at_connect(passes, caplog)
        for _ in range(2):
            run_watch(printer)
            assert non_rfid_warnings(caplog) == []
        run_watch(printer)
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1
    assert "recovery" in warnings[0]
    assert non_rfid_sent(mcu) == ["ace2k_linkproof_state"] * 3
    assert watch_timers(printer) == []
    assert module.get_status(0.0)["link_proven"] is False


# An image whose proof refused busy waits past the window for the flash (since v0.13.0).
RETRY_PAST_WINDOW = {**DEFAULT_CONSTANTS, "ACE2K_LINKPROOF_RETRY_PAST_WINDOW": 1}


def test_an_image_that_retries_past_the_window_is_watched_until_it_stores_the_proof(
    caplog: pytest.LogCaptureFixture,
) -> None:
    ended = {"proven": 0, "remaining_ms": 0}
    passes = [{"proven": 0, "remaining_ms": 2000}, ended, ended, ended]
    passes.append({"proven": 1, "remaining_ms": 0})
    with caplog.at_level("INFO", logger="root"):
        module, printer, mcu = unproven_at_connect(passes, caplog, RETRY_PAST_WINDOW)
        for _ in range(4):
            run_watch(printer)
            assert len(watch_timers(printer)) == 1  # the watch goes on past the window
        warnings = non_rfid_warnings(caplog)
        assert len(warnings) == 1  # once, at the end of the window
        assert "heater" in warnings[0]
        assert "keeps talking" in warnings[0]  # the hold lasts only while the host talks
        assert module.get_status(0.0)["link_proven"] is False
        run_watch(printer)
    assert module.get_status(0.0)["link_proven"] is True
    assert watch_timers(printer) == []
    assert non_rfid_sent(mcu) == ["ace2k_linkproof_state"] * 5  # never ace2k_version


def test_the_retry_past_the_window_is_read_once_at_identify(
    caplog: pytest.LogCaptureFixture,
) -> None:
    ended = {"proven": 0, "remaining_ms": 0}
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu = unproven_at_connect([ended], caplog, RETRY_PAST_WINDOW)
        assert module.linkproof_retry_past_window is True
        del mcu.constants["ACE2K_LINKPROOF_RETRY_PAST_WINDOW"]  # a pass does not read it again
        run_watch(printer)
    assert len(watch_timers(printer)) == 1
    assert "keeps talking" in non_rfid_warnings(caplog)[0]


def test_a_proven_link_at_connect_starts_no_watch() -> None:
    module, printer, mcu = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    fire_ready(printer)
    assert watch_timers(printer) == []
    assert "ace2k_linkproof_state" not in non_rfid_sent(mcu)[2:]  # only the one at connect


def test_the_watch_stops_after_five_unanswered_polls(caplog: pytest.LogCaptureFixture) -> None:
    silent = no_answer("ace2k_linkproof_state_response")
    with caplog.at_level("WARNING", logger="root"):
        module, printer, mcu = unproven_at_connect([silent], caplog)
        for _ in range(4):
            run_watch(printer)
        assert len(watch_timers(printer)) == 1  # four in a row: it tries again
        run_watch(printer)
    assert watch_timers(printer) == []
    assert non_rfid_sent(mcu) == ["ace2k_linkproof_state"] * 5
    assert "stopped" in non_rfid_warnings(caplog)[-1]
    assert module.get_status(0.0)["link_proven"] is False


def test_an_answer_resets_the_count_of_unanswered_polls(caplog: pytest.LogCaptureFixture) -> None:
    silent = no_answer("ace2k_linkproof_state_response")
    module, printer, mcu = unproven_at_connect(
        [silent] * 4 + [{"proven": 0, "remaining_ms": 9000}] + [silent] * 4 + [UNPROVEN], caplog
    )
    for _ in range(9):
        run_watch(printer)
    assert len(watch_timers(printer)) == 1


@pytest.mark.parametrize("event", ["klippy:shutdown", "klippy:disconnect"])
def test_the_watch_stops_on_shutdown(event: str, caplog: pytest.LogCaptureFixture) -> None:
    module, printer, mcu = unproven_at_connect([UNPROVEN], caplog)
    assert len(watch_timers(printer)) == 1
    printer.handlers[event]()
    assert watch_timers(printer) == []


def test_the_primary_mcu_is_looked_up_without_a_suffix() -> None:
    mcu = FakeMcu(RESPONSES)
    printer = FakePrinter(mcu)
    printer.objects["mcu"] = mcu
    module = ace2k.load_config(FakeConfig(printer, mcu_name="mcu"))
    printer.handlers["klippy:mcu_identify"]()
    assert module.get_status(0.0)["version"] == "0.1.0"


def test_build_config_sends_thresholds_and_queries() -> None:
    module, printer, mcu = make(RESPONSES, options={"lane2_insert_mv": [700, 1300]})
    mcu.config_callback()
    # init commands only: nothing in Klipper's config checksum, so editing or removing a key
    # never resets the MCU; a lane without a key goes back to the unit's own band
    config_cmds = [c for c, is_init in mcu.config_cmds if not is_init]
    assert config_cmds == []
    init_cmds = [c for c, is_init in mcu.config_cmds if is_init and not c.startswith(OTHER_MODULES)]
    assert "ace2k_sensors_thresholds_query" not in init_cmds  # asked at connect instead
    assert init_cmds == [  # the health query is polled from the host, not an init command
        # the feed's (ace2k_feed.py registers its config callback first, while [ace2k] loads it)
        "ace2k_lane_scale_set lane=0 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=1 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=2 um_per_count_x10=12342",
        "ace2k_lane_scale_set lane=3 um_per_count_x10=12342",
        "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=15000 stall_check_um=20000",
        "ace2k_feed_load_set park_um=300000 speed_um_s=60000 auto_load=1",
        "ace2k_feed_snag_set fwd_um=20000 lag_um=5000 duty_pct=0 back_um=5000 rest_ms=500"
        " free_um=50000",
        "ace2k_feed_query rest_ticks=120000000",
        "ace2k_sensors_thresholds_reset lane=0",
        "ace2k_sensors_thresholds_set lane=1 low_mv=700 high_mv=1300",
        "ace2k_sensors_thresholds_reset lane=2",
        "ace2k_sensors_thresholds_reset lane=3",
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
        "ace2k_lane_motor",  # subscribed at once; the report runs only while watched
        "ace2k_mains_state",
        "ace2k_feed_state",
        "ace2k_feed_event",
    }


OLD_IMAGE = {k: v for k, v in DEFAULT_CONSTANTS.items() if k != "ACE2K_KEYS_AS_INIT"}


def test_an_older_image_keeps_every_key_a_config_command_and_sends_no_default() -> None:
    """An image without ace2k_sensors_thresholds_reset (before v0.12.0): the keys stay config
    commands, as before, so a host update alone leaves Klipper's config checksum as it was."""
    module, printer, mcu = make(
        RESPONSES,
        options={
            "lane2_insert_mv": [700, 1300],
            "lane2_encoder_scale": 1.25,
            "slip_allow_mm": 10,
        },
        constants=OLD_IMAGE,
    )
    mcu.config_callback()
    cfg = [c for c, is_init in mcu.config_cmds if not is_init and not c.startswith(OTHER_MODULES)]
    assert sorted(cfg) == sorted(
        [
            "ace2k_sensors_thresholds_set lane=1 low_mv=700 high_mv=1300",
            "ace2k_lane_scale_set lane=1 um_per_count_x10=12500",
            "ace2k_feed_thresholds_set slip_check_um=50000 slip_allow_um=10000"
            " stall_check_um=20000",
        ]
    )
    init = [c for c, is_init in mcu.config_cmds if is_init]
    assert not any(c.startswith("ace2k_sensors_thresholds") for c in init)
    assert not any(c.startswith("ace2k_feed_thresholds_set") for c in init)


def test_the_bands_go_out_before_the_sensors_reports_start() -> None:
    module, printer, mcu = make(RESPONSES, options={"lane2_insert_mv": [700, 1300]})
    mcu.config_callback()
    init = [c for c, is_init in mcu.config_cmds if is_init]
    band = init.index("ace2k_sensors_thresholds_set lane=1 low_mv=700 high_mv=1300")
    query = next(i for i, c in enumerate(init) if c.startswith("ace2k_sensors_query"))
    assert band < query


def test_subscriptions_use_the_packed_per_module_formats() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert "ace2k_sensors_thresholds low_mv=%*s high_mv=%*s source=%*s" in mcu.subscription_formats
    assert "ace2k_lane_counters encoder_um=%*s fg=%*s" in mcu.subscription_formats
    assert not any(
        "lane=%c encoder_um" in f or "thresholds lane=%c" in f for f in mcu.subscription_formats
    )


def test_connect_asks_for_the_thresholds_once_the_subscriptions_are_live() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"]
    # and arms the one bounded re-query 1.5 s on
    assert len(printer.reactor.armed()) == 1
    assert printer.reactor.armed()[0][1] == printer.reactor.now + 1.5


def test_a_lost_thresholds_frame_is_asked_for_once_more_then_warned_about(
    caplog: pytest.LogCaptureFixture,
) -> None:
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


def test_a_thresholds_frame_that_arrives_means_no_re_query(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    printer.handlers["klippy:connect"]()
    mcu.subscriptions["ace2k_sensors_thresholds"](thresholds_frame([600] * 4, [1100] * 4, [1] * 4))
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(101.5) == FakeReactor.NEVER
    assert non_rfid_sent(mcu) == ["ace2k_sensors_thresholds_query"]  # the one from connect
    assert not non_rfid_warnings(caplog)


def test_a_thresholds_frame_that_arrives_after_the_re_query_ends_it_quietly(
    caplog: pytest.LogCaptureFixture,
) -> None:
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


def test_the_thresholds_arrive_as_one_frame_for_the_four_lanes() -> None:
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


def test_a_firmware_without_lane_and_mains_is_logged_and_left_out(
    caplog: pytest.LogCaptureFixture,
) -> None:
    known = {
        "ace2k_sensors_query",
        "ace2k_sensors_state",
        "ace2k_sensors_event",
        "ace2k_sensors_thresholds",
        "ace2k_sensors_thresholds_query",
        "ace2k_sensors_thresholds_reset",
        "ace2k_env_query",
        "ace2k_env_state",
        "ace2k_health_query",
        "ace2k_health_state",
        "ace2k_health_clear",
        "ace2k_health_run",
    }
    module, printer, mcu = make(RESPONSES, known=known)
    with caplog.at_level("WARNING", logger="root"):
        mcu.config_callback()
    init_cmds = [c for c, is_init in mcu.config_cmds if is_init]
    assert init_cmds == [
        "ace2k_sensors_thresholds_reset lane=0",
        "ace2k_sensors_thresholds_reset lane=1",
        "ace2k_sensors_thresholds_reset lane=2",
        "ace2k_sensors_thresholds_reset lane=3",
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


def test_a_firmware_without_sensors_sends_no_thresholds_and_no_connect_query(
    caplog: pytest.LogCaptureFixture,
) -> None:
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
        "ace2k_health_state",
        "ace2k_health_clear",
        "ace2k_health_run",
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


def test_sensors_state_decodes_switches_and_millivolts() -> None:
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


def test_sensors_reports_counts_each_state_report() -> None:
    # a reader telling two reports apart (the U1 adapter's eject probe): one count per report
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert module.get_status(0.0)["sensors_reports"] == 0
    frame = {
        "switches": 0,
        "insert_mv": b"\x00" * 8,
        "empty_mv": b"\x00" * 8,
        "aux_mv": 0,
        "age_ms": 1,
        "#sent_time": 1.0,
    }
    mcu.subscriptions["ace2k_sensors_state"](dict(frame))
    assert module.get_status(0.0)["sensors_reports"] == 1
    mcu.subscriptions["ace2k_sensors_state"](dict(frame))
    assert module.get_status(0.0)["sensors_reports"] == 2


def test_env_state_converts_units_and_notifies_listeners() -> None:
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


def test_humidity_is_none_while_the_chamber_bit_is_clear() -> None:
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
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert "humidity=None %" in gcmd.lines[0]
    frame["valid"] = 0xF
    mcu.subscriptions["ace2k_env_state"](frame)
    assert module.get_status(0.0)["humidity"] == 67.0


def test_lane_counters_and_mains_land_in_status() -> None:
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


def test_counters_reset_gcode() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()  # the subsystems are known from the config callback on
    reset = printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"]
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


def test_counters_reset_refused_by_the_unit_is_a_gcode_error() -> None:
    # the host's own picture is a 1 Hz report: a move the unit accepted since, or started
    # itself, is only known to the unit, which answers accepted=0 and changes nothing
    responses = dict(RESPONSES)
    responses["ace2k_lane_counters_reset"] = {"lane": 1, "accepted": 0}
    module, printer, mcu = make(responses)
    mcu.config_callback()
    assert module.reset_confirmed
    gcmd = FakeGcmd({"LANE": "2"})
    with pytest.raises(FakeGcmd.error, match="counters reset refused: a lane is moving or busy"):
        printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"](gcmd)
    assert mcu.queries == [("ace2k_lane_counters_reset", [1])]
    assert gcmd.lines == []  # no "counters reset" line for a refusal


def test_counters_reset_on_a_firmware_without_the_response_is_sent_unconfirmed(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # a v0.2.0 dictionary: the command, no response — a query wrapper on it would raise the
    # parser's error inside the G-code handler, which Klipper answers with a shutdown; the
    # absence is known at config time, logged, and the reset goes out as a plain command
    known = set(KNOWN_ALL_BUT_HEALTH) - {RESET_RESP.split()[0]}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    assert "answers no counters reset; ACEK_COUNTERS_RESET is sent unconfirmed" in caplog.text
    gcmd = FakeGcmd({"LANE": "2"})
    printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"](gcmd)
    assert mcu.queries == [] and mcu.commands == [(RESET_CMD, [1])]
    assert gcmd.lines == ["ace2k: counters reset sent (lane 2); this firmware does not confirm it"]


def test_counters_reset_response_with_another_format_is_a_config_error(
    caplog: pytest.LogCaptureFixture,
) -> None:
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


def test_a_failure_that_is_not_the_dictionarys_answer_propagates_from_the_config_callback() -> None:
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


def test_a_report_with_another_format_is_a_config_error_and_an_absent_one_a_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
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
    assert counters.split(maxsplit=1)[0] not in mcu.subscriptions


def test_refuse_format_returns_the_error_and_words_each_verdict() -> None:
    # returned, never raised: every call site reads "raise refuse_format(...)"
    printer = FakePrinter(FakeMcu(RESPONSES))
    err = refuse_format(
        printer, "ace2k", RESET_RESP, ("differs", "Command format mismatch: a vs b")
    )
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
        ("absent", "Unknown command: x"),
        "ace2k_feed_start lane=%c",
    )
    assert str(err) == (
        "ace2k: ace2k_feed_state is not in the unit's dictionary though ace2k_feed_start is"
        " (Unknown command: x); " + ace2k.FORMAT_HINT
    )
    # … and without one: the clause is dropped, nothing dereferenced
    err = refuse_format(
        printer, "ace2k", "ace2k_feed_state mode=%*s", ("absent", "Unknown command: x")
    )
    assert str(err) == (
        "ace2k: ace2k_feed_state is not in the unit's dictionary (Unknown command: x); "
        + ace2k.FORMAT_HINT
    )


def test_probe_format_tells_ok_differs_and_absent_and_defaults_to_absent() -> None:
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


def test_an_unrecognised_lookup_failure_falls_back_to_the_unconfirmed_reset(
    caplog: pytest.LogCaptureFixture,
) -> None:
    # a Klipper rewording of "Unknown command" must not block a connect that worked before: the
    # fallback, with the parser's text in the warning so the rewording is visible
    module, printer, mcu = make(
        RESPONSES,
        known=KNOWN_ALL_BUT_HEALTH,
        lookup_errors={RESET_RESP.split()[0]: "Nothing named that here"},
    )
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    assert "ACEK_COUNTERS_RESET is sent unconfirmed (Nothing named that here)" in caplog.text
    gcmd = FakeGcmd({"LANE": "2"})
    printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"](gcmd)
    assert mcu.commands == [(RESET_CMD, [1])]


def test_counters_reset_on_a_firmware_without_the_command_is_a_gcode_error() -> None:
    # older still: lane counters, but no reset at all — a G-code error, never the parser's
    # exception out of the handler
    known = set(KNOWN_ALL_BUT_HEALTH) - {RESET_CMD.split()[0], RESET_RESP.split()[0]}
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    assert "lane" in module.present and not module.reset_confirmed
    with pytest.raises(FakeGcmd.error, match="this firmware has no counters reset"):
        printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"](FakeGcmd({"LANE": "2"}))
    assert mcu.commands == [] and mcu.queries == []


def test_counters_reset_is_refused_when_the_firmware_has_no_lane_counters() -> None:
    known = {
        "ace2k_sensors_query",
        "ace2k_sensors_thresholds_query",
        "ace2k_env_query",
        "ace2k_mains_query",
    }
    module, printer, mcu = make(RESPONSES, known=known)
    mcu.config_callback()
    # a gcmd.error, not a bare msgparser error: the latter is not a CommandError, and Klipper
    # answers one from a G-code handler with a shutdown
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_LANE"):
        printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"](FakeGcmd({"LANE": "2"}))
    assert mcu.commands == [] and mcu.queries == [] and mcu.sent == []


def test_each_lane_counters_frame_reaches_the_listeners_with_its_receive_time() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    heard = []
    module.register_counters_listener(lambda rt, encoder_um: heard.append((rt, encoder_um)))
    frame = lane_counters_frame([1000, -2000, 0, 7], [0] * 4)
    mcu.subscriptions["ace2k_lane_counters"](dict(frame, **{"#receive_time": 12.5}))
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([1, 2, 3, 4], [0] * 4))
    assert heard == [(12.5, (1000, -2000, 0, 7)), (0.0, (1, 2, 3, 4))]  # none in the frame: 0.0


def test_the_feed_holds_the_unit_and_its_calibration_hears_the_counters() -> None:
    module, printer, mcu = make(RESPONSES)
    assert module.feed.unit is module
    assert module.counter_listeners == [module.feed.cal.on_counters]


def test_set_counters_period_sends_the_lane_counters_query_when_the_firmware_has_it() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    module.set_counters_period(0.1)
    queries = [c for c in mcu.commands if c[0].startswith("ace2k_lane_counters_query")]
    assert queries == [("ace2k_lane_counters_query rest_ticks=%u", [12000000])]
    module, printer, mcu = make(
        RESPONSES,
        known={"ace2k_sensors_query", "ace2k_sensors_thresholds_query", "ace2k_env_query"},
    )
    mcu.config_callback()
    module.set_counters_period(0.1)  # a firmware without the lane counters: nothing to set
    assert mcu.commands == [] and mcu.sent == []


def test_a_live_scale_change_keeps_encoder_mm_continuous() -> None:
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


def test_a_frame_the_unit_sent_before_the_change_reads_on_at_the_old_offset() -> None:
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


def test_a_change_still_pending_a_report_period_on_applies_with_the_next_frame() -> None:
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


def test_a_second_change_before_a_frame_at_the_first_runs_from_the_scale_frames_showed() -> None:
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


def test_an_accepted_counters_reset_drops_the_offset_of_the_lanes_it_reset() -> None:
    responses = dict(RESPONSES)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    counters = mcu.subscriptions["ace2k_lane_counters"]
    counters(lane_counters_frame([1000000] * 4, [0] * 4))
    for lane in range(4):
        module.keep_encoder_continuous(lane, 1.2342, 1.2468)
    reset = printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"]
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


def test_an_unconfirmed_counters_reset_drops_the_offset_and_a_pending_change_too() -> None:
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
    reset = printer.objects["gcode"].commands["ACEK_COUNTERS_RESET"]
    reset(FakeGcmd({"LANE": "1"}))
    assert module.encoder_offset_um == [0, 0, 0, 0] and module.scale_pending[1] is not None
    reset(FakeGcmd({"LANE": "ALL"}))
    assert module.scale_pending == [None] * 4
    assert mcu.commands == [(RESET_CMD, [0]), (RESET_CMD, [255])]


def test_calibration_save_stages_every_lane() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    mcu.subscriptions["ace2k_sensors_thresholds"](
        thresholds_frame([600, 601, 602, 603], [1100] * 4, [1] * 4)
    )
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_CALIBRATION_SAVE"](gcmd)
    assert printer.objects["configfile"].calls[0] == ("ace2k", "lane1_insert_mv", "600, 1100")
    assert printer.objects["configfile"].calls[3] == ("ace2k", "lane4_insert_mv", "603, 1100")
    assert "SAVE_CONFIG" in gcmd.lines[0]
    assert module.get_status(0.0)["lanes"][2]["source"] == "factory"


def test_calibration_save_before_the_thresholds_is_a_gcode_error() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    gcmd = FakeGcmd()
    with pytest.raises(FakeGcmd.error) as e:
        printer.objects["gcode"].commands["ACEK_CALIBRATION_SAVE"](gcmd)
    assert str(e.value) == "ace2k: thresholds not received yet"
    assert printer.objects["configfile"].calls == []
    assert gcmd.lines == []


def test_status_gcode_prints_lanes_and_temperatures() -> None:
    module, printer, mcu = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    text = gcmd.lines[0]
    assert text.startswith("ace2k 0.1.0, link proven: yes")
    assert "lane 1:" in text and "lane 4:" in text and "ptc_left=" in text
    assert "encoder=None mm fg=None" in text and "mains=None Hz (no report yet)" in text
    mcu.subscriptions["ace2k_lane_counters"](lane_counters_frame([0, 0, 0, 12342], [0, 0, 0, 7]))
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 499, "present": 1, "rejects": 0})
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    text = gcmd.lines[0]
    assert "lane 4:" in text and "encoder=12.342 mm fg=7" in text
    assert "mains=49.9 Hz (present) rejects=0" in text
    mcu.subscriptions["ace2k_mains_state"]({"hz10": 1, "present": 1, "rejects": 0})
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert "mains=missed edge (present) rejects=0" in gcmd.lines[0]


def test_health_gcode_prints_names() -> None:
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
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    assert self_test_sent(mcu) == ["ace2k_health_query"]
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


def test_health_gcode_says_ok_when_nothing_fails() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: OK"
    assert lines[1] == "failed since boot or last clear: none"
    assert module.get_status(0.0)["health"]["ok"] is True


def test_health_run_and_clear_send_their_commands() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    gcmd = FakeGcmd({"RUN": "1"})
    health(gcmd)
    assert mcu.commands == [("ace2k_health_run", [])]
    # nothing read yet: the current pass is read first, so the re-read can tell the new one
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]
    assert gcmd.lines == ["ace2k: self-test started; results in about 2 s"]
    gcmd = FakeGcmd({"CLEAR": "1"})
    health(gcmd)
    assert mcu.commands[-2:] == [("ace2k_health_clear", []), ("ace2k_link_gap_clear", [])]
    # cleared, then shown
    assert self_test_sent(mcu)[-2:] == ["ace2k_health_clear", "ace2k_health_query"]
    assert gcmd.lines[0].startswith("health: OK")
    gcmd = FakeGcmd({"CLEAR": "1", "RUN": "1"})
    health(gcmd)
    # with RUN=1 the clear waits for the re-read of the new pass (test_health_clear_with_run_*)
    assert self_test_sent(mcu)[-1] == "ace2k_health_run"
    assert mcu.commands[-1] == ("ace2k_health_run", [])


LINK_GAP = {"max_ms": 2140, "lost_ms": 3000}
# An image from before the silence on the link was measured (API_VERSION 23)
NO_LINK_GAP = {
    "ace2k_link_gap_query": "Unknown command: ace2k_link_gap_query",
    "ace2k_link_gap_state": "Unknown command: ace2k_link_gap_state",
    "ace2k_link_gap_clear": "Unknown command: ace2k_link_gap_clear",
}


def test_health_reads_the_longest_silence_of_the_host() -> None:
    responses = dict(RESPONSES, ace2k_link_gap_query=LINK_GAP)
    module, printer, mcu = make(responses)
    mcu.config_callback()
    h = module.get_status(0.0)["health"]
    assert (h["link_gap_ms"], h["link_lost_ms"]) == (None, None)  # before the first answer
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_health_query", "ace2k_link_gap_query"]
    lines = gcmd.lines[0].split("\n")
    assert lines[3] == (
        "longest host silence since boot or last clear: 2140 ms (the link is lost at 3000 ms)"
    )
    h = module.get_status(0.0)["health"]
    assert (h["link_gap_ms"], h["link_lost_ms"]) == (2140, 3000)


def test_health_on_an_image_without_the_silence_says_n_a() -> None:
    module, printer, mcu = make(RESPONSES, lookup_errors=NO_LINK_GAP)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    assert non_rfid_sent(mcu) == ["ace2k_health_query"]
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: OK"
    assert lines[3] == "longest host silence: n/a (the unit's firmware does not measure it)"
    h = module.get_status(0.0)["health"]
    assert (h["link_gap_ms"], h["link_lost_ms"]) == (None, None)
    gcmd = FakeGcmd({"CLEAR": "1"})
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    assert mcu.sent[-2:] == ["ace2k_health_clear", "ace2k_health_query"]


def test_health_clear_clears_the_longest_silence_too() -> None:
    responses = dict(RESPONSES, ace2k_link_gap_query=[LINK_GAP, {"max_ms": 40, "lost_ms": 3000}])
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    health(FakeGcmd())
    gcmd = FakeGcmd({"CLEAR": "1"})
    health(gcmd)
    assert mcu.sent[-4:] == [
        "ace2k_health_clear",
        "ace2k_link_gap_clear",
        "ace2k_health_query",
        "ace2k_link_gap_query",
    ]
    line = gcmd.lines[0].split("\n")[3]
    assert line.startswith("longest host silence since boot or last clear: 40 ms")
    assert module.get_status(0.0)["health"]["link_gap_ms"] == 40


def test_an_unanswered_silence_query_keeps_the_health_and_reads_the_silence_unknown(
    caplog: pytest.LogCaptureFixture,
) -> None:
    responses = dict(
        RESPONSES,
        ace2k_link_gap_query=no_answer("ace2k_link_gap_state"),
    )
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcmd = FakeGcmd()
    with caplog.at_level("WARNING", logger="root"):
        printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: OK"  # the self-test answered: its verdict stands
    assert lines[3] == "longest host silence: unknown (no answer from the unit)"
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"], h["link_gap_ms"], h["link_lost_ms"]) == (True, False, None, None)
    assert any("link gap" in w for w in non_rfid_warnings(caplog))


def test_a_poll_whose_silence_query_goes_unanswered_is_no_poll_failure() -> None:
    responses = dict(
        RESPONSES,
        ace2k_link_gap_query=[no_answer("ace2k_link_gap_state"), LINK_GAP],
    )
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    cb, _ = printer.reactor.armed()[0]
    assert cb(105.0) == 115.0
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"], h["link_gap_ms"]) == (True, False, None)
    assert module.health_poll_failures == 0
    assert cb(115.0) == 125.0
    assert module.get_status(0.0)["health"]["link_gap_ms"] == 2140


def test_a_silence_answer_of_another_format_refuses_the_connect() -> None:
    module, printer, mcu = make(
        RESPONSES, formats={"ace2k_link_gap_state": "ace2k_link_gap_state max_ms=%u"}
    )
    with pytest.raises(Exception, match="ace2k_link_gap_state.*format mismatch"):
        mcu.config_callback()


def test_health_gcode_is_refused_when_the_firmware_has_no_self_test() -> None:
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    mcu.config_callback()
    assert "health" not in module.present
    with pytest.raises(FakeGcmd.error, match="CONFIG_ACE2K_HEALTH"):
        printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd())
    assert mcu.commands == [] and mcu.sent == []


def test_uid_is_hex_in_status() -> None:
    module, printer, mcu = make(RESPONSES)
    assert module.get_status(0.0)["uid"] is None
    printer.handlers["klippy:mcu_identify"]()
    assert module.get_status(0.0)["uid"] == SYNTHETIC_UID.hex()
    assert module.get_status(0.0)["uid"] == "1112131415161718191a1b1c"


def test_a_configured_uid_that_matches_connects_whatever_its_case_and_separators() -> None:
    h = SYNTHETIC_UID.hex().upper()
    for written in (
        " ".join(h[i : i + 8] for i in range(0, 24, 8)),
        ":".join(h[i : i + 2] for i in range(0, 24, 2)),
        h[:12] + "-" + h[12:],
    ):
        module, printer, mcu = make(RESPONSES, options={"uid": written})
        printer.handlers["klippy:mcu_identify"]()
        printer.handlers["klippy:connect"]()  # no exception
        assert module.get_status(0.0)["uid"] == SYNTHETIC_UID.hex()


def test_a_configured_uid_that_differs_is_a_config_error_at_connect_naming_the_cables() -> None:
    module, printer, mcu = make(RESPONSES, options={"uid": "00" * 12})
    printer.handlers["klippy:mcu_identify"]()  # identify proves the link and raises nothing
    assert non_rfid_sent(mcu) == ["ace2k_version", "ace2k_linkproof_state", "ace2k_uid_query"]
    with pytest.raises(Exception) as err:
        printer.handlers["klippy:connect"]()
    text = str(err.value)
    assert text.startswith("ace2k: ")
    assert SYNTHETIC_UID.hex() in text and "00" * 12 in text
    assert "cables" in text


@pytest.mark.parametrize("bad", ["1234", "zz" * 12, "11" * 13, "", "0x" + "11" * 11])
def test_a_malformed_uid_is_a_config_error_at_load(bad: str) -> None:
    with pytest.raises(Exception, match="uid"):
        make(RESPONSES, options={"uid": bad})


def test_a_configured_uid_on_a_firmware_without_the_uid_query_is_a_config_error() -> None:
    module, printer, mcu = make(
        RESPONSES, options={"uid": SYNTHETIC_UID.hex()}, known=KNOWN_ALL_BUT_HEALTH
    )
    printer.handlers["klippy:mcu_identify"]()
    with pytest.raises(Exception, match="cannot be checked: the unit's firmware has no"):
        printer.handlers["klippy:connect"]()


def test_a_short_identifier_answer_is_named_as_a_fault_not_as_swapped_cables(
    caplog: pytest.LogCaptureFixture,
) -> None:
    responses = dict(RESPONSES, ace2k_uid_query={"uid": SYNTHETIC_UID[:5]})
    module, printer, mcu = make(responses, options={"uid": SYNTHETIC_UID.hex()})
    printer.handlers["klippy:mcu_identify"]()
    assert module.get_status(0.0)["uid"] is None
    assert "5 bytes long, not 12" in caplog.text
    with pytest.raises(Exception) as err:
        printer.handlers["klippy:connect"]()
    assert "5 bytes long" in str(err.value) and "cables" not in str(err.value)


def test_a_short_identifier_answer_without_uid_set_is_only_a_warning() -> None:
    responses = dict(RESPONSES, ace2k_uid_query={"uid": b""})
    module, printer, mcu = make(responses)
    printer.handlers["klippy:mcu_identify"]()
    printer.handlers["klippy:connect"]()  # no exception
    assert module.get_status(0.0)["uid"] is None


def test_swapped_cables_prove_every_unit_then_name_both_and_whose_uid_each_is() -> None:
    uid_a, uid_b = bytes(range(1, 13)), bytes(range(21, 33))
    units, printer, mcus = make_units(
        options={"a": {"uid": uid_a.hex()}, "b": {"uid": uid_b.hex()}}
    )
    # the cables swapped: section a's port reaches unit b, and b's reaches a
    mcus["a"].responses["ace2k_uid_query"] = {"uid": uid_b}
    mcus["b"].responses["ace2k_uid_query"] = {"uid": uid_a}
    for unit in units:
        unit._on_mcu_identify()  # neither raises: both units are proven
    for mcu in mcus.values():
        assert "ace2k_version" in non_rfid_sent(mcu)
    with pytest.raises(Exception) as err:
        units[0]._on_connect()
    first, second = str(err.value).split("\n")
    assert first.startswith("ace2k a: ") and "(the uid: of [ace2k b])" in first
    assert second.startswith("ace2k b: ") and "(the uid: of [ace2k a])" in second


def test_status_gcode_shows_the_uid_on_the_line_after_the_version() -> None:
    module, printer, _ = make(RESPONSES)
    printer.handlers["klippy:mcu_identify"]()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    first, second = gcmd.lines[0].split("\n")[:2]
    assert first == "ace2k 0.1.0, link proven: yes"  # unchanged: tools read it
    assert second == f"uid {SYNTHETIC_UID.hex()}"


def test_status_gcode_has_no_uid_line_on_a_firmware_without_the_query() -> None:
    module, printer, _ = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    printer.handlers["klippy:mcu_identify"]()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_STATUS"](gcmd)
    assert gcmd.lines[0].split("\n")[1].startswith("lane 1:")


def test_an_unanswered_uid_query_leaves_the_uid_unknown_with_a_warning_not_an_error(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """After M112 and the init script's restart the units are
    still shut down, and the identifier query went unanswered — a second error beside Klipper's
    own "MCU shutdown". It is a warning now; the uid stays unknown."""
    module, printer, mcu = make(RESPONSES)

    def no_answer(name: str, *args: object, **kwargs: object) -> NoReturn:
        raise FakeCommandError("Unable to obtain 'ace2k_uid_response' response")

    real = mcu.lookup_query_command

    def lookup(msgformat: str, respformat: str, *args: object, **kwargs: object) -> FakeQuery:
        query = real(msgformat, respformat, *args, **kwargs)
        if msgformat.startswith("ace2k_uid_query"):
            query.send = lambda *a, **k: no_answer("ace2k_uid_query")
        return query

    mcu.lookup_query_command = lookup
    printer.handlers["klippy:mcu_identify"]()  # no exception
    assert module.get_status(0.0)["uid"] is None
    assert "the unit's identifier was not read" in caplog.text
    assert module.get_status(0.0)["link_proven"] is True
    assert module.uid_answer_bytes is None


def test_an_unanswered_uid_query_with_uid_set_says_it_was_not_read_not_that_the_firmware_lacks_it() -> (  # noqa: E501 — the name is the test's sentence
    None
):
    module, printer, mcu = make(RESPONSES, options={"uid": SYNTHETIC_UID.hex()})
    real = mcu.lookup_query_command

    def lookup(msgformat: str, respformat: str, *args: object, **kwargs: object) -> FakeQuery:
        query = real(msgformat, respformat, *args, **kwargs)
        if msgformat.startswith("ace2k_uid_query"):

            def no_answer(*a: object, **k: object) -> NoReturn:
                raise FakeCommandError("Unable to obtain 'ace2k_uid_response' response")

            query.send = no_answer
        return query

    mcu.lookup_query_command = lookup
    printer.handlers["klippy:mcu_identify"]()
    with pytest.raises(Exception) as err:
        printer.handlers["klippy:connect"]()
    text = str(err.value)
    assert "did not answer its identifier query" in text and "FIRMWARE_RESTART" in text
    assert "CONFIG_ACE2K_HEALTH" not in text


def test_uid_stays_none_when_the_firmware_has_no_self_test() -> None:
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    printer.handlers["klippy:mcu_identify"]()  # no exception, no UID query
    assert non_rfid_sent(mcu) == ["ace2k_version", "ace2k_linkproof_state"]
    assert module.get_status(0.0)["uid"] is None


def test_health_poll_starts_at_ready_and_repeats() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert printer.reactor.armed() == []
    fire_ready(printer)
    assert len(printer.reactor.armed()) == 1
    cb, waketime = printer.reactor.armed()[0]
    assert waketime == printer.reactor.now + 5.0
    assert module.get_status(0.0)["health"]["ok"] is None  # nothing polled yet
    assert cb(105.0) == 115.0
    assert self_test_sent(mcu) == ["ace2k_health_query"]
    st = module.get_status(0.0)["health"]
    assert (st["ok"], st["failing"], st["latched"]) == (True, [], [])
    assert (st["post_ms"], st["reset_cause"]) == (2010, "power-on")


def test_health_poll_survives_one_failure_and_goes_on(caplog: pytest.LogCaptureFixture) -> None:
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
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_query"]
    assert module.get_status(0.0)["health"]["ok"] is True
    assert module.health_poll_failures == 0  # a success resets the streak
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "health poll failed" in warnings[0]


def test_health_poll_stops_after_five_consecutive_failures(
    caplog: pytest.LogCaptureFixture,
) -> None:
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


def test_a_failed_poll_after_a_success_reads_unknown_not_healthy(
    caplog: pytest.LogCaptureFixture,
) -> None:
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


def test_overlapping_health_queries_take_turns_on_one_mutex() -> None:
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

    def answer() -> dict[str, int]:
        depth_at_send.append(mutex.depth)
        if len(depth_at_send) == 1:
            poll_result.append(poll(105.0))  # the poll fires while this query is in flight
        return HEALTHY

    mcu.responses["ace2k_health_query"] = answer
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    assert gcmd.lines[0].startswith("health: OK")
    assert poll_result == [115.0]  # the poll completed and stays armed
    assert depth_at_send == [1, 2]  # both sent under the mutex; the second waited for it
    assert mutex.contended == 1 and mutex.depth == 0
    assert self_test_sent(mcu) == ["ace2k_health_query"] * 2
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"]) == (True, False)
    assert module.health_poll_failures == 0


def test_health_gcode_says_unknown_when_the_unit_does_not_answer() -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
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


def test_health_not_run_yet_while_post_ms_is_zero() -> None:
    # post_ms = 0: the firmware's full evaluation has not run (the first is 2 s after boot),
    # so `now` says nothing about the unit yet
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = {"now": 0, "latched": 0, "post_ms": 0, "reset_cause": 3}
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcmd = FakeGcmd()
    printer.objects["gcode"].commands["ACEK_HEALTH"](gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: not run yet"
    assert lines[1] == "failed since boot or last clear: none"
    assert lines[2] == "last reset: software"  # the reset cause still printed, no "at 0 ms"
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["failing"], h["post_ms"], h["reset_cause"]) == (None, [], 0, "software")


def test_health_run_re_queries_once_after_the_pass() -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [HEALTHY, dict(HEALTHY, post_ms=4530)]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    assert poll(105.0) == 115.0  # the pass read before the run: post_ms 2010
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    # no query with the run
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]
    cb, waketime = armed_timer(printer, module._requery_health)
    assert waketime == printer.reactor.now + 2.5
    assert module.get_status(0.0)["health"]["post_ms"] == 2010
    assert cb(102.5) == FakeReactor.NEVER  # one shot: the pass moved
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_run", "ace2k_health_query"]
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["post_ms"]) == (True, 4530)


def test_health_run_before_any_poll_reads_the_current_pass_first() -> None:
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
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    gcmd = FakeGcmd({"RUN": "1"})
    health(gcmd)
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]  # the query first
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
        self_test_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 2
    )


def test_health_run_is_refused_when_the_baseline_query_goes_unanswered() -> None:
    # no baseline, no run: a re-read with nothing to compare against would accept the boot
    # pass as the new one; the G-code says so and sends nothing
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = FakeCommandError(
        "Timeout on wait for 'ace2k_health_state' response"
    )
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    with pytest.raises(FakeGcmd.error, match="self-test not started"):
        health(FakeGcmd({"RUN": "1"}))
    assert non_rfid_sent(mcu) == ["ace2k_health_query"]
    assert mcu.commands == []
    assert printer.reactor.armed() == []  # the re-read timer is not armed
    h = module.get_status(0.0)["health"]
    assert (h["ok"], h["stale"]) == (None, True)


def test_health_run_re_queries_again_while_the_unit_still_reports_the_previous_pass() -> None:
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
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
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
        self_test_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 2
    )


def test_health_run_re_query_gives_up_after_three_retries_with_a_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    module, printer, mcu = make(RESPONSES)  # HEALTHY forever: post_ms never moves
    mcu.config_callback()
    fire_ready(printer)
    poll, _ = printer.reactor.armed()[0]
    assert poll(105.0) == 115.0
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
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
        self_test_sent(mcu)
        == ["ace2k_health_query", "ace2k_health_run"] + ["ace2k_health_query"] * 4
    )


def test_health_run_re_arms_one_reactor_timer_instead_of_registering_one_per_run() -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [HEALTHY, dict(HEALTHY, post_ms=204010)]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    registered = len(printer.reactor.timers)  # the re-query timer exists from the start, unarmed
    assert printer.reactor.armed() == []
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    health(FakeGcmd({"RUN": "1"}))
    printer.reactor.now = 200.0
    health(FakeGcmd({"RUN": "1"}))
    assert len(printer.reactor.timers) == registered  # nothing new registered
    assert len(printer.reactor.armed()) == 1
    cb, waketime = printer.reactor.armed()[0]
    assert waketime == 202.5  # re-armed from the second RUN
    assert cb(202.5) == FakeReactor.NEVER


ENC1_FG1 = (1 << 17) | (1 << 21)  # encoder1, fg1: the motor moving in the window


def test_health_clear_with_run_clears_after_the_new_pass_is_read() -> None:
    # CLEAR=1 RUN=1 after a pass that failed: the clear waits for the re-read of the new pass,
    # so it sets latched to the new pass, not to the one that failed; a good pass reads none
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        {"now": ENC1_FG1, "latched": ENC1_FG1, "post_ms": 2010, "reset_cause": 1},  # baseline
        {"now": 0, "latched": ENC1_FG1, "post_ms": 4530, "reset_cause": 1},  # the new pass
        {"now": 0, "latched": 0, "post_ms": 4530, "reset_cause": 1},  # read after the clear
        {"now": 0, "latched": 0, "post_ms": 4530, "reset_cause": 1},  # ACEK_HEALTH
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    gcmd = FakeGcmd({"CLEAR": "1", "RUN": "1"})
    health(gcmd)
    assert self_test_sent(mcu) == ["ace2k_health_query", "ace2k_health_run"]  # no clear yet
    assert gcmd.lines == ["ace2k: self-test started; results in about 2 s"]
    cb, _ = armed_timer(printer, module._requery_health)
    assert cb(102.5) == FakeReactor.NEVER
    assert self_test_sent(mcu) == [
        "ace2k_health_query",
        "ace2k_health_run",
        "ace2k_health_query",  # the new pass
        "ace2k_health_clear",
        "ace2k_health_query",  # latched after the clear
    ]
    # the longest silence is cleared with the faults, after the new pass (test_*silence*)
    assert mcu.sent[-4:-2] == ["ace2k_health_clear", "ace2k_link_gap_clear"]
    assert module.get_status(0.0)["health"]["latched"] == []
    gcmd = FakeGcmd()
    health(gcmd)
    lines = gcmd.lines[0].split("\n")
    assert lines[0] == "health: OK"
    assert lines[1] == "failed since boot or last clear: none"


def test_health_clear_with_run_keeps_what_the_new_pass_fails() -> None:
    # the new pass fails too: the clear after it leaves latched = that pass, as a clear does
    zc = 1 << 6
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        {"now": ENC1_FG1, "latched": ENC1_FG1, "post_ms": 2010, "reset_cause": 1},
        {"now": zc, "latched": ENC1_FG1 | zc, "post_ms": 4530, "reset_cause": 1},
        {"now": zc, "latched": zc, "post_ms": 4530, "reset_cause": 1},
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd({"CLEAR": "1", "RUN": "1"}))
    cb, _ = armed_timer(printer, module._requery_health)
    assert cb(102.5) == FakeReactor.NEVER
    assert self_test_sent(mcu)[-3:] == [
        "ace2k_health_query",
        "ace2k_health_clear",
        "ace2k_health_query",
    ]
    h = module.get_status(0.0)["health"]
    assert (h["failing"], h["latched"]) == (["zerocross"], ["zerocross"])


def test_health_clear_with_run_is_dropped_when_the_retries_give_up() -> None:
    # the unit never reports a new pass: a clear now would latch the previous pass, the one the
    # wait is there to skip, so nothing is cleared and the console says so
    module, printer, mcu = make(RESPONSES)  # HEALTHY forever: post_ms never moves
    mcu.config_callback()
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_HEALTH"](FakeGcmd({"CLEAR": "1", "RUN": "1"}))
    cb, _ = armed_timer(printer, module._requery_health)
    assert cb(102.5) == 103.5
    assert cb(103.5) == 104.5
    assert cb(104.5) == 105.5
    assert cb(105.5) == FakeReactor.NEVER
    assert "ace2k_health_clear" not in non_rfid_sent(mcu)
    assert gcode.output[-1] == "ace2k: health not cleared: the unit did not report a new self-test"
    assert module.health_rerun_clear is False


def test_health_clear_with_run_is_dropped_when_the_re_read_goes_unanswered(
    caplog: pytest.LogCaptureFixture,
) -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the baseline RUN=1 reads first
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
        dict(HEALTHY, post_ms=4530),  # the re-read of a later run without CLEAR
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    health = printer.objects["gcode"].commands["ACEK_HEALTH"]
    health(FakeGcmd({"CLEAR": "1", "RUN": "1"}))
    cb, _ = armed_timer(printer, module._requery_health)
    with caplog.at_level("WARNING", logger="root"):
        assert cb(102.5) == FakeReactor.NEVER
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "faults not cleared" in warnings[0]
    assert "ace2k_health_clear" not in non_rfid_sent(mcu)
    gcode = printer.objects["gcode"]
    assert gcode.output[-1] == "ace2k: health not cleared: the unit did not answer"
    health(FakeGcmd({"RUN": "1"}))  # the dropped clear does not ride on the next run
    cb, _ = armed_timer(printer, module._requery_health)
    assert cb(110.0) == FakeReactor.NEVER
    assert "ace2k_health_clear" not in non_rfid_sent(mcu)


def test_health_clear_with_run_says_when_the_read_after_the_clear_goes_unanswered() -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the baseline RUN=1 reads first
        dict(HEALTHY, post_ms=4530),  # the new pass
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_HEALTH"](FakeGcmd({"CLEAR": "1", "RUN": "1"}))
    cb, _ = armed_timer(printer, module._requery_health)
    assert cb(102.5) == FakeReactor.NEVER
    assert self_test_sent(mcu)[-2:] == ["ace2k_health_clear", "ace2k_health_query"]
    assert gcode.output[-1] == (
        "ace2k: health cleared, but the unit did not answer the read after it"
    )


def test_health_run_without_clear_says_nothing_on_the_console() -> None:
    module, printer, mcu = make(RESPONSES)  # HEALTHY forever: post_ms never moves
    mcu.config_callback()
    gcode = printer.objects["gcode"]
    gcode.commands["ACEK_HEALTH"](FakeGcmd({"RUN": "1"}))
    before = list(gcode.output)
    cb, _ = armed_timer(printer, module._requery_health)
    for t in (102.5, 103.5, 104.5, 105.5):
        cb(t)
    assert gcode.output == before


def test_health_re_query_after_run_logs_a_failure_instead_of_raising(
    caplog: pytest.LogCaptureFixture,
) -> None:
    responses = dict(RESPONSES)
    responses["ace2k_health_query"] = [
        HEALTHY,  # the baseline RUN=1 reads first
        FakeCommandError("Timeout on wait for 'ace2k_health_state' response"),
    ]
    module, printer, mcu = make(responses)
    mcu.config_callback()
    printer.objects["gcode"].commands["ACEK_HEALTH"](FakeGcmd({"RUN": "1"}))
    assert module.get_status(0.0)["health"]["ok"] is True
    cb, _ = printer.reactor.armed()[0]
    with caplog.at_level("WARNING", logger="root"):
        assert cb(102.5) == FakeReactor.NEVER
    warnings = non_rfid_warnings(caplog)
    assert len(warnings) == 1 and "re-query after RUN failed" in warnings[0]
    assert module.get_status(0.0)["health"]["ok"] is None


def test_no_health_poll_when_the_firmware_has_no_self_test() -> None:
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    mcu.config_callback()
    fire_ready(printer)
    assert printer.reactor.armed() == []


def test_bad_threshold_pair_is_a_config_error() -> None:
    with pytest.raises(Exception, match="lane1_insert_mv"):
        make(RESPONSES, options={"lane1_insert_mv": [1100, 600]})


def test_the_sensor_type_is_registered_by_the_section() -> None:
    _, printer, _ = make(RESPONSES)
    assert "ace2k" in printer.objects["heaters"].factories


def test_subscribe_response_uses_the_newer_method_when_klipper_has_it() -> None:
    mcu = FakeMcu(RESPONSES)

    def cb(params: Params) -> None:
        return None

    ace2k.subscribe_response(mcu, cb, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions["ace2k_env_state"] is cb
    assert mcu.subscription_formats == ["ace2k_env_state ptc_left_mc=%i"]


def test_subscribe_response_falls_back_to_register_response_by_name() -> None:
    mcu = LegacyFakeMcu(RESPONSES)

    def cb(params: Params) -> None:
        return None

    ace2k.subscribe_response(mcu, cb, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions == {"ace2k_env_state": cb}
    assert mcu.subscription_formats == ["ace2k_env_state"]


def test_subscribe_response_on_a_legacy_klipper_refuses_a_format_the_dictionary_differs_on() -> (
    None
):
    mcu = LegacyFakeMcu(RESPONSES, formats={"ace2k_env_state": "ace2k_env_state other=%c"})
    with pytest.raises(msgproto.error):
        ace2k.subscribe_response(mcu, lambda p: None, "ace2k_env_state ptc_left_mc=%i")
    assert mcu.subscriptions == {}


# --- the motor's travel each way (ace2k_lane_motor, host interface 25) --------------------------


def motor_frame(fwd_um: Sequence[int], back_um: Sequence[int]) -> dict[str, bytes]:
    """One ace2k_lane_motor frame: four uint32 LE forward, four uint32 LE back."""
    return {"fwd_um": struct.pack("<4I", *fwd_um), "back_um": struct.pack("<4I", *back_um)}


def motor_queries(mcu: FakeMcu) -> list[list[Any]]:
    return [data for fmt, data in mcu.commands if fmt.startswith("ace2k_lane_motor_query")]


def test_the_motor_report_is_off_until_a_watch_and_counted_per_watcher() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    assert not any(c.startswith("ace2k_lane_motor_query") for c, _ in mcu.config_cmds)
    assert "ace2k_lane_motor fwd_um=%*s back_um=%*s" in mcu.subscription_formats
    feed = module.feed
    assert feed.has_motor_travel is True
    assert feed.motor_watch(True) is True
    assert feed.motor_watch(True) is True  # a second watcher: the report is already on
    assert motor_queries(mcu) == [[120000000]]  # 1 Hz
    feed.motor_watch(False)
    assert motor_queries(mcu) == [[120000000]]  # one watcher left
    feed.motor_watch(False)
    assert motor_queries(mcu) == [[120000000], [0]]
    feed.motor_watch(False)  # one too many: nothing more goes out
    assert motor_queries(mcu) == [[120000000], [0]]


def test_the_motor_travel_lands_in_the_lane_status_continuous_across_the_wrap() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    module.feed.motor_watch(True)
    lane = module.get_status(0.0)["lanes"][0]
    assert (lane["motor_fwd_mm"], lane["motor_back_mm"]) == (None, None)
    report = mcu.subscriptions["ace2k_lane_motor"]
    report(motor_frame([100000, 0, 0, 0], [2**32 - 1000, 0, 0, 0]))
    lane = module.get_status(0.0)["lanes"][0]
    assert (lane["motor_fwd_mm"], lane["motor_back_mm"]) == (100.0, 4294966.296)
    report(motor_frame([112345, 0, 0, 0], [500, 0, 0, 0]))  # the back count wrapped
    st = module.get_status(0.0)["lanes"]
    assert (st[0]["motor_fwd_mm"], st[0]["motor_back_mm"]) == (112.345, 4294967.796)
    assert (st[1]["motor_fwd_mm"], st[1]["motor_back_mm"]) == (0.0, 0.0)


def test_a_new_watch_starts_from_its_own_first_report() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    feed, report = module.feed, mcu.subscriptions["ace2k_lane_motor"]

    def travel() -> tuple[object, object, object]:
        status = module.get_status(0.0)
        lane = status["lanes"][0]
        return lane["motor_fwd_mm"], lane["motor_back_mm"], status["motor_reports"]

    assert travel() == (None, None, 0)
    feed.motor_watch(True)
    report(motor_frame([100000, 0, 0, 0], [0, 0, 0, 0]))
    assert travel() == (100.0, 0.0, 1)
    feed.motor_watch(False)
    assert travel() == (None, None, 1)  # no watch: no reading that could be stale
    report(motor_frame([150000, 0, 0, 0], [0, 0, 0, 0]))  # a frame sent before the off
    assert travel() == (None, None, 1)
    # Hours of printing later, the next watch reads from its own first report: a consumer that
    # takes the difference of two reports never sees the travel between the watches.
    feed.motor_watch(True)
    assert travel() == (None, None, 1)
    report(motor_frame([900000000, 0, 0, 0], [7000, 0, 0, 0]))
    assert travel() == (900000.0, 7.0, 2)
    report(motor_frame([900005000, 0, 0, 0], [7000, 0, 0, 0]))
    assert travel() == (900005.0, 7.0, 3)


def test_a_motor_watch_whose_send_fails_is_not_counted() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    feed, cmd = module.feed, module.motor_cmd
    sent = cmd.send

    def refuse(data: Sequence[Any] = (), minclock: int = 0, reqclock: int = 0) -> None:
        raise RuntimeError("the link is gone")

    cmd.send = refuse
    with pytest.raises(RuntimeError):
        feed.motor_watch(True)
    cmd.send = sent
    assert feed.motor_watch(True) is True  # still the first watcher: the report goes on
    assert motor_queries(mcu) == [[120000000]]
    cmd.send = refuse
    with pytest.raises(RuntimeError):
        feed.motor_watch(False)
    cmd.send = sent
    feed.motor_watch(False)  # still the last watcher: the report goes off
    assert motor_queries(mcu) == [[120000000], [0]]


def test_a_connect_forgets_the_motor_watchers() -> None:
    module, printer, mcu = make(RESPONSES)
    mcu.config_callback()
    feed = module.feed
    feed.motor_watch(True)
    mcu.subscriptions["ace2k_lane_motor"](motor_frame([5000, 0, 0, 0], [0, 0, 0, 0]))
    mcu.config_callback()  # the MCU restarted: its report is off again
    lane = module.get_status(0.0)["lanes"][0]
    assert (lane["motor_fwd_mm"], lane["motor_back_mm"]) == (None, None)
    feed.motor_watch(True)
    assert motor_queries(mcu) == [[120000000], [120000000]]


def test_an_older_image_has_no_motor_travel_and_a_watch_does_nothing() -> None:
    module, printer, mcu = make(RESPONSES, known=KNOWN_ALL_BUT_HEALTH)
    mcu.config_callback()
    feed = module.feed
    assert feed.has_motor_travel is False
    assert feed.motor_watch(True) is False
    feed.motor_watch(False)
    assert motor_queries(mcu) == []
    lane = module.get_status(0.0)["lanes"][0]
    assert (lane["motor_fwd_mm"], lane["motor_back_mm"]) == (None, None)
