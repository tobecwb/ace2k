"""Descriptions of the Klipper objects the host modules use, for the type checker only.

Other modules import this one under ``if TYPE_CHECKING:``. Nothing here runs on the printer.

Each class lists the methods that the ace2k host modules call on that Klipper object. The
signatures are the ones Klipper uses (klippy/configfile.py, klippy.py, reactor.py, gcode.py,
mcu.py, toolhead.py, extras/heaters.py, extras/print_stats.py, kinematics/extruder.py).

Some values cross the boundary of Klipper without a type. They are a parsed response, a timer
handle and the reply of a command wrapper. They are ``Any``. So are the dicts that the host
modules pass to Klipper, or read back from it as JSON. The aliases below name these types, and
no other module spells ``Any``.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, Callable, Protocol, overload

# A message as the parser of Klipper passes it to a response handler. It maps a field name to a
# value, which is a number, or bytes for a %*s field.
Params = dict[str, Any]
# A status dict as get_status passes it to Klipper. It holds the sensor, motion or tag fields of a
# lane, and each value is whatever the field holds.
LaneStatus = dict[str, Any]
# A feed event as the host keeps it and passes it on (the Klipper event "ace2k:feed_event").
# Its fields are kind, mode, motor_mm, filament_mm, seq and stop_by.
# A seq of 0 means the unit started the motion itself.
FeedEvent = dict[str, Any]
# The record of a tag, as dataclasses.asdict returns it and as the JSON of the tag cache stores it.
Record = dict[str, Any]
# Answers of Moonraker as JSON, such as the status of Klipper or an entry of its G-code store.
Json = dict[str, Any]
# An attribute of the GCodeCommand of Klipper, passed on as it is.
GCodeAttr = Any


class Completion(Protocol):
    """Klipper's ``reactor.ReactorCompletion``, limited to the methods these modules use."""

    def test(self) -> bool: ...
    def complete(self, result: Any) -> None: ...
    def wait(self, waketime: float = ..., waketime_result: Any = ...) -> Any: ...


class Mutex(Protocol):
    """Klipper's ``reactor.ReactorMutex``, limited to the methods these modules use."""

    def test(self) -> bool: ...
    def __enter__(self) -> Any: ...
    def __exit__(self, type: Any = ..., value: Any = ..., tb: Any = ...) -> None: ...


class Reactor(Protocol):
    """Klipper's ``reactor.SelectReactor``, limited to the methods these modules use."""

    NOW: float
    NEVER: float

    def monotonic(self) -> float: ...
    def pause(self, waketime: float) -> float: ...
    def register_timer(self, callback: Callable[[float], float], waketime: float = ...) -> Any: ...
    def update_timer(self, timer_handler: Any, waketime: float) -> None: ...
    def register_async_callback(
        self, callback: Callable[[float], Any], waketime: float = ...
    ) -> None: ...
    def completion(self) -> Completion: ...
    def mutex(self, is_locked: bool = ...) -> Mutex: ...


class Command(Protocol):
    """Klipper's ``mcu.CommandWrapper``, limited to the methods these modules use.

    It is a command built from its format and sent to the unit.
    """

    def send(self, data: Sequence[Any] = ..., minclock: int = ..., reqclock: int = ...) -> None: ...


class QueryCommand(Protocol):
    """Klipper's ``mcu.CommandQueryWrapper``, limited to the methods these modules use.

    It is a command together with the response that answers it.
    """

    def send(
        self,
        data: Sequence[Any] = ...,
        minclock: int = ...,
        reqclock: int = ...,
        retry: bool = ...,
    ) -> Any: ...


class MCU(Protocol):
    """Klipper's ``mcu.MCU``, limited to the methods these modules use."""

    def register_config_callback(self, cb: Callable[[], Any]) -> None: ...
    def add_config_cmd(self, cmd: str, is_init: bool = ..., on_restart: bool = ...) -> None: ...
    def seconds_to_clock(self, time: float) -> int: ...
    def lookup_command(self, msgformat: str, cq: Any = ...) -> Command: ...
    def lookup_query_command(
        self,
        msgformat: str,
        respformat: str,
        oid: int | None = ...,
        cq: Any = ...,
        is_async: bool = ...,
    ) -> QueryCommand: ...
    def try_lookup_command(self, msgformat: str) -> Command | None: ...
    def register_serial_response(
        self, cb: Callable[[Any], Any], msg: str, oid: Any = ...
    ) -> Any: ...
    def register_response(self, cb: Callable[[Any], Any], name: str, oid: Any = ...) -> Any: ...
    def check_valid_response(self, msgformat: str) -> bool: ...
    def get_constant_float(self, name: str) -> float: ...
    def estimated_print_time(self, eventtime: float) -> float: ...


class GCodeCommand(Protocol):
    """Klipper's ``gcode.GCodeCommand``, limited to the methods these modules use."""

    error: type[Exception]

    def get_command(self) -> str: ...
    def get_command_parameters(self) -> dict[str, str]: ...
    def get(
        self,
        name: str,
        default: Any = ...,
        parser: Callable[[str], Any] = ...,
        minval: Any = ...,
        maxval: Any = ...,
        above: Any = ...,
        below: Any = ...,
    ) -> Any: ...
    # A default of None is what an absent parameter returns.
    @overload
    def get_int(
        self,
        name: str,
        default: None,
        minval: int | None = ...,
        maxval: int | None = ...,
    ) -> int | None: ...
    @overload
    def get_int(
        self,
        name: str,
        default: int = ...,
        minval: int | None = ...,
        maxval: int | None = ...,
    ) -> int: ...
    @overload
    def get_float(
        self,
        name: str,
        default: None,
        minval: float | None = ...,
        maxval: float | None = ...,
        above: float | None = ...,
        below: float | None = ...,
    ) -> float | None: ...
    @overload
    def get_float(
        self,
        name: str,
        default: float = ...,
        minval: float | None = ...,
        maxval: float | None = ...,
        above: float | None = ...,
        below: float | None = ...,
    ) -> float: ...
    def respond_info(self, msg: str, log: bool = ...) -> None: ...
    def respond_raw(self, msg: str) -> None: ...


class GCode(Protocol):
    """Klipper's ``gcode.GCodeDispatch``, limited to the methods these modules use."""

    def register_command(
        self,
        cmd: str,
        func: Callable[[GCodeCommand], Any] | None,
        when_not_ready: bool = ...,
        desc: str | None = ...,
    ) -> Any: ...
    def run_script_from_command(self, script: str) -> None: ...
    def run_script(self, script: str) -> None: ...
    def respond_raw(self, msg: str) -> None: ...
    def respond_info(self, msg: str, log: bool = ...) -> None: ...


class Printer(Protocol):
    """Klipper's ``klippy.Printer``, limited to the methods these modules use."""

    config_error: type[Exception]
    command_error: type[Exception]

    def get_start_args(self) -> dict[str, Any]: ...
    def get_reactor(self) -> Reactor: ...
    def is_shutdown(self) -> bool: ...
    def add_object(self, name: str, obj: Any) -> None: ...
    def lookup_object(self, name: str, default: Any = ...) -> Any: ...
    def lookup_objects(self, module: str | None = ...) -> list[tuple[str, Any]]: ...
    def load_object(self, config: Config, section: str, default: Any = ...) -> Any: ...
    def invoke_shutdown(self, msg: str, details: Any = ...) -> None: ...
    def register_event_handler(self, event: str, callback: Callable[..., Any]) -> None: ...
    def send_event(self, event: str, *params: Any) -> list[Any]: ...
    def request_exit(self, result: str) -> None: ...


class Config(Protocol):
    """Klipper's ``configfile.ConfigWrapper``, limited to the methods these modules use."""

    error: type[Exception]

    def get_printer(self) -> Printer: ...
    def get_name(self) -> str: ...
    def get(self, option: str, default: Any = ..., note_valid: bool = ...) -> Any: ...
    def deprecate(self, option: str, value: str | None = ...) -> None: ...
    # A default of None is what an absent option returns.
    @overload
    def getint(
        self,
        option: str,
        default: None,
        minval: int | None = ...,
        maxval: int | None = ...,
        note_valid: bool = ...,
    ) -> int | None: ...
    @overload
    def getint(
        self,
        option: str,
        default: int = ...,
        minval: int | None = ...,
        maxval: int | None = ...,
        note_valid: bool = ...,
    ) -> int: ...
    @overload
    def getfloat(
        self,
        option: str,
        default: None,
        minval: float | None = ...,
        maxval: float | None = ...,
        above: float | None = ...,
        below: float | None = ...,
        note_valid: bool = ...,
    ) -> float | None: ...
    @overload
    def getfloat(
        self,
        option: str,
        default: float = ...,
        minval: float | None = ...,
        maxval: float | None = ...,
        above: float | None = ...,
        below: float | None = ...,
        note_valid: bool = ...,
    ) -> float: ...
    def getboolean(self, option: str, default: Any = ..., note_valid: bool = ...) -> bool: ...
    def getchoice(
        self,
        option: str,
        choices: dict[Any, Any] | Sequence[Any],
        default: Any = ...,
        note_valid: bool = ...,
    ) -> Any: ...
    def getintlist(
        self,
        option: str,
        default: Any = ...,
        sep: str = ...,
        count: int | None = ...,
        note_valid: bool = ...,
    ) -> tuple[int, ...]: ...
    def getsection(self, section: str) -> Config: ...
    def has_section(self, section: str) -> bool: ...
    def get_prefix_sections(self, prefix: str) -> list[Config]: ...
    def get_prefix_options(self, prefix: str) -> list[str]: ...


class Heater(Protocol):
    """Klipper's ``heaters.Heater``, limited to the methods these modules use."""

    can_extrude: bool


class Extruder(Protocol):
    """Klipper's ``kinematics/extruder.PrinterExtruder``, limited to what these modules use."""

    def get_name(self) -> str: ...
    def get_heater(self) -> Heater: ...
    def find_past_position(self, print_time: float) -> float: ...


class ToolHead(Protocol):
    """Klipper's ``toolhead.ToolHead``, limited to the methods these modules use."""

    def get_extruder(self) -> Extruder: ...
    def get_last_move_time(self) -> float: ...
    def wait_moves(self) -> None: ...


class PrintStats(Protocol):
    """Klipper's ``print_stats.PrintStats``, limited to the methods these modules use."""

    def get_status(self, eventtime: float) -> dict[str, Any]: ...


class ToolheadMove(Protocol):
    """Klipper's ``toolhead.Move``, as the planned-move hook of an extruder receives it."""

    axes_d: list[float]
    accel_t: float
    cruise_t: float
    decel_t: float
