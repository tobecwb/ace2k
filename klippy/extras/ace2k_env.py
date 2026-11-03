"""Exposes the temperatures of the unit as Klipper temperature sensors.

    [temperature_sensor ace_a_chamber]
    sensor_type: ace2k
    sensor: chamber          # ptc_left | ptc_right | chamber
    #unit: a                # with several [ace2k <name>] units: which one

The readings come from the periodic ``ace2k_env_state`` report of the [ace2k] module (one per
second). This file only adapts them to the sensor interface of Klipper. So they show in graphs
in Mainsail, and ``min_temp`` and ``max_temp`` apply to them. The ``chamber`` sensor also
publishes ``humidity``.

When the unit marks a reading as invalid, the sensor reports 0.0 °C and ``invalid: True``.
An invalid reading is an NTC outside its window or a chamber sensor that stopped answering.
Klipper uses 0.0 °C for a failed reading, and the sensor never repeats the last good value.
If ``min_temp`` is above zero, an invalid reading shuts the printer down, as configured.

A ``[temperature_sensor]`` reports only the temperature. So each sensor object is also
published as ``ace2k_env <section name>`` with the temperature, the humidity and ``invalid``.
The humidity is in the status of ``[ace2k]`` as well.

Load order: ``[ace2k]`` registers this sensor type. Every ``[ace2k]`` or ``[ace2k <name>]``
section must come before the ``[temperature_sensor]`` sections that use it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Literal, Protocol

try:
    from .ace2k_units import UnitError
except ImportError:  # the host tests put klippy/extras on the path and import by bare name
    from ace2k_units import UnitError  # type: ignore[import-not-found, no-redef]

if TYPE_CHECKING:
    from .ace2k_klipper_types import Config
    from .ace2k_link import EnvState

    Sensor = Literal["ptc_left", "ptc_right", "chamber"]
    EnvListener = Callable[[float, EnvState], None]  # (print time, report)

    class UnitProto(Protocol):
        """The unit (``ace2k.Ace2k``), as a sensor sees it when it subscribes to the reports."""

        def register_env_listener(self, cb: EnvListener) -> None: ...

    class UnitsProto(Protocol):
        """The registry of units (``ace2k_units.Units``), as a sensor looks up its unit."""

        def resolve(self, name: object, key: str = "UNIT=") -> UnitProto | None: ...


REPORT_TIME = 1.0
SENSORS = {"ptc_left": 0x1, "ptc_right": 0x2, "chamber": 0x4}


class Ace2kTemperature:
    def __init__(self, config: Config) -> None:
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.sensor: Sensor = config.getchoice("sensor", {k: k for k in SENSORS})
        self.valid_bit = SENSORS[self.sensor]
        self.min_temp = self.max_temp = 0.0
        self.temp = 0.0
        self.humidity: float | None = None
        self.invalid = True
        self._callback: Callable[[float, float], None] | None = None
        ace2k = self._unit(config)
        ace2k.register_env_listener(self._on_env)
        # The key is the full section name. Two sections of different types can share a name,
        # such as [temperature_sensor chamber] and [temperature_fan chamber], and must not collide.
        self.printer.add_object("ace2k_env " + self.name, self)

    def _unit(self, config: Config) -> UnitProto:
        """Return the unit named by ``unit:``.

        The name is looked up among all units the config declares, not only those loaded so far.
        Otherwise a sensor placed between two unit sections would bind to the first one.
        """
        after = f"ace2k: sensor {self.name} must come after every [ace2k] section"
        units: UnitsProto | None = self.printer.lookup_object("ace2k_units", None)
        if units is None:
            raise config.error(after)
        try:
            unit = units.resolve(config.get("unit", None), key="unit:")
        except UnitError as e:
            raise config.error(f"ace2k: sensor {self.name}: {e}") from None
        if unit is None:
            raise config.error(after)
        return unit

    def setup_minmax(self, min_temp: float, max_temp: float) -> None:
        self.min_temp, self.max_temp = min_temp, max_temp

    def setup_callback(self, cb: Callable[[float, float], None]) -> None:
        self._callback = cb

    def get_report_time_delta(self) -> float:
        return REPORT_TIME

    def _on_env(self, print_time: float, env: EnvState) -> None:
        if env["valid_mask"] & self.valid_bit:
            temp = env[self.sensor]
            assert temp is not None  # in a report, every reading is a number
            self.invalid = False
            self.temp = temp
            if self.sensor == "chamber":
                self.humidity = env["humidity"]
        else:
            # Report 0.0, never the last good value. A graph frozen on the last value would show
            # a temperature that nobody can read. The aht10 sensor of Klipper reports zeros on a
            # failed read in the same way.
            # The min and max check still runs. A min_temp above zero is set to trip on an open
            # NTC. The defaults (-273.15 and 99999999) let it pass.
            self.invalid = True
            self.temp = 0.0
            if self.sensor == "chamber":
                self.humidity = None
        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                f"{self.name} temperature {self.temp:.1f} outside range of"
                f" {self.min_temp:.1f}:{self.max_temp:.1f}"
            )
        if self._callback is not None:
            self._callback(print_time, self.temp)

    def get_status(self, _eventtime: float) -> dict[str, object]:
        status: dict[str, object] = {"temperature": round(self.temp, 2), "invalid": self.invalid}
        if self.sensor == "chamber":
            status["humidity"] = self.humidity
        return status


def load_config(config: Config) -> None:
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("ace2k", Ace2kTemperature)
