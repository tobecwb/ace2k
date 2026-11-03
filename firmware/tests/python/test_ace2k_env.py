"""ace2k_env.py: the sensor type, its callback, min/max, humidity, invalid readings."""

from __future__ import annotations

from typing import TYPE_CHECKING

import ace2k_env

if TYPE_CHECKING:
    from collections.abc import Callable

    from ace2k_env import Ace2kTemperature
    from ace2k_link import EnvState

    # A listener of the env reports: (print time, report)
    EnvListener = Callable[[float, EnvState], None]


class FakeAce2k:
    def __init__(self) -> None:
        self.listeners = []

    def register_env_listener(self, cb: EnvListener) -> None:
        self.listeners.append(cb)


class FakePrinter:
    def __init__(self) -> None:
        self.ace2k = FakeAce2k()
        self.shutdowns = []
        self.factories = {}
        self.objects = {}

    def add_object(self, name: str, obj: object) -> None:
        self.objects[name] = obj

    def lookup_object(self, name: str, default: object = None) -> FakePrinter:
        # the sensor finds its unit through the coordinator: with one unit, unit: may be omitted
        assert name == "ace2k_units"
        return self

    def resolve(self, name: object, key: str = "UNIT=") -> FakeAce2k:
        # the coordinator's resolution with one [ace2k] and no unit: given
        assert name is None and key == "unit:"
        return self.ace2k

    def load_object(self, config: FakeConfig, name: str) -> FakePrinter:
        assert name == "heaters"
        return self

    def add_sensor_factory(
        self, name: str, factory: Callable[[FakeConfig], Ace2kTemperature]
    ) -> None:
        self.factories[name] = factory

    def invoke_shutdown(self, msg: str) -> None:
        self.shutdowns.append(msg)


class FakeConfig:
    def __init__(self, printer: FakePrinter, sensor: str) -> None:
        self.printer, self.sensor = printer, sensor

    def get_printer(self) -> FakePrinter:
        return self.printer

    def get_name(self) -> str:
        return "temperature_sensor ace_a_" + self.sensor

    def get(self, key: str, default: str | None = None) -> str | None:
        assert key == "unit"
        return default

    def getchoice(self, key: str, choices: dict[str, str]) -> str:
        assert key == "sensor" and self.sensor in choices
        return choices[self.sensor]


ENV = {
    "ptc_left": 25.0,
    "ptc_right": 24.5,
    "chamber": 23.48,
    "humidity": 67.0,
    "vdda": 3.3,
    "valid_mask": 0xF,
    "valid": ["ntc_left", "ntc_right", "chamber", "vdda"],
}


def make(sensor: str) -> tuple[Ace2kTemperature, FakePrinter, list[tuple[float, float]]]:
    printer = FakePrinter()
    ace2k_env.load_config(FakeConfig(printer, sensor))
    obj = printer.factories["ace2k"](FakeConfig(printer, sensor))
    obj.setup_minmax(0.0, 100.0)
    seen = []
    obj.setup_callback(lambda pt, t: seen.append((pt, t)))
    return obj, printer, seen


def test_chamber_delivers_temperature_and_humidity() -> None:
    obj, printer, seen = make("chamber")
    printer.ace2k.listeners[0](12.5, ENV)
    assert seen == [(12.5, 23.48)]
    assert obj.get_status(0.0) == {"temperature": 23.48, "invalid": False, "humidity": 67.0}


def test_the_sensor_object_is_published_under_its_full_section_name() -> None:
    # a [temperature_sensor] reports only the temperature; humidity and invalid are read here
    obj, printer, seen = make("chamber")
    assert printer.objects == {"ace2k_env temperature_sensor ace_a_chamber": obj}


def test_ntc_side_selects_its_field_and_has_no_humidity() -> None:
    obj, printer, seen = make("ptc_right")
    printer.ace2k.listeners[0](1.0, ENV)
    assert seen == [(1.0, 24.5)]
    assert "humidity" not in obj.get_status(0.0)


def test_invalid_reading_is_delivered_as_zero_and_flagged() -> None:
    # never the last good value: a graph frozen on 23.48 would claim a reading nobody has
    obj, printer, seen = make("chamber")
    printer.ace2k.listeners[0](1.0, ENV)
    printer.ace2k.listeners[0](2.0, dict(ENV, valid_mask=0xB))
    assert seen == [(1.0, 23.48), (2.0, 0.0)]
    assert obj.get_status(0.0) == {"temperature": 0.0, "invalid": True, "humidity": None}
    assert printer.shutdowns == []  # min_temp 0.0: zero is inside the range


def test_invalid_reading_below_min_temp_invokes_shutdown() -> None:
    obj, printer, seen = make("ptc_left")
    obj.setup_minmax(5.0, 100.0)
    printer.ace2k.listeners[0](1.0, dict(ENV, valid_mask=0xE))
    assert seen == [(1.0, 0.0)]
    assert printer.shutdowns and "ace_a_ptc_left" in printer.shutdowns[0]
    assert "0.0" in printer.shutdowns[0]


def test_out_of_range_invokes_shutdown() -> None:
    obj, printer, seen = make("chamber")
    obj.setup_minmax(0.0, 20.0)
    printer.ace2k.listeners[0](1.0, ENV)
    assert printer.shutdowns and "ace_a_chamber" in printer.shutdowns[0]


def test_the_report_period_is_one_second() -> None:
    obj, printer, seen = make("ptc_left")
    assert obj.get_report_time_delta() == 1.0
