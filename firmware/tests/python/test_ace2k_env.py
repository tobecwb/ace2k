"""ace2k_env.py: the sensor type, its callback, min/max, humidity, invalid readings."""

import ace2k_env


class FakeAce2k:
    def __init__(self):
        self.listeners = []

    def register_env_listener(self, cb):
        self.listeners.append(cb)


class FakePrinter:
    def __init__(self):
        self.ace2k = FakeAce2k()
        self.shutdowns = []
        self.factories = {}
        self.objects = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name):
        assert name == "ace2k"
        return self.ace2k

    def load_object(self, config, name):
        assert name == "heaters"
        return self

    def add_sensor_factory(self, name, factory):
        self.factories[name] = factory

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


class FakeConfig:
    def __init__(self, printer, sensor):
        self.printer, self.sensor = printer, sensor

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "temperature_sensor ace_" + self.sensor

    def getchoice(self, key, choices):
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


def make(sensor):
    printer = FakePrinter()
    ace2k_env.load_config(FakeConfig(printer, sensor))
    obj = printer.factories["ace2k"](FakeConfig(printer, sensor))
    obj.setup_minmax(0.0, 100.0)
    seen = []
    obj.setup_callback(lambda pt, t: seen.append((pt, t)))
    return obj, printer, seen


def test_chamber_delivers_temperature_and_humidity():
    obj, printer, seen = make("chamber")
    printer.ace2k.listeners[0](12.5, ENV)
    assert seen == [(12.5, 23.48)]
    assert obj.get_status(0.0) == {"temperature": 23.48, "invalid": False, "humidity": 67.0}


def test_the_sensor_object_is_published_under_its_full_section_name():
    # a [temperature_sensor] reports only the temperature; humidity and invalid are read here
    obj, printer, seen = make("chamber")
    assert printer.objects == {"ace2k_env temperature_sensor ace_chamber": obj}


def test_ntc_side_selects_its_field_and_has_no_humidity():
    obj, printer, seen = make("ptc_right")
    printer.ace2k.listeners[0](1.0, ENV)
    assert seen == [(1.0, 24.5)]
    assert "humidity" not in obj.get_status(0.0)


def test_invalid_reading_is_delivered_as_zero_and_flagged():
    # never the last good value: a graph frozen on 23.48 would claim a reading nobody has
    obj, printer, seen = make("chamber")
    printer.ace2k.listeners[0](1.0, ENV)
    printer.ace2k.listeners[0](2.0, dict(ENV, valid_mask=0xB))
    assert seen == [(1.0, 23.48), (2.0, 0.0)]
    assert obj.get_status(0.0) == {"temperature": 0.0, "invalid": True, "humidity": None}
    assert printer.shutdowns == []  # min_temp 0.0: zero is inside the range


def test_invalid_reading_below_min_temp_invokes_shutdown():
    obj, printer, seen = make("ptc_left")
    obj.setup_minmax(5.0, 100.0)
    printer.ace2k.listeners[0](1.0, dict(ENV, valid_mask=0xE))
    assert seen == [(1.0, 0.0)]
    assert printer.shutdowns and "ace_ptc_left" in printer.shutdowns[0]
    assert "0.0" in printer.shutdowns[0]


def test_out_of_range_invokes_shutdown():
    obj, printer, seen = make("chamber")
    obj.setup_minmax(0.0, 20.0)
    printer.ace2k.listeners[0](1.0, ENV)
    assert printer.shutdowns and "ace_chamber" in printer.shutdowns[0]


def test_the_report_period_is_one_second():
    obj, printer, seen = make("ptc_left")
    assert obj.get_report_time_delta() == 1.0
