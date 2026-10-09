"""ace2k_env — the unit's temperatures as Klipper temperature sensors.

    [temperature_sensor ace_chamber]
    sensor_type: ace2k
    sensor: chamber          # ptc_left | ptc_right | chamber

The readings come from the [ace2k] module's periodic ``ace2k_env_state`` report (one per
second); this file only adapts them to Klipper's sensor interface, so they graph in Mainsail
and get ``min_temp`` / ``max_temp``.  ``chamber`` also publishes ``humidity``.  A reading the
unit marks invalid (an NTC outside its window, a chamber sensor that stopped answering) is
delivered as 0.0 °C — Klipper's own convention for a failed reading, never the last good value
— and shows as ``invalid: True``; a ``min_temp`` above zero then shuts the printer down on it,
as it was configured to.  A ``[temperature_sensor]`` reports only the
temperature, so each sensor object is also published as ``ace2k_env <section name>`` (temperature,
humidity, invalid), and humidity is in ``[ace2k]``'s status as well.  Load order: ``[ace2k]``
registers this type, so it must come before the ``[temperature_sensor]`` sections that use
it.
"""

REPORT_TIME = 1.0
SENSORS = {"ptc_left": 0x1, "ptc_right": 0x2, "chamber": 0x4}


class Ace2kTemperature:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.sensor = config.getchoice("sensor", {k: k for k in SENSORS})
        self.valid_bit = SENSORS[self.sensor]
        self.min_temp = self.max_temp = 0.0
        self.temp = 0.0
        self.humidity = None
        self.invalid = True
        self._callback = None
        ace2k = self.printer.lookup_object("ace2k")
        ace2k.register_env_listener(self._on_env)
        # Keyed on the full section name: two sections of different types sharing a name
        # ([temperature_sensor chamber] and [temperature_fan chamber]) must not collide.
        self.printer.add_object("ace2k_env " + self.name, self)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp, self.max_temp = min_temp, max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _on_env(self, print_time, env):
        if env["valid_mask"] & self.valid_bit:
            self.invalid = False
            self.temp = env[self.sensor]
            if self.sensor == "chamber":
                self.humidity = env["humidity"]
        else:
            # 0.0, never the last good value: a graph frozen on it would claim a temperature
            # nobody can read (Klipper's aht10.py reports zeros on a failed read the same way).
            # The min/max check still runs: a min_temp above zero was configured to trip on an
            # open NTC; the defaults (−273.15, 99999999) let it pass.
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

    def get_status(self, eventtime):
        status = {"temperature": round(self.temp, 2), "invalid": self.invalid}
        if self.sensor == "chamber":
            status["humidity"] = self.humidity
        return status


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("ace2k", Ace2kTemperature)
