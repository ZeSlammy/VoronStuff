# Support for SGP40 sensors and display on the [nevermore] component in MainSail
#
# Heavily based on work done by Adrien Le Masle and Sanaa Hamel
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math
from . import bus
from struct import unpack_from
from .voc_algorithm import VOCAlgorithm

SGP40_REPORT_TIME = 1
SGP40_CHIP_ADDR = 0x59
SGP40_WORD_LEN = 2

SGP40_CMD = {
    "GET_SERIAL" : [0x36, 0x82],
    "SOFT_RESET" : [0x00, 0x06],
    "SELF_TEST" : [0x28, 0x0E],
    "MEASURE_RAW_NO_COMP" : [0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93]
}

class SGP40Reader:
    def __init__(self, config):
        def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=SGP40_CHIP_ADDR, default_speed=100000)
        self.temp_sensor = config.get('ref_temp_sensor', None)
        self.humidity_sensor = config.get('ref_humidity_sensor', None)
        self.voc_scale = config.getfloat('voc_scale', 1. )
        self.mcu = self.i2c.get_mcu()
        self.raw = 0
        self.voc = 0
        self.temp = 0
        self.humidity = 0
        self.min_temp = self.max_temp = 0
        self.plot_voc = config.getboolean('plot_voc', False)
        self.max_sample_time = 1
        self.sample_timer = None
        self.printer.add_object("sgp40 " + self.name, self)
        self._voc_algorithm = VOCAlgorithm()
        self._voc_algorithm.vocalgorithm_init()
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def read_sensor_data(self, temperature, humidity):
        """Reads and calculates VOC data from SGP40."""
        cmd = [0x26, 0x0F] + self._humidity_to_ticks(humidity) + self._temperature_to_ticks(temperature)
        raw_value = self._read_and_check(cmd)
        voc_index = self.voc_algorithm.vocalgorithm_process(raw_value[0]) if raw_value else 0
        return voc_index, raw_value[0]  # VOC index and raw gas reading

    # Include _read_and_check, _temperature_to_ticks, _humidity_to_ticks methods as in SGP40 class.

class NevermoreSensor:
    DISPLAY_FIELDS = {"temperature", "humidity", "pressure", "voc"}

    def __init__(self, config: ConfigWrapper) -> None:
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.sgp40_reader = SGP40Reader(config)
        self.intake_state = SensorState()
        self.exhaust_state = SensorState()
        self.plot_voc = config.getboolean("plot_voc", False)
        # Other initialization as per original code

    @property
    def state(self) -> SensorState:
        return self.intake_state if self.sensor_kind == SensorKind.INTAKE else self.exhaust_state

    def get_status(self, eventtime: float) -> dict:
        """Retrieve data for rendering in MainSail."""
        # Example data - replace with actual sensor readings
        intake_data = {
            "temperature": 25,
            "humidity": 50,
            "pressure": 1013,
            "voc": 10,
        }
        exhaust_data = {
            "temperature": 24,
            "humidity": 48,
            "pressure": 1010,
            "voc": 12,
        }

        # Track min/max values
        self._update_min_max("intake", intake_data)
        self._update_min_max("exhaust", exhaust_data)

        # Build status with current, min, max values
        status = {
            "intake_temperature": intake_data["temperature"],
            "intake_humidity": intake_data["humidity"],
            "intake_pressure": intake_data["pressure"],
            "intake_voc": intake_data["voc"],
            "exhaust_temperature": exhaust_data["temperature"],
            "exhaust_humidity": exhaust_data["humidity"],
            "exhaust_pressure": exhaust_data["pressure"],
            "exhaust_voc": exhaust_data["voc"],
        }

        for key, value in status.items():
            min_key = f"{key}_min"
            max_key = f"{key}_max"
            status[min_key] = self.min_values.get(key, value)
            status[max_key] = self.max_values.get(key, value)

        return status

    def _update_min_max(self, state: str, data: dict):
        """Helper to track min/max values for intake and exhaust."""
        for key, value in data.items():
            full_key = f"{state}_{key}"
            if full_key not in self.min_values:
                self.min_values[full_key] = value
            if full_key not in self.max_values:
                self.max_values[full_key] = value
            self.min_values[full_key] = min(self.min_values[full_key], value)
            self.max_values[full_key] = max(self.max_values[full_key], value)

 
def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("NevermorePCBSensor", NevermoreSensor)