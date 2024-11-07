class NevermoreSensor:
    DISPLAY_FIELDS = {"temperature", "humidity", "pressure", "voc"}

    def __init__(self, config: ConfigWrapper, is_exhaust=False) -> None:
        self.name = config.get_name().split()[-1]
        self.is_exhaust = is_exhaust
        self.sensor_kind = "exhaust" if is_exhaust else "intake"
        self.sgp40 = SGP40(config)
        self.min_temp = self.max_temp = 0.0
        self.min_humidity = self.max_humidity = 0.0
        self.min_pressure = self.max_pressure = 0.0
        self.min_voc = self.max_voc = 0.0

    def get_status(self, eventtime: float) -> Dict[str, float]:
        status = {
            f"{self.sensor_kind}_temperature": self.sgp40.get_temperature(),
            f"{self.sensor_kind}_humidity": self.sgp40.get_humidity(),
            f"{self.sensor_kind}_pressure": self.sgp40.get_pressure(),
            f"{self.sensor_kind}_voc": self.sgp40.get_voc_index()
        }
        return status

    def setup_minmax(self, min_temp, max_temp, min_humidity, max_humidity, min_pressure, max_pressure, min_voc, max_voc):
        self.min_temp, self.max_temp = min_temp, max_temp
        self.min_humidity, self.max_humidity = min_humidity, max_humidity
        self.min_pressure, self.max_pressure = min_pressure, max_pressure
        self.min_voc, self.max_voc = min_voc, max_voc
    
    def read_bme280_data(self):
        # Check if BME280 is initialized
        if not self.bme280:
            logging.error("BME280 sensor instance not found.")
            return None

        # Access BME280 data
        temp = self.bme280.temp
        pressure = self.bme280.pressure
        humidity = self.bme280.humidity
        gas = getattr(self.bme280, "gas", None)  # Only for BME680

        # You can now use this data in your NevermoreSensor class
        logging.info(f"BME280 Data - Temperature: {temp}, Pressure: {pressure}, Humidity: {humidity}")

        # Return data as a dictionary or process further as needed
        return {
            "temperature": temp,
            "pressure": pressure,
            "humidity": humidity,
            "gas": gas  # Optional if BME680
        }

class SGP40:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=SGP40_CHIP_ADDR, default_speed=100000)
        self.temp_sensor = config.get('ref_temp_sensor', None)
        self.humidity_sensor = config.get('ref_humidity_sensor', None)
        self.voc_scale = config.getfloat('voc_scale', 1.0)
        self.plot_voc = config.getboolean('plot_voc', False)
        
        # Initialize sensor values
        self.raw = 0
        self.voc = 0
        self.temp = 0
        self.humidity = 0
        self.pressure = 1013  # Assuming a default pressure, or can integrate another sensor for this
        
        # Initialize VOC algorithm
        self._voc_algorithm = VOCAlgorithm()
        self._voc_algorithm.vocalgorithm_init()
        
        # Register this sensor with the printer
        self.printer.add_object("sgp40 " + self.name, self)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self):
        self._init_sgp40()
        self.sample_timer = self.reactor.register_timer(self._sample_sgp40)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _init_sgp40(self):
        # Perform self-test and initialize
        self_test = self._read_and_check(SGP40_CMD["SELF_TEST"], wait_time_s=0.5)
        if self_test[0] != 0xD400:
            logging.error("sgp40: Self test error")

    def _sample_sgp40(self, eventtime):
        # Update temperature and humidity
        self.temp = self._read_temperature(eventtime)
        self.humidity = self._read_humidity(eventtime)

        # Get raw VOC reading and process it
        cmd = [0x26, 0x0F] + self._humidity_to_ticks(self.humidity) + self._temperature_to_ticks(self.temp)
        value = self._read_and_check(cmd)
        self.raw = value[0]
        self.voc = self._voc_algorithm.vocalgorithm_process(self.raw)

        # Use a callback if needed for asynchronous handling
        measured_time = self.reactor.monotonic()
        if hasattr(self, '_callback'):
            self._callback(self.mcu.estimated_print_time(measured_time), self.voc * self.voc_scale)

        return measured_time + SGP40_REPORT_TIME

    def _read_temperature(self, eventtime):
        if self.temp_sensor:
            return self.printer.lookup_object(self.temp_sensor).get_status(eventtime)["temperature"]
        return 25  # Default if no external sensor is available

    def _read_humidity(self, eventtime):
        if self.humidity_sensor:
            try:
                return self.printer.lookup_object(self.humidity_sensor).get_status(eventtime)["humidity"]
            except KeyError:
                return 50  # Fallback value if sensor is unavailable
        # Estimate humidity if external sensor is unavailable
        return 50.0 / math.exp(0.0499860 * self.temp - 1.1674630)

    def _read_and_check(self, cmd, read_len=1, wait_time_s=0.05):
        reply_len = read_len * (SGP40_WORD_LEN + 1)  # CRC every word
        self.i2c.i2c_write(cmd)
        self.reactor.pause(self.reactor.monotonic() + wait_time_s)
        response = bytearray(self.i2c.i2c_read([], reply_len)["response"])

        data = []
        for i in range(0, reply_len, 3):
            if not self._check_crc8(response[i:i + 2], response[i + 2]):
                logging.warn("sgp40: Checksum error on read!")
            data.append(unpack_from(">H", response[i:i + 2])[0])

        return data

    def get_temperature(self):
        """Returns the current temperature reading in Celsius."""
        return self.temp

    def get_humidity(self):
        """Returns the current humidity reading as a percentage."""
        return self.humidity

    def get_pressure(self):
        """Returns the current pressure reading in hPa."""
        return self.pressure

    def get_voc_index(self):
        """Returns the VOC index, scaled as per configuration."""
        return self.voc * self.voc_scale

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        """Allows the Nevermore sensor to set a callback for VOC updates."""
        self._callback = cb

    def get_status(self, eventtime):
        if self.plot_voc:
            return {"temperature": self.voc * self.voc_scale}
        else:
            return {
                'temperature': self.temp,
                'humidity': self.humidity,
                'pressure': self.pressure,
                'voc': self.voc * self.voc_scale
            }
