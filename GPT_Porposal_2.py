class NevermoreSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        
        # Retrieve BME280 sensors
        self.bme_out = self.printer.lookup_object("bme280 BME_OUT")
        self.bme_in = self.printer.lookup_object("bme280 BME_IN")
        
        # Retrieve SGP40 sensors
        self.sgp_out = self.printer.lookup_object("sgp40 SGP_OUT")
        self.sgp_in = self.printer.lookup_object("sgp40 SGP_IN")
        
        # Ensure all sensors are found
        if not all([self.bme_out, self.bme_in, self.sgp_out, self.sgp_in]):
            logging.error("One or more sensors not found.")
            return

    def read_bme280_data(self, sensor_type="OUT"):
        # Select the right BME280 sensor based on the input type (OUT/IN)
        if sensor_type == "OUT":
            bme_sensor = self.bme_out
        elif sensor_type == "IN":
            bme_sensor = self.bme_in
        else:
            logging.error("Invalid BME280 sensor type.")
            return None

        # Retrieve sensor data
        temp = bme_sensor.temp
        pressure = bme_sensor.pressure
        humidity = bme_sensor.humidity
        gas = getattr(bme_sensor, "gas", None)  # Optional, in case it's a BME680

        # Log or return the data
        logging.info(f"BME280 {sensor_type} Data - Temperature: {temp}, Pressure: {pressure}, Humidity: {humidity}")
        return {
            "temperature": temp,
            "pressure": pressure,
            "humidity": humidity,
            "gas": gas  # Optional if BME680
        }

    def read_sgp40_data(self, sensor_type="OUT"):
        # Select the right SGP40 sensor based on the input type (OUT/IN)
        if sensor_type == "OUT":
            sgp_sensor = self.sgp_out
        elif sensor_type == "IN":
            sgp_sensor = self.sgp_in
        else:
            logging.error("Invalid SGP40 sensor type.")
            return None

        # Retrieve temperature and humidity from BME sensor for the SGP40
        ref_temp = sgp_sensor.ref_temp_sensor.temp
        ref_humidity = sgp_sensor.ref_humidity_sensor.humidity

        # Retrieve SGP40 data
        voc_index = sgp_sensor.voc_index  # VOC index for air quality measurement
        co2_equivalent = sgp_sensor.co2_equivalent  # CO2 equivalent measurement

        # Log or return the data
        logging.info(f"SGP40 {sensor_type} Data - VOC Index: {voc_index}, CO2 Equivalent: {co2_equivalent}, Temperature: {ref_temp}, Humidity: {ref_humidity}")
        return {
            "voc_index": voc_index,
            "co2_equivalent": co2_equivalent,
            "temperature": ref_temp,
            "humidity": ref_humidity
        }

    def read_sensor_data(self):
        # Example of reading data from all sensors
        bme_out_data = self.read_bme280_data("OUT")
        bme_in_data = self.read_bme280_data("IN")
        
        sgp_out_data = self.read_sgp40_data("OUT")
        sgp_in_data = self.read_sgp40_data("IN")

        return {
            "bme_out_data": bme_out_data,
            "bme_in_data": bme_in_data,
            "sgp_out_data": sgp_out_data,
            "sgp_in_data": sgp_in_data,
        }

