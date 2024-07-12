import serial
import serial.tools.list_ports
import time
import logging

# Configure logging
logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
)


class TPMS:
    def __init__(
        self,
        port=None,
        baudrate=19200,
        temp_unit="Celsius",
        pressure_unit="kPa",
        callback=None,
        debug=False,
        update_interval=0.5,
    ):
        self.baudrate = baudrate
        self.temp_unit = temp_unit
        self.pressure_unit = pressure_unit
        self.debug = debug
        self.callback = callback
        self.update_interval = update_interval  # Interval between reads in seconds
        self.ser = None
        self.incoming_buffer = bytearray()

        self.setup_command_frames()

        self.port = port or self.auto_detect_port()
        self.connect()

    def setup_command_frames(self):
        self.querySensorID = bytearray([0x55, 0xAA, 0x06, 0x07, 0x00, 0x00, 0x0C])
        self.eventACK = bytearray([0x55, 0xAA, 0x06, 0x00, 0x00, 0x00])
        self.heartbeat_frame = bytearray([0x55, 0xAA, 0x06, 0x19, 0x00, 0xE0])

    def auto_detect_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            try:
                ser = serial.Serial(port.device, self.baudrate, timeout=2)
                ser.write(self.querySensorID)
                time.sleep(1)
                response = ser.read(10)
                ser.close()
                if response:
                    return port.device
            except serial.SerialException:
                continue
        raise EnvironmentError("No valid TPMS device found on any port")

    def connect(self):
        if not self.port:
            raise RuntimeError("Serial port not configured.")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=2)
            logger.info(f"Connected to TPMS device on port {self.port}")
        except serial.SerialException as e:
            logger.error(f"Connection failed: {e}")
            raise

    def send_command(self, frame, description):
        try:
            self.ser.write(frame)
            logger.info(f"{description} sent: {frame.hex()}")
            if self.debug:
                response = self.ser.read(100)  # Read potential responses
                logger.debug(f"Response after {description}: {response.hex()}")
        except serial.SerialException as e:
            logger.error(f"Failed to send {description}: {e}")

    def oneshot(self):
        """
        Perform a single data retrieval and return the processed data.
        This method ensures each tyre is reported once with the most recent data.
        """
        try:
            self.send_command(self.querySensorID, "Query Sensor ID")
            time.sleep(1)  # Wait for the device to respond
            data = self.ser.read(
                1024
            )  # Adjust buffer size based on expected data volume
            if data:
                self.incoming_buffer.extend(data)
                processed_data = self.process_frames()
                tyre_data = {}
                for data in processed_data:
                    if data:
                        tyre_data[data["Position"]] = (
                            data  # Update with the most recent data for each tyre
                        )
                return tyre_data
        except serial.SerialException as e:
            logger.error(f"Serial error during oneshot: {e}")
            self.connect()

    def stream_updates(self):
        """
        Continuously read data and process it, sending updates in real-time.
        """
        try:
            while True:
                data = self.ser.read(256)
                if data:
                    logger.debug(f"Received data: {data.hex()}")
                    self.incoming_buffer.extend(data)
                    processed_data = self.process_frames()
                    if self.callback:
                        self.callback(processed_data)
                time.sleep(self.update_interval)  # Use the configurable interval
        except serial.SerialException as e:
            logger.error(f"Serial error while streaming updates: {e}")
            self.connect()
        except KeyboardInterrupt:
            logger.info("Streaming stopped by user")

    def process_frames(self):
        frames = self.extract_frames()
        processed_data = []
        for frame in frames:
            tyre_data = self.handle_frame(frame)
            if tyre_data:
                processed_data.append(tyre_data)
        return processed_data

    def extract_frames(self):
        frames = []
        start = 0
        while start + 2 < len(self.incoming_buffer):
            if (
                self.incoming_buffer[start] == 0x55
                and self.incoming_buffer[start + 1] == 0xAA
            ):
                length = self.incoming_buffer[start + 2]
                if start + length <= len(self.incoming_buffer):
                    frames.append(self.incoming_buffer[start : start + length])
                    start += length
                else:
                    break
            else:
                start += 1
        self.incoming_buffer = self.incoming_buffer[start:]  # Remove processed data
        return frames

    def handle_frame(self, frame):
        if len(frame) < 3:
            logger.error("Received incomplete frame.")
            return

        cmd_type = frame[2]
        tyre_position_codes = {
            0: "Left Front",
            1: "Right Front",
            16: "Left Rear",
            17: "Right Rear",
            5: "Spare",
        }

        if cmd_type == 0x08:  # Tyre status report
            if len(frame) >= 8:
                tyre_position = tyre_position_codes.get(frame[3], "Unknown")
                pressure_raw = (
                    frame[4] * 3.44
                )  # Example conversion factor from device spec
                temperature_raw = frame[5] - 50  # Example offset from device spec
                pressure = self.convert_pressure(pressure_raw)
                temperature = self.convert_temperature(temperature_raw)
                status_byte = frame[6]
                status = {
                    "low_power": bool(status_byte & 0x01),
                    "leakage": bool(status_byte & 0x02),
                    "no_signal": bool(status_byte & 0x04),
                }
                tyre_data = {
                    "Position": tyre_position,
                    "Pressure": pressure,
                    "Temperature": temperature,
                    "Status": status,
                }
                return tyre_data

    def convert_temperature(self, temp):
        if self.temp_unit == "Fahrenheit":
            return (temp * 9 / 5) + 32
        return temp

    def convert_pressure(self, pressure):
        if self.pressure_unit == "psi":
            return pressure * 0.145038
        elif self.pressure_unit == "bar":
            return pressure / 100
        return pressure


if __name__ == "__main__":
    tpms = TPMS(
        debug=True, temp_unit="Fahrenheit", pressure_unit="psi", update_interval=1.0
    )
    tpms.stream_updates()  # Start continuous monitoring with a 1-second update interval
