import serial
import time
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class TPMS:
    def __init__(
        self,
        port=None,
        baudrate=19200,
        temp_unit="Celsius",
        pressure_unit="kPa",
        hardware_version="TY06",
    ):
        self.port = port or self.auto_detect_port()
        self.baudrate = baudrate
        self.temp_unit = temp_unit
        self.pressure_unit = pressure_unit
        self.hardware_version = hardware_version
        self.ser = None
        self.incoming_buffer = bytearray()

        # TPMS command frames
        self.querySensorID = bytearray([0x55, 0xAA, 0x06, 0x07, 0x00, 0x00, 0x0C])
        self.eventACK = bytearray([0x55, 0xAA, 0x06, 0x00, 0x00, 0x00])
        self.heartbeat_frame = bytearray([0x55, 0xAA, 0x06, 0x19, 0x00, 0xE0])
        self.pairing_frames = {
            "left_back": bytearray([0x55, 0xAA, 0x06, 0x01, 0x10, 0x00]),
            "right_back": bytearray([0x55, 0xAA, 0x06, 0x01, 0x11, 0x00]),
            "front_left": bytearray([0x55, 0xAA, 0x06, 0x01, 0x00, 0x00]),
            "front_right": bytearray([0x55, 0xAA, 0x06, 0x01, 0x01, 0x00]),
            "spare": bytearray([0x55, 0xAA, 0x06, 0x01, 0x05, 0x00]),
        }
        self.stop_pairing_frame = bytearray([0x55, 0xAA, 0x06, 0x06, 0x00, 0x00])
        self.reset_frame = bytearray([0x55, 0xAA, 0x06, 0x58, 0x55, 0xE0])

    def auto_detect_port(self):
        import platform

        os_type = platform.system()
        if os_type == "Linux":
            return "/dev/ttyUSB0"
        elif os_type == "Darwin":  # macOS
            return "/dev/tty.usbserial-10"
        elif os_type == "Windows":
            return "COM3"
        else:
            raise ValueError("Unsupported OS")

    def connect(self):
        while True:
            try:
                logger.info("Connecting to TPMS device...")
                self.ser = serial.Serial(self.port, self.baudrate)
                self.ser.timeout = 5
                return
            except serial.SerialException as e:
                logger.error(f"Connection failed: {e}. Retrying in 2 seconds...")
                time.sleep(2)

    def send_heartbeat(self):
        try:
            self.ser.write(self.heartbeat_frame)
            logger.info("Heartbeat signal sent.")
        except serial.SerialException as e:
            logger.error(f"Failed to send heartbeat: {e}")

    def send_encryption(self, seed):
        frame = bytearray([0x55, 0xAA, 0x06, 0x5B, seed, 0xE0])
        try:
            self.ser.write(frame)
            logger.info("Encryption frame sent.")
        except serial.SerialException as e:
            logger.error(f"Failed to send encryption frame: {e}")

    def pair_left_back(self):
        self.ser.write(self.pairing_frames["left_back"])
        logger.info("Pairing command for left back tire sent.")

    def pair_right_back(self):
        self.ser.write(self.pairing_frames["right_back"])
        logger.info("Pairing command for right back tire sent.")

    def pair_front_left(self):
        self.ser.write(self.pairing_frames["front_left"])
        logger.info("Pairing command for front left tire sent.")

    def pair_front_right(self):
        self.ser.write(self.pairing_frames["front_right"])
        logger.info("Pairing command for front right tire sent.")

    def pair_spare(self):
        self.ser.write(self.pairing_frames["spare"])
        logger.info("Pairing command for spare tire sent.")

    def stop_pairing(self):
        self.ser.write(self.stop_pairing_frame)
        logger.info("Stop pairing command sent.")

    def decode_frame(self, frame):
        if len(frame) < 7:
            logger.warning(f"Incomplete frame received: {frame.hex()}")
            return None

        cmd = frame[2]
        if cmd == 0x08:  # Tire status
            return self.decode_tire_status(frame)
        elif cmd == 0x09:  # Query ID response
            return self.decode_query_id(frame)
        else:
            logger.warning(f"Unknown command received: {cmd} in frame {frame.hex()}")
            return None

    def decode_tire_status(self, frame):
        tire_positions = {
            0: "Left Rear",
            1: "Right Front",
            16: "Left Rear",
            17: "Right Rear",
            5: "Spare",
        }
        tire_position = frame[3]
        pressure = frame[4] * 3.44
        temperature = frame[5] - 50
        status = frame[6]

        no_signal = bool(status & 0b00100000)
        if no_signal:
            logger.warning(f"Sensor signal lost for tire position {tire_position}.")
            return {
                "tire_position": tire_position,
                "pressure": None,  # Invalidate pressure
                "temperature": None,  # Invalidate temperature
                "status": {
                    "low_power": bool(status & 0b00010000),
                    "leakage": bool(status & 0b00001000),
                    "no_signal": no_signal,
                },
            }

        return {
            "tire_position": tire_position,
            "pressure": pressure,
            "temperature": temperature,
            "status": {
                "low_power": bool(status & 0b00010000),
                "leakage": bool(status & 0b00001000),
                "no_signal": no_signal,
            },
        }

    def on_event_tire_state(self, event):
        """
        Handles tire state events.

        Parameters:
        event (dict): The decoded tire state event.
        """
        tire_positions = {
            1: "Left Front",
            2: "Right Front",
            3: "Left Rear",
            0: "Right Rear",
            5: "Spare",
        }

        # Extract tire data
        position = tire_positions.get(event["tire_position"], "Unknown")
        no_signal = event["status"]["no_signal"]
        pressure = event["pressure"]
        temperature = event["temperature"]

        # Check if the sensor is out of reception range
        if no_signal:
            logger.warning(f"Sensor signal lost for {position} tire.")
            return

        tire_data = {
            "position": position,
            "low_power": event["status"]["low_power"],
            "leakage": event["status"]["leakage"],
            "no_signal": no_signal,
            "pressure": pressure,
            "temperature": temperature,
        }

        logger.info(f"Tire State Event: {tire_data}")

        self.send_time_seed()

    def protocol_frame_filter(self, buffer):
        frames = []
        while len(buffer) >= 7:
            if buffer[0] == 0x55 and buffer[1] == 0xAA:
                length = buffer[2]
                if length <= len(buffer):
                    frames.append(buffer[:length])
                    buffer = buffer[length:]
                else:
                    # Wait for more data
                    break
            else:
                buffer = buffer[1:]
        return frames, buffer

    def send_time_seed(self):
        # Placeholder for sending time seed if needed
        logger.info("Time seed sent (placeholder).")

    def initiate_tire_exchange(self, tire1, tire2):
        tire_positions = {
            "left_front": 0x00,
            "right_front": 0x01,
            "left_back": 0x10,
            "right_back": 0x11,
            "spare": 0x05,
        }
        if tire1 not in tire_positions or tire2 not in tire_positions:
            logger.error("Invalid tire positions provided.")
            return

        frame = bytearray(
            [0x55, 0xAA, 0x07, 0x03, tire_positions[tire1], tire_positions[tire2], 0x00]
        )
        self.ser.write(frame)
        logger.info(f"Tire exchange initiated between {tire1} and {tire2}.")

    def main(self):
        self.connect()

        try:
            self.ser.write(self.querySensorID)
        except (serial.SerialException, serial.SerialTimeoutException):
            logger.error("Serial port unavailable, reconnecting...")
            self.ser.close()
            self.connect()
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            raise

        # Main loop to read and process data from the TPMS device
        while True:
            try:
                raw_data = self.ser.read(256)  # Read up to 256 bytes at a time
                if raw_data:
                    logger.info(f"Raw data received: {raw_data.hex()}")
                    self.incoming_buffer.extend(raw_data)
                    frames, self.incoming_buffer = self.protocol_frame_filter(
                        self.incoming_buffer
                    )
                    for frame in frames:
                        event = self.decode_frame(frame)
                        if event:
                            self.on_event_tire_state(event)
            except (serial.SerialException, serial.SerialTimeoutException):
                logger.error("Serial port unavailable, reconnecting...")
                self.ser.close()
                self.connect()
            except KeyboardInterrupt:
                if self.ser is not None:
                    self.ser.close()
                    logger.info("Port closed!")
                logger.info("KeyboardInterrupt detected, exiting...")
                break


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Tire Pressure Monitoring System (TPMS) Python Library"
    )
    parser.add_argument(
        "--port", type=str, default=None, help="Serial port (default: auto-detect)"
    )
    parser.add_argument(
        "--baudrate", type=int, default=19200, help="Baud rate (default: 19200)"
    )
    parser.add_argument(
        "--temp_unit",
        type=str,
        choices=["Celsius", "Fahrenheit"],
        default="Celsius",
        help="Temperature unit (default: Celsius)",
    )
    parser.add_argument(
        "--pressure_unit",
        type=str,
        choices=["kPa", "psi", "bar"],
        default="kPa",
        help="Pressure unit (default: kPa)",
    )
    parser.add_argument(
        "--hardware_version",
        type=str,
        choices=["TY06", "TY05"],
        default="TY06",
        help="Hardware version (default: TY06)",
    )

    args = parser.parse_args()

    tpms = TPMS(
        port=args.port,
        baudrate=args.baudrate,
        temp_unit=args.temp_unit,
        pressure_unit=args.pressure_unit,
        hardware_version=args.hardware_version,
    )
    tpms.main()
