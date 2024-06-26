# tpms/tpms.py
import serial
import time
import threading
import logging
import platform

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TPMS:
    def __init__(
        self,
        port=None,
        baudrate=19200,
        temp_unit="Celsius",
        pressure_unit="kPa",
        hardware_version="TY06",
    ):
        self.ser = None
        self.baudrate = baudrate
        self.is_seed_ack_ok = False
        self.error_count = 0
        self.start_data_time = -1
        self.high_press_stamp = 310
        self.low_press_stamp = 180
        self.hi_temp_stamp = 75
        self.front_left = None
        self.front_right = None
        self.back_left = None
        self.back_right = None
        self.spare_tire = None
        self.incoming_buffer = bytearray()
        self.hardware_version = hardware_version.upper()

        # Set default port based on the operating system if not provided
        if port is None:
            if platform.system() == "Windows":
                self.port = "COM1"  # Default COM port on Windows
            elif platform.system() == "Darwin":
                self.port = "/dev/tty.usbserial"  # Default port on macOS
            else:
                self.port = "/dev/ttyUSB0"  # Default port on Linux
        else:
            self.port = port

        # Configuration for temperature and pressure units
        self.config = {
            "temperature_unit": temp_unit,  # Options: "Celsius", "Fahrenheit"
            "pressure_unit": pressure_unit,  # Options: "kPa", "psi", "bar"
        }

    def connect(self):
        """
        Attempts to establish a connection to the serial device.
        Retries every 2 seconds if the connection fails.
        """
        while True:
            try:
                logger.info(f"Connecting to serial port {self.port}...")
                self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
                logger.info("Connected to serial port.")
                return
            except serial.SerialException:
                logger.warning(
                    "Failed to connect to serial port. Retrying in 2 seconds..."
                )
                time.sleep(2)

    def send(self, frame):
        """
        Sends a byte frame to the serial device.

        Parameters:
        frame (bytearray): The byte frame to send.
        """
        try:
            self.ser.write(frame)
            time.sleep(0.06)  # Sleep for 60 ms to mimic the behavior in Java code
        except serial.SerialException as e:
            logger.error(f"Failed to send frame: {e}")

    @staticmethod
    def calc_cc(buf):
        """
        Verifies the checksum (CC) for a given byte buffer.

        Parameters:
        buf (bytearray): The byte buffer to verify the checksum for.

        Returns:
        bool: True if the checksum is correct, False otherwise.
        """
        datalen = buf[2]
        if datalen == 0:
            logger.error(f"Invalid data length: {buf}")
            return False
        elif TPMS.sum_cc(buf) == buf[datalen - 1]:
            return True
        else:
            return False

    @staticmethod
    def sum_cc(buf):
        """
        Calculates the checksum (CC) for a given byte buffer.

        Parameters:
        buf (bytearray): The byte buffer to calculate the checksum for.

        Returns:
        byte: The calculated checksum.
        """
        datalen = buf[2]
        calc = buf[0]
        for i in range(1, datalen - 1):
            calc ^= buf[i]
        return calc

    @staticmethod
    def erase(buf, n):
        """
        Erases n bytes from the beginning of the buffer.

        Parameters:
        buf (bytearray): The byte buffer to erase bytes from.
        n (int): The number of bytes to erase.

        Returns:
        bytearray: The buffer with n bytes erased from the beginning.
        """
        return buf[n:]

    def protocol_frame_filter(self, buf):
        """
        Filters and processes protocol frames.

        Parameters:
        buf (bytearray): The byte buffer to filter and process.

        Returns:
        list: A list of valid protocol frames.
        """
        frames = []
        while len(buf) >= 3:
            if buf[0] == 85 and buf[1] == 170:
                framelen = buf[2]
                if framelen > len(buf):
                    break
                if self.calc_cc(buf[:framelen]):
                    frames.append(buf[:framelen])
                    buf = self.erase(buf, framelen)
                else:
                    buf = self.erase(buf, 1)
            else:
                buf = self.erase(buf, 1)
        return frames, buf

    def decode_frame(self, frame):
        """
        Decodes the raw frame data into a meaningful event.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded event.
        """
        cmd = frame[2]
        if cmd == 8:
            return self.decode_tire_status(frame)
        elif cmd == 6:
            if frame[3] == 24:
                return self.decode_pairing_ack(frame)
            elif frame[3] == 0 and frame[4] == 136:  # -120 as unsigned
                return self.decode_heartbeat_event()
            elif frame[3] == -75:
                return self.decode_time_seed_event(frame)
        elif cmd == 9:
            return self.decode_query_id(frame)
        elif cmd == 7 and frame[3] == 48:
            return self.decode_tire_exchange(frame)
        return None

    def decode_tire_status(self, frame):
        """
        Decodes the tire status frame.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded tire status event.
        """
        tire_position = frame[3]
        pressure = frame[4] * 3.44  # Pressure in kPa
        temperature = frame[5] - 50  # Temperature in Celsius
        status_byte = frame[6]

        # Convert pressure to the desired unit
        if self.config["pressure_unit"] == "psi":
            pressure *= 0.1450377
        elif self.config["pressure_unit"] == "bar":
            pressure /= 100

        # Convert temperature to the desired unit
        if self.config["temperature_unit"] == "Fahrenheit":
            temperature = temperature * 9 / 5 + 32

        status = {
            "low_power": bool(status_byte & 0x10),
            "leakage": bool(status_byte & 0x08),
            "no_signal": bool(status_byte & 0x20),
        }

        return {
            "type": "tire_status",
            "tire_position": tire_position,
            "pressure": pressure,
            "temperature": temperature,
            "status": status,
        }

    @staticmethod
    def decode_pairing_ack(frame):
        """
        Decodes the pairing acknowledgment frame.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded pairing acknowledgment event.
        """
        tire_position = frame[4]
        tire_positions = {
            0: "Left Rear",
            1: "Right Front",
            16: "Left Front",
            17: "Right Rear",
            5: "Spare",
        }
        return {
            "type": "pairing_ack",
            "tire_position": tire_positions.get(tire_position, "Unknown"),
        }

    @staticmethod
    def decode_heartbeat_event():
        """
        Decodes the heartbeat event frame.

        Returns:
        dict: The decoded heartbeat event.
        """
        return {"type": "heartbeat"}

    @staticmethod
    def decode_time_seed_event(frame):
        """
        Decodes the time seed event frame.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded time seed event.
        """
        seed = frame[4]
        return {"type": "time_seed", "seed": seed}

    @staticmethod
    def decode_query_id(frame):
        """
        Decodes the query ID response frame.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded query ID response event.
        """
        tire_position = frame[3]
        id_bytes = frame[4:8]
        tire_id = "".join(format(x, "02x") for x in id_bytes).upper()
        tire_positions = {
            1: "Left Front",
            2: "Right Front",
            3: "Left Rear",
            4: "Right Rear",
            5: "Spare",
        }
        return {
            "type": "query_id",
            "tire_position": tire_positions.get(tire_position, "Unknown"),
            "tire_id": tire_id,
        }

    @staticmethod
    def decode_tire_exchange(frame):
        """
        Decodes the tire exchange frame.

        Parameters:
        frame (bytearray): The raw frame data.

        Returns:
        dict: The decoded tire exchange event.
        """
        exchange_map = {
            (0, 1): "Left Front - Right Front",
            (0, 16): "Left Front - Left Rear",
            (0, 17): "Left Front - Right Rear",
            (1, 16): "Right Front - Left Rear",
            (1, 17): "Right Front - Right Rear",
            (16, 17): "Left Rear - Right Rear",
            (0, 5): "Left Front - Spare",
            (1, 5): "Right Front - Spare",
            (16, 5): "Left Rear - Spare",
            (17, 5): "Right Rear - Spare",
        }
        exchange = exchange_map.get((frame[4], frame[5]), "Unknown Exchange")
        return {"type": "tire_exchange", "exchange": exchange}

    def heartbeat(self):
        """
        Sends a heartbeat signal and encryption seed.
        """
        self.send_heartbeat()
        self.send_encryption(int(time.time()) & 0xFF)
        threading.Timer(1, self.heartbeat).start()

    def send_heartbeat(self):
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 25, 0, 224]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 24, 0, 223]))

    def send_encryption(self, seed):
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 91, seed, 224]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 90, seed, 223]))

    def pair_back_left(self):
        logger.info("Pairing back left tire ID")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 1, 16, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 1, 16, 1]))

    def pair_back_right(self):
        logger.info("Pairing back right tire ID")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 1, 17, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 1, 17, 1]))

    def pair_front_left(self):
        logger.info("Pairing front left tire ID")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 1, 0, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 1, 0, 1]))

    def pair_front_right(self):
        logger.info("Pairing front right tire ID")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 1, 1, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 1, 1, 1]))

    def pair_spare(self):
        logger.info("Pairing spare tire ID")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 1, 5, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 1, 5, 1]))

    def stop_pairing(self):
        logger.info("Stopping pairing")
        if self.hardware_version == "TY06":
            self.send(bytearray([85, 170, 6, 6, 0, 0]))
        elif self.hardware_version == "TY05":
            self.send(bytearray([85, 170, 6, 6, 0, 1]))

    def send_time_seed(self):
        logger.info("Sending time seed...")
        if not self.is_seed_ack_ok:
            if self.hardware_version == "TY06":
                self.send(bytearray([85, 170, 6, 91, int(time.time()) & 255, 224]))
            elif self.hardware_version == "TY05":
                self.send(bytearray([85, 170, 6, 90, int(time.time()) & 255, 223]))
        self.error_count = 0

    def on_event(self, event):
        """
        Handles the events decoded from the frame.

        Parameters:
        event (dict): The decoded event.
        """
        if event["type"] == "tire_status":
            self.on_event_tire_state(event)
        elif event["type"] == "heartbeat":
            self.HeartbeatEventAck()
        elif event["type"] == "pairing_ack":
            logger.info(f"Pairing successful for tire: {event['tire_position']}")
        elif event["type"] == "time_seed":
            logger.info(f"Time seed event with seed: {event['seed']}")
        elif event["type"] == "query_id":
            logger.info(
                f"Query ID response for tire: {event['tire_position']} with ID: {event['tire_id']}"
            )
        elif event["type"] == "tire_exchange":
            logger.info(f"Tire exchange event: {event['exchange']}")
        self.start_data_time = time.time()

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

        tire_data = {
            "position": tire_positions.get(event["tire_position"], "Unknown"),
            "low_power": event["status"]["low_power"],
            "leakage": event["status"]["leakage"],
            "no_signal": event["status"]["no_signal"],
            "pressure": event["pressure"],
            "temperature": event["temperature"],
        }

        logger.info(f"Tire State Event: {tire_data}")

        self.send_time_seed()

    def initiate_tire_exchange(self, tire1, tire2):
        """
        Initiates a tire exchange between two tires.

        Parameters:
        tire1 (int): The position of the first tire.
        tire2 (int): The position of the second tire.
        """
        # Tire position values:
        # 0 - Left Front
        # 1 - Right Front
        # 16 - Left Rear
        # 17 - Right Rear
        # 5 - Spare
        exchange_map = {
            (0, 1): bytearray([85, 170, 7, 3, 0, 1, 0]),  # Left Front - Right Front
            (0, 16): bytearray([85, 170, 7, 3, 0, 16, 0]),  # Left Front - Left Rear
            (0, 17): bytearray([85, 170, 7, 3, 0, 17, 0]),  # Left Front - Right Rear
            (1, 16): bytearray([85, 170, 7, 3, 1, 16, 0]),  # Right Front - Left Rear
            (1, 17): bytearray([85, 170, 7, 3, 1, 17, 0]),  # Right Front - Right Rear
            (16, 17): bytearray([85, 170, 7, 3, 16, 17, 0]),  # Left Rear - Right Rear
            (0, 5): bytearray([85, 170, 7, 3, 0, 5, 0]),  # Left Front - Spare
            (1, 5): bytearray([85, 170, 7, 3, 1, 5, 0]),  # Right Front - Spare
            (16, 5): bytearray([85, 170, 7, 3, 16, 5, 0]),  # Left Rear - Spare
            (17, 5): bytearray([85, 170, 7, 3, 17, 5, 0]),  # Right Rear - Spare
        }
        frame = exchange_map.get((tire1, tire2), None)
        if frame is not None:
            logger.info(f"Initiating tire exchange: {tire1} <-> {tire2}")
            self.send(frame)
        else:
            logger.error(f"Invalid tire exchange request: {tire1} <-> {tire2}")

    def main(self):
        """
        Main function to establish the serial connection, read frames, and decode them.
        """
        self.connect()
        self.heartbeat()  # Start sending heartbeats

        while True:
            try:
                raw_data = self.ser.read(256)  # Read up to 256 bytes at a time
                if raw_data:
                    self.incoming_buffer.extend(raw_data)
                    frames, self.incoming_buffer = self.protocol_frame_filter(
                        self.incoming_buffer
                    )
                    for frame in frames:
                        event = self.decode_frame(frame)
                        if event:
                            self.on_event(event)
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
        "--port",
        type=str,
        help="Serial port address (default: auto-detect based on OS)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=19200,
        help="Baud rate for the serial connection (default: 19200)",
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
