#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TPMS Library - Python implementation of TPMS (Tire Pressure Monitoring System)

This library provides functionality to interface with TPMS sensors via USB,
allowing for reading tire data, pairing sensors, and managing tire positions.
"""

import logging
import time
import threading
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Tuple, Union, Callable
import serial
import serial.tools.list_ports

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("tpms_python.tpms_lib")

# Constants
DEFAULT_TIMEOUT = 1.0  # seconds
DEFAULT_BAUDRATE = 19200  # Correct baudrate for this TPMS device
# Alternative baudrates to try if the default doesn't work
ALTERNATIVE_BAUDRATES = [9600, 115200, 38400, 57600]


class TirePosition(IntEnum):
    """Enum representing tire positions"""

    REAR_LEFT = 0
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    REAR_RIGHT = 3
    SPARE_TIRE = 5


class Command(IntEnum):
    """Command codes used in the TPMS protocol"""

    HEARTBEAT = 6
    EXCHANGE_TIRES = 7
    TIRE_STATE = 8
    QUERY_ID = 9
    RESET = 10


class TireState:
    """Class representing the state of a tire"""

    def __init__(self):
        self.tire_id = ""
        self.air_pressure = 0  # in kPa
        self.temperature = 0  # in Celsius
        self.is_leaking = False
        self.is_low_power = False
        self.no_signal = False
        self.error = ""
        self.last_update = 0

    def __str__(self):
        return (
            f"TireState(ID: {self.tire_id}, Pressure: {self.air_pressure} kPa, "
            f"Temp: {self.temperature}°C, Leaking: {self.is_leaking}, "
            f"Low Power: {self.is_low_power}, No Signal: {self.no_signal})"
        )


class TPMSProtocol:
    """Handles the low-level protocol for TPMS communication"""

    # Frame structure: [0x55, 0xAA, length, command, data..., checksum]
    HEADER_1 = 0x55
    HEADER_2 = 0xAA

    def __init__(self):
        self.buffer = bytearray()

    def create_frame(self, command: int, data: List[int]) -> bytes:
        """Create a frame to send to the TPMS device"""
        # Frame length is command byte + data bytes + checksum byte
        length = 1 + len(data) + 1

        # Create frame without checksum
        frame = bytearray([self.HEADER_1, self.HEADER_2, length, command] + data)

        # Calculate checksum (XOR of all bytes starting from HEADER_1)
        checksum = frame[0]
        for i in range(1, len(frame)):
            checksum ^= frame[i]

        # Add checksum to frame
        frame.append(checksum)

        return bytes(frame)

    def parse_frame(self, data: bytes) -> Optional[Tuple[int, List[int]]]:
        """Parse a frame received from the TPMS device"""
        # Add data to buffer
        self.buffer.extend(data)

        # Process buffer
        while (
            len(self.buffer) >= 5
        ):  # Minimum frame size (header + length + command + checksum)
            # Check for header
            if self.buffer[0] != self.HEADER_1 or self.buffer[1] != self.HEADER_2:
                # Remove first byte and continue
                self.buffer.pop(0)
                continue

            # Get frame length
            length = self.buffer[2]

            # Check if we have a complete frame
            if len(self.buffer) < length:
                # Not enough data yet
                break

            # Extract frame
            frame = self.buffer[:length]

            # Remove frame from buffer
            self.buffer = self.buffer[length:]

            # Verify checksum
            calculated_checksum = frame[0]
            for i in range(1, length - 1):
                calculated_checksum ^= frame[i]

            if calculated_checksum != frame[-1]:
                logger.warning(
                    f"Checksum mismatch: {calculated_checksum} != {frame[-1]}"
                )
                continue

            # Extract command and data
            command = frame[3]
            data = list(frame[4:-1])

            return command, data

        return None


class TPMSDevice:
    """Main class for interfacing with TPMS hardware"""

    def __init__(self, port=None, baudrate=DEFAULT_BAUDRATE, timeout=DEFAULT_TIMEOUT):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.protocol = TPMSProtocol()
        self.running = False
        self.read_thread = None

        # Tire states
        self.tire_states = {
            TirePosition.FRONT_LEFT: TireState(),
            TirePosition.FRONT_RIGHT: TireState(),
            TirePosition.REAR_LEFT: TireState(),
            TirePosition.REAR_RIGHT: TireState(),
            TirePosition.SPARE_TIRE: TireState(),
        }

        # Callbacks
        self.on_tire_state_update = None
        self.on_pairing_complete = None
        self.on_exchange_complete = None

        # Settings
        self.high_pressure_threshold = 310  # kPa (default)
        self.low_pressure_threshold = 180  # kPa (default)
        self.high_temp_threshold = 75  # Celsius (default)
        self.spare_tire_enabled = False

    def find_device(self) -> List[str]:
        """Find available TPMS devices"""
        # Look for USB-Serial devices that are likely to be TPMS devices
        # Common VID/PID combinations for USB-Serial adapters used with TPMS
        # This list can be expanded based on known devices
        tpms_vid_pid = [
            (0x0403, 0x6001),  # FTDI
            (0x1A86, 0x7523),  # CH340
            (0x10C4, 0xEA60),  # CP210x
            (0x067B, 0x2303),  # Prolific
        ]

        # Keywords that might indicate a TPMS device in the description
        tpms_keywords = ["tpms", "tire", "pressure", "usb", "serial", "usbserial"]

        # First, try to find devices with known VID/PID combinations
        tpms_ports = []
        all_ports = []

        for port in serial.tools.list_ports.comports():
            all_ports.append(port.device)

            # Check if this port matches any known VID/PID
            if (
                hasattr(port, "vid")
                and hasattr(port, "pid")
                and port.vid is not None
                and port.pid is not None
            ):
                if (port.vid, port.pid) in tpms_vid_pid:
                    logger.info(
                        f"Found potential TPMS device: {port.device} (VID:PID={port.vid:04x}:{port.pid:04x})"
                    )
                    tpms_ports.append(port.device)
                    continue

            # Check if description contains any keywords
            if hasattr(port, "description") and port.description:
                desc_lower = port.description.lower()
                if any(keyword in desc_lower for keyword in tpms_keywords):
                    logger.info(
                        f"Found potential TPMS device: {port.device} (Description: {port.description})"
                    )
                    tpms_ports.append(port.device)
                    continue

            # Check if device contains 'usb' or 'serial' in the name
            if "usb" in port.device.lower() or "serial" in port.device.lower():
                logger.info(f"Found potential USB-Serial device: {port.device}")
                tpms_ports.append(port.device)

        # If we found any potential TPMS devices, return those
        if tpms_ports:
            return tpms_ports

        # Otherwise, return all ports as a fallback
        logger.warning(
            "No specific TPMS devices identified. Returning all available ports."
        )
        return all_ports

    def connect(self, port=None) -> bool:
        """Connect to the TPMS device"""
        if port:
            self.port = port

        if not self.port:
            available_ports = self.find_device()
            if not available_ports:
                logger.error("No serial ports found")
                return False
            self.port = available_ports[0]

        # Try the default baudrate first
        if self._try_connect(self.baudrate):
            return True

        # If that fails, try alternative baudrates
        logger.info(
            f"Failed to connect with baudrate {self.baudrate}, trying alternatives..."
        )
        for baudrate in ALTERNATIVE_BAUDRATES:
            if self._try_connect(baudrate):
                return True

        logger.error(f"Failed to connect to {self.port} with any baudrate")
        return False

    def _try_connect(self, baudrate) -> bool:
        """Try to connect with a specific baudrate"""
        try:
            # Close existing connection if any
            if self.serial and self.serial.is_open:
                self.serial.close()

            logger.info(f"Trying to connect to {self.port} with baudrate {baudrate}...")
            self.serial = serial.Serial(
                port=self.port, baudrate=baudrate, timeout=self.timeout
            )
            self.baudrate = baudrate  # Update the baudrate if successful
            logger.info(f"Connected to {self.port} with baudrate {baudrate}")

            # Start reading thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()

            # Send heartbeat to initialize connection
            if self._send_heartbeat():
                logger.info("Heartbeat sent successfully")
                return True
            else:
                logger.warning("Failed to send heartbeat")
                self.disconnect()
                return False

        except serial.SerialException as e:
            logger.warning(
                f"Failed to connect to {self.port} with baudrate {baudrate}: {e}"
            )
            return False

    def disconnect(self):
        """Disconnect from the TPMS device"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info(f"Disconnected from {self.port}")

    def _read_loop(self):
        """Background thread for reading data from the device"""
        while self.running and self.serial and self.serial.is_open:
            try:
                # Read data
                data = self.serial.read(100)  # Read up to 100 bytes
                if data:
                    # Log raw data for debugging
                    hex_data = " ".join([f"{b:02X}" for b in data])
                    logger.debug(f"Received raw data: {hex_data}")

                    # Process data
                    result = self.protocol.parse_frame(data)
                    if result:
                        command, cmd_data = result
                        logger.debug(
                            f"Parsed command: {command}, data: {[f'{b:02X}' for b in cmd_data]}"
                        )
                        self._handle_command(command, cmd_data)
                    else:
                        logger.debug("No complete frame found in data")
            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                self.running = False
            except Exception as e:
                logger.error(f"Error in read loop: {e}")
                import traceback

                logger.error(traceback.format_exc())

            # Small delay to prevent CPU hogging
            time.sleep(0.01)

    def _handle_command(self, command: int, data: List[int]):
        """Handle received commands"""
        if command == Command.TIRE_STATE:
            self._handle_tire_state(data)
        elif command == Command.QUERY_ID:
            self._handle_query_id(data)
        elif command == Command.EXCHANGE_TIRES:
            self._handle_exchange_tires(data)
        # Handle direct tire position updates (based on debug logs)
        elif command in [0, 1, 16, 17]:  # These appear to be direct position codes
            self._handle_direct_tire_update(command, data)
        elif command == 5:  # Spare tire
            self._handle_direct_tire_update(command, data)
        # Handle pairing response (from FrameDecode3.java)
        elif command == Command.HEARTBEAT and len(data) > 0:
            # Log all HEARTBEAT commands for debugging
            logger.debug(
                f"HEARTBEAT command received: data={[f'{b:02X}' for b in data]}"
            )

            if data[0] == 24:  # Pairing success response
                logger.info(
                    f"Pairing success response received: {[f'{b:02X}' for b in data]}"
                )
                self._handle_pairing_complete(data)
            elif data[0] == 1:  # Alternative pairing command format
                logger.info(
                    f"Alternative pairing command format received: {[f'{b:02X}' for b in data]}"
                )
                # If this is a pairing command, handle it
                if len(data) > 1:
                    # The second byte might contain the position code
                    position_code = data[1]
                    # Create a new data array with the expected format for _handle_pairing_complete
                    modified_data = [24, position_code]
                    self._handle_pairing_complete(modified_data)
            elif data[0] == 6:  # Another alternative pairing command format
                logger.info(
                    f"Another alternative pairing command format received: {[f'{b:02X}' for b in data]}"
                )
                # If this is a pairing command, handle it
                if len(data) > 1:
                    # The second byte might contain the position code
                    position_code = data[1]
                    # Create a new data array with the expected format for _handle_pairing_complete
                    modified_data = [24, position_code]
                    self._handle_pairing_complete(modified_data)
            elif data[0] == 0 and len(data) > 1 and data[1] == 0x88:  # Heartbeat
                logger.debug("Heartbeat received")
            elif data[0] == 0xB5:  # TimeSeed response
                logger.debug(
                    f"TimeSeed response received: {data[1] if len(data) > 1 else 'unknown'}"
                )
            else:
                # Log unhandled HEARTBEAT payloads to help debug pairing issues
                logger.debug(f"Unhandled HEARTBEAT data: {[f'{b:02X}' for b in data]}")
        else:
            logger.debug(f"Unhandled command: {command}, data: {data}")

    def _handle_pairing_complete(self, data: List[int]):
        """Handle pairing complete response"""
        if len(data) < 2:
            logger.warning("Invalid pairing complete data")
            return

        # Get position code from data[1]
        position_code = data[1]
        position = None

        # Map position codes to enum values (from FrameDecode3.java)
        position_map = {
            0: TirePosition.FRONT_LEFT,  # Left Front
            1: TirePosition.FRONT_RIGHT,  # Right Front
            16: TirePosition.REAR_LEFT,  # Left Rear
            17: TirePosition.REAR_RIGHT,  # Right Rear
            5: TirePosition.SPARE_TIRE,  # Spare Tire
        }

        if position_code in position_map:
            position = position_map[position_code]

            # Get the tire ID (if available)
            tire_id = self.tire_states[position].tire_id
            if not tire_id:
                # If no ID is available, generate one based on position
                tire_id = f"SENSOR_{position.name}"
                self.tire_states[position].tire_id = tire_id

            logger.info(f"Pairing complete for {position.name}: ID {tire_id}")

            # Call pairing callback if registered
            if self.on_pairing_complete:
                self.on_pairing_complete(position, tire_id)

            # Query sensor IDs to get the actual ID
            self.query_sensor_ids()
        else:
            logger.warning(
                f"Unknown tire position code in pairing response: {position_code}"
            )

    def _handle_direct_tire_update(self, position_code: int, data: List[int]):
        """Handle direct tire state updates where the command is the position code"""
        if len(data) < 2:
            logger.warning(
                f"Invalid direct tire update data for position {position_code}"
            )
            return

        # Map position codes to enum values
        position_map = {
            0: TirePosition.FRONT_LEFT,
            1: TirePosition.FRONT_RIGHT,
            16: TirePosition.REAR_LEFT,
            17: TirePosition.REAR_RIGHT,
            5: TirePosition.SPARE_TIRE,
        }

        if position_code not in position_map:
            logger.warning(f"Unknown tire position code: {position_code}")
            return

        position = position_map[position_code]
        tire_state = self.tire_states[position]

        # Store previous pressure for change detection
        previous_pressure = tire_state.air_pressure

        # Update tire state based on the data format observed in logs
        # Format appears to be [pressure, temperature, flags]
        pressure_raw = data[0]
        temp_raw = data[1]
        flags = 0
        if len(data) > 2:
            flags = data[2]

        # Convert raw values to actual values
        tire_state.air_pressure = int(pressure_raw * 3.44)  # Convert to kPa
        tire_state.temperature = temp_raw - 50  # Convert to Celsius

        # Parse flags if present
        tire_state.no_signal = bool(flags & 0x20)
        tire_state.is_leaking = bool(flags & 0x08)
        tire_state.is_low_power = bool(flags & 0x04)

        # Update timestamp
        tire_state.last_update = time.time()

        logger.debug(f"Updated tire state for {position.name}: {tire_state}")

        # Call callback if registered
        if self.on_tire_state_update:
            self.on_tire_state_update(position, tire_state)

        # Check for significant pressure change (which might indicate pairing)
        # If pressure was 0 and now it's not, or if it changed by more than 20 kPa
        if (previous_pressure == 0 and tire_state.air_pressure > 0) or abs(
            tire_state.air_pressure - previous_pressure
        ) > 20:
            logger.info(
                f"Significant pressure change detected for {position.name}: {previous_pressure} -> {tire_state.air_pressure} kPa"
            )

            # Get or generate tire ID
            tire_id = tire_state.tire_id
            if not tire_id:
                # If no ID is available, generate one based on position
                tire_id = f"SENSOR_{position.name}"
                tire_state.tire_id = tire_id

            logger.info(f"Auto-pairing detected for {position.name}: ID {tire_id}")

            # Call pairing callback if registered
            if self.on_pairing_complete:
                self.on_pairing_complete(position, tire_id)

    def _handle_tire_state(self, data: List[int]):
        """Handle tire state update"""
        if len(data) < 4:
            logger.warning("Invalid tire state data")
            return

        # Parse tire position
        position_code = data[0]
        position = None

        if position_code == 0:
            position = TirePosition.FRONT_LEFT
        elif position_code == 1:
            position = TirePosition.FRONT_RIGHT
        elif position_code == 16:
            position = TirePosition.REAR_LEFT
        elif position_code == 17:
            position = TirePosition.REAR_RIGHT
        elif position_code == 5:
            position = TirePosition.SPARE_TIRE
        else:
            logger.warning(f"Unknown tire position code: {position_code}")
            return

        # Get tire state
        tire_state = self.tire_states[position]

        # Update tire state
        tire_state.air_pressure = int(data[1] * 3.44)  # Convert to kPa
        tire_state.temperature = data[2] - 50  # Convert to Celsius

        # Parse flags
        flags = data[3]
        tire_state.no_signal = bool(flags & 0x20)
        tire_state.is_leaking = bool(flags & 0x08)
        tire_state.is_low_power = bool(flags & 0x04)

        # Update timestamp
        tire_state.last_update = time.time()

        # Call callback if registered
        if self.on_tire_state_update:
            self.on_tire_state_update(position, tire_state)

    def _handle_query_id(self, data: List[int]):
        """Handle query ID response

        This method handles the query ID response from the TPMS device.
        According to the Android app (FrameDecode3.java), the format is:
        - Position code is in data[0]
        - ID bytes are in data[1:5]

        The position codes are:
        - 1 = Left Front (FRONT_LEFT)
        - 2 = Right Front (FRONT_RIGHT)
        - 3 = Left Rear (REAR_LEFT)
        - 4 = Right Rear (REAR_RIGHT)
        - 5 = Spare Tire (SPARE_TIRE)
        """
        # Log the raw data for debugging
        logger.info(f"Received query ID response: {[f'{b:02X}' for b in data]}")

        if len(data) < 5:
            logger.warning(f"Query ID response too short: {data}")
            return

        # Parse tire position according to the Android app
        position_code = data[0]
        position = None

        # Map position codes to enum values (from FrameDecode3.java)
        position_map = {
            1: TirePosition.FRONT_LEFT,  # Left Front
            2: TirePosition.FRONT_RIGHT,  # Right Front
            3: TirePosition.REAR_LEFT,  # Left Rear
            4: TirePosition.REAR_RIGHT,  # Right Rear
            5: TirePosition.SPARE_TIRE,  # Spare Tire
        }

        if position_code in position_map:
            position = position_map[position_code]

            # Parse ID (from FrameDecode3.java)
            # paire2.mID = Util.byteToUpperString(frame[4]) + Util.byteToUpperString(frame[5]) +
            #              Util.byteToUpperString(frame[6]) + Util.byteToUpperString(frame[7]);
            # In our case, data starts at frame[4], so we need data[0:4]
            tire_id = "".join(f"{b:02X}" for b in data[1:5])

            # Update tire state
            self.tire_states[position].tire_id = tire_id

            logger.info(f"Tire ID for position {position.name}: {tire_id}")

            # Call pairing callback if registered
            if self.on_pairing_complete:
                self.on_pairing_complete(position, tire_id)
        else:
            logger.warning(f"Unknown tire position code: {position_code}")

    def _handle_exchange_tires(self, data: List[int]):
        """Handle tire exchange response"""
        if len(data) < 2:
            logger.warning("Invalid tire exchange data")
            return

        # Get positions
        pos1 = data[0]
        pos2 = data[1]

        # Map position codes to enum values
        position_map = {
            0: TirePosition.FRONT_LEFT,
            1: TirePosition.FRONT_RIGHT,
            16: TirePosition.REAR_LEFT,
            17: TirePosition.REAR_RIGHT,
            5: TirePosition.SPARE_TIRE,
        }

        # Get position names
        if pos1 in position_map and pos2 in position_map:
            position1 = position_map[pos1]
            position2 = position_map[pos2]

            logger.info(f"Tires exchanged: {position1.name} and {position2.name}")

            # Call callback if registered
            if self.on_exchange_complete:
                self.on_exchange_complete(position1, position2)

    def _send_command(self, command: int, data: List[int]) -> bool:
        """Send a command to the TPMS device"""
        if not self.serial or not self.serial.is_open:
            logger.error("Not connected to device")
            return False

        frame = self.protocol.create_frame(command, data)
        try:
            self.serial.write(frame)
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def _send_heartbeat(self) -> bool:
        """Send heartbeat command"""
        return self._send_command(Command.HEARTBEAT, [25, 0, 0xE0])

    def query_sensor_ids(self) -> bool:
        """Query all sensor IDs

        This method sends the same command as the Android app to query sensor IDs.
        The command is [0x55, 0xAA, 0x06, 0x07, 0x00, 0x00].
        """
        # Send the exact same command as the Android app
        # From FrameEncode3.java: this.FrameEn.send(new byte[]{85, -86, 6, 7, 0, 0});
        # 85 = 0x55, -86 = 0xAA (signed byte), 6 = length, 7 = command code
        frame = bytes([0x55, 0xAA, 0x06, 0x07, 0x00, 0x00])

        try:
            self.serial.write(frame)
            logger.info("Sent query sensor IDs command (Android app format)")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send query sensor IDs command: {e}")
            return False

    def pair_sensor(self, position: TirePosition) -> bool:
        """Start pairing a sensor for the specified position

        This method sends the exact same command as the Android app for pairing.
        From FrameEncode3.java:
        - paireFrontLeft: [85, -86, 6, 1, 0, 0]
        - paireFrontRight: [85, -86, 6, 1, 1, 0]
        - paireBackLeft: [85, -86, 6, 1, 16, 0]
        - paireBackRight: [85, -86, 6, 1, 17, 0]
        - paireSpTired: [85, -86, 6, 1, 5, 0]
        """
        position_code = {
            TirePosition.FRONT_LEFT: 0,
            TirePosition.FRONT_RIGHT: 1,
            TirePosition.REAR_LEFT: 16,
            TirePosition.REAR_RIGHT: 17,
            TirePosition.SPARE_TIRE: 5,
        }[position]

        # Send the exact same command as the Android app
        # [85, -86, 6, 1, position_code, 0]
        # 85 = 0x55, -86 = 0xAA (signed byte), 6 = length, 1 = command code
        frame = bytes([0x55, 0xAA, 0x06, 0x01, position_code, 0x00])

        try:
            self.serial.write(frame)
            logger.info(
                f"Sent pair sensor command for {position.name} (Android app format)"
            )
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send pair sensor command: {e}")
            return False

    def stop_pairing(self) -> bool:
        """Stop the pairing process

        This method sends the exact same command as the Android app for stopping pairing.
        From FrameEncode3.java:
        - stopPaire: [85, -86, 6, 6, 0, 0]
        """
        # Send the exact same command as the Android app
        # [85, -86, 6, 6, 0, 0]
        # 85 = 0x55, -86 = 0xAA (signed byte), 6 = length, 6 = command code
        frame = bytes([0x55, 0xAA, 0x06, 0x06, 0x00, 0x00])

        try:
            self.serial.write(frame)
            logger.info("Sent stop pairing command (Android app format)")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send stop pairing command: {e}")
            return False

    def exchange_tires(self, position1: TirePosition, position2: TirePosition) -> bool:
        """Exchange two tire positions

        This method sends the exact same command as the Android app for exchanging tire positions.
        From FrameEncode3.java:
        - exchangeLeftFrontRightFront: [85, -86, 7, 3, 0, 1, 0]
        - exchangeLeftFrontLeftBack: [85, -86, 7, 3, 0, 16, 0]
        - exchangeLeftFrontRightBack: [85, -86, 7, 3, 0, 17, 0]
        - exchangeRightFrontLeftBack: [85, -86, 7, 3, 1, 16, 0]
        - exchangeRightFrontRightBack: [85, -86, 7, 3, 1, 17, 0]
        - exchangeLeftBackRightBack: [85, -86, 7, 3, 16, 17, 0]
        - exchange_sp_fl: [85, -86, 7, 3, 0, 5, 0]
        - exchange_sp_fr: [85, -86, 7, 3, 1, 5, 0]
        - exchange_sp_bl: [85, -86, 7, 3, 16, 5, 0]
        - exchange_sp_br: [85, -86, 7, 3, 17, 5, 0]
        """
        position_code1 = {
            TirePosition.FRONT_LEFT: 0,
            TirePosition.FRONT_RIGHT: 1,
            TirePosition.REAR_LEFT: 16,
            TirePosition.REAR_RIGHT: 17,
            TirePosition.SPARE_TIRE: 5,
        }[position1]

        position_code2 = {
            TirePosition.FRONT_LEFT: 0,
            TirePosition.FRONT_RIGHT: 1,
            TirePosition.REAR_LEFT: 16,
            TirePosition.REAR_RIGHT: 17,
            TirePosition.SPARE_TIRE: 5,
        }[position2]

        # Send the exact same command as the Android app
        # [85, -86, 7, 3, position_code1, position_code2, 0]
        # 85 = 0x55, -86 = 0xAA (signed byte), 7 = length, 3 = command code
        frame = bytes([0x55, 0xAA, 0x07, 0x03, position_code1, position_code2, 0x00])

        try:
            self.serial.write(frame)
            logger.info(
                f"Sent exchange tires command for {position1.name} and {position2.name} (Android app format)"
            )
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send exchange tires command: {e}")
            return False

    def reset_device(self) -> bool:
        """Reset the TPMS device

        This method sends the exact same command as the Android app for resetting the device.
        From FrameEncode3.java:
        - reset_dev: [85, -86, 6, 88, 85, -32]
        """
        # Send the exact same command as the Android app
        # [85, -86, 6, 88, 85, -32]
        # 85 = 0x55, -86 = 0xAA (signed byte), 6 = length, 88 = command code, 85 = data, -32 = 0xE0 (signed byte)
        frame = bytes([0x55, 0xAA, 0x06, 0x58, 0x55, 0xE0])

        try:
            self.serial.write(frame)
            logger.info("Sent reset device command (Android app format)")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send reset device command: {e}")
            return False

    def set_high_pressure_threshold(self, pressure: int) -> None:
        """Set high pressure threshold in kPa"""
        self.high_pressure_threshold = pressure

    def set_low_pressure_threshold(self, pressure: int) -> None:
        """Set low pressure threshold in kPa"""
        self.low_pressure_threshold = pressure

    def set_high_temp_threshold(self, temp: int) -> None:
        """Set high temperature threshold in Celsius"""
        self.high_temp_threshold = temp

    def set_spare_tire_enabled(self, enabled: bool) -> None:
        """Enable or disable spare tire monitoring"""
        self.spare_tire_enabled = enabled

    def get_tire_state(self, position: TirePosition) -> TireState:
        """Get the current state of a tire"""
        return self.tire_states[position]

    def get_all_tire_states(self) -> Dict[TirePosition, TireState]:
        """Get the current state of all tires"""
        return self.tire_states.copy()

    def register_tire_state_callback(
        self, callback: Callable[[TirePosition, TireState], None]
    ) -> None:
        """Register a callback for tire state updates"""
        self.on_tire_state_update = callback

    def register_pairing_callback(
        self, callback: Callable[[TirePosition, str], None]
    ) -> None:
        """Register a callback for pairing completion"""
        self.on_pairing_complete = callback

    def register_exchange_callback(
        self, callback: Callable[[TirePosition, TirePosition], None]
    ) -> None:
        """Register a callback for tire exchange completion"""
        self.on_exchange_complete = callback
