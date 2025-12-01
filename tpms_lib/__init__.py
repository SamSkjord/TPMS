#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TPMS Library - Python implementation of TPMS (Tyre Pressure Monitoring System)

This library provides functionality to interface with TPMS sensors via USB,
allowing for reading tyre data, pairing sensors, and managing tyre positions.
"""

import logging
import time
import threading
from enum import IntEnum
from typing import Dict, List, Optional, Tuple, Callable
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

# Position code mappings used in protocol
# Pairing/tyre state use: FL=0, FR=1, RL=16, RR=17, Spare=5
# Query ID responses use: FL=1, FR=2, RL=3, RR=4, Spare=5
POSITION_CODE_TO_TYRE = {
    0: "FRONT_LEFT",
    1: "FRONT_RIGHT",
    16: "REAR_LEFT",
    17: "REAR_RIGHT",
    5: "SPARE_TYRE",
}

QUERY_ID_CODE_TO_TYRE = {
    1: "FRONT_LEFT",
    2: "FRONT_RIGHT",
    3: "REAR_LEFT",
    4: "REAR_RIGHT",
    5: "SPARE_TYRE",
}


class TyrePosition(IntEnum):
    """Enum representing tyre positions."""

    REAR_LEFT = 0
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    REAR_RIGHT = 3
    SPARE_TYRE = 5
    SPARE_TIRE = 5  # US spelling alias


class Command(IntEnum):
    """Command codes used in the TPMS protocol"""

    HEARTBEAT = 6
    EXCHANGE_TYRES = 7
    TYRE_STATE = 8
    QUERY_ID = 9
    RESET = 10


class TyreState:
    """Class representing the state of a tyre."""

    def __init__(self):
        self.tyre_id = ""
        self.air_pressure = 0  # in kPa
        self.temperature = 0  # in Celsius
        self.is_leaking = False
        self.is_low_power = False
        self.no_signal = False
        self.error = ""
        self.last_update = 0

    @property
    def tire_id(self):
        """US spelling alias for tyre_id"""
        return self.tyre_id

    @tire_id.setter
    def tire_id(self, value):
        """US spelling alias for tyre_id"""
        self.tyre_id = value

    def __str__(self):
        return (
            f"TyreState(ID: {self.tyre_id}, Pressure: {self.air_pressure} kPa, "
            f"Temp: {self.temperature}°C, Leaking: {self.is_leaking}, "
            f"Low Power: {self.is_low_power}, No Signal: {self.no_signal})"
        )


class TPMSProtocol:
    """Handles the low-level protocol for TPMS communication

    Frame structure: [0x55, 0xAA, length, command, data..., checksum]

    The length byte represents the TOTAL frame size (including headers and checksum).
    The length also serves as a message type discriminator:
      - Length 6: Command messages (pairing, heartbeat, stop, query, reset)
      - Length 7: Exchange tyre positions
      - Length 8: Tyre state updates
      - Length 9: Query ID responses

    Checksum is XOR of bytes[0] through bytes[length-2].
    """

    HEADER_1 = 0x55
    HEADER_2 = 0xAA

    def __init__(self):
        self.buffer = bytearray()

    @staticmethod
    def calc_checksum(frame: bytearray) -> int:
        """Calculate XOR checksum for a frame.

        From PackBufferFrameEn3.calcCC:
        XOR all bytes from index 0 to (length - 2), where length = frame[2].
        """
        length = frame[2]
        checksum = frame[0]
        for i in range(1, length - 1):
            checksum ^= frame[i]
        return checksum & 0xFF

    def create_frame(self, command: int, data: List[int]) -> bytes:
        """Create a frame to send to the TPMS device.

        The length byte is the total frame size: header(2) + length(1) + cmd(1) + data + checksum(1)
        """
        # Total frame length = 2 (header) + 1 (length) + 1 (command) + len(data) + 1 (checksum)
        length = 5 + len(data)

        # Create frame with placeholder for checksum
        frame = bytearray([self.HEADER_1, self.HEADER_2, length, command] + data + [0x00])

        # Calculate and set checksum
        frame[-1] = self.calc_checksum(frame)

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

        # Tyre states
        self.tyre_states = {
            TyrePosition.FRONT_LEFT: TyreState(),
            TyrePosition.FRONT_RIGHT: TyreState(),
            TyrePosition.REAR_LEFT: TyreState(),
            TyrePosition.REAR_RIGHT: TyreState(),
            TyrePosition.SPARE_TYRE: TyreState(),
        }

        # Callbacks
        self.on_tyre_state_update = None
        self.on_pairing_complete = None
        self.on_exchange_complete = None

        # Thread lock for tyre_states access
        self._lock = threading.Lock()

        # Settings
        self.high_pressure_threshold = 310  # kPa (default)
        self.low_pressure_threshold = 180  # kPa (default)
        self.high_temp_threshold = 75  # Celsius (default)
        self.spare_tyre_enabled = False

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

    def _invoke_callback(self, callback, *args):
        """Safely invoke a callback, catching any exceptions to prevent crashing the read thread."""
        if callback is None:
            return
        try:
            callback(*args)
        except Exception as e:
            logger.error(f"Exception in callback: {e}")

    def _read_loop(self):
        """Background thread for reading data from the device."""
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
        """Handle received commands.

        Note: Due to protocol structure, 'command' here is frame[3], which is:
        - For tyre state (length=8): position code (0, 1, 16, 17, 5)
        - For pairing success (length=6): sub-command 0x18 (24)
        - For query ID (length=9): position code (1-5)
        - For exchange (length=7): sub-command 0x30 (48)
        """
        # Pairing success: frame[3] = 0x18 (24)
        if command == 24:  # 0x18 - Pairing success
            logger.info(f"Pairing success response: data={[f'{b:02X}' for b in data]}")
            # data[0] is the position code
            if len(data) >= 1:
                self._handle_pairing_complete([24] + data)
            return

        # Tyre state (length=8): frame[3] = position code, data = [pressure, temp, flags]
        # Query ID (length=9): frame[3] = position code (1-5), data = [id0, id1, id2, id3]
        #
        # Disambiguation: Query ID uses position codes 1-5, tyre state uses 0,1,16,17,5
        # Position codes 2,3,4 are ONLY used by Query ID responses
        # Position code 0 is ONLY used by tyre state (Front Left)
        # Position codes 16,17 are ONLY used by tyre state (Rear Left/Right)
        if command in [2, 3, 4]:  # Query ID only position codes
            if len(data) >= 4:
                self._handle_query_id([command] + data)
            return

        if command in [0, 16, 17]:  # Tyre state only position codes
            if len(data) >= 2:
                self._handle_direct_tyre_update(command, data)
            return

        # Ambiguous: command 1 (FL tyre state OR FR query ID) and 5 (spare tyre OR spare query ID)
        # Use data length to disambiguate: query ID has exactly 4 ID bytes
        if command in [1, 5]:
            if len(data) == 4:
                # Likely query ID response (4 bytes of ID)
                self._handle_query_id([command] + data)
            elif len(data) >= 2:
                # Likely tyre state (pressure, temp, optional flags)
                self._handle_direct_tyre_update(command, data)
            return

        # Exchange response: frame[3] = 0x30 (48) indicates success
        if command == 48:  # 0x30
            if len(data) >= 2:
                self._handle_exchange_tyres(data)
            return

        # Legacy handling for other command patterns
        if command == Command.TYRE_STATE:
            self._handle_tyre_state(data)
        elif command == Command.QUERY_ID:
            self._handle_query_id(data)
        elif command == Command.EXCHANGE_TYRES:
            self._handle_exchange_tyres(data)
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
            0: TyrePosition.FRONT_LEFT,  # Left Front
            1: TyrePosition.FRONT_RIGHT,  # Right Front
            16: TyrePosition.REAR_LEFT,  # Left Rear
            17: TyrePosition.REAR_RIGHT,  # Right Rear
            5: TyrePosition.SPARE_TYRE,  # Spare Tyre
        }

        if position_code in position_map:
            position = position_map[position_code]

            # Get the tyre ID (if available)
            tyre_id = self.tyre_states[position].tyre_id
            if not tyre_id:
                # If no ID is available, generate one based on position
                tyre_id = f"SENSOR_{position.name}"
                self.tyre_states[position].tyre_id = tyre_id

            logger.info(f"Pairing complete for {position.name}: ID {tyre_id}")

            # Call pairing callback if registered
            self._invoke_callback(self.on_pairing_complete, position, tyre_id)

            # Query sensor IDs to get the actual ID
            self.query_sensor_ids()
        else:
            logger.warning(
                f"Unknown tyre position code in pairing response: {position_code}"
            )

    def _handle_direct_tyre_update(self, position_code: int, data: List[int]):
        """Handle direct tyre state updates where the command is the position code."""
        if len(data) < 2:
            logger.warning(
                f"Invalid direct tyre update data for position {position_code}"
            )
            return

        # Map position codes to enum values
        position_map = {
            0: TyrePosition.FRONT_LEFT,
            1: TyrePosition.FRONT_RIGHT,
            16: TyrePosition.REAR_LEFT,
            17: TyrePosition.REAR_RIGHT,
            5: TyrePosition.SPARE_TYRE,
        }

        if position_code not in position_map:
            logger.warning(f"Unknown tyre position code: {position_code}")
            return

        position = position_map[position_code]
        tyre_state = self.tyre_states[position]

        # Store previous pressure for change detection
        previous_pressure = tyre_state.air_pressure

        # Update tyre state based on the data format observed in logs
        # Format appears to be [pressure, temperature, flags]
        pressure_raw = data[0]
        temp_raw = data[1]
        flags = 0
        if len(data) > 2:
            flags = data[2]

        # Convert raw values to actual values
        tyre_state.air_pressure = int(pressure_raw * 3.44)  # Convert to kPa
        tyre_state.temperature = temp_raw - 50  # Convert to Celsius

        # Parse flags if present
        tyre_state.no_signal = bool(flags & 0x20)
        tyre_state.is_leaking = bool(flags & 0x08)
        tyre_state.is_low_power = bool(flags & 0x04)

        # Update timestamp
        tyre_state.last_update = time.time()

        logger.debug(f"Updated tyre state for {position.name}: {tyre_state}")

        # Call callback if registered
        self._invoke_callback(self.on_tyre_state_update, position, tyre_state)

        # Check for significant pressure change (which might indicate pairing)
        # If pressure was 0 and now it's not, or if it changed by more than 20 kPa
        if (previous_pressure == 0 and tyre_state.air_pressure > 0) or abs(
            tyre_state.air_pressure - previous_pressure
        ) > 20:
            logger.info(
                f"Significant pressure change detected for {position.name}: {previous_pressure} -> {tyre_state.air_pressure} kPa"
            )

            # Get or generate tyre ID
            tyre_id = tyre_state.tyre_id
            if not tyre_id:
                # If no ID is available, generate one based on position
                tyre_id = f"SENSOR_{position.name}"
                tyre_state.tyre_id = tyre_id

            logger.info(f"Auto-pairing detected for {position.name}: ID {tyre_id}")

            # Call pairing callback if registered
            self._invoke_callback(self.on_pairing_complete, position, tyre_id)

    def _handle_tyre_state(self, data: List[int]):
        """Handle tyre state update"""
        if len(data) < 4:
            logger.warning("Invalid tyre state data")
            return

        # Parse tyre position
        position_code = data[0]
        position = None

        if position_code == 0:
            position = TyrePosition.FRONT_LEFT
        elif position_code == 1:
            position = TyrePosition.FRONT_RIGHT
        elif position_code == 16:
            position = TyrePosition.REAR_LEFT
        elif position_code == 17:
            position = TyrePosition.REAR_RIGHT
        elif position_code == 5:
            position = TyrePosition.SPARE_TYRE
        else:
            logger.warning(f"Unknown tyre position code: {position_code}")
            return

        # Get tyre state
        tyre_state = self.tyre_states[position]

        # Update tyre state
        tyre_state.air_pressure = int(data[1] * 3.44)  # Convert to kPa
        tyre_state.temperature = data[2] - 50  # Convert to Celsius

        # Parse flags
        flags = data[3]
        tyre_state.no_signal = bool(flags & 0x20)
        tyre_state.is_leaking = bool(flags & 0x08)
        tyre_state.is_low_power = bool(flags & 0x04)

        # Update timestamp
        tyre_state.last_update = time.time()

        # Call callback if registered
        self._invoke_callback(self.on_tyre_state_update, position, tyre_state)

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
        - 5 = Spare Tyre (SPARE_TYRE)
        """
        # Log the raw data for debugging
        logger.info(f"Received query ID response: {[f'{b:02X}' for b in data]}")

        if len(data) < 5:
            logger.warning(f"Query ID response too short: {data}")
            return

        # Parse tyre position according to the Android app
        position_code = data[0]
        position = None

        # Map position codes to enum values (from FrameDecode3.java)
        position_map = {
            1: TyrePosition.FRONT_LEFT,  # Left Front
            2: TyrePosition.FRONT_RIGHT,  # Right Front
            3: TyrePosition.REAR_LEFT,  # Left Rear
            4: TyrePosition.REAR_RIGHT,  # Right Rear
            5: TyrePosition.SPARE_TYRE,  # Spare Tyre
        }

        if position_code in position_map:
            position = position_map[position_code]

            # Parse ID (from FrameDecode3.java)
            # paire2.mID = Util.byteToUpperString(frame[4]) + Util.byteToUpperString(frame[5]) +
            #              Util.byteToUpperString(frame[6]) + Util.byteToUpperString(frame[7]);
            # In our case, data starts at frame[4], so we need data[0:4]
            tyre_id = "".join(f"{b:02X}" for b in data[1:5])

            # Update tyre state
            self.tyre_states[position].tyre_id = tyre_id

            logger.info(f"Tyre ID for position {position.name}: {tyre_id}")

            # Call pairing callback if registered
            self._invoke_callback(self.on_pairing_complete, position, tyre_id)
        else:
            logger.warning(f"Unknown tyre position code: {position_code}")

    def _handle_exchange_tyres(self, data: List[int]):
        """Handle tyre exchange response"""
        if len(data) < 2:
            logger.warning("Invalid tyre exchange data")
            return

        # Get positions
        pos1 = data[0]
        pos2 = data[1]

        # Map position codes to enum values
        position_map = {
            0: TyrePosition.FRONT_LEFT,
            1: TyrePosition.FRONT_RIGHT,
            16: TyrePosition.REAR_LEFT,
            17: TyrePosition.REAR_RIGHT,
            5: TyrePosition.SPARE_TYRE,
        }

        # Get position names
        if pos1 in position_map and pos2 in position_map:
            position1 = position_map[pos1]
            position2 = position_map[pos2]

            logger.info(f"Tyres exchanged: {position1.name} and {position2.name}")

            # Call callback if registered
            self._invoke_callback(self.on_exchange_complete, position1, position2)

    def _send_command(self, command: int, data: List[int]) -> bool:
        """Send a command to the TPMS device using create_frame."""
        if not self.serial or not self.serial.is_open:
            logger.error("Not connected to device")
            return False

        frame = self.protocol.create_frame(command, data)
        try:
            hex_frame = " ".join(f"{b:02X}" for b in frame)
            logger.debug(f"Sending frame: {hex_frame}")
            self.serial.write(frame)
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def _send_raw_frame(self, frame_without_checksum: List[int]) -> bool:
        """Send a raw frame, calculating and appending the checksum.

        This mimics the Java PackBufferFrameEn.send() which calculates
        checksum and sets frame[length-1] before writing.

        Args:
            frame_without_checksum: Frame bytes with placeholder (usually 0) for checksum
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Not connected to device")
            return False

        frame = bytearray(frame_without_checksum)
        frame[-1] = TPMSProtocol.calc_checksum(frame)

        try:
            hex_frame = " ".join(f"{b:02X}" for b in frame)
            logger.debug(f"Sending raw frame: {hex_frame}")
            self.serial.write(bytes(frame))
            time.sleep(0.06)  # 60ms delay between commands (from Java)
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to send raw frame: {e}")
            return False

    def _send_heartbeat(self) -> bool:
        """Send heartbeat command.

        From FrameEncode3.SendHeartbeat: [85, -86, 6, 25, 0, -32]
        Note: The -32 (0xE0) is NOT the checksum - it's data. Checksum is calculated.
        """
        # Frame: [0x55, 0xAA, 0x06, 0x19, 0x00, checksum]
        return self._send_raw_frame([0x55, 0xAA, 0x06, 0x19, 0x00, 0x00])

    def query_sensor_ids(self) -> bool:
        """Query all sensor IDs.

        From FrameEncode3.querySensorID: [85, -86, 6, 7, 0, 0]
        Frame: [0x55, 0xAA, 0x06, 0x07, 0x00, checksum]
        """
        logger.info("Sending query sensor IDs command")
        return self._send_raw_frame([0x55, 0xAA, 0x06, 0x07, 0x00, 0x00])

    def pair_sensor(self, position: TyrePosition) -> bool:
        """Start pairing a sensor for the specified position.

        From FrameEncode3.java:
        - paireFrontLeft: [85, -86, 6, 1, 0, 0]
        - paireFrontRight: [85, -86, 6, 1, 1, 0]
        - paireBackLeft: [85, -86, 6, 1, 16, 0]
        - paireBackRight: [85, -86, 6, 1, 17, 0]
        - paireSpTired: [85, -86, 6, 1, 5, 0]

        Frame: [0x55, 0xAA, 0x06, 0x01, position_code, checksum]
        Position codes: FL=0, FR=1, RL=16, RR=17, Spare=5
        """
        position_code = {
            TyrePosition.FRONT_LEFT: 0,
            TyrePosition.FRONT_RIGHT: 1,
            TyrePosition.REAR_LEFT: 16,
            TyrePosition.REAR_RIGHT: 17,
            TyrePosition.SPARE_TYRE: 5,
        }[position]

        logger.info(f"Sending pair sensor command for {position.name}")
        return self._send_raw_frame([0x55, 0xAA, 0x06, 0x01, position_code, 0x00])

    def stop_pairing(self) -> bool:
        """Stop the pairing process.

        From FrameEncode3.stopPaire: [85, -86, 6, 6, 0, 0]
        Frame: [0x55, 0xAA, 0x06, 0x06, 0x00, checksum]
        """
        logger.info("Sending stop pairing command")
        return self._send_raw_frame([0x55, 0xAA, 0x06, 0x06, 0x00, 0x00])

    def exchange_tyres(self, position1: TyrePosition, position2: TyrePosition) -> bool:
        """Exchange two tyre positions.

        From FrameEncode3.java exchange methods.
        Frame: [0x55, 0xAA, 0x07, 0x03, pos1, pos2, checksum]
        Position codes: FL=0, FR=1, RL=16, RR=17, Spare=5
        """
        position_code1 = {
            TyrePosition.FRONT_LEFT: 0,
            TyrePosition.FRONT_RIGHT: 1,
            TyrePosition.REAR_LEFT: 16,
            TyrePosition.REAR_RIGHT: 17,
            TyrePosition.SPARE_TYRE: 5,
        }[position1]

        position_code2 = {
            TyrePosition.FRONT_LEFT: 0,
            TyrePosition.FRONT_RIGHT: 1,
            TyrePosition.REAR_LEFT: 16,
            TyrePosition.REAR_RIGHT: 17,
            TyrePosition.SPARE_TYRE: 5,
        }[position2]

        logger.info(f"Sending exchange tyres command: {position1.name} <-> {position2.name}")
        return self._send_raw_frame([0x55, 0xAA, 0x07, 0x03, position_code1, position_code2, 0x00])

    def reset_device(self) -> bool:
        """Reset the TPMS device.

        From FrameEncode3.reset_dev: [85, -86, 6, 88, 85, -32]
        Frame: [0x55, 0xAA, 0x06, 0x58, 0x55, checksum]
        Note: 0x58=88 is command, 0x55=85 is data
        """
        logger.info("Sending reset device command")
        return self._send_raw_frame([0x55, 0xAA, 0x06, 0x58, 0x55, 0x00])

    def set_high_pressure_threshold(self, pressure: int) -> None:
        """Set high pressure threshold in kPa"""
        self.high_pressure_threshold = pressure

    def set_low_pressure_threshold(self, pressure: int) -> None:
        """Set low pressure threshold in kPa"""
        self.low_pressure_threshold = pressure

    def set_high_temp_threshold(self, temp: int) -> None:
        """Set high temperature threshold in Celsius"""
        self.high_temp_threshold = temp

    def set_spare_tyre_enabled(self, enabled: bool) -> None:
        """Enable or disable spare tyre monitoring"""
        self.spare_tyre_enabled = enabled

    def get_tyre_state(self, position: TyrePosition) -> TyreState:
        """Get the current state of a tyre."""
        with self._lock:
            return self.tyre_states[position]

    def get_all_tyre_states(self) -> Dict[TyrePosition, TyreState]:
        """Get the current state of all tyres."""
        with self._lock:
            return self.tyre_states.copy()

    def register_tyre_state_callback(
        self, callback: Callable[[TyrePosition, TyreState], None]
    ) -> None:
        """Register a callback for tyre state updates"""
        self.on_tyre_state_update = callback

    def register_pairing_callback(
        self, callback: Callable[[TyrePosition, str], None]
    ) -> None:
        """Register a callback for pairing completion"""
        self.on_pairing_complete = callback

    def register_exchange_callback(
        self, callback: Callable[[TyrePosition, TyrePosition], None]
    ) -> None:
        """Register a callback for tyre exchange completion"""
        self.on_exchange_complete = callback

    # US spelling aliases for backwards compatibility
    def get_tire_state(self, position: TyrePosition) -> TyreState:
        """US spelling alias for get_tyre_state"""
        return self.get_tyre_state(position)

    def get_all_tire_states(self) -> Dict[TyrePosition, TyreState]:
        """US spelling alias for get_all_tyre_states"""
        return self.get_all_tyre_states()

    def register_tire_state_callback(
        self, callback: Callable[[TyrePosition, TyreState], None]
    ) -> None:
        """US spelling alias for register_tyre_state_callback"""
        self.register_tyre_state_callback(callback)

    def exchange_tires(self, position1: TyrePosition, position2: TyrePosition) -> bool:
        """US spelling alias for exchange_tyres"""
        return self.exchange_tyres(position1, position2)

    def set_spare_tire_enabled(self, enabled: bool) -> None:
        """US spelling alias for set_spare_tyre_enabled"""
        self.set_spare_tyre_enabled(enabled)

    @property
    def spare_tire_enabled(self) -> bool:
        """US spelling alias for spare_tyre_enabled"""
        return self.spare_tyre_enabled

    @spare_tire_enabled.setter
    def spare_tire_enabled(self, value: bool) -> None:
        """US spelling alias for spare_tyre_enabled"""
        self.spare_tyre_enabled = value


# US spelling aliases for class names
TirePosition = TyrePosition
TireState = TyreState
