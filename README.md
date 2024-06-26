# TPMS

Tire Pressure Monitoring System Python Library

## Overview

The TPMS (Tire Pressure Monitoring System) Python library is designed to monitor tire pressure and temperature for vehicles equipped with generic (aliexpress) TPMS sensors. This library supports multiple hardware versions and provides options to configure the serial port, baud rate, temperature units, and pressure units.

## Features

- Supports TY06 and TY05 hardware versions (current versions appear to be all TY06)
- Configurable serial port and baud rate
- Temperature units: Celsius, Fahrenheit
- Pressure units: kPa, psi, bar
- Continuous monitoring of tire pressure and temperature
- Logging of tire status events

## Installation

Install the TPMS library using pip:

```bash
pip install tpms
```

Usage
You can run the TPMS library with default settings or customize the serial port, baud rate, temperature unit, pressure unit, and hardware version.

Default Usage
To run the TPMS library with default settings:

```bash
tpms
```
Custom Usage
To specify custom settings:

```bash
tpms --port COM3 --baudrate 19200 --temp_unit Fahrenheit --pressure_unit psi --hardware_version TY06
```
Example Code
Below is an example of how to use the TPMS library in a Python script:

```python
from tpms import TPMS

def main():
    # Initialize TPMS with the desired settings
    tpms = TPMS(
        port='/dev/ttyUSB0',       # Replace with your serial port
        baudrate=19200,
        temp_unit='Celsius',       # Options: 'Celsius', 'Fahrenheit'
        pressure_unit='kPa',       # Options: 'kPa', 'psi', 'bar'
        hardware_version='TY06'    # Options: 'TY06', 'TY05'
    )

    # Connect to the TPMS device
    tpms.connect()

    # Start monitoring the tire pressure and temperature
    tpms.heartbeat()  # Start sending heartbeat signals

    # Infinite loop to read data from the TPMS device
    while True:
        try:
            raw_data = tpms.ser.read(256)  # Read up to 256 bytes at a time
            if raw_data:
                tpms.incoming_buffer.extend(raw_data)
                frames, tpms.incoming_buffer = tpms.protocol_frame_filter(tpms.incoming_buffer)
                for frame in frames:
                    event = tpms.decode_frame(frame)
                    if event:
                        tpms.on_event(event)
        except (serial.SerialException, serial.SerialTimeoutException):
            logger.error("Serial port unavailable, reconnecting...")
            tpms.ser.close()
            tpms.connect()
        except KeyboardInterrupt:
            if tpms.ser is not None:
                tpms.ser.close()
                logger.info("Port closed!")
            logger.info("KeyboardInterrupt detected, exiting...")
            break

if __name__ == "__main__":
    main()
```

Functions
Initialization
```python
tpms = TPMS(
    port='/dev/ttyUSB0',       # Replace with your serial port
    baudrate=19200,
    temp_unit='Celsius',       # Options: 'Celsius', 'Fahrenheit'
    pressure_unit='kPa',       # Options: 'kPa', 'psi', 'bar'
    hardware_version='TY06'    # Options: 'TY06', 'TY05'
)
```

Connect
Connect to the TPMS device:
```python
tpms.connect()
```

Heartbeat
Send periodic heartbeat signals:
```python
tpms.heartbeat()
```

Pairing Sensors
Pair sensors to specific tire positions:
```python
tpms.pair_back_left()
tpms.pair_back_right()
tpms.pair_front_left()
tpms.pair_front_right()
tpms.pair_spare()
```

Stop Pairing
Stop pairing process:

```python
tpms.stop_pairing()
```

Initiate Tire Exchange
Initiate a tire exchange between two tires:

```python
tpms.initiate_tire_exchange(tire1, tire2)
```

License
This project is licensed under the MIT License - see the LICENSE file for details.