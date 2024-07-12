# TPMS

Tyre Pressure Monitoring System Python Library

## Overview

The TPMS (Tyre Pressure Monitoring System) Python library is designed to monitor tyre pressure and temperature for vehicles equipped with generic (aliexpress) TPMS sensors. This library has only been tested on the TY06 hardware type but should also work with TY05.

Despite looking obvious in thethe dongle casing in no way indicates what hardware is inside.
![TPMS Dongle Versions.](/TPMS_type.jpg)


## Features

- Supports TY06 and should also work with TY05 hardware versions (No idea for N variants)
- Serial port can be auto-detected or specified
- Temperature units: Celsius, Fahrenheit
- Pressure units: kPa, psi, bar
- Continuous monitoring of tyre pressure and temperature or oneshot function

## Installation

Install the TPMS library using pip:

```bash
pip install tpms
```
## Usage

[TL;DR](https://github.com/SamSkjord/TPMS/tree/main?tab=readme-ov-file#tldr)

You can run the TPMS library with default settings or customize the serial port, baud rate, temperature unit, pressure unit
There are two methods of retreiving data, 'oneshot' which returns a dict containing data for each tyre and 'stream_updates' which returns data at a configurable update interval (defaulted to 0.5 seconds)

## Example Code

### oneshot
```bash
from tpms import TPMS
import time


def data_received(data):
    """
    Callback function to display data received from the TPMS device.
    This function could also log data to a file or database, send notifications, etc.
    """
    print("Data received:")
    for tyre, info in data.items():
        print(f"Tyre Position: {tyre}")
        print(f"  Pressure: {info['Pressure']} psi")  # Assuming psi for example
        print(
            f"  Temperature: {info['Temperature']}° Fahrenheit"
        )  # Assuming Fahrenheit for example
        status_messages = []
        if info["Status"]["low_power"]:
            status_messages.append("low power")
        if info["Status"]["leakage"]:
            status_messages.append("leakage")
        if info["Status"]["no_signal"]:
            status_messages.append("no signal")
        print(
            f"  Status: {', '.join(status_messages) if status_messages else 'Normal'}"
        )
        print("")


def main():
    """
    Main function to initialize the TPMS system and perform a oneshot data retrieval.
    """
    # Initialize the TPMS system with full configuration
    tpms_system = TPMS(
        # port="/dev/ttyUSB0",  # Adjust this to the correct serial port
        baudrate=19200,  # Set baud rate according to your device specifications
        temp_unit="Fahrenheit",  # Convert temperature readings to Fahrenheit
        pressure_unit="psi",  # Convert pressure readings to psi
        debug=True,  # Enable debug mode for detailed logging
    )

    # Perform a single data retrieval using the oneshot method
    print("Performing a single data retrieval...")
    single_data = tpms_system.oneshot()
    if single_data:
        print("Oneshot Data Retrieval Results:")
        data_received(single_data)
    else:
        print("No data retrieved. Check device connection and settings.")


if __name__ == "__main__":
    main()
```

### stream
```python
import time
from tpms import TPMS


def update_dashboard(data):
    """
    Update the dashboard or console with the latest tyre data.

    Args:
    data (list): A list of dictionaries, each containing tyre data.
    """
    for tyre_data in data:
        if tyre_data:
            print(f"Update for {tyre_data['Position']}:")
            print(f"  Pressure: {tyre_data['Pressure']} kPa")
            print(f"  Temperature: {tyre_data['Temperature']} Celsius")
            print(
                f"  Status: {', '.join([k for k, v in tyre_data['Status'].items() if v]) or 'Normal'}"
            )
            print("")


def main():
    # Initialize the TPMS system with a callback function
    tpms_system = TPMS(
        # port="/dev/ttyUSB0",  # Adjust to your serial port
        baudrate=19200,
        temp_unit="Celsius",
        pressure_unit="kPa",
        callback=update_dashboard,
        debug=False,
        update_interval=0.3,
    )

    # Start the continuous streaming of updates
    try:
        tpms_system.stream_updates()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Monitoring stopped or failed to start.")


if __name__ == "__main__":
    main()

```


# TLDR
```bash
pip install TPMS
```

```python
from tpms import TPMS
import time

tpms_system = TPMS(
    # port="/dev/ttyUSB0",  # Adjust this to the correct serial port (optional)
    temp_unit="Fahrenheit",  # Convert temperature readings to Fahrenheit
    pressure_unit="psi",  # Convert pressure readings to psi
    debug=False,  # Enable debug mode for detailed logging
)

data = tpms_system.oneshot()

for tyre, info in data.items():
    print(tyre, info)
```

## TODO

Tyre exchange & sensor pairing functions

Issues or pull requests gladly recieved

## License
This project is licensed under the MIT License - see the LICENSE file for details.
