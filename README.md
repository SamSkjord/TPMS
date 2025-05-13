# TPMS

Tire Pressure Monitoring System Python Library

## Overview

The TPMS (Tire Pressure Monitoring System) Python library is designed to monitor tire pressure and temperature for vehicles equipped with generic (AliExpress) TPMS sensors. This library has been tested on the TY06 hardware type but should also work with TY05.

Pairing 'should' work but seems a bit iffy currently.
 
V2 of this library has breaking changes and is not directly compatible with V1.

## Installation

Install the TPMS library using pip:

```bash
pip install tpms
```

## Usage

### Basic Usage

```python
from tpms_lib import TPMSDevice, TirePosition

# Create TPMS device
tpms = TPMSDevice()

# Find available devices
available_ports = tpms.find_device()
if available_ports:
    # Connect to the first available port
    tpms.connect(available_ports[0])
    
    # Query sensor IDs
    tpms.query_sensor_ids()
    
    # Get tire states
    tire_states = tpms.get_all_tire_states()
    for position, state in tire_states.items():
        print(f"{position.name}: {state}")
    
    # Disconnect when done
    tpms.disconnect()
```

### Temperature and Pressure Conversion

```python
from tpms_lib import TPMSDevice, TirePosition

tpms = TPMSDevice()
tpms.connect()
tire_states = tpms.get_all_tire_states()

for position, state in tire_states.items():
    pressure_psi = state.air_pressure * 0.145038
    temp_f = (state.temperature * 9/5) + 32

    print(f"{position.name}:")
    print(f"  Pressure: {pressure_psi:.1f} PSI")
    print(f"  Temperature: {temp_f:.1f}°F")

    status = []
    if state.no_signal:
        status.append("NO SIGNAL")
    if state.is_leaking:
        status.append("LEAKAGE")
    if state.is_low_power:
        status.append("LOW BATTERY")
    
    print(f"  Status: {', '.join(status) if status else 'Normal'}")
    print()

tpms.disconnect()
```

### Continuous Monitoring with Callback

```python
from tpms_lib import TPMSDevice, TirePosition, TireState
import time

def on_tire_state_update(position, state):
    pressure_psi = state.air_pressure * 0.145038
    temp_f = (state.temperature * 9/5) + 32

    print(f"Update for {position.name}:")
    print(f"  Pressure: {pressure_psi:.1f} PSI")
    print(f"  Temperature: {temp_f:.1f}°F")

    status = []
    if state.no_signal:
        status.append("NO SIGNAL")
    if state.is_leaking:
        status.append("LEAKAGE")
    if state.is_low_power:
        status.append("LOW BATTERY")
    
    print(f"  Status: {', '.join(status) if status else 'Normal'}")
    print()

tpms = TPMSDevice()
tpms.register_tire_state_callback(on_tire_state_update)
tpms.connect()

try:
    print("Monitoring for 60 seconds...")
    time.sleep(60)
except KeyboardInterrupt:
    print("Monitoring stopped by user")
finally:
    tpms.disconnect()
```

## TL;DR

```bash
pip install tpms
```

```python
from tpms_lib import TPMSDevice

tpms = TPMSDevice()
tpms.connect()
tire_states = tpms.get_all_tire_states()

for position, state in tire_states.items():
    psi = state.air_pressure * 0.145038
    temp_f = (state.temperature * 9/5) + 32
    print(f"{position.name}: {psi:.1f} PSI, {temp_f:.1f}°F")

    if state.no_signal or state.is_leaking or state.is_low_power:
        print("  ALERT: Check tire!")

tpms.disconnect()
```

### Pairing Sensors (Semi-functional)

```python
from tpms_lib import TPMSDevice, TirePosition
import time

tpms = TPMSDevice()
tpms.connect()

def on_pairing_complete(position, tire_id):
    print(f"Successfully paired sensor with ID {tire_id} to {position.name}")

tpms.register_pairing_callback(on_pairing_complete)

print("Starting pairing mode for front left tire...")
print("Please activate the sensor (add/release air or move the tire)...")
tpms.pair_sensor(TirePosition.FRONT_LEFT)

time.sleep(30)

tpms.stop_pairing()
print("Pairing mode stopped")
tpms.disconnect()
```

### Exchanging Tire Positions

```python
from tpms_lib import TPMSDevice, TirePosition

tpms = TPMSDevice()
tpms.connect()

def on_exchange_complete(position1, position2):
    print(f"Successfully exchanged {position1.name} with {position2.name}")

tpms.register_exchange_callback(on_exchange_complete)

print("Exchanging front left and front right tires...")
tpms.exchange_tires(TirePosition.FRONT_LEFT, TirePosition.FRONT_RIGHT)
tpms.disconnect()
```

### Resetting the Device

```python
from tpms_lib import TPMSDevice

tpms = TPMSDevice()
tpms.connect()
tpms.reset_device()
print("Device has been reset. All paired sensors have been cleared.")
tpms.disconnect()
```

## Example Application

An example CLI application is included. After installation, run:

```bash
tpms-monitor
```

The script provides a menu-driven interface for:
- Showing current tire states
- Querying sensor IDs
- Pairing sensors
- Exchanging tire positions
- Resetting the device
- Toggling debug logging

## Debugging

To enable debug logging:

```python
import logging
logging.getLogger("tpms_lib").setLevel(logging.DEBUG)
```

## Protocol Details

The library implements the TPMS protocol based on reverse engineering of the Android app. The protocol uses a simple frame structure:

```
[0x55, 0xAA, length, command, data..., checksum]
```

Where:
- 0x55, 0xAA: Header bytes
- length: Length of the frame (command + data + checksum)
- command: Command code
- data: Command-specific data
- checksum: XOR of all previous bytes

## TODO

- Complete pairing functions
- Add helper functions for unit conversion
- Improve error handling and recovery
- Add more documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
