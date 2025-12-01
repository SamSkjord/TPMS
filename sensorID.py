#!/usr/bin/env python3
"""
Sensor ID Mapping Utility

This script helps identify which physical tyre position corresponds to which
sensor by monitoring pressure changes as you manually trigger each sensor.

To trigger a sensor: add/release air pressure or move the tyre.
"""

import time
from tpms_lib import TPMSDevice, TyrePosition

# Initialize TPMS
tpms = TPMSDevice()

# Track which positions have been identified
sensor_mapping = {}
last_pressures = {}


def on_tyre_update(position, state):
    """Callback to detect pressure changes"""
    global last_pressures

    prev = last_pressures.get(position, 0)
    curr = state.air_pressure

    # Detect significant pressure change (> 5 kPa)
    if prev > 0 and abs(curr - prev) > 5:
        print(f"\n*** Pressure change detected: {position.name} ***")
        print(f"    {prev} -> {curr} kPa (change: {curr - prev:+d} kPa)")

    last_pressures[position] = curr


def main():
    print("TPMS Sensor Identification Utility")
    print("=" * 40)

    # Find and connect to device
    ports = tpms.find_device()
    if not ports:
        print("No TPMS device found!")
        return

    print(f"Found device on: {ports[0]}")

    tpms.register_tyre_state_callback(on_tyre_update)

    if not tpms.connect(ports[0]):
        print("Failed to connect!")
        return

    print("\nConnected. Waiting for initial sensor data...")
    time.sleep(3)

    # Show current state
    print("\nCurrent sensor readings:")
    print("-" * 60)
    for pos, state in tpms.get_all_tyre_states().items():
        print(f"  {pos.name:<15} ID: {state.tyre_id or 'unknown':<10} "
              f"Pressure: {state.air_pressure:>3} kPa")
    print("-" * 60)

    print("\nInstructions:")
    print("1. Trigger each tyre sensor one at a time (add/release air)")
    print("2. Watch for 'Pressure change detected' messages")
    print("3. Note which reported position corresponds to which physical tyre")
    print("4. Press Ctrl+C when done\n")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nFinal sensor readings:")
        print("-" * 60)
        for pos, state in tpms.get_all_tyre_states().items():
            print(f"  {pos.name:<15} ID: {state.tyre_id or 'unknown':<10} "
                  f"Pressure: {state.air_pressure:>3} kPa")
        print("-" * 60)
    finally:
        tpms.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
