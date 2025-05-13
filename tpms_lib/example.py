#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TPMS Library Example

This example demonstrates how to use the TPMS library to interface with TPMS sensors.
"""

import time
import logging
from tpms_lib import TPMSDevice, TirePosition, TireState

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("tpms_example")

# Enable debug logging for the TPMS library
logging.getLogger("tpms_python.tpms_lib").setLevel(logging.DEBUG)


def main():
    """Main function demonstrating TPMS library usage"""
    # Create TPMS device
    tpms = TPMSDevice()

    # Register callbacks
    tpms.register_tire_state_callback(on_tire_state_update)
    tpms.register_pairing_callback(on_pairing_complete)
    tpms.register_exchange_callback(on_exchange_complete)

    # Set thresholds
    tpms.set_high_pressure_threshold(310)  # 310 kPa (45 PSI)
    tpms.set_low_pressure_threshold(180)  # 180 kPa (26 PSI)
    tpms.set_high_temp_threshold(75)  # 75°C (167°F)

    # Enable spare tire monitoring
    tpms.set_spare_tire_enabled(True)

    # Find available devices
    available_ports = tpms.find_device()
    if not available_ports:
        logger.error("No serial ports found")
        return

    logger.info(f"Available ports: {available_ports}")

    # Let the user select which port to use
    if len(available_ports) > 1:
        print("\nMultiple ports found. Please select which one to use:")
        for i, port in enumerate(available_ports):
            print(f"{i+1}. {port}")

        while True:
            try:
                choice = input("Enter port number (or press Enter for first port): ")
                if choice == "":
                    selected_port = available_ports[0]
                    break
                choice = int(choice)
                if 1 <= choice <= len(available_ports):
                    selected_port = available_ports[choice - 1]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(available_ports)}")
            except ValueError:
                print("Please enter a valid number")
    else:
        selected_port = available_ports[0]

    logger.info(f"Selected port: {selected_port}")

    # Connect to the selected port
    if tpms.connect(selected_port):
        try:
            # Query sensor IDs
            logger.info("Querying sensor IDs...")
            tpms.query_sensor_ids()

            # Wait for data and IDs to be received
            logger.info("Waiting for tire data and IDs...")
            time.sleep(2)  # Give some time for IDs to be received

            # Example menu loop
            while True:
                print("\nTPMS Example Menu:")
                print("1. Show current tire states")
                print("2. Query sensor IDs")
                print("3. Start pairing mode for a tire")
                print("4. Exchange tire positions")
                print("5. Reset device")
                print("6. Toggle debug logging")
                print("7. Exit")

                choice = input("Enter your choice (1-7): ")

                if choice == "1":
                    show_tire_states(tpms)
                elif choice == "2":
                    query_sensor_ids(tpms)
                elif choice == "3":
                    start_pairing(tpms)
                elif choice == "4":
                    exchange_tires(tpms)
                elif choice == "5":
                    logger.info("Resetting device...")
                    tpms.reset_device()
                elif choice == "6":
                    toggle_debug_logging(tpms)
                elif choice == "7":
                    logger.info("Exiting...")
                    break
                else:
                    print("Invalid choice. Please try again.")

                # Small delay to prevent CPU hogging
                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            # Disconnect
            tpms.disconnect()
    else:
        logger.error(f"Failed to connect to {selected_port}")


def on_tire_state_update(position: TirePosition, state: TireState):
    """Callback for tire state updates"""
    logger.info(f"Tire {position.name} updated: {state}")

    # Check for alerts
    alerts = []
    if state.no_signal:
        alerts.append("NO SIGNAL")
    if state.is_leaking:
        alerts.append("LEAKAGE")
    if state.is_low_power:
        alerts.append("LOW BATTERY")

    if alerts:
        logger.warning(f"ALERTS for {position.name}: {', '.join(alerts)}")


def on_pairing_complete(position: TirePosition, tire_id: str):
    """Callback for pairing completion"""
    logger.info(f"Pairing complete for {position.name}: ID {tire_id}")


def on_exchange_complete(position1: TirePosition, position2: TirePosition):
    """Callback for tire exchange completion"""
    logger.info(f"Tire exchange complete: {position1.name} <-> {position2.name}")


def toggle_debug_logging(tpms: TPMSDevice):
    """Toggle debug logging for the TPMS library"""
    tpms_logger = logging.getLogger("tpms_python.tpms_lib")
    if tpms_logger.level == logging.DEBUG:
        tpms_logger.setLevel(logging.INFO)
        print("Debug logging disabled")
    else:
        tpms_logger.setLevel(logging.DEBUG)
        print("Debug logging enabled - you will see detailed communication logs")

    # Also log the current state of all tire positions
    print("\nCurrent Tire States:")
    for position, state in tpms.get_all_tire_states().items():
        print(f"{position.name}: {state}")


def query_sensor_ids(tpms: TPMSDevice):
    """Query and display sensor IDs"""
    print("\nQuerying sensor IDs...")

    # Send the query command
    tpms.query_sensor_ids()

    # Wait for responses
    print("Waiting for responses (5 seconds)...")
    time.sleep(5)

    # Display the IDs
    print("\nSensor IDs:")
    print("-" * 50)
    print(f"{'Position':<15} {'ID':<20}")
    print("-" * 50)

    for position, state in tpms.get_all_tire_states().items():
        # Skip spare tire if disabled
        if position == TirePosition.SPARE_TIRE and not tpms.spare_tire_enabled:
            continue

        print(f"{position.name:<15} {state.tire_id:<20}")

    print("-" * 50)
    print("Note: Empty IDs indicate sensors that haven't been paired or detected.")
    print("Use the pairing function to pair sensors with specific positions.")


def show_tire_states(tpms: TPMSDevice):
    """Display current tire states"""
    print("\nCurrent Tire States:")
    print("-" * 80)
    print(
        f"{'Position':<15} {'ID':<10} {'Pressure (kPa)':<15} {'Temp (°C)':<10} {'Status':<20}"
    )
    print("-" * 80)

    for position, state in tpms.get_all_tire_states().items():
        # Skip spare tire if disabled
        if position == TirePosition.SPARE_TIRE and not tpms.spare_tire_enabled:
            continue

        # Determine status
        status = []
        if state.no_signal:
            status.append("NO SIGNAL")
        elif state.is_leaking:
            status.append("LEAKAGE")
        elif state.is_low_power:
            status.append("LOW BATTERY")
        elif state.air_pressure > tpms.high_pressure_threshold:
            status.append("HIGH PRESSURE")
        elif state.air_pressure < tpms.low_pressure_threshold:
            status.append("LOW PRESSURE")
        elif state.temperature > tpms.high_temp_threshold:
            status.append("HIGH TEMP")
        else:
            status.append("OK")

        print(
            f"{position.name:<15} {state.tire_id:<10} {state.air_pressure:<15} "
            f"{state.temperature:<10} {', '.join(status):<20}"
        )

    print("-" * 80)


def start_pairing(tpms: TPMSDevice):
    """Start pairing mode for a tire"""
    print("\nSelect tire position to pair:")
    print("1. Front Left")
    print("2. Front Right")
    print("3. Rear Left")
    print("4. Rear Right")
    print("5. Spare Tire")
    print("6. Cancel")

    choice = input("Enter your choice (1-6): ")

    position_map = {
        "1": TirePosition.FRONT_LEFT,
        "2": TirePosition.FRONT_RIGHT,
        "3": TirePosition.REAR_LEFT,
        "4": TirePosition.REAR_RIGHT,
        "5": TirePosition.SPARE_TIRE,
    }

    if choice in position_map:
        position = position_map[choice]
        logger.info(f"Starting pairing for {position.name}...")
        tpms.pair_sensor(position)

        # Wait for pairing to complete or timeout
        print("Pairing in progress. Press Enter to cancel...")
        input()

        # Stop pairing
        logger.info("Stopping pairing...")
        tpms.stop_pairing()

        # Query sensor IDs to get the actual ID
        logger.info("Querying sensor IDs after pairing...")
        tpms.query_sensor_ids()

        # Wait for responses
        print("Waiting for responses (5 seconds)...")
        time.sleep(5)

        # Show the current tire states
        show_tire_states(tpms)
    elif choice == "6":
        logger.info("Pairing cancelled")
    else:
        print("Invalid choice")


def exchange_tires(tpms: TPMSDevice):
    """Exchange tire positions"""
    print("\nSelect first tire position:")
    print("1. Front Left")
    print("2. Front Right")
    print("3. Rear Left")
    print("4. Rear Right")
    print("5. Spare Tire")
    print("6. Cancel")

    choice1 = input("Enter your choice (1-6): ")

    if choice1 == "6":
        logger.info("Exchange cancelled")
        return

    print("\nSelect second tire position:")
    print("1. Front Left")
    print("2. Front Right")
    print("3. Rear Left")
    print("4. Rear Right")
    print("5. Spare Tire")
    print("6. Cancel")

    choice2 = input("Enter your choice (1-6): ")

    if choice2 == "6":
        logger.info("Exchange cancelled")
        return

    position_map = {
        "1": TirePosition.FRONT_LEFT,
        "2": TirePosition.FRONT_RIGHT,
        "3": TirePosition.REAR_LEFT,
        "4": TirePosition.REAR_RIGHT,
        "5": TirePosition.SPARE_TIRE,
    }

    if choice1 in position_map and choice2 in position_map:
        position1 = position_map[choice1]
        position2 = position_map[choice2]

        if position1 == position2:
            print("Cannot exchange a tire with itself")
            return

        logger.info(f"Exchanging {position1.name} with {position2.name}...")
        tpms.exchange_tires(position1, position2)
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
