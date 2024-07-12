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
