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
