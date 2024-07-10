from tpms import TPMS

# Initialize TPMS
tpms = TPMS(
    port=None,  # Auto-detect
    baudrate=19200,
    temp_unit="Celsius",
    pressure_unit="kPa",
    hardware_version="TY06",
    update_interval=1,
    serial_timeout=1,
)

sensor_mapping = {}

print("Starting sensor identification process...")

# Trigger sensors manually one by one and note the output
for _ in range(10):  # Adjust the range based on your needs
    tire_data = tpms.once()
    print(tire_data)
    user_input = input(
        "Enter the physical position (e.g., Left Front, Right Rear) for the recent data: "
    )
    for position, data in tire_data.items():
        if data["pressure"] is not None:
            sensor_mapping[user_input] = position
            break

print("Sensor identification complete.")
print("Sensor mapping:")
for physical, reported in sensor_mapping.items():
    print(f"{physical} -> {reported}")
