"""
README: CANbus Data Parser and Decoder

This script reads a CAN bus log file named `CANbus.can`, which is delimited by semicolons (`;`).
Each line contains a timestamp, CAN ID, number of data bytes, and up to 8 data bytes in hex format.

The script processes only CAN IDs 0x400 through 0x405:

- 0x400 & 0x401: 
    - 8 data bytes: decode motor current, vehicle speed (km/h), and odometer (km)
- 0x402 & 0x403:
    - 2 data bytes: decode inverter and motor temperatures in °F (each offset by +40°C)
- 0x404 & 0x405:
    - 3 data bytes: decode state of charge (%), fault level, and fault code

The first timestamp is used as time zero. All others are output as relative (zero-based) time.

Output files:
    - 400.csv, 401.csv, 402.csv, 403.csv, 404.csv, 405.csv

Each file includes headers and decoded fields relevant to the corresponding CAN ID.
"""

import csv

# === Decoder for 0x400 and 0x401 ===
def decode_motor_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    if len(bytes_int) < 8:
        bytes_int += [0] * (8 - len(bytes_int))

    motor_current = ((bytes_int[1] << 8) | bytes_int[0]) / 10
    vehicle_speed = ((bytes_int[3] << 8) | bytes_int[2]) / 10
    low_word  = ((bytes_int[5] << 8) | bytes_int[4])
    high_word = ((bytes_int[7] << 8) | bytes_int[6])
    odometer = ((high_word << 16) | low_word) / 100

    return motor_current, vehicle_speed, odometer

# === Decoder for 0x402 and 0x403 ===
def decode_temp_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    if len(bytes_int) < 2:
        bytes_int += [0] * (2 - len(bytes_int))

    inverter_temp_C = bytes_int[0] + 40
    motor_temp_C = bytes_int[1] + 40
    inverter_temp_F = inverter_temp_C * 9 / 5 + 32
    motor_temp_F = motor_temp_C * 9 / 5 + 32

    return inverter_temp_F, motor_temp_F

# === Decoder for 0x404 and 0x405 ===
def decode_fault_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    if len(bytes_int) < 3:
        bytes_int += [0] * (3 - len(bytes_int))

    state_of_charge = bytes_int[0]
    fault_level = bytes_int[1]
    fault_code = bytes_int[2]

    return state_of_charge, fault_code, fault_level

# === Setup ===
input_file = "CANbus.can"

# Grouping CAN IDs by decoder
motor_ids = {"0x400", "0x401"}
temp_ids = {"0x402", "0x403"}
fault_ids = {"0x404", "0x405"}

# Open all output CSVs and write appropriate headers
output_files = {
    "0x400": open("400.csv", "w", newline=''),
    "0x401": open("401.csv", "w", newline=''),
    "0x402": open("402.csv", "w", newline=''),
    "0x403": open("403.csv", "w", newline=''),
    "0x404": open("404.csv", "w", newline=''),
    "0x405": open("405.csv", "w", newline=''),
}

writers = {}
for cid, f in output_files.items():
    writer = csv.writer(f)
    if cid in motor_ids:
        writer.writerow(["time", "id", "motor_current", "vehicle_speed_kph", "odometer_km"])
    elif cid in temp_ids:
        writer.writerow(["time", "id", "inverter_temp_F", "motor_temp_F"])
    elif cid in fault_ids:
        writer.writerow(["time", "id", "state_of_charge", "fault_code", "fault_level"])
    writers[cid] = writer

# Establish zero-based time
base_time = None

# === Read and parse the CANbus log ===
with open(input_file, "r") as infile:
    for line_number, line in enumerate(infile, start=1):
        parts = line.strip().split(';')
        if len(parts) < 5:
            continue  # malformed or header

        try:
            time = float(parts[0])          # timestamp
            can_id = parts[2]               # e.g., "0x401"
            data_len = int(parts[3])        # expected byte count
            data_bytes = parts[4:4 + data_len]
        except Exception as e:
            print(f"[Line {line_number}] Parse error: {e}")
            continue

        if can_id not in writers:
            continue  # only process 0x400–0x405

        if base_time is None:
            base_time = time

        rel_time = time - base_time

        try:
            if can_id in motor_ids:
                motor_current, vehicle_speed, odometer = decode_motor_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}", can_id,
                    f"{motor_current:.1f}",
                    f"{vehicle_speed:.1f}",
                    f"{odometer:.2f}"
                ])
            elif can_id in temp_ids:
                inverter_temp_F, motor_temp_F = decode_temp_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}", can_id,
                    f"{inverter_temp_F:.1f}",
                    f"{motor_temp_F:.1f}"
                ])
            elif can_id in fault_ids:
                soc, fault_code, fault_level = decode_fault_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}", can_id,
                    soc, fault_code, fault_level
                ])
        except Exception as e:
            print(f"[Line {line_number}] Decode error: {e}")

# === Cleanup ===
for f in output_files.values():
    f.close()
