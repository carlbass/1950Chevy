import csv
from datetime import datetime
import os
import sys

# === Check if input file exists ===
input_file = "logFile.can"
if not os.path.exists(input_file):
    print(f"❌ ERROR: Input file '{input_file}' not found.")
    sys.exit(1)

# === Define CAN ID groups ===
motor_ids = {"0x400", "0x401"}
temp_ids = {"0x402", "0x403"}
fault_ids = {"0x404", "0x405"}
raw_ids = {"0x500"}  # <== NEW ID
all_ids = motor_ids | temp_ids | fault_ids | raw_ids

# === Timestamp-based suffix for output files ===
now = datetime.now()
time_suffix = f"{now.year}-{now.month}-{now.day}-{now.hour}-{now.minute}"

# === Prepare output files and writers ===
output_files = {}
writers = {}

def open_output_file(candid, headers):
    file_id = candid[2:]  # e.g., "0x401" → "401"
    filename = f"{file_id}-{time_suffix}.csv"
    try:
        f = open(filename, "w", newline='')
        writer = csv.writer(f)
        writer.writerow(headers)
        output_files[candid] = f
        writers[candid] = writer
        print(f"✅ Opened: {filename}")
    except Exception as e:
        print(f"❌ Failed to open output file for {candid}: {e}")
        sys.exit(1)

for cid in motor_ids:
    open_output_file(cid, ["time", "motor_current", "vehicle_speed_kph", "odometer_km"])
for cid in temp_ids:
    open_output_file(cid, ["time", "inverter_temp_F", "motor_temp_F"])
for cid in fault_ids:
    open_output_file(cid, ["time", "state_of_charge", "fault_code", "fault_level"])
for cid in raw_ids:
    open_output_file(cid, ["time"] + [f"byte{i}" for i in range(8)])  # 0x500 raw bytes

# === Decoding functions ===
def decode_motor_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    bytes_int += [0] * (8 - len(bytes_int))
    motor_current = ((bytes_int[1] << 8) | bytes_int[0]) / 10
    vehicle_speed = ((bytes_int[3] << 8) | bytes_int[2]) / 10
    low_word  = ((bytes_int[5] << 8) | bytes_int[4])
    high_word = ((bytes_int[7] << 8) | bytes_int[6])
    odometer = ((high_word << 16) | low_word) / 100
    return motor_current, vehicle_speed, odometer

def decode_temp_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    bytes_int += [0] * (2 - len(bytes_int))
    inverter_temp_C = bytes_int[0] - 40
    motor_temp_C = bytes_int[1] - 40
    inverter_temp_F = inverter_temp_C * 9 / 5 + 32
    motor_temp_F = motor_temp_C * 9 / 5 + 32
    return inverter_temp_F, motor_temp_F

def decode_fault_data(byte_strs):
    bytes_int = [int(b, 16) for b in byte_strs if b]
    bytes_int += [0] * (3 - len(bytes_int))
    return bytes_int[0], bytes_int[2], bytes_int[1]  # SOC, FaultCode, FaultLevel

# === Process the log file ===
base_time = None
total_lines = 0
written_lines = 0

with open(input_file, "r") as infile:
    for line_number, line in enumerate(infile, start=1):
        total_lines += 1
        parts = line.strip().split(';')
        if len(parts) < 5:
            continue

        try:
            time = float(parts[0])
            can_id = parts[2]
            data_len = int(parts[3])
            data_bytes = parts[4:4 + data_len]
        except Exception as e:
            print(f"[Line {line_number}] Parse error: {e}")
            continue

        if can_id not in all_ids:
            continue

        if base_time is None:
            base_time = time

        rel_time = time - base_time

        try:
            if can_id in motor_ids:
                mc, spd, odo = decode_motor_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}",
                    f"{mc:.1f}",
                    f"{spd:.1f}",
                    f"{odo:.2f}"
                ])
            elif can_id in temp_ids:
                invF, motF = decode_temp_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}",
                    f"{invF:.1f}",
                    f"{motF:.1f}"
                ])
            elif can_id in fault_ids:
                soc, code, level = decode_fault_data(data_bytes)
                writers[can_id].writerow([
                    f"{rel_time:.3f}", soc, code, level
                ])
            elif can_id in raw_ids:
                # write raw bytes (max 8)
                raw = data_bytes + [''] * (8 - len(data_bytes))
                writers[can_id].writerow([f"{rel_time:.3f}"] + raw)
            written_lines += 1
        except Exception as e:
            print(f"[Line {line_number}] Decode error: {e}")

# === Close all files ===
for f in output_files.values():
    f.close()

print("\n✅ Script finished.")
print(f"   Lines processed: {total_lines}")
print(f"   Entries written: {written_lines}")
