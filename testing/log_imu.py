import serial
import csv
import time
import math

# Set this to your actual port and baud rate
PORT = "/dev/ttyACM0"
BAUD = 115200

def safe_float(val):
    try:
        return float(val)
    except ValueError:
        return float('nan')  # or use None or 0.0

with serial.Serial(PORT, BAUD, timeout=1) as ser, open("sensor_log2.csv", "w", newline="") as f:
    writer = csv.writer(f)
    header = ["ax", "ay", "az", "gyrx", "gyry", "gyrz",
              "baro", "gps_x", "gps_y", "gps_z", "gps_hAcc", "gps_vAcc",
              "heading", "mag_x", "mag_y", "mag_z", "time"]
    writer.writerow(header)

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            if not line:
                continue
            parts = line.split(",")

            if len(parts) != len(header):
                print(f"Skipping malformed line ({len(parts)} fields): {line}")
                continue

            parsed = [safe_float(p) for p in parts]
            writer.writerow(parsed)
            print(parsed)

        except KeyboardInterrupt:
            print("\nLogging stopped.")
            break
        except Exception as e:
            print("Error:", e)