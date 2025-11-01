#!/usr/bin/env python3
"""
Debug script to read the exact registers the driver uses for sensor data.
Shows raw bytes and interpreted 16-bit values.
"""

from smbus2 import SMBus
import struct
import time
import sys

BUS = 1
IMU_ADDR = 0x68

def read_sensor_burst():
    """Read 14-byte sensor data burst like the driver does."""
    with SMBus(BUS) as bus:
        # Read 14 bytes starting at 0x09 (TEMP_DATA1)
        data = bytes(bus.read_i2c_block_data(IMU_ADDR, 0x09, 14))

        # Parse as 16-bit big-endian signed values
        temp_raw = struct.unpack('>h', data[0:2])[0]
        accel_x = struct.unpack('>h', data[2:4])[0]
        accel_y = struct.unpack('>h', data[4:6])[0]
        accel_z = struct.unpack('>h', data[6:8])[0]
        gyro_x = struct.unpack('>h', data[8:10])[0]
        gyro_y = struct.unpack('>h', data[10:12])[0]
        gyro_z = struct.unpack('>h', data[12:14])[0]

        return data, (temp_raw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

def main():
    print("=" * 60)
    print("ICM45686 Sensor Data Register Debug")
    print("=" * 60)
    print("\nReading 14 bytes starting at 0x09:")
    print("  0x09-0x0A: Temperature (2 bytes)")
    print("  0x0B-0x10: Accelerometer XYZ (6 bytes)")
    print("  0x11-0x16: Gyroscope XYZ (6 bytes)")
    print("\n⚠ Move the sensor to see if values change!\n")

    print("Reg Range | Hex Bytes                   | Interpreted Value")
    print("-" * 60)

    try:
        for i in range(50):
            raw_bytes, values = read_sensor_burst()
            temp, ax, ay, az, gx, gy, gz = values

            # Format output
            temp_hex = f"{raw_bytes[0]:02X} {raw_bytes[1]:02X}"
            ax_hex = f"{raw_bytes[2]:02X} {raw_bytes[3]:02X}"
            ay_hex = f"{raw_bytes[4]:02X} {raw_bytes[5]:02X}"
            az_hex = f"{raw_bytes[6]:02X} {raw_bytes[7]:02X}"
            gx_hex = f"{raw_bytes[8]:02X} {raw_bytes[9]:02X}"
            gy_hex = f"{raw_bytes[10]:02X} {raw_bytes[11]:02X}"
            gz_hex = f"{raw_bytes[12]:02X} {raw_bytes[13]:02X}"

            print(f"\r0x09-0x0A | {temp_hex}                         | Temp:   {temp:6d}", end="")
            print(f"\n0x0B-0x10 | {ax_hex} {ay_hex} {az_hex}           | Accel: ({ax:6d}, {ay:6d}, {az:6d})", end="")
            print(f"\n0x11-0x16 | {gx_hex} {gy_hex} {gz_hex}           | Gyro:  ({gx:6d}, {gy:6d}, {gz:6d})", end="")
            print("\n" + "-" * 60, end="")

            time.sleep(0.1)

            # Move cursor up
            print("\r\033[4A", end="")

    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("Summary:")
        print("  - If Temperature changes: temp sensor works")
        print("  - If Accel values = 0: accelerometer not configured")
        print("  - If Gyro changes: gyroscope works")
        print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
