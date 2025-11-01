#!/usr/bin/env python3
"""
Initialize the ICM45686 properly, then monitor sensor registers.
"""

from smbus2 import SMBus
import time
import struct

BUS = 1
IMU_ADDR = 0x68
MUX_ADDR = 0x70
MUX_CHANNEL = 0

def select_mux():
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)

def read_reg(reg):
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
        return bus.read_byte_data(IMU_ADDR, reg)

def write_reg(reg, val):
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
        bus.write_byte_data(IMU_ADDR, reg, val)

def read_block(start_reg, length):
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
        return bytes(bus.read_i2c_block_data(IMU_ADDR, start_reg, length))

def main():
    print("=" * 70)
    print("ICM45686 Initialize and Monitor")
    print("=" * 70)

    # 1. Verify WHO_AM_I
    print("\n1. Checking WHO_AM_I...")
    who = read_reg(0x72)
    print(f"   WHO_AM_I (0x72) = 0x{who:02X} (expect 0xE9)")
    if who != 0xE9:
        print("   ✗ Wrong device!")
        return

    # 2. Software reset
    print("\n2. Software reset...")
    write_reg(0x02, 0x02)
    time.sleep(0.1)

    # 3. Wake up device
    print("3. Waking up device (PWR_MGMT0 = 0x0F)...")
    write_reg(0x1F, 0x0F)
    time.sleep(0.05)

    # Verify it was written
    pwr = read_reg(0x1F)
    print(f"   PWR_MGMT0 readback = 0x{pwr:02X}")

    # 4. Configure gyroscope (±1000 dps, 1kHz ODR)
    print("4. Configuring gyroscope (GYRO_CONFIG0 = 0x62)...")
    write_reg(0x20, 0x62)  # (0x06 << 4) | 2 = 0x62
    time.sleep(0.05)

    gyro_cfg = read_reg(0x20)
    print(f"   GYRO_CONFIG0 readback = 0x{gyro_cfg:02X}")

    # 5. Configure accelerometer
    # Try different possible addresses for ACCEL_CONFIG0
    print("5. Trying to configure accelerometer...")
    for accel_config_addr in [0x21, 0x22, 0x23, 0x24, 0x50]:
        print(f"   Trying ACCEL_CONFIG0 at 0x{accel_config_addr:02X}...")
        write_reg(accel_config_addr, 0x61)  # (0x06 << 4) | 1 = 0x61 for ±4g
        time.sleep(0.05)

    # 6. Wait for sensor to stabilize
    print("\n6. Waiting for sensor to stabilize...")
    time.sleep(0.5)

    # 7. Read sensor data registers
    print("\n7. Monitoring sensor data (0x09-0x16)...")
    print("   MOVE THE SENSOR to see if values change!")
    print("\n   Sample | Temp  | Accel (X, Y, Z)       | Gyro (X, Y, Z)")
    print("   " + "-" * 65)

    for i in range(50):
        data = read_block(0x09, 14)

        temp = struct.unpack('>h', data[0:2])[0]
        ax = struct.unpack('>h', data[2:4])[0]
        ay = struct.unpack('>h', data[4:6])[0]
        az = struct.unpack('>h', data[6:8])[0]
        gx = struct.unpack('>h', data[8:10])[0]
        gy = struct.unpack('>h', data[10:12])[0]
        gz = struct.unpack('>h', data[12:14])[0]

        print(f"\r   {i:3d}    | {temp:5d} | ({ax:6d}, {ay:6d}, {az:6d}) | ({gx:6d}, {gy:6d}, {gz:6d})", end="")
        time.sleep(0.1)

    print("\n\n" + "=" * 70)
    print("If all values stayed constant (didn't change when moving):")
    print("  → Sensor data registers are at DIFFERENT addresses")
    print("  → Need to scan for the actual data registers")
    print("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
