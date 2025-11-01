#!/usr/bin/env python3
"""
Diagnostic script to identify what IMU is at address 0x68 (direct connection, no multiplexer).
"""

from smbus2 import SMBus
import sys

BUS = 1
IMU_ADDR = 0x68

# Common IMU WHO_AM_I registers and expected values
IMU_SIGNATURES = {
    'ICM45686': (0x75, 0xE9),
    'ICM20948': (0x00, 0xEA),
    'ICM20689': (0x75, 0x98),
    'MPU9250': (0x75, 0x71),
    'MPU6050': (0x75, 0x68),
    'BNO055': (0x00, 0xA0),
    'LSM6DS3': (0x0F, 0x69),
}

def main():
    print("=" * 60)
    print("IMU Device Identification at 0x68 (Direct Connection)")
    print("=" * 60)

    with SMBus(BUS) as bus:
        print(f"\nReading WHO_AM_I registers from device at 0x{IMU_ADDR:02X}...\n")

        matches = []

        for imu_name, (reg, expected_val) in IMU_SIGNATURES.items():
            try:
                actual_val = bus.read_byte_data(IMU_ADDR, reg)
                match_str = "✓ MATCH!" if actual_val == expected_val else ""
                print(f"  {imu_name:12} - Reg 0x{reg:02X}: 0x{actual_val:02X} (expect 0x{expected_val:02X}) {match_str}")

                if actual_val == expected_val:
                    matches.append(imu_name)
            except Exception as e:
                print(f"  {imu_name:12} - Reg 0x{reg:02X}: Error ({e})")

        # Also read some other common registers for debugging
        print("\nOther register reads:")
        for reg in [0x00, 0x0F, 0x75]:
            try:
                val = bus.read_byte_data(IMU_ADDR, reg)
                print(f"  Register 0x{reg:02X}: 0x{val:02X}")
            except Exception as e:
                print(f"  Register 0x{reg:02X}: Error ({e})")

        print("\n" + "=" * 60)
        if matches:
            print(f"✓ Detected: {', '.join(matches)}")
        else:
            print("✗ No matching IMU signature found")
            print("\nPossible causes:")
            print("  - Device is not an ICM45686")
            print("  - Device is in sleep/reset state")
            print("  - Wrong I2C address")
            print("  - Power supply issue")
        print("=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
