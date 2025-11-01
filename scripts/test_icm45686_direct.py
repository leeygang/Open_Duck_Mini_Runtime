#!/usr/bin/env python3
"""
Test ICM45686 driver with direct I2C connection (no multiplexer).
"""

import sys
import time
from mini_bdx_runtime.icm45686_driver import ICM45686Driver

def main():
    print("=" * 60)
    print("ICM45686 Direct Connection Test")
    print("=" * 60)

    try:
        # Initialize without multiplexer
        print("\n1. Initializing ICM45686 driver (no multiplexer)...")
        driver = ICM45686Driver(
            i2c_bus=1,
            imu_address=0x68,
            mux_address=None,  # No multiplexer
            mux_channel=None,
            gyro_range=1000,
            accel_range=4,
        )
        print("✓ ICM45686 initialized successfully!")
        print(f"  Gyro range: ±{driver.gyro_range} dps")
        print(f"  Accel range: ±{driver.accel_range} g")

        # Read sensor data
        print("\n2. Reading sensor data for 5 seconds...")
        print("   (Move the sensor to see values change)\n")

        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < 5:
            data = driver.read_data()

            print(f"\r  Accel: ({data.accel_x:7.3f}, {data.accel_y:7.3f}, {data.accel_z:7.3f}) m/s² | "
                  f"Gyro: ({data.gyro_x:7.3f}, {data.gyro_y:7.3f}, {data.gyro_z:7.3f}) rad/s | "
                  f"Temp: {data.temperature:5.1f}°C", end="")

            sample_count += 1
            time.sleep(0.02)  # 50 Hz

        avg_rate = sample_count / 5
        print(f"\n\n✓ Test completed successfully!")
        print(f"  Samples: {sample_count}")
        print(f"  Average rate: {avg_rate:.1f} Hz")

        driver.close()

        print("\n" + "=" * 60)
        print("✓ ICM45686 is working correctly!")
        print("\nNext steps:")
        print("  1. Reconnect through PCA9548A multiplexer")
        print("  2. Run: uv run scripts/icm45686_test.py")
        print("=" * 60)
        return 0

    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(130)
