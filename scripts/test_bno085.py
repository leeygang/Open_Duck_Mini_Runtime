#!/usr/bin/env python3
"""
Test script for BNO085 IMU.
This script verifies the BNO085 is properly connected and working.
"""

import sys
import time
import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, '/Users/ygli/projects/Open_Duck_Mini_Runtime/mini_bdx_runtime')

from mini_bdx_runtime.bno085_imu import BNO085Imu


def test_basic_reading(upside_down=False):
    """Test basic sensor readings."""
    print("=" * 60)
    print("BNO085 Basic Reading Test")
    print("=" * 60)
    print(f"Configuration: upside_down={upside_down}")
    print("Press Ctrl+C to exit\n")

    try:
        imu = BNO085Imu(sampling_freq=50, upside_down=upside_down)
        print("✓ BNO085 initialized successfully\n")

        # Give sensor time to start
        time.sleep(1)

        while True:
            # Get quaternion
            quat = imu.get_data()
            print(f"Quaternion: [{quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f}]")

            # Get euler angles
            euler = imu.get_data(euler=True)
            euler_deg = np.rad2deg(euler)
            print(f"Euler (deg): Roll={euler_deg[0]:+6.1f}  Pitch={euler_deg[1]:+6.1f}  Yaw={euler_deg[2]:+6.1f}")

            # Get gyroscope
            gyro = imu.get_gyro()
            print(f"Gyro (rad/s): [{gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}]")

            # Get accelerometer
            accel = imu.get_acceleration()
            accel_magnitude = np.linalg.norm(accel)
            print(f"Accel (m/s²): [{accel[0]:+.3f}, {accel[1]:+.3f}, {accel[2]:+.3f}]  |a|={accel_magnitude:.2f}")

            print("-" * 60)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check I2C is enabled: sudo raspi-config → Interface → I2C")
        print("2. Check wiring: SDA to GPIO2, SCL to GPIO3")
        print("3. Verify sensor address: sudo i2cdetect -y 1")
        print("4. Install library: pip install adafruit-circuitpython-bno08x")
        return False

    return True


def test_orientation_check():
    """
    Test to verify correct orientation.
    Provides guidance on which axis is which.
    """
    print("=" * 60)
    print("BNO085 Orientation Check")
    print("=" * 60)
    print("This test helps verify the sensor orientation is correct.\n")
    print("Instructions:")
    print("1. Place the sensor flat on a table (Z-axis pointing up)")
    print("2. Observe the roll, pitch, yaw values")
    print("3. Tilt around different axes and verify:")
    print("   - Roll: Rotation around X-axis (forward)")
    print("   - Pitch: Rotation around Y-axis (left)")
    print("   - Yaw: Rotation around Z-axis (up)")
    print("\nPress Ctrl+C to exit\n")

    try:
        imu = BNO085Imu(sampling_freq=50, upside_down=False)
        time.sleep(1)

        while True:
            euler = imu.get_data(euler=True)
            euler_deg = np.rad2deg(euler)

            print(f"\rRoll: {euler_deg[0]:+6.1f}°  Pitch: {euler_deg[1]:+6.1f}°  Yaw: {euler_deg[2]:+6.1f}°", end="")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nOrientation check complete")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False

    return True


def test_comparison_mode():
    """
    Run in comparison mode if you have both BNO055 and BNO085.
    This helps verify they give similar readings.
    """
    print("=" * 60)
    print("BNO085 vs BNO055 Comparison")
    print("=" * 60)
    print("This test compares BNO085 with BNO055 (if available)\n")

    try:
        # Try to import BNO055
        from mini_bdx_runtime.imu import Imu as BNO055Imu
        print("✓ BNO055 module found")
    except ImportError:
        print("✗ BNO055 module not available for comparison")
        return False

    try:
        print("Initializing BNO085...")
        imu_085 = BNO085Imu(sampling_freq=50, upside_down=False)
        time.sleep(1)
        print("✓ BNO085 ready")

        print("Initializing BNO055...")
        print("Note: This requires BNO055 to be connected")
        imu_055 = BNO055Imu(sampling_freq=50, upside_down=False)
        time.sleep(1)
        print("✓ BNO055 ready\n")

        print("Comparing readings (Press Ctrl+C to exit):\n")

        while True:
            # Get data from both
            euler_085 = imu_085.get_data(euler=True)
            euler_055 = imu_055.get_data(euler=True)

            euler_085_deg = np.rad2deg(euler_085)
            euler_055_deg = np.rad2deg(euler_055)

            diff = euler_085_deg - euler_055_deg

            print(f"BNO085: R={euler_085_deg[0]:+6.1f}  P={euler_085_deg[1]:+6.1f}  Y={euler_085_deg[2]:+6.1f}")
            print(f"BNO055: R={euler_055_deg[0]:+6.1f}  P={euler_055_deg[1]:+6.1f}  Y={euler_055_deg[2]:+6.1f}")
            print(f"Diff:   R={diff[0]:+6.1f}  P={diff[1]:+6.1f}  Y={diff[2]:+6.1f}")
            print("-" * 60)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nComparison complete")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        print("Make sure both sensors are connected if running comparison mode")
        return False

    return True


def main():
    """Main test menu."""
    print("\n" + "=" * 60)
    print("BNO085 IMU Test Suite")
    print("=" * 60)
    print("\nAvailable tests:")
    print("1. Basic reading test (normal orientation)")
    print("2. Basic reading test (upside down orientation)")
    print("3. Orientation check (verify axes)")
    print("4. Comparison with BNO055 (if available)")
    print("5. Exit")

    while True:
        try:
            choice = input("\nSelect test (1-5): ").strip()

            if choice == "1":
                test_basic_reading(upside_down=False)
            elif choice == "2":
                test_basic_reading(upside_down=True)
            elif choice == "3":
                test_orientation_check()
            elif choice == "4":
                test_comparison_mode()
            elif choice == "5":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please select 1-5.")

        except KeyboardInterrupt:
            print("\n\nExiting...")
            break


if __name__ == "__main__":
    main()
