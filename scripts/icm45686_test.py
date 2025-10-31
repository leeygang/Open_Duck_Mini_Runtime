#!/usr/bin/env python3
"""
ICM45686 IMU Test Script

Comprehensive test utility for ICM45686 6-axis IMU connected via PCA9548A.

Tests:
- I2C communication with PCA9548A multiplexer
- ICM45686 device detection and initialization
- Raw sensor data reading (accelerometer, gyroscope, temperature)
- Sensor fusion (Madgwick filter)
- Orientation output (quaternion, Euler angles)

Usage:
    # Basic test
    python3 scripts/icm45686_test.py

    # With custom I2C addresses
    python3 scripts/icm45686_test.py --imu-addr 0x69 --mux-addr 0x71 --mux-channel 1

    # Test raw driver only (no sensor fusion)
    python3 scripts/icm45686_test.py --raw

    # Debug mode (verbose output)
    python3 scripts/icm45686_test.py --debug
"""

import sys
import time
import argparse
import numpy as np

try:
    from mini_bdx_runtime.pca9548a import PCA9548A, scan_all_channels
    from mini_bdx_runtime.icm45686_driver import ICM45686Driver
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure mini_bdx_runtime is installed:")
    print("  pip install -e .")
    sys.exit(1)


def test_multiplexer(bus: int = 1, mux_addr: int = 0x70):
    """Test PCA9548A multiplexer detection and channel scanning."""
    print("\n" + "=" * 60)
    print("1. Testing PCA9548A Multiplexer")
    print("=" * 60)

    try:
        mux = PCA9548A(bus=bus, address=mux_addr)
        print(f"✓ PCA9548A detected at address 0x{mux_addr:02X}")

        # Show currently active channels
        active = mux.get_selected_channels()
        if active:
            print(f"  Currently active channels: {active}")
        else:
            print("  No channels currently active")

        # Scan all channels
        print("\n  Scanning all channels for I2C devices...")
        results = scan_all_channels(bus=bus, mux_address=mux_addr)

        found_devices = False
        for channel, devices in results.items():
            if devices:
                dev_str = ", ".join(f"0x{addr:02X}" for addr in devices)
                print(f"    Channel {channel}: {dev_str}")
                found_devices = True

        if not found_devices:
            print("    No devices found on any channel")
            print("    ⚠ Check ICM45686 connections")

        mux.disable_all()
        return True

    except Exception as e:
        print(f"✗ Multiplexer test failed: {e}")
        return False


def test_raw_driver(
    bus: int = 1,
    imu_addr: int = 0x68,
    mux_addr: int = 0x70,
    mux_channel: int = 0,
    duration: int = 5
):
    """Test ICM45686 low-level driver."""
    print("\n" + "=" * 60)
    print("2. Testing ICM45686 Driver (Raw Data)")
    print("=" * 60)

    try:
        driver = ICM45686Driver(
            i2c_bus=bus,
            imu_address=imu_addr,
            mux_address=mux_addr,
            mux_channel=mux_channel,
            gyro_range=1000,
            accel_range=4,
        )

        print(f"✓ ICM45686 initialized successfully")
        print(f"  I2C address: 0x{imu_addr:02X}")
        print(f"  Gyro range: ±{driver.gyro_range} dps")
        print(f"  Accel range: ±{driver.accel_range} g")
        print(f"\nReading raw sensor data for {duration}s...\n")

        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration:
            data = driver.read_data()

            print(f"\r  Accel: ({data.accel_x:7.3f}, {data.accel_y:7.3f}, {data.accel_z:7.3f}) m/s² | "
                  f"Gyro: ({data.gyro_x:7.3f}, {data.gyro_y:7.3f}, {data.gyro_z:7.3f}) rad/s | "
                  f"Temp: {data.temperature:5.1f}°C", end="")

            sample_count += 1
            time.sleep(0.02)  # 50 Hz

        avg_rate = sample_count / duration
        print(f"\n\n✓ Raw driver test passed")
        print(f"  Samples: {sample_count}")
        print(f"  Average rate: {avg_rate:.1f} Hz")

        driver.close()
        return True

    except Exception as e:
        print(f"✗ Driver test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_imu_with_fusion(
    bus: int = 1,
    imu_addr: int = 0x68,
    mux_addr: int = 0x70,
    mux_channel: int = 0,
    upside_down: bool = False,
    duration: int = 10
):
    """Test ICM45686Imu class with sensor fusion."""
    print("\n" + "=" * 60)
    print("3. Testing ICM45686 IMU (with Sensor Fusion)")
    print("=" * 60)

    try:
        # Note: This will use duck_config.json if available
        imu = ICM45686Imu(
            sampling_frequency=50,
            user_pitch_bias=0,
            calibrate=False,
            upside_down=upside_down
        )

        print(f"✓ ICM45686 IMU initialized successfully")
        print(f"  Sampling frequency: {imu.sampling_freq} Hz")
        print(f"  Upside down: {upside_down}")
        print(f"\nReading orientation for {duration}s...")
        print("  (Rotate the robot to see orientation changes)\n")

        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration:
            # Get quaternion
            quat = imu.get_data()

            # Get Euler angles
            euler = imu.get_data(euler=True)
            roll, pitch, yaw = np.degrees(euler)

            print(f"\r  Quat: [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] | "
                  f"Roll={roll:7.2f}° Pitch={pitch:7.2f}° Yaw={yaw:7.2f}°", end="")

            sample_count += 1
            time.sleep(0.02)  # 50 Hz

        avg_rate = sample_count / duration
        print(f"\n\n✓ IMU with fusion test passed")
        print(f"  Samples: {sample_count}")
        print(f"  Average rate: {avg_rate:.1f} Hz")

        imu.stop()
        return True

    except Exception as e:
        print(f"✗ IMU fusion test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test ICM45686 IMU with PCA9548A multiplexer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all tests
  python3 scripts/icm45686_test.py

  # Test with IMU at alternate address
  python3 scripts/icm45686_test.py --imu-addr 0x69

  # Test raw driver only
  python3 scripts/icm45686_test.py --raw

  # Test with upside-down mounting
  python3 scripts/icm45686_test.py --upside-down
        """
    )

    parser.add_argument(
        '--bus',
        type=int,
        default=1,
        help='I2C bus number (default: 1)'
    )
    parser.add_argument(
        '--imu-addr',
        type=lambda x: int(x, 0),
        default=0x68,
        help='ICM45686 I2C address (default: 0x68)'
    )
    parser.add_argument(
        '--mux-addr',
        type=lambda x: int(x, 0),
        default=0x70,
        help='PCA9548A I2C address (default: 0x70)'
    )
    parser.add_argument(
        '--mux-channel',
        type=int,
        default=0,
        help='PCA9548A channel number (default: 0)'
    )
    parser.add_argument(
        '--raw',
        action='store_true',
        help='Test raw driver only (skip sensor fusion)'
    )
    parser.add_argument(
        '--upside-down',
        action='store_true',
        help='IMU mounted upside down'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug output'
    )

    args = parser.parse_args()

    print("=" * 60)
    print("ICM45686 IMU Test Suite")
    print("=" * 60)
    print(f"\nConfiguration:")
    print(f"  I2C bus: {args.bus}")
    print(f"  IMU address: 0x{args.imu_addr:02X}")
    print(f"  Multiplexer address: 0x{args.mux_addr:02X}")
    print(f"  Multiplexer channel: {args.mux_channel}")

    results = []

    # Test 1: Multiplexer
    results.append(("Multiplexer", test_multiplexer(args.bus, args.mux_addr)))

    # Test 2: Raw driver
    results.append(("Raw Driver", test_raw_driver(
        args.bus, args.imu_addr, args.mux_addr, args.mux_channel, duration=5
    )))

    # Test 3: IMU with sensor fusion (unless --raw specified)
    if not args.raw:
        results.append(("IMU + Fusion", test_imu_with_fusion(
            args.bus, args.imu_addr, args.mux_addr, args.mux_channel,
            args.upside_down, duration=10
        )))

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed:
            all_passed = False

    print()

    if all_passed:
        print("✓ All tests passed!")
        print("\nNext steps:")
        print("  1. Update duck_config.json with ICM45686 settings")
        print("  2. Run calibration: python3 -m mini_bdx_runtime.icm45686_imu --calibrate")
        print("  3. Use in walking policy: set 'imu_type': 'icm45686' in config")
        return 0
    else:
        print("✗ Some tests failed. Check connections and configuration.")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
