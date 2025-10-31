#!/usr/bin/env python3
"""
Dual IMU Test Script

Tests both BNO085 and ICM45686 connected via PCA9548A multiplexer.
Can run tests individually or compare both IMUs side-by-side.

Hardware Setup:
    Raspberry Pi I2C Bus 1
        └── PCA9548A (0x70)
            ├── Channel 0: ICM45686 (0x68)
            └── Channel 1: BNO085 (0x4A)

Usage:
    # Test both IMUs simultaneously
    python3 scripts/test_dual_imu.py

    # Test only ICM45686
    python3 scripts/test_dual_imu.py --imu icm45686

    # Test only BNO085
    python3 scripts/test_dual_imu.py --imu bno085

    # Custom configuration
    python3 scripts/test_dual_imu.py --icm-channel 2 --bno-channel 3

    # Compare IMU outputs side-by-side
    python3 scripts/test_dual_imu.py --compare
"""

import sys
import time
import argparse
import numpy as np
from typing import Optional, Tuple

try:
    from mini_bdx_runtime.pca9548a import PCA9548A, scan_all_channels
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu
    from mini_bdx_runtime.bno085_imu_mux import BNO085ImuMux
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure mini_bdx_runtime is installed:")
    print("  pip install -e .")
    sys.exit(1)


def test_multiplexer(bus: int = 1, mux_addr: int = 0x70) -> bool:
    """Test PCA9548A multiplexer and scan for devices."""
    print("\n" + "=" * 60)
    print("Testing PCA9548A Multiplexer")
    print("=" * 60)

    try:
        mux = PCA9548A(bus=bus, address=mux_addr)
        print(f"✓ PCA9548A detected at address 0x{mux_addr:02X}")

        # Scan all channels
        print("\nScanning all channels...")
        results = scan_all_channels(bus=bus, mux_address=mux_addr)

        found_devices = False
        for channel, devices in results.items():
            if devices:
                dev_str = ", ".join(f"0x{addr:02X}" for addr in devices)
                print(f"  Channel {channel}: {dev_str}")
                found_devices = True
            else:
                print(f"  Channel {channel}: (empty)")

        if not found_devices:
            print("\n⚠ No devices found on any channel")

        mux.disable_all()
        return True

    except Exception as e:
        print(f"✗ Multiplexer test failed: {e}")
        return False


def test_icm45686(
    mux_addr: int = 0x70,
    mux_channel: int = 0,
    duration: int = 10
) -> bool:
    """Test ICM45686 IMU."""
    print("\n" + "=" * 60)
    print(f"Testing ICM45686 (Channel {mux_channel})")
    print("=" * 60)

    try:
        imu = ICM45686Imu(
            sampling_frequency=50,
            user_pitch_bias=0,
            calibrate=False,
            upside_down=False
        )

        print(f"✓ ICM45686 initialized")
        print(f"\nReading orientation for {duration}s...")
        print("  (Rotate the sensor to see changes)\n")

        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration:
            quat = imu.get_data()
            euler = imu.get_data(euler=True)
            roll, pitch, yaw = np.degrees(euler)

            print(f"\r  Quat: [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] | "
                  f"R={roll:7.2f}° P={pitch:7.2f}° Y={yaw:7.2f}°", end="")

            sample_count += 1
            time.sleep(0.02)

        print(f"\n\n✓ ICM45686 test passed ({sample_count} samples)")
        imu.stop()
        return True

    except Exception as e:
        print(f"✗ ICM45686 test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_bno085(
    mux_addr: int = 0x70,
    mux_channel: int = 1,
    bno085_addr: int = 0x4A,
    duration: int = 10
) -> bool:
    """Test BNO085 IMU."""
    print("\n" + "=" * 60)
    print(f"Testing BNO085 (Channel {mux_channel})")
    print("=" * 60)

    try:
        imu = BNO085ImuMux(
            sampling_freq=50,
            mux_address=mux_addr,
            mux_channel=mux_channel,
            bno085_address=bno085_addr,
            upside_down=False
        )

        print(f"\nReading orientation for {duration}s...")
        print("  (Rotate the sensor to see changes)\n")

        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration:
            quat = imu.get_data()
            euler = imu.get_data(euler=True)
            roll, pitch, yaw = np.degrees(euler)

            print(f"\r  Quat: [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] | "
                  f"R={roll:7.2f}° P={pitch:7.2f}° Y={yaw:7.2f}°", end="")

            sample_count += 1
            time.sleep(0.02)

        print(f"\n\n✓ BNO085 test passed ({sample_count} samples)")
        imu.stop()
        return True

    except Exception as e:
        print(f"✗ BNO085 test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def compare_imus(
    mux_addr: int = 0x70,
    icm_channel: int = 0,
    bno_channel: int = 1,
    bno085_addr: int = 0x4A,
    duration: int = 30
) -> bool:
    """Run both IMUs simultaneously and compare outputs."""
    print("\n" + "=" * 60)
    print("Dual IMU Comparison Test")
    print("=" * 60)
    print(f"\nInitializing both IMUs...")

    try:
        # Initialize ICM45686
        print(f"  ICM45686 on channel {icm_channel}...")
        icm_imu = ICM45686Imu(
            sampling_frequency=50,
            user_pitch_bias=0,
            calibrate=False,
            upside_down=False
        )
        print("  ✓ ICM45686 ready")

        # Initialize BNO085
        print(f"  BNO085 on channel {bno_channel}...")
        bno_imu = BNO085ImuMux(
            sampling_freq=50,
            mux_address=mux_addr,
            mux_channel=bno_channel,
            bno085_address=bno085_addr,
            upside_down=False
        )
        print("  ✓ BNO085 ready")

        print(f"\nComparing outputs for {duration}s...")
        print("  ICM45686 | BNO085 | Difference")
        print("  " + "-" * 54)

        start_time = time.time()
        errors = []

        while time.time() - start_time < duration:
            # Read both IMUs
            icm_quat = icm_imu.get_data()
            bno_quat = bno_imu.get_data()

            icm_euler = icm_imu.get_data(euler=True)
            bno_euler = bno_imu.get_data(euler=True)

            # Calculate differences
            euler_diff = np.degrees(icm_euler - bno_euler)
            quat_diff = np.linalg.norm(icm_quat - bno_quat)

            errors.append(np.abs(euler_diff))

            # Display
            icm_rpy = np.degrees(icm_euler)
            bno_rpy = np.degrees(bno_euler)

            print(f"\r  ICM: R={icm_rpy[0]:6.1f}° P={icm_rpy[1]:6.1f}° Y={icm_rpy[2]:6.1f}° | "
                  f"BNO: R={bno_rpy[0]:6.1f}° P={bno_rpy[1]:6.1f}° Y={bno_rpy[2]:6.1f}° | "
                  f"Δ: {euler_diff[0]:5.1f}° {euler_diff[1]:5.1f}° {euler_diff[2]:5.1f}°", end="")

            time.sleep(0.05)

        # Statistics
        errors = np.array(errors)
        mean_error = np.mean(errors, axis=0)
        max_error = np.max(errors, axis=0)
        std_error = np.std(errors, axis=0)

        print("\n\n" + "=" * 60)
        print("Comparison Statistics")
        print("=" * 60)
        print(f"\nMean Error (deg):  Roll={mean_error[0]:5.2f}  Pitch={mean_error[1]:5.2f}  Yaw={mean_error[2]:5.2f}")
        print(f"Max Error (deg):   Roll={max_error[0]:5.2f}  Pitch={max_error[1]:5.2f}  Yaw={max_error[2]:5.2f}")
        print(f"Std Dev (deg):     Roll={std_error[0]:5.2f}  Pitch={std_error[1]:5.2f}  Yaw={std_error[2]:5.2f}")

        # Check if errors are reasonable
        max_acceptable_error = 10.0  # degrees
        if np.all(mean_error < max_acceptable_error):
            print(f"\n✓ IMUs agree within {max_acceptable_error}° tolerance")
        else:
            print(f"\n⚠ High disagreement detected (>{max_acceptable_error}°)")
            print("  Check IMU mounting orientations and calibration")

        icm_imu.stop()
        bno_imu.stop()
        return True

    except Exception as e:
        print(f"\n✗ Comparison test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test dual IMU setup with PCA9548A",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test both IMUs
  python3 scripts/test_dual_imu.py

  # Test only ICM45686
  python3 scripts/test_dual_imu.py --imu icm45686

  # Compare both IMUs
  python3 scripts/test_dual_imu.py --compare

  # Custom channels
  python3 scripts/test_dual_imu.py --icm-channel 2 --bno-channel 3
        """
    )

    parser.add_argument(
        '--mux-addr',
        type=lambda x: int(x, 0),
        default=0x70,
        help='PCA9548A address (default: 0x70)'
    )
    parser.add_argument(
        '--icm-channel',
        type=int,
        default=0,
        help='ICM45686 channel (default: 0)'
    )
    parser.add_argument(
        '--bno-channel',
        type=int,
        default=1,
        help='BNO085 channel (default: 1)'
    )
    parser.add_argument(
        '--bno085-addr',
        type=lambda x: int(x, 0),
        default=0x4A,
        help='BNO085 address (default: 0x4A)'
    )
    parser.add_argument(
        '--imu',
        choices=['icm45686', 'bno085', 'both'],
        default='both',
        help='Which IMU to test (default: both)'
    )
    parser.add_argument(
        '--compare',
        action='store_true',
        help='Run comparison test (both IMUs simultaneously)'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=10,
        help='Test duration in seconds (default: 10)'
    )

    args = parser.parse_args()

    print("=" * 60)
    print("Dual IMU Test Suite")
    print("=" * 60)
    print(f"\nConfiguration:")
    print(f"  Multiplexer: 0x{args.mux_addr:02X}")
    print(f"  ICM45686: Channel {args.icm_channel}, Address 0x68")
    print(f"  BNO085: Channel {args.bno_channel}, Address 0x{args.bno085_addr:02X}")

    results = []

    # Test multiplexer
    results.append(("Multiplexer", test_multiplexer(mux_addr=args.mux_addr)))

    if args.compare:
        # Comparison mode - run both simultaneously
        results.append(("Comparison", compare_imus(
            mux_addr=args.mux_addr,
            icm_channel=args.icm_channel,
            bno_channel=args.bno_channel,
            bno085_addr=args.bno085_addr,
            duration=args.duration
        )))
    else:
        # Individual tests
        if args.imu in ['icm45686', 'both']:
            results.append(("ICM45686", test_icm45686(
                mux_addr=args.mux_addr,
                mux_channel=args.icm_channel,
                duration=args.duration
            )))

        if args.imu in ['bno085', 'both']:
            results.append(("BNO085", test_bno085(
                mux_addr=args.mux_addr,
                mux_channel=args.bno_channel,
                bno085_addr=args.bno085_addr,
                duration=args.duration
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
        print("  1. Update duck_config.json with your chosen IMU")
        print("  2. Run calibration if needed")
        print("  3. Test with walking policy")
        return 0
    else:
        print("✗ Some tests failed")
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
