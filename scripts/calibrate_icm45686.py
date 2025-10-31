#!/usr/bin/env python3
"""
ICM45686 IMU Calibration Script

Performs gyroscope and accelerometer bias calibration for ICM45686.
The robot must be placed on a flat, stable surface and kept completely
stationary during calibration.

Calibration data is saved to icm45686_calib_data.pkl and automatically
loaded by ICM45686Imu on startup.

Usage:
    # Basic calibration (gyro + accel bias)
    python3 scripts/calibrate_icm45686.py

    # Advanced 6-position calibration (more accurate)
    python3 scripts/calibrate_icm45686.py --full

    # Custom I2C configuration
    python3 scripts/calibrate_icm45686.py --imu-addr 0x69 --mux-channel 1
"""

import sys
import time
import argparse
import numpy as np
import pickle

try:
    from mini_bdx_runtime.icm45686_driver import ICM45686Driver
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure mini_bdx_runtime is installed:")
    print("  pip install -e .")
    sys.exit(1)


def collect_samples(driver: ICM45686Driver, num_samples: int = 1000) -> tuple:
    """
    Collect sensor samples for calibration.

    Args:
        driver: ICM45686Driver instance
        num_samples: Number of samples to collect

    Returns:
        Tuple of (gyro_samples, accel_samples) as numpy arrays
    """
    gyro_samples = []
    accel_samples = []

    print(f"Collecting {num_samples} samples (~{num_samples * 0.01:.1f}s)...")

    for i in range(num_samples):
        try:
            data = driver.read_data()

            gyro_samples.append([data.gyro_x, data.gyro_y, data.gyro_z])
            accel_samples.append([data.accel_x, data.accel_y, data.accel_z])

            if (i + 1) % 100 == 0:
                print(f"  {i + 1}/{num_samples} samples collected")

            time.sleep(0.01)  # 100 Hz

        except Exception as e:
            print(f"Error reading sensor: {e}")
            continue

    return np.array(gyro_samples), np.array(accel_samples)


def basic_calibration(driver: ICM45686Driver) -> dict:
    """
    Perform basic single-position calibration.

    Computes gyroscope bias and accelerometer bias while robot is
    stationary on a flat surface.

    Args:
        driver: ICM45686Driver instance

    Returns:
        Dictionary with calibration data
    """
    print("=" * 60)
    print("ICM45686 Basic Calibration")
    print("=" * 60)
    print("\nPlace robot on a FLAT, STABLE surface.")
    print("Keep robot COMPLETELY STATIONARY during calibration.")
    input("\nPress ENTER to start calibration...")

    # Collect samples
    gyro_samples, accel_samples = collect_samples(driver, num_samples=1000)

    # Compute biases (mean)
    gyro_bias = np.mean(gyro_samples, axis=0)
    accel_bias = np.mean(accel_samples, axis=0)

    # For accelerometer, we expect gravity (9.81 m/s²) on Z axis
    # Only remove XY bias, keep Z gravity
    accel_bias[2] -= 9.81

    # Compute standard deviations (sensor noise)
    gyro_std = np.std(gyro_samples, axis=0)
    accel_std = np.std(accel_samples, axis=0)

    print("\n" + "=" * 60)
    print("Calibration Results")
    print("=" * 60)
    print(f"\nGyroscope Bias (rad/s):")
    print(f"  X: {gyro_bias[0]:9.6f}  (std: {gyro_std[0]:.6f})")
    print(f"  Y: {gyro_bias[1]:9.6f}  (std: {gyro_std[1]:.6f})")
    print(f"  Z: {gyro_bias[2]:9.6f}  (std: {gyro_std[2]:.6f})")

    print(f"\nAccelerometer Bias (m/s²):")
    print(f"  X: {accel_bias[0]:9.6f}  (std: {accel_std[0]:.6f})")
    print(f"  Y: {accel_bias[1]:9.6f}  (std: {accel_std[1]:.6f})")
    print(f"  Z: {accel_bias[2]:9.6f}  (std: {accel_std[2]:.6f})")

    # Check if noise levels are reasonable
    gyro_noise_ok = np.all(gyro_std < 0.01)  # < 0.01 rad/s
    accel_noise_ok = np.all(accel_std < 0.1)  # < 0.1 m/s²

    if not gyro_noise_ok:
        print("\n⚠ WARNING: High gyroscope noise detected!")
        print("  Robot may not have been stationary during calibration.")

    if not accel_noise_ok:
        print("\n⚠ WARNING: High accelerometer noise detected!")
        print("  Check for vibrations or unstable surface.")

    calib_data = {
        'gyro_bias': gyro_bias,
        'accel_bias': accel_bias,
        'gyro_std': gyro_std,
        'accel_std': accel_std,
        'timestamp': time.time(),
        'calibration_type': 'basic',
    }

    return calib_data


def full_calibration(driver: ICM45686Driver) -> dict:
    """
    Perform full 6-position accelerometer calibration.

    More accurate than basic calibration. Requires placing robot in
    6 different orientations (±X, ±Y, ±Z facing up).

    Args:
        driver: ICM45686Driver instance

    Returns:
        Dictionary with calibration data
    """
    print("=" * 60)
    print("ICM45686 Full 6-Position Calibration")
    print("=" * 60)
    print("\nThis calibration requires placing the robot in 6 orientations.")
    print("Follow the prompts for each position.")

    positions = [
        ("Z up (normal standing)", [0, 0, 9.81]),
        ("Z down (upside down)", [0, 0, -9.81]),
        ("X up (left side down)", [9.81, 0, 0]),
        ("X down (right side down)", [-9.81, 0, 0]),
        ("Y up (front down)", [0, 9.81, 0]),
        ("Y down (back down)", [0, -9.81, 0]),
    ]

    all_accel_samples = []
    expected_gravity = []

    # Collect gyro bias (stationary)
    print("\n" + "-" * 60)
    print("Step 1: Gyroscope Calibration")
    print("-" * 60)
    print("Place robot in ANY stable position and keep stationary.")
    input("Press ENTER when ready...")

    gyro_samples, _ = collect_samples(driver, num_samples=1000)
    gyro_bias = np.mean(gyro_samples, axis=0)

    # Collect accelerometer data in 6 positions
    print("\n" + "-" * 60)
    print("Step 2: Accelerometer 6-Position Calibration")
    print("-" * 60)

    for i, (pos_name, expected_g) in enumerate(positions, 1):
        print(f"\nPosition {i}/6: {pos_name}")
        print(f"Expected gravity: {expected_g}")
        input("Press ENTER when robot is in position...")

        _, accel_samples = collect_samples(driver, num_samples=500)
        accel_mean = np.mean(accel_samples, axis=0)

        all_accel_samples.append(accel_mean)
        expected_gravity.append(expected_g)

        print(f"Measured: [{accel_mean[0]:.3f}, {accel_mean[1]:.3f}, {accel_mean[2]:.3f}]")

    # Solve for accelerometer calibration parameters
    # Model: a_true = scale * (a_measured - bias)
    # We have 6 equations (positions) to solve for 6 unknowns (3 scale, 3 bias)

    all_accel_samples = np.array(all_accel_samples)
    expected_gravity = np.array(expected_gravity)

    # Simple least-squares solution (assumes scale = 1)
    accel_bias = np.mean(all_accel_samples - expected_gravity, axis=0)

    print("\n" + "=" * 60)
    print("Calibration Results")
    print("=" * 60)
    print(f"\nGyroscope Bias (rad/s):")
    print(f"  [{gyro_bias[0]:9.6f}, {gyro_bias[1]:9.6f}, {gyro_bias[2]:9.6f}]")

    print(f"\nAccelerometer Bias (m/s²):")
    print(f"  [{accel_bias[0]:9.6f}, {accel_bias[1]:9.6f}, {accel_bias[2]:9.6f}]")

    # Compute calibration error
    errors = []
    for measured, expected in zip(all_accel_samples, expected_gravity):
        corrected = measured - accel_bias
        error = np.linalg.norm(corrected - expected)
        errors.append(error)

    mean_error = np.mean(errors)
    max_error = np.max(errors)

    print(f"\nCalibration Error:")
    print(f"  Mean: {mean_error:.4f} m/s²")
    print(f"  Max: {max_error:.4f} m/s²")

    if mean_error > 0.5:
        print("\n⚠ WARNING: High calibration error!")
        print("  Consider repeating calibration with robot more stable.")

    calib_data = {
        'gyro_bias': gyro_bias,
        'accel_bias': accel_bias,
        'accel_samples': all_accel_samples,
        'expected_gravity': expected_gravity,
        'mean_error': mean_error,
        'max_error': max_error,
        'timestamp': time.time(),
        'calibration_type': 'full',
    }

    return calib_data


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate ICM45686 IMU",
        formatter_class=argparse.RawDescriptionHelpFormatter,
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
        '--full',
        action='store_true',
        help='Perform full 6-position calibration'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='icm45686_calib_data.pkl',
        help='Output calibration file (default: icm45686_calib_data.pkl)'
    )

    args = parser.parse_args()

    try:
        # Initialize driver
        print("Initializing ICM45686...")
        driver = ICM45686Driver(
            i2c_bus=args.bus,
            imu_address=args.imu_addr,
            mux_address=args.mux_addr,
            mux_channel=args.mux_channel,
            gyro_range=1000,
            accel_range=4,
        )
        print("✓ ICM45686 initialized\n")

        # Perform calibration
        if args.full:
            calib_data = full_calibration(driver)
        else:
            calib_data = basic_calibration(driver)

        # Save calibration data
        with open(args.output, 'wb') as f:
            pickle.dump(calib_data, f)

        print(f"\n✓ Calibration data saved to {args.output}")
        print("\nThe ICM45686Imu class will automatically load this file on startup.")

        driver.close()
        return 0

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user")
        return 130

    except Exception as e:
        print(f"\n✗ Calibration failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
