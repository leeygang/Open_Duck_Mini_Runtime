"""
ICM45686 IMU High-Level Interface

Drop-in replacement for the BNO055-based Imu class, using ICM45686
6-axis IMU with software-based sensor fusion (Madgwick filter).

This module provides the same API as mini_bdx_runtime/imu.py but uses
ICM45686 hardware connected through PCA9548A I2C multiplexer.

Usage:
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu

    imu = ICM45686Imu(
        sampling_frequency=50,
        user_pitch_bias=0,
        upside_down=False
    )

    # Get quaternion (scalar-last: x, y, z, w)
    quat = imu.get_data()

    # Get Euler angles (roll, pitch, yaw)
    euler = imu.get_data(euler=True)

    # Get rotation matrix
    mat = imu.get_data(mat=True)
"""

import numpy as np
import pickle
import os
import time
from queue import Queue
from threading import Thread
from scipy.spatial.transform import Rotation as R
from typing import Optional, Union

from mini_bdx_runtime.icm45686_driver import ICM45686Driver
from mini_bdx_runtime.madgwick_filter import MadgwickFilter
from mini_bdx_runtime.duck_config import DuckConfig


class ICM45686Imu:
    """
    High-level IMU interface for ICM45686.

    Compatible with the existing Imu class API. Runs sensor fusion
    in a background thread and provides orientation data via queue.
    """

    def __init__(
        self,
        sampling_frequency: int = 50,
        user_pitch_bias: float = 0.0,
        calibrate: bool = False,
        upside_down: bool = False,
        config_path: Optional[str] = None,
    ):
        """
        Initialize ICM45686 IMU.

        Args:
            sampling_frequency: Sampling rate in Hz
            user_pitch_bias: User-specified pitch bias in degrees
            calibrate: If True, perform calibration and exit
            upside_down: If True, remap axes for upside-down mounting
            config_path: Path to duck_config.json (optional)
        """
        self.sampling_freq = sampling_frequency
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0.0
        self.calibrate = calibrate
        self.upside_down = upside_down

        # Load configuration
        try:
            config = DuckConfig(config_path)
            icm_config = config.config.get('icm45686_config', {})
        except:
            # Fallback to defaults if config not available
            icm_config = {}

        # Extract configuration
        i2c_bus = icm_config.get('i2c_bus', 1)
        imu_address = int(icm_config.get('imu_address', '0x68'), 16)
        mux_address = int(icm_config.get('mux_address', '0x70'), 16) if 'mux_address' in icm_config else 0x70
        mux_channel = icm_config.get('mux_channel', 0)
        gyro_range = icm_config.get('gyro_range', 1000)
        accel_range = icm_config.get('accel_range', 4)
        madgwick_beta = icm_config.get('madgwick_beta', 0.1)

        # Axis remapping configuration
        self.axis_remap = icm_config.get('axis_remap', {'x': 'x', 'y': 'y', 'z': 'z'})
        self.axis_sign = icm_config.get('axis_sign', {'x': 1, 'y': 1, 'z': 1})

        # Override axis remap based on upside_down parameter
        if upside_down:
            # Match BNO055 upside-down remapping behavior
            self.axis_remap = {'x': 'y', 'y': 'x', 'z': 'z'}
            self.axis_sign = {'x': -1, 'y': -1, 'z': -1}
        else:
            self.axis_remap = {'x': 'y', 'y': 'x', 'z': 'z'}
            self.axis_sign = {'x': -1, 'y': 1, 'z': 1}

        # Initialize hardware driver
        try:
            self.driver = ICM45686Driver(
                i2c_bus=i2c_bus,
                imu_address=imu_address,
                mux_address=mux_address,
                mux_channel=mux_channel,
                gyro_range=gyro_range,
                accel_range=accel_range,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to initialize ICM45686 driver: {e}")

        # Initialize Madgwick filter
        self.filter = MadgwickFilter(
            sample_rate=float(sampling_frequency),
            beta=madgwick_beta
        )

        # Calibration data
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.calib_file = "icm45686_calib_data.pkl"

        # Pitch bias
        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        # Perform calibration if requested
        if self.calibrate:
            self._perform_calibration()
            print("CALIBRATION DONE")
            exit()

        # Load calibration data if available
        if os.path.exists(self.calib_file):
            self._load_calibration()
            print(f"✓ Loaded calibration from {self.calib_file}")
        else:
            print(f"⚠ {self.calib_file} not found")
            print("  IMU is running uncalibrated. Run calibration for best results.")

        # Threading for background data acquisition
        self.last_imu_data = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion (x, y, z, w)
        self.imu_queue = Queue(maxsize=1)
        self.running = True
        self.worker_thread = Thread(target=self._imu_worker, daemon=True)
        self.worker_thread.start()

    def _remap_axes(self, data: tuple) -> tuple:
        """
        Remap and flip axes based on configuration.

        Args:
            data: Tuple of (x, y, z) sensor data

        Returns:
            Remapped tuple of (x', y', z')
        """
        x, y, z = data
        data_dict = {'x': x, 'y': y, 'z': z}

        # Remap axes
        x_new = data_dict[self.axis_remap['x']] * self.axis_sign['x']
        y_new = data_dict[self.axis_remap['y']] * self.axis_sign['y']
        z_new = data_dict[self.axis_remap['z']] * self.axis_sign['z']

        return (x_new, y_new, z_new)

    def _imu_worker(self):
        """
        Background worker thread that continuously reads IMU data,
        applies sensor fusion, and updates the orientation queue.
        """
        while self.running:
            start_time = time.time()

            try:
                # Read sensor data
                imu_data = self.driver.read_data()

                # Apply calibration (bias correction)
                gyro = np.array([imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z]) - self.gyro_bias
                accel = np.array([imu_data.accel_x, imu_data.accel_y, imu_data.accel_z]) - self.accel_bias

                # Apply axis remapping
                gyro = self._remap_axes(tuple(gyro))
                accel = self._remap_axes(tuple(accel))

                # Update filter (returns quaternion in scalar-first: w, x, y, z)
                quat_scalar_first = self.filter.update(gyro, accel)

                # Convert to Euler angles to apply pitch bias
                euler = R.from_quat(quat_scalar_first, scalar_first=True).as_euler('xyz')

                # Apply pitch bias correction
                euler[1] -= np.deg2rad(self.pitch_bias)

                # Convert back to quaternion (scalar-last format: x, y, z, w)
                # This matches the format expected by the walking policy
                final_quat = R.from_euler('xyz', euler).as_quat()

                # Update queue (non-blocking, replace old data)
                try:
                    self.imu_queue.get_nowait()  # Remove old data
                except:
                    pass
                self.imu_queue.put(final_quat.copy())

            except Exception as e:
                print(f"[ICM45686 IMU]: {e}")
                continue

            # Maintain sampling frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, 1.0 / self.sampling_freq - elapsed)
            time.sleep(sleep_time)

    def get_data(
        self,
        euler: bool = False,
        mat: bool = False
    ) -> Optional[Union[np.ndarray, tuple]]:
        """
        Get latest IMU orientation data.

        Args:
            euler: If True, return Euler angles (roll, pitch, yaw) in radians
            mat: If True, return rotation matrix (3x3)

        Returns:
            - Quaternion (x, y, z, w) if euler=False and mat=False (default)
            - Euler angles (roll, pitch, yaw) if euler=True
            - Rotation matrix (3x3) if mat=True
            - None if error occurs
        """
        # Get latest data from queue (non-blocking)
        try:
            self.last_imu_data = self.imu_queue.get_nowait()
        except:
            pass  # Use last available data if queue is empty

        try:
            if not euler and not mat:
                # Return quaternion (scalar-last: x, y, z, w)
                return self.last_imu_data
            elif euler:
                # Return Euler angles
                return R.from_quat(self.last_imu_data).as_euler('xyz')
            elif mat:
                # Return rotation matrix
                return R.from_quat(self.last_imu_data).as_matrix()
        except Exception as e:
            print(f"[ICM45686 IMU]: {e}")
            return None

    def _perform_calibration(self):
        """
        Perform gyroscope and accelerometer bias calibration.

        Robot must be stationary on a flat surface during calibration.
        """
        print("=" * 60)
        print("ICM45686 Calibration")
        print("=" * 60)
        print("\nPlace robot on a FLAT, STABLE surface.")
        print("Keep robot COMPLETELY STATIONARY during calibration.")
        input("\nPress ENTER to start calibration...")

        num_samples = 1000
        sample_delay = 0.01  # 10ms between samples

        gyro_samples = []
        accel_samples = []

        print(f"\nCollecting {num_samples} samples (~{num_samples * sample_delay:.1f}s)...")

        for i in range(num_samples):
            try:
                imu_data = self.driver.read_data()

                gyro_samples.append([imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z])
                accel_samples.append([imu_data.accel_x, imu_data.accel_y, imu_data.accel_z])

                if (i + 1) % 100 == 0:
                    print(f"  {i + 1}/{num_samples} samples collected")

                time.sleep(sample_delay)
            except Exception as e:
                print(f"Error reading sensor: {e}")
                continue

        # Compute biases (mean)
        gyro_bias = np.mean(gyro_samples, axis=0)
        accel_bias = np.mean(accel_samples, axis=0)

        # For accelerometer, we expect gravity (9.81 m/s²) on Z axis
        # Only remove XY bias, keep Z gravity
        accel_bias[2] -= 9.81

        print("\nCalibration Results:")
        print(f"  Gyro bias: {gyro_bias}")
        print(f"  Accel bias: {accel_bias}")

        # Save calibration data
        calib_data = {
            'gyro_bias': gyro_bias,
            'accel_bias': accel_bias,
            'timestamp': time.time(),
        }

        with open(self.calib_file, 'wb') as f:
            pickle.dump(calib_data, f)

        print(f"\n✓ Calibration saved to {self.calib_file}")

    def _load_calibration(self):
        """Load calibration data from file."""
        try:
            with open(self.calib_file, 'rb') as f:
                calib_data = pickle.load(f)

            self.gyro_bias = calib_data['gyro_bias']
            self.accel_bias = calib_data['accel_bias']

        except Exception as e:
            print(f"Warning: Failed to load calibration data: {e}")
            self.gyro_bias = np.zeros(3)
            self.accel_bias = np.zeros(3)

    def stop(self):
        """Stop background worker thread."""
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        self.driver.close()

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.stop()
        except:
            pass


if __name__ == "__main__":
    """Test ICM45686Imu class."""
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Test ICM45686 IMU")
    parser.add_argument('--calibrate', action='store_true', help='Perform calibration')
    parser.add_argument('--upside-down', action='store_true', help='IMU mounted upside down')
    args = parser.parse_args()

    print("ICM45686 IMU Test")
    print("=" * 60)

    try:
        imu = ICM45686Imu(
            sampling_frequency=50,
            user_pitch_bias=0,
            calibrate=args.calibrate,
            upside_down=args.upside_down
        )

        print("✓ IMU initialized successfully")
        print("\nReading orientation data (Ctrl+C to stop)...\n")

        while True:
            # Get quaternion
            quat = imu.get_data()

            # Get Euler angles
            euler = imu.get_data(euler=True)
            roll, pitch, yaw = np.degrees(euler)

            print(f"\rQuaternion: [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] | "
                  f"Euler: Roll={roll:7.2f}° Pitch={pitch:7.2f}° Yaw={yaw:7.2f}°", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n✓ Test stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if 'imu' in locals():
            imu.stop()
