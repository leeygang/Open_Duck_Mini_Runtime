"""
BNO085 IMU interface compatible with the existing BNO055 IMU class.
Maintains the same API for drop-in replacement.
"""

import board
import busio
import numpy as np
import pickle
import os

from queue import Queue
from threading import Thread
import time
from scipy.spatial.transform import Rotation as R

# BNO08x imports
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class BNO085Imu:
    """
    BNO085 IMU wrapper that maintains API compatibility with the BNO055 Imu class.

    Key differences from BNO055:
    - Uses SH-2 protocol instead of register-based interface
    - Dynamic calibration (no manual offset storage)
    - Per-report enabling (more flexible than mode-based)
    - Different axis configuration
    """

    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True
    ):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0
        self.calibrate = calibrate
        self.upside_down = upside_down

        # Initialize I2C with higher frequency for BNO085
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

        # Initialize BNO085
        # BNO085 can be at address 0x4A (default) or 0x4B
        try:
            self.imu = BNO08X_I2C(i2c, address=0x4A)
        except Exception as e:
            print(f"Failed to initialize at 0x4A, trying 0x4B: {e}")
            self.imu = BNO08X_I2C(i2c, address=0x4B)

        # Enable required sensor reports
        # BNO085 uses report-based configuration instead of modes
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(BNO_REPORT_ACCELEROMETER)

        # Wait for sensor to stabilize
        time.sleep(0.5)

        # Store axis remapping configuration
        # BNO085 doesn't have hardware axis remapping like BNO055,
        # so we'll apply it in software
        self.axis_remap_config = self._get_axis_remap(upside_down)

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        # Note: BNO085 has automatic dynamic calibration
        # Manual calibration offsets are not typically needed
        if self.calibrate:
            print("BNO085 Note: This sensor uses automatic dynamic calibration.")
            print("Manual calibration offset storage is not required.")
            print("For best results, move the sensor through various orientations")
            print("during the first 10-30 seconds of operation.")

            # We can still check calibration status
            print("\nMonitoring calibration for 10 seconds...")
            start_time = time.time()
            while time.time() - start_time < 10:
                # Read sensor to allow calibration to progress
                try:
                    _ = self.imu.quaternion
                    _ = self.imu.gyro
                    _ = self.imu.acceleration
                except:
                    pass
                time.sleep(0.1)

            print("Calibration monitoring complete.")
            print("Note: BNO085 continues to calibrate dynamically during operation.")

        # Load legacy calibration file if it exists (for compatibility)
        # BNO085 doesn't use these offsets, but we check for migration info
        if os.path.exists("imu_calib_data.pkl"):
            print("Found imu_calib_data.pkl (BNO055 calibration)")
            print("Note: BNO085 uses automatic calibration, offsets not applied")

        self.last_imu_data = [0, 0, 0, 0]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def _get_axis_remap(self, upside_down):
        """
        Define axis remapping for upside down orientation.
        Returns a function that remaps quaternion components.
        """
        if upside_down:
            # For upside down mounting, we need to remap axes
            # This is equivalent to BNO055's axis remap configuration
            def remap(quat):
                # quat is (i, j, k, real) from BNO085
                # Apply rotation to match upside-down orientation
                # This remapping matches the BNO055 configuration
                i, j, k, real = quat
                return np.array([-j, -i, -k, real])  # Remapped quaternion
        else:
            # Normal mounting
            def remap(quat):
                i, j, k, real = quat
                return np.array([-j, -i, k, real])  # Remapped quaternion

        return remap

    def imu_worker(self):
        """
        Background thread that continuously reads IMU data.
        Maintains same interface as BNO055 version.
        """
        while True:
            s = time.time()
            try:
                # BNO085 quaternion format: (i, j, k, real)
                quat = self.imu.quaternion

                if quat is None or None in quat:
                    continue

                # Apply axis remapping
                remapped_quat = self.axis_remap_config(quat)

                # Convert to scalar-first for scipy (real, i, j, k)
                quat_scalar_first = np.array([
                    remapped_quat[3],  # real
                    remapped_quat[0],  # i
                    remapped_quat[1],  # j
                    remapped_quat[2],  # k
                ])

                # Convert to euler to apply pitch bias
                euler = R.from_quat(quat_scalar_first, scalar_first=True).as_euler("xyz")
                euler[1] -= np.deg2rad(self.pitch_bias)

                # Convert back to quaternion (scalar last for Isaac/robot)
                final_orientation_quat = R.from_euler("xyz", euler).as_quat()

                self.imu_queue.put(final_orientation_quat.copy())

            except Exception as e:
                print(f"[BNO085 IMU]: {e}")
                continue

            # Maintain sampling frequency
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False, mat=False):
        """
        Get IMU data in requested format.

        Args:
            euler: If True, return euler angles (xyz)
            mat: If True, return rotation matrix

        Returns:
            Quaternion (scalar last), euler angles, or rotation matrix
        """
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        try:
            if not euler and not mat:
                return self.last_imu_data
            elif euler:
                return R.from_quat(self.last_imu_data).as_euler("xyz")
            elif mat:
                return R.from_quat(self.last_imu_data).as_matrix()

        except Exception as e:
            print(f"[BNO085 IMU]: {e}")
            return None

    def get_gyro(self):
        """Get raw gyroscope data (rad/s)."""
        try:
            gyro = self.imu.gyro
            if gyro is None:
                return np.array([0, 0, 0])
            return np.array(gyro)
        except Exception as e:
            print(f"[BNO085 IMU] get_gyro: {e}")
            return np.array([0, 0, 0])

    def get_acceleration(self):
        """Get raw accelerometer data (m/s²)."""
        try:
            accel = self.imu.acceleration
            if accel is None:
                return np.array([0, 0, 0])
            return np.array(accel)
        except Exception as e:
            print(f"[BNO085 IMU] get_acceleration: {e}")
            return np.array([0, 0, 0])


if __name__ == "__main__":
    """Test script for BNO085 IMU."""

    print("BNO085 IMU Test")
    print("=" * 50)
    print("Testing with upside_down=False")
    print("Move the IMU to see values change")
    print("Press Ctrl+C to exit")
    print("=" * 50)

    # Test with calibration monitoring
    # imu = BNO085Imu(50, calibrate=True, upside_down=False)

    # Regular test
    imu = BNO085Imu(50, upside_down=False)

    try:
        while True:
            # Test quaternion output
            quat = imu.get_data()
            print(f"Quaternion: {np.around(quat, 3)}")

            # Test euler output
            euler = imu.get_data(euler=True)
            euler_deg = np.rad2deg(euler)
            print(f"Euler (deg): roll={euler_deg[0]:.1f} pitch={euler_deg[1]:.1f} yaw={euler_deg[2]:.1f}")

            # Test raw sensors
            gyro = imu.get_gyro()
            print(f"Gyro (rad/s): {np.around(gyro, 3)}")

            accel = imu.get_acceleration()
            print(f"Accel (m/s²): {np.around(accel, 3)}")

            print("---")
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nTest stopped")
