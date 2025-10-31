"""
BNO085 IMU with PCA9548A Multiplexer Support

Extended version of bno085_imu.py that supports connection via PCA9548A
I2C multiplexer. This allows multiple BNO085 sensors or co-existence with
other I2C devices that share the same address.

Hardware Setup:
    Raspberry Pi I2C -> PCA9548A -> BNO085
                                 -> ICM45686 (or other sensors)

Usage:
    imu = BNO085ImuMux(
        sampling_freq=50,
        mux_address=0x70,
        mux_channel=1,
        bno085_address=0x4A,
        upside_down=False
    )
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
from typing import Optional

# BNO08x imports
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# PCA9548A multiplexer
from mini_bdx_runtime.pca9548a import PCA9548A


class BNO085ImuMux:
    """
    BNO085 IMU with PCA9548A multiplexer support.

    Maintains API compatibility with BNO055 Imu class and original BNO085Imu class.
    Adds support for PCA9548A I2C multiplexer to enable multi-IMU setups.
    """

    def __init__(
        self,
        sampling_freq: int,
        user_pitch_bias: float = 0,
        calibrate: bool = False,
        upside_down: bool = True,
        mux_address: Optional[int] = None,
        mux_channel: Optional[int] = None,
        bno085_address: int = 0x4A,
        i2c_bus: int = 1,
    ):
        """
        Initialize BNO085 IMU with optional multiplexer support.

        Args:
            sampling_freq: Sampling frequency in Hz
            user_pitch_bias: User-specified pitch bias in degrees
            calibrate: Enable calibration monitoring mode
            upside_down: IMU mounted upside down
            mux_address: PCA9548A I2C address (None = direct connection)
            mux_channel: PCA9548A channel (0-7, None = direct connection)
            bno085_address: BNO085 I2C address (0x4A or 0x4B)
            i2c_bus: I2C bus number (for multiplexer, default 1)
        """
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0
        self.calibrate = calibrate
        self.upside_down = upside_down
        self.bno085_address = bno085_address

        # Initialize multiplexer if specified
        self.mux: Optional[PCA9548A] = None
        if mux_address is not None and mux_channel is not None:
            print(f"Initializing PCA9548A at 0x{mux_address:02X}, channel {mux_channel}")
            self.mux = PCA9548A(bus=i2c_bus, address=mux_address)
            self.mux.select_channel(mux_channel)
            print(f"✓ Multiplexer channel {mux_channel} selected")
            time.sleep(0.1)  # Let channel selection settle

        # Initialize I2C with higher frequency for BNO085
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

        # Initialize BNO085
        # BNO085 can be at address 0x4A (default) or 0x4B
        try:
            print(f"Initializing BNO085 at 0x{bno085_address:02X}...")
            self.imu = BNO08X_I2C(i2c, address=bno085_address)
            print(f"✓ BNO085 initialized at 0x{bno085_address:02X}")
        except Exception as e:
            # Try alternate address
            alt_address = 0x4B if bno085_address == 0x4A else 0x4A
            print(f"Failed at 0x{bno085_address:02X}, trying 0x{alt_address:02X}: {e}")
            self.imu = BNO08X_I2C(i2c, address=alt_address)
            self.bno085_address = alt_address
            print(f"✓ BNO085 initialized at 0x{alt_address:02X}")

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
        self.running = True
        self.worker_thread = Thread(target=self.imu_worker, daemon=True)
        self.worker_thread.start()

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
        while self.running:
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

                # Update queue (non-blocking replace)
                try:
                    self.imu_queue.get_nowait()
                except:
                    pass
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

    def stop(self):
        """Stop background worker thread."""
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.stop()
            if self.mux:
                self.mux.disable_all()
        except:
            pass


if __name__ == "__main__":
    """Test script for BNO085 IMU with multiplexer."""
    import argparse

    parser = argparse.ArgumentParser(description="Test BNO085 IMU with PCA9548A")
    parser.add_argument('--mux-addr', type=lambda x: int(x, 0), default=0x70,
                        help='PCA9548A address (default: 0x70)')
    parser.add_argument('--mux-channel', type=int, default=1,
                        help='PCA9548A channel (default: 1)')
    parser.add_argument('--bno085-addr', type=lambda x: int(x, 0), default=0x4A,
                        help='BNO085 address (default: 0x4A)')
    parser.add_argument('--no-mux', action='store_true',
                        help='Direct connection (no multiplexer)')
    parser.add_argument('--upside-down', action='store_true',
                        help='IMU mounted upside down')
    args = parser.parse_args()

    print("BNO085 IMU Test (with multiplexer support)")
    print("=" * 60)

    try:
        if args.no_mux:
            print("Mode: Direct I2C connection")
            imu = BNO085ImuMux(
                sampling_freq=50,
                upside_down=args.upside_down,
                bno085_address=args.bno085_addr
            )
        else:
            print("Mode: Via PCA9548A multiplexer")
            imu = BNO085ImuMux(
                sampling_freq=50,
                mux_address=args.mux_addr,
                mux_channel=args.mux_channel,
                bno085_address=args.bno085_addr,
                upside_down=args.upside_down
            )

        print("\nReading IMU data (Ctrl+C to stop)...")
        print("Move the IMU to see values change\n")

        while True:
            # Test quaternion output
            quat = imu.get_data()

            # Test euler output
            euler = imu.get_data(euler=True)
            euler_deg = np.rad2deg(euler)

            # Test raw sensors
            gyro = imu.get_gyro()
            accel = imu.get_acceleration()

            print(f"\rQuat: [{quat[0]:7.4f}, {quat[1]:7.4f}, {quat[2]:7.4f}, {quat[3]:7.4f}] | "
                  f"Euler: R={euler_deg[0]:6.1f}° P={euler_deg[1]:6.1f}° Y={euler_deg[2]:6.1f}° | "
                  f"Gyro: [{gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f}] rad/s", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'imu' in locals():
            imu.stop()
