"""
Madgwick AHRS Filter for Sensor Fusion

Implements the Madgwick orientation filter algorithm for fusing accelerometer
and gyroscope data into quaternion orientation estimates.

Reference:
    S. Madgwick, "An efficient orientation filter for inertial and
    inertial/magnetic sensor arrays," April 2010.
    https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

The filter uses gradient descent to compute orientation from gyroscope angular
rates and accelerometer gravity vector measurements.
"""

import numpy as np
from typing import Tuple


class MadgwickFilter:
    """
    Madgwick AHRS orientation filter.

    Fuses 6-axis IMU data (accelerometer + gyroscope) to estimate orientation
    as a quaternion without requiring magnetometer.
    """

    def __init__(self, sample_rate: float, beta: float = 0.1):
        """
        Initialize Madgwick filter.

        Args:
            sample_rate: Sampling frequency in Hz
            beta: Filter gain (trade-off between gyro drift and accel noise)
                  Typical range: 0.01 (smooth) to 0.3 (responsive)
                  Default 0.1 works well for most applications
        """
        self.sample_rate = sample_rate
        self.beta = beta
        self.dt = 1.0 / sample_rate

        # Quaternion (w, x, y, z) - initialized to no rotation
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def update(
        self,
        gyro: Tuple[float, float, float],
        accel: Tuple[float, float, float],
    ) -> np.ndarray:
        """
        Update orientation estimate with new sensor data.

        Args:
            gyro: Gyroscope (gx, gy, gz) in rad/s
            accel: Accelerometer (ax, ay, az) in m/s² (any units, will be normalized)

        Returns:
            Quaternion (w, x, y, z) representing current orientation
        """
        gx, gy, gz = gyro
        ax, ay, az = accel

        # Normalize accelerometer measurement
        accel_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm == 0:
            return self.q  # Avoid division by zero

        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

        # Extract quaternion components
        q0, q1, q2, q3 = self.q

        # Gradient descent algorithm corrective step
        # Compute objective function (gravity direction error)
        f = np.array([
            2*(q1*q3 - q0*q2) - ax,
            2*(q0*q1 + q2*q3) - ay,
            2*(0.5 - q1*q1 - q2*q2) - az
        ], dtype=np.float64)

        # Compute Jacobian
        J = np.array([
            [-2*q2,  2*q3, -2*q0, 2*q1],
            [ 2*q1,  2*q0,  2*q3, 2*q2],
            [ 0,    -4*q1, -4*q2, 0   ]
        ], dtype=np.float64)

        # Compute gradient (J^T * f)
        gradient = J.T.dot(f)

        # Normalize gradient
        gradient_norm = np.linalg.norm(gradient)
        if gradient_norm > 0:
            gradient /= gradient_norm

        # Quaternion derivative from gyroscope
        q_dot_gyro = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx
        ], dtype=np.float64)

        # Apply feedback correction
        q_dot = q_dot_gyro - self.beta * gradient

        # Integrate to yield quaternion
        self.q += q_dot * self.dt

        # Normalize quaternion
        q_norm = np.linalg.norm(self.q)
        if q_norm > 0:
            self.q /= q_norm

        return self.q.copy()

    def get_quaternion(self) -> np.ndarray:
        """
        Get current orientation quaternion.

        Returns:
            Quaternion (w, x, y, z)
        """
        return self.q.copy()

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        Convert current quaternion to Euler angles (roll, pitch, yaw).

        Returns:
            Tuple of (roll, pitch, yaw) in radians
            Roll: rotation about X axis [-π, π]
            Pitch: rotation about Y axis [-π/2, π/2]
            Yaw: rotation about Z axis [-π, π]
        """
        q0, q1, q2, q3 = self.q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q0*q1 + q2*q3)
        cosr_cosp = 1 - 2 * (q1*q1 + q2*q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q0*q2 - q3*q1)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0*q3 + q1*q2)
        cosy_cosp = 1 - 2 * (q2*q2 + q3*q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def reset(self, quaternion: Tuple[float, float, float, float] = (1, 0, 0, 0)) -> None:
        """
        Reset filter to a specific orientation.

        Args:
            quaternion: Initial quaternion (w, x, y, z), defaults to identity
        """
        self.q = np.array(quaternion, dtype=np.float64)
        q_norm = np.linalg.norm(self.q)
        if q_norm > 0:
            self.q /= q_norm


class ComplementaryFilter:
    """
    Simple complementary filter for orientation estimation.

    Lighter-weight alternative to Madgwick filter. Combines gyro integration
    (fast response, drifts) with accelerometer (slow, stable).
    """

    def __init__(self, sample_rate: float, alpha: float = 0.98):
        """
        Initialize complementary filter.

        Args:
            sample_rate: Sampling frequency in Hz
            alpha: Filter coefficient (0-1)
                   Higher = trust gyro more (responsive but drifts)
                   Lower = trust accel more (stable but slow)
                   Typical: 0.95-0.99
        """
        self.sample_rate = sample_rate
        self.alpha = alpha
        self.dt = 1.0 / sample_rate

        # Current orientation (roll, pitch, yaw) in radians
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(
        self,
        gyro: Tuple[float, float, float],
        accel: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """
        Update orientation estimate.

        Args:
            gyro: Gyroscope (gx, gy, gz) in rad/s
            accel: Accelerometer (ax, ay, az) in m/s²

        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        gx, gy, gz = gyro
        ax, ay, az = accel

        # Normalize accelerometer
        accel_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm > 0:
            ax /= accel_norm
            ay /= accel_norm
            az /= accel_norm

            # Accelerometer-based angles (assuming gravity only)
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az))

            # Integrate gyroscope
            gyro_roll = self.roll + gx * self.dt
            gyro_pitch = self.pitch + gy * self.dt

            # Complementary filter
            self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        # Integrate yaw (no accel correction available without magnetometer)
        self.yaw += gz * self.dt

        return self.roll, self.pitch, self.yaw

    def get_euler_angles(self) -> Tuple[float, float, float]:
        """Get current Euler angles (roll, pitch, yaw) in radians."""
        return self.roll, self.pitch, self.yaw

    def reset(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        """Reset filter to specific angles."""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to quaternion.

    Args:
        roll: Roll angle in radians (rotation about X)
        pitch: Pitch angle in radians (rotation about Y)
        yaw: Yaw angle in radians (rotation about Z)

    Returns:
        Quaternion (w, x, y, z)
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z], dtype=np.float64)


def quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles.

    Args:
        q: Quaternion (w, x, y, z)

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


if __name__ == "__main__":
    """Test Madgwick filter with simulated data."""
    import matplotlib.pyplot as plt

    print("Madgwick Filter Test")
    print("=" * 60)

    # Simulate sensor data (robot rotating about Z axis)
    sample_rate = 100  # Hz
    duration = 10  # seconds
    num_samples = int(sample_rate * duration)
    time = np.linspace(0, duration, num_samples)

    # Simulated angular velocity (constant rotation)
    gyro_z = 0.5  # rad/s (about 28.6 deg/s)

    # Initialize filter
    filter = MadgwickFilter(sample_rate=sample_rate, beta=0.1)

    # Storage for results
    roll_data = []
    pitch_data = []
    yaw_data = []

    print(f"Simulating {duration}s of rotation at {np.degrees(gyro_z):.1f} deg/s...")

    for t in time:
        # Simulated sensor readings
        # Gyro: constant rotation about Z
        gyro = (0.0, 0.0, gyro_z)

        # Accel: gravity vector (no linear acceleration)
        accel = (0.0, 0.0, 9.81)

        # Update filter
        filter.update(gyro, accel)

        # Get Euler angles
        roll, pitch, yaw = filter.get_euler_angles()
        roll_data.append(np.degrees(roll))
        pitch_data.append(np.degrees(pitch))
        yaw_data.append(np.degrees(yaw))

    # Expected yaw angle
    expected_yaw = np.degrees(gyro_z * time)

    print(f"✓ Filter processed {num_samples} samples")
    print(f"  Final yaw: {yaw_data[-1]:.1f}° (expected: {expected_yaw[-1]:.1f}°)")
    print(f"  Yaw error: {abs(yaw_data[-1] - expected_yaw[-1]):.2f}°")

    # Plot results
    try:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))

        axes[0].plot(time, roll_data)
        axes[0].set_ylabel("Roll (°)")
        axes[0].grid(True)

        axes[1].plot(time, pitch_data)
        axes[1].set_ylabel("Pitch (°)")
        axes[1].grid(True)

        axes[2].plot(time, yaw_data, label="Filtered")
        axes[2].plot(time, expected_yaw, '--', label="Expected")
        axes[2].set_ylabel("Yaw (°)")
        axes[2].set_xlabel("Time (s)")
        axes[2].legend()
        axes[2].grid(True)

        plt.suptitle("Madgwick Filter - Simulated Rotation Test")
        plt.tight_layout()
        plt.savefig("madgwick_test.png")
        print("\n✓ Plot saved to madgwick_test.png")
    except Exception as e:
        print(f"\n(Could not generate plot: {e})")
