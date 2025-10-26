"""
Hiwonder Board Controller Hardware Interface (HWI)

This module provides a drop-in replacement for the Feetech HWI (rustypot_position_hwi.py)
using the Hiwonder Bus Servo Controller board commands.

Usage:
    # In your main script, replace:
    # from mini_bdx_runtime.rustypot_position_hwi import HWI
    # with:
    from mini_bdx_runtime.hiwonder_board_hwi import HWI
"""

import time
import numpy as np
from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController


class HWI:
    """
    Hardware interface for Hiwonder servos via the board controller

    This class matches the API of rustypot_position_hwi.HWI to allow easy replacement
    of Feetech servos with Hiwonder servos.
    """

    def __init__(self, duck_config: DuckConfig, usb_port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        """
        Initialize Hiwonder board controller interface

        Args:
            duck_config: Duck configuration object
            usb_port: Serial port for Hiwonder board controller (default /dev/ttyUSB0)
            baudrate: Communication speed (default 9600 for board commands)
        """
        self.duck_config = duck_config

        # Joint names to servo IDs mapping
        # Adjust these IDs to match your servo configuration
        self.joints = {
            "left_hip_yaw": 100,
            "left_hip_roll": 101,
            "left_hip_pitch": 102,
            "left_knee": 103,
            "left_ankle": 104,
            "neck_pitch": 110,
            "head_pitch": 111,
            "head_yaw": 112,
            "head_roll": 113,
            "right_hip_yaw": 105,
            "right_hip_roll": 106,
            "right_hip_pitch": 107,
            "right_knee": 108,
            "right_ankle": 109,
        }

        # Zero position (mechanical zero)
        self.zero_pos = {joint: 0 for joint in self.joints.keys()}

        # Initial standing position (in radians)
        self.init_pos = {
            "left_hip_yaw": 0.002,
            "left_hip_roll": 0.053,
            "left_hip_pitch": -0.63,
            "left_knee": 1.368,
            "left_ankle": -0.784,
            "neck_pitch": 0.0,
            "head_pitch": 0.0,
            "head_yaw": 0,
            "head_roll": 0,
            "right_hip_yaw": -0.003,
            "right_hip_roll": -0.065,
            "right_hip_pitch": 0.635,
            "right_knee": 1.379,
            "right_ankle": -0.796,
        }

        # Load joint offsets from configuration
        self.joints_offsets = self.duck_config.joints_offset

        # Initialize board controller
        self.board = HiwonderBoardController(port=usb_port, baudrate=baudrate)

        # Control parameters
        # Note: Hiwonder servos don't have kp/kd like Feetech,
        # but we keep the variables for API compatibility
        self.kps = np.ones(len(self.joints)) * 32
        self.kds = np.ones(len(self.joints)) * 0
        self.low_torque_kps = np.ones(len(self.joints)) * 2

        # Movement duration (in milliseconds) for position commands
        self.default_move_duration_ms = 20  # 20ms = 50Hz control rate

        print(f"Hiwonder Board HWI initialized with {len(self.joints)} joints")

    def _radians_to_servo_units(self, radians: float) -> int:
        """
        Convert radians to Hiwonder servo units (0-1000)

        Hiwonder servos:
        - 0 units = -120° = -2.094 radians (left limit)
        - 500 units = 0° = 0 radians (center)
        - 1000 units = +120° = +2.094 radians (right limit)

        Total range: 240° = 4.189 radians
        1 radian = 238.73 units
        """
        # Convert radians to servo units
        # Formula: units = 500 + (radians * 1000 / (240° in radians))
        units = 500 + (radians * 1000.0 / 4.18879)  # 4.18879 = 240° in radians

        # Clamp to valid range
        units = int(max(0, min(1000, units)))
        return units

    def _servo_units_to_radians(self, units: int) -> float:
        """
        Convert Hiwonder servo units (0-1000) to radians

        Args:
            units: Servo position units (0-1000)

        Returns:
            Position in radians
        """
        # Formula: radians = (units - 500) * (240° in radians) / 1000
        radians = (units - 500) * 4.18879 / 1000.0
        return radians

    def set_kps(self, kps):
        """
        Set proportional gains (for API compatibility)

        Note: Hiwonder servos don't have adjustable PID gains via this protocol.
        This method is kept for API compatibility but has no effect.
        """
        self.kps = kps
        print("Warning: Hiwonder servos don't support adjustable kp gains via this protocol")

    def set_kds(self, kds):
        """
        Set derivative gains (for API compatibility)

        Note: Hiwonder servos don't have adjustable PID gains via this protocol.
        This method is kept for API compatibility but has no effect.
        """
        self.kds = kds
        print("Warning: Hiwonder servos don't support adjustable kd gains via this protocol")

    def set_kp(self, id, kp):
        """
        Set proportional gain for single servo (for API compatibility)

        Note: Hiwonder servos don't have adjustable PID gains via this protocol.
        """
        print(f"Warning: Hiwonder servos don't support adjustable kp gains via this protocol")

    def turn_on(self):
        """
        Power on servos and move to initial position

        This is a safe startup sequence that gradually brings servos online.
        """
        print("Turning on Hiwonder servos...")

        # Move all servos to init position with slower movement
        print("Moving to init position...")
        self.set_position_all(self.init_pos, move_duration_ms=2000)

        time.sleep(2.5)
        print("Servos ready")

    def turn_off(self):
        """
        Disable torque on all servos (unload)
        """
        print("Turning off Hiwonder servos (unloading)...")
        servo_ids = list(self.joints.values())
        self.board.unload_servos(servo_ids)
        print("Servos unloaded")

    def set_position(self, joint_name: str, pos: float):
        """
        Set position of a single joint

        Args:
            joint_name: Name of the joint (e.g., "left_hip_yaw")
            pos: Position in radians
        """
        if joint_name not in self.joints:
            raise ValueError(f"Unknown joint: {joint_name}")

        servo_id = self.joints[joint_name]

        # Apply offset
        pos_with_offset = pos + self.joints_offsets[joint_name]

        # Convert to servo units
        servo_units = self._radians_to_servo_units(pos_with_offset)

        # Send move command
        self.board.move_servos([(servo_id, servo_units, self.default_move_duration_ms)])

    def set_position_all(self, joints_positions: dict, move_duration_ms: int = None):
        """
        Set positions of all joints simultaneously

        Args:
            joints_positions: Dictionary mapping joint names to positions (in radians)
            move_duration_ms: Movement duration in milliseconds (default: self.default_move_duration_ms)
        """
        if move_duration_ms is None:
            move_duration_ms = self.default_move_duration_ms

        # Build servo commands
        servo_commands = []

        for joint_name, position in joints_positions.items():
            if joint_name not in self.joints:
                print(f"Warning: Unknown joint {joint_name}, skipping")
                continue

            servo_id = self.joints[joint_name]

            # Apply offset
            pos_with_offset = position + self.joints_offsets[joint_name]

            # Convert to servo units
            servo_units = self._radians_to_servo_units(pos_with_offset)

            servo_commands.append((servo_id, servo_units, move_duration_ms))

        # Send all commands at once (synchronized movement)
        if servo_commands:
            self.board.move_servos(servo_commands)

    def get_present_positions(self, ignore: list = None) -> np.ndarray:
        """
        Get current positions of all joints

        Args:
            ignore: List of joint names to ignore

        Returns:
            Numpy array of joint positions in radians
        """
        if ignore is None:
            ignore = []

        try:
            # Get servo IDs to read
            servo_ids_to_read = [
                self.joints[joint]
                for joint in self.joints.keys()
                if joint not in ignore
            ]

            # Read positions from board
            positions_data = self.board.read_servo_positions(servo_ids_to_read)

            if positions_data is None:
                print("Error: Failed to read servo positions")
                return None

            # Create a mapping of servo_id to position
            id_to_position = {servo_id: pos for servo_id, pos in positions_data}

            # Build result array in the correct order
            present_positions = []
            for joint_name in self.joints.keys():
                if joint_name in ignore:
                    continue

                servo_id = self.joints[joint_name]

                if servo_id in id_to_position:
                    # Convert servo units to radians
                    servo_units = id_to_position[servo_id]
                    radians = self._servo_units_to_radians(servo_units)

                    # Remove offset
                    radians_without_offset = radians - self.joints_offsets[joint_name]
                    present_positions.append(radians_without_offset)
                else:
                    print(f"Warning: No position data for servo {servo_id} ({joint_name})")
                    present_positions.append(0.0)

            return np.array(np.around(present_positions, 3))

        except Exception as e:
            print(f"Error reading positions: {e}")
            return None

    def get_present_velocities(self, rad_s: bool = True, ignore: list = None) -> np.ndarray:
        """
        Get current velocities of all joints

        Note: Hiwonder board controller protocol doesn't support velocity reading.
        This method returns zeros for API compatibility.

        Args:
            rad_s: If True, return in rad/s (otherwise rev/min) - not used
            ignore: List of joint names to ignore

        Returns:
            Numpy array of zeros (velocity feedback not available)
        """
        if ignore is None:
            ignore = []

        # Return zeros - Hiwonder board protocol doesn't provide velocity feedback
        num_joints = len([j for j in self.joints.keys() if j not in ignore])
        return np.zeros(num_joints)

    def get_battery_voltage(self) -> float:
        """
        Get battery voltage from board

        Returns:
            Battery voltage in volts, or None if read failed
        """
        return self.board.get_battery_voltage()

    def close(self):
        """Close the board controller connection"""
        self.board.close()
