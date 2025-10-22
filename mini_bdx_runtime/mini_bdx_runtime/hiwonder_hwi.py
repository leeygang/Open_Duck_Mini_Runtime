"""
Hiwonder Serial Bus Servo (HTD-45H) Hardware Interface
Compatible with LewanSoul LX-16A protocol

This module provides an interface for Hiwonder HTD-45H servos that matches
the API style of the Feetech rustypot_position_hwi.py
"""

import serial
import time
import numpy as np
from typing import Dict, List, Optional


class HiwonderServo:
    """
    Low-level Hiwonder servo protocol implementation
    Protocol documentation: https://www.hiwonder.com/
    """

    # Command definitions
    SERVO_MOVE_TIME_WRITE = 1
    SERVO_MOVE_TIME_READ = 2
    SERVO_MOVE_TIME_WAIT_WRITE = 7
    SERVO_MOVE_TIME_WAIT_READ = 8
    SERVO_MOVE_START = 11
    SERVO_MOVE_STOP = 12
    SERVO_ID_WRITE = 13
    SERVO_ID_READ = 14
    SERVO_ANGLE_OFFSET_ADJUST = 17
    SERVO_ANGLE_OFFSET_WRITE = 18
    SERVO_ANGLE_OFFSET_READ = 19
    SERVO_ANGLE_LIMIT_WRITE = 20
    SERVO_ANGLE_LIMIT_READ = 21
    SERVO_VIN_LIMIT_WRITE = 22
    SERVO_VIN_LIMIT_READ = 23
    SERVO_TEMP_MAX_LIMIT_WRITE = 24
    SERVO_TEMP_MAX_LIMIT_READ = 25
    SERVO_TEMP_READ = 26
    SERVO_VIN_READ = 27
    SERVO_POS_READ = 28
    SERVO_OR_MOTOR_MODE_WRITE = 29
    SERVO_OR_MOTOR_MODE_READ = 30
    SERVO_LOAD_OR_UNLOAD_WRITE = 31
    SERVO_LOAD_OR_UNLOAD_READ = 32
    SERVO_LED_CTRL_WRITE = 33
    SERVO_LED_CTRL_READ = 34
    SERVO_LED_ERROR_WRITE = 35
    SERVO_LED_ERROR_READ = 36

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):
        """
        Initialize serial connection to Hiwonder servos

        Args:
            port: Serial port (e.g., /dev/ttyUSB0, /dev/ttyAMA0)
            baudrate: Communication speed (default 115200 for HTD-45H)
            timeout: Serial read timeout in seconds
        """
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            time.sleep(0.1)  # Wait for port to stabilize
            print(f"Hiwonder servo interface initialized on {port} at {baudrate} baud")
        except Exception as e:
            raise Exception(f"Failed to open serial port {port}: {e}")

    def _checksum(self, data):
        """Calculate checksum for Hiwonder protocol"""
        return (~sum(data)) & 0xFF

    def _send_command(self, servo_id, command, params=None):
        """
        Send command to servo

        Args:
            servo_id: Servo ID (1-253, or 254 for broadcast)
            command: Command byte
            params: List of parameter bytes
        """
        if params is None:
            params = []

        length = 3 + len(params)  # ID + Length + Command + params
        packet = [0x55, 0x55, servo_id, length, command] + params
        checksum = self._checksum(packet[2:])  # Checksum from ID onwards
        packet.append(checksum)

        self.serial.write(bytes(packet))
        self.serial.flush()

    def _read_response(self, expected_length=None):
        """
        Read response from servo

        Returns:
            Tuple of (servo_id, command, params) or None if read failed
        """
        # Wait for header
        header = self.serial.read(2)
        if len(header) != 2 or header != b'\x55\x55':
            return None

        # Read ID, length, command
        data = self.serial.read(3)
        if len(data) != 3:
            return None

        servo_id = data[0]
        length = data[1]
        command = data[2]

        # Read parameters and checksum
        remaining = length - 2  # Length includes command and checksum
        payload = self.serial.read(remaining)
        if len(payload) != remaining:
            return None

        params = list(payload[:-1])
        checksum = payload[-1]

        # Verify checksum
        packet_data = [servo_id, length, command] + params
        expected_checksum = self._checksum(packet_data)
        if checksum != expected_checksum:
            print(f"Checksum error: expected {expected_checksum}, got {checksum}")
            return None

        return (servo_id, command, params)

    def move_servo(self, servo_id, position, duration=0):
        """
        Move servo to position

        Args:
            servo_id: Servo ID
            position: Target position (0-1000, where 500 is center)
            duration: Time to complete movement in milliseconds (0 = immediate)
        """
        params = [
            position & 0xFF,
            (position >> 8) & 0xFF,
            duration & 0xFF,
            (duration >> 8) & 0xFF
        ]
        self._send_command(servo_id, self.SERVO_MOVE_TIME_WRITE, params)

    def read_position(self, servo_id):
        """
        Read current servo position

        Returns:
            Position value (0-1000) or None if read failed
        """
        self._send_command(servo_id, self.SERVO_POS_READ)
        response = self._read_response()

        if response is None or len(response[2]) < 2:
            return None

        position = response[2][0] | (response[2][1] << 8)
        return position

    def set_servo_id(self, old_id, new_id):
        """Change servo ID"""
        self._send_command(old_id, self.SERVO_ID_WRITE, [new_id])
        time.sleep(0.1)

    def read_voltage(self, servo_id):
        """Read servo input voltage in millivolts"""
        self._send_command(servo_id, self.SERVO_VIN_READ)
        response = self._read_response()

        if response is None or len(response[2]) < 2:
            return None

        voltage = response[2][0] | (response[2][1] << 8)
        return voltage  # in mV

    def read_temperature(self, servo_id):
        """Read servo temperature in Celsius"""
        self._send_command(servo_id, self.SERVO_TEMP_READ)
        response = self._read_response()

        if response is None or len(response[2]) < 1:
            return None

        return response[2][0]  # in °C

    def set_angle_offset(self, servo_id, offset):
        """
        Set angle offset (calibration)

        Args:
            servo_id: Servo ID
            offset: Offset in servo units (-125 to 125, where each unit ≈ 0.24°)
        """
        offset_byte = offset if offset >= 0 else (256 + offset)
        self._send_command(servo_id, self.SERVO_ANGLE_OFFSET_WRITE, [offset_byte])

    def read_angle_offset(self, servo_id):
        """Read current angle offset"""
        self._send_command(servo_id, self.SERVO_ANGLE_OFFSET_READ)
        response = self._read_response()

        if response is None or len(response[2]) < 1:
            return None

        offset = response[2][0]
        return offset if offset < 128 else offset - 256  # Convert to signed

    def unload_servo(self, servo_id):
        """Turn off servo torque (free movement)"""
        self._send_command(servo_id, self.SERVO_LOAD_OR_UNLOAD_WRITE, [0])

    def load_servo(self, servo_id):
        """Turn on servo torque"""
        self._send_command(servo_id, self.SERVO_LOAD_OR_UNLOAD_WRITE, [1])

    def scan_servos(self, max_id=253):
        """
        Scan for connected servos

        Returns:
            List of found servo IDs
        """
        found = []
        print("Scanning for Hiwonder servos...")
        for servo_id in range(1, max_id + 1):
            pos = self.read_position(servo_id)
            if pos is not None:
                found.append(servo_id)
                print(f"  Found servo ID {servo_id} at position {pos}")
            time.sleep(0.01)
        return found

    def close(self):
        """Close serial connection"""
        if self.serial.is_open:
            self.serial.close()


class HiwonderHWI:
    """
    High-level hardware interface for Hiwonder servos
    Matches the API of rustypot_position_hwi.py for easy integration
    """

    # Position conversion: Hiwonder uses 0-1000, we use radians
    # HTD-45H has 240° range (0.833° per unit)
    POSITION_MIN = 0
    POSITION_MAX = 1000
    POSITION_CENTER = 500
    ANGLE_RANGE_DEGREES = 240.0  # Total range in degrees

    def __init__(self, duck_config, usb_port="/dev/ttyUSB0", baudrate=115200):
        """
        Initialize Hiwonder hardware interface

        Args:
            duck_config: DuckConfig instance
            usb_port: Serial port for Hiwonder servos
            baudrate: Communication baudrate
        """
        self.duck_config = duck_config
        self.servo = HiwonderServo(port=usb_port, baudrate=baudrate)

        # Define your Hiwonder servo joints (example - adjust to your setup)
        self.joints = {
            "hiwonder_joint_1": 100,  # ID 100
            "hiwonder_joint_2": 101,  # ID 101
            # Add more joints as needed
        }

        # Zero positions (in radians)
        self.zero_pos = {name: 0.0 for name in self.joints.keys()}

        # Init positions (standing pose, in radians)
        self.init_pos = {name: 0.0 for name in self.joints.keys()}

        # Joint offsets from config
        self.joints_offsets = {}
        for joint_name in self.joints.keys():
            self.joints_offsets[joint_name] = duck_config.joints_offset.get(joint_name, 0.0)

        print("Hiwonder HWI initialized")

    def radians_to_position(self, radians):
        """Convert radians to Hiwonder position units (0-1000)"""
        # Map radians to 0-1000 range
        # Center (500) = 0 radians
        # Full range = 240° = 4.189 radians
        degrees = np.degrees(radians)
        # Scale to 0-1000 range where 500 is center
        position = int(self.POSITION_CENTER + (degrees / self.ANGLE_RANGE_DEGREES) * self.POSITION_MAX)
        return np.clip(position, self.POSITION_MIN, self.POSITION_MAX)

    def position_to_radians(self, position):
        """Convert Hiwonder position units to radians"""
        # Reverse of above
        normalized = (position - self.POSITION_CENTER) / self.POSITION_MAX
        degrees = normalized * self.ANGLE_RANGE_DEGREES
        return np.radians(degrees)

    def turn_on(self):
        """Enable all servos"""
        print("Turning on Hiwonder servos...")
        for joint_name, servo_id in self.joints.items():
            self.servo.load_servo(servo_id)
            time.sleep(0.05)

        # Move to init position
        self.set_position_all(self.init_pos)
        time.sleep(1)
        print("Hiwonder servos ready")

    def turn_off(self):
        """Disable all servos (free movement)"""
        print("Turning off Hiwonder servos...")
        for joint_name, servo_id in self.joints.items():
            self.servo.unload_servo(servo_id)
            time.sleep(0.05)

    def set_position(self, joint_name, pos, duration=0):
        """
        Set position for a single joint

        Args:
            joint_name: Name of the joint
            pos: Position in radians
            duration: Movement duration in milliseconds (0 = fast as possible)
        """
        if joint_name not in self.joints:
            print(f"Unknown joint: {joint_name}")
            return

        servo_id = self.joints[joint_name]
        offset = self.joints_offsets[joint_name]
        pos_with_offset = pos + offset

        position_units = self.radians_to_position(pos_with_offset)
        self.servo.move_servo(servo_id, position_units, duration)

    def set_position_all(self, joints_positions, duration=0):
        """
        Set positions for multiple joints

        Args:
            joints_positions: Dict of {joint_name: position_in_radians}
            duration: Movement duration in milliseconds
        """
        for joint_name, position in joints_positions.items():
            if joint_name in self.joints:
                self.set_position(joint_name, position, duration)

    def get_present_positions(self, ignore=None):
        """
        Read current positions of all joints

        Returns:
            numpy array of positions in radians
        """
        if ignore is None:
            ignore = []

        positions = []
        for joint_name, servo_id in self.joints.items():
            if joint_name in ignore:
                continue

            pos_units = self.servo.read_position(servo_id)
            if pos_units is None:
                print(f"Failed to read position for {joint_name}")
                return None

            offset = self.joints_offsets[joint_name]
            pos_rad = self.position_to_radians(pos_units) - offset
            positions.append(pos_rad)

        return np.array(positions)

    def get_present_velocities(self, rad_s=True, ignore=None):
        """
        Read current velocities (Hiwonder doesn't provide velocity feedback)

        Returns:
            numpy array of zeros (no velocity feedback available)
        """
        if ignore is None:
            ignore = []

        num_joints = len([j for j in self.joints.keys() if j not in ignore])
        # Hiwonder servos don't provide velocity feedback
        # Return zeros or estimate from position changes
        return np.zeros(num_joints)

    def scan_servos(self):
        """Scan for connected Hiwonder servos"""
        return self.servo.scan_servos()

    def close(self):
        """Close connection"""
        self.servo.close()


if __name__ == "__main__":
    """Test script for Hiwonder servos"""
    from mini_bdx_runtime.duck_config import DuckConfig

    print("=== Hiwonder Servo Test ===")
    print()

    # Create dummy config
    dummy_config = DuckConfig(config_json_path=None, ignore_default=True)

    try:
        # Initialize interface
        hwi = HiwonderHWI(dummy_config, usb_port="/dev/ttyUSB0")

        # Scan for servos
        print("\nScanning for servos...")
        found = hwi.scan_servos()
        print(f"Found {len(found)} servos: {found}")

        if not found:
            print("No servos found! Check connections.")
            hwi.close()
            exit(1)

        # Test first servo
        test_id = found[0]
        print(f"\nTesting servo ID {test_id}")

        # Read current position
        pos = hwi.servo.read_position(test_id)
        print(f"Current position: {pos}")

        # Read voltage
        voltage = hwi.servo.read_voltage(test_id)
        if voltage:
            print(f"Voltage: {voltage/1000:.2f}V")

        # Read temperature
        temp = hwi.servo.read_temperature(test_id)
        if temp:
            print(f"Temperature: {temp}°C")

        # Test movement
        print("\nTesting movement...")
        positions = [400, 500, 600, 500]
        for pos in positions:
            print(f"Moving to position {pos}")
            hwi.servo.move_servo(test_id, pos, duration=500)
            time.sleep(0.6)
            actual = hwi.servo.read_position(test_id)
            print(f"  Reached position: {actual}")

        print("\nTest complete!")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        hwi.close()
