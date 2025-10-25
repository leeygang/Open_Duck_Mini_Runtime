"""
Hiwonder Bus Servo Controller Board Interface

This module provides control for the Hiwonder Bus Servo Controller board itself,
separate from direct servo commands. The board has its own command protocol for
board-level operations like power management, multi-servo sync, etc.

Protocol documentation: https://docs.hiwonder.com/projects/Bus-Servo-Controller/en/latest/index.html
Section 2.1: Control Board Protocol
"""

import serial
import time
from typing import List, Optional, Tuple


class HiwonderBoardController:
    """
    Hiwonder Bus Servo Controller Board interface

    This class handles board-level commands that are separate from direct servo commands.
    The board acts as an intermediary between the computer and servos.
    """

    # Board command definitions (Section 2.1 of documentation)
    # Frame format: Header(2) + ID(1) + Length(1) + CMD(1) + Params(n) + Checksum(1)

    CMD_GET_BOARD_INFO = 0x01       # Get board firmware version and info
    CMD_SET_SERVO_POWER = 0x02      # Turn servo power on/off
    CMD_GET_SERVO_POWER = 0x03      # Query servo power status
    CMD_SET_SERVO_OFFSET = 0x04     # Set offset for servo
    CMD_GET_SERVO_OFFSET = 0x05     # Get offset for servo
    CMD_SAVE_OFFSET = 0x06          # Save offset to EEPROM
    CMD_READ_VOLTAGE = 0x07         # Read board voltage
    CMD_MULTI_SERVO_MOVE = 0x08     # Move multiple servos simultaneously
    CMD_MULTI_SERVO_UNLOAD = 0x09   # Unload (disable torque) multiple servos
    CMD_SERVO_MODE = 0x0A           # Set servo mode (position/continuous rotation)
    CMD_RESET_BOARD = 0xFF          # Reset the board

    HEADER = [0x55, 0x55]           # Protocol header
    BOARD_ID = 0xFE                 # Board ID (different from servo IDs)

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):
        """
        Initialize connection to Hiwonder Bus Servo Controller board

        Args:
            port: Serial port (e.g., /dev/ttyUSB0, /dev/serial0)
            baudrate: Communication speed (default 115200)
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
            time.sleep(0.1)
            print(f"Hiwonder Board Controller initialized on {port} at {baudrate} baud")
        except Exception as e:
            raise Exception(f"Failed to open serial port {port}: {e}")

    def _checksum(self, data: List[int]) -> int:
        """Calculate checksum for board protocol"""
        return (~sum(data)) & 0xFF

    def _send_board_command(self, command: int, params: List[int] = None) -> None:
        """
        Send command to board

        Args:
            command: Command byte
            params: List of parameter bytes
        """
        if params is None:
            params = []

        length = 3 + len(params)  # ID + Length + Command + params
        packet = self.HEADER + [self.BOARD_ID, length, command] + params
        checksum = self._checksum(packet[2:])  # Checksum from ID onwards
        packet.append(checksum)

        self.serial.write(bytes(packet))
        self.serial.flush()

    def _read_board_response(self, expected_length: Optional[int] = None) -> Optional[List[int]]:
        """
        Read response from board

        Args:
            expected_length: Expected number of bytes in response (if known)

        Returns:
            List of response bytes, or None if error
        """
        # Read header
        header = self.serial.read(2)
        if len(header) != 2 or header != bytes(self.HEADER):
            return None

        # Read ID, length, command
        metadata = self.serial.read(3)
        if len(metadata) != 3:
            return None

        board_id, length, command = metadata

        # Read parameters and checksum
        remaining = length - 2  # Length includes command byte and checksum
        data = self.serial.read(remaining)
        if len(data) != remaining:
            return None

        # Verify checksum
        params = list(data[:-1])
        received_checksum = data[-1]
        calculated_checksum = self._checksum([board_id, length, command] + params)

        if received_checksum != calculated_checksum:
            print(f"Checksum error: expected {calculated_checksum}, got {received_checksum}")
            return None

        return [board_id, command] + params

    def get_board_info(self) -> Optional[dict]:
        """
        Get board firmware version and information

        Returns:
            Dictionary with board info, or None if failed
        """
        self._send_board_command(self.CMD_GET_BOARD_INFO)
        response = self._read_board_response()

        if response and len(response) >= 4:
            return {
                'firmware_version': f"{response[2]}.{response[3]}",
                'board_type': 'Hiwonder Bus Servo Controller'
            }
        return None

    def set_servo_power(self, enable: bool) -> bool:
        """
        Turn servo power on or off

        Args:
            enable: True to turn power on, False to turn off

        Returns:
            True if successful
        """
        param = [0x01 if enable else 0x00]
        self._send_board_command(self.CMD_SET_SERVO_POWER, param)
        time.sleep(0.1)

        # Verify by reading power status
        return self.get_servo_power() == enable

    def get_servo_power(self) -> Optional[bool]:
        """
        Query servo power status

        Returns:
            True if power is on, False if off, None if error
        """
        self._send_board_command(self.CMD_GET_SERVO_POWER)
        response = self._read_board_response()

        if response and len(response) >= 3:
            return bool(response[2])
        return None

    def read_board_voltage(self) -> Optional[float]:
        """
        Read board input voltage

        Returns:
            Voltage in volts, or None if error
        """
        self._send_board_command(self.CMD_READ_VOLTAGE)
        response = self._read_board_response()

        if response and len(response) >= 4:
            # Voltage in millivolts (2 bytes, little-endian)
            voltage_mv = response[2] | (response[3] << 8)
            return voltage_mv / 1000.0
        return None

    def multi_servo_move(self, servo_commands: List[Tuple[int, int, int]]) -> bool:
        """
        Move multiple servos simultaneously

        Args:
            servo_commands: List of (servo_id, position, time) tuples
                           where position is 0-1000 and time is in milliseconds

        Returns:
            True if command sent successfully
        """
        # Build parameter list: count + (id + pos_low + pos_high + time_low + time_high) * count
        params = [len(servo_commands)]

        for servo_id, position, duration in servo_commands:
            params.append(servo_id)
            params.append(position & 0xFF)  # Low byte
            params.append((position >> 8) & 0xFF)  # High byte
            params.append(duration & 0xFF)  # Low byte
            params.append((duration >> 8) & 0xFF)  # High byte

        self._send_board_command(self.CMD_MULTI_SERVO_MOVE, params)
        return True

    def multi_servo_unload(self, servo_ids: List[int]) -> bool:
        """
        Unload (disable torque) multiple servos simultaneously

        Args:
            servo_ids: List of servo IDs to unload

        Returns:
            True if command sent successfully
        """
        params = [len(servo_ids)] + servo_ids
        self._send_board_command(self.CMD_MULTI_SERVO_UNLOAD, params)
        return True

    def reset_board(self) -> None:
        """Reset the board (will disconnect and need to reconnect)"""
        self._send_board_command(self.CMD_RESET_BOARD)
        time.sleep(1.0)

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Board controller connection closed")


# Example usage
if __name__ == "__main__":
    print("Testing Hiwonder Board Controller...")

    try:
        board = HiwonderBoardController(port="/dev/ttyUSB0")

        # Get board info
        print("\n1. Getting board information...")
        info = board.get_board_info()
        if info:
            print(f"   Board Type: {info['board_type']}")
            print(f"   Firmware: {info['firmware_version']}")
        else:
            print("   Could not read board info")

        # Check voltage
        print("\n2. Reading board voltage...")
        voltage = board.read_board_voltage()
        if voltage:
            print(f"   Voltage: {voltage:.2f}V")
        else:
            print("   Could not read voltage")

        # Check power status
        print("\n3. Checking servo power status...")
        power = board.get_servo_power()
        if power is not None:
            print(f"   Servo power: {'ON' if power else 'OFF'}")
        else:
            print("   Could not read power status")

        # Test multi-servo move (example)
        print("\n4. Testing multi-servo move command...")
        print("   Moving servos 1, 2, 3 to center position (500)")
        board.multi_servo_move([
            (1, 500, 1000),  # Servo 1 to pos 500 in 1000ms
            (2, 500, 1000),  # Servo 2 to pos 500 in 1000ms
            (3, 500, 1000),  # Servo 3 to pos 500 in 1000ms
        ])
        print("   Command sent")

        board.close()

    except Exception as e:
        print(f"Error: {e}")
