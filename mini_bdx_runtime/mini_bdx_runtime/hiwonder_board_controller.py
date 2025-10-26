"""
Hiwonder Bus Servo Controller Board Interface

This module provides control for the Hiwonder Bus Servo Controller board.
Implements only the commands defined in the official protocol document.

Protocol documentation: Bus Servo Controller Communication Protocol.pdf
"""

import serial
import time
from typing import List, Optional, Tuple


class HiwonderBoardController:
    """
    Hiwonder Bus Servo Controller Board interface

    Implements the 4 board-level commands from the official protocol:
    - CMD_SERVO_MOVE: Move multiple servos simultaneously
    - CMD_GET_BATTERY_VOLTAGE: Read board battery voltage
    - CMD_MULT_SERVO_UNLOAD: Unload (disable torque) multiple servos
    - CMD_MULT_SERVO_POS_READ: Read positions of multiple servos

    Protocol Frame Format:
    [0x55][0x55][ID][Length][Command][Param1]...[ParamN][Checksum]

    - Header: 0x55 0x55 (fixed)
    - ID: 0xFE for board commands
    - Length: Number of bytes from ID to Checksum (inclusive)
    - Command: Command code
    - Params: Variable length parameters
    - Checksum: ~(ID + Length + Command + Param1 + ... + ParamN) & 0xFF
    """

    # Board command codes from official protocol PDF
    CMD_SERVO_MOVE = 0x03              # Move multiple servos
    CMD_GET_BATTERY_VOLTAGE = 0x0F     # Get battery voltage
    CMD_MULT_SERVO_UNLOAD = 0x14       # Unload multiple servos
    CMD_MULT_SERVO_POS_READ = 0x15     # Read multiple servo positions

    HEADER = [0x55, 0x55]              # Protocol header
    BOARD_ID = 0xFE                    # Board ID for board commands

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
            # Flush any existing data
            self.serial.flushInput()
            self.serial.flushOutput()
            print(f"Hiwonder Board Controller initialized on {port} at {baudrate} baud")
        except Exception as e:
            raise Exception(f"Failed to open serial port {port}: {e}")

    def _checksum(self, data: List[int]) -> int:
        """
        Calculate checksum for protocol

        Checksum = ~(ID + Length + Command + Param1 + ... + ParamN) & 0xFF
        """
        return (~sum(data)) & 0xFF

    def _send_command(self, command: int, params: List[int] = None) -> None:
        """
        Send command to board

        Args:
            command: Command byte
            params: List of parameter bytes
        """
        if params is None:
            params = []

        # Length = ID + Length + Command + Params
        length = 3 + len(params)

        # Build packet
        packet = self.HEADER + [self.BOARD_ID, length, command] + params

        # Calculate checksum (from ID to end of params)
        checksum = self._checksum(packet[2:])
        packet.append(checksum)

        # Send
        self.serial.write(bytes(packet))
        self.serial.flush()

    def _read_response(self, timeout: Optional[float] = None) -> Optional[List[int]]:
        """
        Read response from board

        Args:
            timeout: Optional timeout override

        Returns:
            List of [board_id, command, param1, ..., paramN] or None if error
        """
        if timeout is not None:
            old_timeout = self.serial.timeout
            self.serial.timeout = timeout

        try:
            # Read header
            header = self.serial.read(2)
            if len(header) != 2 or list(header) != self.HEADER:
                return None

            # Read ID, length, command
            metadata = self.serial.read(3)
            if len(metadata) != 3:
                return None

            board_id, length, command = metadata

            # Read parameters and checksum
            # Length includes ID, Length, Command, Params, Checksum
            remaining = length - 3
            data = self.serial.read(remaining)
            if len(data) != remaining:
                return None

            # Verify checksum
            params = list(data[:-1])
            received_checksum = data[-1]
            calculated_checksum = self._checksum([board_id, length, command] + params)

            if received_checksum != calculated_checksum:
                print(f"Checksum error: expected {calculated_checksum:02X}, got {received_checksum:02X}")
                return None

            return [board_id, command] + params

        finally:
            if timeout is not None:
                self.serial.timeout = old_timeout

    # Command implementations

    def move_servos(self, servo_commands: List[Tuple[int, int, int]]) -> bool:
        """
        Move multiple servos simultaneously (CMD_SERVO_MOVE = 0x03)

        Frame format:
        [0x55][0x55][0xFE][Length][0x03][Count][ID1][Pos1_L][Pos1_H][Time1_L][Time1_H]...[Checksum]

        Args:
            servo_commands: List of (servo_id, position, time_ms) tuples
                           - servo_id: 1-253
                           - position: 0-1000
                           - time_ms: movement time in milliseconds

        Returns:
            True if command sent successfully

        Example:
            controller.move_servos([
                (1, 500, 1000),  # Servo 1 to position 500 in 1000ms
                (2, 600, 1000),  # Servo 2 to position 600 in 1000ms
            ])
        """
        # Build parameter list: count + (id + pos_low + pos_high + time_low + time_high) * count
        params = [len(servo_commands)]

        for servo_id, position, time_ms in servo_commands:
            params.append(servo_id)
            params.append(position & 0xFF)           # Position low byte
            params.append((position >> 8) & 0xFF)    # Position high byte
            params.append(time_ms & 0xFF)            # Time low byte
            params.append((time_ms >> 8) & 0xFF)     # Time high byte

        self._send_command(self.CMD_SERVO_MOVE, params)
        return True

    def get_battery_voltage(self) -> Optional[float]:
        """
        Read board battery voltage (CMD_GET_BATTERY_VOLTAGE = 0x0F)

        Frame format:
        Send: [0x55][0x55][0xFE][0x03][0x0F][Checksum]
        Receive: [0x55][0x55][0xFE][0x04][0x0F][Voltage_L][Voltage_H][Checksum]

        Returns:
            Voltage in volts, or None if error

        Example:
            voltage = controller.get_battery_voltage()
            print(f"Battery: {voltage:.2f}V")
        """
        self._send_command(self.CMD_GET_BATTERY_VOLTAGE)
        response = self._read_response(timeout=1.0)

        if response and len(response) >= 4:
            # Response: [board_id, command, voltage_low, voltage_high]
            # Voltage in millivolts (little-endian)
            voltage_mv = response[2] | (response[3] << 8)
            return voltage_mv / 1000.0
        return None

    def unload_servos(self, servo_ids: List[int]) -> bool:
        """
        Unload (disable torque) multiple servos (CMD_MULT_SERVO_UNLOAD = 0x14)

        Frame format:
        [0x55][0x55][0xFE][Length][0x14][Count][ID1][ID2]...[IDn][Checksum]

        Args:
            servo_ids: List of servo IDs to unload (1-253)

        Returns:
            True if command sent successfully

        Example:
            controller.unload_servos([1, 2, 3])  # Unload servos 1, 2, 3
        """
        params = [len(servo_ids)] + servo_ids
        self._send_command(self.CMD_MULT_SERVO_UNLOAD, params)
        return True

    def read_servo_positions(self, servo_ids: List[int]) -> Optional[List[Tuple[int, int]]]:
        """
        Read positions of multiple servos (CMD_MULT_SERVO_POS_READ = 0x15)

        Frame format:
        Send: [0x55][0x55][0xFE][Length][0x15][Count][ID1][ID2]...[IDn][Checksum]
        Receive: [0x55][0x55][0xFE][Length][0x15][Count][ID1][Pos1_L][Pos1_H][ID2][Pos2_L][Pos2_H]...[Checksum]

        Args:
            servo_ids: List of servo IDs to read (1-253)

        Returns:
            List of (servo_id, position) tuples, or None if error

        Example:
            positions = controller.read_servo_positions([1, 2, 3])
            for servo_id, position in positions:
                print(f"Servo {servo_id}: position {position}")
        """
        params = [len(servo_ids)] + servo_ids
        self._send_command(self.CMD_MULT_SERVO_POS_READ, params)
        response = self._read_response(timeout=1.0)

        if response and len(response) >= 3:
            # Response: [board_id, command, count, id1, pos1_low, pos1_high, ...]
            count = response[2]
            positions = []

            # Parse servo positions (3 bytes per servo: id + pos_low + pos_high)
            for i in range(count):
                offset = 3 + i * 3
                if offset + 2 < len(response):
                    servo_id = response[offset]
                    pos_low = response[offset + 1]
                    pos_high = response[offset + 2]
                    position = pos_low | (pos_high << 8)
                    positions.append((servo_id, position))

            return positions
        return None

    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Board controller connection closed")


# Example usage
if __name__ == "__main__":
    print("Testing Hiwonder Board Controller...")
    print("Commands: CMD_SERVO_MOVE, CMD_GET_BATTERY_VOLTAGE, CMD_MULT_SERVO_UNLOAD, CMD_MULT_SERVO_POS_READ")
    print()

    try:
        board = HiwonderBoardController(port="/dev/ttyUSB0")

        # Test 1: Get battery voltage
        print("1. Reading battery voltage...")
        voltage = board.get_battery_voltage()
        if voltage is not None:
            print(f"   Battery voltage: {voltage:.2f}V")
            if voltage < 6.0:
                print("   ⚠ Warning: Voltage is low (should be 6-8.4V)")
        else:
            print("   Could not read voltage")
        print()

        # Test 2: Move servos
        print("2. Moving servos 1, 2, 3 to center position (500)...")
        board.move_servos([
            (1, 500, 1000),
            (2, 500, 1000),
            (3, 500, 1000),
        ])
        print("   Command sent")
        time.sleep(1.5)
        print()

        # Test 3: Read servo positions
        print("3. Reading positions of servos 1, 2, 3...")
        positions = board.read_servo_positions([1, 2, 3])
        if positions:
            for servo_id, position in positions:
                print(f"   Servo {servo_id}: position {position}")
        else:
            print("   Could not read positions")
        print()

        # Test 4: Unload servos
        print("4. Unloading servos 1, 2, 3 (disabling torque)...")
        board.unload_servos([1, 2, 3])
        print("   Command sent - servos should be movable manually")
        print()

        board.close()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
