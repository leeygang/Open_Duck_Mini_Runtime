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

    class receivedResponse(object):
        def __init__(self, cmd : int):
            self.cmd = cmd
            self.data = list()

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


    def _send_command(self, command: int, params: List[int] = None) -> None:
        """
        Send command to board

        Args:
            command: Command byte
            params: List of parameter bytes
        """
        if params is None:
            params = []

        # Length = Params + 2
        length = 2 + len(params)

        # Build packet
        packet = self.HEADER + [length, command] + params
        print(f"   TX: {' '.join(f'{b:02X}' for b in packet)}")

        # Send
        self.serial.write(bytes(packet))
        self.serial.flush()

    def _read_response(self, timeout: Optional[float] = None) -> Optional[List[int]]:
        """
        Read response from board

        Args:
            timeout: Optional timeout override

        Returns:
            List of [head, length, command, param1, ..., paramN] or None if error
        """
        if timeout is not None:
            old_timeout = self.serial.timeout
            self.serial.timeout = timeout

        try:
            # Read header
            while(True):
                h = self.serial.read(1)
                if (len(h) == 0):
                    print("Timeout waiting for header")
                    return None
                elif h != b'\x55':
                    print(f"Waiting for header... but read ({h:02X}), continue")
                    continue
                else:
                    break
            # read first header.
            header = self.serial.read(1) 
            if (header != b'\x55'):
                print(f"Header error: expected 2nd 0x55, got: {header:02X}")
                return None

            # length, command
            metadata = self.serial.read(2)
            if len(metadata) != 2:
                print("Failed to read length and command byte")
                return None
            length, command = metadata
            # Read parameters
            remaining = length - 2
            data = self.serial.read(remaining)
            if len(data) != remaining:
                print(f"Failed to read data bytes: expected {remaining}, got: {' '.join(f'{b:02X}' for b in data)}")
                return None
            ret = [length, command] + list(data)
            print(f"Response: {' '.join(f'{b:02X}' for b in ret)}")
            return ret
        finally:
            if timeout is not None:
                self.serial.timeout = old_timeout

    # Command implementations

    def move_servos(self, servo_commands: List[Tuple[int, int]], time_ms: int) -> bool:
        """
        Move multiple servos simultaneously (CMD_SERVO_MOVE = 0x03)

        Frame format:
        [0x55][0x55][Length][0x03][number of Servos][Time low byte][Time high byte][ID1][Pos1_L][Pos1_H]...

        Args:
            servo_commands: List of (servo_id, position, time_ms) tuples
                           - servo_id: 1-253
                           - position: 0-1000
                           - time_ms: movement time in milliseconds

        Returns:
            True if command sent successfully

        Example:
            controller.move_servos([
                (1, 500),  # Servo 1 to position 500 in 1000ms
                (2, 600),  # Servo 2 to position 600 in 1000ms
            ], 1000)
        """
        # Build parameter list: count + (id + pos_low + pos_high + time_low + time_high) * count
        params = [len(servo_commands)]
        print(f"time_L {(time_ms & 0xFF) : 02X}, time_H: {((time_ms >> 8) & 0xFF):02X}")
        params.append(time_ms & 0xFF)            # Time low byte
        params.append((time_ms >> 8) & 0xFF)     # Time high byte
        for servo_id, position in servo_commands:
            params.append(servo_id)
            params.append(position & 0xFF)           # Position low byte
            params.append((position >> 8) & 0xFF)    # Position high byte
            print(f"servo id {servo_id : 02X}, pos_L: {(position & 0xFF) : 02X} pos_H: {((position >> 8) & 0xFF):02X}")

        self._send_command(self.CMD_SERVO_MOVE, params)
        return True

    def get_battery_voltage(self) -> Optional[float]:
        """
        Read board battery voltage (CMD_GET_BATTERY_VOLTAGE = 0x0F)

        Frame format:
        Send: [0x55][0x55][0x02][0x0F]
        Receive: [0x55][0x55][0x04][0x0F][Voltage_L][Voltage_H]

        Returns:
            Voltage in volts, or None if error

        Example:
            voltage = controller.get_battery_voltage()
            print(f"Battery: {voltage:.2f}V")
        """
        self._send_command(self.CMD_GET_BATTERY_VOLTAGE)
        response = self._read_response(timeout=1.0)

        if response and response[1] == self.CMD_GET_BATTERY_VOLTAGE and response[0] == 4:
            # Response: [length, command, voltage_low, voltage_high]
            # Voltage in millivolts (little-endian)
            voltage_mv = response[2] | (response[3] << 8)
            return voltage_mv / 1000.0
        return None

    def unload_servos(self, servo_ids: List[int]) -> bool:
        """
        Unload (disable torque) multiple servos (CMD_MULT_SERVO_UNLOAD = 0x14)

        Frame format:
        [0x55][0x55][Length][0x14][Number of Servos][ID1][ID2]...[IDn]

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
        Send: [0x55][0x55][Length][0x15][Count][ID1][ID2]...[IDn]
        Receive: [0x55][0x55][0xFE][Length][0x15][Count][ID1][Pos1_L][Pos1_H][ID2][Pos2_L][Pos2_H]...

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
