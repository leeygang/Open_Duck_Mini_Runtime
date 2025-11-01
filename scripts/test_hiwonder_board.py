#!/usr/bin/env python3
"""
Test script for Hiwonder Bus Servo Controller board commands

Tests the 4 board-level commands from the official protocol:
- CMD_SERVO_MOVE (0x03): Move multiple servos
- CMD_GET_BATTERY_VOLTAGE (0x0F): Read battery voltage
- CMD_MULT_SERVO_UNLOAD (0x14): Unload multiple servos
- CMD_MULT_SERVO_POS_READ (0x15): Read multiple servo positions
"""

import sys
import os
import time
import argparse

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController

parser = argparse.ArgumentParser(description="Test Hiwonder Bus Servo Controller board")
parser.add_argument(
    "--port",
    help="Serial port. Default is /dev/ttyUSB0",
    default="/dev/ttyUSB0",
)
parser.add_argument(
    "--baudrate",
    help="Baudrate (default 9600)",
    type=int,
    default=9600,
)
parser.add_argument(
    "--servo-ids",
    help="Comma-separated list of servo IDs to test (e.g., 1,2,3)",
    default="1,2,3",
)

args = parser.parse_args()

# Parse servo IDs
try:
    servo_ids = [int(x.strip()) for x in args.servo_ids.split(',')]
except ValueError:
    print(f"Error: Invalid servo IDs: {args.servo_ids}")
    print("Example: --servo-ids 1,2,3")
    sys.exit(1)

print("=== Hiwonder Bus Servo Controller Test ===")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baudrate}")
print(f"Testing servo IDs: {servo_ids}")
print()
print("Commands tested:")
print("  - CMD_SERVO_MOVE (0x03)")
print("  - CMD_GET_BATTERY_VOLTAGE (0x0F)")
print("  - CMD_MULT_SERVO_UNLOAD (0x14)")
print("  - CMD_MULT_SERVO_POS_READ (0x15)")
print()

try:
    board = HiwonderBoardController(port=args.port, baudrate=args.baudrate)

    # Test 1: Get battery voltage
    print("Test 1: CMD_GET_BATTERY_VOLTAGE (0x0F)")
    print("-" * 40)
    voltage = board.get_battery_voltage()
    if voltage is not None:
        print(f"✓ Battery voltage: {voltage:.2f}V")
        if voltage < 6.0:
            print("  ⚠ Warning: Voltage is low (should be 6-8.4V)")
        elif voltage > 8.5:
            print("  ⚠ Warning: Voltage is high (should be 6-8.4V)")
    else:
        print("✗ Could not read voltage")
        print("  Check:")
        print("  1. Board is powered on")
        print("  2. Serial connection is correct")
        print("  3. Board supports this command")
    print()

    # Test 2: Read servo positions
    print(f"Test 2: CMD_MULT_SERVO_POS_READ (0x15)")
    print("-" * 40)
    print(f"Reading positions of servos {servo_ids}...")
    positions = board.read_servo_positions(servo_ids)
    if positions:
        print(f"✓ Read {len(positions)} servo positions:")
        for servo_id, position in positions:
            print(f"  Servo {servo_id}: position {position}")
    else:
        print("✗ Could not read positions")
        print("  Check:")
        print("  1. Servos are connected and powered")
        print("  2. Servo IDs are correct")
        print("  3. External power supply (6-8.4V) is connected")
    print()

    # Test 3: Move servos (optional)
    print("Test 3: CMD_SERVO_MOVE (0x03) - optional")
    print("-" * 40)
    response = input("Do you want to test servo movement? (y/N): ").lower()
    if response == 'y':
        print(f"Moving servos {servo_ids} to center position (500) in 1000ms...")
        servo_commands = [(sid, 500) for sid in servo_ids]
        board.move_servos(servo_commands, 1000)
        print("✓ Command sent")

        time.sleep(1.5)

        # Read positions after movement
        print("\nReading positions after movement...")
        positions = board.read_servo_positions(servo_ids)
        if positions:
            for servo_id, position in positions:
                print(f"  Servo {servo_id}: position {position}")
                if abs(position - 500) < 20:
                    print(f"    ✓ Close to target (500)")
                else:
                    print(f"    ⚠ Differs from target (expected ~500, got {position})")
        else:
            print("  Could not read positions")
    else:
        print("Skipped movement test")
    print()

    # Test 4: Unload servos (optional)
    print("Test 4: CMD_MULT_SERVO_UNLOAD (0x14) - optional")
    print("-" * 40)
    response = input("Do you want to test servo unload (disable torque)? (y/N): ").lower()
    if response == 'y':
        print(f"Unloading servos {servo_ids}...")
        board.unload_servos(servo_ids)
        print("✓ Command sent")
        print("  Servos should now be movable manually (torque disabled)")
        print()

        input("Press Enter to continue (servos will remain unloaded)...")

        # Note: To re-enable torque, you would need to send a load command
        # or send a move command which will automatically re-enable torque
        print("\nNote: To re-enable torque, send a move command")
    else:
        print("Skipped unload test")
    print()

    # Test 5: Additional movement patterns (optional)
    if len(servo_ids) >= 2:
        print("Test 5: Multi-Servo Coordinated Movement - optional")
        print("-" * 40)
        response = input("Do you want to test coordinated movement patterns? (y/N): ").lower()
        if response == 'y':
            print("\nPattern 1: All servos to position 400")
            board.move_servos([(sid, 400, 1000) for sid in servo_ids])
            time.sleep(1.5)

            print("Pattern 2: All servos to position 600")
            board.move_servos([(sid, 600, 1000) for sid in servo_ids])
            time.sleep(1.5)

            print("Pattern 3: Back to center (500)")
            board.move_servos([(sid, 500, 1000) for sid in servo_ids])
            time.sleep(1.5)

            print("✓ Movement patterns complete")

            # Read final positions
            positions = board.read_servo_positions(servo_ids)
            if positions:
                print("\nFinal positions:")
                for servo_id, position in positions:
                    print(f"  Servo {servo_id}: {position}")
        else:
            print("Skipped coordinated movement test")
        print()

    print("=== Test Complete ===")
    print()
    print("Summary:")
    print(f"  Port: {args.port}")
    print(f"  Baudrate: {args.baudrate}")
    print(f"  Tested servos: {servo_ids}")
    print(f"  Battery voltage: {voltage:.2f}V" if voltage else "  Battery voltage: N/A")
    print()
    print("Commands tested:")
    print("  ✓ CMD_SERVO_MOVE (0x03) - Move multiple servos")
    print("  ✓ CMD_GET_BATTERY_VOLTAGE (0x0F) - Read battery voltage")
    print("  ✓ CMD_MULT_SERVO_UNLOAD (0x14) - Unload multiple servos")
    print("  ✓ CMD_MULT_SERVO_POS_READ (0x15) - Read multiple servo positions")

    board.close()

except KeyboardInterrupt:
    print("\n\nTest interrupted by user")
    sys.exit(0)
except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
