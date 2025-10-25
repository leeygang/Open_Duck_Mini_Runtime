#!/usr/bin/env python3
"""
Test script for Hiwonder Bus Servo Controller board commands

This script tests board-level commands (not direct servo commands)
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
    help="Baudrate (default 9600 for board commands)",
    type=int,
    default=9600,
)

args = parser.parse_args()

print("=== Hiwonder Bus Servo Controller Board Test ===")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baudrate}")
print()

try:
    board = HiwonderBoardController(port=args.port, baudrate=args.baudrate)

    # Test 1: Get board information
    print("Test 1: Get Board Information")
    print("-" * 40)
    info = board.get_board_info()
    if info:
        print(f"✓ Board Type: {info['board_type']}")
        print(f"✓ Firmware Version: {info['firmware_version']}")
    else:
        print("✗ Could not read board info")
        print("  Note: Board might not support this command")
    print()

    # Test 2: Read board voltage
    print("Test 2: Read Board Voltage")
    print("-" * 40)
    voltage = board.read_board_voltage()
    if voltage:
        print(f"✓ Board Voltage: {voltage:.2f}V")
        if voltage < 6.0:
            print("  ⚠ Warning: Voltage is low (should be 6-8.4V)")
        elif voltage > 8.5:
            print("  ⚠ Warning: Voltage is high (should be 6-8.4V)")
    else:
        print("✗ Could not read voltage")
        print("  Note: Board might not support this command")
    print()

    # Test 3: Check servo power status
    print("Test 3: Check Servo Power Status")
    print("-" * 40)
    power = board.get_servo_power()
    if power is not None:
        status = "ON" if power else "OFF"
        print(f"✓ Servo Power: {status}")
    else:
        print("✗ Could not read power status")
        print("  Note: Board might not support this command")
    print()

    # Test 4: Toggle servo power (optional, with user confirmation)
    print("Test 4: Servo Power Control (optional)")
    print("-" * 40)
    response = input("Do you want to test power control? (y/N): ").lower()
    if response == 'y':
        print("Turning servo power OFF...")
        if board.set_servo_power(False):
            print("✓ Power turned OFF")
            time.sleep(1)

            print("Turning servo power ON...")
            if board.set_servo_power(True):
                print("✓ Power turned ON")
            else:
                print("✗ Failed to turn power ON")
        else:
            print("✗ Failed to turn power OFF")
    else:
        print("Skipped power control test")
    print()

    # Test 5: Multi-servo move command
    print("Test 5: Multi-Servo Move Command (optional)")
    print("-" * 40)
    response = input("Do you want to test multi-servo move? (y/N): ").lower()
    if response == 'y':
        servo_ids = input("Enter servo IDs to test (comma-separated, e.g., 1,2,3): ")
        try:
            ids = [int(x.strip()) for x in servo_ids.split(',')]
            print(f"Moving servos {ids} to center position (500)...")

            commands = [(servo_id, 500, 1000) for servo_id in ids]
            board.multi_servo_move(commands)
            print("✓ Multi-servo move command sent")
            print("  (Check if servos moved)")
        except ValueError:
            print("✗ Invalid servo IDs")
    else:
        print("Skipped multi-servo move test")
    print()

    print("=== Test Complete ===")
    print()
    print("Summary:")
    print("  If commands returned 'Could not read', the board might:")
    print("  1. Not support these specific board commands")
    print("  2. Be acting as a transparent pass-through only")
    print("  3. Use a different protocol version")
    print()
    print("  In that case, use direct servo commands (hiwonder_hwi.py)")
    print("  which work with servos through the board.")

    board.close()

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
