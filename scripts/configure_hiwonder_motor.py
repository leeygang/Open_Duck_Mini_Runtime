"""
Configure Hiwonder servo ID and test basic functionality

This script helps you:
1. Find a servo (scans IDs if needed)
2. Change its ID
3. Test movement
"""

import argparse
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.hiwonder_hwi import HiwonderServo

DEFAULT_ID = 1  # Brand new Hiwonder servos have ID 1

parser = argparse.ArgumentParser(description="Configure Hiwonder servo ID")
parser.add_argument(
    "--port",
    help="Serial port. Default is /dev/ttyUSB0. Use 'ls /dev/tty* | grep USB' to find it.",
    default="/dev/ttyUSB0",
)
parser.add_argument(
    "--baudrate",
    help="Baudrate (default 115200 for HTD-45H)",
    type=int,
    default=115200,
)
parser.add_argument(
    "--id",
    help="New ID to assign to the servo (1-253)",
    type=int,
    required=True,
)
parser.add_argument(
    "--current-id",
    help="Current ID of the servo (if known). Will scan if not provided.",
    type=int,
    default=None,
)

args = parser.parse_args()

if args.id < 1 or args.id > 253:
    print("Error: ID must be between 1 and 253")
    exit(1)

print("=== Hiwonder Servo Configuration ===")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baudrate}")
print(f"Target ID: {args.id}")
print()

try:
    servo = HiwonderServo(port=args.port, baudrate=args.baudrate)
except Exception as e:
    print(f"Failed to open serial port: {e}")
    print("Check that the port is correct and not in use.")
    exit(1)

current_id = args.current_id

# Try to find the servo
if current_id is None:
    print("No current ID provided. Trying default ID (1)...")
    pos = servo.read_position(DEFAULT_ID)
    if pos is not None:
        current_id = DEFAULT_ID
        print(f"Found servo with ID {current_id}")
    else:
        print(f"No servo found at ID {DEFAULT_ID}. Scanning...")
        found = servo.scan_servos(max_id=20)  # Quick scan first 20 IDs
        if found:
            current_id = found[0]
            print(f"Found servo with ID {current_id}")
        else:
            print("No servos found! Check connections and power.")
            servo.close()
            exit(1)

print()
print(f"Current servo ID: {current_id}")
print(f"New servo ID: {args.id}")
print()

# Read current status
print("Reading servo status...")
pos = servo.read_position(current_id)
voltage = servo.read_voltage(current_id)
temp = servo.read_temperature(current_id)
offset = servo.read_angle_offset(current_id)

print(f"  Position: {pos}")
print(f"  Voltage: {voltage/1000:.2f}V" if voltage else "  Voltage: N/A")
print(f"  Temperature: {temp}°C" if temp else "  Temperature: N/A")
print(f"  Angle offset: {offset}" if offset is not None else "  Angle offset: N/A")
print()

# Test movement before changing ID
print("Testing servo movement...")
test_positions = [400, 600, 500]  # Left, right, center
for test_pos in test_positions:
    print(f"  Moving to position {test_pos}...")
    servo.move_servo(current_id, test_pos, duration=500)
    time.sleep(0.6)
    actual = servo.read_position(current_id)
    print(f"    Actual position: {actual}")

print()

# Confirm ID change
if current_id != args.id:
    response = input(f"Change servo ID from {current_id} to {args.id}? (y/N): ").lower()
    if response != 'y':
        print("ID change cancelled.")
        servo.close()
        exit(0)

    print(f"Changing ID from {current_id} to {args.id}...")
    servo.set_servo_id(current_id, args.id)
    time.sleep(0.5)

    # Verify the change
    print("Verifying new ID...")
    new_pos = servo.read_position(args.id)
    if new_pos is not None:
        print(f"✓ Successfully changed ID to {args.id}")
        print(f"  Position reading: {new_pos}")
        current_id = args.id
    else:
        print("✗ Failed to verify new ID. The change may not have worked.")
        servo.close()
        exit(1)
else:
    print(f"Servo already has ID {args.id}")

print()

# Test movement with new ID
print("Testing servo with new ID...")
servo.move_servo(current_id, 500, duration=500)
time.sleep(0.6)
final_pos = servo.read_position(current_id)
print(f"Final position: {final_pos}")

print()
print("=== Configuration Complete ===")
print(f"Servo ID: {current_id}")
print(f"Position: {final_pos}")
print()
print("You can now add this servo to your robot configuration.")

servo.close()
