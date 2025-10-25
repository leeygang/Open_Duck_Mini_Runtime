"""
Configure Hiwonder servo ID and test basic functionality

This script helps you:
1. Scan and find all connected servos
2. Read current servo ID and status
3. Change servo ID if needed
4. Test movement
"""

import argparse
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.hiwonder_hwi import HiwonderServo

DEFAULT_ID = 1  # Brand new Hiwonder servos have ID 1

parser = argparse.ArgumentParser(
    description="Configure Hiwonder servo ID",
    epilog="Example usage:\n"
           "  Read current servo ID:     python3 configure_hiwonder_motor.py --port /dev/serial0\n"
           "  Change ID to 100:          python3 configure_hiwonder_motor.py --port /dev/serial0 --id 100\n"
           "  Change specific servo ID:  python3 configure_hiwonder_motor.py --port /dev/serial0 --current-id 1 --id 100",
    formatter_class=argparse.RawDescriptionHelpFormatter
)
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
    help="New ID to assign to the servo (1-253). If not provided, will only read current ID.",
    type=int,
    default=None,
)
parser.add_argument(
    "--current-id",
    help="Current ID of the servo (if known). Will scan if not provided.",
    type=int,
    default=None,
)
parser.add_argument(
    "--scan-range",
    help="Max ID to scan when searching for servos (default 20 for quick scan, use 253 for full scan)",
    type=int,
    default=20,
)

args = parser.parse_args()

if args.id is not None and (args.id < 1 or args.id > 253):
    print("Error: ID must be between 1 and 253")
    exit(1)

print("=== Hiwonder Servo Configuration ===")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baudrate}")
if args.id is not None:
    print(f"Target ID: {args.id}")
else:
    print("Mode: Read current servo ID")
print()

try:
    servo = HiwonderServo(port=args.port, baudrate=args.baudrate)
except Exception as e:
    print(f"Failed to open serial port: {e}")
    print("Check that the port is correct and not in use.")
    exit(1)

current_id = args.current_id

# Always scan first to show all connected servos
print(f"Scanning for servos (IDs 1-{args.scan_range})...")
found_servos = servo.scan_servos(max_id=args.scan_range)

if not found_servos:
    print("✗ No servos found! Check connections and power.")
    print()
    print("Troubleshooting:")
    print("  1. Verify servo control board has 6-8.4V power supply")
    print("  2. Check serial wiring (TX→RX, RX→TX, GND→GND)")
    print("  3. Try a different port (use 'ls /dev/tty* | grep USB')")
    print("  4. Increase scan range with --scan-range 253")
    servo.close()
    exit(1)

print(f"✓ Found {len(found_servos)} servo(s): {found_servos}")
print()

# If current ID not provided, determine which servo to work with
if current_id is None:
    if len(found_servos) == 1:
        current_id = found_servos[0]
        print(f"Automatically selected servo with ID {current_id}")
    else:
        print("Multiple servos found. Please specify which one to configure:")
        for idx, servo_id in enumerate(found_servos, 1):
            print(f"  {idx}. Servo ID {servo_id}")
        print()

        if args.id is None:
            # If just reading, show all servos
            print("Reading status of all servos...")
            current_id = None  # Will read all below
        else:
            # If changing ID, need to select one
            try:
                choice = input(f"Select servo (1-{len(found_servos)}) or use --current-id flag: ")
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(found_servos):
                    current_id = found_servos[choice_idx]
                else:
                    print("Invalid selection.")
                    servo.close()
                    exit(1)
            except (ValueError, KeyboardInterrupt):
                print("\nCancelled.")
                servo.close()
                exit(1)
else:
    if current_id not in found_servos:
        print(f"⚠ Warning: Specified ID {current_id} not found in scan results: {found_servos}")
        print("  Continuing anyway in case of scan issues...")
    else:
        print(f"Working with servo ID {current_id}")

print()

# If no specific servo selected and just reading, show all
if current_id is None and args.id is None:
    for servo_id in found_servos:
        print(f"--- Servo ID {servo_id} ---")
        pos = servo.read_position(servo_id)
        voltage = servo.read_voltage(servo_id)
        temp = servo.read_temperature(servo_id)
        offset = servo.read_angle_offset(servo_id)

        print(f"  Position: {pos}")
        print(f"  Voltage: {voltage/1000:.2f}V" if voltage else "  Voltage: N/A")
        print(f"  Temperature: {temp}°C" if temp else "  Temperature: N/A")
        print(f"  Angle offset: {offset}" if offset is not None else "  Angle offset: N/A")
        print()

    print("=== Scan Complete ===")
    print(f"Total servos found: {len(found_servos)}")
    print(f"Servo IDs: {found_servos}")
    servo.close()
    exit(0)

print(f"Current servo ID: {current_id}")
if args.id is not None:
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

# If --id not provided, just show current status and exit
if args.id is None:
    print("=== Current Servo Status ===")
    print(f"Servo ID: {current_id}")
    print(f"Position: {pos}")
    print(f"Voltage: {voltage/1000:.2f}V" if voltage else "Voltage: N/A")
    print(f"Temperature: {temp}°C" if temp else "Temperature: N/A")
    print(f"Angle offset: {offset}" if offset is not None else "Angle offset: N/A")
    print()
    print("To change the ID, run again with --id <new_id>")
    servo.close()
    exit(0)

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
