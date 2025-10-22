"""
Find angle offsets for Hiwonder servos

This script helps you calibrate each servo by:
1. Moving servo to its current zero position
2. Letting you manually adjust it to the mechanically correct zero
3. Calculating and applying the offset
4. Saving offsets to duck_config.json
"""

import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.hiwonder_hwi import HiwonderHWI
from mini_bdx_runtime.duck_config import DuckConfig

print("=== Hiwonder Servo Offset Calibration ===")
print()
print("WARNING: This script will move servos. Make sure the robot is safe to move!")
print()

response = input("Continue? (y/N): ").lower()
if response != 'y':
    print("Cancelled.")
    exit(0)

# Get port
port = input("\nEnter serial port (default /dev/ttyUSB0): ").strip()
if not port:
    port = "/dev/ttyUSB0"

# Initialize
dummy_config = DuckConfig(config_json_path=None, ignore_default=True)
hwi = HiwonderHWI(dummy_config, usb_port=port)

# Turn on servos
print("\nTurning on servos...")
hwi.turn_on()
time.sleep(1)

# Move all to center position (500)
print("Moving all servos to center position (500)...")
for joint_name, servo_id in hwi.joints.items():
    hwi.servo.move_servo(servo_id, 500, duration=1000)
time.sleep(1.5)

print()
print("=== Calibration Process ===")
print()

# Store offsets
calculated_offsets = {}

try:
    for joint_name, servo_id in hwi.joints.items():
        print(f"\n=== Calibrating '{joint_name}' (ID {servo_id}) ===")

        while True:
            response = input("Calibrate this servo? (Enter/y=yes, s=skip): ").lower()

            if response == 's':
                print("Skipped")
                break

            # Move to center
            print("Moving to center position (500)...")
            hwi.servo.move_servo(servo_id, 500, duration=500)
            time.sleep(0.6)

            # Read current position
            current_pos = hwi.servo.read_position(servo_id)
            print(f"Current position: {current_pos}")

            # Turn off torque for manual adjustment
            print("Turning off torque...")
            hwi.servo.unload_servo(servo_id)
            print()
            print(">>> Manually move the servo to the CORRECT ZERO POSITION <<<")
            print(">>> Press Enter when done <<<")
            input()

            # Read new position
            new_pos = hwi.servo.read_position(servo_id)
            print(f"New position: {new_pos}")

            # Calculate offset
            # Hiwonder offset is in units, where each unit ≈ 0.24°
            # Offset range: -125 to +125
            raw_offset = new_pos - current_pos

            # Convert to offset value (clamp to -125 to 125)
            offset = max(-125, min(125, raw_offset))

            print(f"Calculated offset: {offset}")

            # Apply offset
            print("Applying offset...")
            hwi.servo.load_servo(servo_id)
            time.sleep(0.2)
            hwi.servo.set_angle_offset(servo_id, offset)
            time.sleep(0.2)

            # Test: move to center with offset applied
            print("Testing with offset applied...")
            hwi.servo.move_servo(servo_id, 500, duration=500)
            time.sleep(0.6)

            # Verify
            verify_pos = hwi.servo.read_position(servo_id)
            print(f"Position after offset: {verify_pos}")

            response = input("Does this look correct? (y=yes, n=try again): ").lower()
            if response == 'y':
                calculated_offsets[joint_name] = offset
                print(f"✓ Offset saved: {offset}")
                break
            else:
                # Reset offset
                print("Resetting offset...")
                hwi.servo.set_angle_offset(servo_id, 0)
                time.sleep(0.2)

    print()
    print("=== Calibration Complete ===")
    print()
    print("Calculated offsets:")
    for joint_name, offset in calculated_offsets.items():
        print(f"  {joint_name}: {offset}")

    print()
    print("NOTE: Hiwonder offsets are stored in the servo's EEPROM.")
    print("The offsets are already applied to the servos.")
    print()
    print("For duck_config.json, you can add these as software offsets if needed:")
    print("(converted to radians)")
    print()
    print('"joints_offsets": {')
    for joint_name, offset in calculated_offsets.items():
        # Convert offset units to radians (rough approximation)
        # Hiwonder: 1000 units = 240° = 4.189 rad
        # So 1 unit ≈ 0.004189 rad
        offset_rad = offset * 0.004189
        print(f'    "{joint_name}": {offset_rad:.4f},')
    print('}')

except KeyboardInterrupt:
    print("\n\nCalibration interrupted!")
finally:
    print("\nTurning off servos...")
    hwi.turn_off()
    hwi.close()
    print("Done!")
