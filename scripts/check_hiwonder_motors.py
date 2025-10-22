"""
Check all Hiwonder servos in the robot
Verifies connectivity and allows testing movement
"""

import time
import sys
import os
import traceback

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.hiwonder_hwi import HiwonderHWI
from mini_bdx_runtime.duck_config import DuckConfig

def main():
    print("=== Hiwonder Servo Checker ===")
    print()

    # Initialize
    try:
        print("Initializing hardware interface...")
        duck_config = DuckConfig()

        # Adjust port and baudrate as needed
        port = input("Enter serial port (default /dev/ttyUSB0): ").strip()
        if not port:
            port = "/dev/ttyUSB0"

        hwi = HiwonderHWI(duck_config=duck_config, usb_port=port, baudrate=115200)
        print("✓ Successfully connected!")
    except Exception as e:
        print(f"✗ Error connecting to hardware: {e}")
        print(traceback.format_exc())
        return

    print()

    # Scan for servos
    print("Scanning for servos...")
    found_ids = hwi.scan_servos()

    if not found_ids:
        print("No servos found! Check connections and power.")
        hwi.close()
        return

    print(f"Found {len(found_ids)} servos")
    print()

    # Check configured joints
    print("Checking configured joints...")
    unresponsive_joints = []

    for joint_name, joint_id in hwi.joints.items():
        print(f"Testing '{joint_name}' (ID {joint_id})...")
        try:
            pos_units = hwi.servo.read_position(joint_id)
            if pos_units is not None:
                pos_rad = hwi.position_to_radians(pos_units)
                print(f"  ✓ Position: {pos_units} units ({pos_rad:.3f} rad)")

                # Read additional info
                voltage = hwi.servo.read_voltage(joint_id)
                temp = hwi.servo.read_temperature(joint_id)
                if voltage:
                    print(f"  ✓ Voltage: {voltage/1000:.2f}V")
                if temp:
                    print(f"  ✓ Temperature: {temp}°C")
            else:
                print(f"  ✗ Failed to read position")
                unresponsive_joints.append((joint_name, joint_id))
        except Exception as e:
            print(f"  ✗ Error: {e}")
            unresponsive_joints.append((joint_name, joint_id))

        time.sleep(0.1)

    if unresponsive_joints:
        print()
        print("WARNING: Some joints are unresponsive:")
        for name, id in unresponsive_joints:
            print(f"  - {name} (ID {id})")

        response = input("\nContinue anyway? (y/N): ").lower()
        if response != 'y':
            print("Exiting...")
            hwi.close()
            return

    # Movement test
    print()
    print("=== Movement Test ===")
    input("Press Enter to begin movement test (or Ctrl+C to exit)...")

    for joint_name, joint_id in hwi.joints.items():
        if (joint_name, joint_id) in unresponsive_joints:
            continue

        print()
        print(f"Testing '{joint_name}' (ID {joint_id})")
        response = input("Test this joint? (Enter/y=yes, n=skip, q=quit): ").lower()

        if response == 'q':
            break
        if response == 'n':
            continue

        try:
            # Get current position
            current_pos = hwi.servo.read_position(joint_id)
            print(f"  Current position: {current_pos} units")

            # Move slightly
            test_pos = current_pos + 50 if current_pos < 950 else current_pos - 50
            print(f"  Moving to {test_pos} units...")
            hwi.servo.move_servo(joint_id, test_pos, duration=500)
            time.sleep(0.6)

            # Read new position
            new_pos = hwi.servo.read_position(joint_id)
            print(f"  New position: {new_pos} units")

            # Return to original
            print(f"  Returning to original position...")
            hwi.servo.move_servo(joint_id, current_pos, duration=500)
            time.sleep(0.6)

            print("  ✓ Movement test complete")

        except Exception as e:
            print(f"  ✗ Error: {e}")
            print(traceback.format_exc())

    # Turn off
    print()
    print("Turning off servos...")
    hwi.turn_off()
    hwi.close()
    print("Done!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
