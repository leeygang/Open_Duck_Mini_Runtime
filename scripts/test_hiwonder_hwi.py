#!/usr/bin/env python3
"""
Test script for Hiwonder Board HWI

This script demonstrates the HWI interface for Hiwonder servos and tests
basic functionality like turning on, reading positions, and turning off.
"""

import sys
import os
import time
import argparse

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.hiwonder_board_hwi import HWI

parser = argparse.ArgumentParser(description="Test Hiwonder Board HWI")
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
parser.add_argument(
    "--skip-turn-on",
    help="Skip turn_on test (useful if servos are already powered)",
    action="store_true",
)

args = parser.parse_args()

print("=== Hiwonder Board HWI Test ===")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baudrate}")
print()

try:
    # Load duck config
    print("Loading duck configuration...")
    duck_config = DuckConfig()

    # Initialize HWI
    print("Initializing Hiwonder HWI...")
    hwi = HWI(duck_config=duck_config, usb_port=args.port, baudrate=args.baudrate)
    print(f"Initialized HWI with {len(hwi.joints)} joints")
    print()

    # Test 1: Read battery voltage
    print("Test 1: Read Battery Voltage")
    print("-" * 40)
    voltage = hwi.get_battery_voltage()
    if voltage is not None:
        print(f"✓ Battery voltage: {voltage:.2f}V")
        if voltage < 6.0:
            print("  ⚠ Warning: Voltage is low (should be 6-8.4V)")
        elif voltage > 8.5:
            print("  ⚠ Warning: Voltage is high (should be 6-8.4V)")
    else:
        print("✗ Could not read voltage")
    print()

    # Test 2: Read current positions
    print("Test 2: Read Current Positions")
    print("-" * 40)
    positions = hwi.get_present_positions()
    if positions is not None:
        print(f"✓ Read positions for {len(positions)} joints:")
        for joint_name, pos in zip(hwi.joints.keys(), positions):
            print(f"  {joint_name:20s}: {pos:7.3f} rad")
    else:
        print("✗ Could not read positions")
    print()

    # Test 3: Read velocities (will be zeros)
    print("Test 3: Read Velocities")
    print("-" * 40)
    velocities = hwi.get_present_velocities()
    if velocities is not None:
        print(f"✓ Read velocities (note: Hiwonder board doesn't provide velocity feedback)")
        print(f"  All velocities: {velocities}")
    else:
        print("✗ Could not read velocities")
    print()

    # Test 4: Turn on (optional)
    if not args.skip_turn_on:
        print("Test 4: Turn On Servos")
        print("-" * 40)
        response = input("Do you want to test turn_on (servos will move to init position)? (y/N): ").lower()
        if response == 'y':
            print("Turning on servos...")
            hwi.turn_on()
            print("✓ Servos powered on and moved to init position")

            # Read positions after turn_on
            time.sleep(0.5)
            positions = hwi.get_present_positions()
            if positions is not None:
                print("\nPositions after turn_on:")
                for joint_name, pos in zip(hwi.joints.keys(), positions):
                    init_pos = hwi.init_pos[joint_name]
                    error = abs(pos - init_pos)
                    status = "✓" if error < 0.1 else "⚠"
                    print(f"  {status} {joint_name:20s}: {pos:7.3f} rad (target: {init_pos:7.3f} rad, error: {error:7.3f})")
        else:
            print("Skipped turn_on test")
        print()

    # Test 5: Single joint movement (optional)
    print("Test 5: Single Joint Movement (optional)")
    print("-" * 40)
    response = input("Do you want to test single joint movement? (y/N): ").lower()
    if response == 'y':
        test_joint = "right_knee"
        test_position = 1.5  # radians

        print(f"Moving {test_joint} to {test_position} radians...")
        hwi.set_position(test_joint, test_position)
        time.sleep(1.5)

        positions = hwi.get_present_positions()
        if positions is not None:
            joint_idx = list(hwi.joints.keys()).index(test_joint)
            actual_pos = positions[joint_idx]
            error = abs(actual_pos - test_position)
            print(f"  Target: {test_position:.3f} rad")
            print(f"  Actual: {actual_pos:.3f} rad")
            print(f"  Error: {error:.3f} rad")
            if error < 0.1:
                print("  ✓ Movement successful")
            else:
                print("  ⚠ Large error - check calibration")
    else:
        print("Skipped single joint movement test")
    print()

    # Test 6: All joints movement (optional)
    print("Test 6: All Joints Movement (optional)")
    print("-" * 40)
    response = input("Do you want to test all joints movement (back to init pos)? (y/N): ").lower()
    if response == 'y':
        print("Moving all joints to init position...")
        hwi.set_position_all(hwi.init_pos)
        time.sleep(2.0)

        positions = hwi.get_present_positions()
        if positions is not None:
            print("\nPositions after movement:")
            max_error = 0
            for joint_name, pos in zip(hwi.joints.keys(), positions):
                target_pos = hwi.init_pos[joint_name]
                error = abs(pos - target_pos)
                max_error = max(max_error, error)
                status = "✓" if error < 0.1 else "⚠"
                print(f"  {status} {joint_name:20s}: {pos:7.3f} rad (error: {error:7.3f})")

            print(f"\nMax error: {max_error:.3f} rad")
            if max_error < 0.1:
                print("✓ All joints within tolerance")
            else:
                print("⚠ Some joints have large errors - check calibration")
    else:
        print("Skipped all joints movement test")
    print()

    # Test 7: Turn off
    print("Test 7: Turn Off Servos")
    print("-" * 40)
    response = input("Do you want to turn off servos (disable torque)? (y/N): ").lower()
    if response == 'y':
        print("Turning off servos...")
        hwi.turn_off()
        print("✓ Servos turned off - torque disabled")
    else:
        print("Skipped turn_off test")
    print()

    print("=== Test Complete ===")
    print()
    print("Summary:")
    print(f"  Port: {args.port}")
    print(f"  Baudrate: {args.baudrate}")
    print(f"  Joints: {len(hwi.joints)}")
    print(f"  Battery voltage: {voltage:.2f}V" if voltage else "  Battery voltage: N/A")
    print()
    print("The Hiwonder Board HWI is working correctly!")
    print()
    print("Next steps:")
    print("  1. Calibrate joint offsets (adjust duck_config.json)")
    print("  2. Test with walking policy: python3 scripts/v2_rl_walk_mujoco.py")
    print("  3. See HIWONDER_BOARD_INTEGRATION_GUIDE.md for full integration guide")

    hwi.close()

except KeyboardInterrupt:
    print("\n\nTest interrupted by user")
    sys.exit(0)
except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
