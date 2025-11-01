#!/usr/bin/env python3
"""
Find the correct register addresses for accelerometer and gyroscope data.
Reads all registers while moving the sensor to identify which contain sensor data.
"""

from smbus2 import SMBus
import time
import sys

BUS = 1
IMU_ADDR = 0x68

def main():
    print("=" * 60)
    print("ICM45686 Register Discovery")
    print("=" * 60)
    print("\n⚠ Instructions:")
    print("  This script will read all registers multiple times.")
    print("  MOVE and ROTATE the sensor while the script runs.")
    print("  Registers that change are likely sensor data registers.\n")

    input("Press Enter to start (then move the sensor)...")

    with SMBus(BUS) as bus:
        # Read baseline
        print("\nReading baseline (keep still)...")
        baseline = {}
        for reg in range(0x00, 0x100):
            try:
                baseline[reg] = bus.read_byte_data(IMU_ADDR, reg)
            except:
                baseline[reg] = None

        time.sleep(1)

        # Read while moving
        print("Reading while moving (MOVE THE SENSOR NOW!)...")
        changing_regs = {}

        for sample in range(20):
            time.sleep(0.05)
            for reg in range(0x00, 0x100):
                try:
                    val = bus.read_byte_data(IMU_ADDR, reg)
                    if baseline[reg] is not None and val != baseline[reg]:
                        if reg not in changing_regs:
                            changing_regs[reg] = []
                        changing_regs[reg].append(val)
                except:
                    pass

        print("\n" + "=" * 60)
        print("Registers that changed during movement:")
        print("=" * 60)

        if changing_regs:
            print("\nReg  | Baseline | Samples (changing values)")
            print("-" * 60)
            for reg in sorted(changing_regs.keys()):
                samples = changing_regs[reg]
                print(f"0x{reg:02X} | 0x{baseline[reg]:02X}     | {len(samples)} changes, "
                      f"range: 0x{min(samples):02X}-0x{max(samples):02X}")

            # Identify likely sensor data registers (consecutive changing registers)
            print("\n" + "=" * 60)
            print("Analysis:")
            print("=" * 60)

            sorted_regs = sorted(changing_regs.keys())

            # Find consecutive blocks
            blocks = []
            if sorted_regs:
                current_block = [sorted_regs[0]]
                for reg in sorted_regs[1:]:
                    if reg == current_block[-1] + 1:
                        current_block.append(reg)
                    else:
                        if len(current_block) >= 2:
                            blocks.append(current_block)
                        current_block = [reg]
                if len(current_block) >= 2:
                    blocks.append(current_block)

            for block in blocks:
                print(f"\nConsecutive block: 0x{block[0]:02X} - 0x{block[-1]:02X} ({len(block)} registers)")
                if len(block) == 6:
                    print("  → Likely 3-axis sensor (6 bytes = 3 × 16-bit values)")
                    print(f"     Guess: X_MSB=0x{block[0]:02X}, X_LSB=0x{block[1]:02X}")
                    print(f"            Y_MSB=0x{block[2]:02X}, Y_LSB=0x{block[3]:02X}")
                    print(f"            Z_MSB=0x{block[4]:02X}, Z_LSB=0x{block[5]:02X}")
                elif len(block) == 12:
                    print("  → Likely BOTH accel + gyro (12 bytes = 6 × 16-bit)")
                    print(f"     Accel X_MSB=0x{block[0]:02X}, X_LSB=0x{block[1]:02X}")
                    print(f"           Y_MSB=0x{block[2]:02X}, Y_LSB=0x{block[3]:02X}")
                    print(f"           Z_MSB=0x{block[4]:02X}, Z_LSB=0x{block[5]:02X}")
                    print(f"     Gyro  X_MSB=0x{block[6]:02X}, X_LSB=0x{block[7]:02X}")
                    print(f"           Y_MSB=0x{block[8]:02X}, Y_LSB=0x{block[9]:02X}")
                    print(f"           Z_MSB=0x{block[10]:02X}, Z_LSB=0x{block[11]:02X}")
        else:
            print("\n✗ No registers changed!")
            print("  - Make sure you moved/rotated the sensor")
            print("  - Check if sensor is actually outputting data")

        print("\n" + "=" * 60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
