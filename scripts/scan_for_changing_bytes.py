#!/usr/bin/env python3
"""
Scan for registers that change when sensor moves.
Reads individual bytes (not block reads) to avoid address auto-increment issues.
"""

from smbus2 import SMBus
import time
import sys

BUS = 1
IMU_ADDR = 0x68
MUX_ADDR = 0x70
MUX_CHANNEL = 0

def read_register(bus, reg):
    """Read single register through multiplexer."""
    bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
    return bus.read_byte_data(IMU_ADDR, reg)

def main():
    print("=" * 70)
    print("Scanning for changing registers (move sensor NOW!)")
    print("=" * 70)

    with SMBus(BUS) as bus:
        # Take initial snapshot
        print("\n1. Reading initial snapshot (3 seconds)...")
        initial = {}
        for i in range(3):
            for reg in range(0x00, 0x80):
                try:
                    val = read_register(bus, reg)
                    if reg not in initial:
                        initial[reg] = []
                    initial[reg].append(val)
                except:
                    pass
            time.sleep(0.01)

        # Read while moving
        print("2. Reading while MOVING sensor (10 seconds - MOVE IT!)...")
        moving = {}
        for i in range(100):
            for reg in range(0x00, 0x80):
                try:
                    val = read_register(bus, reg)
                    if reg not in moving:
                        moving[reg] = []
                    moving[reg].append(val)
                except:
                    pass
            time.sleep(0.01)

        # Find registers that changed significantly
        print("\n" + "=" * 70)
        print("Registers with significant variation:")
        print("=" * 70)
        print(f"{'Reg':<6} {'Init Min':<10} {'Init Max':<10} {'Move Min':<10} {'Move Max':<10} {'Range':<10}")
        print("-" * 70)

        changing_regs = []
        for reg in range(0x00, 0x80):
            if reg in initial and reg in moving:
                init_min = min(initial[reg])
                init_max = max(initial[reg])
                move_min = min(moving[reg])
                move_max = max(moving[reg])
                total_range = max(move_max, init_max) - min(move_min, init_min)

                # Consider it changing if range > 10
                if total_range > 10:
                    print(f"0x{reg:02X}    {init_min:<10} {init_max:<10} {move_min:<10} {move_max:<10} {total_range:<10}")
                    changing_regs.append(reg)

        if changing_regs:
            print("\n" + "=" * 70)
            print("Likely sensor data register blocks:")
            print("=" * 70)

            # Find consecutive blocks
            blocks = []
            if changing_regs:
                current = [changing_regs[0]]
                for reg in changing_regs[1:]:
                    if reg == current[-1] + 1 or reg == current[-1] + 2:  # Allow 1-2 gap
                        current.append(reg)
                    else:
                        if len(current) >= 2:
                            blocks.append(current)
                        current = [reg]
                if len(current) >= 2:
                    blocks.append(current)

            for block in blocks:
                print(f"\n  Block: 0x{block[0]:02X} - 0x{block[-1]:02X} ({len(block)} registers)")
                if 6 <= len(block) <= 7:
                    print("    → Likely one 3-axis sensor (accel OR gyro)")
                elif 12 <= len(block) <= 14:
                    print("    → Likely both sensors (accel + gyro + maybe temp)")
        else:
            print("\n✗ No changing registers found!")
            print("  Sensor might not be outputting data continuously")

        print("\n" + "=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
        sys.exit(130)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
