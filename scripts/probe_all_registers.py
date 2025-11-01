#!/usr/bin/env python3
"""
Read all registers from 0x00 to 0xFF to see if any return non-zero values.
This helps identify what chip we actually have.
"""

from smbus2 import SMBus
import sys

BUS = 1
IMU_ADDR = 0x68

def main():
    print("=" * 60)
    print(f"Probing ALL registers at address 0x{IMU_ADDR:02X}")
    print("=" * 60)

    non_zero_regs = []

    with SMBus(BUS) as bus:
        print("\nScanning registers 0x00 - 0xFF...\n")

        for reg in range(0x00, 0x100):
            try:
                val = bus.read_byte_data(IMU_ADDR, reg)
                if val != 0x00:
                    print(f"  Reg 0x{reg:02X} = 0x{val:02X}")
                    non_zero_regs.append((reg, val))
            except Exception as e:
                pass  # Ignore errors

        print("\n" + "=" * 60)
        if non_zero_regs:
            print(f"✓ Found {len(non_zero_regs)} non-zero registers:")
            for reg, val in non_zero_regs:
                print(f"    0x{reg:02X} = 0x{val:02X}")
        else:
            print("✗ ALL registers returned 0x00")
            print("\nThis suggests:")
            print("  - Device is completely unresponsive")
            print("  - Wrong chip")
            print("  - Missing critical configuration")
        print("=" * 60)

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
