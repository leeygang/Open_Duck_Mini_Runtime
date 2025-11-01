#!/usr/bin/env python3
"""
Attempt to wake up ICM45686 from sleep/reset state and read WHO_AM_I.
"""

from smbus2 import SMBus
import time
import sys

BUS = 1
IMU_ADDR = 0x68

# ICM45686 Registers
WHO_AM_I = 0x75
PWR_MGMT0 = 0x1F
SIGNAL_PATH_RESET = 0x02
DEVICE_CONFIG = 0x01

def read_register(bus, addr, reg):
    """Read a single register."""
    try:
        return bus.read_byte_data(addr, reg)
    except Exception as e:
        return None

def write_register(bus, addr, reg, value):
    """Write a single register."""
    try:
        bus.write_byte_data(addr, reg, value)
        return True
    except Exception as e:
        print(f"  Write error: {e}")
        return False

def main():
    print("=" * 60)
    print("ICM45686 Wake-Up and Initialization Sequence")
    print("=" * 60)

    with SMBus(BUS) as bus:
        # Initial read
        print(f"\n1. Initial WHO_AM_I read (register 0x75):")
        who_am_i = read_register(bus, IMU_ADDR, WHO_AM_I)
        if who_am_i is not None:
            print(f"   WHO_AM_I = 0x{who_am_i:02X} (expect 0xE9)")
        else:
            print(f"   Read failed")

        # Try software reset
        print(f"\n2. Attempting software reset (reg 0x02 = 0x02):")
        if write_register(bus, IMU_ADDR, SIGNAL_PATH_RESET, 0x02):
            print(f"   Reset command sent")
            time.sleep(0.1)  # Wait 100ms

            who_am_i = read_register(bus, IMU_ADDR, WHO_AM_I)
            if who_am_i is not None:
                print(f"   WHO_AM_I after reset = 0x{who_am_i:02X}")

        # Try waking up device
        print(f"\n3. Attempting to wake device (PWR_MGMT0 = 0x0F):")
        if write_register(bus, IMU_ADDR, PWR_MGMT0, 0x0F):
            print(f"   Wake command sent (enable gyro + accel)")
            time.sleep(0.05)  # Wait 50ms

            who_am_i = read_register(bus, IMU_ADDR, WHO_AM_I)
            if who_am_i is not None:
                print(f"   WHO_AM_I after wake = 0x{who_am_i:02X}")

        # Try device config
        print(f"\n4. Attempting DEVICE_CONFIG (reg 0x01 = 0x01):")
        if write_register(bus, IMU_ADDR, DEVICE_CONFIG, 0x01):
            print(f"   Config command sent")
            time.sleep(0.1)

            who_am_i = read_register(bus, IMU_ADDR, WHO_AM_I)
            if who_am_i is not None:
                print(f"   WHO_AM_I after config = 0x{who_am_i:02X}")

        # Read multiple registers
        print(f"\n5. Reading multiple registers:")
        for reg in [0x00, 0x01, 0x02, 0x0F, 0x1F, 0x75]:
            val = read_register(bus, IMU_ADDR, reg)
            if val is not None:
                print(f"   Reg 0x{reg:02X} = 0x{val:02X}")

        # Check if device might be at alternate address 0x69
        print(f"\n6. Checking alternate address 0x69:")
        try:
            who_am_i_alt = read_register(bus, 0x69, WHO_AM_I)
            if who_am_i_alt is not None:
                print(f"   WHO_AM_I at 0x69 = 0x{who_am_i_alt:02X}")
            else:
                print(f"   No device at 0x69")
        except:
            print(f"   No device at 0x69")

        print("\n" + "=" * 60)
        print("Summary:")
        final_who = read_register(bus, IMU_ADDR, WHO_AM_I)
        if final_who == 0xE9:
            print("✓ ICM45686 detected and responding!")
        elif final_who == 0x00:
            print("✗ Device still returning 0x00")
            print("\nPossible issues:")
            print("  - Wrong chip (not ICM45686)")
            print("  - Defective chip")
            print("  - Requires different initialization")
            print("  - VDDIO not connected (separate digital I/O supply)")
        else:
            print(f"✗ Unexpected WHO_AM_I value: 0x{final_who:02X}")
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
