#!/usr/bin/env python3
"""
Check if ICM45686 uses a register bank system like ICM-20948.
Try switching banks and reading WHO_AM_I and data registers.
"""

from smbus2 import SMBus
import time

BUS = 1
IMU_ADDR = 0x68
MUX_ADDR = 0x70
MUX_CHANNEL = 0

def rw_reg(reg, val=None):
    """Read or write register through multiplexer."""
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
        if val is None:
            return bus.read_byte_data(IMU_ADDR, reg)
        else:
            bus.write_byte_data(IMU_ADDR, reg, val)

def read_block(reg, length):
    """Read block through multiplexer."""
    with SMBus(BUS) as bus:
        bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
        return bytes(bus.read_i2c_block_data(IMU_ADDR, reg, length))

def main():
    print("=" * 70)
    print("ICM45686 Bank System / Register Map Investigation")
    print("=" * 70)

    # Common bank select register addresses
    bank_select_regs = [0x7F, 0x76, 0x7E, 0x00]

    print("\n1. Checking for bank selection system...")
    for bank_reg in bank_select_regs:
        print(f"\n   Trying REG_BANK_SEL at 0x{bank_reg:02X}:")

        # Try reading current bank
        try:
            current = rw_reg(bank_reg)
            print(f"     Current value: 0x{current:02X}")
        except:
            print(f"     Cannot read")
            continue

        # Try switching to different banks
        for bank in range(4):
            try:
                rw_reg(bank_reg, bank << 4)  # Banks usually at bits 4-5
                time.sleep(0.01)
                who = rw_reg(0x00)
                print(f"     Bank {bank}: WHO_AM_I(0x00) = 0x{who:02X}")
            except Exception as e:
                print(f"     Bank {bank}: Error")

    # Check if MREG (Memory Register) system like newer InvenSense chips
    print("\n2. Checking MREG system (newer InvenSense chips)...")
    print("   MREG uses IPREG_SYS registers to access extended memory")

    # Try reading IPREG_SYS1 and IPREG_SYS2 (if they exist)
    for reg in [0x2E, 0x2F, 0x47, 0x48]:
        try:
            val = rw_reg(reg)
            print(f"     Reg 0x{reg:02X} = 0x{val:02X}")
        except:
            pass

    # Check INT_CONFIG and INT_STATUS registers
    print("\n3. Checking interrupt/status registers...")
    status_regs = {
        0x1A: "INT_STATUS",
        0x1B: "INT_STATUS2",
        0x1C: "INT_STATUS3",
        0x1D: "FIFO_COUNTH",
        0x1E: "FIFO_COUNTL",
        0x35: "INT_SOURCE0",
        0x38: "FIFO_CONFIG",
        0x39: "FIFO_CONFIG1",
    }

    for reg, name in status_regs.items():
        try:
            val = rw_reg(reg)
            print(f"     0x{reg:02X} {name:20s} = 0x{val:02X} ({val:3d})")
        except:
            pass

    # Try triggering a single measurement
    print("\n4. Trying to trigger single measurement...")

    # Initialize again
    rw_reg(0x1F, 0x0F)  # PWR_MGMT0 - enable sensors
    time.sleep(0.1)

    # Try writing to potential trigger registers
    trigger_attempts = [
        (0x03, 0x01, "REG 0x03"),  # Might be ODR_ALIGN or similar
        (0x38, 0x40, "FIFO_CONFIG"),
        (0x1A, 0xFF, "INT_CONFIG"),
    ]

    for reg, val, name in trigger_attempts:
        print(f"   Writing 0x{val:02X} to {name} (0x{reg:02X})...")
        try:
            rw_reg(reg, val)
            time.sleep(0.1)

            # Try reading data
            data = read_block(0x09, 14)
            print(f"     Data after: {' '.join(f'{b:02X}' for b in data[:6])}")
        except Exception as e:
            print(f"     Error: {e}")

    print("\n" + "=" * 70)
    print("Suggestions:")
    print("  1. Search for 'ICM45686 v1.2 datasheet' or 'ICM45686 register map'")
    print("  2. Check if seller provided documentation")
    print("  3. Try contacting the Taobao seller for register documentation")
    print("  4. This might be a non-standard variant")
    print("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
