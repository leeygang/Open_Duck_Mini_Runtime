#!/usr/bin/env python3
"""
After initializing the sensor, scan ALL registers (0x00-0xFF) to find which change.
"""

from smbus2 import SMBus
import time
import sys

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

def main():
    print("=" * 70)
    print("Complete Register Scan After Initialization")
    print("=" * 70)

    # Initialize sensor
    print("\nInitializing sensor...")
    rw_reg(0x02, 0x02)  # Reset
    time.sleep(0.1)
    rw_reg(0x1F, 0x0F)  # Power on
    time.sleep(0.1)
    rw_reg(0x20, 0x62)  # Configure gyro
    time.sleep(0.1)
    print("✓ Sensor initialized\n")

    # Baseline scan
    print("Taking baseline snapshot (keep still for 2 seconds)...")
    baseline = {}
    for _ in range(20):
        for reg in range(0x00, 0x100):
            try:
                val = rw_reg(reg)
                if reg not in baseline:
                    baseline[reg] = []
                baseline[reg].append(val)
            except:
                pass
        time.sleep(0.01)

    # Movement scan
    print("Scanning while MOVING (10 seconds - SHAKE IT VIGOROUSLY!)...")
    moving = {}
    for _ in range(100):
        for reg in range(0x00, 0x100):
            try:
                val = rw_reg(reg)
                if reg not in moving:
                    moving[reg] = []
                moving[reg].append(val)
            except:
                pass
        time.sleep(0.01)

    # Analyze
    print("\n" + "=" * 70)
    print("Registers with variation > 20:")
    print("=" * 70)
    print(f"{'Reg':<8} {'Base Range':<15} {'Move Range':<15} {'Total Var':<10}")
    print("-" * 70)

    changing = []
    for reg in range(0x00, 0x100):
        if reg in baseline and reg in moving:
            b_min, b_max = min(baseline[reg]), max(baseline[reg])
            m_min, m_max = min(moving[reg]), max(moving[reg])
            total_var = max(m_max, b_max) - min(m_min, b_min)

            if total_var > 20:
                print(f"0x{reg:02X}     {b_min:3d}-{b_max:3d}          {m_min:3d}-{m_max:3d}          {total_var:3d}")
                changing.append(reg)

    if not changing:
        print("\n✗ NO CHANGING REGISTERS FOUND!")
        print("\nPossible causes:")
        print("  1. Sensor outputs data only when requested (not continuous)")
        print("  2. Need to enable data ready interrupts")
        print("  3. Data in FIFO buffer (need to read FIFO)")
        print("  4. Sensor in wrong mode")
        print("\nLet's check FIFO and status registers...")

        print("\n" + "=" * 70)
        print("Checking potential status/FIFO registers:")
        print("=" * 70)

        status_regs = [0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A]
        for reg in status_regs:
            try:
                val = rw_reg(reg)
                print(f"  Reg 0x{reg:02X} = 0x{val:02X} ({val:3d})")
            except:
                pass

    else:
        print(f"\n✓ Found {len(changing)} changing registers!")

        # Find consecutive blocks
        print("\n" + "=" * 70)
        print("Consecutive blocks (likely sensor data):")
        print("=" * 70)

        blocks = []
        if changing:
            current = [changing[0]]
            for reg in changing[1:]:
                if reg <= current[-1] + 2:  # Allow small gaps
                    current.append(reg)
                else:
                    if len(current) >= 4:
                        blocks.append(current)
                    current = [reg]
            if len(current) >= 4:
                blocks.append(current)

        for block in blocks:
            print(f"\n  0x{block[0]:02X} - 0x{block[-1]:02X} ({len(block)} registers)")
            if len(block) >= 6:
                print("    → Likely sensor data!")

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
