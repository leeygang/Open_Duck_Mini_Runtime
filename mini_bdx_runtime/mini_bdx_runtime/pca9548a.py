"""
PCA9548A I2C Multiplexer Driver

This module provides control for the PCA9548A 8-channel I2C multiplexer/switch.
The PCA9548A allows multiple I2C devices with the same address to coexist on the same bus.

Hardware:
- 8 independent I2C channels (SD0-SD7)
- I2C address: 0x70 (default, configurable via A0-A2 pins)
- Operating voltage: 2.3V - 5.5V

Usage:
    mux = PCA9548A(bus=1, address=0x70)
    mux.select_channel(0)  # Select channel 0
    # Now communicate with device on channel 0
    mux.disable_all()  # Disable all channels
"""

from smbus2 import SMBus
from typing import Optional


class PCA9548A:
    """Driver for PCA9548A I2C multiplexer."""

    def __init__(self, bus: int = 1, address: int = 0x70):
        """
        Initialize PCA9548A multiplexer.

        Args:
            bus: I2C bus number (default 1 on Raspberry Pi)
            address: I2C address of PCA9548A (default 0x70)
        """
        self.bus_num = bus
        self.address = address
        self.smbus = SMBus(bus)
        self.current_channel: Optional[int] = None

        # Verify device is present
        try:
            self.get_selected_channels()
        except Exception as e:
            raise RuntimeError(
                f"PCA9548A not found at address 0x{address:02X} on bus {bus}. "
                f"Check connections and I2C configuration. Error: {e}"
            )

    def select_channel(self, channel: int) -> None:
        """
        Select a single channel on the multiplexer.

        Args:
            channel: Channel number (0-7)

        Raises:
            ValueError: If channel is not in range 0-7
        """
        if not 0 <= channel <= 7:
            raise ValueError(f"Channel must be 0-7, got {channel}")

        # Write channel bitmask (bit N = channel N)
        channel_mask = 1 << channel
        self.smbus.write_byte(self.address, channel_mask)
        self.current_channel = channel

    def select_multiple_channels(self, channels: list[int]) -> None:
        """
        Select multiple channels simultaneously.

        Args:
            channels: List of channel numbers (0-7)

        Raises:
            ValueError: If any channel is not in range 0-7
        """
        if any(ch < 0 or ch > 7 for ch in channels):
            raise ValueError(f"All channels must be 0-7, got {channels}")

        # Combine channel bitmasks
        channel_mask = sum(1 << ch for ch in channels)
        self.smbus.write_byte(self.address, channel_mask)
        self.current_channel = None  # Multiple channels active

    def disable_all(self) -> None:
        """Disable all channels (no channel selected)."""
        self.smbus.write_byte(self.address, 0x00)
        self.current_channel = None

    def get_selected_channels(self) -> list[int]:
        """
        Read which channels are currently selected.

        Returns:
            List of active channel numbers (0-7)
        """
        channel_mask = self.smbus.read_byte(self.address)
        return [i for i in range(8) if channel_mask & (1 << i)]

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disable all channels."""
        self.disable_all()
        self.smbus.close()

    def __del__(self):
        """Cleanup on deletion."""
        try:
            if hasattr(self, 'smbus'):
                self.smbus.close()
        except:
            pass


def scan_all_channels(bus: int = 1, mux_address: int = 0x70) -> dict[int, list[int]]:
    """
    Scan all channels of PCA9548A and detect I2C devices on each.

    Args:
        bus: I2C bus number
        mux_address: PCA9548A I2C address

    Returns:
        Dictionary mapping channel number to list of detected device addresses
    """
    mux = PCA9548A(bus=bus, address=mux_address)
    results = {}

    with SMBus(bus) as smbus:
        for channel in range(8):
            mux.select_channel(channel)
            devices = []

            # Scan common I2C address range (skip mux address itself)
            for addr in range(0x08, 0x78):
                if addr == mux_address:
                    continue
                try:
                    smbus.read_byte(addr)
                    devices.append(addr)
                except:
                    pass

            results[channel] = devices

    mux.disable_all()
    return results


if __name__ == "__main__":
    """Test PCA9548A multiplexer."""
    import sys

    print("PCA9548A Multiplexer Test")
    print("=" * 50)

    try:
        # Initialize multiplexer
        mux = PCA9548A(bus=1, address=0x70)
        print(f"✓ PCA9548A detected at address 0x70")

        # Show currently selected channels
        active = mux.get_selected_channels()
        print(f"Currently active channels: {active}")

        # Scan all channels
        print("\nScanning all channels...")
        results = scan_all_channels()

        for channel, devices in results.items():
            if devices:
                dev_str = ", ".join(f"0x{addr:02X}" for addr in devices)
                print(f"  Channel {channel}: {dev_str}")
            else:
                print(f"  Channel {channel}: (no devices)")

        # Disable all channels
        mux.disable_all()
        print("\n✓ All channels disabled")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)
