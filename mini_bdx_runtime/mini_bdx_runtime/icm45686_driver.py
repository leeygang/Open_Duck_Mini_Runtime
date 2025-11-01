"""
ICM45686 6-Axis IMU Driver

Low-level I2C driver for TDK InvenSense ICM45686 IMU.
Provides access to accelerometer, gyroscope, and temperature sensor.

Hardware:
- 3-axis accelerometer: ±2g, ±4g, ±8g, ±16g
- 3-axis gyroscope: ±250, ±500, ±1000, ±2000 dps
- I2C interface at 0x68 (AD0=GND) or 0x69 (AD0=VCC)
- Operating voltage: 1.71V - 3.6V

Register Map Reference: ICM-45686 Datasheet v1.0
"""

from smbus2 import SMBus
import time
import struct
from typing import Optional, Tuple
from dataclasses import dataclass

from mini_bdx_runtime.pca9548a import PCA9548A


# ICM45686 Register Map (Bank 0)
class ICM45686Registers:
    """ICM45686 register addresses."""
    # Device identification
    WHO_AM_I = 0x72  # Should return 0xE9

    # Power management
    PWR_MGMT0 = 0x1F  # Power management 0

    # Gyroscope configuration
    GYRO_CONFIG0 = 0x20  # Gyro ODR and full scale
    GYRO_CONFIG1 = 0x21  # Gyro filters

    # Accelerometer configuration
    ACCEL_CONFIG0 = 0x21  # Accel ODR and full scale
    ACCEL_CONFIG1 = 0x24  # Accel filters

    # Temperature sensor
    TEMP_DATA1 = 0x09  # Temperature MSB
    TEMP_DATA0 = 0x0A  # Temperature LSB

    # Accelerometer data
    ACCEL_DATA_X1 = 0x0B  # Accel X MSB
    ACCEL_DATA_X0 = 0x0C  # Accel X LSB
    ACCEL_DATA_Y1 = 0x0D  # Accel Y MSB
    ACCEL_DATA_Y0 = 0x0E  # Accel Y LSB
    ACCEL_DATA_Z1 = 0x0F  # Accel Z MSB
    ACCEL_DATA_Z0 = 0x10  # Accel Z LSB

    # Gyroscope data
    GYRO_DATA_X1 = 0x11  # Gyro X MSB
    GYRO_DATA_X0 = 0x12  # Gyro X LSB
    GYRO_DATA_Y1 = 0x13  # Gyro Y MSB
    GYRO_DATA_Y0 = 0x14  # Gyro Y LSB
    GYRO_DATA_Z1 = 0x15  # Gyro Z MSB
    GYRO_DATA_Z0 = 0x16  # Gyro Z LSB

    # Signal path reset
    SIGNAL_PATH_RESET = 0x02

    # FIFO
    FIFO_CONFIG1 = 0x28
    FIFO_DATA = 0x30


@dataclass
class IMUData:
    """Container for IMU sensor data."""
    accel_x: float  # m/s²
    accel_y: float  # m/s²
    accel_z: float  # m/s²
    gyro_x: float   # rad/s
    gyro_y: float   # rad/s
    gyro_z: float   # rad/s
    temperature: float  # °C
    timestamp: float  # seconds


class ICM45686Driver:
    """Low-level driver for ICM45686 IMU."""

    # Device ID
    DEVICE_ID = 0xE9

    # Default I2C addresses
    I2C_ADDR_DEFAULT = 0x68  # AD0 = GND
    I2C_ADDR_ALT = 0x69      # AD0 = VCC

    # Full scale ranges
    GYRO_FS_250DPS = 0
    GYRO_FS_500DPS = 1
    GYRO_FS_1000DPS = 2
    GYRO_FS_2000DPS = 3

    ACCEL_FS_2G = 0
    ACCEL_FS_4G = 1
    ACCEL_FS_8G = 2
    ACCEL_FS_16G = 3

    # Scale factors (LSB to physical units)
    GYRO_SCALE = {
        250: 250.0 / 32768.0,    # dps per LSB
        500: 500.0 / 32768.0,
        1000: 1000.0 / 32768.0,
        2000: 2000.0 / 32768.0,
    }

    ACCEL_SCALE = {
        2: 2.0 / 32768.0,     # g per LSB
        4: 4.0 / 32768.0,
        8: 8.0 / 32768.0,
        16: 16.0 / 32768.0,
    }

    GRAVITY = 9.80665  # m/s²
    DEG_TO_RAD = 3.14159265359 / 180.0

    def __init__(
        self,
        i2c_bus: int = 1,
        imu_address: int = I2C_ADDR_DEFAULT,
        mux_address: Optional[int] = None,
        mux_channel: Optional[int] = None,
        gyro_range: int = 1000,
        accel_range: int = 4,
    ):
        """
        Initialize ICM45686 driver.

        Args:
            i2c_bus: I2C bus number
            imu_address: ICM45686 I2C address (0x68 or 0x69)
            mux_address: PCA9548A multiplexer address (if used)
            mux_channel: PCA9548A channel number (0-7, if used)
            gyro_range: Gyroscope range in dps (250, 500, 1000, 2000)
            accel_range: Accelerometer range in g (2, 4, 8, 16)
        """
        self.bus_num = i2c_bus
        self.address = imu_address
        self.smbus = SMBus(i2c_bus)

        # Multiplexer setup
        self.mux: Optional[PCA9548A] = None
        if mux_address is not None and mux_channel is not None:
            self.mux = PCA9548A(bus=i2c_bus, address=mux_address)
            self.mux.select_channel(mux_channel)

        # Sensor ranges
        if gyro_range not in self.GYRO_SCALE:
            raise ValueError(f"Invalid gyro_range: {gyro_range}. Must be 250, 500, 1000, or 2000")
        if accel_range not in self.ACCEL_SCALE:
            raise ValueError(f"Invalid accel_range: {accel_range}. Must be 2, 4, 8, or 16")

        self.gyro_range = gyro_range
        self.accel_range = accel_range
        self.gyro_scale = self.GYRO_SCALE[gyro_range]
        self.accel_scale = self.ACCEL_SCALE[accel_range]

        # Initialize device
        self._initialize()

    def _select_mux_channel(self) -> None:
        """Select multiplexer channel if using multiplexer."""
        if self.mux:
            self.mux.select_channel(self.mux.current_channel)

    def _read_register(self, register: int) -> int:
        """Read a single byte from a register."""
        self._select_mux_channel()
        return self.smbus.read_byte_data(self.address, register)

    def _write_register(self, register: int, value: int) -> None:
        """Write a single byte to a register."""
        self._select_mux_channel()
        self.smbus.write_byte_data(self.address, register, value)

    def _read_registers(self, register: int, length: int) -> bytes:
        """Read multiple bytes starting from a register."""
        self._select_mux_channel()
        return bytes(self.smbus.read_i2c_block_data(self.address, register, length))

    def _initialize(self) -> None:
        """Initialize and configure the ICM45686."""
        # Verify device ID
        device_id = self._read_register(ICM45686Registers.WHO_AM_I)
        if device_id != self.DEVICE_ID:
            raise RuntimeError(
                f"ICM45686 not found. Expected WHO_AM_I=0x{self.DEVICE_ID:02X}, "
                f"got 0x{device_id:02X}"
            )

        # Software reset
        self._write_register(ICM45686Registers.SIGNAL_PATH_RESET, 0x02)
        time.sleep(0.1)  # Wait for reset

        # Wake up device (exit sleep mode)
        # PWR_MGMT0: Enable gyro and accel in low noise mode
        self._write_register(ICM45686Registers.PWR_MGMT0, 0x0F)
        time.sleep(0.05)

        # Configure gyroscope
        gyro_fs_sel = {250: 0, 500: 1, 1000: 2, 2000: 3}[self.gyro_range]
        # GYRO_CONFIG0: ODR=1kHz (0x06), FS=gyro_fs_sel
        self._write_register(ICM45686Registers.GYRO_CONFIG0, (0x06 << 4) | gyro_fs_sel)

        # Configure accelerometer
        accel_fs_sel = {2: 0, 4: 1, 8: 2, 16: 3}[self.accel_range]
        # ACCEL_CONFIG0: ODR=1kHz (0x06), FS=accel_fs_sel
        self._write_register(ICM45686Registers.ACCEL_CONFIG0, (0x06 << 4) | accel_fs_sel)

        time.sleep(0.05)  # Wait for configuration to take effect

    def read_raw_data(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int], int]:
        """
        Read raw sensor data.

        Returns:
            Tuple of (accel_xyz, gyro_xyz, temp) as 16-bit signed integers
        """
        # Read all sensor data in one burst (14 bytes)
        # Order: TEMP(2) + ACCEL_XYZ(6) + GYRO_XYZ(6)
        data = self._read_registers(ICM45686Registers.TEMP_DATA1, 14)

        # Parse data (big-endian 16-bit signed)
        temp_raw = struct.unpack('>h', data[0:2])[0]
        accel_x = struct.unpack('>h', data[2:4])[0]
        accel_y = struct.unpack('>h', data[4:6])[0]
        accel_z = struct.unpack('>h', data[6:8])[0]
        gyro_x = struct.unpack('>h', data[8:10])[0]
        gyro_y = struct.unpack('>h', data[10:12])[0]
        gyro_z = struct.unpack('>h', data[12:14])[0]

        return (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), temp_raw

    def read_data(self) -> IMUData:
        """
        Read and convert sensor data to physical units.

        Returns:
            IMUData object with calibrated sensor readings
        """
        (ax_raw, ay_raw, az_raw), (gx_raw, gy_raw, gz_raw), temp_raw = self.read_raw_data()

        # Convert to physical units
        accel_x = ax_raw * self.accel_scale * self.GRAVITY  # m/s²
        accel_y = ay_raw * self.accel_scale * self.GRAVITY
        accel_z = az_raw * self.accel_scale * self.GRAVITY

        gyro_x = gx_raw * self.gyro_scale * self.DEG_TO_RAD  # rad/s
        gyro_y = gy_raw * self.gyro_scale * self.DEG_TO_RAD
        gyro_z = gz_raw * self.gyro_scale * self.DEG_TO_RAD

        # Temperature formula from datasheet
        temperature = (temp_raw / 128.0) + 25.0  # °C

        return IMUData(
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            temperature=temperature,
            timestamp=time.time(),
        )

    def close(self) -> None:
        """Close I2C bus."""
        if self.mux:
            self.mux.disable_all()
        self.smbus.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.close()
        except:
            pass


if __name__ == "__main__":
    """Test ICM45686 driver."""
    import sys

    print("ICM45686 Driver Test")
    print("=" * 60)

    try:
        # Initialize driver (with multiplexer)
        driver = ICM45686Driver(
            i2c_bus=1,
            imu_address=0x68,
            mux_address=0x70,
            mux_channel=0,
            gyro_range=1000,
            accel_range=4,
        )

        print("✓ ICM45686 initialized successfully")
        print(f"  Gyro range: ±{driver.gyro_range} dps")
        print(f"  Accel range: ±{driver.accel_range} g")
        print("\nReading sensor data (Ctrl+C to stop)...\n")

        while True:
            data = driver.read_data()

            print(f"\rAccel: ({data.accel_x:7.3f}, {data.accel_y:7.3f}, {data.accel_z:7.3f}) m/s² | "
                  f"Gyro: ({data.gyro_x:7.3f}, {data.gyro_y:7.3f}, {data.gyro_z:7.3f}) rad/s | "
                  f"Temp: {data.temperature:5.1f}°C", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n✓ Test stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)
    finally:
        if 'driver' in locals():
            driver.close()
