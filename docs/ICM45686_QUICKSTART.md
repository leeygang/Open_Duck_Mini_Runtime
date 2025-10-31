# ICM45686 Quick Start Guide

Get your ICM45686 IMU up and running with Duck Mini in 5 minutes.

## Prerequisites

- Raspberry Pi with I2C enabled (`sudo raspi-config` → Interface Options → I2C → Enable)
- ICM45686 IMU connected to PCA9548A multiplexer
- PCA9548A connected to Raspberry Pi I2C (GPIO 2/3)
- Python dependencies installed: `pip install smbus2 numpy scipy`

## Hardware Setup

```
Raspberry Pi (I2C Bus 1)
    ├── SDA (GPIO 2) ──── PCA9548A SDA
    ├── SCL (GPIO 3) ──── PCA9548A SCL
    ├── 3.3V ──────────── PCA9548A VCC
    └── GND ───────────── PCA9548A GND

PCA9548A Channel 0
    ├── SD0 ──────────── ICM45686 SDA
    ├── SC0 ──────────── ICM45686 SCL
    ├── VCC ──────────── ICM45686 VCC (3.3V)
    ├── GND ──────────── ICM45686 GND
    └── AD0 (ICM45686) ── GND (for address 0x68)
```

## Step-by-Step Setup

### 1. Verify Hardware Connection

```bash
# Check that PCA9548A is detected (should see 0x70)
sudo i2cdetect -y 1
```

### 2. Run Hardware Test

```bash
# Test all components
python3 scripts/icm45686_test.py
```

Expected output:
- ✓ PCA9548A detected at address 0x70
- ✓ ICM45686 detected on channel 0
- Raw sensor data streaming
- Orientation (quaternion + Euler angles) streaming

### 3. Calibrate the IMU

**IMPORTANT**: Place robot on a flat surface and keep completely stationary.

```bash
# Basic calibration (recommended)
python3 scripts/calibrate_icm45686.py

# Or full 6-position calibration (more accurate)
python3 scripts/calibrate_icm45686.py --full
```

This creates `icm45686_calib_data.pkl` which is automatically loaded by the IMU class.

### 4. Update Configuration

Copy the ICM45686 example config to your home directory:

```bash
cp example_config_icm45686.json ~/duck_config.json
```

Or manually update your existing `~/duck_config.json`:

```json
{
    "imu_type": "icm45686",
    "imu_upside_down": false,
    "icm45686_config": {
        "i2c_bus": 1,
        "imu_address": "0x68",
        "mux_address": "0x70",
        "mux_channel": 0,
        "gyro_range": 1000,
        "accel_range": 4,
        "madgwick_beta": 0.1
    }
}
```

### 5. Update Walking Script (if needed)

The ICM45686 IMU is designed as a drop-in replacement. If your walking script currently uses:

```python
from mini_bdx_runtime.imu import Imu
```

Update it to automatically select based on config:

```python
from mini_bdx_runtime.duck_config import DuckConfig

config = DuckConfig()

if config.config.get('imu_type') == 'icm45686':
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu as Imu
else:
    from mini_bdx_runtime.imu import Imu

# Rest of code remains the same
imu = Imu(
    sampling_frequency=50,
    user_pitch_bias=0,
    upside_down=config.imu_upside_down
)
```

### 6. Test the Walking Policy

```bash
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <your_model.onnx>
```

The robot should now use the ICM45686 for orientation sensing!

## Configuration Options

### I2C Addresses

- **ICM45686**: `0x68` (default, AD0=GND) or `0x69` (AD0=VCC)
- **PCA9548A**: `0x70` (default, A0-A2=GND)

### Sensor Ranges

- **Gyro**: 250, 500, 1000 (default), 2000 dps
- **Accel**: 2, 4 (default), 8, 16 g

### Madgwick Filter Beta

Controls sensor fusion behavior:
- **Lower (0.01-0.05)**: Smoother, slower response, more drift
- **Default (0.1)**: Balanced
- **Higher (0.2-0.3)**: More responsive, less smooth, minimal drift

### Axis Remapping

If IMU is mounted in a different orientation, adjust `axis_remap` and `axis_sign`:

```json
"axis_remap": {"x": "y", "y": "x", "z": "z"},
"axis_sign": {"x": -1, "y": 1, "z": 1}
```

## Troubleshooting

### Device Not Detected

```bash
# Rescan I2C bus
sudo i2cdetect -y 1

# Check for 0x70 (PCA9548A)
# Run test with debug mode
python3 scripts/icm45686_test.py --debug
```

### Orientation Drift

1. Re-run calibration (robot must be stationary!)
2. Increase Madgwick beta: `"madgwick_beta": 0.15`
3. Check for vibrations affecting sensors

### Poor Orientation Accuracy

1. Perform full 6-position calibration: `python3 scripts/calibrate_icm45686.py --full`
2. Check axis remapping matches your mounting orientation
3. Reduce Madgwick beta for smoother estimates: `"madgwick_beta": 0.08`

### I2C Errors

```bash
# Reduce I2C speed (add to /boot/config.txt)
dtparam=i2c_arm=on,i2c_arm_baudrate=100000

# Reboot
sudo reboot
```

## Next Steps

- **Fine-tune Madgwick beta**: Adjust based on your robot's dynamics
- **Axis calibration**: If orientation looks wrong, adjust `axis_remap` and `axis_sign`
- **Multi-IMU setup**: Use additional PCA9548A channels for multiple IMUs
- **Performance testing**: Monitor IMU update rates during walking

## Additional Resources

- [Full Integration Guide](ICM45686_INTEGRATION.md) - Detailed documentation
- [ICM45686 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/)
- [Madgwick Filter Paper](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

## Getting Help

If you encounter issues:

1. Run diagnostics: `python3 scripts/icm45686_test.py --debug`
2. Check calibration: Ensure robot was stationary during calibration
3. Verify connections: Use `sudo i2cdetect -y 1` to check devices
4. Review logs: Look for error messages in console output

For hardware-related issues, double-check wiring and I2C addresses. For software issues, ensure all dependencies are installed and configuration file is valid JSON.
