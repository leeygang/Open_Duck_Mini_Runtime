# BNO085 IMU Integration Guide

This guide explains how to use the BNO085 (BNO08x series) IMU with the Duck Mini robot as an alternative to the BNO055.

## Overview

The BNO085 is a newer generation 9-axis IMU from Bosch that offers several improvements over the BNO055:

- **Higher update rates**: Up to 400Hz for some sensor reports
- **Improved calibration**: Automatic dynamic calibration
- **SH-2 protocol**: More flexible report-based configuration
- **Better accuracy**: Improved sensor fusion algorithms

## Hardware Setup

### Wiring

Connect the BNO085 to the Raspberry Pi I2C bus:

| BNO085 Pin | Raspberry Pi Pin | Description |
|------------|------------------|-------------|
| VIN        | 3.3V (Pin 1)     | Power supply |
| GND        | GND (Pin 6)      | Ground |
| SDA        | GPIO 2 (Pin 3)   | I2C Data |
| SCL        | GPIO 3 (Pin 5)   | I2C Clock |

**Note**: The BNO085 can share the same I2C bus with the BNO055 if you have both sensors, as they use different I2C addresses.

### I2C Address

- **Default address**: 0x4A
- **Alternative address**: 0x4B (configurable with ADR jumper)

### Enable I2C

```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
sudo reboot
```

### Verify Connection

```bash
# Check if the sensor is detected
sudo i2cdetect -y 1
```

You should see `4a` or `4b` in the output grid.

## Software Installation

### Install Dependencies

```bash
# Using pip
pip install adafruit-circuitpython-bno08x

# Or using uv (faster)
uv pip install adafruit-circuitpython-bno08x
```

### Update the Package

If you've modified `pyproject.toml` (already done), reinstall in editable mode:

```bash
# Using pip
pip install -e .

# Or using uv
uv pip install -e .
```

## Testing the BNO085

### Quick Test

Run the standalone test script:

```bash
python3 scripts/test_bno085.py
```

This provides an interactive menu with several test options:
1. Basic reading test (normal orientation)
2. Basic reading test (upside down orientation)
3. Orientation check (verify axes)
4. Comparison with BNO055 (if available)

### Manual Test

You can also test directly with Python:

```python
from mini_bdx_runtime.bno085_imu import BNO085Imu
import time
import numpy as np

# Initialize IMU
imu = BNO085Imu(sampling_freq=50, upside_down=False)

# Read data
while True:
    quat = imu.get_data()
    euler = imu.get_data(euler=True)
    print(f"Euler (deg): {np.rad2deg(euler)}")
    time.sleep(0.2)
```

### Test Output

A successful test should show:
```
BNO085 IMU Test
==================================================
✓ BNO085 initialized successfully

Quaternion: [+0.000, +0.000, +0.000, +1.000]
Euler (deg): Roll=  +0.5  Pitch=  -1.2  Yaw= +45.3
Gyro (rad/s): [+0.001, -0.002, +0.000]
Accel (m/s²): [+0.12, -0.08, +9.81]  |a|=9.82
```

## Integration with Duck Mini

### Option 1: Modify Existing Code (Simple Switch)

To use BNO085 instead of BNO055 in your robot, simply change the import in files that use the IMU:

**Before:**
```python
from mini_bdx_runtime.imu import Imu
```

**After:**
```python
from mini_bdx_runtime.bno085_imu import BNO085Imu as Imu
```

The API is identical, so no other code changes are needed.

### Option 2: Configuration-Based Selection

Create a flexible IMU loader that chooses based on config:

```python
# In duck_config.py or a new imu_factory.py
def get_imu(sampling_freq, user_pitch_bias=0, upside_down=True, imu_type="bno055"):
    """
    Factory function to get the appropriate IMU instance.

    Args:
        imu_type: "bno055" or "bno085"
    """
    if imu_type == "bno085":
        from mini_bdx_runtime.bno085_imu import BNO085Imu
        return BNO085Imu(sampling_freq, user_pitch_bias, upside_down=upside_down)
    else:
        from mini_bdx_runtime.imu import Imu
        return Imu(sampling_freq, user_pitch_bias, upside_down=upside_down)
```

Then add to `duck_config.json`:
```json
{
    "imu_type": "bno085",
    ...
}
```

### Option 3: Automatic Detection

Auto-detect which IMU is connected:

```python
def get_imu_auto(sampling_freq, user_pitch_bias=0, upside_down=True):
    """Try BNO085 first, fall back to BNO055."""
    try:
        from mini_bdx_runtime.bno085_imu import BNO085Imu
        imu = BNO085Imu(sampling_freq, user_pitch_bias, upside_down=upside_down)
        print("Using BNO085 IMU")
        return imu
    except Exception as e:
        print(f"BNO085 not available ({e}), trying BNO055...")
        from mini_bdx_runtime.imu import Imu
        imu = Imu(sampling_freq, user_pitch_bias, upside_down=upside_down)
        print("Using BNO055 IMU")
        return imu
```

## Calibration

### Key Differences from BNO055

**BNO055**: Requires manual calibration routine and stores offsets in a file (`imu_calib_data.pkl`).

**BNO085**: Uses automatic dynamic calibration that continuously runs in the background.

### BNO085 Calibration Tips

1. **Initial calibration**: During the first 10-30 seconds after power-on, move the sensor through various orientations:
   - Rotate around all three axes
   - Move in different directions
   - Keep the sensor still for a few seconds

2. **Ongoing calibration**: The BNO085 continuously recalibrates during operation, so calibration improves over time.

3. **No manual offsets**: You don't need to run a calibration script or store offsets.

### Migration from BNO055

If you have an existing `imu_calib_data.pkl` from BNO055:
- The BNO085 will detect but **not use** these offsets
- The file is ignored (BNO085 uses automatic calibration)
- You can safely delete the file if using BNO085 exclusively

## Axis Configuration

Both IMUs support `upside_down` mounting orientation:

```python
# Normal mounting (component side up)
imu = BNO085Imu(50, upside_down=False)

# Upside down mounting (component side down)
imu = BNO085Imu(50, upside_down=True)
```

Set the `imu_upside_down` flag in `duck_config.json` to match your mounting.

## API Reference

The BNO085 IMU class maintains 100% API compatibility with the BNO055 version:

### Constructor

```python
BNO085Imu(sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True)
```

- `sampling_freq`: Sensor sampling rate in Hz (typically 50)
- `user_pitch_bias`: Additional pitch bias in degrees (default 0)
- `calibrate`: If True, monitors calibration for 10 seconds (informational only)
- `upside_down`: Axis remapping for upside-down mounting

### Methods

**`get_data(euler=False, mat=False)`**
- Returns quaternion (default), euler angles, or rotation matrix
- Quaternion format: `[x, y, z, w]` (scalar last, compatible with Isaac/ROS)

**`get_gyro()`**
- Returns gyroscope data in rad/s as `[x, y, z]`

**`get_acceleration()`**
- Returns accelerometer data in m/s² as `[x, y, z]`

## Performance Comparison

| Feature | BNO055 | BNO085 |
|---------|--------|--------|
| Max update rate | ~100 Hz | ~400 Hz |
| Quaternion output | ✓ | ✓ |
| Calibration | Manual | Automatic |
| Setup time | ~1 second | ~1 second |
| I2C speed | 400 kHz | 400 kHz |
| Power consumption | ~12 mA | ~4 mA |

## Troubleshooting

### Sensor Not Detected

```
Error: No I2C device at address 0x4A or 0x4B
```

**Solutions**:
1. Check wiring (SDA, SCL, VIN, GND)
2. Verify I2C is enabled: `sudo raspi-config`
3. Check I2C bus: `sudo i2cdetect -y 1`
4. Try the alternative address (check ADR pin)

### Import Error

```
ModuleNotFoundError: No module named 'adafruit_bno08x'
```

**Solution**:
```bash
pip install adafruit-circuitpython-bno08x
```

### Erratic Readings

**Possible causes**:
1. **Insufficient calibration**: Move sensor through various orientations for 30 seconds
2. **Electromagnetic interference**: Keep away from motors, magnets, high-current wires
3. **Loose wiring**: Verify solid connections on I2C pins
4. **Power supply noise**: Add a 10µF capacitor between VIN and GND

### Axis Orientation Wrong

**Solution**:
- Toggle the `upside_down` parameter
- Run the orientation check test: `python3 scripts/test_bno085.py` → Option 3
- Verify mounting matches the config setting

### Values Drifting

**This is normal** during the first minute as the sensor calibrates. If it persists:
1. Check for vibration or movement during operation
2. Ensure stable mounting
3. Verify temperature is stable (avoid heating/cooling)

## Running on Duck Mini

### Testing IMU Only

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/bno085_imu.py
```

### Full Walking Script

After switching to BNO085 in your code:

```bash
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <path_to_onnx>
```

The robot should work exactly as before, but with improved IMU performance.

## Frequently Asked Questions

**Q: Can I use both BNO055 and BNO085 at the same time?**
A: Yes, they can coexist on the same I2C bus with different addresses. However, the robot code is designed to use one IMU at a time.

**Q: Which IMU should I choose?**
A:
- **BNO055**: Proven, well-tested, widely used
- **BNO085**: Newer, better accuracy, automatic calibration, lower power

For new builds, BNO085 is recommended.

**Q: Do I need to recalibrate the robot's joint offsets?**
A: No. Joint offsets in `duck_config.json` are independent of the IMU. Switching IMUs doesn't affect motor calibration.

**Q: Will the RL policy work with BNO085?**
A: Yes. The quaternion output format is identical, so the policy receives the same observation space.

**Q: Is the BNO085 faster?**
A: The BNO085 can provide data at higher rates (up to 400Hz vs ~100Hz), but the current control loop runs at 50Hz, so both IMUs exceed this requirement.

## Additional Resources

- [Adafruit BNO08x Library Documentation](https://docs.circuitpython.org/projects/bno08x/en/latest/)
- [BNO085 Datasheet](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- [I2C Troubleshooting Guide](https://learn.adafruit.com/circuitpython-essentials/circuitpython-i2c)

## Summary

To use the BNO085 IMU:

1. ✓ Wire the sensor to I2C pins
2. ✓ Install `adafruit-circuitpython-bno08x`
3. ✓ Run `python3 scripts/test_bno085.py` to verify
4. ✓ Update imports in your code to use `BNO085Imu`
5. ✓ Configure `imu_upside_down` in `duck_config.json`
6. ✓ Run the robot normally

No calibration files or special procedures needed - the BNO085 handles it automatically!
