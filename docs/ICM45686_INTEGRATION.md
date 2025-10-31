# ICM45686 IMU Integration Guide

## Overview

This guide covers integrating the TDK InvenSense ICM45686 6-axis IMU (accelerometer + gyroscope) with the Open Duck Mini Runtime. The ICM45686 connects via I2C through a PCA9548A I2C multiplexer/switch.

## Hardware Overview

### ICM45686 Features
- **6-axis motion tracking**: 3-axis accelerometer + 3-axis gyroscope
- **I2C interface**: Supports standard (100 kHz) and fast mode (400 kHz, 1 MHz)
- **I2C addresses**: 0x68 (default) or 0x69 (when AD0 pin high)
- **Operating voltage**: 1.71V - 3.6V
- **Gyroscope range**: ±250, ±500, ±1000, ±2000 dps
- **Accelerometer range**: ±2g, ±4g, ±8g, ±16g
- **Built-in temperature sensor**

### PCA9548A I2C Multiplexer
- **8-channel I2C switch**: Allows multiple devices with same address
- **I2C address**: 0x70 (default, configurable via A0-A2 pins)
- **Operating voltage**: 2.3V - 5.5V
- **Use case**: Isolates ICM45686 on a dedicated channel

## Hardware Connection

### Wiring Diagram

```
Raspberry Pi          PCA9548A              ICM45686
------------          --------              --------
GPIO 2 (SDA) -------- SDA
GPIO 3 (SCL) -------- SCL
3.3V ----------------> VCC ----------------> VCC
GND ------------------> GND ----------------> GND
                       SD0-SD7 (choose one)
                       SC0-SC7 (corresponding)
                                <------------ SDA
                                <------------ SCL
                                              AD0 --> GND (0x68) or 3.3V (0x69)
```

### Connection Steps

1. **Enable I2C on Raspberry Pi**:
   ```bash
   sudo raspi-config
   # Interface Options -> I2C -> Enable
   ```

2. **Physical connections**:
   - Connect PCA9548A to Raspberry Pi I2C bus (GPIO 2/3)
   - Connect ICM45686 to one of PCA9548A channels (e.g., channel 0)
   - Connect AD0 pin on ICM45686 to GND for address 0x68

3. **Verify connections**:
   ```bash
   # Check PCA9548A is detected (should see 0x70)
   sudo i2cdetect -y 1

   # After configuring PCA9548A channel, check ICM45686 (0x68)
   ```

## Differences from BNO055

| Feature | BNO055 (current) | ICM45686 (new) |
|---------|------------------|----------------|
| **Axes** | 9-axis (accel + gyro + mag) | 6-axis (accel + gyro) |
| **Sensor fusion** | Built-in quaternion output | Software fusion required |
| **Magnetometer** | Yes | No |
| **Multiplexer** | Not required | Requires PCA9548A |
| **Calibration** | Built-in auto-calibration | Manual calibration needed |
| **Output** | Quaternion, Euler angles | Raw accel/gyro data |

**Key difference**: ICM45686 requires software-based sensor fusion to compute orientation (quaternions/Euler angles). We implement a Madgwick filter for this purpose.

## Software Architecture

### Component Structure

```
mini_bdx_runtime/
├── icm45686_driver.py        # Low-level I2C driver for ICM45686
├── pca9548a.py                # PCA9548A multiplexer control
├── madgwick_filter.py         # Sensor fusion algorithm
├── icm45686_imu.py            # High-level IMU interface (drop-in replacement for imu.py)

scripts/
├── icm45686_test.py           # Test script for ICM45686
├── calibrate_icm45686.py      # Calibration utility
```

### Class Hierarchy

- **`PCA9548A`**: Controls channel selection on the I2C multiplexer
- **`ICM45686Driver`**: Low-level register access, data reading
- **`MadgwickFilter`**: Sensor fusion (accel + gyro → quaternion)
- **`ICM45686Imu`**: High-level interface matching existing `Imu` API

## Configuration

### duck_config.json Settings

Add ICM45686-specific configuration to your `duck_config.json`:

```json
{
  "imu_type": "icm45686",
  "icm45686_config": {
    "i2c_bus": 1,
    "imu_address": "0x68",
    "mux_address": "0x70",
    "mux_channel": 0,
    "gyro_range": 1000,
    "accel_range": 4,
    "sample_rate": 100,
    "axis_remap": {
      "x": "x",
      "y": "y",
      "z": "z"
    },
    "axis_sign": {
      "x": 1,
      "y": 1,
      "z": 1
    }
  },
  "imu_upside_down": false,
  "pitch_bias": 0.0
}
```

**Configuration options**:
- `imu_type`: Set to `"icm45686"` to use the new IMU
- `i2c_bus`: I2C bus number (usually 1 on Raspberry Pi)
- `imu_address`: ICM45686 I2C address (`0x68` or `0x69`)
- `mux_address`: PCA9548A I2C address (default `0x70`)
- `mux_channel`: PCA9548A channel (0-7) where ICM45686 is connected
- `gyro_range`: Gyroscope full-scale range in dps (250, 500, 1000, 2000)
- `accel_range`: Accelerometer full-scale range in g (2, 4, 8, 16)
- `sample_rate`: Sampling frequency in Hz (10-1000)
- `axis_remap`: Remap axes if IMU is mounted in different orientation
- `axis_sign`: Flip axis direction (+1 or -1)

## Installation

### Python Dependencies

```bash
# Install required packages
pip install smbus2 numpy scipy

# Or with uv
uv pip install smbus2 numpy scipy
```

### Verify Installation

```bash
# Test ICM45686 connection and data reading
python3 scripts/icm45686_test.py

# Calibrate the IMU (collect bias data)
python3 scripts/calibrate_icm45686.py

# Network test (similar to existing IMU test)
python3 scripts/icm45686_server.py  # on robot
python3 scripts/icm45686_client.py --ip <robot_ip>  # on dev machine
```

## Usage in Walking Policy

### Minimal Code Change

The `ICM45686Imu` class implements the same API as the existing `Imu` class, so integration is straightforward:

```python
# In scripts/v2_rl_walk_mujoco.py

from mini_bdx_runtime.duck_config import DuckConfig

config = DuckConfig()

# Automatically selects IMU based on config
if config.config.get('imu_type') == 'icm45686':
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu
    imu = ICM45686Imu(
        sampling_frequency=config.control_freq,
        upside_down=config.imu_upside_down
    )
else:
    from mini_bdx_runtime.imu import Imu
    imu = Imu(
        sampling_frequency=config.control_freq,
        upside_down=config.imu_upside_down
    )

# Rest of the code remains the same
imu.start_reading()
# ...
quaternion = imu.get_quaternion()
```

## Calibration Procedure

### Gyroscope Bias Calibration

```bash
python3 scripts/calibrate_icm45686.py
```

1. Place robot on a **flat, stable surface**
2. Keep robot **completely stationary**
3. Script will collect 1000 samples (~10 seconds)
4. Gyro and accel biases saved to `icm45686_calib_data.pkl`

The `ICM45686Imu` class automatically loads calibration data on startup.

### Accelerometer Calibration (Optional)

For improved accuracy, perform 6-position calibration:

```bash
python3 scripts/calibrate_icm45686.py --full
```

Follow prompts to place robot in 6 orientations (±X, ±Y, ±Z facing up).

## Sensor Fusion Algorithm

The ICM45686 provides raw sensor data, so we implement the **Madgwick filter** for sensor fusion:

- **Input**: Accelerometer (m/s²) + Gyroscope (rad/s)
- **Output**: Quaternion (w, x, y, z)
- **Update rate**: 50-100 Hz (configurable)
- **Beta parameter**: 0.1 (filter gain, tune for noise vs. responsiveness)

The Madgwick filter fuses gyro (fast, drifts) with accel (slow, stable) to compute orientation without magnetometer.

## Troubleshooting

### I2C Communication Errors

**Symptom**: `OSError: [Errno 121] Remote I/O error`

**Solutions**:
1. Check physical connections (SDA, SCL, power, ground)
2. Verify I2C is enabled: `sudo raspi-config`
3. Check device detection: `sudo i2cdetect -y 1`
4. Reduce I2C speed if errors persist:
   ```bash
   # Edit /boot/config.txt
   dtparam=i2c_arm=on,i2c_arm_baudrate=100000
   ```

### PCA9548A Not Responding

**Symptom**: Cannot detect device at 0x70

**Solutions**:
1. Check A0, A1, A2 pins on PCA9548A (determines address)
2. Default address: 0x70 (all pins to GND)
3. Try scanning: `sudo i2cdetect -y 1` (should see 0x70)

### ICM45686 Not Detected Through Multiplexer

**Symptom**: Device detected at 0x70 but not 0x68 after channel selection

**Solutions**:
1. Verify correct channel number (0-7)
2. Check connections between PCA9548A and ICM45686
3. Use test script: `python3 scripts/icm45686_test.py --debug`

### Orientation Drift

**Symptom**: Quaternion drifts over time, robot orientation incorrect

**Solutions**:
1. Perform gyroscope calibration (robot must be stationary during calibration)
2. Adjust Madgwick filter beta parameter (higher = more accel influence, less drift)
3. Check mounting orientation matches `axis_remap` config
4. Verify sample rate matches control loop frequency

### Poor Orientation Accuracy

**Symptom**: Orientation jittery or incorrect

**Solutions**:
1. Reduce Madgwick beta for smoother estimates
2. Check for vibrations affecting accelerometer
3. Perform full 6-position accelerometer calibration
4. Verify gyro range is appropriate (1000 dps recommended)

## Performance Considerations

- **I2C speed**: 400 kHz recommended (1 MHz if short wires)
- **Update rate**: 50-100 Hz for walking policy (matches control loop)
- **CPU usage**: Madgwick filter is lightweight (~0.5ms per update on Pi Zero 2W)
- **Latency**: ~2-5ms from sensor read to quaternion output

## Comparison Summary

**Use BNO055 if**:
- You want built-in sensor fusion (less software complexity)
- You need magnetometer for absolute heading
- You prefer auto-calibration

**Use ICM45686 if**:
- You need multiple IMUs (requires PCA9548A for each)
- You want more control over sensor fusion
- BNO055 is unavailable or you prefer modern sensor

Both IMUs work with the walking policy - the choice depends on your hardware setup and preferences.

## References

- [ICM-45686 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-45686/)
- [PCA9548A Datasheet](https://www.ti.com/product/PCA9548A)
- [Madgwick Filter Paper](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- Raspberry Pi I2C Documentation
