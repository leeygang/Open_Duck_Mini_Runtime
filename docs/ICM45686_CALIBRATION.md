# ICM45686 IMU Calibration Guide

This guide explains how to calibrate the ICM45686 v1.2 IMU sensor for optimal performance on the Duck Mini robot.

## Why Calibrate?

All IMU sensors have manufacturing imperfections that cause:
- **Gyroscope bias**: Small false rotation readings when stationary, leading to orientation drift over time
- **Accelerometer bias**: Incorrect gravity readings, causing poor balance estimation

**Without calibration:**
- ❌ Robot orientation drifts over time
- ❌ Walking is unstable or biased to one side
- ❌ Balance calculations are incorrect

**With calibration:**
- ✓ Accurate orientation tracking
- ✓ Stable, balanced walking
- ✓ No drift during operation

## Prerequisites

1. **ICM45686 hardware connected** via PCA9548A multiplexer on I2C bus 1
2. **Flat, stable surface** (table, floor - must be level)
3. **Robot or sensor must be completely stationary** during calibration

## File Locations

Before calibrating, ensure these files are in the correct locations:

```bash
# Configuration file (required)
~/duck_config.json

# Calibration data (will be created during calibration)
~/projects/Open_Duck_Mini_Runtime/icm45686_calib_data.pkl
```

## Calibration Procedure

### Step 1: Navigate to project directory

```bash
cd ~/projects/Open_Duck_Mini_Runtime
```

### Step 2: Run calibration script

```bash
uv run python -m mini_bdx_runtime.icm45686_imu --calibrate
```

Or without uv:

```bash
python3 -m mini_bdx_runtime.icm45686_imu --calibrate
```

### Step 3: Follow on-screen instructions

The script will display:

```
============================================================
ICM45686 Calibration
============================================================

Place robot on a FLAT, STABLE surface.
Keep robot COMPLETELY STATIONARY during calibration.

Press ENTER to start calibration...
```

**IMPORTANT:**
- Place the robot/sensor on a **flat, level surface**
- **Do NOT move it** during the next 10 seconds
- Avoid vibrations, bumps, or air currents

### Step 4: Press ENTER and stay still

The script will collect 1000 samples (~10 seconds):

```
Collecting 1000 samples (~10.0s)...
  100/1000 samples collected
  200/1000 samples collected
  ...
  1000/1000 samples collected
```

### Step 5: Review results

After completion:

```
Calibration Results:
  Gyro bias: [-0.00362723 -0.00142266  0.00068177]
  Accel bias: [-6.10531838 -2.65218647 -2.55578924]

✓ Calibration saved to icm45686_calib_data.pkl
CALIBRATION DONE
```

The calibration file is now saved in the project directory.

## Verify Calibration

Test that calibration is working:

```bash
cd ~/projects/Open_Duck_Mini_Runtime
uv run python -m mini_bdx_runtime.icm45686_imu
```

You should see:

```
✓ Loaded calibration from icm45686_calib_data.pkl
✓ IMU initialized successfully

Reading orientation data (Ctrl+C to stop)...

Quaternion: [ 0.0000,  0.0000,  0.0000,  1.0000] | Euler: Roll=   0.00° Pitch=   0.00° Yaw=   0.00°
```

**Good signs:**
- ✓ "Loaded calibration" message appears
- ✓ When stationary, orientation is stable (not drifting)
- ✓ When you tilt the sensor, angles change smoothly

**Bad signs:**
- ⚠ "calib_data.pkl not found" - calibration file is missing or in wrong location
- ⚠ Orientation drifts even when stationary - recalibrate

Press Ctrl+C to stop the test.

## When to Recalibrate

You should recalibrate the IMU if:

1. **First time setup** - Always calibrate new hardware
2. **Temperature changes significantly** (>20°C difference) - sensors drift with temperature
3. **After months of use** - bias can slowly change over time
4. **Walking behavior degrades** - robot starts drifting or leaning to one side
5. **After physical shock** - dropping or bumping the robot hard
6. **Different operating environment** - moving from indoor to outdoor, or hot to cold

## Troubleshooting

### "No module named mini_bdx_runtime.icm45686_imu"

The package isn't installed in editable mode:

```bash
cd ~/projects/Open_Duck_Mini_Runtime
pip install -e .
# or
uv pip install -e .
```

### "calib_data.pkl not found" after calibration

The file is in the wrong location. Move it to project root:

```bash
# Find where it was created
find ~ -name "icm45686_calib_data.pkl"

# Move to project root
mv <found_path>/icm45686_calib_data.pkl ~/projects/Open_Duck_Mini_Runtime/
```

### Orientation still drifts after calibration

1. **Check mounting** - sensor must be firmly attached (no wobbling)
2. **Recalibrate on a flatter surface** - use a level to verify surface is flat
3. **Keep completely still** during calibration - even small vibrations affect results
4. **Temperature stabilization** - let the sensor run for 2-3 minutes before calibrating

### Calibration values seem too large

Example of reasonable values:
- Gyro bias: typically < 0.1 rad/s (< 6°/s)
- Accel bias: typically < 2 m/s² on X/Y axes

If values are much larger:
- **Check sensor orientation** - make sure Z-axis points up
- **Verify surface is level** - use a bubble level
- **Check for magnetic interference** - keep away from motors, magnets during calibration

### "ICM45686 not found" error

Hardware isn't connected or wrong I2C configuration:

```bash
# Check I2C devices
i2cdetect -y 1

# Should show:
# 0x68 - ICM45686 (through multiplexer)
# 0x70 - PCA9548A multiplexer
```

If devices aren't detected, check wiring and power connections.

## Calibration Data Format

The `.pkl` file contains:

```python
{
    'gyro_bias': [x, y, z],      # rad/s - subtracted from gyro readings
    'accel_bias': [x, y, z],     # m/s² - subtracted from accel readings
    'timestamp': 1234567890.0    # Unix timestamp of calibration
}
```

## Advanced: Manual Calibration Adjustment

If needed, you can manually adjust calibration:

```python
import pickle
import numpy as np

# Load existing calibration
with open('icm45686_calib_data.pkl', 'rb') as f:
    calib = pickle.load(f)

# Adjust values (example: reduce Z-axis gyro bias)
calib['gyro_bias'][2] -= 0.001

# Save modified calibration
with open('icm45686_calib_data.pkl', 'wb') as f:
    pickle.dump(calib, f)
```

**Warning:** Manual adjustment is rarely needed and can make things worse. Only do this if you understand the implications.

## Next Steps

After successful calibration:

1. **Update duck_config.json** to use ICM45686:
   ```json
   {
       "imu_type": "icm45686",
       ...
   }
   ```

2. **Test with walking policy**:
   ```bash
   uv run scripts/v2_rl_walk_mujoco.py --onnx_model_path <model_path>
   ```

3. **Monitor during walking** - if robot drifts to one side over time, recalibrate

## Summary

✅ Calibration is **required** for stable walking
✅ Takes ~10 seconds on a flat surface
✅ Stored in project root: `icm45686_calib_data.pkl`
✅ Recalibrate if behavior degrades or after significant temperature change
✅ Always verify calibration loaded successfully before walking

For more information, see:
- `HIWONDER_INTEGRATION.md` - Hardware setup and wiring
- `CLAUDE.md` - Full project documentation
- `example_config.json` - Configuration reference
