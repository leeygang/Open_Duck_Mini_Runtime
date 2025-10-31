# Multi-IMU Setup Guide

## Overview

This guide explains how to connect and use multiple IMUs (BNO085 + ICM45686) with the Duck Mini robot using a PCA9548A I2C multiplexer. This setup enables:

- **Redundancy**: Fallback to secondary IMU if primary fails
- **Sensor fusion**: Combine data from multiple IMUs for improved accuracy
- **Comparison**: Validate IMU readings against each other
- **Flexibility**: Switch between IMUs without rewiring

## Supported Configurations

### Configuration 1: BNO085 + ICM45686 via PCA9548A (Recommended)

Both IMUs connected through the same multiplexer on different channels.

```
Raspberry Pi (I2C Bus 1)
    └── PCA9548A (0x70)
        ├── Channel 0: ICM45686 (0x68)
        └── Channel 1: BNO085 (0x4A)
```

**Advantages**:
- Clean wiring (single I2C connection to Pi)
- Easy channel switching
- Can add up to 6 more I2C devices

### Configuration 2: Direct Connection (No Multiplexer)

ICM45686 and BNO085 have different default addresses, so they can coexist on the same bus.

```
Raspberry Pi (I2C Bus 1)
    ├── ICM45686 (0x68) [direct]
    └── BNO085 (0x4A) [direct]
```

**Advantages**:
- No multiplexer needed
- Simpler hardware
- Lower cost

**Disadvantages**:
- Both devices on same bus (potential conflicts)
- Can't add more devices with same addresses

## Hardware Setup

### Wiring Diagram (Configuration 1: With Multiplexer)

```
Component Connections:

Raspberry Pi GPIO Header:
  Pin 1  (3.3V)  ────────┬─────────────────┬──────────────
  Pin 3  (SDA)   ───────┐│                 │
  Pin 5  (SCL)   ──────┐││                 │
  Pin 9  (GND)   ─────┐│││                 │
                      │││└─────────────────┼──────────────
                      │││                  │
PCA9548A:             │││                  │
  VCC ────────────────┼┼┼──────────────────┘
  GND ────────────────┘││
  SDA ─────────────────┘│
  SCL ──────────────────┘
  SD0 ──────────────────────┐
  SC0 ──────────────────────┼─────┐
  SD1 ──────────────────────│─────┼────────┐
  SC1 ──────────────────────│─────┼────────┼───┐
                            │     │        │   │
ICM45686 (Channel 0):       │     │        │   │
  VCC ───────────────────── 3.3V  │        │   │
  GND ───────────────────── GND   │        │   │
  SDA ────────────────────────────┘        │   │
  SCL ─────────────────────────────────────┘   │
  AD0 ───────────────────── GND (addr 0x68)    │
                                                │
BNO085 (Channel 1):                             │
  VCC ───────────────────── 3.3V                │
  GND ───────────────────── GND                 │
  SDA ──────────────────────────────────────────┘
  SCL ──────────────────────────────────────────┘
  PS0 ───────────────────── 3.3V (I2C mode)
  PS1 ───────────────────── 3.3V (addr 0x4A)
```

### Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi (Zero 2W or Pi 5) | 1 | With I2C enabled |
| PCA9548A Breakout Board | 1 | 8-channel I2C mux |
| ICM45686 Breakout Board | 1 | 6-axis IMU |
| BNO085 Breakout Board | 1 | 9-axis IMU |
| Jumper Wires | ~12 | For connections |

### Step-by-Step Wiring

1. **Enable I2C on Raspberry Pi**:
   ```bash
   sudo raspi-config
   # Interface Options -> I2C -> Enable
   sudo reboot
   ```

2. **Connect PCA9548A to Raspberry Pi**:
   - PCA9548A VCC → Pi 3.3V (Pin 1)
   - PCA9548A GND → Pi GND (Pin 9)
   - PCA9548A SDA → Pi GPIO 2 (Pin 3)
   - PCA9548A SCL → Pi GPIO 3 (Pin 5)

3. **Connect ICM45686 to Channel 0**:
   - ICM45686 VCC → 3.3V
   - ICM45686 GND → GND
   - ICM45686 SDA → PCA9548A SD0
   - ICM45686 SCL → PCA9548A SC0
   - ICM45686 AD0 → GND (sets address to 0x68)

4. **Connect BNO085 to Channel 1**:
   - BNO085 VCC → 3.3V
   - BNO085 GND → GND
   - BNO085 SDA → PCA9548A SD1
   - BNO085 SCL → PCA9548A SC1
   - BNO085 PS0 → 3.3V (selects I2C mode)
   - BNO085 PS1 → 3.3V (sets address to 0x4A)

5. **Verify Connections**:
   ```bash
   # Should see 0x70 (PCA9548A)
   sudo i2cdetect -y 1
   ```

## Software Installation

### Install Dependencies

```bash
# Core dependencies
pip install smbus2 numpy scipy

# Adafruit libraries for BNO085
pip install adafruit-circuitpython-bno08x

# Or with uv (faster)
uv pip install smbus2 numpy scipy adafruit-circuitpython-bno08x
```

### Verify Installation

```bash
# Test multiplexer and both IMUs
python3 scripts/test_dual_imu.py

# Test only ICM45686
python3 scripts/test_dual_imu.py --imu icm45686

# Test only BNO085
python3 scripts/test_dual_imu.py --imu bno085

# Compare both IMUs
python3 scripts/test_dual_imu.py --compare
```

## Configuration

### Configuration File Structure

Update `~/duck_config.json` with multi-IMU settings:

```json
{
  "imu_type": "icm45686",
  "imu_fallback": "bno085",
  "imu_upside_down": false,

  "icm45686_config": {
    "i2c_bus": 1,
    "imu_address": "0x68",
    "mux_address": "0x70",
    "mux_channel": 0,
    "gyro_range": 1000,
    "accel_range": 4,
    "madgwick_beta": 0.1
  },

  "bno085_config": {
    "i2c_bus": 1,
    "imu_address": "0x4A",
    "mux_address": "0x70",
    "mux_channel": 1,
    "upside_down": false
  }
}
```

### Configuration Options

#### Primary IMU Selection

- `"imu_type": "bno085"` - Use BNO085 as primary IMU
- `"imu_type": "icm45686"` - Use ICM45686 as primary IMU
- `"imu_fallback": "bno085"` - Fallback to BNO085 if primary fails (optional)

#### Per-IMU Configuration

**ICM45686**:
- `mux_channel`: PCA9548A channel (0-7)
- `gyro_range`: 250, 500, 1000, 2000 dps
- `accel_range`: 2, 4, 8, 16 g
- `madgwick_beta`: Filter gain (0.01-0.3)

**BNO085**:
- `mux_channel`: PCA9548A channel (0-7)
- `imu_address`: 0x4A or 0x4B
- `upside_down`: Axis remapping for inverted mounting

## Usage Examples

### Example 1: Use ICM45686 as Primary

```python
from mini_bdx_runtime.icm45686_imu import ICM45686Imu

imu = ICM45686Imu(
    sampling_frequency=50,
    upside_down=False
)

# Read orientation
quat = imu.get_data()  # Quaternion (x, y, z, w)
euler = imu.get_data(euler=True)  # Euler angles (rad)
mat = imu.get_data(mat=True)  # Rotation matrix
```

### Example 2: Use BNO085 as Primary

```python
from mini_bdx_runtime.bno085_imu_mux import BNO085ImuMux

imu = BNO085ImuMux(
    sampling_freq=50,
    mux_address=0x70,
    mux_channel=1,
    bno085_address=0x4A,
    upside_down=False
)

# Same API as ICM45686
quat = imu.get_data()
euler = imu.get_data(euler=True)
```

### Example 3: Automatic Selection from Config

```python
from mini_bdx_runtime.duck_config import DuckConfig

config = DuckConfig()

# Automatically load correct IMU
if config.config.get('imu_type') == 'icm45686':
    from mini_bdx_runtime.icm45686_imu import ICM45686Imu
    imu = ICM45686Imu(
        sampling_frequency=50,
        upside_down=config.imu_upside_down
    )
elif config.config.get('imu_type') == 'bno085':
    from mini_bdx_runtime.bno085_imu_mux import BNO085ImuMux
    bno_config = config.config.get('bno085_config', {})
    imu = BNO085ImuMux(
        sampling_freq=50,
        mux_address=int(bno_config.get('mux_address', '0x70'), 16),
        mux_channel=bno_config.get('mux_channel', 1),
        bno085_address=int(bno_config.get('imu_address', '0x4A'), 16),
        upside_down=config.imu_upside_down
    )
else:
    # Fallback to BNO055
    from mini_bdx_runtime.imu import Imu
    imu = Imu(
        sampling_freq=50,
        upside_down=config.imu_upside_down
    )
```

### Example 4: IMU with Fallback

```python
def initialize_imu_with_fallback(config):
    """Initialize IMU with automatic fallback."""
    primary = config.config.get('imu_type', 'bno055')
    fallback = config.config.get('imu_fallback')

    try:
        if primary == 'icm45686':
            from mini_bdx_runtime.icm45686_imu import ICM45686Imu
            return ICM45686Imu(sampling_frequency=50)
        elif primary == 'bno085':
            from mini_bdx_runtime.bno085_imu_mux import BNO085ImuMux
            bno_config = config.config.get('bno085_config', {})
            return BNO085ImuMux(
                sampling_freq=50,
                mux_address=int(bno_config.get('mux_address', '0x70'), 16),
                mux_channel=bno_config.get('mux_channel', 1)
            )
    except Exception as e:
        print(f"Failed to initialize {primary}: {e}")

        if fallback:
            print(f"Falling back to {fallback}...")
            return initialize_imu_with_fallback(
                type('Config', (), {'config': {'imu_type': fallback}})()
            )
        raise
```

## Calibration

### ICM45686 Calibration

```bash
# Basic calibration (robot stationary on flat surface)
python3 scripts/calibrate_icm45686.py

# Full 6-position calibration
python3 scripts/calibrate_icm45686.py --full
```

Creates `icm45686_calib_data.pkl`.

### BNO085 Calibration

BNO085 has **automatic dynamic calibration** - no manual calibration needed!

For best results:
1. Move the robot through various orientations during first 30 seconds
2. Include figure-8 motions for magnetometer calibration
3. The sensor calibrates continuously during operation

## Testing and Validation

### Quick Hardware Test

```bash
# Verify all hardware detected
python3 scripts/test_dual_imu.py
```

Expected output:
```
✓ PASS: Multiplexer
✓ PASS: ICM45686
✓ PASS: BNO085
```

### Comparison Test

```bash
# Run both IMUs simultaneously
python3 scripts/test_dual_imu.py --compare --duration 30
```

This will:
- Read both IMUs at 50 Hz
- Calculate orientation differences
- Report mean/max/std errors
- Validate agreement within tolerance

**Expected Results**:
- Roll/Pitch error: < 5° (good), < 10° (acceptable)
- Yaw error: < 15° (ICM45686 drifts without magnetometer)

### Walking Policy Test

```bash
# Test with walking policy
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <model.onnx>
```

The policy will use whichever IMU is specified in `duck_config.json`.

## Troubleshooting

### Multiplexer Not Detected

**Symptom**: `sudo i2cdetect -y 1` doesn't show 0x70

**Solutions**:
1. Check PCA9548A power and ground connections
2. Verify SDA/SCL connections to correct GPIO pins
3. Check address pins (A0-A2) on PCA9548A

### IMU Not Detected on Channel

**Symptom**: Multiplexer detected but IMU not responding

**Solutions**:
1. Verify channel wiring (SD0/SC0 for channel 0, etc.)
2. Check IMU power connections
3. Test IMU with direct connection (bypass multiplexer)
4. Use `python3 scripts/test_dual_imu.py --imu <imu_name>`

### IMUs Show Different Orientations

**Symptom**: Comparison test shows high disagreement (>10°)

**Solutions**:
1. Check mounting orientations match
2. Calibrate ICM45686: `python3 scripts/calibrate_icm45686.py`
3. Allow BNO085 to calibrate (move through orientations)
4. Verify `upside_down` setting matches physical mounting
5. Check axis remapping configuration

### High Yaw Drift (ICM45686)

**Symptom**: ICM45686 yaw drifts over time

**Expected Behavior**: ICM45686 has no magnetometer, so yaw drift is normal (~1-2°/min)

**Solutions**:
1. Re-calibrate gyroscope (robot must be stationary!)
2. Use BNO085 for applications requiring absolute heading
3. Increase Madgwick beta to reduce drift: `"madgwick_beta": 0.15`

### I2C Bus Errors

**Symptom**: Random I2C errors, timeouts, or freezes

**Solutions**:
1. Reduce I2C speed in `/boot/config.txt`:
   ```
   dtparam=i2c_arm=on,i2c_arm_baudrate=100000
   ```
2. Check wiring for loose connections
3. Add pull-up resistors (4.7kΩ) if using long wires
4. Keep I2C wires short (< 20cm ideal)

## Performance Considerations

### Update Rates

| IMU | Max Rate | Recommended | CPU Usage (Pi Zero 2W) |
|-----|----------|-------------|------------------------|
| ICM45686 | 1000 Hz | 50-100 Hz | ~0.5 ms/update |
| BNO085 | 400 Hz | 50-100 Hz | ~1-2 ms/update |

### Latency

- **ICM45686**: ~2-5 ms (sensor read + Madgwick filter)
- **BNO085**: ~3-7 ms (sensor read, built-in fusion)
- **Channel switching**: ~0.1 ms (negligible)

### Bandwidth

With both IMUs at 50 Hz:
- Combined I2C traffic: ~50 kbps
- I2C bus capacity: 400 kbps (fast mode)
- Plenty of headroom for additional devices

## Advanced Topics

### Sensor Fusion Between IMUs

For advanced users, you can fuse data from both IMUs:

```python
def fuse_imu_data(icm_quat, bno_quat, icm_weight=0.5):
    """
    Simple weighted quaternion fusion.

    Args:
        icm_quat: ICM45686 quaternion
        bno_quat: BNO085 quaternion
        icm_weight: Weight for ICM (0-1)

    Returns:
        Fused quaternion
    """
    from scipy.spatial.transform import Rotation as R

    # Convert to Rotation objects
    icm_rot = R.from_quat(icm_quat)
    bno_rot = R.from_quat(bno_quat)

    # SLERP interpolation
    slerp = R.from_quat([icm_rot, bno_rot])
    fused = slerp([icm_weight])[0]

    return fused.as_quat()
```

### Adding More IMUs

PCA9548A has 8 channels - you can add up to 8 devices:

```
Channel 0: ICM45686 #1
Channel 1: BNO085 #1
Channel 2: ICM45686 #2
Channel 3: BNO085 #2
...
Channel 7: (spare)
```

### Channel Auto-Discovery

Scan all channels and auto-detect IMUs:

```python
from mini_bdx_runtime.pca9548a import scan_all_channels

# Scan for devices
devices = scan_all_channels(bus=1, mux_address=0x70)

# Find ICM45686 instances
icm_channels = [ch for ch, devs in devices.items() if 0x68 in devs]

# Find BNO085 instances
bno_channels = [ch for ch, devs in devices.items() if 0x4A in devs or 0x4B in devs]

print(f"Found ICM45686 on channels: {icm_channels}")
print(f"Found BNO085 on channels: {bno_channels}")
```

## Summary

### Pros and Cons

**ICM45686**:
- ✅ High performance, low power
- ✅ Fast update rates
- ✅ Precise roll/pitch
- ❌ No magnetometer (yaw drifts)
- ❌ Requires software fusion

**BNO085**:
- ✅ Built-in sensor fusion
- ✅ Automatic calibration
- ✅ Magnetometer (absolute heading)
- ✅ No configuration needed
- ❌ Slightly higher latency
- ❌ More expensive

### Recommended Use Cases

- **Racing/Agility**: ICM45686 (faster, precise orientation)
- **Navigation**: BNO085 (magnetometer for heading)
- **General Walking**: Either works great!
- **Production**: BNO085 (automatic calibration, easier for users)
- **Research**: Both (compare algorithms, redundancy)

## References

- [PCA9548A Datasheet](https://www.ti.com/product/PCA9548A)
- [ICM-45686 Documentation](docs/ICM45686_INTEGRATION.md)
- [BNO085 Datasheet](https://www.ceva-dsp.com/product/bno080-085/)
- [Raspberry Pi I2C Guide](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
