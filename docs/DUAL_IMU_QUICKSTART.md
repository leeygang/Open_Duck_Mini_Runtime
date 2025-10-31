# Dual IMU Quick Start Guide

Get both BNO085 and ICM45686 running on your Duck Mini in 10 minutes.

## Hardware Checklist

- ✅ Raspberry Pi with I2C enabled
- ✅ PCA9548A I2C multiplexer breakout board
- ✅ BNO085 IMU breakout board
- ✅ ICM45686 IMU breakout board
- ✅ Jumper wires

## 5-Minute Setup

### Step 1: Wire Everything

```
Raspberry Pi → PCA9548A:
  3.3V → VCC
  GND  → GND
  SDA  → SDA (GPIO 2)
  SCL  → SCL (GPIO 3)

PCA9548A Channel 0 → ICM45686:
  SD0 → SDA
  SC0 → SCL
  (ICM45686 also needs VCC/GND/AD0→GND)

PCA9548A Channel 1 → BNO085:
  SD1 → SDA
  SC1 → SCL
  (BNO085 also needs VCC/GND/PS0→3.3V/PS1→3.3V)
```

**Power all devices from 3.3V and connect all grounds together!**

### Step 2: Install Software

```bash
# Install dependencies
pip install smbus2 numpy scipy adafruit-circuitpython-bno08x

# Or with uv (faster)
uv pip install smbus2 numpy scipy adafruit-circuitpython-bno08x
```

### Step 3: Test Hardware

```bash
# Verify multiplexer detected
sudo i2cdetect -y 1
# Should see: 0x70

# Test both IMUs
python3 scripts/test_dual_imu.py
```

Expected output:
```
✓ PASS: Multiplexer
  Channel 0: 0x68 (ICM45686)
  Channel 1: 0x4A (BNO085)
✓ PASS: ICM45686
✓ PASS: BNO085
```

### Step 4: Calibrate ICM45686

**IMPORTANT**: BNO085 self-calibrates automatically. ICM45686 needs manual calibration.

```bash
# Place robot on flat surface, keep stationary
python3 scripts/calibrate_icm45686.py
```

### Step 5: Configure

```bash
# Copy dual-IMU config template
cp example_config_dual_imu.json ~/duck_config.json

# Edit to choose primary IMU
nano ~/duck_config.json
```

**Choose your primary IMU:**

For BNO085 (recommended for beginners):
```json
{
  "imu_type": "bno085",
  "imu_fallback": "icm45686"
}
```

For ICM45686 (faster, but drifts without magnetometer):
```json
{
  "imu_type": "icm45686",
  "imu_fallback": "bno085"
}
```

### Step 6: Test Walking

```bash
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <your_model.onnx>
```

Done! 🎉

## Comparison Test

Want to see how both IMUs compare?

```bash
# Run both simultaneously for 30 seconds
python3 scripts/test_dual_imu.py --compare --duration 30
```

This will show:
- Real-time orientation from both IMUs
- Roll/Pitch/Yaw differences
- Mean/Max error statistics

**Expected results:**
- Roll/Pitch error: < 5° (excellent), < 10° (acceptable)
- Yaw error: < 15° (ICM45686 drifts without mag)

## Switching Between IMUs

Just edit `~/duck_config.json` and change `imu_type`:

```bash
# Switch to BNO085
nano ~/duck_config.json
# Change: "imu_type": "bno085"

# Or switch to ICM45686
# Change: "imu_type": "icm45686"

# Restart your walking script
```

**No code changes needed!** The config automatically loads the correct IMU.

## Quick Troubleshooting

### Multiplexer not detected
```bash
# Check wiring, then:
sudo i2cdetect -y 1
```

### IMU not responding
```bash
# Test individual IMU
python3 scripts/test_dual_imu.py --imu bno085
python3 scripts/test_dual_imu.py --imu icm45686
```

### Different orientations from each IMU
1. Check both are mounted same way (not one upside-down)
2. Recalibrate ICM45686
3. Let BNO085 calibrate (move through orientations for 30s)

### High yaw drift on ICM45686
This is **normal** - ICM45686 has no magnetometer. Expected drift: 1-2°/min.

Use BNO085 if you need accurate heading!

## Which IMU Should I Use?

| Use Case | Recommended IMU | Why? |
|----------|----------------|------|
| **General walking** | BNO085 | Auto-calibration, magnetometer, easier |
| **Fast response** | ICM45686 | Lower latency, higher update rate |
| **Outdoor navigation** | BNO085 | Magnetometer for absolute heading |
| **Indoor only** | Either | Both work great without mag |
| **Development** | Both | Compare, validate, redundancy |

**Pro tip**: Start with BNO085 (easier), switch to ICM45686 if you need performance.

## Next Steps

- Read [full multi-IMU guide](MULTI_IMU_SETUP.md) for advanced features
- Experiment with sensor fusion between both IMUs
- Add failover logic (auto-switch if one fails)
- Use both for redundancy in production

## Common Questions

**Q: Can I use both at the same time?**
A: Yes! See `test_dual_imu.py --compare` for example code.

**Q: Which is better?**
A: BNO085 for ease of use, ICM45686 for performance. Both excellent.

**Q: Do I need to calibrate BNO085?**
A: No! It auto-calibrates. Just move it around for 30s on first use.

**Q: Why does ICM45686 yaw drift?**
A: No magnetometer. This is normal and expected.

**Q: Can I add more IMUs?**
A: Yes! PCA9548A has 8 channels. Connect up to 8 devices.

**Q: What if one IMU fails during operation?**
A: Implement fallback logic in your code (check for exceptions, switch IMU).

## Reference Commands

```bash
# Test multiplexer
python3 -m mini_bdx_runtime.pca9548a

# Test ICM45686 driver
python3 -m mini_bdx_runtime.icm45686_driver

# Test BNO085 with mux
python3 -m mini_bdx_runtime.bno085_imu_mux

# Calibrate ICM45686
python3 scripts/calibrate_icm45686.py

# Compare both IMUs
python3 scripts/test_dual_imu.py --compare

# Check I2C devices
sudo i2cdetect -y 1
```

That's it! You now have a dual-IMU setup with automatic failover capability.
