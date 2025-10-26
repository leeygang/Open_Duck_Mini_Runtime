# Hiwonder Integration Summary

This document provides a quick overview of the Hiwonder servo integration for the Duck Mini robot.

## What Was Implemented

### 1. Board Controller Protocol (`hiwonder_board_controller.py`)
A low-level implementation of the Hiwonder Bus Servo Controller protocol with 4 commands:
- **CMD_SERVO_MOVE (0x03)**: Move multiple servos simultaneously
- **CMD_GET_BATTERY_VOLTAGE (0x0F)**: Read board battery voltage
- **CMD_MULT_SERVO_UNLOAD (0x14)**: Disable torque on multiple servos
- **CMD_MULT_SERVO_POS_READ (0x15)**: Read positions of multiple servos

### 2. Hardware Interface (`hiwonder_board_hwi.py`)
A drop-in replacement for `rustypot_position_hwi.py` that provides:
- Same API as Feetech HWI (compatible with existing code)
- Automatic conversion between radians and servo units (0-1000)
- Synchronized multi-servo movements via board commands
- Position reading for all joints
- Battery voltage monitoring

### 3. Test and Configuration Scripts
- `scripts/test_hiwonder_board.py` - Test board controller commands
- `scripts/test_hiwonder_hwi.py` - Test HWI interface
- `scripts/configure_hiwonder_motor.py` - Configure servo IDs (already exists)

### 4. Documentation
- `HIWONDER_BOARD_COMMANDS.md` - Protocol reference and examples
- `HIWONDER_BOARD_INTEGRATION_GUIDE.md` - Complete integration guide
- This summary document

## Quick Start

### Step 1: Hardware Setup
```
Raspberry Pi
    ↓ (USB cable)
USB-to-TTL Adapter (CP2102, FT232, CH340)
    ↓ (Serial: TX, RX, GND)
Hiwonder Bus Servo Control Board (serial pins)
    ↓ (Servo connectors)
Hiwonder Servos (daisy-chained, 6-8.4V external power)
```

**Port**: `/dev/ttyUSB0` or `/dev/serial0` (GPIO UART)
**Baudrate**: `9600` (board commands)

### Step 2: Configure Servo IDs
```bash
# Configure each servo with unique ID (one at a time)
python3 scripts/configure_hiwonder_motor.py --port /dev/ttyUSB0 --baudrate 9600 --id 100
python3 scripts/configure_hiwonder_motor.py --port /dev/ttyUSB0 --baudrate 9600 --id 101
# ... etc for all servos
```

### Step 3: Test Board Controller
```bash
# Test board-level commands
python3 scripts/test_hiwonder_board.py --port /dev/ttyUSB0 --baudrate 9600 --servo-ids 100,101,102
```

### Step 4: Test HWI Interface
```bash
# Test hardware interface
python3 scripts/test_hiwonder_hwi.py --port /dev/ttyUSB0 --baudrate 9600
```

### Step 5: Replace Feetech in Main Code

In your walking script (e.g., `scripts/v2_rl_walk_mujoco.py`):

**Before:**
```python
from mini_bdx_runtime.rustypot_position_hwi import HWI

hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyACM0")
```

**After:**
```python
from mini_bdx_runtime.hiwonder_board_hwi import HWI

hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)
```

That's it! No other code changes needed.

## Default Servo ID Mapping

```python
{
    "left_hip_yaw": 100,
    "left_hip_roll": 101,
    "left_hip_pitch": 102,
    "left_knee": 103,
    "left_ankle": 104,
    "right_hip_yaw": 105,
    "right_hip_roll": 106,
    "right_hip_pitch": 107,
    "right_knee": 108,
    "right_ankle": 109,
    "neck_pitch": 110,
    "head_pitch": 111,
    "head_yaw": 112,
    "head_roll": 113,
}
```

**Why 100+?** To avoid conflicts with Feetech servo IDs (10-33).

## Key Differences from Feetech

| Feature | Feetech | Hiwonder Board |
|---------|---------|----------------|
| **Position Units** | Radians | 0-1000 (auto-converted) |
| **Velocity Feedback** | ✓ Yes | ✗ No (returns zeros) |
| **PID Gains** | ✓ Adjustable | ✗ Not adjustable |
| **Multi-Servo Move** | Sequential | ✓ Synchronized |
| **Baudrate** | 1,000,000 | 9600 |
| **Port** | /dev/ttyACM0 | /dev/ttyUSB0 |

## Important Notes

### 1. No Velocity Feedback
The board protocol doesn't provide velocity data. If your RL policy uses velocities:
- Estimate from position differences
- Or retrain without velocity observations

### 2. PID Gains Not Adjustable
`set_kps()` and `set_kds()` are no-ops (kept for API compatibility only).

### 3. Position Range
- Hiwonder: 0-1000 units = 240° = -120° to +120°
- Center: 500 units = 0°
- HWI converts automatically between radians and units

### 4. Control Frequency
Default movement duration: 20ms (50Hz). Adjust via:
```python
hwi.default_move_duration_ms = 30  # Change to 30ms (33Hz)
```

### 5. Synchronized Movement
`set_position_all()` uses `CMD_SERVO_MOVE` which moves all servos simultaneously with better synchronization than Feetech.

## File Structure

```
mini_bdx_runtime/mini_bdx_runtime/
├── hiwonder_board_controller.py    # Low-level board protocol
├── hiwonder_board_hwi.py           # HWI interface (drop-in replacement)
└── rustypot_position_hwi.py        # Original Feetech HWI

scripts/
├── test_hiwonder_board.py          # Test board commands
├── test_hiwonder_hwi.py            # Test HWI interface
└── configure_hiwonder_motor.py     # Configure servo IDs

Documentation/
├── HIWONDER_BOARD_COMMANDS.md              # Protocol reference
├── HIWONDER_BOARD_INTEGRATION_GUIDE.md     # Full integration guide
├── HIWONDER_INTEGRATION_SUMMARY.md         # This file
└── HIWONDER_SETUP_GUIDE.txt                # Hardware setup
```

## Troubleshooting

### "Could not read voltage"
- Check port is correct (`/dev/ttyUSB0` or `/dev/serial0`)
- Verify baudrate is 9600
- Check board is powered and connected

### "Could not read positions"
- Ensure external power supply (6-8.4V) is connected
- Verify servo IDs in code match actual servo IDs
- Check battery voltage is sufficient (>6V)

### "Servos don't move"
- Check external power supply
- Verify servos are not unloaded
- Check servo IDs are correct

### "Control loop too slow"
- Lower control frequency (e.g., 25Hz instead of 50Hz)
- Increase movement duration (`move_duration_ms`)

## Testing Checklist

- [ ] Hardware connected (USB-to-TTL adapter + board + servos + power)
- [ ] All servos configured with unique IDs (100-113)
- [ ] Board controller test passes (`test_hiwonder_board.py`)
- [ ] HWI test passes (`test_hiwonder_hwi.py`)
- [ ] Servo IDs in `hiwonder_board_hwi.py` match your configuration
- [ ] Joint offsets calibrated in `duck_config.json`
- [ ] Walking script import updated to use `hiwonder_board_hwi`
- [ ] Full integration test with walking policy

## Next Steps

1. **Calibrate joint offsets**: Adjust `joints_offsets` in `duck_config.json`
2. **Test with walking policy**: Run `v2_rl_walk_mujoco.py` with ONNX model
3. **Fine-tune parameters**: Adjust control frequency and movement duration
4. **Monitor performance**: Check battery voltage and servo temperatures

## Reference Documents

- **Protocol Reference**: `HIWONDER_BOARD_COMMANDS.md`
- **Integration Guide**: `HIWONDER_BOARD_INTEGRATION_GUIDE.md`
- **Hardware Setup**: `HIWONDER_SETUP_GUIDE.txt`
- **Main Documentation**: `CLAUDE.md`

## Support

For issues or questions:
1. Check troubleshooting section in `HIWONDER_BOARD_INTEGRATION_GUIDE.md`
2. Review protocol details in `HIWONDER_BOARD_COMMANDS.md`
3. Test components individually with provided test scripts
4. Verify hardware connections and power supply
