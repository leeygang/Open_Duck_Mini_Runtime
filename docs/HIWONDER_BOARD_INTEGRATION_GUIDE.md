# Hiwonder Board Controller Integration Guide

This guide explains how to replace Feetech servos with Hiwonder servos using the board controller interface.

## Overview

The `hiwonder_board_hwi.py` module provides a drop-in replacement for the Feetech `rustypot_position_hwi.py` interface, allowing you to use Hiwonder servos with the board controller without modifying the main walking code.

## Key Differences

### Feetech (rustypot) vs Hiwonder Board Controller

| Feature | Feetech (rustypot) | Hiwonder Board |
|---------|-------------------|----------------|
| Communication | Direct servo commands | Board-level commands |
| Position Units | Radians | 0-1000 (converted internally) |
| Velocity Feedback | ✓ Yes | ✗ No (returns zeros) |
| PID Gains | ✓ Adjustable (kp, kd) | ✗ Not adjustable via protocol |
| Multi-Servo Move | Individual commands | ✓ Synchronized board command |
| Voltage Reading | Per-servo | Board-level only |
| Baudrate | 1,000,000 | 9600 (board commands) |
| Port | /dev/ttyACM0 | /dev/ttyUSB0 or /dev/serial0 |

## Step 1: Configure Servo IDs

First, configure your Hiwonder servos with appropriate IDs using the configuration script:

```bash
# Configure each servo one at a time
python3 scripts/configure_hiwonder_motor.py --port /dev/ttyUSB0 --id 100  # left_hip_yaw
python3 scripts/configure_hiwonder_motor.py --port /dev/ttyUSB0 --id 101  # left_hip_roll
# ... etc
```

**Recommended ID mapping:**

```python
# Default IDs in hiwonder_board_hwi.py:
{
    "left_hip_yaw": 100,
    "left_hip_roll": 101,
    "left_hip_pitch": 102,
    "left_knee": 103,
    "left_ankle": 104,
    "neck_pitch": 110,
    "head_pitch": 111,
    "head_yaw": 112,
    "head_roll": 113,
    "right_hip_yaw": 105,
    "right_hip_roll": 106,
    "right_hip_pitch": 107,
    "right_knee": 108,
    "right_ankle": 109,
}
```

**Why 100+ range?** To avoid conflicts with existing Feetech servo IDs (10-33).

## Step 2: Update Servo IDs in Code (Optional)

If you want to use different servo IDs, edit `mini_bdx_runtime/mini_bdx_runtime/hiwonder_board_hwi.py`:

```python
class HWI:
    def __init__(self, duck_config: DuckConfig, usb_port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        # ...
        self.joints = {
            "left_hip_yaw": 100,  # Change these to your servo IDs
            "left_hip_roll": 101,
            # ... etc
        }
```

## Step 3: Replace Import in Main Script

In your main walking script (e.g., `scripts/v2_rl_walk_mujoco.py`), change the import:

### Before (Feetech):
```python
from mini_bdx_runtime.rustypot_position_hwi import HWI
```

### After (Hiwonder Board):
```python
from mini_bdx_runtime.hiwonder_board_hwi import HWI
```

### Update Initialization

Change the initialization parameters:

### Before (Feetech):
```python
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyACM0")
```

### After (Hiwonder Board):
```python
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)
```

Or with GPIO UART:
```python
hwi = HWI(duck_config=duck_config, usb_port="/dev/serial0", baudrate=9600)
```

## Step 4: Test Basic Functionality

Test the integration with the turn on/off scripts:

### Test Turn On

Create a test script `scripts/test_hiwonder_hwi_turn_on.py`:

```python
#!/usr/bin/env python3
"""Test Hiwonder HWI turn_on functionality"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.hiwonder_board_hwi import HWI

duck_config = DuckConfig()
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)

print("Testing turn_on...")
hwi.turn_on()

print("\nPress Ctrl+C to exit and turn off servos")
try:
    input("Press Enter to turn off servos...")
except KeyboardInterrupt:
    print("\nCaught Ctrl+C")

print("Turning off servos...")
hwi.turn_off()
hwi.close()
```

Run it:
```bash
python3 scripts/test_hiwonder_hwi_turn_on.py
```

### Test Position Reading

```python
#!/usr/bin/env python3
"""Test Hiwonder HWI position reading"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_bdx_runtime'))

from mini_bdx_runtime.duck_config import DuckConfig
from mini_bdx_runtime.hiwonder_board_hwi import HWI

duck_config = DuckConfig()
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)

print("Reading current positions...")
positions = hwi.get_present_positions()

if positions is not None:
    print("\nCurrent joint positions (radians):")
    for joint_name, pos in zip(hwi.joints.keys(), positions):
        print(f"  {joint_name:20s}: {pos:7.3f} rad")
else:
    print("Failed to read positions")

voltage = hwi.get_battery_voltage()
if voltage:
    print(f"\nBattery voltage: {voltage:.2f}V")

hwi.close()
```

## Step 5: Calibrate Offsets

Use the existing offset calibration workflow, but you'll need to manually adjust each servo to its mechanical zero:

```bash
# For each servo, you'll need to:
# 1. Move servo to center position (500 units)
# 2. Unload servo (disable torque)
# 3. Manually adjust to mechanical zero
# 4. Record the offset in duck_config.json
```

Or create a Hiwonder-specific calibration script.

## Step 6: Run Full Integration Test

### Option A: Test with Existing turn_on/turn_off Scripts

Modify `scripts/turn_on.py`:

```python
# Change import:
from mini_bdx_runtime.hiwonder_board_hwi import HWI

# Update initialization:
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)
```

### Option B: Test with Walking Script

```bash
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <path_to_model>
```

## Complete Example: Modified v2_rl_walk_mujoco.py

Here's how to modify the main walking script:

```python
# At the top of the file, replace the import:
# OLD: from mini_bdx_runtime.rustypot_position_hwi import HWI
# NEW:
from mini_bdx_runtime.hiwonder_board_hwi import HWI

# In the RLWalk class __init__ method, update the HWI initialization:
class RLWalk:
    def __init__(self, args):
        # ... other initialization code ...

        # Initialize hardware interface
        # OLD: self.hwi = HWI(duck_config=self.duck_config, usb_port="/dev/ttyACM0")
        # NEW:
        self.hwi = HWI(
            duck_config=self.duck_config,
            usb_port="/dev/ttyUSB0",  # or "/dev/serial0" for GPIO UART
            baudrate=9600
        )

        # ... rest of initialization ...
```

That's it! The rest of the code remains unchanged because the HWI interface is identical.

## Important Notes

### 1. Velocity Observations

Hiwonder board protocol doesn't provide velocity feedback. The `get_present_velocities()` method returns zeros.

**If your RL policy uses velocity observations:**
- Option A: Estimate velocity from position changes (differentiate positions)
- Option B: Retrain policy without velocity observations
- Option C: Use a simple differentiator filter

### 2. Control Frequency

The default movement duration is 20ms (50Hz control rate). You can adjust this:

```python
hwi = HWI(duck_config=duck_config, usb_port="/dev/ttyUSB0", baudrate=9600)
hwi.default_move_duration_ms = 30  # Change to 30ms (33Hz)
```

### 3. PID Gains

Hiwonder servos don't expose PID gain adjustment via the board protocol. The `set_kps()` and `set_kds()` methods are no-ops for API compatibility.

### 4. Multi-Servo Synchronization

Hiwonder board controller's `CMD_SERVO_MOVE` command moves all servos simultaneously, providing better synchronization than sending individual commands.

### 5. Position Range

- Hiwonder native: 0-1000 units (240° range)
- HWI interface: Radians (converted automatically)
- Center position: 500 units = 0 radians

### 6. Baudrate

The board controller uses **9600 baud** for board-level commands (not 115200).

## Troubleshooting

### "Failed to read servo positions"

Check:
- All servos are connected and powered (6-8.4V)
- Servo IDs in code match actual servo IDs
- Battery voltage is sufficient (>6V)
- Port and baudrate are correct

### "Servos don't move smoothly"

Try:
- Increase `move_duration_ms` (e.g., 30-50ms)
- Check battery voltage under load
- Verify servo torque is sufficient for the joint

### "Control loop too slow"

The board protocol reads all servos in one command, which is faster than individual reads. However:
- Reading 14 servos still takes time
- Consider lower control frequency (e.g., 25Hz instead of 50Hz)

### "ValueError: Unknown joint"

Make sure the joint names in `self.joints` dictionary match those used in your walking policy and `duck_config.json`.

## Summary of Files Modified

1. **New file**: `mini_bdx_runtime/mini_bdx_runtime/hiwonder_board_hwi.py` - Main HWI implementation
2. **Modified**: `scripts/v2_rl_walk_mujoco.py` - Change import from `rustypot_position_hwi` to `hiwonder_board_hwi`
3. **Modified**: `scripts/turn_on.py` - Update import and initialization
4. **Modified**: `scripts/turn_off.py` - Update import and initialization
5. **Optional**: Create test scripts for validation

## Reference

- **HWI Implementation**: `mini_bdx_runtime/mini_bdx_runtime/hiwonder_board_hwi.py`
- **Board Controller**: `mini_bdx_runtime/mini_bdx_runtime/hiwonder_board_controller.py`
- **Board Commands**: `HIWONDER_BOARD_COMMANDS.md`
- **Setup Guide**: `HIWONDER_SETUP_GUIDE.txt`
- **Original Feetech HWI**: `mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py`
