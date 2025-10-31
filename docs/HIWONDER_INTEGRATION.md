# Hiwonder HTD-45H Servo Integration Guide

This guide explains how to integrate and calibrate Hiwonder HTD-45H serial bus servos with your Duck Mini robot.

## Hardware Overview

**Hiwonder HTD-45H Specifications:**
- Voltage: 6.0-8.4V
- Protocol: Half-duplex serial (LewanSoul/LX protocol)
- Position range: 0-1000 (240° total range)
- Default baud rate: 115200
- Default ID: 1
- Position feedback: Yes (position, voltage, temperature)
- Communication: UART serial (daisy-chainable like Feetech)

## Physical Connection

### Option 1: Using USB-to-TTL Adapter (Recommended)

The Hiwonder servos must be connected through a **Hiwonder Bus Servo Controller** board (Serial Bus Servo Tester Board), which then connects to the Raspberry Pi via a USB-to-TTL adapter.

**Hardware Architecture:**
```
Raspberry Pi
    ↓ (USB cable)
USB-to-TTL Adapter (CP2102, FT232, CH340, etc.)
    ↓ (Serial: TX, RX, GND)
Hiwonder Bus Servo Control Board (serial pins on board)
    ↓ (Servo connectors - daisy-chained)
Hiwonder Servos (ID 100, 101, 102, ...)
    ↓ (External 6-8.4V power supply)
```

**Important:** The Hiwonder control board typically has only a USB-A port (host port), which CANNOT connect directly to Raspberry Pi. Instead, use the serial pins (TX, RX, GND) on the control board.

**Connection Steps:**
1. **USB-to-TTL Adapter → Control Board Serial Pins:**
   - Adapter TX → Control Board RX
   - Adapter RX → Control Board TX
   - Adapter GND → Control Board GND
   - Do NOT connect power between adapter and control board

2. **Servos → Control Board:**
   - Daisy-chain your Hiwonder servos to the control board's servo connectors

3. **Power Supply → Control Board:**
   - Connect external 6-8.4V power supply to the control board
   - Do NOT power servos from USB 5V

4. **USB-to-TTL Adapter → Raspberry Pi:**
   - Connect the adapter to a USB port on the Raspberry Pi
   - The adapter will appear as `/dev/ttyUSB0` (or `/dev/ttyUSB1`, etc.) on your Pi

**Important Notes:**
- Always use an external 6-8.4V power supply for servos
- The control board passes serial commands to servos (transparent pass-through)
- No special configuration needed on the Pi (no GPIO UART setup, no Bluetooth disabling)
- USB-to-TTL adapter recommendations: CP2102, FT232RL, CH340G

### Option 2: Using Raspberry Pi GPIO UART (Temporary Alternative)

If you don't have a USB-to-TTL adapter available temporarily, you can connect directly to Raspberry Pi GPIO UART pins.

**Hardware Architecture:**
```
Raspberry Pi GPIO Pins (14 & 15)
    ↓ (Direct wiring: TX, RX, GND)
Hiwonder Bus Servo Control Board (serial pins on board)
    ↓ (Servo connectors - daisy-chained)
Hiwonder Servos (ID 100, 101, 102, ...)
    ↓ (External 6-8.4V power supply)
```

**⚠️ Important Notes:**
- This method requires disabling Bluetooth on Raspberry Pi
- Xbox controller won't work over Bluetooth if you disable it
- Recommended only as a temporary solution until you get a USB-to-TTL adapter
- Raspberry Pi 4/5 use 3.3V logic (compatible with most Hiwonder control boards)

**Connection Steps:**

1. **Wiring Raspberry Pi GPIO → Control Board:**
   - Pi GPIO 14 (TXD, Pin 8) → Control Board RX
   - Pi GPIO 15 (RXD, Pin 10) → Control Board TX
   - Pi GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39) → Control Board GND
   - Do NOT connect 3.3V or 5V power pins

   **GPIO Pin Layout Reference:**
   ```
   Pin 1  [3.3V]     [5V]      Pin 2
   Pin 3  [GPIO 2]   [5V]      Pin 4
   Pin 5  [GPIO 3]   [GND]     Pin 6
   Pin 7  [GPIO 4]   [GPIO 14 - TXD] Pin 8  ← TX
   Pin 9  [GND]      [GPIO 15 - RXD] Pin 10 ← RX
   ```

2. **Enable Serial Port on Raspberry Pi:**
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options → Serial Port
   # "Would you like a login shell accessible over serial?" → No
   # "Would you like the serial port hardware enabled?" → Yes
   # Reboot when prompted
   ```

3. **Disable Bluetooth (Required for GPIO UART):**
   ```bash
   # Edit boot config
   sudo nano /boot/config.txt

   # Add these lines at the end:
   dtoverlay=disable-bt
   enable_uart=1

   # Save and exit (Ctrl+X, Y, Enter)

   # Disable Bluetooth service
   sudo systemctl disable hciuart.service
   sudo systemctl disable bluetooth.service

   # Reboot
   sudo reboot
   ```

4. **Verify Serial Port:**
   ```bash
   ls -l /dev/serial*
   # Should show: /dev/serial0 -> ttyAMA0 (or similar)

   # You can use either:
   # /dev/serial0 (recommended, symlink)
   # /dev/ttyAMA0 (direct device)
   # /dev/ttyS0 (on some Pi models)
   ```

5. **Test Connection:**
   ```bash
   # Test with Hiwonder control board
   python3 -c "from mini_bdx_runtime.hiwonder_hwi import HiwonderServo; \
   s = HiwonderServo('/dev/serial0'); \
   print('Found servos:', s.scan_servos(max_id=10)); \
   s.close()"
   ```

**When using GPIO UART in scripts:**
```bash
# Use /dev/serial0 instead of /dev/ttyUSB0
python3 scripts/configure_hiwonder_motor.py --port /dev/serial0 --id 100
python3 scripts/check_hiwonder_motors.py  # (enter /dev/serial0 when prompted)
python3 scripts/find_hiwonder_offsets.py  # (enter /dev/serial0 when prompted)
```

**Re-enabling Bluetooth Later:**
```bash
# When you get a USB-to-TTL adapter and want to re-enable Bluetooth:
sudo nano /boot/config.txt
# Remove or comment out these lines:
# dtoverlay=disable-bt
# enable_uart=1

# Re-enable Bluetooth service
sudo systemctl enable hciuart.service
sudo systemctl enable bluetooth.service

# Reboot
sudo reboot
```

### Mixed with Feetech Servos

If using both Feetech and Hiwonder servos:
- Feetech servos → `/dev/ttyACM0` (Feetech motor control board, connects directly via USB)
- Hiwonder servos → `/dev/ttyUSB0` (USB-to-TTL adapter connected to Hiwonder control board)

Keep them on separate serial buses to avoid protocol conflicts.

## Installation

The Hiwonder interface module is already included. Just install PySerial:

```bash
pip install pyserial
```

No additional dependencies needed!

## Step-by-Step Calibration

### Step 1: Test Connection

First, verify your servo is connected and communicating:

```bash
python3 -c "from mini_bdx_runtime.hiwonder_hwi import HiwonderServo; \
s = HiwonderServo('/dev/ttyUSB0'); \
found = s.scan_servos(max_id=10); \
print(f'Found servos: {found}'); \
s.close()"
```

You should see: `Found servos: [1]` (or whatever IDs are connected)

### Step 2: Configure Servo ID

Brand new Hiwonder servos have ID 1. If you're adding multiple servos, give each a unique ID:

```bash
# Connect ONLY ONE servo at a time
python3 scripts/configure_hiwonder_motor.py --id 100

# For additional servos:
python3 scripts/configure_hiwonder_motor.py --id 101
python3 scripts/configure_hiwonder_motor.py --id 102
# etc.
```

**ID Convention Recommendations:**
- Feetech servos: 10-33 (existing)
- Hiwonder servos: 100-199 (to avoid conflicts)

The script will:
1. Scan for the servo (tries ID 1 first)
2. Show current status (position, voltage, temperature)
3. Test movement
4. Change the ID
5. Verify the new ID works

### Step 3: Add to Code Configuration

Edit `mini_bdx_runtime/mini_bdx_runtime/hiwonder_hwi.py`:

Find the `__init__` method in the `HiwonderHWI` class and update the `self.joints` dictionary:

```python
self.joints = {
    "hiwonder_joint_1": 100,  # Your first Hiwonder servo
    "hiwonder_joint_2": 101,  # Your second Hiwonder servo
    # Add more as needed
}
```

Also update the `init_pos` dictionary with your desired standing positions (in radians):

```python
self.init_pos = {
    "hiwonder_joint_1": 0.0,     # Example: straight
    "hiwonder_joint_2": 0.524,   # Example: 30 degrees
}
```

### Step 4: Test Connectivity

Now test that all your Hiwonder servos are responding:

```bash
python3 scripts/check_hiwonder_motors.py
```

This will:
- Scan for all servos
- Check each configured joint
- Read position, voltage, temperature
- Optionally test movement for each servo

### Step 5: Calibrate Offsets

This is the most important step! It compensates for mechanical assembly variations.

```bash
python3 scripts/find_hiwonder_offsets.py
```

**The calibration process:**

For each servo:
1. Script moves servo to center position (500)
2. Script turns off servo torque
3. **You manually move the servo to the mechanically correct zero position**
4. Press Enter
5. Script calculates the offset
6. Script applies the offset to the servo's EEPROM
7. Script tests the result - verify it looks correct
8. Confirm or retry

**Important Notes:**
- Hiwonder servos store offsets in their internal EEPROM (persistent)
- The offsets are applied at the hardware level
- You can also add software offsets in `duck_config.json` if needed

Example output:
```
Calculated offsets:
  hiwonder_joint_1: -15
  hiwonder_joint_2: 23

For duck_config.json (optional software offsets in radians):
"joints_offsets": {
    "hiwonder_joint_1": -0.0628,
    "hiwonder_joint_2": 0.0963,
}
```

### Step 6: Integration Options

Now you have three options for integrating your Hiwonder servos:

#### Option A: Standalone Feature (Like Antennas)

Use Hiwonder servos for expression features independent of the main locomotion:

**File: `mini_bdx_runtime/mini_bdx_runtime/your_feature.py`**
```python
from mini_bdx_runtime.hiwonder_hwi import HiwonderHWI
from mini_bdx_runtime.duck_config import DuckConfig

class YourFeature:
    def __init__(self, duck_config):
        self.hwi = HiwonderHWI(duck_config, usb_port="/dev/ttyUSB0")
        self.hwi.turn_on()

    def do_something(self, value):
        # Control your Hiwonder servos
        self.hwi.set_position("hiwonder_joint_1", value)

    def stop(self):
        self.hwi.turn_off()
        self.hwi.close()
```

Add to `duck_config.json`:
```json
{
    "expression_features": {
        "your_feature": true
    }
}
```

Use in `v2_rl_walk_mujoco.py`:
```python
if self.duck_config.your_feature:
    from mini_bdx_runtime.your_feature import YourFeature
    self.your_feature = YourFeature(self.duck_config)
```

#### Option B: Mixed with Main Locomotion

Create a hybrid hardware interface that controls both Feetech and Hiwonder servos:

**File: `mini_bdx_runtime/mini_bdx_runtime/hybrid_hwi.py`**
```python
from mini_bdx_runtime.rustypot_position_hwi import HWI as FeetechHWI
from mini_bdx_runtime.hiwonder_hwi import HiwonderHWI
import numpy as np

class HybridHWI:
    """Control both Feetech and Hiwonder servos"""

    def __init__(self, duck_config):
        # Feetech motor control board on /dev/ttyACM0 (connects directly via USB)
        self.feetech = FeetechHWI(duck_config, usb_port="/dev/ttyACM0")

        # USB-to-TTL adapter on /dev/ttyUSB0 (wired to Hiwonder control board serial pins)
        self.hiwonder = HiwonderHWI(duck_config, usb_port="/dev/ttyUSB0")

        # Combined joints
        self.joints = {**self.feetech.joints, **self.hiwonder.joints}
        self.init_pos = {**self.feetech.init_pos, **self.hiwonder.init_pos}

    def turn_on(self):
        self.feetech.turn_on()
        self.hiwonder.turn_on()

    def turn_off(self):
        self.feetech.turn_off()
        self.hiwonder.turn_off()

    def set_position_all(self, positions_dict):
        # Split commands
        feetech_pos = {k: v for k, v in positions_dict.items()
                       if k in self.feetech.joints}
        hiwonder_pos = {k: v for k, v in positions_dict.items()
                        if k in self.hiwonder.joints}

        if feetech_pos:
            self.feetech.set_position_all(feetech_pos)
        if hiwonder_pos:
            self.hiwonder.set_position_all(hiwonder_pos)

    def get_present_positions(self, ignore=None):
        if ignore is None:
            ignore = []

        feetech_pos = self.feetech.get_present_positions(
            ignore=[j for j in ignore if j in self.feetech.joints]
        )
        hiwonder_pos = self.hiwonder.get_present_positions(
            ignore=[j for j in ignore if j in self.hiwonder.joints]
        )

        if feetech_pos is None or hiwonder_pos is None:
            return None

        return np.concatenate([feetech_pos, hiwonder_pos])

    def get_present_velocities(self, rad_s=True, ignore=None):
        # Note: Hiwonder doesn't provide velocity feedback
        # Returns zeros for Hiwonder servos
        if ignore is None:
            ignore = []

        feetech_vel = self.feetech.get_present_velocities(rad_s, ignore)
        hiwonder_vel = self.hiwonder.get_present_velocities(rad_s, ignore)

        if feetech_vel is None:
            return None

        return np.concatenate([feetech_vel, hiwonder_vel])
```

Then in `v2_rl_walk_mujoco.py`, replace:
```python
from mini_bdx_runtime.rustypot_position_hwi import HWI
self.hwi = HWI(self.duck_config, serial_port)
```

With:
```python
from mini_bdx_runtime.hybrid_hwi import HybridHWI
self.hwi = HybridHWI(self.duck_config)
```

#### Option C: Replace Feetech Entirely

If you're replacing all Feetech servos with Hiwonder:

Simply modify `v2_rl_walk_mujoco.py`:
```python
from mini_bdx_runtime.hiwonder_hwi import HiwonderHWI as HWI
self.hwi = HWI(self.duck_config, usb_port="/dev/ttyUSB0")
```

And update all joint names in `hiwonder_hwi.py` to match the existing naming convention.

## Troubleshooting

### Servo Not Found

1. **Check power**: Hiwonder servos need 6-8.4V external power
2. **Check connections**: Verify TX/RX wiring (might be swapped)
3. **Check port**: Try `ls /dev/tty*` to see available ports
4. **Check baud rate**: HTD-45H uses 115200, but older models might use 9600 or 38400
5. **Try scanning**: Run the test script with scanning enabled

### Position Errors / Jittery Movement

1. **Calibrate offsets**: Run `find_hiwonder_offsets.py`
2. **Check voltage**: Low voltage causes poor performance
3. **Check load**: Hiwonder servos are smaller than Feetech - might not have enough torque for heavy joints
4. **Adjust movement duration**: Increase the `duration` parameter in `move_servo()` calls

### Communication Errors

1. **Checksum errors**: Bad wiring or electrical noise
2. **Timeout errors**: Servo may be disconnected or ID is wrong
3. **Protocol conflicts**: Don't mix Hiwonder and Feetech on the same serial bus

### Position Drift

Hiwonder servos don't have as much holding torque as Feetech. For main locomotion joints, you might need:
- Higher control frequency
- Load-appropriate servo selection (HTD-45H is relatively small)

## Technical Notes

### Position Units

- **Hiwonder**: 0-1000 (500 = center, 240° total range)
- **Feetech**: Radians
- **Conversion**: `hiwonder_hwi.py` handles conversion automatically

### Velocity Feedback

⚠️ **Important**: Hiwonder servos do NOT provide velocity feedback!

The `get_present_velocities()` method returns zeros. If your RL policy requires velocity observations, you'll need to:
1. Estimate velocity from position changes
2. Use a differentiator filter
3. Or retrain the policy without velocity observations for Hiwonder joints

### Real-Time Performance

Hiwonder serial communication at 115200 baud is fast enough for 50Hz control, but:
- Reading position takes ~5ms per servo
- Reading 10 Hiwonder servos = 50ms (exceeds 20ms budget!)

**Solutions:**
- Don't read all servos every frame
- Use bulk read commands (not yet implemented)
- Increase control loop period (lower frequency)

### EEPROM Wear

Be careful with offset calibration - Hiwonder servos store offsets in EEPROM, which has limited write cycles (~100,000). Don't repeatedly write offsets unnecessarily.

## Quick Reference

```bash
# 1. Scan for servos
python3 -c "from mini_bdx_runtime.hiwonder_hwi import HiwonderServo; s = HiwonderServo('/dev/ttyUSB0'); print(s.scan_servos()); s.close()"

# 2. Configure servo ID
python3 scripts/configure_hiwonder_motor.py --id 100

# 3. Check all servos
python3 scripts/check_hiwonder_motors.py

# 4. Calibrate offsets
python3 scripts/find_hiwonder_offsets.py

# 5. Test in code
python3 -m mini_bdx_runtime.hiwonder_hwi  # Runs built-in test
```

## Further Reading

- [Hiwonder Official Documentation](https://www.hiwonder.com/)
- [LewanSoul Servo Protocol](https://github.com/ccourson/LX-16A-Servo)
- Original Feetech integration: `rustypot_position_hwi.py`
