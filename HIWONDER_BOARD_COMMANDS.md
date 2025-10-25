# Hiwonder Bus Servo Controller Board Commands

This document explains the difference between **board commands** and **servo commands**, and how to use both.

## Understanding the System

The Hiwonder Bus Servo Controller system has **two levels of control**:

```
Your Computer
    ↓ (Serial Commands)
┌─────────────────────────────────────┐
│ Hiwonder Bus Servo Controller Board │ ← Board-level commands
│  - Power management                 │
│  - Multi-servo synchronization      │
│  - Board status/voltage             │
└─────────────────────────────────────┘
    ↓ (Servo Protocol)
Hiwonder Servos (ID 1, 2, 3, ...)    ← Servo-level commands
```

## Two Types of Commands

### 1. Board Commands (hiwonder_board_controller.py)

These commands control the **board itself**, not individual servos:

- **Get board information** - Firmware version, board type
- **Power management** - Turn servo power on/off
- **Read board voltage** - Monitor input voltage
- **Multi-servo sync** - Move multiple servos simultaneously
- **Board reset** - Reset the controller board

**Usage:**
```python
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController

board = HiwonderBoardController(port="/dev/ttyUSB0")

# Get board info
info = board.get_board_info()
print(f"Firmware: {info['firmware_version']}")

# Read voltage
voltage = board.read_board_voltage()
print(f"Voltage: {voltage:.2f}V")

# Control power
board.set_servo_power(True)  # Turn on
board.set_servo_power(False) # Turn off

# Move multiple servos at once (synchronized)
board.multi_servo_move([
    (1, 500, 1000),  # Servo 1 to position 500 in 1000ms
    (2, 600, 1000),  # Servo 2 to position 600 in 1000ms
    (3, 400, 1000),  # Servo 3 to position 400 in 1000ms
])

board.close()
```

### 2. Servo Commands (hiwonder_hwi.py)

These commands communicate directly with **individual servos** through the board:

- **Read/write position** - Control servo position
- **Read voltage/temperature** - Monitor servo status
- **Set servo ID** - Configure servo ID
- **Set angle offset** - Calibrate servo zero position
- **Control torque** - Enable/disable servo torque

**Usage:**
```python
from mini_bdx_runtime.hiwonder_hwi import HiwonderServo

servo = HiwonderServo(port="/dev/ttyUSB0")

# Move single servo
servo.move_servo(servo_id=1, position=500, duration=1000)

# Read position
pos = servo.read_position(servo_id=1)

# Read voltage and temperature
voltage = servo.read_voltage(servo_id=1)
temp = servo.read_temperature(servo_id=1)

# Change servo ID
servo.set_servo_id(old_id=1, new_id=100)

servo.close()
```

## When to Use Which

### Use Board Commands When:
- ✅ You need to control power to all servos at once
- ✅ You want to move multiple servos simultaneously (synchronized)
- ✅ You need to check board voltage/status
- ✅ You're doing board-level management

### Use Servo Commands When:
- ✅ You need to control individual servos
- ✅ You want to read servo-specific data (position, temp, voltage)
- ✅ You're configuring servo IDs or offsets
- ✅ You're doing normal servo operation

## Testing Board Commands

Test if your board supports board-level commands:

```bash
# Test board commands
python3 scripts/test_hiwonder_board.py --port /dev/ttyUSB0

# Or with GPIO UART
python3 scripts/test_hiwonder_board.py --port /dev/serial0
```

**Note:** Some Hiwonder control boards may act as **transparent pass-through only** and might not support all board-level commands. In that case, just use the servo commands (hiwonder_hwi.py) which will work through the board.

## Protocol Details

### Board Command Frame Format:
```
Header(2) + Board_ID(1) + Length(1) + CMD(1) + Params(n) + Checksum(1)

Header:   0x55 0x55
Board_ID: 0xFE (fixed, different from servo IDs 0x01-0xFD)
Length:   Number of bytes from ID to checksum
CMD:      Command byte
Params:   Command parameters (variable length)
Checksum: (~sum(ID to Params)) & 0xFF
```

### Servo Command Frame Format:
```
Header(2) + Servo_ID(1) + Length(1) + CMD(1) + Params(n) + Checksum(1)

Header:    0x55 0x55
Servo_ID:  0x01-0xFD (specific servo), 0xFE (broadcast)
Length:    Number of bytes from ID to checksum
CMD:       Command byte
Params:    Command parameters (variable length)
Checksum:  (~sum(ID to Params)) & 0xFF
```

**Key Difference:** Board commands use ID `0xFE`, servo commands use IDs `0x01-0xFD`.

## Available Board Commands

| Command | Code | Description |
|---------|------|-------------|
| Get Board Info | 0x01 | Query firmware version |
| Set Servo Power | 0x02 | Turn servo power on/off |
| Get Servo Power | 0x03 | Query power status |
| Set Servo Offset | 0x04 | Set servo offset |
| Get Servo Offset | 0x05 | Read servo offset |
| Save Offset | 0x06 | Save offset to EEPROM |
| Read Voltage | 0x07 | Read board voltage |
| Multi Servo Move | 0x08 | Synchronized multi-servo move |
| Multi Servo Unload | 0x09 | Disable multiple servos |
| Servo Mode | 0x0A | Set position/rotation mode |
| Reset Board | 0xFF | Reset controller board |

## Troubleshooting

### "Could not read board info/voltage/power"

This likely means:
1. Your board version might not support these specific commands
2. The board is acting as a transparent pass-through only
3. Different protocol version or variant

**Solution:** Just use the servo commands (`hiwonder_hwi.py`) which work through the board regardless of board command support.

### Board commands work but servo commands don't

Make sure:
1. Servo power is enabled: `board.set_servo_power(True)`
2. Servos are properly connected to the board
3. External power supply (6-8.4V) is connected

### Both board and servo commands fail

Check:
1. Serial port is correct (`/dev/ttyUSB0` or `/dev/serial0`)
2. Board is powered and connected
3. Baud rate is correct (115200 is standard)
4. No permission issues (`sudo usermod -a -G dialout $USER`)

## Examples

### Complete Example: Power On and Move Servos

```python
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController
from mini_bdx_runtime.hiwonder_hwi import HiwonderServo
import time

# Initialize board controller
board = HiwonderBoardController(port="/dev/ttyUSB0")

# Turn on servo power
print("Turning on servo power...")
board.set_servo_power(True)
time.sleep(0.5)

# Check voltage
voltage = board.read_board_voltage()
print(f"Board voltage: {voltage:.2f}V")

# Now use servo interface for individual servo control
servo = HiwonderServo(port="/dev/ttyUSB0")

# Move servos
print("Moving servos...")
servo.move_servo(1, 500, 1000)
servo.move_servo(2, 600, 1000)

time.sleep(2)

# Clean up
servo.close()
board.close()
```

### Synchronized Multi-Servo Move

```python
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController

board = HiwonderBoardController(port="/dev/ttyUSB0")

# Move multiple servos simultaneously (perfectly synchronized)
board.multi_servo_move([
    (1, 400, 1500),   # Servo 1: position 400, 1500ms
    (2, 500, 1500),   # Servo 2: position 500, 1500ms
    (3, 600, 1500),   # Servo 3: position 600, 1500ms
    (4, 700, 1500),   # Servo 4: position 700, 1500ms
])

board.close()
```

## Reference

- **Hiwonder Documentation:** https://docs.hiwonder.com/projects/Bus-Servo-Controller/en/latest/index.html
- **Section 2.1:** Control Board Protocol
- **Python Module:** `mini_bdx_runtime/hiwonder_board_controller.py`
- **Test Script:** `scripts/test_hiwonder_board.py`
