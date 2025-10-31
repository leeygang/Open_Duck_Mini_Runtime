# Hiwonder Bus Servo Controller - Board Commands

This document explains the 4 board-level commands from the official Hiwonder Bus Servo Controller protocol and how to use them.

## Protocol Overview

The Hiwonder Bus Servo Controller uses a simple serial protocol to control multiple servos simultaneously.

### Frame Format

```
[0x55][0x55][ID][Length][Command][Param1]...[ParamN][Checksum]
```

- **Header**: `0x55 0x55` (fixed)
- **ID**: `0xFE` for board commands
- **Length**: Number of bytes from ID to Checksum (inclusive)
- **Command**: Command code (see below)
- **Params**: Variable length parameters
- **Checksum**: `~(ID + Length + Command + Param1 + ... + ParamN) & 0xFF`

### Connection

- **Baudrate**: 115200
- **Port**: `/dev/ttyUSB0` (USB-to-TTL adapter) or `/dev/serial0` (GPIO UART)
- **Board ID**: `0xFE` (fixed for board commands)

## The 4 Board Commands

### 1. CMD_SERVO_MOVE (0x03)

Move multiple servos simultaneously to specified positions.

**Command Frame:**
```
[0x55][0x55][0xFE][Length][0x03][Count][ID1][Pos1_L][Pos1_H][Time1_L][Time1_H]...[Checksum]
```

**Parameters:**
- `Count`: Number of servos to move
- For each servo:
  - `ID`: Servo ID (1-253)
  - `Pos_L`, `Pos_H`: Target position (0-1000, little-endian)
  - `Time_L`, `Time_H`: Movement time in milliseconds (little-endian)

**Python Usage:**
```python
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController

board = HiwonderBoardController(port="/dev/ttyUSB0")

# Move servos 1, 2, 3 to center position (500) in 1000ms
board.move_servos([
    (1, 500, 1000),  # Servo 1
    (2, 500, 1000),  # Servo 2
    (3, 500, 1000),  # Servo 3
])
```

### 2. CMD_GET_BATTERY_VOLTAGE (0x0F)

Read the board's battery voltage.

**Command Frame (Send):**
```
[0x55][0x55][0xFE][0x03][0x0F][Checksum]
```

**Response Frame:**
```
[0x55][0x55][0xFE][0x04][0x0F][Voltage_L][Voltage_H][Checksum]
```

**Response:**
- `Voltage_L`, `Voltage_H`: Voltage in millivolts (little-endian)

**Python Usage:**
```python
voltage = board.get_battery_voltage()
if voltage is not None:
    print(f"Battery: {voltage:.2f}V")
```

### 3. CMD_MULT_SERVO_UNLOAD (0x14)

Disable torque on multiple servos (servos become freely movable).

**Command Frame:**
```
[0x55][0x55][0xFE][Length][0x14][Count][ID1][ID2]...[IDn][Checksum]
```

**Parameters:**
- `Count`: Number of servos to unload
- `ID1`, `ID2`, ...: Servo IDs (1-253)

**Python Usage:**
```python
# Disable torque on servos 1, 2, 3
board.unload_servos([1, 2, 3])
```

**Note:** To re-enable torque, send a `move_servos` command.

### 4. CMD_MULT_SERVO_POS_READ (0x15)

Read current positions of multiple servos.

**Command Frame (Send):**
```
[0x55][0x55][0xFE][Length][0x15][Count][ID1][ID2]...[IDn][Checksum]
```

**Response Frame:**
```
[0x55][0x55][0xFE][Length][0x15][Count][ID1][Pos1_L][Pos1_H][ID2][Pos2_L][Pos2_H]...[Checksum]
```

**Response:**
- `Count`: Number of servos
- For each servo:
  - `ID`: Servo ID
  - `Pos_L`, `Pos_H`: Current position (0-1000, little-endian)

**Python Usage:**
```python
positions = board.read_servo_positions([1, 2, 3])
if positions:
    for servo_id, position in positions:
        print(f"Servo {servo_id}: position {position}")
```

## Complete Example

```python
from mini_bdx_runtime.hiwonder_board_controller import HiwonderBoardController
import time

# Initialize board controller
board = HiwonderBoardController(port="/dev/ttyUSB0", baudrate=115200)

try:
    # 1. Check battery voltage
    voltage = board.get_battery_voltage()
    print(f"Battery: {voltage:.2f}V")

    # 2. Read initial positions
    positions = board.read_servo_positions([1, 2, 3])
    print("Initial positions:")
    for servo_id, position in positions:
        print(f"  Servo {servo_id}: {position}")

    # 3. Move servos to center
    print("\nMoving servos to center (500)...")
    board.move_servos([
        (1, 500, 1000),
        (2, 500, 1000),
        (3, 500, 1000),
    ])
    time.sleep(1.5)

    # 4. Read positions after movement
    positions = board.read_servo_positions([1, 2, 3])
    print("After movement:")
    for servo_id, position in positions:
        print(f"  Servo {servo_id}: {position}")

    # 5. Unload servos (disable torque)
    print("\nUnloading servos...")
    board.unload_servos([1, 2, 3])
    print("Servos can now be moved manually")

finally:
    board.close()
```

## Testing

Test your setup with the provided test script:

```bash
# Test with USB-to-TTL adapter
python3 scripts/test_hiwonder_board.py --port /dev/ttyUSB0

# Test with GPIO UART
python3 scripts/test_hiwonder_board.py --port /dev/serial0

# Test specific servo IDs
python3 scripts/test_hiwonder_board.py --port /dev/ttyUSB0 --servo-ids 1,2,3
```

The test script will:
1. Read battery voltage
2. Read servo positions
3. Test servo movement (optional)
4. Test servo unload (optional)
5. Test coordinated movement patterns (optional)

## Position Units

Hiwonder servos use a position range of **0-1000** units:
- `0` = -120° (left limit)
- `500` = 0° (center)
- `1000` = +120° (right limit)
- Total range: 240°
- 1 unit ≈ 0.24°

## Connection Methods

### Option 1: USB-to-TTL Adapter (Recommended)

```
Raspberry Pi
    ↓ (USB cable)
USB-to-TTL Adapter (CP2102, FT232, CH340)
    ↓ (Serial wires: TX, RX, GND)
Hiwonder Bus Servo Control Board (serial pins)
    ↓ (Servo connectors)
Hiwonder Servos (daisy-chained)
    ↓ (External power: 6-8.4V)
```

**Settings:**
- Port: `/dev/ttyUSB0`
- Baudrate: `115200`
- Bluetooth: Keep enabled (no conflict)

### Option 2: GPIO UART (Temporary)

```
Raspberry Pi GPIO Pins (14 & 15)
    ↓ (Direct wiring: TX, RX, GND)
Hiwonder Bus Servo Control Board (serial pins)
    ↓ (Servo connectors)
Hiwonder Servos (daisy-chained)
    ↓ (External power: 6-8.4V)
```

**Settings:**
- Port: `/dev/serial0` or `/dev/ttyAMA0`
- Baudrate: `115200`
- Bluetooth: Must be disabled (conflicts with GPIO UART)

**Wiring:**
- Pi GPIO 14 (Pin 8) → Board RX
- Pi GPIO 15 (Pin 10) → Board TX
- Pi GND (Pin 6 or 9) → Board GND

## Troubleshooting

### "Could not read voltage"
- Check board is powered on
- Verify serial connection (TX→RX, RX→TX, GND→GND)
- Check baudrate is 115200
- Verify port is correct (`/dev/ttyUSB0` or `/dev/serial0`)

### "Could not read positions"
- Check servos are connected and powered (6-8.4V external supply)
- Verify servo IDs are correct
- Ensure servos are not in unloaded state
- Check battery voltage is sufficient (>6V)

### "Checksum error"
- Electrical noise or poor connections
- Cable too long or poor quality
- Try shielded cables or shorter cables
- Add ferrite beads

### Servos don't move
- Check external power supply (6-8.4V)
- Verify servos are loaded (not unloaded)
- Check servo IDs are correct
- Verify battery voltage is sufficient

## Command Reference Summary

| Command | Code | Description | Response |
|---------|------|-------------|----------|
| CMD_SERVO_MOVE | 0x03 | Move multiple servos | No response |
| CMD_GET_BATTERY_VOLTAGE | 0x0F | Read battery voltage | Voltage (mV) |
| CMD_MULT_SERVO_UNLOAD | 0x14 | Unload multiple servos | No response |
| CMD_MULT_SERVO_POS_READ | 0x15 | Read multiple servo positions | Positions |

## Reference

- **Protocol Document**: `Bus Servo Controller Communication Protocol.pdf`
- **Python Module**: `mini_bdx_runtime/hiwonder_board_controller.py`
- **Test Script**: `scripts/test_hiwonder_board.py`
- **Setup Guide**: `HIWONDER_SETUP_GUIDE.txt`
- **Integration Guide**: `HIWONDER_INTEGRATION.md`
- **Official Documentation**: https://docs.hiwonder.com/
