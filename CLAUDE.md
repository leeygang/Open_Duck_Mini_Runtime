# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Open Duck Mini Runtime is a Python-based control system for a bipedal robot ("Duck Mini") that runs on Raspberry Pi. The system uses reinforcement learning (RL) policies via ONNX models to enable walking, combined with hardware interfaces for motors (Feetech servos via rustypot, or Hiwonder servos via serial), IMU, Xbox controller input, and optional expression features (eyes, projector, antennas, speaker).

**Target platform**: Raspberry Pi Zero 2W or Raspberry Pi 5 (64-bit Raspberry Pi OS Lite)

**Supported servo types**: Feetech STS3215 (primary), Hiwonder HTD-45H (alternative), standard PWM servos (antennas)

## Installation

This is a Python package installed in editable mode.

### Using pip (traditional method):

```bash
# On the Raspberry Pi (with virtual environment activated)
pip install -e .
```

### Using uv (modern, faster method):

```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install the package in editable mode
uv pip install -e .

# Or create a virtual environment and install
uv venv
source .venv/bin/activate  # On Linux/macOS
uv pip install -e .
```

For Raspberry Pi 5 specifically:
```bash
pip uninstall -y RPi.GPIO
pip install lgpio
# Or with uv:
uv pip install lgpio
```

## Configuration

The robot requires a `duck_config.json` file in the home directory (`~/duck_config.json`). Copy and modify `example_config.json` as a starting point. This file contains:

- **joints_offsets**: Per-joint calibration offsets (in radians) - critical for proper motor positioning
- **expression_features**: Flags for optional hardware (eyes, projector, antennas, speaker, microphone, camera)
- **imu_upside_down**: Boolean for IMU mounting orientation
- **start_paused**: Whether to start the walk policy in paused mode
- **phase_frequency_factor_offset**: Walking gait frequency adjustment

## Common Commands

### Testing Hardware Components

```bash
# Test IMU (verify readings and frame orientation)
python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py

# Test IMU over network (server on robot, client on dev machine)
python3 scripts/imu_server.py  # on robot
python3 scripts/imu_client.py --ip <robot_ip>  # on dev machine

# Test motors (verify all motors connected and configured)
python3 scripts/check_motors.py

# Test Xbox controller
python3 mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py

# Test camera
python3 scripts/cam_test.py

# Test antennas
python3 scripts/antennas_controller_test.py
```

### Motor Configuration

**Feetech servos (default):**
```bash
# Configure all Feetech motors at once
python3 scripts/configure_all_motors.py

# Configure individual Feetech motor
python3 scripts/configure_motor.py --id <desired_id>

# Find joint offsets for Feetech (calibration procedure)
python3 scripts/find_soft_offsets.py
```

**Hiwonder servos (alternative):**
```bash
# Configure Hiwonder servo ID
python3 scripts/configure_hiwonder_motor.py --id <desired_id>

# Check all Hiwonder servos
python3 scripts/check_hiwonder_motors.py

# Find joint offsets for Hiwonder
python3 scripts/find_hiwonder_offsets.py

# See HIWONDER_INTEGRATION.md for complete guide
```

### Running the Walking Policy

```bash
# Main walk script (requires ONNX policy checkpoint)
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <path_to_onnx_file>

# With custom config path
python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <path> --duck_config_path <path>

# Other useful flags:
# --action_scale: Scale RL policy actions (default 0.25)
# -p, -i, -d: PID gains (default p=30, i=0, d=0)
# --control_freq: Control loop frequency in Hz (default 50)
# --pitch_bias: IMU pitch bias in degrees (default 0)
# --cutoff_frequency: Low-pass filter cutoff for actions
```

### IMU Calibration

```bash
python3 scripts/calibrate_imu.py
```

This generates `imu_calib_data.pkl` which the IMU class loads automatically if present.

### Power Management

```bash
# Turn on motors with safe initialization
python3 scripts/turn_on.py

# Turn off motors (disable torque)
python3 scripts/turn_off.py
```

### Data Recording and Analysis

```bash
# Record sensor data during operation
python3 scripts/record_data.py
python3 scripts/new_record_data.py

# Plot recorded data
python3 scripts/plot_recorded_data.py
```

## Architecture

### Hardware Interface Layer

The system supports multiple servo types with modular hardware interfaces:

**`mini_bdx_runtime/rustypot_position_hwi.py`** - `HWI` class (Feetech servos - primary)
- Communicates with Feetech STS3215 servo motors via the `rustypot` library over serial (`/dev/ttyACM0`)
- Maps 14 DOF joint names to motor IDs (e.g., `left_hip_yaw: 20`, `right_hip_yaw: 10`)
- Applies per-joint offsets from `duck_config.json`
- Controls PID gains (kp, kd) per motor
- Provides position read/write at ~50Hz control frequency
- `init_pos` defines the standing pose, `zero_pos` is the mechanical zero

**`mini_bdx_runtime/hiwonder_hwi.py`** - `HiwonderHWI` class (Hiwonder servos - alternative)
- Interfaces with Hiwonder HTD-45H serial bus servos via PySerial
- Uses LewanSoul/LX protocol (half-duplex UART at 115200 baud)
- Position range: 0-1000 units (240° total), automatically converts to/from radians
- Provides position feedback but NOT velocity feedback (returns zeros)
- Typically connected via USB-to-TTL adapter (`/dev/ttyUSB0`) or GPIO UART
- Supports hardware-level offset calibration (stored in servo EEPROM)
- Can be used standalone or mixed with Feetech servos (requires hybrid interface)
- See `HIWONDER_INTEGRATION.md` for complete integration guide

**`mini_bdx_runtime/antennas.py`** - Standard PWM servos
- Controls hobby servos via GPIO PWM (50Hz, 1-2ms pulse width)
- No position feedback (open-loop control)
- Used for expression features like antennas

### Control Loop Architecture

**`scripts/v2_rl_walk_mujoco.py`** - `RLWalk` class

Main control loop structure (runs at 50Hz by default):

1. **Observation gathering**:
   - IMU data (gyro, accelerometer via quaternion orientation)
   - Joint positions and velocities (14 DOF)
   - Last 3 actions (action history)
   - Motor targets
   - Feet contact sensors
   - Imitation phase (sinusoidal gait phase encoding)
   - Command inputs (linear x/y velocity, angular velocity, head control)

2. **Policy inference**:
   - ONNX model processes observation → action (14D)
   - Actions are scaled and added to `init_pos` to get motor targets
   - Optional low-pass filtering of actions

3. **Motor command**:
   - `HWI.set_position_all()` sends targets to all motors

4. **Expression features** (optional):
   - Eyes, projector, sounds, antennas controlled via Xbox buttons

### IMU Handling

**`mini_bdx_runtime/imu.py`** - `Imu` class
- Interfaces with Adafruit BNO055 IMU via I2C
- Runs in `IMUPLUS_MODE` (accel + gyro, no magnetometer)
- Threaded worker reads quaternions at specified sampling frequency
- Supports axis remapping for upside-down mounting
- Applies pitch bias correction
- Loads calibration offsets from `imu_calib_data.pkl` if present

### Input Control

**`mini_bdx_runtime/xbox_controller.py`** - `XBoxController` class
- Uses pygame to read Xbox One controller over Bluetooth
- Left stick: linear x/y velocity commands
- Right stick: angular velocity (yaw) OR head control (when Y button toggled)
- LB button: sprint mode (increases phase frequency)
- A: pause/unpause
- B: play random sound
- X: toggle projector
- Triggers: control antennas
- D-pad up/down: adjust phase frequency offset
- Runs in separate thread at configurable command frequency (default 20Hz)

### Reference Motion

**`mini_bdx_runtime/poly_reference_motion.py`** - `PolyReferenceMotion`
- Loads polynomial coefficients for periodic reference motion
- Used to compute imitation phase (`imitation_phase`) for the RL policy
- Phase advances each step based on `phase_frequency_factor` (modulated by velocity and sprint mode)

### Optional Expression Features

- **Eyes** (`eyes.py`): LED matrix control for expressions
- **Projector** (`projector.py`): GPIO-controlled projector
- **Antennas** (`antennas.py`): Servo-controlled antennas (separate from main 14 DOF)
- **Sounds** (`sounds.py`): Audio playback from `mini_bdx_runtime/assets/`
- **Feet Contacts** (`feet_contacts.py`): Binary contact sensors for feet
- **Camera** (`camera.py`): PiCamera interface

All expression features are enabled/disabled via `duck_config.json` flags.

## Key Concepts

### Joint Offsets
Due to assembly variations, each motor's "zero" position differs slightly. The `joints_offsets` in `duck_config.json` compensates for this. Use `scripts/find_soft_offsets.py` to calibrate these values. All motor commands in `HWI` automatically apply these offsets.

### PID Control
Motors use position control with PID gains (kp, kd). Default kp=30 for legs, kp=8 for head. The HWI sets low torque (kp=2) during `turn_on()` initialization to safely move to `init_pos`, then switches to high torque.

### Control Frequency
The main control loop (`v2_rl_walk_mujoco.py`) runs at 50Hz by default. This is a hard real-time requirement - if policy inference + motor communication exceeds the budget (20ms), a warning is printed. The loop uses `time.sleep()` to maintain precise timing.

### Imitation Phase Encoding
The RL policy was trained with a reference periodic motion. The `imitation_phase` (2D sine/cosine encoding) is part of the observation space, advancing cyclically to provide gait timing information to the policy.

### Configuration System
`DuckConfig` class centralizes all robot configuration. It loads from `~/duck_config.json` by default, and falls back to defaults with a warning/confirmation prompt if not found. This prevents running an uncalibrated robot.

## File Structure

- `mini_bdx_runtime/mini_bdx_runtime/` - Core library modules
  - `rustypot_position_hwi.py` - Feetech motor hardware interface
  - `hiwonder_hwi.py` - Hiwonder motor hardware interface (alternative)
  - `imu.py` - IMU sensor interface
  - `xbox_controller.py` - Controller input
  - `duck_config.py` - Configuration management
  - `onnx_infer.py` - ONNX policy inference
  - `rl_utils.py` - RL-specific utilities (action dict creation, filtering)
  - Expression feature modules (`eyes.py`, `projector.py`, `antennas.py`, `sounds.py`, `camera.py`, `buttons.py`, `feet_contacts.py`)
  - `poly_reference_motion.py` - Reference motion for imitation phase
  - `raw_imu.py` - Low-level IMU testing

- `mini_bdx_runtime/assets/` - Audio files for sound playback

- `scripts/` - Standalone scripts for testing, configuration, and main execution
  - `v2_rl_walk_mujoco.py` - Main walking script
  - Feetech servo scripts:
    - `find_soft_offsets.py` - Joint offset calibration wizard
    - `check_motors.py` - Motor connectivity test
    - `configure_motor.py`, `configure_all_motors.py` - Motor ID configuration
  - Hiwonder servo scripts:
    - `configure_hiwonder_motor.py` - Configure Hiwonder servo ID
    - `check_hiwonder_motors.py` - Check Hiwonder connectivity
    - `find_hiwonder_offsets.py` - Calibrate Hiwonder offsets
  - `imu_server.py`, `imu_client.py` - Network IMU testing
  - `calibrate_imu.py` - IMU calibration procedure
  - `turn_on.py`, `turn_off.py` - Power management
  - Testing scripts (`cam_test.py`, `fc_test.py`, `head_puppet.py`, `antennas_controller_test.py`)
  - Data recording and plotting utilities

## Important Notes

- **Serial ports**:
  - Feetech motors: `/dev/ttyACM0` at 1,000,000 baud (motor control board)
  - Hiwonder motors: `/dev/ttyUSB0` at 115200 baud (USB-to-TTL adapter)
  - Can use both simultaneously on different ports
- **USB latency**: Set FTDI latency timer to 1ms for low-latency motor communication (see README).
- **I2C**: IMU requires I2C enabled via `raspi-config`.
- **GPIO compatibility**: Raspberry Pi 5 requires `lgpio` instead of `RPi.GPIO`.
- **Motor IDs**:
  - Feetech: Left leg 20-24, right leg 10-14, head 30-33
  - Hiwonder: Recommend 100-199 range to avoid conflicts
  - Antennas: PWM servos (no IDs, pin-based)
- **Coordinate frames**: The IMU frame is remapped in software depending on mounting orientation (`imu_upside_down` config).
- **Policy checkpoints**: ONNX models are trained externally and loaded at runtime. The policy expects a specific observation space (see `get_obs()` in `RLWalk`).
- **Thread safety**: IMU and Xbox controller run in separate threads with queues for cross-thread communication.
- **Velocity feedback**: Hiwonder servos do NOT provide velocity feedback. If using Hiwonder for main locomotion, RL policy observations must account for this (zeros returned).

## Development Workflow

1. When modifying motor control logic, test with `scripts/check_motors.py` and `turn_on.py` before running the full walk.
2. Always verify `duck_config.json` is present and correct before running on hardware.
3. Use `--start_paused` or set `"start_paused": true` in config for safe testing - motors will be powered but policy won't execute until A button is pressed.
4. Motor offsets drift over time; re-calibrate with `find_soft_offsets.py` if the robot's zero pose looks off.
5. When developing new features, gate them behind `duck_config.json` flags to avoid breaking minimal setups.
