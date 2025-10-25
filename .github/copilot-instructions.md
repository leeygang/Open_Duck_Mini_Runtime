## Purpose
Help AI coding agents be productive in this repo by listing the project architecture, key files, developer workflows, and safe/hardware-specific constraints.

Keep responses concise and actionable. Prefer referencing concrete files and commands from the repository.

## Big picture (one-paragraph)
This is the runtime for the "Duck Mini" robot: a Python package providing hardware interfaces (motor HWIs, IMU, camera, audio), an RL-based controller that runs ONNX policies, and a set of scripts for testing, calibration and running the walk. The control loop is in `scripts/v2_rl_walk_mujoco.py` and uses `mini_bdx_runtime/mini_bdx_runtime/*` modules.

## Most important files to reference
- `mini_bdx_runtime/mini_bdx_runtime/rustypot_position_hwi.py` — Feetech motor HWI (primary production HWI).
- `mini_bdx_runtime/mini_bdx_runtime/hiwonder_hwi.py` — Hiwonder motor HWI (alternative, different semantics: no velocity feedback).
- `mini_bdx_runtime/mini_bdx_runtime/imu.py` — IMU worker and calibration loader (`imu_calib_data.pkl`).
- `mini_bdx_runtime/mini_bdx_runtime/duck_config.py` — central config loader; expects `~/duck_config.json` (derived from `example_config.json`).
- `scripts/v2_rl_walk_mujoco.py` — main RL control loop (50Hz default) and ONNX inference (`mini_bdx_runtime/mini_bdx_runtime/onnx_infer.py`).
- `scripts/find_soft_offsets.py`, `scripts/check_motors.py`, `scripts/turn_on.py`, `scripts/turn_off.py` — calibration and safety scripts.

## Key patterns & conventions (repo-specific)
- Configuration file lives in the user home: `~/duck_config.json`. Copy `example_config.json` to start.
- Per-joint offsets are stored in `joints_offsets` inside that config and are applied automatically by HWIs.
- Expression features (eyes, projector, antennas, sounds) are gated by flags in `duck_config.json` — always preserve gating when adding features.
- Feetech vs Hiwonder: don't assume both provide the same telemetry. Hiwonder returns zero velocities; tests and policies must tolerate missing velocity feedback.
- Control loop timing is critical: target 50Hz. Optimize ONNX inference and serial calls to keep loop <20ms.

## Hardware & safety notes (must be obeyed)
- Feetech serial: default `/dev/ttyACM0` at 1_000_000 baud. Hiwonder: `/dev/ttyUSB0` at 115200.
- Set FTDI latency to 1ms for low-latency motor control (see README udev rules section).
- Use `scripts/turn_on.py` and `scripts/turn_off.py` when powering motors. Use `--start_paused` or set `start_paused` in config during development.

## Common developer commands (concrete examples)
- Install editable package (standard):
  - `pip install -e .`
  - or with uv: `uv pip install -e .` (see `CLAUDE.md` for uv workflow)
- Run main walk (requires ONNX checkpoint):
  - `python3 scripts/v2_rl_walk_mujoco.py --onnx_model_path <path/to/model.onnx>`
- Test IMU: `python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py`
- Calibrate offsets: `python3 scripts/find_soft_offsets.py`
- Check motors: `python3 scripts/check_motors.py`

## When editing control/HWI code — checklist
1. Confirm `duck_config.json` and relevant offsets are correct (or run `find_soft_offsets.py`).
2. Run `scripts/check_motors.py` and `scripts/turn_on.py` before live testing.
3. Measure control loop latency after changes (keep below 20ms). If inference or serial I/O is slow, consider batching or reducing polling frequency.
4. Preserve feature gating flags (eyes/projector/antennas) in code branches.

## Merge guidance (if updating this file)
- If `.github/copilot-instructions.md` exists, preserve any repository-specific safety rules and the "Hardware & safety notes" section. Add new commands or files under "Most important files" and update the developer commands. Keep the whole document ~20–50 lines.

## Ask for clarification
If anything hardware-specific is unclear (serial port selection, motor ID map, or expected control timings), ask a single focused question and reference the file you inspected (e.g., `rustypot_position_hwi.py` or `v2_rl_walk_mujoco.py`).

---
Please review and tell me any gaps (missing files or commands you'd like included) and I will iterate.
