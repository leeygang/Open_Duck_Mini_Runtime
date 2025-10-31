# Using UV with Open Duck Mini Runtime

This project now supports `uv`, a fast Python package installer and resolver written in Rust.

## Why UV?

- **10-100x faster** than pip
- Better dependency resolution
- Modern `pyproject.toml` support
- Works seamlessly with existing pip workflows

## Installation

### Install UV

```bash
# On Linux/macOS
curl -LsSf https://astral.sh/uv/install.sh | sh

# On Windows
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# Or with pip
pip install uv
```

### Install the Project

```bash
cd ~/Open_Duck_Mini_Runtime

# Option 1: Install in current environment
uv pip install -e .

# Option 2: Create a new virtual environment and install
uv venv .venv
source .venv/bin/activate  # On Linux/macOS
# .venv\Scripts\activate   # On Windows
uv pip install -e .
```

## Common Commands

```bash
# Install the project in editable mode
uv pip install -e .

# Install additional dependencies
uv pip install pyserial

# Install from requirements file
uv pip install -r requirements.txt

# Sync with pyproject.toml
uv pip sync

# Update all dependencies
uv pip install -e . --upgrade

# List installed packages
uv pip list

# Uninstall a package
uv pip uninstall package-name
```

## On Raspberry Pi

```bash
# SSH into your Raspberry Pi
ssh pi@<raspberry_pi_ip>

# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Navigate to project
cd ~/Open_Duck_Mini_Runtime

# Install the project
uv pip install -e .

# For Raspberry Pi 5, install lgpio
uv pip uninstall RPi.GPIO
uv pip install lgpio
```

## Troubleshooting

### "uv: command not found"

```bash
# Add uv to PATH
export PATH="$HOME/.cargo/bin:$PATH"

# Or add to ~/.bashrc for permanent
echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### "ModuleNotFoundError: No module named 'mini_bdx_runtime'"

```bash
# Make sure you've installed the project
cd ~/Open_Duck_Mini_Runtime
uv pip install -e .

# Verify installation
python3 -c "import mini_bdx_runtime; print('Success!')"
```

### Git dependency issues

```bash
# For the pypot git dependency, uv handles it automatically
# If you have issues, you can install it separately:
uv pip install git+https://github.com/pollen-robotics/pypot@support-feetech-sts3215
```

## Migration from pip

If you're currently using pip, you can switch to uv seamlessly:

```bash
# Everything works the same, just replace 'pip' with 'uv pip'
pip install -e .        →  uv pip install -e .
pip list                →  uv pip list
pip install package     →  uv pip install package
pip uninstall package   →  uv pip uninstall package
```

## Performance Comparison

Typical installation times on Raspberry Pi:

| Tool | Time     |
|------|----------|
| pip  | ~5-10min |
| uv   | ~30-60s  |

## More Information

- UV Documentation: https://docs.astral.sh/uv/
- UV GitHub: https://github.com/astral-sh/uv
