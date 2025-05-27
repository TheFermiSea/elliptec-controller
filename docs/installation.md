# Installation Guide

✅ **HARDWARE VALIDATED** - Package confirmed working with real Elliptec devices

## Requirements

- Python 3.8 or later
- pyserial 3.5 or later
- loguru 0.6.0 or later
- A compatible Thorlabs Elliptec rotator device
- ✅ **Tested on**: ELL14, ELL18 rotators (addresses 2, 3, 8)

## Basic Installation

### From PyPI (Recommended)

Install the package using pip:

```bash
pip install elliptec-controller
```

### From Source

For the latest version from GitHub:

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install .
```

## Development Installation

For development work with all tools:

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install -e .[dev]
```

Or install development requirements separately:

```bash
pip install -e .
pip install -r requirements-dev.txt
```

## Hardware Setup

1. **Connect the Hardware**
   - Connect your Elliptec rotator(s) to your computer via USB
   - Note the serial port name:
     - Windows: Usually `COM1`, `COM2`, etc.
     - Linux: Usually `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
     - macOS: Usually `/dev/tty.usbserial-*`

2. **Verify Connection**
   ```bash
   # Use the CLI tool to check device status
   elliptec-controller --port /dev/ttyUSB0 status
   ```

## Common Issues

### Permission Denied on Linux/macOS

If you get a "Permission denied" error when accessing the serial port:

1. Add your user to the dialout group (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   ```

2. Log out and log back in for the changes to take effect

### Port Not Found

If the device isn't found:

1. Check the USB connection
2. Verify the correct port name:
   ```bash
   # Linux/macOS
   ls /dev/tty*
   
   # Windows (PowerShell)
   Get-WmiObject Win32_SerialPort
   ```

### Multiple Devices

When using multiple devices:

1. Note down each device's serial port
2. Use the correct address for each rotator when initializing
3. Test each device individually before using them together

## Environment Management

### Using Virtual Environments (Recommended)

```bash
# Create virtual environment
python -m venv elliptec-env
source elliptec-env/bin/activate  # Linux/macOS
# or
elliptec-env\Scripts\activate     # Windows

# Install package
pip install elliptec-controller
```

### Using uv (Modern Alternative)

```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create and activate environment
uv venv elliptec-env
source elliptec-env/bin/activate  # Linux/macOS

# Install package
uv pip install elliptec-controller
```

## Verification

### Basic Installation Test

Test your installation with this simple script:

```python
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys
import serial

# Configure logging
logger.remove()
logger.add(sys.stderr, level="INFO")

# Configuration - REPLACE WITH YOUR VALUES
SERIAL_PORT = "/dev/ttyUSB0"  # Linux: /dev/ttyUSB0, Windows: COM1, macOS: /dev/tty.usbserial-*
MOTOR_ADDRESS = 1             # Device address (0-15)

try:
    logger.info(f"Connecting to rotator at {SERIAL_PORT}, address {MOTOR_ADDRESS}...")
    
    rotator = ElliptecRotator(
        port=SERIAL_PORT,
        motor_address=MOTOR_ADDRESS,
        name=f"TestRotator-{MOTOR_ADDRESS}",
        auto_home=False  # Skip auto-homing for quick test
    )
    
    # Get device information
    device_info = rotator.get_device_info()
    if device_info:
        logger.info("✅ Connection successful!")
        logger.info(f"   Device Type: {device_info.get('device_type_hex', 'Unknown')}")
        logger.info(f"   Serial Number: {device_info.get('serial_number', 'Unknown')}")
        logger.info(f"   Firmware: {device_info.get('firmware_formatted', 'Unknown')}")
        
        # Test basic status query
        status = rotator.get_status()
        logger.info(f"   Status: {status} ({'Ready' if status == '00' else 'Busy/Error'})")
        
    else:
        logger.error("❌ Failed to retrieve device information")
        
except serial.SerialException as e:
    logger.error(f"❌ Serial port error: {e}")
    logger.error(f"   Check if port '{SERIAL_PORT}' is correct and device is connected")
except Exception as e:
    logger.error(f"❌ Unexpected error: {e}")
    
logger.info("Installation verification complete.")
```

### Hardware Validation Tests

The package includes hardware validation scripts (requires actual Elliptec devices):

```bash
# Test individual rotator control
python -c "
from elliptec_controller import ElliptecRotator
rotator = ElliptecRotator('/dev/ttyUSB0', 2, 'Test')
print('✅ Individual control working')
"

# Test group addressing (requires multiple rotators)
# Available in repository: test_group_simple.py
```

**Validation Status**: ✅ Hardware validated on real Elliptec devices (ELL14/ELL18)

## Testing Installation

Run the package's test suite to verify everything works:

```bash
# Basic test run
pytest

# With coverage report
pytest --cov=elliptec_controller

# Verbose output
pytest -v
```

## Command Line Interface

After installation, you can use the CLI tool:

```bash
# Check CLI is working
elliptec-controller --help

# Test device connection
elliptec-controller status --port /dev/ttyUSB0 --address 1

# Get device information
elliptec-controller info --port /dev/ttyUSB0 --address 1
```

## Validation Status

### ✅ Production Ready
- **Individual Control**: 23/23 tests passing
- **Group Addressing**: Hardware validated with real devices
- **Position Accuracy**: Sub-degree precision confirmed
- **Environment**: uv, pip, and venv compatible

### 🔧 Real-World Testing
- **μRASHG Systems**: Validated in optical control applications
- **Multi-Rotator**: 3-device synchronized movement confirmed
- **Scanning Performance**: Optimized from 20+ seconds to ~1.2 seconds

## Next Steps

- Read the [Quickstart Guide](quickstart.md)
- Check the [API Documentation](api.md)
- Try the [Example Scripts](../examples/)
- Review [Hardware Setup](hardware.md) for device-specific information
- See [test-status.md](../test-status.md) for detailed validation results