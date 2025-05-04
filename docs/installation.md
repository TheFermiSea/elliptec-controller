# Installation Guide

## Requirements

- Python 3.6 or later
- pyserial 3.5 or later
- A compatible Thorlabs Elliptec rotator device

## Basic Installation

Install the package using pip:

```bash
pip install elliptec-controller
```

## Development Installation

For development or to get the latest version from GitHub:

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install -e .
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

## Verification

Test your installation:

```python
from elliptec_controller import TripleRotatorController

# Create a controller (adjust port and addresses as needed)
controller = TripleRotatorController(
    port="/dev/ttyUSB0",
    addresses=[3, 6, 8],
    names=["HWP1", "QWP", "HWP2"]
)

# Test communication
for rotator in controller.rotators:
    info = rotator.get_device_info()
    print(f"{rotator.name} info: {info}")

# Close when done
controller.close()
```

## Next Steps

- Read the [Quickstart Guide](quickstart.md)
- Check the [API Documentation](api.md)
- Try the [Example Scripts](../examples/)