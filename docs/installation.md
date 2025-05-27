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
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys
import serial # For SerialException

# Configure Loguru for this example.
# In a larger application, you might configure this globally.
logger.remove() # Remove default handler to avoid duplication if already configured
logger.add(sys.stderr, level="INFO") # Set to "DEBUG" or "TRACE" for more details

# --- Configuration ---
# IMPORTANT: Replace with your actual serial port and rotator address
SERIAL_PORT = "/dev/ttyUSB0" 
MOTOR_ADDRESS = 1 # Example address (0-F)
# --- End Configuration ---

rotator = None
try:
    logger.info(f"Attempting to verify connection to rotator at {SERIAL_PORT}, address {MOTOR_ADDRESS}...")
    rotator = ElliptecRotator(
        port=SERIAL_PORT, 
        motor_address=MOTOR_ADDRESS, 
        name=f"Rotator-{MOTOR_ADDRESS}"
        # auto_home=True is the default, so get_device_info() and home() are attempted during init.
    )
    
    # Device info is retrieved during initialization by default.
    # Let's log the stored information.
    if rotator.device_info and rotator.device_info.get("type") not in ["Unknown", "Error"]:
        logger.info(f"Successfully connected to {rotator.name}.")
        logger.info(f"  Device Info: {rotator.device_info}")
        logger.info(f"  Pulses per Revolution: {rotator.pulse_per_revolution}")
        
        # Optionally, perform a simple action like getting status
        status = rotator.get_status()
        logger.info(f"  Current status code: {status} (00 means OK/Ready)")
        
        # Example: Get current position (will be 0.0 if homing was successful)
        position = rotator.update_position()
        if position is not None:
            logger.info(f"  Current position: {position:.2f} degrees")
        else:
            logger.warning("  Could not retrieve current position after initialization.")
            
    else:
        logger.error(f"Failed to get valid device info for {rotator.name} during initialization.")
        logger.error(f"  Stored device_info: {rotator.device_info}")
        logger.error("Please check connection, power, and address. Try TRACE level logging for more details.")

except serial.SerialException as e:
    logger.error(f"Serial port error during verification: {e}")
    logger.error(f"Is the port '{SERIAL_PORT}' correct and the device connected & powered?")
except Exception as e:
    logger.error(f"Verification failed with an unexpected error: {e}", exc_info=True)

finally:
    # The ElliptecRotator class's destructor (__del__) will attempt to close 
    # the serial port if it was opened by the class (i.e., if a port name string was passed).
    # If you had passed an already open serial.Serial object to the constructor,
    # you would be responsible for closing it here.
    if rotator:
        logger.info(f"Verification script for {rotator.name} finished.")
    else:
        logger.info("Verification script finished (rotator not initialized).")

```

## Next Steps

- Read the [Quickstart Guide](quickstart.md)
- Check the [API Documentation](api.md)
- Try the [Example Scripts](../examples/)