# Quickstart Guide

This guide will help you get started with the elliptec-controller package.

## Basic Usage

### Command Line Interface

The package provides a command-line tool for basic operations:

```bash
# Get device status
elliptec-controller status

# Home all rotators
elliptec-controller home

# Move a specific rotator
elliptec-controller move-abs -r 0 -pos 45.0

# Get device information
elliptec-controller info
```

### Python API Example

```python
import serial
from elliptec_controller import ElliptecRotator

# Open serial connection
ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# Create rotator instance (replace port and address)
rotator = ElliptecRotator("/dev/ttyUSB0", motor_address=1, name="MyRotator")

try:
    # Get device info (called automatically if port name is given)
    # info = rotator.get_device_info() # You can call it manually if needed
    # print(f"Device info: {rotator.device_info}")

    # Home the device
    info = rotator.get_device_info()
    print(f"Device info: {info}")

    # Home the device
    rotator.home(wait=True)

    # Move to absolute position
    rotator.move_absolute(45.0, wait=True)

    # Get current status
    status = rotator.get_status()
    print(f"Status: {status}")

finally:
    # Always close the serial connection if you opened it manually
    # If ElliptecRotator was initialized with a port name string,
    # it handles closing automatically.
    # ser.close()
```

## Common Operations

### Error Handling

```python
import serial
from elliptec_controller import ElliptecRotator

rotator = None
try:
    rotator = ElliptecRotator(
        port="/dev/ttyUSB0", # Replace with your port
        motor_address=1,    # Replace with your address
        debug=True
    )
except serial.SerialException as e:
    print(f"Failed to connect to rotator: {e}")
    exit(1)
except Exception as e:
    print(f"Failed to initialize rotator: {e}")
    exit(1)

try:
    # Check if rotator is ready before operation
    if not rotator.is_ready():
        print("Rotator not ready. Homing...")
        if not rotator.home(wait=True):
             print("Homing failed.")
             # Decide how to proceed if homing fails
             exit(1)

    # Perform operations
    print("Moving to 0 degrees...")
    if rotator.move_absolute(0.0, wait=True):
        print("Move successful.")
    else:
        print("Move failed.")

except Exception as e:
    print(f"Error during operation: {e}")
    # Attempt to stop the rotator in case of error
    try:
        print("Attempting to stop rotator...")
        rotator.stop()
    except Exception as stop_e:
        print(f"Failed to stop rotator: {stop_e}")

finally:
    # ElliptecRotator closes the port automatically if it opened it.
    print("Operation finished.")

```

## Best Practices

1. **Use try/except/finally for Robustness:** Always wrap communication and control logic in `try...except...finally` blocks to handle potential serial errors, timeouts, or device issues, ensuring resources like serial ports are managed correctly (though the class handles port closing if initialized with a port name).

2. **Check Device Status:** Before critical operations, check if the device is ready using `rotator.is_ready()`. Consider homing (`rotator.home()`) if the device state is unknown or not ready.

3. **Use `wait=True` for Sequential Operations:** When one movement must complete before the next begins, use the `wait=True` argument in methods like `move_absolute()`, `move_relative()`, and `home()`.

4. **Understand Timeouts:** The `send_command` method has internal timeouts. You can override them for specific commands if needed (e.g., `rotator.get_status(timeout_override=0.5)`). Be aware that long operations like homing or optimization might take time; the `wait=True` flag handles waiting based on status checks, not just fixed timeouts.

## Next Steps

- Check the [API Documentation](api.md) for detailed information
- Look at the [examples](../examples/) directory for more complex usage
- Read about [error handling](error_handling.md)
- Learn about [hardware specifics](hardware.md)