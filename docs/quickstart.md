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

# Move all rotators
elliptec-controller move-all -pos 0 45 90

# Get device information
elliptec-controller info
```

### Python API - Single Rotator

```python
import serial
from elliptec_controller import ElliptecRotator

# Open serial connection
ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# Create rotator instance
rotator = ElliptecRotator(ser, motor_address=3, name="HWP1")

try:
    # Get device info
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
    # Always close the serial connection
    ser.close()
```

### Python API - Triple Rotator Setup

```python
from elliptec_controller import TripleRotatorController

# Create controller
controller = TripleRotatorController(
    port="/dev/ttyUSB0",
    addresses=[3, 6, 8],
    names=["HWP1", "QWP", "HWP2"]
)

try:
    # Home all rotators
    controller.home_all(wait=True)

    # Set velocities
    controller.set_all_velocities(40)

    # Move to specific positions
    controller.move_all_absolute([30, 45, 60], wait=True)

    # Move relative
    controller.move_all_relative(
        [10, 15, 20],  # amounts
        ["cw", "cw", "ccw"],  # directions
        wait=True
    )

finally:
    # Always close the controller
    controller.close()
```

## Common Operations

### Error Handling

```python
from elliptec_controller import TripleRotatorController

try:
    controller = TripleRotatorController(
        port="/dev/ttyUSB0",
        addresses=[3, 6, 8]
    )
except Exception as e:
    print(f"Failed to initialize controller: {e}")
    exit(1)

try:
    # Check if rotators are ready
    if not controller.is_all_ready():
        print("Not all rotators ready. Homing...")
        controller.home_all(wait=True)

    # Perform operations
    controller.move_all_absolute([0, 0, 0], wait=True)

except Exception as e:
    print(f"Error during operation: {e}")
    # Attempt to stop all rotators in case of error
    try:
        controller.stop_all()
    except:
        pass

finally:
    controller.close()
```

### Working with Individual Rotators

```python
controller = TripleRotatorController(...)

# Access individual rotators
hwp1 = controller.rotators[0]
qwp = controller.rotators[1]
hwp2 = controller.rotators[2]

# Individual control
hwp1.move_absolute(45)
qwp.set_velocity(30)
hwp2.home()

# Get individual status
for rotator in controller.rotators:
    print(f"{rotator.name} status: {rotator.get_status()}")
```

## Best Practices

1. **Always use context management or try/finally blocks**
   ```python
   with TripleRotatorController(...) as ctrl:
       ctrl.home_all(wait=True)
   ```

2. **Check device status before operations**
   ```python
   if controller.is_all_ready():
       controller.move_all_absolute([0, 45, 90])
   else:
       print("Devices not ready")
   ```

3. **Use wait=True for sequential operations**
   ```python
   # This ensures the home completes before moving
   controller.home_all(wait=True)
   controller.move_all_absolute([45, 45, 45], wait=True)
   ```

4. **Handle timeouts appropriately**
   ```python
   import serial
   try:
       controller = TripleRotatorController(
           port="/dev/ttyUSB0",
           addresses=[3, 6, 8],
           timeout=1.0  # 1 second timeout
       )
   except serial.SerialTimeoutException:
       print("Connection timed out")
   ```

## Next Steps

- Check the [API Documentation](api.md) for detailed information
- Look at the [examples](../examples/) directory for more complex usage
- Read about [error handling](error_handling.md)
- Learn about [hardware specifics](hardware.md)