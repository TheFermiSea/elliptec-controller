# Quickstart Guide

This guide will help you get started with the elliptec-controller package.

## Basic Usage

### Command Line Interface

The package provides a command-line tool for basic operations:

```bash
# Get device status (add --log-level TRACE for detailed logs)
elliptec-controller status --log-level INFO

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
from loguru import logger
import sys

# Configure Loguru for detailed output
logger.remove()
logger.add(sys.stderr, level="DEBUG") # Or "TRACE" for even more detail

# Open serial connection (ElliptecRotator can also open it by name)
# ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# Create rotator instance (replace port and address)
rotator = ElliptecRotator(port="/dev/ttyUSB0", motor_address=1, name="MyRotator")

try:
    # Device info is retrieved during initialization if auto_home=True (default)
    logger.info(f"Device info: {rotator.device_info}")

    # Home the device
    rotator.home(wait=True)
    logger.info("Homing complete.")

    # Move to absolute position
    rotator.move_absolute(45.0, wait=True)
    logger.info("Move to 45.0 degrees complete.")

    # Get current status
    status = rotator.get_status()
    logger.info(f"Status: {status}")

finally:
    # If ElliptecRotator was initialized with a port name string,
    # it handles closing automatically.
    # If you passed an open serial object, you would close it here:
    # if 'ser' in locals() and ser.is_open:
    #     ser.close()
    logger.info("Quickstart example finished.")
```

## Common Operations

### Error Handling

```python
import serial
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys

# Configure Loguru
logger.remove()
logger.add(sys.stderr, level="INFO") # Adjust level as needed

rotator = None
try:
    rotator = ElliptecRotator(
        port="/dev/ttyUSB0", # Replace with your port
        motor_address=1    # Replace with your address
    )
except serial.SerialException as e:
    logger.error(f"Failed to connect to rotator: {e}")
    exit(1)
except Exception as e:
    logger.error(f"Failed to initialize rotator: {e}")
    exit(1)

try:
    # Check if rotator is ready before operation
    if not rotator.is_ready():
        logger.warning("Rotator not ready. Homing...")
        if not rotator.home(wait=True):
             logger.error("Homing failed.")
             # Decide how to proceed if homing fails
             exit(1)

    # Perform operations
    logger.info("Moving to 0 degrees...")
    if rotator.move_absolute(0.0, wait=True):
        logger.info("Move successful.")
    else:
        logger.error("Move failed.")

except Exception as e:
    logger.error(f"Error during operation: {e}", exc_info=True)
    # Attempt to stop the rotator in case of error
    try:
        logger.warning("Attempting to stop rotator due to error...")
        rotator.stop()
    except Exception as stop_e:
        logger.error(f"Failed to stop rotator: {stop_e}")

finally:
    # ElliptecRotator closes the port automatically if it opened it.
    logger.info("Operation finished.")

```

## Logging with Loguru

The `elliptec-controller` package uses the [Loguru](https://loguru.readthedocs.io/en/stable/) library for logging. This provides more flexible and powerful logging capabilities compared to simple `print` statements.

- The `ElliptecRotator` class no longer has a `debug` parameter.
- To see detailed logs (e.g., DEBUG or TRACE level) from the controller, you need to configure Loguru in your application.
- A basic configuration to print all logs to the console:
  ```python
  from loguru import logger
  import sys
  logger.remove() # Removes the default handler
  logger.add(sys.stderr, level="TRACE") # Add a new handler to stderr, showing all messages
  ```
- You can set the `level` to "DEBUG", "INFO", "WARNING", "ERROR", or "CRITICAL" as needed. "TRACE" is the most verbose.
- Consult the Loguru documentation for advanced features like writing to files, log rotation, custom formatting, etc.

## Best Practices

1. **Use try/except/finally for Robustness:** Always wrap communication and control logic in `try...except...finally` blocks to handle potential serial errors, timeouts, or device issues. The `ElliptecRotator` class handles closing the serial port automatically if it opened it (i.e., if you passed a port name string to the constructor).

2. **Check Device Status:** Before critical operations, check if the device is ready using `rotator.is_ready()`. Consider homing (`rotator.home()`) if the device state is unknown or not ready. Loguru messages will provide context on these checks.

3. **Use `wait=True` for Sequential Operations:** When one movement must complete before the next begins, use the `wait=True` argument in methods like `move_absolute()`, `move_relative()`, and `home()`. The internal logging will indicate when these blocking operations start and finish.

4. **Understand Timeouts:** The `send_command` method has internal timeouts. You can override them for specific commands if needed. Log messages (especially at DEBUG or TRACE level) can help understand timeout occurrences. Be aware that long operations like homing or optimization might take time; the `wait=True` flag handles waiting based on status checks, not just fixed timeouts.

## Next Steps

- Check the [API Documentation](api.md) for detailed information.
- Look at the [examples](../examples/) directory for more complex usage scenarios.
- Learn about [hardware specifics](hardware.md).