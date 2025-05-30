# Quickstart Guide

This guide will help you get started with the elliptec-controller package.

## Basic Usage

The elliptec-controller supports both synchronous (blocking) and asynchronous (non-blocking) operation modes. The asynchronous mode uses a dedicated worker thread with per-command response queues for reliable operation.

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

### Synchronous API Example

```python
import serial
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys

# Configure Loguru for detailed output
logger.remove()
logger.add(sys.stderr, level="DEBUG") # Or "TRACE" for even more detail

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
    logger.info("Quickstart example finished.")
```

### Asynchronous API Example

```python
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys
import time

# Configure Loguru for detailed output
logger.remove()
logger.add(sys.stderr, level="DEBUG")

# Using context manager for automatic thread management
with ElliptecRotator(port="/dev/ttyUSB0", motor_address=1, name="MyRotator") as rotator:
    # Thread is automatically started by the context manager
    
    # Device info is retrieved during initialization
    logger.info(f"Device info: {rotator.device_info}")
    
    # Home the device (wait=True still works in async mode)
    rotator.home(wait=True)
    logger.info("Homing complete.")
    
    # Move to absolute position in non-blocking mode
    logger.info("Moving to 45.0 degrees...")
    rotator.move_absolute(45.0, wait=False)
    
    # Do other work while moving
    logger.info("Doing other work while moving...")
    time.sleep(0.5)
    
    # Wait for movement to complete when needed
    rotator.wait_until_ready()
    logger.info(f"Move complete. Position: {rotator.position_degrees:.2f}°")
    
    # Mix synchronous and asynchronous as needed
    rotator.move_absolute(90.0, use_async=False)  # Force synchronous
    rotator.move_absolute(180.0, use_async=True)  # Explicit async
    
    # Wait until ready before exiting
    rotator.wait_until_ready()
    
    # Thread is automatically stopped when exiting the context manager
```

## Synchronous vs Asynchronous

There are two ways to use the Elliptec controller:

1. **Synchronous Mode (Default)**: Commands block until completion
   - Simpler to use and understand
   - Good for sequential operations
   - Use when you don't need to do anything while the device is moving

2. **Asynchronous Mode**: Commands return immediately, operations happen in background
   - Better for responsive applications
   - Allows parallel operations
   - Required for GUI applications to prevent freezing
   - Use context manager (`with` statement) for easiest management

### Manual Thread Management

If you prefer to manually control the async thread lifecycle:

```python
rotator = ElliptecRotator("/dev/ttyUSB0", motor_address=1)

# Start async thread manually
rotator.connect()

# Commands now operate in async mode by default
rotator.move_absolute(45.0)

# Stop async thread manually when done
rotator.disconnect()
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

4. **Use Context Manager for Async Mode:** When using asynchronous mode, prefer the context manager (`with ElliptecRotator(...) as rotator:`) for automatic thread management. This ensures proper cleanup of the worker thread.

5. **Mix Sync and Async as Needed:** You can override the default mode for any command with the `use_async` parameter to mix synchronous and asynchronous operations. After calling `connect()`, async becomes the default mode.

6. **Understand Timeouts:** The `send_command` method has internal timeouts. You can override them for specific commands if needed. Log messages (especially at DEBUG or TRACE level) can help understand timeout occurrences. Be aware that long operations like homing or optimization might take time; the `wait=True` flag handles waiting based on status checks, not just fixed timeouts.

7. **Take Advantage of Per-Command Response Queues:** The async implementation uses dedicated response queues for each command, which improves reliability in busy communication scenarios. This design prevents response mixups when multiple commands are issued rapidly.

## Next Steps

- Check the [API Documentation](api.md) for detailed information.
- Look at the [examples](../examples/) directory for more complex usage scenarios.
- Learn about [hardware specifics](hardware.md).