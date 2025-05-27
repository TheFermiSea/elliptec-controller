```python
#!/usr/bin/env python3
"""
Basic Usage Example for Elliptec Controller
Demonstrates connecting to a single rotator, homing, moving, and getting status.
"""

import serial
from elliptec_controller import ElliptecRotator
import time
from loguru import logger
import sys

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"  # <-- IMPORTANT: Replace with your serial port name
MOTOR_ADDRESS = 1           # <-- IMPORTANT: Replace with your device's address (0-F)

# Configure Loguru for detailed output
logger.remove() # Removes the default handler to avoid duplicate outputs if already configured
logger.add(sys.stderr, level="DEBUG") # Change level to "TRACE" for most verbose, or "INFO" for less

# Using a specific serial port
try:
    logger.info(f"Connecting to rotator at address {MOTOR_ADDRESS} on port {SERIAL_PORT}...")
    # Create a rotator instance - this opens the port and gets device info
    rotator1 = ElliptecRotator(
        port=SERIAL_PORT,
        motor_address=MOTOR_ADDRESS,
        name=f"Rotator-{MOTOR_ADDRESS}"
    )

    # Log device info (retrieved during initialization if auto_home=True)
    logger.info("\n--- Device Information ---")
    if rotator1.device_info and rotator1.device_info.get("type") not in ["Unknown", "Error"]:
        logger.info(f"  Device Info: {rotator1.device_info}")
        logger.info(f"  Pulses per Revolution: {rotator1.pulse_per_revolution}")
    else:
        logger.warning("  Could not retrieve valid device info.")
        # Optionally raise an error or exit if info is critical
        # raise ConnectionError("Failed to get device info")

    # Home the rotator
    logger.info("\n--- Homing ---")
    logger.info("Homing...")
    if rotator1.home(wait=True):
        logger.info("Homing complete.")
    else:
        logger.error("Homing failed.")
    time.sleep(1)

    # Move to an absolute position (in degrees)
    target_pos = 90.0
    logger.info(f"\n--- Moving to {target_pos} degrees ---")
    if rotator1.move_absolute(target_pos, wait=True):
        logger.info("Move complete.")
    else:
        logger.error("Move failed.")
    time.sleep(1)

    # Get the current position
    logger.info("\n--- Checking Position ---")
    position = rotator1.update_position() # debug flag removed
    if position is not None:
        logger.info(f"Current reported position: {position:.2f} degrees")
    else:
        logger.warning("Failed to get current position.")


    # Example of getting status code
    status_code = rotator1.get_status() # debug flag removed
    logger.info(f"Current status code: {status_code} (00 means OK/Ready)")

    logger.info("\nBasic usage example finished.")

except serial.SerialException as e:
    logger.error(f"\n--- Serial Port Error ---")
    logger.error(f"Could not open or communicate on port '{SERIAL_PORT}'.")
    logger.error(f"Error details: {e}")
    logger.error("Please check if the port name is correct and the device is connected.")
except Exception as e:
    logger.error(f"\n--- An Error Occurred ---", exc_info=True)


finally:
    # The ElliptecRotator class handles closing the serial port implicitly
    # when the object is destroyed if it opened the port itself (by passing a string name).
    # No explicit close call is needed here in that case.
    logger.info("\nCleanup: Port (if opened by class) will be closed automatically by ElliptecRotator destructor.")

```