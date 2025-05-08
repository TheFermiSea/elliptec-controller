```python
#!/usr/bin/env python3
"""
Basic Usage Example for Elliptec Controller
Demonstrates connecting to a single rotator, homing, moving, and getting status.
"""

import serial
from elliptec_controller import ElliptecRotator
import time

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"  # <-- IMPORTANT: Replace with your serial port name
MOTOR_ADDRESS = 1           # <-- IMPORTANT: Replace with your device's address (0-F)

# Using a specific serial port
try:
    print(f"Connecting to rotator at address {MOTOR_ADDRESS} on port {SERIAL_PORT}...")
    # Create a rotator instance - this opens the port and gets device info
    rotator1 = ElliptecRotator(
        port=SERIAL_PORT,
        motor_address=MOTOR_ADDRESS,
        name=f"Rotator-{MOTOR_ADDRESS}",
        debug=True           # Enable debug output for detailed info
    )

    # Print device info (retrieved during initialization)
    print("\n--- Device Information ---")
    if rotator1.device_info and rotator1.device_info.get("type") != "Unknown":
        print(f"  Device Info: {rotator1.device_info}")
        print(f"  Pulses per Revolution: {rotator1.pulse_per_revolution}")
    else:
        print("  Could not retrieve valid device info.")
        # Optionally raise an error or exit if info is critical
        # raise ConnectionError("Failed to get device info")

    # Home the rotator
    print("\n--- Homing ---")
    print("Homing...")
    if rotator1.home(wait=True):
        print("Homing complete.")
    else:
        print("Homing failed.")
    time.sleep(1)

    # Move to an absolute position (in degrees)
    target_pos = 90.0
    print(f"\n--- Moving to {target_pos} degrees ---")
    if rotator1.move_absolute(target_pos, wait=True):
        print("Move complete.")
    else:
        print("Move failed.")
    time.sleep(1)

    # Get the current position
    print("\n--- Checking Position ---")
    position = rotator1.update_position(debug=True)
    print(f"Current reported position: {position:.2f} degrees")

    # Example of getting status code
    status_code = rotator1.get_status(debug=True)
    print(f"Current status code: {status_code} (00 means OK/Ready)")

    print("\nBasic usage example finished.")

except serial.SerialException as e:
    print(f"\n--- Serial Port Error ---")
    print(f"Could not open or communicate on port '{SERIAL_PORT}'.")
    print(f"Error details: {e}")
    print("Please check if the port name is correct and the device is connected.")
except Exception as e:
    print(f"\n--- An Error Occurred ---")
    print(f"Error details: {e}")

finally:
    # The ElliptecRotator class handles closing the serial port implicitly
    # when the object is destroyed if it opened the port itself (by passing a string name).
    # No explicit close call is needed here in that case.
    print("\nCleanup: Port will be closed automatically if opened by the class.")

```