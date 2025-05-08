# Elliptec Controller

A Python package for controlling Thorlabs Elliptec rotation stages (like ELL6, ELL14, etc.), providing an intuitive interface for optical control applications.

## Features

- Control individual Elliptec rotation stages.
- Support for relative and absolute positioning.
- Device information retrieval (serial number, firmware, pulse counts).
- Velocity and jog step control.
- Two-device synchronized movement using group addressing with user-defined offsets.
- Comprehensive command set implementation based on the ELLx protocol manual.
- Thread-safe design for multi-threaded applications.

## Installation

```bash
pip install elliptec-controller
```

Or directly from the repository:

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install .
```

## Basic Usage (Single Rotator)

```python
import serial
from elliptec_controller import ElliptecRotator
import time

# Using a specific serial port
try:
    # Create a rotator instance - this opens the port and gets device info
    rotator1 = ElliptecRotator(
        port="/dev/ttyUSB0",  # Replace with your serial port
        motor_address=1,      # Replace with your device address (0-F)
        name="Rotator1",
        debug=True           # Enable debug output
    )

    # Print device info (retrieved during initialization)
    print("Device Info:", rotator1.device_info)
    print(f"Pulses per Revolution: {rotator1.pulse_per_revolution}")

    # Home the rotator
    print("Homing...")
    rotator1.home(wait=True)
    print("Homing complete.")
    time.sleep(1)

    # Move to an absolute position (in degrees)
    print("Moving to 90 degrees...")
    rotator1.move_absolute(90.0, wait=True)
    print("Move complete.")
    time.sleep(1)

    # Get the current position
    position = rotator1.update_position()
    print(f"Current position: {position:.2f} degrees")

    # Clean up (closes the serial port implicitly if opened by the class)
    # No explicit close needed if port name was passed to constructor

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except Exception as e:
    print(f"An error occurred: {e}")

```

## Two-Device Synchronized Movement with Offset

This example shows how to synchronize two rotators connected to the *same* serial port using the group addressing feature. One rotator acts as the "master" (commands are sent to its address), and the other acts as the "slave" (configured to listen to the master's address).

```python
import serial
from elliptec_controller import ElliptecRotator
import time

# Note: Both rotators must be on the same serial port for this to work.
SERIAL_PORT = "/dev/ttyUSB0" # Replace with your serial port
MASTER_ADDR = 1            # Replace with master's address (0-F)
SLAVE_ADDR = 2             # Replace with slave's address (0-F)

try:
    # Use context managers for reliable port handling if sharing a port manually
    # If passing port name to ElliptecRotator, it handles opening/closing.
    # Here, we instantiate them separately.
    master_rot = ElliptecRotator(SERIAL_PORT, MASTER_ADDR, "Master", debug=True)
    slave_rot = ElliptecRotator(SERIAL_PORT, SLAVE_ADDR, "Slave", debug=True)

    # --- Synchronization Setup ---
    slave_offset_deg = 30.0  # Slave will move to (target + 30 degrees)
    master_offset_deg = 0.0  # Master moves to target directly

    print(f"Configuring Slave (Addr {slave_rot.physical_address}) to listen to Master (Addr {master_rot.physical_address}) with {slave_offset_deg} deg offset...")
    if slave_rot.configure_as_group_slave(master_rot.physical_address, slave_offset_deg, debug=True):
        print("Slave configured successfully.")
        # Optionally set an offset for the master itself for the group move
        master_rot.group_offset_degrees = master_offset_deg
    else:
        print("Failed to configure slave!")
        raise RuntimeError("Slave configuration failed")

    # --- Perform Synchronized Move ---
    target_angle = 45.0
    print(f"Sending synchronized move command to Master address {master_rot.physical_address} for logical target {target_angle} deg...")
    # Command is sent to master_rot's address. Both master and slave (listening on master's addr) will react.
    # Each applies its own offset internally.
    if master_rot.move_absolute(target_angle, wait=True, debug=True):
        print("Synchronized move command sent and completed.")
    else:
        print("Synchronized move failed.")

    time.sleep(1)
    # Verify positions (update_position handles offset adjustment for slaves)
    master_pos = master_rot.update_position(debug=True)
    slave_pos_logical = slave_rot.update_position(debug=True) # update_position returns the logical position for a slave
    print(f"Master final position: {master_pos:.2f} deg (Expected physical: {target_angle + master_offset_deg:.2f})")
    # Note: Slave's *physical* position would be target_angle + slave_offset_deg
    # update_position for a slave returns the logical position (physical - offset)
    print(f"Slave final logical position: {slave_pos_logical:.2f} deg (Expected logical: {target_angle:.2f})")


    # --- Revert Synchronization ---
    print("Reverting slave from group mode...")
    if slave_rot.revert_from_group_slave(debug=True):
        print("Slave reverted successfully.")
    else:
        print("Failed to revert slave.")
    master_rot.group_offset_degrees = 0.0 # Clear master offset too

    # Rotators now respond to their individual physical addresses again.

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Ensure ports are closed if necessary - ElliptecRotator closes implicitly
    # if it opened the port based on a string name. If you passed an open
    # serial object, you might need to close it manually here.
    print("Cleanup done.")

```

## Documentation

For detailed documentation on all available commands and features, please refer to the [Thorlabs Elliptec documentation](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9252) and the docstrings within the code.

## Testing

The test suite uses `pytest` and mocks serial communication. To run the tests:

1.  **Install test dependencies:** Make sure `pytest` and `pyserial` are available. If defined in `pyproject.toml` under `[project.optional-dependencies.test]`, you can install them using:
    ```bash
    # If using standard pip/venv
    pip install -e .[test]

    # If using uv
    uv pip install -e .[test]
    ```
    Alternatively, install manually: `pip install pytest pyserial` or `uv pip install pytest pyserial`.

2.  **Run pytest:** Navigate to the project root directory (`elliptec-controller`) and run:
    ```bash
    pytest tests/

    # Or, if using uv to manage the environment:
    uv run pytest tests/
    ```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.