# Elliptec Controller

A Python package for controlling Thorlabs Elliptec rotators, providing an intuitive interface for optical control applications.

## Features

- Control individual Elliptec rotators (ELL6, ELL14, etc.)
- Triple rotator control for common optical setups (e.g., half-wave and quarter-wave plates)
- Support for relative and absolute positioning
- Comprehensive command set for all Elliptec operations
- Thread-safe design for multi-threaded applications

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

## Quick Start

```python
from elliptec_controller import TripleRotatorController

# Initialize a controller with three rotators
controller = TripleRotatorController(
    port="/dev/ttyUSB0",  # Replace with your serial port
    addresses=[3, 6, 8],  # Device addresses
    names=["HWP1", "QWP", "HWP2"]  # Optional names
)

# Home all rotators
controller.home_all(wait=True)

# Move rotators to specific positions
controller.move_all_absolute([45.0, 30.0, 60.0], wait=True)

# Close when done
controller.close()
```

## Single Rotator Usage

```python
import serial
from elliptec_controller import ElliptecRotator

# Open a serial connection
ser = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# Create a rotator instance
rotator = ElliptecRotator(ser, motor_address=1, name="MainRotator")

# Home the rotator
rotator.home()

# Move to an absolute position (in degrees)
rotator.move_absolute(90)

# Get the current position
status = rotator.get_status()
print(f"Current status: {status}")

# Close when done
ser.close()
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