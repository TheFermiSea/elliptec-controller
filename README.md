# Elliptec Controller

A Python package for controlling Thorlabs Elliptec rotation stages (ELL6, ELL14, ELL18, etc.), providing an intuitive interface for optical control applications.

✅ **HARDWARE VALIDATED** - Confirmed working with real Elliptec devices  
✅ **PRODUCTION READY** - 100% core functionality tested  
✅ **GROUP ADDRESSING VERIFIED** - Synchronized multi-rotator control working  
✅ **ASYNCHRONOUS SUPPORT** - Non-blocking operation via threading

## Features

- **Individual Rotator Control**: Control single Elliptec rotation stages with precise positioning
- **Group Synchronization**: Coordinate multiple rotators with configurable offsets *(hardware validated)*
- **Comprehensive Protocol Support**: Full implementation of the ELLx protocol manual
- **Thread-Safe Design**: Safe for use in multi-threaded applications
- **Asynchronous Operation**: Non-blocking commands via dedicated threading *(optimized implementation)*
- **Advanced Logging**: Detailed logging with Loguru for debugging and monitoring
- **Device Information**: Automatic retrieval of device specifications and capabilities
- **Position Conversion**: Seamless conversion between degrees and device-specific pulse counts
- **Hardware Compatibility**: Tested with real μRASHG optical systems

## Installation

### From PyPI (Recommended)

```bash
pip install elliptec-controller
```

### From Source

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install -e .
```

### Development Installation

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install -e .[dev]
```

## Quick Start

### Basic Single Rotator Control

```python
from elliptec_controller import ElliptecRotator
from loguru import logger
import sys

# Configure logging (optional)
logger.remove()
logger.add(sys.stderr, level="INFO")

# Initialize rotator
rotator = ElliptecRotator(
    port="/dev/ttyUSB0",    # Replace with your serial port
    motor_address=1,        # Device address (0-15)
    name="MyRotator"
)

# Basic operations
rotator.home(wait=True)                    # Home the device
rotator.move_absolute(45.0, wait=True)     # Move to 45 degrees
position = rotator.update_position()       # Get current position
print(f"Current position: {position:.2f}°")
```

### Asynchronous Control (Non-blocking)

```python
from elliptec_controller import ElliptecRotator

# Using context manager for automatic thread management
with ElliptecRotator("/dev/ttyUSB0", motor_address=1) as rotator:
    # Commands use async mode by default after connect()
    rotator.home(wait=True)                    # Home the device
    rotator.move_absolute(45.0, wait=False)    # Non-blocking move
    # Do other work while moving...
    rotator.wait_until_ready()                 # Wait when needed
    
    # Mix sync and async as needed
    rotator.move_absolute(90.0, use_async=False)  # Force synchronous
    rotator.move_absolute(180.0, use_async=True)  # Explicit async
    
    # Thread is automatically stopped when exiting context
```

### Command Line Interface

The package includes a CLI tool for quick operations:

```bash
# Get device status
elliptec-controller status --port /dev/ttyUSB0

# Home all connected rotators
elliptec-controller home --port /dev/ttyUSB0

# Move specific rotator to position
elliptec-controller move-abs --port /dev/ttyUSB0 --address 1 --position 90.0

# Get device information
elliptec-controller info --port /dev/ttyUSB0 --address 1
```

## Advanced Usage

### Asynchronous Operation

Control devices with non-blocking commands via dedicated threading:

```python
from elliptec_controller import ElliptecRotator

# Method 1: Using context manager (recommended)
with ElliptecRotator("/dev/ttyUSB0", motor_address=1) as rotator:
    # Thread automatically started by context manager
    rotator.move_absolute(45.0)  # Uses async mode by default
    # Other code runs while device is moving
    
    # Wait only when needed
    rotator.wait_until_ready()
    print(f"Current position: {rotator.position_degrees:.2f}°")
    # Thread automatically stopped when exiting context

# Method 2: Manual thread management
rotator = ElliptecRotator("/dev/ttyUSB0", motor_address=1)
rotator.connect()  # Manually start the async thread

# Mix synchronous and asynchronous as needed
rotator.move_absolute(45.0, use_async=True)   # Explicit async usage
rotator.move_absolute(90.0, use_async=False)  # Force synchronous for this call

rotator.disconnect()  # Manually stop the async thread
```

The implementation uses per-command response queues for improved reliability and clearer error handling.

### Synchronized Group Movement

Control multiple rotators simultaneously with individual offsets:

```python
from elliptec_controller import ElliptecRotator

# Initialize rotators on the same serial port
master = ElliptecRotator("/dev/ttyUSB0", motor_address=1, name="Master")
slave = ElliptecRotator("/dev/ttyUSB0", motor_address=2, name="Slave")

# Configure synchronization
slave_offset = 30.0  # Slave will be offset by 30 degrees
slave.configure_as_group_slave(master.physical_address, slave_offset)

# Synchronized movement - both rotators move together
target_angle = 45.0
master.move_absolute(target_angle, wait=True)
# Master moves to 45°, Slave moves to 75° (45° + 30° offset)

# Cleanup
slave.revert_from_group_slave()
```

### Error Handling and Robustness

```python
import serial
from elliptec_controller import ElliptecRotator
from loguru import logger

try:
    rotator = ElliptecRotator("/dev/ttyUSB0", motor_address=1)
    
    # Check device readiness
    if not rotator.is_ready():
        logger.warning("Device not ready, attempting to home...")
        if not rotator.home(wait=True):
            raise RuntimeError("Failed to home device")
    
    # Perform operations with error checking
    if rotator.move_absolute(90.0, wait=True):
        logger.info("Move completed successfully")
    else:
        logger.error("Move operation failed")
        
except serial.SerialException as e:
    logger.error(f"Serial communication error: {e}")
except Exception as e:
    logger.error(f"Unexpected error: {e}")
    # Emergency stop if needed
    try:
        rotator.stop()
    except:
        pass
```

## Device Compatibility

This package supports Thorlabs Elliptec rotation stages including:

- **ELL6**: 360° rotation mount
- **ELL14**: 360° rotation mount with encoder
- **ELL18**: 360° rotation mount with high resolution

The package automatically detects device-specific parameters such as:
- Pulses per revolution
- Travel range
- Firmware version
- Serial number

## Hardware Setup

1. **Connect Hardware**: Connect your Elliptec rotator via USB
2. **Identify Port**: Find the serial port name:
   - Linux: `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
   - Windows: `COM1`, `COM2`, etc.
   - macOS: `/dev/tty.usbserial-*`
3. **Set Permissions** (Linux/macOS):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

## Logging

The package uses [Loguru](https://loguru.readthedocs.io/) for comprehensive logging:

```python
from loguru import logger
import sys

# Configure logging level
logger.remove()
logger.add(sys.stderr, level="DEBUG")  # Options: TRACE, DEBUG, INFO, WARNING, ERROR

# Logging will show detailed communication and device state information
```

## Testing

### Unit Tests

Run the test suite:

```bash
# Basic test run
pytest

# With coverage
pytest --cov=elliptec_controller

# Verbose output
pytest -v

# Core functionality only (all passing)
pytest tests/test_controller.py -v
```

**Test Status**: ✅ 23/23 core functionality tests passing (100%)

### Hardware Validation

The package includes hardware validation scripts in the `hardware_tests/` directory:

```bash
# Test group addressing (requires 2+ rotators)
python hardware_tests/test_group_simple.py

# Comprehensive group test (requires 3 rotators)
python hardware_tests/test_group_hardware.py
```

**Hardware Status**: ✅ Validated on real Elliptec devices (addresses 2, 3, 8)

See [hardware_tests/README.md](hardware_tests/README.md) for detailed validation procedures and results.

## API Documentation

### ElliptecRotator Class

The main class for controlling individual rotators.

#### Initialization
```python
ElliptecRotator(port, motor_address, name=None, auto_home=True)
```

#### Key Methods
- `home(wait=True)`: Home the rotator
- `move_absolute(degrees, wait=True)`: Move to absolute position
- `move_relative(degrees, wait=True)`: Move by relative amount
- `update_position()`: Get current position
- `get_status()`: Get device status
- `set_velocity(velocity)`: Set movement velocity
- `get_device_info()`: Retrieve device information

#### Thread Management Methods
- `connect()`: Start the async communication thread
- `disconnect()`: Stop the async communication thread
- `__enter__()`, `__exit__()`: Context manager support

#### Group Control Methods
- `configure_as_group_slave(master_address, offset_degrees)`: Configure for synchronized movement
- `revert_from_group_slave()`: Return to individual control

## Examples

The `examples/` directory contains comprehensive usage examples:

- `basic_usage.py`: Single rotator control
- `advanced_usage.py`: Group synchronization and advanced features

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## Development Setup

```bash
git clone https://github.com/TheFermiSea/elliptec-controller.git
cd elliptec-controller
pip install -e .[dev]

# Run tests
pytest

# Format code
black elliptec_controller/

# Type checking
mypy elliptec_controller/
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Validation Status

### ✅ Core Functionality (Production Ready)
- **Individual Control**: 23/23 tests passing
- **Device Communication**: Hardware validated
- **Position Accuracy**: Sub-degree precision confirmed
- **Protocol Implementation**: Complete ELLx support
- **Asynchronous Mode**: Non-blocking operation via threading with per-command response queues

### ✅ Group Addressing (Hardware Validated)
- **Group Formation**: Working on real devices
- **Synchronized Movement**: Multiple rotators coordinated
- **Offset Configuration**: Individual rotator offsets applied
- **Cleanup & Recovery**: Clean reversion to individual control

### 🔧 Test Infrastructure
- **Core Tests**: 100% passing
- **Mock Tests**: Some group controller mocks need refinement
- **Hardware Tests**: Available for validation

## Real-World Usage

This package is actively used in μRASHG (micro Rotational Anisotropy Second Harmonic Generation) experiments with:
- 3 synchronized Elliptec rotators
- Sub-second scanning optimization  
- Reliable group addressing for optical measurements

## Support

- **Issues**: [GitHub Issues](https://github.com/TheFermiSea/elliptec-controller/issues)
- **Documentation**: [docs/](docs/)
- **Thorlabs Manual**: [ELLx Protocol Manual](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9252)
- **Test Status**: [test-status.md](test-status.md)

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for version history and changes.