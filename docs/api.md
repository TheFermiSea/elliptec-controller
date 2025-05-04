# API Documentation

## Classes

### ElliptecRotator

The base class for controlling a single Elliptec rotator.

#### Constructor

```python
ElliptecRotator(serial_connection, motor_address, name=None)
```

- `serial_connection`: A pyserial Serial object or compatible mock
- `motor_address`: Integer address of the rotator (1-31)
- `name`: Optional string identifier for the rotator

#### Methods

##### `send_command(command, data=None, debug=False) -> str`
Send a command to the rotator and return the response.

- `command`: Command string (e.g., "gs", "ma", "sv")
- `data`: Optional data string to send with command
- `debug`: Enable debug output
- Returns: Response string from device

##### `get_status() -> str`
Get the current status of the rotator.
- Returns: Status code (e.g., "00" for OK, "09" for moving)

##### `is_ready() -> bool`
Check if the rotator is ready for commands.
- Returns: True if ready, False if busy or error

##### `wait_until_ready(timeout=None)`
Wait until the rotator is ready or timeout occurs.
- `timeout`: Maximum wait time in seconds (None for no timeout)

##### `home(wait=True) -> bool`
Home the rotator.
- `wait`: Wait for completion if True
- Returns: True if successful

##### `stop() -> bool`
Stop any current movement.
- Returns: True if successful

##### `set_velocity(velocity) -> bool`
Set rotation velocity.
- `velocity`: Integer 0-63
- Returns: True if successful

##### `move_absolute(position, wait=True) -> bool`
Move to absolute position in degrees.
- `position`: Target position in degrees
- `wait`: Wait for completion if True
- Returns: True if successful

##### `move_relative(amount, direction="cw", wait=True) -> bool`
Move by relative amount.
- `amount`: Movement amount in degrees
- `direction`: "cw" or "ccw"
- `wait`: Wait for completion if True
- Returns: True if successful

##### `get_device_info(debug=False) -> dict`
Get device information.
- `debug`: Enable debug output
- Returns: Dictionary of device information

### TripleRotatorController

Controller for three Elliptec rotators.

#### Constructor

```python
TripleRotatorController(port, addresses=[3, 6, 8], names=None)
```

- `port`: Serial port path or name
- `addresses`: List of up to 3 motor addresses
- `names`: Optional list of rotator names

#### Methods

##### `home_all(wait=True) -> bool`
Home all rotators.
- `wait`: Wait for completion if True
- Returns: True if all successful

##### `stop_all() -> bool`
Stop all rotators.
- Returns: True if all successful

##### `is_all_ready() -> bool`
Check if all rotators are ready.
- Returns: True if all ready

##### `set_all_velocities(velocity) -> bool`
Set velocity for all rotators.
- `velocity`: Integer 0-63
- Returns: True if successful

##### `move_all_absolute(positions, wait=True) -> bool`
Move all rotators to absolute positions.
- `positions`: List of positions in degrees
- `wait`: Wait for completion if True
- Returns: True if successful

##### `move_all_relative(amounts, directions=None, wait=True) -> bool`
Move all rotators by relative amounts.
- `amounts`: List of movement amounts in degrees
- `directions`: List of "cw" or "ccw"
- `wait`: Wait for completion if True
- Returns: True if successful

##### `close()`
Close all connections.

## Constants

```python
COMMAND_GET_STATUS = "gs"         # Get status
COMMAND_STOP = "st"              # Stop motion
COMMAND_HOME = "ho"              # Home device
COMMAND_FORWARD = "fw"           # Forward continuous
COMMAND_BACKWARD = "bw"          # Backward continuous
COMMAND_MOVE_ABS = "ma"          # Move absolute
COMMAND_GET_POS = "gp"           # Get position
COMMAND_SET_VELOCITY = "sv"       # Set velocity
COMMAND_JOG_STEP = "sj"          # Set jog step size
COMMAND_GET_INFO = "in"          # Get device info
COMMAND_FREQUENCY_SEARCH = "fs"   # Frequency search
```

## Utility Functions

##### `degrees_to_hex(degrees) -> str`
Convert degrees to hex format.
- `degrees`: Angle in degrees
- Returns: Hex string

##### `hex_to_degrees(hex_str) -> float`
Convert hex string to degrees.
- `hex_str`: Hex string representation
- Returns: Angle in degrees

## Command Line Interface

The package provides a command-line interface through the `elliptec-controller` command:

```bash
elliptec-controller [--port PORT] [--addresses ADDR...] COMMAND [options]
```

See [CLI Documentation](cli.md) for detailed usage.