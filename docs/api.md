# API Documentation

## Classes

### ElliptecRotator

The base class for controlling a single Elliptec rotation stage.

#### Constructor

```python
ElliptecRotator(port, motor_address, name=None, debug=False)
```

- `port`: Serial port name (e.g., `/dev/ttyUSB0`, `COM3`), an open `pyserial.Serial` object, or a compatible mock object for testing. If a port name is given, the class handles opening and closing.
- `motor_address`: Integer address of the rotator (0-15, represented as hex 0-F).
- `name`: Optional string identifier for the rotator.
- `debug`: Boolean, enables detailed debug output if True.

#### Key Attributes

- `physical_address` (str): The device's hardware address (0-F).
- `active_address` (str): The address the device is currently responding to (usually the same as `physical_address` unless in group slave mode).
- `is_slave_in_group` (bool): True if the device is configured via `configure_as_group_slave` to listen to another address.
- `group_offset_degrees` (float): The angular offset applied to this device during group moves.
- `pulse_per_revolution` (int): The number of pulses per 360 degrees, obtained from device info. Used for accurate position conversion.
- `device_info` (dict): Dictionary containing detailed information retrieved from the device using the "in" command.

#### Methods

##### `send_command(command, data=None, debug=False, timeout=None, send_addr_override=None, expect_reply_from_addr=None, timeout_multiplier=1.0) -> str`
Send a command to the rotator and return the response. Handles low-level communication, address formatting, response checking, and timeouts.

- `command` (str): Command string (e.g., "gs", "ma", "sv").
- `data` (str, optional): Optional data string to send with command.
- `debug` (bool): Enable detailed debug output.
- `timeout` (float, optional): Specific timeout in seconds for this command read. Overrides default logic.
- `send_addr_override` (str, optional): Send the command using this address prefix instead of `self.active_address`.
- `expect_reply_from_addr` (str, optional): Expect the response to start with this address prefix instead of `self.active_address`.
- `timeout_multiplier` (float): Multiplies the default timeout duration.
- Returns (str): Response string from device, stripped of CR/LF, or empty string on error/timeout/address mismatch.

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

##### `move_absolute(degrees, wait=True, debug=False) -> bool`
Move to absolute position in degrees.
- `degrees` (float): Target position in degrees (0-360).
- `wait` (bool): Wait for movement completion if True.
- `debug` (bool): Enable debug output for this operation.
- Returns (bool): True if the move command was sent successfully (and completed, if `wait=True`). Applies `group_offset_degrees` automatically if set.

##### `move_relative(degrees, wait=True) -> bool`
Move by a relative amount in degrees.
- `degrees` (float): Amount to move (positive for forward/cw, negative for backward/ccw).
- `wait` (bool): Wait for completion if True.
- Returns (bool): True if successful. Note: Internally uses `move_absolute` based on current position; offset logic applies if grouped.

##### `get_device_info(debug=False) -> dict`
Get detailed device information (serial, firmware, pulse count, etc.) by sending the "in" command. Populates `self.device_info` and `self.pulse_per_revolution`. Called automatically during initialization if a port name is provided.
- `debug` (bool): Enable debug output.
- Returns (dict): Dictionary of device information.

##### `configure_as_group_slave(master_address_to_listen_to, slave_offset=0.0, debug=False) -> bool`
Instruct this rotator (slave) to listen to a different `master_address` for synchronized movement. Sends the "ga" command.
- `master_address_to_listen_to` (str): The address (0-F) this rotator should listen to.
- `slave_offset` (float): Angular offset (degrees) for this slave relative to the group target. Stored in `group_offset_degrees`.
- `debug` (bool): Enable debug output.
- Returns (bool): True if configuration was successful (correct response received). Sets `is_slave_in_group` to True and `active_address` to `master_address_to_listen_to`.

##### `revert_from_group_slave(debug=False) -> bool`
Revert this rotator from slave mode back to its `physical_address`. Sends the "ga" command.
- `debug` (bool): Enable debug output.
- Returns (bool): True if reversion was successful. Resets `is_slave_in_group`, `active_address`, and `group_offset_degrees`.

##### `continuous_move(direction, start=True, debug=False) -> bool`
Start or stop continuous movement using "fw" or "bw" commands.
- `direction` (str): Direction ("fw" for forward, "bw" for backward).
- `start` (bool): True to start movement, False to stop (by sending the "st" command).
- `debug` (bool): Enable debug output.
- Returns (bool): True if the command was sent successfully.

##### `optimize_motors(wait=True) -> bool`
Run the motor optimization routine ("om" command). Applies only to specific devices (e.g., ELL14, ELL18).
- `wait` (bool): Wait for completion (can take time).
- Returns (bool): True if command acknowledged.


## Constants

```python
COMMAND_GET_STATUS = "gs"         # Get status
COMMAND_STOP = "st"              # Stop motion
COMMAND_HOME = "ho"              # Home device
COMMAND_FORWARD = "fw"           # Forward continuous
COMMAND_BACKWARD = "bw"          # Backward continuous
COMMAND_MOVE_ABS = "ma"          # Move absolute
COMMAND_GET_POS = "gp"           # Get position
COMMAND_SET_VELOCITY = \"sv\"       # Set velocity
COMMAND_GET_VELOCITY = \"gv\"       # Get velocity
COMMAND_SET_JOG_STEP = \"sj\"      # Set jog step size
COMMAND_GET_JOG_STEP = \"gj\"      # Get jog step size
COMMAND_SET_HOME_OFFSET = \"so\"   # Set home offset
COMMAND_GET_HOME_OFFSET = \"go\"   # Get home offset
COMMAND_GROUP_ADDRESS = \"ga\"     # Set group address
COMMAND_OPTIMIZE_MOTORS = \"om\"   # Optimize motors
COMMAND_GET_INFO = \"in\"          # Get device info
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

*(Note: The Command Line Interface mentioned in previous versions may be outdated or removed following the refactoring. Please refer to the Python API examples.)*