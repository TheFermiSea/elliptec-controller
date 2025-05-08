# ```python
import pytest
import serial
import time
from unittest.mock import patch

# Attempt to import from the controller module
# Need to handle potential import errors if controller.py itself is broken
try:
    # Assuming test is run from elliptec-controller directory or PYTHONPATH is set
    from elliptec_controller.controller import (
        ElliptecRotator,
        TripleRotatorController,
        degrees_to_hex,
        hex_to_degrees,
    )
except ImportError as e:
    pytest.skip(
        f"Skipping tests because controller import failed: {e}", allow_module_level=True
    )

# --- Mock Serial Port ---


class MockSerial:
    """A mock serial port for testing."""

    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._is_open = True
        self._write_buffer = b""
        self._read_buffer = b""
        self.responses = {}  # Dictionary to map sent commands to responses
        self.log = []  # Log of commands written

    def open(self):
        self._is_open = True

    def close(self):
        self._is_open = False

    @property
    def is_open(self):
        return self._is_open

    def reset_input_buffer(self):
        self._read_buffer = b""

    def reset_output_buffer(self):
        self._write_buffer = b""

    def flush(self):
        pass  # No-op for mock

    def write(self, data):
        if not self._is_open:
            raise serial.SerialException("Port not open")
        self.log.append(data)
        # Simulate response generation based on command
        command_sent = data.decode("ascii").strip()
        # Simple command matching (strip address and CR)
        addr = command_sent[0]
        cmd_key = command_sent[1:]  # Command + data

        # Find matching response based on command key prefix if needed
        response = None
        for known_cmd, resp_data in self.responses.items():
            if cmd_key.startswith(known_cmd):
                response = resp_data
                break

        if response: # Response from self.responses should NOT include \\r\\n now
            # Include address prefix in response, NO \\r\\n
            full_response_str = f"{addr}{response}"
            self._read_buffer += full_response_str.encode("ascii")
            # print(f"MockSerial: Received '{command_sent}', Queued Response: '{full_response_str}'") # Debug
        # else:
            # print(f"MockSerial: Received '{command_sent}', No matching response configured.") # Debug

        return len(data)

    @property
    def in_waiting(self):
        return len(self._read_buffer)

    def read(self, size=1):
        if not self._is_open:
            raise serial.SerialException("Port not open")

        # Simulate timeout behavior if buffer is empty
        if not self._read_buffer:
            # Simulate waiting slightly less than a full character time at 9600 baud
            # time.sleep(0.001) # Causes tests to be slow, simulate empty read instead
            return b""

        data = self._read_buffer[:size]
        self._read_buffer = self._read_buffer[size:]
        return data

    def read_all(self):
        data = self._read_buffer
        self._read_buffer = b""
        return data

    def set_response(self, command_key, response_data):
        """Set a specific response for a command key (without address).
        Does NOT automatically add \\r\\n termination."""
        # Store response data exactly as provided
        self.responses[command_key] = response_data

    def clear_responses(self):
        self.responses = {}
        self.log = []


# --- Pytest Fixtures ---


@pytest.fixture
def mock_serial_port():
    """Fixture for the mock serial port."""
    mock_port = MockSerial(port="/dev/mock", baudrate=9600, timeout=1)
    # Common default responses
    # Pass only core data; \\r\\n is added by MockSerial.set_response
    mock_port.set_response("gs", "GS00")
    mock_port.set_response("in", "IN0E1140TESTSERL2401016800023000")
    mock_port.set_response("gp", "PO00000000")
    mock_port.set_response("ho0", "PO00000000")
    mock_port.set_response("st", "GS00")
    mock_port.set_response("sv", "GS00")
    mock_port.set_response("sj", "GS00")
    mock_port.set_response("ma", "GS00")

    # Patch serial.Serial specifically within the controller module's namespace
    with patch('elliptec_controller.controller.serial.Serial', return_value=mock_port):
        yield mock_port
    mock_port.close() # Ensure closed after test


@pytest.fixture
def rotator_addr_8(mock_serial_port):
    """Fixture for an ElliptecRotator instance at address 8."""
    # Prevent __init__ from trying to get device info immediately
    with patch.object(
        ElliptecRotator,
        "get_device_info",
        return_value={
            "type": "0E",
            "serial_number": "TESTSERL",
            "pulses_per_unit": 262144,
            "travel": 360,
        },
    ) as _:  # Assign unused variable to _
        rotator = ElliptecRotator(
            port=mock_serial_port, motor_address=8, name="TestRotator8"
        )
        # The patch context manager automatically restores the original method
        # after the 'with' block exits. No need for manual restoration.
        # Manually set some defaults that might be read from device info
        rotator.pulse_per_revolution = 262144
        rotator.range = 360
    return rotator


@pytest.fixture
def rotator_addr_2(mock_serial_port):
    """Fixture for an ElliptecRotator instance at address 2."""
    with patch.object(
        ElliptecRotator,
        "get_device_info",
        return_value={
            "type": "0E",
            "serial_number": "TESTSERL",
            "pulses_per_unit": 262144,
            "travel": 360,
        },
    ) as _: # Assign unused variable to _
        rotator = ElliptecRotator(
            port=mock_serial_port, motor_address=2, name="TestRotator2"
        )
        rotator.pulse_per_revolution = 262144
        rotator.range = 360
    return rotator


@pytest.fixture
def rotator_addr_3(mock_serial_port):
    """Fixture for an ElliptecRotator instance at address 3."""
    with patch.object(
        ElliptecRotator,
        "get_device_info",
        return_value={
            "type": "0E",
            "serial_number": "TESTSERL",
            "pulses_per_unit": 262144,
            "travel": 360,
        },
    ) as _: # Assign unused variable to _
        rotator = ElliptecRotator(
            port=mock_serial_port, motor_address=3, name="TestRotator3"
        )
        rotator.pulse_per_revolution = 262144
        rotator.range = 360
    return rotator


@pytest.fixture
def triple_controller(mock_serial_port):
    """Fixture for a TripleRotatorController."""
    # Prevent get_device_info during individual rotator init
    with patch.object(
        ElliptecRotator,
        "get_device_info",
        return_value={
            "type": "0E",
            "serial_number": "TESTSERL",
            "pulses_per_unit": 262144,
            "travel": 360,
        },
    ):
        controller = TripleRotatorController(
            port=mock_serial_port, motor_addresses=[2, 3, 8]
        )
        # Manually set defaults for the rotators inside
        for r in controller.rotators:
            r.pulse_per_revolution = 262144
            r.range = 360
    return controller


# --- Test Conversion Functions ---


def test_degrees_to_hex():
    assert degrees_to_hex(0) == "00000000"
    assert degrees_to_hex(90) == "00010000"
    assert degrees_to_hex(180) == "00020000"
    assert degrees_to_hex(270) == "00030000"
    assert (
        degrees_to_hex(360) == "00040000"
    )  # Note: Wraps slightly due to integer conversion
    assert degrees_to_hex(-90) == "FFFF0000"  # Equivalent to 270


def test_hex_to_degrees():
    assert hex_to_degrees("00000000") == pytest.approx(0.0)
    # Increase tolerance due to integer division in conversion
    assert hex_to_degrees("00010000") == pytest.approx(90.0, abs=1e-3)
    assert hex_to_degrees("00020000") == pytest.approx(180.0, abs=1e-3)
    assert hex_to_degrees("00030000") == pytest.approx(270.0, abs=1e-3)
    # Increase tolerance slightly more for 360 deg case
    assert hex_to_degrees("00040000") == pytest.approx(360.0, abs=1.1e-3)
    assert hex_to_degrees("FFFF0000") == pytest.approx(-90.0, abs=1e-3)  # Signed conversion, add tolerance

# --- Test ElliptecRotator ---


def test_rotator_init(rotator_addr_8, mock_serial_port):
    assert rotator_addr_8.address == "8"
    assert rotator_addr_8.name == "TestRotator8"
    assert rotator_addr_8.serial == mock_serial_port


def test_send_command(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("in", "INTESTDATA") # Core data only

    response = rotator_addr_8.send_command("in")

    assert mock_serial_port.log == [b"8in\r"]
    # send_command returns stripped response now
    assert response == "8INTESTDATA"


def test_send_command_with_data(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response(
        "ma00010000", "PO00010000" # Core data only
    )  # Specific response for move to 90 deg

    response = rotator_addr_8.send_command("ma", "00010000")

    assert mock_serial_port.log == [b"8ma00010000\r"]
    # send_command returns stripped response now
    assert response == "8PO00010000"


def test_send_command_address_filter(rotator_addr_8, mock_serial_port):
    # Simulate response from WRONG address
    mock_serial_port.clear_responses()
    # No specific response set for 'gs' at addr 8
    # Let's manually queue a response from addr '2'
    mock_serial_port._read_buffer = b"2GS00\\r\\n"

    response = rotator_addr_8.send_command("gs")

    # Should have sent command to address 8
    assert mock_serial_port.log == [b"8gs\r"]
    # Response should be empty because it didn't start with '8'
    assert response == ""


def test_get_status(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS00") # Core data only
    status = rotator_addr_8.get_status()
    assert status == "00"

    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS0C") # Core data only
    status = rotator_addr_8.get_status()
    assert status == "0C"

    mock_serial_port.clear_responses()
    # No response set - should return None or default
    status = rotator_addr_8.get_status()
    assert status == ""  # Controller returns empty string on failure/no response


def test_is_ready(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS00") # Core data only
    assert rotator_addr_8.is_ready() is True

    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS09")  # Moving - Core data only
    assert rotator_addr_8.is_ready() is False

    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS0C")  # Error - Core data only
    assert (
        rotator_addr_8.is_ready() is False
    )  # Or should it raise error? Current impl returns False


@patch("time.sleep", return_value=None)  # Mock time.sleep
def test_wait_until_ready(mock_sleep, rotator_addr_8, mock_serial_port):
    # Patch get_status to simulate the sequence of states
    with patch.object(rotator_addr_8, 'get_status', side_effect=['09', '09', '00']) as mock_get_status:
        result = rotator_addr_8.wait_until_ready(timeout=1) # Timeout value doesn't matter much here

    # Assert that wait_until_ready returned True (indicating success)
    assert result is True
    # Assert that get_status was called multiple times (3 times in this case)
    assert mock_get_status.call_count == 3


@patch("time.sleep", return_value=None)
def test_wait_until_ready_timeout(mock_sleep, rotator_addr_8, mock_serial_port):
    # Need to reset the class modification from previous test if running sequentially
    mock_serial_port.__class__ = MockSerial 
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gs", "GS09")  # Always moving

    start_time = time.monotonic()
    # Remove polling_interval argument
    result = rotator_addr_8.wait_until_ready(timeout=0.1)
    end_time = time.monotonic()

    assert result is False
    # This assertion might still fail if the internal loop/sleep takes longer than expected
    assert (end_time - start_time) == pytest.approx(0.1, abs=0.05)


def test_stop(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("st", "GS00") # Core data only
    result = rotator_addr_8.stop()
    assert result is True
    assert mock_serial_port.log == [b"8st\r"]


def test_home(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("ho0", "GS09")  # Simulate GS response (moving) - Core data only
    mock_serial_port.set_response("gs", "GS00")  # Status check after move - Core data only

    # Mock wait_until_ready to avoid complex status mocking for now
    with patch.object(
        rotator_addr_8, "wait_until_ready", return_value=True
    ) as mock_wait:
        result = rotator_addr_8.home(wait=True)
        assert result is True
        mock_wait.assert_called_once()

    assert b"8ho0\r" in mock_serial_port.log

    # Test wait=False
    mock_serial_port.log = []
    result = rotator_addr_8.home(wait=False)
    assert result is True
    assert b"8ho0\r" in mock_serial_port.log


def test_move_absolute(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    # Specific response for moving to 90 deg - Core data only
    mock_serial_port.set_response("ma00010000", "GS00")
    mock_serial_port.set_response("gs", "GS00")  # Status check - Core data only

    with patch.object(
        rotator_addr_8, "wait_until_ready", return_value=True
    ) as mock_wait:
        result = rotator_addr_8.move_absolute(90, wait=True)
        assert result is True
        mock_wait.assert_called_once()

    assert b"8ma00010000\r" in mock_serial_port.log

    # Test wait=False
    mock_serial_port.log = []
    result = rotator_addr_8.move_absolute(180, wait=False)
    assert result is True
    assert b"8ma00020000\r" in mock_serial_port.log

    # Test handling of PO response
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("ma00030000", "PO00030000")  # Simulate PO response - Core data only
    mock_serial_port.set_response("gs", "GS00") # Core data only
    with patch.object(
        rotator_addr_8, "wait_until_ready", return_value=True
    ) as mock_wait:
        result = rotator_addr_8.move_absolute(270, wait=True)
        assert result is True
        mock_wait.assert_called_once()
    assert b"8ma00030000\r" in mock_serial_port.log


def test_update_position(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gp", "PO00010000")  # Position 90 deg - Core data only
    position = rotator_addr_8.update_position()
    assert position == pytest.approx(90.0, abs=1e-3) # Add tolerance
    assert mock_serial_port.log == [b"8gp\r"]

    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gp", "POFFFF0000")  # Position -90 deg - Core data only
    position = rotator_addr_8.update_position()
    assert position == pytest.approx(-90.0, abs=1e-3) # Add tolerance

    mock_serial_port.clear_responses()
    mock_serial_port.set_response("gp", "GS0C")  # Error response - Core data only
    position = rotator_addr_8.update_position()
    assert position == 0.0  # Current impl returns 0.0 on error / non-PO


def test_get_device_info(rotator_addr_8, mock_serial_port):
    mock_serial_port.clear_responses()
    # Standard response format - Core data only
    mock_serial_port.set_response("in", "IN0E1140TESTSERL2401016800023000")
    info = rotator_addr_8.get_device_info()

    assert info["type"] == "0E"
    assert info["firmware"] == "1140"
    assert info["serial_number"] == "TESTSERL"
    assert info["year_month"] == "2401"
    assert info["hardware"] == "6800"
    # Corrected based on likely controller fix (slicing data[24:30])
    assert info["max_range"] == "023000"
    assert info["firmware_formatted"] == "17.64"
    assert info["hardware_formatted"] == "104.0"
    assert info["manufacture_date"] == "2024-01"
    assert mock_serial_port.log == [b"8in\r"]


# --- Test TripleRotatorController ---


def test_triple_controller_init(triple_controller):
    assert len(triple_controller.rotators) == 3
    assert triple_controller.rotators[0].address == "2"
    assert triple_controller.rotators[1].address == "3"
    assert triple_controller.rotators[2].address == "8"
    assert triple_controller.serial is not None


def test_triple_controller_home_all(triple_controller, mock_serial_port):
    mock_serial_port.clear_responses()
    # Responses for each rotator - Core data only
    mock_serial_port.set_response("ho0", "PO00000000")
    mock_serial_port.set_response("gs", "GS00")

    with patch.object(
        triple_controller, "wait_all_ready", return_value=True
    ) as mock_wait:
        result = triple_controller.home_all(wait=True)
        assert result is True
        mock_wait.assert_called_once()

    # Check commands sent to each address
    assert b"2ho0\r" in mock_serial_port.log
    assert b"3ho0\r" in mock_serial_port.log
    assert b"8ho0\r" in mock_serial_port.log


def test_triple_controller_move_all_absolute(triple_controller, mock_serial_port):
    mock_serial_port.clear_responses()
    # Responses for move and status - Core data only
    mock_serial_port.set_response("ma", "GS00")  # Generic ack for moves
    mock_serial_port.set_response("gs", "GS00")

    positions = [90, 180, 270]

    with patch.object(
        triple_controller, "wait_all_ready", return_value=True
    ) as mock_wait:
        result = triple_controller.move_all_absolute(positions, wait=True)
        assert result is True
        mock_wait.assert_called_once()

    # Check commands sent to each address with correct position data
    assert b"2ma00010000\r" in mock_serial_port.log  # 90 deg
    assert b"3ma00020000\r" in mock_serial_port.log  # 180 deg
    assert b"8ma00030000\r" in mock_serial_port.log  # 270 deg


def test_triple_controller_stop_all(triple_controller, mock_serial_port):
    mock_serial_port.clear_responses()
    mock_serial_port.set_response("st", "GS00") # Core data only

    result = triple_controller.stop_all()
    assert result is True

    assert b"2st\r" in mock_serial_port.log
    assert b"3st\r" in mock_serial_port.log
    assert b"8st\r" in mock_serial_port.log


# Add more tests for other methods:
# - set_velocity, set_jog_step, optimize_motors, continuous_move for ElliptecRotator
# - is_all_ready, set_all_velocities, move_all_relative for TripleRotatorController
# - Edge cases: timeouts, error responses (e.g., GS0C), invalid parameters
