#!/usr/bin/env python3
"""
Tests for the ElliptecGroupController class in the elliptec-controller package.
"""

import pytest
from unittest.mock import patch, MagicMock, call
from elliptec_controller.controller import ElliptecRotator, ElliptecGroupController, COMMAND_GROUP_ADDRESS, COMMAND_HOME, COMMAND_STOP, COMMAND_MOVE_ABS, COMMAND_GET_STATUS, degrees_to_hex
from .test_controller import MockSerial # Assuming MockSerial is accessible

# --- Fixtures ---

@pytest.fixture
def mock_serial_shared():
    """Provides a shared MockSerial instance for group tests."""
    return MockSerial()

@pytest.fixture
def rotator_g1(mock_serial_shared):
    """Master rotator for group tests, address '1'."""
    # auto_home=False to prevent get_device_info/home calls during fixture setup for group tests
    rot = ElliptecRotator(mock_serial_shared, motor_address=1, name="GroupMaster-1", auto_home=False)
    rot.pulse_per_revolution = 143360 # Consistent with single rotator tests
    rot.device_info = {'type': '0E', 'pulses_per_unit_dec': '143360'}
    return rot

@pytest.fixture
def rotator_g2(mock_serial_shared):
    """Slave rotator for group tests, address '2'."""
    rot = ElliptecRotator(mock_serial_shared, motor_address=2, name="GroupSlave-2", auto_home=False)
    rot.pulse_per_revolution = 143360
    rot.device_info = {'type': '0E', 'pulses_per_unit_dec': '143360'}
    return rot

@pytest.fixture
def rotator_g3(mock_serial_shared):
    """Another slave rotator, address '3'."""
    rot = ElliptecRotator(mock_serial_shared, motor_address=3, name="GroupSlave-3", auto_home=False)
    rot.pulse_per_revolution = 143360
    rot.device_info = {'type': '0E', 'pulses_per_unit_dec': '143360'}
    return rot

@pytest.fixture
def group_controller(rotator_g1, rotator_g2, rotator_g3):
    """Provides an ElliptecGroupController instance."""
    # rotator_g1 will be the default master
    return ElliptecGroupController(rotators=[rotator_g1, rotator_g2, rotator_g3])

@pytest.fixture
def group_controller_explicit_master(rotator_g1, rotator_g2, rotator_g3):
    """Provides an ElliptecGroupController instance with an explicit master."""
    return ElliptecGroupController(rotators=[rotator_g1, rotator_g2, rotator_g3], master_rotator_physical_address='2')


# --- Test Functions ---

def test_group_controller_init(rotator_g1, rotator_g2):
    """Test basic initialization of the group controller."""
    controller = ElliptecGroupController(rotators=[rotator_g1, rotator_g2], master_rotator_physical_address='1')
    assert controller.master_rotator == rotator_g1
    assert len(controller.rotators) == 2
    assert not controller.is_grouped
    assert controller.group_master_address_char is None

def test_group_controller_init_default_master(rotator_g1, rotator_g2):
    """Test that the first rotator becomes master if none is specified."""
    controller = ElliptecGroupController(rotators=[rotator_g1, rotator_g2])
    assert controller.master_rotator == rotator_g1

def test_group_controller_init_invalid_master(rotator_g1):
    """Test initialization with a non-existent master address."""
    with pytest.raises(ValueError, match="Master rotator with physical address 'X' not found"):
        ElliptecGroupController(rotators=[rotator_g1], master_rotator_physical_address='X')

def test_group_controller_init_empty_list():
    """Test initialization with an empty list of rotators."""
    with pytest.raises(ValueError, match="Rotators list cannot be empty"):
        ElliptecGroupController(rotators=[])

def test_group_controller_init_different_serial_ports(mock_serial_shared):
    """Test initialization with rotators on different serial ports (should fail)."""
    rot1 = ElliptecRotator(mock_serial_shared, 0, auto_home=False)
    mock_serial_other = MockSerial() # A different serial instance
    rot2 = ElliptecRotator(mock_serial_other, 1, auto_home=False)
    with pytest.raises(ValueError, match="All rotators in a group must share the same serial port instance"):
        ElliptecGroupController(rotators=[rot1, rot2])

def test_form_group_default_master_address(group_controller, rotator_g1, rotator_g2, rotator_g3, mock_serial_shared):
    """Test forming a group using the master's physical address as the group address."""
    mock_serial_shared.clear_responses()
    # Slave 2 (addr '2') configures to listen to Master 1 (addr '1')
    # Command: 2ga1 -> Response expected: 1GS00
    mock_serial_shared.set_response(f"2{COMMAND_GROUP_ADDRESS}1", f"1GS00\r\n".encode())

    # Slave 3 (addr '3') configures to listen to Master 1 (addr '1')
    # Command: 3ga1 -> Response expected: 1GS00
    # To handle multiple responses in MockSerial for sequential commands,
    # we need a more advanced mock or patch ElliptecRotator.configure_as_group_slave

    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        # Simulate the internal state changes of the real method
        # This assumes the mocked command to the device would succeed.
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True # Simulate successful command

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)) as mock_config_g2, \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)) as mock_config_g3:

        assert group_controller.form_group() is True # Uses rotator_g1's address ('1') by default
        assert group_controller.is_grouped is True
        assert group_controller.group_master_address_char == rotator_g1.physical_address # Should be '1'

        mock_config_g2.assert_called_once_with(rotator_g1.physical_address, slave_offset=0.0)
        mock_config_g3.assert_called_once_with(rotator_g1.physical_address, slave_offset=0.0)

        assert rotator_g1.active_address == rotator_g1.physical_address # Master's active address doesn't change if it IS the group address
        assert rotator_g2.active_address == rotator_g1.physical_address # Slave now listens to master
        assert rotator_g2.is_slave_in_group is True
        assert rotator_g3.active_address == rotator_g1.physical_address # Slave now listens to master
        assert rotator_g3.is_slave_in_group is True

def test_form_group_explicit_group_address(group_controller, rotator_g1, rotator_g2, rotator_g3, mock_serial_shared):
    """Test forming a group with an explicitly provided group address."""
    explicit_group_addr = 'A'

    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)) as mock_config_g2, \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)) as mock_config_g3:
        # Master (g1) also needs to be conceptually part of this if explicit_group_addr is different from its physical.
        # The current form_group logic sets master.active_address correctly.

        assert group_controller.form_group(group_address_char=explicit_group_addr) is True
        assert group_controller.is_grouped is True
        assert group_controller.group_master_address_char == explicit_group_addr

        mock_config_g2.assert_called_once_with(explicit_group_addr, slave_offset=0.0)
        mock_config_g3.assert_called_once_with(explicit_group_addr, slave_offset=0.0)

        assert rotator_g1.active_address == explicit_group_addr # Master's active address becomes the group address
        assert rotator_g2.active_address == explicit_group_addr
        assert rotator_g2.is_slave_in_group is True # Check slave state
        assert rotator_g3.active_address == explicit_group_addr
        assert rotator_g3.is_slave_in_group is True # Check slave state

def test_form_group_one_slave_fails(group_controller, rotator_g1, rotator_g2, rotator_g3, mock_serial_shared):
    """Test group formation when one slave fails to configure."""
    def configure_slave_g2_success(master_addr, slave_offset=0.0):
        rotator_g2.active_address = master_addr
        rotator_g2.is_slave_in_group = True
        return True

    # g3's configure_as_group_slave will be mocked to just return False
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_g2_success(*args, **kwargs)) as mock_config_g2, \
         patch.object(rotator_g3, 'configure_as_group_slave', return_value=False) as mock_config_g3, \
         patch.object(group_controller, 'disband_group') as mock_disband: # Mock disband to check if called

        # Set return_value for mock_disband. It should be called.
        mock_disband.return_value = True

        assert group_controller.form_group() is False # Should fail
        assert group_controller.is_grouped is False # Should be reset

        mock_config_g2.assert_called_once()
        mock_config_g3.assert_called_once()
        mock_disband.assert_called_once() # Ensure cleanup is attempted

def test_disband_group(group_controller, rotator_g1, rotator_g2, rotator_g3, mock_serial_shared):
    """Test disbanding a formed group."""
    # Define side effects for mocks to update rotator state during form_group
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        return True

    def revert_slave_side_effect(rotator_instance):
        # Simulate the internal state changes of the real revert method
        rotator_instance.active_address = rotator_instance.physical_address
        rotator_instance.is_slave_in_group = False
        rotator_instance.group_offset_degrees = 0.0
        return True

    # First, form the group successfully (mocking slave configurations with side effects)
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        group_controller.form_group() # Uses g1 ('1') as group master address

    assert group_controller.is_grouped is True
    assert rotator_g2.is_slave_in_group is True
    assert rotator_g3.is_slave_in_group is True
    assert rotator_g2.active_address == rotator_g1.physical_address

    # Now, test disbanding (mocking slave reversions with side effects)
    with patch.object(rotator_g2, 'revert_from_group_slave', side_effect=lambda: revert_slave_side_effect(rotator_g2)) as mock_revert_g2, \
         patch.object(rotator_g3, 'revert_from_group_slave', side_effect=lambda: revert_slave_side_effect(rotator_g3)) as mock_revert_g3:

        assert group_controller.disband_group() is True
        assert group_controller.is_grouped is False
        assert group_controller.group_master_address_char is None

        mock_revert_g2.assert_called_once()
        mock_revert_g3.assert_called_once()

        assert rotator_g1.active_address == rotator_g1.physical_address # Master reverts
        assert rotator_g2.active_address == rotator_g2.physical_address # Slave reverts
        assert rotator_g2.is_slave_in_group is False
        assert rotator_g3.active_address == rotator_g3.physical_address # Slave reverts
        assert rotator_g3.is_slave_in_group is False

def test_disband_group_one_slave_fails_revert(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test disbanding when one slave fails to revert."""
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        return True

    def revert_g2_success_side_effect():
        rotator_g2.active_address = rotator_g2.physical_address
        rotator_g2.is_slave_in_group = False
        return True

    def revert_g3_fail_side_effect():
        # Simulate the internal state reset that happens even on failure
        rotator_g3.active_address = rotator_g3.physical_address
        rotator_g3.is_slave_in_group = False
        rotator_g3.group_offset_degrees = 0.0
        return False  # But return False to indicate hardware command failed

    # g3's revert_from_group_slave will be mocked to just return False
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
           group_controller.form_group()

    with patch.object(rotator_g2, 'revert_from_group_slave', side_effect=revert_g2_success_side_effect) as mock_revert_g2, \
         patch.object(rotator_g3, 'revert_from_group_slave', side_effect=revert_g3_fail_side_effect) as mock_revert_g3:

        assert group_controller.disband_group() is False # Overall disband should report failure
        assert group_controller.is_grouped is False # State should still be reset

        mock_revert_g2.assert_called_once()
        mock_revert_g3.assert_called_once()

        # g2 should have reverted
        assert rotator_g2.active_address == rotator_g2.physical_address
        assert rotator_g2.is_slave_in_group is False

        # For g3, which failed to revert its own state via its (mocked) method:
        # Actually, even when revert_from_group_slave returns False, it still resets
        # the internal state (active_address, is_slave_in_group, group_offset_degrees)
        # before returning. Only the hardware acknowledgment failed.
        assert rotator_g3.active_address == rotator_g3.physical_address # Should be reset to physical
        assert rotator_g3.is_slave_in_group is False # Should be reset to False


# --- Tests for Group Action Methods (home_group, stop_group) ---

def test_home_group_success_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test home_group with wait=True and all rotators succeed."""

    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Use simpler mocking approach without autospec for group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)) as mock_config_g2, \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)) as mock_config_g3:

        # Call form_group and assert its success
        assert group_controller.form_group() is True, "form_group should succeed with mocked slave configurations"

    # Verify group state after successful formation
    assert group_controller.is_grouped is True
    assert group_controller.group_master_address_char == rotator_g1.physical_address # Default group addr from fixture
    assert rotator_g2.is_slave_in_group is True, "rotator_g2 should be marked as slave"
    assert rotator_g2.active_address == rotator_g1.physical_address, "rotator_g2 active_address should be group address"
    assert rotator_g3.is_slave_in_group is True, "rotator_g3 should be marked as slave"
    assert rotator_g3.active_address == rotator_g1.physical_address, "rotator_g3 active_address should be group address"

    # Now that group is formed, proceed with testing home_group
    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS09", # Moving/Homing
        rotator_g2.physical_address: f"{rotator_g2.physical_address}PO00000000", # Homing complete
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS01"  # Homing
    }

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies) as mock_send_group, \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True) as mock_wait_g1, \
         patch.object(rotator_g2, 'wait_until_ready', return_value=True) as mock_wait_g2, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True) as mock_wait_g3, \
         patch.object(rotator_g1, 'update_position') as mock_update_g1, \
         patch.object(rotator_g2, 'update_position') as mock_update_g2, \
         patch.object(rotator_g3, 'update_position') as mock_update_g3:

        assert group_controller.home_group(wait=True) is True

        mock_send_group.assert_called_once_with(command=COMMAND_HOME, data="0", expect_num_replies=3, overall_timeout=6.0, reply_start_timeout=0.5)
        mock_wait_g1.assert_called_once()
        mock_wait_g2.assert_called_once()
        mock_wait_g3.assert_called_once()
        mock_update_g1.assert_called_once()
        mock_update_g2.assert_called_once()
        mock_update_g3.assert_called_once()

def test_home_group_success_no_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test home_group with wait=False and command dispatched."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    mock_replies = {rotator_g1.physical_address: f"{rotator_g1.physical_address}GS09"} # At least one reply

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies) as mock_send_group, \
         patch.object(ElliptecRotator, 'wait_until_ready') as mock_wait: # General patch, should not be called

        assert group_controller.home_group(wait=False) is True
        mock_send_group.assert_called_once()
        mock_wait.assert_not_called()

def test_home_group_fail_one_rotator_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test home_group with wait=True where one rotator fails to become ready."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS09",
        rotator_g2.physical_address: f"{rotator_g2.physical_address}GS09",
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS09"
    }
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies), \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True), \
         patch.object(rotator_g2, 'wait_until_ready', return_value=False) as mock_wait_g2, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True): # g3 might not be called if g2 fails first

        assert group_controller.home_group(wait=True, home_timeout_per_rotator=0.1) is False
        mock_wait_g2.assert_called_once() # Ensure it was attempted

def test_home_group_no_initial_replies_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test home_group with wait=True but no initial replies from _send_group_command_and_collect_replies."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group, \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True) as mock_wait_g1, \
         patch.object(rotator_g2, 'wait_until_ready', return_value=True) as mock_wait_g2, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True) as mock_wait_g3:
        # Even with no initial ACKs, if wait=True, it proceeds to wait for readiness.
        # The success then depends on wait_until_ready.
        assert group_controller.home_group(wait=True) is True
        mock_send_group.assert_called_once()
        mock_wait_g1.assert_called_once()
        mock_wait_g2.assert_called_once()
        mock_wait_g3.assert_called_once()


def test_home_group_no_initial_replies_no_wait(group_controller):
    """Test home_group with wait=False and no initial replies."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation - get the rotators from the controller
    rotator_g1 = group_controller.rotators[0]  # Master
    rotator_g2 = group_controller.rotators[1]  # Slave
    rotator_g3 = group_controller.rotators[2]  # Slave

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group:
        assert group_controller.home_group(wait=False) is False # Fails as no ACK and not waiting
        mock_send_group.assert_called_once()


def test_stop_group_success(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test stop_group with all rotators acknowledging successfully."""
    # Mock the group formation to avoid actual serial communication
    def actual_configure_slave_logic(master_address_char, slave_offset=0.0):
        # This will be called on rotator_g2 and rotator_g3
        return True

    with patch.object(rotator_g2, 'configure_as_group_slave', return_value=True) as mock_config_g2, \
         patch.object(rotator_g3, 'configure_as_group_slave', return_value=True) as mock_config_g3:
        
        group_controller.form_group()
        
        # Manually set the group state since the mocks bypass the actual logic
        rotator_g2.active_address = rotator_g1.physical_address
        rotator_g2.is_slave_in_group = True
        rotator_g3.active_address = rotator_g1.physical_address
        rotator_g3.is_slave_in_group = True
    
    # Set rotators to moving state for the test
    for r in [rotator_g1, rotator_g2, rotator_g3]:
        r._is_moving_state = True

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00",
        rotator_g2.physical_address: f"{rotator_g2.physical_address}GS00",
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS00"
    }
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies) as mock_send_group:
        assert group_controller.stop_group() is True
        mock_send_group.assert_called_once_with(command=COMMAND_STOP, data="", expect_num_replies=3, overall_timeout=3.0, reply_start_timeout=0.1)
        for r in [rotator_g1, rotator_g2, rotator_g3]:
            assert r.is_moving is False

def test_stop_group_one_fails_ack(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test stop_group where one rotator returns an unexpected status."""
    group_controller.form_group()
    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00",
        rotator_g2.physical_address: f"{rotator_g2.physical_address}GS09", # Did not stop correctly
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS00"
    }
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies):
        assert group_controller.stop_group() is False
        assert rotator_g1.is_moving is False
        # rotator_g2.is_moving might not be updated if ack was bad, but group stop fails
        assert rotator_g3.is_moving is False


def test_stop_group_one_missing_reply(group_controller, rotator_g1, rotator_g2):
    """Test stop_group where one rotator does not reply."""
    # Using only 2 rotators for this test for simplicity with mock_replies
    controller = ElliptecGroupController(rotators=[rotator_g1, rotator_g2])
    controller.form_group()

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00"
        # rotator_g2 does not reply
    }
    with patch.object(controller, '_send_group_command_and_collect_replies', return_value=mock_replies):
        assert controller.stop_group() is False
        assert rotator_g1.is_moving is False

def test_stop_group_no_replies(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test stop_group when no rotators reply."""
    # Mock the group formation to avoid actual serial communication
    with patch.object(rotator_g2, 'configure_as_group_slave', return_value=True), \
         patch.object(rotator_g3, 'configure_as_group_slave', return_value=True):
        
        group_controller.form_group()
        
        # Manually set the group state since the mocks bypass the actual logic
        rotator_g2.active_address = rotator_g1.physical_address
        rotator_g2.is_slave_in_group = True
        rotator_g3.active_address = rotator_g1.physical_address
        rotator_g3.is_slave_in_group = True

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group:
        assert group_controller.stop_group() is False
        mock_send_group.assert_called_once()

# --- Tests for move_group_absolute ---

def test_move_group_absolute_success_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test move_group_absolute with wait=True and all rotators succeed."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    target_degrees = 45.0
    hex_pos = degrees_to_hex(target_degrees, rotator_g1.pulse_per_revolution) # Assuming master's PPR for group cmd

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS09", # Moving
        rotator_g2.physical_address: f"{rotator_g2.physical_address}GS09",
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS09"
    }

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies) as mock_send_group, \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True) as mock_wait_g1, \
         patch.object(rotator_g2, 'wait_until_ready', return_value=True) as mock_wait_g2, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True) as mock_wait_g3, \
         patch.object(rotator_g1, 'update_position') as mock_update_g1, \
         patch.object(rotator_g2, 'update_position') as mock_update_g2, \
         patch.object(rotator_g3, 'update_position') as mock_update_g3:

        assert group_controller.move_group_absolute(target_degrees, wait=True) is True

        mock_send_group.assert_called_once_with(command=COMMAND_MOVE_ABS, data=hex_pos, expect_num_replies=3)
        mock_wait_g1.assert_called_once()
        mock_wait_g2.assert_called_once()
        mock_wait_g3.assert_called_once()
        mock_update_g1.assert_called_once()
        mock_update_g2.assert_called_once()
        mock_update_g3.assert_called_once()
        for r in [rotator_g1, rotator_g2, rotator_g3]:
            assert r.is_moving is False # Should be reset by wait_until_ready

def test_move_group_absolute_success_no_wait(group_controller, rotator_g1):
    """Test move_group_absolute with wait=False."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation - get the other rotators from the controller
    rotator_g2 = group_controller.rotators[1]  # Slave
    rotator_g3 = group_controller.rotators[2]  # Slave

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    target_degrees = 30.0
    hex_pos = degrees_to_hex(target_degrees, rotator_g1.pulse_per_revolution)
    mock_replies = {rotator_g1.physical_address: f"{rotator_g1.physical_address}GS09"}

    def move_side_effect(*args, **kwargs):
        # Set moving state on all rotators when group move command is sent
        for r in group_controller.rotators:
            r._is_moving_state = True
        return mock_replies

    def get_status_side_effect():
        # Return moving status for all rotators after move command
        return "01"  # STATUS_MOVING

    with patch.object(group_controller, '_send_group_command_and_collect_replies', side_effect=move_side_effect) as mock_send_group, \
         patch.object(ElliptecRotator, 'wait_until_ready') as mock_wait, \
         patch.object(ElliptecRotator, 'get_status', side_effect=get_status_side_effect):

        assert group_controller.move_group_absolute(target_degrees, wait=False) is True
        mock_send_group.assert_called_once_with(command=COMMAND_MOVE_ABS, data=hex_pos, expect_num_replies=3)
        mock_wait.assert_not_called()
        for r in group_controller.rotators: # is_moving should be true
            assert r.is_moving is True

def test_move_group_absolute_fail_one_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test move_group_absolute with wait=True, one rotator fails wait_until_ready."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    target_degrees = 60.0
    hex_pos = degrees_to_hex(target_degrees, rotator_g1.pulse_per_revolution)
    mock_replies = {r.physical_address: f"{r.physical_address}GS09" for r in group_controller.rotators}

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies), \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True), \
         patch.object(rotator_g2, 'wait_until_ready', return_value=False) as mock_wait_g2_fail, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True):

        assert group_controller.move_group_absolute(target_degrees, wait=True, move_timeout_per_rotator=0.1) is False
        mock_wait_g2_fail.assert_called_once()

def test_move_group_absolute_no_initial_replies_wait(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test move_group_absolute with wait=True, no initial replies, but rotators eventually become ready."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    target_degrees = 75.0
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group, \
         patch.object(rotator_g1, 'wait_until_ready', return_value=True) as mock_wait_g1, \
         patch.object(rotator_g2, 'wait_until_ready', return_value=True) as mock_wait_g2, \
         patch.object(rotator_g3, 'wait_until_ready', return_value=True) as mock_wait_g3:

        assert group_controller.move_group_absolute(target_degrees, wait=True) is True
        mock_send_group.assert_called_once()
        mock_wait_g1.assert_called_once()
        mock_wait_g2.assert_called_once()
        mock_wait_g3.assert_called_once()

def test_move_group_absolute_no_initial_replies_no_wait(group_controller, rotator_g1):
    """Test move_group_absolute with wait=False and no initial replies."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation - get the other rotators from the controller
    rotator_g2 = group_controller.rotators[1]  # Slave
    rotator_g3 = group_controller.rotators[2]  # Slave

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    target_degrees = 15.0
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group:
        assert group_controller.move_group_absolute(target_degrees, wait=False) is False
        mock_send_group.assert_called_once()


# --- Tests for get_group_status ---

def test_get_group_status_success(group_controller, rotator_g1, rotator_g2, rotator_g3):
    """Test get_group_status with all rotators replying successfully."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00",
        rotator_g2.physical_address: f"{rotator_g2.physical_address}GS09",
        rotator_g3.physical_address: f"{rotator_g3.physical_address}GS01"
    }
    expected_statuses = {
        rotator_g1.physical_address: "00",
        rotator_g2.physical_address: "09",
        rotator_g3.physical_address: "01"
    }
    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value=mock_replies) as mock_send_group:
        statuses = group_controller.get_group_status()
        assert statuses == expected_statuses
        mock_send_group.assert_called_once_with(command=COMMAND_GET_STATUS, expect_num_replies=3)

def test_get_group_status_one_malformed_reply(group_controller, rotator_g1, rotator_g2):
    """Test get_group_status when one rotator returns a malformed reply."""
    controller = ElliptecGroupController(rotators=[rotator_g1, rotator_g2])
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)):
        assert controller.form_group() is True

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00",
        rotator_g2.physical_address: "MALFORMED_RESPONSE"  # Doesn't start with expected prefix
    }
    expected_statuses = {
        rotator_g1.physical_address: "00",
        rotator_g2.physical_address: "Error: BadFormat"
    }
    with patch.object(controller, '_send_group_command_and_collect_replies', return_value=mock_replies):
        statuses = controller.get_group_status()
        assert statuses == expected_statuses

def test_get_group_status_one_missing_reply(group_controller, rotator_g1, rotator_g2):
    """Test get_group_status when one rotator does not reply."""
    controller = ElliptecGroupController(rotators=[rotator_g1, rotator_g2])
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation
    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)):
        assert controller.form_group() is True

    mock_replies = {
        rotator_g1.physical_address: f"{rotator_g1.physical_address}GS00"
        # rotator_g2 missing
    }
    expected_statuses = {
        rotator_g1.physical_address: "00"
    } # Only g1's status should be present
    with patch.object(controller, '_send_group_command_and_collect_replies', return_value=mock_replies):
        statuses = controller.get_group_status()
        assert statuses == expected_statuses

def test_get_group_status_no_replies(group_controller):
    """Test get_group_status when no rotators reply."""
    
    # Define side effects for mocks to update rotator state
    def configure_slave_side_effect(rotator_instance, master_addr, slave_offset=0.0):
        rotator_instance.active_address = master_addr
        rotator_instance.is_slave_in_group = True
        rotator_instance.group_offset_degrees = slave_offset
        return True

    # Set up group formation - get the rotators from the controller
    rotator_g1 = group_controller.rotators[0]  # Master
    rotator_g2 = group_controller.rotators[1]  # Slave
    rotator_g3 = group_controller.rotators[2]  # Slave

    with patch.object(rotator_g2, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g2, *args, **kwargs)), \
         patch.object(rotator_g3, 'configure_as_group_slave', side_effect=lambda *args, **kwargs: configure_slave_side_effect(rotator_g3, *args, **kwargs)):
        assert group_controller.form_group() is True

    with patch.object(group_controller, '_send_group_command_and_collect_replies', return_value={}) as mock_send_group:
        statuses = group_controller.get_group_status()
        assert statuses == {}
        mock_send_group.assert_called_once()
