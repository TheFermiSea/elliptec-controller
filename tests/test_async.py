#!/usr/bin/env python3
"""
Test module for asynchronous functionality of ElliptecRotator.
"""

import pytest
import time
import threading
import queue
from unittest.mock import MagicMock, patch

from elliptec_controller import ElliptecRotator, ElliptecError


class MockSerial:
    """Mock serial port for testing."""

    def __init__(self):
        self.is_open = True
        self.in_waiting = 0
        self._log = []
        self._responses = queue.Queue()
        self._write_lock = threading.Lock()
        self._next_response_delay = 0

    def write(self, data):
        with self._write_lock:
            self._log.append(data)
            cmd = data.decode('ascii').strip('\r')
            
            # Add a default response if none is queued
            if self._responses.empty():
                if cmd.endswith('gs'):  # Status command
                    self._responses.put(f"{cmd[0]}GS00\r\n".encode('ascii'))
                elif cmd.endswith('gp'):  # Position command
                    self._responses.put(f"{cmd[0]}PO00000000\r\n".encode('ascii'))
                elif cmd.endswith('ma'):  # Move absolute command
                    self._responses.put(f"{cmd[0]}GS00\r\n".encode('ascii'))
                elif cmd.endswith('ho'):  # Home command
                    self._responses.put(f"{cmd[0]}GS00\r\n".encode('ascii'))
                else:
                    self._responses.put(f"{cmd[0]}GS00\r\n".encode('ascii'))

            # Set in_waiting to indicate response is available after a small delay
            if self._next_response_delay > 0:
                time.sleep(self._next_response_delay)
            self.in_waiting = 1
        return len(data)

    def read(self, size=1):
        with self._write_lock:
            if not self._responses.empty():
                response = self._responses.get()
                self.in_waiting = 0
                return response
            self.in_waiting = 0
            return b''

    def queue_response(self, response):
        if isinstance(response, str):
            response = response.encode('ascii')
        self._responses.put(response)

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def set_response_delay(self, delay):
        """Set delay before response is available."""
        self._next_response_delay = delay


@pytest.fixture
def mock_serial():
    """Fixture to provide a mock serial port."""
    return MockSerial()


@pytest.fixture
def rotator(mock_serial):
    """Fixture to provide an ElliptecRotator with a mock serial port."""
    with patch('serial.Serial', return_value=mock_serial):
        rotator = ElliptecRotator(
            port="/dev/mock_port",
            motor_address=1,
            name="TestRotator",
            auto_home=False
        )
        yield rotator


class TestAsyncFunctionality:
    """Test asynchronous functionality of ElliptecRotator."""

    def test_connect_disconnect(self, rotator):
        """Test that connect() and disconnect() work properly."""
        assert rotator._serial_thread is None
        assert rotator._is_connected is False
        
        rotator.connect()
        assert rotator._serial_thread is not None
        assert rotator._serial_thread.is_alive()
        assert rotator._is_connected is True
        assert rotator._use_async is True
        
        rotator.disconnect()
        time.sleep(0.1)  # Allow thread to terminate
        assert rotator._is_connected is False
        assert rotator._use_async is False

    def test_context_manager(self, rotator):
        """Test that the context manager properly manages the thread."""
        assert rotator._serial_thread is None
        
        with rotator:
            assert rotator._serial_thread is not None
            assert rotator._serial_thread.is_alive()
            assert rotator._is_connected is True
            
        time.sleep(0.1)  # Allow thread to terminate
        assert rotator._is_connected is False

    def test_send_command_async(self, rotator, mock_serial):
        """Test sending commands in async mode."""
        # Queue a specific response
        mock_serial.queue_response("1GS00\r\n")
        
        rotator.connect()
        response = rotator.send_command("gs", use_async=True)
        
        assert response == "1GS00"
        assert len(mock_serial._log) > 0
        assert b'1gs' in mock_serial._log[0]
        
        rotator.disconnect()

    def test_send_command_sync_vs_async(self, rotator, mock_serial):
        """Test that sync and async commands can be mixed."""
        mock_serial.queue_response("1GS00\r\n")
        mock_serial.queue_response("1PO00000000\r\n")
        
        # First in sync mode
        sync_response = rotator.send_command("gs", use_async=False)
        assert sync_response == "1GS00"
        
        # Then in async mode
        rotator.connect()
        async_response = rotator.send_command("gp", use_async=True)
        assert async_response == "1PO00000000"
        
        rotator.disconnect()

    def test_async_timeout(self, rotator, mock_serial):
        """Test that async commands properly handle timeouts."""
        rotator.connect()
        
        # Set a delay longer than the command timeout
        mock_serial.set_response_delay(0.2)
        
        # Send with short timeout
        response = rotator._send_command_async("gs", timeout=0.1)
        
        assert response == ""  # Empty response indicates timeout
        
        rotator.disconnect()

    def test_move_with_async(self, rotator, mock_serial):
        """Test move_absolute in async mode."""
        mock_serial.queue_response("1GS00\r\n")
        
        rotator.connect()
        result = rotator.move_absolute(45.0, wait=False)
        
        assert result is True
        assert any(b'1ma' in log for log in mock_serial._log)
        
        rotator.disconnect()

    def test_multiple_async_commands(self, rotator, mock_serial):
        """Test sending multiple async commands in sequence."""
        for _ in range(5):
            mock_serial.queue_response("1GS00\r\n")
        
        rotator.connect()
        
        responses = []
        for _ in range(5):
            response = rotator.send_command("gs", use_async=True)
            responses.append(response)
            
        assert all(response == "1GS00" for response in responses)
        assert len(mock_serial._log) >= 5
        
        rotator.disconnect()

    def test_error_handling_in_async_mode(self, rotator, mock_serial):
        """Test error handling in async mode."""
        # Configure mock to close after first command to simulate error
        old_write = mock_serial.write
        
        def write_then_close(data):
            result = old_write(data)
            mock_serial.is_open = False
            return result
            
        mock_serial.write = write_then_close
        
        rotator.connect()
        
        # This should be handled gracefully
        response = rotator.send_command("gs", use_async=True)
        assert response == ""  # Empty response due to error
        
        # Make sure disconnect doesn't raise exceptions
        rotator.disconnect()

    def test_stop_event(self, rotator):
        """Test that the stop event properly signals the worker thread to exit."""
        rotator.connect()
        
        assert rotator._stop_event.is_set() is False
        
        rotator.disconnect()
        
        assert rotator._stop_event.is_set() is True
        time.sleep(0.1)  # Allow thread to terminate
        assert rotator._is_connected is False

    def test_command_queue_usage(self, rotator, mock_serial):
        """Test that commands are properly queued."""
        rotator.connect()
        
        # Send a command but interrupt before it processes
        rotator._command_queue.put((123, "1gs", queue.Queue()))
        
        # Verify the queue is not empty
        assert rotator._command_queue.empty() is False
        
        # Wait for queue to process
        time.sleep(0.2)
        
        # Queue should be empty after processing
        assert rotator._command_queue.empty() is True
        
        rotator.disconnect()

    def test_thread_shutdown_on_exception(self, rotator, mock_serial):
        """Test that the worker thread shuts down properly on exceptions."""
        # Make the mock serial object raise an exception on read
        mock_serial.read = MagicMock(side_effect=Exception("Simulated read error"))
        
        rotator.connect()
        time.sleep(0.1)  # Allow thread to start
        
        # Send a command that will trigger the read exception
        rotator.send_command("gs", use_async=True)
        
        # Thread should handle the exception and disconnect
        time.sleep(0.2)
        
        # The thread might still be alive but connection status should be False
        assert rotator._is_connected is False
        
        # Clean up
        rotator.disconnect()

    def test_default_mode_after_connect(self, rotator):
        """Test that connect() sets use_async to True as default mode."""
        assert rotator._use_async is False
        
        rotator.connect()
        assert rotator._use_async is True
        
        rotator.disconnect()
        assert rotator._use_async is False


if __name__ == "__main__":
    pytest.main(["-xvs", __file__])