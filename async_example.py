import serial
import time
import threading
import queue
import logging
import re
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

class ElliptecError(Exception):
    """Custom exception for Elliptec controller errors."""
    pass

class ElliptecController:
    """
    Controller for Elliptec devices via serial port, using threading
    for non-blocking communication.
    """

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._serial_connection: Optional[serial.Serial] = None
        self._serial_thread: Optional[threading.Thread] = None
        self._command_queue: queue.Queue = queue.Queue()
        self._response_queue: queue.Queue = queue.Queue()
        self._stop_event: threading.Event = threading.Event()
        self._is_connected = False

        self._current_position = 0.0
        self._is_moving = False
        self._is_ready = False
        self._units = "DEG"
        self._speed = 50

    def _serial_thread_worker(self):
        """Worker function for the serial communication thread."""
        logger.info("Serial communication thread started.")
        try:
            self._serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"Serial connection established on {self.port}")
            self._is_connected = True
            self._is_ready = True

            while not self._stop_event.is_set():
                try:
                    # Get a command from the queue with a timeout
                    command_id, command = self._command_queue.get(timeout=0.1)
                    logger.debug(f"Thread sending command ({command_id}): {command}")

                    command_with_terminator = f"{command}\r"
                    self._serial_connection.write(command_with_terminator.encode('ascii'))
                    time.sleep(0.1) # Small delay for device to process

                    # Read response (blocking, but handled by the thread)
                    # Read multiple lines until timeout or device indicates end
                    try:
                        response = self._serial_connection.readline().decode('ascii').strip()
                        if response:
                            logger.debug(f"Thread received response ({command_id}): {response}")
                            self._response_queue.put((command_id, response))

                        # Read subsequent lines if available (e.g., status updates)
                        while self._serial_connection.in_waiting > 0:
                            status_line = self._serial_connection.readline().decode('ascii').strip()
                            if status_line:
                                logger.debug(f"Thread received status update: {status_line}")
                                # Optionally put status updates on a separate queue or update internal state directly
                                self._process_status_line(status_line)

                    except serial.SerialTimeoutException:
                        logger.debug(f"Thread timed out waiting for response to command ({command_id})")
                        self._response_queue.put((command_id, "TIMEOUT")) # Indicate timeout
                    except Exception as e:
                        logger.error(f"Thread error reading response for command ({command_id}): {e}")
                        self._response_queue.put((command_id, f"ERROR: {e}")) # Indicate error

                    self._command_queue.task_done()

                except queue.Empty:
                    # No commands in the queue, check for status updates
                    try:
                        while self._serial_connection.in_waiting > 0:
                            status_line = self._serial_connection.readline().decode('ascii').strip()
                            if status_line:
                                logger.debug(f"Thread received status update: {status_line}")
                                self._process_status_line(status_line)
                    except Exception as e:
                        logger.error(f"Thread error reading status updates: {e}")
                    time.sleep(0.05) # Small sleep to avoid busy-waiting

                except Exception as e:
                    logger.error(f"Unexpected error in serial thread worker: {e}")
                    time.sleep(0.1) # Avoid tight loop on error

        except serial.SerialException as e:
            logger.error(f"Serial connection failed: {e}")
            self._is_connected = False
            self._is_ready = False
        except Exception as e:
            logger.error(f"Unexpected error in serial thread setup: {e}")
            self._is_connected = False
            self._is_ready = False
        finally:
            if self._serial_connection and self._serial_connection.is_open:
                self._serial_connection.close()
                logger.info("Serial connection closed.")
            self._serial_connection = None
            self._is_connected = False
            self._is_ready = False
            logger.info("Serial communication thread stopped.")

    def _process_status_line(self, line: str):
        """Processes a status line received from the device."""
        match_pos = re.match(r"POS\s*(-?\d+\.\d+)", line)
        if match_pos:
            try:
                self._current_position = float(match_pos.group(1))
            except ValueError:
                logger.warning(f"Could not parse position from status line: {line}")

        match_status = re.match(r"STATUS\s+(\w+)", line)
        if match_status:
            status = match_status.group(1).upper()
            self._is_moving = (status == "MOVING")
            self._is_ready = (status == "READY")

        # Add other status parsing if needed (e.g., speed, units)

    def _send_command_async(self, command: str, wait_for_response: bool = False, response_timeout: float = 1.0) -> Optional[str]:
        """Sends a command to the serial thread and optionally waits for a response."""
        if not self._is_connected:
            raise ElliptecError("Device not connected.")

        command_id = time.time() # Use timestamp as simple command ID
        self._command_queue.put((command_id, command))

        if wait_for_response:
            start_time = time.time()
            while (time.time() - start_time) < response_timeout:
                try:
                    resp_id, response = self._response_queue.get(timeout=0.1)
                    if resp_id == command_id:
                        self._response_queue.task_done()
                        return response
                    else:
                        # Put other responses back
                        self._response_queue.put((resp_id, response))
                except queue.Empty:
                    time.sleep(0.05) # Wait briefly
                except Exception as e:
                    logger.error(f"Error waiting for response to command ({command_id}): {e}")
                    raise ElliptecError(f"Error waiting for response: {e}")

            raise ElliptecError(f"Timeout waiting for response to command: {command}")
        return None

    def connect(self):
        """Starts the serial communication thread."""
        if self._serial_thread and self._serial_thread.is_alive():
            logger.warning("Serial thread is already running.")
            return

        self._stop_event.clear()
        self._serial_thread = threading.Thread(target=self._serial_thread_worker, daemon=True)
        self._serial_thread.start()

        # Wait for the thread to establish connection and initialize
        start_time = time.time()
        timeout = 5.0 # Timeout for connection attempt
        while not self._is_connected and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if not self._is_connected:
            raise ElliptecError("Failed to establish serial connection within timeout.")

        # Send initial configuration after successful connection
        try:
            self.set_units(self.units)
            self.set_speed(self.speed)
            self.get_status() # Get initial status
        except Exception as e:
            logger.warning(f"Initial configuration failed after connection: {e}")
            # Don't raise here, just log and allow further commands

    def disconnect(self):
        """Stops the serial communication thread."""
        if self._serial_thread and self._serial_thread.is_alive():
            self._stop_event.set()
            try:
                self._serial_thread.join(timeout=2.0) # Wait for thread to finish
                if self._serial_thread.is_alive():
                    logger.warning("Serial thread did not shut down cleanly.")
            except Exception as e:
                logger.error(f"Error joining serial thread: {e}")
            self._serial_thread = None
        self._is_connected = False
        self._is_ready = False

    def get_position(self) -> float:
        """Gets the current position of the mount."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        response = self._send_command_async("GET POS", wait_for_response=True)
        if response:
            match = re.match(r"POS\s*(-?\d+\.\d+)", response)
            if match:
                try:
                    self._current_position = float(match.group(1))
                    return self._current_position
                except ValueError:
                    raise ElliptecError(f"Failed to parse position from response: {response}")
            else:
                raise ElliptecError(f"Unexpected response format for GET POS: {response}")
        raise ElliptecError("No response received from device for GET POS.")

    def get_status(self) -> str:
        """Gets the current status of the mount."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        response = self._send_command_async("STATUS", wait_for_response=True)
        if response:
            match = re.match(r"STATUS\s+(\w+)", response)
            if match:
                status = match.group(1).upper()
                self._is_moving = (status == "MOVING")
                self._is_ready = (status == "READY")
                return status
            else:
                raise ElliptecError(f"Unexpected response format for STATUS: {response}")
        raise ElliptecError("No response received from device for STATUS.")

    def set_units(self, units: str):
        """Sets the units for position reporting and movement (DEG or RAD)."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        if units.upper() not in ["DEG", "RAD"]:
            raise ValueError(f"Invalid units: {units}. Use 'DEG' or 'RAD'.")
        self._send_command_async(f"UNITS {units.upper()}", wait_for_response=True)
        self._units = units.upper()
        logger.info(f"Units set to {self._units}")

    def set_speed(self, speed: int):
        """Sets the movement speed (1-100)."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        if not 1 <= speed <= 100:
            raise ValueError(f"Invalid speed: {speed}. Must be between 1 and 100.")
        self._send_command_async(f"SPEED {speed}", wait_for_response=True)
        self._speed = speed
        logger.info(f"Speed set to {self._speed}")

    def move_absolute(self, angle: float):
        """Moves the mount to an absolute angle."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        self._send_command_async(f"SET POS {angle}", wait_for_response=True)
        self._is_moving = True
        logger.info(f"Moving to absolute position: {angle} {self._units}")

    def move_relative(self, angle: float):
        """Moves the mount by a relative angle from the current position."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        self._send_command_async(f"MOVE {angle}", wait_for_response=True)
        self._is_moving = True
        logger.info(f"Moving relatively by: {angle} {self._units}")

    def move_home(self):
        """Moves the mount to the home position."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        self._send_command_async("HOME", wait_for_response=True)
        self._is_moving = True
        logger.info("Moving to home position.")

    def wait_until_ready(self):
        """Waits until the current motion is complete."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        logger.debug("Waiting for motion to complete...")
        start_time = time.time()
        timeout = 60.0 # Example timeout in seconds
        while self._is_moving and (time.time() - start_time) < timeout:
            time.sleep(0.1) # Check status periodically
            self.get_status() # Update status
        if self._is_moving:
            logger.warning("Timeout waiting for motion to complete.")
            raise ElliptecError("Timeout waiting for device to become ready.")
        logger.debug("Motion complete.")

    def is_moving(self) -> bool:
        """Checks if the mount is currently moving."""
        if not self._is_ready:
            raise ElliptecError("Device not ready or connected.")
        self.get_status() # Ensure status is updated
        return self._is_moving

    def get_properties(self) -> Dict[str, Any]:
        """Returns a dictionary of device properties."""
        try:
            self.get_position()
            self.get_status()
            return {
                "position": self._current_position,
                "units": self._units,
                "speed": self._speed,
                "is_moving": self._is_moving,
                "is_ready": self._is_ready,
            }
        except Exception as e:
            raise ElliptecError(f"Failed to get properties: {e}")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
