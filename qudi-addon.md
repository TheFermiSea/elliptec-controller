# Qudi Add-on for Elliptec ELL14 Rotation Mount

This add-on provides a Qudi device driver for controlling the Elliptec ELL14 rotation mount via a serial connection. It uses the `elliptec-controller` library and employs threading for non-blocking communication.

## Features

*   Connects to the Elliptec ELL14 via a serial port.
*   Gets the current position.
*   Sets the absolute position.
*   Moves by a relative angle.
*   Moves to the home position.
*   Sets movement units (degrees or radians).
*   Sets movement speed.
*   Waits for motion to complete.
*   Checks the current device status (moving/ready).
*   Uses threading for non-blocking serial communication, keeping the main application responsive.

## Installation

1.  **Clone the Qudi Add-on Template:**
    ```bash
    git clone https://github.com/Ulm-IQO/qudi-addon-template.git qudi-elliptec-ell14
    cd qudi-elliptec-ell14
    ```

2.  **Clone the Elliptec Controller Library:**
    ```bash
    git clone git@github.com:TheFermiSea/elliptec-controller.git
    ```

3.  **Update `pyproject.toml`:**
    Open the `pyproject.toml` file in the `qudi-elliptec-ell14` directory and modify it as follows:

    ```toml
    [project]
    name = "qudi-elliptec-ell14"
    version = "0.1.0"
    description = "Qudi add-on for controlling the Elliptec ELL14 rotation mount"
    authors = [
        { name = "Your Name", email = "your.email@example.com" },
    ]
    license = { file = "LICENSE" }
    requires-python = ">=3.8"
    dependencies = [
        "qudi>=0.1.0", # Adjust Qudi version as needed
        # Add the elliptec-controller library as a git dependency
        - = "git+https://github.com/TheFermiSea/elliptec-controller.git",
    ]

    [project.entry-points]
    qudi.devices = [
        "elliptec_ell14 = qudi_elliptec_ell14.driver:ElliptecELL14Device",
    ]
    ```

4.  **Modify `driver.py`:**
    Open the `driver.py` file inside the `qudi_elliptec_ell14/qudi_elliptec_ell14/` directory and replace its contents with the following code:

    ```python
    import logging
    import time
    import threading
    import queue
    import re
    from typing import Any, Dict, Optional, Tuple

    # Import the necessary class from the elliptec-controller library
    from elliptec_controller import ElliptecController

    # Import Qudi components
    from quudi.device import Device
    from quudi.core import QudiError

    logger = logging.getLogger(__name__)

    class ElliptecError(Exception):
        """Custom exception for Elliptec controller errors."""
        pass

    class ElliptecELL14Device(Device):
        """
        Qudi device class for controlling the Elliptec ELL14 rotation mount
        using the elliptec-controller library.
        """

        def __init__(self, name: str, config: Dict[str, Any]):
            super().__init__(name, config)

            self.port = config.get("port", "/dev/ttyUSB0")
            self.baudrate = config.get("baudrate", 9600)
            self.timeout = config.get("timeout", 1.0)
            self.units = config.get("units", "DEG") # "DEG" or "RAD"
            self.speed = config.get("speed", 50) # Default speed (1-100)

            self._controller: Optional[ElliptecController] = None
            self._serial_thread: Optional[threading.Thread] = None
            self._command_queue: queue.Queue = queue.Queue()
            self._response_queue: queue.Queue = queue.Queue()
            self._stop_event: threading.Event = threading.Event()
            self._is_connected = False

            self._current_position = 0.0
            self._is_moving = False
            self._is_ready = False

        def _serial_thread_worker(self):
            """Worker function for the serial communication thread."""
            logger.info("Serial communication thread started.")
            try:
                self._controller = ElliptecController(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                self._controller.connect()
                logger.info(f"Serial connection established on {self.port}")
                self._is_connected = True
                self._is_ready = True

                while not self._stop_event.is_set():
                    try:
                        # Get a command from the queue with a timeout
                        command_id, command = self._command_queue.get(timeout=0.1)
                        logger.debug(f"Thread sending command ({command_id}): {command}")

                        command_with_terminator = f"{command}\r"
                        self._controller._serial.write(command_with_terminator.encode('ascii'))
                        time.sleep(0.1) # Small delay for device to process

                        # Read response (blocking, but handled by the thread)
                        try:
                            response = self._controller._serial.readline().decode('ascii').strip()
                            if response:
                                logger.debug(f"Thread received response ({command_id}): {response}")
                                self._response_queue.put((command_id, response))

                            # Read subsequent lines if available (e.g., status updates)
                            while self._controller._serial.in_waiting > 0:
                                status_line = self._controller._serial.readline().decode('ascii').strip()
                                if status_line:
                                    logger.debug(f"Thread received status update: {status_line}")
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
                            while self._controller._serial.in_waiting > 0:
                                status_line = self._controller._serial.readline().decode('ascii').strip()
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
                if self._controller and self._controller._serial and self._controller._serial.is_open:
                    self._controller._serial.close()
                    logger.info("Serial connection closed.")
                self._controller = None
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
                # Consider raising a more specific ElliptecError if needed
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

            # Send initial configuration commands after successful connection
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
'''

5. Update README.md:
Ensure the installation instructions correctly mention the dependency on the elliptec-controller library.
Update the usage example to reflect the actual class name and methods provided by the elliptec-controller library.
6. Install the Add-on:
From the qudi_elliptec-ell14 directory, run:
pip install .
This will install the qudi-elliptec-ell14 package, which will automatically pull in elliptec-controller from the git repository specified in pyproject.toml.
7. Configure Qudi:
In your Qudi configuration file (e.g., config.yaml), define the device using the new driver name:
'''yaml
devices:
  rotation_mount:
    driver: elliptec_ell14
    port: /dev/ttyUSB0  # Replace with your device's serial port (e.g., COM3 on Windows)
    baudrate: 9600
    timeout: 1.0
    # Optional: Override defaults
    units: DEG          # Or RAD
    speed: 50           # 1-100
'''

8. Test:
Run a Qudi script similar to the example provided in the README.md to test the connection and functionality.
This comprehensive guide should help you create a functional Qudi add-on using the elliptec-controller library, leveraging threading for non-blocking communication. Remember to consult the elliptec-controller library's documentation for its exact API and usage details.
